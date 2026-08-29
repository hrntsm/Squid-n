//! ST-Bridge 標準スキーマに沿った断面書き出し。`Section.shape`（[`SectionShape`]）から
//! 標準の断面要素（`StbSecColumn_S` 等）＋形鋼ライブラリ（`StbSecSteel`）を生成する。
//! 材料は ST-Bridge の慣習どおり断面へグレード名で付す（鋼 `strength_main`、RC/SRC/CFT の
//! コンクリート `strength_concrete`）。
//!
//! # 対応形状
//! - 鋼: H形鋼／角形鋼管／鋼管／山形鋼／溝形鋼／T形鋼（`StbSecSteel` 参照）。
//! - RC: 矩形・円形（幾何＋配筋。配筋は `StbSecBarArrangement*` として書き出す）。
//! - CFT: 角形・円形（充填鋼管を `StbSecColumn_CFT`＋`StbSecSteel` 参照で。柱のみ）。
//! - SRC: 矩形（`StbSecColumn_SRC`/`StbSecBeam_SRC`。コンクリート図形＋内蔵鉄骨＋配筋＋鋼種）。
//! - 上記以外（耐震壁・形状未定義・CFT 梁・RC 円形梁）は、標準 ST-Bridge に対応要素がないため
//!   物性直持ちの拡張要素 `StbSecRaw` へフォールバックする（他ソフトは解釈できないが
//!   参照部材の断面リンクは保つ。完全一致の保存は `.scz`）。
//!
//! # 柱／梁の型分けと id 再割当て
//! ST-Bridge では断面が柱用（`StbSecColumn_*`）と梁用（`StbSecBeam_*`）に型分けされ、
//! 部材はその断面 id を参照する。内部モデルは 1 断面を柱・梁で共有し得るため、
//! 共有断面は柱用・梁用の 2 要素へ分割し、梁用へ新しい id を割り当てる。
//! 呼び出し側（[`super::export`]）は返り値の id マップで部材の `id_section` を張り替える。
//! id は ST-Bridge の `positiveInteger`（1 始まり）に合わせ、内部 0 始まり id に +1 する。

use super::export::{esc, fmt as num};
use squid_n_core::model::{ElementKind, Model, Section};
use squid_n_core::section_shape::{RcRebar, SectionShape};
use std::collections::HashMap;

/// ST-Bridge の断面 id は `positiveInteger`（1 以上）。内部 0 始まり id に +1 する。
/// 部材側の断面参照（`export::sec_ref`）も同じく +1 するため一貫する。
fn sid(id: u32) -> u32 {
    id + 1
}

/// 断面の階を `floor` 属性へ整形する（階を持たない断面は属性ごと省く）。
///
/// ST-Bridge 2.0 の `floor` は省略可能な `xs:string` なので、未設定を空文字列で
/// 書き出すと取り込み側で「階なし」と「階が空文字列」を区別できなくなる。
/// 属性を出さないことで往復しても同一性キー（符号＋階）が保たれる。
fn floor_attr(sec: &Section) -> String {
    match &sec.floor {
        Some(f) => format!(" floor=\"{}\"", esc(f)),
        None => String::new(),
    }
}

/// 標準モードで生成した断面ブロックと、部材参照の張り替え用 id マップ。
pub(super) struct StandardSections {
    /// 断面要素群（柱・梁・ブレース。`StbSections` のスキーマ順に整列済み。形鋼ライブラリは含まない）。
    pub sections_xml: String,
    /// 形鋼ライブラリ `<StbSecSteel>`（スキーマ順ではスラブ・壁断面の後に置く）。
    pub steel_lib: String,
    /// 内部断面 id → 柱部材が参照すべき ST-Bridge 断面 id。
    pub col_map: HashMap<u32, u32>,
    /// 内部断面 id → 梁部材が参照すべき ST-Bridge 断面 id。
    pub beam_map: HashMap<u32, u32>,
}

/// 各断面が柱／梁のどちらに使われているかを集計する。
/// 返り値は 内部断面 id → (柱で使用, 梁で使用)。
fn section_roles(model: &Model) -> HashMap<u32, (bool, bool)> {
    let mut roles: HashMap<u32, (bool, bool)> = HashMap::new();
    for e in &model.elements {
        if e.nodes.len() != 2 {
            continue;
        }
        // 梁は幾何で柱/梁を判定、ブレースは梁役割（水平材の断面型）として扱う。
        let is_col = match e.kind {
            ElementKind::Beam => {
                let n0 = &model.nodes[e.nodes[0].index()];
                let n1 = &model.nodes[e.nodes[1].index()];
                // 全クレート共通の 45° 余弦基準で柱/梁を分ける。
                squid_n_core::geom::is_vertical_axis(n0.coord, n1.coord)
            }
            ElementKind::Brace { .. } => false,
            _ => continue,
        };
        let Some(sec) = e.section else { continue };
        let ent = roles.entry(sec.0).or_insert((false, false));
        if is_col {
            ent.0 = true;
        } else {
            ent.1 = true;
        }
    }
    roles
}

/// 断面が持つ鉄筋・内蔵鉄骨の材質名（ST-Bridge はグレード名で書く）。
///
/// ST-Bridge も Squid-n も材料を断面が持つため、部材からの逆算は行わない。
#[derive(Default, Clone, Copy)]
struct BarGrades<'a> {
    /// 主筋（`strength_main`）。
    main: Option<&'a str>,
    /// せん断補強筋（`strength_band` ほか）。
    shear: Option<&'a str>,
    /// SRC の内蔵鉄骨（`strength_steel`）。
    steel: Option<&'a str>,
}

/// 断面の鉄筋・内蔵鉄骨の材質名を引く。
fn bar_grades<'a>(model: &'a Model, sec: &Section) -> BarGrades<'a> {
    let name = |mid: Option<squid_n_core::ids::MaterialId>| -> Option<&'a str> {
        model.materials.get(mid?.index()).map(|m| m.name.as_str())
    };
    BarGrades {
        main: name(sec.rebar_material),
        shear: name(sec.shear_rebar_material),
        steel: name(sec.steel_material),
    }
}

/// 形鋼ライブラリ（`StbSecSteel`）。図形名で重複排除しつつ挿入順を保つ。
#[derive(Default)]
struct SteelLibrary {
    names: std::collections::HashSet<String>,
    entries: Vec<String>,
}

impl SteelLibrary {
    fn add(&mut self, name: &str, entry: String) {
        if self.names.insert(name.to_string()) {
            self.entries.push(entry);
        }
    }
    fn render(&self) -> String {
        if self.entries.is_empty() {
            return String::new();
        }
        // StbSecSteel の子要素はスキーマ順（H → BOX → Pipe → T → C → L → LipC →
        // FlatBar → RoundBar）に並べる必要がある。同順位内は挿入順を保つ（安定ソート）。
        let mut ordered: Vec<&String> = self.entries.iter().collect();
        ordered.sort_by_key(|e| steel_rank(e));
        let mut s = String::from("      <StbSecSteel>\n");
        for e in ordered {
            s.push_str("        ");
            s.push_str(e);
            s.push('\n');
        }
        s.push_str("      </StbSecSteel>\n");
        s
    }
}

/// 形鋼ライブラリ要素の `StbSecSteel` スキーマ順の順位（要素名の接頭辞で判定）。
fn steel_rank(entry: &str) -> u8 {
    let tag_rank = [
        ("<StbSecRoll-H", 0u8),
        ("<StbSecBuild-H", 1),
        ("<StbSecRoll-BOX", 2),
        ("<StbSecBuild-BOX", 3),
        ("<StbSecPipe", 4),
        ("<StbSecRoll-T", 5),
        ("<StbSecRoll-C", 6),
        ("<StbSecRoll-L", 7),
        ("<StbSecLipC", 8),
        ("<StbSecFlatBar", 9),
        ("<StbSecRoundBar", 10),
    ];
    for (tag, rank) in tag_rank {
        if entry.starts_with(tag) {
            return rank;
        }
    }
    99
}

/// H 形鋼の形鋼図形名と `StbSecSteel` エントリ（鋼断面・SRC 内蔵鉄骨で共用）。
fn h_figure(height: f64, width: f64, web_thick: f64, flange_thick: f64) -> (String, String) {
    let name = format!(
        "H-{}x{}x{}x{}",
        num(height),
        num(width),
        num(web_thick),
        num(flange_thick)
    );
    // r（フィレット半径）は内部モデルにないが、スキーマ上 length>0 が必須。取り込みでは
    // 無視される（A/B/t1/t2 のみ使用）ため、フランジ厚を便宜値として与える。
    let body = format!(
        "<StbSecRoll-H name=\"{}\" type=\"H\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" r=\"{}\"/>",
        esc(&name),
        num(height),
        num(width),
        num(web_thick),
        num(flange_thick),
        num(flange_thick)
    );
    (name, body)
}

/// 角形鋼管の形鋼図形名と `StbSecSteel` エントリ（鋼断面・CFT 角形で共用）。
///
/// `corner_r` は断面入力の角部外半径 [mm]。`corner_r > 0` ならその値を r 属性に
/// 出力する。`corner_r <= 0`（未入力、または角部半径を持たない CftBox 由来）は
/// ST-Bridge スキーマ上 r（length）に 0 以下を許さないため、従来通り板厚を
/// 便宜値として与える（取り込み側では r 属性は無視されるため実害はない）。
fn box_figure(height: f64, width: f64, thick: f64, corner_r: f64) -> (String, String) {
    // 形鋼ライブラリ（`SteelLibrary::add`）は名前で重複排除するため、名前は形状の
    // 全パラメータから導く。corner_r を含めないと「同寸で角部半径だけ異なる」
    // 2 断面が同一名に潰れ、後着エントリが捨てられて再取り込みで角部半径が
    // 先着の値に化ける。
    let name = if corner_r > 0.0 {
        format!(
            "BOX-{}x{}x{}r{}",
            num(height),
            num(width),
            num(thick),
            num(corner_r)
        )
    } else {
        format!("BOX-{}x{}x{}", num(height), num(width), num(thick))
    };
    // type は BCP/BCR/STKR/ELSE のいずれか（種別を内部で持たないため ELSE）。
    let r = if corner_r > 0.0 { corner_r } else { thick };
    let body = format!(
        "<StbSecRoll-BOX name=\"{}\" type=\"ELSE\" A=\"{}\" B=\"{}\" t=\"{}\" r=\"{}\"/>",
        esc(&name),
        num(height),
        num(width),
        num(thick),
        num(r)
    );
    (name, body)
}

/// 鋼管の形鋼図形名と `StbSecSteel` エントリ（鋼断面・CFT 円形で共用）。
fn pipe_figure(outer_dia: f64, thick: f64) -> (String, String) {
    let name = format!("P-{}x{}", num(outer_dia), num(thick));
    let body = format!(
        "<StbSecPipe name=\"{}\" D=\"{}\" t=\"{}\"/>",
        esc(&name),
        num(outer_dia),
        num(thick)
    );
    (name, body)
}

/// 鋼断面 → 形鋼図形名 と `StbSecSteel` エントリ。対応しない形状は `None`。
fn steel_figure(shape: &SectionShape) -> Option<(String, String)> {
    let e = |name: &str, body: String| (name.to_string(), body);
    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => Some(h_figure(height, width, web_thick, flange_thick)),
        SectionShape::SteelBox {
            height,
            width,
            thick,
            corner_r,
        } => Some(box_figure(height, width, thick, corner_r)),
        SectionShape::SteelPipe { outer_dia, thick } => Some(pipe_figure(outer_dia, thick)),
        SectionShape::SteelAngle {
            leg_a,
            leg_b,
            thick,
        } => {
            let name = format!("L-{}x{}x{}", num(leg_a), num(leg_b), num(thick));
            let body = format!(
                "<StbSecRoll-L name=\"{}\" type=\"L\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" r1=\"0\" r2=\"0\"/>",
                esc(&name),
                num(leg_a),
                num(leg_b),
                num(thick),
                num(thick)
            );
            Some(e(&name, body))
        }
        SectionShape::SteelChannel {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let name = format!(
                "C-{}x{}x{}x{}",
                num(height),
                num(width),
                num(web_thick),
                num(flange_thick)
            );
            let body = format!(
                "<StbSecRoll-C name=\"{}\" type=\"C\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" r1=\"0\" r2=\"0\"/>",
                esc(&name),
                num(height),
                num(width),
                num(web_thick),
                num(flange_thick)
            );
            Some(e(&name, body))
        }
        SectionShape::SteelTee {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let name = format!(
                "T-{}x{}x{}x{}",
                num(height),
                num(width),
                num(web_thick),
                num(flange_thick)
            );
            let body = format!(
                "<StbSecRoll-T name=\"{}\" type=\"T\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" r1=\"0\" r2=\"0\"/>",
                esc(&name),
                num(height),
                num(width),
                num(web_thick),
                num(flange_thick)
            );
            Some(e(&name, body))
        }
        SectionShape::SteelFlatBar { width, thick } => {
            let name = format!("FB-{}x{}", num(width), num(thick));
            let body = format!(
                "<StbSecRoll-FlatBar name=\"{}\" type=\"FlatBar\" B=\"{}\" t=\"{}\"/>",
                esc(&name),
                num(width),
                num(thick)
            );
            Some(e(&name, body))
        }
        SectionShape::SteelRoundBar { dia } => {
            let name = format!("RB-{}", num(dia));
            let body = format!(
                "<StbSecRoll-RoundBar name=\"{}\" type=\"RoundBar\" D=\"{}\"/>",
                esc(&name),
                num(dia)
            );
            Some(e(&name, body))
        }
        SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            web_thick,
        } => {
            let name = format!(
                "BH-{}x{}x{}x{}x{}x{}",
                num(height),
                num(upper_width),
                num(upper_thick),
                num(lower_width),
                num(lower_thick),
                num(web_thick)
            );
            // 標準属性 A/B/t1/t2 は上フランジで表す（第三者は対称 H として読める）。
            // 下フランジは方言属性 B2/t2_lower で持ち、Squid の完全往復を保証する。
            let body = format!(
                "<StbSecBuild-H name=\"{}\" type=\"H\" A=\"{}\" B=\"{}\" t1=\"{}\" t2=\"{}\" B2=\"{}\" t2_lower=\"{}\"/>",
                esc(&name),
                num(height),
                num(upper_width),
                num(web_thick),
                num(upper_thick),
                num(lower_width),
                num(lower_thick)
            );
            Some(e(&name, body))
        }
        SectionShape::SteelLipChannel {
            height,
            width,
            lip,
            thick,
        } => {
            let name = format!(
                "LipC-{}x{}x{}x{}",
                num(height),
                num(width),
                num(lip),
                num(thick)
            );
            let body = format!(
                "<StbSecRoll-LipC name=\"{}\" type=\"LipC\" A=\"{}\" B=\"{}\" C=\"{}\" t=\"{}\" r=\"0\"/>",
                esc(&name),
                num(height),
                num(width),
                num(lip),
                num(thick)
            );
            Some(e(&name, body))
        }
        _ => None,
    }
}

/// 鋼柱断面 `StbSecColumn_S`。`strength` は形鋼参照へ付す `strength_main` 属性（空可）。
fn steel_column(id: u32, sec: &Section, figure: &str, strength: &str) -> String {
    let id = sid(id);
    format!(
        "      <StbSecColumn_S id=\"{}\" name=\"{}\"{} kind_column=\"COLUMN\">\n\
         \x20       <StbSecSteelFigureColumn_S>\n\
         \x20         <StbSecSteelColumn_S_Same shape=\"{}\"{}/>\n\
         \x20       </StbSecSteelFigureColumn_S>\n\
         \x20     </StbSecColumn_S>\n",
        id,
        esc(&sec.name),
        floor_attr(sec),
        esc(figure),
        strength
    )
}

/// 鋼梁断面 `StbSecBeam_S`。
fn steel_beam(id: u32, sec: &Section, figure: &str, strength: &str) -> String {
    let id = sid(id);
    format!(
        "      <StbSecBeam_S id=\"{}\" name=\"{}\"{} kind_beam=\"GIRDER\">\n\
         \x20       <StbSecSteelFigureBeam_S>\n\
         \x20         <StbSecSteelBeam_S_Straight shape=\"{}\"{}/>\n\
         \x20       </StbSecSteelFigureBeam_S>\n\
         \x20     </StbSecBeam_S>\n",
        id,
        esc(&sec.name),
        floor_attr(sec),
        esc(figure),
        strength
    )
}

/// RC 図形 `StbSecFigureColumn_RC` の中身（矩形／円形）。対応しない形状は `None`。
fn rc_column_figure(shape: &SectionShape) -> Option<String> {
    match *shape {
        SectionShape::RcRect { b, d, .. } => Some(format!(
            "<StbSecColumn_RC_Rect width_X=\"{}\" width_Y=\"{}\"/>",
            num(b),
            num(d)
        )),
        SectionShape::RcCircle { d, .. } => {
            Some(format!("<StbSecColumn_RC_Circle D=\"{}\"/>", num(d)))
        }
        _ => None,
    }
}

/// RC 梁図形 `StbSecFigureBeam_RC` の中身（矩形のみ）。対応しない形状は `None`。
fn rc_beam_figure(shape: &SectionShape) -> Option<String> {
    match *shape {
        SectionShape::RcRect { b, d, .. } => Some(format!(
            "<StbSecBeam_RC_Straight width=\"{}\" depth=\"{}\"/>",
            num(b),
            num(d)
        )),
        _ => None,
    }
}

/// 配筋（[`RcRebar`]）を配筋子要素（`*_Same`）の属性文字列へ整形する（標準名のみ）。
/// かぶりは配置コンテナ側に付くため、ここには含めない。
/// - 柱（`is_beam=false`）: `D_main`・`N_main_X_1st`・`N_main_Y_1st`・帯筋 `D_band`・
///   `pitch_band`・`N_band_direction_X`/`_Y`・`strength_band`。
/// - 梁（`is_beam=true`）: `D_main`・`N_main_top_1st`・`N_main_bottom_1st`・あばら筋
///   `D_stirrup`・`pitch_stirrup`・`N_stirrup`・`strength_stirrup`。
fn rebar_attrs(r: &RcRebar, grades: BarGrades<'_>, is_beam: bool) -> String {
    if is_beam {
        let mut s = format!(
            "D_main=\"{dm}\" N_main_top_1st=\"{nt}\" N_main_bottom_1st=\"{nb}\" \
             D_stirrup=\"{ds}\" pitch_stirrup=\"{ps}\" N_stirrup=\"{ns}\"",
            dm = num(r.main_x.dia),
            nt = r.main_x.count,
            nb = r.main_y.count,
            ds = num(r.shear.dia),
            ps = num(r.shear.pitch),
            ns = r.shear.legs,
        );
        if let Some(g) = grades.shear {
            s.push_str(&format!(" strength_stirrup=\"{}\"", esc(g)));
        }
        if let Some(g) = grades.main {
            s.push_str(&format!(" strength_main=\"{}\"", esc(g)));
        }
        s
    } else {
        let mut s = format!(
            "D_main=\"{dm}\" N_main_X_1st=\"{nx}\" N_main_Y_1st=\"{ny}\" \
             D_band=\"{db}\" pitch_band=\"{pb}\" N_band_direction_X=\"{nb}\" N_band_direction_Y=\"{nb}\"",
            dm = num(r.main_x.dia),
            nx = r.main_x.count,
            ny = r.main_y.count,
            db = num(r.shear.dia),
            pb = num(r.shear.pitch),
            nb = r.shear.legs,
        );
        if let Some(g) = grades.shear {
            s.push_str(&format!(" strength_band=\"{}\"", esc(g)));
        }
        if let Some(g) = grades.main {
            s.push_str(&format!(" strength_main=\"{}\"", esc(g)));
        }
        s
    }
}

/// 梁配筋コンテナのかぶり属性（`cover>0` のときのみ。ST-Bridge の length は >0 必須なので
/// かぶり 0＝未指定は属性ごと省く）。
fn cover_attr_beam(cover: f64) -> String {
    if cover > 0.0 {
        let cv = num(cover);
        format!(" depth_cover_top=\"{cv}\" depth_cover_bottom=\"{cv}\"")
    } else {
        String::new()
    }
}

/// 柱配筋コンテナのかぶり属性（`cover>0` のときのみ）。
fn cover_attr_column(cover: f64) -> String {
    if cover > 0.0 {
        let cv = num(cover);
        format!(
            " depth_cover_start_X=\"{cv}\" depth_cover_end_X=\"{cv}\" \
             depth_cover_start_Y=\"{cv}\" depth_cover_end_Y=\"{cv}\""
        )
    } else {
        String::new()
    }
}

/// RC 柱断面の配筋 `StbSecBarArrangementColumn_RC`（矩形/円形）。配筋のない形状は空文字。
fn rebar_arrangement_column(shape: &SectionShape, grades: BarGrades<'_>) -> String {
    let (child, r) = match shape {
        SectionShape::RcRect { rebar, .. } => ("StbSecBarColumn_RC_RectSame", rebar),
        SectionShape::RcCircle { rebar, .. } => ("StbSecBarColumn_RC_CircleSame", rebar),
        _ => return String::new(),
    };
    format!(
        "        <StbSecBarArrangementColumn_RC{}>\n\
         \x20         <{} {}/>\n\
         \x20       </StbSecBarArrangementColumn_RC>\n",
        cover_attr_column(r.cover),
        child,
        rebar_attrs(r, grades, false)
    )
}

/// RC 梁断面の配筋 `StbSecBarArrangementBeam_RC`（矩形）。配筋のない形状は空文字。
fn rebar_arrangement_beam(shape: &SectionShape, grades: BarGrades<'_>) -> String {
    let r = match shape {
        SectionShape::RcRect { rebar, .. } => rebar,
        _ => return String::new(),
    };
    format!(
        "        <StbSecBarArrangementBeam_RC{}>\n\
         \x20         <StbSecBarBeam_RC_Same {}/>\n\
         \x20       </StbSecBarArrangementBeam_RC>\n",
        cover_attr_beam(r.cover),
        rebar_attrs(r, grades, true)
    )
}

/// RC 柱断面 `StbSecColumn_RC`（図形＋配筋）。`mat` は要素へ付す `strength_concrete`
/// グレード名属性（空可）。
fn rc_column(
    id: u32,
    sec: &Section,
    shape: &SectionShape,
    grades: BarGrades<'_>,
    figure_body: &str,
    id_mat: &str,
) -> String {
    let id = sid(id);
    format!(
        "      <StbSecColumn_RC id=\"{}\" name=\"{}\"{}{}>\n\
         \x20       <StbSecFigureColumn_RC>\n\
         \x20         {}\n\
         \x20       </StbSecFigureColumn_RC>\n\
         {}\
         \x20     </StbSecColumn_RC>\n",
        id,
        esc(&sec.name),
        floor_attr(sec),
        id_mat,
        figure_body,
        rebar_arrangement_column(shape, grades),
    )
}

/// RC 梁断面 `StbSecBeam_RC`（図形＋配筋）。
fn rc_beam(
    id: u32,
    sec: &Section,
    shape: &SectionShape,
    grades: BarGrades<'_>,
    figure_body: &str,
    id_mat: &str,
) -> String {
    let id = sid(id);
    format!(
        "      <StbSecBeam_RC id=\"{}\" name=\"{}\"{}{}>\n\
         \x20       <StbSecFigureBeam_RC>\n\
         \x20         {}\n\
         \x20       </StbSecFigureBeam_RC>\n\
         {}\
         \x20     </StbSecBeam_RC>\n",
        id,
        esc(&sec.name),
        floor_attr(sec),
        id_mat,
        figure_body,
        rebar_arrangement_beam(shape, grades),
    )
}

/// CFT 断面の充填鋼管図形（角形/円形）。`SteelLibrary` に登録し、参照名を返す。
/// CFT 以外は `None`。
fn cft_figure(shape: &SectionShape, steel: &mut SteelLibrary) -> Option<String> {
    let (name, body) = match *shape {
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => box_figure(height, width, thick, 0.0),
        SectionShape::CftPipe { outer_dia, thick } => pipe_figure(outer_dia, thick),
        _ => return None,
    };
    steel.add(&name, body);
    Some(name)
}

/// CFT 柱断面 `StbSecColumn_CFT`（充填鋼管の形鋼参照）。`id_mat` は充填コンクリートの
/// `id_material` 属性（空可）。
fn cft_column(id: u32, sec: &Section, figure: &str, id_mat: &str) -> String {
    let id = sid(id);
    format!(
        "      <StbSecColumn_CFT id=\"{}\" name=\"{}\"{}{}>\n\
         \x20       <StbSecSteelFigureColumn_CFT>\n\
         \x20         <StbSecSteelColumn_CFT_Same shape=\"{}\"/>\n\
         \x20       </StbSecSteelFigureColumn_CFT>\n\
         \x20     </StbSecColumn_CFT>\n",
        id,
        esc(&sec.name),
        floor_attr(sec),
        id_mat,
        esc(figure)
    )
}

/// SRC 断面の内蔵鉄骨（H 形鋼）図形。`SteelLibrary` に登録し、参照名を返す。SRC 以外は `None`。
fn src_steel_figure(shape: &SectionShape, steel: &mut SteelLibrary) -> Option<String> {
    match *shape {
        SectionShape::SrcRect {
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
            ..
        } => {
            let (name, body) = h_figure(
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
            );
            steel.add(&name, body);
            Some(name)
        }
        _ => None,
    }
}

/// SRC 柱／梁断面 `StbSecColumn_SRC` / `StbSecBeam_SRC`
/// （コンクリート図形＋内蔵鉄骨参照＋配筋＋鋼種）。
fn src_section(
    id: u32,
    sec: &Section,
    is_beam: bool,
    shape: &SectionShape,
    grades: BarGrades<'_>,
    steel_fig: &str,
    id_mat: &str,
) -> String {
    // 内蔵鉄骨の鋼種は断面の内蔵鉄骨材料の名前（未割当は空文字列）。
    let steel_grade = grades.steel.unwrap_or("");
    let (b, d, rebar_arrangement, grade) = match shape {
        SectionShape::SrcRect { b, d, .. } => (
            *b,
            *d,
            rebar_arrangement_generic(shape, grades, is_beam, "SRC"),
            steel_grade.to_string(),
        ),
        // 呼び出し側で SrcRect のみ渡す想定。防御的に空で返す。
        _ => return raw(id, sec),
    };
    let (elem, fig_wrap, fig_body, steel_wrap) = if is_beam {
        (
            "StbSecBeam_SRC",
            "StbSecFigureBeam_SRC",
            format!(
                "<StbSecBeam_SRC_Straight width=\"{}\" depth=\"{}\"/>",
                num(b),
                num(d)
            ),
            "StbSecSteelFigureBeam_SRC",
        )
    } else {
        (
            "StbSecColumn_SRC",
            "StbSecFigureColumn_SRC",
            format!(
                "<StbSecColumn_SRC_Rect width_X=\"{}\" width_Y=\"{}\"/>",
                num(b),
                num(d)
            ),
            "StbSecSteelFigureColumn_SRC",
        )
    };
    let steel_same = if is_beam {
        "StbSecSteelBeam_SRC_Same"
    } else {
        "StbSecSteelColumn_SRC_Same"
    };
    let id = sid(id);
    format!(
        "      <{elem} id=\"{id}\" name=\"{name}\"{floor}{id_mat} strength_steel=\"{grade}\">\n\
         \x20       <{fig_wrap}>\n\
         \x20         {fig_body}\n\
         \x20       </{fig_wrap}>\n\
         \x20       <{steel_wrap}>\n\
         \x20         <{steel_same} shape=\"{steel_fig}\"/>\n\
         \x20       </{steel_wrap}>\n\
         {rebar_arrangement}\
         \x20     </{elem}>\n",
        elem = elem,
        id = id,
        name = esc(&sec.name),
        floor = floor_attr(sec),
        id_mat = id_mat,
        grade = esc(&grade),
        fig_wrap = fig_wrap,
        fig_body = fig_body,
        steel_wrap = steel_wrap,
        steel_same = steel_same,
        steel_fig = esc(steel_fig),
        rebar_arrangement = rebar_arrangement,
    )
}

/// SRC の配筋要素 `StbSecBarArrangement{Column,Beam}_SRC`。配筋のない形状は空文字。
/// `kind` は要素名の中置（"SRC"）。
fn rebar_arrangement_generic(
    shape: &SectionShape,
    grades: BarGrades<'_>,
    is_beam: bool,
    kind: &str,
) -> String {
    let r = match shape {
        SectionShape::SrcRect { rebar, .. } => rebar,
        _ => return String::new(),
    };
    let (wrap, child) = if is_beam {
        (
            format!("StbSecBarArrangementBeam_{kind}"),
            format!("StbSecBarBeam_{kind}_Same"),
        )
    } else {
        (
            format!("StbSecBarArrangementColumn_{kind}"),
            format!("StbSecBarColumn_{kind}_RectSame"),
        )
    };
    // かぶりは配置コンテナへ（梁は top/bottom、柱は start_X/end_X/start_Y/end_Y。0 は省く）。
    let cover_attr = if is_beam {
        cover_attr_beam(r.cover)
    } else {
        cover_attr_column(r.cover)
    };
    format!(
        "        <{}{}>\n\
         \x20         <{} {}/>\n\
         \x20       </{}>\n",
        wrap,
        cover_attr,
        child,
        rebar_attrs(r, grades, is_beam),
        wrap
    )
}

/// 標準 ST-Bridge で表現できない断面（形状未定義・CFT 梁・RC 円形梁など）の
/// 最終フォールバック。ST-Bridge に汎用物性断面がないため、物性直持ちの拡張要素
/// `StbSecRaw` で残す（他ソフトは解釈できないが、参照部材の断面リンクは保たれる）。
fn raw(id: u32, sec: &Section) -> String {
    let id = sid(id);
    format!(
        "      <StbSecRaw id=\"{}\" name=\"{}\"{} area=\"{}\" iy=\"{}\" iz=\"{}\" j=\"{}\" depth=\"{}\" width=\"{}\"/>\n",
        id,
        esc(&sec.name),
        floor_attr(sec),
        num(sec.area),
        num(sec.iy),
        num(sec.iz),
        num(sec.j),
        num(sec.depth),
        num(sec.width),
    )
}

/// 標準モードの `<StbSections>` 本体と、部材参照の張り替え用 id マップを生成する。
pub(super) fn standard_sections(model: &Model) -> StandardSections {
    let roles = section_roles(model);
    // 梁用の分割断面へ割り当てる追加 id は、既存 id の最大値の次から採番する。
    let mut next_id = model.sections.iter().map(|s| s.id.0).max().unwrap_or(0) + 1;
    let mut alloc = || {
        let v = next_id;
        next_id += 1;
        v
    };

    // 断面へ付す材料属性（ST-Bridge は材料を断面側にグレード名で持つ）。**内部でも
    // 材料は断面が持つ**ため、部材からの逆算や柱用／梁用の代用は不要で、断面の
    // 主材料をそのまま書き出す。鋼は形鋼参照へ strength_main、RC/CFT/SRC の
    // コンクリートは strength_concrete を付す。
    let mat_name = |base: u32| -> Option<&str> {
        let sec = model.sections.get(base as usize)?;
        let mat = model.materials.get(sec.material?.index())?;
        (!mat.name.is_empty()).then_some(mat.name.as_str())
    };
    let strength_attr = |base: u32| -> String {
        match mat_name(base) {
            Some(name) => format!(" strength_main=\"{}\"", esc(name)),
            None => String::new(),
        }
    };
    let id_mat_attr = |base: u32| -> String {
        match mat_name(base) {
            Some(name) => format!(" strength_concrete=\"{}\"", esc(name)),
            None => String::new(),
        }
    };

    let mut steel = SteelLibrary::default();
    // 断面要素は `StbSections` のスキーマ順（柱 RC/S/SRC/CFT → 梁 RC/S/SRC → …）へ
    // 並べる必要があるため、(順位, XML) で集めて最後に整列する。同順位内は生成順を保つ。
    // 順位: Column_RC=0, Column_S=1, Column_SRC=2, Column_CFT=3, Beam_RC=4, Beam_S=5,
    //       Beam_SRC=6, その他フォールバック(StbSecRaw)=90。
    let mut parts: Vec<(u8, String)> = Vec::new();
    let mut col_map: HashMap<u32, u32> = HashMap::new();
    let mut beam_map: HashMap<u32, u32> = HashMap::new();

    // 壁版だけが参照する厚さ専用断面（thickness のみ・形状なし。import が
    // StbSecWall_RC の厚さから生成する）は、壁断面ブロック（StbSecWall_RC、
    // `export::wall_sections`）側で出力されるためここでは出力しない。
    // 従来は StbSecRaw としても二重に出力され、再取り込みのたびに
    // 「Raw 由来の断面＋厚さ専用断面」が 1 組ずつ増殖していた。
    let wall_only_sections: std::collections::HashSet<u32> = {
        let mut used_by_wall = std::collections::HashSet::new();
        let mut used_by_other = std::collections::HashSet::new();
        for e in &model.elements {
            if let Some(sid) = e.section {
                // 壁ブロック（`export::wall_sections`）の出力対象は壁版と
                // Shell のため、「壁側で出力される」判定もそろえる。
                if matches!(e.kind, ElementKind::Wall | ElementKind::Shell) {
                    used_by_wall.insert(sid.0);
                } else {
                    used_by_other.insert(sid.0);
                }
            }
        }
        for plate in &model.wall_plates {
            if let Some(sid) = plate.section {
                used_by_wall.insert(sid.0);
            }
        }
        // 二次部材（StbBeam/StbPost）は生の断面 id を id_section へ書き出すため、
        // 二次部材が参照する断面を Raw 出力から除外すると出力 XML 内に存在しない
        // 断面参照が生じる。壁専用扱いから外す（Raw を出力する）。
        for sm in model.joists().chain(model.posts()) {
            if let Some(sid) = sm.section {
                used_by_other.insert(sid.0);
            }
        }
        used_by_wall.difference(&used_by_other).copied().collect()
    };

    // 床だけが参照する断面（`SectionShape::RcSlab`）も、スラブ断面ブロック
    // （`StbSecSlab_RC`、`export::slab_sections`）側で出力されるためここでは出さない。
    // 壁と同じ理由で、Raw としても二重に出すと再取り込みのたびに
    // 「Raw 由来の断面＋スラブ断面」が 1 組ずつ増殖する。
    let slab_only_sections: std::collections::HashSet<u32> = {
        let mut used_by_slab = std::collections::HashSet::new();
        let mut used_by_other = std::collections::HashSet::new();
        for slab in &model.slabs {
            if let Some(sid) = slab.section() {
                used_by_slab.insert(sid.0);
            }
        }
        // 小梁は生の断面 id を書き出すため、Raw 出力から外さない。
        for region in &model.floor_regions {
            for j in region.joist_lines() {
                if let Some(sid) = j.section {
                    used_by_other.insert(sid.0);
                }
            }
        }
        for e in &model.elements {
            if let Some(sid) = e.section {
                used_by_other.insert(sid.0);
            }
        }
        for sm in model.joists().chain(model.posts()) {
            if let Some(sid) = sm.section {
                used_by_other.insert(sid.0);
            }
        }
        used_by_slab.difference(&used_by_other).copied().collect()
    };

    for sec in &model.sections {
        let base = sec.id.0;
        if wall_only_sections.contains(&base) && sec.thickness.is_some() && sec.shape.is_none() {
            continue;
        }
        if slab_only_sections.contains(&base)
            && matches!(sec.shape, Some(SectionShape::RcSlab { .. }))
        {
            continue;
        }
        let (used_col, used_beam) = roles.get(&base).copied().unwrap_or((false, false));
        // どの部材からも参照されない断面も出力に残す（既定で柱扱い）。
        let need_col = used_col || !used_beam;
        let need_beam = used_beam;

        // 形状から標準要素を試み、不可なら StbSecRaw へフォールバック。
        let steel_fig = sec.shape.as_ref().and_then(steel_figure);
        if let Some((fig_name, fig_body)) = steel_fig {
            steel.add(&fig_name, fig_body);
            if need_col {
                parts.push((1, steel_column(base, sec, &fig_name, &strength_attr(base))));
                col_map.insert(base, base);
            }
            if need_beam {
                let bid = if need_col { alloc() } else { base };
                parts.push((5, steel_beam(bid, sec, &fig_name, &strength_attr(base))));
                beam_map.insert(base, bid);
            }
            continue;
        }

        // CFT（充填鋼管）: 柱として StbSecColumn_CFT。ST-Bridge に CFT 梁がないため
        // 梁で使われる場合は Raw へフォールバックする。
        if matches!(
            sec.shape,
            Some(SectionShape::CftBox { .. } | SectionShape::CftPipe { .. })
        ) {
            let shape = sec.shape.as_ref().unwrap();
            if need_col {
                let fig = cft_figure(shape, &mut steel).expect("CFT 図形");
                parts.push((3, cft_column(base, sec, &fig, &id_mat_attr(base))));
                col_map.insert(base, base);
            }
            if need_beam {
                let bid = if col_map.contains_key(&base) {
                    alloc()
                } else {
                    base
                };
                parts.push((90, raw(bid, sec)));
                beam_map.insert(base, bid);
            }
            continue;
        }

        // SRC（RC＋内蔵鉄骨）: 柱 StbSecColumn_SRC / 梁 StbSecBeam_SRC。
        if matches!(sec.shape, Some(SectionShape::SrcRect { .. })) {
            let shape = sec.shape.as_ref().unwrap();
            let steel_fig = src_steel_figure(shape, &mut steel).expect("SRC 内蔵鉄骨図形");
            let grades = bar_grades(model, sec);
            if need_col {
                parts.push((
                    2,
                    src_section(
                        base,
                        sec,
                        false,
                        shape,
                        grades,
                        &steel_fig,
                        &id_mat_attr(base),
                    ),
                ));
                col_map.insert(base, base);
            }
            if need_beam {
                let bid = if col_map.contains_key(&base) {
                    alloc()
                } else {
                    base
                };
                parts.push((
                    6,
                    src_section(
                        bid,
                        sec,
                        true,
                        shape,
                        grades,
                        &steel_fig,
                        &id_mat_attr(base),
                    ),
                ));
                beam_map.insert(base, bid);
            }
            continue;
        }

        let rc_col_fig = sec.shape.as_ref().and_then(rc_column_figure);
        let rc_beam_fig = sec.shape.as_ref().and_then(rc_beam_figure);
        if rc_col_fig.is_some() || rc_beam_fig.is_some() {
            let shape = sec.shape.as_ref().expect("RC 図形がある＝shape は Some");
            let grades = bar_grades(model, sec);
            if need_col {
                // 円形など梁図形がない場合も柱としては出力できる。
                if let Some(fig) = &rc_col_fig {
                    parts.push((
                        0,
                        rc_column(base, sec, shape, grades, fig, &id_mat_attr(base)),
                    ));
                    col_map.insert(base, base);
                }
            }
            if need_beam {
                if let Some(fig) = &rc_beam_fig {
                    let bid = if col_map.contains_key(&base) {
                        alloc()
                    } else {
                        base
                    };
                    parts.push((4, rc_beam(bid, sec, shape, grades, fig, &id_mat_attr(base))));
                    beam_map.insert(base, bid);
                } else {
                    // 梁で使われるが梁図形に落ちない形状（例: RC 円形）は Raw で残す。
                    let bid = if col_map.contains_key(&base) {
                        alloc()
                    } else {
                        base
                    };
                    parts.push((90, raw(bid, sec)));
                    beam_map.insert(base, bid);
                }
            }
            // 柱でも梁でも使われない RC 断面は need_col で拾えているが、
            // 梁図形しかない（RcRect を柱に使わない）ケースでも need_col=true のとき
            // rc_col_fig=Some なので出力済み。念のため未出力なら Raw で残す。
            if !col_map.contains_key(&base) && !beam_map.contains_key(&base) {
                parts.push((90, raw(base, sec)));
                col_map.insert(base, base);
                beam_map.insert(base, base);
            }
            continue;
        }

        // フォールバック: 耐震壁・形状未定義。Raw は柱/梁で型分けされないため
        // 両者とも同一 id を参照する。
        parts.push((90, raw(base, sec)));
        col_map.insert(base, base);
        beam_map.insert(base, base);
    }

    // スキーマ順（順位）に整列して結合する。同順位内は生成順（安定ソート）を保つ。
    parts.sort_by_key(|(rank, _)| *rank);
    let mut sections_xml = String::new();
    for (_, xml) in &parts {
        sections_xml.push_str(xml);
    }
    // StbSecSteel（形鋼ライブラリ）は呼び出し側でスラブ・壁断面の後に付す。
    StandardSections {
        sections_xml,
        steel_lib: steel.render(),
        col_map,
        beam_map,
    }
}
