//! ST-Bridge XML のイベント走査（Import の前段）。
//!
//! XML イベントループの可変状態を [`StbParser`] へ集約し、Start/End/Text の各イベント
//! 処理をメソッドとして分割する。要素は一旦 file id 付きの中間表現（`Raw*` / `Pending*`）
//! へ集め、後段の組み立て（`assemble`）が id 正規化とモデル構築を行う。

use super::super::StbError;
use super::rebar::{default_rebar, parse_rebar};
use super::steel::steel_shape_from;
use super::xml::{
    attrs, get_f64, get_f64_any, get_i64, get_opt_f64, get_u32, push_node_id_tokens, Attrs,
};
use super::{
    PendingMember, PendingMemberKind, PendingSec, PendingSecKind, PendingSecondary, RawAxis,
    RawAxisGroup, RawLoadCase, RawMaterial, RawNode, RawSlab, RawStory, RawWall, SecMatRef,
};
use squid_n_core::section_shape::{RcRebar, SectionShape};
use std::collections::HashMap;

/// RC 断面の図形（配筋と組み合わせて `SectionShape` を確定する）。
pub(super) enum RcGeom {
    Rect { b: f64, d: f64 },
    Circle { d: f64 },
}

/// 現在パース中の標準断面要素（子の図形・配筋要素を集める）。
#[derive(Default)]
pub(super) enum CurSec {
    #[default]
    None,
    Steel {
        file_id: u32,
        name: String,
        /// 断面の階（`floor` 属性）。符号と併せて断面の同一性キーになる。
        floor: Option<String>,
        shape_name: Option<String>,
        grade: Option<String>,
    },
    Rc {
        file_id: u32,
        name: String,
        /// 断面の階（`floor` 属性）。符号と併せて断面の同一性キーになる。
        floor: Option<String>,
        geom: Option<RcGeom>,
        rebar: Option<RcRebar>,
        /// 配筋コンテナ（`StbSecBarArrangement*`）側に付くかぶり [mm]。実 ST-Bridge は
        /// かぶりをコンテナに、本数・径を子の `*_Same` に持つため別枠で控える。
        rebar_cover: Option<f64>,
        /// 断面のコンクリート材料（数値 id または `strength_concrete` グレード名）。
        mat: Option<SecMatRef>,
    },
    Cft {
        file_id: u32,
        name: String,
        /// 断面の階（`floor` 属性）。符号と併せて断面の同一性キーになる。
        floor: Option<String>,
        steel_name: Option<String>,
        mat: Option<SecMatRef>,
    },
    Src {
        file_id: u32,
        name: String,
        /// 断面の階（`floor` 属性）。符号と併せて断面の同一性キーになる。
        floor: Option<String>,
        geom: Option<(f64, f64)>,
        rebar: Option<RcRebar>,
        /// 配筋コンテナ側に付くかぶり [mm]（[`CurSec::Rc`] と同じ）。
        rebar_cover: Option<f64>,
        steel_name: Option<String>,
        grade: String,
        mat: Option<SecMatRef>,
    },
    /// RC スラブ断面（`StbSecSlab_RC`）。子の図形要素から厚さを集める。
    Slab {
        file_id: u32,
        thickness: Option<f64>,
    },
    /// RC 壁断面（`StbSecWall_RC`）。子の図形要素から厚さを集める。
    Wall {
        file_id: u32,
        thickness: Option<f64>,
    },
}

/// ST-Bridge の要素のうち Squid-n が未対応で、取り込み時に必ず警告対象とするもの。
/// これに加え、部材（`StbMembers`）・断面（`StbSections`）・荷重（`StbLoadCase`）の直属子で
/// 未対応のものは、このリストにない未知要素であっても警告する（fail-loud。詳細は
/// [`StbParser::record_unsupported`] を参照）。本リストは、直属の親からは判別しづらい要素
/// （通り芯など `StbModel` 直下のもの）を確実に拾うために併用する。
const UNSUPPORTED_ELEMENTS: &[&str] = &[
    // 部材（面要素・基礎・開口。StbSlab・StbWall は対応済み）
    "StbFooting",
    "StbPile",
    "StbFoundationColumn",
    "StbStripFooting",
    "StbParapet",
    "StbOpen",
    // 断面（基礎・開口。鋼ブレース断面 StbSecBrace_S・StbSecSlab_RC・StbSecWall_RC・
    // デッキ合成スラブ StbSecSlabDeck は対応済み。鋼スラブ StbSecSlab_S は未対応）
    "StbSecSlab_S",
    "StbSecFoundation_RC",
    "StbSecFoundationColumn_RC",
    "StbSecFoundationColumn_SRC",
    "StbSecFoundationColumn_CFT",
    "StbSecPile_RC",
    "StbSecPile_S",
    "StbSecPile_PC",
    "StbSecParapet_RC",
    "StbSecOpen_RC",
];

/// `StbMembers` 直下の部材グループコンテナ（複数形）。実 ST-Bridge は部材を
/// `StbMembers > StbColumns > StbColumn` のように複数形コンテナへ入れ子にする
/// （Squid 方言は `StbMembers > StbColumn` と直下に置く）。これらのコンテナ自体は
/// 単なる入れ物なので未対応警告の対象にせず、その直属子で未対応のものだけを
/// 「取り込み対象外」として拾う（fail-loud。[`StbParser::record_unsupported`] を参照）。
const MEMBER_GROUP_CONTAINERS: &[&str] = &[
    "StbColumns",
    "StbPosts",
    "StbGirders",
    "StbBeams",
    "StbBraces",
    "StbSlabs",
    "StbWalls",
    "StbFootings",
    "StbStripFootings",
    "StbFoundationColumns",
    "StbPiles",
    "StbParapets",
    "StbOpens",
];

/// 属性 1 種類（要素名＋属性名）の出現件数と、そのうち取り込んだ件数。
///
/// 同じ属性でも文脈により取り込む・取り込まないが分かれることがある
/// （例: 断面の図形要素が複数あり、2 つ目以降は最初の図形を採るため参照しない）。
/// 件数を分けて持つことで「一部だけ取り込んだ」状態も報告できる。
#[derive(Debug, Default, Clone, Copy)]
pub(super) struct AttrCount {
    pub(super) total: u32,
    pub(super) imported: u32,
}

/// XML イベントループの可変状態（暗黙の状態機械）を集約したパーサ。
///
/// 全要素を一旦 file id 付きの中間表現へ集め、パース後に id を 0 始まり連番へ
/// 正規化して参照を張り替える（他社ファイルの 1 始まり・歯抜け id に対応）。
/// 集めた中間表現は後段の組み立て（`assemble`）が消費する。
#[derive(Default)]
pub(super) struct StbParser {
    /// ルート `ST_BRIDGE` でバージョン 2.x を確認できたか。
    pub(super) version_ok: bool,
    /// 人間可読の警告（断面図形を認識できず取り込めなかった等）。
    pub(super) warnings: Vec<String>,
    /// 未対応要素はタグごとに件数を集計し、最後にまとめて 1 行の警告にする。
    /// 明示リストにない未知の要素も、部材/断面/荷重の直属子であれば「取り込み対象外」
    /// として拾う（fail-loud。取りこぼしを無言で捨てない）ため、キーは String とする。
    pub(super) unsupported: HashMap<String, u32>,
    /// 属性の扱いの集計（キー: 要素名と属性名、値: 出現件数と取り込んだ件数）。
    /// ファイルに現れた属性はすべて記録し、取り込んだものも取り込まなかったものも
    /// [`ImportReport`](super::ImportReport) で報告する。無視リストは持たない。
    pub(super) attr_usage: HashMap<(String, String), AttrCount>,
    /// 開いている要素のスタック（直属の親要素を知り、未知の部材/断面/荷重を検出するため）。
    pub(super) container_stack: Vec<String>,
    pub(super) raw_nodes: Vec<RawNode>,
    pub(super) raw_stories: Vec<RawStory>,
    pub(super) raw_materials: Vec<RawMaterial>,
    pub(super) raw_load_cases: Vec<RawLoadCase>,
    pub(super) pending_secs: Vec<PendingSec>,
    pub(super) pending_members: Vec<PendingMember>,
    pub(super) pending_secondaries: Vec<PendingSecondary>,
    /// 形鋼ライブラリ（形鋼名 → 断面形状）。
    pub(super) steel_lib: HashMap<String, SectionShape>,
    /// 現在パース中の標準断面要素。
    pub(super) cur: CurSec,
    // --- スラブ・壁関連の中間状態 ---
    pub(super) raw_slabs: Vec<RawSlab>,
    pub(super) slab_sec_thickness: HashMap<u32, f64>,
    pub(super) cur_slab: Option<RawSlab>,
    pub(super) raw_walls: Vec<RawWall>,
    pub(super) wall_sec_thickness: HashMap<u32, f64>,
    pub(super) cur_wall: Option<RawWall>,
    /// `StbNodeIdOrder` を開いた直後（テキストの節点 id 列を受け付ける窓）か。
    pub(super) in_node_id_order: bool,
    /// 実 ST-Bridge の `StbStory`（内部に `StbNodeIdList/StbNodeId` を持つ）を開いている間 true。
    /// 開いている `StbNodeId` を直近の階の所属節点として集めるために使う。
    pub(super) in_story: bool,
    // --- 通り芯（`StbAxes` の子。開いているグループの最後の通りへ `StbNodeId` を集める） ---
    pub(super) raw_axis_groups: Vec<RawAxisGroup>,
    pub(super) cur_axis_group: Option<RawAxisGroup>,
}

/// XML 全体を走査し、中間表現を集めた [`StbParser`] を返す（パース段のエントリポイント）。
pub(super) fn parse(xml: &str) -> Result<StbParser, StbError> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut p = StbParser::default();
    loop {
        let ev = reader
            .read_event()
            .map_err(|e| StbError::Parse(e.to_string()))?;
        // 自己終了要素（<Foo/>）は End が来ないためスタックへ積まない。
        let is_empty = matches!(ev, Event::Empty(_));
        match ev {
            Event::Eof => break,
            Event::Start(e) | Event::Empty(e) => {
                let name = e.name();
                let tag = String::from_utf8_lossy(name.as_ref()).to_string();
                let a = attrs(&e)?;
                p.on_start(&tag, &a)?;
                // 属性の参照は on_start の内側で完結する（ハンドラは値を取り出して
                // 中間表現へ写すだけで、`Attrs` を保持しない）。したがって戻った直後が
                // 「この要素で何を読み、何を読まなかったか」を確定できる唯一の地点。
                p.record_attr_usage(&tag, &a);
                // 開始要素はスタックへ積む（自己終了要素は End が来ないため積まない）。
                if !is_empty {
                    p.container_stack.push(tag);
                }
            }
            Event::End(e) => {
                let name = e.name();
                let tag = String::from_utf8_lossy(name.as_ref()).to_string();
                p.on_end(&tag);
            }
            // StbNodeIdOrder のテキスト内容（空白区切りの節点 id 列）を集める。
            // 節点 id は数字と空白のみで XML 実体参照を含まないため、そのまま UTF-8
            // 解釈でよい。CDATA 形式（<![CDATA[0 1 2 3]]>）にも対応する。
            Event::Text(t) if p.in_node_id_order => {
                p.on_node_id_text(&String::from_utf8_lossy(t.as_ref()));
            }
            Event::CData(t) if p.in_node_id_order => {
                p.on_node_id_text(&String::from_utf8_lossy(t.as_ref()));
            }
            _ => {}
        }
    }

    if !p.version_ok {
        return Err(StbError::Version(
            "missing ST_BRIDGE version 2.x root".into(),
        ));
    }
    Ok(p)
}

impl StbParser {
    /// 開始・自己終了イベントを、意味のまとまりごとの補助メソッドへ振り分ける。
    fn on_start(&mut self, tag: &str, a: &Attrs) -> Result<(), StbError> {
        // StbNodeIdOrder のテキストは開始タグ直後の Text/CData のみで届く。
        // 別要素が現れた時点で取り込み窓を閉じる（自己終了タグ
        // <StbNodeIdOrder/> は End が来ずフラグが残るため、この明示リセットで
        // 無関係な子要素のテキストを境界へ誤取り込みするのを防ぐ）。
        if tag != "StbNodeIdOrder" {
            self.in_node_id_order = false;
        }
        // 各補助メソッドは担当タグなら処理して true を返す。担当タグ集合は互いに素
        // なので、順に試しても元の単一 match と同じ挙動になる。
        if self.start_root(tag, a)? {
            return Ok(());
        }
        if self.start_node_story_material(tag, a)? {
            return Ok(());
        }
        if self.start_section(tag, a)? {
            return Ok(());
        }
        if self.start_member(tag, a)? {
            return Ok(());
        }
        if self.start_load(tag, a)? {
            return Ok(());
        }
        if self.start_slab_wall(tag, a) {
            return Ok(());
        }
        if self.start_axes(tag, a) {
            return Ok(());
        }
        if self.start_node_id_ref(tag, a) {
            return Ok(());
        }
        self.record_unsupported(tag);
        Ok(())
    }

    /// ルート要素 `ST_BRIDGE` を処理する（バージョン 2.x の検証）。担当タグなら true。
    fn start_root(&mut self, tag: &str, a: &Attrs) -> Result<bool, StbError> {
        if tag != "ST_BRIDGE" {
            return Ok(false);
        }
        let v = a.get("version").cloned().unwrap_or_default();
        if !v.starts_with("2.") {
            return Err(StbError::Version(v));
        }
        self.version_ok = true;
        Ok(true)
    }

    /// 節点・階・材料の開始要素を処理する。担当タグなら true。
    fn start_node_story_material(&mut self, tag: &str, a: &Attrs) -> Result<bool, StbError> {
        match tag {
            "StbNode" => {
                self.raw_nodes.push(RawNode {
                    file_id: get_u32(a, "id")?,
                    // 座標は ST-Bridge 標準の大文字 `X`/`Y`/`Z`。
                    coord: [get_f64(a, "X")?, get_f64(a, "Y")?, get_f64(a, "Z")?],
                });
            }
            "StbStory" => {
                self.raw_stories.push(RawStory {
                    file_id: get_u32(a, "id")?,
                    name: a.get("name").cloned().unwrap_or_default(),
                    elevation: get_f64(a, "height")?,
                    node_ids: Vec::new(),
                });
                // 直下の StbNodeIdList/StbNodeId をこの階へ集める窓を開く
                // （空の <StbStory/> でも害はない。StbNodeId はスラブ・壁を優先し、
                // かつ階は通常部材より前に現れるため誤取り込みしない）。
                self.in_story = true;
            }
            "StbMaterial" => {
                self.raw_materials.push(RawMaterial {
                    file_id: get_u32(a, "id")?,
                    name: a.get("name").cloned().unwrap_or_default(),
                    young: get_f64(a, "young")?,
                    poisson: get_f64(a, "poisson")?,
                    density: get_f64(a, "density")?,
                    shear: get_opt_f64(a, "shear"),
                    fc: get_opt_f64(a, "fc"),
                    fy: get_opt_f64(a, "fy"),
                });
            }
            _ => return Ok(false),
        }
        Ok(true)
    }

    /// 断面（Raw・鋼・RC・CFT・SRC・スラブ・壁・配筋・形鋼ライブラリ）の
    /// 開始要素を処理する。担当タグなら true。
    fn start_section(&mut self, tag: &str, a: &Attrs) -> Result<bool, StbError> {
        match tag {
            // --- 断面: 物性直持ち（Raw） ---
            "StbSecRaw" => {
                self.pending_secs.push(PendingSec {
                    file_id: get_u32(a, "id")?,
                    name: a.get("name").cloned().unwrap_or_default(),
                    floor: floor_of(a),
                    kind: PendingSecKind::Raw {
                        area: get_f64(a, "area")?,
                        iy: get_f64(a, "iy")?,
                        iz: get_f64(a, "iz")?,
                        j: get_f64(a, "j")?,
                        depth: get_f64(a, "depth").unwrap_or(0.0),
                        width: get_f64(a, "width").unwrap_or(0.0),
                    },
                    mat: None,
                });
            }
            // --- 断面: 標準要素（鋼。柱・梁・ブレース） ---
            "StbSecColumn_S" | "StbSecBeam_S" | "StbSecBrace_S" => {
                self.cur = CurSec::Steel {
                    file_id: get_u32(a, "id")?,
                    name: a.get("name").cloned().unwrap_or_default(),
                    floor: floor_of(a),
                    shape_name: None,
                    // 鋼種は形鋼参照（下）に付くことが多いが、要素側にあれば拾う。
                    grade: a.get("strength_main").cloned(),
                };
            }
            // 鋼／CFT／SRC 断面の図形参照（`*_Same` / `*_Straight`）。`shape` 系属性から
            // 形鋼名を、`strength_main` から鋼種を取り、現在の断面種別へ格納する。
            t if t.starts_with("StbSecSteelColumn_")
                || t.starts_with("StbSecSteelBeam_")
                || t.starts_with("StbSecSteelBrace_") =>
            {
                let sname = a
                    .get("shape")
                    .or_else(|| a.get("shape_start"))
                    .or_else(|| a.get("shape_center"))
                    .or_else(|| a.get("shape_main"))
                    .cloned();
                let gr = a.get("strength_main").cloned();
                match &mut self.cur {
                    CurSec::Steel {
                        shape_name, grade, ..
                    } => {
                        if shape_name.is_none() {
                            *shape_name = sname;
                        }
                        if grade.is_none() {
                            *grade = gr;
                        }
                    }
                    CurSec::Cft { steel_name, .. } if steel_name.is_none() => *steel_name = sname,
                    CurSec::Src { steel_name, .. } if steel_name.is_none() => *steel_name = sname,
                    _ => {}
                }
            }
            // --- 断面: 標準要素（RC） ---
            "StbSecColumn_RC" | "StbSecBeam_RC" => {
                self.cur = CurSec::Rc {
                    file_id: get_u32(a, "id")?,
                    name: a.get("name").cloned().unwrap_or_default(),
                    floor: floor_of(a),
                    geom: None,
                    rebar: None,
                    rebar_cover: None,
                    mat: sec_mat_ref_of(a),
                };
            }
            "StbSecColumn_RC_Rect" => {
                if let CurSec::Rc { geom, .. } = &mut self.cur {
                    if geom.is_none() {
                        *geom = Some(RcGeom::Rect {
                            b: get_f64_any(a, &["width_X", "width_x"])?,
                            d: get_f64_any(a, &["width_Y", "width_y"])?,
                        });
                    }
                }
            }
            "StbSecColumn_RC_Circle" => {
                if let CurSec::Rc { geom, .. } = &mut self.cur {
                    if geom.is_none() {
                        *geom = Some(RcGeom::Circle {
                            d: get_f64_any(a, &["D", "d"])?,
                        });
                    }
                }
            }
            "StbSecBeam_RC_Straight" => {
                if let CurSec::Rc { geom, .. } = &mut self.cur {
                    if geom.is_none() {
                        *geom = Some(RcGeom::Rect {
                            b: get_f64_any(a, &["width", "width_X"])?,
                            d: get_f64_any(a, &["depth", "width_Y"])?,
                        });
                    }
                }
            }
            // --- 断面: 標準要素（CFT） ---
            "StbSecColumn_CFT" => {
                self.cur = CurSec::Cft {
                    file_id: get_u32(a, "id")?,
                    name: a.get("name").cloned().unwrap_or_default(),
                    floor: floor_of(a),
                    steel_name: None,
                    mat: sec_mat_ref_of(a),
                };
            }
            // --- 断面: 標準要素（SRC） ---
            "StbSecColumn_SRC" | "StbSecBeam_SRC" => {
                self.cur = CurSec::Src {
                    file_id: get_u32(a, "id")?,
                    name: a.get("name").cloned().unwrap_or_default(),
                    floor: floor_of(a),
                    geom: None,
                    rebar: None,
                    rebar_cover: None,
                    steel_name: None,
                    grade: a
                        .get("strength_steel")
                        .or_else(|| a.get("strength_main_S"))
                        .cloned()
                        .unwrap_or_default(),
                    mat: sec_mat_ref_of(a),
                };
            }
            "StbSecColumn_SRC_Rect" => {
                if let CurSec::Src { geom, .. } = &mut self.cur {
                    if geom.is_none() {
                        *geom = Some((
                            get_f64_any(a, &["width_X", "width_x"])?,
                            get_f64_any(a, &["width_Y", "width_y"])?,
                        ));
                    }
                }
            }
            "StbSecBeam_SRC_Straight" => {
                if let CurSec::Src { geom, .. } = &mut self.cur {
                    if geom.is_none() {
                        *geom = Some((
                            get_f64_any(a, &["width", "width_X"])?,
                            get_f64_any(a, &["depth", "width_Y"])?,
                        ));
                    }
                }
            }
            // 配筋コンテナ（`StbSecBarArrangement*`）。実 ST-Bridge はかぶり
            // （`depth_cover_*`）を配置コンテナ側に、本数・径を子の `*_Same` 側に持つ。
            // 本数・径は下の `*_Same` 分岐で拾うため、ここではかぶりのみを控える。
            t if t.starts_with("StbSecBarArrangement") => {
                if let Ok(c) = get_f64_any(
                    a,
                    &[
                        "depth_cover",
                        "depth_cover_top",
                        "depth_cover_bottom",
                        "depth_cover_start",
                        "depth_cover_start_X",
                        "depth_cover_end_X",
                        "depth_cover_start_Y",
                        "cover",
                        "kaburi",
                    ],
                ) {
                    match &mut self.cur {
                        CurSec::Rc { rebar_cover, .. } => *rebar_cover = Some(c),
                        CurSec::Src { rebar_cover, .. } => *rebar_cover = Some(c),
                        _ => {}
                    }
                }
            }
            // 配筋（RC / SRC の StbSecBar{Column,Beam}_*_Same 子要素）。現在の断面種別へ格納。
            t if t.starts_with("StbSecBarColumn_") || t.starts_with("StbSecBarBeam_") => {
                match &mut self.cur {
                    CurSec::Rc { rebar, .. } if rebar.is_none() => *rebar = Some(parse_rebar(a)),
                    CurSec::Src { rebar, .. } if rebar.is_none() => *rebar = Some(parse_rebar(a)),
                    _ => {}
                }
            }
            // --- 形鋼ライブラリ ---
            t if t.starts_with("StbSecRoll-")
                || t.starts_with("StbSecBuild-")
                || t == "StbSecPipe" =>
            {
                if let (Some(nm), Some(shape)) = (a.get("name").cloned(), steel_shape_from(t, a)) {
                    self.steel_lib.entry(nm).or_insert(shape);
                }
            }
            // --- スラブ断面: RC（StbSecSlab_RC）／デッキ合成（StbSecSlabDeck）。
            //     厚さ（コンクリート部せい）を図形の子要素から集める。 ---
            "StbSecSlab_RC" | "StbSecSlabDeck" => {
                self.cur = CurSec::Slab {
                    file_id: get_u32(a, "id")?,
                    thickness: get_f64_any(a, &["depth", "thickness", "t", "D"]).ok(),
                };
            }
            // スラブ断面の図形（厚さ = `depth`）。RC・デッキ双方の図形要素を受ける。
            "StbSecSlab_RC_Straight"
            | "StbSecFigureSlab_RC"
            | "StbSecSlabDeckStraight"
            | "StbSecFigureSlabDeck" => {
                if let CurSec::Slab { thickness, .. } = &mut self.cur {
                    // 厚さ属性を持つ図形要素なら更新、なければ既存値を保持。
                    *thickness = get_f64_any(a, &["depth", "thickness", "t", "D"])
                        .ok()
                        .or(*thickness);
                }
            }
            // --- 壁断面（StbSecWall_RC）: 厚さを子要素から集める ---
            "StbSecWall_RC" => {
                self.cur = CurSec::Wall {
                    file_id: get_u32(a, "id")?,
                    thickness: get_f64_any(a, &["thickness", "t", "depth", "D"]).ok(),
                };
            }
            "StbSecWall_RC_Straight" | "StbSecFigureWall_RC" => {
                if let CurSec::Wall { thickness, .. } = &mut self.cur {
                    *thickness = get_f64_any(a, &["thickness", "t", "depth", "D"])
                        .ok()
                        .or(*thickness);
                }
            }
            _ => return Ok(false),
        }
        Ok(true)
    }

    /// 部材（柱・大梁・小梁・間柱・ブレース）の開始要素を処理する。担当タグなら true。
    fn start_member(&mut self, tag: &str, a: &Attrs) -> Result<bool, StbError> {
        match tag {
            "StbColumn" => {
                let bot = get_u32(a, "id_node_bottom")?;
                let top = get_u32(a, "id_node_top")?;
                self.pending_members
                    .push(make_member(a, bot, top, PendingMemberKind::Beam)?);
            }
            "StbGirder" => {
                let st = get_u32(a, "id_node_start")?;
                let en = get_u32(a, "id_node_end")?;
                self.pending_members
                    .push(make_member(a, st, en, PendingMemberKind::Beam)?);
            }
            // 小梁（StbBeam）は二次部材: 全体解析の対象外とし、床荷重・自重を
            // 大梁へ CMQ（中間集中荷重）として伝達する部材として取り込む。
            "StbBeam" => {
                let st = get_u32(a, "id_node_start")?;
                let en = get_u32(a, "id_node_end")?;
                self.pending_secondaries.push(make_secondary(
                    a,
                    st,
                    en,
                    squid_n_core::model::SecondaryMemberKind::Joist,
                ));
            }
            // 間柱（StbPost）も二次部材（鉛直材。柱と同じく bottom/top を持つ。
            // start/end も許容）。
            "StbPost" => {
                let bot = get_u32(a, "id_node_bottom").or_else(|_| get_u32(a, "id_node_start"))?;
                let top = get_u32(a, "id_node_top").or_else(|_| get_u32(a, "id_node_end"))?;
                self.pending_secondaries.push(make_secondary(
                    a,
                    bot,
                    top,
                    squid_n_core::model::SecondaryMemberKind::Post,
                ));
            }
            "StbBrace" => {
                let st = get_u32(a, "id_node_start")?;
                let en = get_u32(a, "id_node_end")?;
                // `feature_brace`（既定 TENSION）。TENSIONANDCOMPRESSION のみ
                // 引張圧縮両用、それ以外（TENSION・未指定）は引張専用。
                let tension_only = a
                    .get("feature_brace")
                    .map(|v| v != "TENSIONANDCOMPRESSION")
                    .unwrap_or(true);
                self.pending_members.push(make_member(
                    a,
                    st,
                    en,
                    PendingMemberKind::Brace { tension_only },
                )?);
            }
            _ => return Ok(false),
        }
        Ok(true)
    }

    /// 荷重ケース・節点荷重の開始要素を処理する。担当タグなら true。
    fn start_load(&mut self, tag: &str, a: &Attrs) -> Result<bool, StbError> {
        match tag {
            "StbLoadCase" => {
                self.raw_load_cases.push(RawLoadCase {
                    name: a.get("name").cloned().unwrap_or_default(),
                    nodal: vec![],
                });
            }
            "StbNodalLoad" => {
                let node = get_u32(a, "id_node")?;
                let values = [
                    get_f64(a, "fx").unwrap_or(0.0),
                    get_f64(a, "fy").unwrap_or(0.0),
                    get_f64(a, "fz").unwrap_or(0.0),
                    get_f64(a, "mx").unwrap_or(0.0),
                    get_f64(a, "my").unwrap_or(0.0),
                    get_f64(a, "mz").unwrap_or(0.0),
                ];
                if let Some(lc) = self.raw_load_cases.last_mut() {
                    lc.nodal.push((node, values));
                } else {
                    return Err(StbError::Parse("StbNodalLoad outside StbLoadCase".into()));
                }
            }
            _ => return Ok(false),
        }
        Ok(true)
    }

    /// スラブ・壁の開始要素を処理する（境界節点ループは後続の `StbNodeIdOrder` 等で
    /// 集める）。担当タグなら true。
    fn start_slab_wall(&mut self, tag: &str, a: &Attrs) -> bool {
        match tag {
            // --- スラブ（StbSlab）: 境界節点ループを StbNodeIdOrder から集める ---
            "StbSlab" => {
                // 自己終了 <StbWall/> 等で残った兄弟状態をクリアし、境界ノードの
                // 取り違えを防ぐ（StbSlab/StbWall は入れ子にならない）。
                self.cur_wall = None;
                self.cur_slab = Some(RawSlab {
                    section_fid: match get_i64(a, "id_section") {
                        Some(s) if s >= 0 => Some(s as u32),
                        _ => None,
                    },
                    boundary: Vec::new(),
                });
            }
            // --- 壁（StbWall）: 境界節点ループを StbNodeIdOrder から集める ---
            "StbWall" => {
                self.cur_slab = None;
                self.cur_wall = Some(RawWall {
                    section_fid: match get_i64(a, "id_section") {
                        Some(s) if s >= 0 => Some(s as u32),
                        _ => None,
                    },
                    material_fid: match get_i64(a, "id_material") {
                        Some(s) if s >= 0 => Some(s as u32),
                        _ => None,
                    },
                    boundary: Vec::new(),
                });
            }
            _ => return false,
        }
        true
    }

    /// 通り芯（`StbAxes` とその子）の開始要素を処理する。担当タグなら true。
    fn start_axes(&mut self, tag: &str, a: &Attrs) -> bool {
        match tag {
            // --- 通り芯（StbAxes）---
            // 平行芯は原点・方向角ごと取り込む。円弧芯・放射芯・作図芯は
            // 幾何を表す型を持たないため `Other` とし、通り名と所属節点だけを
            // 取り込む（通り芯は識別用のデータなので、所属が残れば用を成す）。
            "StbAxes" => {}
            "StbParallelAxes" | "StbArcAxes" | "StbRadialAxes" | "StbDrawingAxes" => {
                if let Some(g) = self.cur_axis_group.take() {
                    self.raw_axis_groups.push(g);
                }
                let kind = if tag == "StbParallelAxes" {
                    squid_n_core::model::AxisGroupKind::Parallel {
                        origin: [
                            get_opt_f64(a, "X").unwrap_or(0.0),
                            get_opt_f64(a, "Y").unwrap_or(0.0),
                        ],
                        angle_deg: get_opt_f64(a, "angle").unwrap_or(0.0),
                    }
                } else {
                    squid_n_core::model::AxisGroupKind::Other
                };
                self.cur_axis_group = Some(RawAxisGroup {
                    name: a.get("group_name").cloned().unwrap_or_default(),
                    kind,
                    axes: Vec::new(),
                });
            }
            "StbParallelAxis" | "StbArcAxis" | "StbRadialAxis" | "StbDrawingAxis" => {
                if let Some(g) = self.cur_axis_group.as_mut() {
                    g.axes.push(RawAxis {
                        name: a.get("name").cloned().unwrap_or_default(),
                        distance: get_opt_f64(a, "distance"),
                        node_ids: Vec::new(),
                    });
                }
            }
            _ => return false,
        }
        true
    }

    /// 節点 id 参照（`StbNodeIdOrder`・`StbNodeId`）の開始要素を処理する。担当タグなら true。
    fn start_node_id_ref(&mut self, tag: &str, a: &Attrs) -> bool {
        match tag {
            "StbNodeIdOrder" => {
                self.in_node_id_order = true;
            }
            // 節点ループを子要素形式（<StbNodeId id="…"/>）で持つ方言に対応。
            // スラブ・壁のうち現在開いている方の境界へ追加する。
            "StbNodeId" => {
                if let Ok(id) = get_u32(a, "id") {
                    if let Some(slab) = self.cur_slab.as_mut() {
                        slab.boundary.push(id);
                    } else if let Some(wall) = self.cur_wall.as_mut() {
                        wall.boundary.push(id);
                    } else if self.in_story {
                        if let Some(story) = self.raw_stories.last_mut() {
                            story.node_ids.push(id);
                        }
                    } else if let Some(axis) =
                        self.cur_axis_group.as_mut().and_then(|g| g.axes.last_mut())
                    {
                        axis.node_ids.push(id);
                    }
                }
            }
            _ => return false,
        }
        true
    }

    /// どの取り込み分岐にも該当しなかった要素をデータ欠落として集計する（fail-loud）。
    ///
    /// 明示リストに加え、部材グループコンテナ（StbColumns 等）の直属子・断面（StbSections、
    /// ただし形鋼ライブラリコンテナ StbSecSteel は除く）・荷重（StbLoadCase）の直属子で
    /// 未対応のものは、リスト外の未知要素であっても「取り込み対象外」として拾う。
    /// グループコンテナ自体（StbColumns 等）や StbMembers は入れ物なので拾わない。
    fn record_unsupported(&mut self, tag: &str) {
        // この要素の直属の親（未知の部材/断面/荷重の検出に使う）。
        let parent = self.container_stack.last().map(|s| s.as_str());
        let is_group_container = tag == "StbMembers" || MEMBER_GROUP_CONTAINERS.contains(&tag);
        let parent_is_member_group = parent.is_some_and(|p| MEMBER_GROUP_CONTAINERS.contains(&p));
        let skipped_data = !is_group_container
            && (UNSUPPORTED_ELEMENTS.contains(&tag)
                || parent_is_member_group
                || (matches!(parent, Some("StbSections")) && tag != "StbSecSteel")
                || matches!(parent, Some("StbLoadCase")));
        if skipped_data {
            *self.unsupported.entry(tag.to_string()).or_insert(0) += 1;
        }
    }

    /// この要素に存在した属性を、取り込んだもの・取り込まなかったものに分けて集計する。
    ///
    /// 無視リストは持たない。`guid` のように解析に用いない属性も「取り込まなかった」と
    /// して報告し、どの属性がどう扱われたかを利用者がすべて追えるようにする。
    fn record_attr_usage(&mut self, tag: &str, a: &Attrs) {
        let unread: std::collections::HashSet<&str> = a.unread().into_iter().collect();
        for name in a.names() {
            let e = self
                .attr_usage
                .entry((tag.to_string(), name.to_string()))
                .or_default();
            e.total += 1;
            if !unread.contains(name) {
                e.imported += 1;
            }
        }
    }

    /// 終了イベントを処理する（開いていた断面・階・通り芯・スラブ・壁を閉じる）。
    fn on_end(&mut self, tag: &str) {
        // 対応する開始要素をスタックから降ろす。
        self.container_stack.pop();
        if self.end_section(tag) {
            return;
        }
        match tag {
            "StbNodeIdOrder" => {
                self.in_node_id_order = false;
            }
            "StbStory" => {
                self.in_story = false;
            }
            "StbParallelAxes" | "StbArcAxes" | "StbRadialAxes" | "StbDrawingAxes" => {
                if let Some(g) = self.cur_axis_group.take() {
                    self.raw_axis_groups.push(g);
                }
            }
            "StbSlab" => {
                if let Some(slab) = self.cur_slab.take() {
                    self.raw_slabs.push(slab);
                }
            }
            "StbWall" => {
                if let Some(wall) = self.cur_wall.take() {
                    self.raw_walls.push(wall);
                }
            }
            _ => {}
        }
    }

    /// 標準断面要素の終了を処理する（開いていた断面を確定して保留リストへ積む）。
    /// 担当タグなら true。
    fn end_section(&mut self, tag: &str) -> bool {
        match tag {
            "StbSecColumn_S" | "StbSecBeam_S" | "StbSecBrace_S" => {
                if let CurSec::Steel {
                    file_id,
                    name,
                    floor,
                    shape_name,
                    grade,
                } = std::mem::replace(&mut self.cur, CurSec::None)
                {
                    self.pending_secs.push(PendingSec {
                        file_id,
                        name,
                        floor,
                        kind: PendingSecKind::SteelRef(shape_name),
                        mat: grade.map(SecMatRef::Grade),
                    });
                }
            }
            "StbSecColumn_RC" | "StbSecBeam_RC" => {
                if let CurSec::Rc {
                    file_id,
                    name,
                    floor,
                    geom,
                    rebar,
                    rebar_cover,
                    mat,
                } = std::mem::replace(&mut self.cur, CurSec::None)
                {
                    match geom {
                        Some(geom) => {
                            // 配筋がない（幾何のみの）ファイルは無筋相当の既定配筋で補う。
                            let mut rebar = rebar.unwrap_or_else(default_rebar);
                            // かぶりが配筋要素側になければ配置コンテナ側の値を採る。
                            if rebar.cover == 0.0 {
                                if let Some(c) = rebar_cover {
                                    rebar.cover = c;
                                }
                            }
                            let shape = match geom {
                                RcGeom::Rect { b, d } => SectionShape::RcRect { b, d, rebar },
                                RcGeom::Circle { d } => SectionShape::RcCircle { d, rebar },
                            };
                            self.pending_secs.push(PendingSec {
                                file_id,
                                name,
                                floor,
                                kind: PendingSecKind::Shape(shape),
                                mat,
                            });
                        }
                        None => self.warnings.push(format!(
                            "RC 断面 (id={file_id}, name=\"{name}\") の図形を認識できず取り込めませんでした（テーパ・ハンチ等は未対応）"
                        )),
                    }
                }
            }
            "StbSecColumn_CFT" => {
                if let CurSec::Cft {
                    file_id,
                    name,
                    floor,
                    steel_name,
                    mat,
                } = std::mem::replace(&mut self.cur, CurSec::None)
                {
                    self.pending_secs.push(PendingSec {
                        file_id,
                        name,
                        floor,
                        kind: PendingSecKind::CftRef(steel_name),
                        mat,
                    });
                }
            }
            "StbSecColumn_SRC" | "StbSecBeam_SRC" => {
                if let CurSec::Src {
                    file_id,
                    name,
                    floor,
                    geom,
                    rebar,
                    rebar_cover,
                    steel_name,
                    grade,
                    mat,
                } = std::mem::replace(&mut self.cur, CurSec::None)
                {
                    match geom {
                        Some((b, d)) => {
                            let mut rebar = rebar.unwrap_or_else(default_rebar);
                            if rebar.cover == 0.0 {
                                if let Some(c) = rebar_cover {
                                    rebar.cover = c;
                                }
                            }
                            self.pending_secs.push(PendingSec {
                                file_id,
                                name,
                                floor,
                                mat,
                                kind: PendingSecKind::SrcRef {
                                    b,
                                    d,
                                    rebar,
                                    steel_name,
                                    grade,
                                },
                            });
                        }
                        None => self.warnings.push(format!(
                            "SRC 断面 (id={file_id}, name=\"{name}\") の図形を認識できず取り込めませんでした"
                        )),
                    }
                }
            }
            "StbSecSlab_RC" | "StbSecSlabDeck" => {
                // 厚さが取れたスラブ断面のみ登録する（cur は必ず None へ戻す）。
                if let CurSec::Slab {
                    file_id,
                    thickness: Some(t),
                } = std::mem::replace(&mut self.cur, CurSec::None)
                {
                    self.slab_sec_thickness.insert(file_id, t);
                }
            }
            "StbSecWall_RC" => {
                if let CurSec::Wall {
                    file_id,
                    thickness: Some(t),
                } = std::mem::replace(&mut self.cur, CurSec::None)
                {
                    self.wall_sec_thickness.insert(file_id, t);
                }
            }
            _ => return false,
        }
        true
    }

    /// `StbNodeIdOrder` のテキスト（空白区切りの節点 id 列）を、開いている
    /// スラブ／壁の境界へ追加する。
    fn on_node_id_text(&mut self, text: &str) {
        let boundary = self
            .cur_slab
            .as_mut()
            .map(|s| &mut s.boundary)
            .or(self.cur_wall.as_mut().map(|w| &mut w.boundary));
        if let Some(b) = boundary {
            push_node_id_tokens(text, b);
        }
    }
}

/// 部材（柱・大梁・ブレース）の中間表現を作る（断面・材料参照・回転角・端部条件を
/// 属性から読む）。
fn make_member(
    a: &Attrs,
    n_i: u32,
    n_j: u32,
    kind: PendingMemberKind,
) -> Result<PendingMember, StbError> {
    let section = match get_i64(a, "id_section") {
        Some(s) if s >= 0 => Some(s as u32),
        _ => None,
    };
    let has_material_attr = a.get("id_material").is_some();
    let material = match get_i64(a, "id_material") {
        Some(m) if m >= 0 => Some(m as u32),
        _ => None,
    };
    // 断面回転角（`rotate`、既定 0）。ref_vector は構築時に軸から算出する。
    let rotate = get_f64(a, "rotate").unwrap_or(0.0);
    // 端部接合条件（柱は bottom/top、大梁・小梁は start/end。既定は FIX）。
    let end_cond = [
        end_condition_of(a, &["condition_bottom", "condition_start"]),
        end_condition_of(a, &["condition_top", "condition_end"]),
    ];
    Ok(PendingMember {
        kind,
        n_i,
        n_j,
        section,
        material,
        has_material_attr,
        rotate,
        end_cond,
    })
}

/// 二次部材（小梁 `StbBeam`・間柱 `StbPost`）の中間表現を作る
/// （[`make_member`] の二次部材版。端部接合条件・回転角は解析に使わないため持たない）。
fn make_secondary(
    a: &Attrs,
    n_i: u32,
    n_j: u32,
    kind: squid_n_core::model::SecondaryMemberKind,
) -> PendingSecondary {
    let section = match get_i64(a, "id_section") {
        Some(s) if s >= 0 => Some(s as u32),
        _ => None,
    };
    let has_material_attr = a.get("id_material").is_some();
    let material = match get_i64(a, "id_material") {
        Some(m) if m >= 0 => Some(m as u32),
        _ => None,
    };
    PendingSecondary {
        kind,
        n_i,
        n_j,
        section,
        material,
        has_material_attr,
        name: a.get("name").cloned().unwrap_or_default(),
    }
}

/// 断面の階（`floor` 属性）。空文字列は「階の指定なし」として `None` に落とす
/// （空文字列と未指定を別扱いにすると、同じ断面がキー違いで二重に残るため）。
fn floor_of(a: &Attrs) -> Option<String> {
    a.get("floor")
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
}

/// 部材端の接合条件属性（`FIX`/`PIN`）を [`EndCondition`] へ写す。既定・未知は `Fixed`。
///
/// [`EndCondition`]: squid_n_core::model::EndCondition
fn end_condition_of(a: &Attrs, keys: &[&str]) -> squid_n_core::model::EndCondition {
    use squid_n_core::model::EndCondition;
    for k in keys {
        if let Some(v) = a.get(k) {
            return match v.as_str() {
                "PIN" => EndCondition::Pinned,
                _ => EndCondition::Fixed,
            };
        }
    }
    EndCondition::Fixed
}

/// RC/SRC/CFT 断面のコンクリート材料参照。数値 id（`id_material` 系）を優先し、
/// なければ ST-Bridge 標準のグレード名 `strength_concrete`（`Fc21` 等）を採る。
fn sec_mat_ref_of(a: &Attrs) -> Option<SecMatRef> {
    let id = get_i64(a, "id_material")
        .or_else(|| get_i64(a, "id_material_concrete"))
        .or_else(|| get_i64(a, "id_material_rc"))
        .filter(|v| *v >= 0);
    if let Some(v) = id {
        return Some(SecMatRef::Id(v as u32));
    }
    a.get("strength_concrete")
        .filter(|s| !s.is_empty())
        .cloned()
        .map(SecMatRef::Grade)
}
