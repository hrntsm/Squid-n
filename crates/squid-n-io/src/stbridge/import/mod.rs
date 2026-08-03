//! ST-Bridge パース（Import）。設計書 §12.5。
//!
//! [`import_stbridge`] は ST-Bridge 標準スキーマ（2.0.2）の断面要素
//! （`StbSecColumn_S`/`StbSecBeam_S`/`StbSecColumn_RC`/`StbSecBeam_RC`/`StbSecColumn_CFT`/
//! `StbSecColumn_SRC`/`StbSecBeam_SRC`/`StbSecSlab_RC`/`StbSecSlabDeck`/`StbSecWall_RC`）＋
//! 形鋼ライブラリ（`StbSecSteel`）を解釈する。形鋼名から内部の [`SectionShape`] を復元し、
//! 断面性能を再算定する。材料は断面のグレード名（鋼 `strength_main`、RC/SRC/CFT の
//! `strength_concrete`）から標準材料表（[`material_std`]）で物性へ解決する。
//! 後方互換のため、Squid-n が過去に書き出した物性直持ち `StbSecRaw` も読み取れる。
//!
//! 標準断面は柱用（`StbSecColumn_*`）と梁用（`StbSecBeam_*`）に型分けされ、柱・梁共有断面の
//! 分割などで断面 id が文書順に整列しないことがある。取り込み後に断面 id を整列・再採番し、
//! 部材の断面参照（`id_section`）を張り替える。node/material/story/section/element の id が
//! 1 始まりや歯抜けでも 0 始まり連番へ正規化する。
//!
//! 処理は 2 段に分かれる。XML のイベント走査（[`parser`]）が要素を file id 付きの
//! 中間表現（本モジュールの `Raw*` / `Pending*` 型）へ集め、組み立て（[`assemble`]）が
//! id 正規化・参照解決を行ってモデルを構築する。

use super::StbError;
use squid_n_core::model::{EndCondition, Model};
use squid_n_core::section_shape::{RcRebar, SectionShape};

mod assemble;
mod material_std;
mod parser;
mod rebar;
mod steel;
mod xml;

/// 断面が持つ材料参照（ST-Bridge は材料を断面側に持つ）。
/// 数値 id（RC/CFT/SRC の `id_material`）または材料名（鋼の `strength_main` グレード）。
#[derive(Clone)]
enum SecMatRef {
    Id(u32),
    Grade(String),
}

/// 取り込み途中の断面（id 整列・形鋼名解決の前）。
struct PendingSec {
    file_id: u32,
    name: String,
    kind: PendingSecKind,
    /// 断面側に付いた材料参照（部材が id_material を持たないとき部材へ伝播する）。
    mat: Option<SecMatRef>,
}

enum PendingSecKind {
    /// 物性直持ち（`StbSecRaw`）。
    Raw {
        area: f64,
        iy: f64,
        iz: f64,
        j: f64,
        depth: f64,
        width: f64,
    },
    /// 形状が確定済み（RC 図形など）。
    Shape(SectionShape),
    /// 形鋼ライブラリ参照（後で名前解決する鋼断面）。
    SteelRef(Option<String>),
    /// CFT（充填鋼管）。充填鋼管の形鋼名を後で解決して CftBox/CftPipe を作る。
    CftRef(Option<String>),
    /// SRC（RC＋内蔵鉄骨）。コンクリート寸法・配筋・鋼種は確定済み、内蔵鉄骨は
    /// 形鋼名を後で解決する。
    SrcRef {
        b: f64,
        d: f64,
        rebar: RcRebar,
        steel_name: Option<String>,
        grade: String,
    },
}

/// 取り込み途中の部材の種別。
enum PendingMemberKind {
    Beam,
    Brace { tension_only: bool },
}

/// 取り込み途中の部材（id 正規化前。参照はすべて file id）。
struct PendingMember {
    kind: PendingMemberKind,
    n_i: u32,
    n_j: u32,
    section: Option<u32>,
    material: Option<u32>,
    /// `id_material` 属性がファイルに存在したか。存在する（=-1 含む）場合は部材が材料を
    /// 明示しているとみなし、断面材料の伝播を行わない（往復で None→Some 化を防ぐ）。
    /// 属性が無い場合のみ断面材料を伝播する。
    has_material_attr: bool,
    /// 部材軸まわりの断面回転角 [deg]（ST-Bridge `rotate`）。ref_vector は節点座標が
    /// 揃う構築時に軸と `rotate` から算出する。
    rotate: f64,
    /// 部材端の接合条件 [i, j]（`condition_bottom`/`top`・`condition_start`/`end`）。
    end_cond: [EndCondition; 2],
}

/// 取り込み途中の二次部材（小梁 `StbBeam`・間柱 `StbPost`。id 正規化前）。
/// 全体解析の対象外で、床荷重・自重を主架構へ CMQ として伝達する部材
/// （`squid_n_core::model::SecondaryMember`）として取り込む。
struct PendingSecondary {
    kind: squid_n_core::model::SecondaryMemberKind,
    n_i: u32,
    n_j: u32,
    section: Option<u32>,
    material: Option<u32>,
    has_material_attr: bool,
    name: String,
}

/// 取り込み途中の節点（id 正規化前）。
struct RawNode {
    file_id: u32,
    coord: [f64; 3],
}

/// 取り込み途中の層（id 正規化前）。
struct RawStory {
    file_id: u32,
    name: String,
    elevation: f64,
    /// `StbStory` 直下 `StbNodeIdList/StbNodeId` が示す所属節点（file node id 列）。
    node_ids: Vec<u32>,
}

/// 取り込み途中の材料（id 正規化前）。
struct RawMaterial {
    file_id: u32,
    name: String,
    young: f64,
    poisson: f64,
    density: f64,
    shear: Option<f64>,
    fc: Option<f64>,
    fy: Option<f64>,
}

/// 取り込み途中の荷重ケース（節点参照は file id）。
struct RawLoadCase {
    name: String,
    nodal: Vec<(u32, [f64; 6])>,
}

/// 取り込み途中のスラブ（節点参照は file id。`StbSlab` + `StbNodeIdOrder`）。
struct RawSlab {
    /// 断面参照（`id_section`。`StbSecSlab_RC` の file id）。負値/未指定は `None`。
    section_fid: Option<u32>,
    /// 境界節点ループ（`StbNodeIdOrder`。file node id 列）。
    boundary: Vec<u32>,
}

/// 取り込み途中の壁（節点参照は file id。`StbWall` + `StbNodeIdOrder`）。
struct RawWall {
    /// 断面参照（`id_section`。`StbSecWall_RC` の file id）。負値/未指定は `None`。
    section_fid: Option<u32>,
    /// 材料参照（`id_material`）。負値/未指定は `None`。
    material_fid: Option<u32>,
    /// 境界節点ループ（`StbNodeIdOrder`。file node id 列）。
    boundary: Vec<u32>,
}

/// 取り込み途中の通り芯グループ（節点参照は file id。`StbAxes` の子）。
struct RawAxisGroup {
    name: String,
    kind: squid_n_core::model::AxisGroupKind,
    axes: Vec<RawAxis>,
}

/// 取り込み途中の通り芯（節点参照は file id）。
struct RawAxis {
    name: String,
    /// 平行芯の原点からの符号付き離れ（`distance`）。平行芯以外は `None`。
    distance: Option<f64>,
    node_ids: Vec<u32>,
}

/// 取り込み時に欠落・近似した内容の報告（データ欠損を顕在化させる）。
#[derive(Debug, Default, Clone)]
pub struct ImportReport {
    /// 人間可読の警告メッセージ（未対応要素のスキップ、断面欠落、参照解決失敗など）。
    pub warnings: Vec<String>,
    /// 取り込み時に自動補完した仮定の通知（支点の自動設定など）。
    /// データ欠損ではないため [`is_clean`](Self::is_clean) には影響しないが、
    /// ユーザーへ明示すべき内容として呼び出し側で表示する。
    pub notes: Vec<String>,
}

impl ImportReport {
    /// 警告が 1 件も無い（＝取り込みで欠落が無かった）か。
    /// 自動補完の通知（`notes`）は欠落ではないため判定に含めない。
    pub fn is_clean(&self) -> bool {
        self.warnings.is_empty()
    }
}

/// ST-Bridge ファイルを読み込み、UTF-8 文字列へデコードする。
///
/// 日本の建築業界では ST-Bridge が Shift_JIS（Windows-31J / CP932）で
/// 保存されるケースが多いため、次の順で判定する（BOM の有無は問わない）。
/// まず UTF-8 BOM 付き、または UTF-8 として妥当なら UTF-8 として扱い、
/// それ以外は Shift_JIS（Windows-31J）としてデコードする。
/// 読み込み自体の失敗 (存在しない等) は [`StbError::Io`] として返す。
pub fn read_stbridge_file(path: &std::path::Path) -> Result<String, StbError> {
    use encoding_rs::SHIFT_JIS;
    let bytes =
        std::fs::read(path).map_err(|e| StbError::Io(format!("{}: {e}", path.display())))?;

    // BOM 付き UTF-8 はそのまま UTF-8 扱い。
    if bytes.starts_with(&[0xEF, 0xBB, 0xBF]) {
        return String::from_utf8(bytes[3..].to_vec())
            .map_err(|e| StbError::Decode(format!("UTF-8 デコードエラー: {e}")));
    }
    // UTF-8 として妥当ならそのまま扱う（ASCII や既存の UTF-8 ファイル互換）。
    if let Ok(s) = String::from_utf8(bytes.clone()) {
        return Ok(s);
    }
    // それ以外は Shift_JIS（Windows-31J / CP932）としてデコードする。
    let (cow, _, had_errors) = SHIFT_JIS.decode(&bytes);
    if had_errors {
        return Err(StbError::Decode(
            "ファイルを UTF-8 または Shift_JIS としてデコードできませんでした".to_string(),
        ));
    }
    Ok(cow.into_owned())
}

/// ST-Bridge 2.0 XML を内部モデルへ取り込む（欠落の報告は破棄する）。
pub fn import_stbridge(xml: &str) -> Result<Model, StbError> {
    import_stbridge_with_report(xml).map(|(m, _)| m)
}

/// ST-Bridge 2.0 XML を内部モデルへ取り込み、[`ImportReport`]（欠落・近似の警告）も返す。
/// XML のイベント走査（[`parser`]）と、中間表現からのモデル組み立て（[`assemble`]）の
/// 2 段で処理する。
pub fn import_stbridge_with_report(xml: &str) -> Result<(Model, ImportReport), StbError> {
    let parsed = parser::parse(xml)?;
    assemble::assemble(parsed)
}
