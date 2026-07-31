//! 仕口パネル（柱梁接合部パネル）の諸元解決。
//!
//! 柱・梁の断面形状からパネルの寸法（柱せい `dc`・板厚 `tp`・梁せい `db`）と、
//! そこから定まる形状係数 κ・実効体積 `Ve` を解決する。
//!
//! # 単一の出所とする理由
//!
//! パネルの諸元は次の 2 箇所で必要になる。
//!
//! - **モデル化**（`squid_n_element::panel`）— せん断剛性 `Kxp = Kyp = G・Ve`
//! - **断面検定**（`squid_n_design_jp::steel::panel_zone`）— 降伏モーメント
//!   `pMy = (Ve/κ)・√(1−n²)・Fy/√3`
//!
//! 両者が別々に断面形状を解釈すると、同じ接合部に対して剛性と耐力が食い違う
//! 諸元で算定されうる。本モジュールを唯一の出所とし、双方がここを呼ぶ。
//!
//! # パネル板厚 `tp` の解決順
//!
//! 1. [`Section::panel_thickness`] が入力されていればその値（ダイアフラム補強・
//!    ダブラープレートによる増厚の明示指定）
//! 2. 未入力なら柱の断面形状から算出する（H 形＝ウェブ厚、角形・円形＝板厚）
//!
//! # 対象とする接合部
//!
//! 節点に次のすべてが揃うとき、その接合部を仕口パネルの対象とする
//! （[`resolve_panel_joint`]）。モデル化と断面検定は同じ判定を通る。
//!
//! - 柱（鉛直材）が 1 本以上・はり（水平材）が 1 本以上取り付く
//! - 取り付く**柱・はりがすべて S/CFT 系**（[`StructureKind::Steel`]）
//! - 諸元を解決できる柱が 1 本以上あり、実効体積 `Ve` が正
//!
//! 斜材（ブレース等）は資料が接合位置と係数 ζ を定めておらずパネル自由度と
//! 連成しないため、種別判定の対象にもしない。
//!
//! RC/SRC が 1 本でも混じる接合部を除くのは、コンクリートが接合部全体を拘束し、
//! 鋼部材だけの実効体積による弾性せん断パネルでは挙動を表せないためである。
//! これらの接合部は剛域と、RC 柱梁接合部・SRC パネルゾーンの断面検定が扱う。
//!
//! # 柱が複数取り付く場合
//!
//! 実効体積 `Ve` が最小になる柱の諸元を採る。要素の並び順に依存せず決定的で、
//! かつ剛性・耐力とも安全側になる。`Ve` は `db` に比例するため、柱の選択は
//! `db` の値によらない。
//!
//! # 柱断面ごとの扱い
//!
//! 諸元を解決できるのは H 形鋼・角形鋼管・円形鋼管・CFT（角形・円形）である。
//! 組立 H 形（`SteelBuiltH`）は上下フランジが異なりパネル形状係数 κ の標準式が
//! 適用できないため対象外とする。
//!
//! **モデル化と断面検定で対象範囲が異なる**点に注意する。
//!
//! | 柱断面 | モデル化 | 断面検定 |
//! |---|---|---|
//! | H 形鋼・角形鋼管・円形鋼管 | 対象 | 対象 |
//! | CFT（角形・円形） | **対象外** | 対象 |
//! | RC・SRC・組立 H 形 | 対象外 | 対象外 |
//!
//! CFT を諸元解決の対象に残したまま、モデル化からは
//! [`PanelJoint::has_filled_column`] で除外する。理由は
//! [`PanelGeometry::is_modeling_target`] を参照。

use crate::ids::{ElemId, NodeId};
use crate::model::{ElementData, ElementKind, Model, Section};
use crate::section_shape::SectionShape;
use crate::structure_kind::{member_structure_kind, StructureKind};

/// 部材軸の鉛直成分がこの値以上なら柱（鉛直材）とみなす。
pub const COLUMN_EZ: f64 = 0.8;
/// 部材軸の鉛直成分がこの値以下なら梁（水平材）とみなす。
pub const BEAM_EZ: f64 = 0.2;

/// 仕口パネルに対する部材の向き。
///
/// 斜材（鉛直成分が [`BEAM_EZ`] と [`COLUMN_EZ`] の間）は、資料が接合位置と
/// 係数 ζ を定めていないため、いずれにも分類しない（`None`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemberOrientation {
    /// 柱（鉛直材）。パネルの上下面で接合する。
    Column,
    /// はり（水平材）。パネルの左右面（柱フェース）で接合する。
    Beam,
}

/// 要素の材軸の鉛直成分から柱・はりを判定する。線材以外・退化長さは `None`。
pub fn member_orientation(model: &Model, elem: &ElementData) -> Option<MemberOrientation> {
    if !matches!(elem.kind, ElementKind::Beam) || elem.nodes.len() < 2 {
        return None;
    }
    let p0 = model.nodes.get(elem.nodes[0].index())?.coord;
    let p1 = model.nodes.get(elem.nodes[1].index())?.coord;
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if l < 1e-12 {
        return None;
    }
    let ez = (d[2] / l).abs();
    if ez >= COLUMN_EZ {
        Some(MemberOrientation::Column)
    } else if ez <= BEAM_EZ {
        Some(MemberOrientation::Beam)
    } else {
        None
    }
}

/// 接合部が占める領域の半寸法 [mm]。
///
/// 部材は節点そのものではなく、パネルの面で接合する。その面までの距離が
/// パネル分のオフセットになる。
///
/// - はりは**柱フェース**で接合するため、オフセットは [`Self::column_half`]
/// - 柱は**梁フェース**で接合するため、オフセットは [`Self::beam_half`]
///
/// # 危険断面位置（`RigidZone::face_i` / `face_j`）と分けている理由
///
/// フェース距離は現在「接合する直交部材せいの 1/2」だが、これは断面算定の
/// **既定の**危険断面位置でもあり、将来は任意位置を取りうる。一方パネルの
/// オフセットは接合部の物理的な寸法そのもので、設計上の評価位置がどこへ動いても
/// 変わってはいけない。両者を別の量として扱う。
///
/// また `face_i` は「概ね直交する全部材の最大せい/2」であり、はりにとっては
/// 直交する**はり**も候補に入る。本構造体は柱・はりを明示的に区別するため、
/// はりのオフセットが直交ばりのせいで決まることがない。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PanelHalfExtent {
    /// 柱せいの最大値の 1/2 [mm]（＝はりのオフセット、パネルの水平半寸法）。
    pub column_half: f64,
    /// はりせいの最大値の 1/2 [mm]（＝柱のオフセット、パネルの鉛直半寸法）。
    pub beam_half: f64,
}

impl PanelHalfExtent {
    /// 向き `orientation` の部材が接合する面までのオフセット [mm]。
    pub fn offset_for(&self, orientation: MemberOrientation) -> f64 {
        match orientation {
            MemberOrientation::Beam => self.column_half,
            MemberOrientation::Column => self.beam_half,
        }
    }
}

/// 節点 `node` に取り付く部材 `members` から接合部の半寸法を集める。
///
/// `members` は当該節点へ接続する要素（隣接マップを持つ呼び出し側はそれを渡し、
/// 持たない側は全要素を渡してよい。節点を端点に持たない要素は読み飛ばす）。
pub fn panel_half_extent<'a>(
    model: &Model,
    node: NodeId,
    members: impl IntoIterator<Item = &'a ElementData>,
) -> PanelHalfExtent {
    let mut extent = PanelHalfExtent::default();
    for e in members {
        if !e.nodes.iter().take(2).any(|n| *n == node) {
            continue;
        }
        let Some(orientation) = member_orientation(model, e) else {
            continue;
        };
        let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let half = sec.depth / 2.0;
        match orientation {
            MemberOrientation::Column => extent.column_half = extent.column_half.max(half),
            MemberOrientation::Beam => extent.beam_half = extent.beam_half.max(half),
        }
    }
    extent
}

/// パネルの断面区分（形状係数 κ・実効体積 `Ve` の算定式を分ける）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PanelShapeKind {
    /// H 形鋼柱。`bc`: フランジ幅、`tf`: フランジ厚。パネルはウェブ 1 枚。
    H { bc: f64, tf: f64 },
    /// 角形鋼管柱（CFT 角形を含む）。`bc`: 柱幅。パネルはウェブ 2 枚。
    Box { bc: f64 },
    /// 円形鋼管柱（CFT 円形を含む）。パネルは円筒。
    Pipe,
}

/// 柱断面から解決した仕口パネルの諸元。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PanelGeometry {
    pub kind: PanelShapeKind,
    /// 柱せい方向のパネル寸法 `dc` [mm]（H 形・角形は板厚中心間、円形は径 − 板厚）。
    pub dc: f64,
    /// パネル板厚 `tp` [mm]。
    pub tp: f64,
    /// 充填コンクリートを持つ柱（CFT）か。
    ///
    /// 断面検定では鋼管部を S 造と同じ式で評価するため区別しないが、仕口パネルの
    /// **モデル化からは除外する**（[`Self::is_modeling_target`]）。
    pub filled: bool,
}

impl PanelGeometry {
    /// 柱の断面からパネル諸元を解決する。対象外の断面（RC・SRC・組立 H 形・
    /// 形状未定義）は `None`。
    ///
    /// `tp` は [`Section::panel_thickness`] が正値で入力されていればそちらを
    /// 優先し、未入力なら断面形状から算出する（モジュール冒頭「パネル板厚 `tp`
    /// の解決順」）。
    pub fn from_column(sec: &Section) -> Option<Self> {
        let (kind, dc, tp, filled) = match sec.shape {
            Some(SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            }) => (
                PanelShapeKind::H {
                    bc: width,
                    tf: flange_thick,
                },
                height - flange_thick,
                web_thick,
                false,
            ),
            Some(SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            }) => (
                PanelShapeKind::Box { bc: width },
                height - thick,
                thick,
                false,
            ),
            Some(SectionShape::CftBox {
                height,
                width,
                thick,
            }) => (
                PanelShapeKind::Box { bc: width },
                height - thick,
                thick,
                true,
            ),
            Some(SectionShape::SteelPipe { outer_dia, thick }) => {
                (PanelShapeKind::Pipe, outer_dia - thick, thick, false)
            }
            Some(SectionShape::CftPipe { outer_dia, thick }) => {
                (PanelShapeKind::Pipe, outer_dia - thick, thick, true)
            }
            _ => return None,
        };
        let tp = match sec.panel_thickness {
            Some(t) if t > 0.0 => t,
            _ => tp,
        };
        Some(Self {
            kind,
            dc,
            tp,
            filled,
        })
    }

    /// 仕口パネルの**モデル化**の対象か。
    ///
    /// S 造（H 形鋼・角形鋼管・円形鋼管）は対象、CFT は対象外とする。CFT の接合部は
    /// 充填コンクリートと通しダイアフラムが接合部のせん断挙動に関与し、鋼管のみの
    /// 実効体積による弾性せん断パネル `G・Ve` では剛性を表せないため、接合部を
    /// 剛節点として扱う。
    ///
    /// 断面検定（S 造パネルゾーン）は本判定に依らず CFT も対象に含める（鋼管部を
    /// S 造と同じ式で評価する従来どおりの扱い）。
    pub fn is_modeling_target(&self) -> bool {
        !self.filled
    }

    /// パネルの形状係数 κ（鋼構造接合部設計指針）。
    ///
    /// - H 形: `κ = 1/(2/3 + 4・bc・tf/(dc・tp)) + 1/(1 + dc・tp/(6・bc・tf))`
    /// - 角形: `κ = 1/(2/3 + 2・bc/dc) + 1/(1 + dc/(3・bc))`
    /// - 円形: `κ = 4/π`
    pub fn kappa(&self) -> f64 {
        match self.kind {
            PanelShapeKind::H { bc, tf } => {
                1.0 / (2.0 / 3.0 + (4.0 * bc * tf) / (self.dc * self.tp))
                    + 1.0 / (1.0 + (self.dc * self.tp) / (6.0 * bc * tf))
            }
            PanelShapeKind::Box { bc } => {
                1.0 / (2.0 / 3.0 + 2.0 * bc / self.dc) + 1.0 / (1.0 + self.dc / (3.0 * bc))
            }
            PanelShapeKind::Pipe => 4.0 / std::f64::consts::PI,
        }
    }

    /// パネルの実効体積 `Ve` [mm³]（`db` は梁フランジ板厚中心間距離）。
    ///
    /// - H 形: `Ve = dc・db・tp`（ウェブ 1 枚）
    /// - 角形・円形: `Ve = 2・dc・db・tp`（ウェブ 2 枚相当）
    ///
    /// せん断剛性 `Kxp = Kyp = G・Ve` と降伏モーメント
    /// `pMy = (Ve/κ)・√(1−n²)・Fy/√3` の双方がこの体積を用いる。
    /// 資料の 6 面体体積 `V = Bx・By・Dz` に対し、H 形柱ではウェブ厚方向の寸法を
    /// `By = tp` と対応させたものであり、中実断面ではなく板厚分の実効体積となる。
    pub fn effective_volume(&self, db: f64) -> f64 {
        let base = self.dc * db * self.tp;
        match self.kind {
            PanelShapeKind::H { .. } => base,
            PanelShapeKind::Box { .. } | PanelShapeKind::Pipe => 2.0 * base,
        }
    }
}

/// 仕口パネルを設ける接合部の諸元。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PanelJoint {
    /// 諸元を採った柱の断面区分・`dc`・`tp`。
    pub geometry: PanelGeometry,
    /// 梁フランジ板厚中心間距離 `db` [mm]（取り付くはりの最大）。
    pub db: f64,
    /// 実効体積 `Ve` [mm³]。
    pub ve: f64,
    /// 諸元を採った柱の要素 ID（軸力比 `n`・基準強度 `F` の解決に用いる）。
    pub column: ElemId,
    /// 取り付く柱に充填断面（CFT）が 1 本でもあるか。
    ///
    /// モデル化はこれが `true` の接合部を対象外とする
    /// （[`PanelGeometry::is_modeling_target`] と同じ理由）。断面検定は
    /// 鋼管部を S 造と同じ式で評価するため区別しない。
    pub has_filled_column: bool,
}

/// 節点 `node` に仕口パネルを設けられるか判定し、諸元を解決する。
///
/// `members` は当該節点へ接続する要素（隣接マップを持つ呼び出し側はそれを渡し、
/// 持たない側は全要素を渡してよい）。
///
/// 判定規則はモジュール冒頭「対象とする接合部」のとおりで、モデル化と断面検定の
/// 双方がこの関数を通る。モデル化はさらに [`PanelJoint::has_filled_column`] が
/// `false` であることを要求する。
pub fn resolve_panel_joint<'a>(
    model: &Model,
    node: NodeId,
    members: impl IntoIterator<Item = &'a ElementData>,
) -> Option<PanelJoint> {
    let mut columns: Vec<&ElementData> = Vec::new();
    let mut beams: Vec<&ElementData> = Vec::new();
    for e in members {
        if !e.nodes.iter().take(2).any(|n| *n == node) {
            continue;
        }
        // 斜材はパネル自由度と連成しないため、種別判定の対象にもしない。
        match member_orientation(model, e) {
            Some(MemberOrientation::Column) => columns.push(e),
            Some(MemberOrientation::Beam) => beams.push(e),
            None => {}
        }
    }
    if columns.is_empty() || beams.is_empty() {
        return None;
    }
    // 取り付く柱・はりがすべて S/CFT 系であること。1 本でも RC/SRC が混じる
    // 接合部は、コンクリートが接合部全体を拘束するため鋼部材だけの実効体積
    // では挙動を表せない。
    if columns
        .iter()
        .chain(beams.iter())
        .any(|e| member_structure_kind(model, e) != StructureKind::Steel)
    {
        return None;
    }

    let section_of = |e: &ElementData| e.section.and_then(|sid| model.sections.get(sid.index()));
    let db = beams
        .iter()
        .filter_map(|e| section_of(e))
        .map(beam_panel_depth)
        .fold(0.0_f64, f64::max);
    if db <= 0.0 {
        return None;
    }

    // 柱が複数取り付く場合は Ve が最小になる柱を採る。要素の並び順に依存せず
    // 決定的で、かつ剛性・耐力とも安全側になる。
    let mut best: Option<PanelJoint> = None;
    let mut has_filled_column = false;
    for e in &columns {
        let Some(geometry) = section_of(e).and_then(PanelGeometry::from_column) else {
            continue;
        };
        has_filled_column |= geometry.filled;
        let ve = geometry.effective_volume(db);
        if ve <= 0.0 {
            continue;
        }
        let better = match &best {
            Some(cur) => ve < cur.ve,
            None => true,
        };
        if better {
            best = Some(PanelJoint {
                geometry,
                db,
                ve,
                column: e.id,
                has_filled_column: false,
            });
        }
    }
    best.map(|j| PanelJoint {
        has_filled_column,
        ..j
    })
}

/// 梁のフランジ板厚中心間距離 `db` [mm]。
///
/// H 形鋼は `せい − フランジ厚`、それ以外の断面は情報が無いため `0.9・せい` で
/// 近似する（S 造パネルゾーン検定と同じ近似）。
pub fn beam_panel_depth(sec: &Section) -> f64 {
    match sec.shape {
        Some(SectionShape::SteelH { flange_thick, .. }) => sec.depth - flange_thick,
        _ => 0.9 * sec.depth,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::SectionId;
    use crate::section_shape::{BarSet, RcRebar, ShearBar};

    fn sec(shape: SectionShape, depth: f64, panel_thickness: Option<f64>) -> Section {
        Section {
            id: SectionId(0),
            name: String::new(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth,
            width: depth,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness,
            thickness: None,
            shape: Some(shape),
        }
    }

    /// H 形柱: dc = せい − フランジ厚、tp = ウェブ厚、Ve = dc・db・tp。
    #[test]
    fn test_h_column_geometry() {
        let s = sec(
            SectionShape::SteelH {
                height: 400.0,
                width: 400.0,
                web_thick: 13.0,
                flange_thick: 21.0,
            },
            400.0,
            None,
        );
        let g = PanelGeometry::from_column(&s).expect("H 形は対象");
        assert!((g.dc - (400.0 - 21.0)).abs() < 1e-9);
        assert!((g.tp - 13.0).abs() < 1e-9);
        let db = 600.0;
        assert!((g.effective_volume(db) - g.dc * db * g.tp).abs() < 1e-6);
    }

    /// 角形・円形はウェブ 2 枚相当で Ve が 2 倍になる。
    #[test]
    fn test_box_and_pipe_double_volume() {
        let b = sec(
            SectionShape::SteelBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
                corner_r: 0.0,
            },
            400.0,
            None,
        );
        let g = PanelGeometry::from_column(&b).expect("角形は対象");
        let db = 500.0;
        assert!((g.effective_volume(db) - 2.0 * g.dc * db * g.tp).abs() < 1e-6);

        let p = sec(
            SectionShape::SteelPipe {
                outer_dia: 400.0,
                thick: 12.0,
            },
            400.0,
            None,
        );
        let gp = PanelGeometry::from_column(&p).expect("円形は対象");
        assert!((gp.dc - (400.0 - 12.0)).abs() < 1e-9);
        assert!((gp.effective_volume(db) - 2.0 * gp.dc * db * gp.tp).abs() < 1e-6);
        assert!((gp.kappa() - 4.0 / std::f64::consts::PI).abs() < 1e-12);
    }

    /// `panel_thickness` の明示入力は断面形状から算出した板厚より優先される
    /// （ダイアフラム補強・ダブラープレートによる増厚）。
    #[test]
    fn test_panel_thickness_overrides_shape() {
        let s = sec(
            SectionShape::SteelH {
                height: 400.0,
                width: 400.0,
                web_thick: 13.0,
                flange_thick: 21.0,
            },
            400.0,
            Some(25.0),
        );
        let g = PanelGeometry::from_column(&s).expect("H 形は対象");
        assert!((g.tp - 25.0).abs() < 1e-9, "明示入力が優先される: {}", g.tp);

        // 0 以下は未入力扱いとし、断面形状の値へフォールバックする。
        let z = sec(
            SectionShape::SteelH {
                height: 400.0,
                width: 400.0,
                web_thick: 13.0,
                flange_thick: 21.0,
            },
            400.0,
            Some(0.0),
        );
        let gz = PanelGeometry::from_column(&z).expect("H 形は対象");
        assert!((gz.tp - 13.0).abs() < 1e-9);
    }

    /// CFT は諸元を解決できるが、モデル化の対象外とする。断面検定は鋼管部を
    /// S 造と同じ式で評価するため、諸元解決自体は成功させる必要がある。
    #[test]
    fn test_cft_resolves_but_is_not_modeling_target() {
        let cases = [
            (
                SectionShape::CftBox {
                    height: 400.0,
                    width: 400.0,
                    thick: 16.0,
                },
                PanelShapeKind::Box { bc: 400.0 },
            ),
            (
                SectionShape::CftPipe {
                    outer_dia: 400.0,
                    thick: 12.0,
                },
                PanelShapeKind::Pipe,
            ),
        ];
        for (shape, kind) in cases {
            let s = sec(shape, 400.0, None);
            let g = PanelGeometry::from_column(&s).expect("CFT も諸元は解決できる");
            assert_eq!(g.kind, kind);
            assert!(g.filled, "CFT は充填断面");
            assert!(!g.is_modeling_target(), "CFT はモデル化の対象外");
            // 検定に使う Ve・κ は鋼管と同じ式で求まる。
            assert!(g.effective_volume(500.0) > 0.0);
            assert!(g.kappa() > 0.0);
        }
    }

    /// CFT でない鋼管（角形・円形）と H 形はモデル化の対象。
    #[test]
    fn test_steel_sections_are_modeling_targets() {
        let shapes = [
            SectionShape::SteelH {
                height: 400.0,
                width: 400.0,
                web_thick: 13.0,
                flange_thick: 21.0,
            },
            SectionShape::SteelBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
                corner_r: 0.0,
            },
            SectionShape::SteelPipe {
                outer_dia: 400.0,
                thick: 12.0,
            },
        ];
        for shape in shapes {
            let s = sec(shape, 400.0, None);
            let g = PanelGeometry::from_column(&s).expect("S 造は対象");
            assert!(!g.filled);
            assert!(g.is_modeling_target());
        }
    }

    /// CFT と対応する鋼管は、断面検定に使う Ve・κ が同一になる
    /// （CFT を検定対象から外していないことの裏付け）。
    #[test]
    fn test_cft_and_steel_tube_share_check_properties() {
        let steel = sec(
            SectionShape::SteelBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
                corner_r: 0.0,
            },
            400.0,
            None,
        );
        let cft = sec(
            SectionShape::CftBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
            },
            400.0,
            None,
        );
        let (gs, gc) = (
            PanelGeometry::from_column(&steel).expect("角形"),
            PanelGeometry::from_column(&cft).expect("CFT 角形"),
        );
        assert!((gs.dc - gc.dc).abs() < 1e-12);
        assert!((gs.tp - gc.tp).abs() < 1e-12);
        assert!((gs.kappa() - gc.kappa()).abs() < 1e-12);
        assert!((gs.effective_volume(500.0) - gc.effective_volume(500.0)).abs() < 1e-9);
    }

    /// RC 柱はパネルの対象外（剛域と RC 柱梁接合部検定で扱う）。
    #[test]
    fn test_rc_column_is_not_panel_target() {
        use crate::section_shape::{BarSet, RcRebar, ShearBar};
        let bars = BarSet {
            dia: 25.0,
            count: 4,
            layers: 1,
        };
        let s = sec(
            SectionShape::RcRect {
                b: 700.0,
                d: 700.0,
                rebar: RcRebar {
                    main_x: bars.clone(),
                    main_y: bars,
                    cover: 40.0,
                    shear: ShearBar {
                        dia: 10.0,
                        pitch: 100.0,
                        legs: 2,
                        grade: None,
                    },
                    main_grade: None,
                },
            },
            700.0,
            None,
        );
        assert!(PanelGeometry::from_column(&s).is_none());
    }

    // ===== 接合部の判定（モデル化・断面検定の共通規則）=====

    fn node(id: u32, coord: [f64; 3]) -> crate::model::Node {
        crate::model::Node {
            id: NodeId(id),
            coord,
            restraint: crate::dof::Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn member(id: u32, n0: u32, n1: u32, sec: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
            section: Some(crate::ids::SectionId(sec)),
            material: Some(crate::ids::MaterialId(0)),
            local_axis: crate::model::LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [
                crate::model::EndCondition::Fixed,
                crate::model::EndCondition::Fixed,
            ],
            force_regime: crate::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    fn h_col() -> SectionShape {
        SectionShape::SteelH {
            height: 400.0,
            width: 400.0,
            web_thick: 13.0,
            flange_thick: 21.0,
        }
    }

    fn h_beam() -> SectionShape {
        SectionShape::SteelH {
            height: 600.0,
            width: 200.0,
            web_thick: 11.0,
            flange_thick: 17.0,
        }
    }

    fn rc_shape() -> SectionShape {
        let bars = BarSet {
            dia: 25.0,
            count: 4,
            layers: 1,
        };
        SectionShape::RcRect {
            b: 400.0,
            d: 700.0,
            rebar: RcRebar {
                main_x: bars.clone(),
                main_y: bars,
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
                main_grade: None,
            },
        }
    }

    /// 節点 0 を接合部とする T 型（梁 1 本・柱 1 本）のモデル。
    /// 断面 0 が梁、断面 1 が柱。
    fn joint_model(beam: SectionShape, beam_depth: f64, col: SectionShape) -> Model {
        Model {
            nodes: vec![
                node(0, [0.0, 0.0, 3000.0]),
                node(1, [6000.0, 0.0, 3000.0]),
                node(2, [0.0, 0.0, 0.0]),
            ],
            sections: vec![sec(beam, beam_depth, None), sec(col, 400.0, None)],
            materials: vec![crate::model::Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: crate::ids::MaterialId(0),
                name: "SN400B".into(),
                young: 205_000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            elements: vec![member(0, 0, 1, 0), member(1, 2, 0, 1)],
            ..Default::default()
        }
    }

    /// 柱・はりがすべて S なら対象になる。
    #[test]
    fn test_all_steel_joint_is_target() {
        let m = joint_model(h_beam(), 600.0, h_col());
        let joint = resolve_panel_joint(&m, NodeId(0), &m.elements).expect("S 造接合部");
        assert!((joint.db - (600.0 - 17.0)).abs() < 1e-9);
        assert_eq!(joint.column, ElemId(1));
        assert!(!joint.has_filled_column);
    }

    /// RC 梁が 1 本でもあれば対象外（柱が S でも接合部は RC になる）。
    #[test]
    fn test_rc_beam_disqualifies_joint() {
        let m = joint_model(rc_shape(), 700.0, h_col());
        assert!(resolve_panel_joint(&m, NodeId(0), &m.elements).is_none());
    }

    /// RC 柱が 1 本でもあれば対象外。
    #[test]
    fn test_rc_column_disqualifies_joint() {
        let m = joint_model(h_beam(), 600.0, rc_shape());
        assert!(resolve_panel_joint(&m, NodeId(0), &m.elements).is_none());
    }

    /// 柱だけ・はりだけの節点は対象外。
    #[test]
    fn test_column_or_beam_only_node_is_not_target() {
        let mut m = joint_model(h_beam(), 600.0, h_col());
        let beam_only = {
            let mut mm = m.clone();
            mm.elements.retain(|e| e.id != ElemId(1));
            mm
        };
        assert!(resolve_panel_joint(&beam_only, NodeId(0), &beam_only.elements).is_none());
        m.elements.retain(|e| e.id != ElemId(0));
        assert!(resolve_panel_joint(&m, NodeId(0), &m.elements).is_none());
    }

    /// 柱が複数あれば Ve が最小の柱を採り、要素の並び順に依存しない。
    #[test]
    fn test_smallest_ve_column_is_selected() {
        let thin = SectionShape::SteelH {
            height: 400.0,
            width: 400.0,
            web_thick: 9.0,
            flange_thick: 21.0,
        };
        let build = |upper_first: bool| {
            let mut m = joint_model(h_beam(), 600.0, h_col());
            m.nodes.push(node(3, [0.0, 0.0, 6000.0]));
            m.sections.push(sec(thin.clone(), 400.0, None));
            let upper = member(2, 0, 3, 2);
            if upper_first {
                m.elements.insert(0, upper);
            } else {
                m.elements.push(upper);
            }
            m
        };
        let a = build(true);
        let b = build(false);
        let ja = resolve_panel_joint(&a, NodeId(0), &a.elements).expect("接合部");
        let jb = resolve_panel_joint(&b, NodeId(0), &b.elements).expect("接合部");
        assert!((ja.geometry.tp - 9.0).abs() < 1e-9, "Ve 最小の柱を採る");
        assert_eq!(ja.ve, jb.ve, "要素の並び順に依存しない");
        assert_eq!(ja.column, jb.column);
    }

    /// CFT 柱の接合部は解決できるが、モデル化からは除外する目印が立つ。
    #[test]
    fn test_cft_column_flags_filled() {
        let m = joint_model(
            h_beam(),
            600.0,
            SectionShape::CftBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
            },
        );
        let joint = resolve_panel_joint(&m, NodeId(0), &m.elements).expect("検定の対象にはなる");
        assert!(joint.has_filled_column, "モデル化からは除外する");
    }

    // ===== 接合部の半寸法 =====

    /// 半寸法は柱せい・梁せいの 1/2。危険断面位置（face）は参照しない。
    #[test]
    fn test_panel_half_extent_uses_member_depths() {
        let mut m = joint_model(h_beam(), 600.0, h_col());
        for e in &mut m.elements {
            e.rigid_zone.face_i = 9999.0;
            e.rigid_zone.face_j = 9999.0;
        }
        let extent = panel_half_extent(&m, NodeId(0), &m.elements);
        assert!((extent.column_half - 200.0).abs() < 1e-9);
        assert!((extent.beam_half - 300.0).abs() < 1e-9);
        assert!((extent.offset_for(MemberOrientation::Beam) - 200.0).abs() < 1e-9);
        assert!((extent.offset_for(MemberOrientation::Column) - 300.0).abs() < 1e-9);
    }

    /// 斜材は柱にもはりにも分類しない（オフセット・ζ が資料で定義されないため）。
    #[test]
    fn test_diagonal_member_has_no_orientation() {
        let mut m = joint_model(h_beam(), 600.0, h_col());
        m.nodes[1].coord = [4000.0, 0.0, 6000.0];
        assert!(member_orientation(&m, &m.elements[0]).is_none());
    }

    /// 梁の db: H 形はせい − フランジ厚、それ以外は 0.9・せい。
    #[test]
    fn test_beam_panel_depth() {
        let h = sec(
            SectionShape::SteelH {
                height: 600.0,
                width: 200.0,
                web_thick: 11.0,
                flange_thick: 17.0,
            },
            600.0,
            None,
        );
        assert!((beam_panel_depth(&h) - (600.0 - 17.0)).abs() < 1e-9);

        let b = sec(
            SectionShape::SteelBox {
                height: 500.0,
                width: 300.0,
                thick: 12.0,
                corner_r: 0.0,
            },
            500.0,
            None,
        );
        assert!((beam_panel_depth(&b) - 0.9 * 500.0).abs() < 1e-9);
    }
}
