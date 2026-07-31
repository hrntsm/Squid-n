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
//! # 対象とする柱断面
//!
//! 諸元を解決できるのは H 形鋼・角形鋼管・円形鋼管・CFT（角形・円形）である。
//! RC・SRC 柱は [`PanelGeometry::from_column`] が `None` を返し、パネルのモデル化・
//! S 造パネル検定のいずれの対象にもならない（RC・SRC の接合部は剛域と各構造の
//! 接合部検定で扱う）。組立 H 形（`SteelBuiltH`）は上下フランジが異なりパネル
//! 形状係数 κ の標準式が適用できないため対象外とする。
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
//! [`PanelGeometry::is_modeling_target`] で除外する。理由は同メソッドを参照。

use crate::model::Section;
use crate::section_shape::SectionShape;

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
