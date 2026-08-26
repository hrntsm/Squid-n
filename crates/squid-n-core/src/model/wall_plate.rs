//! 壁版（`WallPlate`）。
//!
//! 壁領域（[`WallRegion`]、[`super::wall`]）は柱・梁が囲む鉛直構面内の閉領域そのもので、
//! 版の仕様は持たない。版の仕様（断面・開口）は本モジュールの `WallPlate` が持つ。
//! 1 つの壁領域は、区画内が間柱でさらに細かい壁パネルに分かれていれば複数の
//! `WallPlate` を持ちうる（[`WallRegion::wall_plate_ids`]。E5。床側の `FloorRegion`/
//! [`super::Slab`] と同じ関係）。パラペット・腰壁・垂れ壁・自立壁はどの壁領域からも
//! 参照されない、独立した `WallPlate` として存在する（`OutOfFrameMiscWall` の後継）。
//!
//! # 参入レベル（構造壁・n倍法・重量のみ）は型で区別しない
//!
//! 壁が解析にどう参入するか（4 節点要素として剛性・保有水平耐力に算入する「構造壁」、
//! n倍法で偏心率にのみ寄与する「雑壁剛性」、自重のみの「重量のみ」）は、`WallPlate`
//! 自身に列挙型を持たせて利用者に選ばせるのではなく、既存の暗黙規則をそのまま踏襲する
//! （dig Q4=B）。**`section` の有無と、所属する `WallRegion` の種別（囲まれた領域か
//! 取り付き領域か）の組み合わせで、生成ロジック（Step 8・D5）側が決める。**
//!
//! # 自重は必ず断面参照から求める（`OutOfFrameMiscWall` との相違点）
//!
//! 現行 `OutOfFrameMiscWall` は断面を介さず `weight_per_area`（直接入力）と
//! `thickness`（直接入力、n倍法用）を自前で持つ。`WallPlate` はこれを踏襲しない
//! （dig Q5=A）。[`super::Slab`]/[`super::SlabPlate`] と同じく、自重は必ず `section`
//! （厚さ・主材料）から求める。断面未割当は自重 0 とし、解析前チェックが止める
//! （既定厚で補わない）。`OutOfFrameMiscWall` の直接入力経路は、ST-Bridge 取り込みに
//! 生成コードが存在せず実データが 0 件（単体テストの合成データのみ）だったため、
//! 移行対象の実利用がないと判断して廃止した。

use super::*;

/// 壁版の形。[`super::SlabShape`] と同型（囲まれた領域 / 主架構・床領域に取り付く領域）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum WallPlateShape {
    /// 柱・梁が囲む鉛直構面内の領域。境界は [`super::WallRegion`] の境界そのもの、
    /// または間柱で分割した場合はそのサブ境界（節点列。反時計回り、始点は繰り返さない）。
    Enclosed { boundary: Vec<NodeId> },
    /// 主架構・床領域に取り付く領域（パラペット・腰壁・垂れ壁・自立壁）。
    ///
    /// [`RegionAnchor::Line`] の場合、`extent` は D15 の「立ち上がり高さ」
    /// `[d_i, d_j]`（区間の始端側・終端側の高さ [mm]）で、床側（跳ね出し長さ）とは
    /// 張り出す向きが異なる（床は取付き線の左向き法線方向、壁は鉛直上向き）。
    /// [`RegionAnchor::FloorRegion`] の場合も同じ意味（`extent` は高さ、`nodes` は
    /// 壁自体の平面上の始点・終点）。[`RegionAnchor::Point`] は壁の取付き先としては
    /// 使わない（D14 の対応表に壁の用例がなく、出隅スラブ専用のため。
    /// [`WallPlate::boundary_coords`] はこの組み合わせで `None` を返す）。
    Attached {
        anchor: RegionAnchor,
        extent: [f64; 2],
    },
}

/// 壁版。柱・梁が囲む鉛直構面内の版、または主架構・床領域に取り付く版
/// （パラペット・腰壁・垂れ壁・自立壁）ごとに 1 つ。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WallPlate {
    /// 壁版 ID（`Model::wall_plates` の配列インデックスと一致すること）。
    pub id: WallPlateId,
    pub shape: WallPlateShape,
    /// 断面（板厚・材料・開口低減の解決元）。`None` は未割当（自重 0。解析前
    /// チェックが止める。モジュール doc 参照）。
    #[serde(default)]
    pub section: Option<SectionId>,
    /// 個別開口の寸法リスト（[`super::WallOpening`]）。自重控除・開口周比・
    /// 耐震壁検定の開口供給に使う。構造壁でない壁版（n倍法・重量のみ）でも、
    /// 自重控除には意味を持つため共通で持たせる。
    #[serde(default)]
    pub openings: Vec<WallOpening>,
    /// 三方スリット。true の場合、自重は上下分配せず全て上部の節点へ伝達する。
    /// 要素生成される構造壁のときのみ意味を持つ（[`super::WallRegion`] が
    /// 「囲まれた領域」で、かつ `section` 割当ありの場合）。
    #[serde(default)]
    pub three_side_slit: bool,
}

impl WallPlate {
    /// 取り付く壁版か。
    pub fn is_attached(&self) -> bool {
        matches!(self.shape, WallPlateShape::Attached { .. })
    }

    /// 境界の節点列。**柱・梁が囲む壁版のみ**（取り付く壁版は自由端に節点を
    /// 持たないため `None`）。
    pub fn boundary_nodes(&self) -> Option<&[NodeId]> {
        match &self.shape {
            WallPlateShape::Enclosed { boundary } => Some(boundary),
            WallPlateShape::Attached { .. } => None,
        }
    }

    /// 境界多角形の座標列 [mm]（4 点）。取り付く壁版は取付き先と張り出し量
    /// （鉛直上向きの高さ）から算出する。節点が引けない、または壁の取付き先として
    /// 使わない組み合わせ（`RegionAnchor::Point`）の場合は `None`。
    pub fn boundary_coords(&self, model: &Model) -> Option<Vec<[f64; 3]>> {
        match &self.shape {
            WallPlateShape::Enclosed { boundary } => boundary
                .iter()
                .map(|n| model.nodes.get(n.index()).map(|n| n.coord))
                .collect(),
            WallPlateShape::Attached { anchor, extent } => match anchor {
                RegionAnchor::Line { nodes, span, .. } => {
                    let a = model.nodes.get(nodes[0].index())?.coord;
                    let b = model.nodes.get(nodes[1].index())?.coord;
                    Self::extrude_up(a, b, *span, *extent)
                }
                RegionAnchor::FloorRegion { nodes, .. } => {
                    let a = model.nodes.get(nodes[0].index())?.coord;
                    let b = model.nodes.get(nodes[1].index())?.coord;
                    Self::extrude_up(a, b, [0.0, 1.0], *extent)
                }
                // 壁の取付き先としては使わない（モジュール doc 参照）。
                RegionAnchor::Point(_) => None,
            },
        }
    }

    /// 取付き線（`a`→`b`）の無次元区間 `span` を底辺とし、両端の高さ `extent`
    /// （鉛直上向き）だけ立ち上げた 4 点（反時計回り）を返す。
    fn extrude_up(
        a: [f64; 3],
        b: [f64; 3],
        span: [f64; 2],
        extent: [f64; 2],
    ) -> Option<Vec<[f64; 3]>> {
        let lerp = |t: f64| {
            [
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            ]
        };
        let p0 = lerp(span[0]);
        let p1 = lerp(span[1]);
        Some(vec![
            p0,
            p1,
            [p1[0], p1[1], p1[2] + extent[1]],
            [p0[0], p0[1], p0[2] + extent[0]],
        ])
    }

    /// 壁版の面積 [mm²]（[`crate::geom::polygon_area_3d`]。ニューエルの公式による
    /// 3 次元面積。理想平面への投影を経由しない。壁・シェル要素の自重算定と
    /// スラブ・壁の数量拾いが共通で使う関数をそのまま使う）。座標が引けない場合は 0。
    pub fn area(&self, model: &Model) -> f64 {
        self.boundary_coords(model)
            .map(|pts| crate::geom::polygon_area_3d(&pts))
            .unwrap_or(0.0)
    }

    /// 開口の合計面積 [mm²]（[`WallOpening::area`] の和）。
    pub fn total_opening_area(&self) -> f64 {
        self.openings.iter().map(WallOpening::area).sum()
    }
}

impl Model {
    /// 壁版 ID から壁版を引く。存在しなければ `None`。
    pub fn wall_plate(&self, id: WallPlateId) -> Option<&WallPlate> {
        match self.wall_plates.get(id.index()) {
            Some(p) if p.id == id => Some(p),
            _ => self.wall_plates.iter().find(|p| p.id == id),
        }
    }

    /// 壁版へ割り当てた断面。未割当・ダングリングは `None`。
    pub fn wall_plate_section(&self, plate: &WallPlate) -> Option<&Section> {
        plate.section.and_then(|sid| self.sections.get(sid.index()))
    }

    /// 壁版の板厚 [mm]（断面の [`Section::thickness`]。[`Model::slab_plate_thickness`]
    /// と同じ規約）。断面未割当、または断面が板厚を持たない場合は `None`。
    pub fn wall_plate_thickness(&self, plate: &WallPlate) -> Option<f64> {
        self.wall_plate_section(plate)
            .and_then(|s| s.thickness)
            .filter(|t| *t > 0.0)
    }

    /// 壁版の主材料。断面未割当・材料未割当は `None`。
    pub fn wall_plate_material(&self, plate: &WallPlate) -> Option<&Material> {
        self.wall_plate_section(plate)
            .and_then(|s| s.material)
            .and_then(|mid| self.materials.get(mid.index()))
    }

    /// 壁版の自重 [N]（開口控除後の正味面積 × 板厚 × 主材料の密度 × 重力加速度）。
    ///
    /// 断面または主材料が未割当のときは `None`（[`Model::slab_self_weight_intensity`]
    /// と同じ規約。既定厚で補わない）。開口面積が正味面積を超える場合は 0 に丸める。
    pub fn wall_plate_self_weight(&self, plate: &WallPlate, model_for_area: &Model) -> Option<f64> {
        let t = self.wall_plate_thickness(plate)?;
        let mat = self.wall_plate_material(plate)?;
        let area = plate.area(model_for_area);
        let net_area = (area - plate.total_opening_area()).max(0.0);
        Some(t * mat.density * net_area * crate::units::GRAVITY_MM_S2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FloorRegionId, MaterialId, NodeId, SectionId, WallPlateId};

    fn model_with_nodes(pts: &[[f64; 3]]) -> Model {
        let mut m = Model::default();
        for (i, p) in pts.iter().enumerate() {
            m.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *p,
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        m
    }

    #[test]
    fn test_enclosed_boundary_coords_and_area() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: None,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let coords = p.boundary_coords(&m).expect("境界座標");
        assert_eq!(coords.len(), 4);
        assert!((p.area(&m) - 4000.0 * 3000.0).abs() < 1e-6);
    }

    /// 取付き線アンカーは、床の取り付く床板（左向き法線方向へ張り出す）とは異なり、
    /// 鉛直上向きへ立ち上げる（D15「壁は立ち上がり高さ」）。
    #[test]
    fn test_attached_line_extrudes_upward_not_sideways() {
        let m = model_with_nodes(&[[0.0, 0.0, 3000.0], [4000.0, 0.0, 3000.0]]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: [900.0, 900.0],
            },
            section: None,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let coords = p.boundary_coords(&m).expect("境界座標");
        // 4点とも Y=0（左向き法線方向へは動かない）、上 2 点は Z=3900（+900 立ち上げ）。
        for c in &coords {
            assert_eq!(c[1], 0.0, "Y 方向へは張り出さない: {coords:?}");
        }
        assert_eq!(coords[2][2], 3900.0);
        assert_eq!(coords[3][2], 3900.0);
        assert!((p.area(&m) - 4000.0 * 900.0).abs() < 1e-6);
    }

    /// 床領域アンカーは、アンカー自身が持つ節点対を壁の平面上の始点・終点として使う
    /// （dig Q6=B）。所属先の床領域 ID は幾何計算には関与しない。
    #[test]
    fn test_attached_floor_region_anchor_uses_its_own_nodes_for_length() {
        let m = model_with_nodes(&[[0.0, 0.0, 3000.0], [2000.0, 0.0, 3000.0]]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    region: FloorRegionId(0),
                    nodes: [NodeId(0), NodeId(1)],
                },
                extent: [2500.0, 2500.0],
            },
            section: None,
            openings: Vec::new(),
            three_side_slit: false,
        };
        assert!((p.area(&m) - 2000.0 * 2500.0).abs() < 1e-6);
    }

    /// 壁の取付き先として `RegionAnchor::Point` は使わない（D14 の対応表参照）。
    #[test]
    fn test_attached_point_anchor_is_unsupported_for_wall() {
        let m = model_with_nodes(&[[0.0, 0.0, 3000.0]]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Point(NodeId(0)),
                extent: [900.0, 900.0],
            },
            section: None,
            openings: Vec::new(),
            three_side_slit: false,
        };
        assert_eq!(p.boundary_coords(&m), None);
        assert_eq!(p.area(&m), 0.0);
    }

    #[test]
    fn test_self_weight_none_without_section() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: None,
            openings: Vec::new(),
            three_side_slit: false,
        };
        assert_eq!(m.wall_plate_self_weight(&p, &m), None);
    }

    #[test]
    fn test_self_weight_deducts_opening_area() {
        let mut m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        m.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        m.sections.push(Section {
            id: SectionId(0),
            name: "壁 t150".into(),
            area: 150.0 * 3000.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 3000.0,
            width: 150.0,
            as_y: 1.0,
            as_z: 1.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: Some(SectionId(0)),
            openings: vec![WallOpening {
                width: 900.0,
                height: 1200.0,
                offset: Some([1550.0, 0.0]),
            }],
            three_side_slit: false,
        };
        let gross_area = 4000.0 * 3000.0;
        let opening_area = 900.0 * 1200.0;
        let expected = 150.0 * 2.4e-9 * (gross_area - opening_area) * crate::units::GRAVITY_MM_S2;
        let w = m.wall_plate_self_weight(&p, &m).expect("自重が求まる");
        assert!(
            (w - expected).abs() / expected < 1e-9,
            "自重 {w}（期待値 {expected}）"
        );
    }
}
