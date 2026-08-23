//! 床領域（[`FloorRegion`]）と、その版（[`SlabPlate`]）。
//!
//! 床領域は「主架構が囲む領域」または「主架構に取り付く領域」であり、版（床スラブ）と
//! 二次部材（小梁）をまとめる単位である。設計の経緯と決定事項は
//! `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` を参照。
//!
//! # 2 つの形
//!
//! - [`RegionShape::Enclosed`] — 大梁が囲む閉領域（パネル）。境界は節点列で、
//!   [`crate::region_gen`] の面走査から作る。**1 つの閉領域につき 1 つ**とする。
//! - [`RegionShape::Attached`] — 主架構に取り付く領域（片持ちスラブ・バルコニー・出隅）。
//!   主架構に囲まれておらず、張り出し量は主架構から導けない利用者データである。
//!   数の制限は置かない。
//!
//! # 版は任意
//!
//! [`FloorRegion::plate`] は `None` を取りうる（版なし床領域）。吹抜けに架かる繋ぎ小梁や
//! 階段室まわりのように、床版がなくても小梁をまとめる単位は実在する。
//! 版がなければ床荷重は発生しない。

use super::*;

/// 床領域の形。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum RegionShape {
    /// 大梁が囲む閉領域。境界は反時計回りの節点列（始点は繰り返さない）。
    Enclosed { boundary: Vec<NodeId> },
    /// 主架構に取り付く領域（片持ち・バルコニー・出隅）。
    Attached {
        /// 取付き先。荷重の出口は取付き先の種類ごとに決まる（[`RegionAnchor`] 参照）。
        anchor: RegionAnchor,
        /// 張り出し量 [mm]。意味は取付き先の種類で決まる（[`RegionAnchor`] 参照）。
        extent: [f64; 2],
    },
}

/// 取り付き領域の取付き先。
///
/// 節点で指すのは、節点を動かしても追随し、取付き先の大梁を分割しても外れないためである
/// （区間は節点対の間の相対位置なので、間に節点が増えても変わらない）。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum RegionAnchor {
    /// 線に取り付く（片持ちスラブ・バルコニー）。
    ///
    /// `nodes` は取付き線の両端、`span` はその線上の無次元区間 `[t_i, t_j]`（0.0〜1.0、
    /// 全長は `[0.0, 1.0]`）。梁の一部だけに載る場合に用いる。
    ///
    /// 張り出し量 `extent` は `[d_i, d_j]`（区間の始端側・終端側）で、
    /// **符号は取付き線 `nodes[0]`→`nodes[1]` の左側を正とする**。
    ///
    /// 荷重の出口（`transfer`）を選べるのはこの形だけである。点に取り付く領域は
    /// その節点への集中しかありえないため、値を持たせると無意味な組み合わせを
    /// 表現できてしまう。
    Line {
        nodes: [NodeId; 2],
        span: [f64; 2],
        transfer: LoadTransfer,
    },
    /// 点（柱）に取り付く（出隅の片持ちスラブ）。荷重はその節点へ集中する。
    ///
    /// 張り出し量 `extent` は全体座標の `[X 方向, Y 方向]` で、符号が向きを表す。
    Point(NodeId),
}

/// 取り付き線に載る領域の荷重の出口（[`RegionAnchor::Line`] が持つ）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LoadTransfer {
    /// 取付き線へ分布させる（既定。片持ちスラブ・梁に載るパラペット）。
    #[default]
    Anchor,
    /// 取付き線の両端（柱）へ集中させる（出隅・雑壁の柱伝達）。
    Columns,
}

/// 床領域の版（床スラブ）。
///
/// 幾何は領域が持つため、ここには版としての仕様だけを置く。
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SlabPlate {
    /// スラブ断面（符号・板厚・コンクリート材料）。
    ///
    /// **板厚と自重はこの断面から解決する**（[`Model::region_thickness`]・
    /// [`Model::region_self_weight_intensity`]）。板厚を領域と断面の両方に持たせると
    /// 同じ数値の持ち主が 2 つになるため、領域側は断面を指すだけとする。
    ///
    /// `None` は未割当。板厚も自重も定まらないため、解析前チェックが止める
    /// （もっともらしい既定厚で補うと、床の自重が過小なまま長期応力が出る）。
    #[serde(default)]
    pub section: Option<crate::ids::SectionId>,
    /// 仕上げ等の面荷重。
    #[serde(default)]
    pub loads: Vec<AreaLoad>,
    /// 室用途（令別表第1）。`Some` のとき積載荷重（LL）を用途別に自動算定する。
    /// `None` は積載荷重を持たない（`loads` の固定荷重のみ）。
    #[serde(default)]
    pub usage: Option<SlabUsage>,
    /// 床荷重の分配方法。
    pub method: DistributionMethod,
    /// 一方向スラブの伝達方向。`None` は境界辺 0・2 が負担する既定。
    #[serde(default)]
    pub one_way: Option<OneWayDir>,
    /// 小梁ライン（方向とピッチのパラメトリック表現）。
    ///
    /// **廃止予定**。小梁の実体は領域が持つ二次部材へ一本化し、床荷重の分配は
    /// パネルを小梁で再分割して行う（申し送りの D9・Step 4）。それまでの間、
    /// 矩形スラブの二段階伝達と床格子サブモデルがこのリストを使う。
    #[serde(default)]
    pub joists: Vec<JoistLine>,
}

impl SlabPlate {
    /// 仕上げ等の面荷重強度 [N/mm²]（`loads` の合算）。**版自身の自重は含まない。**
    pub fn finish_intensity(&self) -> f64 {
        self.loads.iter().map(|l| l.value).sum()
    }

    /// 用途別の積載荷重（LL）の面荷重強度 [N/mm²]。`usage` 未設定なら 0。
    pub fn live_intensity(&self, purpose: LoadPurpose) -> f64 {
        self.usage.map(|u| u.live_load(purpose)).unwrap_or(0.0)
    }
}

/// 床領域。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FloorRegion {
    /// 床領域 ID（`Model::floor_regions` の配列インデックスと一致すること）。
    pub id: FloorRegionId,
    /// 表示名（ナビゲータ・診断で領域を指し示すために用いる）。空文字は名前なし。
    #[serde(default)]
    pub name: String,
    /// 領域の形（囲まれた領域 / 取り付き領域）。
    pub shape: RegionShape,
    /// 版。`None` は版なし床領域（床荷重は発生しない）。
    #[serde(default)]
    pub plate: Option<SlabPlate>,
    /// この床領域に属する小梁（`SecondaryMember::Joist`）の ID リスト。
    /// リスト内の順序は任意。重複は許可しない（`Model::validate` が確認）。
    #[serde(default)]
    pub secondary_joist_ids: Vec<SecondaryMemberId>,
}

impl FloorRegion {
    /// 囲まれた領域を作る（版なし）。
    pub fn enclosed(id: FloorRegionId, boundary: Vec<NodeId>) -> Self {
        FloorRegion {
            id,
            name: String::new(),
            shape: RegionShape::Enclosed { boundary },
            plate: None,
            secondary_joist_ids: Vec::new(),
        }
    }

    /// 版を差し替える（ビルダ）。
    pub fn with_plate(mut self, plate: SlabPlate) -> Self {
        self.plate = Some(plate);
        self
    }

    /// 取り付き領域か。
    pub fn is_attached(&self) -> bool {
        matches!(self.shape, RegionShape::Attached { .. })
    }

    /// 境界の節点列。**囲まれた領域のみ**（取り付き領域は自由端に節点を持たないため `None`）。
    ///
    /// 節点 ID そのものが要る用途（ST-Bridge 出力・床格子サブモデル）だけがこれを使い、
    /// 幾何が要るだけの用途は [`FloorRegion::boundary_coords`] を使うこと。
    pub fn boundary_nodes(&self) -> Option<&[NodeId]> {
        match &self.shape {
            RegionShape::Enclosed { boundary } => Some(boundary),
            RegionShape::Attached { .. } => None,
        }
    }

    /// 境界多角形の座標列 [mm]。取り付き領域は取付き先と張り出し量から算出する。
    ///
    /// 節点が引けない（陳腐化した参照）場合は `None`。
    pub fn boundary_coords(&self, model: &Model) -> Option<Vec<[f64; 3]>> {
        match &self.shape {
            RegionShape::Enclosed { boundary } => boundary
                .iter()
                .map(|n| model.nodes.get(n.index()).map(|n| n.coord))
                .collect(),
            RegionShape::Attached { anchor, extent } => match anchor {
                RegionAnchor::Line { nodes, span, .. } => {
                    let a = model.nodes.get(nodes[0].index())?.coord;
                    let b = model.nodes.get(nodes[1].index())?.coord;
                    let lerp = |t: f64| {
                        [
                            a[0] + (b[0] - a[0]) * t,
                            a[1] + (b[1] - a[1]) * t,
                            a[2] + (b[2] - a[2]) * t,
                        ]
                    };
                    let p0 = lerp(span[0]);
                    let p1 = lerp(span[1]);
                    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                    let len = (dx * dx + dy * dy).sqrt();
                    if len <= f64::EPSILON {
                        return None;
                    }
                    // 取付き線の左向き単位法線（符号つき張り出し量の正の向き）。
                    let n = [-dy / len, dx / len];
                    Some(vec![
                        p0,
                        p1,
                        [p1[0] + n[0] * extent[1], p1[1] + n[1] * extent[1], p1[2]],
                        [p0[0] + n[0] * extent[0], p0[1] + n[1] * extent[0], p0[2]],
                    ])
                }
                RegionAnchor::Point(nid) => {
                    let p = model.nodes.get(nid.index())?.coord;
                    Some(vec![
                        p,
                        [p[0] + extent[0], p[1], p[2]],
                        [p[0] + extent[0], p[1] + extent[1], p[2]],
                        [p[0], p[1] + extent[1], p[2]],
                    ])
                }
            },
        }
    }

    /// 境界の辺 `k` の両端節点。荷重分配の結果（`LoadTarget::Edge`）を主架構の梁へ
    /// 結びつけるために使う。
    ///
    /// 囲まれた領域は `boundary[k]`→`boundary[k+1]`、取り付き領域（線）は辺 0 が
    /// 取付き線そのもの（それ以外の辺は自由端なので `None`）。取り付き領域（点）は
    /// 辺を持たない。
    pub fn edge_nodes(&self, k: usize) -> Option<[NodeId; 2]> {
        match &self.shape {
            RegionShape::Enclosed { boundary } => {
                let n = boundary.len();
                (n >= 3 && k < n).then(|| [boundary[k], boundary[(k + 1) % n]])
            }
            RegionShape::Attached { anchor, .. } => match anchor {
                RegionAnchor::Line { nodes, .. } if k == 0 => Some(*nodes),
                _ => None,
            },
        }
    }

    /// 領域を代表する節点。囲まれた領域は境界の先頭、取り付き領域は取付き先の節点。
    ///
    /// 階の帰属や診断の表示など、「この領域はどこにあるか」を 1 点で示す用途に使う。
    pub fn reference_node(&self) -> Option<NodeId> {
        match &self.shape {
            RegionShape::Enclosed { boundary } => boundary.first().copied(),
            RegionShape::Attached { anchor, .. } => Some(match anchor {
                RegionAnchor::Line { nodes, .. } => nodes[0],
                RegionAnchor::Point(n) => *n,
            }),
        }
    }

    /// 領域のレベル Z [mm]（境界座標の Z の平均）。境界が引けなければ `None`。
    pub fn level(&self, model: &Model) -> Option<f64> {
        let coords = self.boundary_coords(model)?;
        if coords.is_empty() {
            return None;
        }
        Some(coords.iter().map(|c| c[2]).sum::<f64>() / coords.len() as f64)
    }

    /// 版が持つ小梁ライン（版なしなら空）。廃止予定（[`SlabPlate::joists`] 参照）。
    pub fn joist_lines(&self) -> &[JoistLine] {
        self.plate
            .as_ref()
            .map(|p| p.joists.as_slice())
            .unwrap_or(&[])
    }

    /// 分配方法（版なしは既定の三角形・台形分配を返す。荷重が 0 なので分配は起きない）。
    pub fn method(&self) -> DistributionMethod {
        self.plate
            .as_ref()
            .map(|p| p.method)
            .unwrap_or(DistributionMethod::TriTrapezoid)
    }

    /// 一方向スラブの伝達方向（版なしは `None`）。
    pub fn one_way(&self) -> Option<OneWayDir> {
        self.plate.as_ref().and_then(|p| p.one_way)
    }

    /// 用途別の積載荷重（LL）の面荷重強度 [N/mm²]。版なし・用途未設定は 0。
    pub fn live_intensity(&self, purpose: LoadPurpose) -> f64 {
        self.plate
            .as_ref()
            .map(|p| p.live_intensity(purpose))
            .unwrap_or(0.0)
    }

    /// 版の断面（版なし・未割当は `None`）。
    pub fn section(&self) -> Option<crate::ids::SectionId> {
        self.plate.as_ref().and_then(|p| p.section)
    }

    /// 室用途（版なし・未設定は `None`）。
    pub fn usage(&self) -> Option<SlabUsage> {
        self.plate.as_ref().and_then(|p| p.usage)
    }
}

impl Model {
    /// 床領域の版へ割り当てた断面。未割当・ダングリングは `None`。
    pub fn region_section(&self, region: &FloorRegion) -> Option<&Section> {
        region
            .section()
            .and_then(|sid| self.sections.get(sid.index()))
    }

    /// 版の板厚 [mm]。断面の [`Section::thickness`] をそのまま返す。
    ///
    /// 断面が未割当、または断面が板厚を持たない（板状でない形状を割り当てた）場合は `None`。
    /// **建物一律の [`Model::slab_thickness`] へは退かない**。あちらは「剛性計算に見込む
    /// スラブ厚」であり、既定の 0 は「スラブ協力幅による梁剛性増大を見込まない」を意味する
    /// 別概念のためである。
    pub fn region_thickness(&self, region: &FloorRegion) -> Option<f64> {
        self.region_section(region)
            .and_then(|s| s.thickness)
            .filter(|t| *t > 0.0)
    }

    /// 版の自重の面荷重強度 [N/mm²]（板厚 × 断面の主材料の単位体積重量）。
    ///
    /// 断面または断面の主材料が未割当のときは `None`。自重を面荷重として焼き込まず
    /// 毎回算定するのは、板厚や材料を変えたときに自重が追随しないという食い違いを
    /// 作らないためである。
    pub fn region_self_weight_intensity(&self, region: &FloorRegion) -> Option<f64> {
        let t = self.region_thickness(region)?;
        let mat = self
            .region_section(region)
            .and_then(|s| s.material)
            .and_then(|mid| self.materials.get(mid.index()))?;
        Some(t * mat.density * crate::units::GRAVITY_MM_S2)
    }

    /// 固定荷重（DL）の面荷重強度 [N/mm²]（版の自重 ＋ 仕上げ等）。版なしは 0。
    ///
    /// 自重が算定できない版（断面・主材料が未割当）は仕上げ分だけを返す。
    /// 解析前チェックがこの状態を止めるため、ここでは既定厚で補わない。
    pub fn region_dead_intensity(&self, region: &FloorRegion) -> f64 {
        let Some(plate) = &region.plate else {
            return 0.0;
        };
        self.region_self_weight_intensity(region).unwrap_or(0.0) + plate.finish_intensity()
    }

    /// 用途に応じた合成面荷重強度 [N/mm²]（固定 DL ＋ 積載 LL(purpose)）。版なしは 0。
    /// 長期骨組解析は `Frame`、地震用重量は `Seismic`、床・小梁設計は `Floor`。
    pub fn region_intensity(&self, region: &FloorRegion, purpose: LoadPurpose) -> f64 {
        let Some(plate) = &region.plate else {
            return 0.0;
        };
        self.region_dead_intensity(region) + plate.live_intensity(purpose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FloorRegionId, MaterialId, NodeId, SectionId};

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
    fn test_enclosed_boundary_coords() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 4000.0, 0.0],
            [0.0, 4000.0, 0.0],
        ]);
        let r = FloorRegion::enclosed(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        );
        let coords = r.boundary_coords(&m).expect("境界座標");
        assert_eq!(coords.len(), 4);
        assert_eq!(r.boundary_nodes().map(|b| b.len()), Some(4));
        assert_eq!(r.level(&m), Some(0.0));
        assert!(!r.is_attached());
    }

    /// 取付き線に取り付く領域の輪郭は、取付き線・区間・張り出し量から決まる。
    #[test]
    fn test_attached_line_boundary_coords() {
        let m = model_with_nodes(&[[0.0, 0.0, 3000.0], [8000.0, 0.0, 3000.0]]);
        let r = FloorRegion {
            id: FloorRegionId(0),
            name: "バルコニー".into(),
            shape: RegionShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.25, 0.75],
                    transfer: LoadTransfer::Anchor,
                },
                // 取付き線 0→1（+X 向き）の左＝ +Y 側へ 1500 跳ね出す。
                extent: [1500.0, 1500.0],
            },
            plate: None,
            secondary_joist_ids: Vec::new(),
        };
        let c = r.boundary_coords(&m).expect("境界座標");
        assert_eq!(c.len(), 4);
        // 区間 0.25〜0.75 → x=2000〜6000
        assert!((c[0][0] - 2000.0).abs() < 1e-9 && (c[0][1] - 0.0).abs() < 1e-9);
        assert!((c[1][0] - 6000.0).abs() < 1e-9);
        // 跳ね出しは +Y 側（左）。
        assert!((c[2][1] - 1500.0).abs() < 1e-9);
        assert!((c[3][1] - 1500.0).abs() < 1e-9);
        assert_eq!(r.level(&m), Some(3000.0));
        assert!(r.is_attached());
        assert_eq!(r.boundary_nodes(), None, "取り付き領域は境界節点を持たない");
    }

    /// 張り出し量の符号は、取付き線の左右を表す。
    #[test]
    fn test_attached_extent_sign_flips_side() {
        let m = model_with_nodes(&[[0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]]);
        let mk = |extent: [f64; 2]| FloorRegion {
            id: FloorRegionId(0),
            name: String::new(),
            shape: RegionShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent,
            },
            plate: None,
            secondary_joist_ids: Vec::new(),
        };
        let plus = mk([1000.0, 1000.0]).boundary_coords(&m).unwrap();
        let minus = mk([-1000.0, -1000.0]).boundary_coords(&m).unwrap();
        assert!(plus[2][1] > 0.0, "正は左（+Y）側");
        assert!(minus[2][1] < 0.0, "負は右（-Y）側");
    }

    /// 出隅（点に取り付く領域）の輪郭は、柱位置と X/Y の張り出し量で決まる。
    #[test]
    fn test_attached_point_boundary_coords() {
        let m = model_with_nodes(&[[1000.0, 2000.0, 3000.0]]);
        let r = FloorRegion {
            id: FloorRegionId(0),
            name: "出隅".into(),
            shape: RegionShape::Attached {
                anchor: RegionAnchor::Point(NodeId(0)),
                extent: [1000.0, -800.0],
            },
            plate: None,
            secondary_joist_ids: Vec::new(),
        };
        let c = r.boundary_coords(&m).expect("境界座標");
        assert_eq!(c.len(), 4);
        assert!((c[1][0] - 2000.0).abs() < 1e-9, "X 方向へ +1000");
        assert!((c[2][1] - 1200.0).abs() < 1e-9, "Y 方向へ -800");
    }

    /// 版なし床領域は荷重を持たない。
    #[test]
    fn test_plateless_region_has_no_load() {
        let m = Model::default();
        let r = FloorRegion::enclosed(FloorRegionId(0), vec![]);
        assert_eq!(m.region_dead_intensity(&r), 0.0);
        assert_eq!(m.region_intensity(&r, LoadPurpose::Floor), 0.0);
        assert_eq!(m.region_thickness(&r), None);
    }

    /// 版の自重は断面の板厚と主材料から算定する。
    #[test]
    fn test_plate_intensities() {
        let mut m = Model::default();
        m.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: crate::units::to_internal::mass_density_from_unit_weight_kn_m3(24.0),
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        let mut sec = crate::section_shape::SectionShape::RcSlab { thickness: 150.0 }
            .to_section(SectionId(0), "S15".into());
        sec.material = Some(MaterialId(0));
        m.sections.push(sec);

        let r = FloorRegion::enclosed(FloorRegionId(0), vec![]).with_plate(SlabPlate {
            section: Some(SectionId(0)),
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: 1.0e-3,
            }],
            usage: Some(SlabUsage::Office),
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
            joists: Vec::new(),
        });
        assert_eq!(m.region_thickness(&r), Some(150.0));
        // 自重 = 150 mm × 24 kN/m³ = 3.6e-3 N/mm²
        assert!((m.region_self_weight_intensity(&r).unwrap() - 3.6e-3).abs() < 1e-9);
        assert!((m.region_dead_intensity(&r) - 4.6e-3).abs() < 1e-9);
        // 骨組用の積載 1800 N/m² が加わる。
        assert!((m.region_intensity(&r, LoadPurpose::Frame) - (4.6e-3 + 1800e-6)).abs() < 1e-9);
    }
}
