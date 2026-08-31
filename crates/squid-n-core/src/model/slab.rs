//! 床板（スラブ）。
//!
//! 床領域（[`FloorRegion`]、[`super::region`]）は大梁の 1 スパン区画そのもので、
//! 版の仕様は持たない。版の仕様（厚さ・材料・仕上げ荷重・室用途）は本モジュールの
//! [`Slab`] が持つ。1 つの床領域は、床領域内が小梁でさらに細かい打設単位に
//! 分かれていれば複数の [`Slab`] を持ちうる（[`FloorRegion::slab_ids`]）。
//! 片持ちスラブ・バルコニー・出隅はどの床領域からも参照されない、独立した [`Slab`]
//! として存在する。
//!
//! - [`Slab`] — 床板本体（形状＋版仕様）。
//! - [`SlabShape`] — 床板の形（大梁または小梁で囲まれた領域 / 主架構に取り付く領域）。
//! - [`SlabPlate`] — 版の仕様（断面・仕上げ荷重・室用途・分配方法）。
//! - [`DistributionMethod`] — 床荷重の分配方法。
//! - [`AreaLoad`] — 面荷重。
//! - [`OneWayDir`] — 一方向スラブの伝達方向。
//! - [`LoadPurpose`] — 積載荷重の用途（床用／骨組用／地震用。令85条1項）。
//! - [`SlabUsage`] — 室用途（令別表第1 の積載荷重プリセット）。

use super::*;

/// 床板の版（仕様）。
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SlabPlate {
    /// スラブ断面（符号・板厚・コンクリート材料）。
    ///
    /// **板厚と自重はこの断面から解決する**（[`Model::slab_plate_thickness`]・
    /// [`Model::slab_self_weight_intensity`]）。板厚を床板と断面の両方に持たせると
    /// 同じ数値の持ち主が 2 つになるため、床板側は断面を指すだけとする。
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

/// 床板の形。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SlabShape {
    /// 大梁または小梁で囲まれた領域。境界は反時計回りの節点列（始点は繰り返さない）。
    Enclosed { boundary: Vec<NodeId> },
    /// 主架構に取り付く領域（片持ち・バルコニー・出隅）。
    Attached {
        /// 取付き先。荷重の出口は取付き先の種類ごとに決まる（[`RegionAnchor`] 参照）。
        anchor: RegionAnchor,
        /// 張り出し量 [mm]。意味は取付き先の種類で決まる（[`RegionAnchor`] 参照）。
        extent: [f64; 2],
    },
}

/// 床板。大梁または小梁で囲まれた版、または主架構に取り付く版
/// （片持ち・バルコニー・出隅）ごとに 1 つ。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Slab {
    /// 床板 ID（`Model::slabs` の配列インデックスと一致すること）。
    pub id: SlabId,
    pub shape: SlabShape,
    pub plate: SlabPlate,
}

impl Slab {
    /// 取り付く床板か。
    pub fn is_attached(&self) -> bool {
        matches!(self.shape, SlabShape::Attached { .. })
    }

    /// 境界の節点列。**大梁または小梁で囲まれた床板のみ**（取り付く床板は
    /// 自由端に節点を持たないため `None`）。
    pub fn boundary_nodes(&self) -> Option<&[NodeId]> {
        match &self.shape {
            SlabShape::Enclosed { boundary } => Some(boundary),
            SlabShape::Attached { .. } => None,
        }
    }

    /// 境界多角形の座標列 [mm]。取り付く床板は取付き先と張り出し量から算出する。
    /// 節点が引けない（陳腐化した参照）場合は `None`。
    pub fn boundary_coords(&self, model: &Model) -> Option<Vec<[f64; 3]>> {
        match &self.shape {
            SlabShape::Enclosed { boundary } => boundary
                .iter()
                .map(|n| model.nodes.get(n.index()).map(|n| n.coord))
                .collect(),
            SlabShape::Attached { anchor, extent } => match anchor {
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
                // 床板の取付き先には使わない（`RegionAnchor::FloorRegion` のドキュメント
                // 参照。壁側〔`WallPlate` の `Attached` 形〕専用のアンカーであり、
                // 床板では到達しない）。
                RegionAnchor::FloorRegion { .. } => None,
            },
        }
    }

    /// 境界の辺 `k` の両端節点。荷重分配の結果（`LoadTarget::Edge`）を主架構の梁へ
    /// 結びつけるために使う。
    ///
    /// 大梁または小梁で囲まれた床板は `boundary[k]`→`boundary[k+1]`、取り付く床板
    /// （線）は辺 0 が取付き線そのもの（それ以外の辺は自由端なので `None`）。
    /// 取り付く床板（点）は辺を持たない。
    pub fn edge_nodes(&self, k: usize) -> Option<[NodeId; 2]> {
        match &self.shape {
            SlabShape::Enclosed { boundary } => {
                let n = boundary.len();
                (n >= 3 && k < n).then(|| [boundary[k], boundary[(k + 1) % n]])
            }
            SlabShape::Attached { anchor, .. } => match anchor {
                RegionAnchor::Line { nodes, .. } if k == 0 => Some(*nodes),
                RegionAnchor::Line { .. } | RegionAnchor::Point(_) => None,
                // 床板では到達しない（`boundary_coords` と同じ理由）。
                RegionAnchor::FloorRegion { .. } => None,
            },
        }
    }

    /// 床板を代表する節点。大梁または小梁で囲まれた床板は境界の先頭、
    /// 取り付く床板は取付き先の節点。
    pub fn reference_node(&self) -> Option<NodeId> {
        match &self.shape {
            SlabShape::Enclosed { boundary } => boundary.first().copied(),
            SlabShape::Attached { anchor, .. } => match anchor {
                RegionAnchor::Line { nodes, .. } => Some(nodes[0]),
                RegionAnchor::Point(n) => Some(*n),
                // 床板では到達しない（`boundary_coords` と同じ理由）。
                RegionAnchor::FloorRegion { .. } => None,
            },
        }
    }

    /// 床板のレベル Z [mm]（境界座標の Z の平均）。境界が引けなければ `None`。
    pub fn level(&self, model: &Model) -> Option<f64> {
        let coords = self.boundary_coords(model)?;
        if coords.is_empty() {
            return None;
        }
        Some(coords.iter().map(|c| c[2]).sum::<f64>() / coords.len() as f64)
    }

    /// 分配方法。
    pub fn method(&self) -> DistributionMethod {
        self.plate.method
    }

    /// 一方向スラブの伝達方向。
    pub fn one_way(&self) -> Option<OneWayDir> {
        self.plate.one_way
    }

    /// 用途別の積載荷重（LL）の面荷重強度 [N/mm²]。用途未設定は 0。
    pub fn live_intensity(&self, purpose: LoadPurpose) -> f64 {
        self.plate.live_intensity(purpose)
    }

    /// 版の断面（未割当は `None`）。
    pub fn section(&self) -> Option<crate::ids::SectionId> {
        self.plate.section
    }

    /// 室用途（未設定は `None`）。
    pub fn usage(&self) -> Option<SlabUsage> {
        self.plate.usage
    }

    /// 取り付く床板の設計スパン [mm]（張り出し量の絶対値の大きい方）。
    /// 大梁または小梁で囲まれた床板、値が非有限、または 0 以下のときは `None`。
    pub fn attached_design_span(&self) -> Option<f64> {
        match &self.shape {
            SlabShape::Enclosed { .. } => None,
            SlabShape::Attached { extent, .. } => {
                let span = extent[0].abs().max(extent[1].abs());
                (span.is_finite() && span > 0.0).then_some(span)
            }
        }
    }
}

impl Model {
    /// 床板 ID から床板を引く。存在しなければ `None`。
    pub fn slab(&self, id: SlabId) -> Option<&Slab> {
        match self.slabs.get(id.index()) {
            Some(s) if s.id == id => Some(s),
            _ => self.slabs.iter().find(|s| s.id == id),
        }
    }

    /// 床板の版へ割り当てた断面。未割当・ダングリングは `None`。
    pub fn slab_section(&self, slab: &Slab) -> Option<&Section> {
        slab.section()
            .and_then(|sid| self.sections.get(sid.index()))
    }

    /// 版の板厚 [mm]。断面の [`Section::thickness`] をそのまま返す。
    ///
    /// 断面が未割当、または断面が板厚を持たない（板状でない形状を割り当てた）場合は `None`。
    /// **建物一律の [`Model::slab_thickness`] へは退かない**。あちらは「剛性計算に見込む
    /// スラブ厚」であり、既定の 0 は「スラブ協力幅による梁剛性増大を見込まない」を意味する
    /// 別概念のためである。
    pub fn slab_plate_thickness(&self, slab: &Slab) -> Option<f64> {
        self.slab_section(slab)
            .and_then(|s| s.thickness)
            .filter(|t| *t > 0.0)
    }

    /// 版の自重の面荷重強度 [N/mm²]（板厚 × 断面の主材料の単位体積重量）。
    ///
    /// 断面または断面の主材料が未割当のときは `None`。自重を面荷重として焼き込まず
    /// 毎回算定するのは、板厚や材料を変えたときに自重が追随しないという食い違いを
    /// 作らないためである。
    pub fn slab_self_weight_intensity(&self, slab: &Slab) -> Option<f64> {
        let t = self.slab_plate_thickness(slab)?;
        let mat = self
            .slab_section(slab)
            .and_then(|s| s.material)
            .and_then(|mid| self.materials.get(mid.index()))?;
        Some(t * mat.density * crate::units::GRAVITY_MM_S2)
    }

    /// 固定荷重（DL）の面荷重強度 [N/mm²]（版の自重 ＋ 仕上げ等）。
    ///
    /// 自重が算定できない版（断面・主材料が未割当）は仕上げ分だけを返す。
    /// 解析前チェックがこの状態を止めるため、ここでは既定厚で補わない。
    pub fn slab_dead_intensity(&self, slab: &Slab) -> f64 {
        self.slab_self_weight_intensity(slab).unwrap_or(0.0) + slab.plate.finish_intensity()
    }

    /// 用途に応じた合成面荷重強度 [N/mm²]（固定 DL ＋ 積載 LL(purpose)）。
    /// 長期骨組解析は `Frame`、地震用重量は `Seismic`、床・小梁設計は `Floor`。
    pub fn slab_intensity(&self, slab: &Slab, purpose: LoadPurpose) -> f64 {
        self.slab_dead_intensity(slab) + slab.plate.live_intensity(purpose)
    }
}

/// 床荷重の分配方法。既定は三角形・台形分配。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DistributionMethod {
    #[default]
    TriTrapezoid,
    OneWay,
    TributaryArea,
}

/// 積載荷重の用途（令85条1項・令別表第1 の 3 欄）。
/// - `Floor`（床用）: 床スラブ・小梁の設計用。最も大きい。
/// - `Frame`（骨組用）: 大梁・柱・基礎の設計用（長期骨組解析に用いる）。
/// - `Seismic`（地震用）: 地震力（地震用重量）の算定用。最も小さい。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LoadPurpose {
    Floor,
    Frame,
    Seismic,
}

/// 室の用途（積載荷重プリセット）。`live_load` で用途別の積載荷重 [N/mm²] を返す。
/// `Custom` は 3 欄（床版・小梁計算用／大梁・柱・基礎計算用／地震力計算用）を
/// 直接持つ（内部単位 N/mm²）。
///
/// 出典: 建築基準法施行令 第85条第1項・令別表第1、および国土交通省官庁営繕部
/// 「建築構造設計基準」令和3年度版（同資料は令85条を準用しつつ、官庁施設に特有の
/// 室用途〔書庫・実験室・電算室・機械室・体育館等〕を追加する）。値は N/m² を
/// 内部単位 N/mm²（×1e-6）へ換算して返す。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SlabUsage {
    /// 住宅の居室、住宅以外の建築物における寝室又は病室。
    Residential,
    /// 事務室、会議室及び食堂。
    Office,
    /// 研究室（値は事務室に同じ。実況に応じて算定する）。
    ResearchRoom,
    /// 教室。
    Classroom,
    /// 百貨店又は店舗の売場。
    Store,
    /// 劇場・映画館等の客席又は集会室（固定席の場合）。
    AssemblyFixed,
    /// 同上（その他の場合）。
    AssemblyOther,
    /// 廊下・玄関・階段（劇場・集会場・売場等に連絡するもの）。
    Corridor,
    /// 法務局登記書庫（法務省型鋼製書架 W型8段6連を配置した場合）。
    RegistryArchive,
    /// 一般書庫、倉庫等（通常の階高の室に満載の書架を配置した場合）。
    GeneralArchive,
    /// 移動書架を設置する書庫、電算室の空調機室、用具庫等（一般書庫の1.5倍程度）。
    MobileArchive,
    /// 一般実験室（化学系）。
    LabChemistry,
    /// 一般実験室（物理系）。
    LabPhysics,
    /// 電算室（床版・小梁計算用は電算室用既製床の耐荷重、他は令85条の店舗の売場を準用）。
    ComputerRoom,
    /// 機械室（床版・小梁計算用は機械の平均的な重量、他は令85条の店舗の売場を準用）。
    MachineRoom,
    /// 体育館、武道場等（令85条の劇場等〔その他〕を準用）。
    Gymnasium,
    /// 自動車車庫及び自動車通路。
    Garage,
    /// 片持形式のバルコニー、庇等（令85条のバルコニーを準用）。
    Balcony,
    /// 屋上広場（常時人が使用する場合。学校・百貨店の類を除く）。
    RoofResidential,
    /// 屋上広場（常時人が使用する場合。学校・百貨店の類）。
    RoofStore,
    /// 屋上（通常人が使用しない場合）。
    RoofUnused,
    /// 屋上（鉄骨造体育館、武道場等）。短期荷重として扱い、床版・小梁計算用のみ
    /// 作業荷重を見込む。大梁・柱・基礎計算用および地震力計算用は 0。
    RoofSteelGym,
    /// 任意入力（床版・小梁計算用／大梁・柱・基礎計算用／地震力計算用、いずれも N/mm²）。
    Custom {
        floor: f64,
        frame: f64,
        seismic: f64,
    },
}

impl SlabUsage {
    /// 用途別の積載荷重 [N/mm²]（令別表第1）。
    pub fn live_load(self, purpose: LoadPurpose) -> f64 {
        // プリセットは令別表第1／国交省営繕基準の [N/m²]。内部単位 N/mm² へ ×1e-6。
        // 返り値の並びは (床用, 骨組用, 地震用)。
        let (floor, frame, seismic) = match self {
            SlabUsage::Residential => (1800.0, 1300.0, 600.0),
            SlabUsage::Office => (2900.0, 1800.0, 800.0),
            SlabUsage::ResearchRoom => (2900.0, 1800.0, 800.0),
            SlabUsage::Classroom => (2300.0, 2100.0, 1100.0),
            SlabUsage::Store => (2900.0, 2400.0, 1300.0),
            SlabUsage::AssemblyFixed => (2900.0, 2600.0, 1600.0),
            SlabUsage::AssemblyOther => (3500.0, 3200.0, 2100.0),
            SlabUsage::Corridor => (3500.0, 3200.0, 2100.0),
            SlabUsage::RegistryArchive => (5900.0, 4900.0, 3900.0),
            SlabUsage::GeneralArchive => (7800.0, 6900.0, 4900.0),
            SlabUsage::MobileArchive => (11800.0, 10300.0, 7400.0),
            SlabUsage::LabChemistry => (3900.0, 2400.0, 1600.0),
            SlabUsage::LabPhysics => (4900.0, 3900.0, 2500.0),
            SlabUsage::ComputerRoom => (4900.0, 2400.0, 1300.0),
            SlabUsage::MachineRoom => (4900.0, 2400.0, 1300.0),
            SlabUsage::Gymnasium => (3500.0, 3200.0, 2100.0),
            SlabUsage::Garage => (5400.0, 3900.0, 2000.0),
            SlabUsage::Balcony => (1800.0, 1300.0, 600.0),
            SlabUsage::RoofResidential => (1800.0, 1300.0, 600.0),
            SlabUsage::RoofStore => (2900.0, 2400.0, 1300.0),
            SlabUsage::RoofUnused => (980.0, 600.0, 400.0),
            SlabUsage::RoofSteelGym => (980.0, 0.0, 0.0),
            // Custom は内部単位 N/mm² をそのまま返す（×1e-6 しない）。
            SlabUsage::Custom {
                floor,
                frame,
                seismic,
            } => {
                return match purpose {
                    LoadPurpose::Floor => floor,
                    LoadPurpose::Frame => frame,
                    LoadPurpose::Seismic => seismic,
                };
            }
        };
        let v_n_per_m2 = match purpose {
            LoadPurpose::Floor => floor,
            LoadPurpose::Frame => frame,
            LoadPurpose::Seismic => seismic,
        };
        v_n_per_m2 * 1e-6
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AreaLoad {
    pub kind: String,
    pub value: f64,
}

/// 一方向スラブの荷重伝達方向（床ごとに指定。床荷重の分配における伝達方向〔X〕〔Y〕）。
/// `X` は全体座標 X 方向へ伝達（＝X 方向両側の辺が負担）、`Y` は Y 方向へ伝達。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum OneWayDir {
    X,
    Y,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_usage_table_values_n_per_mm2() {
        // 令別表第1: 事務室 = 床用 2900 / 骨組用 1800 / 地震用 800 [N/m²]。
        let o = SlabUsage::Office;
        assert!((o.live_load(LoadPurpose::Floor) - 2900e-6).abs() < 1e-12);
        assert!((o.live_load(LoadPurpose::Frame) - 1800e-6).abs() < 1e-12);
        assert!((o.live_load(LoadPurpose::Seismic) - 800e-6).abs() < 1e-12);
        // 住宅 = 1800 / 1300 / 600。
        let r = SlabUsage::Residential;
        assert!((r.live_load(LoadPurpose::Floor) - 1800e-6).abs() < 1e-12);
        assert!((r.live_load(LoadPurpose::Frame) - 1300e-6).abs() < 1e-12);
        assert!((r.live_load(LoadPurpose::Seismic) - 600e-6).abs() < 1e-12);
        // 国交省営繕基準（令和3年度版）で追加した室用途の値（[N/m²]）。
        let cases: &[(SlabUsage, f64, f64, f64)] = &[
            (SlabUsage::ResearchRoom, 2900.0, 1800.0, 800.0),
            (SlabUsage::RegistryArchive, 5900.0, 4900.0, 3900.0),
            (SlabUsage::GeneralArchive, 7800.0, 6900.0, 4900.0),
            (SlabUsage::MobileArchive, 11800.0, 10300.0, 7400.0),
            (SlabUsage::LabChemistry, 3900.0, 2400.0, 1600.0),
            (SlabUsage::LabPhysics, 4900.0, 3900.0, 2500.0),
            (SlabUsage::ComputerRoom, 4900.0, 2400.0, 1300.0),
            (SlabUsage::MachineRoom, 4900.0, 2400.0, 1300.0),
            (SlabUsage::Gymnasium, 3500.0, 3200.0, 2100.0),
            (SlabUsage::Balcony, 1800.0, 1300.0, 600.0),
            (SlabUsage::RoofUnused, 980.0, 600.0, 400.0),
            (SlabUsage::RoofSteelGym, 980.0, 0.0, 0.0),
        ];
        for &(u, floor, frame, seismic) in cases {
            assert!((u.live_load(LoadPurpose::Floor) - floor * 1e-6).abs() < 1e-12);
            assert!((u.live_load(LoadPurpose::Frame) - frame * 1e-6).abs() < 1e-12);
            assert!((u.live_load(LoadPurpose::Seismic) - seismic * 1e-6).abs() < 1e-12);
        }

        // 積載は 床用 ≥ 骨組用 ≥ 地震用 の順（全用途で成り立つ）。
        for u in [
            SlabUsage::Residential,
            SlabUsage::Office,
            SlabUsage::ResearchRoom,
            SlabUsage::Classroom,
            SlabUsage::Store,
            SlabUsage::AssemblyFixed,
            SlabUsage::AssemblyOther,
            SlabUsage::Corridor,
            SlabUsage::RegistryArchive,
            SlabUsage::GeneralArchive,
            SlabUsage::MobileArchive,
            SlabUsage::LabChemistry,
            SlabUsage::LabPhysics,
            SlabUsage::ComputerRoom,
            SlabUsage::MachineRoom,
            SlabUsage::Gymnasium,
            SlabUsage::Garage,
            SlabUsage::Balcony,
            SlabUsage::RoofResidential,
            SlabUsage::RoofStore,
            SlabUsage::RoofUnused,
            SlabUsage::RoofSteelGym,
        ] {
            let f = u.live_load(LoadPurpose::Floor);
            let g = u.live_load(LoadPurpose::Frame);
            let s = u.live_load(LoadPurpose::Seismic);
            assert!(f >= g && g >= s, "床用≥骨組用≥地震用: {u:?}");
        }
    }

    #[test]
    fn test_usage_custom_is_internal_units() {
        // Custom は内部単位 N/mm² をそのまま返す（換算しない）。
        let c = SlabUsage::Custom {
            floor: 3.0e-3,
            frame: 2.0e-3,
            seismic: 1.0e-3,
        };
        assert_eq!(c.live_load(LoadPurpose::Floor), 3.0e-3);
        assert_eq!(c.live_load(LoadPurpose::Frame), 2.0e-3);
        assert_eq!(c.live_load(LoadPurpose::Seismic), 1.0e-3);
    }

    /// `RegionAnchor::FloorRegion` は床板の取付き先には使わない
    /// （壁側〔自立壁〕専用のアンカー。モジュール doc・用語集「取付き先」参照）。
    /// `Slab` のメソッドはこの分岐に来ないはずだが、`RegionAnchor` を壁と共有する
    /// 都合上、網羅性のために持つ受け皿の戻り値を固定する。
    #[test]
    fn test_floor_region_anchor_never_yields_slab_geometry() {
        let model = Model::default();
        let slab = Slab {
            id: crate::ids::SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    nodes: [NodeId(0), NodeId(1)],
                },
                extent: [0.0, 0.0],
            },
            plate: SlabPlate::default(),
        };
        assert_eq!(slab.boundary_coords(&model), None);
        assert_eq!(slab.reference_node(), None);
        assert_eq!(slab.edge_nodes(0), None);
    }
}
