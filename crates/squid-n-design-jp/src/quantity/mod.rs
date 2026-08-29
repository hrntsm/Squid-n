//! 数量積算（部位別の概算数量集計）。
//!
//! 建物モデルから部位別（柱・大梁・小梁・基礎梁・床・壁・ブレース）に
//! コンクリート体積・型枠面積・鉄筋重量・鉄骨重量を概算集計する。
//! 構造計算モデルを唯一の入力とする概算数量であり、詳細積算
//! （フック・余長・継手長さ・開口補強筋・接合部プレート等）は
//! 考慮しない（各部位共通事項）。
//!
//! - [`QuantityCfg`] — 積算の設定（定着長さ係数・鉄筋比等）
//! - [`compute_quantity_takeoff`] — モデル全体の数量集計
//! - [`QuantityTakeoff`] / [`MemberQuantity`] — 集計結果
//! - [`member`] — 部位別の算定式（純関数）
//! - [`rebar`] — 鉄筋の単位質量
//!
//! ## 単位
//!
//! モデルの内部単位系は N-mm-s だが、集計結果は実務慣用単位で持つ:
//! コンクリート体積 [m³]・型枠面積 [m²]・鉄筋/鉄骨重量 [t]・長さ [m]。
//! 鉄骨重量は `W = L×A×7.85`（鉄骨単位重量 7.85 t/m³ 固定。
//! 各部位共通事項）による。

pub mod member;
pub mod rebar;

#[cfg(test)]
mod tests;

use std::collections::{BTreeMap, HashMap, HashSet};

use squid_n_core::ids::{ElemId, SlabId};
use squid_n_core::model::{
    ElementData, ElementKind, Model, RegionAnchor, SecondaryMember, SecondaryMemberKind, Slab,
    WallPlate, WallPlateShape,
};
use squid_n_core::section_shape::{RcRebar, SectionShape};

use member::{BeamBarEnd, Haunch};

/// 鉄骨・鉄筋の単位重量 [t/m³]（数量積算の慣用値 7.85 t/m³。各部位共通事項）。
/// 固定荷重の γs=77 kN/m³ とは分野別の慣用値として使い分ける
/// （`squid_n_core::units::STEEL_UNIT_WEIGHT_TAKEOFF_T_M3` 参照）。
const STEEL_UNIT_WEIGHT_T_PER_M3: f64 = squid_n_core::units::STEEL_UNIT_WEIGHT_TAKEOFF_T_M3;

/// 鉄骨単位重量 [t/mm³]（[`STEEL_UNIT_WEIGHT_T_PER_M3`] の内部単位系換算）。
const STEEL_UNIT_WEIGHT_T_PER_MM3: f64 = STEEL_UNIT_WEIGHT_T_PER_M3 * 1e-9;

/// 「同一レベル」とみなす標高差 [mm]（基礎梁レベルの判定用）。
const LEVEL_TOL_MM: f64 = 10.0;

/// 数量積算の設定。
#[derive(Clone, Copy, Debug)]
pub struct QuantityCfg {
    /// 主筋・壁筋の定着長さ係数（L2・S = 係数×呼び径。既定 35d）。
    pub anchorage_dia_factor: f64,
    /// 壁筋の仮定呼び径 [mm]（壁の配筋はせん断補強筋比 ps のみ保持する
    /// ため、定着長さ S=35d の算定に用いる仮定径。既定 D10）。
    pub assumed_wall_bar_dia: f64,
    /// 小梁主筋の鉄筋比（コンクリート体積比。既定 0.8%）。
    pub joist_main_ratio: f64,
    /// 小梁スターラップの鉄筋比（既定 0.1%）。
    pub joist_stirrup_ratio: f64,
    /// 床・片持ち床の鉄筋比（既定 1.0%）。
    pub slab_rebar_ratio: f64,
}

impl Default for QuantityCfg {
    fn default() -> Self {
        Self {
            anchorage_dia_factor: 35.0,
            assumed_wall_bar_dia: 10.0,
            joist_main_ratio: 0.008,
            joist_stirrup_ratio: 0.001,
            slab_rebar_ratio: 0.01,
        }
    }
}

/// 部位分類。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MemberCategory {
    /// 基礎梁（最下層レベルの水平梁）。
    FoundationGirder,
    /// 柱。
    Column,
    /// 大梁（柱に取り付く水平梁）。
    Girder,
    /// 小梁（柱に取り付かない水平梁）。
    Joist,
    /// 床（一般スラブ）。
    Slab,
    /// 片持ち床・出隅。
    CantileverSlab,
    /// 壁（耐震壁・フレーム内雑壁）。
    Wall,
    /// フレーム外雑壁。
    MiscWall,
    /// ブレース。
    Brace,
}

impl MemberCategory {
    /// 表示名（日本語）。
    pub fn label(self) -> &'static str {
        match self {
            MemberCategory::FoundationGirder => "基礎梁",
            MemberCategory::Column => "柱",
            MemberCategory::Girder => "大梁",
            MemberCategory::Joist => "小梁",
            MemberCategory::Slab => "床",
            MemberCategory::CantileverSlab => "片持ち床",
            MemberCategory::Wall => "壁",
            MemberCategory::MiscWall => "雑壁",
            MemberCategory::Brace => "ブレース",
        }
    }
}

pub use squid_n_core::structure_kind::StructureKind;

/// 鉄筋の用途分類。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum RebarUsage {
    /// 柱・梁の主筋。
    MainBar,
    /// 柱フープ。
    Hoop,
    /// 梁スターラップ。
    Stirrup,
    /// 壁横筋。
    WallHorizontal,
    /// 壁縦筋。
    WallVertical,
    /// 床筋（鉄筋比による概算）。
    SlabBar,
    /// 小梁主筋（鉄筋比による概算）。
    JoistMain,
    /// 小梁スターラップ（鉄筋比による概算）。
    JoistStirrup,
}

impl RebarUsage {
    /// 表示名（日本語）。
    pub fn label(self) -> &'static str {
        match self {
            RebarUsage::MainBar => "主筋",
            RebarUsage::Hoop => "フープ",
            RebarUsage::Stirrup => "スターラップ",
            RebarUsage::WallHorizontal => "壁横筋",
            RebarUsage::WallVertical => "壁縦筋",
            RebarUsage::SlabBar => "床筋",
            RebarUsage::JoistMain => "小梁主筋",
            RebarUsage::JoistStirrup => "小梁スターラップ",
        }
    }
}

/// 鉄筋数量 1 件（用途×呼び径ごと）。
#[derive(Clone, Debug)]
pub struct RebarItem {
    pub usage: RebarUsage,
    /// 呼び径 [mm]。鉄筋比による概算（床・小梁）は None。
    pub dia: Option<f64>,
    /// 総長さ [m]（鉄筋比による概算は 0）。
    pub total_length_m: f64,
    /// 重量 [t]。
    pub weight_t: f64,
}

/// 鉄骨数量 1 件。
#[derive(Clone, Debug)]
pub struct SteelItem {
    /// 断面名（種類別集計のキー）。
    pub section_name: String,
    /// 長さ [m]。
    pub length_m: f64,
    /// 重量 [t]（W = L×A×7.85）。
    pub weight_t: f64,
}

/// 部材（またはスラブ・雑壁）1 件分の数量。
#[derive(Clone, Debug)]
pub struct MemberQuantity {
    /// 対象要素（スラブ・雑壁は None）。
    pub elem: Option<ElemId>,
    /// 対象床板（部材・雑壁は None）。
    pub slab: Option<SlabId>,
    /// 符号（断面名等）。
    pub label: String,
    /// 所属階名（未設定は "-"）。
    pub story: String,
    pub category: MemberCategory,
    pub structure: StructureKind,
    /// コンクリート体積 [m³]。
    pub concrete_m3: f64,
    /// 型枠面積 [m²]。
    pub formwork_m2: f64,
    /// 鉄筋数量（用途別）。
    pub rebar: Vec<RebarItem>,
    /// 鉄骨数量（S・SRC・CFT・ブレース）。
    pub steel: Option<SteelItem>,
    /// 鉄筋継手個所数（圧接個所数の集計。柱・大梁・基礎梁）。
    pub rebar_joints: f64,
}

impl MemberQuantity {
    /// 鉄筋重量の合計 [t]。
    pub fn rebar_weight_t(&self) -> f64 {
        // 空イテレータの f64 Sum は -0.0 を返し表示が "-0.0000" になるため
        // +0.0 で正の 0 に正規化する。
        self.rebar.iter().map(|r| r.weight_t).sum::<f64>() + 0.0
    }

    /// 鉄骨重量 [t]（なければ 0）。
    pub fn steel_weight_t(&self) -> f64 {
        self.steel.as_ref().map(|s| s.weight_t).unwrap_or(0.0)
    }
}

/// 数量の小計（コンクリート・型枠・鉄筋・鉄骨・継手）。
#[derive(Clone, Copy, Debug, Default)]
pub struct QuantityTotals {
    pub concrete_m3: f64,
    pub formwork_m2: f64,
    pub rebar_t: f64,
    pub steel_t: f64,
    pub rebar_joints: f64,
}

impl QuantityTotals {
    fn add(&mut self, it: &MemberQuantity) {
        self.concrete_m3 += it.concrete_m3;
        self.formwork_m2 += it.formwork_m2;
        self.rebar_t += it.rebar_weight_t();
        self.steel_t += it.steel_weight_t();
        self.rebar_joints += it.rebar_joints;
    }
}

/// 数量積算の結果一式。
#[derive(Clone, Debug, Default)]
pub struct QuantityTakeoff {
    /// 部材・スラブ・雑壁ごとの明細。
    pub items: Vec<MemberQuantity>,
    /// 前提・未対応事項の注記。
    pub notes: Vec<String>,
}

impl QuantityTakeoff {
    /// 全体合計。
    pub fn totals(&self) -> QuantityTotals {
        let mut t = QuantityTotals::default();
        for it in &self.items {
            t.add(it);
        }
        t
    }

    /// 部位別小計（`MemberCategory` の定義順）。
    pub fn totals_by_category(&self) -> Vec<(MemberCategory, QuantityTotals)> {
        let mut map: BTreeMap<MemberCategory, QuantityTotals> = BTreeMap::new();
        for it in &self.items {
            map.entry(it.category).or_default().add(it);
        }
        map.into_iter().collect()
    }

    /// 階別小計（明細の出現順を保持）。
    pub fn totals_by_story(&self) -> Vec<(String, QuantityTotals)> {
        let mut order: Vec<String> = Vec::new();
        let mut map: HashMap<String, QuantityTotals> = HashMap::new();
        for it in &self.items {
            if !map.contains_key(&it.story) {
                order.push(it.story.clone());
            }
            map.entry(it.story.clone()).or_default().add(it);
        }
        order
            .into_iter()
            .map(|k| {
                let v = map[&k];
                (k, v)
            })
            .collect()
    }

    /// 鉄骨の種類別（断面名別）長さ・重量集計。
    pub fn steel_by_section(&self) -> Vec<SteelItem> {
        let mut order: Vec<String> = Vec::new();
        let mut map: HashMap<String, (f64, f64)> = HashMap::new();
        for it in self.items.iter().filter_map(|i| i.steel.as_ref()) {
            if !map.contains_key(&it.section_name) {
                order.push(it.section_name.clone());
            }
            let e = map.entry(it.section_name.clone()).or_default();
            e.0 += it.length_m;
            e.1 += it.weight_t;
        }
        order
            .into_iter()
            .map(|name| {
                let (l, w) = map[&name];
                SteelItem {
                    section_name: name,
                    length_m: l,
                    weight_t: w,
                }
            })
            .collect()
    }

    /// 鉄筋の呼び径別長さ・重量集計（径 None＝鉄筋比概算は径 0 に集約）。
    pub fn rebar_by_dia(&self) -> Vec<(f64, f64, f64)> {
        let mut map: BTreeMap<u32, (f64, f64)> = BTreeMap::new();
        for it in &self.items {
            for r in &it.rebar {
                let key = r.dia.unwrap_or(0.0).round() as u32;
                let e = map.entry(key).or_default();
                e.0 += r.total_length_m;
                e.1 += r.weight_t;
            }
        }
        map.into_iter()
            .map(|(d, (l, w))| (d as f64, l, w))
            .collect()
    }
}

/// 2 点間距離 [mm]。算定の情報源は `squid-n-core` に置く。
use squid_n_core::geom::vec3::dist as dist3;

/// 鉛直材（柱）判定。判定規則の情報源は `squid-n-core` に置く
/// （仕上げ周長式・雑壁の柱探索・通り芯の自動生成と同一規則）。
use squid_n_core::geom::is_vertical_pair;

/// 平面多角形（3D 座標）の面積 [mm²]。算定の情報源は `squid-n-core` に置く。
use squid_n_core::geom::polygon_area_3d;

/// SRC 内蔵 H 形鉄骨の断面積 [mm²]。
fn src_steel_area(shape: &SectionShape) -> Option<f64> {
    if let SectionShape::SrcRect {
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
        ..
    } = *shape
    {
        Some(
            2.0 * steel_width * steel_flange_thick
                + (steel_height - 2.0 * steel_flange_thick) * steel_web_thick,
        )
    } else {
        None
    }
}

/// CFT の充填コンクリート断面積 [mm²]。
fn cft_infill_area(shape: &SectionShape) -> Option<f64> {
    match *shape {
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => Some(((width - 2.0 * thick) * (height - 2.0 * thick)).max(0.0)),
        SectionShape::CftPipe { outer_dia, thick } => {
            let ri = (outer_dia / 2.0 - thick).max(0.0);
            Some(std::f64::consts::PI * ri * ri)
        }
        _ => None,
    }
}

/// モデル走査用の前処理データ。
struct Ctx<'a> {
    model: &'a Model,
    cfg: &'a QuantityCfg,
    /// 鉛直材（柱）が取り付く節点集合（大梁/小梁の分類用）。
    column_nodes: HashSet<usize>,
    /// スラブ境界辺 (節点対, 昇順) → (隣接スラブ数, 控除厚の max [mm])。
    slab_edges: HashMap<(u32, u32), (u32, f64)>,
    /// 節点 index → その節点に取り付く水平梁（elem index, 節点から見た
    /// 梁の伸びる向きの単位ベクトル xy）。主筋の外端・内端判定
    /// （梁の連続性）に用いる。
    beams_at_node: HashMap<usize, Vec<(usize, [f64; 2])>>,
    /// 最下層レベル（基礎梁判定用の最小節点標高 [mm]）。
    min_z: f64,
}

impl Ctx<'_> {
    /// 節点の所属階名（未設定は "-"）。
    fn story_name(&self, node_idx: usize) -> String {
        self.model.nodes[node_idx]
            .story
            .and_then(|sid| self.model.stories.get(sid.index()))
            .map(|s| s.name.clone())
            .unwrap_or_else(|| "-".to_string())
    }

    /// 梁端の主筋定着条件。節点 `node_idx` から `outward`（端部の外向き
    /// 水平単位ベクトル）方向へ概ね同一直線上（±45°）に続く別の水平梁が
    /// あれば内端（通し、Dc/2＝フェイス距離）、なければ外端（定着 L2）。
    fn beam_bar_end(
        &self,
        elem_idx: usize,
        node_idx: usize,
        outward: [f64; 2],
        face: f64,
        l2: f64,
    ) -> BeamBarEnd {
        let continues = self
            .beams_at_node
            .get(&node_idx)
            .map(|list| {
                list.iter().any(|&(ei, d)| {
                    ei != elem_idx
                        && d[0] * outward[0] + d[1] * outward[1] > std::f64::consts::FRAC_1_SQRT_2
                })
            })
            .unwrap_or(false);
        if continues {
            BeamBarEnd::Interior { half_dc: face }
        } else {
            BeamBarEnd::Exterior { l2 }
        }
    }

    /// 梁の両側のスラブ隣接数（0/1/2）。スラブ境界辺に梁の節点対が
    /// 一致するスラブの数を数える。
    fn adjacent_slab_count(&self, ni: usize, nj: usize) -> u32 {
        let key = ((ni as u32).min(nj as u32), (ni as u32).max(nj as u32));
        self.slab_edges
            .get(&key)
            .map(|(n, _)| *n)
            .unwrap_or(0)
            .min(2)
    }

    /// 梁側面のスラブ厚控除に使う厚さ [mm]（隣接版あり領域の `region_thickness`
    /// の max。辺が無ければ 0。梁せい d でクランプ）。
    fn adjacent_slab_t(&self, ni: usize, nj: usize, d: f64) -> f64 {
        let key = ((ni as u32).min(nj as u32), (ni as u32).max(nj as u32));
        let t = self.slab_edges.get(&key).map(|(_, t)| *t).unwrap_or(0.0);
        t.clamp(0.0, d)
    }
}

/// モデル全体の数量を集計する（部位別の概算数量）。
///
/// 壁の解析要素（`ElementKind::Wall`）は入力の正である `model` には存在しない
/// 生成物（D5）のため、内部で壁展開モデル
/// （[`squid_n_load::wall_expand::expand_wall_elements`]）を組み立てて数量拾いを
/// 行う（呼び出し元に展開を要求しない。忘れると壁の数量が積算から静かに
/// 消えるため。自重算定 `enumerate_self_weight` と同じ理由・同じパターン）。
pub fn compute_quantity_takeoff(model: &Model, cfg: &QuantityCfg) -> QuantityTakeoff {
    let expanded_storage;
    let model: &Model = if squid_n_load::wall_expand::model_has_wall_plates_to_expand(model) {
        let (expanded, _wall_index, _wall_report) =
            squid_n_load::wall_expand::expand_wall_elements(model);
        expanded_storage = expanded;
        &expanded_storage
    } else {
        model
    };
    let mut column_nodes: HashSet<usize> = HashSet::new();
    let mut beams_at_node: HashMap<usize, Vec<(usize, [f64; 2])>> = HashMap::new();
    let mut min_z = f64::INFINITY;

    for (idx, e) in model.elements.iter().enumerate() {
        if e.kind != ElementKind::Beam || e.nodes.len() < 2 {
            continue;
        }
        let ni = e.nodes[0].index();
        let nj = e.nodes[1].index();
        let (Some(a), Some(b)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
            continue;
        };
        let (ca, cb) = (a.coord, b.coord);
        min_z = min_z.min(ca[2]).min(cb[2]);
        if is_vertical_pair(ca, cb) {
            column_nodes.insert(ni);
            column_nodes.insert(nj);
        } else {
            let dx = cb[0] - ca[0];
            let dy = cb[1] - ca[1];
            let len = (dx * dx + dy * dy).sqrt();
            if len > 0.0 {
                let dir = [dx / len, dy / len];
                beams_at_node.entry(ni).or_default().push((idx, dir));
                beams_at_node
                    .entry(nj)
                    .or_default()
                    .push((idx, [-dir[0], -dir[1]]));
            }
        }
    }

    let mut slab_edges: HashMap<(u32, u32), (u32, f64)> = HashMap::new();
    for slab in &model.slabs {
        // 型枠控除は板厚が定まる床板（`slab_plate_thickness`）の辺だけ。断面未割当は控除しない。
        // 厚さは `slab_plate_thickness`。欠落時のみ建物一律 `slab_thickness`（通常は到達しない）。
        let Some(t) = model.slab_plate_thickness(slab) else {
            continue;
        };
        let t = t.max(0.0);
        let mut add_edge = |a: u32, b: u32| {
            let e = slab_edges.entry((a.min(b), a.max(b))).or_insert((0, 0.0));
            e.0 += 1;
            e.1 = e.1.max(t);
        };
        if let Some(boundary) = slab.boundary_nodes() {
            let n = boundary.len();
            if n < 3 {
                continue;
            }
            for i in 0..n {
                add_edge(
                    boundary[i].index() as u32,
                    boundary[(i + 1) % n].index() as u32,
                );
            }
        } else if let Some([a, b]) = slab.edge_nodes(0) {
            add_edge(a.index() as u32, b.index() as u32);
        }
    }

    let ctx = Ctx {
        model,
        cfg,
        column_nodes,
        slab_edges,
        beams_at_node,
        min_z,
    };

    let mut out = QuantityTakeoff::default();

    for (idx, elem) in model.elements.iter().enumerate() {
        match elem.kind {
            ElementKind::Beam if elem.nodes.len() >= 2 => {
                if let Some(item) = line_member_quantity(&ctx, idx, elem) {
                    out.items.push(item);
                }
            }
            ElementKind::Brace { .. } if elem.nodes.len() >= 2 => {
                if let Some(item) = brace_quantity(&ctx, elem) {
                    out.items.push(item);
                }
            }
            ElementKind::Wall | ElementKind::Shell if elem.nodes.len() >= 3 => {
                if let Some(item) = wall_quantity(&ctx, elem) {
                    out.items.push(item);
                }
            }
            _ => {}
        }
    }

    for slab in &model.slabs {
        if let Some(item) = slab_quantity(&ctx, slab) {
            out.items.push(item);
        }
    }

    for plate in &model.wall_plates {
        if !plate.is_attached() {
            continue; // 囲まれた壁版（Enclosed）は解析要素経由の wall_quantity が数える。
        }
        if let Some(item) = attached_wall_plate_quantity(&ctx, plate) {
            out.items.push(item);
        }
    }

    for sm in model.joists().chain(model.posts()) {
        if let Some(item) = secondary_member_quantity(&ctx, sm) {
            out.items.push(item);
        }
    }

    for (i, mw) in model.misc_walls.iter().enumerate() {
        if let Some(t) = mw.thickness {
            let l = ((mw.end[0] - mw.start[0]).powi(2) + (mw.end[1] - mw.start[1]).powi(2)).sqrt();
            let area = l * mw.height;
            out.items.push(MemberQuantity {
                elem: None,
                slab: None,
                label: format!("雑壁{}", i + 1),
                story: "-".to_string(),
                category: MemberCategory::MiscWall,
                structure: StructureKind::Rc,
                concrete_m3: area * t * 1e-9,
                formwork_m2: area * 2.0 * 1e-6,
                rebar: Vec::new(),
                steel: None,
                rebar_joints: 0.0,
            });
        }
    }

    out.notes = build_notes(model);
    out
}

/// 前提・未対応事項の注記を組み立てる。
fn build_notes(model: &Model) -> Vec<String> {
    let mut notes = vec![
        "鉄筋のフック・余長・重ね継手長さ・溶接継手長さは考慮しない（各部位共通事項）。".to_string(),
        "SRC のコンクリート体積は鉄骨体積を差し引かずに計算する（各部位共通事項）。".to_string(),
        "鉄骨重量は W=L×A×7.85（単位重量 7.85t/m³ 固定）による。".to_string(),
        "主筋は全長 1 断面（全断面）配筋として算定する（カットオフ筋の +15d は対象外）。".to_string(),
        "梁の腹筋・幅止筋は段数情報がモデルにないため計上しない。".to_string(),
        "壁筋はせん断補強筋比 ps による等価換算（仮定径 D10・定着 S=35d）。開口部補強筋は考慮しない。".to_string(),
        "基礎フーチングはモデルに定義がないため計上しない。".to_string(),
        "梁端ハンチは部材付帯情報（ハンチ長・せい増分・幅増分）から平均断面×ハンチ長で加算する（未入力の部材はハンチなし）。".to_string(),
        "鉄筋継手は個所数（圧接個所数）として集計する（梁 0.5 個所/本＋5m 毎 0.5、柱 1 個所/本＋7m 毎 1）。".to_string(),
        "二次部材の小梁・間柱も数量に含める。同じ両端を持つ実部材の小梁は線材側のみ数え、二次部材側では重複計上しない。".to_string(),
        "鉄骨継手（部材付帯情報の継手位置）は位置・種別の保持のみで、プレート・ボルト重量は計上しない。".to_string(),
    ];
    if !model.slabs.is_empty() {
        notes.push(
            "床厚は床ごとに割り当てた断面の板厚による（デッキスラブのデッキ高さ控除は未対応）。"
                .to_string(),
        );
        if model
            .floor_regions
            .iter()
            .any(|s| !s.joist_lines().is_empty())
        {
            notes.push(
                "床荷重分配用の小梁ライン（JoistLine）は断面情報がないため集計対象外。二次部材小梁・間柱（領域内および未割当）は節点間長で集計する。".to_string(),
            );
        }
    }
    notes
}

/// 解析要素ではない二次部材（小梁・間柱）の数量。実部材化済み小梁は線材側で数える。
fn secondary_member_quantity(ctx: &Ctx, sm: &SecondaryMember) -> Option<MemberQuantity> {
    let model = ctx.model;
    let a = sm.nodes[0];
    let b = sm.nodes[1];
    if sm.kind == SecondaryMemberKind::Joist
        && model.elements.iter().any(|e| {
            e.kind == ElementKind::Beam
                && e.nodes.len() == 2
                && ((e.nodes[0] == a && e.nodes[1] == b) || (e.nodes[0] == b && e.nodes[1] == a))
        })
    {
        return None;
    }
    let sec = model.sections.get(sm.section?.index())?;
    let mat = model.secondary_material(sm)?;
    let ni = a.index();
    let nj = b.index();
    let (ci, cj) = (model.nodes.get(ni)?.coord, model.nodes.get(nj)?.coord);
    let len = dist3(ci, cj);
    if len <= 0.0 {
        return None;
    }
    let structure = squid_n_core::structure_kind::structure_kind_of(Some(sec), Some(mat.category));
    let category = if sm.kind == SecondaryMemberKind::Post {
        MemberCategory::Column
    } else {
        MemberCategory::Joist
    };
    let story = ctx.story_name(if ci[2] <= cj[2] { ni } else { nj });
    let mut item = MemberQuantity {
        elem: None,
        slab: None,
        label: if sm.name.is_empty() {
            sec.name.clone()
        } else {
            sm.name.clone()
        },
        story,
        category,
        structure,
        concrete_m3: 0.0,
        formwork_m2: 0.0,
        rebar: Vec::new(),
        steel: None,
        rebar_joints: 0.0,
    };
    if structure == StructureKind::S || structure == StructureKind::Cft {
        let area = sec
            .shape
            .as_ref()
            .map(|s| s.calc_area())
            .unwrap_or(sec.area);
        item.steel = Some(SteelItem {
            section_name: sec.name.clone(),
            length_m: len / 1_000.0,
            weight_t: area * len * STEEL_UNIT_WEIGHT_T_PER_MM3,
        });
        return Some(item);
    }
    let (width, depth) = match sec.shape.as_ref() {
        Some(SectionShape::RcRect { b, d, .. } | SectionShape::SrcRect { b, d, .. }) => (*b, *d),
        _ => (sec.width, sec.depth),
    };
    if sm.kind == SecondaryMemberKind::Post {
        item.concrete_m3 = member::column_concrete_volume(width, depth, len) * 1e-9;
        item.formwork_m2 = member::column_formwork_area(width, depth, len) * 1e-6;
    } else {
        item.concrete_m3 = member::joist_concrete_volume(width, depth, len) * 1e-9;
        item.formwork_m2 = member::joist_formwork_area(width, depth, len) * 1e-6;
    }
    Some(item)
}

/// 線材（柱・大梁・小梁・基礎梁）の数量。
fn line_member_quantity(ctx: &Ctx, elem_idx: usize, elem: &ElementData) -> Option<MemberQuantity> {
    let model = ctx.model;
    let sec = model.sections.get(elem.section?.index())?;
    let mat = model.element_material(elem)?;
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    let (ci, cj) = (model.nodes.get(ni)?.coord, model.nodes.get(nj)?.coord);
    let len = dist3(ci, cj);
    if len <= 0.0 {
        return None;
    }
    let vertical = is_vertical_pair(ci, cj);
    let structure = squid_n_core::structure_kind::structure_kind_of(Some(sec), Some(mat.category));

    if vertical {
        Some(column_quantity(ctx, elem, sec, mat, ni, nj, len, structure))
    } else {
        Some(beam_quantity(
            ctx, elem_idx, elem, sec, ni, nj, ci, cj, len, structure,
        ))
    }
}

/// 柱の数量。
#[allow(clippy::too_many_arguments)]
fn column_quantity(
    ctx: &Ctx,
    elem: &ElementData,
    sec: &squid_n_core::model::Section,
    _mat: &squid_n_core::model::Material,
    ni: usize,
    nj: usize,
    h: f64,
    structure: StructureKind,
) -> MemberQuantity {
    // 柱長さは床上〜床上（＝節点間距離。フェイス控除しない）。
    let lower = if ctx.model.nodes[ni].coord[2] <= ctx.model.nodes[nj].coord[2] {
        ni
    } else {
        nj
    };
    let story = ctx.story_name(lower);

    let mut item = MemberQuantity {
        elem: Some(elem.id),
        slab: None,
        label: sec.name.clone(),
        story,
        category: MemberCategory::Column,
        structure,
        concrete_m3: 0.0,
        formwork_m2: 0.0,
        rebar: Vec::new(),
        steel: None,
        rebar_joints: 0.0,
    };

    match (structure, sec.shape.as_ref()) {
        (StructureKind::Rc | StructureKind::Src, shape) => {
            // 矩形: Dx×Dy×H・2(Dx+Dy)×H。円形は等価に πD²/4・πD×H。
            let (vol, form, rebar): (f64, f64, Option<&RcRebar>) = match shape {
                Some(SectionShape::RcRect { b, d, rebar }) => (
                    member::column_concrete_volume(*b, *d, h),
                    member::column_formwork_area(*b, *d, h),
                    Some(rebar),
                ),
                Some(SectionShape::SrcRect { b, d, rebar, .. }) => (
                    member::column_concrete_volume(*b, *d, h),
                    member::column_formwork_area(*b, *d, h),
                    Some(rebar),
                ),
                Some(SectionShape::RcCircle { d, rebar }) => (
                    std::f64::consts::PI * d * d / 4.0 * h,
                    std::f64::consts::PI * d * h,
                    Some(rebar),
                ),
                _ => (
                    member::column_concrete_volume(sec.width, sec.depth, h),
                    member::column_formwork_area(sec.width, sec.depth, h),
                    None,
                ),
            };
            item.concrete_m3 = vol * 1e-9;
            item.formwork_m2 = form * 1e-6;

            if let Some(rebar) = rebar {
                // 主筋: すべての柱は H で計算。X・Y 方向の重なりは差し引かない。
                let mut main_bars = 0u32;
                for bs in [&rebar.main_x, &rebar.main_y] {
                    if bs.count == 0 || bs.dia <= 0.0 {
                        continue;
                    }
                    main_bars += bs.count;
                    let total_len = bs.count as f64 * h;
                    item.rebar.push(RebarItem {
                        usage: RebarUsage::MainBar,
                        dia: Some(bs.dia),
                        total_length_m: total_len / 1_000.0,
                        weight_t: rebar::rebar_weight_t(total_len, bs.dia),
                    });
                }
                // フープ: 一組長さ nx×Dx+ny×Dy、本数 H/ピッチ。
                let (dx, dy, circle) = match shape {
                    Some(SectionShape::RcRect { b, d, .. })
                    | Some(SectionShape::SrcRect { b, d, .. }) => (*b, *d, false),
                    Some(SectionShape::RcCircle { d, .. }) => (*d, *d, true),
                    _ => (sec.width, sec.depth, false),
                };
                let sh = &rebar.shear;
                if sh.dia > 0.0 && sh.pitch > 0.0 {
                    let set_len = if circle {
                        // 円形柱: 円形フープ 1 本の周長 ≒ πD（組数分）。
                        sh.legs.max(1) as f64 * std::f64::consts::PI * dx
                    } else {
                        member::hoop_set_length(dx, dy, sh.legs.max(1), sh.legs.max(1))
                    };
                    let count = member::shear_bar_count(h, sh.pitch);
                    let total_len = set_len * count;
                    item.rebar.push(RebarItem {
                        usage: RebarUsage::Hoop,
                        dia: Some(sh.dia),
                        total_length_m: total_len / 1_000.0,
                        weight_t: rebar::rebar_weight_t(total_len, sh.dia),
                    });
                }
                // 鉄筋継手: 柱頭・柱脚通し主筋（同一断面のため全主筋）×
                // 階ごと 1 個所（階高 7m 以上は 7m 毎に +1）。
                item.rebar_joints = main_bars as f64 * member::column_joint_count(h);
            }

            // SRC: 内蔵鉄骨の重量を加算（コンクリートから差し引かない）。
            if structure == StructureKind::Src {
                if let Some(a_s) = sec.shape.as_ref().and_then(src_steel_area) {
                    item.steel = Some(SteelItem {
                        section_name: sec.name.clone(),
                        length_m: h / 1_000.0,
                        weight_t: a_s * h * STEEL_UNIT_WEIGHT_T_PER_MM3,
                    });
                }
            }
        }
        (StructureKind::S, _) => {
            let a = sec
                .shape
                .as_ref()
                .map(|s| s.calc_area())
                .unwrap_or(sec.area);
            item.steel = Some(SteelItem {
                section_name: sec.name.clone(),
                length_m: h / 1_000.0,
                weight_t: a * h * STEEL_UNIT_WEIGHT_T_PER_MM3,
            });
        }
        (StructureKind::Cft, shape) => {
            // 鋼管重量 + 充填コンクリート体積（型枠は不要）。
            let a = shape.map(|s| s.calc_area()).unwrap_or(sec.area);
            item.steel = Some(SteelItem {
                section_name: sec.name.clone(),
                length_m: h / 1_000.0,
                weight_t: a * h * STEEL_UNIT_WEIGHT_T_PER_MM3,
            });
            if let Some(ai) = shape.and_then(cft_infill_area) {
                item.concrete_m3 = ai * h * 1e-9;
            }
        }
    }
    item
}

/// 水平梁（大梁・小梁・基礎梁）の数量。
#[allow(clippy::too_many_arguments)]
fn beam_quantity(
    ctx: &Ctx,
    elem_idx: usize,
    elem: &ElementData,
    sec: &squid_n_core::model::Section,
    ni: usize,
    nj: usize,
    ci: [f64; 3],
    cj: [f64; 3],
    len: f64,
    structure: StructureKind,
) -> MemberQuantity {
    let model = ctx.model;
    // 分類: 最下層レベルの梁は基礎梁、柱に取り付く梁は大梁、それ以外は小梁。
    let at_base =
        (ci[2] - ctx.min_z).abs() < LEVEL_TOL_MM && (cj[2] - ctx.min_z).abs() < LEVEL_TOL_MM;
    let touches_column = ctx.column_nodes.contains(&ni) || ctx.column_nodes.contains(&nj);
    let category = if at_base && touches_column {
        MemberCategory::FoundationGirder
    } else if touches_column {
        MemberCategory::Girder
    } else {
        MemberCategory::Joist
    };

    let mut item = MemberQuantity {
        elem: Some(elem.id),
        slab: None,
        label: sec.name.clone(),
        story: ctx.story_name(ni),
        category,
        structure,
        concrete_m3: 0.0,
        formwork_m2: 0.0,
        rebar: Vec::new(),
        steel: None,
        rebar_joints: 0.0,
    };

    // 鉄骨系（S）は種類別の長さ・重量のみ（節点間距離で算定）。
    if structure == StructureKind::S || structure == StructureKind::Cft {
        let a = sec
            .shape
            .as_ref()
            .map(|s| s.calc_area())
            .unwrap_or(sec.area);
        item.steel = Some(SteelItem {
            section_name: sec.name.clone(),
            length_m: len / 1_000.0,
            weight_t: a * len * STEEL_UNIT_WEIGHT_T_PER_MM3,
        });
        return item;
    }

    // RC/SRC: 内法長さ（柱フェイス間）で算定。フェース距離が未算定なら節点間長。
    let lo = elem.rigid_zone.clear_span_from(len).unwrap_or(len);
    let (b, d, rebar): (f64, f64, Option<&RcRebar>) = match sec.shape.as_ref() {
        Some(SectionShape::RcRect { b, d, rebar }) => (*b, *d, Some(rebar)),
        Some(SectionShape::SrcRect { b, d, rebar, .. }) => (*b, *d, Some(rebar)),
        _ => (sec.width, sec.depth, None),
    };

    if category == MemberCategory::Joist {
        // 小梁: B×D×L、型枠 (B+2D)×L、鉄筋は体積比（主筋 0.8%・スターラップ 0.1%）。
        let vol = member::joist_concrete_volume(b, d, lo);
        item.concrete_m3 = vol * 1e-9;
        item.formwork_m2 = member::joist_formwork_area(b, d, lo) * 1e-6;
        let vol_m3 = vol * 1e-9;
        item.rebar.push(RebarItem {
            usage: RebarUsage::JoistMain,
            dia: None,
            total_length_m: 0.0,
            weight_t: vol_m3 * ctx.cfg.joist_main_ratio * STEEL_UNIT_WEIGHT_T_PER_M3,
        });
        item.rebar.push(RebarItem {
            usage: RebarUsage::JoistStirrup,
            dia: None,
            total_length_m: 0.0,
            weight_t: vol_m3 * ctx.cfg.joist_stirrup_ratio * STEEL_UNIT_WEIGHT_T_PER_M3,
        });
    } else {
        // 大梁・基礎梁。ハンチは部材付帯情報（`Model::member_detail_attrs`。
        // 剛性には影響しない）から取得し、増分寸法（せい増分・幅増分）を
        // ハンチ端の全幅 Bi・全せい Di へ換算して算定式へ渡す。
        let to_haunch = |h: &squid_n_core::model::Haunch| Haunch {
            b: b + h.width_increase.max(0.0),
            d: d + h.depth_increase.max(0.0),
            len: h.length,
        };
        let detail = model.member_detail(elem.id);
        let haunch_i: Option<Haunch> = detail
            .and_then(|dt| dt.haunch_i.as_ref())
            .filter(|h| h.length > 0.0)
            .map(to_haunch);
        let haunch_j: Option<Haunch> = detail
            .and_then(|dt| dt.haunch_j.as_ref())
            .filter(|h| h.length > 0.0)
            .map(to_haunch);
        let vol = member::girder_concrete_volume(b, d, lo, haunch_i, haunch_j);
        item.concrete_m3 = vol * 1e-9;

        // 型枠のスラブ厚控除: 隣接スラブ数（0/1/2）と隣接版厚で側面せいを決める。
        let n_adj = ctx.adjacent_slab_count(ni, nj);
        let t_slab = if n_adj == 0 {
            0.0
        } else {
            ctx.adjacent_slab_t(ni, nj, d)
        };
        let form = if category == MemberCategory::FoundationGirder {
            // 基礎梁: 側面 1 面＋底面（スラブ＝耐圧版があれば側面せいから控除）。
            let d_side = if n_adj >= 1 { d - t_slab } else { d };
            member::foundation_girder_formwork_area(b, d_side, lo, haunch_i, haunch_j)
        } else {
            let (d1, d2) = match n_adj {
                0 => (d, d),
                1 => (d, d - t_slab),
                _ => (d - t_slab, d - t_slab),
            };
            member::girder_formwork_area(b, d1, d2, lo, haunch_i, haunch_j)
        };
        item.formwork_m2 = form * 1e-6;

        if let Some(rebar) = rebar {
            // 主筋: 1 断面（全断面）配筋。端部条件は梁の連続性から判定する。
            let dir_xy = {
                let dx = cj[0] - ci[0];
                let dy = cj[1] - ci[1];
                let l = (dx * dx + dy * dy).sqrt();
                [dx / l, dy / l]
            };
            let mut main_bars = 0u32;
            for bs in [&rebar.main_x, &rebar.main_y] {
                if bs.count == 0 || bs.dia <= 0.0 {
                    continue;
                }
                main_bars += bs.count;
                let l2 = ctx.cfg.anchorage_dia_factor * bs.dia;
                let end_i = ctx.beam_bar_end(
                    elem_idx,
                    ni,
                    [-dir_xy[0], -dir_xy[1]],
                    elem.rigid_zone.face_i_or_zero(),
                    l2,
                );
                let end_j =
                    ctx.beam_bar_end(elem_idx, nj, dir_xy, elem.rigid_zone.face_j_or_zero(), l2);
                let bar_len = member::girder_main_bar_length(lo, end_i, end_j);
                let total_len = bs.count as f64 * bar_len;
                item.rebar.push(RebarItem {
                    usage: RebarUsage::MainBar,
                    dia: Some(bs.dia),
                    total_length_m: total_len / 1_000.0,
                    weight_t: rebar::rebar_weight_t(total_len, bs.dia),
                });
            }
            // スターラップ: 一組 2B+nD、本数 L/ピッチ。
            let sh = &rebar.shear;
            if sh.dia > 0.0 && sh.pitch > 0.0 {
                let set_len = member::stirrup_set_length(b, d, sh.legs.max(1));
                let count = member::shear_bar_count(lo, sh.pitch);
                let total_len = set_len * count;
                item.rebar.push(RebarItem {
                    usage: RebarUsage::Stirrup,
                    dia: Some(sh.dia),
                    total_length_m: total_len / 1_000.0,
                    weight_t: rebar::rebar_weight_t(total_len, sh.dia),
                });
            }
            // 鉄筋継手: 梁毎 0.5 個所/本（5m 以上は 5m 毎に +0.5）。
            item.rebar_joints = main_bars as f64 * member::beam_joint_count(lo);
        }

        // SRC 梁: 内蔵鉄骨重量を加算。
        if structure == StructureKind::Src {
            if let Some(a_s) = sec.shape.as_ref().and_then(src_steel_area) {
                item.steel = Some(SteelItem {
                    section_name: sec.name.clone(),
                    length_m: len / 1_000.0,
                    weight_t: a_s * len * STEEL_UNIT_WEIGHT_T_PER_MM3,
                });
            }
        }
    }
    item
}

/// ブレースの数量。
///
/// 長さは節点間距離。接合部のプレート・ボルト等は考慮しない。
fn brace_quantity(ctx: &Ctx, elem: &ElementData) -> Option<MemberQuantity> {
    let model = ctx.model;
    let sec = model.sections.get(elem.section?.index())?;
    let mat = model.element_material(elem)?;
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    let (ci, cj) = (model.nodes.get(ni)?.coord, model.nodes.get(nj)?.coord);
    let lb = dist3(ci, cj);
    if lb <= 0.0 {
        return None;
    }
    let structure = squid_n_core::structure_kind::structure_kind_of(Some(sec), Some(mat.category));
    let lower = if ci[2] <= cj[2] { ni } else { nj };

    let mut item = MemberQuantity {
        elem: Some(elem.id),
        slab: None,
        label: sec.name.clone(),
        story: ctx.story_name(lower),
        category: MemberCategory::Brace,
        structure,
        concrete_m3: 0.0,
        formwork_m2: 0.0,
        rebar: Vec::new(),
        steel: None,
        rebar_joints: 0.0,
    };
    let a = sec
        .shape
        .as_ref()
        .map(|s| s.calc_area())
        .unwrap_or(sec.area);
    if structure == StructureKind::Rc {
        // RC ブレース（稀）: コンクリート体積のみ計上する。
        item.concrete_m3 = a * lb * 1e-9;
    } else {
        item.steel = Some(SteelItem {
            section_name: sec.name.clone(),
            length_m: lb / 1_000.0,
            weight_t: a * lb * STEEL_UNIT_WEIGHT_T_PER_MM3,
        });
    }
    Some(item)
}

/// 4 節点壁の内法寸法 (長さ L, 高さ H) [mm]。
///
/// 周辺の柱・梁の断面寸法の半分を芯々寸法から控除する
/// （`squid-n-load::story_gen::wall_clear_area_factor` と同じ規則。
/// 側柱は `min(width, depth)/2`、上下梁は `depth/2` を控除）。
/// 4 節点壁でない場合は None。
fn wall_clear_dims(model: &Model, elem: &ElementData, pts: &[[f64; 3]]) -> Option<(f64, f64)> {
    if elem.kind != ElementKind::Wall || elem.nodes.len() != 4 || pts.len() != 4 {
        return None;
    }
    let n = 4usize;
    let (mut l_len, mut l_cnt) = (0.0, 0u32);
    let (mut h_len, mut h_cnt) = (0.0, 0u32);
    let (mut l_deduct, mut h_deduct) = (0.0, 0.0);
    for i in 0..n {
        let (a, b) = (elem.nodes[i], elem.nodes[(i + 1) % n]);
        let (pa, pb) = (pts[i], pts[(i + 1) % n]);
        let dz = (pb[2] - pa[2]).abs();
        let dh = ((pb[0] - pa[0]).powi(2) + (pb[1] - pa[1]).powi(2)).sqrt();
        let len = (dz * dz + dh * dh).sqrt();
        if len <= 0.0 {
            continue;
        }
        let member_sec = model
            .elements
            .iter()
            .find(|e| {
                e.kind == ElementKind::Beam && e.nodes.len() >= 2 && {
                    let (m0, m1) = (e.nodes[0], e.nodes[e.nodes.len() - 1]);
                    (m0 == a && m1 == b) || (m0 == b && m1 == a)
                }
            })
            .and_then(|e| e.section)
            .and_then(|sid| model.sections.get(sid.index()));
        if dz > dh {
            h_len += len;
            h_cnt += 1;
            if let Some(sec) = member_sec {
                l_deduct += sec.width.min(sec.depth).max(0.0) / 2.0;
            }
        } else {
            l_len += len;
            l_cnt += 1;
            if let Some(sec) = member_sec {
                h_deduct += sec.depth.max(0.0) / 2.0;
            }
        }
    }
    if l_cnt == 0 || h_cnt == 0 {
        return None;
    }
    let l = (l_len / l_cnt as f64 - l_deduct).max(0.0);
    let h = (h_len / h_cnt as f64 - h_deduct).max(0.0);
    Some((l, h))
}

/// 壁（耐震壁・フレーム内雑壁）の数量。
fn wall_quantity(ctx: &Ctx, elem: &ElementData) -> Option<MemberQuantity> {
    let model = ctx.model;
    let sec = model.sections.get(elem.section?.index())?;
    let (t, ps) = match sec.shape.as_ref() {
        Some(SectionShape::RcWall { thickness, ps }) => (*thickness, *ps),
        _ => (sec.thickness?, 0.0),
    };
    if t <= 0.0 {
        return None;
    }
    let pts: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|n| model.nodes.get(n.index()).map(|nd| nd.coord))
        .collect();
    if pts.len() < 3 {
        return None;
    }

    // 寸法のとり方: 幅は柱内法・高さは梁内法（4 節点壁）。それ以外は芯々。
    let (l_clear, h_clear, gross_area) = match wall_clear_dims(model, elem, &pts) {
        Some((l, h)) => (l, h, l * h),
        None => {
            let area = polygon_area_3d(&pts);
            let min_z = pts.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
            let max_z = pts.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
            let h = (max_z - min_z).max(0.0);
            let l = if h > 0.0 { area / h } else { 0.0 };
            (l, h, area)
        }
    };

    // 開口控除。
    let opening = model
        .wall_attrs
        .iter()
        .find(|a| a.elem == elem.id)
        .map(|a| a.total_opening_area())
        .unwrap_or(0.0);
    let net_area = (gross_area - opening).max(0.0);

    let lower = elem
        .nodes
        .iter()
        .map(|n| n.index())
        .min_by(|&a, &b| {
            model.nodes[a].coord[2]
                .partial_cmp(&model.nodes[b].coord[2])
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(elem.nodes[0].index());

    let mut item = MemberQuantity {
        elem: Some(elem.id),
        slab: None,
        label: sec.name.clone(),
        story: ctx.story_name(lower),
        category: MemberCategory::Wall,
        structure: StructureKind::Rc,
        concrete_m3: net_area * t * 1e-9,
        formwork_m2: net_area * 2.0 * 1e-6,
        rebar: Vec::new(),
        steel: None,
        rebar_joints: 0.0,
    };

    // 壁筋: 横筋 (L+2S)×W・縦筋 (H+2S)×h。壁の配筋はせん断補強筋比 ps のみ
    // 保持するため、本数×一本長さを ps による等価体積
    // （横筋 ps·t·H·(L+2S)、縦筋 ps·t·L·(H+2S)）に換算して算定する。
    // S = 35d（仮定径 D10）。開口部補強筋は考慮しない。
    if ps > 0.0 && l_clear > 0.0 && h_clear > 0.0 {
        let dia = ctx.cfg.assumed_wall_bar_dia;
        let s = ctx.cfg.anchorage_dia_factor * dia;
        let one = squid_n_core::section_shape::one_bar_area(dia);
        for (usage, span, other) in [
            (RebarUsage::WallHorizontal, l_clear, h_clear),
            (RebarUsage::WallVertical, h_clear, l_clear),
        ] {
            // 本数 = ps×t×直交方向長さ / 1本断面積（配筋列数を含む等価本数）。
            let count = ps * t * other / one;
            let total_len = member::wall_bar_length(span, s, count);
            item.rebar.push(RebarItem {
                usage,
                dia: Some(dia),
                total_length_m: total_len / 1_000.0,
                weight_t: rebar::rebar_weight_t(total_len, dia),
            });
        }
    }
    Some(item)
}

/// 取り付く壁版（`WallPlateShape::Attached`。パラペット・腰壁・垂れ壁・自立壁）の数量。
///
/// 解析要素を持たない（D5）ため `WallPlate` から直接算定する（[`slab_quantity`] と
/// 同じ位置づけ。エレメント経由の [`wall_quantity`] は `Enclosed`（4 節点の耐震壁）
/// のみを数える）。取り付く壁版は `OutOfFrameMiscWall`（フレーム外雑壁）の後継
/// （`dev_docs/specs/用語集.md`）のため、カテゴリは同じ `MemberCategory::MiscWall` を使う。
/// ラベルは囲まれた壁と同じく割り当てた断面の符号（`wall_quantity` と同じ）。
fn attached_wall_plate_quantity(ctx: &Ctx, plate: &WallPlate) -> Option<MemberQuantity> {
    let model = ctx.model;
    let WallPlateShape::Attached { anchor, .. } = &plate.shape else {
        return None;
    };
    let anchor_nodes = match anchor {
        RegionAnchor::Line { nodes, .. } => *nodes,
        RegionAnchor::FloorRegion { nodes, .. } => *nodes,
        // 壁の取付き先としては使わない（`WallPlate::boundary_coords` と同じ理由）。
        RegionAnchor::Point(_) => return None,
    };
    let sec = model.wall_plate_section(plate)?;
    let t = model.wall_plate_thickness(plate)?;
    let pts = plate.boundary_coords(model)?;
    if pts.len() < 3 {
        return None;
    }
    let area = polygon_area_3d(&pts);
    let net_area = (area - plate.total_opening_area()).max(0.0);

    let lower = anchor_nodes
        .iter()
        .map(|n| n.index())
        .min_by(|&a, &b| {
            model.nodes[a].coord[2]
                .partial_cmp(&model.nodes[b].coord[2])
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(anchor_nodes[0].index());

    Some(MemberQuantity {
        elem: None,
        slab: None,
        label: sec.name.clone(),
        story: ctx.story_name(lower),
        category: MemberCategory::MiscWall,
        structure: StructureKind::Rc,
        concrete_m3: net_area * t * 1e-9,
        formwork_m2: net_area * 2.0 * 1e-6,
        rebar: Vec::new(),
        steel: None,
        rebar_joints: 0.0,
    })
}

/// 床（一般・片持ち・出隅・入隅）の数量。
fn slab_quantity(ctx: &Ctx, slab: &Slab) -> Option<MemberQuantity> {
    let model = ctx.model;
    let pts: Vec<[f64; 3]> = slab.boundary_coords(model)?;
    if pts.len() < 3 {
        return None;
    }
    let area = polygon_area_3d(&pts);
    // 床板ごとの板厚は断面から解決する（建物一律の `slab_thickness` は
    // 剛性計算に見込む厚さであり、実際の板厚とは別概念）。
    let t = model.slab_plate_thickness(slab).unwrap_or(0.0);
    // 主架構に取り付く領域（片持ち・バルコニー・出隅）は片持ち床として拾う。
    let category = if slab.is_attached() {
        MemberCategory::CantileverSlab
    } else {
        MemberCategory::Slab
    };
    let vol = area * t;
    let vol_m3 = vol * 1e-9;

    let mut item = MemberQuantity {
        elem: None,
        slab: Some(slab.id),
        label: format!("S{}", slab.id.0),
        story: slab
            .reference_node()
            .map(|n| ctx.story_name(n.index()))
            .unwrap_or_else(|| "-".to_string()),
        category,
        structure: StructureKind::Rc,
        concrete_m3: vol_m3,
        formwork_m2: area * 1e-6,
        rebar: Vec::new(),
        steel: None,
        rebar_joints: 0.0,
    };
    if vol_m3 > 0.0 {
        // 鉄筋重量: 床コンクリート体積×1.0%（片持ち床も同率）を鋼比重で重量化。
        item.rebar.push(RebarItem {
            usage: RebarUsage::SlabBar,
            dia: None,
            total_length_m: 0.0,
            weight_t: vol_m3 * ctx.cfg.slab_rebar_ratio * STEEL_UNIT_WEIGHT_T_PER_M3,
        });
    }
    Some(item)
}
