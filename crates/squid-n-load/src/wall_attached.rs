//! 取り付く壁版（[`WallPlateShape::Attached`]）の自重配分。
//!
//! `Attached` な壁版（パラペット・腰壁・垂れ壁・自立壁）は解析要素を持たない（D5）ため、
//! 壁展開（[`crate::wall_expand`]）を経由する自重算定（`story_gen::enumerate_self_weight`）
//! では検出できず、これまで地震用重量・長期応力解析のDLのどちらからも自重が
//! 抜け落ちていた（`dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §9・§5.22）。
//! 本モジュールは、取付き先（`RegionAnchor`）の種別ごとに定めた伝達規則（D16）に沿って、
//! 壁版の自重（[`Model::wall_plate_self_weight`]）を配分する。重量の算定式自体は
//! 既存関数をそのまま使い、本モジュールが新たに担うのは配分のみである。
//!
//! - [`attached_wall_beam_loads`] — 「線」アンカー（[`RegionAnchor::Line`]）の壁版を
//!   [`BeamLoad`] へ変換する。取付き先の梁へ分布させる（[`LoadTransfer::Anchor`]、
//!   梁の全長ではなく取付き線の区間 `span` にのみ載せる）か、取付き線両端の柱へ
//!   集中させる（[`LoadTransfer::Columns`]）かは壁版側の設定に従う。床の取り付く床板
//!   （`SlabShape::Attached`）と同じ幾何解決パイプライン
//!   （`squid-n-job::auto_loads::slab_load_case_content`）に合流させる想定で、
//!   `span`（部分区間）を梁全長へ薄めず正確に尊重する（危険側の近似を避けるため。
//!   `AGENTS.md`「実装方針」参照）。
//! - [`floor_region_wall_extra_intensity`] — 「床領域」アンカー（[`RegionAnchor::FloorRegion`]、
//!   自立壁、D17）の壁版の自重を、壁が載っている床領域の床板へ等価な面荷重として
//!   上乗せするための追加強度 [N/mm²] を床板 ID ごとに返す。分配先の床領域 ID は
//!   保存せず、[`Model::self_standing_wall_coverage`] が位置から都度求める。強度の
//!   分母は床の分配と同じ XY 投影面積とする。荷重を流せる床の上に載っていない部分
//!   は分配せず、解析前チェック（`squid-n-solver::precheck`）がエラーで止める。
//!   両端節点への集中荷重へ逃がすフォールバックは持たない（非構造節点への節点荷重は
//!   `DofMap` が無視し、長期 DL から黙って消える危険側だったため）。
//!
//! 張り出し量 `extent[0] != extent[1]`（台形の壁）では、線荷重強度を張り出し高さに
//! 比例させる（[`LoadShape::Linear`]）。柱按分・密度経路の節点集中は同じ台形の
//! 面積重心へ置く。開口は総重量のスケールにだけ効き、開口位置は見ない。

use std::collections::HashMap;

use squid_n_core::geom::vec3::dist as dist3;
use squid_n_core::ids::{ElemId, FloorRegionId, NodeId, SlabId};
use squid_n_core::model::{LoadTransfer, Model, RegionAnchor, WallPlateShape};

use crate::floor::{fem_linear, fem_uniform, polygon_area, BeamLoad, Cmq, LoadShape, LoadTarget};

/// 台形（始端高さ `h0`、終端高さ `h1`）の底辺に沿った無次元重心（始端 = 0、終端 = 1）。
/// 両方の高さが実質 0 なら中点 0.5。
fn trapezoid_centroid_s(h0: f64, h1: f64) -> f64 {
    let sum = h0 + h1;
    if sum <= 1e-12 {
        0.5
    } else {
        (h0 + 2.0 * h1) / (3.0 * sum)
    }
}

/// 取付き線上の集中位置（無次元）。矩形なら区間中点、台形なら面積重心。
fn line_resultant_t(span: [f64; 2], extent: [f64; 2]) -> f64 {
    let s = trapezoid_centroid_s(extent[0].abs(), extent[1].abs());
    span[0] + (span[1] - span[0]) * s
}

/// 線アンカー分布の形状。高さが両端で実質等しければ等分布、異なれば線形変化。
/// 総荷重 `∫ w dx = total` を保存する。
fn line_load_shape(total: f64, len: f64, extent: [f64; 2]) -> (LoadShape, Cmq) {
    let h0 = extent[0].abs();
    let h1 = extent[1].abs();
    let sum_h = h0 + h1;
    if sum_h <= 1e-12 || (h0 - h1).abs() <= 1e-12 * sum_h.max(1.0) {
        let w = total / len;
        (LoadShape::Uniform { w }, fem_uniform(w, len))
    } else {
        let w_i = total * 2.0 * h0 / (sum_h * len);
        let w_j = total * 2.0 * h1 / (sum_h * len);
        (LoadShape::Linear { w_i, w_j }, fem_linear(w_i, w_j, len))
    }
}

/// 節点 `node` への集中荷重（[`LoadTarget::Node`]）を1件積む。総量が実質0なら積まない。
fn push_node_load(loads: &mut Vec<BeamLoad>, node: NodeId, total: f64) {
    if total.abs() <= 1e-9 {
        return;
    }
    loads.push(BeamLoad {
        elem: ElemId(u32::MAX),
        target: LoadTarget::Node(node),
        shape: LoadShape::Point { p: total, x: 0.0 },
        cmq: Cmq {
            c_i: 0.0,
            c_j: 0.0,
            q_i: total,
            q_j: 0.0,
        },
    });
}

/// 「線」アンカーの取り付く壁版（パラペット・腰壁・垂れ壁で梁に取り付くもの）の
/// 自重を [`BeamLoad`] へ変換する（D16）。
pub fn attached_wall_beam_loads(model: &Model) -> Vec<BeamLoad> {
    let mut loads = Vec::new();
    for plate in &model.wall_plates {
        let WallPlateShape::Attached { anchor, extent } = &plate.shape else {
            continue;
        };
        let RegionAnchor::Line {
            nodes,
            span,
            transfer,
        } = anchor
        else {
            continue;
        };
        let Some(total) = model.wall_plate_self_weight(plate, model) else {
            continue;
        };
        if total <= 0.0 {
            continue;
        }
        // 境界座標（span 適用済みの実座標2点＋張り出し高さ2点）の先頭2点が
        // 取付き線上でこの壁版が実際に覆う区間の両端（`WallPlate::extrude_up`）。
        let Some(coords) = plate.boundary_coords(model) else {
            continue;
        };
        let len = dist3(coords[0], coords[1]);
        if len <= 1e-9 {
            continue;
        }
        match transfer {
            LoadTransfer::Anchor => {
                // 取付き線の区間 span にのみ載る分布。梁全長への希釈はしない
                // （危険側の近似を避ける。dig 2026-08-27 Q2=A）。
                // 矩形は等分布、台形は張り出し高さに比例する線形変化。
                let (shape, cmq) = line_load_shape(total, len, *extent);
                loads.push(BeamLoad {
                    elem: ElemId(u32::MAX),
                    target: LoadTarget::Span {
                        nodes: *nodes,
                        t: *span,
                    },
                    shape,
                    cmq,
                });
            }
            LoadTransfer::Columns => {
                // 台形の面積重心（矩形なら区間中点）での単純梁反力按分。
                let t = line_resultant_t(*span, *extent);
                push_node_load(&mut loads, nodes[0], total * (1.0 - t));
                push_node_load(&mut loads, nodes[1], total * t);
            }
        }
    }
    loads
}

/// 床領域の床板合計面積 [mm²]（床板を1枚も持たない、または境界座標が引けない
/// 場合は 0.0）。
///
/// 床の分配（[`crate::floor::polygon_area`]）と同じ XY 投影面積を使う。
/// 3 次元面積（`polygon_area_3d`）で割ると、分配側が XY 面積に強度を掛けるため
/// 総重量が `(A_xy / A_3d)` 倍に縮小し、傾斜床では地震用重量・梁荷重が過小
/// （危険側）になる。鉛直に近い床板は XY 面積が 0 になり、等価面荷重へならせない。
/// その床領域は [`Model::self_standing_wall_coverage`] の候補から外れる。
fn region_slab_area(model: &Model, slab_ids: &[SlabId]) -> f64 {
    slab_ids
        .iter()
        .filter_map(|&id| model.slab(id))
        .filter_map(|s| s.boundary_coords(model))
        .map(|pts| polygon_area(&pts))
        .sum()
}

/// 節点重量配列へ集中荷重を足す（添字が範囲外なら無視）。
fn add_node_weight(node_weight: &mut [f64], node: NodeId, w: f64) {
    let i = node.index();
    if i < node_weight.len() && w.abs() > 1e-9 {
        node_weight[i] += w;
    }
}

/// 取り付く壁版の自重を、密度からの地震用重量直接算入（`include_density_self_weight
/// = true`。DL ケースが無いモデル向け）へ載せる。
///
/// 標準構成では自重は「DL」へ同期され、本関数は呼ばれない（二重計上防止。
/// フレーム外雑壁の [`crate::story_gen::accumulate_misc_wall_weight`] と同じ位置づけ）。
/// DL が無い経路でここを呼ばないと、取り付く壁版の自重が地震用重量から欠落する
/// （囲まれた壁・フレーム外雑壁は `enumerate_self_weight` / 雑壁集計に入るのに、
/// 取り付く壁版だけが抜けていた）。
///
/// 配分は総重量を保存する節点集中（線アンカーは台形の面積重心、矩形なら区間中点の
/// 単純梁反力按分。床領域アンカーも張り出し高さが両端で異なれば同じ重心比、
/// 矩形なら両端半分ずつ）。梁の曲げへの載り方は DL 同期側
/// （[`attached_wall_beam_loads`] / 等価面荷重）が担い、こちらは階重量の総和が
/// 抜けないことだけを保証する。
pub fn accumulate_attached_wall_seismic_weight(model: &Model, node_weight: &mut [f64]) {
    for plate in &model.wall_plates {
        let WallPlateShape::Attached { anchor, extent } = &plate.shape else {
            continue;
        };
        let Some(total) = model.wall_plate_self_weight(plate, model) else {
            continue;
        };
        if total <= 0.0 {
            continue;
        }
        match anchor {
            RegionAnchor::Line { nodes, span, .. } => {
                let t = line_resultant_t(*span, *extent);
                add_node_weight(node_weight, nodes[0], total * (1.0 - t));
                add_node_weight(node_weight, nodes[1], total * t);
            }
            RegionAnchor::FloorRegion { nodes, .. } => {
                let s = trapezoid_centroid_s(extent[0].abs(), extent[1].abs());
                add_node_weight(node_weight, nodes[0], total * (1.0 - s));
                add_node_weight(node_weight, nodes[1], total * s);
            }
            RegionAnchor::Point(_) => {}
        }
    }
}

/// 「床領域」アンカーの取り付く壁版（自立壁）の自重を配分する（D17）。
///
/// 戻り値は床板ごとの追加面荷重強度 [N/mm²]。追加強度は、載っている床領域内の
/// 全床板へ床板面積によらず同一の値を返す（D17「等価な面荷重へならす」を、
/// 床領域内の床板ごとの強度差を無視する形で実装したもの。床領域内で面荷重強度が
/// 異なる床板が混在する場合の扱いは
/// `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §8.1 の残課題と同じ性質の近似）。
///
/// # 荷重を渡す床領域は保存せず、壁の位置から都度求める
///
/// 分配先は [`squid_n_core::model::Model::self_standing_wall_coverage`] が解決する。
/// 床領域をまたぐ壁は境界で内部的に分割し、それぞれの床領域へ台形面積比で配る。
///
/// # 荷重の行き先が無い壁は分配しない（フォールバックしない）
///
/// どの床領域にも載らない部分（`uncovered`）を持つ自立壁は、荷重の行き先が無い
/// モデルの不備であり、**解析前チェック（`squid-n-solver::precheck`）がエラーで止める**。
/// ここでは覆われている部分だけを配り、覆われていない部分は配らない。
///
/// かつては総重量を保存する目的で取付き線の両端節点への集中荷重へ逃がしていたが、
/// この経路は**危険側だった**: 自立壁の両端はどの部材にも接続されない自由節点で
/// ありうる。そこへの節点荷重は非構造節点として `DofMap` が無視するため
/// （`squid_n_core::dof::DofMap::build`）、地震用重量には算入されるのに長期 DL の
/// 応力解析からは黙って消え、梁がその重量を負担しないまま設計される。
/// 不備を下流で糊付けせず、解析前チェックで止める形へ改めた。
pub fn floor_region_wall_extra_intensity(model: &Model) -> HashMap<SlabId, f64> {
    let mut total_by_region: HashMap<FloorRegionId, f64> = HashMap::new();
    for plate in &model.wall_plates {
        let Some(total) = model.wall_plate_self_weight(plate, model) else {
            continue;
        };
        if total <= 0.0 {
            continue;
        }
        let Some(cov) = model.self_standing_wall_coverage(plate) else {
            continue;
        };
        for (region, frac) in cov.per_region {
            *total_by_region.entry(region).or_insert(0.0) += total * frac;
        }
    }
    if total_by_region.is_empty() {
        return HashMap::new();
    }

    let mut extra_intensity: HashMap<SlabId, f64> = HashMap::new();
    for region in &model.floor_regions {
        let Some(&extra) = total_by_region.get(&region.id) else {
            continue;
        };
        // 面積は床の分配と同じ XY 投影（3 次元面積で割ると、分配側が XY 面積に
        // 強度を掛けるため総重量が縮み、傾斜床で危険側になる）。
        // `self_standing_wall_coverage` が候補を「XY 面積が正の床領域」に絞るため、
        // ここへ来る床領域の面積は必ず正になる。
        let area = region_slab_area(model, &region.slab_ids);
        if area <= 0.0 {
            continue;
        }
        let dw = extra / area;
        for &sid in &region.slab_ids {
            *extra_intensity.entry(sid).or_insert(0.0) += dw;
        }
    }

    extra_intensity
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{
        DistributionMethod, FloorRegion, Material, MaterialCategory, Node, Section, Slab,
        SlabPlate, SlabShape, WallPlate,
    };

    const THICKNESS_MM: f64 = 150.0;
    const DENSITY_TON_MM3: f64 = 2.4e-9;

    fn mk_node(id: u32, x: f64, y: f64, z: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    /// 節点0・1（取付き線、z=3000）＋壁厚150mmの断面・材料を持つモデル。
    fn model_with_line_anchor_nodes() -> Model {
        let mut m = Model {
            nodes: vec![
                mk_node(0, 0.0, 0.0, 3000.0),
                mk_node(1, 4000.0, 0.0, 3000.0),
            ],
            ..Default::default()
        };
        m.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: DENSITY_TON_MM3,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        m.sections.push(Section {
            id: SectionId(0),
            name: "壁 t150".into(),
            area: 0.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 0.0,
            width: 0.0,
            as_y: 1.0,
            as_z: 1.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(THICKNESS_MM),
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        m
    }

    fn line_attached_plate(span: [f64; 2], extent: [f64; 2], transfer: LoadTransfer) -> WallPlate {
        WallPlate {
            id: squid_n_core::ids::WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span,
                    transfer,
                },
                extent,
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        }
    }

    /// `Anchor`（分布）・全区間: 取付き線全長への等分布荷重1件になり、
    /// `w×len` が壁版の自重総量と一致する（総和保存）。
    #[test]
    fn anchor_transfer_full_span_is_uniform_over_full_line() {
        let mut m = model_with_line_anchor_nodes();
        let plate = line_attached_plate([0.0, 1.0], [1000.0, 1000.0], LoadTransfer::Anchor);
        m.wall_plates.push(plate.clone());
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");

        let loads = attached_wall_beam_loads(&m);
        assert_eq!(loads.len(), 1);
        let bl = &loads[0];
        assert_eq!(
            bl.target,
            LoadTarget::Span {
                nodes: [NodeId(0), NodeId(1)],
                t: [0.0, 1.0],
            }
        );
        let LoadShape::Uniform { w } = bl.shape else {
            panic!("Uniform を期待");
        };
        let len = 4000.0;
        assert!(
            (w * len - total).abs() / total < 1e-9,
            "w×len={} total={}",
            w * len,
            total
        );
    }

    /// `Anchor`（分布）・部分区間: `span` をそのまま `t` へ引き継ぎ、梁全長へ薄めない
    /// （危険側の近似を避ける。dig 2026-08-27 Q2=A）。
    #[test]
    fn anchor_transfer_partial_span_keeps_span_and_conserves_total() {
        let mut m = model_with_line_anchor_nodes();
        let span = [0.25, 0.75];
        let plate = line_attached_plate(span, [1000.0, 1000.0], LoadTransfer::Anchor);
        m.wall_plates.push(plate.clone());
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");

        let loads = attached_wall_beam_loads(&m);
        assert_eq!(loads.len(), 1);
        let bl = &loads[0];
        assert_eq!(
            bl.target,
            LoadTarget::Span {
                nodes: [NodeId(0), NodeId(1)],
                t: span,
            }
        );
        let LoadShape::Uniform { w } = bl.shape else {
            panic!("Uniform を期待");
        };
        // 実際に覆う長さは全長4000mmの半分（span 0.25〜0.75）。
        let len = 4000.0 * (span[1] - span[0]);
        assert!(
            (w * len - total).abs() / total < 1e-9,
            "w×len={} total={}",
            w * len,
            total
        );
    }

    /// `Columns`（集中）・全区間: 取付き線両端の柱2本へ半分ずつ。
    #[test]
    fn columns_transfer_full_span_splits_evenly() {
        let mut m = model_with_line_anchor_nodes();
        let plate = line_attached_plate([0.0, 1.0], [1000.0, 1000.0], LoadTransfer::Columns);
        m.wall_plates.push(plate.clone());
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");

        let loads = attached_wall_beam_loads(&m);
        assert_eq!(loads.len(), 2);
        for bl in &loads {
            let LoadTarget::Node(n) = bl.target else {
                panic!("Node を期待");
            };
            assert!(n == NodeId(0) || n == NodeId(1));
            let LoadShape::Point { p, .. } = bl.shape else {
                panic!("Point を期待");
            };
            assert!((p - total / 2.0).abs() / total < 1e-9);
        }
    }

    /// `Columns`（集中）・偏った区間中点: 単純梁反力按分（`t_mid` から離れた側が少ない）。
    #[test]
    fn columns_transfer_uses_span_midpoint_ratio() {
        let mut m = model_with_line_anchor_nodes();
        let span = [0.0, 0.5]; // t_mid = 0.25
        let plate = line_attached_plate(span, [1000.0, 1000.0], LoadTransfer::Columns);
        m.wall_plates.push(plate.clone());
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");

        let loads = attached_wall_beam_loads(&m);
        assert_eq!(loads.len(), 2);
        let get = |n: NodeId| -> f64 {
            loads
                .iter()
                .find(|bl| bl.target == LoadTarget::Node(n))
                .map(|bl| match bl.shape {
                    LoadShape::Point { p, .. } => p,
                    _ => panic!("Point を期待"),
                })
                .unwrap()
        };
        let (r0, r1) = (get(NodeId(0)), get(NodeId(1)));
        assert!((r0 - total * 0.75).abs() / total < 1e-9, "r0={r0}");
        assert!((r1 - total * 0.25).abs() / total < 1e-9, "r1={r1}");
    }

    /// 台形の壁（`extent[0] != extent[1]`）は高さに比例する線形分布・面積重心按分。
    /// 総重量も保存する。
    #[test]
    fn asymmetric_extent_uses_height_proportional_linear_and_centroid() {
        let mut m = model_with_line_anchor_nodes();
        let extent = [500.0, 1500.0]; // 台形（始端 500mm・終端 1500mm）。
        let h0 = 500.0;
        let h1 = 1500.0;
        let len = 4000.0;
        let s = (h0 + 2.0 * h1) / (3.0 * (h0 + h1)); // 7/12

        let anchor_plate = line_attached_plate([0.0, 1.0], extent, LoadTransfer::Anchor);
        let anchor_total = m
            .wall_plate_self_weight(&anchor_plate, &m)
            .expect("自重が求まる");
        m.wall_plates.push(anchor_plate);
        let anchor_loads = attached_wall_beam_loads(&m);
        assert_eq!(anchor_loads.len(), 1);
        let LoadShape::Linear { w_i, w_j } = anchor_loads[0].shape else {
            panic!("Linear を期待");
        };
        assert!(
            (w_i / w_j - h0 / h1).abs() < 1e-12,
            "強度比が張り出し高さ比と一致しない: w_i={w_i} w_j={w_j}"
        );
        let integral = len * (w_i + w_j) / 2.0;
        assert!(
            (integral - anchor_total).abs() / anchor_total < 1e-9,
            "台形でも総重量は保存されるはず: integral={integral} total={anchor_total}"
        );

        m.wall_plates.clear();
        let columns_plate = line_attached_plate([0.0, 1.0], extent, LoadTransfer::Columns);
        let columns_total = m
            .wall_plate_self_weight(&columns_plate, &m)
            .expect("自重が求まる");
        m.wall_plates.push(columns_plate);
        let columns_loads = attached_wall_beam_loads(&m);
        let get = |n: NodeId| -> f64 {
            columns_loads
                .iter()
                .find(|bl| bl.target == LoadTarget::Node(n))
                .map(|bl| match bl.shape {
                    LoadShape::Point { p, .. } => p,
                    _ => panic!("Point を期待"),
                })
                .unwrap()
        };
        let (r0, r1) = (get(NodeId(0)), get(NodeId(1)));
        assert!(
            (r0 - columns_total * (1.0 - s)).abs() / columns_total < 1e-9,
            "r0={r0}"
        );
        assert!(
            (r1 - columns_total * s).abs() / columns_total < 1e-9,
            "r1={r1}"
        );
        assert!((r0 + r1 - columns_total).abs() / columns_total < 1e-9);
    }

    /// 台形＋部分区間: 重心は区間内の相対位置に置く（区間中点ではない）。
    #[test]
    fn trapezoid_columns_partial_span_uses_centroid_in_span() {
        let mut m = model_with_line_anchor_nodes();
        let span = [0.0, 0.5];
        let extent = [500.0, 1500.0];
        let s = (500.0 + 2.0 * 1500.0) / (3.0 * 2000.0); // 7/12
        let t = span[0] + (span[1] - span[0]) * s; // 7/24
        let plate = line_attached_plate(span, extent, LoadTransfer::Columns);
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);
        let loads = attached_wall_beam_loads(&m);
        let get = |n: NodeId| -> f64 {
            loads
                .iter()
                .find(|bl| bl.target == LoadTarget::Node(n))
                .map(|bl| match bl.shape {
                    LoadShape::Point { p, .. } => p,
                    _ => panic!("Point を期待"),
                })
                .unwrap()
        };
        let (r0, r1) = (get(NodeId(0)), get(NodeId(1)));
        assert!(
            (r0 - total * (1.0 - t)).abs() / total < 1e-9,
            "r0={r0} t={t}"
        );
        assert!((r1 - total * t).abs() / total < 1e-9, "r1={r1}");
    }

    /// 自立壁が載る床領域を作る。境界は Z=3000（壁の節点と同じレベル）の矩形で、
    /// `slab` が真なら床板を 1 枚持たせる。`z_far` は奥側 2 点の Z（傾斜床の検証用）。
    fn push_floor_region(m: &mut Model, x: [f64; 2], id: u32, slab: bool, z_far: f64) {
        let n0 = m.nodes.len() as u32;
        for (dx, y, z) in [
            (x[0], -2000.0, 3000.0),
            (x[1], -2000.0, 3000.0),
            (x[1], 2000.0, z_far),
            (x[0], 2000.0, z_far),
        ] {
            m.nodes.push(mk_node(m.nodes.len() as u32, dx, y, z));
        }
        let boundary = vec![NodeId(n0), NodeId(n0 + 1), NodeId(n0 + 2), NodeId(n0 + 3)];
        let mut region = FloorRegion::new(FloorRegionId(id), boundary.clone());
        if slab {
            let sid = SlabId(m.slabs.len() as u32);
            m.slabs.push(Slab {
                id: sid,
                shape: SlabShape::Enclosed { boundary },
                plate: SlabPlate {
                    section: None,
                    loads: Vec::new(),
                    usage: None,
                    method: DistributionMethod::TriTrapezoid,
                    one_way: None,
                },
            });
            region.slab_ids.push(sid);
        }
        m.floor_regions.push(region);
    }

    /// 自立壁の壁版（節点 0-1 の間、Z=3000）。
    fn self_standing_plate() -> WallPlate {
        WallPlate {
            id: squid_n_core::ids::WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    nodes: [NodeId(0), NodeId(1)],
                },
                extent: [1000.0, 1000.0],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        }
    }

    /// 「床領域」アンカー（自立壁）: 床板を持つ床領域では、床領域内の全床板へ
    /// 同一の追加強度（総重量÷床板合計面積）が上乗せされる。
    #[test]
    fn floor_region_anchor_adds_equivalent_intensity_to_slabs() {
        let mut m = model_with_line_anchor_nodes();
        // 壁（X=0..4000）を完全に含む床領域（X=-1000..5000）。
        push_floor_region(&mut m, [-1000.0, 5000.0], 0, true, 3000.0);
        let plate = self_standing_plate();
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);

        let extra = floor_region_wall_extra_intensity(&m);
        let dw = extra.get(&SlabId(0)).copied().expect("床板への追加強度");
        let slab_area = 6000.0 * 4000.0;
        assert!(
            (dw - total / slab_area).abs() / (total / slab_area) < 1e-9,
            "dw={dw}"
        );
    }

    /// 床領域をまたぐ自立壁は境界で内部分割し、それぞれの床領域の床板へ配る。
    /// 総重量は保存する。
    #[test]
    fn floor_region_anchor_splits_across_regions() {
        let mut m = model_with_line_anchor_nodes();
        // 壁は X=0..4000。床領域を X=-1000..2000 と X=2000..5000 に置き、
        // 壁は X=2000 で 2:2 に分かれる。
        push_floor_region(&mut m, [-1000.0, 2000.0], 0, true, 3000.0);
        push_floor_region(&mut m, [2000.0, 5000.0], 1, true, 3000.0);
        let plate = self_standing_plate();
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);

        let extra = floor_region_wall_extra_intensity(&m);
        assert_eq!(extra.len(), 2, "両方の床板へ配ること: {extra:?}");
        // 各床板の面積は 3000×4000。強度×面積の和が総重量に一致する。
        let sum: f64 = extra.values().map(|dw| dw * 3000.0 * 4000.0).sum();
        assert!(
            (sum - total).abs() / total < 1e-9,
            "総重量を保存すること sum={sum} total={total}"
        );
        // 半分ずつなので強度も等しい。
        let v: Vec<f64> = extra.values().copied().collect();
        assert!((v[0] - v[1]).abs() / v[0] < 1e-9, "{extra:?}");
    }

    /// 版なし床領域（床板 0 枚）は「力を流す先が無い」ため覆いとみなさず、
    /// 分配しない（フォールバックもしない。解析前チェックが止める）。
    #[test]
    fn floor_region_without_slabs_distributes_nothing() {
        let mut m = model_with_line_anchor_nodes();
        push_floor_region(&mut m, [-1000.0, 5000.0], 0, false, 3000.0);
        let plate = self_standing_plate();
        m.wall_plates.push(plate);

        let extra = floor_region_wall_extra_intensity(&m);
        assert!(
            extra.is_empty(),
            "床板が無ければ分配しない（危険側のフォールバックはしない）: {extra:?}"
        );
    }

    /// 床領域の外へはみ出す自立壁は、覆われている分だけを配る。
    #[test]
    fn floor_region_anchor_distributes_only_covered_part() {
        let mut m = model_with_line_anchor_nodes();
        // 壁は X=0..4000。床領域は X=-1000..2000 だけなので半分がはみ出す。
        push_floor_region(&mut m, [-1000.0, 2000.0], 0, true, 3000.0);
        let plate = self_standing_plate();
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);

        let extra = floor_region_wall_extra_intensity(&m);
        let dw = extra.get(&SlabId(0)).copied().expect("追加強度");
        let carried = dw * 3000.0 * 4000.0;
        assert!(
            (carried - total * 0.5).abs() / total < 1e-9,
            "覆われた半分だけを配ること carried={carried} total={total}"
        );
    }

    /// 床分配は XY 投影面積に強度を掛ける。追加強度の分母も同じ面積でないと
    /// 傾斜床で総重量が縮小する（危険側）。
    #[test]
    fn floor_region_extra_intensity_uses_xy_area_so_distribution_conserves() {
        let mut m = model_with_line_anchor_nodes();
        // 奥側 2 点を Z=3000 から持ち上げ、XY 面積は変えずに 3 次元面積だけ大きくする。
        // レベル判定は境界節点の Z の平均なので、平均が 3000 のままになるよう
        // 手前 2 点を下げる（±500）。
        let n0 = m.nodes.len() as u32;
        for (x, y, z) in [
            (-1000.0, -2000.0, 2500.0),
            (5000.0, -2000.0, 2500.0),
            (5000.0, 2000.0, 3500.0),
            (-1000.0, 2000.0, 3500.0),
        ] {
            m.nodes.push(mk_node(m.nodes.len() as u32, x, y, z));
        }
        let boundary = vec![NodeId(n0), NodeId(n0 + 1), NodeId(n0 + 2), NodeId(n0 + 3)];
        let mut region = FloorRegion::new(FloorRegionId(0), boundary.clone());
        region.slab_ids.push(SlabId(0));
        m.slabs.push(Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed { boundary },
            plate: SlabPlate {
                section: None,
                loads: Vec::new(),
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
            },
        });
        m.floor_regions.push(region);

        let plate = self_standing_plate();
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);

        let extra = floor_region_wall_extra_intensity(&m);
        let dw = extra.get(&SlabId(0)).copied().expect("追加強度");
        let xy_area = 6000.0 * 4000.0;
        assert!(
            (dw * xy_area - total).abs() / total < 1e-9,
            "dw×A_xy={} total={}（3次元面積で割ると縮小する）",
            dw * xy_area,
            total
        );
    }

    #[test]
    fn accumulate_seismic_weight_conserves_line_and_floor_region() {
        let mut m = model_with_line_anchor_nodes();
        let plate = line_attached_plate([0.0, 0.5], [1000.0, 1000.0], LoadTransfer::Anchor);
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);
        let mut nw = vec![0.0; m.nodes.len()];
        accumulate_attached_wall_seismic_weight(&m, &mut nw);
        assert!((nw[0] - total * 0.75).abs() / total < 1e-9);
        assert!((nw[1] - total * 0.25).abs() / total < 1e-9);
    }

    #[test]
    fn accumulate_seismic_weight_uses_trapezoid_centroid() {
        let mut m = model_with_line_anchor_nodes();
        let plate = line_attached_plate([0.0, 1.0], [500.0, 1500.0], LoadTransfer::Anchor);
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);
        let s = (500.0 + 2.0 * 1500.0) / (3.0 * 2000.0);
        let mut nw = vec![0.0; m.nodes.len()];
        accumulate_attached_wall_seismic_weight(&m, &mut nw);
        assert!((nw[0] - total * (1.0 - s)).abs() / total < 1e-9);
        assert!((nw[1] - total * s).abs() / total < 1e-9);
    }
}
