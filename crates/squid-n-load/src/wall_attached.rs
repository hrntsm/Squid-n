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
//!   自立壁、D17）の壁版の自重を、所属する床領域の床板へ等価な面荷重として上乗せする
//!   ための追加強度 [N/mm²] を床板 ID ごとに返す。床領域が床板を1枚も持たない場合
//!   （版なし領域）は等価面荷重へならせないため、総重量を保存する目的で取付き線の
//!   両端節点への集中荷重（フォールバック）を [`BeamLoad`] 列として返す。
//!
//! # 既知の近似（総重量は保存するが、位置の精度には限界がある）
//!
//! 張り出し量 `extent[0] != extent[1]`（台形の壁。取付き線に沿って高さが変わる）の場合、
//! `Anchor` 分布は台形ではなく取付き線全長で均した等分布（`w = 総重量/len`）とする。
//! `Columns` 集中は区間中点 `t_mid` の単純梁反力按分とし、真の面積重心（張り出しの
//! 大きい側へずれる）ではなく区間中点を使う。**いずれも総重量（`wall_plate_self_weight`
//! が Newell の公式で正確に求める値）はそのまま保存されるが、取付き線に沿った
//! 位置の精度は落ちる。** 床側の同じ形（[`RegionAnchor::Line`] ＋ `extent` 非対称）を
//! 扱う `floor::distribute_attached` も同じ近似を持ち、そちらの doc に同じ限界が
//! 明記されている。壁側だけの新しい近似ではない。

use std::collections::HashMap;

use squid_n_core::geom::{polygon_area_3d, vec3::dist as dist3};
use squid_n_core::ids::{ElemId, FloorRegionId, NodeId, SlabId};
use squid_n_core::model::{LoadTransfer, Model, RegionAnchor, WallPlateShape};

use crate::floor::{fem_uniform, BeamLoad, Cmq, LoadShape, LoadTarget};

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
        let WallPlateShape::Attached { anchor, .. } = &plate.shape else {
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
                // 取付き線の区間 span にのみ載る等分布荷重。梁全長への希釈はしない
                // （危険側の近似を避ける。dig 2026-08-27 Q2=A）。
                let w = total / len;
                loads.push(BeamLoad {
                    elem: ElemId(u32::MAX),
                    target: LoadTarget::Span {
                        nodes: *nodes,
                        t: *span,
                    },
                    shape: LoadShape::Uniform { w },
                    cmq: fem_uniform(w, len),
                });
            }
            LoadTransfer::Columns => {
                // 区間中点 t_mid での単純梁反力按分（floor::distribute_attached の
                // Columns 分岐と同じ規則）。
                let t_mid = 0.5 * (span[0] + span[1]);
                push_node_load(&mut loads, nodes[0], total * (1.0 - t_mid));
                push_node_load(&mut loads, nodes[1], total * t_mid);
            }
        }
    }
    loads
}

/// 床領域の床板合計面積 [mm²]（床板を1枚も持たない、または境界座標が引けない
/// 場合は 0.0）。
fn region_slab_area(model: &Model, slab_ids: &[SlabId]) -> f64 {
    slab_ids
        .iter()
        .filter_map(|&id| model.slab(id))
        .filter_map(|s| s.boundary_coords(model))
        .map(|pts| polygon_area_3d(&pts))
        .sum()
}

/// 「床領域」アンカーの取り付く壁版（自立壁）の自重を配分する（D17）。
///
/// 戻り値は `(床板ごとの追加面荷重強度 [N/mm²], フォールバックの BeamLoad 列)`。
/// 追加強度は、所属する床領域内の全床板へ床板面積によらず同一の値を返す
/// （D17「等価な面荷重へならす」を、区画内の床板ごとの強度差を無視する形で実装したもの。
/// 区画内で面荷重強度が異なる床板が混在する場合の扱いは
/// `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §8.1 の残課題と同じ性質の近似）。
///
/// 所属する床領域が床板を1枚も持たない、または床板の合計面積が0の場合（版なし領域）は
/// 等価面荷重へならす先がないため、総重量を保存する目的で取付き線の両端節点（`nodes`）へ
/// 半分ずつの集中荷重として返す。
pub fn floor_region_wall_extra_intensity(model: &Model) -> (HashMap<SlabId, f64>, Vec<BeamLoad>) {
    struct Pending {
        region: FloorRegionId,
        total: f64,
        nodes: [NodeId; 2],
    }

    let mut total_by_region: HashMap<FloorRegionId, f64> = HashMap::new();
    let mut pending: Vec<Pending> = Vec::new();
    for plate in &model.wall_plates {
        let WallPlateShape::Attached { anchor, .. } = &plate.shape else {
            continue;
        };
        let RegionAnchor::FloorRegion { region, nodes } = anchor else {
            continue;
        };
        let Some(total) = model.wall_plate_self_weight(plate, model) else {
            continue;
        };
        if total <= 0.0 {
            continue;
        }
        *total_by_region.entry(*region).or_insert(0.0) += total;
        pending.push(Pending {
            region: *region,
            total,
            nodes: *nodes,
        });
    }
    if total_by_region.is_empty() {
        return (HashMap::new(), Vec::new());
    }

    let mut extra_intensity: HashMap<SlabId, f64> = HashMap::new();
    let mut area_by_region: HashMap<FloorRegionId, f64> = HashMap::new();
    for region in &model.floor_regions {
        let Some(&extra) = total_by_region.get(&region.id) else {
            continue;
        };
        let area = region_slab_area(model, &region.slab_ids);
        area_by_region.insert(region.id, area);
        if area <= 0.0 {
            continue; // 版なし領域はフォールバックへ回す（下記）。
        }
        let dw = extra / area;
        for &sid in &region.slab_ids {
            *extra_intensity.entry(sid).or_insert(0.0) += dw;
        }
    }

    let mut fallback = Vec::new();
    for p in &pending {
        let has_area = area_by_region.get(&p.region).copied().unwrap_or(0.0) > 0.0;
        if has_area {
            continue;
        }
        push_node_load(&mut fallback, p.nodes[0], p.total * 0.5);
        push_node_load(&mut fallback, p.nodes[1], p.total * 0.5);
    }

    (extra_intensity, fallback)
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

    /// 台形の壁（`extent[0] != extent[1]`）でも、位置の精度は落ちる（モジュール doc の
    /// 「既知の近似」参照）が総重量は保存される。`Anchor`・`Columns` の両方で確認する。
    #[test]
    fn asymmetric_extent_conserves_total_weight_for_both_transfers() {
        let mut m = model_with_line_anchor_nodes();
        let extent = [500.0, 1500.0]; // 台形（始端 500mm・終端 1500mm）。

        let anchor_plate = line_attached_plate([0.0, 1.0], extent, LoadTransfer::Anchor);
        let anchor_total = m
            .wall_plate_self_weight(&anchor_plate, &m)
            .expect("自重が求まる");
        m.wall_plates.push(anchor_plate);
        let anchor_loads = attached_wall_beam_loads(&m);
        assert_eq!(anchor_loads.len(), 1);
        let LoadShape::Uniform { w } = anchor_loads[0].shape else {
            panic!("Uniform を期待");
        };
        let len = 4000.0;
        assert!(
            (w * len - anchor_total).abs() / anchor_total < 1e-9,
            "台形でも総重量は保存されるはず: w×len={} total={}",
            w * len,
            anchor_total
        );

        m.wall_plates.clear();
        let columns_plate = line_attached_plate([0.0, 1.0], extent, LoadTransfer::Columns);
        let columns_total = m
            .wall_plate_self_weight(&columns_plate, &m)
            .expect("自重が求まる");
        m.wall_plates.push(columns_plate);
        let columns_loads = attached_wall_beam_loads(&m);
        let sum: f64 = columns_loads
            .iter()
            .map(|bl| match bl.shape {
                LoadShape::Point { p, .. } => p,
                _ => panic!("Point を期待"),
            })
            .sum();
        assert!(
            (sum - columns_total).abs() / columns_total < 1e-9,
            "台形でも総重量は保存されるはず: sum={sum} total={columns_total}"
        );
    }

    /// 「床領域」アンカー（自立壁）: 床板を持つ区画では、区画内の全床板へ
    /// 同一の追加強度（総重量÷床板合計面積）が上乗せされる。
    #[test]
    fn floor_region_anchor_adds_equivalent_intensity_to_slabs() {
        let mut m = model_with_line_anchor_nodes();
        m.nodes.push(mk_node(2, 0.0, 0.0, 0.0));
        m.nodes.push(mk_node(3, 4000.0, 0.0, 0.0));
        m.nodes.push(mk_node(4, 4000.0, 4000.0, 0.0));
        m.nodes.push(mk_node(5, 0.0, 4000.0, 0.0));
        let boundary = vec![NodeId(2), NodeId(3), NodeId(4), NodeId(5)];
        let slab = Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed {
                boundary: boundary.clone(),
            },
            plate: SlabPlate {
                section: None,
                loads: Vec::new(),
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
            },
        };
        let mut region = FloorRegion::new(FloorRegionId(0), boundary);
        region.slab_ids.push(slab.id);
        m.slabs.push(slab);
        m.floor_regions.push(region);

        let plate = WallPlate {
            id: squid_n_core::ids::WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    region: FloorRegionId(0),
                    nodes: [NodeId(0), NodeId(1)],
                },
                extent: [1000.0, 1000.0],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);

        let (extra, fallback) = floor_region_wall_extra_intensity(&m);
        assert!(fallback.is_empty(), "床板があるためフォールバック不要");
        let dw = extra.get(&SlabId(0)).copied().expect("床板への追加強度");
        let slab_area = 4000.0 * 4000.0;
        assert!(
            (dw - total / slab_area).abs() / (total / slab_area) < 1e-9,
            "dw={dw}"
        );
    }

    /// 「床領域」アンカーの所属先が版なし領域（床板0枚）の場合、
    /// 総重量を保存するため取付き線の両端節点への集中荷重にフォールバックする。
    #[test]
    fn floor_region_anchor_falls_back_to_nodes_when_no_slabs() {
        let mut m = model_with_line_anchor_nodes();
        m.nodes.push(mk_node(2, 0.0, 0.0, 0.0));
        m.nodes.push(mk_node(3, 4000.0, 0.0, 0.0));
        m.nodes.push(mk_node(4, 4000.0, 4000.0, 0.0));
        m.nodes.push(mk_node(5, 0.0, 4000.0, 0.0));
        let boundary = vec![NodeId(2), NodeId(3), NodeId(4), NodeId(5)];
        // 床板を1枚も持たない版なし床領域。
        m.floor_regions
            .push(FloorRegion::new(FloorRegionId(0), boundary));

        let plate = WallPlate {
            id: squid_n_core::ids::WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    region: FloorRegionId(0),
                    nodes: [NodeId(0), NodeId(1)],
                },
                extent: [1000.0, 1000.0],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let total = m.wall_plate_self_weight(&plate, &m).expect("自重が求まる");
        m.wall_plates.push(plate);

        let (extra, fallback) = floor_region_wall_extra_intensity(&m);
        assert!(extra.is_empty(), "床板がないため追加強度は発生しない");
        assert_eq!(fallback.len(), 2);
        let sum: f64 = fallback
            .iter()
            .map(|bl| match bl.shape {
                LoadShape::Point { p, .. } => p,
                _ => panic!("Point を期待"),
            })
            .sum();
        assert!((sum - total).abs() / total < 1e-9, "総重量を保存すること");
    }
}
