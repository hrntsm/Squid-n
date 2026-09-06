//! モデルの照会（JSON 出力・クエリ・解析）関数。

use super::*;

pub fn get_model_json(state: &ServerState) -> String {
    serde_json::to_string(&state.model).unwrap_or_default()
}

/// `model.query` の中核ロジック（feature 非依存・テスト可能）。
pub fn query_model(model: &Model, kind: &str, filter: Option<&str>) -> Vec<serde_json::Value> {
    use serde_json::json;
    let items: Vec<serde_json::Value> = match kind {
        "node" | "nodes" => model
            .nodes
            .iter()
            .map(|n| {
                json!({
                    "id": n.id.0,
                    "coord": n.coord,
                    "story": n.story.map(|s| s.0),
                })
            })
            .collect(),
        "member" | "members" | "element" | "elements" => model
            .elements
            .iter()
            .map(|e| {
                let mut v = json!({
                    "id": e.id.0,
                    "kind": format!("{:?}", e.kind),
                    "nodes": e.nodes.iter().map(|n| n.0).collect::<Vec<_>>(),
                    "section": e.section.map(|s| s.0),
                    "material": model.element_material(e).map(|m| m.id.0),
                });
                if let Some(detail) = model.member_detail(e.id) {
                    let haunch_json = |h: &squid_n_core::model::Haunch| {
                        json!({
                            "length": h.length,
                            "depth_increase": h.depth_increase,
                            "width_increase": h.width_increase,
                        })
                    };
                    let obj = v.as_object_mut().expect("json!({...}) is always an object");
                    if let Some(h) = &detail.haunch_i {
                        obj.insert("haunch_i".to_string(), haunch_json(h));
                    }
                    if let Some(h) = &detail.haunch_j {
                        obj.insert("haunch_j".to_string(), haunch_json(h));
                    }
                    if !detail.joints.is_empty() {
                        obj.insert(
                            "joints".to_string(),
                            json!(detail
                                .joints
                                .iter()
                                .map(|j| json!({
                                    "distance": j.distance,
                                    "kind": format!("{:?}", j.kind),
                                }))
                                .collect::<Vec<_>>()),
                        );
                    }
                }
                v
            })
            .collect(),
        "section" | "sections" => model
            .sections
            .iter()
            .map(|s| {
                json!({
                    "id": s.id.0,
                    "name": s.name,
                    "floor": s.floor,
                    "area": s.area,
                    "iy": s.iy,
                    "iz": s.iz,
                    "material": s.material.map(|m| m.0),
                    "rebar_material": s.rebar_material.map(|m| m.0),
                    "shear_rebar_material": s.shear_rebar_material.map(|m| m.0),
                    "steel_material": s.steel_material.map(|m| m.0),
                })
            })
            .collect(),
        "wall_plate" | "wall_plates" => model
            .wall_plates
            .iter()
            .map(|p| {
                let shape = match &p.shape {
                    squid_n_core::model::WallPlateShape::Enclosed { boundary } => json!({
                        "kind": "Enclosed",
                        "boundary": boundary.iter().map(|n| n.0).collect::<Vec<_>>(),
                    }),
                    squid_n_core::model::WallPlateShape::Attached { anchor, extent } => json!({
                        "kind": "Attached",
                        "anchor": anchor,
                        "extent": extent,
                        "resolved_extent": model.wall_plate_extent(p),
                    }),
                };
                json!({
                    "id": p.id.0,
                    "shape": shape,
                    "section": p.section.map(|s| s.0),
                    "opening_area": p.opening_area,
                    "opening_weight": p.opening_weight,
                    "slit": {
                        "column_face": p.slit.column_face,
                        "beam_face": p.slit.beam_face,
                    },
                    "openings": p.openings,
                    "loads": p.loads,
                    "becomes_element": model.wall_plate_becomes_element(p),
                })
            })
            .collect(),
        "slab" | "slabs" => model
            .slabs
            .iter()
            .map(|s| {
                let shape = match &s.shape {
                    squid_n_core::model::SlabShape::Enclosed { boundary } => json!({
                        "kind": "Enclosed",
                        "boundary": boundary.iter().map(|n| n.0).collect::<Vec<_>>(),
                    }),
                    squid_n_core::model::SlabShape::Attached { anchor, extent } => json!({
                        "kind": "Attached",
                        "anchor": anchor,
                        "extent": extent,
                    }),
                };
                json!({
                    "id": s.id.0,
                    "shape": shape,
                    "section": s.plate.section.map(|sid| sid.0),
                    "loads": s.plate.loads,
                    "usage": s.plate.usage,
                    "method": s.plate.method,
                    "one_way": s.plate.one_way,
                })
            })
            .collect(),
        "floor_region" | "floor_regions" => model
            .floor_regions
            .iter()
            .map(|r| {
                json!({
                    "id": r.id.0,
                    "name": r.name,
                    "boundary": r.boundary.iter().map(|n| n.0).collect::<Vec<_>>(),
                    "slab_ids": r.slab_ids.iter().map(|s| s.0).collect::<Vec<_>>(),
                    "secondary_joists": r.secondary_joists,
                })
            })
            .collect(),
        "wall_region" | "wall_regions" => model
            .wall_regions
            .iter()
            .map(|r| {
                json!({
                    "id": r.id.0,
                    "name": r.name,
                    "boundary": r.boundary.iter().map(|n| n.0).collect::<Vec<_>>(),
                    "wall_plate_ids": r.wall_plate_ids.iter().map(|p| p.0).collect::<Vec<_>>(),
                    "posts": r.posts,
                })
            })
            .collect(),
        "unassigned_joist" | "unassigned_joists" => serde_json::to_value(&model.unassigned_joists)
            .ok()
            .and_then(|v| v.as_array().cloned())
            .unwrap_or_default(),
        "unassigned_post" | "unassigned_posts" => serde_json::to_value(&model.unassigned_posts)
            .ok()
            .and_then(|v| v.as_array().cloned())
            .unwrap_or_default(),
        "secondary_joist" | "secondary_joists" => model
            .joists()
            .map(|sm| serde_json::to_value(sm).unwrap_or(json!(null)))
            .collect(),
        _ => vec![],
    };
    match filter {
        Some(f) if !f.is_empty() => items
            .into_iter()
            .filter(|v| v.to_string().contains(f))
            .collect(),
        _ => items,
    }
}

/// 数量積算（feature 非依存・テスト可能）。
pub fn quantity_takeoff_json(model: &Model, group_by: Option<&str>) -> serde_json::Value {
    use serde_json::json;
    use squid_n_design_jp::quantity::{compute_quantity_takeoff, QuantityCfg, QuantityTotals};

    let q = compute_quantity_takeoff(model, &QuantityCfg::default());
    let totals_json = |t: &QuantityTotals| {
        json!({
            "concrete_m3": t.concrete_m3,
            "formwork_m2": t.formwork_m2,
            "rebar_t": t.rebar_t,
            "steel_t": t.steel_t,
            "rebar_joints": t.rebar_joints,
        })
    };
    let rows: Vec<serde_json::Value> = match group_by.unwrap_or("category") {
        "story" | "stories" => q
            .totals_by_story()
            .iter()
            .map(|(story, t)| {
                let mut v = totals_json(t);
                v["story"] = json!(story);
                v
            })
            .collect(),
        "steel" => q
            .steel_by_section()
            .iter()
            .map(|s| {
                json!({
                    "section": s.section_name,
                    "length_m": s.length_m,
                    "weight_t": s.weight_t,
                })
            })
            .collect(),
        "rebar" => q
            .rebar_by_dia()
            .iter()
            .map(|(dia, len, w)| {
                json!({
                    "dia_mm": dia,
                    "length_m": len,
                    "weight_t": w,
                })
            })
            .collect(),
        "detail" | "items" => q
            .items
            .iter()
            .map(|it| {
                json!({
                    "elem": it.elem.map(|e| e.0),
                    "slab": it.slab.map(|s| s.0),
                    "label": it.label,
                    "story": it.story,
                    "category": it.category.label(),
                    "structure": it.structure.label(),
                    "concrete_m3": it.concrete_m3,
                    "formwork_m2": it.formwork_m2,
                    "rebar_t": it.rebar_weight_t(),
                    "steel_t": it.steel_weight_t(),
                    "rebar_joints": it.rebar_joints,
                })
            })
            .collect(),
        _ => q
            .totals_by_category()
            .iter()
            .map(|(cat, t)| {
                let mut v = totals_json(t);
                v["category"] = json!(cat.label());
                v
            })
            .collect(),
    };
    json!({
        "rows": rows,
        "totals": totals_json(&q.totals()),
        "notes": q.notes,
    })
}

/// 解析の実処理（feature 非依存・テスト可能）。
pub fn analyze_model(model: &Model) -> Result<String, String> {
    let mut model = model.clone();
    squid_n_job::prepare::apply_rigid_zones_and_panels(&mut model);
    let Some(lc_id) = model.load_cases.first().map(|lc| lc.id) else {
        return Err("荷重ケースがありません".into());
    };
    let result =
        squid_n_job::compute::compute_linear_static(model, lc_id).map_err(|e| e.to_string())?;
    Ok(serde_json::to_string(&result.disp).unwrap_or_default())
}

/// `analyze_model` の `ServerState` 経由の薄いラッパ。
pub fn analyze(state: &mut ServerState) -> Result<String, String> {
    analyze_model(&state.model)
}
