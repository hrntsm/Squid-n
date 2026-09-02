//! モデル編集（P8 T2: `model.edit`）。GUI と同一の `EditCommand` + `UndoStack` 経路。

use super::*;
use squid_n_core::ids::{FloorRegionId, NodeId, SectionId, SlabId, WallPlateId, WallRegionId};
use squid_n_core::model::SecondaryMember;
use squid_n_core::model::{
    DistributionMethod, OneWayDir, RegionAnchor, SlabPlate, SlabUsage, WallOpening,
};
use squid_n_edit::EditCommand;
use squid_n_edit::{
    AddAttachedSlab, AddAttachedWallPlate, AddEnclosedWallPlate, AddSlab, AddUnassignedJoist,
    AddUnassignedPost, DeleteSlab, DeleteUnassignedJoist, DeleteUnassignedPost, DeleteWallPlate,
    SetAttachedAnchor, SetAttachedExtent, SetAttachedWallPlateAnchor, SetAttachedWallPlateExtent,
    SetFloorRegionName, SetFloorRegionSecondaryJoists, SetSlabOneWay, SetSlabSection, SetSlabUsage,
    SetWallPlateAttrs, SetWallPlateSection, SetWallRegionPosts,
};

#[derive(Debug, Clone, serde::Serialize, schemars::JsonSchema)]
pub struct WriteResult {
    /// 破壊的操作の識別子（クライアント確認・Undo 参照用）。
    pub op_id: String,
    /// コマンドがモデルへ適用されたか。
    pub applied: bool,
    /// Undo 履歴に積まれたか。
    pub undoable: bool,
    /// 何が変わるか（確認用）。
    pub summary: String,
}

/// JSON 引数から `EditCommand` を生成する。`command` キーで種別を指定する。
///
/// MCP ツール引数は `command` をトップレベルに置く（P8 T2）。
/// `{ "body": { "command": ... } }` も受け付ける。
pub fn parse_edit_command(value: &serde_json::Value) -> Result<Box<dyn EditCommand>, String> {
    let value = resolve_edit_payload(value);
    let command = value
        .get("command")
        .and_then(|v| v.as_str())
        .ok_or("command が必要です")?;
    match command {
        "AddEnclosedWallPlate" => {
            let boundary = parse_node_ids(value.get("boundary").ok_or("boundary が必要です")?)?;
            if boundary.len() != 4 {
                return Err("boundary は 4 節点を指定してください".into());
            }
            let mut dedup = boundary.clone();
            dedup.sort_by_key(|n| n.0);
            dedup.dedup();
            if dedup.len() != 4 {
                return Err("boundary に重複した節点があります".into());
            }
            Ok(Box::new(AddEnclosedWallPlate {
                boundary,
                section: parse_optional_section_id(value.get("section"))?,
                opening_area: parse_f64(value.get("opening_area"), "opening_area")?.unwrap_or(0.0),
                opening_weight: parse_f64(value.get("opening_weight"), "opening_weight")?
                    .unwrap_or(0.0),
            }))
        }
        "AddAttachedWallPlate" => {
            let anchor: RegionAnchor =
                serde_json::from_value(value.get("anchor").ok_or("anchor が必要です")?.clone())
                    .map_err(|e| format!("anchor の解析に失敗: {e}"))?;
            let extent = parse_f64_pair(value.get("extent"), "extent")?;
            Ok(Box::new(AddAttachedWallPlate {
                anchor,
                extent,
                section: parse_optional_section_id(value.get("section"))?,
                opening_area: parse_f64(value.get("opening_area"), "opening_area")?.unwrap_or(0.0),
                opening_weight: parse_f64(value.get("opening_weight"), "opening_weight")?
                    .unwrap_or(0.0),
            }))
        }
        "DeleteWallPlate" => Ok(Box::new(DeleteWallPlate {
            id: parse_wall_plate_id(value.get("id").ok_or("id が必要です")?)?,
        })),
        "SetWallPlateSection" => Ok(Box::new(SetWallPlateSection {
            id: parse_wall_plate_id(value.get("id").ok_or("id が必要です")?)?,
            section: parse_optional_section_id(value.get("section"))?,
        })),
        "SetWallPlateAttrs" => {
            let openings = match value.get("openings") {
                None | Some(serde_json::Value::Null) => Vec::new(),
                Some(v) => serde_json::from_value::<Vec<WallOpening>>(v.clone())
                    .map_err(|e| format!("openings の解析に失敗: {e}"))?,
            };
            Ok(Box::new(SetWallPlateAttrs {
                id: parse_wall_plate_id(value.get("id").ok_or("id が必要です")?)?,
                opening_area: parse_f64(value.get("opening_area"), "opening_area")?.unwrap_or(0.0),
                opening_weight: parse_f64(value.get("opening_weight"), "opening_weight")?
                    .unwrap_or(0.0),
                openings,
                slit: parse_slit(value.get("slit"))?,
            }))
        }
        "SetAttachedWallPlateExtent" => Ok(Box::new(SetAttachedWallPlateExtent {
            id: parse_wall_plate_id(value.get("id").ok_or("id が必要です")?)?,
            extent: parse_f64_pair(value.get("extent"), "extent")?,
        })),
        "SetAttachedWallPlateAnchor" => {
            let anchor: RegionAnchor =
                serde_json::from_value(value.get("anchor").ok_or("anchor が必要です")?.clone())
                    .map_err(|e| format!("anchor の解析に失敗: {e}"))?;
            Ok(Box::new(SetAttachedWallPlateAnchor {
                id: parse_wall_plate_id(value.get("id").ok_or("id が必要です")?)?,
                anchor,
            }))
        }
        "AddSlab" => {
            let boundary = parse_node_ids(value.get("boundary").ok_or("boundary が必要です")?)?;
            if boundary.len() < 3 {
                return Err("boundary は 3 節点以上を指定してください".into());
            }
            let loads = match value.get("loads") {
                None | Some(serde_json::Value::Null) => Vec::new(),
                Some(v) => serde_json::from_value(v.clone())
                    .map_err(|e| format!("loads の解析に失敗: {e}"))?,
            };
            Ok(Box::new(AddSlab {
                boundary,
                loads,
                method: parse_distribution_method(value.get("method"))?,
                usage: parse_optional_enum::<SlabUsage>(value.get("usage"))?,
                section: parse_optional_section_id(value.get("section"))?,
            }))
        }
        "AddAttachedSlab" => {
            let anchor: RegionAnchor =
                serde_json::from_value(value.get("anchor").ok_or("anchor が必要です")?.clone())
                    .map_err(|e| format!("anchor の解析に失敗: {e}"))?;
            let extent = parse_f64_pair(value.get("extent"), "extent")?;
            let plate: SlabPlate = if let Some(v) = value.get("plate") {
                serde_json::from_value(v.clone()).map_err(|e| format!("plate の解析に失敗: {e}"))?
            } else {
                SlabPlate {
                    section: parse_optional_section_id(value.get("section"))?,
                    loads: match value.get("loads") {
                        None | Some(serde_json::Value::Null) => Vec::new(),
                        Some(v) => serde_json::from_value(v.clone())
                            .map_err(|e| format!("loads の解析に失敗: {e}"))?,
                    },
                    usage: parse_optional_enum::<SlabUsage>(value.get("usage"))?,
                    method: parse_distribution_method(value.get("method"))?,
                    one_way: parse_optional_enum::<OneWayDir>(value.get("one_way"))?,
                }
            };
            Ok(Box::new(AddAttachedSlab {
                anchor,
                extent,
                plate,
            }))
        }
        "DeleteSlab" => Ok(Box::new(DeleteSlab {
            id: parse_slab_id(value.get("id").ok_or("id が必要です")?)?,
        })),
        "SetSlabSection" => Ok(Box::new(SetSlabSection {
            id: parse_slab_id(value.get("id").ok_or("id が必要です")?)?,
            section: parse_optional_section_id(value.get("section"))?,
        })),
        "SetSlabUsage" => Ok(Box::new(SetSlabUsage {
            id: parse_slab_id(value.get("id").ok_or("id が必要です")?)?,
            usage: parse_optional_enum::<SlabUsage>(value.get("usage"))?,
        })),
        "SetSlabOneWay" => Ok(Box::new(SetSlabOneWay {
            id: parse_slab_id(value.get("id").ok_or("id が必要です")?)?,
            one_way: parse_optional_enum::<OneWayDir>(value.get("one_way"))?,
        })),
        "SetAttachedExtent" => Ok(Box::new(SetAttachedExtent {
            id: parse_slab_id(value.get("id").ok_or("id が必要です")?)?,
            extent: parse_f64_pair(value.get("extent"), "extent")?,
        })),
        "SetAttachedAnchor" => {
            let anchor: RegionAnchor =
                serde_json::from_value(value.get("anchor").ok_or("anchor が必要です")?.clone())
                    .map_err(|e| format!("anchor の解析に失敗: {e}"))?;
            Ok(Box::new(SetAttachedAnchor {
                id: parse_slab_id(value.get("id").ok_or("id が必要です")?)?,
                anchor,
            }))
        }
        "SetFloorRegionName" => Ok(Box::new(SetFloorRegionName {
            id: parse_floor_region_id(value.get("id").ok_or("id が必要です")?)?,
            name: value
                .get("name")
                .and_then(|v| v.as_str())
                .ok_or("name が必要です")?
                .to_string(),
        })),
        // 手入力小梁ライン（JoistLine）は廃止した（§3.4 F1）。小梁の実体は二次部材
        // （SetFloorRegionSecondaryJoists）に一本化したため、黙って無視せず明示エラーにする。
        "SetFloorRegionJoists" => Err(
            "SetFloorRegionJoists は廃止しました。小梁は二次部材へ一本化したため、\
             SetFloorRegionSecondaryJoists（SecondaryMember の配列）を使ってください"
                .into(),
        ),
        "SetFloorRegionSecondaryJoists" => {
            if value.get("secondary_joist_ids").is_some() || value.get("ids").is_some() {
                return Err(
                    "SetFloorRegionSecondaryJoists は secondary_joists（SecondaryMember の配列）が必須です。\
                     旧コマンド SetSlabSecondaryJoistIds / キー secondary_joist_ids は廃止しました"
                        .into(),
                );
            }
            let joists: Vec<SecondaryMember> = match value.get("secondary_joists") {
                None | Some(serde_json::Value::Null) => {
                    return Err("secondary_joists が必要です（空にする場合は [] を渡す）".into());
                }
                Some(v) => serde_json::from_value(v.clone())
                    .map_err(|e| format!("secondary_joists の解析に失敗: {e}"))?,
            };
            Ok(Box::new(SetFloorRegionSecondaryJoists {
                region: parse_floor_region_id(
                    value
                        .get("floor_region")
                        .or(value.get("id"))
                        .ok_or("floor_region が必要です")?,
                )?,
                joists,
            }))
        }
        "SetSlabSecondaryJoistIds" => Err(
            "SetSlabSecondaryJoistIds は廃止しました。SetFloorRegionSecondaryJoists と \
             secondary_joists を使ってください"
                .into(),
        ),
        "SetWallRegionPosts" => {
            let posts: Vec<SecondaryMember> = match value.get("posts") {
                None | Some(serde_json::Value::Null) => {
                    return Err("posts が必要です（空にする場合は [] を渡す）".into());
                }
                Some(v) => serde_json::from_value(v.clone())
                    .map_err(|e| format!("posts の解析に失敗: {e}"))?,
            };
            Ok(Box::new(SetWallRegionPosts {
                region: parse_wall_region_id(
                    value
                        .get("wall_region")
                        .or(value.get("id"))
                        .ok_or("wall_region が必要です")?,
                )?,
                posts,
            }))
        }
        "AddUnassignedJoist" => {
            let sm: SecondaryMember = serde_json::from_value(
                value
                    .get("joist")
                    .cloned()
                    .ok_or("joist が必要です（SecondaryMember）")?,
            )
            .map_err(|e| format!("joist の解析に失敗: {e}"))?;
            Ok(Box::new(AddUnassignedJoist { sm }))
        }
        "DeleteUnassignedJoist" => {
            let index = value
                .get("index")
                .and_then(|v| v.as_u64())
                .ok_or("index が必要です")? as usize;
            Ok(Box::new(DeleteUnassignedJoist { index }))
        }
        "AddUnassignedPost" => {
            let sm: SecondaryMember = serde_json::from_value(
                value
                    .get("post")
                    .cloned()
                    .ok_or("post が必要です（SecondaryMember）")?,
            )
            .map_err(|e| format!("post の解析に失敗: {e}"))?;
            Ok(Box::new(AddUnassignedPost { sm }))
        }
        "DeleteUnassignedPost" => {
            let index = value
                .get("index")
                .and_then(|v| v.as_u64())
                .ok_or("index が必要です")? as usize;
            Ok(Box::new(DeleteUnassignedPost { index }))
        }
        other => Err(format!(
            "未対応の command: {other}（壁版: AddEnclosedWallPlate, AddAttachedWallPlate, \
             DeleteWallPlate, SetWallPlateSection, SetWallPlateAttrs, \
             SetAttachedWallPlateExtent, SetAttachedWallPlateAnchor / \
             床板: AddSlab, AddAttachedSlab, DeleteSlab, SetSlabSection, SetSlabUsage, \
             SetSlabOneWay, SetAttachedExtent, SetAttachedAnchor / \
             床領域: SetFloorRegionName, SetFloorRegionSecondaryJoists / \
             壁領域: SetWallRegionPosts / \
             未割当: AddUnassignedJoist, DeleteUnassignedJoist, AddUnassignedPost, DeleteUnassignedPost）"
        )),
    }
}

/// `ServerState` へ編集コマンドを適用する。
pub fn apply_edit(
    state: &mut ServerState,
    value: &serde_json::Value,
) -> Result<WriteResult, String> {
    let cmd = parse_edit_command(value)?;
    let label = cmd.label().to_string();
    let revision_before = state.undo.revision();
    let applied = state.undo.run(&mut state.model, cmd);
    let op_id = format!("op-{}", state.undo.revision());
    Ok(WriteResult {
        op_id,
        applied,
        undoable: applied,
        summary: if applied {
            format!(
                "{label} を適用しました（revision {revision_before} → {}）",
                state.undo.revision()
            )
        } else {
            format!("{label} は適用されませんでした（参照検証等で Noop）")
        },
    })
}

/// `command` がトップレベルに無いとき、ネストした `body` をコマンド JSON とみなす。
fn resolve_edit_payload(value: &serde_json::Value) -> &serde_json::Value {
    match value {
        serde_json::Value::Object(m)
            if m.get("command").and_then(|v| v.as_str()).is_none()
                && m.get("body")
                    .and_then(|b| b.get("command"))
                    .and_then(|v| v.as_str())
                    .is_some() =>
        {
            &m["body"]
        }
        _ => value,
    }
}

fn parse_node_ids(value: &serde_json::Value) -> Result<Vec<NodeId>, String> {
    let arr = value.as_array().ok_or("節点 ID は配列で指定してください")?;
    arr.iter().map(parse_node_id).collect()
}

fn parse_node_id(value: &serde_json::Value) -> Result<NodeId, String> {
    value
        .as_u64()
        .map(|n| NodeId(n as u32))
        .ok_or_else(|| format!("節点 ID は非負整数です: {value}"))
}

fn parse_wall_plate_id(value: &serde_json::Value) -> Result<WallPlateId, String> {
    value
        .as_u64()
        .map(|n| WallPlateId(n as u32))
        .ok_or_else(|| format!("壁版 ID は非負整数です: {value}"))
}

fn parse_slab_id(value: &serde_json::Value) -> Result<SlabId, String> {
    value
        .as_u64()
        .map(|n| SlabId(n as u32))
        .ok_or_else(|| format!("床板 ID は非負整数です: {value}"))
}

fn parse_floor_region_id(value: &serde_json::Value) -> Result<FloorRegionId, String> {
    value
        .as_u64()
        .map(|n| FloorRegionId(n as u32))
        .ok_or_else(|| format!("床領域 ID は非負整数です: {value}"))
}

fn parse_wall_region_id(value: &serde_json::Value) -> Result<WallRegionId, String> {
    value
        .as_u64()
        .map(|n| WallRegionId(n as u32))
        .ok_or_else(|| format!("壁領域 ID は非負整数です: {value}"))
}

fn parse_distribution_method(
    value: Option<&serde_json::Value>,
) -> Result<DistributionMethod, String> {
    match value {
        None | Some(serde_json::Value::Null) => Ok(DistributionMethod::default()),
        Some(v) => {
            serde_json::from_value(v.clone()).map_err(|e| format!("method の解析に失敗: {e}"))
        }
    }
}

fn parse_optional_enum<T: serde::de::DeserializeOwned>(
    value: Option<&serde_json::Value>,
) -> Result<Option<T>, String> {
    match value {
        None | Some(serde_json::Value::Null) => Ok(None),
        Some(v) => serde_json::from_value(v.clone())
            .map(Some)
            .map_err(|e| format!("列挙値の解析に失敗: {e}")),
    }
}

fn parse_optional_section_id(
    value: Option<&serde_json::Value>,
) -> Result<Option<SectionId>, String> {
    match value {
        None | Some(serde_json::Value::Null) => Ok(None),
        Some(v) => {
            Ok(Some(v.as_u64().map(|n| SectionId(n as u32)).ok_or_else(
                || format!("断面 ID は非負整数または null です: {v}"),
            )?))
        }
    }
}

fn parse_f64(value: Option<&serde_json::Value>, field: &str) -> Result<Option<f64>, String> {
    match value {
        None | Some(serde_json::Value::Null) => Ok(None),
        Some(v) => v
            .as_f64()
            .ok_or_else(|| format!("{field} は数値です: {v}"))
            .map(Some),
    }
}

/// 耐震スリット `slit` を読む。省略・null は「どの辺も切れていない」。
///
/// `{"column_face": [bool, bool], "beam_face": [bool, bool]}` の形で、柱際の添字は
/// `WallPlate::column_face_nodes` が返す 2 節点に、梁際は 0 が下辺・1 が上辺に対応する。
/// 片方のキーだけを与えることもでき、欠けた側は切れていない扱いになる。
fn parse_slit(value: Option<&serde_json::Value>) -> Result<squid_n_core::model::WallSlit, String> {
    let Some(v) = value else {
        return Ok(Default::default());
    };
    if v.is_null() {
        return Ok(Default::default());
    }
    let obj = v
        .as_object()
        .ok_or("slit は column_face / beam_face を持つオブジェクトです")?;
    Ok(squid_n_core::model::WallSlit {
        column_face: parse_bool_pair(obj.get("column_face"), "slit.column_face")?,
        beam_face: parse_bool_pair(obj.get("beam_face"), "slit.beam_face")?,
    })
}

/// 2 要素の真偽値配列を読む。省略・null は `[false, false]`。
fn parse_bool_pair(value: Option<&serde_json::Value>, field: &str) -> Result<[bool; 2], String> {
    let Some(v) = value else {
        return Ok([false, false]);
    };
    if v.is_null() {
        return Ok([false, false]);
    }
    let arr = v
        .as_array()
        .ok_or_else(|| format!("{field} は2要素の真偽値配列です"))?;
    if arr.len() != 2 {
        return Err(format!("{field} は2要素の真偽値配列です"));
    }
    Ok([
        arr[0]
            .as_bool()
            .ok_or_else(|| format!("{field}[0] は真偽値です"))?,
        arr[1]
            .as_bool()
            .ok_or_else(|| format!("{field}[1] は真偽値です"))?,
    ])
}

fn parse_f64_pair(value: Option<&serde_json::Value>, field: &str) -> Result<[f64; 2], String> {
    let arr = value
        .ok_or_else(|| format!("{field} が必要です"))?
        .as_array()
        .ok_or_else(|| format!("{field} は2要素の配列です"))?;
    if arr.len() != 2 {
        return Err(format!("{field} は2要素の配列です"));
    }
    Ok([
        arr[0]
            .as_f64()
            .ok_or_else(|| format!("{field}[0] は数値です"))?,
        arr[1]
            .as_f64()
            .ok_or_else(|| format!("{field}[1] は数値です"))?,
    ])
}
