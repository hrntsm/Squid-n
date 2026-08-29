//! モデル編集（P8 T2: `model.edit`）。GUI と同一の `EditCommand` + `UndoStack` 経路。

use super::*;
use squid_n_core::ids::{NodeId, SectionId, WallPlateId};
use squid_n_core::model::{RegionAnchor, WallOpening};
use squid_n_edit::EditCommand;
use squid_n_edit::{
    AddAttachedWallPlate, AddEnclosedWallPlate, DeleteWallPlate, SetAttachedWallPlateAnchor,
    SetAttachedWallPlateExtent, SetWallPlateAttrs, SetWallPlateSection,
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
pub fn parse_edit_command(value: &serde_json::Value) -> Result<Box<dyn EditCommand>, String> {
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
                three_side_slit: value
                    .get("three_side_slit")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(false),
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
        other => Err(format!(
            "未対応の command: {other}（壁版: AddEnclosedWallPlate, AddAttachedWallPlate, \
             DeleteWallPlate, SetWallPlateSection, SetWallPlateAttrs, \
             SetAttachedWallPlateExtent, SetAttachedWallPlateAnchor）"
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
