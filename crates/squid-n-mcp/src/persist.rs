//! ジョブ結果の永続化と結果取得の関数。

use super::*;

/// `summary`（JSON オブジェクト）に、結果ストアへ書き込んだ場所を示す
/// `"store": {"case": .., "kinds": [..]}` を追記する。
fn attach_store_info(summary: &mut serde_json::Value, case: u32, kinds: &[&str]) {
    if let serde_json::Value::Object(map) = summary {
        map.insert(
            "store".to_string(),
            serde_json::json!({ "case": case, "kinds": kinds }),
        );
    }
}

/// `JobOutcome` を結果ストアへ永続化し、`JobStatus::Done::result_ref` に格納する
/// サマリ JSON 文字列を返す。書き込み・manifest 永続化の失敗は `Err` で返す
/// （呼び出し側はジョブを `Failed` へ遷移させる。かつては失敗を握りつぶして
/// `Done` を報告し、利用者が「結果 0 行」を正常な解析結果と誤解し得た）。
/// `ServerState` のロックを保持したまま
/// （= ジョブ状態更新と同じロック内で）呼び出すこと。
///
/// ## 対応表（JobKind → 書き込む ResultKind）
/// - LinearStatic: NodalDisp（全節点変位）+ MemberForce（評価位置ごとの断面力）。
///   case = 使用した荷重ケース ID。
/// - DesignCheck: MemberForce のみ（検定の元データ）。検定結果自体（OK/NG・検定比）は
///   専用の ResultKind が無いためサマリ JSON にのみ含める。case = 使用した荷重ケース ID。
/// - Eigen: Modal のみ。case は固定で 0 を使う。固有値解析は荷重ケースに依存しない
///   1系統の結果のため、実在する荷重ケース番号と衝突しないダミー値を使う設計とした。
///   manifest のキーは (case, kind) の組であり、Modal は NodalDisp/MemberForce とは
///   別の ResultKind（＝別の名前空間）なので、仮に実際の荷重ケースが `case=0` を
///   使っていても NodalDisp/MemberForce の case=0 エントリとは衝突しない
///   （LoadCaseId(0) を実荷重ケースとしても二重利用してしまう設計は避けている）。
/// - Pushover/TimeHistory: 対応する ResultKind スキーマが無いため
///   （TimeHistory 結果 `ResponseResult` は代表1節点の応答のみを保持し、
///   `ResultKind::TimeHistory` が要求する全節点×全ステップの変位を持たない）
///   ストアへは書き込まず、サマリ JSON のみを返す。
pub fn persist_job_outcome(
    store: &mut squid_n_io::results::FsResultStore,
    outcome: JobOutcome,
) -> Result<String, String> {
    use squid_n_io::results::{member_force_batch, modal_batch, nodal_disp_batch, ResultKind};

    /// バッチを 1 つ書き込んで finish する（バッチ生成・IO いずれの失敗も伝播）。
    fn write_one(
        store: &mut squid_n_io::results::FsResultStore,
        case: u32,
        kind: ResultKind,
        batch: Result<arrow::record_batch::RecordBatch, arrow::error::ArrowError>,
    ) -> Result<(), String> {
        let batch = batch.map_err(|e| format!("結果バッチの生成に失敗: {e}"))?;
        let mut w = store
            .writer(case, kind)
            .map_err(|e| format!("結果ストアの書き込み開始に失敗: {e}"))?;
        w.write_rows(&batch)
            .map_err(|e| format!("結果ストアへの書き込みに失敗: {e}"))?;
        w.finish()
            .map_err(|e| format!("結果ストアの書き込み完了に失敗: {e}"))?;
        Ok(())
    }

    match outcome {
        JobOutcome::LinearStatic {
            case,
            node_ids,
            disp,
            member_force_rows,
            mut summary,
        } => {
            let mut kinds: Vec<&str> = Vec::new();
            write_one(
                store,
                case,
                ResultKind::NodalDisp,
                nodal_disp_batch(&node_ids, &disp),
            )?;
            kinds.push("NodalDisp");
            if !member_force_rows.is_empty() {
                write_one(
                    store,
                    case,
                    ResultKind::MemberForce,
                    member_force_batch(&member_force_rows),
                )?;
                kinds.push("MemberForce");
            }
            store
                .sync()
                .map_err(|e| format!("結果マニフェストの永続化に失敗: {e}"))?;
            attach_store_info(&mut summary, case, &kinds);
            Ok(summary.to_string())
        }
        JobOutcome::DesignCheck {
            case,
            member_force_rows,
            mut summary,
        } => {
            let mut kinds: Vec<&str> = Vec::new();
            if !member_force_rows.is_empty() {
                write_one(
                    store,
                    case,
                    ResultKind::MemberForce,
                    member_force_batch(&member_force_rows),
                )?;
                kinds.push("MemberForce");
            }
            store
                .sync()
                .map_err(|e| format!("結果マニフェストの永続化に失敗: {e}"))?;
            if !kinds.is_empty() {
                attach_store_info(&mut summary, case, &kinds);
            }
            Ok(summary.to_string())
        }
        JobOutcome::Eigen {
            period,
            omega2,
            participation,
            effective_mass,
            mut summary,
        } => {
            // LoadCaseId(0) の二重使用を避けるための設計は上記ドキュメントコメント参照。
            let case = 0u32;
            write_one(
                store,
                case,
                ResultKind::Modal,
                modal_batch(&period, &omega2, &participation, &effective_mass),
            )?;
            store
                .sync()
                .map_err(|e| format!("結果マニフェストの永続化に失敗: {e}"))?;
            attach_store_info(&mut summary, case, &["Modal"]);
            Ok(summary.to_string())
        }
        JobOutcome::Pushover { summary }
        | JobOutcome::TimeHistory { summary }
        | JobOutcome::UltimateCheck { summary } => Ok(summary.to_string()),
    }
}

/// 結果 1 回あたりの `result_get` 応答に含める行数の上限。
/// MCP 応答（JSON-RPC のテキストコンテンツ）が肥大化して呼び出し側（LLM）の
/// コンテキストを圧迫するのを防ぐための安全弁。超過分は "truncated": true で通知する。
const RESULT_GET_ROW_LIMIT: usize = 10_000;

/// 結果種別名（"NodalDisp" 等）を `ResultKind` へ変換する。
fn parse_result_kind(s: &str) -> Result<squid_n_io::results::ResultKind, String> {
    use squid_n_io::results::ResultKind;
    match s {
        "NodalDisp" => Ok(ResultKind::NodalDisp),
        "MemberForce" => Ok(ResultKind::MemberForce),
        "Modal" => Ok(ResultKind::Modal),
        "TimeHistory" => Ok(ResultKind::TimeHistory),
        other => Err(format!(
            "不明な結果種別: {other}（NodalDisp/MemberForce/Modal/TimeHistory のいずれか）"
        )),
    }
}

/// `RecordBatch` を JSON 行配列へ変換する（arrow::json は使わず、既知の列型
/// （UInt32/UInt64/Float64。P8 の4スキーマはすべてこのいずれか）だけを手動で
/// `serde_json::Value` に変換する）。`row_limit` を超える行は切り詰め、
/// 2つ目の戻り値で打ち切ったかどうかを返す。
fn batch_to_json_rows(
    batch: &arrow::record_batch::RecordBatch,
    row_limit: usize,
) -> (Vec<serde_json::Value>, bool) {
    use arrow::array::{Float64Array, UInt32Array, UInt64Array};
    use arrow::datatypes::DataType;

    let schema = batch.schema();
    let total = batch.num_rows();
    let n = total.min(row_limit);
    let mut rows = Vec::with_capacity(n);
    for r in 0..n {
        let mut obj = serde_json::Map::new();
        for (c, field) in schema.fields().iter().enumerate() {
            let col = batch.column(c);
            let value = match field.data_type() {
                DataType::UInt32 => serde_json::json!(col
                    .as_any()
                    .downcast_ref::<UInt32Array>()
                    .expect("UInt32 列のはず")
                    .value(r)),
                DataType::UInt64 => serde_json::json!(col
                    .as_any()
                    .downcast_ref::<UInt64Array>()
                    .expect("UInt64 列のはず")
                    .value(r)),
                DataType::Float64 => serde_json::json!(col
                    .as_any()
                    .downcast_ref::<Float64Array>()
                    .expect("Float64 列のはず")
                    .value(r)),
                // P8 の4スキーマ（NodalDisp/MemberForce/Modal/TimeHistory）に
                // 現れない型。将来スキーマが増えたら対応を追加する。
                other => {
                    let _ = other;
                    serde_json::Value::Null
                }
            };
            obj.insert(field.name().clone(), value);
        }
        rows.push(serde_json::Value::Object(obj));
    }
    (rows, total > row_limit)
}

/// `result_get` ツールの中核ロジック（feature 非依存・テスト可能）。
/// manifest に該当エントリが無ければエラー文字列を返す
/// （呼び出し側は MCP の `invalid_params` へマップする）。
pub fn result_get_json(
    store: &dyn squid_n_io::results::ResultStore,
    case: squid_n_io::results::CaseId,
    kind_str: &str,
    node_ids: Option<Vec<u32>>,
    member_ids: Option<Vec<u32>>,
    step_range: Option<(u64, u64)>,
) -> Result<serde_json::Value, String> {
    let kind = parse_result_kind(kind_str)?;
    let exists = store
        .manifest()
        .entries
        .iter()
        .any(|e| e.case == case && e.kind == kind);
    if !exists {
        return Err(format!(
            "結果がありません（case={case}, kind={kind_str}）。analysis_run で解析を実行してから呼び出してください。"
        ));
    }

    let node_filter = node_ids.map(|ids| ids.into_iter().map(squid_n_core::ids::NodeId).collect());
    let member_filter =
        member_ids.map(|ids| ids.into_iter().map(squid_n_core::ids::ElemId).collect());
    let query = squid_n_io::results::ResultQuery {
        case,
        kind,
        node_filter,
        member_filter,
        step_range,
    };
    let result = store
        .query(&query)
        .map_err(|e| format!("結果の読み出しに失敗しました: {e}"))?;
    let (rows, truncated) = batch_to_json_rows(&result.batch, RESULT_GET_ROW_LIMIT);
    Ok(serde_json::json!({
        "case": case,
        "kind": kind_str,
        "rows": rows,
        "truncated": truncated,
    }))
}
