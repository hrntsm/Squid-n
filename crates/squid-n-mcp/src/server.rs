//! MCP サーバ実装（rmcp によるツールルータ）。

use super::*;
use rmcp::handler::server::tool::ToolRouter;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, Content, Implementation, ServerInfo};
use rmcp::transport::stdio;
use rmcp::{tool, tool_handler, tool_router, ErrorData, ServerHandler, ServiceExt};
use std::sync::Arc;
use tokio::sync::Mutex;

#[derive(Clone)]
pub struct SquidNServer {
    state: Arc<Mutex<ServerState>>,
    #[allow(dead_code)]
    tool_router: ToolRouter<Self>,
}

impl SquidNServer {
    pub fn new(state: ServerState) -> Self {
        Self {
            state: Arc::new(Mutex::new(state)),
            tool_router: Self::tool_router(),
        }
    }
}

#[tool_router]
impl SquidNServer {
    #[tool(description = "節点・部材・断面・壁版・床板・床領域を検索する")]
    pub async fn model_query(
        &self,
        Parameters(args): Parameters<QueryArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        let st = self.state.lock().await;
        let items = super::query_model(&st.model, &args.kind, args.filter.as_deref());
        let result = QueryResult { items };
        Ok(CallToolResult::success(vec![Content::json(result)?]))
    }

    #[tool(
        description = "モデルを編集する（EditCommand 経由。Undo 可能。command キーで種別を指定）"
    )]
    pub async fn model_edit(
        &self,
        Parameters(args): Parameters<EditArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        let mut st = self.state.lock().await;
        let result = super::apply_edit(&mut st, &args.body)
            .map_err(|e| ErrorData::invalid_params(e, None))?;
        Ok(CallToolResult::success(vec![Content::json(result)?]))
    }

    #[tool(description = "数量積算（コンクリート体積・型枠面積・鉄筋/鉄骨重量の概算）を集計する")]
    pub async fn quantity_takeoff(
        &self,
        Parameters(args): Parameters<QuantityTakeoffArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        let st = self.state.lock().await;
        let result = super::quantity_takeoff_json(&st.model, args.group_by.as_deref());
        Ok(CallToolResult::success(vec![Content::json(result)?]))
    }

    #[tool(description = "解析を非同期で実行する")]
    pub async fn analysis_run(
        &self,
        Parameters(args): Parameters<AnalysisRunArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        let params = args
            .to_job_params()
            .map_err(|e| ErrorData::invalid_params(e, None))?;
        let kind = args.kind;

        let (id, model) = {
            let mut st = self.state.lock().await;
            let id = st.jobs.register(kind);
            st.jobs.update(&id, JobStatus::Running { progress: 0.0 });
            (id, st.model.clone())
        };

        let state = self.state.clone();
        let job_id = id.clone();
        tokio::spawn(async move {
            let outcome =
                tokio::task::spawn_blocking(move || super::compute_job(&model, kind, &params))
                    .await;
            match outcome {
                Ok(Ok(job_outcome)) => {
                    let mut st = state.lock().await;
                    match super::persist_job_outcome(&mut st.results, job_outcome) {
                        Ok(summary) => st.jobs.update(
                            &job_id,
                            JobStatus::Done {
                                result_ref: summary,
                            },
                        ),
                        Err(e) => st.jobs.update(
                            &job_id,
                            JobStatus::Failed {
                                error: format!("解析は完了しましたが結果の保存に失敗しました: {e}"),
                                kind: "internal".to_string(),
                            },
                        ),
                    }
                }
                Ok(Err(e)) => {
                    let mut st = state.lock().await;
                    st.jobs.update(
                        &job_id,
                        JobStatus::Failed {
                            error: e.to_string(),
                            kind: e.kind().to_string(),
                        },
                    );
                }
                Err(join_err) => {
                    let mut st = state.lock().await;
                    st.jobs.update(
                        &job_id,
                        JobStatus::Failed {
                            error: format!("解析タスクが異常終了しました: {join_err}"),
                            kind: "internal".to_string(),
                        },
                    );
                }
            }
        });

        Ok(CallToolResult::success(vec![Content::json(
            serde_json::json!({ "job_id": id }),
        )?]))
    }

    /// 結果ストアから結果を取得する。
    #[tool(description = "解析結果ストアから結果を取得する")]
    pub async fn result_get(
        &self,
        Parameters(args): Parameters<ResultGetArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        let step_range = match &args.step_range {
            None => None,
            Some(v) if v.len() == 2 => Some((v[0], v[1])),
            Some(_) => {
                return Err(ErrorData::invalid_params(
                    "step_range は [start, end) の2要素で指定してください",
                    None,
                ));
            }
        };
        let st = self.state.lock().await;
        let result = super::result_get_json(
            &st.results,
            args.case,
            &args.kind,
            args.node_ids.clone(),
            args.member_ids.clone(),
            step_range,
        )
        .map_err(|e| ErrorData::invalid_params(e, None))?;
        Ok(CallToolResult::success(vec![Content::json(result)?]))
    }

    /// ジョブの現在状態を返す。
    #[tool(description = "ジョブの状態を取得する")]
    pub async fn analysis_status(
        &self,
        Parameters(args): Parameters<AnalysisStatusArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        let st = self.state.lock().await;
        let job = st.jobs.get(&args.job_id);
        match job {
            Some(j) => Ok(CallToolResult::success(vec![Content::json(j)?])),
            None => Err(ErrorData::invalid_params("job not found", None)),
        }
    }
}

#[tool_handler]
impl ServerHandler for SquidNServer {
    fn get_info(&self) -> ServerInfo {
        ServerInfo::default().with_server_info(Implementation::new("squid-n-mcp", "0.1.0"))
    }
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct QueryArgs {
    pub kind: String,
    pub filter: Option<String>,
}

/// `model.edit` の引数。`command` キーで種別を指定する。
#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
#[serde(transparent)]
pub struct EditArgs {
    pub body: serde_json::Value,
}

#[derive(Debug, serde::Serialize, schemars::JsonSchema)]
pub struct QueryResult {
    pub items: Vec<serde_json::Value>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct QuantityTakeoffArgs {
    /// 集計単位（既定 `"category"`）。
    pub group_by: Option<String>,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct AnalysisRunArgs {
    pub kind: JobKind,
    /// 対象荷重ケース ID（未指定なら先頭ケース）。
    pub load_case: Option<u32>,
    /// モード数（既定 3）。
    pub n_modes: Option<usize>,
    /// 加力・入力方向 "X"/"Y"（既定 "X"）。
    pub dir: Option<String>,
    /// 最大ステップ数（既定 50）。
    pub steps: Option<usize>,
    /// 目標変位 [mm]。
    pub max_disp: Option<f64>,
    /// 目標最大層間変形角の分母 n（既定 150）。
    pub max_drift_denom: Option<f64>,
    /// サンプル波の時間刻み [s]（既定 0.01）。
    pub dt: Option<f64>,
    /// サンプル波の継続時間 [s]（既定 2.0）。
    pub duration: Option<f64>,
    /// サンプル波の周期 [s]（既定 0.5）。
    pub period: Option<f64>,
    /// サンプル波の振幅 [mm/s²]（既定 1000）。
    pub amp: Option<f64>,
    /// 地域係数 Z（既定 1.0）。
    pub z: Option<f64>,
    /// 地盤種別 `"I"`/`"II"`/`"III"`（既定 `"II"`）。
    pub soil: Option<String>,
    /// 標準せん断力係数 C0（既定 0.2）。
    pub c0: Option<f64>,
    /// Ai 算定法 `"Approx"`/`"SemiPrecise"`（既定 `"Approx"`）。
    pub ai_mode: Option<String>,
    /// 精算時の設計用基本周期 T [s]。
    pub design_period: Option<f64>,
}

impl AnalysisRunArgs {
    /// 任意パラメータを `super::JobParams`（既定値込み）へ変換する。
    /// 不正な文字列の場合のみエラーを返す。
    fn to_job_params(&self) -> Result<super::JobParams, String> {
        let dir = match self.dir.as_deref() {
            None => super::JobDir::X,
            Some("X") => super::JobDir::X,
            Some("Y") => super::JobDir::Y,
            Some(other) => {
                return Err(format!("不明な方向: {other}（\"X\" または \"Y\"）"));
            }
        };
        let soil = match self.soil.as_deref() {
            None => None,
            Some("I") => Some(squid_n_load::ai::SoilClass::I),
            Some("II") => Some(squid_n_load::ai::SoilClass::II),
            Some("III") => Some(squid_n_load::ai::SoilClass::III),
            Some(other) => {
                return Err(format!(
                    "不明な地盤種別: {other}（\"I\"、\"II\"、または \"III\"）"
                ));
            }
        };
        let ai_mode = match self.ai_mode.as_deref() {
            None => None,
            Some("Approx") => Some(squid_n_solver::statics::analysis::AiMode::Approx),
            Some("SemiPrecise") => Some(squid_n_solver::statics::analysis::AiMode::SemiPrecise),
            Some(other) => {
                return Err(format!(
                    "不明な Ai 算定法: {other}（\"Approx\" または \"SemiPrecise\"）"
                ));
            }
        };
        let d = super::JobParams::default();
        Ok(super::JobParams {
            load_case: self.load_case,
            n_modes: self.n_modes.unwrap_or(d.n_modes),
            dir,
            steps: self.steps.unwrap_or(d.steps),
            max_disp: self.max_disp,
            max_drift_denom: self.max_drift_denom,
            dt: self.dt.unwrap_or(d.dt),
            duration: self.duration.unwrap_or(d.duration),
            period: self.period.unwrap_or(d.period),
            amp: self.amp.unwrap_or(d.amp),
            z: self.z.unwrap_or(d.z),
            soil: soil.unwrap_or(d.soil),
            c0: self.c0.unwrap_or(d.c0),
            ai_mode: ai_mode.unwrap_or(d.ai_mode),
            design_period: self.design_period,
        })
    }
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct AnalysisStatusArgs {
    pub job_id: String,
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct ResultGetArgs {
    pub case: u32,
    /// "NodalDisp" | "MemberForce" | "Modal" | "TimeHistory"
    pub kind: String,
    pub node_ids: Option<Vec<u32>>,
    pub member_ids: Option<Vec<u32>>,
    pub step_range: Option<Vec<u64>>,
}

pub async fn run_stdio_server(state: ServerState) -> Result<(), Box<dyn std::error::Error>> {
    let service = SquidNServer::new(state).serve(stdio()).await?;
    service.waiting().await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
        MaterialCategory, NodalLoad, Node, Section, Story,
    };
    use std::path::{Path, PathBuf};
    use std::time::Duration;

    /// テスト用の結果ストアディレクトリを用意する（テストごとに固有の名前を渡すこと）。
    /// 前回実行の残骸を消してから使う（実ストア=ファイルシステムを使うため）。
    /// プロセス ID 入りのサブディレクトリを介し、同一マシンで並行する別プロセスの
    /// テスト実行と衝突しないようにする。
    fn test_store_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "squid-n-test-{}/squid_n_mcp_test_{name}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    /// 実ストア（`FsResultStore`）を使う `ServerState` を組み立てる。
    fn make_state(model: Model, dir: &Path) -> ServerState {
        ServerState::with_fs_store(model, dir).expect("FsResultStore::open が失敗しないこと")
    }

    /// 片持ち梁（node0 固定・node1 自由）+ 荷重ケース1つの、解析が完走できる最小モデル。
    /// LinearStatic/Eigen/TimeHistory/DesignCheck の各ジョブテストで共有する:
    /// - 先端に質量を与えている(Eigen/TimeHistory が固有値解析できるように)。
    /// - 材料名を鋼材(SN400)にし、断面係数(iz)を小さくしている
    ///   (DesignCheck で NG が出ることを確認できるように、わざと過大応力にしている)。
    /// - 荷重は全体座標系 Z 方向のせん断力とし、曲げモーメントが生じるようにしている。
    ///   `ref_vector=[0,0,1]`(=全体Z)の梁は局所 y 軸が全体 Z に一致するため
    ///   （`LocalFrame::from_nodes` 参照）、全体 Z 方向の力が局所 Qy/Mz
    ///   （`compute_design_check_job` が強軸として見る成分）に載る。
    ///   全体 X 方向の軸力だけでは M・Q が実質ゼロで検定に影響しないため使わない。
    fn cantilever_with_load_case() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [1000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: Some([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]),
                    story: None,
                    support_spring: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "beam".into(),
                area: 100.0,
                iy: 833.33,
                iz: 10.0,
                j: 100.0,
                depth: 10.0,
                width: 10.0,
                as_y: 83.33,
                as_z: 83.33,
                floor: None,
                panel_thickness: None,
                thickness: None,
                shape: None,
                material: Some(MaterialId(0)),
                rebar_material: None,
                shear_rebar_material: None,
                steel_material: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                strength_factor: None,
                id: MaterialId(0),
                name: "SN400".into(),
                category: MaterialCategory::Steel,
                young: 20000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: Some(235.0),
            }],
            load_cases: vec![LoadCase {
                id: LoadCaseId(1),
                name: "case1".into(),
                nodal: vec![NodalLoad::manual(
                    NodeId(1),
                    [0.0, 0.0, 1000.0, 0.0, 0.0, 0.0],
                )],
                member: Vec::new(),
                kind: Default::default(),
            }],
            ..Default::default()
        }
    }

    /// 上と同じモデルから荷重ケースだけを抜いたもの（LinearStatic ジョブが
    /// `JobError::LoadCaseNotFound` で失敗する経路を確認するため）。
    fn cantilever_without_load_case() -> Model {
        Model {
            load_cases: vec![],
            ..cantilever_with_load_case()
        }
    }

    /// 1層・鉛直柱モデル（Pushover ジョブ用）。
    /// squid-n-app の `sample::portal_frame`（Beam 要素・SN400B）と同じ材料構成をベースに
    /// 単純な片持ち柱 + Story(地震重量) を組み立てる
    /// （`Analysis::prepare` の線形剛性検証を通す必要があるため、
    /// ねじり剛性を持たない Fiber 要素ではなく Beam 要素を使う。
    /// squid-n-solver 側の `pushover::tests::single_column_model` は Fiber 要素かつ
    /// `Analysis::prepare` を経由しないテストのため、そのままでは
    /// `App::compute_pushover`/本ジョブの「まず prepare で検証する」流儀に合わない）。
    fn pushover_model() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: Some(StoryId(0)),
                    support_spring: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "col".into(),
                area: 10000.0,
                iy: 8.333e6,
                iz: 8.333e6,
                j: 1.0e6,
                depth: 100.0,
                width: 100.0,
                // 有効せん断断面積 As = A·5/6（矩形。0 は解析前チェックで入力エラー）。
                as_y: 10000.0 * 5.0 / 6.0,
                as_z: 10000.0 * 5.0 / 6.0,
                floor: None,
                panel_thickness: None,
                thickness: None,
                shape: None,
                material: Some(MaterialId(0)),
                rebar_material: None,
                shear_rebar_material: None,
                steel_material: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                strength_factor: None,
                id: MaterialId(0),
                name: "steel".into(),
                category: MaterialCategory::Steel,
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                // squid-n-solver 側の `single_column_model`（Fiber 要素、ねじり無視）は
                // shear=Some(0.0) だが、ここは Beam 要素（弾性ねじり剛性 GJ を持つ）なので
                // shear=None にして young/poisson から G を導出させる
                // （G=0 のままだと頂部のねじり自由度の剛性が 0 になり特異行列になる）。
                shear: None,
                fc: None,
                fy: Some(235.0),
            }],
            stories: vec![
                // 階は床であり、先頭は基部の床（`Model::layers` の不変条件）。
                Story {
                    id: StoryId(0),
                    name: "1F".into(),
                    elevation: 0.0,
                    node_ids: vec![NodeId(0)],
                    seismic_weight: None,
                    weight_override: None,
                    structure: Default::default(),
                    level_kind: Default::default(),
                },
                Story {
                    id: StoryId(1),
                    name: "2F".into(),
                    elevation: 3000.0,
                    node_ids: vec![NodeId(1)],
                    seismic_weight: Some(80_000.0),
                    weight_override: None,
                    structure: Default::default(),
                    level_kind: Default::default(),
                },
            ],
            ..Default::default()
        }
    }

    /// `AnalysisRunArgs` を最小限のフィールド指定で組み立てる
    /// （他は全て既定値=None）。
    fn run_args(kind: JobKind) -> AnalysisRunArgs {
        AnalysisRunArgs {
            kind,
            load_case: None,
            n_modes: None,
            dir: None,
            steps: None,
            max_disp: None,
            max_drift_denom: None,
            dt: None,
            duration: None,
            period: None,
            amp: None,
            z: None,
            soil: None,
            c0: None,
            ai_mode: None,
            design_period: None,
        }
    }

    /// `CallToolResult`（`analysis_run` の戻り値）から `job_id` を取り出す。
    fn extract_job_id(result: &CallToolResult) -> String {
        let text = result.content[0]
            .raw
            .as_text()
            .expect("analysis_run の応答は text content のはず")
            .text
            .clone();
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        value["job_id"].as_str().unwrap().to_string()
    }

    /// ジョブが `Done`/`Failed` のいずれかの終端状態に達するまでポーリングする。
    async fn wait_for_terminal(server: &SquidNServer, job_id: &str) -> JobStatus {
        for _ in 0..400 {
            {
                let st = server.state.lock().await;
                if let Some(job) = st.jobs.get(job_id) {
                    match &job.status {
                        JobStatus::Done { .. } | JobStatus::Failed { .. } => {
                            return job.status.clone();
                        }
                        _ => {}
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!("job did not reach a terminal state in time");
    }

    /// `Done` 状態の `result_ref`（サマリ JSON 文字列）を取り出してパースする。
    fn done_summary(status: &JobStatus) -> serde_json::Value {
        match status {
            JobStatus::Done { result_ref } => {
                serde_json::from_str(result_ref).expect("summary は JSON のはず")
            }
            other => panic!("expected Done, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn test_analysis_run_completes_for_valid_model() {
        let dir = test_store_dir("linear_static_basic");
        let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
        let result = server
            .analysis_run(Parameters(run_args(JobKind::LinearStatic)))
            .await
            .unwrap();
        let job_id = extract_job_id(&result);
        let status = wait_for_terminal(&server, &job_id).await;
        assert!(
            matches!(status, JobStatus::Done { .. }),
            "expected Done, got {status:?}"
        );
    }

    #[tokio::test]
    async fn test_analysis_run_fails_without_load_case() {
        let dir = test_store_dir("linear_static_no_case");
        let server = SquidNServer::new(make_state(cantilever_without_load_case(), &dir));
        let result = server
            .analysis_run(Parameters(run_args(JobKind::LinearStatic)))
            .await
            .unwrap();
        let job_id = extract_job_id(&result);
        let status = wait_for_terminal(&server, &job_id).await;
        match status {
            JobStatus::Failed { error, kind } => {
                // 文言は利用者向けの日本語、種別コードは機械可読な安定値。
                assert!(error.contains("荷重ケース"), "unexpected error: {error}");
                assert_eq!(kind, "load_case_not_found");
            }
            other => panic!("expected Failed, got {other:?}"),
        }
    }

    /// LinearStatic ジョブ → Done → manifest に NodalDisp/MemberForce が載る →
    /// result_get(NodalDisp, node_ids 指定) が該当行を返す。
    #[tokio::test]
    async fn test_linear_static_job_persists_and_result_get_filters_nodes() {
        let dir = test_store_dir("linear_static_result_get");
        let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
        let result = server
            .analysis_run(Parameters(run_args(JobKind::LinearStatic)))
            .await
            .unwrap();
        let job_id = extract_job_id(&result);
        let status = wait_for_terminal(&server, &job_id).await;
        let summary = done_summary(&status);
        assert_eq!(summary["store"]["case"], 1);
        let kinds: Vec<String> = summary["store"]["kinds"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap().to_string())
            .collect();
        assert!(kinds.contains(&"NodalDisp".to_string()));
        assert!(kinds.contains(&"MemberForce".to_string()));

        {
            let st = server.state.lock().await;
            let manifest = st.results.manifest();
            assert!(manifest
                .entries
                .iter()
                .any(|e| e.case == 1 && e.kind == squid_n_io::results::ResultKind::NodalDisp));
            assert!(manifest
                .entries
                .iter()
                .any(|e| e.case == 1 && e.kind == squid_n_io::results::ResultKind::MemberForce));
        }

        let got = server
            .result_get(Parameters(ResultGetArgs {
                case: 1,
                kind: "NodalDisp".to_string(),
                node_ids: Some(vec![1]),
                member_ids: None,
                step_range: None,
            }))
            .await
            .unwrap();
        let text = got.content[0].raw.as_text().unwrap().text.clone();
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        let rows = value["rows"].as_array().unwrap();
        assert_eq!(rows.len(), 1, "node_ids=[1] で絞り込んだ行数");
        assert_eq!(rows[0]["node_id"], 1);
        assert_eq!(value["truncated"], false);
    }

    /// Eigen ジョブ → Done（周期がサマリに含まれる）→
    /// result_get(Modal) が n_modes 行返す。
    #[tokio::test]
    async fn test_eigen_job_persists_and_result_get_modal() {
        let dir = test_store_dir("eigen_result_get");
        let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
        let mut args = run_args(JobKind::Eigen);
        args.n_modes = Some(1);
        let result = server.analysis_run(Parameters(args)).await.unwrap();
        let job_id = extract_job_id(&result);
        let status = wait_for_terminal(&server, &job_id).await;
        let summary = done_summary(&status);
        assert_eq!(summary["n_modes"], 1);
        assert!(summary["period"].as_array().unwrap()[0].as_f64().unwrap() > 0.0);
        assert_eq!(summary["store"]["case"], 0);

        let got = server
            .result_get(Parameters(ResultGetArgs {
                case: 0,
                kind: "Modal".to_string(),
                node_ids: None,
                member_ids: None,
                step_range: None,
            }))
            .await
            .unwrap();
        let text = got.content[0].raw.as_text().unwrap().text.clone();
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        let rows = value["rows"].as_array().unwrap();
        assert_eq!(rows.len(), 1, "n_modes=1 なので1行返るはず");
    }

    /// Pushover ジョブ（stories 付きモデル）→ Done でサマリに qu[kN] が含まれる。
    #[tokio::test]
    async fn test_pushover_job_completes_with_qu_in_summary() {
        let dir = test_store_dir("pushover_basic");
        let server = SquidNServer::new(make_state(pushover_model(), &dir));
        let mut args = run_args(JobKind::Pushover);
        // 既定(steps=50, 目標層間変形角1/150)だと機構形成後に特異行列となり得るため
        // (squid-n-solver 側の同種テストと同じ配慮)、目標変位を明示して小さめの値にする。
        args.steps = Some(10);
        args.max_disp = Some(30.0);
        let result = server.analysis_run(Parameters(args)).await.unwrap();
        let job_id = extract_job_id(&result);
        let status = wait_for_terminal(&server, &job_id).await;
        let summary = done_summary(&status);
        assert!(summary["qu_kN"].as_f64().unwrap() > 0.0);
        assert!(
            summary.get("store").is_none(),
            "Pushover はストアへ書かない"
        );
    }

    /// DesignCheck ジョブ → Done でサマリに NG 数が含まれる。
    #[tokio::test]
    async fn test_design_check_job_reports_ng_count() {
        let dir = test_store_dir("design_check_basic");
        let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
        let result = server
            .analysis_run(Parameters(run_args(JobKind::DesignCheck)))
            .await
            .unwrap();
        let job_id = extract_job_id(&result);
        let status = wait_for_terminal(&server, &job_id).await;
        let summary = done_summary(&status);
        assert_eq!(summary["case"], 1);
        assert!(summary["n_checks"].as_u64().unwrap() > 0);
        assert!(
            summary["n_ng"].as_u64().unwrap() > 0,
            "断面係数を小さくしてあるので過大応力で NG になるはず: {summary}"
        );
    }

    /// `result_get`: 存在しない case を指定すると invalid_params エラーになる。
    #[tokio::test]
    async fn test_result_get_missing_case_is_invalid_params() {
        let dir = test_store_dir("result_get_missing");
        let server = SquidNServer::new(make_state(cantilever_with_load_case(), &dir));
        let err = server
            .result_get(Parameters(ResultGetArgs {
                case: 999,
                kind: "NodalDisp".to_string(),
                node_ids: None,
                member_ids: None,
                step_range: None,
            }))
            .await
            .expect_err("manifest にない case は Err のはず");
        assert!(
            err.message.contains("結果がありません"),
            "エラーメッセージに『結果がありません』が含まれるはず: {err:?}"
        );
    }

    #[test]
    fn test_to_job_params_parses_seismic_settings() {
        let mut args = run_args(JobKind::LinearStatic);
        args.z = Some(1.2);
        args.soil = Some("III".to_string());
        args.c0 = Some(0.25);
        args.ai_mode = Some("SemiPrecise".to_string());
        args.design_period = None;
        let params = args
            .to_job_params()
            .expect("SemiPrecise without design_period");
        assert!((params.z - 1.2).abs() < 1e-12);
        assert_eq!(params.soil, squid_n_load::ai::SoilClass::III);
        assert!((params.c0 - 0.25).abs() < 1e-12);
        assert_eq!(
            params.ai_mode,
            squid_n_solver::statics::analysis::AiMode::SemiPrecise
        );
        assert_eq!(params.design_period, None);
    }

    #[test]
    fn test_prepare_notices_for_semi_precise_without_design_period() {
        // 階が無いと地震同期パスに入らず notices が空になる。
        let params = JobParams {
            ai_mode: squid_n_solver::statics::analysis::AiMode::SemiPrecise,
            design_period: None,
            ..Default::default()
        };
        let (_model, notices) = crate::job::model_prepared_for_analysis(&pushover_model(), &params);
        assert!(
            notices.iter().any(|s| s.contains("EX/EY")),
            "精算周期未指定時は EX/EY 未同期の注意が出ること: {notices:?}"
        );
    }

    #[tokio::test]
    async fn test_analysis_run_semi_precise_without_design_period_completes() {
        // ジョブは完了し、サマリ JSON に notices が載ることを確認する。
        let dir = test_store_dir("semi_precise_no_t");
        let server = SquidNServer::new(make_state(pushover_model(), &dir));
        let mut args = run_args(JobKind::Pushover);
        args.steps = Some(10);
        args.max_disp = Some(30.0);
        args.ai_mode = Some("SemiPrecise".to_string());
        args.design_period = None;
        let result = server.analysis_run(Parameters(args)).await.unwrap();
        let job_id = extract_job_id(&result);
        let status = wait_for_terminal(&server, &job_id).await;
        let JobStatus::Done { result_ref } = status else {
            panic!("SemiPrecise without design_period should still complete: {status:?}");
        };
        let summary: serde_json::Value = serde_json::from_str(&result_ref).expect("summary JSON");
        let notices = summary["notices"]
            .as_array()
            .expect("notices がサマリに載ること");
        assert!(
            notices
                .iter()
                .any(|n| n.as_str().is_some_and(|s| s.contains("EX/EY"))),
            "notices に EX/EY 未更新の旨が含まれること: {notices:?}"
        );
    }
}
