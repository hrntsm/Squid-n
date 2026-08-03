use arrow::record_batch::RecordBatch;
use squid_n_core::ids::{ElemId, NodeId};

pub type CaseId = u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ResultKind {
    NodalDisp,
    MemberForce,
    Story,
    Modal,
    TimeHistory,
}

pub struct ResultQuery {
    pub case: CaseId,
    pub kind: ResultKind,
    pub node_filter: Option<Vec<NodeId>>,
    pub member_filter: Option<Vec<ElemId>>,
    pub step_range: Option<(u64, u64)>,
}

pub struct ResultBatch {
    pub batch: RecordBatch,
}

/// 結果の書き込み口。IO 失敗（ディスクフル・権限エラー等）は `Err` で返す。
/// かつては失敗を panic で扱っており、MCP サーバの非同期タスク内で panic すると
/// タスクだけが死んでジョブが `Running` のまま永久に終端しなかった。
pub trait ResultWriter {
    fn write_rows(&mut self, batch: &RecordBatch) -> std::io::Result<()>;
    fn finish(self: Box<Self>) -> std::io::Result<()>;
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultManifest {
    pub entries: Vec<ResultEntry>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultEntry {
    pub case: CaseId,
    pub kind: ResultKind,
    pub rows: u64,
    pub path: String,
}

/// 結果ストア。`Send` を要求するのは、MCP サーバ(P8)が `ServerState` を
/// スレッド間で共有する(`rmcp::ServerHandler: Send + Sync`)ため。
/// IO 失敗・未対応の kind・該当エントリなしはいずれも `Err` で返す（panic しない）。
pub trait ResultStore: Send {
    fn writer(&mut self, case: CaseId, kind: ResultKind) -> std::io::Result<Box<dyn ResultWriter>>;
    fn query(&self, q: &ResultQuery) -> std::io::Result<ResultBatch>;
    fn manifest(&self) -> &ResultManifest;
}
