use squid_n_core::model::Model;
use squid_n_edit::UndoStack;
use squid_n_io::results::ResultStore;
use std::collections::HashMap;
use std::path::PathBuf;

pub type JobId = String;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, schemars::JsonSchema)]
pub enum JobStatus {
    Queued,
    Running {
        progress: f32,
    },
    Done {
        result_ref: String,
    },
    /// 失敗。`error` は利用者向けの日本語メッセージ、`kind` は機械可読な種別コード。
    Failed {
        error: String,
        kind: String,
    },
}

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, schemars::JsonSchema,
)]
pub enum JobKind {
    LinearStatic,
    Eigen,
    Pushover,
    TimeHistory,
    DesignCheck,
    /// 終局検定（靭性保証型耐震設計指針）。
    UltimateCheck,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct Job {
    pub id: JobId,
    pub kind: JobKind,
    pub status: JobStatus,
}

#[derive(Debug, Default)]
pub struct JobRegistry {
    jobs: HashMap<JobId, Job>,
    next_id: u64,
}

impl JobRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register(&mut self, kind: JobKind) -> JobId {
        let id = format!("job-{}", self.next_id);
        self.next_id += 1;
        let job = Job {
            id: id.clone(),
            kind,
            status: JobStatus::Queued,
        };
        self.jobs.insert(id.clone(), job);
        id
    }

    pub fn get(&self, id: &str) -> Option<&Job> {
        self.jobs.get(id)
    }

    pub fn update(&mut self, id: &str, status: JobStatus) {
        if let Some(job) = self.jobs.get_mut(id) {
            job.status = status;
        }
    }
}

/// 結果ストア本体（具象型 `FsResultStore` を直接保持し、書き込み後に `sync()` を呼ぶ）。
pub struct ServerState {
    pub model: Model,
    pub undo: UndoStack,
    pub jobs: JobRegistry,
    pub results: squid_n_io::results::FsResultStore,
}

impl ServerState {
    /// 結果ストアのディレクトリを明示して `ServerState` を構築する。
    pub fn with_fs_store(model: Model, dir: impl Into<PathBuf>) -> std::io::Result<Self> {
        Ok(Self {
            model,
            undo: UndoStack::new(),
            jobs: JobRegistry::new(),
            results: squid_n_io::results::FsResultStore::open(dir)?,
        })
    }
}

/// 結果ストアの既定ディレクトリ（`SQUID_N_RESULT_DIR` があれば優先）。
pub fn default_result_dir() -> PathBuf {
    std::env::var_os("SQUID_N_RESULT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("squid-n-mcp-results"))
}

#[cfg(feature = "mcp")]
pub mod server;

mod edit;
mod job;
mod persist;
mod query;

pub use edit::*;
pub use job::*;
pub use persist::*;
pub use query::*;

#[cfg(test)]
mod tests;
