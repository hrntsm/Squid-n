//! 解析ジョブのエラー型。
//!
//! GUI と MCP サーバの双方がこの型を経由するため、`Display` は**日本語**とする。
//! 種別を型で分けているのは、MCP が原因を機械的に判別できるようにするため。

use thiserror::Error;

/// 解析ジョブのエラー。
#[derive(Debug, Error)]
pub enum JobError {
    /// 解析準備（自由度写像・拘束の縮約・モデル整合性）で失敗した。
    #[error("解析準備エラー: {0}")]
    Prepare(String),

    /// 連立方程式の求解で失敗した（特異・非正定値など）。
    #[error("解析エラー: {0}")]
    Solve(String),

    /// 非線形解析が収束しなかった。
    #[error("収束しませんでした: {0}")]
    Convergence(String),

    /// 入力（モデル・解析条件）が解析の前提を満たさない。
    #[error("入力不備: {0}")]
    InvalidInput(String),

    /// 指定された荷重ケース・組合せが見つからない。
    #[error("荷重ケースが見つかりません: {0}")]
    LoadCaseNotFound(String),
}

impl JobError {
    /// 機械可読な種別コード（MCP のレスポンスに載せる）。
    /// 文言（`Display`）は変わり得るが、このコードは安定とする。
    pub fn kind(&self) -> &'static str {
        match self {
            JobError::Prepare(_) => "prepare",
            JobError::Solve(_) => "solve",
            JobError::Convergence(_) => "convergence",
            JobError::InvalidInput(_) => "invalid_input",
            JobError::LoadCaseNotFound(_) => "load_case_not_found",
        }
    }
}

/// 解析ジョブの結果型。
pub type JobResult<T> = Result<T, JobError>;
