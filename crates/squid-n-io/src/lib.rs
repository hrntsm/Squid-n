pub mod checkpoint;
pub mod manifest;
pub mod migrate;
pub mod results;
pub mod scz;
pub mod stbridge;
pub mod wave_library;

/// テスト用の共有ユーティリティ。
#[cfg(test)]
pub(crate) mod test_util {
    /// テストが書き込む一時ディレクトリ（プロセス ID 入り）。
    ///
    /// `std::env::temp_dir()` 直下へ固定名で書き込むと、同一マシンで並行する
    /// 別プロセスのテスト実行（複数の CI ジョブ等）と衝突するため、
    /// プロセスごとに一意なサブディレクトリを介する。同一プロセス内の
    /// テストスレッド間はファイル名側の一意性（テストごとの固有名）で分離する。
    pub fn test_tmp() -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("squid-n-test-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&d);
        d
    }
}
