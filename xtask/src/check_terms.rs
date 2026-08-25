//! 誤用しやすい建築構造用語の機械チェック。
//!
//! [`dev_docs/specs/用語集.md`](../../../dev_docs/specs/用語集.md) が定める禁止語が、
//! 許可されない文脈で使われていないかをコード・ドキュメント全体から検出する。
//! 「気づいたら用語集に追記する」という運用（AGENTS.md）だけでは、既に離れた場所へ
//! 同じ誤用が再度紛れ込むことを防げないため、`cargo run -p xtask -- check-terms` として
//! CI（`terms` ジョブ）へ組み込み、機械的に止める。
//!
//! # 判定方法
//!
//! 「パネル」を含む行のうち、**床・壁の文脈語も同じ行にあり**、かつ**仕口パネル・UI
//! パネルであることを示す許可語がどこにもない**行を違反として報告する。
//! 文脈語（`context_words`）を要求するのは、UI のドックパネル・断面作成パネルのような
//! 大量の正当な用法（文脈語を伴わない）を誤検知しないためである。行単位の判定なので、
//! 完全ではない（同一行に許可語をたまたま含む誤用は見逃す）。あくまで「気づき」の
//! 運用を補う機械的な安全網であり、最終判断はレビューに委ねる。
//!
//! 新しい用語集エントリを追加したら、対応するルールを [`RULES`] にも追加すること。

use std::path::{Path, PathBuf};

/// 1 つの禁止語に対する判定ルール。
struct TermRule {
    /// 用語集での見出し語（報告メッセージ用）。
    label: &'static str,
    /// 検出対象の禁止語。
    banned: &'static str,
    /// この語と同じ行にあると違反候補になる文脈語（床・壁領域の話題であることの目印）。
    /// 1 つでも含めば候補とする。
    context_words: &'static [&'static str],
    /// この語が同じ行にあれば許可する（仕口パネル・UI パネル等の正当な用法の目印）。
    /// 1 つでも含めば違反としない。
    allow_markers: &'static [&'static str],
}

const RULES: &[TermRule] = &[TermRule {
    label: "パネル",
    banned: "パネル",
    context_words: &["床", "スラブ", "版", "壁", "区画", "格子", "構面"],
    allow_markers: &[
        "仕口",
        "接合部",
        "柱梁",
        "PanelZone",
        "panel_zone",
        "Zone",
        "ゾーン",
    ],
}];

/// スキャン対象の拡張子。
const SCAN_EXTS: &[&str] = &["rs", "md"];

/// 走査から除外するディレクトリ名（ビルド生成物・vendor・VCS 内部）。
const SKIP_DIRS: &[&str] = &["target", "book", ".git", "node_modules", ".claude"];

/// 走査から除外するディレクトリ（ワークスペースルートからの相対パス、`/` 区切り）。
///
/// `dev_docs/handoff` と `dev_docs/v_and_v` は過去の決定・検証記録であり、
/// AGENTS.md の方針により「開発経緯・過去との差分」をそのまま残す場所である。
/// リネーム前の呼称（旧「パネル」等）が正しく残っているのは望ましい状態であり、
/// 機械チェックの対象にはしない（誤検知が常時発生し、チェックの実効性を損なうため）。
const SKIP_RELATIVE_DIRS: &[&str] = &["dev_docs/handoff", "dev_docs/v_and_v"];

/// 走査から除外する個別ファイル（ワークスペースルートからの相対パス、`/` 区切り）。
///
/// 用語集そのものは禁止語を解説のために書く必要があるため対象外とする。
const SKIP_FILES: &[&str] = &["dev_docs/specs/用語集.md"];

/// この文字列を含む行は判定をスキップする（`.rs` はコメント末尾へ、`.md` は
/// `<!-- xtask:allow-panel -->` のように HTML コメントへ書く）。
///
/// 文脈語（`context_words`）だけでは「仕口パネル・PanelZone を指しているが、
/// 同じ行に `allow_markers` が現れない」正当な用法を誤検知することがある。
/// xtask:allow-panel（この行自体が例示のため抑制する）
/// 誤検知だと確認したら、この抑制ではなく
/// [`TermRule::allow_markers`]／[`TermRule::context_words`] 側の見直しを先に検討し、
/// それでも個別の行でしか判別できない場合にだけこの抑制を使うこと。
const SUPPRESS_MARKER: &str = "xtask:allow-panel";

pub struct Violation {
    pub path: PathBuf,
    pub line_no: usize,
    pub line: String,
    pub label: &'static str,
}

/// `workspace_root` からの相対パスを `/` 区切りの文字列にする（Windows でも
/// [`SKIP_RELATIVE_DIRS`]／[`SKIP_FILES`] の比較が一致するように正規化する）。
fn relative_slash_path(workspace_root: &Path, path: &Path) -> String {
    path.strip_prefix(workspace_root)
        .unwrap_or(path)
        .components()
        .map(|c| c.as_os_str().to_string_lossy().into_owned())
        .collect::<Vec<_>>()
        .join("/")
}

/// ワークスペースルート配下を走査し、[`RULES`] に反する行をすべて集める。
pub fn scan(workspace_root: &Path) -> anyhow::Result<Vec<Violation>> {
    let mut violations = Vec::new();
    let mut stack = vec![workspace_root.to_path_buf()];
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(&dir)? {
            let entry = entry?;
            let path = entry.path();
            let file_type = entry.file_type()?;
            if file_type.is_dir() {
                let name = entry.file_name();
                let name = name.to_string_lossy();
                if SKIP_DIRS.iter().any(|s| *s == name) {
                    continue;
                }
                let rel = relative_slash_path(workspace_root, &path);
                if SKIP_RELATIVE_DIRS.iter().any(|s| *s == rel) {
                    continue;
                }
                stack.push(path);
                continue;
            }
            let Some(ext) = path.extension().and_then(|e| e.to_str()) else {
                continue;
            };
            if !SCAN_EXTS.contains(&ext) {
                continue;
            }
            let rel = relative_slash_path(workspace_root, &path);
            if SKIP_FILES.iter().any(|s| *s == rel) {
                continue;
            }
            let Ok(content) = std::fs::read_to_string(&path) else {
                continue; // バイナリ・非 UTF-8 は対象外。
            };
            for (i, line) in content.lines().enumerate() {
                if line.contains(SUPPRESS_MARKER) {
                    continue;
                }
                for rule in RULES {
                    if !line.contains(rule.banned) {
                        continue;
                    }
                    if !rule.context_words.iter().any(|w| line.contains(w)) {
                        continue;
                    }
                    if rule.allow_markers.iter().any(|m| line.contains(m)) {
                        continue;
                    }
                    violations.push(Violation {
                        path: path.clone(),
                        line_no: i + 1,
                        line: line.to_string(),
                        label: rule.label,
                    });
                }
            }
        }
    }
    Ok(violations)
}

pub fn run(workspace_root: &Path) -> anyhow::Result<()> {
    let violations = scan(workspace_root)?;
    if violations.is_empty() {
        println!("用語チェック OK（違反なし）");
        return Ok(());
    }
    for v in &violations {
        eprintln!(
            "{}:{}: 禁止語「{}」の疑い: {}",
            v.path.display(),
            v.line_no,
            v.label,
            v.line.trim()
        );
    }
    anyhow::bail!(
        "用語チェック失敗: {} 件。dev_docs/specs/用語集.md を参照し、\
         誤用なら直す・正当な用法なら xtask/src/check_terms.rs の allow_markers/context_words を見直すこと",
        violations.len()
    );
}
