use std::path::{Path, PathBuf};

/// check-docs の検査結果。
///
/// `summary_links`・`doc_links`・`impl_refs` は検査した件数、`errors` は 1 件 1 行の
/// エラーメッセージ（リポジトリルートからの `/` 区切り相対パス表記）を保持する。
#[derive(Default)]
pub struct Report {
    pub summary_links: usize,
    pub doc_links: usize,
    pub impl_refs: usize,
    pub errors: Vec<String>,
}

/// リポジトリルートの `docs/` を走査し、リンク切れと実装参照の欠落を報告する。
pub fn check_at(root: &Path) -> anyhow::Result<Report> {
    let docs_dir = root.join("docs");
    let summary = docs_dir.join("SUMMARY.md");
    let mut report = Report::default();

    if summary.is_file() {
        let text = std::fs::read_to_string(&summary)?;
        let base = summary.parent().unwrap_or(docs_dir.as_path());
        let rel = rel_display(root, &summary);
        for target in markdown_links(&text) {
            report.summary_links += 1;
            if !resolve_link(base, &target).is_file() {
                report
                    .errors
                    .push(format!("BROKEN SUMMARY LINK: {} -> {}", rel, target));
            }
        }

        for token in impl_refs(&text) {
            report.impl_refs += 1;
            if !root.join(&token).is_file() {
                report
                    .errors
                    .push(format!("BROKEN IMPL REF: {} -> {}", rel, token));
            }
        }
    }

    let mut files = Vec::new();
    collect_md_files(&docs_dir, &mut files)?;
    files.sort();

    for file in &files {
        if file == &summary {
            continue;
        }
        let text = std::fs::read_to_string(file)?;
        let rel = rel_display(root, file);
        let base = file.parent().unwrap_or(docs_dir.as_path());

        for target in markdown_links(&text) {
            report.doc_links += 1;
            if !resolve_link(base, &target).is_file() {
                report
                    .errors
                    .push(format!("BROKEN DOC LINK: {} -> {}", rel, target));
            }
        }

        for token in impl_refs(&text) {
            report.impl_refs += 1;
            if !root.join(&token).is_file() {
                report
                    .errors
                    .push(format!("BROKEN IMPL REF: {} -> {}", rel, token));
            }
        }
    }

    Ok(report)
}

fn collect_md_files(dir: &Path, out: &mut Vec<PathBuf>) -> anyhow::Result<()> {
    if !dir.is_dir() {
        return Ok(());
    }
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        if path.is_dir() {
            collect_md_files(&path, out)?;
        } else if path.extension().and_then(|e| e.to_str()) == Some("md") {
            out.push(path);
        }
    }
    Ok(())
}

fn rel_display(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}

/// リンク元ファイルの親ディレクトリを基準に、`/` 区切りの相対リンク先を解決する。
fn resolve_link(base_dir: &Path, target: &str) -> PathBuf {
    let mut path = base_dir.to_path_buf();
    for component in target.split('/') {
        match component {
            "" | "." => {}
            ".." => {
                path.pop();
            }
            other => path.push(other),
        }
    }
    path
}

/// 相対リンク先が `.md` のときだけ、アンカー（`#` 以降）を除いたパス部分を返す。
fn markdown_path(target: &str) -> Option<&str> {
    let target = target.trim();
    if target.is_empty() || target.starts_with('#') || target.starts_with('/') {
        return None;
    }
    if target.starts_with("http://")
        || target.starts_with("https://")
        || target.starts_with("mailto:")
        || target.contains("://")
    {
        return None;
    }
    let path = match target.split_once('#') {
        Some((path, _)) => path,
        None => target,
    };
    if path.ends_with(".md") {
        Some(path)
    } else {
        None
    }
}

/// コードフェンス内を除いた行の相対リンク先（アンカー除去済み、`.md` のみ）を返す。
fn markdown_links(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for line in content_lines(text) {
        for target in extract_link_targets(line) {
            if let Some(path) = markdown_path(&target) {
                out.push(path.to_string());
            }
        }
    }
    out
}

/// 1 行から `](...)` 形式のリンク先を抽出する。画像リンク（`![...](...)`）は除く。
fn extract_link_targets(line: &str) -> Vec<String> {
    let bytes = line.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i + 1 < bytes.len() {
        if bytes[i] == b']' && bytes[i + 1] == b'(' {
            if let Some(end) = line[i + 2..].find(')') {
                if !is_image_link(line, i) {
                    out.push(line[i + 2..i + 2 + end].to_string());
                }
                i = i + 2 + end + 1;
                continue;
            }
        }
        i += 1;
    }
    out
}

fn is_image_link(line: &str, close_bracket: usize) -> bool {
    let prefix = &line[..close_bracket];
    match prefix.rfind('[') {
        Some(open) => prefix[..open].ends_with('!'),
        None => false,
    }
}

/// コードフェンス内を除いた行のインラインコードから実装参照パスを返す。
fn impl_refs(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for line in content_lines(text) {
        for code in line.split('`').skip(1).step_by(2) {
            for token in code.split_whitespace() {
                if is_impl_ref(token) {
                    out.push(token.to_string());
                }
            }
        }
    }
    out
}

fn is_impl_ref(token: &str) -> bool {
    (token.starts_with("crates/") || token.starts_with("xtask/")) && token.ends_with(".rs")
}

/// 行頭のフェンスの文字・連続長・フェンス文字と空白のみかを返す。3 連未満はフェンスとみなさない。
fn fence_run(trimmed: &str) -> Option<(char, usize, bool)> {
    let first = trimmed.chars().next()?;
    if first != '`' && first != '~' {
        return None;
    }
    let count = trimmed.chars().take_while(|&c| c == first).count();
    if count < 3 {
        return None;
    }
    let bare = trimmed[count..].chars().all(|c| c == ' ' || c == '\t');
    Some((first, count, bare))
}

/// 行頭の blockquote マーカー（連続する `>` とそれに続く空白）を除去する。
fn strip_blockquote_prefix(line: &str) -> &str {
    let mut rest = line.trim_start();
    while let Some(after) = rest.strip_prefix('>') {
        rest = after.trim_start();
    }
    rest
}

/// 開きフェンス（` または ~ の 3 連以上）から、同じ文字で開き以上の連続長を持つ
/// フェンス文字と空白のみの閉じ行までの区間を除外した行を返す。
///
/// フェンス開閉の判定時のみ行頭の blockquote マーカーを無視する。フェンス外の行は
/// 元の行のまま返すため、抽出側は `>` 付きのリンクも従来どおり扱える。
fn content_lines(text: &str) -> Vec<&str> {
    let mut out = Vec::new();
    let mut fence: Option<(char, usize)> = None;
    for line in text.lines() {
        let fence_probe = strip_blockquote_prefix(line);
        match fence {
            None => match fence_run(fence_probe) {
                Some((ch, len, _)) => fence = Some((ch, len)),
                None => out.push(line),
            },
            Some((open_ch, open_len)) => {
                if let Some((ch, len, bare)) = fence_run(fence_probe) {
                    if ch == open_ch && len >= open_len && bare {
                        fence = None;
                    }
                }
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extracts_relative_md_links() {
        let links = markdown_links("参照: [a](./b.md) と [c](../d.md)");
        assert_eq!(links, vec!["./b.md".to_string(), "../d.md".to_string()]);
    }

    #[test]
    fn strips_anchor_from_md_link() {
        let links = markdown_links("[a](./b.md#見出し)");
        assert_eq!(links, vec!["./b.md".to_string()]);
    }

    #[test]
    fn ignores_https_links() {
        assert!(markdown_links("[a](https://example.com/x.md)").is_empty());
    }

    #[test]
    fn ignores_anchor_only_links() {
        assert!(markdown_links("[a](#見出し)").is_empty());
    }

    #[test]
    fn ignores_image_links() {
        assert!(markdown_links("![a](./b.md)").is_empty());
    }

    #[test]
    fn ignores_fenced_content() {
        let text = "```\n[a](./b.md)\n`crates/x.rs`\n```\n[c](./d.md)\n";
        assert_eq!(markdown_links(text), vec!["./d.md".to_string()]);
        assert!(impl_refs(text).is_empty());
    }

    #[test]
    fn ignores_tilde_fenced_content() {
        let text = "~~~\n[a](./b.md)\n`crates/x.rs`\n~~~\n";
        assert!(markdown_links(text).is_empty());
        assert!(impl_refs(text).is_empty());
    }

    #[test]
    fn does_not_close_backtick_fence_with_tildes() {
        let text = "```rust\n[a](./b.md)\n~~~\n`crates/x.rs`\n```\n[c](./d.md)\n";
        assert_eq!(markdown_links(text), vec!["./d.md".to_string()]);
        assert!(impl_refs(text).is_empty());
    }

    #[test]
    fn does_not_close_longer_fence_with_shorter_run() {
        let text = "````\n[a](./b.md)\n```\n`crates/x.rs`\n````\n";
        assert!(markdown_links(text).is_empty());
        assert!(impl_refs(text).is_empty());
    }

    #[test]
    fn ignores_blockquoted_fenced_content() {
        let text = "> ```rust\n> [a](./b.md)\n> `crates/x.rs`\n> ```\n[c](./d.md)\n> [e](./f.md)\n";
        assert_eq!(
            markdown_links(text),
            vec!["./d.md".to_string(), "./f.md".to_string()]
        );
        assert!(impl_refs(text).is_empty());
    }

    #[test]
    fn resolves_existing_relative_links_and_impl_refs() {
        let root = temp_root("resolve_ok");
        let docs = root.join("docs");
        let sub = docs.join("sub");
        let impl_dir = root.join("crates").join("example").join("src");
        std::fs::create_dir_all(&sub).unwrap();
        std::fs::create_dir_all(&impl_dir).unwrap();

        std::fs::write(
            docs.join("a.md"),
            "[b](b.md)\n[c](sub/c.md)\n実装 `crates/example/src/lib.rs`\n",
        )
        .unwrap();
        std::fs::write(sub.join("a.md"), "[b](../b.md)\n").unwrap();
        std::fs::write(docs.join("b.md"), "# b\n").unwrap();
        std::fs::write(sub.join("c.md"), "# c\n").unwrap();
        std::fs::write(impl_dir.join("lib.rs"), "\n").unwrap();

        let report = check_at(&root).unwrap();
        assert_eq!(report.doc_links, 3);
        assert_eq!(report.impl_refs, 1);
        assert!(
            report.errors.is_empty(),
            "unexpected errors: {:?}",
            report.errors
        );

        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn detects_missing_markdown_file() {
        let root = temp_root("missing_md");
        let docs = root.join("docs");
        std::fs::create_dir_all(&docs).unwrap();
        std::fs::write(docs.join("a.md"), "[b](./missing.md)\n").unwrap();

        let report = check_at(&root).unwrap();
        assert_eq!(report.doc_links, 1);
        assert!(report
            .errors
            .iter()
            .any(|e| e == "BROKEN DOC LINK: docs/a.md -> ./missing.md"));

        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn treats_directory_as_missing_markdown_file() {
        let root = temp_root("dir_md");
        let docs = root.join("docs");
        std::fs::create_dir_all(&docs).unwrap();
        std::fs::write(docs.join("a.md"), "[b](./sub.md)\n").unwrap();
        std::fs::create_dir_all(docs.join("sub.md")).unwrap();

        let report = check_at(&root).unwrap();
        assert_eq!(report.doc_links, 1);
        assert!(report
            .errors
            .iter()
            .any(|e| e == "BROKEN DOC LINK: docs/a.md -> ./sub.md"));

        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn extracts_crate_impl_refs() {
        let refs = impl_refs("実装 `crates/a/b.rs` と `xtask/src/c.rs`");
        assert_eq!(
            refs,
            vec!["crates/a/b.rs".to_string(), "xtask/src/c.rs".to_string()]
        );
    }

    #[test]
    fn detects_missing_impl_ref() {
        let root = temp_root("missing_impl");
        let docs = root.join("docs");
        std::fs::create_dir_all(&docs).unwrap();
        std::fs::write(docs.join("a.md"), "実装 `crates/a/b.rs`\n").unwrap();

        let report = check_at(&root).unwrap();
        assert_eq!(report.impl_refs, 1);
        assert!(report
            .errors
            .iter()
            .any(|e| e == "BROKEN IMPL REF: docs/a.md -> crates/a/b.rs"));

        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn detects_missing_impl_ref_in_summary() {
        let root = temp_root("summary_missing_impl");
        let docs = root.join("docs");
        std::fs::create_dir_all(&docs).unwrap();
        std::fs::write(docs.join("SUMMARY.md"), "実装 `crates/a/b.rs`\n").unwrap();

        let report = check_at(&root).unwrap();
        assert_eq!(report.impl_refs, 1);
        assert!(report
            .errors
            .iter()
            .any(|e| e == "BROKEN IMPL REF: docs/SUMMARY.md -> crates/a/b.rs"));

        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn resolves_existing_impl_ref_in_summary() {
        let root = temp_root("summary_existing_impl");
        let docs = root.join("docs");
        let impl_dir = root.join("crates").join("a");
        std::fs::create_dir_all(&docs).unwrap();
        std::fs::create_dir_all(&impl_dir).unwrap();
        std::fs::write(docs.join("SUMMARY.md"), "実装 `crates/a/b.rs`\n").unwrap();
        std::fs::write(impl_dir.join("b.rs"), "\n").unwrap();

        let report = check_at(&root).unwrap();
        assert_eq!(report.impl_refs, 1);
        assert!(
            report.errors.is_empty(),
            "unexpected errors: {:?}",
            report.errors
        );

        let _ = std::fs::remove_dir_all(&root);
    }

    fn temp_root(name: &str) -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "squid_n_xtask_check_docs_{}_{}",
            name,
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&path);
        path
    }
}
