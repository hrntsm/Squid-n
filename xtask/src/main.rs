mod check_terms;

use std::collections::BTreeMap;
use std::path::Path;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let workspace_root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
    match args.get(1).map(String::as_str) {
        Some("check-deps") => check_deps(workspace_root),
        Some("check-terms") => check_terms::run(workspace_root),
        _ => {
            eprintln!("Usage: cargo run -p xtask -- <check-deps|check-terms>");
            std::process::exit(1);
        }
    }
}

fn check_deps(workspace_root: &Path) -> anyhow::Result<()> {
    // レイヤ順（下が下流＝依存される側）。実依存グラフ（DAG）に一致させる。
    // 上位→下位（dep_layer < layer_idx）のみ許可。同層・下位→上位は不許可。
    let layers: &[&[&str]] = &[
        &[
            "squid-n-core",
            "squid-n-math",
            "squid-n-material",
            "squid-n-ml",
        ],
        &["squid-n-section", "squid-n-load", "squid-n-gpu"],
        &["squid-n-edit", "squid-n-skeleton"],
        &["squid-n-element"],
        &["squid-n-solver", "squid-n-io"],
        &["squid-n-design-jp"],
        // 解析ジョブ（前処理・解析条件・純粋計算）。GUI（app）と MCP サーバの
        // 双方が同じ前処理・同じ解析条件で解くための共通層。
        &["squid-n-job"],
        &["squid-n-mcp", "squid-n-app"],
    ];

    // BTreeMap で走査順を固定する（HashMap では実行ごとに出力・検査順が変わり、
    // CI ログの diff 比較や違反の再現確認がしづらい）。
    let layer_map: BTreeMap<&str, usize> = layers
        .iter()
        .enumerate()
        .flat_map(|(i, names)| names.iter().map(move |&n| (n, i)))
        .collect();

    let crate_root = workspace_root.join("crates");

    // OK 件数と違反は分けて数える（かつては同じ Vec に "OK:"/"VIOLATION:" を
    // 混ぜて積み、総数を「upstream checks」件数として表示していた）。
    let mut ok_count = 0usize;
    let mut violations = Vec::new();

    for (name, &layer_idx) in &layer_map {
        // Only check crates/ subdir
        if *name == "xtask" {
            continue;
        }
        let cargo_toml = crate_root.join(name).join("Cargo.toml");
        if !cargo_toml.exists() {
            continue;
        }
        let content = std::fs::read_to_string(&cargo_toml)?;
        let parsed: toml::Value = content.parse()?;

        for (table, verb) in [
            ("dependencies", "depends on"),
            ("dev-dependencies", "dev-depends on"),
        ] {
            let Some(deps) = parsed.get(table).and_then(|d| d.as_table()) else {
                continue;
            };
            for (dep_name, _) in deps {
                let Some(&dep_layer) = layer_map.get(dep_name.as_str()) else {
                    continue;
                };
                if dep_layer < layer_idx {
                    ok_count += 1;
                } else {
                    violations.push(format!(
                        "VIOLATION: {} (layer {}) {} DOWNSTREAM {} (layer {})",
                        name, layer_idx, verb, dep_name, dep_layer
                    ));
                }
            }
        }
    }

    if !violations.is_empty() {
        for v in &violations {
            eprintln!("{}", v);
        }
        anyhow::bail!(
            "Dependency direction check failed with {} violation(s)",
            violations.len()
        );
    }

    println!(
        "All dependency directions OK ({} upstream checks)",
        ok_count
    );
    Ok(())
}
