use std::collections::BTreeMap;
use std::path::Path;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 || args[1] != "check-deps" {
        eprintln!("Usage: cargo run -p xtask -- check-deps");
        std::process::exit(1);
    }

    let layers: &[&[&str]] = &[
        &["squid-n-core", "squid-n-math", "squid-n-material"],
        &["squid-n-section", "squid-n-load"],
        &["squid-n-edit", "squid-n-skeleton"],
        &["squid-n-element"],
        &["squid-n-solver", "squid-n-io"],
        &["squid-n-design-jp"],
        &["squid-n-job"],
        &["squid-n-mcp", "squid-n-app"],
    ];

    let layer_map: BTreeMap<&str, usize> = layers
        .iter()
        .enumerate()
        .flat_map(|(i, names)| names.iter().map(move |&n| (n, i)))
        .collect();

    let workspace_root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
    let crate_root = workspace_root.join("crates");

    let mut ok_count = 0usize;
    let mut violations = Vec::new();

    for (name, &layer_idx) in &layer_map {
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
