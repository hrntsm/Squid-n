use crate::manifest::Manifest;
use crate::migrate::migrate;
use sha2::{Digest, Sha256};
use squid_n_core::model::Model;
use std::io::{Read, Write};
use std::path::Path;

// 未リリースのため後方互換なし。リリース前のスキーマ変更は版を上げずにこのまま 1 とする。
pub const CURRENT_SCHEMA_VERSION: u32 = 1;

/// manifest への記載が必須な zip エントリ。ここに無い名前はハッシュ検証されないまま
/// 読み込まれてしまうため、読込時に存在を強制する。
const REQUIRED_ENTRIES: [&str; 2] = ["model.msgpack", "settings.json"];

/// zip エントリ 1 個あたりの最大展開サイズ [byte]（zip 爆弾／DoS 対策）。
///
/// .scz は単層 zip（deflate）であり、deflate の展開率は理論上約 1032 倍が
/// 上限のため、絶対サイズ上限が実効的なメモリ保護になる（展開率ベースの
/// 判定は、ゼロ成分の多い正当な解析結果でも高圧縮になり誤検知しうるため
/// 採用しない）。時刻歴の詳細記録など正当に数百 MB 級となる結果エントリを
/// 拒否しないよう、上限は 4 GiB とする（保存側はこれより十分小さい閾値で
/// ユーザーに確認を取る。`squid-n-app` の保存確認ダイアログ参照）。
const MAX_ENTRY_UNCOMPRESSED: u64 = 4 * 1024 * 1024 * 1024;

#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("zip: {0}")]
    Zip(String),
    #[error("decode: {0}")]
    Decode(String),
    #[error("hash mismatch for entry {0}")]
    HashMismatch(String),
    #[error("manifest missing required entry {0}")]
    MissingEntry(String),
    #[error("unsupported schema version: {0}")]
    UnsupportedVersion(u32),
    #[error("entry {0} exceeds max uncompressed size")]
    EntryTooLarge(String),
}

fn sha256_of(data: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(data);
    format!("{:x}", hasher.finalize())
}

/// zip エントリを展開サイズ上限付きで読み込む（zip 爆弾対策）。
/// ヘッダ申告サイズで早期に弾き、申告が嘘でも `take` で実バイトを上限に縛る
/// （メモリを使い切る前に止まる）。
fn read_entry_capped(
    archive: &mut zip::ZipArchive<std::fs::File>,
    name: &str,
) -> Result<Vec<u8>, IoError> {
    let mut zf = archive
        .by_name(name)
        .map_err(|e| IoError::Zip(format!("missing entry {}: {}", name, e)))?;
    if zf.size() > MAX_ENTRY_UNCOMPRESSED {
        return Err(IoError::EntryTooLarge(name.to_string()));
    }
    let mut data = Vec::new();
    let read = (&mut zf)
        .take(MAX_ENTRY_UNCOMPRESSED + 1)
        .read_to_end(&mut data)?;
    if read as u64 > MAX_ENTRY_UNCOMPRESSED {
        return Err(IoError::EntryTooLarge(name.to_string()));
    }
    Ok(data)
}

/// 準備計算の結果を格納する zip エントリ名（任意エントリ）。
pub const PREPARATION_ENTRY: &str = "preparation.msgpack";

/// 解析結果を格納する zip エントリ名（任意エントリ）。
pub const RESULTS_ENTRY: &str = "results.msgpack";

/// モデル以外に .scz へ同梱する派生データ（いずれもモデルから再計算できるが、
/// 再計算が高価なため保存して復元するもの）。
///
/// 中身は呼び出し側（アプリ）が msgpack へ直列化したバイト列であり、io 層は
/// 内容を解釈しない（準備計算・解析結果の型はアプリ層にあるため）。`None` の
/// 項目はエントリを書かない。書く項目は manifest に列挙してハッシュ検証の
/// 対象にする。
#[derive(Default, Clone, Copy)]
pub struct SczExtras<'a> {
    /// 準備計算の結果。
    pub preparation: Option<&'a [u8]>,
    /// 解析結果。
    pub results: Option<&'a [u8]>,
}

/// [`load_scz`] の返り値。モデルと、同梱されていれば派生データ。
pub struct SczContents {
    pub model: Model,
    /// 準備計算の結果（同梱が無ければ `None`）。
    pub preparation: Option<Vec<u8>>,
    /// 解析結果（同梱が無ければ `None`）。
    pub results: Option<Vec<u8>>,
}

/// モデルと派生データを .scz へ保存する。
pub fn save_scz(path: &Path, model: &Model, extras: SczExtras<'_>) -> Result<(), IoError> {
    let tmp_path = path.with_extension("scz.tmp");

    let model_bytes = rmp_serde::to_vec(model).map_err(|e| IoError::Decode(e.to_string()))?;
    let settings_bytes = serde_json::to_vec_pretty(&serde_json::json!({
        "code": "JIS B 0001",
        "created_at": "",
    }))
    .map_err(|e| IoError::Decode(e.to_string()))?;

    let model_hash = sha256_of(&model_bytes);
    let settings_hash = sha256_of(&settings_bytes);

    let mut entries = vec![
        crate::manifest::EntryHash {
            name: "model.msgpack".to_string(),
            sha256: model_hash,
        },
        crate::manifest::EntryHash {
            name: "settings.json".to_string(),
            sha256: settings_hash,
        },
    ];
    for (name, data) in [
        (PREPARATION_ENTRY, extras.preparation),
        (RESULTS_ENTRY, extras.results),
    ] {
        if let Some(data) = data {
            entries.push(crate::manifest::EntryHash {
                name: name.to_string(),
                sha256: sha256_of(data),
            });
        }
    }

    let manifest = Manifest {
        schema_version: CURRENT_SCHEMA_VERSION,
        units: "internal: N-mm-s".to_string(),
        created_by: "squid-n-io 0.0.1".to_string(),
        entries,
    };

    let manifest_bytes =
        serde_json::to_vec_pretty(&manifest).map_err(|e| IoError::Decode(e.to_string()))?;

    {
        let f = std::fs::File::create(&tmp_path)?;
        let mut zip = zip::ZipWriter::new(f);
        let opts = zip::write::FileOptions::<()>::default()
            .compression_method(zip::CompressionMethod::Deflated);

        zip.start_file("manifest.json", opts)
            .map_err(|e| IoError::Zip(e.to_string()))?;
        zip.write_all(&manifest_bytes)?;

        zip.start_file("model.msgpack", opts)
            .map_err(|e| IoError::Zip(e.to_string()))?;
        zip.write_all(&model_bytes)?;

        zip.start_file("settings.json", opts)
            .map_err(|e| IoError::Zip(e.to_string()))?;
        zip.write_all(&settings_bytes)?;

        for (name, data) in [
            (PREPARATION_ENTRY, extras.preparation),
            (RESULTS_ENTRY, extras.results),
        ] {
            if let Some(data) = data {
                zip.start_file(name, opts)
                    .map_err(|e| IoError::Zip(e.to_string()))?;
                zip.write_all(data)?;
            }
        }

        // rename 前に内容をディスクへ永続化する。fsync を挟まないと rename が
        // 原子的でも電源断で新ファイルが空・破損になり得る。
        let f = zip.finish().map_err(|e| IoError::Zip(e.to_string()))?;
        f.sync_all()?;
    }

    std::fs::rename(&tmp_path, path)?;
    sync_parent_dir(path)?;
    Ok(())
}

/// rename というディレクトリエントリ変更自体を永続化する（Unix のみ。
/// Windows はディレクトリを fsync できないため no-op）。
#[cfg(unix)]
fn sync_parent_dir(path: &Path) -> std::io::Result<()> {
    let parent = match path.parent() {
        Some(p) if !p.as_os_str().is_empty() => p,
        _ => Path::new("."),
    };
    std::fs::File::open(parent)?.sync_all()
}

#[cfg(not(unix))]
fn sync_parent_dir(_path: &Path) -> std::io::Result<()> {
    Ok(())
}

/// モデルと、同梱されていれば派生データ（準備計算の結果・解析結果）を読み込む。
/// 該当エントリを持たないファイル（それらが最新でない状態で保存したプロジェクト）
/// では [`SczContents`] の該当項目が `None` になる。
pub fn load_scz(path: &Path) -> Result<SczContents, IoError> {
    let f = std::fs::File::open(path)?;
    let mut archive = zip::ZipArchive::new(f).map_err(|e| IoError::Zip(e.to_string()))?;

    let manifest_bytes = read_entry_capped(&mut archive, "manifest.json")?;
    let manifest: Manifest =
        serde_json::from_slice(&manifest_bytes).map_err(|e| IoError::Decode(e.to_string()))?;

    // 未リリースのため後方互換なし。現行版以外は弾く（migrate は将来の版上げ用の骨子）。
    if manifest.schema_version != CURRENT_SCHEMA_VERSION {
        return Err(IoError::UnsupportedVersion(manifest.schema_version));
    }

    // 必須エントリが manifest に列挙されていることを強制する。これが無いと、
    // manifest から model.msgpack を落としたファイルがハッシュ未検証のまま読めてしまう。
    for required in REQUIRED_ENTRIES {
        if !manifest.entries.iter().any(|e| e.name == required) {
            return Err(IoError::MissingEntry(required.to_string()));
        }
    }

    let mut model_data = None;
    let mut preparation = None;
    let mut results = None;
    for entry in &manifest.entries {
        let data = read_entry_capped(&mut archive, &entry.name)?;
        let actual_hash = sha256_of(&data);
        if actual_hash != entry.sha256 {
            return Err(IoError::HashMismatch(entry.name.clone()));
        }
        match entry.name.as_str() {
            "model.msgpack" => model_data = Some(data),
            PREPARATION_ENTRY => preparation = Some(data),
            RESULTS_ENTRY => results = Some(data),
            _ => {}
        }
    }

    // REQUIRED_ENTRIES チェック済みなので必ず Some だが、防御的に扱う。
    let model_data = model_data.ok_or_else(|| IoError::MissingEntry("model.msgpack".into()))?;

    let model_data = migrate(manifest.schema_version, model_data)?;

    let model: Model =
        rmp_serde::from_slice(&model_data).map_err(|e| IoError::Decode(e.to_string()))?;

    Ok(SczContents {
        model,
        preparation,
        results,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::*;
    use squid_n_core::model::*;

    fn make_3node_model() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [1000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [2000.0, 0.0, 0.0],
                    restraint: Dof6Mask::PINNED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_roundtrip() {
        let model = make_3node_model();
        let dir = crate::test_util::test_tmp();
        let path = dir.join("p.scz");
        save_scz(&path, &model, SczExtras::default()).unwrap();
        let back = load_scz(&path).unwrap().model;
        assert_eq!(model.nodes.len(), back.nodes.len());
        assert!(model.eq_ignoring_dofmap(&back));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_hash_mismatch() {
        let model = make_3node_model();
        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_hash.scz");
        save_scz(&path, &model, SczExtras::default()).unwrap();
        let settings_bytes = {
            let f = std::fs::File::open(&path).unwrap();
            let mut ar = zip::ZipArchive::new(f).unwrap();
            let mut sb = Vec::new();
            ar.by_name("settings.json")
                .unwrap()
                .read_to_end(&mut sb)
                .unwrap();
            sb
        };

        // model.msgpack を改竄相当のバイト列に差し替え、manifest のハッシュと食い違わせる。
        // settings.json は正しいハッシュにして、必須エントリチェックではなく
        // ハッシュ検証そのものでエラーになることを確認する。
        let bad_manifest = Manifest {
            schema_version: CURRENT_SCHEMA_VERSION,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![
                crate::manifest::EntryHash {
                    name: "model.msgpack".to_string(),
                    sha256: "badhash".to_string(),
                },
                crate::manifest::EntryHash {
                    name: "settings.json".to_string(),
                    sha256: sha256_of(&settings_bytes),
                },
            ],
        };
        write_zip_with_manifest(&path, &bad_manifest, &[0u8; 4], &settings_bytes);

        let result = load_scz(&path);
        assert!(matches!(result, Err(IoError::HashMismatch(ref name)) if name == "model.msgpack"));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_unsupported_version() {
        let model = make_3node_model();
        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_ver.scz");
        save_scz(&path, &model, SczExtras::default()).unwrap();

        let bad_manifest = Manifest {
            schema_version: 999,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![],
        };
        let bad_bytes = serde_json::to_vec(&bad_manifest).unwrap();
        let tmp_path = path.with_extension("scz.tmp");
        {
            let f = std::fs::File::create(&tmp_path).unwrap();
            let mut zip = zip::ZipWriter::new(f);
            let opts = zip::write::FileOptions::<()>::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zip.start_file("manifest.json", opts).unwrap();
            zip.write_all(&bad_bytes).unwrap();
            zip.finish().unwrap();
        }
        std::fs::rename(&tmp_path, &path).unwrap();

        let result = load_scz(&path);
        assert!(matches!(result, Err(IoError::UnsupportedVersion(999))));
        let _ = std::fs::remove_file(&path);
    }

    /// 未リリースのため後方互換なし: 現行版以外（例: 旧版 2 を名乗るファイル）は
    /// UnsupportedVersion で拒否されること。
    #[test]
    fn test_old_version_rejected() {
        let model = make_3node_model();
        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_old_ver.scz");
        // まず通常保存し、その model.msgpack / settings.json を取り出す。
        save_scz(&path, &model, SczExtras::default()).unwrap();
        let (model_bytes, settings_bytes) = {
            let f = std::fs::File::open(&path).unwrap();
            let mut ar = zip::ZipArchive::new(f).unwrap();
            let mut mb = Vec::new();
            ar.by_name("model.msgpack")
                .unwrap()
                .read_to_end(&mut mb)
                .unwrap();
            let mut sb = Vec::new();
            ar.by_name("settings.json")
                .unwrap()
                .read_to_end(&mut sb)
                .unwrap();
            (mb, sb)
        };

        // 版を 2 と偽った manifest（entries のハッシュは実バイトに一致させる）。
        let manifest = Manifest {
            schema_version: 2,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![
                crate::manifest::EntryHash {
                    name: "model.msgpack".to_string(),
                    sha256: sha256_of(&model_bytes),
                },
                crate::manifest::EntryHash {
                    name: "settings.json".to_string(),
                    sha256: sha256_of(&settings_bytes),
                },
            ],
        };
        write_zip_with_manifest(&path, &manifest, &model_bytes, &settings_bytes);

        let result = load_scz(&path);
        assert!(matches!(result, Err(IoError::UnsupportedVersion(2))));
        let _ = std::fs::remove_file(&path);
    }

    /// manifest.entries から model.msgpack を落としたファイルが、ハッシュ未検証のまま
    /// 読めてしまわないこと（MissingEntry で拒否される）。
    #[test]
    fn test_manifest_missing_required_entry_rejected() {
        let model = make_3node_model();
        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_missing_entry.scz");
        save_scz(&path, &model, SczExtras::default()).unwrap();
        let (model_bytes, settings_bytes) = {
            let f = std::fs::File::open(&path).unwrap();
            let mut ar = zip::ZipArchive::new(f).unwrap();
            let mut mb = Vec::new();
            ar.by_name("model.msgpack")
                .unwrap()
                .read_to_end(&mut mb)
                .unwrap();
            let mut sb = Vec::new();
            ar.by_name("settings.json")
                .unwrap()
                .read_to_end(&mut sb)
                .unwrap();
            (mb, sb)
        };

        // model.msgpack のエントリだけを manifest から落とす（zip 内には実体を残す）。
        let manifest = Manifest {
            schema_version: CURRENT_SCHEMA_VERSION,
            units: "internal: N-mm-s".to_string(),
            created_by: "test".to_string(),
            entries: vec![crate::manifest::EntryHash {
                name: "settings.json".to_string(),
                sha256: sha256_of(&settings_bytes),
            }],
        };
        write_zip_with_manifest(&path, &manifest, &model_bytes, &settings_bytes);

        let result = load_scz(&path);
        assert!(matches!(result, Err(IoError::MissingEntry(ref name)) if name == "model.msgpack"));
        let _ = std::fs::remove_file(&path);
    }

    /// テスト用: 指定 manifest と実バイトで .scz を書き直す。
    fn write_zip_with_manifest(
        path: &Path,
        manifest: &Manifest,
        model_bytes: &[u8],
        settings_bytes: &[u8],
    ) {
        let manifest_bytes = serde_json::to_vec_pretty(manifest).unwrap();
        let tmp_path = path.with_extension("scz.tmp");
        {
            let f = std::fs::File::create(&tmp_path).unwrap();
            let mut zip = zip::ZipWriter::new(f);
            let opts = zip::write::FileOptions::<()>::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zip.start_file("manifest.json", opts).unwrap();
            zip.write_all(&manifest_bytes).unwrap();
            zip.start_file("model.msgpack", opts).unwrap();
            zip.write_all(model_bytes).unwrap();
            zip.start_file("settings.json", opts).unwrap();
            zip.write_all(settings_bytes).unwrap();
            zip.finish().unwrap();
        }
        std::fs::rename(&tmp_path, path).unwrap();
    }

    /// UI設計 §4.2: Section は SectionShape の派生。`to_section` で生成した断面を
    /// 持つモデルを保存→読込しても `shape` が失われず、Some のまま完全一致することを確認する。
    #[test]
    fn test_roundtrip_preserves_section_shape() {
        use squid_n_core::section_shape::SectionShape;

        let shape = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "H-400x200x9x12".to_string());
        assert!(section.shape.is_some());

        let mut model = make_3node_model();
        model.sections.push(section.clone());

        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_shape_roundtrip.scz");
        save_scz(&path, &model, SczExtras::default()).unwrap();
        let back = load_scz(&path).unwrap().model;

        assert_eq!(back.sections.len(), 1);
        assert_eq!(back.sections[0].shape, Some(shape));
        assert!(model.eq_ignoring_dofmap(&back));
        let _ = std::fs::remove_file(&path);
    }

    /// 準備計算の結果（アプリ層が直列化した任意バイト列）が保存→読込で往復し、
    /// manifest のハッシュ検証対象になること。
    #[test]
    fn test_roundtrip_preserves_preparation_entry() {
        let model = make_3node_model();
        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_prep_roundtrip.scz");
        let prep = b"preparation payload".to_vec();
        save_scz(
            &path,
            &model,
            SczExtras {
                preparation: Some(&prep),
                ..Default::default()
            },
        )
        .unwrap();

        let loaded = load_scz(&path).unwrap();
        let (back, back_prep) = (loaded.model, loaded.preparation);
        assert!(model.eq_ignoring_dofmap(&back));
        assert_eq!(back_prep.as_deref(), Some(prep.as_slice()));

        // manifest に列挙され、ハッシュ検証の対象になっている。
        let manifest: Manifest = {
            let f = std::fs::File::open(&path).unwrap();
            let mut ar = zip::ZipArchive::new(f).unwrap();
            let mut mb = Vec::new();
            ar.by_name("manifest.json")
                .unwrap()
                .read_to_end(&mut mb)
                .unwrap();
            serde_json::from_slice(&mb).unwrap()
        };
        let entry = manifest
            .entries
            .iter()
            .find(|e| e.name == PREPARATION_ENTRY)
            .expect("準備計算エントリが manifest にあるはず");
        assert_eq!(entry.sha256, sha256_of(&prep));

        let _ = std::fs::remove_file(&path);
    }

    /// 準備計算を同梱しないで保存したファイルは、準備計算が `None` として読める。
    #[test]
    fn test_load_without_preparation_entry() {
        let model = make_3node_model();
        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_prep_absent.scz");
        save_scz(&path, &model, SczExtras::default()).unwrap();

        let loaded = load_scz(&path).unwrap();
        let (back, back_prep) = (loaded.model, loaded.preparation);
        assert!(model.eq_ignoring_dofmap(&back));
        assert!(back_prep.is_none());
        let _ = std::fs::remove_file(&path);
    }

    /// 一般ブレース要素（軸剛性 KB=E·A/L は材料力学）対応: `ElementKind::Brace`
    /// （構造体バリアント `tension_only`）を持つ要素が保存→読込で完全一致すること。
    #[test]
    fn test_roundtrip_preserves_brace_element() {
        let mut model = make_3node_model();
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Brace { tension_only: true },
            nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Pinned, EndCondition::Pinned],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });

        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_brace_roundtrip.scz");
        save_scz(&path, &model, SczExtras::default()).unwrap();
        let back = load_scz(&path).unwrap().model;

        assert_eq!(back.elements.len(), 1);
        assert_eq!(
            back.elements[0].kind,
            ElementKind::Brace { tension_only: true }
        );
        assert!(model.eq_ignoring_dofmap(&back));
        let _ = std::fs::remove_file(&path);
    }

    /// 部材付帯情報（ハンチ・継手位置）: `Model::member_detail_attrs`
    /// （両端ハンチあり・片端のみハンチ・継手複数（`JointKind::Shop` を含む））
    /// を持つモデルが保存→読込で完全一致すること。
    #[test]
    fn test_roundtrip_preserves_member_detail_attrs() {
        let mut model = make_3node_model();
        model.member_detail_attrs = vec![
            // 両端ハンチあり + 継手複数（現場・工場混在）
            MemberDetailAttr {
                elem: ElemId(0),
                haunch_i: Some(Haunch {
                    length: 700.0,
                    depth_increase: 200.0,
                    width_increase: 50.0,
                }),
                haunch_j: Some(Haunch {
                    length: 500.0,
                    depth_increase: 150.0,
                    width_increase: 0.0,
                }),
                joints: vec![
                    MemberJoint {
                        distance: 1000.0,
                        kind: JointKind::Site,
                    },
                    MemberJoint {
                        distance: 3000.0,
                        kind: JointKind::Shop,
                    },
                ],
            },
            // 片端のみハンチ（i 端のみ）、継手なし
            MemberDetailAttr {
                elem: ElemId(1),
                haunch_i: Some(Haunch {
                    length: 400.0,
                    depth_increase: 100.0,
                    width_increase: 0.0,
                }),
                haunch_j: None,
                joints: Vec::new(),
            },
        ];

        let dir = crate::test_util::test_tmp();
        let path = dir.join("p_member_detail_roundtrip.scz");
        save_scz(&path, &model, SczExtras::default()).unwrap();
        let back = load_scz(&path).unwrap().model;

        assert_eq!(back.member_detail_attrs, model.member_detail_attrs);
        assert!(model.eq_ignoring_dofmap(&back));
        let _ = std::fs::remove_file(&path);
    }
}
