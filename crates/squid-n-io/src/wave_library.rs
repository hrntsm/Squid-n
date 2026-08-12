//! 波形ライブラリ（時刻歴応答解析の入力波形を複数プロジェクトで使い回す置き場）。
//!
//! `.scz` プロジェクトファイルにはモデル・解析結果・解析タブの設定値を保存するが、
//! 時刻歴の入力波形（CSV）そのものは含めない（プロジェクトをまたいで使い回したい、
//! というユーザー要望による設計判断。波形をプロジェクトごとに埋め込むと使い回せず、
//! 元ファイルのパスだけを覚える方式は元ファイルが動く・消えると再現できない）。
//!
//! 代わりに、OS 標準のユーザーデータディレクトリ配下にアプリ専用のフォルダを設け、
//! そこへ波形 CSV をコピーして保管する。全プロジェクトから共通の置き場として参照する。
//!
//! - [`wave_library_dir`] — ライブラリのディレクトリ（OS 標準のユーザーデータディレクトリ配下）。
//! - [`list_wave_library`] — ライブラリ内の波形ファイル名一覧。
//! - [`register_wave`] — 波形 CSV をライブラリへコピーする（同名は上書き）。
//! - [`wave_sha256`] — ライブラリ内の波形ファイルの SHA-256（保存時の内容と一致するかの検証用）。
//!
//! **注意（機械ローカル）**: ライブラリはこのマシンのユーザーデータディレクトリに
//! 置かれる。別のマシンで `.scz` を開いた場合や、プロジェクトファイルを他者へ渡した
//! 場合、そのマシンのライブラリに同名の波形がなければ波形を解決できない。

use sha2::{Digest, Sha256};
use std::io;
use std::path::{Path, PathBuf};

/// 波形として扱う拡張子（大小文字を区別しない）。
const WAVE_EXTENSIONS: [&str; 3] = ["csv", "txt", "dat"];

/// 波形ライブラリのディレクトリ（OS 標準のユーザーデータディレクトリ配下の
/// `squid-n/waves`）。`dirs::data_dir()` が解決できない環境（一部のサンドボックス等）
/// では `None` を返す。ディレクトリの作成はここでは行わない
/// （呼び出し側が必要な操作に応じて `list_wave_library`／`register_wave` で行う）。
pub fn wave_library_dir() -> Option<PathBuf> {
    dirs::data_dir().map(|d| d.join("squid-n").join("waves"))
}

/// ライブラリ内の波形ファイル名一覧（拡張子が [`WAVE_EXTENSIONS`] のもの、
/// ファイル名昇順）。ディレクトリが存在しない場合は空を返す（未作成＝未登録として
/// 扱い、エラーにはしない。初回起動時に常に空でエラーになるのを避けるため）。
pub fn list_wave_library(dir: &Path) -> io::Result<Vec<String>> {
    if !dir.exists() {
        return Ok(Vec::new());
    }
    let mut names: Vec<String> = std::fs::read_dir(dir)?
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|name| {
            let ext = Path::new(name)
                .extension()
                .and_then(|e| e.to_str())
                .unwrap_or("")
                .to_ascii_lowercase();
            WAVE_EXTENSIONS.contains(&ext.as_str())
        })
        .collect();
    names.sort();
    Ok(names)
}

/// ライブラリ内に指定名の波形が既に存在するか。
/// 登録前に呼び出し側が上書き確認ダイアログを出すかどうかの判定に使う。
pub fn wave_exists(dir: &Path, file_name: &str) -> bool {
    dir.join(file_name).is_file()
}

/// 波形 CSV（`src_path`）をライブラリへコピーする。ライブラリディレクトリが
/// 存在しなければ作成する。ライブラリ内に同名ファイルがあれば上書きする
/// （呼び出し側は事前に [`wave_exists`] で確認し、必要なら利用者に確認を取ること）。
///
/// 返り値はライブラリ内でのファイル名（`src_path` のファイル名をそのまま使う）。
pub fn register_wave(dir: &Path, src_path: &Path) -> io::Result<String> {
    let file_name = src_path
        .file_name()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "ファイル名を取得できません"))?
        .to_string_lossy()
        .into_owned();
    std::fs::create_dir_all(dir)?;
    std::fs::copy(src_path, dir.join(&file_name))?;
    Ok(file_name)
}

/// ライブラリ内の波形ファイルの内容から SHA-256 を求める（16進小文字）。
/// 保存時に記録したハッシュと突き合わせ、ライブラリ側のファイルが後から
/// 差し替えられていないかを検証するために使う。
pub fn wave_sha256(dir: &Path, file_name: &str) -> io::Result<String> {
    let data = std::fs::read(dir.join(file_name))?;
    let mut hasher = Sha256::new();
    hasher.update(&data);
    Ok(format!("{:x}", hasher.finalize()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::test_tmp;

    fn unique_dir(label: &str) -> PathBuf {
        test_tmp().join(format!(
            "wave_lib_{label}_{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ))
    }

    #[test]
    fn test_list_empty_when_dir_absent() {
        let dir = unique_dir("absent");
        assert_eq!(list_wave_library(&dir).unwrap(), Vec::<String>::new());
    }

    #[test]
    fn test_register_creates_dir_and_copies() {
        let src_dir = unique_dir("src");
        std::fs::create_dir_all(&src_dir).unwrap();
        let src = src_dir.join("elcentro.csv");
        std::fs::write(&src, "1.0\n2.0\n3.0\n").unwrap();

        let lib_dir = unique_dir("lib");
        assert!(!lib_dir.exists());
        let name = register_wave(&lib_dir, &src).unwrap();
        assert_eq!(name, "elcentro.csv");
        assert!(wave_exists(&lib_dir, "elcentro.csv"));

        let names = list_wave_library(&lib_dir).unwrap();
        assert_eq!(names, vec!["elcentro.csv".to_string()]);
    }

    #[test]
    fn test_register_overwrites_same_name() {
        let src_dir = unique_dir("src2");
        std::fs::create_dir_all(&src_dir).unwrap();
        let src = src_dir.join("wave.csv");
        let lib_dir = unique_dir("lib2");

        std::fs::write(&src, "1.0\n").unwrap();
        register_wave(&lib_dir, &src).unwrap();
        let hash1 = wave_sha256(&lib_dir, "wave.csv").unwrap();

        std::fs::write(&src, "2.0\n3.0\n").unwrap();
        register_wave(&lib_dir, &src).unwrap();
        let hash2 = wave_sha256(&lib_dir, "wave.csv").unwrap();

        assert_ne!(hash1, hash2, "上書き後は内容もハッシュも変わっているはず");
        assert_eq!(
            list_wave_library(&lib_dir).unwrap().len(),
            1,
            "重複登録されない"
        );
    }

    #[test]
    fn test_list_filters_non_wave_extensions_and_sorts() {
        let dir = unique_dir("filter");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("b.csv"), "1.0\n").unwrap();
        std::fs::write(dir.join("a.txt"), "1.0\n").unwrap();
        std::fs::write(dir.join("readme.md"), "not a wave").unwrap();

        let names = list_wave_library(&dir).unwrap();
        assert_eq!(names, vec!["a.txt".to_string(), "b.csv".to_string()]);
    }

    #[test]
    fn test_wave_sha256_matches_direct_hash() {
        let dir = unique_dir("hash");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("w.csv"), "42.0\n").unwrap();

        let got = wave_sha256(&dir, "w.csv").unwrap();
        let mut hasher = Sha256::new();
        hasher.update(b"42.0\n");
        let expected = format!("{:x}", hasher.finalize());
        assert_eq!(got, expected);
    }
}
