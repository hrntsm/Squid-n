//! 波形ライブラリ（`squid_n_io::wave_library`）に関するアクション。
//!
//! 時刻歴応答解析の入力波形（CSV）を複数プロジェクトで使い回すため、OS 標準の
//! ユーザーデータディレクトリ配下に波形を保管する。`.scz` には波形の実体を
//! 埋め込まず、ライブラリ内のファイル名と、実行時点のファイル内容の SHA-256
//! だけを保存する（`SavedAnalysisSettings`）。

use super::*;

impl App {
    /// プロジェクト読込時、保存されていた波形ライブラリの選択を復元する。
    ///
    /// - ライブラリの場所が特定できない、またはライブラリに同名のファイルが
    ///   見つからない場合は、その旨を通知して選択を解除する（実行できない
    ///   選択を残さないため）。
    /// - ファイルは見つかるが内容（SHA-256）が保存時と異なる場合は、選択は
    ///   維持したうえで通知する（ライブラリ側で上書きされている可能性がある
    ///   ことを伝える。次に実行すればハッシュは最新内容へ更新される）。
    pub(crate) fn restore_wave_library_selection(
        &mut self,
        wave_name: Option<String>,
        wave_sha256: Option<String>,
    ) {
        let Some(name) = wave_name else {
            self.wave_library_selection = None;
            self.wave_library_selected_sha256 = None;
            return;
        };
        let Some(dir) = squid_n_io::wave_library::wave_library_dir() else {
            self.report_notice(format!(
                "波形ライブラリの場所を特定できないため、選択されていた波形「{name}」を復元できません。"
            ));
            self.wave_library_selection = None;
            self.wave_library_selected_sha256 = None;
            return;
        };
        if !squid_n_io::wave_library::wave_exists(&dir, &name) {
            self.report_notice(format!(
                "波形ライブラリに「{name}」が見つかりません（削除された可能性があります）。\
                 「🌊 波形を保存…」から再登録してください。"
            ));
            self.wave_library_selection = None;
            self.wave_library_selected_sha256 = None;
            return;
        }
        self.wave_library_selection = Some(name.clone());
        self.wave_library_selected_sha256 = wave_sha256.clone();
        if let Some(saved_hash) = wave_sha256 {
            if let Ok(current_hash) = squid_n_io::wave_library::wave_sha256(&dir, &name) {
                if saved_hash != current_hash {
                    self.report_notice(format!(
                        "波形「{name}」の内容が保存時から変わっています\
                         （ライブラリ側で上書きされた可能性があります）。"
                    ));
                }
            }
        }
    }

    /// 「🌊 波形を保存…」: ファイル選択ダイアログで CSV を選び、波形ライブラリへ
    /// コピーする（実行はしない）。同名の波形が既にあれば、上書き確認ダイアログの
    /// 表示待ちにする（`pending_wave_register`。確定は [`Self::confirm_register_wave`]）。
    #[cfg(feature = "gui")]
    pub fn register_wave_to_library_dialog(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("波形 (CSV/テキスト)", &["csv", "txt", "dat"])
            .pick_file()
        else {
            return;
        };
        let Some(dir) = squid_n_io::wave_library::wave_library_dir() else {
            self.report_error("波形ライブラリの保存先を特定できませんでした。".to_string());
            return;
        };
        let file_name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        if squid_n_io::wave_library::wave_exists(&dir, &file_name) {
            self.pending_wave_register = Some(path);
            return;
        }
        self.finish_register_wave(&dir, &path);
    }

    /// 上書き確認ダイアログ「上書きする」。
    #[cfg(feature = "gui")]
    pub fn confirm_register_wave(&mut self) {
        let Some(path) = self.pending_wave_register.take() else {
            return;
        };
        let Some(dir) = squid_n_io::wave_library::wave_library_dir() else {
            self.report_error("波形ライブラリの保存先を特定できませんでした。".to_string());
            return;
        };
        self.finish_register_wave(&dir, &path);
    }

    /// 上書き確認ダイアログ「キャンセル」。
    #[cfg(feature = "gui")]
    pub fn cancel_register_wave(&mut self) {
        self.pending_wave_register = None;
    }

    #[cfg(feature = "gui")]
    fn finish_register_wave(&mut self, dir: &std::path::Path, src: &std::path::Path) {
        match squid_n_io::wave_library::register_wave(dir, src) {
            Ok(name) => self.report_notice(format!("波形ライブラリへ保存しました: {name}")),
            Err(e) => self.report_error(format!("波形ライブラリへの保存に失敗しました: {e}")),
        }
    }

    /// 「▶ 選択した波形で実行」: 波形ライブラリで選択中のファイルを読み込み、
    /// 時刻歴応答解析をジョブ実行する。実行時点のファイル内容の SHA-256 を
    /// `wave_library_selected_sha256` へ記録する（保存時にこれも同梱され、
    /// 次回読込時の内容検証に使われる）。
    #[cfg(feature = "gui")]
    pub fn run_time_history_from_library(&mut self) {
        let Some(name) = self.wave_library_selection.clone() else {
            return;
        };
        let Some(dir) = squid_n_io::wave_library::wave_library_dir() else {
            self.report_error("波形ライブラリの保存先を特定できませんでした。".to_string());
            return;
        };
        let content = match std::fs::read_to_string(dir.join(&name)) {
            Ok(c) => c,
            Err(e) => {
                self.report_error(format!("波形読込エラー: {}", e));
                return;
            }
        };
        let wave = match ground_motion_from_wave_content(&self.analysis_cfg, &content) {
            Ok(w) => w,
            Err(e) => {
                self.report_error(e);
                return;
            }
        };
        self.wave_library_selected_sha256 = squid_n_io::wave_library::wave_sha256(&dir, &name).ok();
        self.start_time_history_job(wave);
    }
}
