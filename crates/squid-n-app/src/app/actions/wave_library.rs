//! 波形ライブラリ（`squid_n_io::wave_library`）に関するアクション。
//!
//! 時刻歴応答解析の入力波形（CSV）を複数プロジェクトで使い回すため、OS 標準の
//! ユーザーデータディレクトリ配下に波形を保管する。`.scz` には波形の実体を
//! 埋め込まず、ライブラリ内のファイル名と、実行時点のファイル内容の SHA-256
//! だけを保存する（`SavedAnalysisSettings`）。

use super::*;

impl App {
    /// 波形ライブラリの選択を変更する（ドロップダウン UI 用）。選択が実際に
    /// 変わった場合のみ、実行時点のハッシュ（`wave_library_selected_sha256`）を
    /// 破棄する。
    ///
    /// `wave_library_selection` を経由せず直接書き換えると、波形 A を実行した
    /// あとにドロップダウンで波形 B へ選び直しただけで、A のハッシュを持った
    /// まま B が保存できてしまう（一度も実行していない name-hash の組が
    /// `.scz` に残り、再読込時に「内容が変わっている」という事実と異なる
    /// 通知が出る）。ハッシュは [`Self::run_time_history_from_library`] で
    /// 実行したときにだけ結び直す。
    ///
    /// 呼び出し元（波形ライブラリのドロップダウン）は gui 機能配下にしかないため、
    /// gui 機能を無効にしたビルドでは未使用になる。テストからは呼べるよう
    /// `cfg(any(test, feature = "gui"))` とする。
    #[cfg(any(test, feature = "gui"))]
    pub(crate) fn set_wave_library_selection(&mut self, name: Option<String>) {
        if name != self.core.scoped.wave_library_selection {
            self.core.scoped.wave_library_selection = name;
            self.core.scoped.wave_library_selected_sha256 = None;
        }
    }

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
            self.core.scoped.wave_library_selection = None;
            self.core.scoped.wave_library_selected_sha256 = None;
            return;
        };
        let Some(dir) = squid_n_io::wave_library::wave_library_dir() else {
            self.report_notice(format!(
                "波形ライブラリの場所を特定できないため、選択されていた波形「{name}」を復元できません。"
            ));
            self.core.scoped.wave_library_selection = None;
            self.core.scoped.wave_library_selected_sha256 = None;
            return;
        };
        if !squid_n_io::wave_library::wave_exists(&dir, &name) {
            self.report_notice(format!(
                "波形ライブラリに「{name}」が見つかりません（削除された可能性があります）。\
                 「🌊 波形を保存…」から再登録してください。"
            ));
            self.core.scoped.wave_library_selection = None;
            self.core.scoped.wave_library_selected_sha256 = None;
            return;
        }
        self.core.scoped.wave_library_selection = Some(name.clone());
        self.core.scoped.wave_library_selected_sha256 = wave_sha256.clone();
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

    /// 質点系用の波形ライブラリ選択を復元する（立体時刻歴とは独立）。
    pub(crate) fn restore_lumped_wave_library_selection(
        &mut self,
        wave_name: Option<String>,
        wave_sha256: Option<String>,
    ) {
        let Some(name) = wave_name else {
            self.core.scoped.lumped_wave_library_selection = None;
            self.core.scoped.lumped_wave_library_selected_sha256 = None;
            return;
        };
        let Some(dir) = squid_n_io::wave_library::wave_library_dir() else {
            self.report_notice(format!(
                "波形ライブラリの場所を特定できないため、質点系の波形「{name}」を復元できません。"
            ));
            self.core.scoped.lumped_wave_library_selection = None;
            self.core.scoped.lumped_wave_library_selected_sha256 = None;
            return;
        };
        if !squid_n_io::wave_library::wave_exists(&dir, &name) {
            self.report_notice(format!(
                "波形ライブラリに質点系の波形「{name}」が見つかりません。"
            ));
            self.core.scoped.lumped_wave_library_selection = None;
            self.core.scoped.lumped_wave_library_selected_sha256 = None;
            return;
        }
        self.core.scoped.lumped_wave_library_selection = Some(name.clone());
        self.core.scoped.lumped_wave_library_selected_sha256 = wave_sha256.clone();
        if let Some(saved_hash) = wave_sha256 {
            if let Ok(current_hash) = squid_n_io::wave_library::wave_sha256(&dir, &name) {
                if saved_hash != current_hash {
                    self.report_notice(format!(
                        "質点系の波形「{name}」の内容が保存時から変わっています。"
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
            self.ui.view.pending_wave_register = Some(path);
            return;
        }
        self.finish_register_wave(&dir, &path);
    }

    /// 上書き確認ダイアログ「上書きする」。
    #[cfg(feature = "gui")]
    pub fn confirm_register_wave(&mut self) {
        let Some(path) = self.ui.view.pending_wave_register.take() else {
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
        self.ui.view.pending_wave_register = None;
    }

    #[cfg(feature = "gui")]
    fn finish_register_wave(&mut self, dir: &std::path::Path, src: &std::path::Path) {
        match squid_n_io::wave_library::register_wave(dir, src) {
            Ok(name) => self.report_notice(format!("波形ライブラリへ保存しました: {name}")),
            Err(e) => self.report_error(format!("波形ライブラリへの保存に失敗しました: {e}")),
        }
    }

    /// 波形ライブラリ内の `name` を読み、保存先ディレクトリと内容を返す。
    ///
    /// 保存先を特定できない・ファイルを読めない場合は利用者へ通知して `None`。
    /// 呼び出し側は実行時点の SHA-256 を `wave_library::wave_sha256` で算定する
    /// のに同じディレクトリを要するため、内容と一緒に返す。
    #[cfg(feature = "gui")]
    pub(crate) fn read_wave_library_file(
        &mut self,
        name: &str,
    ) -> Option<(std::path::PathBuf, String)> {
        let Some(dir) = squid_n_io::wave_library::wave_library_dir() else {
            self.report_error("波形ライブラリの保存先を特定できませんでした。".to_string());
            return None;
        };
        match std::fs::read_to_string(dir.join(name)) {
            Ok(content) => Some((dir, content)),
            Err(e) => {
                self.report_error(format!("波形読込エラー: {e}"));
                None
            }
        }
    }

    /// 波形 CSV の内容から `GroundMotion` を組み立てる。失敗は利用者へ通知して `None`。
    ///
    /// 解析設定を引数で受けるのは、質点系が `th_dt`・`th_dir` を質点系側の値
    /// （`lumped_th_dt`・`lumped_dir`）へ差し替えた写しを渡すためである。
    #[cfg(feature = "gui")]
    pub(crate) fn ground_motion_or_report(
        &mut self,
        cfg: &crate::app::AnalysisSettings,
        content: &str,
    ) -> Option<squid_n_solver::dynamic::timehistory::GroundMotion> {
        match crate::app::ground_motion_from_wave_content(cfg, content) {
            Ok(w) => Some(w),
            Err(e) => {
                self.report_error(e);
                None
            }
        }
    }

    /// 「▶ 選択した波形で実行」: 波形ライブラリで選択中のファイルを読み込み、
    /// 時刻歴応答解析をジョブ実行する。実行時点のファイル内容の SHA-256 を
    /// `wave_library_selected_sha256` へ記録する（保存時にこれも同梱され、
    /// 次回読込時の内容検証に使われる）。
    #[cfg(feature = "gui")]
    pub fn run_time_history_from_library(&mut self) {
        let Some(name) = self.core.scoped.wave_library_selection.clone() else {
            return;
        };
        let Some((dir, content)) = self.read_wave_library_file(&name) else {
            return;
        };
        let cfg = self.core.analysis_cfg;
        let Some(wave) = self.ground_motion_or_report(&cfg, &content) else {
            return;
        };
        self.core.scoped.wave_library_selected_sha256 =
            squid_n_io::wave_library::wave_sha256(&dir, &name).ok();
        self.start_time_history_job(wave);
    }
}
