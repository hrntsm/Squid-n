//! プロジェクト／ST-Bridge の入出力とモデル読み込み。
//!
//! `actions` からの構造分割。アルゴリズム変更は行わない。

use super::*;

/// 保存確認を出す解析結果サイズの閾値 [byte]。直列化した解析結果（時刻歴の
/// 詳細記録を含む）がこれを超える場合、詳細記録を保存に含めるかを確認する。
/// 読込側の上限（`squid_n_io::scz`、4 GiB）より十分小さい値とする。
pub(crate) const SAVE_RECORDING_CONFIRM_BYTES: usize = 512 * 1024 * 1024;

/// 保存前の確認ダイアログが必要か（解析結果の直列化サイズが閾値超過、かつ
/// 時刻歴の詳細記録を含む場合のみ）。
pub(crate) fn needs_recording_confirm(results_bytes: usize, has_recording: bool) -> bool {
    has_recording && results_bytes > SAVE_RECORDING_CONFIRM_BYTES
}

impl App {
    /// モデルを丸ごと差し替える（新規作成・サンプル読込・ファイル読込で共用）。
    /// undo 履歴・結果・選択・stale 状態をすべてリセットする。
    /// 旧スキーマの自動生成荷重ケース名（「床荷重(自動)」「自重(自動)」等）は
    /// 標準ケース名（DL・LL(架構用)・LL(地震用)）へ移行する。
    pub fn load_model(&mut self, mut model: squid_n_core::model::Model) {
        model.migrate_legacy_auto_load_cases();
        self.model = model;
        self.results = None;
        self.selection = Selection::default();
        self.undo = UndoStack::new();
        self.nav = Navigator::default();
        self.last_static = None;
        self.last_error = None;
        self.last_notice = None;
        self.auto_load_sync_hash = None;
        self.preparation = None;
        self.diagnostics.clear();
        self.staleness = Staleness::default();
        // 旧モデル由来の解析結果・準備計算表示・詳細ウィンドウの選択部材など、
        // モデル差し替えで無効になる状態をすべてリセットする（従来は
        // results/selection 等のみで、時刻歴データ・質点系応答・仕口パネル一覧・
        // 各種ドラフトが旧モデルの ID を指したまま残っていた）。
        //
        // 実行中のバックグラウンド解析ジョブも破棄する。残したままだと、旧モデルで
        // 計算中の結果が完了時に poll_job 経由で新モデルへ「最新結果」として適用され、
        // 別モデルの変位・応力が stale 警告なしに表示される（受信側 Receiver の破棄
        // だけでよい。ワーカースレッドの送信は失敗して静かに終了する）。
        self.job = None;
        // 保存確認ダイアログの保留も破棄する（旧モデル用に選んだパスへ
        // 新モデルを保存してしまうのを防ぐ）。
        self.pending_save_recording = None;
        self.pushover_view_dir = SeismicDir::X;
        self.view_vibration_case = None;
        self.view_lumped_vibration_case = None;
        self.stick_response = None;
        self.combo_error = None;
        self.generated_panels.clear();
        // 波形ライブラリの選択も旧モデル由来の状態なので破棄する（そうしないと、
        // 波形を選択したプロジェクトAの後に別プロジェクトBを開いた際、Bでは
        // 一度も選んでいない波形がAの選択のまま持ち越され、Bを保存すると
        // 実際には使っていない波形がB側に記録されてしまう）。
        self.wave_library_selection = None;
        self.wave_library_selected_sha256 = None;
        #[cfg(feature = "gui")]
        {
            self.frame_target = None;
            self.analysis_target = None;
            self.hinge_detail_elem = None;
            self.hinge_mn_cache = None;
            self.th_detail_elem = None;
            self.th_detail_axial_component = None;
            self.th_scale_cache = None;
            self.th_frame = 0;
            self.th_playing = false;
            self.th_play_time = 0.0;
            self.time_history_data = crate::time_history_view::TimeHistoryData::default();
            self.section_draft = Default::default();
            self.catalog_draft = Default::default();
            self.isolator_support_draft = Default::default();
            self.isolator_member_draft = Default::default();
            self.damper_def_draft = Default::default();
            self.combo_draft = ComboDraft::default();
            self.slab_draft = Default::default();
            self.new_story_draft = (String::new(), 0.0);
            self.pending_story_cmds.clear();
            // 階への複製ダイアログは、旧モデルの階を指したままにすると新モデルの
            // 別の階へ配ってしまうため、選択ごと閉じる。
            self.story_copy = Default::default();
            self.wall_attr_draft = Default::default();
            self.misc_wall_draft = Default::default();
            self.axis_name_draft = Default::default();
            self.load_cfg_draft = Default::default();
            self.member_detail_draft = Default::default();
            self.steel_attr_draft = Default::default();
        }
        #[cfg(feature = "gui")]
        self.reset_draw_modes();
        self.sync_node_edit();
    }

    /// 作成モード（梁・壁・スラブ）とその選択バッファをすべて解除する。
    /// モデル差し替え時は選択中の節点 id が新モデルでは別の節点（または範囲外）を
    /// 指すため、残したままにすると意図しない部材が生成されうる。
    #[cfg(feature = "gui")]
    pub(crate) fn reset_draw_modes(&mut self) {
        self.beam_draw_mode = false;
        self.beam_draw_first = None;
        self.wall_draw_mode = false;
        self.wall_draw_nodes.clear();
        self.slab_draw_mode = false;
        self.slab_draw_nodes.clear();
    }

    /// プロジェクトを指定パスへ保存する。成功時は project_path と未保存フラグを更新。
    ///
    /// 準備計算の結果・解析結果は、いずれも**最新（モデル編集後に再実行済み）の
    /// 場合のみ**同梱する（`preparation_stale` / `results_stale` なら保存しない）。
    /// 読込側が「保存されている＝そのモデルに対して最新」と扱えるようにするため。
    /// 解析結果を同梱しないときは、振動荷重ケースもモデルから外して書く
    /// （結果のない空ケースがナビに残らないようにする。メモリ上のケースは残す）。
    ///
    /// 解析結果の直列化サイズが [`SAVE_RECORDING_CONFIRM_BYTES`] を超え、かつ
    /// 時刻歴の詳細記録（`ThRecording`）を含む場合は、保存せずに確認保留
    /// （`pending_save_recording`）をセットして戻る。確認ダイアログの選択に
    /// 応じて [`Self::save_project_to_opts`] が `Some(true)`（含めて保存）／
    /// `Some(false)`（除外して保存）で再入する。
    pub fn save_project_to(&mut self, path: std::path::PathBuf) {
        self.save_project_to_opts(path, None);
    }

    /// [`Self::save_project_to`] の本体。`include_recording` が `None` のとき、
    /// サイズ超過なら確認保留をセットして保存を中断する。`Some(false)` なら
    /// 時刻歴の詳細記録を除外して保存する（メモリ上の記録は保持したまま）。
    pub fn save_project_to_opts(
        &mut self,
        path: std::path::PathBuf,
        include_recording: Option<bool>,
    ) {
        self.last_error = None;
        // 保存直前に表示中方向の増分解析結果を `pushover` 窓口へ同期する。
        self.sync_pushover_for_save();
        // self を可変借用する `encoded_or_notice` の前に、直列化まで済ませておく。
        let prep = self
            .preparation
            .as_ref()
            .filter(|_| !self.staleness.preparation_stale)
            .map(rmp_serde::to_vec);
        // 時刻歴の詳細記録（`ThRecording`）も解析結果の一部として保存する。
        // 実建物規模では数百MB級になり得るため、既定の閾値を超える場合のみ
        // 確認ダイアログで含める/除外するをユーザーが選ぶ。`Some(false)` の
        // ときは `take()` で一時的に取り除いて直列化し、直後に戻す
        // （記録本体の複製コストを避ける。保存はメモリ上の結果に対し非破壊）。
        let exclude_recording = include_recording == Some(false);
        let taken_recordings = if exclude_recording {
            self.results
                .as_mut()
                .map(|bundle| bundle.take_th_recordings())
        } else {
            None
        };
        let persist_results = self.results.is_some() && !self.staleness.results_stale;
        let results = self
            .results
            .as_ref()
            .filter(|_| persist_results)
            .map(|bundle| SavedResults {
                bundle: bundle.clone(),
                last_static: self.last_static,
                view_vibration_case: self.view_vibration_case,
                view_lumped_vibration_case: self.view_lumped_vibration_case,
                last_run: self.staleness.last_run,
            })
            .as_ref()
            .map(rmp_serde::to_vec);
        if let Some(taken) = taken_recordings {
            if let Some(bundle) = self.results.as_mut() {
                bundle.restore_th_recordings(taken);
            }
        }
        // 解析タブの設定値は、モデルの新陳（staleness）と無関係に常に同梱する
        // （結果を生成した条件そのものであり、結果が古くても・結果がなくても
        // 現在の設定は保存する意味がある）。波形ライブラリの選択（ファイル名・
        // 実行時点のハッシュ）も同じエントリに含める。
        let analysis_settings = Some(rmp_serde::to_vec(&SavedAnalysisSettings {
            cfg: self.analysis_cfg,
            wave_name: self.wave_library_selection.clone(),
            wave_sha256: self.wave_library_selected_sha256.clone(),
            lumped_wave_name: self.lumped_wave_library_selection.clone(),
            lumped_wave_sha256: self.lumped_wave_library_selected_sha256.clone(),
        }));
        let prep_bytes = self.encoded_or_notice(prep, "準備計算の結果");
        let results_bytes = self.encoded_or_notice(results, "解析結果");
        let analysis_settings_bytes = self.encoded_or_notice(analysis_settings, "解析タブの設定値");

        // サイズ超過の確認（未確認の初回のみ）。詳細記録を含む結果が閾値を
        // 超える場合は保存せず、確認ダイアログの表示を要求して戻る。
        if include_recording.is_none() {
            let has_recording = self
                .results
                .as_ref()
                .filter(|_| persist_results)
                .is_some_and(|b| b.has_th_recording());
            let size = results_bytes.as_ref().map(|b| b.len()).unwrap_or(0);
            if needs_recording_confirm(size, has_recording) {
                self.pending_save_recording = Some((path, size as u64 / (1024 * 1024)));
                return;
            }
        }

        let extras = squid_n_io::scz::SczExtras {
            preparation: prep_bytes.as_deref(),
            results: results_bytes.as_deref(),
            analysis_settings: analysis_settings_bytes.as_deref(),
        };
        // 解析結果を同梱しないときは振動ケースもモデルから外して書く。
        // 結果なしのケースがナビに残ると、未実行なのに実行済みに見える。
        // メモリ上のケースは保存後も残す（画面上の結果はまだあるため）。
        let taken_vibration_cases = if persist_results {
            None
        } else {
            Some((
                std::mem::take(&mut self.model.vibration_cases),
                std::mem::take(&mut self.model.lumped_vibration_cases),
            ))
        };
        let save_result = squid_n_io::scz::save_scz(&path, &self.model, extras);
        if let Some((spatial, lumped)) = taken_vibration_cases {
            self.model.vibration_cases = spatial;
            self.model.lumped_vibration_cases = lumped;
        }
        match save_result {
            Ok(()) => {
                // ショートカット保存はダイアログも出ず無反応になるため、
                // 成功をステータスバーとログで明示する。
                let name = path
                    .file_name()
                    .map(|n| n.to_string_lossy().into_owned())
                    .unwrap_or_else(|| path.display().to_string());
                self.report_notice(format!("保存しました: {}", name));
                self.project_path = Some(path);
                self.staleness.unsaved_changes = false;
            }
            Err(e) => self.report_error(format!("保存エラー: {}", e)),
        }
    }

    /// 時刻歴の詳細記録を除いた解析結果のみで保存する（確認ダイアログの
    /// 「除外して保存」）。
    pub fn save_project_without_recording(&mut self, path: std::path::PathBuf) {
        self.save_project_to_opts(path, Some(false));
    }

    /// 保存用の派生データの直列化結果を受け取り、失敗していれば注意を報告する。
    ///
    /// 準備計算・解析結果はいずれもモデルから再計算できる派生データなので、
    /// 直列化に失敗しても保存自体は続行する（モデルは保存し、注意を報告する）。
    fn encoded_or_notice(
        &mut self,
        encoded: Option<Result<Vec<u8>, rmp_serde::encode::Error>>,
        label: &str,
    ) -> Option<Vec<u8>> {
        match encoded {
            Some(Ok(bytes)) => Some(bytes),
            Some(Err(e)) => {
                self.report_notice(format!(
                    "{}を保存できませんでした（モデルは保存します）: {}",
                    label, e
                ));
                None
            }
            None => None,
        }
    }

    /// プロジェクトを指定パスから読み込む。成功時はモデルを差し替える。
    ///
    /// 準備計算の結果・解析結果・解析タブの設定値が同梱されていれば復元し、
    /// 実行済み扱いにする（保存側が最新のときだけ書き出すため、同梱＝そのモデルに
    /// 対して最新である）。準備計算・解析結果は、同梱がない・復号に失敗した場合は
    /// 未実行のままとし、解析実行時または「準備計算 実行」で再計算する。解析タブの
    /// 設定値は、同梱がない・復号に失敗した場合は現状（既定値）のまま変更しない
    /// （旧プロジェクトファイルには含まれないため）。ただし質量モデルの方式
    /// （`mass_method`）だけは、解析タブの設定値の同梱有無によらず、読み込んだ
    /// モデル側の値へ必ず同期する（単一情報源の原則。詳細は本体側のコメント参照）。
    pub fn open_project_from(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        match squid_n_io::scz::load_scz(&path) {
            Ok(contents) => {
                if let Err(e) = contents.model.validate() {
                    self.report_error(format!("読込モデルの検証エラー: {:?}", e));
                    return;
                }
                self.load_model(contents.model);
                if let Some(saved) = self.decode_on_load::<SavedAnalysisSettings>(
                    contents.analysis_settings,
                    "解析タブの設定値",
                ) {
                    self.analysis_cfg = saved.cfg;
                    self.restore_wave_library_selection(saved.wave_name, saved.wave_sha256);
                    self.restore_lumped_wave_library_selection(
                        saved.lumped_wave_name,
                        saved.lumped_wave_sha256,
                    );
                }
                // 質量モデルの方式は `Model::mass_method` が単一情報源（階の
                // 自動生成の実行時に `analysis_cfg.mass_method` からモデルへ
                // 反映される片方向の関係）。解析タブの設定値が同梱されていた
                // 場合はその復元値と、同梱されていない旧プロジェクトファイルの
                // 場合は読込前の値（前のプロジェクトや既定値）と、それぞれ食い違い
                // うる。この行を上の if 節の中に置くと後者（同梱なし）で
                // スキップされ、パネル表示とモデルが実際に使う方式が食い違ったまま
                // 気づけなくなるため、同梱の有無によらず必ずモデル側の値で
                // 上書きする（単一情報源の原則）。
                self.analysis_cfg.mass_method = self.model.mass_method;
                if let Some(prep) =
                    self.decode_on_load::<PreparationResult>(contents.preparation, "準備計算の結果")
                {
                    self.preparation = Some(prep);
                    self.staleness.preparation_stale = false;
                }
                if let Some(saved) =
                    self.decode_on_load::<SavedResults>(contents.results, "解析結果")
                {
                    let mut bundle = saved.bundle;
                    bundle.migrate_legacy_pushover(self.analysis_cfg.push_dir);
                    bundle.migrate_legacy_time_history(
                        &mut self.model,
                        self.wave_library_selection.as_deref().unwrap_or("サンプル"),
                        crate::app::vibration::vibration_th_dir_from_th(self.analysis_cfg.th_dir),
                        self.analysis_cfg.th_nonlinear,
                    );
                    bundle.migrate_legacy_lumped(
                        &mut self.model,
                        self.lumped_wave_library_selection
                            .as_deref()
                            .unwrap_or("サンプル"),
                        crate::app::vibration::lumped_vibration_dir_from_seismic(
                            self.analysis_cfg.lumped_dir,
                        ),
                        self.analysis_cfg.lumped_nonlinear,
                        crate::app::vibration::lumped_vibration_dim_from_stick(
                            self.analysis_cfg.lumped_dim,
                        ),
                    );
                    self.pushover_view_dir =
                        bundle.infer_pushover_view_dir(self.analysis_cfg.push_dir);
                    bundle.pushover = bundle.pushover_for_dir(self.pushover_view_dir).cloned();
                    self.results = Some(bundle);
                    self.last_static = saved.last_static;
                    self.hydrate_saved_vibration_views(
                        saved.view_vibration_case,
                        saved.view_lumped_vibration_case,
                    );
                    self.staleness.last_run = saved.last_run;
                    // 保存側が最新のときだけ書き出すため、復元できた結果は
                    // モデルと整合している（断面検定の結果も同梱されている）。
                    self.staleness.results_stale = false;
                    self.staleness.design_stale = false;
                }
                self.prune_orphan_vibration_cases();
                self.project_path = Some(path);
            }
            Err(e) => self.report_error(format!("読込エラー: {}", e)),
        }
    }

    /// 読み込んだ派生データを復号する。失敗しても読込自体は続行し、
    /// 未復元（再計算が必要）である旨を注意として報告する。
    fn decode_on_load<T: serde::de::DeserializeOwned>(
        &mut self,
        bytes: Option<Vec<u8>>,
        label: &str,
    ) -> Option<T> {
        let bytes = bytes?;
        match rmp_serde::from_slice::<T>(&bytes) {
            Ok(value) => Some(value),
            Err(e) => {
                self.report_notice(format!(
                    "保存された{}を読み込めませんでした（再実行が必要です）: {}",
                    label, e
                ));
                None
            }
        }
    }

    /// ST-Bridge（XML, サブセット）ファイルを読み込む。
    /// Squid-n プロジェクト（.scz）とは別物なので project_path はクリアする。
    /// ファイルが荷重情報（`StbLoadCase`）を持たない場合は、標準荷重ケース
    /// （DL・LL(架構用)・LL(地震用)・EX・EY）と標準荷重組合せ（長期 DL+LL、
    /// 短期地震 DL+LL±EX・DL+LL±EY）を自動作成する（新規モデルと同じ出発点。
    /// DL の自重・スラブ荷重は解析実行前の同期アクションが自動計算する）。
    pub fn import_stbridge_from(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        let xml = match squid_n_io::stbridge::read_stbridge_file(&path) {
            Ok(s) => s,
            Err(e) => {
                self.report_error(format!("ST-Bridge読込エラー: {}", e));
                return;
            }
        };
        match squid_n_io::stbridge::import_stbridge_with_report(&xml) {
            Ok((mut model, report)) => {
                if let Err(e) = model.validate() {
                    self.report_error(format!("ST-Bridge読込モデルの検証エラー: {:?}", e));
                    return;
                }
                if model.load_cases.is_empty() {
                    model.load_cases = squid_n_core::model::default_load_cases();
                    // 荷重ケースを補完した場合は標準荷重組合せも用意する（新規モデルと同じ出発点）。
                    if model.combinations.is_empty() {
                        model.combinations = squid_n_core::model::default_combinations();
                    }
                }
                self.load_model(model);
                self.project_path = None;
                self.log_attribute_dispositions(&report);
                // 欠落・近似（warnings）と自動補完の仮定（notes。支点の自動設定など）が
                // あれば注意として表示する（致命的ではない）。
                let mut lines: Vec<String> = report
                    .warnings
                    .iter()
                    .chain(report.notes.iter())
                    .cloned()
                    .collect();
                // 属性の扱いは件数が多く（実ファイルで数十種類）そのまま並べると読めないため、
                // ここは要約 1 行に留めて全量はログへ出す。
                let dropped = report.dropped_attributes().count();
                if dropped > 0 {
                    lines.push(format!(
                        "取り込まなかった属性が {dropped} 種類あります（扱いの全量はログを参照）"
                    ));
                }
                if !lines.is_empty() {
                    self.report_error(format!(
                        "⚠️ ST-Bridge 取り込み時の注意:\n- {}",
                        lines.join("\n- ")
                    ));
                }
            }
            Err(e) => self.report_error(format!("ST-Bridge読込エラー: {}", e)),
        }
    }

    /// ファイルに現れた属性の扱いをすべてログへ出す。
    ///
    /// 取り込まなかった属性を先に、取り込んだ属性を後に並べる。無視リストを持たない
    /// 設計なので `guid` のように解析へ用いない属性も「未取り込み」として現れるが、
    /// 「どの属性がどう扱われたか」を利用者が漏れなく追えることを優先する。
    fn log_attribute_dispositions(&mut self, report: &squid_n_io::stbridge::ImportReport) {
        if report.attributes.is_empty() {
            return;
        }
        let dropped: Vec<&squid_n_io::stbridge::AttrDisposition> =
            report.dropped_attributes().collect();
        if dropped.is_empty() {
            self.log.push(
                crate::app::LogLevel::Info,
                "ST-Bridge 取り込み: ファイルの属性はすべて取り込みました",
            );
        } else {
            self.log.push(
                crate::app::LogLevel::Notice,
                format!(
                    "ST-Bridge 取り込み: 取り込まなかった属性 {} 種類\n  {}",
                    dropped.len(),
                    dropped
                        .iter()
                        .map(|a| a.log_line())
                        .collect::<Vec<_>>()
                        .join("\n  ")
                ),
            );
        }
        let kept: Vec<String> = report
            .attributes
            .iter()
            .filter(|a| !a.is_dropped())
            .map(|a| a.log_line())
            .collect();
        if !kept.is_empty() {
            self.log.push(
                crate::app::LogLevel::Info,
                format!(
                    "ST-Bridge 取り込み: 取り込んだ属性 {} 種類\n  {}",
                    kept.len(),
                    kept.join("\n  ")
                ),
            );
        }
    }

    /// モデルを標準 ST-Bridge 2.0.2（XML, 幾何サブセット）として指定パスへ書き出す。
    pub fn export_stbridge_to(&mut self, path: std::path::PathBuf) {
        self.last_error = None;
        match squid_n_io::stbridge::export_stbridge(&self.model) {
            Ok(xml) => {
                if let Err(e) = std::fs::write(&path, xml) {
                    self.report_error(format!("ST-Bridge書出エラー: {}", e));
                    return;
                }
                // 平行芯以外の通り芯（円弧芯・放射芯・作図芯）は幾何を保持して
                // いないため書き出せない。無言で落とさず利用者へ知らせる。
                let dropped = self
                    .model
                    .axes
                    .iter()
                    .filter(|g| g.kind == squid_n_core::model::AxisGroupKind::Other)
                    .count();
                if dropped > 0 {
                    self.report_notice(format!(
                        "通り芯のうち平行芯でないグループ {dropped} 件は ST-Bridge へ書き出せないため除きました"
                    ));
                }
            }
            Err(e) => self.report_error(format!("ST-Bridge書出エラー: {}", e)),
        }
    }
}
