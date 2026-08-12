//! `App` のアクション（解析実行・モデル操作）メソッド。
//!
//! 責務ごとのサブモジュール:
//! - [`design`] — 許容・終局・保有水平耐力・床内検定
//! - [`io`] — プロジェクト／ST-Bridge 入出力とモデル読込
//! - [`loads`] — 荷重ケース自動同期・床荷重分配
//! - [`wave_library`] — 時刻歴応答解析の波形ライブラリ（登録・選択実行）

use super::*;

mod design;
mod io;
mod loads;
mod wave_library;

#[cfg(test)]
pub(crate) use io::{needs_recording_confirm, SAVE_RECORDING_CONFIRM_BYTES};

/// 水平力の荷重ケース（種別が地震・風）なのに荷重が入っていないか。
///
/// 準備計算が EX/EY・WX/WY へ水平力を生成するため、空のまま残っているのは
/// 「準備計算が未実行、または階が未定義で算定できなかった」ことの合図になる。
/// これを解くと水平力の項が黙って 0 になり、長期と同じ応力を短期の検定に
/// 使ってしまうため、解析の実行前ガードに使う。
fn is_empty_lateral_case(lc: &squid_n_core::model::LoadCase) -> bool {
    use squid_n_core::model::LoadCaseKind;
    matches!(lc.kind, LoadCaseKind::Seismic | LoadCaseKind::Wind)
        && lc.nodal.is_empty()
        && lc.member.is_empty()
}

/// 解析スレッドの panic（`catch_unwind` の戻り値）を利用者向けエラーメッセージへ
/// 変換する。`panic!`・`assert` 系のメッセージ（`String`／`&str` ペイロード）が
/// 取れる場合は本文へ含める。定型文だけでは利用者が原因（入力不備か
/// プログラム不具合か）にたどり着けず、不具合報告からの診断もできないため。
fn analysis_panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    let detail = payload
        .downcast_ref::<String>()
        .map(String::as_str)
        .or_else(|| payload.downcast_ref::<&str>().copied());
    match detail {
        Some(d) => {
            format!("解析スレッドが異常終了しました（プログラムの不具合の可能性があります）: {d}")
        }
        None => {
            "解析スレッドが異常終了しました（プログラムの不具合の可能性があります）。".to_string()
        }
    }
}

impl App {
    /// エラーを `last_error`（ステータスバー表示）とログの両方へ反映する。
    /// エラーはユーザーが気づかないまま埋もれると解析結果を誤って信頼しかねない
    /// ため、GUI ではログパネルを自動的に開く。
    pub fn report_error(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        self.log.push(LogLevel::Error, msg.clone());
        self.last_error = Some(msg);
        #[cfg(feature = "gui")]
        {
            // 別タブ（診断・テーブル）が表示中でもエラー本文が見えるよう、
            // ドックを開くだけでなくログタブへ切り替える。
            self.bottom_dock_open = true;
            self.bottom_tab = BottomTab::Log;
        }
    }

    /// エラーではないが利用者に知らせたい注意事項を `last_notice` とログの
    /// 両方へ反映する。処理は継続してよい事項のため、エラーと異なりログパネルの
    /// 自動オープンはしない。
    pub fn report_notice(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        self.log.push(LogLevel::Notice, msg.clone());
        self.last_notice = Some(msg);
    }

    /// ログのみに残す情報（ジョブの開始・完了など）。
    pub fn report_info(&mut self, msg: impl Into<String>) {
        self.log.push(LogLevel::Info, msg.into());
    }

    /// 荷重組合せの自動生成に固有のエラー。ステータスバー・ログ
    /// （[`Self::report_error`]）に加え、荷重組合せ欄にだけ出す専用スロット
    /// `combo_error` へも反映する（`last_error` は共用の単一スロットのため、
    /// 組合せ欄へそのまま出すと他の操作のエラーが無関係な欄に現れる）。
    pub fn report_combo_error(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        self.combo_error = Some(msg.clone());
        self.report_error(msg);
    }

    /// 柱位置から通り芯を自動生成してモデルへ反映する（モデルタブ「通り芯」の操作）。
    ///
    /// 通り芯は構造計算に用いないため、**準備計算の一部ではなく利用者が明示的に
    /// 実行する操作**とし、解析結果・設計結果も陳腐化させない
    /// （[`Staleness::mark_non_calc_edited`](crate::app::Staleness::mark_non_calc_edited)）。
    /// 生成規則は [`squid_n_core::axis_gen`] を参照。手動作成・ST-Bridge 取り込み・
    /// 利用者が改名した通り（`AxisSource::Manual`）は保護される。
    pub fn generate_axes_action(&mut self) {
        use squid_n_core::model::AxisSource;
        self.last_error = None;
        self.last_notice = None;
        let axes = squid_n_core::axis_gen::generate_axes(&self.model);
        if axes == self.model.axes {
            self.report_notice("通り芯は既に最新です（柱位置から作られる通りに変更はありません）");
            return;
        }
        let n_auto = axes
            .iter()
            .flat_map(|g| &g.axes)
            .filter(|a| a.source == AxisSource::Auto)
            .count();
        let n_manual = axes
            .iter()
            .flat_map(|g| &g.axes)
            .filter(|a| a.source == AxisSource::Manual)
            .count();
        self.undo.run(
            &mut self.model,
            Box::new(squid_n_edit::ReplaceAxes { axes }),
        );
        self.staleness.mark_non_calc_edited();
        if n_auto == 0 {
            self.report_notice(
                "柱が見つからないため通り芯を生成できませんでした（自動生成の対象は柱の材端節点です）",
            );
        } else {
            self.report_notice(format!(
                "通り芯を生成しました（自動 {n_auto} 本・保持 {n_manual} 本）"
            ));
        }
    }

    /// 節点編集バッファを model.nodes に同期する。
    /// 編集中でない（フォーカス外）セルのみ model 値で更新する。
    pub fn sync_node_edit(&mut self) {
        self.node_edit.resize(
            self.model.nodes.len(),
            ["0".to_string(), "0".to_string(), "0".to_string()],
        );
        for (i, node) in self.model.nodes.iter().enumerate() {
            for (k, slot) in self.node_edit[i].iter_mut().enumerate().take(3) {
                *slot = format!("{:.3}", node.coord[k]);
            }
        }
    }

    /// 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1「剛域」は
    /// 標準実装。解析前に1回適用する）。`squid_n_element::beam::apply_auto_rigid_zones`
    /// は `ZoneSource::Auto` の端のみ更新し `Manual` 端を保護するため、
    /// 各解析エントリの先頭で毎回呼んでも冪等で安全。
    fn apply_rigid_zones_for_analysis(&mut self) {
        // 剛域・仕口パネルの算定規則は MCP サーバと共有する
        // （`squid_n_job::prepare`。前処理が食い違うと同じモデルでも剛性が変わる）。
        self.generated_panels = squid_n_job::prepare::apply_rigid_zones_and_panels(&mut self.model);
    }

    /// `analysis_cfg.threads` を並列度設定（プロセスグローバル）へ反映する。
    /// 各解析エントリの先頭で呼ぶ（バックグラウンドジョブは thread::spawn 前に
    /// 呼べばよい。設定はプロセスグローバルのためジョブ側での再設定は不要）。
    pub(crate) fn apply_parallelism_setting(&self) {
        squid_n_math::parallelism::set_parallelism(
            squid_n_math::parallelism::Parallelism::from_threads(self.analysis_cfg.threads),
        );
    }

    /// 全解析エントリ共通の前処理（同期実行・バックグラウンドジョブ共通）。
    /// 並列度設定 → エラー/通知クリア → 準備計算（剛域・仕口パネルの反映と
    /// 荷重同期。冪等・ハッシュ判定でスキップされる）。
    ///
    /// 解析の入口は必ず本メソッドを通ること。かつては経路ごとに前処理を
    /// 個別に書いており、「増分解析・時刻歴・固有値だけ準備計算を通らず、
    /// 仕口パネルの生成が省かれて静的解析と異なるモデルを解く」という
    /// 抜けが実際に起きていた。
    fn begin_analysis(&mut self) {
        self.apply_parallelism_setting();
        self.last_error = None;
        self.last_notice = None;
        self.ensure_preparation();
    }

    /// バックグラウンドジョブ共通の入口ガード＋前処理。
    /// ジョブ実行中なら案内を出して `false` を返す（呼び出し側は即 return）。
    fn begin_analysis_job(&mut self) -> bool {
        if self.job.is_some() {
            self.report_error("解析実行中です");
            return false;
        }
        self.begin_analysis();
        true
    }

    /// パニックを解析エラーへ変換して計算を実行する（ジョブスレッド用）。
    fn run_compute<T>(f: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
            .unwrap_or_else(|p| Err(analysis_panic_message(p.as_ref())))
    }

    /// 計算クロージャをバックグラウンドスレッドで起動し、ジョブとして登録する
    /// （起動ログ込み）。結果タブへの自動遷移（`jump_on_success`）が必要な場合は
    /// 呼び出し側が登録後に `self.job` へ設定する。
    fn spawn_analysis_job(
        &mut self,
        label: &'static str,
        work: impl FnOnce() -> JobResult + Send + 'static,
    ) {
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(work());
        });
        self.job = Some(AnalysisJob {
            label,
            started: std::time::SystemTime::now(),
            rx,
            #[cfg(feature = "gui")]
            jump_on_success: None,
        });
        self.report_info(format!("⏳ {label} を開始"));
    }

    /// T3: 線形静的解析を実行し、結果を `self.results` に格納する。
    /// 指定した荷重ケースが存在しない場合はエラーメッセージをセット。
    ///
    /// 解析に先立って準備計算（`ensure_preparation`）を実行する。剛域を反映し、
    /// スラブ荷重・躯体自重を「DL」等の標準ケースへ（レビュー §1.1・照合レビュー：
    /// ③梁自重・②壁荷重の CMoQ 経路を長期応力解析へ接続）、階が定義済みなら
    /// 地震荷重を「EX」「EY」ケースへ同期する（モデル・関連設定が前回同期時から
    /// 変わっていなければ荷重の再計算は丸ごとスキップする）。
    pub fn run_linear_static(&mut self, lc: LoadCaseId) {
        self.begin_analysis();
        let res = squid_n_job::compute::compute_linear_static(self.model.clone(), lc)
            .map_err(|e| e.to_string());
        self.apply_static_case_result(StaticCaseKey::User(lc), res);
    }

    /// `compute_linear_static`/`compute_seismic`/`compute_wind` に共通の結果適用
    /// （`StaticCaseKey` で区別される単一荷重ケースの静的解析結果）。
    /// bundle への格納・last_static 設定・staleness.mark_fresh・design_check の
    /// 実行はいずれも `run_linear_static`/`run_seismic`/`run_wind` で同一のため、
    /// ここへ集約し同期版・バックグラウンドジョブ双方から使う。
    fn apply_static_case_result(
        &mut self,
        key: StaticCaseKey,
        res: Result<squid_n_solver::linear::StaticOnce, String>,
    ) {
        match res {
            Ok(res) => {
                let member_forces = res.member_forces.clone();
                let panel_moments = res.panel_moments.clone();
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.statics.retain(|(id, _)| *id != key);
                bundle.statics.push((key, res));
                bundle.member_forces = member_forces;
                bundle.panel_moments = panel_moments;
                self.results = Some(bundle);
                self.last_static = Some(StaticKey::Case(key));
                // 表示対象（focus_result）も新しい結果へ切り替える。据え置くと
                // 変位図・応力図は旧結果、member_forces・断面検定は新結果という
                // 不整合な表示になる（`current_static` は focus_result を優先する）。
                self.nav.focus_result = Some(StaticKey::Case(key));
                self.staleness.mark_fresh();
                self.run_design_check();
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 準備計算が自動生成する標準ケース（EX/EY）のうち、どれに当たるかを
    /// 荷重ケース名と種別から判別する。専用の結果キー
    /// （[`StaticCaseKey::Seismic`]）を持つケースであり、
    /// 剛心の精算・保有水平耐力の判定などがその結果を参照する。
    pub(crate) fn standard_lateral_case(&self, lc: LoadCaseId) -> Option<StaticCaseKey> {
        use squid_n_core::model::{LoadCaseKind, EX_CASE_NAME, EY_CASE_NAME};
        let case = self.model.load_cases.iter().find(|c| c.id == lc)?;
        match (case.name.as_str(), case.kind) {
            (EX_CASE_NAME, LoadCaseKind::Seismic) => Some(StaticCaseKey::Seismic(SeismicDir::X)),
            (EY_CASE_NAME, LoadCaseKind::Seismic) => Some(StaticCaseKey::Seismic(SeismicDir::Y)),
            _ => None,
        }
    }

    /// 荷重ケース 1 つの静的解析をバックグラウンドで実行する（解析パネルの
    /// 「荷重ケース」実行ボタンの入口）。
    ///
    /// 標準の水平力ケース（EX/EY）は、Ai 分布の算定諸元
    /// （`analysis_cfg`）から水平力を組み立て直して解き、結果を方向別の
    /// `StaticCaseKey::Seismic` へ格納する（剛心の精算・保有水平耐力の
    /// 判定がこのキーを参照するため）。それ以外は線形静的解析として
    /// `StaticCaseKey::User` へ格納する。
    pub fn start_load_case_job(&mut self, lc: LoadCaseId) {
        match self.standard_lateral_case(lc) {
            Some(StaticCaseKey::Seismic(dir)) => self.start_seismic_job(dir),
            _ => self.start_linear_static_job(lc),
        }
    }

    /// 線形静的解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_linear_static_job(&mut self, lc: LoadCaseId) {
        if !self.begin_analysis_job() {
            return;
        }
        let model = self.model.clone();
        self.spawn_analysis_job("線形静的解析", move || JobResult::StaticCase {
            key: StaticCaseKey::User(lc),
            res: Self::run_compute(|| {
                squid_n_job::compute::compute_linear_static(model, lc).map_err(|e| e.to_string())
            }),
        });
    }

    /// 静的解析の単体実行（解析パネル「▶ 単体実行」の入口）をバックグラウンドで
    /// 実行する。
    ///
    /// 荷重ケース単体・荷重組合せのどちらも同じ導線で実行する。求解の最小単位は
    /// 荷重ケースであり、荷重組合せは参照する荷重ケースを解いてからその線形和として
    /// 組み立てる（重ね合わせの原理。`Analysis::linear_combination`）。
    pub fn start_static_target_job(&mut self, target: StaticTarget) {
        match target {
            StaticTarget::Case(lc) => self.start_load_case_job(lc),
            StaticTarget::Combo(index) => self.start_combination_job(index),
        }
    }

    /// [`Self::start_static_target_job`] の同期版（解き終わるまで戻らない）。
    /// 振り分け先は同じで、標準の水平力ケース（EX/EY）は方向別の結果キーへ
    /// 格納する（`start_load_case_job` と同じ規約）。
    pub fn run_static_target(&mut self, target: StaticTarget) {
        match target {
            StaticTarget::Case(lc) => match self.standard_lateral_case(lc) {
                Some(StaticCaseKey::Seismic(dir)) => self.run_seismic(dir),
                _ => self.run_linear_static(lc),
            },
            StaticTarget::Combo(index) => self.run_combination(index),
        }
    }

    /// T7: 荷重組合せ解析を実行し、結果を `bundle.combos` に格納する。
    /// 指定インデックスの荷重組合せが存在しない場合はエラーメッセージをセット。
    ///
    /// 求解は参照する荷重ケース単体で行い、組合せの結果はその線形和として
    /// 組み立てる（`Analysis::linear_combination`）。
    ///
    /// 解析に先立って準備計算（`ensure_preparation`）を実行し、スラブ荷重・躯体
    /// 自重を「DL」等の標準ケースへ、階が定義済みなら地震荷重を「EX」「EY」
    /// ケースへ同期する（レビュー §1.1・照合レビュー）。
    /// 組合せが空の地震荷重ケースを参照している場合は解かずにエラーで案内する
    /// （地震項が黙って 0 になるのを防ぐ）。
    pub fn run_combination(&mut self, index: usize) {
        self.begin_analysis();
        let Some(combo) = self.model.combinations.get(index).cloned() else {
            self.report_error(format!("荷重組合せ #{} が存在しません", index));
            return;
        };
        if let Some(name) = self.empty_lateral_case_in_combo(&combo) {
            self.report_error(format!(
                "荷重組合せ「{}」が参照する水平力の荷重ケース「{}」が空です。解析タブの「準備計算 実行」を行って地震力・風圧力を生成してください。",
                combo.name, name
            ));
            return;
        }
        let name = combo.name.clone();
        let res = Self::compute_combination(self.model.clone(), combo);
        self.apply_combo_result(name, res);
    }

    /// 荷重組合せ解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_combination_job`）からも呼び出せる。
    /// `Analysis::linear_combination` は参照する荷重ケースを単体で解いてから
    /// その結果を線形和する（荷重ベクトルを合成して解き直すことはしない）。
    fn compute_combination(
        model: squid_n_core::model::Model,
        combo: squid_n_core::model::LoadCombination,
    ) -> Result<squid_n_solver::linear::StaticOnce, String> {
        match Analysis::prepare(&model) {
            Ok(analysis) => analysis
                .linear_combination(&combo)
                .map_err(|e| format!("荷重組合せ解析エラー: {:?}", e)),
            Err(e) => Err(format!("解析準備エラー: {:?}", e)),
        }
    }

    /// `compute_combination` の結果を適用する（bundle.combos への格納・
    /// last_static 設定・design_term 自動判定・design_check の実行）。
    /// `name` は組合せ名（`bundle.combos` 内の名前一致検索・再実行時の位置差替に
    /// 使う。`run_combination`/`start_combination_job` 双方から使う）。
    fn apply_combo_result(
        &mut self,
        name: String,
        res: Result<squid_n_solver::linear::StaticOnce, String>,
    ) {
        match res {
            Ok(res) => {
                let member_forces = res.member_forces.clone();
                let panel_moments = res.panel_moments.clone();
                let mut bundle = self.results.take().unwrap_or_default();
                // StaticKey::Combo は bundle.combos 上の位置を指す規約
                // （current_static・ナビゲータと共有）。再実行時は既存位置を
                // その場で差し替え、他の組合せ結果のキーを無効化しない。
                let pos = match bundle.combos.iter().position(|(n, _)| *n == name) {
                    Some(pos) => {
                        bundle.combos[pos].1 = res;
                        pos
                    }
                    None => {
                        bundle.combos.push((name.clone(), res));
                        bundle.combos.len() - 1
                    }
                };
                bundle.member_forces = member_forces;
                bundle.panel_moments = panel_moments;
                self.results = Some(bundle);
                self.last_static = Some(StaticKey::Combo(pos));
                // 表示対象も新しい結果へ（`apply_static_case_result` と同じ理由）。
                self.nav.focus_result = Some(StaticKey::Combo(pos));
                self.staleness.mark_fresh();
                // 荷重継続性区分（長期/短期）は組合せ内容から自動判定する
                // （令82条の荷重組合せ: G+P=長期、地震・積雪・風入り=短期）。
                self.design_term = if squid_n_load::combo::is_short_term_combo(&name) {
                    LoadTerm::Short
                } else {
                    LoadTerm::Long
                };
                self.run_design_check();
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 荷重組合せ解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_combination_job(&mut self, index: usize) {
        if !self.begin_analysis_job() {
            return;
        }
        let Some(combo) = self.model.combinations.get(index).cloned() else {
            self.report_error(format!("荷重組合せ #{} が存在しません", index));
            return;
        };
        if let Some(name) = self.empty_lateral_case_in_combo(&combo) {
            self.report_error(format!(
                "荷重組合せ「{}」が参照する水平力の荷重ケース「{}」が空です。解析タブの「準備計算 実行」を行って地震力・風圧力を生成してください。",
                combo.name, name
            ));
            return;
        }
        let model = self.model.clone();
        let name = combo.name.clone();
        self.spawn_analysis_job("荷重組合せ解析", move || JobResult::Combo {
            name,
            res: Self::run_compute(|| Self::compute_combination(model, combo)),
        });
    }

    /// 一括解析（全荷重ケース単体＋全荷重組合せ）を実行し、結果を `bundle` へ
    /// 格納する（解析パネル「▶▶ 一括解析」の入口）。
    ///
    /// 求解は荷重ケース単体のみで行い（`Analysis::prepare` を 1 回だけ行い、
    /// `analysis_cfg.threads` の並列設定に応じて荷重ケース単位に並列解析する）、
    /// 荷重組合せはその結果の線形和として組み立てる（重ね合わせの原理。
    /// `Analysis::linear_static_with_combinations`）。同じ荷重ケースを参照する組合せが
    /// 何件あっても、求解は荷重ケース数ぶんで済む。
    ///
    /// 個別の解析エラーは処理を止めず、件数と最初のエラー内容を `last_error` に
    /// まとめる（他の結果は失わない）。荷重ケースが 1 件もない場合、および 1 件も
    /// 解けなかった場合は既存の結果を変更せず、案内メッセージを `last_error` に
    /// 設定して return する。
    pub fn run_static_all(&mut self) {
        self.begin_analysis();
        if self.model.load_cases.is_empty() {
            self.report_error("荷重ケースがありません。荷重タブで作成してください。");
            return;
        }
        let (case_keys, combos, errors) = self.static_all_inputs();
        let computed = Self::compute_static_all(self.model.clone(), case_keys, combos);
        self.apply_static_all_result(computed, errors);
    }

    /// `run_static_all`/`start_static_all_job` 共通の事前準備。UI スレッド側の
    /// `self.model` を参照するため、バックグラウンドジョブでもここで行う。
    ///
    /// - 荷重ケース: 結果の格納キー（標準の水平力ケースは方向別の
    ///   `StaticCaseKey::Seismic`/`Wind`、それ以外は `User`）を対応付ける。
    ///   空の水平力ケース（未生成の EX/EY 等）は解析対象から外す（水平力が黙って
    ///   0 の結果を方向別キーへ格納すると、剛心の精算・保有水平耐力の判定が
    ///   それを正しい地震時応力として扱ってしまうため）。
    /// - 荷重組合せ: 空の水平力ケースを参照する組合せを除外する（地震・風の項が
    ///   黙って 0 になるのを防ぐ）。
    ///
    /// 戻り値は (荷重ケースと格納キーの対応, 解析対象の組合せ, エラー文一覧)。
    #[allow(clippy::type_complexity)]
    fn static_all_inputs(
        &self,
    ) -> (
        Vec<(LoadCaseId, StaticCaseKey)>,
        Vec<squid_n_core::model::LoadCombination>,
        Vec<String>,
    ) {
        let mut errors: Vec<String> = Vec::new();
        let case_keys = self
            .model
            .load_cases
            .iter()
            .filter(|lc| {
                if is_empty_lateral_case(lc) {
                    errors.push(format!(
                        "[{}] 水平力の荷重ケースが空です。「準備計算 実行」を行って地震力・風圧力を生成してください。",
                        lc.name
                    ));
                    return false;
                }
                true
            })
            .map(|lc| {
                let key = self
                    .standard_lateral_case(lc.id)
                    .unwrap_or(StaticCaseKey::User(lc.id));
                (lc.id, key)
            })
            .collect();
        let combos = self
            .model
            .combinations
            .iter()
            .filter(|combo| match self.empty_lateral_case_in_combo(combo) {
                Some(name) => {
                    errors.push(format!(
                        "[{}] 水平力の荷重ケース「{}」が空です。「準備計算 実行」を行ってください。",
                        combo.name, name
                    ));
                    false
                }
                None => true,
            })
            .cloned()
            .collect();
        (case_keys, combos, errors)
    }

    /// 一括解析の純粋計算部分。所有権を取り `&self` を使わないため、
    /// バックグラウンドジョブ（`start_static_all_job`）からも呼び出せる。
    ///
    /// `Analysis::prepare` を 1 回だけ行い、`case_keys` の荷重ケースを単体で解いて
    /// （荷重ケース単位の並列）、`combos` をその結果の線形和として組み立てる。
    /// `Analysis::prepare` 自体が失敗した場合は `Err` で全体を中断する
    /// （既存結果は `apply_static_all_result` 側で変更しない）。
    fn compute_static_all(
        model: squid_n_core::model::Model,
        case_keys: Vec<(LoadCaseId, StaticCaseKey)>,
        combos: Vec<squid_n_core::model::LoadCombination>,
    ) -> Result<StaticAllComputed, String> {
        let analysis = Analysis::prepare(&model).map_err(|e| format!("解析準備エラー: {:?}", e))?;
        let ids: Vec<LoadCaseId> = case_keys.iter().map(|(id, _)| *id).collect();
        let batch = analysis.linear_static_with_combinations(&ids, &combos);
        let case_name = |id: LoadCaseId| {
            model
                .load_cases
                .iter()
                .find(|c| c.id == id)
                .map(|c| c.name.clone())
                .unwrap_or_else(|| format!("#{}", id.0))
        };
        let cases = case_keys
            .iter()
            .zip(batch.cases)
            .map(|((id, key), res)| {
                (
                    *key,
                    res.map_err(|e| format!("[{}] {:?}", case_name(*id), e)),
                )
            })
            .collect();
        let combos = combos
            .iter()
            .zip(batch.combos)
            .map(|(combo, res)| {
                (
                    combo.name.clone(),
                    res.map_err(|e| format!("[{}] {:?}", combo.name, e)),
                )
            })
            .collect();
        Ok(StaticAllComputed { cases, combos })
    }

    /// `compute_static_all` の結果を適用する。個別の解析エラーは処理を止めず、
    /// 件数と最初のエラー内容を `last_error` にまとめる（他の結果は失わない）。
    /// `pre_errors`（事前フィルタで除外された荷重ケース・組合せのエラー）と合わせて
    /// 1 件も解けなかった場合、および `Analysis::prepare` 自体が失敗した場合は
    /// 既存の結果を変更せず、案内メッセージを `last_error` に設定して return する。
    ///
    /// 表示対象（`last_static`）は最後に成功した荷重組合せ、組合せが 1 件もなければ
    /// 最後に成功した荷重ケースとする。
    fn apply_static_all_result(
        &mut self,
        computed: Result<StaticAllComputed, String>,
        mut errors: Vec<String>,
    ) {
        let items = match computed {
            Ok(items) => items,
            Err(e) => {
                self.report_error(e);
                return;
            }
        };

        let had_results = self.results.is_some();
        let mut bundle = self.results.take().unwrap_or_default();
        let mut last_case: Option<StaticCaseKey> = None;
        for (key, res) in items.cases {
            match res {
                Ok(res) => {
                    bundle.statics.retain(|(k, _)| *k != key);
                    bundle.statics.push((key, res));
                    last_case = Some(key);
                }
                Err(e) => errors.push(e),
            }
        }
        let mut last_combo: Option<(usize, String)> = None;
        for (name, res) in items.combos {
            match res {
                Ok(res) => {
                    // StaticKey::Combo は bundle.combos 上の位置を指す規約
                    // （run_combination と同じ「名前一致なら置換、なければ push」）。
                    let pos = match bundle.combos.iter().position(|(n, _)| *n == name) {
                        Some(pos) => {
                            bundle.combos[pos].1 = res;
                            pos
                        }
                        None => {
                            bundle.combos.push((name.clone(), res));
                            bundle.combos.len() - 1
                        }
                    };
                    last_combo = Some((pos, name));
                }
                Err(e) => errors.push(e),
            }
        }

        let display = match &last_combo {
            Some((pos, _)) => Some(StaticKey::Combo(*pos)),
            None => last_case.map(StaticKey::Case),
        };
        let Some(display) = display else {
            // 1件も解けなかった場合は既存の結果を壊さない（取り出した結果を戻す）。
            if had_results {
                self.results = Some(bundle);
            }
            self.report_error(format!(
                "一括解析エラー（{} 件すべて失敗）: {}",
                errors.len(),
                errors.first().cloned().unwrap_or_default()
            ));
            return;
        };
        // 応力図・断面検定が参照する member_forces は表示対象の結果へ合わせる
        // （`select_displayed_result` と同じ規約）。
        let displayed = match display {
            StaticKey::Combo(pos) => bundle
                .combos
                .get(pos)
                .map(|(_, s)| (s.member_forces.clone(), s.panel_moments.clone())),
            StaticKey::Case(key) => bundle
                .statics
                .iter()
                .find(|(k, _)| *k == key)
                .map(|(_, s)| (s.member_forces.clone(), s.panel_moments.clone())),
        };
        if let Some((member_forces, panel_moments)) = displayed {
            bundle.member_forces = member_forces;
            bundle.panel_moments = panel_moments;
        }
        self.results = Some(bundle);
        self.last_static = Some(display);
        // 表示対象も新しい結果へ（`apply_static_case_result` と同じ理由）。
        self.nav.focus_result = Some(display);
        self.staleness.mark_fresh();
        // 荷重継続性区分（長期/短期）は表示対象の組合せ名から自動判定する
        // （令82条の荷重組合せ: G+P=長期、地震・積雪・風入り=短期）。荷重ケース単体を
        // 表示対象にした場合は現在の区分を維持する（`apply_static_case_result` と同じ）。
        if let Some((_, name)) = &last_combo {
            self.design_term = if squid_n_load::combo::is_short_term_combo(name) {
                LoadTerm::Short
            } else {
                LoadTerm::Long
            };
        }
        self.run_design_check();

        if !errors.is_empty() {
            self.report_error(format!("{} 件でエラー: {}", errors.len(), errors[0]));
        }
    }

    /// 一括解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_static_all_job(&mut self) {
        if !self.begin_analysis_job() {
            return;
        }
        if self.model.load_cases.is_empty() {
            self.report_error("荷重ケースがありません。荷重タブで作成してください。");
            return;
        }
        let (case_keys, combos, pre_errors) = self.static_all_inputs();
        let model = self.model.clone();
        self.spawn_analysis_job("一括解析", move || JobResult::StaticAll {
            computed: Self::run_compute(|| Self::compute_static_all(model, case_keys, combos)),
            pre_errors,
        });
    }

    /// 表示対象の静的解析結果を解決する。優先順: ナビゲータ選択 → 最後に実行した結果。
    pub fn current_static(&self) -> Option<&squid_n_solver::linear::StaticOnce> {
        let bundle = self.results.as_ref()?;
        let resolve = |key: StaticKey| -> Option<&squid_n_solver::linear::StaticOnce> {
            match key {
                StaticKey::Case(case_key) => bundle
                    .statics
                    .iter()
                    .find(|(k, _)| *k == case_key)
                    .map(|(_, s)| s),
                StaticKey::Combo(idx) => bundle.combos.get(idx).map(|(_, s)| s),
            }
        };
        self.nav
            .focus_result
            .and_then(resolve)
            .or_else(|| self.last_static.and_then(resolve))
    }

    /// 結果表示の対象を切り替える（ナビゲータ・結果タブの選択ドロップダウン共通）。
    ///
    /// 変位図・層指標だけでなく、応力図（N/Q/M）・断面検定が参照する
    /// [`ResultsBundle::member_forces`] も選択結果へ差し替える。荷重組合せを選んだ
    /// 場合は荷重継続性区分（長期/短期）を組合せ名から `is_short_term_combo` で
    /// 再判定し、断面検定を再実行する。これにより、選んだ荷重（組合せ）の長期/短期に
    /// 応じた断面算定結果が表示される。単一荷重ケースを選んだ場合は現在の区分を維持する
    /// （`apply_static_case_result` と同じ扱い）。該当キーの解析結果がない場合は何もしない。
    pub fn select_displayed_result(&mut self, key: StaticKey) {
        // 選択キーに対応する解析結果（内力と、組合せなら名前）を取り出す。
        let resolved = self.results.as_ref().and_then(|bundle| match key {
            StaticKey::Case(case_key) => bundle
                .statics
                .iter()
                .find(|(k, _)| *k == case_key)
                .map(|(_, s)| (s.member_forces.clone(), s.panel_moments.clone(), None)),
            StaticKey::Combo(idx) => bundle.combos.get(idx).map(|(name, s)| {
                (
                    s.member_forces.clone(),
                    s.panel_moments.clone(),
                    Some(name.clone()),
                )
            }),
        });
        let Some((member_forces, panel_moments, combo_name)) = resolved else {
            return;
        };
        self.nav.focus_result = Some(key);
        self.last_static = Some(key);
        if let Some(bundle) = self.results.as_mut() {
            bundle.member_forces = member_forces;
            bundle.panel_moments = panel_moments;
        }
        // 組合せは名前から長期/短期を再判定する（単一ケースは現在の区分を維持）。
        if let Some(name) = combo_name {
            self.design_term = if squid_n_load::combo::is_short_term_combo(&name) {
                LoadTerm::Short
            } else {
                LoadTerm::Long
            };
        }
        self.run_design_check();
    }

    /// T3: 固有値解析を実行し、結果を `self.results` に格納する（同期）。
    pub fn run_eigen(&mut self, n_modes: usize) {
        self.begin_analysis();
        let res = squid_n_job::compute::compute_eigen(self.model.clone(), n_modes)
            .map_err(|e| e.to_string());
        self.apply_eigen_result(res);
    }

    /// 固有値解析をバックグラウンドスレッドで実行する（解析パネル「▶ 実行」の
    /// 入口）。かつて固有値だけは UI スレッドで同期実行しており、モード数の
    /// 多い固有値解析中にアプリが無応答になっていた。
    pub fn start_eigen_job(&mut self, n_modes: usize) {
        if !self.begin_analysis_job() {
            return;
        }
        let model = self.model.clone();
        self.spawn_analysis_job("固有値解析", move || {
            JobResult::Modal(Self::run_compute(|| {
                squid_n_job::compute::compute_eigen(model, n_modes).map_err(|e| e.to_string())
            }))
        });
    }

    /// `compute_eigen` の結果を適用する（bundle 格納・最終実行時刻更新）。
    fn apply_eigen_result(&mut self, res: Result<squid_n_solver::eigen::ModalResult, String>) {
        match res {
            Ok(modal) => {
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.modal = Some(modal);
                self.results = Some(bundle);
                // 固有値のみの更新では設計は更新されないが、最新実行時刻は更新
                self.staleness.last_run = Some(SystemTime::now());
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 階(Story)を節点標高から自動生成して適用する（undo 可能）。準備計算
    /// （[`Self::run_preparation`]）の一工程であり、単独の UI 操作ではない。
    ///
    /// 地震重量には kind=Dead/LiveSeismic（なければ Dead+Live、種別未設定なら
    /// 先頭ケース）の荷重ケースの鉛直下向き荷重を用いる（レビュー §1.7）。
    /// 先立ってスラブ荷重・躯体自重を「DL」等の標準ケースへ同期する
    /// （レビュー §1.1）ため、面荷重・自重も地震用重量に反映される
    /// （DL に自重が含まれるため、密度からの自重直接算入は DL がない場合のみ。
    /// `density_self_weight_for_stories`）。主要構造種別は各階の柱・梁の断面形状
    /// から自動判定される（`story_gen`）。
    ///
    /// 階そのもの（階名・階レベル・階種別・地震用重量の手入力）は利用者が定義する
    /// データであり、再生成では書き換えない（`story_gen` が既存の階定義から
    /// そのまま引き継ぐ）。ここで更新されるのは所属節点・剛床・算定重量である。
    ///
    /// 階の適用後、地震荷重を「EX」「EY」ケースへ同期する
    /// （Ai 分布の水平力。これで荷重組合せ G+P±K が実行可能になる）。
    ///
    /// 生成結果が現在のモデルと一致する場合は `ApplyStories` を発行しない
    /// （冪等。準備計算は実行のたびに階を作り直すため、モデルが変わっていないのに
    /// undo 履歴を積んだり `mark_edited` で解析結果を stale にしたりしないようにする。
    /// `sync_one_auto_case` と同じ規約）。
    pub fn generate_stories_action(&mut self) {
        self.last_error = None;
        self.last_notice = None;
        // 柱フェース距離（`RigidZone::face_i/face_j`）の算定は自重の同期より先に
        // 行う。RC/SRC 梁の自重は柱面間の内法長で算定するため
        // （`squid_n_load::story_gen::self_weight_calc`）、face が未算定（0）の
        // まま同期すると節点間距離で算定した過大な自重が DL に入る。以前は同期の
        // 後に算定していたため、1 回目の準備計算だけ DL が過大になり、2 回目の
        // 実行で初めて正しい値へ変わっていた（＝準備計算が冪等でなかった）。
        //
        // face は接合関係と断面せいから決まる幾何量で、剛域長 `length_i/j` の
        // Manual/Auto とは独立に常に再算定される（`recompute_auto_zones`）。
        // ここで呼ぶ `apply_auto_rigid_zones` が剛域長と同時に face も算定する
        // ため、名前に反して自重の前提でもある。算定は部材の幾何と断面のみに
        // 依存し階の生成結果には依存しないため、先に呼んで差し支えない。
        self.apply_rigid_zones_for_analysis();
        self.sync_gravity_load_cases_action();
        let gravity_lcs = gravity_cases_for_seismic_weight(&self.model);
        let include_density = density_self_weight_for_stories(&self.model);
        let mass_method = self.analysis_cfg.mass_method;
        match squid_n_load::story_gen::generate_stories_with_opts(
            &self.model,
            &gravity_lcs,
            include_density,
            mass_method,
        ) {
            Ok(gen) => {
                if !story_gen_changes_model(&self.model, &gen, mass_method) {
                    // 階は既に最新。荷重の同期だけ冪等に確認して終える。
                    self.apply_rigid_zones_for_analysis();
                    self.sync_seismic_load_cases_action();
                    self.auto_load_sync_hash = Some(self.compute_auto_load_sync_hash());
                    return;
                }
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::ApplyStories {
                        stories: gen.stories,
                        node_story: gen.node_story,
                        constraints: gen.constraints,
                        rep_nodes: gen.rep_nodes,
                        generated_masters: gen.generated_masters,
                        mass_method,
                    }),
                );
                self.staleness.mark_edited();
                // 剛域の反映は地震荷重の同期より先に行う（SemiPrecise の固有周期算定が
                // 剛域込みの剛性を用いるようにするため）。
                self.apply_rigid_zones_for_analysis();
                self.sync_seismic_load_cases_action();
                // 直後に run_linear_static 等（`sync_auto_load_cases_action`）が
                // 呼ばれても、いま行った DL/LL/EX/EY の同期を無駄に繰り返さない
                // よう、同期後の状態のハッシュを記録しておく。
                self.auto_load_sync_hash = Some(self.compute_auto_load_sync_hash());
            }
            Err(e) => self.report_error(format!("階の生成エラー: {}", e)),
        }
    }

    /// T3: 地震静的解析（Ai一気通貫）を実行し、結果を `self.results` に格納する。
    /// 方向・Ai算定法・Z・地盤種別・C0 は `analysis_cfg` を用いる。
    /// 結果は `StaticCaseKey::Seismic(dir)` に格納するため、X/Y 双方の地震静的結果
    /// および任意のユーザー荷重ケースの結果と衝突せず共存できる。
    /// あわせて同じ水平力を「EX」「EY」ケースへ同期する（荷重組合せ用。
    /// 準備計算 `ensure_preparation` が行う）。
    ///
    /// 設計用固有周期 T は `design_seismic_period` で暗黙の解析なしに決定する
    /// （内部で固有値解析を実行しない `Analysis::seismic_static_with_period` を
    /// 使う）。SemiPrecise で固有値解析が未実行の場合は解析せず、実行を促す
    /// メッセージを `last_error` に設定して return する。
    pub fn run_seismic(&mut self, dir: SeismicDir) {
        self.begin_analysis();
        let t = match self.design_seismic_period() {
            Ok(t) => t,
            Err(msg) => {
                self.report_error(msg);
                return;
            }
        };
        let cfg = squid_n_solver::analysis::SeismicCfg {
            dir,
            mode: self.analysis_cfg.ai_mode,
            z: self.analysis_cfg.z,
            soil: self.analysis_cfg.soil,
            c0: self.analysis_cfg.c0,
        };
        let res = squid_n_job::compute::compute_seismic(self.model.clone(), cfg, t)
            .map_err(|e| e.to_string());
        self.apply_static_case_result(StaticCaseKey::Seismic(dir), res);
    }

    /// 地震静的解析をバックグラウンドスレッドで実行する（P8 §5）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_seismic_job(&mut self, dir: SeismicDir) {
        if !self.begin_analysis_job() {
            return;
        }
        let t = match self.design_seismic_period() {
            Ok(t) => t,
            Err(msg) => {
                self.report_error(msg);
                return;
            }
        };
        let cfg = squid_n_solver::analysis::SeismicCfg {
            dir,
            mode: self.analysis_cfg.ai_mode,
            z: self.analysis_cfg.z,
            soil: self.analysis_cfg.soil,
            c0: self.analysis_cfg.c0,
        };
        let model = self.model.clone();
        self.spawn_analysis_job("地震静的解析", move || JobResult::StaticCase {
            key: StaticCaseKey::Seismic(dir),
            res: Self::run_compute(|| {
                squid_n_job::compute::compute_seismic(model, cfg, t).map_err(|e| e.to_string())
            }),
        });
    }

    /// 荷重ケースから標準組合せを生成し、undo 可能に一括追加する
    /// （`squid_n_load::combo::standard_combinations`・`AddCombination` を使用）。
    ///
    /// 固定（Dead）・積載（Live）・積雪（Snow）は種別の先頭 1 件を用いる。
    /// 地震は、準備計算が自動生成する標準ケース名（`EX`/`EY`）で方向を判別する
    /// （種別だけでは X・Y を区別できない）。標準名のケースがなければ割り当てない
    /// （方向不明の地震ケースを機械的に EX とみなさない）。
    ///
    /// 風荷重は算定・生成の対象外のため、暴風の組合せは生成しない。
    ///
    /// Dead/Live のいずれかが見つからない場合は組合せを生成せず `last_error` を設定する。
    pub fn auto_generate_combinations_action(&mut self) {
        use squid_n_core::model::{LoadCaseKind, EX_CASE_NAME, EY_CASE_NAME};

        self.last_error = None;
        self.combo_error = None;
        let find_first = |kind: LoadCaseKind| {
            self.model
                .load_cases
                .iter()
                .find(|lc| lc.kind == kind)
                .map(|lc| lc.id)
        };
        let find_named = |name: &str, kind: LoadCaseKind| {
            self.model
                .load_cases
                .iter()
                .find(|lc| lc.name == name && lc.kind == kind)
                .map(|lc| lc.id)
        };
        let Some(dl) = find_first(LoadCaseKind::Dead) else {
            self.report_combo_error("種別「固定荷重」の荷重ケースが見つかりません");
            return;
        };
        let Some(ll) = find_first(LoadCaseKind::Live) else {
            self.report_combo_error("種別「積載荷重(架構用)」の荷重ケースが見つかりません");
            return;
        };
        let snow = find_first(LoadCaseKind::Snow);

        let input = squid_n_load::combo::ComboInput {
            dl,
            ll,
            seismic_x: find_named(EX_CASE_NAME, LoadCaseKind::Seismic),
            seismic_y: find_named(EY_CASE_NAME, LoadCaseKind::Seismic),
            snow,
            heavy_snow_zone: self.analysis_cfg.heavy_snow_zone,
            snow_factors: Some(squid_n_load::combo::SnowFactors {
                delta1: self.analysis_cfg.snow_delta1,
                delta3: self.analysis_cfg.snow_delta3,
            }),
        };
        let combos = squid_n_load::combo::standard_combinations(&input);
        for combo in combos {
            self.undo.run(
                &mut self.model,
                Box::new(squid_n_edit::AddCombination { combo }),
            );
        }
        self.staleness.mark_edited();
    }

    /// `compute_pushover` の結果を適用する（bundle 格納・最終実行時刻更新・エラー設定）。
    fn apply_pushover_result(
        &mut self,
        res: Result<squid_n_solver::pushover::PushoverResult, String>,
    ) {
        match res {
            Ok(result) => {
                // 目標未到達の打ち切り（非収束・特異化）は Qu が過小評価の可能性が
                // あるため、結果画面の警告に加えてログにも残す。
                if result.termination.is_premature() {
                    self.report_notice(format!(
                        "⚠ 増分解析は目標到達前に打ち切られました（{}）。Qu はその時点までの最大値です。",
                        result.termination.describe()
                    ));
                }
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.pushover = Some(result);
                self.results = Some(bundle);
                // mark_fresh で stale を解消する（`apply_static_case_result` と同じ扱い）。
                // last_run の更新だけでは results_stale が立ったままになり、編集後に
                // 増分解析だけを実行してもビューアが「再実行してください」表示のまま
                // 復帰しなかった。
                self.staleness.mark_fresh();
                self.last_error = None;
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 増分解析（プッシュオーバー）を実行する。モデルは複製の上で解析する
    /// （非線形状態の副作用を GUI 上のモデルへ残さないため）。
    /// 鋼板耐震壁を含むモデルの増分解析で、せん断座屈を考慮していない旨を知らせる。
    ///
    /// 鋼板耐震壁の面内せん断終局強度は鋼板のせん断降伏 Qy=t·lw·F/√3 で評価している
    /// （`squid_n_element::wall_panel::WallPanelElement::steel_shear_capacity_of`）。
    /// 幅厚比の大きい無補剛の鋼板は降伏前に面外へせん断座屈するため、その場合は
    /// 耐力を過大評価する（危険側）。解析は継続してよい事項のため注意事項として扱う。
    fn notice_steel_seismic_walls(&mut self) {
        let n = self
            .model
            .elements
            .iter()
            .filter(|e| {
                matches!(e.kind, squid_n_core::model::ElementKind::Wall)
                    && squid_n_element::misc_wall::wall_is_seismic(e, &self.model)
                    && !squid_n_element::misc_wall::is_rc_wall(e, &self.model)
            })
            .count();
        if n == 0 {
            return;
        }
        self.report_notice(format!(
            "鋼板耐震壁 {} 枚の面内せん断終局強度を、鋼板のせん断降伏 Qy=t·lw·F/√3 で評価します。\
             せん断座屈は考慮していないため、幅厚比が大きく補剛のない鋼板では耐力を過大評価します。",
            n
        ));
    }

    pub fn run_pushover(&mut self) {
        self.begin_analysis();
        self.notice_steel_seismic_walls();
        let res = squid_n_job::compute::compute_pushover(self.model.clone(), self.analysis_cfg)
            .map_err(|e| e.to_string());
        self.apply_pushover_result(res);
    }

    /// 増分解析（プッシュオーバー）をバックグラウンドスレッドで実行する（P8 §5、残課題1）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_pushover_job(&mut self) {
        if !self.begin_analysis_job() {
            return;
        }
        self.notice_steel_seismic_walls();
        let model = self.model.clone();
        let cfg = self.analysis_cfg;
        self.spawn_analysis_job("増分解析", move || {
            JobResult::Pushover(Self::run_compute(|| {
                squid_n_job::compute::compute_pushover(model, cfg).map_err(|e| e.to_string())
            }))
        });
        #[cfg(feature = "gui")]
        if let Some(job) = self.job.as_mut() {
            job.jump_on_success = Some((Tab::Results, ResultsView::Pushover));
        }
    }

    /// `compute_time_history` の結果を適用する
    /// （bundle 格納・time_history_data 更新(gui)・最終実行時刻更新・エラー設定）。
    fn apply_time_history_result(
        &mut self,
        res: Result<squid_n_solver::timehistory::ResponseResult, String>,
    ) {
        match res {
            Ok(res) => {
                #[cfg(feature = "gui")]
                {
                    self.time_history_data = crate::time_history_view::TimeHistoryData {
                        time: res.time.clone(),
                        node_disp: res.history.node_disp.clone(),
                        story_shear: res.history.base_shear.clone(),
                        story_drift_angle: res.history.top_drift_angle.clone(),
                        node: res.history.node,
                    };
                }
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.time_history = Some(res);
                self.results = Some(bundle);
                // mark_fresh で stale を解消する（`apply_pushover_result` と同じ理由。
                // last_run の更新だけでは、編集後に時刻歴だけを実行しても
                // アニメーション・部材クリック・詳細ウィンドウが stale 判定で
                // 無効化されたまま復帰しなかった）。
                self.staleness.mark_fresh();
                self.last_error = None;
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 線形時刻歴応答解析を実行する。減衰モデル・積分法は `analysis_cfg` に従う
    /// （剛性比例／Rayleigh、Newmark-β／HHT-α）。
    pub fn run_time_history(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        self.begin_analysis();
        let res =
            squid_n_job::compute::compute_time_history(self.model.clone(), self.analysis_cfg, wave)
                .map_err(|e| e.to_string());
        self.apply_time_history_result(res);
    }

    /// 時刻歴応答解析をバックグラウンドスレッドで実行する（P8 §5、残課題1）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_time_history_job(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        if !self.begin_analysis_job() {
            return;
        }
        let model = self.model.clone();
        let cfg = self.analysis_cfg;
        // 非線形／線形の別をジョブラベル・完了ログへ出す（実行中の判別・履歴の両方で有用）。
        let label = if cfg.th_nonlinear {
            "時刻歴応答(非線形)"
        } else {
            "時刻歴応答(線形)"
        };
        self.spawn_analysis_job(label, move || {
            JobResult::TimeHistory(Box::new(Self::run_compute(|| {
                squid_n_job::compute::compute_time_history(model, cfg, wave)
                    .map_err(|e| e.to_string())
            })))
        });
        #[cfg(feature = "gui")]
        if let Some(job) = self.job.as_mut() {
            job.jump_on_success = Some((Tab::Results, ResultsView::TimeHistory));
        }
    }

    /// 実行中のジョブの完了を確認し、完了していれば結果を適用する。
    /// 成功/失敗いずれかで結果を受信できた場合、またはスレッド異常終了時は
    /// `job` を `None` に戻し `true` を返す。まだ実行中なら `false` を返す。
    pub fn poll_job(&mut self) -> bool {
        let recv = match &self.job {
            Some(job) => job.rx.try_recv(),
            None => return false,
        };
        match recv {
            Ok(result) => {
                // ラベルと経過時間は完了ログ用に、jump_on_success は結果タブへの
                // 自動遷移用に、job を take する前に取り出しておく。
                let job = self.job.take();
                let (label, elapsed_secs) = job
                    .as_ref()
                    .map(|j| {
                        (
                            j.label,
                            j.started.elapsed().unwrap_or_default().as_secs_f64(),
                        )
                    })
                    .unwrap_or(("解析", 0.0));
                #[cfg(feature = "gui")]
                let jump = job.and_then(|j| j.jump_on_success);
                #[cfg(not(feature = "gui"))]
                let _ = job;

                // ジョブ完了は新しい状態の起点なので、ジョブ実行中に発生した無関係の
                // エラー表示（例: ファイル保存失敗）をここでクリアしてから結果を適用する。
                // こうしないと成功判定（下の last_error.is_none()）が古いエラーに
                // 引きずられ、成功したのに完了ログ・自動遷移が行われない
                // （エラー自体はイベントログに残っており失われない）。
                self.last_error = None;
                match result {
                    JobResult::Pushover(res) => self.apply_pushover_result(res),
                    JobResult::Modal(res) => self.apply_eigen_result(res),
                    JobResult::TimeHistory(res) => self.apply_time_history_result(*res),
                    JobResult::StaticCase { key, res } => self.apply_static_case_result(key, res),
                    JobResult::Combo { name, res } => self.apply_combo_result(name, res),
                    JobResult::StaticAll {
                        computed,
                        pre_errors,
                    } => self.apply_static_all_result(computed, pre_errors),
                }
                // 失敗時は各 apply_* が report_error 経由で last_error とログの両方
                // へ反映済みのため、ここでは成功時のみ完了ログを追加する
                // （二重ログを避ける）。
                if self.last_error.is_none() {
                    self.report_info(format!("✅ {} が完了 ({:.1}s)", label, elapsed_secs));
                    #[cfg(feature = "gui")]
                    if let Some((tab, view)) = jump {
                        self.active_tab = tab;
                        self.apply_tab_preset(tab);
                        self.results_view = view;
                    }
                }
                true
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => false,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.job = None;
                self.report_error("解析スレッドが異常終了しました（結果を受信できませんでした）。");
                true
            }
        }
    }

    /// 工程タブ切替時のドック初期配置プリセット。各タブに適した初期配置への
    /// ショートカットであり、ユーザーが後からドック開閉・パネル切替を自由に
    /// 行うことを妨げない（あくまで切替直後の便宜的な既定値）。
    #[cfg(feature = "gui")]
    pub(crate) fn apply_tab_preset(&mut self, tab: Tab) {
        match tab {
            Tab::Model => {
                self.left_dock_open = true;
                self.left_panel = LeftPanel::Navigator;
                self.bottom_dock_open = true;
                self.bottom_tab = BottomTab::Model;
                self.right_dock_open = true;
                self.right_panel = RightPanel::Inspector;
            }
            Tab::Loads => {
                self.left_dock_open = true;
                self.left_panel = LeftPanel::Navigator;
                self.bottom_dock_open = true;
                self.bottom_tab = BottomTab::Loads;
                self.right_dock_open = true;
                self.right_panel = RightPanel::Inspector;
            }
            Tab::Analysis => {
                self.right_dock_open = true;
                // 一貫計算の手順どおり ①（準備計算）から入れるようにする。
                // ② 解析へはパネル先頭の切替行、またはステータスバーの ⚙ から移る。
                self.right_panel = RightPanel::Preparation;
                self.bottom_tab = BottomTab::Log;
            }
            Tab::Results => {
                self.left_dock_open = true;
                self.left_panel = LeftPanel::Navigator;
                self.right_dock_open = true;
                self.right_panel = RightPanel::Inspector;
            }
            Tab::Design => {
                self.right_dock_open = true;
                self.right_panel = RightPanel::Inspector;
            }
            Tab::Report => {}
        }
    }

    /// 正弦減衰のサンプル地震波を `cfg` から組み立てる
    /// （外部波形ファイルなしで機能を試せる導線。同期実行・ジョブ実行の双方で使う）。
    pub(crate) fn sample_wave(cfg: &AnalysisSettings) -> squid_n_solver::timehistory::GroundMotion {
        squid_n_job::sample_ground_motion(cfg)
    }

    /// 正弦減衰のサンプル地震波を生成して時刻歴解析を実行する（同期）。
    pub fn run_time_history_sample(&mut self) {
        self.apply_parallelism_setting();
        let wave = Self::sample_wave(&self.analysis_cfg);
        self.run_time_history(wave);
    }

    /// モデル整合性チェック（診断）を実行し `self.diagnostics` を再構築する。
    /// 下ドック「診断」タブを開いたとき／「再チェック」ボタン押下時のみ呼ばれる
    /// 想定で、解析等の重い処理は行わない。
    ///
    /// 計算量は概ね O(部材数)。ただし耐震壁と周辺架構の種別照合だけは壁 1 枚ごとに
    /// 周辺部材を走査するため O(壁数 × 部材数) になる（解析前チェックが元から
    /// 払っているのと同じコスト）。これ以上重い検査を足す場合は、診断タブを開くたびに
    /// 走ることを踏まえて遅延評価の粒度から見直すこと。
    pub fn run_diagnostics(&mut self) {
        let mut diags = Vec::new();

        // 解析を妨げる不備（モデル検証・支点なし・断面/材料の未割当・シェルの板厚なし・As=0・
        // 耐震壁と周辺架構の種別食い違い・孤立節点など）。判定は解析前チェックと
        // 同じ `model_issues` を使う。診断と解析前チェックが別々に検査を持つと、
        // 片方だけに項目を足したときに「診断は通ったのに解析が止まる」状態になる。
        //
        // 重大度は `ModelIssue` が持つ判定をそのまま使う。解析が必ず止まる不備は
        // Error、解析は通るが入力の意図を確かめたい事柄（剛床のない階など）は
        // Warning になる。準備計算の `PreparationResult::is_ready`
        // （`diag_errors == 0`）が「解析前に解消すべきか」の判定にそのまま使える。
        for issue in squid_n_solver::analysis::precheck::model_issues(&self.model) {
            push_issue_diagnostics(&mut diags, issue);
        }

        // 空の水平力ケース（地震・風）を参照する荷重組合せ: そのまま解くと水平力の
        // 項が黙って 0 になるため（`empty_lateral_case_in_combo` と同じ判定を流用）。
        for combo in &self.model.combinations {
            if let Some(name) = self.empty_lateral_case_in_combo(combo) {
                diags.push(Diagnostic {
                    severity: DiagSeverity::Warning,
                    message: format!(
                        "荷重組合せ「{}」が参照する水平力の荷重ケース「{}」が空です\
                         （解析タブの「準備計算 実行」で生成できます）",
                        combo.name, name
                    ),
                    target: None,
                });
            }
        }

        // 空の荷重ケース（節点・部材荷重とも未定義）。誤って荷重を入れ忘れた
        // ケースに気づけるよう情報表示する。
        for lc in &self.model.load_cases {
            if lc.nodal.is_empty() && lc.member.is_empty() {
                diags.push(Diagnostic {
                    severity: DiagSeverity::Info,
                    message: format!("荷重ケース「{}」に荷重が定義されていません", lc.name),
                    target: None,
                });
            }
        }

        self.diagnostics = diags;
        self.staleness.diagnostics_stale = false;
    }

    /// 診断結果の件数集計（Error数, Warning数）。タブラベル・ステータス表示用。
    pub fn diagnostics_counts(&self) -> (usize, usize) {
        let errors = self
            .diagnostics
            .iter()
            .filter(|d| d.severity == DiagSeverity::Error)
            .count();
        let warnings = self
            .diagnostics
            .iter()
            .filter(|d| d.severity == DiagSeverity::Warning)
            .count();
        (errors, warnings)
    }
}

/// 解析前チェックの不備 1 件を診断行へ展開する。
///
/// 対象が特定できる不備は対象 1 件ごとに行を作り、クリックで 3D 選択へ
/// 飛べるようにする。大モデルで診断リストが溢れないよう
/// `MAX_ISSUE_TARGETS` 件で打ち切り、超過分は集約 1 行にまとめる。
///
/// 重大度は `ModelIssue` の判定をそのまま引き継ぐ。解析が成立しない不備は
/// Error、解析は通るが入力の意図を確かめたい事柄は Warning になる。
fn push_issue_diagnostics(
    diags: &mut Vec<Diagnostic>,
    issue: squid_n_solver::analysis::precheck::ModelIssue,
) {
    use squid_n_solver::analysis::precheck::{IssueSeverity, IssueTargets};

    /// 対象単位の行を並べる上限。超過分は集約 1 行にまとめる。
    const MAX_ISSUE_TARGETS: usize = 100;

    let severity = match issue.severity {
        IssueSeverity::Error => DiagSeverity::Error,
        IssueSeverity::Warning => DiagSeverity::Warning,
    };
    let (n_targets, unit) = match &issue.targets {
        IssueTargets::Model => {
            diags.push(Diagnostic {
                severity,
                message: issue.message,
                target: None,
            });
            return;
        }
        IssueTargets::Members(ids) => {
            for id in ids.iter().take(MAX_ISSUE_TARGETS) {
                diags.push(Diagnostic {
                    severity,
                    message: format!("部材 #{}: {}", id.0, issue.short),
                    target: Some(DiagTarget::Member(*id)),
                });
            }
            (ids.len(), "部材")
        }
        IssueTargets::Nodes(ids) => {
            for id in ids.iter().take(MAX_ISSUE_TARGETS) {
                diags.push(Diagnostic {
                    severity,
                    message: format!("節点 #{}: {}", id.0, issue.short),
                    target: Some(DiagTarget::Node(*id)),
                });
            }
            (ids.len(), "節点")
        }
    };
    if n_targets > MAX_ISSUE_TARGETS {
        diags.push(Diagnostic {
            severity,
            message: format!(
                "…他 {} {unit}で{}",
                n_targets - MAX_ISSUE_TARGETS,
                issue.short
            ),
            target: None,
        });
    }
}
