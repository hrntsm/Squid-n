//! `App` のアクション（解析実行・モデル操作）メソッド。
//!
//! 責務ごとのサブモジュール:
//! - [`design`] — 許容・終局・保有水平耐力・床内検定
//! - [`io`] — プロジェクト／ST-Bridge 入出力とモデル読込
//! - [`loads`] — 荷重ケース自動同期・床荷重分配
//! - [`linear_static`] — 線形静的解析（荷重ケース・組合せ・一括・地震静的）
//! - [`eigen`] — 固有値解析
//! - [`pushover`] — 増分解析
//! - [`time_history`] — 時刻歴応答解析
//! - [`wave_library`] — 時刻歴応答解析の波形ライブラリ（登録・選択実行）

use super::*;

mod design;
mod eigen;
mod io;
mod linear_static;
mod loads;
mod lumped_mass;
mod pushover;
mod time_history;
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
            self.open_log_dock();
        }
    }

    /// 下ドックを開き、ログタブを前面にする。
    ///
    /// ステータスバーのエラー行クリックと [`Self::report_error`] が同じ導線を使う。
    /// 別タブ（診断・テーブル）が表示中でもエラー本文が見えるようにする。
    #[cfg(feature = "gui")]
    pub(crate) fn open_log_dock(&mut self) {
        self.bottom_dock_open = true;
        self.bottom_tab = BottomTab::Log;
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
                    JobResult::LumpedMass(res) => self.apply_lumped_mass_result(*res),
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
                // 各解析パネルへは右アイコン列から移る。
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
        //
        // `model_issues` は壁展開モデル（壁の解析要素を含む一時的な複製。D5）を
        // 渡す。`self.model` そのままでは壁要素が 0 件のため、耐震壁と周辺架構の
        // 種別食い違い等、壁関連の診断が一切出なくなる（解析実行時の
        // `squid_n_job::compute` 側の展開と同じ理由。忘れると壁の不備が診断タブに
        // 現れないまま解析実行時に初めて分かる、という劣化になる）。
        let (wall_expanded_model, _wall_index, _wall_report) =
            squid_n_load::wall_expand::expand_wall_elements(&self.model);
        for issue in squid_n_solver::analysis::precheck::model_issues(&wall_expanded_model) {
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
