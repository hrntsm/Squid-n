//! 荷重ケース自動同期・床荷重分配・CMQ 表示ソース。
//!
//! `actions` からの構造分割。アルゴリズム変更は行わない。

use super::*;

impl App {
    /// 全スラブの床荷重を大梁（および小梁経由の節点反力）へ分配し、
    /// `self.beam_loads` を更新する。対応する梁がない辺の荷重は捨てる。
    ///
    /// `squid_n_load::floor::distribute_slab` が返す `BeamLoad.target` は
    /// `LoadTarget::Edge(i)`（スラブ境界の辺 i、`boundary[i]` → `boundary[(i+1)%n]`、
    /// n = 境界頂点数。矩形に限らず三角形・五角形以上の多角形にも対応）または
    /// `LoadTarget::Node(id)`（小梁反力などの節点集中荷重）。`Edge` はここで
    /// その節点対を両端に持つ `Beam` 要素を探し、実 `ElemId` に置き換える
    /// （ノード順は不問）。`Node` はそのまま（`elem` は番兵 `ElemId(u32::MAX)`
    /// のまま）保持する（部材マッピング不要。`sync_gravity_load_cases_action` が
    /// `NodalLoad` へ変換する。CMQ 図描画側は `elem` で梁を引くため、この番兵は
    /// 単に描画対象外になるだけで安全）。
    pub fn refresh_beam_loads(&mut self) {
        let hash = self.compute_auto_load_sync_hash();
        if self.beam_loads_hash == Some(hash) {
            return;
        }
        self.beam_loads = squid_n_job::auto_loads::compute_dl_beam_loads(&self.model);
        self.beam_loads_hash = Some(hash);
    }

    /// CMQ 図（ビューア）の描画ソース。
    #[cfg(feature = "gui")]
    pub(crate) fn cmq_display_member_loads(&self) -> Vec<squid_n_core::model::MemberLoad> {
        squid_n_job::auto_loads::slab_load_case_content(&self.model, &self.beam_loads).1
    }

    /// 重力系の標準荷重ケース（DL・LL(架構用)・LL(地震用)）へ自動計算値を同期する
    /// （レビュー §1.1: 面荷重→大梁分配の結果を応力解析へ接続する最重要修正／
    /// 床 Phase A-2: 令85条1項の DL/LL 分離／照合レビュー: ③梁自重・②壁荷重の
    /// CMoQ 経路を長期応力解析へ接続）。
    ///
    /// - 「DL」（kind=Dead・[`DL_CASE_NAME`]）: スラブの `loads`（仕上げ等の
    ///   固定荷重）の分配と、躯体自重（柱梁・壁・ダンパー・フレーム外雑壁。
    ///   `squid_n_load::self_weight::self_weight_case_content`）の合算。
    /// - 「LL(架構用)」（kind=Live）: スラブ用途（`SlabUsage`）から令別表第1 の
    ///   **骨組用**積載（LL）を分配（長期骨組解析用。用途未設定のスラブは寄与 0）。
    /// - 「LL(地震用)」（kind=LiveSeismic）: スラブ用途から令別表第1 の地震用積載を
    ///   分配。`gravity_cases_for_seismic_weight` が LiveSeismic を優先採用するため、
    ///   地震用重量にはこの（骨組用より小さい）地震用値が算入される（令85条1項）。
    ///
    /// 各ケースについて現在の自動計算値を求め、既存ケースの内容と一致するなら
    /// 何もしない（undo 履歴・stale フラグを汚さない）。差分があれば
    /// `SyncSlabLoadsToCase`（全置換、undo 対応）を発行する。
    /// 対応するケースがなく内容も空の場合は空ケースを作らない。
    ///
    /// DL に自重を含めるため、階の自動生成（地震用重量）では密度からの自重直接
    /// 算入を無効にして二重計上を防ぐ（`density_self_weight_for_stories`）。
    ///
    /// 解析実行系（`sync_auto_load_cases_action` 経由）・`generate_stories_action`
    /// の入口で毎回呼ぶことを想定した冪等な同期アクション。
    pub fn sync_gravity_load_cases_action(&mut self) {
        let result = squid_n_job::auto_loads::compute_gravity_auto_load_cases(&self.model);
        self.beam_loads = result.dl_beam_loads;
        self.beam_loads_hash = None;
        for case in result.cases {
            self.sync_one_auto_case(case.name, case.kind, case.nodal, case.member);
        }
    }

    /// 地震荷重の標準ケース（EX・EY、kind=Seismic）へ Ai 分布の水平力を同期する。
    ///
    /// 階（`model.stories`）が定義されている場合に、地震静的解析と同じ載荷
    /// （`build_seismic_load_case_from_model`。方向・Ai算定法・Z・地盤種別・C0 は
    /// `analysis_cfg`）を EX/EY ケースへ書き込む。これにより荷重組合せ
    /// （G+P±K など）が EX/EY を参照して解析できる。
    ///
    /// 設計用固有周期 T は `design_seismic_period` で決定する（`Analysis::prepare`
    /// を要しないモデル単独版 `build_seismic_load_case_from_model` を使うため、
    /// 本関数自体は剛性行列組立や固有値解析を一切行わない）。X・Y 双方向で T を
    /// 共有するため `design_seismic_period` の呼び出しは 1 回のみ。
    ///
    /// 階が未定義・地震荷重が構築できない場合は何もしない（既存の EX/EY
    /// ケースは変更しない。組合せ実行時に空の地震ケースを参照していれば
    /// エラーで案内する）。SemiPrecise で固有値解析が未実行の場合も同様に
    /// 何もせず、代わりに `last_notice` へ実行を促すメッセージを設定する
    /// （`last_error` とは別枠。解析自体は継続してよい注意事項のため）。
    /// 冪等な同期アクション（`sync_gravity_load_cases_action` と同じ規約）。
    pub fn sync_seismic_load_cases_action(&mut self) {
        let design_period = match self.analysis_cfg.ai_mode {
            AiMode::SemiPrecise => match self.design_seismic_period() {
                Ok(t) => Some(t),
                Err(msg) => {
                    self.report_notice(msg);
                    return;
                }
            },
            AiMode::Approx => None,
        };
        let result = squid_n_job::auto_loads::compute_seismic_auto_load_cases(
            &self.model,
            &self.analysis_cfg,
            design_period,
        );
        for notice in result.notices {
            self.report_notice(notice);
        }
        for case in result.cases {
            self.sync_one_auto_case(case.name, case.kind, case.nodal, case.member);
        }
    }

    /// `sync_auto_load_cases_action` が同期の要否判定に使うハッシュを計算する。
    ///
    /// 荷重同期（DL/LL/EX/EY）の結果に影響し得る入力をすべて含める:
    /// モデル本体（`bincode` でシリアライズしてハッシュ）、地震荷重の
    /// Ai算定法（`ai_mode`）・地域係数 Z・地盤種別・標準せん断力係数 C0、
    /// および SemiPrecise 時は `design_seismic_period` の値（算定できた場合のみ。
    /// `to_bits()` でビット列化してハッシュ。固有値解析が未実行で `Err` の場合は
    /// 含めない＝モデル・設定が同じなら「未実行」状態も同一ハッシュに畳み込む）。
    pub(crate) fn compute_auto_load_sync_hash(&self) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        if let Ok(bytes) = bincode::serialize(&self.model) {
            bytes.hash(&mut hasher);
        }
        std::mem::discriminant(&self.analysis_cfg.ai_mode).hash(&mut hasher);
        self.analysis_cfg.z.to_bits().hash(&mut hasher);
        (self.analysis_cfg.soil as u8).hash(&mut hasher);
        self.analysis_cfg.c0.to_bits().hash(&mut hasher);
        if matches!(self.analysis_cfg.ai_mode, AiMode::SemiPrecise) {
            if let Ok(t) = self.design_seismic_period() {
                t.to_bits().hash(&mut hasher);
            }
        }
        hasher.finish()
    }

    /// 剛域の反映と、自重・積載・地震荷重の自動同期
    /// （`sync_gravity_load_cases_action`・`sync_seismic_load_cases_action`）を
    /// まとめて行う、準備計算
    /// （`ensure_preparation`・`run_preparation`）のモデル更新部分。
    ///
    /// モデルが交差小梁スラブを含む場合、床荷重分配（DL・LL(架構用)・
    /// LL(地震用)の3系統×床格子サブFEM解析）は重い処理になり得るため、
    /// 前回の同期時からモデル・関連設定（`compute_auto_load_sync_hash`）が
    /// 一切変わっていなければ丸ごとスキップする。
    ///
    /// 手順:
    /// 1. `apply_rigid_zones_for_analysis`（冪等・軽量なので常に実行。
    ///    剛域の反映は地震荷重の同期より先に行う。SemiPrecise の固有周期算定が
    ///    剛域込みの剛性を用いるようにするため）。
    /// 2. 現在のハッシュを計算し、前回保存したハッシュと一致すればスキップ。
    /// 3. 不一致なら `sync_gravity_load_cases_action` →
    ///    `sync_seismic_load_cases_action` の順で実行する。
    /// 4. 同期後（荷重ケースの内容が書き換わった後）のモデルで再度ハッシュを
    ///    計算して保存する（同期前のハッシュを保存すると、次回呼び出しで
    ///    「同期していないのに一致」と誤判定するため、必ず同期後の状態で保存する）。
    pub fn sync_auto_load_cases_action(&mut self) {
        self.apply_rigid_zones_for_analysis();
        let current = self.compute_auto_load_sync_hash();
        if self.auto_load_sync_hash == Some(current) {
            return;
        }
        let design_period = if matches!(self.analysis_cfg.ai_mode, AiMode::SemiPrecise) {
            self.design_seismic_period().ok()
        } else {
            None
        };
        let result = squid_n_job::auto_loads::compute_auto_load_cases(
            &self.model,
            &self.analysis_cfg,
            design_period,
        );
        self.beam_loads = result.dl_beam_loads;
        self.beam_loads_hash = None;
        for notice in result.notices {
            self.report_notice(notice);
        }
        for case in result.cases {
            self.sync_one_auto_case(case.name, case.kind, case.nodal, case.member);
        }
        self.auto_load_sync_hash = Some(self.compute_auto_load_sync_hash());
    }

    /// 名前付き荷重ケースを指定の `kind`・内容へ冪等に同期する
    /// （`sync_gravity_load_cases_action`／`sync_seismic_load_cases_action`
    /// の各ケース同期の共通処理）。
    ///
    /// 同期の対象は自動生成分（`LoadSource::Auto`）だけで、利用者が同じケースへ
    /// 手入力した荷重は残す。要否判定も自動生成分どうしの比較で行う
    /// （手入力を足しただけで同期が走ると、undo 履歴が無意味に伸びる）。
    fn sync_one_auto_case(
        &mut self,
        name: &str,
        kind: squid_n_core::model::LoadCaseKind,
        nodal: Vec<squid_n_core::model::NodalLoad>,
        member: Vec<squid_n_core::model::MemberLoad>,
    ) {
        let existing = self.model.load_cases.iter().find(|lc| lc.name == name);
        let needs_create = existing.is_none() && !(nodal.is_empty() && member.is_empty());
        let needs_update = existing
            .map(|lc| lc.kind != kind || !lc.auto_loads_match(&nodal, &member))
            .unwrap_or(false);
        if !needs_create && !needs_update {
            return;
        }

        self.undo.run(
            &mut self.model,
            Box::new(squid_n_edit::SyncSlabLoadsToCase {
                name: name.to_string(),
                kind,
                nodal,
                member,
            }),
        );
        self.staleness.mark_edited();
    }

    /// 組合せが参照する空の水平力ケース（kind=Seismic／Wind・内容なし）の名前を返す。
    /// 空の地震・風ケースを含む組合せをそのまま解くと水平力の項が黙って 0 になり、
    /// 長期と同じ結果を短期の検定に用いてしまうため、実行前のガードに使う
    /// （`run_combination`/`run_static_all`）。いずれも準備計算が
    /// EX/EY・WX/WY へ内容を生成するため、空のまま残っていることが異常の合図になる。
    pub(crate) fn empty_lateral_case_in_combo(
        &self,
        combo: &squid_n_core::model::LoadCombination,
    ) -> Option<String> {
        combo.terms.iter().find_map(|(id, _)| {
            self.model
                .load_cases
                .iter()
                .find(|lc| lc.id == *id)
                .filter(|lc| is_empty_lateral_case(lc))
                .map(|lc| lc.name.clone())
        })
    }
}
