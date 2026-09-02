//! 静的解析のファサード [`Analysis`]。
//!
//! `prepare` で DofMap 構築・全体剛性 K の組立・拘束縮約・分解を一度行い、以降は
//! 分解済み K を再利用して線形静的・荷重組合せ・固有値・時刻歴・地震の各解析を
//! 実行する。地震荷重の生成は [`seismic`]、設定型は [`config`]、
//! 解析前のモデル検証は [`precheck`] に分離している。

use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use crate::damping::Damping;
use crate::eigen::{self, ModalResult};
use crate::linear::{group_member_loads_by_elem, StaticOnce};
use crate::timehistory::{GroundMotion, NewmarkCfg, ResponseResult};
use std::collections::HashMap;

pub type StaticResult = StaticOnce;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{MemberLoad, Model};
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, LinearSolver, SolveError, SolverBackend};

mod combination;
mod config;
pub mod precheck;
mod seismic;

pub use combination::StaticBatch;
pub use config::{AiMode, SeismicCfg, SeismicDir};
pub(crate) use seismic::distribute_pi_over_diaphragms;
pub use seismic::{
    base_elevation, build_seismic_load_case_from_model, building_height_mm, ground_elevation,
    seismic_distribution_for_model, steel_height_ratio,
};

/// `model.load_cases` 全件の自由 DOF 荷重ベクトルを1回ずつ計算してマップに詰める
/// （[`Analysis::f_free_cache`] の構築。`prepare` から使う）。
fn build_f_free_cache(model: &Model, dofmap: &DofMap) -> HashMap<LoadCaseId, Vec<f64>> {
    model
        .load_cases
        .iter()
        .map(|lc| (lc.id, assemble_global_f(model, dofmap, lc.id)))
        .collect()
}

/// `model.elements` 全件の `(ElementBehavior, global_dofs)` を1回だけ構築する
/// （[`Analysis::behavior_cache`] の構築。`prepare` から使う）。
///
/// `build_behavior` は局所座標変換・断面/材料 clone・SRC/CFT 合成断面換算など
/// 荷重ケースに依存しない処理のため、荷重ケース・組合せごとに毎回呼び直す
/// 必要はない。静解析経路（[`build_behavior`]）は常に弾性要素を返す
/// （履歴状態を持たない）ため、`&self` から複数回・複数スレッドで参照しても
/// 安全（[`ElementBehavior`] の `Send + Sync` supertrait 経由）。
fn build_behavior_cache(model: &Model, dofmap: &DofMap) -> Vec<crate::statics::BehaviorEntry> {
    model
        .elements
        .iter()
        .map(|elem| {
            let behavior = build_behavior(elem, model);
            let gdofs = behavior.global_dofs(dofmap);
            (behavior, gdofs)
        })
        .collect()
}

pub struct Analysis<'m> {
    model: &'m Model,
    dofmap: DofMap,
    reducer: Reducer,
    solver: Box<dyn LinearSolver>,
    n_indep: usize,
    /// SemiPrecise の 1 次固有周期キャッシュ [s]。固有周期は載荷方向に依存しない
    /// ため、同一 `Analysis` 上で X・Y 両方向の地震荷重を構築しても固有値解析は
    /// 1 回で済む（`build_seismic_load_case`）。
    semi_precise_t: std::sync::OnceLock<f64>,
    /// `Model::load_cases` 各ケースの自由 DOF 荷重ベクトル（`assemble_global_f`）の
    /// メモ化。`prepare` 時に全ケースぶん1回だけ計算する（同じ荷重ケースを
    /// `linear_static` で繰り返し解く経路——荷重組合せは参照する荷重ケースを
    /// 単体で解いて線形和する——で、都度 `assemble_global_f` を再計算すると
    /// 無駄が大きいため）。`&self` のみで参照する
    /// （書き込みは `prepare` 構築時のみ）ため、`run_batch` の rayon
    /// 並列からも安全に共有できる。
    f_free_cache: HashMap<LoadCaseId, Vec<f64>>,
    /// `model.elements` 各要素の `(ElementBehavior, global_dofs)` のメモ化
    /// （[`build_behavior_cache`]）。`recover_member_forces` が荷重ケース・組合せ
    /// ごとに `build_behavior` を再構築していたのを避ける（局所座標変換・
    /// 断面/材料 clone・SRC/CFT 合成断面換算は荷重ケースに依存しないため）。
    /// `f_free_cache` と同様、書き込みは構築時のみで `run_batch` の rayon 並列
    /// からも安全に共有できる（`ElementBehavior: Send + Sync`）。
    behavior_cache: Vec<crate::statics::BehaviorEntry>,
}

impl<'m> Analysis<'m> {
    /// Build DofMap, assemble global K, apply constraint reduction, and factorize.
    /// After this, `linear_static` and `linear_combination` can be called
    /// multiple times reusing the factorized K.
    ///
    /// 解析前にモデルの静的検証（`Model::validate` の不変条件・参照整合・拘束・
    /// 断面/材料割当・孤立節点）を行い、問題があればユーザー向けの日本語診断
    /// メッセージ付きでエラーを返す。
    ///
    /// 検証は [`precheck::model_issues`] に一本化されており、UI のモデル整合性
    /// チェック（診断タブ）も同じ関数を呼ぶ。**ここへ検証を直接足さないこと**。
    /// 足すと診断が同じ不備を挙げられなくなり、「診断は通ったのに解析が止まる」
    /// 状態に戻る。
    pub fn prepare(model: &'m Model) -> Result<Self, SolveError> {
        squid_n_math::parallelism::apply_to_faer();
        precheck::precheck_model(model)?;
        let dofmap = DofMap::build(model);
        let n_active = dofmap.n_active();

        if n_active == 0 {
            return Ok(Self {
                model,
                dofmap,
                reducer: Reducer::empty(),
                solver: make_solver(SolverBackend::Auto),
                n_indep: 0,
                semi_precise_t: std::sync::OnceLock::new(),
                f_free_cache: HashMap::new(),
                behavior_cache: Vec::new(),
            });
        }

        let k_free = assemble_global_k(model, &dofmap);
        let reducer = Reducer::build(model, &dofmap);
        let n_indep = reducer.n_indep;
        let k_red = reducer.reduce_k(&k_free);

        let mut solver = make_solver(SolverBackend::Auto);
        if n_indep > 0 {
            solver.factorize(&k_red).map_err(|e| match e {
                SolveError::NotPositiveDefinite => {
                    SolveError::InvalidInput(precheck::singular_diagnosis(model))
                }
                other => other,
            })?;
        }

        let f_free_cache = build_f_free_cache(model, &dofmap);
        let behavior_cache = build_behavior_cache(model, &dofmap);

        Ok(Self {
            model,
            dofmap,
            reducer,
            solver,
            n_indep,
            semi_precise_t: std::sync::OnceLock::new(),
            f_free_cache,
            behavior_cache,
        })
    }

    /// 全自由度ゼロの結果（有効自由度なしのモデル用）。
    fn zero_result(&self) -> StaticOnce {
        StaticOnce {
            disp: vec![[0.0; 6]; self.model.nodes.len()],
            member_forces: Vec::new(),
            panel_moments: Vec::new(),
        }
    }

    /// 仕口パネルのせん断モーメント `{MSX, MSY}` を接合部の節点ごとに回収する。
    /// パネル要素がなければ空。
    fn recover_panel_moments(&self, u_free: &[f64]) -> Vec<(squid_n_core::ids::NodeId, [f64; 2])> {
        let mut out = Vec::new();
        for (elem, (behavior, gdofs)) in self.model.elements.iter().zip(self.behavior_cache.iter())
        {
            let Some(&node) = elem.nodes.first() else {
                continue;
            };
            let u_elem = crate::common::elem_loop::gather_u_elem(gdofs, u_free);
            if let Some(m) = behavior.panel_moments_from(&u_elem) {
                out.push((node, m));
            }
        }
        out
    }

    /// 自由 DOF 空間の荷重ベクトルを縮約 → 解 → 展開し、
    /// 節点変位と部材断面力を復元する（線形静的系の共通経路）。
    ///
    /// `member_loads` は当該解（荷重ケース／組合せ）に含まれる部材中間荷重で、
    /// 内力回復時に両端固定梁のスパン内力として重ね合わせる。節点荷重のみの
    /// 経路（地震・風）は空スライスを渡す。
    fn solve_and_recover(
        &self,
        f_free: &[f64],
        member_loads: &[MemberLoad],
    ) -> Result<StaticOnce, SolveError> {
        let f_red = self.reducer.reduce_f(f_free);
        let u_indep = self.solver.solve(&f_red)?;
        let u_free = self.reducer.expand_u(&u_indep);
        let member_forces = self.recover_member_forces(&u_free, member_loads);
        crate::linear::ensure_line_member_forces(self.model, &member_forces)?;
        Ok(StaticOnce {
            disp: self.expand_disp(&u_free),
            member_forces,
            panel_moments: self.recover_panel_moments(&u_free),
        })
    }

    /// 自由 DOF ベクトルを節点 6 成分配列へ展開する（単一実装は core 側）。
    fn expand_disp(&self, u_free: &[f64]) -> Vec<[f64; 6]> {
        self.dofmap.expand_to_nodes(u_free, self.model.nodes.len())
    }

    /// 自由 DOF ベクトルから全部材の断面力を復元する。
    ///
    /// `K·u` 由来の回復内力に、`member_loads` の部材中間荷重を両端固定梁の
    /// スパン内力として重ね合わせる（[`crate::linear::superpose_member_loads`]）。
    /// これにより等分布荷重下の梁で M が放物線・Q が線形の正しい分布になる。
    /// 分解済み K を再利用する `Analysis` 経路と、一度きりの
    /// [`crate::linear::linear_static_once`] 経路とで内力回復の扱いを一致させる。
    fn recover_member_forces(
        &self,
        u_free: &[f64],
        member_loads: &[MemberLoad],
    ) -> Vec<(
        squid_n_core::ids::ElemId,
        squid_n_element::beam::MemberForces,
    )> {
        // 要素 ID で事前にグルーピングし、要素ごとの全部材荷重総当りスキャンを避ける
        // （`crate::linear::solve_once_inner` と同じ最適化）。
        let member_loads_by_elem = group_member_loads_by_elem(member_loads);
        let mut member_forces = Vec::new();
        // `behavior_cache`（`prepare` で1回だけ構築済み）を参照する。
        // ケースごとの `build_behavior` 再構築（局所座標変換・断面/材料 clone 等）を
        // 排除する（要素順は `self.model.elements` と `behavior_cache` で一致する）。
        for (elem, (behavior, gdofs)) in self.model.elements.iter().zip(self.behavior_cache.iter())
        {
            let mut u_elem = vec![0.0; gdofs.len()];
            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }
            if let Some(mut forces) = behavior.recover_forces(&u_elem) {
                let loads = member_loads_by_elem
                    .get(&elem.id)
                    .map(Vec::as_slice)
                    .unwrap_or(&[]);
                crate::linear::superpose_member_loads(self.model, elem, loads, &mut forces);
                member_forces.push((elem.id, forces));
            }
        }
        member_forces
    }

    /// Solve a single load case (back-substitution only, factorized K is reused).
    pub fn linear_static(&self, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }
        if !self.model.load_cases.iter().any(|c| c.id == lc) {
            return Err(SolveError::InvalidInput(format!(
                "荷重ケース {} が存在しません",
                lc.0
            )));
        }
        // `prepare` が全荷重ケースぶん事前計算済みのメモ化を使う
        // （`assemble_global_f` の再計算を避ける）。キャッシュにない場合
        // （想定外の経路）はその場で計算する。
        let f_free = self
            .f_free_cache
            .get(&lc)
            .cloned()
            .unwrap_or_else(|| assemble_global_f(self.model, &self.dofmap, lc));
        let member_loads = self
            .model
            .load_cases
            .iter()
            .find(|c| c.id == lc)
            .map(|c| c.member.as_slice())
            .unwrap_or(&[]);
        self.solve_and_recover(&f_free, member_loads)
    }

    /// Solve eigenvalue problem (subspace iteration) for n_modes lowest modes.
    ///
    /// 通常は `prepare` で分解済みの `self.solver`（縮約後剛性行列 K_red の分解）を
    /// そのまま再利用する（[`eigen::solve_eigen_with_solver`] 参照）。
    /// 例外は [`Self::eigen_solver_dispatch`] を参照。
    pub fn eigen(&self, n_modes: usize) -> Result<ModalResult, SolveError> {
        self.eigen_solver_dispatch(n_modes)
    }

    /// 固有値解析に使うソルバの振り分け。
    ///
    /// `prepare` の `self.solver` は `SolverBackend::Auto` で生成しており、縮約後
    /// 自由度数が [`squid_n_math::auto::AUTO_ITERATIVE_MIN_DOF`] 以上のモデルでは
    /// f32 精度の反復法（PCG）が選ばれる。部分空間反復は 1 回の分解を
    /// （部分空間サイズ×反復回数）回の求解で再利用する構造のため、反復法では
    /// (1) 求解のたびに数千回規模の PCG 反復が走り桁違いに遅くなり、
    /// (2) f32 精度・緩い収束判定の解では固有値反復の収束判定（相対誤差 1e-10）に
    /// 達せず `NonConvergence` になり得る。このため PCG が選ばれる規模では
    /// `self.solver` を使わず、固有値解析専用に直接法（疎 Cholesky）で分解し直す
    /// [`eigen::solve_eigen`] へ振り分ける（静的解析側の PCG 採用はそのまま維持）。
    /// しきい値未満では `Auto` は常に直接法を選ぶため、従来どおり分解を再利用する。
    pub(crate) fn eigen_solver_dispatch(&self, n_modes: usize) -> Result<ModalResult, SolveError> {
        if self.n_indep >= squid_n_math::auto::AUTO_ITERATIVE_MIN_DOF {
            eigen::solve_eigen(self.model, &self.dofmap, &self.reducer, n_modes)
        } else {
            eigen::solve_eigen_with_solver(
                self.model,
                &self.dofmap,
                &self.reducer,
                n_modes,
                &*self.solver,
            )
        }
    }

    /// 複数の荷重ケースを一括で解く（分解済み K を共有）。
    ///
    /// 並列度設定（[`squid_n_math::parallelism`]）が並列（`Auto`/`Threads`）の
    /// 場合は荷重ケース単位に rayon で並列実行する。各ケースの計算はケース間で
    /// 可変状態を共有しない（`&self` のみ）ため実行順に依存せず、結果の順序は
    /// 入力 `lcs` の順で固定される。`Deterministic`（既定）では従来どおり
    /// 逐次実行し、`linear_static` を順に呼んだ場合とビット一致する。
    pub fn linear_static_batch(&self, lcs: &[LoadCaseId]) -> Vec<Result<StaticOnce, SolveError>> {
        self.run_batch(lcs, |lc| self.linear_static(*lc))
    }

    /// バッチ API の共通経路。並列設定時は項目単位に rayon で並列実行する。
    ///
    /// ケース並列（outer）とソルバ内部＝faer の並列（inner）の合計要求が
    /// コア数を超えるとスレッドを奪い合って逆に遅くなるため
    /// （`examples/parallel_bench` で実測）、総枠 `effective_threads()` を
    /// 両者へ自動配分する: ケース数がコア数以上なら inner=1（ケース並列のみ）、
    /// ケース数が少ないときは余りコア（cores/outer）を faer の内部並列へ回す。
    /// 終了後は設定値（`squid_n_math::parallelism`）を faer へ再適用して戻す。
    fn run_batch<T: Sync, R: Send>(
        &self,
        items: &[T],
        f: impl Fn(&T) -> R + Send + Sync,
    ) -> Vec<R> {
        use rayon::prelude::*;
        if squid_n_math::parallelism::is_parallel() && items.len() > 1 {
            let cores = squid_n_math::parallelism::effective_threads();
            let outer = items.len().min(cores);
            let inner = (cores / outer).max(1);
            if inner == 1 {
                faer::set_global_parallelism(faer::Par::Seq);
            } else {
                faer::set_global_parallelism(faer::Par::rayon(inner));
            }
            let out = squid_n_math::parallelism::run(|| items.par_iter().map(f).collect());
            squid_n_math::parallelism::apply_to_faer();
            out
        } else {
            // 1 件のみのバッチは逐次経路（設定どおりの faer 並列で 1 件を解く）
            items.iter().map(f).collect()
        }
    }

    /// 時刻歴応答解析（Newmark-β、減衰込み）。
    /// 線形専用ラッパ。非線形時刻歴は `timehistory::linear_time_history_analysis`
    /// と同じパターンのフリー関数で実装予定（§4、現在は線形のみ）。
    /// `record_every` は詳細記録（`ThRecording`）の間引き係数。`None` は自動決定。
    pub fn time_history(
        &self,
        wave: &GroundMotion,
        newmark: NewmarkCfg,
        damping: Damping,
        record_every: Option<usize>,
    ) -> Result<ResponseResult, squid_n_math::solver::SolveError> {
        let n_indep = self.n_indep;
        let init = vec![0.0; n_indep];
        crate::timehistory::linear_time_history_analysis(
            self.model,
            &self.dofmap,
            &self.reducer,
            wave,
            &newmark,
            &damping,
            &init,
            &init,
            false,
            record_every,
        )
    }

    /// LoadCase の節点荷重リストから自由 DOF 空間の荷重ベクトルを組み立てる
    /// （地震荷重・風荷重など静的荷重ケースの共通処理）。
    fn assemble_f_free_from_nodal(&self, nodal: &[squid_n_core::model::NodalLoad]) -> Vec<f64> {
        let n_active = self.dofmap.n_active();
        let mut f_free = vec![0.0; n_active];
        for nodal_load in nodal {
            let ni = nodal_load.node.index();
            for d in 0..squid_n_core::dof::DOF_PER_NODE {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = self.dofmap.active(g) {
                    f_free[active as usize] += nodal_load.values[d];
                }
            }
        }
        f_free
    }
}

#[cfg(test)]
mod tests;
