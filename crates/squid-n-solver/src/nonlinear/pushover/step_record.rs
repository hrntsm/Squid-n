//! 増分解析の「1 ステップ確定時の記録」を 1 か所へ集める。
//!
//! 増分解析は長期載荷・荷重制御・変位制御・弧長法の 4 フェーズを順に通るが、
//! **ステップが確定したときに残す記録はどのフェーズでも同一**である。
//! 性能曲線（[`CapacityPoint`]）・ステップ列（[`PushoverStep`]）・塑性率の更新・
//! ヒンジ・せん断降伏の追跡・部材応答履歴の 6 つを、同じ順序・同じ通し番号で
//! 積む必要がある。フェーズごとに違うのは荷重係数 `load_factor` の求め方だけで、
//! それは呼び出し側が決めて渡す。
//!
//! この記録を各フェーズへ複製すると、**追跡項目を増やしたときに更新漏れが起きる**。
//! 実際に、弧長法フェーズだけヒンジ・せん断降伏の追跡が抜けており、耐力ピーク以降に
//! 形成されるヒンジが崩壊機構の判定とヒンジ詳細図から欠落していた。同じ形の抜けが
//! 二度と起きないよう、記録の中身は [`StepRecorder::record`] だけが知る形にする。
//!
//! 終了目標（頂部変位・最大層間変形角）の判定は**呼び出し側に残す**。長期載荷は
//! 水平力に対する応答ではないため目標判定の対象にならず、4 フェーズで扱いが揃わない
//! ためである。判定に要る頂部変位・最大層間変形角は [`RecordedStep`] で返す。

use super::ductility::{compute_ductility_refs, update_ductility, DuctilityRef, DuctilityTracker};
use super::hinge::{compute_hinge_thresholds, track_hinges, HingeThreshold};
use super::member_response::record_member_step;
use super::response::{
    compute_base_shear, compute_story_drift, compute_story_shear, get_roof_disp,
    max_story_drift_angle,
};
use super::shear_yield::{compute_shear_yield_thresholds, track_shear_yield, ShearThreshold};
use super::types::{
    CapacityPoint, DuctilityMethod, HingeEvent, MemberStepState, PushoverStep, ShearYieldEvent,
};
use crate::common::tangent::{add_support_spring_f_int, compute_f_int};
use crate::statics::analysis::SeismicDir;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::ElementBehavior;

/// 1 ステップを記録した結果のうち、呼び出し側が制御判断に使う値。
///
/// 終了目標の判定（[`super::types::PushoverTarget::reached`]）と、均等刻みの
/// 勾配更新（頂部変位の増分）に用いる。
pub(super) struct RecordedStep {
    /// 頂部変位 [mm]（加力方向成分）。
    pub(super) roof: f64,
    /// 全層の最大層間変形角 [rad]。
    pub(super) drift_angle: f64,
}

/// 確定ステップの記録先と、記録に要る追跡状態をまとめて持つ。
///
/// 蓄積したものは解析の最後に [`StepRecorder::finish`] で受け取る。
pub(super) struct StepRecorder<'a> {
    /// 加力方向（ベースシア・層せん断・層間変位の射影に使う）。
    dir: SeismicDir,
    /// 各層の階高 [mm]（層間変形角の分母）。
    heights: &'a [f64],
    /// 部材ごとの曲げヒンジ発生モーメント閾値。
    hinge_thresholds: Vec<HingeThreshold>,
    /// 部材ごとのせん断降伏耐力 Qy。
    shear_thresholds: Vec<ShearThreshold>,
    /// 部材ごとの塑性率基点。
    ductility_refs: Vec<DuctilityRef>,
    /// 塑性率の定義（基点曲率の取り方）。
    ductility_method: DuctilityMethod,
    /// 部材ごとの塑性率トラッカー（最大応答曲率を保持する）。
    ductility_trackers: Vec<DuctilityTracker>,
    /// 記録用の通し番号。長期載荷→荷重制御→変位制御→弧長法で連番とし、
    /// **確定したステップにのみ**採番する。`capacity_curve`・`steps` の並びと
    /// ヒンジ・せん断降伏イベントの `step` を対応付ける単調キーになる。
    step_no: u32,
    capacity_curve: Vec<CapacityPoint>,
    steps: Vec<PushoverStep>,
    hinges: Vec<HingeEvent>,
    shear_yields: Vec<ShearYieldEvent>,
    member_history_steps: Vec<Vec<MemberStepState>>,
}

/// 解析終了時に [`StepRecorder`] から取り出す蓄積結果。
pub(super) struct RecordedSteps {
    pub(super) capacity_curve: Vec<CapacityPoint>,
    pub(super) steps: Vec<PushoverStep>,
    pub(super) hinges: Vec<HingeEvent>,
    pub(super) shear_yields: Vec<ShearYieldEvent>,
    pub(super) member_history_steps: Vec<Vec<MemberStepState>>,
}

impl<'a> StepRecorder<'a> {
    pub(super) fn new(
        model: &Model,
        dir: SeismicDir,
        heights: &'a [f64],
        ductility_method: DuctilityMethod,
    ) -> Self {
        Self {
            dir,
            heights,
            hinge_thresholds: compute_hinge_thresholds(model),
            shear_thresholds: compute_shear_yield_thresholds(model),
            ductility_refs: compute_ductility_refs(model),
            ductility_method,
            ductility_trackers: vec![DuctilityTracker::default(); model.elements.len()],
            step_no: 0,
            capacity_curve: Vec::new(),
            steps: Vec::new(),
            hinges: Vec::new(),
            shear_yields: Vec::new(),
            member_history_steps: Vec::new(),
        }
    }

    /// 確定したステップを 1 つ記録する。
    ///
    /// `load_factor` は参照外力ベクトル q に対する倍率で、フェーズごとに求め方が
    /// 異なる（長期載荷=0、荷重制御=`current_lambda`、変位制御=変位拘束から解いた
    /// λ、弧長法=`arc_lambda`）。それ以外はすべてのフェーズで共通に扱う。
    ///
    /// ベースシアは**内力の釣合いから**算定する。荷重制御では載荷ベクトルの総和でも
    /// 一致するが、フェーズによって求め方が変わらないよう全フェーズを反力ベースへ揃える。
    pub(super) fn record(
        &mut self,
        load_factor: f64,
        model: &Model,
        dofmap: &DofMap,
        behaviors: &[Box<dyn ElementBehavior>],
        total_disp: &[f64],
    ) -> RecordedStep {
        let roof = get_roof_disp(total_disp, model, dofmap, self.dir);
        let mut f_int_now = compute_f_int(model, dofmap, behaviors);
        add_support_spring_f_int(model, dofmap, total_disp, &mut f_int_now);
        let base_shear = compute_base_shear(model, dofmap, &f_int_now, self.dir);
        let story_drift = compute_story_drift(model, dofmap, total_disp, self.dir);
        let drift_angle = max_story_drift_angle(&story_drift, self.heights);

        self.capacity_curve.push(CapacityPoint {
            step: self.step_no,
            roof_disp: roof,
            base_shear,
            story_shear: compute_story_shear(model, dofmap, &f_int_now, self.dir),
            story_drift: story_drift.clone(),
        });
        self.steps.push(PushoverStep {
            load_factor,
            top_disp: roof,
            base_shear,
            story_drifts: story_drift,
        });
        let mu = update_ductility(
            behaviors,
            &mut self.ductility_trackers,
            &self.ductility_refs,
            self.ductility_method,
        );
        track_hinges(
            model,
            behaviors,
            &self.hinge_thresholds,
            &mu,
            self.step_no,
            &mut self.hinges,
        );
        track_shear_yield(
            model,
            behaviors,
            &self.shear_thresholds,
            self.step_no,
            &mut self.shear_yields,
        );
        self.member_history_steps
            .push(record_member_step(model, dofmap, behaviors, total_disp));
        self.step_no += 1;

        RecordedStep { roof, drift_angle }
    }

    /// 1 ステップも確定していないか。荷重制御の最初の増分すら収束しなかった場合で、
    /// 空の性能曲線（Qu=0）を「解析できた」として返すと保有水平耐力を 0 と
    /// 誤認させる（危険側）ため、呼び出し側はこれを見て停止する。
    pub(super) fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    /// 性能曲線上の最大ベースシア＝保有水平耐力 Qu [N]。
    /// 単調載荷では崩壊機構形成後に頭打ちとなるため、ピーク値を採る。
    pub(super) fn qu(&self) -> f64 {
        self.capacity_curve
            .iter()
            .map(|c| c.base_shear)
            .fold(0.0_f64, f64::max)
    }

    /// これまでに記録したヒンジ発生イベント（崩壊機構の判定に使う）。
    pub(super) fn hinges(&self) -> &[HingeEvent] {
        &self.hinges
    }

    /// これまでに記録したせん断降伏イベント。
    pub(super) fn shear_yields(&self) -> &[ShearYieldEvent] {
        &self.shear_yields
    }

    pub(super) fn finish(self) -> RecordedSteps {
        RecordedSteps {
            capacity_curve: self.capacity_curve,
            steps: self.steps,
            hinges: self.hinges,
            shear_yields: self.shear_yields,
            member_history_steps: self.member_history_steps,
        }
    }
}
