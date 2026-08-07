//! 部材塑性率（ファイバーモデルの塑性率、構造力学）の追跡。
//!
//! - [`DuctilityRef`] — 塑性率基点ひずみ
//! - [`compute_ductility_refs`] — 全部材の基点ひずみを算定
//! - [`DuctilityTracker`] — 基点曲率・最大応答曲率の追跡と μ 算定
//! - [`update_ductility`] — 全部材のトラッカー更新と μ 配列の返却

use super::types::DuctilityMethod;
use squid_n_core::model::Model;
use squid_n_element::behavior::{DuctilityProbe, ElementBehavior};

/// 塑性率基点ひずみ（ファイバーモデル（構造力学）の塑性率、方式(1)）。
/// RC 部材は引張 0.01・圧縮 0.005、鉄骨部材は引張・圧縮ともに 0.01。
#[derive(Clone, Copy)]
pub(crate) struct DuctilityRef {
    tens: f64,
    comp: f64,
}

pub(crate) fn compute_ductility_refs(model: &Model) -> Vec<DuctilityRef> {
    model
        .elements
        .iter()
        .map(|elem| {
            let is_rc = model.element_material(elem).and_then(|m| m.fc).is_some();
            if is_rc {
                DuctilityRef {
                    tens: 0.01,
                    comp: 0.005,
                }
            } else {
                DuctilityRef {
                    tens: 0.01,
                    comp: 0.01,
                }
            }
        })
        .collect()
}

/// 部材ごとの塑性率トラッカー。塑性率基点曲率（初到達時）と最大応答曲率を追跡し
/// μ=最大応答曲率/基点曲率を算定する（ファイバーモデルの塑性率、構造力学）。
#[derive(Clone, Copy, Default)]
pub(crate) struct DuctilityTracker {
    kappa_max: f64,
    kappa_ref: Option<f64>,
}

impl DuctilityTracker {
    fn update(&mut self, probe: &DuctilityProbe, reached: bool) {
        self.kappa_max = self.kappa_max.max(probe.curvature);
        if reached && self.kappa_ref.is_none() && probe.curvature > 0.0 {
            self.kappa_ref = Some(probe.curvature);
        }
    }
    /// 部材塑性率 μ。基点未到達（塑性率 1 未満）は 0（未評価、本実装の既定）。
    fn mu(&self) -> f64 {
        match self.kappa_ref {
            Some(kr) if kr > 0.0 => (self.kappa_max / kr).max(1.0),
            _ => 0.0,
        }
    }
}

/// 選択された方式で塑性率基点に到達したか判定する。
fn reference_reached(method: DuctilityMethod, probe: &DuctilityProbe, r: &DuctilityRef) -> bool {
    match method {
        DuctilityMethod::ReferenceStrain => {
            probe.max_tension_strain >= r.tens || probe.max_compression_strain >= r.comp
        }
        DuctilityMethod::WeightedAverageJm => probe.jm >= 1.0,
        DuctilityMethod::FirstYield => probe.max_yield_ratio >= 1.0,
    }
}

/// 全部材の塑性率トラッカーを更新し、部材塑性率 μ の配列を返す。
pub(crate) fn update_ductility(
    behaviors: &[Box<dyn ElementBehavior>],
    trackers: &mut [DuctilityTracker],
    refs: &[DuctilityRef],
    method: DuctilityMethod,
) -> Vec<f64> {
    for ((b, tr), r) in behaviors.iter().zip(trackers.iter_mut()).zip(refs.iter()) {
        if let Some(probe) = b.ductility_probe() {
            let reached = reference_reached(method, &probe, r);
            tr.update(&probe, reached);
        }
    }
    trackers.iter().map(|t| t.mu()).collect()
}
