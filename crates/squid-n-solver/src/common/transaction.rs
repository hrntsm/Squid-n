use squid_n_element::behavior::ElementBehavior;
use std::any::Any;

/// 全要素の確定状態のスナップショット。
pub struct StateSnapshot {
    pub states: Vec<Box<dyn Any>>,
}

impl StateSnapshot {
    /// 現在の全要素の状態をキャプチャ
    pub fn capture(behaviors: &[Box<dyn ElementBehavior>]) -> Self {
        StateSnapshot {
            states: behaviors.iter().map(|b| b.snapshot_state()).collect(),
        }
    }

    /// キャプチャ時点の状態へ全要素を復元する
    pub fn restore(&self, behaviors: &mut [Box<dyn ElementBehavior>]) {
        for (b, s) in behaviors.iter_mut().zip(&self.states) {
            b.restore_state(s.as_ref());
        }
    }
}

/// 全要素の trial を committed に戻す（rollback）
pub fn revert_all(behaviors: &mut [Box<dyn ElementBehavior>]) {
    for b in behaviors.iter_mut() {
        b.revert_state();
    }
}
