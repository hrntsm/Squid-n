use squid_n_element::behavior::ElementBehavior;
use std::any::Any;

/// 全要素の確定状態のスナップショット。
///
/// 非線形解析の増分ステップが収束しなかったとき、ステップ開始時点の要素状態へ
/// 巻き戻すために用いる。要素状態は `behaviors` 側が保持しており `Model` は
/// 関与しないため、キャプチャ・復元とも `behaviors` だけを引数に取る
/// （旧 `StatefulModel` トレイトは self 未使用のまま `&mut Model` を強制して
/// いたため廃止した）。
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
