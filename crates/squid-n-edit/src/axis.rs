//! 通り芯の編集コマンド。
//!
//! - [`ReplaceAxes`] — 通り芯の全置換（自動生成の適用・グループ単位の操作）
//! - [`RenameAxis`] — 通り名の変更（変更した通りは [`AxisSource::Manual`] へ移る）
//!
//! 通り芯は構造計算に用いないため、これらの操作は解析結果・設計結果を陳腐化させない
//! （呼び出し側は `Staleness::mark_edited` ではなく、未保存フラグだけを立てる経路を使う）。

use squid_n_core::model::{AxisGroup, AxisSource, Model};

use crate::EditCommand;

/// 通り芯（[`Model::axes`]）の全置換。逆操作は置換前の内容による同じコマンド。
///
/// 自動生成（`squid_n_core::axis_gen::generate_axes`）の適用に用いる。通り芯は
/// 節点・要素と違って ID の繰り上げを伴わないため、全置換で undo が閉じる。
///
/// 通りが実在しない節点を指す場合は何もしない（`Noop`。crate::refs の規約）。
pub struct ReplaceAxes {
    pub axes: Vec<AxisGroup>,
}

impl EditCommand for ReplaceAxes {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let all_nodes_exist = self.axes.iter().all(|g| {
            g.axes
                .iter()
                .all(|a| a.nodes.iter().all(|&n| crate::refs::node_exists(model, n)))
        });
        if !all_nodes_exist {
            return Box::new(crate::Noop);
        }
        let axes = std::mem::replace(&mut model.axes, self.axes.clone());
        Box::new(ReplaceAxes { axes })
    }

    fn label(&self) -> &str {
        "通り芯の更新"
    }
}

/// 通り名の変更。逆操作は変更前の名前・出所へ戻す [`RestoreAxisName`]。
///
/// 利用者が名前を編集した通りは [`AxisSource::Manual`] へ移り、以後の自動生成で
/// 作り直されなくなる（利用者の入力を自動生成が上書きしないため）。
pub struct RenameAxis {
    /// [`Model::axes`] 内のグループ添字。
    pub group: usize,
    /// グループ内の通りの添字。
    pub axis: usize,
    pub name: String,
}

impl EditCommand for RenameAxis {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(axis) = model
            .axes
            .get_mut(self.group)
            .and_then(|g| g.axes.get_mut(self.axis))
        else {
            return Box::new(crate::Noop);
        };
        let name = std::mem::replace(&mut axis.name, self.name.clone());
        let source = std::mem::replace(&mut axis.source, AxisSource::Manual);
        Box::new(RestoreAxisName {
            group: self.group,
            axis: self.axis,
            name,
            source,
        })
    }

    fn label(&self) -> &str {
        "通り名の変更"
    }
}

/// 通り名と出所を元へ戻す（[`RenameAxis`] の逆操作専用）。
pub struct RestoreAxisName {
    pub group: usize,
    pub axis: usize,
    pub name: String,
    pub source: AxisSource,
}

impl EditCommand for RestoreAxisName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(axis) = model
            .axes
            .get_mut(self.group)
            .and_then(|g| g.axes.get_mut(self.axis))
        else {
            return Box::new(crate::Noop);
        };
        let name = std::mem::replace(&mut axis.name, self.name.clone());
        let source = std::mem::replace(&mut axis.source, self.source);
        Box::new(RestoreAxisName {
            group: self.group,
            axis: self.axis,
            name,
            source,
        })
    }

    fn label(&self) -> &str {
        "通り名の変更の取り消し"
    }
}
