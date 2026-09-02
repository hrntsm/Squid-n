//! ナビゲータ（左パネル）の振動荷重ケースツリー。

use super::*;
use squid_n_core::ids::{LumpedVibrationCaseId, VibrationCaseId};

#[cfg(feature = "gui")]
enum VibrationNavAction {
    Spatial(VibrationCaseId),
    Lumped(LumpedVibrationCaseId),
}

#[cfg(feature = "gui")]
impl App {
    /// 振動荷重ケース（立体時刻歴・質点系）。
    pub(crate) fn nav_vibration_cases(&mut self, ui: &mut egui::Ui) {
        let mut action: Option<VibrationNavAction> = None;

        let header = egui::CollapsingHeader::new("振動荷重ケース")
            .default_open(false)
            .id_salt("nav_vibration_cases");
        header.show(ui, |ui| {
            if self.core.model.vibration_cases.is_empty()
                && self.core.model.lumped_vibration_cases.is_empty()
            {
                ui.colored_label(crate::theme::GRAY_600, "（なし）");
                return;
            }

            let spatial_header = egui::CollapsingHeader::new("立体時刻歴")
                .default_open(false)
                .id_salt("nav_vibration_spatial");
            spatial_header.show(ui, |ui| {
                if self.core.model.vibration_cases.is_empty() {
                    ui.colored_label(crate::theme::GRAY_600, "（なし）");
                }
                for case in &self.core.model.vibration_cases {
                    let is_sel = self.ui.scoped.nav.focus_vibration_case == Some(case.id);
                    if ui.selectable_label(is_sel, &case.name).clicked() {
                        action = Some(VibrationNavAction::Spatial(case.id));
                    }
                }
            });

            let lumped_header = egui::CollapsingHeader::new("質点系")
                .default_open(false)
                .id_salt("nav_vibration_lumped");
            lumped_header.show(ui, |ui| {
                if self.core.model.lumped_vibration_cases.is_empty() {
                    ui.colored_label(crate::theme::GRAY_600, "（なし）");
                }
                for case in &self.core.model.lumped_vibration_cases {
                    let is_sel = self.ui.scoped.nav.focus_lumped_vibration_case == Some(case.id);
                    if ui.selectable_label(is_sel, &case.name).clicked() {
                        action = Some(VibrationNavAction::Lumped(case.id));
                    }
                }
            });
        });

        if let Some(action) = action {
            match action {
                VibrationNavAction::Spatial(id) => self.focus_spatial_vibration_case(id),
                VibrationNavAction::Lumped(id) => self.focus_lumped_vibration_case(id),
            }
        }
    }
}
