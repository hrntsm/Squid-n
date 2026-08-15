//! 振動荷重ケース（立体時刻歴・質点系時刻歴）。
//!
//! 静的荷重ケース（[`LoadCase`]）とは別系統で、解析実行時にのみ生成する。

use crate::ids::{LumpedVibrationCaseId, VibrationCaseId};

/// 立体時刻歴の入力方向（[`squid_n_job::settings::ThDir`] と同値だが core 単体で完結させる）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum VibrationThDir {
    X,
    Y,
    Xy,
}

/// 質点系振動ケースの入力方向（X/Y のみ）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LumpedVibrationDir {
    X,
    Y,
}

/// 質点系振動ケースのモデル次元。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LumpedVibrationDim {
    /// 2 次元せん断串。
    Planar,
    /// 3 次元。
    Spatial,
}

/// 立体時刻歴応答解析の振動ケース（実行時にモデルへ upsert する）。
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct VibrationCase {
    pub id: VibrationCaseId,
    pub name: String,
    pub wave_name: String,
    pub dir: VibrationThDir,
    pub nonlinear: bool,
}

/// 質点系時刻歴応答解析の振動ケース。
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct LumpedVibrationCase {
    pub id: LumpedVibrationCaseId,
    pub name: String,
    pub wave_name: String,
    pub dir: LumpedVibrationDir,
    pub nonlinear: bool,
    pub dim: LumpedVibrationDim,
}

fn dir_label_th(dir: VibrationThDir) -> &'static str {
    match dir {
        VibrationThDir::X => "X",
        VibrationThDir::Y => "Y",
        VibrationThDir::Xy => "X+Y",
    }
}

fn dir_label_lumped(dir: LumpedVibrationDir) -> &'static str {
    match dir {
        LumpedVibrationDir::X => "X",
        LumpedVibrationDir::Y => "Y",
    }
}

fn linearity_label(nonlinear: bool) -> &'static str {
    if nonlinear {
        "非線形"
    } else {
        "線形"
    }
}

fn dim_label(dim: LumpedVibrationDim) -> &'static str {
    match dim {
        LumpedVibrationDim::Planar => "2次元",
        LumpedVibrationDim::Spatial => "3次元",
    }
}

/// 立体時刻歴振動ケースの表示名（規約 B）。
pub fn spatial_vibration_case_name(
    wave_name: &str,
    dir: VibrationThDir,
    nonlinear: bool,
) -> String {
    format!(
        "{} {} ({})",
        wave_name,
        dir_label_th(dir),
        linearity_label(nonlinear)
    )
}

/// 質点系振動ケースの表示名。
pub fn lumped_vibration_case_name(
    wave_name: &str,
    dir: LumpedVibrationDir,
    nonlinear: bool,
    dim: LumpedVibrationDim,
) -> String {
    format!(
        "{} {} ({}・{})",
        wave_name,
        dir_label_lumped(dir),
        linearity_label(nonlinear),
        dim_label(dim)
    )
}

fn next_vibration_case_id(cases: &[VibrationCase]) -> VibrationCaseId {
    let next = cases
        .iter()
        .map(|c| c.id.0)
        .max()
        .map(|m| m + 1)
        .unwrap_or(0);
    VibrationCaseId(next)
}

fn next_lumped_vibration_case_id(cases: &[LumpedVibrationCase]) -> LumpedVibrationCaseId {
    let next = cases
        .iter()
        .map(|c| c.id.0)
        .max()
        .map(|m| m + 1)
        .unwrap_or(0);
    LumpedVibrationCaseId(next)
}

impl super::Model {
    /// 同名の立体振動ケースがあれば ID を維持して属性を更新し、なければ追加する。
    pub fn upsert_vibration_case(
        &mut self,
        wave_name: String,
        dir: VibrationThDir,
        nonlinear: bool,
    ) -> VibrationCaseId {
        let name = spatial_vibration_case_name(&wave_name, dir, nonlinear);
        if let Some(pos) = self.vibration_cases.iter().position(|c| c.name == name) {
            let id = self.vibration_cases[pos].id;
            self.vibration_cases[pos].wave_name = wave_name;
            self.vibration_cases[pos].dir = dir;
            self.vibration_cases[pos].nonlinear = nonlinear;
            return id;
        }
        let id = next_vibration_case_id(&self.vibration_cases);
        self.vibration_cases.push(VibrationCase {
            id,
            name,
            wave_name,
            dir,
            nonlinear,
        });
        id
    }

    /// 同名の質点系振動ケースがあれば ID を維持して属性を更新し、なければ追加する。
    pub fn upsert_lumped_vibration_case(
        &mut self,
        wave_name: String,
        dir: LumpedVibrationDir,
        nonlinear: bool,
        dim: LumpedVibrationDim,
    ) -> LumpedVibrationCaseId {
        let name = lumped_vibration_case_name(&wave_name, dir, nonlinear, dim);
        if let Some(pos) = self
            .lumped_vibration_cases
            .iter()
            .position(|c| c.name == name)
        {
            let id = self.lumped_vibration_cases[pos].id;
            self.lumped_vibration_cases[pos].wave_name = wave_name;
            self.lumped_vibration_cases[pos].dir = dir;
            self.lumped_vibration_cases[pos].nonlinear = nonlinear;
            self.lumped_vibration_cases[pos].dim = dim;
            return id;
        }
        let id = next_lumped_vibration_case_id(&self.lumped_vibration_cases);
        self.lumped_vibration_cases.push(LumpedVibrationCase {
            id,
            name,
            wave_name,
            dir,
            nonlinear,
            dim,
        });
        id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spatial_vibration_case_name_format() {
        assert_eq!(
            spatial_vibration_case_name("サンプル", VibrationThDir::X, false),
            "サンプル X (線形)"
        );
        assert_eq!(
            spatial_vibration_case_name("elcentro", VibrationThDir::Y, true),
            "elcentro Y (非線形)"
        );
        assert_eq!(
            spatial_vibration_case_name("wave", VibrationThDir::Xy, false),
            "wave X+Y (線形)"
        );
    }

    #[test]
    fn lumped_vibration_case_name_format() {
        assert_eq!(
            lumped_vibration_case_name(
                "サンプル",
                LumpedVibrationDir::X,
                false,
                LumpedVibrationDim::Planar
            ),
            "サンプル X (線形・2次元)"
        );
        assert_eq!(
            lumped_vibration_case_name(
                "wave",
                LumpedVibrationDir::Y,
                true,
                LumpedVibrationDim::Spatial
            ),
            "wave Y (非線形・3次元)"
        );
    }

    #[test]
    fn upsert_vibration_case_preserves_id_on_same_name() {
        let mut model = super::super::Model::default();
        let id1 = model.upsert_vibration_case("サンプル".into(), VibrationThDir::X, false);
        let id2 = model.upsert_vibration_case("サンプル".into(), VibrationThDir::X, false);
        assert_eq!(id1, id2);
        assert_eq!(model.vibration_cases.len(), 1);
    }

    #[test]
    fn upsert_vibration_case_distinct_names_get_distinct_ids() {
        let mut model = super::super::Model::default();
        let id1 = model.upsert_vibration_case("サンプル".into(), VibrationThDir::X, false);
        let id2 = model.upsert_vibration_case("サンプル".into(), VibrationThDir::Y, false);
        assert_ne!(id1, id2);
        assert_eq!(model.vibration_cases.len(), 2);
    }
}
