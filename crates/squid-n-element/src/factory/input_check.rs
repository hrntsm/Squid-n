//! 非線形解析（保有水平耐力計算・非線形時刻歴応答解析）の入力チェック。
//!
//! - [`nonlinear_input_issues`] — 部材耐力を算定できない設定不備の列挙
//! - [`ensure_nonlinear_input`] — 不備があれば是正内容を示すエラーを返す
//!
//! 非線形解析は部材の終局耐力（曲げ降伏 My・せん断終局 Qsu・耐震壁の Qu）で
//! 応力を頭打ちにすることで崩壊機構を形成する。耐力を算定できない部材は
//! **弾性のまま／降伏しないまま**扱われ、押し込むほど際限なく応力を負担するため、
//! 崩壊機構が形成されず保有水平耐力を過大評価する（**危険側**）。
//!
//! したがって、材料強度が未入力で耐力を算定できない部材があるモデルは、
//! 代替値（既定の Fc・fy）で無音に埋めず、解析を停止して利用者へ是正を促す。

use squid_n_core::model::{ElementData, ElementKind, Model};

/// エラーメッセージへ列挙する不備の最大件数（超過分は件数のみを示す）。
const MAX_LISTED: usize = 5;

/// 非線形解析で部材耐力を算定できない設定不備を列挙する（是正内容を示す日本語文）。
///
/// 検査対象は、非線形解析で降伏を扱う要素に限る:
/// - 耐震壁（`Wall`）: 面内せん断の終局強度 Qu
///   （[`crate::wall_panel::WallPanelElement::wall_shear_capacity_issue`]）
/// - 線材（`Beam` / `Fiber` / `MultiSpring`）: 曲げ・せん断の終局耐力に要する材料強度
///
/// 弾性としてモデル化することが仕様である要素（`Shell` / `PanelZone` / `NodalSpring`、
/// および特性を専用の属性で持つ `Isolator` / `Damper` / `Brace`）は対象外。
pub fn nonlinear_input_issues(model: &Model) -> Vec<String> {
    let mut issues = Vec::new();
    for elem in &model.elements {
        let issue = match elem.kind {
            ElementKind::Wall => {
                crate::wall_panel::WallPanelElement::wall_shear_capacity_issue(elem, model)
            }
            ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring => {
                member_strength_issue(elem, model)
            }
            _ => None,
        };
        if let Some(msg) = issue {
            issues.push(msg);
        }
    }
    issues
}

/// [`nonlinear_input_issues`] が不備を検出した場合に、解析を停止するための
/// エラーメッセージ（先頭 [`MAX_LISTED`] 件＋残件数）を返す。
pub fn ensure_nonlinear_input(model: &Model) -> Result<(), String> {
    let issues = nonlinear_input_issues(model);
    if issues.is_empty() {
        return Ok(());
    }
    let head: Vec<String> = issues.iter().take(MAX_LISTED).cloned().collect();
    let more = if issues.len() > MAX_LISTED {
        format!("\n他 {} 件", issues.len() - MAX_LISTED)
    } else {
        String::new()
    };
    Err(format!("{}{}", head.join("\n"), more))
}

/// 線材の終局耐力を算定できない設定不備があれば、その内容を返す。
///
/// 検出する不備:
/// - 材料が設定されていない。曲げヒンジ判定は鋼材既定 235 N/mm²、せん断降伏判定は
///   Qy=∞（＝せん断降伏しない）となり、いずれも根拠のない耐力で解析が通ってしまう。
/// - コンクリート系断面（RC/SRC/CFT）なのに材料の Fc が未設定または 0 以下。
///   曲げひび割れ Mc=0.56·√Fc·Ze が 0 となりヒンジが一切検出されず、ファイバー断面は
///   Fc=24 N/mm² を勝手に仮定し、せん断降伏耐力も荒川式を適用できない。
/// - 鋼系・断面形状未設定の部材で、材料に fy も Fc も無い。せん断降伏耐力が
///   Qy=∞ となり、その部材はいくら応力が上がっても降伏しない。
fn member_strength_issue(data: &ElementData, model: &Model) -> Option<String> {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let Some(mat) = data
        .material
        .and_then(|mid| model.materials.get(mid.index()))
    else {
        return Some(format!(
            "部材 ID {} に材料が設定されていません。\
             材料タブで材料を割り当ててください。\
             非線形解析では材料強度から部材の終局耐力を算定します。",
            data.id.0
        ));
    };
    let is_concrete = sec
        .and_then(|s| s.shape.as_ref())
        .is_some_and(|s| s.is_concrete_like());
    if is_concrete {
        return match mat.fc {
            None => Some(format!(
                "部材 ID {} はコンクリート系断面ですが、材料「{}」にコンクリート強度 Fc が\
                 設定されていません。材料タブで Fc を設定してください。\
                 非線形解析では Fc から曲げひび割れ・曲げ降伏・せん断終局の各耐力を算定します。",
                data.id.0, mat.name
            )),
            Some(fc) if fc <= 0.0 => Some(format!(
                "部材 ID {} の材料「{}」のコンクリート強度 Fc が {} で 0 以下です。\
                 材料タブで Fc を設定してください。\
                 非線形解析では Fc から曲げひび割れ・曲げ降伏・せん断終局の各耐力を算定します。",
                data.id.0, mat.name, fc
            )),
            Some(_) => None,
        };
    }
    if mat.fy.is_none() && mat.fc.is_none() {
        return Some(format!(
            "部材 ID {} の材料「{}」に降伏強度 fy もコンクリート強度 Fc も設定されていません。\
             材料タブでいずれかを設定してください。\
             非線形解析では材料強度から部材の終局耐力を算定します。",
            data.id.0, mat.name
        ));
    }
    None
}

#[cfg(test)]
mod tests;
