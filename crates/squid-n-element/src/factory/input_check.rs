//! 非線形解析の入力チェック。
//!
//! - [`nonlinear_input_issues`] — 部材耐力を算定できない設定不備の列挙
//! - [`ensure_nonlinear_input`] — 不備があれば是正内容を示すエラーを返す

use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_core::section_shape::SectionShape;

/// エラーメッセージへ列挙する不備の最大件数（超過分は件数のみを示す）。
const MAX_LISTED: usize = 5;

/// ファイバー断面に鋼材領域（形鋼・鋼管・内蔵鉄骨・管壁）を持つ形状か。
fn shape_has_steel_fiber_region(shape: &SectionShape) -> bool {
    !matches!(
        shape,
        SectionShape::RcRect { .. } | SectionShape::RcCircle { .. } | SectionShape::RcWall { .. }
    )
}

/// 非線形解析で部材耐力を算定できない設定不備を列挙する（是正内容を示す日本語文）。
///
/// 検査対象は、非線形解析で降伏を扱う要素に限る:
/// - 耐震壁（`Wall`）: 面内せん断の終局強度 Qu
///   （[`crate::wall::wall_element::WallElement::wall_shear_capacity_issue`]）
/// - 線材（`Beam` / `Fiber` / `MultiSpring`）: 曲げ・せん断の終局耐力に要する材料強度
///
/// 弾性としてモデル化することが仕様である要素（`Shell` / `PanelZone` / `NodalSpring`、
/// および特性を専用の属性で持つ `Isolator` / `Damper` / `Brace`）は対象外。
pub fn nonlinear_input_issues(model: &Model) -> Vec<String> {
    let mut issues = Vec::new();
    for elem in &model.elements {
        let issue = match elem.kind {
            ElementKind::Wall => {
                crate::wall::wall_element::WallElement::wall_shear_capacity_issue(elem, model)
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

/// 材料の区分が断面形状と矛盾する場合に、その内容を返す。
/// 検出するのは次の 2 つに絞る。
///
/// - 配筋を持つ断面（RC 矩形・RC 円形）に鋼材の材料が付いている。
/// - 線材の材料に鉄筋が割り当てられている。
fn category_mismatch_issue(
    data: &ElementData,
    sec: Option<&squid_n_core::model::Section>,
    mat: &squid_n_core::model::Material,
) -> Option<String> {
    use squid_n_core::model::MaterialCategory;
    if mat.category == MaterialCategory::Rebar {
        return Some(format!(
            "部材 ID {} の材料「{}」は区分が鉄筋です。\
             線材の材料には鋼材またはコンクリートを割り当ててください。\
             RC 断面の配筋は断面タブで主筋の材質として設定します。",
            data.id.0, mat.name
        ));
    }
    let has_rebar = sec.and_then(|s| s.shape.as_ref()).is_some_and(|s| {
        matches!(
            s,
            SectionShape::RcRect { .. } | SectionShape::RcCircle { .. }
        )
    });
    if has_rebar && mat.category == MaterialCategory::Steel {
        return Some(format!(
            "部材 ID {} は配筋を持つ RC 断面ですが、材料「{}」の区分が鋼材です。\
             材料タブで区分をコンクリートに変更してください。\
             区分は構造種別の判定に用い、鋼材のまま検定すると耐力を過大評価します。",
            data.id.0, mat.name
        ));
    }
    None
}

/// 線材の終局耐力を算定できない設定不備があれば、その内容を返す。
///
/// 検出する不備:
/// - 材料が設定されていない。
/// - コンクリート系断面（RC/SRC/CFT）なのに材料の Fc が未設定または 0 以下。
/// - 断面形状未設定の部材で、材料に正の fy も正の Fc もない。
fn member_strength_issue(data: &ElementData, model: &Model) -> Option<String> {
    let Some(sec) = data.section.and_then(|sid| model.sections.get(sid.index())) else {
        return Some(format!(
            "部材 ID {} に断面が設定されていません。\
             部材タブで断面を割り当ててください。\
             非線形解析では断面諸元から部材の剛性・終局耐力を算定します。",
            data.id.0
        ));
    };
    let sec = Some(sec);
    let Some(mat) = model.element_material(data) else {
        return Some(format!(
            "部材 ID {} に材料が設定されていません。\
             材料タブで材料を割り当ててください。\
             非線形解析では材料強度から部材の終局耐力を算定します。",
            data.id.0
        ));
    };
    if let Some(msg) = category_mismatch_issue(data, sec, mat) {
        return Some(msg);
    }
    let is_concrete = sec
        .and_then(|s| s.shape.as_ref())
        .is_some_and(|s| s.is_concrete_like());
    if is_concrete {
        match mat.fc {
            None => {
                return Some(format!(
                    "部材 ID {} はコンクリート系断面ですが、材料「{}」にコンクリート強度 Fc が\
                     設定されていません。材料タブで Fc を設定してください。\
                     非線形解析では Fc から曲げひび割れ・曲げ降伏・せん断終局の各耐力を算定します。",
                    data.id.0, mat.name
                ));
            }
            Some(fc) if fc <= 0.0 => {
                return Some(format!(
                    "部材 ID {} の材料「{}」のコンクリート強度 Fc が {} で 0 以下です。\
                     材料タブで Fc を設定してください。\
                     非線形解析では Fc から曲げひび割れ・曲げ降伏・せん断終局の各耐力を算定します。",
                    data.id.0, mat.name, fc
                ));
            }
            Some(_) => {}
        }
        let rebar = sec.and_then(|s| s.shape.as_ref()).and_then(|s| s.rebar());
        if rebar.is_some()
            && squid_n_core::material_grade::rebar_yield_strength(
                model.element_rebar_material(data),
            )
            .is_none()
        {
            return Some(format!(
                "部材 ID {} の断面に主筋の材料が割り当てられていません。\
                 断面タブで主筋の材料（SD295A・SD345 等）を割り当ててください。\
                 非線形解析では主筋の降伏強度 σy から曲げ降伏耐力を算定します。",
                data.id.0
            ));
        }
        let fiber_shape = sec.and_then(|s| s.shape.as_ref());
        if fiber_shape.is_some_and(shape_has_steel_fiber_region)
            && !crate::frame::fiber::resolve_steel_fiber_fy(
                fiber_shape,
                model.element_steel_material(data),
                mat.fy,
            )
            .is_some_and(|fy| fy > 0.0)
        {
            return Some(format!(
                "部材 ID {} の断面は鋼材領域（内蔵鉄骨・鋼管）を含みますが、鋼材の降伏強度を\
                 解決できません。断面へ内蔵鉄骨の材料を割り当てるか、材料「{}」の fy を\
                 設定してください。ファイバー断面は鋼材の降伏進展を追うため必須です。",
                data.id.0, mat.name
            ));
        }
        return None;
    }
    if sec.and_then(|s| s.shape.as_ref()).is_some() {
        if !mat.fy.is_some_and(|fy| fy > 0.0) {
            return Some(format!(
                "部材 ID {} は鋼材断面ですが、材料「{}」に降伏強度 fy が設定されていません。\
                 材料タブで fy を設定してください。\
                 ファイバー断面は鋼材の降伏進展を追うため fy が必須です。",
                data.id.0, mat.name
            ));
        }
        return None;
    }
    match mat.fc {
        Some(fc) if fc > 0.0 => None,
        Some(fc) => Some(format!(
            "部材 ID {} の材料「{}」のコンクリート強度 Fc が {} で 0 以下です。\
             材料タブで Fc を設定してください。\
             非線形解析では Fc から部材の終局耐力を算定します。",
            data.id.0, mat.name, fc
        )),
        None if mat.fy.is_some_and(|fy| fy > 0.0) => None,
        None => Some(format!(
            "部材 ID {} の材料「{}」に正の降伏強度 fy もコンクリート強度 Fc も設定されていません。\
             材料タブでいずれかを設定してください。\
             非線形解析では材料強度から部材の終局耐力を算定します。",
            data.id.0, mat.name
        )),
    }
}

#[cfg(test)]
mod tests;
