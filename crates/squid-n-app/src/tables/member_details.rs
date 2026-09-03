//! 部材の付帯情報（`Model.member_detail_attrs` = `MemberDetailAttr`: ハンチ・継手
//! 位置）の編集 UI。対象は `ElementKind::Beam` の部材のみ（本ソフトでは柱も梁も
//! `ElementKind::Beam` で表現するフレーム部材のため、実質すべての一般部材が対象）。
//! 編集は `squid_n_edit::{SetMemberDetailAttr, RemoveMemberDetailAttr}` 経由
//! （undo 対応）。空（ハンチなし・継手なし）を適用した場合は削除として扱う
//! （[`MemberDetailAttr::is_empty`]）。
//!
//! ここで設定した付帯情報は剛性・応力解析には影響せず、断面算定の検定位置
//! 追加（`squid_n_app::app::design_positions`）と `squid_n_element` の評価断面
//! 追加に用いられる（§6.2.3「位置はユーザが追加・変更可能」）。

use crate::app::App;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementKind, Haunch, JointKind, MemberDetailAttr, MemberJoint};
use squid_n_edit::{RemoveMemberDetailAttr, SetMemberDetailAttr};

/// 部材付帯情報フォームのドラフト状態（GUI 専用）。
/// 対象部材を選択すると `synced_for` の部材の現在値でバッファを初期化し、
/// 「適用」で `SetMemberDetailAttr`（空なら `RemoveMemberDetailAttr`）を発行する。
#[derive(Clone, Debug, Default)]
pub struct MemberDetailDraft {
    /// 編集対象の部材要素。
    pub elem: Option<ElemId>,
    /// バッファを初期化した対象（`elem` と異なれば model 値で再同期する）。
    pub synced_for: Option<ElemId>,
    /// i 端ハンチの有無。
    pub haunch_i_enabled: bool,
    /// i 端ハンチ長 [mm] の入力バッファ。
    pub haunch_i_length: String,
    /// i 端ハンチのせい増分 [mm] の入力バッファ（空欄可、既定 0）。
    pub haunch_i_depth: String,
    /// i 端ハンチの幅増分 [mm] の入力バッファ（空欄可、既定 0）。
    pub haunch_i_width: String,
    /// j 端ハンチの有無。
    pub haunch_j_enabled: bool,
    /// j 端ハンチ長 [mm] の入力バッファ。
    pub haunch_j_length: String,
    /// j 端ハンチのせい増分 [mm] の入力バッファ（空欄可、既定 0）。
    pub haunch_j_depth: String,
    /// j 端ハンチの幅増分 [mm] の入力バッファ（空欄可、既定 0）。
    pub haunch_j_width: String,
    /// 継手一覧の入力バッファ（1行1件または「,」区切り、`距離` または
    /// `距離/種別`。種別省略時は現場継手）。
    pub joints: String,
}

/// 継手種別のラベル（往復変換に使う固定表記）。
fn joint_kind_label(kind: JointKind) -> &'static str {
    match kind {
        JointKind::Site => "現場",
        JointKind::Shop => "工場",
    }
}

/// 継手種別を表す文字列（`現場`/`工場`/`site`/`shop`、大小文字・前後空白を許容）を
/// パースする。空文字列は既定（現場継手）とする。
fn parse_joint_kind(s: &str) -> Result<JointKind, String> {
    let s = s.trim();
    if s.is_empty() || s.eq_ignore_ascii_case("現場") || s.eq_ignore_ascii_case("site") {
        Ok(JointKind::Site)
    } else if s.eq_ignore_ascii_case("工場") || s.eq_ignore_ascii_case("shop") {
        Ok(JointKind::Shop)
    } else {
        Err(format!(
            "不正な継手種別「{s}」: '現場' または '工場' を指定してください"
        ))
    }
}

/// 継手1件分の入力（`距離` または `距離/種別`）をパースする（egui 非依存の純関数）。
fn parse_single_joint(entry: &str) -> Result<MemberJoint, String> {
    let (dist_str, kind_str) = match entry.split_once('/') {
        Some((d, k)) => (d, Some(k)),
        None => (entry, None),
    };
    let distance: f64 = dist_str
        .trim()
        .parse()
        .map_err(|_| format!("不正な継手位置「{}」: 数値ではありません", dist_str.trim()))?;
    if distance <= 0.0 {
        return Err(format!(
            "継手位置「{entry}」: 距離(始端節点芯から)は正の値で入力してください"
        ));
    }
    let kind = parse_joint_kind(kind_str.unwrap_or(""))?;
    Ok(MemberJoint { distance, kind })
}

/// 継手入力バッファ（1行1件または「,」区切り、`距離` または `距離/種別`）を
/// パースする（egui 非依存の純関数。`wall_plates::parse_openings` の流儀）。
pub fn parse_joints(s: &str) -> Result<Vec<MemberJoint>, String> {
    let normalized = s.replace('\n', ",");
    normalized
        .split(',')
        .map(str::trim)
        .filter(|t| !t.is_empty())
        .map(parse_single_joint)
        .collect()
}

/// 継手リストを入力バッファ書式（1行1件、`距離/種別`）へ整形する
/// （`parse_joints` の逆変換）。既存値をフォームへ読み込む際に使用する。
pub fn format_joints(joints: &[MemberJoint]) -> String {
    joints
        .iter()
        .map(|j| format!("{}/{}", j.distance, joint_kind_label(j.kind)))
        .collect::<Vec<_>>()
        .join("\n")
}

/// ハンチ1端分の入力（有効フラグ・長さ・せい増分・幅増分）をパースする
/// （egui 非依存の純関数）。無効なら `Ok(None)`。長さは正の値が必須、
/// せい増分・幅増分は空欄なら 0 とする。
pub fn parse_haunch(
    enabled: bool,
    length: &str,
    depth_increase: &str,
    width_increase: &str,
) -> Result<Option<Haunch>, String> {
    if !enabled {
        return Ok(None);
    }
    let length: f64 = length
        .trim()
        .parse()
        .map_err(|_| format!("不正なハンチ長「{}」: 数値ではありません", length.trim()))?;
    if length <= 0.0 {
        return Err("ハンチ長は正の値で入力してください".to_string());
    }
    let parse_opt = |s: &str, label: &str| -> Result<f64, String> {
        let s = s.trim();
        if s.is_empty() {
            Ok(0.0)
        } else {
            s.parse()
                .map_err(|_| format!("不正な{label}「{s}」: 数値ではありません"))
        }
    };
    let depth_increase = parse_opt(depth_increase, "せい増分")?;
    let width_increase = parse_opt(width_increase, "幅増分")?;
    Ok(Some(Haunch {
        length,
        depth_increase,
        width_increase,
    }))
}

/// ハンチの表示文字列（一覧行用）。`None` は「なし」。
fn haunch_desc(h: &Option<Haunch>) -> String {
    match h {
        Some(h) => format!(
            "長さ{:.0} せい+{:.0} 幅+{:.0}",
            h.length, h.depth_increase, h.width_increase
        ),
        None => "なし".to_string(),
    }
}

pub fn member_details_table(ui: &mut egui::Ui, app: &mut App) {
    ui.label(
        "部材(柱・梁)の付帯情報(ハンチ・継手位置)を設定します。剛性・応力解析には\
         影響せず、断面算定の検定位置の追加(ハンチ端・継手位置)と数量拾いに用います。",
    );
    ui.separator();

    let target_elems: Vec<ElemId> = app
        .core
        .model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Beam)
        .map(|e| e.id)
        .collect();

    if target_elems.is_empty() {
        ui.label("対象となる部材(柱・梁)がありません。");
        return;
    }

    // ── 既存の付帯情報一覧 ─────────────────────────────────
    let mut pending_remove: Option<ElemId> = None;
    let mut pending_edit: Option<MemberDetailAttr> = None;
    if app.core.model.member_detail_attrs.is_empty() {
        ui.label(
            "設定済みの付帯情報はありません(未設定の部材はハンチ・継手なしとして扱われます)。",
        );
    } else {
        for attr in &app.core.model.member_detail_attrs {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "部材#{}: i端ハンチ[{}] / j端ハンチ[{}] / 継手{}箇所",
                    attr.elem.0,
                    haunch_desc(&attr.haunch_i),
                    haunch_desc(&attr.haunch_j),
                    attr.joints.len(),
                ));
                if ui
                    .button("✏")
                    .on_hover_text("フォームへ読み込んで編集")
                    .clicked()
                {
                    pending_edit = Some(attr.clone());
                }
                if ui.button("🗑").on_hover_text("この付帯情報を削除").clicked() {
                    pending_remove = Some(attr.elem);
                }
            });
        }
    }
    if let Some(attr) = pending_edit {
        app.ui.scoped.member_detail_draft.elem = Some(attr.elem);
        app.ui.scoped.member_detail_draft.synced_for = Some(attr.elem);
        app.ui.scoped.member_detail_draft.haunch_i_enabled = attr.haunch_i.is_some();
        let (li, di, wi) = attr
            .haunch_i
            .map(|h| {
                (
                    format!("{:.0}", h.length),
                    format!("{:.0}", h.depth_increase),
                    format!("{:.0}", h.width_increase),
                )
            })
            .unwrap_or_default();
        app.ui.scoped.member_detail_draft.haunch_i_length = li;
        app.ui.scoped.member_detail_draft.haunch_i_depth = di;
        app.ui.scoped.member_detail_draft.haunch_i_width = wi;
        app.ui.scoped.member_detail_draft.haunch_j_enabled = attr.haunch_j.is_some();
        let (lj, dj, wj) = attr
            .haunch_j
            .map(|h| {
                (
                    format!("{:.0}", h.length),
                    format!("{:.0}", h.depth_increase),
                    format!("{:.0}", h.width_increase),
                )
            })
            .unwrap_or_default();
        app.ui.scoped.member_detail_draft.haunch_j_length = lj;
        app.ui.scoped.member_detail_draft.haunch_j_depth = dj;
        app.ui.scoped.member_detail_draft.haunch_j_width = wj;
        app.ui.scoped.member_detail_draft.joints = format_joints(&attr.joints);
    }
    if let Some(elem) = pending_remove {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(RemoveMemberDetailAttr { elem }),
        );
        app.core.scoped.staleness.mark_edited();
    }

    ui.separator();
    ui.strong("付帯情報を設定");

    // 対象部材の選択(変更時に model 値でバッファを再同期)
    ui.horizontal(|ui| {
        ui.label("対象部材:");
        let text = app
            .ui
            .scoped
            .member_detail_draft
            .elem
            .map(|e| format!("部材#{}", e.0))
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("member_detail_elem")
            .selected_text(text)
            .show_ui(ui, |ui| {
                for &eid in &target_elems {
                    if ui
                        .selectable_label(
                            app.ui.scoped.member_detail_draft.elem == Some(eid),
                            format!("部材#{}", eid.0),
                        )
                        .clicked()
                    {
                        app.ui.scoped.member_detail_draft.elem = Some(eid);
                    }
                }
            });
    });
    if app.ui.scoped.member_detail_draft.elem != app.ui.scoped.member_detail_draft.synced_for {
        if let Some(eid) = app.ui.scoped.member_detail_draft.elem {
            let existing = app.core.model.member_detail(eid).cloned();
            let (haunch_i, haunch_j, joints) = existing
                .map(|a| (a.haunch_i, a.haunch_j, a.joints))
                .unwrap_or((None, None, Vec::new()));
            app.ui.scoped.member_detail_draft.haunch_i_enabled = haunch_i.is_some();
            let (li, di, wi) = haunch_i
                .map(|h| {
                    (
                        format!("{:.0}", h.length),
                        format!("{:.0}", h.depth_increase),
                        format!("{:.0}", h.width_increase),
                    )
                })
                .unwrap_or_default();
            app.ui.scoped.member_detail_draft.haunch_i_length = li;
            app.ui.scoped.member_detail_draft.haunch_i_depth = di;
            app.ui.scoped.member_detail_draft.haunch_i_width = wi;
            app.ui.scoped.member_detail_draft.haunch_j_enabled = haunch_j.is_some();
            let (lj, dj, wj) = haunch_j
                .map(|h| {
                    (
                        format!("{:.0}", h.length),
                        format!("{:.0}", h.depth_increase),
                        format!("{:.0}", h.width_increase),
                    )
                })
                .unwrap_or_default();
            app.ui.scoped.member_detail_draft.haunch_j_length = lj;
            app.ui.scoped.member_detail_draft.haunch_j_depth = dj;
            app.ui.scoped.member_detail_draft.haunch_j_width = wj;
            app.ui.scoped.member_detail_draft.joints = format_joints(&joints);
            app.ui.scoped.member_detail_draft.synced_for = Some(eid);
        }
    }

    ui.horizontal(|ui| {
        ui.checkbox(
            &mut app.ui.scoped.member_detail_draft.haunch_i_enabled,
            "i端ハンチ",
        );
        ui.add_enabled_ui(app.ui.scoped.member_detail_draft.haunch_i_enabled, |ui| {
            ui.label("長さ[mm]:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.member_detail_draft.haunch_i_length)
                    .desired_width(70.0),
            );
            ui.label("せい増分[mm]:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.member_detail_draft.haunch_i_depth)
                    .desired_width(70.0),
            );
            ui.label("幅増分[mm]:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.member_detail_draft.haunch_i_width)
                    .desired_width(70.0),
            );
        });
    });
    ui.horizontal(|ui| {
        ui.checkbox(
            &mut app.ui.scoped.member_detail_draft.haunch_j_enabled,
            "j端ハンチ",
        );
        ui.add_enabled_ui(app.ui.scoped.member_detail_draft.haunch_j_enabled, |ui| {
            ui.label("長さ[mm]:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.member_detail_draft.haunch_j_length)
                    .desired_width(70.0),
            );
            ui.label("せい増分[mm]:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.member_detail_draft.haunch_j_depth)
                    .desired_width(70.0),
            );
            ui.label("幅増分[mm]:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.member_detail_draft.haunch_j_width)
                    .desired_width(70.0),
            );
        });
    });

    ui.label("継手一覧(始端節点芯からの距離[mm]。任意で「/現場」「/工場」を付記):");
    ui.add(
        egui::TextEdit::multiline(&mut app.ui.scoped.member_detail_draft.joints)
            .desired_rows(3)
            .desired_width(320.0)
            .hint_text(
                "距離 または 距離/種別\n\
                 複数継手は改行または「,」区切りで入力\n\
                 例: 1000/現場, 3000/工場",
            ),
    )
    .on_hover_text(
        "1行1件または「,」区切りで '距離' もしくは種別付き '距離/現場'・'距離/工場' を\
         入力します。種別省略時は現場継手として扱います。",
    );

    let parsed_haunch_i = parse_haunch(
        app.ui.scoped.member_detail_draft.haunch_i_enabled,
        &app.ui.scoped.member_detail_draft.haunch_i_length,
        &app.ui.scoped.member_detail_draft.haunch_i_depth,
        &app.ui.scoped.member_detail_draft.haunch_i_width,
    );
    let parsed_haunch_j = parse_haunch(
        app.ui.scoped.member_detail_draft.haunch_j_enabled,
        &app.ui.scoped.member_detail_draft.haunch_j_length,
        &app.ui.scoped.member_detail_draft.haunch_j_depth,
        &app.ui.scoped.member_detail_draft.haunch_j_width,
    );
    let parsed_joints = parse_joints(&app.ui.scoped.member_detail_draft.joints);
    for (label, r) in [
        ("i端ハンチ", &parsed_haunch_i),
        ("j端ハンチ", &parsed_haunch_j),
    ] {
        if let Err(e) = r {
            ui.colored_label(crate::theme::ERROR_RED, format!("{label}の書式エラー: {e}"));
        }
    }
    if let Err(e) = &parsed_joints {
        ui.colored_label(crate::theme::ERROR_RED, format!("継手の書式エラー: {e}"));
    }

    let can_apply = app.ui.scoped.member_detail_draft.elem.is_some()
        && parsed_haunch_i.is_ok()
        && parsed_haunch_j.is_ok()
        && parsed_joints.is_ok();
    ui.horizontal(|ui| {
        if ui
            .add_enabled(can_apply, egui::Button::new("✔ 適用"))
            .on_hover_text(
                "選択した部材にハンチ・継手位置を設定します(すべて空の場合は削除します。undo可)",
            )
            .clicked()
        {
            if let (Some(elem), Ok(haunch_i), Ok(haunch_j), Ok(joints)) = (
                app.ui.scoped.member_detail_draft.elem,
                parsed_haunch_i,
                parsed_haunch_j,
                parsed_joints,
            ) {
                let attr = MemberDetailAttr {
                    elem,
                    haunch_i,
                    haunch_j,
                    joints,
                };
                if attr.is_empty() {
                    app.core.scoped.undo.run(
                        &mut app.core.model,
                        Box::new(RemoveMemberDetailAttr { elem }),
                    );
                } else {
                    app.core
                        .scoped
                        .undo
                        .run(&mut app.core.model, Box::new(SetMemberDetailAttr { attr }));
                }
                app.core.scoped.staleness.mark_edited();
            }
        }
        if app.ui.scoped.member_detail_draft.elem.is_some()
            && ui
                .button("🗑 この部材の付帯情報を削除")
                .on_hover_text("選択した部材の付帯情報を削除します(undo可)")
                .clicked()
        {
            if let Some(elem) = app.ui.scoped.member_detail_draft.elem {
                app.core.scoped.undo.run(
                    &mut app.core.model,
                    Box::new(RemoveMemberDetailAttr { elem }),
                );
                app.core.scoped.staleness.mark_edited();
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 空文字列は「継手なし」を表し、空の Vec になること。
    #[test]
    fn test_parse_joints_empty_is_empty_vec() {
        assert_eq!(parse_joints("").unwrap(), Vec::new());
        assert_eq!(parse_joints("   ").unwrap(), Vec::new());
    }

    /// 種別省略時は現場継手として扱われること。
    #[test]
    fn test_parse_joints_defaults_to_site() {
        let joints = parse_joints("1000").unwrap();
        assert_eq!(
            joints,
            vec![MemberJoint {
                distance: 1000.0,
                kind: JointKind::Site,
            }]
        );
    }

    /// 「,」区切り・種別付きが正しくパースされること。
    #[test]
    fn test_parse_joints_comma_separated_with_kind() {
        let joints = parse_joints("1000/現場, 3000/工場").unwrap();
        assert_eq!(
            joints,
            vec![
                MemberJoint {
                    distance: 1000.0,
                    kind: JointKind::Site,
                },
                MemberJoint {
                    distance: 3000.0,
                    kind: JointKind::Shop,
                },
            ]
        );
    }

    /// 改行区切り・英語表記('site'/'shop'、大小文字を問わない)でもパースできること。
    #[test]
    fn test_parse_joints_newline_separated_english_kind() {
        let joints = parse_joints("1000/SITE\n3000/Shop").unwrap();
        assert_eq!(
            joints,
            vec![
                MemberJoint {
                    distance: 1000.0,
                    kind: JointKind::Site,
                },
                MemberJoint {
                    distance: 3000.0,
                    kind: JointKind::Shop,
                },
            ]
        );
    }

    /// 距離が数値でない・0以下はエラーになること。
    #[test]
    fn test_parse_joints_rejects_invalid_distance() {
        assert!(parse_joints("abc").is_err());
        assert!(parse_joints("0").is_err());
        assert!(parse_joints("-100").is_err());
    }

    /// 不正な種別はエラーになること。
    #[test]
    fn test_parse_joints_rejects_invalid_kind() {
        let err = parse_joints("1000/不明").unwrap_err();
        assert!(err.contains("不明"), "err={err}");
    }

    /// format_joints は parse_joints の逆変換になっていること(往復一致)。
    #[test]
    fn test_format_joints_roundtrip() {
        let joints = vec![
            MemberJoint {
                distance: 1000.0,
                kind: JointKind::Site,
            },
            MemberJoint {
                distance: 3000.0,
                kind: JointKind::Shop,
            },
        ];
        let formatted = format_joints(&joints);
        assert_eq!(parse_joints(&formatted).unwrap(), joints);
    }

    /// 空リストは空文字列へ整形されること。
    #[test]
    fn test_format_joints_empty() {
        assert_eq!(format_joints(&[]), "");
    }

    /// ハンチ無効時は入力値によらず None になること。
    #[test]
    fn test_parse_haunch_disabled_is_none() {
        assert_eq!(parse_haunch(false, "700", "200", "0").unwrap(), None);
    }

    /// せい増分・幅増分が空欄なら 0 として扱われること。
    #[test]
    fn test_parse_haunch_optional_fields_default_zero() {
        let h = parse_haunch(true, "700", "", "").unwrap().unwrap();
        assert_eq!(
            h,
            Haunch {
                length: 700.0,
                depth_increase: 0.0,
                width_increase: 0.0,
            }
        );
    }

    /// 全フィールド指定時の正常系。
    #[test]
    fn test_parse_haunch_all_fields() {
        let h = parse_haunch(true, "700", "200", "50").unwrap().unwrap();
        assert_eq!(
            h,
            Haunch {
                length: 700.0,
                depth_increase: 200.0,
                width_increase: 50.0,
            }
        );
    }

    /// 長さが 0 以下・数値でない場合はエラーになること。
    #[test]
    fn test_parse_haunch_rejects_non_positive_or_invalid_length() {
        assert!(parse_haunch(true, "0", "0", "0").is_err());
        assert!(parse_haunch(true, "-10", "0", "0").is_err());
        assert!(parse_haunch(true, "abc", "0", "0").is_err());
    }

    /// せい増分・幅増分が数値でない場合はエラーになること。
    #[test]
    fn test_parse_haunch_rejects_non_numeric_optional_fields() {
        assert!(parse_haunch(true, "700", "abc", "0").is_err());
        assert!(parse_haunch(true, "700", "0", "abc").is_err());
    }
}
