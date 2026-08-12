//! 部材断面検定の共通オーケストレーション。
//!
//! GUI（`squid-n-app`）と MCP（`squid-n-mcp`）はいずれも、線形静的解析の
//! 部材内力に対して許容応力度検定を行う。検定式そのものは [`crate::DesignCheck`]
//! 各実装が担い、本モジュールは次の**配線**を一箇所に集約する。
//!
//! - 部材単位の [`crate::DesignCtx`] 組み立て（部材長・座屈長さ・せん断スパン比代表値等）
//! - 構造種別に応じた [`crate::checker_for`] による検定器選択
//! - 危険断面位置（[`crate::design_position`]）での断面検定ループ
//! - 節点単位検定（[`crate::joint_wiring`]）と PCa 水平接合面（[`crate::rc::horizontal_joint`]）
//!
//! 呼び出し側は解析結果（部材内力・パネルモーメント）と
//! [`MemberDesignCheckOptions`] を渡す。地震時短期 QD 割増・一本部材グループ合成・
//! 床設計など app 固有の前処理は呼び出し側で行い、本関数は与えられた
//! オプションをそのまま検定文脈へ反映する。

use std::collections::HashMap;

use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::Model;
use squid_n_element::beam::MemberForces;

use crate::design_position::{design_positions, is_near_design_position};
use crate::{
    beam_has_attached_slab, checker_for, BondMethod, CheckOutcome, CheckResult, DesignCheck,
    DesignCtx, LoadTerm, MemberForcesAt, MemberKind, QdMethod, SeismicQd,
};

/// 一本部材グループ合成値（断面検定の採用応力上書き用）。
///
/// `Model.beam_groups` から合成した部材長・端部/中央モーメント等を、
/// 所属する分割梁要素の検定文脈へ上書きする。合成ロジック自体は
/// 呼び出し側（現状 `squid-n-app`）が担う。
#[derive(Clone, Debug, PartialEq)]
pub struct BeamGroupContextOverride {
    /// 一本部材の全長 L [mm]（分割部材長の総和）。
    pub length: f64,
    /// 一本部材両端の強軸曲げ `(M_i端, M_j端)` [N·mm]。
    pub end_moments_z: Option<(f64, f64)>,
    /// 一本部材中央の強軸曲げ Mc [N·mm]。
    pub mid_moment_z: Option<f64>,
    /// グループ内 |Mz| 最大位置の `(|M|, |Q|)`（せん断スパン比の代表値）。
    pub shear_span: Option<(f64, f64)>,
    /// 一本部材の内法長（両外端の剛域控除後）[mm]。
    pub clear_length: f64,
}

/// 部材断面検定オーケストレーションの実行オプション。
#[derive(Clone, Debug, Default)]
pub struct MemberDesignCheckOptions<'a> {
    /// 検定条件（長期/短期）。
    pub term: LoadTerm,
    /// RC 短期許容せん断力で損傷制御式（2/3·α）を使うか。
    pub rc_damage_control: bool,
    /// RC 梁付着検定の方式（既定は 1999）。
    pub bond_method: BondMethod,
    /// 地震時短期 QD の決定方法（`long_member_forces` が与えられた場合のみ有効）。
    pub qd_method: QdMethod,
    /// 地震時短期 QD 用の長期（DL+LL 等）内力。None なら QD 割増なし。
    pub long_member_forces: Option<&'a [(ElemId, MemberForces)]>,
    /// 梁 QD1 用の単純梁せん断 Q0 [N]（部材 ID → 絶対値）。None なら QL で代替。
    pub q_simple_by_elem: Option<&'a HashMap<ElemId, f64>>,
    /// 一本部材グループの検定文脈上書き（梁のみ適用）。None なら部材単体の値を用いる。
    pub beam_group_overrides: Option<&'a HashMap<ElemId, BeamGroupContextOverride>>,
}

/// 部材断面検定オーケストレーションの結果。
#[derive(Clone, Debug, Default)]
pub struct MemberDesignCheckReport {
    /// 部材断面検定（危険断面位置・BRB・PCa 水平接合面を含む）。
    pub member_checks: Vec<(ElemId, f64, CheckOutcome)>,
    /// 節点単位検定（柱梁接合部・パネルゾーン・冷間成形耐力比・耐震壁等）。
    pub joint_checks: Vec<(NodeId, String, CheckResult)>,
}

/// 部材内力に対する許容応力度検定を一括実行する。
///
/// 部材ループ（DesignCtx 組み立て → 危険断面位置での検定）のほか、
/// PCa 水平接合面検定と節点単位検定も含む。床の小梁・スラブ設計は
/// 呼び出し側（`squid-n-app`）の責務とする。
pub fn run_member_design_checks(
    model: &Model,
    member_forces: &[(ElemId, MemberForces)],
    panel_moments: &[(NodeId, [f64; 2])],
    options: &MemberDesignCheckOptions<'_>,
) -> MemberDesignCheckReport {
    let mut elem_by_id: HashMap<ElemId, &squid_n_core::model::ElementData> =
        HashMap::with_capacity(model.elements.len());
    for e in &model.elements {
        elem_by_id.entry(e.id).or_insert(e);
    }

    let long_mf_by_id: Option<HashMap<ElemId, &MemberForces>> =
        options.long_member_forces.map(|list| {
            let mut m = HashMap::with_capacity(list.len());
            for (id, mf) in list {
                m.entry(*id).or_insert(mf);
            }
            m
        });

    let column_k_index = squid_n_core::adjacency::NodeAdjacency::build(model);
    let mut member_checks: Vec<(ElemId, f64, CheckOutcome)> = Vec::new();

    for (elem_id, mf) in member_forces {
        let Some(elem) = elem_by_id.get(elem_id).copied() else {
            continue;
        };
        let sec = elem
            .section
            .and_then(|sid| model.sections.get(sid.index()))
            .filter(|s| s.id == elem.section.unwrap());
        let mat = model.element_material(elem);
        let (Some(sec), Some(mat)) = (sec, mat) else {
            continue;
        };

        let kind = MemberKind::of_element(elem, model);
        let length = model.member_length(elem);
        let shear_span = mf
            .at
            .iter()
            .map(|(_, f)| (f[5].abs(), f[1].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let shear_span_y = mf
            .at
            .iter()
            .map(|(_, f)| (f[4].abs(), f[2].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let m_at = |target: f64| {
            mf.at
                .iter()
                .find(|(p, _)| (p - target).abs() < 1e-9)
                .map(|(_, f)| f[5])
        };
        let end_moments_z = match (m_at(0.0), m_at(1.0)) {
            (Some(a), Some(b)) => Some((a, b)),
            _ => None,
        };
        let steel_attr = model
            .steel_design_attrs
            .iter()
            .find(|a| a.elem == *elem_id)
            .cloned();
        let (lk_y_auto, lk_z_auto) = if kind == MemberKind::Column {
            match crate::steel::buckling::steel_column_k_axes_with_index(
                model,
                &column_k_index,
                elem,
            ) {
                Some((k_y, k_z)) => (Some(k_y * length), Some(k_z * length)),
                None => (None, None),
            }
        } else {
            (None, None)
        };
        let lk_y = steel_attr
            .as_ref()
            .and_then(|a| a.lk_y_direct)
            .or(lk_y_auto);
        let lk_z = steel_attr
            .as_ref()
            .and_then(|a| a.lk_z_direct)
            .or(lk_z_auto);

        let group = if kind == MemberKind::Beam {
            options.beam_group_overrides.and_then(|m| m.get(elem_id))
        } else {
            None
        };
        let (length, shear_span, end_moments_z, mid_moment_z) = match group {
            Some(g) => (g.length, g.shear_span, g.end_moments_z, g.mid_moment_z),
            None => (length, shear_span, end_moments_z, m_at(0.5)),
        };

        let face_sum = elem.rigid_zone.face_i_or_zero() + elem.rigid_zone.face_j_or_zero();
        let clear_span = match group {
            Some(g) => g.clear_length,
            None if length - face_sum > 0.0 => length - face_sum,
            None => length,
        };

        let seismic_qd = long_mf_by_id
            .as_ref()
            .and_then(|map| map.get(elem_id))
            .map(|mf_long| SeismicQd {
                long_at: mf_long.at.clone(),
                // 割増係数 n（柱は 1.5 以上）。梁・柱とも 1.5。QD2 用。
                n_factor: 1.5,
                // メカニズム側 n_mech（マニュアル n2）。ルート連動前は 1.0。
                n_mechanism: 1.0,
                q_simple: options
                    .q_simple_by_elem
                    .and_then(|m| m.get(elem_id).copied()),
                clear_length: clear_span,
                method: options.qd_method,
            });

        // RC 柱のメカニズム ΣMy（梁降伏/柱降伏）。地震時短期かつ長期内力が
        // あるときだけ算定する。部分スリット壁の強制柱ヒンジはモデル未対応。
        let column_sum_my = if kind == MemberKind::Column && seismic_qd.is_some() {
            let n_at = |mf: &MemberForces, end: usize| {
                let target = if end == 0 { 0.0 } else { 1.0 };
                mf.at
                    .iter()
                    .min_by(|a, b| (a.0 - target).abs().total_cmp(&(b.0 - target).abs()))
                    .map(|(_, f)| f[0])
                    .unwrap_or(0.0)
            };
            let (n_combo_i, n_combo_j) = (n_at(mf, 0), n_at(mf, 1));
            let (n_long_i, n_long_j) = long_mf_by_id
                .as_ref()
                .and_then(|m| m.get(elem_id))
                .map(|mf_l| (n_at(mf_l, 0), n_at(mf_l, 1)))
                .unwrap_or((n_combo_i, n_combo_j));
            crate::rc::compute_column_mechanism_sum_my(
                model,
                &column_k_index,
                elem,
                n_long_i,
                n_long_j,
                n_combo_i,
                n_combo_j,
                1.0, // ルート連動前の n（ルート2-3 なら 2.0）
            )
        } else {
            None
        };

        let ctx = DesignCtx {
            rebar_material: model.element_rebar_material(elem).cloned(),
            shear_rebar_material: model.element_shear_rebar_material(elem).cloned(),
            steel_material: model.element_steel_material(elem).cloned(),
            term: options.term,
            kind,
            length,
            clear_length: Some(clear_span),
            lb: None,
            lk_y,
            lk_z,
            shear_span,
            shear_span_y,
            rc_damage_control: options.rc_damage_control,
            bond_method: options.bond_method,
            end_moments_z,
            mid_moment_z,
            seismic_qd,
            column_sum_my,
            beam_has_slab: kind == MemberKind::Beam && beam_has_attached_slab(model, elem),
            steel_attr,
            steel_fb_rule: Default::default(),
        };

        let checker: Box<dyn DesignCheck> = checker_for(
            squid_n_core::structure_kind::structure_kind_of(Some(sec), Some(mat.category)),
        );

        let positions = design_positions(elem, model, length);
        let brb_long = options.term == LoadTerm::Long;

        for (pos, forces) in &mf.at {
            if !is_near_design_position(*pos, &positions) {
                continue;
            }
            let mfa = MemberForcesAt {
                pos: *pos,
                n: forces[0],
                qy: forces[1],
                qz: forces[2],
                my: forces[4],
                mz: forces[5],
            };
            let outcome = if let Some(brb) = model.brb_attrs.iter().find(|a| a.elem == *elem_id) {
                CheckOutcome::Checked(crate::brb::brb_check(brb, mfa.n, length, brb_long))
            } else {
                checker.check(&mfa, sec, mat, &ctx)
            };
            member_checks.push((*elem_id, *pos, outcome));
        }
    }

    let mf_slices: Vec<(ElemId, crate::joint_wiring::ForcesAt)> = member_forces
        .iter()
        .map(|(id, mf)| (*id, mf.at.as_slice()))
        .collect();
    let long_slices: Option<Vec<(ElemId, crate::joint_wiring::ForcesAt)>> =
        options.long_member_forces.map(|list| {
            list.iter()
                .map(|(id, mf)| (*id, mf.at.as_slice()))
                .collect()
        });

    member_checks.extend(
        crate::rc::horizontal_joint::collect_pca_checks(
            model,
            &mf_slices,
            options.term == LoadTerm::Long,
        )
        .into_iter()
        .map(|(id, pos, cr)| (id, pos, CheckOutcome::Checked(cr))),
    );

    let joint_checks = crate::joint_wiring::collect_joint_checks_with_long(
        model,
        &mf_slices,
        long_slices.as_deref(),
        panel_moments,
        options.term,
    );

    MemberDesignCheckReport {
        member_checks,
        joint_checks,
    }
}
