use super::super::*;

impl App {
    /// T7: 解析結果の member_forces から検定結果を生成する。
    /// 危険断面位置（既定は柱フェイスと中央）の内力に対し、
    /// 材種・部材種別に応じた検定を適用する（令82条・各構造設計規準準拠）。
    /// 節点芯は剛域が有る場合は検定対象外。
    ///
    /// - 部材種別は部材軸の鉛直成分から判定（柱/梁/ブレース）。
    /// - せん断スパン比 M/(Q·d) 用の代表値は、モーメントが最大となる
    ///   検定位置の値を採用する方針で部材単位に求める。
    /// - 柱は軸力＋二軸曲げ（n, my, mz）を検定に渡す。
    /// - 検定器は構造種別（`squid_n_core::structure_kind`）で選択する。
    pub fn run_design_check(&mut self) {
        self.apply_rigid_zones_for_analysis();
        let Some(results) = &self.core.scoped.results else {
            return;
        };
        let expanded_storage;
        let design_model: &squid_n_core::model::Model =
            if squid_n_load::wall_expand::model_has_wall_plates_to_expand(&self.core.model) {
                let (expanded, _wall_index, _wall_report) =
                    squid_n_load::wall_expand::expand_wall_elements(&self.core.model);
                expanded_storage = expanded;
                &expanded_storage
            } else {
                &self.core.model
            };
        let is_seismic_combo = match self.core.scoped.last_static {
            Some(StaticKey::Combo(idx)) => results
                .combos
                .get(idx)
                .map(|(n, _)| {
                    let u = n.to_uppercase();
                    u.contains('K') || u.contains('E')
                })
                .unwrap_or(false),
            _ => false,
        };
        let gravity_long_owned = if is_seismic_combo && self.core.design_term == LoadTerm::Short {
            squid_n_job::sum_analyzed_gravity_member_forces(&self.core.model, |lc| {
                results
                    .statics
                    .iter()
                    .find(|(id, _)| *id == StaticCaseKey::User(lc))
                    .map(|(_, s)| s.member_forces.clone())
            })
        } else {
            None
        };
        let long_from_combo: Option<&Vec<(ElemId, squid_n_element::frame::beam::MemberForces)>> =
            if is_seismic_combo && self.core.design_term == LoadTerm::Short {
                results
                    .combos
                    .iter()
                    .find(|(n, _)| n == "DL + LL")
                    .or_else(|| {
                        results
                            .combos
                            .iter()
                            .find(|(n, _)| !squid_n_load::combo::is_short_term_combo(n))
                    })
                    .map(|(_, st)| &st.member_forces)
            } else {
                None
            };
        let long_member_forces: Option<&[(ElemId, squid_n_element::frame::beam::MemberForces)]> =
            gravity_long_owned
                .as_deref()
                .or(long_from_combo.map(|v| v.as_slice()));
        let group_overrides =
            squid_n_design_jp::beam_group_overrides(&self.core.model, &results.member_forces);
        let q0_by_elem = if long_member_forces.is_some() {
            squid_n_job::simple_beam_q0_by_gravity_cases(&self.core.model)
        } else {
            Default::default()
        };
        let report = squid_n_design_jp::run_member_design_checks(
            design_model,
            &results.member_forces,
            &results.panel_moments,
            &squid_n_design_jp::MemberDesignCheckOptions {
                term: self.core.design_term,
                rc_damage_control: self.core.analysis_cfg.rc_damage_control,
                bond_method: self.core.analysis_cfg.bond_method,
                qd_method: self.core.analysis_cfg.qd_method,
                long_member_forces,
                q_simple_by_elem: Some(&q0_by_elem),
                beam_group_overrides: Some(&group_overrides),
            },
        );
        let joint_checks = report
            .joint_checks
            .into_iter()
            .map(|(node, label, cr)| JointCheck {
                node,
                label,
                outcome: squid_n_design_jp::CheckOutcome::Checked(cr),
            })
            .collect();
        let (joist_checks, slab_checks) = self.floor_design_checks();

        let member_checks = group_member_checks(report.member_checks);

        if let Some(bundle) = self.core.scoped.results.as_mut() {
            bundle.member_checks = member_checks;
            bundle.joint_checks = joint_checks;
            bundle.joist_checks = joist_checks;
            bundle.slab_checks = slab_checks;
        }
    }

    /// 床の中での小梁・スラブ設計を算定する（`run_design_check` から呼ぶ）。
    ///
    /// - 二次部材小梁: 二次部材の反力の逐次伝達（[`squid_n_load::cascade`]）が求めた荷重
    ///   （床板分配の辺荷重・自重・架け側から渡された集中荷重）を単純梁または片持ち梁と
    ///   して検定する。
    /// - 実部材化された小梁（支持間に実 Beam がある）は全体 FEM で検定するため対象外。
    /// - 断面未割当・鋼以外の材料・分配荷重が無い・期待床板の欠落・カバー不足の二次部材は表に「未」として残す。
    /// - スラブ: 矩形スラブの短辺を設計スパンとし、一方向版として設計曲げモーメントと
    ///   必要鉄筋量を算定する（鋼小梁・SD295 鉄筋の既定値を用いる）。
    pub(crate) fn floor_design_checks(
        &self,
    ) -> (Vec<crate::app::JoistCheck>, Vec<crate::app::SlabCheck>) {
        use squid_n_core::model::LoadPurpose;
        use squid_n_design_jp::floor as fd;

        let mut joist_checks = Vec::new();
        let mut slab_checks = Vec::new();

        for slab in &self.core.model.slabs {
            let Some(thickness) = self.core.model.slab_plate_thickness(slab) else {
                continue;
            };
            let w = self.core.model.slab_intensity(slab, LoadPurpose::Floor);
            if slab.is_attached() {
                if let Some(span) = slab.attached_design_span() {
                    let r = fd::design_slab_oneway(
                        span,
                        w,
                        2.0,
                        thickness,
                        fd::SLAB_DEFAULT_COVER,
                        fd::REBAR_FT_LONG_SD295,
                        fd::SLAB_J_RATIO,
                    );
                    slab_checks.push((slab.id, r));
                }
            } else if let Some((lx, ly)) =
                squid_n_load::floor::slab_dimensions(&self.core.model, slab)
            {
                use squid_n_core::model::OneWayDir;
                let span = match slab.one_way() {
                    Some(OneWayDir::X) => lx,
                    Some(OneWayDir::Y) => ly,
                    None => lx.min(ly),
                };
                if span > 1e-9 {
                    let r = fd::design_slab_oneway(
                        span,
                        w,
                        8.0,
                        thickness,
                        fd::SLAB_DEFAULT_COVER,
                        fd::REBAR_FT_LONG_SD295,
                        fd::SLAB_J_RATIO,
                    );
                    slab_checks.push((slab.id, r));
                }
            }
        }

        self.design_secondary_joist_checks(&mut joist_checks);

        (joist_checks, slab_checks)
    }

    /// 領域内小梁および未割当の片持ち小梁を、二次部材の反力の逐次伝達
    /// （[`squid_n_load::cascade`]）が求めた荷重で検定する。
    ///
    /// 荷重は床板分配の辺荷重・自重・**架け側の二次部材から渡された集中荷重**の
    /// 重ね合わせである。**荷重同期（`squid-n-job::auto_loads`）と同じ経路を使う**
    /// （判定が 2 か所に分かれると解析と検定で荷重が食い違うため）。
    ///
    /// ただし**荷重の値そのものは一致しない**。共有するのは支持関係の判定と伝達の
    /// 手順であって、面荷重強度は用途ごとに違う。検定は床用（`LoadPurpose::Floor`。
    /// 固定＋床用積載）、荷重同期は固定荷重ケースと積載荷重ケースへ分けて解く。
    ///
    /// 断面未割当・鋼以外の材料・分配が足りないものは表に「未」として残す。
    /// 所属先がない小梁のうち、片持ち小梁以外は検定の対象にしない（表に「未」として
    /// 残す。片持ち小梁は取付き線の支持辺として荷重を受けるため検定する）。
    ///
    /// # 検定できない二次部材（表には「未」の行として残す）
    ///
    /// - **間柱**。壁版から受ける荷重は材軸方向の軸力であり、地震時には壁の面外
    ///   地震力による弱軸曲げも受ける。軸力・面外曲げのいずれも本検定は扱わない。
    /// - **剛床でない床の小梁**。分配 Span 検定は曲げ・せん断・たわみだけを見て
    ///   軸力を見ない。これは「その小梁が載る床面が 1 枚の剛体で、面内力を剛床が
    ///   処理している」ことを前提にしている。剛床でない床の小梁は面内力を負担する
    ///   ため前提が成り立たない（`Model::floor_region_on_single_diaphragm`）。
    ///
    /// 表から消すと検定されていないことに気づけないため、行は残す。
    fn design_secondary_joist_checks(&self, joist_checks: &mut Vec<crate::app::JoistCheck>) {
        use squid_n_core::model::{LoadPurpose, SecondaryMemberKind};
        use squid_n_design_jp::floor as fd;
        use squid_n_load::floor::{cantilever_extremes, simple_beam_extremes};

        let w_of =
            |s: &squid_n_core::model::Slab| self.core.model.slab_intensity(s, LoadPurpose::Floor);
        let transfer = squid_n_load::cascade::solve(&self.core.model, w_of, true);

        for sm in self.core.model.posts() {
            let Some((_, _, span)) = self.core.model.secondary_member_axis(sm) else {
                continue;
            };
            joist_checks.push((
                None,
                crate::app::JoistCheckTarget::SecondaryPost { member: sm.id },
                fd::joist_unchecked(span),
            ));
        }

        for sm in self.core.model.joists() {
            if sm.kind != SecondaryMemberKind::Joist {
                continue;
            }
            if self.core.model.secondary_member_materialized(sm) {
                continue;
            }
            let Some((_, _, span)) = self.core.model.secondary_member_axis(sm) else {
                continue;
            };

            let target = crate::app::JoistCheckTarget::SecondaryJoist { member: sm.id };
            let entry = transfer.members.get(&sm.id);
            let region = self.core.model.floor_region_of_joist(sm.id);
            let region_slab = region.and_then(|r| r.slab_ids.first().copied());
            let slab_id = region_slab.or_else(|| entry.and_then(|e| e.rep_slab_id));

            let on_single_diaphragm = match region {
                Some(r) => self.core.model.floor_region_on_single_diaphragm(r),
                None => sm.is_cantilever(),
            };
            if !on_single_diaphragm {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            }

            let Some(sid) = sm.section else {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            };
            let Some(sec) = self.core.model.sections.get(sid.index()) else {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            };
            let z = if sec.depth > 0.0 {
                sec.iy / (sec.depth / 2.0)
            } else {
                0.0
            };
            let mat = self.core.model.secondary_material(sm);
            let Some((e, ft)) = fd::joist_steel_e_and_ft(mat) else {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            };

            let Some(entry) = entry.filter(|e| e.distribution_ready) else {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            };
            let ex = if sm.is_cantilever() {
                cantilever_extremes(&entry.member_loads, span, e, sec.iy)
            } else {
                simple_beam_extremes(&entry.member_loads, span, e, sec.iy)
            };
            if ex.w_equiv <= 1e-9 && ex.m_max <= 1e-9 {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            }

            let r = fd::design_joist_from_forces(
                span,
                ex.w_equiv,
                ex.m_max,
                ex.q_max,
                ex.deflection,
                z,
                ft,
                fd::DEFLECTION_LIMIT_DENOM,
            );
            joist_checks.push((slab_id, target, r));
        }
    }
}
