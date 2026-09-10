use super::super::*;
use squid_n_core::units::to_display::length_m;

impl App {
    /// 保有水平耐力の層別判定を行う。前提データが不足していれば Err(案内文)。
    ///
    /// 戻り値の第 2 要素は層ごとに採用された部材ランク（`design_rank_auto` が
    /// true の場合は幅厚比からの自動判定、算定できなかった層は `design_rank`
    /// へフォールバック。false の場合は全層 `design_rank`）。
    #[allow(clippy::type_complexity)]
    pub fn compute_holding_capacity(
        &mut self,
    ) -> Result<
        (
            squid_n_design_jp::secondary::holding_capacity::HoldingCapacityResult,
            Vec<squid_n_design_jp::secondary::holding_capacity::MemberRank>,
        ),
        String,
    > {
        use squid_n_core::section_shape::SectionShape;
        use squid_n_design_jp::secondary::ds_group::{
            ds_rc, ds_steel, member_group, rank_index_for_group, rc_beam_type, rc_column_type,
            rc_wall_shear_brittle, rc_wall_tau_over_fc, rc_wall_type, steel_brace_type, GroupType,
        };
        use squid_n_design_jp::secondary::holding_capacity::{
            check_holding_capacity, qud_by_story, MemberRank,
        };
        use squid_n_design_jp::secondary::member_rank::worst_rank;
        use squid_n_design_jp::secondary::rc_capacity::{
            rc_column_mu_simple, rc_qmu_simple, rc_qsu_simple,
        };
        use squid_n_design_jp::steel_f_value_prefix;
        use squid_n_solver::nonlinear::pushover::MechanismType;

        self.apply_rigid_zones_for_analysis();

        if self.core.model.stories.is_empty() {
            return Err(
                "階が未定義です。解析タブの「準備計算 実行」を行ってください。".to_string(),
            );
        }
        let view_dir = self.core.scoped.pushover_view_dir;
        let po = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.pushover_for_dir(view_dir).or(r.pushover.as_ref()))
            .ok_or_else(|| {
                "増分解析未実行です。解析タブから増分解析を実行してください。".to_string()
            })?;
        let st = self.current_static().ok_or_else(|| {
            "静的解析結果がありません。地震静的(Ai)を実行してください。".to_string()
        })?;

        let ctx = crate::summary::metrics_ctx_from_results(self.core.scoped.results.as_ref());
        let metrics = crate::summary::compute_story_metrics_with(
            &self.core.model,
            &st.disp,
            self.core.analysis_cfg.seismic_dir,
            &ctx,
        );

        let layers = self.core.model.layers();
        let weights: Vec<f64> = layers.iter().map(|l| l.weight.unwrap_or(0.0)).collect();
        if weights.iter().any(|w| *w <= 0.0) {
            return Err(
                "地震重量が未設定です。解析タブの「準備計算 実行」を行ってください。".to_string(),
            );
        }

        let t = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.period.first().copied())
            .unwrap_or_else(|| {
                let height_m = length_m(squid_n_solver::statics::analysis::building_height_mm(
                    &self.core.model,
                ));
                let steel_ratio =
                    squid_n_solver::statics::analysis::steel_height_ratio(&self.core.model);
                squid_n_load::ai::approx_t(height_m, steel_ratio)
            });
        let rt = squid_n_load::ai::rt(t, squid_n_load::ai::tc_of(self.core.analysis_cfg.soil));
        let qud = qud_by_story(&weights, self.core.analysis_cfg.z, rt, t);

        let n_stories = weights.len();

        let resp_by_elem: std::collections::HashMap<
            ElemId,
            squid_n_solver::nonlinear::pushover::PushoverMemberResponse,
        > = po.member_response.iter().map(|r| (r.elem, *r)).collect();
        let shear_yield_elems: std::collections::HashSet<ElemId> =
            po.shear_yields.iter().map(|s| s.elem).collect();
        let story_qu: Vec<f64> = (0..n_stories)
            .map(|i| {
                po.capacity_curve
                    .iter()
                    .filter_map(|p| p.story_shear.get(i).copied())
                    .fold(0.0_f64, f64::max)
            })
            .collect();
        let mut cb_members: Vec<Vec<(u8, f64)>> = vec![Vec::new(); n_stories];
        let mut wall_members: Vec<Vec<(u8, f64)>> = vec![Vec::new(); n_stories];
        let mut wall_horizontal: Vec<f64> = vec![0.0; n_stories];

        let (story_ranks, member_ranks): (Vec<MemberRank>, Vec<(ElemId, MemberRank)>) = if self
            .core
            .design_rank_auto
        {
            let mut per_story: Vec<Vec<MemberRank>> = vec![Vec::new(); n_stories];
            let mut computed: Vec<(ElemId, MemberRank)> = Vec::new();
            let gravity_lc = gravity_cases_for_seismic_weight(&self.core.model)
                .first()
                .copied();
            let expanded_storage;
            let model: &squid_n_core::model::Model =
                if squid_n_load::wall_expand::model_has_wall_plates_to_expand(&self.core.model) {
                    let (expanded, _wall_index, _wall_report) =
                        squid_n_load::wall_expand::expand_wall_elements(&self.core.model);
                    expanded_storage = expanded;
                    &expanded_storage
                } else {
                    &self.core.model
                };
            for elem in &model.elements {
                let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
                    continue;
                };
                let Some(mat) = model.element_material(elem) else {
                    continue;
                };
                let rebar_mat = model.element_rebar_material(elem);
                let shear_mat = model.element_shear_rebar_material(elem);
                let steel_grade = model
                    .element_steel_material(elem)
                    .map(|m| m.name.clone())
                    .unwrap_or_default();
                let is_brace_elem =
                    matches!(elem.kind, squid_n_core::model::ElementKind::Brace { .. })
                        || squid_n_design_jp::MemberKind::of_element(elem, model)
                            == squid_n_design_jp::MemberKind::Brace;
                let elem_steel = elem_is_steel(elem, model);
                let rank = if is_brace_elem && elem_steel {
                    let len = model.member_length(elem);
                    let i_min = sec.iy.min(sec.iz);
                    if sec.area <= 0.0 || i_min <= 0.0 || len <= 0.0 {
                        continue;
                    }
                    let radius = (i_min / sec.area).sqrt();
                    if radius <= 0.0 {
                        continue;
                    }
                    let f_value = steel_f_value_prefix(
                        &mat.name,
                        squid_n_design_jp::material_strength::plate_thickness(sec),
                    )
                    .unwrap_or(235.0);
                    steel_brace_type(len / radius, f_value)
                } else if elem_steel {
                    let Some(shape) = sec.shape.as_ref() else {
                        continue;
                    };
                    let member_use = steel_member_use_of(elem, model);
                    let Some(rank) = steel_width_thickness_rank(shape, member_use, &mat.name)
                    else {
                        continue;
                    };
                    rank
                } else if matches!(sec.shape.as_ref(), Some(SectionShape::SrcRect { .. })) {
                    use squid_n_design_jp::secondary::src_rank::{
                        src_column_rank, src_column_rank_ratios,
                    };
                    if squid_n_design_jp::MemberKind::of_element(elem, model)
                        != squid_n_design_jp::MemberKind::Column
                    {
                        continue;
                    }
                    let Some(fc) = mat.fc.filter(|f| *f > 0.0) else {
                        continue;
                    };
                    let Some(resp) = resp_by_elem.get(&elem.id) else {
                        continue;
                    };
                    let shape = sec.shape.as_ref().expect("SrcRect と判定済み");
                    let Some(rebar_sy) = shape.rebar().and_then(|_| {
                        squid_n_core::material_grade::rebar_yield_strength(rebar_mat).or(mat.fy)
                    }) else {
                        continue;
                    };
                    let Some((n_n0, smo_m0)) =
                        src_column_rank_ratios(shape, &steel_grade, fc, rebar_sy, resp.axial)
                    else {
                        continue;
                    };
                    src_column_rank(n_n0, smo_m0, shear_yield_elems.contains(&elem.id))
                } else if let Some(SectionShape::RcWall { thickness, .. }) = sec.shape.as_ref() {
                    if wall_has_src_boundary_column(elem, model) {
                        let qu =
                            squid_n_element::wall::wall_element::WallElement::shear_capacity_of(
                                elem, model,
                            );
                        let Some(resp) = resp_by_elem.get(&elem.id) else {
                            continue;
                        };
                        if qu <= 0.0 {
                            continue;
                        }
                        let shear_failure = resp.horizontal_force >= 0.99 * qu;
                        squid_n_design_jp::secondary::src_rank::src_wall_type(shear_failure)
                    } else {
                        let Some(fc) = mat.fc else {
                            continue;
                        };
                        let Some(resp) = resp_by_elem.get(&elem.id) else {
                            continue;
                        };
                        let Some(wgeom) =
                            squid_n_element::wall::wall_element::wall_element_geometry(elem, model)
                        else {
                            continue;
                        };
                        let wall_len = wgeom.lw;
                        let r2 =
                            squid_n_element::wall::wall_element::WallElement::opening_strength_reduction(
                                elem, model,
                            );
                        let Some(tau_over_fc) = rc_wall_tau_over_fc(
                            resp.horizontal_force,
                            *thickness,
                            wall_len,
                            r2,
                            fc,
                        ) else {
                            continue;
                        };
                        let qu =
                            squid_n_element::wall::wall_element::WallElement::shear_capacity_of(
                                elem, model,
                            );
                        let brittle = rc_wall_shear_brittle(resp.horizontal_force, qu);
                        let wall_structure = self.core.wall_structure;
                        rc_wall_type(tau_over_fc, wall_structure, brittle)
                    }
                } else {
                    let Some(SectionShape::RcRect { b, d, rebar }) = sec.shape.as_ref() else {
                        continue;
                    };
                    let geom_len = model.member_length(elem);
                    let face_sum =
                        elem.rigid_zone.face_i_or_zero() + elem.rigid_zone.face_j_or_zero();
                    let clear_span = if geom_len - face_sum > 0.0 {
                        geom_len - face_sum
                    } else {
                        geom_len
                    };
                    let Some(mut input) = rc_capacity_input_from_rect(
                        *b, *d, rebar, mat, rebar_mat, shear_mat, clear_span,
                    ) else {
                        continue;
                    };
                    let sigma_0 = self
                        .core
                        .scoped
                        .results
                        .as_ref()
                        .map(|r| {
                            rc_sigma_0_from_gravity_or_last_static(
                                &r.statics,
                                &r.member_forces,
                                gravity_lc,
                                elem.id,
                                *b,
                                *d,
                            )
                        })
                        .unwrap_or(0.0);
                    let kind = squid_n_design_jp::MemberKind::of_element(elem, model);
                    let Some(resp) = resp_by_elem.get(&elem.id) else {
                        continue;
                    };
                    let gross = *b * *d;
                    if gross <= 0.0 || input.fc <= 0.0 {
                        continue;
                    }
                    let sigma_0_ult = resp.axial / gross;
                    let _ = sigma_0;
                    input.sigma_0 = sigma_0_ult;
                    let qmu = match kind {
                        squid_n_design_jp::MemberKind::Column => {
                            let ag = squid_n_core::section_shape::bar_set_area(&rebar.main_x);
                            let n_axial = resp.axial;
                            let mu = rc_column_mu_simple(&input, ag, n_axial);
                            if clear_span > 0.0 {
                                2.0 * mu / clear_span
                            } else {
                                0.0
                            }
                        }
                        _ => rc_qmu_simple(&input),
                    };
                    let qsu = rc_qsu_simple(&input);

                    let shear_u = resp.shear_strong.max(resp.shear_weak);
                    let tau_over_fc = (shear_u / gross) / input.fc;
                    let brittle = qmu > 0.0 && qsu < qmu;
                    match kind {
                        squid_n_design_jp::MemberKind::Column => {
                            let sigma0_over_fc = sigma_0_ult / input.fc;
                            let pt_percent = if *b > 0.0 && input.d_eff > 0.0 {
                                100.0 * input.at / (*b * input.d_eff)
                            } else {
                                0.0
                            };
                            let h0_over_d = if *d > 0.0 { clear_span / *d } else { 0.0 };
                            rc_column_type(
                                h0_over_d,
                                sigma0_over_fc,
                                pt_percent,
                                tau_over_fc,
                                brittle,
                            )
                        }
                        _ => rc_beam_type(tau_over_fc, brittle),
                    }
                };
                let Some(story_idx) = elem
                    .nodes
                    .iter()
                    .filter_map(|nid| model.nodes.get(nid.index()))
                    .filter_map(|n| n.story)
                    .max()
                else {
                    continue;
                };
                let Some(idx) = story_idx.index().checked_sub(1) else {
                    continue;
                };
                if idx >= n_stories {
                    continue;
                }
                per_story[idx].push(rank);
                computed.push((elem.id, rank));

                let q_h = resp_by_elem
                    .get(&elem.id)
                    .map(|r| r.horizontal_force)
                    .unwrap_or(0.0);
                let gi = rank_index_for_group(rank);
                if matches!(
                    elem.kind,
                    squid_n_core::model::ElementKind::Wall
                        | squid_n_core::model::ElementKind::Brace { .. }
                ) {
                    wall_members[idx].push((gi, q_h));
                    wall_horizontal[idx] += q_h;
                } else {
                    cb_members[idx].push((gi, q_h));
                }
            }
            let mut fallback_stories: Vec<String> = Vec::new();
            let ranks: Vec<MemberRank> = per_story
                .into_iter()
                .enumerate()
                .map(|(i, rs)| {
                    worst_rank(&rs).unwrap_or_else(|| {
                        if let Some(s) = self.core.model.stories.get(i) {
                            fallback_stories.push(s.name.clone());
                        }
                        self.core.design_rank
                    })
                })
                .collect();
            self.core.scoped.ds_rank_fallback_stories = fallback_stories;
            (ranks, computed)
        } else {
            self.core.scoped.ds_rank_fallback_stories = Vec::new();
            (vec![self.core.design_rank; n_stories], Vec::new())
        };

        let mechanism = &po.mechanism;
        let is_rc_frame = matches!(
            self.core.design_frame,
            squid_n_design_jp::secondary::holding_capacity::FrameType::RcFrame
                | squid_n_design_jp::secondary::holding_capacity::FrameType::RcWall
        );
        let mut beta_u_by_story: Vec<f64> = vec![0.0; n_stories];
        let mut beta_u_unavailable = false;
        let ds_vec: Vec<f64> = (0..n_stories)
            .map(|i| {
                let fallback_group = |rank: MemberRank| match rank {
                    MemberRank::FA => GroupType::A,
                    MemberRank::FB => GroupType::B,
                    MemberRank::FC => GroupType::C,
                    MemberRank::FD => GroupType::D,
                };
                let rep_rank = story_ranks.get(i).copied().unwrap_or(self.core.design_rank);
                let mut cb_group =
                    member_group(&cb_members[i]).unwrap_or_else(|| fallback_group(rep_rank));
                if let MechanismType::StoryCollapse { layer } = mechanism {
                    if *layer == i {
                        cb_group = match cb_group {
                            GroupType::A => GroupType::B,
                            GroupType::B => GroupType::C,
                            GroupType::C | GroupType::D => GroupType::D,
                        };
                    }
                }
                let wall_group = member_group(&wall_members[i]).unwrap_or(GroupType::A);
                let qu_i = story_qu.get(i).copied().unwrap_or(0.0);
                let beta_u = if qu_i > 0.0 {
                    (wall_horizontal[i] / qu_i).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let declares_wall_or_brace = matches!(
                    self.core.design_frame,
                    squid_n_design_jp::secondary::holding_capacity::FrameType::RcWall
                        | squid_n_design_jp::secondary::holding_capacity::FrameType::SteelBrace
                );
                if declares_wall_or_brace && wall_members[i].is_empty() {
                    beta_u_unavailable = true;
                    return squid_n_design_jp::secondary::holding_capacity::ds_value(
                        self.core.design_frame,
                        rep_rank,
                    );
                }
                beta_u_by_story[i] = beta_u;
                if is_rc_frame {
                    ds_rc(wall_group, beta_u, cb_group)
                } else {
                    ds_steel(wall_group, beta_u, cb_group)
                }
            })
            .collect();
        self.core.scoped.ds_beta_u_by_story = beta_u_by_story;
        self.core.scoped.ds_beta_u_unavailable = beta_u_unavailable;

        let heights: Vec<f64> = metrics.iter().map(|m| m.height).collect();
        let rs: Vec<f64> = metrics.iter().map(|m| m.rs).collect();
        let re: Vec<f64> = metrics.iter().map(|m| m.re).collect();
        let fes: Vec<f64> = metrics.iter().map(|m| m.fes).collect();

        let result =
            check_holding_capacity(po, &qud, &ds_vec, &fes, &rs, &re, &heights, member_ranks);
        Ok((result, story_ranks))
    }
}
