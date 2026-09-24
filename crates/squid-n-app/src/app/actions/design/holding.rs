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
                    if elem
                        .section
                        .and_then(|sid| model.sections.get(sid.index()))
                        .is_some_and(|section| {
                            matches!(
                                section.shape,
                                Some(
                                    SectionShape::RcColumnRect { .. }
                                        | SectionShape::RcColumnCircle { .. }
                                )
                            )
                        })
                    {
                        return Err(format!(
                            "部材 {:?}・断面 {:?}: 材料が未設定です",
                            elem.id,
                            elem.section.unwrap()
                        ));
                    }
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
                } else if matches!(
                    sec.shape.as_ref(),
                    Some(SectionShape::SrcRect { .. } | SectionShape::SrcColumnRect { .. })
                ) {
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
                    let shape = sec.shape.as_ref().expect("SRC 矩形柱と判定済み");
                    if shape.rebar().is_none() && shape.rect_column_rebar().is_none() {
                        continue;
                    }
                    let Some(rebar_sy) =
                        squid_n_core::material_grade::rebar_yield_strength(rebar_mat).or(mat.fy)
                    else {
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
                        let capacities = squid_n_element::wall::wall_element::WallElement::directional_shear_capacity_of(elem, model);
                        let qu = resp_by_elem
                            .get(&elem.id)
                            .and_then(|r| r.wall_shear_signed)
                            .map(|q| capacities[usize::from(q < 0.0)])
                            .unwrap_or(0.0);
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
                        let capacities = squid_n_element::wall::wall_element::WallElement::directional_shear_capacity_of(elem, model);
                        let qu = resp_by_elem
                            .get(&elem.id)
                            .and_then(|r| r.wall_shear_signed)
                            .map(|q| capacities[usize::from(q < 0.0)])
                            .unwrap_or(0.0);
                        let brittle = rc_wall_shear_brittle(resp.horizontal_force, qu);
                        let wall_structure = self.core.wall_structure;
                        rc_wall_type(tau_over_fc, wall_structure, brittle)
                    }
                } else if let Some(SectionShape::RcBeamRect { b, d, rebar }) = sec.shape.as_ref() {
                    if squid_n_design_jp::MemberKind::of_element(elem, model)
                        != squid_n_design_jp::MemberKind::Beam
                    {
                        return Err(format!(
                            "部材 {:?}・断面 {:?}: 新型RC梁断面の用途が梁ではありません",
                            elem.id, sec.id
                        ));
                    }
                    let Some(resp) = resp_by_elem.get(&elem.id) else {
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
                    rank_new_rc_beam(
                        elem.id, sec.id, *b, *d, rebar, mat, rebar_mat, shear_mat, resp, clear_span,
                    )?
                } else if let Some(
                    shape @ (SectionShape::RcColumnRect { .. }
                    | SectionShape::RcColumnCircle { .. }),
                ) = sec.shape.as_ref()
                {
                    if squid_n_design_jp::MemberKind::of_element(elem, model)
                        != squid_n_design_jp::MemberKind::Column
                    {
                        return Err(format!(
                            "部材 {:?}・断面 {:?}: 新型RC柱断面の用途が柱ではありません",
                            elem.id, sec.id
                        ));
                    }
                    let Some(resp) = resp_by_elem.get(&elem.id) else {
                        return Err(format!(
                            "部材 {:?}・断面 {:?}: 増分解析応答がありません",
                            elem.id, sec.id
                        ));
                    };
                    let geom_len = model.member_length(elem);
                    let face_sum =
                        elem.rigid_zone.face_i_or_zero() + elem.rigid_zone.face_j_or_zero();
                    let clear_span = if geom_len - face_sum > 0.0 {
                        geom_len - face_sum
                    } else {
                        geom_len
                    };
                    match shape {
                        SectionShape::RcColumnRect { b, d, rebar } => rank_new_rc_column_rect(
                            elem.id, sec.id, *b, *d, rebar, mat, rebar_mat, shear_mat, resp,
                            clear_span,
                        )?,
                        SectionShape::RcColumnCircle { d, rebar } => rank_new_rc_circle_column(
                            elem.id, sec.id, *d, rebar, mat, rebar_mat, shear_mat, resp, clear_span,
                        )?,
                        _ => unreachable!(),
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

#[allow(clippy::too_many_arguments)]
fn rank_new_rc_column_rect(
    elem_id: ElemId,
    section_id: squid_n_core::ids::SectionId,
    b: f64,
    d: f64,
    rebar: &squid_n_core::section_shape::RcRectColumnRebar,
    mat: &squid_n_core::model::Material,
    rebar_mat: Option<&squid_n_core::model::Material>,
    shear_mat: Option<&squid_n_core::model::Material>,
    response: &squid_n_solver::nonlinear::pushover::PushoverMemberResponse,
    clear_span: f64,
) -> Result<squid_n_design_jp::secondary::holding_capacity::MemberRank, String> {
    use squid_n_core::rc_capacity::{rc_column_mu_simple, rc_qsu_simple, RcCapacityInput};
    use squid_n_core::rc_rebar_geom::RectEdge;
    use squid_n_design_jp::secondary::ds_group::rc_column_type;
    let error = |s: &str| format!("部材 {:?}・断面 {:?}: {}", elem_id, section_id, s);
    if rebar.is_unset() {
        return Err(error("配筋が未設定です"));
    }
    rebar
        .validate(b, d)
        .map_err(|_| error("配筋形状が不正です"))?;
    validate_new_column_materials(mat, rebar_mat, shear_mat, clear_span).map_err(&error)?;
    if !b.is_finite() || !d.is_finite() || b <= 0.0 || d <= 0.0 {
        return Err(error("断面寸法が不正です"));
    }
    let fc = mat.fc.unwrap();
    let sigma_y = squid_n_core::material_grade::rebar_yield_strength(rebar_mat)
        .or(mat.fy)
        .ok_or_else(|| error("主筋の降伏強度を解決できません"))?;
    let sigma_wy = squid_n_core::material_grade::shear_rebar_yield_strength(shear_mat)
        .ok_or_else(|| error("せん断補強筋の降伏強度を解決できません"))?;
    let ag = rebar.total_main_area();
    let eval = |strong: bool, shear: f64| {
        let (bd, dd, edge, aw, legs) = if strong {
            (b, d, RectEdge::Top, rebar.aw_x_mm2(), rebar.hoop.legs_x)
        } else {
            (d, b, RectEdge::Left, rebar.aw_y_mm2(), rebar.hoop.legs_y)
        };
        let steel = rebar.edge_steel(edge, b, d);
        let input = RcCapacityInput {
            b: bd,
            d: dd,
            at: steel.area_mm2,
            d_eff: steel.effective_depth_mm,
            sigma_y,
            fc,
            pw: aw / (bd * rebar.hoop.pitch),
            sigma_wy,
            clear_span,
            sigma_0: response.axial / (b * d),
        };
        let qmu = 2.0 * rc_column_mu_simple(&input, ag, response.axial) / clear_span;
        let qsu = rc_qsu_simple(&input);
        let rank = rc_column_type(
            clear_span / dd,
            response.axial / (b * d * fc),
            100.0 * input.at / (bd * input.d_eff),
            (shear / (bd)) / fc,
            qsu < qmu,
        );
        let _ = legs;
        rank
    };
    Ok(squid_n_design_jp::secondary::member_rank::worst_rank(&[
        eval(true, response.shear_strong.abs()),
        eval(false, response.shear_weak.abs()),
    ])
    .expect("二軸の候補は空ではありません"))
}

#[allow(clippy::too_many_arguments)]
fn rank_new_rc_circle_column(
    elem_id: ElemId,
    section_id: squid_n_core::ids::SectionId,
    d: f64,
    rebar: &squid_n_core::section_shape::RcCircleColumnRebar,
    mat: &squid_n_core::model::Material,
    rebar_mat: Option<&squid_n_core::model::Material>,
    shear_mat: Option<&squid_n_core::model::Material>,
    response: &squid_n_solver::nonlinear::pushover::PushoverMemberResponse,
    clear_span: f64,
) -> Result<squid_n_design_jp::secondary::holding_capacity::MemberRank, String> {
    use squid_n_core::rc_capacity::{rc_column_mu_simple, rc_qsu_simple, RcCapacityInput};
    use squid_n_design_jp::secondary::ds_group::rc_column_type;
    let error = |s: &str| format!("部材 {:?}・断面 {:?}: {}", elem_id, section_id, s);
    if rebar.is_unset() {
        return Err(error("配筋が未設定です"));
    }
    rebar.validate(d).map_err(|_| error("配筋形状が不正です"))?;
    validate_new_column_materials(mat, rebar_mat, shear_mat, clear_span).map_err(&error)?;
    if !d.is_finite() || d <= 0.0 {
        return Err(error("断面寸法が不正です"));
    }
    let fc = mat.fc.unwrap();
    let side = rebar.equivalent_square_side_mm(d);
    let sigma_y = squid_n_core::material_grade::rebar_yield_strength(rebar_mat)
        .or(mat.fy)
        .ok_or_else(|| error("主筋の降伏強度を解決できません"))?;
    let sigma_wy = squid_n_core::material_grade::shear_rebar_yield_strength(shear_mat)
        .ok_or_else(|| error("せん断補強筋の降伏強度を解決できません"))?;
    let input = RcCapacityInput {
        b: side,
        d: side,
        at: rebar.equivalent_tension_area_mm2(),
        d_eff: rebar.equivalent_effective_depth_mm(d),
        sigma_y,
        fc,
        pw: rebar.pw(side),
        sigma_wy,
        clear_span,
        sigma_0: response.axial / (side * side),
    };
    let qmu =
        2.0 * rc_column_mu_simple(&input, rebar.total_main_area(), response.axial) / clear_span;
    let qsu = rc_qsu_simple(&input);
    let rank = |shear: f64| {
        rc_column_type(
            clear_span / d,
            response.axial / (side * side * fc),
            100.0 * input.at / (side * input.d_eff),
            (shear / (side * side)) / fc,
            qsu < qmu,
        )
    };
    Ok(squid_n_design_jp::secondary::member_rank::worst_rank(&[
        rank(response.shear_strong.abs()),
        rank(response.shear_weak.abs()),
    ])
    .expect("二軸の候補は空ではありません"))
}

fn validate_new_column_materials(
    mat: &squid_n_core::model::Material,
    rebar_mat: Option<&squid_n_core::model::Material>,
    shear_mat: Option<&squid_n_core::model::Material>,
    clear_span: f64,
) -> Result<(), &'static str> {
    if mat.fc.filter(|v| v.is_finite() && *v > 0.0).is_none() {
        return Err("Fc が未設定または 0 以下です");
    }
    if !clear_span.is_finite() || clear_span <= 0.0 {
        return Err("内法スパンが 0 以下です");
    }
    if squid_n_core::material_grade::rebar_yield_strength(rebar_mat)
        .or(mat.fy)
        .filter(|v| v.is_finite() && *v > 0.0)
        .is_none()
    {
        return Err("主筋の降伏強度を解決できません");
    }
    if squid_n_core::material_grade::shear_rebar_yield_strength(shear_mat)
        .filter(|v| v.is_finite() && *v > 0.0)
        .is_none()
    {
        return Err("せん断補強筋の降伏強度を解決できません");
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn rank_new_rc_beam(
    elem_id: ElemId,
    section_id: squid_n_core::ids::SectionId,
    b: f64,
    d: f64,
    rebar: &squid_n_core::section_shape::RcBeamRebar,
    mat: &squid_n_core::model::Material,
    rebar_mat: Option<&squid_n_core::model::Material>,
    shear_mat: Option<&squid_n_core::model::Material>,
    response: &squid_n_solver::nonlinear::pushover::PushoverMemberResponse,
    clear_span: f64,
) -> Result<squid_n_design_jp::secondary::holding_capacity::MemberRank, String> {
    let error = |message: &str| format!("部材 {:?}・断面 {:?}: {}", elem_id, section_id, message);
    rebar
        .validate(b, d)
        .map_err(|_| error("配筋形状が不正です"))?;
    let fc = mat
        .fc
        .filter(|fc| *fc > 0.0)
        .ok_or_else(|| error("Fc が未設定または 0 以下です"))?;
    if clear_span <= 0.0 {
        return Err(error("内法スパンが 0 以下です"));
    }
    let sigma_y = squid_n_core::material_grade::rebar_yield_strength(rebar_mat)
        .or(mat.fy)
        .ok_or_else(|| error("主筋の降伏強度を解決できません"))?;
    let sigma_wy = squid_n_core::material_grade::shear_rebar_yield_strength(shear_mat)
        .ok_or_else(|| error("せん断補強筋の降伏強度を解決できません"))?;
    if rebar.is_unset() {
        return Err(error("配筋が未設定です"));
    }
    let pw = rebar.pw(b);
    let (top_rank, _, _) = rank_new_rc_beam_side(
        b,
        d,
        rebar,
        true,
        sigma_y,
        fc,
        pw,
        sigma_wy,
        clear_span,
        response.axial,
        response.shear_strong.abs(),
    );
    let (bottom_rank, _, _) = rank_new_rc_beam_side(
        b,
        d,
        rebar,
        false,
        sigma_y,
        fc,
        pw,
        sigma_wy,
        clear_span,
        response.axial,
        response.shear_strong.abs(),
    );
    Ok(worst_rc_beam_rank(top_rank, bottom_rank))
}

fn worst_rc_beam_rank(
    top: squid_n_design_jp::secondary::holding_capacity::MemberRank,
    bottom: squid_n_design_jp::secondary::holding_capacity::MemberRank,
) -> squid_n_design_jp::secondary::holding_capacity::MemberRank {
    squid_n_design_jp::secondary::member_rank::worst_rank(&[top, bottom])
        .expect("上下側のランクは空ではありません")
}

#[allow(clippy::too_many_arguments)]
fn rank_new_rc_beam_side(
    b: f64,
    d: f64,
    rebar: &squid_n_core::section_shape::RcBeamRebar,
    tension_is_top: bool,
    sigma_y: f64,
    fc: f64,
    pw: f64,
    sigma_wy: f64,
    clear_span: f64,
    axial: f64,
    shear: f64,
) -> (
    squid_n_design_jp::secondary::holding_capacity::MemberRank,
    f64,
    f64,
) {
    use squid_n_core::rc_capacity::{rc_qmu_simple, rc_qsu_simple, RcCapacityInput};
    use squid_n_design_jp::secondary::ds_group::rc_beam_type;

    let bending = rebar.bending_steel(d, tension_is_top);
    let input = RcCapacityInput {
        b,
        d,
        at: bending.tension.area_mm2,
        d_eff: bending.tension.effective_depth_mm,
        sigma_y,
        fc,
        pw,
        sigma_wy,
        clear_span,
        sigma_0: axial / (b * d),
    };
    let qmu = rc_qmu_simple(&input);
    let qsu = rc_qsu_simple(&input);
    let tau_over_fc = (shear / (b * d)) / fc;
    let brittle = qmu > 0.0 && qsu < qmu;
    (rc_beam_type(tau_over_fc, brittle), qmu, qsu)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::model::Material;
    use squid_n_core::section_shape::{BeamStirrup, RcBeamRebar};
    use squid_n_design_jp::secondary::holding_capacity::MemberRank;
    use squid_n_design_jp::secondary::member_rank::worst_rank;

    fn rebar() -> RcBeamRebar {
        RcBeamRebar {
            main_dia: 22.0,
            top: vec![4, 2],
            bottom: vec![3, 2],
            cover: 40.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        }
    }

    fn side(top: bool, shear: f64) -> (MemberRank, f64, f64, f64, f64) {
        let rebar = rebar();
        let bending = rebar.bending_steel(600.0, top);
        let (rank, qmu, qsu) = rank_new_rc_beam_side(
            300.0,
            600.0,
            &rebar,
            top,
            345.0,
            24.0,
            rebar.pw(300.0),
            295.0,
            3000.0,
            0.0,
            shear,
        );
        (
            rank,
            qmu,
            qsu,
            bending.tension.area_mm2,
            bending.tension.effective_depth_mm,
        )
    }

    #[test]
    fn 新型_rc梁の上下引張側を個別評価する() {
        use squid_n_core::rc_capacity::{rc_qmu_simple, RcCapacityInput};

        let rebar = rebar();
        let expected = |top| {
            let bending = rebar.bending_steel(600.0, top);
            rc_qmu_simple(&RcCapacityInput {
                b: 300.0,
                d: 600.0,
                at: bending.tension.area_mm2,
                d_eff: bending.tension.effective_depth_mm,
                sigma_y: 345.0,
                fc: 24.0,
                pw: rebar.pw(300.0),
                sigma_wy: 295.0,
                clear_span: 3000.0,
                sigma_0: 0.0,
            })
        };
        let top = side(true, 0.0);
        let bottom = side(false, 0.0);
        assert_eq!(top.1, expected(true));
        assert_eq!(bottom.1, expected(false));
        assert_ne!((top.3, top.4, top.1), (bottom.3, bottom.4, bottom.1));

        assert_eq!(
            worst_rc_beam_rank(MemberRank::FA, MemberRank::FD),
            MemberRank::FD
        );
        assert_eq!(
            worst_rc_beam_rank(MemberRank::FC, MemberRank::FB),
            MemberRank::FC
        );
    }

    fn rank_input() -> (
        Material,
        Material,
        Material,
        squid_n_solver::nonlinear::pushover::PushoverMemberResponse,
    ) {
        let steel = |id, name: &str, fy| Material {
            id: squid_n_core::ids::MaterialId(id),
            name: name.into(),
            category: squid_n_core::model::MaterialCategory::Steel,
            young: 200_000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(fy),
            concrete_class: Default::default(),
            strength_factor: None,
        };
        (
            Material {
                id: squid_n_core::ids::MaterialId(1),
                name: "concrete".into(),
                category: squid_n_core::model::MaterialCategory::Concrete,
                young: 25_000.0,
                poisson: 0.2,
                density: 0.0,
                shear: None,
                fc: Some(24.0),
                fy: None,
                concrete_class: Default::default(),
                strength_factor: None,
            },
            steel(2, "SD345", 345.0),
            steel(3, "SD295", 295.0),
            squid_n_solver::nonlinear::pushover::PushoverMemberResponse {
                elem: ElemId(3),
                m_strong: 0.0,
                m_weak: 0.0,
                shear_strong: 0.0,
                shear_weak: 0.0,
                axial: 0.0,
                rp: 0.0,
                horizontal_force: 0.0,
                wall_shear_signed: None,
            },
        )
    }

    #[test]
    fn 新型_rc梁の不正入力は識別情報付きで失敗する() {
        let rebar = rebar();
        let (mat, main_steel, shear_steel, response) = rank_input();
        let elem = ElemId(3);
        let section = squid_n_core::ids::SectionId(4);
        let unset = RcBeamRebar {
            main_dia: 0.0,
            top: vec![],
            bottom: vec![],
            cover: 0.0,
            stirrup: BeamStirrup {
                dia: 0.0,
                pitch: 0.0,
                legs: 0,
            },
        };
        let rank = |rebar: &RcBeamRebar, mat: &Material, rebar_mat, shear_mat, span| {
            rank_new_rc_beam(
                elem, section, 300.0, 600.0, rebar, mat, rebar_mat, shear_mat, &response, span,
            )
        };
        let valid = rank(&rebar, &mat, Some(&main_steel), Some(&shear_steel), 3000.0);
        assert!(valid.is_ok(), "正常入力で失敗: {valid:?}");
        let assert_error = |result: Result<MemberRank, String>, reason: &str| {
            let error = result.unwrap_err();
            assert!(error.contains("部材 ElemId(3)"), "{error}");
            assert!(error.contains("断面 SectionId(4)"), "{error}");
            assert!(error.contains(reason), "{error}");
        };
        let mut invalid = rebar.clone();
        invalid.top[0] = 0;
        assert_error(
            rank(
                &invalid,
                &mat,
                Some(&main_steel),
                Some(&shear_steel),
                3000.0,
            ),
            "配筋形状が不正",
        );
        assert_error(
            rank(&unset, &mat, Some(&main_steel), Some(&shear_steel), 3000.0),
            "配筋が未設定",
        );
        let mut no_fc = mat.clone();
        no_fc.fc = None;
        assert_error(
            rank(
                &rebar,
                &no_fc,
                Some(&main_steel),
                Some(&shear_steel),
                3000.0,
            ),
            "Fc が未設定",
        );
        let mut zero_fc = mat.clone();
        zero_fc.fc = Some(0.0);
        assert_error(
            rank(
                &rebar,
                &zero_fc,
                Some(&main_steel),
                Some(&shear_steel),
                3000.0,
            ),
            "Fc が未設定",
        );
        assert_error(
            rank(&rebar, &mat, Some(&main_steel), Some(&shear_steel), 0.0),
            "内法スパン",
        );
        let mut no_fy = mat.clone();
        no_fy.fy = None;
        assert_error(
            rank(&rebar, &no_fy, None, Some(&shear_steel), 3000.0),
            "主筋の降伏強度",
        );
        assert_error(
            rank(&rebar, &mat, Some(&main_steel), None, 3000.0),
            "せん断補強筋の降伏強度",
        );
    }

    #[test]
    fn 新型_rc柱の未設定配筋は識別情報付きで失敗する() {
        use squid_n_core::section_shape::{
            CircleColumnHoop, RcCircleColumnRebar, RcRectColumnRebar, RectColumnHoop,
        };

        let (mat, main_steel, shear_steel, response) = rank_input();
        let elem = ElemId(3);
        let section = squid_n_core::ids::SectionId(4);
        let rect = RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![],
            y: vec![],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 2,
            },
        };
        let circle = RcCircleColumnRebar {
            main_dia: 22.0,
            count: 0,
            cover: 40.0,
            hoop: CircleColumnHoop {
                dia: 10.0,
                pitch: 100.0,
            },
        };
        for error in [
            rank_new_rc_column_rect(
                elem,
                section,
                400.0,
                600.0,
                &rect,
                &mat,
                Some(&main_steel),
                Some(&shear_steel),
                &response,
                3000.0,
            )
            .unwrap_err(),
            rank_new_rc_circle_column(
                elem,
                section,
                600.0,
                &circle,
                &mat,
                Some(&main_steel),
                Some(&shear_steel),
                &response,
                3000.0,
            )
            .unwrap_err(),
        ] {
            assert!(error.contains("部材 ElemId(3)"), "{error}");
            assert!(error.contains("断面 SectionId(4)"), "{error}");
            assert!(error.contains("配筋が未設定"), "{error}");
        }
    }

    #[test]
    fn 新型_rc矩形柱は二方向の断面算定ランクの最悪値を採用する() {
        use squid_n_core::rc_capacity::{rc_column_mu_simple, rc_qsu_simple, RcCapacityInput};
        use squid_n_core::rc_rebar_geom::RectEdge;
        use squid_n_core::section_shape::{RcRectColumnRebar, RectColumnHoop};
        use squid_n_design_jp::secondary::ds_group::rc_column_type;

        let (mat, main_steel, shear_steel, response) = rank_input();
        let rebar = RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![3],
            y: vec![3],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 3,
            },
        };
        let a1 = std::f64::consts::PI * 22.0_f64.powi(2) / 4.0;
        let ag = rebar.total_main_area();
        assert!((ag - 8.0 * a1).abs() < 1e-10);
        let expected = |strong: bool| {
            let (bd, dd, edge, aw, shear) = if strong {
                (
                    400.0,
                    600.0,
                    RectEdge::Top,
                    rebar.aw_x_mm2(),
                    response.shear_strong,
                )
            } else {
                (
                    600.0,
                    400.0,
                    RectEdge::Left,
                    rebar.aw_y_mm2(),
                    response.shear_weak,
                )
            };
            let steel = rebar.edge_steel(edge, 400.0, 600.0);
            assert!((steel.area_mm2 - 3.0 * a1).abs() < 1e-10);
            let input = RcCapacityInput {
                b: bd,
                d: dd,
                at: steel.area_mm2,
                d_eff: steel.effective_depth_mm,
                sigma_y: 345.0,
                fc: 24.0,
                pw: aw / (bd * 100.0),
                sigma_wy: 295.0,
                clear_span: 3000.0,
                sigma_0: response.axial / (400.0 * 600.0),
            };
            let qmu = 2.0 * rc_column_mu_simple(&input, ag, response.axial) / 3000.0;
            let qsu = rc_qsu_simple(&input);
            rc_column_type(
                3000.0 / dd,
                response.axial / (400.0 * 600.0 * 24.0),
                100.0 * input.at / (bd * input.d_eff),
                (shear.abs() / (bd)) / 24.0,
                qsu < qmu,
            )
        };
        assert_ne!(rebar.aw_x_mm2(), rebar.aw_y_mm2());
        assert_eq!(
            rebar.aw_x_mm2() / (400.0 * 100.0),
            rebar.aw_y_mm2() / (600.0 * 100.0)
        );
        let expected_rank = worst_rank(&[expected(true), expected(false)]).unwrap();
        assert_eq!(
            rank_new_rc_column_rect(
                ElemId(3),
                squid_n_core::ids::SectionId(4),
                400.0,
                600.0,
                &rebar,
                &mat,
                Some(&main_steel),
                Some(&shear_steel),
                &response,
                3000.0
            )
            .unwrap(),
            expected_rank,
        );
    }

    #[test]
    fn 新型_rc円形柱は実径と二方向せん断応答の最悪ランクを採用する() {
        use squid_n_core::rc_capacity::{rc_column_mu_simple, rc_qsu_simple, RcCapacityInput};
        use squid_n_core::section_shape::{CircleColumnHoop, RcCircleColumnRebar};
        use squid_n_design_jp::secondary::ds_group::rc_column_type;

        let (mat, main_steel, shear_steel, mut response) = rank_input();
        response.shear_strong = 100_000.0;
        response.shear_weak = 500_000.0;
        let rebar = RcCircleColumnRebar {
            main_dia: 22.0,
            count: 8,
            cover: 40.0,
            hoop: CircleColumnHoop {
                dia: 10.0,
                pitch: 100.0,
            },
        };
        let d: f64 = 600.0;
        let side = (std::f64::consts::PI * d.powi(2) / 4.0).sqrt();
        let a1 = std::f64::consts::PI * 22.0_f64.powi(2) / 4.0;
        let ag = rebar.total_main_area();
        let at = rebar.equivalent_tension_area_mm2();
        let pw = 2.0 * (std::f64::consts::PI * 10.0_f64.powi(2) / 4.0) / (side * 100.0);
        assert!((ag - 8.0 * a1).abs() < 1e-10);
        assert!((at - 2.0 * a1).abs() < 1e-10);
        let input = RcCapacityInput {
            b: side,
            d: side,
            at,
            d_eff: rebar.equivalent_effective_depth_mm(d),
            sigma_y: 345.0,
            fc: 24.0,
            pw,
            sigma_wy: 295.0,
            clear_span: 3000.0,
            sigma_0: response.axial / side.powi(2),
        };
        let qmu = 2.0 * rc_column_mu_simple(&input, ag, response.axial) / 3000.0;
        let qsu = rc_qsu_simple(&input);
        let qgross = side.powi(2);
        let expected = |shear: f64| {
            rc_column_type(
                3000.0 / d,
                response.axial / (qgross * 24.0),
                100.0 * at / (side * input.d_eff),
                (shear.abs() / qgross) / 24.0,
                qsu < qmu,
            )
        };
        let expected_rank = worst_rank(&[
            expected(response.shear_strong),
            expected(response.shear_weak),
        ])
        .unwrap();
        assert_ne!(response.shear_strong, response.shear_weak);
        assert_eq!(
            rank_new_rc_circle_column(
                ElemId(3),
                squid_n_core::ids::SectionId(4),
                d,
                &rebar,
                &mat,
                Some(&main_steel),
                Some(&shear_steel),
                &response,
                3000.0
            )
            .unwrap(),
            expected_rank,
        );
    }

    #[test]
    fn 新型_rc柱の不正形状と未設定配筋は識別情報付きで失敗する() {
        use squid_n_core::section_shape::{
            CircleColumnHoop, RcCircleColumnRebar, RcRectColumnRebar, RectColumnHoop,
        };

        let (mat, main_steel, shear_steel, response) = rank_input();
        let elem = ElemId(3);
        let section = squid_n_core::ids::SectionId(4);
        let rect = RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![4, 2],
            y: vec![3],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 3,
            },
        };
        let unset_rect = RcRectColumnRebar {
            x: vec![],
            y: vec![],
            ..rect.clone()
        };
        let unset_circle = RcCircleColumnRebar {
            main_dia: 0.0,
            count: 0,
            cover: 0.0,
            hoop: CircleColumnHoop {
                dia: 0.0,
                pitch: 0.0,
            },
        };
        for error in [
            rank_new_rc_column_rect(
                elem,
                section,
                400.0,
                600.0,
                &rect,
                &mat,
                Some(&main_steel),
                Some(&shear_steel),
                &response,
                3000.0,
            )
            .unwrap_err(),
            rank_new_rc_column_rect(
                elem,
                section,
                400.0,
                600.0,
                &unset_rect,
                &mat,
                Some(&main_steel),
                Some(&shear_steel),
                &response,
                3000.0,
            )
            .unwrap_err(),
            rank_new_rc_circle_column(
                elem,
                section,
                600.0,
                &unset_circle,
                &mat,
                Some(&main_steel),
                Some(&shear_steel),
                &response,
                3000.0,
            )
            .unwrap_err(),
        ] {
            assert!(error.contains("部材 ElemId(3)"), "{error}");
            assert!(error.contains("断面 SectionId(4)"), "{error}");
        }
    }
}
