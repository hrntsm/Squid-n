use super::super::*;

impl App {
    /// T7: 解析結果の member_forces から検定結果を生成する。
    /// 危険断面位置（§6.2.3、既定は柱フェイスと中央）の内力に対し、
    /// 材種・部材種別に応じた検定を適用する（令82条・各構造設計規準準拠）。
    /// 節点芯は剛域が有る場合は検定対象外（節点芯の応力をそのまま使わない、
    /// 設計書 §6.2.3）。
    ///
    /// - 部材種別は部材軸の鉛直成分から判定（柱/梁/ブレース）。
    /// - せん断スパン比 M/(Q·d) 用の代表値は、モーメントが最大となる
    ///   検定位置の値を採用する方針で部材単位に求める。
    /// - 柱は軸力＋二軸曲げ（n, my, mz）を検定に渡す。
    /// - 検定器は構造種別（`squid_n_core::structure_kind`）で選択する。
    pub fn run_design_check(&mut self) {
        // rigid_zone（face_i/j）から危険断面位置を決めるため、算定前に自動剛域を
        // 反映する（設計書 §6.2.1、冪等なので他の解析エントリと重複して呼んでも安全）。
        self.apply_rigid_zones_for_analysis();
        let Some(results) = &self.results else {
            return;
        };
        // 壁の解析要素（`ElementKind::Wall`）は `self.model` には存在しない生成物
        // （D5）だが、`results.member_forces` は壁展開済みモデルで解いた結果の
        // ため壁の `ElemId` を含む。`run_member_design_checks`（内部の
        // `joint_wiring::wall::check_walls`）はこの `ElemId` を `model.element`
        // で引き直すため、`self.model` のまま渡すと耐震壁のせん断断面検定が
        // 常にスキップされる（該当 `ElemId` が見つからず `continue` する）。
        // 壁を持たないモデル（実 ST-Bridge フィクスチャは現状すべて該当する）
        // では複製を避け、`self.model` をそのまま使う。
        let expanded_storage;
        let design_model: &squid_n_core::model::Model =
            if squid_n_load::wall_expand::model_has_wall_plates_to_expand(&self.model) {
                let (expanded, _wall_index, _wall_report) =
                    squid_n_load::wall_expand::expand_wall_elements(&self.model);
                expanded_storage = expanded;
                &expanded_storage
            } else {
                &self.model
            };
        // 地震時短期の設計用せん断力 QD 用の長期内力。
        // 優先: Q0 と同じ重力ケース集合の解析内力加算。なければ組合せ "DL + LL"。
        // 長期が未解析なら None（QD 割増なし＝従来動作）。
        let is_seismic_combo = match self.last_static {
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
        let gravity_long_owned = if is_seismic_combo && self.design_term == LoadTerm::Short {
            squid_n_job::sum_analyzed_gravity_member_forces(&self.model, |lc| {
                results
                    .statics
                    .iter()
                    .find(|(id, _)| *id == StaticCaseKey::User(lc))
                    .map(|(_, s)| s.member_forces.clone())
            })
        } else {
            None
        };
        let long_from_combo: Option<&Vec<(ElemId, squid_n_element::beam::MemberForces)>> =
            if is_seismic_combo && self.design_term == LoadTerm::Short {
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
        let long_member_forces: Option<&[(ElemId, squid_n_element::beam::MemberForces)]> =
            gravity_long_owned
                .as_deref()
                .or(long_from_combo.map(|v| v.as_slice()));
        // 一本部材指定（Model.beam_groups）: グループ単位の採用応力を合成し、
        // 所属部材の検定文脈（部材長・端部/中央モーメント等）を上書きする。
        let group_overrides =
            squid_n_design_jp::beam_group_overrides(&self.model, &results.member_forces);
        // 梁 QD1 用の単純梁せん断 Q0（Dead+LiveSeismic 加算の長期相当）。
        let q0_by_elem = if long_member_forces.is_some() {
            squid_n_job::simple_beam_q0_by_gravity_cases(&self.model)
        } else {
            Default::default()
        };
        let report = squid_n_design_jp::run_member_design_checks(
            design_model,
            &results.member_forces,
            &results.panel_moments,
            &squid_n_design_jp::MemberDesignCheckOptions {
                term: self.design_term,
                rc_damage_control: self.analysis_cfg.rc_damage_control,
                bond_method: self.analysis_cfg.bond_method,
                qd_method: self.analysis_cfg.qd_method,
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
        // 床の中での小梁・スラブ設計（全体 FEM から独立。小梁は大梁を分割しない）。
        let (joist_checks, slab_checks) = self.floor_design_checks();

        let member_checks = group_member_checks(report.member_checks);

        if let Some(bundle) = self.results.as_mut() {
            bundle.member_checks = member_checks;
            bundle.joint_checks = joint_checks;
            bundle.joist_checks = joist_checks;
            bundle.slab_checks = slab_checks;
        }
    }

    /// 床の中での小梁・スラブ設計を算定する（`run_design_check` から呼ぶ）。
    ///
    /// - 小梁: 支持2節点間を単純支持梁とし、床用積載（令85条1項の床用）＋固定荷重の
    ///   等分布 w·spacing で曲げ・たわみを検定する。反力は大梁へ CMQ として伝達する
    ///   前提のため、小梁は大梁を分割しない。実部材化された小梁（支持間に実 Beam が
    ///   存在）は全体 FEM で検定するため対象外。断面未割当の小梁もスキップする。
    /// - スラブ: 矩形スラブの短辺を設計スパンとし、一方向版として設計曲げモーメントと
    ///   必要鉄筋量を算定する（鋼小梁・SD295 鉄筋の既定値を用いる）。
    pub(crate) fn floor_design_checks(
        &self,
    ) -> (Vec<crate::app::JoistCheck>, Vec<crate::app::SlabCheck>) {
        use squid_n_core::model::LoadPurpose;
        use squid_n_design_jp::floor as fd;

        let mut joist_checks = Vec::new();
        let mut slab_checks = Vec::new();

        let beam_between = |a: NodeId, b: NodeId| -> bool {
            self.model.elements.iter().any(|e| {
                e.kind == squid_n_core::model::ElementKind::Beam
                    && e.nodes.len() == 2
                    && ((e.nodes[0] == a && e.nodes[1] == b)
                        || (e.nodes[0] == b && e.nodes[1] == a))
            })
        };

        let sigma_allow = 235.0 / 1.5; // 鋼の長期許容曲げ応力度 F/1.5（既定 F=235）。
        let z_of = |sid: squid_n_core::ids::SectionId| -> Option<f64> {
            let sec = self.model.sections.get(sid.index())?;
            // 強軸断面係数 Z = Iy / (depth/2)。
            Some(if sec.depth > 0.0 {
                sec.iy / (sec.depth / 2.0)
            } else {
                0.0
            })
        };

        // --- 床領域（大梁の区画）ごとの手入力小梁ライン ---
        // 交差があれば床格子サブモデル（二方向）で、なければ単純支持梁で検定する。
        // 積載は床領域の代表床板（`slab_ids` の先頭。squid-n-load の `distribute_region`
        // と同じ規約）から求める。床領域が床板を持たない場合は検定対象外。
        for region in &self.model.floor_regions {
            let Some(rep_slab) = region.slab_ids.first().and_then(|&id| self.model.slab(id)) else {
                continue;
            };
            let w = self.model.slab_intensity(rep_slab, LoadPurpose::Floor);

            let grillage = squid_n_job::floor_grillage::build_slab_grillage(&self.model, region, w)
                .and_then(|g| {
                    squid_n_job::floor_grillage::solve_grillage(&g.model, LoadCaseId(0))
                        .ok()
                        .map(|sol| (g, sol))
                });
            if let Some((g, sol)) = grillage {
                // 格子 FEM の部材力・たわみで各小梁を検定（十字梁の二方向挙動を反映）。
                for (jidx, span, m, q, defl) in
                    squid_n_job::floor_grillage::joist_design_forces(&g, &sol)
                {
                    let Some(j) = region.joist_lines().get(jidx) else {
                        continue;
                    };
                    let Some(sid) = j.section else { continue };
                    let Some(z) = z_of(sid) else { continue };
                    let r = fd::design_joist_from_forces(
                        span,
                        w * j.spacing,
                        m,
                        q,
                        defl,
                        z,
                        sigma_allow,
                        fd::DEFLECTION_LIMIT_DENOM,
                    );
                    joist_checks.push((
                        rep_slab.id,
                        crate::app::JoistCheckTarget::SlabJoist(jidx),
                        r,
                    ));
                }
            } else {
                // 交差なし: 各小梁を独立した単純支持梁として検定。
                for (ji, j) in region.joist_lines().iter().enumerate() {
                    let (a, b) = (j.support[0], j.support[1]);
                    if a == b || beam_between(a, b) {
                        // 実部材化済み or 退化した小梁は床設計の対象外。
                        continue;
                    }
                    let Some(sid) = j.section else { continue };
                    let Some(z) = z_of(sid) else { continue };
                    let Some(sec) = self.model.sections.get(sid.index()) else {
                        continue;
                    };
                    let (Some(na), Some(nb)) = (
                        self.model.nodes.get(a.index()),
                        self.model.nodes.get(b.index()),
                    ) else {
                        continue;
                    };
                    let span = {
                        let d = [
                            nb.coord[0] - na.coord[0],
                            nb.coord[1] - na.coord[1],
                            nb.coord[2] - na.coord[2],
                        ];
                        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
                    };
                    if span <= 1e-9 {
                        continue;
                    }
                    let r = fd::design_joist_simple(
                        span,
                        w * j.spacing,
                        z,
                        sec.iy,
                        fd::STEEL_YOUNG,
                        sigma_allow,
                        fd::DEFLECTION_LIMIT_DENOM,
                    );
                    joist_checks.push((
                        rep_slab.id,
                        crate::app::JoistCheckTarget::SlabJoist(ji),
                        r,
                    ));
                }
            }
        }

        // --- 床板（一方向版）ごとの検定 ---
        // 「版がある」= 断面割当があり板厚が正（`slab_plate_thickness` が `Some` を返す）。
        // 版なし・厚さ 0 は出さない。
        for slab in &self.model.slabs {
            let Some(thickness) = self.model.slab_plate_thickness(slab) else {
                continue;
            };
            let w = self.model.slab_intensity(slab, LoadPurpose::Floor);
            if slab.is_attached() {
                // 取り付き＋版あり: 片持ち M=wL²/2（coef=2）。
                // スパンは張り出し量の絶対値の大きい方。slab_dimensions が
                // None（台形など）でも出す。
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
            } else if let Some((lx, ly)) = squid_n_load::floor::slab_dimensions(&self.model, slab) {
                use squid_n_core::model::OneWayDir;
                // 囲まれ＋版あり＋矩形: 従来どおり単純支持相当（coef=8）。
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

        // --- 二次部材（小梁）: ST-Bridge 取り込み等 `Slab::joists` に載らない小梁 ---
        self.design_secondary_joist_checks(&mut joist_checks, &beam_between, sigma_allow, &z_of);

        (joist_checks, slab_checks)
    }

    /// 領域内小梁および未割当小梁を、床領域分配（[`squid_n_load::floor::distribute_region`]）
    /// の `LoadTarget::Span` 出力を単純梁として重ね合わせて検定する。
    ///
    /// `FloorRegion.joists`（格子解析用手入力）に同じ両端を持つ小梁はスキップする。
    fn design_secondary_joist_checks(
        &self,
        joist_checks: &mut Vec<crate::app::JoistCheck>,
        beam_between: &impl Fn(squid_n_core::ids::NodeId, squid_n_core::ids::NodeId) -> bool,
        sigma_allow: f64,
        z_of: &impl Fn(squid_n_core::ids::SectionId) -> Option<f64>,
    ) {
        use squid_n_core::model::{LoadPurpose, SecondaryMemberKind};
        use squid_n_design_jp::floor as fd;
        use squid_n_load::floor::{
            orient_member_loads, secondary_joist_distribution_loads, simple_beam_extremes,
            span_node_key,
        };
        use std::collections::HashSet;

        let w_of = |s: &squid_n_core::model::Slab| self.model.slab_intensity(s, LoadPurpose::Floor);
        let distribution = secondary_joist_distribution_loads(&self.model, w_of);

        let mut joist_supports = HashSet::new();
        for region in &self.model.floor_regions {
            for j in region.joist_lines() {
                let (a, b) = (j.support[0], j.support[1]);
                if a != b {
                    joist_supports.insert(span_node_key(a, b));
                }
            }
        }

        for (smi, sm) in self.model.joists().enumerate() {
            if sm.kind != SecondaryMemberKind::Joist {
                continue;
            }
            let Some(sid) = sm.section else { continue };
            let Some(z) = z_of(sid) else { continue };
            let Some(sec) = self.model.sections.get(sid.index()) else {
                continue;
            };

            let (a, b) = (sm.nodes[0], sm.nodes[1]);
            if a == b || beam_between(a, b) {
                continue;
            }
            let key = span_node_key(a, b);
            if joist_supports.contains(&key) {
                continue;
            }

            let (Some(na), Some(nb)) = (
                self.model.nodes.get(a.index()),
                self.model.nodes.get(b.index()),
            ) else {
                continue;
            };
            let span = {
                let d = [
                    nb.coord[0] - na.coord[0],
                    nb.coord[1] - na.coord[1],
                    nb.coord[2] - na.coord[2],
                ];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
            };
            if span <= 1e-9 {
                continue;
            }

            let Some(entry) = distribution.get(&key) else {
                continue;
            };
            if entry.member_loads.is_empty() {
                continue;
            }

            let loads = orient_member_loads(&entry.member_loads, span, entry.span_nodes, (a, b));
            let ex = simple_beam_extremes(&loads, span, fd::STEEL_YOUNG, sec.iy);
            if ex.w_equiv <= 1e-9 && ex.m_max <= 1e-9 {
                continue;
            }

            let rep_slab_id = self
                .model
                .floor_regions
                .iter()
                .find(|r| r.secondary_joists.iter().any(|j| j.nodes == sm.nodes))
                .and_then(|r| r.slab_ids.first().copied())
                .unwrap_or(entry.rep_slab_id);

            let r = fd::design_joist_from_forces(
                span,
                ex.w_equiv,
                ex.m_max,
                ex.q_max,
                ex.deflection,
                z,
                sigma_allow,
                fd::DEFLECTION_LIMIT_DENOM,
            );
            joist_checks.push((
                rep_slab_id,
                crate::app::JoistCheckTarget::SecondaryMember(smi),
                r,
            ));
        }
    }
}
