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
            &self.model,
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

        for slab in &self.model.slabs {
            // 床設計は床用積載（最大）＋固定荷重を用いる。
            let w = self.model.slab_intensity(slab, LoadPurpose::Floor);

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

            // --- 小梁: 交差があれば床格子サブモデル（二方向）で、なければ単純支持梁で検定 ---
            let grillage = squid_n_job::floor_grillage::build_slab_grillage(&self.model, slab, w)
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
                    let Some(j) = slab.joists.get(jidx) else {
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
                    joist_checks.push((slab.id, jidx, r));
                }
            } else {
                // 交差なし: 各小梁を独立した単純支持梁として検定。
                for (ji, j) in slab.joists.iter().enumerate() {
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
                    joist_checks.push((slab.id, ji, r));
                }
            }

            // --- スラブ（一方向版） ---
            if let Some((lx, ly)) = squid_n_load::floor::slab_dimensions(&self.model, slab) {
                use squid_n_core::model::OneWayDir;
                // 設計スパンは伝達方向に一致させる（分配エンジンと同じ規約: X→lx, Y→ly）。
                // 一方向指定がない（両方向）場合は安全側に短辺で設計する。
                let span = match slab.one_way {
                    Some(OneWayDir::X) => lx,
                    Some(OneWayDir::Y) => ly,
                    None => lx.min(ly),
                };
                let thickness = self.model.slab_thickness_of(slab).unwrap_or(0.0);
                if span > 1e-9 && thickness > 0.0 {
                    // 単純支持相当（coef=8）。連続版はより小さい係数だが安全側に 8 を用いる。
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
        (joist_checks, slab_checks)
    }
}
