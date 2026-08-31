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
    /// - 二次部材小梁: 二次部材の反力の逐次伝達（[`squid_n_load::cascade`]）が求めた荷重
    ///   （床領域分配の辺荷重・自重・架け側から渡された集中荷重）を単純梁として検定する。
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

        let beam_between = |a: NodeId, b: NodeId| -> bool {
            self.model.elements.iter().any(|e| {
                e.kind == squid_n_core::model::ElementKind::Beam
                    && e.nodes.len() == 2
                    && ((e.nodes[0] == a && e.nodes[1] == b)
                        || (e.nodes[0] == b && e.nodes[1] == a))
            })
        };

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

        // --- 二次部材小梁（領域内 + 未割当） ---
        self.design_secondary_joist_checks(&mut joist_checks, &beam_between);

        (joist_checks, slab_checks)
    }

    /// 領域内小梁および未割当小梁を、二次部材の反力の逐次伝達
    /// （[`squid_n_load::cascade`]）が求めた荷重で検定する。
    ///
    /// 荷重は床領域分配の辺荷重・自重・**架け側の二次部材から渡された集中荷重**の
    /// 重ね合わせである。**荷重同期（`squid-n-job::auto_loads`）と同じ経路を使う**
    /// （判定が 2 か所に分かれると「解析では受け側が架け側の反力を受けているのに、
    /// 検定では受けていない」という食い違いになるため。申し送り §3.4 F6）。
    ///
    /// ただし**荷重の値そのものは一致しない**。共有するのは支持関係の判定と伝達の
    /// 手順であって、面荷重強度は用途ごとに違う。検定は床用（`LoadPurpose::Floor`。
    /// 固定＋床用積載）、荷重同期は固定荷重ケースと積載荷重ケースへ分けて解く。
    ///
    /// 断面未割当・鋼以外の材料・分配が足りないものは表に「未」として残す。
    fn design_secondary_joist_checks(
        &self,
        joist_checks: &mut Vec<crate::app::JoistCheck>,
        beam_between: &impl Fn(squid_n_core::ids::NodeId, squid_n_core::ids::NodeId) -> bool,
    ) {
        use squid_n_core::model::{LoadPurpose, SecondaryMemberKind};
        use squid_n_design_jp::floor as fd;
        use squid_n_load::floor::{simple_beam_extremes, span_node_key};

        let w_of = |s: &squid_n_core::model::Slab| self.model.slab_intensity(s, LoadPurpose::Floor);
        let transfer = squid_n_load::cascade::solve(&self.model, w_of, true);

        for sm in self.model.joists() {
            if sm.kind != SecondaryMemberKind::Joist {
                continue;
            }

            let (a, b) = (sm.nodes[0], sm.nodes[1]);
            if a == b || beam_between(a, b) {
                continue;
            }
            let key = span_node_key(a, b);
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

            let target = crate::app::JoistCheckTarget::SecondaryJoist { nodes: sm.nodes };
            let entry = transfer.members.get(&key);
            let region_slab = self
                .model
                .floor_regions
                .iter()
                .find(|r| {
                    r.secondary_joists
                        .iter()
                        .any(|j| span_node_key(j.nodes[0], j.nodes[1]) == key)
                })
                .and_then(|r| r.slab_ids.first().copied());
            let slab_id = region_slab.or_else(|| entry.and_then(|e| e.rep_slab_id));

            let Some(sid) = sm.section else {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            };
            let Some(sec) = self.model.sections.get(sid.index()) else {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            };
            let z = if sec.depth > 0.0 {
                sec.iy / (sec.depth / 2.0)
            } else {
                0.0
            };
            let mat = self.model.secondary_material(sm);
            let Some((e, ft)) = fd::joist_steel_e_and_ft(mat) else {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            };

            // 逐次伝達の対象外（循環・実部材化済み）、または床分配が足りない本は「未」。
            let Some(entry) = entry.filter(|e| e.distribution_ready) else {
                joist_checks.push((slab_id, target, fd::joist_unchecked(span)));
                continue;
            };
            // `member_loads` は二次部材の節点順（`nodes`）に揃っている。
            let ex = simple_beam_extremes(&entry.member_loads, span, e, sec.iy);
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
