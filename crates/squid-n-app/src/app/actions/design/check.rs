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
        // 積載は区画の代表床板（`slab_ids` の先頭。squid-n-load の `distribute_region`
        // と同じ規約）から求める。区画が床板を持たない場合は検定対象外。
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

    /// `Model::secondary_members` の小梁を単純支持梁として検定する。
    ///
    /// `FloorRegion::joists` に同じ両端を持つ小梁は GUI 定義済みとしてスキップする。
    /// 積載強度は中点が属する床板を XY 多角形判定（内部または辺上）**かつ同じレベル**で
    /// 決める。負担幅は小梁の辺を境界に持つ床板の幾何から求め
    /// （`squid_n_load::secondary::joist_edge_tributary_width`）、どの床板の境界にも
    /// 載らない小梁だけ平行小梁群から算定する（フォールバック）。
    fn design_secondary_joist_checks(
        &self,
        joist_checks: &mut Vec<crate::app::JoistCheck>,
        beam_between: &impl Fn(squid_n_core::ids::NodeId, squid_n_core::ids::NodeId) -> bool,
        sigma_allow: f64,
        z_of: &impl Fn(squid_n_core::ids::SectionId) -> Option<f64>,
    ) {
        use squid_n_core::ids::{SectionId, SlabId};
        use squid_n_core::model::{LoadPurpose, SecondaryMemberKind};
        use squid_n_design_jp::floor as fd;
        use std::collections::{HashMap, HashSet};

        let mut joist_supports = HashSet::new();
        for region in &self.model.floor_regions {
            for j in region.joist_lines() {
                let (a, b) = (j.support[0], j.support[1]);
                if a != b {
                    let key = if a.0 <= b.0 { (a, b) } else { (b, a) };
                    joist_supports.insert(key);
                }
            }
        }

        struct Cand {
            sm_idx: usize,
            slab_id: SlabId,
            slab_idx: usize,
            span: f64,
            perp_coord: f64,
            dir: [f64; 2],
            section: SectionId,
            /// 小梁の辺を境界に持つ床板の幾何から求めた負担幅
            /// （`joist_edge_tributary_width`）。`None` はどの床板の境界にも
            /// 載らない小梁（1 枚の床板の内部にある等）で、平行小梁群から
            /// 負担幅を出す。
            edge_width: Option<f64>,
        }

        let mut candidates = Vec::new();

        for (smi, sm) in self.model.secondary_members.iter().enumerate() {
            if sm.kind != SecondaryMemberKind::Joist {
                continue;
            }
            let Some(sid) = sm.section else { continue };
            if z_of(sid).is_none() {
                continue;
            }

            let (a, b) = (sm.nodes[0], sm.nodes[1]);
            if a == b || beam_between(a, b) {
                continue;
            }
            let key = if a.0 <= b.0 { (a, b) } else { (b, a) };
            if joist_supports.contains(&key) {
                continue;
            }

            let (Some(na), Some(nb)) = (
                self.model.nodes.get(a.index()),
                self.model.nodes.get(b.index()),
            ) else {
                continue;
            };

            let dx = nb.coord[0] - na.coord[0];
            let dy = nb.coord[1] - na.coord[1];
            let span = (dx * dx + dy * dy).sqrt();
            if span <= 1e-9 {
                continue;
            }
            let dir = [dx / span, dy / span];
            let mid = [
                (na.coord[0] + nb.coord[0]) / 2.0,
                (na.coord[1] + nb.coord[1]) / 2.0,
            ];
            let mid_z = (na.coord[2] + nb.coord[2]) / 2.0;

            // 中点を含み、かつ**同じレベルにある**床板を集める。XY だけで判定すると
            // 上下階の床板は平面上で重なるため、別階の床板を掴み、別階の板厚・室用途・
            // 境界寸法で検定してしまう（エラーは出ないまま結果だけが誤る）。
            // レベルの許容差は面走査による領域境界検出と同じ `geom::LEVEL_TOL_MM` を用いる
            // （丸め誤差だけを吸収する幅。段差床は別レベルとして扱う＝該当なしになる）。
            let matched: Vec<(usize, &squid_n_core::model::Slab)> = self
                .model
                .slabs
                .iter()
                .enumerate()
                .filter(|(_, s)| {
                    s.level(&self.model)
                        .is_some_and(|z| (z - mid_z).abs() <= squid_n_core::geom::LEVEL_TOL_MM)
                        && squid_n_load::floor::point_in_slab_boundary(&self.model, s, mid)
                })
                .collect();
            let Some(&(slab_idx, slab)) = matched.first() else {
                continue;
            };

            let perp = [-dir[1], dir[0]];
            let perp_coord = mid[0] * perp[0] + mid[1] * perp[1];

            // 負担幅は、小梁の辺を境界に持つ床板の幾何から求める
            // （`squid_n_load::secondary::joist_edge_tributary_width`）。ST-Bridge
            // 取り込みの床板は小梁で分割された小片として入ってくるため、通常は
            // これで一意に決まる（片側が複数枚に割れている T 字取り付きも、側ごとの
            // 合計幅の半分として正しく扱う。片側 1 枚を代表に選ぶ・全体を単純平均
            // するといった近似はしない）。どの床板の境界にも載らない小梁（1 枚の
            // 床板の内部を通る等）は `None` になるため、そのときだけ下の平行小梁群
            // からの近似（`n == 1` 経路）へフォールバックする。
            let edge_width = squid_n_load::secondary::joist_edge_tributary_width(&self.model, a, b);

            candidates.push(Cand {
                sm_idx: smi,
                slab_id: slab.id,
                slab_idx,
                span,
                perp_coord,
                dir,
                section: sid,
                edge_width,
            });
        }

        fn dir_key(d: [f64; 2]) -> (i64, i64) {
            let mut dx = d[0];
            let mut dy = d[1];
            if dx < 0.0 || (dx.abs() < 1e-9 && dy < 0.0) {
                dx = -dx;
                dy = -dy;
            }
            ((dx * 1000.0).round() as i64, (dy * 1000.0).round() as i64)
        }

        let mut groups: HashMap<(usize, (i64, i64)), Vec<usize>> = HashMap::new();
        for (ci, c) in candidates.iter().enumerate() {
            groups
                .entry((c.slab_idx, dir_key(c.dir)))
                .or_default()
                .push(ci);
        }

        for (_, mut sorted) in groups {
            sorted.sort_by(|&a, &b| {
                candidates[a]
                    .perp_coord
                    .total_cmp(&candidates[b].perp_coord)
            });

            let slab = &self.model.slabs[candidates[sorted[0]].slab_idx];
            let perp = [-candidates[sorted[0]].dir[1], candidates[sorted[0]].dir[0]];
            let (pmin, pmax) = slab_perp_extent(&self.model, slab, perp);
            let n = sorted.len();

            for (ri, &ci) in sorted.iter().enumerate() {
                let c = &candidates[ci];
                let spacing = if let Some(w) = c.edge_width {
                    // 床板の境界の幾何から求めた負担幅（`joist_edge_tributary_width`）。
                    w
                } else if n == 1 {
                    slab_width_across(&self.model, slab, c.dir, pmax - pmin)
                } else {
                    let left = if ri == 0 {
                        c.perp_coord - pmin
                    } else {
                        (c.perp_coord - candidates[sorted[ri - 1]].perp_coord) / 2.0
                    };
                    let right = if ri == n - 1 {
                        pmax - c.perp_coord
                    } else {
                        (candidates[sorted[ri + 1]].perp_coord - c.perp_coord) / 2.0
                    };
                    left + right
                };
                if spacing <= 1e-9 {
                    continue;
                }

                let Some(z) = z_of(c.section) else { continue };
                let Some(sec) = self.model.sections.get(c.section.index()) else {
                    continue;
                };
                let w = self
                    .model
                    .slab_intensity(&self.model.slabs[c.slab_idx], LoadPurpose::Floor);
                let r = fd::design_joist_simple(
                    c.span,
                    w * spacing,
                    z,
                    sec.iy,
                    fd::STEEL_YOUNG,
                    sigma_allow,
                    fd::DEFLECTION_LIMIT_DENOM,
                );
                joist_checks.push((
                    c.slab_id,
                    crate::app::JoistCheckTarget::SecondaryMember(c.sm_idx),
                    r,
                ));
            }
        }
    }
}

/// 床板の境界の直交軸座標の最小・最大（負担幅算定用）。
fn slab_perp_extent(
    model: &squid_n_core::model::Model,
    slab: &squid_n_core::model::Slab,
    perp: [f64; 2],
) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for c in slab.boundary_coords(model).unwrap_or_default() {
        let t = c[0] * perp[0] + c[1] * perp[1];
        min = min.min(t);
        max = max.max(t);
    }
    (min, max)
}

/// 小梁の向き `dir` に直交する方向のスラブ幅。矩形スラブは `slab_dimensions` の
/// 軸直交寸法、それ以外は境界 bbox の `bbox_extent` を用いる。
///
/// どの床板の境界にも載らない小梁（床板の内部にある等）向けのフォールバック専用。
/// 境界に載る小梁の負担幅は `joist_edge_tributary_width` が求める。
fn slab_width_across(
    model: &squid_n_core::model::Model,
    slab: &squid_n_core::model::Slab,
    dir: [f64; 2],
    bbox_extent: f64,
) -> f64 {
    if let Some((lx, ly)) = squid_n_load::floor::slab_dimensions(model, slab) {
        if dir[0].abs() >= dir[1].abs() {
            ly
        } else {
            lx
        }
    } else {
        bbox_extent
    }
}
