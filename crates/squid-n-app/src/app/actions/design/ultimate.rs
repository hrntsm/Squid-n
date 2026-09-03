use super::super::*;

impl App {
    /// 終局検定（靭性保証型耐震設計指針）: RC 矩形部材の終局せん断強度（塑性
    /// 理論式）・付着割裂耐力・軸終局耐力に対する余裕度を算定する。
    ///
    /// 柱の曲げ終局強度 Mu・軸余裕度に用いる設計軸力は、長期（G+P 相当）静的
    /// 解析結果（先頭重力ケースを優先、なければ最後に実行した静的解析）の軸力
    /// （圧縮正）を用いる。静的解析結果がない場合は軸力 0（安全側）で評価する。
    ///
    /// 対象 RC 矩形部材が 1 つもない場合は `Err` を返す（UI 側で案内表示）。
    pub fn compute_ultimate_checks(
        &mut self,
    ) -> Result<Vec<squid_n_design_jp::ultimate::UltimateCheck>, String> {
        use squid_n_core::section_shape::SectionShape;

        // 剛域（face_i/j）を内法長さに反映するため自動剛域を適用（冪等）。
        self.apply_rigid_zones_for_analysis();

        let demand = self.ultimate_demand_by_elem();

        let opts = squid_n_design_jp::ultimate::UltimateShearOptions {
            rp: self.core.ultimate_rp.max(0.0),
            lightweight: self.core.ultimate_lightweight,
            upper_strength_factor: self.core.ultimate_upper_factor.max(0.0),
            sigma_wy: 295.0,
            // せん断補強筋の材質は部材ごとに断面から解決するため、共通オプションでは
            // 未指定（普通強度扱い）とし、部材ループ側で上書きする。
            shear_grade: None,
            include_bond: self.core.ultimate_include_bond,
            shear_method: if self.core.ultimate_shear_ductility {
                squid_n_design_jp::ultimate::ShearMethod::Ductility
            } else {
                squid_n_design_jp::ultimate::ShearMethod::Plastic
            },
            biaxial_shear: self.core.ultimate_biaxial_shear,
            biaxial_bending: self.core.ultimate_biaxial_bending,
        };
        let checks = squid_n_design_jp::ultimate::collect_rc_ultimate_checks(
            &self.core.model,
            &demand,
            &opts,
        );

        // RC 矩形部材がない場合の案内。
        let has_rc_rect = self.core.model.elements.iter().any(|e| {
            e.section
                .and_then(|sid| self.core.model.sections.get(sid.index()))
                .and_then(|s| s.shape.as_ref())
                .map(|sh| matches!(sh, SectionShape::RcRect { .. }))
                .unwrap_or(false)
        });
        if checks.is_empty() {
            if has_rc_rect {
                return Err(
                    "RC 矩形部材の終局検定を算定できませんでした（コンクリート強度 Fc の設定・\
                     有効せいを確認してください）。"
                        .to_string(),
                );
            }
            return Err(
                "終局検定の対象（RcRect の RC 矩形部材）がありません。RC 断面を割り当ててください。"
                    .to_string(),
            );
        }
        Ok(checks)
    }

    /// 終局検定用の部材需要（軸力 [N]圧縮正・強軸/弱軸の設計用曲げ [N·mm]）。
    ///
    /// `ultimate_use_pushover` が真で増分解析応答（部材別応答）が得られる場合は、
    /// 終局時の部材別 Qmu（設計用せん断）・需要曲げ・軸力・Rp を直接反映する
    /// （[`Self::ultimate_demand_from_pushover`]）。それ以外は先頭重力ケース（G+P 相当）の
    /// 静的解析結果を優先し、なければ最後に実行した静的解析結果を用いる（軸力は始端値、
    /// 曲げは部材内の最大絶対値、Qmu は両端ヒンジ 2·Mu/内法、Rp は UI 一律指定）。
    /// いずれの応答もなければ空（＝需要 0）。
    fn ultimate_demand_by_elem(&self) -> Vec<(ElemId, squid_n_design_jp::ultimate::MemberDemand)> {
        // 増分解析応答からの直接反映（優先、指定時かつ応答があれば）。
        if self.core.ultimate_use_pushover {
            if let Some(demand) = self.ultimate_demand_from_pushover() {
                return demand;
            }
        }
        // 単純梁せん断 Q0（MK785/SPR785/SPR685 使用部材の QL=Q0 読み替え用）。
        // Dead+LiveSeismic（なければ Live）を加算した長期相当。
        let q0_map = squid_n_job::simple_beam_q0_by_gravity_cases(&self.core.model);
        // QL も同じ重力ケース集合の解析内力を加算する（先頭ケースのみだと Q0 と積載がずれる）。
        let gravity_long = self.core.scoped.results.as_ref().and_then(|r| {
            squid_n_job::sum_analyzed_gravity_member_forces(&self.core.model, |lc| {
                r.statics
                    .iter()
                    .find(|(id, _)| *id == StaticCaseKey::User(lc))
                    .map(|(_, s)| s.member_forces.clone())
            })
        });
        self.core
            .scoped
            .results
            .as_ref()
            .map(|r| {
                let fallback = gravity_cases_for_seismic_weight(&self.core.model)
                    .first()
                    .and_then(|lc| {
                        r.statics
                            .iter()
                            .find(|(id, _)| *id == StaticCaseKey::User(*lc))
                    })
                    .map(|(_, s)| s.member_forces.as_slice())
                    .unwrap_or(r.member_forces.as_slice());
                let member_forces: &[(ElemId, squid_n_element::beam::MemberForces)] =
                    gravity_long.as_deref().unwrap_or(fallback);
                let ql_map = squid_n_job::q_long_map_from_member_forces(member_forces);
                squid_n_job::member_demand_from_static_forces(
                    member_forces,
                    Some(&ql_map),
                    Some(&q0_map),
                )
            })
            .unwrap_or_default()
    }

    /// 増分解析応答（部材別応答）から終局検定用の部材需要を組み立てる。
    ///
    /// 増分解析最終ステップの部材別応答（[`squid_n_solver::pushover::PushoverMemberResponse`]）
    /// から、軸力（圧縮正）・強軸/弱軸の設計用曲げ・強軸設計用せん断・部材別 Rp を
    /// 反映する。増分解析未実行、または部材別応答が空（ステップ未確定）の場合は
    /// `None`（呼び出し側が静的応答へフォールバック）。
    fn ultimate_demand_from_pushover(
        &self,
    ) -> Option<Vec<(ElemId, squid_n_design_jp::ultimate::MemberDemand)>> {
        let po = self.displayed_pushover()?;
        // 長期せん断力 QL（余裕率の分子控除用）を重力ケース集合の静的結果から引く
        // （Q0 と同じ Dead+LiveSeismic／Live 集合。先頭ケースのみだと積載がずれる）。
        let gravity_long = self.core.scoped.results.as_ref().and_then(|res| {
            squid_n_job::sum_analyzed_gravity_member_forces(&self.core.model, |lc| {
                res.statics
                    .iter()
                    .find(|(id, _)| *id == StaticCaseKey::User(lc))
                    .map(|(_, s)| s.member_forces.clone())
            })
        });
        let ql_by_elem: Option<std::collections::HashMap<ElemId, f64>> =
            gravity_long.as_ref().map(|gl| {
                gl.iter()
                    .map(|(id, mf)| {
                        (
                            *id,
                            mf.at.iter().map(|(_, f)| f[1].abs()).fold(0.0, f64::max),
                        )
                    })
                    .collect()
            });
        // 単純梁せん断 Q0（MK785/SPR785/SPR685 使用部材の QL=Q0 読み替え用）。
        // Dead+LiveSeismic（なければ Live）を加算した長期相当。
        let q0_map = squid_n_job::simple_beam_q0_by_gravity_cases(&self.core.model);
        squid_n_job::member_demand_from_pushover(
            &po.member_response,
            ql_by_elem.as_ref(),
            Some(&q0_map),
        )
    }

    /// CFT 柱の軸終局検定（CFT指針）: CftBox/CftPipe 柱の
    /// 軸圧縮終局耐力 Ncu・軸引張終局耐力 Ntu に対する軸余裕度を算定する。
    ///
    /// 対象 CFT 柱が 1 つもない場合は `Err` を返す（UI 側で案内表示）。
    pub fn compute_cft_ultimate_checks(
        &mut self,
    ) -> Result<Vec<squid_n_design_jp::ultimate::CftUltimateCheck>, String> {
        self.apply_rigid_zones_for_analysis();
        // CFT の軸終局検定は軸力のみを用いる（MemberDemand から軸力を取り出す）。
        let axial: Vec<(ElemId, f64)> = self
            .ultimate_demand_by_elem()
            .into_iter()
            .map(|(id, d)| (id, d.n_axial))
            .collect();
        let checks =
            squid_n_design_jp::ultimate::collect_cft_ultimate_checks(&self.core.model, &axial);
        if checks.is_empty() {
            return Err(
                "終局検定の対象（CftBox/CftPipe の CFT 柱）がありません。CFT 断面と\
                 コンクリート強度 Fc を設定してください。"
                    .to_string(),
            );
        }
        Ok(checks)
    }
}
