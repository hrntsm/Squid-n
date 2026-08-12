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
            rc_wall_type, steel_brace_type, GroupType,
        };
        use squid_n_design_jp::secondary::holding_capacity::{
            check_holding_capacity, qud_by_story, MemberRank,
        };
        use squid_n_design_jp::secondary::member_rank::worst_rank;
        use squid_n_design_jp::secondary::rc_capacity::{
            rc_column_mu_simple, rc_qmu_simple, rc_qsu_simple,
        };
        use squid_n_design_jp::steel_f_value_prefix;
        use squid_n_solver::pushover::MechanismType;

        // rigid_zone（剛域長・face_i/j）を読むため、算定前に自動剛域を反映する
        // （設計書 §6.2.1、冪等なので他の解析エントリと重複して呼んでも安全）。
        self.apply_rigid_zones_for_analysis();

        if self.model.stories.is_empty() {
            return Err(
                "階が未定義です。解析タブの「準備計算 実行」を行ってください。".to_string(),
            );
        }
        let po = self
            .results
            .as_ref()
            .and_then(|r| r.pushover.as_ref())
            .ok_or_else(|| {
                "増分解析未実行です。解析タブから増分解析を実行してください。".to_string()
            })?;
        let st = self.current_static().ok_or_else(|| {
            "静的解析結果がありません。地震静的(Ai)を実行してください。".to_string()
        })?;

        let ctx = crate::summary::metrics_ctx_from_results(self.results.as_ref());
        let metrics = crate::summary::compute_story_metrics_with(
            &self.model,
            &st.disp,
            self.analysis_cfg.seismic_dir,
            &ctx,
        );

        // 地震重量: 下層→上層順（層の重量は上端の階が持つ。`Model::layers` 参照）。
        let layers = self.model.layers();
        let weights: Vec<f64> = layers.iter().map(|l| l.weight.unwrap_or(0.0)).collect();
        if weights.iter().any(|w| *w <= 0.0) {
            return Err(
                "地震重量が未設定です。解析タブの「準備計算 実行」を行ってください。".to_string(),
            );
        }

        // T(1 次周期): 固有値解析があればそれを使用、なければ略算式
        // T = h(0.02+0.01α)。h は建築物の高さ（GL〜PH 階を除く最上階）、
        // α は鉄骨造比（令88条・告示1793号。従来は α=0.0 固定・h は生の
        // 最上階 Z 標高で、S 造モデルや地下階付きモデルの T を誤っていた）。
        let t = self
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.period.first().copied())
            .unwrap_or_else(|| {
                let height_m = length_m(squid_n_solver::analysis::building_height_mm(&self.model));
                let steel_ratio = squid_n_solver::analysis::steel_height_ratio(&self.model);
                squid_n_load::ai::approx_t(height_m, steel_ratio)
            });
        let rt = squid_n_load::ai::rt(t, squid_n_load::ai::tc_of(self.analysis_cfg.soil));
        let qud = qud_by_story(&weights, self.analysis_cfg.z, rt, t);

        let n_stories = weights.len();

        // 終局（崩壊機構形成）時の部材別応答。告示の RC 部材種別が要求する
        // 「Ds 算定時に断面に生じる」平均せん断応力度 τu・軸方向応力度 σ0 と、
        // βu（耐力壁・筋かいの水平耐力の和）の集計に用いる。
        let resp_by_elem: std::collections::HashMap<
            ElemId,
            squid_n_solver::pushover::PushoverMemberResponse,
        > = po.member_response.iter().map(|r| (r.elem, *r)).collect();
        // 増分解析でせん断降伏が記録された部材（SRC 柱・SRC 耐震壁の
        // 「破壊モードがせん断破壊か」の判定に用いる）。
        let shear_yield_elems: std::collections::HashSet<ElemId> =
            po.shear_yields.iter().map(|s| s.elem).collect();
        // 層別の保有水平耐力 Qu（性能曲線の層別ピーク層せん断）。βu の分母。
        let story_qu: Vec<f64> = (0..n_stories)
            .map(|i| {
                po.capacity_curve
                    .iter()
                    .filter_map(|p| p.story_shear.get(i).copied())
                    .fold(0.0_f64, f64::max)
            })
            .collect();
        // 層ごとの「柱・はり」および「耐力壁・筋かい」の (種別インデックス, 水平耐力)。
        // 部材群としての種別（耐力比 γA/γC）と βu の算定に用いる。
        let mut cb_members: Vec<Vec<(u8, f64)>> = vec![Vec::new(); n_stories];
        let mut wall_members: Vec<Vec<(u8, f64)>> = vec![Vec::new(); n_stories];
        let mut wall_horizontal: Vec<f64> = vec![0.0; n_stories];

        let (story_ranks, member_ranks): (Vec<MemberRank>, Vec<(ElemId, MemberRank)>) = if self
            .design_rank_auto
        {
            // 鋼部材は幅厚比、RC 矩形部材はせん断余裕度 Qsu/Qmu の略算から
            // ランクを算定し、所属階ごとに集計する。
            //
            // 所属階の規則: 部材の節点のうち最も高い階(story index 最大)。
            // story_gen::generate_stories は各節点をその節点自身の標高が属する
            // レベルへ割り当てる（柱下端は下階または基部=None、柱上端は上階、
            // 梁は両端とも同一階）ため、柱は自動的に上端側の階（＝各節点の
            // story のうち最大値）に算入される。
            let mut per_story: Vec<Vec<MemberRank>> = vec![Vec::new(); n_stories];
            let mut computed: Vec<(ElemId, MemberRank)> = Vec::new();
            // 長期軸力の簡易近似として使う荷重ケースの id
            // （`generate_stories_action` の gravity_lcs と同じ規則。§1.7:
            // kind による選択の先頭を採用。従来の「先頭ケース」規則は
            // 種別が未設定のモデルに対する後方互換フォールバックとして残る）。
            let gravity_lc = gravity_cases_for_seismic_weight(&self.model)
                .first()
                .copied();
            for elem in &self.model.elements {
                let Some(sec) = elem
                    .section
                    .and_then(|sid| self.model.sections.get(sid.index()))
                else {
                    continue;
                };
                let Some(mat) = self.model.element_material(elem) else {
                    continue;
                };
                // 主筋・せん断補強筋・内蔵鉄骨の材料も断面が持つ。
                let rebar_mat = self.model.element_rebar_material(elem);
                let shear_mat = self.model.element_shear_rebar_material(elem);
                let steel_grade = self
                    .model
                    .element_steel_material(elem)
                    .map(|m| m.name.clone())
                    .unwrap_or_default();
                // 筋かい（軸材）は幅厚比ではなく**有効細長比**で種別を定める
                // （告示「筋かいの種別」表: BA/BB/BC）。要素種別が Brace のもの、
                // または斜材として判定されたものを対象とする。従来は柱・梁と同じ
                // 幅厚比表（梁の行）で判定しており、細長い筋かい（BC＝最も不利）を
                // FA と甘く判定して Ds を過小評価する危険側の誤りだった。
                let is_brace_elem =
                    matches!(elem.kind, squid_n_core::model::ElementKind::Brace { .. })
                        || squid_n_design_jp::MemberKind::of_element(elem, &self.model)
                            == squid_n_design_jp::MemberKind::Brace;
                let elem_steel = elem_is_steel(elem, &self.model);
                let rank = if is_brace_elem && elem_steel {
                    // 有効細長比 λ = Lk/i（節点間長を座屈長さ、i=√(Imin/A) とする
                    // ピン支持の軸材モデル）。断面性能がない場合はスキップ。
                    let len = self.model.member_length(elem);
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
                        sec.shape.as_ref().map(steel_max_thickness).unwrap_or(0.0),
                    )
                    .unwrap_or(235.0);
                    steel_brace_type(len / radius, f_value)
                } else if elem_steel {
                    // 鋼部材: 形状情報がない断面(カタログ数値直入力等)はスキップ。
                    let Some(shape) = sec.shape.as_ref() else {
                        continue;
                    };
                    // 幅厚比による部材ランク判定は準備計算の表示と共通
                    // （`steel_width_thickness_rank`。構造規定の幅厚比表のみで判定し、
                    // 表の対象外形状は未判定＝選択ランクへのフォールバックとする）。
                    let member_use = steel_member_use_of(elem, &self.model);
                    let Some(rank) = steel_width_thickness_rank(shape, member_use, &mat.name)
                    else {
                        continue;
                    };
                    rank
                } else if matches!(sec.shape.as_ref(), Some(SectionShape::SrcRect { .. })) {
                    // SRC 柱: 技術基準解説書 表 2.6.6-5（N/N0・sM0/M0・破壊モード）。
                    // SRC 梁の種別表は原典に規定がないためスキップ（層は選択ランクへ
                    // フォールバック）。破壊モードは増分解析のせん断降伏イベントの
                    // 有無で判定し、N はメカニズム時軸力（圧縮正）を用いる。
                    use squid_n_design_jp::secondary::src_rank::{
                        src_column_rank, src_column_rank_ratios,
                    };
                    if squid_n_design_jp::MemberKind::of_element(elem, &self.model)
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
                    if wall_has_src_boundary_column(elem, &self.model) {
                        // SRC 耐震壁（側柱が SRC の壁）: 技術基準解説書の規定により
                        // 破壊モードがせん断破壊の場合を WC、それ以外を WA とする
                        // （τu/Fc の表は用いない）。
                        //
                        // 破壊モードの判定: 増分解析の壁要素は面内せん断を終局せん断
                        // 強度 Qu で頭打ちにする弾完全塑性のため、終局時の負担水平力が
                        // Qu に達していれば「せん断破壊」とみなす。線材のせん断降伏
                        // イベント（shear_yields）は 2 節点要素のみが対象で、4 節点の
                        // 壁要素はそちらでは検出できない。Qu を算定できない壁
                        // （耐震壁不成立等）と終局時応答がない壁は判定不能として
                        // スキップ（層の選択ランクへフォールバック）。
                        let qu = squid_n_element::wall_panel::WallPanelElement::shear_capacity_of(
                            elem,
                            &self.model,
                        );
                        let Some(resp) = resp_by_elem.get(&elem.id) else {
                            continue;
                        };
                        if qu <= 0.0 {
                            continue;
                        }
                        // 頭打ち到達の判定は数値誤差を見込み 99% で切る（過検出側＝
                        // WC 寄りは Ds を大きくする安全側）。
                        let shear_failure = resp.horizontal_force >= 0.99 * qu;
                        squid_n_design_jp::secondary::src_rank::src_wall_type(shear_failure)
                    } else {
                        // RC 耐力壁: 告示「耐力壁の種別」表（τu/Fc により WA〜WD）。
                        // τu は Ds 算定時（増分解析＝プッシュオーバー終局時）に壁断面に生じる
                        // 平均せん断応力度 = 負担水平力 /(壁厚・壁長)。
                        let Some(fc) = mat.fc else {
                            continue;
                        };
                        let Some(resp) = resp_by_elem.get(&elem.id) else {
                            continue;
                        };
                        // 壁長 lw は壁エレメント要素と同じ幾何（`wall_panel_geometry`）を
                        // 用いる。節点は標高 z で下辺・上辺に分けられ（`ElementData::nodes` の
                        // 並び順には依存しない）、lw は**上下辺長さの平均**となる
                        // （台形壁では上下辺長が異なるため一方の辺では代表長さにならない）。
                        let Some(wgeom) =
                            squid_n_element::wall_panel::wall_panel_geometry(elem, &self.model)
                        else {
                            continue;
                        };
                        let wall_len = wgeom.lw;
                        let area = thickness * wall_len;
                        if area <= 0.0 || fc <= 0.0 {
                            continue;
                        }
                        // 壁式構造か否かは設計設定（設計タブのチェックボックス）による。
                        // 告示「耐力壁の種別」表は壁式構造で限界値が厳しくなる。
                        let wall_structure = self.wall_structure;
                        rc_wall_type((resp.horizontal_force / area) / fc, wall_structure, false)
                    }
                } else {
                    // RC 部材: RcRect のみ対応。RcCircle・形状未設定・
                    // コンクリート強度(fc)未設定の材料はスキップ(選択値へフォールバック)。
                    let Some(SectionShape::RcRect { b, d, rebar }) = sec.shape.as_ref() else {
                        continue;
                    };
                    // 内法スパン = 幾何長 − 両端フェイス距離(直交材せい/2)。
                    // 剛域長(D_orth/2 − D_self/4)を引いた可撓長さとは別物
                    // （設計書 §6.2.1）。フェイス距離の合計が幾何長以上になる
                    // (不整合な入力)場合は下限0を割り込むため、幾何長のままとする。
                    let geom_len = self.model.member_length(elem);
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
                    // σ0: 長期軸力の簡易近似として先頭荷重ケース(gravity_lc)の
                    // 静的解析結果を優先し、なければ最後に実行した静的解析結果
                    // (self.results.member_forces)から当該部材の軸力を引き、
                    // 圧縮のときのみ設定する。
                    let sigma_0 = self
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
                    let kind = squid_n_design_jp::MemberKind::of_element(elem, &self.model);
                    // 告示の部材種別が要求する σ0・τu は「Ds 算定時」＝崩壊機構形成時の
                    // 応力度である。終局時応答が得られない部材は判定不能としてスキップ
                    // し、層は選択ランクへフォールバックする（τu=0 とみなすと FA と
                    // 甘く判定され危険側になるため）。
                    let Some(resp) = resp_by_elem.get(&elem.id) else {
                        continue;
                    };
                    let gross = *b * *d;
                    if gross <= 0.0 || input.fc <= 0.0 {
                        continue;
                    }
                    // せん断余裕度（脆性破壊判定）にも Ds 算定時の軸力を用いる。
                    // 長期軸力（sigma_0）で Qsu/Qmu を評価しつつ表の σ0 には終局時
                    // 軸力を使うと基準が食い違うため、終局時軸力で統一する。
                    let sigma_0_ult = resp.axial / gross;
                    let _ = sigma_0; // 長期軸力は Ds 算定時の評価には用いない
                    input.sigma_0 = sigma_0_ult;
                    // 曲げ終局時せん断 Qmu: 柱は軸力を考慮した終局曲げ Mu
                    // （`rc_column_mu_simple`）から算定する。せん断側 Qsu には既に σ0 を
                    // 反映しているため、曲げ側にも同じ軸力を反映しないと、圧縮軸力を
                    // 受ける柱で「Qmu を梁式（軸力無視）で過小評価しつつ Qsu を軸力で増大」
                    // させ、せん断余裕度 Qsu/Qmu を過大評価→ランクを甘く（FA 寄りに）
                    // 判定する危険側の誤りとなる。梁は従来どおり梁式 Qmu を用いる。
                    let qmu = match kind {
                        squid_n_design_jp::MemberKind::Column => {
                            let ag = squid_n_core::section_shape::bar_set_area(&rebar.main_x);
                            let n_axial = resp.axial; // 終局時軸力（圧縮正 [N]）
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

                    // 平均せん断応力度 τu は強軸・弱軸の大きい方で評価する。
                    // 強軸せん断のみを見ると、弱軸方向に加力される柱で τu を過小評価し
                    // 部材種別を甘く判定する危険側になる。
                    let shear_u = resp.shear_strong.max(resp.shear_weak);
                    let tau_over_fc = (shear_u / gross) / input.fc;
                    // 脆性破壊（せん断破壊・付着割裂等の急激な耐力低下）の判定:
                    // 終局せん断強度が曲げ終局時せん断を下回る＝せん断先行。
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
                // 部材が属する層は、材端節点のうち最も高い節点の所属階を上端と
                // する層（`Model::layers`。層 i の上端は `stories[i + 1]` なので
                // 層番号は階の添字 − 1）。両端とも基部の階にある部材（基礎梁）は
                // どの層にも属さないためスキップする。
                let Some(story_idx) = elem
                    .nodes
                    .iter()
                    .filter_map(|nid| self.model.nodes.get(nid.index()))
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

                // 部材群としての種別（耐力比 γA/γC）と βu の集計。
                // 「部材の耐力」には終局時に当該部材が負担する加力方向の水平力を用いる。
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
            // 階ごとの代表ランク = 算定できた部材ランクの最悪値。
            // 1 本も算定できなかった層は手動選択ランクへフォールバックし、
            // 該当層を表示用に記録する（選択ランク（既定 FA）が実状より甘いと
            // Ds を過小評価する危険側となるため、設計タブで警告する）。
            let mut fallback_stories: Vec<String> = Vec::new();
            let ranks: Vec<MemberRank> = per_story
                .into_iter()
                .enumerate()
                .map(|(i, rs)| {
                    worst_rank(&rs).unwrap_or_else(|| {
                        if let Some(s) = self.model.stories.get(i) {
                            fallback_stories.push(s.name.clone());
                        }
                        self.design_rank
                    })
                })
                .collect();
            self.ds_rank_fallback_stories = fallback_stories;
            (ranks, computed)
        } else {
            // 自動判定 OFF は全層が選択ランクによる明示運用のため、警告対象の
            // フォールバックではない。
            self.ds_rank_fallback_stories = Vec::new();
            (vec![self.design_rank; n_stories], Vec::new())
        };

        // Ds は告示の「各階の Ds」表（耐力壁／筋かいの部材群としての種別 × βu ×
        // 柱及びはりの部材群としての種別）で層ごとに定める。
        //
        // - 部材群としての種別は耐力比 γA/γC（[`member_group`]）で判定する。終局時の
        //   部材水平力が得られず判定できない層は、代表ランク（最不利部材）を種別へ
        //   読み替えるフォールバックとする。
        // - βu = 耐力壁・筋かいが負担する水平力の和 / 保有水平耐力 Qu（層別）。
        // - 崩壊機構補正: 層崩壊形の層は柱はり群種別を 1 段階不利側へ移す（告示表は
        //   全体崩壊形の形成を前提とするため。部分崩壊形＝機構未形成は補正せず UI で
        //   暫定値である旨を警告する）。
        //
        // 旧実装は架構種別 4 種 × ランク 4 段の 2 軸表（`ds_value`）で、βu と部材群
        // 種別を反映していなかったため、βu の大きい架構や壁・筋かい種別が不利な架構で
        // Ds を最大 0.10〜0.15 過小評価する危険側の誤りがあった。
        let mechanism = &po.mechanism;
        let is_rc_frame = matches!(
            self.design_frame,
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
                let rep_rank = story_ranks.get(i).copied().unwrap_or(self.design_rank);
                // 柱はり群種別（耐力比で判定。判定不能なら代表ランクから読み替え）。
                let mut cb_group =
                    member_group(&cb_members[i]).unwrap_or_else(|| fallback_group(rep_rank));
                // 崩壊機構補正: 当該層が層崩壊形なら 1 段階不利側へ。
                if let MechanismType::StoryCollapse { layer } = mechanism {
                    if *layer == i {
                        cb_group = match cb_group {
                            GroupType::A => GroupType::B,
                            GroupType::B => GroupType::C,
                            GroupType::C | GroupType::D => GroupType::D,
                        };
                    }
                }
                // 耐力壁・筋かいの群種別と βu。壁・筋かいがない層は βu=0（純ラーメン）。
                let wall_group = member_group(&wall_members[i]).unwrap_or(GroupType::A);
                let qu_i = story_qu.get(i).copied().unwrap_or(0.0);
                let beta_u = if qu_i > 0.0 {
                    (wall_horizontal[i] / qu_i).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                // 架構種別として耐力壁付き／筋かい付きが選択されているのに、当該層で
                // 耐力壁・筋かいが 1 枚も検出できなかった場合、βu=0（純ラーメン）の行を
                // 使うと Ds を過小評価する（例: RC 壁付きで 0.35 → 0.30）。βu を算定
                // できないことを明示し、従来の架構種別別 Ds 表へフォールバックする。
                let declares_wall_or_brace = matches!(
                    self.design_frame,
                    squid_n_design_jp::secondary::holding_capacity::FrameType::RcWall
                        | squid_n_design_jp::secondary::holding_capacity::FrameType::SteelBrace
                );
                if declares_wall_or_brace && wall_members[i].is_empty() {
                    beta_u_unavailable = true;
                    return squid_n_design_jp::secondary::holding_capacity::ds_value(
                        self.design_frame,
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
        self.ds_beta_u_by_story = beta_u_by_story;
        self.ds_beta_u_unavailable = beta_u_unavailable;

        let heights: Vec<f64> = metrics.iter().map(|m| m.height).collect();
        let rs: Vec<f64> = metrics.iter().map(|m| m.rs).collect();
        let re: Vec<f64> = metrics.iter().map(|m| m.re).collect();
        let fes: Vec<f64> = metrics.iter().map(|m| m.fes).collect();

        let result =
            check_holding_capacity(po, &qud, &ds_vec, &fes, &rs, &re, &heights, member_ranks);
        Ok((result, story_ranks))
    }
}
