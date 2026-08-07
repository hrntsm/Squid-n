//! 断面検定・接合部検定ジョブの純粋計算。
//!
//! - [`compute_design_check_job`] — DesignCheck ジョブの純粋計算部分。
//! - [`member_kind_of`] — 部材種別判定（priv）。
//! - [`elem_geometric_length`] — 部材両端節点間の幾何長（priv）。
//! - [`design_positions`] — 危険断面位置を正規化座標で算定する（priv）。
//! - [`is_near_design_position`] — `pos` が危険断面位置のいずれかと一致するか判定する（priv）。

use super::{model_with_auto_rigid_zones, resolve_load_case, JobOutcome};
use squid_n_core::model::Model;

/// 部材種別判定（部材軸の鉛直成分による幾何判定）。
/// squid-n-app の `member_kind_of`（app.rs）と同じロジック。
fn member_kind_of(
    elem: &squid_n_core::model::ElementData,
    model: &Model,
) -> squid_n_design_jp::MemberKind {
    use squid_n_design_jp::MemberKind;
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return MemberKind::Beam;
    };
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-9 {
        return MemberKind::Beam;
    }
    let ez = (dz / len).abs();
    if ez >= 0.8 {
        MemberKind::Column
    } else if ez <= 0.2 {
        MemberKind::Beam
    } else {
        MemberKind::Brace
    }
}

/// 部材両端節点間の幾何長 \[mm\]（内法補正なしの簡易値。剛域等は考慮しない）。
/// squid-n-app の `elem_geometric_length`（app.rs）と同じロジック
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
fn elem_geometric_length(elem: &squid_n_core::model::ElementData, model: &Model) -> f64 {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return 0.0;
    };
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// 危険断面位置（§6.2.3、既定は柱フェイスと中央）を正規化座標 \[0,1\] で算定する。
/// `squid_n_element::beam::BeamElement::new` の `eval_sections` 算定と同じ規則
/// （xi_i は \[0.0, 0.5) へ、xi_j は (0.5, 1.0\] へクランプ）で face_i/face_j から
/// 求める。face=0（直交材がない端）では節点芯（0.0/1.0）と一致する。
/// 部材付帯情報（ハンチ・継手位置。剛性には影響しない）があれば、その追加検定
/// 位置（`MemberDetailAttr::extra_check_positions`）も加え、ソートして 1e-9
/// 以内の重複を除去する（`BeamElement::new` の `eval_sections` と同じ規則。
/// この一致により応力の評価断面と検定位置が揃う）。
/// squid-n-app の `design_positions`（app.rs）と同じロジック
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
fn design_positions(
    elem: &squid_n_core::model::ElementData,
    model: &Model,
    geom_len: f64,
) -> Vec<f64> {
    let mut xs = if geom_len > 1e-12 {
        let xi_i = (elem.rigid_zone.face_i / geom_len).clamp(0.0, 0.5 - 1e-9);
        let xi_j = (1.0 - elem.rigid_zone.face_j / geom_len).clamp(0.5 + 1e-9, 1.0);
        vec![xi_i, 0.5, xi_j]
    } else {
        vec![0.0, 0.5, 1.0]
    };
    if let Some(detail) = model.member_detail(elem.id) {
        xs.extend(detail.extra_check_positions(&elem.rigid_zone, geom_len));
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    xs
}

/// `pos` が `positions` のいずれかと 1e-6 以内で一致するか判定する。
fn is_near_design_position(pos: f64, positions: &[f64]) -> bool {
    positions.iter().any(|p| (p - pos).abs() < 1e-6)
}

/// DesignCheck ジョブの純粋計算部分。
/// 指定/先頭の荷重ケースで線形静的解析を行い、断面力に対して
/// squid-n-app の `App::run_design_check`（app.rs）と同じ判定
/// （構造種別を `squid_n_core::structure_kind` で求め、[`squid_n_design_jp::checker_for`]
/// で検定器を選ぶ）を行う（squid-n-mcp は squid-n-app に依存しないため複製している）。
/// 検定条件（長期/短期）は既定で長期（`LoadTerm::Long`）とする。
pub(crate) fn compute_design_check_job(
    model: &Model,
    load_case: Option<u32>,
) -> Result<JobOutcome, String> {
    // 剛域自動算定は face_i/face_j による危険断面位置（§6.2.3）の算定にも使う。
    let model = model_with_auto_rigid_zones(model);
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let lc = resolve_load_case(model, load_case)?;
    let lc_id = lc.id.0;
    let result = analysis
        .linear_static(lc.id)
        .map_err(|e| format!("solve failed: {e}"))?;

    let mut member_force_rows: Vec<(u32, f64, [f64; 6])> = Vec::new();
    let mut n_checks = 0usize;
    let mut n_ng = 0usize;
    // 検定不能（Fc 未設定・配筋情報なし等で検定を実施できなかった部材位置）の件数。
    let mut n_skipped = 0usize;
    let mut max_ratio = 0.0_f64;

    // 柱の座屈長さ係数 K 用の節点まわり梁索引（全部材ぶんのループで使い回し、
    // `steel_column_k_axes_with_index` の全要素走査を避ける。app.rs と同じ流儀）。
    let column_k_index = squid_n_core::adjacency::NodeAdjacency::build(model);

    for (elem_id, mf) in &result.member_forces {
        for (pos, forces) in &mf.at {
            member_force_rows.push((elem_id.0, *pos, *forces));
        }

        let Some(elem) = model.elements.iter().find(|e| e.id == *elem_id) else {
            continue;
        };
        let sec = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid));
        let mat = model.element_material(elem);
        let (Some(sec), Some(mat)) = (sec, mat) else {
            continue;
        };

        // 部材種別・部材長・せん断スパン比代表値（app.rs の run_design_check と同じ規則）。
        let kind = member_kind_of(elem, model);
        let length = {
            let coords: Vec<[f64; 3]> = elem
                .nodes
                .iter()
                .filter_map(|nid| model.nodes.get(nid.index()))
                .map(|n| n.coord)
                .take(2)
                .collect();
            match (coords.first(), coords.get(1)) {
                (Some(p0), Some(p1)) => {
                    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
                    (dx * dx + dy * dy + dz * dz).sqrt()
                }
                _ => 0.0,
            }
        };
        let shear_span = mf
            .at
            .iter()
            .map(|(_, f)| (f[5].abs(), f[1].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        // 弱軸方向（|My|max と対応 |Qz|）。柱の qz 方向せん断検定の α 用。
        let shear_span_y = mf
            .at
            .iter()
            .map(|(_, f)| (f[4].abs(), f[2].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        // 端部・中央の強軸曲げ（横座屈 C 係数・たわみ検定用）。
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
        // S 造部材の断面検定属性（欠損率・横座屈長さ・座屈長さ直接入力）。単一
        // ケース・長期検定のため seismic_qd（地震時 QD 割増）は常に None。
        // lk_y/lk_z の解決（直接入力 > 柱の自動算定 > 部材長）で直接入力を
        // 先に参照するため、lk 自動算定より前に取得しておく。
        let steel_attr = model
            .steel_design_attrs
            .iter()
            .find(|a| a.elem == *elem_id)
            .cloned();
        // 柱の座屈長さ lk_y/lk_z = K_y・h／K_z・h（app.rs run_design_check と同じ
        // 規則。K は強軸・弱軸たわみ方向ごとに節点まわり剛度比 G から算定）。
        // SteelDesignAttr の直接入力があればそちらを優先する（柱以外の部材でも
        // 直接入力は有効。柱の自動算定は柱のみ）。
        let (lk_y_auto, lk_z_auto) = if kind == squid_n_design_jp::MemberKind::Column {
            match squid_n_design_jp::steel::buckling::steel_column_k_axes_with_index(
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
        let ctx = squid_n_design_jp::DesignCtx {
            // 材料は断面が持つ（主筋・せん断補強筋・内蔵鉄骨）。
            rebar_material: model.element_rebar_material(elem).cloned(),
            shear_rebar_material: model.element_shear_rebar_material(elem).cloned(),
            steel_material: model.element_steel_material(elem).cloned(),
            term: squid_n_design_jp::LoadTerm::Long,
            kind,
            length,
            lb: None,
            lk_y,
            lk_z,
            shear_span,
            shear_span_y,
            rc_damage_control: true,
            end_moments_z,
            mid_moment_z: m_at(0.5),
            seismic_qd: None,
            steel_attr,
            steel_fb_rule: Default::default(),
        };

        // 検定器の選択は構造種別による（`squid_n_core::structure_kind`。
        // app.rs の run_design_check と同じ規則）。
        let checker: Box<dyn squid_n_design_jp::DesignCheck> = squid_n_design_jp::checker_for(
            squid_n_core::structure_kind::structure_kind_of(Some(sec), Some(mat.category)),
        );
        // 危険断面位置（§6.2.3、既定は柱フェイスと中央）の内力のみ検定する。
        // 節点芯は剛域が有る場合は検定対象外（app.rs の run_design_check と同じ規則）。
        let geom_len = elem_geometric_length(elem, model);
        let positions = design_positions(elem, model, geom_len);
        for (pos, forces) in &mf.at {
            if !is_near_design_position(*pos, &positions) {
                continue;
            }
            // [N, Qy, Qz, Mx, My, Mz] -> MemberForcesAt（N は引張正の部材内力）
            let mfa = squid_n_design_jp::MemberForcesAt {
                pos: *pos,
                n: forces[0],
                qy: forces[1],
                qz: forces[2],
                my: forces[4],
                mz: forces[5],
            };
            // BRB 属性が登録された部材はメーカー許容値による BRB 検定に差し替える
            // （app.rs run_design_check と同じ規則）。BRB 検定は退化ケースを
            // 持たないため常に Checked。
            let outcome = if let Some(brb) = model.brb_attrs.iter().find(|a| a.elem == *elem_id) {
                squid_n_design_jp::CheckOutcome::Checked(squid_n_design_jp::brb::brb_check(
                    brb, mfa.n, length, true,
                ))
            } else {
                checker.check(&mfa, sec, mat, &ctx)
            };
            n_checks += 1;
            match outcome {
                squid_n_design_jp::CheckOutcome::Checked(cr) => {
                    if !cr.ok() {
                        n_ng += 1;
                    }
                    if cr.ratio() > max_ratio {
                        max_ratio = cr.ratio();
                    }
                }
                // 検定不能（Fc 未設定・配筋情報なし等）は NG 扱いにせず、
                // 別カウンタで区別する。
                squid_n_design_jp::CheckOutcome::Skipped { .. } => {
                    n_skipped += 1;
                }
            }
        }
    }

    // 節点単位の検定（RC 柱梁接合部・S パネルゾーン・冷間成形耐力比・耐震壁）。
    let mf_slices: Vec<(
        squid_n_core::ids::ElemId,
        squid_n_design_jp::joint_wiring::ForcesAt,
    )> = result
        .member_forces
        .iter()
        .map(|(id, mf)| (*id, mf.at.as_slice()))
        .collect();
    // PCa 水平接合面の検定（PcaBeamAttr が登録された梁のみ。単一ケース＝長期扱い。
    // 退化ケースを持たないため常に Checked）。
    for (_, _, cr) in
        squid_n_design_jp::rc::horizontal_joint::collect_pca_checks(model, &mf_slices, true)
    {
        n_checks += 1;
        if !cr.ok() {
            n_ng += 1;
        }
        if cr.ratio() > max_ratio {
            max_ratio = cr.ratio();
        }
    }
    // 節点単位の検定も同様に退化ケースを持たない（該当なしの節点は push
    // されない）ため、常に Checked として扱う。
    // 仕口パネルをモデル化した接合部は、解析出力のせん断モーメントを設計用
    // パネルモーメントに用いる（モデル化していない接合部は従来の手組み式）。
    let joint_checks = squid_n_design_jp::joint_wiring::collect_joint_checks_with_long(
        model,
        &mf_slices,
        None,
        &result.panel_moments,
        squid_n_design_jp::LoadTerm::Long,
    );
    let n_joint_checks = joint_checks.len();
    let n_joint_ng = joint_checks.iter().filter(|(_, _, cr)| !cr.ok()).count();
    for (_, _, cr) in &joint_checks {
        if cr.ratio() > max_ratio {
            max_ratio = cr.ratio();
        }
    }

    let summary = serde_json::json!({
        "kind": "DesignCheck",
        "case": lc_id,
        "n_checks": n_checks,
        "n_ng": n_ng,
        // 検定不能（入力不足等で検定を実施できなかった部材位置）の件数。
        // NG には含めない。
        "n_skipped": n_skipped,
        "n_joint_checks": n_joint_checks,
        "n_joint_ng": n_joint_ng,
        "max_ratio": max_ratio,
    });
    Ok(JobOutcome::DesignCheck {
        case: lc_id,
        member_force_rows,
        summary,
    })
}
