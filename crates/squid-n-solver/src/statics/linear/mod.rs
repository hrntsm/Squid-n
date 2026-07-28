use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{ElementData, ElementKind, LoadCaseKind, Model};
use squid_n_element::beam::MemberForces;
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};
use std::borrow::Cow;

/// 長期軸力無効化（一貫構造計算プログラムの実務慣行）で断面積に乗じる縮小係数。
/// 完全にゼロにすると（ブレースのみで支持される節点等で）浮き自由度による
/// 特異行列を招く恐れがあるため、実務上無視できる微小軸剛性を残す
/// （EA×1e-6 は元の軸力の 1e-6 倍程度に留まり回収内力もほぼ0とみなせる）。
const AXIAL_DISABLE_FACTOR: f64 = 1.0e-6;

/// 部材が「柱」（鉛直な `ElementKind::Beam`）かどうかを判定する。
/// 判定規則は `squid-n-design-jp::eccentricity::column_stiffnesses` の柱判定
/// （部材軸単位ベクトルの ez 成分 |ez|>0.707）に合わせる。
fn is_vertical_column(elem: &ElementData, model: &Model) -> bool {
    if !matches!(elem.kind, ElementKind::Beam) || elem.nodes.len() < 2 {
        return false;
    }
    let (Some(n0), Some(n1)) = (
        model.nodes.get(elem.nodes[0].index()),
        model.nodes.get(elem.nodes[1].index()),
    ) else {
        return false;
    };
    let dx = n1.coord[0] - n0.coord[0];
    let dy = n1.coord[1] - n0.coord[1];
    let dz = n1.coord[2] - n0.coord[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-9 {
        return false;
    }
    (dz / len).abs() > 0.707
}

/// 長期応力解析で軸力を負担させない部材（対象: ブレース／柱）かどうかを、
/// `Model::stress_cfg` の指定に基づいて判定する。
fn is_axial_disabled_target(
    elem: &ElementData,
    model: &Model,
    cfg: &squid_n_core::model::StressAnalysisCfg,
) -> bool {
    match elem.kind {
        ElementKind::Brace { .. } => cfg.no_long_axial_brace,
        ElementKind::Beam => cfg.no_long_axial_column && is_vertical_column(elem, model),
        _ => false,
    }
}

/// 長期応力解析の計算条件（一貫構造計算プログラムの実務慣行）を適用したモデルを返す。
///
/// 対象荷重ケースが長期系（`LoadCaseKind::is_long_term`）かつ `stress_cfg` で
/// 軸力無効化が指定されている部材がある場合のみ、対象部材が参照する断面を
/// 複製して断面積を `AXIAL_DISABLE_FACTOR` 倍に縮小したモデルを作る
/// （同じ断面 ID を共有する他部材へは影響しない）。曲げ・せん断・ねじり
/// 関連の断面性能は変更しない。対象が無ければ元のモデルをそのまま返す
/// （既定 `stress_cfg` では常にこちら＝従来どおりの結果に一致する）。
///
/// SRC/CFT 等の合成断面では `beam.rs` の軸剛性用面積 `a_stiff` が `shape` 由来の
/// 値で再計算されるため、複製断面では `shape` を外して数値直入力断面へ落とす。
/// これにより曲げ・せん断は `to_section()` が格納済みの等価換算値のまま、
/// 軸剛性のみ `area × AXIAL_DISABLE_FACTOR` が効く（材料由来の複合換算・
/// スラブ協力幅係数は複製断面では適用されなくなるが、軸力を負担させない
/// 部材の曲げ剛性の微差であり実用上支障ない）。
fn apply_long_axial_cut(model: &Model, lc_kind: LoadCaseKind) -> Cow<'_, Model> {
    let cfg = &model.stress_cfg;
    if !lc_kind.is_long_term() || (!cfg.no_long_axial_brace && !cfg.no_long_axial_column) {
        return Cow::Borrowed(model);
    }

    let targets: Vec<usize> = model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| is_axial_disabled_target(e, model, cfg) && e.section.is_some())
        .map(|(i, _)| i)
        .collect();
    if targets.is_empty() {
        return Cow::Borrowed(model);
    }

    let mut m = model.clone();
    for i in targets {
        let Some(sid) = m.elements[i].section else {
            continue;
        };
        let Some(orig) = m.sections.get(sid.index()) else {
            continue;
        };
        let mut reduced = orig.clone();
        reduced.area *= AXIAL_DISABLE_FACTOR;
        // 合成断面（SRC/CFT）でも軸剛性カットが効くよう shape を外す（関数 doc 参照）。
        reduced.shape = None;
        reduced.id = squid_n_core::ids::SectionId(m.sections.len() as u32);
        m.elements[i].section = Some(reduced.id);
        m.sections.push(reduced);
    }
    Cow::Owned(m)
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct StaticOnce {
    pub disp: Vec<[f64; 6]>,
    pub member_forces: Vec<(squid_n_core::ids::ElemId, MemberForces)>,
}

pub fn linear_static_once(model: &Model, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
    squid_n_math::parallelism::apply_to_faer();
    let lc_kind = model
        .load_cases
        .iter()
        .find(|l| l.id == lc)
        .map(|l| l.kind)
        .unwrap_or_default();
    let model_cow = apply_long_axial_cut(model, lc_kind);
    let model: &Model = &model_cow;

    // 引張専用ブレースの反復（active-set 法）: 計算条件で有効化されており、かつ
    // 引張専用ブレースが存在する場合のみ、圧縮側に入ったブレースを無効化しながら
    // 収束するまで再解析する。無効時は従来どおり弾性剛性 1/2 の一括解析
    // （build_behavior の factor=0.5）で1回だけ解く。
    if model.stress_cfg.tension_only_iteration && has_tension_only_brace(model) {
        return solve_tension_only_iterative(model, lc);
    }
    solve_once_inner(model, lc)
}

/// 引張専用ブレースの active-set 反復の最大回数。通常はブレース本数程度で収束するが、
/// 無効化・再活性が振動（チャタリング）する病的ケースに備えて上限を設ける。
const TENSION_ONLY_MAX_ITER: usize = 50;

/// モデルに引張専用ブレース（`ElementKind::Brace { tension_only: true }`）が
/// 少なくとも1本存在するか。
fn has_tension_only_brace(model: &Model) -> bool {
    model
        .elements
        .iter()
        .any(|e| matches!(e.kind, ElementKind::Brace { tension_only: true }))
}

/// 指定した要素 index のブレースについて、参照断面を複製し軸剛性用の断面積を
/// [`AXIAL_DISABLE_FACTOR`] 倍に縮小したモデルを返す（apply_long_axial_cut と同じ
/// 手法。無効化対象が空なら元のモデルをそのまま借用する）。同じ断面 ID を共有する
/// active なブレースへは影響しない。
fn reduce_brace_axial<'a>(model: &'a Model, disabled: &[usize]) -> Cow<'a, Model> {
    if disabled.is_empty() {
        return Cow::Borrowed(model);
    }
    let mut m = model.clone();
    for &i in disabled {
        let Some(sid) = m.elements[i].section else {
            continue;
        };
        let Some(orig) = m.sections.get(sid.index()) else {
            continue;
        };
        let mut reduced = orig.clone();
        reduced.area *= AXIAL_DISABLE_FACTOR;
        // 合成断面（SRC/CFT）でも軸剛性カットが効くよう shape を外す（apply_long_axial_cut 参照）。
        reduced.shape = None;
        reduced.id = squid_n_core::ids::SectionId(m.sections.len() as u32);
        m.elements[i].section = Some(reduced.id);
        m.sections.push(reduced);
    }
    Cow::Owned(m)
}

/// active-set 反復で追跡する引張専用ブレース1本の情報。
struct ToBrace {
    /// `model.elements` 内の要素 index。
    elem: usize,
    /// i 端・j 端の節点 index。
    ni: usize,
    nj: usize,
    /// 部材軸単位ベクトル（i→j）。軸伸び δ = t·(u_j − u_i) の判定に用いる。
    t: [f64; 3],
}

/// 引張専用ブレースを active-set 法で反復解析する（真の引張専用解析）。
///
/// ブレース（軸剛性 E·A/L）を各反復で解き、圧縮側（軸伸び<0）に入った引張専用
/// ブレースの軸剛性を縮小して無効化する。無効化されたブレースの節点変位から
/// 求めた軸伸びが引張側へ転じれば再び active に戻す。active 集合が前回と一致した
/// 時点で収束とみなす。
///
/// 収束後の部材内力は、active な引張ブレースが EA/L·伸び を負担し、無効化された
/// 圧縮ブレースはほぼ 0（EA×1e-6 相当）となる。
fn solve_tension_only_iterative(model: &Model, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
    // 追跡対象の引張専用ブレースを収集する。幾何が退化した（節点不足・零長）ブレースは
    // 軸剛性が実質ゼロで軸力を負担しないため除外する。
    let mut braces: Vec<ToBrace> = Vec::new();
    for (i, e) in model.elements.iter().enumerate() {
        if !matches!(e.kind, ElementKind::Brace { tension_only: true }) || e.nodes.len() < 2 {
            continue;
        }
        let (ni, nj) = (e.nodes[0].index(), e.nodes[1].index());
        let (Some(n0), Some(n1)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
            continue;
        };
        let d = [
            n1.coord[0] - n0.coord[0],
            n1.coord[1] - n0.coord[1],
            n1.coord[2] - n0.coord[2],
        ];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if l < 1e-12 {
            continue;
        }
        braces.push(ToBrace {
            elem: i,
            ni,
            nj,
            t: [d[0] / l, d[1] / l, d[2] / l],
        });
    }

    // active[k] = k 番目の引張専用ブレースが軸力を負担する（引張側）か。初期は全 active。
    let mut active = vec![true; braces.len()];
    let mut last: Option<StaticOnce> = None;
    for _ in 0..TENSION_ONLY_MAX_ITER {
        let disabled: Vec<usize> = braces
            .iter()
            .zip(&active)
            .filter(|(_, &a)| !a)
            .map(|(b, _)| b.elem)
            .collect();
        let solve_model = reduce_brace_axial(model, &disabled);
        let res = solve_once_inner(&solve_model, lc)?;

        // 各ブレースの軸伸び δ = t·(u_j − u_i) から次の active 集合を判定する。
        // δ≥0（引張）なら active、δ<0（圧縮・スラック）なら無効化。
        let new_active: Vec<bool> = braces
            .iter()
            .map(|b| {
                let du = [
                    res.disp[b.nj][0] - res.disp[b.ni][0],
                    res.disp[b.nj][1] - res.disp[b.ni][1],
                    res.disp[b.nj][2] - res.disp[b.ni][2],
                ];
                b.t[0] * du[0] + b.t[1] * du[1] + b.t[2] * du[2] >= 0.0
            })
            .collect();

        if new_active == active {
            return Ok(res);
        }
        active = new_active;
        last = Some(res);
    }
    // 収束しなかった（active 集合が振動した）場合は最後の結果を返す。
    match last {
        Some(res) => Ok(res),
        None => solve_once_inner(model, lc),
    }
}

fn solve_once_inner(model: &Model, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
    let dofmap = DofMap::build(model);
    let n_active = dofmap.n_active();

    if n_active == 0 {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        return Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        });
    }

    let k_free = assemble_global_k(model, &dofmap);
    let f_free = assemble_global_f(model, &dofmap, lc);

    let reducer = Reducer::build(model, &dofmap);
    let k_red = reducer.reduce_k(&k_free);
    let f_red = reducer.reduce_f(&f_free);
    let n_indep = reducer.n_indep;

    let mut solver = make_solver(SolverBackend::Auto);
    if n_indep > 0 {
        solver.factorize(&k_red)?;
        let u_indep = solver.solve(&f_red)?;
        let u_free = reducer.expand_u(&u_indep);

        let mut disp: Vec<[f64; 6]> = vec![[0.0; 6]; model.nodes.len()];
        for ni in 0..model.nodes.len() {
            for d in 0..squid_n_core::dof::DOF_PER_NODE {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    let val = u_free[active as usize];
                    match d {
                        0 => disp[ni][0] = val,
                        1 => disp[ni][1] = val,
                        2 => disp[ni][2] = val,
                        3 => disp[ni][3] = val,
                        4 => disp[ni][4] = val,
                        _ => disp[ni][5] = val,
                    }
                }
            }
        }

        let mut member_forces = Vec::new();
        let _ctx = squid_n_element::behavior::Ctx { model };
        // 解析対象荷重ケースの部材荷重（内力回復の重ね合わせ用）
        let member_loads: &[squid_n_core::model::MemberLoad] = model
            .load_cases
            .iter()
            .find(|l| l.id == lc)
            .map(|l| l.member.as_slice())
            .unwrap_or(&[]);
        for elem in &model.elements {
            let (behavior, _state) = build_behavior(elem, model);
            let gdofs = behavior.global_dofs(&dofmap);
            let n_gdofs = gdofs.len();
            let mut u_elem = vec![0.0; n_gdofs];

            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }

            if let Some(mut forces) = behavior.recover_forces(&u_elem) {
                superpose_member_loads(model, elem, member_loads, &mut forces);
                member_forces.push((elem.id, forces));
            }
        }
        ensure_line_member_forces(model, &member_forces)?;

        Ok(StaticOnce {
            disp,
            member_forces,
        })
    } else {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
        })
    }
}

/// 内力回収の欠落検出（線材）。
///
/// 線材（梁・柱＝`Beam`、`Fiber`、`MultiSpring`、ブレース）は必ず
/// `ElementBehavior::recover_forces` を実装している必要がある。実装されていない
/// 要素は `recover_forces` が `None` を返し、線形静解析のループで**黙って
/// 読み飛ばされて** `member_forces` から丸ごと欠落する。欠落した部材は応力図・
/// 断面検定・柱梁接合部検定・設計用せん断力 QD のいずれにも現れず、
/// 「結果が空である」ことがユーザーからは正常な計算結果と区別できない。
///
/// 実際に、剛床に載る梁が材端集中ばね梁（`recover_forces` 未実装）で組まれて
/// 全階の梁が無言で欠落する不具合があったため、要素実装の不備を解析エラーとして
/// 顕在化させる（`dev_docs/handoff/剛床上の梁の応力欠落_申し送り.md`）。
pub(crate) fn ensure_line_member_forces(
    model: &Model,
    member_forces: &[(squid_n_core::ids::ElemId, MemberForces)],
) -> Result<(), SolveError> {
    use std::collections::HashSet;

    let recovered: HashSet<squid_n_core::ids::ElemId> =
        member_forces.iter().map(|(id, _)| *id).collect();
    let missing: Vec<u32> = model
        .elements
        .iter()
        .filter(|e| {
            matches!(
                e.kind,
                ElementKind::Beam
                    | ElementKind::Fiber
                    | ElementKind::MultiSpring
                    | ElementKind::Brace { .. }
            )
        })
        .filter(|e| !recovered.contains(&e.id))
        .map(|e| e.id.0)
        .collect();
    if missing.is_empty() {
        return Ok(());
    }
    let head: Vec<String> = missing.iter().take(5).map(|id| id.to_string()).collect();
    let more = if missing.len() > 5 {
        format!(" 他{}件", missing.len() - 5)
    } else {
        String::new()
    };
    Err(SolveError::InvalidInput(format!(
        "線材の部材内力を回収できませんでした: 部材 ID {}{}。\
         要素実装の不具合です（このまま続けると応力図・断面検定から当該部材が\
         無言で欠落します）。",
        head.join(", "),
        more
    )))
}

/// 部材荷重の固定端内力を、`K·u` 由来の回復内力へ各断面で重ね合わせる。
/// 線形重ね合わせ: 実内力 = （等価節点力に対する応答 K·u）＋（両端固定梁のスパン内力）。
///
/// `Analysis` ファサード（分解済み K を再利用する経路）でも同じ重ね合わせが
/// 要るため `pub(crate)` で共有する（[`crate::statics::analysis`] 参照）。
pub(crate) fn superpose_member_loads(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    member_loads: &[squid_n_core::model::MemberLoad],
    forces: &mut squid_n_element::beam::MemberForces,
) {
    let loads: Vec<squid_n_core::model::MemberLoad> = member_loads
        .iter()
        .filter(|ml| ml.elem == elem.id)
        .cloned()
        .collect();
    if loads.is_empty() {
        return;
    }
    // 対象の線材判定・局所座標系・部材長は等価節点力側（`assemble_global_f`）と
    // 同じ規則を共有する（[`crate::assemble::member_load_frame`]）。荷重ベクトル側で
    // 載らなかった荷重が内力回復側だけに重なる（またはその逆）不整合を防ぐ。
    let Some((frame, length)) = crate::assemble::member_load_frame(model, elem) else {
        return;
    };
    for (xi, vals) in forces.at.iter_mut() {
        let fixed = squid_n_element::member_load::fixed_internal_local(&loads, &frame, length, *xi);
        for k in 0..6 {
            vals[k] += fixed[k];
        }
    }
}

#[cfg(test)]
mod tests;
