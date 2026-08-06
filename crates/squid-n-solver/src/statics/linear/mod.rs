use crate::assemble::{add_support_spring_diag, assemble_global_f, support_spring_terms};
use crate::common::csc_cache::CscCache;
use crate::constraint::Reducer;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::{ElemId, LoadCaseId};
use squid_n_core::model::{ElementData, ElementKind, LoadCaseKind, MemberLoad, Model};
use squid_n_element::beam::MemberForces;
use squid_n_element::behavior::{Ctx, ElementBehavior};
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};
use squid_n_math::sparse::{assemble_csc, Triplet};
use std::borrow::Cow;
use std::collections::HashMap;

/// 長期軸力無効化（一貫構造計算プログラムの実務慣行）で断面積に乗じる縮小係数。
/// 完全にゼロにすると（ブレースのみで支持される節点等で）浮き自由度による
/// 特異行列を招く恐れがあるため、実務上無視できる微小軸剛性を残す
/// （EA×1e-6 は元の軸力の 1e-6 倍程度に留まり回収内力もほぼ0とみなせる）。
const AXIAL_DISABLE_FACTOR: f64 = 1.0e-6;

/// 部材が「柱」（鉛直な `ElementKind::Beam`）かどうかを判定する。
/// 判定規則は全クレート共通の 45° 余弦基準
/// （[`squid_n_core::geom::is_vertical_axis`]: |ez| > 0.707）。
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
    squid_n_core::geom::is_vertical_axis(n0.coord, n1.coord)
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
/// 関連の断面性能は変更しない。対象がなければ元のモデルをそのまま返す
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
    /// 仕口パネルのせん断モーメント `{MSX, MSY}` [N·mm]（基準座標系）を接合部の
    /// 節点ごとに保持する。パネルをモデル化していない場合は空。
    ///
    /// 節点変位（`disp`）は標準 6 成分のみのため、パネルのせん断変形角は結果に
    /// 現れない。断面検定の設計用パネルモーメント `pM` へ供給できるよう、
    /// 解析側でモーメントへ換算して持たせる。
    #[serde(default)]
    pub panel_moments: Vec<(squid_n_core::ids::NodeId, [f64; 2])>,
}

/// 静的解析結果（節点変位・部材断面力）を係数倍して足し合わせた結果を返す。
///
/// 線形解析では重ね合わせの原理が成り立つため、荷重組合せ `Σ cᵢ·Lᵢ` の応答は、
/// 荷重ケース単体 `Lᵢ` の応答を `cᵢ` 倍して足し合わせた値と一致する。解析の最小単位を
/// 荷重ケース単体に統一し、荷重組合せはその結果の線形和として組み立てるための共通処理
/// （[`crate::analysis::Analysis::linear_combination`] が使う）。
///
/// 各項は同一モデル・同一 [`crate::analysis::Analysis`] から得た結果である前提で、
/// 節点数・部材の出現順・部材内の評価断面位置が一致しているものとして足し合わせる
/// （評価断面の位置 `xi` は先頭の項の値を採る）。項が空の場合は変位・断面力とも
/// 空の結果を返す。
pub fn superpose_static(terms: &[(&StaticOnce, f64)]) -> StaticOnce {
    let Some((first, _)) = terms.first() else {
        return StaticOnce {
            disp: Vec::new(),
            member_forces: Vec::new(),
            panel_moments: Vec::new(),
        };
    };
    let mut disp = vec![[0.0; 6]; first.disp.len()];
    let mut member_forces: Vec<(ElemId, MemberForces)> = first
        .member_forces
        .iter()
        .map(|(id, mf)| {
            (
                *id,
                MemberForces {
                    at: mf.at.iter().map(|(xi, _)| (*xi, [0.0; 6])).collect(),
                },
            )
        })
        .collect();
    // 仕口パネルは線形弾性（`Kxp = Kyp = G・Ve`）なのでモーメントも線形和で足せる。
    let mut panel_moments: Vec<(squid_n_core::ids::NodeId, [f64; 2])> = first
        .panel_moments
        .iter()
        .map(|(node, _)| (*node, [0.0; 2]))
        .collect();
    for (res, factor) in terms {
        for (dst, src) in disp.iter_mut().zip(res.disp.iter()) {
            for (d, s) in dst.iter_mut().zip(src.iter()) {
                *d += s * factor;
            }
        }
        for ((_, dst), (_, src)) in member_forces.iter_mut().zip(res.member_forces.iter()) {
            for ((_, d6), (_, s6)) in dst.at.iter_mut().zip(src.at.iter()) {
                for (d, s) in d6.iter_mut().zip(s6.iter()) {
                    *d += s * factor;
                }
            }
        }
        for ((_, dst), (_, src)) in panel_moments.iter_mut().zip(res.panel_moments.iter()) {
            for (d, s) in dst.iter_mut().zip(src.iter()) {
                *d += s * factor;
            }
        }
    }
    StaticOnce {
        disp,
        member_forces,
        panel_moments,
    }
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

    // 要素接続・拘束・自由度構成は反復間で不変（変わるのは無効化ブレースの
    // 断面積のみ）なので、DofMap・拘束縮約は1回だけ構築して使い回す。
    let dofmap = DofMap::build(model);
    let n_active = dofmap.n_active();
    if n_active == 0 {
        // 有効自由度なし: 変位は常に0であり、どの反復でも
        // 全ブレース active（初期値）のまま収束する（`solve_once_inner` の
        // 自由度なし分岐と同じ結果）。
        return Ok(StaticOnce {
            disp: vec![[0.0; 6]; model.nodes.len()],
            member_forces: Vec::new(),
            panel_moments: Vec::new(),
        });
    }
    let reducer = Reducer::build(model, &dofmap);
    let n_indep = reducer.n_indep;
    if n_indep == 0 {
        // 独立自由度なし（全自由度が拘束に吸収される特殊な拘束構成）:
        // `solve_once_inner` の対応分岐と同じく、内力回収（`ensure_line_member_forces`
        // の検証含む）を行わずゼロ結果を返す。
        return Ok(StaticOnce {
            disp: vec![[0.0; 6]; model.nodes.len()],
            member_forces: Vec::new(),
            panel_moments: Vec::new(),
        });
    }

    // 要素ごとの global_dofs・局所剛性行列（有効時／無効時）・回収用 behavior を
    // 1回だけ構築する（反復ごとの build_behavior 再実行・model.clone() を排除）。
    let assembly = BraceIterAssembly::build(model, &dofmap, &braces);

    // 荷重ベクトルはブレースの有効/無効に依存しないため1回だけ組み立てる。
    let f_free = assemble_global_f(model, &dofmap, lc);
    let f_red = reducer.reduce_f(&f_free);
    let member_loads: &[MemberLoad] = model
        .load_cases
        .iter()
        .find(|l| l.id == lc)
        .map(|l| l.member.as_slice())
        .unwrap_or(&[]);
    let member_loads_by_elem = group_member_loads_by_elem(member_loads);

    let mut k_free_cache = CscCache::new();
    let mut k_red_cache = CscCache::new();
    let mut solver = make_solver(SolverBackend::Auto);

    // active[k] = k 番目の引張専用ブレースが軸力を負担する（引張側）か。初期は全 active。
    let mut active = vec![true; braces.len()];
    // 収束しなかった場合に返す最後の結果（当該反復で使った active 集合と解）。
    let mut fallback: Option<(Vec<bool>, Vec<f64>)> = None;

    for _ in 0..TENSION_ONLY_MAX_ITER {
        let active_used = active.clone();
        let triplets = assembly.triplets(&active_used);
        let k_free = k_free_cache.assemble(n_active, &triplets);
        let k_red = reducer.reduce_k_cached(&k_free, &mut k_red_cache);

        solver.factorize(&k_red)?;
        let u_indep = solver.solve(&f_red)?;
        let u_free = reducer.expand_u(&u_indep);

        // 各ブレースの軸伸び δ = t·(u_j − u_i) から次の active 集合を判定する。
        // δ≥0（引張）なら active、δ<0（圧縮・スラック）なら無効化。
        let new_active: Vec<bool> = braces
            .iter()
            .map(|b| {
                let du = |ni: usize, d: usize| -> f64 {
                    let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                    dofmap.active(g).map(|a| u_free[a as usize]).unwrap_or(0.0)
                };
                let dux = du(b.nj, 0) - du(b.ni, 0);
                let duy = du(b.nj, 1) - du(b.ni, 1);
                let duz = du(b.nj, 2) - du(b.ni, 2);
                b.t[0] * dux + b.t[1] * duy + b.t[2] * duz >= 0.0
            })
            .collect();

        if new_active == active_used {
            return build_tension_only_result(
                model,
                &dofmap,
                &assembly,
                &active_used,
                &u_free,
                &member_loads_by_elem,
            );
        }
        active = new_active;
        fallback = Some((active_used, u_free));
    }
    // 収束しなかった（active 集合が振動した）場合は最後の結果を返す。
    match fallback {
        Some((active_used, u_free)) => build_tension_only_result(
            model,
            &dofmap,
            &assembly,
            &active_used,
            &u_free,
            &member_loads_by_elem,
        ),
        None => solve_once_inner(model, lc),
    }
}

/// active-set 反復1回分の要素アセンブリ情報。反復間で不変な
/// （global_dofs・局所剛性行列・内力回収用 behavior）を1回だけ計算して保持し、
/// 各反復では「引張専用ブレースの active/disabled のどちらの局所剛性を使うか」を
/// 選択して triplet 化するだけにする。
///
/// 引張専用ブレースの無効化断面（[`reduce_brace_axial`] と同じ、軸剛性用面積を
/// [`AXIAL_DISABLE_FACTOR`] 倍した断面）は、全ブレースぶんまとめて1回だけモデルを
/// 複製して求める（従来は反復ごとに `model.clone()` していた）。個々の要素に対する
/// `build_behavior`/`tangent_stiffness` の呼び出しは、有効時・無効時とも従来の
/// 反復内呼び出しと完全に同じ入力（元モデル or 断面差し替え後のモデル）で行うため、
/// 結果はビット一致する。
struct BraceIterAssembly {
    /// 要素ごとの global_dofs（active-set に依らず不変）。
    gdofs: Vec<smallvec::SmallVec<[usize; 24]>>,
    /// 要素ごとの「有効時」局所剛性行列（引張専用ブレースも含め元の断面のまま）。
    k_active: Vec<squid_n_element::behavior::LocalMat>,
    /// 引張専用ブレース要素のみ Some：無効化時（軸剛性 ×[`AXIAL_DISABLE_FACTOR`]）の
    /// 局所剛性行列。それ以外の要素は常に None（無効化され得ない）。
    k_disabled: Vec<Option<squid_n_element::behavior::LocalMat>>,
    /// 内力回収用の behavior（有効時）。
    behavior_active: Vec<Box<dyn ElementBehavior>>,
    /// 引張専用ブレース要素のみ Some：内力回収用の behavior（無効化時）。
    behavior_disabled: Vec<Option<Box<dyn ElementBehavior>>>,
    /// 要素 index → `braces` 配列上のインデックス（引張専用ブレースのみ）。
    brace_of_elem: HashMap<usize, usize>,
    /// 支点ばねの対角項（ブレースの有効/無効に依存せず不変）。
    spring_terms: Vec<(usize, f64)>,
}

impl BraceIterAssembly {
    /// `dofmap` は元モデル（`model`）から構築したもの。
    fn build(model: &Model, dofmap: &DofMap, braces: &[ToBrace]) -> Self {
        let brace_of_elem: HashMap<usize, usize> = braces
            .iter()
            .enumerate()
            .map(|(k, b)| (b.elem, k))
            .collect();
        // 全引張専用ブレースを無効化した断面を持つモデルを1回だけ複製する
        // （`reduce_brace_axial` と同じ手法。反復ごとの複製を排除）。
        let brace_elems: Vec<usize> = braces.iter().map(|b| b.elem).collect();
        let disabled_model = reduce_brace_axial(model, &brace_elems);

        let n_elem = model.elements.len();
        let mut gdofs = Vec::with_capacity(n_elem);
        let mut k_active = Vec::with_capacity(n_elem);
        let mut k_disabled = Vec::with_capacity(n_elem);
        let mut behavior_active = Vec::with_capacity(n_elem);
        let mut behavior_disabled = Vec::with_capacity(n_elem);

        for (i, elem) in model.elements.iter().enumerate() {
            let b_active = build_behavior(elem, model);
            let g = b_active.global_dofs(dofmap);
            let k = b_active.tangent_stiffness(&Ctx { model });

            if brace_of_elem.contains_key(&i) {
                let delem = &disabled_model.elements[i];
                let b_disabled = build_behavior(delem, &disabled_model);
                let kd = b_disabled.tangent_stiffness(&Ctx {
                    model: &disabled_model,
                });
                k_disabled.push(Some(kd));
                behavior_disabled.push(Some(b_disabled));
            } else {
                k_disabled.push(None);
                behavior_disabled.push(None);
            }
            gdofs.push(g);
            k_active.push(k);
            behavior_active.push(b_active);
        }

        let spring_terms = support_spring_terms(model, dofmap);

        Self {
            gdofs,
            k_active,
            k_disabled,
            behavior_active,
            behavior_disabled,
            brace_of_elem,
            spring_terms,
        }
    }

    /// 要素 index の、現在の `active` 集合における局所剛性行列を返す。
    fn k_local_for(&self, i: usize, active: &[bool]) -> &squid_n_element::behavior::LocalMat {
        match self.brace_of_elem.get(&i) {
            Some(&bidx) if !active[bidx] => self.k_disabled[i]
                .as_ref()
                .expect("引張専用ブレース要素は k_disabled を必ず持つ"),
            _ => &self.k_active[i],
        }
    }

    /// 要素 index の、現在の `active` 集合における内力回収用 behavior を返す。
    fn behavior_for(&self, i: usize, active: &[bool]) -> &dyn ElementBehavior {
        match self.brace_of_elem.get(&i) {
            Some(&bidx) if !active[bidx] => self.behavior_disabled[i]
                .as_deref()
                .expect("引張専用ブレース要素は behavior_disabled を必ず持つ"),
            _ => self.behavior_active[i].as_ref(),
        }
    }

    /// 現在の `active` 集合に基づき全体剛性 K（縮約前）の triplet 列を組み立てる。
    fn triplets(&self, active: &[bool]) -> Vec<Triplet> {
        let mut triplets = Vec::new();
        for i in 0..self.gdofs.len() {
            triplets.extend(self.k_local_for(i, active).to_triplets(&self.gdofs[i]));
        }
        for &(a, k) in &self.spring_terms {
            triplets.push(Triplet {
                row: a,
                col: a,
                val: k,
            });
        }
        triplets
    }
}

/// active-set 反復の収束（または反復上限）後、変位・部材内力を復元する。
/// `solve_once_inner` の内力回収と同じ規則（要素順・重ね合わせ順）を用いる。
fn build_tension_only_result(
    model: &Model,
    dofmap: &DofMap,
    assembly: &BraceIterAssembly,
    active: &[bool],
    u_free: &[f64],
    member_loads_by_elem: &HashMap<ElemId, Vec<MemberLoad>>,
) -> Result<StaticOnce, SolveError> {
    let disp = dofmap.expand_to_nodes(u_free, model.nodes.len());

    let mut member_forces = Vec::new();
    let mut panel_moments = Vec::new();
    for (i, elem) in model.elements.iter().enumerate() {
        let behavior = assembly.behavior_for(i, active);
        let gdofs = &assembly.gdofs[i];
        let u_elem = crate::common::elem_loop::gather_u_elem(gdofs, u_free);
        if let Some(mut forces) = behavior.recover_forces(&u_elem) {
            let loads = member_loads_by_elem
                .get(&elem.id)
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            superpose_member_loads(model, elem, loads, &mut forces);
            member_forces.push((elem.id, forces));
        }
        // 仕口パネルのせん断モーメント（断面検定の設計用パネルモーメント pM）。
        if let (Some(&node), Some(m)) = (elem.nodes.first(), behavior.panel_moments_from(&u_elem)) {
            panel_moments.push((node, m));
        }
    }
    ensure_line_member_forces(model, &member_forces)?;

    Ok(StaticOnce {
        disp,
        member_forces,
        panel_moments,
    })
}

fn solve_once_inner(model: &Model, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
    let dofmap = DofMap::build(model);
    let n_active = dofmap.n_active();

    if n_active == 0 {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        return Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
            panel_moments: Vec::new(),
        });
    }

    // 要素ごとの behavior・global_dofs・局所剛性を1回だけ構築し、K 組立（本関数内）と
    // 内力回収（下の回収ループ）の両方で使い回す（従来は `assemble_global_k` 内部と
    // 回収ループの計2回 `build_behavior` していた）。構築順・演算順は
    // 従来の `assemble_global_k` と完全に同じ（要素 ID 順→支点ばね対角）で、
    // 結果はビット一致する。
    let ctx = Ctx { model };
    let mut behaviors: Vec<crate::statics::BehaviorEntry> =
        Vec::with_capacity(model.elements.len());
    let mut k_triplets = Vec::new();
    for elem in &model.elements {
        let behavior = build_behavior(elem, model);
        let gdofs = behavior.global_dofs(&dofmap);
        let k_local = behavior.tangent_stiffness(&ctx);
        k_triplets.extend(k_local.to_triplets(&gdofs));
        behaviors.push((behavior, gdofs));
    }
    add_support_spring_diag(model, &dofmap, &mut k_triplets);
    let k_free = assemble_csc(n_active, k_triplets);
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

        let disp = dofmap.expand_to_nodes(&u_free, model.nodes.len());

        let mut member_forces = Vec::new();
        // 解析対象荷重ケースの部材荷重（内力回復の重ね合わせ用）。要素 ID で
        // 事前にグルーピングし、要素ごとの全部材荷重総当りスキャンを避ける。
        let member_loads: &[squid_n_core::model::MemberLoad] = model
            .load_cases
            .iter()
            .find(|l| l.id == lc)
            .map(|l| l.member.as_slice())
            .unwrap_or(&[]);
        let member_loads_by_elem = group_member_loads_by_elem(member_loads);
        let mut panel_moments = Vec::new();
        for (elem, (behavior, gdofs)) in model.elements.iter().zip(behaviors.iter()) {
            let u_elem = crate::common::elem_loop::gather_u_elem(gdofs, &u_free);
            if let Some(mut forces) = behavior.recover_forces(&u_elem) {
                let loads = member_loads_by_elem
                    .get(&elem.id)
                    .map(Vec::as_slice)
                    .unwrap_or(&[]);
                superpose_member_loads(model, elem, loads, &mut forces);
                member_forces.push((elem.id, forces));
            }
            // 仕口パネルのせん断モーメント（断面検定の設計用パネルモーメント pM）。
            if let (Some(&node), Some(m)) =
                (elem.nodes.first(), behavior.panel_moments_from(&u_elem))
            {
                panel_moments.push((node, m));
            }
        }
        ensure_line_member_forces(model, &member_forces)?;

        Ok(StaticOnce {
            disp,
            member_forces,
            panel_moments,
        })
    } else {
        let disp = vec![[0.0; 6]; model.nodes.len()];
        Ok(StaticOnce {
            disp,
            member_forces: Vec::new(),
            panel_moments: Vec::new(),
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

/// 部材荷重を要素 ID でグルーピングする（内力回収の重ね合わせ用）。
///
/// 従来は要素ごとに `member_loads.iter().filter(...)` で全部材荷重を毎回
/// 総当りスキャンしていた（O(要素数×荷重数)）。呼び出し側で1回だけ本関数を
/// 呼んでグルーピングし、[`superpose_member_loads`] へ要素分の荷重だけを渡す。
/// 各要素内の荷重順序は `member_loads` 内の元の出現順のまま保たれる
/// （固定端内力の重ね合わせは加算順序に依存しないため数値上は問題ないが、
/// 念のため元の順序を崩さない）。
pub(crate) fn group_member_loads_by_elem(
    member_loads: &[MemberLoad],
) -> HashMap<ElemId, Vec<MemberLoad>> {
    let mut by_elem: HashMap<ElemId, Vec<MemberLoad>> = HashMap::new();
    for ml in member_loads {
        by_elem.entry(ml.elem).or_default().push(ml.clone());
    }
    by_elem
}

/// 部材荷重の固定端内力を、`K·u` 由来の回復内力へ各断面で重ね合わせる。
/// 線形重ね合わせ: 実内力 = （等価節点力に対する応答 K·u）＋（両端固定梁のスパン内力）。
///
/// `loads` は当該要素に作用する部材荷重のみ（[`group_member_loads_by_elem`] で
/// 事前にグルーピングした結果から呼び出し側が取り出して渡す）。
///
/// `Analysis` ファサード（分解済み K を再利用する経路）でも同じ重ね合わせが
/// 要るため `pub(crate)` で共有する（[`crate::statics::analysis`] 参照）。
pub(crate) fn superpose_member_loads(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    loads: &[MemberLoad],
    forces: &mut squid_n_element::beam::MemberForces,
) {
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
        let fixed = squid_n_element::member_load::fixed_internal_local(
            loads,
            &frame,
            length,
            *xi,
            crate::assemble::span_load_transfer(elem),
        );
        for k in 0..6 {
            vals[k] += fixed[k];
        }
    }
}

#[cfg(test)]
mod tests;
