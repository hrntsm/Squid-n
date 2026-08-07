//! 床格子（小梁）の二段階サブストラクチャ解析（床 Phase F の中核）。
//!
//! 床の小梁を独立した小さな `Model`（格子）として組み、既存の線形静的ソルバで
//! 解いて、(a) 各小梁の部材力（設計用）と (b) 大梁接続点の**支点反力**（大梁へ渡す
//! CMQ 荷重）を取り出す。本体架構は分割せず、受け取るのは反力のみ。
//!
//! 反力はソルバが直接返さないため、要素の全体剛性から `K·u` を集計し、外力
//! （節点荷重＋部材荷重の等価節点力）を差し引いて求める（`reaction = K·u − F_ext`。
//! 拘束自由度でのみ意味を持つ）。交点ジョイントは、剛接十字＝交点を共有節点として
//! 二方向曲げ連続、ピン受け/架け＝架け梁側に座標一致の別節点を設け受け梁節点と
//! **鉛直 Uz のみ**を `RigidLink` で結合（曲げは伝えず鉛直せん断のみ）で表現する。

use squid_n_core::ids::{ElemId, LoadCaseId, NodeId};
use squid_n_core::model::{MaterialCategory, Model, Slab};
use squid_n_element::beam::MemberForces;

/// 床格子サブモデルと、支点（大梁接続点）→本体モデルの原節点 id の対応。
/// 支点反力を本体の大梁 CMQ（原節点への集中荷重）へ写すために保持する。
pub struct SlabGrillage {
    /// 独立サブモデル（本体架構を含まない小梁だけの格子）。
    pub model: Model,
    /// サブモデル節点インデックス → 本体モデルの原節点 id（大梁接続点のみ）。
    pub support_origin: Vec<(usize, NodeId)>,
    /// サブモデル要素インデックス → 元の小梁インデックス（設計結果の帰属用）。
    pub elem_joist: Vec<(usize, usize)>,
}

/// 2 線分（XY 平面）の内部交点。端部近傍（`t,u ∈ (eps,1-eps)`）でのみ交差とみなす。
/// 平行・端点接触は `None`。z は `p0` の値（床は水平面と仮定）。
fn segment_intersection(
    p0: [f64; 3],
    p1: [f64; 3],
    q0: [f64; 3],
    q1: [f64; 3],
) -> Option<[f64; 3]> {
    let r = [p1[0] - p0[0], p1[1] - p0[1]];
    let s = [q1[0] - q0[0], q1[1] - q0[1]];
    let denom = r[0] * s[1] - r[1] * s[0];
    if denom.abs() < 1e-9 {
        return None; // 平行
    }
    let qp = [q0[0] - p0[0], q0[1] - p0[1]];
    let t = (qp[0] * s[1] - qp[1] * s[0]) / denom;
    let u = (qp[0] * r[1] - qp[1] * r[0]) / denom;
    let eps = 1e-6;
    if t > eps && t < 1.0 - eps && u > eps && u < 1.0 - eps {
        Some([p0[0] + t * r[0], p0[1] + t * r[1], p0[2]])
    } else {
        None
    }
}

/// 節点座標のレジストリ（座標一致で重複排除。交点を両小梁で共有させるため）。
struct NodeRegistry {
    coords: Vec<[f64; 3]>,
}
impl NodeRegistry {
    fn get_or_add(&mut self, c: [f64; 3]) -> usize {
        for (i, e) in self.coords.iter().enumerate() {
            if (e[0] - c[0]).abs() < 1e-3
                && (e[1] - c[1]).abs() < 1e-3
                && (e[2] - c[2]).abs() < 1e-3
            {
                return i;
            }
        }
        self.coords.push(c);
        self.coords.len() - 1
    }
    /// 座標一致でも重複排除せず、必ず新しい節点を割り当てる（ピン受け/架けの
    /// 交点で、架け梁側に受け梁と座標一致する別節点を作るために用いる）。
    fn add_distinct(&mut self, c: [f64; 3]) -> usize {
        self.coords.push(c);
        self.coords.len() - 1
    }
}

/// スラブの小梁から床格子サブモデルを構築する（床 Phase F-2）。
///
/// - 各小梁は支持2節点間の線分。小梁どうしの内部交点を検出し、交点に節点を作って
///   両小梁をそこで分割する（交点は共有節点＝**剛接十字**。既存の点反力モデルと異なり
///   二方向の曲げ剛性が働く）。
/// - 小梁の端部節点（大梁接続点）は鉛直支持（`Ux,Uy,Uz,Rz` 拘束・`Rx,Ry` 自由の
///   単純支持）とする。大梁は分割しない。
/// - 各小梁分割区間に負担幅の等分布荷重 `w·spacing`（下向き）を載せる。
/// - 全小梁に断面が必要（`JoistLine.section`）。欠ける・支持節点が無効・交点がない
///   （＝格子でない）場合は `None`（呼び出し側は既存の単純梁設計へフォールバック）。
///
/// `w` は面荷重強度 [N/mm²]（床用）。返り値の荷重ケースは `LoadCaseId(0)`。
pub fn build_slab_grillage(model: &Model, slab: &Slab, w: f64) -> Option<SlabGrillage> {
    use squid_n_core::dof::{Dof, Dof6Mask};
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCaseKind, LocalAxis,
        Material, MemberLoad, MemberLoadKind, Node, Section,
    };

    if slab.joists.is_empty() {
        return None;
    }
    // 小梁の幾何（端点座標・原節点 id・負担幅・原断面 id）を収集。全小梁に断面必須。
    struct J {
        a: [f64; 3],
        b: [f64; 3],
        a_id: NodeId,
        b_id: NodeId,
        spacing: f64,
        sec: SectionId,
        idx: usize,
    }
    let mut js: Vec<J> = Vec::new();
    for (idx, j) in slab.joists.iter().enumerate() {
        let a = model.nodes.get(j.support[0].index())?;
        let b = model.nodes.get(j.support[1].index())?;
        let sec = j.section?;
        if sec.index() >= model.sections.len() {
            return None;
        }
        js.push(J {
            a: a.coord,
            b: b.coord,
            a_id: j.support[0],
            b_id: j.support[1],
            spacing: j.spacing,
            sec,
            idx,
        });
    }

    // 斜め小梁（軸非平行）は支点の主曲げ解放を単一の全体回転軸に丸めるため部分固定
    // となり、設計曲げを過小評価しうる（非保守）。安全のため全小梁が軸平行（直交格子）
    // でない場合は格子解析を行わず `None` を返し、単純梁設計（各小梁 M=wL²/8。交点の
    // 中間支持を無視するぶん安全側）へフォールバックする。斜め小梁の正確な支点条件
    // （任意方向の主曲げ解放＋ねじり拘束）は要素端条件が全回転一括のため表現できない。
    for j in &js {
        let dx = (j.b[0] - j.a[0]).abs();
        let dy = (j.b[1] - j.a[1]).abs();
        let maxc = dx.max(dy);
        // 許容 tan≈0.02（約1.1°）。maxc≈0 は幾何無効（呼び出し前提で通常起きない）。
        if maxc <= 1e-9 || dx.min(dy) > 0.02 * maxc {
            return None;
        }
    }

    let mut reg = NodeRegistry { coords: Vec::new() };
    // 端点は原節点座標で登録し、原節点 id を覚える（支点＝大梁接続点）。
    // 小梁が端点を共有する場合の重複は排除する（反力の二重計上を防ぐ）。
    let mut support_origin: Vec<(usize, NodeId)> = Vec::new();
    for j in &js {
        let ia = reg.get_or_add(j.a);
        if !support_origin.iter().any(|(n, _)| *n == ia) {
            support_origin.push((ia, j.a_id));
        }
        let ib = reg.get_or_add(j.b);
        if !support_origin.iter().any(|(n, _)| *n == ib) {
            support_origin.push((ib, j.b_id));
        }
    }
    // 交点検出: 各小梁について、他小梁との内部交点を (t, sub_node, 相手小梁の原index)
    // で収集する（相手 index はピン接合〔受け/架け〕の判定に用いる）。
    //
    // ピン受け/架け（`joists[i].pinned_onto == Some(相手)`）の交点では、架け梁 i の側に
    // 受け梁と**座標一致する別節点**を作り（`add_distinct`）、その鉛直変位のみを受け梁
    // 節点に `RigidLink`（Uz のみ）で結合する。これにより架け梁は自身は連続のまま
    // （曲げ・ねじり連続＝機構〔特異〕を生じない）、受け梁との間では曲げは伝えず鉛直
    // せん断だけを伝える単純支持となる。剛接十字は交点を共有節点として二方向曲げを連続。
    //
    // `major_bending_dof`: 水平梁の「主曲げ回転（鉛直たわみの勾配に対応する回転）」に
    // 対応する全体回転自由度。X 主方向の梁は 主曲げ=Ry（ねじり=Rx）、Y 主方向の梁は
    // 主曲げ=Rx（ねじり=Ry）。小梁は大梁に両端ピン（＝主曲げ解放）で取り付くが、
    // ねじりは接合部で拘束される（大梁がねじり戻しを受け持つ）ため支点では解放しない。
    let major_bending_dof = |a: [f64; 3], b: [f64; 3]| -> Dof {
        if (b[0] - a[0]).abs() >= (b[1] - a[1]).abs() {
            Dof::Ry // X 主方向 → 主曲げは Ry
        } else {
            Dof::Rx // Y 主方向 → 主曲げは Rx
        }
    };
    let mut crossings = false;
    let mut per_joist_pts: Vec<Vec<(f64, usize, usize)>> = vec![Vec::new(); js.len()];
    // (受け梁の共有節点, 架け梁の別節点): Uz を結合する鉛直リンク。
    let mut uz_links: Vec<(usize, usize)> = Vec::new();
    for i in 0..js.len() {
        for k in 0..js.len() {
            if i == k {
                continue;
            }
            if let Some(p) = segment_intersection(js[i].a, js[i].b, js[k].a, js[k].b) {
                crossings = true;
                // 受け梁（または剛接十字）が使う共有節点。
                let shared = reg.get_or_add(p);
                // t パラメータ（a→b 上の位置）。
                let ab = [js[i].b[0] - js[i].a[0], js[i].b[1] - js[i].a[1]];
                let ap = [p[0] - js[i].a[0], p[1] - js[i].a[1]];
                let len2 = ab[0] * ab[0] + ab[1] * ab[1];
                let t = if len2 > 1e-12 {
                    (ap[0] * ab[0] + ap[1] * ab[1]) / len2
                } else {
                    0.0
                };
                // i が k にピン接合（架け）なら i 用の別節点を作り Uz 結合。
                // 相互ピン（i→k かつ k→i）の場合は両側が別節点を持つと共有節点が
                // どの部材にも接続されず特異になるため、原インデックスの小さい側だけを
                // 架け（別節点）とし、他方は共有節点を使う受け梁として破綻を防ぐ。
                let i_pins_k = slab.joists[js[i].idx].pinned_onto == Some(js[k].idx);
                let k_pins_i = slab.joists[js[k].idx].pinned_onto == Some(js[i].idx);
                let i_is_carried = i_pins_k && (!k_pins_i || js[i].idx < js[k].idx);
                let node_i = if i_is_carried {
                    let fresh = reg.add_distinct(p);
                    uz_links.push((shared, fresh));
                    fresh
                } else {
                    shared
                };
                per_joist_pts[i].push((t, node_i, js[k].idx));
            }
        }
    }
    if !crossings {
        // 交差がなければ格子ではない（既存の単純梁設計で十分）。
        return None;
    }

    // 断面レジストリ（原 SectionId → サブ index）。
    let mut sec_map: Vec<(SectionId, usize)> = Vec::new();
    let mut sub_sections: Vec<Section> = Vec::new();
    let sub_sec_id = |orig: SectionId,
                      sub_sections: &mut Vec<Section>,
                      sec_map: &mut Vec<(SectionId, usize)>|
     -> usize {
        if let Some((_, idx)) = sec_map.iter().find(|(o, _)| *o == orig) {
            return *idx;
        }
        let mut s = model.sections[orig.index()].clone();
        let idx = sub_sections.len();
        s.id = SectionId(idx as u32);
        // 格子解析用のサブモデルは材料を 1 つしか持たないため、断面の材料参照は
        // すべてその既定鋼材へ差し替える（剛性のみを見る解析であり、
        // 鉄筋・内蔵鉄骨の材料は使わない）。
        s.material = Some(MaterialId(0));
        s.rebar_material = None;
        s.shear_rebar_material = None;
        s.steel_material = None;
        sub_sections.push(s);
        sec_map.push((orig, idx));
        idx
    };

    // 要素・荷重を構築。各小梁を交点で分割。
    let mut elements: Vec<ElementData> = Vec::new();
    let mut member_loads: Vec<MemberLoad> = Vec::new();
    let mut elem_joist: Vec<(usize, usize)> = Vec::new();
    for j in &js {
        let ia = reg.get_or_add(j.a);
        let ib = reg.get_or_add(j.b);
        // 分割点を t 昇順に並べ、両端を挟んで区間列を作る。
        let mut pts: Vec<(f64, usize, usize)> = per_joist_pts[j.idx].clone();
        pts.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut chain: Vec<usize> = vec![ia];
        for (_, ni, _) in &pts {
            if *chain.last().unwrap() != *ni {
                chain.push(*ni);
            }
        }
        if *chain.last().unwrap() != ib {
            chain.push(ib);
        }
        // 全区間とも端部は曲げ連続（Fixed）。受け/架けの曲げ解放は、架け梁側の
        // 別節点と受け梁節点を Uz のみ結合する `RigidLink` で表現しており、部材端では
        // 解放しない（架け梁は自身は連続＝機構を生じない）。
        let sec_idx = sub_sec_id(j.sec, &mut sub_sections, &mut sec_map);
        let w_udl = w * j.spacing; // 負担幅の等分布荷重（下向き）。
        for seg in chain.windows(2) {
            let (n0, n1) = (seg[0], seg[1]);
            if n0 == n1 {
                continue;
            }
            let eid = elements.len() as u32;
            elements.push(ElementData {
                id: ElemId(eid),
                kind: ElementKind::Beam,
                nodes: [NodeId(n0 as u32), NodeId(n1 as u32)].into_iter().collect(),
                section: Some(SectionId(sec_idx as u32)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                // 全区間 Fixed（曲げ連続）。受け/架けは Uz-only RigidLink で表現。
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
            let seg_len = {
                let p0 = reg.coords[n0];
                let p1 = reg.coords[n1];
                let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
            };
            member_loads.push(MemberLoad::auto(
                ElemId(eid),
                [0.0, 0.0, -1.0],
                MemberLoadKind::Distributed {
                    a: 0.0,
                    b: seg_len,
                    w1: w_udl,
                    w2: w_udl,
                },
            ));
            elem_joist.push((eid as usize, j.idx));
        }
    }

    // 支点集合（端点＝大梁接続点）。
    let support_nodes: std::collections::HashSet<usize> =
        support_origin.iter().map(|(n, _)| *n).collect();
    // 支点拘束（向き依存）: 既定は全並進＋全回転を固定し、そこに取り付く各小梁の
    // **主曲げ回転のみ**を解放する（＝大梁への両端ピン）。ねじり（自軸回転）は
    // 固定のまま残す。これにより (1) ねじり剛体回転〔機構〕を生じず、(2) 剛接十字が
    // 交差材のねじり剛性で曲げ連続を効かせる（たわみ抑制）ことができる。
    let mut sup_free: Vec<Dof6Mask> = vec![Dof6Mask::FREE; reg.coords.len()];
    for j in &js {
        let ia = reg.get_or_add(j.a);
        let ib = reg.get_or_add(j.b);
        let major = major_bending_dof(j.a, j.b);
        sup_free[ia].set_fixed(major); // ここでは「解放する回転」を記録（後で set_free）。
        sup_free[ib].set_fixed(major);
    }
    let nodes: Vec<Node> = reg
        .coords
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let r = if support_nodes.contains(&i) {
                let mut m = Dof6Mask::FIXED; // 全並進＋全回転固定を基準に、
                for d in [Dof::Ux, Dof::Uy, Dof::Uz, Dof::Rx, Dof::Ry, Dof::Rz] {
                    if sup_free[i].is_fixed(d) {
                        m.set_free(d); // 主曲げ回転だけ解放（両端ピン）。
                    }
                }
                m
            } else {
                Dof6Mask::FREE
            };
            Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: r,
                mass: None,
                story: None,
                support_spring: None,
            }
        })
        .collect();

    let materials = vec![Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "小梁鋼材(既定)".into(),
        category: MaterialCategory::Steel,
        young: STEEL_YOUNG,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    }];

    // ピン受け/架けの鉛直リンク: 架け梁の別節点(fresh, master)と受け梁の共有節点
    // (shared, slave)の Uz のみを結合。曲げは伝えず鉛直せん断だけを伝える。
    let mut uz_mask = Dof6Mask::FREE;
    uz_mask.set_fixed(Dof::Uz);
    let constraints: Vec<squid_n_core::model::Constraint> = uz_links
        .iter()
        .map(
            |(shared, fresh)| squid_n_core::model::Constraint::RigidLink {
                master: NodeId(*fresh as u32),
                slaves: vec![NodeId(*shared as u32)],
                dofs: uz_mask,
            },
        )
        .collect();

    let sub = Model {
        nodes,
        elements,
        sections: sub_sections,
        materials,
        constraints,
        load_cases: vec![LoadCase {
            id: LoadCaseId(0),
            name: "床格子".into(),
            kind: LoadCaseKind::Dead,
            nodal: vec![],
            member: member_loads,
        }],
        // 格子解析では小梁のねじり剛性を保持する。剛接十字（`pinned_onto` 無し）は
        // 交差する相手梁の**ねじり**で曲げを受け止めるモデル化そのものであり、
        // 主架構の既定（梁の i 端ねじれ解放）を持ち込むと剛接十字とピン受けの
        // 区別が消えてしまう（架け方の指定が結果に反映されなくなる）。
        beam_torsion: squid_n_core::model::BeamTorsionMode::Keep,
        ..Default::default()
    };
    sub.validate().ok()?;

    Some(SlabGrillage {
        model: sub,
        support_origin,
        elem_joist,
    })
}

/// 鋼小梁の既定ヤング係数 [N/mm²]（設計モジュールと同一）。
const STEEL_YOUNG: f64 = 205_000.0;

/// 床格子サブモデルの解。
pub struct GrillageSolution {
    /// 各節点の変位 `[Ux,Uy,Uz,Rx,Ry,Rz]`（たわみ算定に用いる）。
    pub disp: Vec<[f64; 6]>,
    /// 各小梁要素の部材力（設計に用いる）。
    pub member_forces: Vec<(ElemId, MemberForces)>,
    /// 各節点の全体系反力 `[Fx,Fy,Fz,Mx,My,Mz]`。拘束自由度でのみ有意
    /// （非拘束自由度は釣合いよりほぼ 0）。大梁 CMQ には鉛直成分 `Fz` を用いる。
    pub reactions: Vec<[f64; 6]>,
}

/// 床格子サブモデル `model` の荷重ケース `lc` を解き、部材力と支点反力を返す。
/// `model` は呼び出し側が構築した独立サブモデル（本体架構を含まない）。
pub fn solve_grillage(model: &Model, lc: LoadCaseId) -> Result<GrillageSolution, String> {
    let once = squid_n_solver::linear::linear_static_once(model, lc)
        .map_err(|e| format!("床格子の求解に失敗: {e:?}"))?;
    let reactions = compute_reactions(model, lc, &once.disp);
    Ok(GrillageSolution {
        disp: once.disp,
        member_forces: once.member_forces,
        reactions,
    })
}

/// 格子解から各小梁の設計用部材力を取り出す（床 Phase F-3）。
/// `(小梁インデックス, スパン, |M|max, |Q|max, |δ|max)`。
///
/// - 曲げ `M` は各分割区間・評価点の `|My|,|Mz|` の最大（局所軸規約に頑健化）。
/// - せん断 `Q` は `|Qy|,|Qz|` の最大。
/// - たわみ `δ` はその小梁が通る節点の鉛直変位 `|Uz|` の最大（支点は ~0）。
/// - スパンは分割区間長の総和（＝小梁全長）。
pub fn joist_design_forces(
    grillage: &SlabGrillage,
    sol: &GrillageSolution,
) -> Vec<(usize, f64, f64, f64, f64)> {
    let m = &grillage.model;
    // 小梁ごとに要素・節点を集約。
    let n_joists = grillage
        .elem_joist
        .iter()
        .map(|(_, j)| *j + 1)
        .max()
        .unwrap_or(0);
    let mut span = vec![0.0f64; n_joists];
    let mut m_max = vec![0.0f64; n_joists];
    let mut q_max = vec![0.0f64; n_joists];
    let mut nodes_of: Vec<Vec<usize>> = vec![Vec::new(); n_joists];

    for (eidx, jidx) in &grillage.elem_joist {
        let elem = &m.elements[*eidx];
        let ni = elem.nodes[0].index();
        let nj = elem.nodes[1].index();
        // スパン（区間長）を加算。
        let p0 = m.nodes[ni].coord;
        let p1 = m.nodes[nj].coord;
        let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        span[*jidx] += (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        for n in [ni, nj] {
            if !nodes_of[*jidx].contains(&n) {
                nodes_of[*jidx].push(n);
            }
        }
        // 部材力の最大を集計。
        if let Some((_, mf)) = sol.member_forces.iter().find(|(id, _)| *id == elem.id) {
            for (_, f) in &mf.at {
                let mm = f[4].abs().max(f[5].abs()); // |My|,|Mz|
                let qq = f[1].abs().max(f[2].abs()); // |Qy|,|Qz|
                if mm > m_max[*jidx] {
                    m_max[*jidx] = mm;
                }
                if qq > q_max[*jidx] {
                    q_max[*jidx] = qq;
                }
            }
        }
    }

    (0..n_joists)
        .map(|j| {
            let defl = nodes_of[j]
                .iter()
                .map(|&n| sol.disp[n][2].abs())
                .fold(0.0f64, f64::max);
            (j, span[j], m_max[j], q_max[j], defl)
        })
        .collect()
}

/// `reaction = K·u − F_ext` を全節点・全成分について求める。
/// `K·u` は各要素の全体剛性 × 節点変位を集計、`F_ext` は節点荷重＋部材荷重の
/// 等価節点力（`assemble` と同じ `consistent_load_local` を用いる）。
fn compute_reactions(model: &Model, lc: LoadCaseId, disp: &[[f64; 6]]) -> Vec<[f64; 6]> {
    use squid_n_element::behavior::Ctx;
    use squid_n_element::transform::LocalFrame;

    let n = model.nodes.len();
    // 内力 K·u（全体系）を節点へ集計。
    let mut p_int = vec![[0.0f64; 6]; n];
    for elem in &model.elements {
        if elem.nodes.len() < 2 {
            continue;
        }
        let ni = elem.nodes[0].index();
        let nj = elem.nodes[1].index();
        if ni >= n || nj >= n {
            continue;
        }
        let behavior = squid_n_element::build_behavior(elem, model);
        let k = behavior.tangent_stiffness(&Ctx { model });
        // u_global（12）= [i:0..6, j:6..12]
        let mut u = [0.0f64; 12];
        u[0..6].copy_from_slice(&disp[ni]);
        u[6..12].copy_from_slice(&disp[nj]);
        // f = K·u（全体系）
        for (i, pf) in [ni, nj].into_iter().enumerate() {
            for (d, pd) in p_int[pf].iter_mut().enumerate() {
                let row = i * 6 + d;
                let mut s = 0.0;
                for (j, &uj) in u.iter().enumerate() {
                    s += k.get(row, j) * uj;
                }
                *pd += s;
            }
        }
    }

    // 外力 F_ext（節点荷重＋部材荷重の等価節点力）。
    let mut f_ext = vec![[0.0f64; 6]; n];
    if let Some(case) = model.load_cases.iter().find(|c| c.id == lc) {
        for nl in &case.nodal {
            let idx = nl.node.index();
            if idx < n {
                for (fd, &v) in f_ext[idx].iter_mut().zip(nl.values.iter()) {
                    *fd += v;
                }
            }
        }
        for elem in &model.elements {
            if elem.nodes.len() < 2 {
                continue;
            }
            let loads: Vec<_> = case
                .member
                .iter()
                .filter(|ml| ml.elem == elem.id)
                .cloned()
                .collect();
            if loads.is_empty() {
                continue;
            }
            let ni = elem.nodes[0].index();
            let nj = elem.nodes[1].index();
            if ni >= n || nj >= n {
                continue;
            }
            let p_i = model.nodes[ni].coord;
            let p_j = model.nodes[nj].coord;
            let length = {
                let d = [p_j[0] - p_i[0], p_j[1] - p_i[1], p_j[2] - p_i[2]];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
            };
            if length < 1e-9 {
                continue;
            }
            let frame = LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector);
            // 床格子は大梁・小梁のみで構成され、ブレースは現れない
            // （`SpanLoadTransfer::Consistent` 固定でよい）。
            let q_local = squid_n_element::member_load::consistent_load_local(
                &loads,
                &frame,
                length,
                squid_n_element::member_load::SpanLoadTransfer::Consistent,
            );
            let q_global = frame.rotate_to_global(&q_local);
            for (i, pf) in [ni, nj].into_iter().enumerate() {
                for (d, fd) in f_ext[pf].iter_mut().enumerate() {
                    *fd += q_global[i * 6 + d];
                }
            }
        }
    }

    // reaction = K·u − F_ext
    let mut reactions = vec![[0.0f64; 6]; n];
    for ((r, pi), fe) in reactions.iter_mut().zip(p_int.iter()).zip(f_ext.iter()) {
        for (rd, (pd, fd)) in r.iter_mut().zip(pi.iter().zip(fe.iter())) {
            *rd = pd - fd;
        }
    }
    reactions
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCaseKind, LocalAxis,
        Material, MemberLoad, MemberLoadKind, Node, Section,
    };

    fn beam_section(id: u32) -> Section {
        Section {
            id: SectionId(id),
            name: "H".into(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e6,
            depth: 400.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            // 材料は断面が持つ。
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }
    fn steel(id: u32) -> Material {
        Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(id),
            name: "SN400".into(),
            category: MaterialCategory::Steel,
            young: 205_000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }
    }

    /// 両端固定梁の UDL: 鉛直反力は各端 wL/2、総和 wL（反力抽出の検算）。
    #[test]
    fn test_grillage_reaction_fixed_fixed_udl() {
        let l = 4000.0_f64;
        let w = 10.0_f64; // N/mm（下向き）
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [l, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![beam_section(0)],
            materials: vec![steel(0)],
            load_cases: vec![LoadCase {
                id: LoadCaseId(0),
                name: "床".into(),
                kind: LoadCaseKind::Dead,
                nodal: vec![],
                member: vec![MemberLoad::manual(
                    ElemId(0),
                    [0.0, 0.0, -1.0],
                    MemberLoadKind::Distributed {
                        a: 0.0,
                        b: l,
                        w1: w,
                        w2: w,
                    },
                )],
            }],
            ..Default::default()
        };
        model.validate().expect("submodel validate");

        let sol = solve_grillage(&model, LoadCaseId(0)).expect("solve");
        let total = w * l;
        // 鉛直反力（+Z 上向き）は各端 wL/2。
        assert!(
            (sol.reactions[0][2] - total / 2.0).abs() / (total / 2.0) < 1e-6,
            "R0z={}",
            sol.reactions[0][2]
        );
        assert!(
            (sol.reactions[1][2] - total / 2.0).abs() / (total / 2.0) < 1e-6,
            "R1z={}",
            sol.reactions[1][2]
        );
        // 総和 = 全載荷。
        let sum = sol.reactions[0][2] + sol.reactions[1][2];
        assert!((sum - total).abs() / total < 1e-6, "sum={sum}");
    }

    /// 対称な十字小梁: 格子を構築して解き、支点反力の総和＝全載荷、4支点が対称。
    #[test]
    fn test_build_and_solve_symmetric_cross() {
        use squid_n_core::ids::SlabId;
        use squid_n_core::model::{DistributionMethod, JoistLine, Slab};

        let mk = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let model = Model {
            nodes: vec![
                mk(0, 0.0, 0.0),
                mk(1, 4000.0, 0.0),
                mk(2, 4000.0, 4000.0),
                mk(3, 0.0, 4000.0),
                mk(4, 2000.0, 0.0),    // mid-bottom
                mk(5, 2000.0, 4000.0), // mid-top
                mk(6, 0.0, 2000.0),    // mid-left
                mk(7, 4000.0, 2000.0), // mid-right
            ],
            sections: vec![beam_section(0)],
            slabs: vec![Slab {
                id: SlabId(0),
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                joists: vec![
                    JoistLine {
                        dir: [0.0, 1.0],
                        spacing: 2000.0,
                        support: [NodeId(4), NodeId(5)], // 縦（x=2000）
                        section: Some(SectionId(0)),
                        pinned_onto: None,
                    },
                    JoistLine {
                        dir: [1.0, 0.0],
                        spacing: 2000.0,
                        support: [NodeId(6), NodeId(7)], // 横（y=2000）
                        section: Some(SectionId(0)),
                        pinned_onto: None,
                    },
                ],
                loads: vec![],
                method: DistributionMethod::TriTrapezoid,
                kind: Default::default(),
                one_way: None,
                edge_supported: None,
                usage: None,
                section: None,
            }],
            ..Default::default()
        };

        let w = 0.005_f64; // N/mm²
        let g = build_slab_grillage(&model, &model.slabs[0], w).expect("格子構築");
        // 交点（2000,2000）で分割 → 各小梁2区間・計4要素、節点は端点4＋交点1＝5。
        assert_eq!(g.model.nodes.len(), 5);
        assert_eq!(g.model.elements.len(), 4);

        let sol = solve_grillage(&g.model, LoadCaseId(0)).expect("solve");
        // 総載荷 = w·spacing·L × 2本 = 0.005·2000·4000·2。
        let total = w * 2000.0 * 4000.0 * 2.0;
        // 支点（端点4）の鉛直反力総和。
        let support_sum: f64 = g
            .support_origin
            .iter()
            .map(|(n, _)| sol.reactions[*n][2])
            .sum();
        assert!(
            (support_sum - total).abs() / total < 1e-6,
            "支点反力総和={support_sum} 全載荷={total}"
        );
        // 対称性: 4支点の反力がほぼ等しい。
        let rs: Vec<f64> = g
            .support_origin
            .iter()
            .map(|(n, _)| sol.reactions[*n][2])
            .collect();
        let r0 = rs[0];
        for r in &rs {
            assert!((r - r0).abs() / r0 < 1e-6, "非対称: {rs:?}");
        }
    }

    /// ピン受け/架け（`pinned_onto`）と剛接十字で結果が異なることを確認する。
    /// 同一の対称十字を、(a) 両端固定＝剛接、(b) 小梁0 を小梁1 にピン接合＝架け、
    /// で解き、架け梁（小梁0）の交点での曲げが解放されて設計曲げが変わることを見る。
    #[test]
    fn test_pin_vs_rigid_cross_differ() {
        use squid_n_core::ids::SlabId;
        use squid_n_core::model::{DistributionMethod, JoistLine, Slab};

        let mk = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        // 受け梁を y=1000 に置き、架け梁（x=2000, y=0..4000）を 1000/3000 に非対称分割
        // する。非対称だと交点回転が非ゼロになり、剛接（回転拘束）とピン（回転自由）で
        // 曲げ分布が明確に変わる（対称分割だと交点回転が 0 で差が出ない）。
        let base_nodes = vec![
            mk(0, 0.0, 0.0),
            mk(1, 4000.0, 0.0),
            mk(2, 4000.0, 4000.0),
            mk(3, 0.0, 4000.0),
            mk(4, 2000.0, 0.0),
            mk(5, 2000.0, 4000.0),
            mk(6, 0.0, 1000.0),
            mk(7, 4000.0, 1000.0),
        ];
        // 小梁0＝縦（x=2000, 節点4-5, 通常剛性・架け）,
        // 小梁1＝横（y=2000, 節点6-7, 高剛性・受け）。受け梁を格段に硬くすると
        // 交点で実際に鉛直荷重が伝達され、ピン/剛接の違いが表面化する
        //（対称・同剛性だと交点で荷重が伝わらず差が出ない）。
        // 剛接十字の曲げ連続は交差材の「ねじり剛性(GJ)」で効くため、`j` も大きくする。
        // これにより剛接では架け梁の交点回転が拘束され（部分固定）、ピン（回転自由）と
        // 明確に異なる曲げ分布になる。
        let stiff = {
            let mut s = beam_section(1);
            s.iy = 1.0e11;
            s.iz = 1.0e11;
            s.j = 1.0e11;
            s
        };
        let make_model = |pinned_onto: Option<usize>| Model {
            nodes: base_nodes.clone(),
            sections: vec![beam_section(0), stiff.clone()],
            slabs: vec![Slab {
                id: SlabId(0),
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                joists: vec![
                    JoistLine {
                        dir: [0.0, 1.0],
                        spacing: 2000.0,
                        support: [NodeId(4), NodeId(5)],
                        section: Some(SectionId(0)),
                        pinned_onto, // 小梁0 を小梁1 にピン（架け）
                    },
                    JoistLine {
                        dir: [1.0, 0.0],
                        spacing: 2000.0,
                        support: [NodeId(6), NodeId(7)],
                        section: Some(SectionId(1)),
                        pinned_onto: None,
                    },
                ],
                loads: vec![],
                method: DistributionMethod::TriTrapezoid,
                kind: Default::default(),
                one_way: None,
                edge_supported: None,
                usage: None,
                section: None,
            }],
            ..Default::default()
        };
        let w = 0.005_f64;

        // (a) 剛接十字（pinned_onto=None）。
        let m_rigid = make_model(None);
        let g_rigid = build_slab_grillage(&m_rigid, &m_rigid.slabs[0], w).expect("剛接構築");
        let s_rigid = solve_grillage(&g_rigid.model, LoadCaseId(0)).expect("剛接解");
        let f_rigid = joist_design_forces(&g_rigid, &s_rigid);

        // (b) ピン受け/架け（小梁0 を小梁1 にピン接合）。
        let m_pin = make_model(Some(1));
        let g_pin = build_slab_grillage(&m_pin, &m_pin.slabs[0], w).expect("ピン構築");
        let s_pin = solve_grillage(&g_pin.model, LoadCaseId(0)).expect("ピン解");
        let f_pin = joist_design_forces(&g_pin, &s_pin);

        // 架け梁（小梁0）の設計曲げは、剛接とピンで有意に異なる。
        let m0_rigid = f_rigid[0].2;
        let m0_pin = f_pin[0].2;
        assert!(
            (m0_rigid - m0_pin).abs() / m0_rigid.max(1.0) > 0.05,
            "架け梁の曲げが剛接({m0_rigid})とピン({m0_pin})で変わらない"
        );
        // 総載荷は接合条件によらず不変（釣合いの健全性）。
        let total = w * 2000.0 * 4000.0 * 2.0;
        let sum_pin: f64 = g_pin
            .support_origin
            .iter()
            .map(|(n, _)| s_pin.reactions[*n][2])
            .sum();
        assert!(
            (sum_pin - total).abs() / total < 1e-6,
            "ピン時の支点反力総和={sum_pin} 全載荷={total}"
        );
    }

    /// 斜め小梁（軸非平行）を含む床は格子解析を行わず None（安全側の単純梁へ）。
    #[test]
    fn test_skew_joist_falls_back_to_none() {
        use squid_n_core::ids::SlabId;
        use squid_n_core::model::{DistributionMethod, JoistLine, Slab};

        let mk = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let model = Model {
            nodes: vec![
                mk(0, 0.0, 0.0),
                mk(1, 4000.0, 0.0),
                mk(2, 4000.0, 4000.0),
                mk(3, 0.0, 4000.0),
                mk(4, 2000.0, 0.0),
                mk(5, 2000.0, 4000.0),
            ],
            sections: vec![beam_section(0)],
            slabs: vec![Slab {
                id: SlabId(0),
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                joists: vec![
                    JoistLine {
                        dir: [0.0, 1.0],
                        spacing: 2000.0,
                        support: [NodeId(4), NodeId(5)], // 縦
                        section: Some(SectionId(0)),
                        pinned_onto: None,
                    },
                    JoistLine {
                        dir: [1.0, 1.0],
                        spacing: 2000.0,
                        support: [NodeId(0), NodeId(2)], // 斜め（対角）
                        section: Some(SectionId(0)),
                        pinned_onto: None,
                    },
                ],
                loads: vec![],
                method: DistributionMethod::TriTrapezoid,
                kind: Default::default(),
                one_way: None,
                edge_supported: None,
                usage: None,
                section: None,
            }],
            ..Default::default()
        };
        assert!(
            build_slab_grillage(&model, &model.slabs[0], 0.005).is_none(),
            "斜め小梁があるのに格子を構築した（過小評価の恐れ）"
        );
    }

    /// 相互ピン（i→k かつ k→i）でも特異にならず解ける（低インデックス側だけ架け）。
    #[test]
    fn test_mutual_pin_does_not_singular() {
        use squid_n_core::ids::SlabId;
        use squid_n_core::model::{DistributionMethod, JoistLine, Slab};

        let mk = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let model = Model {
            nodes: vec![
                mk(0, 0.0, 0.0),
                mk(1, 4000.0, 0.0),
                mk(2, 4000.0, 4000.0),
                mk(3, 0.0, 4000.0),
                mk(4, 2000.0, 0.0),
                mk(5, 2000.0, 4000.0),
                mk(6, 0.0, 2000.0),
                mk(7, 4000.0, 2000.0),
            ],
            sections: vec![beam_section(0)],
            slabs: vec![Slab {
                id: SlabId(0),
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                joists: vec![
                    JoistLine {
                        dir: [0.0, 1.0],
                        spacing: 2000.0,
                        support: [NodeId(4), NodeId(5)],
                        section: Some(SectionId(0)),
                        pinned_onto: Some(1), // 相互ピン
                    },
                    JoistLine {
                        dir: [1.0, 0.0],
                        spacing: 2000.0,
                        support: [NodeId(6), NodeId(7)],
                        section: Some(SectionId(0)),
                        pinned_onto: Some(0), // 相互ピン
                    },
                ],
                loads: vec![],
                method: DistributionMethod::TriTrapezoid,
                kind: Default::default(),
                one_way: None,
                edge_supported: None,
                usage: None,
                section: None,
            }],
            ..Default::default()
        };
        let w = 0.005_f64;
        let g = build_slab_grillage(&model, &model.slabs[0], w).expect("相互ピンでも構築");
        let sol = solve_grillage(&g.model, LoadCaseId(0)).expect("相互ピンでも特異にならず解ける");
        let total = w * 2000.0 * 4000.0 * 2.0;
        let sum: f64 = g
            .support_origin
            .iter()
            .map(|(n, _)| sol.reactions[*n][2])
            .sum();
        assert!(
            (sum - total).abs() / total < 1e-6,
            "支点反力総和={sum} 全載荷={total}"
        );
    }
}
