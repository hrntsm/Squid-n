use faer::sparse::SparseColMat;
use squid_n_core::dof::{Dof, DofMap, DOF_PER_NODE};
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_element::factory::build_behavior;
use squid_n_element::transform::LocalFrame;
use squid_n_math::sparse::{assemble_csc, Triplet};

pub fn assemble_global_k(model: &Model, dofmap: &DofMap) -> SparseColMat<usize, f64> {
    let ctx = squid_n_element::behavior::Ctx { model };
    let mut all_triplets = Vec::new();

    for elem in &model.elements {
        let behavior = build_behavior(elem, model);
        let gdofs = behavior.global_dofs(dofmap);
        let k_local = behavior.tangent_stiffness(&ctx);
        let triplets = k_local.to_triplets(&gdofs);
        all_triplets.extend(triplets);
    }

    add_support_spring_diag(model, dofmap, &mut all_triplets);

    assemble_csc(dofmap.n_active(), all_triplets)
}

/// 支点ばね（`Node::support_spring`）の有効項を `(active DOF 番号, ばね剛性 k)` の
/// 列で列挙する。値 0・`restraint` で固定されている自由度（固定支持を優先し
/// ばね値は無視する。[`squid_n_core::model::Node::support_spring`] の仕様）・
/// 不活性（`dofmap` 側で自由度がない＝孤立節点等）の項は含まない。
///
/// 全体剛性の対角加算（[`add_support_spring_diag`]、線形経路）と、非線形経路
/// （`nonlinear::pushover::assembly` の `assemble_k`・`add_support_spring_f_int`）の
/// 双方が同じ列挙結果を使うための共通ヘルパ。ばねは全体座標系の値をそのまま
/// 対角へ用いる線形ばねのため、K 側の対角項も内力側の `k・u` も同じ
/// `(active, k)` の組で足りる。
pub fn support_spring_terms(model: &Model, dofmap: &DofMap) -> Vec<(usize, f64)> {
    let mut terms = Vec::new();
    for (ni, node) in model.nodes.iter().enumerate() {
        let Some(spring) = node.support_spring else {
            continue;
        };
        for (d, &k) in spring.iter().enumerate() {
            if k == 0.0 {
                continue;
            }
            let dof = match d {
                0 => Dof::Ux,
                1 => Dof::Uy,
                2 => Dof::Uz,
                3 => Dof::Rx,
                4 => Dof::Ry,
                _ => Dof::Rz,
            };
            if node.restraint.is_fixed(dof) {
                continue;
            }
            let g = ni * DOF_PER_NODE + d;
            if let Some(active) = dofmap.active(g) {
                terms.push((active as usize, k));
            }
        }
    }
    terms
}

/// 支点ばね（`Node::support_spring`）を全体剛性行列の対角へ加算する。
///
/// 節点集中質量の対角加算（[`assemble_global_m`]）と同じ形で、
/// [`support_spring_terms`] が返す自由 DOF の対角項へ単純加算する。
///
/// 線形経路（本関数内）・非線形経路（`nonlinear::pushover::assembly::assemble_k`）の
/// 双方から呼ばれる共通処理。非線形経路では内力側 `k_i・u_i` の計上も別途必要
/// （`nonlinear::pushover::assembly::add_support_spring_f_int` 参照）。
pub fn add_support_spring_diag(model: &Model, dofmap: &DofMap, triplets: &mut Vec<Triplet>) {
    for (active, k) in support_spring_terms(model, dofmap) {
        triplets.push(Triplet {
            row: active,
            col: active,
            val: k,
        });
    }
}

/// 全体質量行列を組み立てる。
///
/// 質量源は「部材密度による要素質量」と「節点集中質量（`Node::mass`）」の 2 つで、
/// どちらを算入するかはモデルの質量方式（[`squid_n_core::model::MassMethod`]、
/// `Model::mass_method`）に従う:
///
/// - `CorrectedLumped`（既定）: 要素質量＋節点質量（従来どおりの合算）。
///   剛床マスターの節点質量は階生成が「地震用重量−分布質量計上分」の補正値を
///   与えるため、二重計上にならない。
/// - `LumpedOnly`: 節点質量のみ（要素質量は算入しない）。剛床マスターには
///   階生成が地震用重量の全量を与える（水平質点系モデル化）。
pub fn assemble_global_m(
    model: &Model,
    dofmap: &DofMap,
    opt: squid_n_element::behavior::MassOption,
) -> SparseColMat<usize, f64> {
    let mut all_triplets = Vec::new();
    if model.mass_method != squid_n_core::model::MassMethod::LumpedOnly {
        for elem in &model.elements {
            let behavior = build_behavior(elem, model);
            let gdofs = behavior.global_dofs(dofmap);
            let m_local = behavior.mass_matrix(opt);
            let triplets = m_local.to_triplets(&gdofs);
            all_triplets.extend(triplets);
        }
    }

    // 節点集中質量（Node.mass）を対角へ加算する。
    // 床荷重→質量化した層質量や、集中質量モデルの質量はここで反映される。
    // これを欠くと固有値・有効質量比（P2 DoD #2）が物理的に誤る。
    for (ni, node) in model.nodes.iter().enumerate() {
        if let Some(mass) = node.mass {
            for (d, &mval) in mass.iter().enumerate() {
                if mval == 0.0 {
                    continue;
                }
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    all_triplets.push(Triplet {
                        row: active as usize,
                        col: active as usize,
                        val: mval,
                    });
                }
            }
        }
    }

    assemble_csc(dofmap.n_active(), all_triplets)
}

pub fn assemble_global_f(model: &Model, dofmap: &DofMap, lc: LoadCaseId) -> Vec<f64> {
    let n_active = dofmap.n_active();
    let mut f = vec![0.0; n_active];

    // Find the load case
    if let Some(lc_data) = model.load_cases.iter().find(|l| l.id == lc) {
        for nodal_load in &lc_data.nodal {
            let ni = nodal_load.node.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    f[active as usize] += nodal_load.values[d];
                }
            }
        }

        // 部材（梁）荷重 → 等価節点力（consistent load vector）を全体系へ加算。
        add_member_loads(model, dofmap, &lc_data.member, &mut f);
    }

    f
}

/// 部材荷重（等価節点力・固定端内力）を扱える線材要素か。
///
/// 対象は 2 節点の線材（梁・ファイバー梁・マルチスプリング梁・ブレース。
/// [`crate::linear::ensure_line_member_forces`] の対象と同じ集合）。壁・シェル等の
/// 非線材（4 節点）に `MemberLoad` が誤って紐付いた場合、従来は先頭 2 節点だけを
/// 材端とみなして荷重を配ってしまい、エラーも出ずに荷重が誤適用されていた。
pub(crate) fn is_member_load_target(elem: &ElementData) -> bool {
    matches!(
        elem.kind,
        ElementKind::Beam
            | ElementKind::Fiber
            | ElementKind::MultiSpring
            | ElementKind::Brace { .. }
    ) && elem.nodes.len() == 2
}

/// 線材の局所座標系と部材長を返す（部材荷重の等価節点力・固定端内力の共通前処理）。
/// 非対象要素（[`is_member_load_target`] 参照）・節点参照の欠落・退化長さ（<1e-9mm）
/// は `None`。
pub(crate) fn member_load_frame(model: &Model, elem: &ElementData) -> Option<(LocalFrame, f64)> {
    if !is_member_load_target(elem) {
        return None;
    }
    let p_i = model.nodes.get(elem.nodes[0].index())?.coord;
    let p_j = model.nodes.get(elem.nodes[1].index())?.coord;
    let dx = p_j[0] - p_i[0];
    let dy = p_j[1] - p_i[1];
    let dz = p_j[2] - p_i[2];
    let length = (dx * dx + dy * dy + dz * dz).sqrt();
    if length < 1e-9 {
        return None;
    }
    Some((
        LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector),
        length,
    ))
}

/// 部材荷重の等価節点力を local で計算し、全体系へ回して荷重ベクトルへ散布する。
fn add_member_loads(
    model: &Model,
    dofmap: &DofMap,
    member_loads: &[squid_n_core::model::MemberLoad],
    f: &mut [f64],
) {
    use std::collections::HashMap;

    // 要素 ID → 荷重リストへ事前にグルーピングする（従来の全要素×全荷重の
    // 総当り filter は O(要素数×荷重数) で大規模モデルに不利）。
    let mut by_elem: HashMap<squid_n_core::ids::ElemId, Vec<squid_n_core::model::MemberLoad>> =
        HashMap::new();
    for ml in member_loads {
        by_elem.entry(ml.elem).or_default().push(ml.clone());
    }
    if by_elem.is_empty() {
        return;
    }

    for elem in &model.elements {
        let Some(loads) = by_elem.get(&elem.id) else {
            continue;
        };
        let Some((frame, length)) = member_load_frame(model, elem) else {
            continue;
        };
        let ni = elem.nodes[0].index();
        let nj = elem.nodes[1].index();
        let q_local = squid_n_element::member_load::consistent_load_local(loads, &frame, length);
        let q_global = frame.rotate_to_global(&q_local);
        // q_global: [i:0..6, j:6..12] を各節点 DOF へ散布
        for (local_node, &node_idx) in [ni, nj].iter().enumerate() {
            for d in 0..DOF_PER_NODE {
                let g = node_idx * DOF_PER_NODE + d;
                if let Some(active) = dofmap.active(g) {
                    f[active as usize] += q_global[local_node * DOF_PER_NODE + d];
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::{Dof6Mask, DofMap};
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, NodalLoad, Node,
    };

    /// 支点ばね検証用モデル: 節点0(全固定)-節点1(Ux のみ自由) を、零長節点バネ
    /// （軸剛性 `kx`）で結ぶ 1 自由度系。`support_kx` は節点1 の水平支点ばね
    /// （`None` で無指定）。2 節点を同一座標に置くことで
    /// `NodalSpringElement` の局所軸＝全体座標系（零長特例）となり、
    /// 軸バネがそのまま水平（全体 X）成分に一致する。
    fn spring_model(kx: f64, support_kx: Option<f64>, restraint1: Dof6Mask) -> Model {
        Model {
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
                    coord: [0.0, 0.0, 0.0],
                    restraint: restraint1,
                    mass: None,
                    story: None,
                    support_spring: support_kx.map(|k| [k, 0.0, 0.0, 0.0, 0.0, 0.0]),
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::NodalSpring,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: None,
                material: None,
                local_axis: LocalAxis {
                    ref_vector: [0.0, 1.0, 0.0],
                },
                end_cond: [EndCondition::Pinned, EndCondition::Pinned],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: Some([kx, 0.0, 0.0, 0.0, 0.0, 0.0]),
            }],
            load_cases: vec![LoadCase {
                id: LoadCaseId(0),
                name: "TEST".to_string(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                }],
                member: vec![],
                kind: Default::default(),
            }],
            ..Default::default()
        }
    }

    /// 支点ばねが `assemble_global_k` の対角へ、要素剛性と単純加算される形で
    /// 効くこと（`Node::mass` の対角加算と同じ形の検証）。
    #[test]
    fn test_assemble_global_k_adds_support_spring_diagonal() {
        let kx = 1000.0;
        let ks = 2000.0;
        let model_baseline = spring_model(kx, None, Dof6Mask(0b111110));
        let model_spring = spring_model(kx, Some(ks), Dof6Mask(0b111110));

        let dofmap_baseline = DofMap::build(&model_baseline);
        let dofmap_spring = DofMap::build(&model_spring);
        assert_eq!(dofmap_baseline.n_active(), 1);
        assert_eq!(dofmap_spring.n_active(), 1);

        // 線形静解析（`crate::linear::linear_static_once`）で節点1に水平力を与え、
        // 変位 u=F/k から実効剛性を逆算し、要素単独(kx)・要素+支点ばね(kx+ks) の
        // 理論値と一致することを確認する。
        let lc = LoadCaseId(0);
        let disp_baseline = crate::linear::linear_static_once(&model_baseline, lc)
            .expect("baseline linear static should solve")
            .disp[1][0];
        let disp_spring = crate::linear::linear_static_once(&model_spring, lc)
            .expect("spring linear static should solve")
            .disp[1][0];

        let f = 1000.0;
        assert!(
            (disp_baseline - f / kx).abs() < 1e-6 * (f / kx),
            "baseline disp should equal F/kx: got {disp_baseline}, expected {}",
            f / kx
        );
        assert!(
            (disp_spring - f / (kx + ks)).abs() < 1e-6 * (f / (kx + ks)),
            "with support spring, disp should equal F/(kx+ks): got {disp_spring}, expected {}",
            f / (kx + ks)
        );
    }

    /// `restraint` で固定した自由度の支点ばねは無視される（固定支持を優先する仕様）。
    /// 孤立節点（要素非接続）の support_spring だけでは活性自由度が増えないことも
    /// 合わせて確認する（`structural_nodes` の判定は変更しない仕様）。
    #[test]
    fn test_support_spring_on_fixed_dof_is_ignored() {
        // 節点0 は全固定。support_spring を与えても活性自由度に影響しないこと。
        let mut model = spring_model(1000.0, None, Dof6Mask(0b111110));
        model.nodes[0].support_spring = Some([1.0e9, 1.0e9, 1.0e9, 1.0e9, 1.0e9, 1.0e9]);
        let dofmap = DofMap::build(&model);
        assert_eq!(
            dofmap.n_active(),
            1,
            "全固定節点の support_spring は活性 DOF を増やさない"
        );
        let k = assemble_global_k(&model, &dofmap);
        // 活性 DOF はただ1つ（節点1 の Ux）。対角成分は要素剛性 kx のみ
        // （節点0 は固定のため support_spring は無視される）。
        assert_eq!(k.compute_nnz(), 1, "対角成分1つのみ（節点0 の寄与なし）");

        // 節点1 側でも、restraint で固定した成分（Uy 等）の support_spring は無視される。
        let mut model2 = spring_model(1000.0, None, Dof6Mask(0b111110));
        model2.nodes[1].support_spring = Some([500.0, 999.0, 999.0, 999.0, 999.0, 999.0]);
        let dofmap2 = DofMap::build(&model2);
        assert_eq!(dofmap2.n_active(), 1, "Ux 以外は restraint で固定のまま");
        let terms = support_spring_terms(&model2, &dofmap2);
        assert_eq!(
            terms.len(),
            1,
            "固定されている Uy/Uz/Rx/Ry/Rz 成分は列挙されない"
        );
        assert!((terms[0].1 - 500.0).abs() < 1e-9, "Ux 成分のみ有効");
    }

    /// 孤立節点（要素・拘束いずれにも参加しない節点）に support_spring だけを
    /// 与えても、`structural_nodes`（dof.rs）の判定には影響しない
    /// （支点ばねだけの孤立節点は解析対象外のままでよい仕様）。
    #[test]
    fn test_isolated_node_with_support_spring_stays_inactive() {
        let mut model = spring_model(1000.0, None, Dof6Mask(0b111110));
        model.nodes.push(Node {
            id: NodeId(2),
            coord: [5000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: Some([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        });
        let dofmap = DofMap::build(&model);
        // 節点2 は要素・拘束いずれにも参加しないため、support_spring があっても
        // 活性自由度を持たない（既存節点1の1自由度のみ）。
        assert_eq!(dofmap.n_active(), 1);
        let g2 = NodeId(2).index() * DOF_PER_NODE;
        assert!(dofmap.active(g2).is_none());
    }
}
