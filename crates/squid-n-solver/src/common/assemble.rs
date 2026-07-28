use faer::sparse::SparseColMat;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_element::factory::build_behavior;
use squid_n_element::transform::LocalFrame;
use squid_n_math::sparse::{assemble_csc, Triplet};

pub fn assemble_global_k(model: &Model, dofmap: &DofMap) -> SparseColMat<usize, f64> {
    let ctx = squid_n_element::behavior::Ctx { model };
    let mut all_triplets = Vec::new();

    for elem in &model.elements {
        let (behavior, state) = build_behavior(elem, model);
        let gdofs = behavior.global_dofs(dofmap);
        let k_local = behavior.tangent_stiffness(&state, &ctx);
        let triplets = k_local.to_triplets(&gdofs);
        all_triplets.extend(triplets);
    }

    assemble_csc(dofmap.n_active(), all_triplets)
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
            let (behavior, _state) = build_behavior(elem, model);
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
