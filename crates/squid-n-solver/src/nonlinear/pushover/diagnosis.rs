//! 接線剛性が特異（非正定値）になったときの診断メッセージ生成。
//!
//! 増分解析は弾塑性の接線剛性を毎反復分解するため、分解の失敗（`NotPositiveDefinite`）は
//! 「モデルの自由度に剛性が無い（特異）」と「崩壊機構の形成・耐力劣化で正定値性を
//! 失った（不安定）」のどちらでも起こる。どちらであるかは接線剛性の対角成分から
//! 判別できるため、利用者が次の一手を判断できる日本語メッセージへ翻訳する
//! （静解析の [`crate::statics::analysis`] が持つ `singular_diagnosis` と同じ方針だが、
//! 増分解析では接線剛性そのものを見て自由度を名指しできる）。
//!
//! # 走査するのは「拘束縮約後」の接線剛性
//!
//! 実際に分解する行列は拘束縮約後の \\( K_r = T^\top K T \\) であり、対角を見るのも
//! こちらでなければならない。縮約前の自由 DOF 空間 \\( K \\) には、**要素が 1 つも
//! 接続しない節点**が正当に存在するためである。典型は**剛床の代表節点**で、
//! 階自動生成が重心に置く仮想節点は Uz・Rx・Ry だけを数値上のダミー拘束とし、
//! 面内 3 成分（Ux・Uy・Rz）は自由のまま剛床拘束のマスターとして働く。この 3 成分は
//! 縮約前には対角がちょうど 0 で、縮約によってスレーブ側の剛性が集まって初めて
//! 値を持つ。縮約前を走査すると、**階定義のあるモデルでは必ず**代表節点の
//! Ux・Uy・Rz が「剛性がありません」と名指しされ、健全なモデルでも入力不備を
//! 疑わせる誤った案内になる。

use crate::constraint::Reducer;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;

/// 自由度の成分名を返す。`d` は標準の 6 成分（0..6 = Ux..Rz）に加え、
/// 仕口パネルのせん断変形角 γX・γY（6・7）を取りうる。
fn dof_label(d: usize) -> &'static str {
    match d {
        0 => "Ux（X 方向移動）",
        1 => "Uy（Y 方向移動）",
        2 => "Uz（Z 方向移動）",
        3 => "Rx（X 軸まわり回転）",
        4 => "Ry（Y 軸まわり回転）",
        5 => "Rz（Z 軸まわり回転）",
        6 => "γX（仕口パネルのせん断変形角・X'-Z' 面）",
        _ => "γY（仕口パネルのせん断変形角・Y'-Z' 面）",
    }
}

/// 自由度の識別子（節点 ID, 成分 index（0..6 = Ux..Rz、6..8 = 仕口パネルの γX・γY））。
type DofRef = (u32, usize);

/// 接線剛性 `k_red`（拘束縮約後・CSC）の対角成分から、剛性がほぼ 0 の自由度と
/// 負の自由度を集める。戻り値は `(剛性ゼロ, 対角が負)` の並び。
///
/// 縮約後の自由度番号は [`Reducer::free_dof_of`] で自由 DOF 空間へ戻し、さらに
/// [`DofMap::global`] で節点・成分へ翻訳する（縮約前を走査してはいけない理由は
/// モジュールドキュメント参照）。
///
/// 判定しきい値は行列全体の対角最大値に対する相対値（`1e-12`）とする。剛性の絶対値は
/// モデルの単位系・部材寸法で何桁も変わるため、絶対しきい値では判別できない。
fn diagonal_defects(
    dofmap: &DofMap,
    reducer: &Reducer,
    k_red: &faer::sparse::SparseColMat<usize, f64>,
) -> (Vec<DofRef>, Vec<DofRef>) {
    let n = k_red.nrows();
    let col_ptr = k_red.col_ptr();
    let row_idx = k_red.row_idx();
    let values = k_red.val();
    let mut diag = vec![0.0_f64; n];
    for c in 0..n {
        for k in col_ptr[c]..col_ptr[c + 1] {
            if row_idx[k] == c {
                diag[c] += values[k];
            }
        }
    }
    let max_abs = diag.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    let tol = max_abs * 1e-12;
    let mut zero = Vec::new();
    let mut negative = Vec::new();
    for (r, &d) in diag.iter().enumerate() {
        // 縮約空間 → 自由 DOF 空間 → 全体 DOF。
        let Some(active) = reducer.free_dof_of(r) else {
            continue;
        };
        let g = dofmap.global(active as u32);
        // 仕口パネルの追加自由度は標準自由度の後ろに並ぶため、`g / DOF_PER_NODE`
        // では節点番号へ換算できない。パネル自由度は逆写像で節点を引き当てる。
        let entry = if dofmap.is_node_dof(g) {
            ((g / DOF_PER_NODE) as u32, g % DOF_PER_NODE)
        } else {
            match dofmap.panel_dof_ref(g) {
                Some((ni, c)) => (ni as u32, DOF_PER_NODE + c),
                None => continue,
            }
        };
        if d.abs() <= tol {
            zero.push(entry);
        } else if d < 0.0 {
            negative.push(entry);
        }
    }
    (zero, negative)
}

/// 自由度の一覧を「節点 3 の Rx（X 軸まわり回転）」形式で先頭 5 件まで列挙する。
fn format_dofs(list: &[DofRef]) -> String {
    let head: Vec<String> = list
        .iter()
        .take(5)
        .map(|(node, d)| format!("節点 {} の {}", node, dof_label(*d)))
        .collect();
    let more = if list.len() > 5 {
        format!(" 他 {} 件", list.len() - 5)
    } else {
        String::new()
    };
    format!("{}{}", head.join("、"), more)
}

/// 接線剛性の分解に失敗したときの診断メッセージ。
///
/// `phase` は失敗したフェーズ名（「初期接線剛性」「長期荷重の初期載荷」など）で、
/// メッセージ先頭に置く。剛性ゼロの自由度が見つかった場合はそれを名指しし
/// （モデルの入力不備。修正すべき対象が特定できる）、見つからない場合は
/// 崩壊機構・耐力劣化の可能性として案内する。
pub(super) fn tangent_singular_diagnosis(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    k_red: &faer::sparse::SparseColMat<usize, f64>,
    phase: &str,
) -> String {
    let (zero, negative) = diagonal_defects(dofmap, reducer, k_red);
    if !zero.is_empty() {
        return format!(
            "{phase}が特異です。次の自由度に剛性がありません: {}。\
             これらの自由度を拘束するか、剛性を与える部材（直交する梁・柱）を接続してください。\
             梁だけが一直線に並ぶ節点は、ねじれ回転（材軸まわり）に剛性が生じないため\
             特異になります。",
            format_dofs(&zero)
        );
    }
    if !negative.is_empty() {
        return format!(
            "{phase}が非正定値です（剛性の対角が負: {}）。部材の耐力劣化・崩壊機構の形成により\
             構造が不安定になっています。部材耐力・履歴則の設定、または終了目標\
             （目標変位・目標層間変形角）を見直してください。",
            format_dofs(&negative)
        );
    }
    let n_restrained = model.nodes.iter().filter(|n| n.restraint.0 != 0).count();
    format!(
        "{phase}が非正定値です。構造が機構（不安定）になっている可能性があります。\
         考えられる原因: (1) 拘束が不足している（現在 {n_restrained} 節点に拘束あり）、\
         (2) ピン接合が連続し回転が拘束されない部材がある、\
         (3) 部材の降伏が進み崩壊機構が形成された。"
    )
}

/// Newton 反復が収束しなかったときの原因切り分けメッセージ。
///
/// 接線剛性が分解できるかどうかで案内を変える。分解できない（特異・非正定値）なら
/// [`tangent_singular_diagnosis`] の内容を返し、分解できるなら剛性の問題ではなく
/// 反復そのものの非収束として、増分・目標・部材設定の見直しを促す。
/// 「収束しません」と「剛性が特異です」を混ぜて案内すると、実際には健全な剛性の
/// モデルに対して拘束不足を疑わせてしまうため区別する。
pub(super) fn nonconvergence_detail(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    k_red: &faer::sparse::SparseColMat<usize, f64>,
    factorizable: bool,
    phase: &str,
) -> String {
    if !factorizable {
        return tangent_singular_diagnosis(model, dofmap, reducer, k_red, phase);
    }
    format!(
        "{phase}自体は分解できるため、剛性の特異性ではなく Newton 反復の非収束です。\
         ステップ数を増やして増分を細かくする、終了目標（目標変位・目標層間変形角）を\
         小さくする、部材の耐力・履歴則の設定を見直してください。"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
    };
    use squid_n_math::sparse::{assemble_csc, Triplet};

    /// 節点 0 固定・節点 1 自由の 1 部材モデル（active DOF は節点 1 の 6 自由度で、
    /// active index 0..5 が Ux..Rz に対応する）。
    fn two_node_beam_model() -> Model {
        let mk_node = |i: u32, z: f64, restraint: Dof6Mask| Node {
            id: NodeId(i),
            coord: [0.0, 0.0, z],
            restraint,
            mass: None,
            story: None,
            support_spring: None,
        };
        Model {
            nodes: vec![
                mk_node(0, 0.0, Dof6Mask::FIXED),
                mk_node(1, 3000.0, Dof6Mask::FREE),
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: None,
                material: None,
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            ..Default::default()
        }
    }

    /// 剛性ゼロの自由度がある場合は、その自由度を名指しした診断になる。
    #[test]
    fn test_diagnosis_names_zero_stiffness_dof() {
        let model = two_node_beam_model();
        let dofmap = DofMap::build(&model);
        let n = dofmap.n_active();
        // 対角に剛性を入れるが、1 自由度（active 3 = 節点 1 の Rx）だけ 0 のままにする。
        let triplets: Vec<Triplet> = (0..n)
            .filter(|a| *a != 3)
            .map(|a| Triplet {
                row: a,
                col: a,
                val: 1.0e6,
            })
            .collect();
        let k = assemble_csc(n, triplets);
        let reducer = Reducer::build(&model, &dofmap);
        let msg = tangent_singular_diagnosis(&model, &dofmap, &reducer, &k, "初期接線剛性");
        assert!(msg.contains("剛性がありません"), "msg={msg}");
        assert!(msg.contains("Rx"), "msg={msg}");
    }

    /// 対角が負の自由度がある場合は、耐力劣化・機構形成としての診断になる。
    #[test]
    fn test_diagnosis_reports_negative_diagonal() {
        let model = two_node_beam_model();
        let dofmap = DofMap::build(&model);
        let n = dofmap.n_active();
        let triplets: Vec<Triplet> = (0..n)
            .map(|a| Triplet {
                row: a,
                col: a,
                val: if a == 2 { -1.0e6 } else { 1.0e6 },
            })
            .collect();
        let k = assemble_csc(n, triplets);
        let reducer = Reducer::build(&model, &dofmap);
        let msg = tangent_singular_diagnosis(&model, &dofmap, &reducer, &k, "接線剛性");
        assert!(msg.contains("対角が負"), "msg={msg}");
    }

    /// 剛床の代表節点（要素が接続せず、面内 3 成分だけが自由な仮想節点）を
    /// 「剛性がありません」と誤って名指ししない。
    ///
    /// 代表節点の Ux・Uy・Rz は**縮約前**の接線剛性では必ず対角ゼロになるため、
    /// 縮約前を走査すると階定義のあるモデルすべてで偽陽性が出る（モジュール
    /// ドキュメント参照）。縮約後を走査していれば、スレーブの剛性が集約されて
    /// 対角は正になる。
    #[test]
    fn test_diagnosis_ignores_rigid_diaphragm_master() {
        use squid_n_core::ids::StoryId;
        use squid_n_core::model::Constraint;

        let mut model = two_node_beam_model();
        // 代表節点（要素非接続。Uz/Rx/Ry のみダミー拘束＝階自動生成と同じ規則）。
        let mut rep_restraint = Dof6Mask::FREE;
        rep_restraint.set_fixed(squid_n_core::dof::Dof::Uz);
        rep_restraint.set_fixed(squid_n_core::dof::Dof::Rx);
        rep_restraint.set_fixed(squid_n_core::dof::Dof::Ry);
        model.nodes.push(Node {
            id: NodeId(2),
            coord: [0.0, 0.0, 3000.0],
            restraint: rep_restraint,
            mass: None,
            story: None,
            support_spring: None,
        });
        model.constraints.push(Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(2),
            slaves: vec![NodeId(1)],
        });
        model.generated_masters = vec![NodeId(2)];

        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        // 縮約前は「要素が与える剛性」だけ、すなわち節点 1 の 6 自由度のみ対角を持つ。
        let triplets: Vec<Triplet> = (0..dofmap.n_active())
            .filter(|&a| {
                let g = dofmap.global(a as u32);
                g / DOF_PER_NODE == 1
            })
            .map(|a| Triplet {
                row: a,
                col: a,
                val: 1.0e6,
            })
            .collect();
        let k_free = assemble_csc(dofmap.n_active(), triplets);
        let k_red = reducer.reduce_k(&k_free);

        let msg = tangent_singular_diagnosis(&model, &dofmap, &reducer, &k_red, "接線剛性");
        assert!(
            !msg.contains("剛性がありません"),
            "剛床代表節点を剛性ゼロと誤判定している: {msg}"
        );
        assert!(!msg.contains("節点 2"), "代表節点を名指ししている: {msg}");
    }
}
