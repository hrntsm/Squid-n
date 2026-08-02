//! 解析前のモデル静的検証と特異行列診断。
//!
//! よくあるモデリングミス（節点・部材・拘束の欠如、断面/材料未割当、孤立節点）を
//! 特異行列エラーの前に検出し、「何をすれば直るか」を含む日本語メッセージで返す。

use squid_n_core::model::Model;
use squid_n_math::solver::SolveError;

/// 解析前のモデル静的検証。よくあるモデリングミスを特異行列エラーの前に検出し、
/// 「何をすれば直るか」を含むメッセージで返す。
pub(super) fn precheck_model(model: &Model) -> Result<(), SolveError> {
    use squid_n_core::model::ElementKind;

    if model.nodes.is_empty() {
        return Err(SolveError::InvalidInput(
            "節点がありません。モデルタブで節点を追加してください。".into(),
        ));
    }
    if model.elements.is_empty() {
        return Err(SolveError::InvalidInput(
            "部材がありません。モデルタブで部材を追加してください。".into(),
        ));
    }
    if !model.nodes.iter().any(|n| n.restraint.0 != 0) {
        return Err(SolveError::InvalidInput(
            "拘束(支点)が 1 つもありません。境界条件タブで支点を設定してください。".into(),
        ));
    }

    // 梁要素の断面・材料未割当
    let missing: Vec<u32> = model
        .elements
        .iter()
        .filter(|e| {
            matches!(e.kind, ElementKind::Beam) && (e.section.is_none() || e.material.is_none())
        })
        .map(|e| e.id.0)
        .collect();
    if !missing.is_empty() {
        let head: Vec<String> = missing.iter().take(5).map(|id| id.to_string()).collect();
        let more = if missing.len() > 5 {
            format!(" 他{}件", missing.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "断面または材料が未割当の部材があります: ID {}{}。部材タブで割り当ててください。",
            head.join(", "),
            more
        )));
    }

    // 線材の有効せん断断面積 As が 0（未入力）
    //
    // As=0 はティモシェンコ梁の φ=0（＝せん断変形なし）となるうえ、せん断降伏の
    // 判定閾値も Qy=+∞（＝せん断では決して降伏しない）となり、入力不足が黙って
    // 「せん断について無限に強い部材」として通ってしまう（危険側）。
    // せん断変形を無視するモデル化は部材（梁）のモデル化として指定すべきことであり、
    // 断面の As を 0 とする形で表現してはならないため、入力エラーとする。
    let zero_shear: Vec<u32> = model
        .elements
        .iter()
        .filter(|e| {
            matches!(
                e.kind,
                ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring
            )
        })
        .filter(|e| {
            e.section
                .and_then(|sid| model.sections.get(sid.index()))
                .is_some_and(|s| s.as_y <= 0.0 || s.as_z <= 0.0)
        })
        .map(|e| e.id.0)
        .collect();
    if !zero_shear.is_empty() {
        let head: Vec<String> = zero_shear.iter().take(5).map(|id| id.to_string()).collect();
        let more = if zero_shear.len() > 5 {
            format!(" 他{}件", zero_shear.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "有効せん断断面積 As が 0 の断面を使う部材があります: ID {}{}。\
             断面タブで As（Asy・Asz）を設定してください。\
             As=0 はせん断変形が生じず、せん断降伏も判定されない部材となります。",
            head.join(", "),
            more
        )));
    }

    // 耐震壁と周辺架構の構造種別の食い違い
    //
    // 壁エレメントは壁と周辺架構を一体の耐震要素としてモデル化するため、RC 壁に
    // S 骨組（あるいはその逆）を組み合わせた混合構造は耐力式・剛性評価の前提が
    // 成り立たない。一次設計の剛性・断面検定にも効くため、非線形解析だけでなく
    // 全解析の入口で捕捉する。
    if let Some(msg) = model
        .elements
        .iter()
        .find_map(|e| squid_n_element::misc_wall::wall_frame_category_issue(e, model))
    {
        return Err(SolveError::InvalidInput(msg));
    }

    // 孤立節点（要素・拘束・剛床から参照されず、完全固定でもない）
    // → 剛性ゼロの自由 DOF となり特異行列の典型原因
    //
    // 参照のマークは範囲チェック付きで行い、存在しない節点への参照
    // （ダングリング NodeId。編集・インポート層の不整合で混入し得る）は
    // `dangling` に収集して明示エラーにする（従来は直接添字で panic していた）。
    let mut referenced = vec![false; model.nodes.len()];
    let mut dangling: Vec<u32> = Vec::new();
    {
        let mut mark = |n: squid_n_core::ids::NodeId| match referenced.get_mut(n.index()) {
            Some(slot) => *slot = true,
            None => dangling.push(n.0),
        };
        for e in &model.elements {
            for n in &e.nodes {
                mark(*n);
            }
        }
        for c in &model.constraints {
            use squid_n_core::model::Constraint;
            match c {
                Constraint::RigidDiaphragm { master, slaves, .. }
                | Constraint::RigidLink { master, slaves, .. } => {
                    mark(*master);
                    for s in slaves {
                        mark(*s);
                    }
                }
                Constraint::Mpc { master, terms } => {
                    mark(*master);
                    for (n, _, _) in terms {
                        mark(*n);
                    }
                }
            }
        }
        for story in &model.stories {
            for d in &story.diaphragms {
                mark(d.master);
                for s in &d.slaves {
                    mark(*s);
                }
            }
        }
        // 床（スラブ境界・小梁支持点）・二次部材（小梁・間柱）が参照する節点は、
        // 要素が接続しなくても意図的な幾何節点（荷重伝達点）なので孤立扱いしない。
        // これらは `DofMap::build` が解析自由度から自動的に除外するため、
        // 零剛性の自由度にはならない。
        for slab in &model.slabs {
            for n in &slab.boundary {
                mark(*n);
            }
            for j in &slab.joists {
                for n in &j.support {
                    mark(*n);
                }
            }
        }
        for sm in &model.secondary_members {
            for n in &sm.nodes {
                mark(*n);
            }
        }
    }
    if !dangling.is_empty() {
        dangling.sort_unstable();
        dangling.dedup();
        let head: Vec<String> = dangling.iter().take(5).map(|id| id.to_string()).collect();
        let more = if dangling.len() > 5 {
            format!(" 他{}件", dangling.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "存在しない節点への参照があります: 節点 ID {}{}。部材・拘束・剛床・床の\
             節点参照を確認してください(節点削除後の不整合の可能性があります)。",
            head.join(", "),
            more
        )));
    }
    let isolated: Vec<u32> = model
        .nodes
        .iter()
        .filter(|n| !referenced[n.id.index()] && n.restraint != squid_n_core::dof::Dof6Mask::FIXED)
        .map(|n| n.id.0)
        .collect();
    if !isolated.is_empty() {
        let head: Vec<String> = isolated.iter().take(5).map(|id| id.to_string()).collect();
        let more = if isolated.len() > 5 {
            format!(" 他{}件", isolated.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "どの部材にも接続されていない節点があります: ID {}{}。削除するか完全固定にしてください(剛性ゼロの自由度は解析できません)。",
            head.join(", "),
            more
        )));
    }

    Ok(())
}

/// 剛性行列の分解に失敗した（特異・非正定値）ときの診断メッセージ。
pub(super) fn singular_diagnosis(model: &Model) -> String {
    let n_restrained = model.nodes.iter().filter(|n| n.restraint.0 != 0).count();
    format!(
        "剛性行列が特異(非正定値)です。構造が機構(不安定)になっている可能性があります。\
         考えられる原因: (1) 拘束が不足している(現在 {} 節点に拘束あり)、\
         (2) ピン接合が連続し回転が拘束されない部材がある、\
         (3) 断面性能(A・I)が 0 の断面がある。",
        n_restrained
    )
}
