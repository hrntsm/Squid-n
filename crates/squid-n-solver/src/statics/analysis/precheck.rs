//! 解析前のモデル静的検証と特異行列診断。
//!
//! よくあるモデリングミス（節点・部材・拘束の欠如、断面/材料未割当、孤立節点）を
//! 特異行列エラーの前に検出し、「何をすれば直るか」を含む日本語メッセージで返す。
//!
//! 判定の本体は [`model_issues`] にあり、解析前チェック [`precheck_model`] と
//! UI のモデル整合性チェック（診断タブ）はどちらもこれを呼ぶ。両者が別々に検査を
//! 持つと、片方だけに項目を足したときに「診断は通ったのに解析が止まる」状態が生まれる。
//! **解析を妨げる不備の検査を増やすときは、必ず [`model_issues`] へ足すこと。**

use squid_n_core::ids::{ElemId, MaterialId, NodeId};
use squid_n_core::model::Model;
use squid_n_core::section_shape::SectionShape;
use squid_n_math::solver::SolveError;

/// 不備の対象。診断一覧がクリックで 3D 選択・インスペクタへ結びつけるために持つ。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IssueTargets {
    /// 対象を特定できない不備（節点・部材・拘束が 1 つもない等）。
    Model,
    Members(Vec<ElemId>),
    Nodes(Vec<NodeId>),
}

/// 載荷区間が材長を超えているとみなす余裕 [mm]。
///
/// 全長載荷は `b = L` で入力されるため、丸め誤差で僅かに超えた分は不備としない。
const LOAD_EXTENT_TOL_MM: f64 = 1.0;

/// 不備の重大度。
///
/// 診断（[`model_issues`]）は解析を止める不備と、解析は通るが入力の意図を
/// 確かめたい事柄の両方を返す。判定を 1 か所に持ったまま、解析前チェックが
/// 止めるのは [`IssueSeverity::Error`] だけに限るために区別する。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IssueSeverity {
    /// 解析が成立しない不備。解析前チェック（[`precheck_model`]）はこれで止める。
    Error,
    /// 解析は成立するが、入力の取り違えである可能性が高いもの。
    /// 診断タブには警告として並べ、解析は止めない。
    Warning,
}

/// モデルの不備 1 件。
pub struct ModelIssue {
    /// 解析を止めるか（[`IssueSeverity`]）。
    pub severity: IssueSeverity,
    /// 対象 ID と是正方法を含む、単体で完結する説明文。
    /// 解析前チェックはこれをそのままエラーメッセージにする。
    pub message: String,
    /// 対象 1 件ごとに添える短い説明（例: 「断面が未割当です」）。
    /// 診断タブが対象単位の行に並べるときに使う。
    pub short: String,
    pub targets: IssueTargets,
}

impl ModelIssue {
    /// 対象を特定できない不備。
    fn model(message: impl Into<String>) -> Self {
        let message = message.into();
        Self {
            severity: IssueSeverity::Error,
            short: message.clone(),
            message,
            targets: IssueTargets::Model,
        }
    }

    /// 部材を名指しする不備。`what` と `remedy` から説明文を組み立てる。
    fn members(what: &str, label: &str, ids: Vec<ElemId>, short: &str, remedy: &str) -> Self {
        let raw: Vec<u32> = ids.iter().map(|id| id.0).collect();
        Self {
            severity: IssueSeverity::Error,
            message: id_list_message(what, label, &raw, remedy),
            short: short.to_string(),
            targets: IssueTargets::Members(ids),
        }
    }

    /// 節点を名指しする不備。
    fn nodes(what: &str, label: &str, ids: Vec<NodeId>, short: &str, remedy: &str) -> Self {
        let raw: Vec<u32> = ids.iter().map(|id| id.0).collect();
        Self {
            severity: IssueSeverity::Error,
            message: id_list_message(what, label, &raw, remedy),
            short: short.to_string(),
            targets: IssueTargets::Nodes(ids),
        }
    }

    /// 解析は止めない警告へ落とす。
    fn warn(mut self) -> Self {
        self.severity = IssueSeverity::Warning;
        self
    }
}

/// 「{what}: {label}{id 列}。{remedy}」形式の説明文を組み立てる。
///
/// ID は先頭 5 件までを挙げ、残りは件数へまとめる。大規模モデルで同じ不備が
/// 数百件あってもメッセージが際限なく伸びないようにするため。
fn id_list_message<T: std::fmt::Display>(
    what: &str,
    label: &str,
    ids: &[T],
    remedy: &str,
) -> String {
    const HEAD: usize = 5;
    let head: Vec<String> = ids.iter().take(HEAD).map(|id| id.to_string()).collect();
    let more = if ids.len() > HEAD {
        format!(" 他{}件", ids.len() - HEAD)
    } else {
        String::new()
    };
    format!("{what}: {label}{}{more}。{remedy}", head.join(", "))
}

/// 解析を妨げるモデルの不備をすべて集める。
///
/// 返す順は「モデル検証 → モデル全体の欠落 → 部材の入力不備 → 節点参照の不整合」で、
/// [`precheck_model`] はこの先頭 1 件をエラーにする。
///
/// 先頭のモデル検証（[`Model::validate`]）が失敗したときは、その 1 件だけを返して
/// 打ち切る。検証が見るのは「配列添字 == id」の不変条件と参照整合であり、これが
/// 崩れたモデルでは後続の検査が別実体を指した結果を報告してしまうため、まず
/// データの破損を直してもらう。
pub fn model_issues(model: &Model) -> Vec<ModelIssue> {
    use squid_n_core::model::ElementKind;

    let mut issues = Vec::new();

    // `CoreError` は日本語 UI へ出す前提の Display（"index mismatch: ..." 等）を持つ。
    // `{:?}` にすると "IndexMismatch(\"...\")" と Rust の列挙子表記が露出するため
    // Display で出し、他の不備と同じく是正方法を添える。
    if let Err(e) = model.validate() {
        issues.push(ModelIssue::model(format!(
            "モデル検証エラー: {e}。モデルの ID 参照が壊れています。\
             直前の編集を取り消すか、保存済みのプロジェクトファイルを開き直してください。"
        )));
        return issues;
    }

    if model.nodes.is_empty() {
        issues.push(ModelIssue::model(
            "節点がありません。モデルタブで節点を追加してください。",
        ));
    }
    if model.elements.is_empty() {
        issues.push(ModelIssue::model(
            "部材がありません。モデルタブで部材を追加してください。",
        ));
    }
    if !model.nodes.iter().any(|n| n.restraint.0 != 0) {
        issues.push(ModelIssue::model(
            "拘束(支点)が 1 つもありません。境界条件タブで支点を設定してください\
             (拘束がないと構造全体が剛体移動し、剛性行列が特異になります)。",
        ));
    }

    // 断面・材料が必要な要素（線材・面材）の未割当。対象は
    // `ElementKind::requires_section_and_material`（仕口パネル・節点バネ・免震・
    // ダンパーは断面を持たないのが正常なため除かれる）。
    // 未割当のまま要素構築の既定値（ゼロ剛性）へ落ちて特異行列エラーになるか、
    // かつては「もっともらしい既定断面」で無音に解析が通っていた（危険側）。
    //
    // 断面と材料は別々の不備として挙げる。まとめると診断タブの行が
    // 「断面または材料が未割当です」となり、どちらを直せばよいか伝わらないため。
    let needs_input =
        |e: &&squid_n_core::model::ElementData| e.kind.requires_section_and_material();
    let no_section: Vec<ElemId> = model
        .elements
        .iter()
        .filter(needs_input)
        .filter(|e| e.section.is_none())
        .map(|e| e.id)
        .collect();
    if !no_section.is_empty() {
        issues.push(ModelIssue::members(
            "断面が未割当の部材があります",
            "ID ",
            no_section,
            "断面が未割当です",
            "部材タブで断面を割り当ててください。",
        ));
    }
    // 材料は断面が持つ。断面はあるがその断面に材料がない部材を拾う。
    let no_material: Vec<ElemId> = model
        .elements
        .iter()
        .filter(needs_input)
        .filter(|e| e.section.is_some() && model.element_material(e).is_none())
        .map(|e| e.id)
        .collect();
    if !no_material.is_empty() {
        issues.push(ModelIssue::members(
            "材料が未割当の断面を使う部材があります",
            "ID ",
            no_material,
            "断面に材料が未割当です",
            "断面タブで材料を割り当ててください。",
        ));
    }

    // 配筋を持つ断面は主筋・せん断補強筋の、SRC 断面は内蔵鉄骨の材料が要る。
    // 未割当のまま進むと許容応力度・終局耐力の σy や F 値が決まらないため、
    // その断面を使う部材を名指しして止める（診断タブから 3D 選択できるよう、
    // ほかの不備と同じく部材単位で挙げる）。
    let uses_shape_with =
        |e: &squid_n_core::model::ElementData,
         want: fn(&SectionShape) -> bool,
         slot: fn(&squid_n_core::model::Section) -> Option<MaterialId>| {
            model
                .element_section(e)
                .is_some_and(|s| s.shape.as_ref().is_some_and(want) && slot(s).is_none())
        };
    let has_rebar = |sh: &SectionShape| {
        matches!(
            sh,
            SectionShape::RcRect { .. }
                | SectionShape::RcCircle { .. }
                | SectionShape::SrcRect { .. }
        )
    };
    let is_src = |sh: &SectionShape| matches!(sh, SectionShape::SrcRect { .. });
    let collect_ids =
        |want: fn(&SectionShape) -> bool,
         slot: fn(&squid_n_core::model::Section) -> Option<MaterialId>| {
            model
                .elements
                .iter()
                .filter(needs_input)
                .filter(|e| uses_shape_with(e, want, slot))
                .map(|e| e.id)
                .collect::<Vec<_>>()
        };
    let no_rebar = collect_ids(has_rebar, |s| s.rebar_material);
    if !no_rebar.is_empty() {
        issues.push(ModelIssue::members(
            "主筋の材料が未割当の断面を使う部材があります",
            "ID ",
            no_rebar,
            "断面に主筋の材料が未割当です",
            "断面タブで主筋の材料を割り当ててください。",
        ));
    }
    let no_shear_rebar = collect_ids(has_rebar, |s| s.shear_rebar_material);
    if !no_shear_rebar.is_empty() {
        issues.push(ModelIssue::members(
            "せん断補強筋の材料が未割当の断面を使う部材があります",
            "ID ",
            no_shear_rebar,
            "断面にせん断補強筋の材料が未割当です",
            "断面タブでせん断補強筋の材料を割り当ててください。",
        ));
    }
    let no_steel = collect_ids(is_src, |s| s.steel_material);
    if !no_steel.is_empty() {
        issues.push(ModelIssue::members(
            "内蔵鉄骨の材料が未割当の SRC 断面を使う部材があります",
            "ID ",
            no_steel,
            "断面に内蔵鉄骨の材料が未割当です",
            "断面タブで内蔵鉄骨の材料を割り当ててください。",
        ));
    }

    // シェル要素の断面に板厚がない（線材用断面を割り当てた等）。
    // 要素構築は板厚 0（ゼロ剛性）となり特異行列で止まるが、原因が伝わらないため
    // ここで名指しする。
    let no_thickness: Vec<ElemId> = model
        .elements
        .iter()
        .filter(|e| matches!(e.kind, ElementKind::Shell))
        .filter(|e| {
            e.section
                .and_then(|sid| model.sections.get(sid.index()))
                .is_some_and(|s| s.thickness.is_none())
        })
        .map(|e| e.id)
        .collect();
    if !no_thickness.is_empty() {
        issues.push(ModelIssue::members(
            "シェル要素の断面に板厚が設定されていません",
            "部材 ID ",
            no_thickness,
            "シェル要素の断面に板厚がありません",
            "断面タブで板厚を持つ断面を割り当ててください。",
        ));
    }

    // 線材の有効せん断断面積 As が 0（未入力）
    //
    // As=0 はティモシェンコ梁の φ=0（＝せん断変形なし）となるうえ、せん断降伏の
    // 判定閾値も Qy=+∞（＝せん断では決して降伏しない）となり、入力不足が黙って
    // 「せん断について無限に強い部材」として通ってしまう（危険側）。
    // せん断変形を無視するモデル化は部材（梁）のモデル化として指定すべきことであり、
    // 断面の As を 0 とする形で表現してはならないため、入力エラーとする。
    let zero_shear: Vec<ElemId> = model
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
        .map(|e| e.id)
        .collect();
    if !zero_shear.is_empty() {
        issues.push(ModelIssue::members(
            "有効せん断断面積 As が 0 の断面を使う部材があります",
            "ID ",
            zero_shear,
            "断面の有効せん断断面積 As が 0 です",
            "断面タブで As（Asy・Asz）を設定してください。\
             As=0 はせん断変形が生じず、せん断降伏も判定されない部材となります。",
        ));
    }

    // 耐震壁と周辺架構の構造種別の食い違い
    //
    // 壁エレメントは壁と周辺架構を一体の耐震要素としてモデル化するため、RC 壁に
    // S 骨組（あるいはその逆）を組み合わせた混合構造は耐力式・剛性評価の前提が
    // 成り立たない。一次設計の剛性・断面検定にも効くため、非線形解析だけでなく
    // 全解析の入口で捕捉する。メッセージが壁と相手部材を名指しするため、
    // まとめずに壁 1 枚ごとの不備として挙げる。
    for e in &model.elements {
        if let Some(msg) = squid_n_element::misc_wall::wall_frame_category_issue(e, model) {
            issues.push(ModelIssue {
                severity: IssueSeverity::Error,
                message: msg,
                short: "耐震壁と周辺架構の構造種別が食い違っています".to_string(),
                targets: IssueTargets::Members(vec![e.id]),
            });
        }
    }

    // 節点を共有せずに交差する水平大梁。
    //
    // 床領域は「大梁で囲まれた区画」として面走査で求める（`region_gen`）。この走査は
    // 辺どうしが節点でのみ接する平面グラフを前提とするため、交差する梁があると区画が
    // 実際とずれる。解析そのものは通る（交差点で力は伝わらないというモデル化として
    // 成立する）ため警告に留め、意図した入力かを利用者へ確かめる。
    {
        let crossings = squid_n_core::region_gen::crossing_beams(model);
        if !crossings.is_empty() {
            let mut ids: Vec<squid_n_core::ids::ElemId> = crossings
                .iter()
                .flat_map(|(a, b)| [*a, *b])
                .collect::<std::collections::BTreeSet<_>>()
                .into_iter()
                .collect();
            ids.sort_unstable_by_key(|e| e.0);
            issues.push(
                ModelIssue::members(
                    "節点を共有せずに交差する大梁があります",
                    "部材 ",
                    ids,
                    "交差する大梁（節点を共有していません）",
                    "交点に節点を作って梁を分割してください。                     このままでは床領域（大梁で囲まれた区画）の検出が実際の区画とずれます。",
                )
                .warn(),
            );
        }
    }

    // 床領域に属さない小梁・大梁の区画に載らない浮き床板。
    //
    // 作り直し前の現状の床領域で判定する（診断はモデルを書き換えない）。
    // 解析は成立するため警告に留め、割り当てを確かめてもらう。
    {
        let n = squid_n_core::region_rebuild::unassigned_joist_count(model);
        if n != 0 {
            issues.push(
                ModelIssue::model(format!(
                    "どの床領域にも所属しない小梁が {n} 本あります。\
                     小梁の配置または床領域の境界を確認してください。"
                ))
                .warn(),
            );
        }
        let n = squid_n_core::region_rebuild::floating_slab_count(model);
        if n != 0 {
            issues.push(
                ModelIssue::model(format!(
                    "大梁の区画に載らず割り当てられない床板が {n} 枚あります。\
                     浮き床板になっていないか、境界と大梁を確認してください。"
                ))
                .warn(),
            );
        }
    }

    // 壁版のうち解析要素にしないもの。解析は柱梁だけで成立するため止めないが、
    // 耐震壁が消えたことに気づけるよう警告する（D5: 4 節点・断面ありの
    // Enclosed だけが壁要素になる）。
    {
        use squid_n_core::model::WallPlateShape;
        let mut skipped_non_quad = 0usize;
        let mut skipped_no_section = 0usize;
        for plate in &model.wall_plates {
            if !matches!(plate.shape, WallPlateShape::Enclosed { .. }) {
                continue;
            }
            if plate.section.is_none() {
                skipped_no_section += 1;
                continue;
            }
            if !plate.has_quad_boundary() {
                skipped_non_quad += 1;
            }
        }
        if skipped_non_quad != 0 {
            issues.push(
                ModelIssue::model(format!(
                    "境界が 4 節点でない壁版が {skipped_non_quad} 枚あります。\
                     解析要素としては生成しません（T 字取り付き等）。"
                ))
                .warn(),
            );
        }
        if skipped_no_section != 0 {
            issues.push(
                ModelIssue::model(format!(
                    "断面未割当の壁版が {skipped_no_section} 枚あります。\
                     解析要素としては生成しません。"
                ))
                .warn(),
            );
        }
    }

    // 断面が未割当のスラブ・断面の主材料が未割当のスラブ
    //
    // スラブの板厚と自重は断面から解決する（`Model::slab_self_weight_intensity`）。
    // 断面や主材料が無いと自重が算定できず、床の固定荷重が過小なまま長期応力が
    // 出る（危険側）。既定厚・既定材料で補わず、ここで止める。
    let slab_ids = |f: fn(&Model, &squid_n_core::model::Slab) -> bool| -> Vec<u32> {
        model
            .slabs
            .iter()
            .filter(|s| f(model, s))
            .map(|s| s.id.0)
            .collect()
    };
    let no_slab_section = slab_ids(|m, s| m.slab_section(s).is_none());
    if !no_slab_section.is_empty() {
        issues.push(ModelIssue::model(id_list_message(
            "断面が未割当の床があります",
            "ID ",
            &no_slab_section,
            "床タブで断面を割り当ててください。板厚が定まらないと自重が算定できません。",
        )));
    }
    let no_slab_material = slab_ids(|m, s| {
        m.slab_section(s)
            .is_some_and(|sec| sec.material.is_none() || m.slab_plate_thickness(s).is_none())
    });
    if !no_slab_material.is_empty() {
        issues.push(ModelIssue::model(id_list_message(
            "断面の材料または板厚が定まらない床があります",
            "ID ",
            &no_slab_material,
            "断面タブで床の断面へ材料を割り当て、板厚を持つ形状にしてください。",
        )));
    }

    // 載荷区間が材長を超える部材荷重
    //
    // 載荷位置は i 端からの mm の絶対位置である。材長を超える区間を与えると、
    // 等価節点力の積分（`gauss_dist`）が Hermite 形状関数を材外へ外挿するため、
    // 節点力と固定端内力が黙って誤る。荷重を入れたあとに節点を動かしても作れる
    // 状態なので、入力時ではなく解析前に検査する。
    {
        use squid_n_core::model::MemberLoadKind;
        let mut over: Vec<u32> = Vec::new();
        for lc in &model.load_cases {
            for ml in &lc.member {
                let Some(elem) = model.elements.get(ml.elem.index()) else {
                    continue;
                };
                let l = model.member_length(elem);
                if l <= 0.0 {
                    continue;
                }
                let end = match ml.kind {
                    MemberLoadKind::Point { a, .. } => a,
                    MemberLoadKind::Distributed { b, .. } => b,
                };
                if end > l + LOAD_EXTENT_TOL_MM {
                    over.push(ml.elem.0);
                }
            }
        }
        over.sort_unstable();
        over.dedup();
        if !over.is_empty() {
            issues.push(ModelIssue::model(id_list_message(
                "載荷区間が材長を超える部材荷重があります",
                "部材 #",
                &over,
                "等価節点力の積分が材外へ及び、節点力と固定端内力が誤ります。\
                 荷重タブで載荷位置を材長の内側へ直してください。",
            )));
        }
    }

    // 階名の重複
    //
    // 階名は結果の一覧・CSV の列見出し・断面の識別子（符号＋階）に使われる。
    // 同じ名前の階が 2 つあると、どの行がどの階かを判別できず結果を読み違え、
    // 断面の符号＋階も別々の階を同じ断面として指す。解析自体は `StoryId` で
    // 回るが、結果を正しく読めないモデルでの解析は止める。
    // 見た目で区別できない差（前後の空白）は同名として扱う。
    {
        let mut seen: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
        for s in &model.stories {
            *seen.entry(s.name.trim()).or_insert(0) += 1;
        }
        let mut dup: Vec<String> = seen
            .into_iter()
            .filter(|(_, n)| *n > 1)
            .map(|(name, _)| name.to_string())
            .collect();
        dup.sort();
        if !dup.is_empty() {
            issues.push(ModelIssue::model(id_list_message(
                "階名が重複しています",
                "",
                &dup,
                "階名は結果の一覧・CSV の列見出しと、断面の符号＋階に使われます。\
                 どの階の値なのかを判別できないため、階の定義で名前を分けてください。",
            )));
        }
    }

    // 剛床（ダイアフラム）のない階
    //
    // 剛床がない階の水平力は、階に属する節点へ質量比で直接分配される
    // （`distribute_pi_over_diaphragms`）。解析は成立するため止めないが、
    // 剛床を意図していたのに床が拾えていない・準備計算の再実行で消えた場合に
    // 気づけるよう警告として挙げる。
    let no_diaphragm: Vec<String> = model
        .stories
        .iter()
        .filter(|s| model.diaphragms_of(s.id).next().is_none())
        .map(|s| s.name.clone())
        .collect();
    if !no_diaphragm.is_empty() {
        issues.push(
            ModelIssue::model(id_list_message(
                "剛床のない階があります",
                "",
                &no_diaphragm,
                "水平力はその階の節点へ質量比で直接分配されます。\
                 剛床として扱う階なら、床を張って準備計算を実行し直してください。",
            ))
            .warn(),
        );
    }

    // 基部以外で水平拘束された剛床マスターへ地震力が載ると、拘束自由度へ入って
    // 無言で消える（危険側）。地震用重量が正の階だけをエラーにする。
    {
        use squid_n_core::dof::Dof;

        for story in model.stories.iter().skip(1) {
            let diaphragms: Vec<_> = model.diaphragms_of(story.id).collect();
            let single = diaphragms.len() == 1;
            for dia in diaphragms {
                let Some(master) = model.nodes.get(dia.master.index()) else {
                    continue;
                };
                let horiz_restrained =
                    master.restraint.is_fixed(Dof::Ux) || master.restraint.is_fixed(Dof::Uy);
                if !horiz_restrained {
                    continue;
                }
                let weight = diaphragm_seismic_weight(story, &dia, single);
                if weight <= 0.0 {
                    continue;
                }
                issues.push(ModelIssue::nodes(
                    &format!(
                        "{}の剛床マスターが水平拘束されているため、その階の地震力が解析に載りません",
                        story.name
                    ),
                    "節点 ID ",
                    vec![dia.master],
                    "剛床マスターが水平拘束されています",
                    "床面に構造部材（柱・大梁）が取り付くか、剛床の設定を見直してください。",
                ));
            }
        }
    }

    issues.extend(node_reference_issues(model));
    issues
}

/// 剛床が負担する地震用重量 [N]。
///
/// `RigidDiaphragm::weight` が `Some` ならその値。`None` で階に単一剛床なら
/// 層重量（[`squid_n_core::model::Story::seismic_weight`]）全量。多剛床で未算定は 0。
fn diaphragm_seismic_weight(
    story: &squid_n_core::model::Story,
    dia: &squid_n_core::model::DiaphragmRef<'_>,
    single_on_story: bool,
) -> f64 {
    if let Some(w) = dia.weight {
        return w;
    }
    if single_on_story {
        story.seismic_weight.unwrap_or(0.0)
    } else {
        0.0
    }
}

/// 節点参照の不整合（ダングリング参照・孤立節点）を集める。
fn node_reference_issues(model: &Model) -> Vec<ModelIssue> {
    // 孤立節点（要素・拘束・剛床から参照されず、完全固定でもない）
    // → 剛性ゼロの自由 DOF となり特異行列の典型原因
    //
    // 参照のマークは範囲チェック付きで行い、存在しない節点への参照
    // （ダングリング NodeId。編集・インポート層の不整合で混入し得る）は
    // `dangling` に収集して明示エラーにする（直接添字では panic するため）。
    let mut issues = Vec::new();
    let mut referenced = vec![false; model.nodes.len()];
    let mut dangling: Vec<NodeId> = Vec::new();
    {
        let mut mark = |n: NodeId| match referenced.get_mut(n.index()) {
            Some(slot) => *slot = true,
            None => dangling.push(n),
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
        // 床（スラブ境界・小梁支持点）・二次部材（小梁・間柱）が参照する節点は、
        // 要素が接続しなくても意図的な幾何節点（荷重伝達点）なので孤立扱いしない。
        // これらは `DofMap::build` が解析自由度から自動的に除外するため、
        // 零剛性の自由度にはならない。
        for region in &model.floor_regions {
            for n in &region.boundary {
                mark(*n);
            }
            for j in region.joist_lines() {
                for n in &j.support {
                    mark(*n);
                }
            }
        }
        for slab in &model.slabs {
            match &slab.shape {
                squid_n_core::model::SlabShape::Enclosed { boundary } => {
                    for n in boundary {
                        mark(*n);
                    }
                }
                squid_n_core::model::SlabShape::Attached { anchor, .. } => match anchor {
                    squid_n_core::model::RegionAnchor::Line { nodes, .. } => {
                        for n in nodes {
                            mark(*n);
                        }
                    }
                    squid_n_core::model::RegionAnchor::Point(n) => mark(*n),
                    // 床板では到達しない（`slab.rs::boundary_coords` と同じ理由）。
                    squid_n_core::model::RegionAnchor::FloorRegion { .. } => {}
                },
            }
        }
        for sm in &model.secondary_members {
            for n in &sm.nodes {
                mark(*n);
            }
        }
        // 壁版の境界・取付き先も、要素が無くても意図した幾何節点なので孤立扱いしない。
        for plate in &model.wall_plates {
            match &plate.shape {
                squid_n_core::model::WallPlateShape::Enclosed { boundary } => {
                    for n in boundary {
                        mark(*n);
                    }
                }
                squid_n_core::model::WallPlateShape::Attached { anchor, .. } => match anchor {
                    squid_n_core::model::RegionAnchor::Line { nodes, .. } => {
                        for n in nodes {
                            mark(*n);
                        }
                    }
                    squid_n_core::model::RegionAnchor::Point(n) => mark(*n),
                    squid_n_core::model::RegionAnchor::FloorRegion { nodes, .. } => {
                        for n in nodes {
                            mark(*n);
                        }
                    }
                },
            }
        }
    }
    if !dangling.is_empty() {
        dangling.sort_unstable_by_key(|n| n.0);
        dangling.dedup();
        issues.push(ModelIssue::nodes(
            "存在しない節点への参照があります",
            "節点 ID ",
            dangling,
            "存在しない節点を参照しています",
            "部材・拘束・剛床・床の節点参照を確認してください\
             (節点削除後の不整合の可能性があります)。",
        ));
    }
    // 部材が 1 つもないモデルでは全節点が孤立になる。「部材がありません」で
    // 同じことを言っているため、節点を 1 つずつ挙げても情報が増えない。
    if model.elements.is_empty() {
        return issues;
    }
    // `referenced` は添字で引くため、`Model::validate` の不変条件（id == 添字）が
    // 崩れているモデルでは引けない。診断は検証エラーのあるモデルでも動くため、
    // 引けない節点は孤立と決めつけず対象外にする。
    let isolated: Vec<NodeId> = model
        .nodes
        .iter()
        .filter(|n| {
            !referenced.get(n.id.index()).copied().unwrap_or(true)
                && n.restraint != squid_n_core::dof::Dof6Mask::FIXED
        })
        .map(|n| n.id)
        .collect();
    if !isolated.is_empty() {
        issues.push(ModelIssue::nodes(
            "どの部材にも接続されていない節点があります",
            "ID ",
            isolated,
            "どの部材にも接続されていません",
            "削除するか完全固定にしてください(剛性ゼロの自由度は解析できません)。",
        ));
    }
    issues
}

/// 解析前のモデル静的検証。よくあるモデリングミスを特異行列エラーの前に検出し、
/// 「何をすれば直るか」を含むメッセージで返す。
pub(super) fn precheck_model(model: &Model) -> Result<(), SolveError> {
    // 止めるのは解析が成立しない不備だけとする（警告は診断タブへ出す）。
    match model_issues(model)
        .into_iter()
        .find(|i| i.severity == IssueSeverity::Error)
    {
        Some(issue) => Err(SolveError::InvalidInput(issue.message)),
        None => Ok(()),
    }
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
