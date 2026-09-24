//! 解析前のモデル静的検証と特異行列診断。
//!
//! よくあるモデリングミス（節点・部材・拘束の欠如、断面/材料未割当、孤立節点）を
//! 特異行列エラーの前に検出し、「何をすれば直るか」を含む日本語メッセージで返す。
//!
//! 判定の本体は [`model_issues`] にあり、解析前チェック [`precheck_model`] と
//! UI のモデル整合性チェック（診断タブ）はどちらもこれを呼ぶ。

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
/// ID は先頭 5 件までを挙げ、残りは件数へまとめる。
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

/// 支持部材を利用者向けの短い識別子へ整える。主架構は要素 ID、二次部材は安定 ID。
fn support_label(support: squid_n_core::model::SupportMemberId) -> String {
    match support {
        squid_n_core::model::SupportMemberId::Primary(elem) => format!("主架構 部材 {}", elem.0),
        squid_n_core::model::SupportMemberId::Secondary(id) => format!("二次部材 {}", id.0),
    }
}

/// 解析を妨げるモデルの不備をすべて集める。
///
/// 返す順は「モデル検証 → モデル全体の欠落 → 部材の入力不備 → 節点参照の不整合」で、
/// [`precheck_model`] はこの先頭 1 件をエラーにする。
///
/// 先頭のモデル検証（[`Model::validate`]）が失敗したときは、その 1 件だけを返して
/// 打ち切る。
pub fn model_issues(model: &Model) -> Vec<ModelIssue> {
    use squid_n_core::model::ElementKind;

    let mut issues = Vec::new();

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
                | SectionShape::RcBeamRect { .. }
                | SectionShape::RcColumnRect { .. }
                | SectionShape::RcColumnCircle { .. }
                | SectionShape::SrcBeamRect { .. }
                | SectionShape::SrcColumnRect { .. }
        )
    };
    let is_src = |sh: &SectionShape| {
        matches!(
            sh,
            SectionShape::SrcRect { .. }
                | SectionShape::SrcBeamRect { .. }
                | SectionShape::SrcColumnRect { .. }
        )
    };
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

    {
        let mut beam_shape_on_column: Vec<ElemId> = Vec::new();
        let mut column_shape_on_beam: Vec<ElemId> = Vec::new();
        for e in model
            .elements
            .iter()
            .filter(|e| e.kind.requires_section_and_material())
        {
            let Some(shape) = model.element_section(e).and_then(|s| s.shape.as_ref()) else {
                continue;
            };
            let (Some(n0), Some(n1)) = (e.nodes.first(), e.nodes.get(1)) else {
                continue;
            };
            let (Some(n0), Some(n1)) = (model.nodes.get(n0.index()), model.nodes.get(n1.index()))
            else {
                continue;
            };
            let vertical = squid_n_core::geom::is_vertical_axis(n0.coord, n1.coord);
            match shape {
                SectionShape::RcBeamRect { .. } | SectionShape::SrcBeamRect { .. } if vertical => {
                    beam_shape_on_column.push(e.id)
                }
                SectionShape::RcColumnRect { .. }
                | SectionShape::RcColumnCircle { .. }
                | SectionShape::SrcColumnRect { .. }
                    if !vertical =>
                {
                    column_shape_on_beam.push(e.id)
                }
                _ => {}
            }
        }
        if !beam_shape_on_column.is_empty() {
            issues.push(ModelIssue::members(
                "梁用断面を柱部材に割り当てています",
                "ID ",
                beam_shape_on_column,
                "梁用断面が柱部材に割り当てられています",
                "断面タブで柱用断面を割り当てるか、部材の用途を確認してください。",
            ));
        }
        if !column_shape_on_beam.is_empty() {
            issues.push(ModelIssue::members(
                "柱用断面を梁部材に割り当てています",
                "ID ",
                column_shape_on_beam,
                "柱用断面が梁部材に割り当てられています",
                "断面タブで梁用断面を割り当てるか、部材の用途を確認してください。",
            ));
        }
    }

    {
        let mut invalid_rebar: Vec<(ElemId, u32, String, String)> = Vec::new();
        for e in model
            .elements
            .iter()
            .filter(|e| e.kind.requires_section_and_material())
        {
            let Some(sec) = model.element_section(e) else {
                continue;
            };
            let Some(shape) = sec.shape.as_ref() else {
                continue;
            };
            if let Err(err) = shape.validate_rebar() {
                invalid_rebar.push((e.id, sec.id.0, sec.name.clone(), err.to_string()));
            }
        }
        for (id, section_id, section_name, reason) in invalid_rebar {
            issues.push(ModelIssue {
                severity: IssueSeverity::Error,
                message: format!(
                    "実配筋の幾何が不整合な断面を使う部材 ID {} があります\
                     （断面 ID {section_id}({section_name}) の配筋: {reason}）。\
                     断面タブで段別本数・かぶり・断面寸法を見直してください。",
                    id.0
                ),
                short: "断面の実配筋の幾何が不整合です".into(),
                targets: IssueTargets::Members(vec![id]),
            });
        }
    }

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

    let composite_fallback: Vec<ElemId> = model
        .elements
        .iter()
        .filter(|e| {
            matches!(
                e.kind,
                ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring
            )
        })
        .filter(|e| {
            model
                .element_section(e)
                .and_then(|s| s.shape.as_ref())
                .and_then(squid_n_core::structure_kind::shape_composite_kind)
                .is_some()
        })
        .filter(|e| model.element_material(e).is_some())
        .filter(|e| squid_n_element::frame::beam::composite_props_of(model, e).is_none())
        .map(|e| e.id)
        .collect();
    if !composite_fallback.is_empty() {
        issues.push(
            ModelIssue::members(
                "材料由来の等価断面性能を算定できない SRC/CFT 断面を使う部材があります",
                "ID ",
                composite_fallback,
                "等価断面性能を算定できません",
                "断面タブで主材料のコンクリート Fc とヤング係数を設定してください。\
                 CFT では鋼管の板厚・外径（充填部の内法が正の値か）も確認してください。\
                 未設定・不成立の間は、SRC は N_S_EQ=15、CFT は鋼管のみで剛性を評価します。",
            )
            .warn(),
        );
    }

    let side_edges = squid_n_element::wall::side_column::SideColumnEdges::build(model);
    let undefined_side_shapes = model
        .elements
        .iter()
        .filter(|e| {
            side_edges.release_axis(e, model).is_some()
                && model.element_section(e).is_some_and(|s| s.shape.is_none())
        })
        .map(|e| e.id)
        .collect::<Vec<_>>();
    if !undefined_side_shapes.is_empty() {
        issues.push(ModelIssue::members(
            "耐震壁の側柱に断面形状が未定義の部材があります",
            "ID ", undefined_side_shapes,
            "側柱の断面形状が未定義です",
            "断面タブで側柱の断面形状を指定してください。断面積・幅・せいだけでは壁との重複領域を確定できません。",
        ));
    }

    for e in &model.elements {
        if e.kind == squid_n_core::model::ElementKind::Wall {
            if let Err(message) =
                squid_n_element::wall::shear_section::wall_shear_rigidity(e, model)
            {
                issues.push(ModelIssue {
                    severity: IssueSeverity::Error,
                    message,
                    short: "壁のせん断断面を算定できません".into(),
                    targets: IssueTargets::Members(vec![e.id]),
                });
            }
        }
        if let Some(msg) = squid_n_element::wall::misc_wall::wall_frame_category_issue(e, model) {
            issues.push(ModelIssue {
                severity: IssueSeverity::Error,
                message: msg,
                short: "耐震壁と周辺架構の構造種別が食い違っています".to_string(),
                targets: IssueTargets::Members(vec![e.id]),
            });
        }
    }

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
                    "交点に節点を作って梁を分割してください。\
                     このままでは床領域の検出が実際の区画とずれます。",
                )
                .warn(),
            );
        }
    }

    {
        let overlaps = model.same_kind_support_overlaps();
        if !overlaps.is_empty() {
            const HEAD: usize = 5;
            let listed = overlaps
                .iter()
                .take(HEAD)
                .map(|overlap| {
                    format!(
                        "{} と {}（位置 [{:.0}, {:.0}, {:.0}] mm）",
                        support_label(overlap.first),
                        support_label(overlap.second),
                        overlap.midpoint[0],
                        overlap.midpoint[1],
                        overlap.midpoint[2]
                    )
                })
                .collect::<Vec<_>>()
                .join("、");
            let more = if overlaps.len() > HEAD {
                format!(" 他{}件", overlaps.len() - HEAD)
            } else {
                String::new()
            };
            issues.push(ModelIssue::model(format!(
                "同じ位置に重なった同種の支持部材があります（{listed}{more}）。\
                 同じ材軸上で大梁・柱どうし、または小梁・間柱どうしを重ねて配置すると、\
                 どちらが荷重を受けるか一意に決まりません。重複した部材を削除するか、\
                 材軸を分けて配置してください。"
            )));
        }
    }

    {
        let n = squid_n_core::region_rebuild::unassigned_joist_count(model);
        if n != 0 {
            issues.push(ModelIssue::model(format!(
                "どの床領域にも所属しない小梁が {n} 本あります。\
                 小梁の配置または床領域の境界を確認してください。"
            )));
        }
        let n = squid_n_core::wall_region_rebuild::unassigned_post_count(model);
        if n != 0 {
            issues.push(ModelIssue::model(format!(
                "どの壁領域にも所属しない間柱が {n} 本あります。\
                 間柱の配置または壁領域の境界を確認してください。"
            )));
        }
        let gaps = squid_n_load::floor::secondary_joist_distribution_gaps(model);
        if gaps.missing_expected_slabs != 0 {
            issues.push(
                ModelIssue::model(format!(
                    "内法の二次部材小梁で、期待する床板の分配が載っていないものが\
                     {} 本あります。片側の床板が欠けると需要を過小評価するため、\
                     断面検定しません。",
                    gaps.missing_expected_slabs
                ))
                .warn(),
            );
        }
        if gaps.short_cover != 0 || gaps.no_distribution != 0 {
            let n = gaps.short_cover + gaps.no_distribution;
            issues.push(
                ModelIssue::model(format!(
                    "床板分配から荷重が得られない、または載荷区間がスパンの半分未満の\
                     二次部材小梁が {n} 本あります。段差床・傾斜小梁・床板境界外では\
                     断面検定しません。"
                ))
                .warn(),
            );
        }
        {
            use squid_n_core::model::LoadPurpose;
            let w_of =
                |sl: &squid_n_core::model::Slab| model.slab_intensity(sl, LoadPurpose::Joist);
            let transfer = squid_n_load::cascade::solve(model, w_of, true);
            if !transfer.invalid_end_shares.is_empty() {
                issues.push(ModelIssue::model(format!(
                    "鉛直な二次部材の端部負担率が未指定または不正です（{} 本）。間柱の両端への負担率を非負・合計100%で指定し、自由端の負担率は0%にしてください。",
                    transfer.invalid_end_shares.len()
                )));
            }
            if !transfer.unresolved.is_empty() {
                issues.push(ModelIssue::model(format!(
                    "端部がどの主架構にも二次部材にも載っていない二次部材が {} 本あります。\
                         受け持った荷重の行き先がなく、解析へ渡りません。端部を大梁の材軸上、\
                         または受け側となる二次部材の内法・自由端へ載せてください\
                         （自由端に載せる場合は端部の節点を共有してください）。",
                    transfer.unresolved.len()
                )));
            }
            fn same_secondary(
                a: &squid_n_core::model::SecondaryMember,
                b: &squid_n_core::model::SecondaryMember,
            ) -> bool {
                a.id == b.id
            }
            let points_equal = |a: [f64; 3], b: [f64; 3]| {
                let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
                d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
                    <= squid_n_core::geom::MEMBER_AXIS_TOL_MM
                        * squid_n_core::geom::MEMBER_AXIS_TOL_MM
            };
            let node_at = |p: [f64; 3]| {
                model
                    .nodes
                    .iter()
                    .find(|n| points_equal(n.coord, p))
                    .map(|n| n.id)
            };

            let detached = model
                .joists()
                .chain(model.posts())
                .filter(|sm| sm.is_detached())
                .count();
            if detached != 0 {
                issues.push(ModelIssue::model(format!(
                    "支持部材アンカーへ解決できない二次部材が {detached} 本あります。\
                     端点座標で重量・材軸長は保持していますが、受け持った荷重の行き先が決まりません。\
                     端部を大梁の材軸上、または受け側となる二次部材の内法・自由端へ載せてください。"
                )));
            }
            let free_on_support = model
                .joists()
                .chain(model.posts())
                .filter(|sm| {
                    !model.secondary_member_materialized(sm)
                        && sm.is_cantilever()
                        && model
                            .secondary_member_end_points(sm)
                            .is_some_and(|(_, free)| {
                                node_at(free)
                                    .is_some_and(|n| model.node_has_geometric_support(n, sm))
                            })
                })
                .count();
            if free_on_support != 0 {
                issues.push(
                    ModelIssue::model(format!(
                        "幾何的には支持がある端を自由端として扱う二次部材が {free_on_support} 本あります。\
                         支持として扱う場合は端部支持条件を確認してください。"
                    ))
                    .warn(),
                );
            }
            let free_end_tie = model
                .joists()
                .chain(model.posts())
                .filter(|sm| {
                    !model.secondary_member_materialized(sm)
                        && sm.is_cantilever()
                        && model
                            .secondary_member_end_points(sm)
                            .is_some_and(|(_, free)| {
                                model.joists().chain(model.posts()).any(|other| {
                                    !same_secondary(other, sm)
                                        && other.is_cantilever()
                                        && model.secondary_member_end_points(other).is_some_and(
                                            |(_, other_free)| points_equal(other_free, free),
                                        )
                                })
                            })
                })
                .count();
            if free_end_tie != 0 {
                issues.push(
                    ModelIssue::model(format!(
                        "自由端どうしが同じ節点に接する二次部材が {free_end_tie} 本あります。\
                         どちらかを受け側（支持）として扱ってください。"
                    ))
                    .warn(),
                );
            }
            let mut partial_edges = 0usize;
            for slab in &model.slabs {
                let squid_n_core::model::SlabShape::Attached {
                    anchor:
                        squid_n_core::model::RegionAnchor::Line {
                            transfer: squid_n_core::model::LoadTransfer::Anchor,
                            ..
                        },
                    ..
                } = &slab.shape
                else {
                    continue;
                };
                let Some(coords) = slab.boundary_coords(model) else {
                    continue;
                };
                for k in 1..coords.len() {
                    let p0 = coords[k];
                    let p1 = coords[(k + 1) % coords.len()];
                    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
                    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                    if len <= 1e-9 {
                        continue;
                    }
                    let cover = squid_n_load::secondary::beams_along_segment(
                        model,
                        p0,
                        p1,
                        squid_n_core::geom::MEMBER_AXIS_TOL_MM,
                    );
                    if cover.is_empty() {
                        continue;
                    }
                    if !squid_n_load::secondary::coverage_covers_full(
                        &cover,
                        len,
                        squid_n_core::geom::MEMBER_AXIS_TOL_MM,
                    ) {
                        partial_edges += 1;
                    }
                }
            }
            if partial_edges != 0 {
                issues.push(
                    ModelIssue::model(format!(
                        "取り付く床板の辺を全長で覆わない実部材が {partial_edges} 辺あります。\
                         覆われていない部分の荷重は他の支持辺へ回るため、その実部材には載りません。\
                         辺の全長を覆うように実部材を配置してください。"
                    ))
                    .warn(),
                );
            }
            if !transfer.cyclic.is_empty() {
                issues.push(ModelIssue::model(format!(
                    "二次部材どうしの支持関係が一巡しています（{} 本）。互いに載せ合う形は\
                         荷重を流せません。受け側を 1 本の通し部材としてモデル化してください。",
                    transfer.cyclic.len()
                )));
            }
            let crossings = squid_n_load::cascade::secondary_crossings(model);
            if !crossings.is_empty() {
                issues.push(ModelIssue::model(format!(
                    "節点を共有せずに交差している二次部材が {} 組あります。交点に接合がなく、\
                         受け側と架け側を決められません。受け側を通し、架け側を交点で分けて\
                         モデル化してください。",
                    crossings.len()
                )));
            }
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

    {
        let skipped_no_section = model
            .wall_plates
            .iter()
            .filter(|p| p.section.is_none())
            .count();
        if skipped_no_section != 0 {
            issues.push(
                ModelIssue::model(format!(
                    "断面未割当の壁版が {skipped_no_section} 枚あります。\
                     板厚と材料が決まらないため自重を算定できず、解析要素にも\
                     しません。"
                ))
                .warn(),
            );
        }
        let ignored_slit = model
            .wall_plates
            .iter()
            .filter(|p| {
                p.slit.any()
                    && !squid_n_load::wall_plate_load::slit_specification_is_reflected(model, p)
            })
            .count();
        if ignored_slit != 0 {
            issues.push(
                ModelIssue::model(format!(
                    "耐震スリットの指定が効かない壁版が {ignored_slit} 枚あります。\
                     スリットは境界が 4 節点の囲まれた壁版でのみ扱えます\
                     （境界が 4 節点でない、または境界頂点のモデル節点を引けない等で\
                     辺が柱際か梁際かを決められないため）。"
                ))
                .warn(),
            );
        }

        let both_beam_slit: Vec<_> = model
            .wall_plates
            .iter()
            .filter(|p| p.has_quad_boundary(model) && p.slit.both_beam_faces())
            .map(|p| p.id.0)
            .collect();
        if !both_beam_slit.is_empty() {
            let ids = both_beam_slit
                .iter()
                .map(|id| id.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            issues.push(ModelIssue::model(format!(
                "上下の梁際がともに切れた壁版があります（壁版 {ids}）。\
                 対象は境界が 4 節点の囲まれた壁版です。\
                 上下の梁際をともに切った納まりは想定していないため、\
                 自重の支持先の指定によらず入力エラーとします。\
                 いずれかの梁際のスリットを外してください。"
            )));
        }

        let stranded = squid_n_load::wall_plate_load::wall_plates_without_load_path(model);
        if !stranded.is_empty() {
            let ids = stranded
                .iter()
                .map(|id| id.0.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            issues.push(ModelIssue::model(format!(
                "自重の行き先が決まらない壁版があります（壁版 {ids}）。\
                 境界辺への自重負担率が未指定・不正、正の負担率の辺がスリットで縁切り、または壁版割当領域の境界が無く支持部材を解決できないためです。\
                 壁版の自重負担率（合計100%）、境界、支持部材を確認してください。"
            )));
        }
        let uncovered: Vec<u32> = model
            .wall_plates
            .iter()
            .filter(|p| {
                model
                    .self_standing_wall_coverage(p)
                    .is_some_and(|c| c.has_uncovered())
            })
            .map(|p| p.id.0)
            .collect();
        if !uncovered.is_empty() {
            let ids = uncovered
                .iter()
                .map(|i| i.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            issues.push(ModelIssue::model(format!(
                "自立壁（床領域に取り付く壁版）の壁版 {ids} が、荷重を流せる床の上に\
                 載っていません。自重の行き先が無く、長期荷重に算入できません。\
                 壁を床領域の内側へ移すか、床板を割り当てるか、取付き先を梁\
                 （線アンカー）へ変えてください。"
            )));
        }
        let unresolved: Vec<u32> = model
            .wall_plates
            .iter()
            .filter(|p| p.is_attached() && model.wall_plate_extent(p).is_none())
            .map(|p| p.id.0)
            .collect();
        if !unresolved.is_empty() {
            let ids = unresolved
                .iter()
                .map(|i| i.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            issues.push(ModelIssue::model(format!(
                "壁版 {ids} は高さを階高いっぱいとしていますが、壁の下端より上に\
                 階レベルがないため高さが決まりません。自重が算定できません。\
                 上の階を定義するか、立ち上がり高さを数値で指定してください。"
            )));
        }
    }

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
        for region in &model.floor_regions {
            for n in &region.boundary {
                mark(*n);
            }
        }
        for slab in &model.slabs {
            match &slab.shape {
                squid_n_core::model::SlabShape::Enclosed => {
                    if let Some(nodes) = slab.boundary_nodes(model) {
                        for n in &nodes {
                            mark(*n);
                        }
                    }
                }
                squid_n_core::model::SlabShape::Attached { anchor, .. } => match anchor {
                    squid_n_core::model::RegionAnchor::Line { nodes, .. } => {
                        for n in nodes {
                            mark(*n);
                        }
                    }
                    squid_n_core::model::RegionAnchor::Point(n) => mark(*n),
                    squid_n_core::model::RegionAnchor::FloorRegion { .. } => {}
                },
            }
        }
        for plate in &model.wall_plates {
            match &plate.shape {
                squid_n_core::model::WallPlateShape::Enclosed => {
                    if let Some(boundary) = plate.boundary_nodes(model) {
                        for n in &boundary {
                            mark(*n);
                        }
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
    if model.elements.is_empty() {
        return issues;
    }
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
