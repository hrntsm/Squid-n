//! 仕口パネル要素の自動生成（準備計算の前処理）。
//!
//! S 造（CFT を除く）の柱梁接合節点を検出し、[`ElementKind::PanelZone`] の要素を
//! モデルへ生成する。剛域の自動算定（[`crate::frame::beam::apply_auto_rigid_zones`]）と
//! 同じく、解析に先立って 1 回適用する冪等な前処理である。
//!
//! # 生成条件
//!
//! 対象とする接合部の判定は [`squid_n_core::panel_zone::resolve_panel_joint`] が
//! 一手に担う（規則は同モジュールの「対象とする接合部」を参照）。要約すると、
//! 柱とはりが 1 本以上ずつ取り付き、**それらがすべて S 系**で、諸元を解決できる
//! 柱があり `Ve` が正の節点である。
//!
//! モデル化はこれに加えて、取り付く柱に CFT が 1 本もないことを要求する。
//! 充填コンクリートと通しダイアフラムが接合部のせん断挙動に関与し、鋼管のみの
//! 実効体積による弾性せん断パネルでは剛性を表せないため、接合部を剛節点として
//! 扱う。CFT の接合部は S 造パネルゾーンの断面検定の対象には含まれる。
//!
//! # パネル分のオフセットをモデルへ書き込む
//!
//! パネルを設けた接合部では、部材は節点ではなくパネルの面（柱フェース・梁フェース）
//! で接合する。この接合位置までのオフセットは剛体アームそのものなので、生成時に
//! 各部材の `rigid_zone.panel_offset_i/j` へ書き込む。
//!
//! オフセットを要素の組み立て時にだけ折り込む方式では、`rigid_zone` を直接読む
//! 側（幾何剛性・せん断降伏の `h0`・座屈長さの剛度比 `G`・モデル化図）が
//! オフセットを見落とす。モデルに一度だけ確定させることで、
//! [`RigidZone::rigid_length_i`] を読むすべての経路が同じ値を見る。
//!
//! **剛域長 `length_i/j` とは別のフィールドへ入れる。**剛域の自動算定
//! （[`crate::frame::beam::apply_auto_rigid_zones`]）は `Auto` 端の `length_i/j` を無条件に
//! 再算定するため、同じ場所へ入れると増分解析・時刻歴のように剛域算定を単独で
//! 走らせる経路でオフセットが消える。別フィールドなら呼び出し順に依存しない。
//!
//! 剛体アーム長は `max(剛域長, パネルオフセット)` とする。オフセットは
//! 「部材が節点ではなくパネル面で接合する」という幾何的事実なので、剛域長の手動
//! 指定が 0 でも部材が節点まで伸びることはない。手動指定がオフセットより大きければ
//! そちらが効く。
//!
//! 本関数は全要素の `panel_offset_i/j` を毎回求め直すため、パネルを OFF にすれば
//! 値は 0 へ戻り、繰り返し適用しても増えない（冪等）。
//!
//! # 要素 ID の扱い
//!
//! `Model` は「配列添字 == `ElemId`」を不変条件とするため、パネルの生成・削除では
//! 要素 ID の詰め直しが必要になる。本モジュールは
//!
//! 1. 既存のパネル要素をすべて取り除き、残った要素の ID を連番へ詰め直す
//!    （モデル内の全 `ElemId` 参照を同時に付け替える）
//! 2. 新しいパネル要素を末尾へ追加する
//!
//! の順で処理する。パネルは常に末尾へ並ぶため、パネル生成後にモデルを編集しない
//! 限り 1. で ID は動かない。

use squid_n_core::adjacency::NodeAdjacency;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, PanelZoneMode,
};
use squid_n_core::panel_zone::{
    member_orientation, panel_half_extent, resolve_panel_joint, PanelHalfExtent,
};

/// 1 つの接合部に生成するパネルの諸元（準備計算の結果表示用）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeneratedPanel {
    /// 接合部の節点。
    pub node: NodeId,
    /// 柱せい方向のパネル寸法 `dc` [mm]。
    pub dc: f64,
    /// 梁フランジ板厚中心間距離 `db` [mm]。
    pub db: f64,
    /// パネル板厚 `tp` [mm]。
    pub tp: f64,
    /// 実効体積 `Ve` [mm³]。
    pub ve: f64,
    /// パネルせん断剛性 `Kxp = Kyp = G・Ve` [N·mm/rad]。
    pub k_panel: f64,
}

/// モデルから既存の仕口パネル要素をすべて取り除き、残った要素の ID を連番へ
/// 詰め直す（モデル内の全 `ElemId` 参照も同時に付け替える）。
fn remove_existing_panels(model: &mut Model) {
    if !model
        .elements
        .iter()
        .any(|e| matches!(e.kind, ElementKind::PanelZone))
    {
        return;
    }
    let mut remap: Vec<Option<u32>> = vec![None; model.elements.len()];
    let mut kept: Vec<ElementData> = Vec::with_capacity(model.elements.len());
    for mut e in std::mem::take(&mut model.elements) {
        if matches!(e.kind, ElementKind::PanelZone) {
            continue;
        }
        let new_id = kept.len() as u32;
        remap[e.id.index()] = Some(new_id);
        e.id = ElemId(new_id);
        kept.push(e);
    }
    model.elements = kept;

    // 詰め直しに伴う ID 参照の付け替え。パネル要素は側テーブル属性・部材荷重・
    // 部材グループのいずれからも参照されないため、`remap` が `None` になる
    // 参照は生じない（防御的に、解決できない参照はそのまま残す）。
    let shift = |id: &mut ElemId| {
        if let Some(Some(new)) = remap.get(id.index()).copied() {
            *id = ElemId(new);
        }
    };
    for lc in &mut model.load_cases {
        for ml in &mut lc.member {
            shift(&mut ml.elem);
        }
    }
    // 側テーブル属性と一本部材指定（beam_groups）の参照は shift_elem_attr_refs が
    // 一括で付け替える。
    model.shift_elem_attr_refs(shift);
}

/// 節点 `node` にパネルを設けるべきか判定し、設ける場合は諸元と接続節点を返す。
///
/// 判定は [`resolve_panel_joint`] に委ね、本関数はモデル化固有の追加条件
/// （CFT を含まない・せん断弾性係数が正）とパネルせん断剛性の算定を担う。
fn panel_at(
    model: &Model,
    adjacency: &NodeAdjacency,
    node: NodeId,
) -> Option<(GeneratedPanel, Vec<NodeId>)> {
    let joint = resolve_panel_joint(model, node, adjacency.elements_at(model, node))?;
    // CFT はモデル化の対象外（充填部がせん断挙動に関与するため剛節点として扱う）。
    if joint.has_filled_column {
        return None;
    }
    let column = model.elements.get(joint.column.index())?;
    let shear_modulus = model
        .element_material(column)
        .map(|m| m.shear_modulus())
        .unwrap_or(0.0);
    let k_panel = shear_modulus * joint.ve;
    // 諸元を解決できない接合部（材料が欠けている）にはパネルを設けない。
    // 剛性 0 のパネルは追加自由度が零剛性となり全体剛性行列を特異にする。
    if k_panel <= 0.0 || !k_panel.is_finite() {
        return None;
    }

    // 描画・パネル自由度との連成に用いる、接合部へ取り付く部材の他端。
    // 節点の照合を先に行い、合致した部材だけ向きを判定する（向きの判定は
    // 座標参照と平方根を伴うため、全要素へ先に掛けない）。
    let connected: Vec<NodeId> = adjacency
        .elements_at(model, node)
        .filter_map(|e| {
            let far = far_end_at(e, node)?;
            member_orientation(model, e).map(|_| far)
        })
        .collect();

    Some((
        GeneratedPanel {
            node,
            dc: joint.geometry.dc,
            db: joint.db,
            tp: joint.geometry.tp,
            ve: joint.ve,
            k_panel,
        },
        connected,
    ))
}

/// 要素 `e` の端点に `node` が現れるなら、その反対端の節点を返す。
fn far_end_at(e: &ElementData, node: NodeId) -> Option<NodeId> {
    match e.nodes.iter().take(2).position(|n| *n == node)? {
        0 => e.nodes.get(1).copied(),
        _ => e.nodes.first().copied(),
    }
}

/// 部材の `rigid_zone.panel_offset_i/j` を、現在のパネル配置から求め直す。
///
/// パネルが 1 つもなければ全要素の値が 0 になるため、モデル化を OFF にすると
/// オフセットは消える（冪等）。
///
/// 節点ごとに全要素を走査すると パネル数 × 要素数 になるため、半寸法を節点表へ
/// 引けるようにしたうえで、要素側を 1 周して両端を引く。
fn apply_panel_offsets(model: &mut Model, adjacency: &NodeAdjacency, panels: &[GeneratedPanel]) {
    let mut extent_of: Vec<Option<PanelHalfExtent>> = vec![None; model.nodes.len()];
    for p in panels {
        let extent = panel_half_extent(model, p.node, adjacency.elements_at(model, p.node));
        if let Some(slot) = extent_of.get_mut(p.node.index()) {
            *slot = Some(extent);
        }
    }

    let offsets: Vec<(usize, [f64; 2])> = model
        .elements
        .iter()
        .enumerate()
        .map(|(ei, e)| {
            let ends = match (e.nodes.len() >= 2)
                .then(|| member_orientation(model, e))
                .flatten()
            {
                Some(orientation) => {
                    let offset_at = |end: usize| {
                        extent_of
                            .get(e.nodes[end].index())
                            .and_then(|x| x.as_ref())
                            .map(|x| x.offset_for(orientation))
                            .unwrap_or(0.0)
                    };
                    [offset_at(0), offset_at(1)]
                }
                None => [0.0, 0.0],
            };
            (ei, ends)
        })
        .collect();

    for (ei, ends) in offsets {
        let zone = &mut model.elements[ei].rigid_zone;
        zone.panel_offset_i = ends[0];
        zone.panel_offset_j = ends[1];
    }
}

/// モデルの仕口パネル要素を再生成する（冪等）。
///
/// `Model::panel_zone` が [`PanelZoneMode::None`] のときは既存のパネルを取り除く
/// だけで、新しいパネルは生成しない。
///
/// 戻り値は生成したパネルの諸元（節点 index の昇順）。準備計算の結果表示に用いる。
pub fn apply_auto_panel_zones(model: &mut Model) -> Vec<GeneratedPanel> {
    remove_existing_panels(model);
    if model.panel_zone != PanelZoneMode::Model {
        // パネルが 1 つもない状態のオフセット（＝すべて 0）へ戻す。
        for e in &mut model.elements {
            e.rigid_zone.panel_offset_i = 0.0;
            e.rigid_zone.panel_offset_j = 0.0;
        }
        return Vec::new();
    }
    // パネル要素を取り除いた状態で作る（パネル自身は線材ではないため隣接には
    // 入らないが、要素の詰め直しで添字が動くため取り除いた後に構築する）。
    let adjacency = NodeAdjacency::build(model);

    let mut generated = Vec::new();
    let mut new_elements = Vec::new();
    let node_ids: Vec<NodeId> = model.nodes.iter().map(|n| n.id).collect();
    for node in node_ids {
        let Some((panel, connected)) = panel_at(model, &adjacency, node) else {
            continue;
        };
        let mut nodes: smallvec::SmallVec<[NodeId; 8]> = smallvec::smallvec![node];
        nodes.extend(connected);
        new_elements.push(ElementData {
            id: ElemId(0), // 追加時に連番へ振り直す
            kind: ElementKind::PanelZone,
            nodes,
            section: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        generated.push(panel);
    }

    for mut e in new_elements {
        e.id = ElemId(model.elements.len() as u32);
        model.elements.push(e);
    }

    apply_panel_offsets(model, &adjacency, &generated);
    generated
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::MaterialCategory;
    use squid_n_core::model::{Material, Node, Section};
    use squid_n_core::panel_zone::PanelGeometry;
    use squid_n_core::section_shape::SectionShape;

    fn h_shape(height: f64, width: f64, tw: f64, tf: f64) -> SectionShape {
        SectionShape::SteelH {
            height,
            width,
            web_thick: tw,
            flange_thick: tf,
        }
    }

    /// 断面（材料 0 = 鋼材を割り当てる。材料は断面が持つ）。
    fn section(id: u32, shape: SectionShape, depth: f64) -> Section {
        section_with_mat(id, shape, depth, 0)
    }

    /// 主材料を指定して断面を作る。
    fn section_with_mat(id: u32, shape: SectionShape, depth: f64, mat: u32) -> Section {
        Section {
            id: SectionId(id),
            material: Some(MaterialId(mat)),
            name: String::new(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth,
            width: depth,
            as_y: 4.0e3,
            as_z: 4.0e3,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: Some(shape),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }

    /// 材料 0 = 鋼材、材料 1 = コンクリート。
    fn test_material(id: u32, category: MaterialCategory) -> Material {
        Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(id),
            name: String::new(),
            category,
            young: 205_000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }
    }

    fn member(id: u32, n0: u32, n1: u32, sec: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
            section: Some(SectionId(sec)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 柱 1 本・梁 1 本の S 造 L 型接合部モデル。`col_shape` で柱断面を差し替える。
    fn l_frame(col_shape: SectionShape) -> Model {
        l_frame_with_beam(col_shape, h_shape(600.0, 200.0, 11.0, 17.0), 600.0)
    }

    /// [`l_frame`] の梁断面も差し替えられる版。
    fn l_frame_with_beam(
        col_shape: SectionShape,
        beam_shape: SectionShape,
        beam_depth: f64,
    ) -> Model {
        let node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        Model {
            nodes: vec![
                node(0, [0.0, 0.0, 3000.0]),
                node(1, [6000.0, 0.0, 3000.0]),
                node(2, [0.0, 0.0, 0.0]),
            ],
            sections: vec![
                section(0, beam_shape, beam_depth),
                section(1, col_shape, 400.0),
            ],
            materials: vec![
                test_material(0, MaterialCategory::Steel),
                test_material(1, MaterialCategory::Concrete),
            ],
            elements: vec![member(0, 0, 1, 0), member(1, 2, 0, 1)],
            ..Default::default()
        }
    }

    /// S 造の柱梁接合節点にパネルが 1 つ生成され、モデルの不変条件
    /// （配列添字 == ElemId）が保たれる。
    #[test]
    fn test_generates_panel_at_steel_joint() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        let panels = apply_auto_panel_zones(&mut model);

        assert_eq!(panels.len(), 1);
        assert_eq!(panels[0].node, NodeId(0));
        assert!((panels[0].dc - (400.0 - 21.0)).abs() < 1e-9);
        assert!((panels[0].db - (600.0 - 17.0)).abs() < 1e-9);
        assert!((panels[0].tp - 13.0).abs() < 1e-9);
        assert!(panels[0].k_panel > 0.0);

        let panel_elems: Vec<_> = model
            .elements
            .iter()
            .filter(|e| matches!(e.kind, ElementKind::PanelZone))
            .collect();
        assert_eq!(panel_elems.len(), 1);
        assert_eq!(panel_elems[0].nodes[0], NodeId(0), "先頭は接合部の節点");
        // 描画用に接続部材の他端も並ぶ。
        assert!(panel_elems[0].nodes.len() >= 2);
        model.validate().expect("配列添字 == ElemId が保たれる");
    }

    /// 冪等性: 繰り返し適用してもパネルは増えず、要素 ID も動かない。
    #[test]
    fn test_is_idempotent() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        apply_auto_panel_zones(&mut model);
        let first = model.elements.clone();
        apply_auto_panel_zones(&mut model);
        assert_eq!(model.elements.len(), first.len());
        for (a, b) in model.elements.iter().zip(first.iter()) {
            assert_eq!(a.id, b.id);
            assert_eq!(a.kind, b.kind);
            assert_eq!(a.nodes, b.nodes);
        }
        model.validate().expect("不変条件");
    }

    /// モデル化を OFF にすると既存のパネルが取り除かれ、要素 ID が詰め直される。
    #[test]
    fn test_disabled_mode_removes_panels() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        apply_auto_panel_zones(&mut model);
        assert_eq!(model.elements.len(), 3);

        model.panel_zone = PanelZoneMode::None;
        let panels = apply_auto_panel_zones(&mut model);
        assert!(panels.is_empty());
        assert_eq!(model.elements.len(), 2);
        assert!(!model
            .elements
            .iter()
            .any(|e| matches!(e.kind, ElementKind::PanelZone)));
        model.validate().expect("不変条件");
    }

    /// パネル生成後に追加した部材があっても、再生成で ID 参照の整合が保たれる
    /// （パネルを除いて詰め直したうえで末尾へ再追加する）。
    #[test]
    fn test_regeneration_keeps_id_references_consistent() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        apply_auto_panel_zones(&mut model);
        assert_eq!(model.elements.len(), 3, "梁・柱・パネル");

        // パネルの後ろへ利用者が部材を追加した状況（ID は末尾の 3）。
        let mut added = member(3, 1, 2, 0);
        added.id = ElemId(3);
        model.elements.push(added);
        // その部材を参照する部材グループを作る。
        model.beam_groups = vec![vec![ElemId(3)]];

        apply_auto_panel_zones(&mut model);
        model.validate().expect("配列添字 == ElemId が保たれる");

        // 追加部材はパネルを詰めた分だけ前へ繰り上がり、参照も追従する。
        let added_idx = model
            .elements
            .iter()
            .position(|e| e.nodes.as_slice() == [NodeId(1), NodeId(2)])
            .expect("追加した部材");
        assert_eq!(model.elements[added_idx].id.index(), added_idx);
        assert_eq!(model.beam_groups[0][0], model.elements[added_idx].id);
    }

    /// RC 矩形断面（柱・梁の双方に使う）。
    fn rc_shape(b: f64, d: f64) -> SectionShape {
        use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
        let bars = BarSet {
            dia: 25.0,
            count: 4,
            layers: 1,
        };
        SectionShape::RcRect {
            b,
            d,
            rebar: RcRebar {
                main_x: bars.clone(),
                main_y: bars,
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        }
    }

    /// RC 柱の接合部にはパネルを設けない（剛域と RC 柱梁接合部検定で扱う）。
    #[test]
    fn test_rc_column_gets_no_panel() {
        let rc = rc_shape(700.0, 700.0);
        let mut model = l_frame(rc);
        let panels = apply_auto_panel_zones(&mut model);
        assert!(panels.is_empty(), "RC 柱は対象外");
        assert_eq!(model.elements.len(), 2);
    }

    /// CFT 柱の接合部にはパネルを設けない（充填コンクリートと通しダイアフラムが
    /// 接合部のせん断挙動に関与し、鋼管のみの実効体積では剛性を表せないため）。
    /// 断面検定は CFT も対象に含めるため、検定側の判定とは分かれる。
    #[test]
    fn test_cft_column_is_not_modeling_target() {
        for shape in [
            SectionShape::CftBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
            },
            SectionShape::CftPipe {
                outer_dia: 400.0,
                thick: 12.0,
            },
        ] {
            let mut model = l_frame(shape);
            let panels = apply_auto_panel_zones(&mut model);
            assert!(panels.is_empty(), "CFT 柱はモデル化の対象外");
            assert_eq!(model.elements.len(), 2, "パネル要素は生成されない");

            // 一方、諸元の解決自体は成功する（断面検定はこの経路を使う）。
            let geom = PanelGeometry::from_column(&model.sections[1]).expect("諸元は解決できる");
            assert!(geom.filled, "モデル化対象ではない");
            assert!(geom.effective_volume(500.0) > 0.0, "検定用の Ve は求まる");
        }
    }

    /// 角形鋼管・円形鋼管（CFT でない S 造）はモデル化の対象になる。
    #[test]
    fn test_steel_tube_columns_are_modeling_targets() {
        for shape in [
            SectionShape::SteelBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
                corner_r: 0.0,
            },
            SectionShape::SteelPipe {
                outer_dia: 400.0,
                thick: 12.0,
            },
        ] {
            let mut model = l_frame(shape);
            let panels = apply_auto_panel_zones(&mut model);
            assert_eq!(panels.len(), 1, "S 造の鋼管柱はモデル化の対象");
        }
    }

    /// RC 梁が取り付く接合部にはパネルを設けない。柱が S でも接合部は RC になり、
    /// 鋼柱のウェブだけの実効体積では挙動を表せない。
    #[test]
    fn test_rc_beam_gets_no_panel_even_with_steel_column() {
        let mut model = l_frame_with_beam(
            h_shape(400.0, 400.0, 13.0, 21.0),
            rc_shape(400.0, 700.0),
            700.0,
        );
        // 判定は材料の区分による。梁の断面（断面 0）へコンクリートを割り当てる。
        model.sections[0].material = Some(MaterialId(1));
        let panels = apply_auto_panel_zones(&mut model);
        assert!(panels.is_empty(), "RC 梁の接合部は対象外");
        assert_eq!(model.elements.len(), 2, "パネル要素は生成されない");
    }

    /// S 梁と RC 梁が混在する接合部も対象外とする（1 本でも非 S があれば設けない）。
    #[test]
    fn test_mixed_beams_get_no_panel() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        // Y 方向へ RC 梁を追加する。
        model.nodes.push(Node {
            id: NodeId(3),
            coord: [0.0, 6000.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
        model
            .sections
            .push(section_with_mat(2, rc_shape(400.0, 700.0), 700.0, 1));
        // 判定は材料の区分による（断面 2 の材料 1 = コンクリート）。
        model.elements.push(member(2, 0, 3, 2));

        let panels = apply_auto_panel_zones(&mut model);
        assert!(panels.is_empty(), "RC 梁が 1 本でも混じれば対象外");
    }

    /// パネル分のオフセットが `panel_offset_i/j` へ書き込まれる。
    /// 梁の端は柱せいの 1/2、柱の端は梁せいの 1/2。剛域長 `length_i/j` は触らない。
    #[test]
    fn test_offsets_are_written_into_rigid_zones() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        apply_auto_panel_zones(&mut model);

        // 梁（要素 0）の i 端が接合部。オフセットは柱せい 400 の 1/2。
        let beam = &model.elements[0].rigid_zone;
        assert!((beam.panel_offset_i - 200.0).abs() < 1e-9);
        assert_eq!(beam.panel_offset_j, 0.0, "接合部でない端は 0");
        assert_eq!(beam.length_i, 0.0, "剛域長そのものは変えない");
        assert!((beam.rigid_length_i() - 200.0).abs() < 1e-9);

        // 柱（要素 1）の j 端が接合部。オフセットは梁せい 600 の 1/2。
        let col = &model.elements[1].rigid_zone;
        assert!((col.panel_offset_j - 300.0).abs() < 1e-9);
        assert_eq!(col.panel_offset_i, 0.0);
        assert!((col.rigid_length_j() - 300.0).abs() < 1e-9);
    }

    /// 剛体アーム長は `max(剛域長, オフセット)`。手動指定が大きければそちらが効き、
    /// 小さくてもオフセットの分は確保される（接合位置は幾何的事実のため）。
    #[test]
    fn test_rigid_length_takes_larger_of_zone_and_offset() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        model.elements[0].rigid_zone.length_i = 500.0;
        model.elements[0].rigid_zone.source_i = squid_n_core::model::ZoneSource::Manual;
        model.elements[1].rigid_zone.length_j = 10.0;
        model.elements[1].rigid_zone.source_j = squid_n_core::model::ZoneSource::Manual;

        apply_auto_panel_zones(&mut model);
        let beam = &model.elements[0].rigid_zone;
        assert_eq!(beam.length_i, 500.0, "手動指定はそのまま残る");
        assert!((beam.panel_offset_i - 200.0).abs() < 1e-9);
        assert_eq!(beam.rigid_length_i(), 500.0, "大きい方が剛体アーム長");

        let col = &model.elements[1].rigid_zone;
        assert_eq!(col.length_j, 10.0, "手動指定はそのまま残る");
        assert!(
            (col.rigid_length_j() - 300.0).abs() < 1e-9,
            "オフセットの方が大きければそちらが効く"
        );
    }

    /// 剛域の自動算定を単独で走らせてもオフセットは消えない。
    ///
    /// 増分解析・時刻歴・MCP のジョブは `apply_auto_rigid_zones` だけを呼ぶ経路が
    /// あるため、剛域長と同じフィールドへ入れると「パネル要素は残るのに剛体アームだけ
    /// 消えたモデル」で解析が走る。別フィールドに保持することで順序に依存しない。
    #[test]
    fn test_offsets_survive_rigid_zone_recomputation() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        apply_auto_panel_zones(&mut model);
        let before: Vec<_> = model
            .elements
            .iter()
            .map(|e| (e.rigid_zone.panel_offset_i, e.rigid_zone.panel_offset_j))
            .collect();

        crate::frame::beam::apply_auto_rigid_zones(
            &mut model,
            &crate::frame::beam::RigidZoneRule::default(),
        );

        let after: Vec<_> = model
            .elements
            .iter()
            .map(|e| (e.rigid_zone.panel_offset_i, e.rigid_zone.panel_offset_j))
            .collect();
        assert_eq!(before, after, "剛域の再算定でオフセットが消えてはいけない");
        assert!((model.elements[0].rigid_zone.rigid_length_i() - 200.0).abs() < 1e-9);
    }

    /// 繰り返し適用してもオフセットは増えず、OFF にすると 0 へ戻る。
    #[test]
    fn test_offset_write_is_idempotent() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        apply_auto_panel_zones(&mut model);
        let first: Vec<_> = model
            .elements
            .iter()
            .map(|e| (e.rigid_zone.panel_offset_i, e.rigid_zone.panel_offset_j))
            .collect();
        apply_auto_panel_zones(&mut model);
        let second: Vec<_> = model
            .elements
            .iter()
            .map(|e| (e.rigid_zone.panel_offset_i, e.rigid_zone.panel_offset_j))
            .collect();
        assert_eq!(first, second);
        assert!(first.iter().any(|(i, j)| *i > 0.0 || *j > 0.0));

        model.panel_zone = PanelZoneMode::None;
        apply_auto_panel_zones(&mut model);
        assert!(
            model
                .elements
                .iter()
                .all(|e| e.rigid_zone.panel_offset_i == 0.0 && e.rigid_zone.panel_offset_j == 0.0),
            "モデル化を OFF にするとオフセットは消える"
        );
    }

    /// 柱が複数取り付く接合部では、実効体積 Ve が最小になる柱の諸元を採る。
    /// 要素の並び順を入れ替えても結果が変わらないことを併せて確認する。
    #[test]
    fn test_smallest_ve_column_is_used() {
        // 上柱を細い H 形（ウェブ薄 → Ve 小）にする。
        let thin = h_shape(400.0, 400.0, 9.0, 21.0);
        let thick = h_shape(400.0, 400.0, 22.0, 21.0);

        let build = |upper_first: bool| {
            let mut model = l_frame(thick.clone());
            model.nodes.push(Node {
                id: NodeId(3),
                coord: [0.0, 0.0, 6000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            });
            model.sections.push(section(2, thin.clone(), 400.0));
            // 上柱（細い断面）を先頭へ入れるか末尾へ入れるかで順序を変える。
            let upper = member(2, 0, 3, 2);
            if upper_first {
                model.elements.insert(0, upper);
                for (i, e) in model.elements.iter_mut().enumerate() {
                    e.id = ElemId(i as u32);
                }
            } else {
                model.elements.push(upper);
            }
            model
        };

        let mut a = build(true);
        let mut b = build(false);
        let pa = apply_auto_panel_zones(&mut a);
        let pb = apply_auto_panel_zones(&mut b);
        assert_eq!(pa.len(), 1);
        assert_eq!(pb.len(), 1);
        assert!(
            (pa[0].tp - 9.0).abs() < 1e-9,
            "Ve 最小（ウェブ薄）の柱を採る: tp={}",
            pa[0].tp
        );
        assert_eq!(pa[0], pb[0], "要素の並び順に依存しない");
    }

    /// 梁が取り付かない節点（柱だけ）にはパネルを設けない。
    #[test]
    fn test_column_only_node_gets_no_panel() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        // 梁を取り除く
        model.elements.remove(0);
        model.elements[0].id = ElemId(0);
        let panels = apply_auto_panel_zones(&mut model);
        assert!(panels.is_empty());
    }
}
