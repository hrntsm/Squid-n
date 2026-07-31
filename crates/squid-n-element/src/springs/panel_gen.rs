//! 仕口パネル要素の自動生成（準備計算の前処理）。
//!
//! S 造（CFT を除く）の柱梁接合節点を検出し、[`ElementKind::PanelZone`] の要素を
//! モデルへ生成する。剛域の自動算定（[`crate::beam::apply_auto_rigid_zones`]）と
//! 同じく、解析に先立って 1 回適用する冪等な前処理である。
//!
//! # 生成条件
//!
//! 対象とする接合部の判定は [`squid_n_core::panel_zone::resolve_panel_joint`] が
//! 一手に担う（規則は同モジュールの「対象とする接合部」を参照）。要約すると、
//! 柱とはりが 1 本以上ずつ取り付き、**それらがすべて S 系**で、諸元を解決できる
//! 柱があり `Ve` が正の節点である。
//!
//! モデル化はこれに加えて、取り付く柱に CFT が 1 本も無いことを要求する。
//! 充填コンクリートと通しダイアフラムが接合部のせん断挙動に関与し、鋼管のみの
//! 実効体積による弾性せん断パネルでは剛性を表せないため、接合部を剛節点として
//! 扱う。CFT の接合部は S 造パネルゾーンの断面検定の対象には含まれる。
//!
//! # パネル分のオフセットを剛域長へ書き込む
//!
//! パネルを設けた接合部では、部材は節点ではなくパネルの面（柱フェース・梁フェース）
//! で接合する。この接合位置までのオフセットは剛体アームそのものなので、生成時に
//! 各部材の**剛域長へ書き込む**（`max(現在値, オフセット)`）。
//!
//! オフセットを要素の組み立て時にだけ折り込む方式では、`rigid_zone` を直接読む
//! 側（幾何剛性・せん断降伏の `h0`・座屈長さの剛度比 `G`・モデル化図）が
//! オフセットを見落とす。モデルに一度だけ確定させることで、`rigid_zone` を読む
//! すべての経路が同じ値を見る。
//!
//! 書き込みは剛域長の `source`（`Auto`/`Manual`）に依らず行う。オフセットは
//! 「部材が節点ではなくパネル面で接合する」という幾何的事実であり、剛域長の
//! 設計的な調整量とは性質が異なるためである。`max` を取るので、手動指定が
//! オフセットより大きければその値が残る。
//!
//! 剛域の自動算定（[`crate::beam::apply_auto_rigid_zones`]）はパネル生成より前に
//! 走って `Auto` 端を再算定するため、パネルを OFF にすれば書き込みは消え、
//! 繰り返し適用しても値は増えない（冪等）。
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
    for group in &mut model.beam_groups {
        for id in group.iter_mut() {
            shift(id);
        }
    }
    model.shift_elem_attr_refs(shift);
}

/// 節点 `node` にパネルを設けるべきか判定し、設ける場合は諸元と接続節点を返す。
///
/// 判定は [`resolve_panel_joint`] に委ね、本関数はモデル化固有の追加条件
/// （CFT を含まない・せん断弾性係数が正）とパネルせん断剛性の算定を担う。
fn panel_at(model: &Model, node: NodeId) -> Option<(GeneratedPanel, Vec<NodeId>)> {
    let joint = resolve_panel_joint(model, node, &model.elements)?;
    // CFT はモデル化の対象外（`PanelGeometry::is_modeling_target`）。
    if joint.has_filled_column {
        return None;
    }
    let column = model.elements.get(joint.column.index())?;
    let shear_modulus = column
        .material
        .and_then(|mid| model.materials.get(mid.index()))
        .map(|m| m.shear_modulus())
        .unwrap_or(0.0);
    let k_panel = shear_modulus * joint.ve;
    // 諸元を解決できない接合部（材料が欠けている）にはパネルを設けない。
    // 剛性 0 のパネルは追加自由度が零剛性となり全体剛性行列を特異にする。
    if k_panel <= 0.0 || !k_panel.is_finite() {
        return None;
    }

    // 描画・パネル自由度との連成に用いる、接合部へ取り付く部材の他端。
    let connected: Vec<NodeId> = model
        .elements
        .iter()
        .filter(|e| member_orientation(model, e).is_some())
        .filter_map(|e| match (e.nodes[0], e.nodes[1]) {
            (n, far) if n == node => Some(far),
            (far, n) if n == node => Some(far),
            _ => None,
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

/// パネルを設けた接合部で、取り付く部材の剛域長へパネル分のオフセットを
/// 書き込む（モジュール冒頭「パネル分のオフセットを剛域長へ書き込む」）。
fn apply_panel_offsets(model: &mut Model, panel_nodes: &[NodeId]) {
    let extents: Vec<(NodeId, PanelHalfExtent)> = panel_nodes
        .iter()
        .map(|&n| (n, panel_half_extent(model, n, &model.elements)))
        .collect();

    for (node, extent) in extents {
        let updates: Vec<(usize, usize, f64)> = model
            .elements
            .iter()
            .enumerate()
            .filter_map(|(ei, e)| {
                let orientation = member_orientation(model, e)?;
                let offset = extent.offset_for(orientation);
                if offset <= 0.0 {
                    return None;
                }
                let end = e.nodes.iter().take(2).position(|n| *n == node)?;
                Some((ei, end, offset))
            })
            .collect();
        for (ei, end, offset) in updates {
            let zone = &mut model.elements[ei].rigid_zone;
            let length = if end == 0 {
                &mut zone.length_i
            } else {
                &mut zone.length_j
            };
            *length = length.max(offset);
        }
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
        return Vec::new();
    }

    let mut generated = Vec::new();
    let mut new_elements = Vec::new();
    let node_ids: Vec<NodeId> = model.nodes.iter().map(|n| n.id).collect();
    for node in node_ids {
        let Some((panel, connected)) = panel_at(model, node) else {
            continue;
        };
        let mut nodes: smallvec::SmallVec<[NodeId; 8]> = smallvec::smallvec![node];
        nodes.extend(connected);
        new_elements.push(ElementData {
            id: ElemId(0), // 追加時に連番へ振り直す
            kind: ElementKind::PanelZone,
            nodes,
            section: None,
            material: None,
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

    let panel_nodes: Vec<NodeId> = generated.iter().map(|p| p.node).collect();
    apply_panel_offsets(model, &panel_nodes);
    generated
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, SectionId};
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

    fn section(id: u32, shape: SectionShape, depth: f64) -> Section {
        Section {
            id: SectionId(id),
            name: String::new(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth,
            width: depth,
            as_y: 4.0e3,
            as_z: 4.0e3,
            panel_thickness: None,
            thickness: None,
            shape: Some(shape),
        }
    }

    fn member(id: u32, n0: u32, n1: u32, sec: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
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
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "SN400B".into(),
                young: 205_000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
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
                    grade: None,
                },
                main_grade: None,
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
            assert!(!geom.is_modeling_target(), "モデル化対象ではない");
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
            .push(section(2, rc_shape(400.0, 700.0), 700.0));
        model.elements.push(member(2, 0, 3, 2));

        let panels = apply_auto_panel_zones(&mut model);
        assert!(panels.is_empty(), "RC 梁が 1 本でも混じれば対象外");
    }

    /// パネル分のオフセットが部材の剛域長へ書き込まれる。
    /// 梁の端は柱せいの 1/2、柱の端は梁せいの 1/2。
    #[test]
    fn test_offsets_are_written_into_rigid_zones() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        apply_auto_panel_zones(&mut model);

        // 梁（要素 0）の i 端が接合部。オフセットは柱せい 400 の 1/2。
        assert!((model.elements[0].rigid_zone.length_i - 200.0).abs() < 1e-9);
        assert_eq!(
            model.elements[0].rigid_zone.length_j, 0.0,
            "接合部でない端は変えない"
        );
        // 柱（要素 1）の j 端が接合部。オフセットは梁せい 600 の 1/2。
        assert!((model.elements[1].rigid_zone.length_j - 300.0).abs() < 1e-9);
        assert_eq!(model.elements[1].rigid_zone.length_i, 0.0);
    }

    /// 書き込みは `max` なので、手動指定の剛域長がオフセットより大きければ残る。
    /// 逆にオフセットより小さい手動値は上書きする（接合位置は幾何的事実のため）。
    #[test]
    fn test_offsets_keep_larger_manual_rigid_zone() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        model.elements[0].rigid_zone.length_i = 500.0;
        model.elements[0].rigid_zone.source_i = squid_n_core::model::ZoneSource::Manual;
        model.elements[1].rigid_zone.length_j = 10.0;
        model.elements[1].rigid_zone.source_j = squid_n_core::model::ZoneSource::Manual;

        apply_auto_panel_zones(&mut model);
        assert_eq!(
            model.elements[0].rigid_zone.length_i, 500.0,
            "オフセットより大きい手動値は残す"
        );
        assert!(
            (model.elements[1].rigid_zone.length_j - 300.0).abs() < 1e-9,
            "オフセットより小さい手動値は上書きする"
        );
    }

    /// 繰り返し適用しても剛域長は増えない（`max` なので冪等）。
    #[test]
    fn test_offset_write_is_idempotent() {
        let mut model = l_frame(h_shape(400.0, 400.0, 13.0, 21.0));
        apply_auto_panel_zones(&mut model);
        let first: Vec<_> = model
            .elements
            .iter()
            .map(|e| (e.rigid_zone.length_i, e.rigid_zone.length_j))
            .collect();
        apply_auto_panel_zones(&mut model);
        let second: Vec<_> = model
            .elements
            .iter()
            .map(|e| (e.rigid_zone.length_i, e.rigid_zone.length_j))
            .collect();
        assert_eq!(first, second);
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
