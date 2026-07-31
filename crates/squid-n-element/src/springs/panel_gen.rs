//! 仕口パネル要素の自動生成（準備計算の前処理）。
//!
//! S 造（CFT を含む）の柱梁接合節点を検出し、[`ElementKind::PanelZone`] の要素を
//! モデルへ生成する。剛域の自動算定（[`crate::beam::apply_auto_rigid_zones`]）と
//! 同じく、解析に先立って 1 回適用する冪等な前処理である。
//!
//! # 生成条件
//!
//! 節点に次のすべてが揃うときにパネルを設ける。
//!
//! - パネル諸元を解決できる断面（H 形鋼・角形鋼管・円形鋼管・CFT）の**柱**
//!   （鉛直材）が 1 本以上取り付く
//! - 断面の割り当てられた**梁**（水平材）が 1 本以上取り付く
//! - 実効体積 `Ve` が正
//!
//! RC・SRC 柱は [`PanelGeometry::from_column`] が `None` を返すため対象外となる。
//! これらの接合部は従来どおり剛域で有限寸法を評価し、接合部の検定は
//! RC 柱梁接合部・SRC パネルゾーンの断面検定が担う。
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
use squid_n_core::panel_zone::{beam_panel_depth, PanelGeometry};

/// 部材軸の鉛直成分がこの値以上なら柱（鉛直材）とみなす。
const COLUMN_EZ: f64 = 0.8;
/// 部材軸の鉛直成分がこの値以下なら梁（水平材）とみなす。
const BEAM_EZ: f64 = 0.2;

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

/// 要素の単位軸ベクトル（2 節点未満・退化長さは `None`）。
fn axis_of(model: &Model, e: &ElementData) -> Option<[f64; 3]> {
    if e.nodes.len() < 2 {
        return None;
    }
    let p0 = model.nodes.get(e.nodes[0].index())?.coord;
    let p1 = model.nodes.get(e.nodes[1].index())?.coord;
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    (l > 1e-9).then(|| [d[0] / l, d[1] / l, d[2] / l])
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
fn panel_at(model: &Model, node: NodeId) -> Option<(GeneratedPanel, Vec<NodeId>)> {
    let mut geom: Option<PanelGeometry> = None;
    let mut shear_modulus = 0.0_f64;
    let mut db = 0.0_f64;
    let mut has_beam = false;
    let mut connected: Vec<NodeId> = Vec::new();

    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Beam) || !e.nodes.contains(&node) {
            continue;
        }
        let Some(axis) = axis_of(model, e) else {
            continue;
        };
        let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let ez = axis[2].abs();
        let far = if e.nodes[0] == node {
            e.nodes[1]
        } else {
            e.nodes[0]
        };
        if ez >= COLUMN_EZ {
            if geom.is_none() {
                if let Some(g) = PanelGeometry::from_column(sec) {
                    geom = Some(g);
                    shear_modulus = e
                        .material
                        .and_then(|mid| model.materials.get(mid.index()))
                        .map(|m| m.shear_modulus())
                        .unwrap_or(0.0);
                }
            }
            connected.push(far);
        } else if ez <= BEAM_EZ {
            has_beam = true;
            db = db.max(beam_panel_depth(sec));
            connected.push(far);
        }
    }

    let geom = geom?;
    if !has_beam {
        return None;
    }
    let ve = geom.effective_volume(db);
    let k_panel = shear_modulus * ve;
    // 諸元を解決できない接合部（寸法・材料が欠けている）にはパネルを設けない。
    // 剛性 0 のパネルは追加自由度が零剛性となり全体剛性行列を特異にする。
    if ve <= 0.0 || k_panel <= 0.0 || !k_panel.is_finite() {
        return None;
    }
    Some((
        GeneratedPanel {
            node,
            dc: geom.dc,
            db,
            tp: geom.tp,
            ve,
            k_panel,
        },
        connected,
    ))
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
    generated
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{Material, Node, Section};
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
                section(0, h_shape(600.0, 200.0, 11.0, 17.0), 600.0),
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

    /// RC 柱の接合部にはパネルを設けない（剛域と RC 柱梁接合部検定で扱う）。
    #[test]
    fn test_rc_column_gets_no_panel() {
        use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
        let bars = BarSet {
            dia: 25.0,
            count: 4,
            layers: 1,
        };
        let rc = SectionShape::RcRect {
            b: 700.0,
            d: 700.0,
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
        };
        let mut model = l_frame(rc);
        let panels = apply_auto_panel_zones(&mut model);
        assert!(panels.is_empty(), "RC 柱は対象外");
        assert_eq!(model.elements.len(), 2);
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
