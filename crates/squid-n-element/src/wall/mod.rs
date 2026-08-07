//! 壁要素。
//!
//! - [`wall_panel`] —  耐震壁（壁エレメントモデル）要素
//! - [`misc_wall`] —   フレーム内雑壁の判定・幾何
//! - [`side_column`] — 耐震壁の側柱
pub mod misc_wall;
pub mod side_column;
pub mod wall_panel;

/// 壁の上下辺へ大梁を追加する（テスト用）。
///
/// 耐震壁は上下辺が大梁で囲まれた壁を対象とする（[`misc_wall::wall_is_framed`]）ため、
/// 壁エレメントとして扱わせたいテストモデルはこの配置を必要とする。追加する部材は
/// 上下辺の条件を満たすことだけが目的なので断面・材料は割り当てない。
///
/// 側柱（左右の鉛直辺）は追加しない。側柱を持たない耐震壁は壁筋比 ps から等価引張
/// 鉄筋比を算定する正規の対象であり、断面のない鉛直材を置くと「側柱はあるのに主筋量を
/// 読み取れない＝入力不備」となってしまう。側柱を伴うモデルが要るテストは、鉛直辺へ
/// 断面付きの線材を別途追加する。
#[cfg(test)]
pub(crate) fn add_surrounding_frame(
    model: &mut squid_n_core::model::Model,
    wall: &squid_n_core::model::ElementData,
) {
    use squid_n_core::ids::ElemId;
    use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis};

    let g = wall_panel::wall_panel_geometry(wall, model).expect("壁の幾何を取得できない");
    let base = model.elements.iter().map(|e| e.id.0).max().unwrap_or(0) + 1;
    for (i, (a, b)) in [(g.bottom[0], g.bottom[1]), (g.top[0], g.top[1])]
        .into_iter()
        .enumerate()
    {
        model.elements.push(ElementData {
            id: ElemId(base + i as u32),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![a, b],
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
    }
}
