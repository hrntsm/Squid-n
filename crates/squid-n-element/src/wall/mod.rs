//! 壁要素。
//!
//! - [`wall_panel`] —  耐震壁（壁エレメントモデル）要素
//! - [`misc_wall`] —   フレーム内雑壁の判定・幾何
//! - [`side_column`] — 耐震壁の側柱
pub mod misc_wall;
pub mod side_column;
pub mod wall_panel;

/// 壁の四周（上下辺・左右の鉛直辺）へ柱・梁を追加する（テスト用）。
///
/// 耐震壁は四周を柱・梁に囲まれた壁を対象とする（[`misc_wall::wall_is_seismic`]）ため、
/// 壁エレメントとして扱わせたいテストモデルはこの配置を必要とする。追加する部材は
/// 四周条件を満たすことだけが目的なので断面・材料は割り当てない。
///
/// 鉛直辺には `Fiber` 種別を置く。四周条件は線材の種別を問わない一方、終局せん断強度
/// Qu の側柱集計（[`wall_panel::WallPanelElement::shear_capacity_of`]）は `Beam` のみを
/// 側柱として数えるため、断面の無い `Beam` を置くと「側柱はあるのに主筋量を読み取れない
/// ＝入力不備」となり Qu が 0 になってしまう。側柱を伴うモデルが要るテストは、追加後に
/// 鉛直辺（ElemId は下辺・上辺に続く 3 番目・4 番目）へ断面を割り当てる。
#[cfg(test)]
pub(crate) fn add_surrounding_frame(
    model: &mut squid_n_core::model::Model,
    wall: &squid_n_core::model::ElementData,
) {
    use squid_n_core::ids::ElemId;
    use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis};

    let g = wall_panel::wall_panel_geometry(wall, model).expect("壁の幾何を取得できない");
    let mut next = model.elements.iter().map(|e| e.id.0).max().unwrap_or(0) + 1;
    for (a, b, kind) in [
        (g.bottom[0], g.bottom[1], ElementKind::Beam),
        (g.top[0], g.top[1], ElementKind::Beam),
        (g.bottom[0], g.top[0], ElementKind::Fiber),
        (g.bottom[1], g.top[1], ElementKind::Fiber),
    ] {
        model.elements.push(ElementData {
            id: ElemId(next),
            kind,
            nodes: smallvec::smallvec![a, b],
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
        next += 1;
    }
}
