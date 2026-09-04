//! ベンチ例題が共有する骨組生成の定型。
//!
//! 4 本のベンチ（`eigen_bench`・`parallel_bench`・`pushover_bench`・`th_bench`）は
//! いずれも「nx×ny スパン・nz 層の立体ラーメン（柱＋X/Y 大梁）」を測定対象とする。
//! 節点格子の採番・座標・支持条件と、梁要素の構築の定型はどれも同じで、
//! `ElementData` や `Node` にフィールドが増えるたび 4 本すべてを直す必要があった。
//! その定型だけをここへ集める。
//!
//! **各ベンチの差異は意図的に呼び出し側へ残す。** 節点質量の有無・階の割当・
//! 断面を柱と梁で分けるかは、そのベンチが何を測っているかを表す情報だからである。
//! ここへオプションとして畳み込むと、`spec.mass = false` の意味を読むために
//! このモジュールを開くことになり、ベンチ 1 本ずつの読みやすさが落ちる。
//!
//! ベンチごとに使うヘルパが違うため、モジュール全体で `dead_code` を許可する
//! （例題は 1 本ずつ独立してコンパイルされ、使わないヘルパは未使用と報告される）。

#![allow(dead_code)]

use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory,
    Node, Section,
};

/// 立体ラーメンの節点格子（nx×ny スパン・nz 層）。
///
/// 寸法だけを持つ値なので `Copy` とする。`move` クロージャへ渡す使い方
/// （最上層の節点 ID を集める等）が素直に書けるようにするためである。
#[derive(Clone, Copy)]
pub struct FrameGrid {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    /// 1 スパンの長さ [mm]（X・Y 共通）。
    pub span: f64,
    /// 1 層の階高 [mm]。
    pub height: f64,
}

impl FrameGrid {
    /// 既定寸法（6000mm スパン・3500mm 階高）の格子。
    pub fn new(nx: usize, ny: usize, nz: usize) -> Self {
        Self {
            nx,
            ny,
            nz,
            span: 6000.0,
            height: 3500.0,
        }
    }

    /// 格子点 `(ix, iy, iz)` の節点 ID。
    ///
    /// **添字と ID を一致させる**（`nodes[i].id == NodeId(i)`）。`Model::validate` が
    /// 検証する不変条件で、`Model::node` の O(1) 引き当てもこれを前提とする。
    pub fn node_id(&self, ix: usize, iy: usize, iz: usize) -> NodeId {
        NodeId((iz * (self.nx + 1) * (self.ny + 1) + iy * (self.nx + 1) + ix) as u32)
    }

    /// 全格子点の節点を作る。基部（`iz == 0`）は 6 自由度固定、それ以外は自由。
    ///
    /// 節点質量・階の割当はベンチごとに違うため、生成した節点と層番号 `iz` を
    /// `customize` へ渡して呼び出し側に決めさせる。
    pub fn build_nodes(&self, mut customize: impl FnMut(&mut Node, usize)) -> Vec<Node> {
        let mut nodes = Vec::with_capacity((self.nx + 1) * (self.ny + 1) * (self.nz + 1));
        for iz in 0..=self.nz {
            for iy in 0..=self.ny {
                for ix in 0..=self.nx {
                    let mut node = Node {
                        id: self.node_id(ix, iy, iz),
                        coord: [
                            ix as f64 * self.span,
                            iy as f64 * self.span,
                            iz as f64 * self.height,
                        ],
                        restraint: if iz == 0 {
                            Dof6Mask::FIXED
                        } else {
                            Dof6Mask::FREE
                        },
                        mass: None,
                        story: None,
                        support_spring: None,
                    };
                    customize(&mut node, iz);
                    nodes.push(node);
                }
            }
        }
        nodes
    }

    /// 柱（全層・全通り）と大梁（各階の X・Y 両方向）を `elements` へ追加する。
    ///
    /// 大梁は生成のたび `on_beam` へ ID を渡す（長期荷重の載荷対象として控える
    /// ベンチがあるため）。柱は渡さない。
    ///
    /// 局所座標の基準ベクトルは、柱が全体 X 方向・大梁が全体 Z 方向（鉛直上）。
    /// これは水平材・鉛直材それぞれの既定の ref_vector に一致する。
    pub fn push_frame_members(
        &self,
        elements: &mut Vec<ElementData>,
        column_section: SectionId,
        beam_section: SectionId,
        mut on_beam: impl FnMut(ElemId),
    ) {
        for iz in 0..self.nz {
            for iy in 0..=self.ny {
                for ix in 0..=self.nx {
                    push_beam_element(
                        elements,
                        self.node_id(ix, iy, iz),
                        self.node_id(ix, iy, iz + 1),
                        [1.0, 0.0, 0.0],
                        column_section,
                    );
                }
            }
        }
        for iz in 1..=self.nz {
            for iy in 0..=self.ny {
                for ix in 0..self.nx {
                    let id = push_beam_element(
                        elements,
                        self.node_id(ix, iy, iz),
                        self.node_id(ix + 1, iy, iz),
                        [0.0, 0.0, 1.0],
                        beam_section,
                    );
                    on_beam(id);
                }
            }
            for iy in 0..self.ny {
                for ix in 0..=self.nx {
                    let id = push_beam_element(
                        elements,
                        self.node_id(ix, iy, iz),
                        self.node_id(ix, iy + 1, iz),
                        [0.0, 0.0, 1.0],
                        beam_section,
                    );
                    on_beam(id);
                }
            }
        }
    }
}

/// 両端固定・剛域なし・ばねなしの梁要素を追加し、その ID を返す。
///
/// 要素 ID は `elements` の添字と一致させる（節点と同じ不変条件）。
pub fn push_beam_element(
    elements: &mut Vec<ElementData>,
    n0: NodeId,
    n1: NodeId,
    ref_vector: [f64; 3],
    section: SectionId,
) -> ElemId {
    let id = ElemId(elements.len() as u32);
    elements.push(ElementData {
        id,
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![n0, n1],
        section: Some(section),
        local_axis: LocalAxis { ref_vector },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    id
}

/// 柱・大梁の断面 2 種（S 造）。柱 H-400x400x13x21 が `SectionId(0)`、
/// 大梁 H-400x200x8x13 が `SectionId(1)`。いずれも材料は [`sn400_steel`]。
///
/// `eigen_bench`・`pushover_bench`・`th_bench` の 3 本が同じ建物を測るため、
/// 断面諸元も 3 本で共通である。`parallel_bench` だけは単一断面の別モデルを
/// 使うので、ここは共有しない（測っているものが違う）。
pub fn column_beam_sections() -> Vec<Section> {
    vec![
        Section {
            area: 21_870.0,
            iy: 6.6e8,
            iz: 6.6e8,
            j: 2.0e7,
            depth: 400.0,
            width: 400.0,
            as_y: 12_000.0,
            as_z: 12_000.0,
            material: Some(MaterialId(0)),
            ..Section::zero(SectionId(0), "柱 H-400x400x13x21".into())
        },
        Section {
            area: 8_412.0,
            iy: 2.34e8,
            iz: 2.34e8,
            j: 6.0e5,
            depth: 400.0,
            width: 200.0,
            as_y: 4_000.0,
            as_z: 4_000.0,
            material: Some(MaterialId(0)),
            ..Section::zero(SectionId(1), "梁 H-400x200x8x13".into())
        },
    ]
}

/// [`column_beam_sections`] が参照する鋼材 SN400（`MaterialId(0)`）。
///
/// 密度 0 として自重は考えない（ベンチは節点質量・階の地震用重量を直接与える）。
pub fn sn400_steel() -> Vec<Material> {
    vec![Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400".into(),
        category: MaterialCategory::Steel,
        young: 205_000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    }]
}
