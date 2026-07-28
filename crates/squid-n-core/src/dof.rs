use crate::model::Model;

#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum Dof {
    Ux = 0,
    Uy = 1,
    Uz = 2,
    Rx = 3,
    Ry = 4,
    Rz = 5,
}

pub const DOF_PER_NODE: usize = 6;

#[derive(Clone, Copy, Default, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub struct Dof6Mask(pub u8);

impl Dof6Mask {
    pub const FREE: Self = Dof6Mask(0b000000);
    pub const FIXED: Self = Dof6Mask(0b111111);
    pub const PINNED: Self = Dof6Mask(0b000111);
    pub fn is_fixed(self, d: Dof) -> bool {
        self.0 & (1 << d as u8) != 0
    }
    pub fn set_fixed(&mut self, d: Dof) {
        self.0 |= 1 << d as u8;
    }
    /// 指定自由度の拘束を解除（ビットを下ろす）。
    pub fn set_free(&mut self, d: Dof) {
        self.0 &= !(1 << d as u8);
    }
    /// 指定自由度の拘束を ON/OFF で設定する。
    pub fn set(&mut self, d: Dof, fixed: bool) {
        if fixed {
            self.set_fixed(d);
        } else {
            self.set_free(d);
        }
    }
}

pub type GlobalDof = usize;

/// 解析自由度を持つ節点（**構造節点**）の判定を節点 index ごとの真偽で返す。
///
/// 構造節点 = 要素（部材）が接続する節点、または拘束（剛床・剛リンク・MPC）の
/// マスター節点。どちらでもない節点（二次部材（小梁・間柱）の支持点・床境界専用の
/// 幾何節点など）は剛性が一切組み上がらず零剛性の自由度＝特異行列の原因になるため、
/// [`DofMap::build`] が全自由度を不活性にする（解析上は存在しない扱い）。
///
/// 解析（[`DofMap::build`]）と表示（解析対象外の節点を描かない・剛床スレーブから
/// 除く）で同じ規則を使うため、判定をここへ一元化する。
pub fn structural_nodes(model: &Model) -> Vec<bool> {
    let mut structural = vec![false; model.nodes.len()];
    for e in &model.elements {
        for n in &e.nodes {
            if let Some(slot) = structural.get_mut(n.index()) {
                *slot = true;
            }
        }
    }
    for c in &model.constraints {
        use crate::model::Constraint;
        match c {
            Constraint::RigidDiaphragm { master, .. } | Constraint::RigidLink { master, .. } => {
                if let Some(slot) = structural.get_mut(master.index()) {
                    *slot = true;
                }
            }
            // MPC は `master` フィールドがスレーブ節点、`terms` がマスター側。
            Constraint::Mpc { terms, .. } => {
                for (n, _, _) in terms {
                    if let Some(slot) = structural.get_mut(n.index()) {
                        *slot = true;
                    }
                }
            }
        }
    }
    structural
}

#[derive(Clone, Debug, Default)]
pub struct DofMap {
    active_of: Vec<Option<u32>>,
    global_of: Vec<GlobalDof>,
    n_active: usize,
}

impl DofMap {
    pub fn build(model: &Model) -> Self {
        // 構造節点（解析自由度を持つ節点）以外は全自由度を不活性にする
        // （解析上は存在しない扱い。変位は 0 で出力され、そこへの節点荷重は
        // 無視される。荷重は同期側で主架構へ変換する規約）。判定規則は
        // [`structural_nodes`] を参照。
        let structural = structural_nodes(model);

        let n_global = model.nodes.len() * DOF_PER_NODE;
        let mut active_of = vec![None; n_global];
        let mut global_of = Vec::new();
        let mut counter = 0u32;
        for (ni, node) in model.nodes.iter().enumerate() {
            if !structural[ni] {
                continue;
            }
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                let dof = match d {
                    0 => Dof::Ux,
                    1 => Dof::Uy,
                    2 => Dof::Uz,
                    3 => Dof::Rx,
                    4 => Dof::Ry,
                    _ => Dof::Rz,
                };
                if !node.restraint.is_fixed(dof) {
                    active_of[g] = Some(counter);
                    global_of.push(g);
                    counter += 1;
                }
            }
        }
        DofMap {
            active_of,
            global_of,
            n_active: counter as usize,
        }
    }

    pub fn n_active(&self) -> usize {
        self.n_active
    }
    pub fn active(&self, g: GlobalDof) -> Option<u32> {
        self.active_of[g]
    }
    pub fn global(&self, a: u32) -> GlobalDof {
        self.global_of[a as usize]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dof::Dof6Mask;
    use crate::ids::*;
    use crate::model::*;

    fn make_model_with_restraints(restraints: &[Dof6Mask]) -> Model {
        let nodes: Vec<Node> = restraints
            .iter()
            .enumerate()
            .map(|(i, &r)| Node {
                id: NodeId(i as u32),
                coord: [i as f64 * 1000.0, 0.0, 0.0],
                restraint: r,
                mass: None,
                story: None,
                support_spring: None,
            })
            .collect();
        // 要素が接続しない節点は解析自由度から除外されるため、拘束マスキングの
        // 検証用に全節点を鎖状の梁要素でつなぐ（1 節点のみの場合は自己参照でよい）。
        let elements: Vec<ElementData> = (0..restraints.len().max(2) - 1)
            .map(|i| ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: [
                    NodeId(i as u32),
                    NodeId(((i + 1) % restraints.len()) as u32),
                ]
                .into_iter()
                .collect(),
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
            })
            .collect();
        Model {
            nodes,
            elements,
            ..Default::default()
        }
    }

    #[test]
    fn test_set_free_and_set_toggle() {
        let mut m = Dof6Mask::FIXED;
        m.set_free(Dof::Ux);
        assert!(!m.is_fixed(Dof::Ux));
        assert!(m.is_fixed(Dof::Uy));
        // set(false) は解除、set(true) は拘束
        m.set(Dof::Uy, false);
        assert!(!m.is_fixed(Dof::Uy));
        m.set(Dof::Ux, true);
        assert!(m.is_fixed(Dof::Ux));
        // PINNED から Rz を拘束すると並進3 + Rz が拘束される
        let mut p = Dof6Mask::PINNED;
        p.set(Dof::Rz, true);
        assert!(p.is_fixed(Dof::Ux) && p.is_fixed(Dof::Uy) && p.is_fixed(Dof::Uz));
        assert!(p.is_fixed(Dof::Rz));
        assert!(!p.is_fixed(Dof::Rx) && !p.is_fixed(Dof::Ry));
    }

    #[test]
    fn test_all_free() {
        let model = make_model_with_restraints(&[Dof6Mask::FREE; 3]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 18);
    }

    #[test]
    fn test_one_fixed() {
        let model = make_model_with_restraints(&[Dof6Mask::FREE, Dof6Mask::FIXED, Dof6Mask::FREE]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 12);
    }

    #[test]
    fn test_all_fixed() {
        let model = make_model_with_restraints(&[Dof6Mask::FIXED]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 0);
    }

    #[test]
    fn test_pinned() {
        let model = make_model_with_restraints(&[Dof6Mask::PINNED]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 3);
    }

    #[test]
    fn test_mixed() {
        let model = make_model_with_restraints(&[Dof6Mask::FREE, Dof6Mask::PINNED]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 6 + 3);
    }

    /// 要素が接続しない節点（二次部材の支持点など）は解析自由度から除外される。
    /// 拘束（剛床）のマスター節点は要素非接続でも自由度を持つ。
    #[test]
    fn test_unreferenced_node_is_inactive() {
        let mut model = make_model_with_restraints(&[Dof6Mask::FREE, Dof6Mask::FREE]);
        // 要素が接続しない自由節点を追加 → 自由度は増えない。
        model.nodes.push(Node {
            id: NodeId(2),
            coord: [500.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 12, "孤立自由節点は自由度を持たない");
        assert!(map.active(2 * DOF_PER_NODE).is_none());

        // 剛床マスターに指定すると自由度を持つ（拘束されない DOF 分）。
        model.constraints.push(Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(2),
            slaves: vec![NodeId(0), NodeId(1)],
        });
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 18, "拘束マスターは自由度を持つ");
    }
}
