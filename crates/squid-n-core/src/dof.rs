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

/// 仕口パネルが設けられた節点が追加で持つ自由度の数。
///
/// せん断変形角 `γX`・`γY`（基準座標系。X'-Z' 平面と Y'-Z' 平面のパネルせん断
/// 変形角）の 2 個。標準の 6 自由度とは別枠で、[`DofMap`] のグローバル自由度
/// 空間の末尾（`節点数 × DOF_PER_NODE` の後ろ）へ払い出す。
///
/// この置き方にすることで、`節点番号 × DOF_PER_NODE + 成分` でグローバル自由度を
/// 求める既存コードは追加自由度に一切触れず、パネルを持たないモデルでは追加
/// 自由度が 1 つも払い出されないため剛性行列・独立自由度数が従来と完全に一致する。
pub const PANEL_DOF_PER_NODE: usize = 2;

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

/// 仕口パネル（`ElementKind::PanelZone`）が設けられた節点を、節点 index ごとの
/// 真偽で返す。パネル要素の先頭節点（`nodes[0]`）が接合部の節点である。
///
/// 該当する節点は [`PANEL_DOF_PER_NODE`] 個の追加自由度（せん断変形角）を持つ。
pub fn panel_zone_nodes(model: &Model) -> Vec<bool> {
    let mut is_panel = vec![false; model.nodes.len()];
    for e in &model.elements {
        if !matches!(e.kind, crate::model::ElementKind::PanelZone) {
            continue;
        }
        if let Some(n) = e.nodes.first() {
            if let Some(slot) = is_panel.get_mut(n.index()) {
                *slot = true;
            }
        }
    }
    is_panel
}

#[derive(Clone, Debug, Default)]
pub struct DofMap {
    active_of: Vec<Option<u32>>,
    global_of: Vec<GlobalDof>,
    n_active: usize,
    /// 節点 index → 仕口パネル自由度のスロット番号。パネルを持たない節点は `None`。
    /// スロット `s` の `d` 番目のグローバル自由度は
    /// `n_node_global + s * PANEL_DOF_PER_NODE + d`。
    panel_slot_of: Vec<Option<u32>>,
    /// スロット番号 → 節点 index（[`Self::panel_slot_of`] の逆写像）。
    panel_node_of: Vec<u32>,
    /// 標準自由度（節点 × 6）の総数。仕口パネル自由度のグローバル番号はここから始まる。
    n_node_global: usize,
}

impl DofMap {
    pub fn build(model: &Model) -> Self {
        // 構造節点（解析自由度を持つ節点）以外は全自由度を不活性にする
        // （解析上は存在しない扱い。変位は 0 で出力され、そこへの節点荷重は
        // 無視される。荷重は同期側で主架構へ変換する規約）。判定規則は
        // [`structural_nodes`] を参照。
        let structural = structural_nodes(model);
        let is_panel = panel_zone_nodes(model);

        // 仕口パネル自由度は標準自由度の後ろへ連続して並べる。パネルが 1 つも
        // なければ `n_panel_slots == 0` となり、以降は従来と完全に同一の写像になる。
        let n_node_global = model.nodes.len() * DOF_PER_NODE;
        let mut panel_slot_of = vec![None; model.nodes.len()];
        let mut panel_node_of = Vec::new();
        for (ni, &p) in is_panel.iter().enumerate() {
            // 構造節点でない節点にパネルは付かない（パネル要素が接続していれば
            // その節点は必ず構造節点になるため、通常この分岐は成立しない）。
            if p && structural[ni] {
                panel_slot_of[ni] = Some(panel_node_of.len() as u32);
                panel_node_of.push(ni as u32);
            }
        }

        let n_global = n_node_global + panel_node_of.len() * PANEL_DOF_PER_NODE;
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
        // 仕口パネル自由度は `Node::restraint`（6 成分のマスク）の対象外であり、
        // 拘束する手段を持たない。パネル要素が必ず剛性 `Kxp`・`Kyp` を与えるため
        // 零剛性にはならず、常に活性としてよい。
        for (ni, slot) in panel_slot_of.iter().enumerate() {
            let Some(s) = slot else { continue };
            let _ = ni;
            for d in 0..PANEL_DOF_PER_NODE {
                let g = n_node_global + *s as usize * PANEL_DOF_PER_NODE + d;
                active_of[g] = Some(counter);
                global_of.push(g);
                counter += 1;
            }
        }
        DofMap {
            active_of,
            global_of,
            n_active: counter as usize,
            panel_slot_of,
            panel_node_of,
            n_node_global,
        }
    }

    pub fn n_active(&self) -> usize {
        self.n_active
    }
    pub fn active(&self, g: GlobalDof) -> Option<u32> {
        self.active_of.get(g).copied().flatten()
    }

    /// 自由 DOF 空間のベクトル（`active` 添字順。従属自由度は `expand_u` 済み）を
    /// 節点×6 成分の配列へ展開する。拘束・非構造自由度は 0 のまま。
    ///
    /// 静的解析の変位・時刻歴の節点変位・固有モード形状の散布で同一の展開が
    /// 必要になるため、単一実装としてここに置く（各ソルバでの手書きコピーの
    /// 再発防止）。
    pub fn expand_to_nodes(&self, u_free: &[f64], n_nodes: usize) -> Vec<[f64; 6]> {
        let mut out = vec![[0.0f64; 6]; n_nodes];
        for (ni, d6) in out.iter_mut().enumerate() {
            for (d, slot) in d6.iter_mut().enumerate() {
                if let Some(a) = self.active(ni * DOF_PER_NODE + d) {
                    *slot = u_free[a as usize];
                }
            }
        }
        out
    }
    pub fn global(&self, a: u32) -> GlobalDof {
        self.global_of[a as usize]
    }

    /// 節点 `node_idx` の仕口パネル自由度（`d = 0` が γX、`1` が γY）の独立自由度番号。
    /// パネルを持たない節点・範囲外は `None`。
    pub fn panel_dof(&self, node_idx: usize, d: usize) -> Option<u32> {
        let slot = (*self.panel_slot_of.get(node_idx)?)? as usize;
        if d >= PANEL_DOF_PER_NODE {
            return None;
        }
        self.active(self.n_node_global + slot * PANEL_DOF_PER_NODE + d)
    }

    /// 節点 `node_idx` に仕口パネル自由度が払い出されているか。
    pub fn has_panel_dof(&self, node_idx: usize) -> bool {
        self.panel_slot_of
            .get(node_idx)
            .is_some_and(|s| s.is_some())
    }

    /// グローバル自由度 `g` が標準自由度（節点 × 6）の範囲にあるか。
    /// `false` は仕口パネルの追加自由度を指す（`g / DOF_PER_NODE` で節点番号へ
    /// 換算してはならない）。
    pub fn is_node_dof(&self, g: GlobalDof) -> bool {
        g < self.n_node_global
    }

    /// グローバル自由度 `g` が仕口パネルの追加自由度であれば
    /// `(節点 index, 成分（0 = γX, 1 = γY）)` を返す。標準自由度・範囲外は `None`。
    pub fn panel_dof_ref(&self, g: GlobalDof) -> Option<(usize, usize)> {
        let off = g.checked_sub(self.n_node_global)?;
        let slot = off / PANEL_DOF_PER_NODE;
        let d = off % PANEL_DOF_PER_NODE;
        Some((*self.panel_node_of.get(slot)? as usize, d))
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

    /// 仕口パネルが 1 つもないモデルでは追加自由度が払い出されず、独立自由度数・
    /// 写像とも従来（節点 × 6）と完全に一致する（既存モデルの回帰防止）。
    #[test]
    fn test_no_panel_keeps_dof_map_identical() {
        let model = make_model_with_restraints(&[Dof6Mask::FREE; 3]);
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 3 * DOF_PER_NODE);
        for ni in 0..3 {
            assert!(!map.has_panel_dof(ni));
            assert!(map.panel_dof(ni, 0).is_none());
        }
        // 全独立自由度が標準自由度（節点 × 6）の範囲に収まる。
        for a in 0..map.n_active() {
            assert!(map.is_node_dof(map.global(a as u32)));
        }
    }

    /// 仕口パネル要素がある節点には γX・γY の 2 自由度が追加され、標準自由度の
    /// 後ろへ連続して並ぶ。標準自由度側の番号は従来と変わらない。
    #[test]
    fn test_panel_node_gets_two_extra_dofs() {
        let mut model = make_model_with_restraints(&[Dof6Mask::FREE; 3]);
        let base = DofMap::build(&model).n_active();

        // 節点 1 に仕口パネルを設ける（先頭節点が接合部の節点）。
        model.elements.push(ElementData {
            id: ElemId(100),
            kind: ElementKind::PanelZone,
            nodes: [NodeId(1), NodeId(0), NodeId(2)].into_iter().collect(),
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

        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), base + PANEL_DOF_PER_NODE);
        assert!(map.has_panel_dof(1));
        assert!(!map.has_panel_dof(0) && !map.has_panel_dof(2));

        // パネル自由度は標準自由度の後ろ（＝既存の番号を押し出さない）。
        let gx = map.panel_dof(1, 0).expect("γX");
        let gy = map.panel_dof(1, 1).expect("γY");
        assert_eq!(gy, gx + 1);
        assert!(gx as usize >= base);
        assert!(map.panel_dof(1, 2).is_none(), "成分は 2 個まで");

        // 逆写像で節点・成分が引ける（増分解析の特異診断が使う）。
        assert!(!map.is_node_dof(map.global(gx)));
        assert_eq!(map.panel_dof_ref(map.global(gx)), Some((1, 0)));
        assert_eq!(map.panel_dof_ref(map.global(gy)), Some((1, 1)));
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
        model.constraints.push(Constraint::rigid_diaphragm(
            StoryId(0),
            NodeId(2),
            vec![NodeId(0), NodeId(1)],
        ));
        let map = DofMap::build(&model);
        assert_eq!(map.n_active(), 18, "拘束マスターは自由度を持つ");
    }
}
