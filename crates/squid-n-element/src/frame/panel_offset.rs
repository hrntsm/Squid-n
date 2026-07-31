//! 仕口パネルに接合する部材の適合（資料 2.10.2・2.10.3）。
//!
//! 仕口パネルが設けられた節点では、部材は節点そのものではなく、パネル寸法分だけ
//! 離れた「仕口パネルと部材の接合位置」で接合する。節点の変位（せん断変形角
//! `{γX, γY}` を含む 8 成分）と部材端の変位（6 成分）は次式で適合する。
//!
//! ```text
//! {d} = {D} + [B0]{Φ} + [Btp]{S}
//! {φ} = {Φ} + [Bp]{S}
//!
//! [B0] = [ 0  Z0 −Y0]        [Btp] = 1/2 [  0  Z0]      [Bp] = [ζ 0]
//!        [−Z0  0  X0]                    [−Z0   0]             [0 ζ]
//!        [ Y0 −X0  0]                    [−Y0  X0]             [0 0]
//! ```
//!
//! `{X0, Y0, Z0}` は節点から接合位置までのオフセット、`ζ` は部材が仕口パネルの
//! どの面で接合するかで決まる係数で、水平材（はり）は `−0.5`、鉛直材（柱）は
//! `+0.5` とする。
//!
//! # 既存の剛域変換との関係
//!
//! `[B0]{Φ} = Φ × r`（`r = {X0,Y0,Z0}`）は剛体アームによる並進-回転結合そのもので、
//! [`crate::rigid_arm`] の剛域変換が既に実装している。さらに、上記 3 行列の間には
//!
//! ```text
//! [Btp]{S} = ([Bp]{S}) × r
//! ```
//!
//! という恒等関係が成り立つ（`[Btp]` の第 3 行の符号は、はり `ζ = −0.5`・
//! 柱 `ζ = +0.5` の双方でこの関係が成立するように定められている）。したがって
//! `{B}` 全体は「回転を `Φ' = Φ + [Bp]{S}` に置き換えたうえで、オフセット `r` の
//! 剛体アームを適用したもの」に等しい。
//!
//! ```text
//! d = D + Φ'×r = D + [B0]{Φ} + ([Bp]{S})×r = D + [B0]{Φ} + [Btp]{S}   ✓
//! φ = Φ'       = {Φ} + [Bp]{S}                                        ✓
//! ```
//!
//! よって本モジュールは、
//!
//! 1. パネル分のオフセットを部材の**剛域長**へ含めて内側の要素を組む
//!    （剛体アーム `r` は既存の剛域変換が担う）
//! 2. 節点の回転自由度へ `ζ・γ` を加える変換 `T = [I | C]` を被せる
//!
//! の 2 段で資料の `[B]` を厳密に再現する。オフセットの大きさには、剛域自動算定が
//! 既に求めている柱フェース距離（`RigidZone::face_i` / `face_j` ＝ 接続する直交
//! 部材せいの 1/2）を用いる。これにより剛域端と危険断面位置が一致し、資料
//! 表 2.10.1 の「部材端・剛域端・危険断面位置」の整合が取れる。
//!
//! # 適用対象
//!
//! 水平材（はり）と鉛直材（柱）のみを対象とする。斜材（ブレース等）は資料が
//! 接合位置・`ζ` を定めていないため、従来どおり節点で接合する（パネル自由度と
//! 連成させない）。

use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, ElementKind, Model};

/// 部材軸の鉛直成分がこの値以上なら柱（鉛直材）とみなす。
const COLUMN_EZ: f64 = 0.8;
/// 部材軸の鉛直成分がこの値以下なら梁（水平材）とみなす。
const BEAM_EZ: f64 = 0.2;

/// 水平材（はり）が仕口パネルへ接合するときの ζ。
const ZETA_BEAM: f64 = -0.5;
/// 鉛直材（柱）が仕口パネルへ接合するときの ζ。
const ZETA_COLUMN: f64 = 0.5;

/// 部材の一方の端が仕口パネルへ接合することを表す。
#[derive(Clone, Copy, Debug)]
pub struct PanelEnd {
    /// パネルが設けられた節点（追加自由度 `γX`・`γY` の持ち主）。
    pub node: NodeId,
    /// 接合面で決まる係数 ζ（はり `−0.5`・柱 `+0.5`）。
    pub zeta: f64,
}

/// 部材 `data` の各端が仕口パネルへ接合するかを調べる。
///
/// 戻り値は `(パネル分のオフセットを剛域長へ含めた ElementData, [i 端, j 端])`。
/// どちらの端もパネルへ接合しない場合は `None`（従来どおり素の要素を組む）。
pub fn resolve(data: &ElementData, model: &Model) -> Option<(ElementData, [Option<PanelEnd>; 2])> {
    if !matches!(data.kind, ElementKind::Beam) || data.nodes.len() < 2 {
        return None;
    }
    let p0 = model.nodes.get(data.nodes[0].index())?.coord;
    let p1 = model.nodes.get(data.nodes[1].index())?.coord;
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if l < 1e-12 {
        return None;
    }
    // 斜材はオフセット・ζ が定義されないため対象外。
    let ez = (d[2] / l).abs();
    let zeta = if ez >= COLUMN_EZ {
        ZETA_COLUMN
    } else if ez <= BEAM_EZ {
        ZETA_BEAM
    } else {
        return None;
    };

    let is_panel_node = |nid: NodeId| {
        model
            .elements
            .iter()
            .any(|e| matches!(e.kind, ElementKind::PanelZone) && e.nodes.first() == Some(&nid))
    };

    let ends = [
        is_panel_node(data.nodes[0]).then(|| PanelEnd {
            node: data.nodes[0],
            zeta,
        }),
        is_panel_node(data.nodes[1]).then(|| PanelEnd {
            node: data.nodes[1],
            zeta,
        }),
    ];
    if ends[0].is_none() && ends[1].is_none() {
        return None;
    }

    // パネル分のオフセット（＝柱フェース距離）を剛域長へ含める。既に剛域長が
    // フェース距離以上ある端（RC/SRC の剛域）は変えない。
    let mut adjusted = data.clone();
    if ends[0].is_some() {
        adjusted.rigid_zone.length_i = adjusted.rigid_zone.length_i.max(data.rigid_zone.face_i);
    }
    if ends[1].is_some() {
        adjusted.rigid_zone.length_j = adjusted.rigid_zone.length_j.max(data.rigid_zone.face_j);
    }
    Some((adjusted, ends))
}

/// 仕口パネルへ接合する部材。内側の要素（12 自由度）へパネルのせん断変形角を
/// 連成させる変換 `T = [I | C]` を被せる。
///
/// 自由度の並びは `[内側の 12 自由度, (γX, γY)_i?, (γX, γY)_j?]`。
/// `C` は節点回転自由度へ `ζ・γ` を加える成分のみを持つ（モジュール冒頭の
/// 「既存の剛域変換との関係」を参照）。
pub struct PanelOffsetMember {
    inner: Box<dyn ElementBehavior>,
    ends: [Option<PanelEnd>; 2],
}

impl PanelOffsetMember {
    /// `inner` は 12 自由度（節点 2 × 6）の部材要素であること。
    pub fn new(inner: Box<dyn ElementBehavior>, ends: [Option<PanelEnd>; 2]) -> Self {
        debug_assert_eq!(inner.n_dof(), 12, "仕口パネルを被せる要素は 12 自由度");
        Self { inner, ends }
    }

    /// パネル自由度の個数（0・2・4）。
    fn n_panel_dof(&self) -> usize {
        self.ends.iter().filter(|e| e.is_some()).count() * 2
    }

    /// `C`（12 × パネル自由度数）の非零成分を `(内側自由度, パネル自由度, 係数)` で列挙する。
    ///
    /// 節点回転 `ΘX`・`ΘY`（i 端は内側 3・4、j 端は 9・10）へ `ζ・γX`・`ζ・γY` を
    /// 加える。`ΘZ` は `[Bp]` の第 3 行が 0 のため寄与しない。
    fn coupling(&self) -> SmallVec<[(usize, usize, f64); 4]> {
        let mut out = SmallVec::new();
        let mut col = 12;
        for (side, end) in self.ends.iter().enumerate() {
            let Some(end) = end else { continue };
            let rot0 = if side == 0 { 3 } else { 9 };
            out.push((rot0, col, end.zeta));
            out.push((rot0 + 1, col + 1, end.zeta));
            col += 2;
        }
        out
    }

    /// 要素自由度の変位を内側 12 自由度の変位へ写す（`u_inner = T · u_elem`）。
    fn to_inner(&self, u: &[f64]) -> [f64; 12] {
        let mut inner = [0.0; 12];
        for (i, slot) in inner.iter_mut().enumerate() {
            *slot = u.get(i).copied().unwrap_or(0.0);
        }
        for (r, c, z) in self.coupling() {
            inner[r] += z * u.get(c).copied().unwrap_or(0.0);
        }
        inner
    }

    /// 内側 12×12 の行列を要素自由度へ写す（`K_elem = Tᵀ · K_inner · T`）。
    fn transform_matrix(&self, k: &LocalMat) -> LocalMat {
        let n = self.n_dof();
        let mut out = LocalMat::zeros(n);
        // 左上 12×12 は内側そのまま。
        for i in 0..12 {
            for j in 0..12 {
                out.set(i, j, k.get(i, j));
            }
        }
        let cpl = self.coupling();
        // K·C（右上）と Cᵀ·K（左下）。
        for &(r, c, z) in &cpl {
            for i in 0..12 {
                let v = out.get(i, c) + k.get(i, r) * z;
                out.set(i, c, v);
                let v = out.get(c, i) + z * k.get(r, i);
                out.set(c, i, v);
            }
        }
        // Cᵀ·K·C（右下）。上のループで左下・右上を書き換えた後の値ではなく、
        // 内側 `k` から直接組む。
        for &(r1, c1, z1) in &cpl {
            for &(r2, c2, z2) in &cpl {
                let v = out.get(c1, c2) + z1 * k.get(r1, r2) * z2;
                out.set(c1, c2, v);
            }
        }
        out
    }
}

impl ElementBehavior for PanelOffsetMember {
    fn n_dof(&self) -> usize {
        12 + self.n_panel_dof()
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = self.inner.global_dofs(dof);
        for end in self.ends.iter().flatten() {
            let ni = end.node.index();
            for d in 0..2 {
                gdofs.push(dof.panel_dof(ni, d).map_or(usize::MAX, |a| a as usize));
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, state: &ElemState, ctx: &Ctx) -> LocalMat {
        self.transform_matrix(&self.inner.tangent_stiffness(state, ctx))
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        self.transform_matrix(&self.inner.geometric_stiffness(n))
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        self.transform_matrix(&self.inner.mass_matrix(opt))
    }

    fn internal_force(&self, state: &ElemState, ctx: &Ctx) -> LocalVec {
        // f_elem = Tᵀ · f_inner。パネル自由度には ζ·（節点回転まわりのモーメント）が
        // 集まり、パネル要素の内力と釣り合う（資料 (2.10.3-3)）。
        let f_inner = self.inner.internal_force(state, ctx);
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, self.n_dof()),
        };
        for (i, v) in f_inner.data.iter().take(12).enumerate() {
            f.data[i] = *v;
        }
        for (r, c, z) in self.coupling() {
            f.data[c] += z * f_inner.data.get(r).copied().unwrap_or(0.0);
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, ctx: &Ctx) {
        let inner = self.to_inner(&du.data);
        let du_inner = LocalVec {
            data: SmallVec::from_slice(&inner),
        };
        self.inner.update_state(&du_inner, commit, ctx);
    }

    fn commit_state(&mut self) {
        self.inner.commit_state();
    }

    fn revert_state(&mut self) {
        self.inner.revert_state();
    }

    fn snapshot_state(&self) -> Box<dyn std::any::Any> {
        self.inner.snapshot_state()
    }

    fn restore_state(&mut self, state: &dyn std::any::Any) {
        self.inner.restore_state(state);
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        self.inner.serialize_checkpoint()
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        self.inner.deserialize_checkpoint(data)
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        self.inner.recover_forces(&self.to_inner(u_elem))
    }

    fn state_member_forces(
        &self,
        state: &ElemState,
        ctx: &Ctx,
    ) -> Option<crate::beam::MemberForces> {
        // 内側は自身のトライアル変位（パネル寄与を反映済み）を保持している。
        self.inner.state_member_forces(state, ctx)
    }

    fn ductility_probe(&self) -> Option<crate::behavior::DuctilityProbe> {
        self.inner.ductility_probe()
    }

    fn fiber_section_states(&self) -> Option<Vec<crate::behavior::FiberSectionState>> {
        self.inner.fiber_section_states()
    }

    fn set_time_step(&mut self, dt: f64) {
        self.inner.set_time_step(dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        EndCondition, ForceRegime, LocalAxis, Material, Node, RigidZone, Section,
    };
    use squid_n_core::section_shape::SectionShape;

    /// 柱 1 本＋梁 1 本、接合部（節点 0）に仕口パネルを持つモデル。
    fn model_with_panel(face: f64) -> Model {
        let node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let sec = Section {
            id: SectionId(0),
            name: String::new(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 400.0,
            width: 400.0,
            as_y: 5.0e3,
            as_z: 5.0e3,
            panel_thickness: None,
            thickness: None,
            shape: Some(SectionShape::SteelH {
                height: 400.0,
                width: 400.0,
                web_thick: 13.0,
                flange_thick: 21.0,
            }),
        };
        let member = |id: u32, n0: u32, n1: u32, rigid: RigidZone| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: rigid,
            plastic_zone: None,
            spring: None,
        };
        let rigid = RigidZone {
            face_i: face,
            face_j: face,
            ..Default::default()
        };
        Model {
            nodes: vec![
                node(0, [0.0, 0.0, 3000.0]),
                node(1, [6000.0, 0.0, 3000.0]),
                node(2, [0.0, 0.0, 0.0]),
            ],
            sections: vec![sec],
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
            elements: vec![
                member(0, 0, 1, rigid), // 梁（水平材）
                member(1, 2, 0, rigid), // 柱（鉛直材）
                ElementData {
                    id: ElemId(2),
                    kind: ElementKind::PanelZone,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2)],
                    section: None,
                    material: None,
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 1.0, 0.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
            ],
            ..Default::default()
        }
    }

    /// 水平材は ζ = −0.5、鉛直材は ζ = +0.5。パネルが付く端だけが対象になる。
    #[test]
    fn test_resolve_assigns_zeta_by_member_direction() {
        let model = model_with_panel(200.0);
        let (_, beam_ends) = resolve(&model.elements[0], &model).expect("梁の i 端がパネル");
        assert_eq!(beam_ends[0].map(|e| e.zeta), Some(ZETA_BEAM));
        assert!(beam_ends[1].is_none(), "j 端（節点 1）にパネルは無い");

        let (_, col_ends) = resolve(&model.elements[1], &model).expect("柱の j 端がパネル");
        assert!(col_ends[0].is_none(), "i 端（節点 2）にパネルは無い");
        assert_eq!(col_ends[1].map(|e| e.zeta), Some(ZETA_COLUMN));
    }

    /// パネル分のオフセット（＝柱フェース距離）が剛域長へ含まれる。
    /// これにより剛域端と危険断面位置が一致する（資料 表 2.10.1 の整合）。
    #[test]
    fn test_resolve_folds_face_distance_into_rigid_zone() {
        let model = model_with_panel(200.0);
        let (adjusted, _) = resolve(&model.elements[0], &model).expect("梁");
        assert!((adjusted.rigid_zone.length_i - 200.0).abs() < 1e-9);
        assert_eq!(
            adjusted.rigid_zone.length_j, 0.0,
            "パネルが無い端の剛域長は変えない"
        );
        // 危険断面位置（フェース距離）は幾何量なので変えない。
        assert_eq!(adjusted.rigid_zone.face_i, 200.0);
    }

    /// パネルが 1 つも無いモデルでは `None` を返し、従来どおり素の要素が組まれる。
    #[test]
    fn test_resolve_returns_none_without_panel() {
        let mut model = model_with_panel(200.0);
        model
            .elements
            .retain(|e| !matches!(e.kind, ElementKind::PanelZone));
        assert!(resolve(&model.elements[0], &model).is_none());
    }

    /// 斜材はオフセット・ζ が資料で定義されないため対象外とする。
    #[test]
    fn test_resolve_skips_diagonal_member() {
        let mut model = model_with_panel(200.0);
        // 節点 1 を斜め方向へ動かして梁を斜材にする。
        model.nodes[1].coord = [4000.0, 0.0, 6000.0];
        assert!(resolve(&model.elements[0], &model).is_none());
    }

    /// 変換 `T = [I | C]` の性質:
    /// - パネル自由度を 0 に固定すれば、剛性・内力は内側 12 自由度と完全に一致する
    /// - 対称性が保たれる（合同変換 Tᵀ K T）
    #[test]
    fn test_transform_preserves_inner_block_and_symmetry() {
        let model = model_with_panel(200.0);
        let ctx = Ctx { model: &model };
        let (adjusted, ends) = resolve(&model.elements[0], &model).expect("梁");
        let inner = crate::beam::BeamElement::new(&adjusted, &model);
        let k_inner = inner.tangent_stiffness(&ElemState::default(), &ctx);
        let wrapped = PanelOffsetMember::new(Box::new(inner), ends);

        assert_eq!(wrapped.n_dof(), 14, "12 ＋ パネル 2");
        let k = wrapped.tangent_stiffness(&ElemState::default(), &ctx);

        // 左上 12×12 は内側そのまま（パネル変形角 0 のとき従来と一致）。
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k.get(i, j) - k_inner.get(i, j)).abs()
                        <= 1e-6 * k_inner.get(i, j).abs().max(1.0),
                    "({i},{j}) が内側と一致しない"
                );
            }
        }
        // 合同変換なので対称性が保たれる。
        for i in 0..14 {
            for j in 0..14 {
                let (a, b) = (k.get(i, j), k.get(j, i));
                assert!(
                    (a - b).abs() <= 1e-6 * a.abs().max(1.0),
                    "非対称 ({i},{j}): {a} vs {b}"
                );
            }
        }
    }

    /// パネル自由度 γ に単位変形を与えると、内側の部材は節点回転 ζ·γ を受ける
    /// （`{φ} = {Φ} + [Bp]{S}`）。梁は −0.5·γ、柱は +0.5·γ。
    #[test]
    fn test_panel_rotation_enters_member_end_rotation() {
        let model = model_with_panel(200.0);
        let (adjusted, ends) = resolve(&model.elements[0], &model).expect("梁");
        let inner = crate::beam::BeamElement::new(&adjusted, &model);
        let wrapped = PanelOffsetMember::new(Box::new(inner), ends);

        let mut u = vec![0.0; wrapped.n_dof()];
        u[12] = 1.0; // γX
        let inner_u = wrapped.to_inner(&u);
        assert!((inner_u[3] - ZETA_BEAM).abs() < 1e-12, "ΘX へ ζ·γX");
        assert_eq!(inner_u[4], 0.0);

        let mut v = vec![0.0; wrapped.n_dof()];
        v[13] = 1.0; // γY
        let inner_v = wrapped.to_inner(&v);
        assert!((inner_v[4] - ZETA_BEAM).abs() < 1e-12, "ΘY へ ζ·γY");
        assert_eq!(inner_v[3], 0.0);
    }

    /// 内力は `f_elem = Tᵀ · f_inner`。パネル自由度には ζ×（節点モーメント）が集まる。
    /// これがパネル要素の内力と釣り合うことで資料 (2.10.3-3) が満たされる。
    #[test]
    fn test_internal_force_collects_panel_moment() {
        let model = model_with_panel(200.0);
        let ctx = Ctx { model: &model };
        let (adjusted, ends) = resolve(&model.elements[0], &model).expect("梁");
        let inner = crate::beam::BeamElement::new(&adjusted, &model);
        let mut wrapped = PanelOffsetMember::new(Box::new(inner), ends);

        // 梁 j 端（節点 1）に単位回転を与える。
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, wrapped.n_dof()),
        };
        du.data[11] = 1.0e-4;
        wrapped.update_state(&du, true, &ctx);

        let f = wrapped.internal_force(&ElemState::default(), &ctx);
        // i 端の ΘX・ΘY（内側 3・4）に対応するパネル自由度の値は ζ 倍。
        assert!((f.data[12] - ZETA_BEAM * f.data[3]).abs() <= 1e-6 * f.data[3].abs().max(1.0));
        assert!((f.data[13] - ZETA_BEAM * f.data[4]).abs() <= 1e-6 * f.data[4].abs().max(1.0));
    }
}
