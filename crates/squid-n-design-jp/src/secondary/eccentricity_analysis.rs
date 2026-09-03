//! 偏心率の精算層（応力解析結果に基づく。令82条の6・昭55建告1792号）。
//!
//! [`crate::secondary::eccentricity`] のモデル抽出層（D値法の略算）に対し、本
//! モジュールは弾性応力解析結果から柱の水平剛性・長期軸力による重心を直接算定
//! する精算ルートを提供する。剛心・重心・ねじり剛性の合成自体は
//! [`crate::secondary::eccentricity`] の計算コア（`center_of_rigidity` /
//! `eccentricity` / `center_of_mass`）と雑壁剛性（`append_misc_wall_stiffnesses`）
//! をそのまま再利用する。

use squid_n_core::ids::StoryId;
use squid_n_core::model::Model;

use super::story_columns::story_columns;
use squid_n_element::transform::LocalFrame;
use squid_n_solver::statics::linear::StaticOnce;

use super::eccentricity::{
    append_misc_wall_stiffnesses, center_of_mass, center_of_rigidity, eccentricity,
    ColumnStiffness, Eccentricity,
};

/// 当該層に帰属する柱を列挙して `f(elem, 柱頭節点, 柱脚節点)` を呼ぶ。
///
/// 中間節点で分割された鉛直材は連なりとして 1 本とみなす（[`super::story_columns`]）。
/// `crate::secondary::eccentricity::sum_column_area`（雑壁剛性評価が必要とする
/// 柱断面積の集計）からも共用するため `pub(super)`（`secondary` 配下に公開）。
pub(super) fn for_each_story_column(
    model: &Model,
    story: StoryId,
    mut f: impl FnMut(
        &squid_n_core::model::ElementData,
        &squid_n_core::model::Node,
        &squid_n_core::model::Node,
    ),
) {
    for col in story_columns(model, story) {
        let Some(elem) = model.element(col.top_elem) else {
            continue;
        };
        let top = &model.nodes[col.top.index()];
        let bot = &model.nodes[col.bottom.index()];
        f(elem, top, bot);
    }
}

/// 部材始端（xi 最小の評価点）の局所内力 [N, Qy, Qz] を全体座標の力ベクトルへ
/// 変換して返す。`ref_vec` は要素の局所軸参照ベクトル。
fn station_force_global(
    p0: [f64; 3],
    p1: [f64; 3],
    ref_vec: [f64; 3],
    local: [f64; 6],
) -> [f64; 3] {
    let frame = LocalFrame::from_nodes(p0, p1, ref_vec);
    let ex = frame.rot[0];
    let ey = frame.rot[1];
    let ez = frame.rot[2];
    let (n, qy, qz) = (local[0], local[1], local[2]);
    [
        n * ex[0] + qy * ey[0] + qz * ez[0],
        n * ex[1] + qy * ey[1] + qz * ez[1],
        n * ex[2] + qy * ey[2] + qz * ez[2],
    ]
}

/// 地震時応力解析結果から柱の方向別水平剛性 `ki = Qi/δi` を算定する（精算層）。
/// 剛心はその階の柱の水平方向剛性の中心であり、各柱の水平剛性は地震時応力解析
/// 結果のせん断力と層間変位から算定する。
///
/// - `res_x` / `res_y`: X / Y 方向加力時の弾性応力解析結果
/// - `kX` は X 加力時の（せん断力 Qx, 層間変位 δx）から、`kY` は Y 加力時から。
///   δ がほぼ 0 の柱は剛性 0 とする。
pub fn column_stiffnesses_from_analysis(
    model: &Model,
    story: StoryId,
    res_x: &StaticOnce,
    res_y: &StaticOnce,
) -> Vec<ColumnStiffness> {
    use std::collections::HashMap;
    let fx: HashMap<_, _> = res_x.member_forces.iter().map(|(id, f)| (*id, f)).collect();
    let fy: HashMap<_, _> = res_y.member_forces.iter().map(|(id, f)| (*id, f)).collect();

    let mut out = Vec::new();
    for_each_story_column(model, story, |elem, top, bot| {
        let p0 = model.nodes[elem.nodes[0].index()].coord;
        let p1 = model.nodes[elem.nodes[1].index()].coord;
        let k_of = |res: &StaticOnce,
                    forces: &HashMap<
            squid_n_core::ids::ElemId,
            &squid_n_element::frame::beam::MemberForces,
        >,
                    dir: usize|
         -> f64 {
            let (Some(ut), Some(ub)) = (res.disp.get(top.id.index()), res.disp.get(bot.id.index()))
            else {
                return 0.0;
            };
            let delta = (ut[dir] - ub[dir]).abs();
            if delta < 1e-9 {
                return 0.0;
            }
            let Some(mf) = forces.get(&elem.id) else {
                return 0.0;
            };
            let Some(&(_, local)) = mf.at.first() else {
                return 0.0;
            };
            let g = station_force_global(p0, p1, elem.local_axis.ref_vector, local);
            g[dir].abs() / delta
        };
        out.push(ColumnStiffness {
            pos: [top.coord[0], top.coord[1]],
            dx: k_of(res_x, &fx, 0),
            dy: k_of(res_y, &fy, 1),
        });
    });
    out
}

/// 長期応力解析の柱軸力から重心を算定する。各階の重心は、鉛直荷重を支持する柱の
/// 長期荷重による軸力 N およびその部材の平面座標から `gx = Σ(Ni·xi)/ΣNi` として
/// 算定する。
///
/// 軸力は圧縮分を重みに用いる（引張柱は鉛直荷重を「支持」しないため重み 0）。
/// ΣNi ≤ 0（柱なし・全柱引張）の場合は `None`（呼び出し側で質量重心へフォールバック）。
pub fn center_of_gravity_from_axial(
    model: &Model,
    story: StoryId,
    long_term: &StaticOnce,
) -> Option<[f64; 2]> {
    use std::collections::HashMap;
    let forces: HashMap<_, _> = long_term
        .member_forces
        .iter()
        .map(|(id, f)| (*id, f))
        .collect();
    let mut sum_n = 0.0;
    let mut sum_nx = 0.0;
    let mut sum_ny = 0.0;
    for_each_story_column(model, story, |elem, top, _bot| {
        let Some(mf) = forces.get(&elem.id) else {
            return;
        };
        let Some(&(_, local)) = mf.at.first() else {
            return;
        };
        // 局所軸力 n は引張正 → 圧縮 = −n を重みに使う。
        let ni = (-local[0]).max(0.0);
        sum_n += ni;
        sum_nx += ni * top.coord[0];
        sum_ny += ni * top.coord[1];
    });
    if sum_n > 0.0 {
        Some([sum_nx / sum_n, sum_ny / sum_n])
    } else {
        None
    }
}

/// 応力解析結果に基づく当該層の偏心率（精算ルート）。
///
/// - 剛心: `column_stiffnesses_from_analysis`（ki = Qi/δi）
/// - 重心: `center_of_gravity_from_axial`（長期軸力。`long_term` がない/算定不能なら
///   質量重心 `center_of_mass` にフォールバック）
///
/// 偏心率は常に弾性解析結果から算定するため（令82条の6 の運用に基づく実務的
/// 取扱い）、`res_x`/`res_y` には弾性解析の結果を渡すこと。
pub fn story_eccentricity_from_analysis(
    model: &Model,
    story: StoryId,
    res_x: &StaticOnce,
    res_y: &StaticOnce,
    long_term: Option<&StaticOnce>,
) -> Eccentricity {
    let mut cols = column_stiffnesses_from_analysis(model, story, res_x, res_y);
    append_misc_wall_stiffnesses(model, story, &mut cols);
    let cor = center_of_rigidity(&cols);
    let com = long_term
        .and_then(|lt| center_of_gravity_from_axial(model, story, lt))
        .unwrap_or_else(|| center_of_mass(model, story));
    eccentricity(&cols, com, cor)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::secondary::eccentricity::test_support::build_symmetric_frame;
    use squid_n_core::ids::ElemId;
    use squid_n_element::frame::beam::MemberForces;

    /// 柱 4 本（ElemId 0..4、節点 i(底)→i+4(頂)）へ、指定の局所内力
    /// [n, qy, qz] を持つ member_forces と頂部一様変位 disp を合成する。
    /// 柱の ref_vector=[0,1,0] より ex=[0,0,1], ey=[0,1,0], ez=[1,0,0]。
    /// → 全体 X 方向せん断 = qz、全体 Y 方向せん断 = qy。
    fn fabricate_static(top_disp: [f64; 2], col_local_forces: [[f64; 3]; 4]) -> StaticOnce {
        let mut disp = vec![[0.0; 6]; 8];
        for d in disp.iter_mut().skip(4) {
            d[0] = top_disp[0];
            d[1] = top_disp[1];
        }
        let member_forces = col_local_forces
            .iter()
            .enumerate()
            .map(|(i, &[n, qy, qz])| {
                (
                    ElemId(i as u32),
                    MemberForces {
                        at: vec![(0.0, [n, qy, qz, 0.0, 0.0, 0.0])],
                    },
                )
            })
            .collect();
        StaticOnce {
            disp,
            member_forces,
            panel_moments: Vec::new(),
        }
    }

    /// ki = Qi/δi の精算: X 加力（δ=10, Qx=qz=1000 → kX=100）、
    /// Y 加力（δ=10, 左 qy=1000/右 qy=3000 → kY=100/300）→ 剛心 Xs=4500, Ys=3000。
    #[test]
    fn test_column_stiffnesses_from_analysis() {
        let (model, s0) = build_symmetric_frame(None);
        // xy = [(0,0), (6000,0), (0,6000), (6000,6000)]
        let res_x = fabricate_static([10.0, 0.0], [[0.0, 0.0, 1000.0]; 4]);
        let res_y = fabricate_static(
            [0.0, 10.0],
            [
                [0.0, 1000.0, 0.0],
                [0.0, 3000.0, 0.0],
                [0.0, 1000.0, 0.0],
                [0.0, 3000.0, 0.0],
            ],
        );
        let cols = column_stiffnesses_from_analysis(&model, s0, &res_x, &res_y);
        assert_eq!(cols.len(), 4);
        for c in &cols {
            assert!((c.dx - 100.0).abs() < 1e-9, "kX={}", c.dx);
        }
        let cor = center_of_rigidity(&cols);
        assert!((cor[0] - 4500.0).abs() < 1e-9, "Xs={}", cor[0]);
        assert!((cor[1] - 3000.0).abs() < 1e-9, "Ys={}", cor[1]);
    }

    /// 長期軸力による重心: 全柱等圧縮 → 幾何中央 (3000, 3000)。
    /// 右側 2 本を 3 倍圧縮 → gx = 4500。引張柱は重み 0。
    #[test]
    fn test_center_of_gravity_from_axial() {
        let (model, s0) = build_symmetric_frame(None);
        // 圧縮は n 負（引張正の符号規約）
        let uniform = fabricate_static([0.0, 0.0], [[-200.0, 0.0, 0.0]; 4]);
        let g = center_of_gravity_from_axial(&model, s0, &uniform).unwrap();
        assert!((g[0] - 3000.0).abs() < 1e-9 && (g[1] - 3000.0).abs() < 1e-9);

        let biased = fabricate_static(
            [0.0, 0.0],
            [
                [-100.0, 0.0, 0.0],
                [-300.0, 0.0, 0.0],
                [-100.0, 0.0, 0.0],
                [-300.0, 0.0, 0.0],
            ],
        );
        let g = center_of_gravity_from_axial(&model, s0, &biased).unwrap();
        assert!((g[0] - 4500.0).abs() < 1e-9, "gx={}", g[0]);

        // 全柱引張 → None（質量重心へフォールバックさせる）
        let tension = fabricate_static([0.0, 0.0], [[200.0, 0.0, 0.0]; 4]);
        assert!(center_of_gravity_from_axial(&model, s0, &tension).is_none());
    }

    /// 精算ルートの統合: 剛心 Xs=4500・重心 gx=3000 → ex=1500, Rey=ex/rey。
    #[test]
    fn test_story_eccentricity_from_analysis() {
        let (model, s0) = build_symmetric_frame(None);
        let res_x = fabricate_static([10.0, 0.0], [[0.0, 0.0, 1000.0]; 4]);
        let res_y = fabricate_static(
            [0.0, 10.0],
            [
                [0.0, 1000.0, 0.0],
                [0.0, 3000.0, 0.0],
                [0.0, 1000.0, 0.0],
                [0.0, 3000.0, 0.0],
            ],
        );
        let long_term = fabricate_static([0.0, 0.0], [[-200.0, 0.0, 0.0]; 4]);
        let ecc = story_eccentricity_from_analysis(&model, s0, &res_x, &res_y, Some(&long_term));
        assert!((ecc.ex - 1500.0).abs() < 1e-9, "ex={}", ecc.ex);
        assert!(ecc.ey.abs() < 1e-9, "ey={}", ecc.ey);
        // KR = ΣkX·ȳ² + ΣkY·x̄²
        //   ȳ = ±3000（kX=100×4）→ 100·9e6·4 = 3.6e9
        //   x̄ = x−4500 = [-4500,1500,-4500,1500] → 100·4500²+300·1500²（×2組）
        //     = 2·(100·2.025e7 + 300·2.25e6) = 2·(2.025e9+0.675e9) = 5.4e9
        let kr_expect = 3.6e9 + 5.4e9;
        assert!(
            (ecc.kr - kr_expect).abs() / kr_expect < 1e-12,
            "KR={}",
            ecc.kr
        );
        let rey = (kr_expect / 800.0_f64).sqrt();
        assert!((ecc.re_y - 1500.0 / rey).abs() < 1e-9, "Rey={}", ecc.re_y);
    }

    /// 中間節点で分割された柱は 1 本として数えること。
    #[test]
    fn test_column_stiffnesses_count_split_column_once() {
        use smallvec::SmallVec;
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{NodeId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node, RigidZone, Story,
        };

        let base = StoryId(0);
        let top = StoryId(1);
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: Some(base),
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 2000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(top),
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, 4000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(top),
                support_spring: None,
            },
        ];
        let mk_elem = |id: u32, n0: u32, n1: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: {
                let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                v.push(NodeId(n0));
                v.push(NodeId(n1));
                v
            },
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        };
        let model = Model {
            nodes,
            elements: vec![mk_elem(0, 1, 2), mk_elem(1, 0, 1)],
            stories: vec![
                Story {
                    id: base,
                    name: "1F".to_string(),
                    elevation: 0.0,
                    node_ids: vec![NodeId(0)],
                    seismic_weight: None,
                    weight_override: None,
                    structure: Default::default(),
                    level_kind: Default::default(),
                },
                Story {
                    id: top,
                    name: "2F".to_string(),
                    elevation: 4000.0,
                    node_ids: vec![NodeId(2)],
                    seismic_weight: None,
                    weight_override: None,
                    structure: Default::default(),
                    level_kind: Default::default(),
                },
            ],
            ..Default::default()
        };
        let mut disp_x = vec![[0.0; 6]; 3];
        disp_x[2][0] = 10.0; // 上端
        disp_x[0][0] = 0.0; // 下端
        let res_x = StaticOnce {
            disp: disp_x,
            member_forces: vec![(
                ElemId(0),
                MemberForces {
                    at: vec![(0.0, [0.0, 0.0, 1000.0, 0.0, 0.0, 0.0])],
                },
            )],
            panel_moments: Vec::new(),
        };
        let mut disp_y = vec![[0.0; 6]; 3];
        disp_y[2][1] = 10.0;
        let res_y = StaticOnce {
            disp: disp_y,
            member_forces: vec![(
                ElemId(0),
                MemberForces {
                    at: vec![(0.0, [0.0, 1000.0, 0.0, 0.0, 0.0, 0.0])],
                },
            )],
            panel_moments: Vec::new(),
        };
        let cols = column_stiffnesses_from_analysis(&model, top, &res_x, &res_y);
        assert_eq!(cols.len(), 1, "分割柱は 1 本と数える");
        assert!((cols[0].dx - 100.0).abs() < 1e-9);
    }
}
