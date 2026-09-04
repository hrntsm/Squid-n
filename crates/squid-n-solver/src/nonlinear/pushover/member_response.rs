//! 終局（最終確定ステップ）時の部材別応答の算定。
//!
//! - [`compute_member_response`] — 部材端内力を局所座標へ射影し、強軸・弱軸の
//!   設計用曲げ・せん断と軸圧縮力、部材変形角 Rp を [`PushoverMemberResponse`]
//!   として求める
//! - [`record_member_step`] — ヒンジ詳細図用に 1 確定ステップ分の部材端応答
//!   （軸力・剛域フェイスの局所曲げ・弦からの材端回転）を全部材について記録する

use super::geom::{axial_compression, dot3, member_end_forces_at_face};
use super::types::{MemberStepState, PushoverMemberResponse};
use squid_n_core::dof::DofMap;
use squid_n_core::model::{ElementData, Model};
use squid_n_element::behavior::{Ctx, ElementBehavior, LocalVec};
use squid_n_element::transform::LocalFrame;

/// 部材の変形角 R [rad]（弦回転角＝層間変形角相当）を節点変位から算定する。
///
/// - 鉛直材（柱系、|Δz| が水平成分より大きい）: 材端の水平相対変位 / 材長
///   （層間変形角に相当する近似）。
/// - 水平材（梁系）: 材端の鉛直相対変位 / 材長（弦回転角）。
///
/// `disp` は `DofMap` アクティブ添字順の全自由節点変位（プッシュオーバー
/// 最終ステップの `total_disp`）。
///
/// **既知の近似（保守側）:** この弦回転角は弾性変形・剛体回転成分を含む
/// 全回転角であり、靭性保証型耐震設計指針の塑性理論式が想定する塑性回転角
/// （降伏後の塑性成分のみ）に対して過大となる。Rp が大きいほど有効係数 ν・
/// トラス機構の cotφ がともに小さくなり終局せん断強度が下がるため、
/// 検定は安全側に出る。降伏時回転角の控除（θp = θ − θy）は
/// ヒンジ塑性回転の抽出を要するため将来課題とする。
pub(crate) fn member_rp_angle(
    model: &Model,
    dofmap: &DofMap,
    disp: &[f64],
    elem: &ElementData,
) -> f64 {
    if elem.nodes.len() < 2 {
        return 0.0;
    }
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    let (Some(pi), Some(pj)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
        return 0.0;
    };
    // 材長は `Model::member_length` が単一情報源（2 節点未満・節点参照の欠落は 0.0）。
    let length = model.member_length(elem);
    if length <= 0.0 {
        return 0.0;
    }
    let get = |node_index: usize, dof: usize| -> f64 {
        let g = node_index * 6 + dof;
        dofmap
            .active(g)
            .and_then(|a| disp.get(a as usize).copied())
            .unwrap_or(0.0)
    };
    // 全クレート共通の 45° 余弦基準（|ez| > 0.707）。柱系は水平変位/部材長
    // （層間変形角相当）、梁系は鉛直変位/部材長（たわみ角相当）で Rp を測る。
    let vertical = squid_n_core::geom::is_vertical_axis(pi.coord, pj.coord);
    if vertical {
        let dux = get(nj, 0) - get(ni, 0);
        let duy = get(nj, 1) - get(ni, 1);
        (dux * dux + duy * duy).sqrt() / length
    } else {
        (get(nj, 2) - get(ni, 2)).abs() / length
    }
}

/// 部材が伝達する加力方向の水平力 [N] を材端力から求める。
///
/// 材端力の載荷方向成分は釣合いにより「i 側節点群」と「j 側節点群」で符号が反転し、
/// 全節点の総和は 0 になる。2 節点の線材では各側 1 節点だが、**耐震壁（壁エレメント
/// モデル [`squid_n_element::wall`]）は 4 節点 24 自由度**で、節点配列は
/// `[下辺a, 下辺b, 上辺a, 上辺b]`。下辺の 2 節点は同じ向きにせん断を負担するため、
/// 節点群ごとに**合計**してから絶対値を取る必要がある。
///
/// 従来は `data[0..3]`（下辺a）と `data[6..9]`（下辺b）の最大値を取っており、
/// 4 節点壁では「下辺 2 節点の一方」だけを見る形になって水平力を約 1/2 に過小評価し、
/// 上辺 2 節点は無視していた（βu・壁の τu が過小＝ランク／Ds が甘くなる危険側）。
pub(crate) fn horizontal_force_in_dir(f: &LocalVec, n_nodes: usize, dir_idx: usize) -> f64 {
    let n = n_nodes.min(f.data.len() / 6);
    if n < 2 {
        return 0.0;
    }
    let half = n / 2;
    let sum = |range: std::ops::Range<usize>| -> f64 {
        range.map(|k| f.data[k * 6 + dir_idx]).sum::<f64>()
    };
    sum(0..half).abs().max(sum(half..n).abs())
}

/// ヒンジ詳細図用: 1 確定ステップ分の部材端応答（[`MemberStepState`]）を全部材に
/// ついて算定する（`model.elements` と同じ並び。2 節点の線材以外はゼロ埋め）。
///
/// 曲げは危険断面＝剛域フェイス位置の局所成分（[`member_end_forces_at_face`]）、
/// 回転は弦（変形後の材端を結ぶ直線）からの材端回転とする。弦からの相対回転を
/// 使うのは、材端ヒンジの M-θ 曲線では剛体回転（層間変形による部材全体の傾き）を
/// 除いた「ヒンジ部の回転」が意味を持つため。
pub(crate) fn record_member_step(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    total_disp: &[f64],
) -> Vec<MemberStepState> {
    let ctx = Ctx { model };
    model
        .elements
        .iter()
        .zip(behaviors)
        .map(|(elem, b)| {
            if elem.nodes.len() != 2 {
                return MemberStepState::default();
            }
            let ni = elem.nodes[0].index();
            let nj = elem.nodes[1].index();
            let (Some(pi), Some(pj)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
                return MemberStepState::default();
            };
            let dx = [
                pj.coord[0] - pi.coord[0],
                pj.coord[1] - pi.coord[1],
                pj.coord[2] - pi.coord[2],
            ];
            let length = (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]).sqrt();
            if length <= 0.0 {
                return MemberStepState::default();
            }
            let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
            let ex = frame.rot[0];
            let ey = frame.rot[1];
            let ez = frame.rot[2];

            // 剛域フェイス位置の局所曲げ（My=4/10・Mz=5/11）と軸力。
            let f = b.internal_force(&ctx);
            let (my_i, mz_i, my_j, mz_j) = match member_end_forces_at_face(model, elem, &f.data) {
                Some(fl) => (fl[4], fl[5], fl[10], fl[11]),
                None => {
                    let m_i = [f.data[3], f.data[4], f.data[5]];
                    let m_j = [f.data[9], f.data[10], f.data[11]];
                    (dot3(m_i, ey), dot3(m_i, ez), dot3(m_j, ey), dot3(m_j, ez))
                }
            };
            let f_i = [f.data[0], f.data[1], f.data[2]];
            let f_j = [f.data[6], f.data[7], f.data[8]];
            let n = axial_compression(f_i, f_j, ex);

            // 弦からの材端回転。局所たわみ v（ey 方向）・w（ez 方向）の弦回転を
            // 節点回転の局所成分から差し引く（x-y 面: θz−(v_j−v_i)/L、
            // x-z 面: θy+(w_j−w_i)/L。符号は Euler 梁の右手系規約）。
            let get = |node_index: usize, dof: usize| -> f64 {
                let g = node_index * 6 + dof;
                dofmap
                    .active(g)
                    .and_then(|a| total_disp.get(a as usize).copied())
                    .unwrap_or(0.0)
            };
            let u_i = [get(ni, 0), get(ni, 1), get(ni, 2)];
            let u_j = [get(nj, 0), get(nj, 1), get(nj, 2)];
            let r_i = [get(ni, 3), get(ni, 4), get(ni, 5)];
            let r_j = [get(nj, 3), get(nj, 4), get(nj, 5)];
            let chord_v = (dot3(u_j, ey) - dot3(u_i, ey)) / length;
            let chord_w = (dot3(u_j, ez) - dot3(u_i, ez)) / length;
            let ry_i = dot3(r_i, ey) + chord_w;
            let rz_i = dot3(r_i, ez) - chord_v;
            let ry_j = dot3(r_j, ey) + chord_w;
            let rz_j = dot3(r_j, ez) - chord_v;

            MemberStepState {
                n: n as f32,
                my_i: my_i as f32,
                mz_i: mz_i as f32,
                my_j: my_j as f32,
                mz_j: mz_j as f32,
                ry_i: ry_i as f32,
                rz_i: rz_i as f32,
                ry_j: ry_j as f32,
                rz_j: rz_j as f32,
            }
        })
        .collect()
}

/// 最終確定ステップの部材別応答（[`PushoverMemberResponse`]）を算定する。
///
/// 各部材の材端内力（`ElementBehavior::internal_force` のグローバル成分）を
/// 局所座標系（`LocalFrame`）へ射影し、強軸（局所 z まわり Mz・せん断 Vy）・
/// 弱軸（局所 y まわり My・せん断 Vz）の設計用応力と軸圧縮力、部材変形角 Rp を
/// 部材ごとに求める（曲げ・せん断は両端の最大絶対値）。
pub(crate) fn compute_member_response(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    total_disp: &[f64],
    dir: crate::statics::analysis::SeismicDir,
) -> Vec<PushoverMemberResponse> {
    let dir_idx = match dir {
        crate::statics::analysis::SeismicDir::X => 0usize,
        crate::statics::analysis::SeismicDir::Y => 1usize,
    };
    let ctx = Ctx { model };
    let mut out = Vec::with_capacity(model.elements.len());
    for (elem, b) in model.elements.iter().zip(behaviors) {
        if elem.nodes.len() < 2 {
            continue;
        }
        // 4 節点の耐震壁等（非 2 節点）: nodes[0]→nodes[1] を材軸とする線材向けの
        // 局所座標系は幾何的に無意味（壁脚の幅方向を向く）ため、曲げ・せん断・
        // 軸力・Rp は算定せず、βu の分子となる加力方向水平力のみ集計する
        // （[`horizontal_force_in_dir`] は節点群合計で 4 節点壁を正しく扱う）。
        if elem.nodes.len() != 2 {
            let f = b.internal_force(&ctx);
            out.push(PushoverMemberResponse {
                elem: elem.id,
                m_strong: 0.0,
                m_weak: 0.0,
                shear_strong: 0.0,
                shear_weak: 0.0,
                axial: 0.0,
                rp: 0.0,
                horizontal_force: horizontal_force_in_dir(&f, elem.nodes.len(), dir_idx),
            });
            continue;
        }
        let (Some(pi), Some(pj)) = (
            model.nodes.get(elem.nodes[0].index()),
            model.nodes.get(elem.nodes[1].index()),
        ) else {
            continue;
        };
        let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
        let ex = frame.rot[0];
        let ey = frame.rot[1];
        let ez = frame.rot[2];

        let f = b.internal_force(&ctx);
        let f_i = [f.data[0], f.data[1], f.data[2]];
        let f_j = [f.data[6], f.data[7], f.data[8]];

        // 設計用曲げは**危険断面＝剛域フェイス**で評価する（局所座標への射影＋
        // 剛体アームのモーメント控除。[`member_end_forces_at_face`]）。剛域がない
        // 部材では従来どおり局所成分への射影と一致する。せん断・軸力は剛体アームで
        // 変化しないため従来の射影のままとする。
        let (m_strong, m_weak) = match member_end_forces_at_face(model, elem, &f.data) {
            Some(fl) => (fl[5].abs().max(fl[11].abs()), fl[4].abs().max(fl[10].abs())),
            None => {
                let m_i = [f.data[3], f.data[4], f.data[5]];
                let m_j = [f.data[9], f.data[10], f.data[11]];
                (
                    dot3(m_i, ez).abs().max(dot3(m_j, ez).abs()),
                    dot3(m_i, ey).abs().max(dot3(m_j, ey).abs()),
                )
            }
        };
        let shear_strong = dot3(f_i, ey).abs().max(dot3(f_j, ey).abs());
        let shear_weak = dot3(f_i, ez).abs().max(dot3(f_j, ez).abs());
        let axial = axial_compression(f_i, f_j, ex);
        let rp = member_rp_angle(model, dofmap, total_disp, elem);
        // 加力方向の水平力（βu の分子・耐力壁の τu 算定用）。4 節点の耐震壁を含め
        // 正しく集計するため節点群ごとの合計で求める（[`horizontal_force_in_dir`]）。
        let horizontal_force = horizontal_force_in_dir(&f, elem.nodes.len(), dir_idx);

        out.push(PushoverMemberResponse {
            elem: elem.id,
            m_strong,
            m_weak,
            shear_strong,
            shear_weak,
            axial,
            rp,
            horizontal_force,
        });
    }
    out
}
