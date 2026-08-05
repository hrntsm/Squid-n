//! 幾何ヘルパ。
//!
//! - [`dot3`] — 3 次元ベクトルの内積
//! - [`axial_compression`] — 材端力から部材の軸方向圧縮力を算定
//! - [`member_end_forces_at_face`] — 材端力を局所座標・剛域フェイス位置へ変換

use squid_n_core::model::{ElementData, Model};
use squid_n_element::transform::LocalFrame;

pub(crate) fn dot3(v: [f64; 3], w: [f64; 3]) -> f64 {
    v[0] * w[0] + v[1] * w[1] + v[2] * w[2]
}

/// 材端力（グローバル、i端 `f_i`・j端 `f_j`）と局所 `ex`（i→j 方向単位ベクトル、
/// グローバル成分）から、部材の軸方向圧縮力 N_compress [N]（圧縮のみ採用、
/// 引張は 0）を算定する（精緻化2、σ0 = N_compress/gross_area の入力）。
///
/// ## 符号規約（単純片持ち柱による検算）
/// 標準的なトラス/梁要素の軸剛性行列は局所座標で
/// `[[EA/L, -EA/L], [-EA/L, EA/L]]`（i端・j端の軸方向 DOF）であり、
/// 引張正のひずみ `eps0 = (u_j − u_i)/L` に対し軸力 `N = EA・eps0`（引張正）
/// を生じる。この行列を軸方向変位 `(u_i, u_j)` に適用すると、i端の局所x方向
/// 内力は `f_local_x(i) = -N`、j端は `f_local_x(j) = +N` となる
/// （`squid-n-element` の `FiberBeam`・`Beam` とも同一の規約。剛性行列／
/// B行列の符号から導出、要素実装のいずれでも一致）。
///
/// 具体例（節点 i=(0,0,0)・節点 j=(0,0,3000)、`ref_vector=[1,0,0]` の片持ち柱、
/// `LocalFrame::from_nodes` により `ex=[0,0,1]`）で軸圧縮を検算する: 柱頭（j端）
/// を Δ=-1mm だけ ex 方向と逆向き（縮む向き）に変位させると、局所x方向変位は
/// `u_i=0, u_j=dot(Δ,ex)=-1`。ひずみ `eps0=(u_j-u_i)/L=-1/L<0`（圧縮）となり
/// `N=EA・eps0<0`。よって `f_local_x(i)=-N>0`、`f_local_x(j)=N<0`。
///
/// `ElementBehavior::internal_force` はグローバル力を返す契約であり、
/// 局所x軸（`ex`）方向の内力成分はグローバル内力を `ex` へ射影すれば得られる
/// （`AxisTransform::rotate_to_local` の定義 `v_local[0]=dot(ex,v_global)` より）。
/// よって `dot(f_i, ex) = f_local_x(i) = -N`、`dot(f_j, ex) = f_local_x(j) = +N`。
///
/// 圧縮（N<0）成分のみ正の値として取り出すため、i端は `dot(f_i, ex)` を、
/// j端は `-dot(f_j, ex)` を、それぞれ 0 未満をクランプ（引張は 0 とみなす）して
/// 採用し、両端のうち大きい方を部材の代表圧縮力とする（安全側の丸めではなく
/// 実勢値を採る規約。プリズマティック部材で軸方向分布荷重がなければ理論上
/// 両端は一致するが、数値誤差・分布荷重の影響を考慮し大きい方を採用する）。
pub(crate) fn axial_compression(f_i: [f64; 3], f_j: [f64; 3], ex: [f64; 3]) -> f64 {
    let from_i = dot3(f_i, ex).max(0.0);
    let from_j = (-dot3(f_j, ex)).max(0.0);
    from_i.max(from_j)
}

/// 部材の材端力（グローバル）を**局所座標・剛域フェイス位置**の 12 成分へ変換する。
///
/// `ElementBehavior::internal_force` はグローバル系・節点位置の材端力を返す。
/// 断面の降伏判定（曲げヒンジ）や終局時の設計用応力は**危険断面＝剛域フェイス**で
/// 評価するため、
///
/// 1. 局所座標系（`LocalFrame`）へ回転し、
/// 2. 剛体アームのモーメント（アーム長 × 材端せん断）を差し引く
///    （[`squid_n_element::rigid_arm::to_flex_force`]）
///
/// の 2 段で変換する。剛域がない部材では 2. は恒等となり、局所成分への回転のみ。
///
/// 節点位置のモーメントは剛体アーム分だけ大きいため、これを断面耐力 My と直接
/// 比較すると剛域を持つ部材のヒンジを過早に検出し、崩壊荷重を過小評価する。
///
/// **2 節点の線材専用。** 4 節点の耐震壁（壁エレメントモデル。節点配列は
/// `[下辺a, 下辺b, 上辺a, 上辺b]`）に適用すると、nodes[0]→nodes[1] は壁脚の
/// 幅方向（水平）を向き、壁要素本体の等価柱軸（壁脚中心→壁頭中心、鉛直）と
/// ほぼ直交した無意味な局所座標系になるため `None` を返して対象外とする
/// （壁のせん断終局・曲げは壁専用の経路で扱う）。
pub(crate) fn member_end_forces_at_face(
    model: &Model,
    elem: &ElementData,
    f_global: &[f64],
) -> Option<[f64; 12]> {
    if elem.nodes.len() != 2 || f_global.len() < 12 {
        return None;
    }
    let pi = model.nodes.get(elem.nodes[0].index())?;
    let pj = model.nodes.get(elem.nodes[1].index())?;
    let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
    let mut g = [0.0_f64; 12];
    g.copy_from_slice(&f_global[..12]);
    let local = frame.rotate_to_local(&g);
    let d = [
        pj.coord[0] - pi.coord[0],
        pj.coord[1] - pi.coord[1],
        pj.coord[2] - pi.coord[2],
    ];
    let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let (li, lj) = squid_n_element::rigid_arm::resolve_lengths(
        elem.rigid_zone.rigid_length_i(),
        elem.rigid_zone.rigid_length_j(),
        length,
    );
    Some(squid_n_element::rigid_arm::to_flex_force(&local, li, lj))
}
