//! 部材ローカルに沿った応力図（内力分布図）の描画。
//!
//! 6 成分 `[N, Qy, Qz, Mx, My, Mz]` を個別に ON/OFF でき、選んだ成分をすべて
//! 同時に描く。図は要素ローカル軸方向（曲げ平面内）へワールド空間で張り出して
//! から投影する。スクリーン上の部材線の法線ではなくワールドの要素軸を使うため、
//! ビューを回転しても図の張り出し面は要素座標系に追従する（曲げ平面を真横から
//! 見ると線に潰れる）。張り出し面は成分で決まり、強軸側（N・Qy・Mx・Mz）は
//! ローカル y 面、弱軸側（Qz・My）はローカル z 面へ出るため、6 成分同時でも
//! 直交 2 面に分かれて重なりが減る。
//!
//! ## 成分ごとの独立正規化と符号
//!
//! N[N] と M[N·mm] は桁も単位も異なるため、正規化（60px 相当の最大値）・
//! カラーバー・数値ラベルは**成分ごとに独立**に扱う。凡例・ラベルの表示単位は
//! kN・kN·m とする。
//!
//! 断面力の符号規約は `Qy = dMz/dx`・`Qz = −dMy/dx` で My だけせん断との関係が
//! 反転しているため、「正のモーメントを引張側へ張り出す」規約を保つには My の
//! 張り出しだけ符号を反転させる（[`ForceComponent::plot_sign`]）。数値ラベルと
//! コンター色は張り出し値ではなく内力値そのものを使う。
//!
//! ## 塗りつぶしの方式（凸多角形の扇状分割問題への対策）
//!
//! `egui::Shape::convex_polygon` は頂点0からの扇状三角形分割で塗りつぶすため、
//! 「材軸→各張り出し点→材軸」を 1 つの閉多角形として渡すと、M 図のように非凸・
//! 符号反転（両端 hogging＋中央 sagging 等）を含む形状では誤った領域が塗られる。
//! これを避けるため、隣接サンプル区間ごとに「材軸上の2点＋張り出し点2点」の
//! 台形（符号反転区間はゼロ交差で2つの三角形に分割）を個別に塗る
//! （[`diagram_fill_polygons`]）。台形・三角形は必ず凸なので扇状分割の問題が出ない。
//!
//! 輪郭（実線）は閉じない `egui::Shape::line` の折れ線として描く。塗り＋輪郭を
//! 1 つの閉じたポリゴン（Stroke 付き）にする方式だと、値が極小の部材で図形が
//! 材軸上にほぼ潰れ、材軸と張り出し線が浅い角度で接する折り返し点で epaint の
//! マイター結合が発散し、部材軸方向へ画面外まで伸びるスパイク描画になる
//! （CMQ 図の [`super::paint_diagram_polygon`] と同じ問題・同じ対策）。
//!
//! ## M 図の曲線補間
//!
//! 断面力のサンプル位置は危険断面（節点芯・柱フェース・部材中央 ほか）の数点
//! しかないため、そのまま直線で結ぶと等分布荷重の放物線が失われる。M 図だけは
//! せん断を勾配とする 3 次エルミート補間で区間内を稠密化する（Mz は `dMz/dx = Qy`、
//! My は張り出し値 `−My` に対して `d(−My)/dx = Qz`。[`moment_curve_samples`]）。

use crate::app::App;
use crate::theme;

use super::{
    diagram_offset_dir, member_len3, BeamDeflection, ForceComponent, ForceComponents, Projector,
};

/// 張り出しピークがこの px 未満の図形は描かない。60px 正規化に対して値が
/// 相対的に極小の部材（ほぼ潰れた図形）は、輪郭の折り返し点で epaint のマイター
/// 結合が発散し部材軸方向に画面外まで伸びるスパイク描画になるため、視認不能な
/// 図形は端から描かずスキップする。N/Q/M 図・CMQ 図で共有する。
pub(super) const MIN_DIAGRAM_PX: f32 = 0.5;

/// 輪郭の折れ線で、直前の点とのスクリーン距離がこの px 未満の連続点は間引く
/// （ゼロ長セグメントも epaint のマイター結合発散の原因になるため）。
/// N/Q/M 図・CMQ 図で共有する。
pub(super) const MIN_SEGMENT_PX: f32 = 0.25;

/// コンター表示時、各サンプル区間を細分する分割数（滑らかな色階調のため）。
const CONTOUR_SUBDIV: usize = 8;

/// M 図の曲線補間で 1 サンプル区間を分割する点数（[`moment_curve_samples`]）。
const MOMENT_CURVE_DIV: usize = 8;

/// M 図のサンプル `(xi, Mz, Qy)` を、区間ごとの 3 次エルミート補間で稠密化する
/// （純粋関数。戻り値は `(xi, Mz)` の xi 昇順列）。
///
/// 断面内力は材軸に沿って `dMz/dx = Qy` を満たす（`squid_n_element` の断面内力
/// 規約）ため、正規化座標 ξ では `dMz/dξ = Qy·L` が区間端の勾配になる。両端の
/// 値と勾配を与える 3 次エルミート補間は、M が 3 次以下の多項式となる区間
/// ——無載荷（1 次）・等分布荷重（2 次）・等変分布荷重（3 次）——では厳密解に
/// 一致する。評価断面（危険断面）は既定で節点芯・柱フェース・部材中央の数点
/// しかないため、直線で結ぶと等分布荷重の放物線が失われるが、この補間で
/// 評価断面の値を保ったまま区間内の曲率が復元される。
///
/// ## 中間集中荷重が評価断面上にある場合の扱い
///
/// 評価断面 ξ_k に中間集中荷重（小梁反力など）が載ると、その断面の Qy は
/// **左側極限**になる（固定端内力の自由体が半開区間 `[0, x)` のため）。この値は
/// 区間 `[ξ_k, ξ_{k+1}]` の左端勾配としては誤りで、そのまま用いると本来は
/// 折れ線である M 図に架空のふくらみが出る。そこで、区間が 2 次（＝等分布）で
/// あるとして弦勾配 `Sc` と右端勾配 `S1` から定まる左端勾配 `S0* = 2·Sc − S1` を
/// 求め、実測の左端勾配と符号が食い違う場合は `S0*` を採用する。このとき補間は
/// 2 次へ縮退し、中央集中荷重のように M が折れ線になる区間で厳密解に一致する。
/// 区間の右端勾配は常に区間内側からの極限であり、この補正は要らない。
pub(crate) fn moment_curve_samples(
    samples: &[(f64, f64, f64)],
    length: f64,
    div: usize,
) -> Vec<(f64, f64)> {
    let div = div.max(1);
    if samples.len() < 2 {
        return samples.iter().map(|&(xi, m, _)| (xi, m)).collect();
    }
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut out: Vec<(f64, f64)> = Vec::with_capacity((sorted.len() - 1) * div + 1);
    out.push((sorted[0].0, sorted[0].1));
    for w in sorted.windows(2) {
        let (x0, m0, q0) = w[0];
        let (x1, m1, q1) = w[1];
        let h = x1 - x0;
        if h <= 1e-12 {
            // 同一位置の重複サンプル。補間せず値だけ引き継ぐ
            out.push((x1, m1));
            continue;
        }
        // 区間端の dMz/dξ（せん断 × 部材長）
        let s1 = q1 * length;
        let sc = (m1 - m0) / h; // 弦勾配
        let s0_raw = q0 * length;
        let s0_quad = 2.0 * sc - s1; // 2 次と仮定したときの左端勾配
        let s0 = if s0_raw * s0_quad < 0.0 {
            s0_quad // 集中荷重による勾配の不連続とみなす（上記ドキュメント参照）
        } else {
            s0_raw
        };
        let (d0, d1) = (s0 * h, s1 * h);
        for k in 1..=div {
            let t = k as f64 / div as f64;
            let (t2, t3) = (t * t, t * t * t);
            let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
            let h10 = t3 - 2.0 * t2 + t;
            let h01 = -2.0 * t3 + 3.0 * t2;
            let h11 = t3 - t2;
            out.push((x0 + h * t, h00 * m0 + h10 * d0 + h01 * m1 + h11 * d1));
        }
    }
    out
}

/// `(xi, val)` のサンプル列から、塗りつぶし用の凸ポリゴン列を作る（純粋関数）。
///
/// 隣接サンプル区間ごとに `subdiv` 分割し、各細分区間を「材軸上の2点（val=0）＋
/// 張り出し点2点」の台形として返す。区間内で符号が反転する場合は、線形補間で
/// ゼロ交差 `xc = x0 + (x1-x0)*v0/(v0-v1)` を求めて2つの三角形（それぞれ片側符号
/// のみ）に分割する。戻り値の各ポリゴンは xi 昇順の点列で、値の符号は単一
/// （片側符号のみ、または境界でちょうど 0）になる。
///
/// `samples` は呼び出し側の保険として関数内で xi 昇順にソートする。
pub(crate) fn diagram_fill_polygons(samples: &[(f64, f64)], subdiv: usize) -> Vec<Vec<(f64, f64)>> {
    let subdiv = subdiv.max(1);
    if samples.len() < 2 {
        return Vec::new();
    }
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut polys = Vec::new();
    for w in sorted.windows(2) {
        let (x0, v0) = w[0];
        let (x1, v1) = w[1];
        for k in 0..subdiv {
            let ta = k as f64 / subdiv as f64;
            let tb = (k + 1) as f64 / subdiv as f64;
            let xa = x0 + (x1 - x0) * ta;
            let xb = x0 + (x1 - x0) * tb;
            let va = v0 + (v1 - v0) * ta;
            let vb = v0 + (v1 - v0) * tb;
            push_segment_fill(xa, va, xb, vb, &mut polys);
        }
    }
    polys
}

/// 1 細分区間 `(x0,v0)-(x1,v1)` の塗りポリゴンを `out` へ積む。
/// 同符号（またはどちらか一方が 0）なら台形 1 つ、符号反転ならゼロ交差で
/// 2 つの三角形に分割する。両端が 0（面積なし）の場合は何も積まない。
fn push_segment_fill(x0: f64, v0: f64, x1: f64, v1: f64, out: &mut Vec<Vec<(f64, f64)>>) {
    if v0 == 0.0 && v1 == 0.0 {
        return;
    }
    if (v0 >= 0.0 && v1 >= 0.0) || (v0 <= 0.0 && v1 <= 0.0) {
        out.push(vec![(x0, 0.0), (x0, v0), (x1, v1), (x1, 0.0)]);
    } else {
        // 符号反転: 線形補間でゼロ交差を求め、片側符号のみの三角形2つに分割する
        let xc = x0 + (x1 - x0) * v0 / (v0 - v1);
        out.push(vec![(x0, 0.0), (x0, v0), (xc, 0.0)]);
        out.push(vec![(xc, 0.0), (x1, v1), (x1, 0.0)]);
    }
}

/// ポリゴン（[`diagram_fill_polygons`] の1要素）の代表値（中点値）。
/// 材軸上の点（val=0）を除く頂点の値の平均。コンター着色の基準に使う。
fn diagram_poly_repr_val(poly: &[(f64, f64)]) -> f64 {
    let nz: Vec<f64> = poly
        .iter()
        .map(|&(_, v)| v)
        .filter(|v| v.abs() > 1e-15)
        .collect();
    if nz.is_empty() {
        0.0
    } else {
        nz.iter().sum::<f64>() / nz.len() as f64
    }
}

/// コンター配色: `t = val/max_abs ∈ [-1,1]`（範囲外はクランプ）を `map` の色へ写像する。
/// TONMANUAL §3「カラーマップ（連続値）」は Viridis を既定に定めており（[`theme::ColorMap`]
/// の `#[default]`）、UI からは他のカラーマップにも切り替えられる。実体は
/// `map.sample((t+1)/2)` への単純な写像で、独自の配色は持たない（テーマ＝配色の
/// 単一情報源は theme.rs 側に置く）。
pub(crate) fn contour_color(t: f64, map: theme::ColorMap) -> egui::Color32 {
    let t = (t as f32).clamp(-1.0, 1.0);
    map.sample((t + 1.0) * 0.5)
}

/// 成分ごとの最大絶対値のうち、「実質ゼロ」ではなく図を描く意味がある成分か。
///
/// 応力図は**成分ごとに独立正規化**する（60px ＝ その成分のモデル全体最大）ため、
/// 全部材で理論上ゼロの成分は、残った丸め誤差がそのまま最大値になり
/// **誤差が画面いっぱいの図として描かれる**。実際、i 端ねじれを解放したモデルの
/// Mx は理論上どの部材も 0 だが、静縮約 \\( K^* = K_{aa} - K_{ab}K_{bb}^{-1}K_{ba} \\)
/// の桁落ちで \\( 10^{-12}\,\mathrm{N\cdot mm} \\) 程度が残る。この値は絶対値の
/// しきい値では他成分と共通に判別できない（モデルの単位系・規模で何桁も変わる）。
///
/// そこで**同じ次元の成分の最大値**（力 `[N, Qy, Qz]` ／ モーメント `[Mx, My, Mz]`）を
/// 基準にした相対値で判定する。丸め誤差は基準に対して \\( 10^{-12} \\) 以下の桁に
/// 収まる一方、設計上意味を持つ内力は最小でも \\( 10^{-6} \\) 程度の比を持つため、
/// しきい値 \\( 10^{-9} \\) は両者を安全に分離する。同次元の成分がすべてゼロの
/// モデル（無載荷など）では何も描かない。
fn is_significant(maxes: &[f64; 6], c: ForceComponent) -> bool {
    /// 同次元成分の最大値に対する相対しきい値。
    const REL_TOL: f64 = 1e-9;
    let idx = c.force_index();
    // 内力ベクトルの並び `[N, Qy, Qz, Mx, My, Mz]`: 前半が力、後半がモーメント。
    let group = if idx < 3 { &maxes[0..3] } else { &maxes[3..6] };
    let ref_max = group.iter().fold(0.0_f64, |m, v| m.max(*v));
    ref_max > 0.0 && maxes[idx] > ref_max * REL_TOL
}

/// 部材ローカルに沿って応力図を描く。
///
/// `components` が ON にした成分をすべて重ねて描き、成分ごとに
/// **独立の最大値で正規化**する（N[N] と M[N·mm] は桁が異なるため 1 つの最大値を
/// 共有できない）。張り出し面は成分で決まり（[`ForceComponent::plane`]）、
/// 強軸側（N・Qy・Mx・Mz）は局所 ey 面、弱軸側（Qz・My）は局所 ez 面へ出るため、
/// 6 成分同時でも直交 2 面に分かれて重なりが減る。理論上ゼロの成分は描かない
/// （[`is_significant`]）。
pub(super) fn draw_force_diagram(
    painter: &egui::Painter,
    app: &App,
    components: ForceComponents,
    coords3: &[[f64; 3]],
    disp: Option<&[[f64; 6]]>,
    deform_scale: f64,
    proj: &Projector,
) {
    let Some(results) = &app.results else {
        return;
    };
    // 成分ごとの最大絶対値（正規化・凡例・数値ラベルの間引きに共通で使う）。
    let mut maxes = [0.0_f64; 6];
    for (_, mf) in &results.member_forces {
        for (_, f) in &mf.at {
            for (m, v) in maxes.iter_mut().zip(f.iter()) {
                *m = m.max(v.abs());
            }
        }
    }

    let mut legend_rows: Vec<(ForceComponent, f64)> = Vec::new();
    for c in components.selected() {
        // 実質ゼロの成分（例: 平面フレームの Qz、i 端ねじれ解放時の Mx）は図を
        // 描けない。丸め誤差を最大値として正規化すると誤差が画面いっぱいに
        // 描かれてしまうため、最大値も 0 に丸めて凡例へ渡す（[`is_significant`]）。
        // 成分が選択中であること自体は凡例で示す（「選んだのに何も出ない」理由が
        // 分かるようにする）。
        let max_abs = if is_significant(&maxes, c) {
            maxes[c.force_index()]
        } else {
            0.0
        };
        if max_abs > 0.0 {
            draw_component(
                painter,
                app,
                c,
                max_abs,
                coords3,
                disp,
                deform_scale,
                proj,
                results,
            );
        }
        legend_rows.push((c, max_abs));
    }
    draw_force_legend(
        painter,
        &legend_rows,
        app.diagram_contour,
        app.contour_colormap,
    );
}

/// 1 成分ぶんの応力図を全部材について描く（[`draw_force_diagram`] の本体）。
#[allow(clippy::too_many_arguments)]
fn draw_component(
    painter: &egui::Painter,
    app: &App,
    component: ForceComponent,
    max_abs: f64,
    coords3: &[[f64; 3]],
    disp: Option<&[[f64; 6]]>,
    deform_scale: f64,
    proj: &Projector,
    results: &crate::app::ResultsBundle,
) {
    let scale = proj.scale();
    let force_idx = component.force_index();
    let plot_sign = component.plot_sign();
    // 最大値で 60px 相当のワールド長（一様スケール正射影なので px/scale=ワールド長）
    let amp_world = 60.0 / max_abs / scale as f64;

    let contour = app.diagram_contour;
    let colormap = app.contour_colormap;
    // 塗りの不透明度: コンターは色そのものが情報を持つためモノクロより濃くする。
    let fill_alpha: u8 = if contour { 160 } else { 60 };
    // 輪郭は常に成分固定色。コンター中でも「どの図がどの成分か」を判別できる。
    let outline_color = component.color();
    let outline_width: f32 = if contour { 1.0 } else { 1.5 };

    for (elem_id, mf) in &results.member_forces {
        let elem = app.model.elements.iter().find(|e| e.id == *elem_id);
        let Some(elem) = elem else { continue };
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= coords3.len() || n1 >= coords3.len() {
            continue;
        }
        let p_i = coords3[n0];
        let p_j = coords3[n1];
        if member_len3(p_i, p_j) < 1e-9 {
            continue; // ゼロ長部材（同一節点間）は材軸が定まらず図を描けない
        }
        let ref_vec = elem.local_axis.ref_vector;
        let ey = diagram_offset_dir(p_i, p_j, ref_vec, component.plane());
        // 内部たわみ表示が有効な梁は、張り出しの基準線を変形後の Hermite 曲線に
        // する（`disp` が Some＝変形重ね時のみ）。梁の線描画と同じ `BeamDeflection`
        // で評価するため、基準線が梁の描画曲線に厳密一致する。それ以外（梁以外・
        // 内部たわみ OFF・変形重ね無し）は変形後の節点間直線（弦）を基準線にする
        // （従来どおり）。未変形材軸端点から一度だけ前処理する。
        let deflection: Option<BeamDeflection> =
            if app.show_beam_interpolation && elem.kind == squid_n_core::model::ElementKind::Beam {
                disp.and_then(|d| {
                    (n0 < d.len() && n1 < d.len()).then(|| {
                        BeamDeflection::new(
                            app.model.nodes[n0].coord,
                            app.model.nodes[n1].coord,
                            d[n0],
                            d[n1],
                            ref_vec,
                        )
                    })
                })
            } else {
                None
            };
        let p0 = proj.project(p_i);
        let p1 = proj.project(p_j);

        // M 図は曲げ材（梁・柱を表す線材要素）に限り、せん断を区間端の勾配とする
        // 3 次エルミートで曲線化する（等分布荷重の放物線を復元する。
        // [`moment_curve_samples`] 参照）。軸材・面要素・ばね類は曲げ内力場を
        // 持たないため対象外。N 図（一定）・Q 図（等分布下で 1 次）は評価断面を
        // 直線で結べば厳密なので従来どおり。
        let grad = component.moment_gradient_source().filter(|_| {
            matches!(
                elem.kind,
                squid_n_core::model::ElementKind::Beam
                    | squid_n_core::model::ElementKind::Fiber
                    | squid_n_core::model::ElementKind::MultiSpring
            )
        });
        // 張り出し値（= 内力値 × plot_sign）。補間・塗り・輪郭はこの値で行い、
        // 数値ラベル・コンター色は内力値そのもの（張り出し値 × plot_sign）を使う。
        let samples: Vec<(f64, f64)> = if let Some(q) = grad {
            // 勾配 d(張り出し値)/dξ = 対応せん断·L に用いる部材長は、変形倍率の
            // 影響を受けない未変形の節点間距離（内力回復時の材長）とする。
            let model_len = member_len3(app.model.nodes[n0].coord, app.model.nodes[n1].coord);
            let tri: Vec<(f64, f64, f64)> = mf
                .at
                .iter()
                .map(|(xi, f)| (*xi, plot_sign * f[force_idx], f[q.force_index()]))
                .collect();
            moment_curve_samples(&tri, model_len, MOMENT_CURVE_DIV)
        } else {
            let mut s: Vec<(f64, f64)> = mf
                .at
                .iter()
                .map(|(xi, f)| (*xi, plot_sign * f[force_idx]))
                .collect();
            s.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            s
        };
        if samples.len() < 2 {
            continue;
        }
        // コンター時は色の階調のため各区間を細分する。モノクロ時は単色なので不要。
        // 曲線補間済みの M 図は既に稠密なので、さらに細分はしない。
        let subdiv = if contour && grad.is_none() {
            CONTOUR_SUBDIV
        } else {
            1
        };
        // 張り出しピーク px が閾値未満の潰れた図形はスキップ（上記ドキュメント参照）
        let val_max = samples.iter().map(|(_, v)| v.abs()).fold(0.0_f64, f64::max);
        let peak_px = (60.0 * val_max / max_abs) as f32;
        if peak_px < MIN_DIAGRAM_PX {
            continue;
        }

        // (xi, val) → スクリーン座標。val=0 は基準線そのもの（オフセット無し）。
        // 基準線は deflection があれば梁の変形後 Hermite 曲線、無ければ節点間直線。
        let to_screen = |xi: f64, val: f64| -> egui::Pos2 {
            let base3 = match &deflection {
                Some(bd) => bd.point_at(xi, deform_scale),
                None => [
                    p_i[0] + (p_j[0] - p_i[0]) * xi,
                    p_i[1] + (p_j[1] - p_i[1]) * xi,
                    p_i[2] + (p_j[2] - p_i[2]) * xi,
                ],
            };
            if val == 0.0 {
                proj.project(base3)
            } else {
                proj.project_offset(base3, ey, -val * amp_world)
            }
        };

        // --- 塗り: 台形/三角形クワッドを個別に塗る（非凸・符号反転にも正しく対応） ---
        for poly in diagram_fill_polygons(&samples, subdiv) {
            let screen_poly: Vec<egui::Pos2> =
                poly.iter().map(|&(xi, v)| to_screen(xi, v)).collect();
            let fill_color = if contour {
                // 色は内力値そのもの（張り出し値ではない）で決める。
                let repr = plot_sign * diagram_poly_repr_val(&poly) / max_abs;
                theme::translucent(contour_color(repr, colormap), fill_alpha)
            } else {
                theme::translucent(outline_color, fill_alpha)
            };
            painter.add(egui::Shape::convex_polygon(
                screen_poly,
                fill_color,
                egui::Stroke::NONE,
            ));
        }

        // --- 輪郭: 閉じない折れ線（材軸点→各張り出し点→材軸点。マイター発散対策） ---
        let mut outline_pts: Vec<egui::Pos2> = Vec::with_capacity(samples.len() + 2);
        outline_pts.push(p0);
        let mut last = p0;
        for &(xi, val) in &samples {
            let pt = to_screen(xi, val);
            // 直前の点とスクリーン距離が近すぎるサンプル点は間引く
            if (pt.x - last.x).hypot(pt.y - last.y) < MIN_SEGMENT_PX {
                continue;
            }
            last = pt;
            outline_pts.push(pt);
        }
        outline_pts.push(p1);
        painter.add(egui::Shape::line(
            outline_pts,
            egui::Stroke::new(outline_width, outline_color),
        ));

        // --- 数値ラベル: 両端部と中央（ξ=0・0.5・1.0）---
        if app.diagram_values {
            draw_value_labels(painter, mf, component, max_abs, plot_sign, &to_screen);
        }
    }
}

/// 部材の両端部・中央（ξ=0・0.5・1.0）の値を、図の輪郭上へ成分固定色で描く。
///
/// 表示は kN・kN·m（内部単位 N・N·mm から換算）の小数 1 桁。値が小さい点は
/// ラベルが重なるだけで読めないため、**その成分のモデル全体最大の 1% 未満**は
/// 描かない（張り出しピークが極小の部材を端から描かない扱いと同じ方針）。
fn draw_value_labels(
    painter: &egui::Painter,
    mf: &squid_n_element::beam::MemberForces,
    component: ForceComponent,
    max_abs: f64,
    plot_sign: f64,
    to_screen: &dyn Fn(f64, f64) -> egui::Pos2,
) {
    const LABEL_XI: [f64; 3] = [0.0, 0.5, 1.0];
    let idx = component.force_index();
    let threshold = max_abs * 0.01;
    let font = egui::FontId::proportional(11.0);
    for xi_t in LABEL_XI {
        // 評価断面（危険断面）に ξ=0・0.5・1.0 は常に含まれる（`eval_sections_of`）。
        let Some((xi, f)) = mf
            .at
            .iter()
            .find(|(xi, _)| (*xi - xi_t).abs() < 1e-9)
            .map(|(xi, f)| (*xi, f))
        else {
            continue;
        };
        let value = f[idx];
        if value.abs() < threshold {
            continue;
        }
        let pos = to_screen(xi, plot_sign * value);
        // 張り出し先の外側へ少しずらして図と重ならないようにする。
        painter.text(
            pos,
            egui::Align2::CENTER_BOTTOM,
            format!("{:.1}", value * component.display_scale()),
            font.clone(),
            component.color(),
        );
    }
}

/// ビュー左上に応力図の凡例を描く。
///
/// 表示中の成分ごとに「色見本＋成分名＋最大値（kN・kN·m）」を 1 行ずつ並べ、
/// コンター表示中は各行の下にその成分のカラーバー（成分ごとの独立正規化）を
/// 添える。成分で単位も桁も異なるため、最大値・カラーバーは必ず成分ごとに出す。
fn draw_force_legend(
    painter: &egui::Painter,
    rows: &[(ForceComponent, f64)],
    contour: bool,
    colormap: theme::ColorMap,
) {
    const SWATCH: f32 = 10.0;
    const BAR_W: f32 = 160.0;
    const BAR_H: f32 = 10.0;
    const STRIPS: usize = 32;

    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let mut y = rect.min.y + 10.0;
    let font = egui::FontId::proportional(13.0);
    let small = egui::FontId::proportional(11.0);

    for &(c, max_abs) in rows {
        painter.rect_filled(
            egui::Rect::from_min_size(egui::pos2(x0, y + 2.0), egui::vec2(SWATCH, SWATCH)),
            2.0,
            c.color(),
        );
        // 実質ゼロの成分は最大値 0 で渡ってくる（`is_significant`）。数値だけだと
        // 「図が出ないのは不具合か」と迷うため、全部材でゼロである旨を明示する。
        let text = if max_abs > 0.0 {
            format!(
                "{}図 max={:.1} {}",
                c.label(),
                max_abs * c.display_scale(),
                c.unit()
            )
        } else {
            format!("{}図 全部材ゼロ（図なし）", c.label())
        };
        painter.text(
            egui::pos2(x0 + SWATCH + 6.0, y),
            egui::Align2::LEFT_TOP,
            text,
            font.clone(),
            theme::GRAY_700,
        );
        y += 16.0;
        if contour && max_abs >= 1e-12 {
            for i in 0..STRIPS {
                let t = (i as f64 + 0.5) / STRIPS as f64 * 2.0 - 1.0; // 短冊中央の t∈[-1,1]
                let color = contour_color(t, colormap);
                let sx0 = x0 + (i as f32 / STRIPS as f32) * BAR_W;
                let sx1 = x0 + ((i + 1) as f32 / STRIPS as f32) * BAR_W;
                painter.rect_filled(
                    egui::Rect::from_min_max(egui::pos2(sx0, y), egui::pos2(sx1, y + BAR_H)),
                    0.0,
                    color,
                );
            }
            let disp_max = max_abs * c.display_scale();
            painter.text(
                egui::pos2(x0, y + BAR_H + 1.0),
                egui::Align2::LEFT_TOP,
                format!("-{:.1}", disp_max),
                small.clone(),
                theme::GRAY_600,
            );
            painter.text(
                egui::pos2(x0 + BAR_W * 0.5, y + BAR_H + 1.0),
                egui::Align2::CENTER_TOP,
                "0",
                small.clone(),
                theme::GRAY_600,
            );
            painter.text(
                egui::pos2(x0 + BAR_W, y + BAR_H + 1.0),
                egui::Align2::RIGHT_TOP,
                format!("{:.1}", disp_max),
                small.clone(),
                theme::GRAY_600,
            );
            y += BAR_H + 14.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 単調増加（同符号）の区間は、各細分区間が台形（材軸2点＋張り出し2点）になり、
    /// 符号は常に非負（片側符号のみ）であることを確認する。
    #[test]
    fn diagram_fill_polygons_same_sign_is_trapezoids() {
        let samples = [(0.0, 1.0), (1.0, 3.0)];
        let polys = diagram_fill_polygons(&samples, 1);
        assert_eq!(polys.len(), 1);
        let poly = &polys[0];
        assert_eq!(poly.len(), 4);
        // 材軸上の点（val=0）を含む
        assert!(poly.iter().any(|&(_, v)| v == 0.0));
        // 片側符号のみ（すべて非負）
        assert!(poly.iter().all(|&(_, v)| v >= 0.0));
    }

    /// 符号反転区間は、ゼロ交差で2つの片側符号のみの三角形に分割される。
    #[test]
    fn diagram_fill_polygons_sign_change_splits_at_zero_crossing() {
        // v0=1, v1=-1 → 中点 xi=0.5 でゼロ交差
        let samples = [(0.0, 1.0), (1.0, -1.0)];
        let polys = diagram_fill_polygons(&samples, 1);
        assert_eq!(polys.len(), 2);

        // 1つ目は正側のみ、2つ目は負側のみ
        assert!(polys[0].iter().all(|&(_, v)| v >= 0.0));
        assert!(polys[1].iter().all(|&(_, v)| v <= 0.0));

        // どちらも材軸上の点（val=0）を含む
        assert!(polys[0].iter().any(|&(_, v)| v == 0.0));
        assert!(polys[1].iter().any(|&(_, v)| v == 0.0));

        // ゼロ交差位置 xi=0.5 がどちらのポリゴンにも現れる
        let xc_in_first = polys[0]
            .iter()
            .any(|&(xi, v)| v == 0.0 && (xi - 0.5).abs() < 1e-9);
        let xc_in_second = polys[1]
            .iter()
            .any(|&(xi, v)| v == 0.0 && (xi - 0.5).abs() < 1e-9);
        assert!(xc_in_first && xc_in_second);
    }

    /// 全体が単一符号内で常に正しい向き（片側符号のみ）を保つことを、
    /// 複数区間・複数符号反転を含む折れ線でまとめて確認する。
    #[test]
    fn diagram_fill_polygons_multi_segment_each_poly_single_sign() {
        // 両端 hogging（負）・中央 sagging（正）のような M 図の典型形状
        let samples = [(0.0, -2.0), (0.5, 1.0), (1.0, -2.0)];
        let polys = diagram_fill_polygons(&samples, 4);
        assert!(!polys.is_empty());
        for poly in &polys {
            let has_pos = poly.iter().any(|&(_, v)| v > 1e-12);
            let has_neg = poly.iter().any(|&(_, v)| v < -1e-12);
            // 同一ポリゴン内に正と負が同時に現れてはいけない（片側符号のみ）
            assert!(!(has_pos && has_neg), "poly has mixed sign: {:?}", poly);
        }
    }

    /// サンプル列が xi 降順で渡されても、関数内で昇順ソートされ結果が変わらない。
    #[test]
    fn diagram_fill_polygons_sorts_unsorted_input() {
        let sorted = diagram_fill_polygons(&[(0.0, 1.0), (1.0, 3.0)], 1);
        let unsorted = diagram_fill_polygons(&[(1.0, 3.0), (0.0, 1.0)], 1);
        assert_eq!(sorted, unsorted);
    }

    /// サンプルが2点未満なら空を返す。
    #[test]
    fn diagram_fill_polygons_needs_at_least_two_samples() {
        assert!(diagram_fill_polygons(&[], 1).is_empty());
        assert!(diagram_fill_polygons(&[(0.0, 1.0)], 1).is_empty());
    }

    /// 両端固定梁の等分布荷重（M は 2 次）で、評価断面 3 点（ξ=0, 0.5, 1）からの
    /// 補間が理論解 M(ξ)=wL²(6ξ−6ξ²−1)/12 に厳密一致すること。
    /// 直線補間では中央 +wL²/24 と端部 −wL²/12 を結ぶ折れ線になり、たとえば
    /// ξ=0.25 では理論値 +wL²/96（正＝sagging）に対し −wL²/48 と符号まで逆になる。
    #[test]
    fn moment_curve_samples_reproduces_udl_parabola() {
        let (l, w) = (1000.0_f64, 2.0_f64);
        let m = |xi: f64| w * l * l * (6.0 * xi - 6.0 * xi * xi - 1.0) / 12.0;
        let q = |xi: f64| w * l * (1.0 - 2.0 * xi) / 2.0;
        let samples: Vec<(f64, f64, f64)> = [0.0, 0.5, 1.0]
            .iter()
            .map(|&xi| (xi, m(xi), q(xi)))
            .collect();

        let curve = moment_curve_samples(&samples, l, 8);
        assert_eq!(curve.len(), 2 * 8 + 1);
        for &(xi, val) in &curve {
            assert!(
                (val - m(xi)).abs() < 1e-6 * w * l * l,
                "xi={xi} val={val} expected={}",
                m(xi)
            );
        }
        // ξ=0.25 では理論値 +wL²/96 に対し、直線補間なら −wL²/48（符号が逆）。
        // 曲率が復元され符号が正しいことを直線補間と対比して確認する。
        let at_quarter = curve
            .iter()
            .find(|(xi, _)| (xi - 0.25).abs() < 1e-9)
            .expect("ξ=0.25 のサンプルがあるはず")
            .1;
        let straight = 0.5 * (m(0.0) + m(0.5)); // ξ=0..0.5 を直線で結んだ値
        assert!((at_quarter - w * l * l / 96.0).abs() < 1e-6 * w * l * l);
        assert!(at_quarter > 0.0 && straight < 0.0, "直線補間と符号が同じ");
    }

    /// 部材中間荷重が無い部材（M は 1 次・Q は一定）では、補間しても直線のまま。
    #[test]
    fn moment_curve_samples_keeps_linear_field_straight() {
        let l = 1000.0_f64;
        let (m0, m1) = (-100.0_f64, 300.0_f64);
        let q = (m1 - m0) / l; // dM/dx = Q
        let samples = [(0.0, m0, q), (0.5, 0.5 * (m0 + m1), q), (1.0, m1, q)];

        for (xi, val) in moment_curve_samples(&samples, l, 8) {
            let expected = m0 + (m1 - m0) * xi;
            assert!((val - expected).abs() < 1e-9, "xi={xi} val={val}");
        }
    }

    /// 部材中央の集中荷重（小梁反力など）では M は折れ線になる。中央断面の Qy は
    /// 左側極限のため、そのまま右側区間の左端勾配に使うと架空のふくらみが出る。
    /// 勾配の不連続を検出して 2 次へ縮退させ、折れ線を保つこと。
    #[test]
    fn moment_curve_samples_midspan_point_load_stays_piecewise_linear() {
        let (l, p) = (1000.0_f64, 40.0_f64);
        let m_end = -p * l / 8.0;
        let m_mid = p * l / 8.0;
        // ξ=0.5 の Qy は左側極限（+P/2）。ξ=1 は右側の値（−P/2）。
        let samples = [
            (0.0, m_end, p / 2.0),
            (0.5, m_mid, p / 2.0),
            (1.0, m_end, -p / 2.0),
        ];

        for (xi, val) in moment_curve_samples(&samples, l, 8) {
            let expected = if xi <= 0.5 {
                m_end + (m_mid - m_end) * (xi / 0.5)
            } else {
                m_mid + (m_end - m_mid) * ((xi - 0.5) / 0.5)
            };
            assert!(
                (val - expected).abs() < 1e-6 * p * l,
                "xi={xi} val={val} expected={expected}"
            );
        }
    }

    /// 補間しても評価断面（危険断面）の値そのものは変わらない
    /// ——応力図が断面検定の値を通ることを保証する。
    #[test]
    fn moment_curve_samples_preserves_sample_values() {
        let l = 1000.0_f64;
        let samples = [(0.0, -120.0, 0.4), (0.4, 30.0, 0.1), (1.0, -90.0, -0.5)];

        let curve = moment_curve_samples(&samples, l, 4);
        for &(xi, m, _) in &samples {
            let hit = curve
                .iter()
                .find(|(x, _)| (x - xi).abs() < 1e-9)
                .unwrap_or_else(|| panic!("xi={xi} のサンプルが残っていない"));
            assert!((hit.1 - m).abs() < 1e-9, "xi={xi} val={}", hit.1);
        }
        // xi は昇順
        assert!(curve.windows(2).all(|w| w[0].0 <= w[1].0));
    }

    /// サンプルが 2 点未満なら補間せずそのまま返す（値のみ取り出す）。
    #[test]
    fn moment_curve_samples_passes_through_short_input() {
        assert!(moment_curve_samples(&[], 1000.0, 8).is_empty());
        assert_eq!(
            moment_curve_samples(&[(0.0, 5.0, 1.0)], 1000.0, 8),
            vec![(0.0, 5.0)]
        );
    }

    /// Viridis（既定カラーマップ）で t=-1 は濃紫端（#440154）、t=+1 は黄端（#FDE725）、
    /// t=0 は中央の青緑（#26828E）へ写像される。
    #[test]
    fn contour_color_endpoints_and_neutral() {
        let map = theme::ColorMap::Viridis;
        assert_eq!(
            contour_color(-1.0, map),
            egui::Color32::from_rgb(0x44, 0x01, 0x54)
        );
        assert_eq!(
            contour_color(1.0, map),
            egui::Color32::from_rgb(0xFD, 0xE7, 0x25)
        );
        assert_eq!(
            contour_color(0.0, map),
            egui::Color32::from_rgb(0x26, 0x82, 0x8E)
        );
    }

    /// 範囲外の値はクランプされる（t<-1 は t=-1 と同じ、t>1 は t=1 と同じ）。
    #[test]
    fn contour_color_clamps_out_of_range() {
        let map = theme::ColorMap::Viridis;
        assert_eq!(contour_color(-5.0, map), contour_color(-1.0, map));
        assert_eq!(contour_color(5.0, map), contour_color(1.0, map));
    }

    /// Viridis は t の増加とともに G 成分が単調非減少（濃紫→青緑→黄で緑みが増す）。
    #[test]
    fn contour_color_green_channel_is_monotonic() {
        let map = theme::ColorMap::Viridis;
        let ts = [-1.0, -0.5, 0.0, 0.5, 1.0];
        let greens: Vec<u8> = ts.iter().map(|&t| contour_color(t, map).g()).collect();
        for w in greens.windows(2) {
            assert!(w[0] <= w[1], "G成分が非単調: {:?}", greens);
        }
    }

    /// カラーマップを切り替えると異なる色になる（同じ t でも Viridis と Jet で異なる）。
    #[test]
    fn contour_color_respects_selected_colormap() {
        let viridis = contour_color(0.0, theme::ColorMap::Viridis);
        let jet = contour_color(0.0, theme::ColorMap::Jet);
        assert_ne!(viridis, jet);
    }
}

#[cfg(test)]
mod component_tests {
    use super::super::{DiagramPlane, ForceComponent, ForceComponents};
    use super::is_significant;

    /// 列挙順が部材内力ベクトル `[N, Qy, Qz, Mx, My, Mz]` の添字と一致する。
    /// 添字がずれると別成分の値で図を描いてしまう（無言の誤表示）。
    #[test]
    fn force_component_index_matches_declaration_order() {
        for (i, c) in ForceComponent::ALL.into_iter().enumerate() {
            assert_eq!(c.force_index(), i, "{}", c.label());
        }
    }

    /// 単位・換算係数は力＝kN（÷1e3）、モーメント＝kN·m（÷1e6）。
    #[test]
    fn force_component_units_are_kn_based() {
        for c in ForceComponent::ALL {
            if c.is_moment() {
                assert_eq!(c.unit(), "kN·m");
                assert_eq!(c.display_scale(), 1.0e-6);
            } else {
                assert_eq!(c.unit(), "kN");
                assert_eq!(c.display_scale(), 1.0e-3);
            }
        }
    }

    /// 張り出し面は強軸側（N・Qy・Mx・Mz）が ey、弱軸側（Qz・My）が ez。
    /// 曲げとせん断の対（Qy–Mz、Qz–My）が同じ面に出ることが読み取りの前提。
    #[test]
    fn force_component_planes_pair_bending_and_shear() {
        assert_eq!(ForceComponent::Qy.plane(), DiagramPlane::Ey);
        assert_eq!(ForceComponent::Mz.plane(), DiagramPlane::Ey);
        assert_eq!(ForceComponent::N.plane(), DiagramPlane::Ey);
        assert_eq!(ForceComponent::Mx.plane(), DiagramPlane::Ey);
        assert_eq!(ForceComponent::Qz.plane(), DiagramPlane::Ez);
        assert_eq!(ForceComponent::My.plane(), DiagramPlane::Ez);
    }

    /// 断面力の符号規約は `Qy = dMz/dx`・`Qz = −dMy/dx` で My だけ反転しているため、
    /// 「正のモーメントを引張側へ張り出す」規約を保つには My の張り出しだけ符号を
    /// 反転する。あわせて、エルミート補間の勾配源が対になっていることを確認する。
    #[test]
    fn my_is_plotted_with_flipped_sign_and_qz_gradient() {
        assert_eq!(ForceComponent::My.plot_sign(), -1.0);
        for c in ForceComponent::ALL
            .into_iter()
            .filter(|c| *c != ForceComponent::My)
        {
            assert_eq!(c.plot_sign(), 1.0, "{}", c.label());
        }
        assert_eq!(
            ForceComponent::Mz.moment_gradient_source(),
            Some(ForceComponent::Qy)
        );
        assert_eq!(
            ForceComponent::My.moment_gradient_source(),
            Some(ForceComponent::Qz)
        );
        assert_eq!(ForceComponent::N.moment_gradient_source(), None);
        assert_eq!(ForceComponent::Qy.moment_gradient_source(), None);
        assert_eq!(ForceComponent::Mx.moment_gradient_source(), None);
    }

    /// プリセットの内容と既定（M 図＝My・Mz）。
    #[test]
    fn presets_select_expected_components() {
        let sel =
            |s: ForceComponents| -> Vec<&'static str> { s.selected().map(|c| c.label()).collect() };
        assert_eq!(sel(ForceComponents::PRESET_N), vec!["N"]);
        assert_eq!(sel(ForceComponents::PRESET_Q), vec!["Qy", "Qz"]);
        assert_eq!(sel(ForceComponents::PRESET_M), vec!["My", "Mz"]);
        assert_eq!(
            ForceComponents::default(),
            ForceComponents::PRESET_M,
            "既定は M 図プリセット"
        );
    }

    /// 理論上ゼロの成分に残った丸め誤差を「描くに値する」と判定しない。
    ///
    /// 応力図は成分ごとに独立正規化する（60px ＝ その成分の最大値）ため、
    /// 全部材ゼロの成分を描いてしまうと**丸め誤差が画面いっぱいの図になる**。
    /// i 端ねじれ解放時の Mx が実測でこの状態になる（静縮約の桁落ちで
    /// 10⁻¹²N·mm 程度が残り、他成分は 10⁸N·mm 級）。
    #[test]
    fn negligible_component_is_not_drawn() {
        // [N, Qy, Qz, Mx, My, Mz]。Mx は丸め誤差、My/Mz は実応力。
        let maxes = [1.3e5, 7.6e4, 2.5e3, 2.7e-12, 5.4e6, 2.3e8];
        assert!(!is_significant(&maxes, ForceComponent::Mx));
        assert!(is_significant(&maxes, ForceComponent::My));
        assert!(is_significant(&maxes, ForceComponent::Mz));
        assert!(is_significant(&maxes, ForceComponent::N));
        assert!(is_significant(&maxes, ForceComponent::Qz));
    }

    /// 小さくても「同次元の最大値に対して意味のある比」を持つ成分は描く
    /// （平面フレームの面外成分のように、絶対値が小さいだけの実応力を消さない）。
    #[test]
    fn small_but_real_component_is_still_drawn() {
        // Mx は Mz の 10⁻⁵。丸め誤差（10⁻¹²以下）とは桁が違う。
        let maxes = [1.0e5, 1.0e5, 1.0e5, 2.3e3, 5.4e6, 2.3e8];
        assert!(is_significant(&maxes, ForceComponent::Mx));
    }

    /// 同次元の成分がすべてゼロなら何も描かない（無載荷ケースなど）。
    #[test]
    fn all_zero_group_is_not_drawn() {
        let maxes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        for c in ForceComponent::ALL {
            assert!(!is_significant(&maxes, c), "{}", c.label());
        }
    }

    /// チェックボックスの ON/OFF が `selected` の列挙に反映される（表示順は添字順）。
    #[test]
    fn toggling_flags_changes_selection_in_index_order() {
        let mut s = ForceComponents::PRESET_M;
        *s.flag_mut(ForceComponent::N) = true;
        *s.flag_mut(ForceComponent::My) = false;
        let labels: Vec<&'static str> = s.selected().map(|c| c.label()).collect();
        assert_eq!(labels, vec!["N", "Mz"]);
        assert!(s.is_on(ForceComponent::N) && !s.is_on(ForceComponent::My));
    }
}
