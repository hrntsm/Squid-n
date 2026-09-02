//! 変形表示・スケール・未参照節点の変位補間。
//!
//! `viewer` ハブからの構造分割。アルゴリズム変更は行わない。

use crate::app::App;

use squid_n_core::geom::vec3::dist as member_len3;

/// 変形図・モード形で梁の曲げ変形曲線を描く際の要素分割数（点数は +1）。
pub(super) const DEFORM_CURVE_SEGMENTS: usize = 12;

/// 梁要素の Hermite 3 次変形曲線を評価するための前処理データ。
///
/// 端部 6 自由度（節点変位、無倍率）を要素ローカル系へ一度だけ変換して保持し、
/// 材軸パラメータ ξ∈[0,1] での変位・曲線上の点を安価に評価する。曲線描画・応力図
/// の基準線・床節点の追従・変形スケール上限で共有し、「梁の変形後の形」の評価を
/// 一箇所へ集約する（ループでの `LocalFrame` 再構築も避ける）。
///
/// 軸方向は線形内挿、曲げ 2 面は Hermite 3 次形状関数で内挿する（等価節点力
/// [`squid_n_element::member_load`] と同一の形状関数・符号規約。局所 z 面は θy の
/// 符号反転）。ξ=0,1 では回転項が消え端点は節点変位に一致する。本内挿は表示専用
/// であり解析結果（節点変位・内力）は変更しない。要素はせん断変形を含む
/// Timoshenko 梁だが、変形図は Euler–Bernoulli の Hermite 曲線で近似する
/// （変形形状の可視化として実務上標準的）。
pub(super) struct BeamDeflection {
    /// 要素ローカル系（`rot` 行 = ex, ey, ez）。
    frame: squid_n_element::transform::LocalFrame,
    /// 部材長。
    l: f64,
    /// 未変形材軸の始点・終点（グローバル）。
    p_i: [f64; 3],
    p_j: [f64; 3],
    /// i 端のローカル端部変位 `[ux, uy, uz, ry, rz]`。
    ui: [f64; 5],
    /// j 端のローカル端部変位 `[ux, uy, uz, ry, rz]`。
    uj: [f64; 5],
}

impl BeamDeflection {
    /// 端部変位 `d_i`, `d_j`（節点変位 6 成分、無倍率）から前処理する。
    pub(super) fn new(
        p_i: [f64; 3],
        p_j: [f64; 3],
        d_i: [f64; 6],
        d_j: [f64; 6],
        ref_vector: [f64; 3],
    ) -> Self {
        let l = member_len3(p_i, p_j);
        let frame = squid_n_element::transform::LocalFrame::from_nodes(p_i, p_j, ref_vector);
        let g = [
            d_i[0], d_i[1], d_i[2], d_i[3], d_i[4], d_i[5], d_j[0], d_j[1], d_j[2], d_j[3], d_j[4],
            d_j[5],
        ];
        let u = frame.rotate_to_local(&g);
        // 端部: 並進(ux,uy,uz)=u[0..3]/u[6..9]、曲げ回転(ry,rz)=u[4..6]/u[10..12]。
        Self {
            frame,
            l,
            p_i,
            p_j,
            ui: [u[0], u[1], u[2], u[4], u[5]],
            uj: [u[6], u[7], u[8], u[10], u[11]],
        }
    }

    /// 材軸パラメータ ξ での「未変形材軸上の点へ加えるグローバル並進変位」（無倍率）。
    /// 床・二次部材の節点を梁曲線へ載せる補間で用いる（描画曲線から浮かないよう
    /// 同じ評価を共有する）。
    fn disp_at(&self, xi: f64) -> [f64; 3] {
        let l = self.l;
        // Hermite 3 次形状関数（N2,N4 は L 倍を含む回転項）。
        let n1 = 1.0 - 3.0 * xi * xi + 2.0 * xi * xi * xi;
        let n2 = l * (xi - 2.0 * xi * xi + xi * xi * xi);
        let n3 = 3.0 * xi * xi - 2.0 * xi * xi * xi;
        let n4 = l * (-xi * xi + xi * xi * xi);
        let [uxi, uyi, uzi, ryi, rzi] = self.ui;
        let [uxj, uyj, uzj, ryj, rzj] = self.uj;
        // ローカル変位場: y 面は θz、z 面は θy（符号反転、member_load の msign=-1 と一致）。
        let ux = (1.0 - xi) * uxi + xi * uxj;
        let uy = n1 * uyi + n2 * rzi + n3 * uyj + n4 * rzj;
        let uz = n1 * uzi - n2 * ryi + n3 * uzj - n4 * ryj;
        // ローカル→グローバル（rot 行 = ex,ey,ez。global = ux·ex + uy·ey + uz·ez）。
        let r = &self.frame.rot;
        [
            r[0][0] * ux + r[1][0] * uy + r[2][0] * uz,
            r[0][1] * ux + r[1][1] * uy + r[2][1] * uz,
            r[0][2] * ux + r[1][2] * uy + r[2][2] * uz,
        ]
    }

    /// 変形後曲線上の点（未変形材軸上の点 + 倍率付き変位）を ξ で返す。
    /// ξ=0,1 では端点＝節点変位（`scale` 倍）に厳密一致する（節点マーカーと連続）。
    pub(super) fn point_at(&self, xi: f64, scale: f64) -> [f64; 3] {
        let dg = self.disp_at(xi);
        [
            self.p_i[0] + (self.p_j[0] - self.p_i[0]) * xi + dg[0] * scale,
            self.p_i[1] + (self.p_j[1] - self.p_i[1]) * xi + dg[1] * scale,
            self.p_i[2] + (self.p_j[2] - self.p_i[2]) * xi + dg[2] * scale,
        ]
    }

    /// 変形後曲線を両端含む `segments + 1` 点の折れ線で返す（曲線描画用）。
    pub(super) fn polyline(&self, scale: f64, segments: usize) -> Vec<[f64; 3]> {
        let seg = segments.max(1);
        (0..=seg)
            .map(|k| self.point_at(k as f64 / seg as f64, scale))
            .collect()
    }
}

fn interpolate_unreferenced_disp(
    model: &squid_n_core::model::Model,
    mut disp: Vec<[f64; 6]>,
    use_beam_hermite: bool,
) -> Vec<[f64; 6]> {
    let n = model.nodes.len().min(disp.len());

    // 解析自由度を持ち変位が直接求まる節点（構造節点。判定は解析
    // （`DofMap::build`）と共通の `structural_nodes`）。剛床代表節点（階自動生成が
    // 重心に置く仮想節点）は要素に接続しないが拘束のマスターとして正しい解析変位を
    // 持つため、補間で上書きしてはいけない。
    let mut referenced = squid_n_core::dof::structural_nodes(model);
    referenced.truncate(n);
    referenced.resize(n, false);
    if referenced.iter().all(|&r| r) {
        return disp;
    }

    // 補間ソースとなる主架構の線材（2 節点要素）。端点は必ず参照済み（正しい解析
    // 変位を持つ）ため、射影補間は他の未参照節点に依存しない。梁（`Beam`）は
    // 変形図で Hermite 3 次曲線として描画されるため、その線上に載る節点は端点変位
    // の線形補間ではなく梁の Hermite 変位で追従させる（描画曲線から浮かないよう
    // 端点回転を含めて評価する）。梁以外（ブレース等）は従来どおり線形補間とする
    // ため、要素種別と局所座標参照ベクトルを保持する。
    struct AnchorSeg {
        a: usize,
        b: usize,
        beam: bool,
        ref_vec: [f64; 3],
    }
    let segments: Vec<AnchorSeg> = model
        .elements
        .iter()
        .filter(|e| e.nodes.len() == 2)
        .map(|e| AnchorSeg {
            a: e.nodes[0].index(),
            b: e.nodes[1].index(),
            beam: e.kind == squid_n_core::model::ElementKind::Beam,
            ref_vec: e.local_axis.ref_vector,
        })
        .filter(|s| s.a < n && s.b < n)
        .collect();

    // 「大梁に直付き（線上に載る）」と判定する許容垂線距離。モデル寸法に対する
    // 相対値（バウンディングボックス対角長の 0.1%）。これより近い射影は主架構への
    // 直付きアンカーとして主架構変位を直接採用し、遠い節点は二次部材の接続を
    // 辿って追従させる。
    let anchor_tol = (model_bbox_size(model) * 1e-3).max(1e-9);

    // 段階 1: 各未参照節点を最寄り線分へ射影し、垂線距離が許容値以内なら主架構
    // 直付きアンカーとして確定する。射影変位は、伝播が届かなかった場合の
    // フォールバックとしても保持する。
    let mut finalized = referenced.clone();
    let mut proj_disp = vec![[0.0_f64; 6]; n];
    let mut proj_ok = vec![false; n];
    for i in 0..n {
        if referenced[i] {
            continue;
        }
        let p = model.nodes[i].coord;
        // 射影点までの距離が最小の線分を探す（射影パラメータ t は [0,1] にクランプ）。
        let mut best: Option<(f64, usize, f64)> = None; // (垂線距離², 線分 index, 射影 t)
        for (si, s) in segments.iter().enumerate() {
            let pa = model.nodes[s.a].coord;
            let pb = model.nodes[s.b].coord;
            let ab = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
            let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
            let t = if len2 < 1e-12 {
                0.0
            } else {
                (((p[0] - pa[0]) * ab[0] + (p[1] - pa[1]) * ab[1] + (p[2] - pa[2]) * ab[2]) / len2)
                    .clamp(0.0, 1.0)
            };
            let q = [pa[0] + ab[0] * t, pa[1] + ab[1] * t, pa[2] + ab[2] * t];
            let d2 = (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2);
            if best.is_none_or(|(bd, _, _)| d2 < bd) {
                best = Some((d2, si, t));
            }
        }
        if let Some((d2, si, t)) = best {
            let s = &segments[si];
            let (da, db) = (disp[s.a], disp[s.b]);
            // 梁で内部たわみ表示が有効なときのみ Hermite 変位で追従（並進 3 成分は
            // 描画曲線上へ載せ、回転は端点の線形補間で補う）。梁以外、または内部
            // たわみ表示 OFF（梁を直線で描く）のときは全 6 成分を線形補間する。
            let interp: [f64; 6] = if s.beam && use_beam_hermite {
                let hermite = BeamDeflection::new(
                    model.nodes[s.a].coord,
                    model.nodes[s.b].coord,
                    da,
                    db,
                    s.ref_vec,
                )
                .disp_at(t);
                std::array::from_fn(|k| match k {
                    0..=2 => hermite[k],
                    _ => da[k] * (1.0 - t) + db[k] * t,
                })
            } else {
                std::array::from_fn(|k| da[k] * (1.0 - t) + db[k] * t)
            };
            proj_disp[i] = interp;
            proj_ok[i] = true;
            if d2.sqrt() <= anchor_tol {
                disp[i] = interp;
                finalized[i] = true;
            }
        }
    }

    // 段階 2: 二次部材（小梁・間柱）の接続グラフを辿り、大梁に直付きしない節点を
    // 最寄りの確定節点の変位へ追従させる。
    let mut sec_adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for sm in model.joists().chain(model.posts()) {
        let a = sm.nodes[0].index();
        let b = sm.nodes[1].index();
        if a < n && b < n {
            sec_adj[a].push(b);
            sec_adj[b].push(a);
        }
    }
    let node_dist = |a: usize, b: usize| -> f64 {
        let pa = model.nodes[a].coord;
        let pb = model.nodes[b].coord;
        ((pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2)).sqrt()
    };

    // 追従元候補（確定節点から二次部材でつながる未確定節点への辺長と追従変位）。
    let mut best_dist = vec![f64::INFINITY; n];
    let mut src_disp = vec![[0.0_f64; 6]; n];
    let mut has_source = vec![false; n];
    // 確定節点（参照済み＋主架構直付きアンカー）から隣接未確定節点を緩和する。
    for u in 0..n {
        if !finalized[u] {
            continue;
        }
        for &j in &sec_adj[u] {
            if finalized[j] {
                continue;
            }
            let d = node_dist(u, j);
            if d < best_dist[j] {
                best_dist[j] = d;
                src_disp[j] = disp[u];
                has_source[j] = true;
            }
        }
    }
    // 最寄りの確定節点から順に確定させる（辺長を距離とする Dijkstra 的貪欲法）。
    // 二次部材の連鎖が長くても、主架構に最も近い側から変位が伝播する。
    loop {
        let mut pick: Option<(usize, f64)> = None;
        for i in 0..n {
            if finalized[i] || !has_source[i] {
                continue;
            }
            if pick.is_none_or(|(_, bd)| best_dist[i] < bd) {
                pick = Some((i, best_dist[i]));
            }
        }
        let Some((u, _)) = pick else { break };
        disp[u] = src_disp[u];
        finalized[u] = true;
        // u を追従元として、二次部材でつながる未確定の隣接節点を緩和する。
        for &j in &sec_adj[u] {
            if finalized[j] {
                continue;
            }
            let d = node_dist(u, j);
            if d < best_dist[j] {
                best_dist[j] = d;
                src_disp[j] = disp[u];
                has_source[j] = true;
            }
        }
    }

    // フォールバック: まだ確定しない節点（大梁にも直付きせず、二次部材でも確定
    // 節点に到達しない孤立した床境界節点など）は、最寄り線分への射影変位を採る。
    for i in 0..n {
        if !finalized[i] && proj_ok[i] {
            disp[i] = proj_disp[i];
        }
    }
    disp
}

/// 剛床代表節点（マスター）の鉛直変位（Uz）を、スレーブ節点の鉛直変位の平均で
/// 表示用に補う。あくまで描画専用の近似で、解析結果（`StaticOnce::disp`）は変更
/// しない。
///
/// マスターの面内自由度（Ux・Uy・Rz）は解析結果をそのまま使うため水平変形には
/// 追従するが、面外自由度（Uz・Rx・Ry）は零剛性による特異行列を避けるための数値
/// ダミー拘束で 0 に固定されている（`squid-n-load` の `story_gen`）。そのままだと
/// 変形図で代表点だけが原標高に浮き、床の鉛直変形（重力たわみ・地震の転倒による
/// 床の上下動）へ追従しない。スレーブ節点の Uz 平均を代表点の Uz とすることで、
/// 代表点を変形後の床の平均標高へ載せる。
fn fill_diaphragm_master_disp_for_display(
    model: &squid_n_core::model::Model,
    mut disp: Vec<[f64; 6]>,
) -> Vec<[f64; 6]> {
    let n = model.nodes.len().min(disp.len());
    for c in &model.constraints {
        let squid_n_core::model::Constraint::RigidDiaphragm { master, slaves, .. } = c else {
            continue;
        };
        let mi = master.index();
        if mi >= n {
            continue;
        }
        let mut sum = 0.0_f64;
        let mut cnt = 0.0_f64;
        for sl in slaves {
            let si = sl.index();
            if si < n {
                sum += disp[si][2];
                cnt += 1.0;
            }
        }
        if cnt >= 0.5 {
            disp[mi][2] = sum / cnt;
        }
    }
    disp
}

/// 解析変位を表示用に加工する（いずれも描画専用の近似で、解析結果は変更しない）。
///
/// 1. 主架構に接続しない床・二次部材の節点を主架構の変形へ追従させる
///    （[`interpolate_unreferenced_disp`]。梁に載る節点は内部たわみ表示 ON なら
///    梁の Hermite 曲線上へ、OFF なら弦上へ）。
/// 2. 剛床代表節点の鉛直変位をスレーブ平均で補い、代表点を床の変形へ追従させる
///    （[`fill_diaphragm_master_disp_for_display`]）。
pub(super) fn display_disp(
    model: &squid_n_core::model::Model,
    raw: Vec<[f64; 6]>,
    use_beam_hermite: bool,
) -> Vec<[f64; 6]> {
    let d = interpolate_unreferenced_disp(model, raw, use_beam_hermite);
    fill_diaphragm_master_disp_for_display(model, d)
}

/// 変形図の実効表示倍率（自動倍率 × 手動係数）を算定する。変位がない（`None`）・
/// 全並進成分がゼロなら 0 を返す（変形を描かない）。
///
/// 自動倍率は次の小さい方:
/// - **バウンディングボックス基準**: 最大並進変位がモデル対角長の 10% で表示される
///   倍率 `0.1 · model_size / δ_max`。
/// - **梁スパン基準**（`use_beam_interpolation` が真のときのみ）: 各梁の Hermite 内部
///   たわみがスパンの一定割合を超えない上限（[`beam_deflection_scale_limit`]）。
///   内部たわみ OFF（梁を直線で描く）ではふくらみが生じないため併用しない。
///
/// これに手動係数 `factor`（スライダー）を掛けた値を実効倍率とする。
pub(super) fn deform_display_scale(
    model: &squid_n_core::model::Model,
    disp: Option<&[[f64; 6]]>,
    model_size: f64,
    use_beam_interpolation: bool,
    factor: f32,
) -> f64 {
    let Some(d) = disp else {
        return 0.0;
    };
    let max_disp = d
        .iter()
        .map(|v| v[0].abs().max(v[1].abs()).max(v[2].abs()))
        .fold(0.0_f64, f64::max);
    if max_disp <= 1e-12 {
        return 0.0;
    }
    let bbox_scale = model_size * 0.1 / max_disp;
    let auto = if use_beam_interpolation {
        beam_deflection_scale_limit(model, d).map_or(bbox_scale, |lim| bbox_scale.min(lim))
    } else {
        bbox_scale
    };
    auto * factor as f64
}

/// 時刻歴アニメーションの変形倍率キャッシュ（高-2）。
///
/// 通常の変形図（[`deform_display_scale`]）は現在フレームの変位から自動倍率を
/// 算定するため、時刻歴アニメーションへそのまま適用すると振幅の小さいフレームで
/// 倍率が発散し、逆に無変形（初期状態）フレームでは 0 になって表示が消える。
/// 記録全体のピーク変位から 1 回だけ算定した固定倍率を使うことでこれを避ける。
///
/// `auto_scale` は手動係数（`App::deform_scale_factor`）を掛ける前の自動倍率。
/// 記録の同一性は「フレーム数＋ピーク変位」で判定する（解析をやり直すと
/// フレーム数かピーク値のいずれかが変わるため、それで十分にキャッシュを無効化できる）。
/// モデルサイズ・内部たわみ表示 ON/OFF が変わった場合も再計算する。
#[derive(Clone, Copy, Debug, Default)]
pub struct TimeHistoryScaleCache {
    n_frames: usize,
    peak_max_disp: f64,
    model_size: f64,
    use_beam_interpolation: bool,
    auto_scale: f64,
}

/// `ResponseResult::peak_disp`（全ステップ間引きなしのピーク変位、節点×6成分）から、
/// 並進成分（ux/uy/uz）の絶対値最大を求める（純粋関数）。
fn th_peak_translation_disp(result: &squid_n_solver::timehistory::ResponseResult) -> f64 {
    result
        .peak_disp
        .iter()
        .map(|d| d[0].abs().max(d[1].abs()).max(d[2].abs()))
        .fold(0.0_f64, f64::max)
}

/// 時刻歴アニメーションの実効表示倍率（自動倍率 × 手動係数）。
/// `app.ui.scoped.th_scale_cache` を記録の同一性で使い回し、フレーム切替のたびに
/// 自動倍率を再計算しない（高-2）。時刻歴の詳細記録・結果がなければ 0。
pub(super) fn time_history_deform_scale(app: &mut App, model_size: f64) -> f64 {
    let Some(result) = app
        .core
        .scoped
        .results
        .as_ref()
        .and_then(|r| r.time_history.as_ref())
    else {
        app.ui.scoped.th_scale_cache = None;
        return 0.0;
    };
    let n_frames = result.recording.as_ref().map_or(0, |r| r.frame_time.len());
    let peak_max_disp = th_peak_translation_disp(result);
    let use_beam_interpolation = app.ui.view.show_beam_interpolation;

    let reuse = app.ui.scoped.th_scale_cache.is_some_and(|c| {
        c.n_frames == n_frames
            && c.peak_max_disp == peak_max_disp
            && c.model_size == model_size
            && c.use_beam_interpolation == use_beam_interpolation
    });
    let auto_scale = if reuse {
        app.ui
            .scoped
            .th_scale_cache
            .expect("reuse implies Some")
            .auto_scale
    } else {
        // ピーク変位（全ノード・全ステップの並進絶対値最大）を仮想的な変位配列とし、
        // 既存の `deform_display_scale`（バウンディングボックス基準＋梁スパン基準）を
        // 手動係数 1.0 でそのまま流用する（倍率算定ロジックの重複を避ける）。
        let peak_disp_field: Vec<[f64; 6]> = result.peak_disp.clone();
        let auto = deform_display_scale(
            &app.core.model,
            Some(&peak_disp_field),
            model_size,
            use_beam_interpolation,
            1.0,
        );
        app.ui.scoped.th_scale_cache = Some(TimeHistoryScaleCache {
            n_frames,
            peak_max_disp,
            model_size,
            use_beam_interpolation,
            auto_scale: auto,
        });
        auto
    };
    auto_scale * app.ui.view.deform_scale_factor as f64
}

/// 梁のスパンに対する内部たわみが過大にならないよう、表示倍率の上限を算定する。
/// 制約する梁がなければ `None`。
///
/// 変形図の梁は端部 6 自由度からの Hermite 3 次曲線で描くため、端部回転が大きいと
/// 中央のふくらみ（変形後両端を結ぶ弦からの逸脱）がスパンに対して過大になり得る。
/// 各梁について無倍率のたわみ（弦からの最大逸脱）を評価し、
/// `倍率 × たわみ ≤ FRAC × スパン` を満たす倍率上限 `FRAC × スパン / たわみ` の
/// 最小値を返す。バウンディングボックス基準の倍率と併せて小さい方を採ることで、
/// 全体変形も梁のふくらみも過大にならないスケールにする。
fn beam_deflection_scale_limit(
    model: &squid_n_core::model::Model,
    disp: &[[f64; 6]],
) -> Option<f64> {
    /// 梁の内部たわみ（弦からの逸脱）が許容されるスパン比。
    const BEAM_DEFLECTION_DISPLAY_FRAC: f64 = 0.1;
    /// たわみ評価の内部サンプル点数（両端を除く分割）。
    const SAMPLES: usize = 9;

    let n = model.nodes.len().min(disp.len());
    let mut limit: Option<f64> = None;
    for elem in &model.elements {
        if elem.kind != squid_n_core::model::ElementKind::Beam || elem.nodes.len() != 2 {
            continue;
        }
        let a = elem.nodes[0].index();
        let b = elem.nodes[1].index();
        if a >= n || b >= n {
            continue;
        }
        let p_i = model.nodes[a].coord;
        let p_j = model.nodes[b].coord;
        let l = member_len3(p_i, p_j);
        if l < 1e-9 {
            continue;
        }
        let (d_i, d_j) = (disp[a], disp[b]);
        // 無倍率での弦からの最大逸脱（弦＝端部並進の線形補間、曲線＝Hermite 変位）。
        // 端部 DOF のローカル化は ξ に依らないため、梁ごとに 1 回だけ前処理する。
        let bd = BeamDeflection::new(p_i, p_j, d_i, d_j, elem.local_axis.ref_vector);
        let mut max_dev = 0.0_f64;
        for k in 1..SAMPLES {
            let xi = k as f64 / SAMPLES as f64;
            let h = bd.disp_at(xi);
            let dev = ((h[0] - (d_i[0] * (1.0 - xi) + d_j[0] * xi)).powi(2)
                + (h[1] - (d_i[1] * (1.0 - xi) + d_j[1] * xi)).powi(2)
                + (h[2] - (d_i[2] * (1.0 - xi) + d_j[2] * xi)).powi(2))
            .sqrt();
            max_dev = max_dev.max(dev);
        }
        if max_dev > 1e-12 {
            let lim = BEAM_DEFLECTION_DISPLAY_FRAC * l / max_dev;
            limit = Some(limit.map_or(lim, |cur: f64| cur.min(lim)));
        }
    }
    limit
}

/// モデルのバウンディングボックス（min, max）。空なら原点を返す。
pub(super) fn model_bbox(model: &squid_n_core::model::Model) -> ([f64; 3], [f64; 3]) {
    if model.nodes.is_empty() {
        return ([0.0; 3], [0.0; 3]);
    }
    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];
    for n in &model.nodes {
        for k in 0..3 {
            min[k] = min[k].min(n.coord[k]);
            max[k] = max[k].max(n.coord[k]);
        }
    }
    (min, max)
}

pub(super) fn frame_bbox(
    model: &squid_n_core::model::Model,
    frame: &squid_n_core::frame::Frame,
) -> Option<([f64; 3], [f64; 3])> {
    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];
    let mut found = false;
    for (i, e) in model.elements.iter().enumerate() {
        if !frame.elem_on.get(i).copied().unwrap_or(false) {
            continue;
        }
        for nid in &e.nodes {
            let Some(n) = model.nodes.get(nid.index()) else {
                continue;
            };
            found = true;
            for k in 0..3 {
                min[k] = min[k].min(n.coord[k]);
                max[k] = max[k].max(n.coord[k]);
            }
        }
    }
    found.then_some((min, max))
}

/// バウンディングボックスの対角線長。
pub(super) fn bbox_diagonal(min: [f64; 3], max: [f64; 3]) -> f64 {
    let d =
        ((max[0] - min[0]).powi(2) + (max[1] - min[1]).powi(2) + (max[2] - min[2]).powi(2)).sqrt();
    if d > 1e-9 {
        d
    } else {
        1.0
    }
}

/// モデルのバウンディングボックス対角線長。
pub(super) fn model_bbox_size(model: &squid_n_core::model::Model) -> f64 {
    if model.nodes.is_empty() {
        return 1.0;
    }
    let (min, max) = model_bbox(model);
    ((max[0] - min[0]).powi(2) + (max[1] - min[1]).powi(2) + (max[2] - min[2]).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, Node,
        SecondaryMember, SecondaryMemberKind,
    };

    /// 補間テスト用の節点を作る（拘束なし・付加情報なし）。
    fn test_node(id: u32, coord: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    /// 補間テスト用の二次部材（小梁）を作る。
    fn test_secondary(i: u32, j: u32) -> SecondaryMember {
        SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(i), NodeId(j)],
            section: None,
            name: String::new(),
        }
    }

    /// 補間テスト用の 2 節点梁要素を作る。
    fn test_beam(id: u32, i: u32, j: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    #[test]
    fn 主架構に接続する節点の変位は補間で変更されない() {
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));

        let disp = vec![
            [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
            [4.0, 5.0, 6.0, 0.4, 0.5, 0.6],
        ];
        let out = interpolate_unreferenced_disp(&model, disp.clone(), true);
        assert_eq!(out, disp);
    }

    #[test]
    fn 大梁スパン中間の未参照節点は梁のエルミート変位で追従する() {
        // 大梁 n0-n1 のスパン 1/4 点に、節点共有なしで載る小梁支持点 n2
        // （ST-Bridge 取り込みモデルの典型）を置く。梁は変形図で Hermite 曲線として
        // 描かれるため、直付き節点は端点の線形補間ではなく Hermite 変位で追従する。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [8000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [2000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));

        let disp = vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [4.0, 8.0, -12.0, 0.0, 0.0, 0.0],
            [0.0; 6], // 未参照節点は解析結果ではゼロ
        ];
        let out = interpolate_unreferenced_disp(&model, disp, true);
        // t = 2000/8000 = 0.25。端部回転は 0 のため、軸方向（+X）は線形（0.25·4=1.0）、
        // 材軸直交成分（Y,Z）は Hermite の N3(0.25)=0.15625 倍で追従する
        // （線形補間の 0.25 倍より小さく、梁の描画曲線上に載る）。
        assert!((out[2][0] - 1.0).abs() < 1e-12, "X={}", out[2][0]);
        assert!((out[2][1] - 8.0 * 0.15625).abs() < 1e-12, "Y={}", out[2][1]);
        assert!(
            (out[2][2] + 12.0 * 0.15625).abs() < 1e-12,
            "Z={}",
            out[2][2]
        );
    }

    #[test]
    fn 大梁スパン中間の未参照節点は内部たわみオフで線形補間になる() {
        // 内部たわみ表示 OFF（梁を直線で描く「全体変形」表示）では、直付き節点も
        // 端点の線形補間で追従する（梁の直線＝弦の上に載る）。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [8000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [2000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));

        let disp = vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [4.0, 8.0, -12.0, 0.0, 0.0, 0.0],
            [0.0; 6],
        ];
        let out = interpolate_unreferenced_disp(&model, disp, false);
        // t = 0.25 の線形補間（全成分）。
        assert!((out[2][0] - 1.0).abs() < 1e-12, "X={}", out[2][0]);
        assert!((out[2][1] - 2.0).abs() < 1e-12, "Y={}", out[2][1]);
        assert!((out[2][2] + 3.0).abs() < 1e-12, "Z={}", out[2][2]);
    }

    #[test]
    fn 梁上の未参照節点は梁の描画曲線に厳密一致する() {
        // 端部に回転を与えた梁のスパン中央に未参照節点を置く。その補間変位が、同じ
        // 端部変位で BeamDeflection::polyline を描いた曲線の同一パラメータ位置の変位に
        // 厳密一致すること（床・二次部材の節点が梁の描画たわみ曲線から浮かない）。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [3000.0, 0.0, 0.0])); // スパン中央（t=0.5）
        model.elements.push(test_beam(0, 0, 1)); // ref_vector=[0,0,1]

        let d_i = [0.0, 0.0, 0.0, 0.0, 0.0, 0.01];
        let d_j = [0.0, 0.0, 0.0, 0.0, 0.0, -0.01];
        let disp = vec![d_i, d_j, [0.0; 6]];
        let out = interpolate_unreferenced_disp(&model, disp, true);

        // 同じ端部変位で梁曲線を無倍率描画し、中央点（12 分割の index 6=ξ0.5）の
        // 変位（曲線点 − 未変形材軸点）を取る。
        let poly = BeamDeflection::new(
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            d_i,
            d_j,
            [0.0, 0.0, 1.0],
        )
        .polyline(1.0, 12);
        let curve_disp = [poly[6][0] - 3000.0, poly[6][1] - 0.0, poly[6][2] - 0.0];
        for k in 0..3 {
            assert!(
                (out[2][k] - curve_disp[k]).abs() < 1e-9,
                "軸 {k}: 補間 {} と曲線 {} が不一致",
                out[2][k],
                curve_disp[k]
            );
        }
        // 端部回転で中央がたわむため、直線（線形補間＝0）とは異なる。
        assert!(
            out[2][1].abs() > 1.0,
            "Hermite たわみが出ていない: {}",
            out[2][1]
        );
    }

    #[test]
    fn 梁軸から外れた未参照節点も最寄り線分の射影位置で補間される() {
        // 大梁からオフセットした位置の節点（床境界の幾何節点など）は、
        // 最寄り線分への射影点（クランプ込み）の変位で追従する。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [4000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [2000.0, 500.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));

        let disp = vec![[0.0; 6], [10.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6]];
        let out = interpolate_unreferenced_disp(&model, disp, true);
        // 射影点は t=0.5 → 5.0
        assert!((out[2][0] - 5.0).abs() < 1e-12);
    }

    #[test]
    fn 主架構の線材がなければ未参照節点の変位はゼロのまま() {
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [1000.0, 0.0, 0.0]));
        // 要素なし → 補間ソースがなく、変位はゼロのまま
        let out = interpolate_unreferenced_disp(&model, vec![[0.0; 6]; 2], true);
        assert!(out.iter().all(|v| v.iter().all(|&x| x == 0.0)));
    }

    #[test]
    fn 剛床マスター節点の変位は補間で上書きされない() {
        // 剛床代表節点（階自動生成が重心に置く仮想節点）は要素に接続しないが、
        // 拘束のマスターとして解析自由度を持ち正しい変位が求まる
        // （`DofMap::build` の structural 判定と同じ規則）。補間対象にしてはいけない。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [8000.0, 0.0, 0.0]));
        model.nodes.push(test_node(2, [4000.0, 0.0, 0.0])); // 剛床マスター
        model.elements.push(test_beam(0, 0, 1));
        model
            .constraints
            .push(squid_n_core::model::Constraint::rigid_diaphragm(
                squid_n_core::ids::StoryId(0),
                NodeId(2),
                vec![NodeId(0), NodeId(1)],
            ));

        let disp = vec![
            [0.0; 6],
            [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [7.0, 0.0, 0.0, 0.0, 0.0, 0.0], // マスターの解析変位（補間値 5.0 とは異なる）
        ];
        let out = interpolate_unreferenced_disp(&model, disp.clone(), true);
        assert_eq!(out, disp);
    }

    #[test]
    fn 変形曲線の端部は節点変位に一致する() {
        // 水平梁（i→j が +X）。両端に異なる並進・回転を与え、ξ=0,1 が
        // 節点変位（scale 倍）に厳密一致することを確認する。
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d_i = [0.0, 1.0, 0.0, 0.0, 0.0, 0.001];
        let d_j = [2.0, 3.0, 0.0, 0.0, 0.0, -0.002];
        let scale = 2.0;
        let poly = BeamDeflection::new(p_i, p_j, d_i, d_j, [0.0, 0.0, 1.0]).polyline(scale, 12);
        assert_eq!(poly.len(), 13);
        // i 端 = p_i + scale·d_i(並進)
        for k in 0..3 {
            assert!(
                (poly[0][k] - (p_i[k] + scale * d_i[k])).abs() < 1e-6,
                "i端 axis{k}: {}",
                poly[0][k]
            );
            assert!(
                (poly[12][k] - (p_j[k] + scale * d_j[k])).abs() < 1e-6,
                "j端 axis{k}: {}",
                poly[12][k]
            );
        }
    }

    #[test]
    fn 端部回転で中央がたわむ() {
        // 水平梁（i→j が +X）、ref=+Y とすると局所系は全体系と一致
        // （ex=+X, ey=+Y, ez=+Z）。両端の並進を 0、i 端に正・j 端に負の
        // θz（全体=局所 z 軸まわり）を与えると、Hermite 内挿で局所 y(=+Y)へ
        // 中央がふくらむ。直線（節点間）内挿なら中央は原位置のまま（たわみ 0）。
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d_i = [0.0, 0.0, 0.0, 0.0, 0.0, 0.01];
        let d_j = [0.0, 0.0, 0.0, 0.0, 0.0, -0.01];
        let poly = BeamDeflection::new(p_i, p_j, d_i, d_j, [0.0, 1.0, 0.0]).polyline(1.0, 12);
        let mid = poly[6];
        // 中央の材軸位置は x=500、たわみは局所 y=+Y 方向へ非ゼロ
        assert!((mid[0] - 500.0).abs() < 1e-6, "中央 x={}", mid[0]);
        assert!(
            mid[1].abs() > 1.0,
            "中央のたわみが小さすぎる: dy={}",
            mid[1]
        );
        // 端部は原位置（並進 0・回転のみ）
        assert!(poly[0][1].abs() < 1e-9 && poly[12][1].abs() < 1e-9);
    }

    #[test]
    fn 梁変形後曲線の端点は節点変位に一致し中央は弦から外れる() {
        // 応力図の基準線に使う BeamDeflection::point_at の検証。端点（ξ=0,1）は
        // 節点変位（scale 倍）に一致し、中央（ξ=0.5）は端部回転により弦（端点の
        // 線形補間）から外れてたわむ。
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [6000.0, 0.0, 0.0];
        let d_i = [0.0, 0.0, 0.0, 0.0, 0.0, 0.01];
        let d_j = [0.0, 0.0, 0.0, 0.0, 0.0, -0.01];
        let scale = 2.0;
        let bd = BeamDeflection::new(p_i, p_j, d_i, d_j, [0.0, 0.0, 1.0]);
        let a = bd.point_at(0.0, scale);
        let b = bd.point_at(1.0, scale);
        for k in 0..3 {
            assert!(
                (a[k] - (p_i[k] + scale * d_i[k])).abs() < 1e-6,
                "端点i k={k}"
            );
            assert!(
                (b[k] - (p_j[k] + scale * d_j[k])).abs() < 1e-6,
                "端点j k={k}"
            );
        }
        let mid = bd.point_at(0.5, scale);
        let chord_mid = [
            (a[0] + b[0]) * 0.5,
            (a[1] + b[1]) * 0.5,
            (a[2] + b[2]) * 0.5,
        ];
        let dev = ((mid[0] - chord_mid[0]).powi(2)
            + (mid[1] - chord_mid[1]).powi(2)
            + (mid[2] - chord_mid[2]).powi(2))
        .sqrt();
        assert!(dev > 1.0, "中央が弦から外れていない: dev={}", dev);
    }

    #[test]
    fn 大梁に直付きしない二次部材の先端は接続先を辿って追従する() {
        // 大梁 G1(0-1) は節点 1 に大きな水平変位を持つ。node 2 は G1 のスパン上
        // （直付きアンカー）。node 3 は G1 から離れた先端で、二次部材 2-3 で node 2 に
        // つながる。もう 1 本の大梁 G2(4-5)（変位ゼロ）を node 3 の近くに置き、
        // 「最寄り線分へ射影」だけでは node 3 が G2 へ張り付いて追従しないところを、
        // 二次部材経由の追従で取り付き先（node 2）へ揃うことを確認する。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0])); // G1 端
        model.nodes.push(test_node(1, [8000.0, 0.0, 0.0])); // G1 端
        model.nodes.push(test_node(2, [2000.0, 0.0, 0.0])); // G1 上（直付き）
        model.nodes.push(test_node(3, [2000.0, 4000.0, 0.0])); // 先端（G1 から 4000, G2 から 1000）
        model.nodes.push(test_node(4, [0.0, 5000.0, 0.0])); // G2 端
        model.nodes.push(test_node(5, [8000.0, 5000.0, 0.0])); // G2 端
        model.elements.push(test_beam(0, 0, 1)); // G1
        model.elements.push(test_beam(1, 4, 5)); // G2
        model.unassigned_joists.push(test_secondary(2, 3)); // 二次部材 2-3

        // G1 は大きく水平移動、G2 は変位ゼロ。
        let disp = vec![
            [0.0; 6],                         // 0
            [100.0, 0.0, 0.0, 0.0, 0.0, 0.0], // 1
            [0.0; 6],                         // 2（未参照）
            [0.0; 6],                         // 3（未参照）
            [0.0; 6],                         // 4
            [0.0; 6],                         // 5
        ];
        let out = interpolate_unreferenced_disp(&model, disp, true);
        // node 2 は G1 上 t=0.25 → 25.0
        assert!((out[2][0] - 25.0).abs() < 1e-9, "node2={:?}", out[2]);
        // node 3 は最寄り大梁 G2（変位 0）ではなく、二次部材で node 2 に追従 → 25.0
        assert!((out[3][0] - 25.0).abs() < 1e-9, "node3={:?}", out[3]);
    }

    #[test]
    fn 二次部材の連鎖でも主架構に近い側から順に追従する() {
        // node 1(大梁 G1 上, 直付き) → 二次部材 → node 2 → 二次部材 → node 3 の連鎖。
        // node 3 は変位ゼロの別の大梁 G2 に近く、単純射影では G2 へ張り付くが、
        // 連鎖を辿って node 1 の変位へ揃うことを確認する（伝播がないと誤る配置）。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0])); // G1 端
        model.nodes.push(test_node(1, [4000.0, 0.0, 0.0])); // G1 端（直付きアンカー元）
        model.nodes.push(test_node(2, [4000.0, 2000.0, 0.0])); // 連鎖 1 段目
        model.nodes.push(test_node(3, [4000.0, 4000.0, 0.0])); // 連鎖 2 段目（G2 から 1000）
        model.nodes.push(test_node(4, [0.0, 5000.0, 0.0])); // G2 端
        model.nodes.push(test_node(5, [8000.0, 5000.0, 0.0])); // G2 端
        model.elements.push(test_beam(0, 0, 1)); // G1
        model.elements.push(test_beam(1, 4, 5)); // G2（変位ゼロ）
        model.unassigned_joists.push(test_secondary(1, 2));
        model.unassigned_joists.push(test_secondary(2, 3));

        let disp = vec![
            [8.0, 0.0, 0.0, 0.0, 0.0, 0.0], // 0
            [8.0, 0.0, 0.0, 0.0, 0.0, 0.0], // 1（両端同変位＝剛体移動）
            [0.0; 6],                       // 2（未参照）
            [0.0; 6],                       // 3（未参照）
            [0.0; 6],                       // 4
            [0.0; 6],                       // 5
        ];
        let out = interpolate_unreferenced_disp(&model, disp, true);
        // node 2, 3 とも連鎖を辿って node 1 の変位 8.0 に追従する。
        assert!((out[2][0] - 8.0).abs() < 1e-9, "node2={:?}", out[2]);
        assert!((out[3][0] - 8.0).abs() < 1e-9, "node3={:?}", out[3]);
    }

    #[test]
    fn 剛床マスターの鉛直変位はスレーブ平均で補完される() {
        // マスター（重心）はダミー拘束で Uz=0。スレーブの Uz 平均で表示用に補完し、
        // 面内（Ux/Uy/Rz）は解析結果のまま維持されることを確認する。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 3000.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 3000.0]));
        model.nodes.push(test_node(2, [3000.0, 0.0, 3000.0])); // マスター（重心）
        model
            .constraints
            .push(squid_n_core::model::Constraint::rigid_diaphragm(
                squid_n_core::ids::StoryId(0),
                NodeId(2),
                vec![NodeId(0), NodeId(1)],
            ));
        let disp = vec![
            [1.0, 0.0, -4.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, -6.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.02], // マスターの面内変位（Uz は 0）
        ];
        let out = fill_diaphragm_master_disp_for_display(&model, disp);
        // Uz は (-4 + -6)/2 = -5 に補完される。
        assert!((out[2][2] + 5.0).abs() < 1e-12, "Uz={}", out[2][2]);
        // 面内（Ux, Rz）は変更されない。
        assert!((out[2][0] - 1.0).abs() < 1e-12, "Ux={}", out[2][0]);
        assert!((out[2][5] - 0.02).abs() < 1e-12, "Rz={}", out[2][5]);
    }

    #[test]
    fn 梁の内部たわみで変形スケール上限が算定される() {
        // 端部に等・逆回転（θz=±0.01）を与えた L=6000 の梁。両端並進 0 のため弦は
        // 直線で、弦からの逸脱＝Hermite たわみ w(ξ)=0.01·L·ξ(1−ξ)。9 分割の内部
        // サンプルでの最大は ξ=4/9,5/9 の 0.01·6000·(20/81)。
        // 上限 = 0.1·L / w_max = 0.1·6000·81 / (0.01·6000·20) = 40.5。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));
        let disp = vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
            [0.0, 0.0, 0.0, 0.0, 0.0, -0.01],
        ];
        let limit = beam_deflection_scale_limit(&model, &disp).expect("上限が算定される");
        assert!((limit - 40.5).abs() < 1e-9, "limit={}", limit);
    }

    #[test]
    fn 変位ゼロなら梁スケール上限は無し() {
        // たわみが生じない（全変位ゼロ）と制約する梁がなく None を返す。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));
        let disp = vec![[0.0; 6], [0.0; 6]];
        assert!(beam_deflection_scale_limit(&model, &disp).is_none());
    }

    #[test]
    fn 表示倍率は変位なし又は全ゼロでゼロ() {
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [10000.0, 0.0, 0.0]));
        let size = model_bbox_size(&model);
        assert_eq!(deform_display_scale(&model, None, size, true, 1.0), 0.0);
        let zero = vec![[0.0; 6], [0.0; 6]];
        assert_eq!(
            deform_display_scale(&model, Some(&zero), size, true, 1.0),
            0.0
        );
    }

    #[test]
    fn 内部たわみオフの表示倍率はbox基準に手動係数を掛ける() {
        // 梁要素がなく（＝梁スパン基準は無関係）、box 基準 × 手動係数になる。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [10000.0, 0.0, 0.0]));
        let disp = vec![[0.0; 6], [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
        let size = model_bbox_size(&model); // 対角 10000
                                            // box 基準 = 0.1·10000 / 100 = 10、手動係数 2 → 20。
        let s = deform_display_scale(&model, Some(&disp), size, false, 2.0);
        assert!((s - 20.0).abs() < 1e-9, "s={}", s);
    }

    #[test]
    fn 内部たわみオンは梁スパン上限で倍率が制限される() {
        // box 基準が梁スパン上限より大きい配置。ON では min(box, 梁スパン) になる。
        let mut model = Model::default();
        model.nodes.push(test_node(0, [0.0, 0.0, 0.0]));
        model.nodes.push(test_node(1, [6000.0, 0.0, 0.0]));
        model.elements.push(test_beam(0, 0, 1));
        // 端部回転で内部たわみを生み、並進は微小にして box 基準を大きくする。
        let disp = vec![
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
            [0.001, 0.0, 0.0, 0.0, 0.0, -0.01],
        ];
        let size = model_bbox_size(&model); // 6000
        let on = deform_display_scale(&model, Some(&disp), size, true, 1.0);
        let off = deform_display_scale(&model, Some(&disp), size, false, 1.0);
        // OFF は box 基準のみ、ON は梁スパン上限（前掲テストの 40.5）も併用。
        assert!(on < off, "on={on} off={off}");
        assert!((on - 40.5).abs() < 1e-6, "on={on}");
    }
}
