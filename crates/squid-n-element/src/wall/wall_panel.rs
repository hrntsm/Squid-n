//! 耐震壁（壁エレメントモデル）要素（RC規準の耐震壁）。
//!
//! 鉛直の梁要素（壁柱＝間柱）を両端ピンの剛梁ではさみ込んだ 4 節点 24 自由度要素。
//! 剛梁と壁柱は剛接合、剛梁の両端はピン接合のため、四隅節点の並進のみが
//! 壁柱端の並進（両隅の平均）と回転（剛梁の剛体回転＝両隅の変位差/剛梁長）に
//! 伝達され、四隅節点の回転自由度には剛性を与えない（＝ピン）。
//! 剛梁は実要素ではなく、この変換（剛域変換に相当）で表現する。
//!
//! 壁柱の断面性能:
//! - 軸剛性: 壁板断面積 t·lw に鉄筋剛性を考慮（壁筋比 ps を縦横共通とみなし
//!   (1+(n−1)·ps) を乗じる近似。n=Es/Ec）
//! - 曲げ剛性: 壁板断面の面内断面2次モーメント t·lw³/12（側柱のローカル I は
//!   不算入）に同係数を乗じる
//! - せん断剛性: (壁板断面＋側柱断面)/κ に開口低減率 r を乗じる。
//!   κ は側柱がある場合 I 形断面の形状係数（`wall_shear_shape_factor`、
//!   ξ・η の定義は要原典照合）、無い場合は矩形の 1.2
//!
//! 上下大梁の剛性倍率（既定 100 倍）は梁要素側（`beam.rs`）で扱う。
//! 側柱の面内両端ピン化は `side_column.rs`（方向別端部解放の静縮約）で扱う。

use crate::beam::BeamElement;
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, Model};
use squid_n_core::section_shape::{SectionShape, E_STEEL, KAPPA_RC};

/// 耐震壁（壁エレメントモデル）。
pub struct WallPanelElement {
    /// [下辺a, 下辺b, 上辺a, 上辺b]（a→b が剛梁の軸方向。上下で対応付け済み）
    nodes: [NodeId; 4],
    /// 壁柱（仮想中央柱。上下剛梁の中点を結ぶ）
    column: BeamElement,
    /// 壁柱端 12 自由度 ← 四隅 24 自由度 の変換行列 A（row-major 12×24）。
    /// 四隅の回転自由度に対応する列は常に 0（ピン）。
    a_mat: Vec<f64>,
    /// 質量算定用の壁板総質量 [質量単位]
    mass_total: f64,
    /// 確定変位（四隅 24 自由度、グローバル系）。commit_state で trial から確定。
    committed_disp: [f64; 24],
    /// トライアル変位（四隅 24 自由度、グローバル系）。Newton 反復中も蓄積され、
    /// internal_force はこちらを参照する（beam/behavior.rs と同じトライアル追従規約）。
    trial_disp: [f64; 24],
    /// 面内せん断の終局強度 Qu [N]。`0` 以下は**降伏しない**（線形弾性。許容応力度
    /// 計算など弾性解析経路）。保有水平耐力（プッシュオーバー）では
    /// [`crate::factory::build_nonlinear_behavior`] が耐震壁のせん断終局強度を与え、
    /// 面内せん断を弾完全塑性として頭打ちにする。
    qu_shear: f64,
    /// 面内せん断モードベクトル p（24 自由度）。上辺 2 節点の並進を壁面内水平方向
    /// `ex_bottom` へ 1.0 ずつ与えたもの。`pᵀ·f` は上辺が伝達する面内水平力に等しく、
    /// `u − γp·p` で塑性すべりを差し引く（下記 [`WallPanelElement::shear_return_map`]）。
    shear_mode: [f64; 24],
    /// 確定塑性せん断すべり γp [mm]。
    committed_slip: f64,
    /// トライアル塑性せん断すべり γp [mm]。
    trial_slip: f64,
}

/// 壁エレメント（4 節点）の幾何。
///
/// 節点は入力順に依らず**標高 z で下辺 2 節点・上辺 2 節点に分ける**（`ElementData::nodes`
/// の並び順は任意であり、下辺が先頭に来る保証はない）。上辺は下辺 a に近い方を a として
/// 対応付ける。
///
/// 壁長 `lw` は**上下辺長さの平均**とする（台形壁では上下辺長が異なるため、
/// 一方の辺だけでは代表長さにならない）。耐力壁の平均せん断応力度
/// τu = Q/(t·lw) など、壁の断面量を要する算定は本構造体を用いて要素実装と同じ
/// 幾何を共有する。
pub struct WallPanelGeometry {
    /// 下辺の 2 節点（a→b）
    pub bottom: [NodeId; 2],
    /// 上辺の 2 節点（下辺 a に対応する側が先）
    pub top: [NodeId; 2],
    /// 下辺長さ
    pub lw_bottom: f64,
    /// 上辺長さ
    pub lw_top: f64,
    /// 壁長 lw = (下辺長 + 上辺長)/2（台形壁に対応）
    pub lw: f64,
    /// 壁高さ h（上下辺の中点間距離）
    pub h: f64,
    /// 下辺の軸方向単位ベクトル（a→b）
    pub ex_bottom: [f64; 3],
    /// 下辺中点
    pub bottom_center: [f64; 3],
    /// 上辺中点
    pub top_center: [f64; 3],
}

/// 壁エレメント（4 節点）の幾何を算定する（[`WallPanelGeometry`]）。
///
/// 4 節点未満・節点参照が欠落・退化（辺長や高さが 0）の場合は `None`。
pub fn wall_panel_geometry(data: &ElementData, model: &Model) -> Option<WallPanelGeometry> {
    if data.nodes.len() < 4 {
        return None;
    }
    let ids: Vec<NodeId> = data.nodes.iter().take(4).copied().collect();
    let coords: Vec<[f64; 3]> = ids
        .iter()
        .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
        .collect::<Option<Vec<_>>>()?;

    // z で下辺 2 節点・上辺 2 節点に分ける（入力順には依存しない）。
    let mut order: Vec<usize> = (0..4).collect();
    order.sort_by(|&a, &b| coords[a][2].partial_cmp(&coords[b][2]).unwrap());
    let (b0, b1, t0, t1) = (order[0], order[1], order[2], order[3]);

    // 下辺の軸方向 a→b
    let (pa, pb) = (coords[b0], coords[b1]);
    let ex_bot = unit(sub(pb, pa))?;
    // 上辺は下辺の a に近い方を a とする（対応付け）
    let (ta, tb) = {
        let d0 = dot(sub(coords[t0], pa), ex_bot).abs();
        let d1 = dot(sub(coords[t1], pa), ex_bot).abs();
        if d0 <= d1 {
            (t0, t1)
        } else {
            (t1, t0)
        }
    };

    let lw_bot = norm(sub(pb, pa));
    let lw_top = norm(sub(coords[tb], coords[ta]));
    let bc = mid(pa, pb);
    let tc = mid(coords[ta], coords[tb]);
    let h = norm(sub(tc, bc));
    if lw_bot <= 0.0 || lw_top <= 0.0 || h <= 0.0 {
        return None;
    }
    Some(WallPanelGeometry {
        bottom: [ids[b0], ids[b1]],
        top: [ids[ta], ids[tb]],
        lw_bottom: lw_bot,
        lw_top,
        // 台形壁に対応するため上下辺長さの平均を壁長とする。
        lw: 0.5 * (lw_bot + lw_top),
        h,
        ex_bottom: ex_bot,
        bottom_center: bc,
        top_center: tc,
    })
}

impl WallPanelElement {
    /// 生成。4 節点未満・寸法/断面が不定の場合は None
    /// （呼び出し側は従来の暫定等価梁へフォールバックする）。
    pub fn try_new(data: &ElementData, model: &Model) -> Option<Self> {
        Self::try_new_scaled(data, model, 1.0)
    }

    /// 剛性スケール付き生成。耐震壁不成立（フレーム内雑壁）の壁は剛性を
    /// 周辺部材へ算入するため、壁要素自体は `stiffness_scale`（微小値）で
    /// 実質無剛性とし、質量のみを保持する（RC規準の耐震壁。
    /// フレーム内雑壁のモデル化）。
    pub(crate) fn try_new_scaled(
        data: &ElementData,
        model: &Model,
        stiffness_scale: f64,
    ) -> Option<Self> {
        // 幾何（下辺・上辺の対応付け、壁長 lw＝上下辺の平均、高さ h）は
        // [`wall_panel_geometry`] に集約する（保有水平耐力の τu 算定など要素外の
        // 利用と同一の幾何を共有し、定義が食い違わないようにする）。
        let geom = wall_panel_geometry(data, model)?;
        let (ids_b0, ids_b1) = (geom.bottom[0], geom.bottom[1]);
        let (ids_ta, ids_tb) = (geom.top[0], geom.top[1]);
        let coord_of =
            |nid: NodeId| -> Option<[f64; 3]> { model.nodes.get(nid.index()).map(|n| n.coord) };
        let ex_bot = geom.ex_bottom;
        let ex_top = unit(sub(coord_of(ids_tb)?, coord_of(ids_ta)?))?;
        let (bc, tc) = (geom.bottom_center, geom.top_center);
        let h = geom.h;
        let lw = geom.lw;

        // 壁板厚: RcWall 形状 → Section.thickness → Section.width の順で採用
        let sec = data
            .section
            .and_then(|sid| model.sections.get(sid.index()))?;
        let t = match &sec.shape {
            Some(SectionShape::RcWall { thickness, .. }) => *thickness,
            _ => sec.thickness.unwrap_or(sec.width),
        };
        if t <= 0.0 {
            return None;
        }
        let mat = data
            .material
            .and_then(|mid| model.materials.get(mid.index()))?;

        // 開口低減率 r（複数開口モード考慮）。r=0 でせん断断面積が 0 になると
        // φ 項が NaN になるため微小値を下限とする。
        let r = crate::factory::wall_opening_reduction(data, model).max(1e-6);

        // 鉄筋剛性の考慮（壁筋比 ps を縦横共通とみなす近似）: (1+(n−1)·ps)
        let ps = match &sec.shape {
            Some(SectionShape::RcWall { ps, .. }) => (*ps).max(0.0),
            _ => 0.0,
        };
        let rebar_factor = if mat.fc.is_some() && mat.young > 0.0 && ps > 0.0 {
            1.0 + (E_STEEL / mat.young - 1.0) * ps
        } else {
            1.0
        };

        // 側柱（壁の鉛直辺の 2 節点を両端に持つ鉛直 Beam 部材）を収集し、
        // せん断断面への算入と I 形形状係数 κ の算定に用いる。
        let edge_pairs = [[ids_b0, ids_ta], [ids_b1, ids_tb]];
        let mut col_area_sum = 0.0;
        let mut col_depth_sum = 0.0; // 沿壁方向せい（両側の和）
        let mut col_width_max: f64 = 0.0;
        let mut col_main_at: f64 = 0.0;
        // 側柱断面をせん断断面へ算入してよいのは、その側柱が面内両端ピン化される
        // （＝面内せん断を負担しない）場合に限る。ピン化条件
        // （`side_column::wall_side_column_release`）と同じ判定をここでも課さないと、
        // ピン化されない柱の断面を壁が肩代わりして**面内せん断の二重計上**になる。
        let side_columns_released = crate::side_column::is_rc_wall(data, model)
            && crate::misc_wall::wall_is_seismic(data, model);
        for e in &model.elements {
            if !side_columns_released {
                break;
            }
            if !matches!(e.kind, squid_n_core::model::ElementKind::Beam) || e.nodes.len() < 2 {
                continue;
            }
            // 鉛直材のみ（ピン化条件と同じ規約）。
            if let (Some(a), Some(b)) = (
                model.nodes.get(e.nodes[0].index()),
                model.nodes.get(e.nodes[1].index()),
            ) {
                let (dx, dy, dz) = (
                    b.coord[0] - a.coord[0],
                    b.coord[1] - a.coord[1],
                    b.coord[2] - a.coord[2],
                );
                if dz.abs() <= (dx.abs() + dy.abs()) * 0.5 {
                    continue;
                }
            } else {
                continue;
            }
            let (n0, n1) = (e.nodes[0], e.nodes[1]);
            let is_edge = edge_pairs
                .iter()
                .any(|p| (p[0] == n0 && p[1] == n1) || (p[0] == n1 && p[1] == n0));
            if !is_edge {
                continue;
            }
            if let Some(cs) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                col_area_sum += cs.area;
                col_depth_sum += cs.depth.max(cs.width);
                col_width_max = col_width_max.max(cs.width.min(cs.depth).max(t));
                // 終局せん断強度 Qu の等価引張鉄筋比 pte 用に、側柱 1 本の主筋量を採る
                // （引張側最端の柱 1 本。両側柱のうち大きい方を代表とする）。
                if let Some(SectionShape::RcRect { rebar, .. }) = cs.shape.as_ref() {
                    col_main_at =
                        col_main_at.max(squid_n_core::section_shape::bar_set_area(&rebar.main_x));
                }
            }
        }
        // κ: 側柱があれば I 形断面の形状係数（ξ=内法長さ/外面間全長、η=t/側柱幅。
        // 定義は要原典照合）、無ければ矩形の 1.2。
        // κ: 側柱があれば平面 I 形断面（ウェブ＝壁板、フランジ＝側柱）の厳密な
        // せん断形状係数 κ = A/I²·∫Q²/b dy、無ければ矩形の 1.2。
        // 従来の閉形式（`wall_shear_shape_factor`）は記号定義が原典で確認できず、
        // η=1（側柱幅＝壁厚＝一様矩形）でも 0.6(1+ξ) を返すなど内部整合性を欠き、
        // 側柱が大きいほど κ が 1.2 から**減少**して as_y が総断面積を超える
        // 非物理な値（面内せん断剛性が最大 5.8 倍過大）を与えていた。
        let dc_each = col_depth_sum / 2.0;
        let kappa = if col_area_sum > 0.0 && col_width_max > 0.0 && dc_each > 0.0 {
            squid_n_core::section_shape::wall_shear_shape_factor_isection(
                lw + dc_each,
                dc_each,
                col_width_max,
                t,
            )
        } else {
            KAPPA_RC
        };

        let area = t * lw;
        let as_gross = area + col_area_sum;
        let column = BeamElement {
            id: data.id,
            e: mat.young * stiffness_scale,
            g: mat.shear_modulus() * stiffness_scale,
            a: area * rebar_factor,
            a_mass: area,
            // 面内曲げ（局所 z 軸まわり）= t·lw³/12、面外 = lw·t³/12
            iy: lw * t.powi(3) / 12.0,
            iz: t * lw.powi(3) / 12.0 * rebar_factor,
            j: lw * t.powi(3) / 3.0,
            // 面内せん断（局所 y 方向）: (壁板+側柱)/κ に開口低減 r を考慮
            as_y: r * as_gross / kappa,
            // 面外せん断にも開口低減を適用する（開口は面外剛性も低下させる。
            // 従来は面内のみに乗じており面外は取りこぼしていた）。
            as_z: r * area / KAPPA_RC,
            length: h,
            density: mat.density,
            nodes: [ids_b0, ids_ta],
            axis: LocalFrame::from_nodes(bc, tc, ex_bot),
            rigid: Default::default(),
            end_cond: [
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
            ],
            eval_sections: vec![0.0, 0.5, 1.0],
            section: data.section,
            material: data.material,
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
        };

        // 変換行列 A（壁柱端 ← 四隅並進）。
        // 並進: u_c = (u_a + u_b)/2
        // 回転: ω = ex × (u_b − u_a)/lw（剛梁の剛体回転。剛梁軸まわり成分は
        //        ピンのため伝達されず 0）
        let mut a_mat = vec![0.0; 12 * 24];
        let corner_slot = |idx: usize| -> usize {
            // nodes 配列 [b_a, b_b, t_a, t_b] 中の位置 → 24 自由度中のオフセット
            idx * 6
        };
        let node_order = [ids_b0, ids_b1, ids_ta, ids_tb];
        let slot_of = |orig: NodeId| -> usize {
            node_order
                .iter()
                .position(|&x| x == orig)
                .expect("node_order は 4 節点の並べ替え")
        };
        let mut fill_end = |col_base: usize, ca: NodeId, cb: NodeId, ex: [f64; 3], lw_e: f64| {
            let (sa, sb) = (corner_slot(slot_of(ca)), corner_slot(slot_of(cb)));
            for tdof in 0..3 {
                a_mat[(col_base + tdof) * 24 + sa + tdof] += 0.5;
                a_mat[(col_base + tdof) * 24 + sb + tdof] += 0.5;
            }
            // ω_i = Σ_jk ε_ijk・ex_j・(u_b − u_a)_k / lw
            for i in 0..3 {
                for j in 0..3 {
                    for k in 0..3 {
                        let e = levi_civita(i, j, k);
                        if e == 0.0 {
                            continue;
                        }
                        let c = e * ex[j] / lw_e;
                        a_mat[(col_base + 3 + i) * 24 + sb + k] += c;
                        a_mat[(col_base + 3 + i) * 24 + sa + k] -= c;
                    }
                }
            }
        };
        fill_end(0, ids_b0, ids_b1, ex_bot, geom.lw_bottom);
        fill_end(6, ids_ta, ids_tb, ex_top, geom.lw_top);

        // 面内せん断モード p: 上辺 2 節点（スロット 2,3）の並進を ex_bot 方向へ 1.0。
        // pᵀ·f = 上辺 2 節点の ex 方向内力の和 ＝ 壁が伝達する面内水平力。
        let mut shear_mode = [0.0; 24];
        for slot in [2usize, 3usize] {
            for k in 0..3 {
                shear_mode[slot * 6 + k] = ex_bot[k];
            }
        }

        Some(Self {
            nodes: [ids_b0, ids_b1, ids_ta, ids_tb],
            column,
            a_mat,
            // 質量は自重側の控除規約と揃える: **開口面積を控除し、開口部（サッシ等）の
            // 重量を加算**する。従来は gross（t·lw·h、開口控除なし）で、節点質量からの
            // 控除側（`squid_n_load::story_gen` は開口控除・サッシ重量加算済み）と
            // 食い違い、地震用質量が恒常的に過大だった。
            // 残差: 自重側は周辺柱梁の内法寸法補正（`wall_clear_area_factor`）も
            // 行うが、その算定は squid-n-load 側にあり本クレートからは参照できない
            // （V&V §2.4 の残課題）。
            mass_total: {
                let attr = model.wall_attrs.iter().find(|a| a.elem == data.id);
                let opening_area = attr.map(|a| a.total_opening_area()).unwrap_or(0.0);
                let opening_weight = attr.map(|a| a.opening_weight).unwrap_or(0.0);
                let net_area = (lw * h - opening_area).max(0.0);
                (mat.density * t * net_area + opening_weight / squid_n_core::units::GRAVITY_MM_S2)
                    .max(0.0)
            },
            committed_disp: [0.0; 24],
            trial_disp: [0.0; 24],
            // 既定は弾性（降伏なし）。非線形経路が `with_shear_capacity` で与える。
            qu_shear: 0.0,
            shear_mode,
            committed_slip: 0.0,
            trial_slip: 0.0,
        })
    }

    /// 耐震壁の面内せん断終局強度 Qu [N]（保有水平耐力用）。
    ///
    /// [`squid_n_core::rc_wall_capacity::wall_shear_ultimate`]（荒川mean式系）に、
    /// 壁エレメントの幾何・配筋から組み立てた入力を与える。開口低減は**耐力用**
    /// r2 = 1−max(r0, l0/lw, h0/h)（剛性用 r1 = 1−1.25·r0 とは別式）。
    ///
    /// 主な仮定（要・原典照合）:
    /// - 等価壁厚 te は壁厚 t と同値とする。
    /// - 引張側柱の主筋量 at は側柱（`SectionShape::RcRect`）の `main_x` 総断面積。
    ///   側柱が無い／配筋が取れない場合は、壁の縦筋が一様配筋であるとみなして
    ///   `at = ps·te·d`（＝等価引張鉄筋比 pte = 100·ps \[%\]）とする。
    /// - 横筋比 Pwh は壁筋比 ps（縦横共通とみなす近似）、σwh は SD295 相当 295。
    /// - せん断スパン比 M/(Q·D) は壁の h/D（適用範囲 1.0〜3.0 にクランプ）。
    /// - 軸方向応力度 σ0 は 0（軸力は Qu を増やすため、0 とするのは安全側）。
    ///
    /// 算定できない場合（Fc 未設定など）は 0.0 を返し、呼び出し側は弾性のままとする。
    #[allow(clippy::too_many_arguments)]
    fn shear_capacity(
        fc: Option<f64>,
        t: f64,
        lw: f64,
        h: f64,
        ps: f64,
        dc_each: f64,
        col_main_at: f64,
        opening: Option<(f64, f64)>,
    ) -> f64 {
        let Some(fc) = fc else {
            return 0.0;
        };
        if fc <= 0.0 || t <= 0.0 || lw <= 0.0 || h <= 0.0 {
            return 0.0;
        }
        let te = t;
        let d_wall = lw + dc_each;
        let d_eff = d_wall - dc_each / 2.0;
        if d_eff <= 0.0 {
            return 0.0;
        }
        // 側柱主筋が取れない壁は、縦筋一様配筋とみなし pte = 100·ps 相当とする。
        let at = if col_main_at > 0.0 {
            col_main_at
        } else {
            ps.max(0.0) * te * d_eff
        };
        if at <= 0.0 {
            return 0.0;
        }
        squid_n_core::rc_wall_capacity::wall_shear_ultimate(
            &squid_n_core::rc_wall_capacity::RcWallShearInput {
                fc,
                te,
                t,
                d_wall,
                dc_compression: dc_each,
                tension_column_at: at,
                sigma_wh: 295.0,
                pwh_ratio: ps.max(0.0),
                sigma_0: 0.0,
                shear_span_ratio: h / d_wall,
                high_strength_shear_rebar: false,
                opening: opening.map(|(l0, h0)| (l0, h0, h, lw)),
            },
        )
    }

    /// この壁の面内せん断終局強度 Qu [N] を、要素と同じ幾何・配筋から算定する
    /// （非線形経路が [`Self::with_shear_capacity`] へ渡す値）。
    pub(crate) fn shear_capacity_of(data: &ElementData, model: &Model) -> f64 {
        let Some(geom) = wall_panel_geometry(data, model) else {
            return 0.0;
        };
        let Some(sec) = data.section.and_then(|sid| model.sections.get(sid.index())) else {
            return 0.0;
        };
        let (t, ps) = match &sec.shape {
            Some(SectionShape::RcWall { thickness, ps }) => (*thickness, (*ps).max(0.0)),
            _ => (sec.thickness.unwrap_or(sec.width), 0.0),
        };
        let fc = data
            .material
            .and_then(|mid| model.materials.get(mid.index()))
            .and_then(|m| m.fc);
        // 側柱（壁の鉛直辺に取り付く柱）の沿壁方向せい・主筋量。
        let edge_pairs = [[geom.bottom[0], geom.top[0]], [geom.bottom[1], geom.top[1]]];
        let mut col_depth_sum = 0.0;
        let mut col_main_at: f64 = 0.0;
        for e in &model.elements {
            if !matches!(e.kind, squid_n_core::model::ElementKind::Beam) || e.nodes.len() < 2 {
                continue;
            }
            let (n0, n1) = (e.nodes[0], e.nodes[1]);
            if !edge_pairs
                .iter()
                .any(|p| (p[0] == n0 && p[1] == n1) || (p[0] == n1 && p[1] == n0))
            {
                continue;
            }
            if let Some(cs) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                col_depth_sum += cs.depth.max(cs.width);
                if let Some(SectionShape::RcRect { rebar, .. }) = cs.shape.as_ref() {
                    col_main_at =
                        col_main_at.max(squid_n_core::section_shape::bar_set_area(&rebar.main_x));
                }
            }
        }
        // 開口寸法。複数開口は**面積等価**の 1 開口へまとめる（技術基準解説書:
        // 全開口面積と等しい面積を有し、全開口の幅の和と等しい幅を有する開口と
        // みなす → lo = Σli、ho = Σ(li·hi)/lo）。モード別の開口列の作り方
        // （包絡／面積等価／自動）は `opening_dims_for` に従う。
        let opening = model
            .wall_attrs
            .iter()
            .find(|w| w.elem == data.id)
            .and_then(|a| a.opening_dims_for(model.multi_opening_mode))
            .and_then(|dims| {
                let lo: f64 = dims.iter().map(|(l, _)| *l).sum();
                let area: f64 = dims.iter().map(|(l, h)| l * h).sum();
                (lo > 0.0 && area > 0.0).then_some((lo, area / lo))
            });
        Self::shear_capacity(
            fc,
            t,
            geom.lw,
            geom.h,
            ps,
            col_depth_sum / 2.0,
            col_main_at,
            opening,
        )
    }

    /// 面内せん断の終局強度 Qu [N] を与えて弾完全塑性化する（保有水平耐力用）。
    /// `qu <= 0` は弾性のまま（降伏しない）。
    pub(crate) fn with_shear_capacity(mut self, qu: f64) -> Self {
        self.qu_shear = qu.max(0.0);
        self
    }

    /// 面内せん断の弾完全塑性リターンマッピング。
    ///
    /// せん断モード p（[`Self::shear_mode`]）に沿う塑性すべり γp を導入し、
    /// 有効変位を `u_eff = u − γp·p` とする。弾性内力 `f = K·u_eff` に対し
    /// 壁が伝達する面内水平力は `Q = pᵀ·f` であり、`|Q| > Qu` のとき
    /// `Δγp = (|Q| − Qu)·sign(Q)/k_s`（`k_s = pᵀ·K·p`）だけ γp を増やすと
    /// `|Q| = Qu` に戻る（Q は γp に線形なため 1 回の補正で厳密に満たす）。
    ///
    /// 戻り値は `(γp, 降伏しているか)`。`qu_shear <= 0` は常に弾性。
    fn shear_return_map(&self, k: &LocalMat, u24: &[f64; 24]) -> (f64, bool) {
        if self.qu_shear <= 0.0 {
            return (0.0, false);
        }
        // k_s = pᵀ K p
        let kp = Self::mat_vec(k, &self.shear_mode);
        let k_s: f64 = self
            .shear_mode
            .iter()
            .zip(kp.iter())
            .map(|(p, v)| p * v)
            .sum();
        if k_s <= 0.0 {
            return (self.committed_slip, false);
        }
        // 確定すべりを差し引いた弾性試行での面内水平力。
        let mut u_eff = *u24;
        for (ue, p) in u_eff.iter_mut().zip(self.shear_mode.iter()) {
            *ue -= self.committed_slip * p;
        }
        let f = Self::mat_vec(k, &u_eff);
        let q_trial: f64 = self
            .shear_mode
            .iter()
            .zip(f.iter())
            .map(|(p, v)| p * v)
            .sum();
        if q_trial.abs() <= self.qu_shear {
            return (self.committed_slip, false);
        }
        let d_gamma = (q_trial.abs() - self.qu_shear) * q_trial.signum() / k_s;
        (self.committed_slip + d_gamma, true)
    }

    /// K·v（24 次）。
    fn mat_vec(k: &LocalMat, v: &[f64; 24]) -> [f64; 24] {
        let mut out = [0.0; 24];
        for (i, o) in out.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, vj) in v.iter().enumerate() {
                if *vj != 0.0 {
                    s += k.get(i, j) * vj;
                }
            }
            *o = s;
        }
        out
    }

    /// 全体系 24×24 剛性 K = Aᵀ·K_col·A。
    fn stiffness_24(&self) -> LocalMat {
        let k12 = self.column.axis.to_global(&self.column.local_stiffness());
        let mut k = LocalMat::zeros(24);
        // K = Aᵀ K12 A
        for p in 0..24 {
            for q in 0..24 {
                let mut s = 0.0;
                for i in 0..12 {
                    let aip = self.a_mat[i * 24 + p];
                    if aip == 0.0 {
                        continue;
                    }
                    for j in 0..12 {
                        let ajq = self.a_mat[j * 24 + q];
                        if ajq != 0.0 {
                            s += aip * k12.get(i, j) * ajq;
                        }
                    }
                }
                if s != 0.0 {
                    k.set(p, q, s);
                }
            }
        }
        k
    }

    /// 四隅変位 24 → 壁柱端変位 12（全体系）。
    fn to_column_disp(&self, u24: &[f64]) -> [f64; 12] {
        let mut u12 = [0.0; 12];
        for (i, ui) in u12.iter_mut().enumerate() {
            let mut s = 0.0;
            for p in 0..24 {
                s += self.a_mat[i * 24 + p] * u24[p];
            }
            *ui = s;
        }
        u12
    }
}

impl ElementBehavior for WallPanelElement {
    fn n_dof(&self) -> usize {
        24
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        let k = self.stiffness_24();
        let (_, yielded) = self.shear_return_map(&k, &self.trial_disp);
        if !yielded {
            return k;
        }
        // 面内せん断が終局に達している間は、その方向の剛性を取り除いた整合接線
        // K_t = K − (K·p)(K·p)ᵀ/(pᵀ·K·p) とする（弾完全塑性のコンシステント接線）。
        let kp = Self::mat_vec(&k, &self.shear_mode);
        let k_s: f64 = self
            .shear_mode
            .iter()
            .zip(kp.iter())
            .map(|(p, v)| p * v)
            .sum();
        if k_s <= 0.0 {
            return k;
        }
        let mut kt = LocalMat::zeros(24);
        for i in 0..24 {
            for j in 0..24 {
                let v = k.get(i, j) - kp[i] * kp[j] / k_s;
                if v != 0.0 {
                    kt.set(i, j, v);
                }
            }
        }
        kt
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        // f = K24 · (u − γp·p)（トライアル追従。beam/behavior.rs と同じ規約）。
        // γp は面内せん断の塑性すべりで、終局せん断強度 Qu を超える水平力を
        // 負担しないよう [`Self::shear_return_map`] が求める。Qu 未設定
        // （弾性解析経路）では γp=0 で従来どおりの線形弾性。
        let k = self.stiffness_24();
        let (slip, _) = self.shear_return_map(&k, &self.trial_disp);
        let mut u_eff = self.trial_disp;
        for (ue, p) in u_eff.iter_mut().zip(self.shear_mode.iter()) {
            *ue -= slip * p;
        }
        let fv = Self::mat_vec(&k, &u_eff);
        LocalVec {
            data: smallvec::SmallVec::from_slice(&fv),
        }
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        for i in 0..24.min(du.data.len()) {
            self.trial_disp[i] += du.data[i];
        }
        // 塑性すべりはトライアル変位から都度求め直す（経路依存の単調載荷を前提。
        // commit 時に確定値へ移す）。
        let k = self.stiffness_24();
        let (slip, _) = self.shear_return_map(&k, &self.trial_disp);
        self.trial_slip = slip;
        if commit {
            self.committed_disp = self.trial_disp;
            self.committed_slip = self.trial_slip;
        }
    }

    fn commit_state(&mut self) {
        self.committed_disp = self.trial_disp;
        self.committed_slip = self.trial_slip;
    }

    fn revert_state(&mut self) {
        self.trial_disp = self.committed_disp;
        self.trial_slip = self.committed_slip;
    }

    fn snapshot_state(&self) -> Box<dyn std::any::Any> {
        Box::new((
            self.committed_disp,
            self.trial_disp,
            self.committed_slip,
            self.trial_slip,
        ))
    }

    fn restore_state(&mut self, state: &dyn std::any::Any) {
        if let Some((committed, trial, cslip, tslip)) =
            state.downcast_ref::<([f64; 24], [f64; 24], f64, f64)>()
        {
            self.committed_disp = *committed;
            self.trial_disp = *trial;
            self.committed_slip = *cslip;
            self.trial_slip = *tslip;
        }
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        bincode::serialize(&(self.committed_disp, self.trial_disp)).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        // 旧チェックポイント（変位未収録・空バイト列）は「状態なし」として許容する。
        if data.is_empty() {
            return Ok(());
        }
        let (committed, trial): ([f64; 24], [f64; 24]) = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.committed_disp = committed;
        self.trial_disp = trial;
        Ok(())
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        // 壁板質量を四隅の並進へ 1/4 ずつ集中（Consistent 指定も同じ扱い）
        let mut mm = LocalMat::zeros(24);
        let m_node = self.mass_total / 4.0;
        for i in 0..4 {
            let bo = i * 6;
            for d in 0..3 {
                mm.set(bo + d, bo + d, m_node);
            }
        }
        mm
    }

    fn geometric_stiffness(&self, _n: f64) -> LocalMat {
        LocalMat::zeros(24)
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 24 {
            return None;
        }
        // 壁柱の断面力（N・Q・M）として復元する
        let u12 = self.to_column_disp(&u_elem[..24]);
        Some(self.column.recover_forces(&u12))
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn mid(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2]),
    ]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let l = norm(a);
    if l < 1e-9 {
        None
    } else {
        Some([a[0] / l, a[1] / l, a[2] / l])
    }
}

fn levi_civita(i: usize, j: usize, k: usize) -> f64 {
    match (i, j, k) {
        (0, 1, 2) | (1, 2, 0) | (2, 0, 1) => 1.0,
        (0, 2, 1) | (2, 1, 0) | (1, 0, 2) => -1.0,
        _ => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node};
    use squid_n_core::section_shape::SectionShape;

    /// 4000×3000×t150 の壁（X-Z 面内）を持つモデル。
    fn make_wall_model() -> (Model, ElementData) {
        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        };
        let shape = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        let model = Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 0.0]),
                make_node(1, [4000.0, 0.0, 0.0]),
                make_node(2, [4000.0, 0.0, 3000.0]),
                make_node(3, [0.0, 0.0, 3000.0]),
            ],
            sections: vec![shape.to_section(SectionId(0), "W150".into())],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            ..Default::default()
        };
        let data = ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        (model, data)
    }

    fn energy(k: &LocalMat, u: &[f64; 24]) -> f64 {
        let mut s = 0.0;
        for i in 0..24 {
            for j in 0..24 {
                s += u[i] * k.get(i, j) * u[j];
            }
        }
        s
    }

    #[test]
    fn test_wall_panel_rigid_translation_zero_force() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k = wall.stiffness_24();
        // 全節点に同一並進（剛体移動）→ 力ゼロ
        for dir in 0..3 {
            let mut u = [0.0; 24];
            for n in 0..4 {
                u[n * 6 + dir] = 1.0;
            }
            for i in 0..24 {
                let f: f64 = (0..24).map(|j| k.get(i, j) * u[j]).sum();
                assert!(
                    f.abs() < 1e-6,
                    "剛体移動で内力が生じた: dir={dir} i={i} f={f}"
                );
            }
        }
    }

    #[test]
    fn test_wall_panel_inplane_shear_matches_column() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k = wall.stiffness_24();
        // 上辺 2 節点を面内水平(X)に単位変位（下辺固定・上辺回転 0 = 両端固定柱の
        // せん断変形モード）→ ひずみエネルギ uᵀKu が壁柱の両端固定水平剛性
        // 12EI/((1+φ)h³) と一致する
        let mut u = [0.0; 24];
        u[2 * 6] = 1.0; // 上辺 a の ux
        u[3 * 6] = 1.0; // 上辺 b の ux
        let uku = energy(&k, &u);

        let col = &wall.column;
        let phi = 12.0 * col.e * col.iz / (col.g * col.as_y * col.length * col.length);
        let expected = 12.0 * col.e * col.iz / ((1.0 + phi) * col.length.powi(3));
        assert!(
            (uku - expected).abs() / expected < 1e-6,
            "uKu={uku} expected={expected}"
        );
    }

    #[test]
    fn test_wall_panel_vertical_matches_axial() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k = wall.stiffness_24();
        // 上辺 2 節点を鉛直に単位変位 → EA/h
        let mut u = [0.0; 24];
        u[2 * 6 + 2] = 1.0;
        u[3 * 6 + 2] = 1.0;
        let uku = energy(&k, &u);
        let col = &wall.column;
        let expected = col.e * col.a / col.length;
        assert!(
            (uku - expected).abs() / expected < 1e-6,
            "uKu={uku} expected={expected}"
        );
    }

    #[test]
    fn test_wall_panel_corner_rotations_are_pinned() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k = wall.stiffness_24();
        // 四隅の回転自由度は剛性を持たない（剛梁両端ピン）
        for n in 0..4 {
            for d in 3..6 {
                let idx = n * 6 + d;
                for j in 0..24 {
                    assert!(
                        k.get(idx, j).abs() < 1e-9 && k.get(j, idx).abs() < 1e-9,
                        "回転自由度に剛性: node={n} dof={d}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_wall_panel_opening_reduces_inplane_shear() {
        let (mut model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let k_no = wall.stiffness_24();
        model.wall_attrs.push(squid_n_core::model::WallAttr {
            elem: ElemId(0),
            opening_area: 3.0e6, // 25%
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        });
        let wall_o = WallPanelElement::try_new(&data, &model).unwrap();
        let k_o = wall_o.stiffness_24();
        let mut u = [0.0; 24];
        u[2 * 6] = 1.0;
        u[3 * 6] = 1.0;
        assert!(
            energy(&k_o, &u) < energy(&k_no, &u) * 0.999,
            "開口低減が面内せん断剛性に効いていない"
        );
    }

    /// 鉄筋剛性の考慮: a = t·lw·(1+(n−1)·ps)、n=Es/Ec。
    #[test]
    fn test_wall_panel_rebar_factor() {
        let (model, data) = make_wall_model();
        let wall = WallPanelElement::try_new(&data, &model).unwrap();
        let n = squid_n_core::section_shape::E_STEEL / 23000.0;
        let expected = 150.0 * 4000.0 * (1.0 + (n - 1.0) * 0.0025);
        assert!((wall.column.a - expected).abs() < 1e-6);
        // 質量用は幾何断面のまま
        assert!((wall.column.a_mass - 150.0 * 4000.0).abs() < 1e-9);
    }

    /// 側柱があるとせん断断面に算入され、I 形の形状係数 κ が用いられる。
    #[test]
    fn test_wall_panel_side_columns_increase_shear_area() {
        let (mut model, data) = make_wall_model();
        let wall_plain = WallPanelElement::try_new(&data, &model).unwrap();

        // 両側の鉛直辺(節点0-3・1-2)に 600×600 の側柱を追加
        let col_shape = SectionShape::RcRect {
            b: 600.0,
            d: 600.0,
            rebar: squid_n_core::section_shape::RcRebar {
                main_x: squid_n_core::section_shape::BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: squid_n_core::section_shape::BarSet {
                    count: 8,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: squid_n_core::section_shape::ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                    grade: None,
                },
            },
        };
        model
            .sections
            .push(col_shape.to_section(SectionId(1), "C600".into()));
        for (eid, n0, n1) in [(1u32, 0u32, 3u32), (2, 1, 2)] {
            model.elements.push(ElementData {
                id: ElemId(eid),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
                section: Some(SectionId(1)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        let wall_cols = WallPanelElement::try_new(&data, &model).unwrap();
        // (壁板+側柱2本)/κ(I形) > 壁板/1.2
        assert!(
            wall_cols.column.as_y > wall_plain.column.as_y,
            "側柱算入で as_y が増えない: {} vs {}",
            wall_cols.column.as_y,
            wall_plain.column.as_y
        );
        let a_gross = 150.0 * 4000.0 + 2.0 * 360_000.0;
        // κ = as_gross/as_y(逆算)が矩形の 1.2 と異なる(I形の値)
        let kappa = a_gross / wall_cols.column.as_y;
        assert!(
            (kappa - 1.2).abs() > 1e-3,
            "κ が I 形になっていない: {kappa}"
        );
    }

    #[test]
    fn test_wall_panel_try_new_fallbacks() {
        let (model, mut data) = make_wall_model();
        // 2 節点しか無い場合は None（従来の暫定等価梁へ）
        data.nodes = smallvec::smallvec![NodeId(0), NodeId(2)];
        assert!(WallPanelElement::try_new(&data, &model).is_none());
    }

    /// トライアル追従の回帰テスト: update_state(du, commit=false) が internal_force に
    /// 反映され（内力 = K24·u と厳密に一致）、commit / revert / snapshot / restore が
    /// beam/behavior.rs と同じ規律で機能すること。従来は internal_force が恒常的に
    /// ゼロを返しており、非線形解析で耐震壁が復元力を負担していなかった。
    ///
    /// 本テストの K·u 比較は「internal_force と tangent_stiffness が将来ズレない」
    /// ことの回帰ガードであり、K24 の値そのものの正しさは独立の解析解と照合する
    /// `test_wall_panel_inplane_shear_matches_column`（12EI/((1+φ)h³)）が担保する
    /// （両者を合わせて非循環な検証となる）。
    #[test]
    fn test_wall_panel_trial_displacement_tracking() {
        use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalVec};
        let (model, data) = make_wall_model();
        let mut wall = WallPanelElement::try_new(&data, &model).unwrap();
        let ctx = Ctx { model: &model };
        let state = ElemState::default();

        // 上辺 2 節点へ面内水平変位（両端固定柱のせん断変形モード）
        let mut du = LocalVec {
            data: smallvec::smallvec![0.0; 24],
        };
        du.data[2 * 6] = 1.0;
        du.data[3 * 6] = 1.0;
        let snap = wall.snapshot_state();
        wall.update_state(&du, false, &ctx);

        // commit 前でも内力へ反映され、K24·u と厳密に一致する
        let f = wall.internal_force(&state, &ctx);
        let k = wall.stiffness_24();
        for i in 0..24 {
            let expected: f64 = (0..24).map(|j| k.get(i, j) * du.data[j]).sum();
            assert!(
                (f.data[i] - expected).abs() <= 1e-9 * expected.abs().max(1.0),
                "内力が K·u と不一致: i={i} f={} expected={expected}",
                f.data[i]
            );
        }
        // 上辺の水平力は非零（壁がせん断復元力を負担する）
        assert!(f.data[2 * 6].abs() > 1.0, "壁の復元力が生じていない");

        // commit → さらに反復 → revert で確定値へ戻る
        wall.commit_state();
        wall.update_state(&du, false, &ctx);
        wall.revert_state();
        let f2 = wall.internal_force(&state, &ctx);
        for i in 0..24 {
            assert!((f2.data[i] - f.data[i]).abs() < 1e-9);
        }

        // restore_state でスナップショット時点（初期状態）へ完全ロールバック
        wall.restore_state(&*snap);
        let f0 = wall.internal_force(&state, &ctx);
        assert!(f0.data.iter().all(|v| v.abs() < 1e-12));
    }
}

#[cfg(test)]
mod geometry_tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{ElementKind, EndCondition, ForceRegime, LocalAxis, Node};
    use squid_n_core::section_shape::SectionShape;

    /// 任意の 4 隅座標・任意の節点並び順で壁要素データを作る。
    fn wall_with(coords: [[f64; 3]; 4], order: [u32; 4]) -> (Model, ElementData) {
        let shape = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        let model = Model {
            nodes: coords
                .iter()
                .enumerate()
                .map(|(i, c)| Node {
                    id: NodeId(i as u32),
                    coord: *c,
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                })
                .collect(),
            sections: vec![shape.to_section(SectionId(0), "W150".into())],
            ..Default::default()
        };
        let data = ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: order.iter().map(|i| NodeId(*i)).collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        (model, data)
    }

    /// 台形壁（下辺 4000・上辺 2000）の壁長は上下辺の平均 3000 になる。
    /// 下辺だけ／上辺だけを採ると 4000／2000 となり代表長さにならない。
    #[test]
    fn test_wall_length_is_average_of_top_and_bottom_for_trapezoid() {
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [3000.0, 0.0, 3000.0],
            [1000.0, 0.0, 3000.0],
        ];
        let (model, data) = wall_with(coords, [0, 1, 2, 3]);
        let g = wall_panel_geometry(&data, &model).expect("Some");
        assert!((g.lw_bottom - 4000.0).abs() < 1e-6, "{}", g.lw_bottom);
        assert!((g.lw_top - 2000.0).abs() < 1e-6, "{}", g.lw_top);
        assert!(
            (g.lw - 3000.0).abs() < 1e-6,
            "台形壁の壁長は上下辺の平均 3000 であるべき。got {}",
            g.lw
        );
        assert!((g.h - 3000.0).abs() < 1e-6, "{}", g.h);
    }

    /// 節点の並び順に依存しない（z でソートして下辺・上辺を決める）。
    /// 並び順を変えても壁長・高さは不変であること。特に「先頭 2 節点が鉛直辺」に
    /// なる並びでも壁高さを壁長として拾わないこと。
    #[test]
    fn test_wall_geometry_is_independent_of_node_order() {
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ];
        // 先頭 2 節点が鉛直辺（節点0=下、節点3=上）になる並び。
        let (model, data) = wall_with(coords, [0, 3, 1, 2]);
        let g = wall_panel_geometry(&data, &model).expect("Some");
        assert!(
            (g.lw - 4000.0).abs() < 1e-6,
            "節点順に依らず壁長 4000（壁高さ 3000 ではない）。got {}",
            g.lw
        );
        assert!((g.h - 3000.0).abs() < 1e-6, "{}", g.h);
    }
}

#[cfg(test)]
mod shear_yield_tests {
    use super::*;
    use crate::behavior::{Ctx, ElemState, LocalVec};
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node};
    use squid_n_core::section_shape::SectionShape;

    fn wall_model() -> (Model, ElementData) {
        let shape = SectionShape::RcWall {
            thickness: 200.0,
            ps: 0.0025,
        };
        let mk = |id: u32, c: [f64; 3]| Node {
            id: NodeId(id),
            coord: c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        };
        let model = Model {
            nodes: vec![
                mk(0, [0.0, 0.0, 0.0]),
                mk(1, [4000.0, 0.0, 0.0]),
                mk(2, [4000.0, 0.0, 3000.0]),
                mk(3, [0.0, 0.0, 3000.0]),
            ],
            sections: vec![shape.to_section(SectionId(0), "W200".into())],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            ..Default::default()
        };
        let data = ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        (model, data)
    }

    /// 非線形経路（プッシュオーバー）では耐震壁の面内水平力が終局せん断強度 Qu で
    /// 頭打ちになる。従来は線形弾性のままで、押し込むほど際限なく水平力を負担し
    /// （100mm で 17.9 万 kN 等、実強度の数百倍）、崩壊機構が形成されないまま
    /// 保有水平耐力を過大評価していた。
    #[test]
    fn test_wall_shear_yields_at_ultimate_strength() {
        let (model, data) = wall_model();
        let qu = WallPanelElement::shear_capacity_of(&data, &model);
        assert!(qu > 0.0, "Qu が算定できるはず");

        let (mut b, _) = crate::factory::build_nonlinear_behavior(
            &data,
            &model,
            crate::factory::StrengthBasis::MaterialStrength,
        );
        let ctx = Ctx { model: &model };
        let st = ElemState::default();
        let mut max_q: f64 = 0.0;
        for _ in 0..300 {
            let mut du = LocalVec {
                data: smallvec::SmallVec::from_elem(0.0, 24),
            };
            du.data[12] = 1.0; // 上辺a Ux
            du.data[18] = 1.0; // 上辺b Ux
            b.update_state(&du, false, &ctx);
            b.commit_state();
            let f = b.internal_force(&st, &ctx);
            max_q = max_q.max((f.data[0] + f.data[6]).abs());
        }
        // 300mm 押しても Qu を（数値誤差程度を除き）超えない。
        assert!(
            max_q <= qu * 1.001,
            "壁の水平力 {:.3e} N が終局せん断強度 Qu={:.3e} N を超えている",
            max_q,
            qu
        );
        // 十分押しているので Qu に達していること（頭打ちが機能している）。
        assert!(
            max_q > qu * 0.99,
            "max_q={:.3e} が Qu={:.3e} に達していない",
            max_q,
            qu
        );
    }

    /// 弾性経路（許容応力度計算）では従来どおり降伏しない（線形）。
    #[test]
    fn test_wall_stays_elastic_in_linear_path() {
        let (model, data) = wall_model();
        let (mut b, _) = crate::factory::build_behavior(&data, &model);
        let ctx = Ctx { model: &model };
        let st = ElemState::default();
        let mut q_at = vec![];
        for step in 1..=200 {
            let mut du = LocalVec {
                data: smallvec::SmallVec::from_elem(0.0, 24),
            };
            du.data[12] = 1.0;
            du.data[18] = 1.0;
            b.update_state(&du, false, &ctx);
            b.commit_state();
            if step == 100 || step == 200 {
                let f = b.internal_force(&st, &ctx);
                q_at.push((f.data[0] + f.data[6]).abs());
            }
        }
        // 変位 2 倍で力も 2 倍（線形）。
        assert!(
            (q_at[1] - 2.0 * q_at[0]).abs() < q_at[1] * 1e-9,
            "弾性経路は線形であるべき: {:?}",
            q_at
        );
    }
}
