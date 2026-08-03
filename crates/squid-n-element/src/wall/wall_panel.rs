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
use squid_n_core::model::{ElementData, HysteresisModel, Model};
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
    /// 面内せん断の復元力ばね。骨格は従来と同じ弾完全塑性（初期剛性 k_s0・耐力 Qu）
    /// で、除荷・再載荷則を履歴則設定から解決する（既定は最大点指向型。
    /// [`Self::with_shear_hysteresis`]）。`None` は従来の移動硬化型
    /// （弾完全塑性リターンマッピング）のまま。
    shear_spring: Option<Box<dyn squid_n_material::UniaxialMaterial>>,
    /// せん断ばねの変形測度 D = γp + Q/k_s0 に用いる弾性モード剛性
    /// k_s0 = pᵀ·K_elastic·p [N/mm]（ばね骨格の初期剛性と共有）。
    shear_k0: f64,
    /// 壁柱の軸・曲げの弾塑性評価（ファイバー断面＋塑性増分ヒンジ）。
    /// `Some` のとき軸・曲げの応答（剛性・内力）はこのファイバー壁柱から得て、
    /// 面内せん断の Qu 頭打ちは従来どおり塑性すべりで扱う。
    /// 非線形解析（保有水平耐力）の既定で有効化される
    /// （[`Self::with_fiber_flexure`]。線形解析は従来どおり弾性壁柱）。
    fiber_column: Option<crate::fiber::FiberBeam>,
    /// ファイバー壁柱へ与え済みの壁柱端変位（グローバル系 12）。トライアル/確定。
    /// 四隅変位から求めた目標値との差分を増分としてファイバー要素へ渡すためのミラー。
    fiber_u12_trial: [f64; 12],
    fiber_u12_committed: [f64; 12],
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

/// 増分解析（保有水平耐力）で壁柱がファイバー化されるときの塑性化域長 Lp [mm]。
/// ファイバー化されない壁（耐震壁不成立・Qu を算定できない・Fc 未設定など）は `None`。
///
/// 判定条件・Lp の値ともに要素生成（[`crate::factory::build_nonlinear_behavior`] と
/// [`WallPanelElement::with_fiber_flexure`]）と同一のため、モデル化図の表示は解析の
/// モデル化と一致する。Lp は壁長の 0.5 倍を壁高さ h の 45% でクランプした値で、
/// 断面せい基準（0.5D）の柱・梁とは基準が異なる。
pub fn wall_column_fiber_lp(data: &ElementData, model: &Model) -> Option<f64> {
    // 耐震壁不成立（フレーム内雑壁）は剛性が実質 0 のため弾性のまま扱う。
    if !crate::misc_wall::wall_is_seismic(data, model) {
        return None;
    }
    // Qu を算定できない壁は、非線形経路が弾性要素へフォールバックする。
    if WallPanelElement::shear_capacity_of(data, model) <= 0.0 {
        return None;
    }
    let geom = wall_panel_geometry(data, model)?;
    // コンクリート強度が無ければファイバー断面を組めない。
    data.material
        .and_then(|mid| model.materials.get(mid.index()))?
        .fc
        .filter(|fc| *fc > 0.0)?;
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let t = match sec.and_then(|s| s.shape.as_ref()) {
        Some(SectionShape::RcWall { thickness, .. }) => *thickness,
        _ => sec.map(|s| s.thickness.unwrap_or(s.width))?,
    };
    if t <= 0.0 || geom.lw <= 0.0 || geom.h <= 0.0 {
        return None;
    }
    Some(crate::fiber::clamp_plastic_zone(0.5 * geom.lw, geom.h))
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
        let side_columns_released = crate::misc_wall::wall_is_seismic(data, model);
        for e in &model.elements {
            if !side_columns_released {
                break;
            }
            if !crate::side_column::is_side_column_member(e.kind) || e.nodes.len() < 2 {
                continue;
            }
            // 鉛直材のみ（ピン化条件と同じ、全クレート共通の 45° 余弦基準）。
            if let (Some(a), Some(b)) = (
                model.nodes.get(e.nodes[0].index()),
                model.nodes.get(e.nodes[1].index()),
            ) {
                if !squid_n_core::geom::is_vertical_axis(a.coord, b.coord) {
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
            // 壁柱は鉛直材のため、梁のねじれ解放（`beam::torsion`）は適用しない。
            torsion_release: [false, false],
            eval_sections: vec![0.0, 0.5, 1.0],
            section: data.section,
            material: data.material,
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
            local_stiffness_cache: std::sync::OnceLock::new(),
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
            shear_spring: None,
            shear_k0: 0.0,
            fiber_column: None,
            fiber_u12_trial: [0.0; 12],
            fiber_u12_committed: [0.0; 12],
        })
    }

    /// 壁柱の軸・曲げをファイバー断面（コンクリート格子＋縦筋の等価分散配置）の
    /// 弾塑性評価に切り替える（保有水平耐力・非線形解析の既定）。
    /// コンクリート強度 Fc が無い等でファイバー断面を組めない場合は弾性のまま返す。
    ///
    /// ファイバー壁柱は「全長弾性梁＋端部塑性増分ヒンジ」
    /// （[`crate::fiber::FiberBeam::from_raw_parts`]）で、弾性剛性は従来の弾性壁柱
    /// と同じ諸元（軸・面内曲げは鉄筋剛性係数込み、せん断は κ・開口低減込み）を
    /// 用いる。塑性化域長は 0.5·lw（可撓長の 45% までにクランプ）。
    /// 縦筋は壁筋比 ps を各層へ等価分散した鋼材ファイバー
    /// （既定 SD345、材料強度の基準 `basis` の主筋割増を適用）とする。
    pub(crate) fn with_fiber_flexure(
        mut self,
        data: &ElementData,
        model: &Model,
        basis: crate::factory::StrengthBasis,
        kind: squid_n_core::model::AnalysisKind,
    ) -> Self {
        let Some(geom) = wall_panel_geometry(data, model) else {
            return self;
        };
        let Some(mat) = data
            .material
            .and_then(|mid| model.materials.get(mid.index()))
        else {
            return self;
        };
        let Some(fc) = mat.fc.filter(|v| *v > 0.0) else {
            return self;
        };
        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let (t, ps) = match sec.and_then(|s| s.shape.as_ref()) {
            Some(SectionShape::RcWall { thickness, ps }) => (*thickness, *ps),
            _ => match sec.map(|s| s.thickness.unwrap_or(s.width)) {
                Some(t) if t > 0.0 => (t, 0.0025),
                _ => return self,
            },
        };
        let lw = 0.5 * (geom.lw_bottom + geom.lw_top);
        let h = self.column.length;
        if t <= 0.0 || lw <= 0.0 || h <= 0.0 {
            return self;
        }

        // ファイバー断面: コンクリート格子（幅 t × せい lw、面内曲げが κz 面）
        // ＋縦筋の等価分散配置（各せい方向層の中心へ ps·t·lw/nd ずつ）。
        let nw = 4;
        let nd = 20;
        let rebar_fy = 345.0 * basis.rebar_factor(Some(mat));
        // コンクリート除荷則は解析種別と部材個別指定から解決する。壁柱の増分既定は
        // 原点指向型（[`crate::factory::resolve_wall_concrete_hysteresis`] 参照）。
        let concrete_rule = crate::factory::resolve_wall_concrete_hysteresis(data, model, kind);
        let make_section = || {
            let (mut section, mut mats) = crate::fiber::build_gauss_fibers(
                t,
                lw,
                nw,
                nd,
                None,
                Some(fc),
                mat.young,
                None,
                1.0,
                1.0,
                concrete_rule,
            );
            if ps > 0.0 {
                let a_each = ps * t * lw / nd as f64;
                for i in 0..nd {
                    // build_gauss_fibers の回転後座標系: y=せい（lw）方向、z=幅（t）方向。
                    let y = ((i as f64 + 0.5) / nd as f64 - 0.5) * lw;
                    section.fibers.push(squid_n_section::fiber::Fiber {
                        y,
                        z: 0.0,
                        area: a_each,
                        material: 1,
                    });
                    mats.push(crate::fiber::steel_fiber_material(E_STEEL, Some(rebar_fy)));
                }
            }
            (section, mats)
        };

        // 弾性剛性の諸元は従来の弾性壁柱（`try_new_scaled` の column）と同一。
        let col = &self.column;
        let fiber = crate::fiber::FiberBeam::from_raw_parts(
            col.nodes,
            col.length,
            col.axis,
            col.density,
            col.e,
            col.g,
            col.a,
            col.iy,
            col.iz,
            col.as_y,
            col.as_z,
            col.j,
            0.5 * lw,
            [make_section(), make_section()],
        );
        self.fiber_column = Some(fiber);
        self
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
        has_side_column: bool,
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
        // 等価引張鉄筋比 pte = 100·at/(te·d) の at。
        // - 付帯柱（側柱）がある壁: 引張側最端の柱 1 本の主筋量を用いる。
        //   側柱があるのに主筋を読み取れない場合は**断面設定の不備**であり、
        //   代替値で埋めずに 0 を返す（呼び出し側が
        //   [`wall_shear_capacity_issue`] で検出しエラーとする）。
        // - 付帯柱が無い壁: 壁の縦筋が一様配筋であるとみなし at = ps·te·d
        //   （＝ pte = 100·ps \[%\]）とする。壁のみで構成される耐震壁の
        //   正規の扱いであり、データ不備の代替ではない。
        let at = if has_side_column {
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
    ///
    /// モデル化図（`squid-n-app`）が、面内せん断を Qu で頭打ちにする壁かどうかの
    /// 判定と表示値に用いるため公開する。
    pub fn shear_capacity_of(data: &ElementData, model: &Model) -> f64 {
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
        // 鋼板耐震壁はせん断降伏で決まる（[`Self::steel_shear_capacity_of`]）。
        // 荒川式は RC 耐震壁の終局せん断強度のため適用しない。
        if !crate::misc_wall::is_rc_wall(data, model) {
            return Self::steel_shear_capacity_of(data, model);
        }
        let fc = data
            .material
            .and_then(|mid| model.materials.get(mid.index()))
            .and_then(|m| m.fc);
        // 側柱（壁の鉛直辺に取り付く柱）の沿壁方向せい・主筋量。
        let edge_pairs = [[geom.bottom[0], geom.top[0]], [geom.bottom[1], geom.top[1]]];
        let mut col_depth_sum = 0.0;
        let mut col_main_at: f64 = 0.0;
        let mut has_side_column = false;
        for e in &model.elements {
            if !crate::side_column::is_side_column_member(e.kind) || e.nodes.len() < 2 {
                continue;
            }
            let (n0, n1) = (e.nodes[0], e.nodes[1]);
            if !edge_pairs
                .iter()
                .any(|p| (p[0] == n0 && p[1] == n1) || (p[0] == n1 && p[1] == n0))
            {
                continue;
            }
            has_side_column = true;
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
            has_side_column,
            opening,
        )
    }

    /// 鋼板耐震壁の面内せん断終局強度 Qy [N]。
    ///
    /// 鋼板のせん断降伏で決まるものとし、von Mises の降伏条件による純せん断の
    /// 降伏せん断応力度 τy = F/√3 を全断面 t·lw に乗じる。
    ///
    /// ```text
    /// Qy = t · lw · F / √3
    /// ```
    ///
    /// F は材料の降伏強度 `Material.fy` を用いる。
    ///
    /// **せん断座屈は考慮していない。** 幅厚比の大きい無補剛の鋼板は、せん断降伏に
    /// 達する前に面外へせん断座屈して耐力が頭打ちになるため、本式は座屈が生じない
    /// （十分に補剛された）鋼板を前提とする**危険側**の評価である。増分解析の実行時に
    /// その旨を情報表示する（`squid-n-app` の解析実行）。座屈耐力の評価は原典の
    /// 入手後に対応する。
    pub fn steel_shear_capacity_of(data: &ElementData, model: &Model) -> f64 {
        let Some(geom) = wall_panel_geometry(data, model) else {
            return 0.0;
        };
        let Some(sec) = data.section.and_then(|sid| model.sections.get(sid.index())) else {
            return 0.0;
        };
        // 壁形状 `RcWall` は RC 壁専用のため、鋼板壁の板厚は断面の板厚を用いる。
        let t = sec.thickness.unwrap_or(sec.width);
        let f = data
            .material
            .and_then(|mid| model.materials.get(mid.index()))
            .and_then(|m| m.fy)
            .unwrap_or(0.0);
        if t <= 0.0 || geom.lw <= 0.0 || f <= 0.0 {
            return 0.0;
        }
        t * geom.lw * f / 3.0_f64.sqrt()
    }

    /// 耐震壁のせん断終局強度 Qu を算定できない**設定不備**があれば、その内容を返す。
    ///
    /// 保有水平耐力計算では耐震壁を Qu で頭打ちにするため、Qu が算定できない壁は
    /// 際限なく水平力を負担して保有水平耐力を過大評価する（危険側）。したがって
    /// 代替値で埋めずに解析を止め、利用者へ是正を促す。
    ///
    /// 検出する不備:
    /// - 壁エレメントとして構築できない（4 節点未満／節点座標が退化）。この壁は
    ///   暫定等価梁（弾性梁）へフォールバックするため Qu の頭打ちが効かない。
    /// - 断面が設定されていない／壁厚が 0 以下。
    /// - 材料が設定されていない。
    /// - 材料にコンクリート強度 Fc が設定されていない、または Fc が 0 以下。
    /// - 付帯柱（側柱）はあるのに、その断面から主筋量を読み取れない
    ///   （断面形状が RcRect でない／主筋本数・径が 0）。等価引張鉄筋比 pte を
    ///   算定できない。
    /// - 上記のいずれにも当てはまらないが Qu が 0 以下になる（適用範囲外の寸法など）。
    ///
    /// 付帯柱が無い壁（壁のみの耐震壁）は不備ではなく、壁の縦筋比 ps から pte を
    /// 算定する。`ps = 0` の場合は主筋・壁筋がいずれも無いことになるため不備とする。
    ///
    /// 耐震壁として成立する壁について、[`Self::shear_capacity_of`] が 0 を返す
    /// （＝弾性のまま扱われる）ケースを**必ず**いずれかの不備として拾う。個別診断を
    /// 追加し忘れても無音で弾性へ落ちないよう、最後に Qu>0 を確認する総括判定を置く。
    pub fn wall_shear_capacity_issue(data: &ElementData, model: &Model) -> Option<String> {
        if !matches!(data.kind, squid_n_core::model::ElementKind::Wall) {
            return None;
        }
        // 4 節点を与えているのに壁エレメントの幾何を組めない壁（節点の指定ミス・
        // 退化した座標）は、耐震壁の四周条件を判定できず雑壁へ落ちる。雑壁としての
        // 剛性算入も 4 節点の幾何を要するため、剛性も耐力も持たないまま無音で消える。
        // 入力不備として報告する。
        let geom = match wall_panel_geometry(data, model) {
            Some(g) => g,
            None if data.nodes.len() >= 4 => {
                return Some(format!(
                    "耐震壁 ID {} を壁エレメントとして構築できません（4 節点の指定と節点座標を確認してください）。\
                     壁エレメントを構築できない壁は弾性の等価梁として扱われ、\
                     保有水平耐力計算で面内せん断が終局せん断強度で頭打ちになりません。",
                    data.id.0
                ));
            }
            None => return None,
        };
        // 耐震壁として成立しない壁（フレーム内雑壁）は Qu を要さない。
        if !crate::misc_wall::wall_is_seismic(data, model) {
            return None;
        }
        let Some(sec) = data.section.and_then(|sid| model.sections.get(sid.index())) else {
            return Some(format!(
                "耐震壁 ID {} に断面が設定されていません。\
                 断面タブで壁厚・壁筋比を設定してください。\
                 保有水平耐力計算では耐震壁の終局せん断強度が必要です。",
                data.id.0
            ));
        };
        let (t, ps) = match &sec.shape {
            Some(SectionShape::RcWall { thickness, ps }) => (*thickness, (*ps).max(0.0)),
            _ => (sec.thickness.unwrap_or(sec.width), 0.0),
        };
        if t <= 0.0 {
            return Some(format!(
                "耐震壁 ID {} の断面「{}」の壁厚が 0 以下です。\
                 断面タブで壁厚を設定してください。\
                 保有水平耐力計算では耐震壁の終局せん断強度が必要です。",
                data.id.0, sec.name
            ));
        }
        let Some(mat) = data
            .material
            .and_then(|mid| model.materials.get(mid.index()))
        else {
            return Some(format!(
                "耐震壁 ID {} に材料が設定されていません。\
                 材料タブで材料を割り当ててください。\
                 保有水平耐力計算では耐震壁の終局せん断強度が必要です。",
                data.id.0
            ));
        };
        // 鋼板耐震壁はせん断降伏 Qy=t·lw·F/√3 で決まるため、要するのは Fc ではなく
        // 降伏強度 fy である（[`Self::steel_shear_capacity_of`]）。
        if !crate::misc_wall::is_rc_wall(data, model) {
            if !mat.fy.is_some_and(|fy| fy > 0.0) {
                return Some(format!(
                    "鋼板耐震壁 ID {} の材料「{}」に降伏強度 fy が設定されていません。\
                     材料タブで fy を設定してください。\
                     保有水平耐力計算では鋼板のせん断降伏 Qy=t·lw·F/√3 で面内せん断を頭打ちにします。",
                    data.id.0, mat.name
                ));
            }
            if Self::steel_shear_capacity_of(data, model) <= 0.0 {
                return Some(format!(
                    "鋼板耐震壁 ID {} の終局せん断強度 Qy を算定できません（算定結果が 0 以下）。\
                     壁の板厚・壁長さの入力を確認してください。\
                     保有水平耐力計算では Qy が定まらない壁を弾性として扱えません。",
                    data.id.0
                ));
            }
            return None;
        }
        match mat.fc {
            None => {
                return Some(format!(
                    "耐震壁 ID {} の材料「{}」にコンクリート強度 Fc が設定されていません。\
                     保有水平耐力計算では耐震壁の終局せん断強度が必要です。材料タブで Fc を設定してください。",
                    data.id.0, mat.name
                ));
            }
            Some(fc) if fc <= 0.0 => {
                return Some(format!(
                    "耐震壁 ID {} の材料「{}」のコンクリート強度 Fc が {} で 0 以下です。\
                     保有水平耐力計算では耐震壁の終局せん断強度が必要です。材料タブで Fc を設定してください。",
                    data.id.0, mat.name, fc
                ));
            }
            Some(_) => {}
        }

        let edge_pairs = [[geom.bottom[0], geom.top[0]], [geom.bottom[1], geom.top[1]]];
        let mut has_side_column = false;
        let mut col_main_at: f64 = 0.0;
        for e in &model.elements {
            if !crate::side_column::is_side_column_member(e.kind) || e.nodes.len() < 2 {
                continue;
            }
            let (n0, n1) = (e.nodes[0], e.nodes[1]);
            if !edge_pairs
                .iter()
                .any(|p| (p[0] == n0 && p[1] == n1) || (p[0] == n1 && p[1] == n0))
            {
                continue;
            }
            has_side_column = true;
            if let Some(cs) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                if let Some(SectionShape::RcRect { rebar, .. }) = cs.shape.as_ref() {
                    col_main_at =
                        col_main_at.max(squid_n_core::section_shape::bar_set_area(&rebar.main_x));
                }
            }
        }
        if has_side_column && col_main_at <= 0.0 {
            return Some(format!(
                "耐震壁 ID {} の側柱（付帯柱）から主筋量を取得できません。\
                 断面の形状を RC 矩形（RcRect）とし、主筋の本数・径を設定してください。\
                 保有水平耐力計算では側柱主筋から耐震壁の等価引張鉄筋比 pte を算定します。",
                data.id.0
            ));
        }
        if !has_side_column && ps <= 0.0 {
            return Some(format!(
                "耐震壁 ID {} は側柱（付帯柱）が無く、かつ壁筋比 ps が 0 です。\
                 断面タブで壁筋比を設定してください。\
                 保有水平耐力計算では壁筋比から耐震壁の等価引張鉄筋比 pte を算定します。",
                data.id.0
            ));
        }
        // 総括判定: 個別診断に当てはまらない理由（適用範囲外の寸法・開口など）で
        // Qu が 0 になる場合も、無音で弾性へ落とさずここで捕捉する。
        if Self::shear_capacity_of(data, model) <= 0.0 {
            return Some(format!(
                "耐震壁 ID {} の終局せん断強度 Qu を算定できません（算定結果が 0 以下）。\
                 壁の寸法・壁筋比・側柱の配筋・開口寸法の入力を確認してください。\
                 保有水平耐力計算では Qu が定まらない壁を弾性として扱えません。",
                data.id.0
            ));
        }
        None
    }

    /// 面内せん断の終局強度 Qu [N] を与えて弾完全塑性化する（保有水平耐力用）。
    /// `qu <= 0` は弾性のまま（降伏しない）。
    pub(crate) fn with_shear_capacity(mut self, qu: f64) -> Self {
        self.qu_shear = qu.max(0.0);
        self
    }

    /// 面内せん断の復元力ばねを構築する（[`Self::with_shear_capacity`] の後に呼ぶ）。
    ///
    /// 骨格は従来と同じ弾完全塑性（初期剛性 k_s0 = pᵀ·K_elastic·p、耐力 Qu で
    /// 頭打ち）とし、除荷・再載荷則のみ `rule` に従う（既定は最大点指向型）。
    /// トリリニア骨格のひび割れ点は弾性線上（Qu/3）に置きバイリニア相当、
    /// 終局点は降伏変形の 10⁴ 倍（降伏後フラット＝Qu 頭打ちを保持）とする。
    /// `qu_shear <= 0`（弾性）や k_s0 が取れない場合は何もしない
    /// （従来の弾完全塑性リターンマッピングのまま）。
    pub(crate) fn with_shear_hysteresis(mut self, rule: HysteresisModel) -> Self {
        use squid_n_material::{HysteresisMaterial, HysteresisRule};
        if self.qu_shear <= 0.0 {
            return self;
        }
        // 弾性壁柱の全体系剛性で k_s0 = pᵀ·(Aᵀ·K12·A)·p を評価する。
        let k12 = self.column.axis.to_global(&self.column.local_stiffness());
        // v = A·p（12 自由度）
        let mut v = [0.0_f64; 12];
        for (i, vi) in v.iter_mut().enumerate() {
            let mut acc = 0.0;
            for q in 0..24 {
                acc += self.a_mat[i * 24 + q] * self.shear_mode[q];
            }
            *vi = acc;
        }
        // w = K12·v
        let mut w = [0.0_f64; 12];
        for (i, wi) in w.iter_mut().enumerate() {
            let mut acc = 0.0;
            for (j, vj) in v.iter().enumerate() {
                acc += k12.get(i, j) * vj;
            }
            *wi = acc;
        }
        // k_s0 = pᵀ·Aᵀ·w = vᵀ·w
        let k_s0: f64 = v.iter().zip(w.iter()).map(|(a, b)| a * b).sum();
        if k_s0 <= 0.0 {
            return self;
        }
        let qu = self.qu_shear;
        let dy = qu / k_s0;
        let crack = (qu / 3.0, dy / 3.0);
        let yield_point = (qu, dy);
        let ultimate = (qu, dy * 1.0e4);
        let r = match rule {
            HysteresisModel::Retrograde => HysteresisRule::Retrograde {
                crack,
                yield_point,
                ultimate,
            },
            HysteresisModel::Standard => HysteresisRule::Standard {
                crack,
                yield_point,
                ultimate,
            },
            HysteresisModel::OriginOriented => HysteresisRule::OriginOriented {
                yield_point,
                ultimate,
            },
            HysteresisModel::Takeda => HysteresisRule::Takeda {
                crack,
                yield_point,
                ultimate,
                alpha: 0.4,
            },
            // 既定（Auto・Karsan–Jirsa 型等の Q–δ 系でない指定を含む）: 最大点指向型。
            _ => HysteresisRule::MaxPointOriented {
                crack,
                yield_point,
                ultimate,
            },
        };
        self.shear_spring = Some(Box::new(HysteresisMaterial::new(r)));
        self.shear_k0 = k_s0;
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
        if let Some(sp) = &self.shear_spring {
            if self.shear_k0 > 0.0 {
                // 直列ばねの整合: 変形測度 D = γp_c + Q(γp_c)/k_s0 でばね履歴を評価し、
                // 伝達水平力がばね応答 M(D) と一致するよう γp を補正する（Q は γp に
                // 線形なため 1 回で厳密。補正後も D は不変で自己整合）。
                let d = self.committed_slip + q_trial / self.shear_k0;
                let (q_target, _) = sp.probe(d);
                let yielded = (q_trial - q_target).abs() > self.qu_shear * 1e-9;
                return (self.committed_slip + (q_trial - q_target) / k_s, yielded);
            }
        }
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

    /// 壁柱の全体系 12×12 接線剛性（ファイバー壁柱があればその整合接線、
    /// なければ弾性壁柱）。
    fn k12_global(&self, ctx: &Ctx) -> LocalMat {
        match &self.fiber_column {
            Some(f) => f.tangent_stiffness(&ElemState::default(), ctx),
            None => self.column.axis.to_global(&self.column.local_stiffness()),
        }
    }

    /// 壁柱の現在トライアル状態の全体系内力（24 自由度）。ファイバー壁柱専用
    /// （履歴に整合した復元力 f24 = Aᵀ·f12）。
    fn f24_fiber(&self, ctx: &Ctx) -> Option<[f64; 24]> {
        let f = self.fiber_column.as_ref()?;
        let f12 = f.internal_force(&ElemState::default(), ctx);
        let mut f24 = [0.0_f64; 24];
        for (p, fp) in f24.iter_mut().enumerate() {
            let mut s = 0.0;
            for i in 0..12 {
                s += self.a_mat[i * 24 + p] * f12.data[i];
            }
            *fp = s;
        }
        Some(f24)
    }

    /// ファイバー壁柱が現在伝達している面内水平力 Q = pᵀ·f24。
    fn inplane_shear_fiber(&self, ctx: &Ctx) -> Option<f64> {
        let f24 = self.f24_fiber(ctx)?;
        Some(
            self.shear_mode
                .iter()
                .zip(f24.iter())
                .map(|(p, v)| p * v)
                .sum(),
        )
    }

    /// 現在のトライアル状態で壁が伝達している面内水平力 Q = pᵀ·f。
    /// ファイバー壁柱はその内力から、弾性壁柱は f = K·(u − γp·p) から評価する。
    fn inplane_shear_trial(&self, ctx: &Ctx) -> f64 {
        if let Some(q) = self.inplane_shear_fiber(ctx) {
            return q;
        }
        let k = self.stiffness_24(ctx);
        let mut u_eff = self.trial_disp;
        for (ue, p) in u_eff.iter_mut().zip(self.shear_mode.iter()) {
            *ue -= self.trial_slip * p;
        }
        let f = Self::mat_vec(&k, &u_eff);
        self.shear_mode
            .iter()
            .zip(f.iter())
            .map(|(p, v)| p * v)
            .sum()
    }

    /// 全体系 24×24 剛性 K = Aᵀ·K_col·A。
    fn stiffness_24(&self, ctx: &Ctx) -> LocalMat {
        let k12 = self.k12_global(ctx);
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

    fn tangent_stiffness(&self, _state: &ElemState, ctx: &Ctx) -> LocalMat {
        let k = self.stiffness_24(ctx);
        if self.qu_shear <= 0.0 {
            return k;
        }
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
        // せん断方向の剛性低減率 factor:
        // - ばねあり: **剛性を保持**し（factor=0）、せん断非線形は内力側のすべり
        //   補正（`Q = M(D)` の整合）だけで表現する（初期剛性法）。プラトー
        //   （ばね接線 0）で剛性を除去する整合接線にすると、曲げ機構の形成後に
        //   せん断が Qu から除荷へ向かう「角点」で大域 Newton が特異化して発散する。
        //   従来の弾完全塑性リターンマッピングも、降伏判定の許容差により実質的に
        //   全剛性を保持して同じ角点を通過していた（挙動踏襲）。
        // - ばね無し（従来）: 降伏中のみ全除去（弾完全塑性のコンシステント接線）。
        let factor = if self.shear_spring.is_some() && self.shear_k0 > 0.0 {
            0.0
        } else {
            let yielded = match self.inplane_shear_fiber(ctx) {
                // ファイバー壁柱: すべり γp は update_state で確定済み。伝達中の面内
                // 水平力が Qu 近傍なら降伏中（せん断方向の剛性を除去する）。
                Some(q) => q.abs() >= self.qu_shear * (1.0 - 1e-9),
                None => self.shear_return_map(&k, &self.trial_disp).1,
            };
            if yielded {
                1.0
            } else {
                0.0
            }
        };
        if factor <= 0.0 {
            return k;
        }
        let mut kt = LocalMat::zeros(24);
        for i in 0..24 {
            for j in 0..24 {
                let v = k.get(i, j) - factor * kp[i] * kp[j] / k_s;
                if v != 0.0 {
                    kt.set(i, j, v);
                }
            }
        }
        kt
    }

    fn internal_force(&self, _state: &ElemState, ctx: &Ctx) -> LocalVec {
        // ファイバー壁柱: 復元力は履歴に整合したファイバー内力 f24 = Aᵀ·f12
        // （すべり γp は update_state で反映済み）。
        if let Some(f24) = self.f24_fiber(ctx) {
            return LocalVec {
                data: smallvec::SmallVec::from_slice(&f24),
            };
        }
        // 弾性壁柱: f = K24 · (u − γp·p)（トライアル追従。beam/behavior.rs と同じ規約）。
        // γp は面内せん断の塑性すべりで、終局せん断強度 Qu を超える水平力を
        // 負担しないよう [`Self::shear_return_map`] が求める。Qu 未設定
        // （弾性解析経路）では γp=0 で従来どおりの線形弾性。
        let k = self.stiffness_24(ctx);
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

    fn update_state(&mut self, du: &LocalVec, commit: bool, ctx: &Ctx) {
        for i in 0..24.min(du.data.len()) {
            self.trial_disp[i] += du.data[i];
        }
        if self.fiber_column.is_some() {
            // ファイバー壁柱: すべり γp とファイバー状態を固定点反復で整合させる。
            // 確定すべりから出発し、有効変位 u−γp·p を壁柱端変位へ写して
            // ファイバー要素を更新 → 伝達水平力 Q が Qu を超えていれば
            // Δγp = (|Q|−Qu)/k_s（k_s = pᵀ·K_t·p）だけすべりを進める。
            // ファイバー応答は局所的に線形なため数回で収束する。
            let mut slip = self.committed_slip;
            for _ in 0..8 {
                let mut u_eff = self.trial_disp;
                for (ue, p) in u_eff.iter_mut().zip(self.shear_mode.iter()) {
                    *ue -= slip * p;
                }
                let u12 = self.to_column_disp(&u_eff);
                let du12: [f64; 12] = std::array::from_fn(|i| u12[i] - self.fiber_u12_trial[i]);
                let dv = LocalVec {
                    data: smallvec::SmallVec::from_slice(&du12),
                };
                let Some(fiber) = self.fiber_column.as_mut() else {
                    break;
                };
                fiber.update_state(&dv, false, ctx);
                self.fiber_u12_trial = u12;
                if self.qu_shear <= 0.0 {
                    break;
                }
                let Some(q) = self.inplane_shear_fiber(ctx) else {
                    break;
                };
                // ばねあり: 残差 = 伝達水平力 − ばね応答 M(D)（D = γp + Q/k_s0 の
                // 固定点 Q = M(D) を目指す）。ばね無し: 従来の Qu 超過分。
                let residual = if self.shear_spring.is_some() && self.shear_k0 > 0.0 {
                    let d = slip + q / self.shear_k0;
                    let q_target = self
                        .shear_spring
                        .as_ref()
                        .map(|sp| sp.probe(d).0)
                        .unwrap_or(q);
                    q - q_target
                } else if q.abs() > self.qu_shear {
                    (q.abs() - self.qu_shear) * q.signum()
                } else {
                    0.0
                };
                if residual.abs() <= self.qu_shear * 1e-9 {
                    break;
                }
                let k = self.stiffness_24(ctx);
                let kp = Self::mat_vec(&k, &self.shear_mode);
                let k_s: f64 = self
                    .shear_mode
                    .iter()
                    .zip(kp.iter())
                    .map(|(p, v)| p * v)
                    .sum();
                if k_s <= 0.0 {
                    break;
                }
                slip += residual / k_s;
            }
            self.trial_slip = slip;
        } else {
            // 弾性壁柱: 塑性すべりはトライアル変位から都度求め直す
            // （経路依存の単調載荷を前提。commit 時に確定値へ移す）。
            let k = self.stiffness_24(ctx);
            let (slip, _) = self.shear_return_map(&k, &self.trial_disp);
            self.trial_slip = slip;
        }
        // せん断ばねのトライアル状態を最終変形測度で更新（commit_state で確定）。
        if self.shear_spring.is_some() && self.shear_k0 > 0.0 {
            let q = self.inplane_shear_trial(ctx);
            let d = self.trial_slip + q / self.shear_k0;
            if let Some(sp) = &mut self.shear_spring {
                sp.trial(d);
            }
        }
        if commit {
            self.commit_state();
        }
    }

    fn commit_state(&mut self) {
        self.committed_disp = self.trial_disp;
        self.committed_slip = self.trial_slip;
        if let Some(f) = &mut self.fiber_column {
            f.commit_state();
        }
        if let Some(sp) = &mut self.shear_spring {
            sp.commit();
        }
        self.fiber_u12_committed = self.fiber_u12_trial;
    }

    fn revert_state(&mut self) {
        self.trial_disp = self.committed_disp;
        self.trial_slip = self.committed_slip;
        if let Some(f) = &mut self.fiber_column {
            f.revert_state();
        }
        if let Some(sp) = &mut self.shear_spring {
            sp.revert();
        }
        self.fiber_u12_trial = self.fiber_u12_committed;
    }

    fn snapshot_state(&self) -> Box<dyn std::any::Any> {
        Box::new((
            self.committed_disp,
            self.trial_disp,
            self.committed_slip,
            self.trial_slip,
            self.fiber_column.as_ref().map(|f| f.snapshot_state()),
            self.fiber_u12_trial,
            self.fiber_u12_committed,
            self.shear_spring.as_ref().map(|sp| sp.serialize_state()),
        ))
    }

    fn restore_state(&mut self, state: &dyn std::any::Any) {
        type Snapshot = (
            [f64; 24],
            [f64; 24],
            f64,
            f64,
            Option<Box<dyn std::any::Any>>,
            [f64; 12],
            [f64; 12],
            Option<Vec<u8>>,
        );
        if let Some((committed, trial, cslip, tslip, fsnap, u12t, u12c, spring)) =
            state.downcast_ref::<Snapshot>()
        {
            self.committed_disp = *committed;
            self.trial_disp = *trial;
            self.committed_slip = *cslip;
            self.trial_slip = *tslip;
            if let (Some(f), Some(snap)) = (&mut self.fiber_column, fsnap.as_ref()) {
                f.restore_state(snap.as_ref());
            }
            self.fiber_u12_trial = *u12t;
            self.fiber_u12_committed = *u12c;
            if let (Some(sp), Some(bytes)) = (&mut self.shear_spring, spring.as_ref()) {
                // snapshot は同一実行内の巻き戻し用のため、復元失敗はプログラム
                // エラー（形式は常に一致する）。
                sp.deserialize_state(bytes)
                    .expect("壁せん断ばねのスナップショット復元");
            }
        } else if let Some((committed, trial, cslip, tslip)) =
            state.downcast_ref::<([f64; 24], [f64; 24], f64, f64)>()
        {
            self.committed_disp = *committed;
            self.trial_disp = *trial;
            self.committed_slip = *cslip;
            self.trial_slip = *tslip;
        }
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        let cp = WallPanelCheckpoint {
            committed_disp: self.committed_disp,
            trial_disp: self.trial_disp,
            committed_slip: self.committed_slip,
            trial_slip: self.trial_slip,
            fiber: self.fiber_column.as_ref().map(|f| f.serialize_checkpoint()),
            fiber_u12_trial: self.fiber_u12_trial,
            fiber_u12_committed: self.fiber_u12_committed,
            shear_spring: self.shear_spring.as_ref().map(|sp| sp.serialize_state()),
        };
        bincode::serialize(&cp).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        // 旧チェックポイント（変位未収録・空バイト列）は「状態なし」として許容する。
        if data.is_empty() {
            return Ok(());
        }
        if let Ok(cp) = bincode::deserialize::<WallPanelCheckpoint>(data) {
            self.committed_disp = cp.committed_disp;
            self.trial_disp = cp.trial_disp;
            self.committed_slip = cp.committed_slip;
            self.trial_slip = cp.trial_slip;
            if let (Some(f), Some(bytes)) = (&mut self.fiber_column, cp.fiber.as_ref()) {
                f.deserialize_checkpoint(bytes)?;
            }
            self.fiber_u12_trial = cp.fiber_u12_trial;
            self.fiber_u12_committed = cp.fiber_u12_committed;
            if let (Some(sp), Some(bytes)) = (&mut self.shear_spring, cp.shear_spring.as_ref()) {
                sp.deserialize_state(bytes)
                    .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
            }
            return Ok(());
        }
        // 旧形式（変位のみ）。
        let (committed, trial): ([f64; 24], [f64; 24]) = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.committed_disp = committed;
        self.trial_disp = trial;
        Ok(())
    }

    /// 塑性率評価はファイバー壁柱の危険断面プローブへ委譲する（弾性壁柱は None）。
    fn ductility_probe(&self) -> Option<crate::behavior::DuctilityProbe> {
        self.fiber_column.as_ref().and_then(|f| f.ductility_probe())
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

/// [`WallPanelElement`] のチェックポイント形式（現行）。
/// 旧形式（`(committed_disp, trial_disp)` のみ）は読み込み時にフォールバックする。
#[derive(serde::Serialize, serde::Deserialize)]
struct WallPanelCheckpoint {
    committed_disp: [f64; 24],
    trial_disp: [f64; 24],
    committed_slip: f64,
    trial_slip: f64,
    /// ファイバー壁柱のチェックポイント（弾性壁柱は None）。
    fiber: Option<Vec<u8>>,
    fiber_u12_trial: [f64; 12],
    fiber_u12_committed: [f64; 12],
    /// 面内せん断ばねの材料状態（ばね未構築は None。旧形式も None 扱い）。
    #[serde(default)]
    shear_spring: Option<Vec<u8>>,
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
    use squid_n_core::model::MaterialCategory;
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
            support_spring: None,
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
                category: MaterialCategory::Concrete,
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
        // 耐震壁は四周を柱・梁に囲まれた壁を対象とするため、四周に線材を置く。
        let mut model = model;
        crate::wall::add_surrounding_frame(&mut model, &data);
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
        let ctx = Ctx { model: &model };
        let k = wall.stiffness_24(&ctx);
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
        let ctx = Ctx { model: &model };
        let k = wall.stiffness_24(&ctx);
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
        let ctx = Ctx { model: &model };
        let k = wall.stiffness_24(&ctx);
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
        let ctx = Ctx { model: &model };
        let k = wall.stiffness_24(&ctx);
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
        let k_no = {
            let ctx = Ctx { model: &model };
            wall.stiffness_24(&ctx)
        };
        model.wall_attrs.push(squid_n_core::model::WallAttr {
            elem: ElemId(0),
            opening_area: 3.0e6, // 25%
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        });
        let wall_o = WallPanelElement::try_new(&data, &model).unwrap();
        let ctx = Ctx { model: &model };
        let k_o = wall_o.stiffness_24(&ctx);
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
                main_grade: None,
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
        // 左右の鉛直辺（節点 0-3・1-2）へ 600×600 RC 側柱を追加する。
        // `add_surrounding_frame` は上下辺の大梁だけを置くため、側柱はここで足す。
        let base = model.elements.iter().map(|e| e.id.0).max().unwrap_or(0) + 1;
        for (i, (a, b)) in [(NodeId(0), NodeId(3)), (NodeId(1), NodeId(2))]
            .into_iter()
            .enumerate()
        {
            model.elements.push(ElementData {
                id: ElemId(base + i as u32),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![a, b],
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
        let k = wall.stiffness_24(&ctx);
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
                    support_spring: None,
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
    use squid_n_core::model::MaterialCategory;
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
            support_spring: None,
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
                category: MaterialCategory::Concrete,
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
        // 耐震壁は四周を柱・梁に囲まれた壁を対象とするため、四周に線材を置く。
        let mut model = model;
        crate::wall::add_surrounding_frame(&mut model, &data);
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
            crate::factory::AnalysisKind::Incremental,
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

    /// 面内せん断ばねの既定履歴（最大点指向型）: 除荷・再載荷が最大経験点を指向
    /// する割線となり、除荷しても変形が完全には戻らず（残留変形）、再載荷の
    /// 中間点では Qu より明確に小さい力（ピンチング）、最大経験変位まで戻すと
    /// Qu へ復帰する。
    #[test]
    fn test_wall_shear_hysteresis_is_max_point_oriented() {
        let (model, data) = wall_model();
        let qu = WallPanelElement::shear_capacity_of(&data, &model);
        assert!(qu > 0.0);
        let (mut b, _) = crate::factory::build_nonlinear_behavior(
            &data,
            &model,
            crate::factory::StrengthBasis::MaterialStrength,
            crate::factory::AnalysisKind::TimeHistory,
        );
        let ctx = Ctx { model: &model };
        let st = ElemState::default();
        let push = |b: &mut Box<dyn ElementBehavior>, d: f64| -> f64 {
            let mut du = LocalVec {
                data: smallvec::SmallVec::from_elem(0.0, 24),
            };
            du.data[12] = d; // 上辺a Ux
            du.data[18] = d; // 上辺b Ux
            b.update_state(&du, false, &ctx);
            b.commit_state();
            let f = b.internal_force(&st, &ctx);
            f.data[0] + f.data[6]
        };

        // (1) +30mm 押して降伏（|Q| ≈ Qu）。
        let mut q_peak = 0.0;
        for _ in 0..30 {
            q_peak = push(&mut b, 1.0);
        }
        assert!(
            (q_peak.abs() - qu).abs() <= qu * 0.02,
            "ピークで Qu: {q_peak:.3e} vs {qu:.3e}"
        );
        let sgn = q_peak.signum();

        // (2) 除荷: 最大点指向の除荷は反対側の経験点を指向する割線のため、
        // Q が 0 付近へ落ちるまでに要する戻し量は 30mm より明確に小さい
        // （＝残留変形が残る）。
        let mut q = q_peak;
        let mut n_unload = 0;
        while q * sgn > qu * 0.02 && n_unload < 29 {
            q = push(&mut b, -1.0);
            n_unload += 1;
        }
        assert!(
            n_unload < 29,
            "除荷完了までの戻し量 {n_unload}mm が押し量より小さい（残留変形）"
        );

        // (3) 再載荷: 中間点は最大経験点への割線上（Qu より明確に小さい）で、
        // ピーク変位まで戻すと Qu へ復帰する。
        let mut q_mid = 0.0;
        for i in 0..n_unload {
            q = push(&mut b, 1.0);
            if i == n_unload / 2 {
                q_mid = q;
            }
        }
        assert!(
            q_mid * sgn < qu * 0.9,
            "再載荷中間点はピンチング（割線上）: {:.3e} vs Qu={:.3e}",
            q_mid,
            qu
        );
        assert!(
            (q * sgn - qu).abs() <= qu * 0.05,
            "最大経験変位で Qu へ復帰: {:.3e} vs {:.3e}",
            q,
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

#[cfg(test)]
mod capacity_issue_tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::MaterialCategory;
    use squid_n_core::model::{
        ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node, Section,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    /// 側柱あり／なし、側柱断面の指定を切り替えて壁モデルを作る。
    fn model_with(side_col_sec: Option<Section>, ps: f64) -> (Model, ElementData) {
        let shape = SectionShape::RcWall {
            thickness: 200.0,
            ps,
        };
        let mk = |id: u32, c: [f64; 3]| Node {
            id: NodeId(id),
            coord: c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let mut sections = vec![shape.to_section(SectionId(0), "W200".into())];
        let mut elements = vec![];
        let mut edge = |id: u32, n0: u32, n1: u32, sec: Option<SectionId>| {
            elements.push(ElementData {
                id: ElemId(id),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
                section: sec,
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
        };
        let side_sec = side_col_sec.map(|mut cs| {
            cs.id = SectionId(1);
            sections.push(cs);
            SectionId(1)
        });
        // 上下辺の大梁。耐震壁は上下辺が大梁で囲まれた壁を対象とする
        // （`misc_wall::wall_is_framed`）。ElemId は壁（0）に続く連番とする。
        edge(1, 0, 1, None); // 下辺
        edge(2, 3, 2, None); // 上辺
                             // 側柱は「側柱あり」のときだけ鉛直辺へ置く。側柱を持たない耐震壁は壁筋比 ps から
                             // 等価引張鉄筋比 pte を算定する正規の対象であり、鉛直材を置かないことで再現する。
                             // 断面の無い鉛直材を置くと「側柱はあるのに主筋量を読み取れない＝入力不備」となる。
        if let Some(sec) = side_sec {
            edge(3, 0, 3, Some(sec)); // 左の鉛直辺（側柱）
            edge(4, 1, 2, None); // 右の鉛直辺
        }
        let wall = ElementData {
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
        elements.insert(0, wall.clone());
        let model = Model {
            nodes: vec![
                mk(0, [0.0, 0.0, 0.0]),
                mk(1, [4000.0, 0.0, 0.0]),
                mk(2, [4000.0, 0.0, 3000.0]),
                mk(3, [0.0, 0.0, 3000.0]),
            ],
            elements,
            sections,
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                category: MaterialCategory::Concrete,
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            ..Default::default()
        };
        (model, wall)
    }

    fn rc_col(with_rebar: bool) -> Section {
        let shape = if with_rebar {
            SectionShape::RcRect {
                b: 600.0,
                d: 600.0,
                rebar: RcRebar {
                    main_grade: None,
                    main_x: BarSet {
                        count: 8,
                        dia: 22.0,
                        layers: 2,
                    },
                    main_y: BarSet {
                        count: 4,
                        dia: 22.0,
                        layers: 1,
                    },
                    cover: 40.0,
                    shear: ShearBar {
                        dia: 10.0,
                        pitch: 100.0,
                        legs: 2,
                        grade: None,
                    },
                },
            }
        } else {
            // 主筋 0 本（断面設定の不備）。
            SectionShape::RcRect {
                b: 600.0,
                d: 600.0,
                rebar: RcRebar {
                    main_grade: None,
                    main_x: BarSet {
                        count: 0,
                        dia: 0.0,
                        layers: 1,
                    },
                    main_y: BarSet {
                        count: 0,
                        dia: 0.0,
                        layers: 1,
                    },
                    cover: 40.0,
                    shear: ShearBar {
                        dia: 10.0,
                        pitch: 100.0,
                        legs: 2,
                        grade: None,
                    },
                },
            }
        };
        shape.to_section(SectionId(1), "C600".into())
    }

    /// 側柱があり主筋も設定されていれば不備なし・Qu が算定できる。
    #[test]
    fn test_no_issue_when_side_column_rebar_available() {
        let (model, wall) = model_with(Some(rc_col(true)), 0.0025);
        assert_eq!(
            WallPanelElement::wall_shear_capacity_issue(&wall, &model),
            None
        );
        assert!(WallPanelElement::shear_capacity_of(&wall, &model) > 0.0);
    }

    /// 側柱はあるのに主筋が取得できない＝断面設定の不備。壁筋比で代替せずエラーとする。
    #[test]
    fn test_issue_when_side_column_has_no_main_rebar() {
        let (model, wall) = model_with(Some(rc_col(false)), 0.0025);
        let issue = WallPanelElement::wall_shear_capacity_issue(&wall, &model)
            .expect("側柱主筋が無ければ不備として検出されるべき");
        assert!(issue.contains("側柱"), "{}", issue);
        // 壁筋比 ps があっても代替しない（Qu=0 のまま）。
        assert_eq!(WallPanelElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// 側柱が無い壁（壁のみの耐震壁）は不備ではなく、壁筋比から pte を算定する。
    #[test]
    fn test_no_side_column_uses_wall_rebar_ratio() {
        let (model, wall) = model_with(None, 0.0025);
        assert_eq!(
            WallPanelElement::wall_shear_capacity_issue(&wall, &model),
            None
        );
        assert!(WallPanelElement::shear_capacity_of(&wall, &model) > 0.0);
    }

    /// 側柱も壁筋比も無ければ pte を算定できないため不備とする。
    #[test]
    fn test_issue_when_no_side_column_and_no_wall_rebar() {
        let (model, wall) = model_with(None, 0.0);
        let issue = WallPanelElement::wall_shear_capacity_issue(&wall, &model)
            .expect("側柱も壁筋も無ければ不備");
        assert!(issue.contains("壁筋比"), "{}", issue);
    }

    /// コンクリート強度 Fc が未設定の壁は不備として検出する。
    /// Qu を算定できず弾性のまま解析すると保有水平耐力を過大評価する（危険側）。
    #[test]
    fn test_issue_when_fc_unset() {
        let (mut model, wall) = model_with(None, 0.0025);
        model.materials[0].fc = None;
        let issue =
            WallPanelElement::wall_shear_capacity_issue(&wall, &model).expect("Fc 未設定は不備");
        assert!(issue.contains("Fc"), "{}", issue);
        assert_eq!(WallPanelElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// Fc が 0 以下でも Qu を算定できないため不備とする（未設定と同じ扱い）。
    #[test]
    fn test_issue_when_fc_not_positive() {
        let (mut model, wall) = model_with(None, 0.0025);
        model.materials[0].fc = Some(0.0);
        let issue =
            WallPanelElement::wall_shear_capacity_issue(&wall, &model).expect("Fc<=0 は不備");
        assert!(issue.contains("Fc"), "{}", issue);
        assert_eq!(WallPanelElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// 材料が割り当てられていない壁も不備とする（Fc を参照できない）。
    #[test]
    fn test_issue_when_material_missing() {
        let (model, mut wall) = model_with(None, 0.0025);
        wall.material = None;
        let issue =
            WallPanelElement::wall_shear_capacity_issue(&wall, &model).expect("材料未設定は不備");
        assert!(issue.contains("材料が設定されていません"), "{}", issue);
        assert_eq!(WallPanelElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// 断面が割り当てられていない壁も不備とする（壁厚・壁筋比を参照できない）。
    #[test]
    fn test_issue_when_section_missing() {
        let (model, mut wall) = model_with(None, 0.0025);
        wall.section = None;
        let issue =
            WallPanelElement::wall_shear_capacity_issue(&wall, &model).expect("断面未設定は不備");
        assert!(issue.contains("断面が設定されていません"), "{}", issue);
        assert_eq!(WallPanelElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// 4 節点を与えているのに幾何が退化した壁（下辺の 2 節点が同一座標）は、
    /// 壁エレメントも雑壁も組めず剛性・耐力を持たないまま消えるため不備として検出する。
    #[test]
    fn test_issue_when_wall_panel_cannot_be_built() {
        let (mut model, wall) = model_with(None, 0.0025);
        model.nodes[1].coord = model.nodes[0].coord;
        let issue = WallPanelElement::wall_shear_capacity_issue(&wall, &model)
            .expect("壁エレメントを構築できない壁は不備");
        assert!(
            issue.contains("壁エレメントとして構築できません"),
            "{}",
            issue
        );
    }
}
