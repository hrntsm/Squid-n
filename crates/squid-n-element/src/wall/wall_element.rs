//! 耐震壁（壁エレメントモデル）要素。

use crate::behavior::{Ctx, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::frame::beam::BeamElement;
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::geom::vec3::{dot, midpoint as mid, norm, sub, unit};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, HysteresisModel, Model};
use squid_n_core::section_shape::{SectionShape, E_STEEL, KAPPA_RC};

/// 耐震壁（壁エレメントモデル）。
pub struct WallElement {
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
    /// internal_force はこちらを参照する。
    trial_disp: [f64; 24],
    /// 面内せん断の終局強度 Qu [N]。`0` 以下は降伏しない（線形弾性）。
    qu_shear: f64,
    /// 面内せん断モードベクトル p（24 自由度）。上辺 2 節点の並進を壁面内水平方向
    /// `ex_bottom` へ 1.0 ずつ与えたもの。`pᵀ·f` は上辺が伝達する面内水平力に等しく、
    /// `u − γp·p` で塑性すべりを差し引く（下記 [`WallElement::shear_return_map`]）。
    shear_mode: [f64; 24],
    /// 確定塑性せん断すべり γp [mm]。
    committed_slip: f64,
    /// トライアル塑性せん断すべり γp [mm]。
    trial_slip: f64,
    /// 面内せん断の復元力ばね。`None` は弾完全塑性リターンマッピング。
    shear_spring: Option<Box<dyn squid_n_material::UniaxialMaterial>>,
    /// せん断ばねの変形測度 D = γp + Q/k_s0 に用いる弾性モード剛性
    /// k_s0 = pᵀ·K_elastic·p [N/mm]（ばね骨格の初期剛性と共有）。
    shear_k0: f64,
    /// 壁柱の軸・曲げの弾塑性評価（ファイバー断面＋塑性増分ヒンジ）。
    /// `Some` のとき軸・曲げの応答（剛性・内力）はこのファイバー壁柱から得る。
    fiber_column: Option<crate::frame::fiber::FiberBeam>,
    /// ファイバー壁柱へ与え済みの壁柱端変位（グローバル系 12）。トライアル/確定。
    fiber_u12_trial: [f64; 12],
    fiber_u12_committed: [f64; 12],
}

/// 壁エレメント（4 節点）の幾何。
///
/// 節点は入力順に依らず標高 z で下辺 2 節点・上辺 2 節点に分ける。上辺は
/// 下辺 a に近い方を a として対応付ける。壁長 `lw` は上下辺長さの平均とする。
pub struct WallElementGeometry {
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

/// 壁エレメント（4 節点）の幾何を算定する（[`WallElementGeometry`]）。
///
/// 4 節点未満・節点参照が欠落・退化（辺長や高さが 0）の場合は `None`。
pub fn wall_element_geometry(data: &ElementData, model: &Model) -> Option<WallElementGeometry> {
    if data.nodes.len() < 4 {
        return None;
    }
    let ids: Vec<NodeId> = data.nodes.iter().take(4).copied().collect();
    let coords: Vec<[f64; 3]> = ids
        .iter()
        .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
        .collect::<Option<Vec<_>>>()?;

    let mut order: Vec<usize> = (0..4).collect();
    order.sort_by(|&a, &b| coords[a][2].total_cmp(&coords[b][2]));
    let (b0, b1, t0, t1) = (order[0], order[1], order[2], order[3]);

    let (pa, pb) = (coords[b0], coords[b1]);
    let ex_bot = unit(sub(pb, pa))?;
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
    Some(WallElementGeometry {
        bottom: [ids[b0], ids[b1]],
        top: [ids[ta], ids[tb]],
        lw_bottom: lw_bot,
        lw_top,
        lw: 0.5 * (lw_bot + lw_top),
        h,
        ex_bottom: ex_bot,
        bottom_center: bc,
        top_center: tc,
    })
}

/// 増分解析で壁柱がファイバー化されるときの塑性化域長 Lp [mm]。
/// ファイバー化されない壁（耐震壁不成立・Qu を算定できない・Fc 未設定など）は `None`。
pub fn wall_column_fiber_lp(data: &ElementData, model: &Model) -> Option<f64> {
    if !crate::wall::misc_wall::wall_is_seismic(data, model) {
        return None;
    }
    if WallElement::shear_capacity_of(data, model) <= 0.0 {
        return None;
    }
    let geom = wall_element_geometry(data, model)?;
    model.element_material(data)?.fc.filter(|fc| *fc > 0.0)?;
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let t = match sec.and_then(|s| s.shape.as_ref()) {
        Some(SectionShape::RcWall { thickness, .. }) => *thickness,
        _ => sec.map(|s| s.thickness.unwrap_or(s.width))?,
    };
    if t <= 0.0 || geom.lw <= 0.0 || geom.h <= 0.0 {
        return None;
    }
    Some(crate::frame::fiber::clamp_plastic_zone(
        0.5 * geom.lw,
        geom.h,
    ))
}

/// 壁要素の面積等価開口寸法 `(l0, h0)` [mm]。
/// 複数開口は面積等価の 1 開口へまとめる（lo = Σli、ho = Σ(li·hi)/lo）。
/// 個別寸法が取れない・開口なしのときは `None`。
fn wall_opening_equiv_dims(data: &ElementData, model: &Model) -> Option<(f64, f64)> {
    model
        .wall_attrs
        .iter()
        .find(|w| w.elem == data.id)
        .and_then(|a| a.opening_dims_for(model.multi_opening_mode))
        .and_then(|dims| {
            let lo: f64 = dims.iter().map(|(l, _)| *l).sum();
            let area: f64 = dims.iter().map(|(l, h)| l * h).sum();
            (lo > 0.0 && area > 0.0).then_some((lo, area / lo))
        })
}

/// 耐震壁の幾何・配筋・材料。
struct WallShearGeometry {
    /// コンクリート設計基準強度 Fc [N/mm²]（未設定は `None`）
    fc: Option<f64>,
    /// 壁厚 t [mm]
    t: f64,
    /// 付帯柱中心間距離 lw [mm]
    lw: f64,
    /// 壁の上下梁中心間高さ h [mm]
    h: f64,
    /// 壁筋比 ps（小数）
    ps: f64,
    /// 側柱 1 本あたりの沿壁方向せい Dc [mm]
    dc_each: f64,
    /// 引張側柱の主筋断面積 at [mm²]
    col_main_at: f64,
    /// 側柱（付帯柱）があるか
    has_side_column: bool,
    /// 壁横筋の降伏点 σwh [N/mm²]
    sigma_wh: f64,
    /// 壁横筋が高強度せん断補強筋か
    high_strength_shear_rebar: bool,
    /// 開口寸法 `(l0, h0)` [mm]（無開口は `None`）
    opening: Option<(f64, f64)>,
}

impl WallElement {
    /// 生成。4 節点未満・寸法/断面が不定の場合は None。
    pub fn try_new(data: &ElementData, model: &Model) -> Option<Self> {
        Self::try_new_scaled(data, model, 1.0)
    }

    /// 剛性スケール付き生成。`stiffness_scale` で壁要素の剛性をスケールする。
    pub(crate) fn try_new_scaled(
        data: &ElementData,
        model: &Model,
        stiffness_scale: f64,
    ) -> Option<Self> {
        let geom = wall_element_geometry(data, model)?;
        let (ids_b0, ids_b1) = (geom.bottom[0], geom.bottom[1]);
        let (ids_ta, ids_tb) = (geom.top[0], geom.top[1]);
        let coord_of =
            |nid: NodeId| -> Option<[f64; 3]> { model.nodes.get(nid.index()).map(|n| n.coord) };
        let ex_bot = geom.ex_bottom;
        let ex_top = unit(sub(coord_of(ids_tb)?, coord_of(ids_ta)?))?;
        let (bc, tc) = (geom.bottom_center, geom.top_center);
        let h = geom.h;
        let lw = geom.lw;

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
        let mat = model.element_material(data)?;

        let r = crate::factory::wall_opening_reduction(data, model).max(1e-6);

        let ps = match &sec.shape {
            Some(SectionShape::RcWall { ps, .. }) => (*ps).max(0.0),
            _ => 0.0,
        };
        let rebar_factor = if mat.fc.is_some() && mat.young > 0.0 && ps > 0.0 {
            1.0 + (E_STEEL / mat.young - 1.0) * ps
        } else {
            1.0
        };

        let edge_pairs = [[ids_b0, ids_ta], [ids_b1, ids_tb]];
        let mut col_area_sum = 0.0;
        let mut col_depth_sum = 0.0;
        let mut col_width_max: f64 = 0.0;
        let mut col_main_at: f64 = 0.0;
        let side_columns_released = crate::wall::misc_wall::wall_is_seismic(data, model);
        for e in &model.elements {
            if !side_columns_released {
                break;
            }
            if !crate::wall::side_column::is_side_column_member(e.kind) || e.nodes.len() < 2 {
                continue;
            }
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
                if let Some(SectionShape::RcRect { rebar, .. }) = cs.shape.as_ref() {
                    col_main_at =
                        col_main_at.max(squid_n_core::section_shape::bar_set_area(&rebar.main_x));
                }
            }
        }
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
            iy: lw * t.powi(3) / 12.0,
            iz: t * lw.powi(3) / 12.0 * rebar_factor,
            j: lw * t.powi(3) / 3.0,
            as_y: r * as_gross / kappa,
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
            torsion_release: [false, false],
            eval_sections: vec![0.0, 0.5, 1.0],
            section: data.section,
            material: model.element_section(data).and_then(|s| s.material),
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
            local_stiffness_cache: std::sync::OnceLock::new(),
        };

        let mut a_mat = vec![0.0; 12 * 24];
        let corner_slot = |idx: usize| -> usize { idx * 6 };
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
    /// 壁柱の軸・曲げをファイバー断面の弾塑性評価に切り替える。
    /// コンクリート強度 Fc がない等でファイバー断面を組めない場合は弾性のまま返す。
    pub(crate) fn with_fiber_flexure(
        mut self,
        data: &ElementData,
        model: &Model,
        basis: crate::factory::StrengthBasis,
        kind: squid_n_core::model::AnalysisKind,
    ) -> Self {
        let Some(geom) = wall_element_geometry(data, model) else {
            return self;
        };
        let Some(mat) = model.element_material(data) else {
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

        let nw = 4;
        let nd = 20;
        let rebar_fy = 345.0 * basis.rebar_factor(Some(mat));
        let concrete_rule = crate::factory::resolve_wall_concrete_hysteresis(data, model, kind);
        let make_section = || {
            let (mut section, mut mats) = crate::frame::fiber::build_gauss_fibers(
                t,
                lw,
                nw,
                nd,
                None,
                Some(fc),
                mat.young,
                crate::frame::fiber::FiberYield::default(),
                1.0,
                1.0,
                concrete_rule,
            );
            if ps > 0.0 {
                let a_each = ps * t * lw / nd as f64;
                for i in 0..nd {
                    let y = ((i as f64 + 0.5) / nd as f64 - 0.5) * lw;
                    section.fibers.push(squid_n_section::fiber::Fiber {
                        y,
                        z: 0.0,
                        area: a_each,
                        material: 1,
                    });
                    mats.push(crate::frame::fiber::steel_fiber_material(
                        E_STEEL,
                        Some(rebar_fy),
                    ));
                }
            }
            (section, mats)
        };

        let col = &self.column;
        let fiber = crate::frame::fiber::FiberBeam::from_raw_parts(
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

    /// 耐震壁の面内せん断終局強度 Qu [N]（荒川mean式系）。
    /// 開口低減は耐力用 r2 = 1−max(r0, l0/lw, h0/h)（剛性用 r1 = 1−1.25·r0 とは別式）。
    /// 算定できない場合（Fc 未設定など）は 0.0 を返す。
    fn shear_capacity(inp: &WallShearGeometry) -> f64 {
        let &WallShearGeometry {
            fc,
            t,
            lw,
            h,
            ps,
            dc_each,
            col_main_at,
            has_side_column,
            sigma_wh,
            high_strength_shear_rebar,
            opening,
        } = inp;
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
                sigma_wh,
                pwh_ratio: ps.max(0.0),
                sigma_0: 0.0,
                shear_span_ratio: h / d_wall,
                high_strength_shear_rebar,
                opening: opening.map(|(l0, h0)| (l0, h0, h, lw)),
            },
        )
    }

    /// 耐力用開口低減率 r2（無開口は 1.0）。
    pub fn opening_strength_reduction(data: &ElementData, model: &Model) -> f64 {
        let Some(geom) = wall_element_geometry(data, model) else {
            return 1.0;
        };
        let opening =
            wall_opening_equiv_dims(data, model).map(|(l0, h0)| (l0, h0, geom.h, geom.lw));
        squid_n_core::rc_wall_capacity::wall_opening_reduction_strength(opening)
    }

    /// この壁の面内せん断終局強度 Qu [N] を、要素と同じ幾何・配筋から算定する。
    pub fn shear_capacity_of(data: &ElementData, model: &Model) -> f64 {
        let Some(geom) = wall_element_geometry(data, model) else {
            return 0.0;
        };
        let Some(sec) = data.section.and_then(|sid| model.sections.get(sid.index())) else {
            return 0.0;
        };
        let (t, ps) = match &sec.shape {
            Some(SectionShape::RcWall { thickness, ps }) => (*thickness, (*ps).max(0.0)),
            _ => (sec.thickness.unwrap_or(sec.width), 0.0),
        };
        if !crate::wall::misc_wall::is_rc_wall(data, model) {
            return Self::steel_shear_capacity_of(data, model);
        }
        let fc = model.element_material(data).and_then(|m| m.fc);
        let edge_pairs = [[geom.bottom[0], geom.top[0]], [geom.bottom[1], geom.top[1]]];
        let mut col_depth_sum = 0.0;
        let mut col_main_at: f64 = 0.0;
        let mut has_side_column = false;
        for e in &model.elements {
            if !crate::wall::side_column::is_side_column_member(e.kind) || e.nodes.len() < 2 {
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
        let opening = wall_opening_equiv_dims(data, model);
        let shear_mat = model.element_shear_rebar_material(data);
        let sigma_wh = squid_n_core::material_grade::shear_rebar_yield_strength(shear_mat)
            .unwrap_or(squid_n_core::material_grade::SHEAR_REBAR_DEFAULT_FY);
        Self::shear_capacity(&WallShearGeometry {
            fc,
            t,
            lw: geom.lw,
            h: geom.h,
            ps,
            dc_each: col_depth_sum / 2.0,
            col_main_at,
            has_side_column,
            sigma_wh,
            high_strength_shear_rebar:
                squid_n_core::material_grade::is_high_strength_shear_material(shear_mat),
            opening,
        })
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
    /// せん断座屈は考慮していない。
    pub fn steel_shear_capacity_of(data: &ElementData, model: &Model) -> f64 {
        let Some(geom) = wall_element_geometry(data, model) else {
            return 0.0;
        };
        let Some(sec) = data.section.and_then(|sid| model.sections.get(sid.index())) else {
            return 0.0;
        };
        let t = sec.thickness.unwrap_or(sec.width);
        let f = model
            .element_material(data)
            .and_then(|m| m.fy)
            .unwrap_or(0.0);
        if t <= 0.0 || geom.lw <= 0.0 || f <= 0.0 {
            return 0.0;
        }
        t * geom.lw * f / 3.0_f64.sqrt()
    }

    /// 耐震壁のせん断終局強度 Qu を算定できない設定不備があれば、その内容を返す。
    ///
    /// 検出する不備:
    /// - 壁エレメントとして構築できない（4 節点未満／節点座標が退化）。
    /// - 断面が設定されていない／壁厚が 0 以下。
    /// - 材料が設定されていない。
    /// - 材料にコンクリート強度 Fc が設定されていない、または Fc が 0 以下。
    /// - 付帯柱（側柱）はあるのに、その断面から主筋量を読み取れない
    ///   （断面形状が RcRect でない／主筋本数・径が 0）。
    /// - 上記のいずれにも当てはまらないが Qu が 0 以下になる。
    ///
    /// 付帯柱がない壁（壁のみの耐震壁）は不備ではなく、壁の縦筋比 ps から pte を
    /// 算定する。`ps = 0` の場合は不備とする。
    pub fn wall_shear_capacity_issue(data: &ElementData, model: &Model) -> Option<String> {
        if !matches!(data.kind, squid_n_core::model::ElementKind::Wall) {
            return None;
        }
        let geom = match wall_element_geometry(data, model) {
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
        if !crate::wall::misc_wall::wall_is_seismic(data, model) {
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
        let Some(mat) = model.element_material(data) else {
            return Some(format!(
                "耐震壁 ID {} に材料が設定されていません。\
                 材料タブで材料を割り当ててください。\
                 保有水平耐力計算では耐震壁の終局せん断強度が必要です。",
                data.id.0
            ));
        };
        if !crate::wall::misc_wall::is_rc_wall(data, model) {
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
            if !crate::wall::side_column::is_side_column_member(e.kind) || e.nodes.len() < 2 {
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
                "耐震壁 ID {} は側柱（付帯柱）がなく、かつ壁筋比 ps が 0 です。\
                 断面タブで壁筋比を設定してください。\
                 保有水平耐力計算では壁筋比から耐震壁の等価引張鉄筋比 pte を算定します。",
                data.id.0
            ));
        }
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

    /// 面内せん断の復元力ばねを構築する。
    /// 骨格は弾完全塑性（初期剛性 k_s0、耐力 Qu で頭打ち）とし、
    /// 除荷・再載荷則のみ `rule` に従う。
    /// `qu_shear <= 0` や k_s0 が取れない場合は何もしない。
    pub(crate) fn with_shear_hysteresis(mut self, rule: HysteresisModel) -> Self {
        use squid_n_material::{HysteresisMaterial, HysteresisRule};
        if self.qu_shear <= 0.0 {
            return self;
        }
        let k12 = self.column.axis.to_global(&self.column.local_stiffness());
        let mut v = [0.0_f64; 12];
        for (i, vi) in v.iter_mut().enumerate() {
            let mut acc = 0.0;
            for q in 0..24 {
                acc += self.a_mat[i * 24 + q] * self.shear_mode[q];
            }
            *vi = acc;
        }
        let mut w = [0.0_f64; 12];
        for (i, wi) in w.iter_mut().enumerate() {
            let mut acc = 0.0;
            for (j, vj) in v.iter().enumerate() {
                acc += k12.get(i, j) * vj;
            }
            *wi = acc;
        }
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
            Some(f) => f.tangent_stiffness(ctx),
            None => self.column.axis.to_global(&self.column.local_stiffness()),
        }
    }

    /// 壁柱の現在トライアル状態の全体系内力（24 自由度）。ファイバー壁柱専用
    /// （履歴に整合した復元力 f24 = Aᵀ·f12）。
    fn f24_fiber(&self, ctx: &Ctx) -> Option<[f64; 24]> {
        let f = self.fiber_column.as_ref()?;
        let f12 = f.internal_force(ctx);
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

impl ElementBehavior for WallElement {
    fn n_dof(&self) -> usize {
        24
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        crate::behavior::node_global_dofs(&self.nodes, dof)
    }

    fn tangent_stiffness(&self, ctx: &Ctx) -> LocalMat {
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
        let factor = if self.shear_spring.is_some() && self.shear_k0 > 0.0 {
            0.0
        } else {
            let yielded = match self.inplane_shear_fiber(ctx) {
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

    fn internal_force(&self, ctx: &Ctx) -> LocalVec {
        if let Some(f24) = self.f24_fiber(ctx) {
            return LocalVec {
                data: smallvec::SmallVec::from_slice(&f24),
            };
        }
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
            let k = self.stiffness_24(ctx);
            let (slip, _) = self.shear_return_map(&k, &self.trial_disp);
            self.trial_slip = slip;
        }
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
        let (committed, trial, cslip, tslip, fsnap, u12t, u12c, spring) =
            crate::behavior::downcast_snapshot::<Snapshot>("WallElement", state);
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
            sp.deserialize_state(bytes)
                .expect("壁せん断ばねのスナップショット復元");
        }
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        let cp = WallElementCheckpoint {
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
        if data.is_empty() {
            return Ok(());
        }
        if let Ok(cp) = bincode::deserialize::<WallElementCheckpoint>(data) {
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

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::frame::beam::MemberForces> {
        if u_elem.len() < 24 {
            return None;
        }
        let u12 = self.to_column_disp(&u_elem[..24]);
        Some(self.column.recover_forces(&u12))
    }
}

/// [`WallElement`] のチェックポイント形式。
#[derive(serde::Serialize, serde::Deserialize)]
struct WallElementCheckpoint {
    committed_disp: [f64; 24],
    trial_disp: [f64; 24],
    committed_slip: f64,
    trial_slip: f64,
    /// ファイバー壁柱のチェックポイント（弾性壁柱は None）。
    fiber: Option<Vec<u8>>,
    fiber_u12_trial: [f64; 12],
    fiber_u12_committed: [f64; 12],
    /// 面内せん断ばねの材料状態（ばね未構築は None）。
    #[serde(default)]
    shear_spring: Option<Vec<u8>>,
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
            sections: vec![squid_n_core::model::Section {
                material: Some(MaterialId(0)),
                ..shape.to_section(SectionId(0), "W150".into())
            }],
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
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
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
    fn test_wall_element_rigid_translation_zero_force() {
        let (model, data) = make_wall_model();
        let wall = WallElement::try_new(&data, &model).unwrap();
        let ctx = Ctx { model: &model };
        let k = wall.stiffness_24(&ctx);
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
    fn test_wall_element_inplane_shear_matches_column() {
        let (model, data) = make_wall_model();
        let wall = WallElement::try_new(&data, &model).unwrap();
        let ctx = Ctx { model: &model };
        let k = wall.stiffness_24(&ctx);
        let mut u = [0.0; 24];
        u[2 * 6] = 1.0;
        u[3 * 6] = 1.0;
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
    fn test_wall_element_vertical_matches_axial() {
        let (model, data) = make_wall_model();
        let wall = WallElement::try_new(&data, &model).unwrap();
        let ctx = Ctx { model: &model };
        let k = wall.stiffness_24(&ctx);
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
    fn test_wall_element_corner_rotations_are_pinned() {
        let (model, data) = make_wall_model();
        let wall = WallElement::try_new(&data, &model).unwrap();
        let ctx = Ctx { model: &model };
        let k = wall.stiffness_24(&ctx);
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
    fn test_opening_strength_reduction_no_opening_is_one() {
        let (model, data) = make_wall_model();
        assert!((WallElement::opening_strength_reduction(&data, &model) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_opening_strength_reduction_with_opening() {
        use squid_n_core::model::{WallAttr, WallOpening};

        let (mut model, data) = make_wall_model();
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            slit: Default::default(),
            finish_intensity: 0.0,
            openings: vec![WallOpening {
                width: 2000.0,
                height: 1500.0,
                offset: None,
            }],
        });
        let r2 = WallElement::opening_strength_reduction(&data, &model);
        let expected = squid_n_core::rc_wall_capacity::wall_opening_reduction_strength(Some((
            2000.0, 1500.0, 3000.0, 4000.0,
        )));
        assert!((r2 - expected).abs() < 1e-12);
        assert!(r2 < 1.0);
        assert!((r2 - 0.5).abs() < 1e-12);
    }

    #[test]
    fn test_wall_element_opening_reduces_inplane_shear() {
        let (mut model, data) = make_wall_model();
        let wall = WallElement::try_new(&data, &model).unwrap();
        let k_no = {
            let ctx = Ctx { model: &model };
            wall.stiffness_24(&ctx)
        };
        model.wall_attrs.push(squid_n_core::model::WallAttr {
            elem: ElemId(0),
            opening_area: 3.0e6,
            opening_weight: 0.0,
            slit: Default::default(),
            openings: vec![],
            finish_intensity: 0.0,
        });
        let wall_o = WallElement::try_new(&data, &model).unwrap();
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
    fn test_wall_element_rebar_factor() {
        let (model, data) = make_wall_model();
        let wall = WallElement::try_new(&data, &model).unwrap();
        let n = squid_n_core::section_shape::E_STEEL / 23000.0;
        let expected = 150.0 * 4000.0 * (1.0 + (n - 1.0) * 0.0025);
        assert!((wall.column.a - expected).abs() < 1e-6);
        assert!((wall.column.a_mass - 150.0 * 4000.0).abs() < 1e-9);
    }

    /// 側柱があるとせん断断面に算入され、I 形の形状係数 κ が用いられる。
    #[test]
    fn test_wall_element_side_columns_increase_shear_area() {
        let (mut model, data) = make_wall_model();
        let wall_plain = WallElement::try_new(&data, &model).unwrap();

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
                },
            },
        };
        model
            .sections
            .push(col_shape.to_section(SectionId(1), "C600".into()));
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
        let wall_cols = WallElement::try_new(&data, &model).unwrap();
        assert!(
            wall_cols.column.as_y > wall_plain.column.as_y,
            "側柱算入で as_y が増えない: {} vs {}",
            wall_cols.column.as_y,
            wall_plain.column.as_y
        );
        let a_gross = 150.0 * 4000.0 + 2.0 * 360_000.0;
        let kappa = a_gross / wall_cols.column.as_y;
        assert!(
            (kappa - 1.2).abs() > 1e-3,
            "κ が I 形になっていない: {kappa}"
        );
    }

    #[test]
    fn test_wall_element_try_new_fallbacks() {
        let (model, mut data) = make_wall_model();
        data.nodes = smallvec::smallvec![NodeId(0), NodeId(2)];
        assert!(WallElement::try_new(&data, &model).is_none());
    }

    /// トライアル追従の回帰テスト: update_state(du, commit=false) が internal_force に
    /// 反映され（内力 = K24·u と厳密に一致）、commit / revert / snapshot / restore が
    /// beam/behavior.rs と同じ規律で機能すること。
    ///
    /// 本テストの K·u 比較は「internal_force と tangent_stiffness が将来ズレない」
    /// ことの回帰ガードであり、K24 の値そのものの正しさは独立の解析解と照合する
    /// `test_wall_element_inplane_shear_matches_column`（12EI/((1+φ)h³)）が担保する
    /// （両者を合わせて非循環な検証となる）。
    #[test]
    fn test_wall_element_trial_displacement_tracking() {
        use crate::behavior::{Ctx, ElementBehavior, LocalVec};
        let (model, data) = make_wall_model();
        let mut wall = WallElement::try_new(&data, &model).unwrap();
        let ctx = Ctx { model: &model };

        let mut du = LocalVec {
            data: smallvec::smallvec![0.0; 24],
        };
        du.data[2 * 6] = 1.0;
        du.data[3 * 6] = 1.0;
        let snap = wall.snapshot_state();
        wall.update_state(&du, false, &ctx);

        let f = wall.internal_force(&ctx);
        let k = wall.stiffness_24(&ctx);
        for i in 0..24 {
            let expected: f64 = (0..24).map(|j| k.get(i, j) * du.data[j]).sum();
            assert!(
                (f.data[i] - expected).abs() <= 1e-9 * expected.abs().max(1.0),
                "内力が K·u と不一致: i={i} f={} expected={expected}",
                f.data[i]
            );
        }
        assert!(f.data[2 * 6].abs() > 1.0, "壁の復元力が生じていない");

        wall.commit_state();
        wall.update_state(&du, false, &ctx);
        wall.revert_state();
        let f2 = wall.internal_force(&ctx);
        for i in 0..24 {
            assert!((f2.data[i] - f.data[i]).abs() < 1e-9);
        }

        wall.restore_state(&*snap);
        let f0 = wall.internal_force(&ctx);
        assert!(f0.data.iter().all(|v| v.abs() < 1e-12));
    }
}

#[cfg(test)]
mod geometry_tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, SectionId};
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
        let g = wall_element_geometry(&data, &model).expect("Some");
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
        let (model, data) = wall_with(coords, [0, 3, 1, 2]);
        let g = wall_element_geometry(&data, &model).expect("Some");
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
    use crate::behavior::{Ctx, LocalVec};
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
            sections: vec![squid_n_core::model::Section {
                material: Some(MaterialId(0)),
                ..shape.to_section(SectionId(0), "W200".into())
            }],
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
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let mut model = model;
        crate::wall::add_surrounding_frame(&mut model, &data);
        (model, data)
    }

    /// 壁横筋の材料（`SectionShape` によらず断面の `shear_rebar_material`）から
    /// σwh と高強度判定を解決することを確認する。
    ///
    /// - 未割当は SD295 相当（295 N/mm²・普通強度）を既定とする
    /// - SD295 を明示的に割り当てても未割当と同じ Qu になる
    /// - SD390 を割り当てると σwh が上がり Qu が増える
    /// - 高強度品（KH785）は Qu 係数が 0.053 → 0.068 へ切り替わり、さらに増える
    #[test]
    fn test_wall_qu_uses_section_shear_rebar_material() {
        let rebar = |id: u32, name: &str, fy: f64| Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(id),
            name: name.into(),
            category: MaterialCategory::Rebar,
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(fy),
        };
        let (mut model, data) = wall_model();
        let qu_unassigned = WallElement::shear_capacity_of(&data, &model);
        assert!(qu_unassigned > 0.0, "Qu が算定できるはず");

        model.materials.push(rebar(1, "SD295A", 295.0));
        model.materials.push(rebar(2, "SD390", 390.0));
        model.materials.push(rebar(3, "KH785", 785.0));

        model.sections[0].shear_rebar_material = Some(MaterialId(1));
        let qu_sd295 = WallElement::shear_capacity_of(&data, &model);
        assert!(
            (qu_sd295 - qu_unassigned).abs() < 1e-6,
            "未割当の既定は SD295 相当のはず: {qu_unassigned:.6e} vs {qu_sd295:.6e}"
        );

        model.sections[0].shear_rebar_material = Some(MaterialId(2));
        let qu_sd390 = WallElement::shear_capacity_of(&data, &model);
        assert!(
            qu_sd390 > qu_sd295,
            "SD390 の Qu {qu_sd390:.6e} が SD295 の Qu {qu_sd295:.6e} を超えていない"
        );

        model.sections[0].shear_rebar_material = Some(MaterialId(3));
        let qu_kh785 = WallElement::shear_capacity_of(&data, &model);
        assert!(
            qu_kh785 > qu_sd390,
            "高強度せん断補強筋の Qu {qu_kh785:.6e} が SD390 の Qu {qu_sd390:.6e} を超えていない"
        );
    }

    /// 非線形経路（プッシュオーバー）では耐震壁の面内水平力が終局せん断強度 Qu で
    /// 頭打ちになる。
    #[test]
    fn test_wall_shear_yields_at_ultimate_strength() {
        let (model, data) = wall_model();
        let qu = WallElement::shear_capacity_of(&data, &model);
        assert!(qu > 0.0, "Qu が算定できるはず");

        let mut b = crate::factory::build_nonlinear_behavior(
            &data,
            &model,
            crate::factory::StrengthBasis::MaterialStrength,
            crate::factory::AnalysisKind::Incremental,
        );
        let ctx = Ctx { model: &model };
        let mut max_q: f64 = 0.0;
        for _ in 0..300 {
            let mut du = LocalVec {
                data: smallvec::SmallVec::from_elem(0.0, 24),
            };
            du.data[12] = 1.0;
            du.data[18] = 1.0;
            b.update_state(&du, false, &ctx);
            b.commit_state();
            let f = b.internal_force(&ctx);
            max_q = max_q.max((f.data[0] + f.data[6]).abs());
        }
        assert!(
            max_q <= qu * 1.001,
            "壁の水平力 {:.3e} N が終局せん断強度 Qu={:.3e} N を超えている",
            max_q,
            qu
        );
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
        let qu = WallElement::shear_capacity_of(&data, &model);
        assert!(qu > 0.0);
        let mut b = crate::factory::build_nonlinear_behavior(
            &data,
            &model,
            crate::factory::StrengthBasis::MaterialStrength,
            crate::factory::AnalysisKind::TimeHistory,
        );
        let ctx = Ctx { model: &model };
        let push = |b: &mut Box<dyn ElementBehavior>, d: f64| -> f64 {
            let mut du = LocalVec {
                data: smallvec::SmallVec::from_elem(0.0, 24),
            };
            du.data[12] = d;
            du.data[18] = d;
            b.update_state(&du, false, &ctx);
            b.commit_state();
            let f = b.internal_force(&ctx);
            f.data[0] + f.data[6]
        };

        let mut q_peak = 0.0;
        for _ in 0..30 {
            q_peak = push(&mut b, 1.0);
        }
        assert!(
            (q_peak.abs() - qu).abs() <= qu * 0.02,
            "ピークで Qu: {q_peak:.3e} vs {qu:.3e}"
        );
        let sgn = q_peak.signum();

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

    /// 弾性経路（許容応力度計算）では降伏しない（線形）。
    #[test]
    fn test_wall_stays_elastic_in_linear_path() {
        let (model, data) = wall_model();
        let mut b = crate::factory::build_behavior(&data, &model);
        let ctx = Ctx { model: &model };
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
                let f = b.internal_force(&ctx);
                q_at.push((f.data[0] + f.data[6]).abs());
            }
        }
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
        let mut sections = vec![Section {
            material: Some(MaterialId(0)),
            ..shape.to_section(SectionId(0), "W200".into())
        }];
        let mut elements = vec![];
        let mut edge = |id: u32, n0: u32, n1: u32, sec: Option<SectionId>| {
            elements.push(ElementData {
                id: ElemId(id),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
                section: sec,
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
            cs.material = Some(MaterialId(0));
            cs.rebar_material = Some(MaterialId(0));
            cs.shear_rebar_material = Some(MaterialId(0));
            sections.push(cs);
            SectionId(1)
        });
        edge(1, 0, 1, None);
        edge(2, 3, 2, None);
        if let Some(sec) = side_sec {
            edge(3, 0, 3, Some(sec));
            edge(4, 1, 2, None);
        }
        let wall = ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(0)),
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
                    },
                },
            }
        } else {
            SectionShape::RcRect {
                b: 600.0,
                d: 600.0,
                rebar: RcRebar {
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
        assert_eq!(WallElement::wall_shear_capacity_issue(&wall, &model), None);
        assert!(WallElement::shear_capacity_of(&wall, &model) > 0.0);
    }

    /// 側柱はあるのに主筋が取得できない＝断面設定の不備。壁筋比で代替せずエラーとする。
    #[test]
    fn test_issue_when_side_column_has_no_main_rebar() {
        let (model, wall) = model_with(Some(rc_col(false)), 0.0025);
        let issue = WallElement::wall_shear_capacity_issue(&wall, &model)
            .expect("側柱主筋がなければ不備として検出されるべき");
        assert!(issue.contains("側柱"), "{}", issue);
        assert_eq!(WallElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// 側柱がない壁（壁のみの耐震壁）は不備ではなく、壁筋比から pte を算定する。
    #[test]
    fn test_no_side_column_uses_wall_rebar_ratio() {
        let (model, wall) = model_with(None, 0.0025);
        assert_eq!(WallElement::wall_shear_capacity_issue(&wall, &model), None);
        assert!(WallElement::shear_capacity_of(&wall, &model) > 0.0);
    }

    /// 側柱も壁筋比もなければ pte を算定できないため不備とする。
    #[test]
    fn test_issue_when_no_side_column_and_no_wall_rebar() {
        let (model, wall) = model_with(None, 0.0);
        let issue = WallElement::wall_shear_capacity_issue(&wall, &model)
            .expect("側柱も壁筋もなければ不備");
        assert!(issue.contains("壁筋比"), "{}", issue);
    }

    /// コンクリート強度 Fc が未設定の壁は不備として検出する。
    /// Qu を算定できず弾性のまま解析すると保有水平耐力を過大評価する（危険側）。
    #[test]
    fn test_issue_when_fc_unset() {
        let (mut model, wall) = model_with(None, 0.0025);
        model.materials[0].fc = None;
        let issue = WallElement::wall_shear_capacity_issue(&wall, &model).expect("Fc 未設定は不備");
        assert!(issue.contains("Fc"), "{}", issue);
        assert_eq!(WallElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// Fc が 0 以下でも Qu を算定できないため不備とする（未設定と同じ扱い）。
    #[test]
    fn test_issue_when_fc_not_positive() {
        let (mut model, wall) = model_with(None, 0.0025);
        model.materials[0].fc = Some(0.0);
        let issue = WallElement::wall_shear_capacity_issue(&wall, &model).expect("Fc<=0 は不備");
        assert!(issue.contains("Fc"), "{}", issue);
        assert_eq!(WallElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// 断面に材料が割り当てられていない壁も不備とする（Fc を参照できない）。
    #[test]
    fn test_issue_when_material_missing() {
        let (mut model, wall) = model_with(None, 0.0025);
        model.sections[0].material = None;
        let issue =
            WallElement::wall_shear_capacity_issue(&wall, &model).expect("材料未設定は不備");
        assert!(issue.contains("材料が設定されていません"), "{}", issue);
        assert_eq!(WallElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// 断面が割り当てられていない壁も不備とする（壁厚・壁筋比を参照できない）。
    #[test]
    fn test_issue_when_section_missing() {
        let (model, mut wall) = model_with(None, 0.0025);
        wall.section = None;
        let issue =
            WallElement::wall_shear_capacity_issue(&wall, &model).expect("断面未設定は不備");
        assert!(issue.contains("断面が設定されていません"), "{}", issue);
        assert_eq!(WallElement::shear_capacity_of(&wall, &model), 0.0);
    }

    /// 4 節点を与えているのに幾何が退化した壁（下辺の 2 節点が同一座標）は、
    /// 壁エレメントも雑壁も組めず剛性・耐力を持たないまま消えるため不備として検出する。
    #[test]
    fn test_issue_when_wall_element_cannot_be_built() {
        let (mut model, wall) = model_with(None, 0.0025);
        model.nodes[1].coord = model.nodes[0].coord;
        let issue = WallElement::wall_shear_capacity_issue(&wall, &model)
            .expect("壁エレメントを構築できない壁は不備");
        assert!(
            issue.contains("壁エレメントとして構築できません"),
            "{}",
            issue
        );
    }
}
