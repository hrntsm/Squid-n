//! 仕口パネル（柱梁接合部パネル）要素。
//!
//! # モデル化
//!
//! 仕口パネルは柱梁接合部の節点に設ける、寸法 `(Bx, By, Dz)` を持つ 6 面体である。
//! パネルは部材座標系 X'-Z' 平面内と Y'-Z' 平面内でせん断モーメント `{mxp, myp}` と
//! せん断変形角 `{γ'x, γ'y}` を持ち、剛性は寸法とせん断弾性係数 G から定まる。
//!
//! ```text
//! {mxp, myp} = [[Kyp, 0], [0, Kxp]] {γ'x, γ'y},   Kyp = Kxp = G・V
//! ```
//!
//! 体積 `V` には実効体積 `Ve`（[`squid_n_core::panel_zone::PanelGeometry::effective_volume`]）
//! を用いる。H 形柱ではウェブ厚方向の寸法を `By = tp` と対応させたものであり、
//! 中実 6 面体ではなく板厚分の実効体積となる。断面検定の降伏モーメント
//! `pMy = (Ve/κ)・√(1−n²)・Fy/√3` と同じ体積を用いるため、剛性と耐力が同一の
//! 諸元で整合する。
//!
//! パネルが設けられた節点は、基準座標系でせん断モーメント `{MSX, MSY}` と
//! せん断変形角 `{γX, γY}` を持つ。節点の変位とパネルの変形は次式で適合させる。
//!
//! ```text
//! {γ'x, γ'y} = [[-1, 0], [0, 1]] [Tp] {γX, γY}
//! {MSX, MSY} = [Tp]ᵀ [[-1, 0], [0, 1]] {mxp, myp}
//! [Tp] = [[cosθ, sinθ], [-sinθ, cosθ]]
//! ```
//!
//! `−1` が現れるのは部材座標と基準座標の方向が逆向きであるため。`θ` はパネルの
//! 部材座標系が基準座標系 X-Y 平面内で回転する角度で、本実装では 0 固定とする
//! （直交フレームを前提とする。変換自体は一般の θ で実装してあるため、将来
//! 斜交フレームへ拡張する際は `theta` を設定するだけでよい）。
//!
//! `Kxp = Kyp` のとき `[Tp]ᵀ [[-1,0],[0,1]] K' [[-1,0],[0,1]] [Tp] = K'` となり、
//! 節点座標系でのパネル剛性は θ に依らず `diag(K, K)` に帰着する。
//!
//! # 追加自由度
//!
//! `{γX, γY}` は節点の標準 6 自由度とは別枠の追加自由度で、
//! [`DofMap`] のグローバル自由度空間の末尾へ払い出される
//! （[`squid_n_core::dof::PANEL_DOF_PER_NODE`]）。本要素はその 2 自由度に対して
//! のみ剛性を与える。パネル分のオフセットを介した部材端との適合
//! （`{d} = {D} + [B0]{Φ} + [Btp]{S}`）は、部材側のデコレータ
//! [`crate::panel_offset::PanelOffsetMember`] が担う。
//!
//! # 弾塑性
//!
//! 増分解析・時刻歴応答解析では、パネルの降伏を考慮する。骨格は
//! `pMy = (Ve/κ)・√(1−n²)・Fy/√3` を降伏点とするバイリニア（二次勾配比
//! [`PANEL_HARDENING`]）で、履歴則は S 造部材の既定と同じ標準型（Masing）とする。
//! 軸力比 `n` は各ステップの柱軸力から更新する（[`ColumnAxial`]）。

use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_core::panel_zone::{beam_panel_depth, PanelGeometry};
use squid_n_material::uniaxial::{Bilinear, UniaxialMaterial};

/// パネル降伏後の二次勾配比（材端集中ばねの既定と同じ）。
pub const PANEL_HARDENING: f64 = 0.01;

/// パネル諸元を解決できなかった場合に用いる剛性 [N·mm/rad]。
///
/// 実効体積 `Ve` が 0 以下になるのは、柱・梁の断面情報が欠けている異常系のみ。
/// 剛性 0 では追加自由度が零剛性となり全体剛性行列が特異になるため、接合部を
/// 剛（`γ ≈ 0`）とみなせる十分大きな値へ倒す。準備計算のパネル生成
/// （`squid_n_app` の準備計算）は `Ve > 0` を確認した接合部にのみパネルを設ける
/// ため、通常この値は使われない。
const PANEL_RIGID_STIFFNESS: f64 = 1.0e14;

/// 部材軸の鉛直成分がこの値以上なら柱（鉛直材）とみなす。
const COLUMN_EZ: f64 = 0.8;
/// 部材軸の鉛直成分がこの値以下なら梁（水平材）とみなす。
const BEAM_EZ: f64 = 0.2;

/// パネルの降伏モーメント `pMy` の軸力比 `n` を追従するための柱の情報。
///
/// パネル要素は自身の 2 自由度に加えて柱の 12 自由度を自由度写像へ含め、
/// 剛性 0 のまま変位だけを受け取る（`ElementBehavior::update_state` は
/// `global_dofs` と同じ並びの増分を受け取るため、他要素の状態を参照せずに
/// 柱の軸力を追える）。剛性寄与は 0 のため、全体剛性行列には一切影響しない。
///
/// 軸力は材端集中ばねの N-M 相関（`ConcentratedSpringBeam::current_axial_force`）と
/// 同じく、蓄積した節点変位から弾性軸剛性 `EA/L` で評価する近似とする。
/// 節点の並進変位をそのまま用いるため、パネル分オフセットに伴う項
/// （`[Btp]{S}`）は考慮しない（パネル寸法 × せん断変形角のオーダーであり、
/// 階高スケールの軸変形に対して無視できる）。
#[derive(Clone, Debug)]
struct ColumnAxial {
    /// 柱の 2 節点（i 端・j 端）。
    nodes: [NodeId; 2],
    /// 弾性軸剛性 `E・A/L` [N/mm]。
    ea_over_l: f64,
    /// 全体系での柱の単位軸ベクトル。
    axis: [f64; 3],
    /// 軸力比の分母 `Fy・A` [N]。
    n_ref: f64,
    committed: [f64; 12],
    trial: [f64; 12],
}

impl ColumnAxial {
    /// 現在の軸力比 `n`（圧縮を正、引張は 0、1.0 でクランプ）。
    fn axial_ratio(&self) -> f64 {
        let mut d = 0.0;
        for k in 0..3 {
            d += (self.trial[6 + k] - self.trial[k]) * self.axis[k];
        }
        // N は引張正。圧縮側のみ耐力低減に効く（検定側と同じ規約）。
        let n = self.ea_over_l * d;
        ((-n).max(0.0) / self.n_ref).clamp(0.0, 1.0)
    }
}

/// 仕口パネル要素。
pub struct PanelZone {
    /// 接合部の節点（追加自由度 `γX`・`γY` の持ち主）。
    pub node: NodeId,
    /// 柱せい方向のパネル寸法 `dc` [mm]。
    pub dc: f64,
    /// 梁フランジ板厚中心間距離 `db` [mm]。
    pub db: f64,
    /// パネル板厚 `tp` [mm]。
    pub tp: f64,
    /// パネルの実効体積 `Ve` [mm³]。
    pub ve: f64,
    /// パネルの形状係数 κ。
    pub kappa: f64,
    /// せん断弾性係数 `G` [N/mm²]。
    pub g: f64,
    /// パネルの基準強度 `F` [N/mm²]。
    pub fy: f64,
    /// パネルせん断剛性 `Kxp = Kyp = G・Ve` [N·mm/rad]。
    pub k_panel: f64,
    /// パネル部材座標系の回転角 θ [rad]（基準座標系 X-Y 平面内。現状は 0 固定）。
    pub theta: f64,
    /// 軸力比 `n = 0` における降伏モーメント `pMy0 = (Ve/κ)・Fy/√3` [N·mm]。
    pub pmy0: f64,
    /// 弾塑性ばね（`[γ'x, γ'y]` の 2 成分）。`None` は弾性（線形解析）。
    springs: Option<[Box<dyn UniaxialMaterial>; 2]>,
    /// 軸力比 `n` の追従に用いる柱。`None` は `n = 0` 固定。
    column: Option<ColumnAxial>,
    /// 確定・トライアルのせん断変形角 `{γX, γY}`（基準座標系）。
    committed_disp: [f64; 2],
    trial_disp: [f64; 2],
}

/// パネルに取り付く柱・梁から解決した諸元。
struct ResolvedPanel {
    geom: PanelGeometry,
    db: f64,
    g: f64,
    fy: f64,
    column: Option<ColumnAxial>,
}

/// 要素 `e` の単位軸ベクトルと材長を返す（2 節点未満・長さ 0 は `None`）。
fn axis_and_length(model: &Model, e: &ElementData) -> Option<([f64; 3], f64)> {
    if e.nodes.len() < 2 {
        return None;
    }
    let p0 = model.nodes.get(e.nodes[0].index())?.coord;
    let p1 = model.nodes.get(e.nodes[1].index())?.coord;
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if l < 1e-12 {
        return None;
    }
    Some(([d[0] / l, d[1] / l, d[2] / l], l))
}

/// 接合部節点 `node` に取り付く柱・梁からパネル諸元を解決する。
///
/// - 柱: 鉛直材（`|ez| ≥ 0.8`）のうち、モデル化対象の断面
///   （H 形鋼・角形鋼管・円形鋼管。CFT は対象外）を持つ最初のもの。パネル寸法 `dc`・`tp`、
///   せん断弾性係数 `G`、基準強度 `F`、軸力比の基準軸力をこの柱から取る。
/// - 梁: 水平材（`|ez| ≤ 0.2`）のうち最大の `db`（フランジ板厚中心間距離）。
///
/// 柱の `F` 値は鋼種名の前方一致（板厚 40mm 区分）で解決し、解決できない場合は
/// 材料の `fy`、それも無ければ 235 とする（S 造パネルゾーン検定と同じ規則）。
fn resolve(model: &Model, node: NodeId) -> Option<ResolvedPanel> {
    let mut column: Option<(&ElementData, PanelGeometry, [f64; 3], f64)> = None;
    let mut db = 0.0_f64;

    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Beam) || !e.nodes.contains(&node) {
            continue;
        }
        let Some((axis, length)) = axis_and_length(model, e) else {
            continue;
        };
        let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let ez = axis[2].abs();
        if ez >= COLUMN_EZ {
            if column.is_none() {
                // CFT はモデル化の対象外（`PanelGeometry::is_modeling_target`）。
                if let Some(geom) =
                    PanelGeometry::from_column(sec).filter(PanelGeometry::is_modeling_target)
                {
                    column = Some((e, geom, axis, length));
                }
            }
        } else if ez <= BEAM_EZ {
            db = db.max(beam_panel_depth(sec));
        }
    }

    let (col_elem, geom, axis, length) = column?;
    let mat = col_elem
        .material
        .and_then(|mid| model.materials.get(mid.index()))?;
    let sec = col_elem
        .section
        .and_then(|sid| model.sections.get(sid.index()))?;

    let fy = squid_n_core::material_grade::steel_f_value_prefix(&mat.name, 40.0)
        .or(mat.fy)
        .unwrap_or(235.0);

    let column = (col_elem.nodes.len() >= 2).then(|| ColumnAxial {
        nodes: [col_elem.nodes[0], col_elem.nodes[1]],
        ea_over_l: mat.young * sec.area / length.max(1.0),
        axis,
        n_ref: (fy * sec.area).max(1.0),
        committed: [0.0; 12],
        trial: [0.0; 12],
    });

    Some(ResolvedPanel {
        geom,
        db,
        g: mat.shear_modulus(),
        fy,
        column,
    })
}

impl PanelZone {
    /// 線形解析用の弾性パネルを生成する（追加自由度 2 個のみを持つ）。
    pub fn new(data: &ElementData, model: &Model) -> Self {
        Self::build(data, model, false)
    }

    /// 増分解析・時刻歴応答解析用の弾塑性パネルを生成する。
    ///
    /// 降伏点 `pMy` は軸力比 `n` により各ステップで更新されるため、柱の自由度を
    /// 自由度写像へ含める（剛性寄与は 0）。
    pub fn new_nonlinear(data: &ElementData, model: &Model) -> Self {
        Self::build(data, model, true)
    }

    fn build(data: &ElementData, model: &Model, nonlinear: bool) -> Self {
        let node = data.nodes.first().copied().unwrap_or(NodeId(0));
        let resolved = resolve(model, node);

        let (dc, tp, db, ve, kappa, g, fy, column) = match &resolved {
            Some(r) => (
                r.geom.dc,
                r.geom.tp,
                r.db,
                r.geom.effective_volume(r.db),
                r.geom.kappa(),
                r.g,
                r.fy,
                r.column.clone(),
            ),
            None => (0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 235.0, None),
        };

        let k_elastic = g * ve;
        let k_panel = if k_elastic.is_finite() && k_elastic > 0.0 {
            k_elastic
        } else {
            PANEL_RIGID_STIFFNESS
        };
        // pMy0 = (Ve/κ)・Fy/√3（軸力比 n = 0 のときの降伏モーメント）。
        let pmy0 = if kappa > 0.0 {
            (ve / kappa) * fy / 3.0_f64.sqrt()
        } else {
            0.0
        };

        let springs = (nonlinear && pmy0 > 0.0).then(|| {
            let mk = || -> Box<dyn UniaxialMaterial> {
                Box::new(Bilinear::new(k_panel, pmy0, PANEL_HARDENING))
            };
            [mk(), mk()]
        });

        Self {
            node,
            dc,
            db,
            tp,
            ve,
            kappa,
            g,
            fy,
            k_panel,
            theta: 0.0,
            pmy0,
            springs,
            column: if nonlinear { column } else { None },
            committed_disp: [0.0; 2],
            trial_disp: [0.0; 2],
        }
    }

    /// 座標変換 `[Tp] = [[cosθ, sinθ], [-sinθ, cosθ]]`。
    fn tp_matrix(&self) -> [[f64; 2]; 2] {
        let (s, c) = self.theta.sin_cos();
        [[c, s], [-s, c]]
    }

    /// 節点のせん断変形角 `{γX, γY}` からパネル部材座標系の `{γ'x, γ'y}` を返す。
    /// `{γ'} = [[-1, 0], [0, 1]] [Tp] {γ}`。
    fn to_panel_frame(&self, gamma: [f64; 2]) -> [f64; 2] {
        let t = self.tp_matrix();
        let r0 = t[0][0] * gamma[0] + t[0][1] * gamma[1];
        let r1 = t[1][0] * gamma[0] + t[1][1] * gamma[1];
        [-r0, r1]
    }

    /// パネル部材座標系のモーメント `{mxp, myp}` を節点の `{MSX, MSY}` へ戻す。
    /// `{MS} = [Tp]ᵀ [[-1, 0], [0, 1]] {m}`。
    fn to_node_frame(&self, m: [f64; 2]) -> [f64; 2] {
        let t = self.tp_matrix();
        let s = [-m[0], m[1]];
        [
            t[0][0] * s[0] + t[1][0] * s[1],
            t[0][1] * s[0] + t[1][1] * s[1],
        ]
    }

    /// せん断変形角 `{γX, γY}` に対する、パネル部材座標系の接線剛性
    /// `[Kyp, Kxp]`（`{γ'x, γ'y}` 各成分）とモーメント `{mxp, myp}` を返す。
    /// 状態は書き換えない（`probe`）。
    fn response_at(&self, gamma: [f64; 2]) -> ([f64; 2], [f64; 2]) {
        let gp = self.to_panel_frame(gamma);
        match &self.springs {
            Some(sp) => {
                let (m0, k0) = sp[0].probe(gp[0]);
                let (m1, k1) = sp[1].probe(gp[1]);
                ([k0, k1], [m0, m1])
            }
            None => (
                [self.k_panel, self.k_panel],
                [self.k_panel * gp[0], self.k_panel * gp[1]],
            ),
        }
    }

    /// 現在のトライアル状態での接線剛性とモーメント。
    fn panel_response(&self) -> ([f64; 2], [f64; 2]) {
        self.response_at(self.trial_disp)
    }

    /// 節点座標系のパネルせん断モーメント `{MSX, MSY}` [N·mm]。
    ///
    /// 断面検定の設計用パネルモーメント `pM` にそのまま用いる（節点まわりの
    /// モーメント釣り合いが解析上厳密に満たされた値）。
    pub fn panel_moments(&self) -> [f64; 2] {
        let (_, m) = self.panel_response();
        self.to_node_frame(m)
    }

    /// 与えられたせん断変形角に対する節点座標系のパネルせん断モーメント。
    pub fn panel_moments_at(&self, gamma: [f64; 2]) -> [f64; 2] {
        let (_, m) = self.response_at(gamma);
        self.to_node_frame(m)
    }

    /// 現在の軸力比 `n`（追従対象の柱が無ければ 0）。
    pub fn axial_ratio(&self) -> f64 {
        self.column.as_ref().map_or(0.0, ColumnAxial::axial_ratio)
    }

    /// 現在の軸力比における降伏モーメント `pMy = pMy0・√(1−n²)` [N·mm]。
    pub fn yield_moment(&self) -> f64 {
        let n = self.axial_ratio();
        self.pmy0 * (1.0 - n * n).max(0.0).sqrt()
    }

    /// 軸力比の変化を弾塑性ばねの降伏値へ反映する。
    fn apply_axial_interaction(&mut self) {
        if self.springs.is_none() || self.column.is_none() {
            return;
        }
        // 降伏耐力が 0 まで落ちると接線剛性が二次勾配のみになり数値的に不安定な
        // ため、材端集中ばねの N-M 相関と同じく下限を設ける。
        let pmy = self.yield_moment().max(0.02 * self.pmy0);
        if let Some(sp) = self.springs.as_mut() {
            sp[0].set_yield(pmy);
            sp[1].set_yield(pmy);
        }
    }
}

impl ElementBehavior for PanelZone {
    fn n_dof(&self) -> usize {
        // パネルの 2 自由度 ＋（軸力追従する場合）柱の 12 自由度（剛性寄与 0）。
        2 + if self.column.is_some() { 12 } else { 0 }
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        let ni = self.node.index();
        for d in 0..2 {
            gdofs.push(
                dof.panel_dof(ni, d)
                    .map_or(usize::MAX, |active| active as usize),
            );
        }
        if let Some(col) = &self.column {
            for &nid in &col.nodes {
                let cn = nid.index();
                for d in 0..DOF_PER_NODE {
                    let g = cn * DOF_PER_NODE + d;
                    gdofs.push(dof.active(g).map_or(usize::MAX, |a| a as usize));
                }
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        let (k_panel_frame, _) = self.panel_response();
        // K_node = [Tp]ᵀ S K' S [Tp]（S = diag(-1, 1)）。S は対角 ±1 のため
        // S K' S = K' となり、実質 [Tp]ᵀ K' [Tp] に帰着する。
        let t = self.tp_matrix();
        let mut k = LocalMat::zeros(self.n_dof());
        for i in 0..2 {
            for j in 0..2 {
                let mut v = 0.0;
                for p in 0..2 {
                    v += t[p][i] * k_panel_frame[p] * t[p][j];
                }
                k.set(i, j, v);
            }
        }
        k
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let ms = self.panel_moments();
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, self.n_dof()),
        };
        f.data[0] = ms[0];
        f.data[1] = ms[1];
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        for i in 0..2 {
            if let Some(&d) = du.data.get(i) {
                self.trial_disp[i] += d;
            }
        }
        if let Some(col) = self.column.as_mut() {
            for i in 0..12 {
                if let Some(&d) = du.data.get(2 + i) {
                    col.trial[i] += d;
                }
            }
        }
        // 軸力比の変化を降伏値へ反映してから、パネルばねへトライアルを与える。
        self.apply_axial_interaction();
        let gp = self.to_panel_frame(self.trial_disp);
        if let Some(sp) = self.springs.as_mut() {
            sp[0].trial(gp[0]);
            sp[1].trial(gp[1]);
        }
        if commit {
            self.commit_state();
        }
    }

    fn commit_state(&mut self) {
        self.committed_disp = self.trial_disp;
        if let Some(col) = self.column.as_mut() {
            col.committed = col.trial;
        }
        if let Some(sp) = self.springs.as_mut() {
            sp[0].commit();
            sp[1].commit();
        }
    }

    fn revert_state(&mut self) {
        self.trial_disp = self.committed_disp;
        if let Some(col) = self.column.as_mut() {
            col.trial = col.committed;
        }
        if let Some(sp) = self.springs.as_mut() {
            sp[0].revert();
            sp[1].revert();
        }
    }

    fn snapshot_state(&self) -> Box<dyn std::any::Any> {
        let col = self
            .column
            .as_ref()
            .map(|c| (c.committed, c.trial))
            .unwrap_or(([0.0; 12], [0.0; 12]));
        let springs = self
            .springs
            .as_ref()
            .map(|sp| [sp[0].serialize_state(), sp[1].serialize_state()]);
        Box::new((self.committed_disp, self.trial_disp, col, springs))
    }

    fn restore_state(&mut self, state: &dyn std::any::Any) {
        type Snapshot = (
            [f64; 2],
            [f64; 2],
            ([f64; 12], [f64; 12]),
            Option<[Vec<u8>; 2]>,
        );
        if let Some((committed, trial, col, springs)) = state.downcast_ref::<Snapshot>() {
            self.committed_disp = *committed;
            self.trial_disp = *trial;
            if let Some(c) = self.column.as_mut() {
                c.committed = col.0;
                c.trial = col.1;
            }
            if let (Some(sp), Some(data)) = (self.springs.as_mut(), springs.as_ref()) {
                let _ = sp[0].deserialize_state(&data[0]);
                let _ = sp[1].deserialize_state(&data[1]);
            }
        }
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        let col = self
            .column
            .as_ref()
            .map(|c| (c.committed, c.trial))
            .unwrap_or(([0.0; 12], [0.0; 12]));
        let springs = self
            .springs
            .as_ref()
            .map(|sp| [sp[0].serialize_state(), sp[1].serialize_state()]);
        bincode::serialize(&(self.committed_disp, self.trial_disp, col, springs))
            .expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        // 旧チェックポイント（変位未収録・空バイト列）は「状態なし」として許容する。
        if data.is_empty() {
            return Ok(());
        }
        type Checkpoint = (
            [f64; 2],
            [f64; 2],
            ([f64; 12], [f64; 12]),
            Option<[Vec<u8>; 2]>,
        );
        let (committed, trial, col, springs): Checkpoint = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.committed_disp = committed;
        self.trial_disp = trial;
        if let Some(c) = self.column.as_mut() {
            c.committed = col.0;
            c.trial = col.1;
        }
        if let (Some(sp), Some(bytes)) = (self.springs.as_mut(), springs.as_ref()) {
            sp[0].deserialize_state(&bytes[0])?;
            sp[1].deserialize_state(&bytes[1])?;
        }
        Ok(())
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        // パネルは質量を持たない（せん断変形角の自由度に質量は対応しない）。
        // 固有値解析は零質量方向を質量ランク判定で除くため、回転自由度と同じ扱いになる。
        LocalMat::zeros(self.n_dof())
    }

    fn panel_moments_from(&self, u_elem: &[f64]) -> Option<[f64; 2]> {
        // `global_dofs` の並びは [γX, γY, （軸力追従する場合）柱の 12 自由度]。
        let gamma = [
            u_elem.first().copied().unwrap_or(0.0),
            u_elem.get(1).copied().unwrap_or(0.0),
        ];
        Some(self.panel_moments_at(gamma))
    }
}

/// 接合部に取り付く部材の材端応力（フェイスモーメント算定の入力）。
pub struct PanelConnection {
    pub ml_b: f64,
    pub mr_b: f64,
    pub bql: f64,
    pub bqr: f64,
    pub bnl: f64,
    pub bnr: f64,
    pub ml_c: f64,
    pub mu_c: f64,
    pub cql: f64,
    pub cqu: f64,
}

/// フェイスモーメントとパネルせん断の算定結果。
pub struct PanelResult {
    pub b_ml: f64,
    pub b_mr: f64,
    pub c_ml: f64,
    pub c_mu: f64,
    pub pqc: f64,
    pub pqb: f64,
    pub tau: f64,
}

/// 材端応力からフェイスモーメント（柱面・梁面へ換算したモーメント）と
/// パネルせん断・せん断応力度を算定する。
///
/// ```text
/// bml = ml_b − bql・dc/2
/// pQc = ((bml + bmr) − (cql + cqu)・db/2) / db
/// pQb = ((cmu + cml) − (bql + bqr)・dc/2) / dc
/// τ   = pQc / (dc・tp)
/// ```
///
/// 節点のモーメント釣り合い `ml_b + mr_b = ml_c + mu_c` が成立するとき、
/// 整合条件 `pQc・db = pQb・dc` が自動的に満たされる。
pub fn face_moments(dc: f64, db: f64, tp: f64, conn: &PanelConnection) -> PanelResult {
    let dc2 = dc / 2.0;
    let db2 = db / 2.0;

    let b_ml = conn.ml_b - conn.bql * dc2;
    let b_mr = conn.mr_b - conn.bqr * dc2;
    let c_ml = conn.ml_c - conn.cql * db2;
    let c_mu = conn.mu_c - conn.cqu * db2;

    let pqc = ((b_ml + b_mr) - (conn.cql + conn.cqu) * db2) / db;
    let pqb = ((c_mu + c_ml) - (conn.bql + conn.bqr) * dc2) / dc;
    let tau = if tp > 0.0 { pqc / (dc * tp) } else { 0.0 };
    PanelResult {
        b_ml,
        b_mr,
        c_ml,
        c_mu,
        pqc,
        pqb,
        tau,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{EndCondition, ForceRegime, LocalAxis, Material, Node, Section};
    use squid_n_core::section_shape::SectionShape;

    // ── フェイスモーメント（原典: 添付資料『パネルゾーンの力学』小野瀬, 2009） ──

    /// 整合条件 pqc·db = pqb·dc は、節点のモーメント釣り合い
    /// ml_b + mr_b = ml_c + mu_c が成立するとき自動的に満たされる（資料 式(4)）。
    #[test]
    fn test_face_moments_equilibrium_consistency() {
        let (dc, db, tp) = (500.0, 800.0, 19.0);
        let conn = PanelConnection {
            ml_b: 500_000.0,
            mr_b: 300_000.0,
            bql: 150.0,
            bqr: 100.0,
            bnl: 0.0,
            bnr: 0.0,
            ml_c: 400_000.0,
            mu_c: 400_000.0, // 500+300 = 400+400 = 800 ✓
            cql: 120.0,
            cqu: 130.0,
        };
        let res = face_moments(dc, db, tp, &conn);
        assert!((res.pqc * db - res.pqb * dc).abs() < 1e-9);
        assert!((res.tau - res.pqc / (dc * tp)).abs() < 1e-12);
        assert!((res.b_ml - (conn.ml_b - conn.bql * dc / 2.0)).abs() < 1e-9);
        assert!((res.c_mu - (conn.mu_c - conn.cqu * db / 2.0)).abs() < 1e-9);
    }

    /// 資料ケース1 の数値例照合。単位系は資料に合わせ kN, m, kN·m。
    /// 確定値: pQc = 851.135 kN, pQb = 1702.273 kN, τc = τb（整合）。
    #[test]
    fn test_face_moments_reference_case1() {
        let (dc, db, tp) = (0.2_f64, 0.4_f64, 1.0_f64);
        let conn = PanelConnection {
            ml_b: 218.182,
            mr_b: 181.818,
            bql: 72.727,
            bqr: 72.727,
            bnl: 0.0,
            bnr: 0.0,
            ml_c: 150.0,
            mu_c: 250.0,
            cql: 100.0,
            cqu: 125.0,
        };
        let res = face_moments(dc, db, tp, &conn);
        assert!((res.b_ml - 210.909).abs() < 1e-3, "bML={}", res.b_ml);
        assert!((res.b_mr - 174.545).abs() < 1e-3, "bMR={}", res.b_mr);
        assert!((res.c_ml - 130.0).abs() < 1e-9, "cML={}", res.c_ml);
        assert!((res.c_mu - 225.0).abs() < 1e-9, "cMU={}", res.c_mu);
        assert!((res.pqc - 851.135).abs() < 0.05, "pQc={}", res.pqc);
        assert!((res.pqb - 1702.273).abs() < 0.05, "pQb={}", res.pqb);
        let tau_b = res.pqb / (db * tp);
        assert!((res.tau - tau_b).abs() / res.tau.abs() < 1e-4);
    }

    /// 資料ケース2（ト型＝梁が片側のみ）。欠落部材の項を 0 として同一式で計算できる。
    /// 確定値: pQc=854.168 kN, pQb=1708.334 kN。
    #[test]
    fn test_face_moments_reference_case2_t_joint() {
        let (dc, db, tp) = (0.2_f64, 0.4_f64, 1.0_f64);
        let conn = PanelConnection {
            ml_b: 400.0,
            mr_b: 0.0, // 欠落部材 → 0
            bql: 133.333,
            bqr: 0.0,
            bnl: 0.0,
            bnr: 0.0,
            ml_c: 150.0,
            mu_c: 250.0,
            cql: 100.0,
            cqu: 125.0,
        };
        let res = face_moments(dc, db, tp, &conn);
        assert!((res.pqc - 854.168).abs() < 0.05, "pQc={}", res.pqc);
        assert!((res.pqb - 1708.334).abs() < 0.05, "pQb={}", res.pqb);
    }

    /// L 型（右梁・上柱が欠落）・ト型（下柱が欠落）・十字型のいずれでも、
    /// 欠落部材の項を 0 とした同一式で整合条件が満たされる。
    #[test]
    fn test_face_moments_joint_shapes() {
        let (dc, db, tp) = (500.0, 700.0, 12.0);
        let cases = [
            // L 型: 釣り合い ml_b = ml_c
            PanelConnection {
                ml_b: 300_000.0,
                mr_b: 0.0,
                bql: 100.0,
                bqr: 0.0,
                bnl: 0.0,
                bnr: 0.0,
                ml_c: 300_000.0,
                mu_c: 0.0,
                cql: 80.0,
                cqu: 0.0,
            },
            // ト型: 釣り合い ml_b + mr_b = mu_c
            PanelConnection {
                ml_b: 200_000.0,
                mr_b: 100_000.0,
                bql: 80.0,
                bqr: 60.0,
                bnl: 0.0,
                bnr: 0.0,
                ml_c: 0.0,
                mu_c: 300_000.0,
                cql: 0.0,
                cqu: 90.0,
            },
            // 十字型（左右・上下対称）
            PanelConnection {
                ml_b: 450_000.0,
                mr_b: 450_000.0,
                bql: 120.0,
                bqr: 120.0,
                bnl: 0.0,
                bnr: 0.0,
                ml_c: 500_000.0,
                mu_c: 400_000.0,
                cql: 100.0,
                cqu: 100.0,
            },
        ];
        for (i, conn) in cases.iter().enumerate() {
            let res = face_moments(dc, db, tp, conn);
            assert!(res.pqc.is_finite() && res.pqb.is_finite(), "case {i}");
            assert!(
                (res.pqc * db - res.pqb * dc).abs() < 1e-9,
                "case {i}: pqc·db != pqb·dc"
            );
        }
    }

    // ── 仕口パネル要素 ──────────────────────────────────────

    /// 十字型の S 造接合部モデル（柱: H-400×400×13×21、梁: H-600×200×11×17）。
    /// 節点 0 が接合部、1/2 が梁の遠端、3/4 が柱の遠端。
    fn cross_joint_model() -> (Model, ElementData) {
        let node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let section = |id: u32, shape: SectionShape, depth: f64, area: f64| Section {
            id: SectionId(id),
            name: String::new(),
            area,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth,
            width: depth,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: Some(shape),
        };
        let member = |id: u32, n0: u32, n1: u32, sec: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
            section: Some(SectionId(sec)),
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

        let model = Model {
            nodes: vec![
                node(0, [0.0, 0.0, 3000.0]),     // 接合部
                node(1, [-5000.0, 0.0, 3000.0]), // 左梁の遠端
                node(2, [5000.0, 0.0, 3000.0]),  // 右梁の遠端
                node(3, [0.0, 0.0, 0.0]),        // 下柱の遠端
                node(4, [0.0, 0.0, 6000.0]),     // 上柱の遠端
            ],
            sections: vec![
                // 0: 梁 H-600×200×11×17、1: 柱 H-400×400×13×21
                section(
                    0,
                    SectionShape::SteelH {
                        height: 600.0,
                        width: 200.0,
                        web_thick: 11.0,
                        flange_thick: 17.0,
                    },
                    600.0,
                    1.0e4,
                ),
                section(
                    1,
                    SectionShape::SteelH {
                        height: 400.0,
                        width: 400.0,
                        web_thick: 13.0,
                        flange_thick: 21.0,
                    },
                    400.0,
                    2.187e4,
                ),
            ],
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
                member(0, 1, 0, 0), // 左梁
                member(1, 0, 2, 0), // 右梁
                member(2, 3, 0, 1), // 下柱
                member(3, 0, 4, 1), // 上柱
            ],
            ..Default::default()
        };

        let panel = ElementData {
            id: ElemId(10),
            kind: ElementKind::PanelZone,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
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
        };
        (model, panel)
    }

    /// 諸元の解決: dc は柱（鉛直材）から、db は梁（水平材）から取る。
    /// 取り違えると Ve が誤り、剛性・降伏耐力の双方が狂う（回帰テスト）。
    #[test]
    fn test_resolve_takes_dc_from_column_and_db_from_beam() {
        let (model, data) = cross_joint_model();
        let pz = PanelZone::new(&data, &model);
        assert!((pz.dc - (400.0 - 21.0)).abs() < 1e-9, "dc は柱: {}", pz.dc);
        assert!((pz.db - (600.0 - 17.0)).abs() < 1e-9, "db は梁: {}", pz.db);
        assert!((pz.tp - 13.0).abs() < 1e-9, "tp は柱ウェブ厚: {}", pz.tp);
        // H 形柱: Ve = dc·db·tp
        assert!((pz.ve - pz.dc * pz.db * pz.tp).abs() / pz.ve < 1e-12);
    }

    /// 弾性パネルの剛性は Kxp = Kyp = G·Ve で、自由度は 2 個。
    #[test]
    fn test_elastic_stiffness_is_g_times_ve() {
        let (model, data) = cross_joint_model();
        let pz = PanelZone::new(&data, &model);
        assert_eq!(pz.n_dof(), 2, "弾性パネルは 2 自由度");

        let k = pz.tangent_stiffness(&ElemState::default(), &Ctx { model: &model });
        let expected = pz.g * pz.ve;
        assert!((k.get(0, 0) - expected).abs() / expected < 1e-12);
        assert!((k.get(1, 1) - expected).abs() / expected < 1e-12);
        assert!(k.get(0, 1).abs() < 1e-6, "θ=0 では非対角は 0");
    }

    /// 内力は M = K·γ（弾性）。トライアル変位に追従する。
    #[test]
    fn test_internal_force_follows_trial_rotation() {
        let (model, data) = cross_joint_model();
        let ctx = Ctx { model: &model };
        let mut pz = PanelZone::new(&data, &model);
        let k = pz.g * pz.ve;

        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, 2),
        };
        du.data[0] = 1.0e-4;
        pz.update_state(&du, false, &ctx);

        let f = pz.internal_force(&ElemState::default(), &ctx);
        assert!((f.data[0] - k * 1.0e-4).abs() / (k * 1.0e-4) < 1e-12);
        assert!(f.data[1].abs() < 1e-6);
        // panel_moments() は内力と同じ値（検定の pM に供給する）。
        assert!((pz.panel_moments()[0] - f.data[0]).abs() < 1e-6);
    }

    /// Kxp = Kyp のとき、節点座標系のパネル剛性は θ に依らず diag(K, K) になる
    /// （資料 (2.10.1-2)(2.10.1-3) の [Tp] が直交変換であるため）。
    #[test]
    fn test_stiffness_is_invariant_to_theta() {
        let (model, data) = cross_joint_model();
        let ctx = Ctx { model: &model };
        let mut pz = PanelZone::new(&data, &model);
        pz.theta = 0.37; // 任意角
        let k = pz.tangent_stiffness(&ElemState::default(), &ctx);
        let expected = pz.g * pz.ve;
        assert!((k.get(0, 0) - expected).abs() / expected < 1e-12);
        assert!((k.get(1, 1) - expected).abs() / expected < 1e-12);
        assert!(k.get(0, 1).abs() / expected < 1e-12);
        assert!(k.get(1, 0).abs() / expected < 1e-12);
    }

    /// 弾塑性パネル: 降伏点は pMy0 = (Ve/κ)·Fy/√3。降伏後の接線剛性は
    /// 二次勾配比 PANEL_HARDENING 倍になる。
    #[test]
    fn test_nonlinear_panel_yields_at_pmy() {
        let (model, data) = cross_joint_model();
        let ctx = Ctx { model: &model };
        let mut pz = PanelZone::new_nonlinear(&data, &model);

        let expected_pmy0 = (pz.ve / pz.kappa) * pz.fy / 3.0_f64.sqrt();
        assert!((pz.pmy0 - expected_pmy0).abs() / expected_pmy0 < 1e-12);

        // 降伏変形角 γy = pMy0 / (G·Ve)
        let gamma_y = pz.pmy0 / (pz.g * pz.ve);

        // 降伏直前は弾性剛性
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, pz.n_dof()),
        };
        du.data[0] = gamma_y * 0.5;
        pz.update_state(&du, true, &ctx);
        let k = pz.tangent_stiffness(&ElemState::default(), &ctx);
        assert!((k.get(0, 0) - pz.g * pz.ve).abs() / (pz.g * pz.ve) < 1e-9);

        // 降伏後は二次勾配。接線剛性は確定状態からの載荷方向で評価されるため、
        // 確定させずに（commit = false）降伏を超える増分を与えた状態で確認する
        // （確定点そのものでは増分 0 ＝除荷剛性が返るのが弾塑性材の規約）。
        let mut du2 = LocalVec {
            data: SmallVec::from_elem(0.0, pz.n_dof()),
        };
        du2.data[0] = gamma_y * 2.0;
        pz.update_state(&du2, false, &ctx);
        let k2 = pz.tangent_stiffness(&ElemState::default(), &ctx);
        let k_expected = PANEL_HARDENING * pz.g * pz.ve;
        assert!(
            (k2.get(0, 0) - k_expected).abs() / k_expected < 1e-9,
            "降伏後の接線剛性 {} は {} のはず",
            k2.get(0, 0),
            k_expected
        );
        // モーメントは pMy をわずかに超える程度（二次勾配分）で頭打ちになる。
        let m = pz.panel_moments()[0];
        assert!(
            m > pz.pmy0 && m < 1.05 * pz.pmy0,
            "M={} pMy0={}",
            m,
            pz.pmy0
        );
    }

    /// 弾塑性パネルは軸力比の追従のため柱の 12 自由度を自由度写像へ含めるが、
    /// 剛性寄与は 0（全体剛性行列に一切影響しない）。
    #[test]
    fn test_nonlinear_panel_tracks_column_without_stiffness() {
        let (model, data) = cross_joint_model();
        let ctx = Ctx { model: &model };
        let pz = PanelZone::new_nonlinear(&data, &model);
        assert_eq!(pz.n_dof(), 14, "パネル 2 ＋ 柱 12");

        let k = pz.tangent_stiffness(&ElemState::default(), &ctx);
        for i in 0..14 {
            for j in 0..14 {
                if i < 2 && j < 2 {
                    continue;
                }
                assert_eq!(
                    k.get(i, j),
                    0.0,
                    "柱自由度へ剛性を与えてはならない ({i},{j})"
                );
            }
        }
    }

    /// 軸力比 n の増加で降伏モーメントが pMy0·√(1−n²) へ低下する。
    #[test]
    fn test_yield_moment_reduces_with_axial_ratio() {
        let (model, data) = cross_joint_model();
        let ctx = Ctx { model: &model };
        let mut pz = PanelZone::new_nonlinear(&data, &model);
        assert!(
            (pz.yield_moment() - pz.pmy0).abs() / pz.pmy0 < 1e-12,
            "初期 n=0"
        );

        // 柱（節点 3 → 0、鉛直上向き）を圧縮する変位を与える。
        // 自由度並びは [γX, γY, 柱 i 端 6, 柱 j 端 6]。i 端＝節点 3、j 端＝節点 0。
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, pz.n_dof()),
        };
        // j 端（上側）を下げる＝軸方向に縮む → 圧縮
        du.data[2 + 6 + 2] = -1.0;
        pz.update_state(&du, true, &ctx);

        let n = pz.axial_ratio();
        assert!(n > 0.0, "圧縮軸力で n > 0 になるはず: n={n}");
        let expected = pz.pmy0 * (1.0 - n * n).sqrt();
        assert!(
            (pz.yield_moment() - expected).abs() / pz.pmy0 < 1e-12,
            "pMy={} 期待={}",
            pz.yield_moment(),
            expected
        );
        assert!(pz.yield_moment() < pz.pmy0, "軸力で降伏耐力が下がる");
    }

    /// 引張軸力では降伏耐力を低減しない（検定側と同じ規約）。
    #[test]
    fn test_tension_does_not_reduce_yield_moment() {
        let (model, data) = cross_joint_model();
        let ctx = Ctx { model: &model };
        let mut pz = PanelZone::new_nonlinear(&data, &model);
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, pz.n_dof()),
        };
        du.data[2 + 6 + 2] = 1.0; // j 端を上げる＝伸び → 引張
        pz.update_state(&du, true, &ctx);
        assert_eq!(pz.axial_ratio(), 0.0);
        assert!((pz.yield_moment() - pz.pmy0).abs() / pz.pmy0 < 1e-12);
    }

    /// 諸元を解決できない接合部（柱が RC など）は剛パネルへ倒し、
    /// 追加自由度が零剛性になって全体剛性行列が特異になるのを防ぐ。
    #[test]
    fn test_unresolvable_panel_falls_back_to_rigid() {
        let (mut model, data) = cross_joint_model();
        // 柱の断面形状を消してパネル諸元を解決できなくする。
        model.sections[1].shape = None;
        let pz = PanelZone::new(&data, &model);
        assert_eq!(pz.k_panel, PANEL_RIGID_STIFFNESS);
        let k = pz.tangent_stiffness(&ElemState::default(), &Ctx { model: &model });
        assert!(k.get(0, 0) > 0.0 && k.get(1, 1) > 0.0);
    }
}
