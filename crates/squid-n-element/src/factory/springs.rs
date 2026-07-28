//! バネ / 履歴則パラメータ算定。
//!
//! - [`build_fiber`] — ファイバー梁の生成
//! - [`build_flexural_springs`] — 材端曲げバネ（履歴則別・非線形解析用）
//! - [`yield_moment_and_axial`] — 集中バネの My0 と N許容（N-M 相関用）
//! - [`resolve_member_hysteresis`] — 部材の履歴則を解決（UI 表示にも用いる）
//! - [`flexural_yield_moment`] / [`crack_moment`] / [`flexural_alpha_y`] — 骨格の折れ点算定
//! - [`rotational_spring_params`] / [`flexible_length`] / [`is_rc_like_section`] — 補助算定

use squid_n_core::model::{default_member_hysteresis, ElementData, HysteresisModel, Model};
use squid_n_material::uniaxial::{Bilinear, UniaxialMaterial};
use squid_n_material::{HysteresisMaterial, HysteresisRule, SteelBuckling, TsujiYamada};

use super::regime::is_vertical_member;
use super::StrengthBasis;

/// 端部塑性化域モデル（ファイバー要素・MS 要素）の塑性化域長 Lp [mm]。
///
/// `plastic_zone` の指定値、未指定なら断面せいの 0.5 倍（0.5D は既往検討で
/// 標準的に用いられる値）。断面せいが取れない場合は 200mm を仮定する。
/// 要素生成（[`build_fiber`] / [`crate::multi_spring::MultiSpringElement::new`]）と
/// モデル化図の表示で同じ値を用いるため公開する。
/// なお部材長に対する上下限クランプは
/// [`crate::fiber::clamp_plastic_zone`] が担う。
pub fn plastic_zone_length(data: &ElementData, model: &Model) -> f64 {
    let depth = data
        .section
        .and_then(|sid| model.sections.get(sid.index()))
        .map(|s| s.depth)
        .filter(|d| *d > 0.0)
        .unwrap_or(200.0);
    data.plastic_zone.unwrap_or(0.5 * depth)
}

/// ファイバー梁の生成。既定で塑性化域考慮モデル（端部 Lp 区間にファイバー断面、
/// 中央弾性）とし、Lp は [`plastic_zone_length`]（MS 要素と同じ既定）。
pub(super) fn build_fiber(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
) -> crate::fiber::FiberBeam {
    let lp = plastic_zone_length(data, model);
    crate::fiber::FiberBeam::with_plastic_zone(data, model, lp, basis)
}

/// 部材の曲げ終局（降伏）モーメント My [N·mm]（技術基準解説書の曲げ終局強度）。
/// RC=0.9·at·σy·j（[`squid_n_core::rc_capacity::rc_mu_simple`]）、鉄骨=Zp·σy（全塑性 Mp）、
/// それ以外（複合断面・形状不明）は σy·Z弾性でフォールバックする。
/// 従来の材端バネは σy·Z弾性を用いていたが、規準の曲げ終局強度へ改良する。
fn flexural_yield_moment(data: &ElementData, model: &Model, basis: StrengthBasis) -> f64 {
    use squid_n_core::section_shape::SectionShape;
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let depth = sec.map(|s| s.depth.max(s.width)).unwrap_or(100.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    let ze = if depth > 0.0 { iz / (depth / 2.0) } else { 0.0 };
    let fy = mat.and_then(|m| m.fy);
    match sec.and_then(|s| s.shape.as_ref()) {
        Some(SectionShape::RcRect { rebar, d, .. }) | Some(SectionShape::RcCircle { rebar, d }) => {
            // RC 主筋: σy は断面（配筋）の主筋材質 → 部材材料の fy の順で解決する。
            // 保有水平耐力計算では σy に主筋の材料強度係数を乗じる
            // （せん断補強筋は対象外。本分岐は曲げ主筋のみを扱う）。
            let sy = squid_n_core::material_grade::rebar_yield_strength(
                rebar.main_grade.as_deref(),
                mat,
            )
            .unwrap_or(345.0)
                * basis.rebar_factor(mat);
            // Fc 未設定は [`super::ensure_nonlinear_input`] が解析前に停止するため、
            // 非線形解析ではこのフォールバック（0）に到達しない。
            let fc = mat.and_then(|m| m.fc).unwrap_or(0.0);
            let at = squid_n_core::section_shape::bar_set_area(&rebar.main_x) / 2.0;
            let d_eff = (d - rebar.cover - rebar.main_x.dia / 2.0).max(0.0);
            let my = squid_n_core::rc_capacity::rc_mu_simple(
                &squid_n_core::rc_capacity::RcCapacityInput {
                    b: 1.0,
                    d: *d,
                    at,
                    d_eff,
                    sigma_y: sy,
                    fc: fc.max(1e-9),
                    pw: 0.0,
                    sigma_wy: 0.0,
                    clear_span: 1.0,
                    sigma_0: 0.0,
                },
            );
            if my > 0.0 {
                my
            } else {
                sy * ze
            }
        }
        Some(shape) => {
            // 鋼材: 保有水平耐力計算では σy に鋼材の材料強度係数を乗じる。
            let sy = fy.unwrap_or(235.0) * basis.steel_factor(mat);
            match shape.plastic_modulus_strong() {
                Some(zp) => sy * zp,
                None => sy * ze,
            }
        }
        // 複合断面・形状不明: 鋼材文脈として扱う。
        None => fy.unwrap_or(235.0) * basis.steel_factor(mat) * ze,
    }
}

/// 集中バネの降伏モーメント My0 と軸許容耐力 N許容 = σy·A（MN 相関用）。
pub(super) fn yield_moment_and_axial(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
) -> (f64, f64) {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    // 軸許容耐力の σy も鋼材文脈（集中ばね梁は鋼材の N-M 相関を想定）。
    let fy_sigma = mat.and_then(|m| m.fy).unwrap_or(235.0) * basis.steel_factor(mat);
    let area = sec.map(|s| s.area).unwrap_or(1.0e4);
    (flexural_yield_moment(data, model, basis), fy_sigma * area)
}

/// 部材の可撓長さ [mm]（= 節点間長 − 両端剛域長。剛域控除後が非正なら全長）。
fn flexible_length(data: &ElementData, model: &Model) -> f64 {
    let n0 = &model.nodes[data.nodes[0].index()];
    let n1 = &model.nodes[data.nodes[1].index()];
    let l = ((n1.coord[0] - n0.coord[0]).powi(2)
        + (n1.coord[1] - n0.coord[1]).powi(2)
        + (n1.coord[2] - n0.coord[2]).powi(2))
    .sqrt();
    let l_flex = l - data.rigid_zone.length_i - data.rigid_zone.length_j;
    if l_flex > 0.0 {
        l_flex
    } else {
        l
    }
}

/// 材端曲げバネの初期回転剛性 k_rot [N·mm/rad] と降伏モーメント My [N·mm]。
/// k_rot は可とう長 L'（= L − 剛域長。§6.2.1）基準で評価する。
fn rotational_spring_params(data: &ElementData, model: &Model, basis: StrengthBasis) -> (f64, f64) {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let e = mat.map(|m| m.young).unwrap_or(205000.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    // 材端バネの降伏モーメントは規準の曲げ終局強度（RC=0.9·at·σy·d、鉄骨=Zp·σy）を用いる。
    let my = flexural_yield_moment(data, model, basis);

    let l_eff = flexible_length(data, model);
    let k_rot = if l_eff > 0.0 {
        6.0 * e * iz / l_eff
    } else {
        1.0e12
    };
    (k_rot, my)
}

/// 断面形状が RC/SRC/CFT（コンクリート系）か否か（既定履歴則の判定用）。
pub(super) fn is_rc_like_section(data: &ElementData, model: &Model) -> bool {
    data.section
        .and_then(|sid| model.sections.get(sid.index()))
        .and_then(|s| s.shape.as_ref())
        .is_some_and(|s| s.is_concrete_like())
}

/// 部材の履歴則を解決する（属性 override → 構造種別ごとの既定表。本実装の既定の
/// 非線形特性は各履歴則の原典に基づく）。`HysteresisModel::Auto` は
/// 構造種別ごとの既定（RC/SRC/CFT=武田型、S=標準型）へ解決される。UI 表示にも用いる。
pub fn resolve_member_hysteresis(data: &ElementData, model: &Model) -> HysteresisModel {
    match model.member_hysteresis(data.id) {
        Some(r) if r != HysteresisModel::Auto => r,
        _ => default_member_hysteresis(is_rc_like_section(data, model)),
    }
}

/// 材端曲げバネのひび割れモーメント Mc [N·mm]。RC 系は Mc=0.56·√Fc·Ze
/// （Fc [N/mm²]、Ze=断面係数。技術基準解説書 P.621-623）、それ以外は My/3 で
/// 近似する。
fn crack_moment(data: &ElementData, model: &Model, my: f64) -> f64 {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = data
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let depth = sec.map(|s| s.depth.max(s.width)).unwrap_or(100.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    let ze = if depth > 0.0 { iz / (depth / 2.0) } else { 0.0 };
    match (is_rc_like_section(data, model), mat.and_then(|m| m.fc)) {
        (true, Some(fc)) if fc > 0.0 && ze > 0.0 => {
            (0.56 * fc.sqrt() * ze).clamp(my * 0.1, my * 0.9)
        }
        _ => my / 3.0,
    }
}

/// 材端曲げバネの降伏時剛性低下率 αy。
///
/// RC 矩形断面の梁（水平材）は菅野式
/// （[`squid_n_core::rc_capacity::rc_alpha_y_sugano`]、梅村魁『鉄筋コンクリート
/// 建物の動的耐震設計法』P.106-108）で算定する:
/// - `pt` = at/(b·D)（at=main_x の半分を引張側と仮定）
/// - `a` = 可撓長さ/2（せん断スパン）、`a/D` は式側で [1,5] にクランプ
/// - `d` = 有効せい（D − かぶり − 主筋半径）
/// - `n` = Es/Ec（部材材料のヤング係数を Ec とみなす）
///
/// 柱（鉛直材）は菅野式に軸力項を要するため対象外（柱の既定はファイバー
/// モデルで、本バネ経路に乗る場合は従来既定 0.3）。鉄骨・SRC・CFT・情報不足も
/// 従来既定 0.3 を用いる。
pub(super) fn flexural_alpha_y(data: &ElementData, model: &Model) -> f64 {
    use squid_n_core::section_shape::SectionShape;
    const DEFAULT_ALPHA_Y: f64 = 0.3;
    if is_vertical_member(data, model) {
        return DEFAULT_ALPHA_Y;
    }
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let Some(SectionShape::RcRect { b, d, rebar }) = sec.and_then(|s| s.shape.as_ref()) else {
        return DEFAULT_ALPHA_Y;
    };
    if *b <= 0.0 || *d <= 0.0 {
        return DEFAULT_ALPHA_Y;
    }
    let at = squid_n_core::section_shape::bar_set_area(&rebar.main_x) / 2.0;
    let pt = at / (b * d);
    let d_eff = (d - rebar.cover - rebar.main_x.dia / 2.0).max(0.0);
    let ec = data
        .material
        .and_then(|mid| model.materials.get(mid.index()))
        .map(|m| m.young)
        .unwrap_or(0.0);
    let n = if ec > 0.0 {
        squid_n_core::section_shape::E_STEEL / ec
    } else {
        15.0
    };
    let a = flexible_length(data, model) / 2.0;
    let ay = squid_n_core::rc_capacity::rc_alpha_y_sugano(pt, a / d, d_eff / d, n);
    if ay.is_finite() && ay > 1e-6 {
        ay.min(1.0)
    } else {
        DEFAULT_ALPHA_Y
    }
}

/// 材端曲げバネの復元力材料を履歴則に応じて構築する（各履歴則の原典）。
/// 戻り値の bool は N-M 相関（`set_yield`）を適用可能か（バイリニアのみ true）。
/// 標準型・降伏モーメント不定は従来の kinematic バイリニアを用い、武田型/逆行型/
/// 原点指向型/最大点指向型は [`HysteresisMaterial`] のトリリニア（原点指向はバイ
/// リニア）を用いる。
pub(super) fn build_flexural_springs(
    data: &ElementData,
    model: &Model,
    rule: HysteresisModel,
    basis: StrengthBasis,
) -> (Box<dyn UniaxialMaterial>, Box<dyn UniaxialMaterial>, bool) {
    let (k_rot, my) = rotational_spring_params(data, model, basis);
    // 標準型・降伏モーメント不定は従来の kinematic バイリニア（＝標準型相当）。
    if my <= 0.0 || k_rot <= 0.0 || rule == HysteresisModel::Standard {
        let my = my.max(1.0);
        return (
            Box::new(Bilinear::new(k_rot, my, 0.01)),
            Box::new(Bilinear::new(k_rot, my, 0.01)),
            true,
        );
    }
    // 辻・山田型（バイリニア＋β 混合硬化）。K2=0.01·k_rot、β=0.5（既定）。
    // set_yield 対応のため N-M 相関を適用可能。
    if rule == HysteresisModel::TsujiYamada {
        let k2 = 0.01 * k_rot;
        let mk = || Box::new(TsujiYamada::new(k_rot, my, k2, 0.5)) as Box<dyn UniaxialMaterial>;
        return (mk(), mk(), true);
    }
    // 座屈考慮型（耐力劣化型＋RO 除荷）。既定 Mu=1.1·My（座屈細長比の精算は今後の課題。
    // 断面の λb・κ・WF が得られる場合は lateral_buckling_mu_ratio で Mu/Mp を算定可）。
    // set_yield 対応（Mu も比率を保持）のため N-M 相関を適用可能。
    if rule == HysteresisModel::SteelBuckling {
        let mk =
            || Box::new(SteelBuckling::with_defaults(k_rot, my, 1.1)) as Box<dyn UniaxialMaterial>;
        return (mk(), mk(), true);
    }
    // トリリニア折れ点: ひび割れ Mc/θc（初期勾配 k_rot）、降伏 My/θy（降伏時剛性
    // 低下率 αy。RC 矩形梁は菅野式、その他は既定 0.3 = [`flexural_alpha_y`]）、
    // 終局 Mu=1.1·My/θu（塑性率 4）。
    let mc = crack_moment(data, model, my);
    let tc = (mc / k_rot).max(1e-9);
    let alpha_y = flexural_alpha_y(data, model);
    let ty = (my / (alpha_y * k_rot)).max(tc * 1.5);
    let mu = 1.1 * my;
    let tu = ty * 4.0;
    let alpha = 0.4;
    let mk =
        |r: HysteresisRule| -> Box<dyn UniaxialMaterial> { Box::new(HysteresisMaterial::new(r)) };
    let make_pair = |r: HysteresisRule| (mk(r.clone()), mk(r));
    let (a, b) = match rule {
        HysteresisModel::Retrograde => make_pair(HysteresisRule::Retrograde {
            crack: (mc, tc),
            yield_point: (my, ty),
            ultimate: (mu, tu),
        }),
        HysteresisModel::OriginOriented => make_pair(HysteresisRule::OriginOriented {
            yield_point: (my, ty),
            ultimate: (mu, tu),
        }),
        HysteresisModel::MaxPointOriented => make_pair(HysteresisRule::MaxPointOriented {
            crack: (mc, tc),
            yield_point: (my, ty),
            ultimate: (mu, tu),
        }),
        // Takeda（RC 既定）とその他は武田型トリリニア。
        _ => make_pair(HysteresisRule::Takeda {
            crack: (mc, tc),
            yield_point: (my, ty),
            ultimate: (mu, tu),
            alpha,
        }),
    };
    (a, b, false)
}
