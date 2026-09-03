//! バネ / 履歴則パラメータ算定。
//!
//! - [`build_fiber`] — ファイバー梁の生成
//! - [`build_flexural_springs`] — 材端曲げバネ（履歴則別・非線形解析用）
//! - [`yield_moment_and_axial`] — 集中バネの My0 と N許容（N-M 相関用）
//! - [`resolve_member_hysteresis`] — 部材の履歴則を解決（UI 表示にも用いる）
//! - [`flexural_yield_moment`] / [`crack_moment`] / [`flexural_alpha_y`] — 骨格の折れ点算定
//! - [`rotational_spring_params`] / [`flexible_length`] / [`is_rc_like_section`] — 補助算定

use squid_n_core::model::{
    default_fiber_concrete_hysteresis, default_member_hysteresis, AnalysisKind, ElementData,
    HysteresisModel, Model,
};
use squid_n_material::uniaxial::{Bilinear, UniaxialMaterial};
use squid_n_material::{HysteresisMaterial, HysteresisRule, SteelBuckling, TsujiYamada};

use super::regime::is_vertical_member;
use super::StrengthBasis;

/// 端部塑性化域モデル（ファイバー要素・MS 要素）の塑性化域長 Lp [mm]。
///
/// `plastic_zone` の指定値、未指定なら断面せいの 0.5 倍（0.5D は既往検討で
/// 標準的に用いられる値）。断面せいが取れない場合は 200mm を仮定する。
/// 要素生成（[`build_fiber`] / [`crate::frame::multi_spring::MultiSpringElement::new`]）と
/// モデル化図の表示で同じ値を用いるため公開する。
/// なお部材長に対する上下限クランプは
/// [`crate::frame::fiber::clamp_plastic_zone`] が担う。
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
    kind: AnalysisKind,
) -> crate::frame::fiber::FiberBeam {
    let lp = plastic_zone_length(data, model);
    crate::frame::fiber::FiberBeam::with_plastic_zone(data, model, lp, basis, kind)
}

/// 部材の曲げ終局（降伏）モーメント My [N·mm]（技術基準解説書の曲げ終局強度）。
/// RC=0.9·at·σy·j（[`squid_n_core::rc_capacity::rc_mu_simple`]）、鉄骨=Zp·σy（全塑性 Mp）、
/// それ以外（複合断面・形状不明）は σy·Z弾性でフォールバックする。
/// 従来の材端バネは σy·Z弾性を用いていたが、規準の曲げ終局強度へ改良する。
fn flexural_yield_moment(data: &ElementData, model: &Model, basis: StrengthBasis) -> f64 {
    let mat = model.element_material(data);
    let rebar_mat = model.element_rebar_material(data);
    squid_n_core::flexural_strength::member_flexural_yield_moment(
        data,
        model,
        squid_n_core::flexural_strength::FlexuralStrengthFactors {
            steel: basis.steel_factor(mat),
            rebar: basis.rebar_factor(rebar_mat),
        },
    )
}

/// 集中バネの降伏モーメント My0 と軸許容耐力 N許容 = σy·A（MN 相関用）。
pub(super) fn yield_moment_and_axial(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
) -> (f64, f64) {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = model.element_material(data);
    // 軸許容耐力の σy も鋼材文脈（集中ばね梁は鋼材の N-M 相関を想定）。
    let fy_sigma = mat.and_then(|m| m.fy).unwrap_or(235.0) * basis.steel_factor(mat);
    let area = sec.map(|s| s.area).unwrap_or(1.0e4);
    (flexural_yield_moment(data, model, basis), fy_sigma * area)
}

/// 部材の可撓長さ [mm]（= 節点間長 − 両端剛域長。剛域控除後が非正なら全長）。
fn flexible_length(data: &ElementData, model: &Model) -> f64 {
    data.rigid_zone
        .flexible_length_from(model.member_length(data))
}

/// 材端曲げバネの初期回転剛性 k_rot [N·mm/rad] と降伏モーメント My [N·mm]。
/// k_rot は可とう長 L'（= L − 剛域長。§6.2.1）基準で評価する。
fn rotational_spring_params(data: &ElementData, model: &Model, basis: StrengthBasis) -> (f64, f64) {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = model.element_material(data);
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

/// 解析種別に応じた部材個別指定のスロットを返す（増分用／時刻歴用）。
/// 時刻歴用スロットが未指定の部材は増分用の指定に従う（`member_hysteresis_th`）。
fn specified_hysteresis(
    data: &ElementData,
    model: &Model,
    kind: AnalysisKind,
) -> Option<HysteresisModel> {
    match kind {
        AnalysisKind::Incremental => model.member_hysteresis(data.id),
        AnalysisKind::TimeHistory => model.member_hysteresis_th(data.id),
    }
}

/// 部材の曲げ履歴則（材端集中バネ用）を解決する（属性 override → 構造種別ごとの
/// 既定表。本実装の既定の非線形特性は各履歴則の原典に基づく）。
/// `HysteresisModel::Auto` は構造種別ごとの既定（RC/SRC/CFT=武田型、S=標準型。
/// 増分・時刻歴共通）へ解決される。UI 表示にも用いる。
pub fn resolve_member_hysteresis(
    data: &ElementData,
    model: &Model,
    kind: AnalysisKind,
) -> HysteresisModel {
    match specified_hysteresis(data, model, kind) {
        Some(r) if r != HysteresisModel::Auto => r,
        _ => default_member_hysteresis(is_rc_like_section(data, model)),
    }
}

/// ファイバー断面・MS 要素のコンクリート除荷則を解決する。
///
/// 部材個別指定のうちコンクリート履歴として解釈できるもの
/// （逆行型・原点指向型・Karsan–Jirsa 型）を採用し、その他
/// （`Auto`、および武田型等の曲げバネ用履歴）は解析種別ごとの既定
/// （増分=逆行型、時刻歴=Karsan–Jirsa 型。
/// [`default_fiber_concrete_hysteresis`]）へフォールバックする。
/// 鋼材・主筋ファイバは常に Menegotto–Pinto（選択対象外）。UI 表示にも用いる。
pub fn resolve_fiber_concrete_hysteresis(
    data: &ElementData,
    model: &Model,
    kind: AnalysisKind,
) -> HysteresisModel {
    match specified_hysteresis(data, model, kind) {
        Some(
            r @ (HysteresisModel::Retrograde
            | HysteresisModel::OriginOriented
            | HysteresisModel::KarsanJirsa),
        ) => r,
        _ => default_fiber_concrete_hysteresis(kind),
    }
}

/// 耐震壁の壁柱ファイバのコンクリート除荷則を解決する。
///
/// 個別指定の解釈は [`resolve_fiber_concrete_hysteresis`] と同じだが、既定
/// （`Auto`）の増分解析は**原点指向型**とする（逆行型は不可）。壁はロッキングに
/// 伴う中立軸移動で圧縮縁の除荷が本質的に生じ、逆行型（除荷も包絡線を辿る）では
/// 圧縮ブロックの応力が抜けずに曲げ寄与を過大評価し、面内水平力が終局せん断強度
/// Qu の頭打ちを大きく超えてしまう（危険側）ため。時刻歴の既定は柱・梁と同じ
/// Karsan–Jirsa 型。
pub fn resolve_wall_concrete_hysteresis(
    data: &ElementData,
    model: &Model,
    kind: AnalysisKind,
) -> HysteresisModel {
    match specified_hysteresis(data, model, kind) {
        Some(
            r @ (HysteresisModel::Retrograde
            | HysteresisModel::OriginOriented
            | HysteresisModel::KarsanJirsa),
        ) => r,
        _ => match kind {
            AnalysisKind::Incremental => HysteresisModel::OriginOriented,
            AnalysisKind::TimeHistory => HysteresisModel::KarsanJirsa,
        },
    }
}

/// 耐震壁の面内せん断ばねの履歴則を解決する。
///
/// Q–δ 履歴として解釈できる指定（逆行型・標準型・原点指向型・最大点指向型・
/// 武田型）を採用し、その他（`Auto`、Karsan–Jirsa 型等のコンクリート用指定）は
/// 既定の**最大点指向型**（増分・時刻歴共通）へフォールバックする。
/// 壁の面内せん断はひび割れのスリップ性状が強く、除荷開始剛性が初期剛性のまま
/// の移動硬化型（Masing）ではループの吸収エネルギーを過大評価するため、
/// ピーク指向の復元力特性を既定とする。UI 表示にも用いる。
pub fn resolve_wall_shear_hysteresis(
    data: &ElementData,
    model: &Model,
    kind: AnalysisKind,
) -> HysteresisModel {
    match specified_hysteresis(data, model, kind) {
        Some(
            r @ (HysteresisModel::Retrograde
            | HysteresisModel::Standard
            | HysteresisModel::OriginOriented
            | HysteresisModel::MaxPointOriented
            | HysteresisModel::Takeda),
        ) => r,
        _ => HysteresisModel::MaxPointOriented,
    }
}

/// 材端曲げバネのひび割れモーメント Mc [N·mm]。RC 系は Mc=0.56·√Fc·Ze
/// （Fc [N/mm²]、Ze=断面係数。技術基準解説書 P.621-623）、それ以外は My/3 で
/// 近似する。
fn crack_moment(data: &ElementData, model: &Model, my: f64) -> f64 {
    let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = model.element_material(data);
    let depth = sec.map(|s| s.depth.max(s.width)).unwrap_or(100.0);
    let iz = sec.map(|s| s.iz.max(s.iy)).unwrap_or(1.0e6);
    let ze = if depth > 0.0 { iz / (depth / 2.0) } else { 0.0 };
    match (is_rc_like_section(data, model), mat.and_then(|m| m.fc)) {
        (true, Some(fc)) if fc > 0.0 && ze > 0.0 => {
            // 算定は core に集約。My 比によるクランプは材端バネ固有の扱い
            // （Mc が My を跨いでバネの折れ点が逆転するのを防ぐ）。
            squid_n_core::rc_capacity::rc_crack_moment(fc, ze).clamp(my * 0.1, my * 0.9)
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
    // d_eff は断面検定と同規約（帯筋径・多段配筋を考慮した dt）。
    let d_eff = squid_n_core::rc_rebar_geom::rebar_effective_depth(*d, rebar);
    let ec = model.element_material(data).map(|m| m.young).unwrap_or(0.0);
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
        // Takeda（RC 既定）とその他（Auto、および Karsan–Jirsa 型のような M–θ 履歴
        // でない指定を含む）は武田型トリリニア。
        _ => make_pair(HysteresisRule::Takeda {
            crack: (mc, tc),
            yield_point: (my, ty),
            ultimate: (mu, tu),
            alpha,
        }),
    };
    (a, b, false)
}
