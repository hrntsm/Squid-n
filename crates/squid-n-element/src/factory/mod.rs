//! 要素データから振る舞い（[`ElementBehavior`]）を生成するディスパッチャ。
//!
//! 責務ごとにサブモジュールへ分割している:
//!
//! - [`regime`] —       フォースレジーム判定
//! - [`wall_opening`] — 壁開口低減率
//! - [`springs`] —      バネ / 履歴則パラメータ算定
//!
//! 本モジュールは要素種別ごとのディスパッチ（[`build_behavior`] /
//! [`build_nonlinear_behavior`]）と、従来のパスを維持する再エクスポートを担う。

use crate::behavior::{ElemState, ElementBehavior};
use squid_n_core::model::{ElementData, ElementKind, Model};

mod regime;
mod springs;
mod wall_opening;

pub use regime::{resolve_force_regime, ResolvedRegime};
pub use springs::resolve_member_hysteresis;
pub(crate) use wall_opening::wall_opening_reduction;

use springs::{
    build_fiber, build_flexural_springs, build_rotational_springs, yield_moment_and_axial,
};

#[cfg(test)]
use springs::{flexural_alpha_y, is_rc_like_section};
#[cfg(test)]
use squid_n_core::model::ForceRegime;

pub fn build_behavior(data: &ElementData, model: &Model) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        ElementKind::Beam => {
            // RC 耐震壁の側柱: 面内方向は両端ピンのためモーメント・せん断を
            // 負担しない（RC規準の耐震壁規定・側柱の断面性能）。該当する柱は
            // 面内曲げ面のみ端部回転を静的縮約した要素へ差し替える。
            if let Some(axis) = crate::side_column::wall_side_column_release(data, model) {
                let elem = crate::beam::BeamElement::new(data, model);
                return (
                    Box::new(crate::side_column::InPlaneReleasedColumn::new(elem, axis)),
                    ElemState::default(),
                );
            }
            // ForceRegime に基づいて要素種別を選択（P5 §5）
            let regime = resolve_force_regime(data, model);
            match regime {
                ResolvedRegime::ConcentratedSpring => {
                    let elem = crate::beam::BeamElement::new(data, model);
                    // 弾性解析（線形）は常に公称値（fy をそのまま用いる）。
                    let (spring_i, spring_j) =
                        build_rotational_springs(data, model, StrengthBasis::Nominal);
                    (
                        Box::new(
                            crate::concentrated::ConcentratedSpringBeam::new_one_component(
                                elem, spring_i, spring_j,
                            ),
                        ),
                        ElemState::default(),
                    )
                }
                ResolvedRegime::Fiber => {
                    // T2: FiberBeam が実装されるまでの暫定 BeamElement
                    let elem = crate::beam::BeamElement::new(data, model);
                    (Box::new(elem), ElemState::default())
                }
            }
        }
        ElementKind::PanelZone => (
            Box::new(crate::panel::PanelZone::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::Shell => (
            Box::new(crate::shell::ShellElement::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::MultiSpring => (
            // 弾性解析（線形）は常に公称値。
            Box::new(crate::multi_spring::MultiSpringElement::new(
                data,
                model,
                StrengthBasis::Nominal,
            )),
            ElemState::default(),
        ),
        // Fiber 要素：将来 FiberBeam が実装されるまでの暫定 BeamElement
        ElementKind::Fiber => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
        // Wall 要素：壁エレメントモデル（壁エレメント置換モデル。壁柱＋両端ピン剛梁の
        // 4 節点 24 自由度要素）。開口低減率 r は要素内部で考慮される。
        // 耐震壁不成立（フレーム内雑壁）の壁は剛性を周辺の柱・梁の断面性能へ
        // 算入する（beam.rs）ため、壁要素自体は質量のみ保持し剛性は実質ゼロ。
        // 4 節点未満・断面/材料未設定などで構築できない場合は従来の
        // 暫定等価梁にフォールバックする（開口低減 r はせん断剛性に乗じる）。
        ElementKind::Wall => {
            let stiffness_scale = if crate::misc_wall::wall_is_seismic(data, model) {
                1.0
            } else {
                1e-9
            };
            match crate::wall_panel::WallPanelElement::try_new_scaled(data, model, stiffness_scale)
            {
                Some(panel) => (Box::new(panel), ElemState::default()),
                None => {
                    let mut elem = crate::beam::BeamElement::new(data, model);
                    // r=0（開口が壁の 64% 以上）でせん断断面積が 0 になると
                    // ティモシェンコの φ 項が ∞×0 で NaN になるため、微小値を下限とする
                    // （このような壁は本来 RC 耐震壁判定でも不成立となる）。
                    let r = wall_opening_reduction(data, model).max(1e-6);
                    elem.as_y *= r;
                    elem.as_z *= r;
                    elem.a *= stiffness_scale;
                    elem.iy *= stiffness_scale;
                    elem.iz *= stiffness_scale;
                    elem.j *= stiffness_scale;
                    (Box::new(elem), ElemState::default())
                }
            }
        }
        // 一般ブレース：KB = E·A/L（材料力学・トラス要素）。引張専用ブレースは
        // 要素側では特別扱いせず、線形応力解析の active-set 反復
        // （squid-n-solver の solve_tension_only_iterative）で圧縮側を無効化して扱う。
        ElementKind::Brace { .. } => (
            Box::new(crate::truss::TrussElement::new(data, model)),
            ElemState::default(),
        ),
        // 節点バネ：構造力学（部材の変形と自由度）。
        // 局所軸ごとに独立な弾性バネ（軸・せん断・曲げ回転。ねじりは既定 0）。
        ElementKind::NodalSpring => (
            Box::new(crate::spring::NodalSpringElement::new(data, model)),
            ElemState::default(),
        ),
        // 免震支承材：各免震部材指針・製品技術資料（Category B）。
        // 水平は非線形せん断（積層ゴム系バイリニア／摩擦ばね）、鉛直は弾性軸。
        ElementKind::Isolator => (
            Box::new(crate::isolator::IsolatorElement::new(data, model)),
            ElemState::default(),
        ),
        // 制振ダンパー（制振部材の力学モデル）。種別で要素を切替える。
        // - マクスウェル（速度依存型）: 静的・線形では Δt=0 で不活性、時刻歴で活性化。
        // - 履歴型バイリニア（鋼材系）: 変位依存の弾塑性軸ばね（静的・動的で作用）。
        ElementKind::Damper => {
            use squid_n_core::model::DamperKind;
            let kind = model.damper_props(data.id).unwrap_or_default().kind;
            let beh: Box<dyn ElementBehavior> = match kind {
                DamperKind::Maxwell => {
                    Box::new(crate::damper::MaxwellDamperElement::new(data, model))
                }
                DamperKind::HystereticBilinear => {
                    Box::new(crate::damper::HystereticDamperElement::new(data, model))
                }
            };
            (beh, ElemState::default())
        }
    }
}

/// 部材耐力算定に用いる材料強度の基準。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum StrengthBasis {
    /// 公称値(fy をそのまま用いる)。時刻歴応答解析など。
    #[default]
    Nominal,
    /// 保有水平耐力計算用の材料強度(鋼材1.1倍/590N級1.05倍/RC主筋1.1倍、直接入力係数優先)。
    MaterialStrength,
}

impl StrengthBasis {
    /// 鋼材文脈（鋼材断面の集中ばね・純鋼材ファイバー等）の材料強度割増係数。
    /// `Nominal` は常に 1.0、`MaterialStrength` は
    /// [`squid_n_core::material_grade::material_strength_factor_steel`] に委譲する
    /// （材料が無い場合は 1.0）。
    pub(crate) fn steel_factor(self, mat: Option<&squid_n_core::model::Material>) -> f64 {
        match self {
            StrengthBasis::Nominal => 1.0,
            StrengthBasis::MaterialStrength => mat
                .map(squid_n_core::material_grade::material_strength_factor_steel)
                .unwrap_or(1.0),
        }
    }

    /// RC 主筋文脈（RC 断面の集中ばね・主筋ファイバー等。せん断補強筋には用いない）の
    /// 材料強度割増係数。`Nominal` は常に 1.0、`MaterialStrength` は
    /// [`squid_n_core::material_grade::material_strength_factor_rebar`] に委譲する
    /// （材料が無い場合は 1.0）。
    pub(crate) fn rebar_factor(self, mat: Option<&squid_n_core::model::Material>) -> f64 {
        match self {
            StrengthBasis::Nominal => 1.0,
            StrengthBasis::MaterialStrength => mat
                .map(squid_n_core::material_grade::material_strength_factor_rebar)
                .unwrap_or(1.0),
        }
    }
}

/// 非線形解析（pushover）用の要素生成。`ForceRegime` に基づき非線形要素を構築する（P5 §5）。
///
/// 線形弾性解析は従来どおり [`build_behavior`]（弾性 `BeamElement`）を使う。両者を分けるのは、
/// `resolve_force_regime` が剛床に乗らない梁も Fiber へ振り分けるため、共通化すると
/// 線形解析の弾性梁まで非線形要素に置き換わってしまうため。
///
/// 注意（既知の制約）: `ConcentratedSpringBeam` は端ばねスケルトン（降伏モーメント）が必要だが、
/// 現状 `Model` に降伏応力／スケルトン供給経路が無いため、軸-曲げ連成を扱う `FiberBeam` に
/// フォールバックしている（P5 §5 の本来意図は集中ばね梁）。また鋼材はファイバ材料が
/// `Bilinear(My=1e20)` で実質弾性のため、真の降伏は `fc` を持つコンクリート断面でのみ生じる。
/// 鋼材の降伏・集中ばね梁の実体化には Model への降伏応力／スケルトン追加が前提（follow-up）。
///
/// 部材耐力算定に用いる材料強度（鋼材 fy・RC 主筋 σy）の基準は `basis` で指定する。
/// 時刻歴応答解析（`dynamic/timehistory/nonlinear.rs`）は [`StrengthBasis::Nominal`]、
/// 保有水平耐力計算（プッシュオーバー）は [`StrengthBasis::MaterialStrength`] を渡す。
pub fn build_nonlinear_behavior(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        // 耐震壁の側柱は面内曲げ面の端部回転を静的縮約する（線形パスと同じ扱い）。
        // これが無いと、一次設計は「側柱＝面内両端ピン」、保有水平耐力は
        // 「側柱＝両端剛接」という別モデルになり、さらに壁パネルが
        // `as_gross = 壁板 + 側柱断面` で側柱分を既に負担しているため、
        // 非線形解析で側柱の面内せん断が二重計上され保有水平耐力を過大評価する
        // （危険側）。
        ElementKind::Beam
            if crate::side_column::wall_side_column_release(data, model).is_some() =>
        {
            let axis = crate::side_column::wall_side_column_release(data, model)
                .expect("guard で Some を確認済み");
            let elem = crate::beam::BeamElement::new(data, model);
            (
                Box::new(crate::side_column::InPlaneReleasedColumn::new(elem, axis)),
                ElemState::default(),
            )
        }
        ElementKind::Beam => match resolve_force_regime(data, model) {
            ResolvedRegime::ConcentratedSpring => {
                let elem = crate::beam::BeamElement::new(data, model);
                // 履歴則を解決（部材個別指定 → 構造種別ごとの既定表。本実装の既定の
                // 非線形特性は各履歴則の原典に基づく）。RC/SRC/CFT 梁は
                // 武田型トリリニア、S 梁は標準型（kinematic バイリニア）を材端バネに用いる。
                let rule = resolve_member_hysteresis(data, model);
                let (spring_i, spring_j, use_mn) = build_flexural_springs(data, model, rule, basis);
                let beam = crate::concentrated::ConcentratedSpringBeam::new_one_component(
                    elem, spring_i, spring_j,
                );
                // 端バネの N-M 相関（M_lim = My0·(1-|N|/N許容)）はバイリニア（標準型）
                // のみ適用（`set_yield` 対応）。武田型等の履歴材料は骨格固定のため対象外。
                let beam = if use_mn {
                    let (my0, n_allow) = yield_moment_and_axial(data, model, basis);
                    beam.with_mn_interaction(my0, n_allow)
                } else {
                    beam
                };
                (Box::new(beam), ElemState::default())
            }
            ResolvedRegime::Fiber => (
                Box::new(build_fiber(data, model, basis)),
                ElemState::default(),
            ),
        },
        ElementKind::Fiber => (
            Box::new(build_fiber(data, model, basis)),
            ElemState::default(),
        ),
        // MS 要素: 端部バネ断面 + 中央弾性の非線形要素（P5.5 §3）
        ElementKind::MultiSpring => (
            Box::new(crate::multi_spring::MultiSpringElement::new(
                data, model, basis,
            )),
            ElemState::default(),
        ),
        // 一般ブレース(弾塑性): E·A/L の弾性トラス要素（材料力学）。引張専用ブレースの
        // 圧縮無効化は線形応力解析の active-set 反復で扱うため、要素側は特別扱いしない。
        ElementKind::Brace { .. } => (
            Box::new(crate::truss::TrussElement::new(data, model)),
            ElemState::default(),
        ),
        // 耐震壁: 面内せん断を終局せん断強度 Qu で頭打ちにする（弾完全塑性）。
        // 従来は弾性のままで、押し込むほど際限なく水平力を負担し崩壊機構が
        // 形成されず、保有水平耐力 Qu を著しく過大評価していた（危険側）。
        // Qu を算定できない壁（Fc 未設定等）は従来どおり弾性とする。
        ElementKind::Wall => {
            let (b, st) = build_behavior(data, model);
            let qu = crate::wall_panel::WallPanelElement::shear_capacity_of(data, model);
            if qu <= 0.0 {
                return (b, st);
            }
            let stiffness_scale = if crate::misc_wall::wall_is_seismic(data, model) {
                1.0
            } else {
                1e-9
            };
            match crate::wall_panel::WallPanelElement::try_new_scaled(data, model, stiffness_scale)
            {
                Some(panel) => (
                    Box::new(panel.with_shear_capacity(qu)),
                    ElemState::default(),
                ),
                None => (b, st),
            }
        }
        // PanelZone / Shell / NodalSpring は現状の挙動（弾性ベース）を踏襲。
        // 節点バネは非線形解析でも常に弾性のまま（スケルトン未対応）。
        _ => build_behavior(data, model),
    }
}

#[cfg(test)]
mod tests;
