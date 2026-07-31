//! 曲げヒンジの閾値算定と発生追跡（P5 §7.4）。
//!
//! - [`HingeThreshold`] — 部材の曲げひび割れ・降伏モーメント閾値
//! - [`compute_hinge_thresholds`] — 全部材の閾値を算定
//! - [`track_hinges`] — 各ステップのヒンジ発生・レベルを判定し記録

use super::geom::member_end_forces_at_face;
use super::types::{HingeEvent, HingeLevel};
use squid_n_core::material_grade::{
    material_strength_factor_rebar, material_strength_factor_steel,
};
use squid_n_core::model::{ElementData, Model};
use squid_n_core::rc_capacity::{rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{bar_set_area, SectionShape};
use squid_n_core::structure_kind::StructureKind;
use squid_n_element::behavior::{Ctx, ElemState, ElementBehavior};

/// 部材塑性率の終局ヒンジ判定値。降伏後、部材塑性率がこの値以上のヒンジを
/// Ultimate（終局）と分類する（μ<この値は Yield）。塑性率の
/// クライテリアはユーザー設定だが、本実装では既定の終局判定値として 4.0 を用いる
/// （要・原典照合／ユーザー調整余地）。
const ULTIMATE_DUCTILITY: f64 = 4.0;

/// ヒンジ判定のモーメント閾値（実スケルトンの折れ点）。
/// RC はひび割れ Mc=κ·Fc·Ze・降伏 My、鉄骨は全塑性 Mp（Mc=My）。
pub(crate) struct HingeThreshold {
    /// 曲げひび割れモーメント Mc [N·mm]（RC のみ有意。鉄骨は My と同値）。
    pub(crate) mc: f64,
    /// 曲げ降伏モーメント My [N·mm]。
    pub(crate) my: f64,
}

/// 部材の曲げヒンジ閾値（実スケルトン）を算定する。
/// RC: Mc=κ·√Fc·Ze（κ=0.56、技術基準解説書 P.621-623）・My=0.9·at·σy·d（同 P.623）。
/// 鉄骨: Mp=Zp·σy（Mc=My）。
/// 複合断面・形状不明は σy·Ze を降伏とする改良簡易値でフォールバックする。
///
/// 本モジュールは保有水平耐力計算（プッシュオーバー）専用のため、降伏応力
/// σy には無条件で材料強度割増（鋼材=`material_strength_factor_steel`、
/// RC 主筋=`material_strength_factor_rebar`。直接入力係数優先、無ければ
/// 鋼材グレード名判定=1.1/590N級=1.05、主筋=一律1.1）を適用する。
fn member_moment_thresholds(elem: &ElementData, model: &Model) -> HingeThreshold {
    let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
        return HingeThreshold { mc: 0.0, my: 0.0 };
    };
    let mat = elem
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    let depth = sec.depth.max(sec.width);
    let i_gross = sec.iz.max(sec.iy);
    let ze = if depth > 0.0 {
        i_gross / (depth / 2.0)
    } else {
        0.0
    };
    // 降伏応力は部材材料の fy を優先。未設定なら鋼材既定 235 N/mm²（SN400 級）。
    // 保有水平耐力計算のため材料強度割増を乗じる（mat が無ければ係数 1.0）。
    let sigma_y_steel = mat.and_then(|m| m.fy).unwrap_or(235.0)
        * mat.map(material_strength_factor_steel).unwrap_or(1.0);

    // 分岐は構造種別による（`squid_n_core::structure_kind`）。断面形状ではなく
    // 材料の区分で決まるため、H 形のコンクリート部材・矩形断面の鋼部材も
    // 正しい式へ振り分けられる。
    let kind = squid_n_core::structure_kind::structure_kind_of(Some(sec), mat.map(|m| m.category));
    match (&sec.shape, kind) {
        // 配筋を持つ形状は材料の区分に依らず RC の式で評価する。鋼の Mp 式は
        // 配筋を無視した素の断面係数を使うため、RC 断面へ当てると My が桁で
        // 過大になり、増分解析で曲げヒンジが検出されなくなる。
        (
            Some(SectionShape::RcRect { rebar, d, .. }) | Some(SectionShape::RcCircle { rebar, d }),
            _,
        ) => {
            // Fc 未設定（fc=0 → Mc=0 でヒンジが一切検出されない）のモデルは
            // `squid_n_element::factory::ensure_nonlinear_input` が解析前に停止する
            // ため、非線形解析ではこのフォールバックに到達しない。
            let fc = mat.and_then(|m| m.fc).unwrap_or(0.0);
            // 曲げひび割れ Mc = κ·√Fc·Ze（κ=0.56、技術基準解説書 P.621-623）。
            let mc = 0.56 * fc.max(0.0).sqrt() * ze;
            // 曲げ降伏 My = 0.9·at·σy·d（rc_mu_simple）。at は片側引張筋（対称配筋仮定）。
            // σy は**断面（配筋）の主筋材質** → 部材材料の fy の順で解決する。
            // どちらも未設定のモデルは `ensure_nonlinear_input` が解析前に停止するため、
            // 既定値 345 へのフォールバックには到達しない。
            // 保有水平耐力計算のため主筋の材料強度割増を乗じる（せん断補強筋は対象外）。
            let sigma_y_rebar = squid_n_core::material_grade::rebar_yield_strength(
                rebar.main_grade.as_deref(),
                mat,
            )
            .unwrap_or(345.0)
                * mat.map(material_strength_factor_rebar).unwrap_or(1.0);
            let at = bar_set_area(&rebar.main_x) / 2.0;
            let d_eff = (d - rebar.cover - rebar.main_x.dia / 2.0).max(0.0);
            let inp = RcCapacityInput {
                b: 1.0,
                d: *d,
                at,
                d_eff,
                sigma_y: sigma_y_rebar,
                fc: fc.max(1e-9),
                pw: 0.0,
                sigma_wy: 0.0,
                clear_span: 1.0,
                sigma_0: 0.0,
            };
            let my = rc_mu_simple(&inp);
            let my = if my > 0.0 { my } else { sigma_y_rebar * ze };
            HingeThreshold { mc: mc.min(my), my }
        }
        (Some(shape), StructureKind::S) => {
            // 鉄骨: 全塑性モーメント Mp = Zp·σy。ひび割れは無いため Mc=My=Mp。
            // 塑性断面係数を持たない形状は 1.12·Ze で近似する。
            let zp = shape.plastic_modulus_strong().unwrap_or(1.12 * ze);
            let mp = sigma_y_steel * zp;
            HingeThreshold { mc: mp, my: mp }
        }
        _ => {
            // 複合断面(SRC/CFT)・形状不明・配筋を持たない RC 断面:
            // σy·Ze を降伏、コンクリを含むなら κ·Fc·Ze をひび割れとする改良簡易値。
            let my = sigma_y_steel * ze;
            let fc = mat.and_then(|m| m.fc).unwrap_or(0.0);
            let mc = if fc > 0.0 {
                (0.56 * fc.sqrt() * ze).min(my)
            } else {
                my
            };
            HingeThreshold { mc, my }
        }
    }
}

pub(crate) fn compute_hinge_thresholds(model: &Model) -> Vec<HingeThreshold> {
    model
        .elements
        .iter()
        .map(|elem| member_moment_thresholds(elem, model))
        .collect()
}

pub(crate) fn track_hinges(
    model: &Model,
    behaviors: &[Box<dyn ElementBehavior>],
    thresholds: &[HingeThreshold],
    ductility: &[f64],
    step: u32,
    hinges: &mut Vec<HingeEvent>,
) {
    let state = ElemState::default();
    let ctx = Ctx { model };
    for (i, (elem, b)) in model.elements.iter().zip(behaviors).enumerate() {
        let f = b.internal_force(&state, &ctx);
        // 曲げ降伏は**危険断面＝剛域フェイス**で判定する。材端力を局所座標へ回して
        // 剛体アームのモーメントを差し引いた成分（局所 My=4/10・Mz=5/11）を用いる
        // （[`member_end_forces_at_face`]）。節点位置のモーメントはアーム分だけ
        // 大きく、断面耐力 My と直接比較すると剛域を持つ部材のヒンジを過早に検出し、
        // 崩壊荷重を過小評価する。局所成分を使うことで、材軸まわりのねじりを
        // 曲げと取り違えることも無くなる。
        let Some(fl) = member_end_forces_at_face(model, elem, &f.data) else {
            continue;
        };
        let m_i = fl[4].abs().max(fl[5].abs());
        let m_j = fl[10].abs().max(fl[11].abs());
        let m_max = m_i.max(m_j);
        let th = &thresholds[i];
        if th.mc <= 0.0 || m_max < th.mc {
            continue;
        }
        // 塑性率: ファイバー要素はプローブ由来の曲率塑性率、非ファイバー要素は
        // モーメント比（m/My）でフォールバック（従来挙動）。
        let mu = if ductility.get(i).copied().unwrap_or(0.0) > 0.0 {
            ductility[i]
        } else if th.my > 0.0 {
            m_max / th.my
        } else {
            0.0
        };
        // ヒンジは**材端ごとに**記録する。1 部材につき最大モーメント側の 1 端しか
        // 記録しないと、両端が降伏した部材でも崩壊機構の運動学的ゲート
        // （形成降伏ヒンジ数 ≧ r+1、`determine_mechanism`）で 1 個としか数えられず、
        // 柱両端ヒンジによる機構が成立しても Partial と判定されてしまう。
        for (end_idx, m_end) in [(0usize, m_i), (1usize, m_j)] {
            if m_end < th.mc {
                continue;
            }
            let level = if m_end >= th.my {
                if mu >= ULTIMATE_DUCTILITY {
                    HingeLevel::Ultimate
                } else {
                    HingeLevel::Yield
                }
            } else {
                HingeLevel::Crack
            };
            hinges.push(HingeEvent {
                step,
                elem: elem.id,
                pos: end_idx as f64,
                level,
                ductility: mu,
            });
        }
    }
}
