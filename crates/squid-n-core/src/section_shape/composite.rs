//! SRC/CFT の複合換算断面性能。
//!
//! - [`CompositeProps`] — 複合換算後の断面性能
//! - [`SectionShape::src_equivalent_props`] — SRC の等価断面性能
//! - [`SectionShape::cft_equivalent_props`] — CFT の等価断面性能

use super::constants::{E_STEEL, KAPPA_RC, NU_CONCRETE, NU_STEEL};
use super::geometry::{h_web_shear_area, rect_torsion_j};
use super::material::concrete_young_modulus;
use super::types::SectionShape;

/// SRC/CFT の複合換算断面性能（要素剛性用。各種合成構造設計指針）。
///
/// いずれも要素に割り当てた材料（SRC はコンクリート、CFT は鋼管）を基準とした
/// 等価値。質量算定用の断面積（`calc_area`）とは区別して用いること。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompositeProps {
    /// 軸剛性用断面積 [mm²]
    pub area_ax: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
    pub as_y: f64,
    pub as_z: f64,
}

impl SectionShape {
    /// SRC 断面の等価断面性能を、実際のヤング係数比 ns=Es/Ec から算定する
    /// （各種合成構造設計指針: An=rcAn+sAn·(ns−1)、Ie=rcIe+sIe·(ns−1)、
    /// As=rcAs+sAs·(ngs−1)、J=cJ+(sG/cG)·sJ）。
    ///
    /// `ec`/`nu_c`: 要素材料（コンクリート）のヤング係数・ポアソン比。
    /// 鉄骨は Es=`E_STEEL`・νs=0.3 とする。ngs=ns·(1+νc)/(1+νs)。
    /// SrcRect 以外、または ec≤0 では None（呼び出し側は `to_section` の
    /// 既定値=N_S_EQ 固定へフォールバックする）。
    pub fn src_equivalent_props(&self, ec: f64, nu_c: f64) -> Option<CompositeProps> {
        let SectionShape::SrcRect {
            b,
            d,
            steel_height: sh,
            steel_width: sw,
            steel_web_thick: tw,
            steel_flange_thick: tf,
            ..
        } = *self
        else {
            return None;
        };
        if ec <= 0.0 {
            return None;
        }
        let ns = E_STEEL / ec;
        let ngs = ns * (1.0 + nu_c) / (1.0 + NU_STEEL);

        let s_a = 2.0 * sw * tf + (sh - 2.0 * tf) * tw;
        let hw = sh - 2.0 * tf;
        let s_iy = (sw * sh.powi(3) - (sw - tw) * hw.powi(3)) / 12.0;
        let s_iz = (2.0 * tf * sw.powi(3) + hw * tw.powi(3)) / 12.0;
        let s_j = (2.0 * sw * tf.powi(3) + hw * tw.powi(3)) / 3.0;

        let rc_as = b * d / KAPPA_RC;
        Some(CompositeProps {
            area_ax: b * d + (ns - 1.0) * s_a,
            iy: b * d.powi(3) / 12.0 + (ns - 1.0) * s_iy,
            iz: d * b.powi(3) / 12.0 + (ns - 1.0) * s_iz,
            j: rect_torsion_j(b, d) + ngs * s_j,
            as_y: rc_as + (ngs - 1.0) * 2.0 * sw * tf,
            as_z: rc_as + (ngs - 1.0) * h_web_shear_area(sh, tw),
        })
    }

    /// CFT 断面（CftBox/CftPipe）の等価断面性能を鋼管基準で算定する
    /// （各種合成構造設計指針: SRC 柱に準じる累加（CFT もこれに準じる）を鋼基準の
    /// 1/n 換算で適用。J は S 柱の J=(sG/cG)·sJ+cJ を鋼基準 J=sJ+cJ/ngs に換算）。
    ///
    /// `es`/`nu_s`: 要素材料（鋼管）のヤング係数・ポアソン比、
    /// `fc`: 充填コンクリート強度（`Material.fc`）。
    /// Ec は `concrete_young_modulus`（γ=23）・νc=0.2 とする。
    /// CftBox/CftPipe 以外、または Ec≤0 では None（鋼管のみの既定値へ
    /// フォールバック）。
    pub fn cft_equivalent_props(&self, es: f64, nu_s: f64, fc: f64) -> Option<CompositeProps> {
        let ec = concrete_young_modulus(fc);
        if ec <= 0.0 || es <= 0.0 {
            return None;
        }
        let core = self.cft_core_props()?;
        // 内法が消える（板厚が過大な）断面は充填コンクリートを累加できないため、
        // 鋼管のみの既定値へフォールバックする。
        if core.area <= 0.0 {
            return None;
        }
        let n = es / ec;
        let ngs = n * (1.0 + NU_CONCRETE) / (1.0 + nu_s);
        // 鋼管のせん断有効断面積: 角形は加力方向に平行な 2 枚の板、
        // 円形は全断面の 1/2（薄肉円管の慣用）。
        let (s_as_y, s_as_z) = match *self {
            SectionShape::CftBox { thick: t, .. } => {
                (2.0 * t * core.inner_width, 2.0 * t * core.inner_height)
            }
            SectionShape::CftPipe { .. } => {
                let a = self.calc_area() / 2.0;
                (a, a)
            }
            // `cft_core_props` が Some を返すのは CFT 断面のみ。
            _ => unreachable!("cft_core_props が Some を返した非 CFT 断面"),
        };
        Some(CompositeProps {
            area_ax: self.calc_area() + core.area / n,
            iy: self.calc_iy() + core.iy / n,
            iz: self.calc_iz() + core.iz / n,
            j: self.calc_j() + core.j / ngs,
            as_y: s_as_y + core.area / KAPPA_RC / ngs,
            as_z: s_as_z + core.area / KAPPA_RC / ngs,
        })
    }
}

/// CFT 断面の**充填コンクリート部分**（鋼管の内法で囲まれる部分）の諸元。
///
/// 剛性側（[`SectionShape::cft_equivalent_props`]。等価断面性能の累加）と
/// 耐力側（`squid-n-design-jp` の CFT 軸終局検定）が同じ値を使うために切り出す。
/// 片方だけ式を直せば静かに食い違うため、算定はここ 1 か所に置く。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CftCoreProps {
    /// 内法幅 [mm]（角形は `幅 − 2t`、円形は内法径 `外径 − 2t`）。
    pub inner_width: f64,
    /// 内法せい [mm]（角形は `せい − 2t`、円形は内法径）。
    pub inner_height: f64,
    /// 充填コンクリートの断面積 [mm²]。
    pub area: f64,
    /// 強軸（せい方向まわり）の断面二次モーメント [mm⁴]。
    pub iy: f64,
    /// 弱軸（幅方向まわり）の断面二次モーメント [mm⁴]。円形は `iy` と同値。
    pub iz: f64,
    /// St.Venant ねじり定数 [mm⁴]。
    pub j: f64,
}

impl SectionShape {
    /// CFT 断面（`CftBox`/`CftPipe`）の充填コンクリート部分の諸元。
    /// CFT 以外の形状は `None`。
    ///
    /// 内法寸法は 0 で下限クランプするため、板厚が過大で内法が消える断面では
    /// `area` が 0 になる（＝充填コンクリートが効かない）。呼び出し側が
    /// 「鋼管のみへフォールバックする」か「充填ゼロのまま続行する」かを選べるよう、
    /// ここでは `None` にせず 0 を返す。
    pub fn cft_core_props(&self) -> Option<CftCoreProps> {
        match *self {
            SectionShape::CftBox {
                height: h,
                width: w,
                thick: t,
            } => {
                let bi = (w - 2.0 * t).max(0.0);
                let hi = (h - 2.0 * t).max(0.0);
                Some(CftCoreProps {
                    inner_width: bi,
                    inner_height: hi,
                    area: bi * hi,
                    iy: bi * hi.powi(3) / 12.0,
                    iz: hi * bi.powi(3) / 12.0,
                    j: rect_torsion_j(bi, hi),
                })
            }
            SectionShape::CftPipe { outer_dia, thick } => {
                let di = (outer_dia - 2.0 * thick).max(0.0);
                let i = std::f64::consts::PI * di.powi(4) / 64.0;
                Some(CftCoreProps {
                    inner_width: di,
                    inner_height: di,
                    area: std::f64::consts::PI * di * di / 4.0,
                    iy: i,
                    iz: i,
                    // 中実円のねじり定数は極断面二次モーメント Ip = 2·I。
                    j: std::f64::consts::PI * di.powi(4) / 32.0,
                })
            }
            _ => None,
        }
    }
}
