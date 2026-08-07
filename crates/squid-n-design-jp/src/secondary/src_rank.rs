//! SRC 造の部材種別（保有水平耐力計算の Ds 算定）。
//!
//! - [`src_column_rank`] — SRC 柱の部材種別（技術基準解説書 表 2.6.6-5）
//! - [`src_wall_type`] — SRC 耐震壁の種別（せん断破壊 WC・それ以外 WA）
//! - [`src_column_rank_ratios`] — 判定比 N/N0・sM0/M0 の算定
//!
//! 表 2.6.6-5（SRC 柱の部材種別）:
//!
//! | 破壊モード | N/N0≤0.3 かつ sM0/M0≥0.4 | N/N0≤0.3 かつ sM0/M0<0.4 | N/N0≤0.4 かつ sM0/M0≥0.4 | N/N0≤0.4 かつ sM0/M0<0.4 | N/N0>0.4 |
//! |---|---|---|---|---|---|
//! | 曲げ破壊   | FA | FB | FB | FC | FD |
//! | せん断破壊 | FB | FC | FC | FD | FD |
//!
//! - N: メカニズム時の軸方向力（圧縮正）
//! - N0: SRC 断面の圧縮耐力
//! - sM0: 鉄骨の曲げ耐力
//! - M0: SRC 断面の曲げ耐力（N=0 とした時）

use super::holding_capacity::MemberRank;
use squid_n_core::section_shape::{bar_set_area, SectionShape};

/// SRC 柱の部材種別（表 2.6.6-5）。
///
/// - `n_over_n0`: メカニズム時軸力 N（圧縮正）/ SRC 断面の圧縮耐力 N0。
/// - `smo_over_m0`: 鉄骨の曲げ耐力 sM0 / SRC 断面の曲げ耐力 M0（N=0）。
/// - `shear_failure`: 破壊モードがせん断破壊か（増分解析のせん断降伏イベント）。
///
/// せん断破壊は同じ (N/N0, sM0/M0) 区分で曲げ破壊より 1 段階不利になる
/// （FA→FB、FC→FD。N/N0 > 0.4 は破壊モードによらず FD）。
pub fn src_column_rank(n_over_n0: f64, smo_over_m0: f64, shear_failure: bool) -> MemberRank {
    if n_over_n0 > 0.4 {
        return MemberRank::FD;
    }
    // 曲げ破壊の基本ランク（表の上段）。
    let base: u8 = match (n_over_n0 > 0.3, smo_over_m0 < 0.4) {
        (false, false) => 0,                // FA
        (false, true) | (true, false) => 1, // FB
        (true, true) => 2,                  // FC
    };
    let idx = base + u8::from(shear_failure);
    match idx {
        0 => MemberRank::FA,
        1 => MemberRank::FB,
        2 => MemberRank::FC,
        _ => MemberRank::FD,
    }
}

/// SRC 耐震壁の種別。破壊モードがせん断破壊の場合を WC、それ以外を WA とする。
///
/// 返り値は Ds 表の行選択に用いるグループ表現（0=A(WA)…2=C(WC)。
/// [`super::ds_group`] の規約と同じく [`MemberRank`] のインデックスで表す）。
pub fn src_wall_type(shear_failure: bool) -> MemberRank {
    if shear_failure {
        MemberRank::FC // WC（グループ C）
    } else {
        MemberRank::FA // WA（グループ A）
    }
}

/// 表 2.6.6-5 の判定比 `(N/N0, sM0/M0)` を SRC 矩形断面と材料強度から算定する。
///
/// 略算累加による評価（SRC 規準の累加強度の考え方に基づく Ds 算定用の略算）:
/// - `N0 = c·Ac·Fc + sA·sF + ar·σy`（コンクリート正味断面＋内蔵鉄骨＋主筋全量）
/// - `sM0 = sZp·sF`（内蔵 H 形鋼の強軸全塑性モーメント）
/// - `M0 = sM0 + rM0`（`rM0 = 0.9·at·σy·de`。N=0 の RC 部分略算曲げ耐力、
///   [`squid_n_core::rc_capacity::rc_mu_simple`] と同式）
///
/// 内蔵鉄骨の基準強度 sF は `steel_grade` から板厚区分込みで解決し、解決できない
/// 場合は SS400 相当（F=235）にフォールバックする（[`crate::srrc`] の慣例と同じ）。
///
/// - `fc`: コンクリート強度 [N/mm²]（0 以下は算定不能で `None`）。
/// - `rebar_sy`: 主筋の降伏強度 σy [N/mm²]（0 以下は算定不能で `None`）。
/// - `n_ult`: メカニズム時の軸方向力 [N]（**圧縮正**。引張は 0 として扱う）。
///
/// SRC 矩形（[`SectionShape::SrcRect`]）以外の形状は `None`。
pub fn src_column_rank_ratios(
    shape: &SectionShape,
    steel_grade: &str,
    fc: f64,
    rebar_sy: f64,
    n_ult: f64,
) -> Option<(f64, f64)> {
    let SectionShape::SrcRect {
        b,
        d,
        rebar,
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
    } = shape
    else {
        return None;
    };
    if fc <= 0.0 || rebar_sy <= 0.0 || *b <= 0.0 || *d <= 0.0 {
        return None;
    }
    let (sh, sb, tw, tf) = (
        *steel_height,
        *steel_width,
        *steel_web_thick,
        *steel_flange_thick,
    );
    if sh <= 0.0 || sb <= 0.0 || tw <= 0.0 || tf <= 0.0 || sh <= 2.0 * tf {
        return None;
    }
    // 内蔵鉄骨の基準強度（板厚区分はフランジ厚で解決）。
    let s_f = crate::steel::steel_f_value_prefix(steel_grade, tf).unwrap_or(235.0);

    // 内蔵 H 形鋼の断面積と強軸全塑性断面係数。
    let hw = sh - 2.0 * tf;
    let s_a = 2.0 * sb * tf + hw * tw;
    let s_zp = sb * tf * (sh - tf) + tw * hw * hw / 4.0;
    let s_m0 = s_zp * s_f;

    // 主筋量（全量と、強軸曲げの片側引張量）。
    let ar_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
    let at = bar_set_area(&rebar.main_x) / 2.0;

    // N0 = c·Ac·Fc + sA·sF + ar·σy（コンクリートは鉄骨・主筋を控除した正味断面）。
    let ac = (b * d - s_a - ar_total).max(0.0);
    let n0 = ac * fc + s_a * s_f + ar_total * rebar_sy;
    if n0 <= 0.0 {
        return None;
    }

    // rM0 = 0.9·at·σy·de（N=0 の RC 部分略算曲げ耐力。de は引張縁主筋重心まで
    // の距離を控除した有効せい）。
    let dt = crate::rc::tension_dt(rebar.cover, rebar.shear.dia, &rebar.main_x);
    let de = (d - dt).max(0.0);
    let r_m0 = 0.9 * at * rebar_sy * de;
    let m0 = s_m0 + r_m0;
    if m0 <= 0.0 {
        return None;
    }

    Some((n_ult.max(0.0) / n0, s_m0 / m0))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 表 2.6.6-5 の全区分（曲げ破壊・せん断破壊 × 5 列）を検証する。
    #[test]
    fn test_src_column_rank_table_2_6_6_5() {
        // 曲げ破壊（上段）
        assert_eq!(src_column_rank(0.2, 0.5, false), MemberRank::FA);
        assert_eq!(src_column_rank(0.2, 0.3, false), MemberRank::FB);
        assert_eq!(src_column_rank(0.35, 0.5, false), MemberRank::FB);
        assert_eq!(src_column_rank(0.35, 0.3, false), MemberRank::FC);
        assert_eq!(src_column_rank(0.5, 0.5, false), MemberRank::FD);
        // せん断破壊（下段）: 同区分で 1 段階不利
        assert_eq!(src_column_rank(0.2, 0.5, true), MemberRank::FB);
        assert_eq!(src_column_rank(0.2, 0.3, true), MemberRank::FC);
        assert_eq!(src_column_rank(0.35, 0.5, true), MemberRank::FC);
        assert_eq!(src_column_rank(0.35, 0.3, true), MemberRank::FD);
        assert_eq!(src_column_rank(0.5, 0.3, true), MemberRank::FD);
    }

    /// 境界値: N/N0=0.3・0.4、sM0/M0=0.4 は「以下」「以上」側（有利側）に入る。
    #[test]
    fn test_src_column_rank_boundaries() {
        assert_eq!(src_column_rank(0.3, 0.4, false), MemberRank::FA);
        assert_eq!(src_column_rank(0.4, 0.4, false), MemberRank::FB);
        assert_eq!(src_column_rank(0.4, 0.39, false), MemberRank::FC);
    }

    #[test]
    fn test_src_wall_type() {
        assert_eq!(src_wall_type(false), MemberRank::FA, "WA");
        assert_eq!(src_wall_type(true), MemberRank::FC, "WC");
    }

    fn sample_src_shape() -> SectionShape {
        use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
        SectionShape::SrcRect {
            b: 600.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 8,
                    dia: 25.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 25.0,
                    layers: 1,
                },
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
                cover: 40.0,
            },
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 8.0,
            steel_flange_thick: 13.0,
        }
    }

    /// 判定比の算定: N0・sM0・M0 の略算累加が手計算と一致する。
    #[test]
    fn test_src_column_rank_ratios_hand_calc() {
        let shape = sample_src_shape();
        let (n_n0, smo_m0) = src_column_rank_ratios(&shape, "SN400B", 24.0, 345.0, 2_000_000.0)
            .expect("SRC 矩形は算定可能");

        // 手計算（SN400B tf=13 → F=235）:
        // sA = 2·200·13 + (400−26)·8 = 5200 + 2992 = 8192
        // sZp = 200·13·(400−13) + 8·374²/4 = 1_006_200 + 279_752 = 1_285_952
        // sM0 = 1_285_952·235 = 302_198_720 N·mm
        let s_m0 = 1_285_952.0 * 235.0;
        // ar = (8+4)·π/4·25² ≈ 5890.49、at = 8 本/2 側 → 4 本分 ≈ 1963.50
        let one = std::f64::consts::PI / 4.0 * 25.0 * 25.0;
        let ar = 12.0 * one;
        let at = 4.0 * one;
        // Ac = 600·600 − 8192 − ar、N0 = Ac·24 + 8192·235 + ar·345
        let ac = 600.0 * 600.0 - 8192.0 - ar;
        let n0 = ac * 24.0 + 8192.0 * 235.0 + ar * 345.0;
        assert!((n_n0 - 2_000_000.0 / n0).abs() < 1e-9, "N/N0={n_n0}");
        // dt = 40 + 10 + 12.5 = 62.5、de = 537.5、rM0 = 0.9·at·345·537.5
        let r_m0 = 0.9 * at * 345.0 * 537.5;
        let expect = s_m0 / (s_m0 + r_m0);
        assert!(
            (smo_m0 - expect).abs() < 1e-9,
            "sM0/M0={smo_m0} 期待 {expect}"
        );
    }

    /// SRC 矩形以外・強度未設定は None。
    #[test]
    fn test_src_column_rank_ratios_invalid_inputs() {
        let shape = sample_src_shape();
        assert!(src_column_rank_ratios(&shape, "SN400B", 0.0, 345.0, 0.0).is_none());
        assert!(src_column_rank_ratios(&shape, "SN400B", 24.0, 0.0, 0.0).is_none());
        let SectionShape::SrcRect { rebar, .. } = sample_src_shape() else {
            unreachable!()
        };
        let rc = SectionShape::RcRect {
            b: 600.0,
            d: 600.0,
            rebar,
        };
        assert!(src_column_rank_ratios(&rc, "SN400B", 24.0, 345.0, 0.0).is_none());
    }
}
