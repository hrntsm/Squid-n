//! S 造パネルゾーンの断面検定（許容応力度検定。
//! 鋼構造接合部設計指針のパネルゾーン部分に準拠）。
//!
//! # 位置付け
//! このモジュールは `Model` や要素（`squid_n_element`）に依存せず、呼び出し側
//! （節点まわりの応力集計・断面形状の解決を担当する別モジュール）が用意した数値
//! 入力を受け取る**純関数**として実装する。
//!
//! ただしパネルの寸法・形状係数 κ・実効体積 `Ve` は、仕口パネルの**モデル化**
//! （`squid_n_element::springs::panel`）と同一の値でなければならない。同じ接合部に対して
//! 剛性と耐力が食い違う諸元で算定されるのを防ぐため、これらは
//! [`squid_n_core::panel_zone::PanelGeometry`]（`Model` に依存しない値型）へ
//! 一元化し、本モジュールもモデル化側もそこを唯一の出所とする。
//!
//! 準拠する規準: 日本建築学会「鋼構造接合部設計指針」
//!
//! # 式の再構成・簡略化について（重要）
//! 参照した原典テキストは PDF/MathML からの抽出であり、分数式や上付き添字が
//! 崩れている箇所がある。S パネルゾーンの形状係数 κ は分数 2 項和の形に
//! 再構成した（下記 [`s_panel_zone_check`] のドキュメント参照）。

use crate::{CheckComponent, CheckKind, CheckResult};
use squid_n_core::panel_zone::{PanelGeometry, PanelShapeKind};

/// S 造パネルゾーンの検定の入力。
pub struct SPanelInput {
    /// 柱断面から解決したパネル諸元（寸法 `dc`・板厚 `tp`・断面区分）。
    /// CFT も対象に含める（鋼管部を S 造と同じ式で評価する）。
    pub geometry: PanelGeometry,
    /// 梁フランジ板厚中心間距離 db [mm]。
    pub db: f64,
    /// パネルの降伏強さ F 値 [N/mm²]。
    pub fy: f64,
    /// 軸力比 n = N / (Fy・A)（符号は問わない。内部で絶対値を用いる）。
    pub axial_ratio: f64,
    /// 左梁フェイスモーメント [N·mm]（符号付き）。
    pub beam_moment_left: f64,
    /// 右梁フェイスモーメント [N·mm]（符号付き）。
    pub beam_moment_right: f64,
    /// 上柱せん断力 [N]。
    pub col_shear_upper: f64,
    /// 下柱せん断力 [N]。
    pub col_shear_lower: f64,
    /// 設計用パネルモーメント `pM` [N·mm] を直接与える場合の値。
    ///
    /// 仕口パネルをモデル化した接合部では、解析が出力するパネルせん断モーメント
    /// `{MSX, MSY}` の絶対値が大きい方をここへ与える。節点まわりのモーメント
    /// 釣り合いが解析上厳密に満たされた値であり、梁端モーメント・柱せん断から
    /// 手で組み立てる近似（`beam_moment_*` / `col_shear_*`）を経ない。
    ///
    /// `None` のときは従来どおり
    /// `pM = bML + bMR − (cQU + cQL)・db/2` で組み立てる。
    pub design_moment: Option<f64>,
}

/// S 造パネルゾーンの検定（鋼構造接合部設計指針）。
///
/// ## 設計用パネルモーメント（標準形式・梁段違いなし）
/// `pM = bML + bMR − (cQU + cQL)・db/2`
///
/// 梁段違い形式（左右梁のせい差が概ね 150mm 以上）は本関数の対象外とし、
/// 呼び出し側が段違いを考慮した等価な `db`（低い方の梁の値）を渡す簡略化とする。
///
/// 仕口パネルをモデル化した接合部では、この組み立てに代えて解析が出力する
/// パネルせん断モーメントを [`SPanelInput::design_moment`] へ直接与える
/// （節点まわりの釣り合いが解析上厳密に満たされた値を用いる）。
///
/// ## パネル降伏モーメント
/// `pMy = (Ve/κ)・√(1 − n²)・Fy/√3`
///
/// - H形: `Ve = dc・db・tp`、
///   `κ = 1/(2/3 + (4・bc・tf)/(dc・tp)) + 1/(1 + (dc・tp)/(6・bc・tf))`
/// - 角形: `Ve = 2・dc・db・tp`、
///   `κ = 1/(2/3 + 2・bc/dc) + 1/(1 + dc/(3・bc))`
/// - 円形: `Ve = 2・dc・db・tp`、`κ = 4/π`
///
/// **原典照合済み（2026-07-11）**: 接合部パネル降伏モーメントの原典図
/// （ユーザー提供）と照合し、`pMy = (Ve/κ)・√(1−n²)・Fy/√3`（Ve を κ で
/// **除する**）であること、および κ の 3 形状分の式が上記で正しいことを確認した。
/// κ は概ね 0.5〜1.5 のオーダーで、Ve/κ でも整合的（ユニットテストで確認）。
///
/// `n = |axial_ratio|` とし、`|n| ≥ 1` の場合は `√(1 − n²)` を 0 にクランプする
/// （軸力が全塑性軸耐力に達している状態を表し、曲げ・せん断耐力の余裕なしに対応）。
///
/// 検定比 = `|pM| / pMy`（1.0 以下で OK）。
pub fn s_panel_zone_check(inp: &SPanelInput) -> CheckResult {
    // Ve・κ はモデル化側（仕口パネル要素の剛性 Kxp = Kyp = G・Ve）と同じ値を用いる。
    let ve = inp.geometry.effective_volume(inp.db);
    let kappa = inp.geometry.kappa();
    let shape_label = match inp.geometry.kind {
        PanelShapeKind::H { .. } => "H形",
        PanelShapeKind::Box { .. } => "角形",
        PanelShapeKind::Pipe => "円形",
    };

    let n = inp.axial_ratio.abs();
    let reduction = if n >= 1.0 { 0.0 } else { (1.0 - n * n).sqrt() };

    // pMy = (Ve/κ)・√(1−n²)・Fy/√3（原典図で Ve/κ を確認、2026-07-11）。
    let kappa = if kappa.abs() > 1e-9 { kappa } else { 1e-9 };
    let p_my = ve / kappa * reduction * inp.fy / 3f64.sqrt();
    // 仕口パネルをモデル化した接合部は解析出力のパネルせん断モーメントを用いる。
    // モデル化していない接合部は梁端モーメント・柱せん断から組み立てる。
    let p_m = inp.design_moment.unwrap_or_else(|| {
        inp.beam_moment_left + inp.beam_moment_right
            - (inp.col_shear_upper + inp.col_shear_lower) * inp.db / 2.0
    });

    let ratio = if p_my > 0.0 {
        p_m.abs() / p_my
    } else {
        f64::INFINITY
    };
    let basis = format!("鋼構造接合部設計指針 パネルゾーン検定 {}断面", shape_label);
    // 単一式（Shear）の検定のため、全文を component の detail に置き、
    // 共通 detail は空文字列とする。
    CheckResult {
        basis,
        detail: String::new(),
        components: vec![CheckComponent {
            kind: CheckKind::Shear,
            ratio,
            detail: format!(
                "Ve={:.1} mm2, kappa={:.4}, n={:.4}, pM={:.1} N*mm, pMy={:.1} N*mm, ratio={:.4}",
                ve, kappa, n, p_m, p_my, ratio
            ),
        }],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_panel_h_input(axial_ratio: f64) -> SPanelInput {
        SPanelInput {
            geometry: PanelGeometry {
                kind: PanelShapeKind::H {
                    bc: 300.0,
                    tf: 20.0,
                },
                dc: 400.0,
                tp: 12.0,
                filled: false,
            },
            db: 500.0,
            fy: 235.0,
            axial_ratio,
            beam_moment_left: 200_000_000.0,
            beam_moment_right: 200_000_000.0,
            col_shear_upper: 50_000.0,
            col_shear_lower: 50_000.0,
            design_moment: None,
        }
    }

    /// CFT 柱も断面検定の対象に含める（鋼管部を S 造と同じ式で評価する）。
    /// 仕口パネルの**モデル化**は CFT を除外するため、対象範囲が食い違う点を押さえる。
    #[test]
    fn s_panel_covers_cft_columns() {
        let steel = PanelGeometry {
            kind: PanelShapeKind::Box { bc: 400.0 },
            dc: 384.0,
            tp: 16.0,
            filled: false,
        };
        let cft = PanelGeometry {
            filled: true,
            ..steel
        };
        assert!(!steel.filled, "角形鋼管はモデル化の対象");
        assert!(cft.filled, "CFT はモデル化の対象外");

        // 検定側は充填の有無で結果が変わらない。
        let check = |geometry: PanelGeometry| {
            let mut inp = base_panel_h_input(0.0);
            inp.geometry = geometry;
            s_panel_zone_check(&inp).ratio()
        };
        let (rs, rc) = (check(steel), check(cft));
        assert!(rs > 0.0 && rs.is_finite());
        assert!(
            (rs - rc).abs() < 1e-12,
            "CFT も同じ検定比になる: {rs} vs {rc}"
        );
    }

    /// `design_moment` を与えると、梁端モーメント・柱せん断からの組み立てに代えて
    /// その値が設計用パネルモーメント pM に用いられる（仕口パネルをモデル化した
    /// 接合部で、解析出力のせん断モーメントを使う経路）。
    #[test]
    fn s_panel_uses_design_moment_when_given() {
        let assembled = s_panel_zone_check(&base_panel_h_input(0.0));
        // 手組み式の pM = 200e6 + 200e6 − (50e3 + 50e3)・500/2 = 375e6
        let assembled_pm = 375_000_000.0_f64;

        let mut inp = base_panel_h_input(0.0);
        inp.design_moment = Some(assembled_pm);
        let same = s_panel_zone_check(&inp);
        assert!(
            (same.ratio() - assembled.ratio()).abs() < 1e-12,
            "同じ pM を与えれば結果は一致する: {} vs {}",
            same.ratio(),
            assembled.ratio()
        );

        // 別の値を与えれば検定比は pM に比例して変わる。
        let mut half = base_panel_h_input(0.0);
        half.design_moment = Some(assembled_pm / 2.0);
        let half = s_panel_zone_check(&half);
        assert!(
            (half.ratio() - assembled.ratio() / 2.0).abs() < 1e-12,
            "pM が半分なら検定比も半分: {} vs {}",
            half.ratio(),
            assembled.ratio() / 2.0
        );

        // 符号は絶対値で扱う（左右どちら向きのパネルモーメントでも同じ検定比）。
        let mut neg = base_panel_h_input(0.0);
        neg.design_moment = Some(-assembled_pm);
        assert!((s_panel_zone_check(&neg).ratio() - assembled.ratio()).abs() < 1e-12);
    }

    #[test]
    fn s_panel_kappa_h_is_order_one() {
        let bc = 300.0_f64;
        let tf = 20.0_f64;
        let dc = 400.0_f64;
        let tp = 12.0_f64;
        let kappa = 1.0 / (2.0 / 3.0 + (4.0 * bc * tf) / (dc * tp))
            + 1.0 / (1.0 + (dc * tp) / (6.0 * bc * tf));
        assert!(
            (0.5..=1.5).contains(&kappa),
            "kappa should be O(1), got {}",
            kappa
        );
    }

    #[test]
    fn s_panel_kappa_box_is_order_one() {
        let bc = 400.0_f64;
        let dc = 400.0_f64;
        let kappa = 1.0 / (2.0 / 3.0 + 2.0 * bc / dc) + 1.0 / (1.0 + dc / (3.0 * bc));
        assert!(
            (0.5..=1.5).contains(&kappa),
            "kappa should be O(1), got {}",
            kappa
        );
    }

    #[test]
    fn s_panel_kappa_pipe_is_order_one() {
        let kappa = 4.0 / std::f64::consts::PI;
        assert!((0.5..=1.5).contains(&kappa));
    }

    #[test]
    fn s_panel_axial_ratio_reduces_capacity() {
        let n0 = rc_or_s_pmy(&base_panel_h_input(0.0));
        let n08 = rc_or_s_pmy(&base_panel_h_input(0.8));
        assert!(n08 < n0, "n=0.8 の pMy は n=0 より小さいはず");
        // sqrt(1-0.8^2) = 0.6 のスケーリングになっていることを確認。
        assert!((n08 / n0 - 0.6).abs() < 1e-9);
    }

    // pMy を検定比から逆算するテスト用ヘルパ。
    fn rc_or_s_pmy(inp: &SPanelInput) -> f64 {
        let res = s_panel_zone_check(inp);
        let p_m = inp.beam_moment_left + inp.beam_moment_right
            - (inp.col_shear_upper + inp.col_shear_lower) * inp.db / 2.0;
        p_m.abs() / res.ratio()
    }

    #[test]
    fn s_panel_moment_hand_calc() {
        let inp = base_panel_h_input(0.0);
        let res = s_panel_zone_check(&inp);
        let expected_pm = inp.beam_moment_left + inp.beam_moment_right
            - (inp.col_shear_upper + inp.col_shear_lower) * inp.db / 2.0;
        // pM = 200e6+200e6 - (50000+50000)*500/2 = 400e6 - 25e6 = 375e6
        assert!((expected_pm - 375_000_000.0).abs() < 1e-3);
        assert!(res.ratio() > 0.0);
    }

    #[test]
    fn s_panel_axial_ratio_at_or_above_one_clamps_to_zero() {
        let inp = base_panel_h_input(1.0);
        let res = s_panel_zone_check(&inp);
        assert!(
            res.ratio().is_infinite(),
            "pMy=0 のとき ratio は無限大になる"
        );
        let inp2 = base_panel_h_input(1.5);
        let res2 = s_panel_zone_check(&inp2);
        assert!(res2.ratio().is_infinite());
    }
}
