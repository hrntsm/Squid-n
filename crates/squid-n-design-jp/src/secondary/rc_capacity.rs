//! RC 矩形断面の簡易終局耐力算定（部材ランク判定用）— 実装本体は squid-n-core に移設。
//!
//! `squid_n_solver::pushover` のせん断降伏判定（Qy の荒川式化）でも同じ算定式
//! （`rc_qsu_simple` 等）を使うが、squid-n-solver（Layer 4）は squid-n-design-jp
//! （Layer 5）に依存できない（循環依存になる）ため、実装本体は Layer 0 の
//! `squid_n_core::rc_capacity` へ移設した。本モジュールは既存呼び出し
//! （`squid_n_design_jp::secondary::rc_capacity::{rc_qsu_simple, RcCapacityInput}` 等、
//! 例: `squid-n-app::app::rc_capacity_input_from_rect`）を無修正で維持するための
//! 再エクスポートのみを行う。
pub use squid_n_core::rc_capacity::*;

#[cfg(test)]
mod tests {
    use super::*;

    /// 代表断面: b=400, D=600, at=1935(D25×3程度), d_eff=530, σy=345, Fc=24,
    /// pw=0.002, σwy=295, h0=3000（`squid_n_core::rc_capacity` のテストと同一断面）。
    fn sample_input() -> RcCapacityInput {
        RcCapacityInput {
            b: 400.0,
            d: 600.0,
            at: 1935.0,
            d_eff: 530.0,
            sigma_y: 345.0,
            fc: 24.0,
            pw: 0.002,
            sigma_wy: 295.0,
            clear_span: 3000.0,
            sigma_0: 0.0,
        }
    }

    #[test]
    fn test_rc_mu_simple_matches_fiber_analysis() {
        // rc_mu_simple（略算式）と squid-n-skeleton のファイバ解析（build_rc_member_skeleton）の
        // 終局モーメント Mu を突合し、部材ランク判定用の略算の妥当域を回帰で担保する。
        //
        // squid-n-skeleton・squid-n-material は Layer 2/0 だが squid-n-core（Layer 0）からは
        // dev-dependency でも参照できない（xtask check-deps のレイヤ規則、上位→下位のみ許可）
        // ため、この突合テストは squid-n-design-jp（Layer 5）側に残す
        // （算定式本体は squid_n_core::rc_capacity へ移設済み、re-export で参照）。
        //
        // sample_input() と同等の代表 RC 梁断面: b=400, D=600, 引張主筋 at≈1935mm²
        // (D25相当×3, as_bar=645), d_eff=530, σy=345, Fc=24。かぶり+主筋半径を
        // D/2-d_eff=70mm と仮定し、主筋は引張・圧縮側に上下対称に配置する
        // （squid-n-skeleton の既存テスト test_rc_skeleton_ultimate_matches_handcalc 等と
        // 同じ流儀で Section/Reinforcement/Concrete/Bilinear/SkeletonOptions を組む）。
        //
        // 軸力ゼロ（SkeletonOptions.n_axial=0）条件で比較する（rc_mu_simple は軸力を
        // 考慮しない略算式のため）。
        use squid_n_core::ids::SectionId;
        use squid_n_core::model::Section;
        use squid_n_material::{Bilinear, Concrete};
        use squid_n_skeleton::{
            build_rc_member_skeleton, PulloutContribution, Reinforcement, ShearContribution,
            SkeletonOptions,
        };

        let inp = sample_input();
        let b = inp.b;
        let d_total = inp.d;
        let at = inp.at;
        let d_eff = inp.d_eff;
        let fy = inp.sigma_y;
        let fc = inp.fc;

        let sec = Section {
            id: SectionId(0),
            name: "test".into(),
            area: b * d_total,
            iy: b * d_total.powi(3) / 12.0,
            iz: d_total * b.powi(3) / 12.0,
            j: b.powi(3) * d_total / 3.0,
            depth: d_total,
            width: b,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        // 引張側 z = +(d_eff - D/2)、圧縮側 z = -(d_eff - D/2)（上下対称配置、断面積同一）。
        let z_tension = d_eff - d_total / 2.0;
        let rebar = Reinforcement {
            main_bars: vec![(0.0, z_tension, at), (0.0, -z_tension, at)],
            hoop_pitch: 100.0,
            hoop_area: 0.0,
        };
        let concrete = Concrete::new(fc, 2.0);
        let steel = Bilinear::new(200000.0, fy, 0.01);
        let opts = SkeletonOptions {
            span: 4000.0, // Mu（モーメント値）には無関係（M-θ 変換にのみ影響）
            inflection_ratio: 0.5,
            n_axial: 0.0, // 軸力ゼロ条件で比較（rc_mu_simple は軸力非考慮の略算式）
            alpha: 0.4,
        };
        let skeleton = build_rc_member_skeleton(
            &sec,
            &rebar,
            &concrete,
            &steel,
            &opts,
            &ShearContribution::none(),
            &PulloutContribution::none(),
        );

        let mu_fiber = skeleton.points.get(3).map(|p| p.1).unwrap_or(0.0);
        let mu_simple = rc_mu_simple(&inp);
        let ratio = mu_simple / mu_fiber;
        // squid-n-skeleton 側の既存突合テスト（test_rc_skeleton_ultimate_matches_handcalc）が
        // 「規準式との一致は 30% 以内を許容」としている規律に合わせ、略算 Mu とファイバ Mu の
        // 比が 0.7〜1.3 に収まることを部材ランク判定用略算の妥当域として回帰で担保する。
        assert!(
            ratio > 0.7 && ratio < 1.3,
            "rc_mu_simple ({:.3} N·mm) vs fiber Mu ({:.3} N·mm): ratio={:.3}",
            mu_simple,
            mu_fiber,
            ratio
        );
    }
}
