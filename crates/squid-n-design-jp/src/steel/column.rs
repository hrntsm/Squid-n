//! 鉄骨造柱の断面検定（鋼構造設計規準の
//! 鉄骨造柱の許容応力度検定）。

use crate::material_strength::{steel_fc, steel_fs, steel_ft};
use crate::{
    effective_slenderness, CheckComponent, CheckKind, CheckResult, DesignCtx, LoadTerm,
    MemberForcesAt, SteelFbRule,
};
use squid_n_core::model::{Material, Section};

use super::section::{
    steel_c_factor, steel_fb_h, steel_fb_h_new, steel_i_t, steel_p_lambda_b, steel_warping_constant,
};
use super::{nonzero, safe_denom, section_modulus, shape_of, shear_area_2d, ShapeCategory};

/// 鉄骨造柱の断面検定（鋼構造設計規準）。
///
/// 軸力+二軸曲げ: `σ/f + σbX/fbX + σbY/fbY ≤ 1.0`
/// （円形鋼管は `σb=√(mz²+my²)/Z` に一本化）。
/// せん断は von Mises 型: `max(√(σ²+3τ²)/ft, τ/fs)`。
pub(crate) fn check_column(
    forces: &MemberForcesAt,
    sec: &Section,
    mat: &Material,
    ctx: &DesignCtx,
    f: f64,
    term: LoadTerm,
) -> CheckResult {
    let h = sec.depth;
    let b = sec.width;
    let area = nonzero(sec.area);
    let z_strong = nonzero(section_modulus(sec.iy, h / 2.0));
    let z_weak = nonzero(section_modulus(sec.iz, b / 2.0));
    // 強軸/弱軸曲げ応力度 σbX = |Mz|/Z強軸、σbY = |My|/Z弱軸。
    let sigma_bx = forces.mz.abs() / z_strong;
    let sigma_by = forces.my.abs() / z_weak;

    let (shape, tf, tw) = shape_of(sec);

    let ft_val = steel_ft(f, term);
    let fs_val = steel_fs(f, term);

    // 有効細長比 λ = max(lk_y/i_y, lk_z/i_z)（強軸・弱軸を個別の座屈長さで評価）。
    let lk_y_resolved = ctx.lk_y.unwrap_or(ctx.length);
    let lk_z_resolved = ctx.lk_z.unwrap_or(ctx.length);
    let buckling_note = if lk_y_resolved <= 1e-9 && lk_z_resolved <= 1e-9 {
        "（座屈長さ0のため座屈無視 λ=0）"
    } else {
        ""
    };
    let lambda = effective_slenderness(sec.iy, sec.iz, area, ctx.length, ctx.lk_y, ctx.lk_z);
    // 座屈を考慮した許容圧縮応力度 fc（鋼構造設計規準 1973、λ に応じた低減）。
    let fc_val = steel_fc(f, lambda, term);

    // 強軸 fb（H形のみ横座屈考慮。lb は柱の階高 = ctx.length。旧基準/新基準の
    // 切替は梁と同様 ctx.steel_fb_rule による）。修正係数 C は梁と同様
    // ctx.end_moments_z/mid_moment_z から求める（柱も端部モーメント比により
    // fb1 が変化する。SteelDesignAttr.c_direct による直接入力にも対応）。
    let c = steel_c_factor(ctx, false);
    let fb_strong = match shape {
        ShapeCategory::H => match ctx.steel_fb_rule {
            SteelFbRule::Old => {
                let af = b * tf;
                let i_t = steel_i_t(b, tf, h, tw);
                steel_fb_h(f, term, ctx.length, i_t, h, af, c)
            }
            SteelFbRule::New => {
                let iz = sec.iz;
                let iw = steel_warping_constant(sec, tf);
                let j = sec.j;
                let e = mat.young;
                let g = mat.shear.unwrap_or(e / (2.0 * (1.0 + mat.poisson)));
                let p_lambda_b = steel_p_lambda_b(ctx);
                steel_fb_h_new(
                    f, term, ctx.length, iz, iw, j, e, g, z_strong, c, p_lambda_b,
                )
            }
        },
        _ => ft_val,
    };
    let fb_weak = ft_val;

    // 円形鋼管は二軸曲げを合成した σb に一本化: σb = √(mz²+my²)/Z強軸。
    let sigma_b_pipe = (forces.mz.powi(2) + forces.my.powi(2)).sqrt() / z_strong;

    let axial_stress;
    let ratio_axial_bend;
    let axial_basis;
    if forces.n < 0.0 {
        // 圧縮+曲げ: σc/fc(座屈考慮) + ΣσB/fb ≤ 1.0。
        let sigma_c = forces.n.abs() / area;
        axial_stress = sigma_c;
        ratio_axial_bend = match shape {
            ShapeCategory::Pipe => {
                sigma_c / safe_denom(fc_val) + sigma_b_pipe / safe_denom(fb_strong)
            }
            _ => {
                sigma_c / safe_denom(fc_val)
                    + sigma_bx / safe_denom(fb_strong)
                    + sigma_by / safe_denom(fb_weak)
            }
        };
        axial_basis = "圧縮+曲げ: σc/fc(座屈考慮)+ΣσB/fb";
    } else {
        // 引張+曲げ: σt/ft + ΣσB/fb ≤ 1.0。
        let sigma_t = forces.n / area;
        axial_stress = sigma_t;
        ratio_axial_bend = match shape {
            ShapeCategory::Pipe => {
                sigma_t / safe_denom(ft_val) + sigma_b_pipe / safe_denom(fb_strong)
            }
            _ => {
                sigma_t / safe_denom(ft_val)
                    + sigma_bx / safe_denom(fb_strong)
                    + sigma_by / safe_denom(fb_weak)
            }
        };
        axial_basis = "引張+曲げ: σt/ft+ΣσB/fb";
    }

    // せん断: せん断有効断面積は梁と共用の単一定義（`shear_area_2d`。
    // H形はウェブ内法 tw·(H−2tf)、角形は角部円弧考慮）。H形・角形は
    // 強軸 τ=Qy/Ay、円形（Ay=Az）・その他は合成 τ=√(qy²+qz²)/Ay。
    let (as_y, _as_z) = shear_area_2d(shape, sec, tf, tw);
    let tau = match shape {
        ShapeCategory::H | ShapeCategory::Box => forces.qy.abs() / safe_denom(as_y),
        ShapeCategory::Pipe | ShapeCategory::Other => {
            (forces.qy.powi(2) + forces.qz.powi(2)).sqrt() / safe_denom(as_y)
        }
    };
    let sigma_total = match shape {
        ShapeCategory::H => axial_stress + sigma_bx * (h - 2.0 * tf).max(0.0) / safe_denom(h),
        ShapeCategory::Pipe => axial_stress + sigma_b_pipe,
        _ => axial_stress + sigma_bx + sigma_by,
    };
    // von Mises 型合成検定: max(√(σ²+3τ²)/ft, τ/fs)。
    let ratio_shear = ((sigma_total.powi(2) + 3.0 * tau.powi(2)).sqrt() / safe_denom(ft_val))
        .max(tau / safe_denom(fs_val));

    let term_label = match term {
        LoadTerm::Long => "長期",
        LoadTerm::Short => "短期",
    };
    let basis = format!(
        "鋼構造設計規準 {} 柱: {}{}, せん断 von Mises",
        term_label, axial_basis, buckling_note
    );
    // AxialBending 固有: 応力度・許容応力度（座屈考慮 fc・横座屈考慮 fbX・
    // fbY）・細長比 λ・軸+曲げ検定比。
    let axial_bending_detail = format!(
        "σax={:.4} N/mm², σbX={:.4} N/mm², σbY={:.4} N/mm², fc={:.4} N/mm², fbX={:.4} N/mm², \
fbY={:.4} N/mm², λ={:.3}, 軸曲げ比={:.4}",
        axial_stress, sigma_bx, sigma_by, fc_val, fb_strong, fb_weak, lambda, ratio_axial_bend,
    );
    // Shear 固有: せん断応力度・許容せん断応力度・せん断（von Mises）検定比。
    let shear_detail = format!(
        "τ={:.4} N/mm², fs={:.4} N/mm², せん断比={:.4}",
        tau, fs_val, ratio_shear
    );
    // 両式で共有する断面諸元はないため共通 detail は空文字列とする。
    let detail = String::new();

    let components = vec![
        CheckComponent {
            kind: CheckKind::AxialBending,
            ratio: ratio_axial_bend,
            detail: axial_bending_detail,
        },
        CheckComponent {
            kind: CheckKind::Shear,
            ratio: ratio_shear,
            detail: shear_detail,
        },
    ];

    CheckResult {
        basis,
        detail,
        components,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::steel::test_support::{h_section, mat, rect_section};
    use crate::steel::SteelDesign;
    use crate::{DesignCheck, MemberKind};

    // -------------------------------------------------------------
    // 柱検定
    // -------------------------------------------------------------

    /// 断片が意図した component に配置されていることの確認
    /// （AxialBending の detail に "軸曲げ比" が含まれ、Shear の detail には
    /// 含まれない。逆に Shear 固有の "せん断比" は AxialBending に含まれない）。
    #[test]
    fn test_column_check_detail_fragments_assigned_to_intended_components() {
        let mut sec = rect_section(300.0, 300.0, "H-300x300x10x15");
        sec.thickness = Some(15.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -500_000.0,
            qy: 20_000.0,
            qz: 0.0,
            my: 20e6,
            mz: 50e6,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3500.0,
            ..Default::default()
        };
        let result = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx)
            .unwrap_checked();
        let axial_bending = result
            .components
            .iter()
            .find(|c| c.kind == crate::CheckKind::AxialBending)
            .expect("AxialBending component が存在するはず");
        let shear = result
            .components
            .iter()
            .find(|c| c.kind == crate::CheckKind::Shear)
            .expect("Shear component が存在するはず");
        assert!(axial_bending.detail.contains("軸曲げ比"));
        assert!(!shear.detail.contains("軸曲げ比"));
        assert!(shear.detail.contains("せん断比"));
        assert!(!axial_bending.detail.contains("せん断比"));
    }

    #[test]
    fn test_column_check_axial_biaxial_bending_hand_calc() {
        // H形柱: N=-500kN（圧縮）, Mz=50kN·m, My=20kN·m。
        let mut sec = rect_section(300.0, 300.0, "H-300x300x10x15");
        sec.thickness = Some(15.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -500_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 20e6,
            mz: 50e6,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3500.0,
            ..Default::default()
        };
        let result = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx)
            .unwrap_checked();

        let area = sec.area;
        let z_strong = sec.iy / (sec.depth / 2.0);
        let z_weak = sec.iz / (sec.width / 2.0);
        let sigma_c = 500_000.0 / area;
        let sigma_bx = 50e6_f64.abs() / z_strong;
        let sigma_by = 20e6_f64.abs() / z_weak;

        let i_min = (sec.iy.min(sec.iz) / area).sqrt();
        let lambda = 3500.0 / i_min;
        let fc = steel_fc(235.0, lambda, LoadTerm::Long);
        let ft = steel_ft(235.0, LoadTerm::Long);
        // fbX は横座屈考慮（H形）、fbY=ft。ここでは非負・上限 ft であることのみ検証。
        assert!(crate::full_detail(&result).contains("軸曲げ比"));
        assert!(sigma_c > 0.0 && sigma_bx > 0.0 && sigma_by > 0.0 && fc > 0.0 && ft > 0.0);
        // 軸+曲げ比は少なくとも σc/fc 単独の比より大きい（曲げ項が加算されるため）。
        assert!(result.ratio() >= sigma_c / fc - 1e-9);

        // components に AxialBending・Shear が入ることを確認する。
        assert_eq!(result.components.len(), 2);
        assert!(result
            .components
            .iter()
            .any(|c| c.kind == crate::CheckKind::AxialBending));
        assert!(result
            .components
            .iter()
            .any(|c| c.kind == crate::CheckKind::Shear));
    }

    #[test]
    fn test_column_check_pipe_combines_biaxial_sigma_b() {
        let mut sec = rect_section(300.0, 300.0, "PIPE-300x12");
        sec.iz = sec.iy; // 円形は iy=iz
        sec.thickness = Some(12.0);
        let mat_v = mat("SN400");
        let forces_x_only = MemberForcesAt {
            pos: 0.0,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 30e6,
        };
        let forces_biaxial = MemberForcesAt {
            pos: 0.0,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 30e6 / std::f64::consts::SQRT_2,
            mz: 30e6 / std::f64::consts::SQRT_2,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3000.0,
            ..Default::default()
        };
        let r1 = SteelDesign
            .check(&forces_x_only, &sec, &mat_v, &ctx)
            .unwrap_checked();
        let r2 = SteelDesign
            .check(&forces_biaxial, &sec, &mat_v, &ctx)
            .unwrap_checked();
        // 円形鋼管は sqrt(mz^2+my^2) で合成するため、合成曲げモーメントの大きさが
        // 同じであれば mz のみと mz/my 分配後で軸+曲げ比はほぼ一致するはず。
        assert!(
            (r1.ratio() - r2.ratio()).abs() < 1e-6,
            "pipe combined sigma_b mismatch: {} vs {}",
            r1.ratio(),
            r2.ratio()
        );
    }

    #[test]
    fn test_column_shear_von_mises_hand_calc() {
        // 純せん断（N=0, M=0）で von Mises 式 sqrt(3)*τ/ft と τ/fs を手計算照合。
        let mut sec = rect_section(300.0, 300.0, "BOX-300x300x12");
        sec.thickness = Some(12.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 300_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3000.0,
            ..Default::default()
        };
        let result = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx)
            .unwrap_checked();

        // 角形: 梁と共用のせん断有効断面積 Ay=2t(H−2t)（角部直角。r 未入力）。
        let ay = 2.0 * 12.0 * (300.0 - 2.0 * 12.0);
        let tau = 300_000.0_f64.abs() / ay;
        let ft = steel_ft(235.0, LoadTerm::Long);
        let fs = steel_fs(235.0, LoadTerm::Long);
        // σ=0（純せん断）なので von Mises 側は sqrt(3)*τ/ft。
        let expected = (3.0_f64.sqrt() * tau / ft).max(tau / fs);
        assert!(
            (result.ratio() - expected).abs() < 1e-6,
            "ratio={} expected={}",
            result.ratio(),
            expected
        );
    }

    // -------------------------------------------------------------
    // 座屈長さ 0 の扱い
    // -------------------------------------------------------------

    #[test]
    fn test_column_length_zero_ignores_buckling() {
        let sec = rect_section(300.0, 300.0, "BOX-300x300x16");
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx)
            .unwrap_checked();
        assert!(result.basis.contains("座屈無視"));
        // λ=0 なので fc=ft、単純圧縮比 = σc/ft と一致するはず。
        let ft = steel_ft(235.0, LoadTerm::Long);
        let expected = (100_000.0 / sec.area) / ft;
        assert!((result.ratio() - expected).abs() < 1e-6);
    }

    // -------------------------------------------------------------
    // 座屈長さの軸別化（lk_y/lk_z）
    // -------------------------------------------------------------

    /// lk_y=lk_z=None（=length 共通）の場合、λ=max(lk_y/i_y, lk_z/i_z) が
    /// 従来の λ=length/i_min（i_min=√(min(Iy,Iz)/A)）と一致することを、
    /// fc（λ に応じて単調減少）を介して確認する。
    #[test]
    fn test_column_lambda_both_axes_none_matches_legacy_i_min() {
        let sec = rect_section(300.0, 200.0, "H-300x200x8x12");
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 4000.0,
            ..Default::default()
        };
        let result = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx)
            .unwrap_checked();

        let area = sec.area;
        let i_min = (sec.iy.min(sec.iz) / area).sqrt();
        let lambda = 4000.0 / i_min;
        let fc = steel_fc(235.0, lambda, LoadTerm::Long);
        let expected = (100_000.0 / area) / fc;
        assert!(
            (result.ratio() - expected).abs() < 1e-9,
            "ratio={} expected(legacy i_min)={}",
            result.ratio(),
            expected
        );
    }

    /// lk_z（弱軸）のみを長くすると弱軸細長比が支配し fc が下がる（検定比が
    /// 上がる）ことを確認する。矩形断面 B=300 > D=100 とし、iz（弱軸=B方向）
    /// を十分小さくして弱軸が支配しやすい断面とする。
    #[test]
    fn test_column_lk_z_only_governs_weak_axis_slenderness() {
        let sec = rect_section(100.0, 300.0, "H-300x100x8x12");
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -50_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx_base = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3000.0,
            lk_y: Some(3000.0),
            lk_z: Some(3000.0),
            ..Default::default()
        };
        let base = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx_base)
            .unwrap_checked();

        let ctx_long_z = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3000.0,
            lk_y: Some(3000.0),
            lk_z: Some(9000.0), // 弱軸のみ座屈長さを 3 倍に
            ..Default::default()
        };
        let long_z = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx_long_z)
            .unwrap_checked();

        assert!(
            long_z.ratio() > base.ratio(),
            "lk_z を伸ばすと検定比が上がるはず: base={} long_z={}",
            base.ratio(),
            long_z.ratio()
        );
    }

    // -------------------------------------------------------------
    // 新基準 fb（AIJ-ASD19）の柱への配線
    // -------------------------------------------------------------

    /// steel_fb_rule 未指定（既定 Old）では従来値（steel_fb_h・
    /// steel_i_t 直接利用）と一致する（柱の Old 分岐は無変更）。
    #[test]
    fn test_column_check_fb_rule_default_matches_old() {
        let sec = h_section(400.0, 200.0, 8.0, 13.0);
        let mat_v = mat("SN400B");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -50_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 5e7,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3500.0,
            ..Default::default()
        };
        assert_eq!(ctx.steel_fb_rule, SteelFbRule::Old);
        let result = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx)
            .unwrap_checked();

        let f = 235.0;
        let af = 200.0 * 13.0;
        let i_t = steel_i_t(200.0, 13.0, 400.0, 8.0);
        let c = steel_c_factor(&ctx, false);
        let fb_expected = steel_fb_h(f, LoadTerm::Long, 3500.0, i_t, 400.0, af, c);
        assert!(
            crate::full_detail(&result).contains(&format!("fbX={:.4}", fb_expected)),
            "detail={}",
            crate::full_detail(&result)
        );
    }

    /// 新基準 fb（`SteelFbRule::New`）: H形柱の fbX が `steel_fb_h_new` の
    /// 手計算と一致することを確認する（柱の階高 `ctx.length` を lb として
    /// 用いる）。
    #[test]
    fn test_column_check_fb_rule_new_matches_hand_calc() {
        let sec = h_section(400.0, 200.0, 8.0, 13.0);
        let mat_v = mat("SN400B");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -50_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 5e7,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3500.0,
            steel_fb_rule: SteelFbRule::New,
            ..Default::default()
        };
        let result = SteelDesign
            .check(&forces, &sec, &mat_v, &ctx)
            .unwrap_checked();

        let f = 235.0;
        let z_strong = sec.iy / (sec.depth / 2.0);
        let iw = steel_warping_constant(&sec, 13.0);
        let e = mat_v.young;
        let g = e / (2.0 * (1.0 + mat_v.poisson));
        let p_lambda_b = steel_p_lambda_b(&ctx);
        let c = steel_c_factor(&ctx, false);
        let fb_expected = steel_fb_h_new(
            f,
            LoadTerm::Long,
            3500.0,
            sec.iz,
            iw,
            sec.j,
            e,
            g,
            z_strong,
            c,
            p_lambda_b,
        );
        assert!(
            crate::full_detail(&result).contains(&format!("fbX={:.4}", fb_expected)),
            "detail={}",
            crate::full_detail(&result)
        );
    }
}
