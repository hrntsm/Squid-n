//! 部材群としての種別と、各階の構造特性係数 Ds の告示表（昭55建告1792号）。
//!
//! - [`GroupType`] — 部材群としての種別 A/B/C/D
//! - [`member_group`] — 耐力比 γA/γC による部材群種別の判定
//! - [`rc_member_type`] / [`rc_beam_type`] — RC 柱・はりの部材種別（多変数表）
//! - [`rc_wall_type`] — RC 耐力壁の種別（WA〜WD）
//! - [`rc_wall_tau_over_fc`] / [`rc_wall_shear_brittle`] — τu/Fc と WD 判定の補助
//! - [`steel_brace_type`] — 鉄骨筋かいの種別（BA〜BC）
//! - [`ds_rc`] / [`ds_steel`] — 各階の Ds（壁／筋かい群種別 × βu × 柱はり群種別）

use super::holding_capacity::MemberRank;

/// 部材群としての種別（告示「部材群としての種別」表）。
///
/// 柱及びはりの部材群としての種別は A〜D、耐力壁・筋かいの部材群としての種別は A〜C。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum GroupType {
    A,
    B,
    C,
    /// 種別 D 部材（FD／WD）を含む場合。告示の「部材群としての種別」表は A〜C のみを
    /// 定義し D の判定条件を与えていないため、本実装では**種別 D の部材を含む層は
    /// 部材群としての種別を D とする**解釈を採る（要・原典照合。安全側）。
    D,
}

/// 部材群としての種別を耐力比 γA・γC から判定する（告示「部材群としての種別」表）。
///
/// `members` は層に属する部材の `(種別インデックス, 水平耐力)` の並び。種別インデックスは
/// 0=A(FA/WA/BA), 1=B, 2=C, 3=D(FD/WD) とする（筋かいは BA/BB/BC の 3 種別のみ）。
///
/// ```text
/// γA = Σ(種別Aの部材の耐力) / Σ(種別Dを除くすべての部材の水平耐力)
/// γC = Σ(種別Cの部材の耐力) / Σ(種別Dを除くすべての部材の水平耐力)
/// (1) γA ≧ 0.5 かつ γC ≦ 0.2          → A
/// (2) γC < 0.5（種別が A の場合を除く） → B
/// (3) γC ≧ 0.5                         → C
/// ```
///
/// 種別 D の部材を 1 つでも含む場合は [`GroupType::D`] を返す（上記の解釈）。
/// 対象部材がない、または種別 D を除く耐力の総和が 0 の場合は `None`。
pub fn member_group(members: &[(u8, f64)]) -> Option<GroupType> {
    if members.is_empty() {
        return None;
    }
    if members.iter().any(|(idx, _)| *idx >= 3) {
        return Some(GroupType::D);
    }
    let denom: f64 = members.iter().map(|(_, q)| *q).sum();
    if denom <= 0.0 {
        return None;
    }
    let sum_of = |target: u8| -> f64 {
        members
            .iter()
            .filter(|(idx, _)| *idx == target)
            .map(|(_, q)| *q)
            .sum()
    };
    let gamma_a = sum_of(0) / denom;
    let gamma_c = sum_of(2) / denom;
    if gamma_a >= 0.5 && gamma_c <= 0.2 {
        Some(GroupType::A)
    } else if gamma_c < 0.5 {
        Some(GroupType::B)
    } else {
        Some(GroupType::C)
    }
}

/// [`MemberRank`] を [`member_group`] の種別インデックス（0=A..3=D）へ変換する。
pub fn rank_index_for_group(rank: MemberRank) -> u8 {
    match rank {
        MemberRank::FA => 0,
        MemberRank::FB => 1,
        MemberRank::FC => 2,
        MemberRank::FD => 3,
    }
}

/// βu（耐力壁／筋かいの水平耐力の和を保有水平耐力で除した数値）の区分。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BetaBand {
    /// βu = 0（耐力壁・筋かいなし）
    Zero,
    /// 0 < βu ≦ 0.3
    Low,
    /// 0.3 < βu ≦ 0.7
    Mid,
    /// βu > 0.7
    High,
}

fn beta_band(beta_u: f64) -> BetaBand {
    if beta_u <= 0.0 {
        BetaBand::Zero
    } else if beta_u <= 0.3 {
        BetaBand::Low
    } else if beta_u <= 0.7 {
        BetaBand::Mid
    } else {
        BetaBand::High
    }
}

/// 柱及びはりの部材群としての種別を列インデックス（A=0..D=3）へ変換する。
fn group_col(g: GroupType) -> usize {
    match g {
        GroupType::A => 0,
        GroupType::B => 1,
        GroupType::C => 2,
        GroupType::D => 3,
    }
}

/// RC 造の各階の構造特性係数 Ds（告示「各階の Ds」表）。
///
/// `wall_group`: 耐力壁の部材群としての種別、`beta_u`: 耐力壁の水平耐力の和を保有水平
/// 耐力で除した数値、`cb_group`: 柱及びはりの部材群としての種別。
///
/// 耐力壁がない（βu=0）場合は純ラーメンとして A 行相当の 0.30/0.35/0.40/0.45 を用いる。
pub fn ds_rc(wall_group: GroupType, beta_u: f64, cb_group: GroupType) -> f64 {
    let col = group_col(cb_group);
    // 耐力壁なし（βu=0）: RC ラーメンの Ds。
    let row: [f64; 4] = match (wall_group, beta_band(beta_u)) {
        (_, BetaBand::Zero) => [0.30, 0.35, 0.40, 0.45],
        (GroupType::A, BetaBand::Low) => [0.30, 0.35, 0.40, 0.45],
        (GroupType::A, BetaBand::Mid) => [0.35, 0.40, 0.45, 0.50],
        (GroupType::A, BetaBand::High) => [0.40, 0.45, 0.45, 0.55],
        (GroupType::B, BetaBand::Low) => [0.35, 0.35, 0.40, 0.45],
        (GroupType::B, BetaBand::Mid) => [0.40, 0.40, 0.45, 0.50],
        (GroupType::B, BetaBand::High) => [0.45, 0.45, 0.50, 0.55],
        (GroupType::C, BetaBand::Low) => [0.35, 0.35, 0.40, 0.45],
        (GroupType::C, BetaBand::Mid) => [0.40, 0.45, 0.45, 0.50],
        (GroupType::C, BetaBand::High) => [0.50, 0.50, 0.50, 0.55],
        (GroupType::D, BetaBand::Low) => [0.40, 0.40, 0.45, 0.45],
        (GroupType::D, BetaBand::Mid) => [0.45, 0.50, 0.50, 0.50],
        (GroupType::D, BetaBand::High) => [0.55, 0.55, 0.55, 0.55],
    };
    row[col]
}

/// 鉄骨造の各階の構造特性係数 Ds（告示「各階の構造特性係数 Ds」表）。
///
/// `brace_group`: 筋かいの部材群としての種別、`beta_u`: 筋かい（耐力壁を含む）の水平
/// 耐力の和を保有水平耐力で除した数値、`cb_group`: 柱及びはりの部材群としての種別。
///
/// 筋かい群種別 A 又は βu=0 の場合は 0.25/0.30/0.35/0.40。
pub fn ds_steel(brace_group: GroupType, beta_u: f64, cb_group: GroupType) -> f64 {
    let col = group_col(cb_group);
    let row: [f64; 4] = match (brace_group, beta_band(beta_u)) {
        // 「A 又は βu=0 の場合」
        (GroupType::A, _) | (_, BetaBand::Zero) => [0.25, 0.30, 0.35, 0.40],
        (GroupType::B, BetaBand::Low) => [0.25, 0.30, 0.35, 0.40],
        (GroupType::B, BetaBand::Mid) => [0.30, 0.30, 0.35, 0.45],
        (GroupType::B, BetaBand::High) => [0.35, 0.35, 0.40, 0.50],
        // 筋かい群 C は βu ≦ 0.3 / 0.3 < βu ≦ 0.5 / βu > 0.5 の 3 区分。
        (GroupType::C, BetaBand::Low) => [0.30, 0.30, 0.35, 0.40],
        (GroupType::C, BetaBand::Mid) | (GroupType::C, BetaBand::High) => {
            if beta_u <= 0.5 {
                [0.35, 0.35, 0.40, 0.45]
            } else {
                [0.40, 0.40, 0.45, 0.50]
            }
        }
        // 筋かいに種別 D はない（BA/BB/BC）。安全側に C の最不利行を用いる。
        (GroupType::D, _) => [0.40, 0.40, 0.45, 0.50],
    };
    row[col]
}

/// RC 柱の部材種別（告示「柱及びはりの部材種別」表の柱の列）。
///
/// ```text
///        h0/D    σ0/Fc   pt      τu/Fc
/// FA:  2.5以上  0.35以下 0.8以下  0.1以下
/// FB:  2.0以上  0.45以下 1.0以下  0.125以下
/// FC:    —      0.55以下   —      0.15以下
/// FD: FA,FB又はFCのいずれにも該当しない場合
/// ```
///
/// - `h0_over_d`: 柱の内法高さ h0 を加力方向の断面せい D で除した値。
/// - `sigma0_over_fc`: Ds 算定時に柱の断面に生じる軸方向応力度 σ0 をコンクリートの
///   設計基準強度 Fc で除した値。
/// - `pt_percent`: 引張鉄筋比 pt \[%\]。
/// - `tau_over_fc`: Ds 算定時に柱の断面に生じる平均せん断応力度 τu を Fc で除した値。
/// - `brittle`: せん断破壊・付着割裂破壊・圧縮破壊等の急激な耐力低下を生じる場合は
///   `true`（この場合は無条件で FD）。
pub fn rc_column_type(
    h0_over_d: f64,
    sigma0_over_fc: f64,
    pt_percent: f64,
    tau_over_fc: f64,
    brittle: bool,
) -> MemberRank {
    if brittle {
        return MemberRank::FD;
    }
    if h0_over_d >= 2.5 && sigma0_over_fc <= 0.35 && pt_percent <= 0.8 && tau_over_fc <= 0.1 {
        MemberRank::FA
    } else if h0_over_d >= 2.0
        && sigma0_over_fc <= 0.45
        && pt_percent <= 1.0
        && tau_over_fc <= 0.125
    {
        MemberRank::FB
    } else if sigma0_over_fc <= 0.55 && tau_over_fc <= 0.15 {
        MemberRank::FC
    } else {
        MemberRank::FD
    }
}

/// RC はりの部材種別（告示「柱及びはりの部材種別」表のはりの列）。
///
/// ```text
/// FA: τu/Fc ≦ 0.15
/// FB: τu/Fc ≦ 0.2
/// FC: 上記以外（τu/Fc に制限なし）
/// FD: 急激な耐力低下を生じる破壊のおそれがある場合
/// ```
pub fn rc_beam_type(tau_over_fc: f64, brittle: bool) -> MemberRank {
    if brittle {
        return MemberRank::FD;
    }
    if tau_over_fc <= 0.15 {
        MemberRank::FA
    } else if tau_over_fc <= 0.2 {
        MemberRank::FB
    } else {
        MemberRank::FC
    }
}

/// RC 耐力壁のせん断頭打ち（脆性破壊）判定閾値。終局時負担水平力が Qu のこの比率以上なら WD。
pub const RC_WALL_SHEAR_BRITTLE_RATIO: f64 = 0.99;

/// RC 耐力壁のせん断頭打ち到達（脆性）判定。
///
/// 増分解析の壁要素は面内せん断を終局せん断強度 Qu で頭打ちにする弾完全塑性のため、
/// 終局時の負担水平力が Qu に達していれば「せん断破壊」とみなす。
/// Qu が算定不能（0 以下）のときは false（判定不能を WD としない）。
pub fn rc_wall_shear_brittle(horizontal_force: f64, qu: f64) -> bool {
    qu > 0.0 && horizontal_force >= RC_WALL_SHEAR_BRITTLE_RATIO * qu
}

/// RC 耐力壁の平均せん断応力度比 τu/Fc。
///
/// `thickness`・`wall_len` から壁板断面積 t·lw [mm²] を組み立て、耐力用開口低減 r2 を
/// 乗じた有効断面で算定する。有効断面または fc が 0 以下のときは None（判定不能）。
pub fn rc_wall_tau_over_fc(
    horizontal_force: f64,
    thickness: f64,
    wall_len: f64,
    r2: f64,
    fc: f64,
) -> Option<f64> {
    let area = thickness * wall_len * r2;
    if area <= 0.0 || fc <= 0.0 {
        return None;
    }
    Some((horizontal_force / area) / fc)
}

/// RC 耐力壁の種別（告示「耐力壁の種別」表）。WA/WB/WC/WD を
/// [`MemberRank`] の FA/FB/FC/FD に対応させて返す。
///
/// ```text
///                 壁式構造以外   壁式構造
/// WA: τu/Fc ≦      0.20          0.1
/// WB: τu/Fc ≦      0.25          0.125
/// WC: τu/Fc ≦       —            0.15
/// WD: WA,WB,WC のいずれにも該当しない場合
/// ```
///
/// `wall_structure` が真のとき壁式構造の列を用いる。`brittle`（せん断破壊等の急激な
/// 耐力低下を生じる破壊）が真なら WD。
pub fn rc_wall_type(tau_over_fc: f64, wall_structure: bool, brittle: bool) -> MemberRank {
    if brittle {
        return MemberRank::FD;
    }
    if wall_structure {
        if tau_over_fc <= 0.1 {
            MemberRank::FA
        } else if tau_over_fc <= 0.125 {
            MemberRank::FB
        } else if tau_over_fc <= 0.15 {
            MemberRank::FC
        } else {
            MemberRank::FD
        }
    } else if tau_over_fc <= 0.20 {
        MemberRank::FA
    } else if tau_over_fc <= 0.25 {
        MemberRank::FB
    } else {
        // 壁式構造以外の WC 欄は「—」（制限なし）のため、WB を満たさないものは WC。
        MemberRank::FC
    }
}

/// 鉄骨筋かいの種別（告示「筋かいの種別」表）。BA/BB/BC を
/// [`MemberRank`] の FA/FB/FC に対応させて返す。
///
/// ```text
/// BA: λ ≦ 495/√F
/// BB: 495/√F < λ ≦ 890/√F 又は 1980/√F ≦ λ
/// BC: 890/√F < λ < 1980/√F
/// ```
///
/// `lambda`: 筋かいの有効細長比、`f_value`: 基準強度 F \[N/mm²\]（0 以下は 235）。
pub fn steel_brace_type(lambda: f64, f_value: f64) -> MemberRank {
    let f = if f_value <= 0.0 { 235.0 } else { f_value };
    let sqrt_f = f.sqrt();
    let b1 = 495.0 / sqrt_f;
    let b2 = 890.0 / sqrt_f;
    let b3 = 1980.0 / sqrt_f;
    if lambda <= b1 {
        MemberRank::FA
    } else if lambda <= b2 || lambda >= b3 {
        MemberRank::FB
    } else {
        MemberRank::FC
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ===== 部材群としての種別 =====

    #[test]
    fn test_member_group_a() {
        // γA = 60/100 = 0.6 ≧ 0.5、γC = 10/100 = 0.1 ≦ 0.2 → A
        let members = [(0u8, 60.0), (1u8, 30.0), (2u8, 10.0)];
        assert_eq!(member_group(&members), Some(GroupType::A));
    }

    #[test]
    fn test_member_group_b_when_gamma_a_insufficient() {
        // γA = 0.4 < 0.5 → A ではない。γC = 0.1 < 0.5 → B
        let members = [(0u8, 40.0), (1u8, 50.0), (2u8, 10.0)];
        assert_eq!(member_group(&members), Some(GroupType::B));
    }

    #[test]
    fn test_member_group_b_when_gamma_c_over_02() {
        // γA = 0.6 ≧ 0.5 だが γC = 0.3 > 0.2 → A ではない → γC < 0.5 なので B
        let members = [(0u8, 60.0), (2u8, 30.0), (1u8, 10.0)];
        assert_eq!(member_group(&members), Some(GroupType::B));
    }

    #[test]
    fn test_member_group_c() {
        // γC = 0.5 ≧ 0.5 → C
        let members = [(0u8, 30.0), (2u8, 50.0), (1u8, 20.0)];
        assert_eq!(member_group(&members), Some(GroupType::C));
    }

    /// 種別 D（FD/WD）の部材を含む層は D（本実装の解釈、安全側）。
    #[test]
    fn test_member_group_d_when_any_d_member() {
        let members = [(0u8, 90.0), (3u8, 10.0)];
        assert_eq!(member_group(&members), Some(GroupType::D));
    }

    #[test]
    fn test_member_group_empty_is_none() {
        assert_eq!(member_group(&[]), None);
    }

    // ===== RC 柱・はりの部材種別 =====

    #[test]
    fn test_rc_column_type_fa_boundary() {
        // 全項目が FA の境界値ちょうど → FA
        assert_eq!(rc_column_type(2.5, 0.35, 0.8, 0.1, false), MemberRank::FA);
    }

    #[test]
    fn test_rc_column_type_falls_to_fb_when_one_item_exceeds() {
        // τu/Fc だけ FA を超える → FB
        assert_eq!(rc_column_type(2.5, 0.35, 0.8, 0.11, false), MemberRank::FB);
        // h0/D だけ FA 未満 → FB
        assert_eq!(rc_column_type(2.4, 0.35, 0.8, 0.1, false), MemberRank::FB);
    }

    #[test]
    fn test_rc_column_type_fc() {
        // h0/D=1.5（FB 未満）だが σ0/Fc≦0.55 かつ τu/Fc≦0.15 → FC
        assert_eq!(rc_column_type(1.5, 0.5, 1.5, 0.14, false), MemberRank::FC);
    }

    #[test]
    fn test_rc_column_type_fd() {
        // τu/Fc が FC 限界超え → FD
        assert_eq!(rc_column_type(3.0, 0.3, 0.5, 0.16, false), MemberRank::FD);
        // 軸力比が FC 限界超え → FD
        assert_eq!(rc_column_type(3.0, 0.6, 0.5, 0.05, false), MemberRank::FD);
    }

    /// 脆性破壊（せん断破壊・付着割裂等）は無条件で FD。
    #[test]
    fn test_rc_column_type_brittle_is_fd() {
        assert_eq!(rc_column_type(3.0, 0.1, 0.5, 0.05, true), MemberRank::FD);
    }

    #[test]
    fn test_rc_beam_type() {
        assert_eq!(rc_beam_type(0.15, false), MemberRank::FA);
        assert_eq!(rc_beam_type(0.151, false), MemberRank::FB);
        assert_eq!(rc_beam_type(0.2, false), MemberRank::FB);
        assert_eq!(rc_beam_type(0.3, false), MemberRank::FC);
        assert_eq!(rc_beam_type(0.05, true), MemberRank::FD);
    }

    // ===== 耐力壁の種別 =====

    #[test]
    fn test_rc_wall_type_non_wall_structure() {
        assert_eq!(rc_wall_type(0.20, false, false), MemberRank::FA);
        assert_eq!(rc_wall_type(0.25, false, false), MemberRank::FB);
        assert_eq!(rc_wall_type(0.30, false, false), MemberRank::FC);
    }

    #[test]
    fn test_rc_wall_type_wall_structure() {
        assert_eq!(rc_wall_type(0.1, true, false), MemberRank::FA);
        assert_eq!(rc_wall_type(0.125, true, false), MemberRank::FB);
        assert_eq!(rc_wall_type(0.15, true, false), MemberRank::FC);
        assert_eq!(rc_wall_type(0.16, true, false), MemberRank::FD);
    }

    #[test]
    fn test_rc_wall_shear_brittle() {
        let qu = 1_000_000.0;
        assert!(!rc_wall_shear_brittle(0.98 * qu, qu));
        assert!(rc_wall_shear_brittle(RC_WALL_SHEAR_BRITTLE_RATIO * qu, qu));
        assert!(rc_wall_shear_brittle(qu, qu));
        assert!(!rc_wall_shear_brittle(qu, 0.0));
        // せん断頭打ち到達時は τu に関わらず WD
        assert_eq!(rc_wall_type(0.20, false, true), MemberRank::FD);
    }

    #[test]
    fn test_rc_wall_tau_over_fc_with_r2() {
        let h = 1_000_000.0;
        let t = 150.0;
        let lw = 4000.0;
        let fc = 24.0;
        let tau_no = rc_wall_tau_over_fc(h, t, lw, 1.0, fc).unwrap();
        let tau_r2 = rc_wall_tau_over_fc(h, t, lw, 0.5, fc).unwrap();
        assert!((tau_r2 - 2.0 * tau_no).abs() < 1e-9);
        assert!(rc_wall_tau_over_fc(h, t, lw, 0.0, fc).is_none());
    }

    // ===== 筋かいの種別 =====

    #[test]
    fn test_steel_brace_type_f235() {
        // F=235 → √F≈15.33、495/√F≈32.3、890/√F≈58.1、1980/√F≈129.2
        assert_eq!(steel_brace_type(30.0, 235.0), MemberRank::FA);
        assert_eq!(steel_brace_type(50.0, 235.0), MemberRank::FB);
        assert_eq!(steel_brace_type(100.0, 235.0), MemberRank::FC);
        // λ ≧ 1980/√F は BB（極めて細長い筋かいは座屈後も安定）
        assert_eq!(steel_brace_type(130.0, 235.0), MemberRank::FB);
    }

    // ===== Ds 表（RC） =====

    /// 耐力壁なし（βu=0）は RC ラーメンの Ds。
    #[test]
    fn test_ds_rc_no_wall_matches_frame_row() {
        for (g, expected) in [
            (GroupType::A, 0.30),
            (GroupType::B, 0.35),
            (GroupType::C, 0.40),
            (GroupType::D, 0.45),
        ] {
            assert!((ds_rc(GroupType::A, 0.0, g) - expected).abs() < 1e-9);
        }
    }

    /// 告示表の代表値を直接照合する。
    #[test]
    fn test_ds_rc_table_values() {
        // 壁群A, 0<βu≦0.3: 0.3/0.35/0.4/0.45
        assert!((ds_rc(GroupType::A, 0.2, GroupType::A) - 0.30).abs() < 1e-9);
        assert!((ds_rc(GroupType::A, 0.2, GroupType::D) - 0.45).abs() < 1e-9);
        // 壁群A, βu>0.7: 0.4/0.45/0.45/0.55
        assert!((ds_rc(GroupType::A, 0.8, GroupType::A) - 0.40).abs() < 1e-9);
        assert!((ds_rc(GroupType::A, 0.8, GroupType::C) - 0.45).abs() < 1e-9);
        assert!((ds_rc(GroupType::A, 0.8, GroupType::D) - 0.55).abs() < 1e-9);
        // 壁群C, βu>0.7: 0.5/0.5/0.5/0.55
        assert!((ds_rc(GroupType::C, 0.8, GroupType::A) - 0.50).abs() < 1e-9);
        assert!((ds_rc(GroupType::C, 0.8, GroupType::D) - 0.55).abs() < 1e-9);
        // 壁群D, βu>0.7: 全て 0.55
        for g in [GroupType::A, GroupType::B, GroupType::C, GroupType::D] {
            assert!((ds_rc(GroupType::D, 0.8, g) - 0.55).abs() < 1e-9);
        }
        // 壁群D, 0.3<βu≦0.7: 0.45/0.5/0.5/0.5
        assert!((ds_rc(GroupType::D, 0.5, GroupType::A) - 0.45).abs() < 1e-9);
        assert!((ds_rc(GroupType::D, 0.5, GroupType::B) - 0.50).abs() < 1e-9);
    }

    // ===== Ds 表（S） =====

    /// 筋かいなし（βu=0）／筋かい群 A は 0.25/0.3/0.35/0.4。
    #[test]
    fn test_ds_steel_no_brace_row() {
        for (g, expected) in [
            (GroupType::A, 0.25),
            (GroupType::B, 0.30),
            (GroupType::C, 0.35),
            (GroupType::D, 0.40),
        ] {
            assert!((ds_steel(GroupType::A, 0.0, g) - expected).abs() < 1e-9);
            assert!((ds_steel(GroupType::B, 0.0, g) - expected).abs() < 1e-9);
            // 筋かい群 A は βu によらず同じ行。
            assert!((ds_steel(GroupType::A, 0.9, g) - expected).abs() < 1e-9);
        }
    }

    /// 告示表の代表値を直接照合する（従来の単一行実装が過小評価していた領域を含む）。
    #[test]
    fn test_ds_steel_table_values() {
        // 筋かい群B, 0.3<βu≦0.7: 0.3/0.3/0.35/0.45
        assert!((ds_steel(GroupType::B, 0.5, GroupType::A) - 0.30).abs() < 1e-9);
        assert!((ds_steel(GroupType::B, 0.5, GroupType::D) - 0.45).abs() < 1e-9);
        // 筋かい群B, βu>0.7: 0.35/0.35/0.4/0.5（旧実装は柱はり群A で 0.30 と過小）
        assert!((ds_steel(GroupType::B, 0.8, GroupType::A) - 0.35).abs() < 1e-9);
        assert!((ds_steel(GroupType::B, 0.8, GroupType::D) - 0.50).abs() < 1e-9);
        // 筋かい群C, 0<βu≦0.3: 0.3/0.3/0.35/0.4
        assert!((ds_steel(GroupType::C, 0.2, GroupType::A) - 0.30).abs() < 1e-9);
        // 筋かい群C, 0.3<βu≦0.5: 0.35/0.35/0.4/0.45
        assert!((ds_steel(GroupType::C, 0.4, GroupType::A) - 0.35).abs() < 1e-9);
        // 筋かい群C, βu>0.5: 0.4/0.4/0.45/0.5（旧実装は柱はり群A で 0.30 と大幅過小）
        assert!((ds_steel(GroupType::C, 0.6, GroupType::A) - 0.40).abs() < 1e-9);
        assert!((ds_steel(GroupType::C, 0.6, GroupType::C) - 0.45).abs() < 1e-9);
        assert!((ds_steel(GroupType::C, 0.6, GroupType::D) - 0.50).abs() < 1e-9);
    }

    /// βu の境界値（0.3 / 0.5 / 0.7）で行が切り替わる。
    #[test]
    fn test_beta_u_band_boundaries() {
        // RC: βu=0.3 は Low、0.31 は Mid。
        assert!((ds_rc(GroupType::A, 0.3, GroupType::A) - 0.30).abs() < 1e-9);
        assert!((ds_rc(GroupType::A, 0.31, GroupType::A) - 0.35).abs() < 1e-9);
        // RC: βu=0.7 は Mid、0.71 は High。
        assert!((ds_rc(GroupType::A, 0.7, GroupType::A) - 0.35).abs() < 1e-9);
        assert!((ds_rc(GroupType::A, 0.71, GroupType::A) - 0.40).abs() < 1e-9);
        // S 筋かい群C: βu=0.5 は 0.35 行、0.51 は 0.4 行。
        assert!((ds_steel(GroupType::C, 0.5, GroupType::A) - 0.35).abs() < 1e-9);
        assert!((ds_steel(GroupType::C, 0.51, GroupType::A) - 0.40).abs() < 1e-9);
    }
}
