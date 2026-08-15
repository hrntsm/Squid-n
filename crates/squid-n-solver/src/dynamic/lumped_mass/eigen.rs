//! 質点系（串団子）の固有値解析（せん断型多質点系、構造力学）。
//!
//! [`LumpedMassModel`] の初期剛性 K1・質量から、せん断型の一般化固有値問題
//! \\( K x = \omega^2 M x \\) を解く。M は対角（質量マトリクス）なので
//! \\( A = M^{-1/2} K M^{-1/2} \\) の標準固有値問題へ帰着し、faer の対称行列
//! 固有値分解（`self_adjoint_eigen`）で解く（層数は通常数層〜数十層と小さく、
//! 密行列の直接解法で十分）。
//!
//! - [`LumpedMassModal`] — 固有値解析の結果（周期・モード形状）。
//! - [`lumped_mass_eigen`] — 串団子モデルの固有値解析。
//! - [`stick_omega1`] — 減衰算定用の1次固有円振動数（内部専用、フォールバックあり）。

use super::model::LumpedMassModel;
use faer::Side;
use squid_n_math::solver::SolveError;

/// 串団子モデルの固有値解析結果。
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct LumpedMassModal {
    /// 固有値 ω²（昇順、1次から）。
    pub omega2: Vec<f64>,
    /// 固有周期 T=2π/ω [s]（昇順、1次から）。
    pub period: Vec<f64>,
    /// モード形状（各層の相対変位、下層→上層。最上階の値を 1.0 に正規化）。
    /// 2 次元は加力方向変位。3 次元は頂部水平ノルムで正規化した水平合成。
    pub shapes: Vec<Vec<f64>>,
    /// 3 次元のモード形状（各層 [Ux, Uy, θz]。2 次元では空）。
    #[serde(default)]
    pub shapes_xyz: Vec<Vec<[f64; 3]>>,
}

/// 串団子モデルの固有値解析（せん断型多質点系）。
/// `n_modes` が層数を超える場合、返るモード数は層数まで切り詰められる
/// （立体モデルの [`crate::eigen::solve_eigen`] と同じ規約。呼び出し側は
/// 返り値の長さを確認すること）。
///
/// 層に質量が 0 以下のものがあると固有値解析が意味を持たない（実体のない
/// 極端な高周波モードが紛れ込む）ため、`SolveError::InvalidInput` を返す。
pub fn lumped_mass_eigen(
    lm: &LumpedMassModel,
    n_modes: usize,
) -> Result<LumpedMassModal, SolveError> {
    let n = lm.stories.len();
    if n == 0 || n_modes == 0 {
        return Ok(LumpedMassModal::default());
    }
    if let Some(bad) = lm.stories.iter().find(|s| s.mass <= 0.0) {
        return Err(SolveError::InvalidInput(format!(
            "層 {:?} の質量が 0 以下のため固有値解析できません",
            bad.story
        )));
    }

    if lm.is_spatial() {
        return super::spatial::lumped_mass_eigen_spatial(lm, n_modes);
    }

    let mass: Vec<f64> = lm.stories.iter().map(|s| s.mass).collect();
    let k1: Vec<f64> = lm.stories.iter().map(|s| s.skeleton.k1.max(0.0)).collect();
    eigen_from_arrays(&mass, &k1, n_modes)
}

/// せん断型多質点系の固有値解析の計算コア（質量・初期剛性の配列を直接受け取る）。
/// `mass` は全要素が正であることを呼び出し側が保証すること（0 以下は
/// `sqrt_m` を経て NaN／発散を生む）。
fn eigen_from_arrays(
    mass: &[f64],
    k1: &[f64],
    n_modes: usize,
) -> Result<LumpedMassModal, SolveError> {
    let n = mass.len();
    let sqrt_m: Vec<f64> = mass.iter().map(|&m| m.sqrt()).collect();

    // 標準固有値問題 A = M^{-1/2} K M^{-1/2} を組み立てる。K はせん断型三重対角
    // （K[i][i]=k_i+k_{i+1}、副対角=-k_{i+1}、k_{n}=0）。
    let a = faer::Mat::from_fn(n, n, |i, j| {
        let k_ij = if i == j {
            let ki1 = if i + 1 < n { k1[i + 1] } else { 0.0 };
            k1[i] + ki1
        } else if i + 1 == j || j + 1 == i {
            -k1[i.max(j)]
        } else {
            0.0
        };
        k_ij / (sqrt_m[i] * sqrt_m[j])
    });

    let eig = a
        .self_adjoint_eigen(Side::Lower)
        .map_err(|e| SolveError::NonConvergence(format!("串団子モデルの固有値分解: {e:?}")))?;
    let s = eig.S();
    let u = eig.U();

    // faer は昇順で返すが、丸め誤差で微小負になり得るため 0 にクランプする。
    let take = n_modes.min(n);
    let mut omega2 = Vec::with_capacity(take);
    let mut period = Vec::with_capacity(take);
    let mut shapes = Vec::with_capacity(take);
    for j in 0..take {
        let w2 = s[j].max(0.0);
        let w = w2.sqrt();
        omega2.push(w2);
        period.push(if w > 0.0 {
            2.0 * std::f64::consts::PI / w
        } else {
            f64::INFINITY
        });

        // 物理座標のモード形状 x = M^{-1/2} y。最上階（配列末尾）を 1.0 に正規化。
        let mut shape: Vec<f64> = (0..n).map(|i| u[(i, j)] / sqrt_m[i]).collect();
        let top = shape[n - 1];
        if top.abs() > 1e-30 {
            for v in shape.iter_mut() {
                *v /= top;
            }
        }
        shapes.push(shape);
    }
    Ok(LumpedMassModal {
        omega2,
        period,
        shapes,
        shapes_xyz: Vec::new(),
    })
}

/// 減衰算定用の1次固有円振動数 ω1。
///
/// 質量が 0 以下の層があると、そのまま `.max(1e-9)` で丸めて解いた場合
/// ω1 が桁違いに大きくなり、`a1=2h/ω1` が実質ゼロへ潰れて**無音で無減衰**に
/// なる（サイレントな非安全側の破綻。初期剛性比例ではなく接線剛性比例に
/// なっていた過去の減衰バグと同じ失敗形）。これを避けるため、質量 0 以下の
/// 層は他層の正の質量の平均で置き換えてから固有値分解する（表示用の
/// [`lumped_mass_eigen`] はこの補正をせず、質量 0 以下をそのままエラーとして
/// 利用者へ見せる。この関数は減衰算定専用の内部フォールバック）。
///
/// 全層の質量が 0 以下、固有値分解自体が収束しない、または 1 次モードが
/// 特異（ω²=0。層剛性 K1=0 の層がある場合等）のときのみ、旧来の逆反復法
/// （[`super::time_history::fundamental_omega`]）へフォールバックする。
pub(crate) fn stick_omega1(lm: &LumpedMassModel) -> f64 {
    if lm.is_spatial() {
        if let Ok(modal) = super::spatial::lumped_mass_eigen_spatial(lm, 1) {
            if let Some(&w2) = modal.omega2.first() {
                if w2 > 0.0 {
                    return w2.sqrt();
                }
            }
        }
    }

    let mass: Vec<f64> = lm.stories.iter().map(|s| s.mass).collect();
    let k1: Vec<f64> = lm.stories.iter().map(|s| s.skeleton.k1.max(0.0)).collect();

    let (pos_sum, pos_count) = mass
        .iter()
        .filter(|&&m| m > 0.0)
        .fold((0.0, 0usize), |(s, c), &m| (s + m, c + 1));
    if pos_count > 0 {
        let substitute = pos_sum / pos_count as f64;
        let repaired_mass: Vec<f64> = mass
            .iter()
            .map(|&m| if m > 0.0 { m } else { substitute })
            .collect();
        if let Ok(modal) = eigen_from_arrays(&repaired_mass, &k1, 1) {
            if let Some(&w2) = modal.omega2.first() {
                // w2=0（層剛性 K1=0 の層がある等）は 1 次モードが特異で
                // ω1=0 になり、呼び出し側で a1=2h/ω1 が無条件にゼロへ潰れて
                // 無音無減衰になる。質量側の補修と同じ理由で、ここも下の
                // クランプ付きフォールバックへ逃がす（数値ノイズで負に転じた
                // 分は eigen_from_arrays 側で既に 0 にクランプ済みのため、
                // >0.0 判定で「真に特異」と「桁落ちで 0 未満」の両方を拾える）。
                if w2 > 0.0 {
                    return w2.sqrt();
                }
            }
        }
    }

    // 全層の質量が 0 以下、固有値分解が収束しなかった、または 1 次モードが
    // 特異だった場合のみ到達する。
    let mass_clamped: Vec<f64> = mass.iter().map(|&m| m.max(1e-9)).collect();
    let k1_clamped: Vec<f64> = k1.iter().map(|&k| k.max(1e-9)).collect();
    super::time_history::fundamental_omega(&mass_clamped, &k1_clamped)
}
