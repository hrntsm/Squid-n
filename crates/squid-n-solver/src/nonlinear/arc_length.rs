/// 円筒型弧長ステップの結果
pub struct ArcLengthStep {
    pub du: Vec<f64>,
    pub dlambda: f64,
    pub converged: bool,
}

pub type SolverFn<'a> = dyn FnMut(&[f64]) -> Result<Vec<f64>, String> + 'a;

/// 変位増分 δu を要素状態へ反映し、更新後の内力ベクトルを返すクロージャ型。
pub type FintFn<'a> = dyn FnMut(&[f64]) -> Result<Vec<f64>, String> + 'a;

/// 円筒型弧長法ソルバ（Crisfield 1981）
pub struct ArcLengthSolver {
    pub delta_l: f64,
    pub max_iter: u32,
    pub tol: f64,
}

impl ArcLengthSolver {
    pub fn new(delta_l: f64) -> Self {
        ArcLengthSolver {
            delta_l,
            max_iter: 20,
            tol: 1e-6,
        }
    }

    /// 1ステップの弧長制御（予測子＋修正子反復）
    ///
    /// `q`: 参照荷重ベクトル
    /// `solve`: K⁻¹·r を返す線形ソルバクロージャ（修正 Newton＝接線はステップ開始時で固定）
    /// `eval_fint`: 変位増分 δu を要素状態へ反映し、更新後の内力ベクトルを返すクロージャ。
    ///   修正子の各反復で内力 f_int(u) を再評価することで真の非線形反復を行う
    ///   （旧実装は f_int を固定パラメータとして渡しており、非線形反復になっていなかった）。
    /// `prev_du`: 前ステップの変位増分（符号決定・根選択用）
    ///
    /// `solve`／`eval_fint` から返る `du_t`／`du_bar`／`f_int` は呼び出し元クロージャが
    /// 所有する Vec のため確保を避けられないが、それ以外の内部演算（`scale`／`add` の
    /// 呼び出し）は本メソッド内で 1 回だけ確保した作業バッファへ書き込む形に変え、
    /// 修正子反復（最大 `max_iter` 回）ごとの Vec 再確保を無くしている
    /// （時刻歴応答解析高速化・第2波と同じ「同じ演算を同じ順序で行い、確保回数だけを
    /// 減らす」方針。各要素の計算式・演算順序は元の `scale`/`add` 呼び出しと同一で
    /// 数値結果はビット一致する）。
    pub fn step<'b>(
        &self,
        q: &[f64],
        solve: &mut SolverFn<'b>,
        eval_fint: &mut FintFn<'_>,
        prev_du: &[f64],
        lambda: f64,
    ) -> Result<ArcLengthStep, String> {
        let n = q.len();

        let du_t = solve(q)?;
        let ut_norm = dot(&du_t, &du_t).sqrt();
        if ut_norm < 1e-30 {
            return Err("Zero tangent displacement".into());
        }

        let sign = if prev_du.is_empty() || dot(prev_du, &du_t) >= 0.0 {
            1.0
        } else {
            -1.0
        };

        let mut dlambda = sign * self.delta_l / ut_norm;

        // 軟化検知：接線変位が前ステップ増分に比べて著しく大きい場合、予測子を減額
        if !prev_du.is_empty() {
            let prev_norm = dot(prev_du, prev_du).sqrt();
            if prev_norm > 1e-30 {
                let ratio = ut_norm / prev_norm;
                if ratio > 10.0 {
                    dlambda *= 0.5;
                }
            }
        }

        // 修正子反復で繰り返し使う作業バッファ（反復ごとの Vec 再確保を避ける。
        // 各要素は元の scale/add 呼び出しと同一の式で書き込むため数値結果は不変）。
        let mut du = vec![0.0; n];
        scale_into(&du_t, dlambda, &mut du);

        // 予測子スケール制限：円筒半径に対するオーバーシュートを抑制
        let du_pred_norm = dot(&du, &du).sqrt();
        let bound = self.delta_l * 1.5;
        if du_pred_norm > bound {
            let scale_factor = bound / du_pred_norm;
            dlambda *= scale_factor;
            scale_into(&du_t, dlambda, &mut du);
        }

        let qq = dot(q, q);

        // 予測子の変位増分を要素状態へ反映し、その点での内力を取得する。
        let mut f_int = eval_fint(&du)?;

        let mut converged = false;
        let mut r = vec![0.0; n];
        let mut du_aug = vec![0.0; n];
        let mut tmp = vec![0.0; n];
        let mut d1 = vec![0.0; n];
        let mut d2 = vec![0.0; n];
        let mut du_update = vec![0.0; n];
        for _iter in 0..self.max_iter {
            let current_lambda = lambda + dlambda;
            for i in 0..n {
                r[i] = current_lambda * q[i] - f_int[i];
            }

            let r_norm = dot(&r, &r).sqrt();
            let ext_norm = (current_lambda * current_lambda * qq).sqrt() + 1e-30;
            if r_norm < self.tol * ext_norm {
                converged = true;
                break;
            }

            // du_bar = K⁻¹·r
            let du_bar = solve(&r)?;

            // 円筒型拘束の2次方程式 a·δλ² + b·δλ + c = 0
            // a = du_tᵀ·du_t
            // b = 2·(du + du_bar)ᵀ·du_t
            // c = (du + du_bar)ᵀ·(du + du_bar) - Δl²
            add_into(&du, &du_bar, &mut du_aug);
            let a = dot(&du_t, &du_t);
            let b = 2.0 * dot(&du_aug, &du_t);
            let c = dot(&du_aug, &du_aug) - self.delta_l * self.delta_l;

            let disc = b * b - 4.0 * a * c;
            if disc < 0.0 {
                return Err("Negative discriminant in arc-length constraint".into());
            }
            let sqrt_disc = disc.sqrt();
            let dlambda1 = (-b + sqrt_disc) / (2.0 * a);
            let dlambda2 = (-b - sqrt_disc) / (2.0 * a);

            // 根の選択：累積増分方向とのなす角が小さい根を選ぶ
            scale_into(&du_t, dlambda1, &mut tmp);
            add_into(&du_bar, &tmp, &mut d1);
            scale_into(&du_t, dlambda2, &mut tmp);
            add_into(&du_bar, &tmp, &mut d2);
            let cos1 = dot(prev_du, &d1);
            let cos2 = dot(prev_du, &d2);

            let dlambda_sel = if cos1 >= 0.0 && cos2 >= 0.0 {
                if cos1 >= cos2 {
                    dlambda1
                } else {
                    dlambda2
                }
            } else if cos1 >= 0.0 {
                dlambda1
            } else if cos2 >= 0.0 {
                dlambda2
            } else {
                if cos1 > cos2 {
                    dlambda1
                } else {
                    dlambda2
                }
            };

            // 変位・荷重増分の更新
            scale_into(&du_t, dlambda_sel, &mut tmp);
            add_into(&du_bar, &tmp, &mut du_update);
            for i in 0..n {
                du[i] += du_update[i];
            }
            dlambda += dlambda_sel;

            // 修正子増分 δu を要素状態へ反映し、内力を再評価（真の非線形反復）。
            f_int = eval_fint(&du_update)?;
        }

        Ok(ArcLengthStep {
            du,
            dlambda,
            converged,
        })
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// `out[i] = v[i] * s`。[`Self::step`] の修正子反復で使う作業バッファへ書き込む
/// （旧 `scale` と要素ごとに同一の式、Vec 確保が無いだけ）。
fn scale_into(v: &[f64], s: f64, out: &mut [f64]) {
    for i in 0..out.len() {
        out[i] = v[i] * s;
    }
}

/// `out[i] = a[i] + b[i]`。[`Self::step`] の修正子反復で使う作業バッファへ書き込む
/// （旧 `add` と要素ごとに同一の式、Vec 確保が無いだけ）。
fn add_into(a: &[f64], b: &[f64], out: &mut [f64]) {
    for i in 0..out.len() {
        out[i] = a[i] + b[i];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    /// 剛性低下型（軟化）1-DOF 系 f(u)=k·u - c·u² で弧長法をトレースし、
    /// 軟化域に入っても（a）収束すること、（b）予測子 |du| が delta_l * 1.5 を
    /// 超えないことを検証する。修正 Newton は極限点通過困難のため収束可能な範囲で検証。
    #[test]
    fn test_arc_length_softening_predictor_clip() {
        let k = 100.0;
        let c = 50.0;
        // f(u) = k·u - c·u², f'(u) = k - 2c·u, ピーク点 u* = k/(2c) = 1.0
        let f_of = |u: f64| k * u - c * u * u;
        let tangent = |u: f64| {
            let t = k - 2.0 * c * u;
            if t.abs() < 1e-12 {
                if t >= 0.0 {
                    1e-12
                } else {
                    -1e-12
                }
            } else {
                t
            }
        };

        let solver = ArcLengthSolver {
            delta_l: 0.05,
            max_iter: 80,
            tol: 1e-4,
        };
        let q = [1.0_f64];
        let mut u = 0.0_f64;
        let mut lambda = 0.0_f64;
        let mut prev_du: Vec<f64> = Vec::new();
        let mut converged_steps = 0usize;

        for step_i in 0..30 {
            let trial_u = Cell::new(u);
            let step = match solver.step(
                &q,
                &mut |r: &[f64]| -> Result<Vec<f64>, String> {
                    Ok(vec![r[0] / tangent(trial_u.get())])
                },
                &mut |du: &[f64]| -> Result<Vec<f64>, String> {
                    trial_u.set(trial_u.get() + du[0]);
                    Ok(vec![f_of(trial_u.get())])
                },
                &prev_du,
                lambda,
            ) {
                Ok(s) => s,
                Err(_) => break,
            };

            if !step.converged {
                break;
            }

            converged_steps += 1;

            // (b) 予測子 |du| が delta_l * 1.5 を超えないこと
            let du_norm = dot(&step.du, &step.du).sqrt();
            assert!(
                du_norm <= solver.delta_l * 1.5,
                "step {step_i}: |du| = {du_norm} > delta_l * 1.5 = {}",
                solver.delta_l * 1.5
            );

            u += step.du[0];
            lambda += step.dlambda;
            prev_du = step.du;

            // 収束点で非線形平衡が成立すること
            assert!(
                (f_of(u) - lambda * q[0]).abs() < 1e-1,
                "step {step_i}: equilibrium violated: f(u)={}, lambda*q={}",
                f_of(u),
                lambda * q[0]
            );
        }

        // 最低限のステップが収束すること
        assert!(
            converged_steps >= 8,
            "at least 8 steps should converge, got {converged_steps}"
        );
        // 軟化領域付近まで到達していること
        assert!(
            u > 0.5,
            "trace should approach the limit point, final u={u}"
        );
    }

    /// 1-DOF の非線形（剛性増加型）弾性系で弧長法をトレースし、各収束点が
    /// 非線形平衡 f(u)=λ·q を満たすことを検証する。
    /// 内力 f_int を修正子の各反復で再評価していなければ平衡が崩れる
    /// （旧実装は f_int を固定パラメータとしており非線形反復になっていなかった）。
    #[test]
    fn test_arc_length_reevaluates_fint_nonlinear() {
        // f(u) = k·u + c·u²（単調・剛性増加）。接線 = k + 2c·u。
        let k = 100.0;
        let c = 50.0;
        let f_of = |u: f64| k * u + c * u * u;
        let tangent = |u: f64| k + 2.0 * c * u;

        let solver = ArcLengthSolver::new(0.15);
        let q = [1.0_f64];
        let mut u = 0.0_f64;
        let mut lambda = 0.0_f64;
        let mut prev_du: Vec<f64> = Vec::new();
        let eval_calls = Cell::new(0usize);

        for _ in 0..12 {
            let trial_u = Cell::new(u);
            let step = solver
                .step(
                    &q,
                    // 接線 Newton: 現在の trial 変位の接線で K⁻¹ を構成。
                    &mut |r: &[f64]| -> Result<Vec<f64>, String> {
                        Ok(vec![r[0] / tangent(trial_u.get())])
                    },
                    // δu を trial 変位へ反映し、更新後の内力を返す（再評価される）。
                    &mut |du: &[f64]| -> Result<Vec<f64>, String> {
                        trial_u.set(trial_u.get() + du[0]);
                        eval_calls.set(eval_calls.get() + 1);
                        Ok(vec![f_of(trial_u.get())])
                    },
                    &prev_du,
                    lambda,
                )
                .expect("arc-length step should solve");

            assert!(step.converged, "step should converge");
            u += step.du[0];
            lambda += step.dlambda;
            prev_du = step.du;

            // 各収束点で非線形平衡が成立すること。
            assert!(
                (f_of(u) - lambda * q[0]).abs() < 1e-2,
                "equilibrium violated: f(u)={}, lambda*q={}",
                f_of(u),
                lambda * q[0]
            );
        }

        // 非線形領域（2次項が無視できない u）まで到達していること。
        assert!(u > 0.5, "trace should reach nonlinear range, final u={u}");
        // 予測子のみ（12回）でなく修正子でも内力を再評価していること。
        assert!(
            eval_calls.get() > 12,
            "f_int should be re-evaluated in corrector iterations, calls={}",
            eval_calls.get()
        );
    }
}
