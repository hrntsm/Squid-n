//! 部材レベルの履歴則パラメータ [`HysteresisRule`] とスケルトン包絡線の算定。

/// 部材レベルの履歴則パラメータ。
/// 状態（履歴変数）は `HysteresisMaterial` が保持する。本 enum は不変パラメータのみ。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum HysteresisRule {
    /// 武田モデル（剛性低下型トリリニア）。
    Takeda {
        /// ひび割れ点 (Mc, θc)
        crack: (f64, f64),
        /// 降伏点 (My, θy)
        yield_point: (f64, f64),
        /// 終局点 (Mu, θu)
        ultimate: (f64, f64),
        /// 除荷剛性低下指数 α
        alpha: f64,
    },
    /// 武田モデル劣化版。
    TakedaDegrading {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
        alpha: f64,
        /// 劣化率
        degradation: f64,
    },
    /// 原点指向型（せん断）。
    OriginOriented {
        /// 降伏点 (Qy, δy)
        yield_point: (f64, f64),
        /// 終局点 (Qu, δu)
        ultimate: (f64, f64),
    },
    /// スリップ型（せん断）。
    Slip {
        yield_point: (f64, f64),
        ultimate: (f64, f64),
        /// スリップ量係数（0..1）
        slip_factor: f64,
    },
    /// 逆行型。
    Retrograde {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
    },
    /// 標準型（Masing 則）。
    Standard {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
    },
    /// 最大点指向型。
    MaxPointOriented {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
    },
}

impl HysteresisRule {
    /// スケルトン包絡線上の (力, 接線剛性) を返す。符号対称を仮定。
    /// 変形は θ [rad] または δ [mm]、力は [N·mm] または [N]。
    pub fn skeleton_with_degradation(&self, theta: f64, degrade_factor: f64) -> (f64, f64) {
        match *self {
            HysteresisRule::Takeda {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
                ..
            }
            | HysteresisRule::TakedaDegrading {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
                ..
            }
            | HysteresisRule::Retrograde {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
            }
            | HysteresisRule::Standard {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
            }
            | HysteresisRule::MaxPointOriented {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
            } => {
                let (m, k) = trilinear_symmetric(theta, tc, mc, ty, my, tu, mu);
                (m * degrade_factor, k * degrade_factor)
            }
            HysteresisRule::OriginOriented {
                yield_point: (my, ty),
                ultimate: (mu, tu),
            }
            | HysteresisRule::Slip {
                yield_point: (my, ty),
                ultimate: (mu, tu),
                ..
            } => {
                let (m, k) = bilinear_symmetric(theta, ty, my, tu, mu);
                (m * degrade_factor, k * degrade_factor)
            }
        }
    }

    /// スケルトン包絡線（劣化なし）。
    pub fn skeleton(&self, theta: f64) -> (f64, f64) {
        self.skeleton_with_degradation(theta, 1.0)
    }

    /// 劣化版の劣化率。
    pub fn degradation_rate(&self) -> f64 {
        match *self {
            HysteresisRule::TakedaDegrading { degradation, .. } => degradation,
            _ => 1.0,
        }
    }

    /// 降伏点。力・変形の順。
    pub fn yield_point(&self) -> (f64, f64) {
        match *self {
            HysteresisRule::Takeda { yield_point, .. }
            | HysteresisRule::TakedaDegrading { yield_point, .. }
            | HysteresisRule::OriginOriented { yield_point, .. }
            | HysteresisRule::Slip { yield_point, .. }
            | HysteresisRule::Retrograde { yield_point, .. }
            | HysteresisRule::Standard { yield_point, .. }
            | HysteresisRule::MaxPointOriented { yield_point, .. } => yield_point,
        }
    }

    /// 降伏変形。
    pub fn yield_deformation(&self) -> f64 {
        self.yield_point().1
    }

    /// 降伏耐力。
    pub fn yield_strength(&self) -> f64 {
        self.yield_point().0
    }

    /// ひび割れ点。ない場合は None。
    pub fn crack_point(&self) -> Option<(f64, f64)> {
        match *self {
            HysteresisRule::Takeda { crack, .. }
            | HysteresisRule::TakedaDegrading { crack, .. }
            | HysteresisRule::Retrograde { crack, .. }
            | HysteresisRule::Standard { crack, .. }
            | HysteresisRule::MaxPointOriented { crack, .. } => Some(crack),
            _ => None,
        }
    }

    /// 終局点。
    pub fn ultimate_point(&self) -> (f64, f64) {
        match *self {
            HysteresisRule::Takeda { ultimate, .. }
            | HysteresisRule::TakedaDegrading { ultimate, .. }
            | HysteresisRule::OriginOriented { ultimate, .. }
            | HysteresisRule::Slip { ultimate, .. }
            | HysteresisRule::Retrograde { ultimate, .. }
            | HysteresisRule::Standard { ultimate, .. }
            | HysteresisRule::MaxPointOriented { ultimate, .. } => ultimate,
        }
    }

    /// 除荷剛性（降伏後、武田モデル）。
    /// Kd+ = K0 · |δmax / δy2|^(−ν)。K0 = ひび割れ勾配、ν = `alpha`。
    pub fn unloading_stiffness(&self, max_deformation: f64) -> Option<f64> {
        match *self {
            HysteresisRule::Takeda {
                crack: (mc, tc),
                yield_point: (_my, theta_y),
                alpha,
                ..
            }
            | HysteresisRule::TakedaDegrading {
                crack: (mc, tc),
                yield_point: (_my, theta_y),
                alpha,
                ..
            } => {
                if theta_y.abs() < 1e-15 || tc.abs() < 1e-15 {
                    return None;
                }
                let k0 = mc / tc;
                let ratio = (max_deformation.abs() / theta_y).max(1.0);
                Some(k0 * ratio.powf(-alpha))
            }
            HysteresisRule::OriginOriented { .. }
            | HysteresisRule::Slip { .. }
            | HysteresisRule::Retrograde { .. }
            | HysteresisRule::Standard { .. }
            | HysteresisRule::MaxPointOriented { .. } => None,
        }
    }

    pub(crate) fn is_takeda(&self) -> bool {
        matches!(
            *self,
            HysteresisRule::Takeda { .. } | HysteresisRule::TakedaDegrading { .. }
        )
    }

    pub(crate) fn is_retrograde(&self) -> bool {
        matches!(*self, HysteresisRule::Retrograde { .. })
    }

    pub(crate) fn is_standard(&self) -> bool {
        matches!(*self, HysteresisRule::Standard { .. })
    }

    pub(crate) fn is_max_point_oriented(&self) -> bool {
        matches!(*self, HysteresisRule::MaxPointOriented { .. })
    }
}

/// トリリニアの符号対称スケルトン。
fn trilinear_symmetric(
    theta: f64,
    tc: f64,
    mc: f64,
    ty: f64,
    my: f64,
    tu: f64,
    mu: f64,
) -> (f64, f64) {
    let (t, sgn) = if theta >= 0.0 {
        (theta, 1.0)
    } else {
        (-theta, -1.0)
    };
    let (m, k) = if t <= tc {
        (mc * (t / tc).max(0.0), mc / tc)
    } else if t <= ty {
        let k = (my - mc) / (ty - tc);
        (mc + k * (t - tc), k)
    } else if t <= tu {
        let k = (mu - my) / (tu - ty);
        (my + k * (t - ty), k)
    } else {
        (mu, 0.0)
    };
    (sgn * m, k)
}

/// バイリニアの符号対称スケルトン。
fn bilinear_symmetric(theta: f64, ty: f64, my: f64, tu: f64, mu: f64) -> (f64, f64) {
    let (t, sgn) = if theta >= 0.0 {
        (theta, 1.0)
    } else {
        (-theta, -1.0)
    };
    let (m, k) = if t <= ty {
        (my * (t / ty).max(0.0), my / ty)
    } else if t <= tu {
        let k = (mu - my) / (tu - ty);
        (my + k * (t - ty), k)
    } else {
        (mu, 0.0)
    };
    (sgn * m, k)
}

/// テスト用の代表的な履歴則を組み立てるフィクスチャ。
/// `rule` / `material` 双方のテストから共用する（重複定義を避ける）。
#[cfg(test)]
pub(crate) fn takeda() -> HysteresisRule {
    HysteresisRule::Takeda {
        crack: (40.0, 0.002),
        yield_point: (100.0, 0.01),
        ultimate: (120.0, 0.05),
        alpha: 0.4,
    }
}

#[cfg(test)]
pub(crate) fn retrograde() -> HysteresisRule {
    HysteresisRule::Retrograde {
        crack: (40.0, 0.002),
        yield_point: (100.0, 0.01),
        ultimate: (120.0, 0.05),
    }
}

#[cfg(test)]
pub(crate) fn standard() -> HysteresisRule {
    HysteresisRule::Standard {
        crack: (40.0, 0.002),
        yield_point: (100.0, 0.01),
        ultimate: (120.0, 0.05),
    }
}

#[cfg(test)]
pub(crate) fn max_point() -> HysteresisRule {
    HysteresisRule::MaxPointOriented {
        crack: (40.0, 0.002),
        yield_point: (100.0, 0.01),
        ultimate: (120.0, 0.05),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_skeleton_monotonic_takeda() {
        let rule = takeda();
        // 原点-ひび割れ線形
        let (m, k) = rule.skeleton(0.001);
        assert_relative_eq!(m, 20.0, epsilon = 1e-6);
        assert_relative_eq!(k, 20000.0, epsilon = 1e-3);
        // ひび割れ点
        let (m, _) = rule.skeleton(0.002);
        assert_relative_eq!(m, 40.0, epsilon = 1e-6);
        // 降伏点
        let (m, _) = rule.skeleton(0.01);
        assert_relative_eq!(m, 100.0, epsilon = 1e-6);
        // 終局点
        let (m, _) = rule.skeleton(0.05);
        assert_relative_eq!(m, 120.0, epsilon = 1e-6);
        // 負側対称
        let (m, _) = rule.skeleton(-0.01);
        assert_relative_eq!(m, -100.0, epsilon = 1e-6);
    }

    #[test]
    fn test_new_rules_skeleton_matches_takeda_trilinear() {
        // 新規則のスケルトンは武田と同じトリリニア（骨格は履歴則に依存しない）。
        for rule in [retrograde(), standard(), max_point()] {
            let (m, _) = rule.skeleton(0.01);
            assert_relative_eq!(m, 100.0, epsilon = 1e-6);
            let (m2, _) = rule.skeleton(-0.002);
            assert_relative_eq!(m2, -40.0, epsilon = 1e-6);
        }
    }
}
