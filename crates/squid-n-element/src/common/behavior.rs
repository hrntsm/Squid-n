use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use std::any::Any;

/// チェックポイントからの要素状態復元に関するエラー。
#[derive(Debug, thiserror::Error)]
pub enum CheckpointError {
    /// 要素チェックポイント本体（bincode）の復号に失敗した。
    #[error("チェックポイントの復号に失敗しました: {0}")]
    Decode(String),
    /// 内包する材料状態の復元に失敗した。
    #[error(transparent)]
    MaterialState(#[from] squid_n_material::MaterialStateError),
}

#[derive(Clone)]
pub struct LocalMat {
    pub n: usize,
    pub data: Vec<f64>,
}

pub struct LocalVec {
    pub data: SmallVec<[f64; 24]>,
}

pub struct Ctx<'a> {
    pub model: &'a Model,
}

#[derive(Clone, Copy)]
pub enum MassOption {
    Lumped,
    Consistent,
}

/// ヒンジ詳細表示用: 1 ガウス点断面のファイバー状態スナップショット。
/// プッシュオーバー終局時に「断面のどこが塑性化しているか」を可視化するため、
/// 解析終了時点の要素状態から断面内全ファイバーの位置・ひずみ・降伏比を取り出す
/// （[`ElementBehavior::fiber_section_states`]）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct FiberSectionState {
    /// ガウス点の材軸位置 ξ ∈ [-1, 1]（-1 側が i 端、+1 側が j 端）。
    pub xi: f64,
    /// 断面内の全ファイバーの状態。
    pub fibers: Vec<FiberStateSample>,
}

/// ファイバー 1 本の状態（断面内位置・ひずみ・降伏比。[`FiberSectionState`] の要素）。
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct FiberStateSample {
    /// 断面内位置（要素局所 y）[mm]。
    pub y: f64,
    /// 断面内位置（要素局所 z）[mm]。
    pub z: f64,
    /// ファイバー断面積 [mm²]。
    pub area: f64,
    /// 軸ひずみ（引張正）。
    pub strain: f64,
    /// 降伏比 |ε|/εref（≧1 で降伏。基準ひずみを持たない材料は 0）。
    pub yield_ratio: f64,
    /// 材料区分（0=コンクリート、1=主筋、2=鋼材（形鋼・鋼管・内蔵鉄骨）。
    /// 旧結果ファイルでは 0=母材格子（コンクリートまたは鋼材）・1=主筋）。
    pub material: usize,
}

/// 塑性率（ductility）評価用の危険断面プローブ（ファイバーモデルの塑性率、
/// 構造力学）。ファイバー要素が最大曲率のガウス点（危険断面）
/// について現在のひずみ状態を集約して返す。プッシュオーバー解析
/// （`squid_n_solver::pushover`）が各ステップで参照し、塑性率基点曲率と
/// 最大応答曲率から部材塑性率 μ を算定する。
#[derive(Clone, Copy, Debug, Default)]
pub struct DuctilityProbe {
    /// 危険断面の曲率の大きさ |κ| = √(κy²+κz²) [1/mm]。
    pub curvature: f64,
    /// 断面内の最大引張ひずみ（正）。
    pub max_tension_strain: f64,
    /// 断面内の最大圧縮ひずみの大きさ（正で返す）。
    pub max_compression_strain: f64,
    /// 各ファイバの塑性率 μi=|ε|/εref の最大値（≥1 で降伏＝塑性率基点方式(3)）。
    pub max_yield_ratio: f64,
    /// 重み付け平均塑性率 Jm = Σσref·A·|ε|·μi / Σσref·A·|ε|（≥1 で基点＝方式(2)）。
    pub jm: f64,
}

impl LocalMat {
    pub fn zeros(n: usize) -> Self {
        Self {
            n,
            data: vec![0.0; n * n],
        }
    }

    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.data[i * self.n + j]
    }

    pub fn set(&mut self, i: usize, j: usize, v: f64) {
        self.data[i * self.n + j] = v;
    }

    pub fn to_triplets(&self, gdofs: &[usize]) -> Vec<squid_n_math::sparse::Triplet> {
        let mut out = Vec::with_capacity(self.n * self.n);
        for i in 0..self.n {
            let gi = gdofs[i];
            if gi == usize::MAX {
                continue;
            }
            for (j, &gj) in gdofs.iter().enumerate().take(self.n) {
                if gj == usize::MAX {
                    continue;
                }
                let v = self.get(i, j);
                if v != 0.0 {
                    out.push(squid_n_math::sparse::Triplet {
                        row: gi,
                        col: gj,
                        val: v,
                    });
                }
            }
        }
        out
    }
}

/// `Send + Sync` を supertrait とするのは、静解析バッチ（`squid-n-solver::statics`）が
/// 荷重ケース・組合せごとの `Box<dyn ElementBehavior>` キャッシュを rayon 並列
/// （`&self` 共有）から参照するため。全実装型は内部に `Box<dyn UniaxialMaterial>`
/// （既に `Send + Sync` 境界つき）等のみを持ち、`Rc`/`RefCell`/`Cell` 等の
/// 非スレッド安全な型は含まないため自動的に満たされる。
pub trait ElementBehavior: Send + Sync {
    fn n_dof(&self) -> usize;
    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]>;
    fn tangent_stiffness(&self, ctx: &Ctx) -> LocalMat;
    fn internal_force(&self, ctx: &Ctx) -> LocalVec;
    fn update_state(&mut self, _du: &LocalVec, _commit: bool, _ctx: &Ctx) {}
    /// 質量行列を**全体系**で返す（`tangent_stiffness` と同じ契約）。
    /// ソルバはこの返り値をそのまま全体自由度へ散布するため、回転不変でない
    /// 整合質量（軸方向と曲げ方向で係数が異なる）は実装側で
    /// `M_global = Rᵀ M_local R` の変換を済ませること。並進 3 成分が等しい
    /// 対角の集中質量は回転不変なので変換を省略してよい。
    fn mass_matrix(&self, opt: MassOption) -> LocalMat;
    /// 節点変位から部材内力分布を復元する（線形解析の内力回収）。
    ///
    /// 引数は要素の全体系節点変位（線形解析の解 `u`）で、剛性と内力が線形関係に
    /// ある要素だけが実装できる。弾塑性要素は履歴に依存するため、この経路ではなく
    /// [`Self::state_member_forces`] を用いること。
    ///
    /// 線材（梁・柱・ブレース）は必ず実装する。ここで `None` を返すと線形解析の
    /// `member_forces` から当該部材が丸ごと欠落し、応力図・断面検定・接合部検定の
    /// 入力が空になる（線形静解析側で検出してエラーにしている）。
    fn recover_forces(&self, _u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        None
    }
    /// 現在の要素状態（committed / trial）から部材内力分布を返す（非線形解析用）。
    ///
    /// 弾塑性要素の内力は「接線剛性 × 全変位」では**降伏後に誤る**ため、
    /// 履歴状態から求めた復元力（[`Self::internal_force`]）や断面応答を材軸方向へ
    /// 釣合いで分配して組み立てる（[`crate::beam::member_forces_from_end_forces`]）。
    ///
    /// 既定は `None`（内力分布を持たない要素、または状態から正しく内力を
    /// 取り出せない要素）。プッシュオーバー・時刻歴の結果から部材応力を
    /// 取り出す用途はこちらを使う。
    fn state_member_forces(&self, _ctx: &Ctx) -> Option<crate::beam::MemberForces> {
        None
    }
    /// T7: 線形化幾何剛性 Kg（P-Δ）。軸力 N（引張正）。デフォルトはゼロ。
    fn geometric_stiffness(&self, _n: f64) -> LocalMat {
        LocalMat::zeros(12)
    }
    /// T4: 全材料の committed 状態をスナップショット
    fn snapshot_state(&self) -> Box<dyn Any> {
        Box::new(())
    }
    /// T4: スナップショットから状態を復元
    fn restore_state(&mut self, _state: &dyn Any) {}
    /// T4: 全材料の trial を committed に確定
    fn commit_state(&mut self) {}
    /// T4: 全材料の trial を committed に戻す（rollback）
    fn revert_state(&mut self) {}
    /// チェックポイント用: 要素の全状態をバイト列へ直列化
    fn serialize_checkpoint(&self) -> Vec<u8> {
        vec![]
    }
    /// チェックポイント用: バイト列から要素状態を復元。
    /// 復号や内包材料の復元に失敗した場合は [`CheckpointError`] を返す。
    fn deserialize_checkpoint(&mut self, _data: &[u8]) -> Result<(), CheckpointError> {
        Ok(())
    }
    /// 仕口パネル要素のせん断モーメント `{MSX, MSY}` [N·mm]（基準座標系）を、
    /// 与えられた要素自由度の変位から返す（仕口パネルのみ実装。既定は `None`）。
    ///
    /// `u_elem` は [`Self::global_dofs`] と同じ並びの解ベクトル。断面検定の設計用
    /// パネルモーメント `pM` に用いる。節点まわりのモーメント釣り合いが解析上
    /// 厳密に満たされた値であり、部材端内力から手で組み立てる近似を経ない。
    fn panel_moments_from(&self, _u_elem: &[f64]) -> Option<[f64; 2]> {
        None
    }
    /// 塑性率評価用の危険断面プローブ（ファイバー要素のみ実装。既定は None）。
    /// ファイバーモデルの塑性率算定（構造力学）に用いる。
    fn ductility_probe(&self) -> Option<DuctilityProbe> {
        None
    }
    /// ヒンジ詳細表示用: 各ガウス点断面のファイバー状態（位置・ひずみ・降伏比）を
    /// 現在の要素状態から返す（ファイバー要素のみ実装。既定は None）。
    /// プッシュオーバー終局時の断面塑性化状況の可視化に用いる。
    fn fiber_section_states(&self) -> Option<Vec<FiberSectionState>> {
        None
    }
    /// 時刻歴解析の時間刻み Δt [s] を要素へ通知する（構造動力学の時刻歴応答解析。
    /// 制振要素）。速度依存の減衰要素（マクスウェル要素等）が後退 Euler の
    /// ダッシュポット積分に用いる。`dt<=0`（静的・線形）では減衰要素は不活性となる。
    /// 対応しない要素は何もしない（既定）。
    fn set_time_step(&mut self, _dt: f64) {}
}
