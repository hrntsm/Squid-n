use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE, PANEL_DOF_PER_NODE};
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

/// [`ElementBehavior::restore_state`] のスナップショット downcast（診断付き）。
///
/// スナップショットは同一実行内の巻き戻し専用のため、型不一致は
/// `snapshot_state` と `restore_state` の実装対応が崩れたプログラムエラーであり、
/// データ起因では発生しない。従来は不一致を無音で握りつぶして復元をスキップ
/// しており、非収束ステップのロールバックが効かないまま汚染された要素状態で
/// 解析が続行し、誤った結果を返し得た。要素種別と期待型を名指しして panic する。
pub fn downcast_snapshot<'a, T: 'static>(element: &str, state: &'a dyn Any) -> &'a T {
    state.downcast_ref::<T>().unwrap_or_else(|| {
        panic!(
            "{element}::restore_state: スナップショットの型が一致しません（期待: {}）。\
             snapshot_state と restore_state の実装対応が崩れています",
            std::any::type_name::<T>()
        )
    })
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
/// （`squid_n_solver::nonlinear::pushover`）が各ステップで参照し、塑性率基点曲率と
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

/// 節点列の並進・回転自由度を、[`ElementBehavior::global_dofs`] が返す並びへ収集する。
///
/// 節点 1 個あたり [`DOF_PER_NODE`] 成分を節点の順に並べ、拘束・従属で全体方程式から
/// 消えた自由度は `usize::MAX` を置く（[`LocalMat::to_triplets`] がこの番兵を見て
/// 当該行・列の散布を飛ばす）。
///
/// 節点自由度だけで構成される要素はこれをそのまま返せばよい。仕口パネルのように
/// 追加自由度を持つ要素は、[`push_node_global_dofs`] と [`push_panel_global_dofs`] を
/// 必要な順に呼んで並びを組み立てる。
pub fn node_global_dofs(
    nodes: &[squid_n_core::ids::NodeId],
    dof: &DofMap,
) -> SmallVec<[usize; 24]> {
    let mut gdofs = SmallVec::new();
    push_node_global_dofs(&mut gdofs, nodes, dof);
    gdofs
}

/// [`node_global_dofs`] の並びを、既存の並びの末尾へ追加する。
pub fn push_node_global_dofs(
    out: &mut SmallVec<[usize; 24]>,
    nodes: &[squid_n_core::ids::NodeId],
    dof: &DofMap,
) {
    for &nid in nodes {
        let ni = nid.index();
        for d in 0..DOF_PER_NODE {
            let g = ni * DOF_PER_NODE + d;
            out.push(dof.active(g).map_or(usize::MAX, |a| a as usize));
        }
    }
}

/// 仕口パネル節点の追加自由度（せん断変形角。[`PANEL_DOF_PER_NODE`] 成分）を
/// 既存の並びの末尾へ追加する。欠番の扱いは [`push_node_global_dofs`] と同じく
/// `usize::MAX` の番兵。
pub fn push_panel_global_dofs(out: &mut SmallVec<[usize; 24]>, node_idx: usize, dof: &DofMap) {
    for d in 0..PANEL_DOF_PER_NODE {
        out.push(
            dof.panel_dof(node_idx, d)
                .map_or(usize::MAX, |a| a as usize),
        );
    }
}

/// 全状態が「全体系の committed / trial 変位」だけで表せる線形弾性要素へ、
/// [`ElementBehavior`] の `internal_force` と状態管理 6 メソッドを実装する。
///
/// `impl ElementBehavior for X { ... }` の中で `elastic_disp_behavior!(X, 12);` と
/// 呼ぶ。要素側は `committed_disp: [f64; N]` と `trial_disp: [f64; N]` を持ち、
/// `tangent_stiffness` を実装していることが前提になる。
///
/// この定型は Newton 反復のトライアル追従（未確定変位も内力へ反映する。committed
/// だけを見ると反復中に内力が凍結し、収束が準ニュートンへ劣化する）と、レジューム
/// のためのチェックポイント（両変位を収録しないと変位 0 から再計算されて内力が
/// 不整合になる）という 2 つの規約でできている。要素ごとに手書きすると、
/// `IsolatorElement` と `HystereticDamperElement` で実際に起きたように
/// チェックポイントの実装漏れが静かに混入する。
///
/// **材料履歴やばねの状態を併せ持つ要素はこれを使えない。** commit / revert が
/// 変位以外にも及ぶため、`FiberBeam`・`WallElement`・`PanelZone` 等は個別実装が正しい。
///
/// `internal_force` は `tangent_stiffness` を呼んで `f = K_global · u_trial` を組む。
/// 従来は各要素が `tangent_stiffness` と同じ K の組み立て式を内力側にも書いており、
/// 片方だけ直すと両者が静かに食い違う状態だった。
#[macro_export]
macro_rules! elastic_disp_behavior {
    ($ty:ident, $n:expr) => {
        fn internal_force(&self, ctx: &$crate::behavior::Ctx) -> $crate::behavior::LocalVec {
            // f_global = (Rᵀ·K_local·R)·u_global。接線剛性と同一の K を使うことで、
            // 剛性と内力の整合を実装の対応ではなく呼び出しで保証する。
            let k = <Self as $crate::behavior::ElementBehavior>::tangent_stiffness(self, ctx);
            let mut f = $crate::behavior::LocalVec {
                data: ::smallvec::SmallVec::from_elem(0.0, $n),
            };
            for i in 0..$n {
                let mut s = 0.0;
                for j in 0..$n {
                    s += k.get(i, j) * self.trial_disp[j];
                }
                f.data[i] = s;
            }
            f
        }

        fn update_state(
            &mut self,
            du: &$crate::behavior::LocalVec,
            commit: bool,
            _ctx: &$crate::behavior::Ctx,
        ) {
            // `du` はソルバが要素自由度ぶんを集めて渡すため、長さは常に $n に等しい。
            // 食い違ったら添字で panic させる（短い方に合わせて黙って部分更新すると、
            // 変位の一部が欠けたまま解析が続行し、誤った内力を返す）。
            for i in 0..$n {
                self.trial_disp[i] += du.data[i];
            }
            if commit {
                self.committed_disp = self.trial_disp;
            }
        }

        fn commit_state(&mut self) {
            self.committed_disp = self.trial_disp;
        }

        fn revert_state(&mut self) {
            self.trial_disp = self.committed_disp;
        }

        fn snapshot_state(&self) -> Box<dyn ::std::any::Any> {
            Box::new((self.committed_disp, self.trial_disp))
        }

        fn restore_state(&mut self, state: &dyn ::std::any::Any) {
            let (committed, trial) = $crate::behavior::downcast_snapshot::<([f64; $n], [f64; $n])>(
                stringify!($ty),
                state,
            );
            self.committed_disp = *committed;
            self.trial_disp = *trial;
        }

        fn serialize_checkpoint(&self) -> Vec<u8> {
            ::bincode::serialize(&(self.committed_disp, self.trial_disp))
                .expect("serialize checkpoint")
        }

        fn deserialize_checkpoint(
            &mut self,
            data: &[u8],
        ) -> Result<(), $crate::behavior::CheckpointError> {
            // 旧チェックポイント（変位未収録・空バイト列）は「状態なし」として許容する。
            if data.is_empty() {
                return Ok(());
            }
            let (committed, trial): ([f64; $n], [f64; $n]) = ::bincode::deserialize(data)
                .map_err(|e| $crate::behavior::CheckpointError::Decode(e.to_string()))?;
            self.committed_disp = committed;
            self.trial_disp = trial;
            Ok(())
        }
    };
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
    fn recover_forces(&self, _u_elem: &[f64]) -> Option<crate::frame::beam::MemberForces> {
        None
    }
    /// 現在の要素状態（committed / trial）から部材内力分布を返す（非線形解析用）。
    ///
    /// 弾塑性要素の内力は「接線剛性 × 全変位」では**降伏後に誤る**ため、
    /// 履歴状態から求めた復元力（[`Self::internal_force`]）や断面応答を材軸方向へ
    /// 釣合いで分配して組み立てる（[`crate::frame::beam::member_forces_from_end_forces`]）。
    ///
    /// 既定は `None`（内力分布を持たない要素、または状態から正しく内力を
    /// 取り出せない要素）。プッシュオーバー・時刻歴の結果から部材応力を
    /// 取り出す用途はこちらを使う。
    fn state_member_forces(&self, _ctx: &Ctx) -> Option<crate::frame::beam::MemberForces> {
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

/// 内側の要素へ `ElementBehavior` を委譲するラッパ要素の実装を生成する。
///
/// ラッパ（[`crate::frame::multi_spring::MultiSpringElement`]・
/// [`crate::wall::side_column::InPlaneReleasedColumn`]・
/// [`crate::frame::panel_offset::PanelOffsetMember`]）は、大半のメソッドを内側の要素へ
/// そのまま流し、いくつかだけ自前で持つ。これを手書きすると、**流し忘れたメソッドが
/// トレイトの既定値へ静かに落ちる**。既定値は `None` や「何もしない」なので、
/// コンパイルは通り、テストも書いていなければ落ちない。
///
/// 実際に `MultiSpringElement` は `fiber_section_states` の委譲を書き忘れており、
/// 内側の [`crate::frame::fiber::FiberBeam`] が状態を持っているのに、ヒンジ詳細ウィンドウの
/// ファイバー断面の塑性化マップが MS 要素でだけ空になっていた。
///
/// # 使い方
///
/// 全メソッドについて `forward`（内側へ委譲）か `custom`（自前で実装）かを
/// **必ず明示する**。`custom` としたメソッドの本体は末尾の `custom { ... }` へ書く。
///
/// ```ignore
/// forward_element_behavior!(InPlaneReleasedColumn, inner, {
///     n_dof: forward,
///     tangent_stiffness: custom,
///     // …全 19 メソッド…
/// }, custom {
///     fn tangent_stiffness(&self, _ctx: &Ctx) -> LocalMat { … }
/// });
/// ```
///
/// # なぜ全メソッドを列挙させるか
///
/// マクロのパターンが 19 メソッドすべての記載を要求するため、
/// **[`ElementBehavior`] にメソッドを追加してこのマクロを更新すると、
/// 呼び出し側 3 箇所はパターンに一致しなくなってコンパイルエラーになる**。
/// 「新しいメソッドをこのラッパでどう扱うか」を決めない限りビルドが通らない、
/// という形で流し忘れを防ぐ。列挙は冗長だが、その冗長さが保証の実体である。
///
/// `custom` の本体を書いたのに `forward` と記した場合は、同名メソッドの重複定義に
/// なってこれもコンパイルエラーになる。
///
/// **[`ElementBehavior`] にメソッドを追加したら、このマクロにも追加すること。**
#[macro_export]
macro_rules! forward_element_behavior {
    // 内部アーム: `forward` なら本体を出し、`custom` なら何も出さない。
    // 引数は呼び出し側が書いた `forward` / `custom` の識別子がそのまま届く。
    (@opt forward $($body:tt)*) => { $($body)* };
    (@opt custom $($body:tt)*) => {};

    ($ty:ty, $inner:ident, {
        n_dof: $m_n_dof:ident,
        global_dofs: $m_global_dofs:ident,
        tangent_stiffness: $m_tangent_stiffness:ident,
        internal_force: $m_internal_force:ident,
        update_state: $m_update_state:ident,
        mass_matrix: $m_mass_matrix:ident,
        recover_forces: $m_recover_forces:ident,
        state_member_forces: $m_state_member_forces:ident,
        geometric_stiffness: $m_geometric_stiffness:ident,
        snapshot_state: $m_snapshot_state:ident,
        restore_state: $m_restore_state:ident,
        commit_state: $m_commit_state:ident,
        revert_state: $m_revert_state:ident,
        serialize_checkpoint: $m_serialize_checkpoint:ident,
        deserialize_checkpoint: $m_deserialize_checkpoint:ident,
        panel_moments_from: $m_panel_moments_from:ident,
        ductility_probe: $m_ductility_probe:ident,
        fiber_section_states: $m_fiber_section_states:ident,
        set_time_step: $m_set_time_step:ident $(,)?
    } $(, custom { $($custom:tt)* })? $(,)?) => {
        impl $crate::behavior::ElementBehavior for $ty {
            $($($custom)*)?

            $crate::forward_element_behavior!(@opt $m_n_dof
                fn n_dof(&self) -> usize {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.n_dof()
                }
            );
            $crate::forward_element_behavior!(@opt $m_global_dofs
                fn global_dofs(
                    &self,
                    dof: &::squid_n_core::dof::DofMap,
                ) -> ::smallvec::SmallVec<[usize; 24]> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.global_dofs(dof)
                }
            );
            $crate::forward_element_behavior!(@opt $m_tangent_stiffness
                fn tangent_stiffness(
                    &self,
                    ctx: &$crate::behavior::Ctx,
                ) -> $crate::behavior::LocalMat {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.tangent_stiffness(ctx)
                }
            );
            $crate::forward_element_behavior!(@opt $m_internal_force
                fn internal_force(
                    &self,
                    ctx: &$crate::behavior::Ctx,
                ) -> $crate::behavior::LocalVec {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.internal_force(ctx)
                }
            );
            $crate::forward_element_behavior!(@opt $m_update_state
                fn update_state(
                    &mut self,
                    du: &$crate::behavior::LocalVec,
                    commit: bool,
                    ctx: &$crate::behavior::Ctx,
                ) {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.update_state(du, commit, ctx)
                }
            );
            $crate::forward_element_behavior!(@opt $m_mass_matrix
                fn mass_matrix(
                    &self,
                    opt: $crate::behavior::MassOption,
                ) -> $crate::behavior::LocalMat {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.mass_matrix(opt)
                }
            );
            $crate::forward_element_behavior!(@opt $m_recover_forces
                fn recover_forces(&self, u_elem: &[f64]) -> Option<$crate::frame::beam::MemberForces> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.recover_forces(u_elem)
                }
            );
            $crate::forward_element_behavior!(@opt $m_state_member_forces
                fn state_member_forces(
                    &self,
                    ctx: &$crate::behavior::Ctx,
                ) -> Option<$crate::frame::beam::MemberForces> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.state_member_forces(ctx)
                }
            );
            $crate::forward_element_behavior!(@opt $m_geometric_stiffness
                fn geometric_stiffness(&self, n: f64) -> $crate::behavior::LocalMat {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.geometric_stiffness(n)
                }
            );
            $crate::forward_element_behavior!(@opt $m_snapshot_state
                fn snapshot_state(&self) -> Box<dyn ::std::any::Any> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.snapshot_state()
                }
            );
            $crate::forward_element_behavior!(@opt $m_restore_state
                fn restore_state(&mut self, state: &dyn ::std::any::Any) {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.restore_state(state)
                }
            );
            $crate::forward_element_behavior!(@opt $m_commit_state
                fn commit_state(&mut self) {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.commit_state()
                }
            );
            $crate::forward_element_behavior!(@opt $m_revert_state
                fn revert_state(&mut self) {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.revert_state()
                }
            );
            $crate::forward_element_behavior!(@opt $m_serialize_checkpoint
                fn serialize_checkpoint(&self) -> Vec<u8> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.serialize_checkpoint()
                }
            );
            $crate::forward_element_behavior!(@opt $m_deserialize_checkpoint
                fn deserialize_checkpoint(
                    &mut self,
                    data: &[u8],
                ) -> Result<(), $crate::behavior::CheckpointError> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.deserialize_checkpoint(data)
                }
            );
            $crate::forward_element_behavior!(@opt $m_panel_moments_from
                fn panel_moments_from(&self, u_elem: &[f64]) -> Option<[f64; 2]> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.panel_moments_from(u_elem)
                }
            );
            $crate::forward_element_behavior!(@opt $m_ductility_probe
                fn ductility_probe(&self) -> Option<$crate::behavior::DuctilityProbe> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.ductility_probe()
                }
            );
            $crate::forward_element_behavior!(@opt $m_fiber_section_states
                fn fiber_section_states(
                    &self,
                ) -> Option<Vec<$crate::behavior::FiberSectionState>> {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.fiber_section_states()
                }
            );
            $crate::forward_element_behavior!(@opt $m_set_time_step
                fn set_time_step(&mut self, dt: f64) {
                    #[allow(unused_imports)]
                    use $crate::behavior::ElementBehavior as _;
                    self.$inner.set_time_step(dt)
                }
            );
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, Node,
    };

    /// 2 節点 1 部材。節点 1 は全自由度を拘束する。
    fn two_node_model() -> Model {
        let node = |id: u32, z: f64, restraint| Node {
            id: NodeId(id),
            coord: [0.0, 0.0, z],
            restraint,
            mass: None,
            story: None,
            support_spring: None,
        };
        Model {
            nodes: vec![
                node(0, 3000.0, Dof6Mask::FREE),
                node(1, 0.0, Dof6Mask::FIXED),
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: None,
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            ..Default::default()
        }
    }

    /// 節点の並び順どおりに [`DOF_PER_NODE`] 成分ずつ並び、拘束された自由度には
    /// 番兵 `usize::MAX` が入る。
    #[test]
    fn node_global_dofs_orders_by_node_and_marks_restrained() {
        let model = two_node_model();
        let dof = DofMap::build(&model);
        let g = node_global_dofs(&[NodeId(0), NodeId(1)], &dof);

        assert_eq!(g.len(), 2 * DOF_PER_NODE);
        // 自由な節点 0 は全成分が活性番号（重複なし）。
        for (d, &v) in g.iter().take(DOF_PER_NODE).enumerate() {
            assert_eq!(v, dof.active(d).unwrap() as usize);
        }
        // 全拘束の節点 1 は全成分が番兵。
        assert!(g[DOF_PER_NODE..].iter().all(|&v| v == usize::MAX));
    }

    /// 節点を渡す順がそのまま並びになる（要素の節点順に追随する契約）。
    #[test]
    fn node_global_dofs_follows_given_order() {
        let model = two_node_model();
        let dof = DofMap::build(&model);
        let forward = node_global_dofs(&[NodeId(0), NodeId(1)], &dof);
        let reversed = node_global_dofs(&[NodeId(1), NodeId(0)], &dof);

        assert_eq!(forward[..DOF_PER_NODE], reversed[DOF_PER_NODE..]);
        assert_eq!(forward[DOF_PER_NODE..], reversed[..DOF_PER_NODE]);
    }

    /// `push_*` は末尾へ足す。パネル自由度を持たない節点は番兵になる
    /// （パネルのないモデルではパネル自由度が払い出されないため）。
    #[test]
    fn push_helpers_append_to_existing_sequence() {
        let model = two_node_model();
        let dof = DofMap::build(&model);
        let mut g = SmallVec::new();
        push_node_global_dofs(&mut g, &[NodeId(0)], &dof);
        push_panel_global_dofs(&mut g, 0, &dof);

        assert_eq!(g.len(), DOF_PER_NODE + PANEL_DOF_PER_NODE);
        assert_eq!(g[..DOF_PER_NODE], node_global_dofs(&[NodeId(0)], &dof)[..]);
        assert!(g[DOF_PER_NODE..].iter().all(|&v| v == usize::MAX));
    }
}
