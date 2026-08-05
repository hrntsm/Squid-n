//! 通り芯（軸）の型。
//!
//! 通り芯は日本の構造設計で各通りを識別するための呼称であり、**構造計算には
//! 一切用いない**（応力・断面算定・保有耐力のいずれにも入らない）。モデルの
//! 節点をまとまりとして名付けるためのデータとして保持する。
//!
//! - [`AxisSource`] — 通りの出所（自動生成／利用者）。
//! - [`AxisGroupKind`] — グループの幾何（平行芯／それ以外）。
//! - [`AxisPlanDir`] — 平行芯グループが表す平面上の位置の向き。
//! - [`Axis`] — 1 本の通り芯。
//! - [`AxisGroup`] — 同じ幾何規則を共有する通り芯のまとまり。
//!
//! 通り芯が持つのは所属節点だけで、要素は持たない。所属要素は「すべての材端節点が
//! その通りに属する要素」として [`super::Model::axis_elements`] が算出する
//! （算出規則を呼び出し側へ散らさないため、ヘルパーを情報源とする）。

use super::*;

/// 通り芯の出所。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum AxisSource {
    /// 柱位置からの自動生成（[`crate::axis_gen::generate_axes`]）が作った通り。
    /// 再生成のたびに作り直される。
    #[default]
    Auto,
    /// 利用者が作成・改名した通り、および ST-Bridge から取り込んだ通り。
    /// 自動生成では作り直さず、そのまま保持する。
    Manual,
}

/// 平行芯グループが表す、平面上の位置の向き。
///
/// 平行芯は「芯線に直交する向きの離れ」で位置が決まるため、離れを測る向きが
/// グローバル X 軸に沿うグループは X 方向（＝芯線は Y 軸に平行）、
/// Y 軸に沿うグループは Y 方向（＝芯線は X 軸に平行）を表す。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisPlanDir {
    X,
    Y,
}

/// 平行芯グループの向き判定に用いる、単位ベクトル成分の許容差。
const DIR_EPS: f64 = 1e-6;

/// 方向角 [度] の `(sin, cos)`。
///
/// 90° の倍数では厳密値（0 / ±1）へ丸める。`270f64.to_radians().sin()` は
/// -1 ちょうどにならないため、そのまま使うと直交グリッドの通りの離れが
/// `5999.999999999999` のような値になり、一覧表示にも ST-Bridge の書き出しにも
/// 残差が現れる。直交グリッドは通り芯の大多数を占めるため、ここで吸収する。
fn sin_cos_deg(angle_deg: f64) -> (f64, f64) {
    let quadrant = angle_deg / 90.0;
    if (quadrant - quadrant.round()).abs() < 1e-9 {
        return match (quadrant.round() as i64).rem_euclid(4) {
            0 => (0.0, 1.0),
            1 => (1.0, 0.0),
            2 => (0.0, -1.0),
            _ => (-1.0, 0.0),
        };
    }
    let rad = angle_deg.to_radians();
    (rad.sin(), rad.cos())
}

/// 通り芯グループの幾何。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum AxisGroupKind {
    /// 平行芯。原点 `origin` と芯線の方向角 `angle_deg`（度、グローバル X 軸から
    /// 反時計回り）を持ち、各通りは原点からの符号付き離れ（[`Axis::distance`]）で
    /// 位置が決まる。芯線の方向は `(cos θ, sin θ)`、離れを測る向きは
    /// それを 90° 回した `(−sin θ, cos θ)`（[`AxisGroupKind::offset_dir`]）。
    Parallel { origin: [f64; 2], angle_deg: f64 },
    /// 平行芯以外（円弧芯・放射芯・作図芯）。幾何は保持せず、所属節点の
    /// まとまりとしてのみ扱う。この種のグループの通りは [`Axis::distance`] が
    /// `None` になる。
    Other,
}

impl AxisGroupKind {
    /// 離れを測る向きの単位ベクトル `(−sin θ, cos θ)`。`Other` は `None`。
    pub fn offset_dir(&self) -> Option<[f64; 2]> {
        match *self {
            AxisGroupKind::Parallel { angle_deg, .. } => {
                let (sin, cos) = sin_cos_deg(angle_deg);
                Some([-sin, cos])
            }
            AxisGroupKind::Other => None,
        }
    }

    /// 平面上の点 `(x, y)` の、このグループにおける符号付き離れ。`Other` は `None`。
    pub fn distance_of(&self, x: f64, y: f64) -> Option<f64> {
        match *self {
            AxisGroupKind::Parallel { origin, .. } => {
                let d = self.offset_dir()?;
                Some((x - origin[0]) * d[0] + (y - origin[1]) * d[1])
            }
            AxisGroupKind::Other => None,
        }
    }

    /// このグループが表す平面上の位置の向き（[`AxisPlanDir`]）。
    /// 離れを測る向きがグローバル軸に沿っていない斜めのグループ、および
    /// `Other` は `None`。
    pub fn plan_dir(&self) -> Option<AxisPlanDir> {
        let [dx, dy] = self.offset_dir()?;
        if dx.abs() > DIR_EPS && dy.abs() < DIR_EPS {
            Some(AxisPlanDir::X)
        } else if dy.abs() > DIR_EPS && dx.abs() < DIR_EPS {
            Some(AxisPlanDir::Y)
        } else {
            None
        }
    }
}

/// 1 本の通り芯。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Axis {
    /// 通り名（`X1`・`Y2a` など）。図面・一覧での識別に用いる。
    pub name: String,
    /// 平行芯グループでの、グループ原点からの符号付き離れ [mm]。
    /// [`AxisGroupKind::Other`] のグループでは `None`。
    pub distance: Option<f64>,
    /// この通りに属する節点。節点の座標が通り線上にある保証はない
    /// （芯ずれした柱をその通りの所属とするモデル化が実務で行われるため、
    /// 所属は座標から導かず、このリストを正とする）。
    pub nodes: Vec<NodeId>,
    /// 出所。[`AxisSource::Manual`] の通りは自動生成で作り直さない。
    pub source: AxisSource,
}

/// 同じ幾何規則を共有する通り芯のまとまり（X 通り・Y 通りなど）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AxisGroup {
    /// グループ名（`X`・`Y` など）。自動生成の通り名の接頭辞にもなる。
    pub name: String,
    pub kind: AxisGroupKind,
    /// このグループの通り。[`AxisGroupKind::Parallel`] のグループでは
    /// [`Axis::distance`] の昇順に保つ（[`AxisGroup::sort_axes`]）。
    pub axes: Vec<Axis>,
}

impl AxisGroup {
    /// 平行芯グループの通りを [`Axis::distance`] の昇順へ整える
    /// （[`AxisGroupKind::Other`] のグループは並びに意味がないため何もしない）。
    ///
    /// 通りを追加・取り込みしたあとに呼び、一覧表示と ST-Bridge の書き出しが
    /// そのまま座標順になるようにする。この不変条件を保つ場所を 1 つにするため、
    /// 並べ替えは常にこのメソッドを使う。
    pub fn sort_axes(&mut self) {
        if self.kind == AxisGroupKind::Other {
            return;
        }
        // 離れを持たない通りは末尾へ寄せる（平行芯では通常は起こらないが、
        // `distance` を欠いた ST-Bridge ファイルでも順序を決められるようにする）。
        self.axes.sort_by(|a, b| match (a.distance, b.distance) {
            (Some(x), Some(y)) => x.total_cmp(&y),
            (Some(_), None) => std::cmp::Ordering::Less,
            (None, Some(_)) => std::cmp::Ordering::Greater,
            (None, None) => std::cmp::Ordering::Equal,
        });
    }
}

impl Model {
    /// 通り芯に属する要素（すべての材端節点がその通りに属する要素）の ID を、
    /// 要素 ID 昇順で返す。
    ///
    /// 通り芯自身は所属節点しか持たないため、「その通りで構成される構面の要素」は
    /// 常にこのヘルパーで求める。柱・梁・ブレースのような線材のほか、4 節点すべてが
    /// その通りに属する壁・シェルも該当する。
    /// 部材の所属階（材端節点のうち**最も高い節点**の所属階）。
    ///
    /// 階 \\(i\\) の階高区間にある柱は上端が階 \\(i\\) に属し、階 \\(i\\) のレベルにある梁も
    /// 階 \\(i\\) に属する、という数え方であり、階の主要構造種別の判定
    /// （準備計算の階生成）と同じ規則である。判定の情報源を 1 つに保つため、
    /// 部材の所属階が要るところは常にこのメソッドを使う。
    ///
    /// 節点が階に割り当てられていない場合は `None`。
    pub fn member_story(&self, elem: &ElementData) -> Option<crate::ids::StoryId> {
        elem.nodes
            .iter()
            .filter_map(|nid| self.nodes.get(nid.index()))
            .max_by(|a, b| a.coord[2].total_cmp(&b.coord[2]))
            .and_then(|n| n.story)
    }

    pub fn axis_elements(&self, axis: &Axis) -> Vec<ElemId> {
        let on_axis: std::collections::HashSet<NodeId> = axis.nodes.iter().copied().collect();
        let mut ids: Vec<ElemId> = self
            .elements
            .iter()
            .filter(|e| !e.nodes.is_empty() && e.nodes.iter().all(|n| on_axis.contains(n)))
            .map(|e| e.id)
            .collect();
        ids.sort();
        ids
    }
}
