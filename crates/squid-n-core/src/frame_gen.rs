//! 立体グリッド（通り芯 × 階レベル）と、そこからの架構生成。
//!
//! 日本の構造設計では、まず通り芯と階を決め、その交点に柱を立てて通りに沿って
//! 大梁を架ける。本モジュールはその手順をそのままデータにしたもので、2 つの役目を持つ。
//!
//! - [`SpaceGrid`] — 既存モデルの通り芯と階から立体グリッドを導く。
//!   3D ビューのグリッド描画と格子点スナップが情報源として使う。
//! - [`FrameSpec`] / [`generate_frame`] — スパンと階高の入力から、節点・柱・大梁・
//!   柱脚支点・通り芯・階を一括生成する（架構作成ウィザード）。
//!
//! 生成した**部材**の断面と材料は未割当のままとする。断面は利用者が決めるものであり、
//! もっともらしい既定断面を割り当てると、入力し忘れたまま解析が通ってしまう。
//! 解析前チェックが「断面が未割当の部材があります」で止めるため、割り当て漏れは
//! 必ず名指しされる。断面形状が決まらないかぎり材料だけあっても解析は通らないため、
//! 材料も与えない。
//!
//! **床だけは断面と材料を作る。** 床は解析対象の部材ではなく、解析上は荷重
//! （自重・積載）としてのみ効く二次部材である。板厚さえ決めれば断面が確定し、
//! 材料は自重を決める入力にすぎないので、部材に既定を与えることの危険とは別問題になる。
//! 板厚と自重は断面からしか解決できず（[`crate::model::Model::slab_thickness_of`]）、
//! 断面や材料が無い床は解析前チェックが止めてしまうため、`S15` の断面 1 枚と
//! [`FrameSpec::slab_concrete`] のコンクリート 1 つを作り、全階の床へ割り当てる。

use crate::dof::{Dof, Dof6Mask};
use crate::geom::default_local_ref_vector;
use crate::ids::{ElemId, MaterialId, NodeId, StoryId};
use crate::ids::{SectionId, SlabId};
use crate::material_grade::{material_presets, MaterialPreset};
use crate::model::{
    default_story_name, Axis, AxisGroup, AxisGroupKind, AxisPlanDir, AxisSource,
    DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material,
    Model, Node, Section, Slab, SlabUsage, Story,
};

/// 同一の格子線とみなす座標差 [mm]（[`crate::axis_gen::AXIS_TOL_MM`] と同値）。
pub const GRID_TOL_MM: f64 = crate::axis_gen::AXIS_TOL_MM;

/// 立体グリッド。平面の格子線（X 方向・Y 方向の通り芯の位置）と、鉛直の格子面
/// （基部を含む階レベル）で構成する。
///
/// 通り芯は識別のためのデータであり構造計算には用いない（[`crate::model::axis`]）。
/// 本構造体もその位置づけを引き継ぎ、**モデリングの下敷き**としてのみ使う。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SpaceGrid {
    /// X 方向の通りの位置（グローバル X 座標 [mm]。昇順・重複なし）と通り名。
    pub x_lines: Vec<GridLine>,
    /// Y 方向の通りの位置（グローバル Y 座標 [mm]。昇順・重複なし）と通り名。
    pub y_lines: Vec<GridLine>,
    /// レベル（グローバル Z 座標 [mm]。昇順・重複なし）と階名。
    /// 先頭は基部（[`Model::base_elevation`]）で、階名は `GL` とする。
    pub levels: Vec<GridLevel>,
}

/// 平面の格子線 1 本。
#[derive(Clone, Debug, PartialEq)]
pub struct GridLine {
    pub name: String,
    /// グローバル座標 [mm]（X 方向の通りは X 座標、Y 方向の通りは Y 座標）。
    pub coord: f64,
}

/// 鉛直方向の格子面 1 枚（階レベル）。
#[derive(Clone, Debug, PartialEq)]
pub struct GridLevel {
    pub name: String,
    /// 標高 [mm]。
    pub elevation: f64,
    /// 対応する階。基部レベルは `None`。
    pub story: Option<StoryId>,
}

impl SpaceGrid {
    /// 格子が平面・鉛直のどちらかで空なら、格子点を作れない。
    pub fn is_empty(&self) -> bool {
        self.x_lines.is_empty() || self.y_lines.is_empty() || self.levels.is_empty()
    }

    /// 全格子点の座標を返す（X → Y → レベルの順に走査）。
    pub fn points(&self) -> impl Iterator<Item = [f64; 3]> + '_ {
        self.x_lines.iter().flat_map(move |gx| {
            self.y_lines.iter().flat_map(move |gy| {
                self.levels
                    .iter()
                    .map(move |gz| [gx.coord, gy.coord, gz.elevation])
            })
        })
    }
}

/// モデルの通り芯と階から立体グリッドを導く。
///
/// - 平面の格子線は、離れを測る向きがグローバル軸に沿う平行芯グループ
///   （[`AxisGroupKind::plan_dir`]）から取る。斜めの平行芯グループと
///   [`AxisGroupKind::Other`]（円弧芯・放射芯・作図芯）は、直交格子として
///   表せないため対象にしない。
/// - 離れはグループの原点・方向角で測った符号付きの値なので、グローバル座標へ
///   戻してから昇順に並べる（[`AxisGroupKind::offset_dir`] の向きに沿う 1 次元の
///   位置なので、原点＋離れ×向きの該当成分がそのまま座標になる）。
/// - レベルは基部（[`Model::base_elevation`]）と各階の標高。階は標高の昇順に
///   並ぶ不変条件を持つ（[`Model::validate`]）ため、そのまま使える。
///
/// 同じ位置の通りが複数あるモデル（取り込みで重複した通り）では、[`GRID_TOL_MM`]
/// 以内の格子線を 1 本にまとめ、名前は先に現れたものを採る。
pub fn space_grid(model: &Model) -> SpaceGrid {
    let mut x_lines = plan_lines(model, AxisPlanDir::X);
    let mut y_lines = plan_lines(model, AxisPlanDir::Y);
    x_lines.sort_by(|a, b| a.coord.total_cmp(&b.coord));
    y_lines.sort_by(|a, b| a.coord.total_cmp(&b.coord));
    dedup_lines(&mut x_lines);
    dedup_lines(&mut y_lines);

    let base = model.base_elevation();
    let mut levels = vec![GridLevel {
        name: "GL".to_string(),
        elevation: base,
        story: None,
    }];
    for s in &model.stories {
        if (s.elevation - base).abs() <= GRID_TOL_MM {
            continue;
        }
        levels.push(GridLevel {
            name: s.name.clone(),
            elevation: s.elevation,
            story: Some(s.id),
        });
    }

    SpaceGrid {
        x_lines,
        y_lines,
        levels,
    }
}

/// 指定した向きの平行芯グループから、グローバル座標に直した格子線を集める。
fn plan_lines(model: &Model, dir: AxisPlanDir) -> Vec<GridLine> {
    let mut out = Vec::new();
    for g in &model.axes {
        if g.kind.plan_dir() != Some(dir) {
            continue;
        }
        let (AxisGroupKind::Parallel { origin, .. }, Some(off)) = (g.kind, g.kind.offset_dir())
        else {
            continue;
        };
        // 離れを測る向きは、この分岐ではグローバル軸に沿っている（plan_dir が Some）。
        // したがって成分は ±1 で、原点＋離れ×向き の該当成分がそのまま座標になる。
        let (o, d) = match dir {
            AxisPlanDir::X => (origin[0], off[0]),
            AxisPlanDir::Y => (origin[1], off[1]),
        };
        for a in &g.axes {
            let Some(dist) = a.distance else { continue };
            out.push(GridLine {
                name: a.name.clone(),
                coord: o + dist * d,
            });
        }
    }
    out
}

/// 昇順に並んだ格子線から、[`GRID_TOL_MM`] 以内で重なるものを畳む。
fn dedup_lines(lines: &mut Vec<GridLine>) {
    lines.dedup_by(|b, a| (b.coord - a.coord).abs() <= GRID_TOL_MM);
}

/// 柱脚（最下レベルの節点）の支持条件。
///
/// 既定はピン。基礎梁は部材として生成されるため、それによる柱脚の回転拘束は
/// 解析が直接評価する。支点に重ねて固定を与えると、基礎そのものの回転拘束を
/// 無条件に見込むことになり、出発点のモデルとしては危険側に出やすい。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BaseSupport {
    /// 固定（6 自由度すべて拘束）。
    Fixed,
    /// ピン（並進 3 自由度のみ拘束）。
    #[default]
    Pinned,
}

impl BaseSupport {
    fn mask(self) -> Dof6Mask {
        match self {
            BaseSupport::Fixed => Dof6Mask::FIXED,
            BaseSupport::Pinned => {
                let mut m = Dof6Mask::FREE;
                m.set_fixed(Dof::Ux);
                m.set_fixed(Dof::Uy);
                m.set_fixed(Dof::Uz);
                m
            }
        }
    }
}

/// 架構作成ウィザードの入力。
#[derive(Clone, Debug, PartialEq)]
pub struct FrameSpec {
    /// 平面の原点 `(x, y)` [mm]。1 本目の通りの位置になる。
    pub origin: [f64; 2],
    /// 基部の標高 [mm]。最下レベル（柱脚）になる。
    pub base_elevation: f64,
    /// X 方向のスパン [mm]（要素数 ＝ 通りの本数 − 1）。空なら通り 1 本。
    pub x_spans: Vec<f64>,
    /// Y 方向のスパン [mm]（同上）。
    pub y_spans: Vec<f64>,
    /// 下から順の階高 [mm]（要素数 ＝ 生成する**層**の数）。
    pub story_heights: Vec<f64>,
    /// 階（床）の名前。**要素数は `story_heights` より 1 つ多く**、先頭が基部の
    /// 床の名前である（階は床であり、床レベルは層より 1 つ多いため）。
    /// 空文字の要素は [`default_story_name`] で補う。
    pub story_names: Vec<String>,
    /// X 方向グループの名前（通り名の接頭辞）。
    pub x_group_name: String,
    /// Y 方向グループの名前（通り名の接頭辞）。
    pub y_group_name: String,
    /// 柱脚の支持条件。
    pub base_support: BaseSupport,
    /// 大梁を生成するか。
    pub with_girders: bool,
    /// 床を生成するか。各階の各格子パネル（隣り合う通りで囲まれた矩形）に 1 枚ずつ。
    pub with_slabs: bool,
    /// 床の室用途（積載荷重プリセット）。
    pub slab_usage: Option<SlabUsage>,
    /// 床の板厚 [mm]（断面 [`SLAB_SECTION_NAME`] の板厚になる）。
    pub slab_thickness: f64,
    /// 床のコンクリートのグレード名（[`material_presets`] の名称。既定は
    /// [`DEFAULT_SLAB_CONCRETE`]）。この材料を 1 つ作り、床の断面へ割り当てる。
    pub slab_concrete: String,
}

/// ウィザードが作る床の断面の符号。
pub const SLAB_SECTION_NAME: &str = "S15";

/// 床のコンクリートの既定グレード。
pub const DEFAULT_SLAB_CONCRETE: &str = "Fc21";

impl Default for FrameSpec {
    fn default() -> Self {
        Self {
            origin: [0.0, 0.0],
            base_elevation: 0.0,
            x_spans: vec![6000.0, 6000.0],
            y_spans: vec![6000.0],
            story_heights: vec![4000.0, 3500.0, 3500.0],
            story_names: Vec::new(),
            x_group_name: "X".to_string(),
            y_group_name: "Y".to_string(),
            base_support: BaseSupport::Pinned,
            with_girders: true,
            with_slabs: true,
            slab_usage: Some(SlabUsage::Office),
            slab_thickness: 150.0,
            slab_concrete: DEFAULT_SLAB_CONCRETE.to_string(),
        }
    }
}

impl FrameSpec {
    /// X 方向の通りの座標 [mm]（原点から各スパンを積み上げる）。
    pub fn x_coords(&self) -> Vec<f64> {
        accumulate(self.origin[0], &self.x_spans)
    }

    /// Y 方向の通りの座標 [mm]。
    pub fn y_coords(&self) -> Vec<f64> {
        accumulate(self.origin[1], &self.y_spans)
    }

    /// 基部を含むレベル [mm]（先頭が基部）。
    pub fn levels(&self) -> Vec<f64> {
        accumulate(self.base_elevation, &self.story_heights)
    }

    /// 入力の不備を日本語で返す（`None` なら生成できる）。
    ///
    /// スパン・階高は正の値でなければならない。0 以下を許すと同じ位置に節点が
    /// 重なり、長さ 0 の部材ができて剛性行列が特異になる。
    pub fn validate(&self) -> Option<String> {
        if self.story_heights.is_empty() {
            return Some("階高を 1 つ以上入力してください。".into());
        }
        if self.x_spans.iter().chain(&self.y_spans).any(|s| *s <= 0.0) {
            return Some("スパンは正の値で入力してください。".into());
        }
        if self.story_heights.iter().any(|h| *h <= 0.0) {
            return Some("階高は正の値で入力してください。".into());
        }
        if self.with_slabs && self.slab_thickness <= 0.0 {
            return Some("床の板厚は正の値で入力してください。".into());
        }
        if self.with_slabs && self.slab_concrete_preset().is_none() {
            return Some(format!(
                "床のコンクリート「{}」は標準材料にありません。",
                self.slab_concrete
            ));
        }
        None
    }

    /// 床のコンクリートに対応する標準材料プリセット。名称が一致しなければ `None`。
    fn slab_concrete_preset(&self) -> Option<MaterialPreset> {
        material_presets()
            .into_iter()
            .find(|p| p.name == self.slab_concrete)
    }

    /// 生成される節点数・柱本数・梁本数・床枚数。ウィザードが実行前に規模を示すために使う。
    pub fn counts(&self) -> FrameCounts {
        let nx = self.x_spans.len() + 1;
        let ny = self.y_spans.len() + 1;
        let n_story = self.story_heights.len();
        // レベル数 ＝ 階高の数 + 1（基部を含む）。
        let n_level = n_story + 1;
        let columns = nx * ny * n_story;
        let per_level = self.x_spans.len() * ny + self.y_spans.len() * nx;
        // 基部レベルの基礎梁は常に生成し、上のレベルの大梁は `with_girders` に従う。
        let girders = per_level * if self.with_girders { n_level } else { 1 };
        let slabs = if self.with_slabs {
            self.x_spans.len() * self.y_spans.len() * n_level
        } else {
            0
        };
        FrameCounts {
            nodes: nx * ny * n_level,
            columns,
            girders,
            slabs,
        }
    }
}

/// [`FrameSpec::counts`] の結果。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameCounts {
    pub nodes: usize,
    pub columns: usize,
    pub girders: usize,
    pub slabs: usize,
}

/// 始点から差分を積み上げた列（先頭は始点そのもの）。
fn accumulate(start: f64, steps: &[f64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(steps.len() + 1);
    let mut v = start;
    out.push(v);
    for s in steps {
        v += *s;
        out.push(v);
    }
    out
}

/// 架構の生成結果。呼び出し側が新規モデルとして読み込む。
#[derive(Clone, Debug, PartialEq)]
pub struct FrameGenResult {
    pub nodes: Vec<Node>,
    pub elements: Vec<ElementData>,
    pub axes: Vec<AxisGroup>,
    pub stories: Vec<Story>,
    /// 床の断面（`with_slabs` のときだけ 1 枚）。
    pub sections: Vec<Section>,
    /// 床のコンクリート（`with_slabs` のときだけ 1 つ。断面が参照する）。
    pub materials: Vec<Material>,
    pub slabs: Vec<Slab>,
}

/// スパンと階高から架構（節点・柱・大梁・柱脚支点・通り芯・階）を生成する。
///
/// 生成規則:
///
/// 1. **節点**は全格子点（X 通り × Y 通り × レベル）に置く。ID は
///    X → Y → レベルの順の連番で、`ID ＝ 配列添字`の不変条件を満たす。
/// 2. **柱**は各格子点で上下に隣り合うレベルを結ぶ。基部から最上階まで通す。
/// 3. **基礎梁**は基部レベルで、隣り合う通りの間に架ける。日本の建築構造では
///    基礎梁は必ず設けるため、オプションを設けず常に生成する（`with_girders` に
///    依存しない）。
/// 4. **大梁**は基部より上の各レベルで、隣り合う通りの間に架ける
///    （`with_girders` が false なら作らない）。
/// 5. **柱脚支点**は最下レベルの節点へ [`FrameSpec::base_support`] の拘束を与える。
/// 6. **通り芯**は X 方向・Y 方向のグループを新設し、通り名は `{接頭辞}{番号}`
///    （座標の昇順に 1 から）とする。所属節点はその通りの全格子点。出所は
///    [`AxisSource::Manual`] とし、あとで柱位置からの自動生成を実行しても
///    作り直されないようにする。
/// 7. **階**は全レベルに 1 つずつ作る（**先頭は基部の床**）。階は床であり、
///    その列の先頭が基部であることが `squid_n_core::model::story` の不変条件である。
///    階名は入力を優先し、空なら [`default_story_name`] で補う。所属節点・剛床・
///    地震用重量は準備計算が算定する派生値のため、ここでは空のままとする。
/// 8. **床**は全レベルで、隣り合う通りに囲まれた格子パネルへ 1 枚ずつ作る
///    （`with_slabs` が false、または片方向の通りが 1 本のときは作らない）。
///    板厚 [`FrameSpec::slab_thickness`] の断面 [`SLAB_SECTION_NAME`] を 1 枚だけ
///    作り、全階の床で共有する。断面には [`FrameSpec::slab_concrete`] の
///    コンクリートを 1 つ作って割り当てる。
///
/// 部材（柱・大梁）の断面・材料は割り当てない（モジュールドキュメント参照）。
/// 入力が不正な場合（[`FrameSpec::validate`]）はその説明を `Err` で返す。
pub fn generate_frame(spec: &FrameSpec) -> Result<FrameGenResult, String> {
    if let Some(msg) = spec.validate() {
        return Err(msg);
    }
    let xs = spec.x_coords();
    let ys = spec.y_coords();
    let zs = spec.levels();
    let (nx, ny, nz) = (xs.len(), ys.len(), zs.len());

    // 節点 ID は X → Y → レベルの順の連番。格子の添字から ID を引けるようにする。
    let nid = |ix: usize, iy: usize, iz: usize| NodeId(((ix * ny + iy) * nz + iz) as u32);

    let mut nodes = Vec::with_capacity(nx * ny * nz);
    for (ix, x) in xs.iter().enumerate() {
        for (iy, y) in ys.iter().enumerate() {
            for (iz, z) in zs.iter().enumerate() {
                nodes.push(Node {
                    id: nid(ix, iy, iz),
                    coord: [*x, *y, *z],
                    restraint: if iz == 0 {
                        spec.base_support.mask()
                    } else {
                        Dof6Mask::FREE
                    },
                    mass: None,
                    story: None,
                    support_spring: None,
                });
            }
        }
    }

    let mut elements: Vec<ElementData> = Vec::new();
    let push_member = |a: NodeId, b: NodeId, vertical: bool, elements: &mut Vec<ElementData>| {
        elements.push(ElementData {
            id: ElemId(elements.len() as u32),
            kind: ElementKind::Beam,
            nodes: [a, b].into_iter().collect(),
            // 柱は材軸が鉛直なので、局所 y 軸の基準ベクトルに鉛直を使えない。
            // 柱はグローバル X、梁はグローバル Z を基準とする（線材の局所座標系の
            // 一般的な取り方）。
            local_axis: LocalAxis {
                ref_vector: default_local_ref_vector(vertical),
            },
            section: None,
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    };

    // 柱（各格子点で上下に隣り合うレベルを結ぶ）。
    for ix in 0..nx {
        for iy in 0..ny {
            for iz in 0..nz - 1 {
                push_member(nid(ix, iy, iz), nid(ix, iy, iz + 1), true, &mut elements);
            }
        }
    }
    // 水平材（各レベルで隣り合う通りを結ぶ）。基部レベル（iz == 0）は基礎梁で、
    // 日本の建築構造では必ず設けるため常に生成する。基部より上の大梁は
    // `with_girders` に従う。
    for iz in 0..nz {
        if iz > 0 && !spec.with_girders {
            continue;
        }
        for iy in 0..ny {
            for ix in 0..nx - 1 {
                push_member(nid(ix, iy, iz), nid(ix + 1, iy, iz), false, &mut elements);
            }
        }
        for ix in 0..nx {
            for iy in 0..ny - 1 {
                push_member(nid(ix, iy, iz), nid(ix, iy + 1, iz), false, &mut elements);
            }
        }
    }

    // 通り芯。X 方向グループは離れを +X 向きに測る（方向角 270°）。
    let x_group = AxisGroup {
        name: spec.x_group_name.clone(),
        kind: AxisGroupKind::Parallel {
            origin: spec.origin,
            angle_deg: 270.0,
        },
        axes: xs
            .iter()
            .enumerate()
            .map(|(ix, x)| Axis {
                name: format!("{}{}", spec.x_group_name, ix + 1),
                distance: Some(x - spec.origin[0]),
                nodes: (0..ny)
                    .flat_map(|iy| (0..nz).map(move |iz| nid(ix, iy, iz)))
                    .collect(),
                source: AxisSource::Manual,
            })
            .collect(),
    };
    let y_group = AxisGroup {
        name: spec.y_group_name.clone(),
        kind: AxisGroupKind::Parallel {
            origin: spec.origin,
            angle_deg: 0.0,
        },
        axes: ys
            .iter()
            .enumerate()
            .map(|(iy, y)| Axis {
                name: format!("{}{}", spec.y_group_name, iy + 1),
                distance: Some(y - spec.origin[1]),
                nodes: (0..nx)
                    .flat_map(|ix| (0..nz).map(move |iz| nid(ix, iy, iz)))
                    .collect(),
                source: AxisSource::Manual,
            })
            .collect(),
    };

    // 階（全レベル。先頭が基部の床）。階は床であり、基部の床も階として作る
    // （`squid_n_core::model::story` の不変条件）。
    let stories = zs
        .iter()
        .enumerate()
        .map(|(si, z)| {
            let name = spec
                .story_names
                .get(si)
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .unwrap_or_else(|| default_story_name(si));
            Story {
                id: StoryId(si as u32),
                name,
                elevation: *z,
                node_ids: Vec::new(),
                seismic_weight: None,
                weight_override: None,
                structure: Default::default(),
                level_kind: Default::default(),
            }
        })
        .collect();

    // 床（全レベルで、隣り合う通りに囲まれた格子パネル 1 枚ずつ）。
    // 板厚と自重は断面と材料からしか解決できないため、床を作るなら断面とコンクリートも
    // あわせて作る（モジュールドキュメント参照）。
    let mut sections = Vec::new();
    let mut materials = Vec::new();
    let mut slabs = Vec::new();
    if spec.with_slabs && nx >= 2 && ny >= 2 {
        let sec_id = SectionId(0);
        let mat_id = MaterialId(0);
        // `validate` が名称を確かめているため、ここでプリセットは必ず見つかる。
        let preset = spec
            .slab_concrete_preset()
            .expect("床のコンクリートは validate で検査済み");
        materials.push(Material {
            id: mat_id,
            name: preset.name.to_string(),
            category: preset.category,
            young: preset.young,
            poisson: preset.poisson,
            density: preset.density,
            shear: None,
            fc: preset.fc,
            fy: preset.fy,
            concrete_class: Default::default(),
            strength_factor: None,
        });
        let mut section = crate::section_shape::SectionShape::RcSlab {
            thickness: spec.slab_thickness,
        }
        .to_section(sec_id, SLAB_SECTION_NAME.to_string());
        section.material = Some(mat_id);
        sections.push(section);
        for iz in 0..nz {
            for ix in 0..nx - 1 {
                for iy in 0..ny - 1 {
                    // 境界は反時計回り（面積算定の巻き方向をそろえる）。
                    slabs.push(Slab {
                        id: SlabId(slabs.len() as u32),
                        boundary: vec![
                            nid(ix, iy, iz),
                            nid(ix + 1, iy, iz),
                            nid(ix + 1, iy + 1, iz),
                            nid(ix, iy + 1, iz),
                        ],
                        joists: Vec::new(),
                        loads: Vec::new(),
                        method: DistributionMethod::TriTrapezoid,
                        kind: Default::default(),
                        one_way: None,
                        edge_supported: None,
                        usage: spec.slab_usage,
                        section: Some(sec_id),
                        secondary_joist_ids: Vec::new(),
                    });
                }
            }
        }
    }

    Ok(FrameGenResult {
        nodes,
        elements,
        axes: vec![x_group, y_group],
        stories,
        sections,
        materials,
        slabs,
    })
}

/// 生成した架構を標準の荷重ケース付きの新規モデルとして組み立てる。
///
/// 架構作成ウィザードは新規モデルを作る操作なので、既存モデルへの追記ではなく
/// モデルの差し替えとして扱う（呼び出し側が undo 履歴ごと入れ替える）。
pub fn frame_model(spec: &FrameSpec) -> Result<Model, String> {
    let gen = generate_frame(spec)?;
    Ok(Model {
        nodes: gen.nodes,
        elements: gen.elements,
        axes: gen.axes,
        stories: gen.stories,
        sections: gen.sections,
        materials: gen.materials,
        slabs: gen.slabs,
        ..Model::with_default_load_cases()
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 2×1 スパン・3 階の架構が、格子どおりの節点・柱・大梁と柱脚支点を持つ。
    #[test]
    fn test_generate_frame_counts_and_supports() {
        let spec = FrameSpec {
            base_support: BaseSupport::Fixed,
            ..FrameSpec::default()
        };
        let counts = spec.counts();
        let model = frame_model(&spec).unwrap();

        // 3 通り × 2 通り × 4 レベル。
        assert_eq!(model.nodes.len(), 3 * 2 * 4);
        assert_eq!(model.nodes.len(), counts.nodes);
        // 柱 = 格子点 6 × 3 層。梁 = (2×2 + 1×3) × 4 レベル
        //（基部レベルの基礎梁を含む。基礎梁は常に生成する）。
        assert_eq!(counts.columns, 6 * 3);
        assert_eq!(counts.girders, (2 * 2 + 3) * 4);
        assert_eq!(model.elements.len(), counts.columns + counts.girders);

        // 基礎梁は基部レベルに架かる。
        let base_beams = model
            .elements
            .iter()
            .filter(|e| {
                e.nodes.len() == 2
                    && e.nodes
                        .iter()
                        .all(|n| model.nodes[n.index()].coord[2] == 0.0)
            })
            .count();
        assert_eq!(base_beams, 2 * 2 + 3, "基礎梁が基部レベルに架かる");

        // 柱脚は基部レベルの 6 節点だけが固定。
        let fixed: Vec<&Node> = model
            .nodes
            .iter()
            .filter(|n| n.restraint == Dof6Mask::FIXED)
            .collect();
        assert_eq!(fixed.len(), 6);
        assert!(fixed.iter().all(|n| n.coord[2] == 0.0));

        // ID ＝ 配列添字・参照整合。
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 部材の断面は割り当てない（利用者が決めるため）。
    #[test]
    fn test_generated_members_have_no_section() {
        let model = frame_model(&FrameSpec::default()).unwrap();
        assert!(model.elements.iter().all(|e| e.section.is_none()));
    }

    /// 柱脚の既定はピンで、並進 3 自由度だけが拘束される。
    #[test]
    fn test_pinned_base_is_default() {
        let spec = FrameSpec::default();
        assert_eq!(spec.base_support, BaseSupport::Pinned);
        let model = frame_model(&spec).unwrap();
        let base = model.nodes.iter().find(|n| n.coord[2] == 0.0).unwrap();
        assert!(base.restraint.is_fixed(Dof::Ux));
        assert!(base.restraint.is_fixed(Dof::Uz));
        assert!(!base.restraint.is_fixed(Dof::Rx));
    }

    /// 通り芯と階が入力どおりに作られ、通り名は座標の昇順に 1 から振られる。
    #[test]
    fn test_generated_axes_and_stories() {
        let model = frame_model(&FrameSpec::default()).unwrap();
        let names: Vec<&str> = model.axes[0].axes.iter().map(|a| a.name.as_str()).collect();
        assert_eq!(names, vec!["X1", "X2", "X3"]);
        let dists: Vec<f64> = model.axes[0]
            .axes
            .iter()
            .filter_map(|a| a.distance)
            .collect();
        assert_eq!(dists, vec![0.0, 6000.0, 12000.0]);
        let names: Vec<&str> = model.axes[1].axes.iter().map(|a| a.name.as_str()).collect();
        assert_eq!(names, vec!["Y1", "Y2"]);

        // 階は床であり、先頭は基部の床。床基準の既定名で標高の昇順に並ぶ。
        let stories: Vec<(&str, f64)> = model
            .stories
            .iter()
            .map(|s| (s.name.as_str(), s.elevation))
            .collect();
        assert_eq!(
            stories,
            vec![("1F", 0.0), ("2F", 4000.0), ("3F", 7500.0), ("4F", 11000.0)]
        );
        // 不変条件: 階の列の先頭は基部。
        assert_eq!(model.stories[0].elevation, model.base_elevation());
        // 3 階建ての層は 3 つ。名前は下端の階（法令の「i 階」）。
        let layers: Vec<(String, f64)> = model
            .layers()
            .into_iter()
            .map(|l| (l.name, l.height))
            .collect();
        assert_eq!(
            layers,
            vec![
                ("1F".to_string(), 4000.0),
                ("2F".to_string(), 3500.0),
                ("3F".to_string(), 3500.0)
            ]
        );
    }

    /// 階名を入力すればそれを使い、空欄は既定名で補う。
    /// 階名は床ごとなので、階高より 1 つ多く入力できる（先頭が基部の床）。
    #[test]
    fn test_story_names_from_spec() {
        let spec = FrameSpec {
            story_names: vec!["GL".into(), "2FL".into(), "  ".into(), "RFL".into()],
            ..FrameSpec::default()
        };
        let model = frame_model(&spec).unwrap();
        let names: Vec<&str> = model.stories.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, vec!["GL", "2FL", "3F", "RFL"]);
    }

    /// スパン・階高が 0 以下の入力は、長さ 0 の部材を作らずエラーにする。
    #[test]
    fn test_invalid_spec_is_rejected() {
        let zero_span = FrameSpec {
            x_spans: vec![6000.0, 0.0],
            ..FrameSpec::default()
        };
        assert!(frame_model(&zero_span).is_err());
        let no_story = FrameSpec {
            story_heights: Vec::new(),
            ..FrameSpec::default()
        };
        assert!(frame_model(&no_story).is_err());
        let zero_thickness = FrameSpec {
            slab_thickness: 0.0,
            ..FrameSpec::default()
        };
        assert!(frame_model(&zero_thickness).is_err());
        // 標準材料にないグレードは、床の材料が決まらないため生成しない。
        let unknown_concrete = FrameSpec {
            slab_concrete: "Fc999".into(),
            ..FrameSpec::default()
        };
        assert!(frame_model(&unknown_concrete).is_err());
        // 床を作らないなら床のコンクリートは使わないので、名称は問わない。
        let no_slabs = FrameSpec {
            with_slabs: false,
            slab_concrete: String::new(),
            ..FrameSpec::default()
        };
        assert!(frame_model(&no_slabs).is_ok());
    }

    /// 床は各レベルの各格子パネルに 1 枚ずつ作り、板厚 150 mm の断面 `S15` を共有する。
    ///
    /// 床の板厚と自重は断面と材料からしか解決できないため、床を作るなら断面と
    /// コンクリートもあわせて作る。
    #[test]
    fn test_generated_slabs_share_one_section() {
        let spec = FrameSpec::default();
        let model = frame_model(&spec).unwrap();
        // 2×1 スパン × 4 レベル（基部を含む）= 8 枚。
        assert_eq!(model.slabs.len(), 2 * 4);
        assert_eq!(model.slabs.len(), spec.counts().slabs);
        assert_eq!(model.sections.len(), 1, "床の断面は 1 枚だけ");
        let sec = &model.sections[0];
        assert_eq!(sec.name, SLAB_SECTION_NAME);
        assert_eq!(sec.floor, None, "階を持たない断面として作る");
        assert_eq!(sec.thickness, Some(150.0));
        assert_eq!(sec.material, Some(crate::ids::MaterialId(0)));
        assert!(model
            .slabs
            .iter()
            .all(|sl| sl.section == Some(crate::ids::SectionId(0))));
        assert!(model.slabs.iter().all(|sl| sl.usage == spec.slab_usage));
        for sl in &model.slabs {
            assert_eq!(sl.boundary.len(), 4);
            let zs: Vec<f64> = sl
                .boundary
                .iter()
                .map(|n| model.nodes[n.index()].coord[2])
                .collect();
            assert!(zs.windows(2).all(|w| w[0] == w[1]), "床は水平");
        }
        assert!(
            model
                .slabs
                .iter()
                .any(|sl| model.nodes[sl.boundary[0].index()].coord[2] == 0.0),
            "基部レベルにも床を作る"
        );
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 床のコンクリートは既定で Fc21 の 1 つだけを作り、標準材料の規格値を持つ。
    ///
    /// 床は解析対象外の二次部材で、材料は自重を決める入力である。既定のまま生成しても
    /// 自重が入り、解析前チェックの「断面に材料が未割当」で止まらない。
    #[test]
    fn test_slab_concrete_is_created() {
        let model = frame_model(&FrameSpec::default()).unwrap();
        assert_eq!(model.materials.len(), 1, "床のコンクリートだけを作る");
        let mat = &model.materials[0];
        assert_eq!(mat.name, DEFAULT_SLAB_CONCRETE);
        assert_eq!(mat.category, crate::model::MaterialCategory::Concrete);
        assert_eq!(mat.fc, Some(21.0));
        assert!(mat.density > 0.0, "自重が 0 にならない");
        assert!(model.validate().is_ok(), "{:?}", model.validate());

        // グレードを変えれば、その規格値の材料になる。
        let spec = FrameSpec {
            slab_concrete: "Fc30".into(),
            ..FrameSpec::default()
        };
        let model = frame_model(&spec).unwrap();
        assert_eq!(model.materials[0].name, "Fc30");
        assert_eq!(model.materials[0].fc, Some(30.0));
    }

    /// 床を作らない設定では断面もコンクリートも作らない。
    #[test]
    fn test_frame_without_slabs() {
        let spec = FrameSpec {
            with_slabs: false,
            ..FrameSpec::default()
        };
        let model = frame_model(&spec).unwrap();
        assert!(model.slabs.is_empty());
        assert!(model.sections.is_empty());
        assert!(model.materials.is_empty());
        assert_eq!(spec.counts().slabs, 0);
    }

    /// 基礎梁は常に生成する（オプションを設けない）。大梁を作らない設定でも残る。
    ///
    /// 日本の RC・S 造では基礎梁は必ず設けるため、基礎梁のない架構は出発点の
    /// モデルとして不正である。
    #[test]
    fn test_foundation_girders_are_always_generated() {
        let spec = FrameSpec {
            with_girders: false,
            ..FrameSpec::default()
        };
        let model = frame_model(&spec).unwrap();
        let at_base = |e: &crate::model::ElementData| {
            e.nodes.len() == 2
                && e.nodes
                    .iter()
                    .all(|n| model.nodes[n.index()].coord[2] == 0.0)
        };
        let base_beams = model.elements.iter().filter(|e| at_base(e)).count();
        assert_eq!(base_beams, 2 * 2 + 3, "大梁 OFF でも基礎梁は作る");
        assert_eq!(spec.counts().girders, base_beams);
        // 基部より上には梁がない。
        let upper = model.elements.iter().filter(|e| {
            e.nodes.len() == 2 && {
                let a = model.nodes[e.nodes[0].index()].coord[2];
                let b = model.nodes[e.nodes[1].index()].coord[2];
                (a - b).abs() < 1e-9 && a > 0.0
            }
        });
        assert_eq!(upper.count(), 0, "大梁 OFF なら基部より上に梁はない");
    }

    /// 伏図（階の切り出し）が、準備計算を通していない生成直後のモデルでも成立する。
    ///
    /// 帰属は幾何（[`Model::node_stories`]）から引くため、`Node::story` が
    /// 未設定でも階を指定すれば部材が出る。基部の階には柱脚と基礎梁が属する。
    #[test]
    fn test_generated_frame_is_visible_in_story_frames() {
        let model = frame_model(&FrameSpec::default()).unwrap();
        assert!(
            model.nodes.iter().all(|n| n.story.is_none()),
            "前提: 生成直後は所属階が未設定（準備計算が埋める派生値）"
        );
        for story in &model.stories {
            let frame =
                crate::frame::build_frame(&model, crate::frame::FrameTarget::Story(story.id))
                    .expect("階の構面を切り出せる");
            assert!(
                frame.elem_count() > 0,
                "階 {} の伏図に部材が出る",
                story.name
            );
        }
        // 基部の階には基礎梁が属する。
        let base = crate::frame::build_frame(
            &model,
            crate::frame::FrameTarget::Story(model.stories[0].id),
        )
        .unwrap();
        let base_beams = model
            .elements
            .iter()
            .enumerate()
            .filter(|(i, e)| {
                base.elem_on[*i]
                    && e.nodes.len() == 2
                    && e.nodes
                        .iter()
                        .all(|n| model.nodes[n.index()].coord[2] == 0.0)
            })
            .count();
        assert_eq!(base_beams, 2 * 2 + 3, "基礎伏図に基礎梁が出る");
    }

    /// 片方向の通りが 1 本だけなら格子パネルができないため、床は作らない。
    #[test]
    fn test_no_slabs_without_panel() {
        let spec = FrameSpec {
            y_spans: Vec::new(),
            ..FrameSpec::default()
        };
        let model = frame_model(&spec).unwrap();
        assert!(model.slabs.is_empty());
        assert!(model.sections.is_empty());
        assert!(model.materials.is_empty());
    }

    /// 生成した架構の通り芯と階から、元の格子が復元できる。
    /// 3D ビューのグリッド描画・スナップはこの復元結果を使う。
    #[test]
    fn test_space_grid_round_trips_generated_frame() {
        let model = frame_model(&FrameSpec::default()).unwrap();
        let grid = super::space_grid(&model);
        let xs: Vec<f64> = grid.x_lines.iter().map(|l| l.coord).collect();
        assert_eq!(xs, vec![0.0, 6000.0, 12000.0]);
        let ys: Vec<f64> = grid.y_lines.iter().map(|l| l.coord).collect();
        assert_eq!(ys, vec![0.0, 6000.0]);
        let levels: Vec<(&str, f64)> = grid
            .levels
            .iter()
            .map(|l| (l.name.as_str(), l.elevation))
            .collect();
        assert_eq!(
            levels,
            vec![("GL", 0.0), ("2F", 4000.0), ("3F", 7500.0), ("4F", 11000.0)]
        );
        // 格子点は 3 × 2 × 4。
        assert_eq!(grid.points().count(), 24);
        assert!(!grid.is_empty());
    }

    /// 通り芯を持たないモデルの格子は空（描く格子がない）。
    #[test]
    fn test_space_grid_without_axes_is_empty() {
        let model = Model::default();
        assert!(super::space_grid(&model).is_empty());
    }

    /// 方向角 90°（離れを -X 向きに測る）のグループでも、グローバル座標へ
    /// 正しく直してから昇順に並べる。
    #[test]
    fn test_space_grid_handles_reversed_offset_direction() {
        let mut model = frame_model(&FrameSpec::default()).unwrap();
        model.axes = vec![
            AxisGroup {
                name: "X".into(),
                kind: AxisGroupKind::Parallel {
                    origin: [0.0, 0.0],
                    angle_deg: 90.0,
                },
                axes: vec![
                    Axis {
                        name: "X3".into(),
                        distance: Some(-12000.0),
                        nodes: Vec::new(),
                        source: AxisSource::Manual,
                    },
                    Axis {
                        name: "X1".into(),
                        distance: Some(0.0),
                        nodes: Vec::new(),
                        source: AxisSource::Manual,
                    },
                ],
            },
            AxisGroup {
                name: "Y".into(),
                kind: AxisGroupKind::Parallel {
                    origin: [0.0, 0.0],
                    angle_deg: 0.0,
                },
                axes: vec![Axis {
                    name: "Y1".into(),
                    distance: Some(0.0),
                    nodes: Vec::new(),
                    source: AxisSource::Manual,
                }],
            },
        ];
        let grid = super::space_grid(&model);
        let xs: Vec<(&str, f64)> = grid
            .x_lines
            .iter()
            .map(|l| (l.name.as_str(), l.coord))
            .collect();
        assert_eq!(xs, vec![("X1", 0.0), ("X3", 12000.0)]);
    }

    /// 同じ位置の通りが重複しているモデルでは、格子線を 1 本にまとめる。
    #[test]
    fn test_space_grid_merges_duplicate_lines() {
        let mut model = frame_model(&FrameSpec::default()).unwrap();
        let dup = model.axes[0].axes[1].clone();
        model.axes[0].axes.push(Axis {
            name: "X2b".into(),
            ..dup
        });
        let grid = super::space_grid(&model);
        let xs: Vec<(&str, f64)> = grid
            .x_lines
            .iter()
            .map(|l| (l.name.as_str(), l.coord))
            .collect();
        assert_eq!(xs, vec![("X1", 0.0), ("X2", 6000.0), ("X3", 12000.0)]);
    }
}
