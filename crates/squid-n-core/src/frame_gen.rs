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
//! 生成した**部材**の断面は未割当のままとする。断面は利用者が決めるものであり、
//! もっともらしい既定断面を割り当てると、入力し忘れたまま解析が通ってしまう。
//! 解析前チェックが「断面が未割当の部材があります」で止めるため、割り当て漏れは
//! 必ず名指しされる。材料も作らない（材料はプリセットから選ぶだけのデータである）。
//!
//! **床だけは断面を作る。** 床の板厚と自重は断面からしか解決できず
//! （[`crate::model::Model::slab_thickness_of`]）、断面が無い床は解析前チェックが
//! 止めてしまうためである。板厚 150 mm の `S15` を 1 枚だけ作り、全階の床へ
//! 割り当てる。材料は割り当てないので、利用者がコンクリートを選ぶまで自重は 0 になる。

use crate::dof::{Dof, Dof6Mask};
use crate::ids::{ElemId, NodeId, StoryId};
use crate::ids::{SectionId, SlabId};
use crate::model::{
    default_story_name, Axis, AxisGroup, AxisGroupKind, AxisPlanDir, AxisSource,
    DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model,
    Node, Section, Slab, SlabUsage, Story,
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
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BaseSupport {
    /// 固定（6 自由度すべて拘束）。
    #[default]
    Fixed,
    /// ピン（並進 3 自由度のみ拘束）。
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
    /// 下から順の階高 [mm]（要素数 ＝ 生成する階の数）。
    pub story_heights: Vec<f64>,
    /// 階名（`story_heights` と同順・同数）。空文字の要素は
    /// [`default_story_name`] で補う。
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
}

/// ウィザードが作る床の断面の符号。
pub const SLAB_SECTION_NAME: &str = "S15";

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
            base_support: BaseSupport::Fixed,
            with_girders: true,
            with_slabs: true,
            slab_usage: Some(SlabUsage::Office),
            slab_thickness: 150.0,
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
        None
    }

    /// 生成される節点数・柱本数・大梁本数。ウィザードが実行前に規模を示すために使う。
    pub fn counts(&self) -> FrameCounts {
        let nx = self.x_spans.len() + 1;
        let ny = self.y_spans.len() + 1;
        let n_story = self.story_heights.len();
        let columns = nx * ny * n_story;
        let girders = if self.with_girders {
            (self.x_spans.len() * ny + self.y_spans.len() * nx) * n_story
        } else {
            0
        };
        let slabs = if self.with_slabs {
            self.x_spans.len() * self.y_spans.len() * n_story
        } else {
            0
        };
        FrameCounts {
            nodes: nx * ny * (n_story + 1),
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
    /// 床の断面（`with_slabs` のときだけ 1 枚。板厚のみで材料は未割当）。
    pub sections: Vec<Section>,
    pub slabs: Vec<Slab>,
}

/// スパンと階高から架構（節点・柱・大梁・柱脚支点・通り芯・階）を生成する。
///
/// 生成規則:
///
/// 1. **節点**は全格子点（X 通り × Y 通り × レベル）に置く。ID は
///    X → Y → レベルの順の連番で、`ID ＝ 配列添字`の不変条件を満たす。
/// 2. **柱**は各格子点で上下に隣り合うレベルを結ぶ。基部から最上階まで通す。
/// 3. **大梁**は基部より上の各レベルで、隣り合う通りの間に架ける
///    （`with_girders` が false なら作らない）。
/// 4. **柱脚支点**は最下レベルの節点へ [`FrameSpec::base_support`] の拘束を与える。
/// 5. **通り芯**は X 方向・Y 方向のグループを新設し、通り名は `{接頭辞}{番号}`
///    （座標の昇順に 1 から）とする。所属節点はその通りの全格子点。出所は
///    [`AxisSource::Manual`] とし、あとで柱位置からの自動生成を実行しても
///    作り直されないようにする。
/// 6. **階**は基部より上の各レベルに 1 つずつ作る。階名は入力を優先し、
///    空なら [`default_story_name`] で補う。所属節点・剛床・地震用重量は
///    準備計算が算定する派生値のため、ここでは空のままとする。
/// 7. **床**は基部より上の各レベルで、隣り合う通りに囲まれた格子パネルへ 1 枚ずつ
///    作る（`with_slabs` が false、または片方向の通りが 1 本のときは作らない）。
///    板厚 [`FrameSpec::slab_thickness`] の断面 [`SLAB_SECTION_NAME`] を 1 枚だけ
///    作り、全階の床で共有する。
///
/// 部材の断面・材料は割り当てない（モジュールドキュメント参照）。
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
                ref_vector: if vertical {
                    [1.0, 0.0, 0.0]
                } else {
                    [0.0, 0.0, 1.0]
                },
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
    // 大梁（基部より上の各レベルで隣り合う通りを結ぶ）。
    if spec.with_girders {
        for iz in 1..nz {
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

    // 階（基部より上の各レベル）。
    let stories = zs
        .iter()
        .enumerate()
        .skip(1)
        .map(|(iz, z)| {
            let si = iz - 1;
            let name = spec
                .story_names
                .get(si)
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .unwrap_or_else(|| default_story_name(si, nz - 1));
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

    // 床（基部より上の各レベルで、隣り合う通りに囲まれた格子パネル 1 枚ずつ）。
    // 板厚と自重は断面からしか解決できないため、床を作るなら断面もあわせて作る。
    // 材料は割り当てない（利用者がコンクリートを選ぶまで自重は 0 になる）。
    let mut sections = Vec::new();
    let mut slabs = Vec::new();
    if spec.with_slabs && nx >= 2 && ny >= 2 {
        let sec_id = SectionId(0);
        sections.push(
            crate::section_shape::SectionShape::RcSlab {
                thickness: spec.slab_thickness,
            }
            .to_section(sec_id, SLAB_SECTION_NAME.to_string()),
        );
        for iz in 1..nz {
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
        let spec = FrameSpec::default();
        let counts = spec.counts();
        let model = frame_model(&spec).unwrap();

        // 3 通り × 2 通り × 4 レベル。
        assert_eq!(model.nodes.len(), 3 * 2 * 4);
        assert_eq!(model.nodes.len(), counts.nodes);
        // 柱 = 格子点 6 × 3 層、大梁 = (2×2 + 1×3) × 3 レベル。
        assert_eq!(counts.columns, 6 * 3);
        assert_eq!(counts.girders, (2 * 2 + 3) * 3);
        assert_eq!(model.elements.len(), counts.columns + counts.girders);

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

    /// 柱脚をピンにすると並進 3 自由度だけが拘束される。
    #[test]
    fn test_pinned_base() {
        let spec = FrameSpec {
            base_support: BaseSupport::Pinned,
            ..FrameSpec::default()
        };
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

        // 階は床基準の既定名で、標高の昇順に並ぶ。最上階は屋根なので RF。
        let stories: Vec<(&str, f64)> = model
            .stories
            .iter()
            .map(|s| (s.name.as_str(), s.elevation))
            .collect();
        assert_eq!(
            stories,
            vec![("2F", 4000.0), ("3F", 7500.0), ("RF", 11000.0)]
        );
    }

    /// 階名を入力すればそれを使い、空欄は既定名で補う。
    #[test]
    fn test_story_names_from_spec() {
        let spec = FrameSpec {
            story_names: vec!["2FL".into(), "  ".into(), "RFL".into()],
            ..FrameSpec::default()
        };
        let model = frame_model(&spec).unwrap();
        let names: Vec<&str> = model.stories.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, vec!["2FL", "3F", "RFL"]);
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
    }

    /// 床は各階の各格子パネルに 1 枚ずつ作り、板厚 150 mm の断面 `S15` を共有する。
    ///
    /// 床の板厚と自重は断面からしか解決できないため、床を作るなら断面もあわせて作る。
    /// 材料は割り当てない（利用者がコンクリートを選ぶまで自重は 0 になる）。
    #[test]
    fn test_generated_slabs_share_one_section() {
        let spec = FrameSpec::default();
        let model = frame_model(&spec).unwrap();
        // 2×1 スパン × 3 階 = 6 枚。
        assert_eq!(model.slabs.len(), 2 * 3);
        assert_eq!(model.slabs.len(), spec.counts().slabs);
        assert_eq!(model.sections.len(), 1, "床の断面は 1 枚だけ");
        let sec = &model.sections[0];
        assert_eq!(sec.name, SLAB_SECTION_NAME);
        assert_eq!(sec.floor, None, "階を持たない断面として作る");
        assert_eq!(sec.thickness, Some(150.0));
        assert!(sec.material.is_none(), "材料は利用者が割り当てる");
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
            assert!(zs[0] > 0.0, "基部レベルには床を作らない");
        }
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 床を作らない設定では断面も作らない。
    #[test]
    fn test_frame_without_slabs() {
        let spec = FrameSpec {
            with_slabs: false,
            ..FrameSpec::default()
        };
        let model = frame_model(&spec).unwrap();
        assert!(model.slabs.is_empty());
        assert!(model.sections.is_empty());
        assert_eq!(spec.counts().slabs, 0);
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
            vec![("GL", 0.0), ("2F", 4000.0), ("3F", 7500.0), ("RF", 11000.0)]
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
