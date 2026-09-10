//! 雑壁（柱梁に囲まれない自立壁）の剛性を n 倍法で等価剛性要素へ換算する層。
//!
//! - [`misc_wall_stiffness`] — 雑壁 1 枚の等価水平剛性 Kw'。
//! - [`sum_column_area`] — 当該層の柱断面積の和 ΣAc。
//! - [`append_misc_wall_stiffnesses`] — 雑壁を等価剛性要素として `cols` に追加。
//!
//! 対象は自立壁（立ち上がりが直上の階レベルに達しているもの）だけである。
//! 腰壁・垂壁・パラペットは断面性能への算入経路が受け持つため、ここでは数えない。

use squid_n_core::ids::StoryId;
use squid_n_core::model::{Model, RegionAnchor, WallPlate, WallPlateShape, DIAPHRAGM_LEVEL_TOL_MM};

use super::core::ColumnStiffness;

/// 雑壁 1 枚の等価水平剛性 `Kw' = n·Aw'·ΣKc/ΣAc`。
///
/// - `n`: 雑壁の剛性を柱の剛性から求める場合の係数（入力値）
/// - `aw`: 雑壁の断面積 Aw' [mm²]
/// - `sum_kc`: 当該階の柱の剛性の和 ΣKc
/// - `sum_ac`: 当該階の柱の断面積の和 ΣAc [mm²]（0 の場合は Kw' = 0）
pub fn misc_wall_stiffness(n: f64, aw: f64, sum_kc: f64, sum_ac: f64) -> f64 {
    if sum_ac <= 0.0 {
        return 0.0;
    }
    n * aw * sum_kc / sum_ac
}

/// 当該層の柱の断面積の和 ΣAc [mm²]。
pub fn sum_column_area(model: &Model, story: StoryId) -> f64 {
    let mut sum = 0.0;
    crate::secondary::eccentricity_analysis::for_each_story_column(
        model,
        story,
        |elem, _top, _bot| {
            if let Some(sid) = elem.section {
                sum += model.sections[sid.index()].area;
            }
        },
    );
    sum
}

/// n 倍法の対象になる自立壁 1 枚の幾何。
struct SelfStandingStiffness {
    /// 壁の平面中点 [mm]。
    pos: [f64; 2],
    /// 壁面内方向の方向余弦 (cx, cy)。
    dir: [f64; 2],
    /// 断面積 Aw' [mm²]（平面長さ × 構造厚）。
    aw: f64,
    /// 帰属層の判定に使う中間高さ [mm]。
    z_mid: f64,
}

/// 壁版が n 倍法の対象になるなら、その幾何を返す。
///
/// 対象は「断面が割り当たっていて、立ち上がりが直上の階レベルに達している自立壁」
/// である（モジュール doc 参照）。
///
/// `Aw'` の厚さは断面の板厚（構造厚）から引く。仕上げ・増打ち
/// （[`WallPlate::loads`]）は荷重としてのみ効くもので、打ち継ぎで一体性が保証されず
/// 構造厚には含めないため、剛性の算定にも入れない。
fn self_standing_stiffness(model: &Model, plate: &WallPlate) -> Option<SelfStandingStiffness> {
    let WallPlateShape::Attached {
        anchor: RegionAnchor::FloorRegion { nodes },
        ..
    } = &plate.shape
    else {
        return None;
    };
    let t = model.wall_plate_thickness(plate)?;
    let extent = model.wall_plate_extent(plate)?;
    let a = model.nodes.get(nodes[0].index())?.coord;
    let b = model.nodes.get(nodes[1].index())?.coord;
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len = dx.hypot(dy);
    if len <= 0.0 || t <= 0.0 {
        return None;
    }
    let z_base = (a[2] + b[2]) / 2.0;
    let story_height = model.story_height_above(z_base)?;
    let reach = extent[0].min(extent[1]);
    if reach < story_height - DIAPHRAGM_LEVEL_TOL_MM {
        return None;
    }
    Some(SelfStandingStiffness {
        pos: [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0],
        dir: [dx / len, dy / len],
        aw: len * t,
        z_mid: z_base + story_height / 2.0,
    })
}

/// 当該層に帰属する自立壁を n 倍法で等価剛性要素へ換算し、`cols` に追加する
/// （剛心・ねじり剛性への寄与）。
///
/// - n 係数は `Model::stress_cfg.misc_wall_n`（`None` なら雑壁剛性を考慮しない）
/// - 帰属層: 壁の中間高さ z が（直下層 elevation, 当該層 elevation] に入る壁
/// - `Aw' = 壁の平面長さ × 構造厚`（[`self_standing_stiffness`] の対象外は無視）
/// - 方向別に `Kw'x = n·Aw'·ΣKc,x/ΣAc`, `Kw'y = n·Aw'·ΣKc,y/ΣAc` を求め、
///   壁面内方向の方向余弦 (cx, cy) で `dx = Kw'x·cx²`, `dy = Kw'y·cy²` として
///   壁の平面中点に置く。ΣAc = 0 の場合は Kw' = 0（0 除算回避）。
pub fn append_misc_wall_stiffnesses(
    model: &Model,
    story: StoryId,
    cols: &mut Vec<ColumnStiffness>,
) {
    let Some(n) = model.stress_cfg.misc_wall_n else {
        return;
    };
    if model.wall_plates.is_empty() {
        return;
    }
    let sum_ac = sum_column_area(model, story);
    if sum_ac <= 0.0 {
        return;
    }
    let sum_kx: f64 = cols.iter().map(|c| c.dx).sum();
    let sum_ky: f64 = cols.iter().map(|c| c.dy).sum();

    let idx = story.index();
    let Some(elev) = model.stories.get(idx).map(|s| s.elevation) else {
        return;
    };
    let below = if idx == 0 {
        f64::NEG_INFINITY
    } else {
        model.stories[idx - 1].elevation
    };

    for plate in &model.wall_plates {
        let Some(w) = self_standing_stiffness(model, plate) else {
            continue;
        };
        if !(w.z_mid > below + 1e-9 && w.z_mid <= elev + 1e-9) {
            continue;
        }
        let [cx, cy] = w.dir;
        cols.push(ColumnStiffness {
            pos: w.pos,
            dx: misc_wall_stiffness(n, w.aw, sum_kx, sum_ac) * cx * cx,
            dy: misc_wall_stiffness(n, w.aw, sum_ky, sum_ac) * cy * cy,
        });
    }
}
