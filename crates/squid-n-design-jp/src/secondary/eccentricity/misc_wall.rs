//! 雑壁（フレーム外の壁）の剛性を n 倍法で等価剛性要素へ換算する層。
//!
//! - [`misc_wall_stiffness`] — 雑壁 1 枚の等価水平剛性 Kw'。
//! - [`sum_column_area`] — 当該層の柱断面積の和 ΣAc。
//! - [`append_misc_wall_stiffnesses`] — 雑壁を等価剛性要素として `cols` に追加。

use squid_n_core::ids::StoryId;
use squid_n_core::model::Model;

use super::core::ColumnStiffness;

// ===== 雑壁の剛性評価（n 倍法）=====

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

/// 当該層に帰属するフレーム外雑壁を n 倍法で等価剛性要素へ換算し、`cols` に
/// 追加する（剛心・ねじり剛性への寄与）。
///
/// - n 係数は `Model::stress_cfg.misc_wall_n`（`None` なら雑壁剛性を考慮しない）
/// - 帰属層: 壁の中間高さ z が（直下層 elevation, 当該層 elevation] に入る壁
/// - `Aw' = 壁の平面長さ × 壁厚`（`OutOfFrameMiscWall::thickness` 未設定の壁は対象外）
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
    if model.misc_walls.is_empty() {
        return;
    }
    let sum_ac = sum_column_area(model, story);
    if sum_ac <= 0.0 {
        return; // ΣAc = 0 → ΣKw' = 0
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

    for w in &model.misc_walls {
        let Some(t) = w.thickness else {
            continue;
        };
        let z_mid = w.start[2] + w.height * 0.5;
        if !(z_mid > below + 1e-9 && z_mid <= elev + 1e-9) {
            continue;
        }
        let dxw = w.end[0] - w.start[0];
        let dyw = w.end[1] - w.start[1];
        let len = (dxw * dxw + dyw * dyw).sqrt();
        if len <= 0.0 || t <= 0.0 {
            continue;
        }
        let aw = len * t;
        let (cx, cy) = (dxw / len, dyw / len);
        cols.push(ColumnStiffness {
            pos: [(w.start[0] + w.end[0]) * 0.5, (w.start[1] + w.end[1]) * 0.5],
            dx: misc_wall_stiffness(n, aw, sum_kx, sum_ac) * cx * cx,
            dy: misc_wall_stiffness(n, aw, sum_ky, sum_ac) * cy * cy,
        });
    }
}
