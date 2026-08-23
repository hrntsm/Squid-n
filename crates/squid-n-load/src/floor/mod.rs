//! スラブ面荷重の大梁・小梁・柱への分配。
//!
//! 責務ごとにサブモジュールへ分割している:
//! - [`types`] — 基本型（[`LoadShape`]・[`Cmq`]・[`LoadTarget`]・[`BeamLoad`]）と辺荷重ヘルパ
//! - [`geometry`] — 幾何ヘルパ（座標取得・距離・矩形判定・[`polygon_area`]）
//! - [`fem`] — 固定端モーメント・せん断（CMQ）の閉形式公式
//! - [`rect`] — 矩形床の分配戦略（三角形・台形／一方向／負担面積／小梁二段階）
//! - [`cantilever`] — 片持ちスラブ・出隅スラブの分配戦略
//! - [`polygon`] — 多角形床の負担面積法（最近接辺グリッドサンプリング）
//! - [`rigid_zone`] — 剛域を考慮した大梁 CMQ（[`cmq_with_rigid_zone`]）
//!
//! 本モジュールにはこれらを束ねるディスパッチャ [`distribute_slab`] を置く。

mod cantilever;
mod fem;
mod geometry;
mod polygon;
mod rect;
mod rigid_zone;
mod types;

pub use fem::{fixed_end_moments, simple_beam_moment_at, simple_reactions};
pub use geometry::{point_in_slab_boundary, polygon_area, slab_dimensions, slab_dimensions_of};
pub use rigid_zone::{cmq_with_rigid_zone, RigidZoneCmqMode, RigidZoneCmqResult};
pub use types::{BeamLoad, Cmq, LoadShape, LoadTarget};

use cantilever::{distribute_cantilever, distribute_to_node};
use geometry::boundary_coords;
use polygon::distribute_polygon;
use rect::{distribute_rect, distribute_rect_with_joists};
use squid_n_core::model::{
    DistributionMethod, FloorRegion, LoadTransfer, Model, RegionAnchor, RegionShape,
};

#[cfg(test)]
use fem::{fem_trapezoid, fem_triangle, fem_uniform};

/// 床領域の面荷重を境界（および二次部材経由の節点荷重）へ分配する。
///
/// 分岐は領域の形で決まる:
///
/// 1. **取り付き領域**（[`RegionShape::Attached`]）→ [`distribute_attached`]。
///    - 取付き先が点（出隅）: 全荷重をその節点（柱）へ集中する。荷重伝達方向にも
///      片持ち梁の取付きにも依らない（出隅の片持ちスラブの床荷重分配）。
///    - 取付き先が線 ＋ [`LoadTransfer::Anchor`]: 取付き辺へ等分布する
///      （[`distribute_cantilever`]）。
///    - 取付き先が線 ＋ [`LoadTransfer::Columns`]: 取付き線の両端へ半分ずつ集中する。
/// 2. **囲まれた領域**（[`RegionShape::Enclosed`]）
///    - 境界が矩形（[`slab_dimensions`] が `Some` を返す）かつ小梁ラインがあり
///      分配方法が `TriTrapezoid`/`OneWay` → 小梁二段階伝達
///      （[`distribute_rect_with_joists`]）。
///    - 境界が矩形 → 矩形床の分配（[`distribute_rect`]）。一方向の指定があれば
///      その方向（全体座標 X/Y）へ、なければ境界辺 0・2 が負担する。
///    - それ以外（三角形・台形・五角形などの多角形）→ 多角形の負担面積法
///      （[`distribute_polygon`]）。一方向の指定があってもこの経路へ落ちる。
///
/// いずれの経路も総和保存（Σ大梁荷重 (+Σ小梁反力・Σ柱集中荷重) = w×面積）を満たすよう
/// 設計している（床は全体座標 XY 平面内（Z一定）にあることを仮定する）。
/// 入隅の片持ちスラブは本実装では未対応。
///
/// **版なし床領域は面荷重が 0 のため、何も出力しない。**
pub fn distribute_slab(model: &Model, region: &FloorRegion) -> Vec<BeamLoad> {
    // 固定荷重 DL（版の自重＋仕上げ等）の総和を分配する。自重は断面の板厚と
    // 材料から算定する（`Model::region_dead_intensity`）。
    distribute_slab_w(model, region, model.region_dead_intensity(region))
}

/// [`distribute_slab_w`] がこの床領域で**小梁二段階伝達**（`distribute_rect_with_joists`。
/// 小梁点反力 `LoadTarget::Node` ＋境界残り `LoadTarget::Edge` を出力）を採る条件を返す。
///
/// 呼び出し側（床格子サブモデルで小梁点反力を置換したい層）が、平行小梁モデルの
/// 出力形状（Node ＋ remainder Edge）を前提にできるかを判定するために公開する。
/// この条件を満たさない領域（取り付き領域・非矩形・分配法が三角/一方向以外）では
/// 小梁は使われず全面積が Edge/集中で分配されるため、格子反力を上乗せすると
/// 二重計上になる。分岐は [`distribute_slab_w`] と厳密に一致させること。
pub fn uses_joist_distribution(model: &Model, region: &FloorRegion) -> bool {
    if region.is_attached() || region.joist_lines().is_empty() {
        return false;
    }
    if slab_dimensions(model, region).is_none() {
        return false; // 非矩形（多角形経路）。
    }
    matches!(
        region.method(),
        DistributionMethod::TriTrapezoid | DistributionMethod::OneWay
    )
}

/// 指定した面荷重強度 `w`（N/mm²）のみを床領域の境界へ分配する。
///
/// 分岐ロジックは [`distribute_slab`] と同一で、荷重源だけを引数 `w` に差し替える。
/// これにより DL（固定荷重）と LL（積載荷重）を別々の荷重ケースへ分配できる
/// （令85条1項の床用/骨組用/地震用の使い分けや、荷重組合せでの DL/LL 係数分けに用いる）。
/// `w == 0.0` の場合は空の分配結果を返す。
pub fn distribute_slab_w(model: &Model, region: &FloorRegion, w: f64) -> Vec<BeamLoad> {
    let mut loads = Vec::new();
    if w == 0.0 {
        return loads;
    }
    let Some(coords) = boundary_coords(model, region) else {
        return loads;
    };
    if coords.len() < 3 {
        return loads;
    }

    match &region.shape {
        RegionShape::Attached { anchor, .. } => {
            distribute_attached(&coords, w, *anchor, &mut loads);
            return loads;
        }
        RegionShape::Enclosed { .. } => {}
    }

    match slab_dimensions_of(&coords) {
        Some((lx, ly)) => {
            let use_joists = !region.joist_lines().is_empty()
                && matches!(
                    region.method(),
                    DistributionMethod::TriTrapezoid | DistributionMethod::OneWay
                );
            if use_joists {
                distribute_rect_with_joists(model, region, &coords, w, &mut loads);
            } else {
                distribute_rect(region, &coords, lx, ly, w, &mut loads);
            }
        }
        None => distribute_polygon(&coords, w, &mut loads),
    }

    loads
}

/// 取り付き領域（片持ちスラブ・バルコニー・出隅）の分配。
///
/// - 取付き先が点（出隅）: 全荷重をその節点（柱）へ集中する。
/// - 取付き先が線 ＋ [`LoadTransfer::Anchor`]: 取付き辺（境界の辺 0）へ等分布する。
/// - 取付き先が線 ＋ [`LoadTransfer::Columns`]: 全荷重を取付き線の両端へ半分ずつ集中する。
fn distribute_attached(
    coords: &[[f64; 3]],
    w: f64,
    anchor: RegionAnchor,
    loads: &mut Vec<BeamLoad>,
) {
    match anchor {
        RegionAnchor::Point(node) => distribute_to_node(node, coords, w, 1.0, loads),
        RegionAnchor::Line {
            nodes, transfer, ..
        } => match transfer {
            LoadTransfer::Anchor => distribute_cantilever(coords, w, loads),
            LoadTransfer::Columns => {
                distribute_to_node(nodes[0], coords, w, 0.5, loads);
                distribute_to_node(nodes[1], coords, w, 0.5, loads);
            }
        },
    }
}

#[cfg(test)]
mod tests;
