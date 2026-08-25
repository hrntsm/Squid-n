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
//! 本モジュールにはこれらを束ねるディスパッチャ [`distribute_slab`] と、床領域単位の
//! 束ね役 [`distribute_region`] を置く。
//!
//! # 床領域（区画）と床板の分配
//!
//! 床領域（大梁の 1 スパン区画）は、区画内が小梁でさらに細かい打設単位に分かれていれば
//! 複数の床板（[`Slab`]）を持つ。[`distribute_region`] は区画内の各床板を**独立に**
//! [`distribute_slab`] へ渡す。床板の境界辺が大梁でなければ（＝隣の床板と共有する小梁の辺）、
//! その辺荷重は実部材が見つからず `LoadTarget::Span` へ落ち、呼び出し側
//! （`squid-n-job::auto_loads`）が小梁の両端節点への集中荷重（単純梁反力）へ変換する。
//! 同じ小梁を挟む両側の床板がそれぞれ寄与を出すため、小梁の両端に立つ集中荷重は
//! 自然に合算される（総和保存。追加の合成処理を要らない）。
//!
//! 交差小梁の格子解析用に手入力した小梁ライン（[`FloorRegion::joists`]）があり、かつ
//! 区画が床板をちょうど 1 枚だけ持つ場合は、その床板を区画全体として扱う従来の
//! 二段階伝達・床格子サブモデルへ回す（申し送り「床領域・壁領域の再設計」Q1 決定 C。
//! この経路自体は変更しない）。区画が床板を 2 枚以上持つ場合は、代表床板以外の面積・
//! 荷重を無視してしまう（総和保存が崩れる）ため、この経路を採らず通常の
//! 床板ごとの独立分配へ落とす（[`uses_joist_distribution`]）。

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
    DistributionMethod, FloorRegion, LoadTransfer, Model, RegionAnchor, Slab, SlabShape,
};

#[cfg(test)]
use fem::{fem_trapezoid, fem_triangle, fem_uniform};

/// 床板の面荷重を境界（および二次部材経由の節点荷重）へ分配する。
///
/// 分岐は床板の形で決まる:
///
/// 1. **取り付く床板**（[`SlabShape::Attached`]）→ [`distribute_attached`]。
///    - 取付き先が点（出隅）: 全荷重をその節点（柱）へ集中する。荷重伝達方向にも
///      片持ち梁の取付きにも依らない（出隅の片持ちスラブの床荷重分配）。
///    - 取付き先が線 ＋ [`LoadTransfer::Anchor`]: 取付き辺へ等分布する
///      （[`distribute_cantilever`]）。
///    - 取付き先が線 ＋ [`LoadTransfer::Columns`]: 取付き線の両端へ半分ずつ集中する。
/// 2. **大梁または小梁で囲まれた床板**（[`SlabShape::Enclosed`]）
///    - 境界が矩形（[`slab_dimensions`] が `Some` を返す）→ 矩形床の分配
///      （[`distribute_rect`]）。一方向の指定があればその方向（全体座標 X/Y）へ、
///      なければ境界辺 0・2 が負担する。
///    - それ以外（三角形・台形・五角形などの多角形）→ 多角形の負担面積法
///      （[`distribute_polygon`]）。一方向の指定があってもこの経路へ落ちる。
///
/// いずれの経路も総和保存（Σ大梁荷重 (+Σ小梁反力・Σ柱集中荷重) = w×面積）を満たすよう
/// 設計している（床は全体座標 XY 平面内（Z一定）にあることを仮定する）。
/// 入隅の片持ちスラブは本実装では未対応。
pub fn distribute_slab(model: &Model, slab: &Slab) -> Vec<BeamLoad> {
    // 固定荷重 DL（版の自重＋仕上げ等）の総和を分配する。自重は断面の板厚と
    // 材料から算定する（`Model::slab_dead_intensity`）。
    distribute_slab_w(model, slab, model.slab_dead_intensity(slab))
}

/// 指定した面荷重強度 `w`（N/mm²）のみを床板の境界へ分配する。
///
/// 分岐ロジックは [`distribute_slab`] と同一で、荷重源だけを引数 `w` に差し替える。
/// これにより DL（固定荷重）と LL（積載荷重）を別々の荷重ケースへ分配できる
/// （令85条1項の床用/骨組用/地震用の使い分けや、荷重組合せでの DL/LL 係数分けに用いる）。
/// `w == 0.0` の場合は空の分配結果を返す。
pub fn distribute_slab_w(model: &Model, slab: &Slab, w: f64) -> Vec<BeamLoad> {
    let mut loads = Vec::new();
    if w == 0.0 {
        return loads;
    }
    let Some(coords) = boundary_coords(model, slab) else {
        return loads;
    };
    if coords.len() < 3 {
        return loads;
    }

    match &slab.shape {
        SlabShape::Attached { anchor, .. } => {
            distribute_attached(&coords, w, *anchor, &mut loads);
            return loads;
        }
        SlabShape::Enclosed { .. } => {}
    }

    match slab_dimensions_of(&coords) {
        Some((lx, ly)) => distribute_rect(slab, &coords, lx, ly, w, &mut loads),
        None => distribute_polygon(&coords, w, &mut loads),
    }

    loads
}

/// 取り付く床板（片持ちスラブ・バルコニー・出隅）の分配。
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

/// [`distribute_region`] が交差小梁の格子解析・二段階伝達（従来経路）を採る条件。
///
/// `region.joists`（手入力の小梁ライン）が空でなく、かつ区画が床板を**ちょうど 1 枚**
/// だけ持つ場合に、その床板を区画全体とみなして扱う。呼び出し側（床格子サブモデルで
/// 小梁点反力を置換したい層）が、平行小梁モデルの出力形状を前提にできるかを
/// 判定するために公開する。分岐は [`distribute_region`] と厳密に一致させること。
///
/// 床板が 2 枚以上ある区画では、この経路は代表床板（先頭）以外の床板の面積・荷重を
/// 無視してしまう（総和が保存されない）ため、代表床板 1 枚しか見ない前提が崩れる。
/// 手入力小梁ラインと複数床板（打設単位の分割）が両方ある区画は、ここで拒否して
/// 各床板を独立に分配する経路（[`distribute_region`] の else 節）へ落とす。
pub fn uses_joist_distribution(model: &Model, region: &FloorRegion) -> bool {
    if region.joist_lines().is_empty() || region.slab_ids.len() != 1 {
        return false;
    }
    let Some(slab) = region.slab_ids.first().and_then(|&id| model.slab(id)) else {
        return false;
    };
    if slab.is_attached() || slab_dimensions(model, slab).is_none() {
        return false; // 代表床板が矩形でない（多角形経路）。
    }
    matches!(
        slab.method(),
        DistributionMethod::TriTrapezoid | DistributionMethod::OneWay
    )
}

/// 局所辺インデックス（`Edge(k)`）を、床板自身の境界から引いた実節点対の
/// `Span` へ解決する。床領域は複数の床板を束ねるため、`Edge(k)` の `k` は
/// どの床板を指すかによって別々の辺を意味する。呼び出し側（`squid-n-job`）へ
/// 渡す前に、ここで床板の文脈ごと解決してしまう（`Node`/`Span` だけにする）。
fn resolve_edges_to_span(slab: &Slab, loads: Vec<BeamLoad>) -> Vec<BeamLoad> {
    loads
        .into_iter()
        .filter_map(|mut bl| match bl.target {
            LoadTarget::Edge(k) => {
                let [n0, n1] = slab.edge_nodes(k)?;
                bl.target = LoadTarget::Span([n0, n1]);
                // `push_edge` は `elem` に辺インデックスを入れている（実要素とは無関係）。
                // 番兵 `ElemId(u32::MAX)` に戻し、呼び出し側の `find_beam`／幾何割付に
                // 解決させる（さもないと辺インデックスが偶然実在する ElemId と衝突し、
                // 無関係な要素へ全荷重が載ってしまう）。
                bl.elem = squid_n_core::ids::ElemId(u32::MAX);
                Some(bl)
            }
            _ => Some(bl),
        })
        .collect()
}

/// [`distribute_slab_w`] の戻り値を [`resolve_edges_to_span`] で解決した版。
///
/// どの床領域からも参照されない床板（片持ち・バルコニー・出隅、または帰属先が
/// 見つからない浮き床板）を、床領域とは独立に分配する用途に使う
/// （`squid-n-job::auto_loads` 参照）。戻り値の `LoadTarget` は `Node`/`Span` のみ。
pub fn distribute_slab_resolved(model: &Model, slab: &Slab, w: f64) -> Vec<BeamLoad> {
    resolve_edges_to_span(slab, distribute_slab_w(model, slab, w))
}

/// 床領域（大梁の 1 スパン区画）の面荷重を、区画内の床板へ束ねて分配する。
///
/// 区画内が小梁でさらに細かい打設単位に分かれていれば、各床板を独立に
/// [`distribute_slab_w`] へ渡す（[`Self`] のモジュールドキュメント参照）。
/// `w_of` は床板ごとの面荷重強度 [N/mm²] を返す関数（DL/LL を分けるため）。
/// 戻り値の `LoadTarget` は `Node`/`Span` のみ（[`resolve_edges_to_span`]）。
///
/// 手入力の小梁ライン（[`FloorRegion::joists`]）があり、かつ区画の床板が 1 枚だけの
/// 場合は、その床板を区画全体として二段階伝達（[`distribute_rect_with_joists`]）へ回す
/// （[`uses_joist_distribution`] と同じ条件。この経路は変更しない）。床板が 2 枚以上
/// あるとこの経路は代表床板以外を無視して総和保存が崩れるため、`uses_joist_distribution`
/// が拒否し、下の通常経路（各床板を独立に分配）へ落ちる。
pub fn distribute_region(
    model: &Model,
    region: &FloorRegion,
    w_of: impl Fn(&Slab) -> f64,
) -> Vec<BeamLoad> {
    if uses_joist_distribution(model, region) {
        let Some(slab) = region.slab_ids.first().and_then(|&id| model.slab(id)) else {
            return Vec::new();
        };
        let w = w_of(slab);
        let mut loads = Vec::new();
        if w == 0.0 {
            return loads;
        }
        let Some(coords) = boundary_coords(model, slab) else {
            return loads;
        };
        distribute_rect_with_joists(model, region, &coords, w, &mut loads);
        return resolve_edges_to_span(slab, loads);
    }

    let mut loads = Vec::new();
    for &sid in &region.slab_ids {
        let Some(slab) = model.slab(sid) else {
            continue;
        };
        let slab_loads = distribute_slab_w(model, slab, w_of(slab));
        loads.extend(resolve_edges_to_span(slab, slab_loads));
    }
    loads
}

#[cfg(test)]
mod tests;
