//! 支持部材で囲まれた床板・壁版の割当領域。

use super::{ElementKind, Model, Slab, SlabShape, WallPlate, WallPlateShape};
use crate::error::CoreError;
use crate::ids::{
    ElemId, FloorPlateAssignmentRegionId, NodeId, SecondaryMemberId, SlabId,
    WallPlateAssignmentRegionId, WallPlateId,
};
use std::collections::{HashMap, HashSet};

/// 割当領域へ割り当てる版の状態。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PlateAssignment<Id> {
    #[default]
    Unset,
    NoPlate,
    Plate(Id),
}

impl<Id: Copy> PlateAssignment<Id> {
    /// 割り当てられた版 ID。未設定・版なしは `None`。
    pub fn plate(self) -> Option<Id> {
        match self {
            PlateAssignment::Plate(id) => Some(id),
            PlateAssignment::Unset | PlateAssignment::NoPlate => None,
        }
    }

    /// 未設定か。
    pub fn is_unset(self) -> bool {
        matches!(self, PlateAssignment::Unset)
    }

    /// 版なしが明示されているか。
    pub fn is_no_plate(self) -> bool {
        matches!(self, PlateAssignment::NoPlate)
    }
}

/// 割当領域の境界を構成する支持部材。
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub enum SupportMemberId {
    Primary(ElemId),
    Secondary(SecondaryMemberId),
}

/// 二次部材の支持端の取付き位置。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SecondaryMemberAnchor {
    pub support: SupportMemberId,
    /// 支持部材の材軸始端を 0、終端を 1 とする無次元位置。
    pub position: f64,
}

/// 二次部材の両端。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SecondaryMemberEnds {
    Supported([SecondaryMemberAnchor; 2]),
    Cantilever {
        support: SecondaryMemberAnchor,
        /// 支持端から自由端への親構面内ベクトル [mm]。
        free_end_vector: [f64; 2],
    },
    /// 支持部材アンカーへ解決できなかった両端の座標 [mm]。重量・材軸長を欠落させない
    /// ため生座標を保持し、荷重の伝達はせず解析前チェックでエラーにする。
    Detached([[f64; 3]; 2]),
}

impl SecondaryMemberEnds {
    /// 支持端の取付き位置。片持ちは支持端 1 つのみ。未解決の生座標は 0 個。
    pub fn anchors(&self) -> &[SecondaryMemberAnchor] {
        match self {
            SecondaryMemberEnds::Supported(anchors) => anchors,
            SecondaryMemberEnds::Cantilever { support, .. } => std::slice::from_ref(support),
            SecondaryMemberEnds::Detached(_) => &[],
        }
    }
}

/// 面走査へ渡す支持部材の構面内線分。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SupportMemberSegment {
    pub support: SupportMemberId,
    /// `start` と `end` に対応する支持部材材軸上の無次元位置。
    pub axis_span: [f64; 2],
    /// 親構面の 2D 座標 [mm]。
    pub start: [f64; 2],
    /// 親構面の 2D 座標 [mm]。
    pub end: [f64; 2],
}

/// 割当領域の有向境界辺。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SupportBoundary {
    pub support: SupportMemberId,
    /// 境界の進行方向に対応する支持部材材軸上の無次元区間。
    pub span: [f64; 2],
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FloorPlateAssignmentRegion {
    pub id: FloorPlateAssignmentRegionId,
    pub boundary: Vec<SupportBoundary>,
    pub assignment: PlateAssignment<SlabId>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WallPlateAssignmentRegion {
    pub id: WallPlateAssignmentRegionId,
    pub boundary: Vec<SupportBoundary>,
    pub assignment: PlateAssignment<WallPlateId>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FloorPlateAssignmentRegions {
    next_id: u32,
    pub regions: Vec<FloorPlateAssignmentRegion>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WallPlateAssignmentRegions {
    next_id: u32,
    pub regions: Vec<WallPlateAssignmentRegion>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PlateAssignmentRegionRebuildReport {
    pub regions: usize,
    pub preserved: usize,
    pub created_unset: usize,
    pub removed: usize,
    pub unclosed: usize,
}

/// [`Model::rebuild_assignment_regions_dropping_orphan_plates`] の件数報告。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PlateOrphanRebuildReport {
    pub floor: PlateAssignmentRegionRebuildReport,
    pub wall: PlateAssignmentRegionRebuildReport,
    /// 参照先を失って取り除いた囲まれた床板の数。
    pub removed_slabs: usize,
    /// 参照先を失って取り除いた囲まれた壁版の数。
    pub removed_wall_plates: usize,
}

impl FloorPlateAssignmentRegions {
    /// 床板割当領域を安定 ID から引く。ID は配列添字と一致しない。
    pub fn get(&self, id: FloorPlateAssignmentRegionId) -> Option<&FloorPlateAssignmentRegion> {
        self.regions.iter().find(|region| region.id == id)
    }

    /// 床板割当領域を安定 ID から可変で引く。
    pub fn get_mut(
        &mut self,
        id: FloorPlateAssignmentRegionId,
    ) -> Option<&mut FloorPlateAssignmentRegion> {
        self.regions.iter_mut().find(|region| region.id == id)
    }

    /// 未使用の割当領域 ID のうち最小のもの。
    pub(crate) fn next_free_id(&self) -> u32 {
        self.next_id.max(
            self.regions
                .iter()
                .map(|region| region.id.0.saturating_add(1))
                .max()
                .unwrap_or(0),
        )
    }

    /// 既知の境界と版割当から集合を組み立てる（架構作成ウィザードの格子生成用）。
    pub(crate) fn from_assigned(
        entries: Vec<(Vec<SupportBoundary>, PlateAssignment<SlabId>)>,
    ) -> Self {
        let mut regions = FloorPlateAssignmentRegions::default();
        for (boundary, assignment) in entries {
            let id = FloorPlateAssignmentRegionId(regions.next_id);
            regions.next_id += 1;
            regions.regions.push(FloorPlateAssignmentRegion {
                id,
                boundary,
                assignment,
            });
        }
        regions
    }

    /// 大梁と小梁の線分から閉領域を再走査する。同じ支持部材境界は ID と状態を維持する。
    pub fn rebuild(
        &mut self,
        girders: &[SupportMemberSegment],
        joists: &[SupportMemberSegment],
    ) -> PlateAssignmentRegionRebuildReport {
        let (boundaries, unclosed) = scan_bounded_regions(girders.iter().chain(joists));
        self.replace_boundaries(boundaries, unclosed)
    }

    /// 走査済みの境界群で領域を置き換える。同じ境界の領域は ID と割当状態を維持し、
    /// 境界が変わった領域は新しい ID の未設定とする。
    pub(crate) fn replace_boundaries(
        &mut self,
        boundaries: Vec<Vec<SupportBoundary>>,
        unclosed: usize,
    ) -> PlateAssignmentRegionRebuildReport {
        let old = std::mem::take(&mut self.regions);
        self.next_id = self.next_id.max(
            old.iter()
                .map(|region| region.id.0.saturating_add(1))
                .max()
                .unwrap_or(0),
        );
        let mut used = vec![false; old.len()];
        for boundary in boundaries {
            if let Some((index, region)) = old
                .iter()
                .enumerate()
                .find(|(i, region)| !used[*i] && same_boundary(&region.boundary, &boundary))
            {
                used[index] = true;
                self.regions.push(region.clone());
            } else {
                let id = FloorPlateAssignmentRegionId(self.next_id);
                self.next_id += 1;
                self.regions.push(FloorPlateAssignmentRegion {
                    id,
                    boundary,
                    assignment: PlateAssignment::Unset,
                });
            }
        }
        let preserved = used.iter().filter(|v| **v).count();
        PlateAssignmentRegionRebuildReport {
            regions: self.regions.len(),
            preserved,
            created_unset: self.regions.len() - preserved,
            removed: old.len() - preserved,
            unclosed,
        }
    }

    /// 境界が参照する主架構 `ElemId` へ `f` を適用する（要素 ID 繰上げ・繰下げ用）。
    pub fn visit_primary_supports(&mut self, f: &mut impl FnMut(&mut ElemId)) {
        for region in &mut self.regions {
            for edge in &mut region.boundary {
                if let SupportMemberId::Primary(elem) = &mut edge.support {
                    f(elem);
                }
            }
        }
    }

    pub fn validate(&self, slabs: &[Slab]) -> Result<(), CoreError> {
        let enclosed: Vec<bool> = slabs
            .iter()
            .map(|slab| matches!(slab.shape, SlabShape::Enclosed))
            .collect();
        validate_regions(
            &self.regions,
            |region| region.id.0,
            |region| &region.boundary,
            |region| match region.assignment {
                PlateAssignment::Plate(id) => Some((id.index(), id.0)),
                _ => None,
            },
            &enclosed,
            "FloorPlateAssignmentRegion",
            "Slab",
        )
    }
}

impl WallPlateAssignmentRegions {
    /// 壁版割当領域を安定 ID から引く。ID は配列添字と一致しない。
    pub fn get(&self, id: WallPlateAssignmentRegionId) -> Option<&WallPlateAssignmentRegion> {
        self.regions.iter().find(|region| region.id == id)
    }

    /// 壁版割当領域を安定 ID から可変で引く。
    pub fn get_mut(
        &mut self,
        id: WallPlateAssignmentRegionId,
    ) -> Option<&mut WallPlateAssignmentRegion> {
        self.regions.iter_mut().find(|region| region.id == id)
    }

    /// 未使用の割当領域 ID のうち最小のもの。
    pub(crate) fn next_free_id(&self) -> u32 {
        self.next_id.max(
            self.regions
                .iter()
                .map(|region| region.id.0.saturating_add(1))
                .max()
                .unwrap_or(0),
        )
    }

    /// 柱・梁と間柱の線分から閉領域を再走査する。同じ支持部材境界は ID と状態を維持する。
    pub fn rebuild(
        &mut self,
        columns_and_beams: &[SupportMemberSegment],
        posts: &[SupportMemberSegment],
    ) -> PlateAssignmentRegionRebuildReport {
        let (boundaries, unclosed) = scan_bounded_regions(columns_and_beams.iter().chain(posts));
        self.replace_boundaries(boundaries, unclosed)
    }

    /// 走査済みの境界群で領域を置き換える。同じ境界の領域は ID と割当状態を維持し、
    /// 境界が変わった領域は新しい ID の未設定とする。
    pub(crate) fn replace_boundaries(
        &mut self,
        boundaries: Vec<Vec<SupportBoundary>>,
        unclosed: usize,
    ) -> PlateAssignmentRegionRebuildReport {
        let old = std::mem::take(&mut self.regions);
        self.next_id = self.next_id.max(
            old.iter()
                .map(|region| region.id.0.saturating_add(1))
                .max()
                .unwrap_or(0),
        );
        let mut used = vec![false; old.len()];
        for boundary in boundaries {
            if let Some((index, region)) = old
                .iter()
                .enumerate()
                .find(|(i, region)| !used[*i] && same_boundary(&region.boundary, &boundary))
            {
                used[index] = true;
                self.regions.push(region.clone());
            } else {
                let id = WallPlateAssignmentRegionId(self.next_id);
                self.next_id += 1;
                self.regions.push(WallPlateAssignmentRegion {
                    id,
                    boundary,
                    assignment: PlateAssignment::Unset,
                });
            }
        }
        let preserved = used.iter().filter(|v| **v).count();
        PlateAssignmentRegionRebuildReport {
            regions: self.regions.len(),
            preserved,
            created_unset: self.regions.len() - preserved,
            removed: old.len() - preserved,
            unclosed,
        }
    }

    /// 境界が参照する主架構 `ElemId` へ `f` を適用する（要素 ID 繰上げ・繰下げ用）。
    pub fn visit_primary_supports(&mut self, f: &mut impl FnMut(&mut ElemId)) {
        for region in &mut self.regions {
            for edge in &mut region.boundary {
                if let SupportMemberId::Primary(elem) = &mut edge.support {
                    f(elem);
                }
            }
        }
    }

    pub fn validate(&self, plates: &[WallPlate]) -> Result<(), CoreError> {
        let enclosed: Vec<bool> = plates
            .iter()
            .map(|plate| matches!(plate.shape, WallPlateShape::Enclosed))
            .collect();
        validate_regions(
            &self.regions,
            |region| region.id.0,
            |region| &region.boundary,
            |region| match region.assignment {
                PlateAssignment::Plate(id) => Some((id.index(), id.0)),
                _ => None,
            },
            &enclosed,
            "WallPlateAssignmentRegion",
            "WallPlate",
        )
    }
}

fn validate_regions<T>(
    regions: &[T],
    id: impl Fn(&T) -> u32,
    boundary: impl Fn(&T) -> &[SupportBoundary],
    plate: impl Fn(&T) -> Option<(usize, u32)>,
    enclosed: &[bool],
    region_name: &str,
    plate_name: &str,
) -> Result<(), CoreError> {
    let mut ids = HashSet::new();
    let mut plates = HashSet::new();
    let mut boundaries = HashSet::new();
    for region in regions {
        let raw = id(region);
        if !ids.insert(raw) {
            return Err(CoreError::DuplicateId(format!("{region_name}Id({raw})")));
        }
        if boundary(region).len() < 3 || boundary(region).iter().any(|edge| !valid_span(edge.span))
        {
            let bad = boundary(region)
                .iter()
                .find(|edge| !valid_span(edge.span))
                .map(|edge| format!("{:?} {:?}", edge.support, edge.span))
                .unwrap_or_default();
            return Err(CoreError::DanglingRef(format!(
                "{region_name} {raw} の支持部材境界が不正（辺数 {} {bad}）",
                boundary(region).len()
            )));
        }
        if !boundaries.insert(boundary_key(boundary(region))) {
            return Err(CoreError::DuplicateId(format!(
                "{region_name} {raw} は他の割当領域と同じ境界を持つ"
            )));
        }
        if let Some((index, plate_id)) = plate(region) {
            if index >= enclosed.len() {
                return Err(CoreError::DanglingRef(format!(
                    "{region_name} {raw} -> {plate_name} {plate_id}"
                )));
            }
            if !enclosed[index] {
                return Err(CoreError::DanglingRef(format!(
                    "{region_name} {raw} が取り付く{plate_name} {plate_id} を参照している"
                )));
            }
            if !plates.insert(plate_id) {
                return Err(CoreError::DuplicateId(format!(
                    "{plate_name} {plate_id} は複数の割当領域から参照されている"
                )));
            }
        }
    }
    Ok(())
}

fn valid_span(span: [f64; 2]) -> bool {
    span.iter()
        .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
        && span[0] != span[1]
}

/// 割当領域 ID の生値を「R0, R1」形式へまとめる。空なら「なし」。
fn format_region_ids(ids: impl Iterator<Item = u32>) -> String {
    let labels: Vec<String> = ids.map(|id| format!("R{id}")).collect();
    if labels.is_empty() {
        "なし".to_string()
    } else {
        labels.join(", ")
    }
}

/// 構面・レベルをまたいで集めた境界から同一境界を除去する。先に現れた境界を残し、
/// 走査順に依存しない（同じ入力なら同じ結果になる）。同じ境界を 2 つの割当領域が
/// 持つと `validate` が拒否するため、生成段階で防ぐ。
fn dedup_boundaries(boundaries: &mut Vec<Vec<SupportBoundary>>) {
    let mut seen = HashSet::new();
    boundaries.retain(|boundary| seen.insert(boundary_key(boundary)));
}

#[derive(Clone, Copy)]
struct HalfEdge {
    to: usize,
    edge: SupportBoundary,
}

fn scan_bounded_regions<'a>(
    segments: impl Iterator<Item = &'a SupportMemberSegment>,
) -> (Vec<Vec<SupportBoundary>>, usize) {
    let segments: Vec<_> = segments
        .filter(|segment| segment_is_valid(segment))
        .copied()
        .collect();
    let endpoints: Vec<_> = segments
        .iter()
        .flat_map(|segment| [segment.start, segment.end])
        .collect();
    let mut points: Vec<[f64; 2]> = Vec::new();
    let mut around: HashMap<usize, Vec<HalfEdge>> = HashMap::new();
    for segment in &segments {
        let mut cuts = vec![0.0, 1.0];
        cuts.extend(
            endpoints
                .iter()
                .filter_map(|point| point_on_segment_fraction(*point, segment))
                .filter(|t| *t > 1e-9 && *t < 1.0 - 1e-9),
        );
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|a, b| (*a - *b).abs() <= 1e-9);
        for interval in cuts.windows(2) {
            let start = interpolate(segment.start, segment.end, interval[0]);
            let end = interpolate(segment.start, segment.end, interval[1]);
            let a = point_index(&mut points, start);
            let b = point_index(&mut points, end);
            let span = [
                interpolate_scalar(segment.axis_span, interval[0]),
                interpolate_scalar(segment.axis_span, interval[1]),
            ];
            around.entry(a).or_default().push(HalfEdge {
                to: b,
                edge: SupportBoundary {
                    support: segment.support,
                    span,
                },
            });
            around.entry(b).or_default().push(HalfEdge {
                to: a,
                edge: SupportBoundary {
                    support: segment.support,
                    span: [span[1], span[0]],
                },
            });
        }
    }
    for (&from, edges) in &mut around {
        let origin = points[from];
        edges.sort_by(|a, b| angle(origin, points[a.to]).total_cmp(&angle(origin, points[b.to])));
    }
    let mut starts: Vec<(usize, usize)> = around
        .iter()
        .flat_map(|(&from, edges)| edges.iter().map(move |edge| (from, edge.to)))
        .collect();
    starts.sort_unstable();
    let mut visited = HashSet::new();
    let mut regions = Vec::new();
    let mut unclosed = 0;
    for start in starts {
        if visited.contains(&start) {
            continue;
        }
        let mut current = start;
        let mut boundary = Vec::new();
        let mut vertices = Vec::new();
        let mut closed = false;
        for _ in 0..=around.values().map(Vec::len).sum::<usize>() {
            if !visited.insert(current) {
                break;
            }
            vertices.push(points[current.0]);
            let Some(edges) = around.get(&current.0) else {
                break;
            };
            let Some(edge) = edges.iter().find(|edge| edge.to == current.1) else {
                break;
            };
            boundary.push(edge.edge);
            let Some(next_edges) = around.get(&current.1) else {
                break;
            };
            let Some(reverse) = next_edges.iter().position(|edge| edge.to == current.0) else {
                break;
            };
            let next = next_edges[(reverse + next_edges.len() - 1) % next_edges.len()].to;
            current = (current.1, next);
            if current == start {
                closed = true;
                break;
            }
        }
        if !closed {
            unclosed += 1;
        } else if boundary.len() >= 3 && signed_area(&vertices) > 0.0 {
            regions.push(normalize_boundary(boundary));
        }
    }
    regions.sort_by_key(|a| boundary_key(a));
    regions.dedup_by(|a, b| boundary_key(a) == boundary_key(b));
    (regions, unclosed)
}

fn segment_is_valid(segment: &SupportMemberSegment) -> bool {
    segment
        .start
        .iter()
        .chain(segment.end.iter())
        .all(|v| v.is_finite())
        && valid_span(segment.axis_span)
}

fn point_on_segment_fraction(point: [f64; 2], segment: &SupportMemberSegment) -> Option<f64> {
    let delta = [
        segment.end[0] - segment.start[0],
        segment.end[1] - segment.start[1],
    ];
    let length_squared = delta[0] * delta[0] + delta[1] * delta[1];
    if length_squared <= 1e-12 {
        return None;
    }
    let offset = [point[0] - segment.start[0], point[1] - segment.start[1]];
    let t = (offset[0] * delta[0] + offset[1] * delta[1]) / length_squared;
    let projected = interpolate(segment.start, segment.end, t);
    ((projected[0] - point[0]).abs() <= 1e-6
        && (projected[1] - point[1]).abs() <= 1e-6
        && (-1e-9..=1.0 + 1e-9).contains(&t))
    .then_some(t.clamp(0.0, 1.0))
}

fn interpolate(start: [f64; 2], end: [f64; 2], t: f64) -> [f64; 2] {
    [
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
    ]
}

fn interpolate_scalar(span: [f64; 2], t: f64) -> f64 {
    span[0] + (span[1] - span[0]) * t
}

fn point_index(points: &mut Vec<[f64; 2]>, point: [f64; 2]) -> usize {
    if let Some(index) = points
        .iter()
        .position(|p| (p[0] - point[0]).abs() <= 1e-6 && (p[1] - point[1]).abs() <= 1e-6)
    {
        index
    } else {
        points.push(point);
        points.len() - 1
    }
}

fn angle(origin: [f64; 2], point: [f64; 2]) -> f64 {
    (point[1] - origin[1]).atan2(point[0] - origin[0])
}

fn signed_area(points: &[[f64; 2]]) -> f64 {
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .map(|(a, b)| a[0] * b[1] - b[0] * a[1])
        .sum::<f64>()
        * 0.5
}

fn same_boundary(a: &[SupportBoundary], b: &[SupportBoundary]) -> bool {
    boundary_key(a) == boundary_key(b)
}

fn normalize_boundary(boundary: Vec<SupportBoundary>) -> Vec<SupportBoundary> {
    let mut normalized: Vec<SupportBoundary> = Vec::with_capacity(boundary.len());
    for edge in boundary {
        if let Some(previous) = normalized.last_mut() {
            if previous.support == edge.support && previous.span[1] == edge.span[0] {
                let merged = [previous.span[0], edge.span[1]];
                if merged[0] != merged[1] {
                    previous.span[1] = merged[1];
                    continue;
                }
            }
        }
        normalized.push(edge);
    }
    if normalized.len() >= 2 {
        let first = normalized[0];
        let last = *normalized.last().expect("2 辺以上");
        if first.support == last.support
            && last.span[1] == first.span[0]
            && last.span[0] != first.span[1]
        {
            normalized[0].span[0] = last.span[0];
            normalized.pop();
        }
    }
    normalized
}

fn boundary_key(boundary: &[SupportBoundary]) -> Vec<(SupportMemberId, u64, u64)> {
    let direct: Vec<_> = boundary
        .iter()
        .map(|edge| {
            (
                edge.support,
                normalized_bits(edge.span[0]),
                normalized_bits(edge.span[1]),
            )
        })
        .collect();
    let reverse: Vec<_> = boundary
        .iter()
        .rev()
        .map(|edge| {
            (
                edge.support,
                normalized_bits(edge.span[1]),
                normalized_bits(edge.span[0]),
            )
        })
        .collect();
    rotations(&direct)
        .chain(rotations(&reverse))
        .min()
        .unwrap_or_default()
}

fn rotations(
    items: &[(SupportMemberId, u64, u64)],
) -> impl Iterator<Item = Vec<(SupportMemberId, u64, u64)>> + '_ {
    (0..items.len()).map(|offset| {
        items
            .iter()
            .cycle()
            .skip(offset)
            .take(items.len())
            .copied()
            .collect()
    })
}

fn normalized_bits(value: f64) -> u64 {
    if value == 0.0 {
        0.0f64.to_bits()
    } else {
        value.to_bits()
    }
}

impl Model {
    /// 水平な大梁と、同じレベルにある二次部材小梁の線分から床板割当領域を再構築する。
    ///
    /// レベルごとに面走査する（異なる階の重なりを混ぜない）。同じ支持部材境界の
    /// 領域は ID と割当状態を維持し、境界が変わった領域は新しい ID の未設定とする。
    pub fn rebuild_floor_assignment_regions(&mut self) -> PlateAssignmentRegionRebuildReport {
        let mut boundaries: Vec<Vec<SupportBoundary>> = Vec::new();
        let mut unclosed = 0usize;
        for level in self.horizontal_beam_levels() {
            let mut girders = Vec::new();
            let mut joists = Vec::new();
            for e in &self.elements {
                if e.kind != super::ElementKind::Beam || e.nodes.len() != 2 {
                    continue;
                }
                let (Some(a), Some(b)) = (self.node(e.nodes[0]), self.node(e.nodes[1])) else {
                    continue;
                };
                if (a.coord[2] - b.coord[2]).abs() > crate::geom::LEVEL_TOL_MM
                    || ((a.coord[2] + b.coord[2]) / 2.0 - level).abs() > crate::geom::LEVEL_TOL_MM
                {
                    continue;
                }
                girders.push(SupportMemberSegment {
                    support: SupportMemberId::Primary(e.id),
                    axis_span: [0.0, 1.0],
                    start: [a.coord[0], a.coord[1]],
                    end: [b.coord[0], b.coord[1]],
                });
            }
            for sm in self.joists() {
                let id = sm.id;
                let Some((a, b)) = self.support_member_axis(SupportMemberId::Secondary(id)) else {
                    continue;
                };
                if (a[2] - b[2]).abs() > crate::geom::LEVEL_TOL_MM
                    || ((a[2] + b[2]) / 2.0 - level).abs() > crate::geom::LEVEL_TOL_MM
                {
                    continue;
                }
                joists.push(SupportMemberSegment {
                    support: SupportMemberId::Secondary(id),
                    axis_span: [0.0, 1.0],
                    start: [a[0], a[1]],
                    end: [b[0], b[1]],
                });
            }
            let (mut level_boundaries, level_unclosed) =
                scan_bounded_regions(girders.iter().chain(joists.iter()));
            boundaries.append(&mut level_boundaries);
            unclosed += level_unclosed;
        }
        dedup_boundaries(&mut boundaries);
        self.floor_assignment_regions
            .replace_boundaries(boundaries, unclosed)
    }

    /// 水平な 2 節点梁が存在するレベル Z を、許容差でまとめて昇順に返す。
    fn horizontal_beam_levels(&self) -> Vec<f64> {
        let mut levels: Vec<f64> = Vec::new();
        for e in &self.elements {
            if e.kind != super::ElementKind::Beam || e.nodes.len() != 2 {
                continue;
            }
            let (Some(a), Some(b)) = (self.node(e.nodes[0]), self.node(e.nodes[1])) else {
                continue;
            };
            if (a.coord[2] - b.coord[2]).abs() > crate::geom::LEVEL_TOL_MM {
                continue;
            }
            let z = (a.coord[2] + b.coord[2]) / 2.0;
            if !levels
                .iter()
                .any(|l| (l - z).abs() <= crate::geom::LEVEL_TOL_MM)
            {
                levels.push(z);
            }
        }
        levels.sort_by(f64::total_cmp);
        levels
    }

    /// 柱・梁と支持された間柱の線分から壁版割当領域を再構築する。
    ///
    /// 構面（柱脚位置から検出する鉛直平面）ごとに局所座標 `(s, z)` で面走査する。
    /// 同じ支持部材境界の領域は ID と割当状態を維持し、境界が変わった領域は新しい ID の
    /// 未設定とする。片持ち間柱の自由端は閉領域の辺にならないため含めない。
    pub fn rebuild_wall_assignment_regions(&mut self) -> PlateAssignmentRegionRebuildReport {
        let mut boundaries: Vec<Vec<SupportBoundary>> = Vec::new();
        let mut unclosed = 0usize;
        for (origin, direction) in crate::region_gen::wall::wall_planes(self) {
            let project = |coord: [f64; 3]| {
                let v = [coord[0] - origin[0], coord[1] - origin[1]];
                [v[0] * direction[0] + v[1] * direction[1], coord[2]]
            };
            let on_plane = |coord: [f64; 3]| {
                let v = [coord[0] - origin[0], coord[1] - origin[1]];
                (v[0] * direction[1] - v[1] * direction[0]).abs() <= crate::geom::MEMBER_AXIS_TOL_MM
            };
            let mut segments = Vec::new();
            for e in &self.elements {
                if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
                    continue;
                }
                let (Some(a), Some(b)) = (
                    self.node(e.nodes[0]).map(|n| n.coord),
                    self.node(e.nodes[1]).map(|n| n.coord),
                ) else {
                    continue;
                };
                if !on_plane(a) || !on_plane(b) {
                    continue;
                }
                segments.push(SupportMemberSegment {
                    support: SupportMemberId::Primary(e.id),
                    axis_span: [0.0, 1.0],
                    start: project(a),
                    end: project(b),
                });
            }
            for sm in self.posts() {
                let id = sm.id;
                let Some((a, b)) = self.support_member_axis(SupportMemberId::Secondary(id)) else {
                    continue;
                };
                if !on_plane(a) || !on_plane(b) {
                    continue;
                }
                segments.push(SupportMemberSegment {
                    support: SupportMemberId::Secondary(id),
                    axis_span: [0.0, 1.0],
                    start: project(a),
                    end: project(b),
                });
            }
            let (mut plane_boundaries, plane_unclosed) = scan_bounded_regions(segments.iter());
            boundaries.append(&mut plane_boundaries);
            unclosed += plane_unclosed;
        }
        dedup_boundaries(&mut boundaries);
        self.wall_assignment_regions
            .replace_boundaries(boundaries, unclosed)
    }

    /// 床板・壁版の割当領域を再構築し、境界が変わって参照先を失った囲まれた版を
    /// 取り除く。
    ///
    /// 版の割当は領域の境界から導出されるため、小梁・間柱の配置で境界が変わった
    /// 領域は新しい ID の未設定となり、そこだけに載っていた版は行き先を失う。
    /// 版なしのまま残すと [`Model::validate`] の「囲まれた版はちょうど 1 つの割当領域に
    /// 属すること」を満たせないため、失った版は取り除いて領域を未設定に戻す。
    /// 取り除いた版は呼び出し側が undo で復元する。
    pub fn rebuild_assignment_regions_dropping_orphan_plates(
        &mut self,
    ) -> PlateOrphanRebuildReport {
        let floor = self.rebuild_floor_assignment_regions();
        let wall = self.rebuild_wall_assignment_regions();

        let floor_assigned: HashSet<SlabId> = self
            .floor_assignment_regions
            .regions
            .iter()
            .filter_map(|region| region.assignment.plate())
            .collect();
        let before_slabs = self.slabs.len();
        self.retain_slabs(|slab| {
            !matches!(slab.shape, SlabShape::Enclosed) || floor_assigned.contains(&slab.id)
        });
        let removed_slabs = before_slabs - self.slabs.len();

        let wall_assigned: HashSet<WallPlateId> = self
            .wall_assignment_regions
            .regions
            .iter()
            .filter_map(|region| region.assignment.plate())
            .collect();
        let before_plates = self.wall_plates.len();
        self.retain_wall_plates(|plate| {
            !matches!(plate.shape, WallPlateShape::Enclosed) || wall_assigned.contains(&plate.id)
        });
        let removed_wall_plates = before_plates - self.wall_plates.len();

        PlateOrphanRebuildReport {
            floor,
            wall,
            removed_slabs,
            removed_wall_plates,
        }
    }

    /// 床板を割り当てている床板割当領域を返す。未割当・版なしの床板は `None`。
    pub fn slab_assignment_region(&self, slab: SlabId) -> Option<&FloorPlateAssignmentRegion> {
        self.floor_assignment_regions
            .regions
            .iter()
            .find(|region| region.assignment == PlateAssignment::Plate(slab))
    }

    /// 壁版を割り当てている壁版割当領域を返す。未割当・版なしの壁版は `None`。
    pub fn wall_plate_assignment_region(
        &self,
        plate: WallPlateId,
    ) -> Option<&WallPlateAssignmentRegion> {
        self.wall_assignment_regions
            .regions
            .iter()
            .find(|region| region.assignment == PlateAssignment::Plate(plate))
    }

    /// 未設定（`Unset`）の割当領域を床板・壁版別に返す。版なしは利用者が明示的に
    /// 版を置かないと決めた状態のため含めない。
    pub fn unset_plate_assignment_regions(
        &self,
    ) -> (
        Vec<FloorPlateAssignmentRegionId>,
        Vec<WallPlateAssignmentRegionId>,
    ) {
        let floors = self
            .floor_assignment_regions
            .regions
            .iter()
            .filter(|region| region.assignment.is_unset())
            .map(|region| region.id)
            .collect();
        let walls = self
            .wall_assignment_regions
            .regions
            .iter()
            .filter(|region| region.assignment.is_unset())
            .map(|region| region.id)
            .collect();
        (floors, walls)
    }

    /// 未設定の割当領域があれば、件数と対象 ID を床板・壁版別に列挙した注意文を返す。
    /// 未設定が無ければ `None`。計算は許可しつつ、荷重・剛性の過小評価に気づかせる。
    pub fn unset_plate_assignment_warning(&self) -> Option<String> {
        let (floors, walls) = self.unset_plate_assignment_regions();
        if floors.is_empty() && walls.is_empty() {
            return None;
        }
        let n_floors = floors.len();
        let n_walls = walls.len();
        let floors = format_region_ids(floors.iter().map(|id| id.0));
        let walls = format_region_ids(walls.iter().map(|id| id.0));
        Some(format!(
            "未設定の割当領域があります（床板 {n_floors} 件: {floors} / 壁版 {n_walls} 件: {walls}）。\
             未設定の領域は荷重・剛性を過小評価し得ます。"
        ))
    }

    /// 割当領域境界の頂点座標 [mm] を辺順に返す。末尾は先頭へ戻るため重複させない。
    /// 境界が空、または支持部材の材軸を解決できない辺があれば `None`。
    pub fn assignment_region_boundary_coords(
        &self,
        boundary: &[SupportBoundary],
    ) -> Option<Vec<[f64; 3]>> {
        if boundary.is_empty() {
            return None;
        }
        let mut coords = Vec::with_capacity(boundary.len());
        for edge in boundary {
            let (a, b) = self.support_member_axis(edge.support)?;
            let at = |t: f64| {
                [
                    a[0] + (b[0] - a[0]) * t,
                    a[1] + (b[1] - a[1]) * t,
                    a[2] + (b[2] - a[2]) * t,
                ]
            };
            if coords.is_empty() {
                coords.push(at(edge.span[0]));
            }
            coords.push(at(edge.span[1]));
        }
        let first = coords[0];
        let last = *coords.last()?;
        let tol = crate::geom::MEMBER_AXIS_TOL_MM;
        if coords.len() >= 2
            && (first[0] - last[0]).abs() <= tol
            && (first[1] - last[1]).abs() <= tol
            && (first[2] - last[2]).abs() <= tol
        {
            coords.pop();
        }
        Some(coords)
    }

    /// 床板割当領域 ID から境界の頂点座標 [mm] を解決する。領域が無ければ `None`。
    pub fn floor_assignment_region_coords(
        &self,
        id: FloorPlateAssignmentRegionId,
    ) -> Option<Vec<[f64; 3]>> {
        let region = self.floor_assignment_region(id)?;
        self.assignment_region_boundary_coords(&region.boundary)
    }

    /// 床板割当領域 ID から境界頂点に対応する節点列を返す。頂点に一致する節点が
    /// 無い場合（二次部材アンカー由来の頂点など）は `None`。
    pub fn floor_assignment_region_nodes(
        &self,
        id: FloorPlateAssignmentRegionId,
    ) -> Option<Vec<NodeId>> {
        let coords = self.floor_assignment_region_coords(id)?;
        let mut nodes = Vec::with_capacity(coords.len());
        for coord in coords {
            let node = self.nodes.iter().find(|n| {
                crate::geom::vec3::dist(n.coord, coord) <= crate::geom::MEMBER_AXIS_TOL_MM
            })?;
            nodes.push(node.id);
        }
        Some(nodes)
    }

    /// 壁版割当領域 ID から境界の頂点座標 [mm] を解決する。領域が無ければ `None`。
    pub fn wall_assignment_region_coords(
        &self,
        id: WallPlateAssignmentRegionId,
    ) -> Option<Vec<[f64; 3]>> {
        let region = self.wall_assignment_region(id)?;
        self.assignment_region_boundary_coords(&region.boundary)
    }

    /// 壁版割当領域 ID から境界頂点に対応する節点列を返す。頂点に一致する節点が
    /// 無い場合（二次部材アンカー由来の頂点など）は `None`。
    pub fn wall_assignment_region_nodes(
        &self,
        id: WallPlateAssignmentRegionId,
    ) -> Option<Vec<NodeId>> {
        let coords = self.wall_assignment_region_coords(id)?;
        let mut nodes = Vec::with_capacity(coords.len());
        for coord in coords {
            let node = self.nodes.iter().find(|n| {
                crate::geom::vec3::dist(n.coord, coord) <= crate::geom::MEMBER_AXIS_TOL_MM
            })?;
            nodes.push(node.id);
        }
        Some(nodes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::NodeId;
    use crate::model::{validate_secondary_members, SecondaryMember, SecondaryMemberKind};

    fn segment(id: u32, start: [f64; 2], end: [f64; 2]) -> SupportMemberSegment {
        SupportMemberSegment {
            support: SupportMemberId::Primary(ElemId(id)),
            axis_span: [0.0, 1.0],
            start,
            end,
        }
    }

    fn square() -> Vec<SupportMemberSegment> {
        vec![
            segment(0, [0.0, 0.0], [4.0, 0.0]),
            segment(1, [4.0, 0.0], [4.0, 4.0]),
            segment(2, [4.0, 4.0], [0.0, 4.0]),
            segment(3, [0.0, 4.0], [0.0, 0.0]),
        ]
    }

    #[test]
    fn 床板割当領域は同じ境界のidと状態を維持する() {
        let mut regions = FloorPlateAssignmentRegions::default();
        let first = regions.rebuild(&square(), &[]);
        assert_eq!(first.created_unset, 1);
        regions.regions[0].assignment = PlateAssignment::NoPlate;
        let id = regions.regions[0].id;

        let second = regions.rebuild(&square(), &[]);
        assert_eq!(second.preserved, 1);
        assert_eq!(regions.regions[0].id, id);
        assert_eq!(regions.regions[0].assignment, PlateAssignment::NoPlate);
    }

    #[test]
    fn 小梁で分割された領域はunsetで作り直す() {
        let mut regions = FloorPlateAssignmentRegions::default();
        regions.rebuild(&square(), &[]);
        regions.regions[0].assignment = PlateAssignment::Plate(SlabId(0));
        let old_id = regions.regions[0].id;
        let joist = SupportMemberSegment {
            support: SupportMemberId::Secondary(SecondaryMemberId(10)),
            axis_span: [0.0, 1.0],
            start: [2.0, 0.0],
            end: [2.0, 4.0],
        };

        let report = regions.rebuild(&square(), &[joist]);
        assert_eq!(report.regions, 2);
        assert_eq!(report.removed, 1);
        assert!(regions
            .regions
            .iter()
            .all(|region| { region.id != old_id && region.assignment == PlateAssignment::Unset }));
    }

    #[test]
    fn 壁版割当領域も間柱で分割する() {
        let mut regions = WallPlateAssignmentRegions::default();
        let post = SupportMemberSegment {
            support: SupportMemberId::Secondary(SecondaryMemberId(2)),
            axis_span: [0.0, 1.0],
            start: [2.0, 0.0],
            end: [2.0, 4.0],
        };
        let report = regions.rebuild(&square(), &[post]);
        assert_eq!(report.regions, 2);
        assert_eq!(report.created_unset, 2);
    }

    #[test]
    fn 二次部材の取付きと片持ち自由端を検証する() {
        let supported = member(
            0,
            SecondaryMemberEnds::Supported([
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(0)),
                    position: 0.25,
                },
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(1)),
                    position: 0.75,
                },
            ]),
        );
        let cantilever = member(
            1,
            SecondaryMemberEnds::Cantilever {
                support: SecondaryMemberAnchor {
                    support: SupportMemberId::Secondary(SecondaryMemberId(0)),
                    position: 0.5,
                },
                free_end_vector: [0.0, 1500.0],
            },
        );
        assert_eq!(
            validate_secondary_members(&[&supported, &cantilever]),
            Ok(())
        );

        let invalid = member(
            1,
            SecondaryMemberEnds::Cantilever {
                support: SecondaryMemberAnchor {
                    support: SupportMemberId::Secondary(SecondaryMemberId(99)),
                    position: 0.5,
                },
                free_end_vector: [0.0, 1500.0],
            },
        );
        assert!(matches!(
            validate_secondary_members(&[&supported, &invalid]),
            Err(CoreError::DanglingRef(_))
        ));
    }

    #[test]
    fn 割当領域は存在しない版と版の重複割当を拒否する() {
        let mut regions = FloorPlateAssignmentRegions::default();
        let joist = SupportMemberSegment {
            support: SupportMemberId::Secondary(SecondaryMemberId(0)),
            axis_span: [0.0, 1.0],
            start: [2.0, 0.0],
            end: [2.0, 4.0],
        };
        regions.rebuild(&square(), &[joist]);
        regions.regions[0].assignment = PlateAssignment::Plate(SlabId(0));
        assert!(matches!(
            regions.validate(&[]),
            Err(CoreError::DanglingRef(_))
        ));

        regions.regions[1].assignment = PlateAssignment::Plate(SlabId(0));
        assert!(matches!(
            regions.validate(&[crate::model::Slab {
                id: SlabId(0),
                shape: crate::model::SlabShape::Enclosed,
                plate: crate::model::SlabPlate {
                    section: None,
                    loads: vec![],
                    usage: None,
                    method: crate::model::DistributionMethod::TriTrapezoid,
                    one_way: None,
                },
            }]),
            Err(CoreError::DuplicateId(_))
        ));
    }

    #[test]
    fn 二次部材の支持グラフの循環を拒否する() {
        let cantilever = |id: u32, support: u32| {
            member(
                id,
                SecondaryMemberEnds::Cantilever {
                    support: SecondaryMemberAnchor {
                        support: SupportMemberId::Secondary(SecondaryMemberId(support)),
                        position: 0.5,
                    },
                    free_end_vector: [0.0, 1000.0],
                },
            )
        };
        assert!(matches!(
            validate_secondary_members(&[&cantilever(0, 1), &cantilever(1, 0)]),
            Err(CoreError::DanglingRef(_))
        ));
    }

    fn member(id: u32, ends: SecondaryMemberEnds) -> SecondaryMember {
        SecondaryMember {
            id: SecondaryMemberId(id),
            kind: SecondaryMemberKind::Joist,
            ends,
            ..Default::default()
        }
    }

    fn square_model_with_joist() -> Model {
        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(crate::model::Node {
                id: NodeId(i as u32),
                coord: [x, y, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        for (i, (a, b)) in [(0, 1), (1, 2), (2, 3), (3, 0)].into_iter().enumerate() {
            model.elements.push(crate::model::ElementData {
                id: ElemId(i as u32),
                kind: crate::model::ElementKind::Beam,
                nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
                section: None,
                local_axis: crate::model::LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [
                    crate::model::EndCondition::Fixed,
                    crate::model::EndCondition::Fixed,
                ],
                force_regime: crate::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        model.unassigned_joists.push(member(
            0,
            SecondaryMemberEnds::Supported([
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(0)),
                    position: 0.5,
                },
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(2)),
                    position: 0.5,
                },
            ]),
        ));
        model
    }

    #[test]
    fn モデルは大梁と小梁から床板割当領域を作り直す() {
        let mut model = square_model_with_joist();
        let first = model.rebuild_floor_assignment_regions();
        assert_eq!(first.created_unset, 2, "中央小梁で 2 面");
        let ids: Vec<_> = model
            .floor_assignment_regions
            .regions
            .iter()
            .map(|r| r.id)
            .collect();
        model.floor_assignment_regions.regions[0].assignment = PlateAssignment::NoPlate;

        let second = model.rebuild_floor_assignment_regions();
        assert_eq!(second.preserved, 2, "境界不変なら ID と状態を維持");
        assert_eq!(
            model
                .floor_assignment_regions
                .regions
                .iter()
                .map(|r| r.id)
                .collect::<Vec<_>>(),
            ids
        );
        assert!(model
            .floor_assignment_region(ids[0])
            .unwrap()
            .assignment
            .is_no_plate());
    }

    #[test]
    fn 境界節点から囲まれた床板を割当領域へ追加する() {
        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(crate::model::Node {
                id: NodeId(i as u32),
                coord: [x, y, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        for (i, (a, b)) in [(0u32, 1u32), (1, 2), (2, 3), (3, 0)]
            .into_iter()
            .enumerate()
        {
            model.elements.push(crate::model::ElementData {
                id: ElemId(i as u32),
                kind: crate::model::ElementKind::Beam,
                nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
                section: None,
                local_axis: crate::model::LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [
                    crate::model::EndCondition::Fixed,
                    crate::model::EndCondition::Fixed,
                ],
                force_regime: crate::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        let slab_id = model.add_enclosed_slab_from_nodes(
            &[NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            crate::model::SlabPlate::default(),
        );
        assert_eq!(model.slabs.len(), 1);
        let region = model.slab_assignment_region(slab_id).expect("割当済み");
        assert_eq!(region.assignment, PlateAssignment::Plate(slab_id));
        assert_eq!(
            model.slabs[0]
                .boundary_nodes(&model)
                .map(|nodes| nodes.len()),
            Some(4)
        );
    }

    #[test]
    fn 支持部材境界を座標へ解決する() {
        let model = square_model_with_joist();
        let axis = model
            .support_member_axis(SupportMemberId::Secondary(SecondaryMemberId(0)))
            .expect("小梁の材軸");
        assert!((axis.0[0] - 2.0).abs() < 1e-9 && axis.0[1].abs() < 1e-9);
        assert!((axis.1[0] - 2.0).abs() < 1e-9 && (axis.1[1] - 4.0).abs() < 1e-9);

        let segments = model
            .support_boundary_segments(&[SupportBoundary {
                support: SupportMemberId::Primary(ElemId(0)),
                span: [0.25, 0.75],
            }])
            .expect("境界座標");
        assert!((segments[0].0[0] - 1.0).abs() < 1e-9);
        assert!((segments[0].1[0] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn 床板から割当領域を逆引きし境界座標を解決する() {
        let mut model = square_model_with_joist();
        model.rebuild_floor_assignment_regions();
        let region_id = model.floor_assignment_regions.regions[0].id;
        model.floor_assignment_regions.regions[0].assignment = PlateAssignment::Plate(SlabId(0));

        let region = model
            .slab_assignment_region(SlabId(0))
            .expect("逆引きできる");
        assert_eq!(region.id, region_id);
        assert!(model.slab_assignment_region(SlabId(1)).is_none());

        let coords = model
            .floor_assignment_region_coords(region_id)
            .expect("境界座標を解決できる");
        assert!(coords.len() >= 3, "閉領域の頂点数: {coords:?}");
        let first = coords[0];
        let last = *coords.last().unwrap();
        assert!(
            (first[0] - last[0]).abs() > 1e-9 || (first[1] - last[1]).abs() > 1e-9,
            "末尾は先頭へ戻る重複を持たない"
        );
    }

    #[test]
    fn 未設定の割当領域だけを警告する() {
        let mut model = square_model_with_joist();
        model.rebuild_floor_assignment_regions();
        model.rebuild_wall_assignment_regions();
        // 未設定が 2 面。版なしは警告に含めない。
        let (floors, walls) = model.unset_plate_assignment_regions();
        assert_eq!(floors.len(), 2);
        assert!(walls.is_empty());
        let warning = model.unset_plate_assignment_warning().expect("警告文");
        assert!(warning.contains("床板 2 件"), "{warning}");
        assert!(warning.contains("R0"), "{warning}");

        model.floor_assignment_regions.regions[0].assignment = PlateAssignment::NoPlate;
        let (floors, _) = model.unset_plate_assignment_regions();
        assert_eq!(floors.len(), 1, "版なしは未設定に数えない");

        for region in &mut model.floor_assignment_regions.regions {
            region.assignment = PlateAssignment::NoPlate;
        }
        assert!(model.unset_plate_assignment_warning().is_none());
    }

    #[test]
    fn 割当領域は取り付く版を参照できない() {
        let attached = crate::model::Slab {
            id: SlabId(0),
            shape: crate::model::SlabShape::Attached {
                anchor: crate::model::RegionAnchor::Point(NodeId(0)),
                extent: [1000.0, 1000.0],
            },
            plate: crate::model::SlabPlate::default(),
        };
        let mut regions = FloorPlateAssignmentRegions::default();
        regions.rebuild(&square(), &[]);
        regions.regions[0].assignment = PlateAssignment::Plate(SlabId(0));
        assert!(matches!(
            regions.validate(&[attached]),
            Err(CoreError::DanglingRef(_))
        ));
    }

    #[test]
    fn 二次部材idは既存idと衝突しないよう払い出す() {
        let mut model = square_model_with_joist();
        assert_eq!(model.alloc_secondary_member_id(), SecondaryMemberId(1));
        assert_eq!(model.alloc_secondary_member_id(), SecondaryMemberId(2));
        model.next_secondary_member_id = 10;
        assert_eq!(model.alloc_secondary_member_id(), SecondaryMemberId(10));
        assert_eq!(model.next_secondary_member_id, 11);
    }

    #[test]
    fn 取付き位置表現の二次部材を検証する() {
        let mut model = square_model_with_joist();
        let sm = SecondaryMember {
            id: SecondaryMemberId(100),
            kind: SecondaryMemberKind::Joist,
            ends: SecondaryMemberEnds::Supported([
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(0)),
                    position: 0.25,
                },
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(2)),
                    position: 0.75,
                },
            ]),
            ..Default::default()
        };
        model.unassigned_joists.push(sm);
        assert_eq!(model.validate(), Ok(()));

        model.unassigned_joists[0].ends = SecondaryMemberEnds::Supported([
            SecondaryMemberAnchor {
                support: SupportMemberId::Primary(ElemId(99)),
                position: 0.25,
            },
            SecondaryMemberAnchor {
                support: SupportMemberId::Primary(ElemId(2)),
                position: 0.75,
            },
        ]);
        assert!(matches!(model.validate(), Err(CoreError::DanglingRef(_))));
    }

    #[test]
    fn 取付き位置表現から材軸と材軸長を解決する() {
        let mut model = square_model_with_joist();
        let sm = SecondaryMember {
            id: SecondaryMemberId(100),
            kind: SecondaryMemberKind::Joist,
            ends: SecondaryMemberEnds::Supported([
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(0)),
                    position: 0.5,
                },
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(2)),
                    position: 0.5,
                },
            ]),
            ..Default::default()
        };
        let (a, b, len) = model.secondary_member_axis(&sm).expect("材軸を解決できる");
        assert!((a[0] - 2.0).abs() < 1e-9 && a[1].abs() < 1e-9);
        assert!((b[0] - 2.0).abs() < 1e-9 && (b[1] - 4.0).abs() < 1e-9);
        assert!((len - 4.0).abs() < 1e-9);

        model.unassigned_joists.push(sm);
        assert!(model
            .secondary_member_axis(&model.unassigned_joists[0])
            .is_some());
        assert_eq!(model.secondary_joist_axes().len(), 2);
    }

    /// 柱2本・梁2本で閉じた 4m×3m の壁構面（Y=0）を持つ最小モデル。
    fn wall_model() -> Model {
        let mut model = Model::default();
        for (i, (x, z)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 3000.0), (0.0, 3000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(crate::model::Node {
                id: NodeId(i as u32),
                coord: [x, 0.0, z],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        for (i, (a, b)) in [(0, 3), (1, 2), (3, 2), (0, 1)].into_iter().enumerate() {
            model.elements.push(crate::model::ElementData {
                id: ElemId(i as u32),
                kind: crate::model::ElementKind::Beam,
                nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
                section: None,
                local_axis: crate::model::LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [
                    crate::model::EndCondition::Fixed,
                    crate::model::EndCondition::Fixed,
                ],
                force_regime: crate::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        model
    }

    #[test]
    fn モデルは柱梁から壁版割当領域を作り直す() {
        let mut model = wall_model();
        let first = model.rebuild_wall_assignment_regions();
        assert_eq!(first.created_unset, 1, "柱梁で囲まれた 1 面");
        let id = model.wall_assignment_regions.regions[0].id;
        model.wall_assignment_regions.regions[0].assignment = PlateAssignment::NoPlate;

        let second = model.rebuild_wall_assignment_regions();
        assert_eq!(second.preserved, 1, "境界不変なら ID と状態を維持");
        assert_eq!(model.wall_assignment_regions.regions[0].id, id);
        assert!(model
            .wall_assignment_region(id)
            .unwrap()
            .assignment
            .is_no_plate());
    }

    #[test]
    fn 壁版割当領域は間柱で分割され境界が変われば未設定へ戻る() {
        let mut model = wall_model();
        model.rebuild_wall_assignment_regions();
        assert_eq!(model.wall_assignment_regions.regions.len(), 1);
        model.wall_assignment_regions.regions[0].assignment = PlateAssignment::NoPlate;
        let old_id = model.wall_assignment_regions.regions[0].id;

        model.unassigned_posts.push(SecondaryMember {
            id: SecondaryMemberId(0),
            kind: SecondaryMemberKind::Post,
            ends: SecondaryMemberEnds::Supported([
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(3)),
                    position: 0.5,
                },
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(2)),
                    position: 0.5,
                },
            ]),
            ..Default::default()
        });
        let report = model.rebuild_wall_assignment_regions();
        assert_eq!(report.regions, 2, "間柱で 2 面");
        assert_eq!(report.removed, 1, "境界が変わった旧領域は解除");
        assert!(model
            .wall_assignment_regions
            .regions
            .iter()
            .all(|r| r.id != old_id && r.assignment == PlateAssignment::Unset));
    }

    /// 指定した平面位置・レベルに 4 辺が閉じた水平大梁の矩形を追加する。
    fn add_rectangular_grid(
        model: &mut Model,
        level: f64,
        x0: f64,
        x1: f64,
        y0: f64,
        y1: f64,
        elem_base: u32,
    ) {
        let corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
        let node_base = model.nodes.len() as u32;
        for (i, c) in corners.iter().enumerate() {
            model.nodes.push(crate::model::Node {
                id: NodeId(node_base + i as u32),
                coord: [c[0], c[1], level],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        for (i, (a, b)) in [(0u32, 1u32), (1, 2), (2, 3), (3, 0)]
            .into_iter()
            .enumerate()
        {
            model.elements.push(crate::model::ElementData {
                id: ElemId(elem_base + i as u32),
                kind: crate::model::ElementKind::Beam,
                nodes: [NodeId(node_base + a), NodeId(node_base + b)]
                    .into_iter()
                    .collect(),
                section: None,
                local_axis: crate::model::LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [
                    crate::model::EndCondition::Fixed,
                    crate::model::EndCondition::Fixed,
                ],
                force_regime: crate::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
    }

    /// 段差床（同一階で Z の異なる床）は、レベルごとに面走査するため混ざらず、
    /// それぞれ独立した割当領域になる。
    #[test]
    fn 段差床はレベルごとに独立した割当領域になる() {
        let mut model = Model::default();
        add_rectangular_grid(&mut model, 3000.0, 0.0, 4000.0, 0.0, 4000.0, 0);
        add_rectangular_grid(&mut model, 3300.0, 4000.0, 8000.0, 0.0, 4000.0, 4);

        let report = model.rebuild_floor_assignment_regions();
        assert_eq!(report.regions, 2, "段差の各レベルで 1 面ずつ");
        assert_eq!(report.unclosed, 0);
        assert!(model
            .floor_assignment_regions
            .regions
            .iter()
            .all(|r| r.boundary.len() == 4));
    }

    /// 別階で XY が完全に重なる 2 面は、投影だけでは混ざらず別 ID の 2 領域になる。
    #[test]
    fn 別階で重なる床は別の割当領域になる() {
        let mut model = Model::default();
        add_rectangular_grid(&mut model, 3000.0, 0.0, 4000.0, 0.0, 4000.0, 0);
        add_rectangular_grid(&mut model, 6000.0, 0.0, 4000.0, 0.0, 4000.0, 4);

        let report = model.rebuild_floor_assignment_regions();
        assert_eq!(report.regions, 2, "別階の重なりは統合しない");
        let ids: Vec<u32> = model
            .floor_assignment_regions
            .regions
            .iter()
            .map(|r| r.id.0)
            .collect();
        assert_eq!(ids, vec![0, 1], "別 ID を持つ");
        assert!(model
            .floor_assignment_regions
            .regions
            .iter()
            .all(|r| r.boundary.len() == 4));
    }

    /// 境界が変わって領域が消えても、次に作られる領域は古い ID を再利用しない。
    #[test]
    fn 除去した領域のidを再利用しない() {
        let mut regions = FloorPlateAssignmentRegions::default();
        let first = regions.rebuild(&square(), &[]);
        assert_eq!(first.created_unset, 1);
        let old_id = regions.regions[0].id;

        let removed = regions.rebuild(&[], &[]);
        assert_eq!(removed.regions, 0);
        assert_eq!(removed.removed, 1);

        let recreated = regions.rebuild(&square(), &[]);
        assert_eq!(recreated.created_unset, 1);
        assert_ne!(
            regions.regions[0].id, old_id,
            "除去後に旧 ID を再利用しない"
        );
    }
}
