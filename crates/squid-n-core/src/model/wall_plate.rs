//! 壁版（`WallPlate`）。
//!
//! 壁領域（[`WallRegion`]、[`super::wall`]）は柱・梁が囲む鉛直構面内の閉領域そのもので、
//! 版の仕様は持たない。版の仕様（断面・開口）は本モジュールの `WallPlate` が持つ。
//! 1 つの壁領域は、壁領域内が間柱でさらに細かい壁パネルに分かれていれば複数の
//! `WallPlate` を持ちうる（[`WallRegion::wall_plate_ids`]。E5。床側の `FloorRegion`/
//! [`super::Slab`] と同じ関係）。パラペット・腰壁・垂れ壁・自立壁はどの壁領域からも
//! 参照されない、独立した `WallPlate` として存在する（`OutOfFrameMiscWall` の後継）。
//!
//! # 参入レベル（構造壁・n倍法・重量のみ）は型で区別しない
//!
//! 壁が解析にどう参入するか（4 節点要素として剛性・保有水平耐力に算入する「構造壁」、
//! n倍法で偏心率にのみ寄与する「雑壁剛性」、自重のみの「重量のみ」）は、`WallPlate`
//! 自身に列挙型を持たせて利用者に選ばせるのではなく、既存の暗黙規則をそのまま踏襲する
//! （dig Q4=B）。**`section` の有無と、所属する `WallRegion` の種別（囲まれた領域か
//! 取り付き領域か）の組み合わせで、生成ロジック（Step 8・D5）側が決める。**
//!
//! # 自重は必ず断面参照から求める（`OutOfFrameMiscWall` との相違点）
//!
//! 現行 `OutOfFrameMiscWall` は断面を介さず `weight_per_area`（直接入力）と
//! `thickness`（直接入力、n倍法用）を自前で持つ。`WallPlate` はこれを踏襲しない
//! （dig Q5=A）。[`super::Slab`]/[`super::SlabPlate`] と同じく、自重は必ず `section`
//! （厚さ・主材料）から求める。断面未割当は自重 0 とし、解析前チェックが止める
//! （既定厚で補わない）。`OutOfFrameMiscWall` の直接入力経路は、ST-Bridge 取り込みに
//! 生成コードが存在せず実データが 0 件（単体テストの合成データのみ）だったため、
//! 移行対象の実利用がないと判断して廃止した。

use super::*;

/// 壁版の形。[`super::SlabShape`] と同型（囲まれた領域 / 主架構・床領域に取り付く領域）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum WallPlateShape {
    /// 柱・梁が囲む鉛直構面内の領域。境界は [`super::WallRegion`] の境界そのもの、
    /// または間柱で分割した場合はそのサブ境界（節点列。反時計回り、始点は繰り返さない）。
    Enclosed { boundary: Vec<NodeId> },
    /// 主架構・床領域に取り付く領域（パラペット・腰壁・垂れ壁・自立壁）。
    ///
    /// [`RegionAnchor::Line`] の場合、`extent` は D15 の「立ち上がり高さ」
    /// `[d_i, d_j]`（区間の始端側・終端側の高さ [mm]）で、床側（跳ね出し長さ）とは
    /// 張り出す向きが異なる（床は取付き線の左向き法線方向、壁は鉛直上向き）。
    /// [`RegionAnchor::FloorRegion`] の場合も同じ意味（`extent` は高さ、`nodes` は
    /// 壁自体の平面上の始点・終点）。[`RegionAnchor::Point`] は壁の取付き先としては
    /// 使わない（D14 の対応表に壁の用例がなく、出隅スラブ専用のため。
    /// [`WallPlate::boundary_coords`] はこの組み合わせで `None` を返す）。
    Attached {
        anchor: RegionAnchor,
        extent: [f64; 2],
    },
}

/// 壁版。柱・梁が囲む鉛直構面内の版、または主架構・床領域に取り付く版
/// （パラペット・腰壁・垂れ壁・自立壁）ごとに 1 つ。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WallPlate {
    /// 壁版 ID（`Model::wall_plates` の配列インデックスと一致すること）。
    pub id: WallPlateId,
    pub shape: WallPlateShape,
    /// 断面（板厚・材料・開口低減の解決元）。`None` は未割当（自重 0。解析前
    /// チェックが止める。モジュール doc 参照）。
    #[serde(default)]
    pub section: Option<SectionId>,
    /// 開口面積の合計 [mm²]（[`super::WallAttr::opening_area`] の後継）。
    /// `openings`（個別開口）が非空の場合はそちらの面積和を優先し、本フィールドは
    /// 無視される（[`WallPlate::total_opening_area`] 参照）。
    #[serde(default)]
    pub opening_area: f64,
    /// 開口部（サッシ等）の重量 [N]（[`super::WallAttr::opening_weight`] の後継）。
    /// 開口面積控除後の自重に加算する。
    #[serde(default)]
    pub opening_weight: f64,
    /// 個別開口の寸法リスト（[`super::WallOpening`]）。自重控除・開口周比・
    /// 耐震壁検定の開口供給に使う。構造壁でない壁版（n倍法・重量のみ）でも、
    /// 自重控除には意味を持つため共通で持たせる。非空の場合は面積評価を
    /// 優先する（`opening_area` へのフォールバック規約は `WallAttr` と同じ）。
    #[serde(default)]
    pub openings: Vec<WallOpening>,
    /// 三方スリット。true の場合、自重は上下分配せず全て上部の節点へ伝達する。
    /// 要素生成される構造壁のときのみ意味を持つ（[`super::WallRegion`] が
    /// 「囲まれた領域」で、かつ `section` 割当ありの場合）。
    #[serde(default)]
    pub three_side_slit: bool,
}

impl WallPlate {
    /// 取り付く壁版か。
    pub fn is_attached(&self) -> bool {
        matches!(self.shape, WallPlateShape::Attached { .. })
    }

    /// 柱・梁で囲まれた壁版（`Enclosed`）の境界が、壁エレメント（壁柱＋剛梁変換に
    /// よる4節点24自由度モデル。`docs/calc_basis/04_要素剛性/05_壁エレメントモデル.md`
    /// 参照）を組み立てられる形か。境界がちょうど4節点のときだけ `true`。
    ///
    /// 5節点以上（T字取り付き等。他の梁・壁が境界の辺の途中に接続することで
    /// 生じる）や3節点以下は要素を生成しない（Q6=C）。壁エレメントは下辺2節点・
    /// 上辺2節点を前提とした剛体変換の定式化であり、任意の多角形へ一般化する
    /// ことは定式化そのものを崩すため行わない（実データによる検証ができないまま
    /// 「按分」等で無理に4節点へ落とし込むと、根拠不明な近似を耐力評価へ持ち込む
    /// ことになる。dev_docs/handoff/床領域・壁領域の再設計_申し送り.md §9 参照）。
    /// 取り付く壁版（`Attached`）は境界を持たないため常に `false`。
    ///
    /// 解析要素生成（`squid_n_load::wall_expand`）・解析前診断
    /// （`squid-n-solver::precheck`）・ST-Bridge 取り込み（`squid-n-io`）が
    /// 判定を共有する（重複実装の統合）。
    pub fn has_quad_boundary(&self) -> bool {
        matches!(&self.shape, WallPlateShape::Enclosed { boundary } if boundary.len() == 4)
    }

    /// 境界の節点列。**柱・梁が囲む壁版のみ**（取り付く壁版は自由端に節点を
    /// 持たないため `None`）。
    pub fn boundary_nodes(&self) -> Option<&[NodeId]> {
        match &self.shape {
            WallPlateShape::Enclosed { boundary } => Some(boundary),
            WallPlateShape::Attached { .. } => None,
        }
    }

    /// 境界多角形の座標列 [mm]（4 点）。取り付く壁版は取付き先と張り出し量
    /// （鉛直上向きの高さ）から算出する。節点が引けない、または壁の取付き先として
    /// 使わない組み合わせ（`RegionAnchor::Point`）の場合は `None`。
    pub fn boundary_coords(&self, model: &Model) -> Option<Vec<[f64; 3]>> {
        match &self.shape {
            WallPlateShape::Enclosed { boundary } => boundary
                .iter()
                .map(|n| model.nodes.get(n.index()).map(|n| n.coord))
                .collect(),
            WallPlateShape::Attached { anchor, extent } => match anchor {
                RegionAnchor::Line { nodes, span, .. } => {
                    let a = model.nodes.get(nodes[0].index())?.coord;
                    let b = model.nodes.get(nodes[1].index())?.coord;
                    Self::extrude_up(a, b, *span, *extent)
                }
                RegionAnchor::FloorRegion { nodes, .. } => {
                    let a = model.nodes.get(nodes[0].index())?.coord;
                    let b = model.nodes.get(nodes[1].index())?.coord;
                    Self::extrude_up(a, b, [0.0, 1.0], *extent)
                }
                // 壁の取付き先としては使わない（モジュール doc 参照）。
                RegionAnchor::Point(_) => None,
            },
        }
    }

    /// 取付き線（`a`→`b`）の無次元区間 `span` を底辺とし、両端の高さ `extent`
    /// （鉛直上向き）だけ立ち上げた 4 点（反時計回り）を返す。
    fn extrude_up(
        a: [f64; 3],
        b: [f64; 3],
        span: [f64; 2],
        extent: [f64; 2],
    ) -> Option<Vec<[f64; 3]>> {
        let lerp = |t: f64| {
            [
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            ]
        };
        let p0 = lerp(span[0]);
        let p1 = lerp(span[1]);
        Some(vec![
            p0,
            p1,
            [p1[0], p1[1], p1[2] + extent[1]],
            [p0[0], p0[1], p0[2] + extent[0]],
        ])
    }

    /// 壁版の面積 [mm²]（[`crate::geom::polygon_area_3d`]。ニューエルの公式による
    /// 3 次元面積。理想平面への投影を経由しない。壁・シェル要素の自重算定と
    /// スラブ・壁の数量拾いが共通で使う関数をそのまま使う）。座標が引けない場合は 0。
    pub fn area(&self, model: &Model) -> f64 {
        self.boundary_coords(model)
            .map(|pts| crate::geom::polygon_area_3d(&pts))
            .unwrap_or(0.0)
    }

    /// 開口の合計面積 [mm²]。個別開口 `openings` が非空ならその面積和、
    /// 空なら `opening_area` を返す（`WallAttr::total_opening_area` と同じ規約。
    /// 全消費側はこのメソッドを経由すること）。
    pub fn total_opening_area(&self) -> f64 {
        if self.openings.is_empty() {
            self.opening_area.max(0.0)
        } else {
            self.openings.iter().map(WallOpening::area).sum()
        }
    }
}

impl Model {
    /// 壁版 ID から壁版を引く。存在しなければ `None`。
    pub fn wall_plate(&self, id: WallPlateId) -> Option<&WallPlate> {
        match self.wall_plates.get(id.index()) {
            Some(p) if p.id == id => Some(p),
            _ => self.wall_plates.iter().find(|p| p.id == id),
        }
    }

    /// 壁版へ割り当てた断面。未割当・ダングリングは `None`。
    pub fn wall_plate_section(&self, plate: &WallPlate) -> Option<&Section> {
        plate.section.and_then(|sid| self.sections.get(sid.index()))
    }

    /// 壁版の板厚 [mm]（断面の [`Section::thickness`]。[`Model::slab_plate_thickness`]
    /// と同じ規約）。断面未割当、または断面が板厚を持たない場合は `None`。
    pub fn wall_plate_thickness(&self, plate: &WallPlate) -> Option<f64> {
        self.wall_plate_section(plate)
            .and_then(|s| s.thickness)
            .filter(|t| *t > 0.0)
    }

    /// 壁版の主材料。断面未割当・材料未割当は `None`。
    pub fn wall_plate_material(&self, plate: &WallPlate) -> Option<&Material> {
        self.wall_plate_section(plate)
            .and_then(|s| s.material)
            .and_then(|mid| self.materials.get(mid.index()))
    }

    /// 壁版の自重 [N]（開口控除後の正味面積 × 板厚 × 主材料の密度 × 重力加速度
    /// ＋ 開口部（サッシ等）の重量。`WallAttr` の自重算定式と同じ）。
    ///
    /// 断面または主材料が未割当のときは `None`（[`Model::slab_self_weight_intensity`]
    /// と同じ規約。既定厚で補わない）。開口面積が正味面積を超える場合は 0 に丸める。
    pub fn wall_plate_self_weight(&self, plate: &WallPlate, model_for_area: &Model) -> Option<f64> {
        let t = self.wall_plate_thickness(plate)?;
        let mat = self.wall_plate_material(plate)?;
        let area = plate.area(model_for_area);
        let net_area = (area - plate.total_opening_area()).max(0.0);
        let w = mat.density * t * net_area * crate::units::GRAVITY_MM_S2 + plate.opening_weight;
        Some(w.max(0.0))
    }

    /// 自立壁（[`RegionAnchor::FloorRegion`] の取り付く壁版）が、どの床領域の上に
    /// どれだけ載っているかを求める（D17 の分配先の解決）。
    ///
    /// 床領域は主架構から作り直される派生的な入力（D10）で `FloorRegionId` が
    /// 面走査順に振り直されるため、**分配先は保存せず常にここで解決する**
    /// （[`RegionAnchor::FloorRegion`] のドキュメント参照）。
    ///
    /// # 解決の規則
    ///
    /// 壁の下端線分（`nodes` の 2 点）を XY 平面へ落とし、**荷重を流せる床領域**
    /// （壁と同じレベルにあり、床板の XY 投影面積が正である床領域）の多角形で
    /// 厳密にクリップして区間へ分ける。床領域は L 形など非凸になりうるため、
    /// 全交点を求めて区間へ分け、各区間の中点がどの床領域に内包されるかで
    /// 帰属を決める（面走査が返す最小面は重ならないため、内包する床領域は高々 1 つ）。
    ///
    /// 各区間の重量比は、その区間の**台形面積**（立ち上がり高さ `extent` は
    /// 始端から終端へ線形に変わる）が全体に占める割合とする。開口は総重量の
    /// スケールにだけ効き、開口位置は見ない（§5.25 の既知の近似と同じ）。
    ///
    /// # 戻り値
    ///
    /// 自立壁でない、または節点が引けない場合は `None`。それ以外は
    /// [`SelfStandingWallCoverage`] を返す。**どの床領域にも載らない区間**は
    /// `uncovered` に集計する（荷重の行き先が無いモデルの不備。解析前チェックが
    /// エラーにする）。
    pub fn self_standing_wall_coverage(
        &self,
        plate: &WallPlate,
    ) -> Option<SelfStandingWallCoverage> {
        let WallPlateShape::Attached { anchor, extent } = &plate.shape else {
            return None;
        };
        let RegionAnchor::FloorRegion { nodes } = anchor else {
            return None;
        };
        let a = self.nodes.get(nodes[0].index())?.coord;
        let b = self.nodes.get(nodes[1].index())?.coord;
        let (p0, p1) = ([a[0], a[1]], [b[0], b[1]]);
        if (p1[0] - p0[0]).hypot(p1[1] - p0[1]) <= crate::geom::MEMBER_AXIS_TOL_MM {
            // 長さのない壁は面積 0。分配するものが無い。
            return Some(SelfStandingWallCoverage::default());
        }
        // 壁の載るレベルは下端線分の平均標高。床領域のレベル一致判定は
        // `region_rebuild` と同じ `LEVEL_TOL_MM` を用いる。
        let z = (a[2] + b[2]) / 2.0;

        // 荷重を流せる床領域だけを候補にする（床板を 1 枚も持たない床領域、
        // および床板の XY 面積が 0 の床領域は、等価面荷重へならす先が無い）。
        let candidates: Vec<(FloorRegionId, Vec<[f64; 2]>)> = self
            .floor_regions
            .iter()
            .filter(|r| {
                r.level(self)
                    .is_some_and(|rz| (rz - z).abs() <= crate::geom::LEVEL_TOL_MM)
            })
            .filter(|r| self.floor_region_slab_xy_area(r) > 0.0)
            .filter_map(|r| {
                let poly: Vec<[f64; 2]> = r
                    .boundary_coords(self)?
                    .iter()
                    .map(|c| [c[0], c[1]])
                    .collect();
                (poly.len() >= 3).then_some((r.id, poly))
            })
            .collect();

        // 区間の切れ目: 線分と全候補多角形の辺との交点パラメータ。
        let mut cuts: Vec<f64> = vec![0.0, 1.0];
        for (_, poly) in &candidates {
            for i in 0..poly.len() {
                let q0 = poly[i];
                let q1 = poly[(i + 1) % poly.len()];
                if let Some(t) = segment_intersection_t(p0, p1, q0, q1) {
                    cuts.push(t);
                }
            }
        }
        cuts.retain(|t| t.is_finite() && (0.0..=1.0).contains(t));
        cuts.sort_by(|x, y| x.total_cmp(y));
        cuts.dedup_by(|x, y| (*x - *y).abs() <= 1e-12);

        let lerp = |t: f64| [p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t];
        // 立ち上がり高さは始端から終端へ線形。区間の台形面積比が重量比になる。
        let h = |t: f64| (extent[0] + (extent[1] - extent[0]) * t).abs();
        let total_area = (h(0.0) + h(1.0)) / 2.0;

        let mut cov = SelfStandingWallCoverage::default();
        for w in cuts.windows(2) {
            let (t0, t1) = (w[0], w[1]);
            if t1 - t0 <= 1e-12 {
                continue;
            }
            let frac = if total_area > 0.0 {
                (h(t0) + h(t1)) / 2.0 * (t1 - t0) / total_area
            } else {
                // 高さ 0 の壁は重量 0。長さ比で持たせても総和は 0 のまま。
                t1 - t0
            };
            let mid = lerp((t0 + t1) / 2.0);
            match candidates
                .iter()
                .find(|(_, poly)| crate::region_gen::polygon_contains_strict(poly, mid))
            {
                Some((id, _)) => match cov.per_region.iter_mut().find(|(r, _)| r == id) {
                    Some((_, f)) => *f += frac,
                    None => cov.per_region.push((*id, frac)),
                },
                None => cov.uncovered += frac,
            }
        }
        Some(cov)
    }

    /// 床領域に属する床板の XY 投影面積の合計 [mm²]。
    /// 等価面荷重の分母（[`Self::self_standing_wall_coverage`] の候補判定）に使う。
    fn floor_region_slab_xy_area(&self, region: &FloorRegion) -> f64 {
        region
            .slab_ids
            .iter()
            .filter_map(|&id| self.slab(id))
            .filter_map(|s| s.boundary_coords(self))
            .map(|pts| {
                // 床の分配と同じ XY 投影面積（`squid-n-load` の `floor::polygon_area`
                // と同じ規約）。3 次元面積で測ると、面荷重強度を掛ける側が XY 面積を
                // 使うため総重量が縮み、傾斜床で危険側になる。
                let xy: Vec<[f64; 2]> = pts.iter().map(|c| [c[0], c[1]]).collect();
                crate::region_gen::signed_area_abs(&xy)
            })
            .sum()
    }
}

/// 自立壁がどの床領域の上にどれだけ載っているか
/// （[`Model::self_standing_wall_coverage`] の結果）。
///
/// 比率はいずれも壁の全重量に対する割合で、`per_region` の合計と `uncovered` を
/// 足すと 1 になる（丸め誤差を除く）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SelfStandingWallCoverage {
    /// 載っている床領域と、その床領域が受け持つ重量比。
    pub per_region: Vec<(FloorRegionId, f64)>,
    /// どの床領域にも載らない部分の重量比。**0 でなければモデルの不備**
    /// （荷重の行き先が無い。解析前チェックがエラーにする）。
    pub uncovered: f64,
}

impl SelfStandingWallCoverage {
    /// 荷重の行き先が無い部分を持つか（判定のしきい値をここに集約する）。
    ///
    /// 交点計算の丸め、および壁の端がちょうど床領域の境界に載る場合の
    /// ごく短い区間を拾わないよう、全体の 0.1% を下回る取りこぼしは無視する。
    pub fn has_uncovered(&self) -> bool {
        self.uncovered > 1e-3
    }
}

/// 線分 `p0`→`p1` と線分 `q0`→`q1` の交点の、`p` 側のパラメータ `t`（0〜1）。
/// 平行・端点の縮退・交差しない場合は `None`。
fn segment_intersection_t(p0: [f64; 2], p1: [f64; 2], q0: [f64; 2], q1: [f64; 2]) -> Option<f64> {
    let r = [p1[0] - p0[0], p1[1] - p0[1]];
    let s = [q1[0] - q0[0], q1[1] - q0[1]];
    let denom = r[0] * s[1] - r[1] * s[0];
    if denom.abs() <= f64::EPSILON {
        return None; // 平行または縮退（重なりは区間の中点判定側で拾う）。
    }
    let qp = [q0[0] - p0[0], q0[1] - p0[1]];
    let t = (qp[0] * s[1] - qp[1] * s[0]) / denom;
    let u = (qp[0] * r[1] - qp[1] * r[0]) / denom;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some(t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FloorRegionId, MaterialId, NodeId, SectionId, WallPlateId};

    fn model_with_nodes(pts: &[[f64; 3]]) -> Model {
        let mut m = Model::default();
        for (i, p) in pts.iter().enumerate() {
            m.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *p,
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        m
    }

    #[test]
    fn test_enclosed_boundary_coords_and_area() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let coords = p.boundary_coords(&m).expect("境界座標");
        assert_eq!(coords.len(), 4);
        assert!((p.area(&m) - 4000.0 * 3000.0).abs() < 1e-6);
    }

    /// `has_quad_boundary` は境界がちょうど4節点の `Enclosed` 壁版のみ `true`。
    /// 5節点以上（T字取り付き等）・3節点以下・`Attached` は `false`
    /// （Q6=C。壁エレメントの定式化を崩す一般化は行わない）。
    #[test]
    fn test_has_quad_boundary() {
        let quad = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        assert!(quad.has_quad_boundary());

        let mut pentagon = quad.clone();
        pentagon.shape = WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
        };
        assert!(!pentagon.has_quad_boundary(), "5節点は false");

        let mut triangle = quad.clone();
        triangle.shape = WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2)],
        };
        assert!(!triangle.has_quad_boundary(), "3節点は false");

        let mut attached = quad.clone();
        attached.shape = WallPlateShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [900.0, 900.0],
        };
        assert!(
            !attached.has_quad_boundary(),
            "Attached は境界を持たないため false"
        );
    }

    /// 取付き線アンカーは、床の取り付く床板（左向き法線方向へ張り出す）とは異なり、
    /// 鉛直上向きへ立ち上げる（D15「壁は立ち上がり高さ」）。
    #[test]
    fn test_attached_line_extrudes_upward_not_sideways() {
        let m = model_with_nodes(&[[0.0, 0.0, 3000.0], [4000.0, 0.0, 3000.0]]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: [900.0, 900.0],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let coords = p.boundary_coords(&m).expect("境界座標");
        // 4点とも Y=0（左向き法線方向へは動かない）、上 2 点は Z=3900（+900 立ち上げ）。
        for c in &coords {
            assert_eq!(c[1], 0.0, "Y 方向へは張り出さない: {coords:?}");
        }
        assert_eq!(coords[2][2], 3900.0);
        assert_eq!(coords[3][2], 3900.0);
        assert!((p.area(&m) - 4000.0 * 900.0).abs() < 1e-6);
    }

    /// 床領域アンカーは、アンカー自身が持つ節点対を壁の平面上の始点・終点として使う
    /// （dig Q6=B）。所属先の床領域 ID は幾何計算には関与しない。
    #[test]
    fn test_attached_floor_region_anchor_uses_its_own_nodes_for_length() {
        let m = model_with_nodes(&[[0.0, 0.0, 3000.0], [2000.0, 0.0, 3000.0]]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    nodes: [NodeId(0), NodeId(1)],
                },
                extent: [2500.0, 2500.0],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        assert!((p.area(&m) - 2000.0 * 2500.0).abs() < 1e-6);
    }

    /// 壁の取付き先として `RegionAnchor::Point` は使わない（D14 の対応表参照）。
    #[test]
    fn test_attached_point_anchor_is_unsupported_for_wall() {
        let m = model_with_nodes(&[[0.0, 0.0, 3000.0]]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Point(NodeId(0)),
                extent: [900.0, 900.0],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        assert_eq!(p.boundary_coords(&m), None);
        assert_eq!(p.area(&m), 0.0);
    }

    #[test]
    fn test_self_weight_none_without_section() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        assert_eq!(m.wall_plate_self_weight(&p, &m), None);
    }

    #[test]
    fn test_self_weight_deducts_opening_area() {
        let mut m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        m.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        m.sections.push(Section {
            id: SectionId(0),
            name: "壁 t150".into(),
            area: 150.0 * 3000.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 3000.0,
            width: 150.0,
            as_y: 1.0,
            as_z: 1.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![WallOpening {
                width: 900.0,
                height: 1200.0,
                offset: Some([1550.0, 0.0]),
            }],
            three_side_slit: false,
        };
        let gross_area = 4000.0 * 3000.0;
        let opening_area = 900.0 * 1200.0;
        let expected = 150.0 * 2.4e-9 * (gross_area - opening_area) * crate::units::GRAVITY_MM_S2;
        let w = m.wall_plate_self_weight(&p, &m).expect("自重が求まる");
        assert!(
            (w - expected).abs() / expected < 1e-9,
            "自重 {w}（期待値 {expected}）"
        );
    }

    fn model_with_wall_section() -> Model {
        let mut m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        m.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        m.sections.push(Section {
            id: SectionId(0),
            name: "壁 t150".into(),
            area: 150.0 * 3000.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 3000.0,
            width: 150.0,
            as_y: 1.0,
            as_z: 1.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        m
    }

    /// `openings`（個別開口）が空の場合は `opening_area`（合計面積のみ入力）に
    /// フォールバックする（`WallAttr::total_opening_area` と同じ規約）。
    #[test]
    fn test_total_opening_area_falls_back_to_opening_area_field() {
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: None,
            opening_area: 999.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        assert!((p.total_opening_area() - 999.0).abs() < 1e-9);
    }

    /// 開口部（サッシ等）の重量は、開口面積控除後の自重に加算する。
    #[test]
    fn test_self_weight_adds_opening_weight() {
        let m = model_with_wall_section();
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 1234.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let gross_area = 4000.0 * 3000.0;
        let base = 150.0 * 2.4e-9 * gross_area * crate::units::GRAVITY_MM_S2;
        let expected = base + 1234.0;
        let w = m.wall_plate_self_weight(&p, &m).expect("自重が求まる");
        assert!(
            (w - expected).abs() / expected < 1e-9,
            "自重 {w}（期待値 {expected}）"
        );
    }

    /// 床領域 2 つ（4m ずつ、Z=3000）と、それぞれに床板を持つモデル。
    /// X = 0..8000, Y = 0..4000 を X=4000 で 2 分割した並びとする。
    fn model_two_regions() -> Model {
        // 床領域の境界節点（0..5）と、自立壁の両端（6, 7）。
        let mut m = model_with_nodes(&[
            [0.0, 0.0, 3000.0],
            [4000.0, 0.0, 3000.0],
            [8000.0, 0.0, 3000.0],
            [8000.0, 4000.0, 3000.0],
            [4000.0, 4000.0, 3000.0],
            [0.0, 4000.0, 3000.0],
        ]);
        for (ri, b) in [
            vec![NodeId(0), NodeId(1), NodeId(4), NodeId(5)],
            vec![NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
        ]
        .into_iter()
        .enumerate()
        {
            let mut r = FloorRegion::new(FloorRegionId(ri as u32), b.clone());
            r.slab_ids.push(SlabId(ri as u32));
            m.floor_regions.push(r);
            m.slabs.push(Slab {
                id: SlabId(ri as u32),
                shape: SlabShape::Enclosed { boundary: b },
                plate: SlabPlate::default(),
            });
        }
        m
    }

    fn push_self_standing(m: &mut Model, a: [f64; 3], b: [f64; 3], extent: [f64; 2]) -> WallPlate {
        let i = m.nodes.len() as u32;
        for p in [a, b] {
            m.nodes.push(Node {
                id: NodeId(m.nodes.len() as u32),
                coord: p,
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    nodes: [NodeId(i), NodeId(i + 1)],
                },
                extent,
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            three_side_slit: false,
        }
    }

    /// 1 つの床領域に収まる自立壁は、その床領域が全量を受け持つ。
    #[test]
    fn coverage_within_single_region() {
        let mut m = model_two_regions();
        let p = push_self_standing(
            &mut m,
            [500.0, 1000.0, 3000.0],
            [3500.0, 1000.0, 3000.0],
            [2000.0, 2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(!cov.has_uncovered(), "{cov:?}");
        assert_eq!(cov.per_region.len(), 1);
        assert_eq!(cov.per_region[0].0, FloorRegionId(0));
        assert!((cov.per_region[0].1 - 1.0).abs() < 1e-9, "{cov:?}");
    }

    /// 床領域をまたぐ自立壁は、境界で分割して長さ比で配る（矩形なら長さ比＝重量比）。
    #[test]
    fn coverage_splits_across_regions_by_length() {
        let mut m = model_two_regions();
        // X=2000..6000 の壁。X=4000 の境界で 2:2 に分かれる。
        let p = push_self_standing(
            &mut m,
            [2000.0, 1000.0, 3000.0],
            [6000.0, 1000.0, 3000.0],
            [2000.0, 2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(!cov.has_uncovered(), "{cov:?}");
        assert_eq!(cov.per_region.len(), 2, "{cov:?}");
        for (_, f) in &cov.per_region {
            assert!((f - 0.5).abs() < 1e-9, "{cov:?}");
        }
        let sum: f64 = cov.per_region.iter().map(|(_, f)| f).sum();
        assert!((sum - 1.0).abs() < 1e-9, "総和が保存すること: {cov:?}");
    }

    /// 台形（立ち上がり高さが両端で異なる）は、長さ比ではなく台形面積比で配る。
    #[test]
    fn coverage_splits_by_trapezoid_area_not_length() {
        let mut m = model_two_regions();
        // X=0..8000 を X=4000 で 2 分割。高さは 0 → 4000 の線形。
        // 前半の面積比 = (0+2000)/2*0.5 / ((0+4000)/2) = 500/2000 = 0.25
        let p = push_self_standing(
            &mut m,
            [10.0, 1000.0, 3000.0],
            [7990.0, 1000.0, 3000.0],
            [0.0, 4000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(!cov.has_uncovered(), "{cov:?}");
        let f0 = cov
            .per_region
            .iter()
            .find(|(r, _)| *r == FloorRegionId(0))
            .expect("領域0")
            .1;
        assert!((f0 - 0.25).abs() < 2e-3, "台形面積比になること: {f0}");
        let sum: f64 = cov.per_region.iter().map(|(_, f)| f).sum();
        assert!((sum - 1.0).abs() < 1e-9, "総和が保存すること: {cov:?}");
    }

    /// 床領域の外へはみ出す自立壁は、その分を `uncovered` に集計する。
    #[test]
    fn coverage_reports_uncovered_outside_regions() {
        let mut m = model_two_regions();
        // X=6000..10000。X=8000 より外は床領域が無い。
        let p = push_self_standing(
            &mut m,
            [6000.0, 1000.0, 3000.0],
            [10000.0, 1000.0, 3000.0],
            [2000.0, 2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(cov.has_uncovered(), "{cov:?}");
        assert!((cov.uncovered - 0.5).abs() < 1e-9, "{cov:?}");
    }

    /// 床板を持たない床領域は「力を流す先が無い」ため候補にしない。
    #[test]
    fn coverage_ignores_regions_without_slabs() {
        let mut m = model_two_regions();
        m.floor_regions[0].slab_ids.clear();
        let p = push_self_standing(
            &mut m,
            [500.0, 1000.0, 3000.0],
            [3500.0, 1000.0, 3000.0],
            [2000.0, 2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(
            cov.has_uncovered(),
            "版なし床領域は覆いとみなさない: {cov:?}"
        );
        assert!((cov.uncovered - 1.0).abs() < 1e-9, "{cov:?}");
    }

    /// レベルが床領域と一致しない自立壁は、どの床領域にも載らない。
    #[test]
    fn coverage_requires_level_match() {
        let mut m = model_two_regions();
        let p = push_self_standing(
            &mut m,
            [500.0, 1000.0, 6000.0],
            [3500.0, 1000.0, 6000.0],
            [2000.0, 2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(cov.has_uncovered(), "{cov:?}");
        assert!((cov.uncovered - 1.0).abs() < 1e-9, "{cov:?}");
    }

    /// 取り付く壁版でない（囲まれた）壁版は対象外。
    #[test]
    fn coverage_is_none_for_enclosed() {
        let m = model_two_regions();
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(4), NodeId(5)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            three_side_slit: false,
        };
        assert!(m.self_standing_wall_coverage(&p).is_none());
    }
}
