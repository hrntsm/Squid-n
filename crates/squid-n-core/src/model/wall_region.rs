//! 壁領域（`WallRegion`）。柱・梁が囲む鉛直構面内の閉領域。
//!
//! 版の仕様は持たない（[`super::WallPlate`] が持つ）。

use super::*;

/// 壁領域。柱・梁が囲む鉛直構面内の閉領域。版の仕様は持たない
/// （[`WallPlate`] が持つ）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WallRegion {
    /// 壁領域 ID（`Model::wall_regions` の配列インデックスと一致すること）。
    pub id: WallRegionId,
    /// 表示名。空文字は名前なし。
    #[serde(default)]
    pub name: String,
    /// 境界の節点列（柱・梁の閉路。反時計回り、始点は繰り返さない）。
    #[serde(default)]
    pub boundary: Vec<NodeId>,
    /// この壁領域に属する壁版（[`WallPlate`]）の ID リスト。順序は任意。重複・他領域との
    /// 共有は許さない。版なし壁領域は空のままでよい。
    #[serde(default)]
    pub wall_plate_ids: Vec<WallPlateId>,
    /// この壁領域に属する間柱の実体。
    #[serde(default)]
    pub posts: Vec<SecondaryMember>,
}

impl WallRegion {
    /// 壁領域を作る（版なし・間柱なし）。
    pub fn new(id: WallRegionId, boundary: Vec<NodeId>) -> Self {
        WallRegion {
            id,
            name: String::new(),
            boundary,
            wall_plate_ids: Vec::new(),
            posts: Vec::new(),
        }
    }

    /// 境界多角形の座標列 [mm]。節点が引けない場合は `None`。
    pub fn boundary_coords(&self, model: &Model) -> Option<Vec<[f64; 3]>> {
        self.boundary
            .iter()
            .map(|n| model.nodes.get(n.index()).map(|n| n.coord))
            .collect()
    }

    /// 境界の辺 `k` の両端節点。
    pub fn edge_nodes(&self, k: usize) -> Option<[NodeId; 2]> {
        let n = self.boundary.len();
        (n >= 3 && k < n).then(|| [self.boundary[k], self.boundary[(k + 1) % n]])
    }

    /// 領域を代表する節点（境界の先頭）。
    pub fn reference_node(&self) -> Option<NodeId> {
        self.boundary.first().copied()
    }

    /// 境界の面積 [mm²]。座標が引けない場合は 0。
    pub fn area(&self, model: &Model) -> f64 {
        self.boundary_coords(model)
            .map(|pts| crate::geom::polygon::area_3d(&pts))
            .unwrap_or(0.0)
    }
}

impl Model {
    /// 壁領域 ID から壁領域を引く。存在しなければ `None`。
    pub fn wall_region(&self, id: WallRegionId) -> Option<&WallRegion> {
        match self.wall_regions.get(id.index()) {
            Some(r) if r.id == id => Some(r),
            _ => self.wall_regions.iter().find(|r| r.id == id),
        }
    }

    /// 壁領域の境界多角形の内側（境界を除く）に点 `p` [mm] があるか。
    /// 境界座標・構面が引けない場合は `false`。
    pub fn wall_region_contains_point(&self, id: WallRegionId, p: [f64; 3]) -> bool {
        use crate::geom::vec3;
        let Some((poly, to_2d, normal, origin)) = self.wall_region_polygon(id) else {
            return false;
        };
        if vec3::dot(vec3::sub(p, origin), normal).abs() > crate::geom::MEMBER_AXIS_TOL_MM {
            return false;
        }
        crate::geom::polygon::contains_excluding_boundary(&poly, to_2d(p))
    }

    /// 壁領域の境界多角形の内側または境界上（[`crate::geom::MEMBER_AXIS_TOL_MM`] 以内）に
    /// 点 `p` [mm] があるか。境界座標・構面が引けない場合は `false`。
    pub fn wall_region_contains_point_including_boundary(
        &self,
        id: WallRegionId,
        p: [f64; 3],
    ) -> bool {
        use crate::geom::vec3;
        let Some((poly, to_2d, normal, origin)) = self.wall_region_polygon(id) else {
            return false;
        };
        if vec3::dot(vec3::sub(p, origin), normal).abs() > crate::geom::MEMBER_AXIS_TOL_MM {
            return false;
        }
        crate::geom::polygon::contains_within_tol(&poly, to_2d(p), crate::geom::MEMBER_AXIS_TOL_MM)
    }

    /// 壁領域の境界を構面の局所 2D 座標へ写す基底と多角形を返す。
    /// `(多角形, 点を 2D へ写す関数, 構面の法線, 構面上の基準点)`。縮退は `None`。
    #[allow(clippy::type_complexity)]
    fn wall_region_polygon(
        &self,
        id: WallRegionId,
    ) -> Option<(
        Vec<[f64; 2]>,
        impl Fn([f64; 3]) -> [f64; 2],
        [f64; 3],
        [f64; 3],
    )> {
        use crate::geom::vec3;
        let region = self.wall_region(id)?;
        let coords = region.boundary_coords(self)?;
        if coords.len() < 3 {
            return None;
        }
        let origin = coords[0];
        let u = coords[1..]
            .iter()
            .find_map(|c| vec3::unit(vec3::sub(*c, origin)))?;
        let normal = vec3::unit(vec3::cross(
            vec3::sub(coords[1], origin),
            vec3::sub(coords[2], origin),
        ))?;
        let v = vec3::cross(normal, u);
        let to_2d = move |q: [f64; 3]| {
            let d = vec3::sub(q, origin);
            [vec3::dot(d, u), vec3::dot(d, v)]
        };
        let poly: Vec<[f64; 2]> = coords.iter().map(|&c| to_2d(c)).collect();
        Some((poly, to_2d, normal, origin))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::NodeId;

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
    fn test_boundary_coords_and_area() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        let r = WallRegion::new(
            WallRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        );
        let coords = r.boundary_coords(&m).expect("境界座標");
        assert_eq!(coords.len(), 4);
        assert!((r.area(&m) - 4000.0 * 3000.0).abs() < 1e-6);
        assert_eq!(r.reference_node(), Some(NodeId(0)));
        assert_eq!(r.edge_nodes(0), Some([NodeId(0), NodeId(1)]));
    }
}
