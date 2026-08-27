//! 壁領域（`WallRegion`）。柱・梁が囲む鉛直構面内の閉領域。
//!
//! 床領域（[`super::FloorRegion`]、[`super::region`]）と対になる型で、位置づけは同じ
//! ([`super::region`] のモジュール doc 参照)。壁領域は「柱・梁が囲む鉛直構面内の
//! 閉領域ごとに 1 つ」（D1）であり、境界は [`crate::region_gen::wall`] の面走査から
//! 作る。版の仕様は持たない（[`super::WallPlate`] が持つ）。
//!
//! # 壁領域と壁版の違い
//!
//! - **壁領域**（本モジュール）: 柱・梁が囲む鉛直構面内の閉領域そのもの。境界は
//!   柱・梁の閉路（節点列）で、[`crate::region_gen::wall`] の面走査から作る。
//!   **1 つの閉領域につき 1 つ**とする（D1）。版の仕様は持たない。間柱
//!   （[`WallRegion::post_ids`]）と、区画内の壁版一覧（[`WallRegion::wall_plate_ids`]）
//!   を持つ。
//! - **壁版**（[`super::WallPlate`]）: 柱・梁で囲まれた版、または主架構・床領域に
//!   取り付く版（パラペット・腰壁・垂れ壁・自立壁）。断面（板厚・材料）・開口は
//!   ここが持つ。1 つの壁領域は複数の壁版を持ちうる（E5。区画内が間柱でさらに
//!   細かい壁パネルに分かれている場合）。取り付く壁版はどの壁領域からも参照
//!   されない独立した壁版として存在する。
//!
//! `region_gen::wall` の出力から組み立てる経路（[`crate::wall_region_rebuild`]）は
//! 準備計算・ST-Bridge 取り込みへ結線済み（`wall_plate_ids` へ壁版を割り当てる経路
//! 含む）。**未結線（2026-08-26 時点）**なのは、壁版の ST-Bridge 取り込み・要素生成
//! （D5）・断面力/保有水平耐力の参照張り替え。ST-Bridge 書き出しは壁版が正。詳細は
//! `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §5.10・§5.13。

use super::*;

/// 壁領域。柱・梁が囲む鉛直構面内の閉領域（D1）。版の仕様は持たない
/// （[`WallPlate`] が持つ）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WallRegion {
    /// 壁領域 ID（`Model::wall_regions` の配列インデックスと一致すること）。
    pub id: WallRegionId,
    /// 表示名（ナビゲータ・診断で領域を指し示すために用いる）。空文字は名前なし。
    #[serde(default)]
    pub name: String,
    /// 境界の節点列（柱・梁の閉路。反時計回り、始点は繰り返さない）。
    #[serde(default)]
    pub boundary: Vec<NodeId>,
    /// この壁領域に属する壁版（[`WallPlate`]）の ID リスト。区画内が間柱で複数の
    /// 壁パネルに分かれている場合、複数持ちうる（E5）。順序は任意。重複・他領域との
    /// 共有は許さない（[`Model::validate`] が確認）。版なし壁領域（間柱のみの
    /// 雑壁領域等）は空のままでよい。
    #[serde(default)]
    pub wall_plate_ids: Vec<WallPlateId>,
    /// この壁領域に属する間柱（`SecondaryMember::Post`）の ID リスト。
    #[serde(default)]
    pub post_ids: Vec<SecondaryMemberId>,
}

impl WallRegion {
    /// 壁領域を作る（版なし・間柱なし）。
    pub fn new(id: WallRegionId, boundary: Vec<NodeId>) -> Self {
        WallRegion {
            id,
            name: String::new(),
            boundary,
            wall_plate_ids: Vec::new(),
            post_ids: Vec::new(),
        }
    }

    /// 境界多角形の座標列 [mm]。節点が引けない（陳腐化した参照）場合は `None`。
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

    /// 境界の面積 [mm²]（[`crate::geom::polygon_area_3d`]。ニューエルの公式による
    /// 3 次元面積。理想平面への投影を経由しない。§3.2 E3）。座標が引けない場合は 0。
    pub fn area(&self, model: &Model) -> f64 {
        self.boundary_coords(model)
            .map(|pts| crate::geom::polygon_area_3d(&pts))
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
