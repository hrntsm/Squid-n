//! 床領域（[`FloorRegion`]）。大梁の 1 スパン区画。
//!
//! 床領域は「大梁が囲む区画」であり、その中に小梁（[`SecondaryMember`]）と
//! 床板（[`Slab`]、[`super::slab`]）をまとめる単位である。設計の経緯と決定事項は
//! `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` を参照。
//!
//! # 床領域と床板の違い
//!
//! - **床領域**（本モジュール）: 大梁が囲む 1 区画そのもの。境界は大梁の閉路（節点列）で、
//!   [`crate::region_gen`] の走査から作る。**1 つの閉領域につき 1 つ**とする（D1）。
//!   版の仕様は持たない。小梁（[`FloorRegion::secondary_joists`]）と、
//!   床領域内の床板一覧（[`FloorRegion::slab_ids`]）を持つ。
//! - **床板**（[`Slab`]、[`super::slab`]）: 大梁または小梁で囲まれた版、
//!   または主架構に取り付く版（片持ち・バルコニー・出隅）。厚さ・材料・仕上げ荷重・
//!   室用途はここが持つ。1 つの床領域は複数の床板を持ちうる
//!   （床領域内が小梁でさらに細かい打設単位に分かれている場合）。
//!   片持ちスラブはどの床領域からも参照されない独立した床板として存在する。

use super::*;

/// 取り付く床板の取付き先。
///
/// 節点で指すのは、節点を動かしても追随し、取付き先の大梁を分割しても外れないためである
/// （区間は節点対の間の相対位置なので、間に節点が増えても変わらない）。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum RegionAnchor {
    /// 線に取り付く（片持ちスラブ・バルコニー）。
    ///
    /// `nodes` は取付き線の両端、`span` はその線上の無次元区間 `[t_i, t_j]`（0.0〜1.0、
    /// 全長は `[0.0, 1.0]`）。梁の一部だけに載る場合に用いる。
    ///
    /// 張り出し量 `extent` は `[d_i, d_j]`（区間の始端側・終端側）で、
    /// **符号は取付き線 `nodes[0]`→`nodes[1]` の左側を正とする**。
    ///
    /// 荷重の出口（`transfer`）を選べるのはこの形だけである。点に取り付く床板は
    /// その節点への集中しかありえないため、値を持たせると無意味な組み合わせを
    /// 表現できてしまう。
    Line {
        nodes: [NodeId; 2],
        span: [f64; 2],
        transfer: LoadTransfer,
    },
    /// 点（柱）に取り付く（出隅の片持ちスラブ）。荷重はその節点へ集中する。
    ///
    /// 張り出し量 `extent` は全体座標の `[X 方向, Y 方向]` で、符号が向きを表す。
    Point(NodeId),
    /// 床領域に取り付く（自立壁。床の上に立つ間仕切り等）。荷重は壁が載っている
    /// 床領域へ渡し、等価な面荷重へならして分配する（D17）。
    ///
    /// `nodes` は壁の始点・終点（節点参照。`Line` と同じ理由で節点移動に追随させる）。
    /// 面積（＝自重の計算）にはこの2点間の距離を使う。
    ///
    /// **荷重を渡す床領域は保存せず、壁の位置から都度求める**
    /// （[`Model::self_standing_wall_coverage`]）。床領域は主架構から作り直される
    /// 派生的な入力（D10）で、`FloorRegionId` は作り直しのたびに面走査順で振り直される。
    /// ID を保存すると、モデルの位相が変わった瞬間に別の床領域を指すようになり、
    /// 自立壁の自重が黙って別の階へならされる。保存しなければこの陳腐化は起こりえない。
    ///
    /// 壁が複数の床領域にまたがる場合は、床領域の境界で内部的に分割して、
    /// それぞれの床領域へ ∫|立ち上がり高さ| の比で重量を配る。矩形なら長さ比と一致する。
    /// 両端で高さが符号反転するときは高さ 0 で折る（端点絶対値の台形ではない）。
    /// どの床領域にも載らない部分が残る壁は、荷重の行き先がないモデルの不備として
    /// 解析前チェックが止める。
    ///
    /// **面荷重へならす時点で、壁が床領域内のどこにあるかという位置情報は失われる**
    /// （厳密に扱いたい場合は壁の直下に小梁を入れて主架構へ取り付ける運用とする）。
    /// 失われるのは位置であって長さではない。
    ///
    /// 床板（[`super::Slab`]）の取付き先としては使わない。床板が「床領域に取り付く」
    /// 構図は D13 の 2 種別（囲まれた領域・取り付き領域）のどちらにも該当せず生じない
    /// （床は主架構またはほかの床板の辺に取り付くのであって、床領域そのものには
    /// 取り付かない）。壁側（[`super::WallPlate`] の `Attached` 形）専用のアンカーである。
    FloorRegion { nodes: [NodeId; 2] },
}

/// 取付き線の無次元区間 `span = [t_i, t_j]` が規約 `0.0 <= t_i < t_j <= 1.0`
/// を満たすか。
///
/// 判定の情報源を 1 つに保つため、区間の妥当性が要るところは常にこの関数を使う。
/// `Model::validate`（保存時の最終防衛線）・`squid-n-edit` の編集コマンド・
/// GUI の入力欄がそれぞれ同じ式を書いていたが、GUI 側だけ下の許容を持たず、
/// 「保存はできるが GUI で編集し直すと弾かれる」非対称になっていた。
///
/// 両端に `1e-9` の許容を持たせるのは、`span` が浮動小数の無次元比であり、
/// `1.0` を意図した値が計算経路で `1.0 + ε` になりうるためである。区間が
/// つぶれている（`t_j - t_i` が許容以下）ものは、載る範囲が無いので弾く。
pub fn span_is_valid(span: [f64; 2]) -> bool {
    span[0].is_finite()
        && span[1].is_finite()
        && span[0] >= -1e-9
        && span[1] <= 1.0 + 1e-9
        && span[1] - span[0] > 1e-9
}

/// 取り付く床板の荷重の出口（[`RegionAnchor::Line`] が持つ）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LoadTransfer {
    /// 取付き線へ分布させる（既定。片持ちスラブ・梁に載るパラペット）。
    #[default]
    Anchor,
    /// 取付き線の両端（柱）へ集中させる（出隅・雑壁の柱伝達）。
    Columns,
}

/// 床領域。大梁が囲む 1 スパン区画（D1）。版の仕様は持たない（[`Slab`] が持つ）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FloorRegion {
    /// 床領域 ID（`Model::floor_regions` の配列インデックスと一致すること）。
    pub id: FloorRegionId,
    /// 表示名（ナビゲータ・診断で領域を指し示すために用いる）。空文字は名前なし。
    #[serde(default)]
    pub name: String,
    /// 境界の節点列（大梁の閉路。反時計回り、始点は繰り返さない）。
    pub boundary: Vec<NodeId>,
    /// この床領域に属する小梁（`SecondaryMember::Joist`）の実体。
    /// リスト内の順序は任意。
    #[serde(default)]
    pub secondary_joists: Vec<SecondaryMember>,
    /// この床領域に属する床板の ID リスト。床領域内が小梁で複数の打設単位に
    /// 分かれている場合、複数持ちうる。順序は任意。重複・他領域との共有は許さない
    /// （`Model::validate` が確認）。
    #[serde(default)]
    pub slab_ids: Vec<SlabId>,
}

impl Model {
    /// 床領域の境界節点が**すべて同一の剛床**に属するか。
    ///
    /// 小梁の断面検定（分配 Span 検定）は、曲げ・せん断・たわみだけを見て軸力を
    /// 見ない。これは「その小梁が載る床面が 1 枚の剛体で、面内力を剛床が処理して
    /// いる」ことを暗黙の前提にしている。剛床でない床の小梁は面内力を負担する
    /// ため、この前提が成り立たず検定できない（検定側は「未」にする）。
    ///
    /// **判定は節点の集合で行う。** 剛床のスレーブ集合に境界節点がすべて含まれる
    /// ことを求める。将来、節点ごとに剛床の対象を変えられるようにしても、この
    /// 判定はそのまま使える。
    ///
    /// 複数の剛床にまたがる床領域（段差床の階は 1 階に複数の剛床を持つ）は偽と
    /// する。境界で相対変位が生じうるため、面内力が小梁へ回る可能性が残る。
    pub fn floor_region_on_single_diaphragm(&self, region: &FloorRegion) -> bool {
        if region.boundary.is_empty() {
            return false;
        }
        self.constraints.iter().any(|c| match c {
            Constraint::RigidDiaphragm { master, slaves, .. } => region
                .boundary
                .iter()
                .all(|n| n == master || slaves.contains(n)),
            _ => false,
        })
    }

    /// 二次部材（端点対で識別）が属する床領域。どこにも属さなければ `None`。
    pub fn floor_region_of_joist(&self, nodes: [NodeId; 2]) -> Option<&FloorRegion> {
        let key = |a: NodeId, b: NodeId| (a.0.min(b.0), a.0.max(b.0));
        let want = key(nodes[0], nodes[1]);
        self.floor_regions.iter().find(|r| {
            r.secondary_joists
                .iter()
                .any(|j| key(j.nodes[0], j.nodes[1]) == want)
        })
    }
}

impl FloorRegion {
    /// 床領域を作る（版なし・小梁なし）。
    pub fn new(id: FloorRegionId, boundary: Vec<NodeId>) -> Self {
        FloorRegion {
            id,
            name: String::new(),
            boundary,
            secondary_joists: Vec::new(),
            slab_ids: Vec::new(),
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

    /// 領域を代表する節点（境界の先頭）。階の帰属や診断の表示など、
    /// 「この領域はどこにあるか」を 1 点で示す用途に使う。
    pub fn reference_node(&self) -> Option<NodeId> {
        self.boundary.first().copied()
    }

    /// 領域のレベル Z [mm]（境界座標の Z の平均）。境界が引けなければ `None`。
    pub fn level(&self, model: &Model) -> Option<f64> {
        let coords = self.boundary_coords(model)?;
        if coords.is_empty() {
            return None;
        }
        Some(coords.iter().map(|c| c[2]).sum::<f64>() / coords.len() as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FloorRegionId, NodeId};

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
    fn test_boundary_coords() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 4000.0, 0.0],
            [0.0, 4000.0, 0.0],
        ]);
        let r = FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        );
        let coords = r.boundary_coords(&m).expect("境界座標");
        assert_eq!(coords.len(), 4);
        assert_eq!(r.level(&m), Some(0.0));
        assert_eq!(r.reference_node(), Some(NodeId(0)));
        assert_eq!(r.edge_nodes(0), Some([NodeId(0), NodeId(1)]));
    }

    #[test]
    fn span_is_valid_は規約0_0以上t_i未満t_j以下1_0を判定する() {
        // 代表的な妥当値。
        assert!(span_is_valid([0.0, 1.0]));
        assert!(span_is_valid([0.25, 0.75]));

        // 上下端は 1e-9 の許容つき。span は無次元比のため、1.0 を意図した値が
        // 計算経路で 1.0 + ε になりうる。
        assert!(span_is_valid([-1e-10, 1.0 + 1e-10]));
        assert!(!span_is_valid([-1e-3, 1.0]));
        assert!(!span_is_valid([0.0, 1.0 + 1e-3]));

        // つぶれた区間・逆転した区間は載る範囲が無い。
        assert!(!span_is_valid([0.5, 0.5]));
        assert!(!span_is_valid([0.75, 0.25]));

        // 非有限は弾く。
        assert!(!span_is_valid([f64::NAN, 1.0]));
        assert!(!span_is_valid([0.0, f64::INFINITY]));
    }
}
