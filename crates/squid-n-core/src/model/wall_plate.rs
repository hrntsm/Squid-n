//! 壁版（`WallPlate`）。
//!
//! 壁領域（[`WallRegion`]、[`super::wall`]）は柱・梁が囲む鉛直構面内の閉領域そのもので、
//! 版の仕様は持たない。版の仕様（断面・開口）は本モジュールの `WallPlate` が持つ。
//! 1 つの壁領域は、壁領域内が間柱でさらに細かい壁パネルに分かれていれば複数の
//! `WallPlate` を持ちうる（[`WallRegion::wall_plate_ids`]。E5。床側の `FloorRegion`/
//! [`super::Slab`] と同じ関係）。パラペット・腰壁・垂れ壁・自立壁はどの壁領域からも
//! 参照されない、独立した `WallPlate` として存在する。
//!
//! # 参入レベル（構造壁・n倍法・重量のみ）は型で区別しない
//!
//! 壁が解析にどう参入するか（4 節点要素として剛性・保有水平耐力に算入する「構造壁」、
//! n倍法で偏心率にのみ寄与する「雑壁剛性」、自重のみの「重量のみ」）は、`WallPlate`
//! 自身に列挙型を持たせて利用者に選ばせるのではなく、既存の暗黙規則をそのまま踏襲する
//! （dig Q4=B）。**`section` の有無と、所属する `WallRegion` の種別（囲まれた領域か
//! 取り付き領域か）の組み合わせで、生成ロジック（Step 8・D5）側が決める。**
//!
//! # 躯体の自重は必ず断面参照から求める
//!
//! 板厚と主材料は `section` から解決し、面重量を直接入力する経路は持たない
//! （[`super::Slab`]/[`super::SlabPlate`] と同じ規約）。断面未割当は躯体自重 0 と
//! し、解析前チェックが止める（既定厚で補わない）。
//!
//! # 仕上げ・増打ちは面荷重として持つ
//!
//! コンクリート壁は増打ちを伴うのが常で、仕上げ（タイル・モルタル）も無視できない
//! 重さを持つ。どちらも断面の板厚には含めない（打ち継ぎで一体性が保証されないため
//! 構造厚ではなく、剛性・耐力にも算入しない）ので、[`WallPlate::loads`] が面荷重
//! [N/mm²] として受け持つ。床板の [`super::SlabPlate::loads`] と同型である。

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
    ///
    /// **`extent` が `None` のときは階高いっぱいの壁である**
    /// （[`Model::wall_plate_extent`] が壁の下端から直上の階レベルまでの高さへ
    /// 解決する）。階高は設計中に何度も変わるため、全高の壁を絶対寸法で書くと
    /// 変更に追随せず、上階との間に隙間が残る。数値としては破綻しないので黙って
    /// 残る種類の誤りである。
    ///
    /// `None` を許すのは [`RegionAnchor::FloorRegion`]（自立壁）だけである。
    /// 囲む柱梁があるなら壁の高さは幾何から決まるので、全高の壁は
    /// [`WallPlateShape::Enclosed`]（境界＝壁領域の節点列）で表す。線アンカーで
    /// `None` を許すと、`squid_n_element::wall::misc_wall` が階高分の腰壁せいを取付き先の
    /// 梁 1 本へ丸ごと算入し（反対側の梁が無い扱いになる）、梁の剛性を過大に、
    /// 変形を過小に見る危険側の評価になる。`Model::validate` が弾く。
    Attached {
        anchor: RegionAnchor,
        #[serde(default)]
        extent: Option<[f64; 2]>,
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
    /// 仕上げ・増打ち等の面荷重（[`super::AreaLoad`]）。**版自身の躯体自重は
    /// 含まない**（躯体は `section` から求める）。モジュール doc 参照。
    #[serde(default)]
    pub loads: Vec<AreaLoad>,
    /// 耐震スリット（辺ごとの縁切り）。[`WallSlit`] 参照。
    #[serde(default)]
    pub slit: WallSlit,
}

/// 耐震スリット。壁版の 4 辺それぞれについて、周辺部材との縁を切ったかを持つ。
///
/// スリットは辺ごとに入れるものなので、辺ごとに持つ。三方スリットは柱際 2 辺と
/// 上下いずれか 1 辺、完全スリットは 4 辺すべてが切れた状態として表す。
///
/// **垂れ壁・腰壁とは別の概念である。** 垂れ壁は上の梁からぶら下がる短い壁で、
/// 下端に壁そのものが無い（[`WallPlateShape::Attached`] で表す）。一方、下辺に
/// スリットを入れた壁は構面いっぱいの全高の壁であり、下の梁と接してはいるが縁が
/// 切れている。形が違うので、どちらか一方では表せない。
///
/// 規則は 2 つある。**剛性は切れていない辺の部材にだけ算入し**（袖壁は柱際、
/// 腰壁・垂れ壁は梁際）、**自重は切れていない辺へ伝える**。下辺が切れて上辺が
/// 一体なら、自重は全量が上の梁へ向かう。
///
/// 4 辺すべてが一体でなければ耐震壁として成立しない
/// （`squid_n_element::wall::misc_wall::wall_is_seismic`）。切れた辺があると、負担した
/// 面内せん断を周辺の柱梁へ伝えられないためである。
///
/// 境界が 4 節点の囲まれた壁版でのみ意味を持つ。取り付く壁版は柱・梁と接する
/// 4 辺を持たないため参照しない。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct WallSlit {
    /// 柱際（左右の鉛直辺）。添字は [`WallPlate::column_face_nodes`] が返す
    /// 2 節点に対応する。
    #[serde(default)]
    pub column_face: [bool; 2],
    /// 梁際（0: 下辺、1: 上辺）。
    #[serde(default)]
    pub beam_face: [bool; 2],
}

impl WallSlit {
    /// いずれかの辺が切れているか。耐震壁の成立判定に用いる。
    pub fn any(&self) -> bool {
        self.column_face
            .iter()
            .chain(self.beam_face.iter())
            .any(|&s| s)
    }

    /// 上下の梁際がともに切れているか。
    ///
    /// この壁は自重の伝達先を持たない（柱際は壁の重量を受けない）。入力として
    /// ありえないので、解析前チェックがエラーで止める。
    pub fn both_beam_faces(&self) -> bool {
        self.beam_face[0] && self.beam_face[1]
    }
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

    /// 柱際スリット [`WallSlit::column_face`] の添字に対応する境界節点。
    ///
    /// 柱際の鉛直辺は、境界のうち標高の低い 2 節点（下辺）から立ち上がる。
    /// そこで下辺の 2 節点を**境界の並び順**で返し、`column_face[0]` を
    /// 1 つ目、`[1]` を 2 つ目の柱際に対応させる。
    ///
    /// **添字の対応規則はここ 1 か所に置く。** 剛性算入
    /// （`squid_n_element::wall::misc_wall`）が袖壁を辺ごとに評価するとき、および GUI が
    /// どちらのスリットかを節点番号で示すときに、同じ並びを見る必要があるためである。
    ///
    /// 境界が 4 節点でない壁版・取り付く壁版・節点を引けない壁版は `None`。
    pub fn column_face_nodes(&self, model: &Model) -> Option<[NodeId; 2]> {
        let boundary = self.boundary_nodes()?;
        if boundary.len() != 4 {
            return None;
        }
        let z: Vec<f64> = boundary
            .iter()
            .map(|n| model.nodes.get(n.index()).map(|nd| nd.coord[2]))
            .collect::<Option<_>>()?;
        let mut order: Vec<usize> = (0..4).collect();
        order.sort_by(|&a, &b| z[a].total_cmp(&z[b]));
        // 低い方の 2 点を取り、境界の並び順へ戻す。
        let mut bottom = [order[0], order[1]];
        bottom.sort_unstable();
        Some([boundary[bottom[0]], boundary[bottom[1]]])
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
        self.boundary_coords_with(model, |n| model.nodes.get(n.index()).map(|n| n.coord))
    }

    /// 境界多角形の座標列 [mm] を、節点座標の引き方を差し替えて求める。
    ///
    /// [`WallPlate::boundary_coords`] はモデルの節点座標をそのまま使うが、変形図・
    /// モード形・時刻歴は**変形後の節点座標**で描く必要がある。形状の組み立て方
    /// （取付き線の内挿・鉛直上向きへの立ち上げ）は同じなので、座標の引き方だけを
    /// 差し替えられるようにして実装を 1 つに保つ（床側の
    /// [`super::Slab::boundary_coords_with`] と同じ規約）。
    /// `model` は立ち上がり高さの解決（[`Model::wall_plate_extent`]。`extent` が
    /// `None` の壁は階高いっぱい）にだけ使う。階レベルは変形しないので、変形後の
    /// 座標を渡す呼び出しでも元の `model` を渡してよい。
    pub fn boundary_coords_with(
        &self,
        model: &Model,
        coord_of: impl Fn(NodeId) -> Option<[f64; 3]>,
    ) -> Option<Vec<[f64; 3]>> {
        match &self.shape {
            WallPlateShape::Enclosed { boundary } => {
                boundary.iter().map(|n| coord_of(*n)).collect()
            }
            WallPlateShape::Attached { anchor, .. } => {
                let extent = model.wall_plate_extent(self)?;
                match anchor {
                    RegionAnchor::Line { nodes, span, .. } => {
                        let a = coord_of(nodes[0])?;
                        let b = coord_of(nodes[1])?;
                        Self::extrude_up(a, b, *span, extent)
                    }
                    RegionAnchor::FloorRegion { nodes, .. } => {
                        let a = coord_of(nodes[0])?;
                        let b = coord_of(nodes[1])?;
                        Self::extrude_up(a, b, [0.0, 1.0], extent)
                    }
                    // 壁の取付き先としては使わない（モジュール doc 参照）。
                    RegionAnchor::Point(_) => None,
                }
            }
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

    /// 壁版の面積 [mm²]。囲まれた壁版は [`crate::geom::polygon_area_3d`]（ニューエル）。
    /// 取り付く壁版は底辺長 × ∫|立ち上がり高さ|（[`crate::geom::abs_lerp_integral`]）。
    /// 4 点多角形にすると、両端で高さが符号反転する壁は自己交差（蝶ネクタイ）になり、
    /// Newell の面積が打ち消されて 0 近くになる（自重が黙って消える危険側）。
    /// 座標が引けない場合は 0。
    pub fn area(&self, model: &Model) -> f64 {
        match &self.shape {
            WallPlateShape::Enclosed { .. } => self
                .boundary_coords(model)
                .map(|pts| crate::geom::polygon_area_3d(&pts))
                .unwrap_or(0.0),
            WallPlateShape::Attached { .. } => {
                let Some(pts) = self.boundary_coords(model) else {
                    return 0.0;
                };
                let Some(extent) = model.wall_plate_extent(self) else {
                    return 0.0;
                };
                if pts.len() < 2 {
                    return 0.0;
                }
                let len = crate::geom::vec3::dist(pts[0], pts[1]);
                len * crate::geom::abs_lerp_integral(extent[0], extent[1], 0.0, 1.0)
            }
        }
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

    /// 仕上げ・増打ち等の面荷重強度 [N/mm²]（`loads` の合算）。
    /// **版自身の躯体自重は含まない**（[`super::SlabPlate::finish_intensity`] と同じ規約）。
    pub fn finish_intensity(&self) -> f64 {
        self.loads.iter().map(|l| l.value).sum()
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

    /// 壁版が**壁領域全体を覆う 4 節点の壁版**か。
    ///
    /// 壁エレメント（`squid_n_load::wall_expand`）を生成できるのはこの形の壁版だけ
    /// である。壁エレメントは下辺 2 節点・上辺 2 節点を両端ピンの剛梁で結ぶ
    /// 4 節点 24 自由度モデルなので、
    ///
    /// - 壁領域の境界が 5 節点以上（上下の大梁が中間節点で分割されている等）なら、
    ///   剛梁が四隅しか結ばず、中間節点が壁にめり込む向きへ動けてしまう。
    /// - 壁領域が間柱で複数の壁版へ分割されているなら、壁版ごとに要素を作ると
    ///   1 本の長い壁柱が複数の細い壁柱に割れる。
    ///
    /// いずれも定式化の前提が崩れるため、覆っていない壁版は要素にしない
    /// （＝荷重だけを持つ壁版になる。異常ではない）。
    ///
    /// **判定はここ 1 か所に置く。** 要素生成（`wall_expand`）・自重の分配
    /// （`squid_n_load::wall_plate_load`）・フレーム内雑壁の剛性算入
    /// （`squid_n_element`）が同じ答えを見る必要があるためである。
    pub fn wall_plate_covers_region(&self, plate: &WallPlate) -> bool {
        let Some(boundary) = plate.boundary_nodes() else {
            return false; // 取り付く壁版は壁領域に属さない。
        };
        if boundary.len() != 4 {
            return false;
        }
        self.wall_regions
            .iter()
            .filter(|r| r.wall_plate_ids.contains(&plate.id))
            .any(|r| r.boundary.len() == 4 && r.boundary.iter().all(|n| boundary.contains(n)))
    }

    /// 壁版から壁エレメント（`ElementKind::Wall`）が生成されるか。
    ///
    /// 壁領域全体を覆う 4 節点であること（[`Model::wall_plate_covers_region`]）に
    /// 加えて、**断面が割り当たっていること**を要する。断面が無い壁版は板厚・材料が
    /// 引けず要素を組み立てられないためである（`squid_n_load::wall_expand` は
    /// `skipped_no_section` として数える）。
    ///
    /// `covers_region` との違いはこの 1 点だけだが、両者を取り違えると
    /// 「断面未割当の壁版が、要素でもなく荷重だけの壁版でもない」という
    /// どこからも扱われない状態が生まれる。要素が生成されるかを問うときは
    /// 必ず本メソッドを使うこと。
    ///
    /// **生成の可否を決めるのはここ 1 か所である。** `wall_expand` の生成ゲート・
    /// 3D ビューの壁版描画（要素として描かれる壁版を除くため）・
    /// モデルタブ「壁版」の「解析要素」列が同じ答えを見る。
    pub fn wall_plate_becomes_element(&self, plate: &WallPlate) -> bool {
        self.wall_plate_covers_region(plate) && plate.section.is_some()
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

    /// 取り付く壁版の立ち上がり高さ `[d_i, d_j]` [mm]。
    ///
    /// `extent` が `Some` ならその値をそのまま返す。`None`（＝階高いっぱい）は、
    /// 壁の下端から**直上の階レベル**までの高さへ解決する
    /// （[`WallPlateShape::Attached`] のドキュメント参照）。
    ///
    /// 次のいずれかでは高さが決まらないため `None` を返し、解析前チェックが止める。
    /// 0 を返すと壁が面積 0 になり、自重が黙って消える。
    ///
    /// - 取り付く壁版でない
    /// - `extent` が `None` なのに取付き先が線（[`RegionAnchor::FloorRegion`] 以外）。
    ///   [`Model::validate`] が弾く組み合わせである
    /// - 下端の節点が引けない
    /// - 下端より上に階レベルが無い（最上階の壁）。パラペットのように最上階で上へ
    ///   伸ばす壁は、上端を決める階が無いので絶対寸法で指定する
    pub fn wall_plate_extent(&self, plate: &WallPlate) -> Option<[f64; 2]> {
        let WallPlateShape::Attached { anchor, extent } = &plate.shape else {
            return None;
        };
        if let Some(e) = extent {
            return Some(*e);
        }
        let RegionAnchor::FloorRegion { nodes } = anchor else {
            return None;
        };
        // 壁が載るレベルは下端線分の平均標高とする
        // （[`Model::self_standing_wall_coverage`] と同じ規約。両端の標高が違う壁で
        // 帰属レベルの求め方が 2 通りに割れないようにする）。
        let a = self.nodes.get(nodes[0].index())?.coord[2];
        let b = self.nodes.get(nodes[1].index())?.coord[2];
        let h = self.story_height_above((a + b) / 2.0)?;
        Some([h, h])
    }

    /// レベル `z` [mm] から**直上の階レベル**までの高さ [mm]。
    ///
    /// 階レベルは床レベル列（[`Story::elevation`]）なので、`z` より上にある最も低い
    /// 階レベルとの差がその位置の階高になる。`z` と同じレベルの階は、床そのもので
    /// あって「上」ではないため対象にしない（[`crate::model::DIAPHRAGM_LEVEL_TOL_MM`]
    /// の許容差で同一とみなす）。直上に階が無ければ `None`。
    pub fn story_height_above(&self, z: f64) -> Option<f64> {
        let tol = crate::model::DIAPHRAGM_LEVEL_TOL_MM;
        self.stories
            .iter()
            .map(|s| s.elevation)
            .filter(|e| *e > z + tol)
            .fold(None::<f64>, |acc, e| Some(acc.map_or(e, |a: f64| a.min(e))))
            .map(|e| e - z)
    }

    /// 壁版の自重 [N]。躯体（開口控除後の正味面積 × 板厚 × 主材料の密度 ×
    /// 重力加速度）＋ 仕上げ・増打ち（正味面積 × [`WallPlate::finish_intensity`]）
    /// ＋ 開口部（サッシ等）の重量。
    ///
    /// 仕上げ・増打ちを躯体と同じ**正味面積**に乗じるのは、開口部にはコンクリートも
    /// 仕上げも無いためである。開口周りの見込み・額縁の重さは `opening_weight`
    /// （開口部の重量）で見る場所が別にあり、面積側で二重に見ない。
    ///
    /// 断面または主材料が未割当のとき、躯体分は 0 とする（既定厚で補わない。
    /// 解析前チェックが止める）。仕上げ・増打ちは断面に依らないので、この場合も
    /// そのまま計上する。壁版の高さが決まらないときだけ `None` を返す
    /// （[`Model::wall_plate_extent`]）。開口面積が壁の面積を超える場合は正味面積を 0 とする。
    pub fn wall_plate_self_weight(&self, plate: &WallPlate, model_for_area: &Model) -> Option<f64> {
        if plate.is_attached() && model_for_area.wall_plate_extent(plate).is_none() {
            return None;
        }
        let area = plate.area(model_for_area);
        let net_area = (area - plate.total_opening_area()).max(0.0);
        let structural = match (
            self.wall_plate_thickness(plate),
            self.wall_plate_material(plate),
        ) {
            (Some(t), Some(mat)) => mat.density * t * net_area * crate::units::GRAVITY_MM_S2,
            _ => 0.0,
        };
        let finish = plate.finish_intensity() * net_area;
        Some((structural + finish + plate.opening_weight).max(0.0))
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
    /// 各区間の重量比は、その区間の **∫|立ち上がり高さ|**（`extent` は始端から終端へ
    /// 線形。符号が反転するときは高さ 0 で折れる）が全体に占める割合とする。
    /// 端点の絶対値を結んだ台形にすると、符号反転時に分母が過大になり重量比の
    /// 合計が 1 を下回る（危険側）。開口は総重量のスケールにだけ効き、開口位置は
    /// 見ない（§5.25 の既知の近似と同じ）。
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
        let WallPlateShape::Attached { anchor, .. } = &plate.shape else {
            return None;
        };
        let RegionAnchor::FloorRegion { nodes } = anchor else {
            return None;
        };
        let extent = &self.wall_plate_extent(plate)?;
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
        // 立ち上がり高さは始端から終端へ線形。重量比は ∫|h|（符号反転はゼロ交差で折る）。
        let (h0, h1) = (extent[0], extent[1]);
        if h0 * h1 < 0.0 && h0.abs() > 1e-15 && h1.abs() > 1e-15 {
            cuts.push((-h0 / (h1 - h0)).clamp(0.0, 1.0));
            cuts.sort_by(|x, y| x.total_cmp(y));
            cuts.dedup_by(|x, y| (*x - *y).abs() <= 1e-12);
        }
        let total_area = crate::geom::abs_lerp_integral(h0, h1, 0.0, 1.0);

        let mut cov = SelfStandingWallCoverage::default();
        for w in cuts.windows(2) {
            let (t0, t1) = (w[0], w[1]);
            if t1 - t0 <= 1e-12 {
                continue;
            }
            let frac = if total_area > 0.0 {
                crate::geom::abs_lerp_integral(h0, h1, t0, t1) / total_area
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
    use crate::ids::{FloorRegionId, MaterialId, NodeId, SectionId, SlabId, WallPlateId};

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

    /// `boundary_coords_with` は渡した座標をそのまま使う。変形図・モード形が
    /// 変形後の節点座標で壁版を描くための入口であり、モデルの元座標へ落ちない
    /// ことを固定する（落ちると壁版だけが変形前の位置に取り残される）。
    #[test]
    fn test_boundary_coords_with_uses_supplied_coords() {
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
            loads: vec![],
            slit: WallSlit::default(),
        };
        // 全節点を +100 mm ずらした「変形後」の座標を渡す。
        let moved: Vec<[f64; 3]> = m
            .nodes
            .iter()
            .map(|n| [n.coord[0] + 100.0, n.coord[1], n.coord[2]])
            .collect();
        let coords = p
            .boundary_coords_with(&m, |n| moved.get(n.index()).copied())
            .expect("境界座標");
        assert_eq!(coords[0], [100.0, 0.0, 0.0]);
        assert_eq!(coords[1], [4100.0, 0.0, 0.0]);

        // 取り付く壁版も取付き先の節点座標に追従する。
        let attached = WallPlate {
            id: WallPlateId(1),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: Default::default(),
                },
                extent: Some([900.0, 900.0]),
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: WallSlit::default(),
        };
        let coords = attached
            .boundary_coords_with(&m, |n| moved.get(n.index()).copied())
            .expect("境界座標");
        assert_eq!(coords[0], [100.0, 0.0, 0.0]);
        assert_eq!(coords[2], [4100.0, 0.0, 900.0]);
    }

    /// `wall_plate_becomes_element` は断面の有無まで見る。壁領域を覆っていても
    /// 断面が無ければ要素にならないため、3D ビューは壁版として描く必要がある。
    #[test]
    fn test_becomes_element_requires_section() {
        let mut m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        let boundary = vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)];
        m.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: boundary.clone(),
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: WallSlit::default(),
        });
        m.wall_regions.push(crate::model::WallRegion {
            id: crate::ids::WallRegionId(0),
            name: String::new(),
            boundary,
            wall_plate_ids: vec![WallPlateId(0)],
            posts: Vec::new(),
        });

        // 覆ってはいるが断面が無い。
        assert!(m.wall_plate_covers_region(&m.wall_plates[0]));
        assert!(!m.wall_plate_becomes_element(&m.wall_plates[0]));

        m.wall_plates[0].section = Some(SectionId(0));
        assert!(m.wall_plate_becomes_element(&m.wall_plates[0]));
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
            loads: vec![],
            slit: WallSlit::default(),
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
            loads: vec![],
            slit: WallSlit::default(),
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
            extent: Some([900.0, 900.0]),
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
                extent: Some([900.0, 900.0]),
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: WallSlit::default(),
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

    /// 両端で立ち上がり高さの符号が反転する取り付く壁版は、蝶ネクタイの Newell
    /// 面積（≈0）ではなく、2 つの三角形の和を自重面積にする。
    #[test]
    fn attached_area_uses_abs_height_when_extent_sign_reverses() {
        let m = model_with_nodes(&[[0.0, 0.0, 3000.0], [4000.0, 0.0, 3000.0]]);
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: Some([2000.0, -2000.0]),
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: WallSlit::default(),
        };
        let expected = 4000.0 * 2000.0 * 0.5;
        assert!(
            (p.area(&m) - expected).abs() < 1e-6,
            "area={} expected={expected}",
            p.area(&m)
        );
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
                extent: Some([2500.0, 2500.0]),
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: WallSlit::default(),
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
                extent: Some([900.0, 900.0]),
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: WallSlit::default(),
        };
        assert_eq!(p.boundary_coords(&m), None);
        assert_eq!(p.area(&m), 0.0);
    }

    /// 断面が無い壁版の躯体自重は 0 になる（既定厚では補わない）。仕上げ・増打ちは
    /// 断面に依らないので、断面が無くてもそのまま計上する。ここで丸ごと `None` を
    /// 返すと、入力された仕上げの重さが黙って落ちる。
    #[test]
    fn test_self_weight_without_section_counts_only_finish() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]);
        let mut p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: WallSlit::default(),
        };
        assert_eq!(m.wall_plate_self_weight(&p, &m), Some(0.0));

        p.loads.push(AreaLoad {
            kind: "増打ち".into(),
            value: 1.0e-3,
        });
        let w = m.wall_plate_self_weight(&p, &m).expect("自重が求まる");
        assert!((w - 1.0e-3 * 4000.0 * 3000.0).abs() < 1e-6, "{w}");
    }

    /// 仕上げ・増打ちも躯体と同じ正味面積（開口控除後）に乗る。
    #[test]
    fn test_finish_load_deducts_opening_area() {
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
            opening_area: 2.0e6,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![AreaLoad {
                kind: "仕上げ".into(),
                value: 1.0e-3,
            }],
            slit: WallSlit::default(),
        };
        let w = m.wall_plate_self_weight(&p, &m).expect("自重が求まる");
        assert!((w - 1.0e-3 * (4000.0 * 3000.0 - 2.0e6)).abs() < 1e-6, "{w}");
    }

    /// 高さを階高いっぱいとした自立壁は、直上の階レベルまでの高さへ解決する。
    /// 直上に階が無ければ高さが決まらず `None`（解析前チェックが止める）。
    #[test]
    fn test_story_height_extent_resolves_from_stories() {
        use crate::ids::StoryId;
        let mut m = model_with_nodes(&[[0.0, 0.0, 0.0], [4000.0, 0.0, 0.0]]);
        for (i, z) in [0.0_f64, 3200.0].into_iter().enumerate() {
            m.stories.push(Story {
                id: StoryId(i as u32),
                name: format!("{i}F"),
                elevation: z,
                node_ids: vec![],
                seismic_weight: None,
                weight_override: None,
                structure: Default::default(),
                level_kind: Default::default(),
            });
        }
        let p = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    nodes: [NodeId(0), NodeId(1)],
                },
                extent: None,
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: WallSlit::default(),
        };
        assert_eq!(m.wall_plate_extent(&p), Some([3200.0, 3200.0]));

        // 最上階の床に立つ壁は上端を決める階が無い。
        m.stories.truncate(1);
        assert_eq!(m.wall_plate_extent(&p), None);
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
            loads: vec![],
            slit: WallSlit::default(),
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
            loads: vec![],
            slit: WallSlit::default(),
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
            loads: vec![],
            slit: WallSlit::default(),
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
                extent: Some(extent),
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            loads: vec![],
            slit: WallSlit::default(),
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

    /// 立ち上がり高さの符号が両端で反転する自立壁は、ゼロ交差で折れた ∫|h| で
    /// 配る。端点絶対値の台形を分母にすると比率の合計が 0.5 になり重量が消える。
    #[test]
    fn coverage_sign_reversal_preserves_ratio_sum() {
        let mut m = model_two_regions();
        // X=2000..6000。X=4000 の境界かつ高さ 0。各側は同じ三角形。
        let p = push_self_standing(
            &mut m,
            [2000.0, 1000.0, 3000.0],
            [6000.0, 1000.0, 3000.0],
            [2000.0, -2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(!cov.has_uncovered(), "{cov:?}");
        let sum: f64 = cov.per_region.iter().map(|(_, f)| f).sum::<f64>() + cov.uncovered;
        assert!(
            (sum - 1.0).abs() < 1e-9,
            "符号反転でも比率の合計は 1: {cov:?}"
        );
        for (_, f) in &cov.per_region {
            assert!((f - 0.5).abs() < 1e-9, "{cov:?}");
        }
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
            loads: vec![],
            slit: WallSlit::default(),
        };
        assert!(m.self_standing_wall_coverage(&p).is_none());
    }

    /// L 形（非凸）の床領域 1 つ。欠けているのは X=4000..8000, Y=4000..8000。
    fn model_l_region() -> Model {
        let mut m = model_with_nodes(&[
            [0.0, 0.0, 3000.0],
            [8000.0, 0.0, 3000.0],
            [8000.0, 4000.0, 3000.0],
            [4000.0, 4000.0, 3000.0],
            [4000.0, 8000.0, 3000.0],
            [0.0, 8000.0, 3000.0],
        ]);
        let b = vec![
            NodeId(0),
            NodeId(1),
            NodeId(2),
            NodeId(3),
            NodeId(4),
            NodeId(5),
        ];
        let mut r = FloorRegion::new(FloorRegionId(0), b.clone());
        r.slab_ids.push(SlabId(0));
        m.floor_regions.push(r);
        m.slabs.push(Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed { boundary: b },
            plate: SlabPlate::default(),
        });
        m
    }

    /// L 形の底辺に収まる自立壁は、非凸でも全量をその床領域が受け持つ。
    #[test]
    fn coverage_l_shape_interior_is_fully_covered() {
        let mut m = model_l_region();
        let p = push_self_standing(
            &mut m,
            [500.0, 1000.0, 3000.0],
            [7500.0, 1000.0, 3000.0],
            [2000.0, 2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(!cov.has_uncovered(), "{cov:?}");
        assert_eq!(cov.per_region.len(), 1);
        assert_eq!(cov.per_region[0].0, FloorRegionId(0));
        assert!((cov.per_region[0].1 - 1.0).abs() < 1e-9, "{cov:?}");
    }

    /// L 形の欠けへはみ出す壁は、欠け側を `uncovered` にする（中点内包で帰属）。
    #[test]
    fn coverage_l_shape_notch_is_uncovered() {
        let mut m = model_l_region();
        // Y=1000..7000。Y=4000 より上が欠け（X=6000）。
        let p = push_self_standing(
            &mut m,
            [6000.0, 1000.0, 3000.0],
            [6000.0, 7000.0, 3000.0],
            [2000.0, 2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(cov.has_uncovered(), "{cov:?}");
        assert!((cov.uncovered - 0.5).abs() < 1e-9, "{cov:?}");
        let covered: f64 = cov.per_region.iter().map(|(_, f)| f).sum();
        assert!((covered - 0.5).abs() < 1e-9, "{cov:?}");
    }

    /// 床領域の辺上に乗った壁は、厳密内包判定により覆われていない。
    #[test]
    fn coverage_on_region_boundary_is_uncovered() {
        let mut m = model_two_regions();
        // 領域 0 の南辺（Y=0, X=500..3500）にぴったり載せる。
        let p = push_self_standing(
            &mut m,
            [500.0, 0.0, 3000.0],
            [3500.0, 0.0, 3000.0],
            [2000.0, 2000.0],
        );
        let cov = m.self_standing_wall_coverage(&p).expect("自立壁");
        assert!(
            cov.has_uncovered(),
            "辺上は内包しない（大梁真上は線アンカーへ）: {cov:?}"
        );
        assert!((cov.uncovered - 1.0).abs() < 1e-9, "{cov:?}");
    }
}
