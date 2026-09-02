//! 壁展開（壁版から解析要素を組み立てる。D4・D5）。
//!
//! 入力の正は「壁領域」（[`squid_n_core::model::WallRegion`]）と「壁版」
//! （[`squid_n_core::model::WallPlate`]）であり、`Model.elements` には壁の
//! 解析要素（`ElementKind::Wall`）を一切持たせない（D5）。ソルバ・断面力表示・
//! 保有水平耐力など、壁の解析要素を必要とする消費者は、それぞれ
//! [`expand_wall_elements`] を呼んで**壁展開モデル**（生成要素を追加した
//! `Model` の一時的な複製）を得る。ST-Bridge の書き出しは入力の正である壁版
//! から直接行う（展開しない。4 節点でない壁版も往復させるため）。
//!
//! # 壁展開モデル・壁展開インデックス（用語）
//!
//! - **壁展開モデル**: `WallRegion`/`WallPlate` から生成した `ElementKind::Wall`
//!   要素を追加した `Model`。入力の正である呼び出し元のモデルそのものには
//!   書き戻さない一時的な派生値。
//! - **壁展開インデックス**（[`WallExpansionIndex`]）: 壁展開モデル中の生成要素
//!   `ElemId` から、由来する `WallPlateId` を逆引きする対応表。
//!
//! # キャッシュしない（都度計算）
//!
//! 生成は決定的（[`squid_n_core::model::Model::wall_regions`] の並び順、各領域内は
//! `wall_plate_ids` の並び順）であるため、同じモデルに対して複数回呼んでも同じ
//! `ElemId` 割当になる。展開処理自体は `WallRegion`/`WallPlate` に対する軽い
//! 走査であり（境界検出という重い幾何走査は `rebuild_wall_regions` が既に済ませて
//! `model.wall_regions` へキャッシュ済み）、`App` 側に専用のキャッシュフィールドは
//! 持たせない（呼び出し側で都度呼ぶ）。
//!
//! # `wall_attrs` の合成（移行期の内部表現）
//!
//! 生成した壁要素ごとに、由来する `WallPlate` の開口・スリットを写した
//! `WallAttr` を壁展開モデルの `wall_attrs` へ合成して積む。既存の壁消費者
//! （自重算定 [`crate::story_gen::self_weight_calc`]、開口低減・剛性
//! （`squid_n_element::factory::wall_opening`）、偏心率の雑壁剛性、数量拾い等、
//! 現時点で 9 箇所ほど）はいずれも `model.wall_attrs.iter().find(|a| a.elem ==
//! data.id)` という同じ形の参照を持つ。壁展開モデルにこの合成 `wall_attrs` を
//! 持たせることで、それらの消費者は**無改修のまま**壁展開モデルを受け取るだけで
//! 正しく動く（dig Q5=A の狙いそのもの: 型移行で計算根拠を変えない）。
//!
//! これは `WallAttr` の恒久的な存続を意味しない。`Model.wall_attrs` はなお
//! `WallPlate` へ吸収される予定の型（D3・E5）であり、ここでの合成は
//! **壁展開モデル内部だけの一時的な表現**である（入力の正である呼び出し元の
//! `Model` には一切書き戻さない）。個々の消費者を `WallPlate` 直接参照へ
//! 書き換える作業（真の廃止）は別途の残課題として残っている。
//!
//! # 二重展開しないこと（重要な不変条件）
//!
//! [`expand_wall_elements`] は**壁展開モデルに対して呼んではならない**。
//! 壁展開モデルは `model.wall_regions`/`model.wall_plates` を入力モデルから
//! そのまま複製して持つため、既に壁要素が展開された壁展開モデルへ再度
//! 本関数を適用すると、同じ壁版から**もう一組**の壁要素・合成 `WallAttr` が
//! 追加され、壁の自重・剛性が実質的に二重計上される（生成 `ElemId` は
//! 既存要素の最大値より後ろへ振るため ID 衝突では検出できない）。
//!
//! 現状の呼び出し経路（自重算定の内部展開、ソルバ・GUI診断の
//! 呼び出し元展開）は互いに素（どちらも「入力の正である `Model`」だけを受け取り、
//! 他の展開済みモデルを再度渡すことはない）であることを確認済みだが、
//! 呼び出し側の実装ミスに備えて `expand_wall_elements` 自身も
//! `debug_assert!` で「入力モデルに壁要素が最初から含まれていないこと」を
//! 検査する（release ビルドではコストを持たない）。
//!
//! # 生成対象・非対象
//!
//! **壁領域全体を覆う 1 枚の壁版だけが壁エレメントになる。** 具体的には、
//! `WallPlateShape::Enclosed` の壁版の境界節点集合が、所属する壁領域の境界節点
//! 集合と一致し、その節点数がちょうど 4 のときだけ `ElementKind::Wall` を生成する
//! （`section` 割当も要る）。生成後、その要素が実際に耐震壁として成立するか
//! （面内せん断を負担する構造壁か、フレーム内雑壁として剛性のみ寄与するか）は、
//! 既存の `wall_is_seismic`/`wall_is_framed`（`squid_n_element::wall::misc_wall`）が
//! 判定するため、生成側では区別しない。
//!
//! この条件は壁エレメントの定式化そのものから来ている。壁エレメントは下辺 2 節点・
//! 上辺 2 節点を両端ピンの剛梁で結ぶ 4 節点 24 自由度モデルであり、
//!
//! - 壁領域の境界が 5 節点以上（上下の大梁が中間節点で分割されている等）の場合、
//!   剛梁は四隅しか結ばないため、中間節点が壁にめり込む向きへ自由に動ける。
//!   壁があるはずの位置で大梁が曲がれるモデルになり、前提が崩れる。
//! - 壁領域が間柱で複数の壁版に分割されている場合、壁版ごとに要素を作ると
//!   1 本の長い壁柱が複数の細い壁柱に割れ、面内剛性が実態と食い違う。
//!
//! 要素にならなかった壁版（間柱で分割された壁版・腰壁・垂れ壁・パラペット・
//! 5 節点以上の壁領域内の壁版）は**荷重だけを持つ壁版**であり、これは正常な状態
//! である。自重は `crate::wall_plate_load` が辺へ分配し、剛性は
//! `squid_n_element` がフレーム内雑壁（袖壁・腰壁・垂壁）として周辺の柱梁の
//! 断面性能へ算入する。したがって診断で警告を出すことはしない。
//!
//! `section` 未割当の壁版だけは生成対象から外れる（自重すら求まらないため。
//! `WallPlate` モジュール doc 参照）。これは解析前チェックが警告する。

use squid_n_core::ids::{ElemId, WallPlateId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, WallAttr, WallPlateShape,
};
use std::collections::HashMap;

/// 壁展開モデル中の生成要素 `ElemId` → 由来する `WallPlateId` の対応表。
#[derive(Clone, Debug, Default)]
pub struct WallExpansionIndex(HashMap<ElemId, WallPlateId>);

impl WallExpansionIndex {
    /// 生成要素の由来元の壁版 ID。生成物でない `ElemId`（入力の柱・梁等）は `None`。
    pub fn plate_of(&self, elem: ElemId) -> Option<WallPlateId> {
        self.0.get(&elem).copied()
    }

    /// 登録されている生成要素の `ElemId` を走査する（順序は不定）。
    /// 個々の生成要素を由来の壁版から独立に特定したいテスト・UI 向け。
    pub fn generated_elem_ids(&self) -> impl Iterator<Item = ElemId> + '_ {
        self.0.keys().copied()
    }

    /// 登録件数（＝実際に生成した壁要素の数）。
    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// 生成せず読み飛ばした壁版の件数報告（診断・警告向け）。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WallExpansionReport {
    /// 生成した壁要素数。
    pub generated: usize,
    /// 壁領域全体を覆っていないため生成しなかった壁版数（間柱で分割された壁版・
    /// 腰壁・垂れ壁、および壁領域の境界が 4 節点でない場合）。**異常ではない。**
    /// 荷重だけを持つ壁版として自重の分配・雑壁の剛性算入の対象になる。
    pub skipped_not_covering: usize,
    /// 断面未割当のため生成しなかった壁版数。
    pub skipped_no_section: usize,
}

/// 壁展開で 1 件でも要素が生成され得るか（安価な事前判定）。
///
/// [`expand_wall_elements`] は `model.clone()`（要素数に比例するコスト）を
/// 無条件に行うため、壁を持たないモデル（実 ST-Bridge フィクスチャは現状すべて
/// 該当する）に対して都度呼ぶ消費者（解析の入口ではなく、`ensure_preparation`
/// のように呼び出し頻度が高い箇所）は、呼ぶ前にこの判定で複製を避けるとよい。
/// `model.wall_plates` が空でなくても、どの `WallRegion` にも帰属していない
/// （`rebuild_wall_regions` で未割当と報告された）壁版からは要素を生成しない
/// ため、判定は `wall_regions.wall_plate_ids` 側で行う。
pub fn model_has_wall_plates_to_expand(model: &Model) -> bool {
    model
        .wall_regions
        .iter()
        .any(|r| !r.wall_plate_ids.is_empty())
}

/// `model` から壁展開モデル（壁の解析要素を追加した `Model` の複製）を組み立てる。
///
/// `model` 自体は変更しない。返す `Model` は一時的な派生値であり、呼び出し元が
/// 保存・シリアライズしてはならない（D5。壁の解析要素はモデルに残さない）。
/// 呼び出し元がすでに所有する `Model` を渡せるときは
/// [`expand_wall_elements_owned`] を使い、ここでの複製を避ける。
pub fn expand_wall_elements(model: &Model) -> (Model, WallExpansionIndex, WallExpansionReport) {
    expand_wall_elements_owned(model.clone())
}

/// [`expand_wall_elements`] の所有権版。受け取った `Model` へ生成要素を追記して返す。
///
/// 解析入口のように呼び出し元がすでに `Model` の複製を持っているとき、
/// もう一度 `clone` しないための入口。
pub fn expand_wall_elements_owned(
    mut expanded: Model,
) -> (Model, WallExpansionIndex, WallExpansionReport) {
    debug_assert!(
        expanded
            .elements
            .iter()
            .all(|e| e.kind != ElementKind::Wall),
        "expand_wall_elements への入力モデルに既に ElementKind::Wall が含まれている。\
         壁展開モデルを再度展開しようとしていないか確認すること（モジュール doc の\
         「二重展開しないこと」参照。壁の自重・剛性が二重計上される）。"
    );

    let mut index = WallExpansionIndex::default();
    let mut report = WallExpansionReport::default();
    let mut next_id = expanded
        .elements
        .iter()
        .map(|e| e.id.0)
        .max()
        .map_or(0, |m| m + 1);

    // 生成対象を先に集めてから `elements` / `wall_attrs` へ追記する
    // （領域・壁版の走査中にモデルを可変借用できないため）。
    let mut jobs: Vec<(WallPlateId, ElementData, WallAttr)> = Vec::new();
    for region in &expanded.wall_regions {
        for &plate_id in &region.wall_plate_ids {
            let Some(plate) = expanded.wall_plate(plate_id) else {
                continue;
            };
            let WallPlateShape::Enclosed { boundary } = &plate.shape else {
                // 取り付く壁版（パラペット等）は耐震壁要素の対象外。
                continue;
            };
            // 生成の可否は `Model::wall_plate_becomes_element` が単独で決める。
            // 内訳カウンタのためにここで理由も分けるが、**可否そのものを
            // ここで組み立て直さない**（同じ問いを 2 か所で実装すると、
            // 3D ビューの壁版描画・「解析要素」列と食い違う）。
            if !expanded.wall_plate_becomes_element(plate) {
                if expanded.wall_plate_covers_region(plate) {
                    report.skipped_no_section += 1;
                } else {
                    report.skipped_not_covering += 1;
                }
                continue;
            }
            let id = ElemId(next_id);
            next_id += 1;
            jobs.push((
                plate_id,
                ElementData {
                    id,
                    kind: ElementKind::Wall,
                    nodes: boundary.iter().copied().collect(),
                    section: plate.section,
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
                WallAttr {
                    elem: id,
                    opening_area: plate.opening_area,
                    opening_weight: plate.opening_weight,
                    slit: plate.slit,
                    openings: plate.openings.clone(),
                    finish_intensity: plate.finish_intensity(),
                },
            ));
            report.generated += 1;
        }
    }
    for (plate_id, elem, attr) in jobs {
        index.0.insert(elem.id, plate_id);
        expanded.elements.push(elem);
        // 既存の壁消費者（自重算定・開口低減剛性・偏心率の雑壁剛性・数量拾い等）は
        // いずれも `model.wall_attrs` を `elem` で引く同じ形の参照を持つ。
        // 壁展開モデルだけに、由来する壁版の開口・スリットを写した合成
        // `WallAttr` を積み、それらの消費者を無改修のまま動かす
        // （モジュール doc「wall_attrs の合成」参照）。
        expanded.wall_attrs.push(attr);
    }

    (expanded, index, report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{NodeId, SectionId, WallRegionId};
    use squid_n_core::model::{Node, Section, WallPlate, WallRegion};

    fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn section(id: u32) -> Section {
        Section {
            id: SectionId(id),
            name: format!("Wall t150 #{id}"),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
            material: None,
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }

    fn quad_plate(id: u32, section: Option<SectionId>) -> WallPlate {
        WallPlate {
            id: squid_n_core::ids::WallPlateId(id),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: Default::default(),
        }
    }

    fn base_model() -> Model {
        let mut m = Model::default();
        for (id, (x, y, z)) in [
            (0, (0.0, 0.0, 0.0)),
            (1, (4000.0, 0.0, 0.0)),
            (2, (4000.0, 0.0, 3000.0)),
            (3, (0.0, 0.0, 3000.0)),
        ] {
            m.nodes.push(node(id, x, y, z));
        }
        m.sections.push(section(0));
        m
    }

    /// 壁版の仕上げ・増打ちの面荷重を、合成する `WallAttr` へ写す。
    ///
    /// 壁エレメントの自重は要素経由（`story_gen::self_weight_calc`）で算定するため、
    /// ここで写し忘れると**壁エレメントになる壁版だけ**仕上げ・増打ちの重さが
    /// 黙って落ちる。要素にならない壁版は `Model::wall_plate_self_weight` を直接
    /// 使うので落ちず、差が出るのは要素になる壁版だけという気づきにくい形になる。
    #[test]
    fn test_copies_finish_intensity_into_synthesized_wall_attr() {
        use squid_n_core::model::AreaLoad;

        let mut m = base_model();
        let mut plate = quad_plate(0, Some(SectionId(0)));
        plate.loads = vec![
            AreaLoad {
                kind: "増打ち".into(),
                value: 4.0e-4,
            },
            AreaLoad {
                kind: "仕上げ".into(),
                value: 3.0e-4,
            },
        ];
        m.wall_plates.push(plate);
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(0)],
            posts: Vec::new(),
        });

        let (expanded, _index, report) = expand_wall_elements(&m);
        assert_eq!(report.generated, 1);
        let attr = expanded
            .wall_attrs
            .first()
            .expect("合成された WallAttr が 1 件");
        // 複数件の面荷重は合算して写す（`WallPlate::finish_intensity` と同じ）。
        assert!(
            (attr.finish_intensity - 7.0e-4).abs() < 1e-12,
            "{}",
            attr.finish_intensity
        );
    }

    #[test]
    fn test_generates_wall_element_for_quad_plate_with_section() {
        let mut m = base_model();
        m.wall_plates.push(quad_plate(0, Some(SectionId(0))));
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(0)],
            posts: Vec::new(),
        });

        let (expanded, index, report) = expand_wall_elements(&m);

        assert_eq!(report.generated, 1);
        assert_eq!(report.skipped_not_covering, 0);
        assert_eq!(report.skipped_no_section, 0);
        assert_eq!(expanded.elements.len(), 1);
        let elem = &expanded.elements[0];
        assert_eq!(elem.kind, ElementKind::Wall);
        assert_eq!(elem.nodes.len(), 4);
        assert_eq!(
            index.plate_of(elem.id),
            Some(squid_n_core::ids::WallPlateId(0))
        );
        // 既存の壁消費者（自重算定等）が無改修で動くよう、合成 `WallAttr` を積む
        // （モジュール doc「wall_attrs の合成」参照）。
        assert_eq!(expanded.wall_attrs.len(), 1);
        assert_eq!(expanded.wall_attrs[0].elem, elem.id);
        // 入力の正（`m`）は変更されない。
        assert!(m.elements.is_empty());
        assert!(m.wall_attrs.is_empty());
    }

    /// 壁版の開口・スリットは、合成 `WallAttr` へそのまま写る。
    #[test]
    fn test_synthesized_wall_attr_mirrors_plate_openings() {
        let mut m = base_model();
        let mut plate = quad_plate(0, Some(SectionId(0)));
        plate.opening_area = 1_000_000.0;
        plate.opening_weight = 5000.0;
        plate.slit = squid_n_core::model::WallSlit {
            column_face: [true, false],
            beam_face: [false, true],
        };
        m.wall_plates.push(plate);
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(0)],
            posts: Vec::new(),
        });

        let (expanded, _index, _report) = expand_wall_elements(&m);
        let attr = &expanded.wall_attrs[0];
        assert_eq!(attr.opening_area, 1_000_000.0);
        assert_eq!(attr.opening_weight, 5000.0);
        assert_eq!(attr.slit.column_face, [true, false]);
        assert_eq!(attr.slit.beam_face, [false, true]);
    }

    /// `Model::wall_plate_becomes_element` の答えと、実際に要素が生成されたかが
    /// 一致することを固定する。
    ///
    /// 3D ビューの壁版描画とモデルタブ「壁版」の「解析要素」列は、展開を走らせずに
    /// このメソッドだけで「要素になるか」を決める。判定が実際の生成と乖離すると、
    /// 要素としても壁版としても描かれない壁版が生まれる（＝入力したのに図に出ない）。
    #[test]
    fn test_becomes_element_agrees_with_generated_elements() {
        let mut m = base_model();
        // 4 節点・断面あり（要素になる）。
        m.wall_plates.push(quad_plate(0, Some(SectionId(0))));
        // 4 節点だが断面なし（要素にならない）。
        m.wall_plates.push(quad_plate(1, None));
        // 壁領域を覆わない（境界の一部しか共有しない）壁版。
        let mut partial = quad_plate(2, Some(SectionId(0)));
        partial.shape = WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2)],
        };
        m.wall_plates.push(partial);
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![
                squid_n_core::ids::WallPlateId(0),
                squid_n_core::ids::WallPlateId(1),
                squid_n_core::ids::WallPlateId(2),
            ],
            posts: Vec::new(),
        });

        let (_expanded, index, _report) = expand_wall_elements(&m);
        let generated: std::collections::BTreeSet<_> = index.0.values().copied().collect();
        for plate in &m.wall_plates {
            assert_eq!(
                m.wall_plate_becomes_element(plate),
                generated.contains(&plate.id),
                "壁版 {} の判定と生成結果が食い違う",
                plate.id.0
            );
        }
        // 3 枚のうち要素になるのは 1 枚だけ（テスト自体が退化していないことの確認）。
        assert_eq!(generated.len(), 1);
    }

    /// 壁展開モデルへ再度 `expand_wall_elements` を適用しようとすると、
    /// 壁の自重・剛性が二重計上されるため `debug_assert!` で止める
    /// （モジュール doc「二重展開しないこと」参照）。
    #[test]
    #[should_panic(expected = "壁展開モデルを再度展開しようとしていないか")]
    #[cfg(debug_assertions)]
    fn test_re_expanding_an_already_expanded_model_panics_in_debug() {
        let mut m = base_model();
        m.wall_plates.push(quad_plate(0, Some(SectionId(0))));
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(0)],
            posts: Vec::new(),
        });
        let (expanded, _index, _report) = expand_wall_elements(&m);
        let _ = expand_wall_elements(&expanded);
    }

    #[test]
    fn test_skips_plate_without_section() {
        let mut m = base_model();
        m.wall_plates.push(quad_plate(0, None));
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(0)],
            posts: Vec::new(),
        });

        let (expanded, index, report) = expand_wall_elements(&m);
        assert_eq!(report.generated, 0);
        assert_eq!(report.skipped_no_section, 1);
        assert!(expanded.elements.is_empty());
        assert!(index.is_empty());
    }

    /// 壁領域の境界が 5 節点（大梁が中間節点で分割された等）なら、壁版が同じ
    /// 輪郭でも壁エレメントは作れない。剛梁が四隅しか結ばず、中間節点が壁に
    /// めり込む向きへ動けてしまうため、定式化の前提が崩れる。
    #[test]
    fn test_skips_non_quad_boundary() {
        let mut m = base_model();
        m.nodes.push(node(4, 2000.0, 0.0, 3000.0));
        let mut plate = quad_plate(0, Some(SectionId(0)));
        plate.shape = WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(4), NodeId(3)],
        };
        m.wall_plates.push(plate);
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(4), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(0)],
            posts: Vec::new(),
        });

        let (expanded, _index, report) = expand_wall_elements(&m);
        assert_eq!(report.generated, 0);
        assert_eq!(report.skipped_not_covering, 1);
        assert!(expanded.elements.is_empty());
    }

    #[test]
    fn test_attached_plate_is_not_generated() {
        let mut m = base_model();
        m.wall_plates.push(WallPlate {
            id: squid_n_core::ids::WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: squid_n_core::model::RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: squid_n_core::model::LoadTransfer::Anchor,
                },
                extent: Some([900.0, 900.0]),
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: Default::default(),
        });
        // 取り付く壁版はどの壁領域からも参照されない（wall_plate_ids に入らない）ため、
        // この壁版自体を走査しても generated は増えない。
        let (expanded, _index, report) = expand_wall_elements(&m);
        assert_eq!(report.generated, 0);
        assert!(expanded.elements.is_empty());
    }

    /// 生成した `ElemId` は既存要素の最大 ID より後ろへ付与し、衝突しない。
    #[test]
    fn test_generated_elem_id_does_not_collide_with_existing_elements() {
        use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis};
        let mut m = base_model();
        m.elements.push(ElementData {
            id: ElemId(5),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        m.wall_plates.push(quad_plate(0, Some(SectionId(0))));
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(0)],
            posts: Vec::new(),
        });

        let (expanded, _index, _report) = expand_wall_elements(&m);
        assert_eq!(expanded.elements.len(), 2);
        assert!(expanded
            .elements
            .iter()
            .all(|e| e.id != ElemId(5) || e.kind == ElementKind::Beam));
        let wall_elem = expanded
            .elements
            .iter()
            .find(|e| e.kind == ElementKind::Wall)
            .expect("壁要素が生成される");
        assert_eq!(wall_elem.id, ElemId(6), "既存の最大IDより後ろへ付与する");
    }

    /// 展開は決定的で、同じモデルへ複数回呼んでも同じ `ElemId` 割当になる（D5）。
    #[test]
    fn test_expansion_is_deterministic_across_calls() {
        let mut m = base_model();
        m.wall_plates.push(quad_plate(0, Some(SectionId(0))));
        m.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(0)],
            posts: Vec::new(),
        });

        let (e1, i1, _) = expand_wall_elements(&m);
        let (e2, i2, _) = expand_wall_elements(&m);
        assert_eq!(e1.elements[0].id, e2.elements[0].id);
        assert_eq!(
            i1.plate_of(e1.elements[0].id),
            i2.plate_of(e2.elements[0].id)
        );
    }
}
