//! 断面の型。
//!
//! - [`rect_shear_area`] — 矩形断面の有効せん断断面積。
//! - [`Section`] — 断面（断面性能・形状定義）。

use super::*;

pub fn rect_shear_area(area: f64) -> f64 {
    area * 5.0 / 6.0
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Section {
    pub id: SectionId,
    /// 断面符号（例 `C1`・`GY2`）。単独では断面を一意に定めない。
    pub name: String,
    /// 階（例 `1`・`PH1`）。ST-Bridge の `floor` 属性をそのまま保持する自由文字列で、
    /// [`Story`](crate::model::Story) への参照ではない（断面の階名と階の名称は
    /// 一致しないことがあり、対応する階が存在しない階名もあるため）。
    ///
    /// 断面の同一性は **符号＋階** で決まる。同じ符号でも階が違えば別断面として扱い、
    /// 逆に符号＋階が同じ断面がモデル内に複数存在してはならない。階を持たない断面
    /// （アプリ内で作成した断面・階の指定がない ST-Bridge 断面）は `None` とし、
    /// このときは符号だけが同一性キーになる。
    #[serde(default)]
    pub floor: Option<String>,
    pub area: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
    #[serde(default)]
    pub depth: f64,
    #[serde(default)]
    pub width: f64,
    #[serde(default)]
    pub as_y: f64,
    #[serde(default)]
    pub as_z: f64,
    #[serde(default)]
    pub panel_thickness: Option<f64>,
    #[serde(default)]
    pub thickness: Option<f64>,
    /// パラメトリック形状定義（UI設計 §4.2: Section は SectionShape の派生）。
    /// 形状から生成されなかった断面（カタログ数値直入力・ST-Bridge 読込等）は None。
    #[serde(default)]
    pub shape: Option<crate::section_shape::SectionShape>,
    /// 主材料（この断面の弾性剛性 E・ν と自重の密度を決める材料）。
    ///
    /// S 断面は鋼材、RC・SRC・CFT 断面はコンクリートを指す。**材料は断面の属性**で
    /// あり、部材は持たない（材料が違えばそれは別の断面である）。要素から引くときは
    /// [`Model::element_material`] を用いる。
    ///
    /// `None` は未割当。もっともらしい既定値で埋めず、解析へ進む時点で
    /// 解析前チェックが止める。
    #[serde(default)]
    pub material: Option<MaterialId>,
    /// 主筋の材料（RC・SRC 断面のみ意味を持つ）。降伏点は `Material::fy`。
    #[serde(default)]
    pub rebar_material: Option<MaterialId>,
    /// せん断補強筋の材料（RC・SRC 断面のみ意味を持つ）。
    ///
    /// `None` は未設定。呼び出し側は普通強度せん断補強筋の SD295 相当
    /// （[`crate::material_grade::SHEAR_REBAR_DEFAULT_FY`]）を既定とする
    /// （規格上の最小グレードであり、実際がより高強度でも耐力を過小評価する
    /// 側＝安全側に外れる）。
    #[serde(default)]
    pub shear_rebar_material: Option<MaterialId>,
    /// SRC 断面の内蔵鉄骨の材料（SRC 断面のみ意味を持つ）。
    #[serde(default)]
    pub steel_material: Option<MaterialId>,
}

/// 断面の同一性キー（符号＋階）。モデル内で重複してはならない。
pub type SectionKey<'a> = (&'a str, Option<&'a str>);

impl Section {
    /// 同一性キー（符号＋階）を借用で返す。
    pub fn key(&self) -> SectionKey<'_> {
        (self.name.as_str(), self.floor.as_deref())
    }

    /// 表示用のラベル。階を持つ断面は `C1 (2)`、持たない断面は符号のみ。
    pub fn display_name(&self) -> String {
        match &self.floor {
            Some(f) => format!("{} ({})", self.name, f),
            None => self.name.clone(),
        }
    }

    /// 断面性能・形状・材料が一致するか（同一性キーは見ない）。
    /// 取り込み時に符号＋階が衝突した断面を統合してよいかの判定に使う。
    ///
    /// **材料も比較の対象に含める。** 材料は断面が持ち、違う材料を割り当てるなら
    /// それは別の断面であるため、材料だけが違う定義を統合すると片方の材料が
    /// 無言で捨てられる。
    pub fn properties_eq(&self, other: &Section) -> bool {
        self.area == other.area
            && self.iy == other.iy
            && self.iz == other.iz
            && self.j == other.j
            && self.depth == other.depth
            && self.width == other.width
            && self.as_y == other.as_y
            && self.as_z == other.as_z
            && self.panel_thickness == other.panel_thickness
            && self.thickness == other.thickness
            && self.shape == other.shape
            && self.material == other.material
            && self.rebar_material == other.rebar_material
            && self.shear_rebar_material == other.shear_rebar_material
            && self.steel_material == other.steel_material
    }
}

/// `sections` に符号＋階が `key` と一致する断面があるか（`skip` の添字は除く）。
///
/// 断面の追加・改名の前段でモデル側の不変条件（符号＋階は一意）を守るために使う。
/// `skip` は改名時に自分自身を衝突判定から外すためのもので、追加時は `None`。
pub fn section_key_taken(sections: &[Section], key: SectionKey<'_>, skip: Option<usize>) -> bool {
    sections
        .iter()
        .enumerate()
        .any(|(i, s)| Some(i) != skip && s.key() == key)
}

impl Model {
    /// 要素の断面。
    pub fn element_section(&self, elem: &ElementData) -> Option<&Section> {
        self.sections.get(elem.section?.index())
    }

    /// 要素の主材料（弾性剛性 E・ν と自重の密度を決める材料）。
    ///
    /// **材料は断面が持つ**（[`Section::material`]）ため、要素からは断面を経由して
    /// 引く。材料が要る箇所は常にこのヘルパーを情報源とし、断面と材料の対応を
    /// 呼び出し側へ散らさない。断面が未割当、または断面が材料を持たない場合は `None`。
    pub fn element_material(&self, elem: &ElementData) -> Option<&Material> {
        self.materials
            .get(self.element_section(elem)?.material?.index())
    }

    /// 二次部材（小梁・間柱）の主材料（自重算定に用いる）。
    /// 規約は [`Model::element_material`] と同じ。
    pub fn secondary_material(&self, sm: &SecondaryMember) -> Option<&Material> {
        let sec = self.sections.get(sm.section?.index())?;
        self.materials.get(sec.material?.index())
    }

    /// 要素の主筋材料（RC・SRC 断面のみ）。
    pub fn element_rebar_material(&self, elem: &ElementData) -> Option<&Material> {
        self.materials
            .get(self.element_section(elem)?.rebar_material?.index())
    }

    /// 要素のせん断補強筋材料（RC・SRC 断面のみ）。
    pub fn element_shear_rebar_material(&self, elem: &ElementData) -> Option<&Material> {
        self.materials
            .get(self.element_section(elem)?.shear_rebar_material?.index())
    }

    /// 要素の内蔵鉄骨材料（SRC 断面のみ）。
    pub fn element_steel_material(&self, elem: &ElementData) -> Option<&Material> {
        self.materials
            .get(self.element_section(elem)?.steel_material?.index())
    }
}
