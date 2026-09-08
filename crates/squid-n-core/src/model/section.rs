//! 断面の型。

use super::*;

pub fn rect_shear_area(area: f64) -> f64 {
    area * 5.0 / 6.0
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Section {
    pub id: SectionId,
    /// 断面符号。単独では断面を一意に定めない。
    pub name: String,
    /// 階。[`Story`](crate::model::Story) への参照ではない。
    ///
    /// 断面の同一性は符号＋階で決まる。階を持たない断面は `None` とし、
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
    /// パラメトリック形状定義。形状から生成されなかった断面は None。
    #[serde(default)]
    pub shape: Option<crate::section_shape::SectionShape>,
    /// 主材料（この断面の弾性剛性 E・ν と自重の密度を決める材料）。
    ///
    /// `None` は未割当。
    #[serde(default)]
    pub material: Option<MaterialId>,
    /// 主筋の材料（RC・SRC 断面のみ意味を持つ）。
    #[serde(default)]
    pub rebar_material: Option<MaterialId>,
    /// せん断補強筋の材料（RC・SRC 断面のみ意味を持つ）。`None` は未設定。
    #[serde(default)]
    pub shear_rebar_material: Option<MaterialId>,
    /// SRC 断面の内蔵鉄骨の材料（SRC 断面のみ意味を持つ）。
    #[serde(default)]
    pub steel_material: Option<MaterialId>,
}

/// 断面の同一性キー（符号＋階）。モデル内で重複してはならない。
pub type SectionKey<'a> = (&'a str, Option<&'a str>);

impl Section {
    /// 物性がすべてゼロ・形状も材料も持たない断面。
    pub fn zero(id: SectionId, name: String) -> Self {
        Self {
            id,
            name,
            floor: None,
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: None,
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }

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
    /// 断面が未割当、または断面が材料を持たない場合は `None`。
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
