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

    /// 断面性能・形状が一致するか（同一性キーは見ない）。
    /// 取り込み時に符号＋階が衝突した断面を統合してよいかの判定に使う。
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
