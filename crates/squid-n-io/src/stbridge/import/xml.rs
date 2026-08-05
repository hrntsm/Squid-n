//! ST-Bridge パースの XML 属性・数値ヘルパ（属性辞書化と型付き取得）。
//!
//! 属性辞書 [`Attrs`] は**参照されたキーを記録する**。取り込み後に「要素に存在したが
//! 一度も参照されなかった属性」を差分で取り出せるため、無視リストを持たずに
//! 読み飛ばしを検出できる（[`Attrs::unread`]）。パーサに読み取りを足せば記録も
//! 自動的に追随するため、一覧の保守が不要になる。

use super::super::StbError;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};

/// XML 要素 1 個分の属性辞書。参照されたキーを内部で記録する。
///
/// 記録の単位は「参照を試みたキー」で、値の有無は問わない。`get_f64_any` のように
/// 複数の候補キーを順に試す取得では、実際に試みたキーだけが記録される
/// （先に一致したキーで打ち切るため、後続の候補が要素に存在すれば未参照として残る）。
#[derive(Debug, Default)]
pub(super) struct Attrs {
    map: HashMap<String, String>,
    read: RefCell<HashSet<String>>,
}

impl Attrs {
    /// 属性値を取得し、キーを参照済みとして記録する。
    pub(super) fn get(&self, k: &str) -> Option<&String> {
        // 同じキーを何度参照しても記録は 1 回でよいため、未記録のときだけ確保する。
        // `borrow()` の一時値は文の終わりまで生きるので、判定と挿入は文を分ける
        // （1 つの式にまとめると `borrow_mut()` と重なって実行時パニックになる）。
        let seen = self.read.borrow().contains(k);
        if !seen {
            self.read.borrow_mut().insert(k.to_string());
        }
        self.map.get(k)
    }

    /// 要素に存在した属性名（名前昇順。報告の決定性のため整列する）。
    pub(super) fn names(&self) -> Vec<&str> {
        let mut v: Vec<&str> = self.map.keys().map(String::as_str).collect();
        v.sort_unstable();
        v
    }

    /// 要素に存在したが一度も参照されなかった属性名（名前昇順）。
    pub(super) fn unread(&self) -> Vec<&str> {
        let read = self.read.borrow();
        let mut v: Vec<&str> = self
            .map
            .keys()
            .filter(|k| !read.contains(*k))
            .map(String::as_str)
            .collect();
        v.sort_unstable();
        v
    }
}

impl FromIterator<(String, String)> for Attrs {
    fn from_iter<I: IntoIterator<Item = (String, String)>>(iter: I) -> Self {
        Self {
            map: iter.into_iter().collect(),
            read: RefCell::new(HashSet::new()),
        }
    }
}

/// `StbNodeIdOrder` の内容文字列（空白区切りの節点 id 列）を解析し、
/// 数値として読める token を境界（`boundary`）へ追加する（スラブ・壁共用）。
pub(super) fn push_node_id_tokens(text: &str, boundary: &mut Vec<u32>) {
    for tok in text.split_whitespace() {
        if let Ok(id) = tok.parse::<u32>() {
            boundary.push(id);
        }
    }
}

pub(super) fn attrs(e: &quick_xml::events::BytesStart) -> Result<Attrs, StbError> {
    let mut m: Vec<(String, String)> = Vec::new();
    for a in e.attributes() {
        let a = a.map_err(|err| StbError::Parse(err.to_string()))?;
        let key = String::from_utf8_lossy(a.key.as_ref()).to_string();
        let val = a
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|err| StbError::Parse(err.to_string()))?
            .to_string();
        m.push((key, val));
    }
    Ok(m.into_iter().collect())
}

pub(super) fn get_f64(a: &Attrs, k: &str) -> Result<f64, StbError> {
    a.get(k)
        .ok_or_else(|| StbError::Parse(format!("missing attr {k}")))?
        .parse::<f64>()
        .map_err(|_| StbError::Parse(format!("bad f64 attr {k}")))
}

/// 複数の候補キーのいずれかから f64 を取る（属性名の方言差を吸収する）。
pub(super) fn get_f64_any(a: &Attrs, keys: &[&str]) -> Result<f64, StbError> {
    for k in keys {
        if let Some(v) = a.get(k) {
            return v
                .parse::<f64>()
                .map_err(|_| StbError::Parse(format!("bad f64 attr {k}")));
        }
    }
    Err(StbError::Parse(format!("missing attr {:?}", keys)))
}

pub(super) fn get_opt_f64(a: &Attrs, k: &str) -> Option<f64> {
    match a.get(k) {
        Some(v) if !v.is_empty() => v.parse::<f64>().ok(),
        _ => None,
    }
}

pub(super) fn get_u32(a: &Attrs, k: &str) -> Result<u32, StbError> {
    a.get(k)
        .ok_or_else(|| StbError::Parse(format!("missing attr {k}")))?
        .parse::<u32>()
        .map_err(|_| StbError::Parse(format!("bad u32 attr {k}")))
}

pub(super) fn get_i64(a: &Attrs, k: &str) -> Option<i64> {
    a.get(k).and_then(|v| v.parse::<i64>().ok())
}
