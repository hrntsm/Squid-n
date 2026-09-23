use thiserror::Error;

#[derive(Error, Debug, Clone, PartialEq, Eq)]
pub enum CoreError {
    #[error("duplicate id: {0}")]
    DuplicateId(String),
    #[error("dangling reference: {0}")]
    DanglingRef(String),
    #[error("index mismatch: {0}")]
    IndexMismatch(String),
}

/// RC 実配筋の幾何検証エラー。
///
/// 段別配筋の入力値と断面寸法から実鉄筋座標を一意に生成できない場合に返す。
#[derive(Error, Debug, Clone, PartialEq)]
pub enum RebarGeometryError {
    #[error("寸法が非有限または非正です: {field}")]
    InvalidDimension { field: &'static str },
    #[error("段の本数が不正です: {location} 段{layer} の {count} 本")]
    InvalidLayer {
        location: &'static str,
        layer: usize,
        count: u32,
    },
    #[error("矩形柱の交点本数が不足しています: {axis} 方向 段{layer} は {count} 本（{required} 本以上が必要）")]
    TooFewIntersectionBars {
        axis: &'static str,
        layer: usize,
        count: u32,
        required: u32,
    },
    #[error("主筋が断面内に収まりません")]
    OutOfBounds,
    #[error("段が断面中心を越えて衝突します")]
    LayerCollision,
    #[error("対称配置または必要間隔を確保できません")]
    SpacingOrSymmetry,
    #[error("生成した実鉄筋本数が入力と一致しません")]
    CountMismatch,
}
