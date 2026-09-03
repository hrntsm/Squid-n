//! 幾何ユーティリティ（座標計算）。
//!
//! - `polygon_area_3d` — 平面多角形（3D座標）の面積
//!   （[`squid_n_core::geom::polygon::area_3d`] の再エクスポート）
//! - [`dist3`] — 2 点間の 3D 距離
//! - `is_vertical_pair` — 両端が鉛直材（柱）かの判定
//!   （[`squid_n_core::geom::is_vertical_pair`] の再エクスポート）

/// 平面多角形（3D座標）の面積。壁・シェル要素の自重（§1.2）算定に用いる。
/// 算定の情報源は `squid-n-core` に置く。
pub(super) use squid_n_core::geom::polygon::area_3d as polygon_area_3d;

/// 2 点間の 3D 距離 [mm]。算定の情報源は `squid-n-core` に置く。
pub(super) use squid_n_core::geom::vec3::dist as dist3;

/// 「鉛直材（柱）」判定。仕上げ周長式・柱脚梁せい付加・壁領域の構面走査・通り芯の
/// 自動生成が共通で用いるため、判定規則は `squid-n-core` を情報源とする。
pub(super) use squid_n_core::geom::is_vertical_pair;
