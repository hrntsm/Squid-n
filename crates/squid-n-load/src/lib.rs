pub mod ai;
/// 標準荷重組合せ（令82条）。実体は [`squid_n_core::load_combo`]。
/// 荷重ケースの意味（`LoadCaseKind`）と長期判定を持つ core に置き、
/// 新規モデルの既定組合せと同じ実装を使う。
pub use squid_n_core::load_combo as combo;
pub mod floor;
pub mod live_load;
pub mod secondary;
pub mod self_weight;
pub mod story_gen;
pub mod wall_attached;
pub mod wall_expand;
