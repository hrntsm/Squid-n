//! 解析前処理（モデルを解ける状態へ整える）。
//!
//! **解析の直前に必ず通すこと。** 前処理を省くと、仕口パネルのない剛性で解いたり、
//! 地震力ゼロで増分解析したりすることになる（実際に MCP サーバ側でそれが起きて
//! いた。GUI 側は `ensure_preparation` で同じ処理を通していた）。

use squid_n_core::model::Model;

/// 剛域と仕口パネルを自動算定してモデルへ反映する。
///
/// - 剛域: `Model::stress_cfg.rigid_zone_consider_walls` に従って壁を考慮する
/// - 仕口パネル: `Model::panel_zone` が有効なら S 造（CFT を除く）の柱梁接合節点へ
///   パネルを設け、無効なら既存のパネルを取り除く。あわせて部材の
///   `RigidZone::panel_offset_i/j` を現在のパネル配置から求め直す
///
/// いずれも冪等で、書き込み先が異なるため呼び出し順にも依存しない。
/// 戻り値は生成した仕口パネルの一覧（GUI の準備計算表が表示する）。
pub fn apply_rigid_zones_and_panels(
    model: &mut Model,
) -> Vec<squid_n_element::panel_gen::GeneratedPanel> {
    let rule = squid_n_element::beam::RigidZoneRule {
        consider_walls: model.stress_cfg.rigid_zone_consider_walls,
    };
    squid_n_element::beam::apply_auto_rigid_zones(model, &rule);
    squid_n_element::panel_gen::apply_auto_panel_zones(model)
}
