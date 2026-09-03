//! 実モデルから柱・梁を拾って計算コア（[`super::core`]）へ渡す略算抽出層。
//!
//! - [`center_of_mass`] — 重心（質量中心）[Xg, Yg]。
//! - [`column_stiffnesses`] — 各柱の方向別 D 値と平面位置。
//! - [`StoryCenters`] — 剛心・重心をまとめた構造体。
//! - [`story_centers`] — 当該層の剛心・重心。
//! - [`story_eccentricity`] — 当該層の偏心率（雑壁寄与込み）。

use squid_n_core::ids::StoryId;
use squid_n_core::model::{ElementKind, Model};
use squid_n_element::transform::LocalFrame;

use super::core::{center_of_rigidity, d_value, eccentricity, ColumnStiffness, Eccentricity};
use super::misc_wall::append_misc_wall_stiffnesses;

// ===== モデル抽出層（略算）=====

/// 重心（質量中心）[Xg, Yg]。当該層の節点質量（並進成分）で重み付けする。
///
/// 質量未定義の節点は質量 0（剛心の重み付けには寄与しない）。全質量 0 なら幾何重心。
pub fn center_of_mass(model: &Model, story: StoryId) -> [f64; 2] {
    let nodes: Vec<&squid_n_core::model::Node> = model
        .nodes
        .iter()
        .filter(|n| n.story == Some(story))
        .collect();
    if nodes.is_empty() {
        return [0.0, 0.0];
    }
    let mass = |n: &squid_n_core::model::Node| n.mass.map(|m| m[0]).unwrap_or(0.0);
    let total: f64 = nodes.iter().map(|n| mass(n)).sum();
    if total > 0.0 {
        let xg = nodes.iter().map(|n| mass(n) * n.coord[0]).sum::<f64>() / total;
        let yg = nodes.iter().map(|n| mass(n) * n.coord[1]).sum::<f64>() / total;
        [xg, yg]
    } else {
        // 質量未定義 → 幾何重心で代用。
        let n = nodes.len() as f64;
        let xg = nodes.iter().map(|n| n.coord[0]).sum::<f64>() / n;
        let yg = nodes.iter().map(|n| n.coord[1]).sum::<f64>() / n;
        [xg, yg]
    }
}

// ===== モデル自動算定層（column_stiffnesses / StoryCenters / story_centers / story_eccentricity）=====

/// 当該層の各柱について方向別水平剛性（D値）と平面位置を算定して返す（仕様 §5.1）。
///
/// 中間節点で分割された鉛直材は連なりとして 1 本とみなす（[`crate::secondary::story_columns`]）。
/// \( h \) は層の上下端床の標高差であり、セグメントの材長ではない。
/// 断面・材料・局所軸は最上側セグメント（`StoryColumn::top_elem`）から取る。
///
/// `story` には**層の上端の階**（[`squid_n_core::model::Layer::top`]）を渡すこと。
/// 層の柱はその上端の床に取り付くためである。
pub fn column_stiffnesses(model: &Model, story: StoryId) -> Vec<ColumnStiffness> {
    // 最下層判定: 当該階が最下層の上端（＝基部の 1 つ上の階）なら true。
    // D 値法の最下層は柱脚の支持条件が他層と異なるため区別する。
    let first_story = story.index() == 1;

    let mut result = Vec::new();

    for col in crate::secondary::story_columns::story_columns(model, story) {
        let Some(elem) = model.element(col.top_elem) else {
            continue;
        };
        if elem.kind != ElementKind::Beam || elem.nodes.len() != 2 {
            continue;
        }
        let n_top = &model.nodes[col.top.index()];
        let n_bot = &model.nodes[col.bottom.index()];
        let p_top = n_top.coord;
        let p_bot = n_bot.coord;
        let p0 = model.nodes[elem.nodes[0].index()].coord;
        let p1 = model.nodes[elem.nodes[1].index()].coord;

        // material / section が必須。
        let Some(mat) = model.element_material(elem) else {
            continue;
        };
        let sid = match elem.section {
            Some(s) => s,
            None => continue,
        };
        let sec = &model.sections[sid.index()];
        let e = mat.young;
        let h = (p_top[2] - p_bot[2]).abs();
        if h < 1e-12 {
            continue;
        }

        // 局所座標系から ey, ez を取得。
        let ref_vec = elem.local_axis.ref_vector;
        let frame = LocalFrame::from_nodes(p0, p1, ref_vec);
        let ey = frame.rot[1]; // 局所 y 軸（全体方向への射影に使う）
        let ez = frame.rot[2]; // 局所 z 軸

        // 方向別有効断面二次モーメント（局所→全体の射影）。
        // 全体 X 方向変位に抵抗: 局所 y 方向成分 iz, 局所 z 方向成分 iy。
        // 断面レイヤの iy=強軸（せい方向 D³ 系）は、要素座標系では z 軸まわり
        // （たわみ y 方向）に対応するためクロスして用いる（beam/construct.rs と同一規約）。
        let iy = sec.iz;
        let iz = sec.iy;
        let i_global_x = iz * ey[0] * ey[0] + iy * ez[0] * ez[0];
        let i_global_y = iz * ey[1] * ey[1] + iy * ez[1] * ez[1];

        // 梁剛比 ΣKb（武藤 a 補正用）。当該柱の上端・下端節点に取り付く水平梁を探す。
        let (sum_kb_x, sum_kb_y) = {
            let mut skbx = 0.0_f64;
            let mut skby = 0.0_f64;
            for other in &model.elements {
                if other.id == elem.id {
                    continue;
                }
                if other.kind != ElementKind::Beam || other.nodes.len() != 2 {
                    continue;
                }
                // 当該柱の節点（上端または下端）を含む梁か。
                let has_top = other.nodes.contains(&n_top.id);
                let has_bot = other.nodes.contains(&n_bot.id);
                if !has_top && !has_bot {
                    continue;
                }
                // 梁の部材軸単位ベクトル。
                let bn0 = &model.nodes[other.nodes[0].index()];
                let bn1 = &model.nodes[other.nodes[1].index()];
                let bdx = bn1.coord[0] - bn0.coord[0];
                let bdy = bn1.coord[1] - bn0.coord[1];
                let bdz = bn1.coord[2] - bn0.coord[2];
                let bl = (bdx * bdx + bdy * bdy + bdz * bdz).sqrt();
                if bl < 1e-12 {
                    continue;
                }
                // 鉛直材は梁剛比に算入しない（全クレート共通の 45° 余弦基準）。
                if squid_n_core::geom::is_vertical_axis(bn0.coord, bn1.coord) {
                    continue;
                }
                let bex = [bdx / bl, bdy / bl, bdz / bl];
                // 梁の断面二次モーメント（強軸＝断面レイヤの iy）と梁剛比。
                let beam_i_strong = match other.section {
                    Some(s) => model.sections[s.index()].iy,
                    None => continue,
                };
                let kb = beam_i_strong / bl;
                // 方向別に効く梁の判定（全クレート共通の 45° 余弦基準）。
                if squid_n_core::geom::axis_dominates(bex, 0) {
                    skbx += kb;
                }
                if squid_n_core::geom::axis_dominates(bex, 1) {
                    skby += kb;
                }
            }
            (skbx, skby)
        };

        let dx_val = d_value(e, i_global_x, h, sum_kb_x, first_story);
        let dy_val = d_value(e, i_global_y, h, sum_kb_y, first_story);
        let pos = [p_top[0], p_top[1]];
        result.push(ColumnStiffness {
            pos,
            dx: dx_val,
            dy: dy_val,
        });
    }
    result
}

/// 剛心・重心をまとめた構造体。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoryCenters {
    pub center_of_mass: [f64; 2],
    pub center_of_rigidity: [f64; 2],
}

/// 当該層の剛心・重心を算定して返す。
pub fn story_centers(model: &Model, story: StoryId) -> StoryCenters {
    let cols = column_stiffnesses(model, story);
    let com = center_of_mass(model, story);
    let cor = center_of_rigidity(&cols);
    StoryCenters {
        center_of_mass: com,
        center_of_rigidity: cor,
    }
}

/// 当該層の偏心率を算定して返す。
pub fn story_eccentricity(model: &Model, story: StoryId) -> Eccentricity {
    let mut cols = column_stiffnesses(model, story);
    append_misc_wall_stiffnesses(model, story, &mut cols);
    let cor = center_of_rigidity(&cols);
    let com = center_of_mass(model, story);
    eccentricity(&cols, com, cor)
}
