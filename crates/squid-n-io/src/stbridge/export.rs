//! ST-Bridge 直列化（Export）。設計書 §12.5。
//!
//! 出力は **ST-Bridge 2.0.2 標準スキーマ準拠**の幾何モデル（他ソフト・BIM が読める形）。
//! - 断面は標準要素（`StbSecColumn_S`/`StbSecBeam_RC` 等）＋形鋼ライブラリ `StbSecSteel`。
//! - 部材は複数形コンテナ（`StbColumns`/`StbGirders`/`StbBeams`/`StbBraces`/`StbSlabs`/
//!   `StbWalls`）に入れ、向きは `rotate`、端部は `condition_*`、ブレースは `feature_brace`。
//! - 材料は ST-Bridge の慣習どおり断面のグレード名（鋼 `strength_main`、RC/SRC/CFT の
//!   コンクリート `strength_concrete`）で表す（`StbModel` は材料テーブルを持たない）。
//! - id は ST-Bridge の `positiveInteger`（1 始まり）に合わせ、内部 0 始まり id に +1 する。
//!
//! ST-Bridge の幾何スコープ外（材料の E/ν・節点荷重・拘束・独自属性）は往復しない。
//! 完全一致の往復が必要な場合はネイティブの `.scz` を使う（`docs/model_io/`）。
//!
//! - [`export_stbridge`] — 内部モデルを標準 ST-Bridge 2.0.2 XML 文字列へ出力する。
//! - [`fmt`] — 整数値は小数点なし、それ以外は既定の f64 表記で整形する（`pub(super)`）。
//! - [`esc`] — XML 特殊文字をエスケープする（`pub(super)`）。

use super::section_std::standard_sections;
use super::{StbError, STB_VERSION};
use squid_n_core::ids::{SectionId, SlabId};
use squid_n_core::model::{
    AxisGroup, AxisGroupKind, ElementKind, EndCondition, Model, StoryLevelKind,
};

/// ST-Bridge の id は `positiveInteger`（1 以上）。内部 0 始まり id に +1 して出力する。
fn sid(internal_id: u32) -> u32 {
    internal_id + 1
}

/// 内部モデルを標準 ST-Bridge 2.0.2 XML 文字列へ出力する。
pub fn export_stbridge(model: &Model) -> Result<String, StbError> {
    // 標準断面ブロックと、部材参照（id_section）の柱用・梁用張り替えマップ。
    let std = standard_sections(model);
    let (sections_body, steel_lib, col_map, beam_map) =
        (std.sections_xml, std.steel_lib, std.col_map, std.beam_map);

    let mut s = String::new();
    s.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    s.push_str(&format!(
        "<ST_BRIDGE xmlns=\"https://www.building-smart.or.jp/dl\" \
         xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" version=\"{STB_VERSION}\">\n"
    ));

    // StbCommon（ルート必須。プロジェクト名・アプリ名は最小限の既定値）。
    s.push_str(
        "  <StbCommon project_name=\"Squid-n\" app_name=\"Squid-n\" app_version=\"0.0.1\"/>\n",
    );

    s.push_str("  <StbModel>\n");

    // 節点（X/Y/Z、kind 必須）。所属部材が不明なので kind=ON_GRID を既定にする。
    s.push_str("    <StbNodes>\n");
    for n in &model.nodes {
        s.push_str(&format!(
            "      <StbNode id=\"{}\" X=\"{}\" Y=\"{}\" Z=\"{}\" kind=\"ON_GRID\"/>\n",
            sid(n.id.0),
            fmt(n.coord[0]),
            fmt(n.coord[1]),
            fmt(n.coord[2]),
        ));
    }
    s.push_str("    </StbNodes>\n");

    // 通り芯（`StbAxes`。スキーマ上 `StbNodes` と `StbStories` の間に置く）。
    s.push_str(&axes_body(model));

    // 層（name・height・kind 必須。所属節点は StbNodeIdList で列挙）。所属は各節点の
    // `story`（正）と層の `node_ids` の和集合を、節点 id 昇順・重複なしで書き出す。
    s.push_str("    <StbStories>\n");
    for st in &model.stories {
        let mut members: Vec<u32> = model
            .nodes
            .iter()
            .filter(|n| n.story == Some(st.id))
            .map(|n| n.id.0)
            .collect();
        for nid in &st.node_ids {
            if !members.contains(&nid.0) {
                members.push(nid.0);
            }
        }
        members.sort_unstable();
        s.push_str(&format!(
            "      <StbStory id=\"{}\" name=\"{}\" height=\"{}\" kind=\"{}\">\n",
            sid(st.id.0),
            esc(&st.name),
            fmt(st.elevation),
            story_kind(st.level_kind),
        ));
        if !members.is_empty() {
            s.push_str("        <StbNodeIdList>\n");
            for nid in members {
                s.push_str(&format!("          <StbNodeId id=\"{}\"/>\n", sid(nid)));
            }
            s.push_str("        </StbNodeIdList>\n");
        }
        s.push_str("      </StbStory>\n");
    }
    s.push_str("    </StbStories>\n");

    // 部材（複数形コンテナに種別ごとに束ねる。空コンテナは出力しない）。
    s.push_str("    <StbMembers>\n");
    s.push_str(&members_body(model, &col_map, &beam_map));
    s.push_str("    </StbMembers>\n");

    // 断面（標準要素＋形鋼ライブラリ）＋スラブ断面＋壁断面。
    let slab_sec_base = slab_section_id_base(model, &col_map, &beam_map);
    let wall_sec_base = slab_sec_base + model.slabs.len() as u32;

    // スキーマ順: 柱・梁・ブレース断面 → スラブ断面 → 壁断面 → 形鋼ライブラリ。
    s.push_str("    <StbSections>\n");
    s.push_str(&sections_body);
    s.push_str(&slab_sections(model, slab_sec_base));
    s.push_str(&wall_sections(model, wall_sec_base));
    s.push_str(&steel_lib);
    s.push_str("    </StbSections>\n");

    s.push_str("    <StbJoints/>\n");
    s.push_str("  </StbModel>\n");
    s.push_str("</ST_BRIDGE>\n");
    Ok(s)
}

/// スラブ断面 id の採番開始値。既存断面 id（柱・梁。柱/梁の役割分割で
/// 増える分は col_map/beam_map の値域に現れる）と衝突しない範囲から採る。
/// `StbSections`（断面定義側）と `StbMembers`（スラブの断面参照側）が同じ
/// 採番を共有するための単一実装。
fn slab_section_id_base(
    model: &Model,
    col_map: &std::collections::HashMap<u32, u32>,
    beam_map: &std::collections::HashMap<u32, u32>,
) -> u32 {
    col_map
        .values()
        .chain(beam_map.values())
        .copied()
        .max()
        .map(|m| m + 1)
        .unwrap_or(0)
        .max(model.sections.len() as u32)
}

/// `StbMembers` 本体（柱・大梁・ブレース・スラブ・壁を複数形コンテナに束ねる）。
fn members_body(
    model: &Model,
    col_map: &std::collections::HashMap<u32, u32>,
    beam_map: &std::collections::HashMap<u32, u32>,
) -> String {
    let mut columns = String::new();
    let mut girders = String::new();
    let mut braces = String::new();

    for e in &model.elements {
        match e.kind {
            ElementKind::Beam if e.nodes.len() == 2 => {
                let n0 = &model.nodes[e.nodes[0].index()];
                let n1 = &model.nodes[e.nodes[1].index()];
                // 全クレート共通の 45° 余弦基準で柱/大梁を分ける。
                let is_col = squid_n_core::geom::is_vertical_axis(n0.coord, n1.coord);
                let role_map = if is_col { col_map } else { beam_map };
                let sec = e
                    .section
                    .map(|s| role_map.get(&s.0).copied().unwrap_or(s.0))
                    .map(|v| v as i64)
                    .unwrap_or(-1);
                let rot = rotate_of(e, n0.coord, n1.coord);
                let ks = kind_structure(model, e);
                if is_col {
                    let (bot, top) = if n0.coord[2] <= n1.coord[2] {
                        (e.nodes[0], e.nodes[1])
                    } else {
                        (e.nodes[1], e.nodes[0])
                    };
                    let (cb, ct) = if n0.coord[2] <= n1.coord[2] {
                        (e.end_cond[0], e.end_cond[1])
                    } else {
                        (e.end_cond[1], e.end_cond[0])
                    };
                    columns.push_str(&format!(
                        "        <StbColumn id=\"{}\" name=\"C{}\" id_node_bottom=\"{}\" id_node_top=\"{}\" \
                         rotate=\"{}\" id_section=\"{}\" kind_structure=\"{}\" condition_bottom=\"{}\" condition_top=\"{}\"/>\n",
                        sid(e.id.0), sid(e.id.0), sid(bot.0), sid(top.0),
                        fmt(rot), sec_ref(sec), ks, cond(cb), cond(ct),
                    ));
                } else {
                    girders.push_str(&format!(
                        "        <StbGirder id=\"{}\" name=\"G{}\" id_node_start=\"{}\" id_node_end=\"{}\" \
                         rotate=\"{}\" id_section=\"{}\" kind_structure=\"{}\" isFoundation=\"false\" \
                         condition_start=\"{}\" condition_end=\"{}\"/>\n",
                        sid(e.id.0), sid(e.id.0), sid(e.nodes[0].0), sid(e.nodes[1].0),
                        fmt(rot), sec_ref(sec), ks, cond(e.end_cond[0]), cond(e.end_cond[1]),
                    ));
                }
            }
            ElementKind::Brace { tension_only } if e.nodes.len() == 2 => {
                let sec = e
                    .section
                    .map(|s| {
                        col_map
                            .get(&s.0)
                            .or_else(|| beam_map.get(&s.0))
                            .copied()
                            .unwrap_or(s.0) as i64
                    })
                    .unwrap_or(-1);
                let feature = if tension_only {
                    "TENSION"
                } else {
                    "TENSIONANDCOMPRESSION"
                };
                braces.push_str(&format!(
                    "        <StbBrace id=\"{}\" name=\"BR{}\" id_node_start=\"{}\" id_node_end=\"{}\" \
                     rotate=\"0\" id_section=\"{}\" kind_structure=\"S\" feature_brace=\"{}\" \
                     condition_start=\"PIN\" condition_end=\"PIN\"/>\n",
                    sid(e.id.0), sid(e.id.0), sid(e.nodes[0].0), sid(e.nodes[1].0),
                    sec_ref(sec), feature,
                ));
            }
            _ => {}
        }
    }

    // 二次部材（小梁 StbBeam・間柱 StbPost）。全体解析の対象外だが往復のため
    // 書き出す。member id は要素 id と別空間なので要素数の次から採番する。
    let secondary_member_base = model.elements.len() as u32;
    let mut sec_beams = String::new();
    let mut posts = String::new();
    for (i, sm) in model.secondary_members.iter().enumerate() {
        let mid = secondary_member_base + i as u32;
        let sec = sm
            .section
            .map(|s| {
                beam_map
                    .get(&s.0)
                    .or_else(|| col_map.get(&s.0))
                    .copied()
                    .unwrap_or(s.0) as i64
            })
            .unwrap_or(-1);
        let ks = model
            .secondary_material(sm)
            .map(|m| {
                if m.fc.is_some() {
                    "RC".to_string()
                } else {
                    "S".to_string()
                }
            })
            .unwrap_or_else(|| "S".to_string());
        match sm.kind {
            squid_n_core::model::SecondaryMemberKind::Joist => {
                sec_beams.push_str(&format!(
                    "        <StbBeam id=\"{}\" name=\"B{}\" id_node_start=\"{}\" id_node_end=\"{}\" \
                     rotate=\"0\" id_section=\"{}\" kind_structure=\"{}\" isFoundation=\"false\"/>\n",
                    sid(mid), sid(mid), sid(sm.nodes[0].0), sid(sm.nodes[1].0), sec_ref(sec), ks,
                ));
            }
            squid_n_core::model::SecondaryMemberKind::Post => {
                // 下端→上端の順（Z で並べ替え）。
                let n0 = &model.nodes[sm.nodes[0].index()];
                let n1 = &model.nodes[sm.nodes[1].index()];
                let (bot, top) = if n0.coord[2] <= n1.coord[2] {
                    (sm.nodes[0], sm.nodes[1])
                } else {
                    (sm.nodes[1], sm.nodes[0])
                };
                posts.push_str(&format!(
                    "        <StbPost id=\"{}\" name=\"P{}\" id_node_bottom=\"{}\" id_node_top=\"{}\" \
                     rotate=\"0\" id_section=\"{}\" kind_structure=\"{}\"/>\n",
                    sid(mid), sid(mid), sid(bot.0), sid(top.0), sec_ref(sec), ks,
                ));
            }
        }
    }

    // スラブ（StbSlab）。境界節点ループ＋断面参照。member id は要素 id と別空間なので
    // 要素数の次から採番する（1 始まり。二次部材の後）。
    let slab_member_base = model.elements.len() as u32 + model.secondary_members.len() as u32;
    let slab_sec_base = slab_section_id_base(model, col_map, beam_map);
    let slab_sec_ids = slab_section_ids(model, slab_sec_base);
    let mut slabs = String::new();
    for slab in &model.slabs {
        let mid = slab_member_base + slab.id.0;
        let sec = slab_sec_ids.get(&slab.id).copied().unwrap_or(slab_sec_base);
        let order = slab
            .boundary
            .iter()
            .map(|n| sid(n.0).to_string())
            .collect::<Vec<_>>()
            .join(" ");
        let kind_slab = match slab.kind {
            squid_n_core::model::SlabKind::Interior => "NORMAL",
            _ => "CANTI",
        };
        slabs.push_str(&format!(
            "        <StbSlab id=\"{}\" name=\"S{}\" id_section=\"{}\" kind_structure=\"RC\" kind_slab=\"{}\" isFoundation=\"false\">\n",
            sid(mid),
            sid(slab.id.0),
            sid(sec),
            kind_slab,
        ));
        slabs.push_str(&format!(
            "          <StbNodeIdOrder>{order}</StbNodeIdOrder>\n"
        ));
        slabs.push_str("        </StbSlab>\n");
    }

    // 壁（StbWall）。壁要素（Wall/Shell、境界 3〜N 節点）の節点ループ＋断面参照。
    let wall_sec_base = slab_sec_base + model.slabs.len() as u32;
    let mut walls = String::new();
    let mut wall_idx = 0u32;
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Wall | ElementKind::Shell) || e.nodes.len() < 3 {
            continue;
        }
        let order = e
            .nodes
            .iter()
            .map(|n| sid(n.0).to_string())
            .collect::<Vec<_>>()
            .join(" ");
        let sec = wall_sec_base + wall_idx;
        walls.push_str(&format!(
            "        <StbWall id=\"{}\" name=\"W{}\" id_section=\"{}\" kind_structure=\"RC\">\n",
            sid(e.id.0),
            sid(e.id.0),
            sid(sec),
        ));
        walls.push_str(&format!(
            "          <StbNodeIdOrder>{order}</StbNodeIdOrder>\n"
        ));
        walls.push_str("        </StbWall>\n");
        wall_idx += 1;
    }

    // 複数形コンテナはスキーマ上、子を 1 つ以上持つ必要がある。空なら出力しない。
    // 順序はスキーマの sequence（Columns→Posts→Girders→Beams→Braces→Slabs→Walls）
    // に合わせる。
    let mut body = String::new();
    if !columns.is_empty() {
        body.push_str("      <StbColumns>\n");
        body.push_str(&columns);
        body.push_str("      </StbColumns>\n");
    }
    if !posts.is_empty() {
        body.push_str("      <StbPosts>\n");
        body.push_str(&posts);
        body.push_str("      </StbPosts>\n");
    }
    if !girders.is_empty() {
        body.push_str("      <StbGirders>\n");
        body.push_str(&girders);
        body.push_str("      </StbGirders>\n");
    }
    if !sec_beams.is_empty() {
        body.push_str("      <StbBeams>\n");
        body.push_str(&sec_beams);
        body.push_str("      </StbBeams>\n");
    }
    if !braces.is_empty() {
        body.push_str("      <StbBraces>\n");
        body.push_str(&braces);
        body.push_str("      </StbBraces>\n");
    }
    if !slabs.is_empty() {
        body.push_str("      <StbSlabs>\n");
        body.push_str(&slabs);
        body.push_str("      </StbSlabs>\n");
    }
    if !walls.is_empty() {
        body.push_str("      <StbWalls>\n");
        body.push_str(&walls);
        body.push_str("      </StbWalls>\n");
    }
    body
}

/// 断面参照属性値。負（未参照）は -1、そうでなければ +1 した positiveInteger。
fn sec_ref(sec_internal: i64) -> String {
    if sec_internal < 0 {
        "-1".to_string()
    } else {
        format!("{}", sec_internal as u32 + 1)
    }
}

/// 端部接合条件（FIX/PIN）。
fn cond(c: EndCondition) -> &'static str {
    match c {
        EndCondition::Pinned => "PIN",
        _ => "FIX",
    }
}

/// 通り芯（`StbAxes`）。
///
/// 書き出せるのは平行芯（[`AxisGroupKind::Parallel`]）のグループのみ。円弧芯・
/// 放射芯・作図芯に相当する [`AxisGroupKind::Other`] のグループは幾何を保持して
/// いないため出力せず、往復しない（取り込みでは所属節点だけを保つ）。
///
/// `StbParallelAxis` の `id` は ST-Bridge の `positiveInteger`。内部の通り芯は id を
/// 持たないため、グループをまたいで 1 から通し番号を振る。
fn axes_body(model: &Model) -> String {
    let mut s = String::new();
    let groups: Vec<&AxisGroup> = model
        .axes
        .iter()
        .filter(|g| matches!(g.kind, AxisGroupKind::Parallel { .. }))
        .collect();
    if groups.is_empty() {
        return s;
    }
    s.push_str("    <StbAxes>\n");
    let mut next_id = 1u32;
    for g in groups {
        let AxisGroupKind::Parallel { origin, angle_deg } = g.kind else {
            continue;
        };
        s.push_str(&format!(
            "      <StbParallelAxes group_name=\"{}\" X=\"{}\" Y=\"{}\" angle=\"{}\">\n",
            esc(&g.name),
            fmt(origin[0]),
            fmt(origin[1]),
            fmt(angle_deg),
        ));
        for ax in &g.axes {
            s.push_str(&format!(
                "        <StbParallelAxis id=\"{}\" name=\"{}\" distance=\"{}\">\n",
                next_id,
                esc(&ax.name),
                fmt(ax.distance.unwrap_or(0.0)),
            ));
            next_id += 1;
            if !ax.nodes.is_empty() {
                s.push_str("          <StbNodeIdList>\n");
                for n in &ax.nodes {
                    s.push_str(&format!("            <StbNodeId id=\"{}\"/>\n", sid(n.0)));
                }
                s.push_str("          </StbNodeIdList>\n");
            }
            s.push_str("        </StbParallelAxis>\n");
        }
        s.push_str("      </StbParallelAxes>\n");
    }
    s.push_str("    </StbAxes>\n");
    s
}

/// 層種別を ST-Bridge の `kind`（GENERAL/PENTHOUSE/BASEMENT）へ写す。
fn story_kind(k: StoryLevelKind) -> &'static str {
    match k {
        StoryLevelKind::Penthouse { .. } => "PENTHOUSE",
        StoryLevelKind::Basement { .. } => "BASEMENT",
        StoryLevelKind::Normal => "GENERAL",
    }
}

/// 部材の構造種別（`kind_structure`）。
///
/// 判定は [`squid_n_core::structure_kind::member_structure_kind`] に委ね、
/// ラベル（RC / S / SRC / CFT）をそのまま ST-Bridge の属性値として書き出す。
fn kind_structure(
    model: &squid_n_core::model::Model,
    e: &squid_n_core::model::ElementData,
) -> &'static str {
    squid_n_core::structure_kind::member_structure_kind(model, e).label()
}

/// 部材の ref_vector と軸から `rotate` 角 [deg] を復元する（import の逆変換）。
/// `rotate=0` の基準（水平材は鉛直上、鉛直材はグローバル X）に対する軸まわりの回転角。
fn rotate_of(e: &squid_n_core::model::ElementData, p_i: [f64; 3], p_j: [f64; 3]) -> f64 {
    let axis = {
        let d = [p_j[0] - p_i[0], p_j[1] - p_i[1], p_j[2] - p_i[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if l < 1e-9 {
            return 0.0;
        }
        [d[0] / l, d[1] / l, d[2] / l]
    };
    let base = if axis[2].abs() > 0.99 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let bdot = base[0] * axis[0] + base[1] * axis[1] + base[2] * axis[2];
    let ref0 = normalize([
        base[0] - bdot * axis[0],
        base[1] - bdot * axis[1],
        base[2] - bdot * axis[2],
    ]);
    // 現在の ref_vector を軸へ直交化。
    let r = e.local_axis.ref_vector;
    let rdot = r[0] * axis[0] + r[1] * axis[1] + r[2] * axis[2];
    let refv = normalize([
        r[0] - rdot * axis[0],
        r[1] - rdot * axis[1],
        r[2] - rdot * axis[2],
    ]);
    // ref0→refv の軸まわり符号付き角。angle = atan2((ref0×refv)·axis, ref0·refv)。
    let cross = [
        ref0[1] * refv[2] - ref0[2] * refv[1],
        ref0[2] * refv[0] - ref0[0] * refv[2],
        ref0[0] * refv[1] - ref0[1] * refv[0],
    ];
    let sin = cross[0] * axis[0] + cross[1] * axis[1] + cross[2] * axis[2];
    let cos = ref0[0] * refv[0] + ref0[1] * refv[1] + ref0[2] * refv[2];
    if sin.abs() < 1e-9 && cos.abs() < 1e-9 {
        return 0.0;
    }
    sin.atan2(cos).to_degrees()
}

/// 単位ベクトル。縮退したベクトル（長さ 0）は方向を決められないため、
/// ST-Bridge の既定の参照方向として鉛直上向きを返す。
fn normalize(v: [f64; 3]) -> [f64; 3] {
    squid_n_core::geom::vec3::unit(v).unwrap_or([0.0, 0.0, 1.0])
}

/// 各スラブが参照する `StbSecSlab_RC` の id を決める。
///
/// **同じ内部断面を指すスラブは 1 つの ST-Bridge 断面を共有する**。スラブごとに
/// 断面を出すと、断面を共有する床が N 枚あるモデルで同名の断面が N 個並び、
/// 再取り込みのたびに符号が `S15`・`S15#2`… と増殖する。
/// 断面が未割当のスラブは、そのスラブ専用の id を後ろへ割り当てる。
///
/// 割り当てる id は `base` から連番で、総数はスラブ枚数を超えない
/// （呼び出し側が `base + slabs.len()` を壁断面の開始値として予約している）。
/// `StbSections`（断面定義側）と `StbMembers`（スラブの参照側）が同じ採番を
/// 共有するための単一実装。
fn slab_section_ids(model: &Model, base: u32) -> std::collections::HashMap<SlabId, u32> {
    let mut shared: std::collections::HashMap<SectionId, u32> = std::collections::HashMap::new();
    let mut out: std::collections::HashMap<SlabId, u32> = std::collections::HashMap::new();
    let mut next = base;
    for slab in &model.slabs {
        let id = match slab.section {
            Some(sec) => *shared.entry(sec).or_insert_with(|| {
                let v = next;
                next += 1;
                v
            }),
            None => {
                let v = next;
                next += 1;
                v
            }
        };
        out.insert(slab.id, id);
    }
    out
}

/// スラブ断面（`StbSecSlab_RC`）ブロックを生成する。
///
/// 符号・階・板厚・コンクリート材料はいずれも**スラブへ割り当てた断面**から取り、
/// 同じ断面を指すスラブは 1 つのブロックを共有する（[`slab_section_ids`]）。
/// 断面が未割当のスラブは符号を `S{スラブID}`、板厚を建物一律の
/// `model.slab_thickness` として出力する（解析前チェックが止める状態だが、
/// 書き出し自体は不完全なモデルでも通す）。
fn slab_sections(model: &Model, base: u32) -> String {
    let ids = slab_section_ids(model, base);
    let mut body = String::new();
    let mut written: std::collections::HashSet<u32> = std::collections::HashSet::new();
    for slab in &model.slabs {
        let Some(&s) = ids.get(&slab.id) else {
            continue;
        };
        if !written.insert(s) {
            continue;
        }
        let sec = model.slab_section(slab);
        let t = model
            .slab_thickness_of(slab)
            .unwrap_or(model.slab_thickness);
        let name = sec
            .map(|sc| sc.name.clone())
            .filter(|n| !n.is_empty())
            .unwrap_or_else(|| format!("S{}", sid(slab.id.0)));
        body.push_str(&format!(
            "      <StbSecSlab_RC id=\"{}\" name=\"{}\"{}{}>\n",
            sid(s),
            esc(&name),
            sec.map(slab_floor_attr).unwrap_or_default(),
            sec.map(|sc| concrete_attr(model, sc)).unwrap_or_default(),
        ));
        body.push_str("        <StbSecFigureSlab_RC>\n");
        body.push_str(&format!(
            "          <StbSecSlab_RC_Straight depth=\"{}\"/>\n",
            fmt(t),
        ));
        body.push_str("        </StbSecFigureSlab_RC>\n");
        body.push_str("      </StbSecSlab_RC>\n");
    }
    body
}

/// 壁断面（`StbSecWall_RC`）ブロックを生成する。壁要素ごとに 1 つの断面を出力し、
/// 厚さとコンクリート材料は壁の断面（`elem.section`）から取る（厚さの未設定は 0）。
fn wall_sections(model: &Model, base: u32) -> String {
    let mut body = String::new();
    let mut idx = 0u32;
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Wall | ElementKind::Shell) || e.nodes.len() < 3 {
            continue;
        }
        let s = sid(base + idx);
        let t = e
            .section
            .and_then(|sc| model.sections.get(sc.index()))
            .and_then(|sc| sc.thickness)
            .unwrap_or(0.0);
        let sec = e.section.and_then(|sc| model.sections.get(sc.index()));
        body.push_str(&format!(
            "      <StbSecWall_RC id=\"{}\" name=\"{}\"{}>\n",
            s,
            esc(&format!("W{}", sid(e.id.0))),
            sec.map(|sc| concrete_attr(model, sc)).unwrap_or_default(),
        ));
        body.push_str("        <StbSecFigureWall_RC>\n");
        body.push_str(&format!(
            "          <StbSecWall_RC_Straight thickness=\"{}\"/>\n",
            fmt(t),
        ));
        body.push_str("        </StbSecFigureWall_RC>\n");
        body.push_str("      </StbSecWall_RC>\n");
        idx += 1;
    }
    body
}

pub(super) fn fmt(x: f64) -> String {
    // 整数値は小数点なしで、それ以外は既定の f64 表記で（往復で値が保たれる）。
    if x == x.trunc() && x.is_finite() {
        format!("{}", x as i64)
    } else {
        format!("{x}")
    }
}

pub(super) fn esc(s: &str) -> String {
    // XML 1.0 で表現できない C0 制御文字（タブ/改行/CR 以外の #x00-#x1F）は文字参照でも
    // 表せないため除去する。これをしないと不正な XML を出力してしまう。
    let cleaned: String = s
        .chars()
        .filter(|&c| c == '\t' || c == '\n' || c == '\r' || (c as u32) >= 0x20)
        .collect();
    // & を最初に置換した後で制御空白を文字参照化する（後段で `&` を再エスケープしないため安全）。
    // タブ/改行/CR を文字参照にしないと、XML 属性値正規化（読込側 normalized_value）で
    // 空白 (#x20) に潰れ、属性値（例: 断面名・帯筋グレード）が往復で変化してしまう。
    cleaned
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\t', "&#9;")
        .replace('\n', "&#10;")
        .replace('\r', "&#13;")
}

/// 断面の `floor` を ST-Bridge の `floor` 属性へ（未設定は属性ごと省く）。
fn slab_floor_attr(sec: &squid_n_core::model::Section) -> String {
    match &sec.floor {
        Some(f) => format!(" floor=\"{}\"", esc(f)),
        None => String::new(),
    }
}

/// 断面の主材料の名前を `strength_concrete` 属性へ（未割当は属性ごと省く）。
///
/// ST-Bridge は材料をグレード名で表すため、材料の名前をそのまま出す。かつては
/// スラブ・壁だけ `Fc21` 決め打ちだったが、断面が材料を持つようになったため
/// 根拠のない既定値は置かない。
fn concrete_attr(model: &Model, sec: &squid_n_core::model::Section) -> String {
    match sec
        .material
        .and_then(|mid| model.materials.get(mid.index()))
        .map(|m| m.name.as_str())
        .filter(|n| !n.is_empty())
    {
        Some(name) => format!(" strength_concrete=\"{}\"", esc(name)),
        None => String::new(),
    }
}
