//! 節点単位の断面検定（柱梁接合部・パネルゾーン・冷間成形耐力比・耐震壁）の
//! 入力組み立て（令82条・各構造規準の断面検定）。
//!
//! [`crate::rc::joint`]・[`crate::rc::wall`]・[`crate::steel::panel_zone`]・
//! [`crate::steel::cold_formed`]・[`crate::srrc::panel_zone`] の純関数群に対し、
//! `Model` と部材内力から入力を組み立てて
//! 一括実行する。squid-n-app（GUI）と squid-n-mcp（ヘッドレス）の両方から
//! 呼ばれる共通経路。
//!
//! # 入力の組み立てにおける簡略化（doc 兼申し送り）
//! - 部材種別は部材軸の鉛直成分による幾何判定（app の `member_kind_of` と同じ規則）。
//! - RC 接合部の形状（十字/T/ト/L）は「上下柱の有無 × 取り付く梁の本数(2 以上/1)」で
//!   判定し、加力方向別の区別はしない（全方向の梁をまとめて扱う）。
//! - 冷間成形角形鋼管の軸力比に用いる存在軸力は当該解析ケースの軸力
//!   （`NL + 1.5·NE` の割増は組合せ分離情報が無いため未対応）。
//! - S 造パネルの梁段違い形式（せい差 150mm 以上）は判別せず標準形式で計算する。
//! - S 造パネルの軸力比 n は諸元を採った柱（実効体積 Ve が最小の柱）1 本の
//!   軸力から算定する（本来の「上下階の柱軸力の平均値＋ブレース軸力の鉛直方向
//!   成分」は未対応の簡略化）。パネルの Fy も同じ柱の鋼種から板厚 40mm 区分で解決する
//!   （「下側柱の降伏強さ」の上下判別・実パネル厚の板厚区分は未対応。
//!   tp > 40mm の極厚パネルでは F 値を過大評価しうる点に注意）。
//! - 耐震壁は `SectionShape::RcWall` を割り当てた Wall 要素のみ検定する。
//!   設計用せん断力は等価梁化された壁要素の内力の最大水平せん断成分を用いる
//!   （暫定）。`Model::wall_attrs` に開口面積合計・個別開口寸法・三方スリット
//!   の有無が登録されている場合は以下のとおり配線する。
//!   - まず `Model::multi_opening_mode`（建物一律。既定は `Equivalent`）に
//!     応じて `WallAttr::opening_dims_for(mode)` でモード適用後の個別開口
//!     `(l0,h0)` 列を得る（RC規準（耐震壁の複数開口の等価化））。
//!     - `Equivalent`: 個別開口をそのまま返す（従来どおり）。
//!     - `Envelope`: 位置（`offset`）を持つ開口全体の包絡矩形 1 つに置換
//!       する。位置不明の開口は包絡できないため個別のまま残る。
//!     - `Auto`: 包絡可能な開口対が無くなるまで繰り返し包絡開口を作成し、
//!       残った開口列を返す。
//!   - モード適用後のリストが要素数 1（単一開口）の場合は開口の実寸法
//!     `(l0,h0)` をそのまま用いる（等価開口への置換はしない）。単一開口は
//!     実寸法そのものが RC規準18条の `γ1=1−l0/l`・`γ3=1−h0/h` に直接効く
//!     ため、面積が同じでも辺長比が壁と異なれば等価開口に置換した場合と
//!     検定比が変わる。
//!   - モード適用後のリストが複数残る場合（`Equivalent` で複数開口のまま、
//!     `Auto` で包絡しきれない対が残る、`Envelope` で位置不明の開口が残る、
//!     のいずれか）は、RC規準（耐震壁の複数開口の等価化）に従い、包絡できなく
//!     なった時点の開口状況で「等価開口とする」と同様の判定を行い、
//!     [`crate::wall_opening::equivalent_opening`] で面積総和を保つ単一の
//!     等価開口 `(l0′,h0′)` に統合する。
//!   - `opening_dims_for(mode)` が `None`（個別寸法未入力・合計面積のみ）の
//!     場合は従来どおり壁と同じ辺長比を持つ擬似ペア（面積は
//!     `WallAttr::total_opening_area_for(mode)` で評価。個別開口が無い
//!     ためモードによらず合計面積と同値）から
//!     [`crate::wall_opening::equivalent_opening`] で等価開口を復元する
//!     （後方互換）。
//!   - いずれの経路で得た `(l0′,h0′)` も壁寸法 `(l,h)` を超える場合は
//!     安全側にクランプしたうえで、[`crate::rc::wall::rc_wall_shear_check`] の
//!     `RcWallInput.opening` へ供給する（RC規準18条のせん断耐力検定用の
//!     低減係数 `r=min(γ1,γ2,γ3)`）。開口周比 r0（耐震壁判定用、下記）も
//!     このモード適用後の `(l0′,h0′)` から算定するため、判定・検定の双方が
//!     選択したモードに整合する。
//!   - 一方、耐震壁として扱ってよいか（スリットの有無・壁厚・開口周比 r0）は
//!     [`crate::wall_opening::is_seismic_wall`]（RC規準（耐震壁判定））
//!     で判定し、`false` の壁は本検定自体をスキップする（耐震壁ではない
//!     壁に18条検定を適用しない）。
//!
//!   [`crate::wall_opening`] の `r=1−1.25・r0` は剛性計算専用の低減率であり、
//!   上記 18 条の `r=min(γ1,γ2,γ3)` とは準拠する規定も数式も異なる別物
//!   である。02章の r0/r は耐震壁判定・等価開口の算定にのみ用い、18条の
//!   γ式や `Q1,Q2` の計算に流用してはならない（数式が異なるため結果が
//!   変わる）。
//! - SRC 側柱（[`SectionShape::SrcRect`]）を持つ耐震壁は、側柱の内蔵鉄骨の
//!   ウェブせん断断面積 `As = steel_web_thick・(steel_height − 2・steel_flange_thick)`
//!   と、鋼種（`steel_grade`）から [`crate::steel::steel_f_value_prefix`] で
//!   解決した F 値・[`crate::steel::steel_fs`] による許容せん断応力度 `sfs`
//!   から `steel_shear = sfs・As` を算定し、[`WallSideColumn::steel_shear`]
//!   へ供給する（RC 側柱は 0）。F 値の板厚区分にはフランジ厚とウェブ厚の
//!   大きい方を用いる（他のフォールバック箇所と同じ近似）。
//! - SRC 造柱梁接合部（パネルゾーン、[`crate::srrc::panel_zone::src_panel_zone_check`]）:
//!   柱断面形状が `SrcRect` の節点で検定する。梁の上下主筋間距離 mBd・柱の
//!   左右主筋間距離 mCd は、既存の RC 接合部配線（`beam_j` に
//!   `d − rc_dt(rebar)` を用いる近似）に合わせ、「梁せい／柱幅 −
//!   2・rc_dt(rebar)」（`rc_dt` はかぶり＋帯筋径＋主筋径/2）で近似する
//!   （鉄筋位置の実配置ではなく主筋かぶり情報からの近似）。梁が S 造の
//!   場合は mBd の代わりに sBd（フランジ板厚中心間距離、S パネルゾーンの
//!   `db` 算定と同じ近似）を用いる。柱鉄骨のフランジ重心間距離 sCd は
//!   `steel_height − steel_flange_thick`、接合部鉄骨ウェブ厚 Jtw は柱の
//!   `steel_web_thick`、ヤング係数比 n は [`crate::rc::young_ratio_n`]。
//!   内法階高/階高比 h′/h は情報が無いため 1.0 固定とする（暫定）。

mod cold_formed;
mod common;
mod rc_joint;
mod src_panel;
mod steel_panel;
mod wall;

pub use self::common::ForcesAt;
use self::common::MemberInfo;

// tests.rs は `use super::*` でこれらを参照する（RcWallInput / rc_wall_shear_check /
// equivalent_opening / ElementKind）。抽出により mod.rs 本体では未使用となるため、
// テストビルドでのみ再エクスポートして super::* からの解決を維持する。
#[cfg(test)]
pub(crate) use crate::rc::wall::{rc_wall_shear_check, RcWallInput};
#[cfg(test)]
pub(crate) use crate::wall_opening::equivalent_opening;
#[cfg(test)]
pub(crate) use squid_n_core::model::ElementKind;

use crate::{CheckResult, LoadTerm};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::Model;

/// モデルと部材内力から節点単位の検定を一括実行する。
///
/// 戻り値: `(節点, 種別ラベル, 検定結果)` のリスト。
///
/// 冷間成形角形鋼管の存在軸力に `NL + 1.5・NE` の割増を効かせたい場合は
/// [`collect_joint_checks_with_long`] を使う（本関数は割増なし＝当該ケースの
/// 軸力そのまま）。
pub fn collect_joint_checks(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
    term: LoadTerm,
) -> Vec<(NodeId, String, CheckResult)> {
    collect_joint_checks_with_long(model, member_forces, None, &[], term)
}

/// [`collect_joint_checks`] の長期内力付き版。
///
/// `long_member_forces` に長期（G+P）組合せの部材内力を渡すと、冷間成形
/// 角形鋼管の柱梁耐力比チェックの存在軸力を `N = NL + 1.5・NE`
/// （NE = 当該ケースの軸力 − NL。冷間成形角形鋼管設計・施工マニュアルの
/// Ds/Co = 1.5 割増）で算定する。None の場合は当該ケースの
/// 軸力をそのまま用いる（従来動作）。地震時組合せの結果を渡すことを想定する。
///
/// `panel_moments` には、仕口パネルをモデル化した接合部で解析が出力した
/// せん断モーメント `{MSX, MSY}` [N·mm] を節点ごとに渡す。該当する節点の
/// S 造パネルゾーン検定は、この値の絶対値の大きい方を設計用パネルモーメント
/// `pM` に用いる（節点まわりの釣り合いが解析上厳密に満たされた値）。空スライス、
/// または該当節点が含まれない場合は、従来どおり梁端モーメント・柱せん断から
/// `pM` を組み立てる。**検定の実施自体はパネルのモデル化の有無に依らない**。
pub fn collect_joint_checks_with_long(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
    long_member_forces: Option<&[(ElemId, ForcesAt<'_>)]>,
    panel_moments: &[(NodeId, [f64; 2])],
    term: LoadTerm,
) -> Vec<(NodeId, String, CheckResult)> {
    let mut out = Vec::new();

    // 部材情報の収集（2 節点の梁/柱系要素）
    let mut members: Vec<MemberInfo<'_>> = Vec::new();
    for (eid, forces) in member_forces {
        let Some(elem) = model.elements.iter().find(|e| e.id == *eid) else {
            continue;
        };
        if elem.nodes.len() < 2 {
            continue;
        }
        let sec = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid));
        let mat = elem
            .material
            .and_then(|mid| model.materials.iter().find(|m| m.id == mid));
        let (Some(sec), Some(mat)) = (sec, mat) else {
            continue;
        };
        let (Some(p0), Some(p1)) = (
            model.nodes.get(elem.nodes[0].index()).map(|n| n.coord),
            model.nodes.get(elem.nodes[1].index()).map(|n| n.coord),
        ) else {
            continue;
        };
        let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        let length = (dx * dx + dy * dy + dz * dz).sqrt();
        if length < 1e-9 {
            continue;
        }
        members.push(MemberInfo {
            elem,
            sec,
            mat,
            forces,
            kind: squid_n_core::structure_kind::structure_kind_of(Some(sec), Some(mat.category)),
            ez: (dz / length).abs(),
            length,
        });
    }

    // 各検定種別ごとの配線（`out` への push 順序を厳密に保持する）。
    wall::check_walls(model, member_forces, &members, term, &mut out);

    // ── 節点単位の接合部検定 ─────────────────────────────────────
    for (ni, node) in model.nodes.iter().enumerate() {
        let nid = node.id;
        let _ = ni;
        let cols: Vec<&MemberInfo> = members
            .iter()
            .filter(|m| m.is_column() && m.elem.nodes.contains(&nid))
            .collect();
        let beams: Vec<&MemberInfo> = members
            .iter()
            .filter(|m| m.is_beam_horiz() && m.elem.nodes.contains(&nid))
            .collect();
        if cols.is_empty() || beams.is_empty() {
            continue;
        }

        // 仕口パネルをモデル化した接合部は、解析出力のせん断モーメントのうち
        // 絶対値の大きい方（支配的な面）を設計用パネルモーメントに用いる。
        let panel_moment = panel_moments.iter().find(|(n, _)| *n == nid).map(|(_, m)| {
            if m[0].abs() >= m[1].abs() {
                m[0]
            } else {
                m[1]
            }
        });

        rc_joint::check_rc_joint(&cols, &beams, nid, &mut out);
        steel_panel::check_s_panel(model, &cols, &beams, nid, panel_moment, &mut out);
        src_panel::check_src_panel(&cols, &beams, nid, term, &mut out);
        cold_formed::check_cold_formed(&cols, &beams, nid, long_member_forces, &mut out);
    }

    out
}

#[cfg(test)]
mod tests;
