//! CFT 柱の軸終局耐力・N-M 相互作用の検定ドライバ（CFT指針）。
//!
//! - [`CftUltimateCheck`] — 1 CFT 柱の軸終局検定結果。
//! - [`collect_cft_ultimate_checks`] — モデルの CFT 柱を一括検定する。
//! - [`cft_mu_nm`] — N-M 相互作用による終局曲げ耐力 Mu(N)。

use squid_n_core::ids::ElemId;
use squid_n_core::model::Model;
use squid_n_core::section_shape::SectionShape;

use super::cft::{
    self, cft_axial_ultimate, cft_concrete_buckling_axial, cft_concrete_slenderness, cft_ncu1,
    CftColumnClass,
};
use super::cft_nm::{
    cft_long_medium_column_mu, cft_nk, cft_short_column_mu, CftBendingInput, CftLongMediumInput,
};
use super::geometry::geometric_length;

/// 1 CFT 柱の軸終局検定結果。
#[derive(Clone, Debug)]
pub struct CftUltimateCheck {
    /// 部材 ID。
    pub elem: ElemId,
    /// 柱分類（短柱/中柱/長柱）。
    pub class: CftColumnClass,
    /// 軸圧縮終局耐力 Ncu [N]。
    pub ncu: f64,
    /// 軸引張終局耐力 Ntu [N]。
    pub ntu: f64,
    /// 設計軸力における N-M 相互作用の終局曲げ耐力 Mu [N·mm]
    /// （柱分類に応じて短柱／中柱／長柱の式を用いる）。
    pub mu_nm: f64,
    /// 設計軸力 [N]（圧縮正）。
    pub n_design: f64,
    /// 軸余裕度（圧縮 Ncu/N、引張 Ntu/|N|。N=0 は `f64::INFINITY`）。
    pub axial_margin: f64,
    /// 判定（軸余裕度 ≥ 1.0 で true）。
    pub ok: bool,
    /// 詳細（表示用）。
    pub detail: String,
}

/// CFT 断面（角型/円形）の (円形か, 断面せい D, cA, sA, cI(弱軸), sI(弱軸)) を返す。
fn cft_section_props(shape: &SectionShape) -> Option<(bool, f64, f64, f64, f64, f64)> {
    match *shape {
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => {
            let ch = (height - 2.0 * thick).max(0.0);
            let cw = (width - 2.0 * thick).max(0.0);
            let s_area = shape.calc_area();
            let c_area = ch * cw;
            // 弱軸（せい/幅の小さい方まわり）の断面二次モーメントを座屈用に採用。
            let s_inertia = shape.calc_iy().min(shape.calc_iz());
            let c_iy = cw * ch.powi(3) / 12.0;
            let c_iz = ch * cw.powi(3) / 12.0;
            let c_inertia = c_iy.min(c_iz);
            let d = height.min(width); // 弱軸方向のせい
            Some((false, d, c_area, s_area, c_inertia, s_inertia))
        }
        SectionShape::CftPipe { outer_dia, thick } => {
            let di = (outer_dia - 2.0 * thick).max(0.0);
            let s_area = shape.calc_area();
            let c_area = std::f64::consts::PI * di * di / 4.0;
            let s_inertia = shape.calc_iy();
            let c_inertia = std::f64::consts::PI * di.powi(4) / 64.0;
            Some((true, outer_dia, c_area, s_area, c_inertia, s_inertia))
        }
        _ => None,
    }
}

/// モデルの CFT 柱（`CftBox`/`CftPipe`）について軸終局検定を一括実行する
/// （CFT指針）。
///
/// - `axial_by_elem`: 設計軸力 [N]（**圧縮正**）。なければ軸力 0（安全側）。
/// - 座屈長さ lk は部材の幾何長（K=1 相当）を用いる。鋼管の降伏強さ Fy は
///   材料名の板厚区分から解決した F 値（解決できなければ 235）、ヤング係数は
///   205000 N/mm²（鋼）を用いる。Fc は材料の `fc`（未設定はスキップ）。
pub fn collect_cft_ultimate_checks(
    model: &Model,
    axial_by_elem: &[(ElemId, f64)],
) -> Vec<CftUltimateCheck> {
    let mut out = Vec::new();
    for elem in &model.elements {
        let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let Some(mat) = elem
            .material
            .and_then(|mid| model.materials.get(mid.index()))
        else {
            continue;
        };
        let Some(shape) = sec.shape.as_ref() else {
            continue;
        };
        let Some((circular, d_section, c_area, s_area, c_inertia, s_inertia)) =
            cft_section_props(shape)
        else {
            continue;
        };
        let Some(fc) = mat.fc.filter(|v| *v > 0.0) else {
            continue;
        };
        let thick = match *shape {
            SectionShape::CftBox { thick, .. } | SectionShape::CftPipe { thick, .. } => thick,
            _ => 0.0,
        };
        // プリセット外の直接入力材料は fy を基準強度として用いる（それもなければ 235）。
        let fy = crate::material_strength::steel_f_value_prefix(&mat.name, thick)
            .or(mat.fy)
            .unwrap_or(235.0);
        let lk = geometric_length(elem, model);

        let inp = cft::CftAxialInput {
            circular,
            d_section,
            c_area,
            s_area,
            c_inertia,
            s_inertia,
            fc,
            fy,
            s_young: 205000.0,
            lk,
        };
        let r = cft_axial_ultimate(&inp);
        let n_design = axial_by_elem
            .iter()
            .find(|(id, _)| *id == elem.id)
            .map(|(_, n)| *n)
            .unwrap_or(0.0);

        // N-M 相互作用の終局曲げ耐力 Mu(N)。曲げは強軸（せい方向）で評価する。
        let mu_nm = cft_mu_nm(shape, fc, fy, n_design, lk, false).unwrap_or(0.0);

        let axial_margin = if n_design > 0.0 {
            if r.ncu > 0.0 {
                r.ncu / n_design
            } else {
                0.0
            }
        } else if n_design < 0.0 {
            if r.ntu > 0.0 {
                r.ntu / (-n_design)
            } else {
                0.0
            }
        } else {
            f64::INFINITY
        };
        let class_label = match r.class {
            CftColumnClass::Short => "短柱",
            CftColumnClass::Medium => "中柱",
            CftColumnClass::Long => "長柱",
        };
        let detail = format!(
            "分類={class_label}, Ncu={:.0} N, Ntu={:.0} N, Mu(N-M)={:.0} N·mm, N={:.0} N, \
             lk={:.0} mm, cA={:.0} mm², sA={:.0} mm², Fc={:.1}, Fy={:.1}, 軸余裕度={:.3}",
            r.ncu, r.ntu, mu_nm, n_design, lk, c_area, s_area, fc, fy, axial_margin
        );
        out.push(CftUltimateCheck {
            elem: elem.id,
            class: r.class,
            ncu: r.ncu,
            ntu: r.ntu,
            mu_nm,
            n_design,
            axial_margin,
            ok: axial_margin >= 1.0,
            detail,
        });
    }
    out
}

/// CFT 柱の N-M 相互作用による終局曲げ耐力 `Mu(N)` [N·mm]（CFT指針。
/// 柱分類（短柱／中柱・長柱）に応じた式を選択する）。
///
/// - `n_design`: 設計軸力 [N]（**圧縮正**）。
/// - `fy`: 鋼管の降伏強さ（F 値）[N/mm²]、`lk`: 座屈長さ [mm]。
/// - `weak_axis`: 角形で幅方向（弱軸）まわりの曲げを評価する場合 true
///   （円形は同値。柱分類・軸終局は断面代表せい `d_section` のまま評価する近似）。
///
/// 許容応力度検定の設計用せん断力 `QD1 = ΣcMy/h′` の cMy（=Mu(N)）にも用いる
/// （[`crate::cft`]）。CFT 断面（CftBox/CftPipe）以外・Fc/Fy が 0 以下は `None`。
pub fn cft_mu_nm(
    shape: &SectionShape,
    fc: f64,
    fy: f64,
    n_design: f64,
    lk: f64,
    weak_axis: bool,
) -> Option<f64> {
    if fc <= 0.0 || fy <= 0.0 {
        return None;
    }
    let (circular, d_section, c_area, s_area, c_inertia, s_inertia) = cft_section_props(shape)?;
    let (bd, bb, thick) = match *shape {
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => {
            if weak_axis {
                (width, height, thick)
            } else {
                (height, width, thick)
            }
        }
        SectionShape::CftPipe { outer_dia, thick } => (outer_dia, outer_dia, thick),
        _ => return None,
    };
    let inp = cft::CftAxialInput {
        circular,
        d_section,
        c_area,
        s_area,
        c_inertia,
        s_inertia,
        fc,
        fy,
        s_young: 205000.0,
        lk,
    };
    let r = cft_axial_ultimate(&inp);
    let bending = CftBendingInput {
        circular,
        d_steel: bd,
        b_steel: bb,
        c_d: (bd - 2.0 * thick).max(0.0),
        c_b: (bb - 2.0 * thick).max(0.0),
        t: thick,
        fc,
        fy,
    };
    let mu = match r.class {
        CftColumnClass::Short => cft_short_column_mu(&bending, n_design, cft_ncu1(&inp), r.ntu),
        CftColumnClass::Medium | CftColumnClass::Long => cft_long_medium_column_mu(
            &CftLongMediumInput {
                bending,
                is_long: r.class == CftColumnClass::Long,
                c_ncr: cft_concrete_buckling_axial(c_inertia, c_area, fc, lk),
                c_lambda1: cft_concrete_slenderness(c_inertia, c_area, fc, lk),
                nk: cft_nk(c_inertia, s_inertia, 205000.0, fc, lk),
                ncu_axial: r.ncu,
                ntu: r.ntu,
            },
            n_design,
        ),
    };
    Some(mu)
}
