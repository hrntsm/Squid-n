//! 質点系モデルの生成（静解析の線形ばね／増分のトリリニア）。

use crate::error::{JobError, JobResult};
use squid_n_core::model::{Layer, Model};
use squid_n_core::units::GRAVITY_MM_S2;
use squid_n_design_jp::secondary::eccentricity::{
    append_misc_wall_stiffnesses, center_of_mass, center_of_rigidity, eccentricity,
};
use squid_n_design_jp::secondary::eccentricity_analysis::column_stiffnesses_from_analysis;
use squid_n_design_jp::secondary::story_columns::story_columns;
use squid_n_element::transform::LocalFrame;
use squid_n_solver::analysis::SeismicDir;
use squid_n_solver::linear::StaticOnce;
use squid_n_solver::lumped_mass::{
    build_lumped_mass_model, LumpedMassModel, LumpedMassType, LumpedStiffnessSource, StickDim,
    StorySpatial, StoryStick, StoryTrilinear,
};
use squid_n_solver::pushover::{story_reference_node, PushoverResult};

/// 質点系生成に渡す解析結果。
pub struct LumpedMassBuildInput<'a> {
    pub model: &'a Model,
    pub dim: StickDim,
    pub source: LumpedStiffnessSource,
    pub dir: SeismicDir,
    pub nonlinear: bool,
    pub secant_ratio: f64,
    pub res_x: Option<&'a StaticOnce>,
    pub res_y: Option<&'a StaticOnce>,
    pub po_x: Option<&'a PushoverResult>,
    pub po_y: Option<&'a PushoverResult>,
}

pub fn build_lumped_mass(inp: LumpedMassBuildInput<'_>) -> JobResult<LumpedMassModel> {
    match inp.dim {
        StickDim::Planar => build_planar(inp),
        StickDim::Spatial => build_spatial(inp),
    }
}

fn build_planar(inp: LumpedMassBuildInput<'_>) -> JobResult<LumpedMassModel> {
    if inp.nonlinear {
        let po = pushover_of(&inp, inp.dir)?;
        let mut lm = build_lumped_mass_model(
            inp.model,
            po,
            LumpedMassType::EquivalentShear,
            inp.secant_ratio,
        );
        lm.dim = StickDim::Planar;
        lm.stiffness_source = inp.source;
        lm.dir = inp.dir;
        lm.nonlinear = true;
        Ok(lm)
    } else {
        let res = static_of(&inp, inp.dir)?;
        let k = story_stiffness(inp.model, res, inp.dir, inp.source)?;
        let mut stories = Vec::new();
        for (layer, &ki) in inp.model.layers().iter().zip(k.iter()) {
            stories.push(StoryStick {
                story: layer.bottom,
                mass: layer_mass(inp.model, layer),
                height: layer.height.max(0.0),
                skeleton: StoryTrilinear::elastic(ki),
            });
        }
        let mut lm = LumpedMassModel::from_stories(LumpedMassType::EquivalentShear, stories);
        lm.stiffness_source = inp.source;
        lm.dir = inp.dir;
        lm.nonlinear = false;
        Ok(lm)
    }
}

fn build_spatial(inp: LumpedMassBuildInput<'_>) -> JobResult<LumpedMassModel> {
    let res_x = inp.res_x.ok_or_else(|| {
        JobError::InvalidInput("3次元質点系には地震静的 EX の結果が必要です".into())
    })?;
    let res_y = inp.res_y.ok_or_else(|| {
        JobError::InvalidInput("3次元質点系には地震静的 EY の結果が必要です".into())
    })?;

    let kx = story_stiffness(inp.model, res_x, SeismicDir::X, inp.source)?;
    let ky = story_stiffness(inp.model, res_y, SeismicDir::Y, inp.source)?;

    let sk_x;
    let sk_y;
    if inp.nonlinear {
        let po_x = inp.po_x.ok_or_else(|| {
            JobError::InvalidInput("3次元の非線形質点系には X 方向の増分解析結果が必要です".into())
        })?;
        let po_y = inp.po_y.ok_or_else(|| {
            JobError::InvalidInput("3次元の非線形質点系には Y 方向の増分解析結果が必要です".into())
        })?;
        sk_x = build_lumped_mass_model(
            inp.model,
            po_x,
            LumpedMassType::EquivalentShear,
            inp.secant_ratio,
        );
        sk_y = build_lumped_mass_model(
            inp.model,
            po_y,
            LumpedMassType::EquivalentShear,
            inp.secant_ratio,
        );
    } else {
        sk_x = LumpedMassModel::from_stories(LumpedMassType::EquivalentShear, Vec::new());
        sk_y = LumpedMassModel::from_stories(LumpedMassType::EquivalentShear, Vec::new());
    }

    let layers = inp.model.layers();
    if kx.len() != layers.len() || ky.len() != layers.len() {
        return Err(JobError::Solve("層剛性の層数がモデルと一致しません".into()));
    }

    let mut stories = Vec::with_capacity(layers.len());
    let mut spatial = Vec::with_capacity(layers.len());
    for (i, layer) in layers.iter().enumerate() {
        let kxi = kx[i];
        let kyi = ky[i];
        let mut cols = column_stiffnesses_from_analysis(inp.model, layer.top, res_x, res_y);
        append_misc_wall_stiffnesses(inp.model, layer.top, &mut cols);
        let cor = center_of_rigidity(&cols);
        let com = center_of_mass(inp.model, layer.top);
        let kr = eccentricity(&cols, com, cor).kr;
        let j = floor_j(inp.model, layer.top);
        if j <= 0.0 {
            return Err(JobError::InvalidInput(format!(
                "階 {} の回転慣性 J が未設定です。剛床のある階で 3 次元質点系を実行してください",
                layer.name
            )));
        }
        let skeleton_x = if inp.nonlinear {
            sk_x.stories
                .get(i)
                .map(|s| s.skeleton)
                .unwrap_or_else(|| StoryTrilinear::elastic(kxi))
        } else {
            StoryTrilinear::elastic(kxi)
        };
        let skeleton_y = if inp.nonlinear {
            sk_y.stories
                .get(i)
                .map(|s| s.skeleton)
                .unwrap_or_else(|| StoryTrilinear::elastic(kyi))
        } else {
            StoryTrilinear::elastic(kyi)
        };
        let skeleton = match inp.dir {
            SeismicDir::X => skeleton_x,
            SeismicDir::Y => skeleton_y,
        };
        stories.push(StoryStick {
            story: layer.bottom,
            mass: layer_mass(inp.model, layer),
            height: layer.height.max(0.0),
            skeleton,
        });
        spatial.push(StorySpatial {
            j,
            mass_xy: com,
            rigidity_xy: cor,
            k1_x: skeleton_x.k1,
            k1_y: skeleton_y.k1,
            kr,
            skeleton_x,
            skeleton_y,
        });
    }

    Ok(LumpedMassModel {
        model_type: LumpedMassType::EquivalentShear,
        stories,
        dim: StickDim::Spatial,
        stiffness_source: inp.source,
        dir: inp.dir,
        nonlinear: inp.nonlinear,
        spatial,
    })
}

fn static_of<'a>(inp: &'a LumpedMassBuildInput<'a>, dir: SeismicDir) -> JobResult<&'a StaticOnce> {
    let (res, name) = match dir {
        SeismicDir::X => (inp.res_x, "EX"),
        SeismicDir::Y => (inp.res_y, "EY"),
    };
    res.ok_or_else(|| {
        JobError::InvalidInput(format!(
            "線形質点系には地震静的 {name} の結果が必要です。静的解析を実行してください"
        ))
    })
}

fn pushover_of<'a>(
    inp: &'a LumpedMassBuildInput<'a>,
    dir: SeismicDir,
) -> JobResult<&'a PushoverResult> {
    let (po, name) = match dir {
        SeismicDir::X => (inp.po_x, "X"),
        SeismicDir::Y => (inp.po_y, "Y"),
    };
    po.ok_or_else(|| {
        JobError::InvalidInput(format!(
            "非線形質点系には {name} 方向の増分解析結果が必要です。増分解析を実行してください"
        ))
    })
}

fn layer_mass(model: &Model, layer: &Layer) -> f64 {
    match layer.weight {
        Some(w) if w > 0.0 => w / GRAVITY_MM_S2,
        _ => layer
            .node_ids
            .iter()
            .filter_map(|nid| model.nodes.get(nid.index()))
            .filter_map(|n| n.mass)
            .map(|m| m[0].max(m[1]))
            .sum(),
    }
}

fn floor_j(model: &Model, story: squid_n_core::ids::StoryId) -> f64 {
    model
        .diaphragms_of(story)
        .next()
        .and_then(|d| model.nodes.get(d.master.index()))
        .and_then(|n| n.mass)
        .map(|m| m[5])
        .unwrap_or(0.0)
}

fn story_stiffness(
    model: &Model,
    res: &StaticOnce,
    dir: SeismicDir,
    source: LumpedStiffnessSource,
) -> JobResult<Vec<f64>> {
    let layers = model.layers();
    let mut k = Vec::with_capacity(layers.len());
    match source {
        LumpedStiffnessSource::StoryQd => {
            let drifts = story_drifts(model, res, dir);
            let shears = story_shears(model, res, dir);
            if drifts.len() != shears.len() {
                return Err(JobError::Solve(
                    "層せん断と層間変位の層数が一致しません".into(),
                ));
            }
            for (q, d) in shears.iter().zip(drifts.iter()) {
                if d.abs() < 1e-9 {
                    return Err(JobError::InvalidInput(
                        "層間変位がほぼ 0 のため層剛性 K=Q/δ を算定できません".into(),
                    ));
                }
                k.push(q.abs() / d.abs());
            }
        }
        LumpedStiffnessSource::ColumnKi => {
            // 反対方向の結果が無くても、当該方向の ki 合計は res を X/Y の両方に
            // 渡して柱剛性を拾う（使わない方向は 0 になり得る）。
            let dummy = res;
            for layer in &layers {
                let cols = match dir {
                    SeismicDir::X => column_stiffnesses_from_analysis(model, layer.top, res, dummy),
                    SeismicDir::Y => column_stiffnesses_from_analysis(model, layer.top, dummy, res),
                };
                let sum: f64 = cols
                    .iter()
                    .map(|c| match dir {
                        SeismicDir::X => c.dx,
                        SeismicDir::Y => c.dy,
                    })
                    .sum();
                if sum <= 0.0 {
                    return Err(JobError::InvalidInput(format!(
                        "階 {} の柱剛性合計が 0 です",
                        layer.name
                    )));
                }
                k.push(sum);
            }
        }
    }
    Ok(k)
}

fn story_drifts(model: &Model, res: &StaticOnce, dir: SeismicDir) -> Vec<f64> {
    let dir_idx = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };
    let disp_of = |sid: squid_n_core::ids::StoryId| -> f64 {
        model
            .stories
            .get(sid.index())
            .and_then(|story| story_reference_node(model, story))
            .and_then(|nid| res.disp.get(nid.index()))
            .map(|d| d[dir_idx])
            .unwrap_or(0.0)
    };
    model
        .layers()
        .iter()
        .map(|l| disp_of(l.top) - disp_of(l.bottom))
        .collect()
}

fn story_shears(model: &Model, res: &StaticOnce, dir: SeismicDir) -> Vec<f64> {
    let dir_idx = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };
    use std::collections::HashMap;
    let forces: HashMap<_, _> = res.member_forces.iter().map(|(id, f)| (*id, f)).collect();
    let layers = model.layers();
    let n = layers.len();
    let mut level_force = vec![0.0; n];
    for layer in &layers {
        for col in story_columns(model, layer.top) {
            let Some(elem) = model.elements.iter().find(|e| e.id == col.top_elem) else {
                continue;
            };
            let Some(mf) = forces.get(&elem.id) else {
                continue;
            };
            let Some(&(_, local)) = mf.at.first() else {
                continue;
            };
            let p0 = model.nodes[elem.nodes[0].index()].coord;
            let p1 = model.nodes[elem.nodes[1].index()].coord;
            let frame = LocalFrame::from_nodes(p0, p1, elem.local_axis.ref_vector);
            let (n_ax, qy, qz) = (local[0], local[1], local[2]);
            let g = [
                n_ax * frame.rot[0][0] + qy * frame.rot[1][0] + qz * frame.rot[2][0],
                n_ax * frame.rot[0][1] + qy * frame.rot[1][1] + qz * frame.rot[2][1],
                n_ax * frame.rot[0][2] + qy * frame.rot[1][2] + qz * frame.rot[2][2],
            ];
            level_force[layer.index] += g[dir_idx];
        }
    }
    let mut shear = vec![0.0; n];
    let mut acc = 0.0;
    for i in (0..n).rev() {
        acc += level_force[i];
        shear[i] = acc;
    }
    shear
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::model::Model;

    fn input<'a>(
        model: &'a Model,
        dim: StickDim,
        dir: SeismicDir,
        nonlinear: bool,
    ) -> LumpedMassBuildInput<'a> {
        LumpedMassBuildInput {
            model,
            dim,
            source: LumpedStiffnessSource::StoryQd,
            dir,
            nonlinear,
            secant_ratio: 0.75,
            res_x: None,
            res_y: None,
            po_x: None,
            po_y: None,
        }
    }

    #[test]
    fn planar_linear_requires_static() {
        let model = Model::default();
        let err =
            build_lumped_mass(input(&model, StickDim::Planar, SeismicDir::X, false)).unwrap_err();
        assert!(err.to_string().contains("EX"), "{err}");
    }

    #[test]
    fn planar_nonlinear_requires_pushover() {
        let model = Model::default();
        let err =
            build_lumped_mass(input(&model, StickDim::Planar, SeismicDir::Y, true)).unwrap_err();
        assert!(err.to_string().contains("Y"), "{err}");
    }

    #[test]
    fn spatial_requires_ex_and_ey() {
        let model = Model::default();
        let err =
            build_lumped_mass(input(&model, StickDim::Spatial, SeismicDir::X, false)).unwrap_err();
        assert!(err.to_string().contains("EX"), "{err}");
    }
}
