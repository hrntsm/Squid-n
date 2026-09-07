use arrow::array::{Float64Array, UInt32Array, UInt64Array};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use std::sync::Arc;

pub fn nodal_disp_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("node_id", DataType::UInt32, false),
        Field::new("ux", DataType::Float64, false),
        Field::new("uy", DataType::Float64, false),
        Field::new("uz", DataType::Float64, false),
        Field::new("rx", DataType::Float64, false),
        Field::new("ry", DataType::Float64, false),
        Field::new("rz", DataType::Float64, false),
    ]))
}

pub fn nodal_disp_batch(node_ids: &[u32], disp: &[[f64; 6]]) -> arrow::error::Result<RecordBatch> {
    let n = node_ids.len();
    let id_arr = UInt32Array::from(node_ids.to_vec());
    let mut ux = Vec::with_capacity(n);
    let mut uy = Vec::with_capacity(n);
    let mut uz = Vec::with_capacity(n);
    let mut rx = Vec::with_capacity(n);
    let mut ry = Vec::with_capacity(n);
    let mut rz = Vec::with_capacity(n);
    for d in disp {
        ux.push(d[0]);
        uy.push(d[1]);
        uz.push(d[2]);
        rx.push(d[3]);
        ry.push(d[4]);
        rz.push(d[5]);
    }
    RecordBatch::try_new(
        nodal_disp_schema(),
        vec![
            Arc::new(id_arr),
            Arc::new(Float64Array::from(ux)),
            Arc::new(Float64Array::from(uy)),
            Arc::new(Float64Array::from(uz)),
            Arc::new(Float64Array::from(rx)),
            Arc::new(Float64Array::from(ry)),
            Arc::new(Float64Array::from(rz)),
        ],
    )
}

pub fn member_force_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("elem_id", DataType::UInt32, false),
        Field::new("pos", DataType::Float64, false),
        Field::new("n", DataType::Float64, false),
        Field::new("qy", DataType::Float64, false),
        Field::new("qz", DataType::Float64, false),
        Field::new("mx", DataType::Float64, false),
        Field::new("my", DataType::Float64, false),
        Field::new("mz", DataType::Float64, false),
    ]))
}

/// 部材内力（評価位置別）を RecordBatch 化する。
/// `rows`: (要素ID, 評価位置 0..1, [N,Qy,Qz,Mx,My,Mz])
pub fn member_force_batch(rows: &[(u32, f64, [f64; 6])]) -> arrow::error::Result<RecordBatch> {
    let n = rows.len();
    let mut elem = Vec::with_capacity(n);
    let mut pos = Vec::with_capacity(n);
    let mut cols: [Vec<f64>; 6] = Default::default();
    for (e, p, f) in rows {
        elem.push(*e);
        pos.push(*p);
        for (c, v) in cols.iter_mut().zip(f.iter()) {
            c.push(*v);
        }
    }
    let [n_, qy, qz, mx, my, mz] = cols;
    RecordBatch::try_new(
        member_force_schema(),
        vec![
            Arc::new(UInt32Array::from(elem)),
            Arc::new(Float64Array::from(pos)),
            Arc::new(Float64Array::from(n_)),
            Arc::new(Float64Array::from(qy)),
            Arc::new(Float64Array::from(qz)),
            Arc::new(Float64Array::from(mx)),
            Arc::new(Float64Array::from(my)),
            Arc::new(Float64Array::from(mz)),
        ],
    )
}

pub fn modal_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("mode", DataType::UInt32, false),
        Field::new("period", DataType::Float64, false),
        Field::new("omega2", DataType::Float64, false),
        Field::new("part_x", DataType::Float64, false),
        Field::new("part_y", DataType::Float64, false),
        Field::new("part_z", DataType::Float64, false),
        Field::new("eff_x", DataType::Float64, false),
        Field::new("eff_y", DataType::Float64, false),
        Field::new("eff_z", DataType::Float64, false),
    ]))
}

pub fn time_history_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("step", DataType::UInt64, false),
        Field::new("time", DataType::Float64, false),
        Field::new("node_id", DataType::UInt32, false),
        Field::new("ux", DataType::Float64, false),
        Field::new("uy", DataType::Float64, false),
        Field::new("uz", DataType::Float64, false),
        Field::new("rx", DataType::Float64, false),
        Field::new("ry", DataType::Float64, false),
        Field::new("rz", DataType::Float64, false),
    ]))
}

/// モーダル結果（固有周期・刺激係数・有効質量）を RecordBatch 化する。
pub fn modal_batch(
    period: &[f64],
    omega2: &[f64],
    participation: &[[f64; 3]],
    effective_mass: &[[f64; 3]],
) -> arrow::error::Result<RecordBatch> {
    let n = period.len();
    let mode: Vec<u32> = (0..n as u32).collect();
    let mut part: [Vec<f64>; 3] = Default::default();
    let mut eff: [Vec<f64>; 3] = Default::default();
    for i in 0..n {
        for d in 0..3 {
            part[d].push(participation.get(i).map(|p| p[d]).unwrap_or(0.0));
            eff[d].push(effective_mass.get(i).map(|e| e[d]).unwrap_or(0.0));
        }
    }
    let [px, py, pz] = part;
    let [ex, ey, ez] = eff;
    RecordBatch::try_new(
        modal_schema(),
        vec![
            Arc::new(UInt32Array::from(mode)),
            Arc::new(Float64Array::from(period.to_vec())),
            Arc::new(Float64Array::from(omega2.to_vec())),
            Arc::new(Float64Array::from(px)),
            Arc::new(Float64Array::from(py)),
            Arc::new(Float64Array::from(pz)),
            Arc::new(Float64Array::from(ex)),
            Arc::new(Float64Array::from(ey)),
            Arc::new(Float64Array::from(ez)),
        ],
    )
}

pub fn time_history_batch(
    step: u64,
    time: f64,
    node_ids: &[u32],
    disp: &[[f64; 6]],
) -> arrow::error::Result<RecordBatch> {
    let n = node_ids.len();
    let step_arr = UInt64Array::from(vec![step; n]);
    let time_arr = Float64Array::from(vec![time; n]);
    let id_arr = UInt32Array::from(node_ids.to_vec());
    let mut ux = Vec::with_capacity(n);
    let mut uy = Vec::with_capacity(n);
    let mut uz = Vec::with_capacity(n);
    let mut rx = Vec::with_capacity(n);
    let mut ry = Vec::with_capacity(n);
    let mut rz = Vec::with_capacity(n);
    for d in disp {
        ux.push(d[0]);
        uy.push(d[1]);
        uz.push(d[2]);
        rx.push(d[3]);
        ry.push(d[4]);
        rz.push(d[5]);
    }
    RecordBatch::try_new(
        time_history_schema(),
        vec![
            Arc::new(step_arr),
            Arc::new(time_arr),
            Arc::new(id_arr),
            Arc::new(Float64Array::from(ux)),
            Arc::new(Float64Array::from(uy)),
            Arc::new(Float64Array::from(uz)),
            Arc::new(Float64Array::from(rx)),
            Arc::new(Float64Array::from(ry)),
            Arc::new(Float64Array::from(rz)),
        ],
    )
}
