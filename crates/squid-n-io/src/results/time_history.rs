use arrow::array::{BooleanArray, UInt32Array, UInt64Array};
use arrow::record_batch::RecordBatch;

use super::{read_all, time_history_batch, time_history_schema, ParquetWriter, ResultWriter};

pub struct TimeHistoryWriter {
    writer: ParquetWriter,
    step: u64,
}

impl TimeHistoryWriter {
    pub fn create(path: &str) -> parquet::errors::Result<Self> {
        let schema = time_history_schema();
        Ok(Self {
            writer: ParquetWriter::create(path, schema)?,
            step: 0,
        })
    }

    pub fn write_step(
        &mut self,
        time: f64,
        node_ids: &[u32],
        disp: &[[f64; 6]],
    ) -> arrow::error::Result<()> {
        let batch = time_history_batch(self.step, time, node_ids, disp)?;
        self.writer
            .write_rows(&batch)
            .map_err(|e| arrow::error::ArrowError::IoError(e.to_string(), e))?;
        self.step += 1;
        Ok(())
    }

    pub fn finish(self) -> std::io::Result<()> {
        Box::new(self.writer).finish()
    }

    pub fn current_step(&self) -> u64 {
        self.step
    }
}

pub fn read_time_history_range(
    path: &str,
    step_range: Option<(u64, u64)>,
    node_filter: Option<&[u32]>,
) -> parquet::errors::Result<Vec<RecordBatch>> {
    let batches = read_all(path)?;
    if step_range.is_none() && node_filter.is_none() {
        return Ok(batches);
    }

    let node_set: Option<std::collections::HashSet<u32>> =
        node_filter.map(|ids| ids.iter().copied().collect());

    let mut result = Vec::new();
    for batch in batches {
        let step_col = batch
            .column(0)
            .as_any()
            .downcast_ref::<UInt64Array>()
            .expect("step column should be UInt64");
        let node_col = batch
            .column(2)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .expect("node_id column should be UInt32");

        let num_rows = batch.num_rows();
        let mut keep = vec![true; num_rows];

        if let Some((start, end)) = step_range {
            for (i, k) in keep.iter_mut().enumerate().take(num_rows) {
                let s = step_col.value(i);
                if s < start || s > end {
                    *k = false;
                }
            }
        }

        if let Some(ref ids) = node_set {
            for (i, k) in keep.iter_mut().enumerate().take(num_rows) {
                if *k && !ids.contains(&node_col.value(i)) {
                    *k = false;
                }
            }
        }

        let mask = BooleanArray::from(keep);
        let filtered = arrow::compute::filter_record_batch(&batch, &mask)
            .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))?;
        if filtered.num_rows() > 0 {
            result.push(filtered);
        }
    }

    Ok(result)
}
