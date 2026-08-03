use arrow::datatypes::Schema;
use arrow::record_batch::RecordBatch;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::arrow::arrow_writer::ArrowWriter;
use parquet::arrow::ProjectionMask;
use parquet::file::properties::WriterProperties;
use std::fs::File;
use std::sync::Arc;

use super::ResultWriter;

pub struct ParquetWriter {
    inner: ArrowWriter<File>,
    rows: u64,
}

impl ParquetWriter {
    pub fn create(path: &str, schema: Arc<Schema>) -> parquet::errors::Result<Self> {
        let file = File::create(path)?;
        let props = WriterProperties::builder()
            .set_max_row_group_row_count(Some(64 * 1024))
            .build();
        Ok(Self {
            inner: ArrowWriter::try_new(file, schema, Some(props))?,
            rows: 0,
        })
    }
}

impl ResultWriter for ParquetWriter {
    fn write_rows(&mut self, batch: &RecordBatch) -> std::io::Result<()> {
        self.rows += batch.num_rows() as u64;
        self.inner.write(batch).map_err(std::io::Error::other)
    }

    fn finish(self: Box<Self>) -> std::io::Result<()> {
        self.inner.close().map_err(std::io::Error::other)?;
        Ok(())
    }
}

pub fn read_partial(path: &str, columns: Vec<usize>) -> parquet::errors::Result<Vec<RecordBatch>> {
    let file = File::open(path)?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;
    let mask = ProjectionMask::roots(builder.parquet_schema(), columns);
    let reader = builder.with_projection(mask).build()?;
    reader
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))
}

pub fn read_all(path: &str) -> parquet::errors::Result<Vec<RecordBatch>> {
    let file = File::open(path)?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;
    let reader = builder.build()?;
    reader
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))
}
