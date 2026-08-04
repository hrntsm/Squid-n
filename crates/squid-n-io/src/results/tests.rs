use super::*;

#[test]
fn test_parquet_roundtrip() {
    let dir = std::env::temp_dir();
    let path = dir.join("p2_test_nodal.parquet");
    let path_str = path.to_str().unwrap();

    {
        let mut writer = ParquetWriter::create(path_str, nodal_disp_schema()).unwrap();
        let batch = nodal_disp_batch(
            &[0, 1, 2],
            &[
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [1.5, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 2.0, 0.0, 0.0, 0.0, 0.0],
            ],
        )
        .unwrap();
        writer.write_rows(&batch).unwrap();
        Box::new(writer).finish().unwrap();
    }

    let batches = read_all(path_str).unwrap();
    assert_eq!(batches.len(), 1);
    let batch = &batches[0];
    assert_eq!(batch.num_rows(), 3);
    assert_eq!(batch.num_columns(), 7);
}

#[test]
fn test_modal_roundtrip_partial() {
    let dir = std::env::temp_dir();
    let path = dir.join("p2_test_modal.parquet");
    let path_str = path.to_str().unwrap();
    {
        let mut writer = ParquetWriter::create(path_str, modal_schema()).unwrap();
        let batch = modal_batch(
            &[0.3215, 0.1228],
            &[382.0, 2618.0],
            &[[1.0, 0.0, 0.0], [0.5, 0.0, 0.0]],
            &[[1.894, 0.0, 0.0], [0.106, 0.0, 0.0]],
        )
        .unwrap();
        writer.write_rows(&batch).unwrap();
        Box::new(writer).finish().unwrap();
    }
    // 部分読出し: period 列(=1)のみ射影
    let batches = read_partial(path_str, vec![1]).unwrap();
    let b = &batches[0];
    assert_eq!(b.num_columns(), 1);
    assert_eq!(b.num_rows(), 2);
}

#[test]
fn test_member_force_roundtrip() {
    let dir = std::env::temp_dir();
    let path = dir.join("p2_test_member.parquet");
    let path_str = path.to_str().unwrap();
    {
        let mut writer = ParquetWriter::create(path_str, member_force_schema()).unwrap();
        let batch = member_force_batch(&[
            (1, 0.0, [100.0, 5.0, 0.0, 0.0, 0.0, 200.0]),
            (1, 1.0, [100.0, 5.0, 0.0, 0.0, 0.0, -200.0]),
        ])
        .unwrap();
        writer.write_rows(&batch).unwrap();
        Box::new(writer).finish().unwrap();
    }
    let batches = read_all(path_str).unwrap();
    assert_eq!(batches[0].num_rows(), 2);
    assert_eq!(batches[0].num_columns(), 8);
}

#[test]
fn test_time_history_schema_fields() {
    let schema = time_history_schema();
    assert_eq!(schema.fields().len(), 9);
    assert_eq!(schema.field(0).name(), "step");
    assert_eq!(schema.field(1).name(), "time");
    assert_eq!(schema.field(2).name(), "node_id");
    assert_eq!(schema.field(3).name(), "ux");
    assert_eq!(schema.field(4).name(), "uy");
    assert_eq!(schema.field(5).name(), "uz");
    assert_eq!(schema.field(6).name(), "rx");
    assert_eq!(schema.field(7).name(), "ry");
    assert_eq!(schema.field(8).name(), "rz");
    assert_eq!(schema.field(0).data_type(), &DataType::UInt64);
    assert_eq!(schema.field(1).data_type(), &DataType::Float64);
    assert_eq!(schema.field(2).data_type(), &DataType::UInt32);
}

#[test]
fn test_time_history_batch_values() {
    let batch = time_history_batch(
        5,
        2.5,
        &[10, 20],
        &[
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            [1.1, 1.2, 1.3, 1.4, 1.5, 1.6],
        ],
    )
    .unwrap();
    assert_eq!(batch.num_rows(), 2);
    assert_eq!(batch.num_columns(), 9);

    let step_col = batch
        .column(0)
        .as_any()
        .downcast_ref::<UInt64Array>()
        .unwrap();
    assert_eq!(step_col.value(0), 5);
    assert_eq!(step_col.value(1), 5);

    let time_col = batch
        .column(1)
        .as_any()
        .downcast_ref::<Float64Array>()
        .unwrap();
    assert!((time_col.value(0) - 2.5).abs() < f64::EPSILON);

    let node_col = batch
        .column(2)
        .as_any()
        .downcast_ref::<UInt32Array>()
        .unwrap();
    assert_eq!(node_col.value(0), 10);
    assert_eq!(node_col.value(1), 20);

    let ux_col = batch
        .column(3)
        .as_any()
        .downcast_ref::<Float64Array>()
        .unwrap();
    assert!((ux_col.value(1) - 1.1).abs() < f64::EPSILON);
}

#[test]
fn test_time_history_write_read_roundtrip() {
    let dir = std::env::temp_dir();
    let path = dir.join("p6_th_roundtrip.parquet");
    let path_str = path.to_str().unwrap();

    {
        let mut writer = TimeHistoryWriter::create(path_str).unwrap();
        let nodes = [1u32, 2, 3];
        for s in 0..3 {
            let t = s as f64 * 0.1;
            let d = [
                [t + 0.01, t + 0.02, t + 0.03, t + 0.04, t + 0.05, t + 0.06],
                [t + 0.11, t + 0.12, t + 0.13, t + 0.14, t + 0.15, t + 0.16],
                [t + 0.21, t + 0.22, t + 0.23, t + 0.24, t + 0.25, t + 0.26],
            ];
            writer.write_step(t, &nodes, &d).unwrap();
        }
        assert_eq!(writer.current_step(), 3);
        writer.finish().unwrap();
    }

    let batches = read_all(path_str).unwrap();
    let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
    assert_eq!(total_rows, 9);
}

#[test]
fn test_time_history_partial_read_step_range() {
    let dir = std::env::temp_dir();
    let path = dir.join("p6_th_step_range.parquet");
    let path_str = path.to_str().unwrap();

    {
        let mut writer = TimeHistoryWriter::create(path_str).unwrap();
        for s in 0..3 {
            writer
                .write_step(s as f64 * 0.1, &[1, 2], &[[0.1; 6], [0.2; 6]])
                .unwrap();
        }
        writer.finish().unwrap();
    }

    let batches = read_time_history_range(path_str, Some((1, 2)), None).unwrap();
    let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
    assert_eq!(total_rows, 4);

    for batch in &batches {
        let step_col = batch
            .column(0)
            .as_any()
            .downcast_ref::<UInt64Array>()
            .unwrap();
        for i in 0..batch.num_rows() {
            let s = step_col.value(i);
            assert!((1..=2).contains(&s));
        }
    }
}

#[test]
fn test_time_history_partial_read_node_filter() {
    let dir = std::env::temp_dir();
    let path = dir.join("p6_th_node_filter.parquet");
    let path_str = path.to_str().unwrap();

    {
        let mut writer = TimeHistoryWriter::create(path_str).unwrap();
        writer
            .write_step(0.0, &[1, 2, 3], &[[0.1; 6], [0.2; 6], [0.3; 6]])
            .unwrap();
        writer.finish().unwrap();
    }

    let batches = read_time_history_range(path_str, None, Some(&[1u32])).unwrap();
    let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
    assert_eq!(total_rows, 1);

    let batch = &batches[0];
    let node_col = batch
        .column(2)
        .as_any()
        .downcast_ref::<UInt32Array>()
        .unwrap();
    assert_eq!(node_col.value(0), 1);
}

#[test]
fn test_fs_result_store_writer_and_manifest() {
    let dir = std::env::temp_dir().join("p8_fsrs_basic");
    let _ = std::fs::remove_dir_all(&dir);
    let mut store = FsResultStore::open(&dir).unwrap();

    {
        let mut writer = store.writer(1, ResultKind::NodalDisp).unwrap();
        let batch = nodal_disp_batch(
            &[1, 2, 3],
            &[
                [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
            ],
        )
        .unwrap();
        writer.write_rows(&batch).unwrap();
        writer.finish().unwrap();
    }
    store.sync().unwrap();

    let manifest = store.manifest();
    assert_eq!(manifest.entries.len(), 1);
    let entry = &manifest.entries[0];
    assert_eq!(entry.case, 1);
    assert_eq!(entry.kind, ResultKind::NodalDisp);
    assert_eq!(entry.rows, 3);

    let _ = std::fs::remove_dir_all(&dir);
}

/// finish 済みでも `discard_pending` した保留エントリは manifest へ採用されない
/// （途中失敗ジョブの部分結果が後続の sync で照会可能になるのを防ぐ）。
/// 破棄後の sync は何も追加せず、以降の正常な書き込みは通常どおり反映される。
#[test]
fn test_fs_result_store_discard_pending_drops_partial_results() {
    let dir = std::env::temp_dir().join("p8_fsrs_discard");
    let _ = std::fs::remove_dir_all(&dir);
    let mut store = FsResultStore::open(&dir).unwrap();

    // 途中失敗ジョブを模擬: NodalDisp は finish 済み（保留に積まれる）だが、
    // 後続の書き込みが失敗した想定で保留分を破棄する。
    {
        let mut writer = store.writer(1, ResultKind::NodalDisp).unwrap();
        let batch = nodal_disp_batch(&[1], &[[0.1; 6]]).unwrap();
        writer.write_rows(&batch).unwrap();
        writer.finish().unwrap();
    }
    store.discard_pending();
    store.sync().unwrap();
    assert!(
        store.manifest().entries.is_empty(),
        "破棄した保留エントリが manifest へ採用されてはならない"
    );

    // 破棄後の正常な書き込みは通常どおり manifest へ反映される。
    {
        let mut writer = store.writer(2, ResultKind::NodalDisp).unwrap();
        let batch = nodal_disp_batch(&[1, 2], &[[0.1; 6], [0.2; 6]]).unwrap();
        writer.write_rows(&batch).unwrap();
        writer.finish().unwrap();
    }
    store.sync().unwrap();
    assert_eq!(store.manifest().entries.len(), 1);
    assert_eq!(store.manifest().entries[0].case, 2);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_fs_result_store_rewrite_dedup() {
    let dir = std::env::temp_dir().join("p8_fsrs_dedup");
    let _ = std::fs::remove_dir_all(&dir);
    let mut store = FsResultStore::open(&dir).unwrap();

    for rows in [2usize, 4usize] {
        let mut writer = store.writer(1, ResultKind::NodalDisp).unwrap();
        let ids: Vec<u32> = (0..rows as u32).collect();
        let disp: Vec<[f64; 6]> = vec![[0.0; 6]; rows];
        let batch = nodal_disp_batch(&ids, &disp).unwrap();
        writer.write_rows(&batch).unwrap();
        writer.finish().unwrap();
        store.sync().unwrap();
    }

    let manifest = store.manifest();
    assert_eq!(manifest.entries.len(), 1);
    assert_eq!(manifest.entries[0].rows, 4);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_fs_result_store_query_node_filter() {
    let dir = std::env::temp_dir().join("p8_fsrs_query_filter");
    let _ = std::fs::remove_dir_all(&dir);
    let mut store = FsResultStore::open(&dir).unwrap();
    {
        let mut writer = store.writer(1, ResultKind::NodalDisp).unwrap();
        let batch = nodal_disp_batch(&[1, 2, 3], &[[0.1; 6], [0.2; 6], [0.3; 6]]).unwrap();
        writer.write_rows(&batch).unwrap();
        writer.finish().unwrap();
    }
    store.sync().unwrap();

    let result = store
        .query(&ResultQuery {
            case: 1,
            kind: ResultKind::NodalDisp,
            node_filter: Some(vec![NodeId(2)]),
            member_filter: None,
            step_range: None,
        })
        .unwrap();
    assert_eq!(result.batch.num_rows(), 1);
    let node_col = result
        .batch
        .column(0)
        .as_any()
        .downcast_ref::<UInt32Array>()
        .unwrap();
    assert_eq!(node_col.value(0), 2);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_fs_result_store_reopen_restores_manifest() {
    let dir = std::env::temp_dir().join("p8_fsrs_reopen");
    let _ = std::fs::remove_dir_all(&dir);
    {
        let mut store = FsResultStore::open(&dir).unwrap();
        let mut writer = store.writer(1, ResultKind::NodalDisp).unwrap();
        let batch = nodal_disp_batch(&[1, 2], &[[0.1; 6], [0.2; 6]]).unwrap();
        writer.write_rows(&batch).unwrap();
        writer.finish().unwrap();
        store.sync().unwrap();
    }

    let store2 = FsResultStore::open(&dir).unwrap();
    let manifest = store2.manifest();
    assert_eq!(manifest.entries.len(), 1);
    assert_eq!(manifest.entries[0].rows, 2);
    assert_eq!(manifest.entries[0].case, 1);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_fs_result_store_time_history_step_range_query() {
    let dir = std::env::temp_dir().join("p8_fsrs_th_steprange");
    let _ = std::fs::remove_dir_all(&dir);
    let mut store = FsResultStore::open(&dir).unwrap();
    {
        let mut writer = store.writer(1, ResultKind::TimeHistory).unwrap();
        for s in 0..3u64 {
            let batch =
                time_history_batch(s, s as f64 * 0.1, &[1, 2], &[[0.1; 6], [0.2; 6]]).unwrap();
            writer.write_rows(&batch).unwrap();
        }
        writer.finish().unwrap();
    }
    store.sync().unwrap();

    let result = store
        .query(&ResultQuery {
            case: 1,
            kind: ResultKind::TimeHistory,
            node_filter: None,
            member_filter: None,
            step_range: Some((1, 2)),
        })
        .unwrap();
    assert_eq!(result.batch.num_rows(), 4);
    let step_col = result
        .batch
        .column(0)
        .as_any()
        .downcast_ref::<UInt64Array>()
        .unwrap();
    for i in 0..result.batch.num_rows() {
        let s = step_col.value(i);
        assert!((1..=2).contains(&s));
    }

    let _ = std::fs::remove_dir_all(&dir);
}
