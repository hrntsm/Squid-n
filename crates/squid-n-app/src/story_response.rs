//! 時刻歴応答の詳細記録（`squid_n_solver::timehistory::ThRecording`）から
//! 層応答分布（層せん断力・層せん断力係数・階加速度・階速度・階変位）を
//! 求める純粋関数群。GUI 非依存（`time_history_view`（グラフ描画、`gui` 機能）と
//! `summary`（レポート CSV、機能フラグ非依存）の両方から使う。
//!
//! 単位はいずれもソルバ内部単位（N・mm・s・rad）を前提とし、表示単位
//! （kN・gal・m/s）への換算関数をあわせて提供する。

/// 層/階の時系列 `[frame][story]` から、各層(階)の絶対値最大値を層ごとに求める。
/// `n_story` は結果ベクトルの長さ（`series` の要素数が 0 でもゼロ埋めで返す）。
/// 各フレームの要素数が `n_story` と異なる場合は短い方までのみ集計する
/// （層構成が解析中に変わることはないが、防御的に扱う）。
pub fn story_absmax(series: &[Vec<f64>], n_story: usize) -> Vec<f64> {
    let mut out = vec![0.0_f64; n_story];
    for frame in series {
        for (o, &v) in out.iter_mut().zip(frame.iter()) {
            let a = v.abs();
            if a > *o {
                *o = a;
            }
        }
    }
    out
}

/// 記録済みの階 ID 列（`ThRecording`/`StoryResponse::stories`）と現在の
/// `model.stories` を `StoryId` で突き合わせ、階の表示名を求める（低）。
/// 解析後にモデルの階が編集（追加・削除・並び替え）されても、添字ではなく
/// `StoryId` で対応する階を探すため、別の階の名前を誤って表示しない。
/// 該当する階が見つからない場合（記録後に当該階が削除された等）は
/// 「(削除済み階)」を返す。
///
/// `model_stories` は `app.model.stories.iter().map(|s| (s.id, s.name.clone()))`
/// を渡すことを想定する（本関数は GUI 非依存に保つため `Model` 型に依存しない）。
pub fn story_display_names(
    model_stories: &[(squid_n_core::ids::StoryId, String)],
    recorded: &[squid_n_core::ids::StoryId],
) -> Vec<String> {
    recorded
        .iter()
        .map(|id| {
            model_stories
                .iter()
                .find(|(sid, _)| sid == id)
                .map(|(_, name)| name.clone())
                .unwrap_or_else(|| "(削除済み階)".to_string())
        })
        .collect()
}

/// N → kN（表示用。[`squid_n_core::units::to_display::force_kn`] への委譲）。
pub fn n_to_kn(n: f64) -> f64 {
    squid_n_core::units::to_display::force_kn(n)
}

/// mm/s² → gal（1 gal = 1 cm/s² = 10 mm/s²）
pub fn mm_s2_to_gal(a: f64) -> f64 {
    a / 10.0
}

/// mm/s → m/s（表示用。[`squid_n_core::units::to_display::length_m`] への委譲）。
pub fn mm_s_to_m_s(v: f64) -> f64 {
    squid_n_core::units::to_display::length_m(v)
}

/// 層応答分布グラフの表示項目（横軸に取る量）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum StoryResponseKind {
    /// 層せん断力 [kN]（層量、階段状）
    #[default]
    Shear,
    /// 層せん断力係数 Ci [-]（層量、階段状）
    ShearCoeff,
    /// 階絶対加速度 [gal]（階量、点＋折れ線）
    Accel,
    /// 階速度(相対) [m/s]（階量、点＋折れ線）
    Vel,
    /// 階変位(相対) [mm]（階量、点＋折れ線）
    Disp,
}

impl StoryResponseKind {
    /// 層量（層の上下端で一定の階段状の線で描く）か、階量（各階の点）か。
    pub fn is_story_quantity(self) -> bool {
        matches!(self, Self::Shear | Self::ShearCoeff)
    }
}

/// 層応答分布グラフの方向選択（記録済みの X・Y いずれかの層応答）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum StoryRespDir {
    #[default]
    X,
    Y,
}

/// 層応答分布グラフの Y 軸ラベル。整数グリッドのみラベルを返し（ズーム時の
/// 補助グリッドは空文字）、`y=0` は地盤面（GL）、`y=k`（1≦k≦stories数）は
/// `story_names[k-1]`（層 k-1 の上端＝階 k の床）とする。
pub fn story_axis_label(story_names: &[String], y: f64) -> String {
    let rounded = y.round();
    if (y - rounded).abs() > 1e-6 {
        return String::new();
    }
    let k = rounded as i64;
    if k == 0 {
        return "GL".to_string();
    }
    if k >= 1 && (k as usize) <= story_names.len() {
        return story_names[(k - 1) as usize].clone();
    }
    String::new()
}

/// 層量（層せん断力・層せん断力係数）の階段状プロット用点列 `[value, y]`。
/// 層 i の値は `y∈[i, i+1]`（下→上、`model.stories` の並びと同じ）の水平区間として
/// 描画する。隣接する層境界（`y=i+1`）では前後の層の値を結ぶ縦の遷移が入る。
pub fn story_step_points(values: &[f64]) -> Vec<[f64; 2]> {
    let mut pts = Vec::with_capacity(values.len() * 2);
    for (i, &v) in values.iter().enumerate() {
        pts.push([v, i as f64]);
        pts.push([v, (i + 1) as f64]);
    }
    pts
}

/// 階量（階加速度・階速度・階変位）のプロット用点列 `[value, y]`。
/// 階 i の値は `y=i+1`（層 i の上端＝階 i+1 の床）に置く。
pub fn floor_points(values: &[f64]) -> Vec<[f64; 2]> {
    values
        .iter()
        .enumerate()
        .map(|(i, &v)| [v, (i + 1) as f64])
        .collect()
}

/// 層応答分布グラフのホバー位置（プロット座標の y）から、対応する層・階の
/// 添字（0 始まり）を求める。層量（`is_story_quantity=true`）は `y` を含む区間
/// `[i, i+1)` の `i`、階量は最寄りの `y=k` の `k-1`。返り値は `0..n_story` に
/// クランプする（範囲外ホバーでも安全に最寄りの層を指す）。
pub fn hover_story_index(y: f64, n_story: usize, is_story_quantity: bool) -> usize {
    if n_story == 0 {
        return 0;
    }
    let idx = if is_story_quantity {
        y.floor() as i64
    } else {
        y.round() as i64 - 1
    };
    idx.clamp(0, n_story as i64 - 1) as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    use squid_n_core::ids::StoryId;

    #[test]
    fn test_story_display_names_matches_by_story_id_not_index() {
        // モデル側は StoryId(1),StoryId(0) の順（記録時と並びが入れ替わっている想定）。
        let model_stories = vec![
            (StoryId(1), "2F".to_string()),
            (StoryId(0), "1F".to_string()),
        ];
        let recorded = vec![StoryId(0), StoryId(1)];
        assert_eq!(
            story_display_names(&model_stories, &recorded),
            vec!["1F".to_string(), "2F".to_string()],
            "添字ではなく StoryId で対応する階名を引くはず"
        );
    }

    #[test]
    fn test_story_display_names_missing_story_falls_back() {
        let model_stories = vec![(StoryId(0), "1F".to_string())];
        let recorded = vec![StoryId(0), StoryId(1)];
        assert_eq!(
            story_display_names(&model_stories, &recorded),
            vec!["1F".to_string(), "(削除済み階)".to_string()],
            "記録後に削除された階は「(削除済み階)」になるはず"
        );
    }

    #[test]
    fn test_story_absmax_takes_abs_value_max_over_frames() {
        let series = vec![
            vec![1.0, -2.0, 0.5],
            vec![-3.0, 1.0, -0.2],
            vec![0.0, 0.0, 0.9],
        ];
        assert_eq!(story_absmax(&series, 3), vec![3.0, 2.0, 0.9]);
    }

    #[test]
    fn test_story_absmax_empty_series_returns_zeros() {
        let series: Vec<Vec<f64>> = vec![];
        assert_eq!(story_absmax(&series, 2), vec![0.0, 0.0]);
    }

    #[test]
    fn test_story_absmax_short_frame_is_handled_defensively() {
        // フレームの要素数が n_story と異なる場合でも panic せず短い方まで集計する。
        let series = vec![vec![1.0]];
        assert_eq!(story_absmax(&series, 3), vec![1.0, 0.0, 0.0]);
    }

    #[test]
    fn test_unit_conversions() {
        assert!((n_to_kn(1000.0) - 1.0).abs() < 1e-12);
        assert!((mm_s2_to_gal(10.0) - 1.0).abs() < 1e-12);
        assert!((mm_s_to_m_s(1000.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_story_axis_label_ground_and_stories() {
        let names = vec!["1F".to_string(), "2F".to_string(), "RF".to_string()];
        assert_eq!(story_axis_label(&names, 0.0), "GL");
        assert_eq!(story_axis_label(&names, 1.0), "1F");
        assert_eq!(story_axis_label(&names, 2.0), "2F");
        assert_eq!(story_axis_label(&names, 3.0), "RF");
        assert_eq!(story_axis_label(&names, 4.0), "");
        assert_eq!(story_axis_label(&names, -1.0), "");
    }

    #[test]
    fn test_story_axis_label_non_integer_is_blank() {
        let names = vec!["1F".to_string()];
        assert_eq!(story_axis_label(&names, 0.5), "");
    }

    #[test]
    fn test_story_step_points_shape() {
        let pts = story_step_points(&[10.0, 20.0]);
        assert_eq!(
            pts,
            vec![[10.0, 0.0], [10.0, 1.0], [20.0, 1.0], [20.0, 2.0]]
        );
    }

    #[test]
    fn test_floor_points_shape() {
        let pts = floor_points(&[1.0, 2.0, 3.0]);
        assert_eq!(pts, vec![[1.0, 1.0], [2.0, 2.0], [3.0, 3.0]]);
    }

    #[test]
    fn test_hover_story_index_story_quantity() {
        // 層量: y∈[i, i+1) が層 i。
        assert_eq!(hover_story_index(0.4, 3, true), 0);
        assert_eq!(hover_story_index(1.9, 3, true), 1);
        assert_eq!(
            hover_story_index(-1.0, 3, true),
            0,
            "範囲外は最寄りへクランプ"
        );
        assert_eq!(
            hover_story_index(10.0, 3, true),
            2,
            "範囲外は最寄りへクランプ"
        );
    }

    #[test]
    fn test_hover_story_index_floor_quantity() {
        // 階量: y=k の最寄りが層 k-1。
        assert_eq!(hover_story_index(1.0, 3, false), 0);
        assert_eq!(hover_story_index(2.2, 3, false), 1);
        assert_eq!(
            hover_story_index(0.0, 3, false),
            0,
            "範囲外は最寄りへクランプ"
        );
    }
}
