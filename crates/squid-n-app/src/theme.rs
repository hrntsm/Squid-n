//! TONMANUAL（トンマナガイド）に基づく配色・テーマの単一情報源。
//!
//! 色値は本書 §2（カラーパレット）／§3（データビジュアライゼーション）／§3-2（3D ビュー）
//! の値をそのまま定数化したもの。UI 各所はこの定数を参照し、独自色を散らさない。
//! テーマ全体（ライト基準・ブルークローム・角丸）は [`apply_theme`] で egui に適用する。

use egui::{Color32, CornerRadius, FontFamily, FontId, Stroke, TextStyle};

// ===== §2 プライマリ（ブランドブルー） =====
/// ナビゲーション／ツールバー背景
pub const BLUE_200: Color32 = Color32::from_rgb(0xBF, 0xDB, 0xFE);
/// 選択ハイライト、ボタンホバー、ヘッダー
pub const BLUE_300: Color32 = Color32::from_rgb(0x93, 0xC5, 0xFD);
/// アナウンスバー背景
pub const BLUE_400: Color32 = Color32::from_rgb(0x60, 0xA5, 0xFA);
/// メインアクセント。アクティブボタン・枠線・インジケーター
pub const BLUE_500: Color32 = Color32::from_rgb(0x3B, 0x82, 0xF6);
/// アクセントのホバー濃色
pub const BLUE_600: Color32 = Color32::from_rgb(0x25, 0x63, 0xEB);

// ===== §2 セカンダリ（グレースケール） =====
/// 見出し・主要テキスト
pub const GRAY_900: Color32 = Color32::from_rgb(0x11, 0x18, 0x27);
/// ナビゲーションテキスト・サブテキスト
pub const GRAY_700: Color32 = Color32::from_rgb(0x37, 0x41, 0x51);
/// 本文テキスト
pub const GRAY_600: Color32 = Color32::from_rgb(0x4B, 0x55, 0x63);
/// キャンバスのドットグリッド
pub const GRAY_300: Color32 = Color32::from_rgb(0xD1, 0xD5, 0xDB);
/// ボーダー・入力欄枠線・ホバー背景・テーブルストライプ
pub const GRAY_200: Color32 = Color32::from_rgb(0xE5, 0xE7, 0xEB);
/// パネル背景・ウィジェット背景・入力欄背景
pub const GRAY_100: Color32 = Color32::from_rgb(0xF3, 0xF4, 0xF6);
/// メインキャンバス・チャートセル背景
pub const WHITE: Color32 = Color32::WHITE;

// ===== §2 アクション／セマンティック =====
/// 重要操作（実行・確定）アクション
pub const GREEN_500: Color32 = Color32::from_rgb(0x22, 0xC5, 0x5E);
/// アクションのホバー
pub const GREEN_600: Color32 = Color32::from_rgb(0x16, 0xA3, 0x4A);
/// エラー表示（ブランド対象外の固定色）
pub const ERROR_RED: Color32 = Color32::from_rgb(0xEA, 0x43, 0x35);

// ===== §3 データビジュアライゼーション配色 =====
/// 標準・既定（既定の線・要素・変位、グラフの基準系列）
pub const DATA_BLUE: Color32 = Color32::from_rgb(0x42, 0x85, 0xF4);
/// 超過・危険側（検定比 > 1.0＝NG・応力集中・負方向の量・崩壊ヒンジ）
pub const PARETO_RED: Color32 = Color32::from_rgb(0xEA, 0x43, 0x35);
/// 注意・中間（検定比 0.8〜1.0・中間状態・予測値）
pub const BEST_YELLOW: Color32 = Color32::from_rgb(0xFB, 0xBC, 0x04);
/// 良好・OK（検定比 ≤ 0.8＝OK・収束・正常状態）
pub const GOOD_GREEN: Color32 = Color32::from_rgb(0x34, 0xA8, 0x53);
/// ハイライト（選択中の節点・部材・断面）
pub const HILITE_PURPLE: Color32 = Color32::from_rgb(0x7C, 0x4D, 0xFF);
/// 二次部材（小梁・間柱）= 解析対象外の実在部材の線・輪郭
/// （スラブの BEST_YELLOW と同族の暖色。線の視認性のため濃いめのアンバー）
pub const SECONDARY_AMBER: Color32 = Color32::from_rgb(0xD9, 0x77, 0x06);

// ===== §3-2 3D ビュー =====
/// 3D 背景（2D の白とは異なり淡いグレー。立体感のため意図的に白を避ける）
pub const VIEW_BG: Color32 = Color32::from_rgb(0xF0, 0xF2, 0xF5);
/// X 軸（赤系）
pub const AXIS_X: Color32 = Color32::from_rgb(0xD2, 0x64, 0x64);
/// Y 軸（緑系）
pub const AXIS_Y: Color32 = Color32::from_rgb(0x50, 0xAA, 0x50);
/// Z 軸（青系）
pub const AXIS_Z: Color32 = Color32::from_rgb(0x64, 0x64, 0xC8);
/// 免震支承マーカー（軸方向を持たない支持装置の記号。矢印・円弧・ばねのジグザグ／
/// 渦巻（軸色）とも、剛床マーク・ハイライト（紫）とも弁別できる専用色）
pub const ISOLATOR_TEAL: Color32 = Color32::from_rgb(0x0F, 0x76, 0x6E);

/// パレット色を指定アルファで半透明化する（面要素・応力図の塗りつぶし用）。
pub fn translucent(c: Color32, alpha: u8) -> Color32 {
    Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), alpha)
}

/// 色を白側へ `t`（0.0–1.0）だけ寄せて淡くする（§3-2 軸ラベル負方向端の淡色など）。
pub fn lighten(c: Color32, t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    let mix = |ch: u8| (ch as f32 + (255.0 - ch as f32) * t).round() as u8;
    Color32::from_rgb(mix(c.r()), mix(c.g()), mix(c.b()))
}

/// テーブル行（ヘッダ・本文とも）の上下余白。行高がフォントの行高ちょうどだと
/// セル内の文字が枠線に接して窮屈に見えるため、視認性のために少し余裕を持たせる。
const TABLE_ROW_PADDING_PX: f32 = 3.0;

/// テーブルの標準行高。本文フォントの行高＋ [`TABLE_ROW_PADDING_PX`]。
/// 固定 px 値（旧 18px 等）では日本語フォントの行高（Body 13pt で約 19px）に足りず
/// 文字の下側が見切れるため、実測から算定する。ヘッダにも同じ値を使う。
pub fn table_row_height(ui: &egui::Ui) -> f32 {
    ui.text_style_height(&TextStyle::Body) + TABLE_ROW_PADDING_PX
}

/// 検定比などの「状態」を §3 のセマンティック 3 色へ写像する。
/// 良好(≤0.8)=緑／注意(≤1.0)=黄／超過(>1.0)=赤。
pub fn status_color(ratio: f64) -> Color32 {
    if ratio <= 0.8 {
        GOOD_GREEN
    } else if ratio <= 1.0 {
        BEST_YELLOW
    } else {
        PARETO_RED
    }
}

/// 検定比のグラデーション配色（[`check_ratio_color`]）のアンカー点。
///
/// t は検定比そのもの（0.0–1.0）。0.8 に同じ t のアンカーを2つ置くことで、
/// 良好域（緑）と注意域（黄）の境界を連続補間せず不連続に切り替える
/// （判定基準 0.8 をまたいだことを色相の変化で明示するため）。
/// t=0.0 の淡緑は `lighten(GOOD_GREEN, 0.6)` の値（const 文脈では関数を
/// 呼べないため展開した定数）。
const CHECK_RATIO_LUT: [(f32, Color32); 4] = [
    (0.0, Color32::from_rgb(0xAE, 0xDC, 0xBA)),
    (0.8, GOOD_GREEN),
    (0.8, BEST_YELLOW),
    (1.0, SECONDARY_AMBER),
];

/// 検定比を連続グラデーション（コンター）へ写像する（3D ビューの検定比図用）。
///
/// [`status_color`] と同じ判定境界（0.8／1.0）を色相の切り替わりとして保ちつつ、
/// 各帯域の内部を連続的に濃くすることで、数値ラベルを出さなくても余裕度の
/// 大小を色から読み取れるようにしたもの。
///
/// - ≤0.8: 淡緑→緑（余裕が小さいほど濃い緑）
/// - ≤1.0: 黄→アンバー（注意域。0.8 の境界で緑から明確に色相が変わる）
/// - >1.0: 赤（NG。連続値ではなく単色）
///
/// 検定表（`design_view.rs`）のセル着色は従来どおり [`status_color`] の3色規約を
/// 使う（表では文字と背景色の対比が要るため、淡い連続色は適さない）。
pub fn check_ratio_color(ratio: f64) -> Color32 {
    if ratio > 1.0 {
        PARETO_RED
    } else {
        lut_sample(&CHECK_RATIO_LUT, ratio as f32)
    }
}

/// LUT（`(t, Color32)` のアンカー点列。t 昇順・[0,1] を覆う）を区間線形補間する
/// 共通ヘルパ。各カラーマップ（[`ColorMap::sample`]）はこれを呼ぶだけの薄い実装にする。
/// `t` は 0.0–1.0 にクランプしてから探索する。
fn lut_sample(lut: &[(f32, Color32)], t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    for w in lut.windows(2) {
        let (t0, c0) = w[0];
        let (t1, c1) = w[1];
        if t <= t1 {
            let local = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
            let mix = |a: u8, b: u8| (a as f32 + (b as f32 - a as f32) * local).round() as u8;
            return Color32::from_rgb(
                mix(c0.r(), c1.r()),
                mix(c0.g(), c1.g()),
                mix(c0.b(), c1.b()),
            );
        }
    }
    lut.last().map(|&(_, c)| c).unwrap_or(WHITE)
}

/// Viridis カラーマップのアンカー点（matplotlib 版, t=0.000〜1.000 を 0.125 刻み）。
const VIRIDIS_LUT: [(f32, Color32); 9] = [
    (0.000, Color32::from_rgb(0x44, 0x01, 0x54)),
    (0.125, Color32::from_rgb(0x48, 0x28, 0x78)),
    (0.250, Color32::from_rgb(0x3E, 0x49, 0x89)),
    (0.375, Color32::from_rgb(0x31, 0x68, 0x8E)),
    (0.500, Color32::from_rgb(0x26, 0x82, 0x8E)),
    (0.625, Color32::from_rgb(0x1F, 0x9E, 0x89)),
    (0.750, Color32::from_rgb(0x35, 0xB7, 0x79)),
    (0.875, Color32::from_rgb(0x6D, 0xCD, 0x59)),
    (1.000, Color32::from_rgb(0xFD, 0xE7, 0x25)),
];

/// Plasma カラーマップのアンカー点（matplotlib 版, t=0.000〜1.000 を 0.125 刻み）。
const PLASMA_LUT: [(f32, Color32); 9] = [
    (0.000, Color32::from_rgb(0x0D, 0x08, 0x87)),
    (0.125, Color32::from_rgb(0x46, 0x03, 0x9F)),
    (0.250, Color32::from_rgb(0x72, 0x01, 0xA8)),
    (0.375, Color32::from_rgb(0x9C, 0x17, 0x9E)),
    (0.500, Color32::from_rgb(0xBD, 0x37, 0x86)),
    (0.625, Color32::from_rgb(0xD8, 0x57, 0x6B)),
    (0.750, Color32::from_rgb(0xED, 0x79, 0x53)),
    (0.875, Color32::from_rgb(0xFB, 0x9F, 0x3A)),
    (1.000, Color32::from_rgb(0xF0, 0xF9, 0x21)),
];

/// Turbo カラーマップのアンカー点（t=0.000〜1.000 を 0.125 刻み）。
/// 端点（t=0.000/1.000）は Google 公開の 256 点参照テーブルの正確な値
/// （インデックス 0 = `#30123B`、255 = `#7A0403`）、中間 7 点は広く知られた
/// Turbo の 5 次多項式近似から算出した値で、暗青→シアン→緑→黄→橙→暗赤の
/// 帯域配分を保つ。
const TURBO_LUT: [(f32, Color32); 9] = [
    (0.000, Color32::from_rgb(0x30, 0x12, 0x3B)),
    (0.125, Color32::from_rgb(0x45, 0x69, 0xEE)),
    (0.250, Color32::from_rgb(0x26, 0xBC, 0xE1)),
    (0.375, Color32::from_rgb(0x3F, 0xF3, 0x93)),
    (0.500, Color32::from_rgb(0x95, 0xFB, 0x51)),
    (0.625, Color32::from_rgb(0xEC, 0xD1, 0x2E)),
    (0.750, Color32::from_rgb(0xFF, 0x82, 0x1D)),
    (0.875, Color32::from_rgb(0xCB, 0x2F, 0x0D)),
    (1.000, Color32::from_rgb(0x7A, 0x04, 0x03)),
];

/// Jet カラーマップのアンカー点（伝統的な虹色レインボー。青→シアン→緑→黄→赤）。
const JET_LUT: [(f32, Color32); 9] = [
    (0.000, Color32::from_rgb(0x00, 0x00, 0x7F)),
    (0.125, Color32::from_rgb(0x00, 0x00, 0xFF)),
    (0.250, Color32::from_rgb(0x00, 0x7F, 0xFF)),
    (0.375, Color32::from_rgb(0x00, 0xFF, 0xFF)),
    (0.500, Color32::from_rgb(0x7F, 0xFF, 0x7F)),
    (0.625, Color32::from_rgb(0xFF, 0xFF, 0x00)),
    (0.750, Color32::from_rgb(0xFF, 0x7F, 0x00)),
    (0.875, Color32::from_rgb(0xFF, 0x00, 0x00)),
    (1.000, Color32::from_rgb(0x7F, 0x00, 0x00)),
];

/// 発散型（青-白-赤）カラーマップのアンカー点。独自色は使わず theme.rs の
/// 既存色（[`DATA_BLUE`] / [`WHITE`] / [`PARETO_RED`]）のみで構成する。
const BLUE_WHITE_RED_LUT: [(f32, Color32); 3] = [(0.0, DATA_BLUE), (0.5, WHITE), (1.0, PARETO_RED)];

/// コンター等の連続値表示で選択可能なカラーマップ。
///
/// TONMANUAL §3「カラーマップ（連続値）」は「知覚均等で色覚多様性に配慮した Viridis を
/// 既定とする」と規定しており、この規定は本 enum の `#[default]` を [`ColorMap::Viridis`]
/// にすることで維持している。他の選択肢は UI からの明示的な切替のためのものであり、
/// 既定動作を変更するものではない。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ColorMap {
    /// 知覚均等・色覚多様性配慮（TONMANUAL §3 既定）
    #[default]
    Viridis,
    /// 知覚均等系（紫→赤→黄）
    Plasma,
    /// Google 開発の改良レインボー（知覚的な歪みを抑えたレインボー配色）
    Turbo,
    /// 伝統的な虹色レインボー（知覚均等ではないが慣れ親しまれている）
    Jet,
    /// 発散型（負=青／中央=白／正=赤）。符号のある値の可視化向け
    BlueWhiteRed,
}

impl ColorMap {
    /// UI 表示用ラベル。
    pub fn label(&self) -> &'static str {
        match self {
            ColorMap::Viridis => "Viridis",
            ColorMap::Plasma => "Plasma",
            ColorMap::Turbo => "Turbo",
            ColorMap::Jet => "Jet",
            ColorMap::BlueWhiteRed => "青-白-赤",
        }
    }

    /// `t`（0.0–1.0、範囲外はクランプ）をこのカラーマップの色へ写像する。
    pub fn sample(&self, t: f32) -> Color32 {
        let lut: &[(f32, Color32)] = match self {
            ColorMap::Viridis => &VIRIDIS_LUT,
            ColorMap::Plasma => &PLASMA_LUT,
            ColorMap::Turbo => &TURBO_LUT,
            ColorMap::Jet => &JET_LUT,
            ColorMap::BlueWhiteRed => &BLUE_WHITE_RED_LUT,
        };
        lut_sample(lut, t)
    }
}

/// TONMANUAL に沿ったテーマ（ライト基準・ブルークローム・角丸 4/6px・タイポスケール）を
/// egui コンテキストへ適用する。アプリ起動時に一度だけ呼ぶ。
pub fn apply_theme(ctx: &egui::Context) {
    // eframe がダークテーマで起動する場合を防ぐため、
    // visuals を先にライトテーマで上書きしてから詳細設定を重ねる
    ctx.set_visuals(egui::Visuals::light());

    let mut style = (*ctx.global_style()).clone();
    let mut v = egui::Visuals::light();

    // 背景の階層（§2）: パネル＝gray-100、入力欄＝gray-100、ストライプ＝gray-200
    v.panel_fill = GRAY_100;
    v.window_fill = GRAY_100;
    v.extreme_bg_color = GRAY_100;
    v.faint_bg_color = GRAY_200;
    v.hyperlink_color = BLUE_500;

    // 選択ハイライト（§6 アクティブ）= blue-500 背景 + 白文字
    v.selection.bg_fill = BLUE_500;
    v.selection.stroke = Stroke::new(1.0_f32, WHITE);

    v.window_corner_radius = CornerRadius::same(6);
    v.menu_corner_radius = CornerRadius::same(6);

    let r4 = CornerRadius::same(4); // 小要素（ボタン）
    let r6 = CornerRadius::same(6); // カード／パネル

    // 静かなクローム: 非対話（パネル・ラベル・カード）= gray-100 / gray-200 枠 / gray-700 文字
    v.widgets.noninteractive.bg_fill = GRAY_100;
    v.widgets.noninteractive.weak_bg_fill = GRAY_100;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, GRAY_200);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0_f32, GRAY_700);
    v.widgets.noninteractive.corner_radius = r6;

    // ボタン（rest）: 透明背景・gray-700 文字・gray-200 枠（入力欄/コンボ兼用）
    v.widgets.inactive.bg_fill = Color32::TRANSPARENT;
    v.widgets.inactive.weak_bg_fill = Color32::TRANSPARENT;
    v.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, GRAY_200);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0_f32, GRAY_700);
    v.widgets.inactive.corner_radius = r4;

    // ホバー: blue-300 背景 + 白文字
    v.widgets.hovered.bg_fill = BLUE_300;
    v.widgets.hovered.weak_bg_fill = BLUE_300;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0_f32, BLUE_300);
    v.widgets.hovered.fg_stroke = Stroke::new(1.5_f32, WHITE);
    v.widgets.hovered.corner_radius = r4;

    // アクティブ（押下・選択）: blue-500 背景 + 白文字
    v.widgets.active.bg_fill = BLUE_500;
    v.widgets.active.weak_bg_fill = BLUE_500;
    v.widgets.active.bg_stroke = Stroke::new(1.0_f32, BLUE_500);
    v.widgets.active.fg_stroke = Stroke::new(1.5_f32, WHITE);
    v.widgets.active.corner_radius = r4;

    // コンボボックス展開トリガ: 入力欄相当（gray-100 / gray-200 枠）
    v.widgets.open.bg_fill = GRAY_100;
    v.widgets.open.weak_bg_fill = GRAY_100;
    v.widgets.open.bg_stroke = Stroke::new(1.0_f32, GRAY_200);
    v.widgets.open.fg_stroke = Stroke::new(1.0_f32, GRAY_700);
    v.widgets.open.corner_radius = r4;

    style.visuals = v;

    // タイポグラフィスケール（§4）: 見出し / 13 / 12 / 11
    style.text_styles = [
        (
            TextStyle::Heading,
            FontId::new(18.0, FontFamily::Proportional),
        ),
        (TextStyle::Body, FontId::new(13.0, FontFamily::Proportional)),
        (
            TextStyle::Button,
            FontId::new(13.0, FontFamily::Proportional),
        ),
        (
            TextStyle::Monospace,
            FontId::new(12.0, FontFamily::Monospace),
        ),
        (
            TextStyle::Small,
            FontId::new(11.0, FontFamily::Proportional),
        ),
    ]
    .into();

    ctx.set_global_style(style);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// TONMANUAL §3 の規定どおり、既定のカラーマップは Viridis。
    #[test]
    fn colormap_default_is_viridis() {
        assert_eq!(ColorMap::default(), ColorMap::Viridis);
    }

    /// 各カラーマップの端点（t=0.0 / t=1.0）が LUT の先頭・末尾の色と一致する。
    #[test]
    fn colormap_sample_matches_endpoint_anchors() {
        assert_eq!(
            ColorMap::Viridis.sample(0.0),
            Color32::from_rgb(0x44, 0x01, 0x54)
        );
        assert_eq!(
            ColorMap::Viridis.sample(1.0),
            Color32::from_rgb(0xFD, 0xE7, 0x25)
        );
        assert_eq!(
            ColorMap::Plasma.sample(0.0),
            Color32::from_rgb(0x0D, 0x08, 0x87)
        );
        assert_eq!(
            ColorMap::Plasma.sample(1.0),
            Color32::from_rgb(0xF0, 0xF9, 0x21)
        );
        assert_eq!(
            ColorMap::Turbo.sample(0.0),
            Color32::from_rgb(0x30, 0x12, 0x3B)
        );
        assert_eq!(
            ColorMap::Turbo.sample(1.0),
            Color32::from_rgb(0x7A, 0x04, 0x03)
        );
        assert_eq!(
            ColorMap::Jet.sample(0.0),
            Color32::from_rgb(0x00, 0x00, 0x7F)
        );
        assert_eq!(
            ColorMap::Jet.sample(1.0),
            Color32::from_rgb(0x7F, 0x00, 0x00)
        );
        assert_eq!(ColorMap::BlueWhiteRed.sample(0.0), DATA_BLUE);
        assert_eq!(ColorMap::BlueWhiteRed.sample(1.0), PARETO_RED);
    }

    /// 発散型（青-白-赤）は中央 t=0.5 がニュートラルな白になる。
    #[test]
    fn colormap_blue_white_red_midpoint_is_white() {
        assert_eq!(ColorMap::BlueWhiteRed.sample(0.5), WHITE);
    }

    /// 範囲外の値はクランプされる（全カラーマップ共通の `lut_sample` の挙動）。
    #[test]
    fn colormap_sample_clamps_out_of_range() {
        assert_eq!(
            ColorMap::Viridis.sample(-5.0),
            ColorMap::Viridis.sample(0.0)
        );
        assert_eq!(ColorMap::Viridis.sample(5.0), ColorMap::Viridis.sample(1.0));
    }

    /// 検定比のグラデーションは、判定境界で `status_color` と同じ色になる
    /// （良好域の上端＝緑、注意域の下端＝黄、NG＝赤）。
    #[test]
    fn check_ratio_color_matches_status_color_at_boundaries() {
        assert_eq!(check_ratio_color(0.8), GOOD_GREEN);
        assert_eq!(check_ratio_color(0.80001), BEST_YELLOW);
        assert_eq!(check_ratio_color(1.0), SECONDARY_AMBER);
        assert_eq!(check_ratio_color(1.0001), PARETO_RED);
        assert_eq!(check_ratio_color(5.0), PARETO_RED);
    }

    /// 良好域（≤0.8）は淡緑から緑へ連続的に変化する（同じ緑一色にはならない）。
    #[test]
    fn check_ratio_color_is_gradient_below_caution() {
        let c0 = check_ratio_color(0.0);
        let c4 = check_ratio_color(0.4);
        let c8 = check_ratio_color(0.8);
        assert_ne!(c0, c4);
        assert_ne!(c4, c8);
        // 検定比が大きいほど濃い（緑成分が減る）方向へ変化する
        assert!(c0.g() > c4.g());
        assert!(c4.g() > c8.g());
    }

    /// 負値は 0 側へクランプされる（下限の淡緑）。
    #[test]
    fn check_ratio_color_clamps_negative() {
        assert_eq!(check_ratio_color(-1.0), check_ratio_color(0.0));
    }

    /// 表示ラベルが想定どおり。
    #[test]
    fn colormap_labels() {
        assert_eq!(ColorMap::Viridis.label(), "Viridis");
        assert_eq!(ColorMap::Plasma.label(), "Plasma");
        assert_eq!(ColorMap::Turbo.label(), "Turbo");
        assert_eq!(ColorMap::Jet.label(), "Jet");
        assert_eq!(ColorMap::BlueWhiteRed.label(), "青-白-赤");
    }
}
