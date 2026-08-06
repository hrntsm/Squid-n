//! 表の共通フォーマット（列幅・縦線・クリップ・見出し・セル）を 1 箇所へまとめる
//! ヘルパ。TONMANUAL §6「テーブル」の規約は本モジュールだけが実装する。
//!
//! **表を描くときは必ず [`standard_table`] を通すこと。** `TableBuilder` を直に
//! 組むと、列区切りの縦線・クリップ・列幅の規約から外れ、表ごとに見た目が割れる。
//! 行の中身は呼び出し側のクロージャがそのまま描くため、セルに編集ウィジェット
//! （`TextEdit`・`ComboBox` 等）を置く表にも使える。
//!
//! 例外はスプレッドシート様式のグリッド（[`crate::grid`]）で、こちらは白地＋共有
//! 罫線という別様式（`dev_docs/specs/グリッド操作.md` §6）を意図的に採っている。

use egui_extras::{Column, TableBuilder, TableRow};

/// 列幅の用途トークン。同じ意味の列が表ごとに違う幅で並ばないよう、列幅は
/// 生の pt ではなくこのトークンで指定する。実幅はフォントから実測して決める
/// （TONMANUAL §4「テキストを内包する箱の寸法を固定 px で書かない」。和文
/// フォールバックの字幅は欧文フォントの想定と異なるため、定数では合わない）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ColWidth {
    /// ID 列。5 桁の整数と選択ボタンの余白が収まる幅
    Id,
    /// 数値列（`{:.0}`〜`{:.3}` の実数・整数）
    Num,
    /// 複合数値列（`1800 × 500`・`12.3 / 45.6` のように 2 値を 1 セルに置く列）
    WideNum,
    /// 短ラベル列（階名・種別記号・OK/NG）
    Label,
    /// 名称・符号列（材料名・部材符号・区分名）
    Name,
    /// 長い文字列の列（断面形状・注記・接合部の説明）
    Text,
    /// 行操作ボタンだけを置く列。値はボタンの個数
    Actions(u8),
}

/// 各トークンの幅を決める代表文字列。実データの最大長ではなく「この用途で
/// 通常は収まってほしい長さ」を表す。これを超える内容は切り詰められるため、
/// 文字列セルは [`text_cell`] でホバーに全文を出す。
const SAMPLE_ID: &str = "00000";
const SAMPLE_NUM: &str = "-00000.00";
const SAMPLE_WIDE_NUM: &str = "00000 × 00000";
const SAMPLE_LABEL: &str = "PH000";
const SAMPLE_NAME: &str = "STKR400(SRC)";
const SAMPLE_TEXT: &str = "BOX-300x300x12 (溶接組立)";
const SAMPLE_ACTIONS: &str = "🗑";

/// Body フォントで実測した文字列の幅 [pt]。
fn text_width(ui: &egui::Ui, text: &str) -> f32 {
    let font = egui::TextStyle::Body.resolve(ui.style());
    ui.painter()
        .layout_no_wrap(text.to_owned(), font, egui::Color32::PLACEHOLDER)
        .size()
        .x
}

/// セル内容が列区切りの縦線に接しないための左右余白。ボタンの内側余白と同量。
fn cell_padding(ui: &egui::Ui) -> f32 {
    ui.spacing().button_padding.x * 2.0
}

/// 列がリサイズで潰れられる下限。見出しすら読めない幅まで縮まないようにする。
fn min_column_width(ui: &egui::Ui) -> f32 {
    text_width(ui, "000") + cell_padding(ui)
}

impl ColWidth {
    /// トークンの実幅 [pt]。
    fn to_pt(self, ui: &egui::Ui) -> f32 {
        let sample = match self {
            // ボタンを置く列は、セル余白の内側にボタン自身の余白が入るため 2 重に見る
            Self::Id => return text_width(ui, SAMPLE_ID) + cell_padding(ui) * 2.0,
            Self::Actions(n) => {
                let n = n.max(1);
                let button = text_width(ui, SAMPLE_ACTIONS) + cell_padding(ui) * 2.0;
                let gaps = ui.spacing().item_spacing.x * f32::from(n - 1);
                return button * f32::from(n) + gaps;
            }
            Self::Num => SAMPLE_NUM,
            Self::WideNum => SAMPLE_WIDE_NUM,
            Self::Label => SAMPLE_LABEL,
            Self::Name => SAMPLE_NAME,
            Self::Text => SAMPLE_TEXT,
        };
        text_width(ui, sample) + cell_padding(ui)
    }
}

/// 列の定義（見出し・幅・見出しのツールチップ）。見出しと幅を 1 つにまとめて
/// あるため、列数と見出し数がずれることが起きない。
#[derive(Clone, Debug)]
pub(crate) struct Col<'a> {
    header: &'a str,
    width: ColWidth,
    hover: Option<&'a str>,
}

impl<'a> Col<'a> {
    fn new(header: &'a str, width: ColWidth) -> Self {
        Self {
            header,
            width,
            hover: None,
        }
    }

    /// ID 列（見出しは `ID` 固定）。
    pub(crate) fn id() -> Self {
        Self::new("ID", ColWidth::Id)
    }

    /// ID を載せるが見出しが `ID` ではない列（「部材」「節点」など）。
    pub(crate) fn id_named(header: &'a str) -> Self {
        Self::new(header, ColWidth::Id)
    }

    /// 数値列。
    pub(crate) fn num(header: &'a str) -> Self {
        Self::new(header, ColWidth::Num)
    }

    /// 複合数値列（`1800 × 500` のように 2 値を 1 セルに置く列）。
    pub(crate) fn wide_num(header: &'a str) -> Self {
        Self::new(header, ColWidth::WideNum)
    }

    /// 短ラベル列（階名・種別記号・OK/NG）。
    pub(crate) fn label(header: &'a str) -> Self {
        Self::new(header, ColWidth::Label)
    }

    /// 名称・符号列。
    pub(crate) fn name(header: &'a str) -> Self {
        Self::new(header, ColWidth::Name)
    }

    /// 長い文字列の列。
    pub(crate) fn text(header: &'a str) -> Self {
        Self::new(header, ColWidth::Text)
    }

    /// 行操作ボタンを 1 つ置く列（見出しなし）。
    pub(crate) fn actions() -> Self {
        Self::actions_n(1)
    }

    /// 行操作ボタンを `n` 個並べる列（見出しなし）。
    pub(crate) fn actions_n(n: u8) -> Self {
        debug_assert!(n >= 1, "操作列のボタンは 1 個以上");
        Self::new("", ColWidth::Actions(n))
    }

    /// 見出しにツールチップを付ける（列の意味・単位・既定値の補足）。
    pub(crate) fn hover(mut self, text: &'a str) -> Self {
        self.hover = Some(text);
        self
    }

    /// この列の初期幅 [pt]。トークンの幅と見出しの幅の大きい方を採る
    /// （見出しが切り詰められると列の意味が読めなくなるため）。
    fn width_pt(&self, ui: &egui::Ui) -> f32 {
        self.width
            .to_pt(ui)
            .max(text_width(ui, self.header) + cell_padding(ui))
    }

    /// 列定義を `egui_extras` の列へ変換する。
    ///
    /// リサイズの下限は `min_w` だが、操作列のように初期幅がそれより狭い列は
    /// 自分の初期幅を下限にする（下限が初期幅を上回ると、列が意図より広く開く）。
    fn to_column(&self, ui: &egui::Ui, min_w: f32) -> Column {
        let width = self.width_pt(ui);
        Column::initial(width).clip(true).at_least(min_w.min(width))
    }
}

/// 表が横スクロールを要するときの、スクロール領域の縦方向の最小高さ。
/// 行が数行しか見えない高さまで縮むと表として読めないため、行高から導出する
/// （TONMANUAL §4: テキストを内包する箱の寸法を固定 px で書かない）。
fn min_scrolled_height(row_h: f32) -> f32 {
    row_h * 8.0
}

/// 共通フォーマットの表を描く。`salt` は同一パネル内に複数の表を置くときの
/// egui Id 衝突（および列幅状態の共有）を避けるためのもので、表ごとに一意にする。
///
/// 列はすべてクリップ有効・リサイズ可能にする。
///
/// - **クリップ**: `egui_extras::Column` は既定でクリップしないため、セルの内容が
///   列幅を超えるとその行だけ以降のセルが右へ押し出され、行ごとに列位置がずれる
///   （内容の広い行が 1 つあるだけで表が崩れる）。またクリップ無効のセルは
///   テキストが折り返すため、`Column::auto()` と組むと「折り返した幅で列幅が
///   決まり、列が広がらないので折り返し続ける」という膠着に陥る。クリップを
///   有効にすると内容はセル内で切り詰められ、列位置は全行で揃う。
/// - **リサイズ**: `egui_extras` は列境界の縦線をリサイズハンドルとして描くため、
///   これを有効にすることが列区切りの縦線を出す手段でもある。
///
/// 縦横のスクロールは表の外側の [`egui::ScrollArea`] が持ち、`TableBuilder` 自身の
/// 縦スクロールは切る（`vscroll(false)`）。`egui_extras` は横スクロールを持たないため
/// 外側に出す必要があり、縦だけを内側に残すと縦スクロールバーが表の右端＝横スクロール
/// で画面外へ流れる位置に付いてしまうため、縦横ともに外側へ寄せている。
///
/// 行の仮想化（可視行だけ描く）は失われない。`TableBody::rows` は自身のスクロール
/// 状態ではなくクリップ矩形と表の先頭位置の差から可視範囲を求めるため、外側の
/// スクロール領域でも正しく働く。
pub(crate) fn standard_table(
    ui: &mut egui::Ui,
    salt: &str,
    cols: &[Col<'_>],
    n_rows: usize,
    mut row_fn: impl FnMut(&mut TableRow),
) {
    let row_h = crate::theme::table_row_height(ui);
    let min_w = min_column_width(ui);
    let spacing_x = ui.spacing().item_spacing.x;

    // 表の実幅は利用者の列リサイズで変わるため、前フレームの実測値を使う。
    // 初回は列定義からの見積もりで代用し、実測値との差が出たら再描画を要求する。
    let estimate: f32 = cols.iter().map(|c| c.width_pt(ui)).sum::<f32>()
        + spacing_x * cols.len().saturating_sub(1) as f32;
    let width_id = egui::Id::new(("table_content_width", salt));
    let content_w = ui.data(|d| d.get_temp::<f32>(width_id)).unwrap_or(estimate);
    // 表が可視幅より狭いときは可視幅を与える（余白に横スクロールバーを出さない）。
    let table_w = content_w.max(ui.available_width());

    let columns: Vec<Column> = cols.iter().map(|c| c.to_column(ui, min_w)).collect();

    let out = egui::ScrollArea::both()
        .id_salt((salt, "scroll"))
        // 横は可視幅いっぱいに広げ（スクロールバーをパネル幅で出す）、
        // 縦は内容ぶんに縮める（短い表がパネル高さを占有しないように）。
        .auto_shrink([false, true])
        .min_scrolled_height(min_scrolled_height(row_h))
        .show(ui, |ui| {
            // 横スクロール領域の内側では利用可能幅が無限になるため、表の幅を明示する。
            ui.set_max_width(table_w);

            let mut tb = TableBuilder::new(ui)
                .id_salt(salt)
                .striped(true)
                .resizable(true)
                .vscroll(false);
            for c in columns {
                tb = tb.column(c);
            }
            tb.header(row_h, |mut h| {
                for c in cols {
                    h.col(|ui| {
                        let resp = ui.strong(c.header);
                        if let Some(hover) = c.hover {
                            resp.on_hover_text(hover);
                        }
                    });
                }
            })
            .body(|body| {
                body.rows(row_h, n_rows, |mut row| row_fn(&mut row));
            });
        });

    let measured = out.content_size.x;
    ui.data_mut(|d| d.insert_temp(width_id, measured));
    if (measured - content_w).abs() > 0.5 {
        // 見積もりと実幅がずれたフレームは、横スクロールバーの要否が変わる。
        ui.ctx().request_repaint();
    }
}

// ===== セル =====

/// 文字列セル。列幅で切り詰められても内容を追えるよう、全文をホバーに出す。
/// 空文字のときはホバーを付けない（空のツールチップが出るのを避ける）。
pub(crate) fn text_cell(ui: &mut egui::Ui, text: &str) {
    let resp = ui.label(text);
    if !text.is_empty() {
        resp.on_hover_text(text);
    }
}

/// 値を持たない・対象外であることを示す文字列セル（淡色）。`hover` に理由を書く。
pub(crate) fn muted_cell(ui: &mut egui::Ui, text: &str, hover: &str) {
    ui.colored_label(crate::theme::GRAY_600, text)
        .on_hover_text(hover);
}

/// 行選択に連動する ID セル。クリックされたとき `true` を返す。
///
/// 選択時の背景（blue-500）と文字色（白）はテーマの `selection` が決めるため、
/// 呼び出し側で色を指定しないこと。
pub(crate) fn id_cell(ui: &mut egui::Ui, selected: bool, id: u32, hover: &str) -> bool {
    ui.add(egui::Button::selectable(selected, id.to_string()))
        .on_hover_text(hover)
        .clicked()
}

/// 行選択を持たない表の ID セル。
pub(crate) fn id_label(ui: &mut egui::Ui, id: u32) {
    ui.label(id.to_string());
}

/// 行削除ボタンのセル。押されたとき `true` を返す。
///
/// `hover` は削除の対象・巻き添えを説明する文言。`blocked` に理由を入れると
/// ボタンを無効化し、そちらを理由としてホバーに出す
/// （`on_hover_text` は無効なウィジェットでは表示されないため、無効時は
/// `on_disabled_hover_text` を使わないと理由が出ない）。
pub(crate) fn delete_cell(ui: &mut egui::Ui, hover: &str, blocked: Option<&str>) -> bool {
    let btn = ui.add_enabled(blocked.is_none(), egui::Button::new("🗑"));
    match blocked {
        Some(reason) => {
            btn.on_disabled_hover_text(reason);
            false
        }
        None => btn.on_hover_text(hover).clicked(),
    }
}

/// セル幅いっぱいの 1 行テキスト入力。列をリサイズすると入力欄も追従する
/// （ウィジェット側に固定幅を書くと列幅と食い違い、列がずれる・欄が見切れる）。
pub(crate) fn cell_text_edit(ui: &mut egui::Ui, buf: &mut String) -> egui::Response {
    let w = ui.available_width();
    ui.add(egui::TextEdit::singleline(buf).desired_width(w))
}

/// セル幅いっぱいの数値ドラッグ入力。`enabled` が `false` のときは無効表示にする
/// （行の種別によって意味を持たない諸元がある表で使う）。
pub(crate) fn cell_drag_value(
    ui: &mut egui::Ui,
    enabled: bool,
    dv: egui::DragValue<'_>,
) -> egui::Response {
    let size = egui::vec2(ui.available_width(), ui.available_height());
    ui.add_enabled_ui(enabled, |ui| ui.add_sized(size, dv))
        .inner
}

/// セル幅いっぱいのコンボボックス。
pub(crate) fn cell_combo<R>(
    ui: &mut egui::Ui,
    id_salt: impl std::hash::Hash,
    selected_text: impl Into<egui::WidgetText>,
    contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<Option<R>> {
    let w = ui.available_width();
    egui::ComboBox::from_id_salt(id_salt)
        .selected_text(selected_text)
        .width(w)
        .show_ui(ui, contents)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// フォントを読み込んだテスト用 `Ui` でクロージャを実行する。
    ///
    /// `egui::__run_test_ui` は CPU 時間の節約のためフォントを読み込まない
    /// （`FontDefinitions::empty()`）ので、文字幅が常に 0 になり本モジュールの
    /// 幅の導出を検証できない。既定フォントを読み込み、アプリと同じスタイル
    /// （[`crate::theme::apply_theme`] のタイポスケール）を適用して測る。
    fn run_test_ui(mut add_contents: impl FnMut(&mut egui::Ui)) {
        let ctx = egui::Context::default();
        ctx.set_fonts(egui::FontDefinitions::default());
        crate::theme::apply_theme(&ctx);
        let _ = ctx.run_ui(Default::default(), |ui| add_contents(ui));
    }

    /// ID 列は 5 桁の ID とボタン余白が収まる幅になる（`Column::auto()` 時代に
    /// 3 桁の ID が「10 / 0」と折り返していた症状の再発防止）。
    #[test]
    fn id_column_fits_five_digits() {
        run_test_ui(|ui| {
            let w = Col::id().width_pt(ui);
            assert!(
                w >= text_width(ui, "00000") + cell_padding(ui) * 2.0,
                "ID 列幅 {w} が 5 桁分に足りない"
            );
            // 見出し "ID" は 5 桁より狭いので、幅はトークン側で決まる
            assert!(w > text_width(ui, "ID") + cell_padding(ui));
        });
    }

    /// 幅トークンは用途の広さ順（操作 < 短ラベル < 名称 < 長文）を保つ。
    #[test]
    fn width_tokens_are_ordered() {
        run_test_ui(|ui| {
            let actions = ColWidth::Actions(1).to_pt(ui);
            let label = ColWidth::Label.to_pt(ui);
            let name = ColWidth::Name.to_pt(ui);
            let text = ColWidth::Text.to_pt(ui);
            assert!(actions < label, "{actions} < {label}");
            assert!(label < name, "{label} < {name}");
            assert!(name < text, "{name} < {text}");
            // 複合数値は単一の数値より広い
            assert!(ColWidth::Num.to_pt(ui) < ColWidth::WideNum.to_pt(ui));
        });
    }

    /// 見出しがトークン幅より長い列は、見出しが収まる幅まで広がる
    /// （クリップ有効下で見出しが切り詰められると列の意味が読めなくなるため）。
    #[test]
    fn column_widens_for_long_header() {
        run_test_ui(|ui| {
            let long = "剛性率Rs(≥0.6) の長い見出し";
            let col = Col::num(long);
            assert!(col.width_pt(ui) >= text_width(ui, long) + cell_padding(ui));
            assert!(col.width_pt(ui) > ColWidth::Num.to_pt(ui));
        });
    }

    /// `Col` は見出しとホバーを保持する。
    #[test]
    fn col_keeps_header_and_hover() {
        let col = Col::name("符号").hover("断面の呼び名");
        assert_eq!(col.header, "符号");
        assert_eq!(col.hover, Some("断面の呼び名"));
        assert_eq!(col.width, ColWidth::Name);
        assert_eq!(Col::actions().header, "");
        assert_eq!(Col::id().header, "ID");
        assert_eq!(Col::num("A").hover, None);
    }

    /// 横スクロール時のスクロール領域の最小高さは、行が数行しか見えない高さまで
    /// 縮まない（行高から導出しているので、フォントを変えても比が保たれる）。
    #[test]
    fn min_scrolled_height_keeps_several_rows() {
        run_test_ui(|ui| {
            let row_h = crate::theme::table_row_height(ui);
            assert!(min_scrolled_height(row_h) >= row_h * 4.0);
        });
    }

    /// リサイズの下限は 0 ではなく、3 桁が読める幅を残す。
    #[test]
    fn min_column_width_keeps_three_digits() {
        run_test_ui(|ui| {
            assert!(min_column_width(ui) >= text_width(ui, "000"));
            assert!(min_column_width(ui) < ColWidth::Name.to_pt(ui));
        });
    }
}
