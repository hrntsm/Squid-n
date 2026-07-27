//! グリッド操作の純ロジック層（egui 非依存）。
//!
//! 座標系（`CellRef.col`）は**データ列のみ**を数える（dev_docs/specs/グリッド操作.md
//! §3.2）。行ヘッダ列（ID）はグリッドの関知外で、テーブル側が描画し、クリックを
//! `select_row` 呼び出しに変換する。この分離により「編集不可の ID 列への貼り付け」
//! という不正カテゴリが構造的に消滅し、ペースト検証の不正は「列はみ出し」
//! 「値パース失敗」の 2 種だけになる。

/// セル参照（0 始まりの行・データ列）
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CellRef {
    pub row: usize,
    pub col: usize,
}

/// 矩形選択範囲（両端含む）
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SelRect {
    pub r0: usize,
    pub r1: usize,
    pub c0: usize,
    pub c1: usize,
}

impl SelRect {
    pub fn contains(&self, row: usize, col: usize) -> bool {
        (self.r0..=self.r1).contains(&row) && (self.c0..=self.c1).contains(&col)
    }
}

/// 選択モード／編集モードの状態機械。
/// anchor と cursor の対で矩形選択を表し、editing が Some の間は編集モード。
/// anchor が「アクティブセル」（Excel の枠付きセル。編集・通常移動の基準）で、
/// cursor は Shift 拡張・ドラッグで伸びていく側の端。
pub struct GridState {
    pub rows: usize,
    pub cols: usize,
    pub anchor: CellRef,
    pub cursor: CellRef,
    pub editing: Option<CellRef>,
    /// 選択が生きているか。false の間は選択なし（表外クリック・Esc で解除）。
    /// Excel と異なり「選択なし」状態を持つ: 本実装ではテーブルが 3D ビュー等と
    /// 同居するため、他パネル作業中に選択表示が残るのはノイズになる（§4.4）。
    /// anchor/cursor は解除中も保持し、矢印キー等での復帰位置に使う。
    pub active: bool,
}

impl GridState {
    /// active=false（選択なし）で開始する
    pub fn new(rows: usize, cols: usize) -> Self {
        let origin = CellRef { row: 0, col: 0 };
        Self {
            rows,
            cols,
            anchor: origin,
            cursor: origin,
            editing: None,
            active: false,
        }
    }

    /// 選択を完全解除する（anchor/cursor は復帰位置として保持）
    pub fn deactivate(&mut self) {
        self.active = false;
    }

    fn clamp(&self, cell: CellRef) -> CellRef {
        CellRef {
            row: cell.row.min(self.rows.saturating_sub(1)),
            col: cell.col.min(self.cols.saturating_sub(1)),
        }
    }

    /// グリッドが空（行または列が 0）か。空の間は選択状態になれない
    /// （不変条件: rows == 0 || cols == 0 の間は active にならない。
    /// これを破ると空グリッドへのコピー等で範囲外アクセスになる）
    fn is_empty(&self) -> bool {
        self.rows == 0 || self.cols == 0
    }

    /// クリック: 通常はアクティブセルを移す。Shift+クリックは
    /// アクティブセルを動かさず選択端（cursor）だけを伸ばす（Excel と同じ）。
    pub fn click(&mut self, cell: CellRef, extend: bool) {
        if self.is_empty() {
            return;
        }
        self.active = true;
        self.cursor = self.clamp(cell);
        if !extend {
            self.anchor = self.cursor;
        }
    }

    /// ドラッグ選択の継続。cursor 側だけが動く（起点＝アクティブセルは不動）
    pub fn drag_to(&mut self, cell: CellRef) {
        if self.is_empty() {
            return;
        }
        self.cursor = self.clamp(cell);
    }

    /// 通常移動はアクティブセル（anchor）基準・選択を畳む。
    /// Shift 拡張は cursor 側だけを動かす（Excel と同じ）。
    /// 選択解除中は移動せず、直前のアクティブセルで選択を復帰する（§4.4）。
    pub fn move_cursor(&mut self, dr: isize, dc: isize, extend: bool) {
        if self.is_empty() {
            return;
        }
        if !self.active {
            self.active = true;
            self.cursor = self.anchor;
            return;
        }
        let base = if extend { self.cursor } else { self.anchor };
        let next = CellRef {
            row: base.row.saturating_add_signed(dr),
            col: base.col.saturating_add_signed(dc),
        };
        self.cursor = self.clamp(next);
        if !extend {
            self.anchor = self.cursor;
        }
    }

    /// 行選択: その行のデータ列すべてを選択する。呼び出し元は行ヘッダの
    /// クリックやナビゲータなど（grid_core は誰が呼ぶかを知らない）。
    /// extend（Shift やドラッグ）は Excel 準拠で「起点行〜対象行 × 全列」に
    /// 広げる（アクティブセルの列は先頭列に寄る）。
    pub fn select_row(&mut self, row: usize, extend: bool) {
        if self.is_empty() {
            return;
        }
        self.active = true;
        let row = row.min(self.rows.saturating_sub(1));
        self.cursor = CellRef {
            row,
            col: self.cols.saturating_sub(1),
        };
        let anchor_row = if extend { self.anchor.row } else { row };
        self.anchor = CellRef {
            row: anchor_row,
            col: 0,
        };
    }

    /// 列選択: その列の全行を選択する（列ヘッダクリック。select_row と対称）。
    /// extend は Excel 準拠で「起点列〜対象列 × 全行」に広げる。
    pub fn select_col(&mut self, col: usize, extend: bool) {
        if self.is_empty() {
            return;
        }
        self.active = true;
        let col = col.min(self.cols.saturating_sub(1));
        self.cursor = CellRef {
            row: self.rows.saturating_sub(1),
            col,
        };
        let anchor_col = if extend { self.anchor.col } else { col };
        self.anchor = CellRef {
            row: 0,
            col: anchor_col,
        };
    }

    /// 全選択（左上コーナークリック・Ctrl+A）。アクティブセルは先頭セルになる
    pub fn select_all(&mut self) {
        if self.is_empty() {
            return;
        }
        self.active = true;
        self.anchor = CellRef { row: 0, col: 0 };
        self.cursor = CellRef {
            row: self.rows.saturating_sub(1),
            col: self.cols.saturating_sub(1),
        };
    }

    /// 行数変更（ペーストでの行追加・削除・リセット）後に選択を表内へ収める
    pub fn clamp_selection(&mut self) {
        self.anchor = self.clamp(self.anchor);
        self.cursor = self.clamp(self.cursor);
    }

    pub fn rect(&self) -> SelRect {
        SelRect {
            r0: self.anchor.row.min(self.cursor.row),
            r1: self.anchor.row.max(self.cursor.row),
            c0: self.anchor.col.min(self.cursor.col),
            c1: self.anchor.col.max(self.cursor.col),
        }
    }

    /// アクティブセル（anchor）で編集モードへ（選択は単一セルに畳む）
    pub fn begin_edit(&mut self) {
        self.cursor = self.anchor;
        self.editing = Some(self.anchor);
    }

    pub fn end_edit(&mut self) {
        self.editing = None;
    }
}

/// 矩形範囲を TSV 化する（コピー用）。セル文字列は**内部値の正準文字列**を
/// 渡すこと（表示用に丸めた文字列を使うと、コピーで精度が落ちる。§5.1）
pub fn rect_to_tsv(rect: SelRect, cell_text: impl Fn(usize, usize) -> String) -> String {
    (rect.r0..=rect.r1)
        .map(|r| {
            (rect.c0..=rect.c1)
                .map(|c| cell_text(r, c))
                .collect::<Vec<_>>()
                .join("\t")
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// クリップボード文字列を TSV ブロックへ（Excel の CRLF・末尾改行を吸収）
pub fn parse_tsv(text: &str) -> Vec<Vec<String>> {
    let mut lines: Vec<&str> = text
        .split('\n')
        .map(|l| l.strip_suffix('\r').unwrap_or(l))
        .collect();
    while lines.last().is_some_and(|l| l.is_empty()) {
        lines.pop();
    }
    lines
        .iter()
        .map(|l| l.split('\t').map(str::to_string).collect())
        .collect()
}

/// コピー元ブロックを選択範囲へタイル展開する（Excel 互換。§5.2.2）。
/// 選択範囲の行数・列数が**ともに**ブロックの整数倍（かつどちらかが拡大）の
/// ときだけ繰り返しで埋めたブロックを返し、それ以外は元のブロックをそのまま返す。
/// 例: 1×1 のコピーを 3×2 の選択へ → 3×2 に複製。2×1 を 4×1 へ → 2 回繰り返し。
pub fn tile_block(block: &[Vec<String>], sel_rows: usize, sel_cols: usize) -> Vec<Vec<String>> {
    let br = block.len();
    let bc = block.iter().map(Vec::len).max().unwrap_or(0);
    if br == 0 || bc == 0 {
        return block.to_vec();
    }
    let fits = sel_rows.is_multiple_of(br)
        && sel_cols.is_multiple_of(bc)
        && (sel_rows / br > 1 || sel_cols / bc > 1);
    if !fits {
        return block.to_vec();
    }
    (0..sel_rows)
        .map(|r| {
            (0..sel_cols)
                .map(|c| block[r % br].get(c % bc).cloned().unwrap_or_default())
                .collect()
        })
        .collect()
}

/// テーブルアダプタ（§3.4）。汎用グリッドレイヤとテーブルの境界で、
/// ドメイン知識（セルの型・行追加の可否と方法）はすべてこちら側に置く。
/// 第 2 弾以降のテーブル展開は「アダプタ実装の追加」だけで済む。
pub trait GridAdapter {
    fn rows(&self) -> usize;
    fn cols(&self) -> usize;

    /// コピー・編集開始用のセル文字列。**内部値の正準文字列**を返す
    /// （表示用に丸めた文字列を返さない。§5.1）
    fn cell_text(&self, row: usize, col: usize) -> String;

    /// 行ヘッダ（ID 列）の表示文字列。既定は 1 始まりの行番号。
    /// ID＝配列位置のテーブルは ID 表記に合わせて上書きする（節点 = 0 始まり ID）
    fn row_label(&self, row: usize) -> String {
        (row + 1).to_string()
    }

    /// ペースト・編集確定の 1 セル分の検証。Err はセル単位の不正理由。
    /// [`plan_paste`]（§3.3）がこれを全セルに適用する
    fn validate_cell(&self, row: usize, col: usize, text: &str) -> Result<(), String>;

    /// 検証済みセル群の適用。squid-n-edit の複合コマンド 1 個に落とす（§3.5）。
    /// append_rows > 0 なら先に行を追加する（追加行の貼り付け対象外の列は
    /// アダプタの既定値）。呼び出し規約:
    /// - 通常のペースト: cells 全部 + append_rows = はみ出し行数（自動追加。§5.2.5）
    /// - 行追加非対応テーブル: widget が cells を row < rows() にフィルタして渡し、
    ///   append_rows = 0（はみ出し分は切り捨て）
    /// - 新規行プレースホルダでの編集確定: cells = その 1 セル、append_rows = 1
    fn apply_block(&mut self, cells: &[(usize, usize, String)], append_rows: usize);

    /// 選択範囲クリア（Delete）。クリアの意味（0 埋め・既定値・禁止）は
    /// アダプタが決める。cells は選択が跨ぐ**実データ行**のセルのみ
    /// （プレースホルダは渡されない）。クリアしたセル数を返す
    /// （クリア非対応のテーブルは 0 を返し、widget が理由をログする）
    fn clear_cells(&mut self, cells: &[(usize, usize)]) -> usize;

    /// はみ出し行の追加・新規行プレースホルダに対応するか
    /// （節点 = true、非対応テーブル = false）
    fn can_append_rows(&self) -> bool;

    /// 行削除に対応するか
    fn can_delete_rows(&self) -> bool;

    /// 1 行分の削除可否検証。参照中（部材が接続された節点等）なら
    /// Err に「なぜ削除できないか（参照元）」を返す
    fn validate_row_deletion(&self, row: usize) -> Result<(), String>;

    /// 検証済みの行削除。squid-n-edit の複合コマンド 1 個に落とす。
    /// ID＝配列位置の繰り上げと undo の整合のため、行番号の**降順**で
    /// DeleteNode を並べること
    fn delete_rows(&mut self, rows: &[usize]);
}

/// 編集確定の結果。widget がログ・フラッシュ表示に変換する
#[derive(Debug, PartialEq, Eq)]
pub enum CommitOutcome {
    /// 空のまま確定 = 「変更なし」（§4.3。Backspace→Enter は no-op）
    NoChange,
    /// 適用した（appended = 新規行プレースホルダで行追加を伴ったか）
    Applied { appended: bool },
    /// 不正値のため変更しなかった（理由）
    Rejected(String),
}

/// 編集確定（1 セル）の純ロジック。検証・空確定の扱い・プレースホルダの
/// 行追加判定を egui 非依存で行い、適用はアダプタへ委譲する（§4.3・§4.5）。
pub fn commit_cell_text(adapter: &mut dyn GridAdapter, cell: CellRef, raw: &str) -> CommitOutcome {
    let t = raw.trim();
    if t.is_empty() {
        return CommitOutcome::NoChange;
    }
    if let Err(reason) = adapter.validate_cell(cell.row, cell.col, t) {
        return CommitOutcome::Rejected(reason);
    }
    let appended = cell.row >= adapter.rows();
    // プレースホルダへの確定 = 行追加＋値設定（アダプタが複合コマンド 1 個に落とす）
    adapter.apply_block(&[(cell.row, cell.col, t.to_string())], appended as usize);
    CommitOutcome::Applied { appended }
}

/// ペーストブロックのセル数上限（行数×最大列数。§5.2.1）。
/// Excel の「列全体コピー」（104 万行）の誤ペーストで UI がフリーズするのを
/// 防ぐ暴発ガードであり、実務のモデル規模（数千〜数万セル）には影響しない。
pub const MAX_PASTE_CELLS: usize = 100_000;

/// ペースト計画。全セルの検証を通過した場合のみ得られる（all-or-nothing）。
#[derive(Debug)]
pub struct PastePlan {
    /// 適用するセル（行, データ列, セル文字列）。行は表の末尾を超えることがある。
    /// 値の型変換はアダプタが適用時に行う（検証済みなので失敗しない）
    pub set: Vec<(usize, usize, String)>,
    /// 表の末尾を超える行数（自動追加の対象。§5.2.5）
    pub extra_rows: usize,
    /// 空セルとしてスキップした数（既存値維持）
    pub skipped_empty: usize,
    /// 貼り付け起点（適用後にブロック範囲を選択状態へ戻すために保持）
    pub anchor: CellRef,
    /// ペーストブロックの行数・列数（空セル含む見かけの矩形）
    pub block_rows: usize,
    pub block_cols: usize,
}

/// ペースト検証: 1 セルでも不正があれば Err（理由の一覧）を返し、何も適用しない
/// （all-or-nothing。§5.2.3）。検証を全部通ってから初めて適用する、という順序を
/// 変えないこと。
///
/// - 空セルは「変更なし」マーカーとして扱い、不正には数えない（§5.2.4）
/// - セル値の検証はアダプタの `validate_cell` へ委譲する（引数 `validate`）。
///   行ヘッダ列は座標系に存在しないため、不正カテゴリは「列はみ出し」と
///   「validate の Err（値パース失敗等）」の 2 つだけになる
/// - ブロックが [`MAX_PASTE_CELLS`] を超える場合は全体拒否（§5.2.1）
pub fn plan_paste(
    block: &[Vec<String>],
    anchor: CellRef,
    rows: usize,
    cols: usize,
    validate: impl Fn(usize, usize, &str) -> Result<(), String>,
) -> Result<PastePlan, Vec<String>> {
    let block_rows = block.len();
    let block_cols = block.iter().map(Vec::len).max().unwrap_or(0);
    let cells = block_rows.saturating_mul(block_cols);
    if cells > MAX_PASTE_CELLS {
        return Err(vec![format!(
            "ペーストブロックが大きすぎます（{block_rows}行×{block_cols}列 = {cells} セル。上限 {MAX_PASTE_CELLS} セル）"
        )]);
    }
    let mut errors = Vec::new();
    let mut set = Vec::new();
    let mut skipped_empty = 0usize;
    let mut max_row = 0usize;
    for (i, line) in block.iter().enumerate() {
        for (j, text) in line.iter().enumerate() {
            let (r, c) = (anchor.row + i, anchor.col + j);
            let t = text.trim();
            if t.is_empty() {
                skipped_empty += 1;
                continue;
            }
            if c >= cols {
                errors.push(format!(
                    "ブロック{}行{}列目「{}」: 貼り付け先が表の列範囲外",
                    i + 1,
                    j + 1,
                    t
                ));
                continue;
            }
            match validate(r, c, t) {
                Ok(()) => {
                    set.push((r, c, t.to_string()));
                    max_row = max_row.max(r);
                }
                Err(reason) => {
                    errors.push(format!(
                        "ブロック{}行{}列目「{}」: {}",
                        i + 1,
                        j + 1,
                        t,
                        reason
                    ));
                }
            }
        }
    }
    if !errors.is_empty() {
        return Err(errors);
    }
    let extra_rows = if set.is_empty() {
        0
    } else {
        (max_row + 1).saturating_sub(rows)
    };
    Ok(PastePlan {
        set,
        extra_rows,
        skipped_empty,
        anchor,
        block_rows,
        block_cols,
    })
}
