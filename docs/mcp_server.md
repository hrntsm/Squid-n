# MCP サーバ

## 概要

`squid-n-mcp` は Squid-N の構造モデルを [Model Context Protocol (MCP)](https://modelcontextprotocol.io/) 経由で AI エージェントに公開するサーバである。標準入出力（stdio）をトランスポートとして動作し、接続したクライアント（Claude Code、Claude Desktop など）から次のことができる。

- 構造モデル（節点・部材・断面）の照会
- 数量積算（コンクリート・型枠・鉄筋・鉄骨）の集計
- 解析（線形静解析・固有値解析・プッシュオーバー・時刻歴応答解析・断面検定・終局検定）の非同期実行
- ジョブ状態のポーリングと解析結果の取得

MCP プロトコル層（`rmcp`/`tokio` に依存する部分）はすべて Cargo の機能フラグ `mcp` の配下にあり、**`mcp` は非デフォルト**（`default = []`）である。フラグを付けずにビルドした場合、MCP サーバ（起動バイナリ・ツール群）はビルド対象に含まれない。

## ビルドと起動

```bash
# ビルドのみ（バイナリは target/debug/squid-n-mcp に生成される。
# --release 付きなら target/release/squid-n-mcp）
cargo build -p squid-n-mcp --features mcp

# モデルファイルを指定して起動
cargo run -p squid-n-mcp --features mcp -- model.scz

# 引数を省略すると空モデルで起動する
cargo run -p squid-n-mcp --features mcp
```

- 起動時の第1引数（`.scz` パス）がモデルの読み込み元になる。省略時は空モデルで起動する。
- 解析結果ストアのディレクトリは環境変数 `SQUID_N_RESULT_DIR` で指定できる。未設定の場合は OS 一時ディレクトリ配下の `squid-n-mcp-results` を使う。

```bash
SQUID_N_RESULT_DIR=/path/to/results cargo run -p squid-n-mcp --features mcp -- model.scz
```

> **注意**: stdout は MCP の JSON-RPC トランスポートそのものである。ログや診断出力を stdout に書くと、クライアントは壊れたフレームとして接続を切断する。

よくあるエラー:

| 状況 | 結果 |
|---|---|
| `--features mcp` を `squid-n-mcp` 以外のクレート（例: `-p squid-n-app`）に付けて実行する | そのクレートに `mcp` という機能フラグが無いためビルドエラーになる |
| `squid-n-mcp` を `--features mcp` なしでビルド/実行しようとする | 起動バイナリ自体が `required-features = ["mcp"]` でゲートされているため、バイナリが存在せず実行できない |

## クライアント設定例

### Claude Code

```bash
claude mcp add squid-n -- /path/to/squid-n-mcp model.scz
```

もしくはプロジェクトの `.mcp.json` に登録する。

```json
{
  "mcpServers": {
    "squid-n": {
      "command": "/path/to/squid-n-mcp",
      "args": ["model.scz"]
    }
  }
}
```

### Claude Desktop

`claude_desktop_config.json` に以下のエントリを追加する。

```json
{
  "mcpServers": {
    "squid-n": {
      "command": "/path/to/squid-n-mcp",
      "args": ["model.scz"],
      "env": {
        "SQUID_N_RESULT_DIR": "/path/to/results"
      }
    }
  }
}
```

## ツール一覧

公開されている MCP ツールは以下の5個。

### `model_query`

節点・部材・断面を検索する。

| 引数 | 型 | 必須/任意 | 意味 |
|---|---|---|---|
| `kind` | `String` | 必須 | `"node"`/`"nodes"`、`"member"`/`"members"`/`"element"`/`"elements"`、`"section"`/`"sections"` のいずれか。それ以外は空配列を返す |
| `filter` | `Option<String>` | 任意 | 各アイテムを JSON 文字列化した内容に対する部分一致フィルタ |

返り値:

```json
{ "items": [ /* 検索結果の JSON オブジェクト配列 */ ] }
```

- `node`: `{ "id", "coord", "story" }`
- `member`/`element`: `{ "id", "kind", "nodes", "section", "material" }`。部材付帯情報（ハンチ・継手位置）があれば `haunch_i`/`haunch_j`（`length`/`depth_increase`/`width_increase`）と `joints`（`distance`/`kind`）を追加
- `section`: `{ "id", "name", "area", "iy", "iz" }`

### `quantity_takeoff`

数量積算（コンクリート体積・型枠面積・鉄筋/鉄骨重量の概算）を集計する。

| 引数 | 型 | 必須/任意 | 意味 | 既定値 |
|---|---|---|---|---|
| `group_by` | `Option<String>` | 任意 | `"category"`（部位別）/`"story"`（階別）/`"steel"`（鉄骨種類別）/`"rebar"`（鉄筋径別）/`"detail"`（明細） | `"category"` |

返り値:

```json
{ "rows": [ /* group_by に応じた行 */ ], "totals": { "concrete_m3": .., "formwork_m2": .., "rebar_t": .., "steel_t": .., "rebar_joints": .. }, "notes": [ /* 注記 */ ] }
```

`rows` の形は `group_by` により異なる。

| `group_by` | 行の形 |
|---|---|
| `category`（既定） | `{ category, concrete_m3, formwork_m2, rebar_t, steel_t, rebar_joints }` |
| `story` | `{ story, concrete_m3, formwork_m2, rebar_t, steel_t, rebar_joints }` |
| `steel` | `{ section, length_m, weight_t }` |
| `rebar` | `{ dia_mm, length_m, weight_t }` |
| `detail` | `{ elem, slab, label, story, category, structure, concrete_m3, formwork_m2, rebar_t, steel_t, rebar_joints }` |

数量の算定式は[計算根拠 11. 数量積算](./calc_basis/11_数量積算/README.md)を参照。

### `analysis_run`

解析を非同期で実行する。ジョブを登録し `job_id` を即座に返す（計算完了は待たない）。

| 引数 | 型 | 必須/任意 | 意味 | 既定値 |
|---|---|---|---|---|
| `kind` | `JobKind` | 必須 | `LinearStatic`/`Eigen`/`Pushover`/`TimeHistory`/`DesignCheck`/`UltimateCheck` | — |
| `load_case` | `Option<u32>` | 任意 | LinearStatic/DesignCheck/UltimateCheck: 対象荷重ケース ID | 先頭の荷重ケース |
| `n_modes` | `Option<usize>` | 任意 | Eigen: モード数 | 3 |
| `dir` | `Option<String>` | 任意 | Pushover/TimeHistory: 加力・入力方向。`"X"`/`"Y"` 以外はエラー | `"X"` |
| `steps` | `Option<usize>` | 任意 | Pushover: 最大ステップ数 | 50 |
| `max_disp` | `Option<f64>` | 任意 | Pushover: 目標変位 [mm] | 500 |
| `dt` | `Option<f64>` | 任意 | TimeHistory: サンプル波の時間刻み [s] | 0.01 |
| `duration` | `Option<f64>` | 任意 | TimeHistory: サンプル波の継続時間 [s] | 2.0 |
| `period` | `Option<f64>` | 任意 | TimeHistory: サンプル波の周期 [s] | 0.5 |
| `amp` | `Option<f64>` | 任意 | TimeHistory: サンプル波の振幅 [mm/s²] | 1000 |

返り値:

```json
{ "job_id": "job-0" }
```

`kind` ごとに使用するパラメータと概略処理:

| `kind` | 使用パラメータ | 処理概要 |
|---|---|---|
| `LinearStatic` | `load_case` | 指定/先頭の荷重ケースで線形静解析 |
| `Eigen` | `n_modes` | 固有値解析（周期・刺激係数・有効質量） |
| `Pushover` | `dir`, `steps`, `max_disp` | プッシュオーバー解析（漸増静的） |
| `TimeHistory` | `dir`, `dt`, `duration`, `period`, `amp` | サンプル波（`amp * sin(ωt) * e^{-0.3t}`）による時刻歴応答解析。減衰は剛性比例減衰（h=0.02、1次固有振動数使用）固定 |
| `DesignCheck` | `load_case` | 荷重ケースの線形静解析結果に対する断面検定（鋼/RC 許容応力度、危険断面位置基準）。検定条件は長期固定 |
| `UltimateCheck` | `load_case` | RC 部材の終局せん断・付着・軸余裕度、CFT 柱の軸終局検定（靭性保証型耐震設計指針） |

### `analysis_status`

ジョブの状態を取得する。

| 引数 | 型 | 必須/任意 | 意味 |
|---|---|---|---|
| `job_id` | `String` | 必須 | `analysis_run` が返した ID |

返り値:

```json
{ "id": "job-0", "kind": "LinearStatic", "status": { /* Queued | Running{progress} | Done{result_ref} | Failed{error} */ } }
```

- `status` は次のいずれかの形を取る。
  - `"Queued"`
  - `{ "Running": { "progress": 0.0 } }`
  - `{ "Done": { "result_ref": "<サマリ JSON 文字列>" } }`
  - `{ "Failed": { "error": "<エラーメッセージ>" } }`
- `job_id` が存在しない場合はエラー（`invalid_params`）。

### `result_get`

解析結果ストアから結果を取得する。

| 引数 | 型 | 必須/任意 | 意味 |
|---|---|---|---|
| `case` | `u32` | 必須 | 荷重ケース ID（Eigen の結果は `case=0` 固定） |
| `kind` | `String` | 必須 | `"NodalDisp"`/`"MemberForce"`/`"Modal"`/`"TimeHistory"` のいずれか |
| `node_ids` | `Option<Vec<u32>>` | 任意 | 節点 ID での絞り込み |
| `member_ids` | `Option<Vec<u32>>` | 任意 | 部材 ID での絞り込み |
| `step_range` | `Option<Vec<u64>>` | 任意 | `[start, end)` のちょうど2要素。それ以外はエラー |

返り値:

```json
{ "case": 1, "kind": "NodalDisp", "rows": [ /* 明細行 */ ], "truncated": false }
```

`(case, kind)` の組が結果ストアに無い場合はエラーになる（`analysis_run` を先に実行する必要がある）。

## 解析ジョブのフロー

1. `analysis_run` を呼び、`job_id` を受け取る（この時点でジョブは `Running` としてバックグラウンド実行される）
2. `analysis_status(job_id)` を `Done`/`Failed` になるまでポーリングする
3. `Done` の場合、`analysis_status` の `result_ref` にサマリ JSON が入っている。さらに詳細な明細（節点変位・部材内力など）が必要なら `result_get` を呼ぶ

```json
// 1. analysis_run(kind="LinearStatic") の返り値
{ "job_id": "job-0" }

// 2. analysis_status(job_id="job-0") の返り値（Done になった時点）
{
  "id": "job-0",
  "kind": "LinearStatic",
  "status": {
    "Done": {
      "result_ref": "{\"kind\":\"LinearStatic\",\"case\":1,\"n_nodes\":2,\"n_member_force_rows\":3,\"max_abs_disp\":1.23,\"store\":{\"case\":1,\"kinds\":[\"NodalDisp\",\"MemberForce\"]}}"
    }
  }
}

// 3. result_get(case=1, kind="NodalDisp") の返り値
{ "case": 1, "kind": "NodalDisp", "rows": [ { "node_id": 1, "ux": 0.0, "uy": 0.0, "uz": 1.23, "rx": 0.0, "ry": 0.0, "rz": 0.0 } ], "truncated": false }
```

`Eigen` ジョブの結果は結果ストアに `case=0` として格納される（実荷重ケース番号との衝突を避けるための固定値）。`result_get(case=0, kind="Modal")` で取得する。

## 結果ストアのスキーマ

`result_get` の `kind` に指定できる4種のスキーマは以下のとおり（すべて Arrow の `UInt32`/`UInt64`/`Float64` 列で構成される）。

### NodalDisp

| 列名 | 型 |
|---|---|
| `node_id` | UInt32 |
| `ux`, `uy`, `uz` | Float64 |
| `rx`, `ry`, `rz` | Float64 |

### MemberForce

| 列名 | 型 | 備考 |
|---|---|---|
| `elem_id` | UInt32 | |
| `pos` | Float64 | 評価位置（0..1 の正規化座標） |
| `n`, `qy`, `qz`, `mx`, `my`, `mz` | Float64 | 部材内力（軸力・せん断・ねじり・曲げ） |

### Modal

| 列名 | 型 |
|---|---|
| `mode` | UInt32 |
| `period` | Float64 |
| `omega2` | Float64 |
| `part_x`, `part_y`, `part_z` | Float64 |
| `eff_x`, `eff_y`, `eff_z` | Float64 |

### TimeHistory

| 列名 | 型 |
|---|---|
| `step` | UInt64 |
| `time` | Float64 |
| `node_id` | UInt32 |
| `ux`, `uy`, `uz` | Float64 |
| `rx`, `ry`, `rz` | Float64 |

`result_get` が1回に返す行数の上限は 10,000 行。超過分は切り詰められ、応答の `truncated` が `true` になる。

## 制約・注意点

- **モデルの読み込みは起動時引数のみ**。`.scz` ファイルパスをコマンドライン引数として渡す方法しかなく、実行中にモデルを差し替えたり読み込んだりする `model_open` のような MCP ツールは存在しない。
- **編集・保存系ツールは公開されていない**。サーバ内部状態は `UndoStack`（`squid-n-edit`）を保持しているが、現状どの MCP ツールからも参照されておらず、モデルの変更・保存を行う手段は無い。
- `Pushover`・`TimeHistory`・`UltimateCheck` ジョブは結果ストアへ書き込まない。対応する結果スキーマが無い、あるいはスキーマが要求する粒度（全節点×全ステップ）のデータを持たないため、結果は `analysis_status` の `result_ref`（サマリ JSON）としてのみ得られる。
- `DesignCheck` ジョブの検定結果（OK/NG・検定比）自体もサマリにのみ含まれる。結果ストアに書き込まれるのは検定の元データである `MemberForce` のみ。
