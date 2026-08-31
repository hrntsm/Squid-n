# V&V レポート（検証・妥当性確認）

本ディレクトリは、Squid-n の各要素・各設計式に対する V&V（Verification & Validation）レポートを格納する。

## クイックリンク

| 一覧 | 内容 |
|------|------|
| [**未検証一覧**](未検証一覧.md) | ❌/🔶 の集約チェックリスト（パッと見る用） |
| [§レポート目録](#レポート目録) | 各 `.md` レポートへの索引 |
| [§索引（要素→テスト）](#索引要素設計式--テスト) | コード上のテスト対応表（#1–#27） |
| [pending_items.md](pending_items.md) | P9 仕様乖離の歴史的記録（訂正履歴含む） |

## V&V の定義

| 用語 | 意味 |
|------|------|
| Verification（検証） | 「式を正しく解いているか」。理論解・手計算・規準例題との一致で確認 |
| Validation（妥当性確認） | 「正しい現象を表しているか」。実験・実測との一致で確認 |

## 「参照実装」について

本セクションで「参照実装」とは、検証の突合（クロスチェック）に参照解として用いた
市販の構造計算一貫プログラムとその計算マニュアルを指す。参照実装は 2 次資料であり、
計算根拠（[calc_basis](../../docs/calc_basis/README.md)）の出典としては用いない。計算根拠は
法令・告示・学会規準等の 1 次資料で示し、参照実装との突合はあくまで
「同種の実務計算と結果・機能範囲が整合しているか」の検証記録として保持する。

## レポート構造

各エントリは以下の項目を持つ:

- **対象**: 検証対象（例: ティモシェンコ梁 / パネルゾーン / Ai分布 / プッシュオーバー機構）
- **参照解の出典**: 理論式 / 実験 / 商用ソフト / 規準例題 / 添付資料
- **入力モデル**: 再現可能な定義（テストに対応づけ）
- **許容差**: 厳密=1e-9 / 収束=±% / 規準例題照合
- **結果**: 合否・差分・グラフ

## 検証の性格

| 区分 | 該当項目 | 許容差 |
|------|----------|--------|
| 厳密一致 | IIE 梁・CMQ・σ=M/Z・剛性率 | 1e-9 |
| 収束 | MITC4 板・固有値・時刻歴 | ±5% |
| 規準例題照合 | Ai・Ds・許容応力度 | 告示値一致 |

## テスト階層

| レベル | 内容 | ツール |
|--------|------|--------|
| 単体 | 要素剛性・履歴則・断面算定式 | `cargo test`, `approx` |
| 性質 | 剛性対称性・エネルギー保存・パッチテスト | `proptest` |
| 回帰 | 履歴ループ・スケルトン形状 | `insta`（スナップショット） |
| 数値照合 | 理論解（梁・板・SDOF/MDOF） | 専用ベンチ集 |
| ベンチマーク照合 | 既往実験・商用ソフト | 検証レポート |
| 性能 | 速度回帰 | `criterion` + CI 閾値 |
| 決定性 | 同一入力ビット一致 | 専用テスト |

## レポート目録

各レポートの照合結果・修正履歴・残課題の詳細。**未完了の要約**は
[未検証一覧.md](未検証一覧.md) を参照。

### 参照実装マニュアル照合

| レポート | 対象章 | 判定 | 備考 |
|----------|--------|------|------|
| [load_calculation_review.md](load_calculation_review.md) | 01 荷重計算 | 🔶 | 風荷重等のギャップ |
| [剛性計算_参照実装照合.md](剛性計算_参照実装照合.md) | 02 剛性計算 | 🔶 | 免震・製品要素は未実装 |
| [応力解析_参照実装照合.md](応力解析_参照実装照合.md) | 03 応力解析 | 🔶 | 制振間柱等 |
| [断面検定_参照実装照合.md](断面検定_参照実装照合.md) | 04 断面検定 | 🔶 | C 節に残置項目 |
| [非線形モデル_参照実装照合.md](非線形モデル_参照実装照合.md) | 05 非線形モデル | 🔶 | 免震 UI・分割ロジック等 |
| [終局検定_参照実装照合.md](終局検定_参照実装照合.md) | 06 終局検定 | 🔶 | Vu・二軸曲げ等 |
| [非線形動的解析_参照実装照合.md](非線形動的解析_参照実装照合.md) | 07 非線形動的 | 🔶 | UI/IO 経路整備 |
| [数量積算_参照実装照合.md](数量積算_参照実装照合.md) | 数量積算 | 🔶 | モデル制約による C 節 |

### 原典・論文・資料照合

| レポート | 対象 | 判定 |
|----------|------|------|
| [材料強度_基準強度照合.md](材料強度_基準強度照合.md) | 告示・材料強度資料 | 🔶 |
| [未入力材料強度の危険側フォールバック_2026-07.md](未入力材料強度の危険側フォールバック_2026-07.md) | フォールバック是正 | 🔶 |
| [ファイバー材料モデル_論文照合_2026-07.md](ファイバー材料モデル_論文照合_2026-07.md) | MP/Mander/Yassin | ✅ |
| [ファイバー形状_長期初期載荷_2026-07.md](ファイバー形状_長期初期載荷_2026-07.md) | ファイバー形状・長期荷重 | ✅ |
| [仕口パネル_定式化と検証_2026-07.md](仕口パネル_定式化と検証_2026-07.md) | パネルゾーン力学 | 🔶 |

### 敵対的レビュー・定式化レビュー

| レポート | 対象 | 判定 |
|----------|------|------|
| [adversarial_review_2026-07.md](adversarial_review_2026-07.md) | 横断（PO・剛域・MITC4 等） | 🔶 |
| [解析コア_敵対的レビュー_2026-07.md](解析コア_敵対的レビュー_2026-07.md) | 静解析・固有値・増分解析 | 🔶 |
| [耐震壁_敵対的レビュー_2026-07.md](耐震壁_敵対的レビュー_2026-07.md) | 壁エレメント | 🔶 |
| [保有水平耐力_プッシュオーバー_敵対的レビュー_2026-07.md](保有水平耐力_プッシュオーバー_敵対的レビュー_2026-07.md) | ルート3 PO | 🔶 |
| [材端集中ばね梁_定式化レビュー_2026-07.md](材端集中ばね梁_定式化レビュー_2026-07.md) | ConcentratedSpringBeam | 🔶 |
| [増分解析_ヒンジ形成と剛性低下_検証_2026-07.md](増分解析_ヒンジ形成と剛性低下_検証_2026-07.md) | ヒンジ・剛性低下 | ✅ |
| [増分解析_長期載荷の接線剛性破綻_2026-07.md](増分解析_長期載荷の接線剛性破綻_2026-07.md) | 長期荷重接線剛性 | ✅ |
| [変位制御_荷重パターン保持_2026-07.md](変位制御_荷重パターン保持_2026-07.md) | 変位制御 PO | ✅ |
| [剛域_保有水平耐力_系レベル検証.md](剛域_保有水平耐力_系レベル検証.md) | 剛域の系レベル影響 | 🔶 |
| [小梁設計のスラブ帰属_レベル一致_2026-08.md](小梁設計のスラブ帰属_レベル一致_2026-08.md) | 小梁が負担するスラブの決め方 | 🔶 |
| [床辺荷重の梁への幾何割付_2026-08.md](床辺荷重の梁への幾何割付_2026-08.md) | 床の辺荷重を覆う梁へ割り付ける経路 | ✅ |
| [床領域のパネル統合_2026-08.md](床領域のパネル統合_2026-08.md) | ST-Bridge 小片スラブを大梁の床領域へ畳む | 🔶 |
| [床領域の再設計_荷重分配とSlabFloorRegion分離_2026-08.md](床領域の再設計_荷重分配とSlabFloorRegion分離_2026-08.md) | 荷重分配を `FloorRegion`/`Slab` 単位へ作り替え、型を再分離 | 🔶 |
| [小梁検定の負担幅を床板境界から求める_2026-08.md](小梁検定の負担幅を床板境界から求める_2026-08.md) | 小梁検定の負担幅を床板境界の幾何から求める（Step 5 一部。**二次部材経路は §5.39・§5.40 で supersede**） | 🔶 |
| [小梁設計を分配結果から出す_2026-08.md](小梁設計を分配結果から出す_2026-08.md) | 二次部材小梁の断面検定を分配 `Span` から出す（Step 5。§5.40〜§5.43） | ☑ |
| [二次部材の反力の逐次伝達_2026-08.md](二次部材の反力の逐次伝達_2026-08.md) | 小梁を二次部材へ一本化し、二次部材に支持された二次部材の荷重が解析から消える危険側の穴を塞ぐ（§3.4・§5.44）。実データで床固定荷重の 7.2% が失われていた | ☑ |
| [壁版の取り込み・要素生成・参照張り替え_2026-08.md](壁版の取り込み・要素生成・参照張り替え_2026-08.md) | 壁の解析要素を準備計算からの生成物へ転換（Step 7+8 本体） | ☑ |
| [剛域算定の壁展開順序不整合_2026-08.md](剛域算定の壁展開順序不整合_2026-08.md) | 壁展開モデルを見ていなかった4箇所（剛域自動算定・耐震壁のせん断断面検定・数量拾い・保有水平耐力の部材ランク自動判定）の是正 | ☑ |

### フェーズ監査

| レポート | フェーズ | 判定 |
|----------|----------|------|
| [p3_review.md](p3_review.md) | P3 最小 UI | 🔶 |
| [p4_review.md](p4_review.md) | P4 材料断面 | ✅ |
| [p7_review.md](p7_review.md) | P7 二次設計 | 🔶 |
| [p8_review.md](p8_review.md) | P8 操作連携 | 🔶 |
| [pending_items.md](pending_items.md) | P9 仕上げ | 🔶 |

## 索引（要素/設計式 → テスト）

| # | 対象 | クレート | ソースファイル | テスト関数 | フェーズ | 状態 |
|---|------|----------|---------------|-----------|---------|------|
| 1 | ティモシェンコ梁 | squid-n-element | beam.rs | `test_phi_zero_converges_to_bernoulli`, `test_beam_axial_stiffness`, `test_beam_torsion_stiffness` | P1 | ✅ |
| 2 | 剛域あり梁 | squid-n-element | beam.rs | `test_auto_rigid_zone_standard_formula` | P1 | 🔶 |
| 3 | 端部ばね（ピン・半剛） | squid-n-element | beam.rs | `test_pinned_end_releases_moment` | P1 | 🔶 |
| 4 | MITC4 シェル（膜） | squid-n-element | shell.rs | `test_patch_membrane_distorted`（歪みメッシュ・機械精度） | P1.5 | ✅ |
| 5 | MITC4 シェル（曲げ） | squid-n-element | shell.rs | `test_patch_bending_distorted`（歪みメッシュ定曲率・機械精度） | P1.5 | ✅ |
| 6 | MITC4 シェル（せん断/収束） | squid-n-solver | linear.rs | `test_ss_plate_convergence`, `test_clamped_plate_convergence`（板たわみ ±2% 収束＝ロッキングなし） | P1.5 | ✅ |
| 7 | パネルゾーンのフェイスモーメント | squid-n-element | panel.rs | `test_face_moments_reference_case1`（pQc=851.135kN 等）, `test_face_moments_reference_case2_t_joint`（ト型） | P1 | ✅ |
| 7a | 仕口パネル（せん断変形角の追加自由度） | squid-n-solver | tests/panel_zone.rs | `test_panel_shear_angle_matches_closed_form`（M=K·γ）, `test_panel_dof_equilibrium_residual_is_zero`（資料 2.10.3-3）。定式化と残課題は [仕口パネル_定式化と検証_2026-07.md](仕口パネル_定式化と検証_2026-07.md) | P1 | ✅ |
| 8 | 線形静的解析 | squid-n-solver | linear.rs | `test_*`（座標変換回帰 `test_beam_to_global_transverse_uses_correct_inertia` 含む） | P2 | ✅ |
| 9 | 固有値解析 | squid-n-solver | eigen.rs | `test_1dof_period` | P2 | ✅ |
| 10 | Ai分布 | squid-n-load | ai.rs | `test_*` | P2 | ✅ |
| 11 | 床荷重分割 | squid-n-load | floor.rs | `test_*` | P2 | ✅ |
| 12 | 荷重組合せ | squid-n-load | combo.rs | `test_combinations` | P2 | ✅ |
| 13 | 許容応力度設計 | squid-n-design-jp | allowable_stress.rs | `test_steel_check_bending_spec_p3_6_4` 他 | P3 | ✅ |
| 14 | 保有耐力 | squid-n-design-jp | holding_capacity.rs | `test_*` | P7 | 🔶 |
| 15 | プッシュオーバー | squid-n-solver | pushover.rs | — | P5 | 🔶 |
| 16 | 壁（TVLEM） | squid-n-element | — | — | P5.5 | ❌ |
| 17 | 時刻歴 | squid-n-solver | timehistory.rs | — | P6 | ❌ |
| 18 | 一軸履歴則（Concrete/Bilinear/MP） | squid-n-material | uniaxial.rs | `test_concrete_*`/`test_bilinear_*`/`test_menegotto_pinto_*` | P4 | ✅ |
| 19 | 部材履歴則（武田・原点指向・スリップ） | squid-n-material | hysteresis.rs | `tests/hysteresis_snapshots.rs`/`tests/uniaxial_snapshots.rs` | P4 | ✅ |
| 20 | ファイバ断面（M–φ 積分） | squid-n-section | fiber.rs | `test_section_*` | P4 | ✅ |
| 21 | スケルトン自動算定（M–φ→M–θ） | squid-n-skeleton | lib.rs | `test_rc_skeleton_*` | P4 | ✅ |
| 22 | MCP サーバ（rmcp） | squid-n-mcp | server.rs, job/*.rs | `model_query`/`model_edit`（壁版・床板・床領域）/`quantity_takeoff`/`analysis_run`/`result_get`/`analysis_status`（`--features mcp` で CI 検証。`tests.rs` + `server.rs` 統合テスト）。**未公開**: `model.load`/`model.save`/`report.export` | P8 | 🔶 |
| 23 | ST-Bridge 入出力 | squid-n-io | stbridge.rs | `test_roundtrip_*` | P8 | 🔶 |
| 24 | 編集トランザクション（EditCommand/Undo） | squid-n-edit | lib.rs | `test_*`（MCP `model_edit` は壁版・床板・床領域を配線。他コマンドは未 → [未検証一覧 §6](未検証一覧.md)） | P3/P8 | ✅ |
| 25 | 終局検定（塑性 Qsu・付着 Qbu・軸 Nuc/Nut・2軸せん断・接合部 Vju/Qdu・CFT 軸終局+N-M） | squid-n-design-jp | ultimate/{rc_shear,rc_axial,joint,cft,cft_nm,mod}.rs | `test_rc_shear_qsu_plastic_*`/`test_rc_joint_ultimate_*`/`test_cft_*`/`test_cft_short_column_mu_*`/`test_biaxial_*`/`test_collect_*_ultimate_checks_*` | P7 | 🔶 |
| 26 | 数量積算（部位別のコンクリート・型枠・鉄筋・鉄骨・継手個所） | squid-n-design-jp | quantity/{mod,member,rebar}.rs | `quantity::member::tests::*`（手計算照合）/`quantity::tests::*`（走査・分類）/`summary::tests::test_quantity_csv_from_sample_model`（CSV 一気通貫）/`test_quantity_takeoff_json_column`（MCP） | 横断 | 🔶 |
| 27 | 材料グレード対応表（F 値・鉄筋・Fc・プリセット） | squid-n-core | material_grade.rs | `material_grade::tests::*`（告示値一致） | 横断 | ✅ |
| 28 | 二次部材小梁の分配 Span 検定 | squid-n-load / squid-n-app | floor/joist_design.rs, check.rs | `distribution_loads_on_shared_joist_match_average_width` / `split_slab_edges_compose_onto_full_joist` / `span_attaches_to_nearest_joist_only` / `perimeter_parallel_joist_does_not_steal_beam_span` / `joist_distribution_cover_rejects_half_span` / `shared_joist_expects_both_slabs` / `missing_expected_slab_is_not_ready` / `zero_expected_axis_does_not_receive_spans` / `joist_design_checks_cover_imported_secondary_members` | 横断 | ☑ |

凡例: ✅ 実装済み・🔶 一部実装（要拡張）・❌ 未実装

各 # の修正履歴・監査結果の詳細は [§レポート目録](#レポート目録) の該当レポートを参照。
未完了の要約は [未検証一覧.md](未検証一覧.md) を参照。

## 1 次参照: 手計算／理論解

本ソフトの V&V は、原則として **手計算／理論解** を一次基準とする。各フェーズの DoD は手計算・理論解・告示式の自己整合・添付資料数値例で合否判定できるよう構成されている。

**実測／商用ソフト照合は補助**（入手できれば追加）であり、なくてもビルド・単体テスト・一次 V&V は通る。

**唯一の例外＝壁（壁谷澤）の妥当性確認** は、モデルの性質上、実験照合が本質的に必須（Category B）。
技術リードが実験データを用意する（R4/R23）。

## パネルゾーン参照解

出典: 添付資料『パネルゾーンの力学』(小野瀬, 2009) 図18–20

| ケース | pQc | pQb | τ |
|--------|-----|-----|---|
| ケース1 | 851.135 kN | 1702.273 kN | 42.557/tp |
| ケース2 | (資料参照) | (資料参照) | (資料参照) |
| ケース3 | (資料参照) | (資料参照) | (資料参照) |

## 決定性テスト

全解析種別で「CPU・単一スレッドでビット一致」を検証（R28）。並列／Parquet書込／GPU は値一致で検証（ビット一致保証外）。

| 解析種別 | 状態 | ファイル |
|----------|------|----------|
| 線形静的 | ✅ | linear.rs |
| 疎行列組立 | ✅ | sparse.rs |
| Cholesky 分解 | ✅ | cholesky.rs |
| 固有値 | ✅ | eigen.rs |
| 時刻歴 | 🔶 | timehistory.rs（P6 実装後に本格化） |
| プッシュオーバー | 🔶 | pushover.rs（P5 実装後に本格化） |
| 並列バッチ（値一致） | ✅ | squid-n-solver/tests/parallel_batch.rs（並列時のケース並列バッチが個別解と一致） |

## 性能ベンチマーク

`criterion` による性能ベンチマーク（線形静的・固有値・プッシュオーバー1ステップ・時刻歴1ステップ）の計測は CI 導入時（P9）に整備予定。

並列計算（ケース並列バッチ・faer 内部並列）の速度比は
`cargo run -p squid-n-solver --example parallel_bench --release` で計測できる
（ドキュメントサイト 5.10 並列計算に参考値を記載）。
