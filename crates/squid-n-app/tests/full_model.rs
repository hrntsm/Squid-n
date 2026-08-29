//! 実建物モデル（ST-Bridge）を読み込み、全解析を通す統合テスト。
//!
//! # 目的
//!
//! 既存のテストはいずれも手組みの小規模モデル（門型ラーメン・1 層立体フレーム）を
//! 対象としており、実建物特有の構成（剛床・二次部材・混構造・多数のスラブ）が
//! 揃って初めて現れる不具合を検出できない。本テストは実際の設計モデルを 1 つ
//! フィクスチャとして固定し、GUI のボタンが呼ぶのと同じ入口（`App` の
//! `run_*` / `compute_*`）を通して全解析を実行する。
//!
//! # モデル（`tests/fixtures/model.stb`）
//!
//! 4 層＋PH の S 造（一部 RC）。節点 166・解析要素 115（柱 40・大梁 75）・
//! 二次部材 56（小梁）・床領域 26（大梁1床領域単位）・階 5（Z=200/4700/8700/12700/16500）。
//! 荷重は ST-Bridge に含まれないため、取り込み時に標準荷重ケース
//! （DL・LL(架構用)・LL(地震用)・EX・EY）が自動生成される。支点情報も
//! 含まれないため、最下レベルの柱脚 12 箇所がピン支点として自動設定される。
//!
//! # 検証の三層構造
//!
//! 1. **煙テスト** — エラーなく完走し、結果が空でない
//! 2. **構造的不変量** — 「エラーは出ないが結果が静かに劣化する」退行を捕まえる
//!    （全部材に断面力がある／柱脚が圧縮／固有周期が正で降順／層せん断が上階ほど
//!    小さい 等）。過去に発生した「剛床上の梁の応力欠落」「サイレントにゼロ変位」
//!    の類はここで落ちる
//! 3. **代表スカラのスナップショット**（[`snapshot_key_scalars`]） — 値そのものの
//!    変化を可視化する。CI（Linux）と手元（Windows）の浮動小数差で偽陽性に
//!    ならないよう、有効数字 4 桁の指数表記に丸めてから記録する
//!
//! # 解析の再現性
//!
//! `analysis_cfg.threads = 1`（単一スレッド）を全テストで指定する。既定の 0
//! （全コア）では並列リダクションの加算順が実行ごとに変わりうるため、
//! スナップショットの比較が安定しない。
//!
//! # 新しい解析を追加したとき
//!
//! `App` に解析エントリを追加したら、本ファイルにもテストを追加すること
//! （CONTRIBUTING.md「実モデルの統合テスト」参照）。追加を怠ると、その機能だけが
//! 回帰検出の対象外になる。

use squid_n_app::app::{App, StaticCaseKey, ThDampingModel, ThDir, DL_CASE_NAME};
use squid_n_core::dof::Dof6Mask;
use squid_n_core::model::ElementKind;
use squid_n_solver::analysis::SeismicDir;

// ===================== フィクスチャと共通ヘルパー =====================

/// 固定フィクスチャ（実建物の ST-Bridge）のパス。
fn fixture_path() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("model.stb")
}

/// テストが書き込む一時ディレクトリ（プロセス ID 入り）。
/// `std::env::temp_dir()` 直下へ固定名で書き込むと、同一マシンで並行する
/// 別プロセスのテスト実行と衝突するため、プロセスごとに一意なサブディレクトリを
/// 介する（同一プロセス内はテストごとの固有ファイル名で分離する）。
fn test_tmp() -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("squid-n-full-model-{}", std::process::id()));
    let _ = std::fs::create_dir_all(&d);
    d
}

/// 直前の操作でエラーが出ていないことを確認する。
///
/// `App::last_error` はステータスバー共用の単一スロットで、前の操作の内容が
/// 残るため、各ステップの前に [`clear_error`] でクリアしてから使う。
fn assert_no_error(app: &App, what: &str) {
    assert!(
        app.last_error.is_none(),
        "{what} でエラー: {}",
        app.last_error.as_deref().unwrap_or("")
    );
}

/// `last_error` をクリアする（次のステップの判定に前の内容を持ち越さない）。
fn clear_error(app: &mut App) {
    app.last_error = None;
}

/// フィクスチャを取り込んだ `App`。
///
/// ST-Bridge 取り込みは、欠落属性の要約や支点の自動設定といった**注意**も
/// `report_error` 経由で `last_error` に載せる（`app/actions.rs` の
/// `import_stbridge_from`。ステータスバーで見落とされないようにするための仕様）。
/// そのため「注意（先頭が ⚠️）以外のエラーが出ていないこと」を確認したうえで
/// クリアする。
fn imported() -> App {
    let mut app = App::default();
    // 解析結果の完全再現性を確保する（スナップショット比較の前提）。
    app.analysis_cfg.threads = 1;
    app.import_stbridge_from(fixture_path());
    if let Some(e) = &app.last_error {
        assert!(
            e.starts_with('⚠'),
            "ST-Bridge 取り込みが失敗した（注意ではなくエラー）: {e}"
        );
    }
    clear_error(&mut app);
    app
}

/// 取り込み＋準備計算まで済ませた `App`（階・剛域・仕口パネル・地震力が確定した状態）。
fn prepared() -> App {
    let mut app = imported();
    app.run_preparation();
    assert_no_error(&app, "準備計算");
    app
}

/// 準備計算＋静的解析（全荷重ケース・全組合せ）＋固有値解析まで済ませた `App`。
fn analyzed() -> App {
    let mut app = prepared();
    app.run_static_all();
    assert_no_error(&app, "静的解析（一括）");
    app.run_eigen(app.analysis_cfg.n_modes);
    assert_no_error(&app, "固有値解析");
    app
}

/// 解析対象の梁要素（断面力が必ず得られる部材）の本数。
/// 準備計算が自動生成する仕口パネル要素（`PanelZone`）は断面力の対象外のため除く。
fn frame_elem_count(app: &App) -> usize {
    app.model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Beam && e.nodes.len() == 2)
        .count()
}

/// 荷重ケースの解析結果が格納されるキー。
///
/// 標準の水平力ケース（EX/EY）は Ai 分布から水平力を組み立て直して解かれ、
/// 方向別の [`StaticCaseKey::Seismic`] に格納される（`App::standard_lateral_case`。
/// `pub(crate)` のためテストからは呼べず、同じ判定をここに置く）。それ以外は
/// [`StaticCaseKey::User`]。
fn static_case_key(app: &App, lc: squid_n_core::ids::LoadCaseId) -> StaticCaseKey {
    use squid_n_core::model::{LoadCaseKind, EX_CASE_NAME, EY_CASE_NAME};
    let case = app
        .model
        .load_cases
        .iter()
        .find(|c| c.id == lc)
        .unwrap_or_else(|| panic!("荷重ケース {lc:?} が見つからない"));
    match (case.name.as_str(), case.kind) {
        (EX_CASE_NAME, LoadCaseKind::Seismic) => StaticCaseKey::Seismic(SeismicDir::X),
        (EY_CASE_NAME, LoadCaseKind::Seismic) => StaticCaseKey::Seismic(SeismicDir::Y),
        _ => StaticCaseKey::User(lc),
    }
}

/// 指定した静的結果を取り出す。
fn static_of(app: &App, key: StaticCaseKey) -> &squid_n_solver::linear::StaticOnce {
    app.results
        .as_ref()
        .expect("解析結果が格納されているはず")
        .statics
        .iter()
        .find(|(k, _)| *k == key)
        .map(|(_, v)| v)
        .unwrap_or_else(|| panic!("{key:?} の静的結果が見つからない"))
}

/// DL（固定荷重）の荷重ケース ID。
fn dl_case_id(app: &App) -> squid_n_core::ids::LoadCaseId {
    app.model
        .load_cases
        .iter()
        .find(|lc| lc.name == DL_CASE_NAME)
        .expect("標準荷重ケース DL が自動生成されるはず")
        .id
}

/// 名前で荷重ケースを取り出す（内容の比較用）。
fn auto_case(app: &App, name: &str) -> squid_n_core::model::LoadCase {
    app.model
        .load_cases
        .iter()
        .find(|lc| lc.name == name)
        .unwrap_or_else(|| panic!("荷重ケース「{name}」が見つからない"))
        .clone()
}

/// 柱脚（支点に取り付く鉛直部材の支点側材端）の軸力 [N] を集める。
///
/// 鉛直荷重の伝達経路が壊れた（剛床・二次部材の CMQ 変換・スラブ分配のいずれかが
/// 荷重を落とした）場合、ここの合計が変化する回帰検出用の指標であり、**総反力
/// （＝総荷重）の代わりにはならない**（このフィクスチャでは合計 [`base_column_axials`]
/// が総荷重の半分未満）。垂直部材（柱）自身の材端軸力だけを見ており、同じ支点節点に
/// 取り付く基礎梁（水平部材）が負担する分は含まない。荷重の分配経路が変わると
/// （床領域単位で1枚に畳んで配るか、床板ごとに個別へ配るか等）、柱の軸力と基礎梁の
/// せん断のどちらへどれだけ載るかの配分が変わるため、この合計も変わりうる
/// （床領域の荷重分配作り替え〔Step 4〕の際に実際に約 7.7% 動いた。総荷重は
/// `dev_docs/v_and_v/床領域の再設計_荷重分配とSlabFloorRegion分離_2026-08.md`
/// の追記のとおりビット単位で一致しており、荷重が失われたのではない）。
fn base_column_axials(app: &App, res: &squid_n_solver::linear::StaticOnce) -> Vec<f64> {
    let mut out = Vec::new();
    for (eid, mf) in &res.member_forces {
        let Some(e) = app.model.elements.get(eid.index()) else {
            continue;
        };
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(na), Some(nb)) = (
            app.model.nodes.get(e.nodes[0].index()),
            app.model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        // 鉛直部材（柱）のみ対象。
        if (nb.coord[2] - na.coord[2]).abs() <= 1e-6 {
            continue;
        }
        let i_is_base = na.coord[2] < nb.coord[2];
        let base_node = if i_is_base { na } else { nb };
        if base_node.restraint == Dof6Mask::FREE {
            continue;
        }
        let (Some((_, fi)), Some((_, fj))) = (mf.at.first(), mf.at.last()) else {
            continue;
        };
        out.push(if i_is_base { fi[0] } else { fj[0] });
    }
    out
}

/// 有効数字 4 桁の指数表記へ丸める（スナップショット用）。
///
/// 手元（Windows）で生成したスナップショットを CI（Linux）で照合するため、
/// 浮動小数の環境差（ベクトル化の差・`faer` の並べ替え・libm の実装差。
/// 通常 1e-12 オーダー）が結果に出ないところまで桁を落とす。4 桁あれば
/// 設計上意味のある変化（例: T1 が 0.4676→0.4931）は確実に捕まる。
fn sig4(v: f64) -> String {
    if !v.is_finite() {
        return format!("{v}");
    }
    // -0.0 が "-0.000e0" と "0.000e0" で揺れないよう正規化する。
    let v = if v == 0.0 { 0.0 } else { v };
    format!("{v:.3e}")
}

// ===================== 1. 取り込み =====================

/// ST-Bridge の取り込みが、想定どおりのモデル構成（部材・二次部材・床・階・
/// 荷重ケース・支点）を組み立てる。
///
/// 取り込み側の分類が変わると（例: 小梁を解析要素として取り込むようになる）、
/// 以降のすべての解析の前提が変わるため、まず構成を固定する。
#[test]
fn import_builds_expected_model() {
    let app = imported();
    let m = &app.model;

    assert_eq!(m.nodes.len(), 166, "節点数");
    assert_eq!(m.elements.len(), 115, "解析要素数（柱 40・大梁 75）");
    assert_eq!(m.joists().count(), 56, "二次部材（小梁）");
    assert_eq!(m.floor_regions.len(), 26, "床領域（大梁1床領域単位）");
    assert_eq!(m.stories.len(), 5, "階（1FL/2FL/3FL/RFL/PHRFL）");

    use squid_n_core::region_gen::generate_region_boundaries;
    assert!(
        m.slabs
            .iter()
            .all(|s| !s.is_attached() && s.section().is_some()),
        "すべて Enclosed かつ版あり"
    );
    let boundaries = generate_region_boundaries(m);
    for r in &m.floor_regions {
        for sm in &r.secondary_joists {
            let coords = r.boundary_coords(m).expect("領域境界");
            let n = coords.len() as f64;
            let centroid = [
                coords.iter().map(|p| p[0]).sum::<f64>() / n,
                coords.iter().map(|p| p[1]).sum::<f64>() / n,
            ];
            let z = coords[0][2];
            let boundary = boundaries
                .iter()
                .find(|b| b.is_same_level(z) && b.contains(m, centroid))
                .expect("領域が大梁の境界に載る");
            let a = m.nodes[sm.nodes[0].index()].coord;
            let b = m.nodes[sm.nodes[1].index()].coord;
            let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
            assert!(
                boundary.contains(m, mid),
                "小梁 {:?} の中点が所属領域に入らない",
                sm.nodes
            );
        }
    }

    // 荷重は ST-Bridge に含まれないため標準荷重ケースが自動生成される。
    let names: Vec<&str> = m.load_cases.iter().map(|lc| lc.name.as_str()).collect();
    assert_eq!(
        names,
        vec!["DL", "LL(架構用)", "LL(地震用)", "EX", "EY"],
        "標準荷重ケースが自動生成される"
    );
    assert!(!m.combinations.is_empty(), "標準荷重組合せも用意される");

    // 支点情報も含まれないため、最下レベルの柱脚がピン支点として自動設定される。
    let supports = m
        .nodes
        .iter()
        .filter(|n| n.restraint != Dof6Mask::FREE)
        .count();
    assert_eq!(supports, 12, "自動設定される支点の数");

    // 断面・材料が全部材へ割り当たっている（未割当があると解析前チェックで落ちる）。
    for e in &m.elements {
        assert!(
            e.section.is_some(),
            "部材 {:?} に断面が割り当たっていない",
            e.id
        );
    }
}

// ===================== 2. 準備計算 =====================

/// 準備計算が階・地震重量・Ai 分布・剛域・仕口パネルを算定する。
///
/// 地震力については「ΣW から Q1 が令 88 条どおり求まる」ことを式で検算する
/// （`Q1 = Z·Rt·C0·ΣW`。Ai は最下層で 1.0）。準備計算の内部でどう組み立てても、
/// この恒等式は成り立たなければならない。
#[test]
fn preparation_computes_stories_and_seismic_forces() {
    let app = prepared();
    let prep = app.preparation.as_ref().expect("準備計算の結果が入るはず");

    assert!(prep.is_ready(), "整合性チェックにエラーがない");
    assert_eq!(app.model.stories.len(), 5, "階（1FL/2FL/3FL/RFL/PHRFL）");
    // 準備計算の階の表は「層」（階と階の間）を並べるため、階数より 1 少ない。
    assert_eq!(prep.stories.len(), 4, "階の分布の行数");
    assert_eq!(prep.summary.n_supports, 12, "支点数");
    assert_eq!(prep.summary.n_diaphragms, 5, "剛床数");
    assert!(
        prep.summary.total_seismic_weight > 0.0,
        "地震用重量の総和が正"
    );

    // 階は下から上へ、床レベルが単調増加する。
    let elevations: Vec<f64> = prep.stories.iter().map(|s| s.elevation).collect();
    assert!(
        elevations.windows(2).all(|w| w[0] < w[1]),
        "階は下階→上階の順（床レベル単調増加）: {elevations:?}"
    );

    for s in &prep.stories {
        assert!(s.height > 0.0, "階 {} の階高が正", s.name);
        assert!(s.weight > 0.0, "階 {} の地震用重量が正", s.name);
    }

    let seismic = prep.seismic.as_ref().expect("地震力が算定されるはず");
    assert_eq!(seismic.rows.len(), prep.stories.len(), "Ai 分布の層数");
    assert!(
        !seismic.clamped_negative_pi,
        "Pi に負値クランプが発生しない"
    );
    assert!(seismic.t > 0.0, "設計用一次固有周期 T が正");
    assert!(
        seismic.rt > 0.0 && seismic.rt <= 1.0,
        "振動特性係数 Rt は 0<Rt≤1"
    );

    // 令88条: Q1 = Z·Rt·C0·ΣW（最下層は Ai=1.0）。
    let expected_q1 = seismic.z * seismic.rt * seismic.c0 * prep.summary.total_seismic_weight;
    assert!(
        (seismic.base_shear - expected_q1).abs() <= expected_q1.abs() * 1e-9,
        "基部せん断力 Q1={} が Z·Rt·C0·ΣW={} と一致しない",
        seismic.base_shear,
        expected_q1
    );

    // 層せん断力 Qi は上階ほど小さい（Ai 分布の性質）。
    let qi: Vec<f64> = seismic.rows.iter().map(|r| r.qi).collect();
    assert!(
        qi.windows(2).all(|w| w[0] >= w[1]),
        "層せん断力 Qi が上階ほど小さくない: {qi:?}"
    );

    // 剛域・仕口パネル・断面性能が算定されている。
    assert!(
        prep.rigid_zone_candidates > 0,
        "剛域の算定対象となる梁がある"
    );
    assert!(!prep.sections.is_empty(), "断面性能が算定される");
    assert_eq!(
        prep.load_cases.len(),
        app.model.load_cases.len(),
        "荷重ケースの集計行数"
    );
}

/// 準備計算は 1 回目から冪等である（何度実行しても階・地震用重量・固定荷重が同じ）。
///
/// RC/SRC 梁の自重は柱面間の内法長（節点間長 − 両端の柱フェース距離
/// `RigidZone::face_i/face_j`）で算定するため、自重の同期は
/// フェース距離の算定より後に行わなければならない。かつては
/// `generate_stories_action` が同期を先に行っており、1 回目の準備計算だけ
/// フェース距離が未算定（0）のまま節点間距離で算定した過大な自重が DL に入り、
/// 2 回目の実行で初めて正しい値へ変わっていた。準備計算は各解析の実行前にも
/// 自動で走るため、「準備計算を実行した回数」で柱脚軸力・断面検定が変わる
/// 状態だった。
#[test]
fn preparation_is_idempotent() {
    let mut app = prepared();
    assert_eq!(app.model.floor_regions.len(), 26, "準備計算後も床領域は 26");
    let stories = app.model.stories.len();
    let weights: Vec<Option<f64>> = app.model.stories.iter().map(|s| s.seismic_weight).collect();
    let dl = auto_case(&app, DL_CASE_NAME);

    for n in 2..=3 {
        app.run_preparation();
        assert_no_error(&app, &format!("準備計算（{n} 回目）"));
        assert_eq!(app.model.stories.len(), stories, "{n} 回目で階数が変わった");
        assert_eq!(
            app.model
                .stories
                .iter()
                .map(|s| s.seismic_weight)
                .collect::<Vec<_>>(),
            weights,
            "{n} 回目で地震用重量が変わった"
        );
        assert_eq!(
            auto_case(&app, DL_CASE_NAME),
            dl,
            "{n} 回目で固定荷重 DL の内容が変わった"
        );
    }
}

/// 固定荷重の算定は、剛域の自動算定を先に走らせたかどうかに依存しない。
///
/// RC/SRC 梁の自重は柱面間の内法長で算定する。柱フェース距離はかつて剛域の
/// 自動算定（`apply_auto_rigid_zones`）だけが埋めるキャッシュで、算定前に読むと
/// 0 になり、節点間距離で自重を算定してしまっていた（申し送り
/// 「実モデル統合テスト」4.1）。現在は幾何から直接求めるため順序に依存しない。
///
/// 「取り込んだ直後に自重を同期した DL」と「準備計算まで済ませた DL」が
/// 一致することで、この順序非依存を固定する。
#[test]
fn dead_load_does_not_depend_on_rigid_zone_timing() {
    let mut early = imported();
    early.sync_gravity_load_cases_action();
    let dl_early = auto_case(&early, DL_CASE_NAME);

    let prepared = prepared();
    let dl_prepared = auto_case(&prepared, DL_CASE_NAME);

    assert_eq!(
        dl_early.member.len(),
        dl_prepared.member.len(),
        "部材荷重の件数が一致しない"
    );
    for (a, b) in dl_early.member.iter().zip(dl_prepared.member.iter()) {
        assert_eq!(
            a.kind, b.kind,
            "部材 {:?} の固定荷重が剛域算定の前後で変わる",
            a.elem
        );
    }
}

// ===================== 3. 診断 =====================

/// 実建物モデルの整合性チェックがエラー・警告ともに 0 件である。
///
/// 診断が誤検知を出すようになると、利用者は解析前に赤い表示を見ることになる。
/// 逆に検出漏れが起きると解析が謎のエラーで落ちる。実モデルで 0 件を固定しておく。
#[test]
fn diagnostics_are_clean() {
    let mut app = prepared();
    app.run_diagnostics();
    assert_no_error(&app, "診断");
    assert_eq!(
        app.diagnostics_counts(),
        (0, 0),
        "実建物モデルの診断は (エラー, 警告) = (0, 0) のはず"
    );
}

// ===================== 4. 静的解析（全荷重ケース・全組合せ） =====================

/// 全荷重ケース・全荷重組合せが解け、すべての解析対象部材に断面力が入る。
///
/// 「解析は成功したのに一部の部材だけ断面力が空」という静かな劣化
/// （剛床上の梁・二次部材が絡む経路で過去に発生）を検出する。
#[test]
fn static_all_solves_every_case_and_combination() {
    let app = analyzed();
    let results = app.results.as_ref().expect("解析結果");
    let n_elems = frame_elem_count(&app);

    assert_eq!(results.statics.len(), 5, "荷重ケース単体の結果数");
    assert_eq!(
        results.combos.len(),
        app.model.combinations.len(),
        "荷重組合せの結果数"
    );

    for (key, once) in &results.statics {
        assert_eq!(
            once.disp.len(),
            app.model.nodes.len(),
            "{key:?}: 変位が全節点分ない"
        );
        assert!(
            once.disp.iter().flatten().all(|v| v.is_finite()),
            "{key:?}: 変位に非有限値がある"
        );
        assert_eq!(
            once.member_forces.len(),
            n_elems,
            "{key:?}: 断面力が全部材分ない（応力の欠落）"
        );
        for (eid, mf) in &once.member_forces {
            assert!(!mf.at.is_empty(), "{key:?}: 部材 {eid:?} の評価断面が空");
            assert!(
                mf.at.iter().all(|(_, f)| f.iter().all(|v| v.is_finite())),
                "{key:?}: 部材 {eid:?} の断面力に非有限値がある"
            );
        }
    }

    for (name, once) in &results.combos {
        assert_eq!(
            once.member_forces.len(),
            n_elems,
            "組合せ「{name}」: 断面力が全部材分ない"
        );
    }
}

/// 固定荷重（DL）の応答が鉛直荷重の伝達経路として妥当である。
///
/// - すべての節点が下向き（または不動）に変位する
/// - すべての柱脚が圧縮（軸力が負）
///
/// スラブ分配・小梁の CMQ 変換・剛床のいずれかが荷重を落とすと、柱脚軸力の合計が
/// 変わる。合計値そのものは [`snapshot_key_scalars`] で固定する。
#[test]
fn dead_load_transfers_to_column_bases() {
    let app = analyzed();
    let dl = static_of(&app, StaticCaseKey::User(dl_case_id(&app)));

    let max_uz = dl
        .disp
        .iter()
        .map(|d| d[2])
        .fold(f64::NEG_INFINITY, f64::max);
    let min_uz = dl.disp.iter().map(|d| d[2]).fold(f64::INFINITY, f64::min);
    assert!(
        max_uz <= 1e-9,
        "固定荷重で上向きに変位する節点がある（最大 uz={max_uz}）"
    );
    assert!(min_uz < 0.0, "固定荷重で誰も沈まない（荷重が載っていない）");

    let axials = base_column_axials(&app, dl);
    assert_eq!(axials.len(), 12, "柱脚の本数（支点に取り付く柱）");
    for (i, n) in axials.iter().enumerate() {
        assert!(*n < 0.0, "柱脚 {i} が固定荷重で圧縮になっていない（N={n}）");
    }
}

/// 荷重組合せの結果が、参照する荷重ケース単体の線形和と一致する（重ね合わせの原理）。
///
/// 組合せの求解は荷重ケース単体の線形和として組み立てられるため
/// （`Analysis::linear_combination`）、実モデルでもこの関係が保たれる。
/// **全組合せ**を対象に検算する（1 件だけだと、組合せの構成が変わったときに
/// 検証していない組合せが増えても気づけない）。
#[test]
fn combination_is_linear_sum_of_load_cases() {
    let app = analyzed();
    let results = app.results.as_ref().expect("解析結果");
    assert!(!results.combos.is_empty(), "荷重組合せの結果が空");

    for (name, combo_res) in &results.combos {
        let combo = app
            .model
            .combinations
            .iter()
            .find(|c| c.name == *name)
            .unwrap_or_else(|| panic!("組合せ「{name}」の定義が見つからない"));

        let mut expected = vec![[0.0_f64; 6]; app.model.nodes.len()];
        for (case, factor) in &combo.terms {
            let once = static_of(&app, static_case_key(&app, *case));
            for (dst, src) in expected.iter_mut().zip(once.disp.iter()) {
                for k in 0..6 {
                    dst[k] += factor * src[k];
                }
            }
        }
        for (i, (got, want)) in combo_res.disp.iter().zip(expected.iter()).enumerate() {
            for k in 0..6 {
                let tol = want[k].abs() * 1e-9 + 1e-9;
                assert!(
                    (got[k] - want[k]).abs() <= tol,
                    "組合せ「{name}」節点 {i} 成分 {k}: {} != {}（線形和と不一致）",
                    got[k],
                    want[k]
                );
            }
        }
    }
}

// ===================== 5. 固有値解析 =====================

/// 固有値解析が指定モード数の正の固有周期を降順で返し、モード形状が全節点分ある。
#[test]
fn eigen_returns_descending_positive_periods() {
    let app = analyzed();
    let modal = app
        .results
        .as_ref()
        .expect("解析結果")
        .modal
        .as_ref()
        .expect("固有値解析の結果");

    assert_eq!(modal.period.len(), 3, "モード数（既定 3）");
    assert!(
        modal.period.iter().all(|t| t.is_finite() && *t > 0.0),
        "固有周期に非正・非有限がある: {:?}",
        modal.period
    );
    assert!(
        modal.period.windows(2).all(|w| w[0] > w[1]),
        "固有周期が降順でない（1 次が最長のはず）: {:?}",
        modal.period
    );
    assert_eq!(modal.node_shapes.len(), 3, "モード形状の本数");
    for (i, shape) in modal.node_shapes.iter().enumerate() {
        assert_eq!(
            shape.len(),
            app.model.nodes.len(),
            "{i} 次モードの形状が全節点分ない"
        );
        assert!(
            shape.iter().flatten().all(|v| v.is_finite()),
            "{i} 次モードの形状に非有限値がある"
        );
        assert!(
            shape.iter().flatten().any(|v| v.abs() > 1e-12),
            "{i} 次モードの形状が全ゼロ（サイレント失敗）"
        );
    }
}

// ===================== 6. 地震静的解析（Ai 分布） =====================

/// 地震静的解析が X・Y 両方向で解け、上階ほど水平変位が大きくなる。
///
/// 剛床が効いていない・水平力が一部の階に載っていないといった配線の破壊は、
/// 「階の水平変位が上階へ向かって単調増加しない」形で現れる。
#[test]
fn seismic_static_produces_monotonic_story_displacement() {
    for dir in [SeismicDir::X, SeismicDir::Y] {
        let mut app = prepared();
        app.run_seismic(dir);
        assert_no_error(&app, &format!("地震静的解析 {dir:?}"));

        let res = static_of(&app, StaticCaseKey::Seismic(dir));
        let comp = match dir {
            SeismicDir::X => 0,
            SeismicDir::Y => 1,
        };

        let mut prev = -1.0_f64;
        for s in &app.model.stories {
            let mx = s
                .node_ids
                .iter()
                .filter_map(|n| res.disp.get(n.index()))
                .map(|d| d[comp].abs())
                .fold(0.0_f64, f64::max);
            assert!(
                mx >= prev,
                "{dir:?}: 階 {} の水平変位 {mx} が下階の {prev} より小さい",
                s.name
            );
            prev = mx;
        }
        assert!(
            prev > 0.0,
            "{dir:?}: 最上階が全く動いていない（水平力が載っていない）"
        );
    }
}

// ===================== 7. 一次設計（断面検定） =====================

/// 断面検定が全部材・全接合部・全スラブを対象に実施され、検定比が有限かつ非負。
#[test]
fn design_check_covers_every_member() {
    let mut app = analyzed();
    app.run_design_check();
    assert_no_error(&app, "断面検定");

    let results = app.results.as_ref().expect("解析結果");
    assert_eq!(
        results.member_checks.len(),
        frame_elem_count(&app),
        "検定された部材が全部材分ない"
    );
    assert!(!results.joint_checks.is_empty(), "接合部の検定結果が空");
    assert!(!results.slab_checks.is_empty(), "スラブの検定結果が空");

    let mut checked = 0usize;
    for mc in &results.member_checks {
        assert!(
            !mc.positions.is_empty(),
            "部材 {:?} の検定位置が空",
            mc.elem
        );
        for p in &mc.positions {
            if let squid_n_design_jp::CheckOutcome::Checked(r) = &p.outcome {
                let ratio = r.ratio();
                assert!(
                    ratio.is_finite() && ratio >= 0.0,
                    "部材 {:?} 位置 {} の検定比が異常: {ratio}",
                    mc.elem,
                    p.xi
                );
                checked += 1;
            }
        }
    }
    assert!(checked > 0, "検定が 1 件も実施されていない（全件 Skipped）");
}

// ===================== 8. 二次設計（層指標） =====================

/// 層間変形角・剛性率 Rs・偏心率 Re が全層で算定される。
#[test]
fn story_metrics_computed_for_every_layer() {
    let app = analyzed();
    let results = app.results.as_ref().expect("解析結果");
    let ex = static_of(&app, StaticCaseKey::Seismic(SeismicDir::X));
    let ctx = squid_n_app::summary::metrics_ctx_from_results(Some(results));
    let metrics =
        squid_n_app::summary::compute_story_metrics_with(&app.model, &ex.disp, SeismicDir::X, &ctx);

    assert_eq!(metrics.len(), 4, "層指標の層数（階数 5 − 1）");
    for m in &metrics {
        assert!(m.height > 0.0, "層 {} の階高が正でない", m.name);
        assert!(
            m.drift > 0.0 && m.drift.is_finite(),
            "層 {} の層間変位が異常: {}",
            m.name,
            m.drift
        );
        assert!(
            m.rs > 0.0 && m.rs.is_finite(),
            "層 {} の剛性率が異常: {}",
            m.name,
            m.rs
        );
        assert!(
            m.re >= 0.0 && m.re.is_finite(),
            "層 {} の偏心率が異常: {}",
            m.name,
            m.re
        );
        assert!(m.fes >= 1.0, "層 {} の Fes が 1.0 未満: {}", m.name, m.fes);
    }
}

// ===================== 9. 保有水平耐力・Ds・終局検定 =====================

/// 保有水平耐力・Ds・部材ランク・RC 終局検定が算定される。
#[test]
fn holding_capacity_and_ultimate_checks() {
    let mut app = analyzed();
    app.run_pushover();
    assert_no_error(&app, "増分解析");

    let (holding, ranks) = app
        .compute_holding_capacity()
        .expect("保有水平耐力が算定できるはず");
    assert_eq!(holding.stories.len(), 4, "保有水平耐力の層数");
    assert_eq!(ranks.len(), 4, "層ごとの部材ランク");
    for s in &holding.stories {
        assert!(
            s.qu > 0.0 && s.qu.is_finite(),
            "保有水平耐力 Qu が異常: {}",
            s.qu
        );
        assert!(
            (0.25..=0.55).contains(&s.ds),
            "構造特性係数 Ds が規定の範囲外: {}",
            s.ds
        );
        assert!(
            s.qun > 0.0 && s.qun.is_finite(),
            "必要保有水平耐力 Qun が異常: {}",
            s.qun
        );
        assert!(s.fes >= 1.0, "Fes が 1.0 未満: {}", s.fes);
    }

    let ultimate = app
        .compute_ultimate_checks()
        .expect("終局検定が算定できるはず");
    assert!(!ultimate.is_empty(), "RC 部材の終局検定が 1 件もない");
    for u in &ultimate {
        assert!(u.mu.is_finite() && u.mu > 0.0, "Mu が異常: {}", u.mu);
        assert!(u.qsu.is_finite() && u.qsu > 0.0, "Qsu が異常: {}", u.qsu);
        assert!(
            u.shear_margin.is_finite() && u.shear_margin > 0.0,
            "せん断余裕度が異常: {}",
            u.shear_margin
        );
    }
}

// ===================== 10. 増分解析（プッシュオーバー） =====================

/// 増分解析が終局まで進み、性能曲線・層せん断が力学的に整合する。
#[test]
fn pushover_reaches_ultimate_state() {
    let mut app = prepared();
    app.run_pushover();
    assert_no_error(&app, "増分解析");

    let push = app
        .results
        .as_ref()
        .expect("解析結果")
        .pushover
        .as_ref()
        .expect("増分解析の結果");

    assert!(
        push.steps.len() > 1,
        "確定ステップが 1 つ以下（即座に発散）"
    );
    assert!(!push.capacity_curve.is_empty(), "性能曲線が空");
    assert!(
        push.qu > 0.0 && push.qu.is_finite(),
        "保有水平耐力 Qu が異常: {}",
        push.qu
    );

    // 屋根変位は増分とともに単調増加する。
    let roof: Vec<f64> = push.capacity_curve.iter().map(|c| c.roof_disp).collect();
    assert!(
        roof.windows(2).all(|w| w[1] >= w[0]),
        "性能曲線の屋根変位が単調増加でない: {roof:?}"
    );

    let last = push.capacity_curve.last().expect("最終ステップ");
    assert!(
        (last.base_shear - push.qu).abs() <= push.qu.abs() * 1e-9,
        "最終ステップのベースシア {} が Qu {} と一致しない",
        last.base_shear,
        push.qu
    );
    // 層せん断はベースシアから始まり、上階へ向かって減少する（水平力の累積）。
    assert!(
        (last.story_shear[0] - last.base_shear).abs() <= last.base_shear.abs() * 1e-9,
        "最下層のせん断力がベースシアと一致しない: {:?}",
        last.story_shear
    );
    assert!(
        last.story_shear.windows(2).all(|w| w[0] >= w[1]),
        "層せん断力が上階ほど小さくない: {:?}",
        last.story_shear
    );
    // 最終ステップの部材別応答が記録される（保有水平耐力・部材ランクの入力になる）。
    assert!(
        !push.member_response.is_empty(),
        "最終確定ステップの部材別応答が空"
    );
}

// ===================== 11. 時刻歴応答解析 =====================

/// 線形時刻歴応答解析（サンプル波）が完走し、応答が有限で層応答が記録される。
#[test]
fn time_history_linear_runs() {
    let mut app = prepared();
    app.analysis_cfg.th_dir = ThDir::X;
    app.analysis_cfg.th_nonlinear = false;
    app.run_time_history_sample();
    assert_no_error(&app, "線形時刻歴応答解析");

    let th = app
        .results
        .as_ref()
        .expect("解析結果")
        .time_history
        .as_ref()
        .expect("時刻歴の結果");

    let expected_frames =
        (app.analysis_cfg.th_duration / app.analysis_cfg.th_dt).round() as usize + 1;
    assert_eq!(th.time.len(), expected_frames, "時刻ステップ数");
    assert!(!th.nonlinear, "線形として記録される");
    assert_eq!(
        th.peak_disp.len(),
        app.model.nodes.len(),
        "ピーク変位が全節点分ない"
    );
    assert!(
        th.peak_disp.iter().flatten().all(|v| v.is_finite()),
        "ピーク変位に非有限値がある（発散）"
    );
    assert!(
        th.peak_disp.iter().any(|d| d[0].abs() > 1e-6),
        "X 加振なのに X 方向の応答がゼロ"
    );
    assert_eq!(th.story_drift_angle.len(), 4, "層間変形角の層数");
    assert!(
        th.story_drift_angle
            .iter()
            .all(|a| a.is_finite() && *a > 0.0),
        "層間変形角が異常: {:?}",
        th.story_drift_angle
    );
}

/// 長い波形でも、応答が減衰しきった末尾で偽の非収束を出さない。
///
/// 収束判定の基準ノルムは動的釣り合いの各項の最大を採るが、それだけでは応答が
/// 減衰しきった時刻で 3 項すべてが床（1 N）を下回り、判定が到達不能な絶対値判定へ
/// 化ける。継続時間 120 秒では 202 ステップがこれに当たっていた。解析中に観測した
/// 力のスケールの最大値に対する下限を設けて解消したことを、既定の 10 秒では
/// 現れない長さで固定する（`dev_docs/handoff/非線形時刻歴の収束_申し送り.md` 4.1）。
#[test]
fn time_history_nonlinear_long_duration_has_no_false_non_convergence() {
    let mut app = prepared();
    app.analysis_cfg.th_dir = ThDir::X;
    app.analysis_cfg.th_nonlinear = true;
    app.analysis_cfg.th_duration = 120.0;
    // 刻みは既定より粗くする（この不具合は応答の減衰で決まり、刻みには依らない）。
    // 既定の 0.01 秒では 12000 ステップになり、テスト時間が 5 倍以上に伸びる。
    app.analysis_cfg.th_dt = 0.05;
    app.run_time_history_sample();
    assert_no_error(&app, "非線形時刻歴応答解析（120 秒）");

    let th = app
        .results
        .as_ref()
        .expect("解析結果")
        .time_history
        .as_ref()
        .expect("時刻歴の結果");
    assert_eq!(
        th.non_converged_steps, 0,
        "減衰しきった末尾で偽の非収束が出ている"
    );
    assert!(
        th.peak_disp.iter().flatten().all(|v| v.is_finite()),
        "ピーク変位に非有限値がある"
    );
}

/// 非線形時刻歴応答解析（サンプル波）が完走する。
///
/// かつては既定設定（サンプル波 dt=0.01・継続時間 10 秒・減衰 2%・Newmark-β）で
/// step 50 の Newton 反復が収束せず落ちていた。原因は収束判定の基準ノルムで、
/// 動的外力  だけを基準にしていたため、地動加速度がゼロを横切る時刻
/// （サンプル波は周期 0.5 秒なので t=0.25・0.50…）で基準が消え、判定が
/// 「残差 < 1e-6 N」という到達不能な絶対値判定に化けていた。
#[test]
fn time_history_nonlinear_runs() {
    let mut app = prepared();
    app.analysis_cfg.th_dir = ThDir::X;
    app.analysis_cfg.th_nonlinear = true;
    app.run_time_history_sample();
    assert_no_error(&app, "非線形時刻歴応答解析");

    let th = app
        .results
        .as_ref()
        .expect("解析結果")
        .time_history
        .as_ref()
        .expect("時刻歴の結果");
    assert!(th.nonlinear, "非線形として記録される");
    assert!(
        th.peak_disp.iter().flatten().all(|v| v.is_finite()),
        "ピーク変位に非有限値がある（発散）"
    );
}

// ===================== 12. 保存・読込の往復 =====================

/// プロジェクトファイル（SCZ）へ保存し、読み直してもモデルと結果が保たれる。
#[test]
fn scz_roundtrip_preserves_model_and_results() {
    let mut app = analyzed();
    app.run_design_check();
    clear_error(&mut app);
    // 既定値から変えておき、往復で「既定値に戻ってしまう」誤りを検出できるようにする。
    app.analysis_cfg.th_damping = 0.037;
    app.analysis_cfg.th_damping_model = ThDampingModel::Rayleigh;
    app.analysis_cfg.n_modes = 5;

    let path = test_tmp().join("full_model_roundtrip.scz");
    app.save_project_to(path.clone());
    assert_no_error(&app, "プロジェクト保存");

    let mut reopened = App::default();
    reopened.analysis_cfg.threads = 1;
    reopened.open_project_from(path.clone());
    assert_no_error(&reopened, "プロジェクト読込");

    assert_eq!(reopened.model.nodes.len(), app.model.nodes.len(), "節点数");
    assert_eq!(
        reopened.model.elements.len(),
        app.model.elements.len(),
        "要素数"
    );
    assert_eq!(
        reopened.model.joists().count(),
        app.model.joists().count(),
        "二次部材数"
    );
    assert_eq!(
        reopened.model.floor_regions.len(),
        app.model.floor_regions.len(),
        "スラブ数"
    );
    assert_eq!(
        reopened.model.stories.len(),
        app.model.stories.len(),
        "階数"
    );

    let before = app.results.as_ref().expect("保存前の結果");
    let after = reopened
        .results
        .as_ref()
        .expect("読込後に結果が復元されるはず");
    assert_eq!(after.statics.len(), before.statics.len(), "静的結果の件数");
    assert_eq!(after.combos.len(), before.combos.len(), "組合せ結果の件数");
    assert!(after.modal.is_some(), "固有値解析の結果が復元される");

    // 解析タブの設定値（結果を生成した条件）も往復で保たれる。既定値と異なる値に
    // しておいたので、既定値へ戻ってしまう回帰（設定が保存されない）を検出できる。
    assert_eq!(
        reopened.analysis_cfg.th_damping, app.analysis_cfg.th_damping,
        "時刻歴の減衰比"
    );
    assert_eq!(
        reopened.analysis_cfg.th_damping_model, app.analysis_cfg.th_damping_model,
        "時刻歴の減衰モデル"
    );
    assert_eq!(
        reopened.analysis_cfg.n_modes, app.analysis_cfg.n_modes,
        "固有値解析のモード数"
    );

    std::fs::remove_file(&path).ok();
}

/// ST-Bridge へ書き出し、読み直してもモデル構成が保たれ、そのまま再解析できる。
///
/// 「読める → 書ける → また読める → 解ける」の往復は、取り込み・書き出しの
/// どちらが壊れても落ちる最も安価な回帰テストになる。
///
/// 書き出しは**準備計算の前**（取り込み直後）の状態から行う。準備計算は仕口パネル
/// 要素を `Model::elements` へ追加するが、これは解析用の生成物であって
/// ST-Bridge の部材ではないため書き出されない。準備計算後のモデルと比べると
/// 要素数が一致せず、往復の検証にならない。
#[test]
fn stbridge_roundtrip_is_reanalyzable() {
    let mut app = imported();
    let path = test_tmp().join("full_model_roundtrip.stb");
    app.export_stbridge_to(path.clone());
    assert_no_error(&app, "ST-Bridge 書き出し");

    let mut reimported = App::default();
    reimported.analysis_cfg.threads = 1;
    reimported.import_stbridge_from(path.clone());
    if let Some(e) = &reimported.last_error {
        assert!(
            e.starts_with('⚠'),
            "書き出したファイルの再取り込みが失敗: {e}"
        );
    }
    clear_error(&mut reimported);

    assert_eq!(
        reimported.model.nodes.len(),
        app.model.nodes.len(),
        "往復で節点数が変わる"
    );
    assert_eq!(
        reimported.model.elements.len(),
        app.model.elements.len(),
        "往復で要素数が変わる"
    );
    assert_eq!(
        reimported.model.joists().count(),
        app.model.joists().count(),
        "往復で二次部材数が変わる"
    );
    assert_eq!(
        reimported.model.floor_regions.len(),
        app.model.floor_regions.len(),
        "往復でスラブ数が変わる"
    );
    assert_eq!(
        app.model.floor_regions.len(),
        26,
        "STB 再取り込みで小片 82 に戻らない"
    );
    assert_eq!(reimported.model.floor_regions.len(), 26);

    reimported.run_preparation();
    assert_eq!(
        reimported.model.floor_regions.len(),
        26,
        "準備計算で 26 のまま"
    );
    assert_no_error(&reimported, "往復後の準備計算");
    reimported.run_static_all();
    assert_no_error(&reimported, "往復後の静的解析");

    std::fs::remove_file(&path).ok();
}

// ===================== 既知の欠落 =====================

/// ST-Bridge から取り込んだ小梁が、床の小梁設計で検定される。
///
/// あわせて、各小梁が**自分と同じレベルのスラブ**で検定されていることを確認する。
/// スラブの内包判定は XY 平面へ投影して行うため、レベルを見ないと上下階のスラブが
/// すべて該当し、別階の板厚・室用途・境界寸法で検定されてしまう（エラーは出ない）。
#[test]
fn joist_design_checks_cover_imported_secondary_members() {
    let mut app = analyzed();
    app.run_design_check();
    assert_no_error(&app, "断面検定");

    let results = app.results.as_ref().expect("解析結果");
    let n_joists = app.model.joists().count();
    assert!(
        !results.joist_checks.is_empty(),
        "小梁 {n_joists} 本が 1 件も検定されていない"
    );

    let mut checked = 0;
    for (slab_id, target, jr) in &results.joist_checks {
        let squid_n_app::app::JoistCheckTarget::SecondaryJoist { nodes } = target else {
            continue;
        };
        checked += 1;
        if jr.unchecked {
            continue;
        }
        let key = (nodes[0].0.min(nodes[1].0), nodes[0].0.max(nodes[1].0));
        let sm = app
            .model
            .joists()
            .find(|sm| {
                let a = sm.nodes[0].0.min(sm.nodes[1].0);
                let b = sm.nodes[0].0.max(sm.nodes[1].0);
                (a, b) == key
            })
            .expect("小梁");
        let z_joist = sm
            .nodes
            .iter()
            .map(|n| app.model.nodes[n.index()].coord[2])
            .sum::<f64>()
            / 2.0;
        let slab = app
            .model
            .slabs
            .iter()
            .find(|s| s.id == *slab_id)
            .expect("検定結果の床板が実在する");
        let z_slab = slab.level(&app.model).expect("床板のレベル");
        assert!(
            (z_slab - z_joist).abs() <= 1.0,
            "小梁 {}-{}（Z={z_joist}）が別レベルのスラブ {:?}（Z={z_slab}）で検定されている",
            nodes[0].0,
            nodes[1].0,
            slab_id
        );
    }
    assert_eq!(
        checked, n_joists,
        "取り込んだ小梁がすべて検定されていない（{checked}/{n_joists}）"
    );
}

/// 主架構の面走査（`region_gen`）が、大梁が囲む区画をレベルごとに検出する。
///
/// 床領域は「大梁で囲まれた領域ごとに 1 つ」と定めるため（D1）、その検出が実建物で
/// 期待どおりの数になることを固定する。期待値は Euler の公式（内部面数 `F = E − V + C`）
/// で独立に検算した値である。
#[test]
fn region_gen_finds_beam_bounded_regions() {
    use squid_n_core::region_gen::generate_region_boundaries;
    use std::collections::BTreeMap;

    let app = imported();
    let boundaries = generate_region_boundaries(&app.model);

    let mut per_level: BTreeMap<i64, (usize, f64)> = BTreeMap::new();
    for b in &boundaries {
        let e = per_level.entry(b.level.round() as i64).or_insert((0, 0.0));
        e.0 += 1;
        e.1 += b.area(&app.model);
    }
    let counts: Vec<(i64, usize)> = per_level.iter().map(|(z, (n, _))| (*z, *n)).collect();
    assert_eq!(
        counts,
        vec![(200, 6), (4700, 6), (8700, 6), (12700, 7), (16500, 1)],
        "レベル別の床領域数（Euler の公式による検算値と一致すること）"
    );
    assert_eq!(boundaries.len(), 26, "床領域総数");
    assert_eq!(
        app.model.floor_regions.len(),
        boundaries.len(),
        "取り込み後の床領域数は床領域数 26"
    );

    // 大梁の区画の面積の合計は、そのレベルの床板面積の合計と一致する
    // （床板は小梁で細分されているが、覆う範囲は大梁の区画と同じ）。
    let mut slab_area: BTreeMap<i64, f64> = BTreeMap::new();
    for s in &app.model.slabs {
        let Some(coords) = s.boundary_coords(&app.model) else {
            continue;
        };
        if coords.len() < 3 {
            continue;
        }
        *slab_area.entry(coords[0][2].round() as i64).or_default() +=
            squid_n_load::floor::polygon_area(&coords);
    }
    for (z, (_, area)) in &per_level {
        let s = slab_area.get(z).copied().unwrap_or(0.0);
        assert!(
            (area - s).abs() / s < 1e-6,
            "Z={z}: 床領域の面積 {area} と床板面積 {s} が一致しない"
        );
    }
}

/// 柱・梁が実建物データで壁側の鉛直構面をどれだけ検出できるかを実測して固定する。
///
/// `crates/squid-n-app/tests/wall_model.rs`（壁 1 パネル・雑壁 1 本の最小フィクスチャ）は
/// 頂部にしか梁がなく（`柱脚 4 節点は固定支点`、梁は「頂部で閉路」の 4 本のみ）、
/// 各鉛直構面が「柱 2 本＋頂部の梁 1 本」という開いた U 字にしかならないため、
/// `region_gen::wall` の境界検出を一切通らない（面が 0 件になる）。壁領域検出の
/// 実データによる検証は、本テストが唯一の経路である（`region_gen::wall` は
/// `ElementKind::Wall` の有無を見ず、柱・梁の幾何だけで構面を検出するため、
/// 壁要素が 0 件のこの実フィクスチャでも検出は成立する）。
#[test]
fn region_gen_finds_wall_bounded_regions() {
    use squid_n_core::region_gen::scan_wall_region_boundaries;
    use std::collections::HashSet;

    let app = imported();
    let scan = scan_wall_region_boundaries(&app.model);
    assert_eq!(scan.unclosed, 0, "半辺の後続は一意に定まるはず");

    // 単一の観測値（境界数・合計面積）だけでは、直線の重複統合に不具合があって
    // 同じ構面を 2 回検出していても気づけない。境界どうしが節点集合として
    // 重複していないこと（構面の重複統合ミスの検出）と、面積が必ず正であること
    // （外周面の判別・ニューエル面積算定の破綻の検出）を独立の不変条件として確認する。
    // 本フィクスチャは直交グリッドの S 造建物であり（`import_builds_expected_model` 等
    // 参照）、正の面積を持つ壁境界の構面はすべて軸方向（X または Y）のはずである。
    // 斜め方向の構面に正の面積を持つ境界が現れた場合、`wall_planes` が実際には
    // つながっていない柱の組を誤って直線候補として拾い、構造的に無意味な面を
    // 検出している疑いが強い（列挙した候補直線ごとに面走査をかけるため、
    // 実在しない斜めの「構面」でも部材がたまたま条件を満たせば面ができうる）。
    let mut seen: HashSet<Vec<u32>> = HashSet::new();
    for b in &scan.boundaries {
        let area = b.area(&app.model);
        assert!(area > 0.0, "境界の面積は必ず正: {area}");
        let axis_aligned = b.plane_direction[0].abs() < 1e-6 || b.plane_direction[1].abs() < 1e-6;
        assert!(
            axis_aligned,
            "直交グリッド建物のはずが斜め構面に正の面積を持つ境界がある（構造的に無意味な面の疑い）: {:?}",
            b.plane_direction
        );
        let mut nodes: Vec<u32> = b.boundary.iter().map(|n| n.0).collect();
        nodes.sort_unstable();
        assert!(
            seen.insert(nodes.clone()),
            "同じ節点集合を持つ境界が重複している（構面の重複統合ミスの疑い）: {nodes:?}"
        );
    }

    let total_area: f64 = scan.boundaries.iter().map(|b| b.area(&app.model)).sum();
    assert_eq!(scan.boundaries.len(), 55, "壁側の鉛直構面の境界数");
    assert!(
        (total_area - 1_427_640_000.0).abs() / total_area < 1e-6,
        "境界の合計面積（ニューエルの公式） {total_area}"
    );
}

/// ST-Bridge 取り込み（`assemble.rs`）が `rebuild_wall_regions` を実際に呼び、
/// `model.wall_regions` が `region_gen::wall` の検出結果と一致する件数で
/// 埋まっていること（§5.9 で結線した経路の end-to-end 確認）。
///
/// 本フィクスチャは壁版（`WallPlate`）を 1 枚も持たないため、検出した壁領域は
/// すべて `wall_plate_ids` が空のまま（幾何の検出だけが先に結線され、壁版の取り込み
/// はまだ Step 7+8 本体で行うため）。
#[test]
fn import_populates_wall_regions_from_region_gen() {
    let app = imported();
    assert_eq!(
        app.model.wall_regions.len(),
        55,
        "region_gen_finds_wall_bounded_regions と同じ件数のはず"
    );
    assert!(
        app.model
            .wall_regions
            .iter()
            .all(|r| r.wall_plate_ids.is_empty()),
        "本フィクスチャは壁版を持たないため、壁版の割当は 0 件のはず"
    );
    for (i, r) in app.model.wall_regions.iter().enumerate() {
        assert_eq!(r.id.0, i as u32, "id は配列添字と一致するはず");
    }
    assert!(app.model.validate().is_ok(), "{:?}", app.model.validate());
}

/// 壁領域は「保存 → 読込 → 再度準備計算」を経ても ID・境界が変わらない。
///
/// `WallRegion` の ID は `scan_wall_region_boundaries` の走査順（`model.elements`
/// の並びに依存。D10）で割り当たる。保存・再読込を経て `model.elements` の並びが
/// 保たれなければ、再準備計算のたびに壁領域 ID が振り直され、UI・保存済み結果の
/// 対応付けが壊れる。1 回の準備計算だけでは検出できない回帰のため、
/// 「読込直後の状態」ではなく「再準備計算後の状態」を比較する。
#[test]
fn wall_regions_survive_save_reopen_reprepare() {
    let app = prepared();
    let before = app.model.wall_regions.clone();
    assert!(!before.is_empty(), "本フィクスチャは壁領域を持つはず");

    let path = test_tmp().join("wall_regions_reprepare_roundtrip.scz");
    let mut app = app;
    app.save_project_to(path.clone());
    assert_no_error(&app, "プロジェクト保存");

    let mut reopened = App::default();
    reopened.analysis_cfg.threads = 1;
    reopened.open_project_from(path.clone());
    assert_no_error(&reopened, "プロジェクト読込");

    reopened.run_preparation();
    assert_no_error(&reopened, "再度の準備計算");

    assert_eq!(
        reopened.model.wall_regions, before,
        "保存→読込→再準備計算を経ても壁領域（ID・境界）は変わらないはず"
    );

    std::fs::remove_file(&path).ok();
}

/// 取り込んだ床板が、大梁で囲まれた床領域の境界へ過不足なく収まる。
///
/// 大梁が囲む区画（床領域）は 1 つの境界につき 1 つ（D1）。床領域内の床板
/// （小梁でさらに細分された打設単位）は重複・欠落なく、ちょうど 1 つの
/// 床領域へ割り当たることを固定する。
#[test]
fn slabs_fold_into_regions_without_gaps() {
    use squid_n_core::region_gen::scan_region_boundaries;
    use std::collections::BTreeMap;

    let app = imported();
    let scan = scan_region_boundaries(&app.model);
    assert_eq!(scan.unclosed, 0, "閉じない面走査はない");
    assert!(
        scan.crossings.is_empty(),
        "節点を共有せずに交差する大梁がある: {:?}",
        scan.crossings
    );

    let mut by_region: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut unassigned = Vec::new();
    for (si, slab) in app.model.slabs.iter().enumerate() {
        let Some(coords) = slab.boundary_coords(&app.model) else {
            continue;
        };
        if coords.len() < 3 {
            continue;
        }
        let n = coords.len() as f64;
        let centroid = [
            coords.iter().map(|p| p[0]).sum::<f64>() / n,
            coords.iter().map(|p| p[1]).sum::<f64>() / n,
        ];
        let z = coords[0][2];
        match scan
            .boundaries
            .iter()
            .position(|b| b.is_same_level(z) && b.contains(&app.model, centroid))
        {
            Some(bi) => by_region.entry(bi).or_default().push(si),
            None => unassigned.push(si),
        }
    }

    assert!(
        unassigned.is_empty(),
        "どの床領域にも収まらない床板: {unassigned:?}"
    );
    assert_eq!(unassigned.len(), 0, "未割当 0");
    assert_eq!(
        app.model.floor_regions.len(),
        scan.boundaries.len(),
        "床領域数＝床領域数 26"
    );
    assert_eq!(
        by_region.len(),
        scan.boundaries.len(),
        "床板を持たない床領域はない"
    );
}

// ===================== スナップショット =====================

/// 全解析の代表スカラをスナップショットで固定する。
///
/// 不変量のアサートは「壊れ方」を捕まえるが、「値が静かに変わったこと」自体は
/// 捕まえない。ソルバー・断面性能・荷重分配のいずれかに手を入れて結果が動いた
/// 場合、ここが差分として現れる。意図した変更なら `cargo insta review` で承認する。
///
/// 値は有効数字 4 桁の指数表記へ丸めてある（[`sig4`]。手元と CI の浮動小数差で
/// 偽陽性にならないようにするため）。
#[test]
fn snapshot_key_scalars() {
    let mut app = analyzed();
    app.run_design_check();
    clear_error(&mut app);

    let mut out = String::new();
    let mut line = |k: &str, v: String| {
        out.push_str(k);
        out.push_str(" = ");
        out.push_str(&v);
        out.push('\n');
    };

    // --- モデル構成 ---
    line("model.nodes", app.model.nodes.len().to_string());
    line("model.elements", app.model.elements.len().to_string());
    line("model.joists()", app.model.joists().count().to_string());
    assert_eq!(
        app.model.floor_regions.len(),
        26,
        "スナップショット対象の床領域数"
    );
    line(
        "model.floor_regions",
        app.model.floor_regions.len().to_string(),
    );
    line("model.stories", app.model.stories.len().to_string());

    // --- 準備計算 ---
    let prep = app.preparation.as_ref().expect("準備計算の結果");
    line(
        "prep.total_seismic_weight",
        sig4(prep.summary.total_seismic_weight),
    );
    line("prep.height", sig4(prep.summary.height_mm));
    for s in &prep.stories {
        line(
            &format!("prep.story[{}].seismic_weight", s.name),
            sig4(s.weight),
        );
    }
    let seismic = prep.seismic.as_ref().expect("地震力");
    line("prep.seismic.T", sig4(seismic.t));
    line("prep.seismic.Rt", sig4(seismic.rt));
    line("prep.seismic.base_shear", sig4(seismic.base_shear));
    for r in &seismic.rows {
        line(&format!("prep.seismic[{}].Ai", r.name), sig4(r.ai));
        line(&format!("prep.seismic[{}].Qi", r.name), sig4(r.qi));
    }

    // --- 固有値 ---
    let modal = app
        .results
        .as_ref()
        .expect("解析結果")
        .modal
        .as_ref()
        .expect("固有値");
    for (i, t) in modal.period.iter().enumerate() {
        line(&format!("eigen.T[{i}]"), sig4(*t));
    }

    // --- 静的（DL・EX） ---
    let dl = static_of(&app, StaticCaseKey::User(dl_case_id(&app)));
    line(
        "static.DL.min_uz",
        sig4(dl.disp.iter().map(|d| d[2]).fold(f64::INFINITY, f64::min)),
    );
    line(
        "static.DL.sum_base_axial",
        sig4(base_column_axials(&app, dl).iter().sum::<f64>()),
    );
    let ex = static_of(&app, StaticCaseKey::Seismic(SeismicDir::X));
    for s in &app.model.stories {
        let mx = s
            .node_ids
            .iter()
            .filter_map(|n| ex.disp.get(n.index()))
            .map(|d| d[0].abs())
            .fold(0.0_f64, f64::max);
        line(&format!("static.EX.story[{}].max_ux", s.name), sig4(mx));
    }

    // --- 断面検定 ---
    let results = app.results.as_ref().expect("解析結果");
    line(
        "design.member_checks",
        results.member_checks.len().to_string(),
    );
    line(
        "design.joint_checks",
        results.joint_checks.len().to_string(),
    );
    line(
        "design.joist_checks",
        results.joist_checks.len().to_string(),
    );
    line("design.slab_checks", results.slab_checks.len().to_string());
    let max_ratio = results
        .member_checks
        .iter()
        .flat_map(|mc| mc.positions.iter())
        .filter_map(|p| match &p.outcome {
            squid_n_design_jp::CheckOutcome::Checked(r) => Some(r.ratio()),
            squid_n_design_jp::CheckOutcome::Skipped { .. } => None,
        })
        .fold(0.0_f64, f64::max);
    line("design.max_ratio", sig4(max_ratio));
    // 小梁の最大検定比。件数だけでは「どのスラブで検定したか」の変化を捉えられないため、
    // 値そのものも固定する（負担幅・床荷重強度の取り違えはここに現れる）。
    let joist_max_ratio = results
        .joist_checks
        .iter()
        .filter(|(_, _, r)| !r.unchecked)
        .map(|(_, _, r)| r.ratio)
        .fold(0.0_f64, f64::max);
    line("design.joist_max_ratio", sig4(joist_max_ratio));

    // --- 層指標 ---
    let ctx = squid_n_app::summary::metrics_ctx_from_results(Some(results));
    let metrics =
        squid_n_app::summary::compute_story_metrics_with(&app.model, &ex.disp, SeismicDir::X, &ctx);
    for m in &metrics {
        line(
            &format!("metrics[{}].drift_angle", m.name),
            sig4(m.drift_angle),
        );
        line(&format!("metrics[{}].Rs", m.name), sig4(m.rs));
        line(&format!("metrics[{}].Re", m.name), sig4(m.re));
    }

    // --- 増分解析・保有水平耐力 ---
    app.run_pushover();
    clear_error(&mut app);
    let push = app
        .results
        .as_ref()
        .expect("解析結果")
        .pushover
        .as_ref()
        .expect("増分解析");
    line("pushover.steps", push.steps.len().to_string());
    line("pushover.Qu", sig4(push.qu));
    line("pushover.mechanism", format!("{:?}", push.mechanism));
    line("pushover.hinges", push.hinges.len().to_string());

    let (holding, _) = app.compute_holding_capacity().expect("保有水平耐力");
    for (i, s) in holding.stories.iter().enumerate() {
        line(&format!("holding[{i}].Qu"), sig4(s.qu));
        line(&format!("holding[{i}].Qun"), sig4(s.qun));
        line(&format!("holding[{i}].Ds"), sig4(s.ds));
        line(&format!("holding[{i}].Fes"), sig4(s.fes));
    }
    line(
        "ultimate.checks",
        app.compute_ultimate_checks()
            .expect("終局検定")
            .len()
            .to_string(),
    );

    // --- 時刻歴（線形） ---
    app.analysis_cfg.th_dir = ThDir::X;
    app.analysis_cfg.th_nonlinear = false;
    app.run_time_history_sample();
    clear_error(&mut app);
    let th = app
        .results
        .as_ref()
        .expect("解析結果")
        .time_history
        .as_ref()
        .expect("時刻歴");
    line("th.frames", th.time.len().to_string());
    line(
        "th.peak_ux",
        sig4(th.peak_disp.iter().map(|d| d[0].abs()).fold(0.0, f64::max)),
    );
    for (i, a) in th.story_drift_angle.iter().enumerate() {
        line(&format!("th.drift_angle[{i}]"), sig4(*a));
    }

    // --- 時刻歴（非線形） ---
    app.analysis_cfg.th_nonlinear = true;
    app.run_time_history_sample();
    clear_error(&mut app);
    let th = app
        .results
        .as_ref()
        .expect("解析結果")
        .time_history
        .as_ref()
        .expect("時刻歴");
    line("th_nl.frames", th.time.len().to_string());
    line(
        "th_nl.peak_ux",
        sig4(th.peak_disp.iter().map(|d| d[0].abs()).fold(0.0, f64::max)),
    );
    for (i, a) in th.story_drift_angle.iter().enumerate() {
        line(&format!("th_nl.drift_angle[{i}]"), sig4(*a));
    }

    insta::assert_snapshot!(out);
}
