//! 準備計算（解析前の前処理）とその結果の保持。
//!
//! 一貫計算では、応力解析に先立って「解析入力を確定させる計算」＝準備計算を
//! 行う。本モジュールはその実行（[`App::run_preparation`]）と、利用者が
//! 解析前に確認するための結果（[`PreparationResult`]）を提供する。GUI 非依存で
//! あり、テスト・レポート出力（`crate::summary`）からも利用する。
//!
//! 準備計算が確定させる内容:
//!
//! 1. **階（層）データ** — 節点の標高から階高・剛床・地震用重量・主要構造種別を
//!    算定してモデルへ反映する。以降のすべての項目の前提であり、
//!    [`App::run_preparation`] は実行のたびに再生成する（利用者の手入力である
//!    地震用重量の手入力値・階の種別は引き継がれる）。
//! 2. **剛域** — 部材端の剛域長 λ・柱フェース距離を接続部材のせいから自動算定し
//!    モデルへ反映する（`ZoneSource::Manual` の端は保護される）。
//! 3. **床荷重・自重・積載** — スラブの床荷重分配、躯体自重、用途別の積載荷重を
//!    標準荷重ケース（DL・LL(架構用)・LL(地震用)）へ同期する。
//! 4. **地震力（Ai 分布）** — 設計用固有周期 T から Rt・αi・Ai・Ci・Qi・Pi を
//!    算定し、水平力を EX・EY ケースへ同期する。
//! 5. **モデル整合性チェック** — 解析を妨げる不備（支点なし・断面未割当など）を
//!    検出する。
//!
//! 地震力が荷重ケース（EX/EY）として確定するため、解析側は
//! 荷重ケース・荷重組合せを解くだけでよく、地震のための専用の解析入口を
//! 別に設けない。

use super::*;

use squid_n_core::ids::MaterialId;
use squid_n_core::model::{
    LoadCaseKind, MemberLoadKind, StoryLevelKind, StoryStructure, ZoneSource,
};

/// 準備計算の結果（解析前の確認用）。GUI 非依存。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PreparationResult {
    /// 実行時刻。
    pub computed_at: SystemTime,
    /// 建物全体の諸元。
    pub summary: PrepSummary,
    /// 階の分布（下階→上階。`model.stories` と同順）。
    pub stories: Vec<PrepStoryRow>,
    /// 地震力（Ai 分布）の算定結果。算定できなかった場合は `None`。
    pub seismic: Option<PrepSeismic>,
    /// 地震力を算定できなかった理由（`seismic` が `None` のとき）。
    pub seismic_note: Option<String>,
    /// 剛域・危険断面位置の算定結果（剛域長 λ または柱フェース距離が
    /// 付いた部材のみ。部材 ID 昇順）。
    pub rigid_zones: Vec<PrepRigidZoneRow>,
    /// 算定対象となった梁要素の総数（`rigid_zones` はそのうち λ・フェース距離の
    /// いずれかが 0 でないもの）。
    pub rigid_zone_candidates: usize,
    /// 断面性能（`model.sections` と同順）。
    pub sections: Vec<PrepSectionRow>,
    /// 鋼断面の幅厚比・部材ランク（断面 × 部材用途 × 材料でまとめる）。
    pub width_thickness: Vec<PrepWidthThicknessRow>,
    /// 部材単位の剛性割増し（スラブ協力幅・合成梁・壁エレメント上下大梁）と
    /// SRC/CFT 等価断面。割増しも等価換算もない部材は含まない。
    pub member_stiffness: Vec<PrepMemberStiffnessRow>,
    /// 剛性割増し・等価換算の算定対象となった梁要素の総数。
    pub member_stiffness_candidates: usize,
    /// ねじり解放（i 端ねじれピン）の対象外となり、ねじり剛性が残る部材。
    /// 部材 ID 昇順。設定が OFF（全部材で保持）のときは空。
    pub torsion_skipped: Vec<PrepTorsionSkipRow>,
    /// ねじり解放の設定が有効か（`Model::beam_torsion == ReleaseIEnd`）。
    /// false のときは全部材でねじり剛性を保持している。
    pub torsion_release_enabled: bool,
    /// 生成した仕口パネル（節点 index の昇順）。設定が OFF のときは空。
    #[serde(default)]
    pub panels: Vec<PrepPanelRow>,
    /// 仕口パネルのモデル化が有効か（`Model::panel_zone` が有効）。
    /// false のときは接合部を剛節点として扱っている。
    #[serde(default)]
    pub panel_modeling_enabled: bool,
    /// 荷重ケースの集計（`model.load_cases` と同順）。
    pub load_cases: Vec<PrepLoadCaseRow>,
    /// モデル整合性チェックのエラー件数。
    pub diag_errors: usize,
    /// モデル整合性チェックの警告件数。
    pub diag_warnings: usize,
}

impl PreparationResult {
    /// 解析を進めてよい状態か（整合性チェックにエラーがないか）。
    pub fn is_ready(&self) -> bool {
        self.diag_errors == 0
    }
}

/// 建物全体の諸元。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepSummary {
    pub n_nodes: usize,
    pub n_elements: usize,
    pub n_stories: usize,
    /// 支点（拘束を持つ節点）の数。階の自動生成が作る剛床代表節点は
    /// 面外拘束を持つが実際の支点ではないため除外する。
    pub n_supports: usize,
    /// 剛床（ダイアフラム）の総数。
    pub n_diaphragms: usize,
    /// 地盤面（GL）標高 [mm]。
    pub ground_elevation: f64,
    /// 建築物の高さ h [mm]（GL から PH 階を除く最上階の床レベルまで）。
    pub height_mm: f64,
    /// 略算周期の鉄骨造高さ比 α。
    pub steel_height_ratio: f64,
    /// 地震用重量の総和 ΣW [N]。
    pub total_seismic_weight: f64,
    /// 質量モデルの方式。
    pub mass_method: squid_n_core::model::MassMethod,
}

/// 階の分布 1 行。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepStoryRow {
    pub name: String,
    /// 床レベル [mm]（モデル座標）。
    pub elevation: f64,
    /// 階高 [mm]（直下階との差。最下階は GL からの高さ）。
    pub height: f64,
    /// その階に属する節点数。
    pub n_nodes: usize,
    /// その階の剛床（ダイアフラム）数。
    pub n_diaphragms: usize,
    /// 地震用重量 Wi [N]（未設定は 0）。
    pub weight: f64,
    /// 当該階以上の累積地震用重量 ΣWj [N]。
    pub cumulative_weight: f64,
    pub structure: StoryStructure,
    pub level_kind: StoryLevelKind,
}

/// 地震力（Ai 分布）の算定諸元と層ごとの結果。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepSeismic {
    /// 設計用一次固有周期 T [s]。
    pub t: f64,
    /// T の算定法（略算／精算）。
    pub t_mode: AiMode,
    /// 地盤種別に応じた Tc [s]。
    pub tc: f64,
    /// 振動特性係数 Rt。
    pub rt: f64,
    pub z: f64,
    pub c0: f64,
    pub soil: squid_n_load::ai::SoilClass,
    /// 層ごとの結果（`stories` と同順＝下階→上階）。
    pub rows: Vec<PrepSeismicRow>,
    /// 基部せん断力 Q1 [N]（最下層の Qi）。
    pub base_shear: f64,
    /// Pi の算定で負値が現れ 0 へクランプしたか（重量分布の異常シグナル）。
    pub clamped_negative_pi: bool,
}

/// 地震力（Ai 分布）の 1 層分。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepSeismicRow {
    pub name: String,
    /// 当該階の地震用重量 Wi [N]。
    pub weight: f64,
    /// 当該階以上の累積地震用重量 ΣWj [N]。
    pub cumulative_weight: f64,
    /// 重量比 αi。
    pub alpha: f64,
    /// 高さ方向分布係数 Ai。
    pub ai: f64,
    /// 層せん断力係数 Ci（PH 階は震度 k、地下階は水平震度 K）。
    pub ci: f64,
    /// 層せん断力 Qi [N]。
    pub qi: f64,
    /// 層の水平外力 Pi [N]。
    pub pi: f64,
    pub level_kind: StoryLevelKind,
}

/// ねじり解放（i 端ねじれピン）の対象外となった部材の 1 行。
///
/// 対象外になるのは「解放すると材軸まわりの回転を拘束するものがない節点が
/// 生じる部材」で、この部材だけがねじり剛性 GJ/L を保持する。ねじり剛性を
/// もともと持たない部材（断面の J≤0・材料の G≤0・トラス扱いのブレース）は
/// 解放してもしなくても剛性が 0 のため、この表には含めない。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepTorsionSkipRow {
    pub elem: ElemId,
    /// 部材種別（柱／梁／ブレース）。
    pub kind: squid_n_design_jp::MemberKind,
    /// 判定に落ちた節点（この節点の材軸まわり回転を拘束するものがない）。
    pub node: NodeId,
}

/// 生成した仕口パネル 1 箇所分。
///
/// S 造（CFT を除く）の柱梁接合節点に設けたパネルの寸法とせん断剛性を示す。
/// `Model::panel_zone` が無効なとき、および RC・SRC 接合部では空になる。
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepPanelRow {
    /// 接合部の節点。
    pub node: NodeId,
    /// 柱せい方向のパネル寸法 dc [mm]。
    pub dc: f64,
    /// 梁フランジ板厚中心間距離 db [mm]。
    pub db: f64,
    /// パネル板厚 tp [mm]。
    pub tp: f64,
    /// 実効体積 Ve [mm³]。
    pub ve: f64,
    /// パネルせん断剛性 Kxp = Kyp = G・Ve [N·mm/rad]。
    pub k_panel: f64,
}

/// 剛域の算定結果 1 部材分。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepRigidZoneRow {
    pub elem: ElemId,
    /// 部材種別（柱／梁／ブレース。部材軸の鉛直成分による幾何判定）。
    pub kind: squid_n_design_jp::MemberKind,
    pub node_i: NodeId,
    pub node_j: NodeId,
    /// 材長 L [mm]（節点間距離）。
    pub length: f64,
    /// i 端・j 端の剛域長 λ [mm]。
    pub zone_i: f64,
    pub zone_j: f64,
    pub source_i: ZoneSource,
    pub source_j: ZoneSource,
    /// i 端・j 端の柱フェース距離 [mm]（危険断面位置の基準）。
    pub face_i: f64,
    pub face_j: f64,
    /// i 端・j 端の仕口パネル分オフセット [mm]（パネルがない端は 0）。
    /// 剛域長とは別の量で、剛体アーム長は両者の大きい方になる。
    pub panel_offset_i: f64,
    pub panel_offset_j: f64,
    /// 可とう長 L' = L − 剛体アーム長 [mm]（剛域長とパネルオフセットの大きい方を控除）。
    pub clear_length: f64,
    /// 剛域比（剛体アーム長の合計）/ L。
    pub ratio: f64,
}

/// 断面性能 1 行（弾性解析に用いる断面諸量）。
///
/// 値は `Section` が保持する解析入力そのもの（形状定義から `to_section` で
/// 生成されたもの、またはカタログ数値の直入力）であり、ここで再計算はしない。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepSectionRow {
    pub section: SectionId,
    /// 断面符号。階と組で断面の同一性キーになるため、単独では一意にならない。
    pub name: String,
    /// 階（階を持たない断面は `None`）。同じ符号の断面を見分けるために併記する。
    pub floor: Option<String>,
    /// 形状種別の表示名（形状定義を持たない断面は `None`）。
    pub shape_label: Option<String>,
    /// この断面を割り当てられた部材の数。
    pub n_elements: usize,
    /// 断面積 A [mm²]。
    pub area: f64,
    /// 断面二次モーメント Iy・Iz [mm⁴]（y=強軸まわり）。
    pub iy: f64,
    pub iz: f64,
    /// ねじり定数 J [mm⁴]。
    pub j: f64,
    /// せん断有効断面積 Asy・Asz [mm²]。
    pub as_y: f64,
    pub as_z: f64,
    /// せい D・幅 B [mm]。
    pub depth: f64,
    pub width: f64,
    /// 断面二次半径 iy・iz [mm]（√(I/A)。座屈長さ比の確認用）。
    pub ry: f64,
    pub rz: f64,
    /// この断面に割り当てられた材料名（複数あれば「〜 他N」）。未割当は `None`。
    pub material: Option<String>,
    /// 代表材料のヤング係数 E [N/mm²]。
    pub young: Option<f64>,
}

/// 鋼断面の幅厚比・部材ランク 1 行。
///
/// ランクは断面形状・部材用途（柱／梁）・鋼種でのみ決まるため、同じ組合せの
/// 部材は 1 行にまとめる（大規模モデルで行数が部材数に比例して増えるのを避ける）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepWidthThicknessRow {
    pub section: SectionId,
    pub section_name: String,
    /// 幅厚比表の行（柱／梁）。
    pub member_use: squid_n_design_jp::secondary::width_thickness::SteelMemberUse,
    pub material: String,
    /// この組合せに該当する部材の数。
    pub n_elements: usize,
    /// 代表最大幅厚比（形状寸法からの簡易法）。算定できない形状は `None`。
    pub max_ratio: Option<f64>,
    /// 幅厚比による部材ランク（FA〜FD）。判定できない形状は `None`。
    pub rank: Option<squid_n_design_jp::secondary::holding_capacity::MemberRank>,
}

/// 部材単位の剛性割増し・SRC/CFT 等価断面 1 行。
///
/// 断面性能の表（断面単位）では表せない、**部材ごとに決まる**剛性の割増しを示す。
/// 値は [`squid_n_element::beam::stiffness_breakdown`] ・
/// [`squid_n_element::beam::composite_props_of`] を通した、要素構築が実際に
/// 適用するものと同じ算定結果。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepMemberStiffnessRow {
    pub elem: ElemId,
    /// 部材種別（柱／梁／ブレース）。
    pub kind: squid_n_design_jp::MemberKind,
    pub section_name: String,
    pub material: String,
    /// スラブ協力幅（RC 矩形梁）・合成梁（H 形鋼梁）による強軸曲げ剛性の増大率。
    pub slab_factor: f64,
    /// 壁エレメントモデルの上下大梁の剛性倍率（軸・曲げ・ねじり・せん断に一律）。
    pub wall_girder_factor: f64,
    /// SRC/CFT 等価断面性能（対象外・算定不能なら `None`）。
    pub composite: Option<PrepCompositeProps>,
    /// 元断面の A・Iy（強軸）[mm²・mm⁴]。等価換算・割増しとの比較用。
    pub section_area: f64,
    pub section_iy: f64,
    /// 割増し・等価換算をすべて適用した後の強軸曲げ剛性用 I [mm⁴]。
    /// 元断面 Iy に対する比が「実際に効いている総増大率」になる。
    pub effective_iy: f64,
    /// 同じく軸剛性用の断面積 [mm²]。
    pub effective_area: f64,
}

/// SRC/CFT の等価断面性能（[`squid_n_core::section_shape::CompositeProps`] の
/// 保存・表示用の写し）。
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepCompositeProps {
    /// 軸剛性用断面積 [mm²]。
    pub area_ax: f64,
    /// 強軸・弱軸の断面二次モーメント [mm⁴]。
    pub iy: f64,
    pub iz: f64,
    /// ねじり定数 [mm⁴]。
    pub j: f64,
    /// せん断有効断面積 [mm²]。
    pub as_y: f64,
    pub as_z: f64,
}

/// 荷重ケースの集計 1 行。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrepLoadCaseRow {
    pub name: String,
    pub kind: LoadCaseKind,
    pub n_nodal: usize,
    pub n_member: usize,
    /// 節点荷重・部材荷重を合わせた全体座標系の合力 [N]。
    /// 部材荷重の合力は集中荷重 p、分布荷重は (w1+w2)/2·(b−a) を作用方向へ投影して
    /// 積算する（等価節点力に置換する前の外力の総和であり、支点反力と釣り合う）。
    pub sum_force: [f64; 3],
}

/// 剛域比がこの値を超える部材を「剛域が大きい」として注意喚起する
/// （可とう長が材長の半分を切ると応力・剛性が剛域の設定に強く支配されるため、
/// 直交材せいの入力ミスを疑う目安として用いる）。
pub const RIGID_ZONE_RATIO_WARN: f64 = 0.5;

impl App {
    /// 準備計算を実行する（利用者が明示的に実行する入口）。
    ///
    /// 階（層）の生成は準備計算の一工程であり、実行のたびに
    /// [`App::generate_stories_action`] で再生成する（節点・断面・荷重の変更が
    /// 階高・剛床・地震用重量へ反映されるようにするため。階高・剛床・地震用重量は
    /// 剛域以外のすべての準備計算項目の前提となる）。利用者の手入力
    /// （地震用重量の手入力値・階の種別）は再生成後も引き継がれる。
    ///
    /// 階を生成できないモデル（節点がない・単一レベルのみ）でも中断せず、
    /// 生成エラーを `last_error` に残したまま残りの項目（剛域・断面性能・
    /// 整合性チェックなど、階を前提としない項目）を集計する。
    ///
    /// 以降は [`App::refresh_preparation`] と同じ処理を行い、結果を
    /// `self.core.scoped.preparation` へ格納する。
    pub fn run_preparation(&mut self) {
        self.core.scoped.last_error = None;
        self.core.scoped.last_notice = None;
        // 階の生成に失敗しても `last_error` は以降で上書きされない
        // （`refresh_preparation` と `report_info` はエラーを設定しない）。
        self.generate_stories_action();
        let story_error = self.core.scoped.last_error.is_some();
        self.refresh_preparation();
        if story_error {
            self.report_info("準備計算を実行しました（階の生成に失敗したため、階を前提とする項目は算定していません）");
            return;
        }
        let msg = match self.core.scoped.preparation.as_ref() {
            Some(p) if !p.is_ready() => format!(
                "準備計算を実行しました（整合性チェック: エラー {} 件・警告 {} 件）。解析前に不備を解消してください。",
                p.diag_errors, p.diag_warnings
            ),
            Some(p) => format!(
                "準備計算を実行しました（階 {} ・剛域 {} 部材・整合性チェック警告 {} 件）",
                p.stories.len(),
                p.rigid_zones.len(),
                p.diag_warnings
            ),
            None => "準備計算を実行しました".to_string(),
        };
        self.report_info(msg);
    }

    /// 解析の実行前に準備計算の結果を最新化する（各解析エントリの先頭で呼ぶ）。
    ///
    /// 剛域の反映・荷重の同期は冪等であり、`sync_auto_load_cases_action` が
    /// モデル・関連設定のハッシュで再計算要否を判定するため、毎回呼んでも
    /// 重い再計算は繰り返さない。階の自動生成は行わない
    /// （解析の実行が暗黙にモデルの階構成を書き換えないようにするため。
    /// 階が必要な解析は階が未定義であればエラーで案内する）。
    pub(crate) fn ensure_preparation(&mut self) {
        self.refresh_preparation();
    }

    /// 剛域の反映・荷重の同期・整合性チェックを行い、結果を集計して
    /// `self.core.scoped.preparation` へ格納する。モデルの階構成は変更しない。
    fn refresh_preparation(&mut self) {
        self.apply_parallelism_setting();
        // 剛域の自動算定と DL/LL/EX/EY の同期（内部で冪等・ハッシュによる
        // スキップ判定あり）。
        self.sync_auto_load_cases_action();
        // 準備計算タブの診断件数は `build_preparation_result` が
        // `diagnostics_counts` を読む。stale のときだけ更新すると、モデル未編集の
        // まま診断タブを開かずに準備計算だけ再実行したときに件数が古いまま残る。
        // 解析実行時の precheck は都度最新だが、画面上の件数は準備計算の結果に載る。
        self.run_diagnostics();
        self.core.scoped.preparation = Some(self.build_preparation_result());
        // 荷重同期が `mark_edited`（＝preparation_stale = true）を呼びうるため、
        // フラグのクリアは必ず集計の後に行う。
        self.core.scoped.staleness.preparation_stale = false;
    }

    /// 現在のモデル・解析設定から準備計算の結果を集計する（モデルは変更しない）。
    fn build_preparation_result(&self) -> PreparationResult {
        let (diag_errors, diag_warnings) = self.diagnostics_counts();
        let (seismic, seismic_note) = self.build_prep_seismic();
        let (rigid_zones, rigid_zone_candidates) = self.build_prep_rigid_zones();
        let (member_stiffness, member_stiffness_candidates) = self.build_prep_member_stiffness();
        let torsion_skipped = self.build_prep_torsion_skipped();
        PreparationResult {
            computed_at: SystemTime::now(),
            summary: self.build_prep_summary(),
            stories: self.build_prep_stories(),
            seismic,
            seismic_note,
            rigid_zones,
            rigid_zone_candidates,
            sections: self.build_prep_sections(),
            width_thickness: self.build_prep_width_thickness(),
            member_stiffness,
            member_stiffness_candidates,
            torsion_skipped,
            torsion_release_enabled: self.core.model.beam_torsion
                == squid_n_core::model::BeamTorsionMode::ReleaseIEnd,
            panels: self
                .core
                .scoped
                .generated_panels
                .iter()
                .map(|p| PrepPanelRow {
                    node: p.node,
                    dc: p.dc,
                    db: p.db,
                    tp: p.tp,
                    ve: p.ve,
                    k_panel: p.k_panel,
                })
                .collect(),
            panel_modeling_enabled: self.core.model.panel_zone.is_enabled(),
            load_cases: self.build_prep_load_cases(),
            diag_errors,
            diag_warnings,
        }
    }

    fn build_prep_summary(&self) -> PrepSummary {
        let model = &self.core.model;
        PrepSummary {
            n_nodes: model.nodes.len(),
            n_elements: model.elements.len(),
            // 「階数」は法規上の層の数（`Model::layers`）を表示する。
            n_stories: model.layer_count(),
            n_supports: {
                let generated: std::collections::HashSet<NodeId> =
                    model.generated_masters.iter().copied().collect();
                model
                    .nodes
                    .iter()
                    .filter(|n| n.restraint.0 != 0 && !generated.contains(&n.id))
                    .count()
            },
            n_diaphragms: model
                .stories
                .iter()
                .map(|s| model.diaphragms_of(s.id).count())
                .sum(),
            ground_elevation: squid_n_solver::analysis::ground_elevation(model),
            height_mm: squid_n_solver::analysis::building_height_mm(model),
            steel_height_ratio: squid_n_solver::analysis::steel_height_ratio(model),
            // 地震用重量の合計は層の重量の総和。基部の階の重量（柱脚・基礎梁）は
            // 地盤が直接受けて層せん断力を生まないため含めない。
            total_seismic_weight: model.layers().iter().map(|l| l.weight.unwrap_or(0.0)).sum(),
            mass_method: model.mass_method,
        }
    }

    /// 層（法規上の「i 階」）ごとの分布表。名前は下端の階、重量・所属節点・剛床・
    /// 階種別は上端の階から採る（`squid_n_core::model::Layer`）。
    fn build_prep_stories(&self) -> Vec<PrepStoryRow> {
        let model = &self.core.model;
        let layers = model.layers();
        let weights: Vec<f64> = layers.iter().map(|l| l.weight.unwrap_or(0.0)).collect();
        layers
            .iter()
            .map(|l| PrepStoryRow {
                name: l.name.clone(),
                elevation: l.top_elevation,
                height: l.height,
                n_nodes: l.node_ids.len(),
                n_diaphragms: model.diaphragms_of(l.top).count(),
                weight: weights[l.index],
                cumulative_weight: weights[l.index..].iter().sum(),
                structure: l.structure,
                level_kind: l.level_kind,
            })
            .collect()
    }

    /// Ai 分布（層せん断力の分布）を算定する。階が未定義・地震用重量が全 0・
    /// 精算周期を選択して固有値解析が未実行、のいずれかでは `None` と理由を返す。
    fn build_prep_seismic(&self) -> (Option<PrepSeismic>, Option<String>) {
        if self.core.model.stories.is_empty() {
            return (
                None,
                Some(
                    "階(Story)が未定義のため地震力(Ai分布)を算定できません。\
                     「① 準備計算」パネルの「準備計算 実行」を行ってください。"
                        .to_string(),
                ),
            );
        }
        let t = match self.design_seismic_period() {
            Ok(t) => t,
            Err(msg) => return (None, Some(msg)),
        };
        let cfg = squid_n_solver::analysis::SeismicCfg {
            // Ai 分布は加力方向によらないため方向は結果に影響しない。
            dir: SeismicDir::X,
            mode: self.core.analysis_cfg.ai_mode,
            z: self.core.analysis_cfg.z,
            soil: self.core.analysis_cfg.soil,
            c0: self.core.analysis_cfg.c0,
        };
        let dist = match squid_n_solver::analysis::seismic_distribution_for_model(
            &self.core.model,
            cfg,
            t,
        ) {
            Ok(d) => d,
            Err(e) => return (None, Some(format!("地震力(Ai分布)の算定エラー: {}", e))),
        };
        let tc = squid_n_load::ai::tc_of(cfg.soil);
        // Ai 分布は層ごと。名前は下端の階、重量・階種別は上端の階から採る。
        let layers = self.core.model.layers();
        let weights: Vec<f64> = layers.iter().map(|l| l.weight.unwrap_or(0.0)).collect();
        let rows: Vec<PrepSeismicRow> = layers
            .iter()
            .map(|l| {
                let i = l.index;
                PrepSeismicRow {
                    name: l.name.clone(),
                    weight: weights[i],
                    cumulative_weight: weights[i..].iter().sum(),
                    alpha: dist.alpha.get(i).copied().unwrap_or(0.0),
                    ai: dist.ai.get(i).copied().unwrap_or(0.0),
                    ci: dist.ci.get(i).copied().unwrap_or(0.0),
                    qi: dist.qi.get(i).copied().unwrap_or(0.0),
                    pi: dist.pi.get(i).copied().unwrap_or(0.0),
                    level_kind: l.level_kind,
                }
            })
            .collect();
        let seismic = PrepSeismic {
            t,
            t_mode: self.core.analysis_cfg.ai_mode,
            tc,
            rt: squid_n_load::ai::rt(t, tc),
            z: cfg.z,
            c0: cfg.c0,
            soil: cfg.soil,
            base_shear: dist.qi.first().copied().unwrap_or(0.0),
            clamped_negative_pi: dist.clamped_negative_pi,
            rows,
        };
        (Some(seismic), None)
    }

    /// 剛域長 λ または柱フェース距離を持つ梁要素を一覧化する。
    /// 返り値は `(該当部材の行, 梁要素の総数)`。
    ///
    /// λ とフェース距離は別概念であり（設計書 §6.2.1）、S 造の仕口では
    /// λ = 0 でもフェース距離は付く。危険断面位置の確認のため、
    /// λ = 0 でもフェース距離を持つ部材は一覧に含める。
    fn build_prep_rigid_zones(&self) -> (Vec<PrepRigidZoneRow>, usize) {
        let model = &self.core.model;
        let mut candidates = 0usize;
        let mut rows = Vec::new();
        for e in &model.elements {
            if !matches!(e.kind, squid_n_core::model::ElementKind::Beam) || e.nodes.len() < 2 {
                continue;
            }
            candidates += 1;
            let rz = e.rigid_zone;
            if rz.length_i <= 0.0
                && rz.length_j <= 0.0
                && rz.face_i_or_zero() <= 0.0
                && rz.face_j_or_zero() <= 0.0
            {
                continue;
            }
            let (Some(ni), Some(nj)) = (
                model.nodes.get(e.nodes[0].index()),
                model.nodes.get(e.nodes[1].index()),
            ) else {
                continue;
            };
            let d = [
                nj.coord[0] - ni.coord[0],
                nj.coord[1] - ni.coord[1],
                nj.coord[2] - ni.coord[2],
            ];
            let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            let (arm_i, arm_j) = (rz.rigid_length_i(), rz.rigid_length_j());
            let clear_length = length - arm_i - arm_j;
            rows.push(PrepRigidZoneRow {
                elem: e.id,
                kind: squid_n_design_jp::MemberKind::of_element(e, model),
                node_i: e.nodes[0],
                node_j: e.nodes[1],
                length,
                zone_i: rz.length_i,
                zone_j: rz.length_j,
                source_i: rz.source_i,
                source_j: rz.source_j,
                face_i: rz.face_i_or_zero(),
                face_j: rz.face_j_or_zero(),
                panel_offset_i: rz.panel_offset_i,
                panel_offset_j: rz.panel_offset_j,
                clear_length,
                ratio: if length > 0.0 {
                    (arm_i + arm_j) / length
                } else {
                    0.0
                },
            });
        }
        (rows, candidates)
    }

    /// ねじり解放（i 端ねじれピン）の対象外となった部材を一覧化する。
    ///
    /// 「ねじり剛性が残っている部材」を確認するための表なので、判定で除外された
    /// 部材（[`squid_n_element::beam::TorsionReleaseSkip::UnrestrainedRotation`]）
    /// だけを載せる。ねじり剛性をもともと持たない部材は、解放してもしなくても
    /// 剛性が 0 で設計上の影響がないため対象外とする。
    fn build_prep_torsion_skipped(&self) -> Vec<PrepTorsionSkipRow> {
        use squid_n_element::beam::TorsionReleaseSkip;
        let model = &self.core.model;
        let mut rows = Vec::new();
        for e in &model.elements {
            let Some(TorsionReleaseSkip::UnrestrainedRotation { node }) =
                squid_n_element::beam::i_end_torsion_release_skip(e, model)
            else {
                continue;
            };
            // ねじり剛性をもともと持たない部材は載せない（J≤0 または G≤0）。
            let j = e
                .section
                .and_then(|sid| model.sections.get(sid.index()))
                .map(|s| s.j)
                .unwrap_or(0.0);
            let g = model
                .element_material(e)
                .map(|m| m.shear_modulus())
                .unwrap_or(0.0);
            if j <= 0.0 || g <= 0.0 {
                continue;
            }
            rows.push(PrepTorsionSkipRow {
                elem: e.id,
                kind: squid_n_design_jp::MemberKind::of_element(e, model),
                node,
            });
        }
        rows
    }

    /// 断面性能を一覧化する。断面ごとに使用部材数と、断面が持つ主材料を添える。
    fn build_prep_sections(&self) -> Vec<PrepSectionRow> {
        let model = &self.core.model;
        // 断面 → 使用部材数
        let mut usage: Vec<usize> = vec![0; model.sections.len()];
        let mut count = |sid: Option<squid_n_core::ids::SectionId>| {
            if let Some(slot) = sid.and_then(|s| usage.get_mut(s.index())) {
                *slot += 1;
            }
        };
        for e in &model.elements {
            count(e.section);
        }
        // 床板も断面を参照する（板厚・自重の情報源）。数えないと、床だけが使う断面が
        // 「使用部材数 0」として淡色表示され、未使用の断面と見分けられなくなる。
        for s in &model.slabs {
            count(s.section());
        }

        model
            .sections
            .iter()
            .enumerate()
            .map(|(i, sec)| {
                let n_elements = usage.get(i).copied().unwrap_or(0);
                let first_mat = sec
                    .material
                    .and_then(|mid| model.materials.get(mid.index()));
                let material = first_mat.map(|m| m.name.clone());
                // 断面二次半径 i = √(I/A)。A が 0 の断面（未設定）は 0 とする。
                let radius = |inertia: f64| {
                    if sec.area > 0.0 && inertia > 0.0 {
                        (inertia / sec.area).sqrt()
                    } else {
                        0.0
                    }
                };
                PrepSectionRow {
                    section: sec.id,
                    name: sec.name.clone(),
                    floor: sec.floor.clone(),
                    shape_label: sec
                        .shape
                        .as_ref()
                        .map(|sh| section_shape_label(sh).to_string()),
                    n_elements,
                    area: sec.area,
                    iy: sec.iy,
                    iz: sec.iz,
                    j: sec.j,
                    as_y: sec.as_y,
                    as_z: sec.as_z,
                    depth: sec.depth,
                    width: sec.width,
                    ry: radius(sec.iy),
                    rz: radius(sec.iz),
                    material,
                    young: first_mat.map(|m| m.young),
                }
            })
            .collect()
    }

    /// 鋼部材の幅厚比・部材ランクを、断面 × 部材用途（柱／梁）× 材料でまとめる。
    ///
    /// ランクの判定は保有水平耐力の Ds 算定と同じ [`super::steel_width_thickness_rank`]
    /// を用いる（表示と算定の乖離を避けるため）。形状定義を持たない断面
    /// （カタログ数値の直入力等）と非鋼材は対象外。
    fn build_prep_width_thickness(&self) -> Vec<PrepWidthThicknessRow> {
        use squid_n_design_jp::secondary::width_thickness::{max_width_thickness, SteelMemberUse};
        let model = &self.core.model;
        let mut rows: Vec<PrepWidthThicknessRow> = Vec::new();
        let mut index: std::collections::HashMap<(SectionId, SteelMemberUse, MaterialId), usize> =
            std::collections::HashMap::new();

        for e in &model.elements {
            let Some(sid) = e.section else {
                continue;
            };
            let Some(sec) = model.sections.get(sid.index()) else {
                continue;
            };
            let (Some(mid), Some(mat)) = (sec.material, model.element_material(e)) else {
                continue;
            };
            if !super::elem_is_steel(e, model) {
                continue;
            }
            let Some(shape) = sec.shape.as_ref() else {
                continue;
            };
            let member_use = super::steel_member_use_of(e, model);
            let key = (sid, member_use, mid);
            if let Some(&i) = index.get(&key) {
                rows[i].n_elements += 1;
                continue;
            }
            index.insert(key, rows.len());
            rows.push(PrepWidthThicknessRow {
                section: sid,
                section_name: sec.name.clone(),
                member_use,
                material: mat.name.clone(),
                n_elements: 1,
                max_ratio: max_width_thickness(shape),
                rank: super::steel_width_thickness_rank(shape, member_use, &mat.name),
            });
        }
        rows
    }

    /// 部材単位の剛性割増し（スラブ協力幅・合成梁・壁エレメント上下大梁）と
    /// SRC/CFT 等価断面を一覧化する。返り値は `(該当部材の行, 梁要素の総数)`。
    ///
    /// 割増しも等価換算も生じ得ないモデル（スラブ剛性を考慮しない・壁がない・
    /// SRC/CFT 断面がない）では、部材ごとの判定（`O(部材数)` の走査を含む）を
    /// 行わずに空を返す。
    ///
    /// 壁の解析要素（`ElementKind::Wall`）は `self.core.model` には存在しない生成物
    /// （D5）のため、壁上下大梁の剛性割増しを確認表へ出すには壁展開モデルを
    /// 都度組み立てて走査する（`App` に専用のキャッシュは持たせない。
    /// `squid_n_load::wall_expand` の「都度計算」方針に揃える）。壁展開は
    /// 既存要素の末尾へ生成要素を追記するだけなので、柱・梁の `ElemId` は
    /// 展開の有無で変わらない。壁を持たないモデル（実 ST-Bridge フィクスチャは
    /// 現状すべて該当する）では `expand_wall_elements` の `model.clone()` を
    /// 避け、`self.core.model` をそのまま見る。
    fn build_prep_member_stiffness(&self) -> (Vec<PrepMemberStiffnessRow>, usize) {
        use squid_n_core::model::{ElementKind, Model};
        use squid_n_core::section_shape::SectionShape;

        let expanded_storage;
        let model: &Model =
            if squid_n_load::wall_expand::model_has_wall_plates_to_expand(&self.core.model) {
                let (expanded, _wall_index, _wall_report) =
                    squid_n_load::wall_expand::expand_wall_elements(&self.core.model);
                expanded_storage = expanded;
                &expanded_storage
            } else {
                &self.core.model
            };
        let candidates = model
            .elements
            .iter()
            .filter(|e| matches!(e.kind, ElementKind::Beam) && e.nodes.len() >= 2)
            .count();

        // 事前判定: どれか 1 つでも該当しうる場合のみ部材ごとの算定へ進む。
        // スラブ協力幅・合成梁の板厚は「該当する床板の `slab_plate_thickness`
        // （複数なら最大）、取れないときの控えが建物一律 `slab_thickness`」の順で
        // 決まる（`squid_n_element::beam::stiffness_factors`）。事前判定もこれに
        // 揃え、建物一律だけでなく個々の床板の板厚も見る（さもないと `slab_thickness`
        // 未設定＝既定 0 のモデルで、実際は割増しが効く床板があっても表が空になる）。
        let slab_stiffness_enabled = (!model.floor_regions.is_empty() || !model.slabs.is_empty())
            && (model.slab_thickness > 0.0
                || model
                    .slabs
                    .iter()
                    .any(|s| model.slab_plate_thickness(s).is_some()));
        let has_wall_element = model
            .elements
            .iter()
            .any(|e| matches!(e.kind, ElementKind::Wall) && e.nodes.len() >= 4);
        // 複合断面（SRC・CFT）の判定は `squid_n_core::structure_kind` に一元化する。
        let has_composite_section = model.sections.iter().any(|s| {
            s.shape
                .as_ref()
                .and_then(squid_n_core::structure_kind::shape_composite_kind)
                .is_some()
        });
        if !(slab_stiffness_enabled || has_wall_element || has_composite_section) {
            return (Vec::new(), candidates);
        }

        let mut rows = Vec::new();
        for e in &model.elements {
            if !matches!(e.kind, ElementKind::Beam) || e.nodes.len() < 2 {
                continue;
            }
            let (Some(sec), Some(mat)) = (
                e.section.and_then(|sid| model.sections.get(sid.index())),
                model.element_material(e),
            ) else {
                continue;
            };
            let factors = squid_n_element::beam::stiffness_breakdown(model, e);
            let composite = squid_n_element::beam::composite_props_of(model, e);
            // 実効値: 等価換算 → スラブ／合成梁（強軸曲げのみ）→ 壁上下大梁（一律）。
            // 要素構築（`BeamElement::new`）が適用するのと同じ順序・同じ規則。
            // SRC で材料から等価換算できない（Fc 未設定等）場合も、軸剛性だけは
            // 既定の累加（`calc_axial_stiffness_area`）が効く点まで一致させる。
            let base_iy = composite.map(|p| p.iy).unwrap_or(sec.iy);
            let base_area = match (composite, sec.shape.as_ref()) {
                (Some(p), _) => p.area_ax,
                (None, Some(shape @ SectionShape::SrcRect { .. })) => {
                    shape.calc_axial_stiffness_area()
                }
                _ => sec.area,
            };
            if factors.slab == 1.0
                && factors.wall_girder == 1.0
                && composite.is_none()
                && base_area == sec.area
            {
                continue;
            }
            rows.push(PrepMemberStiffnessRow {
                elem: e.id,
                kind: squid_n_design_jp::MemberKind::of_element(e, model),
                section_name: sec.name.clone(),
                material: mat.name.clone(),
                slab_factor: factors.slab,
                wall_girder_factor: factors.wall_girder,
                composite: composite.map(|p| PrepCompositeProps {
                    area_ax: p.area_ax,
                    iy: p.iy,
                    iz: p.iz,
                    j: p.j,
                    as_y: p.as_y,
                    as_z: p.as_z,
                }),
                section_area: sec.area,
                section_iy: sec.iy,
                effective_iy: base_iy * factors.slab * factors.wall_girder,
                effective_area: base_area * factors.wall_girder,
            });
        }
        (rows, candidates)
    }

    fn build_prep_load_cases(&self) -> Vec<PrepLoadCaseRow> {
        self.core
            .model
            .load_cases
            .iter()
            .map(|lc| {
                let mut sum = [0.0_f64; 3];
                for nl in &lc.nodal {
                    for (k, s) in sum.iter_mut().enumerate() {
                        *s += nl.values[k];
                    }
                }
                for ml in &lc.member {
                    let magnitude = match ml.kind {
                        MemberLoadKind::Point { p, .. } => p,
                        MemberLoadKind::Distributed { a, b, w1, w2 } => 0.5 * (w1 + w2) * (b - a),
                    };
                    let dir = ml.dir;
                    let norm = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
                    if norm <= 0.0 {
                        continue;
                    }
                    for (k, s) in sum.iter_mut().enumerate() {
                        *s += magnitude * dir[k] / norm;
                    }
                }
                PrepLoadCaseRow {
                    name: lc.name.clone(),
                    kind: lc.kind,
                    n_nodal: lc.nodal.len(),
                    n_member: lc.member.len(),
                    sum_force: sum,
                }
            })
            .collect()
    }
}

/// 断面形状の表示名。
pub fn section_shape_label(shape: &squid_n_core::section_shape::SectionShape) -> &'static str {
    use squid_n_core::section_shape::SectionShape;
    match shape {
        SectionShape::RcRect { .. } => "RC 矩形",
        SectionShape::RcCircle { .. } => "RC 円形",
        SectionShape::RcWall { .. } => "RC 壁",
        SectionShape::RcSlab { .. } => "RC スラブ",
        SectionShape::SrcRect { .. } => "SRC 矩形",
        SectionShape::SteelH { .. } => "H 形鋼",
        SectionShape::SteelBuiltH { .. } => "組立 H 形鋼",
        SectionShape::SteelBox { .. } => "角形鋼管",
        SectionShape::SteelPipe { .. } => "円形鋼管",
        SectionShape::SteelAngle { .. } => "山形鋼",
        SectionShape::SteelChannel { .. } => "溝形鋼",
        SectionShape::SteelLipChannel { .. } => "リップ溝形鋼",
        SectionShape::SteelTee { .. } => "T 形鋼",
        SectionShape::SteelFlatBar { .. } => "平鋼",
        SectionShape::SteelRoundBar { .. } => "丸鋼",
        SectionShape::CftBox { .. } => "CFT 角形",
        SectionShape::CftPipe { .. } => "CFT 円形",
    }
}

/// 幅厚比ランク表の行（柱／梁）の表示名。
pub fn steel_member_use_label(
    u: squid_n_design_jp::secondary::width_thickness::SteelMemberUse,
) -> &'static str {
    use squid_n_design_jp::secondary::width_thickness::SteelMemberUse;
    match u {
        SteelMemberUse::Column => "柱",
        SteelMemberUse::Beam => "梁",
    }
}

/// 部材ランクの表示名。
pub fn member_rank_label(
    r: squid_n_design_jp::secondary::holding_capacity::MemberRank,
) -> &'static str {
    use squid_n_design_jp::secondary::holding_capacity::MemberRank;
    match r {
        MemberRank::FA => "FA",
        MemberRank::FB => "FB",
        MemberRank::FC => "FC",
        MemberRank::FD => "FD",
    }
}

/// 階の構造種別の表示名。
pub fn story_structure_label(s: StoryStructure) -> &'static str {
    match s {
        StoryStructure::Rc => "RC",
        StoryStructure::S => "S",
        StoryStructure::Src => "SRC",
    }
}

/// 階の種別の表示名（PH の震度・地下の深さを含む）。
pub fn story_level_kind_label(k: StoryLevelKind) -> String {
    match k {
        StoryLevelKind::Normal => "一般".to_string(),
        StoryLevelKind::Penthouse { k } => format!("PH(k={:.2})", k),
        StoryLevelKind::Basement { depth_m } => format!("地下(H={:.1}m)", depth_m),
    }
}

/// 荷重ケース種別の表示名（GUI 全体で共有する単一定義）。
///
/// 荷重タブの種別セレクタ・準備計算の荷重集計表・CSV レポートのすべてが
/// 本関数を用いる。従来は荷重タブに別定義があり、同じ種別が画面によって
/// 「積載(架構用)」「積載荷重(長期)」と食い違って表示されていた。
/// 訳語は標準ケース名（DL・LL(架構用)・LL(地震用)）と同じ「架構用/地震用」の
/// 区別に揃える。
pub fn load_case_kind_label(k: LoadCaseKind) -> &'static str {
    match k {
        LoadCaseKind::Dead => "固定荷重",
        LoadCaseKind::Live => "積載荷重(架構用)",
        LoadCaseKind::LiveSeismic => "積載荷重(地震用)",
        LoadCaseKind::Snow => "積雪荷重",
        LoadCaseKind::Wind => "風荷重",
        LoadCaseKind::Seismic => "地震荷重",
        LoadCaseKind::Other => "その他/未設定",
    }
}

/// 剛域の出所の表示名。
pub fn zone_source_label(s: ZoneSource) -> &'static str {
    match s {
        ZoneSource::Auto => "自動",
        ZoneSource::Manual => "手動",
    }
}

/// 部材種別の表示名（GUI 全体で共有する単一定義）。
///
/// 準備計算の各表・終局検定ビュー・CSV レポートのすべてが本関数を用いる。
/// 従来は終局検定ビューに別定義があり、`Brace` が画面によって
/// 「ブレース」「斜材」と食い違って表示されていた。
pub fn member_kind_label(k: squid_n_design_jp::MemberKind) -> &'static str {
    use squid_n_design_jp::MemberKind;
    match k {
        MemberKind::Column => "柱",
        MemberKind::Beam => "梁",
        MemberKind::Brace => "ブレース",
    }
}

/// 地盤種別の表示名。
pub fn soil_class_label(s: squid_n_load::ai::SoilClass) -> &'static str {
    use squid_n_load::ai::SoilClass;
    match s {
        SoilClass::I => "第1種",
        SoilClass::II => "第2種",
        SoilClass::III => "第3種",
    }
}

/// 設計用固有周期の算定法の表示名。
pub fn ai_mode_label(m: AiMode) -> &'static str {
    match m {
        AiMode::Approx => "略算 T=h(0.02+0.01α)",
        AiMode::SemiPrecise => "精算(固有値解析)",
    }
}
