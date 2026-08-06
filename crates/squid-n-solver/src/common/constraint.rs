use crate::common::csc_cache::CscCache;
use faer::sparse::SparseColMat;
use squid_n_core::dof::{Dof, DofMap, DOF_PER_NODE};
use squid_n_core::model::{Constraint, Model};
use squid_n_math::sparse::{assemble_csc, Triplet};

pub struct Reducer {
    pub t_rows: Vec<Vec<(usize, f64)>>,
    pub n_indep: usize,
    pub n_free: usize,
    /// 縮約空間の自由度番号 → 自由 DOF 空間の自由度番号（`n_indep` 長）。
    /// `t_rows` の逆写像だが、従属行が偶然 `[(a, 1.0)]` の形（剛リンクのスレーブ等）に
    /// なるため `t_rows` からは一意に復元できない。縮約後の行列の行・列を
    /// 節点・成分へ翻訳する用途（[`crate::pushover`] の特異診断）に用いる。
    indep_free: Vec<usize>,
}

impl Reducer {
    /// 自由度が 1 つもないモデル向けの空の縮約（全長 0）。
    pub fn empty() -> Self {
        Reducer {
            t_rows: Vec::new(),
            n_indep: 0,
            n_free: 0,
            indep_free: Vec::new(),
        }
    }

    pub fn build(model: &Model, dofmap: &DofMap) -> Self {
        let n_free = dofmap.n_active();
        let n_nodes = model.nodes.len();
        let mut t_rows: Vec<Vec<(usize, f64)>> = (0..n_free).map(|i| vec![(i, 1.0)]).collect();
        let node_coords: Vec<[f64; 3]> = model.nodes.iter().map(|n| n.coord).collect();

        // ダングリング参照（存在しない節点を指す拘束）は panic せず読み飛ばす。
        // ユーザー向けの診断は解析入口の precheck（`statics::analysis::precheck`）が
        // 「拘束が存在しない節点を参照」エラーとして行い、ここは防御のみ担う
        // （`linear_static_once` のように precheck を通らない経路でも落ちないため）。

        // MPC: master フィールドはスレーブ節点、terms は (マスター節点, マスター DOF, 係数)
        for constraint in &model.constraints {
            if let Constraint::Mpc { master, terms } = constraint {
                let slave_node = master.index();
                if slave_node >= n_nodes {
                    continue;
                }
                // スレーブ DOF d を、同じ d のマスター寄与の和で表す
                for d in 0..DOF_PER_NODE {
                    let sg = slave_node * DOF_PER_NODE + d;
                    if let Some(sa) = dofmap.active(sg) {
                        let s_idx = sa as usize;
                        // マスター側の対象 DOF が拘束済み（非 active）の項は変位 0 で
                        // 寄与しないため row から落とす。全項が拘束済みなら空行＝
                        // スレーブも 0 に縮約する（従来は空行のとき恒等写像のまま
                        // 残してスレーブが独立自由度になり、拘束が無言で破れていた。
                        // RigidDiaphragm の扱いと同じ規則に統一）。
                        // ただし当該 DOF にマスター項が 1 つもない場合は MPC の対象外
                        // なので恒等のまま残す。
                        let mut has_term = false;
                        let mut row = Vec::new();
                        for &(m_node, m_dof, coef) in terms {
                            if m_dof as usize == d {
                                has_term = true;
                                if m_node.index() >= n_nodes {
                                    continue;
                                }
                                let mg = m_node.index() * DOF_PER_NODE + d;
                                if let Some(ma) = dofmap.active(mg) {
                                    row.push((ma as usize, coef));
                                }
                            }
                        }
                        if s_idx < t_rows.len() && has_term {
                            t_rows[s_idx] = row;
                        }
                    }
                }
            }
        }

        // RigidLink: 指定 DOF をマスター節点に拘束
        for constraint in &model.constraints {
            if let Constraint::RigidLink {
                master,
                slaves,
                dofs,
            } = constraint
            {
                let mi = master.index();
                if mi >= n_nodes {
                    continue;
                }
                for &slave in slaves {
                    let si = slave.index();
                    if si >= n_nodes {
                        continue;
                    }
                    for d in 0..DOF_PER_NODE {
                        let dof = match d {
                            0 => Dof::Ux,
                            1 => Dof::Uy,
                            2 => Dof::Uz,
                            3 => Dof::Rx,
                            4 => Dof::Ry,
                            _ => Dof::Rz,
                        };
                        if dofs.is_fixed(dof) {
                            let sg = si * DOF_PER_NODE + d;
                            let mg = mi * DOF_PER_NODE + d;
                            if let Some(sa) = dofmap.active(sg) {
                                let s_idx = sa as usize;
                                // マスター DOF が拘束済み（非 active）なら空行＝
                                // スレーブも 0 に縮約する（MPC・RigidDiaphragm と
                                // 同じ規則。従来はスレーブが独立自由度のまま残り、
                                // 剛リンクが無言で破れていた）。
                                let row = match dofmap.active(mg) {
                                    Some(ma) => vec![(ma as usize, 1.0)],
                                    None => Vec::new(),
                                };
                                if s_idx < t_rows.len() {
                                    t_rows[s_idx] = row;
                                }
                            }
                        }
                    }
                }
            }
        }

        // RigidDiaphragm
        for constraint in &model.constraints {
            if let Constraint::RigidDiaphragm { master, slaves, .. } = constraint {
                let mi = master.index();
                let Some(&[mx, my, _]) = node_coords.get(mi) else {
                    continue;
                };
                for &slave in slaves {
                    let si = slave.index();
                    let Some(&[sx, sy, _]) = node_coords.get(si) else {
                        continue;
                    };
                    let dx = sx - mx;
                    let dy = sy - my;
                    // Ux
                    let sg_ux = si * DOF_PER_NODE;
                    let mg_ux = mi * DOF_PER_NODE;
                    let mg_rz = mi * DOF_PER_NODE + 5;
                    if let Some(sa) = dofmap.active(sg_ux) {
                        let s_idx = sa as usize;
                        let mut row = Vec::new();
                        if let Some(ma) = dofmap.active(mg_ux) {
                            row.push((ma as usize, 1.0));
                        }
                        if let Some(ma) = dofmap.active(mg_rz) {
                            row.push((ma as usize, -dy));
                        }
                        if s_idx < t_rows.len() {
                            t_rows[s_idx] = row;
                        }
                    }
                    // Uy
                    let sg_uy = si * DOF_PER_NODE + 1;
                    let mg_uy = mi * DOF_PER_NODE + 1;
                    if let Some(sa) = dofmap.active(sg_uy) {
                        let s_idx = sa as usize;
                        let mut row = Vec::new();
                        if let Some(ma) = dofmap.active(mg_uy) {
                            row.push((ma as usize, 1.0));
                        }
                        if let Some(ma) = dofmap.active(mg_rz) {
                            row.push((ma as usize, dx));
                        }
                        if s_idx < t_rows.len() {
                            t_rows[s_idx] = row;
                        }
                    }
                    // Rz
                    let sg_rz = si * DOF_PER_NODE + 5;
                    if let Some(sa) = dofmap.active(sg_rz) {
                        let s_idx = sa as usize;
                        if let Some(ma) = dofmap.active(mg_rz) {
                            if s_idx < t_rows.len() {
                                t_rows[s_idx] = vec![(ma as usize, 1.0)];
                            }
                        }
                    }
                }
            }
        }

        let mut is_indep = vec![false; t_rows.len()];
        for (i, row) in t_rows.iter().enumerate() {
            if row.len() == 1 && row[0].0 == i && (row[0].1 - 1.0).abs() < 1e-12 {
                is_indep[i] = true;
            }
        }

        // 連鎖拘束の合成: あるスレーブ DOF のマスターがさらに別拘束のスレーブを
        // 兼ねる場合（剛床スレーブが剛リンク/MPC のマスターを兼ねる等）、各従属行を
        // 独立 DOF のみで表すよう推移的に代入する。従来は未合成で、被参照の従属 DOF に
        // 偽の独立番号が割り当てられ、連鎖側の拘束が無言で破れていた。非連鎖の通常
        // ケースでは 1 パスで変化なし（既存挙動と一致）。
        fn merge(row: &mut Vec<(usize, f64)>, idx: usize, coef: f64) {
            if let Some(e) = row.iter_mut().find(|(j, _)| *j == idx) {
                e.1 += coef;
            } else {
                row.push((idx, coef));
            }
        }
        // 非巡回な拘束連鎖は連鎖長回で収束する。上限はサイクル入力への安全弁。
        let max_pass = t_rows.len() + 1;
        for _ in 0..max_pass {
            let mut changed = false;
            for i in 0..t_rows.len() {
                if is_indep[i] {
                    continue;
                }
                // 従属 DOF（自己以外）を参照している行のみ展開する。
                if t_rows[i].iter().any(|&(j, _)| j != i && !is_indep[j]) {
                    let old = std::mem::take(&mut t_rows[i]);
                    let mut newrow: Vec<(usize, f64)> = Vec::new();
                    for (j, c) in old {
                        if is_indep[j] || j == i {
                            merge(&mut newrow, j, c);
                        } else {
                            // マスター j 自身が従属 → その行を係数 c 倍で代入。
                            let sub = t_rows[j].clone();
                            for (k, ck) in sub {
                                merge(&mut newrow, k, c * ck);
                            }
                        }
                    }
                    newrow.retain(|&(_, c)| c.abs() > 1e-15);
                    t_rows[i] = newrow;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }

        let mut new_indep = vec![usize::MAX; t_rows.len()];
        let mut counter = 0usize;
        for i in 0..t_rows.len() {
            if is_indep[i] {
                new_indep[i] = counter;
                counter += 1;
            }
        }
        for row in &t_rows {
            for &(idx, _) in row {
                if idx < new_indep.len() && new_indep[idx] == usize::MAX {
                    new_indep[idx] = counter;
                    counter += 1;
                }
            }
        }

        let remapped: Vec<Vec<(usize, f64)>> = t_rows
            .into_iter()
            .map(|row| {
                row.into_iter()
                    .filter_map(|(idx, val)| {
                        if idx < new_indep.len() && new_indep[idx] != usize::MAX {
                            Some((new_indep[idx], val))
                        } else {
                            None
                        }
                    })
                    .collect()
            })
            .collect();

        // 縮約空間 → 自由 DOF 空間の逆写像。`new_indep[i] != usize::MAX` の i が
        // 縮約後に残る（独立な）自由度なので、その対応をそのまま反転する。
        let mut indep_free = vec![usize::MAX; counter];
        for (i, &r) in new_indep.iter().enumerate() {
            if r != usize::MAX {
                indep_free[r] = i;
            }
        }

        Reducer {
            t_rows: remapped,
            n_indep: counter,
            n_free,
            indep_free,
        }
    }

    /// 縮約空間の自由度 `reduced` に対応する自由 DOF 空間の自由度番号。
    /// 範囲外・対応不明は `None`。
    pub fn free_dof_of(&self, reduced: usize) -> Option<usize> {
        self.indep_free
            .get(reduced)
            .copied()
            .filter(|&i| i != usize::MAX)
    }

    /// 拘束縮約 Tᵀ·K·T を計算する。
    ///
    /// K は疎行列なので、格納された非ゼロ要素（CSC 列ごと）だけを走査する。
    /// 従来の n_free² 全ペア走査＋要素ごとの二分探索 (`get`) を廃し、非ゼロ数に
    /// 比例するコストに落とす（結果は同一）。K[i][j] が列 j・行 i の格納値。
    pub fn reduce_k(&self, k_free: &SparseColMat<usize, f64>) -> SparseColMat<usize, f64> {
        assemble_csc(self.n_indep, self.reduce_k_triplets(k_free))
    }

    /// [`Self::reduce_k`] と同じ triplet 列（Tᵀ·K·T の非ゼロ要素、加算前）を返す。
    /// [`Self::reduce_k`]・[`Self::reduce_k_cached`] が共有する。
    fn reduce_k_triplets(&self, k_free: &SparseColMat<usize, f64>) -> Vec<Triplet> {
        let mut triplets = Vec::new();
        self.reduce_k_triplets_into(k_free, &mut triplets);
        triplets
    }

    /// [`Self::reduce_k_triplets`] の結果を呼び出し側の既存バッファへ書き込む版
    /// （`out` は先頭で `clear()` してから書き込むため、確保済みの容量は保持され、
    /// Newton 反復のように毎回呼ぶ場面で再確保が発生しない）。計算内容・順序は
    /// [`Self::reduce_k_triplets`] と同一（ビット完全一致）。
    fn reduce_k_triplets_into(&self, k_free: &SparseColMat<usize, f64>, out: &mut Vec<Triplet>) {
        out.clear();
        let col_ptr = k_free.col_ptr();
        let row_idx = k_free.row_idx();
        let values = k_free.val();
        let ncols = k_free.ncols();
        for j in 0..ncols {
            let tj_list = &self.t_rows[j];
            if tj_list.is_empty() {
                continue;
            }
            for pos in col_ptr[j]..col_ptr[j + 1] {
                let i = row_idx[pos];
                let v = values[pos];
                if v == 0.0 {
                    continue;
                }
                let ti_list = &self.t_rows[i];
                if ti_list.is_empty() {
                    continue;
                }
                for &(a, ta) in ti_list {
                    for &(b, tb) in tj_list {
                        out.push(Triplet {
                            row: a,
                            col: b,
                            val: ta * v * tb,
                        });
                    }
                }
            }
        }
    }

    /// [`Self::reduce_k`] のキャッシュ版。時刻歴応答解析の Newton 反復のように、
    /// 同一 `Reducer`（＝同一拘束構成）で毎回 `k_free` を縮約する場面向け
    /// （[`crate::common::csc_cache::CscCache`] 参照）。K の非ゼロパターンが不変なら
    /// 縮約後の triplet 列の座標・並び順も不変なため（`t_rows` はここでは変わらない
    /// 定数）、高速パスが有効に働く。結果は常に [`Self::reduce_k`] とビット一致する。
    pub fn reduce_k_cached(
        &self,
        k_free: &SparseColMat<usize, f64>,
        cache: &mut CscCache,
    ) -> SparseColMat<usize, f64> {
        let triplets = self.reduce_k_triplets(k_free);
        cache.assemble(self.n_indep, &triplets)
    }

    /// [`Self::reduce_k_cached`] の参照返し版。呼び出し側の triplet バッファ
    /// `buf`（`clear()` して再利用、容量は呼び出し間で維持される）を使い、
    /// `cache` が内部保持する行列への参照を返す（`.clone()` を伴わない）。
    /// 非線形時刻歴の Newton 反復のように、結果をすぐ読むだけで所有権が要らない
    /// 呼び出し元向け（[`crate::common::csc_cache::CscCache::assemble_ref`] 参照）。
    /// 結果は常に [`Self::reduce_k`] とビット一致する。
    pub fn reduce_k_cached_ref<'a>(
        &self,
        k_free: &SparseColMat<usize, f64>,
        cache: &'a mut CscCache,
        buf: &mut Vec<Triplet>,
    ) -> &'a SparseColMat<usize, f64> {
        self.reduce_k_triplets_into(k_free, buf);
        cache.assemble_ref(self.n_indep, buf)
    }

    /// [`Self::reduce_f`] の結果を呼び出し側の既存バッファへ書き込む版（毎ステップの
    /// Vec 確保を避ける）。`f_red` の長さは `self.n_indep` でなければならない。
    /// 計算順序は [`Self::reduce_f`] と同一（ビット完全一致）。
    pub fn reduce_f_into(&self, f_free: &[f64], f_red: &mut [f64]) {
        for v in f_red.iter_mut() {
            *v = 0.0;
        }
        for i in 0..self.n_free {
            if f_free[i] != 0.0 {
                for &(a, ta) in &self.t_rows[i] {
                    f_red[a] += ta * f_free[i];
                }
            }
        }
    }

    pub fn reduce_f(&self, f_free: &[f64]) -> Vec<f64> {
        let mut f_red = vec![0.0; self.n_indep];
        self.reduce_f_into(f_free, &mut f_red);
        f_red
    }

    /// [`Self::expand_u`] の結果を呼び出し側の既存バッファへ書き込む版（毎ステップの
    /// Vec 確保を避ける）。`u_free` の長さは `self.n_free` でなければならない。
    /// 計算順序は [`Self::expand_u`] と同一（ビット完全一致）。
    pub fn expand_u_into(&self, u_indep: &[f64], u_free: &mut [f64]) {
        for i in 0..self.n_free {
            u_free[i] = 0.0;
            for &(a, ta) in &self.t_rows[i] {
                u_free[i] += ta * u_indep[a];
            }
        }
    }

    pub fn expand_u(&self, u_indep: &[f64]) -> Vec<f64> {
        let mut u_free = vec![0.0; self.n_free];
        self.expand_u_into(u_indep, &mut u_free);
        u_free
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::MaterialCategory;
    use squid_n_core::model::{
        Constraint, ElementData, ElementKind, LocalAxis, Material, Model, Node, Section,
    };

    fn make_3node_model() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 1000.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [1000.0, 1000.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
            ],
            // 全節点に要素を接続する（要素が接続しない節点は DofMap が解析自由度
            // から除外するため、拘束縮約のテスト対象にならない）。
            elements: (0..2)
                .map(|i| ElementData {
                    id: ElemId(i),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(i), NodeId(i + 1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [
                        squid_n_core::model::EndCondition::Fixed,
                        squid_n_core::model::EndCondition::Fixed,
                    ],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                })
                .collect(),
            sections: vec![Section {
                id: SectionId(0),
                name: "sec".to_string(),
                area: 100.0,
                iy: 1000.0,
                iz: 1000.0,
                j: 100.0,
                depth: 100.0,
                width: 100.0,
                as_y: 83.33,
                as_z: 83.33,
                floor: None,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "mat".to_string(),
                category: MaterialCategory::Steel,
                young: 1000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_no_constraint_identity() {
        let model = make_3node_model();
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        assert_eq!(reducer.n_indep, reducer.n_free);
        for i in 0..reducer.n_free {
            assert_eq!(reducer.t_rows[i], vec![(i, 1.0)]);
        }
    }

    #[test]
    fn test_rigid_diaphragm() {
        let mut model = make_3node_model();
        model.constraints.push(Constraint::rigid_diaphragm(
            StoryId(0),
            NodeId(1),
            vec![NodeId(2)],
        ));
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        // slave Ux/Uy/Rz が master に従うため独立 DOF が減る
        assert!(reducer.n_indep < reducer.n_free);
    }

    #[test]
    fn test_rigid_link() {
        let mut model = make_3node_model();
        model.constraints.push(Constraint::RigidLink {
            master: NodeId(1),
            slaves: vec![NodeId(2)],
            dofs: Dof6Mask::FIXED,
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        // スレーブ 6 DOF がマスターに従う
        assert!(reducer.n_indep < reducer.n_free);
    }

    /// 代表節点（要素非接続・Uz/Rx/Ry 固定の浮遊節点）をマスターとした剛床で、
    /// スレーブの面内変位が ix = Gx − iry·Gθz, iy = Gy + irx·Gθz（剛床仮定に
    /// よる面内剛体変位の運動学）どおりに復元されることを確認する。
    #[test]
    fn test_rigid_diaphragm_master_recovers_translation_and_torsion() {
        let mut model = make_3node_model();
        let mut rep_restraint = Dof6Mask::FREE;
        rep_restraint.set_fixed(Dof::Uz);
        rep_restraint.set_fixed(Dof::Rx);
        rep_restraint.set_fixed(Dof::Ry);
        let master_coord = [500.0, 1000.0, 0.0];
        model.nodes.push(Node {
            id: NodeId(3),
            coord: master_coord,
            restraint: rep_restraint,
            mass: None,
            story: None,
            support_spring: None,
        });
        model.constraints.push(Constraint::rigid_diaphragm(
            StoryId(0),
            NodeId(3),
            vec![NodeId(1), NodeId(2)],
        ));
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        let g_master_ux = 3 * DOF_PER_NODE;
        let g_master_uy = 3 * DOF_PER_NODE + 1;
        let g_master_rz = 3 * DOF_PER_NODE + 5;
        let a_ux = dofmap.active(g_master_ux).unwrap() as usize;
        let a_uy = dofmap.active(g_master_uy).unwrap() as usize;
        let a_rz = dofmap.active(g_master_rz).unwrap() as usize;
        // マスター自身の DOF は独立(恒等写像の行)のはず。
        let idx_ux = reducer.t_rows[a_ux][0].0;
        let idx_uy = reducer.t_rows[a_uy][0].0;
        let idx_rz = reducer.t_rows[a_rz][0].0;

        let (gx, gy, gtheta) = (2.0, -1.5, 0.002);
        let mut u_indep = vec![0.0; reducer.n_indep];
        u_indep[idx_ux] = gx;
        u_indep[idx_uy] = gy;
        u_indep[idx_rz] = gtheta;

        let u_free = reducer.expand_u(&u_indep);

        for &slave in &[NodeId(1), NodeId(2)] {
            let si = slave.index();
            let dx = model.nodes[si].coord[0] - master_coord[0];
            let dy = model.nodes[si].coord[1] - master_coord[1];
            let expected_ux = gx - dy * gtheta;
            let expected_uy = gy + dx * gtheta;
            let a_slave_ux = dofmap.active(si * DOF_PER_NODE).unwrap() as usize;
            let a_slave_uy = dofmap.active(si * DOF_PER_NODE + 1).unwrap() as usize;
            assert!(
                (u_free[a_slave_ux] - expected_ux).abs() < 1e-9,
                "ix: got={} want={}",
                u_free[a_slave_ux],
                expected_ux
            );
            assert!(
                (u_free[a_slave_uy] - expected_uy).abs() < 1e-9,
                "iy: got={} want={}",
                u_free[a_slave_uy],
                expected_uy
            );
        }
    }

    /// マスター節点の対象 DOF が拘束済み（非 active）の剛リンクでは、スレーブ DOF も
    /// 0 に縮約される（空行）こと。従来はスレーブが独立自由度のまま残り、拘束が
    /// 無言で破れていた（RigidDiaphragm だけが正しく空行へ縮約していた不整合）。
    #[test]
    fn test_rigid_link_fixed_master_forces_slave_to_zero() {
        let mut model = make_3node_model();
        // node1 を完全拘束し、node2 を剛リンクで node1 へ従属させる。
        model.nodes[1].restraint = Dof6Mask::FIXED;
        model.constraints.push(Constraint::RigidLink {
            master: NodeId(1),
            slaves: vec![NodeId(2)],
            dofs: Dof6Mask::FIXED,
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        // node2 の全 DOF はマスター（=0）へ従属し、独立自由度を持たない（空行）。
        for d in 0..DOF_PER_NODE {
            let g = 2 * DOF_PER_NODE + d;
            if let Some(sa) = dofmap.active(g) {
                assert!(
                    reducer.t_rows[sa as usize].is_empty(),
                    "slave dof {} should reduce to zero (empty row), got {:?}",
                    d,
                    reducer.t_rows[sa as usize]
                );
            }
        }
        // 展開結果も常に 0。
        let u_free = reducer.expand_u(&vec![1.0; reducer.n_indep]);
        for d in 0..DOF_PER_NODE {
            let g = 2 * DOF_PER_NODE + d;
            if let Some(sa) = dofmap.active(g) {
                assert_eq!(u_free[sa as usize], 0.0, "slave dof {} must be zero", d);
            }
        }
    }

    /// マスター側の対象 DOF がすべて拘束済みの MPC でも、スレーブ DOF は 0 に
    /// 縮約される（従来はスレーブが独立自由度のまま残っていた）。
    #[test]
    fn test_mpc_fixed_master_forces_slave_to_zero() {
        let mut model = make_3node_model();
        // node0 は FIXED のまま。node2.Ux = 0.5 * node0.Ux（node0.Ux は拘束済み）。
        model.constraints.push(Constraint::Mpc {
            master: NodeId(2),
            terms: vec![(NodeId(0), squid_n_core::dof::Dof::Ux, 0.5)],
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let sa = dofmap.active(2 * DOF_PER_NODE).unwrap() as usize;
        assert!(
            reducer.t_rows[sa].is_empty(),
            "slave Ux should reduce to zero, got {:?}",
            reducer.t_rows[sa]
        );
    }

    /// 存在しない節点を参照する拘束（ダングリング NodeId）があっても panic せず、
    /// 当該拘束を読み飛ばして構築できること（ユーザー向け診断は precheck が担う）。
    #[test]
    fn test_dangling_constraint_reference_does_not_panic() {
        let mut model = make_3node_model();
        model.constraints.push(Constraint::rigid_diaphragm(
            StoryId(0),
            NodeId(99),
            vec![NodeId(1)],
        ));
        model.constraints.push(Constraint::RigidLink {
            master: NodeId(1),
            slaves: vec![NodeId(98)],
            dofs: Dof6Mask::FIXED,
        });
        model.constraints.push(Constraint::Mpc {
            master: NodeId(97),
            terms: vec![(NodeId(1), squid_n_core::dof::Dof::Ux, 1.0)],
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        // ダングリング拘束は無効化され、独立自由度は全自由度のまま。
        assert_eq!(reducer.n_indep, reducer.n_free);
    }

    #[test]
    fn test_mpc() {
        let mut model = make_3node_model();
        // スレーブ NodeId(2) の Ux = 0.5 * NodeId(1) の Ux
        model.constraints.push(Constraint::Mpc {
            master: NodeId(2),
            terms: vec![(NodeId(1), squid_n_core::dof::Dof::Ux, 0.5)],
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        assert!(reducer.n_indep < reducer.n_free);
    }

    /// 連鎖拘束（スレーブのマスターがさらに別拘束のスレーブ）の合成を検証する。
    /// node1.Ux = node0.Ux（MPC）、node2.Ux = node1.Ux（MPC）の連鎖では、
    /// node2.Ux は推移的に node0.Ux に一致しなければならない。従来は node2 が
    /// node1 の従属自由度へ偽の独立番号で結び付き、連鎖が無言で破れていた。
    #[test]
    fn test_chained_mpc_constraints_compose_transitively() {
        let mut model = make_3node_model();
        model.nodes[0].restraint = Dof6Mask::FREE; // 3 節点とも自由に
                                                   // node1.Ux = node0.Ux
        model.constraints.push(Constraint::Mpc {
            master: NodeId(1),
            terms: vec![(NodeId(0), squid_n_core::dof::Dof::Ux, 1.0)],
        });
        // node2.Ux = node1.Ux（node1 は上の MPC のスレーブ＝連鎖）
        model.constraints.push(Constraint::Mpc {
            master: NodeId(2),
            terms: vec![(NodeId(1), squid_n_core::dof::Dof::Ux, 1.0)],
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        // 各ノードの Ux グローバル DOF = node_index * DOF_PER_NODE。
        let a0 = dofmap.active(0).unwrap() as usize; // node0 Ux
        let a1 = dofmap.active(DOF_PER_NODE).unwrap() as usize; // node1 Ux
        let a2 = dofmap.active(2 * DOF_PER_NODE).unwrap() as usize; // node2 Ux

        // node0.Ux は独立（行は自身への単位行）。その独立番号にだけ単位値を与える。
        assert_eq!(reducer.t_rows[a0].len(), 1);
        let idx0 = reducer.t_rows[a0][0].0;
        let mut u_indep = vec![0.0; reducer.n_indep];
        u_indep[idx0] = 1.0;
        let u_free = reducer.expand_u(&u_indep);

        assert!((u_free[a0] - 1.0).abs() < 1e-12, "node0.Ux={}", u_free[a0]);
        assert!(
            (u_free[a1] - 1.0).abs() < 1e-12,
            "node1.Ux={} should follow node0",
            u_free[a1]
        );
        assert!(
            (u_free[a2] - 1.0).abs() < 1e-12,
            "node2.Ux={} should follow node0 transitively",
            u_free[a2]
        );
        // 連鎖で独立 DOF が 2 個（node1.Ux, node2.Ux）減る。
        assert_eq!(reducer.n_indep, reducer.n_free - 2);
    }
}
