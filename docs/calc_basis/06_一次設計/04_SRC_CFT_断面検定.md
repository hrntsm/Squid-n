# 6.4 SRC / CFT 断面検定

SRC・CFT の断面検定を、SRC規準1987（累加強度式・併用せん断式）および構造規定（技術基準解説書）に基づいて行います。

**算定式**

SRC 梁 曲げ（鉄骨と RC の累加）:

\\[ M_A = {}\_s M_o + {}\_r M_A \\]

鉄骨部:

\\[ {}\_s M_o = {}\_s Z \cdot {}\_s f_t \\]

RC 部:

\\[ {}\_r M_A = a_t \cdot f_t \cdot j \\]

SRC 梁 せん断は、鉄骨部と RC 部を弾性分担（または地震時短期の構造規定方式 sQD/rQD）と比較します。

鉄骨部:

\\[ {}\_s Q_A = t_w \cdot d_w \cdot {}\_s f_s \\]

RC 部（pw は 0.6% 上限）:

\\[ {}\_r Q_A = \min(b \cdot {}\_r j \cdot ({}\_r \alpha \cdot f_s + 0.5 \cdot p_w \cdot {}\_w f_t), b \cdot {}\_r j \cdot (2(b'/b) \cdot f_s + p_w \cdot {}\_w f_t)) \\]

- SRC 柱 N-M 相関を累加強度式で構築する（コンクリート許容応力度は鉄骨フランジ食い込み低減後 \\( f_c' = f_c(1 - 15 \cdot {}\_s p_c) \\)）、二軸曲げは線形和。RC 部 N-M 曲線は引張側アンカー \\( ({}\_r N_t, 0) \\) を持ち、純引張耐力近傍で \\( {}\_r M \to 0 \\) に収束する。
SRC 柱 せん断の長期は、併用式を全せん断力と比較します。

\\[ Q_A = (1+\beta) \cdot b \cdot {}\_r j \cdot a' \cdot f_s \\]

ここで \\( a' = {}\_r \alpha \\)（\\( b'/b \ge {}\_r \alpha/3 \\)）または \\( 3b'/b \\)、\\( 1 \le {}\_r \alpha \le 2 \\)、β は鉄骨ウェブの形式と寸法による係数とします。

短期は鉄骨部 `sQA`（強軸 \\( d_w \cdot t_w \cdot {}\_s f_s \\)／弱軸 \\( (4/3) \cdot b_f \cdot t_f \cdot {}\_s f_s \\)）と RC 部を分担 sQD/rQD と比較します。

\\[ {}\_r Q_{AS1} = b \cdot {}\_r j \cdot (f_s + 0.5 \cdot p_w \cdot {}\_w f_t) \\]

（α を含まない）

\\[ {}\_r Q_{AS2} = b \cdot {}\_r j \cdot (2(b'/b) \cdot f_s + p_w \cdot {}\_w f_t) \\]
- CFT 柱は SRC 規準を CFT 断面に適用する（相互拘束効果の強度割増しは考慮せず、非拘束・安全側とする）。せん断有効断面積は方向別（\\( 2t(H-2t) \\)／\\( 2t(B-2t) \\)）とする。
CFT 柱の地震時設計用せん断:

\\[ Q_D = \min(Q_{D1}, Q_{D2}) \\]

\\[ Q_{D1} = \sum {}\_c M_y/h' \\]

ここで cMy には CFT 指針（コンクリート充填鋼管構造設計指針）の N-M 相互作用による終局曲げ耐力 Mu(N)（柱分類対応）を用い、柱頭・柱脚同一断面の仮定で \\( \sum {}\_c M_y = 2 \cdot M_u(N) \\) とします。

\\[ Q_{D2} = Q_L + n \cdot Q_E \\]

**実装**：`srrc::{beam, column, panel_zone}`、`cft::mod` が検定します。
SRC パネルゾーンは \\( {}\_c V \cdot j\delta \cdot f_s \cdot (1+\beta) \ge (h'/h)({}\_B M_1 + {}\_B M_2) \\)。
