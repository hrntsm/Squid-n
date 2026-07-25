# 8.5 SRC / SRRC 梁 せん断終局（充腹・非充腹）

SRC/SRRC 梁のせん断終局耐力を、充腹と非充腹に分けて算定します。
技術基準解説書（SRC 梁せん断終局）と SRC規準（SRC 梁せん断終局）に基づき、非充腹は荒川 mean 式系によります。

**算定式**:

- 充腹（累加）\\( Q_u = {}_r Q_u + {}_s Q_u \\)。`rQu`（コンクリート・RC）に `sQu`（鉄骨ウェブ \\( {}_s A_w \cdot {}_s \sigma_y/\sqrt{3} \\)）を累加する。
  RC 部の許容せん断応力度 fs は工学単位（kgf/cm²）で定義された式を SI に換算して用いる:
  技術基準解説書式 \\( f_s = \min(F_c/20, (0.49 + F_c/100) \cdot 1.5) \\)、
  SRC規準式 \\( f_s = \min(0.15 \cdot F_c, 2.21 + 0.045 \cdot F_c) \\)（いずれも N/mm²）
- 非充腹 格子材 \\( Q_{su} = \\{ \kappa \cdot p_t^{0.23} \cdot k_{cs} \cdot (18+F_c)/(M/Qd+0.12) + 0.85\sqrt{{}\_r p_w \cdot {}\_r \sigma_{wy} + 0.5 \cdot {}\_s p_w \cdot {}\_s \sigma_{wy}} \\} \cdot b_e \cdot j \\)
  （√ は帯板項まで全体に掛かる。\\( p_t = {}_r p_t + {}_s p_t \\)、\\( j = 0.8D \\)、\\( M/Qd \in [1,3] \\)、\\( \kappa = 0.053 \\)／高強度0.068）
- 非充腹 ラチス材 \\( Q_{su} = \\{\dots\\} \cdot b_e \cdot {}_r j + {}_s Q_u \\)

**実装**：`srrc::beam_nonlinear`（`beam_nonlinear.rs`）。
