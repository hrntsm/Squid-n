# SRC / SRRC 梁 せん断終局（充腹・非充腹）

SRC/SRRC 梁のせん断終局耐力は、技術基準解説書（SRC 梁せん断終局）と SRC規準（SRC 梁せん断終局）に基づき、充腹と非充腹に分けて算定し、非充腹は荒川 mean 式系によります。

**算定式**:

- 充腹（累加）\\( Q\_u = {}\_r Q\_u + {}\_s Q\_u \\)。`rQu`（コンクリート・RC）に `sQu`（鉄骨ウェブ \\( {}\_s A\_w \cdot {}\_s \sigma\_y/\sqrt{3} \\)）を累加する。
  RC 部の許容せん断応力度 fs は工学単位（kgf/cm²）で定義された式を SI に換算して用いる:
  技術基準解説書式 \\( f\_s = \min(F\_c/20, (0.49 + F\_c/100) \cdot 1.5) \\)、
  SRC規準式 \\( f\_s = \min(0.15 \cdot F\_c, 2.21 + 0.045 \cdot F\_c) \\)（いずれも N/mm²）
- 非充腹 格子材 \\( Q\_{su} = \\{ \kappa \cdot p\_t^{0.23} \cdot k\_{cs} \cdot (18+F\_c)/(M/Qd+0.12) + 0.85\sqrt{{}\_r p\_w \cdot {}\_r \sigma\_{wy} + 0.5 \cdot {}\_s p\_w \cdot {}\_s \sigma\_{wy}} \\} \cdot b\_e \cdot j \\)
  （√ は帯板項まで全体に掛かる。\\( p\_t = {}\_r p\_t + {}\_s p\_t \\)、\\( j = 0.8D \\)、\\( M/Qd \in [1,3] \\)、\\( \kappa = 0.053 \\)／高強度0.068）
- 非充腹 ラチス材 \\( Q\_{su} = \\{\dots\\} \cdot b\_e \cdot {}\_r j + {}\_s Q\_u \\)

<div class="impl-ref">

**実装参照**：`squid_n_design_jp::srrc::beam_nonlinear`（`beam_nonlinear.rs`）。

</div>
