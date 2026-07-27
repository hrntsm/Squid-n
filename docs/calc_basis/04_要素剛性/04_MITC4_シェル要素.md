# 4.4 MITC4 シェル要素

せん断ロッキングを回避する混合補間法 MITC4（Mixed Interpolation of Tensorial Components）を用い、膜・曲げ・せん断・ドリリング安定化を含む板要素の剛性を算定します。

**算定式**

膜、曲げ、せん断（せん断補正係数 5/6）:

\\[ D_m = \frac{E \cdot t}{1-\nu^2} \cdot [\cdots] \\]

\\[ D_b = \frac{E \cdot t^3}{12(1-\nu^2)} \cdot [\cdots] \\]

\\[ D_s = (5/6) \cdot G \cdot t \cdot I \\]
- MITC4 せん断補間: タイング点 A(0,+1)/B(−1,0)/C(0,−1)/D(+1,0) の共変ひずみを補間し、逆ヤコビアンで
  直交座標へ射影
- 剛性は 2×2 Gauss 積分 \\( B^T \cdot D \cdot B \\)、ドリリング安定化 \\( \text{scale} = \gamma \cdot G \cdot t \cdot A \\)（既定 \\( \gamma = 10^{-3} \\)）

**実装**：`shell::{local_stiffness, shear_b_mitc4, add_drilling}`（`shell/mod.rs`）が算定します。
剛床時は面内成分（Ux/Uy/Rz）を無効化します。
