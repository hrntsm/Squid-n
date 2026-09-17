# MITC4 シェル要素

せん断ロッキングを回避する混合補間法 MITC4（Mixed Interpolation of Tensorial Components）を用い、膜・曲げ・せん断・ドリリング安定化を含む板要素の剛性を算定します。

**算定式**

膜、曲げ、せん断（せん断補正係数 5/6）:

\\[ D\_m = \frac{E \cdot t}{1-\nu^2} \cdot [\cdots] \\]

\\[ D\_b = \frac{E \cdot t^3}{12(1-\nu^2)} \cdot [\cdots] \\]

\\[ D\_s = (5/6) \cdot G \cdot t \cdot I \\]
- MITC4 せん断補間: タイング点 A(0,+1)/B(−1,0)/C(0,−1)/D(+1,0) の共変ひずみを補間し、逆ヤコビアンで
  直交座標へ射影
- 剛性は 2×2 Gauss 積分 \\( B^T \cdot D \cdot B \\)、ドリリング安定化 \\( \text{scale} = \gamma \cdot G \cdot t \cdot A \\)（既定 \\( \gamma = 10^{-3} \\)）

**記号・単位**

| 記号 | 意味 | 単位 |
|------|------|------|
| \\( D\_m \\) | 膜の構成則マトリクス | N/mm |
| \\( D\_b \\) | 曲げの構成則マトリクス | N·mm |
| \\( D\_s \\) | せん断の構成則マトリクス | N/mm |
| \\( \text{scale} \\) | ドリリング安定化の剛性 | N·mm |
| \\( E \\) | ヤング係数 | N/mm² |
| \\( G \\) | せん断弾性係数 | N/mm² |
| \\( \nu \\) | ポアソン比 | - |
| \\( t \\) | シェル要素の板厚 | mm |
| \\( \gamma \\) | ドリリング安定化の係数（既定 \\( 10^{-3} \\)） | - |

<div class="impl-ref">

**実装参照**：`squid_n_element::shell::ShellElement::{local_stiffness, add_drilling}`（`crates/squid-n-element/src/shell/stiffness.rs`）と `squid_n_element::shell::ShellElement::shear_b_mitc4`（`crates/squid-n-element/src/shell/bmatrix.rs`）が算定します。
剛床時は面内成分（Ux/Uy/Rz）を無効化します。

</div>
