# 3.6 SRC / CFT の等価断面性能

SRC/CFT の等価断面性能は、ヤング係数比による等価換算断面の累加（SRC 規準の考え方）で算定します。
SRC は次により求めます。

\\[ A\_n = {}\_{rc}A\_n + {}\_s A\_n \cdot (n\_s - 1) \\]

\\[ I\_e = {}\_{rc}I\_e + {}\_s I\_e \cdot (n\_s - 1) \\]

\\[ A\_s = {}\_{rc}A\_s + {}\_s A\_s \cdot (n\_{gs} - 1) \\]

\\[ J = {}\_c J + ({}\_s G/{}\_c G) \cdot {}\_s J \\]
CFT は SRC 柱に準じ、鋼基準の 1/n 換算で累加します。

**算定式**

換算係数（\\( \nu_s = 0.3 \\)、\\( \nu_c = 0.2 \\)）:

\\[ n_s = E_{\text{steel}}/E_c \\]

\\[ n_{gs} = n_s \cdot \frac{1 + \nu_c}{1 + \nu_s} \\]

SRC 軸剛性用:

\\[ A = b \cdot d + (n_s - 1) \cdot {}_s a \\]

SRC 曲げ:

\\[ I_y = \frac{b \cdot d^3}{12} + (n_s - 1) \cdot {}_s i_y \\]

CFT（充填コンクリートを鋼基準へ換算）:

\\[ A = A\_{\text{steel}} + {}\_c a/n \\]

\\[ I\_y = I\_{y,\text{steel}} + {}\_c i\_y/n \\]

**実装**：`section_shape::{src_equivalent_props, cft_equivalent_props}` が算定する。
ns の暫定既定は N_S_EQ = 15 とする。
