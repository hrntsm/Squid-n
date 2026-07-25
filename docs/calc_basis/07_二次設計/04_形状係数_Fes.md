# 7.4 形状係数 Fes（剛性率 Fs・偏心率 Fe）

## 7.4.1 剛性率 Rs・Fs

建築基準法施行令 第82条の6 と告示第1792号に基づき、剛性率 Rs（規定は \\( R_s \ge 0.6 \\)）から Fs を算定します。

**算定式**

\\[ K_s = h/\delta \\]

\\[ R_s = K_s/\mathrm{mean}(K_s) \\]

\\[ F_s = \begin{cases} 1.0 & (R_s \ge 0.6) \\\\ 2.0 - R_s/0.6 & (R_s < 0.6) \end{cases} \\]

**実装**：`holding_capacity::{stiffness_ratios, fs}` が算定します。剛性率用の層間変位には重心変位 δg
（`secondary::stiffness_ratio::cog_story_drifts`）を用います。

## 7.4.2 偏心率 Re・Fe

建築基準法施行令 第82条の6 と告示第1792号に基づき、偏心率 Re（規定は \\( R_e \le 0.15 \\)）から Fe を算定します。
剛心と弾力半径は武藤 D 値法で求めます。

**算定式**

D 値（一般階）:

\\[ D = a \cdot K_{c0} \\]

\\[ K_{c0} = 12EI_c/h^3 \\]

\\[ a = \bar{k}/(2+\bar{k}) \\]

剛心と偏心距離:

\\[ X_s = \sum(D_y \cdot x)/\sum D_y \\]

\\[ e_x = |X_g - X_s| \\]

弾力半径:

\\[ r_{ex} = \sqrt{K_R/\sum D_x} \\]

\\[ K_R = \sum(D_x \cdot \bar{y}^2) + \sum(D_y \cdot \bar{x}^2) \\]

偏心率と Fe:

\\[ R_e = e/r_e \\]

\\[ F_e = \begin{cases} 1.0 & (R_e \le 0.15) \\\\ \min(1.0 + 0.5(R_e - 0.15)/0.15, 1.5) & (R_e > 0.15) \end{cases} \\]

**実装**：`secondary::eccentricity`（D 値法略算）と `secondary::eccentricity_analysis`（応力解析結果からの精算層。剛心は解析剛性、重心は長期軸力による）、および `holding_capacity::fe` が算定します。

## 7.4.3 形状係数 Fes

形状係数 Fes は、剛性率による Fs と偏心率による Fe の積として算定します。

**算定式**

\\[ F_{es} = F_s \cdot F_e \\]

**実装**：`holding_capacity::fes` が算定します。
