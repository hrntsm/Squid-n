# Beam と Fiber の整合質量は共通の断面質量特性から算定する

Status: accepted

## 決定

RC/SRC のコンクリート材料は `Material.fc` から純コンクリート密度を求め、Fc 未設定は入力エラーとする。標準材料の `density` は鉄筋込み γRC であり、鉄筋領域の加算へ流用しない。

Beam と Fiber の整合質量は、断面の材料領域から求めた `SectionMassProperties` を共通の入力とし、`squid_n_element::frame::prismatic::consistent_mass_timoshenko` で算定する。質量特性は単位長さ当たり質量と断面 y 軸・z 軸まわりの質量二次モーメントを持つ。RC/SRC のコンクリート材料は `Material.fc` から γC（純コンクリート）を単位体積重量表で導き、Fc 未設定は入力エラーとする。標準材料の密度は鉄筋込みの γRC であるため、鉄筋領域の加算へ流用しない。その他の材料は `Material.density` を使い、CFT の充填コンクリートも同じ γC を使う。

材軸まわりの回転慣性には、ねじり剛性 `GJ/L` のねじり定数 `J` ではなく、質量用極二次モーメント `Ip = Iy + Iz` を用いる。剛域を含む場合は、可撓部の質量を剛体アームで変換し、剛域の分布質量を加えて部材全長の質量を保存する。端部解放がある場合も、剛性側と同じ拡大自由度で質量を組み、初期弾性剛性から固定した `S=-Kbb^-1 Kba`, `R=[I;S]` により `M=Rᵀ Mbar R` として縮約する。回転ばねは無質量とする。

## 背景と理由

従来は Beam が密度・幾何断面積とねじり定数から質量を作り、Fiber はファイバー面積の総和から質量を作っていたため、同じ断面でも整合質量の入力と回転自由度が一致しなかった。材料領域を質量の正本とすることで、RC・SRC・CFT の材料置換と断面回転慣性を要素種別から分離する。

ねじり定数はせん断応力に対するねじり剛性の量であり、質量回転慣性の量ではない。開断面では両者の差が大きくなりうるため、質量積分から得られる極二次モーメントを使う。

## 影響

- `MassOption::Lumped` は Beam が `density × a_mass × L`、Fiber が `density × ΣAf × L` を使う従来契約を維持する。
- `MassOption::Consistent` を使う固有値解析・時刻歴解析では、Beam と Fiber の質量行列および材軸回転慣性が変わる。
- 端部解放の質量縮約は Beam/Fiber で共通化し、Fiber は塑性状態の接線剛性を使わない。
- `Kbb` が特異な端部解放では、解放なし質量へフォールバックせず、Beam/Fiber とも明示的に失敗する。
- 形状を持たない断面は、主材料の密度と `Section` の断面諸元へフォールバックする。
- 材料領域質量、標準 Fc/SD 材料、Beam/Fiber 一致、剛域質量保存、軸振動および純ねじり固有値をテストで検証する。

## 関連

- `crates/squid-n-core/src/model/mass.rs`
- `crates/squid-n-element/src/frame/prismatic.rs`
- `crates/squid-n-element/src/frame/beam/behavior.rs`
- `crates/squid-n-element/src/frame/fiber/mod.rs`
- `docs/calc_basis/04_要素剛性/01_ティモシェンコ梁要素.md`
- `docs/calc_basis/04_要素剛性/09_非線形梁要素のモデル化.md`
