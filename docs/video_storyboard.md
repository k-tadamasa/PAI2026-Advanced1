# 動画構成 (WebGL 1080p版)

タイトルは `PAI最終課題_TADAMASA_NDA` とする．公開設定は最低でも限定公開とし，子ども向け・年齢制限・インスタントプレミア公開は設定しない．

生成物は `output/video/PAI最終課題_TADAMASA_NDA.mp4` である．Human3D + SegFitのローカル推論結果とKitaharaらの体表面追従4列往復軌道（走査線間隔1.5倍）をWebGL viewerで再生したkinematic visualizationであり，Isaac Sim物理実行映像とは表記しない．

## 0:00-0:12 課題とAI

- title card: AI Body Screening Demo
- 警備員が接近せずに人体表面をrobotで検査する目的を1文で表示
- Human3D → SegFit → Front ROI → 4列往復軌道 → TORM IKのpipelineを常時表示
- AI推定meshと対象ROIを重ね，`LOCAL AI RESULT` と表示

## 0:12-0:25 検査対象

- 全身meshから法線75°未満・中央幅80%・上端100 mm除外後の胴体，骨盤，左右大腿をhighlight
- `Human3D part IDs: 2, 7, 14, 15` を表示
- 頭，腕，下腿などが対象外であることを示す

## 0:25-0:55 4列往復軌道とTORM robot preview

- XZ局所直線近似、55 mm距離補正、LOESS平滑化を表示
- `4 lanes / center gap 1.2x baseline / LOESS 50 samples` と表示
- 被覆率96.20%，経路長3.33 m，TORM姿勢897から適応細分化した3,845制御姿勢を使用
- `ffw_sg2_mecanum`が理想Mecanum移動で正面trajectoryを追従
- SMPL-Xを薄い灰色，MD実体を灰色，探知楕円体を境界の薄い透明青，検査済み人体面を緑で表示
- trajectoryを2.2 pxの太線で表示し，未通過はシアン，通過後は白へ切り替える。一度でも実形状交差した人体faceと右グリッパリンクは以後も赤で保持（MDと探知楕円体は変色させない）

## 0:55-1:19 結果と今後の課題

- `Coverage 96.20%`
- `Collision avoidance is not implemented`
- 赤い人体faceと右グリッパlinkはMD collision meshまたは右グリッパ実meshとの交差履歴を示し，回避は今後の課題と説明
- GitHub repository URLを表示

## 概要欄案

```text
Physical AI応用編1 最終課題
Account: TADAMASA_NDA / ID: 46962

Human3DとSegFitによるAI推論結果から，胴体・骨盤・左右大腿のうち，面法線のXY投影と-X軸の角度が75°未満となる正面を抽出しました．さらに左右を抽出幅の中央80%，上端を最高面重心から100 mm下へ限定し，Kitaharaらの体表面追従手法を走査線間隔1.5倍の4列往復へ調整しました．FFW-SG2上半身とRidgeback Mecanum台車の11自由度軌道はTORM path-wise IKで平滑化しています．台車移動はスリップ・加速度制限を省略した理想運動です．衝突回避は未実装であり，接触可能区間を赤色で表示して今後の課題としています．

Repository: https://github.com/k-tadamasa/PAI2026-Advanced1
```
