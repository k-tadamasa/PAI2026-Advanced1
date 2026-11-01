# 動画構成 (生成済み62.8秒)

タイトルは `PAI最終課題_TADAMASA_NDA` とする．公開設定は最低でも限定公開とし，子ども向け・年齢制限・インスタントプレミア公開は設定しない．

生成物は `output/video/PAI最終課題_TADAMASA_NDA.mp4` である．公開JSONを使うkinematic previewであり，Isaac Simの実行映像とは表記しない．

## 0:00-0:12 課題とAI

- title card: AI Body Screening Demo
- 警備員が接近せずに人体表面をrobotで検査する目的を1文で表示
- Human3D → SegFit → Front ROI → IFToMMのpipelineを常時表示
- AI推定meshと対象ROIを重ねて表示

## 0:12-0:25 検査対象

- 全身meshから正面の胴体，骨盤，左右大腿だけをhighlight
- `Human3D part IDs: 2, 7, 14, 15` を表示
- 頭，腕，下腿などが対象外であることを示す

## 0:25-0:45 IFToMM軌道とrobot preview

- 制御点生成とspline補間を表示
- `Coverage-aware local candidate search` と表示
- 被覆率99.03%，経路長6.41 m，姿勢628を表示
- 簡易2-link robot armが正面trajectoryを追従
- safe trajectoryを緑，collision proxyを赤で表示

## 0:45-1:02 結果と今後の課題

- `Coverage 99.03%`
- `Collision avoidance is not implemented`
- 赤区間は検出器shaft等の接触可能性を示し，今後の課題と説明
- GitHub repository URLを表示

## 概要欄案

```text
Physical AI応用編1 最終課題
Account: TADAMASA_NDA / ID: 46962

Human3DとSegFitによるAI推論結果から，胴体・骨盤・左右大腿の正面を抽出し，公開済みIFToMM手法で検査軌道を生成しました．動画は公開JSONを用いたkinematic previewです．衝突回避は未実装であり，接触可能区間を赤色で表示して今後の課題としています．Isaac Sim用のLula/Franka再生scriptはrepositoryに収録しています．

Repository: https://github.com/k-tadamasa/PAI2026-Advanced1
```
