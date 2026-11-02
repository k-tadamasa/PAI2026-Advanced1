# PAI2026 Advanced1 公開版実装構成

## 目的と公開境界

AI推定人体面のうち胴体・骨盤・左右大腿を対象とし、面法線のXY投影と`-X`軸の角度が75°未満となる正面を抽出する。抽出幅の中央80%へ左右を限定し、最高面重心から上端を100 mm下げた領域だけを、Kitaharaらの体表面追従4列往復軌道（6列基準比で間隔1.5倍）で検査する教育用デモとする。未公開の衝突回避、motion compensation、surface patch最適化などは含めない。

- 公開する: 論文記載の往復軌道を1.5倍間隔へ調整した4列軌道、TORM公開手法、匿名mesh、Web viewer、FFW-SG2 / Ridgeback公開資産、MD STL
- 公開しない: AI model、checkpoint、SMPL-X model data、実測AI mesh、private repository履歴・runtime data
- 保護対象: local/GitHubの`body_screening`は変更しない

## 対象領域

| Human3D ID | 部位 |
|---:|---|
| 2 | 右大腿 |
| 7 | 胴体 |
| 14 | 左大腿 |
| 15 | 骨盤 |

対象集合は`frozenset({2, 7, 14, 15})`とする。各face法線をXY平面へ投影して正規化し、`-X`軸との角度が75°未満となるfaceを抽出する。XY投影長が0のfaceと75°境界上のfaceは除外する。その面重心Y範囲の中央80%を左右境界とし、面重心Z最大値から100 mm下を上境界とする。ROIが空の場合は停止し、全身へfallbackしない。

## データフロー

```text
Human3D + SegFit result
  -> MeshData(vertices, faces, part IDs)
  -> front ROI
  -> Kitahara four-lane surface raster (1.5x spacing)
  -> post-plan MD / right-gripper triangle-mesh collision
  -> frame coverage events
  -> ideal Mecanum states
  -> pai2026_web_scene_v5 JSON
  -> React / Three.js viewer
  -> deterministic 1080p video frames
```

AI推論結果は表示だけではなく、対象face、表面位置、法線、MD姿勢、被覆判定へ直接使用する。

## Kitahara体表面追従軌道

- standoff: 55 mm
- detection ellipsoid radii: 110 / 111 / 90.6 mm
- scan columns: 選択ROI内の4列、上下交互、6列基準比で間隔1.5倍
- point strip: `y_ref ± 2 mm`相当のメッシュ断面
- local fit: 上下5点ずつを用いたXZ平面の直線近似
- gap interpolation: 2 mm
- distance correction: 3次元最近傍点から55 mm
- smoothing: 位置・姿勢を50点LOESS
- detector frame: +Xを人体方向、+Zを鉛直走査方向
- output interpolation: 1 cm、姿勢最大1.5°/frame
- target coverage criterion: 95%
- lane placement: ROI幅の12.5 / 35 / 65 / 87.5%（最外線を維持し、中央間隔を従来比1.2倍）

衝突は軌道生成後に判定する。`MD_no_hole_poly.stl`とURDF FKで配置した右グリッパ5リンクのSTLを人体meshと三角形交差判定する。一度でも接触した人体faceと右グリッパlinkは以後も赤く保持し、MDと探知楕円体は衝突時も基準色を維持する。判定結果は選択scoreを変更しない。

## Robot visualization

`ffw_sg2_mecanum`は次の公開資産から独立構成する。

- ROBOTIS AI Worker FFW-SG2 upper body: Apache-2.0
- Clearpath Ridgeback base: BSD-3-Clause
- MD visual/collision STL: submission-author asset

台車は標準四輪Mecanum逆運動学に基づく理想holonomic運動とする。スリップ、ローラ接触、トルク限界は再現しない。左腕は`body_screening`でGUI確認済みの保存姿勢を維持する。SciPy逐次IKでseed列を生成後、現行`body_screening`のTORM二段階path-wise IKを用い、台車X/Y/ヨー、リフト、右腕7関節の11自由度を全軌道で平滑化する。最終段ではMD位置をhard taskとして全XYZへ厳密投影し、姿勢は位置Jacobianのnull space内で再最適化する。課題方針に従い衝突勾配は無効化し、衝突は後段表示だけとする。IFToMM detector frameはlocal +X=人体方向、+Y=MD長軸、+Z=走査方向とする。

## Web表示

- 検査済み人体面: green
- SMPL-X基準面: light grey
- MD: grey（衝突時も不変）
- detection ellipsoid: transparent blue with a very faint boundary（衝突時も不変）
- trajectory: 2.2 px cyan before traversal / white after traversal
- once-collided right-gripper links and human faces: persistent red

再生、一時停止、シーク、速度、camera、layer表示、被覆率、経路長、姿勢番号、台車位置、衝突数を操作・確認できる。

## 動画

Web viewerを1920×1080で実時間録画し、FFmpegでH.264 / CRF 18 / 30 fps / yuv420pへ変換する。録画前に描画済みURDFのMD制御点と目標軌道の位置・姿勢誤差を自動検査する。動画はkinematic visualizationであり、Isaac Sim物理実行とは表記しない。

## 合格条件

- 対象faceがID 2 / 7 / 14 / 15、法線75°未満、中央幅80%、上端100 mm除外
- 検査済みfaceがtrajectory再生に合わせて緑へ累積変化
- MDと探知楕円体が現在姿勢へ追従
- `ffw_sg2_mecanum`台車がX/Yへ理想移動
- 衝突を赤で表示し、軌道は変更しない
- Python test、TypeScript typecheck、production buildが成功
- Web console errorがない
- 公開assetのライセンスとprivate data境界を文書化
