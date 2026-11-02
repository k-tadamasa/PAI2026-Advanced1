# PAI2026 Advanced1: AI Body Screening Viewer

Physical AI応用編1の最終課題として、AIで推定した人体表面の限定領域を`ffw_sg2_mecanum`で走査する公開デモです。Human3DとSegFitの推論結果から胴体・骨盤・左右大腿を選び、各面法線をXY平面へ投影した方向と`-X`軸の角度が75°未満の正面を抽出します。さらに抽出幅の中央80%へ左右を限定し、上端を最高面重心から100 mm下げてから、Kitaharaらの体表面追従往復軌道を走査線間隔1.5倍（6列基準から4列）で生成します。

- 提出アカウント: `TADAMASA_NDA`
- ID: `46962`
- Web viewer: https://k-tadamasa.github.io/PAI2026-Advanced1/

氏名、所属、メールアドレスなどの個人情報は収録していません。

## デモの要点

- AI入力: Human3Dによる人体・部位推論とSegFitによるSMPL-X fitting
- 検査対象: Human3D部位ID `{2, 7, 14, 15}`
- 対象面: 法線角度75°未満、抽出幅の中央80%、上端から100 mm除外
- 軌道: Kitaharaらの体表面追従4列往復軌道（最外線を維持し、中央2本の間隔を従来比1.2倍へ拡大）
- ロボット: FFW-SG2上半身 + Clearpath Ridgeback Mecanum台車
- 台車運動: スリップ・加速度制限を省略した理想holonomic Mecanum運動。走査時間を基準の1/2（走査速度2倍）とし、TORM姿勢間をMD位置2 mm・姿勢0.5°・各関節0.5°・リフト2 mm以下となるまで適応分割
- MD: `MD.stl`実体、半径`110 / 111 / 90.6 mm`の探知楕円体
- 可視化: SMPL-Xを薄い灰色、検査済みfaceを緑、MDを灰色、探知領域を淡い透明青で表示。軌道は2.2 pxの太線で未通過をシアン、通過済みを白へ切り替える。一度でも交差した人体faceと右グリッパ実形状linkは以後も赤で保持し、MDと探知楕円体は衝突時も変色させない
- 制約: 衝突は計画から除外せず、今後の課題として明示

実測済みのHuman3D + SegFit結果では次の結果を得ました。実測meshはGitHubへ収録せず、ローカル動画生成だけに使用します。

| 指標 | 結果 |
|---|---:|
| AI入力点数 | 2,624 |
| fitting mesh | 10,475頂点 / 20,908面 |
| 正面ROI面積 | 0.3055 m² |
| 被覆率 | 96.20% |
| 経路長 | 3.33 m |
| 2倍速走査の検査時間 | 44.8 s |
| 制御点数 | 1,910 |
| TORM姿勢 / 適応細分化後の制御姿勢 | 897 / 3,845 |
| MD・右グリッパ実形状衝突姿勢 | 2 / 897 |

## Webアプリ

公開WebアプリはVite、React、TypeScript、Three.jsで構築しています。GitHub Pages版は匿名の合成人体面を読み込み、次をブラウザで操作できます。

- 再生、一時停止、シーク、0.5 / 1 / 2倍速
- 全景、正面、MD追従カメラ
- ロボット、MD、探知領域、検査済み面、軌道、実形状衝突の表示切替
- 被覆率、経路長、姿勢番号、台車位置、衝突姿勢数
- SciPy `least_squares`で各点seedを生成し、TORMで理想Mecanum X/Y/ヨー・リフト・FFW-SG2右腕7関節の11自由度軌道を全点最適化
- MD位置はhard taskとして全XYZを厳密投影し、姿勢は位置null-space内で可能な限り追従
- `body_screening`の保存姿勢と同じ左腕、左右グリッパー、右手首7番リンクに同じ固定変換で装着したMD（MDの世界姿勢は体表面追従軌道に追従）

```bash
cd web
npm install
npm run dev
```

production build:

```bash
cd web
npm ci
npm run typecheck
npm run build
```

`main`へのpushで`.github/workflows/pages.yml`がGitHub Pagesを更新します。

## 軌道生成

選択済みROIの横幅を`W`として、最外2本を従来どおり左右端から`W/8`に固定し、内側2本だけ外へ移動します。4本の位置は幅の`12.5%、35%、65%、87.5%`で、中央間隔は従来の`W/4`から`0.30W`（1.2倍）へ広げます。第2列から第3列へ移る中央部では、MD中心を保ったまま表面法線軸まわりに180°回転し、持ち手方向を左右反転します。各定Y断面と人体meshの交点を参照点とし、各参照点の上下5点ずつ、合計11点をXZ平面で局所直線近似して法線方向へ55 mmオフセットします。列内は上下交互に走査し、2 mmを超える間隔と列間を線形補間します。

各入力目標点から人体表面の最近傍点を求め、最近傍点から55 mmとなる位置へ3次元距離補正します。位置と姿勢を50点LOESSで平滑化し、MDの`+X`軸を人体表面へ向け、世界Z方向の往復走査に対してロールを固定します。位置間隔1 cmまたは姿勢変化1.5°を超える区間へ補間点を追加します。検査済み判定は論文の点群評価に合わせ、各face重心が探知楕円体へ入ったかで判定します。

IKはFFW-SG2の関節原点・軸から実装した順運動学へSciPy `least_squares`を逐次適用してseed列を生成し、`body_screening`と同じTORM二段階path-wise最適化を適用します。TORMは台車X/Y/ヨー、リフト、右腕7関節の11自由度を対象に、CHOMP/TORM平滑化をタスクJacobianのnull spaceへ射影し、各MD姿勢へdamped IKで戻します。密な表示点はTORM key補正をPCHIP展開後、全点を再投影します。最後にMD位置をhard taskとして、Zをリフト・冗長腕へ、X/Yを理想Mecanum並進へ厳密投影します。姿勢は位置を固定したJacobian null-space内で再最適化し、両立不能な場合だけ緩和します。課題方針に従い衝突勾配は使わず、MD collision STLとURDF FKで配置した右グリッパ5リンクのSTLを人体三角形と交差判定し、結果を軌道生成後の色付けにだけ使用します。

## Pythonセットアップと公開デモ生成

Python 3.10以上を使用します。

```bash
python3 -m venv .venv
.venv/bin/pip install -e '.[dev,report,video]'
```

匿名meshとWeb scene JSONを再生成します。

```bash
python scripts/create_synthetic_demo.py \
  --output web/public/data/public-demo-mesh.json
PYTHONPATH=src python scripts/build_web_demo.py
PYTHONPATH=src pytest
```

Human3D + SegFit結果をローカルで使用する場合:

```bash
PYTHONPATH=src python scripts/prepare_ai_input.py \
  --result /path/to/segfit/result.json \
  --face-parts /path/to/human3d-face-parts.json \
  --camera-position X Y Z \
  --output outputs/ai-planner-input.json

PYTHONPATH=src python scripts/build_web_demo.py \
  --mesh outputs/ai-planner-input.json \
  --output web/public/data/private-ai-scene.json
```

`private-*.json`はGitから除外されます。

## 1080p提出動画

Web viewerの3,845制御姿勢を1920×1080で時刻追従再生し、H.264 / CRF 18 / yuv420pへ変換します。制御姿勢はTORMの897姿勢間を空間変位に応じて2〜9分割したもので、固定fpsへの時間再サンプリングではありません。動画にはロボットが動く3Dビューと色凡例だけを収録します。録画前に描画済みURDFのMD制御点を直接計測し、位置誤差1 µm超または姿勢誤差2°超なら生成を停止します。

```bash
cd web
npm run record:video -- \
  'http://127.0.0.1:5173/?scene=data/private-ai-scene.json' \
  '../output/video/PAI最終課題_TADAMASA_NDA.mp4' \
  '../output/video/PAI最終課題_TADAMASA_NDA_poster.png'
```

これはkinematic visualizationであり、Isaac Sim物理実行映像ではありません。

### 差し替え可能な最終課題動画

最終動画は、後から一部分だけ変更できるように8本の独立クリップとして生成し、最後に無劣化結合します。

| ファイル | 内容 |
|---|---|
| `00_op.mp4` | OP |
| `01_project_overview.mp4` | 目的と実装内容 |
| `02_subject_orbit.mp4` | 姿勢推定前の女性モデル周回表示 |
| `03_ai_pipeline.mp4` | 点群取得 → Human3D → SMPL-X fitting |
| `04_robot_overview_2x.mp4` | ロボット全景（2倍速） |
| `05_detector_closeup_2x.mp4` | 検査部拡大（2倍速） |
| `06_results.mp4` | 検査対象範囲の網羅率・時間・軌道長・計算時間 |
| `07_ed.mp4` | ED |

```bash
python scripts/build_modular_presentation.py
```

個別クリップは`output/video/segments/`、結合版は`output/video/final/PAI最終課題_TADAMASA_NDA.mp4`へ出力します。既存クリップは保持されるため、差し替え対象だけを削除して上記コマンドを再実行できます。全クリップを再生成する場合は`--force`を付けます。OP/ED原本および研究用入力データはGitへ登録しません。

## Isaac Sim再生adapter

`scripts/isaac_lula_replay.py`は外部stage、URDF、Lula robot descriptionへ公開trajectoryを渡す汎用adapterです。Isaac Sim 5.1 RTX rendererが現在の開発hostでnative crashするため、提出動画はIsaac Simに依存させていません。

## 公開データ境界

- private研究版のGit履歴、未公開手法、論文用runtime結果を取り込みません。
- Human3D / SegFit model、checkpoint、SMPL-X model data、実測meshを再配布しません。
- FFW-SG2とRidgebackは各原ライセンスを同梱します。
- MD STLは提出者が公開を許可したassetです。
- 衝突回避、surface patch最適化、motion compensationなどの未公開研究機能を含めません。
- 保護対象の`body_screening`リポジトリをbuildや実行先として使用しません。

詳細は[第三者通知](THIRD_PARTY_NOTICES.md)を参照してください。

## 参考資料

- T. Kitahara, S. Abiko, and T. Tsujita, "Coverage-Aware Trajectory Generation for Manipulator-Based Body Screening," IFToMM 2026.
- T. Kitahara et al., "Calculation of Target Attitude of Metal Detector Along the Body Surface for Automation of Body Check Using Manipulators," 2024.
- https://github.com/human-3d/Human3D
- https://segfit.github.io/
- https://github.com/ROBOTIS-GIT/ai_worker
- https://github.com/ridgeback/ridgeback

## ライセンス

本リポジトリの独自コードは教育課題での閲覧用です。第三者componentには各提供元のライセンスが適用されます。明示的な再利用ライセンスは設定していません。
