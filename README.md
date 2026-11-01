# PAI2026 Advanced1: AI Body Screening Demo

Physical AI応用編1の最終課題として，AIで推定した人体表面の限定領域をロボットで走査するシミュレーションです．Human3DとSegFitの推論結果から胴体，骨盤，左右大腿の正面だけを抽出し，公開済みIFToMM手法で検査軌道を生成します．

提出アカウントは `TADAMASA_NDA`，アカウントIDは `46962` です．氏名，所属，メールアドレスなどの個人情報は収録していません．

## デモの要点

- AI: Human3Dによる人体・部位推論とSegFitによるSMPL-X fitting
- 検査対象: Human3D部位ID `{2, 7, 14, 15}`
- 対象面: カメラ側を向く正面faceのみ
- 軌道: KitaharaらのIFToMM coverage-aware trajectory baseline
- 可視化: 検査軌道を緑，衝突proxyが検出された区間を赤で表示
- 制約: 衝突は計画から除外せず，今後の課題として明示
- 実行: NVIDIA Isaac SimのLula IKへ軌道姿勢を渡す汎用再生スクリプト

実測済みのHuman3D + SegFit結果に対して，次の結果を得ました．

| 指標 | 結果 |
|---|---:|
| AI入力点数 | 2,624 |
| fitting mesh | 10,475頂点 / 20,908面 |
| 正面ROI面積 | 0.4499 m2 |
| 被覆率 | 99.03% |
| 経路長 | 6.41 m |
| 制御点数 | 72 |
| 計画時間 | 約3.4 s |
| 衝突proxy姿勢 | 214 / 628 |

## アルゴリズム

論文の手順に沿い，対象meshの面重心と法線から55 mm離隔した制御点候補を生成します．現在点で検査済みとなった面に隣接する未検査面を候補とし，次の評価関数が最大の点を順次選択します．

```text
J = 1.0 * newly_covered_area / target_area
  - 0.5 * travel_distance / 0.4
  - 0.8 * lateral_motion / 0.4
```

探知可能領域は半径 `(110, 111, 90.6) mm` の楕円体で近似し，三角形の3頂点がすべて楕円体内に入ったとき検査済みとします．制御点列は弧長で再parameterizeし，位置をcubic spline，姿勢をquaternion SLERPで補間します．

## セットアップ

Python 3.10以上を使用します．

```bash
python3 -m venv .venv
.venv/bin/pip install -e '.[dev,report,video]'
```

### 公開smoke test

合成meshはアルゴリズムの動作確認専用であり，AI評価結果としては使用しません．

```bash
python scripts/create_synthetic_demo.py --output /tmp/pai2026-mesh.json
pai-body-screening /tmp/pai2026-mesh.json \
  --output outputs/trajectory.json \
  --preview outputs/trajectory-preview.png \
  --coverage 0.99
pytest
```

### Human3D + SegFit結果を使う

モデル，checkpoint，SMPL-Xデータは各公式配布元から取得し，このリポジトリには置きません．完了済み推論結果とface-part対応表を次のように一時入力へ変換します．

```bash
PYTHONPATH=src python scripts/prepare_ai_input.py \
  --result /path/to/segfit/result.json \
  --face-parts /path/to/human3d-face-parts.json \
  --camera-position X Y Z \
  --output outputs/ai-planner-input.json

pai-body-screening outputs/ai-planner-input.json \
  --output outputs/trajectory.json \
  --preview outputs/trajectory-preview.png
```

AI推論は表示だけに使うのではなく，SegFit meshの表面位置と法線，Human3D部位IDによるROI選択へ直接使用します．

## Isaac Simでの再生

`scripts/isaac_lula_replay.py` は，既存のUSD stage，URDF，Lula robot descriptionを外部入力として受け取ります．private研究リポジトリやその履歴への依存はありません．

```bash
/path/to/isaac-sim/python.sh scripts/isaac_lula_replay.py \
  --trajectory outputs/trajectory.json \
  --stage /path/to/scene.usd \
  --robot-prim /World/Robot \
  --robot-description /path/to/robot_descriptor.yaml \
  --urdf /path/to/robot.urdf \
  --end-effector detector_control_point
```

この再生はLula IKの成功した姿勢だけをロボットへ適用します．衝突回避は行わず，赤い軌道点を今後の課題として残します．Isaac Sim 5.1のLula APIを対象としています．

Isaac Sim標準Frankaだけで新規stageを構築する独立デモも用意しています．private stageやprivate robot assetは使用しません．

```bash
/path/to/isaac-sim/python.sh scripts/isaac_franka_demo.py \
  --trajectory outputs/trajectory.json \
  --ai-input outputs/ai-planner-input.json \
  --execution-report outputs/franka-execution.json \
  --headless
```

現在の開発hostではIsaac Sim 5.1のRTX rendererがアプリ初期化中にnative crashしたため，このFranka実行のruntime検証は完了していません．公開版の計画処理，JSON出力，回帰test，動画previewはIsaac Simから独立して検証済みです．

## 提出用動画preview

公開JSONだけから，AI mesh，正面ROI，IFToMM軌道，簡易2-link robot armを描くMP4を生成できます．これはIsaac Sim実行映像ではなく，軌道とcollision proxyを確認するkinematic previewです．

```bash
python scripts/build_demo_video.py
```

標準設定では `output/video/PAI最終課題_TADAMASA_NDA.mp4` に62.8秒，960×540，H.264/yuv420pの動画を出力します．緑はsafe proxy，赤はcollision proxyであり，collision avoidanceは行いません．

## PDF生成

AI入力，軌道JSON，preview画像を準備した後，次のコマンドで3ページのA4説明資料を生成します．

```bash
python scripts/build_report.py
```

出力先は `output/pdf/説明資料_PAI最終課題_TADAMASA_NDA.pdf` です．

## ディレクトリ

```text
configs/                 公開パラメータ
docs/                    実装計画，動画構成，提出確認
scripts/                 AI入力変換，合成test，Isaac再生，PDF・動画生成
src/pai_body_screening/  ROI，IFToMM軌道，衝突色付け，入出力
tests/                   公開版の回帰test
```

## 公開方針

- private研究版のGit履歴，未公開手法，論文用実験結果を取り込みません．
- third-partyのモデル，checkpoint，dataset，SMPL-Xデータ，配布PDFを再配布しません．
- 絶対パス，内部IP，資格情報，個人情報をcommitしません．
- 衝突回避，surface patch分割，MILP順序最適化，motion compensation，全身IK研究機能を含めません．
- collision proxyは評価と色付け専用であり，計画の評価関数には使用しません．

詳細は [公開版実装計画](docs/implementation_plan.md) と [第三者通知](THIRD_PARTY_NOTICES.md) を参照してください．

## 参考資料

- T. Kitahara, S. Abiko, and T. Tsujita, "Coverage-Aware Trajectory Generation for Manipulator-Based Body Screening," IFToMM 2026.
- T. Kitahara et al., "Calculation of Target Attitude of Metal Detector Along the Body Surface for Automation of Body Check Using Manipulators," 2024.
- [Human3D](https://github.com/human-3d/Human3D)
- [SegFit](https://segfit.github.io/)
- [SMPL-X](https://smpl-x.is.tue.mpg.de/)
- [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/)

## ライセンス

本リポジトリの独自コードは教育課題での閲覧用です．third-party componentには各提供元のライセンスが適用されます．明示的な再利用ライセンスは設定していません．
