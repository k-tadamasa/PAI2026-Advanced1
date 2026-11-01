# PAI2026 Advanced1 Body Screening Demo 実装計画

この公開版は，3D点群から推定した人体表面の限定領域を，シミュレーション内のロボットで走査する教育用デモである．公開済みAI手法と公開済み軌道生成手法だけを使用し，未公開研究，第三者モデルファイル，実験データを含めない．

# 対象範囲

走査対象はHuman3Dの次の部位に限定する．

| ID | 部位 |
|---:|---|
| 2 | 右大腿 |
| 7 | 胴体 |
| 14 | 左大腿 |
| 15 | 骨盤 |

実装上の対象集合は `frozenset({2, 7, 14, 15})` とする．対象外のfaceを経路生成やカバレッジ評価へ混入させない．

# 最小パイプライン

1. 3Dセンサー相当の点群を取得する．
2. Human3Dで人体と身体部位を推論する．
3. SegFitでSMPL-Xメッシュを点群へフィッティングする．
4. 部位IDから走査対象faceを抽出する．
5. 前面可視面に対して走査軌道を生成する．
6. NVIDIA Isaac Simでロボット軌道を実行する．
7. 対象面積カバレッジ，経路長，実行時間を表示する．

AI推論結果は表示だけに使わず，対象face，表面位置，法線，離隔を決める入力として使用する．

# 軌道生成

提出版は，KitaharaらのIFToMM 2026論文で公開されたcoverage-aware trajectory generationを使用する．未検査領域に隣接する局所候補から，新規被覆面積，移動距離，横移動を評価して次の制御点を選ぶ．論文で報告された55 mm離隔，探知楕円体半径110 / 111 / 90.6 mm，重み1.0 / 0.5 / 0.8を使用する．

位置はcubic spline，姿勢はSLERPで補間する．衝突回避は評価関数へ追加しない．検出器proxyと人体meshが重なる姿勢を赤で表示し，IFToMM論文と同じく今後の課題として扱う．

# 実行環境

Isaac Sim WorkstationはWindows 11とUbuntuで利用できる．本プロジェクトでは，安定して再現できる環境を主実行環境とし，別OSへの移植を提出の必須条件にしない．

Windowsで試す場合は，Compatibility Checker，空シーン，単純ロボット，単一センサーの順に確認する．高解像度描画，多数のRTXセンサー，complete local asset packを初期構成へ含めない．AI推論を別の対応環境で実行し，推定済みメッシュとラベルを受け取る分離構成も許容する．

# 公開構成

```text
PAI2026-Advanced1/
├─ README.md
├─ THIRD_PARTY_NOTICES.md
├─ configs/
│  └─ demo.json
├─ src/pai_body_screening/
│  ├─ ai_adapter.py
│  ├─ roi.py
│  ├─ iftomm.py
│  ├─ collision.py
│  └─ cli.py
├─ scripts/                 # 入力変換，Isaac再生，PDF・動画生成
├─ tests/                   # 公開版回帰test
└─ docs/
```

# 実装手順

1. ROI抽出と単体テストを実装する．
2. 保存済み推論結果で対象メッシュを表示する．
3. 公開済みIFToMM手法を独立実装する．
4. 軌道が対象faceだけを参照することを検証する．
5. 離隔，速度，姿勢，部位間移動を3D表示で確認する．
6. Isaac Simで一部位を低速実行する．
7. 胴体，骨盤，左右大腿へ拡張する．
8. 指標，設定，バージョンを保存する．
9. 動画，README，第三者通知を整える．

# 合格条件

- ROIに含まれる部位IDが `2`，`7`，`14`，`15` のいずれかである．
- ROIが空または不正な場合は停止し，全身へ黙ってフォールバックしない．
- 軌道点が前面可視面を基準とし，設定した離隔許容差内にある．
- 衝突判定は軌道生成後に行い，計画結果を変更しない．
- 衝突proxy姿勢を赤，それ以外を緑で表示する．
- 使用手法，パラメータ，座標系，ソフトウェアバージョンを保存する．
- 動画で点群，AI推論，対象領域，軌道，ロボット実行，結果を確認できる．

# 公開しないもの

- 第三者のモデル，checkpoint，データセット，配布PDF．
- 未公開の研究手法，評価結果，論文資料．
- 個人情報，資格情報，内部ネットワーク情報，ホスト固有パス．
- 別のprivateリポジトリのGit履歴．

# 第三者依存

- Human3D：https://github.com/human-3d/Human3D
- SegFit：https://segfit.github.io/
- SMPL-X：https://smpl-x.is.tue.mpg.de/
- NVIDIA Isaac Sim：https://docs.isaacsim.omniverse.nvidia.com/

モデル本体とcheckpointは各公式配布元のライセンスに従って取得し，本リポジトリへ再配布しない．
