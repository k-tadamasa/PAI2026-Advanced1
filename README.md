# PAI2026 Advanced1 Body Screening Demo

Physical AI応用編1の最終課題用に，Body Screening Research Platformから非新規の教育用ベースラインだけを独立させる公開リポジトリです．

Human3DとSegFitで推定した人体表面のうち，胴体，骨盤，左右大腿だけを対象に，公開済み手法または単純な基準軌道でロボット走査を実行します．private研究版の新規手法，未公開実験，論文資料，第三者モデル・checkpointは含めません．

## 現在の状態

- リポジトリの初期構成を作成中です．
- 対象Human3D部位IDは `{2, 7, 14, 15}` です．
- 軌道生成は，追加されるSI2024／IFToMM PDFを確認して決定します．
- 締切優先のフォールバックは，部位または連結成分ごとのPCA蛇行走査です．
- 主実行環境はUbuntuのIsaac Simとし，Windows実行は互換性確認後の追加経路とします．

## 作業開始

最初に [公開版実装計画](docs/implementation_plan.md) を全文確認してください．

## 公開方針

- private研究リポジトリのGit履歴を取り込みません．
- 公開対象を空のリポジトリへホワイトリスト方式で追加します．
- SMPL-Xモデル，Human3D／SegFitの重み，第三者PDFをcommitしません．
- 個人情報，内部IP，ホスト固有パス，資格情報をcommitしません．

## ライセンス

独自コードのライセンスは，第三者依存と研究成果の公開範囲を確認してから決定します．
