# Third-party notices

このリポジトリは公開可能な第三者資産をライセンス表示とともに収録し、AIモデル本体・checkpoint・実測人体meshは収録しません。

| Component | Purpose | Included |
|---|---|---|
| ROBOTIS AI Worker FFW-SG2 | 上半身URDF・visual mesh | Yes / Apache-2.0 |
| Clearpath Ridgeback | Mecanum台車visual mesh | Yes / BSD-3-Clause |
| MD.stl / MD_no_hole_poly.stl | 金属探知機visual・collision proxy | Yes / submission-author asset |
| Human3D | 3D人体instance・部位推論 | No |
| SegFit | 点群からSMPL-X fitting | No |
| SMPL-X | 人体mesh model | No |
| React / React Three Fiber | Web UI・3D scene | Dependency |
| Three.js | WebGL rendering | Dependency |
| URDF Loader | BrowserでのURDF読込 | Dependency / Apache-2.0 |
| Lucide | UI icons | Dependency / ISC |
| NumPy / SciPy | trajectory calculation | Dependency |
| TORM | path-wise whole-body inverse kinematics | Included algorithmic port / BSD |
| ReportLab | 提出PDF生成 | Build tool |
| Playwright Core / Chromium | deterministic video frames | Build tool |
| FFmpeg | H.264 encoding | Build tool |

収録した原ライセンスは次に保存しています。

- `web/public/assets/robot/ffw_description/LICENSE-APACHE-2.0.txt`
- `web/public/assets/robot/ridgeback_description/LICENSE-BSD-3-CLAUSE.txt`

## TORM

`src/pai_body_screening/torm_pathwise_optimizer.py` is an
algorithm-preserving port of parts of TORM.

- Project: https://github.com/cheulkang/TORM
- Reference revision: `c19690af549eb362c3d2359c757a6dcc89ec65e7`
- Package license: BSD
- Original TORM implementation modified by Mincheul Kang
- Portions retain `Copyright (c) 2009, Willow Garage, Inc.`

The complete BSD license notice is retained in the source module. Related
publications are Kang et al., IROS 2020, DOI
`10.1109/IROS45743.2020.9341358`, and Kang and Yoon, Autonomous Robots 2022,
DOI `10.1007/s10514-022-10040-1`.

公式情報:

- ROBOTIS AI Worker: https://github.com/ROBOTIS-GIT/ai_worker
- ROBOTIS simulation assets: https://github.com/ROBOTIS-GIT/cyclo_lab
- Ridgeback: https://github.com/ridgeback/ridgeback
- Human3D: https://github.com/human-3d/Human3D
- SegFit: https://segfit.github.io/
- SMPL-X: https://smpl-x.is.tue.mpg.de/
- Three.js: https://threejs.org/
- React Three Fiber: https://r3f.docs.pmnd.rs/
- URDF Loader: https://github.com/gkjohnson/urdf-loaders
- Lucide: https://lucide.dev/

## Public/private data boundary

- GitHub Pagesは単純な楕円断面から生成した匿名人体meshを使用します。
- Human3D + SegFitの実測推論meshは`outputs/`および`private-*.json`としてignoreし、提出動画のローカル生成にだけ使用します。
- `body_screening` repositoryの履歴・未公開手法・runtime dataは収録しません。
- Isaac Sim再生scriptが参照する外部stage・robot descriptionは利用者が各配布条件に従って用意します。
