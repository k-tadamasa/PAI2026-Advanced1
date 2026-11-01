# Third-party notices

このリポジトリは，次の外部softwareまたはmodelの出力を利用できるadapterを含みます．model本体，checkpoint，dataset，SMPL-X model data，USD/URDF assetは同梱していません．利用者が各公式配布元の条件に従って用意してください．

| Component | Purpose | Included here |
|---|---|---|
| Human3D | 3D人体instance・部位推論 | No |
| SegFit | 点群からSMPL-X fitting | No |
| SMPL-X | 人体mesh model | No |
| NVIDIA Isaac Sim | simulation・Lula IK | No |
| NumPy | numerical calculation | Dependency only |
| SciPy | cubic spline・SLERP | Dependency only |
| Matplotlib | preview rendering | Dependency only |
| ReportLab | 提出PDF生成 | Build tool only |
| imageio-ffmpeg | 提出動画のH.264 encoding | Build tool only |

公式情報:

- Human3D: https://github.com/human-3d/Human3D
- SegFit: https://segfit.github.io/
- SMPL-X: https://smpl-x.is.tue.mpg.de/
- NVIDIA Isaac Sim: https://docs.isaacsim.omniverse.nvidia.com/
- NumPy: https://numpy.org/
- SciPy: https://scipy.org/
- Matplotlib: https://matplotlib.org/
- ReportLab: https://www.reportlab.com/
- imageio-ffmpeg: https://github.com/imageio/imageio-ffmpeg

`scripts/isaac_lula_replay.py` は外部のrobot USD，URDF，Lula robot descriptionを引数として受け取ります．これらのassetを本リポジトリへcommitしないでください．
