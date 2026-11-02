#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (
    Image,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


ACCOUNT = "TADAMASA_NDA"
ACCOUNT_ID = "46962"
TITLE = "AI推論と体表面追従軌道によるロボットボディスクリーニング"


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument("--trajectory", default="outputs/trajectory.json")
    result.add_argument("--ai-input", default="outputs/ai-planner-input.json")
    result.add_argument("--scene", default="web/public/data/private-ai-scene.json")
    result.add_argument(
        "--presentation", default="web/public/data/private-presentation.json"
    )
    result.add_argument("--preview", default="outputs/trajectory-preview.png")
    result.add_argument(
        "--web-preview",
        default="output/web/private-ai-preview.png",
    )
    result.add_argument(
        "--output",
        default=f"output/pdf/説明資料_PAI最終課題_{ACCOUNT}.pdf",
    )
    return result


def register_fonts() -> tuple[str, str]:
    pdfmetrics.registerFont(UnicodeCIDFont("HeiseiKakuGo-W5"))
    pdfmetrics.registerFont(UnicodeCIDFont("HeiseiMin-W3"))
    return "HeiseiKakuGo-W5", "HeiseiMin-W3"


def footer(canvas, document) -> None:
    canvas.saveState()
    canvas.setFont("HeiseiKakuGo-W5", 8)
    canvas.setFillColor(colors.HexColor("#64748b"))
    canvas.drawString(20 * mm, 11 * mm, f"PAI2026 Advanced1 | {ACCOUNT} | ID {ACCOUNT_ID}")
    canvas.drawRightString(190 * mm, 11 * mm, f"{document.page}")
    canvas.restoreState()


def p(text: str, style: ParagraphStyle) -> Paragraph:
    return Paragraph(text, style)


def main() -> int:
    args = parser().parse_args()
    sans, serif = register_fonts()
    trajectory = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    ai_input = json.loads(Path(args.ai_input).read_text(encoding="utf-8"))
    scene_path = Path(args.scene)
    scene = (
        json.loads(scene_path.read_text(encoding="utf-8"))
        if scene_path.is_file()
        else trajectory
    )
    metrics = scene["metrics"]
    ai = ai_input["ai_inference"]
    presentation_path = Path(args.presentation)
    presentation = (
        json.loads(presentation_path.read_text(encoding="utf-8"))
        if presentation_path.is_file()
        else None
    )
    if presentation is not None:
        ai = {
            "human3d_point_count": presentation["human3d"]["point_count"],
            "fitted_vertex_count": len(presentation["smplx"]["vertices"]),
            "fitted_face_count": len(presentation["smplx"]["faces"]),
        }
    source_pose_count = int(
        metrics.get("source_trajectory_pose_count", metrics["trajectory_pose_count"])
    )
    inspection_time_s = float(
        metrics.get(
            "inspection_time_s",
            scene.get("trajectory", [{}])[-1].get("robot", {}).get("time_s", 0.0),
        )
    )
    pose_estimation_time_s = (
        float(presentation["metrics"]["pose_estimation_time_s"])
        if presentation is not None
        else None
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    document = SimpleDocTemplate(
        str(output),
        pagesize=A4,
        rightMargin=18 * mm,
        leftMargin=18 * mm,
        topMargin=17 * mm,
        bottomMargin=19 * mm,
        title=TITLE,
        author=ACCOUNT,
        subject="Physical AI応用編1 最終課題",
    )
    base = getSampleStyleSheet()
    styles = {
        "title": ParagraphStyle(
            "JPTitle",
            parent=base["Title"],
            fontName=serif,
            fontSize=22,
            leading=31,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#0f172a"),
            spaceAfter=8 * mm,
        ),
        "subtitle": ParagraphStyle(
            "JPSubtitle",
            parent=base["Normal"],
            fontName=sans,
            fontSize=10.5,
            leading=16,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#475569"),
            spaceAfter=8 * mm,
        ),
        "heading": ParagraphStyle(
            "JPHeading",
            parent=base["Heading2"],
            fontName=sans,
            fontSize=15,
            leading=21,
            textColor=colors.HexColor("#0f4c81"),
            spaceBefore=5 * mm,
            spaceAfter=3 * mm,
        ),
        "body": ParagraphStyle(
            "JPBody",
            parent=base["BodyText"],
            fontName=sans,
            fontSize=9.3,
            leading=15.2,
            alignment=TA_LEFT,
            textColor=colors.HexColor("#1e293b"),
            spaceAfter=2.2 * mm,
        ),
        "small": ParagraphStyle(
            "JPSmall",
            parent=base["BodyText"],
            fontName=sans,
            fontSize=7.8,
            leading=11.5,
            textColor=colors.HexColor("#475569"),
        ),
        "formula": ParagraphStyle(
            "Formula",
            parent=base["BodyText"],
            fontName=sans,
            fontSize=9.5,
            leading=15,
            leftIndent=8 * mm,
            borderColor=colors.HexColor("#cbd5e1"),
            borderWidth=0.6,
            borderPadding=6,
            backColor=colors.HexColor("#f8fafc"),
            spaceBefore=2 * mm,
            spaceAfter=3 * mm,
        ),
    }

    story = []
    story.append(Spacer(1, 13 * mm))
    story.append(p(TITLE, styles["title"]))
    story.append(
        p(
            f"Physical AI応用編1 最終課題<br/>Account: {ACCOUNT} / ID: {ACCOUNT_ID}",
            styles["subtitle"],
        )
    )
    story.append(p("1. 課題概要", styles["heading"]))
    story.append(
        p(
            "警備員が検査対象者へ接近する危険を減らすため，AIで推定した人体表面をロボットが非接触で走査する可視化システムを構築した．3D点群へHuman3DとSegFitを適用し，胴体・骨盤・左右大腿のうち面法線のXY投影と-X軸の角度が75°未満となる正面を抽出する．さらに左右を抽出幅の中央80%，上端を最高面重心から100 mm下へ限定する．抽出面に対して走査線間隔を1.5倍にした体表面追従4列往復軌道を生成し，TORMでFFW-SG2とMecanum台車の11自由度軌道を平滑化してWeb上で再生する．",
            styles["body"],
        )
    )
    story.append(p("2. AIの利用", styles["heading"]))
    story.append(
        p(
            "Human3Dの推論で人体点群と身体部位IDを求め，SegFitでSMPL-X meshを点群へfittingした．学習は行わず推論のみを利用した．AI結果は表示用途だけではなく，検査対象face，表面位置，法線，離隔を決める入力として使用している．検査対象はHuman3D部位ID 2 (右大腿)，7 (胴体)，14 (左大腿)，15 (骨盤) に固定した．",
            styles["body"],
        )
    )

    pipeline = Table(
        [["3D point cloud", "→", "Human3D", "→", "SegFit", "→", "4-lane raster", "→", "Web Viewer"]],
        colWidths=[31 * mm, 7 * mm, 24 * mm, 7 * mm, 22 * mm, 7 * mm, 23 * mm, 7 * mm, 27 * mm],
    )
    pipeline.setStyle(
        TableStyle(
            [
                ("FONTNAME", (0, 0), (-1, -1), sans),
                ("FONTSIZE", (0, 0), (-1, -1), 8),
                ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("BACKGROUND", (0, 0), (0, 0), colors.HexColor("#e2e8f0")),
                ("BACKGROUND", (2, 0), (2, 0), colors.HexColor("#dbeafe")),
                ("BACKGROUND", (4, 0), (4, 0), colors.HexColor("#dbeafe")),
                ("BACKGROUND", (6, 0), (6, 0), colors.HexColor("#dcfce7")),
                ("BACKGROUND", (8, 0), (8, 0), colors.HexColor("#fef3c7")),
                ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
                ("TOPPADDING", (0, 0), (-1, -1), 7),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
            ]
        )
    )
    story.append(Spacer(1, 3 * mm))
    story.append(pipeline)
    story.append(p("3. 結果", styles["heading"]))
    metric_rows = [
        ["項目", "結果"],
        ["AI推論点数", f"{ai['human3d_point_count']:,}"],
        ["SegFit mesh", f"{ai['fitted_vertex_count']:,}頂点 / {ai['fitted_face_count']:,}面"],
        ["正面ROI面積", f"{metrics['target_area_m2']:.4f} m2"],
        ["被覆率", f"{100 * metrics['coverage_ratio']:.2f}%"],
        ["経路長", f"{metrics['path_length_m']:.2f} m"],
        ["検査時間", f"{inspection_time_s:.1f} s（走査速度2倍）"],
        ["TORM姿勢 / 適応細分化後", f"{source_pose_count} / {metrics['trajectory_pose_count']}"],
        ["軌道生成時間", f"{metrics['planning_time_s']:.2f} s"],
        *(
            [["姿勢推定時間", f"{pose_estimation_time_s:.3f} s"]]
            if pose_estimation_time_s is not None
            else []
        ),
        ["実形状衝突姿勢", f"{metrics['collision_pose_count']} / {source_pose_count}"],
    ]
    metric_table = Table(metric_rows, colWidths=[68 * mm, 83 * mm])
    metric_table.setStyle(
        TableStyle(
            [
                ("FONTNAME", (0, 0), (-1, -1), sans),
                ("FONTSIZE", (0, 0), (-1, -1), 8.5),
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#0f4c81")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#cbd5e1")),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f8fafc")]),
                ("ALIGN", (1, 1), (1, -1), "RIGHT"),
                ("TOPPADDING", (0, 0), (-1, -1), 4.5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4.5),
            ]
        )
    )
    story.append(metric_table)

    story.append(PageBreak())
    story.append(p("4. 体表面追従4列往復軌道とTORM IK", styles["heading"]))
    story.append(
        p(
            "選択ROIの横幅へ4本のy_ref列を置き，最外線を従来位置へ維持したまま内側2本を外へ移動し，中央間隔を従来比1.2倍とした．各定Y断面と人体meshの交点を参照点とし，各点の上下5点ずつをXZ平面で局所直線近似して法線方向へ55 mmオフセットした．列内を上下交互に走査し，2 mmを超える間隔と列間を線形補間した．",
            styles["body"],
        )
    )
    story.append(
        p(
            "p_mod = p_near + l_des (p_des - p_near) / ||p_des - p_near||,  l_des = 55 mm",
            styles["formula"],
        )
    )
    story.append(
        p(
            f"位置と姿勢を50点LOESSで平滑化し，TORMで得た{source_pose_count}姿勢の各区間を，MD位置2 mm，MD姿勢0.5°，各関節0.5°，リフト2 mm以下となるまで適応分割した．その結果，{metrics['trajectory_pose_count']}制御姿勢となり，検査対象ROIに対して被覆率{100 * metrics['coverage_ratio']:.2f}%を得た．",
            styles["body"],
        )
    )
    story.append(
        p(
            "Web viewerはThree.jsによるkinematic simulationであり，FFW-SG2上半身とRidgeback Mecanum台車を表示して，台車を理想holonomic運動として軌道へ追従させた．SMPL-X基準面を薄い灰色，MD実体を灰色，探知可能楕円体を淡い透明青，累積検査済みfaceを緑で表示する．一度でも交差した人体faceと右グリッパlinkは以後も赤く保持し，MDと楕円体は変色させない．",
            styles["body"],
        )
    )
    preview_path = Path(args.web_preview)
    if not preview_path.is_file():
        preview_path = Path(args.preview)
    if preview_path.is_file():
        image = Image(str(preview_path))
        image._restrictSize(174 * mm, 132 * mm)
        story.append(Spacer(1, 2 * mm))
        story.append(image)
        story.append(
            p(
                "図1 1080p Web viewer．薄い灰色はSMPL-X基準面，緑は検査済み人体面，淡い透明青は探知楕円体，灰色はMD実体，赤は累積した人体face・右グリッパlinkの実mesh交差履歴を示す．",
                styles["small"],
            )
        )

    story.append(PageBreak())
    story.append(p("5. 工夫した点", styles["heading"]))
    improvements = [
        (
            "1. 問題設定",
            "警備員が危険物を所持する可能性のある対象者へ接近するリスクを減らすため，"
            "人体形状をAIで推定し，その表面に沿う検査軌道を生成してロボットで実行する課題を設定した．"
            "形状推定・軌道生成・ロボットシミュレーションを一連のタスクとして結び付け，"
            "AIの推論結果を実際のロボット動作へ利用する点を工夫した．",
        ),
        (
            "2. 検査済み領域の人体形状モデルへの記録",
            "各軌道姿勢で探知楕円体に含まれた人体メッシュのface IDを検査済みとして記録し，"
            "それまでの履歴を人体形状モデル上へ保持した．この記録から検査対象ROIだけを分母とする"
            "網羅率を算出するとともに，動画では検査済みfaceを緑へ変化させた．"
            "これにより，ロボットがどこを検査したか，どこが未検査かを時間経過とともに確認できる．",
        ),
    ]
    for title, detail in improvements:
        story.append(p(f"<b>{title}</b><br/>{detail}", styles["body"]))
        story.append(Spacer(1, 2 * mm))

    story.append(p("6. 制約と今後の課題", styles["heading"]))
    story.append(
        p(
            f"本実装は衝突情報を軌道生成へ入れていない．{source_pose_count} TORM姿勢のうち{metrics['collision_pose_count']}姿勢で，URDF FKにより配置したMD collision STL・右グリッパ5リンクSTLと人体meshの三角形交差を検出した．これはThree.js上のkinematicな形状交差可視化であり，接触力を計算する物理simulationではない．Mecanum台車はスリップ，速度・加速度，床接触を省略した理想運動である．今後は自己干渉，台車運動制約，環境衝突と衝突回避を導入する必要がある．",
            styles["body"],
        )
    )
    story.append(p("7. まとめ", styles["heading"]))
    story.append(
        p(
            f"Human3DとSegFitのAI推論結果を用い，限定した人体正面ROIへ走査線間隔1.5倍の体表面追従4列往復軌道を適用し，公開TORM手法で11自由度軌道を平滑化した．<br/>生成軌道の被覆率は{100 * metrics['coverage_ratio']:.2f}%であり，FFW-SG2 Mecanum，MD，探知領域，検査済み面，衝突可能区間をWeb上で確認できる．研究用private platformの未公開新規性を含まないpublic codeとして整理した．",
            styles["body"],
        )
    )

    story.append(p("参考文献", styles["heading"]))
    references = [
        "[1] “Calculation of Target Attitude of Metal Detector Along the Body Surface for Automation of Body Check Using Manipulators,” SI 2024.",
        "[2] Human3D, https://github.com/human-3d/Human3D",
        "[3] SegFit, https://segfit.github.io/",
        "[4] SMPL-X, https://smpl-x.is.tue.mpg.de/",
        "[5] ROBOTIS AI Worker, https://github.com/ROBOTIS-GIT/ai_worker",
        "[6] Clearpath Ridgeback, https://github.com/ridgeback/ridgeback",
        "[7] Three.js, https://threejs.org/",
    ]
    for reference in references:
        story.append(p(reference, styles["small"]))

    document.build(story, onFirstPage=footer, onLaterPages=footer)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
