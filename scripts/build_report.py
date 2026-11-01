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
    KeepTogether,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


ACCOUNT = "TADAMASA_NDA"
ACCOUNT_ID = "46962"
TITLE = "AI推論とIFToMM軌道によるロボットボディスクリーニング"


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument("--trajectory", default="outputs/trajectory.json")
    result.add_argument("--ai-input", default="outputs/ai-planner-input.json")
    result.add_argument("--preview", default="outputs/trajectory-preview.png")
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
    metrics = trajectory["metrics"]
    ai = ai_input["ai_inference"]
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
            "警備員が検査対象者へ接近する危険を減らすため，AIで推定した人体表面をロボットが非接触で走査するシミュレーションを構築した．3D点群へHuman3DとSegFitを適用し，胴体・骨盤・左右大腿の正面だけを抽出する．抽出面に対して公開済みIFToMM手法でcoverage-aware trajectoryを生成し，Isaac Sim上のロボットへ入力できる形式へ変換する．",
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
        [["3D point cloud", "→", "Human3D", "→", "SegFit", "→", "IFToMM", "→", "Isaac Sim"]],
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
        ["制御点 / 補間姿勢", f"{metrics['control_point_count']} / {metrics['trajectory_pose_count']}"],
        ["軌道生成時間", f"{metrics['planning_time_s']:.2f} s"],
        ["衝突proxy姿勢", f"{metrics['collision_pose_count']} / {metrics['trajectory_pose_count']}"],
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
    story.append(p("4. IFToMM軌道生成", styles["heading"]))
    story.append(
        p(
            "対象faceの面重心と法線を計算し，法線方向へ55 mm離した点を制御点候補とした．探知可能領域は半径110 / 111 / 90.6 mmの楕円体で近似し，faceの3頂点がすべて楕円体内へ入ったとき検査済みと判定した．検査済み領域に隣接する未検査faceを局所候補とし，次式を最大化する候補を順次選択した．",
            styles["body"],
        )
    )
    story.append(
        p(
            "J = 1.0 × (新規被覆面積 / 対象面積) - 0.5 × (移動距離 / 0.4) - 0.8 × (横移動 / 0.4)",
            styles["formula"],
        )
    )
    story.append(
        p(
            "制御点列を弧長で再parameterizeし，位置をcubic spline，姿勢をquaternion SLERPで補間した．目標被覆率は99%とし，99.03%に到達して停止した．",
            styles["body"],
        )
    )
    preview_path = Path(args.preview)
    if preview_path.is_file():
        image = Image(str(preview_path))
        image._restrictSize(150 * mm, 155 * mm)
        story.append(Spacer(1, 2 * mm))
        story.append(image)
        story.append(
            p(
                "図1 正面の胴体・骨盤・左右大腿に対する生成軌道．緑はcollision proxy非検出，赤はcollision proxy検出姿勢，青点はIFToMM制御点を示す．",
                styles["small"],
            )
        )

    story.append(PageBreak())
    story.append(p("5. 工夫した点", styles["heading"]))
    improvements = [
        "AI推論結果をROI選択と軌道生成へ直接接続し，単なるAI可視化にしなかった．",
        "検査対象を正面の胴体・骨盤・左右大腿へ限定し，対象外faceが混入した場合に全身へ自動fallbackしない設計とした．",
        "衝突回避を追加してIFToMM手法を変更せず，衝突proxy検出区間を赤く表示して限界を隠さない構成とした．",
        "private研究版から履歴や未公開手法を持ち込まず，論文で公開済みの式だけを独立実装した．",
        "第三者model，checkpoint，SMPL-X dataを再配布せず，外部入力adapterとして分離した．",
    ]
    for index, item in enumerate(improvements, 1):
        story.append(p(f"{index}. {item}", styles["body"]))

    story.append(p("6. 制約と今後の課題", styles["heading"]))
    story.append(
        p(
            "本実装は衝突情報を軌道の評価関数へ入れていない．628補間姿勢のうち214姿勢で保守的な直方体proxyと人体meshの重なりを検出したが，これは物理simulation上の接触率ではなく，可視化用の近似判定である．また，軌道生成時点ではrobotの可動域，自己干渉，環境衝突を考慮していない．今後は検出器shaftを含む正確な形状制約，IK到達可能性，自己干渉回避を導入し，正面・背面を連続して検査する必要がある．",
            styles["body"],
        )
    )
    story.append(p("7. まとめ", styles["heading"]))
    story.append(
        p(
            "Human3DとSegFitのAI推論結果を用い，限定した人体正面ROIへ公開済みIFToMM軌道生成を適用した．<br/>生成軌道の被覆率は99.03%であり，衝突可能区間を色で明示した．研究用private platformの未公開新規性を含まないpublic codeとして整理し，Isaac Simで再生可能な入力形式とした．",
            styles["body"],
        )
    )

    story.append(p("参考文献", styles["heading"]))
    references = [
        "[1] T. Kitahara, S. Abiko, and T. Tsujita, “Coverage-Aware Trajectory Generation for Manipulator-Based Body Screening,” IFToMM 2026.",
        "[2] T. Kitahara et al., “Calculation of Target Attitude of Metal Detector Along the Body Surface for Automation of Body Check Using Manipulators,” 2024.",
        "[3] Human3D, https://github.com/human-3d/Human3D",
        "[4] SegFit, https://segfit.github.io/",
        "[5] SMPL-X, https://smpl-x.is.tue.mpg.de/",
        "[6] NVIDIA Isaac Sim, https://docs.isaacsim.omniverse.nvidia.com/",
    ]
    for reference in references:
        story.append(p(reference, styles["small"]))

    story.append(Spacer(1, 6 * mm))
    note = Table(
        [[p("公開情報", styles["small"]), p("GitHub: https://github.com/k-tadamasa/PAI2026-Advanced1<br/>個人情報・第三者model・private研究履歴は含めていない．", styles["small"])]],
        colWidths=[26 * mm, 125 * mm],
    )
    note.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), colors.HexColor("#f1f5f9")),
                ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("TOPPADDING", (0, 0), (-1, -1), 7),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
            ]
        )
    )
    story.append(KeepTogether(note))

    document.build(story, onFirstPage=footer, onLaterPages=footer)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
