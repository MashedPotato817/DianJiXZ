# REPORT — 电赛控制题技术报告资源

本文件夹为全国大学生电子设计竞赛控制类题目的技术报告资源库，包含往年范例和可直接使用的 LaTeX 模板。

---

## 文件夹结构

```
REPORT/
├── README.md              # 本文件 — 总索引
├── 范例/                   # 往年技术报告范例
│   ├── README.md          #   范例说明 + 报告标准结构 + 评分框架
│   ├── 2024_H题_自动行驶小车_技术报告.pdf   # 2024 年 H 题报告 PDF
│   └── 报告参考模板.doc    #   竞赛官方参考模板（Word 格式）
└── ready/                 # 比赛可直接用的 LaTeX 模板
    ├── README.md          #   模板使用说明 + 编译指南 + 赛题对照
    ├── template_general.tex   # 通用控制题模板
    ├── template_car.tex       # 小车巡线/自动行驶类模板
    └── template_sensor.tex    # 传感器测控/物理量闭环类模板
```

---

## 使用流程

```
比赛拿到题目
    │
    ├── 1. 翻看 范例/2024_H题_自动行驶小车_技术报告.pdf
    │       了解一份完整报告长什么样
    │
    ├── 2. 阅读 范例/README.md
    │       熟悉报告的 7 章标准结构和评分要点
    │
    ├── 3. 根据题目类型，在 ready/ 中选择最接近的模板
    │       - 小车巡线类 → template_car.tex
    │       - 传感器测控类 → template_sensor.tex
    │       - 不确定选哪个 → template_general.tex
    │
    ├── 4. 阅读 ready/README.md
    │       了解编译方法和填空顺序
    │
    └── 5. 用编辑器打开 .tex 模板，按标记填空
            搜索 \blank 或 \placeholder 快速定位填空位
```

---

## 关于本仓库

- **项目**：NUEDC 控制类备赛 — 基于 MSPM0G3507 的差速电机小车平台
- **主控**：TI MSPM0G3507 (Cortex-M0+)
- **固件主目录**：[firmware/WHEELTEC_C07A_CAR/](../firmware/WHEELTEC_C07A_CAR/)
- **2024 年参赛报告源码**：[report_2024/](../report_2024/)

---

## 外部资源链接

| 资源 | 链接 |
|------|------|
| 竞赛培训网（官方） | [nuedc-training.com.cn](https://www.nuedc-training.com.cn) |
| LaTeX 电赛模板（GitHub） | [chenboshuo/electronics_design_contest_template](https://github.com/chenboshuo/electronics_design_contest_template) |
| 历年赛题汇总（GitHub） | [John-Jameson/NUEDC_Topic](https://github.com/John-Jameson/NUEDC_Topic) |
| 本队项目仓库 | [github.com/yangran12/DianJiXZ](https://github.com/yangran12/DianJiXZ) |

---

> **最后更新**：2026-07-28
> **面向比赛**：2026 年全国大学生电子设计竞赛
