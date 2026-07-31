# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 语言偏好

### 规则

所有与用户的交流、生成的文档、代码注释、回答等，默认使用**简体中文**。除非用户明确要求使用其他语言。

### 表达风格

- 不过度口语化，也不过度堆砌专业术语；
- 涉及专业术语时，用通俗易懂的方式解释清楚，确保用户能听懂（先讲清含义、再引用术语本身）。

### 范围

- 所有代码注释和文档字符串
- 所有回答和解释
- 所有生成的文档文件
- 所有提交信息

用户使用英文提问时，也默认优先使用中文回答。涉及文档、注释等产出物同样使用中文。

## 项目定位

全国大学生电子设计竞赛（NUEDC）控制类备赛项目——基于 MSPM0G3507 的差速电机小车平台。仓库名 "DianJiXZ" = 电机小车。

## 项目结构

当前 H 题工作区在 `project/`（仓库根 `firmware/` 下为早期 WHEELTEC 基线工程）：

```text
project/
├── README.md              # 项目入口与协作约定
├── topic/                 # 赛题原题 + software_workflow
├── docs/                  # 接线、巡线验证计划、遥测说明
├── research/              # 调研类文档（静摩擦调研、算法调研）
├── preview/               # 审查/算法分析/评估类文档
├── mspm0/                 # MSPM0G3507 主控工程
│   ├── Control/           # ball_task/ball_control/ball_calibrate/control/show/debug_telemetry
│   ├── Hardware/          # k230_link/servo/calib_store/motor/encoder/oled/key/board
│   ├── keil/              # Keil uVision5 工程文件
│   └── source/            # TI DriverLib + CMSIS (subset)
└── k230/                  # K230 小球视觉、标定、协议与日志
```

## 构建

用 Keil uVision5 打开 `project/mspm0/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`，目标名 `MSPM0G3507_Project`。

命令行构建：

```powershell
& 'C:\Keil_MDK\UV4\UV4.exe' -b '...\project\mspm0\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx' -t 'MSPM0G3507_Project'
```

见 `project/README.md` 中的完整构建说明和当前参数表。

## 比赛参考资料

`电赛控制类题目总结_2019-2025.md` — 2019-2025 年电赛控制类题目分析、评分标准、备赛建议和代码框架推荐。

`demand.md` — 待填充的需求文档。

## 协作仓库

队友：`yangran` [https://github.com/yangran12/DianJiXZ](https://github.com/yangran12/DianJiXZ)

```bash
git remote add yangran https://github.com/yangran12/DianJiXZ.git
git fetch yangran
git merge yangran/main
```

队友贡献了 `k210_link.c/h`（K210/K230 通信链路）、**ESP32-S3 黑匣子遥测**、陀螺仪里程计、H 题任务状态机等。关键文件：

| 文件 | 内容 |
|------|------|
| `firmware/.../Hardware/k210_link.c/h` | MSPM0 端通信——**二进制协议**（0xAA 头+8字节 XOR）和**ASCII 握手**（$CAR,HELLO# / $K210,OK#）。K230 可复用此协议框架 |
| `firmware/ESP32S3_Blackbox/` | ESP32-S3 遥测黑匣子 |

视觉芯片用 **UART_0**（和 JY62/蓝牙的 UART_1 分开）。

## 视觉芯片选型

**已选定 K230（CanMV），放弃 K210。** K210 工具链老旧、MicroPython 支持差、通信未调通。K230 CanMV 固件兼容 OpenMV API（`find_blobs()` 等函数直接可用），¥188~249，淘宝搜"立创庐山派 K230"或"CanMV K230"。详见 `workflow/`。

## 编码规范

编辑 `.c` / `.h` / `.js` / `.html` / `.py` 文件前，**必须先调用** `andrej-karpathy-skills:karpathy-guidelines`，遵循其简洁、手术式修改、不过度设计的原则。

## 项目记忆

项目记忆存放于 `.claude/projects/` 下对应的 session 目录。
