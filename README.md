# DianJiXZ

全国大学生电子设计竞赛控制类备赛仓库。以 WHEELTEC C07A / TI MSPM0G3507 差速车为底盘，沉淀巡线、姿态/里程计、视觉通信、任务状态机与赛题分析资料。

GitHub Pages 入口：从仓库 Pages 首页进入“学习笔记”“技术路线”和历年赛题；本地直接打开 [index.html](index.html) 也可浏览导航。

## 仓库内容

- `firmware/WHEELTEC_C07A_CAR/`：Keil5 固件工程副本，包含应用层源码、硬件驱动、Keil 工程文件，以及本工程编译需要的最小 TI DriverLib/CMSIS 依赖子集。
- `docs/巡线速度与逻辑评估_20260710.md`：几轮速度参数、处理逻辑和回退方案评估。
- `docs/JY62定位融合调研_20260711.md`：JY62惯性传感器接入、案例调研、安装和定位融合预研方案。
- `docs/build_20260710_高速三档当前版.log`：当前高速三档版在原工程中的增量构建记录。
- `docs/build_20260710_github_upload.log`：本仓库整理副本的完整构建记录。
- `archive/control_20260710_高速三档当前版.c`：当前实车测试推荐版 `control.c` 快照。
- `docs_learn/`：PWM/PID、巡线、运动学、DMA、JY62、K230、VOFA+ 与云台学习笔记。
- `workflow/`：当前硬件状态、技术路线与风险清单。
- `docs_2021/`、`docs_2024/`：智能送药小车 F 题、自动行驶小车 H 题及路线拆解。
- `docs_2026guess/`：2026 赛前技术预案、材料线索分析与历史题库入口；仅作备赛参考，不是官方赛题。

## 当前推荐版本

当前固件是“高速三档版”，重点放在简单电赛地图的速度和鲁棒性：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `GRAY_BASE_SPEED_MM_S` | `300.0f` | 中心线稳定时直线速度 |
| `GRAY_MID_SPEED_MM_S` | `240.0f` | 轻微偏移时中速 |
| `GRAY_CURVE_SPEED_MM_S` | `185.0f` | 大偏移/多灯时弯道速度 |
| `GRAY_CROSS_SPEED_MM_S` | `310.0f` | 十字/大面积黑线直冲速度 |
| `GRAY_STEER_GAIN` | `1.70f` | 转向增益 |
| `GRAY_MAX_ANGULAR_SPEED` | `3.60f` | 最大角速度限制 |
| `GRAY_LOST_TURN_SPEED` | `1.80f` | 全白丢线时原地找线角速度 |

主要逻辑：

1. 8 路灰度传感器按黑线位置计算横向偏差。
2. 中心附近高速，大偏移或多灯时降速，减少弯道冲出。
3. `black_count >= 7` 连续触发后判定十字/大面积黑线，短时间直冲通过，并设置冷却避免重复触发。
4. 8 路全白丢线时停止前进，根据上一次偏移趋势原地顺/逆时针旋转找回黑线。

## 编译方式

推荐用 Keil uVision5 打开：

```text
firmware/WHEELTEC_C07A_CAR/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
```

目标名：`MSPM0G3507_Project`

命令行构建示例：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'D:\EDC2026\github_upload\DianJiXZ\firmware\WHEELTEC_C07A_CAR\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx' -t 'MSPM0G3507_Project'
```

`tools/keil/syscfg.bat` 默认查找 `C:\ti\sysconfig_1.20.0\sysconfig_cli.bat`。如果本机 SysConfig 路径不同，需要改这个 bat。当前仓库已经包含生成好的 `ti_msp_dl_config.c/h`，即使预构建脚本因 `.metadata/product.json` 或路径问题提示警告，只要最终 Keil 输出 `0 Error(s)`，固件编译就是通过的。

本仓库整理副本已完成一次完整命令行构建，结果为 `0 Error(s), 0 Warning(s)`；hex 已正常生成。构建时仍可能看到 SysConfig 预构建脚本的 `.metadata/product.json` 提示，但它不会阻止已有生成配置下的 Keil 编译链接。

## 文件编码

WHEELTEC 原始 C/H 源码和部分 Keil 日志中有 GBK/ANSI 中文注释。为避免 GitHub 页面乱码，本仓库整理副本已把这些文本统一保存为 UTF-8；原始可跑工程目录未被改动。UTF-8 注释不影响 Keil/ARMCLANG 编译，仓库内已经用命令行构建验证。

## 说明

本仓库是调试和学习归档。TI DriverLib/CMSIS 文件保留 TI 原版权声明；WHEELTEC 示例代码保留原文件头和版权声明。公开分发前请确认原厂资料包授权要求。
