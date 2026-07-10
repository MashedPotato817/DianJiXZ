# DianJiXZ — 电机小车

全国大学生电子设计竞赛（NUEDC）控制类备赛项目。基于 MSPM0G3507 的差速电机小车平台，集成灰度循迹、视觉识别、热成像感知等功能。

## 硬件

| 组件 | 型号 |
| ---- | ---- |
| 主控 | MSPM0G3507 (Cortex-M0+, 80MHz) |
| 电机驱动 | TB6612 |
| 编码器 | 双路正交编码器 |
| 显示 | OLED 128×64 SSD1306 |
| 传感器 | 8路灰度, K210 视觉, HTPA32 热成像 |
| 调试 | UART0 (115200bps), UART1 (9600bps) |

## 目录结构

```text
DianJiXZ/
├── demo/                         # 老师提供的参考代码 (WHEELTEC 底盘)
│   ├── empty.c                   # 入口
│   ├── ti_msp_dl_config.c/h      # SysConfig 生成的外设初始化
│   ├── Hardware/                 # 外设驱动 (motor/encoder/oled/key/led/printf/systick)
│   └── App/                      # 应用层 (control: PI速度控制, show: OLED显示)
├── 电赛控制类题目总结_2019-2025.md
└── demand.md                     # 需求文档
```

## 开发环境

- **IDE**: VS Code + EIDE (Embedded IDE) 插件
- **编译器**: Keil MDK v5 + ARMCLANG v6
- **SDK**: TI MSPM0 SDK v2.01.00.03
- **配置工具**: TI SysConfig（生成 `ti_msp_dl_config.c/h`）

## 参考资料

- [全国大学生电子设计竞赛培训网](https://www.nuedc-training.com.cn/)
- `电赛控制类题目总结_2019-2025.md` — 历年控制类题目分析与备赛建议
