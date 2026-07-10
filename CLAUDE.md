# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目定位

全国大学生电子设计竞赛（NUEDC）控制类备赛项目——基于 MSPM0G3507 的差速电机小车平台。仓库名 "DianJiXZ" = 电机小车。

## demo/ 目录说明

`demo/` 是老师提供的 MSPM0G3507 + TB6612 底盘参考代码（源自 WHEELTEC），**不是本项目代码**。项目实际代码将在后续开发中逐步加入。参考代码中可复用的部分：

- `ti_msp_dl_config.c/h` — SysConfig 生成的外设初始化模板
- `Hardware/motor.c/h` — TB6612 电机驱动
- `Hardware/encoder.c/h` — 编码器读取
- `App/control.c/h` — PI 速度控制算法

## 硬件平台

| 组件 | 型号/说明 |
| ---- | --------- |
| 主控 | MSPM0G3507 (Cortex-M0+, 80MHz) |
| 电机驱动 | TB6612 (双 H 桥) |
| 编码器 | 双路正交编码器 (13线, 2倍频, 减速比 28:1) |
| 显示 | OLED 128×64 SSD1306 (软件模拟 I2C) |
| 传感器(规划) | 8路灰度模块, K210 视觉, HTPA32 热成像 |
| 调试输出 | UART0 (115200 bps, printf), UART1 (9600 bps, DMA 接收) |

## 代码架构

```text
demo/
├── empty.c                  # 入口: main() → SYSCFG_DL_init() → user_init() → user_main()
├── ti_msp_dl_config.c/h     # ⚠️ SysConfig 自动生成，勿手动编辑
├── Hardware/                # 外设驱动层
│   ├── user.c/h             # 应用主逻辑: 初始化、主循环、10ms 定时器 ISR
│   ├── motor.c/h            # PWM 输出 + 方向控制 (Set_PWM, Velocity_A/B)
│   ├── encoder.c/h          # 正交编码器读取 (GPIO 中断, 4倍频判向)
│   ├── oled.c/h             # SSD1306 OLED 驱动 (软件模拟 SPI/I2C)
│   ├── key.c/h              # 按键消抖 + 单击/双击检测
│   ├── led.c/h              # LED 闪烁控制
│   ├── bsp_printf.c/h       # UART printf 重定向
│   └── bsp_systick.c/h      # SysTick 毫秒级时钟
└── App/                     # 应用算法层
    ├── control.c/h          # 增量式 PI 速度控制 + 编码器速度换算 + 低通滤波
    └── show.c/h             # OLED 显示界面
```

### 核心控制流

1. **10ms 定时器中断** (`TIMER_0_INST_IRQHandler` in `user.c:72`) 是控制核心：
   - 读取编码器累计值 → `Get_Velocity_From_Encoder()` 换算速度并低通滤波
   - 清零编码器计数器
   - 左右轮分别运行增量式 PI 控制器 → `Incremental_PI_Left/Right()`
   - `Set_PWM()` 输出 PWM + 方向信号到 TB6612

2. **GPIO 中断** (`GROUP1_IRQHandler` in `user.c:95`)：编码器 A/B 相上升沿触发，4倍频判向计数

3. **主循环** (`user_main`): 仅做 OLED 刷新 (10Hz) 和 printf 调试输出 (2Hz)

### 控制参数 (定义在 `control.h`)

- `Frequency = 100Hz` — 控制频率（与 10ms 中断对应）
- `Perimeter = 0.210m` — 轮子周长
- `SPEED_FILTER_ALPHA = 0.4` — 速度低通滤波系数
- `PI_DEADBAND = 0.005 m/s` — PI 死区，偏差小于此值不调节
- `PWM_MAX = 7800` — PWM 限幅 (周期 8000，即最大 97.5% 占空比)

## SysConfig 工作流

`ti_msp_dl_config.c/h` 由 TI SysConfig 工具根据 `.syscfg` 文件生成。修改外设配置（GPIO 引脚、时钟、PWM 参数等）时：

1. 在 SysConfig 中修改 `.syscfg` 文件
2. 重新生成 `ti_msp_dl_config.c/h`
3. **不要手动编辑** 这两个生成文件

## 比赛参考资料

`电赛控制类题目总结_2019-2025.md` — 2019-2025 年电赛控制类题目分析、评分标准、备赛建议和代码框架推荐。

`demand.md` — 待填充的需求文档。

## 项目记忆

项目记忆存放于 `.claude/projects/` 下对应的 session 目录。
