# 2026 TI 杯 H 题：车载平衡滚球运动控制系统

> 最后更新：2026-07-31 13:03

本工程以 `firmware/WHEELTEC_C07A_CAR` 为唯一硬件与构建底座，目标 MCU 为 MSPM0G3507。`firmware/Medicine_Cart` 只用于参考目录边界、驱动层与任务层的组织方式，不复制其业务逻辑或外设配置。

## 现阶段工程边界

- 已继承：Keil 工程、`empty.syscfg`、TI DriverLib、TB6612 双电机、双编码器、OLED、按键、电池采样、8 路灰度巡线、5 ms 控制中断。
- 新增模块：`Hardware/servo.*`、`Hardware/k230_link.*`、`Control/ball_control.*`、`Control/ball_task.*`、`Control/debug_telemetry.*`。
- 当前串口：UART0 PA10/PA11 接电脑调试助手；UART1 PB6/PB7 接 K230；均为 115200 8N1，内部回环关闭。
- `empty.syscfg` 已作为唯一配置源重新生成 `ti_msp_dl_config.c/h`。Keil 构建使用仓库内生成文件，不依赖预构建时自动运行 SysConfig。

## 模块职责

| 目录 | 模块 | 责任 |
| --- | --- | --- |
| `Hardware/` | `motor`、`encoder`、`oled` 等 | 已验证底盘外设驱动 |
| `Hardware/servo` | 摆杆舵机绝对角度命令、5~175°机械限幅和 PWM 映射 |
| `Hardware/k230_link` | 解析 K230 固定字段、CRC-8/ATM BALL 帧并维护链路诊断 |
| `Control/control` | 原有 5 ms 底盘速度闭环与灰度巡线 |
| `Control/ball_control` | `ACCEL→RUN→BRAKE→PASS→CAPTURE` 混合闭环、受控过零、速度估计与动态平衡角 |
| `Control/ball_task` | H 题任务状态：单圈、居中稳定、行驶保持与指定点保持 |
| `Control/debug_telemetry` | 每 50 ms 用 UART0 汇总 K230、编码器、舵机和链路诊断数据 |

## 当前关键接口

1. 舵机：PA8/TIMA0，50 Hz；机械允许5~175°。122.4°只作为上电初始种子（对应 PWM 1780 us；2026-07-31舵机重装后按试凑更新，此前为90°），运行中动态平衡角 `trim` 可在15~165°内慢速变化。
2. K230：IO40/UART1_TX → PB7/UART1_RX，IO41/UART1_RX ← PB6/UART1_TX，共地。
3. 电脑调试：PA10/UART0_TX → USB-TTL RX；如需下发命令再连接 PA11/UART0_RX ← USB-TTL TX；只使用 3.3 V TTL。
4. 小球控制是一维闭环；K230 坐标右正左负。当前仍允许未完成五点标定的临时位置参与联调，不能据此宣称位置精度达标。

UART0 遥测字段与采集方法见 [`../docs/UART0_AI调试遥测说明_20260730.md`](../docs/UART0_AI调试遥测说明_20260730.md)。

## 验证状态

截至 2026-07-31 18:00，单帧运动确认、脱困后卸力、中心非线性阻尼与受控过零 `PASS` 状态已完成实机验证（均在硬件修缮前）。硬件修缮期间舵机被重新安装，修缮后实测（`000009`/`000010`）均未能进入居中静止状态，且位置持续偏向一侧；据此不再依赖手测中位值精调，新增**自动扫掠标定**（新增 `Control/ball_calibrate.*`，按键长按触发，利用 K230 位置反馈二分定位摆杆平衡角并写入 `trim` 初值），同时放宽了 `trim` 在线学习窗口约2~3倍，使其能自行收敛残余误差。`servo.h` 中 `SERVO_ANGLE_NEUTRAL_DEG=122.4°`（对应 PWM 1780 us）保留为无标定时的回退默认值，不再人工精调。此外代码检查发现 `TIMER_0`（5 ms控制中断内的灰度巡线）与 `UART1` 中断优先级相同、互不能抢占，K230字节可能在灰度巡线忙等待期间被硬件RX FIFO溢出丢弃，且此前未被诊断计数捕捉；已提升 `UART1` 中断优先级并新增 `rx_hw_overrun` 诊断字段。以上改动均已通过 Keil 构建（`0 Error(s), 0 Warning(s)`），均尚待硬件验证。详见 [`../docs/球杆平衡闭环算法分析与改进方案_20260730.md`](../docs/球杆平衡闭环算法分析与改进方案_20260730.md) 第14~19节。

> 最后更新：2026-07-31 18:00
