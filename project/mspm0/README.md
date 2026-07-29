# 2026 TI 杯 H 题：车载平衡滚球运动控制系统

本工程以 `firmware/WHEELTEC_C07A_CAR` 为唯一硬件与构建底座，目标 MCU 为 MSPM0G3507。`firmware/Medicine_Cart` 只用于参考目录边界、驱动层与任务层的组织方式，不复制其业务逻辑或外设配置。

## 现阶段工程边界

- 已继承：Keil 工程、`empty.syscfg`、TI DriverLib、TB6612 双电机、双编码器、OLED、按键、电池采样、8 路灰度巡线、5 ms 控制中断。
- 新增模块：`Hardware/servo.*`、`Hardware/k230_link.*`、`Control/ball_control.*`、`Control/ball_task.*`。
- 尚未改动：`empty.syscfg` 和 `ti_msp_dl_config.c/h`。这是有意保留的安全边界，避免在未知舵机信号引脚时误占用 UART、编码器或电机 PWM 资源。Keil 的自动 SysConfig 预构建命令已关闭：原脚本依赖未随项目提供的 SDK `product.json`，且不能正确处理含空格的本机路径；当前构建使用已复制的生成文件。

## 模块职责

| 目录 | 模块 | 责任 |
| --- | --- | --- |
| `Hardware/` | `motor`、`encoder`、`oled` 等 | 已验证底盘外设驱动 |
| `Hardware/servo` | 摆杆舵机角度命令、限幅、后续 PWM 映射 |
| `Hardware/k230_link` | 小球位置数据接口；待 K230 程序确认串口帧格式 |
| `Control/control` | 原有 5 ms 底盘速度闭环与灰度巡线 |
| `Control/ball_control` | 小球 x 方向位置闭环，输出摆杆目标角度 |
| `Control/ball_task` | H 题任务状态：单圈、居中稳定、行驶保持与指定点保持 |

## 必须确认后才能接入硬件闭环

1. 舵机型号、信号线接到 MSPM0 的具体引脚，以及其机械零位、有效转角、50 Hz 脉宽范围。
2. K230 与 MSPM0 的 UART 连接（当前底座 UART0 为 PA0/PA1，UART1 为 PB6/PB7），并确定小球坐标系、单位和数据帧格式。
3. 摆杆单自由度还是双自由度。当前 `ball_control` 仅预留沿杆方向的一维小球位置闭环。

确认引脚后，用 TI SysConfig GUI 打开 `empty.syscfg`，添加独立的 50 Hz TimerA PWM 通道和 K230 UART 配置，重新生成 `ti_msp_dl_config.c/h`；完成后再恢复 Keil 的 SysConfig 预构建流程。

## 验证状态

底盘底座来自既有工程；本次新增模块目前仅完成不依赖引脚的接口与控制骨架，尚未进行 Keil 编译、舵机输出、K230 串口接收或实车验证。
