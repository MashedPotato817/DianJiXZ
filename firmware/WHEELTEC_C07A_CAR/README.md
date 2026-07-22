# WHEELTEC C07A 基础差速车工程

## 用途

基于 MSPM0G3507 与 WHEELTEC C07A 底盘的基础控制工程。它提供编码器速度闭环、TB6612 电机驱动、8 路灰度巡线、OLED 显示、按键/遥控与电池电压采样，可作为新的差速车赛题项目起点。

## 硬件范围

- MSPM0G3507 控制板
- TB6612 双 H 桥、电机与正交编码器
- 8 路灰度传感器、OLED、按键和电池电压采样

本工程的灰度逻辑按现有赛道的黑线条件编写。更换线色、传感器或赛道前，应先做原始采样和低速实车验证。

## 代码入口

- `empty.c`：初始化中断、OLED，并在主循环处理电压、蓝牙缓冲和显示。
- `Control/control.c`：5 ms 控制中断、灰度巡线、运动学和左右轮 PI。
- `Hardware/`：电机、编码器、灰度、OLED、按键等驱动。
- `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`：Keil 工程，目标 `MSPM0G3507_Project`。

## 构建

1. 用 Keil uVision5 打开 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
2. 选择目标 `MSPM0G3507_Project`，编译并下载。
3. SysConfig 生成的 `ti_msp_dl_config.c/h` 已随工程保存；如需改引脚或外设，使用 TI SysConfig 修改后重新生成，不手改生成文件。

## 已验证范围

本目录是基础底盘代码，不等同于任何特定赛题的完整方案。速度、巡线和实际赛道适应性必须以本次硬件测试记录为准；加入视觉、IMU、无线或任务状态机时，建议在新的独立项目目录中完成。
