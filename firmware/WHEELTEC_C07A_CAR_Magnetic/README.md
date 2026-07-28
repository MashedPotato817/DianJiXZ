# WHEELTEC C07A CAR — 电磁铁吸取版

基于 WHEELTEC_C07A_CAR 巡线小车，增加**继电器+12V 电磁铁**吸取/释放功能。

## 新增模块

### Hardware/electromagnet.c/h — 继电器底层驱动
- `MAGNET_Init()` — 初始化 PA21 GPIO 输出，默认低电平（释放）
- `MAGNET_ON()` — GPIO 拉高 → 继电器吸合 → 电磁铁通电吸取
- `MAGNET_OFF()` — GPIO 拉低 → 继电器断开 → 电磁铁断电释放
- `MAGNET_Toggle()` — 状态翻转
- `MAGNET_GetState()` — 读取当前 IO 电平

**引脚分配：PA21（LQFP-64 pin 15，IOMUX PINCM46）**

### Control/magnetic_task.c/h — 吸取状态机
状态转换：

```
按键长按或蓝牙指令
      |
      v
  MAGNET_IDLE --(吸取指令)--> MAGNET_SUCK --(500ms超时)--> MAGNET_HOLD
       ^                                                        |
       |                                                        |
       +--- MAGNET_RELEASE <--(释放指令)----<--------------------+
              (300ms超时)
```

- `Magnetic_Task_Init()` — 初始化状态机，电磁铁默认释放（上电安全）
- `Magnetic_Task_Process()` — 主循环调用，处理状态超时
- `Magnetic_Task_Tick5ms()` — 5ms 定时器调用，递增计时基准
- `Magnetic_Command_Suck()` — 外部触发吸取
- `Magnetic_Command_Release()` — 外部触发释放
- `Magnetic_Command_Toggle()` — 翻转吸取/释放

## 操作方式

| 操作 | 方式 | 说明 |
|------|------|------|
| 吸取/释放切换 | 长按按键（>200ms） | 在 5ms ISR 的 Key() 中处理 |
| 蓝牙指令（待扩展） | UART1 接收 "MAGNET ON/OFF" | 可在 uart_callback.c 的 bt_control() 中扩展 |

## 硬件接线

```
MSPM0G3507 PA21 --- 继电器模块 IN
                   |
              5V   |   继电器 COM --- 12V 电源+
              GND  |   继电器 NO  --- 电磁铁+
                   |   电磁铁-   --- 12V 电源-
                   |
              （继电器模块需自备，常见 5V 低电平触发）
```

**注意：** 继电器模块通常需要 5V 供电，MSPM0 的 3.3V GPIO 可直接驱动大部分 5V 继电器模块的信号输入端（低电平触发型）。如使用高电平触发型，需加三极管/光耦驱动。

## 保留的原有功能

所有 WHEELTEC_C07A_CAR 原有功能完整保留：

- 差速电机闭环控制（TB6612 + 编码器 PI）
- 8 路灰度巡线（Pure Pursuit 曲率控制）
- 蓝牙 APP 遥控（UART1，9600 baud）
- OLED 显示 + DataScope 虚拟示波器
- CCD 线性 CCD 驱动（保留，未使用）
- 电池电压 ADC 检测

## 构建

用 Keil uVision5 打开 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。

> **Keil 工程需重新添加新增的 .c 文件到工程：**
> 1. 右键 Target → "Add Existing Files to Group"
> 2. 添加 `Hardware/electromagnet.c`
> 3. 添加 `Control/magnetic_task.c`

## SysConfig 重新生成（可选）

如需更换继电器控制引脚：

1. 在 Keil 中打开 `empty.syscfg`
2. 添加 GPIO 输出，命名为 "RELAY"
3. 分配目标引脚（如 PA21）
4. 保存，SysConfig 自动重新生成 `ti_msp_dl_config.c/h`

当前 `ti_msp_dl_config.h` 中已手动添加 RELAY 引脚定义（PA21, PINCM46），无需重新生成即可编译。
