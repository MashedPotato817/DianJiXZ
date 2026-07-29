# Servo_Demo — S20C 180° 舵机驱动

## 文件结构

```text
firmware/Servo_Demo/
├── servo.h                  # 舵机驱动头文件（含几何换算）
├── servo.c                  # 舵机驱动实现（TIMA0, 50Hz PWM）
├── servo_test.c             # 测试程序
├── ti_msp_dl_config.h       # DriverLib 配置（SysConfig 生成）
├── ti_msp_dl_config.c       # DriverLib 初始化
├── README.md
└── keil/
    ├── Servo_Demo.uvprojx   # Keil uVision5 工程 → 双击打开
    ├── Servo_Demo.uvoptx    # 工程选项
    ├── mspm0g3507.sct       # 链接脚本
    └── startup_mspm0g350x_uvision.s  # 启动文件
```

## 快速开始

1. Keil uVision5 打开 `keil/Servo_Demo.uvprojx`
2. **F7** 编译 → **F8** 烧录
3. 串口工具接 PB6/PB7（9600bps）查看输出

## 接线图

```text
┌───────────────────────────────────────────┐
│           MSPM0G3507 开发板                 │
│                                           │
│  PA8 ────→ 舵机信号线 (白/橙)              │
│  GND ──┬─→ 舵机地线 (棕/黑)               │
│        │                                  │
│  PB6 ──┼─→ USB转TTL RX (调试串口,9600)    │
│  PB7 ──┼─→ USB转TTL TX                   │
│        │                                  │
└────────┼──────────────────────────────────┘
         │
    ┌────┴─────────────────┐
    │  外接电源 5V~6V       │
    │  VCC(+) → 舵机红线    │
    │  GND(-) → 共地        │
    └──────────────────────┘

⚠ 舵机必须外接 5V~6V 供电！板载 3.3V 带不动 S20C（堵转 ~700mA）。
```

### 引脚速查

| MSPM0 引脚 | 连接 | 功能 |
| :--- | :--- | :--- |
| **PA8** | 舵机信号线 (白/橙) | TIMA0_CCP0, 50Hz PWM |
| **GND** | 舵机地 (棕/黑) + 电源地 | 共地 |
| **外接 5V** | 舵机电源线 (红) | 独立供电 |
| PB6 | USB转TTL RX | UART1 TX (调试) |
| PB7 | USB转TTL TX | UART1 RX |

## 机械几何

```text
        支撑平台（绕合页转动）
        ╱
       ╱  ← 平台倾角 = atan(h / 250mm)
      ╱
  ╔══╗──── 丝杆触点（推此处）
  ║  ║  ↑
  ║  ║  h = 舵机角度 × 25mm/90°
  ║  ║  ↓
  ╚══╝──○── 合页铰链
  ←── 250mm ──→
```

| 舵机角度 | 丝杆高度 | 平台倾角 |
| :--- | :--- | :--- |
| 0° | 0 mm | 0.00° |
| 45° | 12.5 mm | 2.86° |
| 90° | 25 mm | 5.71° |
| 135° | 37.5 mm | 8.53° |
| 180° | 50 mm | 11.31° |

## API

```c
/* 基础控制 */
void     Servo_Init(void);                    // 初始化, 默认 90° (中位)
void     Servo_SetAngle(uint8_t degree);      // 设置角度 0~180
void     Servo_SetAngleFloat(float degree);   // 浮点角度 (0.1° 精度)
void     Servo_SetPulseUs(uint16_t pulse_us); // 底层脉宽直控

/* 丝杆高度 */
void     Servo_SetHeight_mm(uint8_t mm);      // 按丝杆高度设置 (0mm=0°, 25mm=90°)

/* 平台倾角 (合页距丝杆触点 250mm) */
void     Servo_SetPlatformAngle(float deg);   // 目标平台倾角 → 自动反算舵机
float    Servo_GetPlatformAngle(void);        // 读取当前平台倾角

/* 查询 */
uint8_t  Servo_GetAngle(void);                // 读取当前舵机角度

/* 几何宏 */
SERVO_ANGLE_TO_MM(deg)         // 舵机角度 → 丝杆高度 (mm)
SERVO_MM_TO_ANGLE(mm)          // 丝杆高度 → 舵机角度
SERVO_PLATFORM_ANGLE_DEG(deg)  // 舵机角度 → 平台倾角 (°)
SERVO_ANGLE_FROM_PLATFORM_DEG(plat_deg)  // 平台倾角 → 舵机角度
```

## 移植

1. 将 `servo.c`、`servo.h` 加入工程
2. include path 能找到 `ti_msp_dl_config.h`
3. `main()` 中 `SYSCFG_DL_init()` 之后调用 `Servo_Init()`
4. 换引脚只改 `servo.c` 顶部四个宏：

```c
#define SERVO_PORT        GPIOA
#define SERVO_PIN         DL_GPIO_PIN_8
#define SERVO_IOMUX       IOMUX_PINCM19
#define SERVO_IOMUX_FUNC  IOMUX_PINCM19_PF_TIMA0_CCP0
```

| 备选引脚 | TIMA0 通道 | IOMUX |
| :--- | :--- | :--- |
| PA0 | CCP0 | `IOMUX_PINCM1` / `IOMUX_PINCM1_PF_TIMA0_CCP0` |
| PA1 | CCP1 | `IOMUX_PINCM2` / `IOMUX_PINCM2_PF_TIMA0_CCP1` |
| PA6 | CCP3 | `IOMUX_PINCM9` / `IOMUX_PINCM9_PF_TIMA0_CCP3` |
| PB4 | CCP2 | `IOMUX_PINCM17` / `IOMUX_PINCM17_PF_TIMA0_CCP2` |

## S20C 脉宽标定

如需更大行程，修改 `servo.h`：

```c
#define SERVO_PULSE_MIN_US  500U   // 0°   (原 600)
#define SERVO_PULSE_MAX_US  2500U  // 180° (原 2400)
```

合页距离调整：

```c
#define SERVO_HINGE_DISTANCE_MM  250.0f  // 改为你的实际距离
```
