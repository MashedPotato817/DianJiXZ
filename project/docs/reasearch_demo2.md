# `firmware/demo2` 循迹实现调研

> 范围：仅阅读仓库当前 `firmware/demo2` 与 `project/mspm0` 源码、工程配置和源码契约测试。本文不代表 `demo2` 已在当前小车上实车验证，也不建议直接复制其全部策略。

> **状态说明（2026-07-29）：** 本文是借鉴思路的研究快照。当前巡线参数、全白行为和
> 实车结论以 [`../mspm0/README.md`](../mspm0/README.md) 与
> `../mspm0/Control/control.c/h` 为准；本文不能作为当前固件参数表。

## 1. 结论先行

`demo2` 是一套针对另一份硬件配置、以 **10 ms / 100 Hz** 为周期的完整巡线工程；它不是可直接套入 `project/mspm0` 的“更稳定算法”。

对当前“直线左右晃、行驶一顿一顿”的基础巡线，最有价值的是两条**思路**：

1. 对中心灰度组合显式输出左右同速，避免中心两路交替时反复施加差速；
2. 将“循迹目标是否抖动”和“速度闭环输出是否抖动”分开观测后再改参数。

当前 `project/mspm0` 通过 `GRAY_CENTER_SENSOR_MASK=0x18` 将通道 3/4 的
单灯或双灯组合显式置零，应先记录其边界处的左右目标/实际速度/PWM。
**不应**在没有这份证据前迁移 `demo2` 的 PD、IMU 航向、丢线斜坡、
路口保持、起步斜坡或电机卡滞保护。

## 2. `demo2` 结构与入口

| 层次 | 文件 | 代码证据 |
| --- | --- | --- |
| 程序入口 | `firmware/demo2/empty.c` | `SYSCFG_DL_init()` 后调用 `JY62_Init()`、`user_init()`，主循环进入 `user_main()`。 |
| 应用/巡线 | `firmware/demo2/Hardware/user.c` | 灰度扫描、`Track_Update_Target()`、按键启停、10 ms 中断均在同一文件。 |
| 速度闭环 | `firmware/demo2/App/control.c` | 编码器速度换算、一阶低通、增量 PI、起步 PWM 下限。 |
| 编码器 | `firmware/demo2/Hardware/encoder.c` | A/B 相 GPIO 上升沿中断计数并判断另一相方向。 |
| 电机输出 | `firmware/demo2/Hardware/motor.c` | `Set_PWM()` 分离方向 GPIO 与两个 PWM 通道。 |
| 工程 | `firmware/demo2/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` | 已列入 `empty.c`、`user.c`、`motor.c`、`encoder.c`、`control.c`。 |

## 3. 控制数据流（代码实现）

```text
10 ms TIMER_0_INST_IRQHandler
  -> GraySensor_ReadRaw(): AD2/AD1/AD0 选择 0..7，稳定等待后读 OUT
  -> Track_Update_Target(): 位掩码 -> 位置误差/状态 -> 左右轮目标速度
  -> Get_Velocity_From_Encoder(): 两轮计数 -> m/s -> 一阶低通
  -> Incremental_PI_Left/Right(): 目标与实测速度 -> PWM
  -> Set_PWM(): 方向 GPIO + TIMA PWM 输出

GPIO Group 1 中断
  -> Get_Encoder(): 正交 A/B 相上升沿计数
```

实车运行时，`user_main()` 还以 10 Hz 刷 OLED、以 2 Hz 打印 `GRAY`、两轮目标速度、PWM 与实测速度；JY62 在主循环中解析，巡线侧只在有有效角度帧时才可产生航向修正。

## 4. 关键符号和参数（仅代码事实）

| 类别 | `demo2` | 含义/备注 |
| --- | --- | --- |
| 控制周期 | `Frequency=100.0f`，`empty.syscfg` 的 `TIMER_0=10ms` | 速度换算和控制均按 100 Hz。 |
| 轮周长/轮距 | `0.2104867 m` / `0.1610 m` | 属于 demo2 底盘几何，不能当作当前车参数。 |
| 编码器分母 | `728 = 13 × 2 × 28` | 速度换算在 `control.c` 写死为 `728.0f`。 |
| 速度 PI | `KP=220`，`KI=130`，`KD=0` | 增量式 PI；有 `0.005 m/s` 死区、`0.4` 一阶低通。 |
| PWM | 硬件周期 `8000`，PI 限幅 `5200` | 与当前工程 `PWM_MAX=7800` 不同。 |
| 巡线巡航速度 | `0.240/0.212/0.178 m/s` | 分别为直线/中等偏差/大偏差。不是当前赛题达标速度证据。 |
| 中心判定 | 中间第 3、4 位的单点或双点 | 两者均显式输出相同的左右目标速度。 |
| 灰度读法 | `AD2/AD1/AD0 + OUT`，8 路扫描 | 每次选通后 `delay_cycles(80)`；极性为 `GRAY_BLACK_LEVEL=1`。 |

## 5. `demo2` 的循迹策略

1. 使用权重 `{-7,-5,-3,-1,1,3,5,7}` 对有效黑线通道求平均误差。
2. 中心双点、中心单点、以及 `|error| <= 2` 时直接左右同速；此处不保留差速命令。
3. 非中心时使用 `steer = Kp(error) + Kd(error - last_error)`，误差较大时降低基准速度，并用 `base_speed ± steer` 生成轮速目标。
4. 无黑线时先维持两拍直行，再沿最近一次有效转向方向，以低速差速恢复；恢复后还有若干拍过渡。
5. 有至少 4 个黑线通道时作为宽线/路口，保持直行若干拍。
6. 包含起步速度斜坡、IMU yaw 参考、PWM 起步下限、左轮卡滞保护等附加策略。

这是一套耦合状态机，不是只有“PD”一个改动；其中若干宏当前为 `0`，例如 `TRACK_CENTER_PAIR_BLEND_TICKS`、`TRACK_START_BALANCE_TICKS`，说明代码还保留了可选/试验分支。

## 6. 与 `project/mspm0` 当前基础巡线对比

| 项目 | `firmware/demo2` | `project/mspm0` 当前代码 | 判断 |
| --- | --- | --- | --- |
| 控制周期 | 10 ms / 100 Hz | 5 ms / 200 Hz | PI 与所有时间相关阈值不能照抄。 |
| 灰度表示 | 8 位位图 + 权重 | 8 路 mm 位置质心 | 当前实现更适合做物理中心偏移校准。 |
| 中心处理 | 中间单/双点与小误差同速 | 通道 3/4 单灯或双灯（`0x18` 范围内）置零 | 目标一致；当前仍需验证离开中心组合后的转向阶跃。 |
| 转向输出 | 直接设置左右轮目标速度差 | 质心 -> 曲率 -> `Move_Z` -> 差速逆解 | 不应混用两套转向公式。 |
| 速度估计 | 100 Hz、728、周长 0.2104867 m | 200 Hz、728、轮径 0.0768 m | 当前项目应以自己的轮径和控制频率为准。 |
| PI | 220/130，PWM 上限 5200，带起步下限 | 当前 PI 与 PWM 上限不同 | `demo2` 参数不可迁移。 |
| 电机/引脚 | 方向 GPIO + PB2/PB3 两 PWM | 当前工程的电机驱动与 SysConfig 不同 | 禁止复制 `motor.c` 或 SysConfig。 |

## 7. 可复用点（最小、可验证）

### 7.1 优先复用：中心状态下左右目标同速

`demo2` 的代码意图明确：中心单点/双点与近中心误差均清零并输出相同左右目标，
避免中间两路切换造成低频摆动。当前项目通过 `GRAY_CENTER_SENSOR_MASK=0x18`
实现中心 3/4 单灯或双灯同速，但实车直线摆动仍未解决。

下一步不是再加一层滤波或 D 项，而是静止记录 OLED 的 `P`：

- 物理居中时 `P` 稳定偏正/偏负：只校准 `GRAY_CENTER_OFFSET_MM`；
- 物理居中时 `P` 在两个离散值间跳：将死区覆盖该对跳变值，且保留边缘通道可产生转向；
- `P` 已进入死区而 `Z` 仍非零：检查其他调用是否覆盖 `Move_Z`，再改代码。

### 7.2 可复用：把“顿挫”定位为目标侧或执行侧

`demo2` 已有串口输出格式：灰度位图、两轮目标速度、PWM、两轮实测速度。当前工程也有对应变量 `Gray_Line_Pos_mm`、`Move_Z`、`MotorA/B.Target_Encoder`、`MotorA/B.Current_Encoder`、`MotorA/B.Motor_Pwm`。

因此先以 105 mm/s 当前档位采样：

| 观察 | 含义 | 下一步 |
| --- | --- | --- |
| `P/Z` 交替，左右目标速度随之交替 | 循迹量化/中心校准问题 | 仅校准偏移或死区。 |
| 左右目标相同，但实际速度或 PWM 周期性跳动 | 速度测量/PI/起步摩擦问题 | 先检查编码器计数和实际轮速，再只调一个 PI 参数。 |
| 目标、实测、PWM 都稳定，但车仍顿挫 | 电源、驱动、接线、轮胎或机械阻力 | 先查硬件，不加算法。 |

### 7.3 可借鉴但暂不迁移：速度 PI 观测手段

`demo2` 的低通、PI 死区、积分复位与 PWM 限幅都针对编码器量化和起停残留 PWM；它们是值得逐项验证的诊断方向。当前项目已具备低通、PI 死区、PWM 限幅，故不能以“demo2 也有”作为新增滤波或起步下限的理由。

## 8. 不可直接复用的内容

- `TRACK_PD_KP/KD` 与按误差分档降速：转向模型、灰度坐标和控制频率均不同，直接搬运会改变当前已能巡线的主链。
- JY62 航向修正：需要安装方向、帧率、零漂、弯道行为的实车数据；当前“直线晃/顿挫”尚未证明由航向误差造成。
- 丢线延时、搜线斜坡、路口宽线保持、转弯后状态机：这属于赛道特征处理，不是直线稳定的最小改动。
- `MOTOR_*_START/RUN_FLOOR_PWM` 和电机 A 卡滞保护：需基于当前电机、驱动和电源测得的起转阈值；固定下限可能放大顿挫。
- `Frequency=100`、`Perimeter=0.2104867`、`Wheelspacing=0.1610`、PI 220/130、PWM 5200：均为 demo2 专属标定。
- `empty.syscfg`、`motor.c`、`encoder.c`：当前 `demo2` 的引脚和电机驱动模型与 `project/mspm0` 不同。

## 9. 当前证据质量与未解决问题

### 已确认的代码证据

- `demo2` 具备 Keil 工程条目，但本次未编译、未下载、未上车。
- `demo2` 10 ms 中断包含灰度采样、循迹目标、编码器速度、速度 PI 和 PWM 输出的完整调用链。
- 当前 `project/mspm0` 5 ms 中断具备同类链路，但巡线转向模型为“质心 -> 曲率 -> 角速度”，不同于 demo2 的“权重误差 -> 左右速度差”。

### 发现的不一致（重要）

`firmware/demo2/tests/` 中 4 份源码契约测试与当前 `demo2` 文件不一致：测试期待四路 PWM、另一组编码器/灰度/JY62 引脚和旧循迹符号，而当前 `empty.syscfg`/`user.c` 仍是两 PWM + 方向 GPIO、另一组引脚和不同函数结构。本次运行 `python -m unittest discover -s firmware/demo2/tests -v` 的结果为 **18 项中 4 项失败、10 项错误**。

这只能证明测试基线已经陈旧或对应另一硬件版本；它**不能**证明当前 `demo2` 固件或实车循迹失败。报告中的硬件结论因此只以当前 `empty.syscfg`、生成头文件和源码为准。

同时，仓库内未找到 `firmware/demo2` 的提交历史记录，也未找到编译日志、示波器记录、视频或赛道实测数据。因此本文不把任何参数、算法或“稳定版”注释当成实车验证结果。

## 10. 面向当前 `project/mspm0` 的建议验证顺序

1. **不改代码，静态检查中心。** 将车置于直线黑线物理中心，连续记录 `P` 和 `Z`；再把黑线分别移动到中间两个通道及相邻通道，确认进入/离开死区的边界。
2. **不改 PI，检查目标是否对称。** 在 105 mm/s、直线段记录 `Move_Z` 与 `MotorA/B.Target_Encoder`；居中时应为 `Z=0` 且左右目标相同。
3. **再定位顿挫。** 同一时段同时看 `MotorA/B.Current_Encoder` 与 `MotorA/B.Motor_Pwm`：只有在目标已经稳定、实际速度或 PWM 仍摆动时，才进入速度环排查。
4. **一次只动一个参数。** 若中心偏差固定，先改 `GRAY_CENTER_OFFSET_MM`；若死区覆盖正确仍有目标侧跳变，再重新确定死区范围。不要同时改巡线增益、速度 PI、滤波和速度档。
5. **逐级提速。** 每档依次跑 1 m 直线、左右半圆、整圈，并记录是否晃动、是否顿挫、是否丢线和左右实际速度；达到当前档稳定后再提高 `GRAY_BASE_SPEED_MM_S`。

## 11. 证据路径

- `firmware/demo2/empty.c:39-49`：应用初始化与主循环入口。
- `firmware/demo2/Hardware/user.c:225-279`：8 路灰度选通和位图采集。
- `firmware/demo2/Hardware/user.c:355-605`：巡线状态、中心同速、PD 差速、分档速度。
- `firmware/demo2/Hardware/user.c:712-734`：10 ms 控制中断链路。
- `firmware/demo2/App/control.c:54-75, 161-201`：速度估计、滤波、PI、PWM 起步下限。
- `firmware/demo2/Hardware/encoder.c:12-67`：正交编码器计数。
- `firmware/demo2/Hardware/motor.c:20-50`：方向与 PWM 输出。
- `firmware/demo2/empty.syscfg:88-189`：编码器、灰度、PWM、10 ms 定时器及 UART 配置。
- `firmware/demo2/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx:388-470`：Keil 源文件清单。
- `project/mspm0/Control/control.c:64-160, 172-285`：当前 5 ms 灰度、速度与 PI 链路。
- `project/mspm0/Control/control.h:31-56`：当前底盘、速度环与灰度参数。
