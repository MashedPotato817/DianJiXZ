# MSPM0 送药车工程代码审查（第 1 轮）

| 项目 | 内容 |
| --- | --- |
| 审查人 | Codex |
| 审查时间 | 2026-07-25 21:02 |
| 审查范围 | `mspm0/`、`k230/det_uart_medicine.py`，截至提交 `768e533` |
| 结论 | 编译当前失败（21 errors）；先修复头文件依赖后，再处理 3 个实车联调前必须修复的问题。Task 3/4 的业务功能仍是明确标注的占位实现。 |

## 已确认的优点

- `mspm0/` 已从通用底盘独立，新增的 `medicine_task`、`route`、`line_follow`、`k230_link`、`load_detect` 和 `indicator` 的职责划分清楚。
- K230 和 MSPM0 的握手、心跳、`$...#` 帧定界、`RESULT` 帧格式一致；K230 端与主控端均设置了连续 3 次确认。
- UART1 的实际接线方向一致：K230 IO40/TX → MSPM0 PB7/RX，K230 IO41/RX → MSPM0 PB6/TX。
- 主循环承担 UART 解析与状态机，5ms 中断承担编码器、巡线、速度 PI 与 PWM，整体方向符合 `workflow.md`。

## 编译结果（阻塞）

已核对 `log.md`，并已使用 `C:\Keil\Keil_v5\UV4\UV4.exe` 实际重建 `mspm0/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。Keil 使用 ARM Compiler V6.21，结果为 **21 errors、0 warning，Target not created**；构建日志位于 `mspm0/keil/Objects/empty_LP_MSPM0G3507_nortos_keil.build_log.htm`。

### 0. 新模块头文件没有被入口和控制文件显式包含

**位置：** `empty.h`；`empty.c:22-110`；`Control/control.c:102-111`

编译日志中的 18 个 `empty.c` 错误和 3 个 `control.c` 错误均为未声明的类型、变量或函数：`Medicine_State`、`K230_*`、`Gray_Raw`、`Gray_Line_Pos_mm`、`Medicine_Task_*`、`Line_Follow_*`、`Route_*`、`Load_Detect_*`、`Indicator_Init`、`Key` 等。

根因是 `empty.h` 目前只包含 `board.h`，而 `empty.c` 实际使用了多个新模块接口；`control.c` 也没有显式包含 `k230_link.h`、`key.h`、`line_follow.h`。依赖 `board.h` 的间接包含已在前次循环依赖修复中移除，因此编译失败是必然结果。

**建议修复：**

- `empty.c` 直接包含其使用的 `medicine_task.h`、`k230_link.h`、`line_follow.h`、`route.h`、`load_detect.h`、`indicator.h`；或仅在 `empty.h` 中聚合这些应用层头文件，但避免让底层 `board.h` 反向依赖应用模块。
- `control.c` 直接包含 `k230_link.h`、`key.h`、`line_follow.h`，不要依赖其他头文件的偶然传递。
- 修复后重新构建；在编译通过前，后续静态审查结论不代表可下载运行。

## 必须修复

### 1. 复位或断链后，旧病房号仍会自动生效

**位置：** `Hardware/k230_link.c:118-134, 240-248`；`Control/medicine_task.c:97-102, 149-151`

K230 的 `g_latestResult.confirmed` 在目标锁定后从未清除；链路超时仅清除握手标志，`Medicine_Task_Reset()` 也不会清除该目标。因此完成一次任务后复位，或 K230 已离线时复位，`WAIT_TARGET` 会立即读取旧目标并跳转到 `WAIT_LOAD`。

这与 Task 2“通信超时后拒绝启动”的完成判据冲突，可能导致小车按上一轮病房号运行。

**建议修复：**

- 在 `K230_Link` 增加明确的 `K230_Clear_Target()`；在任务复位、链路超时和进入 `WAIT_TARGET` 时调用。
- `WAIT_TARGET` 的跳转条件改为“`K230_Is_Online()` 且存在本轮确认后的目标”。
- 为锁定结果增加会话号或时间戳有效期，防止复位前缓存被误用。

### 2. UART1 的 SysConfig 与运行时代码不一致

**位置：** `empty.syscfg:185-196`；`ti_msp_dl_config.h:135`；`ti_msp_dl_config.c:405`；`Hardware/k230_link.c:167-170`

`empty.syscfg` 仍启用 UART1 的 DMA RX 和 `DMA_DONE_RX` 中断；生成头文件和初始化代码仍显示/配置为 9600。随后 `K230_Link_Init()` 才在运行时手动关闭 DMA/中断并改为 115200。

这会造成配置源、生成文件与实际运行三者不一致；下次从 SysConfig 重新生成后，运行行为可能回退，也违反项目“不要手动编辑 SysConfig 生成文件”的约定。

**建议修复：** 仅通过 TI SysConfig GUI 修改 `empty.syscfg`：UART1 设为 115200、关闭 RX DMA 和 DMA 完成中断，然后重新生成 `ti_msp_dl_config.c/h`。之后删除 `K230_Link_Init()` 中重复覆盖 UART1 配置的代码，或只保留运行时 FIFO 清空。

### 3. 停车后速度 PI 的历史 PWM 未被清零，重新起步可能突跳

**位置：** `Control/control.c:59-90`

两个 `Incremental_PI_*()` 函数先检查 `PI_DEADBAND`，若误差较小就直接返回历史 `Pwm`；停车时的清零逻辑在该返回之后。任务状态将 `Flag_Stop` 从 1 切回 0 时，可能把停车前残留的历史 PWM 直接输出，造成起步突跳。

**建议修复：** 在两个 PI 函数的最前面优先处理 `Flag_Stop`，清零 `Pwm/Bias/Last_bias` 后返回 0；或提供 `Control_Reset_PI()`，并在每次进入停车状态和出发状态时调用。

## 应在下一轮实现前处理

### 4. 定时器中断条件写成常量判断

**位置：** `Control/control.c:96-99`

`if (DL_TIMER_IIDX_ZERO)` 判断的是枚举常量，恒为真；实际读取到的中断索引没有保存并比较。当前 SysConfig 仅启用 ZERO 事件时通常不显现，但一旦增加其他 TimerG 中断，控制环会在非 ZERO 事件上运行。

**建议修复：**

```c
if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
    /* 5ms 控制环 */
}
```

并使用与 `TIMG0` 对应的 `DL_TimerG_*` API，避免 TimerA/TimerG 混用。

### 5. 状态机接口尚未真正控制巡线和路线

**位置：** `Control/control.c:111`；`Control/medicine_task.c:104-142`；`Control/route.c`

当前 5ms 中断无条件调用 `Line_Follow_Run()`，未使用 `Medicine_Task_AllowMove()`；`Route_SetTarget()` 从未调用，路线表和到位判据均返回 0。这意味着当前工程可作为通信/底盘骨架，但尚不能完成任何一个基础赛题闭环。

这不是骨架阶段的错误，但在开始实车前应明确：Task 3、Task 4 未完成前，禁止将 `Flag_Stop` 默认放开或把该版本当作可运行送药任务。

### 6. 红绿指示灯和装卸检测仍是空实现

**位置：** `Hardware/indicator.c`；`Hardware/load_detect.c`；`Control/medicine_task.c:69-86`

`indicator` 函数为空，状态机中也仍是 TODO；`Load_Detect_IsLoaded()` 和 `Load_Detect_IsUnloaded()` 固定返回 0。基础赛题要求的红灯等待、绿灯完成和自动装卸触发尚未实现。

**建议：** 保持 `Flag_Stop=1` 的安全默认值，待 Task 3 的硬件和 GPIO 分配确定后再接入，禁止以常量返回值临时绕过。

## 验证情况与限制

- 已完成：代码静态审查、K230 端/主控端协议字段核对、SysConfig 源与生成文件核对、Keil 工程文件列表核对、Keil 编译日志核对。
- 编译状态：已使用 `C:\Keil\Keil_v5\UV4\UV4.exe` 复跑，仍为 21 errors，目标文件未生成。
- 尚未验证：红线传感器响应、轮向/通道映射、UART 物理链路、装卸传感器、路口和门口到位判据。

## 建议修复顺序

1. 用 SysConfig GUI 修复 UART1 配置并重新生成文件。
2. 修复旧目标缓存与 PI 停车复位，确保上电、复位、断链均保持停车。
3. 修复 TimerG 中断索引判断。
4. 按 `task1.md` 实测红线后，才调整 `line_follow.c` 的电平和转向符号。
5. 按 `task2.md` 做 UART 实机联调；通过后再接入 Task 3、Task 4 的硬件与路线逻辑。
