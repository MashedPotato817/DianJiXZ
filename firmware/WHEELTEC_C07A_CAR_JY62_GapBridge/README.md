# WHEELTEC C07A JY62 / GapBridge 实验参考工程

## 状态

**实验/参考，未完成。** 本目录保留过往 JY62 姿态、断桥导航、路线状态机、K230 兼容链路和云台相关探索，用于查阅、迁移和后续 Git 历史回溯；不要直接将它作为已完成的 H 题或比赛工程下载。

当前 `empty.c` 的入口程序是 pitch 轴 PWM 往复演示，不会启动该目录中保留的巡线任务逻辑。这是后续恢复历史任务功能前必须先处理的入口问题。

## 参考内容

- `Control/control.c`：JY62 辅助的断桥/路线导航、任务模式和位姿相关逻辑。
- `Hardware/jy62.c/.h`：JY62 帧解析。
- `Hardware/k210_link.c/.h`：兼容 K230 的 UART 二进制帧与握手框架。
- `Hardware/gimbal.c/.h`：云台 PWM 驱动。
- `Control/show.c`：OLED 诊断显示。

## 构建入口

- Keil 工程：`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
- Keil 目标：`WHEELTEC_JY62_GapBridge`

可编译不表示任务状态机、JY62、K230 和云台能同时正确运行。恢复任一历史功能时，应先在 Git 历史中定位对应提交，确认当时的 `empty.c`、SysConfig 和 UART 分配，再复制到新的独立工程进行验证。

## 使用边界

1. 仅迁移已理解且与当前硬件匹配的模块，不直接覆盖基础底盘工程。
2. 云台 PWM、JY62、K230 和遥测模块可能争用定时器或串口，迁移前先做资源表。
3. 对本目录新增改动应标明验证条件和实际硬件结果。
