# H题第三项「摆杆控制装置定点运动」算法调研 —— GitHub 开源实现

> 调研日期：2026-07-31
>
> 适用范围：H题第三项 —— 摆杆控制装置控制钢球从中心 O 移动到 +5cm、折返、再到 -5cm 并稳定；运行时间 ≤5s，±5cm 处最大误差绝对值 ≤1cm。
>
> 当前系统背景：摆杆（25cm、PPR 凹槽、球径约 1cm）一端铰链、一端舵机驱动；K230（OpenMV 兼容，cv_lite 霍夫圆）测小球毫米坐标经 UART 送 MSPM0；MSPM0 跑位置 PD（Kp=0.24°/mm、Kd=0.040°/s/mm，无积分）+ 状态机（ACCEL→RUN→BRAKE→PASS→CAPTURE），**无摆杆角度反馈内环**。
>
> 已知痛点：① 静摩擦小误差卡死；② +5→-5 过冲严重（实测 -64mm）；③ 收敛慢（数十次 ACC/PASS 反复）。

---

## ① 调研结论摘要（直接回答"第三项可行算法"）

1. **串级 PID 是球杆/滚球定点控制的标准且最实用方案，应作为首选**。结构为：**外环位置环（PD）→ 输出目标摆杆角度 → 内环角度环（P/PD）→ 执行器**。本地轮趣参考工程（位置环 50Hz Kp=0.2 Kd=0.2 Ki=0 + 角度环 100Hz Kp=8）就是这一结构；Lund 大学硕士论文、Doc1996 硕士论文（PD-PD 串级）、以及大量中文电赛资料均一致推荐。内环必须比外环快 3~5 倍（轮趣注释）或至少更快（Lund），否则两层打架震荡。**本系统当前缺的正是角度内环**。

2. **过冲抑制的第一功臣是位置环 D 项（球速反馈/速度阻尼），其次是输出限幅与积分抗饱和**。所有硬件实现都靠 D 项刹停：milind220 的 D 高达 1800（P=5.3）、Mohamed-Hawas 用 Kd=1.5 + D 项一阶低通（alpha=0.2）、control_BALL 的作者明确"球出界就把 D 调大"、轮趣位置环 Kd=0.2 且对 D 做了 2Hz 低通。当前系统 Kd=0.040°/s/mm 偏小，且速度估计未滤波，是过冲-64mm 的直接原因。

3. **静摩擦脱困的主流做法不是加大 Kp，而是"死区 + 受限起动力脉冲（kick/breakaway）"或"条件积分/最小有效输出"**，且配套输出死区（死区/最小频率/限幅）。本工程已有《静摩擦脱困策略调研_20260730.md》和 ACCEL 状态机，方向正确；开源参考中 control_BALL 用"换更重的球"，轮趣用"步进死区 5Hz + 最小运行频率 20Hz"，文献用"摩擦补偿窗口"——共同点都是**先大力起动、再回调、带冷却**，避免持续大倾角引发过冲。

4. **对本系统（舵机 + 无摆杆角度反馈）最实际的结构改进，是把"摆杆角度"这一环补上**：轮趣用角位移传感器（ADC）、Doc1996 用电机角度模型、Lund 用内环角度反馈。若短期不加传感器，则把舵机内部位置环当作角度内环，位置外环输出"目标舵机角度"而非直接 PWM，并对输出做死区+限幅——即把现有 PD 输出解释为"相对平衡角的舵机偏移"。

5. **5s 内完成 ±5cm 往返并稳定在 ±1cm 完全可行**。学术设计指标（CTM）为 settling <3s、overshoot <5%；实测开源系统在 20~50Hz 控制率、PD/串级下均能稳定定点。关键不是"算法复杂度"，而是：内环快、外环 D 项阻尼、目标角度限幅、以及"接近目标时收力+远离时给力"的分段策略。现有 ACCEL/RUN/BRAKE 状态机的"运动证据驱动"思路与文献一致，但**建议把状态机的输出改为"目标摆杆角度"（接入角度内环）或保留"目标舵机偏移角"并加限幅**。

---

## ② 仓库/项目清单表格

| # | 仓库 / 项目 | 传感器 | 执行器 | 控制结构 | 关键参数 | 对过冲/摩擦/快速定点的处理 |
|---|---|---|---|---|---|---|
| 1 | [Charmve/BallPlate](https://github.com/Charmve/BallPlate)（2017电赛B题板球，全国二等奖） | OV2640 摄像头 27~28fps，软件二值化提取质心 | MG996R 舵机 ×2，50Hz PWM | PD 单环（智能车常用） | PID 定时器中断 20ms（50Hz）；Kp/Kd 未公开 | 未专门处理；作者自评参数仍需调整 |
| 2 | [linaoct/GunQiuKongZhiXiTong](https://github.com/linaoct/GunQiuKongZhiXiTong)（2017电赛B题滚球控制） | OpenMV 视觉（openmv.py） | 舵机 | PID（细节在代码中） | 未公开 | 含 STM32 工程 + ESP8266 Web 控制端 |
| 3 | **轮趣 WHEELTEC 球杆平衡**（本地参考工程） | STP23L 激光测距（UART5）+ 角位移传感器（ADC2） | 步进电机（TIM8） | **串级：位置环 PD(50Hz) → 角度环 P(100Hz) → 步进频率** | 位置 Kp=0.2 / Ki=0 / Kd=0.2 / D滤2Hz / 积分限幅500；角度 Kp=8.0 / Ki=Kd=0；角度输出限±300ADC；步进死区5Hz / 最小20Hz / 最大1500Hz | 输出死区+最小运行频率防抖动；D 项低通；启动1s稳定期+PID复位；**与第三项架构最接近** |
| 4 | [milind220/ball-balance](https://github.com/milind220/ball-balance) | HC-SR04 超声波 | MG-996R 舵机 | PID 单环 | P=5.3 / I=0.07 / D=**1800**；约 50Hz（20ms） | **I 仅在距设定点很近时启用（门控）+ 幅值上限**（防 windup）；靠 D 项刹停，PD 组合"送回+停住"非常有效 |
| 5 | [Mohamed-Hawas/Balancing-Ball](https://github.com/Mohamed-Hawas/Balancing-Ball) | 超声波（0~32cm 有效，无效沿用上次值） | 舵机（限幅 −45°~+60°） | PID 单环 | Kp=3 / Ki=1 / Kd=1.5；D 项一阶低通 alpha=0.2；舵机命令平滑 alpha=0.3 | **积分钳位 anti-windup**（饱和时清积分）；LPF 抑制 D 项噪声；传感器失效保持 |
| 6 | [yusufborham/Balancing-a-ball-on-a-beam](https://github.com/yusufborham/Balancing-a-ball-on-a-beam) | VL53L1X ToF 测距 | 舵机 | PID 单环 | 参数未公开 | 声称调到"最小过冲+稳态误差"；含 Simulink 模型 |
| 7 | [Doc1996/ball-and-beam-system](https://github.com/Doc1996/ball-and-beam-system)（硕士论文） | 未指明（位置+角度两路反馈） | 电机（含静摩擦建模） | **PD-PD 串级** + EKF/低通估计器 | 未公开 | 电机静摩擦显式建模；最终采用"状态空间+EKF"最优 |
| 8 | [fardinabbasi/Ball_and_Beam_Control_System](https://github.com/fardinabbasi/Ball_and_Beam_Control_System)（德黑兰大学课程） | —（建模+线性化） | 舵机（模型） | PID / 状态反馈 / 观测器对比 | 未公开 | 学术对比：含摩擦、弹簧刚度、惯性矩的非线性模型 |
| 9 | [iman-sharifi-ghb/Ball-and-Beam-Control...](https://github.com/iman-sharifi-ghb/Ball-and-Beam-Control-using-State-Feedback-Controller-Observer-and-descrete-Controller) | —（MATLAB 仿真） | 伺服（模型） | **LQR + Servo Integrator/FeedForward** | LQR 增益（MATLAB 文件） | LQR 最优跟踪 + 观测器；纯仿真，未上硬件 |
| 10 | [control_BALL (1797058516)](https://github.com/1797058516/control_BALL) | OpenMV H7 视觉（识别黑板上的球） | 舵机（PWM） | PID 单环（放定时器中断） | 未公开 | 调参顺序 P→I→D；**换更重球克服摩擦**（乒乓球太轻，30° 重力分力<摩擦力）；出界调大 D |
| 11 | [mark8769/Ball-and-Beam-Balancing-using-PID](https://github.com/mark8769/Ball-and-Beam-Balancing-using-PID) | 超声波 | Futaba S3003 舵机 | PID 单环 | 4 个电位器在线调参 | 硬件/调参参考；单环结构 |
| 12 | [OpenJugglePlate](https://github.com/Zw-awa/OpenJugglePlate)（乒乓板球平衡+杂耍） | 单俯视摄像头 | 3 执行器并联平台 + 步进电机 | Linux SBC 高层 + STM32 实时（规划中） | 未公开 | 架构参考（视觉高层 + MCU 实时底层），尚未落地 |

---

## ③ 每个代表项目的详细分析

### 3.1 轮趣 WHEELTEC 球杆平衡（本地参考，最接近本系统）—— 见第 ④ 节专章

### 3.2 Charmve/BallPlate（2017 电赛 B 题板球，全国二等奖）

- **为什么值得看**：国内电赛滚球控制最著名的开源项目之一，赛题 B 题与 H 题第三项本质同类（小球从位置 1 精确到位置 2）。
- **方案**：STM32F407 + 正点原子 OV2640（27~28fps），软件二值化 + 极简质心提取定位；MG996R 舵机 ×2（PF9/PF7，50Hz PWM）驱动平板两轴。
- **控制**：PID 定时器中断 20ms（50Hz），采用"智能车常见的 PD 控制"，Kp/Kd 未公开（作者自评仍需调整）。
- **对本系统的启示**：50Hz 视觉外环 + 50Hz 控制率即可完成板球定点，说明**20ms 级别的控制周期对本任务足够**；PD 无积分即可定位（与当前系统一致）。但该作品是平板（两轴），无摆杆角度内环，性能上限低于串级方案。
- **不足**：未公开参数、无过冲/摩擦专门处理，参考价值主要在工程结构与赛题资料。

### 3.3 milind220/ball-balance（Arduino + 超声波 + 舵机）

- **控制**：P=5.3、I=0.07、**D=1800**，约 50Hz。作者纯手工调参：先 P 调出轻微震荡，再单独调 D 抵消球速，最后小 I。
- **过冲抑制（最有价值）**：D 项巨大，作者明确说 **"PD 组合在把物体送回设定点并让它停下来这方面非常有效"**——即 D 项=速度阻尼是刹车核心。当前系统 Kd=0.040 数量级远小于其比例，属于阻尼不足。
- **静摩擦/防 windup**：**I 项只在"物体距设定点非常近"时才参与**，并限幅——这就是条件积分/门控积分，避免远距离积分累积导致过冲。这一思路与本系统"中心附近才学习 trim"一致。
- **对本系统的启示**：把 I 项做"接近目标才启用"的门控，是一种低成本替代"无积分 PD"的方案；D 项应显著加大并配合低通滤波。

### 3.4 Mohamed-Hawas/Balancing-Ball（STM32F401 + 超声波 + 舵机）

- **控制**：Kp=3、Ki=1、Kd=1.5；带 D 项一阶低通（alpha=0.2）、舵机命令一阶平滑（alpha=0.3）。
- **过冲/饱和处理（最有价值）**：**积分钳位 anti-windup**——当"舵机角度与上一拍相同（已饱和）+ 误差与角度同号（同方向继续要求修正）+ 角度超限（±45°/60°）"三个条件同时成立时清零积分。这是教科书式条件积分实现。
- **传感器鲁棒性**：0~32cm 范围外读数无效并沿用上次值（与当前系统"丢帧保持"一致）。
- **对本系统的启示**：若未来引入积分，必须照此实现条件积分/钳位；D 项低通（等效截止频率几 Hz）直接可搬。

### 3.5 Doc1996/ball-and-beam-system（硕士论文项目）

- **控制**：设计了 **PD-PD 串级控制器**（外环位置 PD → 内环角度 PD）与状态空间控制器，并对比了"低通估计器 vs 扩展卡尔曼滤波 EKF"。最终选**状态空间 + EKF**，效果最好。
- **建模（最有价值）**：对**电机静摩擦显式建模**、对扰动补偿和参考跟踪做了分析。
- **对本系统的启示**：串级 PD-PD 是第三项的可行基线；若追求最优，可用状态反馈（LQR/极点配置）+ 状态估计。但工程上 PD-PD 串级已足够（见 3.2/3.3 硬件实例）。

### 3.6 fardinabbasi / iman-sharifi-ghb（学术仿真对比）

- **fardinabbasi**：德黑兰大学课程项目，覆盖非线性建模（含摩擦、弹簧刚度、惯性矩）、线性化、可控可观性、PID/状态反馈/观测器对比。属**理论设计参考**，无嵌入式代码。
- **iman-sharifi-ghb**：MATLAB，含 **LQR 控制器** + Servo Integrator/FeedForward 用于轨迹跟踪、以及离散化/观测器版本。属**仿真参考**：LQR 增益可移植到 C，但需角度反馈和状态估计，工程成本高。
- **对本系统的启示**：LQR 类方案在 5s/±1cm 指标下不是必需；串级 PD 更简单、可调、且已被硬件验证。

### 3.7 control_BALL（STM32C8T6 + OpenMV H7）

- **方案**：OpenMV H7 识别白板上的黑球；STM32F103C8T6 跑 PID（放定时器中断，不放串口中断——因为串口中断时间不固定、长线丢数据）；舵机 PWM 驱动。
- **摩擦处理（最有价值）**：作者明确换**更重的球**来克服静摩擦——"乒乓球太轻，板子倾斜 30° 的重力分力都大不过摩擦力"。这是硬件层面解决静摩擦的直接手段（本系统钢球≈1cm 直径已较重，但可检查凹槽/表面粗糙度）。
- **调参经验**：先单方向调参、先 P 调震荡、再 I 调抖动、最后 D 消抖动；**球出界说明刹车不足，把 D 调大**。
- **对本系统的启示**：佐证 D 项是刹车关键；"重球/低摩擦导轨"是静摩擦脱困的硬件捷径。

### 3.8 学术文献结论（过冲/静摩擦的通用方法）

- 球杆开环本质是**双重积分器**（marginally stable），直接位置 PD 易震荡，**D 项（速度反馈）提供系统阻尼是必备**。
- **静摩擦（stiction）** 导致稳态误差与极限环（limit cycle）；**PI 积分**可消除稳态误差（实测 kp=0.5, ki=1.5），但积分会与饱和冲突 → 必须 anti-windup。
- **输入饱和 → windup → 过冲**；Rockwell/Beckhoff 工业伺服做法是**摩擦补偿窗口**（误差窗口内按比例叠加补偿、窗口内禁用积分）——与本系统"死区+最小有效输出"同源。
- 设计指标参考：CTM 建议 **settling <3s、overshoot <5%**，与第三项 5s/±1cm 兼容。

---

## ④ 本地轮趣参考工程分析（`reference/球杆平衡控制系统/WHEELTEC_APP/RobotControl_task.c`）

### 4.1 硬件与信号流

| 环节 | 实现 | 说明 |
|---|---|---|
| 位置反馈 | **STP23L 激光测距**（UART5，FreeRTOS 任务解析，`g_readonly_distance`） | 200 字节/帧，中断只收数据、任务外解析；无效/越界帧沿用上次值 |
| 角度反馈 | **角位移传感器**（PA1 / ADC2，80 点平均缓冲） | 减零点 `ROLLBALL_ANGLE_ADC_ZERO=2470` |
| 执行器 | **步进电机**（PC6/PC8/PB14，TIM8 脉冲 + 方向线） | 指令为"带符号频率 Hz"；方向电平宏可配 |
| 主控 | STM32（FreeRTOS 任务） | 100Hz 任务 + STP23L 解析任务 |

### 4.2 串级 PID 结构（核心）

```text
目标小球位置(mm)
   │ 位置环 PD  —— 50Hz（每 2 轮跑 1 次，position_loop_divider=2）
   ▼
目标摆杆角度(ADC 偏差)  ← 输出限幅 ±300 ADC
   │ 角度环 P  —— 100Hz（每轮都跑）
   ▼
步进脉冲频率(Hz)  ← 死区 5Hz / 最小 20Hz / 最大 1500Hz
   ▼
步进电机 → 摆杆
   ▲                ▲
小球位置(激光)   杆角度(ADC)
```

### 4.3 PID 参数（`#define` 区，可直接对照）

| 环 | Kp | Ki | Kd | 输出限幅 | D 低通 | 控制率 |
|---|---|---|---|---|---|---|
| **位置外环** | **0.2** | **0.0** | **0.20** | ±300（ADC，即目标角度限幅） | **2 Hz** | **50 Hz** |
| **角度内环** | **8.0** | **0.0** | **0.0** | ±1500 Hz（步进频率） | 关 | **100 Hz** |

- 单位链：位置环输入 mm、输出 ADC 偏差；角度环输入 ADC 偏差、输出 Hz。**数值不可直接搬给舵机系统**，但**比值/结构可参考**。
- 关键注释原文：*"位置环 Kp=0.2 Kd=0.2 Ki=0：球杆系统靠 D 项（球速）提供阻尼，没有 D 项小球一定会来回震荡越来越大。Ki 设 0 是因为积分会让系统更不稳。"* —— 与本工程现有结论完全一致。

### 4.4 工程细节与可迁移点

1. **D 项低通**：`D_FILTER_HZ=2`，一阶 IIR（`alpha=ωdt/(1+ωdt)`），防止测距噪声被微分放大。
2. **输出护栏**：
   - 角度环输出（步进频率）**死区 5Hz** —— 指令太小直接停转，防高频正反抖动（对应本系统舵机小角度抖动的同类问题）；
   - **最小运行频率 20Hz** —— 步进电机极低频会"一步一步蹦"；
   - **最大频率 1500Hz** + 角度目标限幅 ±300 ADC —— 防机构撞限位。
3. **启动保护**：长按启动后 **1s 稳定期**（等 ADC 平均缓冲填满、激光收到帧），期间 PID 复位、电机停转；之后才闭环。每次启停、改目标都复位 PID 防"开机踢腿"。
4. **符号标定**：位置/角度/方向三套 `±1` 符号宏 + 模式化调试（0 纯监视 → 1 方向点动 → 2 角度环 → 3 串级），先验证符号再闭环。
5. **测量保持**：距离越界/无效时沿用上次有效值，控制器不收到跳变（与当前系统丢帧保持一致）。

### 4.5 对本系统的差距分析

| 对比项 | 轮趣参考 | 本系统现状 | 差距 |
|---|---|---|---|
| 角度内环 | 有（角位移传感器 ADC 反馈） | **无**（舵机内部位置环隐含） | 短期不可迁移（无角度传感器）；可用"舵机角度命令限幅"近似 |
| 位置环 D 项 | Kd/Kp 比值 = 0.2/0.2 = **1.0** | 0.040/0.24 = **0.167** | **D 阻尼明显偏弱**，是过冲 -64mm 的关键嫌疑 |
| D 项滤波 | 2Hz 一阶低通 | 速度估计未滤波（在改进方案中已建议 2~4Hz） | 需补滤波 |
| 输出死区/最小动作 | 5Hz/20Hz 步进护栏 | 1mm 位置死区 + 12°/16°/20° 状态机输出 | 方向对，幅度需实测标定 |
| 启动/切换保护 | 1s 稳定期 + PID 复位 | 有（ACC/丢帧冻结 trim） | 一致 |
| 调参顺序 | 0→1→2→3 分模式验证 | 单模式直接闭环 | 建议借鉴分步验证 |

---

## ⑤ 完整来源列表

### GitHub 仓库

1. https://github.com/Charmve/BallPlate — 2017 电赛 B 题板球控制系统（STM32F407+OV2640+MG996R，PD，国二）
2. https://github.com/linaoct/GunQiuKongZhiXiTong — 2017 电赛 B 题滚球控制系统（STM32 工程 + OpenMV + ESP8266 Web）
3. https://github.com/milind220/ball-balance — Arduino PID 球平衡（HC-SR04 + MG-996R，P=5.3/I=0.07/D=1800，I 门控）
4. https://github.com/Mohamed-Hawas/Balancing-Ball — STM32F401 球平衡（超声波+舵机，Kp=3/Ki=1/Kd=1.5，积分钳位，LPF）
5. https://github.com/yusufborham/Balancing-a-ball-on-a-beam — VL53L1X 测距 + 舵机 + PID（最小过冲）
6. https://github.com/Doc1996/ball-and-beam-system — 硕士论文（PD-PD 串级 + EKF，静摩擦建模）
7. https://github.com/fardinabbasi/Ball_and_Beam_Control_System — 德黑兰大学（PID/状态反馈/观测器对比）
8. https://github.com/iman-sharifi-ghb/Ball-and-Beam-Control-using-State-Feedback-Controller-Observer-and-descrete-Controller — LQR + Servo Integrator（MATLAB）
9. https://github.com/1797058516/control_BALL — STM32F103C8T6 + OpenMV H7 板球控制（PID 定时器中断、重球克服摩擦）
10. https://github.com/mark8769/Ball-and-Beam-Balancing-using-PID — 超声波 + Futaba S3003 + 单环 PID（4 电位器调参）
11. https://github.com/Zw-awa/OpenJugglePlate — 乒乓板球平衡/杂耍（视觉 + STM32 实时，规划中）
12. https://github.com/Jason-xy/Balance_ball — 横杆小球精确定位（README 未详，代码在 SDK/Doc 目录）

### 本地参考工程

- `C:\Users\Mashed Potato\Desktop\260427NUEDC\DianJiXZ\reference\球杆平衡控制系统\WHEELTEC_APP\RobotControl_task.c`（轮趣球杆平衡串级 PID 主控制任务）
- 配套：`WHEELTEC_BSP\bsp_rollball_stepper.c/h`（步进驱动）

### 网页/文献

- 2026 电赛 H 题深度拆解视频（PID 设计）：https://www.bilibili.com/video/BV1Pn326vEg6/
- 2026 电赛 H 题 4 秒摆杆控制：https://www.bilibili.com/video/BV1F13p6cEPg/
- 17 年电赛 B 题滚球控制系统——案例分享：https://www.bilibili.com/video/BV15f4y1t7SD/
- 串级 PID 算法在滚球控制系统中的应用（万方）：https://d.wanfangdata.com.cn/periodical/xddzjs202009029
- 球杆系统非线性控制研究与仿真（CSDN 文库）：https://wenku.csdn.net/doc/7nf9ce0on6
- 光电科技协会板球控制系统（CSDN）：https://blog.csdn.net/tama1204/article/details/77832975
- OpenMV 板球控制系统 PID 实战（CSDN）：https://blog.csdn.net/weixin_42520573/article/details/160200534
- OpenMV-based cricket control system（programmersought）：https://www.programmersought.com/article/96069815869/
- CTM Ball & Beam Modeling（控制理论经典，指标 settling<3s/overshoot<5%）：https://www.dii.unisi.it/~control/ctm/examples/ball/ball.html
- Rockwell 静摩擦补偿（工业摩擦补偿窗口）：https://literature.rockwellautomation.com/idc/groups/literature/documents/at/motion-at005_-en-p.pdf
- MathWorks Anti-Windup Control Using PID Controller Block：https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html

---

## 附：与本工程既有文档的衔接

- 《静摩擦脱困策略调研_20260730.md》（`project/research/`）：静摩擦的 kick/窗口/防 windup 详细分析与状态机建议，与本文 ①-3、3.8 结论互相印证。
- 《球杆平衡闭环算法分析与改进方案_20260730.md》（`project/preview/`）：已对本地轮趣参考工程做过一次串级结构分析，本文第 ④ 节在此基础上补充了完整参数表与差距量化。
