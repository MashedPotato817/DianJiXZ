# JY62 In-The-Loop — 陀螺仪数据采集系统

JY62 陀螺仪 → ESP32-C3 → WiFi → PC 浏览器 → CSV 导出 → PID 调参

## 概述

一个独立的 JY62 陀螺仪数据采集工具，脱离小车底盘独立运行。ESP32-C3 读取传感器原始数据，通过 WiFi 实时传输到 PC 浏览器，支持录制导出 CSV 文件用于后续的系统辨识和 PID 参数整定。

**典型应用**：管子+钢球平衡控制系统中，采集管子摆动时的角度/角速度数据，分析系统动态特性，设计 PID 控制器参数。

---

## 硬件接线

```
JY62          ESP32-C3
VCC   ────    3.3V
GND   ────    GND
TX    ────    GPIO4 (RX)
RX    ────    GPIO5 (TX, 可选)
```

> JY62 出厂默认 115200 8N1，上电自动发送加速度/角速度/角度三帧数据。

---

## PC 端使用

### 第一步：烧录固件

```bash
# Windows CMD 中运行：
D:\esp\v5.5.4\esp-idf\export.bat
cd /d "D:\DianJiXZ\firmware\JY62 in the loop\esp32c3_jy62_bridge"
idf.py set-target esp32c3          # 仅首次
idf.py build
idf.py -p COMx flash monitor       # COM 号根据实际修改
```

### 第二步：连接并采集

1. PC 连接 WiFi 热点 `JY62_LOOP`，密码 `12345678`
2. 浏览器打开 `http://192.168.4.1/`
3. 观察实时折线图确认传感器在线
4. **按 ESP32-C3 的 BOOT 键** → 蓝灯常亮 → 开始录制
5. 操作设备（比如推动管子产生摆动）
6. **再按 BOOT 键** → 蓝灯熄灭 → CSV 自动下载

---

## CSV 数据格式说明

### 列定义

| 列名 | 单位 | 含义 | 备注 |
|------|------|------|------|
| `ts_ms` | ms | ESP32 毫秒时间戳 | 从上电开始计时，用于对齐多设备数据 |
| `roll_deg` | ° (度) | 滚转角 | 绕 X 轴旋转，JY62 水平放置时 ≈0 |
| `pitch_deg` | ° (度) | 俯仰角 | 绕 Y 轴旋转，JY62 水平放置时 ≈0 |
| `yaw_deg` | ° (度) | 偏航角 | 绕 Z 轴旋转，上电时归零，漂移约 1-3°/min |
| `wx_dps` | °/s | X 轴角速度 | 陀螺仪原始角速度，噪音小、延迟低 |
| `wy_dps` | °/s | Y 轴角速度 | 同上 |
| `wz_dps` | °/s | Z 轴角速度 | 同上 |
| `ax_g` | g | X 轴加速度 | 1g = 9.8m/s²，静止时 Z 轴 ≈1g |
| `ay_g` | g | Y 轴加速度 | 含重力分量 |
| `az_g` | g | Z 轴加速度 | 静止水平放置时 ≈1g |

### 实际数据示例

```
ts_ms,roll_deg,pitch_deg,yaw_deg,wx_dps,wy_dps,wz_dps,ax_g,ay_g,az_g
154493,-0.38,-1.59,-21.68,-0.73,0.12,36.25,0.019,0.045,0.999
154533,-0.31,-1.56,-20.21,-0.06,-0.37,36.74,0.03,0.037,1.01
154583,-0.23,-1.47,-18.54,-0.18,0.06,33.08,0.018,0.045,0.996
```

- 采样间隔约 10-50ms（取决于 JY62 数据帧速率，约 100Hz 每类帧，三帧合并后约 30-100Hz）
- `ts_ms` 为相对时间戳，跨 CSV 文件可通过 `ts_ms` 差值对齐
- `yaw_deg` 会缓慢漂移，长时间录制建议以 `wz_dps` 积分值或 pitch/roll 为主要参考

### 坐标系

JY62 水平放置时（芯片面朝上，引脚朝前）：
```
      X (前) → roll 轴
      Y (右) → pitch 轴
      Z (下) → yaw 轴
```

---

## Python 分析示例

### 读取 & 快速绘图

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('jy62_20260730_111340.csv')

# 角度
fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
axes[0].plot(df['ts_ms'], df['roll_deg'], label='Roll', color='#ff6b6b')
axes[0].plot(df['ts_ms'], df['pitch_deg'], label='Pitch', color='#ffd93d')
axes[0].plot(df['ts_ms'], df['yaw_deg'], label='Yaw', color='#6bcb77')
axes[0].set_ylabel('Angle (deg)')
axes[0].legend(); axes[0].grid(True)

# 角速度
axes[1].plot(df['ts_ms'], df['wx_dps'], label='Wx', color='#ff6b6b')
axes[1].plot(df['ts_ms'], df['wy_dps'], label='Wy', color='#ffd93d')
axes[1].plot(df['ts_ms'], df['wz_dps'], label='Wz', color='#6bcb77')
axes[1].set_ylabel('Angular Rate (deg/s)')
axes[1].set_xlabel('Time (ms)')
axes[1].legend(); axes[1].grid(True)

plt.tight_layout(); plt.show()
```

### 阶跃响应分析

```python
# 找到阶跃开始时刻（比如 Roll 突变 > 5°）
d_roll = df['roll_deg'].diff().abs()
step_idx = d_roll[d_roll > 5].index[0]  # 第一个突变点

# 截取阶跃后的数据
step_data = df.iloc[step_idx:step_idx+200]  # 取 200 个采样点
t = (step_data['ts_ms'] - step_data['ts_ms'].iloc[0]) / 1000  # 转为秒
y = step_data['roll_deg']

plt.plot(t, y)
plt.xlabel('Time (s)')
plt.ylabel('Roll Angle (deg)')
plt.title('Step Response')
plt.grid(True); plt.show()
```

### 频域分析

```python
from scipy import signal
import numpy as np

# 对 gyro 数据做频谱分析
fs = 1 / (df['ts_ms'].diff().median() / 1000)  # 估计采样率
f, Pxx = signal.welch(df['wx_dps'].dropna(), fs=fs, nperseg=1024)

plt.semilogy(f, Pxx)
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD')
plt.title('Gyro X Power Spectrum')
plt.grid(True); plt.show()
```

---

## PID 调参工作流

对于"管子+钢球"平衡控制系统的推荐流程：

### 步骤 1：开环数据采集

1. JY62 固定在管子上
2. 按 BOOT 键开始录制
3. **手动推动管子**，产生一个阶跃扰动（快速推一下松手）
4. 让管子自然摆动到停止
5. 按 BOOT 键停止 → 得到阶跃响应 CSV

### 步骤 2：系统辨识

用 Python 分析 CSV 中的阶跃响应：
- 测量**自然频率 ωₙ**（摆动频率）
- 测量**阻尼比 ζ**（振幅衰减速度）
- 拟合二阶传递函数：G(s) = K·ωₙ²/(s² + 2ζωₙs + ωₙ²)

### 步骤 3：PID 设计

根据辨识结果设计**双环控制器**：

| 环路 | 反馈量 | 执行器 | 带宽 |
|------|--------|--------|------|
| 内环（角度环） | roll/pitch + gyro | 舵机/电机 | 高（>10Hz） |
| 外环（位置环） | 钢球位置传感器 | 角度环给定值 | 低（1-3Hz） |

- 内环用 gyro 角速度做 D 项（提供阻尼，减小超调）
- 外环用球位置偏差做 P 项，输出为目标角度

### 步骤 4：闭环验证

将初步 PID 参数写入 MSPM0 / 控制固件，再次录制闭环阶跃响应，对比性能指标（上升时间、超调量、稳态误差），迭代优化。

---

## 文件命名规范

CSV 文件名格式：`jy62_YYYYMMDD_HHMMSS.csv`

```
jy62_20260730_111340.csv
      │        │
      └─ 日期   └─ 时间（按 BOOT 键开始录制的时刻）
```

建议按实验命名：
- `jy62_openloop_step_01.csv` — 开环阶跃响应第 1 次
- `jy62_closed_P3_I0_D5_02.csv` — P=3, I=0, D=5 闭环测试
- `jy62_disturbance_test.csv` — 抗扰动测试

---

## 项目文件结构

```
JY62 in the loop/
├── README.md                       # 本文件
├── esp32c3_jy62_bridge/            # ESP32-C3 固件 (ESP-IDF v5.5)
│   ├── main/main.c                 # 主程序
│   ├── main/jy62_protocol.h        # JY62 协议解析
│   ├── main/config.h               # 引脚/参数配置
│   └── main/index.html             # Web 仪表盘（构建时嵌入固件）
└── pc_display/                     # PC 端参考
    ├── index.html                  # 独立可打开的仪表盘副本
    └── README.md
```
