# HC-06 Bluetooth 双向通讯测试

基于 MSPM0G3507 (WHEELTEC C07A) 开发板验证 HC-06 蓝牙模块收发功能。

## 硬件接线

| HC-06 引脚 | 开发板引脚 | 说明 |
|-----------|-----------|------|
| VCC | 3.3V (或 5V) | 按你的模块版本接 |
| GND | GND | 共地 |
| **TXD** | **PB7** (UART1 RX) | HC-06 发 → MSPM0 收 |
| **RXD** | **PB6** (UART1 TX) | HC-06 收 ← MSPM0 发 |

> 接线时 TXD→RX、RXD→TX，**交叉连接**。

## 模块状态确认

- 上电后 HC-06 **快闪** = 等待配对
- 手机/电脑连上后 HC-06 **慢闪**（或常亮） = 已连接
- HC-06 默认波特率 **9600**，与代码一致，无需 AT 配置

## 编译 & 烧录

1. 双击 `keil\HC06_Test.uvprojx` 用 Keil uVision5 打开
2. **F7** 编译（应 0 Error 0 Warning）
3. **F8** 烧录到开发板

## 验证双向通讯（手机蓝牙调试助手）

烧录后程序就开始运行（LED 每 ~3s 闪一次）。

### Step 1 — 手机配对 HC-06

手机设置 → 蓝牙 → 添加设备 → 找到 `HC-06` → 配对，密码 `1234`。

### Step 2 — 打开蓝牙调试助手连接 HC-06

打开手机蓝牙调试助手 APP → 选择 HC-06 → 连接。

### Step 3 — 验证命令

连接后应能看到启动 Banner 和心跳消息：

```
====================================
  HC-06 Bluetooth Test v1.0
  UART1 @ 9600 bps 8N1
  TX:PB6  RX:PB7
====================================
 Type HELP for commands.

[1] HC06 Alive
[2] HC06 Alive
```

收到心跳消息 = **MCU→HC06 方向 OK** ✅

然后测试以下命令（命令大小写不敏感，如 `ping`、`PING`、`Ping` 均可）：

| 命令 | 响应 | 说明 |
|------|------|------|
| `PING` | `PONG 12345` | 连接测试（返回运行时间 ms） |
| `LED ON` / `LED OFF` | `LED: ON` / `LED: OFF` | 控制 PB9 LED |
| `LED` | `LED: ON`（查询） | 查询 LED 状态 |
| `ECHO ON` / `ECHO OFF` | `ECHO: ON` / `ECHO: OFF` | 开关 Echo 模式 |
| `hello world` | `hello world` | Echo 回显（仅 ECHO ON 时） |
| `AT+VERSION` | (HC06 版本信息) | AT 指令透传，配置 HC06 |
| `VER` | `HC06 Test v1.0 MSPM0G3507` | 固件版本 |
| `HELP` 或 `?` | (命令列表) | 查看所有命令 |

发送 `hello` → 回显 `hello` = **HC06→MCU 方向 OK** ✅

### AT 透传说明

所有以 `AT` 开头的命令会原样发给 HC06，可用来配置模块参数：

| AT 指令 | 功能 |
|---------|------|
| `AT+VERSION` | 查询 HC06 固件版本 |
| `AT+NAMEnewname` | 修改蓝牙名称 |
| `AT+PIN1234` | 修改配对密码 |
| `AT+BAUD8` | 修改波特率为 115200 |

> HC06 的 AT 响应会直接回显到手机，Echo 关闭时也会显示。

### PC Python 终端（备选验证方式）

#### Step 1 — PC 配对 HC-06

Windows 设置 → 蓝牙和设备 → 添加设备 → 蓝牙 → 找到 `HC-06` → 配对，密码 `1234`

#### Step 2 — 找到 COM 口

设备管理器 → 端口 (COM 和 LPT) → 找到 `Standard Serial over Bluetooth link`，记住 COM 号（如 `COM4`）

#### Step 3 — 运行终端

```bash
python firmware/HC06/hc06_terminal.py COM4
```

不带参数运行会列出所有可用 COM 口让你选：
```bash
python firmware/HC06/hc06_terminal.py
```

输入 `:quit` 退出。

### 手机 Chrome 浏览器（实验性）

> HC-06 是经典蓝牙（BR/EDR），不是 BLE。Chrome Web Bluetooth 对经典蓝牙的支持取决于系统版本，可能搜不到设备。搜不到就用蓝牙调试助手 APP 或方法一。

手机 Chrome 打开 `bt_terminal.html`（可以先发到手机上，或者用 `python -m http.server` 开个本地服务器）。

## 故障排查

| 现象 | 可能原因 |
|------|----------|
| PC/手机搜不到 HC-06 | 检查供电、模块是否快闪 |
| PC 配对成功但 Python 连不上 | 确认 COM 口号（设备管理器里看） |
| 手机 APP 连上但无数据显示 | 确认接线交叉、APP 波特率 9600 |
| 收到乱码 | 波特率不是 9600 |
| 有心跳但 Echo 不回 | 检查 PB7 接线（HC-06 TXD→PB7），或用 `ECHO` 命令确认 Echo 开关状态 |
| 命令无响应 | 发送时末尾要带换行（蓝牙调试助手通常自带） |
| AT 透传无响应 | 确认 HC-06 支持 AT 模式（部分老模块不支持），试试 `AT` 不带参数 |
| Keil 编译报错 | 确认 Pack `TexasInstruments.MSPM0G1X0X_G3X0X_DFP` 已安装 |

## 项目文件

```
firmware/HC06/
├── main.c                  # 测试固件（环形缓冲 + 命令解析 + 心跳）
├── hc06_terminal.py        # PC Python 蓝牙串口终端
├── bt_terminal.html        # 手机 Chrome Web Bluetooth 终端（实验性）
├── README.md
└── keil/
    ├── HC06_Test.uvprojx   # Keil 工程
    ├── HC06_Test.uvoptx
    ├── startup_mspm0g350x_uvision.s
    └── mspm0g3507.sct
```
