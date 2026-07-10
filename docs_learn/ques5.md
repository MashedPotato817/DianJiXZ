---
title: DMA配置与使用
---
# DMA 配置与使用 —— Q&A

> 基于 WHEELTEC C07A UART1 + DMA 蓝牙接收 实物代码分析

---

## Q1：DMA 是什么？为什么要用它？

### 没有 DMA 的情况

UART 每收到一个字节就触发中断 → CPU 暂停手头的事 → 进 ISR → 读寄存器 → 存到 buffer → 退出。9600bps 大约 1ms 来一个字节，CPU 每秒被打断 1000 次。

### 有 DMA 的情况

```
UART收到字节 → 硬件触发DMA → DMA自动: 读UART寄存器 → 写内存buffer → 指针+1
                                                              ↑ CPU完全不用管
```

CPU 只在 DMA 传完一整包数据后才处理一次。省掉了 99% 的中断开销。

### 类比

无 DMA = 快递小哥每个包裹都敲门让你签收  
有 DMA = 快递小哥把包裹放进快递柜，你空了去取

---

## Q2：当前项目 DMA 用在哪里？

**UART1（9600bps）接收蓝牙/APP 遥控数据。**

数据流：

```
手机蓝牙 → 蓝牙模块 → UART1 RX引脚
                          │
                    (每个字节)DMA自动搬运
                          │
                    gBTPacket[200] 缓冲区
                          │
                    BTBufferHandler() 主循环处理
                          │
                    bt_control() 逐字节解析命令
```

配置代码在 `ti_msp_dl_config.c:441-458`，运行时代码在 `uart_callback.c:25-32`。

---

## Q3：DMA 怎么配置？逐字段解释

### Step 1：初始化配置（硬件参数）

```c
static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,   // ①
    .extendedMode   = DL_DMA_NORMAL_MODE,             // ②
    .destIncrement  = DL_DMA_ADDR_INCREMENT,          // ③
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,          // ④
    .destWidth      = DL_DMA_WIDTH_BYTE,              // ⑤
    .srcWidth       = DL_DMA_WIDTH_BYTE,              // ⑥
    .trigger        = UART_1_INST_DMA_TRIGGER,        // ⑦
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,   // ⑧
};

DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID, &gDMA_CH0Config);
```

| 字段 | 值 | 含义 |
| ---- | -- | ---- |
| ① transferMode | SINGLE | 每次触发传 1 个数据单元，传完等下次触发 |
| ② extendedMode | NORMAL | 普通模式（非乒乓/非 scatter-gather） |
| ③ destIncrement | INCREMENT | 目标地址每传一个字节 +1，依次填 buffer |
| ④ srcIncrement | UNCHANGED | 源地址不变——永远是同一个 UART RXDATA 寄存器 |
| ⑤ destWidth | BYTE | 每次传 1 字节 |
| ⑥ srcWidth | BYTE | 每次读 1 字节 |
| ⑦ trigger | UART_1_DMA | 触发源：UART1 收到数据 |
| ⑧ triggerType | EXTERNAL | 外部硬件触发（UART 产生） |

**关键理解**：src 不变（读同一个硬件寄存器） + dest 自增（填 buffer），这是所有外设→内存 DMA 的标准写法。

### Step 2：运行时配置（动态参数）

初始化只配了"怎么传"，运行前还要告诉 DMA "从哪搬到哪、搬多少"：

```c
void BT_DAMConfig(void)
{
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);              // ①先停掉
    DL_DMA_setSrcAddr(DMA, CH0, (uint32_t)(&UART_1->RXDATA)); // ②源：UART接收寄存器
    DL_DMA_setDestAddr(DMA, CH0, (uint32_t)&gBTPacket[0]);    // ③目标：buffer首地址
    DL_DMA_setTransferSize(DMA, CH0, BT_PACKET_SIZE);          // ④总共传200字节
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);               // ⑤启动
}
```

### Step 3：UART 侧配合

DMA 只管搬数据，UART 需要配合开启 DMA 接收事件：

```c
// UART初始化时（ti_msp_dl_config.c:414-415）
DL_UART_Main_enableDMAReceiveEvent(UART_1_INST, DL_UART_DMA_INTERRUPT_RX);
```

---

## Q4：接收到的数据怎么处理？

### 整体逻辑

```
DMA 不停往 gBTPacket[200] 填字节
         │
         ▼
BTBufferHandler() 在主循环调用
  ├── 读 DMA 剩余传输大小 → 反算已收了多少字节
  ├── 有变化？→ 等 1ms 超时（连包判断）
  ├── 超时 → 逐字节调 bt_control() 解析
  └── buffer 过半？→ 重置 DMA，防止溢出
```

### 核心代码（`uart_callback.c:38-78`）

```c
void BTBufferHandler(void)
{
    // 已接收字节数 = 总数 - 剩余未传数
    uint8_t recvsize = BT_PACKET_SIZE - DL_DMA_getTransferSize(DMA, CH0);

    if (recvsize != lastSize) {          // 有新数据
        handleflag = 1;                   // 标记有待处理
        tick = Systick_getTick();         // 刷新时间戳
    } else if (/*超时1ms*/ && handleflag) {
        handleflag = 0;
        for (i = handleSize; i < recvsize; i++)
            bt_control(gBTPacket[i]);     // 逐字节解析
        handleSize = recvsize;

        if (recvsize >= 100) {            // 过半就重置
            BT_DAMConfig();               // DMA重头开始
            handleSize = 0;
        }
    }
    lastSize = recvsize;
}
```

### 两个巧妙设计

**超时检测（1ms）**：蓝牙发数据是连续的——如果 1ms 没收到新字节，认为"这一包发完了"，开始处理。避免逐字节处理破坏协议完整性。

**过半重置**：buffer 只有 200 字节。收到 ≥100 字节就重置 DMA（等效于清空 buffer、DMA 重新从地址 0 开始写），防止溢出。因为数据已经在重置前被 `bt_control()` 处理完了，丢掉也不怕。

---

## Q5：DMA 传完了会怎样？中断处理

```c
void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_IIDX_RX:                              // 普通接收中断
            gBTPacket[gBTCounts++] = DL_UART_Main_receiveData(...);
            break;
        case DL_UART_MAIN_IIDX_DMA_DONE_RX:                // DMA传完200字节
            BT_DAMConfig();                                 // 立即重新配置
            break;
    }
}
```

两个中断源：
- **RX 中断**：当前固件里 DMA 接管了接收，这个分支基本不触发（备用）
- **DMA_DONE_RX 中断**：DMA 传满 200 字节后触发，立即调 `BT_DAMConfig()` 重新配置，让 DMA 循环接收

注意：`BT_DAMConfig()` 也会在 `BTBufferHandler()` 过半时被调用——双重保障。

---

## Q6：如果要给新的外设加 DMA，怎么写？

假设要给 CCD 的 ADC 加 DMA（128 像素 × 每次 ADC 采样，自动存数组）。

### 模板

```c
// 1. 定义 buffer
volatile uint16_t gADCBuffer[128];

// 2. 初始化配置
static const DL_DMA_Config gDMA_CH1Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,     // 填buffer递增
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,      // 读同一个ADC结果寄存器
    .destWidth      = DL_DMA_WIDTH_HALF_WORD,     // 12位ADC结果占16bit
    .srcWidth       = DL_DMA_WIDTH_HALF_WORD,
    .trigger        = ADC12_INST_DMA_TRIGGER,     // ADC转换完成触发
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};
DL_DMA_initChannel(DMA, DMA_CH1_CHAN_ID, &gDMA_CH1Config);

// 3. 运行时配置
DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
DL_DMA_setSrcAddr(DMA, CH1, (uint32_t)(&ADC12_INST->MEMRES[0]));
DL_DMA_setDestAddr(DMA, CH1, (uint32_t)gADCBuffer);
DL_DMA_setTransferSize(DMA, CH1, 128);            // 128个像素
DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);

// 4. ADC侧配合
DL_ADC12_enableDMA(ADC12_INST);
```

### 通用公式

```
src = 外设数据寄存器地址（不变）
dst = 内存数组地址（递增）
size = 要传的数据个数
trigger = 外设的DMA触发源
```

---

## Q7：DMA 常见坑

| 坑 | 说明 |
| -- | ---- |
| **地址对齐** | 16bit 传输要求 dst/src 地址 2 字节对齐，32bit 要求 4 字节对齐。byte 模式无要求 |
| **DMA 和 CPU 抢总线** | MSPM0 是单总线，DMA 传输时 CPU 可能暂停一个周期。大数据量（如 ADC 高速连续采样）要注意 |
| **buffer 溢出** | DMA 只负责搬，不负责判断是否满了。必须像当前代码一样有"过半重置"或"环形 buffer"保护 |
| **重新配置前先停** | `DL_DMA_disableChannel()` → 改参数 → `DL_DMA_enableChannel()`，否则参数可能不生效 |
| **触发源要对** | UART 接收触发和 ADC 转换完成触发是不同的触发 ID，查 DriverLib 文档确认 |
