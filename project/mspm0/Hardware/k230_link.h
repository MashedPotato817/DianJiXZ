#ifndef H_K230_LINK
#define H_K230_LINK

#include <stdint.h>

/*
 * 固定安全帧：
 * $K230,BALL,<x10>,<valid>,<seq>,<edge>*<crc8>#
 * crc8 使用 CRC-8/ATM（poly=0x07, init=0x00），覆盖 '$' 后到 '*' 前的
 * ASCII 字节。x10 为毫米值乘 10，禁止使用无校验旧帧进入闭环。
 */
#define K230_LINK_TIMEOUT_MS 100U
#define K230_LINK_MAX_POSITION_X10 1500

/* K230 输出的小球位置：摆杆中心为 0，右侧为正，单位 mm。 */
typedef struct {
    float x_mm;
    float y_mm;
    uint8_t valid;
    int8_t edge_direction; /* -1 左侧离开视野，+1 右侧离开，0 非边缘或正常。 */
    uint32_t timestamp_ms; /* M0 收到并解析该帧的本地时刻。 */
} K230_BallPosition;

typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_overruns;
    uint32_t rx_hw_overruns;
    uint32_t valid_frames;
    uint32_t parse_errors;
    uint32_t crc_errors;
    uint32_t range_errors;
    uint32_t sequence_gaps;
    uint32_t last_seq;
    uint8_t timed_out;
} K230_LinkDiagnostics;

void K230_Link_Init(void);
/* 主循环调用：轮询 UART1 FIFO、解帧并维护超时。 */
void K230_Link_Process(void);
void K230_Link_UART1_IRQHandler(void);
/* 5 ms 定时中断调用：提供链路时间基准。 */
void K230_Link_Tick5ms(void);
void K230_Link_UpdatePosition(float x_mm, float y_mm, uint8_t valid,
                              int8_t edge_direction, uint32_t timestamp_ms);
void K230_Link_GetPosition(K230_BallPosition *position);
void K230_Link_GetDiagnostics(K230_LinkDiagnostics *diagnostics);
/* 最小联调 LED 指示：至少接收过一帧合法 BALL 数据时返回 1。 */
uint8_t K230_Link_HasValidFrame(void);
/* 最小联调 LED 指示：至少尝试过一次 UART1 发送时返回 1。 */
uint8_t K230_Link_HasTxAttempt(void);
/* UART1 PB6->PB7 回环测试：收到自身 HELLO 帧时返回 1。 */
uint8_t K230_Link_IsLocalLoopbackDetected(void);
/* 至少从 UART1 FIFO 读出过一个字节时返回 1，不要求帧格式正确。 */
uint8_t K230_Link_HasRxBytes(void);

#endif
