#ifndef H_K230_LINK
#define H_K230_LINK

#include <stdint.h>

/* UART1 最小联调协议：$K230,BALL,<x_mm>,<valid>,<seq># */
#define K230_LINK_TIMEOUT_MS 100U

/* K230 输出的小球位置：摆杆中心为 0，右侧为正，单位 mm。 */
typedef struct {
    float x_mm;
    float y_mm;
    uint8_t valid;
    uint32_t timestamp_ms;
} K230_BallPosition;

typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_overruns;
    uint32_t valid_frames;
    uint32_t parse_errors;
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
                              uint32_t timestamp_ms);
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
