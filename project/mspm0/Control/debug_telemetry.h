#ifndef H_DEBUG_TELEMETRY
#define H_DEBUG_TELEMETRY

#include <stdint.h>

/* UART0 调试遥测周期与 K230 视觉发送周期一致。 */
#define DEBUG_TELEMETRY_PERIOD_MS 50U
/*
 * 当前工程未确认底盘双轮编码器接入球杆机构。保留原始计数供排查干扰，
 * 但显式标记为无效，禁止把悬空输入误判为杆角反馈。
 */
#define DEBUG_WHEEL_ENCODERS_CONNECTED 0U

void Debug_Telemetry_Init(void);
/* 5 ms 定时中断调用：累计与该时间窗口对应的两路编码器增量。 */
void Debug_Telemetry_Tick5ms(int encoder_a_delta, int encoder_b_delta);
/* 主循环调用：通过 UART0 输出一行紧凑、带时间戳的 CSV。 */
void Debug_Telemetry_Process(void);

#endif
