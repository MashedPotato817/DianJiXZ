#ifndef H_DEBUG_TELEMETRY
#define H_DEBUG_TELEMETRY

#include <stdint.h>

/* UART0 调试遥测周期与 K230 视觉发送周期一致。 */
#define DEBUG_TELEMETRY_PERIOD_MS 50U

void Debug_Telemetry_Init(void);
/* 5 ms 定时中断调用：提供时基；编码器参数已精简忽略（保留以维持调用点）。 */
void Debug_Telemetry_Tick5ms(int encoder_a_delta, int encoder_b_delta);
/* 主循环调用：通过 UART0 输出一行紧凑、带时间戳的 CSV。 */
void Debug_Telemetry_Process(void);

/*
 * 记录一个带时间戳的事件行（$E,<t_ms>,<text>#），供按键、复位、任务状态
 * 变化等低频事件写入。中断/主循环均可调用；内部存入环形缓冲，由
 * Debug_Telemetry_Process 在 UART0 上统一发送，不阻塞调用方。
 */
#define DEBUG_TELEMETRY_EVENT_BUFFER 8U
#define DEBUG_TELEMETRY_EVENT_MAX_LEN 24U
void Debug_Telemetry_LogEvent(const char *text);

#endif
