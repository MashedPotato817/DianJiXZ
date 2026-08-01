#include "debug_telemetry.h"

#include "ball_control.h"
#include "ball_task.h"
#include "k230_link.h"
#include "servo.h"
#include "ti_msp_dl_config.h"

#include <string.h>

/*
 * ctrl: 0=OFF, 1=TRACK, 2=EDGE, 3=HOLD, 4=LOST。
 */

static volatile uint32_t g_telemetry_now_ms;
static uint32_t g_last_send_ms;
static uint32_t g_last_header_ms;

/* 事件环形缓冲：中断写入 tail，主循环发送 head。 */
static char g_event_text[DEBUG_TELEMETRY_EVENT_BUFFER][DEBUG_TELEMETRY_EVENT_MAX_LEN];
static volatile uint32_t g_event_time[DEBUG_TELEMETRY_EVENT_BUFFER];
static volatile uint8_t g_event_head;
static volatile uint8_t g_event_tail;

#define DEBUG_TELEMETRY_HEADER_PERIOD_MS 5000U

static void Debug_Telemetry_SendChar(char value)
{
    while (DL_UART_Main_isTXFIFOFull(UART_0_INST)) {
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)value);
}

static void Debug_Telemetry_SendString(const char *text)
{
    while (*text != '\0') {
        Debug_Telemetry_SendChar(*text++);
    }
}

static void Debug_Telemetry_SendUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t length = 0U;

    do {
        digits[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (length < sizeof(digits)));

    while (length != 0U) {
        Debug_Telemetry_SendChar(digits[--length]);
    }
}

static void Debug_Telemetry_SendSigned(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        Debug_Telemetry_SendChar('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    Debug_Telemetry_SendUnsigned(magnitude);
}

static void Debug_Telemetry_FieldUnsigned(uint32_t value)
{
    Debug_Telemetry_SendChar(',');
    Debug_Telemetry_SendUnsigned(value);
}

static void Debug_Telemetry_FieldSigned(int32_t value)
{
    Debug_Telemetry_SendChar(',');
    Debug_Telemetry_SendSigned(value);
}

static int32_t Debug_Telemetry_Scale10(float value)
{
    if (value >= 0.0f) {
        return (int32_t)(value * 10.0f + 0.5f);
    }
    return (int32_t)(value * 10.0f - 0.5f);
}

static uint8_t Debug_Telemetry_GetControlState(const Ball_Control *control)
{
    if ((control == 0) || (control->enabled == 0U)) {
        return 0U;
    }
    return (uint8_t)control->phase;
}

static void Debug_Telemetry_SendHeader(void)
{
    Debug_Telemetry_SendString(
        "#TELEM_V4,t_ms,dt_ms,rx_ms,rx_age_ms,seq,x10,valid,edge,"
        "v10,trim10,servo_us,out10,"
        "ctrl,parse_err,crc_err,seq_gap,target_x10,task#\r\n");
}

void Debug_Telemetry_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t i;

    __disable_irq();
    g_telemetry_now_ms = 0U;
    g_last_send_ms = 0U;
    g_last_header_ms = 0U;
    g_event_head = 0U;
    g_event_tail = 0U;
    for (i = 0U; i < DEBUG_TELEMETRY_EVENT_BUFFER; i++) {
        g_event_time[i] = 0U;
        g_event_text[i][0] = '\0';
    }
    if (primask == 0U) {
        __enable_irq();
    }

    Debug_Telemetry_SendHeader();
}

void Debug_Telemetry_LogEvent(const char *text)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t next;

    if (text == 0) {
        return;
    }
    __disable_irq();
    g_event_time[g_event_tail] = g_telemetry_now_ms;
    (void)strncpy(g_event_text[g_event_tail], text,
                  DEBUG_TELEMETRY_EVENT_MAX_LEN - 1U);
    g_event_text[g_event_tail][DEBUG_TELEMETRY_EVENT_MAX_LEN - 1U] = '\0';
    next = (uint8_t)((g_event_tail + 1U) % DEBUG_TELEMETRY_EVENT_BUFFER);
    if (next == g_event_head) {
        g_event_head = (uint8_t)((g_event_head + 1U) %
                                 DEBUG_TELEMETRY_EVENT_BUFFER);
    }
    g_event_tail = next;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Debug_Telemetry_Tick5ms(int encoder_a_delta, int encoder_b_delta)
{
    /* 编码器字段已从遥测精简移除；参数保留以维持 5ms 中断调用点不变。 */
    (void)encoder_a_delta;
    (void)encoder_b_delta;
    g_telemetry_now_ms += 5U;
}

void Debug_Telemetry_Process(void)
{
    K230_BallPosition position;
    K230_LinkDiagnostics diagnostics;
    const Ball_Control *control;
    uint32_t now_ms;
    uint32_t dt_ms;
    uint32_t rx_age_delta;
    uint32_t primask;
    int32_t rx_age_ms;

    now_ms = g_telemetry_now_ms;

    /* 事件行不等 50ms 遥测周期，有事件就尽快发送。 */
    while (g_event_head != g_event_tail) {
        Debug_Telemetry_SendString("$E,");
        Debug_Telemetry_SendUnsigned(g_event_time[g_event_head]);
        Debug_Telemetry_SendChar(',');
        Debug_Telemetry_SendString(g_event_text[g_event_head]);
        Debug_Telemetry_SendString("#\r\n");
        g_event_head = (uint8_t)((g_event_head + 1U) %
                                 DEBUG_TELEMETRY_EVENT_BUFFER);
    }

    if ((uint32_t)(now_ms - g_last_send_ms) < DEBUG_TELEMETRY_PERIOD_MS) {
        return;
    }
    if ((uint32_t)(now_ms - g_last_header_ms) >=
        DEBUG_TELEMETRY_HEADER_PERIOD_MS) {
        Debug_Telemetry_SendHeader();
        g_last_header_ms = now_ms;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    now_ms = g_telemetry_now_ms;
    if (primask == 0U) {
        __enable_irq();
    }

    dt_ms = (uint32_t)(now_ms - g_last_send_ms);
    g_last_send_ms = now_ms;
    K230_Link_GetPosition(&position);
    K230_Link_GetDiagnostics(&diagnostics);
    control = Ball_Control_Get();
    if (diagnostics.valid_frames == 0U) {
        rx_age_ms = -1;
    } else if (now_ms < position.timestamp_ms) {
        /* 两个 5 ms 计数器在同一中断内顺序更新时，最多相差一个节拍。 */
        rx_age_ms = 0;
    } else {
        rx_age_delta = now_ms - position.timestamp_ms;
        rx_age_ms = (rx_age_delta > 2147483647U) ?
                    2147483647 : (int32_t)rx_age_delta;
    }

    Debug_Telemetry_SendString("$T");
    Debug_Telemetry_FieldUnsigned(now_ms);
    Debug_Telemetry_FieldUnsigned(dt_ms);
    Debug_Telemetry_FieldUnsigned(position.timestamp_ms);
    Debug_Telemetry_FieldSigned(rx_age_ms);
    Debug_Telemetry_FieldUnsigned(diagnostics.last_seq);
    Debug_Telemetry_FieldSigned(Debug_Telemetry_Scale10(position.x_mm));
    Debug_Telemetry_FieldUnsigned(position.valid);
    Debug_Telemetry_FieldSigned(position.edge_direction);
    Debug_Telemetry_FieldSigned(
        Debug_Telemetry_Scale10(
            control->filtered_velocity_mm_s));
    Debug_Telemetry_FieldSigned(
        Debug_Telemetry_Scale10(
            control->trim_angle_deg));
    Debug_Telemetry_FieldUnsigned(Servo_GetPulseUs());
    Debug_Telemetry_FieldSigned(
        Debug_Telemetry_Scale10(
            control->last_output_deg));
    Debug_Telemetry_FieldUnsigned(Debug_Telemetry_GetControlState(control));
    /* 精简：只保留核心链路诊断（parse/crc/seq_gap）；wheel_enc 恒 0 已去。 */
    Debug_Telemetry_FieldUnsigned(diagnostics.parse_errors);
    Debug_Telemetry_FieldUnsigned(diagnostics.crc_errors);
    Debug_Telemetry_FieldUnsigned(diagnostics.sequence_gaps);
    /* 当前目标位置（×10，mm）与 Ball_Task 状态。 */
    Debug_Telemetry_FieldSigned(
        Debug_Telemetry_Scale10(control->target_x_mm));
    Debug_Telemetry_FieldUnsigned((uint32_t)Ball_Task_GetState());
    Debug_Telemetry_SendString("#\r\n");
}
