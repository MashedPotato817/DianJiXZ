#include "debug_telemetry.h"

#include "ball_control.h"
#include "k230_link.h"
#include "servo.h"
#include "ti_msp_dl_config.h"

/*
 * ctrl:
 * 0=OFF, 1=CAP, 2=ACC, 3=RUN, 4=BRK, 5=EDGE, 6=FLT, 7=HOLD, 8=LOST.
 * wheel_encA_raw/wheel_encB_raw 是 dt_ms 时间窗内的底盘轮编码器累计增量，
 * 不是舵机角度。wheel_enc_valid=0 时仅用于观察悬空输入或电气干扰。
 */

static volatile uint32_t g_telemetry_now_ms;
static volatile int32_t g_encoder_window_a;
static volatile int32_t g_encoder_window_b;
static uint32_t g_last_send_ms;
static uint32_t g_last_header_ms;

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
        "#TELEM_V3,t_ms,dt_ms,rx_ms,rx_age_ms,seq,x10,valid,edge,"
        "v10,toward_v10,stop10,trim10,wheel_enc_valid,"
        "wheel_encA_raw,wheel_encB_raw,servo_us,out10,"
        "ctrl,parse_err,crc_err,range_err,seq_gap,rx_overrun,timeout#\r\n");
}

void Debug_Telemetry_Init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_telemetry_now_ms = 0U;
    g_encoder_window_a = 0;
    g_encoder_window_b = 0;
    g_last_send_ms = 0U;
    g_last_header_ms = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    Debug_Telemetry_SendHeader();
}

void Debug_Telemetry_Tick5ms(int encoder_a_delta, int encoder_b_delta)
{
    g_telemetry_now_ms += 5U;
    g_encoder_window_a += (int32_t)encoder_a_delta;
    g_encoder_window_b += (int32_t)encoder_b_delta;
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
    int32_t encoder_a;
    int32_t encoder_b;

    now_ms = g_telemetry_now_ms;
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
    encoder_a = g_encoder_window_a;
    encoder_b = g_encoder_window_b;
    g_encoder_window_a = 0;
    g_encoder_window_b = 0;
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
            control->toward_velocity_mm_s));
    Debug_Telemetry_FieldSigned(
        Debug_Telemetry_Scale10(
            control->stopping_distance_mm));
    Debug_Telemetry_FieldSigned(
        Debug_Telemetry_Scale10(
            control->trim_angle_deg));
    Debug_Telemetry_FieldUnsigned(DEBUG_WHEEL_ENCODERS_CONNECTED);
    Debug_Telemetry_FieldSigned(encoder_a);
    Debug_Telemetry_FieldSigned(encoder_b);
    Debug_Telemetry_FieldUnsigned(Servo_GetPulseUs());
    Debug_Telemetry_FieldSigned(
        Debug_Telemetry_Scale10(control->last_output_deg));
    Debug_Telemetry_FieldUnsigned(Debug_Telemetry_GetControlState(control));
    Debug_Telemetry_FieldUnsigned(diagnostics.parse_errors);
    Debug_Telemetry_FieldUnsigned(diagnostics.crc_errors);
    Debug_Telemetry_FieldUnsigned(diagnostics.range_errors);
    Debug_Telemetry_FieldUnsigned(diagnostics.sequence_gaps);
    Debug_Telemetry_FieldUnsigned(diagnostics.rx_overruns);
    Debug_Telemetry_FieldUnsigned(diagnostics.timed_out);
    Debug_Telemetry_SendString("#\r\n");
}
