#include "k230_link.h"
#include "ti_msp_dl_config.h"

#define K230_LINE_BUFFER_SIZE 96U
#define K230_HELLO_PERIOD_MS  500U
#define K230_RX_RING_SIZE     128U

static K230_BallPosition g_ball_position;
static K230_LinkDiagnostics g_diagnostics;
static char g_line_buffer[K230_LINE_BUFFER_SIZE];
static uint8_t g_line_length;
static uint32_t g_now_ms;
static uint32_t g_last_frame_ms;
static uint32_t g_last_hello_ms;
static uint8_t g_first_frame_ack_sent;
static uint8_t g_tx_attempted;
static uint8_t g_local_loopback_detected;
static volatile uint8_t g_rx_ring[K230_RX_RING_SIZE];
static volatile uint8_t g_rx_ring_read;
static volatile uint8_t g_rx_ring_write;
static volatile uint32_t g_rx_bytes;
static volatile uint32_t g_rx_overruns;

static uint8_t K230_ParseUnsigned(const char **text, uint32_t *value)
{
    uint32_t result = 0U;
    uint8_t digits = 0U;

    while ((**text >= '0') && (**text <= '9')) {
        result = result * 10U + (uint32_t)(**text - '0');
        (*text)++;
        digits++;
    }
    *value = result;
    return digits;
}

static uint8_t K230_ParseFloat(const char **text, float *value)
{
    uint32_t whole = 0U;
    uint32_t fraction = 0U;
    uint32_t divisor = 1U;
    uint8_t negative = 0U;
    uint8_t digits;

    if (**text == '-') {
        negative = 1U;
        (*text)++;
    }
    digits = K230_ParseUnsigned(text, &whole);
    if (**text == '.') {
        (*text)++;
        while ((**text >= '0') && (**text <= '9')) {
            if (divisor < 1000000U) {
                fraction = fraction * 10U + (uint32_t)(**text - '0');
                divisor *= 10U;
            }
            (*text)++;
            digits++;
        }
    }
    if (digits == 0U) {
        return 0U;
    }
    *value = (float)whole + (float)fraction / (float)divisor;
    if (negative != 0U) {
        *value = -*value;
    }
    return 1U;
}

static uint8_t K230_ParseSigned(const char **text, int32_t *value)
{
    uint32_t magnitude;
    uint8_t negative = 0U;

    if (**text == '-') {
        negative = 1U;
        (*text)++;
    } else if (**text == '+') {
        (*text)++;
    }
    if ((K230_ParseUnsigned(text, &magnitude) == 0U) ||
        (magnitude > 2147483647U)) {
        return 0U;
    }
    *value = (negative != 0U) ? -(int32_t)magnitude : (int32_t)magnitude;
    return 1U;
}

static uint8_t K230_ParseEdgeDirection(const char **text, int8_t *value)
{
    uint32_t magnitude;
    int8_t sign = 1;

    if (**text == '-') {
        sign = -1;
        (*text)++;
    } else if (**text == '+') {
        (*text)++;
    }
    if ((K230_ParseUnsigned(text, &magnitude) == 0U) || (magnitude > 1U)) {
        return 0U;
    }
    *value = (int8_t)((int32_t)sign * (int32_t)magnitude);
    return 1U;
}

static void K230_SendString(const char *text)
{
    g_tx_attempted = 1U;
    while (*text != '\0') {
        while (DL_UART_Main_isTXFIFOFull(UART_1_INST)) {
        }
        DL_UART_Main_transmitData(UART_1_INST, (uint8_t)*text++);
    }
}

static void K230_ParseLine(const char *line)
{
    const char *text = line;
    float x_mm;
    uint32_t valid;
    uint32_t seq;
    uint32_t source_timestamp_ms = 0U;
    uint32_t fps_x10 = 0U;
    int32_t center_x_px = -1;
    int8_t edge_direction = 0;

    /* UART1 PB6->PB7 本地回环的最短令牌，排除长帧字段解析干扰。 */
    if ((text[0] == '$') && (text[1] == 'L') && (text[2] == '#') &&
        (text[3] == '\0')) {
        g_local_loopback_detected = 1U;
        return;
    }

    if ((text[0] == '$') && (text[1] == 'M') && (text[2] == 'S') &&
        (text[3] == 'P') && (text[4] == 'M') && (text[5] == '0') &&
        (text[6] == ',') && (text[7] == 'H') && (text[8] == 'E') &&
        (text[9] == 'L') && (text[10] == 'L') && (text[11] == 'O') &&
        (text[12] == '#') && (text[13] == '\0')) {
        g_local_loopback_detected = 1U;
        return;
    }

    if ((text[0] != '$') || (text[1] != 'K') || (text[2] != '2') ||
        (text[3] != '3') || (text[4] != '0') || (text[5] != ',') ||
        (text[6] != 'B') || (text[7] != 'A') || (text[8] != 'L') ||
        (text[9] != 'L') || (text[10] != ',')) {
        g_diagnostics.parse_errors++;
        return;
    }

    text += 11;
    if ((K230_ParseFloat(&text, &x_mm) == 0U) || (*text++ != ',') ||
        (K230_ParseUnsigned(&text, &valid) == 0U) || (*text++ != ',') ||
        (K230_ParseUnsigned(&text, &seq) == 0U) || (valid > 1U)) {
        g_diagnostics.parse_errors++;
        return;
    }
    if (*text == ',') {
        text++;
        if (K230_ParseEdgeDirection(&text, &edge_direction) == 0U) {
            g_diagnostics.parse_errors++;
            return;
        }
        if (*text == ',') {
            text++;
            if (K230_ParseUnsigned(&text, &source_timestamp_ms) == 0U) {
                g_diagnostics.parse_errors++;
                return;
            }
        }
        if (*text == ',') {
            text++;
            if ((K230_ParseSigned(&text, &center_x_px) == 0U) ||
                (center_x_px < -1) || (center_x_px > 32767)) {
                g_diagnostics.parse_errors++;
                return;
            }
        }
        if (*text == ',') {
            text++;
            if ((K230_ParseUnsigned(&text, &fps_x10) == 0U) ||
                (fps_x10 > 65535U)) {
                g_diagnostics.parse_errors++;
                return;
            }
        }
    }
    if ((*text != '#') || (text[1] != '\0')) {
        g_diagnostics.parse_errors++;
        return;
    }

    K230_Link_UpdatePosition(x_mm, 0.0f, (uint8_t)valid, edge_direction,
                             g_now_ms, source_timestamp_ms,
                             (int16_t)center_x_px, (uint16_t)fps_x10);
    g_last_frame_ms = g_now_ms;
    if ((g_diagnostics.valid_frames != 0U) &&
        (seq > g_diagnostics.last_seq) &&
        ((seq - g_diagnostics.last_seq) > 1U)) {
        g_diagnostics.sequence_gaps +=
            seq - g_diagnostics.last_seq - 1U;
    }
    g_diagnostics.last_seq = seq;
    g_diagnostics.valid_frames++;
    g_diagnostics.timed_out = 0U;
    if (g_first_frame_ack_sent == 0U) {
        K230_SendString("$MSPM0,ACK#\r\n");
        g_first_frame_ack_sent = 1U;
    }
}

static void K230_ReceiveByte(uint8_t byte)
{
    if (byte == '$') {
        g_line_length = 0U;
        g_line_buffer[g_line_length++] = (char)byte;
        return;
    }
    if (g_line_length == 0U) {
        return;
    }
    if (g_line_length >= (K230_LINE_BUFFER_SIZE - 1U)) {
        g_line_length = 0U;
        g_diagnostics.parse_errors++;
        return;
    }

    g_line_buffer[g_line_length++] = (char)byte;
    if (byte == '#') {
        g_line_buffer[g_line_length] = '\0';
        K230_ParseLine(g_line_buffer);
        g_line_length = 0U;
    }
}

void K230_Link_Init(void)
{
    g_ball_position.x_mm = 0.0f;
    g_ball_position.y_mm = 0.0f;
    g_ball_position.valid = 0U;
    g_ball_position.edge_direction = 0;
    g_ball_position.timestamp_ms = 0U;
    g_ball_position.source_timestamp_ms = 0U;
    g_ball_position.center_x_px = -1;
    g_ball_position.fps_x10 = 0U;
    g_diagnostics.rx_bytes = 0U;
    g_diagnostics.rx_overruns = 0U;
    g_diagnostics.valid_frames = 0U;
    g_diagnostics.parse_errors = 0U;
    g_diagnostics.sequence_gaps = 0U;
    g_diagnostics.last_seq = 0U;
    g_diagnostics.timed_out = 1U;
    g_line_length = 0U;
    g_now_ms = 0U;
    g_last_frame_ms = 0U;
    g_last_hello_ms = 0U;
    g_first_frame_ack_sent = 0U;
    g_tx_attempted = 0U;
    g_local_loopback_detected = 0U;
    g_rx_ring_read = 0U;
    g_rx_ring_write = 0U;
    g_rx_bytes = 0U;
    g_rx_overruns = 0U;
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        (void)DL_UART_Main_receiveData(UART_1_INST);
    }
    /* 最小联调上电即发一次，避免首帧诊断依赖 5 ms 时基。 */
    K230_SendString("$L#");
    K230_SendString("$MSPM0,HELLO#\r\n");
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
}

void K230_Link_Tick5ms(void)
{
    g_now_ms += 5U;
}

void K230_Link_Process(void)
{
    while (g_rx_ring_read != g_rx_ring_write) {
        K230_ReceiveByte(g_rx_ring[g_rx_ring_read]);
        g_rx_ring_read = (uint8_t)((g_rx_ring_read + 1U) &
                                   (K230_RX_RING_SIZE - 1U));
    }
    g_diagnostics.rx_bytes = g_rx_bytes;
    g_diagnostics.rx_overruns = g_rx_overruns;

    if ((uint32_t)(g_now_ms - g_last_hello_ms) >= K230_HELLO_PERIOD_MS) {
        K230_SendString("$MSPM0,HELLO#\r\n");
        g_last_hello_ms = g_now_ms;
    }

    if ((g_diagnostics.timed_out == 0U) &&
        ((uint32_t)(g_now_ms - g_last_frame_ms) > K230_LINK_TIMEOUT_MS)) {
        g_ball_position.valid = 0U;
        g_ball_position.edge_direction = 0;
        g_diagnostics.timed_out = 1U;
    }
}

void K230_Link_UART1_IRQHandler(void)
{
    uint8_t next_write;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        next_write = (uint8_t)((g_rx_ring_write + 1U) &
                               (K230_RX_RING_SIZE - 1U));
        if (next_write == g_rx_ring_read) {
            (void)DL_UART_Main_receiveData(UART_1_INST);
            g_rx_overruns++;
        } else {
            g_rx_ring[g_rx_ring_write] = DL_UART_Main_receiveData(UART_1_INST);
            g_rx_ring_write = next_write;
            g_rx_bytes++;
        }
    }
}

void K230_Link_UpdatePosition(float x_mm, float y_mm, uint8_t valid,
                              int8_t edge_direction, uint32_t timestamp_ms,
                              uint32_t source_timestamp_ms,
                              int16_t center_x_px, uint16_t fps_x10)
{
    g_ball_position.x_mm = x_mm;
    g_ball_position.y_mm = y_mm;
    g_ball_position.valid = valid;
    g_ball_position.edge_direction = edge_direction;
    g_ball_position.timestamp_ms = timestamp_ms;
    g_ball_position.source_timestamp_ms = source_timestamp_ms;
    g_ball_position.center_x_px = center_x_px;
    g_ball_position.fps_x10 = fps_x10;
}

void K230_Link_GetPosition(K230_BallPosition *position)
{
    if (position != 0) {
        *position = g_ball_position;
    }
}

void K230_Link_GetDiagnostics(K230_LinkDiagnostics *diagnostics)
{
    if (diagnostics != 0) {
        *diagnostics = g_diagnostics;
    }
}

uint8_t K230_Link_HasValidFrame(void)
{
    return (g_diagnostics.valid_frames != 0U) ? 1U : 0U;
}

uint8_t K230_Link_HasTxAttempt(void)
{
    return g_tx_attempted;
}

uint8_t K230_Link_IsLocalLoopbackDetected(void)
{
    return g_local_loopback_detected;
}

uint8_t K230_Link_HasRxBytes(void)
{
    return (g_rx_bytes != 0U) ? 1U : 0U;
}
