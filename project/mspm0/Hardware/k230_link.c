#include "k230_link.h"
#include "ti_msp_dl_config.h"

#define K230_LINE_BUFFER_SIZE 64U
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
static volatile uint32_t g_rx_hw_overruns;

static uint8_t K230_ParseUnsigned(const char **text, uint32_t *value)
{
    uint32_t result = 0U;
    uint32_t digit;
    uint8_t digits = 0U;

    while ((**text >= '0') && (**text <= '9')) {
        digit = (uint32_t)(**text - '0');
        if (result > ((4294967295U - digit) / 10U)) {
            return 0U;
        }
        result = result * 10U + digit;
        (*text)++;
        digits++;
    }
    *value = result;
    return digits;
}

static uint8_t K230_HexValue(char value, uint8_t *result)
{
    if ((value >= '0') && (value <= '9')) {
        *result = (uint8_t)(value - '0');
        return 1U;
    }
    if ((value >= 'A') && (value <= 'F')) {
        *result = (uint8_t)(value - 'A' + 10);
        return 1U;
    }
    if ((value >= 'a') && (value <= 'f')) {
        *result = (uint8_t)(value - 'a' + 10);
        return 1U;
    }
    return 0U;
}

static uint8_t K230_Crc8Atm(const char *begin, const char *end)
{
    uint8_t crc = 0U;
    uint8_t bit;

    while (begin < end) {
        crc ^= (uint8_t)*begin++;
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
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
    const char *crc_marker;
    int32_t x10;
    uint32_t valid;
    uint32_t seq;
    int8_t edge_direction = 0;
    uint8_t crc_high;
    uint8_t crc_low;
    uint8_t received_crc;
    uint8_t calculated_crc;

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

    crc_marker = text;
    while ((*crc_marker != '\0') && (*crc_marker != '*')) {
        crc_marker++;
    }
    if ((*crc_marker != '*') ||
        (K230_HexValue(crc_marker[1], &crc_high) == 0U) ||
        (K230_HexValue(crc_marker[2], &crc_low) == 0U) ||
        (crc_marker[3] != '#') || (crc_marker[4] != '\0')) {
        g_diagnostics.parse_errors++;
        return;
    }
    received_crc = (uint8_t)((crc_high << 4U) | crc_low);
    calculated_crc = K230_Crc8Atm(line + 1, crc_marker);
    if (received_crc != calculated_crc) {
        g_diagnostics.crc_errors++;
        return;
    }

    text += 11;
    if ((K230_ParseSigned(&text, &x10) == 0U) || (*text++ != ',') ||
        (K230_ParseUnsigned(&text, &valid) == 0U) || (*text++ != ',') ||
        (K230_ParseUnsigned(&text, &seq) == 0U) || (*text++ != ',') ||
        (K230_ParseEdgeDirection(&text, &edge_direction) == 0U) ||
        (*text != '*')) {
        g_diagnostics.parse_errors++;
        return;
    }
    if ((x10 < -K230_LINK_MAX_POSITION_X10) ||
        (x10 > K230_LINK_MAX_POSITION_X10) || (valid > 1U) ||
        ((valid != 0U) && (edge_direction != 0))) {
        g_diagnostics.range_errors++;
        return;
    }

    K230_Link_UpdatePosition((float)x10 / 10.0f, 0.0f, (uint8_t)valid,
                             edge_direction, g_now_ms);
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
    g_diagnostics.rx_bytes = 0U;
    g_diagnostics.rx_overruns = 0U;
    g_diagnostics.rx_hw_overruns = 0U;
    g_diagnostics.valid_frames = 0U;
    g_diagnostics.parse_errors = 0U;
    g_diagnostics.crc_errors = 0U;
    g_diagnostics.range_errors = 0U;
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
    g_rx_hw_overruns = 0U;
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        (void)DL_UART_Main_receiveData(UART_1_INST);
    }
    DL_UART_Main_clearInterruptStatus(
        UART_1_INST, DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
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
    g_diagnostics.rx_hw_overruns = g_rx_hw_overruns;

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

    /*
     * 硬件RX FIFO仅4字节深，溢出发生在数据位读出之前。用RIS标志位
     * 判断，不读RXDATA，避免额外弹出一个字节。
     */
    if (DL_UART_Main_getRawInterruptStatus(
            UART_1_INST, DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR) != 0U) {
        g_rx_hw_overruns++;
        DL_UART_Main_clearInterruptStatus(
            UART_1_INST, DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
    }

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
                              int8_t edge_direction, uint32_t timestamp_ms)
{
    g_ball_position.x_mm = x_mm;
    g_ball_position.y_mm = y_mm;
    g_ball_position.valid = valid;
    g_ball_position.edge_direction = edge_direction;
    g_ball_position.timestamp_ms = timestamp_ms;
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
