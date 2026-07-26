/*
 * K230 与 MSPM0G3507 双向 UART 最小联调示例。
 * 接线：K230 IO40(TX) -> PB7(RX)，IO41(RX) -> PB6(TX)，GND 共地。
 */
#include "ti_msp_dl_config.h"
#include <string.h>

#define K230_UART_IBRD_115200    (21U)
#define K230_UART_FBRD_115200    (45U)
#define HELLO_PERIOD_MS          (500U)
#define DATA_PERIOD_MS           (1000U)
#define LINK_TIMEOUT_MS          (3000U)
#define LINE_BUF_SIZE            (32U)

static volatile uint32_t g_nowMs;
static uint8_t g_k230Hello;
static uint8_t g_k230Ack;
static uint8_t g_linkOnline;
static uint32_t g_lastRxMs;
static uint32_t g_lastHelloMs;
static uint32_t g_lastDataMs;
static char g_lineBuf[LINE_BUF_SIZE];
static uint8_t g_lineIndex;

/* RESULT/LEFT/RIGHT 帧接收 */
static uint8_t g_lastWard;
static uint8_t g_lastConf;
static uint32_t g_lastResultMs;
static uint8_t g_leftWard;
static uint8_t g_leftConf;
static uint8_t g_rightWard;
static uint8_t g_rightConf;

static void UART1_SendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_1_INST)) {
    }
    DL_UART_Main_transmitData(UART_1_INST, data);
}

static void UART1_SendString(const char *text)
{
    while (*text != '\0') {
        UART1_SendByte((uint8_t)*text++);
    }
}

static uint8_t TimeReached(uint32_t now, uint32_t last, uint32_t period)
{
    return ((uint32_t)(now - last) >= period);
}

/*
 * 解析 $K230,<TYPE>,<ward>,<conf>#
 * 找到第二个逗号后的 ward 和 conf，适用 RESULT/LEFT/RIGHT。
 * 返回 1 表示解析成功，结果写入 *ward 和 *conf。
 */
static uint8_t ParseResult(const char *line, uint8_t *ward, uint8_t *conf)
{
    const char *p;
    uint8_t w = 0, c = 0;

    /* 跳过 "$K230," */
    if (strncmp(line, "$K230,", 6) != 0) return 0;

    /* 跳过 TYPE 字段到第一个逗号后 */
    p = line + 6;
    while (*p != '\0' && *p != ',' && *p != '#') p++;
    if (*p != ',') return 0;
    p++;  /* 跳过逗号 */

    /* 解析病房号 */
    while (*p >= '0' && *p <= '9') {
        w = w * 10 + (uint8_t)(*p++ - '0');
    }
    if (*p != ',') return 0;
    p++;
    /* 解析置信度 */
    while (*p >= '0' && *p <= '9') {
        c = c * 10 + (uint8_t)(*p++ - '0');
    }
    if (w < 1 || w > 8) return 0;

    *ward = w;
    *conf = (c <= 100) ? c : 100;
    return 1;
}

static void HandleLine(const char *line)
{
    g_lastRxMs = g_nowMs;

    if (strcmp(line, "$K230,HELLO#") == 0) {
        g_k230Hello = 1;
        UART1_SendString("$MSPM0,ACK#\r\n");
    } else if (strcmp(line, "$K230,ACK#") == 0) {
        g_k230Ack = 1;
    } else if (strcmp(line, "$K230,DATA#") == 0) {
        UART1_SendString("$MSPM0,DATA_ACK#\r\n");
    } else if (strncmp(line, "$K230,RESULT,", 13) == 0) {
        uint8_t ward, conf;
        if (ParseResult(line, &ward, &conf)) {
            g_lastWard = ward;
            g_lastConf = conf;
            g_lastResultMs = g_nowMs;
            DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
            UART1_SendString("$MSPM0,RESULT_ACK#\r\n");
        }
    } else if (strncmp(line, "$K230,LEFT,", 11) == 0) {
        uint8_t ward, conf;
        if (ParseResult(line, &ward, &conf)) {
            g_leftWard = ward;
            g_leftConf = conf;
            DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
            UART1_SendString("$MSPM0,LEFT_ACK#\r\n");
        }
    } else if (strncmp(line, "$K230,RIGHT,", 12) == 0) {
        uint8_t ward, conf;
        if (ParseResult(line, &ward, &conf)) {
            g_rightWard = ward;
            g_rightConf = conf;
            DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
            UART1_SendString("$MSPM0,RIGHT_ACK#\r\n");
        }
    }
}

static void ProcessRx(void)
{
    uint8_t data;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        data = DL_UART_Main_receiveData(UART_1_INST);
        if (data == '$') {
            g_lineIndex = 0;
        }
        if (g_lineIndex >= (LINE_BUF_SIZE - 1U)) {
            g_lineIndex = 0;
            continue;
        }
        if ((data == '\r') || (data == '\n')) {
            continue;
        }
        g_lineBuf[g_lineIndex++] = (char)data;
        if (data == '#') {
            g_lineBuf[g_lineIndex] = '\0';
            HandleLine(g_lineBuf);
            g_lineIndex = 0;
        }
    }
}

void TIMG0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMERG_IIDX_ZERO) {
        g_nowMs += 5U;
    }
}

int main(void)
{
    SYSCFG_DL_init();

    /* 此 Demo 不使用原车工程中 UART1 的 DMA 接收。 */
    DL_UART_Main_disableDMAReceiveEvent(UART_1_INST, DL_UART_DMA_INTERRUPT_RX);
    DL_UART_Main_disableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST,
        K230_UART_IBRD_115200, K230_UART_FBRD_115200);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    UART1_SendString("$MSPM0,BOOT#\r\n");

    while (1) {
        ProcessRx();

        if (!g_k230Ack && TimeReached(g_nowMs, g_lastHelloMs, HELLO_PERIOD_MS)) {
            g_lastHelloMs = g_nowMs;
            UART1_SendString("$MSPM0,HELLO#\r\n");
        }

        if (!g_linkOnline && g_k230Hello && g_k230Ack) {
            g_linkOnline = 1;
            UART1_SendString("$MSPM0,LINK_OK#\r\n");
        }

        if (g_linkOnline && TimeReached(g_nowMs, g_lastDataMs, DATA_PERIOD_MS)) {
            g_lastDataMs = g_nowMs;
            UART1_SendString("$MSPM0,DATA#\r\n");
        }

        if (g_linkOnline && TimeReached(g_nowMs, g_lastRxMs, LINK_TIMEOUT_MS)) {
            g_linkOnline = 0;
            g_k230Hello = 0;
            g_k230Ack = 0;
            g_lastWard = 0;
            g_lastConf = 0;
            UART1_SendString("$MSPM0,LINK_LOST#\r\n");
        }

        /* LED 反馈：收到 RESULT 时亮 200ms，在线时心跳闪烁 */
        if (g_lastWard && !TimeReached(g_nowMs, g_lastResultMs, 200U)) {
            DL_GPIO_clearPins(LED_PORT, LED_led_PIN);  /* LED ON */
        } else if (g_linkOnline && ((g_nowMs / 250U) & 1U)) {
            DL_GPIO_clearPins(LED_PORT, LED_led_PIN);  /* heartbeat */
        } else {
            DL_GPIO_setPins(LED_PORT, LED_led_PIN);    /* LED OFF */
        }
    }
}
