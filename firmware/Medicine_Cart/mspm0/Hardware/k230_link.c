/*
 * K230 串口通信模块 — UART1 (PB6/PB7), 115200 8N1, 轮询接收。
 *
 * 协议：$...# 定界 ASCII 帧
 *   握手：  $K230,HELLO# / $MSPM0,HELLO# → ACK → LINK_OK
 *   心跳：  $K230,DATA# / $MSPM0,DATA# (1 Hz)
 *   结果：  $K230,RESULT,<病房号>,<置信度>#  (Task 2 新增)
 *   超时：  3 秒无收 → LINK_LOST → 复位握手
 *
 * 病房号确认：连续 K230_TARGET_CONFIRM_COUNT 次相同结果后锁定。
 */
#include "k230_link.h"
#include "board.h"
#include <string.h>

/* ---- 内部状态 ---- */
static char g_lineBuf[K230_LINE_BUF_SIZE];
static uint8_t g_lineIndex;

static volatile uint32_t g_nowMs;
static uint32_t g_lastRxMs;
static uint32_t g_lastHelloMs;
static uint32_t g_lastDataMs;

static uint8_t g_k230Hello;
static uint8_t g_k230Ack;
static K230_LinkState g_linkState;

/* ---- 识别结果稳定确认 ---- */
static K230_Result g_latestResult;
static uint8_t g_pendingWard;
static uint8_t g_pendingCount;

/* ========== 底层 UART 操作 ========== */

static void UART1_SendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_1_INST)) {}
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

/* ========== 帧解析 ========== */

/*
 * 解析 $K230,RESULT,<ward>,<conf>#
 * 例如 "$K230,RESULT,3,95#" → ward=3, confidence=95
 */
static void ParseResult(const char *line, K230_Result *out)
{
    uint8_t ward = 0;
    uint8_t conf = 0;
    const char *p;
    int field;

    /* 跳过 "$K230,RESULT," 共13字符 */
    if (strncmp(line, "$K230,RESULT,", 13) != 0) {
        out->ward = K230_WARD_NONE;
        out->confidence = 0;
        return;
    }

    p = line + 13;
    field = 0;
    while (*p != '\0' && *p != '#') {
        if (*p == ',') {
            field++;
            p++;
            continue;
        }
        if (field == 0 && *p >= '0' && *p <= '9') {
            ward = ward * 10 + (uint8_t)(*p - '0');
        } else if (field == 1 && *p >= '0' && *p <= '9') {
            conf = conf * 10 + (uint8_t)(*p - '0');
        }
        p++;
    }

    if (ward >= K230_WARD_MIN && ward <= K230_WARD_MAX) {
        out->ward = ward;
        out->confidence = (conf <= 100) ? conf : 100;
    } else {
        out->ward = K230_WARD_NONE;
        out->confidence = 0;
    }
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
        /* 识别结果帧 */
        K230_Result parsed;
        ParseResult(line, &parsed);

        if (parsed.ward != K230_WARD_NONE && !g_latestResult.confirmed) {
            /* 稳定性确认：连续相同 */
            if (parsed.ward == g_pendingWard) {
                g_pendingCount++;
                if (g_pendingCount >= K230_TARGET_CONFIRM_COUNT) {
                    g_latestResult = parsed;
                    g_latestResult.confirmed = 1;
                    g_latestResult.timestamp_ms = g_nowMs;
                }
            } else {
                g_pendingWard = parsed.ward;
                g_pendingCount = 1;
            }
        }
        /* 总是更新最新原始结果 */
        if (!g_latestResult.confirmed) {
            g_latestResult.ward = parsed.ward;
            g_latestResult.confidence = parsed.confidence;
        }
    }
}

/* ========== 接收处理 ========== */

static void ProcessRx(void)
{
    uint8_t data;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        data = DL_UART_Main_receiveData(UART_1_INST);

        if (data == '$') {
            g_lineIndex = 0;
        }
        if (g_lineIndex >= (K230_LINE_BUF_SIZE - 1U)) {
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

/* ========== 公开接口 ========== */

void K230_Link_Init(void)
{
    uint8_t data;

    g_lineIndex = 0;
    g_nowMs = 0;
    g_lastRxMs = 0;
    g_lastHelloMs = 0;
    g_lastDataMs = 0;
    g_k230Hello = 0;
    g_k230Ack = 0;
    g_linkState = K230_LINK_DOWN;
    g_pendingWard = 0;
    g_pendingCount = 0;
    memset(&g_latestResult, 0, sizeof(g_latestResult));
    memset(g_lineBuf, 0, sizeof(g_lineBuf));

    /* 清空 FIFO 残留 */
    while (DL_UART_Main_receiveDataCheck(UART_1_INST, &data)) {}

    /*
     * 配置 UART1 为 115200 轮询模式（关闭 SysConfig 默认的 DMA/中断）。
     * 波特率分频器值与 K230_UART_Demo 一致：IBRD=21, FBRD=45 @ 40MHz BUSCLK。
     */
    DL_UART_Main_disableDMAReceiveEvent(UART_1_INST, DL_UART_DMA_INTERRUPT_RX);
    DL_UART_Main_disableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST, 21U, 45U);

    UART1_SendString("$MSPM0,BOOT#\r\n");
}

void K230_Link_Process(void)
{
    uint32_t now = g_nowMs;

    ProcessRx();

    /* 握手阶段：发 HELLO 直到收到 ACK */
    if (!g_k230Ack && TimeReached(now, g_lastHelloMs, K230_HELLO_PERIOD_MS)) {
        g_lastHelloMs = now;
        UART1_SendString("$MSPM0,HELLO#\r\n");
    }

    /* 握手完成 */
    if (g_linkState < K230_LINK_ONLINE && g_k230Hello && g_k230Ack) {
        g_linkState = K230_LINK_ONLINE;
        UART1_SendString("$MSPM0,LINK_OK#\r\n");
    } else if (g_linkState == K230_LINK_DOWN && (g_k230Hello || g_k230Ack)) {
        g_linkState = K230_LINK_HANDSHAKE;
    }

    /* 在线心跳 */
    if (g_linkState == K230_LINK_ONLINE &&
        TimeReached(now, g_lastDataMs, K230_DATA_PERIOD_MS)) {
        g_lastDataMs = now;
        UART1_SendString("$MSPM0,DATA#\r\n");
    }

    /* 链路超时检测 */
    if (g_linkState >= K230_LINK_HANDSHAKE &&
        TimeReached(now, g_lastRxMs, K230_LINK_TIMEOUT_MS)) {
        g_linkState = K230_LINK_DOWN;
        g_k230Hello = 0;
        g_k230Ack = 0;
        UART1_SendString("$MSPM0,LINK_LOST#\r\n");
    }
}

uint8_t K230_Get_TargetWard(void)
{
    if (g_latestResult.confirmed) {
        return g_latestResult.ward;
    }
    return K230_WARD_NONE;
}

uint8_t K230_Is_TargetLocked(void)
{
    return g_latestResult.confirmed;
}

uint8_t K230_Is_Online(void)
{
    return (g_linkState == K230_LINK_ONLINE) ? 1 : 0;
}

K230_LinkState K230_Get_State(void)
{
    return g_linkState;
}

void K230_Get_Result(K230_Result *result)
{
    if (result) {
        *result = g_latestResult;
    }
}

/* ========== 5ms 时基（由 TIMG0 ISR 更新） ========== */
void K230_Tick5ms(void)
{
    g_nowMs += 5U;
}
