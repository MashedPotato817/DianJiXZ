#include "k210_link.h"
#include <string.h>

#define K210_BINARY_HEAD          0xAAU
#define K210_BINARY_LEN           8U
#define K210_ASCII_BUF_SIZE       24U
#define K210_HELLO_PERIOD_MS      500U
#define K210_ONLINE_TIMEOUT_MS    1500U
#define K210_UART_INST            UART_0_INST

static uint8_t g_k210BinBuf[K210_BINARY_LEN];
static uint8_t g_k210BinIndex;
static char g_k210AsciiBuf[K210_ASCII_BUF_SIZE];
static uint8_t g_k210AsciiIndex;
static uint8_t g_k210AsciiActive;
static volatile uint32_t g_k210NowMs;
static uint32_t g_k210LastHelloMs;
static uint32_t g_k210LastOkMs;
static uint8_t g_k210Online;
static uint8_t g_k210HandshakeOk;
static uint32_t g_k210RxBytes;
static uint32_t g_k210RxFrames;
static uint32_t g_k210VisionFrames;
static uint32_t g_k210ChecksumErrors;
static K210_VisionFrame g_k210LatestVision;

static uint8_t K210_TimeReached(uint32_t now, uint32_t last, uint32_t period)
{
    return ((uint32_t)(now - last) >= period);
}

static void K210_SetOnline(void)
{
    g_k210Online = 1;
    g_k210LastOkMs = g_k210NowMs;
}

static void K210_UART_SendString(const char *str)
{
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(K210_UART_INST, (uint8_t)*str++);
    }
}

static uint8_t K210_Xor7(const uint8_t *data)
{
    uint8_t sum = 0;
    uint8_t i;

    for (i = 0; i < 7U; i++) {
        sum ^= data[i];
    }
    return sum;
}

static uint16_t K210_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void K210_HandleBinaryFrame(const uint8_t *frame)
{
    g_k210RxFrames++;
    if (K210_Xor7(frame) != frame[7]) {
        g_k210ChecksumErrors++;
        return;
    }

    g_k210LatestVision.valid = (K210_ReadU16LE(&frame[5]) != 0U);
    g_k210LatestVision.x = K210_ReadU16LE(&frame[1]);
    g_k210LatestVision.y = K210_ReadU16LE(&frame[3]);
    g_k210LatestVision.area = K210_ReadU16LE(&frame[5]);
    g_k210LatestVision.sequence = ++g_k210VisionFrames;
    g_k210LatestVision.timestamp_ms = g_k210NowMs;
    K210_SetOnline();
}

static void K210_HandleAsciiFrame(const char *frame)
{
    g_k210RxFrames++;
    if (strcmp(frame, K210_HANDSHAKE_RESPONSE) == 0) {
        g_k210HandshakeOk = 1;
        K210_SetOnline();
    }
}

static void K210_ParseByte(uint8_t data)
{
    if (g_k210BinIndex > 0U) {
        g_k210BinBuf[g_k210BinIndex++] = data;
        if (g_k210BinIndex >= K210_BINARY_LEN) {
            K210_HandleBinaryFrame(g_k210BinBuf);
            g_k210BinIndex = 0;
        }
        return;
    }

    if (data == K210_BINARY_HEAD) {
        g_k210BinIndex = 0;
        g_k210BinBuf[g_k210BinIndex++] = data;
        g_k210AsciiActive = 0;
        return;
    }

    if (data == '$') {
        g_k210AsciiActive = 1;
        g_k210AsciiIndex = 0;
        g_k210AsciiBuf[g_k210AsciiIndex++] = (char)data;
        return;
    }

    if (!g_k210AsciiActive) {
        return;
    }

    if (g_k210AsciiIndex >= (K210_ASCII_BUF_SIZE - 1U)) {
        g_k210AsciiActive = 0;
        g_k210AsciiIndex = 0;
        return;
    }

    g_k210AsciiBuf[g_k210AsciiIndex++] = (char)data;
    if (data == '#') {
        g_k210AsciiBuf[g_k210AsciiIndex] = '\0';
        K210_HandleAsciiFrame(g_k210AsciiBuf);
        g_k210AsciiActive = 0;
        g_k210AsciiIndex = 0;
    }
}

void K210_Link_Init(void)
{
    uint8_t data;

    g_k210BinIndex = 0;
    g_k210AsciiIndex = 0;
    g_k210AsciiActive = 0;
    g_k210NowMs = 0;
    g_k210LastHelloMs = 0;
    g_k210LastOkMs = 0;
    g_k210Online = 0;
    g_k210HandshakeOk = 0;
    g_k210RxBytes = 0;
    g_k210RxFrames = 0;
    g_k210VisionFrames = 0;
    g_k210ChecksumErrors = 0;
    memset(&g_k210LatestVision, 0, sizeof(g_k210LatestVision));

    while (DL_UART_Main_receiveDataCheck(K210_UART_INST, &data)) {
    }
    DL_UART_Main_enableInterrupt(K210_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
}

void K210_Link_Tick10ms(void)
{
    g_k210NowMs += 10U;
}

void K210_Link_Process(void)
{
    uint8_t data;
    uint32_t now = g_k210NowMs;

    while (DL_UART_Main_receiveDataCheck(K210_UART_INST, &data)) {
        K210_Link_InputByteFromISR(data);
    }

    if (K210_TimeReached(now, g_k210LastHelloMs, K210_HELLO_PERIOD_MS)) {
        g_k210LastHelloMs = now;
        K210_UART_SendString(K210_HANDSHAKE_REQUEST);
    }

    if (g_k210Online &&
        K210_TimeReached(now, g_k210LastOkMs, K210_ONLINE_TIMEOUT_MS)) {
        g_k210Online = 0;
        g_k210HandshakeOk = 0;
    }
}

void K210_Link_InputByteFromISR(uint8_t data)
{
    g_k210RxBytes++;
    K210_ParseByte(data);
}

uint8_t K210_Link_IsOnline(void)
{
    return g_k210Online;
}

uint8_t K210_Link_IsHandshakeOk(void)
{
    return g_k210HandshakeOk;
}

uint8_t K210_Link_GetLatestVision(K210_VisionFrame *frame)
{
    if ((frame == 0) || (g_k210VisionFrames == 0U)) {
        return 0;
    }
    *frame = g_k210LatestVision;
    return 1;
}

uint32_t K210_Link_RxBytes(void)
{
    return g_k210RxBytes;
}

uint32_t K210_Link_RxFrames(void)
{
    return g_k210RxFrames;
}

uint32_t K210_Link_VisionFrames(void)
{
    return g_k210VisionFrames;
}

uint32_t K210_Link_ChecksumErrors(void)
{
    return g_k210ChecksumErrors;
}
