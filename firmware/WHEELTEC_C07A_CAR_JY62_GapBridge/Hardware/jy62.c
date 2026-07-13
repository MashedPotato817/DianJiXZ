#include "jy62.h"
#include "ti_msp_dl_config.h"

#define JY62_FRAME_LEN          11
#define JY62_FRAME_HEAD         0x55
#define JY62_FRAME_ACC          0x51
#define JY62_FRAME_GYRO         0x52
#define JY62_FRAME_ANGLE        0x53
#define JY62_ONLINE_TIMEOUT_MS  500
#define JY62_FIXED_BAUD         115200U

static uint8_t g_jy62Frame[JY62_FRAME_LEN];
static uint8_t g_jy62Index;
static uint32_t g_jy62NowMs;
static uint32_t g_jy62LastFrameMs;
static uint32_t g_jy62LastGyroMs;
static JY62_Data g_jy62Data;

static void JY62_ClearRxFifo(void)
{
    uint8_t data;

    while (DL_UART_Main_receiveDataCheck(UART_1_INST, &data)) {
    }
}

static void JY62_ResetParser(void)
{
    g_jy62Index = 0;
    g_jy62Data.online = 0;
}

static int16_t JY62_ReadI16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint8_t JY62_ChecksumOk(const uint8_t *frame)
{
    uint8_t sum = 0;
    uint8_t i;

    for (i = 0; i < (JY62_FRAME_LEN - 1); i++) {
        sum = (uint8_t)(sum + frame[i]);
    }

    return (sum == frame[JY62_FRAME_LEN - 1]);
}

static void JY62_SetOnline(void)
{
    g_jy62LastFrameMs = g_jy62NowMs;
    g_jy62Data.online = 1;
}

static void JY62_HandleFrame(const uint8_t *frame)
{
    int16_t x;
    int16_t y;
    int16_t z;

    if (!JY62_ChecksumOk(frame)) {
        g_jy62Data.checksum_errors++;
        return;
    }

    x = JY62_ReadI16(&frame[2]);
    y = JY62_ReadI16(&frame[4]);
    z = JY62_ReadI16(&frame[6]);

    switch (frame[1]) {
    case JY62_FRAME_ACC:
        g_jy62Data.ax_g = (float)x / 32768.0f * 16.0f;
        g_jy62Data.ay_g = (float)y / 32768.0f * 16.0f;
        g_jy62Data.az_g = (float)z / 32768.0f * 16.0f;
        break;
    case JY62_FRAME_GYRO:
        g_jy62Data.wx_dps = (float)x / 32768.0f * 2000.0f;
        g_jy62Data.wy_dps = (float)y / 32768.0f * 2000.0f;
        g_jy62Data.wz_dps = (float)z / 32768.0f * 2000.0f;
        if (g_jy62LastGyroMs != 0U) {
            float dt_s = (float)((uint32_t)(g_jy62NowMs - g_jy62LastGyroMs)) / 1000.0f;
            if ((dt_s > 0.0f) && (dt_s < 0.2f)) {
                g_jy62Data.gyro_roll_int_deg += g_jy62Data.wx_dps * dt_s;
                g_jy62Data.gyro_pitch_int_deg += g_jy62Data.wy_dps * dt_s;
                g_jy62Data.gyro_yaw_int_deg += g_jy62Data.wz_dps * dt_s;
            }
        }
        g_jy62LastGyroMs = g_jy62NowMs;
        break;
    case JY62_FRAME_ANGLE:
        g_jy62Data.roll_deg = (float)x / 32768.0f * 180.0f;
        g_jy62Data.pitch_deg = (float)y / 32768.0f * 180.0f;
        g_jy62Data.yaw_deg = (float)z / 32768.0f * 180.0f;
        break;
    default:
        return;
    }

    g_jy62Data.valid_frames++;
    JY62_SetOnline();
}

void JY62_InputByteFromISR(uint8_t data)
{
    g_jy62Data.rx_bytes++;

    if (g_jy62Index == 0) {
        if (data != JY62_FRAME_HEAD) {
            return;
        }
        g_jy62Frame[g_jy62Index++] = data;
        return;
    }

    if (g_jy62Index == 1) {
        if ((data != JY62_FRAME_ACC) && (data != JY62_FRAME_GYRO) &&
            (data != JY62_FRAME_ANGLE)) {
            if (data == JY62_FRAME_HEAD) {
                g_jy62Frame[0] = data;
                g_jy62Index = 1;
            } else {
                g_jy62Index = 0;
            }
            return;
        }
    }

    if ((g_jy62Index >= 2U) && (data == JY62_FRAME_HEAD)) {
        g_jy62Frame[0] = data;
        g_jy62Index = 1;
        return;
    }

    g_jy62Frame[g_jy62Index++] = data;

    if (g_jy62Index >= JY62_FRAME_LEN) {
        JY62_HandleFrame(g_jy62Frame);
        g_jy62Index = 0;
    }
}

void JY62_Init(void)
{
    g_jy62Data = (JY62_Data){0};
    g_jy62Index = 0;
    g_jy62NowMs = 0;
    g_jy62LastFrameMs = 0;
    g_jy62LastGyroMs = 0;
    JY62_ClearRxFifo();
    JY62_ResetParser();
}

void JY62_Tick10ms(void)
{
    g_jy62NowMs += 10;
}

void JY62_Process(void)
{
    uint8_t data;

    while (DL_UART_Main_receiveDataCheck(UART_1_INST, &data)) {
        JY62_InputByteFromISR(data);
    }

    if (g_jy62Data.online &&
        ((uint32_t)(g_jy62NowMs - g_jy62LastFrameMs) > JY62_ONLINE_TIMEOUT_MS)) {
        g_jy62Data.online = 0;
    }
}

uint8_t JY62_IsOnline(void)
{
    return g_jy62Data.online;
}

uint8_t JY62_GetData(JY62_Data *data)
{
    if (data == 0) {
        return 0;
    }

    *data = g_jy62Data;
    return g_jy62Data.online;
}

int16_t JY62_GetYawDegX10(void)
{
    return (int16_t)(g_jy62Data.yaw_deg * 10.0f);
}

uint32_t JY62_GetRxBytes(void)
{
    return g_jy62Data.rx_bytes;
}

uint32_t JY62_GetValidFrames(void)
{
    return g_jy62Data.valid_frames;
}

uint32_t JY62_GetChecksumErrors(void)
{
    return g_jy62Data.checksum_errors;
}

uint32_t JY62_GetCurrentBaud(void)
{
    return JY62_FIXED_BAUD;
}
