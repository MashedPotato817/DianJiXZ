/*
 * 舵机摆动测试 — 验证丝杆/齿轮条机械结构
 *
 * 接线:
 *   PA8  → 舵机信号线 (白/橙)
 *   5V   → 舵机电源线 (红) — 必须外接, 板载3.3V带不动!
 *   GND  → 舵机地线 (棕/黑) + 电源地 — 务必共地
 *   PB6  → USB转TTL RX (调试串口, 9600bps)
 *   PB7  → USB转TTL TX
 *
 * 测试内容:
 *   舵机在 0°~90° 之间来回平滑扫描
 *   90° = 齿轮条约 2.5cm 位移
 *
 * 构建: 打开 keil/Servo_Demo.uvprojx, F7编译, F8烧录
 */

#include <stdio.h>
#include "ti_msp_dl_config.h"
#include "servo.h"

/* ---- printf → UART_1 (PB6/PB7, 9600bps) ---- */
int fputc(int ch, FILE *stream)
{
    while (DL_UART_Main_isBusy(UART_1_INST) == true);
    DL_UART_Main_transmitDataBlocking(UART_1_INST, ch);
    return ch;
}

/* ---- SysTick 毫秒计数 ---- */
static volatile uint32_t g_tick_ms;

void SysTick_Handler(void)
{
    g_tick_ms++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_tick_ms;
    while ((g_tick_ms - start) < ms);
}

/* ================================================================ */

int main(void)
{
    SYSCFG_DL_init();   /* 系统时钟 / GPIO / UART / SysTick (1ms) */
    Servo_Init();       /* TIMA0 → PA8, 50Hz PWM, 默认 90° */

    printf("\r\n===========================================\r\n");
    printf("  Servo Swing Test — 结构验证\r\n");
    printf("  90 deg = 2.5cm 齿轮条位移\r\n");
    printf("  PA8=TIMA0_CCP0, 50Hz PWM\r\n");
    printf("===========================================\r\n\r\n");

    uint8_t  angle  = 0;
    int8_t   dir    = 1;          /* 1=角度增大, -1=减小 */
    uint32_t tick   = g_tick_ms;

    while (1) {
        /* 每 15ms 步进 1°, 平滑扫描 */
        if ((g_tick_ms - tick) >= 15) {
            tick = g_tick_ms;

            Servo_SetAngle(angle);

            float mm = SERVO_ANGLE_TO_MM(angle);
            printf("  servo=%3u deg  rack=%.1f mm\r\n", angle, mm);

            angle += dir;

            /* 到达端点时反向 */
            if (angle >= 90) {
                angle = 90;
                dir   = -1;
            } else if (angle == 0) {
                dir = 1;
            }
        }
    }
}
