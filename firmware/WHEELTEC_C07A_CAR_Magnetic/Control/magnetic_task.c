#include "magnetic_task.h"
#include "electromagnet.h"

/* 吸取时间配置 (ms) */
#define MAGNET_SUCK_TIMEOUT_MS   500   /* 吸取建立时间 */
#define MAGNET_RELEASE_TIMEOUT_MS 300  /* 释放确认时间 */

MagnetState magnet_state = MAGNET_IDLE;

static uint32_t magnet_timer_ms = 0;
static uint32_t magnet_tick_ms  = 0;

/* 由 5ms 定时器 ISR 调用，递增毫秒计数器 */
void Magnetic_Task_Tick5ms(void)
{
    magnet_tick_ms += 5;
}

void Magnetic_Task_Init(void)
{
    MAGNET_Init();
    magnet_state    = MAGNET_IDLE;
    magnet_timer_ms = 0;
    magnet_tick_ms  = 0;
}

void Magnetic_Command_Suck(void)
{
    if (magnet_state == MAGNET_IDLE || magnet_state == MAGNET_RELEASE) {
        magnet_state    = MAGNET_SUCK;
        magnet_timer_ms = magnet_tick_ms;
        MAGNET_ON();
    }
}

void Magnetic_Command_Release(void)
{
    if (magnet_state == MAGNET_HOLD || magnet_state == MAGNET_SUCK) {
        magnet_state    = MAGNET_RELEASE;
        magnet_timer_ms = magnet_tick_ms;
        MAGNET_OFF();
    }
}

void Magnetic_Command_Toggle(void)
{
    if (magnet_state == MAGNET_HOLD || magnet_state == MAGNET_SUCK) {
        Magnetic_Command_Release();
    } else {
        Magnetic_Command_Suck();
    }
}

/* 按键事件处理
 * key_event: 1=单击, 2=双击, 3=长按
 * 在当前设计中，长按切换电磁铁状态 */
void Magnetic_Key_Handler(uint8_t key_event)
{
    if (key_event == 3) {  /* 长按 */
        Magnetic_Command_Toggle();
    }
}

void Magnetic_Task_Process(void)
{
    uint32_t elapsed;

    switch (magnet_state) {
    case MAGNET_SUCK:
        elapsed = magnet_tick_ms - magnet_timer_ms;
        if (elapsed >= MAGNET_SUCK_TIMEOUT_MS) {
            magnet_state = MAGNET_HOLD;
        }
        break;

    case MAGNET_RELEASE:
        elapsed = magnet_tick_ms - magnet_timer_ms;
        if (elapsed >= MAGNET_RELEASE_TIMEOUT_MS) {
            magnet_state = MAGNET_IDLE;
        }
        break;

    case MAGNET_HOLD:
    case MAGNET_IDLE:
    default:
        break;
    }
}
