#include "electromagnet.h"

void MAGNET_Init(void)
{
    /* 已在 SYSCFG_DL_GPIO_init() 中初始化
     * 这里确保默认为释放状态（低电平） */
    DL_GPIO_clearPins(MAGNET_PORT, MAGNET_PIN);
}

void MAGNET_ON(void)
{
    DL_GPIO_setPins(MAGNET_PORT, MAGNET_PIN);
}

void MAGNET_OFF(void)
{
    DL_GPIO_clearPins(MAGNET_PORT, MAGNET_PIN);
}

void MAGNET_Toggle(void)
{
    DL_GPIO_togglePins(MAGNET_PORT, MAGNET_PIN);
}

uint8_t MAGNET_GetState(void)
{
    return (DL_GPIO_readPins(MAGNET_PORT, MAGNET_PIN) != 0) ? 1 : 0;
}
