#ifndef _ELECTROMAGNET_H
#define _ELECTROMAGNET_H
#include "ti_msp_dl_config.h"

/* 继电器控制引脚: PA21 (LQFP-64 pin 15)
 * 如需更换引脚，在 SysConfig 中修改并重新生成 ti_msp_dl_config.h */
#define MAGNET_PORT         RELAY_PORT
#define MAGNET_PIN          RELAY_PIN_RELAY_PIN

void MAGNET_Init(void);
void MAGNET_ON(void);
void MAGNET_OFF(void);
void MAGNET_Toggle(void);
uint8_t MAGNET_GetState(void);

#endif
