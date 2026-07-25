#ifndef __INDICATOR_H
#define __INDICATOR_H
#include "ti_msp_dl_config.h"

/*
 * 红绿指示灯 — 待分配 GPIO 后实现。
 * 当前为占位模块。
 */

void Indicator_Init(void);
void Indicator_Red_On(void);
void Indicator_Red_Off(void);
void Indicator_Green_On(void);
void Indicator_Green_Off(void);
void Indicator_Both_Off(void);

#endif
