#ifndef __ROUTE_H
#define __ROUTE_H
#include "ti_msp_dl_config.h"

/*
 * 病房路线表 — Task 4 实现。
 * 将病房号 1-8 映射为路口序列、转向动作和到位判据。
 *
 * 当前为占位模块，待 Task 4 确定场地参数后实现。
 */

void Route_Init(void);

/* 根据目标病房号加载路线 */
void Route_SetTarget(uint8_t ward);

/* OUTBOUND 中调用：检查是否到达目标病房门口 */
uint8_t Route_CheckArrived(void);

/* RETURN 中调用：检查是否回到药房 */
uint8_t Route_CheckHome(void);

#endif
