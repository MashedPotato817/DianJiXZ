#ifndef __LOAD_DETECT_H
#define __LOAD_DETECT_H
#include "ti_msp_dl_config.h"

/*
 * 装载/卸载检测 — Task 3 实现。
 * 传感器方案待 Codex 调研后确定（称重/限位开关/红外遮挡）。
 *
 * 当前为占位模块。
 */

void Load_Detect_Init(void);

/* 主循环调用：去抖 + 状态更新 */
void Load_Detect_Process(void);

/* 药品是否已装载 */
uint8_t Load_Detect_IsLoaded(void);

/* 药品是否已卸载 */
uint8_t Load_Detect_IsUnloaded(void);

#endif
