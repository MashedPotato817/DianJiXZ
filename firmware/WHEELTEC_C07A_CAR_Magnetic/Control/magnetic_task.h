#ifndef _MAGNETIC_TASK_H
#define _MAGNETIC_TASK_H
#include "board.h"

/* 电磁铁吸取状态机 */
typedef enum {
    MAGNET_IDLE = 0,      /* 空闲，等待指令 */
    MAGNET_SUCK,          /* 吸取中 */
    MAGNET_HOLD,          /* 保持吸取 */
    MAGNET_RELEASE,       /* 释放 */
} MagnetState;

extern MagnetState magnet_state;

void Magnetic_Task_Init(void);
void Magnetic_Task_Process(void);
void Magnetic_Command_Suck(void);
void Magnetic_Command_Release(void);
void Magnetic_Command_Toggle(void);
void Magnetic_Key_Handler(uint8_t key_event);

#endif
