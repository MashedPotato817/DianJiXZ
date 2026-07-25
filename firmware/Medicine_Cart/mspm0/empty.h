#ifndef __EMPTY_H
#define __EMPTY_H
#include "board.h"

/* 主循环用 5ms 时基计数器（TIMG0 ISR 递增） */
extern volatile uint32_t g_sysTick5ms;

#endif
