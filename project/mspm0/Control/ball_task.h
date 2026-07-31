#ifndef H_BALL_TASK
#define H_BALL_TASK

#include <stdint.h>

/*
 * H 题任务状态机。当前实现题目要求 3 的定点运动：
 * 球从中心 O 出发 → 到 +50 mm 稳定 → 折返到 -50 mm 稳定，全程计时。
 * 触发：BLS(PA18) 单击（Ball_Task_StartPoint）。
 */
typedef enum {
    BALL_TASK_IDLE = 0,
    BALL_TASK_POINT_PLUS,   /* 目标 +50 mm，等待稳定 */
    BALL_TASK_POINT_MINUS,  /* 目标 -50 mm，等待稳定 */
    BALL_TASK_POINT_DONE,   /* -50 mm 已稳定，total_ms 有效 */
    BALL_TASK_FAULT         /* 阶段超时 */
} Ball_TaskState;

void Ball_Task_Init(void);
/* 启动定点任务：0 → +50 mm → -50 mm。 */
void Ball_Task_StartPoint(void);
/* 5 ms 控制中断内调用，非阻塞推进。 */
void Ball_Task_Tick5ms(void);
Ball_TaskState Ball_Task_GetState(void);
/* 定点任务总耗时（ms），DONE 后有效。 */
uint32_t Ball_Task_GetTotalMs(void);
/* 各段全程最大 |x-target|（mm），DONE/FAULT 后有效，用于对照 ±10mm 验收。 */
float Ball_Task_GetMaxErrorPlusMm(void);
float Ball_Task_GetMaxErrorMinusMm(void);

#endif
