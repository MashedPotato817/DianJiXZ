#ifndef H_BALL_TASK
#define H_BALL_TASK

#include <stdint.h>

typedef enum {
    BALL_TASK_IDLE = 0,
    BALL_TASK_CHASSIS_LAP,
    BALL_TASK_HOLD_CENTER,
    BALL_TASK_LAP_HOLD_CENTER,
    BALL_TASK_LAP_HOLD_TARGET,
    BALL_TASK_FAULT
} Ball_TaskState;

void Ball_Task_Init(void);
void Ball_Task_SetState(Ball_TaskState state);
Ball_TaskState Ball_Task_GetState(void);

#endif
