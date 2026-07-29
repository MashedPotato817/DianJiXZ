#include "ball_task.h"

static Ball_TaskState g_ball_task_state;

void Ball_Task_Init(void)
{
    g_ball_task_state = BALL_TASK_IDLE;
}

void Ball_Task_SetState(Ball_TaskState state)
{
    g_ball_task_state = state;
}

Ball_TaskState Ball_Task_GetState(void)
{
    return g_ball_task_state;
}
