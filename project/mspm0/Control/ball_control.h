#ifndef H_BALL_CONTROL
#define H_BALL_CONTROL

#include "k230_link.h"

typedef struct {
    float kp;
    float kd;
    float target_x_mm;
    float last_error_mm;
} Ball_Control;

void Ball_Control_Init(void);
void Ball_Control_SetTarget(float target_x_mm);
void Ball_Control_Step(const K230_BallPosition *position, float period_s);
const Ball_Control *Ball_Control_Get(void);

#endif
