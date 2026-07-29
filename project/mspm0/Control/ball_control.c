#include "ball_control.h"
#include "servo.h"

static Ball_Control g_ball_control;

void Ball_Control_Init(void)
{
    g_ball_control.kp = 0.0f;
    g_ball_control.kd = 0.0f;
    g_ball_control.target_x_mm = 0.0f;
    g_ball_control.last_error_mm = 0.0f;
}

void Ball_Control_SetTarget(float target_x_mm)
{
    g_ball_control.target_x_mm = target_x_mm;
    g_ball_control.last_error_mm = 0.0f;
}

void Ball_Control_Step(const K230_BallPosition *position, float period_s)
{
    float error;
    float derivative;
    float output;

    if ((position == 0) || (position->valid == 0U) || (period_s <= 0.0f)) {
        return;
    }

    error = g_ball_control.target_x_mm - position->x_mm;
    derivative = (error - g_ball_control.last_error_mm) / period_s;
    output = g_ball_control.kp * error + g_ball_control.kd * derivative;
    Servo_SetTargetAngle(output);
    g_ball_control.last_error_mm = error;
}

const Ball_Control *Ball_Control_Get(void)
{
    return &g_ball_control;
}
