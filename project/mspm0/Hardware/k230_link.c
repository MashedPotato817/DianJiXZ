#include "k230_link.h"

static K230_BallPosition g_ball_position;

void K230_Link_Init(void)
{
    g_ball_position.x_mm = 0.0f;
    g_ball_position.y_mm = 0.0f;
    g_ball_position.valid = 0U;
    g_ball_position.timestamp_ms = 0U;
}

void K230_Link_UpdatePosition(float x_mm, float y_mm, uint8_t valid,
                              uint32_t timestamp_ms)
{
    g_ball_position.x_mm = x_mm;
    g_ball_position.y_mm = y_mm;
    g_ball_position.valid = valid;
    g_ball_position.timestamp_ms = timestamp_ms;
}

void K230_Link_GetPosition(K230_BallPosition *position)
{
    if (position != 0) {
        *position = g_ball_position;
    }
}
