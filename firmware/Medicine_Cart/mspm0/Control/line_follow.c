/*
 * 红线巡线模块 — 从 WHEELTEC_C07A_CAR Gray_Mode() 迁移。
 *
 * ==== 待 Codex 实测确认后修改 ====
 * 1. 转向符号：research.md §四 指出当前正号可能让车反向转向，
 *    待通道映射 + 轮子方向试验后，可能需要恢复为负号 (Move_Z = -K*v*curvature)。
 * 2. 红线检测阈值：GRAY_BLACK_LEVEL=1 是黑线逻辑；红线响应电平需 Task 1 实测后确认。
 *    若红线输出与黑线相反，修改 Gray_ToBlack() 或 GRAY_BLACK_LEVEL。
 * 3. 传感器间距 GRAY_SENSOR_PITCH_MM 是否为实际物理值，需与通道映射一起验证。
 * ==== 以上均不改动底盘基线，仅在本文件内修改 ====
 */
#include "line_follow.h"
#include "board.h"

uint16_t Gray_Data[8];
uint16_t Gray_Raw[8];
float Gray_Line_Pos_mm;

static const float Gray_Pos_mm[8] = {
    -3.5f * GRAY_SENSOR_PITCH_MM,
    -2.5f * GRAY_SENSOR_PITCH_MM,
    -1.5f * GRAY_SENSOR_PITCH_MM,
    -0.5f * GRAY_SENSOR_PITCH_MM,
     0.5f * GRAY_SENSOR_PITCH_MM,
     1.5f * GRAY_SENSOR_PITCH_MM,
     2.5f * GRAY_SENSOR_PITCH_MM,
     3.5f * GRAY_SENSOR_PITCH_MM
};

static float lost_search_angle;
static float last_search_move_z;
static uint8_t line_seen;

static void Gray_Select_Channel(uint8_t channel)
{
    if (channel & 0x01) DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    else                DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);

    if (channel & 0x02) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    else                DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);

    if (channel & 0x04) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    else                DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
}

static int Gray_ToBlack(uint32_t pin_state)
{
    int level = pin_state ? 1 : 0;
    return (level == GRAY_BLACK_LEVEL) ? 1 : 0;
}

void Gray_Read_All(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        Gray_Select_Channel(i);
        delay_us(50);
        Gray_Raw[i] = DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) ? 1 : 0;
        Gray_Data[i] = Gray_ToBlack(Gray_Raw[i]);
    }
}

void Line_Follow_Init(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        Gray_Data[i] = 0;
        Gray_Raw[i] = 0;
    }
    Gray_Line_Pos_mm = 0;
    lost_search_angle = 0;
    last_search_move_z = 0;
    line_seen = 0;
}

void Line_Follow_Run(void)
{
    float pos_sum = 0;
    int black_count = 0;
    float y_m;
    float lookahead_m;
    float curvature;
    uint8_t i;

    Gray_Read_All();
    for (i = 0; i < 8; i++) {
        if (Gray_Data[i]) {
            pos_sum += Gray_Pos_mm[i];
            black_count++;
        }
    }

    if (black_count == 0) {
        Gray_Line_Pos_mm = 0;
        if ((!line_seen) || (last_search_move_z == 0.0f) ||
            (lost_search_angle >= GRAY_LOST_SEARCH_MAX_ANGLE_RAD)) {
            Move_X = 0;
            Move_Z = 0;
        } else {
            Move_X = 0;
            Move_Z = last_search_move_z;
            lost_search_angle += GRAY_LOST_SEARCH_ANGULAR_SPEED / Frequency;
        }
        return;
    }

    line_seen = 1;
    lost_search_angle = 0;
    Gray_Line_Pos_mm = pos_sum / black_count;
    Move_X = GRAY_BASE_SPEED_MM_S / 1000.0f;

    y_m = Gray_Line_Pos_mm / 1000.0f;
    lookahead_m = GRAY_SENSOR_FORWARD_MM / 1000.0f;
    curvature = (2.0f * y_m) / (lookahead_m * lookahead_m + y_m * y_m);

    /*
     * TODO(Codex): 通道映射试验后确认符号。
     * 历史高速版用负号：Move_Z = -GRAY_STEER_GAIN * Move_X * curvature
     * 当前暂用正号（与 WHEELTEC_C07A_CAR 基线一致）。
     */
    Move_Z = GRAY_STEER_GAIN * Move_X * curvature;

    if (Move_Z > GRAY_MAX_ANGULAR_SPEED)  Move_Z = GRAY_MAX_ANGULAR_SPEED;
    if (Move_Z < -GRAY_MAX_ANGULAR_SPEED) Move_Z = -GRAY_MAX_ANGULAR_SPEED;

    if (Move_Z > 0.0f)       last_search_move_z = GRAY_LOST_SEARCH_ANGULAR_SPEED;
    else if (Move_Z < 0.0f)  last_search_move_z = -GRAY_LOST_SEARCH_ANGULAR_SPEED;
}
