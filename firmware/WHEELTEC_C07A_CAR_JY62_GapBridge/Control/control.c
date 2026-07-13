/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "control.h"
#include "jy62.h"
#include "k210_link.h"
#include <math.h>

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
uint16_t Gray_Data[8];
uint16_t Gray_Raw[8];
float Gray_Line_Pos_mm;
volatile uint8_t Gray_Nav_Mode = GRAY_NAV_MODE_LINE;
volatile int16_t Gray_YawDegX10 = 0;
volatile int16_t Gray_BridgeErrDegX10 = 0;
volatile uint16_t Gray_BridgeDistance_mm = 0;
volatile uint8_t Gray_BlackCount_Debug = 0;
volatile uint8_t Gray_Task_Mode = GRAY_TASK_4_CCW_4LAP;
volatile uint8_t Gray_Task_Lap = 0;
volatile uint8_t Gray_Task_TargetLap = 1;
volatile uint8_t Gray_Task_PointCount = 0;
volatile uint8_t Gray_LastPointChar = 'A';
volatile int32_t Gray_PoseX_mm = 0;
volatile int32_t Gray_PoseY_mm = 0;
volatile int32_t Gray_PoseDistance_mm = 0;
volatile int16_t Gray_PoseHeadingDegX10 = 0;
volatile int16_t Gray_GyroZBiasDpsX10 = 0;

Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
float Velocity_KP=400,Velocity_KI=300;	
int Run_Mode=1;//小车运行模式
u8 Flag_Stop=1;//小车启动标志位

#define GRAY_BLACK_LEVEL              1
#define GRAY_BASE_SPEED_MM_S          300.0f
#define GRAY_MID_SPEED_MM_S           240.0f
#define GRAY_CURVE_SPEED_MM_S         205.0f
#define GRAY_CROSS_SPEED_MM_S         310.0f
#define GRAY_LOST_SEARCH_SPEED_MM_S   0.0f
#define GRAY_SENSOR_PITCH_MM          10.0f
#define GRAY_SENSOR_FORWARD_MM        150.0f
#define GRAY_CAR_WHEELBASE_MM         105.0f
#define GRAY_STEER_GAIN               1.70f
#define GRAY_MAX_ANGULAR_SPEED        3.60f
#define GRAY_LOST_TURN_SPEED          1.80f
#define GRAY_LOST_POS_DEADBAND_MM     3.0f
#define GRAY_MID_POS_MM               10.0f
#define GRAY_CURVE_POS_MM             22.0f
#define GRAY_CROSS_BLACK_COUNT        7
#define GRAY_CROSS_DETECT_COUNT       2
#define GRAY_CROSS_RUN_TICKS          32
#define GRAY_CROSS_COOLDOWN_TICKS     60
#define GRAY_BRIDGE_ENTRY_SPEED_MM_S  280.0f
#define GRAY_BRIDGE_RECOVER_SPEED_MM_S 160.0f
#define GRAY_BRIDGE_EXPECT_DISTANCE_MM 1050.0f
#define GRAY_BRIDGE_SEARCH_DISTANCE_MM 1180.0f
#define GRAY_BRIDGE_MAX_DISTANCE_MM   1700.0f
#define GRAY_BRIDGE_MIN_STABLE_TICKS  8U
#define GRAY_BRIDGE_REACQUIRE_TICKS   1U
#define GRAY_BRIDGE_HEADING_KP        0.075f
#define GRAY_BRIDGE_GYRO_KD           0.010f
#define GRAY_BRIDGE_MAX_TURN_SPEED    2.20f
#define GRAY_BRIDGE_SEARCH_TURN_SPEED 0.42f
#define GRAY_BRIDGE_ALIGN_TICKS       110U
#define GRAY_BRIDGE_ALIGN_ERR_DEG     5.0f
#define GRAY_BRIDGE_ALIGN_SPEED_MM_S  60.0f
#define GRAY_LINE_GYRO_KD             0.006f
#define GRAY_LINE_YAW_ALPHA           0.18f
#define GRAY_LINE_YAW_CENTER_MM       14.0f
#define GRAY_ARC_MIN_TURN_DEG         135.0f
#define GRAY_ARC_TURN_MIN_DPS         8.0f
#define GRAY_DIAGONAL_EXIT_TURN_DEG   42.0f
#define GRAY_NOTIFY_TOGGLE_TICKS      10U
#define GRAY_NOTIFY_TOGGLE_TOTAL      6U
#define GRAY_POSE_DT_S                0.005f
#define GRAY_POSE_DEG_TO_RAD          0.0174532925f
#define GRAY_GYRO_BIAS_ALPHA          0.002f
#define GRAY_GYRO_BIAS_LEARN_MAX_DPS  2.0f
#define GRAY_GYRO_DEADBAND_DPS        0.5f
#define GRAY_TASK4_START_HEADING_DEG   180.0f
#define GRAY_TASK4_D_TANGENT_DEG       0.0f
#define GRAY_TASK4_D_TO_B_DEG          38.7f
#define GRAY_TASK4_C_TANGENT_DEG       180.0f
#define GRAY_TASK4_C_TO_A_DEG          141.3f
#define GRAY_TASK4_POINT_TOL_M         0.25f
#define GRAY_TASK4_WHITE_TICKS         3U
#define GRAY_TASK4_ALIGN_TICKS_MAX     500U
#define GRAY_TASK4_ALIGN_STABLE_TICKS  5U
#define GRAY_TASK4_ALIGN_ERR_DEG       4.0f
#define GRAY_TASK4_BRIDGE_SPEED_MM_S   280.0f
#define GRAY_TASK4_BRIDGE_MAX_MM       1600.0f
#define GRAY_TASK4_LOST_CREEP_MM_S     80.0f

typedef enum {
    GRAY_ROUTE_IDLE = 0,
    GRAY_ROUTE_BRIDGE_TOP_LR,
    GRAY_ROUTE_ARC_RIGHT_DOWN,
    GRAY_ROUTE_BRIDGE_BOTTOM_RL,
    GRAY_ROUTE_ARC_LEFT_UP,
    GRAY_ROUTE_ARC_LEFT_DOWN,
    GRAY_ROUTE_BRIDGE_BOTTOM_LR,
    GRAY_ROUTE_ARC_RIGHT_UP,
    GRAY_ROUTE_BRIDGE_TOP_RL
} GrayRouteSegment_t;

typedef enum {
    GRAY_TASK4_LINE_AD = 0,
    GRAY_TASK4_ALIGN_D_TANGENT,
    GRAY_TASK4_ALIGN_D_TO_B,
    GRAY_TASK4_BRIDGE_D_TO_B,
    GRAY_TASK4_LINE_BC,
    GRAY_TASK4_ALIGN_C_TANGENT,
    GRAY_TASK4_ALIGN_C_TO_A,
    GRAY_TASK4_BRIDGE_C_TO_A,
    GRAY_TASK4_ALIGN_A_TANGENT
} GrayTask4State_t;

static GrayRouteSegment_t g_grayRoute = GRAY_ROUTE_IDLE;
static uint8_t g_grayMissionDone = 0;
static uint8_t g_grayNotifyTicks = 0;
static uint8_t g_grayNotifyToggleCount = 0;
static uint8_t g_grayNeedReset = 1;
static uint8_t g_grayPrepArcToA = 0;
static float g_grayPoseX_m = 0;
static float g_grayPoseY_m = 0;
static float g_grayPoseHeadingDeg = 0;
static float g_grayPoseDistance_m = 0;
static float g_grayGyroZBiasDps = 0;
static float g_grayPoseYawOriginDeg = 0;
static uint8_t g_grayPoseYawOriginValid = 0;
static float g_grayBridgeBaseYawDeg = 0;
static uint8_t g_grayBridgeBaseYawValid = 0;
static float g_grayLineYawRefDeg = 0;
static uint8_t g_grayLineYawRefValid = 0;
static int8_t g_grayArcTurnDir = 0;
static GrayTask4State_t g_grayTask4State = GRAY_TASK4_LINE_AD;
static float g_grayTask4LastLinePos_mm = 0;
static int8_t g_grayTask4LastSearchDir = -1;
static uint8_t g_grayTask4WhiteTicks = 0;
static uint16_t g_grayTask4AlignTicks = 0;
static uint8_t g_grayTask4AlignStableTicks = 0;
static float g_grayTask4BridgeDistance_mm = 0;
static uint8_t g_grayTask4ReacquireTicks = 0;

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

static float Gray_NormalizeYawDeg(float yaw_deg)
{
    while (yaw_deg > 180.0f) {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg < -180.0f) {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}

static void Gray_LineYawRefUpdate(float yaw_deg)
{
    float err_deg;

    if (!g_grayLineYawRefValid) {
        g_grayLineYawRefDeg = yaw_deg;
        g_grayLineYawRefValid = 1;
        return;
    }

    err_deg = Gray_NormalizeYawDeg(yaw_deg - g_grayLineYawRefDeg);
    g_grayLineYawRefDeg = Gray_NormalizeYawDeg(g_grayLineYawRefDeg + GRAY_LINE_YAW_ALPHA * err_deg);
}

static uint8_t Gray_RouteIsArc(GrayRouteSegment_t route)
{
    return ((route == GRAY_ROUTE_ARC_RIGHT_DOWN) ||
            (route == GRAY_ROUTE_ARC_LEFT_UP) ||
            (route == GRAY_ROUTE_ARC_LEFT_DOWN) ||
            (route == GRAY_ROUTE_ARC_RIGHT_UP)) ? 1U : 0U;
}

static float Gray_AbsDeg(float deg)
{
    return (deg >= 0.0f) ? deg : -deg;
}

static float Gray_BridgeYawRefGet(float current_yaw_deg)
{
    if ((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
        (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP)) {
        float exit_yaw_deg = g_grayLineYawRefValid ? g_grayLineYawRefDeg : current_yaw_deg;
        float turn_dir;

        switch (g_grayRoute) {
        case GRAY_ROUTE_BRIDGE_BOTTOM_LR:
        case GRAY_ROUTE_BRIDGE_BOTTOM_RL:
            if (g_grayArcTurnDir != 0) {
                turn_dir = (float)g_grayArcTurnDir;
            } else {
                turn_dir = (g_grayRoute == GRAY_ROUTE_BRIDGE_BOTTOM_LR) ? -1.0f : 1.0f;
            }
            return Gray_NormalizeYawDeg(exit_yaw_deg +
                                        turn_dir * GRAY_DIAGONAL_EXIT_TURN_DEG);

        default:
            return exit_yaw_deg;
        }
    }

    if (!g_grayBridgeBaseYawValid) {
        g_grayBridgeBaseYawDeg = current_yaw_deg;
        g_grayBridgeBaseYawValid = 1;
    }

    switch (g_grayRoute) {
    case GRAY_ROUTE_BRIDGE_TOP_LR:
    case GRAY_ROUTE_BRIDGE_BOTTOM_LR:
        return g_grayBridgeBaseYawDeg;

    case GRAY_ROUTE_BRIDGE_BOTTOM_RL:
    case GRAY_ROUTE_BRIDGE_TOP_RL:
        return Gray_NormalizeYawDeg(g_grayBridgeBaseYawDeg + 180.0f);

    default:
        return current_yaw_deg;
    }
}

static uint8_t Gray_TaskTargetLapGet(void)
{
    if (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP) {
        return 4U;
    }
    return 1U;
}

static GrayRouteSegment_t Gray_TaskStartRouteGet(void)
{
    if ((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
        (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP)) {
        return GRAY_ROUTE_ARC_LEFT_UP;
    }
    return GRAY_ROUTE_BRIDGE_TOP_LR;
}

static void Gray_StartNotify(void)
{
    g_grayNotifyTicks = GRAY_NOTIFY_TOGGLE_TICKS;
    g_grayNotifyToggleCount = GRAY_NOTIFY_TOGGLE_TOTAL;
    LED_ON();
}

static void Gray_PosePublish(void)
{
    Gray_PoseX_mm = (int32_t)(g_grayPoseX_m * 1000.0f);
    Gray_PoseY_mm = (int32_t)(g_grayPoseY_m * 1000.0f);
    Gray_PoseDistance_mm = (int32_t)(g_grayPoseDistance_m * 1000.0f);
    Gray_PoseHeadingDegX10 = (int16_t)(Gray_NormalizeYawDeg(g_grayPoseHeadingDeg) * 10.0f);
    Gray_GyroZBiasDpsX10 = (int16_t)(g_grayGyroZBiasDps * 10.0f);
}

static void Gray_PoseReset(void)
{
    g_grayPoseX_m = 0;
    g_grayPoseY_m = 0;
    g_grayPoseHeadingDeg = (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP) ?
                          GRAY_TASK4_START_HEADING_DEG : 0;
    g_grayPoseDistance_m = 0;
    g_grayPoseYawOriginDeg = 0;
    g_grayPoseYawOriginValid = 0;
    Gray_PosePublish();
}

static void Gray_PoseUpdate(void)
{
    JY62_Data data;
    float wz_dps;
    float speed_mps;
    float step_m;
    float heading_rad;

    if (!JY62_GetData(&data)) {
        return;
    }

    wz_dps = data.wz_dps;
    if ((wz_dps > -GRAY_GYRO_BIAS_LEARN_MAX_DPS) &&
        (wz_dps < GRAY_GYRO_BIAS_LEARN_MAX_DPS) &&
        Flag_Stop) {
        g_grayGyroZBiasDps += GRAY_GYRO_BIAS_ALPHA * (wz_dps - g_grayGyroZBiasDps);
    }

    wz_dps -= g_grayGyroZBiasDps;
    if ((wz_dps > -GRAY_GYRO_DEADBAND_DPS) && (wz_dps < GRAY_GYRO_DEADBAND_DPS)) {
        wz_dps = 0.0f;
    }

    if (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP) {
        if (!g_grayPoseYawOriginValid) {
            g_grayPoseYawOriginDeg = data.yaw_deg;
            g_grayPoseYawOriginValid = 1;
        }
        g_grayPoseHeadingDeg = Gray_NormalizeYawDeg(
            GRAY_TASK4_START_HEADING_DEG +
            Gray_NormalizeYawDeg(data.yaw_deg - g_grayPoseYawOriginDeg));
    } else {
        g_grayPoseHeadingDeg += wz_dps * GRAY_POSE_DT_S;
        g_grayPoseHeadingDeg = Gray_NormalizeYawDeg(g_grayPoseHeadingDeg);
    }

    if (Flag_Stop) {
        Gray_PosePublish();
        return;
    }

    speed_mps = (MotorA.Current_Encoder + MotorB.Current_Encoder) * 0.5f;
    step_m = speed_mps * GRAY_POSE_DT_S;
    heading_rad = g_grayPoseHeadingDeg * GRAY_POSE_DEG_TO_RAD;

    g_grayPoseX_m += step_m * cosf(heading_rad);
    g_grayPoseY_m += step_m * sinf(heading_rad);
    if (step_m >= 0.0f) {
        g_grayPoseDistance_m += step_m;
    } else {
        g_grayPoseDistance_m -= step_m;
    }

    Gray_PosePublish();
}

static void Gray_StopMission(void)
{
    g_grayMissionDone = 1;
    Flag_Stop = 1;
    Move_X = 0;
    Move_Z = 0;
    Get_Target_Encoder(0, 0);
    Gray_Nav_Mode = GRAY_NAV_MODE_LINE;
    Gray_BridgeDistance_mm = 0;
    Gray_BridgeErrDegX10 = 0;
    Gray_StartNotify();
}

static void Gray_RecordPoint(uint8_t point_char)
{
    Gray_LastPointChar = point_char;
    if (Gray_Task_PointCount < 255U) {
        Gray_Task_PointCount++;
    }
    Gray_StartNotify();
}

static void Gray_ResetTaskState(void)
{
    Gray_Task_TargetLap = Gray_TaskTargetLapGet();
    Gray_Task_Lap = 0;
    Gray_Task_PointCount = 0;
    Gray_LastPointChar = 'A';
    Gray_Nav_Mode = GRAY_NAV_MODE_LINE;
    Gray_BridgeDistance_mm = 0;
    Gray_BridgeErrDegX10 = 0;
    g_grayMissionDone = 0;
    g_grayRoute = Gray_TaskStartRouteGet();
    g_grayPrepArcToA = ((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
                        (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP)) ? 1U : 0U;
    g_grayNotifyTicks = 0;
    g_grayNotifyToggleCount = 0;
    g_grayNeedReset = 1;
    g_grayBridgeBaseYawDeg = 0;
    g_grayBridgeBaseYawValid = 0;
    g_grayLineYawRefDeg = 0;
    g_grayLineYawRefValid = 0;
    g_grayArcTurnDir = 0;
    g_grayTask4State = GRAY_TASK4_LINE_AD;
    g_grayTask4LastLinePos_mm = 0;
    g_grayTask4LastSearchDir = -1;
    g_grayTask4WhiteTicks = 0;
    g_grayTask4AlignTicks = 0;
    g_grayTask4AlignStableTicks = 0;
    g_grayTask4BridgeDistance_mm = 0;
    g_grayTask4ReacquireTicks = 0;
    Gray_PoseReset();
    LED_OFF();
}

void Gray_TaskTick(void)
{
    if (g_grayNotifyToggleCount == 0U) {
        return;
    }

    if (g_grayNotifyTicks > 0U) {
        g_grayNotifyTicks--;
        return;
    }

    LED_Toggle();
    g_grayNotifyTicks = GRAY_NOTIFY_TOGGLE_TICKS;
    g_grayNotifyToggleCount--;
    if (g_grayNotifyToggleCount == 0U) {
        LED_OFF();
    }
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

static uint8_t Gray_OnBridgeStart(void)
{
    switch (g_grayRoute) {
    case GRAY_ROUTE_ARC_RIGHT_DOWN:
        Gray_RecordPoint('C');
        g_grayRoute = GRAY_ROUTE_BRIDGE_BOTTOM_RL;
        break;
    case GRAY_ROUTE_ARC_LEFT_UP:
        Gray_RecordPoint('A');
        if ((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
            (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP)) {
            uint8_t was_prep_arc = g_grayPrepArcToA;
            if (g_grayPrepArcToA) {
                g_grayPrepArcToA = 0;
            } else if (Gray_Task_Lap < 255U) {
                Gray_Task_Lap++;
            }

            if ((!was_prep_arc) &&
                ((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
                 (Gray_Task_Lap >= Gray_Task_TargetLap))) {
                Gray_StopMission();
                return 1U;
            }
            g_grayRoute = GRAY_ROUTE_BRIDGE_BOTTOM_LR;
            g_grayBridgeBaseYawValid = 0;
            break;
        }
        if (Gray_Task_Lap < 255U) {
            Gray_Task_Lap++;
        }
        if (Gray_Task_Mode == GRAY_TASK_2_CW_1LAP) {
            Gray_StopMission();
            return 1U;
        }
        g_grayRoute = GRAY_ROUTE_BRIDGE_TOP_LR;
        break;
    case GRAY_ROUTE_ARC_LEFT_DOWN:
        Gray_RecordPoint('D');
        g_grayRoute = GRAY_ROUTE_BRIDGE_BOTTOM_LR;
        break;
    case GRAY_ROUTE_ARC_RIGHT_UP:
        Gray_RecordPoint('B');
        if ((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
            (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP)) {
            g_grayRoute = GRAY_ROUTE_BRIDGE_BOTTOM_RL;
        } else {
            g_grayRoute = GRAY_ROUTE_BRIDGE_TOP_RL;
        }
        g_grayBridgeBaseYawValid = 0;
        break;
    default:
        break;
    }

    return 0U;
}

static uint8_t Gray_OnBridgeFinish(void)
{
    switch (g_grayRoute) {
    case GRAY_ROUTE_BRIDGE_TOP_LR:
        Gray_RecordPoint('B');
        if (Gray_Task_Mode == GRAY_TASK_1_AB_STOP) {
            Gray_StopMission();
            return 1U;
        }
        g_grayRoute = GRAY_ROUTE_ARC_RIGHT_DOWN;
        break;
    case GRAY_ROUTE_BRIDGE_BOTTOM_RL:
        Gray_RecordPoint('D');
        g_grayRoute = GRAY_ROUTE_ARC_LEFT_UP;
        break;
    case GRAY_ROUTE_BRIDGE_BOTTOM_LR:
        Gray_RecordPoint('C');
        g_grayRoute = GRAY_ROUTE_ARC_RIGHT_UP;
        break;
    case GRAY_ROUTE_BRIDGE_TOP_RL:
        Gray_RecordPoint('A');
        if (Gray_Task_Lap < 255U) {
            Gray_Task_Lap++;
        }
        if ((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
            ((Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP) &&
             (Gray_Task_Lap >= Gray_Task_TargetLap))) {
            Gray_StopMission();
            return 1U;
        }
        g_grayRoute = GRAY_ROUTE_ARC_LEFT_DOWN;
        break;
    default:
        break;
    }

    if ((Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP) &&
        (Gray_Task_Lap >= Gray_Task_TargetLap) &&
        (g_grayRoute == GRAY_ROUTE_ARC_LEFT_DOWN)) {
        Gray_StopMission();
        return 1U;
    }

    return 0U;
}

static uint8_t Gray_Task4PoseNear(float x_m, float y_m)
{
    return ((Gray_AbsDeg(g_grayPoseX_m - x_m) <= GRAY_TASK4_POINT_TOL_M) &&
            (Gray_AbsDeg(g_grayPoseY_m - y_m) <= GRAY_TASK4_POINT_TOL_M)) ? 1U : 0U;
}

static void Gray_Task4PoseSnap(float x_m, float y_m)
{
    g_grayPoseX_m = x_m;
    g_grayPoseY_m = y_m;
    Gray_PosePublish();
}

static void Gray_Task4Command(float speed_mps, float turn_speed, uint8_t nav_mode)
{
    float turn_limit = (nav_mode == GRAY_NAV_MODE_LINE) ?
                       GRAY_MAX_ANGULAR_SPEED : GRAY_BRIDGE_MAX_TURN_SPEED;

    if (turn_speed > turn_limit) turn_speed = turn_limit;
    if (turn_speed < -turn_limit) turn_speed = -turn_limit;

    Move_X = speed_mps;
    Move_Z = turn_speed;
    Gray_Nav_Mode = nav_mode;
    Get_Target_Encoder(Move_X, Move_Z);
}

static void Gray_Task4FollowLine(int black_count, float pos_sum, float wz_dps)
{
    float abs_line_pos_mm;
    float y_m;
    float lookahead_m;
    float curvature;
    float speed_mps;
    float turn_speed;

    if (black_count == 0) {
        Gray_Line_Pos_mm = g_grayTask4LastLinePos_mm;
        Gray_BridgeErrDegX10 = 0;
        Gray_Task4Command(GRAY_TASK4_LOST_CREEP_MM_S / 1000.0f,
                          g_grayTask4LastSearchDir * GRAY_LOST_TURN_SPEED,
                          GRAY_NAV_MODE_LOST);
        return;
    }

    Gray_Line_Pos_mm = pos_sum / black_count;
    g_grayTask4LastLinePos_mm = Gray_Line_Pos_mm;
    abs_line_pos_mm = Gray_AbsDeg(Gray_Line_Pos_mm);

    if (Gray_Line_Pos_mm > GRAY_LOST_POS_DEADBAND_MM) {
        g_grayTask4LastSearchDir = -1;
    } else if (Gray_Line_Pos_mm < -GRAY_LOST_POS_DEADBAND_MM) {
        g_grayTask4LastSearchDir = 1;
    }

    speed_mps = GRAY_BASE_SPEED_MM_S / 1000.0f;
    if ((black_count >= 5) || (abs_line_pos_mm >= GRAY_CURVE_POS_MM)) {
        speed_mps = GRAY_CURVE_SPEED_MM_S / 1000.0f;
    } else if (abs_line_pos_mm >= GRAY_MID_POS_MM) {
        speed_mps = GRAY_MID_SPEED_MM_S / 1000.0f;
    }

    y_m = Gray_Line_Pos_mm / 1000.0f;
    lookahead_m = GRAY_SENSOR_FORWARD_MM / 1000.0f;
    curvature = (2.0f * y_m) / (lookahead_m * lookahead_m + y_m * y_m);
    turn_speed = -GRAY_STEER_GAIN * speed_mps * curvature - GRAY_LINE_GYRO_KD * wz_dps;
    if (turn_speed > GRAY_MAX_ANGULAR_SPEED) turn_speed = GRAY_MAX_ANGULAR_SPEED;
    if (turn_speed < -GRAY_MAX_ANGULAR_SPEED) turn_speed = -GRAY_MAX_ANGULAR_SPEED;

    Gray_BridgeErrDegX10 = 0;
    Gray_Task4Command(speed_mps, turn_speed, GRAY_NAV_MODE_LINE);
}

static uint8_t Gray_Task4AlignHeading(float target_heading_deg, float wz_dps)
{
    float heading_err_deg = Gray_NormalizeYawDeg(target_heading_deg - g_grayPoseHeadingDeg);
    float turn_speed = GRAY_BRIDGE_HEADING_KP * heading_err_deg - GRAY_BRIDGE_GYRO_KD * wz_dps;

    Gray_BridgeErrDegX10 = (int16_t)(heading_err_deg * 10.0f);
    if (Gray_AbsDeg(heading_err_deg) <= GRAY_TASK4_ALIGN_ERR_DEG) {
        if (g_grayTask4AlignStableTicks < 255U) {
            g_grayTask4AlignStableTicks++;
        }
        if (g_grayTask4AlignStableTicks >= GRAY_TASK4_ALIGN_STABLE_TICKS) {
            Gray_Task4Command(0, 0, GRAY_NAV_MODE_BRIDGE);
            return 1U;
        }
    } else {
        g_grayTask4AlignStableTicks = 0;
    }

    if (g_grayTask4AlignTicks >= GRAY_TASK4_ALIGN_TICKS_MAX) {
        Gray_StopMission();
        return 0U;
    }
    g_grayTask4AlignTicks++;
    Gray_Task4Command(0, turn_speed, GRAY_NAV_MODE_BRIDGE);
    return 0U;
}

static void Gray_Task4BeginAlign(GrayTask4State_t state)
{
    g_grayTask4State = state;
    g_grayTask4AlignTicks = 0;
    g_grayTask4AlignStableTicks = 0;
    g_grayTask4ReacquireTicks = 0;
    Gray_BridgeDistance_mm = 0;
}

static void Gray_Task4BridgeCommand(float target_heading_deg, float wz_dps)
{
    float heading_err_deg = Gray_NormalizeYawDeg(target_heading_deg - g_grayPoseHeadingDeg);
    float turn_speed = GRAY_BRIDGE_HEADING_KP * heading_err_deg - GRAY_BRIDGE_GYRO_KD * wz_dps;

    g_grayTask4BridgeDistance_mm += GRAY_TASK4_BRIDGE_SPEED_MM_S * GRAY_POSE_DT_S;
    Gray_BridgeDistance_mm = (uint16_t)g_grayTask4BridgeDistance_mm;
    Gray_BridgeErrDegX10 = (int16_t)(heading_err_deg * 10.0f);

    if (g_grayTask4BridgeDistance_mm >= GRAY_TASK4_BRIDGE_MAX_MM) {
        Gray_StopMission();
        return;
    }

    Gray_Task4Command(GRAY_TASK4_BRIDGE_SPEED_MM_S / 1000.0f,
                      turn_speed, GRAY_NAV_MODE_BRIDGE);
}

static uint8_t Gray_Task4WhitePointReached(float x_m, float y_m)
{
    if (Gray_Task4PoseNear(x_m, y_m)) {
        if (g_grayTask4WhiteTicks < 255U) {
            g_grayTask4WhiteTicks++;
        }
    } else {
        g_grayTask4WhiteTicks = 0;
    }

    return (g_grayTask4WhiteTicks >= GRAY_TASK4_WHITE_TICKS) ? 1U : 0U;
}

static void Gray_Task4Mode(void)
{
    uint8_t i;
    int black_count = 0;
    float pos_sum = 0;
    float wz_dps = 0;
    JY62_Data jy62_data;

    if (g_grayMissionDone) {
        Gray_Task4Command(0, 0, GRAY_NAV_MODE_LINE);
        return;
    }

    Gray_Read_All();
    for (i = 0; i < 8; i++) {
        if (Gray_Data[i]) {
            pos_sum += Gray_Pos_mm[i];
            black_count++;
        }
    }
    Gray_BlackCount_Debug = (uint8_t)black_count;

    if (!JY62_IsOnline() || !JY62_GetData(&jy62_data) || !g_grayPoseYawOriginValid) {
        Gray_Task4Command(0, 0, GRAY_NAV_MODE_LOST);
        return;
    }
    wz_dps = jy62_data.wz_dps;
    Gray_YawDegX10 = (int16_t)(jy62_data.yaw_deg * 10.0f);

    switch (g_grayTask4State) {
    case GRAY_TASK4_LINE_AD:
        if ((black_count == 0) && Gray_Task4WhitePointReached(0.0f, -0.80f)) {
            Gray_Task4PoseSnap(0.0f, -0.80f);
            Gray_RecordPoint('D');
            Gray_Task4BeginAlign(GRAY_TASK4_ALIGN_D_TANGENT);
            Gray_Task4Command(0, 0, GRAY_NAV_MODE_BRIDGE);
            return;
        }
        if (black_count != 0) g_grayTask4WhiteTicks = 0;
        Gray_Task4FollowLine(black_count, pos_sum, wz_dps);
        return;

    case GRAY_TASK4_ALIGN_D_TANGENT:
        if (Gray_Task4AlignHeading(GRAY_TASK4_D_TANGENT_DEG, wz_dps)) {
            Gray_Task4BeginAlign(GRAY_TASK4_ALIGN_D_TO_B);
        }
        return;

    case GRAY_TASK4_ALIGN_D_TO_B:
        if (Gray_Task4AlignHeading(GRAY_TASK4_D_TO_B_DEG, wz_dps)) {
            g_grayTask4State = GRAY_TASK4_BRIDGE_D_TO_B;
            g_grayTask4BridgeDistance_mm = 0;
            g_grayTask4ReacquireTicks = 0;
        }
        return;

    case GRAY_TASK4_BRIDGE_D_TO_B:
        if (black_count > 0) {
            if (g_grayTask4ReacquireTicks < 255U) {
                g_grayTask4ReacquireTicks++;
            }
            if (g_grayTask4ReacquireTicks >= 2U) {
                Gray_RecordPoint('B');
                g_grayTask4State = GRAY_TASK4_LINE_BC;
                g_grayTask4WhiteTicks = 0;
                g_grayTask4BridgeDistance_mm = 0;
                Gray_BridgeDistance_mm = 0;
                Gray_Task4FollowLine(black_count, pos_sum, wz_dps);
                return;
            }
        } else {
            g_grayTask4ReacquireTicks = 0;
        }
        Gray_Task4BridgeCommand(GRAY_TASK4_D_TO_B_DEG, wz_dps);
        return;

    case GRAY_TASK4_LINE_BC:
        if ((black_count == 0) && Gray_Task4WhitePointReached(1.00f, -0.80f)) {
            Gray_Task4PoseSnap(1.00f, -0.80f);
            Gray_RecordPoint('C');
            Gray_Task4BeginAlign(GRAY_TASK4_ALIGN_C_TANGENT);
            Gray_Task4Command(0, 0, GRAY_NAV_MODE_BRIDGE);
            return;
        }
        if (black_count != 0) g_grayTask4WhiteTicks = 0;
        Gray_Task4FollowLine(black_count, pos_sum, wz_dps);
        return;

    case GRAY_TASK4_ALIGN_C_TANGENT:
        if (Gray_Task4AlignHeading(GRAY_TASK4_C_TANGENT_DEG, wz_dps)) {
            Gray_Task4BeginAlign(GRAY_TASK4_ALIGN_C_TO_A);
        }
        return;

    case GRAY_TASK4_ALIGN_C_TO_A:
        if (Gray_Task4AlignHeading(GRAY_TASK4_C_TO_A_DEG, wz_dps)) {
            g_grayTask4State = GRAY_TASK4_BRIDGE_C_TO_A;
            g_grayTask4BridgeDistance_mm = 0;
            g_grayTask4ReacquireTicks = 0;
        }
        return;

    case GRAY_TASK4_BRIDGE_C_TO_A:
        if ((black_count > 0) && Gray_Task4PoseNear(0.0f, 0.0f)) {
            if (g_grayTask4ReacquireTicks < 255U) {
                g_grayTask4ReacquireTicks++;
            }
            if (g_grayTask4ReacquireTicks >= 2U) {
                Gray_Task4PoseSnap(0.0f, 0.0f);
                Gray_RecordPoint('A');
                if (Gray_Task_Lap < 255U) {
                    Gray_Task_Lap++;
                }
                if (Gray_Task_Lap >= Gray_Task_TargetLap) {
                    Gray_StopMission();
                    return;
                }
                Gray_Task4BeginAlign(GRAY_TASK4_ALIGN_A_TANGENT);
                Gray_Task4Command(0, 0, GRAY_NAV_MODE_BRIDGE);
                return;
            }
        } else {
            g_grayTask4ReacquireTicks = 0;
        }
        Gray_Task4BridgeCommand(GRAY_TASK4_C_TO_A_DEG, wz_dps);
        return;

    case GRAY_TASK4_ALIGN_A_TANGENT:
        if (Gray_Task4AlignHeading(GRAY_TASK4_START_HEADING_DEG, wz_dps)) {
            g_grayTask4State = GRAY_TASK4_LINE_AD;
            g_grayTask4WhiteTicks = 0;
            Gray_Task4Command(0, 0, GRAY_NAV_MODE_LINE);
        }
        return;

    default:
        Gray_StopMission();
        return;
    }
}

void Gray_Mode(void)
{
    if (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP) {
        Gray_Task4Mode();
        return;
    }

    static float last_line_pos_mm = 0;
    static int8_t last_search_dir = 1;
    static uint8_t cross_detect_count = 0;
    static uint8_t cross_run_ticks = 0;
    static uint8_t cross_cooldown_ticks = 0;
    static uint8_t line_stable_ticks = 0;
    static uint8_t bridge_active = 0;
    static uint8_t bridge_reacquire_count = 0;
    static uint8_t bridge_align_ticks = 0;
    static uint8_t arc_yaw_valid = 0;
    static float arc_entry_yaw_deg = 0;
    static float bridge_yaw_ref_deg = 0;
    static float bridge_distance_mm = 0;
    float pos_sum = 0;
    float abs_line_pos_mm;
    int black_count = 0;
    float y_m;
    float lookahead_m;
    float curvature;
    float yaw_deg = 0;
    float wz_dps = 0;
    float yaw_err_deg = 0;
    float arc_turn_deg = 0;
    float abs_arc_turn_deg = 0;
    uint8_t i;
    JY62_Data jy62_data;

    if (g_grayNeedReset) {
        last_line_pos_mm = 0;
        last_search_dir = 1;
        cross_detect_count = 0;
        cross_run_ticks = 0;
        cross_cooldown_ticks = 0;
        line_stable_ticks = 0;
        bridge_active = 0;
        bridge_reacquire_count = 0;
        bridge_align_ticks = 0;
        arc_yaw_valid = 0;
        arc_entry_yaw_deg = 0;
        bridge_yaw_ref_deg = 0;
        bridge_distance_mm = 0;
        Gray_Line_Pos_mm = 0;
        Gray_YawDegX10 = 0;
        Gray_BridgeErrDegX10 = 0;
        Gray_BridgeDistance_mm = 0;
        Gray_BlackCount_Debug = 0;
        g_grayNeedReset = 0;
    }

    if (g_grayMissionDone) {
        Move_X = 0;
        Move_Z = 0;
        Get_Target_Encoder(0, 0);
        return;
    }

    Gray_Read_All();
    for (i = 0; i < 8; i++) {
        if (Gray_Data[i]) {
            pos_sum += Gray_Pos_mm[i];
            black_count++;
        }
    }
    Gray_BlackCount_Debug = (uint8_t) black_count;

    if (JY62_GetData(&jy62_data)) {
        yaw_deg = jy62_data.yaw_deg;
        wz_dps = jy62_data.wz_dps;
        Gray_YawDegX10 = (int16_t) (yaw_deg * 10.0f);
    }

    if (cross_cooldown_ticks > 0) {
        cross_cooldown_ticks--;
    }

    if (cross_run_ticks > 0) {
        bridge_active = 0;
        bridge_align_ticks = 0;
        bridge_distance_mm = 0;
        Gray_BridgeDistance_mm = 0;
        Gray_BridgeErrDegX10 = 0;
        Gray_Nav_Mode = GRAY_NAV_MODE_LINE;
        cross_run_ticks--;
        Move_X = GRAY_CROSS_SPEED_MM_S / 1000.0f;
        Move_Z = 0;
        Get_Target_Encoder(Move_X, Move_Z);
        return;
    }

    if (black_count >= GRAY_CROSS_BLACK_COUNT) {
        if (cross_detect_count < GRAY_CROSS_DETECT_COUNT) {
            cross_detect_count++;
        }
        if ((cross_detect_count >= GRAY_CROSS_DETECT_COUNT) && (cross_cooldown_ticks == 0)) {
            cross_detect_count = 0;
            cross_run_ticks = GRAY_CROSS_RUN_TICKS;
            cross_cooldown_ticks = GRAY_CROSS_COOLDOWN_TICKS;
            bridge_active = 0;
            bridge_align_ticks = 0;
            bridge_distance_mm = 0;
            Gray_BridgeDistance_mm = 0;
            Gray_BridgeErrDegX10 = 0;
            Gray_Nav_Mode = GRAY_NAV_MODE_LINE;
            Move_X = GRAY_CROSS_SPEED_MM_S / 1000.0f;
            Move_Z = 0;
            Get_Target_Encoder(Move_X, Move_Z);
            return;
        }
    } else {
        cross_detect_count = 0;
    }

    Move_X = GRAY_BASE_SPEED_MM_S / 1000.0f;
    if (black_count == 0) {
        Gray_Line_Pos_mm = last_line_pos_mm;

        if (bridge_active) {
            if ((bridge_distance_mm < GRAY_BRIDGE_MAX_DISTANCE_MM) && JY62_IsOnline()) {
                yaw_err_deg = Gray_NormalizeYawDeg(bridge_yaw_ref_deg - yaw_deg);

                if (bridge_align_ticks > 0U) {
                    bridge_align_ticks--;
                    Move_X = GRAY_BRIDGE_ALIGN_SPEED_MM_S / 1000.0f;
                    Move_Z = GRAY_BRIDGE_HEADING_KP * yaw_err_deg - GRAY_BRIDGE_GYRO_KD * wz_dps;
                    if ((yaw_err_deg < GRAY_BRIDGE_ALIGN_ERR_DEG) &&
                        (yaw_err_deg > -GRAY_BRIDGE_ALIGN_ERR_DEG)) {
                        bridge_align_ticks = 0;
                    }

                    if (Move_Z > GRAY_BRIDGE_MAX_TURN_SPEED) Move_Z = GRAY_BRIDGE_MAX_TURN_SPEED;
                    if (Move_Z < -GRAY_BRIDGE_MAX_TURN_SPEED) Move_Z = -GRAY_BRIDGE_MAX_TURN_SPEED;

                    Gray_BridgeErrDegX10 = (int16_t) (yaw_err_deg * 10.0f);
                    Gray_Nav_Mode = GRAY_NAV_MODE_BRIDGE;
                    Get_Target_Encoder(Move_X, Move_Z);
                    return;
                }

                bridge_distance_mm += GRAY_BRIDGE_ENTRY_SPEED_MM_S * 0.005f;
                Gray_BridgeDistance_mm = (uint16_t) bridge_distance_mm;
                Move_X = (bridge_distance_mm >= GRAY_BRIDGE_EXPECT_DISTANCE_MM) ?
                         (GRAY_BRIDGE_RECOVER_SPEED_MM_S / 1000.0f) :
                         (GRAY_BRIDGE_ENTRY_SPEED_MM_S / 1000.0f);
                Move_Z = GRAY_BRIDGE_HEADING_KP * yaw_err_deg - GRAY_BRIDGE_GYRO_KD * wz_dps;
                if (bridge_distance_mm >= GRAY_BRIDGE_SEARCH_DISTANCE_MM) {
                    Move_Z += (g_grayRoute == GRAY_ROUTE_BRIDGE_BOTTOM_LR) ?
                              GRAY_BRIDGE_SEARCH_TURN_SPEED :
                              -GRAY_BRIDGE_SEARCH_TURN_SPEED;
                }

                if (Move_Z > GRAY_BRIDGE_MAX_TURN_SPEED) Move_Z = GRAY_BRIDGE_MAX_TURN_SPEED;
                if (Move_Z < -GRAY_BRIDGE_MAX_TURN_SPEED) Move_Z = -GRAY_BRIDGE_MAX_TURN_SPEED;

                Gray_BridgeErrDegX10 = (int16_t) (yaw_err_deg * 10.0f);
                Gray_Nav_Mode = GRAY_NAV_MODE_BRIDGE;
                Get_Target_Encoder(Move_X, Move_Z);
                return;
            }

            bridge_active = 0;
            bridge_reacquire_count = 0;
            bridge_align_ticks = 0;
        }

        if ((!bridge_active) && (line_stable_ticks >= GRAY_BRIDGE_MIN_STABLE_TICKS) && JY62_IsOnline()) {
            if (((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
                 (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP)) &&
                Gray_RouteIsArc(g_grayRoute) && arc_yaw_valid) {
                arc_turn_deg = Gray_NormalizeYawDeg(yaw_deg - arc_entry_yaw_deg);
                abs_arc_turn_deg = Gray_AbsDeg(arc_turn_deg);
                if ((!g_grayPrepArcToA) &&
                    (abs_arc_turn_deg < GRAY_ARC_MIN_TURN_DEG)) {
                    line_stable_ticks = 0;
                    Gray_Nav_Mode = GRAY_NAV_MODE_LOST;
                    Move_X = GRAY_LOST_SEARCH_SPEED_MM_S / 1000.0f;
                    Move_Z = last_search_dir * GRAY_LOST_TURN_SPEED;
                    Get_Target_Encoder(Move_X, Move_Z);
                    return;
                }

                g_grayLineYawRefDeg = yaw_deg;
                g_grayLineYawRefValid = 1;
            }

            if (Gray_OnBridgeStart()) {
                return;
            }
            bridge_active = 1;
            bridge_reacquire_count = 0;
            bridge_align_ticks = (((Gray_Task_Mode == GRAY_TASK_3_CCW_1LAP) ||
                                   (Gray_Task_Mode == GRAY_TASK_4_CCW_4LAP)) ?
                                  GRAY_BRIDGE_ALIGN_TICKS : 0U);
            bridge_yaw_ref_deg = Gray_BridgeYawRefGet(yaw_deg);
            bridge_distance_mm = 0;
            Gray_BridgeDistance_mm = 0;
            Gray_BridgeErrDegX10 = 0;
            arc_yaw_valid = 0;
            Gray_Nav_Mode = GRAY_NAV_MODE_BRIDGE;
            Move_X = GRAY_BRIDGE_ENTRY_SPEED_MM_S / 1000.0f;
            Move_Z = 0;
            Get_Target_Encoder(Move_X, Move_Z);
            return;
        }

        line_stable_ticks = 0;
        Gray_BridgeErrDegX10 = 0;
        Gray_Nav_Mode = GRAY_NAV_MODE_LOST;
        Move_X = GRAY_LOST_SEARCH_SPEED_MM_S / 1000.0f;
        Move_Z = last_search_dir * GRAY_LOST_TURN_SPEED;
        Get_Target_Encoder(Move_X, Move_Z);
        return;
    }

    Gray_Line_Pos_mm = pos_sum / black_count;
    last_line_pos_mm = Gray_Line_Pos_mm;
    Gray_BridgeErrDegX10 = 0;
    abs_line_pos_mm = (Gray_Line_Pos_mm >= 0) ? Gray_Line_Pos_mm : -Gray_Line_Pos_mm;

    if (line_stable_ticks < 255U) {
        line_stable_ticks++;
    }

    if (JY62_IsOnline() && Gray_RouteIsArc(g_grayRoute) && (!arc_yaw_valid)) {
        arc_entry_yaw_deg = yaw_deg;
        arc_yaw_valid = 1;
    }

    if (JY62_IsOnline() && Gray_RouteIsArc(g_grayRoute) &&
        (Gray_AbsDeg(wz_dps) >= GRAY_ARC_TURN_MIN_DPS)) {
        g_grayArcTurnDir = (wz_dps >= 0.0f) ? 1 : -1;
    }

    if (JY62_IsOnline() && (abs_line_pos_mm <= GRAY_LINE_YAW_CENTER_MM) &&
        (black_count > 0) && (black_count <= 4)) {
        Gray_LineYawRefUpdate(yaw_deg);
    }

    if (bridge_active) {
        uint8_t reacquire_black_count = (bridge_distance_mm >= GRAY_BRIDGE_EXPECT_DISTANCE_MM) ? 1U : 2U;
        if (black_count >= reacquire_black_count) {
            if (bridge_reacquire_count < GRAY_BRIDGE_REACQUIRE_TICKS) {
                bridge_reacquire_count++;
            }
            if (bridge_reacquire_count >= GRAY_BRIDGE_REACQUIRE_TICKS) {
                bridge_active = 0;
                bridge_reacquire_count = 0;
                bridge_align_ticks = 0;
                bridge_distance_mm = 0;
                Gray_BridgeDistance_mm = 0;
                if (Gray_OnBridgeFinish()) {
                    return;
                }
            }
        } else {
            bridge_reacquire_count = 0;
        }
    } else {
        bridge_reacquire_count = 0;
    }

    Gray_Nav_Mode = GRAY_NAV_MODE_LINE;

    if (Gray_Line_Pos_mm > GRAY_LOST_POS_DEADBAND_MM) {
        last_search_dir = -1;
    } else if (Gray_Line_Pos_mm < -GRAY_LOST_POS_DEADBAND_MM) {
        last_search_dir = 1;
    }

    if ((black_count >= 5) || (abs_line_pos_mm >= GRAY_CURVE_POS_MM)) {
        Move_X = GRAY_CURVE_SPEED_MM_S / 1000.0f;
    } else if (abs_line_pos_mm >= GRAY_MID_POS_MM) {
        Move_X = GRAY_MID_SPEED_MM_S / 1000.0f;
    }

    y_m = Gray_Line_Pos_mm / 1000.0f;
    lookahead_m = GRAY_SENSOR_FORWARD_MM / 1000.0f;
    curvature = (2.0f * y_m) / (lookahead_m * lookahead_m + y_m * y_m);
    Move_Z = -GRAY_STEER_GAIN * Move_X * curvature;
    if (JY62_IsOnline()) {
        Move_Z -= GRAY_LINE_GYRO_KD * wz_dps;
    }

    if (Move_Z > GRAY_MAX_ANGULAR_SPEED) Move_Z = GRAY_MAX_ANGULAR_SPEED;
    if (Move_Z < -GRAY_MAX_ANGULAR_SPEED) Move_Z = -GRAY_MAX_ANGULAR_SPEED;

    Get_Target_Encoder(Move_X, Move_Z);
}
void TIMER_0_INST_IRQHandler(void)
{
    static uint8_t jy62_tick_div = 0;

    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			show_cnt++;
			jy62_tick_div++;
			if (jy62_tick_div >= 2U) {
				jy62_tick_div = 0;
				JY62_Tick10ms();
				K210_Link_Tick10ms();
			}
			Key();
			Gray_TaskTick();
			Get_Velocity_From_Encoder(-Get_Encoder_countA,-Get_Encoder_countB);
			Get_Encoder_countA=Get_Encoder_countB=0;
			Gray_PoseUpdate();
			if(Run_Mode==0)
			{
				Get_RC();         //Handle the APP remote commands //处理APP遥控命令
			}else if(Run_Mode==1){
				Gray_Mode();//8路灰度巡线
			}
//			//计算左右电机对应的PWM
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
			if(!Flag_Stop)
			{
				Set_PWM(MotorA.Motor_Pwm,-MotorB.Motor_Pwm);
			}else Set_PWM(0,0);
		}
    }
}

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	
	//Retrieves the original data of the encoder
	//获取编码器的原始数据
	float Encoder_A_pr,Encoder_B_pr; 
	OriginalEncoder.A=-Encoder1;	
	OriginalEncoder.B=-Encoder2;	
	Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;
	//编码器原始数据转换为车轮速度，单位m/s
	MotorA.Current_Encoder= -Encoder_A_pr*Frequency*Perimeter/780.0f;  
	MotorB.Current_Encoder= -Encoder_B_pr*Frequency*Perimeter/780.0f;   //1560=2*13*30=2（两路脉冲）*1（上升沿计数）*霍尔编码器13线*电机的减速比
	
}
//运动学逆解，由x和y的速度得到编码器的速度,Vx是m/s,Vz单位是度/s(角度制)
void Get_Target_Encoder(float Vx,float Vz)
{
	float amplitude=3.5f; //Wheel target speed limit //车轮目标速度限幅
	if(Vx<0) Vz=-Vz;
	else     Vz=Vz;
	//Inverse kinematics //运动学逆解
	 MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //计算出左轮的目标速度
	 MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //计算出右轮的目标速度
	//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
//	 MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	 MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
}


/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//电压高于7V且使能开关打开
//	{
//		temp = 1;
//	}
	return temp;			
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
//	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
//	{
//	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
//	 { 
//			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
//			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
//			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
//			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
//			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
//			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
//			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
//			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
//			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
//	 }
//	 if(Flag_Move==0)		
//	 {	
//		 //If no direction control instruction is available, check the steering control status
//		 //如果无方向控制指令，检查转向控制状态
//		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
//		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
//		 else 		               Move_Z=0;                       //stop           //停止
//	 }
//	}	
//	else //Non-omnidirectional moving trolley //非全向移动小车
//	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
//	}
	
//	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/200;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Get_Target_Encoder(Move_X,Move_Z);
}

/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp;
	tmp=key_scan(200);//click_N_Double(50);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
        if (!Flag_Stop) {
            Gray_ResetTaskState();
            Gray_StartNotify();
        } else {
            g_grayMissionDone = 0;
            Move_X = 0;
            Move_Z = 0;
            Get_Target_Encoder(0, 0);
        }
	}		//单击控制小车的启停
	else if(tmp==2)
	{
        if (Flag_Stop) {
            Gray_Task_Mode++;
            Gray_Task_Mode %= 4U;
            Gray_ResetTaskState();
            Gray_StartNotify();
        } else {
		    Run_Mode++;
		    Run_Mode%=2;
        }
	}
    else if ((tmp == 3) && Flag_Stop) {
        Gray_ResetTaskState();
        Gray_StartNotify();
    }
}
