/***********************************************
Brand: WHEELTEC
***********************************************/
#include "show.h"
#include "control.h"
#include "jy62.h"

static void OLED_ShowSignedNumber(uint8_t x, uint8_t y, int value, uint8_t len)
{
    OLED_ShowString(x, y, (value < 0) ? "-" : "+");
    OLED_ShowNumber(x + 8U, y, myabs(value), len, 12);
}

static void oled_show_jy62_diag(void)
{
    JY62_Data data;

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    JY62_GetData(&data);

    OLED_ShowString(0, 0, "JY");
    OLED_ShowString(18, 0, data.online ? "ON " : "OFF");
    OLED_ShowString(48, 0, "VF");
    OLED_ShowNumber(66, 0, data.valid_frames % 10000U, 4, 12);
    OLED_ShowString(0, 10, "ER");
    OLED_ShowNumber(18, 10, data.checksum_errors % 10000U, 4, 12);
    OLED_ShowString(54, 10, "B");
    OLED_ShowSignedNumber(64, 10, Gray_GyroZBiasDpsX10 / 10, 2);
    OLED_ShowString(92, 10, "S");
    OLED_ShowNumber(102, 10, (uint32_t)myabs((int)Gray_PoseDistance_mm) % 10000U, 4, 12);

    OLED_ShowString(0, 20, "R");
    OLED_ShowSignedNumber(10, 20, (int)data.roll_deg, 3);
    OLED_ShowString(46, 20, "P");
    OLED_ShowSignedNumber(56, 20, (int)data.pitch_deg, 3);
    OLED_ShowString(92, 20, "Y");
    OLED_ShowSignedNumber(102, 20, (int)data.yaw_deg, 3);

    OLED_ShowString(0, 30, "GX");
    OLED_ShowSignedNumber(14, 30, (int)data.wx_dps, 3);
    OLED_ShowString(50, 30, "GY");
    OLED_ShowSignedNumber(64, 30, (int)data.wy_dps, 3);

    OLED_ShowString(0, 40, "GZ");
    OLED_ShowSignedNumber(14, 40, (int)data.wz_dps, 3);
    OLED_ShowString(54, 40, "H");
    OLED_ShowSignedNumber(64, 40, Gray_PoseHeadingDegX10 / 10, 3);

    OLED_ShowString(0, 50, "X");
    OLED_ShowSignedNumber(10, 50, (int)Gray_PoseX_mm, 4);
    OLED_ShowString(58, 50, "Y");
    OLED_ShowSignedNumber(68, 50, (int)Gray_PoseY_mm, 4);

    OLED_Refresh_Gram();
}

void oled_show(void)
{
    u8 i;
    int gray_pos_show;
    int yaw_show;
    int err_show;
    char point_str[2];

    if (Flag_Stop) {
        oled_show_jy62_diag();
        return;
    }

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    point_str[0] = (char) Gray_LastPointChar;
    point_str[1] = '\0';

    if (Car_Mode == 0) OLED_ShowString(0, 0, "Mec ");
    else if (Car_Mode == 1) OLED_ShowString(0, 0, "Omni");
    else if (Car_Mode == 2) OLED_ShowString(0, 0, "AKM ");
    else if (Car_Mode == 3) OLED_ShowString(0, 0, "Diff");
    else if (Car_Mode == 4) OLED_ShowString(0, 0, "4WD ");
    else if (Car_Mode == 5) OLED_ShowString(0, 0, "Tank");

    OLED_ShowString(42, 0, "T");
    OLED_ShowNumber(52, 0, Gray_Task_Mode + 1U, 1, 12);
    OLED_ShowString(66, 0, "L");
    OLED_ShowNumber(76, 0, Gray_Task_Lap, 1, 12);
    OLED_ShowString(84, 0, "/");
    OLED_ShowNumber(92, 0, Gray_Task_TargetLap, 1, 12);
    if (Run_Mode == 0) OLED_ShowString(108, 0, "AP");
    else if (Run_Mode == 1) OLED_ShowString(108, 0, "GY");

    OLED_ShowString(0, 10, "G");
    for (i = 0; i < 8; i++) {
        OLED_ShowString(12 + i * 10, 10, Gray_Raw[i] ? "1" : "0");
    }

    gray_pos_show = (int) Gray_Line_Pos_mm;
    OLED_ShowString(0, 20, "P");
    OLED_ShowString(16, 20, (gray_pos_show < 0) ? "-" : "+");
    OLED_ShowNumber(26, 20, myabs(gray_pos_show), 3, 12);
    OLED_ShowString(60, 20, "Z");
    OLED_ShowString(76, 20, (Move_Z < 0) ? "-" : "+");
    OLED_ShowNumber(86, 20, myabs((int) (Move_Z * 1000)), 4, 12);

    OLED_ShowString(0, 30, "M");
    if (Gray_Nav_Mode == GRAY_NAV_MODE_LINE) OLED_ShowString(12, 30, "LIN");
    else if (Gray_Nav_Mode == GRAY_NAV_MODE_BRIDGE) OLED_ShowString(12, 30, "BRG");
    else OLED_ShowString(12, 30, "LST");
    OLED_ShowString(48, 30, "B");
    OLED_ShowNumber(60, 30, Gray_BlackCount_Debug, 1, 12);
    OLED_ShowString(78, 30, "D");
    OLED_ShowNumber(90, 30, Gray_BridgeDistance_mm, 3, 12);
    OLED_ShowString(108, 30, "P");
    OLED_ShowString(118, 30, point_str);

    yaw_show = Gray_YawDegX10;
    err_show = Gray_BridgeErrDegX10;
    OLED_ShowString(0, 40, "Y");
    OLED_ShowString(12, 40, (yaw_show < 0) ? "-" : "+");
    OLED_ShowNumber(22, 40, myabs(yaw_show / 10), 3, 12);
    OLED_ShowString(60, 40, "E");
    OLED_ShowString(72, 40, (err_show < 0) ? "-" : "+");
    OLED_ShowNumber(82, 40, myabs(err_show / 10), 3, 12);

    OLED_ShowString(0, 50, "V");
    OLED_ShowNumber(12, 50, (int) Voltage, 2, 12);
    OLED_ShowString(28, 50, ".");
    OLED_ShowNumber(38, 50, (u16) (Voltage * 10) % 10, 1, 12);
    if (Flag_Stop) OLED_ShowString(90, 50, "OFF");
    else OLED_ShowString(90, 50, "ON ");

    OLED_Refresh_Gram();
}

void APP_Show(void)
{
}

void DataScope(void)
{
}
