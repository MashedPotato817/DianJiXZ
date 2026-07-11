/***********************************************
Brand: WHEELTEC
***********************************************/
#include "show.h"
#include "control.h"

void oled_show(void)
{
    u8 i;
    int gray_pos_show;
    int yaw_show;
    int err_show;

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));

    if (Car_Mode == 0) OLED_ShowString(0, 0, "Mec ");
    else if (Car_Mode == 1) OLED_ShowString(0, 0, "Omni");
    else if (Car_Mode == 2) OLED_ShowString(0, 0, "AKM ");
    else if (Car_Mode == 3) OLED_ShowString(0, 0, "Diff");
    else if (Car_Mode == 4) OLED_ShowString(0, 0, "4WD ");
    else if (Car_Mode == 5) OLED_ShowString(0, 0, "Tank");

    if (Run_Mode == 0) OLED_ShowString(90, 0, "APP");
    else if (Run_Mode == 1) OLED_ShowString(90, 0, "GRY");

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
