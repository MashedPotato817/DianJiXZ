#include "ti_msp_dl_config.h"
#include "board.h"
#include "uart_callback.h"
#include "jy62.h"
#include <stdio.h>

int Flag_Left, Flag_Right, Flag_Direction = 0, Turn_Flag;

void BT_DAMConfig(void)
{
}

void BTBufferHandler(void)
{
    JY62_Process();
}

void BlackboxTelemetry_Process(void)
{
    static u16 last_send_cnt = 0;
    JY62_Data data;
    int voltage_x10;

    if ((u16)(show_cnt - last_send_cnt) < 20U) {
        return;
    }
    last_send_cnt = show_cnt;

    JY62_GetData(&data);
    voltage_x10 = (int)(Voltage * 10.0f);

    printf("T=%lu,TASK=%u,LAP=%u,MODE=%u,BC=%u,"
           "YAW=%d,H=%d,X=%ld,Y=%ld,S=%ld,D=%u,"
           "V=%d,VF=%lu,ER=%lu\r\n",
           (unsigned long)show_cnt * 5UL,
           Gray_Task_Mode + 1U,
           Gray_Task_Lap,
           Gray_Nav_Mode,
           Gray_BlackCount_Debug,
           (int)(data.yaw_deg * 10.0f),
           Gray_PoseHeadingDegX10,
           (long)Gray_PoseX_mm,
           (long)Gray_PoseY_mm,
           (long)Gray_PoseDistance_mm,
           Gray_BridgeDistance_mm,
           voltage_x10,
           (unsigned long)data.valid_frames,
           (unsigned long)data.checksum_errors);
}

void UART_1_INST_IRQHandler(void)
{
    uint8_t data;

    while (DL_UART_Main_receiveDataCheck(UART_1_INST, &data)) {
        JY62_InputByteFromISR(data);
    }
}
