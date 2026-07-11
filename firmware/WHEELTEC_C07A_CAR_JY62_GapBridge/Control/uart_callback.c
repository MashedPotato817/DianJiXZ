#include "ti_msp_dl_config.h"
#include "board.h"
#include "uart_callback.h"
#include "jy62.h"

int Flag_Left, Flag_Right, Flag_Direction = 0, Turn_Flag;

void BT_DAMConfig(void)
{
}

void BTBufferHandler(void)
{
    JY62_Process();
}

void UART_1_INST_IRQHandler(void)
{
    uint8_t data;

    while (DL_UART_Main_receiveDataCheck(UART_1_INST, &data)) {
        JY62_InputByteFromISR(data);
    }
}
