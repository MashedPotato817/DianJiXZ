#include "ti_msp_dl_config.h"
#include "board.h"
#include "uart_callback.h"
#include "jy62.h"
#include "k210_link.h"

int Flag_Left, Flag_Right, Flag_Direction = 0, Turn_Flag;

void BT_DAMConfig(void)
{
}

void BTBufferHandler(void)
{
}

void UART_1_INST_IRQHandler(void)
{
    uint8_t data;

    while (DL_UART_Main_receiveDataCheck(UART_1_INST, &data)) {
        K210_Link_InputByteFromISR(data);
    }
}

void UART_0_INST_IRQHandler(void)
{
    uint8_t data;

    while (DL_UART_Main_receiveDataCheck(UART_0_INST, &data)) {
    }
}
