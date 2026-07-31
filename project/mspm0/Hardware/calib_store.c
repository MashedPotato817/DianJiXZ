#include "calib_store.h"
#include "ti_msp_dl_config.h"

#include <string.h>

uint8_t CalibStore_Load(float *balance_deg)
{
    const uint32_t *words = (const uint32_t *)CAL_STORE_ADDR;
    float value;

    if (balance_deg == 0) {
        return 0U;
    }
    if (words[0] != CAL_STORE_MAGIC) {
        return 0U;
    }
    (void)memcpy(&value, &words[1], sizeof(value));
    if ((value < 5.0f) || (value > 175.0f)) {
        return 0U;
    }
    *balance_deg = value;
    return 1U;
}

uint8_t CalibStore_Save(float balance_deg)
{
    uint32_t primask;
    uint32_t words[2];
    bool erase_ok;
    bool program_ok;

    if ((balance_deg < 5.0f) || (balance_deg > 175.0f)) {
        return 0U;
    }

    words[0] = CAL_STORE_MAGIC;
    (void)memcpy(&words[1], &balance_deg, sizeof(balance_deg));

    /*
     * 擦除和编程都要求 Flash 命令期间 CPU 不取指自正在写的区域，
     * DriverLib 的 FromRAM 变体内部已处理。写 Flash 期间关闭全局中断，
     * 避免中断向量/ISR 取指 stall；标定完成是低频事件，短暂阻塞可接受。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_unprotectMainMemory(FLASHCTL);
    (void)DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, CAL_STORE_ADDR, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    erase_ok = DL_FlashCTL_waitForCmdDone(FLASHCTL);

    program_ok = erase_ok;
    if (erase_ok) {
        (void)DL_FlashCTL_programMemoryFromRAM32(
            FLASHCTL, CAL_STORE_ADDR, &words[0]);
        program_ok = DL_FlashCTL_waitForCmdDone(FLASHCTL);
    }
    if (program_ok) {
        (void)DL_FlashCTL_programMemoryFromRAM32(
            FLASHCTL, CAL_STORE_ADDR + 4U, &words[1]);
        program_ok = DL_FlashCTL_waitForCmdDone(FLASHCTL);
    }
    DL_FlashCTL_protectMainMemory(FLASHCTL);

    if (primask == 0U) {
        __enable_irq();
    }
    return program_ok ? 1U : 0U;
}
