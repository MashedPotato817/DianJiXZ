#include "calib_store.h"
#include "servo.h"
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
    /* 宽范围自动标定允许机构重装后的任意有效舵机角度。 */
    if (!((value >= SERVO_ANGLE_MIN_DEG) &&
          (value <= SERVO_ANGLE_MAX_DEG))) {
        return 0U;
    }
    *balance_deg = value;
    return 1U;
}

uint8_t CalibStore_Save(float balance_deg)
{
    uint32_t primask;
    uint32_t words[2];
    DL_FLASHCTL_COMMAND_STATUS command_status;
    bool erase_ok;
    bool program_ok;

    if (!((balance_deg >= SERVO_ANGLE_MIN_DEG) &&
          (balance_deg <= SERVO_ANGLE_MAX_DEG))) {
        return 0U;
    }
    if (((CAL_STORE_ADDR % CAL_STORE_SECTOR_SIZE) != 0U) ||
        ((CAL_STORE_ADDR + CAL_STORE_SECTOR_SIZE) > CAL_STORE_FLASH_SIZE)) {
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

    DL_FlashCTL_unprotectSector(
        FLASHCTL, CAL_STORE_ADDR, DL_FLASHCTL_REGION_SELECT_MAIN);
    command_status = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, CAL_STORE_ADDR, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    erase_ok = (command_status == DL_FLASHCTL_COMMAND_STATUS_PASSED);

    program_ok = erase_ok;
    if (erase_ok) {
        /* 每条命令完成后硬件会重新保护 Flash，编程前必须再次解锁。 */
        DL_FlashCTL_unprotectSector(
            FLASHCTL, CAL_STORE_ADDR, DL_FLASHCTL_REGION_SELECT_MAIN);
        /* 目标地址要求 64 位对齐，magic、float 与硬件生成的 ECC 一次写完。 */
        command_status = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
            FLASHCTL, CAL_STORE_ADDR, &words[0]);
        program_ok =
            (command_status == DL_FLASHCTL_COMMAND_STATUS_PASSED);
    }
    DL_FlashCTL_protectMainMemory(FLASHCTL);

    if (primask == 0U) {
        __enable_irq();
    }
    return program_ok ? 1U : 0U;
}
