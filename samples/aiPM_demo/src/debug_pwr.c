/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdint.h>
#include <cmsis_compiler.h>
#include <soc.h>

#include "debug_pwr.h"

LOG_MODULE_REGISTER(debug_pwr);

void debug_pwr_print(void) {
    uint32_t reg_data;

    /* power domains list */
    printf("Power Domains:\r\n");
    reg_data = *((volatile uint32_t *)0x1A60A004);// VBATSEC PWR_CTRL
    printf("PD0=AON_STOP\r\n");
    printf("PD1=%u\r\n",                  (reg_data & 1U));
    printf("PD2=AON_STBY\r\n");
    printf("PD3=%u\r\n",                  (*((volatile uint32_t *)0x1A601008) & 15U) > 0 ? 1 : 0); // PPU-HE
#if !(defined(CONFIG_ENSEMBLE_GEN2) || defined(CONFIG_SOC_SERIES_E1C))
    printf("PD4=%u\r\n",                  (*((volatile uint32_t *)0x1A605058) & 0xFFFU) > 1 ? 1 : 0);
#endif
    printf("SRAM0=%u\r\n",                (reg_data >> 8) & 1U ? 0 : 1);
    printf("SRAM1=%u\r\n",                (reg_data >> 12) & 1U ? 0 : 1);
    reg_data = (reg_data >> 16) & 3U;
    if ((reg_data == 0) || (reg_data == 3)) reg_data = 1;
    else reg_data = 0;
    printf("MRAM=%u\r\n",                 reg_data);
    /* insert a delay to let PD6 idle before getting the power state */
    for(volatile uint32_t loop_cnt = 0; loop_cnt < 10000; loop_cnt++) __NOP();
    reg_data = *((volatile uint32_t *)0x1A010404);
    printf("PD6=%u\r\n",                  (reg_data >> 3) & 7U);
    printf("PD7=%u\r\n",                  (*((volatile uint32_t *)0x1A600008) & 15U) > 0 ? 1 : 0); // PPU-HP
    printf("PD8=%u\r\n",                  (reg_data >> 2) & 1U);
    printf("PD9=n/a\r\n\n");

    /* get m55 vtor addresses */
    reg_data = *((volatile uint32_t *)0x1A605014);
    printf("M55_HP VTOR=0x%08X\r\n", reg_data);
    reg_data = *((volatile uint32_t *)0x1A60A024);
    printf("M55_HE VTOR=0x%08X\r\n\n", reg_data);

    /* retention settings & stop mode wakeup source enables */
    printf("RETENTON Settings:\r\n");
    reg_data = *((volatile uint32_t *)0x1A60A038);   // VBAT_ANA_REG1
    printf("RET LDO VOUT=%d\r\n",               (reg_data >> 4) & 15U);
    printf("RET LDO VBAT_EN=%u MAIN_EN=%u\r\n", (reg_data >> 8) & 1U, (reg_data >> 10) & 1U);
#if defined(CONFIG_SOC_SERIES_E1C)
    reg_data = *((volatile uint32_t *)0x1A60900C);   // VBATALL RET_CTRL
    printf("BK RAM=0x%X HE TCM=0x%X BLE MEM=0x%X\r\n", (reg_data & 1U), (reg_data >> 1) & 63U, (reg_data >> 7) & 15U);
    reg_data = *((volatile uint32_t *)0x1A60A018);   // VBATSEC RET_CTRL
    printf("FW RAM=0x%X SE RAM=0x%X\r\n",   (reg_data & 1U), (reg_data >> 4) & 15U);
#else
    reg_data = *((volatile uint32_t *)0x1A60900C);   // VBATALL RET_CTRL
#if defined(CONFIG_ENSEMBLE_GEN2)
    printf("SRAM0=0x%02X SRAM1=0x%02X\r\n",   (reg_data >> 8) & 255U, (reg_data >> 16) & 3U);
#endif
    printf("BK RAM=0x%X HE TCM=0x%X\r\n",   (reg_data & 3U), (reg_data >> 4) & 15U);
    reg_data = *((volatile uint32_t *)0x1A60A018);   // VBATSEC RET_CTRL
    printf("FW RAM=0x%X SE RAM=0x%X\r\n",   (reg_data & 3U), (reg_data >> 4) & 3U);
#endif
    printf("VBAT_WKUP=0x%08X\r\n\n",       *((volatile uint32_t *)0x1A60A008));// VBATSEC WKUP_CTRL

    printf("PHY Power:\r\n");
    reg_data = *((volatile uint32_t *)0x1A609008);   // VBATALL PWR_CTRL
    printf("PWR_CTRL=0x%08X\r\n\n", reg_data);

    /* bandgap trim settings */
    printf("TRIM Settings:\r\n");
    reg_data = *((volatile uint32_t *)0x1A60A038);   // VBAT_ANA_REG1
    printf("RC OSC 32.768k=%u\r\n",       (reg_data & 15U));
    reg_data = *((volatile uint32_t *)0x1A60A03C);   // VBAT_ANA_REG2
    printf("RC OSC 76.800M=%u\r\n",       ((reg_data >> 13) & 62U) | ((reg_data >> 10) & 1U));
    printf("DIG LDO VOUT=%u EN=%u\r\n",   (reg_data >> 6) & 15U, (reg_data >> 5) & 1U);
    printf("Bandgap PMU=%u\r\n",          (reg_data >> 1) & 15U);
    reg_data = *((volatile uint32_t *)0x1A60A040);   // VBAT_ANA_REG2
#if defined(CONFIG_ENSEMBLE_GEN2)
    printf("Bandgap AON=%u\r\n", ((reg_data >> 22) & 30U) | ((reg_data >> 3) & 1U));
#else
    printf("Bandgap AON=%u\r\n", (reg_data >> 23) & 15U);
#endif
    printf("AON LDO VOUT=%u\r\n",    (reg_data >> 27) & 15U);
    printf("MAIN LDO VOUT=%u\r\n\n", (reg_data & 7U));

    /* miscellaneous */
    printf("MISC Settings:\r\n");
    printf("GPIO_FLEX=%s\r\n", ((volatile uint32_t *)0x1A609000) ? "1.8V" : "3.3V");// VBATALL GPIO_CTRL
#if defined(CONFIG_SOC_SERIES_E1C)
    printf("DCDC_MODE=PFM\r\n");
#else
    reg_data = *((volatile uint32_t *)0x1A60A034);
    printf("DCDC_MODE=%s\r\n", (reg_data >> 23) & 1U ? "PFM" : "PWM");
#endif
    reg_data = (*((volatile uint32_t *)0x1A60A030) >> 3) & 63U;
    printf("DCDC_TRIM[8:3]=%u (0x%02X)\r\n", reg_data, reg_data);
#if !(defined(CONFIG_ENSEMBLE_GEN2) || defined(CONFIG_SOC_SERIES_E1C))
    reg_data = *((volatile uint32_t *)0x1A60B000);
    printf("MDM_RET=0x%08X\r\n", reg_data);
    reg_data = *((volatile uint32_t *)0x1A60B008);
    printf("MDM_CTRL=0x%08X\r\n", reg_data);
#endif
    printf("\n");
}
