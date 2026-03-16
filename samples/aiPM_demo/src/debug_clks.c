#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdint.h>
#include <cmsis_compiler.h>
#include <soc.h>

#include "debug_clks.h"

static const char *active[2] = {
    "[ ]", "[x]"
};

static const char *gated[2] = {
    "*", ""
};

void DEBUG_frequencies(const struct shell *shell) {
    uint32_t hfrc_top_clock = 76800000;
    uint32_t hfxo_top_clock = 38400000;
    uint32_t sys_osc_clk, periph_osc_clk;
    uint32_t cpupll_clk, syspll_clk, a32_cpuclk, gic_clk, ctrl_clk, dbg_clk;
    uint32_t syst_refclk, syst_aclk, syst_hclk, syst_pclk;
    uint32_t rtss_hp_clk, rtss_he_clk, pd4_clk;

    uint32_t hfrc_div_select = 0;
    uint32_t hfrc_div_active;
    uint32_t hfrc_div_standby;
    uint32_t hfxo_div;
    uint32_t bus_clk_div;
    uint32_t hf_osc_sel;
    uint32_t pll_clk_sel;
    uint32_t es_clk_sel;
    uint32_t clk_ena;
    uint32_t pll_boost;

    uint32_t reg_data;

    reg_data = *((volatile uint32_t *)0x1A60A03C);
    hfrc_div_active = (reg_data >> 11) & 7U;
    hfrc_div_standby = (reg_data >> 19) & 7U;

    if (hfrc_div_standby > 6) hfrc_div_standby += 3;        // 2^(7   + 3) = 1024 (75k)
    else if (hfrc_div_standby > 3) hfrc_div_standby += 2;   // 2^(4-6 + 2) = 64-256 (1.2M-300k)
    else if (hfrc_div_standby > 2) hfrc_div_standby += 1;   // 2^(3   + 1) = 16 (4.8M)
                                                            // 2^(0-2 + 0) = 1-4 (76.8M-19.2M)

#if defined(CONFIG_SOC_SERIES_E1C)
    reg_data = *((volatile uint32_t *)0x1A60A004);
    hfrc_div_select = reg_data & 3U;
#else
    reg_data = *((volatile uint32_t *)0x1A60A008);
    hfrc_div_select = (reg_data >> 6) & 3U;
#endif

    /* value of 0, 1, or 3 means HFRC ACTIVE divider is used */
    if (hfrc_div_select == 2)
        hfrc_div_select = 1;    // HFRC STANDBY is selected
    else
        hfrc_div_select = 0;    // HFRC ACTIVE is selected

    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Top Level Clock Sources\r\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "HFRC ACTIVE%10dHz%s\r\n", hfrc_top_clock >> hfrc_div_active, active[hfrc_div_select^1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "HFRC STANDBY%9dHz%s\r\n", hfrc_top_clock >> hfrc_div_standby, active[hfrc_div_select]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "[x] means clock is selected\r\n\n");

 #if 0   
    reg_data = *((volatile uint32_t *)0x1A60400C);  // MISC_REG1
#if defined(CONFIG_ENSEMBLE_GEN2) || defined(CONFIG_SOC_SERIES_E1C)
    hfxo_div = (reg_data >> 17) & 15U;
#else
    hfxo_div = (reg_data >> 13) & 15U;
#endif
    if (hfxo_div > 7) {
        hfxo_div -= 8;
        if (hfxo_div > 6) hfxo_div += 3;            // 2^(7   + 3) = 1024 (75k)
        else if (hfxo_div > 3) hfxo_div += 2;       // 2^(4-6 + 2) = 64-256 (1.2M-300k)
        else if (hfxo_div > 2) hfxo_div += 1;       // 2^(3   + 1) = 16 (4.8M)
                                                    // 2^(0-2 + 0) = 1-4 (76.8M-19.2M)
    }
    hfxo_top_clock >>= hfxo_div;
#endif 
    reg_data = *((volatile uint32_t *)0x1A60A034);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "LFXO CLK%13dHz%s\r\n", 32768, active[reg_data & 1U]);
    reg_data = *((volatile uint32_t *)0x1A605020);  // XO_REG1
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "HFXO CLK%13dHz%s\r\n", hfxo_top_clock, active[reg_data & 1U]);

    reg_data = *((volatile uint32_t *)0x1A602000);  // PLL_LOCK
#if defined(CONFIG_ENSEMBLE_GEN2)
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "PLL  CLK    800000000Hz%s\r\n", active[reg_data & 1U]);
#elif defined(CONFIG_SOC_SERIES_E1C)
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "PLL  CLK    480000000Hz%s\r\n", active[reg_data & 1U]);
#else
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "PLL  CLK   2400000000Hz%s\r\n", active[reg_data & 1U]);
#endif
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "[x] means clock is enabled\r\n\n");

    /* calculate hfrc_top_clock based on standby override bit */
    if (hfrc_div_select) {
        hfrc_top_clock >>= hfrc_div_standby;
    } else {
        hfrc_top_clock >>= hfrc_div_active;
    }

   
    hf_osc_sel = *((volatile uint32_t *)0x1A602004);
    pll_clk_sel = *((volatile uint32_t *)0x1A602008);
    #if defined(CONFIG_ENSEMBLE_GEN2)
    es_clk_sel = *((volatile uint32_t *)0x1A602010);
    #else
    es_clk_sel = *((volatile uint32_t *)0x1A60200C);
    #endif
    clk_ena = *((volatile uint32_t *)0x1A602014);
    //bus_clk_div = *((volatile uint32_t *)0x1A604008);

#if defined(CONFIG_SOC_SERIES_E1C)
    pll_boost = ((*(volatile uint32_t *)(0x1A605028)) >> 19) & 1;
#endif

    /* calculate the sys_osc_clk */
    if (hf_osc_sel & 1) {
#if defined(CONFIG_ENSEMBLE_GEN2) || defined(CONFIG_SOC_SERIES_E1C)
        sys_osc_clk = 76800000;
#else
        sys_osc_clk = hfxo_top_clock;
#endif
    }
    else {
        sys_osc_clk = hfrc_top_clock;
    }

    /* calculate the periph_osc_clk */
    if (hf_osc_sel & 16) {
        periph_osc_clk = hfxo_top_clock;
    }
    else {
        periph_osc_clk = hfrc_top_clock >> 1;
    }
#if defined(CONFIG_ENSEMBLE_GEN2)
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "HFXO_OUT%13dHz%s\r\n", sys_osc_clk, (clk_ena & (1U << 19)) == 0 ? "*" : "");
#elif defined(CONFIG_SOC_SERIES_E1C)
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "HFXO_OUT%13dHz%s\r\n", sys_osc_clk, (clk_ena & (1U << 18)) == 0 ? "*" : "");
#endif 
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "HFOSC_CLK%12dHz%s\r\n", periph_osc_clk, gated[(clk_ena >> 23) & 1U]);

     
    /* calculate the REFCLK */
    if (pll_clk_sel & 1) {
#if defined(CONFIG_SOC_SERIES_E1C)
        if (pll_boost)
            syst_refclk = 120000000;
        else
            syst_refclk = 80000000;
#else
        syst_refclk = 100000000;
#endif
    }
    else {
        syst_refclk = sys_osc_clk;
    }

    /* calculate the CPUPLL_CLK and SYSPLL_CLK */
    if (pll_clk_sel & 16) {
#if defined(CONFIG_SOC_SERIES_E1C)
        if (pll_boost)
            syspll_clk = 240000000;
        else
            syspll_clk = 160000000;
#else
            cpupll_clk = 800000000;
            syspll_clk = 400000000;
#endif
    }
    else {
        cpupll_clk = sys_osc_clk;
        syspll_clk = sys_osc_clk;
    }

#if !defined(CONFIG_SOC_SERIES_E1C)
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "CPUPLL_CLK%11dHz%s\r\n", cpupll_clk, gated[(clk_ena >> 4) & 1U]);
#endif
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "SYSPLL_CLK%11dHz%s\r\n", syspll_clk, gated[clk_ena & 1U]);

   
#if !defined(CONFIG_SOC_SERIES_E1C)
    /* calculate the APSS_CLK */
    /* HOSTCPUCLK_CTRL value at [7:0] and
     * HOSTCPUCLK_DIV0 value at [15:8] and
     * HOSTCPUCLK_DIV1 value at [23:16] */
    uint32_t apssclk_status = 0;
    reg_data = *((volatile uint32_t *)(0x1A010800));
    apssclk_status |= (reg_data >> 8) & 0xFF;
    reg_data = *((volatile uint32_t *)(0x1A010804));
    apssclk_status |= ((reg_data >> 16) & 0xFF) << 8;
    reg_data = *((volatile uint32_t *)(0x1A010808));
    apssclk_status |= ((reg_data >> 16) & 0xFF) << 16;
    if ((apssclk_status & 0xF) == 4) {
        a32_cpuclk = cpupll_clk;
        a32_cpuclk /= ((apssclk_status >> 16) & 0xFF) + 1;   // apssclk divider is (n + 1)
    }
    else if ((apssclk_status & 0xF) == 2) {
        a32_cpuclk = syspll_clk;
        a32_cpuclk /= ((apssclk_status >> 8) & 0xFF) + 1;    // apssclk divider is (n + 1)
    }
    else if ((apssclk_status & 0xF) == 1) {
        a32_cpuclk = syst_refclk;
    }
    else {
        a32_cpuclk = 0;
    }
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "A32_CPUCLK%11dHz%s\r\n", a32_cpuclk, (apssclk_status & 0xF) == 0 ? gated[0] : gated[1]);

    /* calculate the GIC_CLK */
    /* GICCLK_CTRL value at [7:0] and
     * GICCLK_DIV0 value at [15:8] */
    uint32_t gicclk_status = 0;
    reg_data = *((volatile uint32_t *)(0x1A010810));
    gicclk_status |= (reg_data >> 8) & 0xFF;
    reg_data = *((volatile uint32_t *)(0x1A010814));
    gicclk_status |= ((reg_data >> 16) & 0xFF) << 8;
    if ((gicclk_status & 0xF) == 2) {
        gic_clk = syspll_clk;
        gic_clk /= ((gicclk_status >> 8) & 0xFF) + 1;        // gicclk divider is (n + 1)
    }
    else if ((gicclk_status & 0xF) == 1) {
        gic_clk = syst_refclk;
    }
    else {
        gic_clk = 0;
    }
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "GIC_CLK%14dHz%s\r\n", gic_clk, (gicclk_status & 0xF) == 0 ? gated[0] : gated[1]);
#endif

    /* calculate the SYST_ACLK, HCLK, PCLK */
    /* ACLK_CTRL value at [7:0] and
     * ACLK_DIV0 value at [15:8] */
    uint32_t aclk_status = 0;
    reg_data = *((volatile uint32_t *)(0x1A010820));
    aclk_status |= (reg_data >> 8) & 0xFF;
    reg_data = *((volatile uint32_t *)(0x1A010824));
    aclk_status |= ((reg_data >> 16) & 0xFF) << 8;
    if ((aclk_status & 0xF) == 2) {
        syst_aclk = syspll_clk;
        syst_aclk /= ((aclk_status >> 8) & 0xFF) + 1;        // aclk divider is (n + 1)
    }
    else if ((aclk_status & 0xF) == 1) {
        syst_aclk = syst_refclk;
    }
    else {
        syst_aclk = 0;
    }
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "SYST_ACLK%12dHz%s\r\n", syst_aclk, (aclk_status & 0xF) == 0 ? gated[0] : gated[1]);
#if defined(CONFIG_ENSEMBLE_GEN2)
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " SRAM0_CLK%11dHz%s\r\n", syst_aclk, gated[(clk_ena >> 27) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " SRAM1_CLK%11dHz%s\r\n", syst_aclk, gated[(clk_ena >> 28) & 1U]);
#elif defined(CONFIG_SOC_SERIES_E1C)
#else
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " SRAM0_CLK%11dHz%s\r\n", syst_aclk, gated[(clk_ena >> 24) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " SRAM1_CLK%11dHz%s\r\n", syst_aclk, gated[(clk_ena >> 28) & 1U]);
#endif
 
    /* HCLK and PCLK divider is 2^n, but n = 0, 1, or 2 only */
#if defined(CONFIG_ENSEMBLE_GEN2) || defined(CONFIG_SOC_SERIES_E1C)
    reg_data = (bus_clk_div >> 8) & 3U; if (reg_data == 3) reg_data = 2;
    syst_hclk = syspll_clk >> reg_data;

    reg_data = bus_clk_div & 3U; if (reg_data == 3) reg_data = 2;
    syst_pclk = syspll_clk >> reg_data;
#else
    reg_data = (bus_clk_div >> 8) & 3U; if (reg_data == 3) reg_data = 2;
    syst_hclk = syst_aclk >> reg_data;

    reg_data = bus_clk_div & 3U; if (reg_data == 3) reg_data = 2;
    syst_pclk = syst_aclk >> reg_data;
#endif

    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " SYST_HCLK%11dHz\r\n", syst_hclk);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " SYST_PCLK%11dHz\r\n", syst_pclk);

    /* calculate the CTRL_CLK */
    /* CTRLCLK_CTRL value at [7:0] and
     * CTRLCLK_DIV0 value at [15:8] */
    uint32_t ctrlclk_status = 0;
    reg_data = *((volatile uint32_t *)(0x1A010830));
    ctrlclk_status |= (reg_data >> 8) & 0xFF;
    reg_data = *((volatile uint32_t *)(0x1A010834));
    ctrlclk_status |= ((reg_data >> 16) & 0xFF) << 8;
    if ((ctrlclk_status & 0xF) == 2) {
        ctrl_clk = syspll_clk;
        ctrl_clk /= ((ctrlclk_status >> 8) & 0xFF) + 1;      // ctrlclk divider is (n + 1)
    }
    else if ((ctrlclk_status & 0xF) == 1) {
        ctrl_clk = syst_refclk;
    }
    else {
        ctrl_clk = 0;
    }
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "CTRL_CLK%13dHz%s\r\n", ctrl_clk, (ctrlclk_status & 0xF) == 0 ? gated[0] : gated[1]);

    /* calculate the DBG_CLK */
    /* DBGCLK_CTRL value at [7:0] and
     * DBGCLK_DIV0 value at [15:8] */
    uint32_t dbgclk_status = 0;
    reg_data = *((volatile uint32_t *)(0x1A010840));
    dbgclk_status |= (reg_data >> 8) & 0xFF;
    reg_data = *((volatile uint32_t *)(0x1A010844));
    dbgclk_status |= ((reg_data >> 16) & 0xFF) << 8;
    if ((dbgclk_status & 0xF) == 2) {
        dbg_clk = syspll_clk;
        dbg_clk /= ((dbgclk_status >> 8) & 0xFF) + 1;        // dbgclk divider is (n + 1)
    }
    else if ((dbgclk_status & 0xF) == 1) {
        dbg_clk = syst_refclk;
    }
    else {
        dbg_clk = 0;
    }
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "DBG_CLK%14dHz%s\r\n", dbg_clk, (dbgclk_status & 0xF) == 0 ? gated[0] : gated[1]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "SYST_REFCLK%10dHz\r\n", syst_refclk);

#if !defined(CONFIG_SOC_SERIES_E1C)
    /* calculate the RTSS_HP_CLK */
    const uint32_t pll_rtss_hp_clocks[4] = {100000000UL, 200000000UL, 300000000UL, 400000000UL};
    if (pll_clk_sel & (1U << 16)) {
        rtss_hp_clk = pll_rtss_hp_clocks[es_clk_sel & 3U];
    }
    else {
        switch((es_clk_sel >> 8) & 3U) {
        case 0:
            rtss_hp_clk = hfrc_top_clock;
            break;
        case 1:
            rtss_hp_clk = hfrc_top_clock >> 1;
            break;
        case 2:
            rtss_hp_clk = 76800000;
            break;
        case 3:
            rtss_hp_clk = hfxo_top_clock;
            break;
        }
    }
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "RTSS_HP_CLK%10dHz%s\r\n", rtss_hp_clk, gated[(clk_ena >> 12) & 1U]);
#endif

    /* calculate the RTSS_HE_CLK */
    const uint32_t pll_rtss_he_clocks[4] = {60000000UL, 80000000UL, 120000000UL, 160000000UL};
    if (pll_clk_sel & (1U << 20)) {
        rtss_he_clk = pll_rtss_he_clocks[(es_clk_sel >> 4) & 3U];
    }
    else {
        switch((es_clk_sel >> 12) & 3U) {
        case 0:
            rtss_he_clk = hfrc_top_clock;
            break;
        case 1:
            rtss_he_clk = hfrc_top_clock >> 1;
            break;
        case 2:
            rtss_he_clk = 76800000;
            break;
        case 3:
            rtss_he_clk = hfxo_top_clock;
            break;
        }
    }
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "RTSS_HE_CLK%10dHz%s\r\n", rtss_he_clk, gated[(clk_ena >> 13) & 1U]);


#if !(defined(CONFIG_ENSEMBLE_GEN2) || defined(CONFIG_SOC_SERIES_E1C))
    uint32_t pll_pd4_clocks[4] = {hfxo_top_clock, 0, 120000000, 160000000};

    /* calculate PD4 SRAM6 to SRAM9 clocks */
    if (*((volatile uint32_t *)0x1A60504C) & 1U) {
        pd4_clk = pll_pd4_clocks[*((volatile uint32_t *)0x1A605040) & 3U];
    }
    else {
        pd4_clk = pll_pd4_clocks[0];
    }

    /* check if PD4 is alive */
    reg_data = *((volatile uint32_t *)0x1A605058);
    reg_data &= 0x7FFUL;
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "PD4_CLK%14dHz%s\r\n", pd4_clk, reg_data > 1 ? gated[1] : gated[0]);
#endif


#if defined(CONFIG_ENSEMBLE_GEN2)
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " 10M_CLK%13dHz%s\r\n",  10000000, gated[(clk_ena >> 9) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " 20M_CLK%13dHz%s\r\n",  20000000, gated[(clk_ena >> 22) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " 25M_CLK%13dHz%s\r\n",  25000000, gated[(clk_ena >> 5) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " 50M_CLK%13dHz%s\r\n",  50000000, gated[(clk_ena >> 6) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " 80M_CLK%13dHz%s\r\n",  80000000, gated[(clk_ena >> 10) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "100M_CLK%13dHz%s\r\n", 100000000, gated[(clk_ena >> 7) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "160M_CLK%13dHz%s\r\n", 160000000, gated[(clk_ena >> 20) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "200M_CLK%13dHz%s\r\n", 200000000, gated[(clk_ena >> 14) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "266M_CLK%13dHz%s\r\n", 266000000, gated[(clk_ena >> 21) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "400M_CLK%13dHz%s\r\n", 400000000, gated[(clk_ena >> 15) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "38.4M_CLK%12dHz%s\r\n", 38400000, gated[(clk_ena >> 23) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "76.8M_CLK%12dHz%s\r\n", 76800000, gated[(clk_ena >> 24) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "MRAM_CLK%13dHz%s\r\n", syst_aclk/6, gated[(clk_ena >> 11) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ISP_CLK %13dHz%s\r\n", syst_aclk, gated[(clk_ena >> 29) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "JPEG_CLK%13dHz%s\r\n", syst_aclk, gated[(clk_ena >> 30) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "U85_CLK %13dHz%s\r\n", syst_aclk, gated[(clk_ena >> 31) & 1U]);
#else
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "10M/20M_CLK%10dHz%s\r\n", 20000000, gated[(clk_ena >> 22) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "100M_CLK%13dHz%s\r\n", 100000000, gated[(clk_ena >> 21) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "160M_CLK%13dHz%s\r\n", 160000000, gated[(clk_ena >> 20) & 1U]);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "38.4M_CLK%12dHz%s\r\n", 38400000, gated[(clk_ena >> 23) & 1U]);
#endif
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "* means clock is gated\r\n\n");
  
}
