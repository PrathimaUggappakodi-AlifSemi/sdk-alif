/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https: //alifsemi.com/license
 *
 */

#include "aipm.h"
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#if defined(CONFIG_POWEROFF)
#include <zephyr/sys/poweroff.h>
#endif
#include <zephyr/drivers/counter.h>
#include <se_service.h>
#include <zephyr/sys/util.h>
#include <soc_common.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pm_system_off, LOG_LEVEL_INF);

/**
 * As per the application requirements, it can remove the memory blocks which are not in use.
 */
#if defined(CONFIG_SOC_SERIES_E1C) || defined(CONFIG_SOC_SERIES_B1)
	#define APP_RET_MEM_BLOCKS SRAM4_1_MASK | SRAM4_2_MASK | SRAM4_3_MASK | SRAM4_4_MASK | \
					SRAM5_1_MASK | SRAM5_2_MASK | SRAM5_3_MASK | SRAM5_4_MASK |\
					SRAM5_5_MASK
	#define SERAM_MEMORY_BLOCKS_IN_USE SERAM_1_MASK | SERAM_2_MASK | SERAM_3_MASK | SERAM_4_MASK
#else
	#define APP_RET_MEM_BLOCKS SRAM4_1_MASK | SRAM4_2_MASK | SRAM5_1_MASK | SRAM5_2_MASK
	#define SERAM_MEMORY_BLOCKS_IN_USE SERAM_MASK
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(rtc0), snps_dw_apb_rtc, okay)
	#define WAKEUP_SOURCE DT_NODELABEL(rtc0)
	#define SE_OFFP_EWIC_CFG EWIC_RTC_A
	#define SE_OFFP_WAKEUP_EVENTS WE_LPRTC
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(timer0), snps_dw_timers, okay)
	#define WAKEUP_SOURCE DT_NODELABEL(timer0)
	#define SE_OFFP_EWIC_CFG EWIC_VBAT_TIMER
	#define SE_OFFP_WAKEUP_EVENTS WE_LPTIMER0
#else
#error "Wakeup Device not enabled in the dts"
#endif

/* Sleep duration for PM_STATE_RUNTIME_IDLE */
#define RUNTIME_IDLE_SLEEP_USEC (18 * 1000 * 1000)
/* Sleep duration for PM_STATE_SUSPEND_TO_RAM substate 0 (STANDBY) */
#define S2RAM_STANDBY_SLEEP_USEC (20 * 1000 * 1000)
/* Sleep duration for PM_STATE_SUSPEND_TO_RAM substate 1 (STOP) */
#define S2RAM_STOP_SLEEP_USEC (22 * 1000 * 1000)
/* Sleep duration for PM_STATE_SOFT_OFF */
#define SOFT_OFF_SLEEP_USEC (24 * 1000 * 1000)
/* Wakeup duration for sys_poweroff (permanent power off) */
#define POWEROFF_WAKEUP_USEC (30 * 1000 * 1000)

/*
 * MRAM base address - used to determine boot location
 * TCM boot: VTOR = 0x0
 * MRAM boot: VTOR >= 0x80000000
 */
#define MRAM_BASE_ADDRESS 0x80000000

/*
 * Helper macro to check if booting from MRAM
 */
#define IS_BOOTING_FROM_MRAM() (SCB->VTOR >= MRAM_BASE_ADDRESS)

/*
 * PM_STATE_SUSPEND_TO_RAM (S2RAM) support:
 * - HP core: NOT supported (no retention capability)
 * - HE core + TCM boot: SUPPORTED (TCM retention keeps code and context)
 */
#if defined(CONFIG_RTSS_HE)
#define S2RAM_SUPPORTED (!IS_BOOTING_FROM_MRAM())
#else
#define S2RAM_SUPPORTED 0
#endif

#define START_WITH_HFRC 1
/*
 * PM_STATE_SOFT_OFF support:
 * - HP core: Always supported (no retention, must use SOFT_OFF)
 * - HE core + MRAM boot: Supported (MRAM preserved, wakeup possible)
 * - HE core + TCM boot: Skip (use S2RAM with retention instead)
 */
//#if defined(CONFIG_RTSS_HP)
#define SOFT_OFF_SUPPORTED 1
//#elif defined(CONFIG_RTSS_HE)
//#define SOFT_OFF_SUPPORTED IS_BOOTING_FROM_MRAM()
//#else
//#define SOFT_OFF_SUPPORTED 1
//#endif

#define OFF_STATE_NODE_ID DT_PHANDLE_BY_IDX(DT_NODELABEL(cpu0), cpu_power_states, 0)

//BUILD_ASSERT((RUNTIME_IDLE_SLEEP_USEC < DT_PROP_OR(OFF_STATE_NODE_ID, min_residency_us, 0)),
//	"RUNTIME_IDLE sleep should be less than min-residency-us");

#if 0
#if defined(CONFIG_RTSS_HE)
/* Additional validation for power state sleep durations */
BUILD_ASSERT((S2RAM_STOP_SLEEP_USEC > S2RAM_STANDBY_SLEEP_USEC),
	"STOP sleep duration should be greater than STANDBY sleep duration");
BUILD_ASSERT((SOFT_OFF_SLEEP_USEC > S2RAM_STOP_SLEEP_USEC),
	"SOFT_OFF sleep duration should be greater than STOP sleep duration");
#endif
#endif

/**
 * Helper function to lock/unlock deeper power states
 * @param lock true to lock deeper states (allow only RUNTIME_IDLE), false to unlock all
 */
void app_pm_lock_deeper_states(bool lock)
{
	const char *state_desc;

#if defined(CONFIG_RTSS_HP)
	/* HP core: only SOFT_OFF (no S2RAM support) */
	enum pm_state deep_states[] = {
		PM_STATE_SOFT_OFF
	};
	state_desc = "SOFT_OFF";

	for (int i = 0; i < ARRAY_SIZE(deep_states); i++) {
		if (lock) {
			pm_policy_state_lock_get(deep_states[i], PM_ALL_SUBSTATES);
		} else {
			pm_policy_state_lock_put(deep_states[i], PM_ALL_SUBSTATES);
		}
	}

#elif defined(CONFIG_RTSS_HE)
	/*
	 * HE core: States depend on boot location
	 * - TCM boot: S2RAM only (SOFT_OFF not needed with retention)
	 * - MRAM boot: SOFT_OFF only
	 */
	enum pm_state deep_states[2];
	int num_states = 0;

	if (S2RAM_SUPPORTED) {
		/* TCM boot: S2RAM works with retention */
		deep_states[num_states++] = PM_STATE_SUSPEND_TO_RAM;
		state_desc = "S2RAM";
	}

	if (SOFT_OFF_SUPPORTED) {
		/* MRAM boot: SOFT_OFF is the only deep sleep option for now */
		deep_states[num_states++] = PM_STATE_SOFT_OFF;
		state_desc = "SOFT_OFF";
	}

	for (int i = 0; i < num_states; i++) {
		if (lock) {
			pm_policy_state_lock_get(deep_states[i], PM_ALL_SUBSTATES);
		} else {
			pm_policy_state_lock_put(deep_states[i], PM_ALL_SUBSTATES);
		}
	}

#else
	#error "Unknown core type"
#endif

	LOG_DBG("%s deeper power state(s) (%s)",
	       lock ? "Locked" : "Unlocked", state_desc);
}

void app_pm_unlock_deeper_states(uint32_t period_ms)
{
	k_sleep(K_MSEC(period_ms));
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
}


#if defined(CONFIG_START_WITH_DIVIDED_HFRC)

/**
 * Set the RUN profile parameters for this application.
 */
static int app_set_run_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_SYST_MASK ;
	runp.dcdc_voltage = 775;
	runp.dcdc_mode = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src = CLK_SRC_LFXO;
	runp.run_clk_src = CLK_SRC_HFRC;
	runp.cpu_clk_freq = CLOCK_FREQUENCY_76_8_RC_MHZ;
	runp.phy_pwr_gating = 0;
	runp.ip_clock_gating = 0;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.scaled_clk_freq = SCALED_FREQ_RC_ACTIVE_38_4_MHZ;

	runp.memory_blocks = 0;

	ret = se_service_set_run_cfg(&runp);

	return ret;
}

SYS_INIT(app_set_run_params, PRE_KERNEL_1, 46);


int app_set_standby_params(void)
{
	run_profile_t runp;
	off_profile_t offp;
	int ret;

	runp.power_domains = PD_SSE700_AON_MASK ;
	runp.dcdc_voltage = 760;
	runp.dcdc_mode = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src = CLK_SRC_LFXO;
	runp.run_clk_src = CLK_SRC_HFRC;
	runp.cpu_clk_freq = CLOCK_FREQUENCY_76_8_RC_MHZ;
	runp.phy_pwr_gating = 0;
	runp.ip_clock_gating = 0;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.scaled_clk_freq = SCALED_FREQ_RC_ACTIVE_38_4_MHZ;

	runp.memory_blocks = 0;

	ret = se_service_set_run_cfg(&runp);
	__ASSERT(ret == 0, "SE: set_run_cfg failed = %d", ret);
	
	offp.wakeup_events     = 0;
    	offp.ewic_cfg          = 0;
    	offp.aon_clk_src       = CLK_SRC_LFXO;
    	offp.stby_clk_src      = CLK_SRC_HFRC;
    	offp.stby_clk_freq     = SCALED_FREQ_RC_STDBY_0_6_MHZ;
   	offp.memory_blocks     = SRAM4_1_MASK | SRAM4_2_MASK | SRAM5_1_MASK | SRAM5_2_MASK | SERAM_MASK;
    	offp.power_domains     = PD_SSE700_AON_MASK;
    	offp.dcdc_mode         = DCDC_MODE_PFM_FORCED;
    	offp.dcdc_voltage      = 760;
    	offp.vdd_ioflex_3V3    = IOFLEX_LEVEL_1V8;
    	offp.vtor_address      = SCB->VTOR;
    	offp.vtor_address_ns   = SCB->VTOR;
	
	ret = se_service_set_off_cfg(&offp);
	__ASSERT(ret == 0, "SE: set_off_cfg failed = %d", ret);
	
	return ret;
	
}
#endif


#if defined(CONFIG_START_WITH_HFRC)

/**
 * Set the RUN profile parameters for this application.
 */
static int app_set_run_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_VBAT_AON_MASK | PD_SYST_MASK | PD_SSE700_AON_MASK | PD_SESS_MASK;
	runp.dcdc_voltage = 775;
	runp.dcdc_mode = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src = CLK_SRC_LFXO;
	runp.run_clk_src = CLK_SRC_HFRC;
	runp.cpu_clk_freq = CLOCK_FREQUENCY_76_8_RC_MHZ;
	runp.phy_pwr_gating = 0;
	runp.ip_clock_gating = 0;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.scaled_clk_freq = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;

	runp.memory_blocks = MRAM_MASK;
	runp.memory_blocks |= SERAM_MEMORY_BLOCKS_IN_USE;
	runp.memory_blocks |= APP_RET_MEM_BLOCKS;

	if (IS_ENABLED(CONFIG_MIPI_DSI)) {
		runp.phy_pwr_gating |= MIPI_TX_DPHY_MASK | MIPI_RX_DPHY_MASK | MIPI_PLL_DPHY_MASK;
		runp.ip_clock_gating |= CDC200_MASK | MIPI_DSI_MASK | GPU_MASK;
	}

	ret = se_service_set_run_cfg(&runp);

	return ret;
}

SYS_INIT(app_set_run_params, PRE_KERNEL_1, 46);


int app_set_go4_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_SSE700_AON_MASK ;
	runp.dcdc_voltage  = 775;
	runp.dcdc_mode     = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src   = CLK_SRC_LFXO;
	runp.run_clk_src   = CLK_SRC_HFRC;
	runp.scaled_clk_freq = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.ip_clock_gating = 0;
	runp.phy_pwr_gating = 0;
#if defined(CONFIG_RTSS_HP)
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_76_8_RC_MHZ;
#else
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_76_8_RC_MHZ;
#endif

	runp.memory_blocks =  SERAM_MASK;

	ret = se_service_set_run_cfg(&runp);
	__ASSERT(ret == 0, "SE: set_run_cfg failed = %d", ret);

	return ret;
}


int app_set_ready2_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_SSE700_AON_MASK ;
	runp.dcdc_voltage  = 825;
	runp.dcdc_mode     = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src   = CLK_SRC_LFXO;
	runp.run_clk_src   = CLK_SRC_HFRC;
	runp.scaled_clk_freq = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.ip_clock_gating = 0;
	runp.phy_pwr_gating = 0;
#if defined(CONFIG_RTSS_HP)
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_76_8_RC_MHZ;
#else
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_76_8_RC_MHZ;
#endif

	runp.memory_blocks =  0;

	ret = se_service_set_run_cfg(&runp);
	__ASSERT(ret == 0, "SE: set_run_cfg failed = %d", ret);

	return ret;
}



#else

int app_set_go1_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_SYST_MASK | PD_SESS_MASK;
	runp.dcdc_voltage  = 825;
	runp.dcdc_mode     = DCDC_MODE_PWM;
	runp.aon_clk_src   = CLK_SRC_LFXO;
	runp.run_clk_src   = CLK_SRC_PLL;
	runp.scaled_clk_freq = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.ip_clock_gating = 0;
	runp.phy_pwr_gating = 0;
#if defined(CONFIG_RTSS_HP)
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_400MHZ;
#else
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_160MHZ;
#endif

	runp.memory_blocks =  SRAM2_MASK | SRAM3_MASK | MRAM_MASK;

	ret = se_service_set_run_cfg(&runp);
	__ASSERT(ret == 0, "SE: set_run_cfg failed = %d", ret);

	return ret;
}

int app_set_ready1_systop_on_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_SYST_MASK ;
	runp.dcdc_voltage  = 825;
	runp.dcdc_mode     = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src   = CLK_SRC_LFXO;
	runp.run_clk_src   = CLK_SRC_PLL;
	runp.scaled_clk_freq = SCALED_FREQ_RC_STDBY_0_6_MHZ;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.ip_clock_gating = 0;
	runp.phy_pwr_gating = 0;
#if defined(CONFIG_RTSS_HP)
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_400MHZ;
#else
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_160MHZ;
#endif

	runp.memory_blocks =  SRAM3_MASK;

	ret = se_service_set_run_cfg(&runp);
	__ASSERT(ret == 0, "SE: set_run_cfg failed = %d", ret);

	return ret;
}

int app_set_ready1_systop_off_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_SSE700_AON_MASK ;
	runp.dcdc_voltage  = 825;
	runp.dcdc_mode     = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src   = CLK_SRC_LFXO;
	runp.run_clk_src   = CLK_SRC_PLL;
	runp.scaled_clk_freq = SCALED_FREQ_RC_STDBY_0_6_MHZ;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.ip_clock_gating = 0;
	runp.phy_pwr_gating = 0;
#if defined(CONFIG_RTSS_HP)
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_400MHZ;
#else
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_160MHZ;
#endif

	runp.memory_blocks =  SRAM3_MASK;

	ret = se_service_set_run_cfg(&runp);
	__ASSERT(ret == 0, "SE: set_run_cfg failed = %d", ret);

	return ret;
}

int app_set_go3_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_SYST_MASK ;
	runp.dcdc_voltage  = 825;
	runp.dcdc_mode     = DCDC_MODE_PWM;
	runp.aon_clk_src   = CLK_SRC_LFXO;
	runp.run_clk_src   = CLK_SRC_PLL;
	runp.scaled_clk_freq = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.ip_clock_gating = 0;
	runp.phy_pwr_gating = 0;
#if defined(CONFIG_RTSS_HP)
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_400MHZ;
#else
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_160MHZ;
#endif

	runp.memory_blocks =  SERAM_MASK;

	ret = se_service_set_run_cfg(&runp);
	__ASSERT(ret == 0, "SE: set_run_cfg failed = %d", ret);

	return ret;
}

int app_set_stop1_params()
{
        off_profile_t offp;
        int ret;
        uint32_t regdata;
        
        offp.ewic_cfg = 0;
	offp.wakeup_events = 0;
	offp.vtor_address = SCB->VTOR;
	offp.vtor_address_ns = SCB->VTOR;
	offp.stby_clk_src      = CLK_SRC_HFRC;
        offp.stby_clk_freq     = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
        offp.power_domains     = 0;    // no power domains will request STOP Mode
        offp.dcdc_mode         = DCDC_MODE_PWM;
        offp.dcdc_voltage      = DCDC_VOUT_0825;
        offp.vdd_ioflex_3V3    = IOFLEX_LEVEL_1V8;
        offp.aon_clk_src       = CLK_SRC_LFXO;
        offp.memory_blocks     = SRAM4_1_MASK | SRAM4_2_MASK | SRAM5_1_MASK | SRAM5_2_MASK | BACKUP4K_MASK;
        
        //BOD-enable
        regdata = sys_read32((ANA_BASE + 0x40));
	regdata |= (1U << 8);
	sys_write32(regdata, (ANA_BASE + 0x40));
	
	//enable LPRTC
	regdata = sys_read32((VBAT_REG_BASE + 0x10));
	regdata |= 0x1;
	sys_write32(regdata, (VBAT_REG_BASE + 0x10));
        
        ret = se_service_set_off_cfg(&offp);
	__ASSERT(ret == 0, "SE: set_off_cfg failed = %d", ret);
	
	return ret;

}


int app_set_stop2_params()
{
        off_profile_t offp;
        int ret;
        uint32_t regdata;
        
        offp.ewic_cfg = 0;
	offp.wakeup_events = 0;
	offp.vtor_address = SCB->VTOR;
	offp.vtor_address_ns = SCB->VTOR;
	offp.stby_clk_src      = CLK_SRC_HFRC;
        offp.stby_clk_freq     = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
        offp.power_domains     = 0;    // no power domains will request STOP Mode
        offp.dcdc_mode         = DCDC_MODE_PWM;
        offp.dcdc_voltage      = DCDC_VOUT_0825;
        offp.vdd_ioflex_3V3    = IOFLEX_LEVEL_1V8;
        offp.aon_clk_src       = CLK_SRC_LFXO;
        offp.memory_blocks     = BACKUP4K_MASK;
        
        //BOD-enable
        regdata = sys_read32((ANA_BASE + 0x40));
	regdata |= (1U << 8);
	sys_write32(regdata, (ANA_BASE + 0x40));
	
	//enable LPRTC
	regdata = sys_read32((VBAT_REG_BASE + 0x10));
	regdata |= 0x1;
	sys_write32(regdata, (VBAT_REG_BASE + 0x10));
        
        ret = se_service_set_off_cfg(&offp);
	__ASSERT(ret == 0, "SE: set_off_cfg failed = %d", ret);
	
	return ret;

}



int app_set_stop3_params()
{
        off_profile_t offp;
        int ret;
        uint32_t regdata;
        
        offp.ewic_cfg = 0;
	offp.wakeup_events = 0;
	offp.vtor_address = SCB->VTOR;
	offp.vtor_address_ns = SCB->VTOR;
	offp.stby_clk_src      = CLK_SRC_HFRC;
        offp.stby_clk_freq     = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
        offp.power_domains     = 0;    // no power domains will request STOP Mode
        offp.dcdc_mode         = DCDC_MODE_PWM;
        offp.dcdc_voltage      = DCDC_VOUT_0825;
        offp.vdd_ioflex_3V3    = IOFLEX_LEVEL_1V8;
        offp.aon_clk_src       = CLK_SRC_LFXO;
        offp.memory_blocks     = 0;
        
        //BOD-enable
        regdata = sys_read32((ANA_BASE + 0x40));
	regdata |= (1U << 8);
	sys_write32(regdata, (ANA_BASE + 0x40));
	
	//enable LPRTC
	regdata = sys_read32((VBAT_REG_BASE + 0x10));
	regdata |= 0x1;
	sys_write32(regdata, (VBAT_REG_BASE + 0x10));
        
        ret = se_service_set_off_cfg(&offp);
	__ASSERT(ret == 0, "SE: set_off_cfg failed = %d", ret);
	
	return ret;

}

int app_set_stop4_params(void)
{
        off_profile_t offp;
        int ret;
        uint32_t regdata;
        
        offp.ewic_cfg = 0;
	offp.wakeup_events = 0;
	offp.vtor_address = SCB->VTOR;
	offp.vtor_address_ns = SCB->VTOR;
	offp.stby_clk_src      = CLK_SRC_HFRC;
        offp.stby_clk_freq     = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
        offp.power_domains     = 0;    // no power domains will request STOP Mode
        offp.dcdc_mode         = DCDC_MODE_PWM;
        offp.dcdc_voltage      = DCDC_VOUT_0825;
        offp.vdd_ioflex_3V3    = IOFLEX_LEVEL_1V8;
        offp.aon_clk_src       = CLK_SRC_LFXO;
        offp.memory_blocks     = 0;
        
        //BOD-disable
        regdata = sys_read32((ANA_BASE + 0x40));
	regdata &= ~(1U << 8);
	sys_write32(regdata, (ANA_BASE + 0x40));
	
	//enable LPRTC
	regdata = sys_read32((VBAT_REG_BASE + 0x10));
	regdata |= 0x1;
	sys_write32(regdata, (VBAT_REG_BASE + 0x10));
        
        ret = se_service_set_off_cfg(&offp);
	__ASSERT(ret == 0, "SE: set_off_cfg failed = %d", ret);
	
	return ret;

}



int app_set_stop5_params(void)
{
        off_profile_t offp;
        int ret;
        uint32_t regdata;
        
        offp.ewic_cfg = 0;
	offp.wakeup_events = 0;
	offp.vtor_address = SCB->VTOR;
	offp.vtor_address_ns = SCB->VTOR;
	offp.stby_clk_src      = CLK_SRC_HFRC;
        offp.stby_clk_freq     = SCALED_FREQ_RC_ACTIVE_76_8_MHZ;
        offp.power_domains     = 0;    // no power domains will request STOP Mode
        offp.dcdc_mode         = DCDC_MODE_PWM;
        offp.dcdc_voltage      = DCDC_VOUT_0825;
        offp.vdd_ioflex_3V3    = IOFLEX_LEVEL_1V8;
        offp.aon_clk_src       = CLK_SRC_LFXO;
        offp.memory_blocks     = 0;
        
        //BOD-disable
        regdata = sys_read32((ANA_BASE + 0x40));
	regdata &= ~(1U << 8);
	sys_write32(regdata, (ANA_BASE + 0x40));
	
	//disable LPRTC
	regdata = sys_read32((VBAT_REG_BASE + 0x10));
	regdata &= ~(0x1);
	sys_write32(regdata, (VBAT_REG_BASE + 0x10));
        
        ret = se_service_set_off_cfg(&offp);
	__ASSERT(ret == 0, "SE: set_off_cfg failed = %d", ret);
	
	return ret;

}



#endif






