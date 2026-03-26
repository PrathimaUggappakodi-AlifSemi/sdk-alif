/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/* This application demonstrates the communication and control of a device
 * allowing to remotely control an LED, and to transmit the state of a button.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/poweroff.h>
#include <cmsis_core.h>
#include <soc.h>
#include <se_service.h>
#include <es0_power_manager.h>
#include <zephyr/shell/shell.h>

#include "debug_pwr.h"
#include "debug_clks.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
//This is used for the LED toggle in the main function. It can be changed as per the requirement. 
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

//The below function locks the S2RAM and SOFT OFF state to prevent the system from going into these states while executing the other test cases. 
// This is required since the system cannot wake up from these states without a reset and it will be difficult to debug if the system goes into either of these states while executing the other test cases.

static int app_pre_kernel_init(void)
{

	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);

	return 0;
}
SYS_INIT(app_pre_kernel_init, PRE_KERNEL_1, 39);

//Enable CONFIG GPIO to toggle the LED in the main function to ensure Zephyr is running during lowest scaled clk freq setting.

#if defined(CONFIG_GPIO)

int app_set_hfrc_params(void)
{
	run_profile_t runp;
	int ret;

	runp.power_domains = PD_SSE700_AON_MASK ;
	runp.dcdc_voltage  = 775;
	runp.dcdc_mode     = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src   = CLK_SRC_LFXO;
	runp.run_clk_src   = CLK_SRC_HFRC;
	runp.scaled_clk_freq = SCALED_FREQ_RC_ACTIVE_0_6_MHZ;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.ip_clock_gating = 0;
	runp.phy_pwr_gating = 0;
	runp.cpu_clk_freq  = CLOCK_FREQUENCY_76_8_RC_MHZ;
	runp.memory_blocks =  SERAM_MASK;

	ret = se_service_set_run_cfg(&runp);
	__ASSERT(ret == 0, "SE: set_run_cfg failed = %d", ret);
	

	return ret;
}
//The below cfg is set while booting Zephyr in the lowest power mode.
SYS_INIT(app_set_hfrc_params, PRE_KERNEL_1, 46);
#endif

/* Macros */
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);


int main(void)
{
	#if defined (CONFIG_GPIO)
	
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}

    #else
	printk("PM_SHELL test app boot\n");

	while (1) {
		/*  Sleep Long period */
		k_sleep(K_MSEC(1000000));

	}
	#endif
	return 0;
}
