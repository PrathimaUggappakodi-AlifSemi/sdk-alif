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
#include <zephyr/sys/poweroff.h>
#include <cmsis_core.h>
#include <soc.h>
#include <se_service.h>
#include <es0_power_manager.h>

#include <power_mgr.h>



/*
 * CRITICAL: Must run at PRE_KERNEL_1 to restore SYSTOP before peripherals initialize.
 *
 * On cold boot: SYSTOP is already ON by default, safe to call.
 * On SOFT_OFF wakeup: SYSTOP is OFF, must restore BEFORE peripherals access registers.
 */
//SYS_INIT(app_set_run_params, PRE_KERNEL_1, 46);


/* Macros */
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);


int main(void)
{
	if (power_mgr_cold_boot()) {
		printk("PM_SHELL test app boot\n");
	}
	#if 0
	#if defined(CONFIG_START_WITH_DIVIDED_HFRC)
	app_set_standby_params();
	app_pm_lock_deeper_states(true);
	app_pm_unlock_deeper_states(1000);
	#endif
	#endif

	while (1) {
		/*  Sleep Long period */
		k_sleep(K_MSEC(1000000));

	}
	return 0;
}
