/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

#include <zephyr/shell/shell.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/balletto-pinctrl.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#if defined(CONFIG_PM)
#include <power_mgr.h>
#include "aiPM_modes.h"
#endif
#include <es0_power_manager.h>
#include "se_service.h"
#include <stdlib.h>
#include <inttypes.h>

#define LOG_MODULE_NAME alif_power_mgr_shell
#define LOG_LEVEL       LOG_LEVEL_INFO

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define NVD_BOOT_PARAMS_MAX_SIZE (512)

#define LL_CLK_SEL_CTRL_REG_ADDR   0x1A60201C
#define LL_UART_CLK_SEL_CTRL_16MHZ 0x00
#define LL_UART_CLK_SEL_CTRL_24MHZ 0x01
#define LL_UART_CLK_SEL_CTRL_48MHZ 0x03

/* Tag status: (STATUS_VALID | STATUS_NOT_LOCKED | STATUS_NOT_ERASED) */
#define DEFAULT_TAG_STATUS (0x00 | 0x02 | 0x04)

/* Boot time value definitions */
#define BOOT_PARAM_ID_LE_CODED_PHY_500          0x85
#define BOOT_PARAM_ID_DFT_SLAVE_MD              0x20
#define BOOT_PARAM_ID_CH_CLASS_REP_INTV         0x36
#define BOOT_PARAM_ID_BD_ADDRESS                0x01
#define BOOT_PARAM_ID_ACTIVITY_MOVE_CONFIG      0x15
#define BOOT_PARAM_ID_SCAN_EXT_ADV              0x16
#define BOOT_PARAM_ID_RSSI_HIGH_THR             0x3A
#define BOOT_PARAM_ID_RSSI_LOW_THR              0x3B
#define BOOT_PARAM_ID_SLEEP_ENABLE              0x11
#define BOOT_PARAM_ID_EXT_WAKEUP_ENABLE         0x12
#define BOOT_PARAM_ID_ENABLE_CHANNEL_ASSESSMENT 0x19
#define BOOT_PARAM_ID_RSSI_INTERF_THR           0x3C
#define BOOT_PARAM_ID_UART_BAUDRATE             0x10
#define BOOT_PARAM_ID_UART_INPUT_CLK_FREQ       0xC0
#define BOOT_PARAM_ID_NO_PARAM                  0xFF
#define BOOT_PARAM_ID_EXT_WAKEUP_TIME           0x0D
#define BOOT_PARAM_ID_OSC_WAKEUP_TIME           0x0E
#define BOOT_PARAM_ID_RM_WAKEUP_TIME            0x0F
#define BOOT_PARAM_ID_EXT_WARMBOOT_WAKEUP_TIME  0xD0
#define BOOT_PARAM_ID_LPCLK_DRIFT               0x07
#define BOOT_PARAM_ID_ACTCLK_DRIFT              0x09
#define BOOT_PARAM_ID_CONFIGURATION             0xD1

#define BOOT_PARAM_LEN_LE_CODED_PHY_500          1
#define BOOT_PARAM_LEN_DFT_SLAVE_MD              1
#define BOOT_PARAM_LEN_CH_CLASS_REP_INTV         2
#define BOOT_PARAM_LEN_BD_ADDRESS                6
#define BOOT_PARAM_LEN_ACTIVITY_MOVE_CONFIG      1
#define BOOT_PARAM_LEN_SCAN_EXT_ADV              1
#define BOOT_PARAM_LEN_RSSI_THR                  1
#define BOOT_PARAM_LEN_SLEEP_ENABLE              1
#define BOOT_PARAM_LEN_EXT_WAKEUP_ENABLE         1
#define BOOT_PARAM_LEN_ENABLE_CHANNEL_ASSESSMENT 1
#define BOOT_PARAM_LEN_UART_BAUDRATE             4
#define BOOT_PARAM_LEN_UART_INPUT_CLK_FREQ       4
#define BOOT_PARAM_LEN_EXT_WAKEUP_TIME           2
#define BOOT_PARAM_LEN_OSC_WAKEUP_TIME           2
#define BOOT_PARAM_LEN_RM_WAKEUP_TIME            2
#define BOOT_PARAM_LEN_EXT_WARMBOOT_WAKEUP_TIME  2
#define BOOT_PARAM_LEN_LPCLK_DRIFT               2
#define BOOT_PARAM_LEN_ACTCLK_DRIFT              1
#define BOOT_PARAM_LEN_CONFIGURATION             4

#define ES0_PM_ERROR_NO_ERROR             0
#define ES0_PM_ERROR_TOO_MANY_USERS       -1
#define ES0_PM_ERROR_TOO_MANY_BOOT_PARAMS -2
#define ES0_PM_ERROR_INVALID_BOOT_PARAMS  -3
#define ES0_PM_ERROR_START_FAILED         -4
#define ES0_PM_ERROR_NO_BAUDRATE          -5
#define ES0_PM_ERROR_BAUDRATE_MISMATCH    -6

/* LP timer callback status */
static volatile int lp_timer_fired = 0;

/* Zephyr timer for low-power duration */
K_TIMER_DEFINE(lp_timer, NULL, NULL);


#if defined(CONFIG_PM)
static uint32_t wakeup_counter __attribute__((noinit));
static pm_state_mode_type_e pm_off_mode __attribute__((noinit));


static int pm_application_init(void)
{
	if (power_mgr_cold_boot()) {
		wakeup_counter = 0;
	} else {
		if (wakeup_counter) {
			/* Set a off profile */
			if (power_mgr_set_offprofile(pm_off_mode)) {
				printk("Error to set off profile\n");
				wakeup_counter = 0;
				return 0;
			}
			wakeup_counter--;
			//power_mgr_ready_for_sleep();
			//power_mgr_set_subsys_off_period(sleep_period);
		}
	}
	return 0;
}
SYS_INIT(pm_application_init, APPLICATION, 1);


static int cmd_standby_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start Standby mode \n");
	
        int ret = app_set_standby_params();
        app_pm_lock_deeper_states(true);
        app_pm_unlock_deeper_states(1000);
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Standby state set \n");

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		//wakeup_counter = 0;
		return ret;
	}

	return ret;
}

static int cmd_ready1_systop_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start Ready 1 mode with SYSTOP on\n");
	
        int ret = app_set_ready1_systop_on_params();
        app_pm_lock_deeper_states(true);
         k_sleep(K_MSEC(1000));
	pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
	k_sleep(K_FOREVER);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		//wakeup_counter = 0;
		return ret;
	}

	return ret;
}


static int cmd_ready1_systop_off_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start Ready 1 mode with SYSTOP on\n");
	
        int ret = app_set_ready1_systop_off_params();
        app_pm_lock_deeper_states(true);
         k_sleep(K_MSEC(1000));
	pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
	k_sleep(K_FOREVER);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		//wakeup_counter = 0;
		return ret;
	}

	return ret;
}


static int cmd_go1_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start GO 1 mode \n");
	
        int ret = app_set_go1_params();

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		//wakeup_counter = 0;
		return ret;
	}

	return ret;
}

static int cmd_go3_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start GO 3 mode \n");
	
        int ret = app_set_go3_params();

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		//wakeup_counter = 0;
		return ret;
	}

	return ret;
}

static int cmd_go4_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start GO 4 mode \n");
	
        int ret = app_set_go4_params();

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		//wakeup_counter = 0;
		return ret;
	}

	return ret;
}

static int cmd_ready2_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start Ready 2 mode \n");
	
        int ret = app_set_ready2_params();
        app_pm_lock_deeper_states(true);
        k_sleep(K_MSEC(1000));
	pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
	k_sleep(K_FOREVER);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		//wakeup_counter = 0;
		return ret;
	}

	return ret;
}
static int cmd_stop1_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start STOP 1 mode \n");
	
        int ret = app_set_stop1_params();
        app_pm_lock_deeper_states(false);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		return ret;
	}

	return ret;
}

static int cmd_stop2_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start STOP 2 mode \n");
	
        int ret = app_set_stop2_params();
        app_pm_lock_deeper_states(false);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		return ret;
	}

	return ret;
}

static int cmd_stop3_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start STOP 3 mode \n");
	
        int ret = app_set_stop3_params();
        app_pm_lock_deeper_states(false);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		return ret;
	}

	return ret;
}

static int cmd_stop4_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start STOP 4 mode \n");
	
        int ret = app_set_stop4_params();
        app_pm_lock_deeper_states(false);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		return ret;
	}

	return ret;
}

static int cmd_stop5_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start STOP 5 mode \n");
	
        int ret = app_set_stop5_params();
        app_pm_lock_deeper_states(false);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		return ret;
	}

	return ret;
}
#endif


SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_cmds, /*SHELL_CMD_ARG(start, NULL, "es0 start", cmd_start, 1, 10),*/
#if defined(CONFIG_PM)
        
        #if !defined(CONFIG_START_WITH_HFRC)
	SHELL_CMD_ARG(pm_go1, NULL, "Start GO1-state ", cmd_go1_test, 1,
		      10),
	SHELL_CMD_ARG(pm_go3, NULL, "Start GO3-state ", cmd_go3_test, 1,
		      10),
	SHELL_CMD_ARG(pm_ready1_with_systop, NULL, "Start ready with systop on-state ", cmd_ready1_systop_test, 1,
		      10),
	SHELL_CMD_ARG(pm_ready1_with_off_systop, NULL, "Start ready with systop off-state ", cmd_ready1_systop_off_test, 1,
		      10),
	SHELL_CMD_ARG(pm_stop1, NULL, "Start STOP1-state ", cmd_stop1_test, 1,
		      10),
	SHELL_CMD_ARG(pm_stop2, NULL, "Start STOP2-state ", cmd_stop2_test, 1,
		      10),
	SHELL_CMD_ARG(pm_stop3, NULL, "Start STOP3-state ", cmd_stop3_test, 1,
		      10),
	SHELL_CMD_ARG(pm_stop4, NULL, "Start STOP4-state ", cmd_stop4_test, 1,
		      10),
	SHELL_CMD_ARG(pm_stop5, NULL, "Start STOP5-state ", cmd_stop5_test, 1,
		      10), 
	#elif defined(CONFIG_START_WITH_HFRC)	
	SHELL_CMD_ARG(pm_go4, NULL, "Start GO4-state ", cmd_go4_test, 1,
		      10),	
	SHELL_CMD_ARG(pm_ready2, NULL, "Start Ready2-state ", cmd_ready2_test, 1,
		      10),
        #endif
		      
	#if defined(CONFIG_START_WITH_DIVIDED_HFRC)
	//SHELL_CMD_ARG(pm_standby, NULL, "Start standby-state ", cmd_standby_test, 1,
	//	      10),
	#endif


#endif
	//SHELL_CMD_ARG(stop, NULL, "es0 stop", cmd_stop, 1, 10),

	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pwr, &sub_cmds, "Power management test commands", NULL);
