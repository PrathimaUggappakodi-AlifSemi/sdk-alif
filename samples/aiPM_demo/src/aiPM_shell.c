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
#include "aiPM_modes.h"
#endif
#include <es0_power_manager.h>
#include "se_service.h"
#include <stdlib.h>
#include <inttypes.h>

//#define LOG_MODULE_NAME alif_power_mgr_shell
#define LOG_LEVEL       LOG_LEVEL_INFO

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);


#define BK_RAM_BASE 0x4902C000U
#define BK_RAM_SIZE 1000U
#define POWER_MODE_OFFSET 0
#define HP_START_OFFSET 1

uint32_t power_mode_input=0;
uint32_t hp_start_flag=0;

static int32_t bk_ram_wr(uint32_t *data, uint32_t offset)
{
    if (offset < BK_RAM_SIZE) {
        sys_write32(*data, BK_RAM_BASE + (offset * sizeof(uint32_t)));
        return 0;
    }
    return -1;
}

static void backup_ram_write(uint32_t power_mode)
{
     while(1)
     {
          if( (bk_ram_wr(&power_mode,POWER_MODE_OFFSET)!= -1))
          {
             break;
          }
     }

     while(1)
     {
          hp_start_flag=1;
          if((bk_ram_wr(&hp_start_flag,HP_START_OFFSET))!= -1)
          break;
     }
     
}

#if defined(CONFIG_PM)


static int cmd_standby_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start Standby mode \n");
	
        int ret = app_set_standby_params();
        app_pm_lock_deeper_states(true);
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Standby state set \n");

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
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
		return ret;
	}

	return ret;
}


static int cmd_ready1_systop_off_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start Ready 1 mode with SYSTOP off\n");
	
        int ret = app_set_ready1_systop_off_params();
        app_pm_lock_deeper_states(true);
         k_sleep(K_MSEC(1000));
	pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
	k_sleep(K_FOREVER);

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
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
		return ret;
	}

	return ret;
}
static int cmd_stop1_test(const struct shell *shell, size_t argc, char **argv)
{

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start STOP 1 mode \n");
        int ret = app_set_stop1_params();
        app_pm_lock_deeper_states(false);
        k_sleep(K_USEC(22000000));

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
        k_sleep(K_USEC(22000000));

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
        k_sleep(K_USEC(22000000));

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
        k_sleep(K_USEC(22000000));

	if (ret) {
		shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "ERROR: %d\n", ret);
		return ret;
	}

	return ret;
}

static int cmd_stop5_test(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t regdata1=12, regdata2=1;

	shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "Start STOP 5 mode \n");
	//backup_ram_write(12);
	// sys_write32(regdata1, (BK_RAM_BASE + 0x0));
	// sys_write32(regdata2, (BK_RAM_BASE +0x10));
        int ret = app_set_stop5_params();
        app_pm_lock_deeper_states(false);
        k_sleep(K_USEC(27000000));


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
        
        #if !defined(CONFIG_START_WITH_HFRC) || !defined(CONFIG_START_WITH_DIVIDED_HFRC)
	SHELL_CMD_ARG(pm_go1, NULL, "Start GO1-state ", cmd_go1_test, 1,
		      10),
	SHELL_CMD_ARG(pm_go3, NULL, "Start GO3-state ", cmd_go3_test, 1,
		      10),
	SHELL_CMD_ARG(pm_ready1_with_systop, NULL, "Start ready with systop on-state ", cmd_ready1_systop_test, 1,
		      10),
	SHELL_CMD_ARG(pm_ready1_with_off_systop, NULL, "Start ready with systop off-state ", cmd_ready1_systop_off_test, 1,
		      10),
	SHELL_CMD_ARG(pm_standby, NULL, "Start standby-state ", cmd_standby_test, 1,
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
	//Keeping the infrastructure for shell commands. Omce HFRC support for UART is available, these lines can be uncommented
	#elif defined(CONFIG_START_WITH_HFRC)	
	//SHELL_CMD_ARG(pm_go4, NULL, "Start GO4-state ", cmd_go4_test, 1,
		      10),	
	//SHELL_CMD_ARG(pm_ready2, NULL, "Start Ready2-state ", cmd_ready2_test, 1,
		      10),
        #endif


#endif

	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pwr, &sub_cmds, "Power management test commands", NULL);
