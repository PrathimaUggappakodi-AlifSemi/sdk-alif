typedef enum _POWER_MODE_TYPE {
    GOMODE1=1,
    GOMODE3,
    GOMODE4,
    READYMODE1_WITH_SYSTOP,
    READYMODE1_WITHOUT_SYSTOP,
    READY2,
    STANDBYMODE,
    STOPMODE1,
    STOPMODE2,
    STOPMODE3,
    STOPMODE4,
    STOPMODE5
}POWER_MODE_TYPE;

int app_set_go3_params(void);
int app_set_go1_params(void);
int app_set_go4_params(void);
int app_set_ready2_params(void);
int app_set_stop1_params(void);
int app_set_stop2_params(void);
int app_set_stop3_params(void);
int app_set_stop4_params(void);
int app_set_stop5_params(void);
int app_set_standby_params(void);
int app_set_ready1_systop_on_params(void);
int app_set_ready1_systop_off_params(void);
void app_pm_lock_deeper_states(bool lock);
void app_pm_unlock_deeper_states(uint32_t period_ms);
