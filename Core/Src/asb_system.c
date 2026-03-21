/* =========================================================
 * ASB System Control — Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_system.h"
#include "main.h"

/* Private Variables */
static bool sdc_is_closed     = false;
static bool wdg_opm_enabled   = false;
static bool wdg_is_running    = false;

/* External handles from main.c */
extern TIM_HandleTypeDef htim3;

/* System Initialization */
void ASB_System_Init(void) {
    sdc_is_closed = false;
    wdg_opm_enabled = false;
    wdg_is_running = false;

    HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);
 }

/* SHUTDOWN CONTROL */

/* SDC Close */
void SDC_Close(void)
{
    sdc_is_closed = true;
    HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_SET);
}

/* SDC Open */
void SDC_Open(void)
{
    sdc_is_closed = false;
    HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);
}       

/* WATCHDOG CONTROL */

/* WDG OPM: timer produces single pulse then stops, code must reset it every 10ms via WDG_Reset(),
 if code freezes, no reset occurs, pulse stops, external watchdog triggers EBS */

/* Watchdog Reset */
void WDG_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/* Watchdog Stop */
void WDG_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    wdg_is_running = false;
}

/* Watchdog Start */
void WDG_Start(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    wdg_is_running = true;
}

/* Watchdog Enable OPM */
 void WDG_EnableOPMode(void)
{
    WDG_Stop();
    htim3.Instance->CR1 |= TIM_CR1_OPM;
    WDG_Reset();
    WDG_Start();

    wdg_opm_enabled = true;
}

/* Watchdog Disable OPM */
void WDG_DisableOPMode(void)
{
    WDG_Stop();
    htim3.Instance->CR1 &= ~TIM_CR1_OPM;
    WDG_Reset();
    WDG_Start();

    wdg_opm_enabled = false;
    
}
/* GPIO */

/* GPIO Input Reads */

bool SYS_GetASMS(void)
{
    return HAL_GPIO_ReadPin(ASMS_Out_GPIO_Port, ASMS_Out_Pin);
}

bool SYS_GetTSMS(void)
{
    return HAL_GPIO_ReadPin(TSMS_Out_NOT_GPIO_Port, TSMS_Out_NOT_Pin);
}

bool SYS_GetASRelayIn(void)
{
    return HAL_GPIO_ReadPin(ASRelay_In_GPIO_Port, ASRelay_In_Pin);
}

bool SYS_GetASRelayOut(void)
{
    return HAL_GPIO_ReadPin(ASRelay_Out_GPIO_Port, ASRelay_Out_Pin);
}

bool SYS_GetInterlock1(void)
{
    return HAL_GPIO_ReadPin(Interlock_Valve1_GPIO_Port, Interlock_Valve1_Pin);
}

bool SYS_GetInterlock2(void)
{
    return HAL_GPIO_ReadPin(Interlock_valve2_GPIO_Port, Interlock_valve2_Pin);
}

bool SYS_GetInterlockService(void)
{
    return HAL_GPIO_ReadPin(Interlock_PV_GPIO_Port, Interlock_PV_Pin);
}

bool SYS_GetInterlockSteering(void)
{
    return HAL_GPIO_ReadPin(Interlock_Steering_GPIO_Port, Interlock_Steering_Pin);
}

/* States */

bool SDC_IsClosed(void)     { return sdc_is_closed;   }
bool WDG_IsOPMEnabled(void) { return wdg_opm_enabled; }
bool WDG_IsRunning(void)    { return wdg_is_running;  }