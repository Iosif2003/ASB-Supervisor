/* =========================================================
 * ASB System Control - Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_system.h"
#include "main.h"
#include "can_mcu.h"
#include "asb_can.h"
#include "asb_ebs.h"
#include "asb_monitoring.h"
#include "asb_config.h"

/* Private Variables */
static bool sdc_is_closed     = false;
static bool wdg_is_running    = false;
static bool manual_checked = false;

/* External handles from main.c */
extern TIM_HandleTypeDef htim3;


/* System Initialization */
void ASB_System_Init(void) {
    sdc_is_closed = false;
    wdg_is_running = false;
    manual_checked = false;

    /* Freeze TIM3 while the core is halted by the debugger - otherwise a
       breakpoint held >40ms fires the OPM update event and the watchdog
       pulse dies. No effect when no debugger is attached. */
    __HAL_DBGMCU_FREEZE_TIM3();

    HAL_GPIO_WritePin(AS_Relay_Signal_GPIO_Port, AS_Relay_Signal_Pin, GPIO_PIN_RESET);
}

/* SHUTDOWN CONTROL */

/* SDC Close */
void SDC_Close(void)
{
    sdc_is_closed = true;
    HAL_GPIO_WritePin(AS_Relay_Signal_GPIO_Port, AS_Relay_Signal_Pin, GPIO_PIN_SET);
}

/* SDC Open */
void SDC_Open(void)
{
    sdc_is_closed = false;
    HAL_GPIO_WritePin(AS_Relay_Signal_GPIO_Port, AS_Relay_Signal_Pin, GPIO_PIN_RESET);
}       

/* WATCHDOG CONTROL */

/* WDG OPM: timer produces single pulse then stops, code must reset it every 10ms via WDG_Reset(),
 if code freezes, no reset occurs, pulse stops, external watchdog triggers EBS */

/* Watchdog Reset */
void WDG_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    /* OPM trap: one missed 40ms deadline (e.g. debugger halt) fires the
       update event and hardware clears CEN - a counter write alone can
       never restart the pulse train. Being called here proves the code is
       alive, so re-arm. Guarded: during WDG_Stop() the timer must stay off. */
    if (wdg_is_running && ((htim3.Instance->CR1 & TIM_CR1_CEN) == 0U))
    {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE(&htim3);
    }
}

/* Watchdog Stop */
void WDG_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

    /* With the channel off the open-drain pin is released and the external
       pull-up holds the line steadily HIGH - the watchdog circuit can read
       that as "alive" and never drop the AS relay. Take the pin over as a
       plain GPIO and drive it LOW so the line is unambiguously dead. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = WatchdogPWM_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WatchdogPWM_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(WatchdogPWM_GPIO_Port, WatchdogPWM_Pin, GPIO_PIN_RESET);

    wdg_is_running = false;
}

/* Watchdog Start */
void WDG_Start(void)
{
    /* Hand the pin back to TIM3 CH4 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = WatchdogPWM_Pin;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(WatchdogPWM_GPIO_Port, &gpio);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);   /* stale UIF from a missed deadline */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    wdg_is_running = true;
}

/* GPIO */

/* GPIO Input Reads */

static struct {
    bool asms, tsms, AS_Relay_In, AS_Relay_Out;
    bool interlock1, interlock2, interlock_service, interlock_steering;
} sys_in;

void Inputs_Sample(void)
{
    sys_in.asms               = HAL_GPIO_ReadPin(ASMS_GPIO_Port, ASMS_Pin);
    sys_in.tsms               = HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin);  /* raw pin: 1 = TSMS closed (SDC live), 0 = open */
    sys_in.AS_Relay_In        = HAL_GPIO_ReadPin(AS_Relay_In_GPIO_Port, AS_Relay_In_Pin);
    sys_in.AS_Relay_Out       = HAL_GPIO_ReadPin(AS_Relay_Out_GPIO_Port, AS_Relay_Out_Pin);
    sys_in.interlock1         = HAL_GPIO_ReadPin(Interlock_Valve1_GPIO_Port, Interlock_Valve1_Pin);
    sys_in.interlock2         = HAL_GPIO_ReadPin(Interlock_Valve2_GPIO_Port, Interlock_Valve2_Pin);
    sys_in.interlock_service  = HAL_GPIO_ReadPin(Interlock_Proportional_GPIO_Port, Interlock_Proportional_Pin);
    sys_in.interlock_steering = HAL_GPIO_ReadPin(Interlock_Steering_GPIO_Port, Interlock_Steering_Pin);
}

bool SYS_GetASMS(void)              { return sys_in.asms;               }
bool SYS_GetTSMS(void)              { return sys_in.tsms;               }
bool SYS_GetASRelayIn(void)         { return sys_in.AS_Relay_In;        }
bool SYS_GetASRelayOut(void)        { return sys_in.AS_Relay_Out;       }
bool SYS_GetInterlock1(void)        { return sys_in.interlock1;         }
bool SYS_GetInterlock2(void)        { return sys_in.interlock2;         }
bool SYS_GetInterlockService(void)  { return sys_in.interlock_service;  }
bool SYS_GetInterlockSteering(void) { return sys_in.interlock_steering; }

/* States */

bool SDC_IsClosed(void)     { return sdc_is_closed;   }
bool WDG_IsRunning(void)    { return wdg_is_running;  }

/* Mission Selection */

int SYS_GetSelectedMission(void)
{
    if((CAN_GetVCUMode() == 0) &&
       (CAN_GetASMission() >= 1) &&
       (CAN_GetASMission() <= 6))
        return MISSION_AUTONOMOUS;
    else if((CAN_GetVCUMode() == 1) ||
            (CAN_GetASMission() == 7))
        return MISSION_MANUAL;
    else
        return MISSION_NONE;
}

/* Manual Checks - Monitoring */

static Check_t manual_run_ok = { .limit = ASB_MON_MANUAL_LIMIT };   
/* Run - called from main loop every 10ms via timer flag */
void Manual_Run(void)
{
    if (!manual_checked)
    {
        /* ASMS must be off and the pneumatic system depressurized
           (EBS unavailable) - no autonomous brake actuation possible */
        if (SYS_GetASMS() /*|| EBS_State() != EBS_UNAVAILABLE*/)
        {
            SDC_Open();
            return;
        }

        /* Command SDC close */
        SDC_Close();

        /* Verify SDC is closed (TSMS high = closed) */
        if (!SYS_GetTSMS())
            return;


        manual_checked = true;
    }
    else
    {
        /* Monitoring - Check() true = healthy; persistent violation (2s)
           or mission change -> SDC open */
        if (!Check(&manual_run_ok, (!SYS_GetASMS() /*&& EBS_State() == EBS_UNAVAILABLE*/))
        || SYS_GetSelectedMission() != MISSION_MANUAL)
        {
            SDC_Open();
            manual_checked = false;
        }
    }
}
