/* =========================================================
 * ASB System Control — Implementation
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

/* GPIO */

/* GPIO Input Reads */

static struct {
    bool asms, tsms, AS_Relay_In, AS_Relay_Out;
    bool interlock1, interlock2, interlock_service, interlock_steering;
} sys_in;

void Inputs_Sample(void)
{
    sys_in.asms               = HAL_GPIO_ReadPin(ASMS_GPIO_Port, ASMS_Pin);
    sys_in.tsms               = !HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin);  /* pin reads true TSMS (new HW); invert so GetTSMS()==0 means SDC closed, matching old TSMS_Out_NOT convention */
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
/* Run — called from main loop every 10ms via timer flag */
void Manual_Run(void)
{
    if (!manual_checked)
    {
        /* ASMS must be off and the pneumatic system depressurized
           (EBS unavailable) — no autonomous brake actuation possible */
        if (SYS_GetASMS() || EBS_State() != EBS_UNAVAILABLE)
        {
            SDC_Open();
            return;
        }

        /* Command SDC close */
        SDC_Close();

        /* Verify SDC is closed */
        if (SYS_GetTSMS())
            return;

        manual_checked = true;
    }
    else
    {
        /* Monitoring — Check() true = healthy; persistent violation (2s)
           or mission change → SDC open */
        if (!Check(&manual_run_ok, (!SYS_GetASMS() && EBS_State() == EBS_UNAVAILABLE))
        || SYS_GetSelectedMission() != MISSION_MANUAL)
        {
            SDC_Open();
            manual_checked = false;
        }
    }
}
