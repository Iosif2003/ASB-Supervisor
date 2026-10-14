/* =========================================================
 * ASB System Control - Implementation
 * Aristurtle Formula Student | 2026
 *
 * SDC relay, hardware watchdog, digital inputs, mission selection and
 * the manual driving checks. See asb_system.h for the pin meaning.
 * ========================================================= */

#include "asb_system.h"
#include "main.h"
#include "can_mcu.h"
#include "asb_can.h"
#include "asb_ebs.h"
#include "asb_monitoring.h"
#include "asb_config.h"
#include "asb_sensors.h"

/* Private Variables */
static bool sdc_is_closed     = false;   /* what we last commanded */
static bool wdg_is_running    = false;   /* note: never set to true anywhere at the moment */
static bool manual_checked = false;      /* manual mode: startup check passed */

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

    /* start with the SDC open */
    HAL_GPIO_WritePin(AS_Relay_Signal_GPIO_Port, AS_Relay_Signal_Pin, GPIO_PIN_RESET);
}

/* SHUTDOWN CONTROL */
/* AS_Relay_Signal (PB13, open drain with external pull-up):
   high = we close our relay in the SDC, low = we open it.
   Whether the relay really closed is read back on AS_Relay_Out. */

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
/* TIM3 runs at 1MHz (1us per count) in one pulse mode with a 40ms period,
   CH4 (PC9) is high for the first 30us of each run. Restarting it every
   10ms from the main loop gives a 30us pulse every 10ms. If the main
   loop hangs for more than 40ms the timer stops and the pulses stop. */

/* Watchdog Reset - restart the one pulse run from 0 */
void WDG_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE(&htim3);

}



/* GPIO */

/* GPIO Input Reads */

/* snapshot of all inputs, refreshed once per 10ms tick by Inputs_Sample() */
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

/* autonomous = VCU in driverless mode AND the APU has a driverless mission (AMI 1-6)
   manual     = VCU in manual mode OR AMI 7
   anything else (e.g. nothing received yet) = none */
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

static Check_t manual_run_ok = { .limit = ASB_MON_MANUAL_LIMIT };   /* not used at the moment */
/* Run - called from main loop every 10ms via timer flag.
   In manual driving the ASB only closes the SDC when the EBS can't brake
   (ASMS off, both tanks vented), and opens it again if that changes. */
void Manual_Run(void)
{
    if (!manual_checked)
    {
        /* ASMS must be off and the pneumatic system depressurized
            no autonomous brake actuation possible */
        if (SYS_GetASMS() || Sensors_TankPressure1Valid() || Sensors_TankPressure2Valid())
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
        /* note: no debounce here right now, manual_run_ok is not used,
           a single bad sample opens the SDC */
        if (Sensors_TankPressure1Valid() || Sensors_TankPressure2Valid() || SYS_GetSelectedMission() != MISSION_MANUAL)
        {
            SDC_Open();
            manual_checked = false;
        }
    }
}
