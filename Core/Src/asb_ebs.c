/* =========================================================
 * ASB EBS Control - Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_ebs.h"
#include "asb_sensors.h"
#include "asb_system.h"
#include "asb_initial_check.h"
#include "main.h"
#include <stdbool.h>

/* Private Variables */
static bool ebs_is_activated  = true;
static bool ebs_system1_state = true;
static bool ebs_system2_state = true;

static EBS_State_t EBS_state = EBS_UNAVAILABLE;

/* EBS States */
bool EBS_IsActivated(void)   { return ebs_is_activated;  }
bool EBS_System1_State(void) { return ebs_system1_state; }
bool EBS_System2_State(void) { return ebs_system2_state; }

/* GET EBS STATE */
EBS_State_t EBS_State(void) 
{
    if(!Sensors_TankPressureValid())            { EBS_state = EBS_UNAVAILABLE; }

    else if(ebs_is_activated || !SYS_GetTSMS()) { EBS_state = EBS_TRIGGERED; }  /* activated or SDC open */
    
    else { EBS_state = EBS_ARMED; }

    return EBS_state;
}

/* DV System Status mapping (FS Driverless Specification 2026 v1.1, Table 2).
 * Priority order matters:
 * - before the initial check passes the spec state is "deactivated" (T 14.8.2
 *   startup condition), even though the valves physically hold the car.
 * - manual release (ASMS off + vented circuits) beats the activation latch,
 *   so after the brake-release points are pulled we report "deactivated".
 * - activation (T 14.8.1: supply cut after the check) beats tank-pressure
 *   loss, so a leak-triggered EBS stop still reports "activated". */
DV_EbsStatus_t EBS_DvStatus(void)
{
    if (IC_GetState() < IC_NOTIFY_APU)                  { return DV_EBS_DEACTIVATED; }
    if (!SYS_GetASMS() && !Sensors_TankPressureValid()) { return DV_EBS_DEACTIVATED; }   /* manual release (T 14.8.2) */
    if (EBS_IsActivated() || !SYS_GetTSMS())            { return DV_EBS_ACTIVATED; }     /* T 14.8.1 */
    if (!Sensors_TankPressureValid())                   { return DV_EBS_DEACTIVATED; }   /* loss before the monitor trips */
    return DV_EBS_INITIAL_CHECKUP_PASSED;
}

DV_REbsStatus_t EBS_DvRedundancyStatus(void)
{
    if (IC_GetState() < IC_NOTIFY_APU)                  { return DV_R_EBS_DEACTIVATED; }
    if (!SYS_GetASMS() && !Sensors_TankPressureValid()) { return DV_R_EBS_DEACTIVATED; } /* manual release (T 14.8.2) */
    if (EBS_IsActivated() || !SYS_GetTSMS())            { return DV_R_EBS_ENGAGED; }     /* T 14.8.1 */
    if (!Sensors_TankPressureValid())                   { return DV_R_EBS_DEACTIVATED; } /* loss before the monitor trips */
    return DV_R_EBS_INITIAL_CHECKUP_PASSED;
}


/* EBS Initialization */
void EBS_Init(void)
{
    EBS_state = EBS_UNAVAILABLE;
    ebs_is_activated  = true;
    ebs_system1_state = true;
    ebs_system2_state = true;

    HAL_GPIO_WritePin(EBS_Valve1_GND_GPIO_Port, EBS_Valve1_GND_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EBS_Valve2_GND_GPIO_Port, EBS_Valve2_GND_Pin, GPIO_PIN_RESET);
}

/* EBS Activation */
void EBS_Activate(void)
{
    EBS_state = EBS_TRIGGERED;
    ebs_is_activated  = true;
    ebs_system1_state = true;
    ebs_system2_state = true;

    HAL_GPIO_WritePin(EBS_Valve1_GND_GPIO_Port, EBS_Valve1_GND_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EBS_Valve2_GND_GPIO_Port, EBS_Valve2_GND_Pin, GPIO_PIN_RESET);
}

/* EBS Release All */
void EBS_Release_All(void)
{
    ebs_system1_state = false;
    ebs_system2_state = false;
    ebs_is_activated  = false;

    HAL_GPIO_WritePin(EBS_Valve1_GND_GPIO_Port, EBS_Valve1_GND_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EBS_Valve2_GND_GPIO_Port, EBS_Valve2_GND_Pin, GPIO_PIN_SET);
}

/* EBS Release System 1 */
void EBS_Release_System1(void)
{
    ebs_system1_state = false;
    if(!ebs_system2_state)
        ebs_is_activated = false;
    HAL_GPIO_WritePin(EBS_Valve1_GND_GPIO_Port, EBS_Valve1_GND_Pin, GPIO_PIN_SET);
}

/* EBS Release System 2 */
void EBS_Release_System2(void)
{
    ebs_system2_state = false;
    if(!ebs_system1_state)
        ebs_is_activated = false;

    HAL_GPIO_WritePin(EBS_Valve2_GND_GPIO_Port, EBS_Valve2_GND_Pin, GPIO_PIN_SET);
}

/* EBS Activate System 1 */
void EBS_Activate_System1(void)
{
    ebs_system1_state = true;
    ebs_is_activated = true;

    HAL_GPIO_WritePin(EBS_Valve1_GND_GPIO_Port, EBS_Valve1_GND_Pin, GPIO_PIN_RESET);
}

/* EBS Activate System 2 */
void EBS_Activate_System2(void)
{
    ebs_system2_state = true;
    ebs_is_activated = true;

    HAL_GPIO_WritePin(EBS_Valve2_GND_GPIO_Port, EBS_Valve2_GND_Pin, GPIO_PIN_RESET);
}
