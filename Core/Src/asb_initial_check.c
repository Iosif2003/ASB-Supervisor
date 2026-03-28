/* =========================================================
 * ASB Initial Check — Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_initial_check.h"
#include "asb_config.h"
#include "asb_ebs.h"
#include "asb_system.h"
#include "asb_sensors.h"
#include "asb_service_brake.h"
#include "can_mcu.h"

/* Private Variables */
static IC_State_t ic_state = IC_WAIT_MISSION;
static uint32_t   ic_timer = 0U;

/* IC Initialization */
void IC_Init(void)
{
    ic_state = IC_WAIT_MISSION;
    ic_timer = 0U;
}

/* IC Safety Check  */
static bool IC_SafetyCheck(void)
{
    if(SYS_GetASMS() != 1)                             { return false; }
    if(SYS_GetSelectedMission() != MISSION_AUTONOMOUS) { return false; }

    if(ic_state >= IC_WAIT_TSMS && ic_state < IC_WAIT_AS_READY)
    {
        if(SYS_GetTSMS() != 0)       { return false; }
    }
    else if(ic_state >= IC_WAIT_SDC_CLOSE)
    {
        if(SYS_GetASRelayOut() != 1) { return false; }
    }
    else if(ic_state > IC_WAIT_RES)
    {
        if(SYS_GetASRelayIn() != 1)  { return false; }
    }

    return true;
}

/* IC Fail */
static void IC_Fail(void)
{
    ic_state = IC_FAILED;
    EBS_Activate();
    SDC_Open();
}

/* States */
IC_State_t IC_GetState(void)   { return ic_state;                }
bool       IC_IsComplete(void) { return ic_state == IC_COMPLETE; }

void IC_Run(void)
{
    switch(ic_state)
    {

        case IC_WAIT_ASMS:
        if (SYS_GetASMS())  { ic_state = IC_ACTIVATE_EBS; }
        break;
    
        case IC_ACTIVATE_EBS:
        EBS_Activate();
        ic_state = IC_WAIT_RES;
        break;

         case IC_WAIT_RES:
        /* ASRelay_In == 1 means SDC before ASB is closed (RES closed) */
        if (SYS_GetASRelayIn()) { ic_state = IC_WAIT_PRESSURE; }
        break;

        case IC_WAIT_PRESSURE:
        /* Tank pressure valid AND brake pressure confirms engaged */
        if (Sensors_TankPressureValid() && Sensors_BrakePressureEngaged()) {
            ic_state = IC_CLOSE_SDC;
        }
        break;
    