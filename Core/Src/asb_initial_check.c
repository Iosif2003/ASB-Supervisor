/* =========================================================
 * ASB Initial Check — Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */
#include "main.h"
#include "asb_initial_check.h"
#include "asb_config.h"
#include "asb_ebs.h"
#include "asb_system.h"
#include "asb_sensors.h"
#include "asb_service_brake.h"
#include "asb_can.h"


/* Private Variables */
static IC_State_t ic_state = IC_WAIT_MISSION;
static uint32_t   ic_timer = 0U;

/* IC Initialization */
void IC_Init(void)
{
    ic_state = IC_WAIT_MISSION;
    ic_timer = 0U;
}

/* IC Fail */
static void IC_Fail(void)
{
    ic_state = IC_FAILED;
    EBS_Activate();
    SDC_Open();
}

/* IC Safety Check  */
static void IC_SafetyCheck(void)
{
    if(SYS_GetASMS() != 1)                             { IC_Fail(); }
    if(SYS_GetSelectedMission() != MISSION_AUTONOMOUS) { IC_Fail(); }

    if(ic_state >= IC_WAIT_TSMS && ic_state < IC_COMPLETE)
    {
        //TSMS_Out_NOT is after AS_Relay_Out 
        //on vehicle SDC so checking that includes AS_Relay_Out and therefore in as well
        if(SYS_GetTSMS() != 0)       { IC_Fail(); }
    }
    else if(ic_state >= IC_WAIT_SDC_CLOSE_FIRST_TIME)
    {
        //From now on we check AS_Relay_Out only inste of IN=0 -> OUT=0 as well so it is included ?????
        if(SYS_GetASRelayOut() != 1) { IC_Fail(); }
    }
    else if(ic_state > IC_WAIT_RES)
    {
        //AS_relay_in is RES1_Out so RES is Open	
        if(SYS_GetASRelayIn() != 1)  { IC_Fail(); }
    }
}

 static bool Notify_APU(void){
    CAN_SetInitialChecked(true);
    return (CAN_GetASState() == 2U);
}

static bool IC_TimerElapsed(uint32_t duration_ms)
{
    if (ic_timer == 0U) { ic_timer = HAL_GetTick(); }
    if (HAL_GetTick() - ic_timer >= duration_ms)
    {
        ic_timer = 0U;
        return true;
    }
    return false;
}

/* States */
IC_State_t IC_GetState(void)   { return ic_state;                }
bool       IC_IsComplete(void) { return ic_state == IC_COMPLETE; }

void IC_Run(void)
{
    if (ic_state > IC_WAIT_ASMS && ic_state < IC_COMPLETE)  { IC_SafetyCheck(); }
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
            if (SYS_GetASRelayIn()) { ic_state = IC_WAIT_PRESSURES; }
            break;

        case IC_WAIT_PRESSURES:
            /* Tank pressure valid AND brake pressure confirms engaged */
            if (Sensors_TankPressureValid() && Sensors_BrakePressureEngaged()) { ic_state = IC_SET_DIGPIN_HIGH;}
            break;
          
        case IC_SET_DIGPIN_HIGH: // check sdc deeper in the next version
            SDC_Close();
            ic_state = IC_WAIT_SDC_CLOSE_FIRST_TIME;
            break;

        case IC_WAIT_SDC_CLOSE_FIRST_TIME:
            if (SYS_GetASRelayOut()) { ic_state = IC_STOP_WATCHDOG; }
            break;

        case IC_STOP_WATCHDOG:
            WDG_Stop();
            ic_state = IC_WAIT_SDC_OPEN_SECOND_TIME;
            break;

        case IC_WAIT_SDC_OPEN_SECOND_TIME:
            if (!SYS_GetASRelayOut()) { ic_state = IC_START_WATCHDOG; }
            break;

        case IC_START_WATCHDOG:
            WDG_Start();
            ic_state = IC_WAIT_SDC_CLOSE_SECOND_TIME;
            break;

        case IC_WAIT_SDC_CLOSE_SECOND_TIME:
            if (SYS_GetASRelayOut()) { ic_state = IC_WAIT_TSMS; }
            break;

        case IC_WAIT_TSMS: // need fix in the next version 
            if (!SYS_GetTSMS()) { ic_state = IC_RELEASE_EBS; }
            break;

        case IC_RELEASE_EBS:
            if (EBS_IsActivated()) { EBS_Release_All(); }
            if (IC_TimerElapsed(ASB_IC_VALVE_SETTLE_MS) && Sensors_BrakePressureReleased())
            {
                ic_state = IC_ENGAGE_SYSTEM1;
            }
            break;

        case IC_ENGAGE_SYSTEM1:
            if (!EBS_System1_State()) { EBS_Activate_System1(); }
            if (IC_TimerElapsed(ASB_IC_VALVE_SETTLE_MS) && Sensors_BrakePressureEngaged())
            {
                ic_state = IC_RELEASE_SYSTEM1;
            }
            break;   
            
        case IC_RELEASE_SYSTEM1:
            if (EBS_System1_State()) { EBS_Release_System1(); }
            if (IC_TimerElapsed(ASB_IC_VALVE_SETTLE_MS) && Sensors_BrakePressureReleased())
            {
                ic_state = IC_ENGAGE_SYSTEM2;
            }
            break;

        case IC_ENGAGE_SYSTEM2:
            if (!EBS_System2_State()) { EBS_Activate_System2(); }
            if (IC_TimerElapsed(ASB_IC_VALVE_SETTLE_MS) && Sensors_BrakePressureEngaged())
            {
                ic_state = IC_RELEASE_SYSTEM2;
            }
            break;  
        
        case IC_RELEASE_SYSTEM2:
            if (EBS_System2_State()) { EBS_Release_System2(); }
            if (IC_TimerElapsed(ASB_IC_VALVE_SETTLE_MS) && Sensors_BrakePressureReleased())
            { 
                ic_state = IC_ENGAGE_SERVICE_BRAKE;
            }
            break;  
        
        case IC_ENGAGE_SERVICE_BRAKE:
            if(ServiceBrake_State() != SERVICE_BRAKE_ENGAGED) { ServiceBrake_Engage(); }
            if (IC_TimerElapsed(ASB_IC_VALVE_SETTLE_MS) && Sensors_BrakePressureEngaged())
            {
                ic_state = IC_RELEASE_SERVICE_BRAKE;
            }
            break;

        case IC_RELEASE_SERVICE_BRAKE:
            if(ServiceBrake_State() != SERVICE_BRAKE_DISENGAGED) { ServiceBrake_Disengage(); }
            if (IC_TimerElapsed(ASB_IC_VALVE_SETTLE_MS) && Sensors_BrakePressureReleased())
            {
                ic_state = IC_PARK_SERVICE_BRAKE;
            }
            break;
    
        case IC_PARK_SERVICE_BRAKE:
            ServiceBrake_Park();
            ic_state = IC_NOTIFY_APU;
            break;      

        case IC_NOTIFY_APU:
            if (Notify_APU())    { ic_state = IC_ENABLE_OPM; }
            else if (IC_TimerElapsed(ASB_IC_APU_READY_TIMEOUT_MS))    { IC_Fail(); }
            break;
        case IC_ENABLE_OPM:
            WDG_EnableOPMode();
            ic_state = IC_COMPLETE;
            break;
        }
}