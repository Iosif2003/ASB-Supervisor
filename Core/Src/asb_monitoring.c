/* =========================================================
 * ASB Monitor - Implementation
 * Aristurtle Formula Student | 2026
 *
 * Every Monitor_Run() call takes one sample of each condition and feeds
 * it to its Check_t counter (see Check() in asb_monitoring.h). If any
 * check has been bad for longer than its limit, Monitor_Run() returns
 * false and main.c brakes + opens the SDC. Limits are in asb_config.h.
 * ========================================================= */
#include "asb_monitoring.h"
#include "asb_config.h"
#include "asb_system.h"
#include "asb_sensors.h"
#include "asb_can.h"
#include "asb_service_brake.h"

/* one counter per check. getter starts true so nothing reads as failed
   before the first Monitor_Run() */
static Check_t interlock_valve1         = { .limit = ASB_MON_INTERLOCK_LIMIT ,.getter = true};
static Check_t interlock_valve2         = { .limit = ASB_MON_INTERLOCK_LIMIT ,.getter = true};
static Check_t interlock_proportional   = { .limit = ASB_MON_INTERLOCK_LIMIT ,.getter = true};
static Check_t interlock_steering       = { .limit = ASB_MON_INTERLOCK_LIMIT ,.getter = true};
static Check_t TankPressure_OK          = { .limit = ASB_MON_TANK_LIMIT ,.getter = true};
static Check_t BrakePressure_OK         = { .limit = ASB_MON_BRAKE_LIMIT ,.getter = true};
static Check_t apu_OK                   = { .limit = ASB_MON_APU_LIMIT ,.getter = true};
static Check_t ServiceBrake_OK          = { .limit = ASB_MON_SERVICE_BRAKE_LIMIT ,.getter = true};
static Check_t Watchdog_OK              = { .limit = ASB_MON_WATCHDOG_LIMIT ,.getter = true};
static Check_t res_OK					= { .limit = ASB_MON_APU_LIMIT ,.getter = true};   /* same limit as the APU */


/* helper conditions - true = healthy */

/* brake pressure is trusted only if both CAN sources are alive, the value
   is in the sensor range and the SMU doesn't flag it out of range */
static bool brake_Pressure_ok(void)
{
    float p = Sensors_GetBrakePressure();
    return CAN_IsBrakePressureAlive() && (p >= 0.0f) && (p < ASB_BRAKE_MAX_BAR) && !CAN_IsBrakeOutOfRange();  /* Sheet 8: 0..350 */
}

/* Cont.Mon Sheet 5: engaged (24 bar) / park (10 bar) must actually hold
   pressure - fail below 5 bar. Available is only checked while commanded to
   actuate (no command signal in the DBC yet, so that branch stays passive).
   Disengaged (0 bar) has nothing to prove. The interlock leg of Sheet 5 is
   the separate interlock_proportional check. Actuation ramp-up is covered
   by the 1s debounce (ASB_MON_SERVICE_BRAKE_LIMIT). */
static bool service_brake_ok(void)
{
    ServiceBrakeState_t s = ServiceBrake_State();
    if (s == SERVICE_BRAKE_ENGAGED || s == SERVICE_BRAKE_PARK)
        return Sensors_GetBrakePressure() > ASB_BRAKE_ACTIVE_MIN_BAR;
    if (s == SERVICE_BRAKE_AVAILABLE && CAN_GetServoCommand() != 0)
        return Sensors_GetBrakePressure() > ASB_BRAKE_ACTIVE_MIN_BAR;
    return true;   /* disengaged, or available with no command -> ok */
}

/* The watchdog opens the AS relay by itself when the pulses stop.
   So: we command the SDC closed, the SDC before us is closed (relay in = 1),
   but the relay output reads open -> the watchdog tripped.
   During the IC watchdog test this happens on purpose, so the open time
   there has to stay below ASB_MON_WATCHDOG_LIMIT or this check trips. */
static bool watchdog_ok(void)
{
    /* SDC closed & RES closed (in=1) but relay opened (out=0) -> watchdog tripped it */
    return !(SDC_IsClosed() && SYS_GetASRelayIn() && !SYS_GetASRelayOut());
}

/* API */
void Monitor_Init(void)
{
    interlock_valve1.errors = interlock_valve2.errors = interlock_proportional.errors = interlock_steering.errors = 0;
    TankPressure_OK.errors = BrakePressure_OK.errors = apu_OK.errors = 0;
    ServiceBrake_OK.errors = Watchdog_OK.errors = res_OK.errors = 0;

}

/* one sample of every check, true only if ALL of them are ok.
   All checks run on every call (no early return), so every counter
   keeps counting even when another check already failed. */
bool Monitor_Run(void)
{   interlock_valve1.getter       = Check(&interlock_valve1,  SYS_GetInterlock1());
    interlock_valve2.getter       = Check(&interlock_valve2,  SYS_GetInterlock2());
    interlock_proportional.getter = Check(&interlock_proportional, SYS_GetInterlockService());
    interlock_steering.getter     = Check(&interlock_steering, SYS_GetInterlockSteering());
    TankPressure_OK.getter        = Check(&TankPressure_OK,  (Sensors_TankPressure1Valid() && Sensors_TankPressure2Valid()));
    BrakePressure_OK.getter       = Check(&BrakePressure_OK, brake_Pressure_ok());
    apu_OK.getter                 = Check(&apu_OK,   CAN_IsAPUAlive());
    Watchdog_OK.getter            = Check(&Watchdog_OK,   watchdog_ok());
    /* RES: alive with good signal AND never reported an internal error */
    res_OK.getter 			      = Check(&res_OK, CAN_IsRESAlive() && (CAN_GetRESErrorRegister() == 0U));
    ServiceBrake_OK.getter        = Check(&ServiceBrake_OK, service_brake_ok());
    return TankPressure_OK.getter && BrakePressure_OK.getter && interlock_valve1.getter && interlock_valve2.getter
    		&& interlock_proportional.getter && apu_OK.getter  && res_OK.getter && interlock_steering.getter
            && Watchdog_OK.getter && ServiceBrake_OK.getter;


}

/* last results, no new sample taken here (not used anywhere yet) */
bool Monitor_InterlocksOk(void)   { return interlock_valve1.getter && interlock_valve2.getter && interlock_proportional.getter && interlock_steering.getter; }
bool Monitor_TankOk(void)         { return TankPressure_OK.getter; }
bool Monitor_BrakeOk(void)        { return BrakePressure_OK.getter;}
bool Monitor_ApuOk(void)          { return apu_OK.getter;  }
bool Monitor_ServiceBrakeOk(void) { return ServiceBrake_OK.getter;   }
bool Monitor_WatchdogOk(void)     { return Watchdog_OK.getter;  }
