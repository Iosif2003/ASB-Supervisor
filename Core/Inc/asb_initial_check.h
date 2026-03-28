#ifndef ASB_INITIAL_CHECK_H
#define ASB_INITIAL_CHECK_H

#include <stdbool.h>

/* =========================================================
 * ASB Initial Check — asb_initial_check.h
 * Aristurtle Formula Student | 2026
 * ========================================================= */

/* Initial Check States */
typedef enum {
    IC_WAIT_MISSION = 0,
    IC_WAIT_ASMS,
    IC_ACTIVATE_EBS,
    IC_WAIT_RES,
    IC_WAIT_PRESSURE,
    IC_CLOSE_SDC,
    IC_WAIT_SDC_CLOSE,
    IC_STOP_WATCHDOG,
    IC_WAIT_RELAY_OPEN,
    IC_START_WATCHDOG,
    IC_WAIT_RELAY_RECLOSE,
    IC_WAIT_TSMS,
    IC_RELEASE_EBS,
    IC_WAIT_BRAKES_RELEASE,
    IC_ACTIVATE_SYSTEM1,
    IC_WAIT_BRAKES_S1,
    IC_RELEASE_SYSTEM1,
    IC_WAIT_RELEASE_S1,
    IC_ACTIVATE_SYSTEM2,
    IC_WAIT_BRAKES_S2,
    IC_RELEASE_SYSTEM2,
    IC_WAIT_RELEASE_S2,
    IC_ENGAGE_SERVICE_BRAKE,
    IC_WAIT_SB_ENGAGE,
    IC_DISENGAGE_SERVICE_BRAKE,
    IC_WAIT_SB_RELEASE,
    IC_PARK_SERVICE_BRAKE,
    IC_NOTIFY_APU,
    IC_WAIT_AS_READY,
    IC_ENABLE_OPM,
    IC_COMPLETE,
    IC_FAILED
} IC_State_t;

/* Initialization */
void IC_Init(void);

/* Run */
void IC_Run(void);

/* Getters */
IC_State_t IC_GetState(void);
bool IC_IsComplete(void);


#endif /* ASB_INITIAL_CHECK_H */