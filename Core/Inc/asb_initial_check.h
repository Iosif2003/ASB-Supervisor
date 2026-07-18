#ifndef ASB_INITIAL_CHECK_H
#define ASB_INITIAL_CHECK_H

#include <stdbool.h>

/* =========================================================
 * ASB Initial Check - asb_initial_check.h
 * Aristurtle Formula Student | 2026
 * ========================================================= */

/* Initial Check States */
typedef enum {
    IC_WAIT_MISSION               = 0,
    IC_WAIT_ASMS                  = 1,
    IC_ACTIVATE_EBS               = 2,
    IC_WAIT_RES                   = 3,
    IC_WAIT_PRESSURES             = 4,
    IC_SET_DIGPIN_HIGH            = 5,
    IC_WAIT_SDC_CLOSE_FIRST_TIME  = 6,
    IC_STOP_WATCHDOG              = 7,
    IC_WAIT_SDC_OPEN_SECOND_TIME  = 8,
    IC_START_WATCHDOG             = 9,
    IC_WAIT_SDC_CLOSE_SECOND_TIME = 10,
    IC_WAIT_TSMS                  = 11,
    IC_RELEASE_EBS                = 12,
    IC_ENGAGE_SYSTEM1             = 13,
    IC_RELEASE_SYSTEM1            = 14,
    IC_ENGAGE_SYSTEM2             = 15,
    IC_RELEASE_SYSTEM2            = 16,
    IC_ENGAGE_SERVICE_BRAKE       = 17,
    IC_RELEASE_SERVICE_BRAKE      = 18,
    IC_PARK_SERVICE_BRAKE         = 19,
    IC_NOTIFY_APU                 = 20,
    IC_ENABLE_OPM                 = 21,
    IC_COMPLETE                   = 22,
} IC_State_t;

/* Initialization */
void IC_Init(void);

/* Run */
void IC_Run(void);

/* Getters */
IC_State_t IC_GetState(void);
bool IC_IsComplete(void);


#endif /* ASB_INITIAL_CHECK_H */