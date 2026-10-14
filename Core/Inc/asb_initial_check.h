 #ifndef ASB_INITIAL_CHECK_H
#define ASB_INITIAL_CHECK_H

#include <stdbool.h>

/* =========================================================
 * ASB Initial Check - asb_initial_check.h
 * Aristurtle Formula Student | 2026
 *
 * The initial checkup the rules ask for before the car can go to
 * AS_Ready. It's a state machine stepped every 10ms by IC_Run() while
 * the APU is in AS_Off and a driverless mission is selected.
 * It tests the SDC / watchdog path, then each EBS valve on its own and
 * the service brake, by commanding the valve and checking the brake
 * pressure that comes back. Any failure -> back to the start with EBS
 * engaged and the SDC open.
 * ========================================================= */

/* Initial Check States - run top to bottom.
   The valve steps wait ASB_IC_VALVE_SETTLE_MS and then check the
   brake pressure. The number of the state is sent on CAN
   (initial_check_step), so the APU / dash can see where it is stuck. */
typedef enum {
    IC_WAIT_MISSION            = 0,  /* driverless mission selected */
    IC_WAIT_ASMS                  ,  /* ASMS switched on */
	IC_ACTIVATE_EBS				  ,  /* both EBS circuits brake */
    IC_WAIT_RES                   ,  /* SDC before the ASB closed (AS_Relay_In high) */
    IC_WAIT_PRESSURES             ,  /* both tanks in range + brakes engaged */
    IC_WAIT_SDC_CLOSE_FIRST_TIME  ,  /* we close the AS relay */
    IC_STOP_WATCHDOG              ,  /* watchdog test: main loop stops petting the watchdog */
    IC_WAIT_SDC_OPEN_SECOND_TIME  ,  /* -> the external watchdog has to open the relay */
    IC_START_WATCHDOG             ,  /* petting again */
    IC_WAIT_SDC_CLOSE_SECOND_TIME ,  /* -> relay closes again */
    IC_WAIT_TSMS                  ,  /* TSMS on = whole SDC closed */
    IC_RELEASE_SYSTEM1            ,  /* only circuit 2 brakes -> one valve pressure */
    IC_ENGAGE_SYSTEM1             ,  /* both brake -> engaged pressure */
    IC_RELEASE_SYSTEM2            ,  /* only circuit 1 brakes -> one valve pressure */
    IC_ENGAGE_SYSTEM2             ,  /* both brake -> engaged pressure */
    IC_RELEASE_SYSTEM1_SECOND_TIME,  /* only circuit 2 brakes again */
    IC_ENGAGE_SERVICE_BRAKE       ,  /* proportional valve full -> circuit 1 back -> engaged */
    IC_RELEASE_SYSTEM2_SECOND_TIME,  /* EBS fully released, only the service brake holds */
    IC_NOTIFY_APU                 ,  /* initial_checked = 1 on CAN, wait for AS_Ready */
    IC_COMPLETE                      /* done, stays here */
} IC_State_t;

/* Initialization */
void IC_Init(void);

/* Run - one step of the state machine, call every 10ms */
void IC_Run(void);

/* Getters */
IC_State_t IC_GetState(void);
bool IC_IsComplete(void);


#endif /* ASB_INITIAL_CHECK_H */
