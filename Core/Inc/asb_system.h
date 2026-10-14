#ifndef ASB_SYSTEM_H
#define ASB_SYSTEM_H

#include <stdbool.h>

/* =========================================================
 * ASB System Control - asb_system.h
 * Aristurtle Formula Student | 2026
 *
 * Board level I/O that is not brakes or CAN:
 *  - SDC: the AS relay that closes / opens our part of the shutdown circuit
 *  - watchdog: TIM3 pulse train the external hardware watchdog listens to
 *  - digital inputs: ASMS, TSMS, AS relay in/out, valve interlocks
 *  - mission selection and the manual driving checks
 * ========================================================= */

 /* Initialization */
 void ASB_System_Init(void);

 /* SDC Control - drives the AS relay signal (PB13) */
void SDC_Close(void);
void SDC_Open(void);

/* Watchdog Control */
void WDG_Reset(void);   /* pet the watchdog, has to be called every 10ms */
void WDG_Stop(void);    /* declared but not implemented, not used */
void WDG_Start(void);   /* declared but not implemented, not used */

/* GPIO Input Reads - Inputs_Sample() reads all pins once per 10ms tick,
   the getters return that snapshot so one tick works on consistent values */
void Inputs_Sample(void);
bool SYS_GetASMS(void);              /* 1 = ASMS on */
bool SYS_GetTSMS(void);              /* 1 = TSMS closed, SDC live */
bool SYS_GetASRelayIn(void);         /* 1 = SDC closed up to our relay (RES etc.) */
bool SYS_GetASRelayOut(void);        /* 1 = our relay output closed */
bool SYS_GetInterlock1(void);        /* 1 = EBS valve 1 interlock ok */
bool SYS_GetInterlock2(void);        /* 1 = EBS valve 2 interlock ok */
bool SYS_GetInterlockService(void);  /* 1 = proportional valve interlock ok */
bool SYS_GetInterlockSteering(void); /* 1 = steering actuator interlock ok */

/* States */
bool SDC_IsClosed(void);    /* what we last commanded, not the measured relay */
bool WDG_IsRunning(void);

/* Mission Selection */
typedef enum {
    MISSION_NONE       = 0,
    MISSION_AUTONOMOUS = 1,
    MISSION_MANUAL     = 2
} MissionMode_t;

int SYS_GetSelectedMission(void);   /* returns a MissionMode_t */

void Manual_Run(void);   /* manual driving checks, every 10ms */

#endif /* ASB_SYSTEM_H */

