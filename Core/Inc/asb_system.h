#ifndef ASB_SYSTEM_H
#define ASB_SYSTEM_H

#include <stdbool.h>

/* =========================================================
 * ASB System Control - asb_system.h
 * Aristurtle Formula Student | 2026
 * ========================================================= */

 /* Initialization */
 void ASB_System_Init(void);

 /* SDC Control */
void SDC_Close(void);
void SDC_Open(void);

/* Watchdog Control */
void WDG_Reset(void);
void WDG_Stop(void);
void WDG_Start(void);

/* GPIO Input Reads */
void Inputs_Sample(void);
bool SYS_GetASMS(void);
bool SYS_GetTSMS(void);
bool SYS_GetASRelayIn(void);
bool SYS_GetASRelayOut(void);
bool SYS_GetInterlock1(void);
bool SYS_GetInterlock2(void);
bool SYS_GetInterlockService(void);
bool SYS_GetInterlockSteering(void);

/* States */
bool SDC_IsClosed(void);
bool WDG_IsRunning(void);

/* Mission Selection */
typedef enum {
    MISSION_NONE       = 0,
    MISSION_AUTONOMOUS = 1,
    MISSION_MANUAL     = 2
} MissionMode_t;

int SYS_GetSelectedMission(void);
 
void Manual_Run(void);

#endif /* ASB_SYSTEM_H */


