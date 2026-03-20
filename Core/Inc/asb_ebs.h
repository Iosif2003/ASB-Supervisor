#ifndef ASB_EBS_H
#define ASB_EBS_H

#include <stdbool.h>

/* =========================================================
 * ASB EBS Control — Public API
 * Aristurtle Formula Student | 2026
 * ========================================================= */

/* Control */
void EBS_Activate(void);
void EBS_Release_Valve1(void);
void EBS_Release_Valve2(void);
void EBS_Activate_Valve1(void);
void EBS_Activate_Valve2(void);
void EBS_Release_All(void);

/* States */
bool EBS_IsActivated(void);
bool EBS_Valve1_State(void);
bool EBS_Valve2_State(void);
bool EBS_RelayState(void);

 #endif /* ASB_EBS_H */