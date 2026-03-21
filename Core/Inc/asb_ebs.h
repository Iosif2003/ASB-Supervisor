#ifndef ASB_EBS_H
#define ASB_EBS_H

#include <stdbool.h>

/* =========================================================
 * ASB EBS Control - asb_ebs.h
 * Aristurtle Formula Student | 2026
 * ========================================================= */

/* EBS States */
bool EBS_IsActivated(void);
bool EBS_System1_State(void);
bool EBS_System2_State(void);

/* EBS Initialization */
void EBS_Init(void);

/* EBS Control */
void EBS_Activate(void);
void EBS_Activate_System1(void);
void EBS_Activate_System2(void);
void EBS_Release_All(void);
void EBS_Release_System1(void);
void EBS_Release_System2(void);

#endif /* ASB_EBS_H */