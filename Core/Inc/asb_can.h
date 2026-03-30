#ifndef ASB_CAN_H
#define ASB_CAN_H
#include "main.h"
#include <stdbool.h>

/* =========================================================
 * ASB CAN — asb_can.h
 * Aristurtle Formula Student | 2026
 * ========================================================= */

/* Initialization */
void CAN_App_Init(void);

/* TX Tasks */
void CAN_SendAsbStatus(void);

/* RX Processing (called from HAL_CAN_RxFifo0MsgPendingCallback) */
void CAN_ProcessRxMessage(void);

/* RX Getters — APU */
uint8_t CAN_GetASState(void);
uint8_t CAN_GetASMission(void);
bool    CAN_GetASSetFinished(void);

/* RX Getters — VCU */
uint8_t CAN_GetVCUMode(void);
int     CAN_GetServoCommand(void);

/* RX Getters — DASH */
float CAN_GetBrakePressureFront(void);
float CAN_GetBrakePressureRear(void);

/* Communication Alive Checks */
bool CAN_IsAPUAlive(void);
bool CAN_IsBrakePressureAlive(void);

#endif /* ASB_CAN_H */