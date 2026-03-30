/* =========================================================
 * ASB EBS Control — Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_ebs.h"
#include "main.h"
#include <stdbool.h>

/* Private Variables */
static bool ebs_is_activated  = true;
static bool ebs_system1_state = true;
static bool ebs_system2_state = true;

static EBS_State_t EBS_state = EBS_UNAVAILABLE;

/* EBS States */
bool EBS_IsActivated(void)   { return ebs_is_activated;  }
bool EBS_System1_State(void) { return ebs_system1_state; }
bool EBS_System2_State(void) { return ebs_system2_state; }

EBS_State_t EBS_State(void) { return EBS_state; }
/* EBS Initialization */
void EBS_Init(void)
{
    ebs_is_activated  = true;
    ebs_system1_state = true;
    ebs_system2_state = true;

    HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port, Valve1_GND_ST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port, Valve2_GND_ST_Pin, GPIO_PIN_RESET);
}

/* EBS Activation */
void EBS_Activate(void)
{
    ebs_is_activated  = true;
    ebs_system1_state = true;
    ebs_system2_state = true;

    HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port, Valve1_GND_ST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port, Valve2_GND_ST_Pin, GPIO_PIN_RESET);
}

/* EBS Release All */
void EBS_Release_All(void)
{
    ebs_system1_state = false;
    ebs_system2_state = false;
    ebs_is_activated  = false;

    HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port, Valve1_GND_ST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port, Valve2_GND_ST_Pin, GPIO_PIN_SET);
}

/* EBS Release System 1 */
void EBS_Release_System1(void)
{
    ebs_system1_state = false;
    if(!ebs_system2_state)
        ebs_is_activated = false;
    HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port, Valve1_GND_ST_Pin, GPIO_PIN_SET);
}

/* EBS Release System 2 */
void EBS_Release_System2(void)
{
    ebs_system2_state = false;
    if(!ebs_system1_state)
        ebs_is_activated = false;

    HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port, Valve2_GND_ST_Pin, GPIO_PIN_SET);
}

/* EBS Activate System 1 */
void EBS_Activate_System1(void)
{
    ebs_system1_state = true;
    if(ebs_system2_state)
        ebs_is_activated = true;

    HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port, Valve1_GND_ST_Pin, GPIO_PIN_RESET);
}

/* EBS Activate System 2 */
void EBS_Activate_System2(void)
{
    ebs_system2_state = true;
    if(ebs_system1_state)
        ebs_is_activated = true;

    HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port, Valve2_GND_ST_Pin, GPIO_PIN_RESET);
}