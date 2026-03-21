/* =========================================================
 * ASB Service Brake — Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_service_brake.h"
#include "asb_config.h"
#include "main.h"

/* External handles from main.c */
extern DAC_HandleTypeDef hdac;

/* Private Variables */
static ServiceBrakeState_t service_brake_state = SERVICE_BRAKE_DISENGAGED;

/* Service Brake Initialization */
void ServiceBrake_Init(void)
{
    service_brake_state = SERVICE_BRAKE_DISENGAGED;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ASB_SB_DAC_DISENGAGED);
}

/* Service Brake Engage */
void ServiceBrake_Engage(void)
{
    service_brake_state = SERVICE_BRAKE_ENGAGED;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ASB_SB_DAC_ENGAGED);
}

/* Service Brake Disengage */
void ServiceBrake_Disengage(void)
{
    service_brake_state = SERVICE_BRAKE_DISENGAGED;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ASB_SB_DAC_DISENGAGED);
}
/* Service Brake Park */
void ServiceBrake_Park(void)
{
    service_brake_state = SERVICE_BRAKE_PARK;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ASB_SB_DAC_PARK);
}

/* Service Brake State */
ServiceBrakeState_t ServiceBrake_State(void) { return service_brake_state; }