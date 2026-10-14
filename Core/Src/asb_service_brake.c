/* =========================================================
 * ASB Service Brake - Implementation
 * Aristurtle Formula Student | 2026
 *
 * Sets the DAC output (PA4) that drives the proportional valve and keeps
 * track of the state we asked for. The DAC holds its value until the
 * next HAL_DAC_SetValue(), so each function is a one-shot command.
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

/* Service Brake Engage - full pressure (used by the initial check) */
void ServiceBrake_Engage(void)
{
    service_brake_state = SERVICE_BRAKE_ENGAGED;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ASB_SB_DAC_ENGAGED);
}

/* Service Brake Disengage - no pressure */
void ServiceBrake_Disengage(void)
{
    service_brake_state = SERVICE_BRAKE_DISENGAGED;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ASB_SB_DAC_DISENGAGED);
}
/* Service Brake Park - hold the car in AS_Ready */
void ServiceBrake_Park(void)
{
    service_brake_state = SERVICE_BRAKE_PARK;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ASB_SB_DAC_PARK);
}

/* Available - AS_Driving. main.c calls Disengage() right before this, so
   while driving the DAC sits at 0 and the ASB does no linear braking */
void ServiceBrake_Available(void)
{
    service_brake_state = SERVICE_BRAKE_AVAILABLE;
    // TODO: implement the available state properly, for now it only
    // sets the state and leaves the DAC alone
}
/* Service Brake State */
ServiceBrakeState_t ServiceBrake_State(void) { return service_brake_state; }
