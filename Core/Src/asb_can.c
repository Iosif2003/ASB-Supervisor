#include "main.h"
#include <stdbool.h>

void CAN_SetInitialChecked(bool checked)
{
    canTxStruct_asb.initial_checked = checked;
}

uint8_t CAN_GetASState(void)
{
    return can_mcu_apu_state_mission.as_state;
}