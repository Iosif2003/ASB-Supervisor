
/* =========================================================
 * ASB CAN - Implementation
 * Aristurtle Formula Student | 2026
 *
 * RX: every accepted frame raises the CAN1 RX0 interrupt ->
 *     HAL_CAN_RxFifo0MsgPendingCallback (main.c) -> CAN_ProcessRxMessage().
 *     The frame is decoded with the cantools unpack functions and kept in
 *     a static struct. For the safety relevant senders (APU, RES, brake
 *     pressures) we also store the tick of the last good frame, that's
 *     what the CAN_Is*Alive() timeouts look at.
 * TX: CAN_Send*() read the current state from the other modules, pack
 *     it and send it. They are called from the main loop, not from ISRs.
 * ========================================================= */

#include "asb_can.h"
#include "can_mcu.h"
#include "asb_config.h"
#include "asb_system.h"
#include "asb_initial_check.h"
#include "asb_service_brake.h"
#include "asb_ebs.h"
#include "asb_sensors.h"
#include "asb_monitoring.h"
#include <string.h>



/* External handles from main.c */
extern CAN_HandleTypeDef hcan1;

/* CANopen EMCY from the RES: 0x080 + NodeID (0x11). Deliberately NOT in the
   DBC - handled as a raw frame (bytes 0-1 error code, byte 2 error register).
   If the message is ever added to the DBC, switch to the generated unpack. */
#define RES_EMCY_FRAME_ID  0x091U

/* Private RX/TX structs - one per DBC message, layout from can_mcu.h */
static struct can_mcu_dv_system_status_t  can_apu_state;          /* RX 0x502 from APU */
static struct can_mcu_vcu_bools_t         can_vcu_bools;          /* RX 0x320 from VCU */
static struct can_mcu_smu_brake_t         can_smu_brake;          /* RX 0x065 front brake pressure */
static struct can_mcu_asb_t               can_tx_asb;             /* TX 0x023 */
static struct can_mcu_asb_datalogger_t    can_tx_datalogger;      /* TX 0x511 */
static struct can_mcu_asb_hydraulic_pressures_t can_tx_hydraulic; /* TX 0x512 */
static struct can_mcu_asb_tank_pressures_t      can_tx_tank;      /* TX 0x513 */
static struct can_mcu_res_status_t 		  can_res;                /* RX 0x191 from RES */
static struct can_mcu_vcu_brake_t 		  can_vcu_brake;          /* RX 0x078 rear brake pressure */
static struct can_mcu_apu_set_finished_t  can_apu_set_finished;   /* RX 0x02D from APU */
static struct can_mcu_apu_res_init_t      can_apu_res_init;       /* RX 0x000 NMT, APU -> RES */


/* Private CAN buffers */
static CAN_RxHeaderTypeDef msg_header_rx;
static uint8_t rx_data[8];
static uint8_t tx_data[8];   /* shared by all senders, fine since they all run from the main loop */
static uint32_t tx_mailbox;

/* Decoded RX values */
static float brake_pressure_front = 0.0f;
static float brake_pressure_rear  = 0.0f;
static uint8_t res_error_register = 0U;   /* byte 2 of RES EMCY, latched until reboot */

/* Tick (ms) of the last good frame of each sender, for the alive checks */
static uint32_t apu_last_rx_tick = 0U;
static uint32_t brake_front_last_rx_tick = 0U;
static uint32_t brake_rear_last_rx_tick = 0U;
static uint32_t res_last_rx_tick = 0U;


/* Initialization */
void CAN_App_Init(void)
{
    /* start the alive timers from now, so the first timeout counts from boot */
    apu_last_rx_tick         = HAL_GetTick();
    brake_front_last_rx_tick = HAL_GetTick();
    brake_rear_last_rx_tick  = HAL_GetTick();
    res_last_rx_tick         = HAL_GetTick();
    res_error_register       = 0U;

    /* RX filters: 32 bit ID LIST mode, so each filter bank holds two exact
       IDs (one in the Id register, one in the Mask register) instead of
       id + mask. Only the IDs below get into FIFO0, everything else on
       the bus is dropped in hardware. */
    CAN_FilterTypeDef fc;

    fc.FilterMode           = CAN_FILTERMODE_IDLIST;
    fc.FilterScale          = CAN_FILTERSCALE_32BIT;
    fc.FilterFIFOAssignment = CAN_RX_FIFO0;
    fc.FilterActivation     = ENABLE;
    fc.SlaveStartFilterBank = 27;   /* banks 0-26 for CAN1, CAN2 is not used */

    static const uint16_t ids[] = {
        0x502,   /* DV_SYSTEM_STATUS  */
        0x065,   /* SMU_BRAKE          */
        0x320,   /* VCU_BOOLS          */
        0x191,   /* RES_STATUS         */
        0x78,   /* VCU_BRAKE          */
        0x2d,   /* APU_SET_FINISHED   */
        0x091,  /* RES EMCY (CANopen, raw - not in DBC) */
        0x000,  /* APU_RES_INIT (CANopen NMT from APU to the RES) */
    };
    const int count = sizeof(ids) / sizeof(ids[0]);
    int bank = 0;

    /* two IDs per bank. The 11 bit standard ID sits in bits 31..21 of the
       filter register, so in the high half word it is shifted left by 5.
       IDE / RTR bits stay 0 -> only standard data frames match. */
    for (int i = 0; i < count; i += 2)
    {
        fc.FilterBank     = bank++;
        fc.FilterIdHigh   = ids[i] << 5;
        fc.FilterIdLow    = 0;

        if (i + 1 < count) {
            fc.FilterMaskIdHigh = ids[i + 1] << 5;
            fc.FilterMaskIdLow  = 0;
        } else {
            /* odd number of IDs -> second slot of the last bank becomes 0x000 */
            fc.FilterMaskIdHigh = 0;
            fc.FilterMaskIdLow  = 0;
        }

        if (HAL_CAN_ConfigFilter(&hcan1, &fc) != HAL_OK) {
            Error_Handler();
        }
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    /* Enable RX interrupt - without this HAL_CAN_RxFifo0MsgPendingCallback never fires */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    /* Default RX state */
    memset(&can_apu_state, 0, sizeof(can_apu_state));
    memset(&can_vcu_bools, 0, sizeof(can_vcu_bools));
    memset(&can_smu_brake, 0, sizeof(can_smu_brake));
    memset(&can_res, 0, sizeof(can_res));
    memset(&can_tx_asb, 0, sizeof(can_tx_asb));
    memset(&can_tx_datalogger, 0, sizeof(can_tx_datalogger));
    memset(&can_tx_hydraulic, 0, sizeof(can_tx_hydraulic));
    memset(&can_tx_tank, 0, sizeof(can_tx_tank));
    memset(&can_vcu_brake, 0, sizeof(can_vcu_brake));
    memset(&can_apu_set_finished, 0, sizeof(can_apu_set_finished));
    memset(&can_apu_res_init, 0, sizeof(can_apu_res_init));
    memset(rx_data, 0, sizeof(rx_data));
    memset(tx_data, 0, sizeof(tx_data));
}

/* Send one standard (11 bit) data frame. Waits max 5ms for a free TX
   mailbox. AutoRetransmission is off (MX_CAN1_Init), so a frame is tried
   once and then dropped, a dead bus can't block the main loop. */
static HAL_StatusTypeDef CAN_SendStdMessage(uint32_t std_id, uint32_t dlc,
                                             const uint8_t *data)
{
    CAN_TxHeaderTypeDef hdr = {0};
    uint32_t start = HAL_GetTick();

    hdr.StdId = std_id;
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = dlc;

    /* Wait for free mailbox - max 5ms */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        if ((HAL_GetTick() - start) > 5U)
        {
            /* Bus dead / no ACK: flush all mailboxes so they don't wedge forever */
            HAL_CAN_AbortTxRequest(&hcan1,
                CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
            return HAL_TIMEOUT;
        }
    }

    /* Send */
    if (HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t *)data, &tx_mailbox) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* CAN Senders */

/* ASB status for the APU (every 20ms). initial_checked tells the APU it can
   go to AS_Ready, the EBS fields get forwarded into DV_System_Status. */
void CAN_SendAsbStatus(void)
{
    can_tx_asb.asms_state             = SYS_GetASMS();
    can_tx_asb.tsms_out               = SYS_GetTSMS();   /* raw: 1 = closed, same as main */
    /* DV spec 2026 values - forwarded by the APU into DV_System_Status */
    can_tx_asb.r_ebs_status           = EBS_DvRedundancyStatus();
    can_tx_asb.ebs_status             = EBS_DvStatus();
    can_tx_asb.sdc_ok                 = SYS_GetASRelayOut();
    can_tx_asb.initial_check_step     = (uint8_t)IC_GetState();   /* IC_State_t number, for debugging */
    if(IC_GetState()>= IC_NOTIFY_APU) { can_tx_asb.initial_checked = true; IC_IsComplete(); }
    else { can_tx_asb.initial_checked = false; }
    // initial_checked only goes true from IC_NOTIFY_APU onwards (check done, APU told)
    // (the IC_IsComplete() call above has no effect, its result is not used)
    can_tx_asb.steering_state          = SYS_GetInterlockSteering();

    can_mcu_asb_pack(tx_data, &can_tx_asb, sizeof(tx_data));
    CAN_SendStdMessage(CAN_MCU_ASB_FRAME_ID, CAN_MCU_ASB_LENGTH, tx_data);
}

/* Datalogger frame (every 100ms) - for logging / debugging.
   Each state is sent as one bit per value. */
void CAN_SendDatalogger(void)
{
    can_tx_datalogger.eb_sstate_activated                       = (EBS_State() == EBS_TRIGGERED) ? 1U : 0U;
    can_tx_datalogger.eb_sstate_armed                           = (EBS_State() == EBS_ARMED) ? 1U : 0U;
    can_tx_datalogger.eb_sstate_unavailable                     = (EBS_State() == EBS_UNAVAILABLE) ? 1U : 0U;
    can_tx_datalogger.servicebrakestate_available               = (ServiceBrake_State() == SERVICE_BRAKE_AVAILABLE) ? 1U : 0U;
    can_tx_datalogger.servicebrakestate_engaged                 = (ServiceBrake_State() == SERVICE_BRAKE_ENGAGED) ? 1U : 0U;
    can_tx_datalogger.servicebrakestate_disengaged              = (ServiceBrake_State() == SERVICE_BRAKE_DISENGAGED) ? 1U : 0U;
    /* interlock inputs, 1 = ok */
    can_tx_datalogger.ebs_activation_valve_ok                   = (SYS_GetInterlock1()) ? 1U : 0U;
    can_tx_datalogger.ebs_redundancy_activation_valve_ok        = (SYS_GetInterlock2()) ? 1U : 0U;
    can_tx_datalogger.steering_actuator_interlock               = (SYS_GetInterlockSteering()) ? 1U : 0U;
    can_tx_datalogger.propotional_valve_ok                      = (SYS_GetInterlockService()) ? 1U : 0U;
    /* note: wdg_is_running is never set to true at the moment, so this is always 0 */
    can_tx_datalogger.watchdog_ok                               = WDG_IsRunning() ? 1U : 0U;

    can_mcu_asb_datalogger_pack(tx_data, &can_tx_datalogger, sizeof(tx_data));
    CAN_SendStdMessage(CAN_MCU_ASB_DATALOGGER_FRAME_ID, CAN_MCU_ASB_DATALOGGER_LENGTH, tx_data);
}


/* Re-sends the front / rear brake pressure we received, in our own frame
   (every 50ms). The *_encode() functions apply the DBC scale and offset. */
void CAN_SendHydraulicPressures(void)
{
    can_tx_hydraulic.brake_pressure_front =  can_mcu_asb_hydraulic_pressures_brake_pressure_front_encode(brake_pressure_front);
    can_tx_hydraulic.brake_pressure_rear  =  can_mcu_asb_hydraulic_pressures_brake_pressure_rear_encode(brake_pressure_rear);

    can_mcu_asb_hydraulic_pressures_pack(tx_data, &can_tx_hydraulic, sizeof(tx_data));
    CAN_SendStdMessage(CAN_MCU_ASB_HYDRAULIC_PRESSURES_FRAME_ID, CAN_MCU_ASB_HYDRAULIC_PRESSURES_LENGTH, tx_data);
}

/* Tank pressures from the ADC (every 50ms). tank 1 = EBS, tank 2 = EBS redundancy */
void CAN_SendTankPressures(void)
{

    can_tx_tank.ebs_tank_pressure_sensor            =   can_mcu_asb_tank_pressures_ebs_tank_pressure_sensor_encode(Sensors_GetTankPressure1());
	can_tx_tank.ebs_redundancy_tank_pressure_sensor =	can_mcu_asb_tank_pressures_ebs_redundancy_tank_pressure_sensor_encode(Sensors_GetTankPressure2());

	can_mcu_asb_tank_pressures_pack(tx_data, &can_tx_tank, sizeof(tx_data));
    CAN_SendStdMessage(CAN_MCU_ASB_TANK_PRESSURES_FRAME_ID, CAN_MCU_ASB_TANK_PRESSURES_LENGTH, tx_data);
}

/* Runs inside the CAN RX interrupt - keep it short, only decode and store.
   unpack() returns 0 on success, a too short frame doesn't refresh the
   alive timer. */
void CAN_ProcessRxMessage(void)
{
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &msg_header_rx, rx_data) != HAL_OK)
    {
        Error_Handler();
    }

    switch (msg_header_rx.StdId)
    {
        case CAN_MCU_SMU_BRAKE_FRAME_ID:
            /* front hydraulic pressure from the SMU (+ out of range flag) */
            if (can_mcu_smu_brake_unpack(&can_smu_brake, rx_data, msg_header_rx.DLC) == 0)
            {
                brake_pressure_front = can_mcu_smu_brake_brake_pressure_front_decode(can_smu_brake.brake_pressure_front);

                /* Monitor if brake pressure front is alive */
                brake_front_last_rx_tick = HAL_GetTick();
            }
            break;

        case CAN_MCU_VCU_BRAKE_FRAME_ID:
            /* rear hydraulic pressure from the VCU */
            if (can_mcu_vcu_brake_unpack(&can_vcu_brake, rx_data, msg_header_rx.DLC) == 0)
            {
                brake_pressure_rear = can_mcu_vcu_brake_hydr_press_rear_decode(can_vcu_brake.hydr_press_rear);

                /* Monitor if brake pressure rear is alive */
                brake_rear_last_rx_tick = HAL_GetTick();
            }
            break;

        case CAN_MCU_DV_SYSTEM_STATUS_FRAME_ID:
            /* APU: AS state + selected mission */
            if (can_mcu_dv_system_status_unpack(&can_apu_state, rx_data, msg_header_rx.DLC) == 0)
            {
                /*Monitor if APU is alive*/
                apu_last_rx_tick = HAL_GetTick();
            }
            break;

        case CAN_MCU_VCU_BOOLS_FRAME_ID:
            /* VCU mode (manual / driverless), no alive check on this one */
            can_mcu_vcu_bools_unpack(&can_vcu_bools, rx_data, msg_header_rx.DLC);
            break;

        case CAN_MCU_RES_STATUS_FRAME_ID:
            /* RES: stop bit, go signals, radio signal strength */
            if (can_mcu_res_status_unpack(&can_res, rx_data, msg_header_rx.DLC) == 0)
            {
                /* Monitor if RES is alive - a frame with weak radio signal
                   doesn't count, so a bad link times out like a lost one */
                if (can_res.signal_strength > RES_OK) { res_last_rx_tick = HAL_GetTick(); }
            }
            break;

        case CAN_MCU_APU_SET_FINISHED_FRAME_ID:
            can_mcu_apu_set_finished_unpack(&can_apu_set_finished, rx_data, msg_header_rx.DLC);
            break;

        case CAN_MCU_APU_RES_INIT_FRAME_ID:
            /* CANopen NMT (0x000) that the APU sends to start the RES */
            can_mcu_apu_res_init_unpack(&can_apu_res_init, rx_data, msg_header_rx.DLC);
            break;

        case RES_EMCY_FRAME_ID:
            /* bytes 0-1 = error code, byte 2 = error register (0 = healthy).
               Latched: a RES that reported an internal fault is not trusted
               again without a power cycle, even if it later sends error-reset. */
            if ((msg_header_rx.DLC >= 3U) && (rx_data[2] != 0U))
            {
                res_error_register = rx_data[2];
            }
            break;

        default:
            break;
    }
}

/* RX Getters - APU */

uint8_t CAN_GetASState(void)
{
    return can_apu_state.as_status;
}

uint8_t CAN_GetASMission(void)
{
    return can_apu_state.ami_state;
}

bool CAN_GetASSetFinished(void)
{
    return can_apu_set_finished.set_finished;
}

/* RX Getters - VCU */

uint8_t CAN_GetVCUMode(void)
{
    return can_vcu_bools.mode;
}

int CAN_GetServoCommand(void)
{
    /* No servo/brake command signal exists in the current DBC. 0 = no
       actuation expected, keeps service_brake_ok() passive until the
       service brake monitor rewrite brings a real source. */
    return 0;
}

/* RX Getters - brake pressures (bar) */

float CAN_GetBrakePressureFront(void)
{
    return brake_pressure_front;
}

float CAN_GetBrakePressureRear(void)
{
    return brake_pressure_rear;
}

/* RX Getters - RES */

uint8_t CAN_GetRESStop(void)   { return can_res.stop;   }
uint8_t CAN_GetRESToggle(void) { return can_res.toggle; }
uint8_t CAN_GetRESButton(void) { return can_res.button; }

bool CAN_GetRESEmergency(void)
{
    /* stop bit: 0 = emergency active. can_res starts zeroed, so before the
       first RES frame this reports emergency - deliberate fail-safe. */
    return (can_res.stop == 0U);
}

uint8_t CAN_GetRESSignalStrength(void)
{
    return can_res.signal_strength;
}

uint8_t CAN_GetRESErrorRegister(void)
{
    return res_error_register;
}

/* RX Getters - RES Init (NMT sent by APU, monitored by ASB) */

uint8_t CAN_GetRESInitRequestedState(void) { return can_apu_res_init.requested_state; }
uint8_t CAN_GetRESInitAddressedNode(void)  { return can_apu_res_init.addresed_node;   }

/* Alive checks. The subtraction is unsigned, so it keeps working when
   HAL_GetTick() wraps around (after ~49 days). */

/* APU Alive */

bool CAN_IsAPUAlive(void)
{
    /* ASF Sheet 9: APU sends @100ms, timeout 300ms (tolerates 2 lost messages) */
    return (HAL_GetTick() - apu_last_rx_tick < 300U);
}

/* RES Alive */

bool CAN_IsRESAlive(void)
{
    /* ASF Sheet 8: RES sends @30ms, timeout 90ms */
    return (HAL_GetTick() - res_last_rx_tick < 90U);
}

/* Brake Pressure Alive */

bool CAN_IsBrakePressureAlive(void)
{
    /* Sheet 8: sensors @10ms, timeout 100ms - both sources have to be alive */
    uint32_t now = HAL_GetTick();
    return (now - brake_front_last_rx_tick < 100U) &&
           (now - brake_rear_last_rx_tick  < 100U);
}

/* out of range flag from the SMU_BRAKE frame, set by the SMU itself */
bool CAN_IsBrakeOutOfRange(void){
	return can_smu_brake.brake_out_of_range;

}
