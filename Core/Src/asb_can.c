/* =========================================================
 * ASB CAN - Implementation
 * Aristurtle Formula Student | 2026
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

/* Private RX/TX structs */
static struct can_mcu_dv_system_status_t  can_apu_state;
static struct can_mcu_vcu_bools_t         can_vcu_bools;
static struct can_mcu_smu_brake_t         can_smu_brake;
static struct can_mcu_asb_t               can_tx_asb;
static struct can_mcu_asb_datalogger_t    can_tx_datalogger;
static struct can_mcu_res_status_t 		  can_res;
static struct can_mcu_vcu_brake_t 		  can_vcu_brake; 
static struct can_mcu_apu_set_finished_t  can_apu_set_finished;   


/* Private CAN buffers */
static CAN_RxHeaderTypeDef msg_header_rx;
static uint8_t rx_data[8];
static uint8_t tx_data[8];
static uint32_t tx_mailbox;

/* Decoded RX values */
static float brake_pressure_front = 0.0f;
static float brake_pressure_rear  = 0.0f;

/* APU alive tracking */
static uint32_t apu_last_rx_tick = 0U;
static uint32_t brake_front_last_rx_tick = 0U;
static uint32_t brake_rear_last_rx_tick = 0U;
static uint32_t res_last_rx_tick = 0U;


/* Initialization */
void CAN_App_Init(void)
{
    apu_last_rx_tick         = HAL_GetTick();
    brake_front_last_rx_tick = HAL_GetTick();
    brake_rear_last_rx_tick  = HAL_GetTick();
    res_last_rx_tick         = HAL_GetTick();

    CAN_FilterTypeDef fc;

    fc.FilterMode           = CAN_FILTERMODE_IDLIST;
    fc.FilterScale          = CAN_FILTERSCALE_32BIT;
    fc.FilterFIFOAssignment = CAN_RX_FIFO0;
    fc.FilterActivation     = ENABLE;
    fc.SlaveStartFilterBank = 27;

    static const uint16_t ids[] = {
        0x502,   /* DV_SYSTEM_STATUS  */
        0x065,   /* SMU_BRAKE          */
        0x320,   /* VCU_BOOLS          */
        0x191,   /* RES_STATUS         */
        0x78,   /* VCU_BRAKE          */
        0x2d,   /* APU_SET_FINISHED   */
    };
    const int count = sizeof(ids) / sizeof(ids[0]);
    int bank = 0;

    for (int i = 0; i < count; i += 2)
    {
        fc.FilterBank     = bank++;
        fc.FilterIdHigh   = ids[i] << 5;
        fc.FilterIdLow    = 0;

        if (i + 1 < count) {
            fc.FilterMaskIdHigh = ids[i + 1] << 5;
            fc.FilterMaskIdLow  = 0;
        } else {
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
    memset(&can_vcu_brake, 0, sizeof(can_vcu_brake));
    memset(&can_apu_set_finished, 0, sizeof(can_apu_set_finished));
    memset(rx_data, 0, sizeof(rx_data));
    memset(tx_data, 0, sizeof(tx_data));
}

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
void CAN_SendAsbStatus(void)
{
    can_tx_asb.asms_state             = SYS_GetASMS();
    can_tx_asb.tsms_out               = SYS_GetTSMS();   /* raw: 1 = closed, same as main */
    can_tx_asb.r_ebs_status           = EBS_State();
    can_tx_asb.ebs_status             = EBS_State();
    can_tx_asb.sdc_ok                 = SDC_IsClosed();
    can_tx_asb.initial_check_step     = (uint8_t)IC_GetState();
    if(IC_GetState()>= IC_NOTIFY_APU) { can_tx_asb.initial_checked = true; }
    else { can_tx_asb.initial_checked = false; }
    // initial_checked only goes true from IC_NOTIFY_APU onwards (check done, APU told)
    can_tx_asb.steering_state          = SYS_GetInterlockSteering();

    can_mcu_asb_pack(tx_data, &can_tx_asb, sizeof(tx_data));
    CAN_SendStdMessage(CAN_MCU_ASB_FRAME_ID, CAN_MCU_ASB_LENGTH, tx_data);
}

/* DBC: the ASB_Datalogger pressure signals are 8-bit, scale 1 -> raw bar, 0..255 */
static uint8_t pressure_to_u8(float bar)
{
    if (bar <= 0.0f)   { return 0U; }
    if (bar >= 255.0f) { return 255U; }
    return (uint8_t)bar;
}

void CAN_SendDatalogger(void)
{
    can_tx_datalogger.brake_pressure_front                      = pressure_to_u8(brake_pressure_front);
    can_tx_datalogger.brake_pressure_rear                       = pressure_to_u8(brake_pressure_rear);
    can_tx_datalogger.ebs_tank_pressure_sensor                  = pressure_to_u8(Sensors_GetTankPressure1());
    can_tx_datalogger.ebs_redundancy_tank_pressure_sensor       = pressure_to_u8(Sensors_GetTankPressure2());
    can_tx_datalogger.eb_sstate_activated                       = (EBS_State() == EBS_TRIGGERED) ? 1U : 0U;
    can_tx_datalogger.eb_sstate_armed                           = (EBS_State() == EBS_ARMED) ? 1U : 0U;
    can_tx_datalogger.eb_sstate_unavailable                     = (EBS_State() == EBS_UNAVAILABLE) ? 1U : 0U; 
    can_tx_datalogger.servicebrakestate_available               = (ServiceBrake_State() == SERVICE_BRAKE_AVAILABLE) ? 1U : 0U;
    can_tx_datalogger.servicebrakestate_engaged                 = (ServiceBrake_State() == SERVICE_BRAKE_ENGAGED) ? 1U : 0U;
    can_tx_datalogger.servicebrakestate_disengaged              = (ServiceBrake_State() == SERVICE_BRAKE_DISENGAGED) ? 1U : 0U;
    can_tx_datalogger.ebs_activation_valve_ok                   = (SYS_GetInterlock1()) ? 1U : 0U;
    can_tx_datalogger.ebs_redundancy_activation_valve_ok        = (SYS_GetInterlock2()) ? 1U : 0U;
    can_tx_datalogger.steering_actuator_interlock               = (SYS_GetInterlockSteering()) ? 1U : 0U;
    can_tx_datalogger.propotional_valve_ok                      = (SYS_GetInterlockService()) ? 1U : 0U;
    can_tx_datalogger.watchdog_ok                               = WDG_IsRunning() ? 1U : 0U;

    can_mcu_asb_datalogger_pack(tx_data, &can_tx_datalogger, sizeof(tx_data));
    CAN_SendStdMessage(CAN_MCU_ASB_DATALOGGER_FRAME_ID, CAN_MCU_ASB_DATALOGGER_LENGTH, tx_data);
}

void CAN_ProcessRxMessage(void)
{
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &msg_header_rx, rx_data) != HAL_OK)
    {
        Error_Handler();
    }

    switch (msg_header_rx.StdId)
    {
        case CAN_MCU_SMU_BRAKE_FRAME_ID:
            if (can_mcu_smu_brake_unpack(&can_smu_brake, rx_data, msg_header_rx.DLC) == 0)
            {
                brake_pressure_front = can_mcu_smu_brake_brake_pressure_front_decode(can_smu_brake.brake_pressure_front);

                /* Monitor if brake pressure front is alive */
                brake_front_last_rx_tick = HAL_GetTick();
            }
            break;

        case CAN_MCU_VCU_BRAKE_FRAME_ID:
            if (can_mcu_vcu_brake_unpack(&can_vcu_brake, rx_data, msg_header_rx.DLC) == 0)
            {
                brake_pressure_rear = can_mcu_vcu_brake_hydr_press_rear_decode(can_vcu_brake.hydr_press_rear);

                /* Monitor if brake pressure rear is alive */
                brake_rear_last_rx_tick = HAL_GetTick();
            }
            break;

        case CAN_MCU_DV_SYSTEM_STATUS_FRAME_ID:
            if (can_mcu_dv_system_status_unpack(&can_apu_state, rx_data, msg_header_rx.DLC) == 0)
            {
                /*Monitor if APU is alive*/
                apu_last_rx_tick = HAL_GetTick();
            }
            break;

        case CAN_MCU_VCU_BOOLS_FRAME_ID:
            can_mcu_vcu_bools_unpack(&can_vcu_bools, rx_data, msg_header_rx.DLC);
            break;

        case CAN_MCU_RES_STATUS_FRAME_ID:
            if (can_mcu_res_status_unpack(&can_res, rx_data, msg_header_rx.DLC) == 0)
            {
                /* Monitor if RES is alive */
                if (can_res.signal_strength > RES_OK) { res_last_rx_tick = HAL_GetTick(); }
            }
            break;

        case CAN_MCU_APU_SET_FINISHED_FRAME_ID:
            can_mcu_apu_set_finished_unpack(&can_apu_set_finished, rx_data, msg_header_rx.DLC);
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

/* RX Getters - DASH */

float CAN_GetBrakePressureFront(void)
{
    return brake_pressure_front;
}

float CAN_GetBrakePressureRear(void)
{
    return brake_pressure_rear;
}

/* RX Getters - RES */

uint8_t CAN_GetRESSignalStrength(void)
{
    return can_res.signal_strength;
}

/* APU Alive */

bool CAN_IsAPUAlive(void)
{
    return (HAL_GetTick() - apu_last_rx_tick < 480U);
}

/* RES Alive */

bool CAN_IsRESAlive(void)
{
    return (HAL_GetTick() - res_last_rx_tick < 100U);
}

/* Brake Pressure Alive */

bool CAN_IsBrakePressureAlive(void)
{
    /* Sheet 8: sensors @10ms, timeout 100ms - both sources have to be alive */
    uint32_t now = HAL_GetTick();
    return (now - brake_front_last_rx_tick < 100U) &&
           (now - brake_rear_last_rx_tick  < 100U);
}

