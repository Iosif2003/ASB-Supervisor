/* =========================================================
 * ASB CAN — Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_can.h"
#include "can_mcu.h"
#include "asb_system.h"
#include "asb_initial_check.h"
#include "asb_service_brake.h"
#include "asb_ebs.h"
#include "asb_sensors.h"
#include <string.h>

/* External handles from main.c */
extern CAN_HandleTypeDef hcan1;

/* ── Private RX/TX structs ── */
static struct can_mcu_apu_state_mission_t can_apu_state;
static struct can_mcu_vcu_bools_t         can_vcu_bools;
static struct can_mcu_vcu_servo_control_t can_vcu_servo;
static struct can_mcu_dash_brake_t        can_dash_brake;
static struct can_mcu_asb_t               can_tx_asb;

/* ── Private CAN buffers ── */
static CAN_RxHeaderTypeDef msg_header_rx;
static uint8_t rx_data[8];
static uint8_t tx_data[8];
static uint32_t tx_mailbox;

/* ── Decoded RX values ── */
static float brake_pressure_front = 0.0f;
static float brake_pressure_rear  = 0.0f;

/* ── APU alive tracking ── */
static uint32_t apu_last_rx_tick = 0U;
static uint32_t brake_last_rx_tick = 0U;


/* ── Initialization ── */
void CAN_App_Init(void)
{
    apu_last_rx_tick   = HAL_GetTick();
    brake_last_rx_tick = HAL_GetTick();

    CAN_FilterTypeDef fc;

    fc.FilterMode           = CAN_FILTERMODE_IDLIST;
    fc.FilterScale          = CAN_FILTERSCALE_32BIT;
    fc.FilterFIFOAssignment = CAN_RX_FIFO0;
    fc.FilterActivation     = ENABLE;
    fc.SlaveStartFilterBank = 27;

    static const uint16_t ids[] = {
        0x00A,   /* APU_STATE_MISSION  */
        0x065,   /* DASH_BRAKE         */
        0x320,   /* VCU_BOOLS          */
        0x324,   /* VCU_SERVO_CONTROL  */
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

    /* Default RX state */
    memset(&can_apu_state, 0, sizeof(can_apu_state));
    memset(&can_vcu_bools, 0, sizeof(can_vcu_bools));
    memset(&can_vcu_servo, 0, sizeof(can_vcu_servo));
    memset(&can_dash_brake, 0, sizeof(can_dash_brake));
    memset(&can_tx_asb, 0, sizeof(can_tx_asb));
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

    /* Wait for free mailbox — max 5ms */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        if ((HAL_GetTick() - start) > 5U)
        {
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

/* ── CAN Senders ── */
void CAN_SendAsbStatus(void)
{
    can_tx_asb.asms_state             = SYS_GetASMS();
    can_tx_asb.tsms_out               = !SYS_GetTSMS();
    can_tx_asb.initial_checked        = (IC_GetState() == IC_NOTIFY_APU);
    can_tx_asb.service_brake_status   = ServiceBrake_State();
    can_tx_asb.ebs_status             = EBS_State(); 
    can_tx_asb.initial_check_step     = (uint8_t)IC_GetState();
    can_tx_asb.monitor_tank_pressure  = Sensors_TankPressureValid(); /*TODO: Implement monitoring may */
    can_tx_asb.monitor_brake_pressure = true;   /* TODO: monitoring module */
    can_tx_asb.monitor_apu            = CAN_IsAPUAlive(); /*TODO: Check after make the function */
    can_tx_asb.ebs_tank_pressure      = (uint16_t)(Sensors_GetTankPressure1() * 100.0f);

    can_mcu_asb_pack(tx_data, &can_tx_asb, sizeof(tx_data));
    CAN_SendStdMessage(CAN_MCU_ASB_FRAME_ID, CAN_MCU_ASB_LENGTH, tx_data);
}

void CAN_ProcessRxMessage(void)
{
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,
                              &msg_header_rx, rx_data) != HAL_OK)
    {
        Error_Handler();
    }

    switch (msg_header_rx.StdId)
    {
        case CAN_MCU_DASH_BRAKE_FRAME_ID:
            can_mcu_dash_brake_unpack(&can_dash_brake, rx_data, msg_header_rx.DLC);
            brake_pressure_front = can_mcu_dash_brake_brake_pressure_front_decode(can_dash_brake.brake_pressure_front);
            brake_pressure_rear  = can_mcu_dash_brake_brake_pressure_rear_decode(can_dash_brake.brake_pressure_rear);
            
            /* Monitor if brake pressure is alive */
            brake_last_rx_tick = HAL_GetTick();
            break;

        case CAN_MCU_APU_STATE_MISSION_FRAME_ID:
            can_mcu_apu_state_mission_unpack(&can_apu_state, rx_data, msg_header_rx.DLC);
            
            /*Monitor if APU is alive*/
            apu_last_rx_tick = HAL_GetTick();
            break;

        case CAN_MCU_VCU_SERVO_CONTROL_FRAME_ID:
            can_mcu_vcu_servo_control_unpack(&can_vcu_servo, rx_data, msg_header_rx.DLC);
            break;

        case CAN_MCU_VCU_BOOLS_FRAME_ID:
            can_mcu_vcu_bools_unpack(&can_vcu_bools, rx_data, msg_header_rx.DLC);
            break;

        default:
            break;
    }
}

/* ── RX Getters — APU ── */

uint8_t CAN_GetASState(void)
{
    return can_apu_state.as_state;
}

uint8_t CAN_GetASMission(void)
{
    return can_apu_state.as_mission;
}

bool CAN_GetASSetFinished(void)
{
    return can_apu_state.as_set_finished;
}

/* ── RX Getters — VCU ── */

uint8_t CAN_GetVCUMode(void)
{
    return can_vcu_bools.mode;
}

int CAN_GetServoCommand(void)
{
    return can_vcu_servo.servo_control;
}

/* ── RX Getters — DASH ── */

float CAN_GetBrakePressureFront(void)
{
    return brake_pressure_front;
}

float CAN_GetBrakePressureRear(void)
{
    return brake_pressure_rear;
}

/* ── APU Alive ── */

bool CAN_IsAPUAlive(void)
{
    return (HAL_GetTick() - apu_last_rx_tick < 480U);
}

/* ── Brake Pressure Alive ── */

bool CAN_IsBrakePressureAlive(void)
{
    return (HAL_GetTick() - brake_last_rx_tick < 400U);
}

