#include "can_motor.h"
#include "bsp_config.h"
#include <string.h>

CAN_HandleTypeDef g_can1_handle;
CAN_HandleTypeDef g_can2_handle;

/* CAN TX header */
static CAN_TxHeaderTypeDef g_can_tx_header;
static uint8_t g_can_tx_data[8];

/* CAN RX header */
static CAN_RxHeaderTypeDef g_can_rx_header;
static uint8_t g_can_rx_data[8];

/* ========== CyberGear MIT Frame Packing ========== */
static void cybergear_mit_pack_command(uint32_t motor_id,
                                       float pos, float vel, float kp, float kd,
                                       uint8_t *data)
{
    uint16_t p_int = (uint16_t)float_to_uint(pos, CG_P_MIN, CG_P_MAX, 16);
    uint16_t v_int = (uint16_t)float_to_uint(vel, CG_V_MIN, CG_V_MAX, 16);
    uint16_t kp_int = (uint16_t)float_to_uint(kp, CG_KP_MIN, CG_KP_MAX, 16);
    uint16_t kd_int = (uint16_t)float_to_uint(kd, CG_KD_MIN, CG_KD_MAX, 16);

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)(v_int >> 8);
    data[3] = (uint8_t)(v_int & 0xFF);
    data[4] = (uint8_t)(kp_int >> 8);
    data[5] = (uint8_t)(kp_int & 0xFF);
    data[6] = (uint8_t)(kd_int >> 8);
    data[7] = (uint8_t)(kd_int & 0xFF);
}

static void cybergear_mit_unpack_reply(const uint8_t *data, JointStatus_t *status)
{
    uint16_t p_raw = ((uint16_t)data[0] << 8) | data[1];
    uint16_t v_raw = ((uint16_t)data[2] << 8) | data[3];
    uint16_t t_raw = ((uint16_t)data[4] << 8) | data[5];

    status->position = (int32_t)(uint_to_float(p_raw, CG_P_MIN, CG_P_MAX, 16) * 1000.0f);
    status->velocity = (int32_t)(uint_to_float(v_raw, CG_V_MIN, CG_V_MAX, 16) * 1000.0f);
    status->torque   = (int32_t)(uint_to_float(t_raw, CG_T_MIN, CG_T_MAX, 16) * 1000.0f);
    status->temperature = (uint16_t)(((uint16_t)data[6] << 8) | data[7]);
    status->fault_code = 0;
}

/* ========== RS01 MIT Frame Packing ========== */
static void rs01_mit_pack_command(uint32_t motor_id,
                                  float pos, float vel, float kp, float kd, float torque,
                                  uint8_t *data)
{
    uint16_t p_int = (uint16_t)float_to_uint(pos, RS01_P_MIN, RS01_P_MAX, 16);
    uint16_t v_int = (uint16_t)float_to_uint(vel, RS01_V_MIN, RS01_V_MAX, RS01_V_BITS);
    uint16_t kp_int = (uint16_t)float_to_uint(kp, RS01_KP_MIN, RS01_KP_MAX, RS01_KP_BITS);
    uint16_t kd_int = (uint16_t)float_to_uint(kd, RS01_KD_MIN, RS01_KD_MAX, RS01_KD_BITS);
    uint16_t t_int = (uint16_t)float_to_uint(torque, RS01_T_MIN, RS01_T_MAX, RS01_T_BITS);

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)((v_int >> 4) & 0xFF);
    data[3] = (uint8_t)(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    data[4] = (uint8_t)(kp_int & 0xFF);
    data[5] = (uint8_t)((kd_int >> 4) & 0xFF);
    data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    data[7] = (uint8_t)(t_int & 0xFF);
}

static void rs01_mit_unpack_reply(const uint8_t *data, JointStatus_t *status)
{
    uint16_t p_raw = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v_raw = (uint16_t)(((uint16_t)data[3] << 4) | ((data[4] >> 4) & 0x0F));
    uint16_t t_raw = (uint16_t)(((uint16_t)(data[4] & 0x0F) << 8) | data[5]);
    uint16_t temp_raw = ((uint16_t)data[6] << 8) | data[7];

    status->position = (int32_t)(uint_to_float(p_raw, RS01_P_MIN, RS01_P_MAX, 16) * 1000.0f);
    status->velocity = (int32_t)(uint_to_float(v_raw, RS01_V_MIN, RS01_V_MAX, RS01_V_BITS) * 1000.0f);
    status->torque   = (int32_t)(uint_to_float(t_raw, RS01_T_MIN, RS01_T_MAX, RS01_T_BITS) * 1000.0f);
    status->temperature = temp_raw;
    status->fault_code = 0;
}

static void rs01_mit_pack_special(uint32_t motor_id, uint8_t cmd, uint8_t *data)
{
    memset(data, 0xFF, 8);
    data[7] = cmd;
}

/* ========== CAN Init ========== */
void can_motor_init(void)
{
    /* CAN1 Init (CyberGear, 1Mbps) - SILENT mode for no-motor debugging
     * Change to CAN_MODE_NORMAL when motors are connected with 120ohm terminator */
    g_can1_handle.Instance = CAN1;
    g_can1_handle.Init.Prescaler = 3;
    g_can1_handle.Init.Mode = CAN_MODE_SILENT;
    g_can1_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    g_can1_handle.Init.TimeSeg1 = CAN_BS1_10TQ;
    g_can1_handle.Init.TimeSeg2 = CAN_BS2_3TQ;
    g_can1_handle.Init.TimeTriggeredMode = DISABLE;
    g_can1_handle.Init.AutoBusOff = ENABLE;
    g_can1_handle.Init.AutoWakeUp = DISABLE;
    g_can1_handle.Init.AutoRetransmission = ENABLE;
    g_can1_handle.Init.ReceiveFifoLocked = DISABLE;
    g_can1_handle.Init.TransmitFifoPriority = ENABLE;
    HAL_CAN_Init(&g_can1_handle);

    /* CAN2 Init (RS01, 1Mbps) - disabled for now, enable after CAN1 verified
    g_can2_handle.Instance = CAN2;
    g_can2_handle.Init.Prescaler = 3;
    g_can2_handle.Init.Mode = CAN_MODE_SILENT;
    g_can2_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    g_can2_handle.Init.TimeSeg1 = CAN_BS1_10TQ;
    g_can2_handle.Init.TimeSeg2 = CAN_BS2_3TQ;
    g_can2_handle.Init.TimeTriggeredMode = DISABLE;
    g_can2_handle.Init.AutoBusOff = ENABLE;
    g_can2_handle.Init.AutoWakeUp = DISABLE;
    g_can2_handle.Init.AutoRetransmission = ENABLE;
    g_can2_handle.Init.ReceiveFifoLocked = DISABLE;
    g_can2_handle.Init.TransmitFifoPriority = ENABLE;
    HAL_CAN_Init(&g_can2_handle);
    */

    /* CAN Filter: accept all standard frames */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    /* CAN1: use Filter Bank 0 */
    sFilterConfig.FilterBank = 0;
    HAL_CAN_ConfigFilter(&g_can1_handle, &sFilterConfig);

    /* Enable CAN1 */
    HAL_CAN_Start(&g_can1_handle);
    HAL_CAN_ActivateNotification(&g_can1_handle, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* Enable NVIC for CAN1 RX interrupts */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

/* ========== CAN Send ========== */
void can_motor_send_command(CAN_HandleTypeDef *hcan, uint32_t motor_id,
                            int32_t target, uint8_t mode)
{
    uint32_t tx_mailbox;
    float pos, vel, kp, kd, torque;

    float torque_nm = (float)target / 1000.0f;

    memset(&g_can_tx_header, 0, sizeof(g_can_tx_header));
    g_can_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can_tx_header.IDE = CAN_ID_STD;
    g_can_tx_header.RTR = CAN_RTR_DATA;
    g_can_tx_header.StdId = motor_id & 0x7FF;

    if (hcan->Instance == CAN1) {
        cybergear_mit_pack_command(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, g_can_tx_data);
    } else {
        rs01_mit_pack_command(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, torque_nm, g_can_tx_data);
    }

    HAL_CAN_AddTxMessage(hcan, &g_can_tx_header, g_can_tx_data, &tx_mailbox);
}

/* ========== CAN Receive ========== */
void can_motor_receive_status(CAN_HandleTypeDef *hcan, JointStatus_t *status)
{
    status->joint_id = 0;

    if (!__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FF0)) {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &g_can_rx_header, g_can_rx_data) != HAL_OK) {
        return;
    }

    uint32_t can_id = g_can_rx_header.StdId;

    if (hcan->Instance == CAN1) {
        if (can_id == MOTOR_LEG_0_ID) {
            status->joint_id = MOTOR_LEG_0_ID;
        } else if (can_id == MOTOR_LEG_1_ID) {
            status->joint_id = MOTOR_LEG_1_ID;
        } else {
            return;
        }
        cybergear_mit_unpack_reply(g_can_rx_data, status);
    } else {
        if (can_id >= MOTOR_ARM_0_ID && can_id <= MOTOR_ARM_3_ID) {
            status->joint_id = (uint8_t)can_id;
        } else {
            return;
        }
        rs01_mit_unpack_reply(g_can_rx_data, status);
    }
}

/* ========== RS01 Special Commands ========== */
void rs01_mit_enable(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t tx_mailbox;
    memset(&g_can_tx_header, 0, sizeof(g_can_tx_header));
    g_can_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can_tx_header.IDE = CAN_ID_STD;
    g_can_tx_header.RTR = CAN_RTR_DATA;
    g_can_tx_header.StdId = motor_id & 0x7FF;

    rs01_mit_pack_special(motor_id, RS01_CMD_ENABLE, g_can_tx_data);
    HAL_CAN_AddTxMessage(hcan, &g_can_tx_header, g_can_tx_data, &tx_mailbox);
}

void rs01_mit_disable(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t tx_mailbox;
    memset(&g_can_tx_header, 0, sizeof(g_can_tx_header));
    g_can_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can_tx_header.IDE = CAN_ID_STD;
    g_can_tx_header.RTR = CAN_RTR_DATA;
    g_can_tx_header.StdId = motor_id & 0x7FF;

    rs01_mit_pack_special(motor_id, RS01_CMD_DISABLE, g_can_tx_data);
    HAL_CAN_AddTxMessage(hcan, &g_can_tx_header, g_can_tx_data, &tx_mailbox);
}

void can_motor_disable_all(void)
{
    uint32_t tx_mailbox;
    uint8_t disable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    memset(&g_can_tx_header, 0, sizeof(g_can_tx_header));
    g_can_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can_tx_header.IDE = CAN_ID_STD;
    g_can_tx_header.RTR = CAN_RTR_DATA;

    g_can_tx_header.StdId = MOTOR_LEG_0_ID;
    HAL_CAN_AddTxMessage(&g_can1_handle, &g_can_tx_header, disable_data, &tx_mailbox);
    g_can_tx_header.StdId = MOTOR_LEG_1_ID;
    HAL_CAN_AddTxMessage(&g_can1_handle, &g_can_tx_header, disable_data, &tx_mailbox);
}

void can_motor_disable_bus(CAN_HandleTypeDef *hcan)
{
    uint32_t tx_mailbox;
    uint8_t disable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    memset(&g_can_tx_header, 0, sizeof(g_can_tx_header));
    g_can_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can_tx_header.IDE = CAN_ID_STD;
    g_can_tx_header.RTR = CAN_RTR_DATA;

    if (hcan->Instance == CAN1) {
        g_can_tx_header.StdId = MOTOR_LEG_0_ID;
        HAL_CAN_AddTxMessage(hcan, &g_can_tx_header, disable_data, &tx_mailbox);
        g_can_tx_header.StdId = MOTOR_LEG_1_ID;
        HAL_CAN_AddTxMessage(hcan, &g_can_tx_header, disable_data, &tx_mailbox);
    }
}
