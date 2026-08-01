#include "main.h"
#include "bsp_config.h"
#include "can_motor.h"
#include "safety.h"
#include <string.h>

/* CAN handles now in main.c via CubeMX (hcan1/hcan2), aliased in main.h */

/* Per-CAN TX buffers (eliminates shared-buffer race condition) */
static CAN_TxHeaderTypeDef g_can1_tx_header;
static uint8_t g_can1_tx_data[8];
static CAN_TxHeaderTypeDef g_can2_tx_header;
static uint8_t g_can2_tx_data[8];

#define CAN_TX_TIMEOUT_MS     2
#define RAD_TO_MDEG           57295.78f
static volatile uint32_t g_can1_tx_stuck_ts = 0;
static volatile uint32_t g_can2_tx_stuck_ts = 0;

/* Per-CAN RX buffers */
static CAN_RxHeaderTypeDef g_can1_rx_header;
static uint8_t g_can1_rx_data[8];
static CAN_RxHeaderTypeDef g_can2_rx_header;
static uint8_t g_can2_rx_data[8];

/* ========== Float ↔ uint helpers ========== */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - x_min) * ((float)((1u << bits) - 1)) / span);
}

static float uint_to_float(int x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (float)x / ((float)((1u << bits) - 1)) * span + x_min;
}

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

    status->position = (int32_t)(uint_to_float(p_raw, CG_P_MIN, CG_P_MAX, 16) * RAD_TO_MDEG);
    status->velocity = (int32_t)(uint_to_float(v_raw, CG_V_MIN, CG_V_MAX, 16) * RAD_TO_MDEG);
    status->torque   = (int32_t)(uint_to_float(t_raw, CG_T_MIN, CG_T_MAX, 16) * 1000.0f);
    status->temperature = (uint16_t)(data[6] * 10);
    status->fault_code = (uint16_t)data[7];
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

    status->position = (int32_t)(uint_to_float(p_raw, RS01_P_MIN, RS01_P_MAX, 16) * RAD_TO_MDEG);
    status->velocity = (int32_t)(uint_to_float(v_raw, RS01_V_MIN, RS01_V_MAX, RS01_V_BITS) * RAD_TO_MDEG);
    status->torque   = (int32_t)(uint_to_float(t_raw, RS01_T_MIN, RS01_T_MAX, RS01_T_BITS) * 1000.0f);
    status->temperature = (uint16_t)(data[6] * 10);
    status->fault_code = (uint16_t)data[7];
}

static void rs01_mit_pack_special(uint32_t motor_id, uint8_t cmd, uint8_t *data)
{
    memset(data, 0xFF, 8);
    data[7] = cmd;
}

/* ========== CAN Mode Switch ========== */
uint8_t can_set_mode(CAN_HandleTypeDef *hcan, uint32_t mode)
{
    HAL_CAN_Stop(hcan);
    hcan->Init.Mode = mode;
    if (HAL_CAN_Init(hcan) != HAL_OK) {
        return 1;
    }
    return (HAL_CAN_Start(hcan) != HAL_OK) ? 1 : 0;
}

/* ========== CAN Init ========== */
void can_motor_init(void)
{
    /* CAN1: CyberGear motors, 1Mbps, NORMAL mode */
    g_can1_handle.Instance = CAN1;
    g_can1_handle.Init.Prescaler = 3;
    g_can1_handle.Init.Mode = CAN_MODE_NORMAL;
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

    /* CAN2: RS01 motors, 1Mbps, NORMAL mode */
    g_can2_handle.Instance = CAN2;
    g_can2_handle.Init.Prescaler = 3;
    g_can2_handle.Init.Mode = CAN_MODE_NORMAL;
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

    /* CAN1 Filter: Bank 0 → CyberGear feedback IDs 0x200~0x207
       (CyberGear: feedback ID = 0x200 + motor_id, motor_id = 0x01/0x02) */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = (0x200 << 5);
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0xFF80;  /* Mask: match 0x200~0x27F (top 5 bits) */
    sFilterConfig.FilterMaskIdLow = 0x0000;
    HAL_CAN_ConfigFilter(&g_can1_handle, &sFilterConfig);

    /* CAN2 Filter: Bank 14 → RS01 feedback IDs 0x10~0x1F
       (RS01: feedback ID = motor_id, motor_id = 0x10~0x13) */
    sFilterConfig.FilterBank = 14;
    sFilterConfig.FilterIdHigh = (MOTOR_ARM_0_ID << 5);
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0xFF00;  /* Mask: match 0x10~0x1F (top 8 bits) */
    sFilterConfig.FilterMaskIdLow = 0x0000;
    HAL_CAN_ConfigFilter(&g_can2_handle, &sFilterConfig);

    /* Start CAN1 */
    HAL_CAN_Start(&g_can1_handle);
    HAL_CAN_ActivateNotification(&g_can1_handle, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* Start CAN2 */
    HAL_CAN_Start(&g_can2_handle);
    HAL_CAN_ActivateNotification(&g_can2_handle, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* RS01 motors: auto-enable on power-up (0xFC) */
    HAL_Delay(10);
    for (uint32_t id = MOTOR_ARM_0_ID; id <= MOTOR_ARM_3_ID; id++) {
        rs01_mit_enable(&g_can2_handle, id);
        HAL_Delay(2);
    }
}

/* ========== CAN Send ========== */
void can_motor_send_command(CAN_HandleTypeDef *hcan, uint32_t motor_id,
                            int32_t target, uint8_t mode)
{
    uint32_t tx_mailbox;
    float torque_nm = (float)target / 1000.0f;
    HAL_StatusTypeDef status;

    if (hcan->Instance == CAN1) {
        memset(&g_can1_tx_header, 0, sizeof(g_can1_tx_header));
        g_can1_tx_header.DLC = CAN_MIT_FRAME_LEN;
        g_can1_tx_header.IDE = CAN_ID_STD;
        g_can1_tx_header.RTR = CAN_RTR_DATA;
        g_can1_tx_header.StdId = motor_id & 0x7FF;

        if (mode == CTRL_MODE_TORQUE) {
            cybergear_mit_pack_command(motor_id, 0.0f, 0.0f, 0.0f, torque_nm, g_can1_tx_data);
        } else {
            float pos_rad = (float)target / RAD_TO_MDEG;
            cybergear_mit_pack_command(motor_id, pos_rad, 0.0f, 10.0f, 0.5f, g_can1_tx_data);
        }
        status = HAL_CAN_AddTxMessage(hcan, &g_can1_tx_header, g_can1_tx_data, &tx_mailbox);
        if (status == HAL_OK) {
            g_can1_tx_stuck_ts = 0;
        } else {
            uint32_t now = HAL_GetTick();
            if (g_can1_tx_stuck_ts == 0) g_can1_tx_stuck_ts = now;
            if (now - g_can1_tx_stuck_ts >= CAN_TX_TIMEOUT_MS) {
                g_safety_state.fault_code |= FAULT_CAN1_TIMEOUT;
            }
        }
    } else {
        memset(&g_can2_tx_header, 0, sizeof(g_can2_tx_header));
        g_can2_tx_header.DLC = CAN_MIT_FRAME_LEN;
        g_can2_tx_header.IDE = CAN_ID_STD;
        g_can2_tx_header.RTR = CAN_RTR_DATA;
        g_can2_tx_header.StdId = motor_id & 0x7FF;

        if (mode == CTRL_MODE_TORQUE) {
            rs01_mit_pack_command(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, torque_nm, g_can2_tx_data);
        } else {
            float pos_rad = (float)target / RAD_TO_MDEG;
            rs01_mit_pack_command(motor_id, pos_rad, 0.0f, 10.0f, 0.5f, 0.0f, g_can2_tx_data);
        }
        status = HAL_CAN_AddTxMessage(hcan, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);
        if (status == HAL_OK) {
            g_can2_tx_stuck_ts = 0;
        } else {
            uint32_t now = HAL_GetTick();
            if (g_can2_tx_stuck_ts == 0) g_can2_tx_stuck_ts = now;
            if (now - g_can2_tx_stuck_ts >= CAN_TX_TIMEOUT_MS) {
                g_safety_state.fault_code |= FAULT_CAN2_TIMEOUT;
            }
        }
    }
}

/* ========== CAN Receive (FMP0 check, not FF0) ========== */
void can_motor_receive_status(CAN_HandleTypeDef *hcan, JointStatus_t *status)
{
    status->joint_id = 0;

    /* Check FIFO Message Pending (FMP0), not FIFO Full (FF0) */
    if (hcan->Instance == CAN1) {
        if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) == 0) {
            return;
        }
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &g_can1_rx_header, g_can1_rx_data) != HAL_OK) {
            return;
        }

        uint32_t can_id = g_can1_rx_header.StdId;
        /* CyberGear: feedback ID = 0x200 + motor_id (0x01/0x02 → 0x201/0x202) */
        if (can_id == (0x200 + MOTOR_LEG_0_ID)) {
            status->joint_id = MOTOR_LEG_0_ID;
        } else if (can_id == (0x200 + MOTOR_LEG_1_ID)) {
            status->joint_id = MOTOR_LEG_1_ID;
        } else {
            return;
        }
        cybergear_mit_unpack_reply(g_can1_rx_data, status);
    } else {
        if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) == 0) {
            return;
        }
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &g_can2_rx_header, g_can2_rx_data) != HAL_OK) {
            return;
        }

        uint32_t can_id = g_can2_rx_header.StdId;
        if (can_id >= MOTOR_ARM_0_ID && can_id <= MOTOR_ARM_3_ID) {
            status->joint_id = (uint8_t)can_id;
        } else {
            return;
        }
        rs01_mit_unpack_reply(g_can2_rx_data, status);
    }
}

/* ========== RS01 Special Commands ========== */
void rs01_mit_enable(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t tx_mailbox;
    memset(&g_can2_tx_header, 0, sizeof(g_can2_tx_header));
    g_can2_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can2_tx_header.IDE = CAN_ID_STD;
    g_can2_tx_header.RTR = CAN_RTR_DATA;
    g_can2_tx_header.StdId = motor_id & 0x7FF;

    rs01_mit_pack_special(motor_id, RS01_CMD_ENABLE, g_can2_tx_data);
    HAL_CAN_AddTxMessage(hcan, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);
}

void rs01_mit_disable(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t tx_mailbox;
    memset(&g_can2_tx_header, 0, sizeof(g_can2_tx_header));
    g_can2_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can2_tx_header.IDE = CAN_ID_STD;
    g_can2_tx_header.RTR = CAN_RTR_DATA;
    g_can2_tx_header.StdId = motor_id & 0x7FF;

    rs01_mit_pack_special(motor_id, RS01_CMD_DISABLE, g_can2_tx_data);
    HAL_CAN_AddTxMessage(hcan, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);
}

/* ========== Emergency Disable ========== */
void can_motor_disable_all(void)
{
    uint32_t tx_mailbox;

    /* Cybergear: MIT disable frame (pos=0,vel=0,kp=0,kd=0 = zero torque) */
    memset(&g_can1_tx_header, 0, sizeof(g_can1_tx_header));
    g_can1_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can1_tx_header.IDE = CAN_ID_STD;
    g_can1_tx_header.RTR = CAN_RTR_DATA;

    g_can1_tx_header.StdId = MOTOR_LEG_0_ID;
    cybergear_mit_pack_command(MOTOR_LEG_0_ID, 0.0f, 0.0f, 0.0f, 0.0f, g_can1_tx_data);
    HAL_CAN_AddTxMessage(&g_can1_handle, &g_can1_tx_header, g_can1_tx_data, &tx_mailbox);

    g_can1_tx_header.StdId = MOTOR_LEG_1_ID;
    cybergear_mit_pack_command(MOTOR_LEG_1_ID, 0.0f, 0.0f, 0.0f, 0.0f, g_can1_tx_data);
    HAL_CAN_AddTxMessage(&g_can1_handle, &g_can1_tx_header, g_can1_tx_data, &tx_mailbox);

    /* RS01: MIT disable frame (0xFD) */
    memset(&g_can2_tx_header, 0, sizeof(g_can2_tx_header));
    g_can2_tx_header.DLC = CAN_MIT_FRAME_LEN;
    g_can2_tx_header.IDE = CAN_ID_STD;
    g_can2_tx_header.RTR = CAN_RTR_DATA;

    g_can2_tx_header.StdId = MOTOR_ARM_0_ID;
    rs01_mit_pack_special(MOTOR_ARM_0_ID, RS01_CMD_DISABLE, g_can2_tx_data);
    HAL_CAN_AddTxMessage(&g_can2_handle, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);

    g_can2_tx_header.StdId = MOTOR_ARM_1_ID;
    rs01_mit_pack_special(MOTOR_ARM_1_ID, RS01_CMD_DISABLE, g_can2_tx_data);
    HAL_CAN_AddTxMessage(&g_can2_handle, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);

    g_can2_tx_header.StdId = MOTOR_ARM_2_ID;
    rs01_mit_pack_special(MOTOR_ARM_2_ID, RS01_CMD_DISABLE, g_can2_tx_data);
    HAL_CAN_AddTxMessage(&g_can2_handle, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);

    g_can2_tx_header.StdId = MOTOR_ARM_3_ID;
    rs01_mit_pack_special(MOTOR_ARM_3_ID, RS01_CMD_DISABLE, g_can2_tx_data);
    HAL_CAN_AddTxMessage(&g_can2_handle, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);
}

void can_motor_disable_bus(CAN_HandleTypeDef *hcan)
{
    uint32_t tx_mailbox;

    if (hcan->Instance == CAN1) {
        memset(&g_can1_tx_header, 0, sizeof(g_can1_tx_header));
        g_can1_tx_header.DLC = CAN_MIT_FRAME_LEN;
        g_can1_tx_header.IDE = CAN_ID_STD;
        g_can1_tx_header.RTR = CAN_RTR_DATA;

        g_can1_tx_header.StdId = MOTOR_LEG_0_ID;
        cybergear_mit_pack_command(MOTOR_LEG_0_ID, 0.0f, 0.0f, 0.0f, 0.0f, g_can1_tx_data);
        HAL_CAN_AddTxMessage(hcan, &g_can1_tx_header, g_can1_tx_data, &tx_mailbox);

        g_can1_tx_header.StdId = MOTOR_LEG_1_ID;
        cybergear_mit_pack_command(MOTOR_LEG_1_ID, 0.0f, 0.0f, 0.0f, 0.0f, g_can1_tx_data);
        HAL_CAN_AddTxMessage(hcan, &g_can1_tx_header, g_can1_tx_data, &tx_mailbox);
    } else {
        memset(&g_can2_tx_header, 0, sizeof(g_can2_tx_header));
        g_can2_tx_header.DLC = CAN_MIT_FRAME_LEN;
        g_can2_tx_header.IDE = CAN_ID_STD;
        g_can2_tx_header.RTR = CAN_RTR_DATA;

        g_can2_tx_header.StdId = MOTOR_ARM_0_ID;
        rs01_mit_pack_special(MOTOR_ARM_0_ID, RS01_CMD_DISABLE, g_can2_tx_data);
        HAL_CAN_AddTxMessage(hcan, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);

        g_can2_tx_header.StdId = MOTOR_ARM_1_ID;
        rs01_mit_pack_special(MOTOR_ARM_1_ID, RS01_CMD_DISABLE, g_can2_tx_data);
        HAL_CAN_AddTxMessage(hcan, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);

        g_can2_tx_header.StdId = MOTOR_ARM_2_ID;
        rs01_mit_pack_special(MOTOR_ARM_2_ID, RS01_CMD_DISABLE, g_can2_tx_data);
        HAL_CAN_AddTxMessage(hcan, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);

        g_can2_tx_header.StdId = MOTOR_ARM_3_ID;
        rs01_mit_pack_special(MOTOR_ARM_3_ID, RS01_CMD_DISABLE, g_can2_tx_data);
        HAL_CAN_AddTxMessage(hcan, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);
    }
}

/* ========== CyberGear Current Mode (CMD=0x01) ========== */
void cybergear_send_current_cmd(CAN_HandleTypeDef *hcan, uint32_t motor_id, float current_a)
{
    uint32_t tx_mailbox;
    uint8_t data[8];
    memset(data, 0, 8);
    data[0] = 0x01;
    memcpy(&data[1], &current_a, 4);

    CAN_TxHeaderTypeDef tx_header;
    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.DLC = 8;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.StdId = motor_id & 0x7FF;

    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}