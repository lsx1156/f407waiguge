#include "main.h"
#include "bsp_config.h"
#include "can_motor.h"
#include "safety.h"
#include <string.h>
#include <stdio.h>

/* v1.6.2: ABO 助力力矩 (定义在 control_isr.c) */
extern int32_t g_abo_assist_torque[6];

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

/* Forward declaration for online status tracking */
static void can_motor_mark_online(CAN_HandleTypeDef *hcan, uint32_t can_id);

/* Forward declarations (defined later in file) */
static uint8_t can_tx_ext_frame(CAN_HandleTypeDef *hcan, uint32_t ext_id, const uint8_t *data, uint8_t dlc);
static uint8_t can_tx_std_frame(CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t dlc);

/* Forward declaration for global status array (defined later in file) */
/* v1.6.3+fix: 从 2 扩容到 6 (腿2个 + 臂4个), 防止臂部状态写越界破坏CAN2控制结构体 */
static JointStatus_t g_joint_status[6];

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

/* ========== CyberGear Extended Frame ID Packing ========== */
/* 29-bit Extended ID: mode(5bit)<<24 | data(16bit)<<8 | id(8bit) */
static uint32_t cybergear_pack_ext_id(uint8_t mode, uint16_t data, uint8_t id)
{
    return ((uint32_t)(mode & 0x1F) << 24) |
           ((uint32_t)(data & 0xFFFF) << 8) |
           ((uint32_t)(id & 0x7F));
}

/* Extract fields from 29-bit extended feedback ID (CyberGear type 2)
   Feedback frame layout (different from send frame!):
     - Bit 28~24: mode (should be 2 for feedback)
     - Bit 23~22: mode_status (0=Reset, 1=Cali, 2=Motor)
     - Bit 21~16: fault_flags  ← motor internal error bits
     - Bit 15~8:  motor CAN ID (from_motor_id)
     - Bit 7~0:   master_id low (to_master_id, parameter 0x200B) */
static void cybergear_unpack_ext_id(uint32_t ext_id, uint8_t *mode, uint8_t *motor_id,
                                     uint8_t *fault_flags, uint8_t *mode_status)
{
    *mode        = (uint8_t)((ext_id >> 24) & 0x1F);
    *mode_status = (uint8_t)((ext_id >> 22) & 0x03);
    *fault_flags = (uint8_t)((ext_id >> 16) & 0x3F);
    *motor_id    = (uint8_t)((ext_id >> 8) & 0xFF);
}

/* ========== RS01/L91 Extended Frame ID (29-bit, 出厂默认私有协议) ==========
 * ★ RS01 出厂默认 = 私有协议 = 29-bit 扩展帧 (说明书 P27~P30)
 *   仅切换到 MIT 协议后才用 11-bit 标准帧, 当前使用默认私有协议
 * ExtId = (mode << 24) | (data << 8) | motor_id
 *   mode:     bit 28-24 (5bit)  — 通信类型 (1=MIT运控, 2=反馈, 3=使能, 4=停止, 6=零位)
 *   data:     bit 23-8  (16bit) — 数据区 (MIT控制时放 torque uint16)
 *   motor_id: bit 7-0   (8bit)  — 目标电机 CAN ID
 */
static uint32_t rs01_pack_ext_id(uint8_t mode, uint16_t data, uint8_t motor_id)
{
    return ((uint32_t)(mode & 0x1F) << 24) |
           ((uint32_t)(data & 0xFFFF) << 8) |
           ((uint32_t)(motor_id & 0xFF));
}

/* 从 RS01 反馈扩展帧中解包 mode, motor_id, fault_flags, mode_status
 * ★ v1.6.8: 反馈帧 ExtId 布局 (依据 RS01 使用说明书):
 *   bit 28-24: mode (5bit) — 通信类型 (2=反馈)
 *   bit 23-22: mode_status (2bit) — 0=Reset, 1=Cali, 2=Motor
 *   bit 21-16: fault_flags (6bit) — 电机内部故障码
 *   bit 15-8:  motor CAN ID (from_motor_id)
 *   bit 7-0:   master_id low (参数 0x200B)
 * 发送帧 motor_id 在 bit 7-0, 但反馈帧 motor_id 在 bit 15-8 */
static void rs01_unpack_ext_id(uint32_t ext_id, uint8_t *mode, uint8_t *motor_id,
                                uint8_t *fault_flags, uint8_t *mode_status)
{
    *mode        = (uint8_t)((ext_id >> 24) & 0x1F);
    *mode_status = (uint8_t)((ext_id >> 22) & 0x03);
    *fault_flags = (uint8_t)((ext_id >> 16) & 0x3F);
    *motor_id    = (uint8_t)((ext_id >> 8) & 0xFF);
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
    /* v1.6.3: 温度是 int16 有符号 (data[6]高 data[7]低, 大端), 单位 0.1°C → 存成 0.1°C 数值直接即可
     * 之前: temperature = data[6]*10 (丢了 data[7]), fault_code=data[7] (把温度高字节当故障码!)
     * 修复: 正确合成 int16, 负温度截断为 0 (防止 uint16 溢出) */
    int16_t temp_raw = (int16_t)(((uint16_t)data[6] << 8) | (uint16_t)data[7]);
    status->temperature = (uint16_t)((temp_raw > 0) ? (uint16_t)temp_raw : 0);
    /* fault_code 在接收处由 EXID fault_flags 单独填充, 此处清零占位 */
    status->fault_code = 0;
}

/* ========== RS01 MIT Frame Packing (v1.6.3: 标准帧, 半字节压缩 12-bit 字段) ==========
 * RS01 协议: 标准帧 (11-bit ID = motor_id), 8 字节数据
 *   Byte0-1: Position (16-bit)
 *   Byte2:   Velocity[11:4] (12-bit 高 8 位)
 *   Byte3:   Velocity[3:0] | Kp[11:8]
 *   Byte4:   Kp[7:0]
 *   Byte5:   Kd[11:4]
 *   Byte6:   Kd[3:0] | Torque[11:8]
 *   Byte7:   Torque[7:0]
 */
static void rs01_mit_pack_command(uint32_t motor_id,
                                  float pos, float vel, float kp, float kd, float torque,
                                  uint8_t *data)
{
    uint16_t p_int  = (uint16_t)float_to_uint(pos,   RS01_P_MIN,   RS01_P_MAX,   16);
    uint16_t v_int  = (uint16_t)float_to_uint(vel,   RS01_V_MIN,   RS01_V_MAX,   RS01_V_BITS);
    uint16_t kp_int = (uint16_t)float_to_uint(kp,    RS01_KP_MIN,  RS01_KP_MAX,  RS01_KP_BITS);
    uint16_t kd_int = (uint16_t)float_to_uint(kd,    RS01_KD_MIN,  RS01_KD_MAX,  RS01_KD_BITS);
    uint16_t t_int  = (uint16_t)float_to_uint(torque, RS01_T_MIN,  RS01_T_MAX,   RS01_T_BITS);

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)((v_int >> 4) & 0xFF);
    data[3] = (uint8_t)(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    data[4] = (uint8_t)(kp_int & 0xFF);
    data[5] = (uint8_t)((kd_int >> 4) & 0xFF);
    data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    data[7] = (uint8_t)(t_int & 0xFF);
}

/* RS01 反馈帧解析 (标准帧, 应答指令1 格式)
 *   Byte0:   Motor CAN ID
 *   Byte1-2: Position (16-bit)
 *   Byte3:   Velocity[11:4] (12-bit 高 8 位)
 *   Byte4:   Velocity[3:0] | Torque[11:8]
 *   Byte5:   Torque[7:0]
 *   Byte6-7: Temperature (int16, 单位 0.1°C)
 */
static void rs01_mit_unpack_reply(const uint8_t *data, JointStatus_t *status)
{
    uint16_t p_raw = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v_raw = ((uint16_t)data[3] << 4) | ((data[4] >> 4) & 0x0F);
    uint16_t t_raw = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];

    status->position = (int32_t)(uint_to_float(p_raw, RS01_P_MIN, RS01_P_MAX, 16) * RAD_TO_MDEG);
    status->velocity = (int32_t)(uint_to_float(v_raw, RS01_V_MIN, RS01_V_MAX, RS01_V_BITS) * RAD_TO_MDEG);
    status->torque   = (int32_t)(uint_to_float(t_raw, RS01_T_MIN, RS01_T_MAX, RS01_T_BITS) * 1000.0f);
    /* v1.6.5: 匹配参考代码 — data[6]=温度(×10=0.1°C), data[7]=故障码 */
    status->temperature = (uint16_t)(data[6] * 10);
    status->fault_code = (uint16_t)data[7];
}

/* ========== RS01 扩展帧 MIT 控制 Data 打包 (16-bit 字节对齐)
 * ★ v1.6.8: 依据 RS01 使用说明书, 扩展帧 Data 格式为 16-bit 字节对齐
 *   POSITION 模式: Data = [pos_16][vel_16][Kp_16][Kd_16] (各 2 字节大端)
 *   TORQUE 模式:  Data = [torque_16][0][0][0] (torque 在 Data[0-1], 其余清零)
 *   之前的 12-bit 压缩格式是错误诊断, 实际 root cause 是 torque 放错了位置 */
static void rs01_ext_pack_data(float pos, float vel, float kp, float kd, uint8_t *data)
{
    uint16_t p_int  = (uint16_t)float_to_uint(pos, RS01_P_MIN, RS01_P_MAX, 16);
    uint16_t v_int  = (uint16_t)float_to_uint(vel, RS01_V_MIN, RS01_V_MAX, 16);
    uint16_t kp_int = (uint16_t)float_to_uint(kp,  RS01_KP_MIN, RS01_KP_MAX, 16);
    uint16_t kd_int = (uint16_t)float_to_uint(kd,  RS01_KD_MIN, RS01_KD_MAX, 16);

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)(v_int >> 8);
    data[3] = (uint8_t)(v_int & 0xFF);
    data[4] = (uint8_t)(kp_int >> 8);
    data[5] = (uint8_t)(kp_int & 0xFF);
    data[6] = (uint8_t)(kd_int >> 8);
    data[7] = (uint8_t)(kd_int & 0xFF);
}

/* RS01 扩展帧反馈 Data 解包 (16-bit 字节对齐)
 * ★ v1.6.8: 依据 RS01 使用说明书, 扩展帧反馈 Data 格式:
 *   Data[0-1]: Position (16-bit)
 *   Data[2-3]: Velocity (16-bit)
 *   Data[4-5]: Torque (16-bit)
 *   Data[6-7]: Temperature (int16, 0.1°C)
 * 之前用 12-bit 压缩解析, 导致位置/速度/力矩全部错位 */
static void rs01_ext_unpack_reply(const uint8_t *data, JointStatus_t *status)
{
    uint16_t p_raw = ((uint16_t)data[0] << 8) | data[1];
    uint16_t v_raw = ((uint16_t)data[2] << 8) | data[3];
    uint16_t t_raw = ((uint16_t)data[4] << 8) | data[5];

    status->position = (int32_t)(uint_to_float(p_raw, RS01_P_MIN, RS01_P_MAX, 16) * RAD_TO_MDEG);
    status->velocity = (int32_t)(uint_to_float(v_raw, RS01_V_MIN, RS01_V_MAX, 16) * RAD_TO_MDEG);
    status->torque   = (int32_t)(uint_to_float(t_raw, RS01_T_MIN, RS01_T_MAX, 16) * 1000.0f);
    int16_t temp_raw = (int16_t)(((uint16_t)data[6] << 8) | (uint16_t)data[7]);
    status->temperature = (uint16_t)((temp_raw > 0) ? (uint16_t)temp_raw : 0);
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
    /* CAN1: CyberGear (Leg) motors, 1Mbps, NORMAL mode */
    g_can1_handle.Instance = CAN1;
    g_can1_handle.Init.Prescaler = 3;
    g_can1_handle.Init.Mode = CAN_MODE_NORMAL;
    g_can1_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    g_can1_handle.Init.TimeSeg1 = CAN_BS1_10TQ;
    g_can1_handle.Init.TimeSeg2 = CAN_BS2_3TQ;
    g_can1_handle.Init.TimeTriggeredMode = DISABLE;
    g_can1_handle.Init.AutoBusOff = ENABLE;
    g_can1_handle.Init.AutoWakeUp = DISABLE;
    g_can1_handle.Init.AutoRetransmission = DISABLE;   /* 关闭硬件自动重传，避免邮箱长期占用导致死锁 */
    g_can1_handle.Init.ReceiveFifoLocked = DISABLE;
    g_can1_handle.Init.TransmitFifoPriority = ENABLE;
    HAL_CAN_Init(&g_can1_handle);

    /* v1.6.3: 强制确保 CAN1 不处于 Loopback 模式 (BTR.LBKM 位) */
    g_can1_handle.Instance->BTR &= ~CAN_BTR_LBKM;

    /* CAN2: RS01 (Arm) motors, 1Mbps, NORMAL mode */
    g_can2_handle.Instance = CAN2;
    g_can2_handle.Init.Prescaler = 3;
    g_can2_handle.Init.Mode = CAN_MODE_NORMAL;
    g_can2_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    g_can2_handle.Init.TimeSeg1 = CAN_BS1_10TQ;
    g_can2_handle.Init.TimeSeg2 = CAN_BS2_3TQ;
    g_can2_handle.Init.TimeTriggeredMode = DISABLE;
    g_can2_handle.Init.AutoBusOff = ENABLE;
    g_can2_handle.Init.AutoWakeUp = DISABLE;
    g_can2_handle.Init.AutoRetransmission = DISABLE;   /* 关闭硬件自动重传，避免邮箱长期占用导致死锁 */
    g_can2_handle.Init.ReceiveFifoLocked = DISABLE;
    g_can2_handle.Init.TransmitFifoPriority = ENABLE;
    HAL_CAN_Init(&g_can2_handle);

    /* v1.6.3: 强制确保 CAN2 不处于 Loopback 模式 (BTR.LBKM 位) */
    g_can2_handle.Instance->BTR &= ~CAN_BTR_LBKM;

    /* CAN1 Filter: Bank 0 → 接收所有帧 (Mask=0)
     * 软件层在 can_motor_receive_status() 中过滤电机 ID
     */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&g_can1_handle, &sFilterConfig);

    /* CAN2 Filter: Bank 14 → 接收所有帧 (Mask=0)
     * 软件层在 can_motor_receive_status() 中过滤电机 ID
     */
    sFilterConfig.FilterBank = 14;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&g_can2_handle, &sFilterConfig);

    /* Start CAN1 (polling mode, no RX interrupt)
     * v1.6.4: 在 Start 之前强制清除 LBKM(bit30) 和 SILM(bit31), 防止 Silent Loopback
     * HAL_CAN_Start 会使控制器立即按照 BTR 配置工作, 若此时 BTR 残留 LBKM/SILM,
     * 控制器进入内部回环模式 —— TX 成功(自 ACK), 但物理总线无任何动作, 电机收不到帧 */
    g_can1_handle.Instance->BTR &= ~(CAN_BTR_LBKM | CAN_BTR_SILM);
    g_can2_handle.Instance->BTR &= ~(CAN_BTR_LBKM | CAN_BTR_SILM);

    /* 打印启动前 BTR 状态, 确认 LBKM/SILM 已清零 */
    printf("[CAN] Pre-Start BTR1=0x%08lX BTR2=0x%08lX LBKM1=%lu SILM1=%lu LBKM2=%lu SILM2=%lu\r\n",
           (unsigned long)g_can1_handle.Instance->BTR,
           (unsigned long)g_can2_handle.Instance->BTR,
           (unsigned long)((g_can1_handle.Instance->BTR >> 30) & 1),
           (unsigned long)((g_can1_handle.Instance->BTR >> 31) & 1),
           (unsigned long)((g_can2_handle.Instance->BTR >> 30) & 1),
           (unsigned long)((g_can2_handle.Instance->BTR >> 31) & 1));

    HAL_CAN_Start(&g_can1_handle);
    HAL_CAN_Start(&g_can2_handle);

    /* v1.6.4: Start 后再次清除, 作为双保险 */
    g_can1_handle.Instance->BTR &= ~(CAN_BTR_LBKM | CAN_BTR_SILM);
    g_can2_handle.Instance->BTR &= ~(CAN_BTR_LBKM | CAN_BTR_SILM);

    /* CyberGear motors (CAN1): 清故障 → 使能 → 零力矩 (v1.6.3+fix3: 回退SetZero写Flash, 导致电机异常不反馈!)
     *   IDs: 0x01=左髋(L-Hip), 0x02=右髋(R-Hip) —— CAN1=双髋关节, 非髋膝
     *   ★ SetZero(CMD6)写Flash会导致CyberGear异常无反馈, CAN1_TIMEOUT. 上电零位由机械结构保证 */
    HAL_Delay(10);
    /* 电机1 (0x01 左髋): CMD=4(STOP/清故障) → CMD=3(ENABLE) → CMD=1(MIT零力矩) 恢复通信 */
    cybergear_mit_disable(&g_can1_handle, MOTOR_LEG_0_ID);  /* 第一步: 停止/清故障 (确保从故障态恢复) */
    HAL_Delay(3);
    cybergear_mit_enable(&g_can1_handle, MOTOR_LEG_0_ID);   /* 第二步: 使能进入运控模式 */
    HAL_Delay(10);                                            /* 等待电机内部状态机就绪 */
    cybergear_mit_zero_torque(&g_can1_handle, MOTOR_LEG_0_ID); /* 第三步: MIT零力矩, 建立反馈链路 */
    HAL_Delay(3);
    /* 电机2 (0x02 右髋): 同三步法 */
    cybergear_mit_disable(&g_can1_handle, MOTOR_LEG_1_ID);
    HAL_Delay(3);
    cybergear_mit_enable(&g_can1_handle, MOTOR_LEG_1_ID);
    HAL_Delay(10);
    cybergear_mit_zero_torque(&g_can1_handle, MOTOR_LEG_1_ID);
    HAL_Delay(3);

    /* RS01 motors (CAN2): ★ v1.6.8 完整初始化序列 — IDs 0x10~0x13
     * 序列: 停止/清故障 → 写参数(master_id) → 设零位 → 使能自动上报 → 使能 → 零力矩
     * ★ 必须发 0x18 (自动上报) 才有反馈帧, 否则电机不主动上报 → "NO FEEDBACK"
     * ★ torque 必须在 Data[0-1] (NOT ExtId), ExtId data=MasterID */
    for (uint32_t id = MOTOR_ARM_0_ID; id <= MOTOR_ARM_0_ID + 3; id++) {
        rs01_init_one(&g_can2_handle, id);
        HAL_Delay(2);
    }

    /* 清空使能过程中的反馈帧 (CAN1 + CAN2 都要清)
     * 防止 Reset/Cali 阶段的无效帧 (p_raw=0 → -720°假值) 进入后续 ISR 接收循环 */
    HAL_Delay(20);
    {
        CAN_RxHeaderTypeDef hdr;
        uint8_t data[8];
        while ((g_can1_handle.Instance->RF0R & 0x03) != 0) {
            HAL_CAN_GetRxMessage(&g_can1_handle, CAN_RX_FIFO0, &hdr, data);
        }
        while ((g_can2_handle.Instance->RF0R & 0x03) != 0) {
            HAL_CAN_GetRxMessage(&g_can2_handle, CAN_RX_FIFO0, &hdr, data);
        }
    }

    /* v1.6.4: BOOT 阶段 CAN2 (RS01) 即时反馈自诊断
     * 向 4 个臂电机各发 2 次零力矩 MIT 帧, 确认能否收到反馈
     * 直接给出"哪个电机有问题"的明确信息, 避免运行期 FAULT_CAN2_TIMEOUT 难排查 */
    {
        printf("[CAN2] BOOT RS01 feedback self-diagnostic:\r\n");
        uint8_t arm_ids[4] = {MOTOR_ARM_0_ID, MOTOR_ARM_0_ID+1,
                              MOTOR_ARM_0_ID+2, MOTOR_ARM_0_ID+3};
        const char *arm_names[4] = {"Arm0(L-Sho)", "Arm1(L-Elb)",
                                    "Arm2(R-Sho)", "Arm3(R-Elb)"};
        uint8_t online_cnt = 0;
        for (int m = 0; m < 4; m++) {
            uint8_t got_feedback = 0;
            /* 发 2 次零力矩帧 + 各 5ms 接收窗口 */
            for (int txr = 0; txr < 2 && !got_feedback; txr++) {
                uint8_t r = rs01_send_zero_torque(&g_can2_handle, arm_ids[m]);
                if (r != 0) {
                    printf("[CAN2]   %s (ID=0x%02X): TX ret=%d (bus open/no ACK) — 检查 CAN2 接线/收发器/电机供电\r\n",
                           arm_names[m], arm_ids[m], r);
                    break;
                }
                uint32_t w_ts = HAL_GetTick();
                while (HAL_GetTick() - w_ts < 5) {
                    JointStatus_t st;
                    can_motor_receive_status(&g_can2_handle, &st);
                    if (st.joint_id == arm_ids[m]) {
                        got_feedback = 1;
                        break;
                    }
                }
            }
            if (got_feedback) {
                online_cnt++;
                printf("[CAN2]   %s (ID=0x%02X): OK (feedback received)\r\n",
                       arm_names[m], arm_ids[m]);
            } else {
                printf("[CAN2]   %s (ID=0x%02X): NO FEEDBACK — 检查电机 CAN ID 配置(应为0x%02X)和电机使能状态\r\n",
                       arm_names[m], arm_ids[m], arm_ids[m]);
            }
            /* 清空 FIFO 残余帧, 避免干扰下个电机检测 */
            CAN_RxHeaderTypeDef hdr; uint8_t data[8];
            while ((g_can2_handle.Instance->RF0R & 0x03) != 0)
                HAL_CAN_GetRxMessage(&g_can2_handle, CAN_RX_FIFO0, &hdr, data);
        }
        printf("[CAN2] RS01 summary: %u/4 motors online\r\n", online_cnt);

        /* ★ v1.6.8fix2: 如果有电机离线, 扫描 0x00-0x1F 查找实际 CAN ID
         * 可能电机出厂 ID 未设为 0x10-0x13 */
        if (online_cnt < 4) {
            printf("[CAN2] Scanning IDs 0x00-0x1F to find offline motors...\r\n");
            for (uint8_t scan_id = 0; scan_id < 0x20; scan_id++) {
                /* 跳过已在线的 ID */
                uint8_t skip = 0;
                for (int m = 0; m < 4; m++) {
                    if (arm_ids[m] == scan_id) { skip = 1; break; }
                }
                if (skip) continue;

                uint8_t got = 0;
                rs01_send_zero_torque(&g_can2_handle, scan_id);
                uint32_t w_ts = HAL_GetTick();
                while (HAL_GetTick() - w_ts < 3) {
                    JointStatus_t st;
                    can_motor_receive_status(&g_can2_handle, &st);
                    if (st.joint_id == scan_id) { got = 1; break; }
                }
                if (got) {
                    printf("[CAN2]   ★ Found motor at ID=0x%02X (unexpected!) — 可能是出厂默认ID, 需修改代码或用工具设ID\r\n", scan_id);
                }
                /* 清 FIFO */
                CAN_RxHeaderTypeDef hdr; uint8_t data[8];
                while ((g_can2_handle.Instance->RF0R & 0x03) != 0)
                    HAL_CAN_GetRxMessage(&g_can2_handle, CAN_RX_FIFO0, &hdr, data);
            }
            printf("[CAN2] ID scan complete.\r\n");
        }

        /* v1.6.4: 打印 CAN2 错误状态寄存器, 辅助定位硬件问题
         * ESR: bit23-16=TEC(发送错误计数), bit15-8=REC(接收错误计数), bit2=BOFF, bit1=EPVF, bit0=EWF
         * TEC>0 说明发送了帧但出错(无ACK会快速+8); REC>0 说明收到了错误帧;
         * BOFF=Bus-Off; EPVF=Error Passive; EWF=Error Warning */
        uint32_t esr2 = g_can2_handle.Instance->ESR;
        printf("[CAN2] ESR=0x%08lX TEC=%lu REC=%lu BOFF=%lu EPVF=%lu EWF=%lu\r\n",
               (unsigned long)esr2,
               (unsigned long)((esr2 >> 16) & 0xFF),
               (unsigned long)((esr2 >> 8) & 0xFF),
               (unsigned long)((esr2 >> 2) & 1),
               (unsigned long)((esr2 >> 1) & 1),
               (unsigned long)(esr2 & 1));
        /* 同时打印 CAN2 MSR/TSR 看 ALST/ERR 位 */
        printf("[CAN2] MSR=0x%08lX TSR=0x%08lX RF0R=0x%08lX\r\n",
               (unsigned long)g_can2_handle.Instance->MSR,
               (unsigned long)g_can2_handle.Instance->TSR,
               (unsigned long)g_can2_handle.Instance->RF0R);
    }
}

/**
 * @brief  Detect if CAN bus has active devices by sending a test frame
 *         and checking for ACK errors / Transmit Error Counter (TEC)
 * @param  hcan: CAN handle
 * @param  test_id: CAN ID to send test frame to
 * @retval 0 = devices present on bus, 1 = no devices / bus open
 */
uint8_t can_bus_detect(CAN_HandleTypeDef *hcan, uint32_t test_id)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;
    uint32_t start_tick;
    uint8_t result = 0;

    /* Clear any pending errors first */
    hcan->Instance->ESR = 0;
    hcan->Instance->MSR = CAN_MSR_ERRI;

    /* Send an 8-byte data frame (not DLC=0, some devices ignore zero-length) */
    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.DLC = 8;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.StdId = test_id & 0x7FF;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        return 1;
    }

    /* Wait for TX completion or timeout (50ms, allow for auto-retransmission) */
    start_tick = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(hcan, tx_mailbox)) {
        if (HAL_GetTick() - start_tick >= 50) {
            result = 1;
            break;
        }
    }

    /* Read Error Status Register: check TEC and last error code */
    uint32_t esr = hcan->Instance->ESR;
    uint8_t tec = (uint8_t)((esr >> 16) & 0xFF);
    uint8_t lec = (uint8_t)((esr >> 4) & 0x07);

    /* ACK error (lec=3) or TEC > 0 means no device acknowledged */
    if (tec > 0 || lec == 3) {
        result = 1;
    }

    /* Abort the pending tx if still pending (bus open case) */
    if (result == 1) {
        HAL_CAN_AbortTxRequest(hcan, tx_mailbox);
    }

    /* Clear error flags for next operation */
    hcan->Instance->ESR = 0;

    return result;
}

/* ========== CAN Send ========== */
void can_motor_send_command(CAN_HandleTypeDef *hcan, uint32_t motor_id,
                            int32_t target, uint8_t mode)
{
    uint32_t tx_mailbox;
    float torque_nm = (float)target / 1000.0f;
    HAL_StatusTypeDef status;

    if (hcan->Instance == CAN1) {
        /* CAN1 = CyberGear (Leg): Extended Frame MIT Control (type 1) */
        memset(&g_can1_tx_header, 0, sizeof(g_can1_tx_header));
        g_can1_tx_header.DLC = CAN_MIT_FRAME_LEN;
        g_can1_tx_header.IDE = CAN_ID_EXT;
        g_can1_tx_header.RTR = CAN_RTR_DATA;

        if (mode == CTRL_MODE_TORQUE) {
            uint16_t t_uint = (uint16_t)float_to_uint(torque_nm, CG_T_MIN, CG_T_MAX, 16);
            g_can1_tx_header.ExtId = cybergear_pack_ext_id(CG_CMD_MIT_CTRL, t_uint, (uint8_t)motor_id);
            cybergear_mit_pack_command(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, g_can1_tx_data);
        } else {
            /* v1.6.2: POSITION 模式叠加 ABO 前馈力矩 */
            float pos_rad = (float)target / RAD_TO_MDEG;
            int abo_idx = (motor_id == MOTOR_LEG_0_ID) ? 0 : 1;
            float torque_ff = (float)g_abo_assist_torque[abo_idx] / 1000.0f;
            uint16_t t_uint = (uint16_t)float_to_uint(torque_ff, CG_T_MIN, CG_T_MAX, 16);
            g_can1_tx_header.ExtId = cybergear_pack_ext_id(CG_CMD_MIT_CTRL, t_uint, (uint8_t)motor_id);
            cybergear_mit_pack_command(motor_id, pos_rad, 0.0f, 10.0f, 0.5f, g_can1_tx_data);
        }
        status = HAL_CAN_AddTxMessage(hcan, &g_can1_tx_header, g_can1_tx_data, &tx_mailbox);
        if (status == HAL_OK) {
            g_can1_tx_stuck_ts = 0;
            /* 非阻塞: 不等待发送完成, 由 CAN 控制器异步发送
             * 如果需要超时检测, 可在主循环异步检查 TME 标志 */
        } else {
            uint32_t now = HAL_GetTick();
            if (g_can1_tx_stuck_ts == 0) g_can1_tx_stuck_ts = now;
            if (now - g_can1_tx_stuck_ts >= CAN_TX_TIMEOUT_MS) {
                /* 超时后 abort 所有三个邮箱，防止邮箱堆积 */
                HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
                HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
                HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);
                SAFETY_LOCK();
                g_safety_state.fault_code |= FAULT_CAN1_TIMEOUT;
                SAFETY_UNLOCK();
            }
        }
    } else {
        /* CAN2 = RS01/L91 (Arm): 扩展帧, mode=1 (MIT运控), 出厂默认私有协议
         * ★ v1.6.8fix: RS01 与 CyberGear 协议一致 — 力矩在 ExtId data 字段
         *   ExtId = (1<<24)|(torque_uint16<<8)|motor_id
         *   Data  = [pos_16][vel_16][Kp_16][Kd_16] (16-bit 字节对齐, 非 12-bit 压缩)
         *   MIT: τ = Kp×(pos-pos_act) + Kd×(vel-vel_act) + τ_ff(ExtId)
         *   TORQUE 模式:  ExtId data=torque, Data=全0 (Kp=Kd=0 → 纯力矩控制)
         *   POSITION 模式: ExtId data=0(零FF), Data=[pos][vel][Kp][Kd]
         * ★ 之前把 MasterID(0x0001) 放入 ExtId data → 电机解读为 -17 N·m → 全速反转! */
        memset(&g_can2_tx_header, 0, sizeof(g_can2_tx_header));
        g_can2_tx_header.DLC = CAN_MIT_FRAME_LEN;
        g_can2_tx_header.IDE = CAN_ID_EXT;
        g_can2_tx_header.RTR = CAN_RTR_DATA;

        if (mode == CTRL_MODE_TORQUE) {
            /* 力矩在 ExtId data 字段 (与 CyberGear 一致), Data=全0 (Kp=Kd=0) */
            uint16_t t_uint = (uint16_t)float_to_uint(torque_nm, RS01_T_MIN, RS01_T_MAX, 16);
            g_can2_tx_header.ExtId = rs01_pack_ext_id(RS01_CMD_MIT_CTRL, t_uint, (uint8_t)motor_id);
            memset(g_can2_tx_data, 0, 8);
        } else {
            /* POSITION 模式: ExtId data=0x7FFF(零力矩FF), Data=[pos][vel][Kp][Kd] */
            float pos_rad = (float)target / RAD_TO_MDEG;
            uint16_t t_zero = (uint16_t)float_to_uint(0.0f, RS01_T_MIN, RS01_T_MAX, 16);
            g_can2_tx_header.ExtId = rs01_pack_ext_id(RS01_CMD_MIT_CTRL, t_zero, (uint8_t)motor_id);
            rs01_ext_pack_data(pos_rad, 0.0f, 10.0f, 0.5f, g_can2_tx_data);
        }
        status = HAL_CAN_AddTxMessage(hcan, &g_can2_tx_header, g_can2_tx_data, &tx_mailbox);
        if (status == HAL_OK) {
            g_can2_tx_stuck_ts = 0;
            /* 非阻塞: 不等待发送完成, 由 CAN 控制器异步发送 */
        } else {
            uint32_t now = HAL_GetTick();
            if (g_can2_tx_stuck_ts == 0) g_can2_tx_stuck_ts = now;
            if (now - g_can2_tx_stuck_ts >= CAN_TX_TIMEOUT_MS) {
                /* 超时后 abort 所有三个邮箱，防止邮箱堆积 */
                HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
                HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
                HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);
                SAFETY_LOCK();
                g_safety_state.fault_code |= FAULT_CAN2_TIMEOUT;
                SAFETY_UNLOCK();
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
        /* CAN1 = CyberGear (Leg): feedback is Extended Frame, type=2, motor_id in bit 8~15 */
        if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) == 0) {
            return;
        }
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &g_can1_rx_header, g_can1_rx_data) != HAL_OK) {
            return;
        }

        uint32_t can_id;
        uint8_t cg_mode, cg_motor_id, cg_fault, cg_mod_st;
        if (g_can1_rx_header.IDE == CAN_ID_EXT) {
            can_id = g_can1_rx_header.ExtId;
            cybergear_unpack_ext_id(can_id, &cg_mode, &cg_motor_id, &cg_fault, &cg_mod_st);
            if (cg_mode == CG_CMD_FEEDBACK && (cg_motor_id == MOTOR_LEG_0_ID || cg_motor_id == MOTOR_LEG_1_ID)) {
                status->joint_id = cg_motor_id;
                can_motor_mark_online(hcan, cg_motor_id);
            } else {
                return;
            }
        } else {
            return;
        }
        /* v1.6.3+fix: 有效性判断 = (位置原始值非边界)
         *   - 移除 cg_mod_st==2 检查: can.txt CyberGear EXID bit23-8 是数据区/主机ID, 不是 mode_status
         *     旧代码误读 bit23-22, 导致所有反馈帧被误判为 Reset/Cali 态, position 永远锁死为 0
         *   - p_raw 边界(0x0000 或 0xFFFF): 编码器未初始化或总线全 0/全 1 噪声帧, 需过滤
         * 无效时: 不修改 position/velocity/torque, 仅刷新温度+故障码 */
        uint16_t p_raw_check = ((uint16_t)g_can1_rx_data[0] << 8) | (uint16_t)g_can1_rx_data[1];
        uint8_t data_valid = (p_raw_check != 0x0000) && (p_raw_check != 0xFFFF);

        if (data_valid) {
            cybergear_mit_unpack_reply(g_can1_rx_data, status);
        } else {
            /* 无效帧: 只更新温度和故障码, 不触碰位置/速度/力矩字段 (让调用方保留上帧值) */
            int16_t temp_raw = (int16_t)(((uint16_t)g_can1_rx_data[6] << 8) | (uint16_t)g_can1_rx_data[7]);
            status->temperature = (uint16_t)((temp_raw > 0) ? (uint16_t)temp_raw : 0);
        }
        status->fault_code = (uint16_t)cg_fault;
        /* Store in global status array: 无效帧时从 g_joint_status 继承上一帧有效 P/V/T */
        if (status->joint_id >= MOTOR_LEG_0_ID && status->joint_id <= MOTOR_LEG_1_ID) {
            uint8_t idx = status->joint_id - MOTOR_LEG_0_ID;
            if (!data_valid) {
                status->position = g_joint_status[idx].position;
                status->velocity = g_joint_status[idx].velocity;
                status->torque   = g_joint_status[idx].torque;
            }
            g_joint_status[idx] = *status;
        }
    } else {
        /* CAN2 = RS01/L91 (Arm): 扩展帧, 反馈 mode=2 (RS01_CMD_FEEDBACK)
         * ★ v1.6.8: 反馈帧 ExtId: bit23-22=mode_status, bit21-16=fault_flags, bit15-8=motor_id
         *   Data: 16-bit 对齐 [pos_16][vel_16][torque_16][temp_16] */
        if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) == 0) {
            return;
        }
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &g_can2_rx_header, g_can2_rx_data) != HAL_OK) {
            return;
        }

        /* RS01 默认私有协议使用扩展帧 */
        if (g_can2_rx_header.IDE != CAN_ID_EXT) {
            return;
        }

        uint8_t rs01_mode, rs01_motor_id, rs01_fault, rs01_mod_st;
        rs01_unpack_ext_id(g_can2_rx_header.ExtId, &rs01_mode, &rs01_motor_id,
                           &rs01_fault, &rs01_mod_st);

        /* 验证: mode=FEEDBACK(2) 且 motor_id 在 RS01 范围内 (0x10~0x13) */
        if (rs01_mode != RS01_CMD_FEEDBACK ||
            rs01_motor_id < MOTOR_ARM_0_ID || rs01_motor_id > MOTOR_ARM_0_ID + 3) {
            return;
        }

        status->joint_id = rs01_motor_id;
        can_motor_mark_online(hcan, rs01_motor_id);

        /* 有效性判断: mode_status=2 (Motor 运行态) 且 p_raw 非边界 */
        uint16_t p_raw_check = ((uint16_t)g_can2_rx_data[0] << 8) | (uint16_t)g_can2_rx_data[1];
        uint8_t data_valid = (p_raw_check != 0x0000) && (p_raw_check != 0xFFFF);

        if (data_valid) {
            rs01_ext_unpack_reply(g_can2_rx_data, status);
        } else {
            /* 无效帧: 不修改 P/V/T, 从全局状态继承上一帧值 */
            uint8_t idx = rs01_motor_id - MOTOR_ARM_0_ID + 2;
            status->position = g_joint_status[idx].position;
            status->velocity = g_joint_status[idx].velocity;
            status->torque   = g_joint_status[idx].torque;
            /* 温度仍从当前帧解析 */
            int16_t temp_raw = (int16_t)(((uint16_t)g_can2_rx_data[6] << 8) | (uint16_t)g_can2_rx_data[7]);
            status->temperature = (uint16_t)((temp_raw > 0) ? (uint16_t)temp_raw : 0);
        }
        /* fault_code = 电机内部故障标志 (6bit) | mode_status(高字节)
         * mode_status: 0=Reset, 1=Cali, 2=Motor — 上层可据此判断电机是否正常工作 */
        status->fault_code = (uint16_t)((rs01_mod_st << 8) | rs01_fault);

        /* Store in global status array */
        if (rs01_motor_id >= MOTOR_ARM_0_ID && rs01_motor_id <= MOTOR_ARM_0_ID + 3) {
            g_joint_status[rs01_motor_id - MOTOR_ARM_0_ID + 2] = *status;
        }
    }
}

/* ========== RS01/L91 Special Commands (扩展帧, 出厂默认私有协议) ==========
 * ★ v1.6.8fix: RS01 与 CyberGear 协议一致 — 力矩在 ExtId data 字段
 * MIT运控: mode=1,  ExtId=(1<<24)|(torque_uint16<<8)|motor_id
 *          Data=[pos_16][vel_16][Kp_16][Kd_16] (16-bit 字节对齐)
 *          TORQUE模式:  ExtId data=torque, Data=全0 (Kp=Kd=0)
 *          POSITION模式: ExtId data=0x7FFF(零FF), Data=[pos][vel][Kp][Kd]
 * 使能:    mode=3,  ExtId=(3<<24)|(master_id<<8)|motor_id,    Data=全0
 * 停止:    mode=4,  ExtId=(4<<24)|(master_id<<8)|motor_id,    Data=全0
 * 零位:    mode=6,  ExtId=(6<<24)|(master_id<<8)|motor_id,    Data=全0
 * 写参数:  mode=0x12, ExtId=(0x12<<24)|(master_id<<8)|motor_id, Data=[idx16][0][float32]
 * 自动上报: mode=0x18, ExtId=(0x18<<24)|(master_id<<8)|motor_id, Data=[period_ms16][0...]
 * 返回: 0=成功, 1=超时 */
uint8_t rs01_mit_enable(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t ext_id = rs01_pack_ext_id(RS01_CMD_ENABLE, RS01_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0};
    return can_tx_ext_frame(hcan, ext_id, data, 8);
}

uint8_t rs01_mit_disable(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t ext_id = rs01_pack_ext_id(RS01_CMD_STOP, RS01_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0};
    return can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* RS01 设置机械零位 (扩展帧 mode=6)
 * ★ v1.6.8fix: Data[0]=0x01 确认位 (说明书要求, 全0不执行) */
uint8_t rs01_mit_set_zero(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t ext_id = rs01_pack_ext_id(RS01_CMD_SET_ZERO, RS01_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0x01, 0,0,0,0,0,0,0};  /* Byte[0]=1 确认设零点 */
    return can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* ★ v1.6.8: RS01 参数写入 (扩展帧 mode=0x12)
 * Data[0-1]: 参数索引 (uint16, 小端)
 * Data[2-3]: 保留 (0)
 * Data[4-7]: 参数值 (float, 小端)
 * 返回: 0=成功, 1=超时 */
uint8_t rs01_write_param(CAN_HandleTypeDef *hcan, uint32_t motor_id, uint16_t index, float value)
{
    uint32_t ext_id = rs01_pack_ext_id(RS01_CMD_WRITE_PARAM, RS01_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(index & 0xFF);
    data[1] = (uint8_t)((index >> 8) & 0xFF);
    /* float 小端: data[4]=byte0(LSB) ... data[7]=byte3(MSB) */
    memcpy(&data[4], &value, 4);
    return can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* ★ v1.6.8fix: RS01 使能自动上报 (扩展帧 mode=0x18)
 * ★ 无此命令电机不主动发反馈帧 — 这是 "NO FEEDBACK" 的 root cause
 * ★ v1.6.8fix: data[6]=F_CMD=1 开启上报 (说明书格式, 非周期值)
 *   上报周期由参数 0x7026 单独设置 (在 rs01_init_one 中写入)
 * 返回: 0=成功, 1=超时 */
uint8_t rs01_enable_auto_report(CAN_HandleTypeDef *hcan, uint32_t motor_id, uint16_t period_ms)
{
    (void)period_ms;  /* 周期由参数 0x7026 设置, 此处仅使能 */
    uint32_t ext_id = rs01_pack_ext_id(RS01_CMD_AUTO_REPORT, RS01_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0,0,0,0,0,0,0x01,0};  /* data[6]=F_CMD=1 开启上报 */
    return can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* ★ v1.6.8fix: RS01 单电机完整初始化序列 (依据 RS01 使用说明书)
 * 序列: 停止/清故障 → 写控制器参数(掉电丢失) → 设零位 → 使能自动上报 → 使能 → 零力矩
 * ★ v1.6.8fix 补全 8 个电机控制器参数 (说明书 P24, 失效态写入)
 * 每步之间留延时确保电机内部状态机就绪
 * 返回: 0=最后一步成功, 非0=某步失败(仍继续执行后续步骤) */
uint8_t rs01_init_one(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint8_t ret = 0;

    /* 1. 停止/清故障 (mode=4) */
    rs01_mit_disable(hcan, motor_id);
    HAL_Delay(5);

    /* 2. ★ v1.6.8fix: 写电机控制器参数 (掉电丢失, 上电必写)
     *    依据 RS01 说明书 P24 参数表, 失效态写入 */
    rs01_write_param(hcan, motor_id, 0x700B, 17.0f);    /* limit_torque = 17 Nm */
    HAL_Delay(1);
    rs01_write_param(hcan, motor_id, 0x701E, 40.0f);    /* loc_kp = 40 (位置环增益) */
    HAL_Delay(1);
    rs01_write_param(hcan, motor_id, 0x701F, 6.0f);     /* spd_kp = 6 (速度环增益) */
    HAL_Delay(1);
    rs01_write_param(hcan, motor_id, 0x7020, 0.02f);    /* spd_ki = 0.02 (速度环积分) */
    HAL_Delay(1);
    rs01_write_param(hcan, motor_id, 0x7010, 0.17f);    /* cur_kp = 0.17 (电流环增益) */
    HAL_Delay(1);
    rs01_write_param(hcan, motor_id, 0x7011, 0.012f);   /* cur_ki = 0.012 (电流环积分) */
    HAL_Delay(1);
    rs01_write_param(hcan, motor_id, 0x7028, 0.0f);     /* 关 CAN 超时保护 (防初始化阶段误触发) */
    HAL_Delay(1);
    rs01_write_param(hcan, motor_id, 0x7026, 1.0f);     /* 上报周期 = 1 (×10ms = 10ms) */
    HAL_Delay(1);
    rs01_write_param(hcan, motor_id, 0x200B, (float)RS01_MASTER_ID); /* master_id */
    HAL_Delay(5);

    /* 3. ★ v1.6.8fix2: 跳过 set_zero (mode=6 写 Flash)
     *    原因: RS01 SetZero 写 Flash 会导致部分电机状态机卡死, 不再响应后续命令
     *          (与 CyberGear 同样问题). 零位由机械结构保证.
     *    若必须设零, 需单独工具设一次, 不要每次上电都写 */
    /* rs01_mit_set_zero(hcan, motor_id); -- 注释掉 */
    /* HAL_Delay(10); -- 注释掉 */

    /* 4. 使能自动上报 (mode=0x18, data[6]=F_CMD=1) — ★ 关键步骤!
     *    无此命令电机不主动发反馈帧, 导致 CAN2 一直 NO FEEDBACK
     *    上报周期已由参数 0x7026 设置
     * ★ v1.6.8fix: 增加重试 (3次), 失败则打印警告 */
    {
        uint8_t ar_ok = 0;
        for (uint8_t r = 0; r < 3; r++) {
            ret = rs01_enable_auto_report(hcan, motor_id, 10);
            if (ret == 0) { ar_ok = 1; break; }
            HAL_Delay(2);
        }
        if (!ar_ok) {
            printf("[CAN2] RS01 ID=0x%02X auto_report FAILED (ret=%d) — 将无反馈!\r\n",
                   (unsigned)motor_id, ret);
        }
    }
    HAL_Delay(5);

    /* 5. 使能电机 (mode=3) */
    uint8_t en_ok = 0;
    for (uint8_t r = 0; r < 3; r++) {
        ret = rs01_mit_enable(hcan, motor_id);
        if (ret == 0) { en_ok = 1; break; }
        HAL_Delay(5);
    }
    if (!en_ok) {
        printf("[CAN2] RS01 ID=0x%02X enable FAILED (ret=%d)\r\n", (unsigned)motor_id, ret);
    }
    HAL_Delay(10);

    /* 6. 发零力矩帧 (mode=1) — 建立通信, 消除跨复位残留目标 */
    for (uint8_t r = 0; r < 3; r++) {
        ret = rs01_send_zero_torque(hcan, motor_id);
        if (ret == 0) break;
        HAL_Delay(2);
    }
    return ret;
}

/* ========== CyberGear Special Commands (Extended Frame, direct register access) ========== */
static uint8_t can_tx_ext_frame(CAN_HandleTypeDef *hcan, uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TypeDef *can = hcan->Instance;
    uint32_t tx_mailbox;
    uint32_t start_tick;

    /* Find a free TX mailbox */
    if ((can->TSR & CAN_TSR_TME0) != 0) tx_mailbox = 0;
    else if ((can->TSR & CAN_TSR_TME1) != 0) tx_mailbox = 1;
    else if ((can->TSR & CAN_TSR_TME2) != 0) tx_mailbox = 2;
    else {
        printf("[CAN_ERR] No free TX mailbox\r\n");
        return 1;
    }

    /* Set up the mailbox registers directly */
    __IO uint32_t *tx_reg = &can->sTxMailBox[tx_mailbox].TIR;

    /* Set up TIR first (Extended ID + IDE, no TXRQ yet) */
    *tx_reg = ((ext_id & 0x1FFFFFFF) << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE;

    /* Write data low/high */
    can->sTxMailBox[tx_mailbox].TDLR = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) |
                                          ((uint32_t)data[1] << 8)  | ((uint32_t)data[0]);
    can->sTxMailBox[tx_mailbox].TDHR = ((uint32_t)data[7] << 24) | ((uint32_t)data[6] << 16) |
                                          ((uint32_t)data[5] << 8)  | ((uint32_t)data[4]);

    /* Set DLC */
    can->sTxMailBox[tx_mailbox].TDTR = dlc & 0x0F;

    /* Request transmission (last step, same as HAL) */
    *tx_reg |= CAN_TI0R_TXRQ;

    /* Wait for completion (timeout 20ms) */
    start_tick = HAL_GetTick();
    uint32_t ok_mask = (tx_mailbox == 0) ? CAN_TSR_RQCP0 :
                        (tx_mailbox == 1) ? CAN_TSR_RQCP1 : CAN_TSR_RQCP2;
    while ((can->TSR & ok_mask) == 0) {
        if (HAL_GetTick() - start_tick >= 20) {
            printf("[CAN_ERR] TX timeout on mailbox %lu, aborting\r\n", (unsigned long)tx_mailbox);
            /* Abort */
            if (tx_mailbox == 0) can->TSR = CAN_TSR_ABRQ0;
            else if (tx_mailbox == 1) can->TSR = CAN_TSR_ABRQ1;
            else can->TSR = CAN_TSR_ABRQ2;
            return 1;
        }
    }
    return 0;
}

/* ========== Standard Frame TX (RS01 uses 11-bit IDs) ========== */
static uint8_t can_tx_std_frame(CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TypeDef *can = hcan->Instance;
    uint32_t tx_mailbox;
    uint32_t start_tick;

    if ((can->TSR & CAN_TSR_TME0) != 0) tx_mailbox = 0;
    else if ((can->TSR & CAN_TSR_TME1) != 0) tx_mailbox = 1;
    else if ((can->TSR & CAN_TSR_TME2) != 0) tx_mailbox = 2;
    else {
        printf("[CAN_ERR] No free TX mailbox\r\n");
        return 1;
    }

    __IO uint32_t *tx_reg = &can->sTxMailBox[tx_mailbox].TIR;

    /* v1.6.3: 清除上一次发送的 RQCPx 标志, 防止误判 */
    uint32_t rqcp_clear = (tx_mailbox == 0) ? CAN_TSR_RQCP0 :
                          (tx_mailbox == 1) ? CAN_TSR_RQCP1 : CAN_TSR_RQCP2;
    can->TSR = ~rqcp_clear;  /* 写 0 清除对应 RQCPx 位 */

    /* Set up TIR: standard 11-bit ID + no IDE */
    *tx_reg = (std_id & 0x7FF) << CAN_TI0R_STID_Pos;

    /* Write data low/high */
    can->sTxMailBox[tx_mailbox].TDLR = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) |
                                          ((uint32_t)data[1] << 8)  | ((uint32_t)data[0]);
    can->sTxMailBox[tx_mailbox].TDHR = ((uint32_t)data[7] << 24) | ((uint32_t)data[6] << 16) |
                                          ((uint32_t)data[5] << 8)  | ((uint32_t)data[4]);

    can->sTxMailBox[tx_mailbox].TDTR = dlc & 0x0F;

    *tx_reg |= CAN_TI0R_TXRQ;

    /* 等待发送完成 (RQCP=1), 或超时 */
    start_tick = HAL_GetTick();
    uint32_t rqcp_mask = (tx_mailbox == 0) ? CAN_TSR_RQCP0 :
                         (tx_mailbox == 1) ? CAN_TSR_RQCP1 : CAN_TSR_RQCP2;
    while ((can->TSR & rqcp_mask) == 0) {
        if (HAL_GetTick() - start_tick >= 20) {
            printf("[CAN_ERR] TX timeout on mailbox %lu, aborting\r\n", (unsigned long)tx_mailbox);
            if (tx_mailbox == 0) can->TSR = CAN_TSR_ABRQ0;
            else if (tx_mailbox == 1) can->TSR = CAN_TSR_ABRQ1;
            else can->TSR = CAN_TSR_ABRQ2;
            return 1;
        }
    }

    /* v1.6.3: 检查 TXOK 标志确认发送是否真正成功 (有 ACK) */
    uint32_t txok_mask = (tx_mailbox == 0) ? CAN_TSR_TXOK0 :
                         (tx_mailbox == 1) ? CAN_TSR_TXOK1 : CAN_TSR_TXOK2;
    if ((can->TSR & txok_mask) == 0) {
        /* 发送完成但未收到 ACK — 总线上没有设备响应 */
        return 2;  /* 区分超时(1) 和无 ACK(2) */
    }

    return 0;
}

void cybergear_mit_enable(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t ext_id = cybergear_pack_ext_id(CG_CMD_ENABLE, CG_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0};
    can_tx_ext_frame(hcan, ext_id, data, 8);
}

void cybergear_mit_disable(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t ext_id = cybergear_pack_ext_id(CG_CMD_STOP, CG_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0};
    can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* v1.6.3+fix: CyberGear 设置机械零位 (CMD=6, Byte0=1) — can.txt L120
 * 需要写电机内部 Flash, 调用后需 ≥10ms 延迟让其保存 */
void cybergear_mit_set_zero(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint32_t ext_id = cybergear_pack_ext_id(CG_CMD_SET_ZERO, CG_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0};
    data[0] = 1;   /* can.txt: Byte0=1 执行零位设置 */
    can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* Send MIT zero-torque control frame (type 1) to trigger feedback */
void cybergear_mit_zero_torque(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint16_t t_uint = (uint16_t)float_to_uint(0.0f, CG_T_MIN, CG_T_MAX, 16);
    uint32_t ext_id = cybergear_pack_ext_id(CG_CMD_MIT_CTRL, t_uint, (uint8_t)motor_id);
    uint8_t data[8];
    cybergear_mit_pack_command(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, data);
    can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* RS01 zero-torque: 扩展帧, MIT运控模式(mode=1), torque=0
 * ★ v1.6.8fix: torque 在 ExtId data 字段 (0x7FFF = 0 N·m), Data=全0 (Kp=Kd=0)
 *   与 CyberGear 协议一致 */
uint8_t rs01_send_zero_torque(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    uint16_t t_uint = (uint16_t)float_to_uint(0.0f, RS01_T_MIN, RS01_T_MAX, 16);
    uint32_t ext_id = rs01_pack_ext_id(RS01_CMD_MIT_CTRL, t_uint, (uint8_t)motor_id);
    uint8_t data[8] = {0};
    return can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* Send MIT position control frame (type 1) via direct register access
   pos_mdeg: target position in millidegrees
   kp, kd: stiffness and damping (0-500, 0-5) */
void cybergear_mit_set_position(CAN_HandleTypeDef *hcan, uint32_t motor_id,
                                 int32_t pos_mdeg, float kp, float kd)
{
    float pos_rad = (float)pos_mdeg / RAD_TO_MDEG;
    uint16_t t_uint = (uint16_t)float_to_uint(0.0f, CG_T_MIN, CG_T_MAX, 16);
    uint32_t ext_id = cybergear_pack_ext_id(CG_CMD_MIT_CTRL, t_uint, (uint8_t)motor_id);
    uint8_t data[8];
    cybergear_mit_pack_command(motor_id, pos_rad, 0.0f, kp, kd, data);
    can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* Send MIT torque control frame (type 1) via direct register access
   torque_mnm: target torque in millinewton-meters */
void cybergear_mit_set_torque(CAN_HandleTypeDef *hcan, uint32_t motor_id, int32_t torque_mnm)
{
    float torque_nm = (float)torque_mnm / 1000.0f;
    uint16_t t_uint = (uint16_t)float_to_uint(torque_nm, CG_T_MIN, CG_T_MAX, 16);
    uint32_t ext_id = cybergear_pack_ext_id(CG_CMD_MIT_CTRL, t_uint, (uint8_t)motor_id);
    uint8_t data[8];
    cybergear_mit_pack_command(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, data);
    can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* Read a parameter via type 17 - useful for verifying communication */
void cybergear_read_param(CAN_HandleTypeDef *hcan, uint32_t motor_id, uint16_t index)
{
    uint32_t ext_id = cybergear_pack_ext_id(CG_CMD_READ_PARAM, CG_MASTER_ID, (uint8_t)motor_id);
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(index & 0xFF);
    data[1] = (uint8_t)((index >> 8) & 0xFF);
    can_tx_ext_frame(hcan, ext_id, data, 8);
}

/* ========== Emergency Disable ========== */
void can_motor_disable_all(void)
{
    uint32_t tx_mailbox;

    /* CyberGear (CAN1): Extended Frame stop command (type 4) — IDs 1, 2 (Leg) */
    cybergear_mit_disable(&g_can1_handle, MOTOR_LEG_0_ID);
    cybergear_mit_disable(&g_can1_handle, MOTOR_LEG_1_ID);

    /* RS01 (CAN2): 扩展帧停止命令 (mode=4) — IDs 0x10~0x13 (Arm) */
    for (uint32_t id = MOTOR_ARM_0_ID; id <= MOTOR_ARM_0_ID + 3; id++) {
        rs01_mit_disable(&g_can2_handle, id);
    }
}

void can_motor_disable_bus(CAN_HandleTypeDef *hcan)
{
    uint32_t tx_mailbox;

    if (hcan->Instance == CAN1) {
        /* CAN1 = CyberGear (Leg) — IDs 1, 2 */
        cybergear_mit_disable(hcan, MOTOR_LEG_0_ID);
        cybergear_mit_disable(hcan, MOTOR_LEG_1_ID);
    } else {
        /* CAN2 = RS01 (Arm) — IDs 0x10~0x13 */
        for (uint32_t id = MOTOR_ARM_0_ID; id <= MOTOR_ARM_0_ID + 3; id++) {
            rs01_mit_disable(hcan, id);
        }
    }
}

/* ========== CyberGear Current Mode (Write param 0x7006 = iq_ref) ========== */
void cybergear_send_current_cmd(CAN_HandleTypeDef *hcan, uint32_t motor_id, float current_a)
{
    uint32_t tx_mailbox;
    uint8_t data[8];
    memset(data, 0, 8);
    /* Index 0x7006 = iq_ref (current reference) */
    data[0] = 0x06; data[1] = 0x70;
    /* Float current value in Byte4-7 */
    memcpy(&data[4], &current_a, 4);

    CAN_TxHeaderTypeDef tx_header;
    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.DLC = 8;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.ExtId = cybergear_pack_ext_id(CG_CMD_WRITE_PARAM, 0, (uint8_t)motor_id);

    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}

/* ========== Motor Online Status Tracking ========== */

typedef struct {
    CAN_HandleTypeDef *hcan;
    uint32_t can_id;
    uint32_t last_feedback_ms;
    uint8_t  online;
} motor_track_t;

static motor_track_t g_motor_track[MAX_MOTORS];
static uint8_t g_motor_count = 0;

/* Global motor status (index: [0]=Leg0 ID1, [1]=Leg1 ID2, [2]=Arm0 ID16, [3]=Arm1 ID17, [4]=Arm2 ID18, [5]=Arm3 ID19) */
/* NOTE: g_joint_status[6] is forward-declared at top of file — 腿2个 + 臂4个 = 6 */

/* v1.6.3+fix: 同时支持双髋 CAN ID (0x01=左髋,0x02=右髋) 和 臂部 CAN ID (0x10~0x13/16~19) */
JointStatus_t *can_motor_get_status(uint8_t can_id)
{
    if (can_id >= MOTOR_LEG_0_ID && can_id <= MOTOR_LEG_1_ID) {
        /* 双髋 CyberGear: CAN ID 0x01 → idx0(左髋 L-Hip), 0x02 → idx1(右髋 R-Hip) */
        return &g_joint_status[can_id - MOTOR_LEG_0_ID];
    }
    if (can_id >= MOTOR_ARM_0_ID && can_id <= MOTOR_ARM_3_ID) {
        /* 臂部 RS01: CAN ID 0x10→idx2, 0x11→idx3, 0x12→idx4, 0x13→idx5 */
        return &g_joint_status[(can_id - MOTOR_ARM_0_ID) + 2];
    }
    return NULL;
}

void can_motor_set_mapping(uint8_t display_id, CAN_HandleTypeDef *hcan, uint32_t can_id)
{
    if (display_id >= MAX_MOTORS) return;
    g_motor_track[display_id].hcan = hcan;
    g_motor_track[display_id].can_id = can_id;
    g_motor_track[display_id].last_feedback_ms = 0;
    g_motor_track[display_id].online = 0;
    if (display_id + 1 > g_motor_count) g_motor_count = display_id + 1;
}

void can_motor_online_tick(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < g_motor_count; i++) {
        if (g_motor_track[i].online &&
            (now - g_motor_track[i].last_feedback_ms > MOTOR_TIMEOUT_MS)) {
            g_motor_track[i].online = 0;
        }
    }
}

uint8_t can_motor_is_online(uint8_t display_id)
{
    if (display_id >= MAX_MOTORS) return 0;
    return g_motor_track[display_id].online;
}

static void can_motor_mark_online(CAN_HandleTypeDef *hcan, uint32_t can_id)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < g_motor_count; i++) {
        if (g_motor_track[i].hcan == hcan && g_motor_track[i].can_id == can_id) {
            g_motor_track[i].last_feedback_ms = now;
            g_motor_track[i].online = 1;
            return;
        }
    }
}