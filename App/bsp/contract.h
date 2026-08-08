#ifndef __CONTRACT_H__
#define __CONTRACT_H__

#include <stdint.h>

/* Cross-compiler packed attribute
 * ARMCLANG (V6+) and GCC support __attribute__((packed))
 * Only ARMCC5 (__CC_ARM without __clang__) needs __packed
 */
#if defined(__CC_ARM) && !defined(__clang__)
  #define PACKED __packed
#else
  #define PACKED __attribute__((packed))
#endif

/* ========== Unit Type Aliases (documentation only) ==========
   position:   mdeg (millidegrees, 1 deg = 1000 mdeg)
   velocity:   mdeg/s
   torque:     mNm  (millinewton-meters, 1 Nm = 1000 mNm)
   temperature:0.1 C (uint16_t, e.g. 250 = 25.0 C)
*/
typedef int32_t  Position_mdeg_t;
typedef int32_t  Velocity_mdeg_s_t;
typedef int32_t  Torque_mNm_t;
typedef uint16_t Temp_deciC_t;

/* ========== Frame Types ========== */
#define FRAME_TYPE_REPORT       0x01
#define FRAME_TYPE_COMMAND      0x02
#define FRAME_TYPE_PARAM        0x03
#define FRAME_TYPE_HEARTBEAT    0x0F

/* ========== Joint IDs ========== */
#define JOINT_LEFT_HIP          0x01
#define JOINT_LEFT_KNEE         0x02
#define JOINT_ARM_BIG           0x10
#define JOINT_ARM_SMALL         0x11
#define JOINT_ARM_CLAW          0x12
#define JOINT_ARM_RESERVED      0x13

/* ========== Control Modes ========== */
#define CTRL_MODE_POSITION      0
#define CTRL_MODE_TORQUE        1
#define CTRL_MODE_MIXED         2

/* ========== Fault Codes ========== */
#define FAULT_NONE              0x0000
#define FAULT_ESTOP             0x0001
#define FAULT_CAN1_TIMEOUT      0x0002
#define FAULT_CAN2_TIMEOUT      0x0004
#define FAULT_POSITION_JUMP     0x0008
#define FAULT_VELOCITY_LIMIT    0x0010
#define FAULT_TORQUE_LIMIT      0x0020
#define FAULT_VOLTAGE_LOW       0x0040
#define FAULT_WEIGHT_OPEN       0x0080
#define FAULT_COMM_LOST         0x0100

/* ========== Parameter Table Types ========== */
#define TABLE_TYPE_DAMPING      0
#define TABLE_TYPE_FRICTION     1
#define TABLE_TYPE_PID          2

/* ========== Parameter Subcommands ========== */
#define PARAM_CMD_WRITETABLE    0x01
#define PARAM_CMD_UPDATEPID     0x02
#define PARAM_CMD_UPDATEFAULT   0x03
#define PARAM_CMD_SWITCHTABLE   0x04
#define PARAM_CMD_WRITE_ABO     0x10   /* 写 ABO 观测器参数 (RK3506 下发) */

/* ========== ABO Observer Defaults (Q 定点) ========== */
#define ABO_DEFAULT_GAIN_Q10    1024   /* G = 1.0 (0 助力起步, 人感觉不到电机存在) */
#define ABO_DEFAULT_ALPHA_Q16   655    /* α ≈ 0.01, HPF 截止 ~ 1.6 Hz (1ms 采样) */
#define ABO_DEFAULT_LEAK_Q16    66     /* 偏置泄漏 ≈ 0.001, 时间常数 ~1s */

/* ========== Communication Mode ========== */
typedef enum {
    COMM_MODE_HOST       = 0,   /* 上位机控制模式 (默认, 心跳超时→故障) */
    COMM_MODE_STANDALONE = 1    /* 离线自主模式 (心跳超时忽略, 本地步态/按键控制) */
} CommMode_e;

extern volatile CommMode_e g_comm_mode;

/* ========== ABO Observer State (per-joint) ========== */
/* ========== ABO → ESO3 迁移路径说明 (P2-6) ==========
 * legacy: g_abo_state[] 在 1kHz ISR 持续更新 (abo_update_one), 各关节独立 bias/HPF.
 * v1.8+:  JointUnitState.eso_enable=1 后启用 §B ESO3 路径, 此时 g_abo_state 仅作
 *         "只读回退"——上层(ABO参数下发/LCD显示/EEPROM持久化)仍可读 g_abo_state,
 *         但其 bias_est/assist 不再驱动电机 (力矩由 ESO3+导纳 tau_cmd 接管).
 *         迁移期间 legacy 字段保持兼容, 不删除以避免破坏 EEPROM 布局与上位机协议. */
typedef struct PACKED {
    int32_t  bias_est;         /* 重力偏置估计 (mNm), 持久化存 EEPROM */
    int32_t  hpf_state;        /* HPF 内部状态 (mNm) */
    uint16_t assist_gain_q10;  /* 助力增益 G * 1024 (RK 下发) */
    uint16_t hpf_alpha_q16;    /* HPF 系数 α * 65536 (RK 下发) */
    uint16_t bias_leak_q16;    /* 偏置泄漏系数 * 65536 (RK 下发) */
    uint8_t  enable;           /* 该关节是否启用 ABO (0=关, 1=开) */
    uint8_t  industrial_mode;  /* ★ v1.7: 1=工业模式, 0=医疗模式 */
    /* ★ v1.7: 工业模式专用字段 */
    int32_t  load_est_q10;     /* 估计外部负载力矩 (mNm, Q10) */
    uint16_t load_freeze_cnt;  /* 负载突变冻结偏置计数器 (ms) */
    int32_t  bp_lpf_state;     /* 带通低通部分状态 (mNm) */
    int32_t  tau_prev;         /* 上周期力矩 (用于突变检测) */
} ABOState_t;

/* ========== Frame Header (8 bytes) ========== */
typedef struct PACKED {
    uint8_t frame_type;
    uint8_t reserved;
    uint16_t seq_num;
    uint32_t timestamp;
} FrameHeader_t;

/* ========== Single Joint Status (17 bytes packed) ========== */
typedef struct PACKED {
    uint8_t joint_id;
    int32_t position;
    int32_t velocity;
    int32_t torque;
    uint16_t temperature;
    uint16_t fault_code;
} JointStatus_t;

/* ========== Report Frame ========== */
typedef struct PACKED {
    FrameHeader_t header;
    JointStatus_t leg_status[2];
    JointStatus_t arm_status[4];
    uint16_t crc;
} ReportFrame_t;

/* ========== Single Joint Command (22 bytes packed, v1.1 with ABO) ========== */
typedef struct PACKED {
    uint8_t joint_id;
    uint8_t control_mode;
    int32_t syn_target;
    uint16_t max_velocity;
    uint16_t max_acceleration;
    uint16_t torque_rate_limit;
    uint8_t pid_set_index;
    uint8_t kp;
    uint8_t kd;
    uint8_t abo_enable;         /* ABO 使能 (该关节是否启用观测器) */
    uint16_t assist_gain_q10;   /* 助力增益 G * 1024 */
    uint16_t hpf_alpha_q16;     /* HPF 系数 α * 65536 */
    uint16_t bias_leak_q16;     /* 偏置泄漏系数 * 65536 */
} JointCommand_t;

/* ========== Command Frame ========== */
typedef struct PACKED {
    FrameHeader_t header;
    JointCommand_t commands[6];
    uint16_t crc;
} CommandFrame_t;

/* ========== PID Parameters Set (16 bytes packed) ========== */
typedef struct PACKED {
    uint8_t kp;
    uint8_t kd;
    int32_t torque_limit;
    int32_t velocity_limit;
    int32_t dead_zone;
    uint16_t reserved;
} PIDParams_t;

/* ========== Parameter Frame ========== */
typedef struct PACKED {
    FrameHeader_t header;
    uint8_t sub_command;
    uint8_t table_type;
    uint16_t table_offset;
    uint8_t table_data[];
} ParamFrame_t;

/* ========== Heartbeat Frame (12 bytes) ========== */
typedef struct PACKED {
    FrameHeader_t header;
    uint16_t data;
    uint16_t crc;
} HeartbeatFrame_t;

/* ========== Frame Size Constants (match packed sizeof) ========== */
#define FRAME_HEADER_SIZE       8
#define JOINT_STATUS_SIZE       17
#define JOINT_COMMAND_SIZE      22    /* v1.1: +ABO params, 16→22 */
#define PID_PARAMS_SIZE         16
#define CRC_SIZE                2

#define REPORT_FRAME_SIZE       (FRAME_HEADER_SIZE + 6*JOINT_STATUS_SIZE + CRC_SIZE)
#define COMMAND_FRAME_SIZE      (8 + 6*22 + 2)   /* v1.1: 6 joints × 22 bytes */
#define HEARTBEAT_FRAME_SIZE    (8 + 2 + 2)



#endif
