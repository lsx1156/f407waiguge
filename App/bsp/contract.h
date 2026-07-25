#ifndef __CONTRACT_H__
#define __CONTRACT_H__

#include <stdint.h>

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

/* ========== Parameter Table Types ========== */
#define TABLE_TYPE_DAMPING      0
#define TABLE_TYPE_FRICTION     1
#define TABLE_TYPE_PID          2

/* ========== Parameter Subcommands ========== */
#define PARAM_CMD_WRITETABLE    0x01
#define PARAM_CMD_UPDATEPID     0x02
#define PARAM_CMD_UPDATEFAULT   0x03
#define PARAM_CMD_SWITCHTABLE   0x04

/* ========== Frame Header (8 bytes) ========== */
typedef struct __attribute__((packed)) {
    uint8_t frame_type;
    uint8_t reserved;
    uint16_t seq_num;
    uint32_t timestamp;
} FrameHeader_t;

/* ========== Single Joint Status (17 bytes packed) ========== */
typedef struct __attribute__((packed)) {
    uint8_t joint_id;
    int32_t position;
    int32_t velocity;
    int32_t torque;
    uint16_t temperature;
    uint16_t fault_code;
} JointStatus_t;

/* ========== Report Frame ========== */
typedef struct __attribute__((packed)) {
    FrameHeader_t header;
    JointStatus_t leg_status[2];
    JointStatus_t arm_status[4];
    uint16_t weight_adc;
    uint16_t voltage_adc;
    uint16_t crc;
} ReportFrame_t;

/* ========== Single Joint Command (16 bytes packed) ========== */
typedef struct __attribute__((packed)) {
    uint8_t joint_id;
    uint8_t control_mode;
    int32_t syn_target;
    uint16_t max_velocity;
    uint16_t max_acceleration;
    uint16_t torque_rate_limit;
    uint8_t pid_set_index;
    uint8_t kp;
    uint8_t kd;
    uint8_t reserved;
} JointCommand_t;

/* ========== Command Frame ========== */
typedef struct __attribute__((packed)) {
    FrameHeader_t header;
    JointCommand_t commands[6];
    uint16_t crc;
} CommandFrame_t;

/* ========== PID Parameters Set (16 bytes packed) ========== */
typedef struct __attribute__((packed)) {
    uint8_t kp;
    uint8_t kd;
    int32_t torque_limit;
    int32_t velocity_limit;
    int32_t dead_zone;
    uint16_t reserved;
} PIDParams_t;

/* ========== Parameter Frame ========== */
typedef struct __attribute__((packed)) {
    FrameHeader_t header;
    uint8_t sub_command;
    uint8_t table_type;
    uint16_t table_offset;
    uint8_t table_data[];
} ParamFrame_t;

/* ========== Heartbeat Frame (12 bytes) ========== */
typedef struct __attribute__((packed)) {
    FrameHeader_t header;
    uint16_t data;
    uint16_t crc;
} HeartbeatFrame_t;

/* ========== Frame Size Constants (match packed sizeof) ========== */
#define FRAME_HEADER_SIZE       8
#define JOINT_STATUS_SIZE       17
#define JOINT_COMMAND_SIZE      16
#define PID_PARAMS_SIZE         16
#define CRC_SIZE                2

#define REPORT_FRAME_SIZE       (FRAME_HEADER_SIZE + 6*JOINT_STATUS_SIZE + 2 + 2 + CRC_SIZE)
#define COMMAND_FRAME_SIZE      (8 + 6*16 + 2)
#define HEARTBEAT_FRAME_SIZE    (8 + 2 + 2)



#endif
