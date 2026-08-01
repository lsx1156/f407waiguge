#ifndef __UDP_PROTOCOL_H__
#define __UDP_PROTOCOL_H__

#include "contract.h"
#include "bsp_config.h"

extern JointCommand_t g_active_command[6];
extern uint32_t g_report_seq_num;

typedef struct {
    JointCommand_t cmd[6];
    uint32_t       crc;
    uint8_t        version;
} SafeCmd_t;

void     safe_cmd_write(JointCommand_t *new_cmd);
uint8_t  safe_cmd_read(JointCommand_t *out_cmd);
uint32_t safe_cmd_get_version(void);

#define COMM_HEARTBEAT_TIMEOUT_MS   50
extern volatile uint32_t g_last_comm_ts;
void     comm_heartbeat_kick(void);
uint8_t  comm_is_heartbeat_ok(void);

uint16_t crc16(const uint8_t *data, uint32_t length);
void report_frame_build(ReportFrame_t *frame, JointStatus_t *leg, JointStatus_t *arm);
void command_frame_parse(CommandFrame_t *frame, JointCommand_t *commands);
void udp_protocol_init(void);

uint8_t report_fifo_write(ReportFrame_t *frame);
uint8_t command_fifo_write(JointCommand_t *commands);
uint8_t fault_fifo_write(uint16_t fault_code);
uint8_t report_fifo_read(ReportFrame_t *frame);
uint8_t command_fifo_read(JointCommand_t *commands);
uint8_t fault_fifo_read(uint16_t *fault_code);

#endif
