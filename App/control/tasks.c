#include "tasks.h"
#include "main.h"
#include "udp_protocol.h"
#include "control_isr.h"
#include "safety.h"
#include "eeprom.h"
#include "can_motor.h"
#include "bsp_config.h"
#include "contract.h"
#include "lwip_comm.h"
#include "udp_net.h"
#include <string.h>

void network_task_run(void)
{
    static uint8_t comm_lost_flag = 0;

    lwip_pkt_handle();
    lwip_periodic_handle(10);

    can_tx_task_drain();
    
    ReportFrame_t frame;
    while (report_fifo_read(&frame)) {
        udp_net_send((uint8_t*)&frame, sizeof(ReportFrame_t));
    }
    
    uint16_t fault_code;
    while (fault_fifo_read(&fault_code)) {
        HeartbeatFrame_t hb_frame;
        hb_frame.header.frame_type = FRAME_TYPE_HEARTBEAT;
        hb_frame.header.reserved = 0;
        hb_frame.header.seq_num = 0;
        hb_frame.header.timestamp = HAL_GetTick();
        hb_frame.data = fault_code;
        hb_frame.crc = crc16((uint8_t*)&hb_frame, sizeof(HeartbeatFrame_t) - 2);
        udp_net_send((uint8_t*)&hb_frame, sizeof(HeartbeatFrame_t));
    }
    
    JointCommand_t new_cmd[6];
    while (command_fifo_read(new_cmd)) {
        safe_cmd_write(new_cmd);
    }

    if (HAL_GetTick() - g_last_comm_ts > COMM_HEARTBEAT_TIMEOUT_MS) {
        if (!comm_lost_flag) {
            comm_lost_flag = 1;
            fault_fifo_write(FAULT_COMM_LOST);
        }
    } else {
        comm_lost_flag = 0;
    }

    static uint32_t last_hb_send = 0;
    static uint16_t hb_seq = 0;
    if (HAL_GetTick() - last_hb_send >= 200) {
        last_hb_send = HAL_GetTick();
        HeartbeatFrame_t hb;
        memset(&hb, 0, sizeof(hb));
        hb.header.frame_type = FRAME_TYPE_HEARTBEAT;
        hb.header.seq_num = hb_seq++;
        hb.header.timestamp = HAL_GetTick();
        hb.data = g_safety_state.fault_code;
        hb.crc = crc16((uint8_t*)&hb, sizeof(HeartbeatFrame_t) - 2);
        udp_net_send((uint8_t*)&hb, sizeof(HeartbeatFrame_t));
    }
}

void control_task_run(void)
{
    safety_task_run();
    safety_handle_fault();
}

void eeprom_task_run(void)
{
    eeprom_process_write_buffer();
}

void tasks_init(void)
{
    eeprom_init();
    udp_net_init();
}
