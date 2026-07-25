#include "tasks.h"
#include "udp_protocol.h"
#include "control_isr.h"
#include "safety.h"
#include "eeprom.h"
#include "can_motor.h"
#include "bsp_config.h"
#include "contract.h"
#include "lwip_comm.h"
#include "udp_net.h"

void network_task_run(void)
{
    lwip_pkt_handle();
    lwip_periodic_handle(10);
    
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
        __disable_irq();
        for (int i = 0; i < 6; i++) {
            g_active_command[i] = new_cmd[i];
        }
        __enable_irq();
    }
}

void control_task_run(void)
{
    safety_task_run();
}

void safety_task_run(void)
{
    uint16_t faults = safety_get_faults();
    
    if (faults != 0) {
        fault_fifo_write(faults);
        
        if (faults & FAULT_ESTOP) {
            can_motor_disable_all();
        }
        
        if (faults & FAULT_CAN1_TIMEOUT) {
            can_motor_disable_bus(&g_can1_handle);
        }
        
        if (faults & FAULT_CAN2_TIMEOUT) {
            can_motor_disable_bus(&g_can2_handle);
        }
    }
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
