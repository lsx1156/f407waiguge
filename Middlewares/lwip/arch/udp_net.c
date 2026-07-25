#include "udp_net.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip_comm.h"
#include "udp_protocol.h"
#include "contract.h"
#include "interpolation.h"
#include <string.h>

struct udp_pcb *g_udp_pcb = NULL;

static void udp_recv_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

void udp_net_init(void)
{
    err_t err;
    ip_addr_t rmtipaddr;
    
    g_udp_pcb = udp_new();
    if (g_udp_pcb) {
        IP4_ADDR(&rmtipaddr, g_lwipdev.remoteip[0], g_lwipdev.remoteip[1], 
                 g_lwipdev.remoteip[2], g_lwipdev.remoteip[3]);
        
        err = udp_connect(g_udp_pcb, &rmtipaddr, UDP_PORT);
        if (err == ERR_OK) {
            err = udp_bind(g_udp_pcb, IP_ADDR_ANY, UDP_PORT);
            if (err == ERR_OK) {
                udp_recv(g_udp_pcb, udp_recv_callback, NULL);
            }
        }
    }
}

void udp_net_send(uint8_t *data, uint16_t len)
{
    if (g_udp_pcb && data && len > 0) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);
        if (p) {
            pbuf_take(p, (char *)data, len);
            udp_send(g_udp_pcb, p);
            pbuf_free(p);
        }
    }
}

static void udp_recv_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL) {
        uint32_t data_len = 0;
        struct pbuf *q;
        uint8_t buf[256] = {0};
        const uint32_t buf_size = sizeof(buf);
        
        for (q = p; q != NULL; q = q->next) {
            uint32_t copy_len = (q->len > (buf_size - data_len)) ? (buf_size - data_len) : q->len;
            memcpy(buf + data_len, q->payload, copy_len);
            data_len += copy_len;
            if (data_len >= buf_size) break;
        }
        
        if (data_len > buf_size) {
            data_len = buf_size;
        }
        
        upcb->remote_ip = *addr;
        upcb->remote_port = port;
        
        uint8_t frame_type = buf[0];
        switch (frame_type) {
            case FRAME_TYPE_COMMAND:
            {
                CommandFrame_t *frame = (CommandFrame_t *)buf;
                if (frame->crc == crc16(buf, data_len - 2)) {
                    JointCommand_t commands[6];
                    command_frame_parse(frame, commands);
                    command_fifo_write(commands);
                }
                break;
            }
            case FRAME_TYPE_PARAM:
            {
                ParamFrame_t *frame = (ParamFrame_t *)buf;
                uint16_t *crc_ptr = (uint16_t *)(buf + data_len - 2);
                if (*crc_ptr == crc16(buf, data_len - 2)) {
                    switch (frame->sub_command) {
                        case PARAM_CMD_WRITETABLE:
                        {
                            if (frame->table_type == TABLE_TYPE_DAMPING) {
                                uint16_t offset = frame->table_offset;
                                for (int i = 0; i < (data_len - sizeof(FrameHeader_t) - 4) / sizeof(int32_t); i++) {
                                    if (offset + i < DAMPING_TABLE_SIZE) {
                                        g_damping_table.damping[offset + i] = ((int32_t*)frame->table_data)[i];
                                    }
                                }
                            } else if (frame->table_type == TABLE_TYPE_FRICTION) {
                                uint16_t offset = frame->table_offset;
                                for (int i = 0; i < (data_len - sizeof(FrameHeader_t) - 4) / sizeof(int32_t); i++) {
                                    if (offset + i < FRICTION_TABLE_SIZE) {
                                        g_friction_table.friction[offset + i] = ((int32_t*)frame->table_data)[i];
                                    }
                                }
                            }
                            break;
                        }
                        case PARAM_CMD_UPDATEPID:
                        {
                            uint8_t pid_index = frame->table_offset & 0x0F;
                            if (pid_index < PID_PARAM_SET_COUNT) {
                                g_pid_params[pid_index].kp = frame->table_data[0];
                                g_pid_params[pid_index].kd = frame->table_data[1];
                                g_pid_params[pid_index].torque_limit = ((int32_t*)frame->table_data)[2];
                                g_pid_params[pid_index].velocity_limit = ((int32_t*)frame->table_data)[3];
                                g_pid_params[pid_index].dead_zone = ((int32_t*)frame->table_data)[4];
                            }
                            break;
                        }
                        default:
                            break;
                    }
                }
                break;
            }
            case FRAME_TYPE_HEARTBEAT:
            {
                HeartbeatFrame_t *hb_frame = (HeartbeatFrame_t *)buf;
                if (hb_frame->data == 0xFFFF) {
                    fault_fifo_write(FAULT_ESTOP);
                }
                break;
            }
            default:
                break;
        }
        
        pbuf_free(p);
    }
}