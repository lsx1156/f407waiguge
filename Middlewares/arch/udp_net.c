#include "udp_net.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/sys.h"
#include "lwip_comm.h"
#include "udp_protocol.h"
#include "contract.h"
#include "interpolation.h"
#include <string.h>

#define UDP_RX_QUEUE_SIZE    8

struct udp_pcb *g_udp_pcb = NULL;

static ip_addr_t g_remote_ip;
static u16_t g_remote_port;
static uint8_t g_remote_bound = 0;
static volatile uint32_t g_udp_rx_drop_cnt = 0;

static struct pbuf *g_rx_queue[UDP_RX_QUEUE_SIZE];
static volatile uint32_t g_rx_head = 0;
static volatile uint32_t g_rx_tail = 0;

static void udp_recv_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static void udp_process_pbuf(struct pbuf *p);

static uint8_t rx_queue_try_push(struct pbuf *p)
{
    uint32_t next;
    sys_prot_t lev;

    next = (g_rx_head + 1) % UDP_RX_QUEUE_SIZE;
    if (next == g_rx_tail) {
        return 0;
    }

    lev = sys_arch_protect();
    g_rx_queue[g_rx_head] = p;
    g_rx_head = next;
    sys_arch_unprotect(lev);

    return 1;
}

static struct pbuf *rx_queue_try_pop(void)
{
    struct pbuf *p = NULL;
    sys_prot_t lev;

    if (g_rx_head == g_rx_tail) {
        return NULL;
    }

    lev = sys_arch_protect();
    if (g_rx_head != g_rx_tail) {
        p = g_rx_queue[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1) % UDP_RX_QUEUE_SIZE;
    }
    sys_arch_unprotect(lev);

    return p;
}

void udp_net_init(void)
{
    err_t err;

    g_rx_head = 0;
    g_rx_tail = 0;
    memset(g_rx_queue, 0, sizeof(g_rx_queue));

    IP4_ADDR(&g_remote_ip, g_lwipdev.remoteip[0], g_lwipdev.remoteip[1],
             g_lwipdev.remoteip[2], g_lwipdev.remoteip[3]);
    g_remote_port = UDP_PORT;

    g_udp_pcb = udp_new();
    if (g_udp_pcb) {
        err = udp_bind(g_udp_pcb, IP_ADDR_ANY, UDP_PORT);
        if (err == ERR_OK) {
            udp_recv(g_udp_pcb, udp_recv_callback, NULL);
        }
    }
}

void udp_net_send(uint8_t *data, uint16_t len)
{
    if (g_udp_pcb && data && len > 0) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);
        if (p) {
            pbuf_take(p, (char *)data, len);
            udp_sendto(g_udp_pcb, p, &g_remote_ip, g_remote_port);
            pbuf_free(p);
        }
    }
}

static void udp_recv_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL) {
        if (g_remote_bound == 0) {
            g_remote_ip = *addr;
            g_remote_port = port;
            g_remote_bound = 1;
        }

        pbuf_ref(p);
        if (!rx_queue_try_push(p)) {
            g_udp_rx_drop_cnt++;
            pbuf_free(p);
        }
    }
}

void udp_net_poll(void)
{
    struct pbuf *p;

    while ((p = rx_queue_try_pop()) != NULL) {
        udp_process_pbuf(p);
        pbuf_free(p);
    }
}

static void udp_process_pbuf(struct pbuf *p)
{
    struct pbuf *q;
    uint8_t *buf;
    uint32_t data_len;
    uint8_t frame_type;

    comm_heartbeat_kick();

    if (p == NULL || p->tot_len < 1) {
        return;
    }

    data_len = p->tot_len;
    if (data_len > 1500) {
        return;
    }

    q = pbuf_alloc(PBUF_RAW, data_len, PBUF_POOL);
    if (q == NULL) {
        return;
    }

    if (pbuf_copy_partial(p, q->payload, data_len, 0) != data_len) {
        pbuf_free(q);
        return;
    }

    buf = (uint8_t *)q->payload;
    frame_type = buf[0];
    switch (frame_type) {
        case FRAME_TYPE_COMMAND:
        {
            CommandFrame_t *frame = (CommandFrame_t *)buf;
            if (data_len >= sizeof(CommandFrame_t) &&
                frame->crc == crc16(buf, data_len - 2)) {
                JointCommand_t commands[6];
                command_frame_parse(frame, commands);
                command_fifo_write(commands);
            }
            break;
        }
        case FRAME_TYPE_PARAM:
        {
            ParamFrame_t *frame = (ParamFrame_t *)buf;
            if (data_len >= sizeof(FrameHeader_t) + 4) {
                uint16_t *crc_ptr = (uint16_t *)(buf + data_len - 2);
                if (*crc_ptr == crc16(buf, data_len - 2)) {
                    switch (frame->sub_command) {
                        case PARAM_CMD_WRITETABLE:
                        {
                            int32_t *table_data = (int32_t *)frame->table_data;
                            uint16_t offset = frame->table_offset;
                            uint32_t data_count = (data_len - sizeof(FrameHeader_t) - 4) / sizeof(int32_t);

                            if (frame->table_type == TABLE_TYPE_DAMPING) {
                                for (uint32_t i = 0; i < data_count; i++) {
                                    if (offset + i < DAMPING_TABLE_SIZE) {
                                        g_damping_table.damping[offset + i] = table_data[i];
                                    }
                                }
                            } else if (frame->table_type == TABLE_TYPE_FRICTION) {
                                for (uint32_t i = 0; i < data_count; i++) {
                                    if (offset + i < FRICTION_TABLE_SIZE) {
                                        g_friction_table.friction[offset + i] = table_data[i];
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
            }
            break;
        }
        case FRAME_TYPE_HEARTBEAT:
        {
            HeartbeatFrame_t *hb_frame = (HeartbeatFrame_t *)buf;
            if (data_len >= sizeof(HeartbeatFrame_t) && hb_frame->data == 0xFFFF) {
                fault_fifo_write(FAULT_ESTOP);
            }
            break;
        }
        default:
            break;
    }

    pbuf_free(q);
}
