#include "udp_net.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/sys.h"
#include "lwip_comm.h"
#include "udp_protocol.h"
#include "contract.h"
#include "control_isr.h"
#include "interpolation.h"
#include <string.h>

#define UDP_RX_QUEUE_SIZE    32    /* v1.2 从8调大, 防突发包丢包 */

struct udp_pcb *g_udp_pcb = NULL;

static ip_addr_t g_remote_ip;
static u16_t g_remote_port;
uint8_t g_remote_bound = 0;                /* v1.2: 移至头文件暴露, 便于调试 */
volatile uint32_t g_udp_rx_drop_cnt = 0;   /* v1.2: 移至头文件暴露, 便于调试 */

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

    /* 关键: 清零 udp_pcbs 全局链表头.
     * 软复位 (IWDG/SYSRESETREQ) 不清内部 RAM .bss,
     * udp_pcbs 仍指向上次崩溃的脏 PCB, 收到包时会遍历到
     * 旧 PCB 并调用其 recv 回调 (垃圾指针).
     * 必须在 udp_new() 之前将链表头置空. */
    {
        extern struct udp_pcb *udp_pcbs;
        udp_pcbs = NULL;
    }

    g_rx_head = 0;
    g_rx_tail = 0;
    memset(g_rx_queue, 0, sizeof(g_rx_queue));

    IP4_ADDR(&g_remote_ip, g_lwipdev.remoteip[0], g_lwipdev.remoteip[1],
             g_lwipdev.remoteip[2], g_lwipdev.remoteip[3]);
    g_remote_port = UDP_PORT;

    g_udp_pcb = udp_new();
    if (g_udp_pcb) {
        /* 软复位 (IWDG/SYSRESETREQ) 不清内部 RAM .bss,
         * MEMP 静态池残留上次崩溃的脏数据 (如 recv 回调指针).
         * udp_new() 只做最小初始化, 不清零业务字段.
         * 显式清零整个 pcb, 确保所有指针/回调从干净状态开始. */
        memset(g_udp_pcb, 0, sizeof(struct udp_pcb));
        err = udp_bind(g_udp_pcb, IP_ADDR_ANY, UDP_PORT);
        if (err == ERR_OK) {
            udp_recv(g_udp_pcb, udp_recv_callback, NULL);
        }
    }
}

void udp_net_send(uint8_t *data, uint16_t len)
{
    static uint32_t last_err_print = 0;

    if (g_udp_pcb && data && len > 0) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);
        if (p) {
            pbuf_take(p, (char *)data, len);
            err_t err = udp_sendto(g_udp_pcb, p, &g_remote_ip, g_remote_port);
            if (err != ERR_OK) {
                /* 限速：最多每秒打印一次错误 */
                if (HAL_GetTick() - last_err_print >= 1000) {
                    last_err_print = HAL_GetTick();
                    printf("[DBG] udp_sendto FAILED, err=%d, len=%d\r\n", err, len);
                }
            }
            pbuf_free(p);
        } else {
            /* 限速：最多每秒打印一次错误 */
            if (HAL_GetTick() - last_err_print >= 1000) {
                last_err_print = HAL_GetTick();
                printf("[DBG] pbuf_alloc FAILED, len=%d\r\n", len);
            }
        }
    }
}

static void udp_recv_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL) {
        /* v1.2 调试: 打印收到包的源 IP/端口和绑定状态 */
        printf("[UDP RX] src=%d.%d.%d.%d:%d len=%d bound=%d expect=%d.%d.%d.%d:%d\r\n",
               ip4_addr1(addr), ip4_addr2(addr), ip4_addr3(addr), ip4_addr4(addr), port,
               p->tot_len,
               g_remote_bound,
               ip4_addr1(&g_remote_ip), ip4_addr2(&g_remote_ip),
               ip4_addr3(&g_remote_ip), ip4_addr4(&g_remote_ip),
               g_remote_port);

        /* v1.2: STANDALONE 模式下收到上位机包 → 自动切换到 HOST 模式 */
        if (g_comm_mode == COMM_MODE_STANDALONE) {
            g_comm_mode = COMM_MODE_HOST;
            g_remote_bound = 0;  /* 重置绑定, 使用新上位机地址 */
            printf("[MODE] AUTO → HOST (detected upper PC at %d.%d.%d.%d:%d)\r\n",
                   ip4_addr1(addr), ip4_addr2(addr),
                   ip4_addr3(addr), ip4_addr4(addr), port);
        }

        if (g_remote_bound == 0) {
            g_remote_ip = *addr;
            g_remote_port = port;
            g_remote_bound = 1;
            printf("[UDP] BIND to %d.%d.%d.%d:%d\r\n",
                   ip4_addr1(addr), ip4_addr2(addr),
                   ip4_addr3(addr), ip4_addr4(addr), port);
        } else {
            /* 防劫持: 检查是否来自已绑定的远程地址 */
            if (addr->addr != g_remote_ip.addr || port != g_remote_port) {
                printf("[UDP] DROP unknown src=%d.%d.%d.%d:%d (expected bound)\r\n",
                       ip4_addr1(addr), ip4_addr2(addr),
                       ip4_addr3(addr), ip4_addr4(addr), port);
                pbuf_free(p);
                return;
            }
        }

        pbuf_ref(p);
        if (!rx_queue_try_push(p)) {
            g_udp_rx_drop_cnt++;
            printf("[UDP] RX queue full, drop cnt=%lu\r\n", (unsigned long)g_udp_rx_drop_cnt);
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
                        /* ====== v1.1 ABO Observer 参数下发 ======
                         *  table_type = joint_idx (0~5: 0=左髋,1=右髋/膝,2~5=四臂)
                         *  table_data: [0-1]=assist_gain_q10, [2-3]=hpf_alpha_q16,
                         *              [4-5]=bias_leak_q16, [6]=enable */
                        case PARAM_CMD_WRITE_ABO:
                        {
                            uint32_t payload_len = data_len - sizeof(FrameHeader_t) - 4;
                            if (payload_len >= 7) {
                                uint8_t joint_idx = frame->table_type & 0x07;
                                uint16_t gain   = (uint16_t)(frame->table_data[0] | (frame->table_data[1] << 8));
                                uint16_t alpha  = (uint16_t)(frame->table_data[2] | (frame->table_data[3] << 8));
                                uint16_t leak   = (uint16_t)(frame->table_data[4] | (frame->table_data[5] << 8));
                                uint8_t  enable = frame->table_data[6];
                                abo_set_param(joint_idx, gain, alpha, leak, enable);
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
