#ifndef __UDP_NET_H__
#define __UDP_NET_H__

#include "lwip/udp.h"
#include "bsp_config.h"

extern struct udp_pcb *g_udp_pcb;
extern uint8_t g_remote_bound;       /* 1=已绑定上位机地址, 0=未绑定 */
extern volatile uint32_t g_udp_rx_drop_cnt;

void udp_net_init(void);
void udp_net_send(uint8_t *data, uint16_t len);
void udp_net_poll(void);

#endif
