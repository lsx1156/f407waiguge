#ifndef __UDP_NET_H__
#define __UDP_NET_H__

#include "lwip/udp.h"
#include "bsp_config.h"

extern struct udp_pcb *g_udp_pcb;

void udp_net_init(void);
void udp_net_send(uint8_t *data, uint16_t len);
void udp_net_poll(void);

#endif
