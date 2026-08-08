#ifndef _LWIP_COMM_H
#define _LWIP_COMM_H 
#include "ethernet.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#define LWIP_DHCP_OFF                   (uint8_t) 0
#define LWIP_DHCP_START                 (uint8_t) 1
#define LWIP_DHCP_WAIT_ADDRESS          (uint8_t) 2
#define LWIP_DHCP_ADDRESS_ASSIGNED      (uint8_t) 3
#define LWIP_DHCP_TIMEOUT               (uint8_t) 4
#define LWIP_DHCP_LINK_DOWN             (uint8_t) 5

#define LWIP_MAX_DHCP_TRIES                       4

typedef struct  
{
    uint8_t mac[6];
    uint8_t remoteip[4];
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t dhcpstatus;
}__lwip_dev;

extern __lwip_dev g_lwipdev;
extern struct netif g_lwip_netif;
extern uint8_t g_lwip_inited;   /* 0=lwip初始化失败/未完成, 任务函数应跳过所有lwip调用 */

void lwip_pkt_handle(void);
void lwip_periodic_handle(uint32_t elapsed_ms);
void lwip_comm_default_ip_set(__lwip_dev *lwipx);
uint8_t lwip_comm_init(void);

#endif
