/**
 * @file
 * Ethernet output for LwIP
 */

#ifndef LWIP_HDR_NETIF_ETHERNET_H
#define LWIP_HDR_NETIF_ETHERNET_H

#include "lwip/opt.h"

#if LWIP_ARP || LWIP_ETHERNET

#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/prot/ethernet.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIZEOF_ETH_HDR 14

#ifndef ETHARP_HWADDR_LEN
#define ETHARP_HWADDR_LEN ETH_HWADDR_LEN
#endif

extern const struct eth_addr ethbroadcast;
extern const struct eth_addr ethzero;

#define LL_IP4_MULTICAST_ADDR_0 0x01U
#define LL_IP4_MULTICAST_ADDR_1 0x00U
#define LL_IP4_MULTICAST_ADDR_2 0x5EU

#define eth_addr_cmp(addr1, addr2) (memcmp((addr1)->addr, (addr2)->addr, ETH_HWADDR_LEN) == 0)

err_t ethernet_input(struct pbuf *p, struct netif *netif);
err_t ethernet_output(struct netif *netif, struct pbuf *p, const struct eth_addr *src, const struct eth_addr *dst, u16_t eth_type);

#ifdef __cplusplus
}
#endif

#endif

#endif
