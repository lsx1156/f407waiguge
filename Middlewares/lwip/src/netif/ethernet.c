#include "lwip/opt.h"

#if LWIP_ARP || LWIP_ETHERNET

#include "netif/ethernet.h"
#include "lwip/etharp.h"
#include "lwip/stats.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include <string.h>

const struct eth_addr ethbroadcast = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
const struct eth_addr ethzero = {{0, 0, 0, 0, 0, 0}};

err_t
ethernet_output(struct netif *netif, struct pbuf *p,
                const struct eth_addr *src, const struct eth_addr *dst,
                u16_t eth_type)
{
  struct eth_hdr *ethhdr;

  if ((p->len < SIZEOF_ETH_HDR) || ((p->len != p->tot_len))) {
    return ERR_BUF;
  }

  if (pbuf_add_header(p, SIZEOF_ETH_HDR) != 0) {
    return ERR_BUF;
  }

  ethhdr = (struct eth_hdr *)p->payload;
  ethhdr->type = lwip_htons(eth_type);
  SMEMCPY(&ethhdr->dest, dst, sizeof(struct eth_addr));
  SMEMCPY(&ethhdr->src, src, sizeof(struct eth_addr));

  LWIP_ASSERT("netif->hwaddr_len must be 6 for ethernet_output!",
              (netif->hwaddr_len == ETH_HWADDR_LEN));
  LWIP_DEBUGF(ETHARP_DEBUG | LWIP_DBG_TRACE,
              ("ethernet_output: sending packet %p\n", (void *)p));

  return netif->linkoutput(netif, p);
}

err_t
ethernet_input(struct pbuf *p, struct netif *netif)
{
  struct eth_hdr *ethhdr;
  u16_t type;
#if LWIP_ARP || ETHARP_SUPPORT_VLAN
  s16_t ip_hdr_offset = SIZEOF_ETH_HDR;
#endif

  if (p->len <= SIZEOF_ETH_HDR) {
    ETHARP_STATS_INC(lenerr);
    goto free_and_return;
  }

  if (pbuf_remove_header(p, SIZEOF_ETH_HDR) != 0) {
    goto free_and_return;
  }

  ethhdr = (struct eth_hdr *)((u8_t *)p->payload - SIZEOF_ETH_HDR);
  type = ethhdr->type;

#if ETHARP_SUPPORT_VLAN
  if (type == PP_HTONS(ETHTYPE_VLAN)) {
    struct eth_vlan_hdr *vlan = (struct eth_vlan_hdr *)p->payload;
    if (p->len <= SIZEOF_VLAN_HDR) {
      ETHARP_STATS_INC(lenerr);
      goto free_and_return;
    }
    ip_hdr_offset = SIZEOF_ETH_HDR + SIZEOF_VLAN_HDR;
    type = vlan->tpid;
    if (pbuf_remove_header(p, SIZEOF_VLAN_HDR) != 0) {
      goto free_and_return;
    }
    ethhdr = (struct eth_hdr *)((u8_t *)p->payload - SIZEOF_ETH_HDR);
    type = vlan->tpid;
  }
#endif

#if LWIP_ARP
  if (eth_type_vlan(type)) {
    goto free_and_return;
  }
#endif

  switch (type) {
#if LWIP_ARP
    case PP_HTONS(ETHTYPE_ARP):
      if (p->len >= SIZEOF_ETHARP_HDR) {
        etharp_input(p, netif);
        return ERR_OK;
      }
      break;
#endif
#if LWIP_IPV4
    case PP_HTONS(ETHTYPE_IP):
      if (p->len >= ip_hdr_offset) {
        if (ip4_input(p, netif) == ERR_OK) {
          return ERR_OK;
        }
      }
      break;
#endif
#if LWIP_IPV6
    case PP_HTONS(ETHTYPE_IPV6):
      if (p->len >= ip_hdr_offset) {
        if (ip6_input(p, netif) == ERR_OK) {
          return ERR_OK;
        }
      }
      break;
#endif
    default:
      break;
  }

free_and_return:
  pbuf_free(p);
  return ERR_OK;
}

#endif
