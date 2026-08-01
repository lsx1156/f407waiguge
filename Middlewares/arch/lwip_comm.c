#include "lwip_comm.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include "lwip/dhcp.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/init.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "ethernetif.h"
#include "udp_net.h"
#include "sram.h"
#include <stdio.h>

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

__lwip_dev g_lwipdev;
struct netif g_lwip_netif;

#if LWIP_DHCP
uint32_t g_dhcp_fine_timer = 0;
__IO uint8_t g_lwip_dhcp_state = LWIP_DHCP_OFF;
#endif

void lwip_link_status_updated(struct netif *netif);
void lwip_dhcp_process(struct netif *netif);

void lwip_comm_default_ip_set(__lwip_dev *lwipx)
{
    lwipx->remoteip[0] = 192;
    lwipx->remoteip[1] = 168;
    lwipx->remoteip[2] = 10;
    lwipx->remoteip[3] = 101;
    
    lwipx->mac[0] = 0xB8;
    lwipx->mac[1] = 0xAE;
    lwipx->mac[2] = 0x1D;
    lwipx->mac[3] = 0x00;
    lwipx->mac[4] = 0x01;
    lwipx->mac[5] = 0x00;
    
    lwipx->ip[0] = 192;
    lwipx->ip[1] = 168;
    lwipx->ip[2] = 10;
    lwipx->ip[3] = 88;
    
    lwipx->netmask[0] = 255;
    lwipx->netmask[1] = 255;
    lwipx->netmask[2] = 255;
    lwipx->netmask[3] = 0;
    
    lwipx->gateway[0] = 192;
    lwipx->gateway[1] = 168;
    lwipx->gateway[2] = 10;
    lwipx->gateway[3] = 1;
    lwipx->dhcpstatus = 0;
}

uint8_t lwip_comm_init(void)
{
    uint8_t retry = 0;
    struct netif *netif_init_flag;
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

    lwip_comm_default_ip_set(&g_lwipdev);

    while (ethernet_init())
    {
        retry++;
        if (retry > 5)
        {
            retry = 0;
            return 3;
        }
    }

    lwip_init();

#if LWIP_DHCP
    ip_addr_set_zero_ip4(&ipaddr);
    ip_addr_set_zero_ip4(&netmask);
    ip_addr_set_zero_ip4(&gw);
#else
    IP4_ADDR(&ipaddr, g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
    IP4_ADDR(&netmask, g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
    IP4_ADDR(&gw, g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
    printf("MAC: %d.%d.%d.%d.%d.%d\r\n", g_lwipdev.mac[0], g_lwipdev.mac[1], g_lwipdev.mac[2], g_lwipdev.mac[3], g_lwipdev.mac[4], g_lwipdev.mac[5]);
    printf("IP: %d.%d.%d.%d\r\n", g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
    printf("Netmask: %d.%d.%d.%d\r\n", g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
    printf("Gateway: %d.%d.%d.%d\r\n", g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
    g_lwipdev.dhcpstatus = 0XFF;
#endif
    netif_init_flag = netif_add(&g_lwip_netif, (const ip_addr_t *)&ipaddr, (const ip_addr_t *)&netmask, (const ip_addr_t *)&gw, NULL, &ethernetif_init, &ethernet_input);

    if (netif_init_flag == NULL)
    {
        return 2;
    }
    else
    {
        netif_set_default(&g_lwip_netif);

        if (netif_is_link_up(&g_lwip_netif))
        {
            netif_set_up(&g_lwip_netif);
        }
        else
        {
            netif_set_down(&g_lwip_netif);
        }

        lwip_link_status_updated(&g_lwip_netif);
        netif_set_link_callback(&g_lwip_netif, lwip_link_status_updated);
    }

#if LWIP_DHCP
    g_lwipdev.dhcpstatus = 0;
#endif
    return 0;
}

void lwip_pkt_handle(void)
{
    ethernetif_input(&g_lwip_netif);
    ethernetif_check_link_status(&g_lwip_netif);
    udp_net_poll();
}

void lwip_link_status_updated(struct netif *netif)
{
    if (netif_is_up(netif))
    {
#if LWIP_DHCP
        g_lwip_dhcp_state = LWIP_DHCP_START;
#endif
    }
    else
    {
#if LWIP_DHCP
        g_lwip_dhcp_state = LWIP_DHCP_LINK_DOWN;
#endif
    }
}

void lwip_periodic_handle(uint32_t elapsed_ms)
{
    UNUSED(elapsed_ms);
    ethernetif_check_link_status(&g_lwip_netif);
    sys_check_timeouts();

#if LWIP_DHCP
    if (HAL_GetTick() - g_dhcp_fine_timer >= DHCP_FINE_TIMER_MSECS)
    {
        g_dhcp_fine_timer = HAL_GetTick();
        lwip_dhcp_process(&g_lwip_netif);
    }
#endif
}

#if LWIP_DHCP

void lwip_dhcp_process(struct netif *netif)
{
    uint32_t ip = 0;
    uint32_t netmask = 0;
    uint32_t gw = 0;
    struct dhcp *dhcp;
    uint8_t iptxt[20];
    g_lwipdev.dhcpstatus = 1;
    
    switch (g_lwip_dhcp_state)
    {
        case LWIP_DHCP_START:
        {
            ip_addr_set_zero_ip4(&netif->ip_addr);
            ip_addr_set_zero_ip4(&netif->netmask);
            ip_addr_set_zero_ip4(&netif->gw);
            g_lwip_dhcp_state = LWIP_DHCP_WAIT_ADDRESS;
            dhcp_start(netif);
        }
        break;

        case LWIP_DHCP_WAIT_ADDRESS:
        {
            ip = g_lwip_netif.ip_addr.addr;
            netmask = g_lwip_netif.netmask.addr;
            gw = g_lwip_netif.gw.addr;
            
            if (dhcp_supplied_address(netif)) 
            {
                g_lwip_dhcp_state = LWIP_DHCP_ADDRESS_ASSIGNED;
                sprintf((char *)iptxt, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->ip_addr));
                
                if (ip != 0)
                {
                    g_lwipdev.dhcpstatus = 2;
                    g_lwipdev.ip[3] = (uint8_t)(ip >> 24);
                    g_lwipdev.ip[2] = (uint8_t)(ip >> 16);
                    g_lwipdev.ip[1] = (uint8_t)(ip >> 8);
                    g_lwipdev.ip[0] = (uint8_t)(ip);
                    g_lwipdev.netmask[3] = (uint8_t)(netmask >> 24);
                    g_lwipdev.netmask[2] = (uint8_t)(netmask >> 16);
                    g_lwipdev.netmask[1] = (uint8_t)(netmask >> 8);
                    g_lwipdev.netmask[0] = (uint8_t)(netmask);
                    g_lwipdev.gateway[3] = (uint8_t)(gw >> 24);
                    g_lwipdev.gateway[2] = (uint8_t)(gw >> 16);
                    g_lwipdev.gateway[1] = (uint8_t)(gw >> 8);
                    g_lwipdev.gateway[0] = (uint8_t)(gw);
                }
            }
            else
            {
                dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

                if (dhcp->tries > LWIP_MAX_DHCP_TRIES)
                {
                    g_lwip_dhcp_state = LWIP_DHCP_TIMEOUT;
                    dhcp_stop(netif);
                    g_lwipdev.dhcpstatus = 0XFF;
                    IP4_ADDR(&(g_lwip_netif.ip_addr), g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
                    IP4_ADDR(&(g_lwip_netif.netmask), g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
                    IP4_ADDR(&(g_lwip_netif.gw), g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
                    netif_set_addr(netif, &g_lwip_netif.ip_addr, &g_lwip_netif.netmask, &g_lwip_netif.gw);
                }
            }
        }
        break;
        case LWIP_DHCP_LINK_DOWN:
        {
            dhcp_stop(netif);
            g_lwip_dhcp_state = LWIP_DHCP_OFF; 
        }
        break;
        default: break;
    }
}
#endif