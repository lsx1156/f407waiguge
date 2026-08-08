/**
 * @file
 * Ethernet Interface Skeleton
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/opt.h"
#include "lwip/etharp.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/ethip6.h"
#include "netif/ethernet.h"
#if PPP_SUPPORT
#include "netif/ppp/pppoe.h"
#endif
#include "ethernetif.h"
#include "lwip_comm.h"
#include "string.h"


/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'


/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif {
    struct eth_addr *ethaddr;
    /* Add whatever per-interface state that is needed here. */
};

/* Forward declarations. */
void  ethernetif_input(struct netif *netif);

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
low_level_init(struct netif *netif)
{
    netif->hwaddr_len = ETHARP_HWADDR_LEN; /* ����MAC��ַ����,Ϊ6���ֽ� */
    /* ��ʼ��MAC��ַ,����ʲô��ַ���û��Լ�����,���ǲ����������������豸MAC��ַ�ظ� */
    netif->hwaddr[0] = g_lwipdev.mac[0]; 
    netif->hwaddr[1] = g_lwipdev.mac[1]; 
    netif->hwaddr[2] = g_lwipdev.mac[2];
    netif->hwaddr[3] = g_lwipdev.mac[3];   
    netif->hwaddr[4] = g_lwipdev.mac[4];
    netif->hwaddr[5] = g_lwipdev.mac[5];
    
    netif->mtu = 1500;

    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
    
    HAL_ETH_Start(&g_eth_handle);
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t
low_level_output(struct netif *netif, struct pbuf *p)
{
    err_t errval;
    struct pbuf *q;
    
    uint8_t *buffer = (uint8_t *)(g_eth_handle.TxDesc->Buffer1Addr);
    __IO ETH_DMADescTypeDef *DmaTxDesc;
    uint32_t framelength = 0;
    uint32_t bufferoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t payloadoffset = 0;

    DmaTxDesc = g_eth_handle.TxDesc;
    bufferoffset = 0;

#if ETH_PAD_SIZE
  pbuf_remove_header(p, ETH_PAD_SIZE); /* drop the padding word */
#endif
    
    /* ��pbuf�п���Ҫ���͵����� */
    for (q = p;q != NULL;q = q->next)
    {
        /* �жϴ˷����������Ƿ���Ч�����жϴ˷����������Ƿ����̫��DMA���� */
        if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
        {
            errval = ERR_USE;
            goto error;               /* ������������Ч�������� */
        }
        
        byteslefttocopy = q->len;     /* Ҫ���͵����ݳ��� */
        payloadoffset = 0; 
        
        /* ��pbuf��Ҫ���͵�����д�뵽��̫�������������У���ʱ������Ҫ���͵����ݿ��ܴ���һ����̫��
           ��������Tx Buffer�����������Ҫ�ֶ�ν����ݿ�������������������� */
        while ((byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE )
        {
            /* �����ݿ�������̫��������������Tx Buffer�� */
            memcpy((uint8_t*)((uint8_t*)buffer + bufferoffset),(uint8_t*)((uint8_t*)q->payload + payloadoffset),(ETH_TX_BUF_SIZE - bufferoffset));
            /* DmaTxDscָ����һ������������ */
            DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);
            /* ����µķ����������Ƿ���Ч */
            if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
            {
                errval = ERR_USE;
                goto error;     /* ������������Ч�������� */
            }
            
            buffer = (uint8_t *)(DmaTxDesc->Buffer1Addr);   /* ����buffer��ַ��ָ���µķ�����������Tx Buffer */
            byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
            payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
            framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
            bufferoffset = 0;
        }
        /* ����ʣ������� */
        memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset),(uint8_t*)((uint8_t*)q->payload+payloadoffset),byteslefttocopy );
        bufferoffset = bufferoffset + byteslefttocopy;
        framelength = framelength + byteslefttocopy;
    }
    
    /* ������Ҫ���͵����ݶ��Ž�������������Tx Buffer�Ժ�Ϳɷ��ʹ�֡�� */
    HAL_ETH_TransmitFrame(&g_eth_handle,framelength);
    errval = ERR_OK;
error:            
    /* ���ͻ������������磬һ�����ͻ�������������TxDMA��������״̬ */
    if ((g_eth_handle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
    {
        /* ��������־ */
        g_eth_handle.Instance->DMASR = ETH_DMASR_TUS;
        /* ������֡�г�����������ʱ��TxDMA�������ʱ����Ҫ��DMATPDR�Ĵ��� */
        /* ���д��һ��ֵ�����份�ѣ��˴�����д0 */
        g_eth_handle.Instance->DMATPDR = 0;
    }
    
#if ETH_PAD_SIZE
  pbuf_add_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
    
    return errval;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *
low_level_input(struct netif *netif)
{  
    struct pbuf *p, *q;
    u16_t len;
    uint8_t *buffer;
    __IO ETH_DMADescTypeDef *dmarxdesc;
    uint32_t bufferoffset = 0;
    uint32_t payloadoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t i = 0;
  
    if (HAL_ETH_GetReceivedFrame(&g_eth_handle) != HAL_OK)  /* �ж��Ƿ���յ����� */
    return NULL;
    
    len = g_eth_handle.RxFrameInfos.length;                /* ��ȡ���յ�����̫��֡���� */
    
#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif
    
    buffer = (uint8_t *)g_eth_handle.RxFrameInfos.buffer;  /* ��ȡ���յ�����̫��֡������buffer */
  
    p = pbuf_alloc(PBUF_RAW,len,PBUF_POOL);                 /* ����pbuf */
    
    if (p != NULL)                                           /* pbuf����ɹ� */
    {
        dmarxdesc = g_eth_handle.RxFrameInfos.FSRxDesc;    /* ��ȡ���������������еĵ�һ�������� */
        bufferoffset = 0;
        
        for (q = p;q != NULL;q = q->next)
        {
            byteslefttocopy = q->len;
            payloadoffset = 0;
            
            /* ��������������Rx Buffer�����ݿ�����pbuf�� */
            while ((byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE )
            {
                /* �����ݿ�����pbuf�� */
                memcpy((uint8_t*)((uint8_t*)q->payload+payloadoffset),(uint8_t*)((uint8_t*)buffer + bufferoffset),(ETH_RX_BUF_SIZE - bufferoffset));
                 /* dmarxdesc����һ������������ */
                dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
                /* ����buffer��ַ��ָ���µĽ�����������Rx Buffer */
                buffer = (uint8_t *)(dmarxdesc->Buffer1Addr);
 
                byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
                payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
                bufferoffset = 0;
            }
            /* ����ʣ������� */
            memcpy((uint8_t*)((uint8_t*)q->payload + payloadoffset),(uint8_t*)((uint8_t*)buffer + bufferoffset),byteslefttocopy);
            bufferoffset = bufferoffset + byteslefttocopy;
        }
    }
    else
    {
        /* drop packet();  �����������б�д */
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);
        MIB2_STATS_NETIF_INC(netif, ifindiscards);
    }
    
    /* �ͷ�DMA������ */
    dmarxdesc = g_eth_handle.RxFrameInfos.FSRxDesc;
    
    for (i = 0;i < g_eth_handle.RxFrameInfos.SegCount; i ++)
    {  
        dmarxdesc->Status |= ETH_DMARXDESC_OWN;       /* �����������DMA���� */
        dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
    }
    
    g_eth_handle.RxFrameInfos.SegCount = 0;           /* ����μ����� */
    
    if ((g_eth_handle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)  /* ���ջ����������� */
    {
        /* ������ջ����������ñ�־ */
        g_eth_handle.Instance->DMASR = ETH_DMASR_RBUS;
        /* �����ջ����������õ�ʱ��RxDMA���ȥ����״̬��ͨ����DMARPDRд������һ��ֵ������Rx DMA */
        g_eth_handle.Instance->DMARPDR = 0;
    }
    
    return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void
ethernetif_input(struct netif *netif)
{
    struct pbuf *p;

    /* move received packet into a new pbuf */
    p = low_level_input(netif);
    /* if no packet could be read, silently ignore this */
    if (p != NULL)
    {
        /* pass all packets to ethernet_input, which decides what packets it supports */
        if (netif->input(p, netif) != ERR_OK)
        {
            LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
            pbuf_free(p);
            p = NULL;
        }
    }

} 

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
ethernetif_init(struct netif *netif)
{
    struct ethernetif *ethernetif;

    LWIP_ASSERT("netif != NULL", (netif != NULL));

    ethernetif = mem_malloc(sizeof(struct ethernetif));
    
    if (ethernetif == NULL)
    {
        LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
        return ERR_MEM;
    }

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    /*
    * Initialize the snmp variables and counters inside the struct netif.
    * The last argument should be replaced with your link speed, in units
    * of bits per second.
    */
    MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

    netif->state = ethernetif;
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
    * You can instead declare your own function an call etharp_output()
    * from it if you have to do some checks before sending (e.g. if link
    * is available...) */
#if LWIP_IPV4
    netif->output = etharp_output;
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
    netif->linkoutput = low_level_output;

    ethernetif->ethaddr = (struct eth_addr *) & (netif->hwaddr[0]);

    /* initialize the hardware */
    low_level_init(netif);

    return ERR_OK;
}



