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
#include "ethernet.h"
#include "lwip_comm.h"
#include "main.h"
#include "string.h"
#include "stm32f4xx_hal.h"


/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'

#define LINK_STABLE_CHECK_CNT     3

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

static uint32_t g_last_link_check_tick = 0;
static uint8_t  g_link_stable_cnt = 0;
static uint8_t  g_last_link_state = 0;

/* Forward declarations. */
void  ethernetif_input(struct netif *netif);
void  ethernetif_check_link_status(struct netif *netif);

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
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    netif->hwaddr[0] = g_lwipdev.mac[0];
    netif->hwaddr[1] = g_lwipdev.mac[1];
    netif->hwaddr[2] = g_lwipdev.mac[2];
    netif->hwaddr[3] = g_lwipdev.mac[3];
    netif->hwaddr[4] = g_lwipdev.mac[4];
    netif->hwaddr[5] = g_lwipdev.mac[5];

    netif->mtu = 1500;

    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    /* ===== v1.5: 手动初始化 DMA 描述符链 (比 HAL 更可控) =====
     * 关键: 所有权位(OWN)、缓冲区地址、链表指针必须全部正确,
     *       且 MPU 已允许 DMA 访问该区域 (mpu_config.c Region 5). */

    /* --- Rx Descriptors: 所有权归 DMA (OWN=1), 环形链表 --- */
    for (int i = 0; i < ETH_RXBUFNB; i++) {
        g_eth_dma_rx_dscr_tab[i].Status = ETH_DMARXDESC_OWN;  /* 归 DMA 所有 */
        /* RCH=Return Chained (用第二地址指向下一个描述符), 缓冲区大小 */
        g_eth_dma_rx_dscr_tab[i].ControlBufferSize = ETH_DMARXDESC_RCH | ETH_RX_BUF_SIZE;
        g_eth_dma_rx_dscr_tab[i].Buffer1Addr = (uint32_t)&g_eth_rx_buf[i][0];
        /* 链接下一个描述符, 最后一个指向第一个 (环形) */
        g_eth_dma_rx_dscr_tab[i].Buffer2NextDescAddr =
            (i < ETH_RXBUFNB - 1) ? (uint32_t)&g_eth_dma_rx_dscr_tab[i + 1]
                                   : (uint32_t)&g_eth_dma_rx_dscr_tab[0];
    }
    g_eth_handle.Instance->DMARDLAR = (uint32_t)&g_eth_dma_rx_dscr_tab[0];

    /* --- Tx Descriptors: 所有权归 CPU (OWN=0), 环形链表 --- */
    for (int i = 0; i < ETH_TXBUFNB; i++) {
        /* TCH=Second Address Chained (链式), 校验和由硬件插入 */
        g_eth_dma_tx_dscr_tab[i].Status = ETH_DMATXDESC_TCH
                                         | ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL;
        g_eth_dma_tx_dscr_tab[i].Buffer1Addr = (uint32_t)&g_eth_tx_buf[i][0];
        g_eth_dma_tx_dscr_tab[i].Buffer2NextDescAddr =
            (i < ETH_TXBUFNB - 1) ? (uint32_t)&g_eth_dma_tx_dscr_tab[i + 1]
                                   : (uint32_t)&g_eth_dma_tx_dscr_tab[0];
    }
    g_eth_handle.Instance->DMATDLAR = (uint32_t)&g_eth_dma_tx_dscr_tab[0];

    /* 同步 handle 的描述符指针, HAL 发送/接收函数要用 */
    g_eth_handle.RxDesc = g_eth_dma_rx_dscr_tab;
    g_eth_handle.TxDesc = g_eth_dma_tx_dscr_tab;

    printf("[ETH] DMARDLAR=0x%08lX DMATDLAR=0x%08lX\r\n",
           (unsigned long)g_eth_handle.Instance->DMARDLAR,
           (unsigned long)g_eth_handle.Instance->DMATDLAR);

    /* 启动 MAC/DMA */
    if (HAL_ETH_Start(&g_eth_handle) != HAL_OK) {
        printf("[ETH] HAL_ETH_Start FAILED\r\n");
    } else {
        printf("[ETH] HAL_ETH_Start OK\r\n");
    }

    /* 保险: 强制拉起 MAC TE/RE 和 DMA ST/SR (HAL_ETH_Start 理论上已做) */
    g_eth_handle.Instance->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;
    g_eth_handle.Instance->DMAOMR |= ETH_DMAOMR_ST | ETH_DMAOMR_SR;
    g_eth_handle.Instance->DMAIER |= ETH_DMAIER_NISE | ETH_DMAIER_RIE | ETH_DMAIER_TIE;

    /* 等待 DMA 进程跑起来 (最多 10ms)
     * DMASR.RPS (bit17-18) = 01 -> Rx Running
     * DMASR.TPS (bit20-21) = 01 -> Tx Running */
    {
        uint32_t tick = HAL_GetTick();
        while (((g_eth_handle.Instance->DMASR & ETH_DMASR_RPS) >> 17) != 1 ||
               ((g_eth_handle.Instance->DMASR & ETH_DMASR_TPS) >> 20) != 1) {
            if (HAL_GetTick() - tick > 10) {
                printf("[ETH] WARNING: DMA Start Timeout (RPS=%d TPS=%d)\r\n",
                       (int)((g_eth_handle.Instance->DMASR & ETH_DMASR_RPS) >> 17),
                       (int)((g_eth_handle.Instance->DMASR & ETH_DMASR_TPS) >> 20));
                break;
            }
        }
    }

    printf("[ETH] Final: MACCR=0x%08lX (TE=%d RE=%d) DMAOMR=0x%08lX (ST=%d SR=%d) DMASR=0x%08lX (RPS=%d TPS=%d)\r\n",
           (unsigned long)g_eth_handle.Instance->MACCR,
           (int)((g_eth_handle.Instance->MACCR >> 3) & 1),
           (int)((g_eth_handle.Instance->MACCR >> 2) & 1),
           (unsigned long)g_eth_handle.Instance->DMAOMR,
           (int)((g_eth_handle.Instance->DMAOMR >> 13) & 1),
           (int)((g_eth_handle.Instance->DMAOMR >> 1) & 1),
           (unsigned long)g_eth_handle.Instance->DMASR,
           (int)((g_eth_handle.Instance->DMASR >> 17) & 3),
           (int)((g_eth_handle.Instance->DMASR >> 20) & 3));

    g_last_link_check_tick = 0;
    g_link_stable_cnt = 0;
    g_last_link_state = 0;
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
    __DSB();
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
volatile uint32_t g_dbg_eth_rx_frames = 0;
volatile uint32_t g_dbg_eth_input_calls = 0;
volatile uint32_t g_dbg_eth_getframe_fail = 0;
volatile uint32_t g_dbg_eth_dmasr = 0;
volatile uint32_t g_dbg_eth_desc0_status = 0;

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
  
    if (HAL_ETH_GetReceivedFrame(&g_eth_handle) != HAL_OK) {
        g_dbg_eth_getframe_fail++;
        /* 每 1 秒记录一次 DMA 状态 */
        static uint32_t last_dbg = 0;
        if (HAL_GetTick() - last_dbg >= 1000) {
            last_dbg = HAL_GetTick();
            g_dbg_eth_dmasr = g_eth_handle.Instance->DMASR;
            g_dbg_eth_desc0_status = g_eth_handle.RxDesc->Status;
        }
        return NULL;
    }
    
    g_dbg_eth_rx_frames++;
    
    len = g_eth_handle.RxFrameInfos.length;                /* 获取接收到的以太网帧长度 */
    
#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif
    
    buffer = (uint8_t *)g_eth_handle.RxFrameInfos.buffer;

    __DSB();

    p = pbuf_alloc(PBUF_LINK, len, PBUF_POOL);
    
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
    
    /* v1.6.3+fix: 释放DMA描述符后无条件重启Rx DMA (解决RPS/RBU导致的8万+rx_fail+92%CPU占用) */
    dmarxdesc = g_eth_handle.RxFrameInfos.FSRxDesc;
    
    for (i = 0;i < g_eth_handle.RxFrameInfos.SegCount; i ++)
    {  
        dmarxdesc->Status |= ETH_DMARXDESC_OWN;
        dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
    }
    
    g_eth_handle.RxFrameInfos.SegCount = 0;

    /* RBUS 清除 + 无条件写 DMARPDR 任何值确保 Rx DMA 重新轮询描述符 */
    if ((g_eth_handle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET) {
        g_eth_handle.Instance->DMASR = ETH_DMASR_RBUS;
    }
    g_eth_handle.Instance->DMARPDR = 0;   /* 强制 Rx DMA 脱离 RPS(停止) 状态, 不再依赖 RBUS 标志 */
    
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

    g_dbg_eth_input_calls++;

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

void
ethernetif_check_link_status(struct netif *netif)
{
    uint32_t phy_reg = 0;
    uint8_t  link_up = 0;
    uint32_t now = HAL_GetTick();

    if ((now - g_last_link_check_tick) < 100) {
        return;
    }
    g_last_link_check_tick = now;

    if (HAL_ETH_ReadPHYRegister(&g_eth_handle, PHY_BSR, &phy_reg) != HAL_OK) {
        return;
    }

    link_up = (phy_reg & PHY_LINKED_STATUS) ? 1 : 0;

    if (link_up == g_last_link_state) {
        g_link_stable_cnt = 0;
        return;
    }

    g_link_stable_cnt++;
    if (g_link_stable_cnt < LINK_STABLE_CHECK_CNT) {
        return;
    }

    g_link_stable_cnt = 0;
    g_last_link_state = link_up;

    if (link_up) {
        netif_set_link_up(netif);
    } else {
        netif_set_link_down(netif);
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



