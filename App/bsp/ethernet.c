#include "ethernet.h"
#include "main.h"
#include "lwip_comm.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_eth.h"

/* ETH handle now in main.c (heth), aliased in main.h */

/* Cross-compiler alignment attribute
 * ARMCLANG (V6+) and GCC support __attribute__((aligned))
 * Only ARMCC5 (__CC_ARM without __clang__) needs __align()
 */
#if defined(__CC_ARM) && !defined(__clang__)
  #define ALIGN4 __align(4)
#else
  #define ALIGN4 __attribute__((aligned(4)))
#endif

ALIGN4 ETH_DMADescTypeDef  g_eth_dma_rx_dscr_tab[ETH_RXBUFNB];
ALIGN4 ETH_DMADescTypeDef  g_eth_dma_tx_dscr_tab[ETH_TXBUFNB];
ALIGN4 uint8_t  g_eth_rx_buf[ETH_RXBUFNB][ETH_RX_BUF_SIZE];
ALIGN4 uint8_t  g_eth_tx_buf[ETH_TXBUFNB][ETH_TX_BUF_SIZE];

uint8_t ethernet_init(void)
{
    g_eth_handle.Instance = ETH;
    /* v1.5: 禁用自动协商, 用固定 100M/全双工.
     * 原因: HAL_ETH_Init 启用自动协商时会阻塞等待链路(5s)+协商完成(5s),
     *   没插网线就卡 10 秒, 拖垮主循环和喂狗.
     *   链路状态后续由主循环 ethernetif_check_link_status() 轮询检测. */
    g_eth_handle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;
    g_eth_handle.Init.Speed = ETH_SPEED_100M;
    g_eth_handle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    g_eth_handle.Init.PhyAddress = 0;
    g_eth_handle.Init.MACAddr = (uint8_t *)g_lwipdev.mac;
    g_eth_handle.Init.RxMode = ETH_RXINTERRUPT_MODE;
    g_eth_handle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    g_eth_handle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

    if (HAL_ETH_Init(&g_eth_handle) != HAL_OK)
    {
        printf("[ETH] HAL_ETH_Init FAILED (MAC config)\r\n");
        return 1;
    }
    printf("[ETH] HAL_ETH_Init OK (PhyAddr=0, 100M/Full, no-autoneg)\r\n");
    printf("[ETH] SYSCFG_PMC=0x%08lX (RMII_SEL=%s)\r\n",
           (unsigned long)SYSCFG->PMC,
           (SYSCFG->PMC & SYSCFG_PMC_MII_RMII_SEL) ? "RMII" : "MII");

    return 0;
}
