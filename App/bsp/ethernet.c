#include "ethernet.h"
#include "lwip_comm.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_eth.h"

ETH_HandleTypeDef g_eth_handle;

ETH_DMADescTypeDef  g_eth_dma_rx_dscr_tab[ETH_RXBUFNB] __attribute__((aligned(4)));
ETH_DMADescTypeDef  g_eth_dma_tx_dscr_tab[ETH_TXBUFNB] __attribute__((aligned(4)));
uint8_t  g_eth_rx_buf[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((aligned(4)));
uint8_t  g_eth_tx_buf[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__((aligned(4)));

uint8_t ethernet_init(void)
{
    g_eth_handle.Instance = ETH;
    g_eth_handle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
    g_eth_handle.Init.Speed = ETH_SPEED_100M;
    g_eth_handle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    g_eth_handle.Init.PhyAddress = 0;
    g_eth_handle.Init.MACAddr = (uint8_t *)g_lwipdev.mac;
    g_eth_handle.Init.RxMode = ETH_RXINTERRUPT_MODE;
    g_eth_handle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    g_eth_handle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

    if (HAL_ETH_Init(&g_eth_handle) != HAL_OK)
    {
        return 1;
    }

    HAL_ETH_DMATxDescListInit(&g_eth_handle, g_eth_dma_tx_dscr_tab, &g_eth_tx_buf[0][0], ETH_TXBUFNB);
    HAL_ETH_DMARxDescListInit(&g_eth_handle, g_eth_dma_rx_dscr_tab, &g_eth_rx_buf[0][0], ETH_RXBUFNB);

    return 0;
}