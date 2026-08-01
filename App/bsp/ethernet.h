#ifndef __ETHERNET_H__
#define __ETHERNET_H__

#include "stm32f4xx_hal.h"
#include "main.h"

#ifndef ETH_RXBUFNB
#define ETH_RXBUFNB        4
#endif
#ifndef ETH_TXBUFNB
#define ETH_TXBUFNB        4
#endif
#ifndef ETH_RX_BUF_SIZE
#define ETH_RX_BUF_SIZE    1520
#endif
#ifndef ETH_TX_BUF_SIZE
#define ETH_TX_BUF_SIZE    1520
#endif

/* Use real HAL handle name (heth), not alias (g_eth_handle) */
extern ETH_HandleTypeDef heth;

extern ETH_DMADescTypeDef  g_eth_dma_rx_dscr_tab[];
extern ETH_DMADescTypeDef  g_eth_dma_tx_dscr_tab[];
extern uint8_t  g_eth_rx_buf[ETH_RXBUFNB][ETH_RX_BUF_SIZE];
extern uint8_t  g_eth_tx_buf[ETH_TXBUFNB][ETH_TX_BUF_SIZE];

uint8_t ethernet_init(void);

#endif
