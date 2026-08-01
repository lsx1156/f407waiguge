/**
 ****************************************************************************************************
 * @file        key.c
 * @author      MZXQ
 * @version     V1.0
 * @date        2021-10-14
 * @brief       按键输入 驱动代码
 * @license     Copyright (c) 2020-2032, MZXQ
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:MZXQ F407开发板
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/KEY/key.h"
#include "./SYSTEM/delay/delay.h"


/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;                          /* GPIO配置参数存储变量 */
    KEY0_GPIO_CLK_ENABLE();                                     /* KEY0时钟使能 */
    WKUP_GPIO_CLK_ENABLE();                                     /* WKUP时钟使能 */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0引脚模式设置,上拉输入 */

    gpio_init_struct.Pin = WKUP_GPIO_PIN;                       /* WKUP引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                      /* 下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);           /* WKUP引脚模式设置,下拉输入 */

}

/**
 * @brief       按键扫描函数
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 1, KEY0按下
 *              WKUP_PRES, 4, WKUP按下
 */
uint8_t key_scan()
{
		if (KEY0 == 1){
			delay_ms(10);
				if (KEY0 == 1){
					while(KEY0 == 1);
					return KEY0_PRES;
				}
		}
		if (WK_UP == 1){
			delay_ms(10);
				if (WK_UP == 1){
					while(WK_UP == 1);
					return WKUP_PRES;
				}
		}
    return 0;              /* 返回键值 */
}




















