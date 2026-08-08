/**
 ****************************************************************************************************
 * @file        key.h
 * @author      MZXQ
 * @version     V1.0
 * @date        2021-10-14
 * @brief       �������� ��������
 * @license     Copyright (c) 2020-2032, MZXQ
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:MZXQ F407������
 *
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __KEY_H
#define __KEY_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* ���� ���� */

#define KEY0_GPIO_PORT                  GPIOE
#define KEY0_GPIO_PIN                   GPIO_PIN_4
#define KEY0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE��ʱ��ʹ�� */

#define WKUP_GPIO_PORT                  GPIOA
#define WKUP_GPIO_PIN                   GPIO_PIN_0
#define WKUP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA��ʱ��ʹ�� */

/******************************************************************************************/

#define KEY0        HAL_GPIO_ReadPin(KEY0_GPIO_PORT, KEY0_GPIO_PIN)     /* ��ȡKEY0���� */
#define WK_UP       HAL_GPIO_ReadPin(WKUP_GPIO_PORT, WKUP_GPIO_PIN)     /* ��ȡWKUP���� */


#define KEY0_PRES        1   /* KEY0 short press */
#define WKUP_PRES        4   /* WKUP short press */
#define KEY0_LONG_PRES   2   /* KEY0 long press (>800ms) */
#define WKUP_LONG_PRES   8   /* WKUP long press (>2000ms, 防误触) */

#define KEY_LONG_PRESS_MS   3000   /* KEY0 长按阈值 (E-STOP, 设长防误触) */
#define WKUP_LONG_PRESS_MS  5000   /* WKUP 长按阈值 (模式切换, 设长防误触) */

void key_init(void);
uint8_t key_scan(void);
//uint8_t key_scan(uint8_t mode);     /* ����ɨ�躯�� */
#endif


















