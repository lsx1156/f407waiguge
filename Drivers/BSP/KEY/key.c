/**
 ****************************************************************************************************
 * @file        key.c
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

#include "./BSP/KEY/key.h"
#include "./SYSTEM/delay/delay.h"


/**
 * @brief       ������ʼ������
 * @param       ��
 * @retval      ��
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;                          /* GPIO���ò����洢���� */
    KEY0_GPIO_CLK_ENABLE();                                     /* KEY0ʱ��ʹ�� */
    WKUP_GPIO_CLK_ENABLE();                                     /* WKUPʱ��ʹ�� */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0���� */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* ���� */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0����ģʽ����,�������� */

    gpio_init_struct.Pin = WKUP_GPIO_PIN;                       /* WKUP���� */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* ���� */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                      /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
    HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);           /* WKUP����ģʽ����,�������� */

}

/**
 * @brief       Key scan function with long-press support (non-blocking)
 * @retval      Key event code:
 *              KEY0_PRES (1)      - KEY0 short press
 *              KEY0_LONG_PRES (2) - KEY0 long press (>800ms)
 *              WKUP_PRES (4)      - WKUP short press
 *              WKUP_LONG_PRES (8) - WKUP long press (>800ms)
 *              0                   - No key event
 */
uint8_t key_scan(void)
{
    static uint8_t key0_state = 0;      /* 0=idle, 1=debounce, 2=pressed, 3=long fired */
    static uint32_t key0_press_ts = 0;
    static uint8_t wkup_state = 0;
    static uint32_t wkup_press_ts = 0;
    uint8_t result = 0;
    uint32_t now = HAL_GetTick();

#define KEY_DEBOUNCE_MS  20   /* 消抖时间, 非阻塞 */

    /* ---- KEY0 ---- */
    /* KEY0 (PE4) 上拉输入: 未按下=1(高), 按下=0(低) */
    if (key0_state == 0) {
        if (KEY0 == 0) {
            key0_state = 1;          /* 进入消抖 */
            key0_press_ts = now;
        }
    } else if (key0_state == 1) {
        if (now - key0_press_ts >= KEY_DEBOUNCE_MS) {
            if (KEY0 == 0) {
                key0_state = 2;      /* 确认按下 */
                key0_press_ts = now; /* 重新计时用于长按检测 */
            } else {
                key0_state = 0;      /* 抖动, 复位 */
            }
        }
    } else if (key0_state == 2) {
        if (KEY0 == 1) {
            /* Released - short press */
            key0_state = 0;
            result = KEY0_PRES;
        } else if (now - key0_press_ts >= KEY_LONG_PRESS_MS) {
            /* Long press detected */
            key0_state = 3;
            result = KEY0_LONG_PRES;
        }
    } else if (key0_state == 3) {
        if (KEY0 == 1) {
            key0_state = 0;
        }
    }

    /* If KEY0 already produced a result, skip WKUP this cycle */
    if (result != 0) return result;

    /* ---- WKUP ---- */
    if (wkup_state == 0) {
        if (WK_UP == 1) {
            wkup_state = 1;          /* 进入消抖 */
            wkup_press_ts = now;
        }
    } else if (wkup_state == 1) {
        if (now - wkup_press_ts >= KEY_DEBOUNCE_MS) {
            if (WK_UP == 1) {
                wkup_state = 2;      /* 确认按下 */
                wkup_press_ts = now; /* 重新计时用于长按检测 */
            } else {
                wkup_state = 0;      /* 抖动, 复位 */
            }
        }
    } else if (wkup_state == 2) {
        if (WK_UP == 0) {
            /* Released - short press */
            wkup_state = 0;
            result = WKUP_PRES;
        } else if (now - wkup_press_ts >= WKUP_LONG_PRESS_MS) {
            /* Long press detected */
            wkup_state = 3;
            result = WKUP_LONG_PRES;
        }
    } else if (wkup_state == 3) {
        if (WK_UP == 0) {
            wkup_state = 0;
        }
    }

    return result;
}




















