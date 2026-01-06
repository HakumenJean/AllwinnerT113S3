/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#ifndef __DRV_GPIO_H__
#define __DRV_GPIO_H__

#include "hal_gpio.h"

#define PIN_MODE_AF_FUN_2       (GPIO_MUXSEL_FUNCTION2 << 4)
#define PIN_MODE_AF_FUN_3       (GPIO_MUXSEL_FUNCTION3 << 4)
#define PIN_MODE_AF_FUN_4       (GPIO_MUXSEL_FUNCTION4 << 4)
#define PIN_MODE_AF_FUN_5       (GPIO_MUXSEL_FUNCTION5 << 4)
#define PIN_MODE_AF_FUN_6       (GPIO_MUXSEL_FUNCTION6 << 4)
#define PIN_MODE_AF_FUN_7       (GPIO_MUXSEL_FUNCTION7 << 4)
#define PIN_MODE_AF_FUN_8       (GPIO_MUXSEL_FUNCTION8 << 4)
#define PIN_MODE_AF_FUN_14      (GPIO_MUXSEL_EINT << 4)

typedef struct gpio_cfg
{
    gpio_pin_t pin;
    gpio_muxsel_t fun;
    gpio_pull_status_t pull;
    gpio_driving_level_t drv;
}gpio_cfg_t;

void gpio_set_config(struct gpio_cfg gpio);
int rt_hw_gpio_init(void);

#endif /* __DRV_GPIO_H__ */
