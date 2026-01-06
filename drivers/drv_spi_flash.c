/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#if defined (BSP_USING_SPI_FLASH) && defined (BSP_USING_SPI0) && defined (RT_USING_SFUD)

#include "spi_flash_sfud.h"

#define SPI_FLASH_DEVICE_NAME       "spi00"

int rt_hw_spi_flash_with_sfud_init(void)
{
    static rt_spi_flash_device_t spi_flash_device;

    spi_flash_device = rt_sfud_flash_probe(SPI_FLASH_CHIP_NAME, SPI_FLASH_DEVICE_NAME);

    if(spi_flash_device == RT_NULL) return RT_ERROR;

    return RT_EOK;
}
INIT_PREV_EXPORT(rt_hw_spi_flash_with_sfud_init);

#endif
