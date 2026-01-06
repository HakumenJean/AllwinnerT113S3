/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include <stdlib.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

#include "drv_gpadc.h"
#include "hal_gpadc.h"

#define DBG_TAG  "gpadc"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

int rt_hw_sunxi_gpadc_init(void)
{
    hal_gpadc_init();

    LOG_I("gpadc init success");

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_sunxi_gpadc_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

int channel = -1;

int gpadc_irq_callback(uint32_t dada_type, uint32_t data)
{
    int vol_data;
    data = ((VOL_RANGE / 4096) * data);
    vol_data = data / 1000;
    LOG_I("channel %d vol data: %d", channel, vol_data);
    hal_gpadc_channel_exit(channel);
    hal_gpadc_deinit();
    return 0;
}

void gpadc_test(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_E("usage: gpadc_test channel");
        return;
    }

    hal_gpadc_init();

    channel = atoi(argv[1]);

    if (channel < 0 || channel > GPADC_CHANNEL_NUM)
    {
        LOG_E("channel %d is wrong, must between 0 and %d", channel, GPADC_CHANNEL_NUM);
        return;
    }

    hal_gpadc_channel_init(channel);
    hal_gpadc_register_callback(channel, gpadc_irq_callback);
}

MSH_CMD_EXPORT(gpadc_test, set gpadc);

#endif

