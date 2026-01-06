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
#include <rtdevice.h>
#include "board.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define LED2 GPIOG(7)

int main(void)
{
    LOG_D("CPU CLK: %luMHz", rt_hw_cpux_axi_clk_get() / 1000000);
    LOG_D("Periph CLK: (1X) = %luMHz, (2X) = %luMHz, (800M) = %luMHz", rt_hw_periph1x_clk_get() / 1000000,
            rt_hw_periph2x_clk_get() / 1000000, rt_hw_periph800M_clk_get() / 1000000);
    LOG_D("DDR CLK: %luMHz", rt_hw_ddr_clk_get() / 1000000);
    LOG_D("AHB CLK: %luMHz", rt_hw_ahb_clk_get() / 1000000);
    LOG_D("APB CLK: (APB0) = %luMHz, (APB1) = %luMHz", rt_hw_apb0_clk_get() / 1000000, rt_hw_apb1_clk_get() / 1000000);

    rt_pin_mode(LED2, PIN_MODE_OUTPUT);
    while (1)
    {
        rt_pin_write(LED2, PIN_LOW);
        rt_thread_mdelay(75);
        rt_pin_write(LED2, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(LED2, PIN_LOW);
        rt_thread_mdelay(75);
        rt_pin_write(LED2, PIN_HIGH);
        rt_thread_mdelay(650);
    }

    return 0;
}
