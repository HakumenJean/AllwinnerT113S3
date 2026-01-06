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
#include <math.h>

#include "board.h"
#include "drv_clock.h"
#include "drv_timer.h"
#include "hal_clk.h"

uint64_t get_arch_counter(void)
{
    uint32_t low = 0, high = 0;
    asm volatile("mrrc p15, 0, %0, %1, c14" : "=r"(low), "=r"(high) : : "memory");
    return ((uint64_t)high << 32) | (uint64_t)low;
}

uint32_t time_ms(void)
{
    return get_arch_counter() / 24000;
}

uint64_t time_us(void)
{
    return get_arch_counter() / (uint64_t)24;
}

void rt_hw_us_delay(rt_uint32_t us)
{
    uint64_t now;

    now = time_us();
    while (time_us() - now < us);
}

void mdelay(uint32_t ms)
{
    uint32_t now;

    now = time_ms();
    while (time_ms() - now < ms);
}

void sdelay(uint32_t loops)
{
    __asm__ volatile("1:\n" "subs %0, %1, #1\n"
                     "bne 1b" : "=r"(loops) : "0"(loops));
}

int rt_hw_periph1x_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_PERIPH0));
}

int rt_hw_periph2x_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_PERIPH0_2X));
}

int rt_hw_periph800M_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_PERIPH0_800M));
}

int rt_hw_cpux_axi_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_CPUX));
}

int rt_hw_ddr_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_DDR0));
}

int rt_hw_ahb_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PSI_AHB));
}

int rt_hw_apb0_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_APB0));
}

int rt_hw_apb1_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_APB1));
}

int rt_hw_video0_4X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_VIDEO0_4X));
}

int rt_hw_video0_2X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_VIDEO0_2X));
}

int rt_hw_video0_1X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_VIDEO0));
}

int rt_hw_video1_4X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_VIDEO1_4X));
}

int rt_hw_video1_2X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_VIDEO1_2X));
}

int rt_hw_video1_1X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_VIDEO1));
}

int rt_hw_audio0_4X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_AUDIO0_4X));
}

int rt_hw_audio0_2X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_AUDIO0_2X));
}

int rt_hw_audio0_1X_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_AUDIO0));
}

int rt_hw_audio1_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_AUDIO1));
}

int rt_hw_audio1_div2_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_AUDIO1_DIV2));
}

int rt_hw_audio1_div5_clk_get(void)
{
    return hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_AUDIO1_DIV5));
}

int rt_hw_clock_init(void)
{
    hal_clock_init();
    hal_clk_set_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_CPUX), 1200000000);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_clock_init);
