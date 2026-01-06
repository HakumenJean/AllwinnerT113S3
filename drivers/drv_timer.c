/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"
#include "interrupt.h"
#include "drv_clock.h"

#include "drv_timer.h"

static void hal_hw_timer_init(void)
{
    hal_timer_init();
    hal_htimer_init();
}

static void rt_hw_timer_isr(void *param)
{
    rt_tick_increase();
}

int rt_hw_timer_init(void)
{
    hal_hw_timer_init();

    hal_timer_set(SUNXI_TMR0, 1000, rt_hw_timer_isr, NULL);
    hal_timer_start(SUNXI_TMR0, true);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_timer_init);

#ifdef RT_USING_FINSH

int cnt = 0;
int tm_cnt = 0;
int id = 0;

void sunxi_timer_handler(void *param)
{
    if(++cnt >= tm_cnt) hal_timer_stop(id);
    rt_kprintf("sunxi timer running, cnt : %d\r\n", cnt);
}

static int timer_test(int args, char *argv[])
{
    id = atoi(argv[1]);
    int clk = atoi(argv[2]);
    tm_cnt = atoi(argv[3]);

    cnt = 0;

    hal_timer_set(id, clk, sunxi_timer_handler, RT_NULL);
    hal_timer_start(id, true);

    return 0;
}
MSH_CMD_EXPORT(timer_test, test sunxi timer);

void sunxi_htimer_handler(void *param)
{
    if(++cnt >= tm_cnt) hal_htimer_stop(id);
    rt_kprintf("sunxi htimer running, cnt : %d\r\n", cnt);
}

static int htimer_test(int args, char *argv[])
{
    id = atoi(argv[1]);
    int clk = atoi(argv[2]);
    tm_cnt = atoi(argv[3]);

    cnt = 0;

    hal_htimer_set(id, clk, sunxi_htimer_handler, RT_NULL);
    hal_htimer_start(id, true);

    return 0;
}
MSH_CMD_EXPORT(htimer_test, test sunxi htimer);

#endif

