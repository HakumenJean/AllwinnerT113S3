/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include "hal_common.h"
#include "hal_interrupt.h"
#include "hal_log.h"
#include "hal_hwspinlock.h"

void hal_hwspinlock_init(void)
{
    hal_reset_type_t reset_type = HAL_SUNXI_RESET;
    u32 reset_id = RST_HWSPINLOCK;
    hal_clk_type_t clk_type = HAL_SUNXI_CCU;
    hal_clk_id_t clk_id = CLK_HWSPINLOCK;
    hal_clk_t clk;
    struct reset_control *reset;

    reset = hal_reset_control_get(reset_type, reset_id);
    hal_reset_control_deassert(reset);

    clk = hal_clock_get(clk_type, clk_id);
    hal_clock_enable(clk);
}

int hal_hwspinlock_check_taken(int num)
{
    return !!(readl(SPINLOCK_STATUS_REG) & (1 << num));
}

int hal_hwspinlock_get(int num)
{
    unsigned long addr = SPINLOCK_LOCK_REG(num);
    int status;

    if (num > SPINLOCK_NUM)
        return HWSPINLOCK_EXCEED_MAX;

    status = readl(addr);

    if (status == SPINLOCK_NOTTAKEN)
        return HWSPINLOCK_OK;

    return HWSPINLOCK_ERR;
}

int hal_hwspinlock_put(int num)
{
    unsigned long addr = SPINLOCK_LOCK_REG(num);

    if (num > SPINLOCK_NUM)
        return HWSPINLOCK_EXCEED_MAX;

    writel(SPINLOCK_NOTTAKEN, addr);

    return HWSPINLOCK_OK;
}

int hal_hwspin_lock(int num)
{
    if (num > SPINLOCK_NUM)
        return HWSPINLOCK_ERR;

    while(hal_hwspinlock_get(num) != HWSPINLOCK_OK);
    return HWSPINLOCK_OK;
}

int hal_hwspin_lock_timeout(int num, int ms_timeout)
{
    int us;

    hal_assert(!(hal_interrupt_get_nest() && ms_timeout == 0));

    if (num > SPINLOCK_NUM)
        return HWSPINLOCK_ERR;

    us = ms_timeout ? SPINLOCK_MAX_UDELAY : 0;
    while(hal_hwspinlock_get(num) != HWSPINLOCK_OK) {
        if (us-- > 0) {
            hal_udelay(1);
        } else if (ms_timeout-- > 0) {
            hal_msleep(1);
        } else {
            return HWSPINLOCK_TIMEOUT;
        }
    }
    return HWSPINLOCK_OK;
}

void hal_hwspin_unlock(int num)
{
    hal_hwspinlock_put(num);
}



