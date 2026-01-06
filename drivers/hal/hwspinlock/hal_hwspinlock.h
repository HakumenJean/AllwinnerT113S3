/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#ifndef DRIVERS_HAL_HWSPINLOCK_HAL_HWSPINLOCK_H_
#define DRIVERS_HAL_HWSPINLOCK_HAL_HWSPINLOCK_H_

#include "hal_clk.h"
#include "hal_reset.h"
#include "platform/hwspinlock-sun8iw20.h"

#define SPINLOCK_SYSTATUS_REG (SPIN_LOCK_BASE + 0x000)
#define SPINLOCK_STATUS_REG   (SPIN_LOCK_BASE + 0x010)
#define SPINLOCK_LOCK_REG(x)  (SPIN_LOCK_BASE + 0x100 + x * 4)

/* Possible values of SPINLOCK_LOCK_REG */
#define SPINLOCK_NOTTAKEN               (0)     /* free */
#define SPINLOCK_TAKEN                  (1)     /* locked */
#define SPINLOCK_NUM                    (32)    /* max lock num */
#define SPINLOCK_MAX_UDELAY             (200)
#define SPINLOCK_IRQ_ALL_ENABLE         (0xffffffff)

enum hwspinlock_err {
    HWSPINLOCK_OK = 0,
    HWSPINLOCK_ERR = -1,
    HWSPINLOCK_EXCEED_MAX = -2,
    HWSPINLOCK_PM_ERR = -3,
    HWSPINLOCK_TIMEOUT = -4,
};

#define SPINLOCK_CLI_UART_LOCK_BIT (0)

void hal_hwspinlock_init(void);
int hal_hwspinlock_put(int num);
int hal_hwspinlock_get(int num);
int hal_hwspinlock_check_taken(int num);

#endif /* DRIVERS_HAL_HWSPINLOCK_HAL_HWSPINLOCK_H_ */
