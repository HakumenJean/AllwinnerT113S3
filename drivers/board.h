/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include <stdint.h>
#include "mmu.h"
#include "hal_interrupt.h"
#include "drv_clock.h"
#include "drv_gpio.h"

extern unsigned char __bss_start;
extern unsigned char __bss_end;

#define RT_HW_HEAP_BEGIN    (void*)&__bss_end
#define RT_HW_HEAP_END      (void*)(0x40000000 + 128 * 1024 * 1024)

/* the definition needed by gic.c */
#define __REG32(x)  (*((volatile unsigned int *)(x)))

extern struct mem_desc platform_mem_desc[];
extern const rt_uint32_t platform_mem_desc_size;

void rt_hw_board_init(void);

#endif
