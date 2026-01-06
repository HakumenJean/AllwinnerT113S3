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

#ifdef RT_USING_SMP

#include <stdint.h>
#include <rthw.h>
#include "board.h"
#include "gic.h"

#include <interrupt.h>

#include "drv_timer.h"
#include "hal_interrupt.h"

#ifndef readl
#define readl(addr)           (*(volatile unsigned int *)(addr))
#endif
#ifndef writel
#define writel(value,addr)    (*(volatile unsigned int *)(addr) = (value))
#endif

extern void rt_secondary_cpu_entry(void);

static void rt_hw_timer1_isr(void *param)
{
    rt_tick_increase();
}

/*
The Soft Entry Address Register of CPU0 is 0x070005C4.
The Soft Entry Address Register of CPU1 is 0x070005C8.
*/
void rt_hw_secondary_cpu_up(void)
{
    uint32_t cpuboot_membase = 0x070005c4;
    uint32_t cpuxcfg_membase = 0x09010000;
    uint32_t cpu, reg;

    cpu = 1;
    /* Set CPU boot address */
    writel((uint32_t)(rt_secondary_cpu_entry), cpuboot_membase + 4 * cpu);

    /* Deassert the CPU core in reset */
    reg = readl(cpuxcfg_membase);
    writel(reg | (1 << cpu), cpuxcfg_membase);

    rt_hw_dsb();

    rt_hw_ipi_send(0, 1 << 1);
}

extern size_t MMUTable[];

void rt_hw_secondary_cpu_bsp_start(void)
{
    __asm__ volatile("mrc p15, 0, r1, c1, c0, 1\n"
                     "mov r0, #(1<<6)\n"
                     "orr r1, r0\n"
                     "mcr p15, 0, r1, c1, c0, 1\n"
                     "\n"
                     "mrc p15, 0, r0, c1, c0, 0\n"
                     "orr r0, r0, #(1<<11)\n"
                     "mcr p15, 0, r0, c1, c0, 0"
                    );

    rt_hw_vector_init();

    rt_hw_spin_lock(&_cpus_lock);
    rt_uint32_t mmutable_p;
    mmutable_p = (rt_uint32_t)MMUTable + (rt_uint32_t)PV_OFFSET;
    rt_hw_mmu_switch((void *)mmutable_p);

    arm_gic_cpu_init(0, GIC_INTERFACE_BASE_ADDR);
    arm_gic_set_cpu(0, SUNXI_IRQ_TIMER1, 0x2);

    hal_timer_set(SUNXI_TMR1, 1000, rt_hw_timer1_isr, NULL);
    hal_timer_start(SUNXI_TMR1, true);

    rt_system_scheduler_start();
}

void rt_hw_secondary_cpu_idle_exec(void)
{
     asm volatile ("wfe":::"memory", "cc");
}

#endif /* RT_USING_SMP*/


