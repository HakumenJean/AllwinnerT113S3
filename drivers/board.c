/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_clock.h"
#include "drv_dma.h"

#define DMA_MEM     (SHARED|AP_RW|DOMAIN0|MEMWT|DESC_SEC)

struct mem_desc platform_mem_desc[] = {
    {0x00000000, 0x40000000, 0x00000000, DEVICE_MEM},
    {0x40000000, 0x41000000, 0x40000000, NORMAL_MEM},
    {0x41000000, 0x41800000, 0x41000000, DMA_MEM},
    {0x41800000, 0x48000000, 0x41800000, NORMAL_MEM}
};

const rt_uint32_t platform_mem_desc_size = sizeof(platform_mem_desc)/sizeof(platform_mem_desc[0]);

void idle_wfi(void)
{
    asm volatile ("wfi");
}

void rt_hw_board_init(void)
{
    rt_hw_init_mmu_table(platform_mem_desc, platform_mem_desc_size);
    rt_hw_mmu_init();

#ifdef RT_USING_HEAP
    /* init memory system */
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

    rt_hw_interrupt_init();
    rt_hw_dma_init();

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#ifdef RT_USING_CONSOLE
    /* set console device */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif /* RT_USING_CONSOLE */

    rt_thread_idle_sethook(idle_wfi);

#ifdef RT_USING_SMP
    /* install IPI handle */
    extern void rt_hw_ipi_handler_install(int ipi_vector, rt_isr_handler_t ipi_isr_handler);
    rt_hw_ipi_handler_install(RT_SCHEDULE_IPI, rt_scheduler_ipi_handler);
#endif
}
