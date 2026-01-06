/* SPDX-License-Identifier: GPL-2.0 */

#include "hal_common.h"
#include "hal_interrupt.h"

uint32_t platform_get_gic_dist_base(void)
{
    return GIC_DISTRIBUTOR_BASE_ADDR;
}

uint32_t platform_get_gic_cpu_base(void)
{
    return GIC_INTERFACE_BASE_ADDR;
}

int32_t hal_request_irq(int32_t irq, hal_isr_handler_t handler, const char *name, void *data)
{
    rt_hw_interrupt_install(irq, handler, data, name);
    return 0;
}

void hal_free_irq(int32_t irq)
{
    rt_hw_interrupt_mask(irq);
}

int hal_enable_irq(int32_t irq)
{
    rt_hw_interrupt_umask(irq);
    return 0;
}

void hal_disable_irq(int32_t irq)
{
    rt_hw_interrupt_mask(irq);
}

unsigned long hal_interrupt_is_disable(void)
{
    unsigned long result;
    __asm__ volatile ("mrs %0, cpsr" : "=r" (result) );
    if (result & 0x80)
        return 1;
    return 0;
}

uint32_t hal_interrupt_get_nest(void)
{
    uint32_t nest = rt_interrupt_get_nest();
    return nest;
}

//void hal_interrupt_enable(void)
//{
//    asm volatile(
//        " cpsie i                 @ arch_local_irq_enable"
//        :
//        :
//        : "memory", "cc");
//}
//
//void hal_interrupt_disable(void)
//{
//    asm volatile(
//        " cpsid i                 @ arch_local_irq_disable"
//        :
//        :
//        : "memory", "cc");
//}

unsigned long hal_interrupt_disable_irqsave(void)
{
    return rt_hw_interrupt_disable();
}

void hal_interrupt_enable_irqrestore(unsigned long flag)
{
    return rt_hw_interrupt_enable(flag);
}

//void hal_interrupt_enter(void)
//{
//    rt_interrupt_enter();
//}
//
//void hal_interrupt_leave(void)
//{
//    rt_interrupt_leave();
//}


