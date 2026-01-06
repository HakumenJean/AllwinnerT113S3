/* SPDX-License-Identifier: GPL-2.0 */

#include "hal_os.h"
#include "hal_atomic.h"

///* TODO: handle lock */
//void hal_spin_lock(hal_spinlock_t *lock)
//{
//    rt_enter_critical();
//    return;
//}
//
///* TODO: handle lock */
//void hal_spin_unlock(hal_spinlock_t *lock)
//{
//    rt_exit_critical();
//    return;
//}
//
///* TODO: handle lock */
//uint32_t hal_spin_lock_irqsave(hal_spinlock_t *lock)
//{
//    uint32_t ret;
//    // CPSR_ALLOC();
//
//    // MELIS_CPU_CRITICAL_ENTER();
//    rt_enter_critical();
//    ret = rt_hw_interrupt_disable();
//
//    return ret;
//}
//
///* TODO: handle lock */
//void hal_spin_unlock_irqrestore(hal_spinlock_t *lock, uint32_t __cpsr)
//{
//    rt_hw_interrupt_enable(__cpsr);
//    rt_exit_critical();
//
//    // MELIS_CPU_CRITICAL_LEAVE();
//}

