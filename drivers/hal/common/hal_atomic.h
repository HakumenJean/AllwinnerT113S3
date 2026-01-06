/* SPDX-License-Identifier: GPL-2.0 */

#ifndef DRIVERS_HAL_COMMON_HAL_ATOMIC_H_
#define DRIVERS_HAL_COMMON_HAL_ATOMIC_H_

#include "hal_os.h"

//typedef unsigned int melis_spinlock_t;
//typedef melis_spinlock_t hal_spinlock_t;

//#include <stdint.h>
//#include <stddef.h>
//
//void hal_spin_lock(hal_spinlock_t *lock);
//void hal_spin_unlock(hal_spinlock_t *lock);
//
//uint32_t hal_spin_lock_irqsave(hal_spinlock_t *lock);
//void hal_spin_unlock_irqrestore(hal_spinlock_t *lock, uint32_t __cpsr);

typedef rt_spinlock_t hal_spinlock_t;

#define HAL_SPINLOCK_INIT               RT_SPINLOCK_INIT
#define hal_spin_lock_init              rt_spin_lock_init
#define hal_spin_lock_deinit(x)         do{}while(0)
#define hal_spin_lock                   rt_spin_lock
#define hal_spin_unlock                 rt_spin_unlock
#define hal_spin_lock_irqsave           rt_spin_lock_irqsave
#define hal_spin_unlock_irqrestore      rt_spin_unlock_irqrestore

#endif /* DRIVERS_HAL_COMMON_HAL_ATOMIC_H_ */
