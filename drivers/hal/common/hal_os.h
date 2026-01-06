/* SPDX-License-Identifier: GPL-2.0 */

#ifndef DRIVERS_HAL_COMMON_HAL_OS_H_
#define DRIVERS_HAL_COMMON_HAL_OS_H_

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "posix/string.h"

#define hal_msleep              rt_thread_mdelay
#define hal_usleep              rt_hw_us_delay
#define hal_udelay              rt_hw_us_delay

#define hal_strcmp              rt_strcmp
#define hal_strncmp             rt_strncmp
#define hal_strlen              rt_strlen
#define hal_strncpy             rt_strncpy

#define hal_sprintf             rt_sprintf
#define hal_snprintf            rt_snprintf
#define hal_printf              rt_kprintf

#define hal_soft_break(...)                     \
    do {                                        \
        asm volatile ("ebreak": : :"memory");   \
    }while(0)

/* -------------------------------- jiffies -----------------------------*/
#define HZ 100
#define jiffies ((unsigned long)rt_tick_get())

#ifndef isb
#define isb(option)             __asm__ __volatile__ ("isb " #option : : : "memory")
#endif
#ifndef dsb
#define dsb(option)             __asm__ __volatile__ ("dsb " #option : : : "memory")
#endif
#ifndef dmb
#define dmb(option)             __asm__ __volatile__ ("dmb " #option : : : "memory")
#endif

#ifndef barrier
#define barrier()               __asm__ __volatile__("": : :"memory")
#endif

/**********************************************************
 * critical
 *
 *********************************************************/
#define hal_enter_critical      rt_enter_critical
#define hal_exit_critical       rt_exit_critical

/**********************************************************
 * memory
 *
 *********************************************************/

#define hal_malloc              rt_malloc
#define hal_free                rt_free
#define hal_malloc_align        rt_malloc_align
#define hal_free_align          rt_free_align

#define hal_memcmp              rt_memcmp
#define hal_memcpy              rt_memcpy
#define hal_memset              rt_memset

void hal_free_coherent(void *addr);
void *hal_malloc_coherent(size_t size);
void hal_free_coherent_align(void *addr);
void *hal_malloc_coherent_align(size_t size, int align);

/**********************************************************
 * cache
 *
 *********************************************************/
inline void hal_dcache_clean(void *addr, int size);
inline void hal_dcache_invalidate(void *addr, int size);

/**********************************************************
 * IPC
 *
 *********************************************************/

#define HAL_WAIT_FOREVER        RT_WAITING_FOREVER

typedef struct rt_semaphore     hal_sem;
typedef rt_sem_t                hal_sem_t;
typedef rt_mailbox_t            hal_mailbox_t;
typedef rt_mq_t                 hal_queue_t;
typedef struct rt_workqueue     hal_workqueue;
typedef struct rt_work          hal_work;

void hal_sem_init(hal_sem_t sem, unsigned int cnt);
void hal_sem_deinit(hal_sem_t sem);
hal_sem_t hal_sem_create(unsigned int cnt);
int hal_sem_delete(hal_sem_t sem);
int hal_sem_wait(hal_sem_t sem);
#define hal_sem_post            rt_sem_release
#define hal_sem_timedwait       rt_sem_take

#define hal_work_init rt_work_init
hal_workqueue *hal_workqueue_create(const char *name, unsigned short stack_size, unsigned char priority);
int hal_workqueue_dowork(hal_workqueue *queue, hal_work *work);

/**********************************************************
 * thread
 *
 *********************************************************/

typedef rt_thread_t hal_thread_t;
typedef struct rt_thread hal_thread;

typedef rt_tick_t               hal_tick_t;
#define hal_tick_get            rt_tick_get
#define OSTICK_TO_MS(x)         (x)
#define MS_TO_OSTICK(x)         (x)

#define HAL_THREAD_PRIORITY_APP     (4)
#define HAL_THREAD_PRIORITY_CLI     (3)
#define HAL_THREAD_PRIORITY_SYS     (3)
#define HAL_THREAD_PRIORITY_NET     (3)

#define HAL_THREAD_PRIORITY_HIGHEST (0)
#define HAL_THREAD_PRIORITY_LOWEST  (31)
#define HAL_THREAD_PRIORITY_MIDDLE  (15)

#define HAL_THREAD_STACK_SIZE    (0x2000)
#define HAL_THREAD_TIMESLICE     (    10)

#define HAL_THREAD_PRIORITY      HAL_THREAD_PRIORITY_APP

void *hal_thread_create(void (*threadfn)(void *data), void *data, const char *namefmt, int stacksize, int priority);
int hal_thread_stop(void *thread);
int hal_thread_start(void *thread);
void *hal_thread_self(void);
int hal_thread_resume(void *thread);
int hal_thread_suspend(void *thread);
int hal_thread_msleep(int ms);
int hal_thread_sleep(int tick);
int hal_thread_scheduler_is_running(void);
int hal_thread_is_in_critical_context(void);
void hal_thread_tick_increase(void);
char *hal_thread_get_name(void *thread);
int hal_thread_scheduler_suspend(void);
int hal_thread_scheduler_resume(void);

#define hal_thread_run(threadfn, data, namefmt, ...)               \
({                                     \
    void *__k                          \
        = hal_thread_create(threadfn, data, namefmt, HAL_THREAD_STACK_SIZE, HAL_THREAD_PRIORITY_SYS); \
    if (__k)                   \
        hal_thread_start(__k);                     \
    __k;                                   \
})

#endif /* DRIVERS_HAL_COMMON_HAL_OS_H_ */
