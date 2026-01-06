/* SPDX-License-Identifier: GPL-2.0 */

#include "hal_common.h"
#include "hal_interrupt.h"

/**********************************************************
 * memory
 *
 *********************************************************/
void hal_free_coherent(void *addr)
{
    void *malloc_ptr = NULL;
    if (!addr)
    {
        return;
    }
    malloc_ptr = (void *) * (unsigned long *)((unsigned long *)addr - 1);
    hal_free(malloc_ptr);
}

void *hal_malloc_coherent(size_t size)
{
    void *fake_ptr = NULL;
    void *malloc_ptr = NULL;

    malloc_ptr = hal_malloc(size + 2 * CACHELINE_LEN);

    if ((unsigned long)malloc_ptr & (sizeof(long) - 1))
    {
        hal_printf("error: mm_alloc not align. \r\n");
        return NULL;
    }

    if (!malloc_ptr)
    {
        return NULL;
    }

    fake_ptr = (void *)((unsigned long)(malloc_ptr + CACHELINE_LEN) & (~(CACHELINE_LEN - 1)));
    *(unsigned long *)((unsigned long *)fake_ptr - 1) = (unsigned long)malloc_ptr;

    return fake_ptr;
}

void hal_free_coherent_align(void *addr)
{
    void *malloc_ptr = NULL;
    if (!addr)
    {
        return;
    }
    malloc_ptr = (void *) * (unsigned long *)((unsigned long *)addr - 1);
    hal_free(malloc_ptr);
}

void *hal_malloc_coherent_align(size_t size, int align)
{
    void *fake_ptr = NULL;
    void *malloc_ptr = NULL;

    malloc_ptr = hal_malloc(size + CACHELINE_LEN + align);
    if ((unsigned long)malloc_ptr & (sizeof(long) - 1))
    {
        hal_printf("error: mm_alloc not align. \r\n");
        return NULL;
    }

    if (!malloc_ptr)
    {
        return NULL;
    }

    fake_ptr = (void *)((unsigned long)(malloc_ptr + align) & (~(align - 1)));
    *(unsigned long *)((unsigned long *)fake_ptr - 1) = (unsigned long)malloc_ptr;

    return fake_ptr;
}

/**********************************************************
 * cache
 *
 *********************************************************/
inline void hal_dcache_clean(void *addr, int size)
{
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, addr, size);
}

inline void hal_dcache_invalidate(void *addr, int size)
{
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, addr, size);
}

/**********************************************************
 * IPC
 *
 *********************************************************/

void hal_sem_init(hal_sem_t sem, unsigned int cnt)
{
    rt_sem_init(sem, "hal_sem", cnt, RT_IPC_FLAG_FIFO);
}

void hal_sem_deinit(hal_sem_t sem)
{
    rt_sem_detach(sem);
}

hal_sem_t hal_sem_create(unsigned int cnt)
{
    return rt_sem_create("hal sem", cnt, RT_IPC_FLAG_FIFO);
}

int hal_sem_delete(hal_sem_t sem)
{
    rt_sem_delete(sem);
    sem = NULL;

    return HAL_OK;
}

int hal_sem_wait(hal_sem_t sem)
{
    rt_err_t ret;

    if (sem == NULL)
    {
        hal_printf("fatal error, parameter is invalid.");
        return -1;
    }

    ret = rt_sem_take(sem, RT_WAITING_FOREVER);
    if (ret != RT_EOK)
    {
        // timeout.
        return -2;
    }

    return 0;
}

hal_workqueue *hal_workqueue_create(const char *name, unsigned short stack_size, unsigned char priority)
{
    return rt_workqueue_create(name, stack_size, priority);
}

int hal_workqueue_dowork(hal_workqueue *queue, hal_work *work)
{
    int ret = 0;
    ret = rt_workqueue_dowork(queue, work);
    return ret;
}

int hal_workqueue_destroy(hal_workqueue *queue)
{
    return rt_workqueue_destroy(queue);
}


/**********************************************************
 * thread
 *
 *********************************************************/

void *hal_thread_create(void (*threadfn)(void *data), void *data, const char *namefmt, int stacksize, int priority)
{
    rt_thread_t thr;

    thr = rt_thread_create(namefmt, threadfn, data, \
                           stacksize, \
                           priority, \
                           HAL_THREAD_TIMESLICE);

    RT_ASSERT(thr != RT_NULL);

    return (void *)thr;
}

int hal_thread_start(void *thread)
{
    rt_thread_t thr;
    rt_err_t ret;

    RT_ASSERT(thread != RT_NULL);

    thr = (rt_thread_t)thread;

    ret = rt_thread_startup(thr);

    return ret;
}

int hal_thread_msleep(int ms)
{
    return rt_thread_mdelay(ms);
}

int hal_thread_sleep(int tick)
{
    return rt_thread_delay(tick);
}

int hal_thread_stop(void *thread)
{
    rt_thread_delete((rt_thread_t)thread);

    return 0;
}

char *hal_thread_get_name(void *thread)
{
    return RT_NULL;
}

int hal_thread_resume(void *thread)
{
    rt_thread_t thr;
    rt_err_t err;

    RT_ASSERT(thread != RT_NULL);

    thr = (rt_thread_t)thread;

    err = rt_thread_resume(thr);
    if (err)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

int hal_thread_suspend(void *thread)
{
    rt_thread_t thr;
    rt_err_t err;

    RT_ASSERT(thread != RT_NULL);

    thr = (rt_thread_t)thread;

    err = rt_thread_suspend(thr);
    if (err)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void *hal_thread_self(void)
{
    return (void *)rt_thread_self();
}

int hal_thread_scheduler_is_running(void)
{
    return (rt_critical_level() == 0);
}

int hal_thread_in_critical_context(void)
{
    if (hal_interrupt_get_nest())
        return 1;
    if (hal_thread_scheduler_is_running())
        return 1;
    if (hal_interrupt_is_disable())
        return 1;

    return 0;
}

void hal_thread_tick_increase(void)
{
    rt_tick_increase();
}

int hal_thread_scheduler_suspend(void)
{
    rt_enter_critical();
    return HAL_OK;
}

int hal_thread_scheduler_resume(void)
{
    rt_exit_critical();
    return HAL_OK;
}



