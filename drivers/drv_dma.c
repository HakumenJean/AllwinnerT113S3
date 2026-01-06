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
#include <rtdevice.h>
#include <rthw.h>

#include "drv_dma.h"
#include "hal_dma.h"

#define DBG_TAG "dma"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

int rt_hw_dma_init(void)
{
    hal_dma_init();

    return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

#define DMA_TEST_LEN    1024
static void dma_test_cb(void *param)
{
    rt_kprintf("DMA finished, callback to do something...\n");
}

void dma_test(int argc, char **argv)
{
    int ret, i;
    struct sunxi_dma_chan *hdma = NULL;
    char *buf1 = NULL,*buf2 = NULL;
    struct dma_slave_config config = {0};
    uint32_t size = 0;

    rt_kprintf("run in dma test\r\n");

    buf2 = rt_malloc(DMA_TEST_LEN);
    buf1 = rt_malloc(DMA_TEST_LEN);

    rt_memset(buf1, 0, DMA_TEST_LEN);
    rt_memset(buf2, 0, DMA_TEST_LEN);

    for (i = 0;i < DMA_TEST_LEN; i++)
        buf1[i] = i & 0xff;

    /* request dma chan */
    ret = hal_dma_chan_request(&hdma);
    if (ret == HAL_DMA_CHAN_STATUS_BUSY) {
        rt_kprintf("dma channel busy!");
        goto end;
    }

    config.direction = DMA_MEM_TO_MEM;
    config.dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
    config.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
    config.dst_maxburst = DMA_SLAVE_BURST_16;
    config.src_maxburst = DMA_SLAVE_BURST_16;
    config.slave_id = sunxi_slave_id(DRQDST_SDRAM, DRQSRC_SDRAM);

    ret = hal_dma_slave_config(hdma, &config);

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, buf1, DMA_TEST_LEN);
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, buf2, DMA_TEST_LEN);

    if (ret != HAL_DMA_STATUS_OK) {
        rt_kprintf("dma config error, ret:%d", ret);
        goto end;
    }

    ret = hal_dma_prep_memcpy(hdma, (uint32_t)buf2, (uint32_t)buf1, DMA_TEST_LEN);
    if (ret != HAL_DMA_STATUS_OK) {
        rt_kprintf("dma prep error, ret:%d", ret);
        goto end;
    }

    /* register dma callback */
    ret = hal_dma_callback_install(hdma, dma_test_cb, hdma);
    if (ret != HAL_DMA_STATUS_OK) {
        rt_kprintf("register dma callback failed!");
        goto end;
    }

    ret = hal_dma_start(hdma);
    if (ret != HAL_DMA_STATUS_OK) {
        rt_kprintf("dma start error, ret:%d", ret);
        goto end;
    }

    while (hal_dma_tx_status(hdma, &size)!= 0);

    rt_kprintf("src buf:\n");
    for (i = 0;i < DMA_TEST_LEN; i++)
        rt_kprintf("0x%x ", buf1[i]);

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, buf2, DMA_TEST_LEN);

    rt_kprintf("dst buf:\n");
    for (i = 0;i < DMA_TEST_LEN; i++)
        rt_kprintf("0x%x ", buf2[i]);

    hal_dma_stop(hdma);

    ret = hal_dma_chan_free(hdma);
    if (ret != HAL_DMA_STATUS_OK) {
        rt_kprintf("dma free error, ret:%d", ret);
        goto end;
    }

end:
    rt_free(buf1);
    rt_free(buf2);
}
MSH_CMD_EXPORT(dma_test, set dma);

#endif

