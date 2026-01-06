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

#include "board.h"
#include "drv_g2d.h"

#define DBG_TAG  "g2d"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "g2d_driver.h"

extern int sunxi_g2d_control(int cmd, void *arg);
extern int sunxi_g2d_close(void);
extern int sunxi_g2d_open(void);
extern int g2d_probe(void);

static rt_err_t sunxi_g2d_init(rt_device_t dev)
{
    return 0;
}

static rt_err_t sunxi_g2d_open_wrap(rt_device_t dev, rt_uint16_t oflag)
{
    return sunxi_g2d_open();
}

static rt_err_t sunxi_g2d_close_wrap(rt_device_t dev)
{
    return sunxi_g2d_close();
}

static rt_err_t sunxi_g2d_control_wrap(rt_device_t dev, int cmd, void *args)
{
    return sunxi_g2d_control(cmd, args);
}

#ifdef RT_USING_DEVICE_OPS

const static struct rt_device_ops g2d_ops = {
    sunxi_g2d_init,
    sunxi_g2d_open_wrap,
    sunxi_g2d_close_wrap,
    RT_NULL,
    RT_NULL,
    sunxi_g2d_control_wrap
};

#endif

static int rt_hw_g2d_init(void)
{
    int ret = -1;
    rt_device_t device;

    device = rt_device_create(RT_Device_Class_Graphic, 0);
    if (!device) {
        return ret;
    }
    rt_memset(device, 0, sizeof(struct rt_device));

#ifdef RT_USING_DEVICE_OPS
    device->ops = &g2d_ops;
#else
    device->init = sunxi_g2d_init;
    device->open = sunxi_g2d_open_wrap;
    device->close = sunxi_g2d_close_wrap;
    device->control = sunxi_g2d_control_wrap;
#endif

    ret = rt_device_register(device, "g2d", RT_DEVICE_FLAG_RDWR);
    if (ret != 0) {
        return ret;
    }

    g2d_probe();

    LOG_I("graphic device g2d init ok");

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_g2d_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

#define IMG_SIZE1 32*16
#define X1 32
#define Y1 16

void g2d_blend_test(int argc, char **argv)
{
    int ret, i;
    int *src = NULL, *src2 = NULL, *dst = NULL;
    g2d_bld info;

    //unsigned int src_w, src_h, src2_w, src2_h, dst_w, dst_h;

    rt_kprintf("hello g2d blending test\r\n");

    sunxi_g2d_open();
    //note:the tests follow need three  input image file
    src = rt_malloc(IMG_SIZE1 * 4);
    src2 = rt_malloc(IMG_SIZE1 * 4);
    dst = rt_malloc(IMG_SIZE1 * 4);

    if(src == NULL || src2 == NULL || dst == NULL) {
        rt_kprintf("fatal error, buf is null.\n");
        ret = -1;
        goto error;
    }

    rt_memset(src, 0, IMG_SIZE1 * 4);
    rt_memset(src2, 0, IMG_SIZE1 * 4);
    rt_memset(dst, 0, IMG_SIZE1 * 4);
    rt_memset(&info, 0, sizeof(g2d_bld));

    //Test1------------------------------------------------------------------
    // src = src2 = dst RGB
    info.src_image[0].laddr[0] = __va_to_pa((uint32_t)src);
    info.src_image[1].laddr[0] = __va_to_pa((uint32_t)src2);
    info.dst_image.laddr[0] = __va_to_pa((uint32_t)dst);

    info.src_image[0].laddr[0] = (unsigned int)(info.src_image[0].laddr[0]);
    info.src_image[0].laddr[1] = 0;
    info.src_image[0].laddr[2] = 0;
    info.src_image[0].use_phy_addr = 1;

    info.src_image[1].laddr[0] = (unsigned int)(info.src_image[1].laddr[0]);
    info.src_image[1].laddr[1] = 0;
    info.src_image[1].laddr[2] = 0;
    info.src_image[1].use_phy_addr = 1;

    info.dst_image.laddr[0] = (unsigned int)(info.dst_image.laddr[0]);
    info.dst_image.laddr[1] = 0;
    info.dst_image.laddr[2] = 0;
    info.dst_image.use_phy_addr = 1;

    info.bld_cmd = G2D_BLD_SRCATOP;//G2D_BLD_SRCOVER;

    for (i = 0; i < 2; i++) {
        info.src_image[i].format = G2D_FORMAT_ARGB8888;
        info.src_image[i].mode = G2D_GLOBAL_ALPHA;
        info.src_image[i].width = X1;
        info.src_image[i].height = Y1;
        info.src_image[i].clip_rect.x = 0;
        info.src_image[i].clip_rect.y = 0;
        info.src_image[i].clip_rect.w = X1;
        info.src_image[i].clip_rect.h = Y1;
        info.src_image[i].coor.x = 0;
        info.src_image[i].coor.y = 0;
    }

    info.src_image[0].alpha = 0xd0;
    info.src_image[1].alpha = 0x50;

    info.dst_image.format = G2D_FORMAT_ARGB8888;
    info.dst_image.mode = G2D_GLOBAL_ALPHA;
    info.dst_image.alpha = 0xff;
    info.dst_image.width = X1;
    info.dst_image.height = Y1;
    info.dst_image.clip_rect.x = 0;
    info.dst_image.clip_rect.y = 0;
    info.dst_image.clip_rect.w = X1;
    info.dst_image.clip_rect.h = Y1;

    for(i = 0; i < IMG_SIZE1; i++)
    {
        src[i] = 0xFF808080 + i;
        src2[i] = 0xFF606060 + i;
    }

    //we use hal_malloc for iamge input,so we need to make sure caches flushed
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, src, IMG_SIZE1 * 4);
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, src2, IMG_SIZE1 * 4);
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, dst, IMG_SIZE1 * 4);

    rt_kprintf("start control\r\n");

    ret = sunxi_g2d_control(G2D_CMD_BLD_H, &info);
    if (ret) {
        rt_kprintf("g2d G2D_CMD_BLD_H fail\r\n");
        ret = -1;
        goto error;
    } else {
        rt_kprintf("G2D_CMD_BLD_H ok\r\n");
    }

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, dst, IMG_SIZE1 * 4);

    rt_kprintf("\r\nblend data: ");
    for(i = 0; i < IMG_SIZE1; i++)
    {
        if(i % 16 == 0) rt_kprintf("\r\n");
        rt_kprintf("%X ", dst[i]);
    }

    rt_kprintf("\r\n\r\ng2d blend test finished!\r\n");

error:
    rt_free(src);
    rt_free(src2);
    rt_free(dst);
    sunxi_g2d_close();
}
MSH_CMD_EXPORT(g2d_blend_test, test g2d blend);

void g2d_fillrect_test(int argc, char **argv)
{
    int ret, i;
    int *buf1 = NULL;

    g2d_fillrect_h info;

    rt_kprintf("hello g2d fillrect test\n");
    ret = sunxi_g2d_open();
    if (ret) {
        rt_kprintf("g2d open fail\n");
        return;
    }

    buf1 = rt_malloc(IMG_SIZE1 * 4);
    rt_memset(&info, 0, sizeof(g2d_fillrect_h));
    rt_memset((void *)buf1, 0, IMG_SIZE1 * 4);

    info.dst_image_h.format = G2D_FORMAT_ARGB8888;
    info.dst_image_h.width = X1;
    info.dst_image_h.height = Y1;
    info.dst_image_h.clip_rect.x = 0;
    info.dst_image_h.clip_rect.y = 0;
    info.dst_image_h.clip_rect.w = X1;
    info.dst_image_h.clip_rect.h = Y1;
    info.dst_image_h.color = 0xFFFF0000;
    info.dst_image_h.mode = 1;
    info.dst_image_h.alpha = 255;
    info.dst_image_h.laddr[0] = __va_to_pa((uint32_t)buf1);
    info.dst_image_h.use_phy_addr = 1;

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, buf1, IMG_SIZE1 * 4);

    rt_kprintf("start control\n");

    ret = sunxi_g2d_control(G2D_CMD_FILLRECT_H, &info);
    if (ret){
        rt_kprintf("g2d G2D_CMD_FILLRECT_H fail\n");
        }
    else {
        rt_kprintf("G2D_CMD_FILLRECT_H ok\n");
    }

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, buf1, IMG_SIZE1 * 4);

    rt_kprintf("\r\nfillrect data: ");
    for(i = 0; i < IMG_SIZE1; i++)
    {
        if(i % 16 == 0) rt_kprintf("\r\n");
        rt_kprintf("%X ", buf1[i]);
    }

    rt_kprintf("\r\n\r\ng2d fillrect test finished!\r\n");

    rt_free(buf1);
    sunxi_g2d_close();
}
MSH_CMD_EXPORT(g2d_fillrect_test, test g2d fillrect);

void g2d_bitblt_test(int argc, char **argv)
{
    int ret, i;
    int *buf1 = NULL,*buf2 = NULL;
    g2d_blt_h blit_para;

    rt_kprintf("hello g2d bitblt test\n");
    ret = sunxi_g2d_open();
    if (ret) {
        rt_kprintf("g2d open fail\n");
        return;
    }

    buf1 = rt_malloc(IMG_SIZE1*4);
    buf2 = rt_malloc(IMG_SIZE1*4);

    rt_memset(&blit_para, 0, sizeof(blit_para));
    rt_memset((void *)buf1, 0, IMG_SIZE1 * 4);
    rt_memset((void *)buf2, 0, IMG_SIZE1 * 4);

    blit_para.flag_h = G2D_ROT_0;

    blit_para.src_image_h.laddr[0] = __va_to_pa((uint32_t)buf1);
    blit_para.src_image_h.format = G2D_FORMAT_ARGB8888;
//    blit_para.src_image_h.mode = G2D_GLOBAL_ALPHA;
//    blit_para.src_image_h.alpha = 0xFF;
    blit_para.src_image_h.width = X1;
    blit_para.src_image_h.height = Y1;
    blit_para.src_image_h.clip_rect.x = 0;
    blit_para.src_image_h.clip_rect.y = 0;
    blit_para.src_image_h.clip_rect.w = X1;
    blit_para.src_image_h.clip_rect.h = Y1;

    blit_para.dst_image_h.laddr[0] = __va_to_pa((uint32_t)buf2);
    blit_para.dst_image_h.format = G2D_FORMAT_ARGB8888;
//    blit_para.dst_image_h.mode = G2D_GLOBAL_ALPHA;
//    blit_para.dst_image_h.alpha = 0xFF;
    blit_para.dst_image_h.width = X1;
    blit_para.dst_image_h.height = Y1;
    blit_para.dst_image_h.clip_rect.x = 0;
    blit_para.dst_image_h.clip_rect.y = 0;
    blit_para.dst_image_h.clip_rect.w = X1;
    blit_para.dst_image_h.clip_rect.h = Y1;

    for(i = 0; i < X1 * Y1; i++)
    {
        buf1[i] = 0xFF808080;
    }

    //we use hal_malloc for iamge input,so we need to make sure caches flushed
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, buf1, IMG_SIZE1 * 4);
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, buf2, IMG_SIZE1 * 4);

    rt_kprintf("start control\n");

    ret = sunxi_g2d_control(G2D_CMD_BITBLT_H, &blit_para);
    if (ret) {
        rt_kprintf("g2d G2D_CMD_BITBLT_H fail\n");
        ret = -1;
    }
    else {
        rt_kprintf("G2D_CMD_BITBLT_H ok\n");
    }

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, buf2, IMG_SIZE1 * 4);

    rt_kprintf("\r\nbitblt data: ");
    for(i = 0; i < IMG_SIZE1; i++)
    {
        if(i % 16 == 0) rt_kprintf("\r\n");
        rt_kprintf("%X ", buf2[i]);
    }

    rt_kprintf("\r\n\r\ng2d bitblt test finished!\r\n");

    rt_free(buf1);
    rt_free(buf2);
    sunxi_g2d_close();
}
MSH_CMD_EXPORT(g2d_bitblt_test, test g2d bitblt);

#endif

