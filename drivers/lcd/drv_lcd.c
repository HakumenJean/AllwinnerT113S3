/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include <stdlib.h>
#include "rtthread.h"
#include <rthw.h>
#include "rtdevice.h"
#include <string.h>
#include <stdatomic.h>

#include "interrupt.h"
#include "mmu.h"
#include "cache.h"

#define DBG_TAG "lcd"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef BSP_USING_LCD

#include "drv_lcd.h"
#include "lcd_cfg.h"

#include "dev_disp.h"
#include "sunxi_display2.h"

#define DEFAULT_SCREEN (0)

#define LCD_DRV_FB_SZ (lcd_drv->lcd_info.width * lcd_drv->lcd_info.height * sizeof(rt_uint32_t))

enum lcd_evt
{
    LCD_EVT_VSYNC = 1,
};

/* kind of a candidate for the official lcd driver framework */
typedef struct lcd_device
{
    struct rt_device lcd;
    struct rt_device_graphic_info lcd_info;     /* rtdef.h */
    struct rt_event lcd_evt;
    int use_screen;                  /* screen index */
    atomic_uint refresh_flag;        /* atom flag,  0: nothing. 1:framebuffer ==> front_buf. */

    rt_uint32_t *front_buff;         /* TCON hardware framebuffer */
}*lcd_device_t;

static struct lcd_device _lcd_device;

extern void rt_hw_cpu_dcache_clean(void *addr, int size);
extern int disp_ioctl(int cmd, void *arg);
extern int disp_probe(void);

s32 lcd_vsync_event_process(u32 sel)
{
    lcd_device_t lcd_drv = &_lcd_device;
    uint32_t refresh_flag = atomic_exchange(&lcd_drv->refresh_flag, 0); // read-modify-write, read & clean.

    if (refresh_flag != 0)
    {
        uint32_t len = LCD_DRV_FB_SZ;
        void *dst = lcd_drv->front_buff;
        const void *src = lcd_drv->lcd_info.framebuffer;

        rt_memcpy((uint32_t *)dst, (uint32_t *)src, len);
        rt_hw_cpu_dcache_clean(dst, len);

#if 0
        uint32_t len_stage1 = 1024;
        rt_memcpy((uint32_t *)dst, (uint32_t *)src, len_stage1);
        rt_hw_cpu_dcache_clean(dst, len_stage1);

        rt_memcpy((uint32_t *)(dst + len_stage1), (uint32_t *)(src + len_stage1), len - len_stage1);
        rt_hw_cpu_dcache_clean((uint32_t *)(dst + len_stage1), len - len_stage1);
#endif

        rt_event_send(&lcd_drv->lcd_evt, LCD_EVT_VSYNC);
    }

    rt_event_send(&lcd_drv->lcd_evt, LCD_EVT_VSYNC);

    return 0;
}

/* pixel format, only 565 (2 bytes) or 666 (4 bytes) are supported */
static inline int _lcd_format_get(rt_uint8_t pixel_format)
{
    switch (pixel_format)
    {
        case RTGRAPHIC_PIXEL_FORMAT_RGB565:
            return DISP_FORMAT_RGB_565;
        case RTGRAPHIC_PIXEL_FORMAT_ARGB888:
            return DISP_FORMAT_ARGB_8888;
        default:
            return -1;
    }
}

static int _lcd_drv_init(lcd_device_t lcd_drv)
{
    unsigned long arg[6] = {0};
    void *framebuffer = RT_NULL;
    void *frontbuf = RT_NULL;
    void *backbuf = RT_NULL;

    /*
     * The event is used for the synchronization between updating the
     * framebuffer and flushing the screen.
     */
    rt_event_init(&lcd_drv->lcd_evt, "lcd_evt", RT_IPC_FLAG_FIFO);

    /* the lcd device information defined by RT-Thread */
    arg[0] = lcd_drv->use_screen;
    lcd_drv->lcd_info.width = (rt_uint16_t)disp_ioctl(DISP_GET_SCN_WIDTH, arg);
    lcd_drv->lcd_info.height = (rt_uint16_t)disp_ioctl(DISP_GET_SCN_HEIGHT, arg);
    lcd_drv->lcd_info.bits_per_pixel = 32;
    lcd_drv->lcd_info.pixel_format = RTGRAPHIC_PIXEL_FORMAT_ARGB888; /* should be coherent to adding layers */

    /* allocate the framebuffer, the front buffer and the back buffer */
    /* framebuffer */
    framebuffer = rt_malloc(LCD_DRV_FB_SZ);
    if (!framebuffer)
    {
        rt_kprintf("malloc framebuffer fail\n");
        goto out;
    }
    lcd_drv->lcd_info.framebuffer = framebuffer;
    lcd_drv->lcd_info.smem_len = LCD_DRV_FB_SZ;
    rt_memset(framebuffer, 0, LCD_DRV_FB_SZ);
    rt_hw_cpu_dcache_clean(lcd_drv->lcd_info.framebuffer, LCD_DRV_FB_SZ);

    frontbuf = rt_malloc(LCD_DRV_FB_SZ);
    if (!frontbuf)
    {
        rt_kprintf("malloc frontbuf fail\n");
        goto out;
    }
    lcd_drv->front_buff = frontbuf;
    rt_memset(frontbuf, 0xFFFFFFFF, LCD_DRV_FB_SZ);
    rt_hw_cpu_dcache_clean(lcd_drv->front_buff, LCD_DRV_FB_SZ);

    return RT_EOK;
out:
    if (framebuffer)
    {
        rt_free(framebuffer);
    }

    if (frontbuf)
    {
        rt_free(frontbuf);
    }

    if (backbuf)
    {
        rt_free(backbuf);
    }

    return -RT_ERROR;
}

static int _lcd_layer_init(lcd_device_t lcd_drv)
{
    int format;
    int ret;
    unsigned long arg[6] = {0};
    static struct disp_layer_config layer_cfg;

    format = _lcd_format_get(lcd_drv->lcd_info.pixel_format);
    if (format < 0)
    {
        rt_kprintf("lcd init faile pixel_format:%d\n", lcd_drv->lcd_info.pixel_format);
        return -RT_ERROR;
    }

    // config layer info
    rt_memset(&layer_cfg, 0, sizeof(layer_cfg));
    layer_cfg.info.b_trd_out = 0;
    layer_cfg.channel = de_feat_get_num_vi_chns(lcd_drv->use_screen); // skip vi channel
    layer_cfg.layer_id = 0;
    layer_cfg.info.fb.format = format;
    layer_cfg.info.fb.crop.x = 0;
    layer_cfg.info.fb.crop.y = 0;
    layer_cfg.info.fb.crop.width = lcd_drv->lcd_info.width;
    layer_cfg.info.fb.crop.height = lcd_drv->lcd_info.height;
    layer_cfg.info.fb.crop.width = layer_cfg.info.fb.crop.width << 32;
    layer_cfg.info.fb.crop.height = layer_cfg.info.fb.crop.height << 32;
    layer_cfg.info.fb.align[0] = 4;
    layer_cfg.info.mode = 0; // LAYER_MODE_BUFFER
    layer_cfg.info.alpha_mode = 1;
    layer_cfg.info.alpha_value = 255;
    layer_cfg.info.zorder = 0;
    layer_cfg.info.screen_win.x = 0;
    layer_cfg.info.screen_win.y = 0;
    layer_cfg.info.screen_win.width = lcd_drv->lcd_info.width;
    layer_cfg.info.screen_win.height = lcd_drv->lcd_info.height;

    layer_cfg.info.fb.size[0].width = lcd_drv->lcd_info.width;
    layer_cfg.info.fb.size[0].height = lcd_drv->lcd_info.height;
    layer_cfg.info.fb.size[1].width = lcd_drv->lcd_info.width;
    layer_cfg.info.fb.size[1].height = lcd_drv->lcd_info.height;
    layer_cfg.info.fb.size[2].width = lcd_drv->lcd_info.width;
    layer_cfg.info.fb.size[2].height = lcd_drv->lcd_info.height;

    layer_cfg.info.fb.addr[0] = (size_t)lcd_drv->front_buff;

    /* INTERLEAVED */
    layer_cfg.info.fb.addr[0] = (unsigned long long)(layer_cfg.info.fb.addr[0] + lcd_drv->lcd_info.width * lcd_drv->lcd_info.height / 3 * 0);
    layer_cfg.info.fb.addr[1] = (unsigned long long)(layer_cfg.info.fb.addr[0] + lcd_drv->lcd_info.width * lcd_drv->lcd_info.height / 3 * 1);
    layer_cfg.info.fb.addr[2] = (unsigned long long)(layer_cfg.info.fb.addr[0] + lcd_drv->lcd_info.width * lcd_drv->lcd_info.height / 3 * 2);
    layer_cfg.info.fb.trd_right_addr[0] = (unsigned int)(layer_cfg.info.fb.addr[0] + lcd_drv->lcd_info.width * lcd_drv->lcd_info.height * 3 / 2);
    layer_cfg.info.fb.trd_right_addr[1] = (unsigned int)(layer_cfg.info.fb.trd_right_addr[0] + lcd_drv->lcd_info.width * lcd_drv->lcd_info.height);
    layer_cfg.info.fb.trd_right_addr[2] = (unsigned int)(layer_cfg.info.fb.trd_right_addr[0] + lcd_drv->lcd_info.width * lcd_drv->lcd_info.height * 3 / 2);

    layer_cfg.enable = 1;

    arg[0] = lcd_drv->use_screen;
    arg[1] = (unsigned long)&layer_cfg;
    arg[2] = 1;
    arg[3] = 0;
    ret = disp_ioctl(DISP_LAYER_SET_CONFIG, (void *)arg);
    if (0 != ret)
    {
        rt_kprintf("fail to set layer cfg %d\n", ret);
        return -RT_ERROR;
    }

    arg[0] = lcd_drv->use_screen;
    arg[1] = 1; // enable
    arg[2] = 0;
    ret = disp_ioctl(DISP_VSYNC_EVENT_EN, (void *)arg);
    if (0 != ret)
    {
        rt_kprintf("fail to set vsync enable %d\n", ret);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/* Add the first layer, then enable the interrupt */
static rt_err_t rt_lcd_init(rt_device_t dev)
{
    static int lcd_init = 0;
    lcd_device_t lcd_drv = (lcd_device_t)dev;

    RT_ASSERT(lcd_drv != RT_NULL);

    if (lcd_init) return RT_EOK;
    lcd_init = 1;

    load_lcd_config();
    if (disp_probe() != 0)
    {
        rt_kprintf("lcd disp probe failure\n");
        return -RT_ERROR;
    }

    if (_lcd_drv_init(lcd_drv) != RT_EOK)
    {
        rt_kprintf("lcd drv init failure\n");
        return -RT_ERROR;
    }

    if (_lcd_layer_init(lcd_drv) != RT_EOK)
    {
        rt_kprintf("disp layer init failure\n");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t rt_lcd_control(rt_device_t dev, int cmd, void *args)
{
    struct lcd_device *lcd_drv = (struct lcd_device *)dev;

    switch (cmd)
    {
        case RTGRAPHIC_CTRL_RECT_UPDATE:
            {
                // memcpy?

                // clean event.
                rt_event_recv(&lcd_drv->lcd_evt, LCD_EVT_VSYNC, RT_EVENT_FLAG_CLEAR | RT_EVENT_FLAG_OR, 0, NULL);

                atomic_store(&lcd_drv->refresh_flag, 1); // lcd_drv->refresh_flag = 1;

                // wait irq
                rt_err_t result = rt_event_recv(&lcd_drv->lcd_evt, LCD_EVT_VSYNC, RT_EVENT_FLAG_CLEAR | RT_EVENT_FLAG_OR, RT_TICK_PER_SECOND / 20, NULL);
                if (result != RT_EOK)
                {
                    rt_kprintf("RTGRAPHIC_CTRL_RECT_UPDATE wait LCD_EVT_VSYNC:%d\n", result);
                }

                break;
            }
        case RTGRAPHIC_CTRL_POWERON:
            disp_ioctl(DISP_LCD_BACKLIGHT_ENABLE, RT_NULL);
            break;
        case RTGRAPHIC_CTRL_POWEROFF:
            disp_ioctl(DISP_LCD_BACKLIGHT_DISABLE, RT_NULL);
            break;
        case RTGRAPHIC_CTRL_SET_BRIGHTNESS:
            {
                unsigned long arg[6] = {0};
                arg[1] = *((rt_uint8_t *)args);
                disp_ioctl(DISP_LCD_SET_BRIGHTNESS, arg);
            }
            break;

        case RTGRAPHIC_CTRL_GET_BRIGHTNESS:
            *((rt_uint8_t *)args) = disp_ioctl(DISP_LCD_GET_BRIGHTNESS, RT_NULL);
            break;
        case RTGRAPHIC_CTRL_GET_INFO:
            rt_memcpy(args, &lcd_drv->lcd_info, sizeof(struct rt_device_graphic_info));
            break;
        case RTGRAPHIC_CTRL_SET_MODE:
            break;
    }
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS

const static struct rt_device_ops lcd_ops = {
    rt_lcd_init,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_lcd_control
};

#endif

/* set up the 'lcd_device' and register it */
int rt_hw_lcd_init(void)
{
    struct lcd_device *lcd_drv = &_lcd_device;

    /* the content of tcon control registers can be loaded from a xml file ? */
    // _panel = load_config_from_xml();
    rt_memset(lcd_drv, 0, sizeof(struct lcd_device));

    lcd_drv->use_screen = DEFAULT_SCREEN;

    /* initialize device structure, the type of 'lcd' is 'rt_device' */
    lcd_drv->lcd.type = RT_Device_Class_Graphic;
#ifdef RT_USING_DEVICE_OPS
    lcd_drv->lcd.ops = &lcd_ops;
#else
    lcd_drv->lcd.init = rt_lcd_init;
    lcd_drv->lcd.open = RT_NULL;
    lcd_drv->lcd.close = RT_NULL;
    lcd_drv->lcd.control = rt_lcd_control;
#endif
    lcd_drv->lcd.user_data = (void *)&lcd_drv->lcd_info;

    /* register lcd device to RT-Thread */
    rt_device_register(&lcd_drv->lcd, "lcd", RT_DEVICE_FLAG_RDWR);

    rt_lcd_init((rt_device_t)lcd_drv);

    LOG_I("graphic device rgb lcd init success");

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_lcd_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

static int lcd_draw_point(int args, char *argv[])
{
    struct lcd_device *lcd_drv = &_lcd_device;
    int x = 0;
    int y = 0;
    int i, k;

    rt_kprintf("lcd_draw_point\n");

    x = atoi(argv[1]);
    y = atoi(argv[2]);

    if (x >= lcd_drv->lcd_info.width)
        x = lcd_drv->lcd_info.width - 1;
    if (y >= lcd_drv->lcd_info.height)
        y = lcd_drv->lcd_info.height - 1;
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;

    rt_kprintf("Darw point is x:%d,y:%d\n", x, y);

    for (i = y - 100; i < y + 100; i++)
    {
        if (i < 0)
            continue;
        if (i >= lcd_drv->lcd_info.height)
            break;
        for (k = x - 100; k < x + 100; k++)
        {
            if (k < 0)
                continue;
            if (k >= lcd_drv->lcd_info.width)
                break;

            *((uint32_t *)lcd_drv->lcd_info.framebuffer + lcd_drv->lcd_info.width * i + k) = 0xff00ff00;
        }
    }

    *((uint32_t *)lcd_drv->lcd_info.framebuffer + lcd_drv->lcd_info.width * y + x) = 0xffff0000;
    // *((uint32_t *)lcd->lcd_info.framebuffer + lcd_drv->lcd_info.width * y + x + 2) = 0xff00ff00;

    rt_hw_cpu_dcache_clean(lcd_drv->lcd_info.framebuffer, LCD_DRV_FB_SZ);
    rt_lcd_control((rt_device_t)lcd_drv, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);

    return 0;
}
MSH_CMD_EXPORT(lcd_draw_point, draw a point on lcd);

void lcd_pwm_test(int argc, char **argv)
{
    int value = atoi(argv[1]);
    struct lcd_device *lcd_drv = &_lcd_device;
    rt_lcd_control((rt_device_t)lcd_drv, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &value);
}

MSH_CMD_EXPORT(lcd_pwm_test, set pwm);
#endif

#endif



