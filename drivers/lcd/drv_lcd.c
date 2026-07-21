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

/*
 * Double-buffer design:
 *   buf_pool[0] and buf_pool[1] are two independent framebuffers.
 *   At any moment, TCON scans from buf_pool[front_idx] and the application
 *   draws to buf_pool[!front_idx] (exposed as lcd_info.framebuffer).
 *
 *   On RECT_UPDATE, the TCON layer address is updated to point to the
 *   back buffer (via ioctl in thread context), the indices are swapped,
 *   and the old front buffer becomes the new draw buffer.
 *   This eliminates the memcpy entirely — zero pixel copy per frame.
 */
typedef struct lcd_device
{
    struct rt_device lcd;
    struct rt_device_graphic_info lcd_info;     /* rtdef.h */
    struct rt_event lcd_evt;
    int use_screen;                  /* screen index */
    atomic_uint refresh_flag;        /* VSYNC handshake: 1 = pending swap */
    atomic_uint front_idx;           /* which buf_pool[] TCON is scanning (0 or 1) */

    rt_uint32_t *buf_pool[2];        /* double buffer: [0] and [1] alternate as front/back */
    struct disp_layer_config layer_cfg; /* cached layer config for address-only updates */
}*lcd_device_t;

static struct lcd_device _lcd_device;

extern void rt_hw_cpu_dcache_clean(void *addr, int size);
extern int disp_ioctl(int cmd, void *arg);
extern int disp_probe(void);

/*
 * VSYNC callback — runs in ISR context, keep it minimal.
 * The actual buffer swap and TCON address update are done in
 * RTGRAPHIC_CTRL_RECT_UPDATE (thread context), because disp_ioctl()
 * uses a mutex and cannot be called from ISR.
 *
 * refresh_flag serves as a handshake:
 *   RECT_UPDATE sets it to 1 after updating the TCON address,
 *   VSYNC ISR clears it and sends the event to unblock the app.
 */
s32 lcd_vsync_event_process(u32 sel)
{
    lcd_device_t lcd_drv = &_lcd_device;

    if (atomic_exchange(&lcd_drv->refresh_flag, 0) != 0)
    {
        rt_event_send(&lcd_drv->lcd_evt, LCD_EVT_VSYNC);
    }

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

    /*
     * The event is used for the synchronization between updating the
     * framebuffer and VSYNC.
     */
    rt_event_init(&lcd_drv->lcd_evt, "lcd_evt", RT_IPC_FLAG_FIFO);

    /* the lcd device information defined by RT-Thread */
    arg[0] = lcd_drv->use_screen;
    lcd_drv->lcd_info.width = (rt_uint16_t)disp_ioctl(DISP_GET_SCN_WIDTH, arg);
    lcd_drv->lcd_info.height = (rt_uint16_t)disp_ioctl(DISP_GET_SCN_HEIGHT, arg);
    lcd_drv->lcd_info.bits_per_pixel = 32;
    lcd_drv->lcd_info.pixel_format = RTGRAPHIC_PIXEL_FORMAT_ARGB888;
    lcd_drv->lcd_info.smem_len = LCD_DRV_FB_SZ;

    /* Allocate two buffers for double buffering */
    for (int i = 0; i < 2; i++)
    {
        lcd_drv->buf_pool[i] = (rt_uint32_t *)rt_malloc(LCD_DRV_FB_SZ);
        if (!lcd_drv->buf_pool[i])
        {
            rt_kprintf("malloc buf_pool[%d] fail\n", i);
            goto out;
        }
        rt_memset(lcd_drv->buf_pool[i], 0, LCD_DRV_FB_SZ);
        rt_hw_cpu_dcache_clean(lcd_drv->buf_pool[i], LCD_DRV_FB_SZ);
    }

    /*
     * Initial state:
     *   TCON scans from buf_pool[0] (front_idx = 0)
     *   Application draws to buf_pool[1] (framebuffer → buf_pool[1])
     */
    atomic_init(&lcd_drv->front_idx, 0);
    atomic_init(&lcd_drv->refresh_flag, 0);
    lcd_drv->lcd_info.framebuffer = (rt_uint8_t *)lcd_drv->buf_pool[1];

    return RT_EOK;

out:
    for (int i = 0; i < 2; i++)
    {
        if (lcd_drv->buf_pool[i])
        {
            rt_free(lcd_drv->buf_pool[i]);
            lcd_drv->buf_pool[i] = RT_NULL;
        }
    }

    return -RT_ERROR;
}

/*
 * Update the TCON layer buffer address. Called from THREAD context only
 * (never from ISR), because disp_ioctl uses a mutex internally.
 * The DE shadow register mechanism ensures the address takes effect
 * at the next VSYNC, guaranteeing tear-free transition.
 */
static int _lcd_update_layer_addr(lcd_device_t lcd_drv, rt_uint32_t *new_buf)
{
    unsigned long arg[6] = {0};
    struct disp_layer_config *cfg = &lcd_drv->layer_cfg;
    int w = lcd_drv->lcd_info.width;
    int h = lcd_drv->lcd_info.height;

    cfg->info.fb.addr[0] = (size_t)new_buf;

    /* INTERLEAVED address calculation */
    cfg->info.fb.addr[0] = (unsigned long long)(cfg->info.fb.addr[0] + w * h / 3 * 0);
    cfg->info.fb.addr[1] = (unsigned long long)(cfg->info.fb.addr[0] + w * h / 3 * 1);
    cfg->info.fb.addr[2] = (unsigned long long)(cfg->info.fb.addr[0] + w * h / 3 * 2);
    cfg->info.fb.trd_right_addr[0] = (unsigned int)(cfg->info.fb.addr[0] + w * h * 3 / 2);
    cfg->info.fb.trd_right_addr[1] = (unsigned int)(cfg->info.fb.trd_right_addr[0] + w * h);
    cfg->info.fb.trd_right_addr[2] = (unsigned int)(cfg->info.fb.trd_right_addr[0] + w * h * 3 / 2);

    arg[0] = lcd_drv->use_screen;
    arg[1] = (unsigned long)cfg;
    arg[2] = 1;
    arg[3] = 0;

    return disp_ioctl(DISP_LAYER_SET_CONFIG, (void *)arg);
}

/*
 * Build the layer config template and set the initial TCON buffer address.
 * The template is cached in lcd_drv->layer_cfg and later only the address
 * fields are patched via _lcd_update_layer_addr().
 */
static int _lcd_layer_init(lcd_device_t lcd_drv)
{
    int format;
    int ret;
    struct disp_layer_config *cfg = &lcd_drv->layer_cfg;
    int w = lcd_drv->lcd_info.width;
    int h = lcd_drv->lcd_info.height;

    format = _lcd_format_get(lcd_drv->lcd_info.pixel_format);
    if (format < 0)
    {
        rt_kprintf("lcd init fail pixel_format:%d\n", lcd_drv->lcd_info.pixel_format);
        return -RT_ERROR;
    }

    /* Build template — address fields will be patched per-frame */
    rt_memset(cfg, 0, sizeof(*cfg));
    cfg->channel                 = de_feat_get_num_vi_chns(lcd_drv->use_screen);
    cfg->layer_id                = 0;
    cfg->info.b_trd_out          = 0;
    cfg->info.fb.format          = format;
    cfg->info.fb.crop.x          = 0;
    cfg->info.fb.crop.y          = 0;
    cfg->info.fb.crop.width      = (long long)w << 32;
    cfg->info.fb.crop.height     = (long long)h << 32;
    cfg->info.fb.align[0]        = 4;
    cfg->info.fb.size[0].width   = w;
    cfg->info.fb.size[0].height  = h;
    cfg->info.fb.size[1].width   = w;
    cfg->info.fb.size[1].height  = h;
    cfg->info.fb.size[2].width   = w;
    cfg->info.fb.size[2].height  = h;
    cfg->info.mode               = 0; /* LAYER_MODE_BUFFER */
    cfg->info.alpha_mode         = 1;
    cfg->info.alpha_value        = 255;
    cfg->info.zorder             = 0;
    cfg->info.screen_win.x       = 0;
    cfg->info.screen_win.y       = 0;
    cfg->info.screen_win.width   = w;
    cfg->info.screen_win.height  = h;
    cfg->enable                  = 1;

    /* Point TCON at buf_pool[0] initially */
    ret = _lcd_update_layer_addr(lcd_drv, lcd_drv->buf_pool[0]);
    if (ret != 0)
    {
        rt_kprintf("fail to set layer cfg %d\n", ret);
        return -RT_ERROR;
    }

    /* Enable VSYNC event */
    {
        unsigned long arg[6] = {0};
        arg[0] = lcd_drv->use_screen;
        arg[1] = 1; /* enable */
        arg[2] = 0;
        ret = disp_ioctl(DISP_VSYNC_EVENT_EN, (void *)arg);
        if (ret != 0)
        {
            rt_kprintf("fail to set vsync enable %d\n", ret);
            return -RT_ERROR;
        }
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
                /*
                 * Double-buffer swap in THREAD context.
                 *
                 * Flow:
                 *  1. Determine which buffer the app just drew to (back)
                 *  2. Clean dcache for that buffer
                 *  3. Update TCON layer address → back buffer (via ioctl, safe in thread context)
                 *  4. Arm refresh_flag and wait for VSYNC
                 *  5. After VSYNC: swap indices. Old front → new draw buffer
                 */
                uint32_t front = atomic_load(&lcd_drv->front_idx);
                uint32_t back  = front ^ 1;

                /* Clean the back buffer dcache (app has been drawing to it) */
                rt_hw_cpu_dcache_clean(lcd_drv->buf_pool[back], LCD_DRV_FB_SZ);

                /* Clear stale VSYNC events before arming */
                rt_event_recv(&lcd_drv->lcd_evt, LCD_EVT_VSYNC,
                              RT_EVENT_FLAG_CLEAR | RT_EVENT_FLAG_OR, 0, NULL);

                /*
                 * Tell TCON to scan from the back buffer.
                 * The DE shadow register mechanism ensures this takes effect
                 * at the NEXT VSYNC — no tearing.
                 */
                _lcd_update_layer_addr(lcd_drv, lcd_drv->buf_pool[back]);

                /* Arm: VSYNC ISR will send the event when the switch is done */
                atomic_store(&lcd_drv->refresh_flag, 1);

                /* Wait for VSYNC confirmation */
                rt_err_t result = rt_event_recv(&lcd_drv->lcd_evt, LCD_EVT_VSYNC,
                                                RT_EVENT_FLAG_CLEAR | RT_EVENT_FLAG_OR,
                                                RT_TICK_PER_SECOND / 20, NULL);
                if (result != RT_EOK)
                {
                    rt_kprintf("RECT_UPDATE wait VSYNC timeout:%d\n", result);
                    break; /* Don't swap — hardware didn't latch the new address */
                }

                /*
                 * Swap: the back buffer is now the front (TCON is scanning it),
                 * and the old front buffer becomes available for drawing.
                 *
                 * Sync the new draw buffer with the current display content.
                 * Without this, partial updates would leave stale data from
                 * two frames ago, causing artifacts.
                 *
                 * The memcpy runs in thread context (not ISR) and writes to a
                 * buffer TCON is NOT reading from — no tearing.
                 */
                atomic_store(&lcd_drv->front_idx, back);
                lcd_drv->lcd_info.framebuffer = (rt_uint8_t *)lcd_drv->buf_pool[front];

                /*
                 * Sync the new draw buffer with the current display content.
                 * Without this, partial updates would leave stale data from
                 * two frames ago, causing artifacts.
                 *
                 * If the application always does full-frame rendering,
                 * these two lines can be commented out to save the 1.5MB
                 * memcpy per frame, achieving true zero-copy double buffering.
                 */
                rt_memcpy(lcd_drv->buf_pool[front], lcd_drv->buf_pool[back], LCD_DRV_FB_SZ);
                rt_hw_cpu_dcache_clean(lcd_drv->buf_pool[front], LCD_DRV_FB_SZ);

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

    LOG_I("graphic device rgb lcd init success (double buffer)");

    extern uint32_t test_image[384000];
    for(int j = 0; j < 480; j++) {
        for(int i = 0; i < 800; i++) {
            *((volatile uint32_t *)lcd_drv->lcd_info.framebuffer + i + j * lcd_drv->lcd_info.width) = test_image[i + j * 800]; //0xFFFF00FF;
        }
    }

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, lcd_drv->lcd_info.framebuffer, LCD_DRV_FB_SZ);
    rt_lcd_control((rt_device_t)lcd_drv, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);

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



