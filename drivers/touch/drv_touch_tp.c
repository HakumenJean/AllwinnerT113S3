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
#include <string.h>

#include "interrupt.h"
#include "drv_touch.h"
#include "drv_clock.h"

#define DBG_TAG "tp0"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef RT_USING_TOUCH

int perform_calibration(struct tscal_t * cal)
{
    float n, x, y, x2, y2, xy, z, zx, zy;
    float det, a, b, c, e, f, i;
    float scaling = 65536.0;
    int j;

    n = x = y = x2 = y2 = xy = 0;
    for(j = 0; j < 5; j++)
    {
        n += 1.0;
        x += (float)cal->x[j];
        y += (float)cal->y[j];
        x2 += (float)(cal->x[j]*cal->x[j]);
        y2 += (float)(cal->y[j]*cal->y[j]);
        xy += (float)(cal->x[j]*cal->y[j]);
    }

    det = n * (x2 * y2 - xy * xy) + x * (xy * y - x * y2) + y * (x * xy - y * x2);
    if(det < 0.1 && det > -0.1)
        return 0;

    a = (x2 * y2 - xy * xy) / det;
    b = (xy * y - x * y2) / det;
    c = (x * xy - y * x2) / det;
    e = (n * y2 - y * y) / det;
    f = (x * y - n * xy) / det;
    i = (n * x2 - x * x) / det;

    z = zx = zy = 0;
    for(j = 0; j < 5; j++)
    {
        z += (float)cal->xfb[j];
        zx += (float)(cal->xfb[j] * cal->x[j]);
        zy += (float)(cal->xfb[j] * cal->y[j]);
    }

    cal->a[0] = (int)((b * z + e * zx + f * zy) * (scaling));
    cal->a[1] = (int)((c * z + f * zx + i * zy) * (scaling));
    cal->a[2] = (int)((a * z + b * zx + c * zy) * (scaling));

    z = zx = zy = 0;
    for(j = 0; j < 5; j++)
    {
        z += (float)cal->yfb[j];
        zx += (float)(cal->yfb[j] * cal->x[j]);
        zy += (float)(cal->yfb[j] * cal->y[j]);
    }

    cal->a[3] = (int)((b * z + e * zx + f * zy) * (scaling));
    cal->a[4] = (int)((c * z + f * zx + i * zy) * (scaling));
    cal->a[5] = (int)((a * z + b * zx + c * zy) * (scaling));

    cal->a[6] = (int)scaling;
    return 1;
}

#ifdef BSP_USING_TOUCH_TP

#include "hal_tpadc.h"

static rt_int32_t pre_x = 0;
static rt_int32_t pre_y = 0;
static rt_uint8_t s_tp_dowm = 0;
static rt_uint8_t ignore_fifo_data = 1;
static rt_int32_t calibration[7] = {8305, -5, -1442028, -67, 5075, -1893492, 65536};

static void sunxi_tp_xy_correction(rt_int32_t *x, rt_int32_t *y)
{
    *x = (calibration[2] + calibration[0] * *x + calibration[1] * *y) / calibration[6];
    *y = (calibration[5] + calibration[3] * *x + calibration[4] * *y) / calibration[6];
}

static rt_size_t sunxi_tp_read_point(struct rt_touch_device *touch, void *buf, rt_size_t touch_num)
{
    rt_device_t device = (rt_device_t)&touch->parent;
    struct rt_touch_data *touch_data = (struct rt_touch_data *)device->user_data;
    rt_memcpy(buf, touch_data, sizeof(struct rt_touch_data));
    return 1;
}

static rt_err_t sunxi_tp_control(struct rt_touch_device *touch, int cmd, void *arg)
{
    switch(cmd)
    {
        case RT_TOUCH_CTRL_GET_ID: break;
        case RT_TOUCH_CTRL_GET_INFO: break;
        case RT_TOUCH_CTRL_SET_X_RANGE: break;
        case RT_TOUCH_CTRL_SET_Y_RANGE: break;
        case RT_TOUCH_CTRL_SET_X_TO_Y: break;
        case RT_TOUCH_CTRL_SET_MODE:
            {
                rt_uint16_t trig_type = *(rt_uint16_t *)arg;
                if(trig_type == RT_DEVICE_FLAG_INT_RX) {
                    hal_tpadc_resume();
                }
            }
            break;
        case RT_TOUCH_CTRL_SET_CALIBRATION:
            rt_memcpy(calibration, arg, 7 * sizeof(rt_int32_t));
            break;
        default: break;
    }

    return RT_EOK;
}

static struct rt_touch_ops tp_touch_ops =
{
    .touch_readpoint = sunxi_tp_read_point,
    .touch_control = sunxi_tp_control,
};

void rt_touch_tpadc_usercallback(void *param, uint32_t x_data, uint32_t y_data, data_flag_t flag)
{
    rt_int32_t x = 0, y = 0;
    rt_touch_t touch = (rt_touch_t)param;
    rt_device_t device = (rt_device_t)&touch->parent;
    struct rt_touch_data *touch_data = (struct rt_touch_data *)device->user_data;

    if(flag == DATA_UP) {
        if(s_tp_dowm) touch_data->event = RT_TOUCH_EVENT_UP;
        else touch_data->event = RT_TOUCH_EVENT_NONE;
        s_tp_dowm = 0;
        ignore_fifo_data = 1;
        touch_data->x_coordinate = pre_x;
        touch_data->y_coordinate = pre_y;
        LOG_D("touch event up : x %d y %d", pre_x, pre_y);
    } else {
        x = x_data;
        y = y_data;
        if(!ignore_fifo_data)
        {
            sunxi_tp_xy_correction(&x, &y);
            if(!s_tp_dowm)
            {
                s_tp_dowm = 1;
                touch_data->x_coordinate = x;
                touch_data->y_coordinate = y;
                touch_data->event = RT_TOUCH_EVENT_DOWN;
            }
            else
            {
                if((pre_x != x) || (pre_y != y))
                {
                    touch_data->x_coordinate = x;
                    touch_data->y_coordinate = y;
                    touch_data->event = RT_TOUCH_EVENT_MOVE;
                }
            }
            pre_x = x;
            pre_y = y;
            LOG_D("touch event down : x %d y %d", x, y);
        }
        else
        {
            ignore_fifo_data = 0;
        }
    }
}

int rt_hw_touch_tp_init(void)
{
    struct rt_touch_device *touch_device = RT_NULL;

    touch_device = (struct rt_touch_device *)rt_malloc(sizeof(struct rt_touch_device));
    if(touch_device == RT_NULL)
    {
        LOG_E("touch device malloc fail");
        return -RT_ERROR;
    }
    rt_memset((void *)touch_device, 0, sizeof(struct rt_touch_device));

    if(hal_tpadc_init() != RT_EOK)
    {
        LOG_E("touch tp0 hw init fail");
        return -RT_ERROR;
    }

    hal_tpadc_register_callback(rt_touch_tpadc_usercallback, touch_device);

    /* register touch device */
    touch_device->info.type = RT_TOUCH_TYPE_RESISTANCE;
    touch_device->info.vendor = RT_TOUCH_VENDOR_UNKNOWN;
    touch_device->config.irq_pin.pin = PIN_NONE;

    touch_device->ops = &tp_touch_ops;
    touch_device->irq_handle = RT_NULL;

    struct rt_touch_data *touch_data = (struct rt_touch_data *)rt_malloc(sizeof(struct rt_touch_data));
    rt_hw_touch_register(touch_device, "tp0", RT_DEVICE_FLAG_INT_RX, touch_data);

    LOG_I("touch device tp0 init success");

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_touch_tp_init);

void rt_touch_tp_init(void)
{
    rt_device_t tp_dev = rt_device_find("tp0");
    if(tp_dev == RT_NULL)
    {
        LOG_E("Could not find touch device tp0");
        return;
    }

    if (rt_device_open(tp_dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Could not open touch device tp0");
        return;
    }

    rt_uint16_t control_flag = RT_DEVICE_FLAG_INT_RX;
    rt_device_control(tp_dev, RT_TOUCH_CTRL_SET_MODE, &control_flag);
}

#endif

#endif
