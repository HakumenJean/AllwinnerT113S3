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
#include <string.h>

#include "drv_gpio.h"
#include "drv_touch.h"
#include "drv_i2c.h"

#define DBG_TAG "gt9xx"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#if defined (RT_USING_TOUCH) && defined (BSP_USING_TOUCH_GT9XX)

#ifndef GT9XX_I2C_BUS_NAME
#define GT9XX_I2C_BUS_NAME "i2c2"
#endif

#define TP_INT_PIN GPIOD(22) // GPIOE(11) // GPIOD(22)
#define TP_RST_PIN GPIOG(10) // GPIOE(10) // GPIOG(10)


#define GT911_ADDR_LEN          2
#define GT911_REGITER_LEN       2
#define GT911_MAX_TOUCH         5
#define GT911_POINT_INFO_NUM    5

#define GT911_ADDRESS_HIGH      0x5D
#define GT911_ADDRESS_LOW       0x14

#define GT911_USE_LOW_ADDRESS   1

#define GT911_COMMAND_REG       0x8040
#define GT911_CONFIG_REG        0x8047

#define GT911_PRODUCT_ID        0x8140
#define GT911_VENDOR_ID         0x814A
#define GT911_READ_STATUS       0x814E

#define GT911_POINT1_REG        0x814F
#define GT911_POINT2_REG        0x8157
#define GT911_POINT3_REG        0x815F
#define GT911_POINT4_REG        0x8167
#define GT911_POINT5_REG        0x816F

#define GT911_CHECK_SUM         0x80FF

static struct rt_i2c_client gt911_client;

static rt_uint8_t GT911_CFG_TBL[] =
{
    // 此文件请勿随意改动，如滑动不对则需要修改中断触发方式

// f1c100s 配套4.3寸屏幕 480 * 272
    0xC9,0xE0,0x01,0x10,0x01,0x05,0x3D,0x20,0x22,0x08,
    0x28,0x08,0x5F,0x41,0x03,0x05,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x18,0x1A,0x1E,0x14,0x89,0x2A,0x09,
    0xCD,0xCF,0xB5,0x06,0x00,0x00,0x00,0x21,0x01,0x1C,
    0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0xB4,0xEF,0x9E,0xD5,0xF4,0x07,0x00,0x00,0x04,
    0x88,0xB9,0x00,0x83,0xC4,0x00,0x7F,0xCF,0x00,0x7B,
    0xDB,0x00,0x77,0xE8,0x00,0x77,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x10,0x12,
    0x14,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0x00,0x02,0x04,0x06,0x08,0x0A,0x0F,0x10,
    0x12,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,
    0x24,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0x00,0x00

// Binpi t113s3 配套5寸屏幕 800 * 480
//    0xC9,0x20,0x03,0xE0,0x01,0x05,0x3D,0x20,0x22,0x08,
//    0x28,0x08,0x5F,0x41,0x03,0x05,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x18,0x1A,0x1E,0x14,0x89,0x2A,0x09,
//    0xCD,0xCF,0xB5,0x06,0x00,0x00,0x00,0x42,0x01,0x1D,
//    0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0xB4,0xEF,0x9E,0xD5,0xF4,0x07,0x00,0x00,0x04,
//    0x87,0xB9,0x00,0x82,0xC4,0x00,0x7F,0xCF,0x00,0x7C,
//    0xDB,0x00,0x79,0xE8,0x00,0x79,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x10,0x12,
//    0x14,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0F,0x10,
//    0x12,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,
//    0x24,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00

// 淘宝购买 4.3寸屏 800 * 480
//    0x41,0x20,0x03,0xE0,0x01,0x05,0x0D,0x20,0x01,0x0A,
//    0x28,0x0F,0x5A,0x3C,0x03,0x05,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x8A,0x2A,0x0C,
//    0x32,0x34,0x0C,0x08,0x00,0x00,0x00,0x03,0x02,0x25,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x28,0x55,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
//    0x8D,0x2B,0x00,0x80,0x32,0x00,0x75,0x3A,0x00,0x6C,
//    0x43,0x00,0x64,0x4F,0x00,0x64,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x18,0x16,0x14,0x12,0x10,0x0E,0x0C,0x0A,
//    0x08,0x06,0x04,0x02,0xFF,0xFF,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x24,0x22,0x21,0x20,0x1F,0x1E,0x1D,0x1C,
//    0x18,0x16,0x13,0x12,0x10,0x0F,0x0A,0x08,0x06,0x04,
//    0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00

// rk3506 5寸配套屏幕 800 * 480
//    0x81,0x20,0x03,0xE0,0x01,0x05,0x0D,0x20,0x01,0x0A,
//    0x28,0x0F,0x5A,0x3C,0x03,0x05,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x8A,0x2A,0x0C,
//    0x32,0x34,0x0C,0x08,0x00,0x00,0x00,0x03,0x02,0x25,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x28,0x55,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
//    0x8D,0x2B,0x00,0x80,0x32,0x00,0x75,0x3A,0x00,0x6C,
//    0x43,0x00,0x64,0x4F,0x00,0x64,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x18,0x16,0x14,0x12,0x10,0x0E,0x0C,0x0A,
//    0x08,0x06,0x04,0x02,0xFF,0xFF,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x24,0x22,0x21,0x20,0x1F,0x1E,0x1D,0x1C,
//    0x18,0x16,0x13,0x12,0x10,0x0F,0x0A,0x08,0x06,0x04,
//    0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//    0x00,0x00,0x00,0x00,0x00,0x00
};

static rt_err_t gt911_write_reg(struct rt_i2c_client *dev, rt_uint8_t *data, rt_uint8_t len)
{
    struct rt_i2c_msg msgs;

    msgs.addr  = dev->client_addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf   = data;
    msgs.len   = len;

    if (rt_i2c_transfer(dev->bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

static rt_err_t gt911_read_regs(struct rt_i2c_client *dev, rt_uint8_t *reg, rt_uint8_t *data, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = dev->client_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = reg;
    msgs[0].len   = GT911_REGITER_LEN;

    msgs[1].addr  = dev->client_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = len;

    if (rt_i2c_transfer(dev->bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

static rt_err_t gt911_get_product_id(struct rt_i2c_client *dev, rt_uint8_t *data, rt_uint8_t len)
{
    rt_uint8_t reg[2];

    reg[0] = (rt_uint8_t)(GT911_PRODUCT_ID >> 8);
    reg[1] = (rt_uint8_t)(GT911_PRODUCT_ID & 0xff);

    if (gt911_read_regs(dev, reg, data, len) != RT_EOK)
    {
        LOG_E("read id failed");
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t gt911_get_info(struct rt_i2c_client *dev, struct rt_touch_info *info)
{
    rt_uint8_t reg[2];
    rt_uint8_t out_info[7];
    rt_uint8_t out_len = 7;

    reg[0] = (rt_uint8_t)(GT911_CONFIG_REG >> 8);
    reg[1] = (rt_uint8_t)(GT911_CONFIG_REG & 0xFF);

    if(gt911_read_regs(dev, reg, out_info, out_len) != RT_EOK)
    {
        LOG_E("read info failed");
        return -RT_ERROR;
    }

    rt_kprintf("version: %d\r\n", out_info[0]);

    info->range_x = (out_info[2] << 8) | out_info[1];
    info->range_y = (out_info[4] << 8) | out_info[3];
    info->point_num = out_info[5] & 0x0f;

    return RT_EOK;
}

rt_err_t gt911_soft_reset(struct rt_i2c_client *dev)
{
    rt_uint8_t buf[3];

    buf[0] = (rt_uint8_t)(GT911_COMMAND_REG >> 8);
    buf[1] = (rt_uint8_t)(GT911_COMMAND_REG & 0xFF);
    buf[2] = 0x02;

    if(gt911_write_reg(dev, buf, 3) != RT_EOK)
    {
        LOG_E("soft reset failed");
        return -RT_ERROR;
    }
    return RT_EOK;
}

static int16_t pre_x[GT911_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static int16_t pre_y[GT911_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static int16_t pre_w[GT911_MAX_TOUCH] = {-1, -1, -1, -1, -1};
static rt_uint8_t s_tp_dowm[GT911_MAX_TOUCH];
static struct rt_touch_data *read_data;

static void gt911_touch_up(void *buf, int8_t id)
{
    read_data = (struct rt_touch_data *)buf;

    if(s_tp_dowm[id] == 1)
    {
        s_tp_dowm[id] = 0;
        read_data[id].event = RT_TOUCH_EVENT_UP;
    }
    else
    {
        read_data[id].event = RT_TOUCH_EVENT_NONE;
    }

    //read_data[id].timestamp = rt_touch_get_ts();
    read_data[id].width = pre_w[id];
    read_data[id].x_coordinate = pre_x[id];
    read_data[id].y_coordinate = pre_y[id];
    read_data[id].track_id = id;

    pre_x[id] = -1;  /* last point is none */
    pre_y[id] = -1;
    pre_w[id] = -1;
}

static void gt911_touch_down(void *buf, int8_t id, int16_t x, int16_t y, int16_t w)
{
    read_data = (struct rt_touch_data *)buf;

    if (s_tp_dowm[id] == 1)
    {
        read_data[id].event = RT_TOUCH_EVENT_MOVE;

    }
    else
    {
        read_data[id].event = RT_TOUCH_EVENT_DOWN;
        s_tp_dowm[id] = 1;
    }

    //read_data[id].timestamp = rt_touch_get_ts();
    read_data[id].width = w;
    read_data[id].x_coordinate = x;
    read_data[id].y_coordinate = y;
    read_data[id].track_id = id;

    pre_x[id] = x; /* save last point */
    pre_y[id] = y;
    pre_w[id] = w;
}

static rt_size_t gt911_read_point(struct rt_touch_device *touch, void *buf, rt_size_t read_num)
{
    rt_uint8_t point_status = 0;
    rt_uint8_t touch_num = 0;
    rt_uint8_t write_buf[3];
    rt_uint8_t cmd[2];
    rt_uint8_t read_buf[8 * GT911_MAX_TOUCH] = {0};
    rt_uint8_t read_index;
    int8_t read_id = 0;
    int16_t input_x = 0;
    int16_t input_y = 0;
    int16_t input_w = 0;

    static rt_uint8_t pre_touch = 0;
    static int8_t pre_id[GT911_MAX_TOUCH] = {0};

    /* point status register */
    cmd[0] = (rt_uint8_t)((GT911_READ_STATUS >> 8) & 0xFF);
    cmd[1] = (rt_uint8_t)(GT911_READ_STATUS & 0xFF);

    if (gt911_read_regs(&gt911_client, cmd, &point_status, 1) != RT_EOK)
    {
        LOG_D("read point failed\n");
        read_num = 0;
        goto exit_;
    }

    if (point_status == 0)             /* no data */
    {
        read_num = 0;
        goto exit_;
    }

    if ((point_status & 0x80) == 0)    /* data is not ready */
    {
        read_num = 0;
        goto exit_;
    }

    touch_num = point_status & 0x0f;  /* get point num */

    if (touch_num > GT911_MAX_TOUCH) /* point num is not correct */
    {
        read_num = 0;
        goto exit_;
    }

    cmd[0] = (rt_uint8_t)((GT911_POINT1_REG >> 8) & 0xFF);
    cmd[1] = (rt_uint8_t)(GT911_POINT1_REG & 0xFF);

    /* read point num is touch_num */
    if(gt911_read_regs(&gt911_client, cmd, read_buf, read_num * GT911_POINT_INFO_NUM) !=RT_EOK)
    {
        LOG_D("read point failed\n");
        read_num = 0;
        goto exit_;
    }

    if(pre_touch > touch_num)                                       /* point up */
    {
        for (read_index = 0; read_index < pre_touch; read_index++)
        {
            rt_uint8_t j;

            for (j = 0; j < touch_num; j++)                          /* this time touch num */
            {
                read_id = read_buf[j * 8] & 0x0F;

                if (pre_id[read_index] == read_id)                   /* this id is not free */
                    break;

                if (j >= touch_num - 1)
                {
                    rt_uint8_t up_id;
                    up_id = pre_id[read_index];
                    gt911_touch_up(buf, up_id);
                }
            }
        }
    }

    if(touch_num)                                                 /* point down */
    {
        rt_uint8_t off_set;

        for(read_index = 0; read_index < touch_num; read_index++)
        {
            off_set = read_index * 8;
            read_id = read_buf[off_set] & 0x0f;
            pre_id[read_index] = read_id;
            input_x = read_buf[off_set + 1] | (read_buf[off_set + 2] << 8); /* x */
            input_y = read_buf[off_set + 3] | (read_buf[off_set + 4] << 8); /* y */
            input_w = read_buf[off_set + 5] | (read_buf[off_set + 6] << 8); /* size */

            gt911_touch_down(buf, read_id, input_x, input_y, input_w);
        }
    }
    else if (pre_touch)
    {
        for(read_index = 0; read_index < pre_touch; read_index++)
        {
            gt911_touch_up(buf, pre_id[read_index]);
        }
    }

    pre_touch = touch_num;

exit_:
    write_buf[0] = (rt_uint8_t)((GT911_READ_STATUS >> 8) & 0xFF);
    write_buf[1] = (rt_uint8_t)(GT911_READ_STATUS & 0xFF);
    write_buf[2] = 0x00;
    gt911_write_reg(&gt911_client, write_buf, 3);

    return read_num;
}

static rt_err_t gt911_control(struct rt_touch_device *touch, int cmd, void *arg)
{
    rt_uint8_t buf[4];
    rt_uint8_t i = 0;
    rt_uint8_t *config;

    if (cmd == RT_TOUCH_CTRL_GET_ID)
    {
        return gt911_get_product_id(&gt911_client, arg, 6);
    }

    if (cmd == RT_TOUCH_CTRL_GET_INFO)
    {
        return gt911_get_info(&gt911_client, arg);
    }

    config = (rt_uint8_t *)rt_calloc(1, sizeof(GT911_CFG_TBL) + GT911_REGITER_LEN);
    if(config == RT_NULL)
    {
        LOG_D("malloc config memory failed\n");
        return -RT_ERROR;
    }

    config[0] = (rt_uint8_t)((GT911_CONFIG_REG >> 8) & 0xFF);
    config[1] = (rt_uint8_t)(GT911_CONFIG_REG & 0xFF);

    rt_memcpy(&config[2], GT911_CFG_TBL, sizeof(GT911_CFG_TBL));

    //修改配置时注释以下内容
    if(gt911_read_regs(&gt911_client, config, &config[2], sizeof(GT911_CFG_TBL) - 2) != RT_EOK)
    {
        LOG_E("read info failed");
        return -RT_ERROR;
    }
    config[7] = 0x01;

    switch(cmd)
    {
        case RT_TOUCH_CTRL_SET_X_RANGE:
        {
            rt_uint16_t x_range;

            x_range = *(rt_uint16_t *)arg;
            config[4] = (rt_uint8_t)(x_range >> 8);
            config[3] = (rt_uint8_t)(x_range & 0xff);

            GT911_CFG_TBL[2] = config[4];
            GT911_CFG_TBL[1] = config[3];
            break;
        }
        case RT_TOUCH_CTRL_SET_Y_RANGE:
        {
            rt_uint16_t y_range;

            y_range = *(rt_uint16_t *)arg;
            config[6] = (rt_uint8_t)(y_range >> 8);
            config[5] = (rt_uint8_t)(y_range & 0xff);

            GT911_CFG_TBL[4] = config[6];
            GT911_CFG_TBL[3] = config[5];
            break;
        }
        case RT_TOUCH_CTRL_SET_X_TO_Y:
        {
            config[8] ^= (1 << 3);
            break;
        }
        case RT_TOUCH_CTRL_SET_MODE:
        {
            rt_uint16_t trig_type;
            trig_type = *(rt_uint16_t *)arg;

            switch (trig_type)
            {
                case RT_DEVICE_FLAG_INT_RX:
#if GT911_USE_LOW_ADDRESS
                    config[8] &= 0xFD;
#else
                    config[8] &= 0xFC;
#endif
                    break;
                case RT_DEVICE_FLAG_RDONLY:
                    config[8] &= 0xFC;
                    config[8] |= 0x02;
                    break;
                default:
                    break;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if(gt911_write_reg(&gt911_client, config, sizeof(GT911_CFG_TBL) + GT911_ADDR_LEN) != RT_EOK)
    {
        LOG_D("send config failed");
        return -1;
    }

    buf[0] = (rt_uint8_t)((GT911_CHECK_SUM >> 8) & 0xFF);
    buf[1] = (rt_uint8_t)(GT911_CHECK_SUM & 0xFF);
    buf[2] = 0;

    for(i = GT911_ADDR_LEN; i < sizeof(GT911_CFG_TBL) + GT911_ADDR_LEN; i++)
    {
        buf[GT911_ADDR_LEN] += config[i];
    }

    buf[2] = (~buf[2]) + 1;
    buf[3] = 1;

    gt911_write_reg(&gt911_client, buf, 4);
    rt_free(config);

    return RT_EOK;
}

static struct rt_touch_ops gt911_touch_ops =
{
    .touch_readpoint = gt911_read_point,
    .touch_control = gt911_control,
};

int rt_hw_gt911_init(void)
{
    struct rt_touch_device *gt911_device = RT_NULL;

    gt911_device = (struct rt_touch_device *)rt_malloc(sizeof(struct rt_touch_device));
    if(gt911_device == RT_NULL)
    {
        LOG_E("touch device gt911 malloc fail");
        return -RT_ERROR;
    }
    rt_memset((void *)gt911_device, 0, sizeof(struct rt_touch_device));

    gt911_device->info.type = RT_TOUCH_TYPE_CAPACITANCE;
    gt911_device->info.vendor = RT_TOUCH_VENDOR_GT;

    gt911_device->config.dev_name = GT9XX_I2C_BUS_NAME;
    gt911_device->ops = &gt911_touch_ops;

    gt911_device->config.irq_pin.pin = TP_INT_PIN;
    gt911_device->config.irq_pin.mode = PIN_MODE_INPUT_PULLUP;

    /* hw init*/
    rt_pin_mode(TP_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(gt911_device->config.irq_pin.pin, PIN_MODE_OUTPUT);

#ifdef GT911_USE_LOW_ADDRESS
    rt_pin_write(TP_RST_PIN, PIN_LOW);
    rt_pin_write(gt911_device->config.irq_pin.pin, PIN_HIGH);
    rt_thread_mdelay(30);
    rt_pin_write(TP_RST_PIN, PIN_HIGH);
    rt_thread_mdelay(30);
    rt_pin_write(gt911_device->config.irq_pin.pin, PIN_LOW);
    rt_thread_mdelay(30);
    rt_pin_write(gt911_device->config.irq_pin.pin, PIN_HIGH);
    gt911_client.client_addr = GT911_ADDRESS_LOW;
#else
    rt_pin_write(gt911_device->config.irq_pin.pin, PIN_LOW);
    rt_pin_write(TP_RST_PIN, PIN_LOW);
    rt_thread_mdelay(30);
    rt_pin_write(TP_RST_PIN, PIN_HIGH);
    rt_thread_mdelay(30);
    rt_pin_write(gt911_device->config.irq_pin.pin, PIN_LOW);
    rt_thread_mdelay(30);
    rt_pin_write(gt911_device->config.irq_pin.pin, PIN_HIGH);
    gt911_client.client_addr = GT911_ADDRESS_HIGH;
#endif

    gt911_client.bus = (struct rt_i2c_bus_device *)rt_device_find(gt911_device->config.dev_name);

    if(gt911_client.bus == RT_NULL)
    {
        LOG_E("Can't find %s device", gt911_device->config.dev_name);
        rt_free(gt911_device);
        return -RT_ERROR;
    }

    if(rt_device_open((rt_device_t)gt911_client.bus, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open %s device failed", gt911_device->config.dev_name);
        rt_free(gt911_device);
        return -RT_ERROR;
    }

    //gt911_soft_reset(&gt911_client);

    /* register touch device */
    rt_hw_touch_register(gt911_device, "gt911", RT_DEVICE_FLAG_INT_RX, RT_NULL);

    LOG_I("touch device gt911 init success");

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_gt911_init);

static rt_thread_t  gt911_thread = RT_NULL;
static rt_sem_t     gt911_sem = RT_NULL;
static rt_device_t  gt911_dev = RT_NULL;
static struct rt_touch_data *gt911_touch_data = RT_NULL;
static struct rt_touch_info info;

static void gt911_entry(void *parameter)
{
    while (1)
    {
        rt_sem_take(gt911_sem, RT_WAITING_FOREVER);

        if (rt_device_read(gt911_dev, 0, gt911_touch_data, info.point_num) == info.point_num)
        {
            rt_kprintf("x: %d, y: %d\r\n", gt911_touch_data->x_coordinate, gt911_touch_data->y_coordinate);
//            for (rt_uint8_t i = 0; i < info.point_num; i++)
//            {
//                if (gt911_touch_data[i].event == RT_TOUCH_EVENT_DOWN || gt911_touch_data[i].event == RT_TOUCH_EVENT_MOVE)
//                {
//                    LOG_D("%d %d %d %d %d", gt911_touch_data[i].track_id,
//                               gt911_touch_data[i].x_coordinate,
//                               gt911_touch_data[i].y_coordinate,
//                               gt911_touch_data[i].timestamp,
//                               gt911_touch_data[i].width);
//                }
//            }
        }
        rt_device_control(gt911_dev, RT_TOUCH_CTRL_ENABLE_INT, RT_NULL);
    }
}

static rt_err_t rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(gt911_sem);
    rt_device_control(gt911_dev, RT_TOUCH_CTRL_DISABLE_INT, RT_NULL);
    return 0;
}

void rt_touch_gt911_init(void)
{
    rt_uint8_t read_id[8] = { 0 };

    gt911_dev = rt_device_find("gt911");
    if(gt911_dev == RT_NULL)
    {
        LOG_E("Could not find touch device GT9XX");
        return;
    }

    if (rt_device_open(gt911_dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Could not open touch device GT9XX");
        return;
    }

    rt_device_control(gt911_dev, RT_TOUCH_CTRL_GET_ID, &read_id);
    LOG_D("id = GT%d%d%d", read_id[0] - '0', read_id[1] - '0', read_id[2] - '0');

    rt_uint32_t x = 480, y = 272;
    rt_device_control(gt911_dev, RT_TOUCH_CTRL_SET_X_RANGE, &x);  /* if possible you can set your x y coordinate*/
    rt_device_control(gt911_dev, RT_TOUCH_CTRL_SET_Y_RANGE, &y);
    rt_device_control(gt911_dev, RT_TOUCH_CTRL_GET_INFO, &info);
    LOG_D("range_x = %d, range_y = %d, point_num = %d", info.range_x, info.range_y, info.point_num);

    gt911_touch_data = (struct rt_touch_data *)rt_malloc(sizeof(struct rt_touch_data) * info.point_num);
    if(gt911_touch_data == RT_NULL)
    {
        LOG_E("create touch gt911 touch data failed");
        return;
    }
    rt_memset(gt911_touch_data, 0, sizeof(struct rt_touch_data) * info.point_num);

    gt911_sem = rt_sem_create("gt911_sem", 0, RT_IPC_FLAG_PRIO);
    if (gt911_sem == RT_NULL)
    {
        LOG_E("create touch gt911 dynamic semaphore failed");
        return;
    }
    rt_device_set_rx_indicate(gt911_dev, rx_callback);

    gt911_thread = rt_thread_create("gt911", gt911_entry, RT_NULL, 1024, 18, 5);
    if (gt911_thread != RT_NULL) {
        rt_thread_startup(gt911_thread);
    } else {
        rt_free(gt911_touch_data);
        gt911_touch_data = RT_NULL;
    }
}

void get_gt911_touch_data(struct rt_touch_data *touch_data)
{
    if(gt911_touch_data) {
        rt_memcpy(touch_data, gt911_touch_data, sizeof(struct rt_touch_data));
    }
}

#endif

