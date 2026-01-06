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
#include <sys/time.h>

#include "drv_rtc.h"

#define DBG_TAG "rtc"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if defined(RT_USING_RTC) && defined(BSP_USING_RTC)

#include "hal_rtc.h"

#ifdef BSP_USING_RTC_DS1307

#define DS1307_I2C_ADDR     0x68
#define DS1307_REG_SECOND   0x00
#define DS1307_REG_MINUTE   0x01
#define DS1307_REG_HOUR     0x02
#define DS1307_REG_DOW      0x03
#define DS1307_REG_DATE     0x04
#define DS1307_REG_MONTH    0x05
#define DS1307_REG_YEAR     0x06
#define DS1307_REG_CONTROL  0x07

#define RTC_DEFAULT_TIME (1733712488)    // time_t time 2024/12/9 10:47

static struct rt_i2c_client rtc_i2c_dev;
static rt_timer_t rtc_soft_timer;
static time_t time_local = RTC_DEFAULT_TIME;
static struct tm tm_local;

static uint8_t DS1307_DecodeBCD(uint8_t bin)
{
    return (((bin & 0xf0) >> 4) * 10) + (bin & 0x0f);
}

static uint8_t DS1307_EncodeBCD(uint8_t dec)
{
    return (dec % 10 + ((dec / 10) << 4));
}

static rt_err_t DS1307_WriteRegs(uint8_t regAddr, uint8_t *data, uint32_t size)
{
    struct rt_i2c_msg msg;
    static uint8_t buff[32] = { 0 };

    buff[0] = regAddr;
    rt_memcpy(&buff[1], data, size);

    msg.addr  = rtc_i2c_dev.client_addr;
    msg.flags = RT_I2C_WR;
    msg.buf   = buff;
    msg.len   = size + 1;

    if (rt_i2c_transfer(rtc_i2c_dev.bus, &msg, 1) != 1)
    //if(rtc_i2c_dev.bus->ops->master_xfer(rtc_i2c_dev.bus, &msg, 1) != 1)
        return -RT_ERROR;

    return RT_EOK;
}

static rt_err_t DS1307_ReadRegs(uint8_t regAddr, uint8_t *data, uint32_t size)
{
    struct rt_i2c_msg msgs[2];
    msgs[0].addr  = rtc_i2c_dev.client_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &regAddr;
    msgs[0].len   = 1;

    msgs[1].addr  = rtc_i2c_dev.client_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = size;

    if (rt_i2c_transfer(rtc_i2c_dev.bus, msgs, 2) != 2)
    //if(rtc_i2c_dev.bus->ops->master_xfer(rtc_i2c_dev.bus, msgs, 2) != 2)
        return -RT_ERROR;

    return RT_EOK;
}

static rt_err_t DS1307_default_init()
{
    uint8_t data[7] = { 0 };
    time_local = RTC_DEFAULT_TIME;
    localtime_r(&time_local, &tm_local);

    data[0] = DS1307_EncodeBCD(tm_local.tm_sec);
    data[1] = DS1307_EncodeBCD(tm_local.tm_min);
    data[2] = DS1307_EncodeBCD(tm_local.tm_hour & 0x3F);
    data[3] = DS1307_EncodeBCD(tm_local.tm_wday + 1);
    data[4] = DS1307_EncodeBCD(tm_local.tm_mday);
    data[5] = DS1307_EncodeBCD(tm_local.tm_mon + 1);
    data[6] = DS1307_EncodeBCD((tm_local.tm_year - 100) % 100);

    return DS1307_WriteRegs(DS1307_REG_SECOND, data, 7);
}

static rt_err_t DS1307_StartWork()
{
    uint8_t data;
    rt_err_t err = DS1307_ReadRegs(DS1307_REG_SECOND, &data, 1);
    if(err != RT_EOK) return err;

    data &= 0x7F;
    return DS1307_WriteRegs(DS1307_REG_SECOND, &data, 1);
}

static rt_err_t DS1307_GetClockHalt(rt_bool_t *halt)
{
    uint8_t clockHalt;
    rt_err_t err = DS1307_ReadRegs(DS1307_REG_SECOND, &clockHalt, 1);
    *halt = (clockHalt & 0x80) >> 7;
    return err;
}

static void DS1307_get_time_cb(void *parameter)
{
    uint8_t data[7] = { 0 };
    rt_err_t err = DS1307_ReadRegs(DS1307_REG_SECOND, data, 7);
    if(err != RT_EOK) return;

    int tm_sec  = DS1307_DecodeBCD(data[0] & 0x7F);
    int tm_min  = DS1307_DecodeBCD(data[1]);
    int tm_hour = DS1307_DecodeBCD(data[2] & 0x3F);
    int tm_mday = DS1307_DecodeBCD(data[4]);
    int tm_mon  = DS1307_DecodeBCD(data[5]);
    int tm_year = DS1307_DecodeBCD(data[6]);

    if(tm_sec >= 0 && tm_sec < 60) tm_local.tm_sec = tm_sec;
    if(tm_min >= 0 && tm_min < 60) tm_local.tm_min = tm_min;
    if(tm_hour >= 0 && tm_hour < 24) tm_local.tm_hour = tm_hour;
    if(tm_mday >= 1 && tm_mday < 32) tm_local.tm_mday = tm_mday;
    if(tm_mon >= 1 && tm_mon < 13) tm_local.tm_mon = tm_mon - 1;
    if(tm_min >= 0 && tm_min < 100) tm_local.tm_year = tm_year + 100;

    time_local = mktime(&tm_local);
}

static rt_err_t rtc_hw_init(const char *dev_name)
{
    rt_err_t err = RT_EOK;

    rtc_i2c_dev.bus = (struct rt_i2c_bus_device *)rt_device_find(dev_name);
    rtc_i2c_dev.client_addr = DS1307_I2C_ADDR;

    if(rtc_i2c_dev.bus == RT_NULL)
    {
        LOG_E("Can't find %s device", dev_name);
        return -RT_ERROR;
    }

    if(rt_device_open((rt_device_t)rtc_i2c_dev.bus, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open %s device failed", dev_name);
        return -RT_ERROR;
    }

    int clk = 100 * 1000;
    rt_device_control((rt_device_t)rtc_i2c_dev.bus, RT_I2C_DEV_CTRL_CLK, &clk);

    rt_bool_t clockHalt;
    err = DS1307_GetClockHalt(&clockHalt);
    if(err != RT_EOK) return err;

    if(clockHalt)
    {
        err = DS1307_default_init();
        if(err != RT_EOK) return err;
        err = DS1307_StartWork();
        if(err != RT_EOK) return err;
    }

    return err;
}

static rt_err_t rtc_init(struct rt_device *dev)
{
    return RT_EOK;
}

static rt_err_t rtc_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    uint8_t data[7] = { 0 };
    time_t *time_stamp = (time_t *)args;

    switch (cmd)
    {
        case RT_DEVICE_CTRL_RTC_GET_TIME:
            *time_stamp = time_local;
            break;
        case RT_DEVICE_CTRL_RTC_SET_TIME:
            time_local = *time_stamp;
            localtime_r(&time_local, &tm_local);

            data[0] = DS1307_EncodeBCD(tm_local.tm_sec);
            data[1] = DS1307_EncodeBCD(tm_local.tm_min);
            data[2] = DS1307_EncodeBCD(tm_local.tm_hour & 0x3F);
            data[3] = DS1307_EncodeBCD(tm_local.tm_wday + 1);
            data[4] = DS1307_EncodeBCD(tm_local.tm_mday);
            data[5] = DS1307_EncodeBCD(tm_local.tm_mon + 1);
            data[6] = DS1307_EncodeBCD((tm_local.tm_year - 100) % 100);

            if (DS1307_WriteRegs(DS1307_REG_SECOND, data, 7) != 0)
            {
                LOG_E("rtc settime failed!\n");
                return -RT_ERROR;
            }
            break;
        default:
            return -RT_EINVAL;
    }

    return result;
}

#elif defined (BSP_USING_RTC_SUNXI)

static rt_err_t rtc_init(struct rt_device *dev)
{
    if (hal_rtc_init() != 0)
    {
        LOG_E("init rtc hal failed!");
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t rtc_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    struct tm time_temp;
    struct tm *time_now;
    struct rtc_time hal_rtc_time;
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:

        if (hal_rtc_gettime(&hal_rtc_time) != 0)
        {
            LOG_E("rtc gettime failed!\n");
            return -RT_ERROR;
        }

        time_temp.tm_sec = hal_rtc_time.tm_sec;
        time_temp.tm_min = hal_rtc_time.tm_min;
        time_temp.tm_hour = hal_rtc_time.tm_hour;
        time_temp.tm_mday = hal_rtc_time.tm_mday;
        time_temp.tm_mon = hal_rtc_time.tm_mon;
        time_temp.tm_year = hal_rtc_time.tm_year;

        *((time_t *)args) = mktime(&time_temp);
        break;
    case RT_DEVICE_CTRL_RTC_SET_TIME:

        rt_enter_critical();
        /* converts calendar time time into local time. */
        time_now = localtime((const time_t *)args);
        /* copy the statically located variable */
        rt_memcpy(&time_temp, time_now, sizeof(struct tm));
        /* unlock scheduler. */
        rt_exit_critical();

        hal_rtc_time.tm_sec = time_temp.tm_sec;
        hal_rtc_time.tm_min = time_temp.tm_min;
        hal_rtc_time.tm_hour = time_temp.tm_hour;
        hal_rtc_time.tm_mday = time_temp.tm_mday;
        hal_rtc_time.tm_mon = time_temp.tm_mon;
        hal_rtc_time.tm_year = time_temp.tm_year;
        if (hal_rtc_settime(&hal_rtc_time) != 0)
        {
            LOG_E("rtc settime failed!\n");
            return -RT_ERROR;
        }
        break;
    default:
        return -RT_EINVAL;
    }

    return result;
}

#endif

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rt_hw_rtc_ops =
{
    rtc_init,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rtc_control
};
#endif

int rt_hw_rtc_init(void)
{
    rt_err_t ret = RT_EOK;
    static struct rt_device rtc_dev;

    rtc_dev.type = RT_Device_Class_RTC;
    rtc_dev.rx_indicate = RT_NULL;
    rtc_dev.tx_complete = RT_NULL;

#ifdef BSP_USING_RTC_DS1307

    ret = rtc_hw_init("i2c1");
    if(ret != RT_EOK)
    {
        LOG_E("rtc init failed!");
        return ret;
    }

    rtc_soft_timer = rt_timer_create("DS1307", DS1307_get_time_cb, NULL, 100, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if(rtc_soft_timer == NULL) return -RT_ENOMEM;
    rt_timer_start(rtc_soft_timer);

#endif

#ifdef RT_USING_DEVICE_OPS
    rtc_dev.ops = &rt_hw_rtc_ops;
#else
    rtc_dev.init = rtc_init;
    rtc_dev.open = RT_NULL;
    rtc_dev.close = RT_NULL;
    rtc_dev.read = RT_NULL;
    rtc_dev.write = RT_NULL;
    rtc_dev.control = rtc_control;
#endif

    rtc_dev.user_data = RT_NULL;

    ret = rt_device_register(&rtc_dev, "rtc", RT_DEVICE_FLAG_RDWR);

    return ret;
}
INIT_DEVICE_EXPORT(rt_hw_rtc_init);

#endif
