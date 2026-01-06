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

#include "drv_gpio.h"
#include "drv_i2c.h"

#define DBG_TAG  "IIC"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if defined (RT_USING_I2C) && defined (BSP_USING_I2C)

#include "hal_twi.h"

struct sunxi_i2c
{
    struct rt_i2c_bus_device parent;

    twi_port_t port;
    struct rt_mutex lock;

    struct gpio_cfg gpio_sda;
    struct gpio_cfg gpio_scl;
};

static void sunxi_i2c_init(struct sunxi_i2c *i2c)
{
    rt_mutex_init(&i2c->lock, "i2c lock", RT_IPC_FLAG_PRIO);

    gpio_set_config(i2c->gpio_scl);
    gpio_set_config(i2c->gpio_sda);

    hal_twi_init(i2c->port, TWI_DRV_XFER);
}

rt_ssize_t sunxi_i2c_master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    int i = 0;
    twi_status_t status;
    twi_msg_t *twi_msg = RT_NULL;
    struct sunxi_i2c *i2c = (struct sunxi_i2c *)bus;

    twi_msg = rt_malloc(sizeof(*twi_msg) * num);
    if (!twi_msg)
    {
        LOG_E("i2c xfer malloc(%d) failure\n", sizeof(*twi_msg) * num);
        return -RT_ENOMEM;
    }

    for(i = 0; i < num; i++)
    {
        if (msgs[i].flags == RT_I2C_RD)
        {
            twi_msg[i].flags = TWI_M_RD;
        }
        else if(msgs[i].flags == RT_I2C_WR)
        {
            twi_msg[i].flags = 0;
        }

        if (i2c->parent.flags & RT_I2C_DEV_CTRL_10BIT)
        {
            twi_msg[i].flags |= TWI_M_TEN;
        }

        twi_msg[i].addr = msgs[i].addr;
        twi_msg[i].len = msgs[i].len;
        twi_msg[i].buf = msgs[i].buf;
    }

    rt_mutex_take(&i2c->lock, RT_WAITING_FOREVER);
    status = hal_twi_xfer(i2c->port, twi_msg, i);
    rt_mutex_release(&i2c->lock);

    if (status != TWI_STATUS_OK)
    {
        i = 0;
        LOG_E("i2c xfer failure\n");
    }

    rt_free(twi_msg);

    return i;
}

rt_err_t sunxi_i2c_bus_control(struct rt_i2c_bus_device *bus, int cmd, void *arg)
{
    struct sunxi_i2c *i2c = (struct sunxi_i2c *)bus;
    hal_twi_set_rate(i2c->port, *(int *)arg);

    return RT_EOK;
}

static const struct rt_i2c_bus_device_ops _i2c_ops =
{
    sunxi_i2c_master_xfer,
    RT_NULL,
    sunxi_i2c_bus_control
};

int rt_hw_i2c_init(void)
{
    struct sunxi_i2c *i2c;
#ifdef BSP_USING_I2C0
    {
        static struct sunxi_i2c i2c0;

        i2c = &i2c0;
        i2c->parent.ops = &_i2c_ops;
        i2c->port = 0;
//        i2c->gpio_sda = (struct gpio_cfg){ GPIOE(1), GPIO_MUXSEL_FUNCTION4, GPIO_PULL_UP, GPIO_DRIVING_LEVEL1 };
//        i2c->gpio_scl = (struct gpio_cfg){ GPIOE(0), GPIO_MUXSEL_FUNCTION4, GPIO_PULL_UP, GPIO_DRIVING_LEVEL1 };

        sunxi_i2c_init(i2c);
        rt_i2c_bus_device_register(&i2c->parent, "i2c0");
    }
#endif

#ifdef BSP_USING_I2C1
    {
        static struct sunxi_i2c i2c1;

        i2c = &i2c1;
        i2c->parent.ops = &_i2c_ops;
        i2c->port = 1;
        i2c->gpio_sda = (struct gpio_cfg){ GPIOE(1), GPIO_MUXSEL_FUNCTION4, GPIO_PULL_UP, GPIO_DRIVING_LEVEL1 };
        i2c->gpio_scl = (struct gpio_cfg){ GPIOE(0), GPIO_MUXSEL_FUNCTION4, GPIO_PULL_UP, GPIO_DRIVING_LEVEL1 };

        sunxi_i2c_init(i2c);
        rt_i2c_bus_device_register(&i2c->parent, "i2c1");
    }
#endif

#ifdef BSP_USING_I2C2
    {
        static struct sunxi_i2c i2c2;

        i2c = &i2c2;
        i2c->parent.ops = &_i2c_ops;
        i2c->port = 2;
        i2c->gpio_sda = (struct gpio_cfg){ GPIOE(13), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL1 };
        i2c->gpio_scl = (struct gpio_cfg){ GPIOE(12), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL1 };

        sunxi_i2c_init(i2c);
        rt_i2c_bus_device_register(&i2c->parent, "i2c2");
    }
#endif

#ifdef BSP_USING_I2C3
    {
        static struct sunxi_i2c i2c3;

        i2c = &i2c3;
        i2c->parent.ops = &_i2c_ops;
        i2c->port = 3;
//        i2c->gpio_sda = (struct gpio_cfg){ GPIOE(1), GPIO_MUXSEL_FUNCTION4, GPIO_PULL_UP, GPIO_DRIVING_LEVEL1 };
//        i2c->gpio_scl = (struct gpio_cfg){ GPIOE(0), GPIO_MUXSEL_FUNCTION4, GPIO_PULL_UP, GPIO_DRIVING_LEVEL1 };

        sunxi_i2c_init(i2c);
        rt_i2c_bus_device_register(&i2c->parent, "i2c3");
    }
#endif

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#endif





