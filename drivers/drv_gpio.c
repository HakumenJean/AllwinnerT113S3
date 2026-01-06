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
#include <rthw.h>

#include "drv_gpio.h"
#include "interrupt.h"

#define DBG_TAG  "GPIO"
#define DBG_LVL  DBG_INFO
#include <rtdbg.h>

void gpio_set_config(struct gpio_cfg gpio)
{
    hal_gpio_pinmux_set_function(gpio.pin, gpio.fun);
    hal_gpio_set_pull(gpio.pin, gpio.pull);
    hal_gpio_set_driving_level(gpio.pin, gpio.drv);
}

#ifdef RT_USING_PIN

#include <rtdevice.h>

static void pin_mode(struct rt_device *dev, rt_base_t pin, rt_uint8_t mode)
{
    gpio_muxsel_t sunxi_io_mode = GPIO_MUXSEL_DISABLED;
    gpio_pull_status_t sunxi_io_pull = GPIO_PULL_DOWN_DISABLED;

    if (pin > GPIO_MAX)
    {
        LOG_D("pin:%d value wrongful", pin);
        return;
    }

    hal_gpio_set_driving_level(pin, GPIO_DRIVING_LEVEL3);

    if(mode >> 4)
    {
        hal_gpio_set_pull(pin, GPIO_PULL_DOWN_DISABLED);
        hal_gpio_pinmux_set_function(pin, (mode >> 4) & 0xF);
        return;
    }

    switch(mode)
    {
        case PIN_MODE_OUTPUT:
        case PIN_MODE_OUTPUT_OD:
            sunxi_io_mode = GPIO_MUXSEL_OUT;
            break;
        case PIN_MODE_INPUT:
            sunxi_io_mode = GPIO_MUXSEL_IN;
            break;
        case PIN_MODE_INPUT_PULLUP:
            sunxi_io_pull = GPIO_PULL_UP;
            sunxi_io_mode = GPIO_MUXSEL_IN;
            break;
        case PIN_MODE_INPUT_PULLDOWN:
            sunxi_io_pull = GPIO_PULL_DOWN;
            sunxi_io_mode = GPIO_MUXSEL_IN;
            break;
        default: break;
    }

    hal_gpio_pinmux_set_function(pin, sunxi_io_mode);
    hal_gpio_set_pull(pin, sunxi_io_pull);
}

static void pin_write(struct rt_device *dev, rt_base_t pin, rt_uint8_t value)
{
    if (pin > GPIO_MAX)
    {
        LOG_D("pin:%d value wrongful", pin);
        return;
    }

    hal_gpio_set_data(pin, value);
}

static int pin_read(struct rt_device *device, rt_base_t pin)
{
    gpio_data_t data;

    if (pin > GPIO_MAX)
    {
        LOG_D("pin:%d value wrongful", pin);
        return 0;
    }

    hal_gpio_get_data(pin, &data);
    return data;
}

static rt_err_t pin_attach_irq(struct rt_device *device, rt_int32_t pin, rt_uint8_t mode, void (*hdr)(void *args), void *args)
{
    uint32_t irq;
    gpio_interrupt_mode_t irq_type = IRQ_TYPE_NONE;

    if (pin > GPIO_MAX)
    {
        LOG_D("pin:%d value wrongful", pin);
        return RT_ERROR;
    }

    switch(mode)
    {
        case PIN_IRQ_MODE_RISING: irq_type = IRQ_TYPE_EDGE_RISING; break;
        case PIN_IRQ_MODE_FALLING: irq_type = IRQ_TYPE_EDGE_FALLING; break;
        case PIN_IRQ_MODE_RISING_FALLING: irq_type = IRQ_TYPE_EDGE_BOTH; break;
        case PIN_IRQ_MODE_HIGH_LEVEL: irq_type = IRQ_TYPE_LEVEL_HIGH; break;
        case PIN_IRQ_MODE_LOW_LEVEL: irq_type = IRQ_TYPE_LEVEL_LOW; break;
        default: break;
    }

    hal_gpio_to_irq(pin, &irq);
    hal_gpio_irq_request(irq, hdr, irq_type, args);

    return RT_EOK;
}

static rt_err_t pin_detach_irq(struct rt_device *device, rt_int32_t pin)
{
    uint32_t irq;

    if (pin > GPIO_MAX)
    {
        LOG_D("pin:%d value wrongful", pin);
        return RT_ERROR;
    }

    hal_gpio_to_irq(pin, &irq);
    hal_gpio_irq_free(irq);

    return RT_EOK;
}

rt_err_t pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint8_t enabled)
{
    uint32_t irq;

    if (pin > GPIO_MAX)
    {
        LOG_D("pin:%d value wrongful", pin);
        return RT_ERROR;
    }

    hal_gpio_to_irq(pin, &irq);

    if (enabled)
        hal_gpio_irq_enable(irq);
    else
        hal_gpio_irq_disable(irq);

    return RT_EOK;
}

static const struct rt_pin_ops ops =
{
    pin_mode,
    pin_write,
    pin_read,
    pin_attach_irq,
    pin_detach_irq,
    pin_irq_enable,
    RT_NULL,
};
#endif

int rt_hw_gpio_init(void)
{
    hal_gpio_init();

#ifdef RT_USING_PIN
    rt_device_pin_register("gpio", &ops, RT_NULL);
#endif

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_gpio_init);
