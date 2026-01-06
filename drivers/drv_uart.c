/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "drv_gpio.h"
#include "drv_uart.h"
#include "hal_uart.h"

#ifdef RT_USING_SERIAL

struct sunxi_uart
{
    struct rt_serial_device *serial;
    rt_uint8_t port;

    struct gpio_cfg gpio_rx;
    struct gpio_cfg gpio_tx;
};

void hal_uart_callback(uart_callback_event_t event, void *user_data)
{
    struct sunxi_uart *uart = (struct sunxi_uart *)user_data;
    struct rt_serial_device *serial = uart->serial;

    // UART_EVENT_RX_DATA
    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
}

static int sunxi_uart_init(struct sunxi_uart *uart)
{
    /* config gpio port */
    gpio_set_config(uart->gpio_rx);
    gpio_set_config(uart->gpio_tx);

    hal_uart_init(uart->port, hal_uart_callback, uart);

    return 0;
}

static rt_err_t uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct sunxi_uart *uart = serial->parent.user_data;
    serial->config = *cfg;

    hal_uart_set_format(uart->port, cfg->data_bits, cfg->stop_bits, cfg->parity);
    hal_uart_set_baudrate(uart->port, cfg->baud_rate);

    if(cfg->flowcontrol) hal_uart_set_hardware_flowcontrol(uart->port);
    else hal_uart_disable_flowcontrol(uart->port);

    return RT_EOK;
}

static rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct sunxi_uart *uart;

    uart = serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    switch (cmd)
    {
        case RT_DEVICE_CTRL_CLR_INT:
            /* Disable the UART Interrupt */
            hal_uart_disable_rx(uart->port);
            break;

        case RT_DEVICE_CTRL_SET_INT:
            /* Enable the UART Interrupt */
            hal_uart_enable_rx(uart->port);
            break;
        case RT_DEVICE_CTRL_CONFIG:
            uart_configure(serial, (struct serial_configure *)arg);
            break;
        case RT_DEVICE_CTRL_CLOSE:
            hal_uart_deinit(uart->port);
            break;
    }

    return (RT_EOK);
}

static int uart_putc(struct rt_serial_device *serial, char c)
{
    struct sunxi_uart *uart = serial->parent.user_data;

    return hal_uart_putc(uart->port, c);
}

static int uart_getc(struct rt_serial_device *serial)
{
    struct sunxi_uart *uart = serial->parent.user_data;

    return hal_uart_getc(uart->port);
}

static rt_ssize_t uart_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    /*
    struct sunxi_uart *uart = serial->parent.user_data;

    if(!size) return 0;
    if(RT_SERIAL_DMA_TX == direction)
    {
        ///@todo: dma transmit, return size;
    }
    */

    return 0;
}

const struct rt_uart_ops _uart_ops =
{
    uart_configure,
    uart_control,
    uart_putc,
    uart_getc,
    uart_dma_transmit
};

/*
 * UART Initiation
 */
int rt_hw_uart_init(void)
{
    struct rt_serial_device *serial;
    struct sunxi_uart      *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef BSP_USING_UART0
    {
        static struct rt_serial_device  serial0;
        static struct sunxi_uart       uart0;

        serial  = &serial0;
        uart    = &uart0;

        serial->ops              = &_uart_ops;
        serial->config           = config;

        uart->port = 0;
        uart->gpio_rx = (struct gpio_cfg){ GPIOE(3), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
        uart->gpio_tx = (struct gpio_cfg){ GPIOE(2), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
        uart->serial = serial;
        sunxi_uart_init(uart);

        rt_hw_serial_register(serial, "uart0", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
    }
#endif

#ifdef BSP_USING_UART1
    {
        static struct rt_serial_device  serial1;
        static struct sunxi_uart       uart1;

        serial  = &serial1;
        uart    = &uart1;

        serial->ops              = &_uart_ops;
        serial->config           = config;

        uart->port = 1;
//        uart->gpio_rx = (struct gpio_cfg){ GPIOE(3), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
//        uart->gpio_tx = (struct gpio_cfg){ GPIOE(2), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
        uart->serial = serial;
        sunxi_uart_init(uart);

        rt_hw_serial_register(serial, "uart1", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
    }
#endif

#ifdef BSP_USING_UART2
    {
        static struct rt_serial_device  serial2;
        static struct sunxi_uart       uart2;

        serial  = &serial2;
        uart    = &uart2;

        serial->ops              = &_uart_ops;
        serial->config           = config;

        uart->port = 2;
//        uart->gpio_rx = (struct gpio_cfg){ GPIOE(3), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
//        uart->gpio_tx = (struct gpio_cfg){ GPIOE(2), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
        uart->serial = serial;
        sunxi_uart_init(uart);

        rt_hw_serial_register(serial, "uart2", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
    }
#endif

#ifdef BSP_USING_UART3
    {
        static struct rt_serial_device  serial3;
        static struct sunxi_uart       uart3;

        serial  = &serial3;
        uart    = &uart3;

        serial->ops              = &_uart_ops;
        serial->config           = config;

        uart->port = 3;
        uart->gpio_rx = (struct gpio_cfg){ GPIOB(7), GPIO_MUXSEL_FUNCTION7, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
        uart->gpio_tx = (struct gpio_cfg){ GPIOB(6), GPIO_MUXSEL_FUNCTION7, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
        uart->serial = serial;
        sunxi_uart_init(uart);

        rt_hw_serial_register(serial, "uart3", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
    }
#endif

#ifdef BSP_USING_UART4
    {
        static struct rt_serial_device  serial4;
        static struct sunxi_uart       uart4;

        serial  = &serial4;
        uart    = &uart4;

        serial->ops              = &_uart_ops;
        serial->config           = config;

        uart->port = 4;
//        uart->gpio_rx = (struct gpio_cfg){ GPIOE(3), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
//        uart->gpio_tx = (struct gpio_cfg){ GPIOE(2), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
        uart->serial = serial;
        sunxi_uart_init(uart);

        rt_hw_serial_register(serial, "uart4", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
    }
#endif

#ifdef BSP_USING_UART5
    {
        static struct rt_serial_device  serial5;
        static struct sunxi_uart       uart5;

        serial  = &serial5;
        uart    = &uart5;

        serial->ops              = &_uart_ops;
        serial->config           = config;

        uart->port = 5;
//        uart->gpio_rx = (struct gpio_cfg){ GPIOE(3), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
//        uart->gpio_tx = (struct gpio_cfg){ GPIOE(2), GPIO_MUXSEL_FUNCTION6, GPIO_PULL_DOWN_DISABLED, GPIO_DRIVING_LEVEL1 };
        uart->serial = serial;
        sunxi_uart_init(uart);

        rt_hw_serial_register(serial, "uart5", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
    }
#endif

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_uart_init);

#endif
