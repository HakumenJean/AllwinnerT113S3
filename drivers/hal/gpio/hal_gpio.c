/*
 * Copyright 2018 AllWinnertech  Co., Ltd
 * wangwei@allwinnertech.com
 */

#include "hal_common.h"
#include "hal_interrupt.h"
#include "hal_log.h"
#include "hal_gpio.h"
#include "gpio_reg.h"

//#define CONFIG_DRIVERS_GPIO_DEBUG
#ifdef  CONFIG_DRIVERS_GPIO_DEBUG
#define GPIO_INFO(fmt, arg...) hal_log_info(fmt, ##arg)
#else
#define GPIO_INFO(fmt, arg...) do {}while(0)
#endif

#define GPIO_ERR(fmt, arg...) hal_log_err(fmt, ##arg)

struct gpio_irq_def
{
    void *irq_arg[32];
    void (*irq_cb[32])(void *param);
};

static struct gpio_irq_def _g_gpio_irq_tbl[GPIO_PORT_NUM];

int hal_gpio_get_data(gpio_pin_t pin, gpio_data_t *value)
{
    uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_DATA_ADDR(GET_GPIO_PORT(pin));
    offset = GET_GPIO_PIN(pin);

    data = readl(addr);
    *value = (data >> offset) & 0x01;

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));

    return 0;
}

int hal_gpio_set_data(gpio_pin_t pin, gpio_data_t value)
{
    uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_DATA_ADDR(GET_GPIO_PORT(pin));
    offset = GET_GPIO_PIN(pin);

    data = readl(addr);
    data &= ~(1 << offset);
    data |= value << offset;
    writel(data, addr);

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
    return 0;
}

int hal_gpio_pinmux_set_function(gpio_pin_t pin, gpio_muxsel_t function_index)
{
    uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_CFG_ADDR(GET_GPIO_PORT(pin)) + ((GET_GPIO_PIN(pin) >> 3) << 2);
    offset = (GET_GPIO_PIN(pin) & 0x7) << 2;

    data = readl(addr);
    data &= ~(0xF << offset);
    data |= (function_index & 0xF) << offset;
    writel(data, addr);

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
    return 0;
}

int hal_gpio_pinmux_get_function(gpio_pin_t pin, gpio_muxsel_t *function_index)
{
    uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_CFG_ADDR(GET_GPIO_PORT(pin)) + ((GET_GPIO_PIN(pin) >> 3) << 2);
    offset = (GET_GPIO_PIN(pin) & 0x7) << 2;

    data = readl(addr);
    *function_index = (data >> offset) & 0xF;

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
    return 0;
}

int hal_gpio_set_direction(gpio_pin_t pin, gpio_direction_t direction)
{
    return hal_gpio_pinmux_set_function(pin, direction);
}

int hal_gpio_get_direction(gpio_pin_t pin, gpio_direction_t *direction)
{
    gpio_muxsel_t function_index;
    hal_gpio_pinmux_get_function(pin, &function_index);
    *direction = function_index & 0x1;

    return 0;
}

int hal_gpio_set_pull(gpio_pin_t pin, gpio_pull_status_t pull)
{
    uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_PUL_ADDR(GET_GPIO_PORT(pin));
    addr += GET_GPIO_PIN(pin) > 15 ? 0x4 : 0x0;
    offset = (GET_GPIO_PIN(pin) & 0xf) << 1;

    data = readl(addr);
    data &= ~(0x3 << offset);
    data |= pull << offset;
    writel(data, addr);

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
    return 0;
}

int hal_gpio_get_pull(gpio_pin_t pin, gpio_pull_status_t *pull)
{
    uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_PUL_ADDR(GET_GPIO_PORT(pin));
    addr += GET_GPIO_PIN(pin) > 15 ? 0x4 : 0x0;
    offset = (GET_GPIO_PIN(pin) & 0xf) << 1;

    data = readl(addr);
    *pull = (data >> offset) & 0x3;

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
    return 0;
}

int hal_gpio_set_driving_level(gpio_pin_t pin, gpio_driving_level_t level)
{
    uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_DRV_ADDR(GET_GPIO_PORT(pin));
    addr += GET_GPIO_PIN(pin) > 15 ? 0x4 : 0x0;
    offset = (GET_GPIO_PIN(pin) & 0xf) << 1;

    data = readl(addr);
    data &= ~(0x3 << offset);
    data |= level << offset;
    writel(data, addr);

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
    return 0;
}

int hal_gpio_get_driving_level(gpio_pin_t pin, gpio_driving_level_t *level)
{
    uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_DRV_ADDR(GET_GPIO_PORT(pin));
    addr += GET_GPIO_PIN(pin) > 15 ? 0x4 : 0x0;
    offset = (GET_GPIO_PIN(pin) & 0xf) << 1;

    data = readl(addr);
    *level = (data >> offset) & 0x3;

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
    return 0;
}

static void hal_gpio_power_switch_pf(u32 tar_vol, u32 pow_sel)
{
    uint32_t cur_vol;

    cur_vol = readl(GPIO_BASE_ADDR + GPIO_PIO_POW_VOL_SEL_CTL_REG) & BIT(0);
    tar_vol &= BIT(0);

    if (cur_vol < tar_vol) {
        writel(pow_sel, GPIO_BASE_ADDR + GPIO_PIO_POW_MOD_SEL_REG);
        writel(tar_vol, GPIO_BASE_ADDR + GPIO_PIO_POW_VOL_SEL_CTL_REG);
    } else if (cur_vol > tar_vol) {
        writel(tar_vol, GPIO_BASE_ADDR + GPIO_PIO_POW_VOL_SEL_CTL_REG);
        writel(pow_sel, GPIO_BASE_ADDR + GPIO_PIO_POW_MOD_SEL_REG);
    } else {
        writel(pow_sel, GPIO_BASE_ADDR + GPIO_PIO_POW_MOD_SEL_REG);
    }
}

int hal_gpio_sel_vol_mode(gpio_pin_t pin, gpio_power_mode_t pm_sel)
{
    uint32_t data, bank;

    RT_ASSERT(pin <= GPIO_MAX);

    bank = GET_GPIO_PORT(pin);
    data = readl(GPIO_BASE_ADDR + GPIO_PIO_POW_MOD_SEL_REG);
    data &= ~(1 << bank);
    data |= (pm_sel << bank);

    if(bank == 5) {
        hal_gpio_power_switch_pf(~pm_sel, data);
    } else {
        writel(data, GPIO_BASE_ADDR + GPIO_PIO_POW_MOD_SEL_REG);
    }

    return 0;
}

int hal_gpio_set_debounce(gpio_pin_t pin, unsigned value)
{
    uint32_t addr, data;
    unsigned int val_clk_select, val_clk_per_scale;

    RT_ASSERT(pin <= GPIO_MAX);

    val_clk_select = value & 1;
    val_clk_per_scale = (value >> 4) & 0x07;

    addr = GPIOn_INT_DEB_ADDR(GET_GPIO_PORT(pin));
    data = readl(addr);
    data &= ~((0x07 << 4) | (0x01 << 0));
    data |= (val_clk_per_scale << 4) | val_clk_select;
    writel(data, addr);

    GPIO_INFO("[line]:%d addr:%08x data:%08x", __LINE__, addr, *((rt_uint32_t *)addr));
    return 0;
}

int hal_gpio_to_irq(gpio_pin_t pin, uint32_t *irq)
{
    RT_ASSERT(pin <= GPIO_MAX);
    *irq = pin;

    return 0;
}

static void hal_gpio_set_irq_callback(gpio_pin_t pin, void (*hdr)(void *args), void *data)
{
    hal_enter_critical();
    _g_gpio_irq_tbl[GET_GPIO_PORT(pin)].irq_cb[GET_GPIO_PIN(pin)]    = hdr;
    _g_gpio_irq_tbl[GET_GPIO_PORT(pin)].irq_arg[GET_GPIO_PIN(pin)]   = data;
    hal_exit_critical();
}

static void hal_gpio_set_irq_type(gpio_pin_t pin, unsigned long irq_type)
{
    rt_uint32_t addr, offset, data;

    addr = GPIOn_INT_CFG_ADDR(GET_GPIO_PORT(pin)) + ((GET_GPIO_PIN(pin) >> 3) << 2);
    offset = (GET_GPIO_PIN(pin) & 0x7) << 2;

    data = readl(addr);
    data &= ~(0xF << offset);
    data |= irq_type << offset;
    writel(data, addr);

    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
}

static void hal_gpio_irq_clear(uint32_t port, uint32_t pin)
{
    rt_uint32_t addr, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_INT_STA_ADDR(port);
    data = readl(addr);
    data |= 0x1 << pin;
    writel(data, addr);
}

int hal_gpio_irq_enable(uint32_t irq)
{
    gpio_pin_t pin = irq;
    rt_uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    addr = GPIOn_INT_CTRL_ADDR(GET_GPIO_PORT(pin));
    offset = GET_GPIO_PIN(pin);

    data = readl(addr);
    data |= 0x1 << offset;
    writel(data, addr);
    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));

    return 0;
}

int hal_gpio_irq_disable(uint32_t irq)
{
    gpio_pin_t pin = irq;
    rt_uint32_t addr, offset, data;

    RT_ASSERT(pin <= GPIO_MAX);

    hal_gpio_irq_clear(GET_GPIO_PORT(pin), GET_GPIO_PIN(pin));
    addr = GPIOn_INT_CTRL_ADDR(GET_GPIO_PORT(pin));
    offset = GET_GPIO_PIN(pin);

    data = readl(addr);
    data &= ~(0x1 << offset);

    writel(data, addr);
    GPIO_INFO("[line]:%d offset:%d addr:%08x data:%08x", __LINE__, offset, addr, *((rt_uint32_t *)addr));
    return 0;
}

int hal_gpio_irq_request(uint32_t irq, void (*hdr)(void *args), unsigned long type, void *data)
{
    gpio_pin_t pin = irq;

    hal_gpio_pinmux_set_function(pin, GPIO_MUXSEL_EINT);
    hal_gpio_set_irq_callback(pin, hdr, data);
    hal_gpio_set_irq_type(pin, type);

    return irq;
}

int hal_gpio_irq_free(uint32_t irq)
{
    gpio_pin_t pin = irq;

    RT_ASSERT(pin <= GPIO_MAX);
    hal_gpio_irq_disable(irq);
    hal_gpio_set_irq_callback(pin, NULL, NULL);

    return irq;
}

static void hal_gpio_irq_handler(int vector, void *param)
{
    struct gpio_irq_def *irq_def = (struct gpio_irq_def *)param;
    rt_uint32_t pend, enable;
    int port, pin;
    rt_uint32_t addr;

    port = ((vector - SUNXI_IRQ_GPIOB_NS) >> 1) + 1;
    RT_ASSERT(port <= GPIO_PORT_G);

    pin = 0;
    addr = GPIOn_INT_STA_ADDR(port);
    pend = readl(addr);
    addr = GPIOn_INT_CTRL_ADDR(port);
    enable = readl(addr);
    pend &= enable;

    while (pend)
    {
        if ((pend & 0x1) && (irq_def->irq_cb[pin] != RT_NULL))
        {
            GPIO_INFO("do irq callback...", port, pin);
            irq_def->irq_cb[pin](irq_def->irq_arg[pin]);
        }
        pin++;
        pend = pend >> 1;
        hal_gpio_irq_clear(port, pin);
    }
}

void hal_gpio_init(void)
{
    hal_request_irq(SUNXI_IRQ_GPIOB_NS, hal_gpio_irq_handler, "gpiob_irq", &_g_gpio_irq_tbl[GPIO_PORT_B]);
    hal_enable_irq(SUNXI_IRQ_GPIOB_NS);

    hal_request_irq(SUNXI_IRQ_GPIOC_NS, hal_gpio_irq_handler, "gpioc_irq", &_g_gpio_irq_tbl[GPIO_PORT_C]);
    hal_enable_irq(SUNXI_IRQ_GPIOC_NS);

    hal_request_irq(SUNXI_IRQ_GPIOD_NS, hal_gpio_irq_handler, "gpiod_irq", &_g_gpio_irq_tbl[GPIO_PORT_D]);
    hal_enable_irq(SUNXI_IRQ_GPIOD_NS);

    hal_request_irq(SUNXI_IRQ_GPIOE_NS, hal_gpio_irq_handler, "gpioe_irq", &_g_gpio_irq_tbl[GPIO_PORT_E]);
    hal_enable_irq(SUNXI_IRQ_GPIOE_NS);

    hal_request_irq(SUNXI_IRQ_GPIOF_NS, hal_gpio_irq_handler, "gpiof_irq", &_g_gpio_irq_tbl[GPIO_PORT_F]);
    hal_enable_irq(SUNXI_IRQ_GPIOF_NS);

    hal_request_irq(SUNXI_IRQ_GPIOG_NS, hal_gpio_irq_handler, "gpiog_irq", &_g_gpio_irq_tbl[GPIO_PORT_G]);
    hal_enable_irq(SUNXI_IRQ_GPIOG_NS);
}


