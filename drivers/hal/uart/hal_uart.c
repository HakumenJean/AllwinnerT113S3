/*
 * Copyright 2018 AllWinnertech  Co., Ltd
 * wangwei@allwinnertech.com
 */

#include "hal_uart.h"
#include "hal_interrupt.h"
#include "hal_clk.h"
#include "hal_reset.h"
#include "uart.h"

//#define UART_LOG_DEBUG
#define UART_INIT(fmt, ...) // hal_printf("uart: "fmt, ##__VA_ARGS__)
#define UART_ERR(fmt, ...)  //hal_printf("uart: "fmt, ##__VA_ARGS__)

#ifdef UART_LOG_DEBUG
#define UART_INFO(fmt, ...) hal_printf("[%s %d]"fmt, __func__, __LINE__, ##__VA_ARGS__)
#define UART_INFO_IRQ(fmt, ...) hal_printf("[%s %d]"fmt, __func__, __LINE__, ##__VA_ARGS__)
#else
#define UART_INFO(fmt, ...)
#define UART_INFO_IRQ(fmt, ...)
#endif

static unsigned long sunxi_uart_port[] =
{
    SUNXI_UART0_BASE, SUNXI_UART1_BASE, SUNXI_UART2_BASE, SUNXI_UART3_BASE, SUNXI_UART4_BASE, SUNXI_UART5_BASE
};

static const uint32_t g_uart_irqn[] =
{
    SUNXI_IRQ_UART0, SUNXI_IRQ_UART1, SUNXI_IRQ_UART2, SUNXI_IRQ_UART3, SUNXI_IRQ_UART4, SUNXI_IRQ_UART5
};

static uart_priv_t g_uart_priv[UART_MAX];

void hal_uart_set_format(uart_port_t uart_port, uint32_t word_length, uint32_t stop_bit, uint32_t parity)
{
    unsigned long irq_flags;

    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    const unsigned long uart_base = uart_priv->uart_base;
    uint32_t value;

    irq_flags = hal_spin_lock_irqsave(&uart_priv->spinlock);

    value = hal_readb(uart_base + UART_LCR);

    /* set word length */
    value &= ~(UART_LCR_DLEN_MASK);
    switch (word_length)
    {
        case UART_WORD_LENGTH_5:
            value |= UART_LCR_WLEN5;
            break;
        case UART_WORD_LENGTH_6:
            value |= UART_LCR_WLEN6;
            break;
        case UART_WORD_LENGTH_7:
            value |= UART_LCR_WLEN7;
            break;
        case UART_WORD_LENGTH_8:
        default:
            value |= UART_LCR_WLEN8;
            break;
    }

    /* set stop bit */
    switch (stop_bit)
    {
        case UART_STOP_BIT_1:
        default:
            value &= ~(UART_LCR_STOP);
            break;
        case UART_STOP_BIT_2:
            value |= UART_LCR_STOP;
            break;
    }

    /* set parity bit */
    value &= ~(UART_LCR_PARITY_MASK);
    switch (parity)
    {
        case UART_PARITY_NONE:
            value &= ~(UART_LCR_PARITY);
            break;
        case UART_PARITY_ODD:
            value |= UART_LCR_PARITY;
            break;
        case UART_PARITY_EVEN:
            value |= UART_LCR_PARITY;
            value |= UART_LCR_EPAR;
            break;
    }

    uart_priv->lcr = value;
    hal_writeb(uart_priv->lcr, uart_base + UART_LCR);

    hal_spin_unlock_irqrestore(&uart_priv->spinlock, irq_flags);
}

void hal_uart_set_baudrate(uart_port_t uart_port, uint32_t baudrate)
{
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    const unsigned long uart_base = uart_priv->uart_base;
    uint32_t actual_baudrate = baudrate;
    uint32_t quot, uart_clk;

    uart_clk = 24000000; /* FIXME: fixed to 24MHz */

    quot = (uart_clk + 8 * actual_baudrate) / (16 * actual_baudrate);

    UART_INFO("baudrate: %d, quot = %d\r\n", actual_baudrate, quot);

    uart_priv->dlh = quot >> 8;
    uart_priv->dll = quot & 0xff;

    /* hold tx so that uart will update lcr and baud in the gap of tx */
    hal_writeb(UART_HALT_HTX | UART_HALT_FORCECFG, uart_base + UART_HALT);
    hal_writeb(uart_priv->lcr | UART_LCR_DLAB, uart_base + UART_LCR);
    hal_writeb(uart_priv->dlh, uart_base + UART_DLH);
    hal_writeb(uart_priv->dll, uart_base + UART_DLL);
    hal_writeb(UART_HALT_HTX | UART_HALT_FORCECFG | UART_HALT_LCRUP, uart_base + UART_HALT);
    /* FIXME: implement timeout */
    while (hal_readb(uart_base + UART_HALT) & UART_HALT_LCRUP)
        ;

    /* In fact there are two DLABs(DLAB and DLAB_BAK) in the hardware implementation.
     * The DLAB_BAK is sellected only when SW_UART_HALT_FORCECFG is set to 1,
     * and this bit can be access no matter uart is busy or not.
     * So we select the DLAB_BAK always by leaving SW_UART_HALT_FORCECFG to be 1. */
    hal_writeb(uart_priv->lcr, uart_base + UART_LCR);
    hal_writeb(UART_HALT_FORCECFG, uart_base + UART_HALT);
}

static void uart_set_fifo(uart_port_t uart_port, uint32_t value)
{
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    const unsigned long uart_base = uart_priv->uart_base;

    uart_priv->fcr = value;
    hal_writeb(uart_priv->fcr, uart_base + UART_FCR);
}

void hal_uart_set_hardware_flowcontrol(uart_port_t uart_port)
{
    unsigned long irq_flags;
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    const unsigned long uart_base = uart_priv->uart_base;
    uint32_t value;

    irq_flags = hal_spin_lock_irqsave(&uart_priv->spinlock);

    value = hal_readb(uart_base + UART_MCR);
    value |= UART_MCR_DTR | UART_MCR_RTS | UART_MCR_AFE;
    uart_priv->mcr = value;
    hal_writeb(uart_priv->mcr, uart_base + UART_MCR);

    /* enable with modem status interrupts */
    value = hal_readb(uart_base + UART_IER);
    value |= UART_IER_MSI;
    uart_priv->ier = value;
    hal_writeb(uart_priv->ier, uart_base + UART_IER);

    hal_spin_unlock_irqrestore(&uart_priv->spinlock, irq_flags);
}

void hal_uart_disable_flowcontrol(uart_port_t uart_port)
{
    unsigned long irq_flags;
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    const unsigned long uart_base = uart_priv->uart_base;
    uint32_t value;

    irq_flags = hal_spin_lock_irqsave(&uart_priv->spinlock);

    value = hal_readb(uart_base + UART_MCR);
    value &= ~(UART_MCR_DTR | UART_MCR_RTS | UART_MCR_AFE);
    uart_priv->mcr = value;
    hal_writeb(uart_priv->mcr, uart_base + UART_MCR);

    /* disable with modem status interrupts */
    value = hal_readb(uart_base + UART_IER);
    value &= ~(UART_IER_MSI);
    uart_priv->ier = value;
    hal_writeb(uart_priv->ier, uart_base + UART_IER);

    hal_spin_unlock_irqrestore(&uart_priv->spinlock, irq_flags);
}

static void uart_force_idle(uart_port_t uart_port)
{
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    const unsigned long uart_base = uart_priv->uart_base;

    if (uart_priv->fcr & UART_FCR_FIFO_EN)
    {
        hal_writeb(UART_FCR_FIFO_EN, uart_base + UART_FCR);
        hal_writeb(UART_FCR_TXFIFO_RST
                | UART_FCR_RXFIFO_RST
                | UART_FCR_FIFO_EN, uart_base + UART_FCR);
        hal_writeb(0, uart_base + UART_FCR);
    }

    hal_writeb(uart_priv->fcr, uart_base + UART_FCR);
    (void)hal_readb(uart_base + UART_FCR);
}

static void uart_handle_busy(uart_port_t uart_port)
{
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    const unsigned long uart_base = uart_priv->uart_base;

    (void)hal_readb(uart_base + UART_USR);

    /*
     * Before reseting lcr, we should ensure than uart is not in busy
     * state. Otherwise, a new busy interrupt will be introduced.
     * It is wise to set uart into loopback mode, since it can cut down the
     * serial in, then we should reset fifo(in my test, busy state
     * (UART_USR_BUSY) can't be cleard until the fifo is empty).
     */
    hal_writeb(uart_priv->mcr | UART_MCR_LOOP, uart_base + UART_MCR);
    uart_force_idle(uart_port);
    hal_writeb(uart_priv->lcr, uart_base + UART_LCR);
    hal_writeb(uart_priv->mcr, uart_base + UART_MCR);
}

static void uart_irq_handler(int irq, void *dev_id)
{
    uart_priv_t *uart_priv = (uart_priv_t *)dev_id;
    uart_port_t uart_port = uart_priv->uart_port;
    const unsigned long uart_base = uart_priv->uart_base;
    uint32_t iir, lsr;

    iir = hal_readb(uart_base + UART_IIR) & UART_IIR_IID_MASK;
    lsr = hal_readb(uart_base + UART_LSR);

    if (iir == UART_IIR_IID_NOIRQ) return;

    UART_INFO_IRQ("IRQ uart%d lsr is %08x \n", uart_port, lsr);

    if (iir == UART_IIR_IID_BUSBSY)
    {
        uart_handle_busy(uart_port);
    }
    else
    {
        if(lsr & (UART_LSR_DR | UART_LSR_BI))
        {
            if(uart_priv->func)
                uart_priv->func(UART_EVENT_RX_DATA, uart_priv->arg);
        }
        else if (iir & UART_IIR_IID_CHARTO)
        {
            /* has charto irq but no dr lsr? just read and ignore */
            hal_readb(uart_base + UART_RBR);
        }
    }
}

static void uart_enable_irq(uart_port_t uart_port, uint32_t irq_type)
{
    unsigned long irq_flags;
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uint32_t value;

    irq_flags = hal_spin_lock_irqsave(&g_uart_priv[uart_port].spinlock);

    value = hal_readb(uart_base + UART_IER);
    value |= irq_type;
    hal_writeb(value, uart_base + UART_IER);

    hal_spin_unlock_irqrestore(&g_uart_priv[uart_port].spinlock, irq_flags);
}

static void uart_disable_irq(uart_port_t uart_port, uint32_t irq_type)
{
    unsigned long irq_flags;
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uint32_t value;

    irq_flags = hal_spin_lock_irqsave(&g_uart_priv[uart_port].spinlock);

    value = hal_readb(uart_base + UART_IER);
    value &= ~irq_type;
    hal_writeb(value, uart_base + UART_IER);

    hal_spin_unlock_irqrestore(&g_uart_priv[uart_port].spinlock, irq_flags);
}

static void uart_enable_busy_cfg(uart_port_t uart_port)
{
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uint32_t value;

    value = hal_readb(uart_base + UART_HALT);
    value |= UART_HALT_FORCECFG;
    hal_writeb(value, uart_base + UART_HALT);
}

static int uart_clk_init(uart_port_t uart_port, bool enable)
{
    hal_clk_status_t ret;
    hal_reset_type_t reset_type = HAL_SUNXI_RESET;
    u32  reset_id;
    hal_clk_type_t clk_type = HAL_SUNXI_CCU;
    hal_clk_id_t clk_id;
    hal_clk_t clk;
    struct reset_control *reset;

    switch (uart_port)
    {
        case 0:
            clk_id = SUNXI_CLK_UART0;
            reset_id = SUNXI_RST_UART0;
            break;
        case 1:
            clk_id = SUNXI_CLK_UART1;
            reset_id = SUNXI_RST_UART1;
            break;
        case 2:
            clk_id = SUNXI_CLK_UART2;
            reset_id = SUNXI_RST_UART2;
            break;
        case 3:
            clk_id = SUNXI_CLK_UART3;
            reset_id = SUNXI_RST_UART3;
            break;
        case 4:
            clk_id = SUNXI_CLK_UART4;
            reset_id = SUNXI_RST_UART4;
            break;
        case 5:
            clk_id = SUNXI_CLK_UART5;
            reset_id = SUNXI_RST_UART5;
            break;
        default:
            UART_ERR("uart%d is invalid\n", uart_port);
            return -1;
    }

    if (enable)
    {
        reset = hal_reset_control_get(reset_type, reset_id);
        hal_reset_control_deassert(reset);

        clk = hal_clock_get(clk_type, clk_id);
        ret = hal_clock_enable(clk);
        if (ret)
        {
            UART_ERR("[uart%d] couldn't enable clk!\n", uart_port);
            return -1;
        }
    }
    else
    {
        clk = hal_clock_get(clk_type, clk_id);
        ret = hal_clock_disable(clk);
        if (ret)
        {
            UART_ERR("[uart%d] couldn't disable clk!\n", uart_port);
            return -1;
        }
    }
    return 0;
}

_uart_config_t def_config = {
    UART_BAUDRATE_115200,
    UART_WORD_LENGTH_8,
    UART_STOP_BIT_1,
    UART_PARITY_NONE,
    0,
};

int32_t hal_uart_init(uart_port_t uart_port, uart_callback_t callback, void *data)
{
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    uint32_t irqn = g_uart_irqn[uart_port];
    uint32_t value = 0;
    char uart_name[12] = {0};

    uart_priv->uart_port = uart_port;
    uart_priv->uart_base = sunxi_uart_port[uart_port];
    uart_priv->irqn = irqn;

    uart_priv->func = callback;
    uart_priv->arg = data;

    uart_priv->spinlock = (hal_spinlock_t)HAL_SPINLOCK_INIT;
    hal_spin_lock_init(&uart_priv->spinlock);

    /* enable clk */
    uart_clk_init(uart_port, true);

    /* config uart attributes */
    hal_uart_set_format(uart_port, def_config.word_length, def_config.stop_bit, def_config.parity);
    hal_uart_set_baudrate(uart_port, def_config.baudrate);
    if(def_config.flowctrl) hal_uart_set_hardware_flowcontrol(uart_port);
    else hal_uart_disable_flowcontrol(uart_port);

    value |= UART_FCR_RXTRG_1_2 | UART_FCR_TXTRG_1_2 | UART_FCR_FIFO_EN;
    uart_set_fifo(uart_port, value);

    hal_sprintf(uart_name, "uart%d", (int)uart_port);
    if (hal_request_irq(irqn, uart_irq_handler, uart_name, uart_priv) < 0)
    {
        UART_ERR("request irq error\n");
        return -1;
    }

    /* force config */
    uart_enable_busy_cfg(uart_port);

    return 0;
}

int32_t hal_uart_deinit(uart_port_t uart_port)
{
    uart_priv_t *uart_priv = &g_uart_priv[uart_port];
    uint32_t irqn = g_uart_irqn[uart_port];

    /* disable clk */
    uart_clk_init(uart_port, false);

    uart_disable_irq(uart_port, UART_IER_RDI | UART_IER_RLSI);
    hal_disable_irq(irqn);
    hal_free_irq(irqn);

    uart_priv->uart_port = UART_MAX;
    uart_priv->irqn = 0;

    hal_spin_lock_deinit(&uart_priv->spinlock);

    return 0;
}

int32_t hal_uart_disable_rx(uart_port_t uart_port)
{
    uint32_t irqn = g_uart_irqn[uart_port];
    uart_disable_irq(uart_port, UART_IER_RDI | UART_IER_RLSI);
    hal_disable_irq(irqn);
    return 0;
}

int32_t hal_uart_enable_rx(uart_port_t uart_port)
{
    uint32_t irqn = g_uart_irqn[uart_port];
    hal_enable_irq(irqn);
    uart_enable_irq(uart_port, UART_IER_RDI | UART_IER_RLSI);
    return 0;
}

int hal_uart_putc(uart_port_t uart_port, char c)
{
    volatile uint32_t *sed_buf;
    volatile uint32_t *sta;
    const unsigned long uart_base = sunxi_uart_port[uart_port];

    sed_buf = (uint32_t *)(uart_base + UART_THR);
    sta = (uint32_t *)(uart_base + UART_USR);

    /* FIFO status, contain valid data */
    while (!(*sta & 0x02));
    *sed_buf = c;

    return 1;
}

int hal_uart_getc(uart_port_t uart_port)
{
    int ch = -1;
    volatile uint32_t *rec_buf;
    volatile uint32_t *sta;
    const unsigned long uart_base = sunxi_uart_port[uart_port];

    rec_buf = (uint32_t *)(uart_base + UART_RBR);
    sta = (uint32_t *)(uart_base + UART_USR);

    /* Receive Data Available */
    if (*sta & 0x08) ch = *rec_buf & 0xff;

    return ch;
}

void hal_uart_set_loopback(uart_port_t uart_port, bool enable)
{
    unsigned long irq_flags;
    const unsigned long uart_base = sunxi_uart_port[uart_port];
    uint32_t value;

    irq_flags = hal_spin_lock_irqsave(&g_uart_priv[uart_port].spinlock);

    value = hal_readb(uart_base + UART_MCR);
    if (enable)
        value |= UART_MCR_LOOP;
    else
        value &= ~(UART_MCR_LOOP);
    hal_writeb(value, uart_base + UART_MCR);

    hal_spin_unlock_irqrestore(&g_uart_priv[uart_port].spinlock, irq_flags);
}



