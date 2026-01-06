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
#include <rtdevice.h>
#include <stdbool.h>

#include "hal_clk.h"
#include "hal_reset.h"
#include "hal_dma.h"
#include "hal_spi.h"

#include "drv_gpio.h"
#include "drv_spi.h"

#define DBG_TAG  "SPI"
#define DBG_LVL  DBG_INFO
#include <rtdbg.h>

#ifdef RT_USING_SPI

#define SPI_MODULE_CLK              (200 * 1000 * 1000U)
#define SPI_FIFO_SIZE               (64)
#define SUNXI_SPI_INT_STA_MASK      (0x77 | (0x3f << 8))
#define XFER_TIMEOUT    (5000)

typedef struct sunxi_spi_config
{
    sunxi_spi_bus_mode_t bus_mode;
    rt_uint32_t mode;
    rt_uint32_t clock_frequency;
    rt_uint32_t datawidth;

    sunxi_spi_bus_sample_mode_t bus_sample_mode;
    sunxi_spi_spi_sample_delay_t spi_sample_mode;
    rt_uint32_t sample_delay;
}sunxi_spi_config_t;

typedef struct spi_dma
{
    struct dma_slave_config config;
    struct sunxi_dma_chan *chan;
} spi_dma_t;

typedef struct
{
    const uint8_t *tx_buf;  /* Data buffer to send */
    uint32_t tx_len;        /* The total number of bytes to send */
    uint32_t tx_single_len; /* The number of bytes to send in single mode */
    uint8_t *rx_buf;        /* Received data buffer, */
    uint32_t rx_len;        /* The valid number of bytes received */
    uint8_t tx_nbits : 3;   /* Data buffer to send in nbits mode */
    uint8_t rx_nbits : 3;   /* Data buffer to received in nbits mode */
    uint8_t dummy_byte;     /* Flash send dummy byte, default 0*/
#define SPI_NBITS_SINGLE    0x01 /* 1bit transfer */
#define SPI_NBITS_DUAL      0x02 /* 2bits transfer */
#define SPI_NBITS_QUAD      0x04 /* 4bits transfer */
    uint8_t bits_per_word;  /* transfer bit_per_word */
} hal_spi_master_transfer_t;

struct sunxi_spi
{
    unsigned long base;
    rt_uint32_t irqnum;     /* no used */
    rt_uint8_t busid;
    uint8_t rx_triglevel;
    uint8_t tx_triglevel;
    bool use_dma;

    hal_clk_t pclk; /* PLL clock */
    hal_clk_t bus_clk; /* BUS clock */
    hal_clk_t mclk; /* spi module clock */
    struct reset_control *reset;

    uint32_t drq_tx;
    uint32_t drq_rx;
    spi_dma_t dma_rx;
    spi_dma_t dma_tx;
    char *align_dma_buf;
#define ALIGN_DMA_BUF_SIZE (4096 + 64)

    spi_mode_type_t mode_type;
    rt_sem_t done;
    rt_mutex_t mutex;
    bool xfer_setup;
    bool slave_aborted;
    int8_t result : 2;
#define SPI_XFER_READY 0
#define SPI_XFER_OK 1
#define SPI_XFER_FAILED -1

    struct sunxi_spi_config config;
    //hal_spi_master_transfer_t *transfer;
    struct rt_spi_message *transfer;

    struct gpio_cfg gpio_clk;
    struct gpio_cfg gpio_mosi;
    struct gpio_cfg gpio_miso;
};

static const unsigned int sunxi_spi_sample_mode[] = {
    0x100, /* SUNXI_SPI_SAMP_DELAY_CYCLE_0_0 */
    0x000, /* SUNXI_SPI_SAMP_DELAY_CYCLE_0_5 */
    0x010, /* SUNXI_SPI_SAMP_DELAY_CYCLE_1_0 */
    0x110, /* SUNXI_SPI_SAMP_DELAY_CYCLE_1_5 */
    0x101, /* SUNXI_SPI_SAMP_DELAY_CYCLE_2_0 */
    0x001, /* SUNXI_SPI_SAMP_DELAY_CYCLE_2_5 */
    0x011  /* SUNXI_SPI_SAMP_DELAY_CYCLE_3_0 */
};

static rt_bool_t sunxi_spi_clk_init(struct sunxi_spi *sspi, uint32_t mod_clk)
{
    unsigned long rate;
    int rate_pll = 0, rate_hosc = 0;
    hal_clk_t pclk_pll, pclk_hosc;

    hal_reset_control_deassert(sspi->reset);

    pclk_pll = hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_PERIPH0);
    pclk_hosc = hal_clock_get(HAL_SUNXI_FIXED_CCU, CLK_SRC_HOSC24M);

    if(hal_clk_get_rate(pclk_pll) >= mod_clk) {
        if(!hal_clk_set_parent(sspi->mclk, pclk_pll))
            rate_pll = hal_clk_round_rate(sspi->mclk, mod_clk);
    }
    if(hal_clk_get_rate(pclk_hosc) >= mod_clk) {
        if(!hal_clk_set_parent(sspi->mclk, pclk_hosc))
            rate_hosc = hal_clk_round_rate(sspi->mclk, mod_clk);
    }

    if(abs(rate_pll - mod_clk) < abs(rate_hosc - mod_clk)) {
        sspi->pclk = pclk_pll;
        LOG_D("spi%d choose pll clk mod rate %d", sspi->busid, rate_pll);
    } else {
        sspi->pclk = pclk_hosc;
        LOG_D("spi%d choose hosc clk mod rate %d", sspi->busid, rate_hosc);
    }

    if(hal_clk_set_parent(sspi->mclk, sspi->pclk)) {
        LOG_E("spi%d failed set mclk parent to pclk", sspi->busid);
        return -RT_ERROR;
    }

    rate = hal_clk_round_rate(sspi->mclk, mod_clk);
    if(hal_clk_set_rate(sspi->mclk, rate)) {
        LOG_E("spi%d failed set mclk %lu rate", sspi->busid, rate);
        return -RT_ERROR;
    }

    sspi->config.clock_frequency = rate;

    if(hal_clock_enable(sspi->bus_clk)) {
        LOG_E("spi%d failed to enable bus clk", sspi->busid);
        return -RT_ERROR;
    }

    if(hal_clock_enable(sspi->mclk)) {
        LOG_E("spi%d failed to enable mclk", sspi->busid);
        return -RT_ERROR;
    }

    LOG_I("spi%d init clock rate %lu success", sspi->busid, rate);

    return RT_EOK;
}

static int sunxi_spi_clk_exit(struct sunxi_spi *sspi)
{
    hal_clock_disable(sspi->bus_clk);
    hal_clock_disable(sspi->mclk);

    hal_clock_put(sspi->bus_clk);
    hal_clock_put(sspi->mclk);

    hal_reset_control_assert(sspi->reset);
    hal_reset_control_put(sspi->reset);

    return RT_EOK;
}

static void sunxi_spi_pinctrl_init(struct sunxi_spi *sspi)
{
    gpio_set_config(sspi->gpio_clk);
    gpio_set_config(sspi->gpio_mosi);
    gpio_set_config(sspi->gpio_miso);
}

static void sunxi_spi_soft_reset(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_GC_REG);

    reg_val |= SUNXI_SPI_GC_SRST;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_GC_REG);

    while (hal_readl(sspi->base + SUNXI_SPI_GC_REG) & SUNXI_SPI_GC_SRST);
}

static void sunxi_spi_enable_bus(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_GC_REG);
    reg_val |= SUNXI_SPI_GC_EN;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_GC_REG);
}

static void sunxi_spi_disable_bus(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_GC_REG);
    reg_val &= ~SUNXI_SPI_GC_EN;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_GC_REG);
}

static void sunxi_spi_set_master(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_GC_REG);
    reg_val |= SUNXI_SPI_GC_MODE;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_GC_REG);
}

static void sunxi_spi_set_dummy_type(struct sunxi_spi *sspi, rt_bool_t ddb)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_TC_REG);

    if (ddb)
        reg_val |= SUNXI_SPI_TC_DDB;
    else
        reg_val &= ~SUNXI_SPI_TC_DDB;

    hal_writel(reg_val, sspi->base + SUNXI_SPI_TC_REG);
}

static void sunxi_spi_ss_ctrl(struct sunxi_spi *sspi, rt_bool_t ssctl)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_TC_REG);

    if (ssctl)
        reg_val |= SUNXI_SPI_TC_SSCTL;
    else
        reg_val &= ~SUNXI_SPI_TC_SSCTL;

    hal_writel(reg_val, sspi->base + SUNXI_SPI_TC_REG);
}

static void sunxi_spi_enable_tp(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_GC_REG);
    reg_val |= SUNXI_SPI_GC_TP_EN;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_GC_REG);
}

static void sunxi_spi_disable_tp(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_GC_REG);
    reg_val &= ~SUNXI_SPI_GC_TP_EN;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_GC_REG);
}

static void sunxi_spi_ss_owner(struct sunxi_spi *sspi, rt_uint32_t owner)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_TC_REG);

    if(!owner) reg_val &= ~SUNXI_SPI_TC_SS_OWNER;
    else reg_val |= SUNXI_SPI_TC_SS_OWNER;

    hal_writel(reg_val, sspi->base + SUNXI_SPI_TC_REG);
}

static void sunxi_spi_bus_sample_mode(struct sunxi_spi *sspi, sunxi_spi_bus_sample_mode_t mode)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_GC_REG);

    if(SUNXI_SPI_SAMP_MODE_OLD == mode) reg_val &= ~SUNXI_SPI_GC_MODE_SEL;
    else reg_val |= SUNXI_SPI_GC_MODE_SEL;

    hal_writel(reg_val, sspi->base + SUNXI_SPI_GC_REG);
}

static void sunxi_spi_set_sample_delay_sw(struct sunxi_spi *sspi, rt_uint32_t status)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_SAMP_DL_REG);

    if (status)
        reg_val |= SUNXI_SPI_SAMP_DL_SW_EN;
    else
        reg_val &= ~SUNXI_SPI_SAMP_DL_SW_EN;

    hal_writel(reg_val, sspi->base + SUNXI_SPI_SAMP_DL_REG);
}

static void sunxi_spi_set_sample_mode(struct sunxi_spi *sspi, sunxi_spi_spi_sample_delay_t mode)
{
    uint32_t reg_old, reg_new;
    uint32_t sdm, sdc, sdc1;

    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_TC_REG);

    sdm = (sunxi_spi_sample_mode[mode] >> 8) & 0xf;
    sdc = (sunxi_spi_sample_mode[mode] >> 4) & 0xf;
    sdc1 = (sunxi_spi_sample_mode[mode] >> 0) & 0xf;

    if (sdm)
        reg_new |= SUNXI_SPI_TC_SDM;
    else
        reg_new &= ~SUNXI_SPI_TC_SDM;

    if (sdc)
        reg_new |= SUNXI_SPI_TC_SDC;
    else
        reg_new &= ~SUNXI_SPI_TC_SDC;

    if (sdc1)
        reg_new |= SUNXI_SPI_TC_SDC1;
    else
        reg_new &= ~SUNXI_SPI_TC_SDC1;

    if (reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_TC_REG);
}

static void sunxi_spi_set_sample_delay(struct sunxi_spi *sspi, rt_uint32_t sample_delay)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_SAMP_DL_REG);
    reg_val &= ~SUNXI_SPI_SAMP_DL_SW;
    reg_val |= (sample_delay & SUNXI_SPI_SAMP_DL_SW);
    hal_writel(reg_val, sspi->base + SUNXI_SPI_SAMP_DL_REG);
}

static void sunxi_spi_set_delay_chain(struct sunxi_spi *sspi, sunxi_spi_bus_sample_mode_t sample_mode, rt_uint32_t clk)
{
    sunxi_spi_bus_sample_mode(sspi, sample_mode);

    if(SUNXI_SPI_SAMP_MODE_OLD == sample_mode)
    {
        sunxi_spi_set_sample_delay_sw(sspi, RT_FALSE);
        if(clk >= SUNXI_SPI_SAMP_HIGH_FREQ) sunxi_spi_set_sample_mode(sspi, SUNXI_SPI_SAMP_DELAY_CYCLE_1_0);
        else if( clk <= SUNXI_SPI_SAMP_LOW_FREQ) sunxi_spi_set_sample_mode(sspi, SUNXI_SPI_SAMP_DELAY_CYCLE_0_0);
        else sunxi_spi_set_sample_mode(sspi, SUNXI_SPI_SAMP_DELAY_CYCLE_0_5);
    }
    else
    {
        sunxi_spi_set_sample_delay_sw(sspi, RT_TRUE);
        sunxi_spi_set_sample_mode(sspi, sspi->config.spi_sample_mode);
        sunxi_spi_set_sample_delay(sspi, sspi->config.sample_delay);
    }
}

static void sunxi_spi_reset_txfifo(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_FIFO_CTL_REG);

    reg_val |= SUNXI_SPI_FIFO_CTL_TX_RST;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_FIFO_CTL_REG);

    while (hal_readl(sspi->base + SUNXI_SPI_FIFO_CTL_REG) & SUNXI_SPI_FIFO_CTL_TX_RST)
        ;
}

static void sunxi_spi_reset_rxfifo(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_FIFO_CTL_REG);

    reg_val |= SUNXI_SPI_FIFO_CTL_RX_RST;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_FIFO_CTL_REG);

    while (hal_readl(sspi->base + SUNXI_SPI_FIFO_CTL_REG) & SUNXI_SPI_FIFO_CTL_RX_RST)
        ;
}

static void sunxi_spi_reset_fifo(struct sunxi_spi *sspi)
{
    sunxi_spi_reset_txfifo(sspi);
    sunxi_spi_reset_rxfifo(sspi);
}

static void sunxi_spi_enable_dma_irq(struct sunxi_spi *sspi, uint32_t bitmap)
{
    uint32_t reg_old, reg_new;
    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_FIFO_CTL_REG);
    bitmap &= SUNXI_SPI_FIFO_CTL_DRQ_EN;
    reg_new |= bitmap;
    if (reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_FIFO_CTL_REG);
}

static void sunxi_spi_disable_dma_irq(struct sunxi_spi *sspi, uint32_t bitmap)
{
    uint32_t reg_old, reg_new;
    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_FIFO_CTL_REG);
    bitmap &= SUNXI_SPI_FIFO_CTL_DRQ_EN;
    reg_new &= ~bitmap;
    if (reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_FIFO_CTL_REG);
}

static void sunxi_spi_set_fifo_trig_level_rx(struct sunxi_spi *sspi, uint32_t level)
{
    uint32_t reg_old, reg_new;
    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_FIFO_CTL_REG);
    level &= (SUNXI_SPI_FIFO_CTL_RX_TRIG_LEVEL >> SUNXI_SPI_FIFO_CTL_RX_TRIG_LEVEL_POS);
    reg_new &= ~SUNXI_SPI_FIFO_CTL_RX_TRIG_LEVEL;
    reg_new |= (level << SUNXI_SPI_FIFO_CTL_RX_TRIG_LEVEL_POS);
    if (reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_FIFO_CTL_REG);
}

static void sunxi_spi_set_fifo_trig_level_tx(struct sunxi_spi *sspi, uint32_t level)
{
    uint32_t reg_old, reg_new;
    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_FIFO_CTL_REG);
    level &= (SUNXI_SPI_FIFO_CTL_TX_TRIG_LEVEL >> SUNXI_SPI_FIFO_CTL_TX_TRIG_LEVEL_POS);
    reg_new &= ~SUNXI_SPI_FIFO_CTL_TX_TRIG_LEVEL;
    reg_new |= (level << SUNXI_SPI_FIFO_CTL_TX_TRIG_LEVEL_POS);
    if (reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_FIFO_CTL_REG);
}

static void sunxi_spi_bus_set_cs(struct sunxi_spi *sspi, rt_base_t cs_pin, rt_bool_t status)
{
    rt_pin_write(cs_pin, status);
}

static void sunxi_spi_set_cs(struct sunxi_spi *sspi, rt_base_t cs_pin, rt_bool_t status)
{
    if(HAL_SPI_BUS_BIT == sspi->config.bus_mode) {

    } else {
        sunxi_spi_bus_set_cs(sspi, cs_pin, status);
    }
}

static void sunxi_spi_config_tc(struct sunxi_spi *sspi, rt_uint32_t config)
{
    rt_uint32_t reg_old, reg_new;

    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_TC_REG);

    if(config & RT_SPI_CPOL)
        reg_new |= SUNXI_SPI_TC_CPOL;
    else
        reg_new &= ~SUNXI_SPI_TC_CPOL;

    if(config & RT_SPI_CPHA)
        reg_new |= SUNXI_SPI_TC_CPHA;
    else
        reg_new &= ~SUNXI_SPI_TC_CPHA;

    if(config & RT_SPI_MSB)
        reg_new &= ~SUNXI_SPI_TC_FBS;
    else
        reg_new |= SUNXI_SPI_TC_FBS;

    if(reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_TC_REG);
}

static void sunxi_spi_xfer_setup(struct sunxi_spi *sspi)
{
    if(sspi->xfer_setup) return;
    else sspi->xfer_setup = RT_TRUE;

    if(HAL_SPI_BUS_BIT == sspi->config.bus_mode) {

    } else {
        sunxi_spi_config_tc(sspi, sspi->config.mode);
    }
}

static void sunxi_spi_enable_irq(struct sunxi_spi *sspi, uint32_t bitmap)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_INT_CTL_REG);
    bitmap &= SUNXI_SPI_INT_CTL_MASK;
    reg_val |= bitmap;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_INT_CTL_REG);
}

static void sunxi_spi_disable_irq(struct sunxi_spi *sspi, uint32_t bitmap)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_INT_CTL_REG);
    bitmap &= SUNXI_SPI_INT_CTL_MASK;
    reg_val &= ~bitmap;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_INT_CTL_REG);
}

static uint32_t sunxi_spi_qry_irq_enable(struct sunxi_spi *sspi)
{
    return (SUNXI_SPI_INT_CTL_MASK & hal_readl(sspi->base + SUNXI_SPI_INT_CTL_REG));
}

static uint32_t sunxi_spi_qry_irq_pending(struct sunxi_spi *sspi)
{
    return (SUNXI_SPI_INT_STA_MASK & hal_readl(sspi->base + SUNXI_SPI_INT_STA_REG));
}

static void sunxi_spi_clr_irq_pending(struct sunxi_spi *sspi, rt_uint32_t pending_bit)
{
    pending_bit &= SUNXI_SPI_INT_STA_MASK;
    hal_writel(pending_bit, sspi->base + SUNXI_SPI_INT_STA_REG);
}

static void sunxi_spi_set_discard_burst(struct sunxi_spi *sspi, rt_bool_t dhb)
{
    uint32_t reg_old, reg_new;

    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_TC_REG);

    if (dhb)
        reg_new |= SUNXI_SPI_TC_DHB;
    else
        reg_new &= ~SUNXI_SPI_TC_DHB;

    if (reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_TC_REG);
}

static void sunxi_spi_disable_quad(struct sunxi_spi *sspi)
{
    uint32_t reg_new, reg_old;
    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_BCC_REG);
    reg_new &= ~SUNXI_SPI_BCC_QUAD_EN;
    if (reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_BCC_REG);
}

static void sunxi_spi_disable_dual(struct sunxi_spi *sspi)
{
    uint32_t reg_new, reg_old;
    reg_new = reg_old = hal_readl(sspi->base + SUNXI_SPI_BCC_REG);
    reg_new &= ~SUNXI_SPI_BCC_DRM;
    if (reg_new != reg_old)
        hal_writel(reg_new, sspi->base + SUNXI_SPI_BCC_REG);
}

static void sunxi_spi_set_bc_tc_stc(struct sunxi_spi *sspi, rt_uint32_t tx_len, rt_uint32_t rx_len, rt_uint32_t stc_len, rt_uint32_t dummy_cnt)
{
    uint32_t reg_val;

    /* set MBC(0x30) = tx_len + rx_len + dummy_cnt */
    reg_val = hal_readl(sspi->base + SUNXI_SPI_MBC_REG);
    reg_val &= ~SUNXI_SPI_MBC;
    reg_val |= ((tx_len + rx_len + dummy_cnt) & SUNXI_SPI_MBC);
    hal_writel(reg_val, sspi->base + SUNXI_SPI_MBC_REG);

    /* set MTC(0x34) = tx_len */
    reg_val = hal_readl(sspi->base + SUNXI_SPI_MTC_REG);
    reg_val &= ~SUNXI_SPI_MWTC;
    reg_val |= (tx_len & SUNXI_SPI_MWTC);
    hal_writel(reg_val, sspi->base + SUNXI_SPI_MTC_REG);

    /* set BBC(0x38) = dummy cnt & single mode transmit counter */
    reg_val = hal_readl(sspi->base + SUNXI_SPI_BCC_REG);
    reg_val &= ~SUNXI_SPI_BCC_STC;
    reg_val |= (stc_len & SUNXI_SPI_MWTC);
    reg_val &= ~SUNXI_SPI_BCC_DBC;
    reg_val |= (dummy_cnt & (SUNXI_SPI_BCC_DBC >> SUNXI_SPI_BCC_DBC_POS)) << SUNXI_SPI_BCC_DBC_POS;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_BCC_REG);
}

static bool sunxi_spi_can_dma(struct sunxi_spi *sspi, uint32_t len)
{
    return (sspi->use_dma && len >= SPI_FIFO_SIZE);
}

static void sunxi_spi_mode_check_master(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    if(message->send_buf && message->recv_buf)
    {
        // if (sspi->config.flash)
        sunxi_spi_set_discard_burst(sspi, RT_FALSE);
        sunxi_spi_disable_quad(sspi);
        sunxi_spi_disable_dual(sspi);
        sunxi_spi_set_bc_tc_stc(sspi, message->length, 0, message->length, 0);
        sspi->mode_type = SINGLE_FULL_DUPLEX_TX_RX;
    }
    else
    {
        sunxi_spi_set_discard_burst(sspi, RT_TRUE);
        if(message->send_buf) {
            sunxi_spi_disable_quad(sspi);
            sunxi_spi_disable_dual(sspi);
            sunxi_spi_set_bc_tc_stc(sspi, message->length, 0, message->length, 0);
            sspi->mode_type = SINGLE_HALF_DUPLEX_TX;
        } else if(message->recv_buf) {
            sunxi_spi_disable_quad(sspi);
            sunxi_spi_disable_dual(sspi);
            sunxi_spi_set_bc_tc_stc(sspi, 0, message->length, 0, 0);
            sspi->mode_type = SINGLE_HALF_DUPLEX_RX;
        }
    }
}

static void sunxi_spi_mode_check(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    switch(sspi->config.bus_mode)
    {
        case HAL_SPI_BUS_MASTER:
            sunxi_spi_mode_check_master(sspi, message);
            break;
        case HAL_SPI_BUS_SLAVE:
            break;
        case HAL_SPI_BUS_BIT:
            break;
    }
}

static void sunxi_spi_start_xfer(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_TC_REG);
    reg_val |= SUNXI_SPI_TC_XCH;
    hal_writel(reg_val, sspi->base + SUNXI_SPI_TC_REG);
}

static rt_uint32_t sunxi_spi_get_txfifo_cnt(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_FIFO_STA_REG) & SUNXI_SPI_FIFO_STA_TX_CNT;
    return (reg_val >> SUNXI_SPI_FIFO_STA_TX_CNT_POS);
}

static rt_uint32_t sunxi_spi_get_rxfifo_cnt(struct sunxi_spi *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SUNXI_SPI_FIFO_STA_REG) & SUNXI_SPI_FIFO_STA_RX_CNT;
    return (reg_val >> SUNXI_SPI_FIFO_STA_RX_CNT_POS);
}

static int sunxi_spi_cpu_rx(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    rt_uint32_t len = message->length;
    rt_uint8_t *buf = (rt_uint8_t *)message->recv_buf;
    int poll_time = 0xFFFFFF;

    while(len && poll_time)
    {
        if(sunxi_spi_get_rxfifo_cnt(sspi)) {
            *buf++ = hal_readb(sspi->base + SUNXI_SPI_RXDATA_REG);
            --len;
            poll_time = 0xFFFFFF;
        } else {
            --poll_time;
        }
    }

    if(poll_time <= 0) {
        LOG_E("spi%d cpu receive data time out", sspi->busid);
        sspi->result = SPI_XFER_FAILED;
        return -RT_ETIMEOUT;
    }

    return RT_EOK;
}

static int sunxi_spi_cpu_tx(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    rt_uint32_t len = message->length;
    rt_uint8_t *buf = (rt_uint8_t *)message->send_buf;
    int poll_time = 0xFFFFFF;

    while(len && poll_time)
    {
        if(sunxi_spi_get_txfifo_cnt(sspi) >= SPI_FIFO_SIZE) {
            --poll_time;
        } else {
            hal_writeb(*buf++, sspi->base + SUNXI_SPI_TXDATA_REG);
            --len;
            poll_time = 0xFFFFFF;
        }
    }

    if(poll_time <= 0) {
        LOG_E("spi%d cpu transfer data time out", sspi->busid);
        sspi->result = SPI_XFER_FAILED;
        return -RT_ETIMEOUT;
    }

    return RT_EOK;
}

static int sunxi_spi_cpu_tx_rx(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    int ret = RT_EOK;
    rt_uint32_t len;
    const rt_uint8_t *tx_buf = (rt_uint8_t *)message->send_buf;
    rt_uint8_t *rx_buf = (rt_uint8_t *)message->recv_buf;
    uint32_t align_loop, left_loop;
    int i = 0;
    uint8_t fifosize = SPI_FIFO_SIZE;

    len = message->length;

    align_loop = len / fifosize;
    left_loop  = len % fifosize;

    if (align_loop > 0) {
        for (i = 0; i < align_loop; i++) {
            sspi->transfer->length = fifosize;
            ret = sunxi_spi_cpu_tx(sspi, message);
            if (ret < 0)
                goto err0;
            ret = sunxi_spi_cpu_rx(sspi, message);
            if (ret < 0)
                goto err0;
            sspi->transfer->send_buf += fifosize;
            sspi->transfer->recv_buf += fifosize;
        }
    }

    if (left_loop) {
        sspi->transfer->length = left_loop;
        ret = sunxi_spi_cpu_tx(sspi, message);
        if (ret < 0)
            goto err0;
        ret = sunxi_spi_cpu_rx(sspi, message);
        if (ret < 0)
            goto err0;
    }

err0:
    sspi->transfer->length = len;
    sspi->transfer->send_buf = tx_buf;
    sspi->transfer->recv_buf = rx_buf;

    if(ret < 0)
        LOG_E("spi%d cpu tx rx error with align_%d left_%d i_%d ret_%d", sspi->busid, align_loop, left_loop, i, ret);

    return ret;
}

static void sunxi_spi_dma_cb_rx(void *data)
{
    struct sunxi_spi *sspi = (struct sunxi_spi *)data;
    struct rt_spi_message *message = sspi->transfer;
    uint32_t cnt;

    cnt = sunxi_spi_get_rxfifo_cnt(sspi);
    if (cnt > 0) {
        LOG_E("spi%d dma done but rxfifo not empty 0x%x\n", sspi->busid, cnt);
        sunxi_spi_soft_reset(sspi);
        sspi->result = SPI_XFER_FAILED;
    }

    hal_dcache_invalidate(sspi->align_dma_buf, ALIGN_UP(message->length, 64));
    rt_memcpy(message->recv_buf, sspi->align_dma_buf, message->length);

    rt_sem_release(sspi->done);
}

static void sunxi_spi_dma_cb_tx(void *data)
{
    __attribute__((__unused__)) struct sunxi_spi *sspi = (struct sunxi_spi *)data;
}

static void sunxi_spi_config_dma(struct dma_slave_config *config, int len, uint32_t triglevel, bool dma_force_fixed)
{
    int width, burst;

    if (dma_force_fixed) {
        /* if dma is force fixed, use old configuration to make sure the stability and compatibility */
        if (len % DMA_SLAVE_BUSWIDTH_4_BYTES == 0)
            width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        else
            width = DMA_SLAVE_BUSWIDTH_1_BYTE;
        burst = 4;
    } else {
        if (len % DMA_SLAVE_BUSWIDTH_4_BYTES == 0) {
            width = DMA_SLAVE_BUSWIDTH_4_BYTES;
            if (triglevel < SUNXI_SPI_FIFO_DEFAULT)
                burst = 8;
            else
                burst = 16;
        } else if (len % DMA_SLAVE_BUSWIDTH_2_BYTES == 0) {
            width = DMA_SLAVE_BUSWIDTH_2_BYTES;
            burst = 16;
        } else {
            width = DMA_SLAVE_BUSWIDTH_1_BYTE;
            burst = 16;
        }
    }

    config->src_addr_width = width;
    config->dst_addr_width = width;
    config->src_maxburst = burst;
    config->dst_maxburst = burst;
}

static int sunxi_spi_config_dma_rx(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    struct dma_slave_config *config = &sspi->dma_rx.config;

    if (message->length > ALIGN_DMA_BUF_SIZE) {
        LOG_E("spi%d rx len is over dma align buf size %d\n", sspi->busid, ALIGN_DMA_BUF_SIZE);
        return -RT_EINVAL;
    }

    rt_memset(sspi->align_dma_buf, 0, message->length);
    hal_dcache_invalidate(sspi->align_dma_buf, ALIGN_UP(message->length, 64));

    config->direction = DMA_DEV_TO_MEM;
    config->dst_addr = (unsigned long)sspi->align_dma_buf;
    config->src_addr = sspi->base + SUNXI_SPI_RXDATA_REG;
    if (sspi->config.bus_mode == HAL_SPI_BUS_SLAVE) {
        config->src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
        config->dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
        config->dst_maxburst = DMA_SLAVE_BURST_1;
        config->src_maxburst = DMA_SLAVE_BURST_1;
    } else {
        sunxi_spi_config_dma(config, message->length, sspi->rx_triglevel, true);
    }
    config->slave_id = sunxi_slave_id(DRQDST_SDRAM, sspi->drq_rx);

    if (hal_dma_slave_config(sspi->dma_rx.chan, config)) {
        LOG_E("spi%d dma rx slave config failed", sspi->busid);
        return -RT_ERROR;
    }

    if (hal_dma_prep_device(sspi->dma_rx.chan, config->dst_addr, config->src_addr, message->length, config->direction)) {
        LOG_E("spi%d dma rx prep device failed", sspi->busid);
        return -RT_ERROR;
    }

    sspi->dma_rx.chan->callback = sunxi_spi_dma_cb_rx;
    sspi->dma_rx.chan->callback_param = (void *)sspi;

    return RT_EOK;
}

static int sunxi_spi_config_dma_tx(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    struct dma_slave_config *config = &sspi->dma_tx.config;

    if (message->length > ALIGN_DMA_BUF_SIZE) {
        LOG_E("spi%d tx len is over dma align buf size %d\n", sspi->busid, ALIGN_DMA_BUF_SIZE);
        return -RT_EINVAL;
    }

    rt_memcpy(sspi->align_dma_buf, message->send_buf, message->length);
    hal_dcache_clean(sspi->align_dma_buf, ALIGN_UP(message->length, 64));

    config->direction = DMA_MEM_TO_DEV;
    config->dst_addr = sspi->base + SUNXI_SPI_TXDATA_REG;
    config->src_addr = (unsigned long)sspi->align_dma_buf;
    sunxi_spi_config_dma(config, message->length, sspi->tx_triglevel, true);
    config->slave_id = sunxi_slave_id(sspi->drq_tx, DRQDST_SDRAM);

    if (hal_dma_slave_config(sspi->dma_tx.chan, config)) {
        LOG_E("spi%d dma tx slave config failed", sspi->busid);
        return -RT_ERROR;
    }

    if (hal_dma_prep_device(sspi->dma_tx.chan, config->dst_addr, config->src_addr, message->length, config->direction)) {
        LOG_E("spi%d dma tx prep device failed", sspi->busid);
        return -RT_ERROR;
    }

    sspi->dma_tx.chan->callback = sunxi_spi_dma_cb_tx;
    sspi->dma_tx.chan->callback_param = (void *)sspi;

    return RT_EOK;
}

static int sunxi_spi_dma_rx(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    sunxi_spi_enable_dma_irq(sspi, SUNXI_SPI_FIFO_CTL_RX_DRQ_EN);
    sunxi_spi_config_dma_rx(sspi, message);
    if (hal_dma_start(sspi->dma_rx.chan)) {
        LOG_E("spi%d dma rx start error", sspi->busid);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int sunxi_spi_dma_tx(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    sunxi_spi_enable_dma_irq(sspi, SUNXI_SPI_FIFO_CTL_TX_DRQ_EN);
    sunxi_spi_config_dma_tx(sspi, message);
    if (hal_dma_start(sspi->dma_tx.chan)) {
        LOG_E("spi%d dma tx start error", sspi->busid);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t sunxi_spi_xfer_master(struct sunxi_spi *sspi, struct rt_spi_message *message)
{
    int timeout;
    bool can_dma = sunxi_spi_can_dma(sspi, message->length);

    sunxi_spi_enable_irq(sspi, SUNXI_SPI_INT_CTL_TC_EN);

    switch(sspi->mode_type)
    {
        case SINGLE_HALF_DUPLEX_RX:
        case DUAL_HALF_DUPLEX_RX:
        case QUAD_HALF_DUPLEX_RX:
            if(can_dma) {
                sunxi_spi_disable_irq(sspi, SUNXI_SPI_INT_CTL_TC_EN);
                if(sunxi_spi_dma_rx(sspi, message) < 0)
                    goto out;
                sunxi_spi_start_xfer(sspi);
            } else {
                sunxi_spi_start_xfer(sspi);
                if(sunxi_spi_cpu_rx(sspi, message) < 0)
                    goto out;
            }
            break;
        case SINGLE_HALF_DUPLEX_TX:
        case DUAL_HALF_DUPLEX_TX:
        case QUAD_HALF_DUPLEX_TX:
            if(can_dma) {
                if(sunxi_spi_dma_tx(sspi, message) < 0)
                    goto out;
                sunxi_spi_start_xfer(sspi);
            } else {
                sunxi_spi_start_xfer(sspi);
                if(sunxi_spi_cpu_tx(sspi, message) < 0)
                    goto out;
            }
            break;
        case SINGLE_FULL_DUPLEX_TX_RX:
            if(can_dma) {
                sunxi_spi_disable_irq(sspi, SUNXI_SPI_INT_CTL_TC_EN);
                if(sunxi_spi_dma_rx(sspi, message) < 0)
                    goto out;
                if(sunxi_spi_dma_tx(sspi, message) < 0)
                    goto out;
                sunxi_spi_start_xfer(sspi);
            } else {
                sunxi_spi_start_xfer(sspi);
                if(sunxi_spi_cpu_tx_rx(sspi, message))
                    goto out;
            }
            break;
        case FULL_DUPLEX_TX_RX:
            break;
        default:
            break;
    }

    timeout = rt_sem_take(sspi->done, XFER_TIMEOUT);

    if (timeout != 0) {
        LOG_E("spi%d master transfer timeout type(%d)", sspi->mode_type);
        sspi->result = SPI_XFER_FAILED;
        goto out;
    } else if (sspi->result < 0) {
        LOG_E("spi%d master transfer failed %d\n", sspi->busid, sspi->result);
        goto out;
    }

    return RT_EOK;

out:
    if (can_dma) {
        hal_dma_stop(sspi->dma_rx.chan);
        hal_dma_stop(sspi->dma_tx.chan);
        hal_dma_chan_desc_free(sspi->dma_rx.chan);
        hal_dma_chan_desc_free(sspi->dma_tx.chan);
    }

    return -RT_ERROR;
}

static int sunxi_spi_bus_handler(struct sunxi_spi *sspi)
{
    uint32_t status = 0, enable = 0, irq = 0;
    bool compelte = false;

    enable = sunxi_spi_qry_irq_enable(sspi);
    status = sunxi_spi_qry_irq_pending(sspi);
    sunxi_spi_clr_irq_pending(sspi, status);
    LOG_D("spi%d irq handler enable(%x) status(%x)", sspi->busid, enable, status);

    if ((enable & SUNXI_SPI_INT_CTL_SS_EN) && (status & SUNXI_SPI_INT_STA_SSI)) {
        LOG_D("spi%d irq bus cs invalid detect", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_SS_EN;
        sspi->result = SPI_XFER_FAILED;
        compelte = true;
    }
    if ((enable & SUNXI_SPI_INT_CTL_TC_EN) && (status & SUNXI_SPI_INT_STA_TC)) {
        LOG_D("spi%d irq bus tc comes", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_TC_EN;
        compelte = true;
    }
    if ((enable & SUNXI_SPI_INT_CTL_TX_UDR_EN) && (status & SUNXI_SPI_INT_STA_TX_UDR)) {
        LOG_E("spi%d irq bus txfifo underrun", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_TX_UDR_EN;
        sspi->result = SPI_XFER_FAILED;
        compelte = true;
    }
    if ((enable & SUNXI_SPI_INT_CTL_TX_OVF_EN) && (status & SUNXI_SPI_INT_STA_TX_OVF)) {
        LOG_E("spi%d irq bus txfifo overflow", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_TX_OVF_EN;
        sunxi_spi_reset_txfifo(sspi);
        sspi->result = SPI_XFER_FAILED;
        compelte = true;
    }
    if ((enable & SUNXI_SPI_INT_CTL_RX_UDR_EN) && (status & SUNXI_SPI_INT_STA_RX_UDR)) {
        LOG_E("spi%d irq bus rxfifo underrun", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_RX_UDR_EN;
        sspi->result = SPI_XFER_FAILED;
        compelte = true;
    }
    if ((enable & SUNXI_SPI_INT_CTL_RX_OVF_EN) && (status & SUNXI_SPI_INT_STA_RX_OVF)) {
        LOG_E("spi%d irq bus rxfifo overflow", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_RX_OVF_EN;
        sunxi_spi_reset_rxfifo(sspi);
        sspi->result = SPI_XFER_FAILED;
        compelte = true;
    }
    if ((enable & SUNXI_SPI_INT_CTL_TX_FUL_EN) && (status & SUNXI_SPI_INT_STA_TX_FULL)) {
        LOG_D("spi%d irq bus txfifo full", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_TX_FUL_EN;
    }
    if ((enable & SUNXI_SPI_INT_CTL_TX_EMP_EN) && (status & SUNXI_SPI_INT_STA_TX_EMP)) {
        LOG_D("spi%d irq bus txfifo empty", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_TX_EMP_EN;
        compelte = true;
    }
    if ((enable & SUNXI_SPI_INT_CTL_TX_ERQ_EN) && (status & SUNXI_SPI_INT_STA_TX_RDY)) {
        LOG_D("spi%d irq bus txfifo ready", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_TX_ERQ_EN;
    }
    if ((enable & SUNXI_SPI_INT_CTL_RX_FUL_EN) && (status & SUNXI_SPI_INT_STA_RX_FULL)) {
        LOG_D("spi%d irq bus rxfifo full", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_RX_FUL_EN;
    }
    if ((enable & SUNXI_SPI_INT_CTL_RX_EMP_EN) && (status & SUNXI_SPI_INT_STA_RX_EMP)) {
        LOG_D("spi%d irq bus rxfifo empty", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_RX_EMP_EN;
    }
    if ((enable & SUNXI_SPI_INT_CTL_RX_RDY_EN) && (status & SUNXI_SPI_INT_STA_RX_RDY)) {
        LOG_D("spi%d irq bus rxfifo ready", sspi->busid);
        irq |= SUNXI_SPI_INT_CTL_RX_RDY_EN;
        compelte = true;
    }

    sunxi_spi_disable_irq(sspi, irq);
    if (compelte)
        rt_sem_release(sspi->done);

    return 0;
}

static void sunxi_spi_handler(int irq, void *ptr)
{
    struct sunxi_spi *sspi = (struct sunxi_spi *)ptr;

    switch (sspi->config.bus_mode)
    {
        case HAL_SPI_BUS_MASTER:
        case HAL_SPI_BUS_SLAVE:
            sunxi_spi_bus_handler(sspi);
            break;
        case HAL_SPI_BUS_BIT:
            break;
        default:
            break;
    }
}

static int sunxi_spi_request_dma(struct sunxi_spi *sspi)
{
    if(hal_dma_chan_request(&sspi->dma_tx.chan) == HAL_DMA_CHAN_STATUS_BUSY) {
        LOG_E("spi%d failed to request dma tx channel", sspi->busid);
        return -RT_ERROR;
    }

    if(hal_dma_chan_request(&sspi->dma_rx.chan) == HAL_DMA_CHAN_STATUS_BUSY) {
        LOG_E("spi%d failed to request dma rx channel", sspi->busid);
        hal_dma_chan_free(sspi->dma_tx.chan);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int sunxi_spi_release_dma(struct sunxi_spi *sspi)
{
    if(!sspi->use_dma)
        return RT_EOK;

    if(hal_dma_stop(sspi->dma_tx.chan)) {
        LOG_E("spi%d failed stop dma tx", sspi->busid);
        return -RT_ERROR;
    }

    if(hal_dma_stop(sspi->dma_rx.chan)) {
        LOG_E("spi%d failed stop dma rx", sspi->busid);
        return -RT_ERROR;
    }

    if(hal_dma_chan_free(sspi->dma_tx.chan)) {
        LOG_E("spi%d failed free dma tx channel", sspi->busid);
        return -RT_ERROR;
    }

    if(hal_dma_chan_free(sspi->dma_rx.chan)) {
        LOG_E("spi%d failed free dma rx channel", sspi->busid);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int sunxi_spi_request_irq(struct sunxi_spi *sspi)
{
    if (hal_request_irq(sspi->irqnum, sunxi_spi_handler, "spi irq", sspi) < 0)
        return -RT_ERROR;

    hal_enable_irq(sspi->irqnum);

    return RT_EOK;
}

static void sunxi_spi_release_irq(struct sunxi_spi *sspi)
{
    hal_disable_irq(sspi->irqnum);
    hal_free_irq(sspi->irqnum);
}

static int sunxi_spi_hw_init(struct sunxi_spi *sspi)
{
    sspi->xfer_setup = false;

    if(sspi->config.clock_frequency < SUNXI_SPI_MIN_FREQUENCY || sspi->config.clock_frequency > SUNXI_SPI_MAX_FREQUENCY) {
        sspi->config.clock_frequency = SUNXI_SPI_MAX_FREQUENCY;
        LOG_E("spi%d frequency not in range, use default value %d", sspi->busid, sspi->config.clock_frequency);
    }

    if (sunxi_spi_clk_init(sspi, sspi->config.clock_frequency)) {
        LOG_E("spi%d init clk rate %d failed", sspi->busid, sspi->config.clock_frequency);
        return -RT_ERROR;
    }

    sunxi_spi_soft_reset(sspi);

    switch(sspi->config.bus_mode)
    {
        case HAL_SPI_BUS_MASTER:
            sunxi_spi_enable_bus(sspi);
            sunxi_spi_set_master(sspi);
            sunxi_spi_set_dummy_type(sspi, RT_FALSE);
            sunxi_spi_ss_ctrl(sspi, RT_FALSE);
            sunxi_spi_enable_tp(sspi);
            sunxi_spi_ss_owner(sspi, 1);
            sunxi_spi_set_delay_chain(sspi, sspi->config.bus_sample_mode, sspi->config.clock_frequency);
            break;
        case HAL_SPI_BUS_SLAVE:
            break;
        case HAL_SPI_BUS_BIT:
            break;
    }

    /* reset fifo */
    sunxi_spi_reset_fifo(sspi);
    sunxi_spi_set_fifo_trig_level_rx(sspi, sspi->rx_triglevel);
    sunxi_spi_set_fifo_trig_level_tx(sspi, sspi->tx_triglevel);

    return RT_EOK;
}

static int sunxi_spi_hw_exit(struct sunxi_spi *sspi)
{
    switch (sspi->config.bus_mode) {
        case HAL_SPI_BUS_MASTER:
            sunxi_spi_disable_bus(sspi);
            sunxi_spi_disable_tp(sspi);
            break;
        case HAL_SPI_BUS_SLAVE:
            break;
        case HAL_SPI_BUS_BIT:
            break;
    }

    sunxi_spi_clk_exit(sspi);

    return RT_EOK;
}

static rt_err_t hal_spi_init(struct sunxi_spi *sspi, struct sunxi_spi_config *cfg)
{
    rt_err_t ret;

    if(sspi->busid == 0) {
        sspi->base = SUNXI_SPI0_PBASE;
        sspi->irqnum = SPI0_IRQn;

        sspi->drq_tx = DRQDST_SPI0_TX;
        sspi->drq_rx = DRQSRC_SPI0_RX;

        sspi->reset = hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_SPI0);
        sspi->mclk = hal_clock_get(HAL_SUNXI_CCU, CLK_SPI0);
        sspi->bus_clk = hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_SPI0);
    } else {
        sspi->base = SUNXI_SPI1_PBASE;
        sspi->irqnum = SPI1_IRQn;

        sspi->drq_tx = DRQDST_SPI1_TX;
        sspi->drq_rx = DRQSRC_SPI1_RX;

        sspi->reset = hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_SPI1);
        sspi->mclk = hal_clock_get(HAL_SUNXI_CCU, CLK_SPI1);
        sspi->bus_clk = hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_SPI1);
    }

    sspi->mode_type = MODE_TYPE_NULL;
    sspi->rx_triglevel = SPI_FIFO_SIZE / 2;
    sspi->tx_triglevel = SPI_FIFO_SIZE / 2;
    rt_memcpy(&sspi->config, cfg, sizeof(sspi->config));

    if(sunxi_spi_request_dma(sspi) == RT_EOK) {
        sspi->use_dma = true;
        sspi->align_dma_buf = hal_malloc_coherent(ALIGN_DMA_BUF_SIZE);
        if(!sspi->align_dma_buf) {
            LOG_E("spi%d alloc dma coherent failed", sspi->busid);
            ret = -RT_ENOMEM;
            goto exit_dma;
        }
    } else {
        LOG_E("spi%d dma channel request failed", sspi->busid);
        sspi->use_dma = false;
    }

    if(sunxi_spi_request_irq(sspi) < 0) {
        LOG_E("spi%d request irq failed", sspi->busid);
        ret = -RT_ERROR;
        goto exit_dma_buf;
    }

    sunxi_spi_pinctrl_init(sspi);

    if(sunxi_spi_hw_init(sspi) < 0) {
        LOG_E("spi%d hw init failed", sspi->busid);
        ret = -RT_ERROR;
        goto exit_irq;
    }

    sspi->done = rt_sem_create("spi sem", 0, RT_IPC_FLAG_FIFO);
    if(!sspi->done) {
        LOG_E("spi%d create done failed", sspi->busid);
        ret = -RT_ERROR;
        goto exit_hw;
    }

    sspi->mutex = rt_mutex_create("spi mutex", RT_IPC_FLAG_FIFO);
    if(!sspi->mutex) {
        LOG_E("spi%d create mutex failed", sspi->busid);
        ret = -RT_ERROR;
        goto exit_sem;
    }

    return RT_EOK;

exit_sem:
    rt_sem_delete(sspi->done);
exit_hw:
    sunxi_spi_hw_exit(sspi);
exit_irq:
    sunxi_spi_release_irq(sspi);
exit_dma_buf:
    hal_free_coherent(sspi->align_dma_buf);
exit_dma:
    sunxi_spi_release_dma(sspi);
    return ret;
}

static rt_err_t configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration)
{
    struct sunxi_spi *sspi = (struct sunxi_spi *)device->parent.user_data;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    LOG_D("%s -> %d", __FUNCTION__, __LINE__);
    LOG_D("spi address: %08X", (rt_uint32_t)sspi->base);

    if (configuration->data_width != 8)
    {
        LOG_E("error: spi data_width is %d", configuration->data_width);
        return RT_EIO;
    }

    if(configuration->mode & RT_SPI_3WIRE) sspi->config.bus_mode = HAL_SPI_BUS_BIT;
    else if(configuration->mode & RT_SPI_SLAVE) sspi->config.bus_mode = HAL_SPI_BUS_SLAVE;
    else sspi->config.bus_mode = HAL_SPI_BUS_MASTER;

    sspi->config.mode = configuration->mode;
    sspi->config.clock_frequency = configuration->max_hz;
    sspi->config.datawidth = configuration->data_width;

    if(sunxi_spi_hw_init(sspi) < 0) {
        LOG_E("spi%d bus config failed", sspi->busid);
        return -RT_ERROR;
    }

    return RT_EOK;
};

static rt_ssize_t xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    struct sunxi_spi *sspi = (struct sunxi_spi *)device->parent.user_data;
    struct rt_spi_configuration *config = &device->config;

    RT_ASSERT(device != NULL);
    RT_ASSERT(message != NULL);

    if(config->data_width != 8) return 0;

    /* take CS */
    if (message->cs_take)
    {
        sunxi_spi_set_cs(sspi, device->cs_pin, RT_FALSE);
    }

    LOG_D("spi%d start xfer", sspi->busid);

    sunxi_spi_xfer_setup(sspi);

    switch(sspi->config.bus_mode)
    {
        case HAL_SPI_BUS_BIT:
            break;
        case HAL_SPI_BUS_SLAVE:
            break;
        default:
            sunxi_spi_reset_fifo(sspi);
            sunxi_spi_set_fifo_trig_level_rx(sspi, sspi->rx_triglevel);
            sunxi_spi_set_fifo_trig_level_tx(sspi, sspi->tx_triglevel);

            sunxi_spi_disable_irq(sspi, SUNXI_SPI_INT_CTL_MASK);
            sunxi_spi_clr_irq_pending(sspi, SUNXI_SPI_INT_STA_MASK);
            sunxi_spi_enable_irq(sspi, SUNXI_SPI_INT_CTL_ERR);
            sunxi_spi_disable_dma_irq(sspi, SUNXI_SPI_FIFO_CTL_DRQ_EN);
            break;
    }

    sunxi_spi_mode_check(sspi, message);

    sspi->transfer = message;
    sspi->reset = SPI_XFER_READY;
    rt_sem_control(sspi->done, RT_IPC_CMD_RESET, NULL);

    switch(sspi->config.bus_mode)
    {
        case HAL_SPI_BUS_MASTER:
            sunxi_spi_xfer_master(sspi, message);
            break;
        case HAL_SPI_BUS_SLAVE:
            break;
        case HAL_SPI_BUS_BIT:
            break;
    }

    if (sspi->mode_type != MODE_TYPE_NULL)
        sspi->mode_type = MODE_TYPE_NULL;

    /* release CS */
    if (message->cs_release)
    {
        sunxi_spi_set_cs(sspi, device->cs_pin, RT_TRUE);
    }

    return message->length;
};

static struct rt_spi_ops spi_ops =
{
    configure,
    xfer
};

static struct sunxi_spi_config default_cfg = {
    .bus_mode = HAL_SPI_BUS_MASTER,
    .mode = RT_SPI_MSB,
    .clock_frequency = 5000000,
    .datawidth = 8,
    .bus_sample_mode = SUNXI_SPI_SAMP_MODE_OLD,
    .spi_sample_mode = SUNXI_SPI_SAMP_DELAY_CYCLE_0_0,
    .sample_delay = 0,

};

int rt_hw_spi_init(void)
{
#ifdef BSP_USING_SPI0
    {
        static struct rt_spi_device spi_dev00;
        static struct rt_spi_device spi_dev01;
        static struct rt_spi_bus spi_bus0;
        static struct sunxi_spi spi0;

        spi0.busid = 0;
        spi0.gpio_clk = (struct gpio_cfg){ GPIOC(2), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
        spi0.gpio_mosi = (struct gpio_cfg){ GPIOC(4), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
        spi0.gpio_miso = (struct gpio_cfg){ GPIOC(5), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };

        hal_spi_init(&spi0, &default_cfg);

        /* bus register and attach device */
        rt_spi_bus_register(&spi_bus0, "spi0", &spi_ops);
        rt_spi_bus_attach_device_cspin(&spi_dev00, "spi00", "spi0", GPIOC(3), &spi0);
        rt_spi_bus_attach_device_cspin(&spi_dev01, "spi01", "spi0", GPIOF(6), &spi0);
    }
#endif

#ifdef BSP_USING_SPI1
    {
        static struct rt_spi_device spi_dev1;
        static struct rt_spi_bus spi_bus1;
        static struct sunxi_spi spi1;

        spi1.busid = 1;
//        spi1.gpio_clk = (struct gpio_cfg){ GPIOC(2), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        spi1.gpio_mosi = (struct gpio_cfg){ GPIOC(4), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        spi1.gpio_miso = (struct gpio_cfg){ GPIOC(5), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };

        hal_spi_init(&spi0, &default_cfg);

        /* bus register and attach device */
        rt_spi_bus_register(&spi_bus1, "spi1", &spi_ops);
//        rt_spi_bus_attach_device_cspin(&spi_dev1, "spi10", "spi1", GPIOC(3), &spi1);
    }
#endif

    return RT_EOK;
}
INIT_DRIVER_EARLY_EXPORT(rt_hw_spi_init);

#endif
