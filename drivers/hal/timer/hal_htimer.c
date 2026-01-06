/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the people's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY’S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS’SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY’S TECHNOLOGY.
*
*
* THIS SOFTWARE IS PROVIDED BY ALLWINNER"AS IS" AND TO THE MAXIMUM EXTENT
* PERMITTED BY LAW, ALLWINNER EXPRESSLY DISCLAIMS ALL WARRANTIES OF ANY KIND,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING WITHOUT LIMITATION REGARDING
* THE TITLE, NON-INFRINGEMENT, ACCURACY, CONDITION, COMPLETENESS, PERFORMANCE
* OR MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* IN NO EVENT SHALL ALLWINNER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "hal_clk.h"
#include "hal_htimer.h"

static struct sunxi_htimer g_htimer[HAL_HRTIMER_NUM];

static void sunxi_htimer_irq_handle(int irq, void *data)
{
    struct sunxi_htimer *htimer = (struct sunxi_htimer *)data;

    /* clear pending */
    writel((0x1 << htimer->timer_id), HTIMER_IRQ_ST_REG);

    /*callback*/
    if (htimer->callback != NULL)
    {
        htimer->callback(htimer->param);
    }
}

static void hal_htimer_clk_init(void)
{
    hal_clk_t bus_clk;
    struct reset_control *reset;

    reset = hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_HSTIMER);
    hal_reset_control_deassert(reset);

    bus_clk = hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_HSTIMER);
    hal_clock_enable(bus_clk);
}

void hal_htimer_init(void)
{
    uint32_t val;
    struct sunxi_htimer *htimer = NULL;

    hal_htimer_clk_init();

    for(int i = 0; i < ARRAY_SIZE(g_htimer); i++)
    {
        /* disable all hrtimer */
        val = readl(HTIMER_CTL_REG(i));
        writel(val & ~HTIMER_CTL_ENABLE, HTIMER_CTL_REG(i));

        /* clear pending */
        writel((0x1 << i), HTIMER_IRQ_ST_REG);

        htimer = &g_htimer[i];

        htimer->timer_id = i;
        htimer->clk_rate = hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PSI_AHB));
        htimer->irq = SUNXI_IRQ_HTMR(i);
        htimer->min_delta_ticks = HTIMER_SYNC_TICKS;
        htimer->max_delta_ticks = 0xffffffff;
        htimer->callback = NULL;
        htimer->param = NULL;

        hal_request_irq(htimer->irq, sunxi_htimer_irq_handle, "sunxi htimer", htimer);
        hal_enable_irq(htimer->irq);

        /*enable timer irq*/
        val = readl(HTIMER_IRQ_EN_REG);
        val |= HTIMER_IRQ_EN(i);
        writel(val, HTIMER_IRQ_EN_REG);
    }
}

static void hal_htimer_sync(hal_htimer_id id)
{
    uint32_t old = readl(HTIMER_CNTVAL_LO_REG(id));

    while ((old - readl(HTIMER_CNTVAL_LO_REG(id))) < HTIMER_SYNC_TICKS)
    {
        int i = 10;
        while (i--);
        break;
    }
}

static void hal_htimer_setup(uint32_t tick, hal_htimer_id id)
{
    writel(tick, HTIMER_INTVAL_LO_REG(id));
}

void hal_htimer_stop(hal_htimer_id id)
{
    uint32_t val = readl(HTIMER_CTL_REG(id));

    writel(val & ~HTIMER_CTL_ENABLE, HTIMER_CTL_REG(id));

    hal_htimer_sync(id);
}

void hal_htimer_start(hal_htimer_id id, bool periodic)
{
    uint32_t val = readl(HTIMER_CTL_REG(id));

    if (periodic) val &= ~HTIMER_CTL_ONESHOT;
    else val |= HTIMER_CTL_ONESHOT;

    writel(val | HTIMER_CTL_ENABLE | HTIMER_CTL_RELOAD, HTIMER_CTL_REG(id));
}

int hal_htimer_set(hal_htimer_id id, uint32_t clock, htimer_callback callback, void *callback_param)
{
    uint32_t tick;
    struct sunxi_htimer *htimer = NULL;

    htimer = &g_htimer[id];
    tick = htimer->clk_rate / clock;

    if(tick < htimer->min_delta_ticks || tick > htimer->max_delta_ticks) {
        hal_printf("htimer not support this clock\r\n");
        return -1;
    }

    if(callback != NULL) {
        htimer->callback = callback;
        htimer->param = callback_param;
    }

    hal_htimer_stop(id);
    hal_htimer_setup(tick, id);

    return 0;
}

