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
#include "hal_timer.h"

static struct sunxi_timer g_timer[SUNXI_TMR_NUM];

static void sunxi_timer_irq_handle(int irq, void *dev)
{
    struct sunxi_timer *timer = (struct sunxi_timer *)dev;

    /* clear pending */
    hal_writel((0x1 << timer->timer_id), (unsigned long)TIMER_IRQ_ST_REG);

    /*callback*/
    if (timer->callback != NULL)
    {
        timer->callback(timer->param);
    }
}

static void sunxi_timer_sync(hal_timer_id id)
{
    uint32_t old = hal_readl((unsigned long)TIMER_CNTVAL_REG(id));

    while ((old - hal_readl((unsigned long)TIMER_CNTVAL_REG(id))) < TIMER_SYNC_TICKS)
    {
        int i = 10;
        while (i--);
        break;
    }
}

void hal_timer_stop(hal_timer_id id)
{
    uint32_t val = hal_readl((unsigned long)TIMER_CTL_REG(id));

    hal_writel(val & ~TIMER_CTL_ENABLE, (unsigned long)TIMER_CTL_REG(id));

    sunxi_timer_sync(id);
}

void hal_timer_start(hal_timer_id id, bool periodic)
{
    uint32_t val = hal_readl((unsigned long)TIMER_CTL_REG(id));

    if (periodic)
    {
        val &= ~TIMER_CTL_ONESHOT;
    }
    else
    {
        val |= TIMER_CTL_ONESHOT;
    }

    val |= TIMER_CTL_CLK_PRES(0);           //24M
    val &= ~TIMER_CTL_CLK_SRC(0x3);
    val |= TIMER_CTL_CLK_SRC(TIMER_CTL_CLK_SRC_OSC24M);

    hal_writel(val | TIMER_CTL_ENABLE | TIMER_CTL_RELOAD, (unsigned long)TIMER_CTL_REG(id));
}

static void hal_timer_setup(uint32_t tick, hal_timer_id id)
{
    hal_writel(tick, (unsigned long)TIMER_INTVAL_REG(id));
}

int hal_timer_set(hal_timer_id id, uint32_t clock, timer_callback callback, void *callback_param)
{
    uint32_t tick;
    struct sunxi_timer *timer = NULL;

    timer = &g_timer[id];
    tick = timer->clk_rate / clock;

    if(tick < timer->min_delta_ticks || tick > timer->max_delta_ticks) {
        hal_printf("timer not support this clock\r\n");
        return -1;
    }

    if(callback != NULL) {
        timer->callback = callback;
        timer->param = callback_param;
    }

    hal_timer_stop(id);
    hal_timer_setup(tick, id);

    return 0;
}

void hal_timer_init(void)
{
    uint32_t val;
    struct sunxi_timer *timer = NULL;

    for(int i = 0; i < ARRAY_SIZE(g_timer); i++)
    {
        /* disable all timer */
        val = hal_readl((unsigned long)TIMER_CTL_REG(i));
        hal_writel(val & ~TIMER_CTL_ENABLE, (unsigned long)TIMER_CTL_REG(i));

        /* clear pending */
        hal_writel((0x1 << i), (unsigned long)TIMER_IRQ_ST_REG);

        timer = &g_timer[i];

        timer->timer_id = i;
        timer->clk_rate = hal_clk_get_rate(hal_clock_get(HAL_SUNXI_FIXED_CCU, CLK_SRC_HOSC24M));
        timer->irq = SUNXI_IRQ_TMR(i);
        timer->min_delta_ticks = TIMER_SYNC_TICKS;
        timer->max_delta_ticks = 0xffffffff;
        timer->callback = NULL;
        timer->param = NULL;

        hal_request_irq(timer->irq, sunxi_timer_irq_handle, "sunxi timer", timer);
        hal_enable_irq(timer->irq);

        /*enable timer irq*/
        val = hal_readl((unsigned long)TIMER_IRQ_EN_REG);
        val |= TIMER_IRQ_EN(i);
        hal_writel(val, (unsigned long)TIMER_IRQ_EN_REG);
    }
}



