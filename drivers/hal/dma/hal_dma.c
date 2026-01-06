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

#include "hal_reset.h"
#include "hal_common.h"
#include "hal_clk.h"
#include "hal_interrupt.h"
#include "hal_dma.h"
#include "hal_log.h"

//#define DMA_DEBUG
#define DMA_PKG_NUM_MAX 0xFFFFFFFF
#define DMA_ERR(fmt, arg...) hal_printf("%s()%d " fmt, __func__, __LINE__, ##arg)

#ifdef DMA_START_CHAN
#define SUNXI_DMA_CHAN_START DMA_START_CHAN
#else
#define SUNXI_DMA_CHAN_START 0
#endif
#define SET_DESC_HIGH_ADDR(x) ((((u64)x >> 32) & 0x3UL) | (x & 0xFFFFFFFC))

static struct sunxi_dma_chan    dma_chan_source[NR_MAX_CHAN];
static hal_spinlock_t dma_lock;
/* Avoid repeated requests irq, which may led to unkown problem such as memory leak */
static uint32_t irq_inited = 0;

/*
 * Fix sconfig's bus width according to at_dmac.
 * 1 byte -> 0, 2 bytes -> 1, 4 bytes -> 2, 8bytes -> 3
 */
static inline uint32_t convert_buswidth(enum dma_slave_buswidth addr_width)
{
    if (addr_width > DMA_SLAVE_BUSWIDTH_8_BYTES)
    {
        return 0;
    }

    switch (addr_width)
    {
        case DMA_SLAVE_BUSWIDTH_2_BYTES:
            return 1;
        case DMA_SLAVE_BUSWIDTH_4_BYTES:
            return 2;
        case DMA_SLAVE_BUSWIDTH_8_BYTES:
            return 3;
        default:
            /* For 1 byte width or fallback */
            return 0;
    }
}

static inline void convert_burst(uint32_t *maxburst)
{
    switch (*maxburst)
    {
        case 1:
            *maxburst = 0;
            break;
        case 4:
            *maxburst = 1;
            break;
        case 8:
            *maxburst = 2;
            break;
        case 16:
            *maxburst = 3;
            break;
        default:
            hal_printf("unknown maxburst\n");
            *maxburst = 0;
            break;
    }
}

static inline void hal_dma_irq_set(uint32_t chan, bool enable)
{
    if (!enable) {
#ifdef DMA_IRQ_V2
        hal_writel(0, DMA_IRQ_EN(chan));
#else
        uint32_t high = 0;
        high = (chan >= HIGH_CHAN) ? 1 : 0;
        /*disable all dma irq*/
        hal_writel(0, DMA_IRQ_EN(high));
#endif
    }

}

static inline uint32_t hal_dma_irq_status(uint32_t chan_num, bool enable)
{
    uint32_t status = 0;
    if (enable) {
#ifdef DMA_IRQ_V2
        hal_writel(0xf, DMA_IRQ_EN(chan_num));
        hal_writel(0xf, DMA_IRQ_STAT(chan_num));
#else
        uint32_t high = 0;
        high = (chan_num >= HIGH_CHAN) ? 1 : 0;
        /*clear all dma irq pending*/
        hal_writel(0xffffffff, DMA_IRQ_STAT(high));
#endif
    } else {
#ifdef DMA_IRQ_V2
        status = hal_readl(DMA_IRQ_STAT(chan_num));
        hal_writel(status, DMA_IRQ_STAT(chan_num));
#else
        uint32_t status_l = 0, status_h = 0;
#if START_CHAN_OFFSET < HIGH_CHAN
        status_l = hal_readl(DMA_IRQ_STAT(0));
#endif
#if NR_MAX_CHAN + START_CHAN_OFFSET > HIGH_CHAN
        status_h = hal_readl(DMA_IRQ_STAT(1));
#endif
#if START_CHAN_OFFSET < HIGH_CHAN
        hal_writel(status_l, DMA_IRQ_STAT(0));
#endif
#if NR_MAX_CHAN + START_CHAN_OFFSET > HIGH_CHAN
        hal_writel(status_h, DMA_IRQ_STAT(1));
#endif
        status = (chan_num + START_CHAN_OFFSET >= HIGH_CHAN) \
                     ? (status_h >> ((chan_num + START_CHAN_OFFSET - HIGH_CHAN) << 2)) \
                    : (status_l >> ((chan_num + START_CHAN_OFFSET) << 2));
#endif
        return status;
    }
    return 0;

}

static inline void sunxi_cfg_lli(struct sunxi_dma_lli *lli, uint32_t src_addr,
                            uint32_t dst_addr, uint32_t len,
                            struct dma_slave_config *config, enum dma_transfer_direction dir)
{
    uint32_t src_width = 0, dst_width = 0;

    if (NULL == lli && NULL == config)
    {
        return;
    }

    src_width = convert_buswidth(config->src_addr_width);
    dst_width = convert_buswidth(config->dst_addr_width);
    lli->cfg = SRC_BURST(config->src_maxburst) | \
               SRC_WIDTH(src_width) | \
               DST_BURST(config->dst_maxburst) | \
               DST_WIDTH(dst_width);

//    if (dir == DMA_MEM_TO_MEM) {
//        lli->src = src_addr;
//        lli->dst = dst_addr;
//    } else if (dir == DMA_MEM_TO_DEV) {
//        lli->src = src_addr;
//        lli->dst = dst_addr;
//    } else if (dir == DMA_DEV_TO_MEM) {
//        lli->src = src_addr;
//        lli->dst = dst_addr;
//    } else {
        lli->src = src_addr;
        lli->dst = dst_addr;
//    }
    lli->len = len;
    lli->para = NORMAL_WAIT;
}

static void sunxi_dump_lli(struct sunxi_dma_chan *chan, struct sunxi_dma_lli *lli)
{
#ifdef DMA_DEBUG
    hal_printf("channum:%x\n"
           "\t\tdesc:desc - 0x%08x desc p - 0x%08x desc v - 0x%08x\n"
           "\t\tlli: v- 0x%08x v_lln - 0x%08x s - 0x%08x d - 0x%08x c - 0x%08x\n"
           "\t\tdist - 0x%08x len - 0x%08x para - 0x%08x p_lln - 0x%08x\n",
           chan->chan_count,
       (uint32_t)chan->desc, (uint32_t)chan->desc->p_lln, (uint32_t)chan->desc->vlln,
       (uint32_t)lli, (uint32_t)lli->vlln, (uint32_t)lli->src, (uint32_t)lli->dst, (uint32_t)lli->cfg,
           (uint32_t)lli->dst, (uint32_t)lli->len, (uint32_t)lli->para, (uint32_t)lli->p_lln);
#endif
}


static void sunxi_dump_com_regs(void)
{
#ifdef DMA_DEBUG
    hal_printf("Common register:\n"
           "\tmask0: 0x%08x\n"
           "\tmask1: 0x%08x\n"
           "\tpend0: 0x%08x\n"
           "\tpend1: 0x%08x\n"
#ifdef DMA_SECURE
           "\tsecur: 0x%08x\n"
#endif
#ifdef DMA_GATE
           "\t_gate: 0x%08x\n"
#endif
           "\tstats: 0x%08x\n",
           (uint32_t)hal_readl(DMA_IRQ_EN(0)),
           (uint32_t)hal_readl(DMA_IRQ_EN(1)),
           (uint32_t)hal_readl(DMA_IRQ_STAT(0)),
           (uint32_t)hal_readl(DMA_IRQ_STAT(1)),
#ifdef DMA_SECURE
           (uint32_t)hal_readl(DMA_SECURE),
#endif
#ifdef DMA_GATE
           (uint32_t)hal_readl(DMA_GATE),
#endif
           (uint32_t)hal_readl(DMA_STAT));
#endif
}

static inline void sunxi_dump_chan_regs(struct sunxi_dma_chan *ch)
{
#ifdef DMA_DEBUG
    u32 chan_num = ch->chan_count;
    hal_printf("Chan %d reg:\n"
           "\t___en: \t0x%08x\n"
           "\tpause: \t0x%08x\n"
           "\tstart: \t0x%08x\n"
           "\t__cfg: \t0x%08x\n"
           "\t__src: \t0x%08x\n"
           "\t__dst: \t0x%08x\n"
           "\tcount: \t0x%08x\n"
           "\t_para: \t0x%08x\n\n",
           chan_num,
           (uint32_t)hal_readl(DMA_ENABLE(chan_num)),
           (uint32_t)hal_readl(DMA_PAUSE(chan_num)),
           (uint32_t)hal_readl(DMA_LLI_ADDR(chan_num)),
           (uint32_t)hal_readl(DMA_CFG(chan_num)),
           (uint32_t)hal_readl(DMA_CUR_SRC(chan_num)),
           (uint32_t)hal_readl(DMA_CUR_DST(chan_num)),
           (uint32_t)hal_readl(DMA_CNT(chan_num)),
           (uint32_t)hal_readl(DMA_PARA(chan_num)));
#endif
}

static void *sunxi_lli_list(struct sunxi_dma_lli *prev, struct sunxi_dma_lli *next,
                        struct sunxi_dma_chan *chan)
{
    uint32_t temp_desc;

    if ((!prev && !chan) || !next)
    {
        return NULL;
    }

    temp_desc = __va_to_pa((unsigned long)next);
    if (!prev)
    {
        chan->desc = next;
        chan->desc->p_lln = temp_desc;
        chan->desc->vlln = next;
    }
    else
    {
        prev->p_lln = temp_desc;
        prev->vlln = next;
    }

    next->p_lln = LINK_END;
    next->vlln = NULL;

    return next;
}

static void sunxi_dma_irq_handle(int irq, void *ptr)
{
    int i = 0;
#ifndef DMA_IRQ_V2
    uint32_t status_l = 0, status_h = 0;
#if START_CHAN_OFFSET < HIGH_CHAN
    status_l = hal_readl(DMA_IRQ_STAT(0));
#endif
#if NR_MAX_CHAN + START_CHAN_OFFSET > HIGH_CHAN
    status_h = hal_readl(DMA_IRQ_STAT(1));
#endif
#if START_CHAN_OFFSET < HIGH_CHAN
    hal_writel(status_l, DMA_IRQ_STAT(0));
#endif
#if NR_MAX_CHAN + START_CHAN_OFFSET > HIGH_CHAN
    hal_writel(status_h, DMA_IRQ_STAT(1));
#endif
#endif

    for (i = SUNXI_DMA_CHAN_START; i < NR_MAX_CHAN; i++)
    {
        unsigned long __cpsr;
        struct sunxi_dma_chan *chan = &dma_chan_source[i];

        uint32_t chan_num = chan->chan_count;
        uint32_t status = 0;

        if (chan->used == 0) continue;

        __cpsr = hal_spin_lock_irqsave(&chan->lock);

#ifdef DMA_IRQ_V2
        status = hal_dma_irq_status(chan_num, 0);
#else
        status = (chan_num + START_CHAN_OFFSET >= HIGH_CHAN) \
                 ? (status_h >> ((chan_num + START_CHAN_OFFSET - HIGH_CHAN) << 2)) \
                 : (status_l >> ((chan_num + START_CHAN_OFFSET) << 2));
#endif
        if (!(chan->irq_type & status))
        {
            hal_spin_unlock_irqrestore(&chan->lock, __cpsr);
            continue;
        }

        if (chan->cyclic)
        {
            dma_callback cb = NULL;
            void *cb_data = NULL;

            chan->periods_pos ++;
            if (chan->periods_pos * chan->desc->len >= chan->buf_len)
            {
                chan->periods_pos = 0;
            }
            cb = chan->callback;
            cb_data = chan->callback_param;

            hal_spin_unlock_irqrestore(&chan->lock, __cpsr);
            if (cb)
            {
                cb(cb_data);
            }
        }
        else
        {
            dma_callback cb = NULL;
            void *cb_data = NULL;

            cb = chan->callback;
            cb_data = chan->callback_param;

            hal_spin_unlock_irqrestore(&chan->lock, __cpsr);
            if (cb)
            {
                cb(cb_data);
            }
        }
    }
}

static int sunxi_dma_clk_init(bool enable)
{
    hal_clk_status_t ret;
    u32  reset_id;
    hal_clk_id_t clk_id, mbus_clk_id, mcu_clk_id;
    hal_clk_t clk;
    struct reset_control *reset;

    clk_id = SUNXI_CLK_DMA;
    reset_id = SUNXI_RST_DMA;
    mbus_clk_id = SUNXI_CLK_MBUS_DMA;
#ifdef CONFIG_ARCH_SUN55IW3
    hal_reset_type_t reset_type = HAL_SUNXI_MCU_RESET;
    hal_clk_type_t clk_type = HAL_SUNXI_MCU;
    mcu_clk_id = SUNXI_MCU_CLK;
#else
    hal_reset_type_t reset_type = HAL_SUNXI_RESET;
    hal_clk_type_t clk_type = HAL_SUNXI_CCU;
    mcu_clk_id = 0;
#endif

    DMA_ERR("enter dma clk init\n");
    if (enable)
    {
    if (reset_id) {
        reset = hal_reset_control_get(reset_type, reset_id);
        hal_reset_control_deassert(reset);
        hal_reset_control_put(reset);
    }
    if (mbus_clk_id)
        hal_clock_enable(hal_clock_get(clk_type, SUNXI_CLK_MBUS_DMA));

    clk = hal_clock_get(clk_type, clk_id);
    ret = hal_clock_enable(clk);
    if (ret != HAL_CLK_STATUS_OK)
        DMA_ERR("DMA clock enable failed.\n");
    if (mcu_clk_id)
        hal_clock_enable(hal_clock_get(clk_type, mcu_clk_id));
    }
    else
    {
    clk = hal_clock_get(clk_type, clk_id);
    ret = hal_clock_disable(clk);
    if (ret != HAL_CLK_STATUS_OK)
        DMA_ERR("DMA clock disable failed.\n");
    if (mbus_clk_id)
        hal_clock_disable(hal_clock_get(clk_type, SUNXI_CLK_MBUS_DMA));
    hal_clock_put(clk);
    if (reset_id) {
        reset = hal_reset_control_get(reset_type, reset_id);
        hal_reset_control_assert(reset);
        hal_reset_control_put(reset);
    }
    }
    return ret;
}

void sunxi_dma_free_ill(struct sunxi_dma_chan *chan)
{
    struct sunxi_dma_lli *li_adr = NULL, *next = NULL;
    unsigned long __cpsr;

    if (NULL == chan)
    {
        DMA_ERR("[dma] chan is NULL\n");
        return ;
    }

    __cpsr = hal_spin_lock_irqsave(&chan->lock);
    li_adr = chan->desc;
    chan->desc = NULL;
    chan->callback = NULL;
    chan->callback_param = NULL;
    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    while (li_adr)
    {
        next = li_adr->vlln;
        hal_free_coherent(li_adr);
        li_adr = next;
    }
}

hal_dma_chan_status_t hal_dma_chan_request(struct sunxi_dma_chan **dma_chan)
{
    int i = 0;
    struct sunxi_dma_chan *chan;
    unsigned long __cpsr;

    __cpsr = hal_spin_lock_irqsave(&dma_lock);
    for (i = SUNXI_DMA_CHAN_START; i < NR_MAX_CHAN; i++)
    {
        chan = &dma_chan_source[i];
        if (chan->used == 0)
        {
            chan->used = 1;
            chan->chan_count = i;
            hal_spin_unlock_irqrestore(&dma_lock, __cpsr);
            *dma_chan = &dma_chan_source[i];
            return HAL_DMA_CHAN_STATUS_FREE;
        }
    }
    hal_spin_unlock_irqrestore(&dma_lock, __cpsr);

    return HAL_DMA_CHAN_STATUS_BUSY;
}

hal_dma_status_t hal_dma_prep_memcpy(struct sunxi_dma_chan *chan,
                       uint32_t dest, uint32_t src, uint32_t len)
{
    struct sunxi_dma_lli *l_item = NULL;
    struct dma_slave_config *config = NULL;
    unsigned long __cpsr;

    if ((NULL == chan) || (dest == 0 || src == 0))
    {
        DMA_ERR("[dma] chan is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    hal_assert(!hal_interrupt_is_disable());
    hal_assert(!in_interrupt());

    l_item = (struct sunxi_dma_lli *)hal_malloc_coherent(sizeof(struct sunxi_dma_lli));
    if (!l_item)
        return HAL_DMA_STATUS_NO_MEM;
    hal_memset(l_item, 0, sizeof(struct sunxi_dma_lli));

    __cpsr = hal_spin_lock_irqsave(&chan->lock);

    config = &chan->cfg;
    dest = __va_to_pa(dest);
    src = __va_to_pa(src);
    sunxi_cfg_lli(l_item, src, dest, len, config, DMA_MEM_TO_MEM);

    l_item->cfg |= SRC_DRQ(DRQSRC_SDRAM) \
                   | DST_DRQ(DRQDST_SDRAM) \
                   | DST_LINEAR_MODE \
                   | SRC_LINEAR_MODE;
    sunxi_lli_list(NULL, l_item, chan);
    sunxi_dump_lli(chan, l_item);

    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    return HAL_DMA_STATUS_OK;
}

hal_dma_status_t hal_dma_prep_device(struct sunxi_dma_chan *chan,
                       uint32_t dest, uint32_t src,
                       uint32_t len, enum dma_transfer_direction dir)
{
    struct sunxi_dma_lli *l_item = NULL;
    struct dma_slave_config *config = NULL;
    unsigned long __cpsr;

    if ((NULL == chan) || (dest == 0 || src == 0))
    {
        DMA_ERR("[dma] chan is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    hal_assert(!hal_interrupt_is_disable());
    hal_assert(!in_interrupt());

    l_item = (struct sunxi_dma_lli *)hal_malloc_coherent(sizeof(struct sunxi_dma_lli));
    if (!l_item)
        return HAL_DMA_STATUS_NO_MEM;
    hal_memset(l_item, 0, sizeof(struct sunxi_dma_lli));

    __cpsr = hal_spin_lock_irqsave(&chan->lock);

    config = &chan->cfg;

    if (dir == DMA_MEM_TO_DEV)
    {
        src = __va_to_pa(src);
        sunxi_cfg_lli(l_item, src, dest, len, config, dir);
        l_item->cfg |= GET_DST_DRQ(config->slave_id) \
                       | SRC_LINEAR_MODE \
                       | DST_IO_MODE \
                       | SRC_DRQ(DRQSRC_SDRAM);
    }
    else if (dir == DMA_DEV_TO_MEM)
    {
        dest = __va_to_pa(dest);
        sunxi_cfg_lli(l_item, src, dest, len, config, dir);
        l_item ->cfg |= GET_SRC_DRQ(config->slave_id)  \
                        | DST_LINEAR_MODE \
                        | SRC_IO_MODE \
                        | DST_DRQ(DRQSRC_SDRAM);
    }
    else if (dir == DMA_DEV_TO_DEV)
    {
        sunxi_cfg_lli(l_item, src, dest, len, config, dir);
        l_item->cfg |= GET_SRC_DRQ(config->slave_id) \
                       | DST_IO_MODE \
                       | SRC_IO_MODE \
                       | GET_DST_DRQ(config->slave_id);
    }

    sunxi_lli_list(NULL, l_item, chan);

    sunxi_dump_lli(chan, l_item);

    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    return HAL_DMA_STATUS_OK;
}

hal_dma_status_t hal_dma_prep_cyclic(struct sunxi_dma_chan *chan,
                     uint32_t buf_addr, uint32_t buf_len,
                     uint32_t period_len, enum dma_transfer_direction dir)
{
    struct sunxi_dma_lli *prev = NULL, *lli_old = NULL;
    uint32_t periods = buf_len / period_len;
    struct dma_slave_config *config = NULL;
    uint32_t i = 0;
    struct sunxi_dma_lli *l_item[periods];
    unsigned long __cpsr;

    if ((NULL == chan && chan->cyclic) || (0 == buf_addr))
    {
        DMA_ERR("[dma] chan or buf_addr is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

//    hal_assert(!hal_interrupt_is_disable());
//    hal_assert(!in_interrupt());

    hal_memset(l_item, 0, sizeof(l_item));
    for (i = 0; i < periods; i++) {
        l_item[i] = hal_malloc_coherent(sizeof(struct sunxi_dma_lli));
        if (!l_item[i])
            goto no_mem;
        hal_memset(l_item[i], 0, sizeof(struct sunxi_dma_lli));
    }

    __cpsr = hal_spin_lock_irqsave(&chan->lock);

    chan->new_val = 0;
    chan->old_val = 0;
    chan->val = 0;

    if (chan->desc) {
        lli_old = chan->desc;
        chan->desc = NULL;
    }

    config = &chan->cfg;
    for (i = 0; i < periods; i++)
    {
        if (dir == DMA_MEM_TO_DEV)
        {
            sunxi_cfg_lli(l_item[i], __va_to_pa(buf_addr + period_len * i),
                          config->dst_addr, period_len, config, dir);
            l_item[i]->cfg |= GET_DST_DRQ(config->slave_id) \
                           | SRC_LINEAR_MODE \
                           | DST_IO_MODE \
                           | SRC_DRQ(DRQSRC_SDRAM);
        }
        else if (dir == DMA_DEV_TO_MEM)
        {
            sunxi_cfg_lli(l_item[i], config->src_addr, \
                          __va_to_pa(buf_addr + period_len * i), \
                          period_len, config, dir);
            l_item[i]->cfg |= GET_SRC_DRQ(config->slave_id)  \
                            | DST_LINEAR_MODE \
                            | SRC_IO_MODE \
                            | DST_DRQ(DRQSRC_SDRAM);
        }
        else if (dir == DMA_DEV_TO_DEV)
        {
            sunxi_cfg_lli(l_item[i], config->src_addr, \
                          config->dst_addr, period_len, config, dir);
            l_item[i]->cfg |= GET_SRC_DRQ(config->slave_id) \
                           | DST_IO_MODE \
                           | SRC_IO_MODE \
                           | GET_DST_DRQ(config->slave_id);

        }
        else if (dir == DMA_MEM_TO_MEM)
        {
            sunxi_cfg_lli(l_item[i], config->src_addr, \
                          config->dst_addr, period_len, config, dir);
            l_item[i]->cfg |= GET_SRC_DRQ(config->slave_id) \
                           | DST_LINEAR_MODE \
                           | SRC_LINEAR_MODE \
                           | GET_DST_DRQ(config->slave_id);

        }

        prev = sunxi_lli_list(prev, l_item[i], chan);
    }
    prev->p_lln = __va_to_pa((unsigned long)chan->desc);
    chan->cyclic = true;
#ifdef DMA_DEBUG
    for (prev = chan->desc; prev != NULL; prev = prev->vlln)
    {
        sunxi_dump_lli(chan, prev);
    }
#endif
    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    if (lli_old) {
        struct sunxi_dma_lli *next = NULL;

        while (lli_old)
        {
            next = lli_old->vlln;
            hal_free_coherent(lli_old);
            lli_old = next;
        }
    }

    return HAL_DMA_STATUS_OK;

no_mem:
    for (i = 0; i < periods; i++) {
        if (!l_item[i])
            continue;
        hal_free_coherent(l_item[i]);
    }
    return HAL_DMA_STATUS_NO_MEM;
}

hal_dma_status_t hal_dma_callback_install(struct sunxi_dma_chan *chan,
                      dma_callback callback,
                      void *callback_param)
{
    unsigned long __cpsr;

    if (NULL == chan)
    {
        DMA_ERR("[dma] chan is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    if (NULL == callback)
    {
        DMA_ERR("[dma] callback is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    if (NULL == callback_param)
    {
        DMA_ERR("[dma] callback_param is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    __cpsr = hal_spin_lock_irqsave(&chan->lock);
    chan->callback = callback;
    chan->callback_param = callback_param;
    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    return HAL_DMA_STATUS_OK;
}

hal_dma_status_t hal_dma_slave_config(struct sunxi_dma_chan *chan,
                      struct dma_slave_config *config)
{
    unsigned long __cpsr;

    if (NULL == config || NULL == chan)
    {
        DMA_ERR("[dma] dma config is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    __cpsr = hal_spin_lock_irqsave(&chan->lock);
    convert_burst(&config->src_maxburst);
    convert_burst(&config->dst_maxburst);
    hal_memcpy((void *) & (chan->cfg), (void *)config, sizeof(struct dma_slave_config));
    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    return HAL_DMA_STATUS_OK;
}

hal_dma_status_t hal_dma_tx_pkgnum(struct sunxi_dma_chan *chan)
{
    int pkgnum;

    pkgnum = hal_readl(DMA_PKGNUM(chan->chan_count));
    return  pkgnum;
}


uint32_t hal_dma_pkg_status(struct sunxi_dma_chan *chan, int buf_size, uint32_t *total)
{
    size_t bytes;

    chan->new_val = hal_readl(DMA_PKGNUM(chan->chan_count));

    if (chan->new_val < chan->old_val)
        chan->val += DMA_PKG_NUM_MAX - chan->old_val + 1 + chan->new_val;
    else
        chan->val += chan->new_val - chan->old_val;

    chan->old_val = chan->new_val;
    chan->val = chan->val % buf_size;
    bytes = buf_size - chan->val;

    *total = chan->new_val;
    return bytes;
}

enum dma_status hal_dma_tx_status(struct sunxi_dma_chan *chan, uint32_t *left_size)
{
    uint32_t i = 0;
    struct sunxi_dma_lli *l_item = NULL;
    enum dma_status status = DMA_INVALID_PARAMETER;
    unsigned long __cpsr;

    if (NULL == chan || NULL == left_size)
    {
        DMA_ERR("[dma] chan or left_size is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    __cpsr = hal_spin_lock_irqsave(&chan->lock);
    if (chan->cyclic)
    {
        for (i = 0, l_item = chan->desc; i <= chan->periods_pos; i++, l_item = l_item->vlln)
        {
            if (NULL == l_item)
            {
                *left_size = 0;
                status = DMA_COMPLETE;
                goto unlock;
            }
        }
        if (NULL == l_item)
        {
            *left_size = 0;
            status = DMA_COMPLETE;
        }
        else
        {
            uint32_t pos = 0;
            bool count = false;

            pos = hal_readl(DMA_LLI_ADDR(chan->chan_count));
            *left_size = hal_readl(DMA_CNT(chan->chan_count));
            if (pos == LINK_END)
            {
                status = DMA_COMPLETE;
                goto unlock;
            }
            for (l_item = chan->desc; l_item != NULL; l_item = l_item->vlln)
            {
                if (l_item->p_lln == pos)
                {
                    count = true;
                    continue;
                }
                if (count)
                {
                    *left_size += l_item->len;
                }
            }
            status = DMA_IN_PROGRESS;
        }
    }
    else
    {
        *left_size = hal_readl(DMA_CNT(chan->chan_count));

        if (*left_size == 0)
        {
            status = DMA_COMPLETE;
        }
        else
        {
            status = DMA_IN_PROGRESS;
        }
    }

unlock:
    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    //while(!((hal_readl(DMA_STAT) >> chan->chan_count) & 0x01));

    return status;
}

hal_dma_status_t hal_dma_start(struct sunxi_dma_chan *chan)
{
    uint32_t irq_val = 0;
    //uint32_t high = 0;
    struct sunxi_dma_lli *prev = NULL;
    unsigned long __cpsr, __ccpsr;
    uint32_t desc_addr;

    if (NULL == chan)
    {
        DMA_ERR("[dma] chan is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    __cpsr = hal_spin_lock_irqsave(&chan->lock);

    if (chan->cyclic)
        chan->irq_type = IRQ_PKG;
    else
        chan->irq_type = IRQ_QUEUE;

#ifdef DMA_IRQ_V2
    __ccpsr = hal_spin_lock_irqsave(&dma_lock);
    irq_val = hal_readl(DMA_IRQ_EN(chan->chan_count));
    irq_val |= chan->irq_type;
    hal_writel(irq_val, DMA_IRQ_EN(chan->chan_count));
    hal_spin_unlock_irqrestore(&dma_lock, __ccpsr);
    irq_val = hal_readl(DMA_IRQ_EN(chan->chan_count));
#else
    uint32_t high = 0;
    high = (chan->chan_count + START_CHAN_OFFSET >= HIGH_CHAN) ? 1 : 0;

    __ccpsr = hal_spin_lock_irqsave(&dma_lock);
    irq_val = hal_readl(DMA_IRQ_EN(high));
    irq_val |= SHIFT_IRQ_MASK(chan->irq_type, chan->chan_count);
    hal_writel(irq_val, DMA_IRQ_EN(high));
    hal_spin_unlock_irqrestore(&dma_lock, __ccpsr);
#endif

    /* FlashCtrl cannot support handshake mode  */
    /* SET_OP_MODE(chan->chan_count, SRC_HS_MASK | DST_HS_MASK); */

    for (prev = chan->desc; prev != NULL; prev = prev->vlln)
    {
        hal_dcache_clean(prev, sizeof(struct sunxi_dma_lli));
    }

    desc_addr = __va_to_pa((unsigned long)chan->desc);
    hal_writel(SET_DESC_HIGH_ADDR(desc_addr), DMA_LLI_ADDR(chan->chan_count));
    hal_writel(CHAN_START, DMA_ENABLE(chan->chan_count));
    sunxi_dump_com_regs();
    sunxi_dump_chan_regs(chan);
    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);
    return HAL_DMA_STATUS_OK;
}

hal_dma_status_t hal_dma_stop(struct sunxi_dma_chan *chan)
{
    unsigned long __cpsr;

    if (NULL == chan)
    {
        DMA_ERR("[dma] chan is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    __cpsr = hal_spin_lock_irqsave(&chan->lock);
    /*We should entry PAUSE state first to avoid missing data
    * count witch transferring on bus.
    */
    hal_writel(CHAN_PAUSE, DMA_PAUSE(chan->chan_count));
    hal_writel(CHAN_STOP, DMA_ENABLE(chan->chan_count));
    hal_writel(CHAN_RESUME, DMA_PAUSE(chan->chan_count));

    if (chan->cyclic)
    {
        chan->cyclic = false;
    }
    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    return HAL_DMA_STATUS_OK;
}

hal_dma_status_t hal_dma_chan_free(struct sunxi_dma_chan *chan)
{
    unsigned long irq_val = 0;
    //uint32_t high = 0;
    unsigned long __cpsr, __ccpsr;

    if (NULL == chan)
    {
        DMA_ERR("[dma] chan is NULL\n");
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    if (!chan->used)
    {
        return HAL_DMA_STATUS_INVALID_PARAMETER;
    }

    __cpsr = hal_spin_lock_irqsave(&chan->lock);

#ifdef DMA_IRQ_V2
    __ccpsr = hal_spin_lock_irqsave(&dma_lock);
    irq_val = hal_readl(DMA_IRQ_EN(chan->chan_count));
    irq_val &= ~chan->irq_type;
    hal_writel(irq_val, DMA_IRQ_EN(chan->chan_count));
    hal_spin_unlock_irqrestore(&dma_lock, __ccpsr);
#else
    uint32_t high = 0;
    high = (chan->chan_count + START_CHAN_OFFSET >= HIGH_CHAN) ? 1 : 0;

    __ccpsr = hal_spin_lock_irqsave(&dma_lock);
    irq_val = hal_readl(DMA_IRQ_EN(high));
    irq_val &= ~(SHIFT_IRQ_MASK(chan->irq_type, chan->chan_count));
    hal_writel(irq_val, DMA_IRQ_EN(high));
    hal_spin_unlock_irqrestore(&dma_lock, __ccpsr);
#endif

    //sunxi_dma_free_ill(chan);
    struct sunxi_dma_lli *li_adr = NULL, *next = NULL;
    li_adr = chan->desc;
    chan->desc = NULL;
    chan->callback = NULL;
    chan->callback_param = NULL;
    chan->used = 0;
    hal_spin_unlock_irqrestore(&chan->lock, __cpsr);

    while (li_adr)
    {
        next = li_adr->vlln;
        hal_free_coherent(li_adr);
        li_adr = next;
    }

    return HAL_DMA_STATUS_OK;
}

hal_dma_status_t hal_dma_chan_desc_free(struct sunxi_dma_chan *chan)
{
    hal_assert(!hal_interrupt_is_disable());
    hal_assert(!in_interrupt());

    sunxi_dma_free_ill(chan);

    return HAL_DMA_STATUS_OK;
}

void hal_dma_resource_get(void)
{
    uint32_t i = 0, secure = 0;
    struct sunxi_dma_chan *chan;

    dma_lock = (hal_spinlock_t)HAL_SPINLOCK_INIT;
    hal_spin_lock_init(&dma_lock);

    /* Detect that if there is a master that has not freed DMA, it will not memset! */
    for (i = SUNXI_DMA_CHAN_START; i < NR_MAX_CHAN; i++)
    {
        chan = &dma_chan_source[i];
        chan->lock = (hal_spinlock_t)HAL_SPINLOCK_INIT;
        hal_spin_lock_init(&chan->lock);
        if (chan->used == 0)
        {
            hal_memset(chan, 0, sizeof(struct sunxi_dma_chan));
        }
    }

    for (i = START_CHAN_OFFSET + SUNXI_DMA_CHAN_START; i < START_CHAN_OFFSET + NR_MAX_CHAN; i++)
    {
        hal_dma_irq_set(i, 0);
        hal_dma_irq_status(i, 1);
        /*set non-secure*/
        secure = hal_readl(DMA_SECURE);
        if ((secure & (1 << i)) == 0)
            hal_writel(secure | (1 << i), DMA_SECURE);
    }
    /* disable auto gating */
    hal_writel(DMA_MCLK_GATE | DMA_COMMON_GATE | DMA_CHAN_GATE, DMA_GATE);
    sunxi_dma_clk_init(true);

#ifdef SUNXI_IRQ_CONTROL
    hal_writel(SUNXI_CPU_IRQ_CONTROL_VAL, SUNXI_CPU_IRQ_CONTROL_OFFSET);
    hal_writel(SUNXI_MCU_IRQ_CONTROL_VAL, SUNXI_MCU_IRQ_CONTROL_OFFSET);
#endif
    /*request dma irq*/
    if (!(irq_inited & 0x1)) {
        if (hal_request_irq(DMA_IRQ_NUM, sunxi_dma_irq_handle, "dma", NULL) < 0)
            DMA_ERR("[dma] request irq error\n");
        else
            irq_inited |= 0x1;
    }
    hal_enable_irq(DMA_IRQ_NUM);
}

/* only need to be executed once */
void hal_dma_init(void)
{
    hal_dma_resource_get();
}



