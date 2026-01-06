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
#include <string.h>

#include "drv_gpio.h"
#include "drv_sdio.h"
#include "cache.h"

#include "hal_sdmmc.h"
#include "hal_clk.h"
#include "hal_reset.h"

#define DBG_TAG  "SMHC"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef RT_USING_SDIO

#ifdef RT_USING_DFS
#include <dfs_fs.h>
#endif

#ifdef SMHC_DEBUG
#define SDC_LOGD(...)    LOG_D(...)
#define SDC_LOGW(...)    LOG_W(...)
#define SDC_LOGE(...)    LOG_E(...)
#else
#define SDC_LOGD(...)   do { }while(0)
#define SDC_LOGW(...)   do { }while(0)
#define SDC_LOGE(...)   do { }while(0)
#endif

struct scatterlist {
    void *buffer;
    uint32_t len;
};

typedef struct {
    rt_uint32_t config;

#define SDXC_IDMAC_DES0_DIC             BIT(1) /* disable interrupt on completion */
#define SDXC_IDMAC_DES0_LD              BIT(2) /* 1-this data buffer is the last buffer */
#define SDXC_IDMAC_DES0_FD              BIT(3) /* 1-data buffer is the first buffer, 0-data buffer contained in the next descriptor is the first data buffer */
#define SDXC_IDMAC_DES0_CH              BIT(4) /* 1-the 2nd address in the descriptor is the next descriptor address */
#define SDXC_IDMAC_DES0_ER              BIT(5) /* 1-last descriptor flag when using dual data buffer in descriptor */
#define SDXC_IDMAC_DES0_CES             BIT(30) /* transfer error flag */
#define SDXC_IDMAC_DES0_OWN             BIT(31) /* des owner:1-idma owns it, 0-host owns it */

    rt_uint32_t data_buf1_sz            : 16,
                data_buf2_sz            : 16;

    rt_uint32_t buf_addr_ptr1;
    rt_uint32_t buf_addr_ptr2;

}smc_idma_des;

struct sunxi_smhc
{
    rt_uint32_t             reg_base;
    rt_uint32_t             irq_num;
    rt_uint8_t              sdc_id;

    rt_uint8_t              usedma;
    rt_uint16_t             power_on;
    rt_uint32_t             sdio_irq_mask;
    struct __mci_ctrl_regs  regs_back;

    struct gpio_cfg         gpio_clk;
    struct gpio_cfg         gpio_cmd;
    struct gpio_cfg         gpio_d0;
    struct gpio_cfg         gpio_d1;
    struct gpio_cfg         gpio_d2;
    struct gpio_cfg         gpio_d3;

    rt_uint32_t             int_err;       /* for Interrupt Controller */
    rt_uint32_t             int_use;       /* Control */
    rt_uint32_t             int_sum;       /* interrupt summary */
    rt_uint16_t             trans_done;
    rt_uint16_t             dma_done;

    volatile rt_uint32_t    smc_cmd;
    rt_uint32_t             wait;

    smc_idma_des            *idma_des;
    smc_idma_des            *dma_hdle;
    rt_int8_t               *align_dma_buf;

    rt_spinlock_t           sdmmc_lock;
    rt_sem_t                cmp_sem;
    rt_mutex_t              req_mutex;

    struct rt_mmcsd_req     *req;
    struct scatterlist      sg;
};

static void __mci_irq_handler(int irq, void *param)
{
    uint32_t raw_int;
    uint32_t msk_int;
    uint32_t idma_inte;
    uint32_t idma_int;
    struct sunxi_smhc *smhc = (struct sunxi_smhc *)param;

    idma_int = readl(smhc->reg_base + SDXC_REG_IDST);
    idma_inte = readl(smhc->reg_base + SDXC_REG_IDIE);
    raw_int = readl(smhc->reg_base + SDXC_REG_RINTR);
    msk_int = readl(smhc->reg_base + SDXC_REG_MISTA);

    if (!msk_int && !idma_int) {
        SDC_LOGE("sdc nop irq: ri:%08lx mi:%08lx ie:%08lx idi:%08lx",
                 HAL_PR_SZ_L(raw_int), HAL_PR_SZ_L(msk_int), HAL_PR_SZ_L(idma_inte), HAL_PR_SZ_L(idma_int));
        return;
    }

    smhc->int_sum = raw_int;
    SDC_LOGD("sdc %d ri:%02x(%02x), mi:%x, ie:%x, idi:%x", __LINE__,
        (int)raw_int, (int)smhc->int_sum, (int)msk_int, (int)idma_inte, (int)idma_int);

    (void)idma_inte;

    if(smhc->wait == SDC_WAIT_NONE) {
        SDC_LOGE("%s nothing to complete, ri:%08lx, mi:%08lx",
                __func__, HAL_PR_SZ_L(raw_int), HAL_PR_SZ_L(msk_int));
        goto irq_out;
    }

    if ((raw_int & SDXC_IntErrBit) || (idma_int & SDXC_IDMA_ERR)) {
        smhc->int_err = raw_int & SDXC_IntErrBit;
        smhc->wait = SDC_WAIT_FINALIZE;
        SDC_LOGE("%s,%d raw_int:%lx err!", __func__, __LINE__, HAL_PR_SZ_L(raw_int));
        goto irq_out;
    }
    if (raw_int & SDXC_HardWLocked) {
        LOG_E("command hardware lock");
    }

    if (idma_int & (SDXC_IDMACTransmitInt | SDXC_IDMACReceiveInt)) {
        smhc->dma_done = 1;
        writel(idma_int, smhc->reg_base + SDXC_REG_IDST);
    }
    if (msk_int & (SDXC_AutoCMDDone|SDXC_DataOver|SDXC_CmdDone|SDXC_VolChgDone))
        smhc->trans_done = 1;
    if ((smhc->trans_done && \
        (smhc->wait == SDC_WAIT_AUTOCMD_DONE || smhc->wait == SDC_WAIT_DATA_OVER
         || smhc->wait == SDC_WAIT_CMD_DONE || smhc->wait == SDC_WAIT_SWITCH1V8))
         || (smhc->trans_done && smhc->dma_done && (smhc->wait & SDC_WAIT_IDMA_DONE))) {
        smhc->wait = SDC_WAIT_FINALIZE;
    }

irq_out:
    writel(msk_int & (~SDXC_SDIOInt), smhc->reg_base + SDXC_REG_RINTR);
    writel(idma_int, smhc->reg_base + SDXC_REG_IDST);

    if (smhc->wait == SDC_WAIT_FINALIZE) {
        writel(smhc->sdio_irq_mask, smhc->reg_base + SDXC_REG_IMASK);

        dsb(0xf);

        SDC_LOGD("SDC irq post, trans:%d, dma:%d", (int)smhc->trans_done, (int)smhc->dma_done);
        rt_sem_release(smhc->cmp_sem);
    }
}

static void __mci_clk_enable_MClock(struct sunxi_smhc *smhc, int enable)
{
    if (smhc->sdc_id == 0) {
        if(enable) hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC0));
        else hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC0));
    } else if (smhc->sdc_id == 1) {
        if(enable) hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC1));
        else hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC1));
    } else if (smhc->sdc_id == 2) {
        if(enable) hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC2));
        else hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC2));
    }
}

static __inline void __mci_sel_access_mode(struct sunxi_smhc *smhc, uint32_t access_mode)
{
    writel((readl(smhc->reg_base + SDXC_REG_GCTRL) & (~SDXC_ACCESS_BY_AHB)) | access_mode, smhc->reg_base + SDXC_REG_GCTRL);
}

static int32_t __mci_reset(struct sunxi_smhc *smhc)
{
    uint32_t value;
    int32_t timeout = 1000;

    value = readl(smhc->reg_base + SDXC_REG_GCTRL) | SDXC_HWReset;
    writel(value, smhc->reg_base + SDXC_REG_GCTRL);
    while ((readl(smhc->reg_base + SDXC_REG_GCTRL) & SDXC_SoftReset))
    {
        hal_msleep(1);
        if(timeout-- < 0) {
            LOG_E("SDC reset time out");
            return -1;
        }
    }
    LOG_D("%s,%d SDC reset finish ", __func__, __LINE__);

    return 0;
}

static void __mci_clk_prepare_enable(struct sunxi_smhc *smhc)
{
    if (smhc->sdc_id == 0) {
        hal_reset_control_deassert(hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_MMC0));
        hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_MMC0));
        hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC0));
    } else if (smhc->sdc_id == 1) {
        hal_reset_control_deassert(hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_MMC1));
        hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_MMC1));
        hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC1));
    } else if (smhc->sdc_id == 2) {
        hal_reset_control_deassert(hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_MMC2));
        hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_MMC2));
        hal_clock_enable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC2));
    }
}

static void __mci_clk_disable_unprepare(struct sunxi_smhc *smhc)
{
    if (smhc->sdc_id == 0) {
        hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC0));
        hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_MMC0));
        hal_reset_control_assert(hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_MMC0));
    } else if (smhc->sdc_id == 1) {
        hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC1));
        hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_MMC1));
        hal_reset_control_assert(hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_MMC1));
    } else if (smhc->sdc_id == 2) {
        hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_MMC2));
        hal_clock_disable(hal_clock_get(HAL_SUNXI_CCU, CLK_BUS_MMC2));
        hal_reset_control_assert(hal_reset_control_get(HAL_SUNXI_RESET, RST_BUS_MMC2));
    }
}

static void __mci_debounce_onoff(struct sunxi_smhc *smhc, uint32_t onoff)
{
    uint32_t rval = readl(smhc->reg_base + SDXC_REG_GCTRL);

    rval &= ~SDXC_DebounceEnb;
    if (onoff)
        rval |= SDXC_DebounceEnb;
    writel(rval, smhc->reg_base + SDXC_REG_GCTRL);
    SDC_LOGD("%s,%d,%ld",__FUNCTION__,__LINE__, HAL_PR_SZ_L(onoff));
}

static int32_t __mci_check_busy_over(struct sunxi_smhc *smhc)
{
    uint32_t i;
    for(i = 0; i < SDC_BUSY_WAIT_LOOP; i++){
        if(!(readl(smhc->reg_base + SDXC_REG_STAS) & SDXC_CardBusy)){
            return 0;
        }
        if(i > (SDC_BUSY_WAIT_LOOP / 8)) {
            rt_thread_mdelay(10);
            if(i % 8)
                SDC_LOGW("Waiting reg %x bitmap %x clear v %x，i %x",\
                        HAL_PR_SZ(HAL_PT_TO_U(smhc->reg_base)) + SDXC_REG_STAS,\
                         (unsigned int)SDXC_CardBusy,\
                         (unsigned int)readl(smhc->reg_base + SDXC_CardBusy),\
                         (unsigned int)i);
        }
        SDC_LOGD("Waiting reg %x bitmap %x clear v %x，i %x",\
                         HAL_PR_SZ(HAL_PT_TO_U(smhc->reg_base)) + SDXC_REG_STAS,\
                         (unsigned int)SDXC_CardBusy,\
                         (unsigned int)readl(smhc->reg_base + SDXC_CardBusy),\
                         (unsigned int)i);
    }
    LOG_E("Wait busy timeout %x，%x", (unsigned int)readl(smhc->reg_base + SDXC_REG_STAS), (unsigned int)i);

    return -1;
}

int32_t __mci_check_bit_clear(struct sunxi_smhc *smhc, uint32_t reg_offset, uint32_t bit_map)
{
    uint32_t i;
    for(i = 0; i< SDC_BUSY_WAIT_LOOP; i++){
        if(!(readl(smhc->reg_base + reg_offset) & bit_map)){
            SDC_LOGD("Wait reg %x bitmap %x clear v %lx，i %x ok",\
                     (smhc->reg_base) + reg_offset,\
                     (unsigned int)bit_map,\
                     HAL_PR_SZ_L(readl(smhc->reg_base + reg_offset)),\
                     (unsigned int)i);
            return 0;
        }
        if(i > (SDC_BUSY_WAIT_LOOP / 8)) {
            rt_thread_mdelay(10);
            if(i % 8)
                SDC_LOGW("Waiting reg %x bitmap %x clear v %lx，i %x",\
                         (smhc->reg_base) + reg_offset,\
                         (unsigned int)bit_map,\
                         HAL_PR_SZ_L(readl(smhc->reg_base + reg_offset)),\
                         (unsigned int)i);
        }
    }
    LOG_E("Wait reg %x bitmap %x clear timeout v %lx，i %x",\
             (smhc->reg_base) + reg_offset,\
             (unsigned int) bit_map,\
             HAL_PR_SZ_L(readl(smhc->reg_base + reg_offset)),\
             (unsigned int)i);
    return -1;
}

#define SDXC_DAT_DRV_PH_SEL         (1U<<17)
#define SDXC_CMD_DRV_PH_SEL         (1U<<16)
#define SDXC_STIMING_CMD_PH_MASK        (0x00000030)
#define SDXC_STIMING_DAT_PH_MASK        (0x00000300)
#define SDXC_STIMING_CMD_PH_SHIFT           (4)
#define SDXC_STIMING_DAT_PH_SHIFT           (8)
static void sunxi_mmc_set_clk_dly(struct sunxi_smhc *smhc, int clk)
{
    u32 rval = 0;
    u32 cmd_drv_ph = 0;
    u32 dat_drv_ph = 0;
    u32 sam_ph_dat = 0;
    u32 sam_ph_cmd = 0;

    if (clk <= 400 * 1000) {
        cmd_drv_ph = 1;
        dat_drv_ph = 0;
        sam_ph_dat = 0;
        sam_ph_cmd = 0;
    } else if (clk <= 26 * 1000 * 1000) {
        cmd_drv_ph = 1;
        dat_drv_ph = 0;
        sam_ph_dat = 0;
        sam_ph_cmd = 0;
    } else if (clk <= 52 * 1000 * 1000) {
        cmd_drv_ph = 1;
        dat_drv_ph = 1;
        sam_ph_dat = 1;
        sam_ph_cmd = 1;
    } else if (clk <= 104 * 1000 * 1000) {
        cmd_drv_ph = 1;
        dat_drv_ph = 0;
        sam_ph_dat = 0;
        sam_ph_cmd = 0;
    } else if (clk <= 208 * 1000 * 1000) {
        cmd_drv_ph = 1;
        dat_drv_ph = 0;
        sam_ph_dat = 0;
        sam_ph_cmd = 0;
    } else {
        LOG_E("clk %d is out of range", clk);
        return;
    }

    SDC_LOGD("raw DRV_DL=0x%x NTSR=0x%x", readl(smhc->reg_base + SDXC_REG_DRV_DL), readl(smhc->reg_base + SDXC_REG_NTSR));

    rval = readl(smhc->reg_base + SDXC_REG_DRV_DL);
    if (cmd_drv_ph)
        rval |= SDXC_CMD_DRV_PH_SEL;    /* 180 phase */
    else
        rval &= ~SDXC_CMD_DRV_PH_SEL;   /* 90 phase */

    if (dat_drv_ph)
        rval |= SDXC_DAT_DRV_PH_SEL;    /* 180 phase */
    else
        rval &= ~SDXC_DAT_DRV_PH_SEL;   /* 90 phase */

    __mci_clk_enable_MClock(smhc, RT_FALSE);
    writel(rval, smhc->reg_base + SDXC_REG_DRV_DL);
    __mci_clk_enable_MClock(smhc, RT_TRUE);

    rval = readl(smhc->reg_base + SDXC_REG_NTSR);
    rval &= ~SDXC_STIMING_DAT_PH_MASK;
    rval |= (sam_ph_dat << SDXC_STIMING_DAT_PH_SHIFT) &
        SDXC_STIMING_DAT_PH_MASK;
    writel(rval, smhc->reg_base + SDXC_REG_NTSR);

    rval = readl(smhc->reg_base + SDXC_REG_NTSR);
    rval &= ~SDXC_STIMING_CMD_PH_MASK;
    rval |= (sam_ph_cmd << SDXC_STIMING_CMD_PH_SHIFT) &
        SDXC_STIMING_CMD_PH_MASK;
    writel(rval, smhc->reg_base + SDXC_REG_NTSR);

    SDC_LOGD("after set DRV_DL=0x%x NTSR=0x%x", readl(smhc->reg_base + SDXC_REG_DRV_DL), readl(smhc->reg_base + SDXC_REG_NTSR));
}

static void __mci_regs_save(struct sunxi_smhc *smhc)
{
    struct __mci_ctrl_regs *bak_regs = &smhc->regs_back;

    bak_regs->gctrl = readl(smhc->reg_base + SDXC_REG_GCTRL);
    bak_regs->clkc = readl(smhc->reg_base + SDXC_REG_CLKCR);
    bak_regs->timeout = readl(smhc->reg_base + SDXC_REG_TMOUT);
    bak_regs->buswid = readl(smhc->reg_base + SDXC_REG_WIDTH);
    bak_regs->waterlvl = readl(smhc->reg_base + SDXC_REG_FTRGL);
    bak_regs->funcsel = readl(smhc->reg_base + SDXC_REG_FUNS);
    bak_regs->debugc = readl(smhc->reg_base + SDXC_REG_DBGC);
    bak_regs->idmacc = readl(smhc->reg_base + SDXC_REG_DMAC);
    bak_regs->dlba = readl(smhc->reg_base + SDXC_REG_DLBA);
    bak_regs->imask = readl(smhc->reg_base + SDXC_REG_IMASK);
    bak_regs->drv_dl = readl(smhc->reg_base + SDXC_REG_DRV_DL);
    bak_regs->samp_dl = readl(smhc->reg_base + SDXC_REG_SAMP_DL);
    bak_regs->ds_dl = readl(smhc->reg_base + SDXC_REG_DS_DL);
    bak_regs->sd_ntsr = readl(smhc->reg_base + SDXC_REG_NTSR);
    bak_regs->edsd = readl(smhc->reg_base + SDXC_REG_DSBD);
    bak_regs->csdc = readl(smhc->reg_base + SDXC_REG_CSDC);

}

static void __mci_regs_restore(struct sunxi_smhc *smhc)
{
    struct __mci_ctrl_regs* bak_regs = &smhc->regs_back;

    writel(bak_regs->gctrl, smhc->reg_base + SDXC_REG_GCTRL);
    writel(bak_regs->clkc, smhc->reg_base + SDXC_REG_CLKCR);
    writel(bak_regs->timeout, smhc->reg_base + SDXC_REG_TMOUT);
    writel(bak_regs->buswid, smhc->reg_base + SDXC_REG_WIDTH);
    writel(bak_regs->waterlvl, smhc->reg_base + SDXC_REG_FTRGL);

    writel(bak_regs->funcsel, smhc->reg_base + SDXC_REG_FUNS);
    writel(bak_regs->debugc, smhc->reg_base + SDXC_REG_DBGC);

    writel(bak_regs->idmacc, smhc->reg_base + SDXC_REG_DMAC);
    writel(bak_regs->dlba, smhc->reg_base + SDXC_REG_DLBA);
    writel(bak_regs->imask, smhc->reg_base + SDXC_REG_IMASK);
    __mci_clk_enable_MClock(smhc, RT_FALSE);
    writel(bak_regs->drv_dl, smhc->reg_base + SDXC_REG_DRV_DL);
    __mci_clk_enable_MClock(smhc, RT_TRUE);
    writel(bak_regs->samp_dl, smhc->reg_base + SDXC_REG_SAMP_DL);
    writel(bak_regs->ds_dl, smhc->reg_base + SDXC_REG_DS_DL);
    writel(bak_regs->sd_ntsr, smhc->reg_base + SDXC_REG_NTSR);
    writel(bak_regs->edsd, smhc->reg_base + SDXC_REG_DSBD);
    writel(bak_regs->csdc, smhc->reg_base + SDXC_REG_CSDC);
}

static int32_t __mci_program_clk(struct sunxi_smhc *smhc)
{
    uint32_t value;
    int32_t ret = 0;
    int32_t timeout = 1000;

    /* disable command done interrupt */
    writel((readl(smhc->reg_base + SDXC_REG_IMASK) & (~SDXC_CmdDone)) | smhc->sdio_irq_mask, smhc->reg_base + SDXC_REG_IMASK);

    writel(readl(smhc->reg_base + SDXC_REG_CLKCR) | SDXC_Mask_Data0, smhc->reg_base + SDXC_REG_CLKCR);

    value = SDXC_Start | SDXC_UPCLKOnly | SDXC_WaitPreOver;
    writel(value, smhc->reg_base + SDXC_REG_CMDR);

    value = readl(smhc->reg_base + SDXC_REG_CMDR);
    while(value & SDXC_Start)
    {
        rt_thread_mdelay(1);
        if(timeout-- < 0)
        {
            LOG_E("%s,%d SDC change clock time out", __func__, __LINE__);
            ret = -1;
            break;
        }
        value = readl(smhc->reg_base + SDXC_REG_CMDR);
    }

    /* clear command done flag */
    value = readl(smhc->reg_base + SDXC_REG_RINTR);
    writel(value, smhc->reg_base + SDXC_REG_RINTR);

    writel(readl(smhc->reg_base + SDXC_REG_CLKCR) & ~SDXC_Mask_Data0, smhc->reg_base + SDXC_REG_CLKCR);

    /* enable command done interrupt */
    writel((readl(smhc->reg_base + SDXC_REG_IMASK) | SDXC_CmdDone) | smhc->sdio_irq_mask, smhc->reg_base + SDXC_REG_IMASK);

    return ret;
}

static int32_t __mci_update_clock(struct sunxi_smhc *smhc, uint32_t cclk)
{
    uint32_t sclk;
    uint32_t div;
    uint32_t rval;
    uint32_t src = 0;
    uint32_t m, n;

    if (cclk > DEFINE_SYS_CRYSTAL/2) {
        src = 1;
        sclk = DEFINE_SYS_DEVCLK;
    } else {
        src = 0;
        sclk = DEFINE_SYS_CRYSTAL;
    }
    cclk = cclk * 2; /* 2x MODE clock configure */
    div = (2 * sclk + cclk) / (2 * cclk);
    div = div == 0 ? 1 : div;
    if (div > 128) {
        n = 3;
        m = 16;
        LOG_E("source clk is too high!");
    } else if (div > 64) {
        n = 3;
        m = div >> 3;
    } else if (div > 32) {
        n = 2;
        m = div >> 2;
    } else if (div > 16) {
        n = 1;
        m = div >> 1;
    } else {
        n = 0;
        m = div;
    }
    m = m - 1;

    if (smhc->sdc_id == 0) {
        rval = hal_readl(SDC_CCM_SDC0_SCLK_CTRL);
        rval &= ~(1U << 31);
        writel(rval, SDC_CCM_SDC0_SCLK_CTRL);
        dsb(0xf);
        rval = (src << 24) | (n << SDC_CCM_SClk_DIV_N_SHIFT) | (m << SDC_CCM_SClk_DIV_M_SHIFT );

        writel(rval,SDC_CCM_SDC0_SCLK_CTRL);
        dsb(0xf);
        rval =hal_readl(SDC_CCM_SDC0_SCLK_CTRL);
        rval |= (1U<<31);
        writel(rval,SDC_CCM_SDC0_SCLK_CTRL);
        dsb(0xf);

        LOG_D("SDC clock=%ld Hz,src:%x, n:%d, m:%d",
            HAL_PR_SZ_L((src ? DEFINE_SYS_DEVCLK:DEFINE_SYS_CRYSTAL)/(1<<n)/(m+1)/2),
            (int)src, (int)n, (int)m);
     } else if (smhc->sdc_id == 1) {
        rval = hal_readl(SDC_CCM_SDC1_SCLK_CTRL);
        rval &= ~(1U<<31);
        hal_writel(rval,SDC_CCM_SDC1_SCLK_CTRL);

        dsb(0xf);
        rval = (src << 24) | (n << SDC_CCM_SClk_DIV_N_SHIFT) | (m << SDC_CCM_SClk_DIV_M_SHIFT);
        hal_writel(rval,SDC_CCM_SDC1_SCLK_CTRL);

        dsb(0xf);
        rval = hal_readl(SDC_CCM_SDC1_SCLK_CTRL);
        rval |= (1U<<31);
        hal_writel(rval,SDC_CCM_SDC1_SCLK_CTRL);
        dsb(0xf);
        LOG_D("SDC clock=%ld Hz,src:%x, n:%d, m:%d",
            HAL_PR_SZ_L((src?DEFINE_SYS_DEVCLK:DEFINE_SYS_CRYSTAL)/(1<<n)/(m+1)/2),
            (int)src, (int)n, (int)m);
    } else if (smhc->sdc_id == 2) {
        rval = readl(SDC_CCM_SDC2_SCLK_CTRL);
        rval &= ~(1U<<31);
        writel(rval,SDC_CCM_SDC2_SCLK_CTRL);
        dsb(0xf);
        rval = (src << 24) | (n << SDC_CCM_SClk_DIV_N_SHIFT) | (m << SDC_CCM_SClk_DIV_M_SHIFT );

        writel(rval,SDC_CCM_SDC2_SCLK_CTRL);
        dsb(0xf);
        rval = readl(SDC_CCM_SDC2_SCLK_CTRL);
        rval |= (1U << 31);
        writel(rval, SDC_CCM_SDC2_SCLK_CTRL);
        dsb(0xf);

        LOG_D("SDC clock=%ld Hz,src:%x, n:%d, m:%d",
            HAL_PR_SZ_L((src ? DEFINE_SYS_DEVCLK:DEFINE_SYS_CRYSTAL)/(1<<n)/(m+1)/2),
            (int)src, (int)n, (int)m);
     }

     /* clear internal divider */
      rval = readl(smhc->reg_base + SDXC_REG_CLKCR) & (~0xff);
      writel(rval, smhc->reg_base + SDXC_REG_CLKCR);

      sunxi_mmc_set_clk_dly(smhc, cclk / 2);

      return cclk;
}

static int32_t __mci_update_clk(struct sunxi_smhc *smhc)
{
    uint32_t value;
    int32_t ret = 0;
    int32_t timeout = 1000;

    writel(readl(smhc->reg_base + SDXC_REG_CLKCR) | SDXC_Mask_Data0, smhc->reg_base + SDXC_REG_CLKCR);

    value = SDXC_Start | SDXC_UPCLKOnly | SDXC_WaitPreOver;
    writel(value, smhc->reg_base + SDXC_REG_CMDR);

    value = readl(smhc->reg_base + SDXC_REG_CMDR);
    while(value & SDXC_Start)
    {
        rt_thread_mdelay(1);
        if(timeout-- < 0)
        {
            LOG_E("%s,%d SDC change clock time out", __func__, __LINE__);
            ret = -1;
            break;
        }
        value = readl(smhc->reg_base + SDXC_REG_CMDR);
    }

    /* clear command done flag */
    value = readl(smhc->reg_base + SDXC_REG_RINTR);
    writel(value, smhc->reg_base + SDXC_REG_RINTR);

    writel(readl(smhc->reg_base + SDXC_REG_CLKCR) & ~SDXC_Mask_Data0, smhc->reg_base + SDXC_REG_CLKCR);

    return ret;
}

int32_t HAL_SDC_Clk_PWR_Opt(struct sunxi_smhc *smhc, uint32_t oclk_en, uint32_t pwr_save)
{
    uint32_t rval;

    if (smhc->power_on)
        __mci_update_clk(smhc);

    rval = readl(smhc->reg_base + SDXC_REG_CLKCR);

    rval &= ~(SDXC_CardClkOn | SDXC_LowPowerOn);
    if (oclk_en)
        rval |= SDXC_CardClkOn;
    if (pwr_save)
        rval |= SDXC_LowPowerOn;
    writel(rval, smhc->reg_base + SDXC_REG_CLKCR);

    return 0;
}

void HAL_SDC_Set_BusWidth(struct sunxi_smhc *smhc, uint32_t width)
{
    if (!smhc) {
        LOG_E("%s,%d err", __func__, __LINE__);
        return;
    }

    switch (width)
    {
        case MMC_BUS_WIDTH_1:
            writel(SDXC_WIDTH1, smhc->reg_base + SDXC_REG_WIDTH);
            break;
        case MMC_BUS_WIDTH_4:
            writel(SDXC_WIDTH4, smhc->reg_base + SDXC_REG_WIDTH);
            break;
        case MMC_BUS_WIDTH_8:
            writel(SDXC_WIDTH8, smhc->reg_base + SDXC_REG_WIDTH);
            break;
        default:
            return;
    }
}

int32_t HAL_SDC_PowerOn(struct sunxi_smhc *smhc)
{
    uint32_t rval;

    if (!smhc) {
        LOG_E("%s,%d err", __func__, __LINE__);
        return -1;
    }

    SDC_LOGD("MMC Driver init host");

    __mci_clk_prepare_enable(smhc);

    /* delay 1ms ? */
    __mci_update_clock(smhc, 400000);

    /* reset controller*/
    rval = readl(smhc->reg_base + SDXC_REG_GCTRL) | SDXC_HWReset | SDXC_INTEnb | SDXC_AccessDoneDirect;
    writel(rval, smhc->reg_base + SDXC_REG_GCTRL);
    __mci_check_bit_clear(smhc, SDXC_REG_GCTRL, SDXC_HWReset);
    SDC_LOGD("%s,%d",__FUNCTION__,__LINE__);

    writel(0xffffffff, smhc->reg_base + SDXC_REG_RINTR);

    writel(smhc->sdio_irq_mask, smhc->reg_base + SDXC_REG_IMASK);

#define SDMC_DATA_TIMEOUT  0x0ffffffU
#define SDMC_RESP_TIMEOUT  0xffU
    /* Set Data & Response Timeout Value */
    writel((SDMC_DATA_TIMEOUT << 8) | SDMC_RESP_TIMEOUT, smhc->reg_base + SDXC_REG_TMOUT);
#undef SDMC_RESP_TIMEOUT
#undef SDMC_DATA_TIMEOUT

    HAL_SDC_Set_BusWidth(smhc, MMC_BUS_WIDTH_1);

    rt_hw_interrupt_umask(smhc->irq_num);
    smhc->power_on = 1;

    return 0;
}

int hal_mmc_power_up(struct sunxi_smhc *smhc)
{
    int32_t err = -1;

    HAL_SDC_Clk_PWR_Opt(smhc, 1, 0);

    rt_thread_mdelay(40);
    HAL_SDC_PowerOn(smhc);

    rt_thread_mdelay(5);
    HAL_SDC_Clk_PWR_Opt(smhc, 0, 0);

    return err;
}

int32_t HAL_SDC_Update_Clk(struct sunxi_smhc *smhc, uint32_t clk)
{
    uint32_t rval;

    if (clk > SDC_MAX_FREQ_SDR_1V8) {
        clk = SDC_MAX_FREQ_SDR_1V8;
        LOG_D("support max freq is 150M");
    }

    /* Disable Clock */
    rval = readl(smhc->reg_base + SDXC_REG_CLKCR) & (~SDXC_CardClkOn) & (~SDXC_LowPowerOn);
    writel(rval, smhc->reg_base + SDXC_REG_CLKCR);
    if(__mci_program_clk(smhc)) {
        LOG_E("Clock Program Failed 0!!");
        return -1;
    }

    __mci_update_clock(smhc, clk);

    /* Re-enable Clock */
    rval = readl(smhc->reg_base + SDXC_REG_CLKCR) | (SDXC_CardClkOn);// | SDXC_LowPowerOn;
    writel(rval, smhc->reg_base + SDXC_REG_CLKCR);
    if (__mci_program_clk(smhc)) {
        LOG_E("Clock Program Failed 1!!");
        return -1;
    }

    return 0;
}

static void __mci_trans_by_ahb(struct sunxi_smhc *smhc, struct rt_mmcsd_data *data)
{
    uint32_t j;
    uint32_t *buf_temp; /* Cortex-M3/4 can access data with unaligned address */
    int32_t timeout = 1000;

    buf_temp = smhc->sg.buffer;
    if (data->flags & DATA_DIR_READ) {
        for (j = 0; j < (smhc->sg.len >> 2); j++) { /* sg.len should be multiply of 4 */
            timeout = 1000;
            while((readl(smhc->reg_base + SDXC_REG_STAS) & SDXC_FIFOEmpty))
            {
                hal_msleep(1);
                if(timeout-- <= 0) {
                    LOG_E("read from fifo timeout1");
                    return;
                }
            }
            buf_temp[j] = readl(smhc->reg_base + SDXC_REG_FIFO);
        }

        if((smhc->sg.len) & 0x3) {
            timeout = 1000;
            while((readl(smhc->reg_base + SDXC_REG_STAS) & SDXC_FIFOEmpty))
            {
                hal_msleep(1);
                if(timeout-- <= 0) {
                    LOG_E("read from fifo timeout2");
                    return;
                }
            }

            if(((smhc->sg.len) & 0x3) == 2){
                    buf_temp[j] = (uint16_t)readl(smhc->reg_base + SDXC_REG_FIFO);
            }else if(((smhc->sg.len) & 0x3) == 1){
                    buf_temp[j] = (uint8_t)readl(smhc->reg_base + SDXC_REG_FIFO);
            }else
                LOG_W("smhc read len %d error", smhc->sg.len);
        }
    } else if (data->flags & DATA_DIR_WRITE) {
        for (j = 0; j < (smhc->sg.len >> 2); j++) { /* sg[i].len should be multiply of 4 */
            timeout = 1000;
            while((readl(smhc->reg_base + SDXC_REG_STAS) & SDXC_FIFOFull))
            {
                hal_msleep(1);
                if(timeout-- <= 0) {
                    LOG_E("read from fifo timeout2");
                    return;
                }
            }

            writel(buf_temp[j], smhc->reg_base + SDXC_REG_FIFO);
        }

        if((smhc->sg.len) & 0x3) {
            timeout = 1000;
            while((readl(smhc->reg_base + SDXC_REG_STAS) & SDXC_FIFOFull))
            {
                hal_msleep(1);
                if(timeout-- <= 0) {
                    LOG_E("read from fifo timeout2");
                    return;
                }
            }

            if(((smhc->sg.len) & 0x3) == 2){
                    writel((uint16_t)buf_temp[j], smhc->reg_base + SDXC_REG_FIFO);
            }else if(((smhc->sg.len) & 0x3) == 1){
                    writel((uint8_t)buf_temp[j], smhc->reg_base + SDXC_REG_FIFO);
            }else
                LOG_W("smhc write len %d error", smhc->sg.len);
        }

    } else {
        LOG_W("illigle data request");
        return;
    }
    dsb(0xf);
}

static smc_idma_des *__mci_alloc_idma_des(struct sunxi_smhc *smhc)
{
    smc_idma_des *pdes = smhc->idma_des;
    uint32_t des_idx = 0;
    uint32_t buff_frag_num = 0;
    uint32_t remain;
    uint32_t j;
    uint32_t config;

    /* init IDMA Descriptor, two mode: 1-fixed skip length, 2-chain mode */
    buff_frag_num = smhc->sg.len >> SDXC_DES_NUM_SHIFT; /* num = len/8192 = len>>13  */
    remain = smhc->sg.len & (SDXC_DES_BUFFER_MAX_LEN - 1);
    if (remain) {
        buff_frag_num++;
    } else {
        remain = SDXC_DES_BUFFER_MAX_LEN;
    }

    for (j = 0; j < buff_frag_num; j++, des_idx++) {
        rt_memset((void *)&pdes[des_idx], 0, sizeof(smc_idma_des));
        config = SDXC_IDMAC_DES0_CH | SDXC_IDMAC_DES0_OWN | SDXC_IDMAC_DES0_DIC;
        if (buff_frag_num > 1 && j != buff_frag_num - 1) {
            pdes[des_idx].data_buf1_sz = SDXC_DES_BUFFER_MAX_LEN;
        } else {
            pdes[des_idx].data_buf1_sz = remain;
        }

        pdes[des_idx].buf_addr_ptr1 = SDXC_IDMAC_DES_ADDR(HAL_PT_TO_U(smhc->sg.buffer) + j * SDXC_DES_BUFFER_MAX_LEN);
        if (j == 0) {
            config |= SDXC_IDMAC_DES0_FD;
        }

        if (j == buff_frag_num - 1) {
            config &= ~SDXC_IDMAC_DES0_DIC;
            config |= SDXC_IDMAC_DES0_LD | SDXC_IDMAC_DES0_ER;
            pdes[des_idx].buf_addr_ptr2 = 0;
        } else {
            pdes[des_idx].buf_addr_ptr2 = SDXC_IDMAC_DES_ADDR(HAL_PT_TO_U(&pdes[des_idx + 1]));
        }
        pdes[des_idx].config = config;
        SDC_LOGD("sg %lu, frag %lu, remain %lu, des[%lu](%p): [0]:%lx, [1]:%lx, [2]:%lx, [3]:%lx",
                 0, HAL_PR_SZ_L(j), HAL_PR_SZ_L(remain), HAL_PR_SZ_L(des_idx), &pdes[des_idx],
                 HAL_PR_SZ_L(((uint32_t *)&pdes[des_idx])[0]), HAL_PR_SZ_L(((uint32_t *)&pdes[des_idx])[1]),
                 HAL_PR_SZ_L(((uint32_t *)&pdes[des_idx])[2]), HAL_PR_SZ_L(((uint32_t *)&pdes[des_idx])[3]));
    }

    rt_hw_cpu_dcache_clean_and_invalidate(pdes, HAL_ALIGN(SDXC_MAX_DES_NUM * sizeof(smc_idma_des), CACHELINE_LEN));
    dsb(0xf);

    return pdes;
}

static smc_idma_des *__mci_prepare_dma(struct sunxi_smhc *smhc)
{
    uint32_t temp;
    smc_idma_des *pdes = NULL;

    /* creat descriptor list, two mode: 1-fixed skip length, 2-chain mode */
    pdes = __mci_alloc_idma_des(smhc);
    if (NULL == pdes) {
        LOG_W("alloc IDMA descriptor failed");
        return NULL;
    }

    temp = readl(smhc->reg_base + SDXC_REG_GCTRL);
    temp |= SDXC_DMAEnb;
    writel(temp, smhc->reg_base + SDXC_REG_GCTRL);
    temp |= (SDXC_DMAReset | SDXC_FIFOReset);
    writel(temp, smhc->reg_base + SDXC_REG_GCTRL);
    __mci_check_bit_clear(smhc, SDXC_REG_GCTRL, (SDXC_DMAReset | SDXC_FIFOReset));
    writel(SDXC_IDMACSoftRST, smhc->reg_base + SDXC_REG_DMAC); /* reset IDMAC */
    __mci_check_bit_clear(smhc, SDXC_REG_DMAC, SDXC_IDMACSoftRST);
    temp = SDXC_IDMACFixBurst | SDXC_IDMACIDMAOn;
    writel(temp, smhc->reg_base + SDXC_REG_DMAC);
    /* enable IDMA interrupt, here not use */
    temp = readl(smhc->reg_base + SDXC_REG_IDIE);
    temp &= ~(SDXC_IDMACReceiveInt | SDXC_IDMACTransmitInt);

    if (smhc->req->data->flags & DATA_DIR_WRITE) {
        ;//temp |= SDXC_IDMACTransmitInt; /* disable dma int for less irqs */
    } else {
        temp |= SDXC_IDMACReceiveInt;
    }
    writel(temp, smhc->reg_base + SDXC_REG_IDIE);

    /* write descriptor address to register */
    writel(SDXC_IDMAC_DES_ADDR(HAL_PT_TO_U(pdes)), smhc->reg_base + SDXC_REG_DLBA);
    /* write water level */
    writel((BURST_SIZE << 28) | (SMC_RX_WLEVEL << 16) | SMC_TX_WLEVEL, smhc->reg_base + SDXC_REG_FTRGL);

    return pdes;
}

static void __mci_free_idma_des(smc_idma_des *pdes)
{
    pdes->config &= ~SDXC_IDMAC_DES0_OWN;
}

int32_t __mci_wait_access_done(struct sunxi_smhc *smhc)
{
    int32_t timeout = 1000;

    while(!(readl(smhc->reg_base + SDXC_REG_GCTRL) & SDXC_MemAccessDone))
    {
        hal_msleep(1);
        if(timeout-- < 0) LOG_E("wait memory access done timeout !!");
    }

    return 0;
}

static int32_t __mci_request_done(struct sunxi_smhc *smhc)
{
    unsigned long iflags;
    uint32_t temp;
    int32_t ret = 0;
    struct rt_mmcsd_req *req = smhc->req;
    struct rt_mmcsd_cmd *cmd = req->cmd;

    iflags = rt_spin_lock_irqsave(&smhc->sdmmc_lock);
    if (smhc->wait != SDC_WAIT_FINALIZE) {
        rt_spin_unlock_irqrestore(&smhc->sdmmc_lock, iflags);
        LOG_W("%s nothing finalize, wt %lx", __func__, HAL_PR_SZ_L(smhc->wait));
        return -1;
    }
    smhc->wait = SDC_WAIT_NONE;
    smhc->trans_done = 0;
    smhc->dma_done = 0;
    rt_spin_unlock_irqrestore(&smhc->sdmmc_lock, iflags);

    if (smhc->int_sum & SDXC_IntErrBit) {
        rt_thread_mdelay(1);
        LOG_E("SDC err, cmd %ld, %s%s%s%s%s%s%s%s%s%s", HAL_PR_SZ_L(smhc->smc_cmd & SDXC_CMD_OPCODE),
             smhc->int_sum & SDXC_RespErr    ? " RE" : "",
             smhc->int_sum & SDXC_RespCRCErr  ? " RCE" : "",
             smhc->int_sum & SDXC_DataCRCErr  ? " DCE" : "",
             smhc->int_sum & SDXC_RespTimeout ? " RTO" : "",
             smhc->int_sum & SDXC_DataTimeout ? " DTO" : "",
             smhc->int_sum & SDXC_DataStarve  ? " DS" : "",
             smhc->int_sum & SDXC_FIFORunErr  ? " FRE" : "",
             smhc->int_sum & SDXC_HardWLocked ? " HL" : "",
             smhc->int_sum & SDXC_StartBitErr ? " SBE" : "",
             smhc->int_sum & SDXC_EndBitErr   ? " EBE" : "");
        ret = -1;
        goto out;
    }

    if (resp_type(cmd) == RESP_R2) {
        cmd->resp[0] = readl(smhc->reg_base + SDXC_REG_RESP3);
        cmd->resp[1] = readl(smhc->reg_base + SDXC_REG_RESP2);
        cmd->resp[2] = readl(smhc->reg_base + SDXC_REG_RESP1);
        cmd->resp[3] = readl(smhc->reg_base + SDXC_REG_RESP0);
    } else
        cmd->resp[0] = readl(smhc->reg_base + SDXC_REG_RESP0);

out:
    if (req->data) {
        if (smhc->dma_hdle) {
            __mci_wait_access_done(smhc);
            writel(0x337, smhc->reg_base + SDXC_REG_IDST); /* clear interrupt flags */
            writel(0, smhc->reg_base + SDXC_REG_IDIE); /* disable idma interrupt */
            writel(0, smhc->reg_base + SDXC_REG_DMAC); /* idma off */
            temp = readl(smhc->reg_base + SDXC_REG_GCTRL);
            writel(temp | SDXC_DMAReset, smhc->reg_base + SDXC_REG_GCTRL);
            temp &= ~SDXC_DMAEnb;
            writel(temp, smhc->reg_base + SDXC_REG_GCTRL); /* disable IDMA */
            temp |= SDXC_FIFOReset;
            writel(temp, smhc->reg_base + SDXC_REG_GCTRL);
            __mci_free_idma_des((void *)smhc->dma_hdle);
            smhc->dma_hdle = NULL;
        }
        writel(readl(smhc->reg_base + SDXC_REG_GCTRL) | SDXC_FIFOReset, smhc->reg_base + SDXC_REG_GCTRL);
    }

    writel(smhc->sdio_irq_mask, smhc->reg_base + SDXC_REG_IMASK);

    if (smhc->int_sum & (SDXC_RespErr | SDXC_HardWLocked | SDXC_RespTimeout)) {
        SDC_LOGE("sdc line %d abnormal status: %s", __LINE__,
                 smhc->int_sum & SDXC_HardWLocked ? "HardWLocked" : "RespErr");
    }

    writel(0xffff, smhc->reg_base + SDXC_REG_RINTR);

    LOG_D("SDC done, resp %lx %lx %lx %lx", HAL_PR_SZ_L(cmd->resp[0]),
            HAL_PR_SZ_L(cmd->resp[1]), HAL_PR_SZ_L(cmd->resp[2]), HAL_PR_SZ_L(cmd->resp[3]));

    if (req->data && (smhc->int_sum & SDXC_IntErrBit)) {
        LOG_W("found data error, need to send stop command !!");
        __mci_regs_save(smhc);
        __mci_reset(smhc);
        __mci_clk_disable_unprepare(smhc);

        __mci_clk_prepare_enable(smhc);
        __mci_regs_restore(smhc);
        __mci_program_clk(smhc);
    }

    smhc->int_err = 0;
    return ret;
}

static void __mci_send_cmd(struct sunxi_smhc *smhc, struct rt_mmcsd_cmd *cmd)
{
    uint32_t imask = SDXC_IntErrBit;
    uint32_t cmd_val = SDXC_Start | (cmd->cmd_code & 0x3f);
    unsigned long iflags;
    uint32_t wait = SDC_WAIT_CMD_DONE;
    struct rt_mmcsd_data *data = cmd->data;

    if (cmd->cmd_code == GO_IDLE_STATE) {
        cmd_val |= SDXC_SendInitSeq;
        imask |= SDXC_CmdDone;
    }

    if (resp_type(cmd) != RESP_NONE) { /* with response */
        cmd_val |= SDXC_RspExp;
        if (resp_type(cmd) == RESP_R2) /* long response */
            cmd_val |= SDXC_LongRsp;
        if ((resp_type(cmd) != RESP_R3) && (resp_type(cmd) != RESP_R4))
            cmd_val |= SDXC_CheckRspCRC;

        if (cmd_type(cmd) == CMD_ADTC) { /* with data */
            if (!data) {
                LOG_E("%s,%d no data exist!", __func__, __LINE__);
                return;
            }
            cmd_val |= SDXC_DataExp | SDXC_WaitPreOver;
            wait = SDC_WAIT_DATA_OVER;
            if (data->flags & DATA_STREAM) { /* sequence mode */
                imask |= SDXC_AutoCMDDone;
                cmd_val |= SDXC_Seqmod | SDXC_SendAutoStop;
                wait = SDC_WAIT_AUTOCMD_DONE;
            }
            if (cmd->data->stop) {
                imask |= SDXC_AutoCMDDone;
                cmd_val |= SDXC_SendAutoStop;
                wait = SDC_WAIT_AUTOCMD_DONE;
            } else
                imask |= SDXC_DataOver;

            if (data->flags & DATA_DIR_WRITE) {
                cmd_val |= SDXC_Write;
            } else if (smhc->dma_hdle) {
                wait |= SDC_WAIT_IDMA_DONE;
            }
            LOG_D("blk_size:%lu, sg len:%lu", HAL_PR_SZ_L(data->blksize), HAL_PR_SZ_L(smhc->sg.len));
        } else
            imask |= SDXC_CmdDone;
    } else
        imask |= SDXC_CmdDone;

    LOG_D("sdc cmd:%ld(%lx), arg:%lx ie:%lx wt:%lx len:%lu",
             HAL_PR_SZ_L(cmd_val & SDXC_CMD_OPCODE), HAL_PR_SZ_L(cmd_val), HAL_PR_SZ_L(cmd->arg), HAL_PR_SZ_L(imask), HAL_PR_SZ_L(wait),
             HAL_PR_SZ_L(cmd->data ? cmd->data->blksize * cmd->data->blks : 0));

    iflags = rt_spin_lock_irqsave(&smhc->sdmmc_lock);
    smhc->smc_cmd = cmd_val;
    smhc->wait = wait;

    writel(imask | smhc->sdio_irq_mask, smhc->reg_base + SDXC_REG_IMASK);

    if (cmd_val & SDXC_SendAutoStop)
        writel(0, smhc->reg_base + SDXC_REG_A12A);
    else
        writel(0xffff, smhc->reg_base + SDXC_REG_A12A);

    writel(cmd->arg, smhc->reg_base + SDXC_REG_CARG);
    dsb(0xf);
    writel(cmd_val, smhc->reg_base + SDXC_REG_CMDR);

    rt_spin_unlock_irqrestore(&smhc->sdmmc_lock, iflags);
    if (data && NULL == smhc->dma_hdle) {
        __mci_trans_by_ahb(smhc, data);
    }
}

static int32_t HAL_SDC_Request(struct sunxi_smhc *smhc, struct rt_mmcsd_req *req)
{
    int32_t ret = 0;
    struct rt_mmcsd_cmd *cmd = req->cmd;
    struct rt_mmcsd_data *data = cmd->data;

    __mci_check_busy_over(smhc);
    __mci_check_bit_clear(smhc, SDXC_REG_GCTRL, SDXC_HWReset);

    /* disable debounce */
    __mci_debounce_onoff(smhc, 0);

    smhc->req = req;

    if(data) {
        rt_hw_cpu_dcache_clean_and_invalidate(smhc->sg.buffer, HAL_ALIGN(smhc->sg.len, CACHELINE_LEN));

        writel(data->blksize, smhc->reg_base + SDXC_REG_BLKSZ);
        writel(smhc->sg.len, smhc->reg_base + SDXC_REG_BCNTR);

        if(smhc->usedma) {
            __mci_sel_access_mode(smhc, SDXC_ACCESS_BY_DMA);
            smhc->dma_hdle = __mci_prepare_dma(smhc);
            if (NULL == smhc->dma_hdle) {
                LOG_W("SDC prepare DMA failed");
                __mci_sel_access_mode(smhc, SDXC_ACCESS_BY_AHB);
            }
        } else {
            /* switch data bus to ahb */
            __mci_sel_access_mode(smhc, SDXC_ACCESS_BY_AHB);
        }
    }

    __mci_send_cmd(smhc, cmd);

    ret = rt_sem_take(smhc->cmp_sem, SDC_DMA_TIMEOUT);

    if(ret != RT_EOK) {
        uint32_t rval = 0;
        LOG_E("sdc cmd:%ld, wait command done timeout !!", HAL_PR_SZ_L(smhc->smc_cmd & SDXC_CMD_OPCODE));

        if (data) {
            LOG_E("dump data blksz %ld,blocks %ld,flags %lx", HAL_PR_SZ_L(data->blksize), HAL_PR_SZ_L(data->blks), HAL_PR_SZ_L(data->flags));

            if(smhc->usedma) {
                hal_dcache_invalidate(smhc->sg.buffer, HAL_ALIGN(smhc->sg.len, CACHELINE_LEN));
            }
        }

        SDC_LOGE("%s,%d",__FUNCTION__,__LINE__);
        writel(0xdeb, smhc->reg_base + SDXC_REG_TMOUT);
        SDC_LOGE("*force wr smc timeout reg %x*",(unsigned int)readl(smhc->reg_base + SDXC_REG_TMOUT));

        SDC_LOGE("*force dump sram*");
        rval = hal_readl(0x03000000 + 0x0048);
        rval |= 1 << 4;
        writel(rval, 0x03000000 + 0x0048);
        cmd->err = -RT_ETIMEOUT;

        goto out;
    }

    ret = __mci_request_done(smhc);
    if(ret != RT_EOK) {
        cmd->err = -RT_ERROR;
    }

out:
    /* enable debounce */
    __mci_debounce_onoff(smhc, 1);

    return ret;
}

static void smhc_request_send(struct rt_mmcsd_host *host, struct rt_mmcsd_req *req)
{
    int32_t ret = 0;
    uint32_t byte_cnt = 0;
    unsigned long addr = 0;
    struct sunxi_smhc *smhc = (struct sunxi_smhc *)host->private_data;

    rt_mutex_take(smhc->req_mutex, SDC_DMA_TIMEOUT + 50);

    smhc->usedma = 0;
    if(req->data) {
        byte_cnt = req->data->blks * req->data->blksize;
        addr = HAL_PT_TO_U(req->data->buf);

        smhc->sg.buffer = req->data->buf;
        smhc->sg.len = byte_cnt;

        if((addr & 0x03) || (addr & (CACHELINE_LEN - 1)) || (byte_cnt & (CACHELINE_LEN - 1))) {
            smhc->usedma = 1;
            SDC_LOGD("try to use dma debounce buff, addr %lx, len %ld", HAL_PR_SZ_L(addr), HAL_PR_SZ_L(byte_cnt));
        }

        if(smhc->usedma) {
            if(byte_cnt > SDC_ALIGN_DMA_BUF_SIZE) {
                SDC_LOGE("byte count %ld over dma debounce buf size %d", HAL_PR_SZ_L(byte_cnt), SDC_ALIGN_DMA_BUF_SIZE);
                mmcsd_req_complete(host);
                return;
            } else if(byte_cnt < SDC_MAX_CPU_TRANS_LEN) {
                SDC_LOGD("byte cnt %ld is too small not use dma align buf", HAL_PR_SZ_L(byte_cnt));
                smhc->usedma = 0;
            } else {
                if(req->data->flags & DATA_DIR_WRITE) {
                    rt_memcpy(smhc->align_dma_buf, req->data->buf, byte_cnt);
                }

                smhc->sg.buffer = smhc->align_dma_buf;
                SDC_LOGD("use dma debounce buff addr %lx, len %d, align_dma_buf %lx", \
                    HAL_PT_TO_U(smhc->sg.buffer), (unsigned int)byte_cnt, HAL_PT_TO_U(smhc->align_dma_buf));
            }
        }
    }

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, smhc->sg.buffer, smhc->sg.len);

    ret = HAL_SDC_Request(smhc, req);

    if(!ret && req->data && (req->data->flags & DATA_DIR_READ)) {
        rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, smhc->sg.buffer, smhc->sg.len);
        if(smhc->usedma) {
            rt_memcpy(req->data->buf, smhc->sg.buffer, smhc->sg.len);
        }
    }

    rt_mutex_release(smhc->req_mutex);

    mmcsd_req_complete(host);
}

static void smhc_set_iocfg(struct rt_mmcsd_host *host, struct rt_mmcsd_io_cfg *cfg)
{
    struct sunxi_smhc *smhc = (struct sunxi_smhc *)host->private_data;

    LOG_D("smhc set io bus width:%d clock:%d", (cfg->bus_width == MMCSD_BUS_WIDTH_8 ?
            8 : (cfg->bus_width == MMCSD_BUS_WIDTH_4 ? 4 : 1)), cfg->clock);

    // power on off ?

    /* change clock */
    if((cfg->clock >= 400 * 1000) && (HAL_SDC_Update_Clk(smhc, cfg->clock) != RT_EOK))
    {
        LOG_E("update clock failed");
        return;
    }

    /* Change bus width */
    HAL_SDC_Set_BusWidth(smhc, cfg->bus_width);
}

int hal_sdc_init(struct sunxi_smhc *smhc)
{
    if(smhc->sdc_id == 0) {
        smhc->reg_base = SMC0_BASE;
        smhc->irq_num = SDC0_IRQn;
    }
    else if(smhc->sdc_id == 1) {
        smhc->reg_base = SMC1_BASE;
        smhc->irq_num = SDC1_IRQn;
    }
    else if(smhc->sdc_id == 2) {
        smhc->reg_base = SMC2_BASE;
        smhc->irq_num = SDC2_IRQn;
    }
    else {
        LOG_W("%s unsupport sdc id:%d!", __func__, smhc->sdc_id);
        return -1;
    }

    smhc->wait = SDC_WAIT_NONE;
    smhc->trans_done = 0;
    smhc->dma_done = 0;
    smhc->int_err = 0;

    smhc->power_on = 0;
    smhc->sdio_irq_mask = 0;

    rt_hw_interrupt_install(smhc->irq_num, __mci_irq_handler, smhc, "smhc_irq");

    return 0;
}

static rt_err_t sunxi_smhc_init(struct sunxi_smhc *smhc)
{
    smhc->align_dma_buf = hal_malloc_coherent(SDC_ALIGN_DMA_BUF_SIZE);
    if(smhc->align_dma_buf == NULL) {
        LOG_E("smhc align_dma_buf malloc failed");
        return -RT_ENOMEM;
    }
    smhc->idma_des = hal_malloc_coherent(HAL_ALIGN(SDXC_MAX_DES_NUM * sizeof(smc_idma_des), CACHELINE_LEN));
    if(smhc->idma_des == NULL) {
        LOG_E("smhc idma_des malloc failed");
        hal_free_coherent(smhc->align_dma_buf);
        return -RT_ENOMEM;
    }

    smhc->sdmmc_lock = (rt_spinlock_t)RT_SPINLOCK_INIT;
    rt_spin_lock_init(&smhc->sdmmc_lock);

    smhc->cmp_sem = rt_sem_create("sdio_sem", 0, RT_IPC_FLAG_PRIO);
    if (smhc->cmp_sem == RT_NULL) {
        LOG_E("smhc sem create failed");
        hal_free_coherent(smhc->align_dma_buf);
        hal_free_coherent(smhc->idma_des);
        return -RT_ERROR;
    }

    smhc->req_mutex = rt_mutex_create("sdio_mutex", RT_IPC_FLAG_FIFO);
    if(smhc->req_mutex == RT_NULL) {
        LOG_E("smhc mutex create failed");
        hal_free_coherent(smhc->align_dma_buf);
        hal_free_coherent(smhc->idma_des);
        rt_sem_delete(smhc->cmp_sem);
        return -RT_ERROR;
    }

    gpio_set_config(smhc->gpio_clk);
    gpio_set_config(smhc->gpio_cmd);
    gpio_set_config(smhc->gpio_d0);
    gpio_set_config(smhc->gpio_d1);
    gpio_set_config(smhc->gpio_d2);
    gpio_set_config(smhc->gpio_d3);

    hal_sdc_init(smhc);
    hal_mmc_power_up(smhc);

    return RT_EOK;
}

static const struct rt_mmcsd_host_ops ops =
{
    smhc_request_send,
    smhc_set_iocfg,
    RT_NULL,
    RT_NULL,
};

int rt_hw_sdio_init(void)
{
    struct rt_mmcsd_host *host;

#ifdef BSP_USING_SMHC0
    {
        static struct sunxi_smhc smhc0;

        host = mmcsd_alloc_host();
        if (!host) {
            LOG_E("alloc host failed");
            return -RT_ERROR;
        }

        smhc0.sdc_id = 0;
        smhc0.gpio_clk = (struct gpio_cfg){ GPIOF(2), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
        smhc0.gpio_cmd = (struct gpio_cfg){ GPIOF(3), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
        smhc0.gpio_d0 = (struct gpio_cfg){ GPIOF(1), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
        smhc0.gpio_d1 = (struct gpio_cfg){ GPIOF(0), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
        smhc0.gpio_d2 = (struct gpio_cfg){ GPIOF(5), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
        smhc0.gpio_d3 = (struct gpio_cfg){ GPIOF(4), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };

        if(sunxi_smhc_init(&smhc0) != RT_EOK) {
            LOG_E("sunxi_smhc_init failed");
            mmcsd_free_host(host);
            return -RT_ERROR;
        }

        host->ops = &ops;
        host->freq_min = 400 * 1000;
        host->freq_max = 150 * 1000 * 1000;
        host->valid_ocr = VDD_28_29 | VDD_29_30 | VDD_30_31 | VDD_31_32 | VDD_32_33 | VDD_33_34;
        host->flags = MMCSD_BUSWIDTH_4 | MMCSD_MUTBLKWRITE | MMCSD_SUP_SDIO_IRQ | MMCSD_SUP_HIGHSPEED;
        host->max_seg_size = SDXC_DES_BUFFER_MAX_LEN;
        host->max_dma_segs = 128;
        host->max_blk_size = 4096;
        host->max_blk_count = 8192;

        host->private_data = &smhc0;

        mmcsd_change(host);
    }
#endif

#ifdef BSP_USING_SMHC1
    {
        static struct sunxi_smhc smhc1;

        host = mmcsd_alloc_host();
        if (!host) {
            LOG_E("alloc host failed");
            return -RT_ERROR;
        }

        smhc1.sdc_id = 1;
//        smhc1.gpio_clk = (struct gpio_cfg){ GPIOF(2), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc1.gpio_cmd = (struct gpio_cfg){ GPIOF(3), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc1.gpio_d0 = (struct gpio_cfg){ GPIOF(1), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc1.gpio_d1 = (struct gpio_cfg){ GPIOF(0), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc1.gpio_d2 = (struct gpio_cfg){ GPIOF(5), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc1.gpio_d3 = (struct gpio_cfg){ GPIOF(4), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };

        if(sunxi_smhc_init(&smhc1) != RT_EOK) {
            LOG_E("sunxi_smhc_init failed");
            mmcsd_free_host(host);
            return -RT_ERROR;
        }

        host->ops = &ops;
        host->freq_min = 400 * 1000;
        host->freq_max = 150 * 1000 * 1000;
        host->valid_ocr = VDD_28_29 | VDD_29_30 | VDD_30_31 | VDD_31_32 | VDD_32_33 | VDD_33_34;
        host->flags = MMCSD_BUSWIDTH_4 | MMCSD_MUTBLKWRITE | MMCSD_SUP_SDIO_IRQ | MMCSD_SUP_HIGHSPEED;
        host->max_seg_size = SDXC_DES_BUFFER_MAX_LEN;
        host->max_dma_segs = 128;
        host->max_blk_size = 4096;
        host->max_blk_count = 8192;

        host->private_data = &smhc1;

        mmcsd_change(host);
    }
#endif

#ifdef BSP_USING_SMHC2
    {
        static struct sunxi_smhc smhc2;

        host = mmcsd_alloc_host();
        if (!host) {
            LOG_E("alloc host failed");
            return -RT_ERROR;
        }

        smhc2.sdc_id = 2;
//        smhc2.gpio_clk = (struct gpio_cfg){ GPIOF(2), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc2.gpio_cmd = (struct gpio_cfg){ GPIOF(3), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc2.gpio_d0 = (struct gpio_cfg){ GPIOF(1), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc2.gpio_d1 = (struct gpio_cfg){ GPIOF(0), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc2.gpio_d2 = (struct gpio_cfg){ GPIOF(5), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };
//        smhc2.gpio_d3 = (struct gpio_cfg){ GPIOF(4), GPIO_MUXSEL_FUNCTION2, GPIO_PULL_UP, GPIO_DRIVING_LEVEL3 };

        if(sunxi_smhc_init(&smhc2) != RT_EOK) {
            LOG_E("sunxi_smhc_init failed");
            mmcsd_free_host(host);
            return -RT_ERROR;
        }

        host->ops = &ops;
        host->freq_min = 400 * 1000;
        host->freq_max = 50 * 1000 * 1000;
        host->valid_ocr = VDD_28_29 | VDD_29_30 | VDD_30_31 | VDD_31_32 | VDD_32_33 | VDD_33_34;
        host->flags = MMCSD_BUSWIDTH_4 | MMCSD_MUTBLKWRITE | MMCSD_SUP_SDIO_IRQ | MMCSD_SUP_HIGHSPEED;
        host->max_seg_size = SDXC_DES_BUFFER_MAX_LEN;
        host->max_dma_segs = 128;
        host->max_blk_size = 4096;
        host->max_blk_count = 8192;

        host->private_data = &smhc2;

        mmcsd_change(host);
    }
#endif

    return RT_EOK;
}
INIT_ENV_EXPORT(rt_hw_sdio_init);

rt_err_t rt_hw_wait_mmc_attach(void)
{
    if(MMCSD_HOST_PLUGED == mmcsd_wait_cd_changed(1000))
    {
#ifdef RT_USING_DFS
        if(dfs_mount("sd0", "/sdcard", "elm", 0, 0) == 0)
        {
            rt_kprintf("/sdcard mount successfully\r\n");
        }
        else
        {
            rt_kprintf("%s mount failed\r\n", "sd0");
        }
#endif
        return RT_EOK;
    }
    else
    {
        rt_kprintf("Please insert a TF card!\r\n");
    }

    return -RT_ERROR;
}

#endif
