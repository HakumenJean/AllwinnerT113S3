/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#ifndef DRIVERS_HAL_COMMON_HAL_SDMMC_H_
#define DRIVERS_HAL_COMMON_HAL_SDMMC_H_

#include "hal_common.h"
#include "hal_interrupt.h"

#define SMC0_BASE                       (0x04020000)
#define SMC1_BASE                       (0x04021000)
#define SMC2_BASE                       (0x04022000)

#define SDC_CCM_BASE                    (0x2001000)

#define  SDC0_IRQn                      SUNXI_IRQ_SMHC0
#define  SDC1_IRQn                      SUNXI_IRQ_SMHC1
#define  SDC2_IRQn                      SUNXI_IRQ_SMHC2

#define SDC_PLL_CLK                     (600*1000*1000)

#define SDC_DES_ADDR_SHIFT              (2)

#define SDC_MAX_FREQ_SDR_1V8            150000000


#define MMC_BUS_WIDTH_1         0
#define MMC_BUS_WIDTH_4         2
#define MMC_BUS_WIDTH_8         3


#define SDXC_REG_GCTRL                  (0x00)      /* SMC Global Control Register */
#define SDXC_REG_CLKCR                  (0x04)      /* SMC Clock Control Register */
#define SDXC_REG_TMOUT                  (0x08)      /* SMC Time Out Register */
#define SDXC_REG_WIDTH                  (0x0C)      /* SMC Bus Width Register */
#define SDXC_REG_BLKSZ                  (0x10)      /* SMC Block Size Register */
#define SDXC_REG_BCNTR                  (0x14)      /* SMC Byte Count Register */
#define SDXC_REG_CMDR                   (0x18)      /* SMC Command Register */
#define SDXC_REG_CARG                   (0x1C)      /* SMC Argument Register */
#define SDXC_REG_RESP0                  (0x20)      /* SMC Response Register 0 */
#define SDXC_REG_RESP1                  (0x24)      /* SMC Response Register 1 */
#define SDXC_REG_RESP2                  (0x28)      /* SMC Response Register 2 */
#define SDXC_REG_RESP3                  (0x2C)      /* SMC Response Register 3 */
#define SDXC_REG_IMASK                  (0x30)      /* SMC Interrupt Mask Register */
#define SDXC_REG_MISTA                  (0x34)      /* SMC Masked Interrupt Status Register */
#define SDXC_REG_RINTR                  (0x38)      /* SMC Raw Interrupt Status Register */
#define SDXC_REG_STAS                   (0x3C)      /* SMC Status Register */
#define SDXC_REG_FTRGL                  (0x40)      /* SMC FIFO Threshold Watermark Register */
#define SDXC_REG_FUNS                   (0x44)      /* SMC Function Select Register */
#define SDXC_REG_CBCR                   (0x48)      /* SMC CIU Byte Count Register */
#define SDXC_REG_BBCR                   (0x4C)      /* SMC BIU Byte Count Register */
#define SDXC_REG_DBGC                   (0x50)      /* SMC Debug Enable Register */
#define SDXC_REG_CSDC                   (0x54)      /* SMC CRC status detect control Register */
#define SDXC_REG_A12A                   (0x58)      /* SMC auto command 12 argument */
#define SDXC_REG_NTSR                   (0x5C)      /* SMC NewTiming Set Register(RX TX) */
#define SDXC_REG_SDEG                   (0x60)      /* SMC NewTiming Set debg */
#define SDXC_REG_HWST                   (0x78)      /* SMC SMC hardware reset register */
#define SDXC_REG_DMAC                   (0x80)      /* SMC IDMAC Control Register */
#define SDXC_REG_DLBA                   (0x84)      /* SMC IDMAC Descriptor List Base Address Register */
#define SDXC_REG_IDST                   (0x88)  /* SMC IDMAC Status Register */
#define SDXC_REG_IDIE                   (0x8C)      /* SMC IDMAC Interrupt Enable Register */
#define SDXC_REG_CHDA                   (0x90)      /* SMC Current Host Descriptor Address Register */
#define SDXC_REG_CBDA                   (0x94)      /* SMC Current Buffer Descriptor Address Register */
#define SDXC_REG_THLDC                  (0x100)     /* SMC Threshold Control Register */
#define SDXC_REG_DSBD                   (0x10C)
#define SDXC_REG_RESP_CRC               (0x110)
#define SDXC_REG_DAT7_CRC               (0x114)
#define SDXC_REG_DAT6_CRC               (0x118)
#define SDXC_REG_DAT5_CRC               (0x11C)
#define SDXC_REG_DAT4_CRC               (0x120)
#define SDXC_REG_DAT3_CRC               (0x124)
#define SDXC_REG_DAT2_CRC               (0x128)
#define SDXC_REG_DAT1_CRC               (0x12C)
#define SDXC_REG_DAT0_CRC               (0x130)
#define SDXC_REG_CRC_STA                (0x134)
#define SDXC_REG_DRV_DL                 (0x0140)
#define SDXC_REG_SAMP_DL                (0x144)
#define SDXC_REG_DS_DL                  (0x148)
#define SDXC_REG_FIFO                   (0x200) /* SMC FIFO Access Address */

#define SDXC_REG_FCTL                   (0x64)      /* SMC FIFO Access Address */
#define SDXC_REG_FCTL_OS                (0x64)      /* SMC FIFO Access Address */

/* global control register */
#define SDXC_SoftReset                  BIT(0 )
#define SDXC_FIFOReset                  BIT(1 )
#define SDXC_DMAReset                   BIT(2 )
#define SDXC_HWReset                    (SDXC_SoftReset|SDXC_FIFOReset|SDXC_DMAReset)
#define SDXC_INTEnb                     BIT(4 )
#define SDXC_DMAEnb                     BIT(5 )
#define SDXC_DebounceEnb                BIT(8 )
#define SDXC_DDR_MODE                   BIT(10)
#define SDXC_MemAccessDone              BIT(29)
#define SDXC_AccessDoneDirect           BIT(30)
#define SDXC_ACCESS_BY_AHB              BIT(31)
#define SDXC_ACCESS_BY_DMA              (0x0U << 31)

/* Clock control */
#define SDXC_CardClkOn                  (0x1U << 16)
#define SDXC_LowPowerOn                 (0x1U << 17)
#define SDXC_Mask_Data0                 BIT(31)

/* bus width */
#define SDXC_WIDTH1                     (0)
#define SDXC_WIDTH4                     (1)
#define SDXC_WIDTH8                     (2)

/* Struct for SMC Commands */
#define SDXC_CMD_OPCODE                 (0x3F )         /* 0x00000040 */
#define SDXC_RspExp                     BIT(6 )         /* 0x00000080 */
#define SDXC_LongRsp                    BIT(7 )         /* 0x00000100 */
#define SDXC_CheckRspCRC                BIT(8 )         /* 0x00000200 */
#define SDXC_DataExp                    BIT(9 )         /* 0x00000000 */
#define SDXC_Read                       (0x0U<<10 )     /* 0x00000400 */
#define SDXC_Write                      BIT(10)         /* 0x00000000 */
#define SDXC_Blockmod                   (0x0U<<11 )     /* 0x00000800 */
#define SDXC_Seqmod                     BIT(11)         /* 0x00001000 */
#define SDXC_SendAutoStop               BIT(12)         /* 0x00002000 */
#define SDXC_WaitPreOver                BIT(13)         /* 0x00004000 */
#define SDXC_StopAbortCMD               BIT(14)         /* 0x00008000 */
#define SDXC_SendInitSeq                BIT(15)         /* 0x00200000 */
#define SDXC_UPCLKOnly                  BIT(21)         /* 0x00400000 */
#define SDXC_RdCEATADev                 BIT(22)         /* 0x00800000 */
#define SDXC_CCSExp                     BIT(23)         /* 0x01000000 */
#define SDXC_EnbBoot                    BIT(24)         /* 0x02000000 */
#define SDXC_AltBootOpt                 BIT(25)         /* 0x00000000 */
#define SDXC_MandBootOpt                (0x0U<<25)      /* 0x04000000 */
#define SDXC_BootACKExp                 BIT(26)         /* 0x08000000 */
#define SDXC_DisableBoot                BIT(27)         /* 0x10000000 */
#define SDXC_VolSwitch                  BIT(28)         /* 0x80000000 */
#define SDXC_Start                      BIT(31)

/* Struct for Intrrrupt Information */
#define SDXC_RespErr                    BIT(1)          /* 0x00000002 */
#define SDXC_CmdDone                    BIT(2)          /* 0x00000004 */
#define SDXC_DataOver                   BIT(3)          /* 0x00000008 */
#define SDXC_TxDataReq                  BIT(4)          /* 0x00000010 */
#define SDXC_RxDataReq                  BIT(5)          /* 0x00000020 */
#define SDXC_RespCRCErr                 BIT(6)          /* 0x00000040 */
#define SDXC_DataCRCErr                 BIT(7)          /* 0x00000080 */
#define SDXC_RespTimeout                BIT(8)          /* 0x00000100 */
#define SDXC_ACKRcv                     BIT(8)          /* 0x00000100 */
#define SDXC_DataTimeout                BIT(9)          /* 0x00000200 */
#define SDXC_BootStart                  BIT(9)          /* 0x00000200 */
#define SDXC_DataStarve                 BIT(10)         /* 0x00000400 */
#define SDXC_VolChgDone                 BIT(10)         /* 0x00000400 */
#define SDXC_FIFORunErr                 BIT(11)         /* 0x00000800 */
#define SDXC_HardWLocked                BIT(12)         /* 0x00001000 */
#define SDXC_StartBitErr                BIT(13)         /* 0x00002000 */
#define SDXC_AutoCMDDone                BIT(14)         /* 0x00004000 */
#define SDXC_EndBitErr                  BIT(15)         /* 0x00008000 */
#define SDXC_SDIOInt                    BIT(16)         /* 0x00010000 */
#define SDXC_CardInsert                 BIT(30)         /* 0x40000000 */
#define SDXC_CardRemove                 BIT(31)         /* 0x80000000 */
#define SDXC_IntErrBit                  (SDXC_RespErr | SDXC_RespCRCErr | SDXC_DataCRCErr | SDXC_RespTimeout | SDXC_DataTimeout | \
                                         SDXC_FIFORunErr | SDXC_HardWLocked | SDXC_StartBitErr | SDXC_EndBitErr) /* 0xbbc2 */
#define SDXC_SWITCH_DDONE_BIT \
    (SDXC_VolChgDone | SDXC_CmdDone)

/* status */
#define SDXC_RXWLFlag                   BIT(0)
#define SDXC_TXWLFlag                   BIT(1)
#define SDXC_FIFOEmpty                  BIT(2)
#define SDXC_FIFOFull                   BIT(3)
#define SDXC_CardPresent                BIT(8)
#define SDXC_CardDataBusy               BIT(9)
#define SDXC_DataFSMBusy                BIT(10)
#define SDXC_DMAReq                     BIT(31)

/* Function select */
#define SDXC_CEATAOn                    (0xceaaU << 16)
#define SDXC_SendIrqRsp                 BIT(0)
#define SDXC_SDIORdWait                 BIT(1)
#define SDXC_AbtRdData                  BIT(2)
#define SDXC_SendCCSD                   BIT(8)
#define SDXC_SendAutoStopCCSD           BIT(9)
#define SDXC_CEATADevIntEnb             BIT(10)
/* status bit */
#define SDXC_CARD_PRESENT               BIT(8)
#define SDXC_CardBusy                   BIT(9)
/* IDMA controller bus mod bit field */
#define SDXC_IDMACSoftRST               BIT(0)
#define SDXC_IDMACFixBurst              BIT(1)
#define SDXC_IDMACIDMAOn                BIT(7)
#define SDXC_IDMACRefetchDES            BIT(31)

/* IDMA status bit field */
#define SDXC_IDMACTransmitInt           BIT(0)
#define SDXC_IDMACReceiveInt            BIT(1)
#define SDXC_IDMACFatalBusErr           BIT(2)
#define SDXC_IDMACDesInvalid            BIT(4)
#define SDXC_IDMACCardErrSum            BIT(5)
#define SDXC_IDMACNormalIntSum          BIT(8)
#define SDXC_IDMACAbnormalIntSum        BIT(9)
#define SDXC_IDMACHostAbtInTx           BIT(10)
#define SDXC_IDMACHostAbtInRx           BIT(10)
#define SDXC_IDMACIdle                  (0x0U << 13)
#define SDXC_IDMACSuspend               (0x1U << 13)
#define SDXC_IDMACDESCRd                (0x2U << 13)
#define SDXC_IDMACDESCCheck             (0x3U << 13)
#define SDXC_IDMACRdReqWait             (0x4U << 13)
#define SDXC_IDMACWrReqWait             (0x5U << 13)
#define SDXC_IDMACRd                    (0x6U << 13)
#define SDXC_IDMACWr                    (0x7U << 13)
#define SDXC_IDMACDESCClose             (0x8U << 13)

#define SDXC_IDMA_OVER                  (SDXC_IDMACTransmitInt|SDXC_IDMACReceiveInt|SDXC_IDMACNormalIntSum)
#define SDXC_IDMA_ERR                   (SDXC_IDMACFatalBusErr|SDXC_IDMACDesInvalid|SDXC_IDMACCardErrSum|SDXC_IDMACAbnormalIntSum)

#ifndef SDC_CCM_SDC0_SCLK_CTRL
#define SDC_CCM_SDC0_SCLK_CTRL          (SDC_CCM_BASE + 0x830)
#endif

#ifndef SDC_CCM_SDC1_SCLK_CTRL
#define SDC_CCM_SDC1_SCLK_CTRL          (SDC_CCM_BASE + 0x834)
#define SDC_CCM_SDC_BUS_GATE_RESET      (SDC_CCM_BASE + 0x84c)
#endif
#ifndef SDC_CCM_SDC2_SCLK_CTRL
#define SDC_CCM_SDC2_SCLK_CTRL          (SDC_CCM_BASE + 0x838)
#endif

#ifndef SDC_CCM_SClk_DIV_N_SHIFT
#define SDC_CCM_SClk_DIV_N_SHIFT   (8)
#endif

#ifndef SDC_CCM_SClk_DIV_M_SHIFT
#define SDC_CCM_SClk_DIV_M_SHIFT   (0)
#endif


#define DEFINE_SYS_CRYSTAL  24 * 1000 * 1000
#define DEFINE_SYS_DEVCLK   hal_clk_get_rate(hal_clock_get(HAL_SUNXI_CCU, CLK_PLL_PERIPH0_2X))


#define SDC_CACHE_ALIGN_BYTES   (64)
#define SDC_ALIGN_DMA_BUF_SIZE  (64*1024)
#define SDC_MAX_CPU_TRANS_LEN           (4)

#define SDC_BUSY_WAIT_LOOP     (0xffffffff)

#define SDXC_IDMAC_DES_ADDR(a)  ((a)>>SDC_DES_ADDR_SHIFT)

#define SMC_RX_WLEVEL                   7
#define SMC_TX_WLEVEL                   248
#define BURST_SIZE                      2

#define SDC_DMA_TIMEOUT         (1000*20) /* not much data to write on this platform */

#define SDC_WAIT_NONE                   BIT(0)
#define SDC_WAIT_CMD_DONE               BIT(1)
#define SDC_WAIT_DATA_OVER              BIT(2)
#define SDC_WAIT_AUTOCMD_DONE           BIT(3)
#define SDC_WAIT_IDMA_DONE              BIT(4)
#define SDC_WAIT_IDMA_ERR               BIT(5)
#define SDC_WAIT_ERROR                  BIT(6)
#define SDC_WAIT_RXDATA_OVER            (SDC_WAIT_DATA_OVER|SDC_WAIT_IDMA_DONE)
#define SDC_WAIT_RXAUTOCMD_DONE         (SDC_WAIT_AUTOCMD_DONE|SDC_WAIT_IDMA_DONE)
#define SDC_WAIT_SWITCH1V8              BIT(7)
#define SDC_WAIT_FINALIZE               BIT(8)

#define SDXC_DES_NUM_SHIFT              13
#define SDXC_DES_BUFFER_MAX_LEN         (1 << SDXC_DES_NUM_SHIFT)
#define SDXC_MAX_TRANS_LEN              (1 << 18 << 4)       /* max len is 4M */
#define SDXC_MAX_DES_NUM                (SDXC_MAX_TRANS_LEN >> SDXC_DES_NUM_SHIFT)

struct __mci_ctrl_regs {
    uint32_t gctrl;
    uint32_t clkc;
    uint32_t timeout;
    uint32_t buswid;
    uint32_t waterlvl;
    uint32_t funcsel;
    uint32_t debugc;
    uint32_t idmacc;
    uint32_t dlba;
    uint32_t imask;
    uint32_t drv_dl;
    uint32_t samp_dl;
    uint32_t ds_dl;
    uint32_t sd_ntsr;
    uint32_t edsd;
    uint32_t csdc;
};

#endif /* DRIVERS_HAL_COMMON_HAL_SDMMC_H_ */
