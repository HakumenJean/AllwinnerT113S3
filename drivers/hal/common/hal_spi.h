/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the People's Republic of China and other countries.
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
#ifndef DRIVERS_HAL_COMMON_HAL_SPI_H_
#define DRIVERS_HAL_COMMON_HAL_SPI_H_

#include "hal_common.h"
#include "hal_interrupt.h"

#define SUNXI_SPI0_PBASE        (0x04025000ul) /* 4K */
#define SUNXI_SPI1_PBASE        (0x04026000ul) /* 4K */

#define SPI0_IRQn               SUNXI_IRQ_SPI0
#define SPI1_IRQn               SUNXI_IRQ_SPI1

/* Debug Dump */
#define HEXADECIMAL (0x10)
#define REG_END     (0x0f)
#define SAMPLE_NUMBER   (0x80)
#define REG_INTERVAL    (0x04)
#define REG_CL      (0x0c)

#define SUNXI_SPI_CS_MAX        (4)         /* SPI Controller support max chip select */
#define SUNXI_SPI_FIFO_DEFAULT  (64)        /* SPI Controller default fifo depth */
#define SUNXI_SPI_MAX_FREQUENCY (100000000) /* SPI Controller support max freq 100Mhz */
#define SUNXI_SPI_MIN_FREQUENCY (3000)      /* SPI Controller support min freq 3Khz */

#define SUNXI_SPI_BIT_MAX_LEN   (32)

/* SPI Global Control Register */
#define SUNXI_SPI_GC_REG        (0x04)
    #define SUNXI_SPI_GC_SRST           BIT(31) /* Soft Reset */
    #define SUNXI_SPI_GC_TP_EN          BIT(7)  /* Transmit Pause Enable */
    #define SUNXI_SPI_GC_DBI_EN         BIT(4)  /* DBI Mode Enable Control */
    #define SUNXI_SPI_GC_DBI_MODE_SEL   BIT(3)  /* SPI DBI Working Mode Select */
    #define SUNXI_SPI_GC_MODE_SEL       BIT(2)  /* Sample Timing Mode Select */
    #define SUNXI_SPI_GC_MODE           BIT(1)  /* SPI Function Mode Select */
    #define SUNXI_SPI_GC_EN             BIT(0)  /* SPI Module Enable Control */

/* SPI Transfer Control Register */
#define SUNXI_SPI_TC_REG        (0x08)
    #define SUNXI_SPI_TC_XCH        BIT(31) /* Exchange Burst */
    #define SUNXI_SPI_TC_SDC1       BIT(15) /* Master Sample Data Control Register 1 */
    #define SUNXI_SPI_TC_SDDM       BIT(14) /* Sending Data Delay Mode */
    #define SUNXI_SPI_TC_SDM        BIT(13) /* Master Sample Data Mode */
    #define SUNXI_SPI_TC_FBS        BIT(12) /* First Transmit Bit Select */
    #define SUNXI_SPI_TC_SDC        BIT(11) /* Master Sample Data Control */
    #define SUNXI_SPI_TC_RPSM       BIT(10) /* Select Rapids mode for high speed write */
    #define SUNXI_SPI_TC_DDB        BIT(9)  /* Dummy Burst Type */
    #define SUNXI_SPI_TC_DHB        BIT(8)  /* Discard Hash Burst */
    #define SUNXI_SPI_TC_SS_LEVEL   BIT(7)  /* SS Signal Level Output */
    #define SUNXI_SPI_TC_SS_OWNER   BIT(6)  /* SS Output Owner Select */
    #define SUNXI_SPI_TC_SS_SEL     (0x3 << 4)  /* SPI Chip Select */
    #define SUNXI_SPI_TC_SSCTL      BIT(3)
    #define SUNXI_SPI_TC_SPOL       BIT(2)  /* SPI Chip Select Signal Polarity Control */
    #define SUNXI_SPI_TC_CPOL       BIT(1)  /* SPI Clock Polarity Control */
    #define SUNXI_SPI_TC_CPHA       BIT(0)  /* SPI Clock/Data Phase Control */
    #define SUNXI_SPI_TC_SS_SEL_POS (4)

/* SPI Interrupt Control Register */
#define SUNXI_SPI_INT_CTL_REG   (0x10)
    #define SUNXI_SPI_INT_CTL_SS_EN     BIT(13) /* SSI Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_TC_EN     BIT(12) /* Transfer Completed Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_TX_UDR_EN BIT(11) /* TX FIFO Underrun Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_TX_OVF_EN BIT(10) /* TX FIFO Overflow Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_RX_UDR_EN BIT(9)  /* RX FIFO Underrun Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_RX_OVF_EN BIT(8)  /* RX FIFO Overflow Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_TX_FUL_EN BIT(6)  /* TX FIFO Full Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_TX_EMP_EN BIT(5)  /* TX FIFO Empty Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_TX_ERQ_EN BIT(4)  /* TX FIFO Empty Request Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_RX_FUL_EN BIT(2)  /* RX FIFO Full Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_RX_EMP_EN BIT(1)  /* RX FIFO Empty Interrupt Enable */
    #define SUNXI_SPI_INT_CTL_RX_RDY_EN BIT(0)  /* RX FIFO Ready Request Interrupt Enable */
    /* non-Register */
    #define SUNXI_SPI_INT_CTL_ERR       (SUNXI_SPI_INT_CTL_TX_OVF_EN|SUNXI_SPI_INT_CTL_RX_UDR_EN|SUNXI_SPI_INT_CTL_RX_OVF_EN)   /* No TX FIFO Underrun */
    #define SUNXI_SPI_INT_CTL_MASK      (0x77 | (0x3f << 8))

/* SPI Interrupt Status Register */
#define SUNXI_SPI_INT_STA_REG   (0x14)
    #define SUNXI_SPI_INT_STA_SSI       BIT(13) /* SS Invalid Interrupt */
    #define SUNXI_SPI_INT_STA_TC        BIT(12) /* Transfer Completed */
    #define SUNXI_SPI_INT_STA_TX_UDR    BIT(11) /* TX FIFO Underrun */
    #define SUNXI_SPI_INT_STA_TX_OVF    BIT(10) /* TX FIFO Overflow */
    #define SUNXI_SPI_INT_STA_RX_UDR    BIT(9)  /* RX FIFO Underrun */
    #define SUNXI_SPI_INT_STA_RX_OVF    BIT(8)  /* RX FIFO Overflow */
    #define SUNXI_SPI_INT_STA_TX_FULL   BIT(6)  /* TX FIFO Full */
    #define SUNXI_SPI_INT_STA_TX_EMP    BIT(5)  /* TX FIFO Empty */
    #define SUNXI_SPI_INT_STA_TX_RDY    BIT(4)  /* TX FIFO Ready */
    #define SUNXI_SPI_INT_STA_RX_FULL   BIT(2)  /* RX FIFO Full */
    #define SUNXI_SPI_INT_STA_RX_EMP    BIT(1)  /* RX FIFO Empty */
    #define SUNXI_SPI_INT_STA_RX_RDY    BIT(0)  /* RX FIFO Ready */
    /* non-Register */
    #define SUNXI_SPI_INT_STA_ERR       (SUNXI_SPI_INT_STA_TX_OVF|SUNXI_SPI_INT_STA_RX_UDR|SUNXI_SPI_INT_STA_RX_OVF)    /* No TX FIFO Underrun */
    #define SUNXI_SPI_INT_STA_MASK      (0x77 | (0x3f << 8))

/* SPI FIFO Control Register */
#define SUNXI_SPI_FIFO_CTL_REG  (0x18)
    #define SUNXI_SPI_FIFO_CTL_TX_RST           BIT(31)         /* TX FIFO Reset */
    #define SUNXI_SPI_FIFO_CTL_TX_TEST_EN       BIT(30)         /* TX Test Mode Enable */
    #define SUNXI_SPI_FIFO_CTL_TX_DRQ_EN        BIT(24)         /* TX FIFO DMA Request Enable */
    #define SUNXI_SPI_FIFO_CTL_TX_TRIG_LEVEL    (0xFF << 16)    /* TX FIFO Empty Request Trigger Level */
    #define SUNXI_SPI_FIFO_CTL_RX_RST           BIT(15)         /* RX FIFO Reset */
    #define SUNXI_SPI_FIFO_CTL_RX_TEST_EN       BIT(14)         /* RX Test Mode Enable */
    #define SUNXI_SPI_FIFO_CTL_RX_DRQ_EN        BIT(8)          /* RX FIFO DMA Request Enable */
    #define SUNXI_SPI_FIFO_CTL_RX_TRIG_LEVEL    (0xFF << 0)     /* RX FIFO Ready Request Trigger Level */
    /* non-Register */
    #define SUNXI_SPI_FIFO_CTL_RST      (SUNXI_SPI_FIFO_CTL_TX_RST|SUNXI_SPI_FIFO_CTL_RX_RST)
    #define SUNXI_SPI_FIFO_CTL_DRQ_EN   (SUNXI_SPI_FIFO_CTL_TX_DRQ_EN|SUNXI_SPI_FIFO_CTL_RX_DRQ_EN)
    #define SUNXI_SPI_FIFO_CTL_TX_TRIG_LEVEL_POS    (16)
    #define SUNXI_SPI_FIFO_CTL_RX_TRIG_LEVEL_POS    (0)

/* SPI FIFO Status Register */
#define SUNXI_SPI_FIFO_STA_REG  (0x1C)
    #define SUNXI_SPI_FIFO_STA_TB_WR    BIT(31)         /* TX FIFO Write Buffer Write Enable */
    #define SUNXI_SPI_FIFO_STA_TB_CNT   (0x7 << 28) /* TX FIFO Write Buffer Counter - Unused after 1890 */
    #define SUNXI_SPI_FIFO_STA_TX_CNT   (0xFF << 16)    /* TX FIFO Counter */
    #define SUNXI_SPI_FIFO_STA_RB_WR    BIT(15)         /* RX FIFO Read Buffer Write Enable */
    #define SUNXI_SPI_FIFO_STA_RB_CNT   (0x7 << 12) /* RX FIFO Read Buffer Counter - Unused after 1890 */
    #define SUNXI_SPI_FIFO_STA_RX_CNT   (0xFF << 0) /* RX FIFO Counter */
    /* non-Register */
    #define SUNXI_SPI_FIFO_STA_TB_CNT_POS   (28)
    #define SUNXI_SPI_FIFO_STA_TX_CNT_POS   (16)
    #define SUNXI_SPI_FIFO_STA_RB_CNT_POS   (12)
    #define SUNXI_SPI_FIFO_STA_RX_CNT_POS   (0)

/* SPI Sample Delay Register */
#define SUNXI_SPI_SAMP_DL_REG   (0x28)
    #define SUNXI_SPI_SAMP_DL_CAL_START BIT(15)         /* Sample Delay Calibration Start */
    #define SUNXI_SPI_SAMP_DL_CAL_DONE  BIT(14)         /* Sample Delay Calibration Done */
    #define SUNXI_SPI_SAMP_DL           (0x3F << 8)     /* Sample Delay */
    #define SUNXI_SPI_SAMP_DL_SW_EN     BIT(7)          /* Sample Delay Software Enable */
    #define SUNXI_SPI_SAMP_DL_SW        (0x3F << 0) /* Sample Delay Software */

/* SPI Master Burst Counter Register */
#define SUNXI_SPI_MBC_REG   (0x30)
    #define SUNXI_SPI_MBC       (0xFFFFFF << 0) /* Master Burst Counter */

/* SPI Master Transmit Counter Register */
#define SUNXI_SPI_MTC_REG   (0x34)
    #define SUNXI_SPI_MWTC      (0xFFFFFF << 0) /* Master Write Transmit Counter */

/* SPI Master Burst Control Counter Register */
#define SUNXI_SPI_BCC_REG       (0x38)
    #define SUNXI_SPI_BCC_QUAD_EN   BIT(29) /* Master Quad Mode Enable */
    #define SUNXI_SPI_BCC_DRM       BIT(28) /* Master Dual Mode Enable */
    #define SUNXI_SPI_BCC_DBC       (0xF << 24) /* Master Dummy Burst Counter */
    #define SUNXI_SPI_BCC_STC       (0xFFFFFF << 0) /* Master Single Mode Transmit Counter */
    /* non-Register */
    #define SUNXI_SPI_BCC_DBC_POS   (24)
    #define SUNXI_SPI_BCC_STC_POS   (0)

/* SPI Bit-Aligned Transfer Configure Register */
#define SUNXI_SPI_BATC_REG  (0x40)
    #define SUNXI_SPI_BATC_TCE          BIT(31)         /* Transfer Control Enable */
    #define SUNXI_SPI_BATC_MSMS         BIT(30)         /* Master Sample Standard */
    #define SUNXI_SPI_BATC_TBC          BIT(25)         /* Transfer Bits Completed */
    #define SUNXI_SPI_BATC_TBC_INT_EN   BIT(24)         /* Transfer Bits Completed Interrupt Enable */
    #define SUNXI_SPI_BATC_RX_FRM_LEN_POS   (16)        /* Configure the length of serial data frame(burst) of RX */
    #define SUNXI_SPI_BATC_RX_FRM_LEN_MASK  (0x3f << SUNXI_SPI_BATC_RX_FRM_LEN_POS)
    #define SUNXI_SPI_BATC_TX_FRM_LEN_POS   (8)         /* Configure the length of serial data frame(burst) of TX */
    #define SUNXI_SPI_BATC_TX_FRM_LEN_MASK  (0x3f << SUNXI_SPI_BATC_TX_FRM_LEN_POS)
    #define SUNXI_SPI_BATC_SS_LEVEL     BIT(7)          /* SS Signal Level Output */
    #define SUNXI_SPI_BATC_SS_OWNER     BIT(6)          /* SS Output Owner Select */
    #define SUNXI_SPI_BATC_SPOL         BIT(5)          /* SPI Chip Select Signal Polarity Control */
    #define SUNXI_SPI_BATC_SS_SEL_POS   (2)             /* SPI Chip Select */
    #define SUNXI_SPI_BATC_SS_SEL_MASK  (0x3 << SUNXI_SPI_BATC_SS_SEL_POS)
    #define SUNXI_SPI_BATC_WMS_POS      (0)             /* Work Mode Select */
    #define SUNXI_SPI_BATC_WMS_MASK         (0x3 << SUNXI_SPI_BATC_WMS_POS)
    /* non-Register */
    #define SUNXI_SPI_BIT_3WIRE_MODE    (2)
    #define SUNXI_SPI_BIT_STD_MODE      (3)

/* SPI TX Bit Register */
#define SUNXI_SPI_TB_REG    (0x48)

/* SPI RX Bit Register */
#define SUNXI_SPI_RB_REG    (0x4C)

/* SPI TX Data Register */
#define SUNXI_SPI_TXDATA_REG    (0x200)

/* SPI RX Data Register */
#define SUNXI_SPI_RXDATA_REG    (0x300)

#define SUNXI_SPI_SAMP_HIGH_FREQ    (60000000)  /* sample mode threshold frequency */
#define SUNXI_SPI_SAMP_LOW_FREQ     (24000000)  /* sample mode threshold frequency */
#define SUNXI_SPI_SAMP_MODE_DL_DEFAULT  (0xaaaaffff)

typedef enum spi_mode_type
{
    SINGLE_HALF_DUPLEX_RX,      /* single mode, half duplex read */
    SINGLE_HALF_DUPLEX_TX,      /* single mode, half duplex write */
    SINGLE_FULL_DUPLEX_TX_RX,   /* single mode, full duplex read and write */
    DUAL_HALF_DUPLEX_RX,        /* dual mode, half duplex read */
    DUAL_HALF_DUPLEX_TX,        /* dual mode, half duplex write */
    QUAD_HALF_DUPLEX_RX,        /* quad mode, half duplex read */
    QUAD_HALF_DUPLEX_TX,        /* quad mode, half duplex write */
    FULL_DUPLEX_TX_RX,          /* full duplex read and write */
    MODE_TYPE_NULL,
} spi_mode_type_t;

typedef enum
{
    HAL_SPI_BUS_MASTER = 0,
    HAL_SPI_BUS_SLAVE = 1,
    HAL_SPI_BUS_BIT = 2,
} sunxi_spi_bus_mode_t;

typedef enum
{
    SUNXI_SPI_SAMP_MODE_OLD = 0,
    SUNXI_SPI_SAMP_MODE_NEW = 1,
} sunxi_spi_bus_sample_mode_t;

typedef enum
{
    SUNXI_SPI_SAMP_DELAY_CYCLE_0_0 = 0,
    SUNXI_SPI_SAMP_DELAY_CYCLE_0_5 = 1,
    SUNXI_SPI_SAMP_DELAY_CYCLE_1_0 = 2,
    SUNXI_SPI_SAMP_DELAY_CYCLE_1_5 = 3,
    SUNXI_SPI_SAMP_DELAY_CYCLE_2_0 = 4,
    SUNXI_SPI_SAMP_DELAY_CYCLE_2_5 = 5,
    SUNXI_SPI_SAMP_DELAY_CYCLE_3_0 = 6,
} sunxi_spi_spi_sample_delay_t;

#endif /* DRIVERS_HAL_COMMON_HAL_SPI_H_ */
