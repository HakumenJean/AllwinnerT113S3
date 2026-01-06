/*
* Allwinner GMAC driver.
*
* Copyright(c) 2022-2027 Allwinnertech Co., Ltd.
*
* This file is licensed under the terms of the GNU General Public
* License version 2.  This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/

#ifndef DRIVERS_HAL_GMAC_PLATFORM_GMAC_SUN8IW20_H_
#define DRIVERS_HAL_GMAC_PLATFORM_GMAC_SUN8IW20_H_

#include "hal_gpio.h"
#include "hal_common.h"
#include "hal_interrupt.h"

#define GMAC_BASE       0x04500000
#define SYSCFG_BASE     0x03000030

#define GMAC_CLK        CLK_BUS_EMAC0
#define GMAC25M_CLK     CLK_EMAC0_25M
#define GMAC_RST        RST_BUS_EMAC0

#define IRQ_GMAC        SUNXI_IRQ_EMAC

#endif /* DRIVERS_HAL_GMAC_PLATFORM_GMAC_SUN8IW20_H_ */
