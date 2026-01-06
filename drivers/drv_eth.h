/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#ifndef DRIVERS_DRV_ETH_H_
#define DRIVERS_DRV_ETH_H_

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#define PHY_USING_IP101GR

/*  The PHY status register. */
#ifdef PHY_USING_LAN8720

#define PHY_Status_REG              0x1FU
#define PHY_10M_MASK                (1 << 2)
#define PHY_100M_MASK               (1 << 3)
#define PHY_FULL_DUPLEX_MASK        (1 << 4)

#elif defined(PHY_USING_IP101GR)

#define PHY_Status_REG              0x1EU
#define PHY_10M_MASK                (1 << 0)
#define PHY_100M_MASK               (1 << 1)
#define PHY_FULL_DUPLEX_MASK        (1 << 2)

#endif

#define PHY_STATUS_LINK_DOWN                (1)
#define PHY_STATUS_1000MBITS_FULLDUPLEX     (2)
#define PHY_STATUS_1000MBITS_HALFDUPLEX     (3)
#define PHY_STATUS_100MBITS_FULLDUPLEX      (4)
#define PHY_STATUS_100MBITS_HALFDUPLEX      (5)
#define PHY_STATUS_10MBITS_FULLDUPLEX       (6)
#define PHY_STATUS_10MBITS_HALFDUPLEX       (7)

#endif /* DRIVERS_DRV_ETH_H_ */
