/*
 * Copyright 2018 AllWinnertech  Co., Ltd
 * wangwei@allwinnertech.com
 */

#ifndef DRIVERS_HAL_GMAC_HAL_GMAC_H_
#define DRIVERS_HAL_GMAC_HAL_GMAC_H_

#include <stdlib.h>

#include "hal_common.h"
#include "hal_gpio.h"
#include "hal_log.h"
#include "hal_clk.h"
#include "hal_reset.h"
#include "hal_common.h"
#include "hal_atomic.h"

#include "hal_mii.h"
#include "hal_mdio.h"
#include "hal_miiphy.h"
#include "hal_phy.h"

#include "platform_gmac.h"

#include "lwip/pbuf.h"
#include "lwip/netif.h"

#define MAX_ADDR_LEN 6

enum {
    ETH_SPEED_10M,
    ETH_SPEED_100M,
    ETH_SPEED_1000M,
};

enum {
    ETH_HALFDUPLEX_MODE,
    ETH_FULLDUPLEX_MODE,
};

struct gmac_chip {
    /* interface address info, hw address */
    u8 mac_addr[MAX_ADDR_LEN];

    /* private variable */
    struct reset_control *rst;
    hal_clk_t gmac_clk;
    hal_clk_t gmac25m_clk;

    struct dma_desc *dma_desc_tx;
    struct dma_desc *dma_desc_rx;

    u32 dma_desc_tx_phy;
    u32 dma_desc_rx_phy;

    unsigned char **rx_buf;

    u32 tx_clean;
    u32 tx_dirty;
    u32 rx_clean;
    u32 rx_dirty;

    u32 tx_delay;
    u32 rx_delay;

    phy_interface_t phy_interface;

    u32 base;
    u32 syscfg_base;

    u32 speed;
    u32 duplex;

    hal_spinlock_t tx_lock;
    void (*callback)(void *p);
};

extern const u32 hal_gmac_dma_desc_num;

void hal_gmac_int_enable(struct gmac_chip *chip);
void hal_gmac_int_disable(struct gmac_chip *chip);

void hal_random_hwaddr(u8 *hwaddr);
int hal_gmac_netif_init(struct gmac_chip *chip, phy_interface_t phy_interface_mode, void (*callback)(void *p));
void hal_gmac_config(struct gmac_chip *chip);

u16 hal_gmac_phy_read(struct gmac_chip *chip, u8 phy_addr, u16 reg);
void hal_gmac_phy_write(struct gmac_chip *chip, u8 phy_addr, u8 reg, u16 data);

void hal_gmac_tx_complete(struct gmac_chip *chip);
int hal_gmac_rx(struct gmac_chip *chip, struct netif *netif, int limit);
struct pbuf *hal_gmac_input(struct gmac_chip *chip);
int hal_gmac_output(struct gmac_chip *chip, struct pbuf *p);

#endif /* DRIVERS_HAL_GMAC_HAL_GMAC_H_ */
