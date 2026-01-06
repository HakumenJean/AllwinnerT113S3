/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include "drv_eth.h"

#define DBG_TAG  "eth"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if defined(BSP_USING_ETH)

#include <netif/ethernetif.h>
#include <lwipopts.h>
#include "hal_gmac.h"

#ifndef LWIP_NO_RX_THREAD
#warning "You'd better turn this option on and use the driver I provided"
#endif

#if !defined(LWIP_NO_TX_THREAD) && !defined(RT_USING_SMP)
#warning "There may be some problems when using only single core and tx thread"
#endif

static struct rt_sunxi_eth
{
    struct eth_device   parent;

    struct gmac_chip    *chip;
    rt_uint8_t          phy_addr;
#ifdef LWIP_NO_RX_THREAD
    rt_sem_t            eth_rx_thread_sem;
    rt_mailbox_t        eth_rx_thread_mb;
    rt_thread_t         eth_rx_thread;
#endif
}sunxi_eth_device;

void eth_rx_cb(void *param)
{
    rt_err_t result;

#ifndef LWIP_NO_RX_THREAD
    result = eth_device_ready(&(sunxi_eth_device.parent));
#else
    result = rt_sem_release(sunxi_eth_device.eth_rx_thread_sem);
#endif
    if (result != RT_EOK)
    {
        LOG_I("RxCpltCallback err = %d", result);
    }
}

#ifdef LWIP_NO_RX_THREAD
static void eth_rx_thread_entry(void* parameter)
{
    int work_done = 0;
    int budget = hal_gmac_dma_desc_num / 4;
    struct gmac_chip *chip = sunxi_eth_device.chip;
    struct eth_device *device = &sunxi_eth_device.parent;

    while (1)
    {
        rt_sem_take(sunxi_eth_device.eth_rx_thread_sem, RT_WAITING_FOREVER);
        work_done = 0;

        while(1)
        {
            //hal_gmac_tx_complete(chip);
            work_done = hal_gmac_rx(chip, device->netif, budget);
            if(work_done < budget) {
                hal_gmac_int_enable(chip);
                break;
            }
        }
    }
}
#endif

static rt_err_t rt_sunxi_eth_init(rt_device_t dev)
{
    rt_err_t ret = RT_EOK;

    ret = hal_gmac_netif_init(sunxi_eth_device.chip, PHY_INTERFACE_MODE_RMII, eth_rx_cb);

    // phy init 移至 phy_monitor_thread_entry

    return ret;
}

static rt_err_t rt_sunxi_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    LOG_D("emac open");
    return RT_EOK;
}

static rt_err_t rt_sunxi_eth_close(rt_device_t dev)
{
    LOG_D("emac close");
    return RT_EOK;
}

static rt_ssize_t rt_sunxi_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    LOG_D("emac read");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_ssize_t rt_sunxi_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    LOG_D("emac write");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_sunxi_eth_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
        case NIOCTL_GADDR:
            /* get mac address */
            if (args)
            {
                SMEMCPY(args, sunxi_eth_device.chip->mac_addr, MAX_ADDR_LEN);
            }
            else
            {
                return -RT_ERROR;
            }
            break;

        default :
            break;
    }

    return RT_EOK;
}

/* ethernet device interface */
/* transmit data*/
rt_err_t rt_sunxi_eth_tx(rt_device_t dev, struct pbuf *p)
{
    return hal_gmac_output(sunxi_eth_device.chip, p);
}

/* receive data*/
struct pbuf *rt_sunxi_eth_rx(rt_device_t dev)
{
#ifndef LWIP_NO_RX_THREAD
    return hal_gmac_input(sunxi_eth_device.chip);
#else
    return RT_NULL;
#endif
}

static rt_uint32_t phy_link_state_get(void)
{
    rt_uint32_t status = 0;

    status = hal_gmac_phy_read(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_BMSR);
    status = hal_gmac_phy_read(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_BMSR);
    LOG_D("phy basic status reg is 0x%X", status);

    if((status & BMSR_LSTATUS) == 0)
        return PHY_STATUS_LINK_DOWN;

    status = hal_gmac_phy_read(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_BMCR);
    if((status & BMCR_ANENABLE) != BMCR_ANENABLE) {
        if(status & BMCR_SPEED1000) {
            if(status & BMCR_FULLDPLX) return PHY_STATUS_1000MBITS_FULLDUPLEX;
            else return PHY_STATUS_1000MBITS_HALFDUPLEX;
        } else if(status & BMCR_SPEED100) {
            if(status & BMCR_FULLDPLX) return PHY_STATUS_100MBITS_FULLDUPLEX;
            else return PHY_STATUS_100MBITS_HALFDUPLEX;
        } else {
            if(status & BMCR_FULLDPLX) return PHY_STATUS_10MBITS_FULLDUPLEX;
            else return PHY_STATUS_10MBITS_HALFDUPLEX;
        }
    } else {
        status = hal_gmac_phy_read(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, PHY_Status_REG);

        ///@todo: 1000M
        if(status & PHY_100M_MASK) {
            if(status & PHY_FULL_DUPLEX_MASK) return PHY_STATUS_100MBITS_FULLDUPLEX;
            else return PHY_STATUS_100MBITS_HALFDUPLEX;
        } else {
            if(status & PHY_FULL_DUPLEX_MASK) return PHY_STATUS_10MBITS_FULLDUPLEX;
            else return PHY_STATUS_10MBITS_HALFDUPLEX;
        }
    }
}

static void sunxi_eth_gpio_init(void)
{
#ifdef PHY_USING_LAN8720
    for (int i = 0; i < 10; i++) {
        hal_gpio_pinmux_set_function(GPIOE(i), 8);
        hal_gpio_set_driving_level(GPIOE(i), GPIO_DRIVING_LEVEL1);
    }
#elif defined(PHY_USING_IP101GR)
    hal_gpio_pinmux_set_function(GPIO_PG0, 4);
    hal_gpio_set_driving_level(GPIO_PG0, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG1, 4);
    hal_gpio_set_driving_level(GPIO_PG1, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG2, 4);
    hal_gpio_set_driving_level(GPIO_PG2, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG3, 4);
    hal_gpio_set_driving_level(GPIO_PG3, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG4, 4);
    hal_gpio_set_driving_level(GPIO_PG4, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG5, 4);
    hal_gpio_set_driving_level(GPIO_PG5, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG11, 4);
    hal_gpio_set_driving_level(GPIO_PG11, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG12, 4);
    hal_gpio_set_driving_level(GPIO_PG12, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG13, 4);
    hal_gpio_set_driving_level(GPIO_PG13, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG14, 4);
    hal_gpio_set_driving_level(GPIO_PG14, GPIO_DRIVING_LEVEL1);
    hal_gpio_pinmux_set_function(GPIO_PG15, 4);
    hal_gpio_set_driving_level(GPIO_PG15, GPIO_DRIVING_LEVEL1);

//#ifdef PHY_USING_IP101GR
//    hal_gpio_set_direction(GPIOG(6), 1);
//    hal_gpio_set_data(GPIOG(6), 1);
//#endif
#endif
}

static void sunxi_eth_phy_init(void)
{
    rt_uint32_t phy_val, time_out = 0;

    // reset phy
    hal_gmac_phy_write(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_BMCR, BMCR_RESET);
    phy_val = hal_gmac_phy_read(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_BMCR);
    while(phy_val & BMCR_RESET)
    {
        phy_val = hal_gmac_phy_read(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_BMCR);
        rt_thread_mdelay(1);
        time_out++;
        if(time_out >= 2000) break;
    }

    //rt_thread_mdelay(2000);

    // 开启自动协商
    phy_val = hal_gmac_phy_read(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_BMCR);
    phy_val |= BMCR_ANENABLE;
    hal_gmac_phy_write(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_BMCR, phy_val);
}

static void phy_monitor_thread_entry(void *parameter)
{
    int i;
    rt_int32_t phyLinkState = 0;
    rt_uint32_t phy_val;

//#ifdef PHY_USING_IP101GR
//    hal_gpio_set_data(GPIOG(6), 1);
//    hal_msleep(100);
//    hal_gpio_set_data(GPIOG(6), 1);
//    rt_thread_mdelay(2000);
//#endif

    /* scan mdio bus, find phy addr */
    for (i = 0; i < 0x20; i++) {
        sunxi_eth_device.phy_addr = i;
        phy_val = hal_gmac_phy_read(sunxi_eth_device.chip, sunxi_eth_device.phy_addr, MII_PHYSID1);
        if(phy_val != 0xFFFF && phy_val != 0x00) {
            LOG_I("Found a phy, address: 0x%x", i);
            break;
        }
    }
    if(i == 0x20) LOG_E("No phy found!");

    sunxi_eth_phy_init();

    // 监控网络连接状态
    for(;;)
    {
        phyLinkState = phy_link_state_get();
        if(netif_is_link_up(sunxi_eth_device.parent.netif) && (phyLinkState <= PHY_STATUS_LINK_DOWN))
        {
            LOG_I("phy is disconnect");
            hal_gmac_int_disable(sunxi_eth_device.chip);
            eth_device_linkchange(&sunxi_eth_device.parent, RT_FALSE);
        }
        else if(!netif_is_link_up(sunxi_eth_device.parent.netif) && (phyLinkState > PHY_STATUS_LINK_DOWN))
        {
            switch(phyLinkState)
            {
                case PHY_STATUS_1000MBITS_FULLDUPLEX:
                    sunxi_eth_device.chip->speed = ETH_SPEED_1000M;
                    sunxi_eth_device.chip->duplex = ETH_FULLDUPLEX_MODE;
                    LOG_I("phy connect, 1000Mbps, Full duplex");
                    break;
                case PHY_STATUS_1000MBITS_HALFDUPLEX:
                    sunxi_eth_device.chip->speed = ETH_SPEED_1000M;
                    sunxi_eth_device.chip->duplex = ETH_HALFDUPLEX_MODE;
                    LOG_I("phy connect, 1000Mbps, Half duplex");
                    break;
                case PHY_STATUS_100MBITS_FULLDUPLEX:
                    sunxi_eth_device.chip->speed = ETH_SPEED_100M;
                    sunxi_eth_device.chip->duplex = ETH_FULLDUPLEX_MODE;
                    LOG_I("phy connect, 100Mbps, Full duplex");
                    break;
                case PHY_STATUS_100MBITS_HALFDUPLEX:
                    sunxi_eth_device.chip->speed = ETH_SPEED_100M;
                    sunxi_eth_device.chip->duplex = ETH_HALFDUPLEX_MODE;
                    LOG_I("phy connect, 100Mbps, Half duplex");
                    break;
                case PHY_STATUS_10MBITS_FULLDUPLEX:
                    sunxi_eth_device.chip->speed = ETH_SPEED_10M;
                    sunxi_eth_device.chip->duplex = ETH_FULLDUPLEX_MODE;
                    LOG_I("phy connect, 10Mbps, Full duplex");
                    break;
                default:
                    sunxi_eth_device.chip->speed = ETH_SPEED_10M;
                    sunxi_eth_device.chip->duplex = ETH_HALFDUPLEX_MODE;
                    LOG_I("phy connect, 10Mbps, Half duplex");
                    break;
            }

            hal_gmac_config(sunxi_eth_device.chip);
            hal_gmac_int_enable(sunxi_eth_device.chip);
            eth_device_linkchange(&sunxi_eth_device.parent, RT_TRUE);
        }

        rt_thread_mdelay(1000);
    }
}

#ifdef RT_USING_DEVICE_OPS

static struct rt_device_ops sunxi_eth_ops = {
    rt_sunxi_eth_init,
    rt_sunxi_eth_open,
    rt_sunxi_eth_close,
    rt_sunxi_eth_read,
    rt_sunxi_eth_write,
    rt_sunxi_eth_control
};

#endif

/*
 * 与传统配置lwip的关联
 * lwip_system_init         --> tcp_ip_init
 * eth_device_init          --> netif_add (--> ethernetif_init)
 * eth_netif_device_init    --> ethernetif_init
 * rt_device_init(device)   --> rt_sunxi_eth_init --> low_level_init
 * rt_sunxi_eth_rx          --> low_level_input
 * rt_sunxi_eth_tx          --> low_level_output
 * 同样的监听网络链路连接需要自写, 再调用eth_device_linkchange
 */

/* Register the EMAC device */
int rt_hw_sunxi_eth_init(void)
{
    rt_err_t ret = RT_EOK;

    sunxi_eth_device.chip = (struct gmac_chip *)rt_malloc(sizeof(struct gmac_chip));
    if(sunxi_eth_device.chip == NULL) {
        LOG_E("emac device chip malloc failed");
        return -RT_ENOMEM;
    }

    hal_random_hwaddr(sunxi_eth_device.chip->mac_addr);
    sunxi_eth_gpio_init();

#ifdef RT_USING_DEVICE_OPS
    sunxi_eth_device.parent.parent.ops = &sunxi_eth_ops;
#else
    sunxi_eth_device.parent.parent.init = rt_sunxi_eth_init;
    sunxi_eth_device.parent.parent.open = rt_sunxi_eth_open;
    sunxi_eth_device.parent.parent.close = rt_sunxi_eth_close;
    sunxi_eth_device.parent.parent.read = rt_sunxi_eth_read;
    sunxi_eth_device.parent.parent.write = rt_sunxi_eth_write;
    sunxi_eth_device.parent.parent.control = rt_sunxi_eth_control;
#endif

    sunxi_eth_device.parent.parent.user_data = RT_NULL;

    sunxi_eth_device.parent.eth_rx = rt_sunxi_eth_rx;
    sunxi_eth_device.parent.eth_tx = rt_sunxi_eth_tx;

#ifdef LWIP_NO_RX_THREAD
    sunxi_eth_device.eth_rx_thread_sem = rt_sem_create("erxsem", 0, RT_IPC_FLAG_PRIO);
    RT_ASSERT(sunxi_eth_device.eth_rx_thread_sem != RT_NULL);

    sunxi_eth_device.eth_rx_thread = rt_thread_create("erx", eth_rx_thread_entry, RT_NULL,
            RT_LWIP_ETHTHREAD_STACKSIZE, RT_LWIP_ETHTHREAD_PRIORITY, 16);
    RT_ASSERT(sunxi_eth_device.eth_rx_thread != RT_NULL);
    rt_thread_startup(sunxi_eth_device.eth_rx_thread);
#endif

    /* register eth device */
    ret = eth_device_init(&(sunxi_eth_device.parent), "e0");
    if(ret == RT_EOK) {
        LOG_I("emac device init success");
    } else {
        LOG_E("emac device init failed: %d", ret);
        goto __exit;
    }

    rt_thread_t tid = rt_thread_create("phy", phy_monitor_thread_entry, RT_NULL, 2048, 14, 5);
    if(tid != RT_NULL) {
        rt_thread_startup(tid);
    } else {
        LOG_E("emac phy thread create failed: %d", ret);
        goto __exit;
    }

    return RT_EOK;

__exit:
    rt_free(sunxi_eth_device.chip);

    return ret;
}
INIT_DEVICE_EXPORT(rt_hw_sunxi_eth_init);

#endif

