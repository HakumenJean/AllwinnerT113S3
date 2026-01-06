/*
* Allwinner GMAC driver.
*
* Copyright(c) 2022-2027 Allwinnertech Co., Ltd.
*
* This file is licensed under the terms of the GNU General Public
* License version 2.  This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/


#include "hal_gmac.h"

const u32 hal_gmac_dma_desc_num = 256;

/* Ring buffer caculate method */
#define circ_cnt(head, tail, size) (((head) > (tail)) ? \
                    ((head) - (tail)) : \
                    ((head) - (tail)) & ((size) - 1))

#define circ_space(head, tail, size) circ_cnt((tail), ((head) + 1), (size))

#define circ_inc(n, s) (((n) + 1) % (s))

#ifdef DEBUG
static void pkt_hex_dump(char *prefix_str, void *pkt, unsigned int len)
{
    int i;
    unsigned char *data = (unsigned char *)pkt;
    for (i = 0; i < len; i++) {
        if (!(i % 16))
            hal_printf("\n%s %08x:", prefix_str, i);

        hal_printf(" %02x", data[i]);
    }
    hal_printf("\n");
}
#else
#define pkt_hex_dump(a, b, c)  {}
#endif

void hal_random_hwaddr(u8 *hwaddr)
{
    int i;

    for (i = 0; i < MAX_ADDR_LEN; i++)
        hwaddr[i] = (u8_t)rand();

    hwaddr[0] &= 0xfe;  /* clear multicast bit */
    hwaddr[0] |= 0x02;  /* set local assignment bit (IEEE802) */
}

static void hal_gmac_desc_init_chain(struct dma_desc *desc, u32 addr, u32 size)
{
    int i;
    struct dma_desc *p = desc;
    u32 dma_phy = addr;

    for (i = 0; i < (size - 1); i++) {
        dma_phy += sizeof(*p);
        p->desc3 = dma_phy;

        p->desc1.all |= 1 << GMAC_CHAIN_MODE_OFFSET;
        p++;
    }

    p->desc1.all |= 1 << GMAC_CHAIN_MODE_OFFSET;
    p->desc3 = addr;
}

static void hal_gmac_write_hwaddr(struct gmac_chip *chip)
{
    u32 mac;
    u8_t *mac_arr = chip->mac_addr;

    mac = (mac_arr[5] << 8) | mac_arr[4];
    hal_writel(mac, chip->base + GMAC_ADDR_HI(0));

    mac = (mac_arr[3] << 24) | (mac_arr[2] << 16) | (mac_arr[1] << 8) | mac_arr[0];
    writel(mac, chip->base + GMAC_ADDR_LO(0));
}

u16 hal_gmac_phy_read(struct gmac_chip *chip, u8 phy_addr, u16 reg)
{
    u32 value = 0;
    const u32 wait_us = 1;
    u32 timeout = GMAC_TIMEOUT_US;

    value |= GMAC_MDIO_MDC_DIV;
    value |= (((phy_addr << GMAC_MDIO_PHYADDR_OFFSET) & GMAC_MDIO_PHYADDR_MASK) |
            ((reg << GMAC_MDIO_PHYREG_OFFSET) & GMAC_MDIO_PHYREG_MASK) |
                    MII_BUSY);

    hal_writel(value, chip->base + GMAC_MDIO_ADDR);

    while (hal_readl(chip->base + GMAC_MDIO_ADDR) & MII_BUSY) {
        hal_udelay(wait_us);
        timeout -= wait_us;
        if (timeout <  0) {
            hal_printf("Error: %s Mii operation timeout\n", __func__);
            break;
        }
    }

    return hal_readl(chip->base + GMAC_MDIO_DATA);
}

void hal_gmac_phy_write(struct gmac_chip *chip, u8 phy_addr, u8 reg, u16 data)
{
    u32 value = 0;
    const u32 wait_us = 1;
    u32 timeout = GMAC_TIMEOUT_US;

    value |= GMAC_MDIO_MDC_DIV;
    value |= (((phy_addr << GMAC_MDIO_PHYADDR_OFFSET) & GMAC_MDIO_PHYADDR_MASK) |
            ((reg << GMAC_MDIO_PHYREG_OFFSET) & GMAC_MDIO_PHYREG_MASK) |
                    MII_WRITE | MII_BUSY);

    hal_writel(data, chip->base + GMAC_MDIO_DATA);
    hal_writel(value, chip->base + GMAC_MDIO_ADDR);

    while (hal_readl(chip->base + GMAC_MDIO_ADDR) & MII_BUSY) {
        hal_udelay(wait_us);
        timeout -= wait_us;
        if (timeout < 0) {
            hal_printf("Error: %s Mii operation timeout\n", __func__);
            break;
        }
    }
}

void hal_gmac_int_enable(struct gmac_chip *chip)
{
    u32 val;

    val = hal_readl(chip->base + GMAC_INT_EN);
    val |= GMAC_RX_INT_EN; // | 0x10
    hal_writel(val, chip->base + GMAC_INT_EN);
}

void hal_gmac_int_disable(struct gmac_chip *chip)
{
    u32 val;

    val = hal_readl(chip->base + GMAC_INT_EN);
    val &= ~GMAC_RX_INT_EN;
    hal_writel(val, chip->base + GMAC_INT_EN);
}

static void hal_gmac_syscfg_init(struct gmac_chip *chip)
{
    u32 value;

    value = hal_readl(chip->syscfg_base);

    /* Write phy type */
    value &= ~(1 << GMAC_PHY_SELECT_OFFSET);

    /* Only support RGMII/RMII */
    if (chip->phy_interface == PHY_INTERFACE_MODE_RGMII)
        value |= GMAC_PHY_RGMII_MASK;
    else
        value &= (~GMAC_PHY_RGMII_MASK);

    /* Write RGMII ETCS ret */
    value &= (~GMAC_ETCS_RMII_MASK);
    if (chip->phy_interface == PHY_INTERFACE_MODE_RGMII)
        value |= GMAC_RGMII_INTCLK_MASK;
    else if (chip->phy_interface == PHY_INTERFACE_MODE_RMII)
        value |= GMAC_RMII_MASK | (1 << 0);

    /*
     * Adjust Tx/Rx clock delay
     * Tx clock delay: 0~7
     * Rx clock delay: 0~31
     */
    value &= ~(GMAC_TX_DELAY_MASK << GMAC_TX_DELAY_OFFSET);
    value |= ((chip->tx_delay & GMAC_TX_DELAY_MASK) << GMAC_TX_DELAY_OFFSET);
    value &= ~(GMAC_RX_DELAY_MASK << GMAC_RX_DELAY_OFFSET);
    value |= ((chip->rx_delay & GMAC_RX_DELAY_MASK) << GMAC_RX_DELAY_OFFSET);

    hal_writel(value, chip->syscfg_base);
}

void hal_gmac_tx_complete(struct gmac_chip *chip)
{
    struct dma_desc *desc;
    u32 entry = 0;

    hal_spin_lock(&chip->tx_lock);

    while (circ_cnt(chip->tx_dirty, chip->tx_clean, hal_gmac_dma_desc_num) > 0) {
        entry = chip->tx_clean;
        desc = chip->dma_desc_tx + entry;

        hal_dcache_invalidate(desc, sizeof(struct dma_desc));

        /* make sure invalidate dcache finish */
        isb();

        if (desc->desc0.tx.own)
            break;

        desc->desc1.all = 0;
        desc->desc2 = 0;
        desc->desc1.all |= (1 << GMAC_CHAIN_MODE_OFFSET);

        hal_dcache_clean(desc, sizeof(struct dma_desc));

        chip->tx_clean = circ_inc(entry, hal_gmac_dma_desc_num);
    }
    hal_spin_unlock(&chip->tx_lock);
}

int hal_gmac_output(struct gmac_chip *chip, struct pbuf *p)
{
    struct pbuf *q;
    struct dma_desc *tx_p, *first;
    u32 value, send_status, frame_len, entry;

    entry = chip->tx_dirty;
    first = chip->dma_desc_tx + entry;
    tx_p = chip->dma_desc_tx + entry;

    if (circ_space(chip->tx_dirty, chip->tx_clean, hal_gmac_dma_desc_num) < 1) {
        hal_printf("Error: Gmac not have sufficient desc\n");
        return -EBUSY;
    }

    send_status = hal_readl((chip->base + GMAC_TX_DMA_STA)) & GMAC_TX_DMA_MASK;

    for (q = p; q != NULL; q = q->next) {
        tx_p = chip->dma_desc_tx + entry;
        frame_len = q->len;

        hal_dcache_invalidate(tx_p, sizeof(struct dma_desc));

        /* make sure invalidate dcache finish */
        isb();

        tx_p->desc1.all &= ~GMAC_DMA_BUFF_SIZE_MASK;
        tx_p->desc1.all |= GMAC_DMA_BUFF_SIZE(frame_len);
#ifdef RT_LWIP_USING_HW_CHECKSUM
        tx_p->desc1.tx.cic = 3;         /* Enable CRC & IPv4 Header Checksum */
#endif
        tx_p->desc2 = __va_to_pa(q->payload);

        if (first != tx_p)
            tx_p->desc0.all |= GMAC_DMA_OWN_DESC;  /* Set Own */

        hal_dcache_clean(q->payload, frame_len);
        hal_dcache_clean(tx_p, sizeof(struct dma_desc));

        entry = circ_inc(entry, hal_gmac_dma_desc_num);
        pkt_hex_dump("TX", (void *)q->payload, 64);
    }

    /* tx close */
    first->desc1.tx.first_sg = 1;
    tx_p->desc1.tx.last_seg = 1;
    tx_p->desc1.tx.interrupt = 1;
    chip->tx_dirty = entry;

    /* make sure config dma desc finish */
    dmb();

    first->desc0.all = GMAC_DMA_OWN_DESC;  /* Set Own */
    hal_dcache_clean(first, sizeof(struct dma_desc));
    hal_dcache_clean(tx_p, sizeof(struct dma_desc));

    /* make sure clean dcache finish */
    isb();

//    value = hal_readl(chip->base + GMAC_TX_CTL1);
//    value |= GMAC_TX_DMA_FLUSH_FIFO;
//    hal_writel(value, chip->base + GMAC_TX_CTL1);

    /* Enable transmit and Poll transmit */
    value = hal_readl(chip->base + GMAC_TX_CTL1);
    if (send_status == GMAC_TX_DMA_STOP)
        value |= GMAC_TX_DMA_EN;
    else
        value |= GMAC_TX_DMA_START;
    hal_writel(value, chip->base + GMAC_TX_CTL1);

    hal_gmac_tx_complete(chip);

    return 0;
}

static void hal_gmac_desc_buf_set(struct dma_desc *desc, u32 dma_addr, int size)
{
    desc->desc1.all &= ~GMAC_DMA_BUFF_SIZE_MASK;
    desc->desc1.all |= GMAC_DMA_BUFF_SIZE(size);
    desc->desc2 = dma_addr;
}

static void hal_gmac_rx_refill(struct gmac_chip *chip)
{
    struct dma_desc *desc;
    unsigned char *rx = NULL;
    u32 dma_addr;

    while (circ_space(chip->rx_clean, chip->rx_dirty, hal_gmac_dma_desc_num) > 0) {
        int entry = chip->rx_clean;

        desc = chip->dma_desc_rx + entry;

        if (!chip->rx_buf[entry]) {
            rx = hal_malloc_align(sizeof(unsigned char) * MAX_BUF_SZ, CACHELINE_LEN);
            if (!rx)
                break;

            chip->rx_buf[entry] = rx;
            rx = NULL;
        }

        hal_memset(chip->rx_buf[entry], 0, sizeof(unsigned char) * MAX_BUF_SZ);
        hal_dcache_clean(chip->rx_buf[entry], sizeof(unsigned char) * MAX_BUF_SZ);
        dma_addr = __va_to_pa((unsigned long)chip->rx_buf[entry]);
        hal_gmac_desc_buf_set(desc, dma_addr, MAX_BUF_SZ);

//        if (!chip->rx_buf[entry]) {
//            rx = hal_malloc_align(sizeof(unsigned char) * MAX_BUF_SZ, CACHELINE_LEN);
//            if (!rx)
//                break;
//
//            hal_memset((void *)rx, 0, sizeof(unsigned char) * MAX_BUF_SZ);
//            hal_dcache_clean(rx, sizeof(unsigned char) * MAX_BUF_SZ);
//            chip->rx_buf[entry] = rx;
//            dma_addr = __va_to_pa((unsigned long)rx);
//            hal_gmac_desc_buf_set(desc, dma_addr, MAX_BUF_SZ);
//
//            rx = NULL;
//        }

        dmb();
        desc->desc0.all = GMAC_DMA_OWN_DESC;  /* Set Own */
        hal_dcache_clean(desc, sizeof(struct dma_desc));

        chip->rx_clean = circ_inc(chip->rx_clean, hal_gmac_dma_desc_num);
    }
}

static int hal_gmac_rx_status(struct dma_desc *p)
{
    int ret = good_frame;

    if (p->desc0.rx.last_desc == 0)
        ret = discard_frame;

    if (p->desc0.rx.frm_type && (p->desc0.rx.chsum_err
            || p->desc0.rx.ipch_err))
        ret = discard_frame;

    if (p->desc0.rx.err_sum)
        ret = discard_frame;

    if (p->desc0.rx.len_err)
        ret = discard_frame;

    if (p->desc0.rx.mii_err)
        ret = discard_frame;

    return ret;
}

int hal_gmac_rx(struct gmac_chip *chip, struct netif *netif, int limit)
{
    err_t err;
    unsigned int rxcount = 0;
    struct dma_desc *desc;
    struct pbuf *p = NULL;
    u32 entry, frame_len;
    int status;

    while(rxcount < limit)
    {
        entry = chip->rx_dirty;
        desc = chip->dma_desc_rx + entry;

        if (desc->desc0.all & GMAC_DMA_OWN_DESC)
            break;

        rxcount++;
        chip->rx_dirty = circ_inc(chip->rx_dirty, hal_gmac_dma_desc_num);

        frame_len = desc->desc0.rx.frm_len;
        status = hal_gmac_rx_status(desc);

        if(chip->rx_buf[entry] == NULL) {
            hal_printf("Error: rx buf is NULL\r\n");
            break;
        }

        if (status == discard_frame) {
            hal_printf("Error: get error pkt\n");
            continue;
        }

        p = pbuf_alloc(PBUF_RAW, frame_len, PBUF_POOL);
        if(p != NULL) {
            hal_dcache_invalidate(chip->rx_buf[entry], MAX_BUF_SZ);
            hal_memcpy(p->payload, chip->rx_buf[entry], frame_len);
            err = netif->input(p, netif);
            if(err != ERR_OK)
            {
                hal_printf("ethernetif_input: Input error, %d\r\n", err);
                pbuf_free(p);
                p = NULL;
            }
        } else {
            hal_printf("Error: pbuf alloc failed\r\n");
        }

        hal_free_align(chip->rx_buf[entry]);
        chip->rx_buf[entry] = NULL;
    }

    hal_gmac_rx_refill(chip);

    return rxcount;
}

struct pbuf *hal_gmac_input(struct gmac_chip *chip)
{
    struct dma_desc *desc;
    struct pbuf *p = NULL;
    u32 entry, frame_len;
    int status;

    entry = chip->rx_dirty;
    desc = chip->dma_desc_rx + entry;

    hal_dcache_invalidate(desc, sizeof(struct dma_desc));

    if (desc->desc0.all & GMAC_DMA_OWN_DESC) {
        return NULL;
    }

    hal_dcache_invalidate(&desc->desc2, MAX_BUF_SZ);
    status = hal_gmac_rx_status(desc);
    if (status == discard_frame) {
        hal_printf("Error: get error pkt\n");
        goto out;
    }

    chip->rx_dirty = circ_inc(chip->rx_dirty, hal_gmac_dma_desc_num);

    /* get packet len from rx dma desc */
    frame_len = desc->desc0.rx.frm_len;

    //hal_printf("eth rx len: %u\r\n", frame_len);
    p = pbuf_alloc(PBUF_RAW, frame_len, PBUF_POOL);
    hal_dcache_invalidate(chip->rx_buf[entry], MAX_BUF_SZ);
    hal_memcpy(p->payload, chip->rx_buf[entry], frame_len);

out:
    hal_free_align(chip->rx_buf[entry]);
    chip->rx_buf[entry] = NULL;

    hal_gmac_rx_refill(chip);

    hal_gmac_int_enable(chip);

    return p;
}

static void hal_gmac_irq_handler(int irq, void *p)
{
    struct gmac_chip *chip = (struct gmac_chip *)p;
    u32 val;

    hal_gmac_int_disable(chip);

    val = hal_readl(chip->base + GMAC_INT_STA);
    hal_writel(val, chip->base + GMAC_INT_STA);

    if(chip->callback) chip->callback(chip);
}

static int hal_gmac_reset(u32 iobase, int n)
{
    u32 value;

    value = hal_readl(iobase + GMAC_BASIC_CTL1);
    value |= GMAC_SOFT_RST;
    hal_writel(value, iobase + GMAC_BASIC_CTL1);

    while((hal_readl(iobase + GMAC_BASIC_CTL1) & GMAC_SOFT_RST) && (n > 0))
    {
        hal_udelay(1);
        n--;
    }

    if(n <= 0) return -1;
    return 0;
}

void hal_gmac_config(struct gmac_chip *chip)
{
    u32 value;

    value = hal_readl((chip->base + GMAC_BASIC_CTL0));
    if(chip->duplex == ETH_FULLDUPLEX_MODE)
        value |= GMAC_MAC_DUPLEX_MASK;
    else
        value &= ~GMAC_MAC_DUPLEX_MASK;

    value &= ~GMAC_MAC_SPEED_MASK;
    if(chip->speed == ETH_SPEED_1000M)
        value |= GMAC_MAC_SPEED_1000M;
    else
        value |= GMAC_MAC_SPEED_100M;

    hal_writel(value, chip->base + GMAC_BASIC_CTL0);
}

static int hal_gmac_init(struct gmac_chip *chip)
{
    u32 value;
    int ret;

    ret = hal_gmac_reset(chip->base, 10000);    // 10ms
    if (ret) {
        hal_printf("Error: gmac reset failed, please check mac and phy clk, base : %X\n", chip->base);
        return -EINVAL;
    }

    /* Initialize core */
    value = hal_readl(chip->base + GMAC_TX_CTL0);
    value |= (1 << GMAC_TX_FRM_LEN_OFFSET);
    value |= (1 << GMAC_TX_EN_OFFSET);
    hal_writel(value, chip->base + GMAC_TX_CTL0);

    value = hal_readl(chip->base + GMAC_TX_CTL1);
    value |= GMAC_TX_MD;
    value |= GMAC_TX_NEXT_FRM;
    hal_writel(value, chip->base + GMAC_TX_CTL1);

    value = hal_readl(chip->base + GMAC_RX_CTL0);
    value |= (1 << GMAC_CHECK_CRC_OFFSET);      /* Enable CRC & IPv4 Header Checksum */
    /*
     * Enable rx
     * Enable strip_fcs
     */
    value |= (0x9 << GMAC_RX_EN_OFFSET);
    //value |= 1 << 16;       // rx flow ctl en
    hal_writel(value, chip->base + GMAC_RX_CTL0);

    value = hal_readl(chip->base + GMAC_RX_CTL1);
    //value |= GMAC_RX_MD;
    value |= GMAC_RX_DMA_MODE;
    //value |= (8 | 4);
    hal_writel(value, chip->base + GMAC_RX_CTL1);

    /* GMAC frame filter */
    value = hal_readl(chip->base + GMAC_RX_FRM_FLT);
    value = GMAC_HASH_MULTICAST;
    hal_writel(value, chip->base + GMAC_RX_FRM_FLT);

    /* Burst should be 8 */
    value = hal_readl(chip->base + GMAC_BASIC_CTL1);
    value |= (GMAC_BURST_LEN << GMAC_BURST_LEN_OFFSET);
    //value |= 1 << 1;        // rx priority is over tx
    hal_writel(value, chip->base + GMAC_BASIC_CTL1);

    /* Disable all interrupt of dma */
    hal_writel(0x00UL, chip->base + GMAC_INT_EN);

    hal_gmac_desc_init_chain(chip->dma_desc_tx, chip->dma_desc_tx_phy, hal_gmac_dma_desc_num);
    hal_gmac_desc_init_chain(chip->dma_desc_rx, chip->dma_desc_rx_phy, hal_gmac_dma_desc_num);

    chip->rx_clean = 0;
    chip->tx_clean = 0;
    chip->rx_dirty = 0;
    chip->tx_dirty = 0;

    hal_dcache_clean(chip->dma_desc_tx, sizeof(*chip->dma_desc_tx) * hal_gmac_dma_desc_num);
    hal_dcache_clean(chip->dma_desc_rx, sizeof(*chip->dma_desc_rx) * hal_gmac_dma_desc_num);

    hal_writel((unsigned long)chip->dma_desc_tx, chip->base + GMAC_TX_DESC_LIST);
    hal_writel((unsigned long)chip->dma_desc_rx, chip->base + GMAC_RX_DESC_LIST);

    value = hal_readl(chip->base + GMAC_RX_CTL1);
    value |= GMAC_RX_DMA_EN;
    hal_writel(value, chip->base + GMAC_RX_CTL1);

    hal_gmac_write_hwaddr(chip);

    hal_gmac_rx_refill(chip);

    return 0;
}

static void hal_gmac_init_param(struct gmac_chip *chip)
{
    chip->base = GMAC_BASE;
    chip->syscfg_base = SYSCFG_BASE;
    chip->tx_delay = 0;
    chip->rx_delay = 0;
}

static int hal_gmac_resource_get(struct gmac_chip *chip)
{
    hal_gmac_init_param(chip);

    chip->rst = hal_reset_control_get(HAL_SUNXI_RESET, GMAC_RST);
    if (!chip->rst) {
        hal_printf("get gmac rst failed\n");
        return -EINVAL;
    }

    chip->gmac_clk = hal_clock_get(HAL_SUNXI_CCU, GMAC_CLK);
    if (!chip->gmac_clk) {
        hal_printf("get gmac clk failed\n");
        return -EINVAL;
    }

    chip->gmac25m_clk = hal_clock_get(HAL_SUNXI_CCU, GMAC25M_CLK);
    if (!chip->gmac25m_clk) {
        hal_printf("get gmac 25m clk failed\n");
        return -EINVAL;
    }

    return 0;
}

static void hal_gmac_resource_put(struct gmac_chip *chip)
{
    hal_reset_control_put(chip->rst);
    hal_clock_put(chip->gmac_clk);
    hal_clock_put(chip->gmac25m_clk);
}

static int hal_gmac_clk_init(struct gmac_chip *chip)
{
    int ret;

    ret = hal_reset_control_deassert(chip->rst);
    if (ret) {
        hal_printf("deassert gmac rst failed\n");
        goto err0;
    }

    ret = hal_clock_enable(chip->gmac_clk);
    if (ret) {
        hal_printf("enable gmac clk failed\n");
        goto err1;
    }

    ret = hal_clock_enable(chip->gmac25m_clk);
    if (ret) {
        hal_printf("enable gmac 25m clk failed\n");
        goto err2;
    }

    return 0;

err2:
    hal_clock_disable(chip->gmac_clk);
err1:
    hal_reset_control_assert(chip->rst);
err0:
    return ret;
}

static void hal_gmac_clk_exit(struct gmac_chip *chip)
{
    hal_reset_control_assert(chip->rst);
    hal_clock_disable(chip->gmac_clk);
    hal_clock_disable(chip->gmac25m_clk);
}

static int hal_gmac_hardware_init(struct gmac_chip *chip)
{
    int ret;

    ret = hal_gmac_clk_init(chip);
    if (ret)
        goto err0;

    hal_gmac_syscfg_init(chip);

    return 0;

err0:
    return ret;
}

static void hal_gmac_hardware_exit(struct gmac_chip *chip)
{
    hal_gmac_clk_exit(chip);
}

static void hal_gmac_disable_irq(struct gmac_chip *chip)
{
    hal_gmac_int_disable(chip);
    hal_free_irq(IRQ_GMAC);
}

int hal_gmac_netif_init(struct gmac_chip *chip, phy_interface_t phy_interface_mode, void (*callback)(void *p))
{
    int ret;

    chip->dma_desc_tx = hal_malloc_align(sizeof(*chip->dma_desc_tx) * hal_gmac_dma_desc_num, CACHELINE_LEN);
    if (!chip->dma_desc_tx) {
        ret = -ENOMEM;
        goto err0;
    }
    chip->dma_desc_tx_phy = __va_to_pa((unsigned long)chip->dma_desc_tx);

    chip->dma_desc_rx = hal_malloc_align(sizeof(*chip->dma_desc_rx) * hal_gmac_dma_desc_num, CACHELINE_LEN);
    if (!chip->dma_desc_rx) {
        ret = -ENOMEM;
        goto err1;
    }
    chip->dma_desc_rx_phy = __va_to_pa((unsigned long)chip->dma_desc_rx);

    hal_memset((void *)chip->dma_desc_tx, 0, sizeof(*chip->dma_desc_tx) * hal_gmac_dma_desc_num);
    hal_memset((void *)chip->dma_desc_rx, 0, sizeof(*chip->dma_desc_rx) * hal_gmac_dma_desc_num);
    hal_dcache_clean(chip->dma_desc_tx, sizeof(*chip->dma_desc_tx) * hal_gmac_dma_desc_num);
    hal_dcache_clean(chip->dma_desc_rx, sizeof(*chip->dma_desc_rx) * hal_gmac_dma_desc_num);

    chip->rx_buf = hal_malloc_align(sizeof(unsigned char *) * hal_gmac_dma_desc_num, CACHELINE_LEN);
    if (!chip->rx_buf) {
        ret = -ENOMEM;
        goto err2;
    }
    hal_memset((void *)chip->rx_buf, 0, sizeof(unsigned char *) * hal_gmac_dma_desc_num);
    hal_dcache_clean(chip->rx_buf, sizeof(unsigned char *) * hal_gmac_dma_desc_num);

    chip->tx_lock = (hal_spinlock_t)HAL_SPINLOCK_INIT;
    hal_spin_lock_init(&chip->tx_lock);

    // 获取寄存器基地址，时钟信息
    chip->phy_interface = phy_interface_mode;
    ret = hal_gmac_resource_get(chip);
    if (ret)
        goto err3;

    // 初始化时钟
    ret = hal_gmac_hardware_init(chip);
    if (ret)
        goto err4;

    // 初始化mac
    ret = hal_gmac_init(chip);
    if (ret)
        goto err5;

    // 注册中断
    ret = hal_request_irq(IRQ_GMAC, hal_gmac_irq_handler, "gmac", chip);
    if (ret)
        return ret;
    hal_enable_irq(IRQ_GMAC);

    // 注册接收中断回调
    chip->callback = callback;

    return 0;

err5:
    hal_gmac_hardware_exit(chip);
err4:
    hal_gmac_resource_put(chip);
err3:
    hal_free_align(chip->rx_buf);
err2:
    hal_free_align(chip->dma_desc_rx);
err1:
    hal_free_align(chip->dma_desc_tx);
err0:
    return ret;
}

void hal_gmac_netif_exit(struct gmac_chip *chip)
{
    hal_gmac_disable_irq(chip);
    hal_gmac_hardware_exit(chip);
    hal_gmac_resource_put(chip);
    hal_free_align(chip->rx_buf);
    hal_free_align(chip->dma_desc_rx);
    hal_free_align(chip->dma_desc_tx);
}


