/* SPDX-License-Identifier: GPL-2.0 */

#ifndef DRIVERS_HAL_COMMON_HAL_INTERRUPT_H_
#define DRIVERS_HAL_COMMON_HAL_INTERRUPT_H_

#define SUNXI_IRQ_UART0              (34)
#define SUNXI_IRQ_UART1              (35)
#define SUNXI_IRQ_UART2              (36)
#define SUNXI_IRQ_UART3              (37)
#define SUNXI_IRQ_UART4              (38)
#define SUNXI_IRQ_UART5              (39)
#define SUNXI_IRQ_TWI0               (41)
#define SUNXI_IRQ_TWI1               (42)
#define SUNXI_IRQ_TWI2               (43)
#define SUNXI_IRQ_TWI3               (44)
#define SUNXI_IRQ_SPI0               (47)
#define SUNXI_IRQ_SPI1               (48)
#define SUNXI_IRQ_PWM                (50)
#define SUNXI_IRQ_IR_TX              (51)
#define SUNXI_IRQ_LEDC               (52)
#define SUNXI_IRQ_OWA                (55)
#define SUNXI_IRQ_DMIC               (56)
#define SUNXI_IRQ_AUDIO_CODEC        (57)
#define SUNXI_IRQ_I2S0               (58)
#define SUNXI_IRQ_I2S1               (59)
#define SUNXI_IRQ_I2S2               (60)
#define SUNXI_IRQ_USB0_DEVICE        (61)
#define SUNXI_IRQ_USB0_EHCI          (62)
#define SUNXI_IRQ_USB0_OHCI          (63)
#define SUNXI_IRQ_USB1_EHCI          (65)
#define SUNXI_IRQ_USB1_OHCI          (66)
#define SUNXI_IRQ_SMHC0              (72)
#define SUNXI_IRQ_SMHC1              (73)
#define SUNXI_IRQ_SMHC2              (74)
#define SUNXI_IRQ_MSI                (75)
#define SUNXI_IRQ_SMC                (76)
#define SUNXI_IRQ_EMAC               (78)
#define SUNXI_IRQ_TZMA_ERR           (79)
#define SUNXI_IRQ_ECCU_FERR          (80)
#define SUNXI_IRQ_AHB_TIMEOUT        (81)
#define SUNXI_IRQ_DMAC_NS            (82)
#define SUNXI_IRQ_DMAC_S             (83)
#define SUNXI_IRQ_CE_NS              (84)
#define SUNXI_IRQ_CE_S               (85)
#define SUNXI_IRQ_SPINLOCK           (86)
#define SUNXI_IRQ_HSTIME0            (87)
#define SUNXI_IRQ_HSTIME1            (88)
#define SUNXI_IRQ_GPADC              (89)
#define SUNXI_IRQ_THS                (90)
#define SUNXI_IRQ_TIMER0             (91)
#define SUNXI_IRQ_TIMER1             (92)
#define SUNXI_IRQ_LRADC              (93)
#define SUNXI_IRQ_TPADC              (94)
#define SUNXI_IRQ_WATCHDOG           (95)
#define SUNXI_IRQ_IOMMU              (96)
#define SUNXI_IRQ_VE                 (98)
#define SUNXI_IRQ_GPIOB_NS           (101)
#define SUNXI_IRQ_GPIOB_S            (102)
#define SUNXI_IRQ_GPIOC_NS           (103)
#define SUNXI_IRQ_GPIOC_S            (104)
#define SUNXI_IRQ_GPIOD_NS           (105)
#define SUNXI_IRQ_GPIOD_S            (106)
#define SUNXI_IRQ_GPIOE_NS           (107)
#define SUNXI_IRQ_GPIOE_S            (108)
#define SUNXI_IRQ_GPIOF_NS           (109)
#define SUNXI_IRQ_GPIOF_S            (110)
#define SUNXI_IRQ_GPIOG_NS           (111)
#define SUNXI_IRQ_DE                 (119)
#define SUNXI_IRQ_DI                 (120)
#define SUNXI_IRQ_G2D                (121)
#define SUNXI_IRQ_LCD                (122)
#define SUNXI_IRQ_TV                 (123)
#define SUNXI_IRQ_DSI                (124)
#define SUNXI_IRQ_CSI_DMA0           (127)
#define SUNXI_IRQ_CSI_DMA1           (128)
#define SUNXI_IRQ_CSI_PARSER0        (132)
#define SUNXI_IRQ_CSI_TOP_PKT        (138)
#define SUNXI_IRQ_TVD                (139)
#define SUNXI_IRQ_DSP_DFE            (152)
#define SUNXI_IRQ_DSP_PFE            (153)
#define SUNXI_IRQ_DSP_WDG            (154)
#define SUNXI_IRQ_DSP_MBOX_RISCV_W   (155)
#define SUNXI_IRQ_DSP_TZMA           (157)
#define SUNXI_IRQ_NMI                (168)
#define SUNXI_IRQ_PPU                (169)
#define SUNXI_IRQ_TWD                (170)
#define SUNXI_IRQ_TIMER0_X           (172)
#define SUNXI_IRQ_TIMER1_X           (173)
#define SUNXI_IRQ_TIMER2_X           (174)
#define SUNXI_IRQ_TIMER3_X           (175)
#define SUNXI_IRQ_ALARM0             (176)
#define SUNXI_IRQ_IRRX               (183)
#define SUNXI_IRQ_C0_CTI0            (192)
#define SUNXI_IRQ_C0_CTI1            (193)
#define SUNXI_IRQ_C0_COMMTX0         (196)
#define SUNXI_IRQ_C0_COMMTX1         (197)
#define SUNXI_IRQ_C0_COMMRX0         (200)
#define SUNXI_IRQ_C0_COMMRX1         (201)
#define SUNXI_IRQ_C0_PMU0            (204)
#define SUNXI_IRQ_C0_PMU1            (205)
#define SUNXI_IRQ_C0_AXI_ERROR       (208)
#define SUNXI_IRQ_AXI_WR_IRQ         (210)
#define SUNXI_IRQ_AXI_RD_IRQ         (211)
#define SUNXI_IRQ_DBGWRUPREQ_OUT0    (212)
#define SUNXI_IRQ_DBGWRUPREQ_OUT1    (213)

#define SUNXI_IRQ_COUNT              224

/* the maximum entries of the exception table */
#define MAX_HANDLERS                SUNXI_IRQ_COUNT
/* number of interrupts on board */
#define ARM_GIC_NR_IRQS             SUNXI_IRQ_COUNT
/* only one GIC available */
#define ARM_GIC_MAX_NR              1

#define GIC_IRQ_START               0

#define GIC_ACK_INTID_MASK          0x000003FF

#define GIC400_BASE_ADDR                (0x03020000U)
#define GIC_DISTRIBUTOR_BASE_ADDR       (0x03021000U)
#define GIC_INTERFACE_BASE_ADDR         (0x03022000U)
#define GICVSELF_BASE_ADDR              (0x03024000U)
#define GICV_BASE_ADDR                  (0x03025000U)

typedef rt_isr_handler_t hal_isr_handler_t;

uint32_t platform_get_gic_dist_base(void);
uint32_t platform_get_gic_cpu_base(void);

int32_t hal_request_irq(int32_t irq, hal_isr_handler_t handler, const char *name, void *data);
void hal_free_irq(int32_t irq);
int hal_enable_irq(int32_t irq);
void hal_disable_irq(int32_t irq);

unsigned long hal_interrupt_is_disable(void);
uint32_t hal_interrupt_get_nest(void);

#define in_interrupt(...)       hal_interrupt_get_nest()

//void hal_interrupt_enter(void);
//void hal_interrupt_leave(void);
//
//void hal_interrupt_enable(void);
//void hal_interrupt_disable(void);

unsigned long hal_interrupt_disable_irqsave(void);
void hal_interrupt_enable_irqrestore(unsigned long flag);

#endif /* DRIVERS_HAL_COMMON_HAL_INTERRUPT_H_ */
