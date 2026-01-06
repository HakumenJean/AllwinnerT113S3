/*
 * Copyright 2018 AllWinnertech  Co., Ltd
 * wangwei@allwinnertech.com
 */

#ifndef DRIVERS_HAL_GPIO_GPIO_REG_H_
#define DRIVERS_HAL_GPIO_GPIO_REG_H_

#define GPIO_BASE_ADDR                  (0x02000000U)

#define GPIOn_CFG_ADDR(n)               (GPIO_BASE_ADDR + (n) * 0x30 + 0x00)
#define GPIOn_DATA_ADDR(n)              (GPIO_BASE_ADDR + (n) * 0x30 + 0x10)
#define GPIOn_DRV_ADDR(n)               (GPIO_BASE_ADDR + (n) * 0x30 + 0x14)
#define GPIOn_PUL_ADDR(n)               (GPIO_BASE_ADDR + (n) * 0x30 + 0x24);

#define GPIOn_INT_CFG_ADDR(n)           (GPIO_BASE_ADDR + 0x200 + (n) * 0x20 + 0x00)
#define GPIOn_INT_CTRL_ADDR(n)          (GPIO_BASE_ADDR + 0x200 + (n) * 0x20 + 0x10)
#define GPIOn_INT_STA_ADDR(n)           (GPIO_BASE_ADDR + 0x200 + (n) * 0x20 + 0x14)
#define GPIOn_INT_DEB_ADDR(n)           (GPIO_BASE_ADDR + 0x200 + (n) * 0x20 + 0x18)

#define GPIO_PIO_POW_MOD_SEL_REG        (0x0340)
#define GPIO_PIO_POW_MS_CTL_REG         (0x0344)
#define GPIO_PIO_POW_VAL_REG            (0x0348)
#define GPIO_PIO_POW_VOL_SEL_CTL_REG    (0x0350)

#endif /* DRIVERS_HAL_GPIO_GPIO_REG_H_ */
