/*
 * Copyright 2018 AllWinnertech  Co., Ltd
 * wangwei@allwinnertech.com
 */

#ifndef DRIVERS_HAL_WATCHDOG_HAL_WATCHDOG_H_
#define DRIVERS_HAL_WATCHDOG_HAL_WATCHDOG_H_

#define WDT_BASE                0x020500A0

/* watchdog register offset */
#define WDT_IRQ_EN              (WDT_BASE + 0x00)
#define WDT_STA                 (WDT_BASE + 0x04)
#define WDT_CTL                 (WDT_BASE + 0x10)
#define WDT_CFG                 (WDT_BASE + 0x14)
#define WDT_MODE                (WDT_BASE + 0x18)
//#define WDT_OUT_CFG             (WDT_BASE + 0x1C)

//#define WDT_TIMEOUT             16

#define WDT_CTRL_RESTART        (0x1 << 0)
#define WDT_CTRL_KEY            (0x0a57 << 1)

#define WDT_CFG_RESET           (0x1)
#define WDT_MODE_EN             (0x1)
#define KEY_FIELD_MAGIC         (0x16AA0000)

struct hal_sunxi_wdt {
        volatile u32 irq_en;    //0x00
        volatile u32 sta;       //0x04
        volatile u32 rsl[2];
        volatile u32 ctl;       //0x10
        volatile u32 cfg;       //0x14
        volatile u32 mode;      //0x18
};

void hal_watchdog_disable(void);
void hal_watchdog_reset(int timeout);
void hal_watchdog_restart(void);
void hal_watchdog_info(void);
void hal_watchdog_init(void);
void hal_watchdog_stop(int timeout);
void hal_watchdog_start(int timeout);
void hal_watchdog_feed(void);
int hal_watchdog_suspend(int timeout);
int hal_watchdog_resume(int timeout);
int hal_watchdog_is_running(void);

#endif /* DRIVERS_HAL_WATCHDOG_HAL_WATCHDOG_H_ */
