/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#ifndef DRIVERS_HAL_RTC_HAL_RTC_H_
#define DRIVERS_HAL_RTC_HAL_RTC_H_

#include "hal_common.h"
#include "hal_clk.h"
#include "hal_reset.h"
#include "platform/rtc_sun20iw1.h"

/*
 * Time unit conversions
 */
#define SEC_IN_MIN              60
#define SEC_IN_HOUR             (60 * SEC_IN_MIN)
#define SEC_IN_DAY              (24 * SEC_IN_HOUR)

/*
 * The year parameter passed to the driver is usually an offset relative to
 * the year 1900. This macro is used to convert this offset to another one
 * relative to the minimum year allowed by the hardware.
 */
#define SUNXI_YEAR_OFF(x)           ((x)->min - 1900)

#define EFEX_FLAG  (0x5AA5A55A)
#define RTC_FEL_INDEX  2
#define RTC_BOOT_INDEX 6
#define RTC_LOG_LEVEL_INDEX 5

/* debug */
#define SUNXI_DEBUG_MODE_FLAG           (0x59)
/* efex */
#define SUNXI_EFEX_CMD_FLAG             (0x5A)
/* boot-resignature */
#define SUNXI_BOOT_RESIGNATURE_FLAG     (0x5B)
/* recovery or boot-recovery */
#define SUNXI_BOOT_RECOVERY_FLAG        (0x5C)
/* sysrecovery */
#define SUNXI_SYS_RECOVERY_FLAG         (0x5D)
/* usb-recovery*/
#define SUNXI_USB_RECOVERY_FLAG         (0x5E)
/* bootloader */
#define SUNXI_FASTBOOT_FLAG             (0x5F)
/* uboot */
#define SUNXI_UBOOT_FLAG                (0x60)

#define SUNXI_MASK_DH               0x0000001f
#define SUNXI_MASK_SM               0x0000003f
#define SUNXI_MASK_M                0x0000000f
#define SUNXI_MASK_LY               0x00000001
#define SUNXI_MASK_D                0x00000ffe

#define SUNXI_GET(x, mask, shift)       (((x) & ((mask) << (shift))) \
        >> (shift))

#define SUNXI_SET(x, mask, shift)       (((x) & (mask)) << (shift))

/*
 * Get date values
 */
#define SUNXI_DATE_GET_DAY_VALUE(x)     SUNXI_GET(x, SUNXI_MASK_DH, 0)
#define SUNXI_DATE_GET_MON_VALUE(x)     SUNXI_GET(x, SUNXI_MASK_M, 8)
#define SUNXI_DATE_GET_YEAR_VALUE(x, d) SUNXI_GET(x, (d)->mask, (d)->yshift)

/*
 * Get time values
 */
#define SUNXI_TIME_GET_SEC_VALUE(x)     SUNXI_GET(x, SUNXI_MASK_SM, 0)
#define SUNXI_TIME_GET_MIN_VALUE(x)     SUNXI_GET(x, SUNXI_MASK_SM, 8)
#define SUNXI_TIME_GET_HOUR_VALUE(x)    SUNXI_GET(x, SUNXI_MASK_DH, 16)

/*
 * Get alarm values
 */
#define SUNXI_ALRM_GET_SEC_VALUE(x)     SUNXI_GET(x, SUNXI_MASK_SM, 0)
#define SUNXI_ALRM_GET_MIN_VALUE(x)     SUNXI_GET(x, SUNXI_MASK_SM, 8)
#define SUNXI_ALRM_GET_HOUR_VALUE(x)    SUNXI_GET(x, SUNXI_MASK_DH, 16)

/*
 * Set date values
 */
#define SUNXI_DATE_SET_DAY_VALUE(x)     SUNXI_DATE_GET_DAY_VALUE(x)
#define SUNXI_DATE_SET_MON_VALUE(x)     SUNXI_SET(x, SUNXI_MASK_M, 8)
#define SUNXI_DATE_SET_YEAR_VALUE(x, d) SUNXI_SET(x, (d)->mask, (d)->yshift)
#define SUNXI_LEAP_SET_VALUE(x, shift)  SUNXI_SET(x, SUNXI_MASK_LY, shift)

/*
 * Set time values
 */
#define SUNXI_TIME_SET_SEC_VALUE(x)     SUNXI_TIME_GET_SEC_VALUE(x)
#define SUNXI_TIME_SET_MIN_VALUE(x)     SUNXI_SET(x, SUNXI_MASK_SM, 8)
#define SUNXI_TIME_SET_HOUR_VALUE(x)    SUNXI_SET(x, SUNXI_MASK_DH, 16)

/*
 * Set alarm values
 */
#define SUNXI_ALRM_SET_SEC_VALUE(x)     SUNXI_ALRM_GET_SEC_VALUE(x)
#define SUNXI_ALRM_SET_MIN_VALUE(x)     SUNXI_SET(x, SUNXI_MASK_SM, 8)
#define SUNXI_ALRM_SET_HOUR_VALUE(x)    SUNXI_SET(x, SUNXI_MASK_DH, 16)
#define SUNXI_ALRM_SET_DAY_VALUE(x)     SUNXI_SET(x, SUNXI_MASK_D, 21)

typedef int (*rtc_callback_t)(void);
/*
 * min and max year are arbitrary set considering the limited range of the
 * hardware register field
 */
struct hal_rtc_data_year
{
    unsigned int min;       /* min year allowed */
    unsigned int max;       /* max year allowed */
    unsigned int mask;      /* mask for the year field */
    unsigned int yshift;        /* bit shift to get the year */
    unsigned char leap_shift;   /* bit shift to get the leap year */
};

struct hal_rtc_dev
{
    struct hal_rtc_data_year *data_year;
    rtc_callback_t user_callback;
    unsigned long base;
    int irq;
    hal_clk_t bus_clk;
    hal_clk_t rtc1k_clk;
    hal_clk_t rtcspi_clk;
    struct reset_control *reset;

};

typedef enum
{
    RTC_GET_TIME = 0,
    RTC_SET_TIME = 1,
    RTC_GET_ALARM = 2,
    RTC_SET_ALARM = 3,
    RTC_CALLBACK = 4,
    RTC_IRQENABLE = 5
} hal_rtc_transfer_cmd_t;

/*
 * The struct used to pass data via the following ioctl. Similar to the
 * struct tm in <time.h>, but it needs to be here so that the kernel
 * source is self contained, allowing cross-compiles, etc. etc.
 */

struct rtc_time
{
    int tm_sec;
    int tm_min;
    int tm_hour;
    int tm_mday;
    int tm_mon;
    int tm_year;
    int tm_wday;
    int tm_yday;
    int tm_isdst;
};

typedef s64 time64_t;
/*
 * This data structure is inspired by the EFI (v0.92) wakeup
 * alarm API.
 */
struct rtc_wkalrm
{
    unsigned char enabled;  /* 0 = alarm disabled, 1 = alarm enabled */
    unsigned char pending;  /* 0 = alarm not pending, 1 = alarm pending */
    struct rtc_time time;   /* time the alarm is set to */
};

typedef enum
{
    RTC_IRQ_ERROR = -3,
    RTC_CLK_ERROR = -2,
    RTC_ERROR = -1,
    RTC_OK = 0,
}hal_rtc_status_t;

int rtc_month_days(unsigned int month, unsigned int year);
int rtc_year_days(unsigned int day, unsigned int month, unsigned int year);
int rtc_valid_tm(struct rtc_time *tm);
time64_t rtc_tm_to_time64(struct rtc_time *tm);
void rtc_time64_to_tm(time64_t time, struct rtc_time *tm);

inline time64_t rtc_tm_sub(struct rtc_time *lhs, struct rtc_time *rhs);
inline int is_leap_year(unsigned int year);
inline void rtc_time_to_tm(unsigned long time, struct rtc_time *tm);
inline int rtc_tm_to_time(struct rtc_time *tm, unsigned long *time);

void hal_rtc_set_fel_flag(void);
u32  hal_rtc_probe_fel_flag(void);
void hal_rtc_clear_fel_flag(void);
int hal_rtc_get_bootmode_flag(void);
int hal_rtc_set_bootmode_flag(u8 flag);
void hal_rtc_write_data(int index, u32 val);
u32  hal_rtc_read_data(int index);
int hal_rtc_gettime(struct rtc_time *rtc_tm);
int hal_rtc_settime(struct rtc_time *rtc_tm);
int hal_rtc_getalarm(struct rtc_wkalrm *wkalrm);
int hal_rtc_setalarm(struct rtc_wkalrm *wkalrm);
int hal_rtc_alarm_irq_enable(unsigned int enabled);
void hal_rtc_min_year_show(unsigned int *min);
void hal_rtc_max_year_show(unsigned int *max);
int hal_rtc_register_callback(rtc_callback_t user_callback);
int hal_rtc_init(void);
int hal_rtc_deinit(void);

#endif /* DRIVERS_HAL_RTC_HAL_RTC_H_ */
