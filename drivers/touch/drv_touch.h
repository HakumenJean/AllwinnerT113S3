/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#ifndef DRIVERS_TOUCH_DRV_TOUCH_H_
#define DRIVERS_TOUCH_DRV_TOUCH_H_

#ifdef RT_USING_TOUCH

#define RT_TOUCH_USE_IRQ 1
#define RT_TOUCH_CTRL_SET_CALIBRATION        (11)

struct tscal_t {
    rt_int32_t x[5], xfb[5];
    rt_int32_t y[5], yfb[5];
    rt_int32_t a[7];
};

#ifdef BSP_USING_TOUCH_GT9XX

void rt_touch_gt911_init(void);
void get_gt911_touch_data(struct rt_touch_data *touch_data);

#endif

#ifdef BSP_USING_TOUCH_TP

void rt_touch_tp_init(void);

#endif

int perform_calibration(struct tscal_t * cal);

#endif

#endif /* DRIVERS_TOUCH_DRV_TOUCH_H_ */
