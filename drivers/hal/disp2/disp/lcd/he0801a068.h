/* drivers/video/sunxi/disp2/disp/lcd/he0801a068.h
 *
 * Copyright (c) 2017 Allwinnertech Co., Ltd.
 * Author: zhengxiaobin <zhengxiaobin@allwinnertech.com>
 *
 * he0801a-068 panel driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef DRIVERS_HAL_DISP2_DISP_LCD_HE0801A068_H_
#define DRIVERS_HAL_DISP2_DISP_LCD_HE0801A068_H_

#include "panels.h"

extern struct __lcd_panel he0801a068_panel;

extern s32 bsp_disp_get_panel_info(u32 screen_id, struct disp_panel_para *info);

#endif /* DRIVERS_HAL_DISP2_DISP_LCD_HE0801A068_H_ */
