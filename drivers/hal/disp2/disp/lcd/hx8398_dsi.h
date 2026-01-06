/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-08-25     Admin       the first version
 */
#ifndef DRIVERS_HAL_DISP2_DISP_LCD_HX8398_DSI_H_
#define DRIVERS_HAL_DISP2_DISP_LCD_HX8398_DSI_H_

#include "panels.h"

extern struct __lcd_panel hx8398_dsi_panel;

extern s32 bsp_disp_get_panel_info(u32 screen_id, struct disp_panel_para *info);

#endif /* DRIVERS_HAL_DISP2_DISP_LCD_HX8398_DSI_H_ */
