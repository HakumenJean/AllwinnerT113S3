/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#ifndef __DRV_CLOCK_H__
#define __DRV_CLOCK_H__

#define HOSC_24M                        (24000000U)
#define CLK_32K                         (32678U)
#define CLK_16M_RC                      (16000000U)

void sdelay(uint32_t loops);

int rt_hw_clock_init(void);

int rt_hw_periph1x_clk_get(void);
int rt_hw_periph2x_clk_get(void);
int rt_hw_periph800M_clk_get(void);
int rt_hw_cpux_axi_clk_get(void);
int rt_hw_ddr_clk_get(void);
int rt_hw_ahb_clk_get(void);
int rt_hw_apb0_clk_get(void);
int rt_hw_apb1_clk_get(void);
int rt_hw_video0_4X_clk_get(void);
int rt_hw_video0_2X_clk_get(void);
int rt_hw_video0_1X_clk_get(void);
int rt_hw_video1_4X_clk_get(void);
int rt_hw_video1_2X_clk_get(void);
int rt_hw_video1_1X_clk_get(void);
int rt_hw_audio0_4X_clk_get(void);
int rt_hw_audio0_2X_clk_get(void);
int rt_hw_audio0_1X_clk_get(void);
int rt_hw_audio1_clk_get(void);
int rt_hw_audio1_div2_clk_get(void);
int rt_hw_audio1_div5_clk_get(void);

#endif
