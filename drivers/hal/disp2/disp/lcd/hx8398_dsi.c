/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "hx8398_dsi.h"

static void lcd_power_on(u32 sel);
static void lcd_power_off(u32 sel);
static void lcd_bl_open(u32 sel);
static void lcd_bl_close(u32 sel);

static void lcd_panel_init(u32 sel);
static void lcd_panel_exit(u32 sel);

#define panel_reset(sel, val) sunxi_lcd_gpio_set_value(sel, 0, val)

static void lcd_cfg_panel_info(struct panel_extend_para *info)
{
    u32 i = 0, j = 0;
    u32 items;
    u8 lcd_gamma_tbl[][2] = {
        {0, 0},
        {15, 15},
        {30, 30},
        {45, 45},
        {60, 60},
        {75, 75},
        {90, 90},
        {105, 105},
        {120, 120},
        {135, 135},
        {150, 150},
        {165, 165},
        {180, 180},
        {195, 195},
        {210, 210},
        {225, 225},
        {240, 240},
        {255, 255},
    };

    u32 lcd_cmap_tbl[2][3][4] = {
    {
        {LCD_CMAP_G0, LCD_CMAP_B1, LCD_CMAP_G2, LCD_CMAP_B3},
        {LCD_CMAP_B0, LCD_CMAP_R1, LCD_CMAP_B2, LCD_CMAP_R3},
        {LCD_CMAP_R0, LCD_CMAP_G1, LCD_CMAP_R2, LCD_CMAP_G3},
        },
        {
        {LCD_CMAP_B3, LCD_CMAP_G2, LCD_CMAP_B1, LCD_CMAP_G0},
        {LCD_CMAP_R3, LCD_CMAP_B2, LCD_CMAP_R1, LCD_CMAP_B0},
        {LCD_CMAP_G3, LCD_CMAP_R2, LCD_CMAP_G1, LCD_CMAP_R0},
        },
    };

    items = sizeof(lcd_gamma_tbl) / 2;
    for (i = 0; i < items - 1; i++) {
        u32 num = lcd_gamma_tbl[i+1][0] - lcd_gamma_tbl[i][0];

        for (j = 0; j < num; j++) {
            u32 value = 0;

            value = lcd_gamma_tbl[i][1] +
                ((lcd_gamma_tbl[i+1][1] - lcd_gamma_tbl[i][1])
                * j) / num;
            info->lcd_gamma_tbl[lcd_gamma_tbl[i][0] + j] =
                            (value<<16)
                            + (value<<8) + value;
        }
    }
    info->lcd_gamma_tbl[255] = (lcd_gamma_tbl[items-1][1]<<16) +
                    (lcd_gamma_tbl[items-1][1]<<8)
                    + lcd_gamma_tbl[items-1][1];

    hal_memcpy(info->lcd_cmap_tbl, lcd_cmap_tbl, sizeof(lcd_cmap_tbl));

}

static s32 lcd_open_flow(u32 sel)
{
    LCD_OPEN_FUNC(sel, lcd_power_on, 30);
    LCD_OPEN_FUNC(sel, lcd_panel_init, 50);
    LCD_OPEN_FUNC(sel, sunxi_lcd_tcon_enable, 100);
    LCD_OPEN_FUNC(sel, lcd_bl_open, 0);

    return 0;
}

static s32 lcd_close_flow(u32 sel)
{
    LCD_CLOSE_FUNC(sel, lcd_bl_close, 0);
    LCD_CLOSE_FUNC(sel, sunxi_lcd_tcon_disable, 0);
    LCD_CLOSE_FUNC(sel, lcd_panel_exit, 200);
    LCD_CLOSE_FUNC(sel, lcd_power_off, 500);

    return 0;
}

static void lcd_power_on(u32 sel)
{
    sunxi_lcd_power_enable(sel, 0);
    sunxi_lcd_delay_ms(10);
    sunxi_lcd_power_enable(sel, 1);
    sunxi_lcd_delay_ms(10);
    sunxi_lcd_pin_cfg(sel, 1);
    sunxi_lcd_delay_ms(50);
    panel_reset(sel, 1);
    sunxi_lcd_delay_ms(100);
    panel_reset(sel, 0);
    sunxi_lcd_delay_ms(100);
    panel_reset(sel, 1);
    sunxi_lcd_delay_ms(100);

}

static void lcd_power_off(u32 sel)
{
    sunxi_lcd_pin_cfg(sel, 0);
    sunxi_lcd_delay_ms(20);
    panel_reset(sel, 0);
    sunxi_lcd_delay_ms(5);
    sunxi_lcd_power_disable(sel, 1);
    sunxi_lcd_delay_ms(5);
    sunxi_lcd_power_disable(sel, 0);
}

static void lcd_bl_open(u32 sel)
{
    sunxi_lcd_pwm_enable(sel);
    sunxi_lcd_backlight_enable(sel);
}

static void lcd_bl_close(u32 sel)
{
    /* config lcd_bl_en pin to close lcd backlight */
    sunxi_lcd_backlight_disable(sel);
    sunxi_lcd_pwm_disable(sel);
}

#define REGFLAG_DELAY 0XFE
#define REGFLAG_END_OF_TABLE 0xFF /* END OF REGISTERS MARKER */

struct LCM_setting_table {
    u8 cmd;
    u32 count;
    u8 para_list[64];
};

/*add panel initialization code*/
static struct LCM_setting_table LCM_HX8398_setting[] = {
    {0xB9, 0x03, {0xFF, 0x83, 0x89} },
    {0xBA, 0x07, {0x41, 0x93, 0x00, 0x16, 0xA4, 0x00, 0x18} },
    {0xB1, 0x13, {0x00, 0x00, 0x07, 0xF6, 0x50, 0x10, 0x11, 0xF8, 0xF7, 0x36, 0x3E, 0x3F, 0x3F, 0x42, 0x01, 0x3A, 0xF7, 0x00, 0xE6} },
    {0xB2, 0x07, {0x00, 0x00, 0x78, 0x0E, 0x12, 0x3F, 0xC0} },
    {0xB4, 0x17, {0x80, 0x14, 0x00, 0x32, 0x10, 0x05, 0x43, 0x13, 0xD3, 0x32, 0x10, 0x00, 0x47, 0x43, 0x44, 0x07, 0x47, 0x43, 0x44, 0x14, 0xFF, 0xFF, 0x0A} },
    {0xD5, 0x30, {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x60, 0x00, 0x99, 0x88, 0x88, 0x88, 0x88, 0x23, 0x88, 0x01, 0x88, 0x67, 0x88, 0x45, 0x01, 0x23,
                  0x23, 0x88, 0x88, 0x88, 0x88, 0x88, 0x99, 0x88, 0x88, 0x88, 0x54, 0x88, 0x76, 0x88, 0x10, 0x88, 0x32, 0x32, 0x10, 0x88, 0x88, 0x88, 0x88, 0x88} },
    {0xB6, 0x04, {0x00, 0x90, 0x00, 0x90} },
    {0XE0, 0X22, {0X01, 0X1F, 0X20, 0X2C, 0X2B, 0X2B, 0X31, 0X4B, 0X07, 0X0E, 0X0F, 0X14, 0X16, 0X14, 0X16, 0X12, 0X1E, 0X01, 0X1F, 0X20, 0X2C, 0X2B, 0X2B, 0X31,
                  0X4B, 0X07, 0X0E, 0X0F, 0X14, 0X16, 0X14, 0X16, 0X12, 0X1E} },
    {0xCC, 0x01, {0x02} },
    {0x3A, 0x01, {0x77} },
    {0x11, 0x01, {0x00} },// Sleep Out
    {REGFLAG_DELAY, REGFLAG_DELAY, {120} },
    {0x29, 0x01, {0x00} },// Display On
    {REGFLAG_END_OF_TABLE, REGFLAG_END_OF_TABLE, {} }
};

static void lcd_panel_init(u32 sel)
{
    sunxi_lcd_dsi_clk_enable(sel);
    sunxi_lcd_delay_ms(20);

    rt_kprintf("panel init!\r\n");

    for(int i = 0; ; i++)
    {
        if(LCM_HX8398_setting[i].cmd == REGFLAG_END_OF_TABLE)
            break;
        else if(LCM_HX8398_setting[i].cmd == REGFLAG_DELAY)
            sunxi_lcd_delay_ms(LCM_HX8398_setting[i].para_list[0]);
#ifdef SUPPORT_DSI
        else
            sunxi_lcd_dsi_dcs_write(sel, LCM_HX8398_setting[i].cmd, LCM_HX8398_setting[i].para_list, LCM_HX8398_setting[i].count);
#endif
    }
}

static void lcd_panel_exit(u32 sel)
{
    sunxi_lcd_dsi_dcs_write_0para(sel, DSI_DCS_SET_DISPLAY_OFF);
    sunxi_lcd_delay_ms(20);
    sunxi_lcd_dsi_dcs_write_0para(sel, DSI_DCS_ENTER_SLEEP_MODE);
    sunxi_lcd_delay_ms(80);
}

/* sel: 0:lcd0; 1:lcd1 */
static s32 lcd_user_defined_func(u32 sel, u32 para1, u32 para2, u32 para3)
{
    return 0;
}

/* sel: 0:lcd0; 1:lcd1 */
/*static s32 LCD_set_bright(u32 sel, u32 bright)*/
/*{*/
    /*sunxi_lcd_dsi_dcs_write_1para(sel,0x51,bright);*/
    /*return 0;*/
/*}*/

struct __lcd_panel hx8398_dsi_panel = {
    /* panel driver name, must mach the name of
     * lcd_drv_name in sys_config.fex
    */
    .name = "hx8398_dsi",
    .func = {
    .cfg_panel_info = lcd_cfg_panel_info,
    .cfg_open_flow = lcd_open_flow,
    .cfg_close_flow = lcd_close_flow,
    .lcd_user_defined_func = lcd_user_defined_func,
    /*.set_bright = LCD_set_bright, */
    },
};
