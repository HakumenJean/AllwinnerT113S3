/*
 * Copyright 2018 AllWinnertech  Co., Ltd
 * wangwei@allwinnertech.com
 */

#ifndef DRIVERS_HAL_TPADC_HAL_TPADC_H_
#define DRIVERS_HAL_TPADC_HAL_TPADC_H_

#include "hal_clk.h"
#include "hal_reset.h"
#include "platform/tpadc_sun8iw20.h"
#include "common_tpadc.h"

typedef enum
{
    TPADC_IRQ_ERROR = -4,
    TPADC_CHANNEL_ERROR = -3,
    TPADC_CLK_ERROR = -2,
    TPADC_ERROR = -1,
    TPADC_OK = 0,
} hal_tpadc_status_t;

typedef enum
{
    DATA_XY,
    DATA_UP,
} data_flag_t;

typedef enum
{
    TP_CH_0 = 0,
    TP_CH_1,
    TP_CH_2,
    TP_CH_3,
    TP_CH_MAX,
} tp_channel_id;

typedef void (*tpadc_usercallback_t)(void *param, uint32_t x_data, uint32_t y_data, data_flag_t flag);
typedef void (*tpadc_adc_usercallback_t)(void *param, uint32_t data, tp_channel_id channel);

typedef struct hal_tpadc
{
    unsigned long reg_base;
    uint32_t channel_num;
    uint32_t irq_num;
    uint32_t rate;
    hal_clk_id_t bus_clk_id;
    hal_clk_id_t mod_clk_id;
    hal_reset_id_t rst_clk_id;
    hal_clk_t   bus_clk;
    hal_clk_t   mod_clk;
    struct reset_control    *rst_clk;
    tpadc_usercallback_t callback;
    void *param;
    tpadc_adc_usercallback_t adc_callback[TP_CH_MAX];
    void *adc_param[TP_CH_MAX];
} hal_tpadc_t;

hal_tpadc_status_t hal_tpadc_init(void);
hal_tpadc_status_t hal_tpadc_exit(void);
hal_tpadc_status_t hal_tpadc_register_callback(tpadc_usercallback_t user_callback, void *param);

hal_tpadc_status_t hal_tpadc_adc_init(void);
hal_tpadc_status_t hal_tpadc_adc_channel_init(tp_channel_id channel);
hal_tpadc_status_t hal_tpadc_adc_channel_exit(tp_channel_id channel);
hal_tpadc_status_t hal_tpadc_adc_exit(void);
hal_tpadc_status_t hal_tpadc_adc_register_callback(tp_channel_id channel , tpadc_adc_usercallback_t user_callback, void *adc_param);

hal_tpadc_status_t hal_tpadc_resume(void);
hal_tpadc_status_t hal_tpadc_suspend(void);

#endif /* DRIVERS_HAL_TPADC_HAL_TPADC_H_ */
