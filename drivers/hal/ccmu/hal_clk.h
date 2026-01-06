/* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
 *
 * Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
 *the the People's Republic of China and other countries.
 * All Allwinner Technology Co.,Ltd. trademarks are used with permission.
 *
 * DISCLAIMER
 * THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
 * IF YOU NEED TO INTEGRATE THIRD PARTY’S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
 * IN ALLWINNERS’SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
 * ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
 * ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
 * COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
 * YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY’S TECHNOLOGY.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY ALLWINNER"AS IS" AND TO THE MAXIMUM EXTENT
 * PERMITTED BY LAW, ALLWINNER EXPRESSLY DISCLAIMS ALL WARRANTIES OF ANY KIND,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING WITHOUT LIMITATION REGARDING
 * THE TITLE, NON-INFRINGEMENT, ACCURACY, CONDITION, COMPLETENESS, PERFORMANCE
 * OR MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * IN NO EVENT SHALL ALLWINNER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DRIVERS_HAL_CCMU_HAL_CLK_H_
#define DRIVERS_HAL_CCMU_HAL_CLK_H_

#include "hal_common.h"
#include "sunxi-ng/clk-fixed-rate.h"
#include "sunxi-ng/ccu-sun8iw20.h"
#include "sunxi-ng/ccu-sun8iw20-r.h"
#include "sunxi-ng/ccu-sun8iw20-rtc.h"

typedef unsigned int hal_clk_id_t;
typedef struct clk *hal_clk_t;

/************************************************************************************************
* Enum hal_clk_status_t
* @Description: This enum defines the return status of Clock APIs. User should check the return value after calling the APIs
*************************************************************************************************/
typedef enum
{
    HAL_CLK_STATUS_DISABLED = -1,
    HAL_CLK_STATUS_ENABLED = 0,
    HAL_CLK_STATUS_ERROR_CLK_FACTOR_REFUSED = -11,
    HAL_CLK_STATUS_ERROR_CLK_NEED_DISABLED  = -10,
    HAL_CLK_STATUS_ERROR_CLK_PARENT_DISABLED  = -9,
    HAL_CLK_STATUS_ERROR_CLK_ENABLED_FAILED  = -8,
    HAL_CLK_STATUS_ERROR_CLK_ROUND_FAILED = -7,
    HAL_CLK_STATUS_ERROR_CLK_SET_RATE_REFUSED = -6,
    HAL_CLK_STATUS_ERROR_CLK_NOT_FOUND  = -5,
    HAL_CLK_STATUS_ERROT_CLK_UNDEFINED  = -4,
    HAL_CLK_STATUS_UNINITIALIZED = -3,        /**< Uninitialized clock driver. */
    HAL_CLK_STATUS_INVALID_PARAMETER = -2,    /**< Invalid parameter. */
    HAL_CLK_STATUS_ERROR = -1,                /**< Unknown error. */
    HAL_CLK_STATUS_OK = 0,                    /**< Successful. */
} hal_clk_status_t;

/************************************************************************************************
* Enum hal_clk_type_t
* @Description: This enum defines the type of Clock
*************************************************************************************************/
typedef enum
{
    HAL_SUNXI_FIXED_CCU = 0,
    HAL_SUNXI_RTC_CCU,
    HAL_SUNXI_CCU,
    HAL_SUNXI_R_CCU,
    HAL_SUNXI_CCU_NUMBER,
} hal_clk_type_t;

/************************************************************************************************
* @Function: hal_clock_init
* @Description: provide HAL API for initialize soc clocks during the system power-on startup phase
* @Parameters:
* # void: No parameters required
* @Return values:
* # HAL_CLK_STATUS_OK: soc clocks initialize successed
* # others : soc clocks initialization may have some abnormal problems
* @Attention: clock initialize timing depands on specific soc platform clock design
*************************************************************************************************/
void hal_clock_init(void);

/************************************************************************************************
* @Function: hal_clock_init
* @Description: provide HAL API for initialize soc clocks during the system power-on startup phase
* @Parameters:
* # void: No parameters required
* @Return values:
* # HAL_CLK_STATUS_OK: soc clocks initialize successed
* # others : soc clocks initialization may have some abnormal problems
* @Attention: clock initialize timing depands on specific soc platform clock design
*************************************************************************************************/
hal_clk_t hal_clock_get(hal_clk_type_t type, hal_clk_id_t id);

hal_clk_status_t hal_clock_put(hal_clk_t clk);

/************************************************************************************************
* @Function: hal_clock_is_enabled
* @Description: provide HAL API for bus-clk and periph-clk to get clock enabled statue
* @Parameters:
* # clk: clock-id of soc specific clock
* @Return values:
* # HAL_CLK_STATUS_INVALID_PARAMETER: input parameter of clock-id undefined in hal ot rate value is invalid
* # HAL_CLK_STATUS_ERROR_CLK_SET_RATE_REFUSED: fixed-clk and factor clk not allowed User to change rate because of stability
* # HAL_CLK_STATUS_ERROT_CLK_UNDEFINED: input parameter of clock-id defined in hal but not defined by soc clock driver
* # HAL_CLK_STATUS_ERROR_CLK_NOT_FOUND: input parameter of clock-id defined in hal but not defined by soc clock driver
* # HAL_CLK_STATUS_ENABLED: clock current status is enabled
* # HAL_CLK_STATUS_DISABLED: clock current status is disabled
* @Attention: .etc
*************************************************************************************************/
hal_clk_status_t hal_clock_is_enabled(hal_clk_t clk);

/************************************************************************************************
* @Function: hal_clock_enable
* @Description: provide HAL API for bus-clk and periph-clk to enable clock
* @Parameters:
* # clk: clock-id of soc specific clock
* @Return values:
* # HAL_CLK_STATUS_INVALID_PARAMETER: input parameter of clock-id undefined in hal ot rate value is invalid
* # HAL_CLK_STATUS_ERROR_CLK_SET_RATE_REFUSED: fixed-clk and factor clk not allowed User to change rate because of stability
* # HAL_CLK_STATUS_ERROT_CLK_UNDEFINED: input parameter of clock-id defined in hal but not defined by soc clock driver
* # HAL_CLK_STATUS_ERROR_CLK_NOT_FOUND: input parameter of clock-id defined in hal but not defined by soc clock driver
* # HAL_CLK_STATUS_ENABLED: clock current status is enabled
* # HAL_CLK_STATUS_DISABLED: clock current status is disabled
* @Attention: .etc
*************************************************************************************************/
hal_clk_status_t hal_clock_enable(hal_clk_t clk);

/************************************************************************************************
* @Function: hal_clock_disable
* @Description: provide HAL API for bus-clk and periph-clk to disable clock
* @Parameters:
* # clk: clock-id of soc specific clock
* @Return values:
* # HAL_CLK_STATUS_INVALID_PARAMETER: input parameter of clock-id undefined in hal ot rate value is invalid
* # HAL_CLK_STATUS_ERROR_CLK_SET_RATE_REFUSED: fixed-clk and factor clk not allowed User to change rate because of stability
* # HAL_CLK_STATUS_ERROT_CLK_UNDEFINED: input parameter of clock-id defined in hal but not defined by soc clock driver
* # HAL_CLK_STATUS_ERROR_CLK_NOT_FOUND: input parameter of clock-id defined in hal but not defined by soc clock driver
* # HAL_CLK_STATUS_OK: clock current status disabled successed
* @Attention: .etc
*************************************************************************************************/
hal_clk_status_t hal_clock_disable(hal_clk_t clk);

/************************************************************************************************
* @Function: hal_clk_recalc_rate
* @Description: provide HAL API for factor-clk, bus-clk and periph-clk to recalculate current Runtime rate
* @Parameters:
* # clk: clock-id of soc specific clock
* @Return values:
* # HAL_CLK_STATUS_INVALID_PARAMETER: input parameter of clock-id undefined in hal
* # HAL_CLK_RATE_UNINITIALIZED : input parameter of clock-id defined in hal but not defined by soc clock driver or clock disbaled
* # others: return current clock rate successed
* @Attention: .etc
*************************************************************************************************/
u32 hal_clk_recalc_rate(hal_clk_t clk);

/************************************************************************************************
* @Function: hal_clk_round_rate
* @Description: provide HAL API for factor-clk, bus-clk and periph-clk round target rate to the most suitable rate
* @Parameters:
* # clk: clock-id of soc specific clock
* # rate: the target rate form API-User
* @Return values:
* # HAL_CLK_STATUS_INVALID_PARAMETER: input parameter of clock-id undefined in hal ot rate value is invalid
* # HAL_CLK_RATE_UNINITIALIZED : input parameter of clock-id defined in hal but not defined by soc clock driver or clock disbaled
* # others: return round rate successed
* @Attention: .etc
*************************************************************************************************/
u32 hal_clk_round_rate(hal_clk_t clk, u32 rate);

/************************************************************************************************
* @Function: hal_clk_get_rate
* @Description: provide HAL API for factor-clk, bus-clk and periph-clk get current rate cached witch may not current Runtime rate
* @Parameters:
* # clk: clock-id of soc specific clock
* @Return values:
* # HAL_CLK_STATUS_INVALID_PARAMETER: input parameter of clock-id undefined in hal ot rate value is invalid
* # HAL_CLK_RATE_UNINITIALIZED : input parameter of clock-id defined in hal but not defined by soc clock driver or clock disbaled
* # others: return rate cached successed
* @Attention: .etc
*************************************************************************************************/
u32  hal_clk_get_rate(hal_clk_t clk);

/************************************************************************************************
* @Function: hal_clk_set_rate
* @Description: provide HAL API for bus-clk and periph-clk to set new rate
* @Parameters:
* # clk: clock-id of soc specific clock
* # rate: the new rate value
* @Return values:
* # HAL_CLK_STATUS_INVALID_PARAMETER: input parameter of clock-id undefined in hal ot rate value is invalid
* # HAL_CLK_STATUS_ERROR_CLK_SET_RATE_REFUSED: fixed-clk and factor clk not allowed User to change rate because of stability
* # HAL_CLK_STATUS_ERROT_CLK_UNDEFINED: input parameter of clock-id defined in hal but not defined by soc clock driver
* # HAL_CLK_STATUS_ERROR_CLK_NOT_FOUND: input parameter of clock-id defined in hal but not defined by soc clock driver
* # HAL_CLK_STATUS_OK: set new rate successed
* @Attention: .etc
*************************************************************************************************/
hal_clk_status_t hal_clk_set_rate(hal_clk_t clk,  u32 rate);

/************************************************************************************************
* @Function: hal_clk_set_parent
* @Description: provide HAL API for factor-clk, bus-clk and periph-clk to select parent clock
* @Parameters:
* # clk: clock-id of soc specific clock witch nedds to adjust parent clock
* # parent: clock-id of soc specific clock's parent clock
* @Return values:
* # HAL_CLK_STATUS_OK: soc specific clock select and siwtch parent clock successed
* # others : soc specific clock select and siwtch parent clock may have some abnormal problems
* @Attention: soc specific clock and parent clock must be according to the SOC_User_Manual definition
*************************************************************************************************/
hal_clk_status_t hal_clk_set_parent(hal_clk_t clk, hal_clk_t parent);

/************************************************************************************************
* @Function: hal_clk_get_parent
* @Description: provide HAL API for factor-clk, bus-clk and periph-clk to get current parent clock
* @Parameters:
* # clk: clock-id of soc specific clock
* @Return values:
* # HAL_CLK_STATUS_INVALID_PARAMETER: input parameter of clock-id undefined in hal
* # HAL_CLK_UNINITIALIZED : input parameter of clock-id defined in hal but not defined by soc clock driver
* # others: return current parent clock-id successed
* @Attention: soc specific clock and parent clock must be according to the SOC_User_Manual definition
*************************************************************************************************/
hal_clk_t hal_clk_get_parent(hal_clk_t clk);


/************************************************************************************************
* @Function: hal_clk_ccu_aon_set_freq_trim
* @Description: provide HAL API to config ccu-aon dxco freq trim
* @Parameters:
* # value: dxco freq trim value
* @Return values:
* # # HAL_CLK_STATUS_OK: config success
* # others: config fail
* @Attention: nly successful register writing can be guaranteed here,
*             and the actual value needs to be read out for 'hal_clk_ccu_aon_get_freq_trim'
*************************************************************************************************/
hal_clk_status_t hal_clk_ccu_aon_set_freq_trim(uint32_t value);


/************************************************************************************************
* @Function: hal_clk_ccu_aon_get_freq_trim
* @Description: provide HAL API to obtain ccu-aon dxco freq trim
* @Parameters:
* # void: No parameters required
* @Return values:
* # the return value is ccu-aon dxco freq trim
* @Attention: .etc
*************************************************************************************************/
uint32_t hal_clk_ccu_aon_get_freq_trim(void);


#endif /* DRIVERS_HAL_CCMU_HAL_CLK_H_ */
