/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the People's Republic of China and other countries.
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
#ifndef DRIVERS_HAL_TIMER_HAL_HTIMER_H_
#define DRIVERS_HAL_TIMER_HAL_HTIMER_H_

#include "hal_clk.h"
#include "hal_reset.h"
#include "platform/timer_sun8iw20.h"

#include "hal_common_timer.h"

#ifndef BIT
#define BIT(_x) (1UL << (_x))
#endif

#define HTIMER_IRQ_EN_REG               (SUNXI_HSTIMER_PBASE + 0x00)
#define HTIMER_IRQ_EN(val)              BIT(val)
#define HTIMER_IRQ_ST_REG               (SUNXI_HSTIMER_PBASE + 0x04)
#define HTIMER_CTL_REG(val)             (SUNXI_HSTIMER_PBASE + (0x20 * val + 0x20))
#define HTIMER_CTL_ENABLE               BIT(0)
#define HTIMER_CTL_RELOAD               BIT(1)
#define HTIMER_CTL_CLK_PRES(val)        (((val) & 0x7) << 4)
#define HTIMER_CTL_ONESHOT              BIT(7)
#define HTIMER_INTVAL_LO_REG(val)       (SUNXI_HSTIMER_PBASE + (0x20 * (val) + 0x24))
#define HTIMER_INTVAL_HI_REG(val)       (SUNXI_HSTIMER_PBASE + (0x20 * (val) + 0x28))
#define HTIMER_CNTVAL_LO_REG(val)       (SUNXI_HSTIMER_PBASE + (0x20 * (val) + 0x2c))
#define HTIMER_CNTVAL_HI_REG(val)       (SUNXI_HSTIMER_PBASE + (0x20 * (val) + 0x30))

#define HTIMER_SYNC_TICKS           3

struct sunxi_htimer
{
    uint32_t timer_id;
    uint32_t ticks;
    uint32_t clk_rate;
    uint32_t irq;
    uint32_t min_delta_ticks;
    uint32_t max_delta_ticks;
    htimer_callback callback;
    void *param;
};

void hal_htimer_init(void);

void hal_htimer_stop(hal_htimer_id id);
void hal_htimer_start(hal_htimer_id id, bool periodic);
int hal_htimer_set(hal_htimer_id id, uint32_t clock, htimer_callback callback, void *callback_param);

#endif /* DRIVERS_HAL_TIMER_HAL_HTIMER_H_ */
