/* SPDX-License-Identifier: GPL-2.0 */

#ifndef DRIVERS_HAL_COMMON_HAL_LOG_H_
#define DRIVERS_HAL_COMMON_HAL_LOG_H_

#include "hal_os.h"

#ifndef DBG_TAG
#define DBG_TAG  "HAL"
#define DBG_LVL  DBG_INFO
#include <rtdbg.h>
#endif

#define hal_log_err             LOG_E
#define hal_log_warn            LOG_W
#define hal_log_info            LOG_I
#define hal_log_debug           LOG_D

#define pr_err(fmt, ...)           LOG_E(fmt, ##__VA_ARGS__)
#define pr_warn(fmt, ...)          LOG_W(fmt, ##__VA_ARGS__)
#define pr_info(fmt, ...)          LOG_I(fmt, ##__VA_ARGS__)
#define pr_debug(fmt, ...)         LOG_D(fmt, ##__VA_ARGS__)

#endif /* DRIVERS_HAL_COMMON_HAL_LOG_H_ */
