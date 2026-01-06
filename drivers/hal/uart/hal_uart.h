/*
 * ===========================================================================================
 *
 *       Filename:  hal_uart.h
 *
 *    Description:  UART HAL definition.
 *
 *        Version:  Melis3.0
 *         Create:  2019-11-14 11:11:56
 *       Revision:  none
 *       Compiler:  GCC:version 9.2.1 20170904 (release),SUNXI_HAL/embedded-7-branch revision 255204
 *
 *         Author:  bantao@allwinnertech.com
 *   Organization:  SWC-BPD
 *  Last Modified:  2020-04-02 19:39:41
 *
 * ===========================================================================================
 */

#ifndef DRIVERS_HAL_UART_HAL_UART_H_
#define DRIVERS_HAL_UART_HAL_UART_H_

#include "hal_common.h"
#include "platform/uart_sun8iw20.h"
#include "hal_atomic.h"

/*
 * This enum defines return status of the UART HAL public API.
 * User should check return value after calling these APIs.
 */
typedef enum
{
    HAL_UART_STATUS_ERROR_PARAMETER = -4,      /**< Invalid user input parameter. */
    HAL_UART_STATUS_ERROR_BUSY = -3,           /**< UART port is currently in use. */
    HAL_UART_STATUS_ERROR_UNINITIALIZED = -2,  /**< UART port has not been initialized. */
    HAL_UART_STATUS_ERROR = -1,                /**< UART driver detected a common error. */
    HAL_UART_STATUS_OK = 0                     /**< UART function executed successfully. */
} hal_uart_status_t;

typedef enum
{
    UART_0 = 0,
    UART_1,
    UART_2,
    UART_3,
    UART_4,
    UART_5,
    UART_MAX,
} uart_port_t;

#define UART_BAUDRATE_300                   300
#define UART_BAUDRATE_600                   600
#define UART_BAUDRATE_1200                  1200
#define UART_BAUDRATE_2400                  2400
#define UART_BAUDRATE_4800                  4800
#define UART_BAUDRATE_9600                  9600
#define UART_BAUDRATE_19200                 19200
#define UART_BAUDRATE_38400                 38400
#define UART_BAUDRATE_57600                 57600
#define UART_BAUDRATE_115200                115200
#define UART_BAUDRATE_230400                230400
#define UART_BAUDRATE_460800                460800
#define UART_BAUDRATE_500000                500000
#define UART_BAUDRATE_576000                576000
#define UART_BAUDRATE_921600                921600
#define UART_BAUDRATE_1000000               1000000
#define UART_BAUDRATE_1152000               1152000
#define UART_BAUDRATE_1500000               1500000
#define UART_BAUDRATE_2000000               2000000
#define UART_BAUDRATE_2500000               2500000
#define UART_BAUDRATE_3000000               3000000
#define UART_BAUDRATE_3500000               3500000
#define UART_BAUDRATE_4000000               4000000

#define UART_WORD_LENGTH_5                  5
#define UART_WORD_LENGTH_6                  6
#define UART_WORD_LENGTH_7                  7
#define UART_WORD_LENGTH_8                  8

#define UART_STOP_BIT_1                     0
#define UART_STOP_BIT_2                     1

#define UART_PARITY_NONE                    0
#define UART_PARITY_ODD                     1
#define UART_PARITY_EVEN                    2

/* This struct defines UART configure parameters. */
typedef struct
{
    uint32_t baudrate;
    uint32_t word_length;
    uint32_t stop_bit;
    uint32_t parity;
    int flowctrl;
} _uart_config_t;

#ifndef BIT
#define BIT(nr)     (1UL << (nr))
#endif

//=================================reg===================================================//


/* This enum defines the UART event when an interrupt occurs. */
typedef enum
{
    UART_EVENT_TRANSACTION_ERROR = -1,
    UART_EVENT_RX_BUFFER_ERROR = -2,
    UART_EVENT_TX_COMPLETE = 1,
    UART_EVENT_RX_COMPLETE = 2,
    UART_EVENT_RX_DATA = 3,
} uart_callback_event_t;

/** @brief This typedef defines user's callback function prototype.
 *             This callback function will be called in UART interrupt handler when UART interrupt is raised.
 *             User should call uart_register_callback() to register callbacks to UART driver explicitly.
 *             Note, that the callback function is not appropriate for time-consuming operations. \n
 *             parameter "event" : for more information, please refer to description of #uart_callback_event_t.
 *             parameter "user_data" : a user defined data used in the callback function.
 */
typedef void (*uart_callback_t)(uart_callback_event_t event, void *user_data);

/* This struct defines UART private data */
typedef struct
{
    /* basic info */
    unsigned long uart_base;
    uart_port_t uart_port;
    uint32_t irqn;

    /* uart register value */
    unsigned char ier;
    unsigned char lcr;
    unsigned char mcr;
    unsigned char fcr;
    unsigned char dll;
    unsigned char dlh;

    /* user callback */
    uart_callback_t func;
    void *arg;

    hal_spinlock_t spinlock;
} uart_priv_t;

int32_t hal_uart_init(uart_port_t uart_port, uart_callback_t callback, void *data);
int32_t hal_uart_deinit(uart_port_t uart_port);

void hal_uart_set_format(uart_port_t uart_port, uint32_t word_length, uint32_t stop_bit, uint32_t parity);
void hal_uart_set_baudrate(uart_port_t uart_port, uint32_t baudrate);
void hal_uart_set_hardware_flowcontrol(uart_port_t uart_port);
void hal_uart_disable_flowcontrol(uart_port_t uart_port);
void hal_uart_set_loopback(uart_port_t uart_port, bool enable);

int hal_uart_putc(uart_port_t uart_port, char c);
int hal_uart_getc(uart_port_t uart_port);

int32_t hal_uart_enable_rx(uart_port_t uart_port);
int32_t hal_uart_disable_rx(uart_port_t uart_port);

#endif /* DRIVERS_HAL_UART_HAL_UART_H_ */
