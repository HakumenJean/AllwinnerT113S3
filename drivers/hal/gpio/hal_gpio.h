/*
 * Copyright 2018 AllWinnertech  Co., Ltd
 * wangwei@allwinnertech.com
 */

#ifndef DRIVERS_HAL_GPIO_HAL_GPIO_H_
#define DRIVERS_HAL_GPIO_HAL_GPIO_H_

#include <stdint.h>

/* IO port */
enum gpio_port
{
    GPIO_PORT_RESERVED0 = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_NUM,
};

#define PB_BASE 32
#define PC_BASE 64
#define PD_BASE 96
#define PE_BASE 128
#define PF_BASE 160
#define PG_BASE 192

/* sunxi gpio name space */
#define GPIOB(n)    (PB_BASE + (n))
#define GPIOC(n)    (PC_BASE + (n))
#define GPIOD(n)    (PD_BASE + (n))
#define GPIOE(n)    (PE_BASE + (n))
#define GPIOF(n)    (PF_BASE + (n))
#define GPIOG(n)    (PG_BASE + (n))

#define GET_GPIO_PORT(PIN)       (PIN / 32)
#define GET_GPIO_PIN(PIN)        (PIN % 32)

typedef enum
{
    GPIO_PNONE  = 0,
    GPIO_PB0 = GPIOB(0),
    GPIO_PB1 = GPIOB(1),
    GPIO_PB2 = GPIOB(2),
    GPIO_PB3 = GPIOB(3),
    GPIO_PB4 = GPIOB(4),
    GPIO_PB5 = GPIOB(5),
    GPIO_PB6 = GPIOB(6),
    GPIO_PB7 = GPIOB(7),

    GPIO_PC0 = GPIOC(0),
    GPIO_PC1 = GPIOC(1),
    GPIO_PC2 = GPIOC(2),
    GPIO_PC3 = GPIOC(3),
    GPIO_PC4 = GPIOC(4),
    GPIO_PC5 = GPIOC(5),
    GPIO_PC6 = GPIOC(6),
    GPIO_PC7 = GPIOC(7),

    GPIO_PD0 = GPIOD(0),
    GPIO_PD1 = GPIOD(1),
    GPIO_PD2 = GPIOD(2),
    GPIO_PD3 = GPIOD(3),
    GPIO_PD4 = GPIOD(4),
    GPIO_PD5 = GPIOD(5),
    GPIO_PD6 = GPIOD(6),
    GPIO_PD7 = GPIOD(7),
    GPIO_PD8 = GPIOD(8),
    GPIO_PD9 = GPIOD(9),
    GPIO_PD10 = GPIOD(10),
    GPIO_PD11 = GPIOD(11),
    GPIO_PD12 = GPIOD(12),
    GPIO_PD13 = GPIOD(13),
    GPIO_PD14 = GPIOD(14),
    GPIO_PD15 = GPIOD(15),
    GPIO_PD16 = GPIOD(16),
    GPIO_PD17 = GPIOD(17),
    GPIO_PD18 = GPIOD(18),
    GPIO_PD19 = GPIOD(19),
    GPIO_PD20 = GPIOD(20),
    GPIO_PD21 = GPIOD(21),
    GPIO_PD22 = GPIOD(22),

    GPIO_PE0 = GPIOE(0),
    GPIO_PE1 = GPIOE(1),
    GPIO_PE2 = GPIOE(2),
    GPIO_PE3 = GPIOE(3),
    GPIO_PE4 = GPIOE(4),
    GPIO_PE5 = GPIOE(5),
    GPIO_PE6 = GPIOE(6),
    GPIO_PE7 = GPIOE(7),
    GPIO_PE8 = GPIOE(8),
    GPIO_PE9 = GPIOE(9),
    GPIO_PE10 = GPIOE(10),
    GPIO_PE11 = GPIOE(11),
    GPIO_PE12 = GPIOE(12),
    GPIO_PE13 = GPIOE(13),
    GPIO_PE14 = GPIOE(14),
    GPIO_PE15 = GPIOE(15),
    GPIO_PE16 = GPIOE(16),
    GPIO_PE17 = GPIOE(17),

    GPIO_PF0 = GPIOF(0),
    GPIO_PF1 = GPIOF(1),
    GPIO_PF2 = GPIOF(2),
    GPIO_PF3 = GPIOF(3),
    GPIO_PF4 = GPIOF(4),
    GPIO_PF5 = GPIOF(5),
    GPIO_PF6 = GPIOF(6),

    GPIO_PG0 = GPIOG(0),
    GPIO_PG1 = GPIOG(1),
    GPIO_PG2 = GPIOG(2),
    GPIO_PG3 = GPIOG(3),
    GPIO_PG4 = GPIOG(4),
    GPIO_PG5 = GPIOG(5),
    GPIO_PG6 = GPIOG(6),
    GPIO_PG7 = GPIOG(7),
    GPIO_PG8 = GPIOG(8),
    GPIO_PG9 = GPIOG(9),
    GPIO_PG10 = GPIOG(10),
    GPIO_PG11 = GPIOG(11),
    GPIO_PG12 = GPIOG(12),
    GPIO_PG13 = GPIOG(13),
    GPIO_PG14 = GPIOG(14),
    GPIO_PG15 = GPIOG(15),

    /* To aviod compile warnings. */
    GPIO_MAX = GPIOG(15),

} gpio_pin_t;

typedef enum
{
    GPIO_MUXSEL_IN = 0,
    GPIO_MUXSEL_OUT = 1,
    GPIO_MUXSEL_FUNCTION2 = 2,
    GPIO_MUXSEL_FUNCTION3 = 3,
    GPIO_MUXSEL_FUNCTION4 = 4,
    GPIO_MUXSEL_FUNCTION5 = 5,
    GPIO_MUXSEL_FUNCTION6 = 6,
    GPIO_MUXSEL_FUNCTION7 = 7,
    GPIO_MUXSEL_FUNCTION8 = 8,
    GPIO_MUXSEL_FUNCTION9 = 9,
    GPIO_MUXSEL_FUNCTION10 = 10,
    GPIO_MUXSEL_FUNCTION11 = 11,
    GPIO_MUXSEL_FUNCTION12 = 12,
    GPIO_MUXSEL_FUNCTION13 = 13,
    GPIO_MUXSEL_EINT = 14,
    GPIO_MUXSEL_DISABLED = 15,
} gpio_muxsel_t;

typedef enum
{
    GPIO_DRIVING_LEVEL0    = 0,        /**< Defines GPIO driving current as level0.  */
    GPIO_DRIVING_LEVEL1    = 1,        /**< Defines GPIO driving current as level1.  */
    GPIO_DRIVING_LEVEL2    = 2,        /**< Defines GPIO driving current as level2. */
    GPIO_DRIVING_LEVEL3    = 3         /**< Defines GPIO driving current as level3. */
} gpio_driving_level_t;

typedef enum
{
    GPIO_PULL_DOWN_DISABLED     = 0,        /**< Defines GPIO pull up and pull down disable.  */
    GPIO_PULL_UP                = 1,        /**< Defines GPIO is pull up state.  */
    GPIO_PULL_DOWN              = 2,        /**< Defines GPIO is pull down state. */
} gpio_pull_status_t;

/** This enum defines the GPIO direction. */
typedef enum
{
    GPIO_DIRECTION_INPUT  = 0,              /**<  GPIO input direction. */
    GPIO_DIRECTION_OUTPUT = 1               /**<  GPIO output direction. */
} gpio_direction_t;

/** This enum defines the data type of GPIO. */
typedef enum
{
    GPIO_DATA_LOW  = 0,                     /**<  GPIO data low. */
    GPIO_DATA_HIGH = 1                      /**<  GPIO data high. */
} gpio_data_t;

typedef enum
{
    POWER_MODE_330 = 0,
    POWER_MODE_180 = 1
} gpio_power_mode_t;

typedef enum
{
    IRQ_TYPE_NONE           = 0x00,
    IRQ_TYPE_EDGE_RISING    = 0x00,
    IRQ_TYPE_EDGE_FALLING   = 0x01,
    IRQ_TYPE_LEVEL_HIGH     = 0x02,
    IRQ_TYPE_LEVEL_LOW      = 0x03,
    IRQ_TYPE_EDGE_BOTH      = 0x04,
} gpio_interrupt_mode_t;

int hal_gpio_get_data(gpio_pin_t pin, gpio_data_t *value);
int hal_gpio_set_data(gpio_pin_t pin, gpio_data_t value);
int hal_gpio_pinmux_set_function(gpio_pin_t pin, gpio_muxsel_t function_index);
int hal_gpio_pinmux_get_function(gpio_pin_t pin, gpio_muxsel_t *function_index);
int hal_gpio_set_direction(gpio_pin_t pin, gpio_direction_t direction);
int hal_gpio_get_direction(gpio_pin_t pin, gpio_direction_t *direction);
int hal_gpio_set_pull(gpio_pin_t pin, gpio_pull_status_t pull);
int hal_gpio_get_pull(gpio_pin_t pin, gpio_pull_status_t *pull);
int hal_gpio_set_driving_level(gpio_pin_t pin, gpio_driving_level_t level);
int hal_gpio_get_driving_level(gpio_pin_t pin, gpio_driving_level_t *level);
int hal_gpio_sel_vol_mode(gpio_pin_t pin, gpio_power_mode_t pm_sel);
int hal_gpio_set_debounce(gpio_pin_t pin, unsigned value);

int hal_gpio_to_irq(gpio_pin_t pin, uint32_t *irq);
int hal_gpio_irq_enable(uint32_t irq);
int hal_gpio_irq_disable(uint32_t irq);
int hal_gpio_irq_request(uint32_t irq, void (*hdr)(void *args), unsigned long type, void *data);
int hal_gpio_irq_free(uint32_t irq);

void hal_gpio_init(void);

#endif /* DRIVERS_HAL_GPIO_HAL_GPIO_H_ */
