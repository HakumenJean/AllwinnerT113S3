/* SPDX-License-Identifier: GPL-2.0 */

#ifndef DRIVERS_HAL_COMMON_HAL_COMMON_H_
#define DRIVERS_HAL_COMMON_HAL_COMMON_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_os.h"

#define CONFIG_ARCH_SUN8IW20
#define CONFIG_SOC_SUN20IW1

/**********************************************************
 * data type
 *
 *********************************************************/
#ifndef INT_MAX
#define INT_MAX     ((int)(~0U>>1))
#endif
#ifndef INT_MIN
#define INT_MIN     (-INT_MAX - 1)
#endif
#ifndef UINT_MAX
#define UINT_MAX    (~0U)
#endif
#ifndef LONG_MAX
#define LONG_MAX    ((long)(~0UL>>1))
#endif
#ifndef LONG_MIN
#define LONG_MIN    (-LONG_MAX - 1)
#endif
#ifndef ULONG_MAX
#define ULONG_MAX   (~0UL)
#endif
#ifndef LLONG_MAX
#define LLONG_MAX   ((long long)(~0ULL>>1))
#endif
#ifndef LLONG_MIN
#define LLONG_MIN   (-LLONG_MAX - 1)
#endif
#ifndef ULLONG_MAX
#define ULLONG_MAX  (~0ULL)
#endif

#ifndef min
#define min(a, b)  ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b)   ((a) < (b) ? (b) : (a))
#endif

/* return value defines */
#define OK  (0)
#define FAIL    (-1)

#define CACHELINE_LEN (64)

typedef int8_t      __s8;
typedef int8_t      s8;
typedef uint8_t     __u8;
typedef uint8_t     u8;

typedef int16_t     __s16;
typedef int16_t     s16;
typedef uint16_t    __u16;
typedef uint16_t    u16;

typedef unsigned int    __u32;
typedef unsigned int    u32;
typedef int32_t     __s32;
typedef int32_t     s32;

typedef int64_t     __s64;
typedef int64_t     s64;
typedef uint64_t    __u64;
typedef uint64_t    u64;

#ifndef __packed
#define __packed        __attribute__((packed))
#endif
#ifndef __aligned
#define __aligned(x)    __attribute__((__aligned__(x)))
#endif
#define HAL_ARG_UNUSED(NAME)   (void)(NAME)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)       (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif

#define __ALIGN_KERNEL(x, a) __ALIGN_KERNEL_MASK(x, (typeof(x))(a) - 1)
#define __ALIGN_KERNEL_MASK(x, mask) (((x) + (mask)) & ~(mask))

#ifndef ALIGN_UP
#define ALIGN_UP(x, a) __ALIGN_KERNEL((x), (a))
#endif

#ifndef ALIGN_DOWN
#define ALIGN_DOWN(x, a) __ALIGN_KERNEL((x) - ((a) - 1), (a))
#endif

#define HAL_ALIGN(x, a) ALIGN_UP(x, a)

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:    the pointer to the member.
 * @type:   the type of the container struct this is embedded in.
 * @member: the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({          \
    const typeof(((type *)0)->member) * __mptr = (ptr); \
    (type *)((char *)__mptr - offsetof(type, member)); })

#define UNUSED(var) {(void)var;}

/**********************************************************
 * common register access operation.
 *
 *********************************************************/

#define get_bvalue(addr)    (*((volatile unsigned char  *)(addr)))
#define put_bvalue(addr, v) (*((volatile unsigned char  *)(addr)) = (unsigned char)(v))
#define get_hvalue(addr)    (*((volatile unsigned short *)(addr)))
#define put_hvalue(addr, v) (*((volatile unsigned short *)(addr)) = (unsigned short)(v))
#define get_wvalue(addr)    (*((volatile unsigned int   *)(addr)))
#define put_wvalue(addr, v) (*((volatile unsigned int   *)(addr)) = (unsigned int)(v))

#define set_byte(addr, v)    (*((volatile unsigned char  *)(addr)) |=  (unsigned char)(v))
#define clr_byte(addr, v)    (*((volatile unsigned char  *)(addr)) &= ~(unsigned char)(v))
#define set_half_word(addr, v)   (*((volatile unsigned short *)(addr)) |=  (unsigned short)(v))
#define clr_half_word(addr, v)   (*((volatile unsigned short *)(addr)) &= ~(unsigned short)(v))
#define set_word(addr, v)   (*((volatile unsigned int   *)(addr)) |=  (unsigned int)(v))
#define clr_word(addr, v)   (*((volatile unsigned int   *)(addr)) &= ~(unsigned int)(v))

#define hal_readb(reg)          (*(volatile uint8_t  *)(long)(reg))
#define hal_readw(reg)          (*(volatile uint16_t *)(reg))
#define hal_readl(reg)          (*(volatile uint32_t *)(reg))
#define hal_writeb(value,reg)   (*(volatile uint8_t  *)(long)(reg) = (value))
#define hal_writew(value,reg)   (*(volatile uint16_t *)(reg) = (value))
#define hal_writel(value,reg)   (*(volatile uint32_t *)(reg) = (value))

#define hal_write_reg8(addr ,data)      ((*(volatile u8 *)(addr)) = (u8)(data))
#define hal_write_reg16(addr ,data)     ((*(volatile u16 *)(addr)) = (u16)(data))
#define hal_write_reg32(addr ,data)     ((*(volatile u32 *)(addr)) = (u32)(data))
#define hal_read_reg8(x)                (*(volatile u8 *)(x))
#define hal_read_reg16(x)               (*(volatile u16 *)(x))
#define hal_read_reg32(x)               (*(volatile u32 *)(x))

#ifndef readl
#define readl(x)            hal_read_reg32(x)
#endif
#ifndef writel
#define writel(val, reg)    hal_write_reg32(reg, val)
#endif

#ifndef readw
#define readw(x)            hal_readw(x)
#endif
#ifndef writew
#define writew(v, addr)     hal_writew(v, addr)
#endif

#ifndef readb
#define readb(x)            hal_readb(x)
#endif
#ifndef writeb
#define writeb(v, addr)     hal_writeb(v, addr)
#endif

#define HAL_SIZE_T      (unsigned long)
#define HAL_PT_TO_U(v)      (HAL_SIZE_T(v))

#define HAL_PR_SZ_L(v)      (HAL_SIZE_T(v))
#define HAL_PR_SZ(v)        ((unsigned int)(v))
#define HAL_PR_SZ_P(v)      ((unsigned int *)(v))

#define __va_to_pa(vaddr) ((unsigned long)vaddr)
#define __pa_to_va(vaddr) ((unsigned long)vaddr)

#ifndef BIT
#define BIT(nr)     (1UL << (nr))
#endif

#define hal_assert(ex)                                                  \
    if (!(ex)) {                                                        \
        hal_printf("%s line %d, fatal error.\n", __func__, __LINE__);       \
        while(1);                                                       \
    }

#define MAX_ERRNO   4095

#define IS_ERR_VALUE(x) ((x) >= (unsigned long)-MAX_ERRNO)

static inline long IS_ERR(const void *ptr)
{
    return IS_ERR_VALUE((unsigned long)ptr);
}

/**********************************************************
 * hal status
 *
 *********************************************************/
typedef enum {
    HAL_OK      = 0,    /* success */
    HAL_ERROR   = -1,   /* general error */
    HAL_BUSY    = -2,   /* device or resource busy */
    HAL_TIMEOUT = -3,   /* wait timeout */
    HAL_INVALID = -4,   /* invalid argument */
    HAL_NOMEM   = -5,   /* no memory */
} hal_status_t;

#endif /* DRIVERS_HAL_COMMON_HAL_COMMON_H_ */
