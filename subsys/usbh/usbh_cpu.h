/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UC_USBH_CPU_H_
#define UC_USBH_CPU_H_

#include <zephyr.h>
#include <sys/math_extras.h>

typedef void (*CPU_FNCT_PTR)(void *);

#define CPU_ENDIAN_TYPE_NONE 0u
#define CPU_ENDIAN_TYPE_BIG 1u
#define CPU_ENDIAN_TYPE_LITTLE 2u

#define CPU_WORD_SIZE_08 1u
#define CPU_WORD_SIZE_16 2u
#define CPU_WORD_SIZE_32 4u
#define CPU_WORD_SIZE_64 8u

#define CPU_SW_EXCEPTION(pff)

#define CPU_CntLeadZeros(val) u32_count_leading_zeros(val)
#define CPU_CntTrailZeros(val) u32_count_trailing_zeros(val)

/* ****** */

#define DEF_TRUE true
#define DEF_FALSE false

#define DEF_ENABLED 1
#define DEF_DISABLED 0

#define CPU_WORD_SIZE_08 1u
#define CPU_WORD_SIZE_16 2u
#define CPU_WORD_SIZE_32 4u
#define CPU_WORD_SIZE_64 8u

#define CPU_CRITICAL_ENTER() key = irq_lock()
#define CPU_CRITICAL_EXIT() irq_unlock(key);
#define CPU_SR_ALLOC() int key

#define CPU_INT_EN()
#define CPU_INT_DIS()

#define CPU_CFG_ADDR_SIZE CPU_WORD_SIZE_32     /* Defines CPU address word size  (in octets). */
#define CPU_CFG_DATA_SIZE CPU_WORD_SIZE_32     /* Defines CPU data    word size  (in octets). */
#define CPU_CFG_DATA_SIZE_MAX CPU_WORD_SIZE_32 /* Defines CPU maximum word size  (in octets). */

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define CPU_CFG_ENDIAN_TYPE CPU_ENDIAN_TYPE_LITTLE
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define CPU_CFG_ENDIAN_TYPE CPU_ENDIAN_TYPE_BIG
#else
#error "Unknown byte order"
#endif

typedef uint32_t CPU_ADDR;
typedef uint32_t CPU_DATA;

typedef CPU_DATA CPU_ALIGN;  /* Defines CPU data-word-alignment size.  */
typedef CPU_ADDR CPU_SIZE_T; /* Defines CPU standard 'size_t'   size.  */
#endif