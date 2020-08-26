/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _USBH_LIB_MEM_H_
#define _USBH_LIB_MEM_H_

#include <kernel.h>
#include <string.h>
#include <sys/byteorder.h>
#include <usbh_cpu.h>

#define DEF_MAX(a, b) MAX(a, b)

#if (CPU_CFG_ENDIAN_TYPE == CPU_ENDIAN_TYPE_BIG)
#define MEM_VAL_GET_INT08U(val) (uint8_t) * (val)
#define MEM_VAL_GET_INT16U(val) sys_get_be16(val)
#define MEM_VAL_GET_INT24U(val) sys_get_be24(val)
#define MEM_VAL_GET_INT32U(val) sys_get_be32(val)
#elif (CPU_CFG_ENDIAN_TYPE == CPU_ENDIAN_TYPE_LITTLE)
#define MEM_VAL_GET_INT08U(val) (uint8_t) * (val)
#define MEM_VAL_GET_INT16U(val) sys_get_le16(val)
#define MEM_VAL_GET_INT24U(val) sys_get_le24(val)
#define MEM_VAL_GET_INT32U(val) sys_get_le32(val)
#else
#error "Unknown byte order"
#endif

#define MEM_VAL_GET_INT16U_LITTLE(val) sys_get_le16((uint8_t *)val)
#define MEM_VAL_GET_INT32U_LITTLE(val) sys_get_le32((uint8_t *)val)

#define MEM_VAL_COPY_GET_INT32U_BIG(dest, src) sys_memcpy_swap(dest, src, sizeof(uint32_t))
#define MEM_VAL_COPY_SET_INT32U_BIG(dest, src) sys_memcpy_swap(dest, src, sizeof(uint32_t))
#define MEM_VAL_COPY_SET_INT16U_BIG(dest, src) sys_memcpy_swap(dest, src, sizeof(uint16_t))

#define DEF_BIT_SET(val, mask) (val = (val | mask))
#define DEF_BIT_IS_SET(val, mask) ((((val & mask) == mask) && (mask != 0)) ? true : false)
#define DEF_BIT_IS_CLR(val, mask) (((val & mask) == 0 && (mask != 0)) ? true : false)

#define DEF_BIT_CLR_08(val, mask) ((val) = (uint8_t)(((uint8_t)(val)) & (uint8_t)(~((uint8_t)(mask)))))
#define DEF_BIT_CLR_16(val, mask) ((val) = (uint16_t)(((uint16_t)(val)) & (uint16_t)(~((uint16_t)(mask)))))
#define DEF_BIT_CLR_32(val, mask) ((val) = (uint32_t)(((uint32_t)(val)) & (uint32_t)(~((uint32_t)(mask)))))
#define DEF_BIT_CLR(val, mask) ((sizeof((val)) == CPU_WORD_SIZE_08) ? DEF_BIT_CLR_08((val), (mask)) : ((sizeof((val)) == CPU_WORD_SIZE_16) ? DEF_BIT_CLR_16((val), (mask)) : ((sizeof((val)) == CPU_WORD_SIZE_32) ? DEF_BIT_CLR_32((val), (mask)) : 0)))

#define DEF_BIT(n) BIT(n)
#define DEF_BIT_00 BIT(0)
#define DEF_BIT_01 BIT(1)
#define DEF_BIT_02 BIT(2)
#define DEF_BIT_03 BIT(3)
#define DEF_BIT_04 BIT(4)
#define DEF_BIT_05 BIT(5)
#define DEF_BIT_06 BIT(6)
#define DEF_BIT_07 BIT(7)
#define DEF_BIT_08 BIT(8)
#define DEF_BIT_09 BIT(9)
#define DEF_BIT_10 BIT(10)

#endif /* _USBH_LIB_MEM_H_ */
