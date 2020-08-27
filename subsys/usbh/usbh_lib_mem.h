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




//#define MEM_VAL_COPY_GET_INT32U_BIG(dest, src) sys_memcpy_swap(dest, src, sizeof(uint32_t))
//#define MEM_VAL_COPY_SET_INT32U_BIG(dest, src) sys_memcpy_swap(dest, src, sizeof(uint32_t))
#define MEM_VAL_COPY_SET_INT16U_BIG(dest, src) sys_memcpy_swap(dest, src, sizeof(uint16_t))
#define DEF_BIT_IS_SET(val, mask) ((((val & mask) == mask) && (mask != 0)) ? true : false)
#define DEF_BIT_IS_CLR(val, mask) (((val & mask) == 0 && (mask != 0)) ? true : false)

#endif /* _USBH_LIB_MEM_H_ */
