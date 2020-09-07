/*
*********************************************************************************************************
*                                             uC/USB-Host
*                                     The Embedded USB Host Stack
*
*                    Copyright 2004-2020 Silicon Laboratories Inc. www.silabs.com
*
*                                 SPDX-License-Identifier: APACHE-2.0
*
*               This software is subject to an open source license and is distributed by
*                Silicon Laboratories Inc. pursuant to the terms of the Apache License,
*                    Version 2.0 available at www.apache.org/licenses/LICENSE-2.0.
*
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                      MASS STORAGE CLASS (MSC)
*
* Filename : usbh_msc.h
* Version  : V3.42.00
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               MODULE
*********************************************************************************************************
*/

#ifndef USBH_MSC_MODULE_PRESENT
#define USBH_MSC_MODULE_PRESENT
#include "usbh_class.h"

/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

#ifdef USBH_MSC_MODULE
#define USBH_MSC_EXT
#else
#define USBH_MSC_EXT extern
#endif

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define USBH_MSC_TIMEOUT 10000u

#define USBH_MSC_DEV_NOT_IN_USE 0u
#define USBH_MSC_DEV_IN_USE 1u

#define USBH_MSC_DATA_DIR_IN 0x80u
#define USBH_MSC_DATA_DIR_OUT 0x00u
#define USBH_MSC_DATA_DIR_NONE 0x01u

/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/

typedef uint8_t USBH_MSC_DATA_DIR;

/* -------------------- MSC DEVICE -------------------- */
typedef struct usbh_msc_dev
{
    struct usbh_ep BulkInEP;  /* Bulk IN  endpoint.                                   */
    struct usbh_ep BulkOutEP; /* Bulk OUT endpoint.                                   */
    struct usbh_dev *DevPtr;  /* Pointer to USB device.                               */
    struct usbh_if *IF_Ptr;   /* Pointer to interface.                                */
    uint8_t State;  /* State of MSC device.                                 */
    uint8_t RefCnt; /* Cnt of app ref on this dev.                          */
    struct k_mutex HMutex;
} USBH_MSC_DEV;

typedef struct msc_inquiry_info
{
    uint8_t DevType;
    uint8_t IsRemovable;
    uint8_t Vendor_ID[8];
    uint8_t Product_ID[16];
    uint32_t ProductRevisionLevel;
} USBH_MSC_INQUIRY_INFO;

/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/

USBH_MSC_EXT const struct usbh_class_drv USBH_MSC_ClassDrv;

/*
*********************************************************************************************************
*                                                 MACROS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

int usbh_msc_init(USBH_MSC_DEV *p_msc_dev,
                       uint8_t lun);

uint8_t usbh_msc_max_lun_get(USBH_MSC_DEV *p_msc_dev,
                               int *p_err);

bool usbh_msc_unit_rdy_test(USBH_MSC_DEV *p_msc_dev,
                                 uint8_t lun,
                                 int *p_err);

int usbh_msc_capacity_rd(USBH_MSC_DEV *p_msc_dev,
                             uint8_t lun,
                             uint32_t *p_nbr_blks,
                             uint32_t *p_blk_size);

int usbh_msc_std_inquiry(USBH_MSC_DEV *p_msc_dev,
                             USBH_MSC_INQUIRY_INFO *p_msc_inquiry_info,
                             uint8_t lun);

int usbh_msc_ref_add(USBH_MSC_DEV *p_msc_dev);

int usbh_msc_ref_rel(USBH_MSC_DEV *p_msc_dev);

uint32_t usbh_msc_read(USBH_MSC_DEV *p_msc_dev,
                       uint8_t lun,
                       uint32_t blk_addr,
                       uint16_t nbr_blks,
                       uint32_t blk_size,
                       void *p_arg,
                       int *p_err);

uint32_t usbh_msc_write(USBH_MSC_DEV *p_msc_dev,
                       uint8_t lun,
                       uint32_t blk_addr,
                       uint16_t nbr_blks,
                       uint32_t blk_size,
                       const void *p_arg,
                       int *p_err);

/*
*********************************************************************************************************
*                                          CONFIGURATION ERRORS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
*/

#endif
