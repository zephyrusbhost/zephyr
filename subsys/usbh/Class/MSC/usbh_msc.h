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

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "usbh_os.h"
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
    USBH_EP BulkInEP;  /* Bulk IN  endpoint.                                   */
    USBH_EP BulkOutEP; /* Bulk OUT endpoint.                                   */
    USBH_DEV *DevPtr;  /* Pointer to USB device.                               */
    USBH_IF *IF_Ptr;   /* Pointer to interface.                                */
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

USBH_MSC_EXT USBH_CLASS_DRV USBH_MSC_ClassDrv;

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

USBH_ERR USBH_MSC_Init(USBH_MSC_DEV *p_msc_dev,
                       uint8_t lun);

uint8_t USBH_MSC_MaxLUN_Get(USBH_MSC_DEV *p_msc_dev,
                               USBH_ERR *p_err);

bool USBH_MSC_UnitRdyTest(USBH_MSC_DEV *p_msc_dev,
                                 uint8_t lun,
                                 USBH_ERR *p_err);

USBH_ERR USBH_MSC_CapacityRd(USBH_MSC_DEV *p_msc_dev,
                             uint8_t lun,
                             uint32_t *p_nbr_blks,
                             uint32_t *p_blk_size);

USBH_ERR USBH_MSC_StdInquiry(USBH_MSC_DEV *p_msc_dev,
                             USBH_MSC_INQUIRY_INFO *p_msc_inquiry_info,
                             uint8_t lun);

USBH_ERR USBH_MSC_RefAdd(USBH_MSC_DEV *p_msc_dev);

USBH_ERR USBH_MSC_RefRel(USBH_MSC_DEV *p_msc_dev);

uint32_t USBH_MSC_Rd(USBH_MSC_DEV *p_msc_dev,
                       uint8_t lun,
                       uint32_t blk_addr,
                       uint16_t nbr_blks,
                       uint32_t blk_size,
                       void *p_arg,
                       USBH_ERR *p_err);

uint32_t USBH_MSC_Wr(USBH_MSC_DEV *p_msc_dev,
                       uint8_t lun,
                       uint32_t blk_addr,
                       uint16_t nbr_blks,
                       uint32_t blk_size,
                       const void *p_arg,
                       USBH_ERR *p_err);

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
