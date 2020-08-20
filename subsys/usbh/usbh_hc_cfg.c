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
*                               USB HOST CONTROLLER CONFIGURATION FILE
*
*                                              TEMPLATE
*
* Filename : usbh_hc_cfg.c
* Version  : V3.42.00
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define   USBH_HC_CFG_MODULE
#define   MICRIUM_SOURCE
#include  <usbh_hc_cfg.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             LOCAL TABLES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

USBH_HC_CFG  USBH_HC_TemplateCfg = {
    (CPU_ADDR)0x41005000,                                      /* Base addr of host controller hw registers.           */
    (CPU_ADDR)0x00000000u,                                      /* Base addr of host controller dedicated mem.          */
              0u,                                               /* Size      of host controller dedicated mem.          */
              DEF_ENABLED,                                      /* Does HC can access sys mem?                          */
              USBH_DATA_BUF_MAX_LEN,                                            /* Data buf max len.                                    */
              USBH_MAX_NBR_EP_BULK_OPEN,                                               /* Max nbr opened bulk EP.                              */
              USBH_MAX_NBR_EP_INTR_OPEN,                                               /* Max nbr opened intr EP.                              */
              2u                                                /* Max nbr opened isoc EP.                              */
};


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/
