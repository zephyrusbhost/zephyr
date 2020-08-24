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
* Filename : usbh_hc_cfg.h
* Version  : V3.42.00
*********************************************************************************************************
*/

#ifndef  USBH_HC_CFG_MODULE_PRESENT
#define  USBH_HC_CFG_MODULE_PRESENT


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  <usbh_core.h>


/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

#ifdef   USBH_HC_CFG_MODULE
#define  USBH_HC_CFG_MODULE_EXT
#else
#define  USBH_HC_CFG_MODULE_EXT  extern
#endif


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define USBH_DATA_BUF_MAX_LEN 1024
#define USBH_MAX_NBR_EP_BULK_OPEN 2
#define USBH_MAX_NBR_EP_INTR_OPEN 2

/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                  USB HOST CONTROLLER CONFIGURATION
*********************************************************************************************************
*/

USBH_HC_CFG_MODULE_EXT const struct usbh_hc_cfg  USBH_HC_TemplateCfg;


/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                        CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif
