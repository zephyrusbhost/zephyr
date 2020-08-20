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
*                                   TEMPLATE HOST CONTROLLER DRIVER
*
* Filename : usbh_hcd_template.h
* Version  : V3.42.00
*********************************************************************************************************
*/

#ifndef  USBH_HCD_TEMPLATE_H
#define  USBH_HCD_TEMPLATE_H


/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/

#include  "../../Source/usbh_core.h"


/*
*********************************************************************************************************
*                                                 EXTERNS
*********************************************************************************************************
*/

#ifdef   USBH_HCD_TEMPLATE_MODULE
#define  USBH_HCD_TEMPLATE_EXT
#else
#define  USBH_HCD_TEMPLATE_EXT  extern
#endif


/*
*********************************************************************************************************
*                                                 DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/

USBH_HCD_TEMPLATE_EXT  USBH_HC_DRV_API  USBH_TemplateHCD_DrvAPI;
USBH_HCD_TEMPLATE_EXT  USBH_HC_RH_API   USBH_TemplateHCD_RH_API;


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
