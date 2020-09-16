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
 *                                    USB HOST STACK ERROR CODES
 *
 * Filename : usbh_err.h
 * Version  : V3.42.00
 *********************************************************************************************************
 */

/*
 **********************************************************************************************************
 *                                               MODULE
 **********************************************************************************************************
 */

#ifndef  USBH_ERR_H
#define  USBH_ERR_H


typedef  enum  usbh_err {





	USBH_ERR_EP_STALL               =   404u,


/*
 ********************************************************************************************************
 *                                 USB REQUEST BLOCK (URB) ERROR CODES
 ********************************************************************************************************
 */



/*
 *********************************************************************************************************
 *                                       DESCRIPTOR ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_DESC_LANG_ID_NOT_SUPPORTED     =   602u,
	USBH_ERR_DESC_EXTRA_NOT_FOUND           =   603u,


/*
 *********************************************************************************************************
 *                                     HOST CONTROLLER ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_HC_ALLOC       =   700u,
	USBH_ERR_HC_INIT        =   701u,
	USBH_ERR_HC_START       =   702u,
	USBH_ERR_HC_IO          =   703u,
	USBH_ERR_HC_HALTED      =   704u,
	USBH_ERR_HC_PORT_RESET  =   705u,


/*
 *********************************************************************************************************
 *                               OPERATING SYSTEM (OS) LAYER ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_OS_TASK_CREATE         =   800u,
	USBH_ERR_OS_SIGNAL_CREATE       =   801u,
	USBH_ERR_OS_DEL                 =   802u,
	USBH_ERR_OS_TIMEOUT             =   803u,
	USBH_ERR_OS_ABORT               =   804u,
	USBH_ERR_OS_FAIL                =   805u,


/*
 *********************************************************************************************************
 *                                          CLASS ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_CLASS_PROBE_FAIL       =  1000u,
	USBH_ERR_CLASS_DRV_NOT_FOUND    =  1001u,
	USBH_ERR_CLASS_DRV_ALLOC        =  1002u,


/*
 *********************************************************************************************************
 *                            COMMUNICATION DEVICE CLASS (CDC) ERROR CODES
 *********************************************************************************************************
 */


/*
 *********************************************************************************************************
 *                        CDC ABSTRACT CONTROL MODEL (ACM) SUBCLASS ERROR CODES
 *********************************************************************************************************
 */


/*
 *********************************************************************************************************
 *                                        HUB CLASS ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_HUB_INVALID_PORT_NBR   =  1200u,
	USBH_ERR_HUB_PORT               =  1201u,


/*
 *********************************************************************************************************
 *                           HUMAN INTERFACE DEVICE (HID) CLASS ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_HID_ITEM_LONG                  =  1300u,
	USBH_ERR_HID_ITEM_UNKNOWN               =  1301u,
	USBH_ERR_HID_MISMATCH_COLL              =  1302u,
	USBH_ERR_HID_NOT_APP_COLL               =  1303u,
	USBH_ERR_HID_REPORT_OUTSIDE_COLL        =  1304u,
	USBH_ERR_HID_MISMATCH_PUSH_POP          =  1305u,
	USBH_ERR_HID_USAGE_PAGE_INVALID         =  1306u,
	USBH_ERR_HID_REPORT_ID                  =  1307u,
	USBH_ERR_HID_REPORT_CNT                 =  1308u,
	USBH_ERR_HID_PUSH_SIZE                  =  1309u,
	USBH_ERR_HID_POP_SIZE                   =  1310u,
	USBH_ERR_HID_REPORT_INVALID_VAL         =  1311u,
	USBH_ERR_HID_RD_PARSER_FAIL             =  1312u,
	USBH_ERR_HID_NOT_IN_REPORT              =  1313u,


/*
 *********************************************************************************************************
 *                                MASS STORAGE CLASS (MSC) ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_MSC_CMD_FAILED                 =  1400u,
	USBH_ERR_MSC_CMD_PHASE                  =  1401u,
	USBH_ERR_MSC_IO                         =  1402u,
	USBH_ERR_MSC_LUN_ALLOC                  =  1403u,

/*
 *********************************************************************************************************
 *                  FUTURE TECHNOLOGY DEVICES INTERNATIONAL CLASS (FTDI) ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_FTDI_LINE                      =  1500u,

/*
 *********************************************************************************************************
 *                               PRINTER CLASS (PRN) ERROR CODES
 *********************************************************************************************************
 */

	USBH_ERR_PRN_NO_ACTIVE_PDL              = 1600u,                /* no current pdl selected error code  */
	USBH_ERR_PRN_NO_MORE_FONTS              = 1601u,                /* no more fonts available error code  */
	USBH_ERR_PRN_FONT_GET                   = 1602u,                /* error code for retrieving first font*/
	USBH_ERR_PRN_INVALID_FONT               = 1603u,                /* invalid font selection error code   */
	USBH_ERR_PRN_LINE_PARSE                 = 1604u,                /* line buffer parsing error code      */
	USBH_ERR_PRN_CFG_MAX_NBR_PRN_DEV        = 1605u,                /* printer device configuration .....  */
	USBH_ERR_PRN_PJL_STATUS                 = 1606u                 /* pjl status error                    */


} USBH_ERR;



#endif
