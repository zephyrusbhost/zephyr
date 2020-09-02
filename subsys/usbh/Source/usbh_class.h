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

#ifndef USBH_CLASS_H_
#define USBH_CLASS_H_


#include "usbh_core.h"
/*
#ifdef USBH_CLASS_MODULE
#define USBH_CLASS_EXT
#else
#define USBH_CLASS_EXT extern
#endif
*/


#define USBH_CLASS_DEV_STATE_NONE 0u
#define USBH_CLASS_DEV_STATE_CONN 1u
#define USBH_CLASS_DEV_STATE_DISCONN 2u
#define USBH_CLASS_DEV_STATE_SUSPEND 3u

#define USBH_CLASS_DRV_TYPE_NONE 0u
#define USBH_CLASS_DRV_TYPE_IF_CLASS_DRV 1u
#define USBH_CLASS_DRV_TYPE_DEV_CLASS_DRV 2u

struct usbh_class_drv
{
    uint8_t *NamePtr; /* Name of the class driver.                            */

    void (*GlobalInit)(USBH_ERR *p_err); /* Global initialization function.                      */

    void *(*ProbeDev)(struct usbh_dev *p_dev,
                      /* Probe device descriptor.                             */
                      USBH_ERR *p_err);

    void *(*ProbeIF)(struct usbh_dev *p_dev,
                     /* Probe interface descriptor.                          */
                     struct usbh_if *p_if,
                     USBH_ERR *p_err);

    void (*Suspend)(void *p_class_dev); /* Called when bus suspends.                            */

    void (*Resume)(void *p_class_dev); /* Called when bus resumes.                             */

    void (*Disconn)(void *p_class_dev); /* Called when device is removed.                       */
};


typedef void (*USBH_CLASS_NOTIFY_FNCT)(void *p_class_dev,
                                       uint8_t is_conn,
                                       void *p_ctx);

struct usbh_class_drv_reg
{
    const struct usbh_class_drv *ClassDrvPtr;          /* Class driver structure                               */
    USBH_CLASS_NOTIFY_FNCT NotifyFnctPtr; /* Called when device connection status changes         */
    void *NotifyArgPtr;                   /* Context of the notification funtion                  */
    uint8_t InUse;
};


extern struct usbh_class_drv_reg usbh_class_drv_list[];


USBH_ERR usbh_reg_class_drv(const struct usbh_class_drv *p_class_drv,
			    USBH_CLASS_NOTIFY_FNCT class_notify_fnct,
			    void *p_class_notify_ctx);

USBH_ERR usbh_class_drv_unreg(const struct usbh_class_drv *p_class_drv);

void usbh_class_suspend(struct usbh_dev *p_dev);

void usbh_class_resume(struct usbh_dev *p_dev);

USBH_ERR usbh_class_drv_conn(struct usbh_dev *p_dev);

void usbh_class_drv_disconn(struct usbh_dev *p_dev);

#endif /* USBH_CLASS_H_ */
