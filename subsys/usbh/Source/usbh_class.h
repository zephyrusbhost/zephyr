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


#include <usbh_core.h>

#define USBH_CLASS_DEV_STATE_NONE 0u
#define USBH_CLASS_DEV_STATE_CONN 1u
#define USBH_CLASS_DEV_STATE_DISCONN 2u
#define USBH_CLASS_DEV_STATE_SUSPEND 3u

#define USBH_CLASS_DRV_TYPE_NONE 0u
#define USBH_CLASS_DRV_TYPE_IF_CLASS_DRV 1u
#define USBH_CLASS_DRV_TYPE_DEV_CLASS_DRV 2u

struct usbh_class_drv {
	uint8_t *name_ptr;               /* Name of the class driver.                            */

	void (*global_init)(int *p_err); /* Global initialization function.                      */

	void *(*probe_dev)(struct usbh_dev *p_dev,
	                  /* Probe device descriptor.                             */
			  int *p_err);

	void *(*probe_if)(struct usbh_dev *p_dev,
	                 /* Probe interface descriptor.                          */
			 struct usbh_if *p_if,
			 int *p_err);

	void (*suspend)(void *p_class_dev);     /* Called when bus suspends.                            */

	void (*resume)(void *p_class_dev);      /* Called when bus resumes.                             */

	void (*disconn)(void *p_class_dev);     /* Called when device is removed.                       */
};


typedef void (*usbh_class_notify_fnct)(void *p_class_dev,
				       uint8_t is_conn,
				       void *p_ctx);

struct usbh_class_drv_reg {
	const struct usbh_class_drv *ClassDrvPtr;       /* Class driver structure                               */
	usbh_class_notify_fnct NotifyFnctPtr;           /* Called when device connection status changes         */
	void *NotifyArgPtr;                             /* Context of the notification funtion                  */
	uint8_t InUse;
};


extern struct usbh_class_drv_reg usbh_class_drv_list[];


int usbh_reg_class_drv(const struct usbh_class_drv *p_class_drv,
		       usbh_class_notify_fnct class_notify_fnct,
		       void *p_class_notify_ctx);

int usbh_class_drv_unreg(const struct usbh_class_drv *p_class_drv);

void usbh_class_suspend(struct usbh_dev *p_dev);

void usbh_class_resume(struct usbh_dev *p_dev);

int usbh_class_drv_conn(struct usbh_dev *p_dev);

void usbh_class_drv_disconn(struct usbh_dev *p_dev);

#endif /* USBH_CLASS_H_ */
