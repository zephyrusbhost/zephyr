/*
 * Copyright 2004-2020 Silicon Laboratories Inc. www.silabs.com
 *
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: APACHE-2.0
 */

#ifndef ZEPHYR_USBH_CLASS_H_
#define ZEPHYR_USBH_CLASS_H_

#include <usbh_core.h>

#define USBH_CLASS_DEV_STATE_NONE 0u
#define USBH_CLASS_DEV_STATE_CONN 1u
#define USBH_CLASS_DEV_STATE_DISCONN 2u
#define USBH_CLASS_DEV_STATE_SUSPEND 3u

#define USBH_CLASS_DRV_TYPE_NONE 0u
#define USBH_CLASS_DRV_TYPE_IF_CLASS_DRV 1u
#define USBH_CLASS_DRV_TYPE_DEV_CLASS_DRV 2u

struct usbh_class_drv {
	uint8_t *name_ptr;                                      /* Name of the class driver.                            */

	void (*global_init)(int *p_err);                        /* Global initialization function.                      */

	void *(*probe_dev)(struct usbh_dev *p_dev, int *p_err); /* Probe device descriptor.                             */


	void *(*probe_if)(struct usbh_dev *p_dev,
			  struct usbh_if *p_if,
			  int *p_err);          /* Probe interface descriptor. */

	void (*suspend)(void *p_class_dev);     /* Called when bus suspends.                            */

	void (*resume)(void *p_class_dev);      /* Called when bus resumes.                             */

	void (*disconn)(void *p_class_dev);     /* Called when device is removed.                       */
};


typedef void (*usbh_class_notify_fnct)(void *p_class_dev,
				       uint8_t is_conn,
				       void *p_ctx);

struct usbh_class_drv_reg {
	const struct usbh_class_drv *class_drv_ptr;             /* Class driver structure                               */
	usbh_class_notify_fnct notify_fnct_ptr;                 /* Called when device connection status changes         */
	void *notify_arg_ptr;                                   /* Context of the notification funtion                  */
	uint8_t in_use;
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

#endif /* ZEPHYR_USBH_CLASS_H_ */
