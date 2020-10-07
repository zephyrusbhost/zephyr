/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <usbh_msc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(main);

static void App_USBH_MSC_ClassNotify(void *p_class_dev, uint8_t is_conn,
				     void *p_ctx)
{
	struct usbh_msc_dev *p_msc_dev;
	int usb_err;

	(void)p_ctx;
	LOG_ERR("state %d", is_conn);
	p_msc_dev = (struct usbh_msc_dev *)p_class_dev;
	if (is_conn == USBH_CLASS_DEV_STATE_CONN) {
		LOG_INF("MSC connected");
		usb_err = usbh_msc_ref_add(p_msc_dev);
		if (usb_err != 0) {
			LOG_ERR("Cannot add new MSC reference w/err: %d\r\n",
				usb_err);
			return;
		}
	} else{
		LOG_INF("MSC disconn");
		usb_err = usbh_msc_ref_rel(p_msc_dev);
		if (usb_err != 0) {
			LOG_ERR("Cannot release MSC reference w/err: %d\r\n",
				usb_err);
			return;
		}
	}
	int lun = usbh_msc_max_lun_get(p_msc_dev, &usb_err);
	if (usb_err == 0) {
		uint32_t blocks = 0;
		uint32_t blocksize = 0;
		usbh_msc_capacity_rd(p_msc_dev, lun, &blocks, &blocksize);
		LOG_INF("blocks %d blockSize %d", blocks, blocksize);
		if (usbh_msc_init(p_msc_dev, lun) == 0) {
			LOG_INF("msc initialized");
		}
	}
}
void main(void)
{

	int err = usbh_init();
	printk("Init return %d\n", err);

	uint8_t hc_nbr =
		usbh_hc_add(&err);

	err = usbh_reg_class_drv(&USBH_MSC_ClassDrv, App_USBH_MSC_ClassNotify,
				 NULL);

	printk("HC_add return %d\n", hc_nbr);
	err = usbh_hc_start(hc_nbr);
	if (err != 0) {
		printk("error start hc %d\n", err);
	}
}
