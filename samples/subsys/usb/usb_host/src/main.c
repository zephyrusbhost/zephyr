/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#define DT_DRV_COMPAT atmel_sam0_usbh

#include <zephyr.h>
#include <sys/printk.h>
#include <bsp_usbh_template.h>
#include <usbh_msc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(main);

// static void App_USBH_HID_ClassDevNotify(void *p_class_dev, CPU_INT08U state,
// 					void *p_ctx)
// {
// 	USBH_HID_DEV *p_hid_dev;

// 	(void)p_ctx;
// 	p_hid_dev = (USBH_HID_DEV *)p_class_dev;

// 	switch (state) {
// 	case USBH_CLASS_DEV_STATE_CONN:
// 		LOG_INF("HID device connected"); /* ---------------- HID DEVICE CONN'D ----------------- */
// 		//APP_TRACE_INFO(("HID Demo: Device Connected\r\n"));
// 		//  App_USBH_HID_DevConn(p_hid_dev);
// 		break;

// 	case USBH_CLASS_DEV_STATE_DISCONN:
// 		LOG_INF("HID device disconnected"); /* ---------------- HID DEVICE CONN'D ----------------- */
// 		/* ---------------- HID DEVICE REMOVED ---------------- */
// 		//APP_TRACE_INFO(("HID Demo: Device Removed\r\n"));
// 		//  App_USBH_HID_DevDisconn(p_hid_dev);
// 		break;

// 	default:
// 		break;
// 	}
// }

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

	//printk("%d\n", DT_INST_PROP(0, buf_len));
	int err = usbh_init();
	printk("Init return %d\n", err);

	uint8_t hc_nbr =
		usbh_hc_add(&USBH_ATSAMX_HCD_DrvAPI,
			    &USBH_ATSAMX_HCD_RH_API, &USBH_DrvBSP_Template,
			    &err);

	err = usbh_reg_class_drv(&USBH_MSC_ClassDrv, App_USBH_MSC_ClassNotify,
				 NULL);

	printk("HC_add return %d\n", hc_nbr);
	err = usbh_hc_start(hc_nbr);
	if (err != 0) {
		printk("error start hc %d\n", err);
	}

	LOG_INF("init all done");

}
