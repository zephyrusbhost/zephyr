/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <usbh_hc_cfg.h>
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
	USBH_MSC_DEV *p_msc_dev;
	USBH_ERR usb_err;
	// CPU_INT32U unit_nbr;

	(void)p_ctx;
	LOG_ERR("state %d", is_conn);
	p_msc_dev = (USBH_MSC_DEV *)p_class_dev;
	if (is_conn == USBH_CLASS_DEV_STATE_CONN) {
		LOG_INF("MSC connected");
		usb_err = USBH_MSC_RefAdd(p_msc_dev);
		if (usb_err != USBH_ERR_NONE) {
			LOG_ERR("Cannot add new MSC reference w/err: %d\r\n",
				usb_err);
			return;
		}
	} else{
		LOG_INF("MSC disconn");
		usb_err = USBH_MSC_RefRel(p_msc_dev);
		if (usb_err != USBH_ERR_NONE) {
			LOG_ERR("Cannot release MSC reference w/err: %d\r\n",
				usb_err);
			return;
		}
	}
	int lun = USBH_MSC_MaxLUN_Get(p_msc_dev, &usb_err);
	if (usb_err == USBH_ERR_NONE) {
		uint32_t blocks = 0;
		uint32_t blocksize = 0;
		USBH_MSC_CapacityRd(p_msc_dev, lun, &blocks, &blocksize);
		LOG_INF("blocks %d blockSize %d", blocks, blocksize);
		if (USBH_MSC_Init(p_msc_dev, lun) == USBH_ERR_NONE) {
			LOG_INF("msc initialized");
		}
	}
}
void main(void)
{
	USBH_ERR err = USBH_Init(NULL, NULL);
	printk("Init return %d\n", err);

	uint8_t hc_nbr =
		USBH_HC_Add(&USBH_HC_TemplateCfg, &USBH_ATSAMX_HCD_DrvAPI,
			    &USBH_ATSAMX_HCD_RH_API, &USBH_DrvBSP_Template,
			    &err);

	err = USBH_ClassDrvReg(
		&USBH_MSC_ClassDrv, /* Register MSC driver.                                 */
		App_USBH_MSC_ClassNotify, (void *)0);

	printk("HC_add return %d\n", hc_nbr);
	err = USBH_HC_Start(hc_nbr);
	if (err != USBH_ERR_NONE) {
		printk("error start hc %d\n", err);
	}

	LOG_INF("init all done");

}
