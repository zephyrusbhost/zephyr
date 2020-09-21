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
 *                                       USB HOST HUB OPERATIONS
 *
 * Filename : usbh_hub.c
 * Version  : V3.42.00
 *********************************************************************************************************
 */


#include <usbh_hub.h>
#include <usbh_core.h>
#include <usbh_class.h>
#include <sys/byteorder.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(hub);


/*
 ********************************************************************************************************
 *                                     ROOT HUB DEVICE DESCRIPTOR
 ********************************************************************************************************
 */

static const uint8_t usbh_hub_rh_dev_desc[18] = {
	USBH_LEN_DESC_DEV,              /* b_length                                              */
	USBH_DESC_TYPE_DEV,             /* b_desc_type: Device                              */
	0x10u, 0x01u,                   /* bcd_usb: v1.1                                         */
	USBH_CLASS_CODE_HUB,            /* b_device_class: HUB_CLASSCODE                          */
	USBH_SUBCLASS_CODE_USE_IF_DESC, /* b_device_sub_class                                      */
	USBH_PROTOCOL_CODE_USE_IF_DESC, /* b_device_protocol                                      */
	0x40u,                          /* b_max_packet_size_zero: 64 Bytes                            */
	0x00u, 0x00u,                   /* id_vendor                                             */
	0x00u, 0x00u,                   /* id_product                                            */
	0x00u, 0x00u,                   /* bcd_device                                            */
	0x00u,                          /* i_manufacturer                                        */
	0x00u,                          /* i_product                                             */
	0x00u,                          /* i_serial_number                                        */
	0x01u                           /* bNumConfigurations                                   */
};

/*
 ********************************************************************************************************
 *                                  ROOT HUB CONFIGURATION DESCRIPTOR
 ********************************************************************************************************
 */

static const uint8_t usbh_hub_rh_fs_cfg_desc[] = {
	/* ------------- CONFIGURATION DESCRIPTOR ------------- */
	USBH_LEN_DESC_CFG,      /* b_length                                              */
	USBH_DESC_TYPE_CFG,     /* b_desc_type CONFIGURATION                        */
	0x19u, 0x00u,           /* le16 w_total_length                                    */
	0x01u,                  /* bNumInterfaces                                       */
	0x01u,                  /* b_cfg_value                                  */
	0x00u,                  /* i_cfg                                       */
	0xC0u,                  /* bm_attributes -> Self-powered|Remote wakeup           */
	0x00u,                  /* b_max_pwr                                            */

	/* --------------- INTERFACE DESCRIPTOR --------------- */
	USBH_LEN_DESC_IF,       /* b_length                                              */
	USBH_DESC_TYPE_IF,      /* b_desc_type: Interface                           */
	0x00u,                  /* bInterfaceNumber                                     */
	0x00u,                  /* bAlternateSetting                                    */
	0x01u,                  /* bNumEndpoints                                        */
	USBH_CLASS_CODE_HUB,    /* bInterfaceClass HUB_CLASSCODE                        */
	0x00u,                  /* bInterfaceSubClass                                   */
	0x00u,                  /* bInterfaceProtocol                                   */
	0x00u,                  /* iInterface                                           */

	/* --------------- ENDPOINT DESCRIPTOR ---------------- */
	USBH_LEN_DESC_EP,       /* b_length                                              */
	USBH_DESC_TYPE_EP,      /* b_desc_type: Endpoint                            */
	0x81u,                  /* bEndpointAddress: IN Endpoint 1                      */
	0x03u,                  /* bm_attributes Interrupt                               */
	0x08u, 0x00u,           /* wMaxPacketSize                                       */
	0x1u                    /* bInterval                                            */
};

/*
 ********************************************************************************************************
 *                                     ROOT HUB STRING DESCRIPTOR
 *
 * Note(s):  (1) A USB device can store strings in multiple languages. i.e. the string index defines which
 *               word or phrase should be communicated to the user and the LANGID code identifies to a
 *               device, which language to retrieve that word or phase for. The LANGIDs are defined in the
 *               document called "Language Identifiers (LANGIDs), 3/29/00, Version 1.0" available at
 *               http://www.usb.org/developers/docs/USB_LANGIDs.pdf.
 ********************************************************************************************************
 */

static const uint8_t usbh_hub_rh_lang_id[] = {
	0x04u,
	USBH_DESC_TYPE_STR,
	0x09u, 0x04u, /* Identifer for English (United States). See Note #1.  */
};

static uint8_t usbh_hub_desc_buf[USBH_HUB_MAX_DESC_LEN];
static struct usbh_hub_dev usbh_hub_arr[USBH_CFG_MAX_HUBS];
static int8_t hub_count = USBH_CFG_MAX_HUBS;
static volatile struct usbh_hub_dev *usbh_hub_head_ptr;
static volatile struct usbh_hub_dev *usbh_hub_tail_ptr;
static struct k_sem usbh_hub_event_sem;


static void usbh_hub_class_init(int *p_err);

static void *usbh_hub_if_probe(struct usbh_dev *p_dev,
			       struct usbh_if *p_if,
			       int *p_err);

static void usbh_hub_suspend(void *p_class_dev);

static void usbh_hub_resume(void *p_class_dev);

static void usbh_hub_disconn(void *p_class_dev);

static int usbh_hub_init(struct usbh_hub_dev *p_hub_dev);

static void usbh_hub_uninit(struct usbh_hub_dev *p_hub_dev);

static int usbh_hub_ep_open(struct usbh_hub_dev *p_hub_dev);

static void usbh_hub_ep_close(struct usbh_hub_dev *p_hub_dev);

static int usbh_hub_event_req(struct usbh_hub_dev *p_hub_dev);

static void usbh_hub_isr_cb(struct usbh_ep *p_ep,
			 void *p_buf,
			 uint32_t buf_len,
			 uint32_t xfer_len,
			 void *p_arg,
			 int err);

static void usbh_hub_event_proc(void);

static int usbh_hub_desc_get(struct usbh_hub_dev *p_hub_dev);

static int usbh_hub_ports_init(struct usbh_hub_dev *p_hub_dev);

static int usbh_hub_port_status_get(struct usbh_hub_dev *p_hub_dev,
				       uint16_t port_nbr,
				       struct usbh_hub_port_status *p_port_status);

static int usbh_hub_port_reset_set(struct usbh_hub_dev *p_hub_dev,
				      uint16_t port_nbr);

static int usbh_hub_port_rst_chng_clr(struct usbh_hub_dev *p_hub_dev,
					uint16_t port_nbr);

static int usbh_hub_port_en_chng_clr(struct usbh_hub_dev *p_hub_dev,
				       uint16_t port_nbr);

static int usbh_hub_port_conn_chng_clr(struct usbh_hub_dev *p_hub_dev,
					 uint16_t port_nbr);

static int usbh_hub_port_pwr_set(struct usbh_hub_dev *p_hub_dev,
				    uint16_t port_nbr);

static int usbh_hub_port_susp_clr(struct usbh_hub_dev *p_hub_dev,
					uint16_t port_nbr);

static int usbh_hub_port_en_clr(struct usbh_hub_dev *p_hub_dev,
				   uint16_t port_nbr);

static int usbh_hub_port_en_set(struct usbh_hub_dev *p_hub_dev,
				   uint16_t port_nbr);

static void usbh_hub_clr(struct usbh_hub_dev *p_hub_dev);

static int usbh_hub_ref_add(struct usbh_hub_dev *p_hub_dev);

static int usbh_hub_ref_rel(struct usbh_hub_dev *p_hub_dev);

/*
 *********************************************************************************************************
 *                                          HUB CLASS DRIVER
 *********************************************************************************************************
 */

const struct usbh_class_drv usbh_hub_drv = {
	(uint8_t *)"HUB",
	usbh_hub_class_init,
	NULL,
	usbh_hub_if_probe,
	usbh_hub_suspend,
	usbh_hub_resume,
	usbh_hub_disconn
};

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                           GLOBAL FUNCTION
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                        USBH_HUB_EventTask()
 *
 * Description : Task that process HUB Events.
 *
 * Argument(s) : None.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

void usbh_hub_event_task(void *p_arg, void *p_arg2, void *p_arg3)
{
	while (1) {
		k_sem_take(&usbh_hub_event_sem, K_FOREVER);
		usbh_hub_event_proc();
	}
}

/*
 *********************************************************************************************************
 *                                         USBH_HUB_PortDis()
 *
 * Description : Disable given port on hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : int_NONE,                          If port is successfully disabled.
 *               USBH_ERR_INVALID_ARG,                   If invalid parameter passed to 'p_hub_dev'.
 *
 *                                                       ----- RETURNED BY usbh_hub_port_en_clr() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : None
 *********************************************************************************************************
 */

int usbh_hub_port_dis(struct usbh_hub_dev *p_hub_dev,
			   uint16_t port_nbr)
{
	int err;

	if (p_hub_dev == NULL) {
		return EINVAL;
	}

	err = usbh_hub_port_en_clr(p_hub_dev, port_nbr);

	return err;
}

/*
 *********************************************************************************************************
 *                                          USBH_HUB_PortEn()
 *
 * Description : Enable given port on hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          If port is successfully enabled.
 *               USBH_ERR_INVALID_ARG,                   If invalid parameter passed to 'p_hub_dev'.
 *
 *                                                       ----- RETURNED BY usbh_hub_port_en_set() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_hub_port_en(struct usbh_hub_dev *p_hub_dev,
			  uint16_t port_nbr)
{
	int err;

	if (p_hub_dev == NULL) {
		return EINVAL;
	}

	err = usbh_hub_port_en_set(p_hub_dev, port_nbr);

	return err;
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                           LOCAL FUNCTIONS
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                        usbh_hub_class_init()
 *
 * Description : Initializes all USBH_HUB_DEV structures, device lists and HUB pool.
 *
 * Argument(s) : p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                   USBH_ERR_NONE                   If initialization is successful.
 *                   USBH_ERR_ALLOC                  If failed to create hub memory pool.
 *
 *                                                   ----- RETURNED BY USBH_OS_SemCreate() : -----
 *                   USBH_ERR_OS_SIGNAL_CREATE,      If semaphore creation failed.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_hub_class_init(int *p_err)
{
	uint8_t hub_ix;

	for (hub_ix = 0; hub_ix < USBH_CFG_MAX_HUBS; hub_ix++) { /* Clr all HUB dev structs.                             */
		usbh_hub_clr(&usbh_hub_arr[hub_ix]);
	}
	hub_count = (USBH_CFG_MAX_HUBS - 1);

	*p_err = k_sem_init(&usbh_hub_event_sem, 0, USBH_OS_SEM_REQUIRED);

	usbh_hub_head_ptr = NULL;
	usbh_hub_tail_ptr = NULL;

	memset(usbh_hub_desc_buf, 0, USBH_HUB_MAX_DESC_LEN);
}

/*
 *********************************************************************************************************
 *                                         usbh_hub_if_probe()
 *
 * Description : Determine whether connected device implements hub class by examining it's interface
 *               descriptor.
 *
 * Argument(s) : p_dev   Pointer to device.
 *
 *               p_if    Pointer to interface.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                   USBH_ERR_NONE                           Device implements hub class.
 *                   USBH_ERR_DEV_ALLOC                      No more space in hub memory pool.
 *                   USBH_ERR_CLASS_DRV_NOT_FOUND            Device does not implement hub class.
 *
 *                                                           ----- RETURNED BY USBH_IF_DescGet() : -----
 *                   USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'alt_ix'.
 *
 *                                                           ----- RETURNED BY usbh_hub_init() : -----
 *                   USBH_ERR_DESC_INVALID,                  if hub descriptor is invalid.
 *                   USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *                   USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *                   USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *                   USBH_ERR_HC_IO,                         Root hub input/output error.
 *                   USBH_ERR_EP_STALL,                      Root hub does not support request.
 *                   USBH_ERR_EP_ALLOC,                      If USBH_CFG_MAX_NBR_EPS reached.
 *                   USBH_ERR_EP_NOT_FOUND,                  If endpoint with given type and direction not found.
 *                   USBH_ERR_OS_SIGNAL_CREATE,              if mutex or semaphore creation failed.
 *                   USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *                   USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *                   USBH_ERR_HC_IO,                         Root hub input/output error.
 *                   USBH_ERR_EP_INVALID_TYPE                If endpoint type is not interrupt or direction is not IN.
 *                   USBH_ERR_ALLOC                          If URB cannot be allocated.
 *                   Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Pointer to hub device structure,        If probe is successful.
 *               0,                                      Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void *usbh_hub_if_probe(struct usbh_dev *p_dev,
			       struct usbh_if *p_if,
			       int *p_err)
{
	LOG_INF("hub if");
	struct usbh_hub_dev *p_hub_dev;
	struct usbh_if_desc if_desc;

	p_hub_dev = NULL;
	*p_err = usbh_if_desc_get(p_if, /* Get IF desc.                                         */
				  0,
				  &if_desc);
	if (*p_err != 0) {
		return NULL;
	}

	if (if_desc.bInterfaceClass == USBH_CLASS_CODE_HUB) { /* If IF is HUB, alloc HUB dev.                         */
		if (hub_count < 0) {
			*p_err = EAGAIN;
			return NULL;
		} else   {
			p_hub_dev = &usbh_hub_arr[hub_count--];
		}

		usbh_hub_clr(p_hub_dev);
		usbh_hub_ref_add(p_hub_dev);

		p_hub_dev->State = (uint8_t)USBH_CLASS_DEV_STATE_CONN;
		p_hub_dev->DevPtr = p_dev;
		p_hub_dev->IF_Ptr = p_if;
		p_hub_dev->ErrCnt = 0;

		if ((p_dev->IsRootHub == true) &&
		    (p_dev->HC_Ptr->IsVirRootHub == true)) {
			p_dev->HC_Ptr->RH_ClassDevPtr = p_hub_dev;
		}

		*p_err = usbh_hub_init(p_hub_dev); /* Init HUB.                                            */
		if (*p_err != 0) {
			usbh_hub_ref_rel(p_hub_dev);
		}
	} else   {
		*p_err = EAGAIN;
	}

	return ((void *)p_hub_dev);
}

/*
 *********************************************************************************************************
 *                                         usbh_hub_suspend()
 *
 * Description : Suspend given hub and all devices connected to it.
 *
 * Argument(s) : p_class_dev     Pointer to hub structure.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_hub_suspend(void *p_class_dev)
{
	uint16_t nbr_ports;
	uint16_t port_ix;
	struct usbh_dev *p_dev;
	struct usbh_hub_dev *p_hub_dev;

	p_hub_dev = (struct usbh_hub_dev *)p_class_dev;
	nbr_ports = MIN(p_hub_dev->Desc.b_nbr_ports, USBH_CFG_MAX_HUB_PORTS);

	for (port_ix = 0; port_ix < nbr_ports; port_ix++) {
		p_dev = (struct usbh_dev *)p_hub_dev->DevPtrList[port_ix];

		if (p_dev != NULL) {
			usbh_class_suspend(p_dev);
		}
	}
}

/*
 *********************************************************************************************************
 *                                          usbh_hub_resume()
 *
 * Description : Resume given hub and all devices connected to it.
 *
 * Argument(s) : p_class_dev     Pointer to hub device.
 *
 * Return(s)   : None.
 *
 * Note(s)     : (1) According to 'Universal Serial Bus Specification Revision 2.0', Section 11.5.1.10,
 *                   "[the resuming state] is a timed state with a nominal duration of 20ms"
 *********************************************************************************************************
 */

static void usbh_hub_resume(void *p_class_dev)
{
	uint16_t nbr_ports;
	uint16_t port_ix;
	struct usbh_dev *p_dev;
	struct usbh_hub_dev *p_hub_dev;
	struct usbh_hub_port_status port_status;

	p_hub_dev = (struct usbh_hub_dev *)p_class_dev;
	nbr_ports = MIN(p_hub_dev->Desc.b_nbr_ports, USBH_CFG_MAX_HUB_PORTS);

	for (port_ix = 0; port_ix < nbr_ports; port_ix++) {
		usbh_hub_port_susp_clr(p_hub_dev, port_ix + 1); /* En resume signaling on port.                         */
	}

	k_sleep(K_MSEC(20u + 12u)); /* See Note (1).                                        */

	for (port_ix = 0; port_ix < nbr_ports; port_ix++) {
		p_dev = p_hub_dev->DevPtrList[port_ix];

		if (p_dev != NULL) {
			usbh_class_resume(p_dev);
		} else   {/* Get port status info.                                */
			usbh_hub_port_status_get(p_hub_dev,
						     port_ix + 1,
						     &port_status);

			if ((port_status.w_port_status & USBH_HUB_STATUS_PORT_CONN) != 0) {
				usbh_hub_port_reset_set(p_hub_dev,
							    port_ix + 1);
			}
		}
	}
}

/*
 *********************************************************************************************************
 *                                         usbh_hub_disconn()
 *
 * Description : Disconnect given hub.
 *
 * Argument(s) : p_class_dev      Pointer to hub device.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_hub_disconn(void *p_class_dev)
{
	struct usbh_hub_dev *p_hub_dev;

	p_hub_dev = (struct usbh_hub_dev *)p_class_dev;
	p_hub_dev->State = USBH_CLASS_DEV_STATE_DISCONN;

	usbh_hub_uninit(p_hub_dev);
	usbh_hub_ref_rel(p_hub_dev);
}

/*
 *********************************************************************************************************
 *                                           usbh_hub_init()
 *
 * Description : Opens the endpoints, reads hub descriptor, initializes ports and submits request to
 *               start receiving hub events.
 *
 * Argument(s) : p_hub_dev       Pointer to hub structure.
 *
 * Return(s)   : USBH_ERR_NONE,                          If hub is successfully initialized.
 *
 *                                                       ----- RETURNED BY usbh_hub_desc_get() : -----
 *               USBH_ERR_DESC_INVALID,                  if hub descriptor is invalid.
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 *                                                       ----- RETURNED BY usbh_hub_ep_open() : -----
 *               USBH_ERR_EP_ALLOC,                      If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND,                  If endpoint with given type and direction not found.
 *               USBH_ERR_OS_SIGNAL_CREATE,              if mutex or semaphore creation failed.
 *               Host controller drivers error,          Otherwise.
 *
 *                                                       ----- RETURNED BY usbh_hub_ports_init() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 *                                                       ----- RETURNED BY usbh_hub_event_req() : -----
 *               USBH_ERR_INVALID_ARG                    If invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_TYPE                If endpoint type is not interrupt or direction is not IN.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static int usbh_hub_init(struct usbh_hub_dev *p_hub_dev)
{
	int err;

	err = usbh_hub_ep_open(p_hub_dev); /* Open intr EP.                                        */
	if (err != 0) {
		return err;
	}

	err = usbh_hub_desc_get(p_hub_dev); /* Get hub desc.                                        */
	if (err != 0) {
		return err;
	}

	err = usbh_hub_ports_init(p_hub_dev); /* Init hub ports.                                      */
	if (err != 0) {
		return err;
	}

	err = usbh_hub_event_req(p_hub_dev); /* Start receiving hub evts.                            */

	return err;
}

/*
 *********************************************************************************************************
 *                                          usbh_hub_uninit()
 *
 * Description : Uninitialize given hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_hub_uninit(struct usbh_hub_dev *p_hub_dev)
{
	uint16_t nbr_ports;
	uint16_t port_ix;
	struct usbh_dev *p_dev;

	usbh_hub_ep_close(p_hub_dev);
	nbr_ports = MIN(p_hub_dev->Desc.b_nbr_ports, USBH_CFG_MAX_HUB_PORTS);

	for (port_ix = 0; port_ix < nbr_ports; port_ix++) {
		p_dev = (struct usbh_dev *)p_hub_dev->DevPtrList[port_ix];

		if (p_dev != NULL) {
			usbh_dev_disconn(p_dev);
			p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount++;

			p_hub_dev->DevPtrList[port_ix] = NULL;
		}
	}
}

/*
 *********************************************************************************************************
 *                                         usbh_hub_ep_open()
 *
 * Description : Open interrupt endpoint required to receive hub events.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 * Return(s)   : USBH_ERR_NONE,                      if the endpoints are opened.
 *
 *                                                   ----- RETURNED BY USBH_IntrInOpen() : -----
 *               USBH_ERR_EP_ALLOC,                  If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND,              If endpoint with given type and direction not found.
 *               USBH_ERR_OS_SIGNAL_CREATE,          if mutex or semaphore creation failed.
 *               Host controller drivers error,      Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static int usbh_hub_ep_open(struct usbh_hub_dev *p_hub_dev)
{
	int err;
	struct usbh_dev *p_dev;
	struct usbh_if *p_if;

	p_dev = p_hub_dev->DevPtr;
	p_if = p_hub_dev->IF_Ptr;

	err = usbh_intr_in_open(p_dev, /* Find and open hub intr EP.                           */
				p_if,
				&p_hub_dev->IntrEP);

	return err;
}

/*
 *********************************************************************************************************
 *                                         usbh_hub_ep_close()
 *
 * Description : Close interrupt endpoint.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_hub_ep_close(struct usbh_hub_dev *p_hub_dev)
{
	usbh_ep_close(&p_hub_dev->IntrEP); /* Close hub intr EP.                                   */
}

/*
 *********************************************************************************************************
 *                                         usbh_hub_event_req()
 *
 * Description : Issue an asynchronous interrupt request to receive hub events.
 *
 * Argument(s) : p_hub_dev            Pointer to the hub device structure.
 *
 * Return(s)   : USBH_ERR_NONE                           If hub events are started
 *
 *                                                       ----- RETURNED BY USBH_IntrRxAsync() : -----
 *               USBH_ERR_NONE                           If request is successfully submitted.
 *               USBH_ERR_INVALID_ARG                    If invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_TYPE                If endpoint type is not interrupt or direction is not IN.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) Hub reports only as many bits as there are ports on the hub,
 *                   subject to the byte-granularity requirement (i.e., round up to the nearest byte)
 *                  (See section 11.12.4, USB 2.0 spec)
 *********************************************************************************************************
 */

static int usbh_hub_event_req(struct usbh_hub_dev *p_hub_dev)
{
	uint32_t len;
	bool valid;
	const struct usbh_hc_rh_api *p_rh_api;
	int err;
	struct usbh_dev *p_dev;

	p_dev = p_hub_dev->DevPtr;

	if ((p_dev->IsRootHub == true) && /* Chk if RH fncts are supported before calling HCD.    */
	    (p_dev->HC_Ptr->IsVirRootHub == true)) {

		p_rh_api = p_dev->HC_Ptr->HC_Drv.RH_API_Ptr;
		valid = p_rh_api->IntEn(&p_dev->HC_Ptr->HC_Drv);
		if (valid != 1) {
			return EIO;
		} else   {
			return 0;
		}
	}

	len = (p_hub_dev->Desc.b_nbr_ports / 8) + 1;    /* See Note (1).                                        */
	err = usbh_intr_rx_async(&p_hub_dev->IntrEP,    /* Start receiving hub events.                          */
				 (void *)p_hub_dev->HubIntrBuf,
				 len,
				 usbh_hub_isr_cb,
				 (void *)p_hub_dev);
	return err;
}

/*
 *********************************************************************************************************
 *                                           usbh_hub_isr_cb()
 *
 * Description : Handles hub interrupt.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to data buffer.
 *
 *               buf_len         Buffer length in octets.
 *
 *               xfer_len        Transfered length.
 *
 *               p_arg           Context Pointer.
 *
 *               err             Error code.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_hub_isr_cb(struct usbh_ep *p_ep,
			 void *p_buf,
			 uint32_t buf_len,
			 uint32_t xfer_len,
			 void *p_arg,
			 int err)
{
	struct usbh_hub_dev *p_hub_dev;
	int key;

	(void)buf_len;
	(void)p_buf;
	(void)p_ep;
	(void)xfer_len;

	p_hub_dev = (struct usbh_hub_dev *)p_arg;

	if (err != 0) {
		if (p_hub_dev->State == USBH_CLASS_DEV_STATE_CONN) {
			if (p_hub_dev->ErrCnt < 3u) {
				LOG_ERR("usbh_hub_isr_cb() fails. Err=%d errcnt=%d\r\n",
					err,
					p_hub_dev->ErrCnt);

				p_hub_dev->ErrCnt++;
				usbh_hub_event_req(p_hub_dev); /* Retry URB.                                           */
			}
		}
		return;
	}

	p_hub_dev->ErrCnt = 0;

	usbh_hub_ref_add(p_hub_dev);

	key = irq_lock();
	if (usbh_hub_head_ptr == NULL) {
		usbh_hub_head_ptr = usbh_hub_tail_ptr = p_hub_dev;
	} else   {
		usbh_hub_tail_ptr->NxtPtr = p_hub_dev;
		usbh_hub_tail_ptr = p_hub_dev;
	}
	irq_unlock(key);

	k_sem_give(&usbh_hub_event_sem);
}

/*
 *********************************************************************************************************
 *                                       usbh_hub_event_proc()
 *
 * Description : Determine status of each of hub ports. Newly connected device will be reset & configured.
 *               Appropriate notifications & cleanup will be performed if a device has been disconnected.
 *
 * Argument(s) : None.
 *
 * Return(s)   : None.
 *
 * Note(s)     : (1) Some device require a delay following connection detection. This is related to the
 *                   debounce interval which ensures that power is stable at the device for at least 100 ms
 *                   before any requests will be sent to the device (See section 11.8.2, USB 2.0 spec).
 *
 *               (2) Open Host Controller Interface specification Release 1.0a states that Port Reset Status
 *                   Change bit is set at the end of 10 ms port reset signal. See section 7.4.4, PRSC field.
 *********************************************************************************************************
 */

static void usbh_hub_event_proc(void)
{
	uint16_t nbr_ports;
	uint16_t port_nbr;
	uint8_t dev_spd;
	struct usbh_hub_dev *p_hub_dev;
	struct usbh_hub_port_status port_status;
	struct usbh_dev *p_dev;
	int err;
	int key;

	key = irq_lock();
	p_hub_dev = (struct usbh_hub_dev *)usbh_hub_head_ptr;

	if (usbh_hub_head_ptr == usbh_hub_tail_ptr) {
		usbh_hub_head_ptr = NULL;
		usbh_hub_tail_ptr = NULL;
	} else   {
		usbh_hub_head_ptr = usbh_hub_head_ptr->NxtPtr;
	}
	irq_unlock(key);

	if (p_hub_dev == NULL) {
		return;
	}

	if (p_hub_dev->State == USBH_CLASS_DEV_STATE_DISCONN) {
		LOG_DBG("device state disconnected");
		err = usbh_hub_ref_rel(p_hub_dev);
		if (err != 0) {
			LOG_ERR("could not release reference %d", err);
		}  
		return;
	}

	port_nbr = 1;
	nbr_ports = MIN(p_hub_dev->Desc.b_nbr_ports, USBH_CFG_MAX_HUB_PORTS);

	while (port_nbr <= nbr_ports) {

		err = usbh_hub_port_status_get(p_hub_dev, /* Get port status info..                               */
					     port_nbr,
					     &port_status);
		if (err != 0) {
			break;
		}
		/* ------------- CONNECTION STATUS CHANGE ------------- */
		if (DEF_BIT_IS_SET(port_status.w_port_change, USBH_HUB_STATUS_C_PORT_CONN) == true) {
			LOG_DBG("connection status change");
			err = usbh_hub_port_conn_chng_clr(p_hub_dev, /* Clr port conn chng.                                  */
						       port_nbr);
			if (err != 0) {
				break;
			}
			/* -------------- DEV HAS BEEN CONNECTED -------------- */
			if (DEF_BIT_IS_SET(port_status.w_port_status, USBH_HUB_STATUS_PORT_CONN) == true) {

				LOG_DBG("Port %d : Device Connected.\r\n", port_nbr);

				p_hub_dev->ConnCnt = 0; /* Reset re-connection counter                          */
				p_dev = p_hub_dev->DevPtrList[port_nbr - 1];
				if (p_dev != NULL) {
					usbh_dev_disconn(p_dev);
					p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount++;
					p_hub_dev->DevPtrList[port_nbr - 1] = NULL;
				}

				k_sleep(K_MSEC(100u));                  /* See Notes #1.                                        */
				err = usbh_hub_port_reset_set(p_hub_dev,  /* Apply port reset.                                    */
							    port_nbr);
				if (err != 0) {
					break;
				}

				k_sleep(K_MSEC(USBH_HUB_DLY_DEV_RESET));        /* See Notes #2.                                        */
			  	continue;                                       /* Handle port reset status change.                     */
			} else   {/* --------------- DEV HAS BEEN REMOVED --------------- */
				LOG_DBG("device has been removed");
				k_sleep(K_MSEC(10u)); /* Wait for any pending I/O xfer to rtn err.            */

				p_dev = p_hub_dev->DevPtrList[port_nbr - 1];

				if (p_dev != NULL) {
					usbh_dev_disconn(p_dev);
					p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount++;

					p_hub_dev->DevPtrList[port_nbr - 1] = NULL;
				}
			}
		}
		/* ------------- PORT RESET STATUS CHANGE ------------- */
		if (DEF_BIT_IS_SET(port_status.w_port_change, USBH_HUB_STATUS_C_PORT_RESET) == true) {
			err = usbh_hub_port_rst_chng_clr(p_hub_dev, port_nbr);
			if (err != 0) {
				break;
			}
			/* Dev has been connected.                              */
			if (DEF_BIT_IS_SET(port_status.w_port_status, USBH_HUB_STATUS_PORT_CONN) == true) {

				err = usbh_hub_port_status_get(p_hub_dev, /* Get port status info.                                */
							     port_nbr,
							     &port_status);
				if (err != 0) {
					break;
				}

				/* Determine dev spd.                                   */
				if (DEF_BIT_IS_SET(port_status.w_port_status, USBH_HUB_STATUS_PORT_LOW_SPD) == true) {
					dev_spd = USBH_LOW_SPEED;
				} else if (DEF_BIT_IS_SET(port_status.w_port_status, USBH_HUB_STATUS_PORT_HIGH_SPD) == true) {
					dev_spd = USBH_HIGH_SPEED;
				} else   {
					dev_spd = USBH_FULL_SPEED;
				}

				LOG_DBG("Port %d : Port Reset complete, device speed is %s\r\n", port_nbr,
					(dev_spd == USBH_LOW_SPEED) ? "LOW Speed(1.5 Mb/Sec)" : (dev_spd == USBH_FULL_SPEED) ? "FULL Speed(12 Mb/Sec)" : "HIGH Speed(480 Mb/Sec)");

				p_dev = p_hub_dev->DevPtrList[port_nbr - 1];

				if (p_dev != NULL) {
					continue;
				}

				if (p_hub_dev->DevPtr->HC_Ptr->HostPtr->State == USBH_HOST_STATE_SUSPENDED) {
					continue;
				}
				if (p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount < 0) {
					usbh_hub_port_dis(p_hub_dev, port_nbr);
					usbh_hub_ref_rel(p_hub_dev);
					usbh_hub_event_req(p_hub_dev); /* Retry URB.                                           */

					return;
				} else   {
					p_dev = &p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevList[p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount--];
				}

				p_dev->DevSpd = dev_spd;
				p_dev->HubDevPtr = p_hub_dev->DevPtr;
				p_dev->PortNbr = port_nbr;
				p_dev->HC_Ptr = p_hub_dev->DevPtr->HC_Ptr;

				if (dev_spd == USBH_HIGH_SPEED) {
					p_dev->HubHS_Ptr = p_hub_dev;
				} else   {
					if (p_hub_dev->IntrEP.DevSpd == USBH_HIGH_SPEED) {
						p_dev->HubHS_Ptr = p_hub_dev;
					} else   {
						p_dev->HubHS_Ptr = p_hub_dev->DevPtr->HubHS_Ptr;
					}
				}

				k_sleep(K_MSEC(50u));
				err = usbh_dev_conn(p_dev); /* Conn dev.                                            */
				if (err != 0) {
					usbh_hub_port_dis(p_hub_dev, port_nbr);
					usbh_dev_disconn(p_dev);

					p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount++;

					if (p_hub_dev->ConnCnt < USBH_CFG_MAX_NUM_DEV_RECONN) {
						/*This condition may happen due to EP_STALL return      */
						err = usbh_hub_port_reset_set(p_hub_dev, /* Apply port reset.                                    */
									    port_nbr);
						if (err != 0) {
							break;
						}

						k_sleep(K_MSEC(USBH_HUB_DLY_DEV_RESET)); /* See Notes #2.                                        */
						p_hub_dev->ConnCnt++;
						;
						continue; /* Handle port reset status change.                     */
					} else   {
						p_hub_dev->DevPtrList[port_nbr - 1] = NULL;
					}
				} else   {
					p_hub_dev->DevPtrList[port_nbr - 1] = p_dev;
				}
			}
		}
		/* ------------ PORT ENABLE STATUS CHANGE ------------- */
		if (DEF_BIT_IS_SET(port_status.w_port_change, USBH_HUB_STATUS_C_PORT_EN) == true) {
			err = usbh_hub_port_en_chng_clr(p_hub_dev, port_nbr);
			if (err != 0) {
				break;
			}
		}
		port_nbr++;
	}
	usbh_hub_event_req(p_hub_dev); /* Retry URB.                                           */

	usbh_hub_ref_rel(p_hub_dev);
}

/*
 *********************************************************************************************************
 *                                         usbh_hub_desc_get()
 *
 * Description : Retrieve hub descriptor.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 * Return(s)   : USBH_ERR_NONE,                          if descriptor is successfully obtained.
 *               USBH_ERR_DESC_INVALID,                  if hub descriptor is invalid.
 *
 *                                                       ----- RETURNED BY USBH_CtrlRx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) GET HUB DESCRIPTOR standard request is defined in USB2.0 specification,
 *                   section 11.24.2.5. Read 2 bytes of hub descriptor. Offset 0 of Hub Descriptor
 *                   contains total length of descriptor and offset 1 represents descriptor type.
 *********************************************************************************************************
 */

static int usbh_hub_desc_get(struct usbh_hub_dev *p_hub_dev)
{
	int err;
	uint32_t len;
	uint32_t i;
	struct usbh_desc_hdr hdr;

	for (i = 0; i < USBH_CFG_STD_REQ_RETRY; i++) { /* Attempt to get desc hdr 3 times.                     */
		len = usbh_ctrl_rx(p_hub_dev->DevPtr,
				   USBH_REQ_GET_DESC, /* See Note (1).                                        */
				   (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_TYPE_CLASS),
				   (USBH_HUB_DESC_TYPE_HUB << 8),
				   0,
				   (void *)&hdr,
				   USBH_LEN_DESC_HDR,
				   USBH_HUB_TIMEOUT,
				   &err);
		if ((err == EBUSY) ||
		    (len == 0)) {
			usbh_ep_reset(p_hub_dev->DevPtr,
				      NULL);
		} else   {
			break;
		}
	}

	if (len != USBH_LEN_DESC_HDR) {
		return EINVAL;
	}

	if ((hdr.b_length == 0) ||
	    (hdr.b_length > USBH_HUB_MAX_DESC_LEN) ||
	    (hdr.b_desc_type != USBH_HUB_DESC_TYPE_HUB)) {
		return EINVAL;
	}

	for (i = 0; i < USBH_CFG_STD_REQ_RETRY; i++) { /* Attempt to get full desc 3 times.                    */
		len = usbh_ctrl_rx(p_hub_dev->DevPtr,
				   USBH_REQ_GET_DESC,
				   (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_TYPE_CLASS),
				   (USBH_HUB_DESC_TYPE_HUB << 8),
				   0,
				   (void *)usbh_hub_desc_buf,
				   hdr.b_length,
				   USBH_HUB_TIMEOUT,
				   &err);
		if ((err == EBUSY) ||
		    (len < hdr.b_length)) {
			usbh_ep_reset(p_hub_dev->DevPtr,
				      NULL);
		} else   {
			break;
		}
	}

	usbh_hub_parse_hub_desc(&p_hub_dev->Desc,
				usbh_hub_desc_buf);

	if (p_hub_dev->Desc.b_nbr_ports > USBH_CFG_MAX_HUB_PORTS) { /* Warns limit on hub port nbr to max cfg'd.            */
		LOG_WRN("Only ports [1..%d] are active.\r\n", USBH_CFG_MAX_HUB_PORTS);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                        usbh_hub_ports_init()
 *
 * Description : Enable power on each hub port & initialize device on each port.
 *
 * Argument(s) : p_hub_dev       Pointer to the hub.
 *
 * Return(s)   : USBH_ERR_NONE,                          if ports were successfully initialized.
 *
 *                                                       ----- RETURNED BY usbh_hub_port_pwr_set() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) USB2.0 specification states that the host must wait for (b_pwr_on_to_pwr_good * 2) ms
 *                   before accessing a powered port. See section 11.23.2.1, b_pwr_on_to_pwr_good field in hub
 *                   descriptor.
 *********************************************************************************************************
 */

static int usbh_hub_ports_init(struct usbh_hub_dev *p_hub_dev)
{
	int err;
	uint32_t i;
	uint16_t nbr_ports;

	nbr_ports = MIN(p_hub_dev->Desc.b_nbr_ports, USBH_CFG_MAX_HUB_PORTS);

	for (i = 0; i < nbr_ports; i++) {
		err = usbh_hub_port_pwr_set(p_hub_dev, i + 1); /* Set port pwr.                                      */

		if (err != 0) {
			LOG_ERR("PortPwrSet error");
			return err;
		}
		k_sleep(K_MSEC(p_hub_dev->Desc.b_pwr_on_to_pwr_good * 2)); /* See Note (1).                                       */
	}
	return 0;
}

/*
 *********************************************************************************************************
 *                                      usbh_hub_port_status_get()
 *
 * Description : Get port status on given hub.
 *
 * Argument(s) : p_ep             Pointer to hub device.
 *
 *               port_nbr         Port number.
 *
 *               p_port_status    Variable that will receive port status.
 *
 * Return(s)   : USBH_ERR_NONE,                          if request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlRx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) HUB class specific request GET PORT STATUS is defined in USB2.0 specification in
 *                   section 11.24.2.7.
 *********************************************************************************************************
 */

static int usbh_hub_port_status_get(struct usbh_hub_dev *p_hub_dev,
				       uint16_t port_nbr,
				       struct usbh_hub_port_status *p_port_status)
{
	uint8_t *p_buf;
	struct usbh_hub_port_status port_status;
	int err;

	p_buf = (uint8_t *)&port_status;

	usbh_ctrl_rx(p_hub_dev->DevPtr, /* See Note (1).                                        */
			   USBH_REQ_GET_STATUS,
			   (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   0,
			   port_nbr,
			   (void *)p_buf,
			   USBH_HUB_LEN_HUB_PORT_STATUS,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	} else   {
		p_port_status->w_port_status = sys_get_le16((uint8_t *)&port_status.w_port_status);
		p_port_status->w_port_change = sys_get_le16((uint8_t *)&port_status.w_port_change);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                       usbh_hub_port_reset_set()
 *
 * Description : Set port reset on given hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          if the request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The HUB class specific request SET PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.13
 *********************************************************************************************************
 */

static int usbh_hub_port_reset_set(struct usbh_hub_dev *p_hub_dev,
				      uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note (1).                                        */
			   USBH_REQ_SET_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_PORT_RESET,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                      usbh_hub_port_rst_chng_clr()
 *
 * Description : Clear port reset change on given hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          If request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) HUB class specific request CLEAR PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.2.
 *********************************************************************************************************
 */

static int usbh_hub_port_rst_chng_clr(struct usbh_hub_dev *p_hub_dev,
					uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note (1).                                        */
			   USBH_REQ_CLR_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_C_PORT_RESET,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                      usbh_hub_port_en_chng_clr()
 *
 * Description : Clear port enable change on given hub.
 *
 * Argument(s) : p_hub_dev        Pointer to hub device.
 *
 *               port_nbr         Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          if the request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN                        Unknown error occurred.
 *               USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) HUB class specific request CLEAR PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.2.
 *********************************************************************************************************
 */

static int usbh_hub_port_en_chng_clr(struct usbh_hub_dev *p_hub_dev,
				       uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note (1).                                        */
			   USBH_REQ_CLR_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_C_PORT_EN,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                     usbh_hub_port_conn_chng_clr()
 *
 * Description : Clear port connection change on given hub.
 *
 * Argument(s) : p_hub_dev        Pointer to hub device.
 *
 *               port_nbr         Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          if the request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN                        Unknown error occurred.
 *               USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The HUB class specific request CLEAR PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.2.
 *********************************************************************************************************
 */

static int usbh_hub_port_conn_chng_clr(struct usbh_hub_dev *p_hub_dev,
					 uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
			   USBH_REQ_CLR_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_C_PORT_CONN,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                         usbh_hub_port_pwr_set()
 *
 * Description : Set power on given hub and port.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          if the request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The HUB class specific request SET PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.13.
 *********************************************************************************************************
 */

static int usbh_hub_port_pwr_set(struct usbh_hub_dev *p_hub_dev,
				    uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
			   USBH_REQ_SET_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_PORT_PWR,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                      usbh_hub_port_susp_clr()
 *
 * Description : Clear port suspend on given hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          if the request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The HUB class specific request CLEAR PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.2.
 *********************************************************************************************************
 */

static int usbh_hub_port_susp_clr(struct usbh_hub_dev *p_hub_dev,
					uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
			   USBH_REQ_CLR_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_C_PORT_SUSPEND,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                        usbh_hub_port_en_clr()
 *
 * Description : Clear port enable on given hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          if the request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN                        Unknown error occurred.
 *               USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The HUB class specific request CLEAR PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.2
 *********************************************************************************************************
 */

static int usbh_hub_port_en_clr(struct usbh_hub_dev *p_hub_dev,
				   uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
			   USBH_REQ_CLR_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_PORT_EN,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                        usbh_hub_port_en_set()
 *
 * Description : Set port enable on given hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : USBH_ERR_NONE,       if the request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The HUB class specific request SET PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.13.
 *********************************************************************************************************
 */

static int usbh_hub_port_en_set(struct usbh_hub_dev *p_hub_dev,
				   uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
			   USBH_REQ_SET_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_PORT_EN,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                      USBH_HUB_PortSuspendSet()
 *
 * Description : Set port suspend on given hub.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 *               port_nbr        Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          if the request was successfully sent.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The HUB class specific request SET PORT FEATURE is defined in USB2.0 specification in
 *                   section 11.24.2.13.
 *********************************************************************************************************
 */

int usbh_hub_port_suspend_set(struct usbh_hub_dev *p_hub_dev,
				   uint16_t port_nbr)
{
	int err;

	usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
			   USBH_REQ_SET_FEATURE,
			   (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
			   USBH_HUB_FEATURE_SEL_PORT_SUSPEND,
			   port_nbr,
			   NULL,
			   0,
			   USBH_HUB_TIMEOUT,
			   &err);
	if (err != 0) {
		usbh_ep_reset(p_hub_dev->DevPtr, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                           usbh_hub_clr()
 *
 * Description : Initializes USBH_HUB_DEV structure.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 * Return(s)   : None
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_hub_clr(struct usbh_hub_dev *p_hub_dev)
{
	uint8_t dev_ix;

	p_hub_dev->DevPtr = NULL;
	p_hub_dev->IF_Ptr = NULL;
	/* Clr dev ptr lst.                                     */
	for (dev_ix = 0; dev_ix < USBH_CFG_MAX_HUB_PORTS; dev_ix++) {
		p_hub_dev->DevPtrList[dev_ix] = NULL;
	}

	p_hub_dev->RefCnt = 0;
	p_hub_dev->State = USBH_CLASS_DEV_STATE_NONE;
	p_hub_dev->NxtPtr = 0;
}

/*
 *********************************************************************************************************
 *                                          usbh_hub_ref_add()
 *
 * Description : Increment access reference count to a hub device.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 * Return(s)   : USBH_ERR_NONE,          if access is successful.
 *               USBH_ERR_INVALID_ARG,   if invalid argument passed to 'p_hub_dev'.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static int usbh_hub_ref_add(struct usbh_hub_dev *p_hub_dev)
{
	int key;

	if (p_hub_dev == NULL) {
		return EINVAL;
	}

	key = irq_lock();
	p_hub_dev->RefCnt++; /* Increment access ref cnt to hub dev.                 */

	irq_unlock(key);
	return 0;
}

/*
 *********************************************************************************************************
 *                                          usbh_hub_ref_rel()
 *
 * Description : Increment the access reference count to a hub device.
 *
 * Argument(s) : p_hub_dev       Pointer to hub device.
 *
 * Return(s)   : USBH_ERR_NONE,          if access is successful.
 *               USBH_ERR_INVALID_ARG,   if invalid argument passed to 'p_hub_dev'.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static int usbh_hub_ref_rel(struct usbh_hub_dev *p_hub_dev)
{
	int key;

	if (p_hub_dev == NULL) {
		return EINVAL;
	}

	key = irq_lock();
	if (p_hub_dev->RefCnt > 0) {
		p_hub_dev->RefCnt--; /* Decrement access ref cnt to hub dev.                 */

		if (p_hub_dev->RefCnt == 0) {
			hub_count++;
		}
	}
	irq_unlock(key);

	return 0;
}

/*
 *********************************************************************************************************
 *                                        USBH_HUB_RH_CtrlReq()
 *
 * Description : Process hub request from core layer.
 *
 * Argument(s) : p_hc            Pointer to host controller.
 *
 *               b_req           Setup packet b_request.
 *
 *               bm_req_type     Setup packet bm_request_type.
 *
 *               w_val           Setup packet w_value.
 *
 *               w_ix            Setup packet w_index.
 *
 *               p_buf           Data buffer pointer.
 *
 *               buf_len         Size of data buffer in bytes.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           USBH_ERR_NONE,          Root hub control request completed successfully.
 *                           USBH_ERR_HC_IO,         Root hub input/output error.
 *                           USBH_ERR_EP_STALL,      Root hub does not support request.
 *
 * Return(s)   : Buffer length in octets.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint32_t usbh_rh_ctrl_req(struct usbh_hc *p_hc,
			  uint8_t b_req,
			  uint8_t bm_req_type,
			  uint16_t w_val,
			  uint16_t w_ix,
			  void *p_buf,
			  uint32_t buf_len,
			  int *p_err)
{
	uint32_t len;
	struct usbh_hc_drv *p_hc_drv;
	const struct usbh_hc_rh_api *p_hc_rh_api;
	bool valid;

	p_hc_drv = &p_hc->HC_Drv;
	p_hc_rh_api = p_hc_drv->RH_API_Ptr;
	*p_err = 0;
	len = 0;
	valid = 1;

	switch (b_req) {
	case USBH_REQ_GET_STATUS:
		/* Only port status is supported.                       */
		if ((bm_req_type & USBH_REQ_RECIPIENT_OTHER) == USBH_REQ_RECIPIENT_OTHER) {
			valid = p_hc_rh_api->PortStatusGet(p_hc_drv,
							   w_ix,
							   (struct usbh_hub_port_status *)p_buf);
		} else   {
			len = buf_len;
			memset(p_buf, 0, len); /* Return 0 for other reqs.                             */
		}
		break;

	case USBH_REQ_CLR_FEATURE:
		switch (w_val) {
		case USBH_HUB_FEATURE_SEL_PORT_EN:
			valid = p_hc_rh_api->PortEnClr(p_hc_drv, w_ix);
			break;

		case USBH_HUB_FEATURE_SEL_PORT_PWR:
			valid = p_hc_rh_api->PortPwrClr(p_hc_drv, w_ix);
			break;

		case USBH_HUB_FEATURE_SEL_C_PORT_CONN:
			valid = p_hc_rh_api->PortConnChngClr(p_hc_drv, w_ix);
			break;

		case USBH_HUB_FEATURE_SEL_C_PORT_RESET:
			valid = p_hc_rh_api->PortResetChngClr(p_hc_drv, w_ix);
			break;

		case USBH_HUB_FEATURE_SEL_C_PORT_EN:
			valid = p_hc_rh_api->PortEnChngClr(p_hc_drv, w_ix);
			break;

		case USBH_HUB_FEATURE_SEL_PORT_INDICATOR:
		case USBH_HUB_FEATURE_SEL_PORT_SUSPEND:
		case USBH_HUB_FEATURE_SEL_C_PORT_SUSPEND:
			valid = p_hc_rh_api->PortSuspendClr(p_hc_drv, w_ix);
			break;

		case USBH_HUB_FEATURE_SEL_C_PORT_OVER_CUR:
			*p_err = EBUSY;
			break;

		default:
			break;
		}
		break;

	case USBH_REQ_SET_FEATURE:
		switch (w_val) {
		case USBH_HUB_FEATURE_SEL_PORT_EN:
			valid = p_hc_rh_api->PortEnSet(p_hc_drv, w_ix);
			break;

		case USBH_HUB_FEATURE_SEL_PORT_RESET:
			valid = p_hc_rh_api->PortResetSet(p_hc_drv, w_ix);
			break;

		case USBH_HUB_FEATURE_SEL_PORT_PWR:
			valid = p_hc_rh_api->PortPwrSet(p_hc_drv, w_ix);
			break;
		/* Not supported reqs.                                  */
		case USBH_HUB_FEATURE_SEL_PORT_SUSPEND:
		case USBH_HUB_FEATURE_SEL_PORT_TEST:
		case USBH_HUB_FEATURE_SEL_PORT_INDICATOR:
		case USBH_HUB_FEATURE_SEL_C_PORT_CONN:
		case USBH_HUB_FEATURE_SEL_C_PORT_RESET:
		case USBH_HUB_FEATURE_SEL_C_PORT_EN:
		case USBH_HUB_FEATURE_SEL_C_PORT_SUSPEND:
		case USBH_HUB_FEATURE_SEL_C_PORT_OVER_CUR:
			*p_err = EBUSY;
			break;

		default:
			break;
		}
		break;

	case USBH_REQ_SET_ADDR:
		break;

	case USBH_REQ_GET_DESC:
		switch (w_val >> 8) { /* Desc type.                                           */
		case USBH_DESC_TYPE_DEV:
			if (buf_len > sizeof(usbh_hub_rh_dev_desc)) {
				len = sizeof(usbh_hub_rh_dev_desc);
			} else   {
				len = buf_len;
			}

			memcpy(p_buf,
			       (void *)usbh_hub_rh_dev_desc,
			       len);
			break;

		case USBH_DESC_TYPE_CFG: /* Return cfg desc.                                     */
			if (buf_len > sizeof(usbh_hub_rh_fs_cfg_desc)) {
				len = sizeof(usbh_hub_rh_fs_cfg_desc);
			} else   {
				len = buf_len;
			}
			memcpy(p_buf,
			       (void *)usbh_hub_rh_fs_cfg_desc,
			       len);
			break;

		case USBH_HUB_DESC_TYPE_HUB: /* Return hub desc.                                     */
			len = buf_len;
			valid = p_hc_rh_api->HubDescGet(p_hc_drv,
							(struct usbh_hub_desc *)p_buf,
							len);
			break;

		case USBH_DESC_TYPE_STR:

			if ((w_val & 0x00FF) == 0) {
				if (buf_len > sizeof(usbh_hub_rh_lang_id)) {
					len = sizeof(usbh_hub_rh_lang_id);
				} else   {
					len = buf_len;
				}
				memcpy(p_buf,
				       (void *)usbh_hub_rh_lang_id,
				       len);
			} else   {
				*p_err = EBUSY;
				break;
			}
			break;

		default:
			break;
		}
		break;

	case USBH_REQ_SET_CFG:
		break;

	case USBH_REQ_GET_CFG:
	case USBH_REQ_GET_IF:
	case USBH_REQ_SET_IF:
	case USBH_REQ_SET_DESC:
	case USBH_REQ_SYNCH_FRAME:
		*p_err = EBUSY;
		break;

	default:
		break;
	}

	if ((valid != 1) &&
	    (*p_err == 0)) {
		*p_err = EIO;
	}

	return len;
}

/*
 *********************************************************************************************************
 *                                         USBH_HUB_RH_Event()
 *
 * Description : Queue a root hub event.
 *
 * Argument(s) : p_dev       Pointer to hub device.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

void usbh_rh_event(struct usbh_dev *p_dev)
{
	struct usbh_hub_dev *p_hub_dev;
	struct usbh_hc_drv *p_hc_drv;
	const struct usbh_hc_rh_api *p_rh_drv_api;
	int key;

	p_hub_dev = p_dev->HC_Ptr->RH_ClassDevPtr;
	p_hc_drv = &p_dev->HC_Ptr->HC_Drv;
	p_rh_drv_api = p_hc_drv->RH_API_Ptr;
	if (p_hub_dev == NULL) {
		p_rh_drv_api->IntDis(p_hc_drv);
		return;
	}

	p_rh_drv_api->IntDis(p_hc_drv);
	LOG_DBG("RefAdd");
	usbh_hub_ref_add(p_hub_dev);

	key = irq_lock();
	if (usbh_hub_head_ptr == NULL) {
		usbh_hub_head_ptr = p_hub_dev;
		usbh_hub_tail_ptr = p_hub_dev;
	} else   {
		usbh_hub_tail_ptr->NxtPtr = p_hub_dev;
		usbh_hub_tail_ptr = p_hub_dev;
	}
	irq_unlock(key);
	k_sem_give(&usbh_hub_event_sem);
}

/*
 **************************************************************************************************************
 *                                       USBH_HUB_ClassNotify()
 *
 * Description : Handle device state change notification for hub class devices.
 *
 * Argument(s) : p_class_dev     Pointer to class device.
 *
 *               state           State of device.
 *
 *               p_ctx           Pointer to context.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 **************************************************************************************************************
 */

void usbh_hub_class_notify(void *p_class_dev,
			   uint8_t state,
			   void *p_ctx)
{
	struct usbh_hub_dev *p_hub_dev;
	struct usbh_dev *p_dev;

	p_hub_dev = (struct usbh_hub_dev *)p_class_dev;
	p_dev = p_hub_dev->DevPtr;

	if (p_dev->IsRootHub == true) { /* If RH, return immediately.                           */
		return;
	}

	switch (state) { /* External hub has been identified.                    */
	case USBH_CLASS_DEV_STATE_CONN:
		LOG_WRN("Ext HUB (Addr# %i) connected\r\n", p_dev->DevAddr);
		break;

	case USBH_CLASS_DEV_STATE_DISCONN:
		LOG_WRN("Ext HUB (Addr# %i) disconnected\r\n", p_dev->DevAddr);
		break;

	default:
		break;
	}
}

/*
 *********************************************************************************************************
 *                                        USBH_HUB_ParseHubDesc()
 *
 * Description : Parse hub descriptor into hub descriptor structure.
 *
 * Argument(s) : p_hub_desc      Variable that will hold the parsed hub descriptor.
 *
 *               p_buf_src       Pointer to buffer that holds hub descriptor.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

void usbh_hub_parse_hub_desc(struct usbh_hub_desc *p_hub_desc,
			     void *p_buf_src)
{
	struct usbh_hub_desc *p_buf_src_desc;
	uint8_t i;

	p_buf_src_desc = (struct usbh_hub_desc *)p_buf_src;

	p_hub_desc->b_desc_length = p_buf_src_desc->b_desc_length;
	p_hub_desc->b_desc_type = p_buf_src_desc->b_desc_type;
	p_hub_desc->b_nbr_ports = p_buf_src_desc->b_nbr_ports;
	p_hub_desc->w_hub_characteristics = sys_get_le16((uint8_t *)&p_buf_src_desc->w_hub_characteristics);
	p_hub_desc->b_pwr_on_to_pwr_good = p_buf_src_desc->b_pwr_on_to_pwr_good;
	p_hub_desc->b_hub_contr_current = p_buf_src_desc->b_hub_contr_current;
	p_hub_desc->device_removable = p_buf_src_desc->device_removable;

	for (i = 0; i < USBH_CFG_MAX_HUB_PORTS; i++) {
		p_hub_desc->port_pwr_ctrl_mask[i] = sys_get_le32((uint8_t *)&p_buf_src_desc->port_pwr_ctrl_mask[i]);
	}
}

/*
 *********************************************************************************************************
 *                                        USBH_HUB_FmtHubDesc()
 *
 * Description : Format hub descriptor from hub descriptor structure.
 *
 * Argument(s) : p_hub_desc       Variable that holds hub descriptor information.
 *
 *               p_buf_dest       Pointer to buffer that will contain hub descriptor.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

void usbh_hub_fmt_hub_desc(struct usbh_hub_desc *p_hub_desc,
			   void *p_buf_dest)
{
	static uint8_t temp;

	temp++;
	struct usbh_hub_desc *p_buf_dest_desc;

	p_buf_dest_desc = (struct usbh_hub_desc *)p_buf_dest;

	p_buf_dest_desc->b_desc_length = p_hub_desc->b_desc_length;
	p_buf_dest_desc->b_desc_type = p_hub_desc->b_desc_type;
	p_buf_dest_desc->b_nbr_ports = p_hub_desc->b_nbr_ports;
	p_buf_dest_desc->w_hub_characteristics = sys_get_le16((uint8_t *)&p_hub_desc->w_hub_characteristics);
	p_buf_dest_desc->b_hub_contr_current = p_hub_desc->b_hub_contr_current;
	p_buf_dest_desc->device_removable = p_hub_desc->device_removable;

	memcpy(&p_buf_dest_desc->b_pwr_on_to_pwr_good, &p_hub_desc->b_pwr_on_to_pwr_good, sizeof(uint8_t));
	memcpy(&p_buf_dest_desc->port_pwr_ctrl_mask[0], &p_hub_desc->port_pwr_ctrl_mask[0], (sizeof(uint32_t) * USBH_CFG_MAX_HUB_PORTS));
}
