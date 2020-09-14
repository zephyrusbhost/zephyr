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
 *                                      USB HOST CORE OPERATIONS
 *
 * Filename : usbh_core.c
 * Version  : V3.42.00
 *********************************************************************************************************
 */

#include "usbh_core.h"
#include "usbh_class.h"
#include "usbh_hub.h"
#include <sys/byteorder.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(usbh_core, 4);

K_THREAD_STACK_DEFINE(USBH_AsyncTask_Stack, 1024);
K_THREAD_STACK_DEFINE(USBH_HUB_EventTask_Stack, 2048);
K_MEM_POOL_DEFINE(AsyncURB_PPool, sizeof(struct usbh_urb),
		  sizeof(struct usbh_urb),
		  (USBH_CFG_MAX_NBR_DEVS * USBH_CFG_MAX_EXTRA_URB_PER_DEV),
		  sizeof(uint32_t));


static volatile struct usbh_urb *USBH_URB_HeadPtr;
static volatile struct usbh_urb *USBH_URB_TailPtr;
static struct k_sem USBH_URB_Sem;

/*
 *********************************************************************************************************
 *                                         HOST MAIN STRUCTURE
 *********************************************************************************************************
 */

static struct usbh_host USBH_Host;

/*
 *********************************************************************************************************
 *                                      LOCAL FUNCTION PROTOTYPES
 *********************************************************************************************************
 */

static int usbh_ep_open(struct usbh_dev *p_dev, struct usbh_if *p_if,
			     uint8_t ep_type, uint8_t ep_dir,
			     struct usbh_ep *p_ep);

static uint32_t usbh_sync_transfer(struct usbh_ep *p_ep, void *p_buf,
				   uint32_t buf_len,
				   struct usbh_isoc_desc *p_isoc_desc,
				   uint8_t token, uint32_t timeout_ms,
				   int *p_err);

static int usbh_async_transfer(struct usbh_ep *p_ep, void *p_buf,
				    uint32_t buf_len,
				    struct usbh_isoc_desc *p_isoc_desc,
				    uint8_t token, void *p_fnct,
				    void *p_fnct_arg);

static uint16_t usbh_sync_ctrl_transfer(struct usbh_ep *p_ep, uint8_t b_req,
					uint8_t bm_req_type, uint16_t w_val,
					uint16_t w_ix, void *p_arg,
					uint16_t w_len, uint32_t timeout_ms,
					int *p_err);

static void usbh_urb_abort(struct usbh_urb *p_urb);

static void usb_urb_notify(struct usbh_urb *p_urb);

static int usbh_urb_submit(struct usbh_urb *p_urb);

static void usbh_urb_clr(struct usbh_urb *p_urb);

static int usbh_dflt_ep_open(struct usbh_dev *p_dev);

static int usbh_dev_desc_rd(struct usbh_dev *p_dev);

static int usbh_cfg_rd(struct usbh_dev *p_dev, uint8_t cfg_ix);

static int usbh_cfg_parse(struct usbh_dev *p_dev, struct usbh_cfg *p_cfg);

static int usbh_dev_addr_set(struct usbh_dev *p_dev);

static uint32_t usbh_str_desc_get(struct usbh_dev *p_dev, uint8_t desc_ix,
				  uint16_t lang_id, void *p_buf,
				  uint32_t buf_len, int *p_err);

// static void USBH_StrDescPrint(USBH_DEV *p_dev, CPU_INT08U *p_str_prefix,
// 							  CPU_INT08U desc_idx);

static struct usbh_desc_hdr *usbh_next_desc_get(void *p_buf,
						uint32_t *p_offset);

static void usbh_fmt_setup_req(struct usbh_setup_req *p_setup_req,
			       void *p_buf_dest);

static void usbh_parse_dev_desc(struct usbh_dev_desc *p_dev_desc,
				void *p_buf_src);

static void usbh_parse_cfg_desc(struct usbh_cfg_desc *p_cfg_desc,
				void *p_buf_src);

static void usbh_parse_if_desc(struct usbh_if_desc *p_if_desc, void *p_buf_src);

static void usbh_parse_ep_desc(struct usbh_ep_desc *p_ep_desc, void *p_buf_src);

static void usbh_async_task(void *p_arg, void *p_arg2, void *p_arg3);

/*
 *********************************************************************************************************
 *                                             USBH_Init()
 *
 * Description : Allocates and initializes resources required by USB Host stack.
 *
 * Argument(s) : async_task_info     Information on asynchronous task.
 *
 *               hub_task_info       Information on hub task.
 *
 * Return(s)   : USBH_ERR_NONE,                  If host stack initialization succeed.
 *               USBH_ERR_ALLOC,                 If memory pool allocation failed.
 *
 *                                               ----- RETURNED BY USBH_ClassDrvReg() : -----
 *               USBH_ERR_NONE,                  If the class driver is registered.
 *               USBH_ERR_INVALID_ARG,           If invalid argument(s) passed to 'p_host'/ 'p_class_drv'.
 *               USBH_ERR_CLASS_DRV_ALLOC,       If maximum class driver limit reached.
 *
 *                                               ----- RETURNED BY USBH_OS_MutexCreate() : -----
 *               USBH_ERR_OS_SIGNAL_CREATE,      if mutex creation failed.
 *
 *                                               ----- RETURNED BY USBH_OS_SemCreate() : -----
 *               USBH_ERR_OS_SIGNAL_CREATE,      If semaphore creation failed.
 *
 *                                               ----- RETURNED BY USBH_OS_TaskCreate() : -----
 *               USBH_ERR_OS_TASK_CREATE,        Task failed to be created.
 *
 * Note(s)     : USBH_Init() must be called:
 *               (1) Only once from a product s application.
 *               (2) After product s OS has been initialized.
 *********************************************************************************************************
 */

int usbh_init(USBH_KERNEL_TASK_INFO *async_task_info,
		   USBH_KERNEL_TASK_INFO *hub_task_info)
{
	int err;
	uint8_t ix;

	USBH_URB_HeadPtr = NULL;
	USBH_URB_TailPtr = NULL;

	USBH_Host.HC_NbrNext = 0;
	USBH_Host.State = USBH_HOST_STATE_NONE;

	for (ix = 0; ix < USBH_CFG_MAX_NBR_CLASS_DRVS;
	     ix++) { /* Clr class drv struct table.                          */
		usbh_class_drv_list[ix].ClassDrvPtr = NULL;
		usbh_class_drv_list[ix].NotifyFnctPtr = NULL;
		usbh_class_drv_list[ix].NotifyArgPtr = NULL;
		usbh_class_drv_list[ix].InUse = 0;
	}

	err = usbh_reg_class_drv(&USBH_HUB_Drv, usbh_hub_class_notify,
				 NULL);
	if (err != 0) {
		return err;
	}

	err = k_sem_init(&USBH_URB_Sem, 0,
			 USBH_OS_SEM_REQUIRED);
	if (err != 0) {
		return err;
	}

	/* Create a task for processing async req.              */
	k_thread_create(&USBH_Host.HAsyncTask, USBH_AsyncTask_Stack,
			K_THREAD_STACK_SIZEOF(USBH_AsyncTask_Stack),
			usbh_async_task, NULL, NULL, NULL, 0, 0, K_NO_WAIT);

	/* Create a task for processing hub events.             */
	k_thread_create(&USBH_Host.HHubTask, USBH_HUB_EventTask_Stack,
			K_THREAD_STACK_SIZEOF(USBH_HUB_EventTask_Stack),
			usbh_hub_event_task, NULL, NULL, NULL, 0, 0, K_NO_WAIT);

	for (ix = 0; ix < USBH_MAX_NBR_DEVS;
	     ix++) { /* Init USB dev list.                                   */
		USBH_Host.DevList[ix].DevAddr =
			ix +
			1; /* USB addr is ix + 1. Addr 0 is rsvd.                  */
		k_mutex_init(&USBH_Host.DevList[ix].DfltEP_Mutex);
	}
	USBH_Host.IsocCount = (USBH_CFG_MAX_ISOC_DESC - 1);
	USBH_Host.DevCount = (USBH_MAX_NBR_DEVS - 1);
	USBH_Host.AsyncURB_Pool = AsyncURB_PPool;


	return 0;
}

/*
 *********************************************************************************************************
 *                                           USBH_Suspend()
 *
 * Description : Suspends USB Host Stack by calling suspend for every class driver loaded
 *               and then calling the host controller suspend.
 *
 * Argument(s) : None.
 *
 * Return(s)   : int_NONE                       If host is suspended.
 *               Host controller driver error,       Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_suspend(void)
{
	uint8_t ix;
	struct usbh_hc *hc;
	int err;

	for (ix = 0; ix < USBH_Host.HC_NbrNext; ix++) {
		hc = &USBH_Host.HC_Tbl[ix];

		usbh_class_suspend(
			hc->HC_Drv.RH_DevPtr); /* Suspend RH, and all downstream dev.                  */
		k_mutex_lock(&hc->HCD_Mutex, K_NO_WAIT);
		hc->HC_Drv.API_Ptr->Suspend(&hc->HC_Drv, &err);
		k_mutex_unlock(&hc->HCD_Mutex); /* Suspend HC.                                          */
	}

	USBH_Host.State = USBH_HOST_STATE_SUSPENDED;

	return err;
}

/*
 *********************************************************************************************************
 *                                            USBH_Resume()
 *
 * Description : Resumes USB Host Stack by calling host controller resume and then
 *               calling resume for every class driver loaded.
 *
 * Argument(s) : None.
 *
 * Return(s)   : USBH_ERR_NONE,                      If host is resumed.
 *               Host controller driver error,       Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_resume(void)
{
	uint8_t ix;
	struct usbh_hc *hc;
	int err;

	for (ix = 0; ix < USBH_Host.HC_NbrNext; ix++) {
		hc = &USBH_Host.HC_Tbl[ix];

		k_mutex_lock(&hc->HCD_Mutex, K_NO_WAIT);
		hc->HC_Drv.API_Ptr->Resume(&hc->HC_Drv, &err);
		k_mutex_unlock(&hc->HCD_Mutex);                  /* Resume HC.                                           */
		usbh_class_resume(
			hc->HC_Drv.RH_DevPtr);  /* Resume RH, and all downstream dev.                   */
	}

	USBH_Host.State = USBH_HOST_STATE_RESUMED;

	return err;
}

/*
 *********************************************************************************************************
 *                                            USBH_HC_Add()
 *
 * Description : Add a host controller.
 *
 * Argument(s) : p_hc_cfg        Pointer to specific USB host controller configuration.
 *
 *               p_drv_api       Pointer to specific USB host controller driver API.
 *
 *               p_hc_rh_api     Pointer to specific USB host controller root hub driver API.
 *
 *               p_hc_bsp_api    Pointer to specific USB host controller board-specific API.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           int_NONE                       Host controller successfully added.
 *                           USBH_ERR_INVALID_ARG                Invalid argument passed to 'p_hc_cfg' / 'p_drv_api' /
 *                                                               'p_hc_rh_api' / 'p_hc_bsp_api'.
 *                           USBH_ERR_HC_ALLOC                   Maximum number of host controller reached.
 *                           USBH_ERR_DEV_ALLOC                  Cannot allocate device structure for root hub.
 *                           Host controller driver error,       Otherwise.
 *
 *                                                               ----- RETURNED BY USBH_OS_MutexCreate() : -----
 *                           USBH_ERR_OS_SIGNAL_CREATE,          if mutex creation failed.
 *
 * Return(s)   : Host Controller index,   if host controller successfully added.
 *               USBH_HC_NBR_NONE,        Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint8_t usbh_hc_add(const struct usbh_hc_cfg *p_hc_cfg,
		    const struct usbh_hc_drv_api *p_drv_api,
		    const struct usbh_hc_rh_api *p_hc_rh_api,
		    const struct usbh_hc_bsp_api *p_hc_bsp_api, int *p_err)
{
	struct usbh_dev *p_rh_dev;
	uint8_t hc_nbr;
	struct usbh_hc *p_hc;
	struct usbh_hc_drv *p_hc_drv;
	int key;

	if ((p_hc_cfg == NULL) || /* ------------------ VALIDATE ARGS ------------------- */
	    (p_drv_api == NULL) ||
	    (p_hc_rh_api == NULL) ||
	    (p_hc_bsp_api == NULL)) {
		*p_err = EINVAL;
		return USBH_HC_NBR_NONE;
	}

	key = irq_lock();
	hc_nbr = USBH_Host.HC_NbrNext;
	if (hc_nbr >=
	    USBH_CFG_MAX_NBR_HC) { /* Chk if HC nbr is valid.                              */
		irq_unlock(key);
		*p_err = EIO;
		return USBH_HC_NBR_NONE;
	}
	USBH_Host.HC_NbrNext++;
	irq_unlock(key);

	p_hc = &USBH_Host.HC_Tbl[hc_nbr];
	p_hc_drv = &p_hc->HC_Drv;

	if (USBH_Host.DevCount < 0) {
		return USBH_HC_NBR_NONE;
	} else {
		p_rh_dev = &USBH_Host.DevList[USBH_Host.DevCount--];
	}

	p_rh_dev->IsRootHub = true;
	p_rh_dev->HC_Ptr = p_hc;

	p_hc->HostPtr = &USBH_Host;

	if (p_hc_rh_api == NULL) {
		p_hc->IsVirRootHub = false;
	} else {
		p_hc->IsVirRootHub = true;
	}

	p_hc_drv->HC_CfgPtr = p_hc_cfg;
	p_hc_drv->DataPtr = NULL;
	p_hc_drv->RH_DevPtr = p_rh_dev;
	p_hc_drv->API_Ptr = p_drv_api;
	p_hc_drv->BSP_API_Ptr = p_hc_bsp_api;
	p_hc_drv->RH_API_Ptr = p_hc_rh_api;
	p_hc_drv->Nbr = hc_nbr;

	k_mutex_init(&p_hc->HCD_Mutex);

	k_mutex_lock(&p_hc->HCD_Mutex, K_NO_WAIT);
	p_hc->HC_Drv.API_Ptr->Init(&p_hc->HC_Drv, p_err);
	k_mutex_unlock(&p_hc->HCD_Mutex); /* Init HCD.                                            */
	if (*p_err != 0) {
		return USBH_HC_NBR_NONE;
	}

	k_mutex_lock(&p_hc->HCD_Mutex, K_NO_WAIT);
	p_rh_dev->DevSpd = p_hc->HC_Drv.API_Ptr->SpdGet(&p_hc->HC_Drv,
							  p_err);
	k_mutex_unlock(&p_hc->HCD_Mutex);

	return hc_nbr;
}

/*
 *********************************************************************************************************
 *                                           USBH_HC_Start()
 *
 * Description : Start given host controller.
 *
 * Argument(s) : hc_nbr      Host controller number.
 *
 * Return(s)   : USBH_ERR_NONE                       If host controller successfully started.
 *               USBH_ERR_INVALID_ARG                If invalid argument passed to 'hc_nbr'.
 *               Host controller driver error,       Otherwise.
 *
 *                                                   ----- RETURNED BY USBH_DevConn() : -----
 *               USBH_ERR_DESC_INVALID               If device contains 0 configurations
 *               USBH_ERR_CFG_ALLOC                  If maximum number of configurations reached.
 *               USBH_ERR_DESC_INVALID,              If invalid descriptor was fetched.
 *               USBH_ERR_CFG_MAX_CFG_LEN,           If cannot allocate descriptor
 *               USBH_ERR_UNKNOWN,                   Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,               Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,          Endpoint is not opened.
 *               USBH_ERR_HC_IO,                     Root hub input/output error.
 *               USBH_ERR_EP_STALL,                  Root hub does not support request.
 *               USBH_ERR_DRIVER_NOT_FOUND           If no class driver was found.
 *               USBH_ERR_OS_SIGNAL_CREATE,          If semaphore or mutex creation failed.
 *               Host controller driver error,       Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_hc_start(uint8_t hc_nbr)
{
	LOG_DBG("Start.");
	int err;
	struct usbh_hc *p_hc;
	struct usbh_dev *p_rh_dev;

	if (hc_nbr >= USBH_Host.HC_NbrNext) { /* Chk if HC nbr is valid.
		                               */
		LOG_DBG("Start HC nbr invalid.");
		return EINVAL;
	}

	p_hc = &USBH_Host.HC_Tbl[hc_nbr];
	p_rh_dev = p_hc->HC_Drv.RH_DevPtr;
	err = usbh_dev_conn(
		p_rh_dev); /* Add RH of given HC.                                  */
	if (err == 0) {
		USBH_Host.State = USBH_HOST_STATE_RESUMED;
	} else {
		LOG_DBG("DevDisconn.");
		usbh_dev_disconn(p_rh_dev);
	}
	k_mutex_lock(&p_hc->HCD_Mutex, K_NO_WAIT);
	p_hc->HC_Drv.API_Ptr->Start(&p_hc->HC_Drv, &err);
	k_mutex_unlock(&p_hc->HCD_Mutex);

	return err;
}

/*
 *********************************************************************************************************
 *                                           USBH_HC_Stop()
 *
 * Description : Stop the given host controller.
 *
 * Argument(s) : hc_nbr      Host controller number.
 *
 * Return(s)   : USBH_ERR_NONE,                      If host controller successfully stoped.
 *               USBH_ERR_INVALID_ARG,               If invalid argument passed to 'hc_nbr'.
 *               Host controller driver error,       Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_hc_stop(uint8_t hc_nbr)
{
	int err;
	struct usbh_hc *p_hc;
	struct usbh_dev *p_rh_dev;

	if (hc_nbr >=
	    USBH_Host.HC_NbrNext) { /* Chk if HC nbr is valid.                              */
		return EINVAL;
	}

	p_hc = &USBH_Host.HC_Tbl[hc_nbr];
	p_rh_dev = p_hc->HC_Drv.RH_DevPtr;

	usbh_dev_disconn(
		p_rh_dev); /* Disconn RH dev.                                      */
	k_mutex_lock(&p_hc->HCD_Mutex, K_NO_WAIT);
	p_hc->HC_Drv.API_Ptr->Stop(&p_hc->HC_Drv, &err);
	k_mutex_unlock(&p_hc->HCD_Mutex);

	return err;
}

/*
 *********************************************************************************************************
 *                                          USBH_HC_PortEn()
 *
 * Description : Enable given port of given host controller's root hub.
 *
 * Argument(s) : hc_nbr    Host controller number.
 *
 *               port_nbr  Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          If port successfully enabled.
 *               USBH_ERR_INVALID_ARG,                   If invalid argument passed to 'hc_nbr'.
 *
 *                                                       ----- RETURNED BY USBH_HUB_PortEn() : -----
 *               USBH_ERR_INVALID_ARG,                   If invalid parameter passed to 'p_hub_dev'.
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

int usbh_hc_port_en(uint8_t hc_nbr, uint8_t port_nbr)
{
	LOG_DBG("PortEn");
	int err;
	struct usbh_hc *p_hc;

	if (hc_nbr >=
	    USBH_Host.HC_NbrNext) { /* Chk if HC nbr is valid.                              */
		return EINVAL;
	}

	p_hc = &USBH_Host.HC_Tbl[hc_nbr];
	err = usbh_hub_port_en(p_hc->RH_ClassDevPtr, port_nbr);

	return err;
}

/*
 *********************************************************************************************************
 *                                          USBH_HC_PortDis()
 *
 * Description : Disable given port of given host controller's root hub.
 *
 * Argument(s) : hc_nbr    Host controller number.
 *
 *               port_nbr  Port number.
 *
 * Return(s)   : USBH_ERR_NONE,                          If port successfully disabled.
 *               USBH_ERR_INVALID_ARG,                   If invalid argument passed to 'hc_nbr'.
 *
 *                                                       ----- RETURNED BY USBH_HUB_PortDis() : -----
 *               USBH_ERR_INVALID_ARG,                   If invalid parameter passed to 'p_hub_dev'.
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

int usbh_hc_port_dis(uint8_t hc_nbr, uint8_t port_nbr)
{
	LOG_DBG("PortDis");
	int err;
	struct usbh_hc *p_hc;

	if (hc_nbr >=
	    USBH_Host.HC_NbrNext) { /* Chk if HC nbr is valid.                              */
		return EINVAL;
	}

	p_hc = &USBH_Host.HC_Tbl[hc_nbr];
	err = usbh_hub_port_dis(p_hc->RH_ClassDevPtr, port_nbr);
	return err;
}

/*
 *********************************************************************************************************
 *                                         USBH_HC_FrameNbrGet()
 *
 * Description : Get current frame number.
 *
 * Argument(s) : hc_nbr  Index of Host Controller.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           USBH_ERR_NONE                           Frame number successfully fetched.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to hc_nbr.
 *                           Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Curent frame number processed by Host Controller, If success.
 *               0,                                                Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint32_t usbh_hc_frame_nbr_get(uint8_t hc_nbr, int *p_err)
{
	uint32_t frame_nbr;
	struct usbh_hc *p_hc;

	if (hc_nbr >=
	    USBH_Host.HC_NbrNext) { /* Chk if HC nbr is valid.                              */
		*p_err = EINVAL;
		return 0;
	}

	p_hc = &USBH_Host.HC_Tbl[hc_nbr];

	k_mutex_lock(&p_hc->HCD_Mutex, K_NO_WAIT);
	frame_nbr = p_hc->HC_Drv.API_Ptr->FrmNbrGet(&p_hc->HC_Drv, p_err);
	k_mutex_unlock(&p_hc->HCD_Mutex);

	return frame_nbr;
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                    DEVICE ENUMERATION FUNCTIONS
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                           USBH_DevConn()
 *
 * Description : Enumerates newly connected USB device. Reads device and configuration descriptor from
 *               device and loads appropriate class driver(s).
 *
 * Argument(s) : p_dev       Pointer to USB device structure.
 *
 * Return(s)   : USBH_ERR_NONE                           If device connection is successful.
 *               USBH_ERR_DESC_INVALID                   If device contains 0 configurations
 *               USBH_ERR_CFG_ALLOC                      If maximum number of configurations reached.
 *
 *                                                       ----- RETURNED BY USBH_CfgRd() : -----
 *               USBH_ERR_DESC_INVALID,                  If invalid configuration descriptor was fetched.
 *               USBH_ERR_CFG_MAX_CFG_LEN,               If configuration descriptor length > USBH_CFG_MAX_CFG_DATA_LEN
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 *                                                       ----- RETURNED BY USBH_CDC_RespRx() : -----
 *               USBH_ERR_DESC_INVALID,                  If an invalid device descriptor was fetched.
 *               USBH_ERR_DEV_NOT_RESPONDING,            If device is not responding.
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 *                                                       ----- RETURNED BY USBH_DevAddrSet() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 *
 *                                                       ----- RETURNED BY USBH_ClassDrvConn() : -----
 *               USBH_ERR_DRIVER_NOT_FOUND               If no class driver was found.
 *               USBH_ERR_UNKNOWN                        Unknown error occurred.
 *               USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 *                                                       ----- RETURNED BY USBH_DfltEP_Open() : -----
 *               USBH_ERR_OS_SIGNAL_CREATE,              If semaphore or mutex creation failed.
 *               Host controller driver error,           Otherwise.
 *
 *                                                       -------- RETURNED BY USBH_CfgRd() : --------
 *               USBH_ERR_NULL_PTR                       If configuration read returns a null pointer.
 *               Host controller driver error,           Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_dev_conn(struct usbh_dev *p_dev)
{
	LOG_DBG("DevConn");
	int err;
	uint8_t nbr_cfgs;
	uint8_t cfg_ix;

	p_dev->SelCfg = 0;

	p_dev->ClassDrvRegPtr = NULL;
	memset(p_dev->DevDesc, 0, USBH_LEN_DESC_DEV);

	LOG_DBG("DftlEP_Open");
	err = usbh_dflt_ep_open(p_dev);
	if (err != 0) {
		return err;
	}

	LOG_DBG("DevDescRd");
	err = usbh_dev_desc_rd(
		p_dev); /* ------------------- RD DEV DESC -------------------- */
	if (err != 0) {
		return err;
	}

	LOG_DBG("DevAddrSet");
	err = usbh_dev_addr_set(
		p_dev); /* -------------- ASSIGN NEW ADDR TO DEV -------------- */
	if (err != 0) {
		return err;
	}

	LOG_DBG("Port %d: Device Address: %d.\r\n", p_dev->PortNbr,
		p_dev->DevAddr);

	// if (p_dev->DevDesc[14] != 0u)
	// { /* iManufacturer = 0 -> no str desc for manufacturer.   */
	// 	USBH_StrDescPrint(p_dev,
	// 					  (CPU_INT08U *)"Manufacturer : ",
	// 					  p_dev->DevDesc[14]);
	// }

	// if (p_dev->DevDesc[15] != 0u)
	// { /* iProduct = 0 -> no str desc for product.             */
	// 	USBH_StrDescPrint(p_dev,
	// 					  (CPU_INT08U *)"Product      : ",
	// 					  p_dev->DevDesc[15]);
	// }

	nbr_cfgs = usbh_dev_cfg_nbr_get(
		p_dev); /* ---------- GET NBR OF CFG PRESENT IN DEV ----------- */
	if (nbr_cfgs == 0) {
		LOG_ERR("descriptor invalid %d", nbr_cfgs);
		return EAGAIN;
	} else if (nbr_cfgs > USBH_CFG_MAX_NBR_CFGS) {
		LOG_ERR("config nbr %d", nbr_cfgs);
		return EAGAIN;
	} else {
		/* Empty Else Statement                                 */
	}

	for (cfg_ix = 0; cfg_ix < nbr_cfgs;
	     cfg_ix++) { /* -------------------- RD ALL CFG -------------------- */
		err = usbh_cfg_rd(p_dev, cfg_ix);
		if (err != 0) {
			LOG_ERR("err cfg read");
			return err;
		}
	}

	LOG_DBG("Call ClassDrvConn");
	err = usbh_class_drv_conn(
		p_dev); /* ------------- PROBE/LOAD CLASS DRV(S) -------------- */

	return err;
}

/*
 *********************************************************************************************************
 *                                          USBH_DevDisconn()
 *
 * Description : Unload class drivers & close default endpoint.
 *
 * Argument(s) : p_dev       Pointer to USB device structure.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

void usbh_dev_disconn(struct usbh_dev *p_dev)
{
	LOG_DBG("DevDisconn");
	usbh_class_drv_disconn(
		p_dev); /* Unload class drv(s).                                 */

	usbh_ep_close(
		&p_dev->DfltEP); /* Close dflt EPs.                                      */
}

/*
 *********************************************************************************************************
 *                                          USBH_DevCfgNbrGet()
 *
 * Description : Get number of configurations supported by specified device.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 * Return(s)   : Number of configurations.
 *
 * Note(s)     : (1) USB2.0 spec, section 9.6.1 states that offset 17 of standard device descriptor
 *                   contains number of configurations.
 *********************************************************************************************************
 */

uint8_t usbh_dev_cfg_nbr_get(struct usbh_dev *p_dev)
{
	return (p_dev->DevDesc
		[17]);         /* See Note (1).                                        */
}

/*
 *********************************************************************************************************
 *                                          USBH_DevDescGet()
 *
 * Description : Get device descriptor of specified USB device.
 *
 * Argument(s) : p_dev            Pointer to USB device.
 *
 *               p_dev_desc       Pointer to device descriptor.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

void usbh_dev_desc_get(struct usbh_dev *p_dev, struct usbh_dev_desc *p_dev_desc)
{
	usbh_parse_dev_desc(p_dev_desc, (void *)p_dev->DevDesc);
}

/*
 *********************************************************************************************************
 *                                             USBH_CfgSet()
 *
 * Description : Select a configration in specified device.
 *
 * Argument(s) : p_dev       Pointer to USB device
 *
 *               cfg_nbr     Configuration number to be selected
 *
 * Return(s)   : USBH_ERR_NONE,                          If given configuration was successfully selected.
 *
 *                                                       ----- RETURNED BY USBH_SET_CFG() : -----
 *               USBH_ERR_UNKNOWN                        Unknown error occurred.
 *               USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) (a) The SET_CONFIGURATION request is described in "Universal Serial Bus Specification
 *                       Revision 2.0", section 9.4.6.
 *
 *                   (b) If the device is in the default state, the behaviour of the device is not
 *                       specified. Implementations stall such request.
 *
 *                   (c) If the device is in the addressed state &
 *
 *                       (1) ... the configuration number is zero, then the device remains in the address state.
 *                       (2) ... the configuration number is non-zero & matches a valid configuration
 *                               number, then that configuration is selected & the device enters the
 *                               configured state.
 *                       (3) ... the configuration number is non-zero & does NOT match a valid configuration
 *                               number, then the device responds with a request error.
 *
 *                   (d) If the device is in the configured state &
 *
 *                       (1) ... the configuration number is zero, then the device enters the address state.
 *                       (2) ... the configuration number is non-zero & matches a valid configuration
 *                               number, then that configuration is selected & the device remains in the
 *                               configured state.
 *                       (3) ... the configuration number is non-zero & does NOT match a valid configuration
 *                               number, then the device responds with a request error.
 *********************************************************************************************************
 */

int usbh_cfg_set(struct usbh_dev *p_dev, uint8_t cfg_nbr)
{
	int err;

	usbh_ctrl_tx(p_dev, USBH_REQ_SET_CFG,
		     (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_RECIPIENT_DEV),
		     cfg_nbr, 0, NULL, 0, USBH_CFG_STD_REQ_TIMEOUT, &err); /* See Note (1).                                        */

	if (err == 0) {
		p_dev->SelCfg = cfg_nbr;
	}
	LOG_DBG("set configuration %d ret %d", cfg_nbr, err);

	return err;
}

/*
 *********************************************************************************************************
 *                                            USBH_CfgGet()
 *
 * Description : Get a pointer to specified configuration data of specified device.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               cfg_ix      Zero based index of configuration.
 *
 * Return(s)   : Pointer to configuration,  If configuration number is valid.
 *               0,                         Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

struct usbh_cfg *usbh_cfg_get(struct usbh_dev *p_dev, uint8_t cfg_ix)
{
	uint8_t nbr_cfgs;

	if (p_dev == NULL) {
		return NULL;
	}

	nbr_cfgs = usbh_dev_cfg_nbr_get(
		p_dev); /* Get nbr of cfg(s) present in dev.                    */
	if ((cfg_ix >= nbr_cfgs) || (nbr_cfgs == 0)) {
		return NULL;
	}

	return (&p_dev->CfgList
		[cfg_ix]);          /* Get cfg struct.                                      */
}

/*
 *********************************************************************************************************
 *                                          USBH_CfgIF_NbrGet()
 *
 * Description : Get number of interfaces in given configuration.
 *
 * Argument(s) : p_cfg       Pointer to configuration
 *
 * Return(s)   : Number of interfaces.
 *
 * Note(s)     : (1) USB2.0 spec, section 9.6.1 states that offset 4 of standard configuration descriptor
 *                   represents number of interfaces in the configuration.
 *********************************************************************************************************
 */

uint8_t usbh_cfg_if_nbr_get(struct usbh_cfg *p_cfg)
{
	if (p_cfg != NULL) {
		return (p_cfg->CfgData
			[4]);         /* See Note (1).                                        */
	} else {
		return 0;
	}
}

/*
 *********************************************************************************************************
 *                                          USBH_CfgDescGet()
 *
 * Description : Get configuration descriptor data.
 *
 * Argument(s) : p_cfg           Pointer to USB configuration
 *
 *               p_cfg_desc      Pointer to a variable that will contain configuration descriptor.
 *
 * Return(s)   : USBH_ERR_NONE            If a valid configuration descriptor is found.
 *               USBH_ERR_INVALID_ARG     If invalid argument passed to 'p_cfg' / 'p_cfg_desc'.
 *               USBH_ERR_DESC_INVALID    If an invalid configuration descriptor was found.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_cfg_desc_get(struct usbh_cfg *p_cfg,
			   struct usbh_cfg_desc *p_cfg_desc)
{
	struct usbh_desc_hdr *p_desc;

	if ((p_cfg == NULL) ||
	    (p_cfg_desc == NULL)) {
		return EINVAL;
	}

	p_desc =
		(struct usbh_desc_hdr *)p_cfg
		->CfgData;         /* Check for valid cfg desc.                            */

	if ((p_desc->bLength == USBH_LEN_DESC_CFG) &&
	    (p_desc->bDescriptorType == USBH_DESC_TYPE_CFG)) {
		usbh_parse_cfg_desc(p_cfg_desc, (void *)p_desc);

		return 0;
	} else {
		return EAGAIN;
	}
}

/*
 *********************************************************************************************************
 *                                       USBH_CfgExtraDescGet()
 *
 * Description : Get extra descriptor immediately following configuration descriptor.
 *
 * Argument(s) : p_cfg       Pointer to USB configuration.
 *
 *               p_err       Pointer to variable that will receive the return error code from this function :
 *
 *                       USBH_ERR_NONE,                   If a valid extra descriptor is present.
 *                       USBH_ERR_INVALID_ARG,            If invalid argument passed to 'p_cfg'.
 *                       USBH_ERR_DESC_EXTRA_NOT_FOUND,   If extra descriptor is not present.
 *
 * Return(s)   : Pointer to extra descriptor,  If a valid extra descriptor is present.
 *               0,                            Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

struct usbh_desc_hdr *usbh_cfg_extra_desc_get(struct usbh_cfg *p_cfg,
					      int *p_err)
{
	struct usbh_desc_hdr *p_desc;
	struct usbh_desc_hdr *p_extra_desc;
	uint32_t cfg_off;

	if (p_cfg == NULL) {
		*p_err = EINVAL;
		return NULL;
	}

	p_desc =
		(struct usbh_desc_hdr *)p_cfg
		->CfgData;         /* Get config desc data.                                */

	if ((p_desc->bLength == USBH_LEN_DESC_CFG) &&
	    (p_desc->bDescriptorType == USBH_DESC_TYPE_CFG) &&
	    (p_cfg->CfgDataLen > (p_desc->bLength + 2))) {
		cfg_off = p_desc->bLength;
		p_extra_desc = usbh_next_desc_get(
			p_desc,
			&cfg_off); /* Get desc that follows config desc.                   */

		/* No extra desc present.                               */
		if (p_extra_desc->bDescriptorType != USBH_DESC_TYPE_IF) {
			*p_err = 0;
			return p_extra_desc;
		}
	}

	*p_err = USBH_ERR_DESC_EXTRA_NOT_FOUND;

	return NULL;
}

/*
 *********************************************************************************************************
 *                                            USBH_IF_Set()
 *
 * Description : Select specified alternate setting of interface.
 *
 * Argument(s) : p_if        Pointer to interface.
 *
 *               alt_nbr     Alternate setting number to select.
 *
 * Return(s)   : USBH_ERR_NONE,                          If specified alternate setting was successfully selected.
 *               USBH_ERR_INVALID_ARG,                   If invalid argument passed to 'p_if' / 'alt_nbr'.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The standard SET_INTERFACE request is defined in the Universal Serial Bus
 *                   specification revision 2.0, section 9.4.10
 *********************************************************************************************************
 */

int usbh_if_set(struct usbh_if *p_if, uint8_t alt_nbr)
{
	uint8_t nbr_alts;
	uint8_t if_nbr;
	struct usbh_dev *p_dev;
	int err;

	if (p_if == NULL) {
		return EINVAL;
	}

	nbr_alts = usbh_if_alt_nbr_get(
		p_if); /* Get nbr of alternate settings in IF.                 */
	if (alt_nbr >= nbr_alts) {
		return EINVAL;
	}

	if_nbr = usbh_if_nbr_get(
		p_if); /* Get IF nbr.                                          */
	p_dev = p_if->DevPtr;

	usbh_ctrl_tx(p_dev, USBH_REQ_SET_IF, (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_RECIPIENT_IF), alt_nbr, if_nbr, NULL, 0, USBH_CFG_STD_REQ_TIMEOUT, &err);
	if (err != 0) {
		return err;
	}

	p_if->AltIxSel =
		alt_nbr; /* Update selected alternate setting.                   */

	return err;
}

/*
 *********************************************************************************************************
 *                                             USBH_IF_Get()
 *
 * Description : Get specified interface from given configuration.
 *
 * Argument(s) : p_cfg       Pointer to configuration.
 *
 *               if_ix       Zero based index of the Interface.
 *
 * Return(s)   : Pointer to interface data,  If interface number is valid.
 *               0,                          Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

struct usbh_if *usbu_if_get(struct usbh_cfg *p_cfg, uint8_t if_ix)
{
	uint8_t nbr_ifs;

	nbr_ifs = usbh_cfg_if_nbr_get(
		p_cfg); /* Get nbr of IFs.                                      */

	if ((if_ix < nbr_ifs) && (if_ix < USBH_CFG_MAX_NBR_IFS)) {
		return (&p_cfg->IF_List
			[if_ix]);          /* Return IF structure at selected ix.                  */
	} else {
		return NULL;
	}
}

/*
 *********************************************************************************************************
 *                                         USBH_IF_AltNbrGet()
 *
 * Description : Get number of alternate settings supported by the given interface.
 *
 * Argument(s) : p_if        Pointer to interface.
 *
 * Return(s)   : Number of alternate settings.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint8_t usbh_if_alt_nbr_get(struct usbh_if *p_if)
{
	struct usbh_desc_hdr *p_desc;
	uint32_t if_off;
	uint8_t nbr_alts;

	nbr_alts = 0;
	if_off = 0;
	p_desc = (struct usbh_desc_hdr *)p_if->IF_DataPtr;

	while (if_off <
	       p_if->IF_DataLen) { /* Cnt nbr of alternate settings.                       */
		p_desc = usbh_next_desc_get((void *)p_desc, &if_off);

		if (p_desc->bDescriptorType == USBH_DESC_TYPE_IF) {
			nbr_alts++;
		}
	}

	return nbr_alts;
}

/*
 *********************************************************************************************************
 *                                           USBH_IF_NbrGet()
 *
 * Description : Get number of given interface.
 *
 * Argument(s) : p_if        Pointer to interface.
 *
 * Return(s)   : Interface number.
 *
 * Note(s)     : (1) USB2.0 spec, section 9.6.5 states that offset 2 (bInterfaceNumber) of standard
 *                   interface descriptor contains the interface number of this interface.
 *********************************************************************************************************
 */

uint8_t usbh_if_nbr_get(struct usbh_if *p_if)
{
	return (p_if->IF_DataPtr
		[2]);         /* See Note (1)                                         */
}

/*
 *********************************************************************************************************
 *                                         USBH_IF_EP_NbrGet()
 *
 * Description : Determine number of endpoints in given alternate setting of interface.
 *
 * Argument(s) : p_if      Pointer to interface.
 *
 *               alt_ix    Alternate setting index.
 *
 * Return(s)   : Number of endpoints.
 *
 * Note(s)     : (1) USB2.0 spec, section 9.6.5 states that offset 4 of standard interface descriptor
 *                   (bNumEndpoints) contains the number of endpoints in this interface descriptor.
 *********************************************************************************************************
 */

uint8_t usbh_if_ep_nbr_get(struct usbh_if *p_if, uint8_t alt_ix)
{
	struct usbh_desc_hdr *p_desc;
	uint32_t if_off;

	if_off = 0;
	p_desc = (struct usbh_desc_hdr *)p_if->IF_DataPtr;

	while (if_off < p_if->IF_DataLen) {
		p_desc = usbh_next_desc_get((void *)p_desc, &if_off);
		/* IF desc.                                             */
		if (p_desc->bDescriptorType == USBH_DESC_TYPE_IF) {
			if (alt_ix ==
			    ((uint8_t *)p_desc)
			    [3]) {              /* Chk alternate setting.                               */
				return (((uint8_t *)p_desc)
					[4]);   /* IF desc offset 4 contains nbr of EPs.                */
			}
		}
	}

	return 0;
}

/*
 *********************************************************************************************************
 *                                          USBH_IF_DescGet()
 *
 * Description : Get descriptor of interface at specified alternate setting index.
 *
 * Argument(s) : p_if        Pointer to USB interface.
 *
 *               alt_ix      Alternate setting index.
 *
 *               p_if_desc   Pointer to a variable to hold the interface descriptor data.
 *
 * Return(s)   : USBH_ERR_NONE          If interface descritor was found.
 *               USBH_ERR_INVALID_ARG   Invalid argument passed to 'alt_ix'.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_if_desc_get(struct usbh_if *p_if, uint8_t alt_ix,
			  struct usbh_if_desc *p_if_desc)
{
	struct usbh_desc_hdr *p_desc;
	uint32_t if_off;

	if_off = 0;
	p_desc = (struct usbh_desc_hdr *)p_if->IF_DataPtr;

	while (if_off < p_if->IF_DataLen) {
		p_desc = usbh_next_desc_get((void *)p_desc, &if_off);

		if ((p_desc->bLength == USBH_LEN_DESC_IF) &&
		    (p_desc->bDescriptorType == USBH_DESC_TYPE_IF) &&
		    (alt_ix == ((uint8_t *)p_desc)[3])) {
			usbh_parse_if_desc(p_if_desc, (void *)p_desc);
			return 0;
		}
	}

	return EINVAL;
}

/*
 *********************************************************************************************************
 *                                       USBH_IF_ExtraDescGet()
 *
 * Description : Get the descriptor immediately following the interface descriptor.
 *
 * Argument(s) : p_if            Pointer to USB interface.
 *
 *               alt_ix          Alternate setting number.
 *
 *               p_data_len      Length of extra interface descriptor.
 *
 * Return(s)   : Pointer to extra descriptor,  If success.
 *               0,                            Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint8_t *usbh_if_extra_desc_get(struct usbh_if *p_if, uint8_t alt_ix,
				uint16_t *p_data_len)
{
	struct usbh_desc_hdr *p_desc;
	uint8_t *p_data;
	uint32_t if_off;

	if ((p_if == NULL) ||
	    (p_if->IF_DataPtr == NULL)) {
		return NULL;
	}

	if_off = 0;
	p_desc = (struct usbh_desc_hdr *)p_if->IF_DataPtr;

	while (if_off < p_if->IF_DataLen) {
		p_desc = usbh_next_desc_get(
			(void *)p_desc,
			&if_off); /* Get next desc from IF.                               */

		if ((p_desc->bLength == USBH_LEN_DESC_IF) &&
		    (p_desc->bDescriptorType == USBH_DESC_TYPE_IF) &&
		    (alt_ix == ((uint8_t *)p_desc)[3])) {
			if (if_off <
			    p_if->IF_DataLen) { /* Get desc that follows selected alternate setting.    */
				p_desc = usbh_next_desc_get((void *)p_desc,
							    &if_off);
				p_data = (uint8_t *)p_desc;
				*p_data_len = 0;

				while ((p_desc->bDescriptorType !=
					USBH_DESC_TYPE_IF) &&
				       (p_desc->bDescriptorType !=
					USBH_DESC_TYPE_EP)) {
					*p_data_len += p_desc->bLength;
					p_desc = usbh_next_desc_get(
						(void *)p_desc, /* Get next desc from IF.                               */
						&if_off);
					if (if_off >= p_if->IF_DataLen) {
						break;
					}
				}

				if (*p_data_len == 0) {
					return NULL;
				} else {
					return (uint8_t *)p_data;
				}
			}
		}
	}

	return NULL;
}

/*
 *********************************************************************************************************
 *                                          USBH_BulkInOpen()
 *
 * Description : Open a bulk IN endpoint.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               p_if        Pointer to USB interface.
 *
 *               p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE                       If the bulk IN endpoint is opened successfully.
 *
 *                                                   ----- RETURNED BY USBH_EP_Open() : -----
 *               USBH_ERR_EP_ALLOC,                  If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND,              If endpoint with given type and direction not found.
 *               USBH_ERR_OS_SIGNAL_CREATE,          if mutex or semaphore creation failed.
 *               Host controller drivers error,      Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_bulk_in_open(struct usbh_dev *p_dev, struct usbh_if *p_if,
			   struct usbh_ep *p_ep)
{
	int err;

	err = usbh_ep_open(p_dev, p_if, USBH_EP_TYPE_BULK, USBH_EP_DIR_IN,
			   p_ep);

	return err;
}

/*
 *********************************************************************************************************
 *                                         USBH_BulkOutOpen()
 *
 * Description : Open a bulk OUT endpoint.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               p_if        Pointer to USB interface.
 *
 *               p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE                       If the bulk OUT endpoint is opened successfully.
 *
 *                                                   ----- RETURNED BY USBH_EP_Open() : -----
 *               USBH_ERR_EP_ALLOC,                  If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND,              If endpoint with given type and direction not found.
 *               USBH_ERR_OS_SIGNAL_CREATE,          if mutex or semaphore creation failed.
 *               Host controller drivers error,      Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_bulk_out_open(struct usbh_dev *p_dev, struct usbh_if *p_if,
			    struct usbh_ep *p_ep)
{
	int err;

	err = usbh_ep_open(p_dev, p_if, USBH_EP_TYPE_BULK, USBH_EP_DIR_OUT,
			   p_ep);

	return err;
}

/*
 *********************************************************************************************************
 *                                          USBH_IntrInOpen()
 *
 * Description : Open an interrupt IN endpoint.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               p_if        Pointer to USB interface.
 *
 *               p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE                       If the interrupt IN endpoint is opened successfully.
 *
 *                                                   ----- RETURNED BY USBH_EP_Open() : -----
 *               USBH_ERR_EP_ALLOC,                  If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND,              If endpoint with given type and direction not found.
 *               USBH_ERR_OS_SIGNAL_CREATE,          if mutex or semaphore creation failed.
 *               Host controller drivers error,      Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_intr_in_open(struct usbh_dev *p_dev, struct usbh_if *p_if,
			   struct usbh_ep *p_ep)
{
	int err;

	err = usbh_ep_open(p_dev, p_if, USBH_EP_TYPE_INTR, USBH_EP_DIR_IN,
			   p_ep);

	return err;
}

/*
 *********************************************************************************************************
 *                                         USBH_IntrOutOpen()
 *
 * Description : Open and interrupt OUT endpoint.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               p_if        Pointer to USB interface.
 *
 *               p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE                       If the interrupt OUT endpoint is opened successfully.
 *
 *                                                   ----- RETURNED BY USBH_EP_Open() : -----
 *               USBH_ERR_EP_ALLOC,                  If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND,              If endpoint with given type and direction not found.
 *               USBH_ERR_OS_SIGNAL_CREATE,          if mutex or semaphore creation failed.
 *               Host controller drivers error,      Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_intr_out_open(struct usbh_dev *p_dev, struct usbh_if *p_if,
			    struct usbh_ep *p_ep)
{
	int err;

	err = usbh_ep_open(p_dev, p_if, USBH_EP_TYPE_INTR, USBH_EP_DIR_OUT,
			   p_ep);

	return err;
}

/*
 *********************************************************************************************************
 *                                          USBH_IsocInOpen()
 *
 * Description : Open an isochronous IN endpoint.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               p_if        Pointer to USB interface.
 *
 *               p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE                       If the isochronous IN endpoint is opened successfully.
 *
 *                                                   ----- RETURNED BY USBH_EP_Open() : -----
 *               USBH_ERR_EP_ALLOC,                  If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND,              If endpoint with given type and direction not found.
 *               USBH_ERR_OS_SIGNAL_CREATE,          if mutex or semaphore creation failed.
 *               Host controller drivers error,      Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_isoc_in_open(struct usbh_dev *p_dev, struct usbh_if *p_if,
			   struct usbh_ep *p_ep)
{
	int err;

	err = usbh_ep_open(p_dev, p_if, USBH_EP_TYPE_ISOC, USBH_EP_DIR_IN,
			   p_ep);

	return err;
}

/*
 *********************************************************************************************************
 *                                         USBH_IsocOutOpen()
 *
 * Description : Open an isochronous OUT endpoint.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               p_if        Pointer to USB interface.
 *
 *               p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE                       If the isochronous OUT endpoint is opened successfully.
 *
 *                                                   ----- RETURNED BY USBH_EP_Open() : -----
 *               USBH_ERR_EP_ALLOC,                  If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND,              If endpoint with given type and direction not found.
 *               USBH_ERR_OS_SIGNAL_CREATE,          if mutex or semaphore creation failed.
 *               Host controller drivers error,      Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_isoc_out_open(struct usbh_dev *p_dev, struct usbh_if *p_if,
			    struct usbh_ep *p_ep)
{
	int err;

	err = usbh_ep_open(p_dev, p_if, USBH_EP_TYPE_ISOC, USBH_EP_DIR_OUT,
			   p_ep);

	return err;
}

/*
 *********************************************************************************************************
 *                                            USBH_CtrlTx()
 *
 * Description : Issue control request to device and send data to it.
 *
 * Argument(s) : p_dev            Pointer to USB device.
 *
 *               b_req            One-byte value that specifies request.
 *
 *               bm_req_type      One-byte value that specifies direction, type and the recipient of
 *                                request.
 *
 *               w_val            Two-byte value used to pass information to device.
 *
 *               w_ix             Two-byte value used to pass an index or offset (such as interface or
 *                                endpoint number) to device.
 *
 *               p_data           Pointer to data buffer to transmit.
 *
 *               w_len            Two-byte value containing buffer length in octets.
 *
 *               timeout_ms       Timeout, in milliseconds.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *                           USBH_ERR_NONE                           Control transfer successfully transmitted.
 *
 *                                                                   ----- RETURNED BY USBH_SyncCtrlXfer() : -----
 *                           USBH_ERR_NONE                           Control transfer successfully completed.
 *                           USBH_ERR_UNKNOWN                        Unknown error occurred.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           Host controller drivers error code,     Otherwise.
 *
 *                                                                   ----- RETURNED BY USBH_HUB_RHCtrlReq() : -----
 *                           USBH_ERR_HC_IO,                         Root hub input/output error.
 *                           USBH_ERR_EP_STALL,                      Root hub does not support request.
 *
 * Return(s)   : Number of octets transmitted.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint16_t usbh_ctrl_tx(struct usbh_dev *p_dev, uint8_t b_req,
		      uint8_t bm_req_type, uint16_t w_val, uint16_t w_ix,
		      void *p_data, uint16_t w_len, uint32_t timeout_ms,
		      int *p_err)
{
	// LOG_DBG("CtrlTx");
	uint16_t xfer_len;

	k_mutex_lock(&p_dev->DfltEP_Mutex, K_NO_WAIT);

	if ((p_dev->IsRootHub ==
	     true) && /* Check if RH features are supported.                  */
	    (p_dev->HC_Ptr->IsVirRootHub == true)) {
		xfer_len = usbh_rh_ctrl_req(
			p_dev->HC_Ptr, /* Send req to virtual HUB.                             */
			b_req, bm_req_type, w_val, w_ix, p_data, w_len, p_err);
	} else {
		xfer_len = usbh_sync_ctrl_transfer(&p_dev->DfltEP, b_req,
						   bm_req_type, w_val, w_ix,
						   p_data, w_len, timeout_ms,
						   p_err);
	}

	k_mutex_unlock(&p_dev->DfltEP_Mutex);
	return xfer_len;
}

/*
 *********************************************************************************************************
 *                                            USBH_CtrlRx()
 *
 * Description : Issue control request to device and receive data from it.
 *
 * Argument(s) : p_dev            Pointer to USB device.
 *
 *               b_req            One-byte value that specifies request.
 *
 *               bm_req_type      One-byte value that specifies direction, type and the recipient of
 *                                request.
 *
 *               w_val            Two-byte value used to pass information to device.
 *
 *               w_ix             Two-byte value used to pass an index or offset (such as interface or
 *                                endpoint number) to device.
 *
 *               p_data           Pointer to destination buffer to receive data.
 *
 *               w_len            Two-byte value containing buffer length in octets.
 *
 *               timeout_ms       Timeout, in milliseconds.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           USBH_ERR_NONE                           Control transfer successfully received.
 *
 *                                                                   ----- RETURNED BY USBH_SyncCtrlXfer() : -----
 *                           int_NONE                           Control transfer successfully completed.
 *                           USBH_ERR_UNKNOWN                        Unknown error occurred.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           Host controller drivers error code,     Otherwise.
 *
 *                                                                   ----- RETURNED BY USBH_HUB_RHCtrlReq() : -----
 *                           USBH_ERR_HC_IO,                         Root hub input/output error.
 *                           USBH_ERR_EP_STALL,                      Root hub does not support request.
 *
 * Return(s)   : Number of octets received.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint16_t usbh_ctrl_rx(struct usbh_dev *p_dev, uint8_t b_req,
		      uint8_t bm_req_type, uint16_t w_val, uint16_t w_ix,
		      void *p_data, uint16_t w_len, uint32_t timeout_ms,
		      int *p_err)
{
	// LOG_DBG("CtrlRx");
	uint16_t xfer_len;

	k_mutex_lock(&p_dev->DfltEP_Mutex, K_NO_WAIT);

	if ((p_dev->IsRootHub ==
	     true) && /* Check if RH features are supported.                  */
	    (p_dev->HC_Ptr->IsVirRootHub == true)) {
		LOG_DBG("request from roothub");
		xfer_len = usbh_rh_ctrl_req(
			p_dev->HC_Ptr, /* Send req to virtual HUB.                             */
			b_req, bm_req_type, w_val, w_ix, p_data, w_len, p_err);
	} else {
		LOG_DBG("request from non roothub");
		xfer_len = usbh_sync_ctrl_transfer(&p_dev->DfltEP, b_req,
						   bm_req_type, w_val, w_ix,
						   p_data, w_len, timeout_ms,
						   p_err);
	}

	k_mutex_unlock(&p_dev->DfltEP_Mutex);

	return xfer_len;
}

/*
 *********************************************************************************************************
 *                                             USBH_BulkTx()
 *
 * Description : Issue bulk request to transmit data to device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to data buffer to transmit.
 *
 *               buf_len         Buffer length in octets.
 *
 *               timeout_ms      Timeout, in milliseconds.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           USBH_ERR_NONE                           Bulk transfer successfully transmitted.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_TYPE                Endpoint type is not Bulk or direction is not OUT.
 *
 *                                                                   ----- RETURNED BY USBH_SyncXfer() : -----
 *                           USBH_ERR_NONE                           Transfer successfully completed.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           USBH_ERR_NONE,                          URB is successfully submitted to host controller.
 *                           Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Number of octets transmitted.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint32_t usbh_bulk_tx(struct usbh_ep *p_ep, void *p_buf, uint32_t buf_len,
		      uint32_t timeout_ms, int *p_err)
{
	uint8_t ep_type;
	uint8_t ep_dir;
	uint32_t xfer_len;

	if (p_ep == NULL) {
		*p_err = EINVAL;
		return 0;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_BULK) || (ep_dir != USBH_EP_DIR_OUT)) {
		*p_err = EAGAIN;
		return 0;
	}

	xfer_len = usbh_sync_transfer(p_ep, p_buf, buf_len,
				      NULL,
				      USBH_TOKEN_OUT, timeout_ms, p_err);

	return xfer_len;
}

/*
 *********************************************************************************************************
 *                                          USBH_BulkTxAsync()
 *
 * Description : Issue asynchronous bulk request to transmit data to device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               buf             Pointer to data buffer to transmit.
 *
 *               buf_len         Buffer length in octets.
 *
 *               p_fnct          Function that will be invoked upon completion of transmit operation.
 *
 *               p_fnct_arg      Pointer to argument that will be passed as parameter of 'p_fnct'.
 *
 * Return(s)   : USBH_ERR_NONE                           If request is successfully submitted.
 *               USBH_ERR_INVALID_ARG                    If invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_TYPE                If endpoint type is not bulk or direction is not OUT.
 *
 *                                                       ----- RETURNED BY USBH_AsyncXfer() : -----
 *               USBH_ERR_NONE                           If transfer successfully submitted.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) This function returns immediately. The URB completion will be invoked by the
 *                   asynchronous I/O task when transfer is completed.
 *********************************************************************************************************
 */

int usbh_bulk_tx_async(struct usbh_ep *p_ep, void *p_buf, uint32_t buf_len,
			    USBH_XFER_CMPL_FNCT fnct, void *p_fnct_arg)
{
	int err;
	uint8_t ep_type;
	uint8_t ep_dir;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_BULK) || (ep_dir != USBH_EP_DIR_OUT)) {
		return EAGAIN;
	}

	err = usbh_async_transfer(p_ep, p_buf, buf_len,
				  NULL, USBH_TOKEN_OUT,
				  (void *)fnct, p_fnct_arg);

	return err;
}

/*
 *********************************************************************************************************
 *                                             USBH_BulkRx()
 *
 * Description : Issue bulk request to receive data from device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to destination buffer to receive data.
 *
 *               buf_len         Buffer length in octets.
 *
 *               timeout_ms      Timeout, in milliseconds.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           USBH_ERR_NONE                           Bulk transfer successfully received.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_TYPE                Endpoint type is not Bulk or direction is not IN.
 *
 *                                                                   ----- RETURNED BY USBH_SyncXfer() : -----
 *                           USBH_ERR_NONE                           Transfer successfully completed.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           USBH_ERR_NONE,                          URB is successfully submitted to host controller.
 *                           Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Number of octets received.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint32_t usbh_bulk_rx(struct usbh_ep *p_ep, void *p_buf, uint32_t buf_len,
		      uint32_t timeout_ms, int *p_err)
{
	uint8_t ep_type;
	uint8_t ep_dir;
	uint32_t xfer_len;

	if (p_ep == NULL) {
		*p_err = EINVAL;
		return 0;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_BULK) || (ep_dir != USBH_EP_DIR_IN)) {
		*p_err = EAGAIN;
		return 0;
	}

	xfer_len = usbh_sync_transfer(p_ep, p_buf, buf_len,
				      NULL, USBH_TOKEN_IN,
				      timeout_ms, p_err);

	return xfer_len;
}

/*
 *********************************************************************************************************
 *                                          USBH_BulkRxAsync()
 *
 * Description : Issue asynchronous bulk request to receive data from device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to destination buffer to receive data.
 *
 *               buf_len         Buffer length in octets.
 *
 *               fnct            Function that will be invoked upon completion of receive operation.
 *
 *               p_fnct_arg      Pointer to argument that will be passed as parameter of 'p_fnct'.
 *
 * Return(s)   : USBH_ERR_NONE                           If request is successfully submitted.
 *               USBH_ERR_INVALID_ARG                    If invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_TYPE                If endpoint type is not Bulk or direction is not IN.
 *
 *                                                       ----- RETURNED BY USBH_AsyncXfer() : -----
 *               USBH_ERR_NONE                           If transfer successfully submitted.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) This function returns immediately. The URB completion will be invoked by the
 *                   asynchronous I/O task when the transfer is completed.
 *********************************************************************************************************
 */

int usbh_bulk_rx_async(struct usbh_ep *p_ep, void *p_buf, uint32_t buf_len,
			    USBH_XFER_CMPL_FNCT fnct, void *p_fnct_arg)
{
	int err;
	uint8_t ep_type;
	uint8_t ep_dir;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_BULK) || (ep_dir != USBH_EP_DIR_IN)) {
		return EAGAIN;
	}

	err = usbh_async_transfer(p_ep, p_buf, buf_len,
				  NULL, USBH_TOKEN_IN,
				  (void *)fnct, p_fnct_arg);

	return err;
}

/*
 *********************************************************************************************************
 *                                             USBH_IntrTx()
 *
 * Description : Issue interrupt request to transmit data to device
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to data buffer to transmit.
 *
 *               buf_len         Buffer length in octets.
 *
 *               timeout_ms      Timeout, in milliseconds.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           int_NONE                           Interrupt transfer successfully transmitted.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_TYPE                Endpoint type is not interrupt or direction is not OUT.
 *
 *                                                                   ----- RETURNED BY USBH_SyncXfer() : -----
 *                           USBH_ERR_NONE                           Transfer successfully completed.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           USBH_ERR_NONE,                          URB is successfully submitted to host controller.
 *                           Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Number of octets transmitted.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint32_t usbh_intr_tx(struct usbh_ep *p_ep, void *p_buf, uint32_t buf_len,
		      uint32_t timeout_ms, int *p_err)
{
	uint8_t ep_type;
	uint8_t ep_dir;
	uint32_t xfer_len;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_INTR) || (ep_dir != USBH_EP_DIR_OUT)) {
		return EAGAIN;
	}

	xfer_len = usbh_sync_transfer(p_ep, p_buf, buf_len,
				      NULL,
				      USBH_TOKEN_OUT, timeout_ms, p_err);

	return xfer_len;
}

/*
 *********************************************************************************************************
 *                                          USBH_IntrTxAsync()
 *
 * Description : Issue asynchronous interrupt request to transmit data to device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to data buffer to transmit.
 *
 *               buf_len         Buffer length in octets.
 *
 *               fnct            Function that will be invoked upon completion of transmit operation.
 *
 *               p_fnct_arg      Pointer to argument that will be passed as parameter of 'p_fnct'.
 *
 * Return(s)   : USBH_ERR_NONE                           If request is successfully submitted.
 *               USBH_ERR_INVALID_ARG                    If invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_TYPE                If endpoint type is not interrupt or direction is not OUT.
 *
 *                                                       ----- RETURNED BY USBH_AsyncXfer() : -----
 *               USBH_ERR_NONE                           If transfer successfully submitted.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) This function returns immediately. The URB completion will be invoked by the
 *                   asynchronous I/O task when the transfer is completed.
 *********************************************************************************************************
 */

int usbh_intr_tx_async(struct usbh_ep *p_ep, void *p_buf, uint32_t buf_len,
			    USBH_XFER_CMPL_FNCT fnct, void *p_fnct_arg)
{
	int err;
	uint8_t ep_type;
	uint8_t ep_dir;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_INTR) || (ep_dir != USBH_EP_DIR_OUT)) {
		return EAGAIN;
	}

	err = usbh_async_transfer(p_ep, p_buf, buf_len,
				  NULL, USBH_TOKEN_OUT,
				  (void *)fnct, p_fnct_arg);

	return err;
}

/*
 *********************************************************************************************************
 *                                             USBH_IntrRx()
 *
 * Description : Issue interrupt request to receive data from device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to destination buffer to receive data.
 *
 *               buf_len         Buffer length in octets.
 *
 *               timeout_ms      Timeout, in milliseconds.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           USBH_ERR_NONE                           Interrupt transfer successfully received.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_TYPE                Endpoint type is not interrupt or direction is not IN.
 *
 *                                                                   ----- RETURNED BY USBH_SyncXfer() : -----
 *                           USBH_ERR_NONE                           Transfer successfully completed.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           USBH_ERR_NONE,                          URB is successfully submitted to host controller.
 *                           Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Number of octets received.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint32_t usbh_intr_rx(struct usbh_ep *p_ep, void *p_buf, uint32_t buf_len,
		      uint32_t timeout_ms, int *p_err)
{
	uint8_t ep_type;
	uint8_t ep_dir;
	uint32_t xfer_len;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_INTR) || (ep_dir != USBH_EP_DIR_IN)) {
		return EAGAIN;
	}

	xfer_len = usbh_sync_transfer(p_ep, p_buf, buf_len,
				      NULL, USBH_TOKEN_IN,
				      timeout_ms, p_err);

	return xfer_len;
}

/*
 *********************************************************************************************************
 *                                          USBH_IntrRxAsync()
 *
 * Description : Issue asynchronous interrupt request to receive data from device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to data buffer to transmit.
 *
 *               buf_len         Buffer length in octets.
 *
 *               fnct            Function that will be invoked upon completion of receive operation.
 *
 *               p_fnct_arg      Pointer to argument that will be passed as parameter of 'p_fnct'.
 *
 * Return(s)   : USBH_ERR_NONE                           If request is successfully submitted.
 *               USBH_ERR_INVALID_ARG                    If invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_TYPE                If endpoint type is not interrupt or direction is not IN.
 *
 *                                                       ----- RETURNED BY USBH_AsyncXfer() : -----
 *               USBH_ERR_NONE                           If transfer successfully submitted.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) This function returns immediately. The URB completion will be invoked by the
 *                   asynchronous I/O task when the transfer is completed.
 *********************************************************************************************************
 */

int usbh_intr_rx_async(struct usbh_ep *p_ep, void *p_buf, uint32_t buf_len,
			    USBH_XFER_CMPL_FNCT fnct, void *p_fnct_arg)
{
	int err;
	uint8_t ep_type;
	uint8_t ep_dir;

	/* Argument checks for valid settings                   */
	if (p_ep == NULL) {
		return EINVAL;
	}

	if (p_ep->IsOpen == false) {
		return EAGAIN;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_INTR) || (ep_dir != USBH_EP_DIR_IN)) {
		return EAGAIN;
	}

	err = usbh_async_transfer(p_ep, p_buf, buf_len,
				  NULL, USBH_TOKEN_IN,
				  (void *)fnct, p_fnct_arg);

	return err;
}

/*
 *********************************************************************************************************
 *                                             USBH_IsocTx()
 *
 * Description : Issue isochronous request to transmit data to device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to data buffer to transmit.
 *
 *               buf_len         Buffer length in octets.
 *
 *               start_frm       Transfer start frame number.
 *
 *               nbr_frm         Number of frames.
 *
 *               p_frm_len
 *
 *               p_frm_err
 *
 *               timeout_ms      Timeout, in milliseconds.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           USBH_ERR_NONE                           Isochronous transfer successfully transmitted.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_TYPE                Endpoint type is not isochronous or direction is not OUT.
 *
 *                                                                   ----- RETURNED BY USBH_SyncXfer() : -----
 *                           USBH_ERR_NONE                           Transfer successfully completed.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           USBH_ERR_NONE,                          URB is successfully submitted to host controller.
 *                           Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Number of octets transmitted.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint32_t usbh_isoc_tx(struct usbh_ep *p_ep, uint8_t *p_buf, uint32_t buf_len,
		      uint32_t start_frm, uint32_t nbr_frm, uint16_t *p_frm_len,
		      int *p_frm_err, uint32_t timeout_ms, int *p_err)
{
	uint8_t ep_type;
	uint8_t ep_dir;
	uint32_t xfer_len;
	struct usbh_isoc_desc isoc_desc;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_ISOC) || (ep_dir != USBH_EP_DIR_OUT)) {
		return EAGAIN;
	}

	isoc_desc.BufPtr = p_buf;
	isoc_desc.BufLen = buf_len;
	isoc_desc.StartFrm = start_frm;
	isoc_desc.NbrFrm = nbr_frm;
	isoc_desc.FrmLen = p_frm_len;
	isoc_desc.FrmErr = p_frm_err;

	xfer_len = usbh_sync_transfer(p_ep, p_buf, buf_len, &isoc_desc,
				      USBH_TOKEN_OUT, timeout_ms, p_err);

	return xfer_len;
}

/*
 *********************************************************************************************************
 *                                          USBH_IsocTxAsync()
 *
 * Description : Issue asynchronous isochronous request to transmit data to device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to data buffer to transmit.
 *
 *               buf_len         Buffer length in octets.
 *
 *               start_frm
 *
 *               nbr_frm
 *
 *               p_frm_len
 *
 *               p_frm_err
 *
 *               fnct            Function that will be invoked upon completion of transmit operation.
 *
 *               p_fnct_arg      Pointer to argument that will be passed as parameter of 'p_fnct'.
 *
 * Return(s)   : USBH_ERR_NONE                           If request is successfully submitted.
 *               USBH_ERR_INVALID_ARG                    If invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_TYPE                If endpoint type is not isochronous or direction is not OUT.
 *
 *                                                       ----- RETURNED BY USBH_AsyncXfer() : -----
 *               USBH_ERR_NONE                           If transfer successfully submitted.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) This function returns immediately. The URB completion will be invoked by the
 *                   asynchronous I/O task when the transfer is completed.
 *********************************************************************************************************
 */

int usbh_isoc_tx_async(struct usbh_ep *p_ep, uint8_t *p_buf,
			    uint32_t buf_len, uint32_t start_frm,
			    uint32_t nbr_frm, uint16_t *p_frm_len,
			    int *p_frm_err, USBH_ISOC_CMPL_FNCT fnct,
			    void *p_fnct_arg)
{
	int err;
	uint8_t ep_type;
	uint8_t ep_dir;
	struct usbh_isoc_desc *p_isoc_desc;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_ISOC) || (ep_dir != USBH_EP_DIR_OUT)) {
		return EAGAIN;
	}

	if (p_ep->DevPtr->HC_Ptr->HostPtr->IsocCount < 0) {
		return USBH_ERR_DESC_ALLOC;
	} else {
		p_isoc_desc = &p_ep->DevPtr->HC_Ptr->HostPtr
			      ->IsocDesc[p_ep->DevPtr->HC_Ptr->HostPtr
					 ->IsocCount--];
	}

	p_isoc_desc->BufPtr = p_buf;
	p_isoc_desc->BufLen = buf_len;
	p_isoc_desc->StartFrm = start_frm;
	p_isoc_desc->NbrFrm = nbr_frm;
	p_isoc_desc->FrmLen = p_frm_len;
	p_isoc_desc->FrmErr = p_frm_err;

	err = usbh_async_transfer(p_ep, p_buf, buf_len, p_isoc_desc,
				  USBH_TOKEN_IN, (void *)fnct, p_fnct_arg);
	if (err != 0) {
		p_ep->DevPtr->HC_Ptr->HostPtr->IsocCount++;
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                             USBH_IsocRx()
 *
 * Description : Issue isochronous request to receive data from device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to destination buffer to receive data.
 *
 *               buf_len         Buffer length in octets.
 *
 *               start_frm
 *
 *               nbr_frm
 *
 *               p_frm_len
 *
 *               p_frm_err
 *
 *               timeout_ms      Timeout, in milliseconds.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                           USBH_ERR_NONE                           Isochronous transfer successfully received.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_TYPE                Endpoint type is not isochronous or direction is not IN.
 *
 *                                                                   ----- RETURNED BY USBH_SyncXfer() : -----
 *                           USBH_ERR_NONE                           Transfer successfully completed.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           USBH_ERR_NONE,                          URB is successfully submitted to host controller.
 *                           Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Number of octets received.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

uint32_t usbh_isoc_rx(struct usbh_ep *p_ep, uint8_t *p_buf, uint32_t buf_len,
		      uint32_t start_frm, uint32_t nbr_frm, uint16_t *p_frm_len,
		      int *p_frm_err, uint32_t timeout_ms, int *p_err)
{
	uint8_t ep_type;
	uint8_t ep_dir;
	uint32_t xfer_len;
	struct usbh_isoc_desc isoc_desc;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_ISOC) || (ep_dir != USBH_EP_DIR_IN)) {
		return EAGAIN;
	}

	isoc_desc.BufPtr = p_buf;
	isoc_desc.BufLen = buf_len;
	isoc_desc.StartFrm = start_frm;
	isoc_desc.NbrFrm = nbr_frm;
	isoc_desc.FrmLen = p_frm_len;
	isoc_desc.FrmErr = p_frm_err;

	xfer_len = usbh_sync_transfer(p_ep, p_buf, buf_len, &isoc_desc,
				      USBH_TOKEN_IN, timeout_ms, p_err);

	return xfer_len;
}

/*
 *********************************************************************************************************
 *                                          USBH_IsocRxAsync()
 *
 * Description : Issue asynchronous isochronous request to receive data from device.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to destination buffer to receive data.
 *
 *               buf_len         Buffer length in octets.
 *
 *               start_frm
 *
 *               nbr_frm
 *
 *               p_frm_len
 *
 *               p_frm_err
 *
 *               fnct            Function that will be invoked upon completion of receive operation.
 *
 *               p_fnct_arg      Pointer to argument that will be passed as parameter of 'p_fnct'.
 *
 * Return(s)   : USBH_ERR_NONE                           If request is successfully submitted.
 *               USBH_ERR_INVALID_ARG                    If invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_TYPE                If endpoint type is not isochronous or direction is not IN.
 *
 *                                                       ----- RETURNED BY USBH_AsyncXfer() : -----
 *               USBH_ERR_NONE                           If transfer successfully submitted.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) This function returns immediately. The URB completion will be invoked by the
 *                   asynchronous I/O task when the transfer is completed.
 *********************************************************************************************************
 */

int usbh_isoc_rx_async(struct usbh_ep *p_ep, uint8_t *p_buf,
			    uint32_t buf_len, uint32_t start_frm,
			    uint32_t nbr_frm, uint16_t *p_frm_len,
			    int *p_frm_err, USBH_ISOC_CMPL_FNCT fnct,
			    void *p_fnct_arg)
{
	int err;
	uint8_t ep_type;
	uint8_t ep_dir;
	struct usbh_isoc_desc *p_isoc_desc;

	if (p_ep == NULL) {
		return EINVAL;
	}

	ep_type = usbh_ep_type_get(p_ep);
	ep_dir = usbh_ep_dir_get(p_ep);

	if ((ep_type != USBH_EP_TYPE_ISOC) || (ep_dir != USBH_EP_DIR_IN)) {
		return EAGAIN;
	}

	if (p_ep->DevPtr->HC_Ptr->HostPtr->IsocCount < 0) {
		return USBH_ERR_DESC_ALLOC;
	} else {
		p_isoc_desc = &p_ep->DevPtr->HC_Ptr->HostPtr
			      ->IsocDesc[p_ep->DevPtr->HC_Ptr->HostPtr
					 ->IsocCount--];
	}

	p_isoc_desc->BufPtr = p_buf;
	p_isoc_desc->BufLen = buf_len;
	p_isoc_desc->StartFrm = start_frm;
	p_isoc_desc->NbrFrm = nbr_frm;
	p_isoc_desc->FrmLen = p_frm_len;
	p_isoc_desc->FrmErr = p_frm_err;

	err = usbh_async_transfer(p_ep, p_buf, buf_len, p_isoc_desc,
				  USBH_TOKEN_IN, (void *)fnct, p_fnct_arg);
	if (err != 0) {
		p_ep->DevPtr->HC_Ptr->HostPtr->IsocCount++;
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                         USBH_EP_LogNbrGet()
 *
 * Description : Get logical number of given endpoint.
 *
 * Argument(s) : p_ep        Pointer to endpoint
 *
 * Return(s)   : Logical number of given endpoint,
 *
 * Note(s)     : (1) USB2.0 spec, section 9.6.6 states that bits 3...0 in offset 2 of standard endpoint
 *                   descriptor (bEndpointAddress) contains the endpoint logical number.
 *********************************************************************************************************
 */

uint8_t usbh_ep_log_nbr_get(struct usbh_ep *p_ep)
{
	return ((uint8_t)p_ep->Desc.bEndpointAddress &
		0x7F); /* See Note (1).                                        */
}

/*
 *********************************************************************************************************
 *                                          USBH_EP_DirGet()
 *
 * Description : Get the direction of given endpoint.
 *
 * Argument(s) : p_ep        Pointer to endpoint structure.
 *
 * Return(s)   : USBH_EP_DIR_NONE,   If endpoint is of type control.
 *               USBH_EP_DIR_OUT,    If endpoint direction is OUT.
 *               USBH_EP_DIR_IN,     If endpoint direction is IN.
 *
 * Note(s)     : (1) USB2.0 spec, section 9.6.6 states that bit 7 in offset 2 of standard endpoint
 *                   descriptor (bEndpointAddress) gives the direction of that endpoint.
 *
 *                   (a) 0 = OUT endpoint
 *                   (b) 1 = IN  endpoint
 *********************************************************************************************************
 */

uint8_t usbh_ep_dir_get(struct usbh_ep *p_ep)
{
	uint8_t ep_type;

	ep_type = usbh_ep_type_get(p_ep);
	if (ep_type == USBH_EP_TYPE_CTRL) {
		return USBH_EP_DIR_NONE;
	}

	if (((uint8_t)p_ep->Desc.bEndpointAddress & 0x80) !=
	    0) { /* See Note (1).                                        */
		return USBH_EP_DIR_IN;
	} else {
		return USBH_EP_DIR_OUT;
	}
}

/*
 *********************************************************************************************************
 *                                            USBH_EP_MaxPktSizeGet()
 *
 * Description : Get the maximum packet size of the given endpoint
 *
 * Argument(s) : p_ep        Pointer to endpoint structure
 *
 * Return(s)   : Maximum Packet Size of the given endpoint
 *
 * Note(s)     : (1) USB2.0 spec, section 9.6.6 states that offset 4 of standard endpoint descriptor
 *                   (wMaxPacketSize) gives the maximum packet size supported by this endpoint.
 *********************************************************************************************************
 */

uint16_t usbh_ep_max_pkt_size_get(struct usbh_ep *p_ep)
{
	return ((uint16_t)p_ep->Desc.wMaxPacketSize &
		0x07FF); /* See Note (1).                                        */
}

/*
 *********************************************************************************************************
 *                                            USBH_EP_TypeGet()
 *
 * Description : Get type of the given endpoint
 *
 * Argument(s) : p_ep          Pointer to endpoint
 *
 * Return(s)   : Type of the given endpoint
 *
 * Note(s)     : (1) USB2.0 spec, section 9.6.6 states that bits 1:0 in offset 3 of standard endpoint
 *                   descriptor (bmAttributes) gives type of the endpoint.
 *
 *                   (a) 00 = Control
 *                   (b) 01 = Isochronous
 *                   (c) 10 = Bulk
 *                   (d) 11 = Interrupt
 *********************************************************************************************************
 */

uint8_t usbh_ep_type_get(struct usbh_ep *p_ep)
{
	return ((uint8_t)p_ep->Desc.bmAttributes &
		0x03); /* See Note (1).                                        */
}

/*
 *********************************************************************************************************
 *                                            USBH_EP_Get()
 *
 * Description : Get endpoint specified by given index / alternate setting / interface.
 *
 * Argument(s) : p_if    Pointer to interface.
 *
 *               alt_ix  Alternate setting index.
 *
 *               ep_ix   Endpoint index.
 *
 *               p_ep    Pointer to endpoint structure.
 *
 * Return(s)   : USBH_ERR_NONE,           If endpoint was found.
 *               USBH_ERR_INVALID_ARG,    If invalid argument passed to 'p_if' / 'p_ep'.
 *               USBH_ERR_EP_NOT_FOUND,   If endpoint was not found.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_ep_get(struct usbh_if *p_if, uint8_t alt_ix, uint8_t ep_ix,
		     struct usbh_ep *p_ep)
{
	struct usbh_desc_hdr *p_desc;
	uint32_t if_off;
	uint8_t ix;

	if ((p_if == NULL) || (p_ep == NULL)) {
		return EINVAL;
	}

	ix = 0;
	if_off = 0;
	p_desc = (struct usbh_desc_hdr *)p_if->IF_DataPtr;

	while (if_off < p_if->IF_DataLen) {
		p_desc = usbh_next_desc_get((void *)p_desc, &if_off);

		if (p_desc->bDescriptorType ==
		    USBH_DESC_TYPE_IF) { /* Chk if IF desc.                                      */
			if (alt_ix ==
			    ((uint8_t *)p_desc)
			    [3]) {         /* Compare alternate setting ix.                        */
				break;
			}
		}
	}

	while (if_off < p_if->IF_DataLen) {
		p_desc = usbh_next_desc_get((void *)p_desc, &if_off);

		if (p_desc->bDescriptorType ==
		    USBH_DESC_TYPE_EP) {        /* Chk if EP desc.                                      */
			if (ix ==
			    ep_ix) {            /* Compare EP ix.                                       */
				usbh_parse_ep_desc(&p_ep->Desc, (void *)p_desc);
				return 0;
			}
			ix++;
		}
	}

	return USBH_ERR_EP_NOT_FOUND;
}

/*
 *********************************************************************************************************
 *                                         USBH_EP_StallSet()
 *
 * Description : Set the STALL condition on endpoint.
 *
 * Argument(s) : p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE,                          If endpoint STALL state is set.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN                        Unknown error occurred.
 *               USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_ep_stall_set(struct usbh_ep *p_ep)
{
	int err;
	struct usbh_dev *p_dev;

	p_dev = p_ep->DevPtr;

	(void)usbh_ctrl_tx(p_dev, USBH_REQ_SET_FEATURE,
			   USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_STD |
			   USBH_REQ_RECIPIENT_EP,
			   USBH_FEATURE_SEL_EP_HALT,
			   p_ep->Desc.bEndpointAddress, NULL, 0,
			   USBH_CFG_STD_REQ_TIMEOUT, &err);
	if (err != 0) {
		usbh_ep_reset(p_dev, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                         USBH_EP_StallClr()
 *
 * Description : Clear the STALL condition on endpoint.
 *
 * Argument(s) : p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE,                          If endpoint STALL state is cleared.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN                        Unknown error occurred.
 *               USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The standard CLEAR_FEATURE request is defined in the Universal Serial Bus
 *                   specification revision 2.0, section 9.4.1.
 *********************************************************************************************************
 */

int usbh_ep_stall_clr(struct usbh_ep *p_ep)
{
	int err;
	struct usbh_dev *p_dev;

	p_dev = p_ep->DevPtr;
	(void)usbh_ctrl_tx(
		p_dev,
		USBH_REQ_CLR_FEATURE, /* See Note (1)                                         */
		USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_STD |
		USBH_REQ_RECIPIENT_EP,
		USBH_FEATURE_SEL_EP_HALT, p_ep->Desc.bEndpointAddress,
		NULL, 0, USBH_CFG_STD_REQ_TIMEOUT, &err);
	if (err != 0) {
		usbh_ep_reset(p_dev, NULL);
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                            USBH_EP_Reset()
 *
 * Description : This function is used to reset the opened endpoint
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               p_ep        Pointer to endpoint. Default endpoint if null is passed.
 *
 * Return(s)   : USBH_ERR_NONE,                  If endpoint successfully reseted.
 *               Host controller driver error,   otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_ep_reset(struct usbh_dev *p_dev, struct usbh_ep *p_ep)
{
	struct usbh_ep *p_ep_t;
	int err;

	if (p_ep == NULL) {
		p_ep_t = &p_dev->DfltEP;
	} else {
		p_ep_t = p_ep;
	}

	if ((p_dev->IsRootHub ==
	     true) && /* Do nothing if virtual RH.                            */
	    (p_dev->HC_Ptr->IsVirRootHub == true)) {
		return 0;
	}

	k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
	p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Abort(&p_dev->HC_Ptr->HC_Drv,
						p_ep_t, &err);
	k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
	if (err != 0) {
		return err;
	}

	k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
	p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Close(&p_dev->HC_Ptr->HC_Drv,
						p_ep_t, &err);
	k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
	if (err != 0) {
		return err;
	}

	k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
	p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Open(&p_dev->HC_Ptr->HC_Drv, p_ep_t,
					       &err);
	k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
	if (err != 0) {
		return err;
	}

	return 0;
}

/*
 *********************************************************************************************************
 *                                            USBH_EP_Close()
 *
 * Description : Closes given endpoint and makes it unavailable for I/O transfers.
 *
 * Argument(s) : p_ep        Pointer to endpoint.
 *
 * Return(s)   : USBH_ERR_NONE,                  If endpoint successfully closed.
 *               USBH_ERR_INVALID_ARG,           If invalid argument passed to 'p_ep'.
 *               Host controller driver error,   Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_ep_close(struct usbh_ep *p_ep)
{
	LOG_DBG("EP_Close");
	int err;
	struct usbh_dev *p_dev;
	struct usbh_urb *p_async_urb;

	if (p_ep == NULL) {
		return EINVAL;
	}

	p_ep->IsOpen = false;
	p_dev = p_ep->DevPtr;
	err = 0;

	if (!((p_dev->IsRootHub == true) &&
	      (p_dev->HC_Ptr->IsVirRootHub == true))) {
		usbh_urb_abort(
			&p_ep->URB); /* Abort any pending URB.                               */
	}

	if (!((p_dev->IsRootHub ==
	       true) && /* Close EP on HC.                                      */
	      (p_dev->HC_Ptr->IsVirRootHub == true))) {
		LOG_DBG("close address %d",
			((p_ep->DevAddr << 8) | p_ep->Desc.bEndpointAddress));
		k_mutex_lock(&p_ep->DevPtr->HC_Ptr->HCD_Mutex, K_NO_WAIT);
		p_ep->DevPtr->HC_Ptr->HC_Drv.API_Ptr->EP_Close(&p_ep->DevPtr->HC_Ptr->HC_Drv,
							       p_ep, &err);
		k_mutex_unlock(&p_ep->DevPtr->HC_Ptr->HCD_Mutex);
	}

	if (p_ep->XferNbrInProgress > 1) {
		p_async_urb = p_ep->URB.AsyncURB_NxtPtr;
		while (p_async_urb != 0) {
			/* Free extra URB.                                      */
			k_free(p_async_urb);

			p_async_urb = p_async_urb->AsyncURB_NxtPtr;
		}

		p_ep->XferNbrInProgress = 0;
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                           USBH_URB_Done()
 *
 * Description : Handle a USB request block (URB) that has been completed by host controller.
 *
 * Argument(s) : p_urb       Pointer to URB.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

void usbh_urb_done(struct usbh_urb *p_urb)
{
	int key;

	if (p_urb->State ==
	    USBH_URB_STATE_SCHEDULED) {         /* URB must be in scheduled state.                      */
		p_urb->State =
			USBH_URB_STATE_QUEUED;  /* Set URB state to done.                               */

		if (p_urb->FnctPtr != NULL) { /* Check if req is async.                               */
			key = irq_lock();
			p_urb->NxtPtr = NULL;

			if (USBH_URB_HeadPtr == NULL) {
				USBH_URB_HeadPtr = p_urb;
				USBH_URB_TailPtr = p_urb;
			} else {
				USBH_URB_TailPtr->NxtPtr = p_urb;
				USBH_URB_TailPtr = p_urb;
			}
			irq_unlock(key);

			k_sem_give(&USBH_URB_Sem);
		} else {
			k_sem_give(&p_urb->Sem); /* Post notification to waiting task.                   */
		}
	}
}

/*
 *********************************************************************************************************
 *                                         USBH_URB_Complete()
 *
 * Description : Handle a URB after transfer has been completed or aborted.
 *
 * Argument(s) : p_urb       Pointer to URB.
 *
 * Return(s)   : USBH_ERR_NONE
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

int usbh_urb_complete(struct usbh_urb *p_urb)
{
	struct usbh_dev *p_dev;
	int err;
	struct usbh_urb *p_async_urb_to_remove;
	struct usbh_urb *p_prev_async_urb;
	struct usbh_urb urb_temp;
	struct usbh_ep *p_ep;
	int key;

	p_ep = p_urb->EP_Ptr;
	p_dev = p_ep->DevPtr;

	if (p_urb->State == USBH_URB_STATE_QUEUED) {
		k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
		p_dev->HC_Ptr->HC_Drv.API_Ptr->URB_Complete(&p_dev->HC_Ptr->HC_Drv,
							    p_urb, &err);
		k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
	} else if (p_urb->State == USBH_URB_STATE_ABORTED) {
		k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
		p_dev->HC_Ptr->HC_Drv.API_Ptr->URB_Abort(&p_dev->HC_Ptr->HC_Drv,
							 p_urb, &err);
		k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);

		p_urb->Err = USBH_ERR_URB_ABORT;
		p_urb->XferLen = 0;
	} else {
		/* Empty Else Statement                                 */
	}

	memcpy(
		(void *)&urb_temp, /* Copy urb locally before freeing it.                  */
		(void *)p_urb, sizeof(struct usbh_urb));

	/* --------- FREE URB BEFORE NOTIFYING CLASS ---------- */
	if ((p_urb !=
	     &p_ep->URB) && /* Is the URB an extra URB for async function?          */
	    (p_urb->FnctPtr != 0)) {
		p_async_urb_to_remove = &p_ep->URB;
		p_prev_async_urb = &p_ep->URB;

		while (p_async_urb_to_remove->AsyncURB_NxtPtr !=
		       0) { /* Srch extra URB to remove.                            */
			/* Extra URB found                                      */
			if (p_async_urb_to_remove->AsyncURB_NxtPtr == p_urb) {
				/* Remove from Q.                                       */
				p_prev_async_urb->AsyncURB_NxtPtr =
					p_urb->AsyncURB_NxtPtr;
				break;
			}
			p_prev_async_urb = p_async_urb_to_remove;
			p_async_urb_to_remove =
				p_async_urb_to_remove->AsyncURB_NxtPtr;
		}
		/* Free extra URB.                                      */
		k_free(p_urb);
	}

	key = irq_lock();
	if (p_ep->XferNbrInProgress > 0) {
		p_ep->XferNbrInProgress--;
	}
	irq_unlock(key);

	if ((urb_temp.State == USBH_URB_STATE_QUEUED) ||
	    (urb_temp.State == USBH_URB_STATE_ABORTED)) {
		usb_urb_notify(&urb_temp);
	}

	return 0;
}

/*
 *********************************************************************************************************
 *                                            USBH_StrGet()
 *
 * Description : Read specified string descriptor, remove header and extract string data.
 *
 * Argument(s) : p_dev            Pointer to USB device.
 *
 *               desc_ix          Index of string descriptor.
 *
 *               lang_id          Language identifier.
 *
 *               p_buf            Buffer in which the string descriptor will be stored.
 *
 *               buf_len          Buffer length in octets.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                       USBH_ERR_NONE                           String descriptor successfully fetched.
 *                       USBH_ERR_INVALID_ARG                    Invalid argument passed to 'desc_ix'.
 *                       USBH_ERR_DESC_INVALID                   Invalid string / lang id descriptor.
 *
 *                                                               ----- RETURNED BY USBH_StrDescGet() : -----
 *                       USBH_ERR_DESC_INVALID,                  Invalid string descriptor fetched.
 *                       USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *                       USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *                       USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *                       USBH_ERR_HC_IO,                         Root hub input/output error.
 *                       USBH_ERR_EP_STALL,                      Root hub does not support request.
 *                       Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Length of string received.
 *
 * Note(s)     : (1) The string descriptor MUST be at least four bytes :
 *
 *                   (a) Byte  0   = Length
 *                   (b) Byte  1   = Type
 *                   (c) Bytes 2-3 = Default language ID
 *********************************************************************************************************
 */

uint32_t usbh_str_get(struct usbh_dev *p_dev, uint8_t desc_ix, uint16_t lang_id,
		      uint8_t *p_buf, uint32_t buf_len, int *p_err)
{
	uint32_t ix;
	uint32_t str_len;
	uint8_t *p_str;
	struct usbh_desc_hdr *p_hdr;

	if (desc_ix ==
	    0) { /* Invalid desc ix.                                     */
		*p_err = EINVAL;
		return 0;
	}

	lang_id = p_dev->LangID;

	if (lang_id ==
	    0) { /* If lang ID is zero, get dflt used by the dev.        */
		str_len =
			usbh_str_desc_get(p_dev, 0, 0, p_buf, buf_len, p_err);
		if (str_len <
		    4u) { /* See Note #1.                                         */
			*p_err = USBH_ERR_DESC_INVALID;
			return 0;
		}

		lang_id = sys_get_le16((uint8_t *)&p_buf[2]); /* Rd language ID into CPU endianness.                  */

		if (lang_id == 0) {
			*p_err = USBH_ERR_DESC_INVALID;
			return 0;
		} else {
			p_dev->LangID = lang_id;
		}
	}

	p_str = p_buf;
	p_hdr = (struct usbh_desc_hdr *)p_buf;
	str_len = usbh_str_desc_get(
		p_dev, /* Rd str desc with lang ID.                            */
		desc_ix, lang_id, p_hdr, buf_len, p_err);

	if (str_len > USBH_LEN_DESC_HDR) {
		str_len =
			(p_hdr->bLength -
			 2); /* Remove 2-byte header.                                */

		if (str_len > (buf_len - 2)) {
			str_len = (buf_len - 2);
		}

		for (ix = 0; ix < str_len; ix++) {
			p_str[ix] = p_str
				    [2 +
				     ix]; /* Str starts from byte 3 in desc.                      */
		}

		p_str[ix] = 0;
		p_str[++ix] = 0;
		str_len =
			str_len /
			2; /* Len of ANSI str.                                     */

		return str_len;
	} else {
		*p_err = USBH_ERR_DESC_INVALID;
		return 0;
	}
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
 *                                           USBH_EP_Open()
 *
 * Description : Open an endpoint.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               p_if        Pointer to interface containing endpoint.
 *
 *               ep_type     Endpoint type.
 *
 *               ep_dir      Endpoint direction.
 *
 *               p_ep        Pointer to endpoint passed by class.
 *
 * Return(s)   : USBH_ERR_NONE                       If endpoint is successfully opened.
 *               USBH_ERR_EP_ALLOC                   If USBH_CFG_MAX_NBR_EPS reached.
 *               USBH_ERR_EP_NOT_FOUND               If endpoint with given type and direction not found.
 *               Host controller drivers error,      Otherwise.
 *
 *                                                   ----- RETURNED BY USBH_OS_MutexCreate() : -----
 *               USBH_ERR_OS_SIGNAL_CREATE,          if mutex creation failed.
 *
 *                                                   ----- RETURNED BY USBH_OS_SemCreate() : -----
 *               USBH_ERR_OS_SIGNAL_CREATE,          If semaphore creation failed.
 *
 * Note(s)     : (1) A full-speed interrupt endpoint can specify a desired period from 1 ms to 255 ms.
 *                   Low-speed endpoints are limited to specifying only 10 ms to 255 ms. High-speed
 *                   endpoints can specify a desired period 2^(bInterval-1) x 125us, where bInterval is in
 *                   the range [1 .. 16].
 *
 *               (2) Full-/high-speed isochronous endpoints must specify a desired period as
 *                   2^(bInterval-1) x F, where bInterval is in the range [1 .. 16] and F is 125us for
 *                   high-speed and 1ms for full-speed.
 *********************************************************************************************************
 */

static int usbh_ep_open(struct usbh_dev *p_dev, struct usbh_if *p_if,
			     uint8_t ep_type, uint8_t ep_dir,
			     struct usbh_ep *p_ep)
{
	uint8_t ep_desc_dir;
	uint8_t ep_desc_type;
	uint8_t ep_ix;
	uint8_t nbr_eps;
	bool ep_found;
	int err;

	if (p_ep->IsOpen == true) {
		return 0;
	}

	usbh_urb_clr(&p_ep->URB);

	ep_found = false;
	ep_desc_type = 0;
	nbr_eps = usbh_if_ep_nbr_get(p_if, p_if->AltIxSel);

	if (nbr_eps > USBH_CFG_MAX_NBR_EPS) {
		err = EBUSY;
		return err;
	}

	for (ep_ix = 0; ep_ix < nbr_eps; ep_ix++) {
		usbh_ep_get(p_if, p_if->AltIxSel, ep_ix, p_ep);

		ep_desc_type =
			p_ep->Desc.bmAttributes &
			0x03u; /* EP type from desc.                                   */
		ep_desc_dir =
			p_ep->Desc.bEndpointAddress &
			0x80u; /* EP dir from desc.                                    */

		if (ep_desc_type == ep_type) {
			if ((ep_desc_type == USBH_EP_TYPE_CTRL) ||
			    (ep_desc_dir == ep_dir)) {
				ep_found = true;
				break;
			}
		}
	}

	if (ep_found == false) {
		return USBH_ERR_EP_NOT_FOUND; /* Class specified EP not found.                        */
	}

	p_ep->Interval = 0;
	if (ep_desc_type ==
	    USBH_EP_TYPE_INTR) { /* ------------ DETERMINE POLLING INTERVAL ------------ */

		if (p_ep->Desc.bInterval >
		    0) { /* See Note #1.                                         */

			if ((p_dev->DevSpd == USBH_LOW_SPEED) ||
			    (p_dev->DevSpd == USBH_FULL_SPEED)) {
				if (p_dev->HubHS_Ptr != NULL) {
					p_ep->Interval =
						8 *
						p_ep->Desc
						.bInterval;         /* 1 (1ms)frame = 8 (125us)microframe.                  */
				} else {
					p_ep->Interval = p_ep->Desc.bInterval;
				}
			} else { /* DevSpd == USBH_DEV_SPD_HIGH                          */
				p_ep->Interval =
					1
						<< (p_ep->Desc.bInterval -
						    1); /* For HS, interval is 2 ^ (bInterval - 1).          */
			}
		}
	} else if (ep_desc_type == USBH_EP_TYPE_ISOC) {
		p_ep->Interval =
			1
				<< (p_ep->Desc.bInterval -
				    1); /* Isoc interval is 2 ^ (bInterval - 1). See Note #2.   */
	} else {
		/* Empty Else Statement                                 */
	}

	p_ep->DevAddr = p_dev->DevAddr;
	p_ep->DevSpd = p_dev->DevSpd;
	p_ep->DevPtr = p_dev;

	if (!((p_dev->IsRootHub == true) &&
	      (p_dev->HC_Ptr->IsVirRootHub == true))) {
		k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
		p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Open(&p_dev->HC_Ptr->HC_Drv,
						       p_ep, &err);
		k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
		if (err != 0) {
			return err;
		}
	}

	err = k_sem_init(&p_ep->URB.Sem, 0, USBH_OS_SEM_REQUIRED); /* Sem for I/O wait.                                    */
	if (err != 0) {
		return err;
	}

	k_mutex_init(&p_ep->Mutex);

	p_ep->IsOpen = true;
	p_ep->URB.EP_Ptr = p_ep;

	return err;
}

/*
 *********************************************************************************************************
 *                                           USBH_SyncTransfer()
 *
 * Description : Perform synchronous transfer on endpoint.
 *
 * Argument(s) : p_ep                 Pointer to endpoint.
 *
 *               p_buf                Pointer to data buffer.
 *
 *               buf_len              Number of data bytes in buffer.
 *
 *               p_isoc_desc          Pointer to isochronous descriptor.
 *
 *               token                USB token to issue.
 *
 *               timeout_ms           Timeout, in milliseconds.
 *
 *               p_err                Variable that will receive the return error code from this function.
 *                           USBH_ERR_NONE                           Transfer successfully completed.
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *
 *                                                                   ----- RETURNED BY USBH_URB_Submit() : -----
 *                           USBH_ERR_NONE,                          URB is successfully submitted to host controller.
 *                           USBH_ERR_EP_INVALID_STATE,              Endpoint is in halt state.
 *                           Host controller drivers error code,     Otherwise.
 *
 *                                                                   ----- RETURNED BY USBH_OS_SemWait() : -----
 *                           USBH_ERR_INVALID_ARG                    If invalid argument passed to 'sem'.
 *                           USBH_ERR_OS_TIMEOUT                     If semaphore pend reached specified timeout.
 *                           USBH_ERR_OS_ABORT                       If pend on semaphore was aborted.
 *                           USBH_ERR_OS_FAIL                        Otherwise.
 *
 * Return(s)   : Number of bytes transmitted or received.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static uint32_t usbh_sync_transfer(struct usbh_ep *p_ep, void *p_buf,
				   uint32_t buf_len,
				   struct usbh_isoc_desc *p_isoc_desc,
				   uint8_t token, uint32_t timeout_ms,
				   int *p_err)
{
	// LOG_DBG("SyncXfer");
	uint32_t len;
	struct usbh_urb *p_urb;

	/* Argument checks for valid settings                   */
	if (p_ep == NULL) {
		*p_err = EINVAL;
		return 0;
	}

	if (p_ep->IsOpen == false) {
		*p_err = EAGAIN;
		return 0;
	}

	k_mutex_lock(&p_ep->Mutex, K_NO_WAIT);

	p_urb = &p_ep->URB;
	p_urb->EP_Ptr = p_ep;
	p_urb->IsocDescPtr = p_isoc_desc;
	p_urb->UserBufPtr = p_buf;
	p_urb->UserBufLen = buf_len;
	p_urb->DMA_BufLen = 0;
	p_urb->DMA_BufPtr = NULL;
	p_urb->XferLen = 0;
	p_urb->FnctPtr = 0;
	p_urb->FnctArgPtr = 0;
	p_urb->State = USBH_URB_STATE_NONE;
	p_urb->ArgPtr = NULL;
	p_urb->Token = token;

	*p_err = usbh_urb_submit(p_urb);

	if (*p_err ==
	    0) {                                                        /* Transfer URB to HC.                                  */
		*p_err = k_sem_take(&p_urb->Sem, K_MSEC(timeout_ms));   /* Wait on URB completion notification.                 */
	}

	if (*p_err == 0) {
		usbh_urb_complete(p_urb);
		*p_err = p_urb->Err;
	} else {
		usbh_urb_abort(p_urb);
	}

	len = p_urb->XferLen;
	p_urb->State = USBH_URB_STATE_NONE;
	k_mutex_unlock(&p_ep->Mutex);

	return len;
}

/*
 *********************************************************************************************************
 *                                           USBH_AsyncXfer()
 *
 * Description : Perform asynchronous transfer on endpoint.
 *
 * Argument(s) : p_ep            Pointer to endpoint.
 *
 *               p_buf           Pointer to data buffer.
 *
 *               buf_len         Number of data bytes in buffer.
 *
 *               p_isoc_desc     Pointer to isochronous descriptor.
 *
 *               token           USB token to issue.
 *
 *               p_fnct          Function that will be invoked upon completion of receive operation.
 *
 *               p_fnct_arg      Pointer to argument that will be passed as parameter of 'p_fnct'.
 *
 * Return(s)   : USBH_ERR_NONE                           If transfer successfully submitted.
 *               USBH_ERR_EP_INVALID_STATE               If endpoint is not opened.
 *               USBH_ERR_ALLOC                          If URB cannot be allocated.
 *               USBH_ERR_UNKNOWN                        If unknown error occured.
 *
 *                                                       ----- RETURNED BY USBH_URB_Submit() : -----
 *               USBH_ERR_NONE,                          If URB is successfully submitted to host controller.
 *               USBH_ERR_EP_INVALID_STATE,              If endpoint is in halt state.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) This function returns immediately. The URB completion will be invoked by the
 *                   asynchronous I/O task when the transfer is completed.
 *********************************************************************************************************
 */

static int usbh_async_transfer(struct usbh_ep *p_ep, void *p_buf,
				    uint32_t buf_len,
				    struct usbh_isoc_desc *p_isoc_desc,
				    uint8_t token, void *p_fnct,
				    void *p_fnct_arg)
{
	int err;
	struct usbh_urb *p_urb;
	struct usbh_urb *p_async_urb;
	struct k_mem_pool *p_async_urb_pool;

	if (p_ep->IsOpen == false) {
		return EAGAIN;
	}

	if ((p_ep->URB.State !=
	     USBH_URB_STATE_SCHEDULED) &&       /* Chk if no xfer is pending or in progress on EP.      */
	    (p_ep->XferNbrInProgress == 0)) {
		p_urb = &p_ep->URB;             /* Use URB struct associated to EP.                     */
	} else if (p_ep->XferNbrInProgress >= 1) {
		/* Get a new URB struct from the URB async pool.        */
		p_async_urb_pool =
			&p_ep->DevPtr->HC_Ptr->HostPtr->AsyncURB_Pool;
		p_urb = k_mem_pool_malloc(p_async_urb_pool,
					  sizeof(struct usbh_urb));
		if (p_urb == NULL) {
			return ENOMEM;
		}

		usbh_urb_clr(p_urb);

		p_async_urb =
			&p_ep->URB; /* Get head of extra async URB Q.                       */

		while (p_async_urb->AsyncURB_NxtPtr !=
		       0) { /* Srch tail of extra async URB Q.                      */
			p_async_urb = p_async_urb->AsyncURB_NxtPtr;
		}

		p_async_urb->AsyncURB_NxtPtr =
			p_urb; /* Insert new URB at end of extra async URB Q.          */
	} else {
		return EAGAIN;
	}
	p_ep->XferNbrInProgress++;

	p_urb->EP_Ptr =
		p_ep; /* ------------------- PREPARE URB -------------------- */
	p_urb->IsocDescPtr = p_isoc_desc;
	p_urb->UserBufPtr = p_buf;
	p_urb->UserBufLen = buf_len;
	p_urb->XferLen = 0;
	p_urb->FnctPtr = (void *)p_fnct;
	p_urb->FnctArgPtr = p_fnct_arg;
	p_urb->State = USBH_URB_STATE_NONE;
	p_urb->ArgPtr = NULL;
	p_urb->Token = token;

	err = usbh_urb_submit(
		p_urb); /* See Note (1).                                        */

	return err;
}

/*
 *********************************************************************************************************
 *                                         USBH_SyncCtrlXfer()
 *
 * Description : Perform synchronous control transfer on endpoint.
 *
 * Argument(s) : p_ep              Pointer to endpoint.
 *
 *               b_req             One-byte value that specifies request.
 *
 *               bm_req_type       One-byte value that specifies direction, type and recipient of request.
 *
 *               w_value           Two-byte value used to pass information to device.
 *
 *               w_ix              Two-byte value used to pass an index or offset (such as an interface or
 *                                 endpoint number) to device.
 *
 *               p_arg             Pointer to data buffer.
 *
 *               w_len             Two-byte value containing number of data bytes in data stage.
 *
 *               timeout_ms        Timeout, in milliseconds.
 *
 *               p_err             Variable that will receive the return error code from this function.
 *
 *                           USBH_ERR_NONE                           Control transfer successfully completed.
 *                           USBH_ERR_UNKNOWN                        Unknown error occurred.
 *
 *                                                                   ----- RETURNED BY USBH_SyncXfer() : -----
 *                           USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
 *                           USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
 *                           Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Number of bytes transfered.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static uint16_t usbh_sync_ctrl_transfer(struct usbh_ep *p_ep, uint8_t b_req,
					uint8_t bm_req_type, uint16_t w_val,
					uint16_t w_ix, void *p_arg,
					uint16_t w_len, uint32_t timeout_ms,
					int *p_err)
{
	// LOG_DBG("SyncCtrlXfer");
	struct usbh_setup_req setup;
	uint8_t setup_buf[8];
	bool is_in;
	uint32_t len;
	uint16_t rtn_len;
	uint8_t *p_data_08;

	setup.bmRequestType =
		bm_req_type; /* ------------------- SETUP STAGE -------------------- */
	setup.bRequest = b_req;
	setup.wValue = w_val;
	setup.wIndex = w_ix;
	setup.wLength = w_len;

	usbh_fmt_setup_req(&setup, setup_buf);
	is_in = (bm_req_type & USBH_REQ_DIR_MASK) != 0 ? true : false;

	len = usbh_sync_transfer(p_ep, (void *)&setup_buf[0],
				 USBH_LEN_SETUP_PKT, NULL,
				 USBH_TOKEN_SETUP, timeout_ms, p_err);
	if (*p_err != 0) {
		return 0;
	}

	if (len != USBH_LEN_SETUP_PKT) {
		*p_err = EAGAIN;
		return 0;
	}

	if (w_len >
	    0) { /* -------------------- DATA STAGE -------------------- */
		p_data_08 = (uint8_t *)p_arg;

		rtn_len = usbh_sync_transfer(p_ep, (void *)p_data_08, w_len,
					     NULL,
					     (is_in ? USBH_TOKEN_IN :
					      USBH_TOKEN_OUT),
					     timeout_ms, p_err);
		if (*p_err != 0) {
			return 0;
		}
	} else {
		rtn_len = 0;
	}

	(void)usbh_sync_transfer(
		p_ep, /* ------------------- STATUS STAGE ------------------- */
		NULL, 0, NULL,
		((w_len && is_in) ? USBH_TOKEN_OUT : USBH_TOKEN_IN), timeout_ms,
		p_err);
	if (*p_err != 0) {
		return 0;
	}

	return rtn_len;
}

/*
 *********************************************************************************************************
 *                                          USBH_URB_Abort()
 *
 * Description : Abort pending URB.
 *
 * Argument(s) : p_urb       Pointer to URB.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_urb_abort(struct usbh_urb *p_urb)
{
	bool cmpl;
	int key;

	cmpl = false;

	key = irq_lock();

	if (p_urb->State == USBH_URB_STATE_SCHEDULED) {
		p_urb->State =
			USBH_URB_STATE_ABORTED; /* Abort scheduled URB.                                 */
		cmpl = true;                    /* Mark URB as completion pending.                      */
	} else if (p_urb->State ==
		   USBH_URB_STATE_QUEUED) {     /* Is URB queued in async Q?                            */
		/* URB is in async lst.                                 */

		p_urb->State = USBH_URB_STATE_ABORTED;
	} else {
		/* Empty Else Statement                                 */
	}
	irq_unlock(key);

	if (cmpl == true) {
		usbh_urb_complete(p_urb);
	}
}

/*
 *********************************************************************************************************
 *                                          USBH_URB_Notify()
 *
 * Description : Notify application about state of given URB.
 *
 * Argument(s) : p_urb       Pointer to URB.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usb_urb_notify(struct usbh_urb *p_urb)
{
	uint16_t nbr_frm;
	uint16_t *p_frm_len;
	uint32_t buf_len;
	uint32_t xfer_len;
	uint32_t start_frm;
	void *p_buf;
	void *p_arg;
	struct usbh_ep *p_ep;
	struct usbh_isoc_desc *p_isoc_desc;
	USBH_XFER_CMPL_FNCT p_xfer_fnct;
	USBH_ISOC_CMPL_FNCT p_isoc_fnct;
	int *p_frm_err;
	int err;
	int key;

	p_ep = p_urb->EP_Ptr;
	p_isoc_desc = p_urb->IsocDescPtr;

	key = irq_lock();
	if ((p_urb->State == USBH_URB_STATE_ABORTED) &&
	    (p_urb->FnctPtr == NULL)) {
		p_urb->State = USBH_URB_STATE_NONE;
		k_sem_reset(&p_urb->Sem);
	}

	if (p_urb->FnctPtr != NULL) { /*  Save URB info.                                       */

		p_buf = p_urb->UserBufPtr;
		buf_len = p_urb->UserBufLen;
		xfer_len = p_urb->XferLen;
		p_arg = p_urb->FnctArgPtr;
		err = p_urb->Err;
		p_urb->State = USBH_URB_STATE_NONE;

		if (p_isoc_desc == NULL) {
			p_xfer_fnct = (USBH_XFER_CMPL_FNCT)p_urb->FnctPtr;
			irq_unlock(key);

			p_xfer_fnct(p_ep, p_buf, buf_len, xfer_len, p_arg, err);
		} else {
			p_isoc_fnct = (USBH_ISOC_CMPL_FNCT)p_urb->FnctPtr;
			start_frm = p_isoc_desc->StartFrm;
			nbr_frm = p_isoc_desc->NbrFrm;
			p_frm_len = p_isoc_desc->FrmLen;
			p_frm_err = p_isoc_desc->FrmErr;
			irq_unlock(key);

			p_ep->DevPtr->HC_Ptr->HostPtr->IsocCount++;

			p_isoc_fnct(p_ep, p_buf, buf_len, xfer_len, start_frm,
				    nbr_frm, p_frm_len, p_frm_err, p_arg, err);
		}
	} else {
		irq_unlock(key);
	}
}

/*
 *********************************************************************************************************
 *                                           USBH_URB_Submit()
 *
 * Description : Submit given URB to host controller.
 *
 * Argument(s) : p_urb       Pointer to URB.
 *
 * Return(s)   : USBH_ERR_NONE,                          If URB is successfully submitted to host controller.
 *               USBH_ERR_EP_INVALID_STATE,              If endpoint is in halt state.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static int usbh_urb_submit(struct usbh_urb *p_urb)
{
	int err;
	bool ep_is_halt;
	struct usbh_dev *p_dev;

	p_dev = p_urb->EP_Ptr->DevPtr;
	k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
	ep_is_halt = p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_IsHalt(&p_dev->HC_Ptr->HC_Drv,
							      p_urb->EP_Ptr,
							      &err);
	k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
	if ((ep_is_halt == true) && (err == 0)) {
		return EAGAIN;
	}

	p_urb->State =
		USBH_URB_STATE_SCHEDULED; /* Set URB state to scheduled.                          */
	p_urb->Err = 0;

	k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
	p_dev->HC_Ptr->HC_Drv.API_Ptr->URB_Submit(&p_dev->HC_Ptr->HC_Drv,
						  p_urb, &err);
	k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);

	return err;
}

/*
 *********************************************************************************************************
 *                                           USBH_URB_Clr()
 *
 * Description : Clear URB structure
 *
 * Argument(s) : p_urb       Pointer to URB structure to clear.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_urb_clr(struct usbh_urb *p_urb)
{
	p_urb->Err = 0;
	p_urb->State = USBH_URB_STATE_NONE;
	p_urb->AsyncURB_NxtPtr = NULL;
}

/*
 *********************************************************************************************************
 *                                          USBH_DfltEP_Open()
 *
 * Description : Open default control endpoint of given USB device.
 *
 * Argument(s) : p_dev       Pointer to USB device
 *
 * Return(s)   : int_NONE,                      If control endpoint was successfully opened.
 *               Host controller driver error,       otherwise.
 *
 *                                                   ----- RETURNED BY USBH_OS_SemCreate() : -----
 *               USBH_ERR_OS_SIGNAL_CREATE,          If semaphore creation failed.
 *
 *                                                   ----- RETURNED BY USBH_OS_MutexCreate() : -----
 *               USBH_ERR_OS_SIGNAL_CREATE,          If mutex creation failed.
 *
 * Note(s)     : (1) The maximum packet size is unknown until device descriptor is read. For initial
 *                   transfer, the "Universal Serial Bus Specification Revision 2.0", Section 5.5.3 states
 *                   that a maximum packet size of 8 should be used for low speed devices and
 *                   a maximum packet size of 64 should be used for high speed devices.
 *********************************************************************************************************
 */

static int usbh_dflt_ep_open(struct usbh_dev *p_dev)
{
	uint16_t ep_max_pkt_size;
	struct usbh_ep *p_ep;
	int err;

	p_ep = &p_dev->DfltEP;

	if (p_ep->IsOpen == true) {
		return 0;
	}

	p_ep->DevAddr = 0;
	p_ep->DevSpd = p_dev->DevSpd;
	p_ep->DevPtr = p_dev;

	if (p_dev->DevSpd ==
	    USBH_LOW_SPEED) { /* See Note (1).                                        */
		ep_max_pkt_size = 8;
	} else {
		ep_max_pkt_size = 64u;
	}

	p_ep->Desc.bLength = 7u;
	p_ep->Desc.bDescriptorType = USBH_DESC_TYPE_EP;
	p_ep->Desc.bEndpointAddress = 0;
	p_ep->Desc.bmAttributes = USBH_EP_TYPE_CTRL;
	p_ep->Desc.wMaxPacketSize = ep_max_pkt_size;
	p_ep->Desc.bInterval = 0;

	if (!((p_dev->IsRootHub ==
	       true) && /* Chk if RH fncts are supported before calling HCD.    */
	      (p_dev->HC_Ptr->IsVirRootHub == true))) {
		k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
		p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Open(&p_dev->HC_Ptr->HC_Drv,
						       p_ep, &err);
		k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
		if (err != 0) {
			return err;
		}
	}

	err = k_sem_init(&p_ep->URB.Sem, 0, USBH_OS_SEM_REQUIRED); /* Create OS resources needed for EP.                   */
	if (err != 0) {
		return err;
	}

	k_mutex_init(&p_ep->Mutex);

	p_ep->URB.EP_Ptr = p_ep;
	p_ep->IsOpen = true;

	return err;
}

/*
 *********************************************************************************************************
 *                                          USBH_DevDescRd()
 *
 * Description : Read device descriptor from device.
 *
 * Argument(s) : p_dev       Pointer to USB device
 *
 * Return(s)   : USBH_ERR_NONE,                          If a valid device descriptor was fetched.
 *               USBH_ERR_DESC_INVALID,                  If an invalid device descriptor was fetched.
 *               USBH_ERR_DEV_NOT_RESPONDING,            If device is not responding.
 *
 *                                                       ----- RETURNED BY USBH_CtrlTx() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) (a) The GET_DESCRIPTOR request is described in "Universal Serial Bus Specification
 *                       Revision 2.0", section 9.4.3.
 *
 *                   (b) According the USB Spec, when host doesn't know the control max packet size, it
 *                       should assume 8 byte pkt size and request only 8 bytes from device descriptor.
 *********************************************************************************************************
 */

static int usbh_dev_desc_rd(struct usbh_dev *p_dev)
{
	int err;
	uint8_t retry;
	struct usbh_dev_desc dev_desc;

	retry = 3u;
	while (retry > 0) {
		retry--;
		/* ---------- READ FIRST 8 BYTES OF DEV DESC ---------- */
		usbh_ctrl_tx(p_dev, USBH_REQ_GET_DESC,
			     (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_RECIPIENT_DEV),
			     (USBH_DESC_TYPE_DEV << 8) | 0, 0, p_dev->DevDesc,
			     8, USBH_CFG_STD_REQ_TIMEOUT, &err);
		if (err != 0) {
			usbh_ep_reset(p_dev, NULL);
			k_sleep(K_MSEC(100u));
		} else {
			break;
		}
	}
	if (err != 0) {
		return err;
	}

	if (!((p_dev->IsRootHub == true) &&
	      (p_dev->HC_Ptr->IsVirRootHub == true))) {
		/* Retrieve EP 0 max pkt size.                          */
		p_dev->DfltEP.Desc.wMaxPacketSize = (uint8_t) p_dev->DevDesc[7u];
		if (p_dev->DfltEP.Desc.wMaxPacketSize > 64u) {
			return USBH_ERR_DESC_INVALID;
		}

		k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
		p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Close(&p_dev->HC_Ptr->HC_Drv,
							&p_dev->DfltEP, &err);
		k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);

		k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
		p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Open(&p_dev->HC_Ptr->HC_Drv,
						       &p_dev->DfltEP, &err);
		k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
		if (err != 0) {
			LOG_ERR("%d", err);
			return err;
		}
	}

	retry = 3u;
	while (retry > 0) {
		retry--;
		/* ---------------- RD FULL DEV DESC. ----------------- */
		usbh_ctrl_tx(p_dev, USBH_REQ_GET_DESC,
			     (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_RECIPIENT_DEV),
			     (USBH_DESC_TYPE_DEV << 8) | 0, 0, p_dev->DevDesc,
			     USBH_LEN_DESC_DEV, USBH_CFG_STD_REQ_TIMEOUT,
			     &err);
		if (err != 0) {
			usbh_ep_reset(p_dev, NULL);
			k_sleep(K_MSEC(100u));
		} else {
			break;
		}
	}
	if (err != 0) {
		return err;
	}

	/* ---------------- VALIDATE DEV DESC ----------------- */
	usbh_dev_desc_get(p_dev, &dev_desc);

	if ((dev_desc.bLength < USBH_LEN_DESC_DEV) ||
	    (dev_desc.bDescriptorType != USBH_DESC_TYPE_DEV) ||
	    (dev_desc.bNbrConfigurations == 0)) {
		return USBH_ERR_DESC_INVALID;
	}

	if ((dev_desc.bDeviceClass != USBH_CLASS_CODE_USE_IF_DESC) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_AUDIO) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_CDC_CTRL) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_HID) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_PHYSICAL) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_IMAGE) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_PRINTER) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_MASS_STORAGE) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_HUB) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_CDC_DATA) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_SMART_CARD) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_CONTENT_SECURITY) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_VIDEO) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_PERSONAL_HEALTHCARE) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_DIAGNOSTIC_DEV) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_WIRELESS_CTRLR) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_MISCELLANEOUS) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_APP_SPECIFIC) &&
	    (dev_desc.bDeviceClass != USBH_CLASS_CODE_VENDOR_SPECIFIC)) {
		LOG_ERR("descriptor invalid");
		return USBH_ERR_DESC_INVALID;
	}

	return 0;
}

/*
 *********************************************************************************************************
 *                                             USBH_CfgRd()
 *
 * Description : Read configuration descriptor for a given configuration number.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 *               cfg_ix      Configuration number.
 *
 * Return(s)   : USBH_ERR_NONE,                          If valid configuration descriptor was fetched.
 *               USBH_ERR_DESC_INVALID,                  If invalid configuration descriptor was fetched.
 *               USBH_ERR_CFG_MAX_CFG_LEN,               If configuration descriptor length > USBH_CFG_MAX_CFG_DATA_LEN
 *               USBH_ERR_NULL_PTR                       If an invalid null pointer was obtain for 'p_cfg'.
 *
 *                                                       ----- RETURNED BY USBH_GET_DESC() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The standard GET_DESCRIPTOR request is described in "Universal Serial Bus
 *                   Specification Revision 2.0", section 9.4.3.
 *
 *               (2) According to "Universal Serial Bus Specification Revision 2.0",  the standard
 *                   configuration descriptor is of length 9 bytes.
 *
 *               (3) Offset 2 of standard configuration descriptor contains the total length of that
 *                   configuration which includes all interfaces and endpoint descriptors.
 *********************************************************************************************************
 */

static int usbh_cfg_rd(struct usbh_dev *p_dev, uint8_t cfg_ix)
{
	uint16_t w_tot_len;
	uint16_t b_read;
	int err;
	struct usbh_cfg *p_cfg;
	uint8_t retry;

	p_cfg = usbh_cfg_get(p_dev, cfg_ix);
	if (p_cfg == NULL) {
		LOG_ERR("err cfg get");
		return ENOMEM;
	}

	retry = 3u;
	while (retry > 0) {
		retry--;
		b_read = usbh_ctrl_tx(p_dev, USBH_REQ_GET_DESC,
				      (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_RECIPIENT_DEV),
				      (USBH_DESC_TYPE_CFG << 8) | cfg_ix, 0,
				      p_cfg->CfgData, USBH_LEN_DESC_CFG,
				      USBH_CFG_STD_REQ_TIMEOUT, &err);
		if (err != 0) {
			LOG_ERR("err get descriptor");
			usbh_ep_reset(p_dev, NULL);
			k_sleep(K_MSEC(100u));
		} else {
			break;
		}
	}
	if (err != 0) {
		LOG_ERR("---");
		return err;
	}

	if (b_read < USBH_LEN_DESC_CFG) {
		LOG_ERR("bread < %d", USBH_LEN_DESC_CFG);
		return USBH_ERR_DESC_INVALID;
	}

	if (p_cfg->CfgData[1] != USBH_DESC_TYPE_CFG) {
		LOG_ERR("desc invalid");
		return USBH_ERR_DESC_INVALID;
	}

	w_tot_len = sys_get_le16((uint8_t *)&p_cfg->CfgData[2]); /* See Note (3).                                        */

	if (w_tot_len >
	    USBH_CFG_MAX_CFG_DATA_LEN) { /* Chk total len of config desc.                        */
		LOG_ERR("w_tot_len > %d", USBH_CFG_MAX_CFG_DATA_LEN);
		return ENOMEM;
	}

	retry = 3u;
	while (retry > 0) {
		retry--;
		b_read = usbh_ctrl_tx(p_dev, USBH_REQ_GET_DESC,
				      (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_RECIPIENT_DEV),
				      (USBH_DESC_TYPE_CFG << 8) | cfg_ix, 0,
				      p_cfg->CfgData, w_tot_len,
				      USBH_CFG_STD_REQ_TIMEOUT, &err);
		if (err != 0) {
			LOG_ERR("err get full descriptor");
			usbh_ep_reset(p_dev, NULL);
			k_sleep(K_MSEC(100u));
		} else {
			break;
		}
	}
	if (err != 0) {
		LOG_ERR("...");
		return err;
	}

	if (b_read < w_tot_len) {
		LOG_ERR("bread < w_tot_len");
		return USBH_ERR_DESC_INVALID;
	}

	if (p_cfg->CfgData[1] !=
	    USBH_DESC_TYPE_CFG) { /* Validate config desc.                                */
		LOG_ERR("ivalid");
		return USBH_ERR_DESC_INVALID;
	}

	p_cfg->CfgDataLen = w_tot_len;
	err = usbh_cfg_parse(p_dev, p_cfg);
	LOG_ERR("%d", err);
	return err;
}

/*
 *********************************************************************************************************
 *                                           USBH_CfgParse()
 *
 * Description : Parse given configuration descriptor.
 *
 * Argument(s) : p_dev       Pointer to USB Device.
 *
 *               p_cfg       Pointer to USB configuration.
 *
 * Return(s)   : USBH_ERR_NONE           If configuration descriptor successfully parsed.
 *               USBH_ERR_IF_ALLOC       If number of interfaces > USBH_CFG_MAX_NBR_IFS.
 *               USBH_ERR_DESC_INVALID   If any descriptor in this configuration is invalid.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static int usbh_cfg_parse(struct usbh_dev *p_dev, struct usbh_cfg *p_cfg)
{
	int8_t if_ix;
	uint8_t nbr_ifs;
	uint32_t cfg_off;
	struct usbh_if *p_if;
	struct usbh_desc_hdr *p_desc;
	struct usbh_cfg_desc cfg_desc;
	struct usbh_if_desc if_desc;
	struct usbh_ep_desc ep_desc;

	cfg_off = 0;
	p_desc = (struct usbh_desc_hdr *)p_cfg->CfgData;

	/* ---------------- VALIDATE CFG DESC ----------------- */
	usbh_parse_cfg_desc(&cfg_desc, p_desc);
	if ((cfg_desc.bMaxPower > 250u) || (cfg_desc.bNbrInterfaces == 0)) {
		return USBH_ERR_DESC_INVALID;
	}

	nbr_ifs = usbh_cfg_if_nbr_get(
		p_cfg); /* Nbr of IFs present in config.                        */
	LOG_ERR("%d", nbr_ifs);
	if (nbr_ifs > USBH_CFG_MAX_NBR_IFS) {
		LOG_ERR("asdasd");
		return ENOMEM;
	}

	if_ix = 0;
	p_if = NULL;

	while (cfg_off < p_cfg->CfgDataLen) {
		p_desc = usbh_next_desc_get((void *)p_desc, &cfg_off);

		/* ---------- VALIDATE INTERFACE DESCRIPTOR ----------- */
		if (p_desc->bDescriptorType == USBH_DESC_TYPE_IF) {
			usbh_parse_if_desc(&if_desc, p_desc);
			if ((if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_AUDIO) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_CDC_CTRL) &&
			    (if_desc.bInterfaceClass != USBH_CLASS_CODE_HID) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_PHYSICAL) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_IMAGE) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_PRINTER) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_MASS_STORAGE) &&
			    (if_desc.bInterfaceClass != USBH_CLASS_CODE_HUB) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_CDC_DATA) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_SMART_CARD) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_CONTENT_SECURITY) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_VIDEO) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_PERSONAL_HEALTHCARE) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_DIAGNOSTIC_DEV) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_WIRELESS_CTRLR) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_MISCELLANEOUS) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_APP_SPECIFIC) &&
			    (if_desc.bInterfaceClass !=
			     USBH_CLASS_CODE_VENDOR_SPECIFIC)) {
				return USBH_ERR_DESC_INVALID;
			}

			if ((if_desc.bNbrEndpoints > 30u)) {
				return USBH_ERR_DESC_INVALID;
			}

			if (if_desc.bAlternateSetting == 0) {
				p_if = &p_cfg->IF_List[if_ix];
				p_if->DevPtr = (struct usbh_dev *)p_dev;
				p_if->IF_DataPtr = (uint8_t *)p_desc;
				p_if->IF_DataLen = 0;
				if_ix++;
			}
		}

		if (p_desc->bDescriptorType == USBH_DESC_TYPE_EP) {
			usbh_parse_ep_desc(&ep_desc, p_desc);

			if ((ep_desc.bEndpointAddress == 0x00u) ||
			    (ep_desc.bEndpointAddress == 0x80u) ||
			    (ep_desc.wMaxPacketSize == 0)) {
				return USBH_ERR_DESC_INVALID;
			}
		}

		if (p_if != NULL) {
			p_if->IF_DataLen += p_desc->bLength;
		}
	}

	if (if_ix !=
	    nbr_ifs) { /* IF count must match max nbr of IFs.                  */
		return USBH_ERR_DESC_INVALID;
	}

	return 0;
}

/*
 *********************************************************************************************************
 *                                          USBH_DevAddrSet()
 *
 * Description : Assign address to given USB device.
 *
 * Argument(s) : p_dev       Pointer to USB device.
 *
 * Return(s)   : USBH_ERR_NONE,                          If device address was successfully set.
 *               Host controller driver error,           Otherwise.
 *
 *                                                       ----- RETURNED BY USBH_SET_ADDR() : -----
 *               USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *               USBH_ERR_HC_IO,                         Root hub input/output error.
 *               USBH_ERR_EP_STALL,                      Root hub does not support request.
 *               Host controller drivers error code,     Otherwise.
 *
 * Note(s)     : (1) The SET_ADDRESS request is described in "Universal Serial Bus Specification
 *                   Revision 2.0", section 9.4.6.
 *
 *               (2) USB 2.0 spec states that after issuing SET_ADDRESS request, the host must wait
 *                   at least 2 millisecs. During this time the device will go into the addressed state.
 *********************************************************************************************************
 */

static int usbh_dev_addr_set(struct usbh_dev *p_dev)
{
	LOG_DBG("set device address");
	int err;
	uint8_t retry;

	retry = 3u;
	while (retry > 0) {
		retry--;
		
		usbh_ctrl_tx(p_dev, USBH_REQ_SET_ADDR, (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_RECIPIENT_DEV), p_dev->DevAddr, 0, NULL, 0, USBH_CFG_STD_REQ_TIMEOUT, &err); /* See Note (1). */
		if (err != 0) {
			usbh_ep_reset(p_dev, NULL);
			k_sleep(K_MSEC(100u));
		} else {
			break;
		}
	}
	if (err != 0) {
		return err;
	}

	if ((p_dev->IsRootHub == true) &&
	    (p_dev->HC_Ptr->IsVirRootHub == true)) {
		return 0;
	}

	k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
	p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Close(&p_dev->HC_Ptr->HC_Drv,
						&p_dev->DfltEP, &err);
	k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);

	p_dev->DfltEP.DevAddr =
		p_dev->DevAddr; /* Update addr.                                         */

	k_mutex_lock(&p_dev->HC_Ptr->HCD_Mutex, K_NO_WAIT);
	p_dev->HC_Ptr->HC_Drv.API_Ptr->EP_Open(&p_dev->HC_Ptr->HC_Drv,
					       &p_dev->DfltEP, &err);
	k_mutex_unlock(&p_dev->HC_Ptr->HCD_Mutex);
	if (err != 0) {
		return err;
	}

	k_sleep(K_MSEC(2)); /* See Note (2).                                        */

	return err;
}

/*
 *********************************************************************************************************
 *                                          USBH_StrDescGet()
 *
 * Description : Read specified string descriptor from USB device.
 *
 * Argument(s) : p_dev            Pointer to USB device.
 *
 *               desc_ix          Index of the string descriptor.
 *
 *               lang_id          Language identifier.
 *
 *               p_buf            Buffer in which the string descriptor will be stored.
 *
 *               buf_len          Buffer length, in octets.
 *
 *               p_err   Pointer to variable that will receive the return error code from this function :
 *
 *                       USBH_ERR_NONE,                          String descriptor successfully fetched.
 *                       USBH_ERR_DESC_INVALID,                  Invalid string descriptor fetched.
 *
 *                                                               ----- RETURNED BY USBH_CtrlRx() : -----
 *                       USBH_ERR_UNKNOWN,                       Unknown error occurred.
 *                       USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
 *                       USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
 *                       USBH_ERR_HC_IO,                         Root hub input/output error.
 *                       USBH_ERR_EP_STALL,                      Root hub does not support request.
 *                       Host controller drivers error code,     Otherwise.
 *
 * Return(s)   : Length of string descriptor.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static uint32_t usbh_str_desc_get(struct usbh_dev *p_dev, uint8_t desc_ix,
				  uint16_t lang_id, void *p_buf,
				  uint32_t buf_len, int *p_err)
{
	uint32_t len;
	uint32_t req_len;
	uint8_t i;
	struct usbh_desc_hdr *p_hdr;

	if (desc_ix == USBH_STRING_DESC_LANGID) {
		req_len =
			0x04u; /* Size of lang ID = 4.                                 */
	} else {
		req_len = USBH_LEN_DESC_HDR;
	}
	req_len = MIN(req_len, buf_len);

	for (i = 0; i < USBH_CFG_STD_REQ_RETRY;
	     i++) { /* Retry up to 3 times.                                 */
		len = usbh_ctrl_rx(
			p_dev, USBH_REQ_GET_DESC,
			USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_RECIPIENT_DEV,
			(USBH_DESC_TYPE_STR << 8) | desc_ix, lang_id, p_buf,
			req_len, USBH_CFG_STD_REQ_TIMEOUT, p_err);
		if ((len == 0) || (*p_err == USBH_ERR_EP_STALL)) {
			usbh_ep_reset(
				p_dev,
				NULL);         /* Rst EP to clr HC halt state.                         */
		} else {
			break;
		}
	}

	if (*p_err != 0) {
		return 0;
	}

	p_hdr = (struct usbh_desc_hdr *)p_buf;

	if ((len ==
	     req_len) && /* Chk desc hdr.                                        */
	    (p_hdr->bLength != 0) &&
	    (p_hdr->bDescriptorType == USBH_DESC_TYPE_STR)) {
		len = p_hdr->bLength;
		if (desc_ix == USBH_STRING_DESC_LANGID) {
			return len;
		}
	} else {
		*p_err = USBH_ERR_DESC_INVALID;
		return 0;
	}

	if (len > buf_len) {
		len = buf_len;
	}
	/* Get full str desc.                                   */
	for (i = 0; i < USBH_CFG_STD_REQ_RETRY;
	     i++) { /* Retry up to 3 times.                                 */
		len = usbh_ctrl_rx(p_dev, USBH_REQ_GET_DESC,
				   USBH_REQ_DIR_DEV_TO_HOST |
				   USBH_REQ_RECIPIENT_DEV,
				   (USBH_DESC_TYPE_STR << 8) | desc_ix, lang_id,
				   p_buf, len, USBH_CFG_STD_REQ_TIMEOUT, p_err);

		if ((len == 0) || (*p_err == USBH_ERR_EP_STALL)) {
			usbh_ep_reset(p_dev, NULL);
		} else {
			break;
		}
	}

	if (*p_err != 0) {
		return 0;
	}

	if (len == 0) {
		*p_err = USBH_ERR_DESC_INVALID;
		return 0;
	}

	return len;
}

/*
 *********************************************************************************************************
 *                                            USBH_StrDescPrint()
 *
 * Description : Print specified string index to default output terminal.
 *
 * Argument(s) : p_dev           Pointer to USB device.
 *
 *               str_prefix      Caller's specific prefix string to add.
 *
 *               desc_ix         String descriptor index.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */
// static void USBH_StrDescPrint(USBH_DEV *p_dev, CPU_INT08U *str_prefix,
// 							  CPU_INT08U desc_ix)
// {
// 	USBH_ERR err;
// 	CPU_INT32U str_len;
// 	CPU_INT08U str[USBH_CFG_MAX_STR_LEN];
// 	CPU_INT16U ch;
// 	CPU_INT32U ix;
// 	CPU_INT32U buf_len;

// 	str_len = USBH_StrGet(p_dev, desc_ix, USBH_STRING_DESC_LANGID, &str[0],
// 						  USBH_CFG_MAX_STR_LEN, &err);

// 	printk("%s", str_prefix); /* Print prefix str.                                    */

// 	if (str_len > 0u)
// 	{ /* Print unicode string rd from the dev.                */
// 		buf_len = str_len * 2u;
// 		for (ix = 0u; (buf_len - ix) >= 2u; ix += 2u)
// 		{
// 			ch = sys_get_le16(&str[ix]);
// 			if (ch == 0u)
// 			{
// 				break;
// 			}
// 			printk("%c", ch);
// 		}
// 	}
// 	printk("\r\n");
// }

/*
 *********************************************************************************************************
 *                                         USBH_NextDescGet()
 *
 * Description : Get pointer to next descriptor in buffer that contains configuration data.
 *
 * Argument(s) : p_buf       Pointer to current header in buffer.
 *
 *               p_off       Pointer to buffer offset.
 *
 * Return(s)   : Pointer to next descriptor header.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static struct usbh_desc_hdr *usbh_next_desc_get(void *p_buf, uint32_t *p_offset)
{
	struct usbh_desc_hdr *p_next_hdr;
	struct usbh_desc_hdr *p_hdr;

	p_hdr = (struct usbh_desc_hdr *)
		p_buf; /* Current desc hdr.                                    */

	if (*p_offset ==
	    0) {       /* 1st desc in buf.                                     */
		p_next_hdr = p_hdr;
	} else {        /* Next desc is at end of current desc.                 */
		p_next_hdr = (struct usbh_desc_hdr *)((uint8_t *)p_buf +
						      p_hdr->bLength);
	}

	*p_offset +=
		p_hdr->bLength; /* Update buf offset.                                   */

	return p_next_hdr;
}

/*
 *********************************************************************************************************
 *                                          USBH_FmtSetupReq()
 *
 * Description : Format setup request from setup request structure.
 *
 * Argument(s) : p_setup_req      Variable that holds setup request information.
 *
 *               p_buf_dest       Pointer to buffer that will receive setup request.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_fmt_setup_req(struct usbh_setup_req *p_setup_req,
			       void *p_buf_dest)
{
	// LOG_DBG("FmtSetupReq");
	struct usbh_setup_req *p_buf_dest_setup_req;

	p_buf_dest_setup_req = (struct usbh_setup_req *)p_buf_dest;

	p_buf_dest_setup_req->bmRequestType = p_setup_req->bmRequestType;
	p_buf_dest_setup_req->bRequest = p_setup_req->bRequest;
	p_buf_dest_setup_req->wValue =
		sys_get_le16((uint8_t *)&p_setup_req->wValue);
	p_buf_dest_setup_req->wIndex =
		sys_get_le16((uint8_t *)&p_setup_req->wIndex);
	p_buf_dest_setup_req->wLength =
		sys_get_le16((uint8_t *)&p_setup_req->wLength);
}

/*
 *********************************************************************************************************
 *                                          USBH_ParseDevDesc()
 *
 * Description : Parse device descriptor into device descriptor structure.
 *
 * Argument(s) : p_dev_desc       Variable that will hold parsed device descriptor.
 *
 *               p_buf_src        Pointer to buffer that holds device descriptor.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_parse_dev_desc(struct usbh_dev_desc *p_dev_desc,
				void *p_buf_src)
{
	struct usbh_dev_desc *p_buf_src_dev_desc;

	p_buf_src_dev_desc = (struct usbh_dev_desc *)p_buf_src;

	p_dev_desc->bLength = p_buf_src_dev_desc->bLength;
	p_dev_desc->bDescriptorType = p_buf_src_dev_desc->bDescriptorType;
	p_dev_desc->bcdUSB =
		sys_get_le16((uint8_t *)&p_buf_src_dev_desc->bcdUSB);
	p_dev_desc->bDeviceClass = p_buf_src_dev_desc->bDeviceClass;
	p_dev_desc->bDeviceSubClass = p_buf_src_dev_desc->bDeviceSubClass;
	p_dev_desc->bDeviceProtocol = p_buf_src_dev_desc->bDeviceProtocol;
	p_dev_desc->bMaxPacketSize0 = p_buf_src_dev_desc->bMaxPacketSize0;
	p_dev_desc->idVendor =
		sys_get_le16((uint8_t *)&p_buf_src_dev_desc->idVendor);
	p_dev_desc->idProduct =
		sys_get_le16((uint8_t *)&p_buf_src_dev_desc->idProduct);
	p_dev_desc->bcdDevice =
		sys_get_le16((uint8_t *)&p_buf_src_dev_desc->bcdDevice);
	p_dev_desc->iManufacturer = p_buf_src_dev_desc->iManufacturer;
	p_dev_desc->iProduct = p_buf_src_dev_desc->iProduct;
	p_dev_desc->iSerialNumber = p_buf_src_dev_desc->iSerialNumber;
	p_dev_desc->bNbrConfigurations = p_buf_src_dev_desc->bNbrConfigurations;
}

/*
 *********************************************************************************************************
 *                                          USBH_ParseCfgDesc()
 *
 * Description : Parse configuration descriptor into configuration descriptor structure.
 *
 * Argument(s) : p_cfg_desc       Variable that will hold parsed configuration descriptor.
 *
 *               p_buf_src        Pointer to buffer that holds configuration descriptor.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_parse_cfg_desc(struct usbh_cfg_desc *p_cfg_desc,
				void *p_buf_src)
{
	struct usbh_cfg_desc *p_buf_src_cfg_desc;

	p_buf_src_cfg_desc = (struct usbh_cfg_desc *)p_buf_src;

	p_cfg_desc->bLength = p_buf_src_cfg_desc->bLength;
	p_cfg_desc->bDescriptorType = p_buf_src_cfg_desc->bDescriptorType;
	p_cfg_desc->wTotalLength =
		sys_get_le16((uint8_t *)&p_buf_src_cfg_desc->wTotalLength);
	p_cfg_desc->bNbrInterfaces = p_buf_src_cfg_desc->bNbrInterfaces;
	p_cfg_desc->bConfigurationValue =
		p_buf_src_cfg_desc->bConfigurationValue;
	p_cfg_desc->iConfiguration = p_buf_src_cfg_desc->iConfiguration;
	p_cfg_desc->bmAttributes = p_buf_src_cfg_desc->bmAttributes;
	p_cfg_desc->bMaxPower = p_buf_src_cfg_desc->bMaxPower;
}

/*
 *********************************************************************************************************
 *                                         USBH_ParseIF_Desc()
 *
 * Description : Parse interface descriptor into interface descriptor structure.
 *
 * Argument(s) : p_if_desc        Variable that will hold parsed interface descriptor.
 *
 *               p_buf_src        Pointer to buffer that holds interface descriptor.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_parse_if_desc(struct usbh_if_desc *p_if_desc, void *p_buf_src)
{
	struct usbh_if_desc *p_buf_src_if_desc;

	p_buf_src_if_desc = (struct usbh_if_desc *)p_buf_src;

	p_if_desc->bLength = p_buf_src_if_desc->bLength;
	p_if_desc->bDescriptorType = p_buf_src_if_desc->bDescriptorType;
	p_if_desc->bInterfaceNumber = p_buf_src_if_desc->bInterfaceNumber;
	p_if_desc->bAlternateSetting = p_buf_src_if_desc->bAlternateSetting;
	p_if_desc->bNbrEndpoints = p_buf_src_if_desc->bNbrEndpoints;
	p_if_desc->bInterfaceClass = p_buf_src_if_desc->bInterfaceClass;
	p_if_desc->bInterfaceSubClass = p_buf_src_if_desc->bInterfaceSubClass;
	p_if_desc->bInterfaceProtocol = p_buf_src_if_desc->bInterfaceProtocol;
	p_if_desc->iInterface = p_buf_src_if_desc->iInterface;
}

/*
 *********************************************************************************************************
 *                                          USBH_ParseEP_Desc()
 *
 * Description : Parse endpoint descriptor into endpoint descriptor structure.
 *
 * Argument(s) : p_ep_desc        Variable that will hold the parsed endpoint descriptor.
 *
 *               p_buf_src        Pointer to buffer that holds endpoint descriptor.
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_parse_ep_desc(struct usbh_ep_desc *p_ep_desc, void *p_buf_src)
{
	struct usbh_ep_desc *p_buf_desc;

	p_buf_desc = (struct usbh_ep_desc *)p_buf_src;
	p_ep_desc->bLength = p_buf_desc->bLength;
	p_ep_desc->bDescriptorType = p_buf_desc->bDescriptorType;
	p_ep_desc->bEndpointAddress = p_buf_desc->bEndpointAddress;
	p_ep_desc->bmAttributes = p_buf_desc->bmAttributes;
	p_ep_desc->wMaxPacketSize =
		sys_get_le16((uint8_t *)&p_buf_desc->wMaxPacketSize);
	p_ep_desc->bInterval = p_buf_desc->bInterval;
	/* Following fields only relevant for isoc EPs.         */
	if ((p_ep_desc->bmAttributes & 0x03) == USBH_EP_TYPE_ISOC) {
		p_ep_desc->bRefresh = p_buf_desc->bRefresh;
		p_ep_desc->bSynchAddress = p_buf_desc->bSynchAddress;
	}
}

/*
 *********************************************************************************************************
 *                                          USBH_AsyncTask()
 *
 * Description : Task that process asynchronous URBs
 *
 * Argument(s) : p_arg       Pointer to a variable (Here it is 0)
 *
 * Return(s)   : None.
 *
 * Note(s)     : None.
 *********************************************************************************************************
 */

static void usbh_async_task(void *p_arg, void *p_arg2, void *p_arg3)
{
	struct usbh_urb *p_urb;
	int key;


	while (true) {
		k_sem_take(&USBH_URB_Sem, K_FOREVER); /* Wait for URBs processed by HC.                       */

		key = irq_lock();
		p_urb = (struct usbh_urb *)USBH_URB_HeadPtr;

		if (USBH_URB_HeadPtr == USBH_URB_TailPtr) {
			USBH_URB_HeadPtr = NULL;
			USBH_URB_TailPtr = NULL;
		} else {
			USBH_URB_HeadPtr = USBH_URB_HeadPtr->NxtPtr;
		}
		irq_unlock(key);

		if (p_urb != NULL) {
			usbh_urb_complete(p_urb);
		}
	}
}
