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
*                                    ATSAMX HOST CONTROLLER DRIVER
*
* Filename : usbh_hcd_atsamx.c
* Version  : V3.42.00
*********************************************************************************************************
* Note(s)  : (1) Due to hardware limitations, the ATSAM D5x/E5x host controller does not support a
*                combination of Full-Speed HUB + Low-Speed device
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#define USBH_HCD_ATSAMX_MODULE
#include "usbh_hcd_atsamx.h"
#include "usbh_hub.h"
#include <usbh_hc_cfg.h>
#include <soc.h>
#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(hcd);

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define USBH_ATSAMX_CTRLA_HOST_MODE \
	DEF_BIT_07 /* HOST Operation mode                                  */
#define USBH_ATSAMX_CTRLA_RUNSTBY \
	DEF_BIT_02 /* Run in standby mode                                  */
#define USBH_ATSAMX_CTRLA_ENABLE \
	DEF_BIT_01 /* Enable                                               */
#define USBH_ATSAMX_CTRLA_SWRST \
	DEF_BIT_00 /* Software reset                                       */

#define USBH_ATSAMX_SYNCBUSY_ENABLE \
	DEF_BIT_01 /* Synchronization enable status                        */
#define USBH_ATSAMX_SYNCBUSY_SWRST \
	DEF_BIT_00 /* Synchronization Software reset status                */
#define USBH_ATSAMX_SYNCBUSY_MSK \
	(USBH_ATSAMX_SYNCBUSY_ENABLE | USBH_ATSAMX_SYNCBUSY_SWRST)

#define USBH_ATSAMX_QOSCTRL_DQOS_HIGH \
	(3u << 2u) /* Data quality of service set to HIGH critical latency */
#define USBH_ATSAMX_QOSCTRL_CQOS_HIGH \
	(3u << 0u) /* Cfg  quality of service set to HIGH critical latency */

#define USBH_ATSAMX_FSMSTATE_OFF 0x01u
#define USBH_ATSAMX_FSMSTATE_ON 0x02u
#define USBH_ATSAMX_FSMSTATE_SUSPEND 0x04u
#define USBH_ATSAMX_FSMSTATE_SLEEP 0x08u
#define USBH_ATSAMX_FSMSTATE_DNRESUME 0x10u
#define USBH_ATSAMX_FSMSTATE_UPRESUME 0x20u
#define USBH_ATSAMX_FSMSTATE_RESET 0x40u

#define USBH_ATSAMX_CTRLB_L1RESUME \
	DEF_BIT_11 /* Send USB L1 resume                                   */
#define USBH_ATSAMX_CTRLB_VBUSOK \
	DEF_BIT_10 /* VBUS is ok                                           */
#define USBH_ATSAMX_CTRLB_BUSRESET \
	DEF_BIT_09 /* Send USB reset                                       */
#define USBH_ATSAMX_CTRLB_SOFE \
	DEF_BIT_08 /* Start-of-Frame enable                                */
#define USBH_ATSAMX_CTRLB_SPDCONF_LSFS \
	(0x0u << 2u) /* Speed configuration for host                         */
#define USBH_ATSAMX_CTRLB_SPDCONF_MSK (0x3u << 2u)
#define USBH_ATSAMX_CTRLB_RESUME \
	DEF_BIT_01 /* Send USB resume                                      */

#define USBH_ATSAMX_HSOFC_FLENCE \
	DEF_BIT_07 /* Frame length control enable                          */

#define USBH_ATSAMX_STATUS_SPEED_POS 2u
#define USBH_ATSAMX_STATUS_SPEED_MSK 0xCu
#define USBH_ATSAMX_STATUS_SPEED_FS 0x0u
#define USBH_ATSAMX_STATUS_SPEED_LS 0x1u
#define USBH_ATSAMX_STATUS_LINESTATE_POS 6u
#define USBH_ATSAMX_STATUS_LINESTATE_MSK 0xC0u

#define USBH_ATSAMX_FNUM_MSK 0x3FF8u
#define USBH_ATSAMX_FNUM_POS 3u

#define USBH_ATSAMX_INT_DDISC \
	DEF_BIT_09 /* Device disconnection interrupt                       */
#define USBH_ATSAMX_INT_DCONN \
	DEF_BIT_08 /* Device connection interrtup                          */
#define USBH_ATSAMX_INT_RAMACER \
	DEF_BIT_07 /* RAM acces interrupt                                  */
#define USBH_ATSAMX_INT_UPRSM \
	DEF_BIT_06 /* Upstream resume from the device interrupt            */
#define USBH_ATSAMX_INT_DNRSM \
	DEF_BIT_05 /* Downstream resume interrupt                          */
#define USBH_ATSAMX_INT_WAKEUP \
	DEF_BIT_04 /* Wake up interrupt                                    */
#define USBH_ATSAMX_INT_RST \
	DEF_BIT_03 /* Bus reset interrupt                                  */
#define USBH_ATSAMX_INT_HSOF \
	DEF_BIT_02 /* Host Start-of-Frame interrupt                        */
#define USBH_ATSAMX_INT_MSK                            \
	(USBH_ATSAMX_INT_DDISC | USBH_ATSAMX_INT_DCONN |   \
	 USBH_ATSAMX_INT_RAMACER | USBH_ATSAMX_INT_UPRSM | \
	 USBH_ATSAMX_INT_DNRSM | USBH_ATSAMX_INT_WAKEUP |  \
	 USBH_ATSAMX_INT_RST | USBH_ATSAMX_INT_HSOF)

#define USBH_ATSAMX_PCFG_PTYPE_MSK (0x7u << 3u)
#define USBH_ATSAMX_PCFG_PTYPE_DISABLED \
	(0x0u << 3u) /* Pipe is disabled                                     */
#define USBH_ATSAMX_PCFG_PTYPE_CONTROL \
	(0x1u << 3u) /* Pipe is enabled and configured as CONTROL            */
#define USBH_ATSAMX_PCFG_PTYPE_ISO \
	(0x2u << 3u) /* Pipe is enabled and configured as ISO                */
#define USBH_ATSAMX_PCFG_PTYPE_BULK \
	(0x3u << 3u) /* Pipe is enabled and configured as BULK               */
#define USBH_ATSAMX_PCFG_PTYPE_INTERRUPT \
	(0x4u << 3u) /* Pipe is enabled and configured as INTERRUPT          */
#define USBH_ATSAMX_PCFG_PTYPE_EXTENDED \
	(0x5u << 3u) /* Pipe is enabled and configured as EXTENDED           */
#define USBH_ATSAMX_PCFG_BK \
	DEF_BIT_02 /* Pipe bank                                            */
#define USBH_ATSAMX_PCFG_PTOKEN_SETUP 0x0u
#define USBH_ATSAMX_PCFG_PTOKEN_IN 0x1u
#define USBH_ATSAMX_PCFG_PTOKEN_OUT 0x2u
#define USBH_ATSAMX_PCFG_PTOKEN_MSK 0x3u

#define USBH_ATSAMX_PSTATUS_BK1RDY \
	DEF_BIT_07 /* Bank 1 ready                                         */
#define USBH_ATSAMX_PSTATUS_BK0RDY \
	DEF_BIT_06 /* Bank 0 ready                                         */
#define USBH_ATSAMX_PSTATUS_PFREEZE \
	DEF_BIT_04 /* Pipe freeze                                          */
#define USBH_ATSAMX_PSTATUS_CURBK \
	DEF_BIT_02 /* Current bank                                         */
#define USBH_ATSAMX_PSTATUS_DTGL \
	DEF_BIT_00 /* Data toggle                                          */

#define USBH_ATSAMX_PINT_STALL \
	DEF_BIT_05 /* Pipe stall received interrupt                        */
#define USBH_ATSAMX_PINT_TXSTP \
	DEF_BIT_04 /* Pipe transmitted setup interrupt                     */
#define USBH_ATSAMX_PINT_PERR \
	DEF_BIT_03 /* Pipe error interrupt                                 */
#define USBH_ATSAMX_PINT_TRFAIL \
	DEF_BIT_02 /* Pipe transfer fail interrupt                         */
#define USBH_ATSAMX_PINT_TRCPT \
	DEF_BIT_00 /* Pipe transfer complete x interrupt                   */
#define USBH_ATSAMX_PINT_ALL                           \
	(USBH_ATSAMX_PINT_STALL | USBH_ATSAMX_PINT_TXSTP | \
	 USBH_ATSAMX_PINT_PERR | USBH_ATSAMX_PINT_TRFAIL | \
	 USBH_ATSAMX_PINT_TRCPT)

#define USBH_ATSAMX_PDESC_BYTE_COUNT_MSK 0x3FFFu
#define USBH_ATSAMX_PDESC_PCKSIZE_BYTE_COUNT_MSK 0x0FFFFFFFu

/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/

#define USBH_ATSAMX_GET_BYTE_CNT(reg) \
	((reg & USBH_ATSAMX_PDESC_BYTE_COUNT_MSK) >> 0u)
#define USBH_ATSAMX_GET_DPID(reg) ((reg & USBH_ATSAMX_PSTATUS_DTGL) >> 0u)
#define USBH_ATSAMX_PCFG_PTYPE(value) ((value + 1u) << 3u)

#define USBH_ATSAMX_WAIT_FOR_SYNC(p_reg, value)                      \
	do                                                               \
	{                                                                \
		; /* Wait for synchronizations bits to be completed       */ \
	} while ((p_reg)->SYNCBUSY & (value));

#define USBH_ATSAMX_CTRLA_READ(p_reg, value)                          \
	do                                                                \
	{                                                                 \
		USBH_ATSAMX_WAIT_FOR_SYNC((p_reg), USBH_ATSAMX_SYNCBUSY_MSK); \
		(value) = (p_reg)->CTRLA;                                     \
	} while (0u);

#define USBH_ATSAMX_CTRLA_WRITE(p_reg, value)                         \
	do                                                                \
	{                                                                 \
		CPU_CRITICAL_ENTER();                                         \
		(p_reg)->CTRLA = (value);                                     \
		USBH_ATSAMX_WAIT_FOR_SYNC((p_reg), USBH_ATSAMX_SYNCBUSY_MSK); \
		CPU_CRITICAL_EXIT();                                          \
	} while (0u);

/*
*********************************************************************************************************
*                                   ATSAMD5X/ATSAME5X DEFINES
*********************************************************************************************************
*/
/* NVM software calibration area address                */
#define ATSAMX_NVM_SW_CAL_AREA ((volatile uint32_t *)0x00800080u)
#define ATSAMX_NVM_USB_TRANSN_POS 32u
#define ATSAMX_NVM_USB_TRANSP_POS 37u
#define ATSAMX_NVM_USB_TRIM_POS 42u

#define ATSAMX_NVM_USB_TRANSN_SIZE 5u
#define ATSAMX_NVM_USB_TRANSP_SIZE 5u
#define ATSAMX_NVM_USB_TRIM_SIZE 3u

#define ATSAMX_NVM_USB_TRANSN                                         \
	((*(ATSAMX_NVM_SW_CAL_AREA + (ATSAMX_NVM_USB_TRANSN_POS / 32)) >> \
	  (ATSAMX_NVM_USB_TRANSN_POS % 32)) &                             \
	 ((1 << ATSAMX_NVM_USB_TRANSN_SIZE) - 1))
#define ATSAMX_NVM_USB_TRANSP                                         \
	((*(ATSAMX_NVM_SW_CAL_AREA + (ATSAMX_NVM_USB_TRANSP_POS / 32)) >> \
	  (ATSAMX_NVM_USB_TRANSP_POS % 32)) &                             \
	 ((1 << ATSAMX_NVM_USB_TRANSP_SIZE) - 1))
#define ATSAMX_NVM_USB_TRIM                                         \
	((*(ATSAMX_NVM_SW_CAL_AREA + (ATSAMX_NVM_USB_TRIM_POS / 32)) >> \
	  (ATSAMX_NVM_USB_TRIM_POS % 32)) &                             \
	 ((1 << ATSAMX_NVM_USB_TRIM_SIZE) - 1))

#define ATSAMX_NVM_USB_TRANSN_MSK 0x1Fu
#define ATSAMX_NVM_USB_TRANSP_MSK 0x1Fu
#define ATSAMX_NVM_USB_TRIM_MSK 0x07u

#define ATSAMX_MAX_NBR_PIPE \
	8u /* Maximum number of host pipes                         */
#define ATSAMX_INVALID_PIPE \
	0xFFu /* Invalid pipe number.                                 */
#define ATSAMX_DFLT_EP_ADDR \
	0xFFFFu /* Default endpoint address.                            */

/* If necessary, re-define these values in 'usbh_cfg.h' */
#ifndef ATSAMX_URB_PROC_TASK_STK_SIZE
#define ATSAMX_URB_PROC_TASK_STK_SIZE 256u
#endif

#ifndef ATSAMX_URB_PROC_TASK_PRIO
#define ATSAMX_URB_PROC_TASK_PRIO 15u
#endif

#ifndef ATSAMX_URB_PROC_Q_MAX
#define ATSAMX_URB_PROC_Q_MAX 8u
#endif

/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           LOCAL DATA TYPES
*********************************************************************************************************
*/

typedef struct usbh_atsamx_desc_bank
{
	/* ----------- USB HOST PIPE DESCRIPTOR BANK ---------- */
	volatile uint32_t ADDR;		 /* Address of the data buffer                           */
	volatile uint32_t PCKSIZE;	 /* Packet size                                          */
	volatile uint16_t EXTREG;	 /* Extended register                                    */
	volatile uint8_t STATUS_BK; /* Host status bank                                     */
	volatile uint8_t RSVD0;
	volatile uint16_t CTRL_PIPE;   /* Host control pipe                                    */
	volatile uint16_t STATUS_PIPE; /* Host Status pipe                                     */
} USBH_ATSAMX_DESC_BANK;

typedef struct usbh_atsamx_pipe_desc
{
	USBH_ATSAMX_DESC_BANK DescBank[2u];
} USBH_ATSAMX_PIPE_DESC;

typedef struct usbh_atsamx_pipe_reg
{
	/* -------------- USB HOST PIPE REGISTERS ------------- */
	volatile uint8_t PCFG; /* Host pipe configuration                              */
	volatile uint8_t RSVD0[2u];
	volatile uint8_t BINTERVAL;  /* Interval for the Bulk-Out/Ping transaction           */
	volatile uint8_t PSTATUSCLR; /* Pipe status clear                                    */
	volatile uint8_t PSTATUSSET; /* Pipe status set                                      */
	volatile uint8_t PSTATUS;	  /* Pipe status register                                 */
	volatile uint8_t PINTFLAG;	  /* Host pipe interrupt flag register                    */
	volatile uint8_t PINTENCLR;  /* Host pipe interrupt clear register                   */
	volatile uint8_t PINTENSET;  /* Host pipe interrupt set register                     */
	volatile uint8_t RSVD1[22u];
} USBH_ATSAMX_PIPE_REG;

typedef struct usbh_atsamx_reg
{
	/* ------------ USB HOST GENERAL REGISTERS ------------ */
	volatile uint8_t CTRLA; /* Control A                                            */
	volatile uint8_t RSVD0;
	volatile uint8_t SYNCBUSY; /* Synchronization Busy                                 */
	volatile uint8_t QOSCTRL;	/* QOS control                                          */
	volatile uint32_t RSVD1;
	volatile uint16_t CTRLB; /* Control B                                            */
	volatile uint8_t HSOFC; /* Host Start-of-Frame control                          */
	volatile uint8_t RSVD2;
	volatile uint8_t STATUS;	 /* Status                                               */
	volatile uint8_t FSMSTATUS; /* Finite state machine status                          */
	volatile uint16_t RSVD3;
	volatile uint16_t FNUM;		/* Host frame number                                    */
	volatile uint8_t FLENHIGH; /* Host frame length                                    */
	volatile uint8_t RSVD4;
	volatile uint16_t INTENCLR; /* Host interrupt enable register clear                 */
	volatile uint16_t RSVD5;
	volatile uint16_t INTENSET; /* Host interrupt enable register set                   */
	volatile uint16_t RSVD6;
	volatile uint16_t INTFLAG; /* Host interrupt flag status and clear                 */
	volatile uint16_t RSVD7;
	volatile uint16_t PINTSMRY; /* Pipe interrupt summary                               */
	volatile uint16_t RSVD8;
	volatile uint32_t DESCADD; /* Descriptor address                                   */
	volatile uint16_t PADCAL;  /* Pad calibration                                      */
	volatile uint8_t RSVD9[214u];
	USBH_ATSAMX_PIPE_REG HPIPE
		[ATSAMX_MAX_NBR_PIPE]; /* Host pipes                                           */
} USBH_ATSAMX_REG;

typedef struct usbh_atsamx_pinfo
{
	uint16_t
	EP_Addr; /* Device addr | EP DIR | EP NBR.                       */
	/* To ensure the URB EP xfer size is not corrupted ...  */
	uint16_t
	AppBufLen; /* ... for multi-transaction transfer                   */
	uint32_t NextXferLen;
	USBH_URB *URB_Ptr;
} USBH_ATSAMX_PINFO;

typedef struct usbh_drv_data
{
	USBH_ATSAMX_PIPE_DESC DescTbl[ATSAMX_MAX_NBR_PIPE];
	USBH_ATSAMX_PINFO PipeTbl[ATSAMX_MAX_NBR_PIPE];
	uint16_t
	PipeUsed; /* Bit array for BULK, ISOC, INTR, CTRL pipe mgmt.      */
	uint8_t RH_Desc
		[USBH_HUB_LEN_HUB_DESC]; /* RH desc content.                                     */
	uint16_t
	RH_PortStat; /* Root Hub Port status.                                */
	uint16_t
	RH_PortChng; /* Root Hub Port status change.                         */
} USBH_DRV_DATA;

/*
*********************************************************************************************************
*                                             LOCAL TABLES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

K_MSGQ_DEFINE(ATSAMX_URB_Proc_Q, sizeof(USBH_URB), ATSAMX_URB_PROC_Q_MAX, 4);
K_THREAD_STACK_DEFINE(ATSAMX_URB_ProcTaskStk, ATSAMX_URB_PROC_TASK_STK_SIZE);
K_MEM_POOL_DEFINE(ATSAMX_DrvMemPool, USBH_DATA_BUF_MAX_LEN, USBH_DATA_BUF_MAX_LEN, (USBH_MAX_NBR_EP_BULK_OPEN + USBH_MAX_NBR_EP_INTR_OPEN + 1), sizeof(CPU_ALIGN));

/*
*********************************************************************************************************
*                                       DRIVER FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/* --------------- DRIVER API FUNCTIONS --------------- */
static void usbh_atsamx_hcd_init(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err);

static void usbh_atsamx_hcd_start(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err);

static void usbh_atsamx_hcd_stop(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err);

static USBH_DEV_SPD usbh_atsamx_hcd_spd_get(USBH_HC_DRV *p_hc_drv,
										   USBH_ERR *p_err);

static void usbh_atsamx_hcd_suspend(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err);

static void usbh_atsamx_hcd_resume(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err);

static uint32_t usbh_atsamx_hcd_frame_nbr_get(USBH_HC_DRV *p_hc_drv,
											  USBH_ERR *p_err);

static void usbh_atsamx_hcd_ep_open(USBH_HC_DRV *p_hc_drv, USBH_EP *p_ep,
									USBH_ERR *p_err);

static void usbh_atsamx_hcd_ep_close(USBH_HC_DRV *p_hc_drv, USBH_EP *p_ep,
									 USBH_ERR *p_err);

static void usbh_atsamx_hcd_ep_abort(USBH_HC_DRV *p_hc_drv, USBH_EP *p_ep,
									 USBH_ERR *p_err);

static bool usbh_atsamx_hcd_ep_is_halt(USBH_HC_DRV *p_hc_drv,
											 USBH_EP *p_ep, USBH_ERR *p_err);

static void usbh_atsamx_hcd_urb_submit(USBH_HC_DRV *p_hc_drv, USBH_URB *p_urb,
									   USBH_ERR *p_err);

static void usbh_atsamx_hcd_urb_complete(USBH_HC_DRV *p_hc_drv, USBH_URB *p_urb,
										 USBH_ERR *p_err);

static void usbh_atsamx_hcd_urb_abort(USBH_HC_DRV *p_hc_drv, USBH_URB *p_urb,
									  USBH_ERR *p_err);

/* -------------- ROOT HUB API FUNCTIONS -------------- */
static bool
usbh_atsamx_rh_port_status_get(USBH_HC_DRV *p_hc_drv, uint8_t port_nbr,
							  USBH_HUB_PORT_STATUS *p_port_status);

static bool usbh_atsamx_rh_hub_desc_get(USBH_HC_DRV *p_hc_drv,
											  void *p_buf,
											  uint8_t buf_len);

static bool usbh_atsamx_rh_port_en_set(USBH_HC_DRV *p_hc_drv,
											 uint8_t port_nbr);

static bool usbh_atsamx_rh_port_en_clr(USBH_HC_DRV *p_hc_drv,
											 uint8_t port_nbr);

static bool usbh_atsamx_rh_port_en_chng_clr(USBH_HC_DRV *p_hc_drv,
												 uint8_t port_nbr);

static bool usbh_atsamx_rh_port_pwr_set(USBH_HC_DRV *p_hc_drv,
											  uint8_t port_nbr);

static bool usbh_atsamx_rh_port_pwr_clr(USBH_HC_DRV *p_hc_drv,
											  uint8_t port_nbr);

static bool usbh_atsamx_hcd_port_reset_set(USBH_HC_DRV *p_hc_drv,
												uint8_t port_nbr);

static bool usbh_atsamx_hcd_port_reset_chng_clr(USBH_HC_DRV *p_hc_drv,
													uint8_t port_nbr);

static bool usbh_atsamx_hcd_port_suspend_clr(USBH_HC_DRV *p_hc_drv,
												  uint8_t port_nbr);

static bool usbh_atsamx_hcd_port_conn_chng_clr(USBH_HC_DRV *p_hc_drv,
												   uint8_t port_nbr);

static bool usbh_atsamx_rh_int_en(USBH_HC_DRV *p_hc_drv);

static bool usbh_atsamx_rh_int_dis(USBH_HC_DRV *p_hc_drv);

/*
*********************************************************************************************************
*                                       LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void usbh_atsamx_isr_callback(void *p_drv);

static void usbh_atsamx_process_urb(void *p_arg, void *p_arg2, void *p_arg3);

static uint8_t usbh_atsamx_get_free_pipe(USBH_DRV_DATA *p_drv_data);

static uint8_t usbh_atsamx_get_pipe_nbr(USBH_DRV_DATA *p_drv_data,
										 USBH_EP *p_ep);

static void usbh_atsamx_pipe_cfg(USBH_URB *p_urb,
								USBH_ATSAMX_PIPE_REG *p_reg_hpipe,
								USBH_ATSAMX_PINFO *p_pipe_info,
								USBH_ATSAMX_DESC_BANK *p_desc_bank);

/*
*********************************************************************************************************
*                                    INITIALIZED GLOBAL VARIABLES
*********************************************************************************************************
*/

USBH_HC_DRV_API USBH_ATSAMX_HCD_DrvAPI = {
	usbh_atsamx_hcd_init,
	usbh_atsamx_hcd_start,
	usbh_atsamx_hcd_stop,
	usbh_atsamx_hcd_spd_get,
	usbh_atsamx_hcd_suspend,
	usbh_atsamx_hcd_resume,
	usbh_atsamx_hcd_frame_nbr_get,

	usbh_atsamx_hcd_ep_open,
	usbh_atsamx_hcd_ep_close,
	usbh_atsamx_hcd_ep_abort,
	usbh_atsamx_hcd_ep_is_halt,

	usbh_atsamx_hcd_urb_submit,
	usbh_atsamx_hcd_urb_complete,
	usbh_atsamx_hcd_urb_abort,
};

USBH_HC_RH_API USBH_ATSAMX_HCD_RH_API = {
	usbh_atsamx_rh_port_status_get,usbh_atsamx_rh_hub_desc_get,
	
	usbh_atsamx_rh_port_en_set, usbh_atsamx_rh_port_en_clr,
	usbh_atsamx_rh_port_en_chng_clr,

	usbh_atsamx_rh_port_pwr_set, usbh_atsamx_rh_port_pwr_clr,

	usbh_atsamx_hcd_port_reset_set, usbh_atsamx_hcd_port_reset_chng_clr,

	usbh_atsamx_hcd_port_suspend_clr, usbh_atsamx_hcd_port_conn_chng_clr,

	usbh_atsamx_rh_int_en, usbh_atsamx_rh_int_dis};

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                           GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                       USBH_ATSAMX_HCD_Init()
*
* Description : Initialize host controller and allocate driver's internal memory/variables.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           HCD init successful.
*                                Specific error code     otherwise.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
struct k_thread htask;

static void usbh_atsamx_hcd_init(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err)
{
	LOG_DBG("atsam hcd init");
	USBH_DRV_DATA *p_drv_data;

	p_drv_data = (USBH_DRV_DATA *)Mem_HeapAlloc(sizeof(USBH_DRV_DATA),
												sizeof(CPU_ALIGN),
												&octets_reqd, &err_lib);
	if (p_drv_data == NULL)
	{
		*p_err = USBH_ERR_ALLOC;
		return;
	}

	Mem_Clr(p_drv_data, sizeof(USBH_DRV_DATA));
	p_hc_drv->DataPtr = (void *)p_drv_data;

	if ((p_hc_drv->HC_CfgPtr->DataBufMaxLen %
		 USBH_MAX_EP_SIZE_TYPE_BULK_FS) != 0u)
	{
		*p_err = USBH_ERR_ALLOC;
		return;
	}

	k_thread_create(&htask, ATSAMX_URB_ProcTaskStk,
					K_THREAD_STACK_SIZEOF(ATSAMX_URB_ProcTaskStk),
					usbh_atsamx_process_urb,
					(void *)p_hc_drv,
					NULL, NULL, 0, 0,
					K_NO_WAIT);

	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                       USBH_ATSAMX_HCD_Start()
*
* Description : Start Host Controller.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           HCD start successful.
*                                Specific error code     otherwise.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
#define DT_DRV_COMPAT atmel_sam0_usb
static USBH_HC_DRV *p_hc_drv_local;
static void usbh_atsamx_hcd_start(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err)
{
	p_hc_drv_local = p_hc_drv;
	USBH_ATSAMX_REG *p_reg;
	USBH_HC_BSP_API *p_bsp_api;
	USBH_DRV_DATA *p_drv_data;
	uint32_t pad_transn;
	uint32_t pad_transp;
	uint32_t pad_trim;
	uint8_t reg_val;
	uint8_t i;
	CPU_SR_ALLOC();

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;
	p_bsp_api = p_hc_drv->BSP_API_Ptr;

	if (p_bsp_api->Init != (void *)0)
	{
		p_bsp_api->Init(p_hc_drv, p_err);
		if (*p_err != USBH_ERR_NONE)
		{
			return;
		}
	}
	if (!(p_reg->SYNCBUSY &
		  USBH_ATSAMX_SYNCBUSY_SWRST))
	{ /* Check if synchronization is completed                */
		USBH_ATSAMX_CTRLA_READ(p_reg, reg_val);
		if (reg_val & USBH_ATSAMX_CTRLA_ENABLE)
		{
			reg_val &= ~USBH_ATSAMX_CTRLA_ENABLE;
			USBH_ATSAMX_CTRLA_WRITE(
				p_reg,
				reg_val); /* Disable USB peripheral                               */
			USBH_ATSAMX_WAIT_FOR_SYNC(p_reg,
									  USBH_ATSAMX_SYNCBUSY_ENABLE)
		}
		/* Resets all regs in the USB to initial state, and ..  */
		/* .. the USB will be disabled   
                               */
		USBH_ATSAMX_CTRLA_WRITE(p_reg, USBH_ATSAMX_CTRLA_SWRST);
	}

	USBH_ATSAMX_WAIT_FOR_SYNC(p_reg, USBH_ATSAMX_SYNCBUSY_SWRST)

	/* -- LOAD PAD CALIBRATION REG WITH PRODUCTION VALUES-- */
	pad_transn = ATSAMX_NVM_USB_TRANSN;
	pad_transp = ATSAMX_NVM_USB_TRANSP;
	pad_trim = ATSAMX_NVM_USB_TRIM;

	if ((pad_transn == 0u) || (pad_transn == ATSAMX_NVM_USB_TRANSN_MSK))
	{
		pad_transn =
			9u; /* Default value                                        */
	}
	if ((pad_transp == 0u) || (pad_transp == ATSAMX_NVM_USB_TRANSP_MSK))
	{
		pad_transp =
			25u; /* Default value                                        */
	}
	if ((pad_trim == 0u) || (pad_trim == ATSAMX_NVM_USB_TRIM_MSK))
	{
		pad_trim =
			6u; /* Default value                                        */
	}

	p_reg->PADCAL =
		((pad_transp << 0u) | (pad_transn << 6) | (pad_trim << 12u));

	/* Write quality of service RAM access                  */
	p_reg->QOSCTRL =
		(USBH_ATSAMX_QOSCTRL_DQOS_HIGH | USBH_ATSAMX_QOSCTRL_CQOS_HIGH);
	/* Set Host mode & set USB clk to run in standby mode   */
	USBH_ATSAMX_CTRLA_WRITE(p_reg, (USBH_ATSAMX_CTRLA_HOST_MODE |
									USBH_ATSAMX_CTRLA_RUNSTBY));

	p_reg->DESCADD =
		(uint32_t)&p_drv_data->DescTbl
			[0u]; /* Set Pipe Descriptor address                          */

	p_reg->CTRLB &=
		~USBH_ATSAMX_CTRLB_SPDCONF_MSK; /* Clear USB speed configuration                        */
	p_reg->CTRLB |=
		(USBH_ATSAMX_CTRLB_SPDCONF_LSFS | /* Set USB LS/FS speed configuration                    */
		 USBH_ATSAMX_CTRLB_VBUSOK);
	if (p_bsp_api->ISR_Reg != (void *)0)
	{
		IRQ_CONNECT(7, 0, usbh_atsamx_isr_callback, 0, 0);
		irq_enable(7);
	}

	USBH_ATSAMX_CTRLA_READ(p_reg, reg_val);
	if ((reg_val & USBH_ATSAMX_CTRLA_ENABLE) == 0u)
	{
		USBH_ATSAMX_CTRLA_WRITE(p_reg,
								(reg_val | USBH_ATSAMX_CTRLA_ENABLE));
		USBH_ATSAMX_WAIT_FOR_SYNC(p_reg, USBH_ATSAMX_SYNCBUSY_ENABLE)
	}

	for (i = 0u; i < ATSAMX_MAX_NBR_PIPE; i++)
	{
		p_reg->HPIPE[i].PCFG =
			0u; /* Disable the pipe                                     */
		/* Set default pipe info fields.                        */
		p_drv_data->PipeTbl[i].AppBufLen = 0u;
		p_drv_data->PipeTbl[i].NextXferLen = 0u;
		p_drv_data->PipeTbl[i].EP_Addr = ATSAMX_DFLT_EP_ADDR;
	}

	p_reg->INTFLAG =
		USBH_ATSAMX_INT_MSK; /* Clear all interrupts                                 */
	p_reg->INTENSET =
		(USBH_ATSAMX_INT_DCONN | /* Enable interrupts to detect connection               */
		 USBH_ATSAMX_INT_RST | USBH_ATSAMX_INT_WAKEUP);
	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                       usbh_atsamx_hcd_stop()
*
* Description : Stop Host Controller.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           HCD stop successful.
*                                Specific error code     otherwise.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_stop(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err)
{
	LOG_DBG("Stop");

	USBH_ATSAMX_REG *p_reg;
	USBH_HC_BSP_API *p_bsp_api;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_bsp_api = p_hc_drv->BSP_API_Ptr;

	p_reg->INTENCLR =
		USBH_ATSAMX_INT_MSK; /* Disable all interrupts                               */
	p_reg->INTFLAG =
		USBH_ATSAMX_INT_MSK; /* Clear all interrupts                                 */

	if (p_bsp_api->ISR_Unreg != (void *)0)
	{
		irq_disable(7);
	}

	p_reg->CTRLB &= ~USBH_ATSAMX_CTRLB_VBUSOK;

	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                      usbh_atsamx_hcd_spd_get()
*
* Description : Returns Host Controller speed.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           Host controller speed retrieved successfuly.
*
* Return(s)   : Host controller speed.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static USBH_DEV_SPD usbh_atsamx_hcd_spd_get(USBH_HC_DRV *p_hc_drv,
										   USBH_ERR *p_err)
{
	(void)p_hc_drv;

	*p_err = USBH_ERR_NONE;

	return (USBH_DEV_SPD_FULL);
}

/*
*********************************************************************************************************
*                                      usbh_atsamx_hcd_suspend()
*
* Description : Suspend Host Controller.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           HCD suspend successful.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_suspend(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err)
{
	LOG_DBG("Suspend");
	USBH_ATSAMX_REG *p_reg;
	USBH_DRV_DATA *p_drv_data;
	uint8_t pipe_nbr;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	for (pipe_nbr = 0u; pipe_nbr < ATSAMX_MAX_NBR_PIPE; pipe_nbr++)
	{
		if (DEF_BIT_IS_SET(p_drv_data->PipeUsed, DEF_BIT(pipe_nbr)))
		{
			p_reg->HPIPE[pipe_nbr].PSTATUSSET =
				USBH_ATSAMX_PSTATUS_PFREEZE; /* Stop transfer                          */
		}
	}

	USBH_OS_DlyMS(
		1u); /* wait at least 1 complete frame                       */

	p_reg->CTRLB &=
		~USBH_ATSAMX_CTRLB_SOFE; /* Stop sending start of frames                         */

	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                      usbh_atsamx_hcd_resume()
*
* Description : Resume Host Controller.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           HCD resume successful.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_resume(USBH_HC_DRV *p_hc_drv, USBH_ERR *p_err)
{
	LOG_DBG("Resume");
	USBH_ATSAMX_REG *p_reg;
	USBH_DRV_DATA *p_drv_data;
	uint8_t pipe_nbr;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	p_reg->CTRLB |=
		USBH_ATSAMX_CTRLB_SOFE; /* Start sending start of frames                        */
	p_reg->CTRLB |=
		USBH_ATSAMX_CTRLB_RESUME; /* Do resume to downstream                              */
	p_reg->INTENSET =
		USBH_ATSAMX_INT_WAKEUP; /* Force a wakeup interrupt                             */

	USBH_OS_DlyMS(20u);

	for (pipe_nbr = 0u; pipe_nbr < ATSAMX_MAX_NBR_PIPE; pipe_nbr++)
	{
		if (DEF_BIT_IS_SET(p_drv_data->PipeUsed, DEF_BIT(pipe_nbr)))
		{
			p_reg->HPIPE[pipe_nbr].PSTATUSCLR =
				USBH_ATSAMX_PSTATUS_PFREEZE; /* Start transfer                         */
		}
	}

	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                    usbh_atsamx_hcd_frame_nbr_get()
*
* Description : Retrieve current frame number.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           HC frame number retrieved successfuly.
*
* Return(s)   : Frame number.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static uint32_t usbh_atsamx_hcd_frame_nbr_get(USBH_HC_DRV *p_hc_drv,
											  USBH_ERR *p_err)
{
	USBH_ATSAMX_REG *p_reg;
	uint32_t frm_nbr;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	frm_nbr = (p_reg->FNUM & USBH_ATSAMX_FNUM_MSK) >> USBH_ATSAMX_FNUM_POS;

	*p_err = USBH_ERR_NONE;

	return (frm_nbr);
}

/*
*********************************************************************************************************
*                                      usbh_atsamx_hcd_ep_open()
*
* Description : Allocate/open endpoint of given type.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_ep         Pointer to endpoint structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           Endpoint opened successfuly.
*                                Specific error code     otherwise.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_ep_open(USBH_HC_DRV *p_hc_drv, USBH_EP *p_ep,
									USBH_ERR *p_err)
{
	LOG_DBG("EP_Open");
	USBH_ATSAMX_REG *p_reg;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_ep->DataPID =
		0u; /* Set PID to DATA0                                     */
	p_ep->URB.Err = USBH_ERR_NONE;
	*p_err = USBH_ERR_NONE;

	if (p_reg->STATUS ==
		0u)
	{ /* Do not open Endpoint if device is disconnected       */
		LOG_ERR("device not connected");
		*p_err = USBH_ERR_FAIL;
		return;
	}
}

/*
*********************************************************************************************************
*                                     usbh_atsamx_hcd_ep_close()
*
* Description : Close specified endpoint.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_ep         Pointer to endpoint structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           Endpoint closed successfully.
*                                Specific error code     otherwise.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_ep_close(USBH_HC_DRV *p_hc_drv, USBH_EP *p_ep,
									 USBH_ERR *p_err)
{
	LOG_DBG("EP_Close");
	USBH_ATSAMX_REG *p_reg;
	USBH_DRV_DATA *p_drv_data;
	uint8_t pipe_nbr;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;
	pipe_nbr = usbh_atsamx_get_pipe_nbr(p_drv_data, p_ep);
	p_ep->DataPID =
		0u; /* Set PID to DATA0                                      */
	*p_err = USBH_ERR_NONE;

	if (p_ep->URB.DMA_BufPtr != (void *)0u)
	{
		Mem_PoolBlkFree(&ATSAMX_DrvMemPool, p_ep->URB.DMA_BufPtr,
						&err_lib);
		p_ep->URB.DMA_BufPtr = (void *)0u;
	}

	if (pipe_nbr != ATSAMX_INVALID_PIPE)
	{
		p_reg->HPIPE[pipe_nbr].PINTENCLR =
			USBH_ATSAMX_PINT_ALL; /* Disable all pipe interrupts                           */
		p_reg->HPIPE[pipe_nbr].PINTFLAG =
			USBH_ATSAMX_PINT_ALL; /* Clear   all pipe interrupt flags                      */
		p_reg->HPIPE[pipe_nbr].PCFG =
			0u; /* Disable the pipe                                      */

		p_drv_data->PipeTbl[pipe_nbr].EP_Addr = ATSAMX_DFLT_EP_ADDR;
		p_drv_data->PipeTbl[pipe_nbr].AppBufLen = 0u;
		p_drv_data->PipeTbl[pipe_nbr].NextXferLen = 0u;
		DEF_BIT_CLR(p_drv_data->PipeUsed, DEF_BIT(pipe_nbr));
	}
}

/*
*********************************************************************************************************
*                                     usbh_atsamx_hcd_ep_abort()
*
* Description : Abort all pending URBs on endpoint.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_ep         Pointer to endpoint structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           Endpoint aborted successfuly.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_ep_abort(USBH_HC_DRV *p_hc_drv, USBH_EP *p_ep,
									 USBH_ERR *p_err)
{
	(void)p_hc_drv;
	(void)p_ep;

	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                    usbh_atsamx_hcd_ep_is_halt()
*
* Description : Retrieve endpoint halt state.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_ep         Pointer to endpoint structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           Endpoint halt status retrieved successfuly.
*
* Return(s)   : DEF_TRUE,       If endpoint halted.
*
*               DEF_FALSE,      Otherwise.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_hcd_ep_is_halt(USBH_HC_DRV *p_hc_drv,
											 USBH_EP *p_ep, USBH_ERR *p_err)
{
	(void)p_hc_drv;

	*p_err = USBH_ERR_NONE;
	if (p_ep->URB.Err == USBH_ERR_HC_IO)
	{
		return (DEF_TRUE);
	}

	return (DEF_FALSE);
}

/*
*********************************************************************************************************
*                                    usbh_atsamx_hcd_urb_submit()
*
* Description : Submit specified URB.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_urb        Pointer to URB structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           URB submitted successfuly.
*                                Specific error code     otherwise.
*
* Return(s)   : None.
*
* Note(s)     : (1) If Device is disconnected & some remaining URBs processed by AsyncThread, ignore it
*               (2) Set minimum BULK interval to 1ms to avoid taking all bandwidth.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_urb_submit(USBH_HC_DRV *p_hc_drv, USBH_URB *p_urb,
									   USBH_ERR *p_err)
{
	USBH_ATSAMX_REG *p_reg;
	USBH_DRV_DATA *p_drv_data;
	uint8_t ep_type;
	uint8_t pipe_nbr;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;
	ep_type = usbh_ep_type_get(p_urb->EP_Ptr);

	if (p_urb->State ==
		USBH_URB_STATE_ABORTED)
	{ /* See note 1.                                          */
		*p_err = USBH_ERR_FAIL;
		return;
	}

	pipe_nbr = usbh_atsamx_get_free_pipe(p_drv_data);
	if (pipe_nbr == ATSAMX_INVALID_PIPE)
	{
		*p_err = USBH_ERR_EP_ALLOC;
		return;
	}
	LOG_DBG("pipe %d set ep adress %d", pipe_nbr, ((p_urb->EP_Ptr->DevAddr << 8u) | p_urb->EP_Ptr->Desc.bEndpointAddress));
	p_drv_data->PipeTbl[pipe_nbr].EP_Addr =
		((p_urb->EP_Ptr->DevAddr << 8u) |
		 p_urb->EP_Ptr->Desc.bEndpointAddress);
	p_drv_data->PipeTbl[pipe_nbr].AppBufLen = 0u;
	p_drv_data->PipeTbl[pipe_nbr].NextXferLen = 0u;

	if (p_urb->DMA_BufPtr == (void *)0u)
	{
		p_urb->DMA_BufPtr =
			Mem_PoolBlkGet(&ATSAMX_DrvMemPool,
						   USBH_DATA_BUF_MAX_LEN, &err_lib);
		if (p_urb->DMA_BufPtr == NULL)
		{
			*p_err = USBH_ERR_HC_ALLOC;
			return;
		}
	}

	p_reg->HPIPE[pipe_nbr].PCFG = USBH_ATSAMX_PCFG_PTYPE(
		ep_type); /* Set pipe type and single bank               */
	p_reg->HPIPE[pipe_nbr].PINTENCLR =
		USBH_ATSAMX_PINT_ALL; /* Disable all interrupts                               */
	p_reg->HPIPE[pipe_nbr].PINTFLAG =
		USBH_ATSAMX_PINT_ALL; /* Clear   all interrupts                               */

	/*enable general error and stall interrupts             */
	p_reg->HPIPE[pipe_nbr].PINTENSET =
		(USBH_ATSAMX_PINT_STALL | USBH_ATSAMX_PINT_PERR);

	if ((ep_type == USBH_EP_TYPE_BULK) && (p_urb->EP_Ptr->Interval < 1u))
	{
		p_reg->HPIPE[pipe_nbr].BINTERVAL =
			1u; /* See Note 2.                                          */
	}
	else
	{
		p_reg->HPIPE[pipe_nbr].BINTERVAL = p_urb->EP_Ptr->Interval;
	}

	p_drv_data->PipeTbl[pipe_nbr].URB_Ptr =
		p_urb; /* Store transaction URB                                */

	switch (ep_type)
	{
	case USBH_EP_TYPE_CTRL:
		if (p_urb->Token == USBH_TOKEN_SETUP)
		{
			p_reg->HPIPE[pipe_nbr].PINTENSET =
				USBH_ATSAMX_PINT_TXSTP; /* Enable setup transfer interrupt      */
			p_reg->HPIPE[pipe_nbr].PSTATUSCLR =
				USBH_ATSAMX_PSTATUS_DTGL; /* Set PID to DATA0                     */
		}
		else
		{
			p_reg->HPIPE[pipe_nbr].PINTENSET =
				USBH_ATSAMX_PINT_TRCPT; /* Enable transfer complete interrupt   */
			p_reg->HPIPE[pipe_nbr].PSTATUSSET =
				USBH_ATSAMX_PSTATUS_DTGL; /* Set PID to DATA1                     */
		}
		break;

	case USBH_EP_TYPE_INTR:
	case USBH_EP_TYPE_BULK:
		p_reg->HPIPE[pipe_nbr].PINTENSET =
			USBH_ATSAMX_PINT_TRCPT; /* Enable transfer complete interrupt   */

		if (p_urb->EP_Ptr->DataPID == 0u)
		{
			p_reg->HPIPE[pipe_nbr].PSTATUSCLR =
				USBH_ATSAMX_PSTATUS_DTGL; /* Set PID to DATA0                     */
		}
		else
		{
			p_reg->HPIPE[pipe_nbr].PSTATUSSET =
				USBH_ATSAMX_PSTATUS_DTGL; /* Set PID to DATA1                     */
		}
		break;

	case USBH_EP_TYPE_ISOC:
	default:
		*p_err = USBH_ERR_NOT_SUPPORTED;
		return;
	}

	usbh_atsamx_pipe_cfg(p_urb, &p_reg->HPIPE[pipe_nbr],
						&p_drv_data->PipeTbl[pipe_nbr],
						&p_drv_data->DescTbl[pipe_nbr].DescBank[0]);
	/* Start transfer                                       */
	p_reg->HPIPE[pipe_nbr].PSTATUSCLR = USBH_ATSAMX_PSTATUS_PFREEZE;
	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                   usbh_atsamx_hcd_urb_complete()
*
* Description : Complete specified URB.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_urb        Pointer to URB structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           URB completed successfuly.
*                                Specific error code     otherwise.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_urb_complete(USBH_HC_DRV *p_hc_drv, USBH_URB *p_urb,
										 USBH_ERR *p_err)
{
	USBH_ATSAMX_REG *p_reg;
	USBH_DRV_DATA *p_drv_data;
	uint8_t pipe_nbr;
	uint16_t xfer_len;
	CPU_SR_ALLOC();

	*p_err = USBH_ERR_NONE;
	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;
	pipe_nbr = usbh_atsamx_get_pipe_nbr(p_drv_data, p_urb->EP_Ptr);

	CPU_CRITICAL_ENTER();
	xfer_len = USBH_ATSAMX_GET_BYTE_CNT(
		p_drv_data->DescTbl[pipe_nbr].DescBank[0].PCKSIZE);

	if (p_urb->Token ==
		USBH_TOKEN_IN)
	{ /* -------------- HANDLE IN TRANSACTIONS -------------- */
		Mem_Copy((void *)((uint32_t)p_urb->UserBufPtr +
						  p_urb->XferLen),
				 p_urb->DMA_BufPtr, xfer_len);

		/* Check if it rx'd more data than what was expected    */
		if (p_drv_data->PipeTbl[pipe_nbr].AppBufLen >
			p_urb->UserBufLen)
		{
			p_urb->XferLen += xfer_len;
			p_urb->Err = USBH_ERR_HC_IO;
		}
		else
		{
			p_urb->XferLen += xfer_len;
		}
	}
	else
	{ /* ----------- HANDLE SETUP/OUT TRANSACTIONS ---------- */
		xfer_len = (p_drv_data->PipeTbl[pipe_nbr].AppBufLen - xfer_len);
		if (xfer_len == 0u)
		{
			p_urb->XferLen +=
				p_drv_data->PipeTbl[pipe_nbr].NextXferLen;
		}
		else
		{
			p_urb->XferLen +=
				(p_drv_data->PipeTbl[pipe_nbr].NextXferLen -
				 xfer_len);
		}
	}

	p_reg->HPIPE[pipe_nbr].PINTENCLR =
		USBH_ATSAMX_PINT_ALL; /* Disable all pipe interrupts                          */
	p_reg->HPIPE[pipe_nbr].PINTFLAG =
		USBH_ATSAMX_PINT_ALL; /* Clear   all pipe interrupts                          */
	p_reg->HPIPE[pipe_nbr].PCFG =
		0u; /* Disable the pipe                                     */
	CPU_CRITICAL_EXIT();

	Mem_PoolBlkFree(&ATSAMX_DrvMemPool, p_urb->DMA_BufPtr, &err_lib);
	p_urb->DMA_BufPtr = (void *)0u;

	p_drv_data->PipeTbl[pipe_nbr].EP_Addr = ATSAMX_DFLT_EP_ADDR;
	p_drv_data->PipeTbl[pipe_nbr].AppBufLen = 0u;
	p_drv_data->PipeTbl[pipe_nbr].NextXferLen = 0u;
	p_drv_data->PipeTbl[pipe_nbr].URB_Ptr = (USBH_URB *)0u;
	DEF_BIT_CLR(p_drv_data->PipeUsed, DEF_BIT(pipe_nbr));
}

/*
*********************************************************************************************************
*                                     usbh_atsamx_hcd_urb_abort()
*
* Description : Abort specified URB.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               p_urb        Pointer to URB structure.
*
*               p_err        Pointer to variable that will receive the return error code from this function
*                                USBH_ERR_NONE           URB aborted successfuly.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_hcd_urb_abort(USBH_HC_DRV *p_hc_drv, USBH_URB *p_urb,
									  USBH_ERR *p_err)
{
	(void)p_hc_drv;

	p_urb->State = USBH_URB_STATE_ABORTED;
	p_urb->Err = USBH_ERR_URB_ABORT;

	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*********************************************************************************************************
*                                         ROOT HUB FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                   usbh_atsamx_rh_port_status_get()
*
* Description : Retrieve port status changes and port status.
*
* Argument(s) : p_hc_drv          Pointer to host controller driver structure.
*
*               port_nbr          Port Number.
*
*               p_port_status     Pointer to structure that will receive port status.
*
* Return(s)   : DEF_OK,           If successful.
*               DEF_FAIL,         otherwise.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool
usbh_atsamx_rh_port_status_get(USBH_HC_DRV *p_hc_drv, uint8_t port_nbr,
							  USBH_HUB_PORT_STATUS *p_port_status)
{
	USBH_ATSAMX_REG *p_reg;
	USBH_DRV_DATA *p_drv_data;
	uint8_t reg_val;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	if (port_nbr != 1u)
	{
		p_port_status->wPortStatus = 0u;
		p_port_status->wPortChange = 0u;
		return (DEF_FAIL);
	}
	/* Bits not used by the stack. Maintain constant value. */
	p_drv_data->RH_PortStat &= ~USBH_HUB_STATUS_PORT_TEST;
	p_drv_data->RH_PortStat &= ~USBH_HUB_STATUS_PORT_INDICATOR;

	reg_val = (p_reg->STATUS & USBH_ATSAMX_STATUS_SPEED_MSK) >>
			  USBH_ATSAMX_STATUS_SPEED_POS;
	if (reg_val ==
		USBH_ATSAMX_STATUS_SPEED_FS)
	{ /* ------------- FULL-SPEED DEVICE ATTACHED ----------- */
		p_drv_data->RH_PortStat &=
			~USBH_HUB_STATUS_PORT_LOW_SPD; /* PORT_LOW_SPEED  = 0 = FS dev attached.        */
		p_drv_data->RH_PortStat &=
			~USBH_HUB_STATUS_PORT_HIGH_SPD; /* PORT_HIGH_SPEED = 0 = FS dev attached.        */
	}
	else if (reg_val ==
			 USBH_ATSAMX_STATUS_SPEED_LS)
	{ /* ------------- LOW-SPEED DEVICE ATTACHED ------------ */
		p_drv_data->RH_PortStat |=
			USBH_HUB_STATUS_PORT_LOW_SPD; /* PORT_LOW_SPEED  = 1 = LS dev attached.        */
		p_drv_data->RH_PortStat &=
			~USBH_HUB_STATUS_PORT_HIGH_SPD; /* PORT_HIGH_SPEED = 0 = LS dev attached.        */
	}

	p_port_status->wPortStatus = p_drv_data->RH_PortStat;
	p_port_status->wPortChange = p_drv_data->RH_PortChng;

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                    usbh_atsamx_rh_hub_desc_get()
*
* Description : Retrieve root hub descriptor.
*
* Argument(s) : p_hc_drv    Pointer to host controller driver structure.
*
*               p_buf       Pointer to buffer that will receive hub descriptor.
*
*               buf_len     Buffer length in octets.
*
* Return(s)   : DEF_OK,         If successful.
*               DEF_FAIL,       otherwise.
*
* Note(s)     : (1) For more information about the hub descriptor, see 'Universal Serial Bus Specification
*                   Revision 2.0', Chapter 11.23.2.
*
*               (2) (a) 'bNbrPorts' is assigned the "number of downstream facing ports that this hub
*                       supports".
*
*                   (b) 'wHubCharacteristics' is a bit-mapped variables as follows :
*
*                       (1) Bits 0-1 (Logical Power Switching Mode) :
*                                       00b = All ports' power at once.
*                                       01b = Individual port power switching.
*                                       1xb = No power switching.
*
*                       (2) Bit 2 (Compound Device Identifier).
*
*                       (3) Bits 3-4 (Over-current Protection Mode).
*
*                       (4) Bits 5-6 (TT Think Time).
*
*                       (5) Bit  7 (Port Indicators Support).
*
*                   (c) 'bPwrOn2PwrGood' is assigned the "time (in 2 ms intervals) from the time the
*                       power-on sequence begins on a port until power is good on that port.
*
*                   (d) 'bHubContrCurrent' is assigned the "maximum current requirements of the Hub
*                       Controller electronics in mA."
*********************************************************************************************************
*/

static bool usbh_atsamx_rh_hub_desc_get(USBH_HC_DRV *p_hc_drv,
											  void *p_buf,
											  uint8_t buf_len)
{
	USBH_DRV_DATA *p_drv_data;
	USBH_HUB_DESC hub_desc;

	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	hub_desc.bDescLength = USBH_HUB_LEN_HUB_DESC;
	hub_desc.bDescriptorType = USBH_HUB_DESC_TYPE_HUB;
	hub_desc.bNbrPorts = 1u;
	hub_desc.wHubCharacteristics = 0u;
	hub_desc.bPwrOn2PwrGood = 100u;
	hub_desc.bHubContrCurrent = 0u;

	USBH_HUB_FmtHubDesc(
		&hub_desc,
		p_drv_data->RH_Desc); /* Write the structure in USB format                    */

	buf_len = DEF_MIN(buf_len, sizeof(USBH_HUB_DESC));
	Mem_Copy(
		p_buf, p_drv_data->RH_Desc,
		buf_len); /* Copy the formatted structure into the buffer         */

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                     usbh_atsamx_rh_port_en_set()
*
* Description : Enable given port.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_rh_port_en_set(USBH_HC_DRV *p_hc_drv,
											 uint8_t port_nbr)
{
	(void)p_hc_drv;
	(void)port_nbr;

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                     usbh_atsamx_rh_port_en_clr()
*
* Description : Clear port enable status.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_rh_port_en_clr(USBH_HC_DRV *p_hc_drv,
											 uint8_t port_nbr)
{
	USBH_DRV_DATA *p_drv_data;

	(void)port_nbr;

	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	p_drv_data->RH_PortStat &=
		~USBH_HUB_STATUS_PORT_EN; /* Bit is clr by ClearPortFeature(PORT_ENABLE)          */

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                   usbh_atsamx_rh_port_en_chng_clr()
*
* Description : Clear port enable status change.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_rh_port_en_chng_clr(USBH_HC_DRV *p_hc_drv,
												 uint8_t port_nbr)
{
	USBH_DRV_DATA *p_drv_data;

	(void)port_nbr;

	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	p_drv_data->RH_PortChng &=
		~USBH_HUB_STATUS_C_PORT_EN; /* Bit is clr by ClearPortFeature(C_PORT_ENABLE)        */

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                    usbh_atsamx_rh_port_pwr_set()
*
* Description : Set port power based on port power mode.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_rh_port_pwr_set(USBH_HC_DRV *p_hc_drv,
											  uint8_t port_nbr)
{
	USBH_DRV_DATA *p_drv_data;

	(void)port_nbr;

	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	p_drv_data->RH_PortStat |=
		USBH_HUB_STATUS_PORT_PWR; /* Bit is set by SetPortFeature(PORT_POWER) request     */

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                    usbh_atsamx_rh_port_pwr_clr()
*
* Description : Clear port power.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_rh_port_pwr_clr(USBH_HC_DRV *p_hc_drv,
											  uint8_t port_nbr)
{
	(void)p_hc_drv;
	(void)port_nbr;

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                   usbh_atsamx_hcd_port_reset_set()
*
* Description : Reset given port.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_hcd_port_reset_set(USBH_HC_DRV *p_hc_drv,
												uint8_t port_nbr)
{
	USBH_ATSAMX_REG *p_reg;
	USBH_DRV_DATA *p_drv_data;

	(void)port_nbr;

	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	p_drv_data->RH_PortStat |=
		USBH_HUB_STATUS_PORT_RESET; /* This bit is set while in the resetting state         */
	p_reg->CTRLB |=
		USBH_ATSAMX_CTRLB_BUSRESET; /* Send USB reset signal.                               */

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                 usbh_atsamx_hcd_port_reset_chng_clr()
*
* Description : Clear port reset status change.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_hcd_port_reset_chng_clr(USBH_HC_DRV *p_hc_drv,
													uint8_t port_nbr)
{
	USBH_DRV_DATA *p_drv_data;

	(void)port_nbr;

	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;

	p_drv_data->RH_PortChng &=
		~USBH_HUB_STATUS_C_PORT_RESET; /* Bit is clr by ClearPortFeature(C_PORT_RESET) request */

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                  usbh_atsamx_hcd_port_suspend_clr()
*
* Description : Resume given port if port is suspended.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_hcd_port_suspend_clr(USBH_HC_DRV *p_hc_drv,
												  uint8_t port_nbr)
{
	(void)p_hc_drv;
	(void)port_nbr;

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                  usbh_atsamx_hcd_port_conn_chng_clr()
*
* Description : Clear port connect status change.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
*               port_nbr     Port Number.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_hcd_port_conn_chng_clr(USBH_HC_DRV *p_hc_drv,
												   uint8_t port_nbr)
{
	USBH_DRV_DATA *p_drv_data;

	(void)port_nbr;

	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;
	DEF_BIT_CLR(p_drv_data->RH_PortChng, USBH_HUB_STATUS_C_PORT_CONN);

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                    usbh_atsamx_rh_int_en()
*
* Description : Enable root hub interrupt.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_rh_int_en(USBH_HC_DRV *p_hc_drv)
{
	(void)p_hc_drv;

	return (DEF_OK);
}

/*
*********************************************************************************************************
*                                    usbh_atsamx_rh_int_dis()
*
* Description : Disable root hub interrupt.
*
* Argument(s) : p_hc_drv     Pointer to host controller driver structure.
*
* Return(s)   : DEF_OK,      If successful.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static bool usbh_atsamx_rh_int_dis(USBH_HC_DRV *p_hc_drv)
{

	(void)p_hc_drv;

	return (DEF_OK);
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
*                                      usbh_atsamx_isr_callback()
*
* Description : ISR handler.
*
* Arguments   : p_drv     Pointer to host controller driver structure.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_isr_callback(void *p_drv)
{
	USBH_ATSAMX_REG *p_reg;
	USBH_DRV_DATA *p_drv_data;
	USBH_HC_DRV *p_hc_drv;
	uint16_t int_stat;
	uint16_t pipe_stat;
	uint16_t pipe_nbr;
	uint16_t xfer_len;
	uint16_t max_pkt_size;
	USBH_URB *p_urb;

	p_hc_drv = (USBH_HC_DRV *)p_hc_drv_local;
	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;
	int_stat = p_reg->INTFLAG;
	int_stat &= p_reg->INTENSET;

	/* ----------- HANDLE INTERRUPT FLAG STATUS ----------- */
	if (int_stat & USBH_ATSAMX_INT_RST)
	{
		LOG_DBG("bus reset");
		p_reg->INTFLAG =
			USBH_ATSAMX_INT_RST; /* Clear BUS RESET interrupt flag                       */

		p_drv_data->RH_PortStat |=
			USBH_HUB_STATUS_PORT_EN; /* Bit may be set due to SetPortFeature(PORT_RESET)     */
		p_drv_data->RH_PortChng |=
			USBH_HUB_STATUS_C_PORT_RESET; /* Transitioned from RESET state to ENABLED State       */

		USBH_HUB_RH_Event(
			p_hc_drv->RH_DevPtr); /* Notify the core layer.                               */
	}
	else if (int_stat & USBH_ATSAMX_INT_DDISC)
	{
		LOG_DBG("disconnect usb");

		/* Clear device disconnect/connect interrupt flags      */
		p_reg->INTFLAG =
			(USBH_ATSAMX_INT_DDISC | USBH_ATSAMX_INT_DCONN);
		p_reg->INTENCLR =
			(USBH_ATSAMX_INT_DDISC |
			 USBH_ATSAMX_INT_WAKEUP); /* Disable disconnect/wakeup interrupt     */

		/* Clear device connect/wakeup interrupt flags          */
		p_reg->INTFLAG =
			(USBH_ATSAMX_INT_DCONN | USBH_ATSAMX_INT_WAKEUP);
		p_reg->INTENSET =
			(USBH_ATSAMX_INT_DCONN |
			 USBH_ATSAMX_INT_WAKEUP); /* Enable connect/wakeup interrupt         */

		p_drv_data->RH_PortStat =
			0u; /* Clear Root hub Port Status bits                      */
		p_drv_data->RH_PortChng |=
			USBH_HUB_STATUS_C_PORT_CONN; /* Current connect status has changed                   */
		USBH_HUB_RH_Event(
			p_hc_drv->RH_DevPtr); /* Notify the core layer.                               */
	}
	else if (int_stat & USBH_ATSAMX_INT_DCONN)
	{
		LOG_DBG("connect usb");

		p_reg->INTENCLR =
			USBH_ATSAMX_INT_DCONN; /* Disable device connect interrupt                     */

		p_drv_data->RH_PortStat |=
			USBH_HUB_STATUS_PORT_CONN; /* Bit reflects if a device is currently connected      */
		p_drv_data->RH_PortChng |=
			USBH_HUB_STATUS_C_PORT_CONN; /* Bit indicates a Port's current connect status change */

		p_reg->INTFLAG =
			USBH_ATSAMX_INT_DDISC; /* Clear  disconnection interrupt                       */
		p_reg->INTENSET =
			USBH_ATSAMX_INT_DDISC; /* Enable disconnection interrupt                       */

		USBH_HUB_RH_Event(
			p_hc_drv->RH_DevPtr); /* Notify the core layer.                               */
	}
	/* ----------------- WAKE UP TO POWER ----------------- */
	if (int_stat & (USBH_ATSAMX_INT_WAKEUP | USBH_ATSAMX_INT_DCONN))
	{
		LOG_DBG("Wake up to power");
		p_reg->INTFLAG =
			USBH_ATSAMX_INT_WAKEUP; /* Clear WAKEUP interrupt flag                          */
	}

	/* ---------------------- RESUME ---------------------- */
	if (int_stat & (USBH_ATSAMX_INT_WAKEUP | USBH_ATSAMX_INT_UPRSM |
					USBH_ATSAMX_INT_DNRSM))
	{
		LOG_DBG("resume\n");
		/* Clear interrupt flag                                 */
		p_reg->INTFLAG =
			(USBH_ATSAMX_INT_WAKEUP | USBH_ATSAMX_INT_UPRSM |
			 USBH_ATSAMX_INT_DNRSM);
		/* Disable interrupts                                   */
		p_reg->INTENCLR =
			(USBH_ATSAMX_INT_WAKEUP | USBH_ATSAMX_INT_UPRSM |
			 USBH_ATSAMX_INT_DNRSM);
		p_reg->INTENSET = (USBH_ATSAMX_INT_RST | USBH_ATSAMX_INT_DDISC);

		p_reg->CTRLB |=
			USBH_ATSAMX_CTRLB_SOFE; /* enable start of frames */
	}

	/* ----------- HANDLE PIPE INTERRUPT STATUS ----------- */
	pipe_stat =
		p_reg->PINTSMRY; /* Read Pipe interrupt summary                          */
	while (pipe_stat !=
		   0u)
	{ /* Check if there is a pipe to handle                   */
		pipe_nbr = CPU_CntTrailZeros(pipe_stat);
		int_stat = p_reg->HPIPE[pipe_nbr].PINTFLAG;
		int_stat &= p_reg->HPIPE[pipe_nbr].PINTENSET;
		p_urb = p_drv_data->PipeTbl[pipe_nbr].URB_Ptr;
		xfer_len = USBH_ATSAMX_GET_BYTE_CNT(
			p_drv_data->DescTbl[pipe_nbr].DescBank[0].PCKSIZE);

		if (int_stat & USBH_ATSAMX_PINT_STALL)
		{
			LOG_DBG("stall");
			p_reg->HPIPE[pipe_nbr].PSTATUSSET =
				USBH_ATSAMX_PSTATUS_PFREEZE; /* Stop transfer                          */
			p_reg->HPIPE[pipe_nbr].PINTFLAG =
				USBH_ATSAMX_PINT_STALL; /* Clear Stall interrupt flag             */
			p_urb->Err = USBH_ERR_EP_STALL;
			usbh_urb_done(
				p_urb); /* Notify the Core layer about the URB completion       */
		}

		if (int_stat & USBH_ATSAMX_PINT_PERR)
		{
			LOG_DBG("pipe error interrupt");
			p_reg->HPIPE[pipe_nbr].PSTATUSSET =
				USBH_ATSAMX_PSTATUS_PFREEZE; /* Stop transfer                          */
			p_reg->HPIPE[pipe_nbr].PINTFLAG =
				USBH_ATSAMX_PINT_PERR; /* Clear Pipe error interrupt flag        */
			p_urb->Err = USBH_ERR_HC_IO;
			usbh_urb_done(
				p_urb); /* Notify the Core layer about the URB completion       */
		}

		if (int_stat &
			USBH_ATSAMX_PINT_TXSTP)
		{ /* --------------- HANDLE SETUP PACKETS --------------- */
			LOG_DBG("handle setup packets");
			p_reg->HPIPE[pipe_nbr].PINTENCLR =
				USBH_ATSAMX_PINT_TXSTP; /* Disable Setup transfer interrupt              */
			p_reg->HPIPE[pipe_nbr].PINTFLAG =
				USBH_ATSAMX_PINT_TXSTP; /* Clear   Setup transfer flag                   */

			xfer_len = p_drv_data->PipeTbl[pipe_nbr].AppBufLen +
					   p_urb->XferLen;
			p_urb->Err = USBH_ERR_NONE;

			usbh_urb_done(
				p_urb); /* Notify the Core layer about the URB completion       */
		}

		if (int_stat & USBH_ATSAMX_PINT_TRCPT)
		{
			p_reg->HPIPE[pipe_nbr].PINTENCLR =
				USBH_ATSAMX_PINT_TRCPT; /* Disable transfer complete interrupt           */
			p_reg->HPIPE[pipe_nbr].PINTFLAG =
				USBH_ATSAMX_PINT_TRCPT; /* Clear   transfer complete flag                */

			/* Keep track of PID DATA toggle                        */
			p_urb->EP_Ptr->DataPID = USBH_ATSAMX_GET_DPID(
				p_reg->HPIPE[pipe_nbr].PSTATUS);

			if (p_urb->Token ==
				USBH_TOKEN_IN)
			{ /* ---------------- IN PACKETS HANDLER ---------------- */
				LOG_DBG("in packets handler");
				max_pkt_size =
					usbh_ep_max_pkt_size_get(p_urb->EP_Ptr);
				p_drv_data->PipeTbl[pipe_nbr].AppBufLen +=
					xfer_len;

				if ((p_drv_data->PipeTbl[pipe_nbr].AppBufLen ==
					 p_urb->UserBufLen) ||
					(xfer_len < max_pkt_size))
				{
					p_urb->Err = USBH_ERR_NONE;
					usbh_urb_done(
						p_urb); /* Notify the Core layer about the URB completion       */
				}
				else
				{
					p_urb->Err = USBH_OS_MsgQueuePut(
						&ATSAMX_URB_Proc_Q,
						(void *)p_urb);
					if (p_urb->Err != USBH_ERR_NONE)
					{
						usbh_urb_done(
							p_urb); /* Notify the Core layer about the URB completion       */
					}
				}
			}
			else
			{ /* ---------------- OUT PACKETS HANDLER --------------- */
				LOG_DBG("out packets handler");
				xfer_len =
					p_drv_data->PipeTbl[pipe_nbr].AppBufLen +
					p_urb->XferLen;

				if (xfer_len == p_urb->UserBufLen)
				{
					p_urb->Err = USBH_ERR_NONE;
					usbh_urb_done(
						p_urb); /* Notify the Core layer about the URB completion       */
				}
				else
				{
					p_urb->Err = USBH_OS_MsgQueuePut(
						&ATSAMX_URB_Proc_Q,
						(void *)p_urb);
					if (p_urb->Err != USBH_ERR_NONE)
					{
						usbh_urb_done(
							p_urb); /* Notify the Core layer about the URB completion       */
					}
				}
			}
		}

		DEF_BIT_CLR(pipe_stat, DEF_BIT(pipe_nbr));
	}
}

/*
*********************************************************************************************************
*                                     usbh_atsamx_process_urb()
*
* Description : The task handles additional EP IN/OUT transactions when needed.
*
* Argument(s) : p_arg     Pointer to the argument passed to 'usbh_atsamx_process_urb()' by
*                         'USBH_OS_TaskCreate()'.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void usbh_atsamx_process_urb(void *p_arg, void *p_arg2, void *p_arg3)
{
	USBH_HC_DRV *p_hc_drv;
	USBH_DRV_DATA *p_drv_data;
	USBH_ATSAMX_REG *p_reg;
	USBH_URB *p_urb = NULL;
	uint32_t xfer_len;
	uint8_t pipe_nbr;
	USBH_ERR p_err;
	CPU_SR_ALLOC();

	p_hc_drv = (USBH_HC_DRV *)p_arg;
	p_drv_data = (USBH_DRV_DATA *)p_hc_drv->DataPtr;
	p_reg = (USBH_ATSAMX_REG *)p_hc_drv->HC_CfgPtr->BaseAddr;

	while (DEF_TRUE)
	{
		USBH_OS_MsgQueueGet(&ATSAMX_URB_Proc_Q, 0u, &p_err,
							(void *)p_urb);
		if (p_err != USBH_ERR_NONE)
		{
			LOG_ERR("Cannot get USB URB");
		}

		pipe_nbr = usbh_atsamx_get_pipe_nbr(p_drv_data, p_urb->EP_Ptr);

		if (pipe_nbr != ATSAMX_INVALID_PIPE)
		{
			CPU_CRITICAL_ENTER();
			xfer_len = USBH_ATSAMX_GET_BYTE_CNT(
				p_drv_data->DescTbl[pipe_nbr]
					.DescBank[0]
					.PCKSIZE);

			if (p_urb->Token ==
				USBH_TOKEN_IN)
			{ /* -------------- HANDLE IN TRANSACTIONS -------------- */
				Mem_Copy(
					(void *)((uint32_t)p_urb->UserBufPtr +
							 p_urb->XferLen),
					p_urb->DMA_BufPtr, xfer_len);
				/* Check if it rx'd more data than what was expected    */
				if (xfer_len >
					p_drv_data->PipeTbl[pipe_nbr]
						.NextXferLen)
				{ /* Rx'd more data than what was expected   */
					p_urb->XferLen +=
						p_drv_data->PipeTbl[pipe_nbr]
							.NextXferLen;
					p_urb->Err = USBH_ERR_HC_IO;

					LOG_ERR("DRV: Rx'd more data than was expected.\r\n");
					usbh_urb_done(
						p_urb); /* Notify the Core layer about the URB completion       */
				}
				else
				{
					p_urb->XferLen += xfer_len;
				}
			}
			else
			{ /* ------------- HANDLE OUT TRANSACTIONS -------------- */
				xfer_len = (p_drv_data->PipeTbl[pipe_nbr]
								.AppBufLen -
							xfer_len);
				if (xfer_len == 0u)
				{
					p_urb->XferLen +=
						p_drv_data->PipeTbl[pipe_nbr]
							.NextXferLen;
				}
				else
				{
					p_urb->XferLen +=
						(p_drv_data->PipeTbl[pipe_nbr]
							 .NextXferLen -
						 xfer_len);
				}
			}

			if (p_urb->Err == USBH_ERR_NONE)
			{
				usbh_atsamx_pipe_cfg(
					p_urb, &p_reg->HPIPE[pipe_nbr],
					&p_drv_data->PipeTbl[pipe_nbr],
					&p_drv_data->DescTbl[pipe_nbr]
						 .DescBank[0]);

				p_reg->HPIPE[pipe_nbr].PINTENSET =
					USBH_ATSAMX_PINT_TRCPT; /* Enable transfer complete interrupt */
				p_reg->HPIPE[pipe_nbr].PSTATUSCLR =
					USBH_ATSAMX_PSTATUS_PFREEZE; /* Start  transfer                    */
			}
			CPU_CRITICAL_EXIT();
		}
	}
}

/*
*********************************************************************************************************
*                                        usbh_atsamx_pipe_cfg()
*
* Description : Initialize pipe configurations based on the endpoint direction and characteristics.
*
* Argument(s) : p_reg           Pointer to ATSAMX Pipe register structure.
*
*               p_urb           Pointer to URB structure.
*
*               p_pipe_info     Pointer to pipe information.
*
*               p_desc_bank     Pointer to Host pipe descriptor ban.
*
* Return(s)   : None.
*
* Note(s)     : None
*********************************************************************************************************
*/

static void usbh_atsamx_pipe_cfg(USBH_URB *p_urb,
								USBH_ATSAMX_PIPE_REG *p_reg_hpipe,
								USBH_ATSAMX_PINFO *p_pipe_info,
								USBH_ATSAMX_DESC_BANK *p_desc_bank)
{
	uint8_t ep_nbr;
	uint8_t reg_val;
	uint16_t max_pkt_size;
	CPU_SR_ALLOC();

	max_pkt_size = usbh_ep_max_pkt_size_get(p_urb->EP_Ptr);
	ep_nbr = usbh_ep_log_nbr_get(p_urb->EP_Ptr);
	p_pipe_info->NextXferLen = p_urb->UserBufLen - p_urb->XferLen;
	p_pipe_info->NextXferLen =
		DEF_MIN(p_pipe_info->NextXferLen, USBH_DATA_BUF_MAX_LEN);

	Mem_Clr(p_urb->DMA_BufPtr, p_pipe_info->NextXferLen);
	if (p_urb->Token !=
		USBH_TOKEN_IN)
	{ /* ---------------- SETUP/OUT PACKETS ----------------- */
		p_pipe_info->AppBufLen = p_pipe_info->NextXferLen;

		Mem_Copy(p_urb->DMA_BufPtr,
				 (void *)((uint32_t)p_urb->UserBufPtr +
						  p_urb->XferLen),
				 p_pipe_info->NextXferLen);

		p_desc_bank->PCKSIZE = p_pipe_info->AppBufLen;
	}
	else
	{ /* -------------------- IN PACKETS -------------------- */
		p_desc_bank->PCKSIZE = (p_pipe_info->NextXferLen << 14u);
	}

	p_desc_bank->ADDR = (uint32_t)p_urb->DMA_BufPtr;
	p_desc_bank->PCKSIZE |= (CPU_CntTrailZeros(max_pkt_size >> 3u) << 28u);
	p_desc_bank->CTRL_PIPE = (p_urb->EP_Ptr->DevAddr | (ep_nbr << 8u));

	if (p_urb->Token !=
		USBH_TOKEN_IN)
	{ /* ---------------- SETUP/OUT PACKETS ----------------- */
		LOG_DBG("setup out packets");
		CPU_CRITICAL_ENTER();
		reg_val = p_reg_hpipe->PCFG;
		reg_val &= ~USBH_ATSAMX_PCFG_PTOKEN_MSK;
		reg_val |= (p_urb->Token ? USBH_ATSAMX_PCFG_PTOKEN_OUT : USBH_ATSAMX_PCFG_PTOKEN_SETUP);
		p_reg_hpipe->PCFG = reg_val;
		CPU_CRITICAL_EXIT();

		p_reg_hpipe->PSTATUSSET =
			USBH_ATSAMX_PSTATUS_BK0RDY; /* Set Bank0 ready : Pipe can send data to device       */
	}
	else
	{ /* -------------------- IN PACKETS -------------------- */
		LOG_DBG("setup in packets");
		CPU_CRITICAL_ENTER();
		reg_val = p_reg_hpipe->PCFG;
		reg_val &= ~USBH_ATSAMX_PCFG_PTOKEN_MSK;
		reg_val |= USBH_ATSAMX_PCFG_PTOKEN_IN;
		p_reg_hpipe->PCFG = reg_val;
		CPU_CRITICAL_EXIT();

		p_reg_hpipe->PSTATUSCLR =
			USBH_ATSAMX_PSTATUS_BK0RDY; /* Clear Bank0 ready: Pipe can receive data from device */
	}
}

/*
*********************************************************************************************************
*                                      usbh_atsamx_get_free_pipe()
*
* Description : Allocate a free host pipe number for the newly opened pipe.
*
* Argument(s) : p_drv_data     Pointer to host driver data structure.
*
* Return(s)   : Free pipe number or 0xFF if no host pipe is found
*
* Note(s)     : None.
*********************************************************************************************************
*/

static uint8_t usbh_atsamx_get_free_pipe(USBH_DRV_DATA *p_drv_data)
{
	uint8_t pipe_nbr;

	for (pipe_nbr = 0u; pipe_nbr < ATSAMX_MAX_NBR_PIPE; pipe_nbr++)
	{
		if (DEF_BIT_IS_CLR(p_drv_data->PipeUsed, DEF_BIT(pipe_nbr)))
		{
			DEF_BIT_SET(p_drv_data->PipeUsed, DEF_BIT(pipe_nbr));
			return (pipe_nbr);
		}
	}
	return ATSAMX_INVALID_PIPE;
}

/*
*********************************************************************************************************
*                                      usbh_atsamx_get_pipe_nbr()
*
* Description : Get the host pipe number corresponding to a given device address, endpoint number
*               and direction.
*
* Argument(s) : p_drv_data     Pointer to host driver data structure.
*
*               p_ep           Pointer to endpoint structure.
*
* Return(s)   : Host pipe number or 0xFF if no host pipe is found
*
* Note(s)     : None.
*********************************************************************************************************
*/

static uint8_t usbh_atsamx_get_pipe_nbr(USBH_DRV_DATA *p_drv_data,
										 USBH_EP *p_ep)
{
	uint8_t pipe_nbr;
	uint16_t ep_addr;

	ep_addr = ((p_ep->DevAddr << 8u) | p_ep->Desc.bEndpointAddress);

	for (pipe_nbr = 0u; pipe_nbr < ATSAMX_MAX_NBR_PIPE; pipe_nbr++)
	{
		if (p_drv_data->PipeTbl[pipe_nbr].EP_Addr == ep_addr)
		{
			return (pipe_nbr);
		}
	}
	return ATSAMX_INVALID_PIPE;
}