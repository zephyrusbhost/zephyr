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
 *                                       USB HOST OS OPERATIONS
 *
 * Filename : usb_os.h
 * Version  : V3.42.00
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                               MODULE
 *********************************************************************************************************
 */

#ifndef USBH_OS_H
#define USBH_OS_H

/*
 *********************************************************************************************************
 *                                            INCLUDE FILES
 *********************************************************************************************************
 */

#include <usbh_cpu.h>
#include "usbh_err.h"
#include <zephyr.h>
#include <usbh_lib_mem.h>

/*
 *********************************************************************************************************
 *                                               EXTERNS
 *********************************************************************************************************
 */

#ifdef USBH_OS_MODULE
#define USBH_OS_EXT
#else
#define USBH_OS_EXT extern
#endif

/*
 *********************************************************************************************************
 *                                               DEFINES
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                             DATA TYPES
 *********************************************************************************************************
 */

typedef k_thread_entry_t USBH_TASK_FNCT;        /* Task function.                                       */

/*
 *********************************************************************************************************
 *                                          GLOBAL VARIABLES
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                               MACRO'S
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                         FUNCTION PROTOTYPES
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_LayerInit(void);
/* --------------- DELAY TASK FUNCTIONS --------------- */
void USBH_OS_DlyMS(uint32_t dly);

void USBH_OS_DlyUS(uint32_t dly);

/* ----------------- MUTEX FUNCTIONS ------------------ */
USBH_ERR USBH_OS_MutexCreate(struct k_mutex *p_mutex);

USBH_ERR USBH_OS_MutexLock(struct k_mutex mutex);

USBH_ERR USBH_OS_MutexUnlock(struct k_mutex mutex);

USBH_ERR USBH_OS_MutexDestroy(struct k_mutex mutex);

/* --------------- SEMAPHORE FUNCTIONS ---------------- */
USBH_ERR USBH_OS_SemCreate(struct k_sem *p_sem,
			   uint32_t cnt);

USBH_ERR USBH_OS_SemWait(struct k_sem *sem,
			 uint32_t timeout);

USBH_ERR USBH_OS_SemWaitAbort(struct k_sem *sem);

USBH_ERR USBH_OS_SemPost(struct k_sem *sem);

USBH_ERR USBH_OS_SemDestroy(struct k_sem sem);

/* ------------------ TASK FUNCTIONS ------------------ */
USBH_ERR USBH_OS_TaskCreate(char *p_name,
			    uint32_t prio,
			    USBH_TASK_FNCT task_fnct,
			    void *p_data,
			    uint32_t *p_stk,
			    uint32_t stk_size,
			    struct k_thread *p_task);

/* --------------- MSG QUEUE FUNCTIONS ---------------- */
struct k_msgq USBH_OS_MsgQueueCreate(void **p_start,
				   uint16_t size,
				   USBH_ERR *p_err);

USBH_ERR USBH_OS_MsgQueuePut(struct k_msgq *msg_q,
			     void *p_msg);

void USBH_OS_MsgQueueGet(struct k_msgq *msg_q,
			 uint32_t timeout,
			 USBH_ERR *p_err,
			 void *p_data);

/* ----------- INTERNAL USE TIMER FUNCTIONS ----------- */
struct k_timer USBH_OS_TmrCreate(char *p_name,
			    uint32_t interval_ms,
			    void (*p_callback)(void *p_tmr, void *p_arg),
			    void *p_callback_arg,
			    USBH_ERR *p_err);

USBH_ERR USBH_OS_TmrStart(struct k_timer tmr);

USBH_ERR USBH_OS_TmrDel(struct k_timer tmr);

/* ------------------- MISCELLANEOUS ------------------ */
void *USBH_OS_VirToBus(void *x);

void *USBH_OS_BusToVir(void *x);

/*
 *********************************************************************************************************
 *                                        CONFIGURATION ERRORS
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                             MODULE END
 *********************************************************************************************************
 */
#endif
