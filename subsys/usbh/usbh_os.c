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
 *                                   USB HOST OPERATING SYSTEM LAYER
 *                                              TEMPLATE
 *
 * Filename : usbh_os.c
 * Version  : V3.42.00
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                            INCLUDE FILES
 *********************************************************************************************************
 */

#define USBH_OS_MODULE
#define MICRIUM_SOURCE

#include "usbh_os.h"
#include <usbh_cfg.h>
#include <zephyr.h>
#include <usbh_lib_mem.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(os_layer);
/*
 *********************************************************************************************************
 *                                            LOCAL DEFINES
 *********************************************************************************************************
 */

#define USBH_OS_MUTEX_REQUIRED						      \
	((((USBH_CFG_MAX_NBR_EPS * USBH_CFG_MAX_NBR_IFS) + 1u) *	      \
	  USBH_CFG_MAX_NBR_DEVS) +					      \
	 USBH_CFG_MAX_NBR_DEVS + USBH_CFG_MAX_NBR_HC + USBH_CDC_CFG_MAX_DEV + \
	 USBH_HID_CFG_MAX_DEV + USBH_MSC_CFG_MAX_DEV)
#define USBH_OS_SEM_REQUIRED					      \
	(3u + (((USBH_CFG_MAX_NBR_EPS * USBH_CFG_MAX_NBR_IFS) + 1u) * \
	       USBH_CFG_MAX_NBR_DEVS))
#define USBH_OS_TCB_REQUIRED 4u
#define USBH_OS_Q_REQUIRED 1u

/*
 *********************************************************************************************************
 *                                           LOCAL CONSTANTS
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                          LOCAL DATA TYPES
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                            LOCAL TABLES
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                       LOCAL GLOBAL VARIABLES
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                      LOCAL FUNCTION PROTOTYPES
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                         USBH_OS_LayerInit()
 *
 * Description : Initialize OS layer.
 *
 * Argument(s) : none.
 *
 * Return(s)   : USBH_ERR_NONE, if successful.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_LayerInit(void)
{
	return (USBH_ERR_NONE);
}

/*
 **************************************************************************************************************
 *                                       USBH_OS_VirToBus()
 *
 * Description : Convert from virtual address to physical address if the operating system uses
 *               virtual memory
 *
 * Arguments   : x       Virtual address to convert
 *
 * Returns     : The corresponding physical address
 *
 * Note(s)     : None.
 **************************************************************************************************************
 */

void *USBH_OS_VirToBus(void *x)
{
	return (x);
}

/*
 **************************************************************************************************************
 *                                       USBH_OS_BusToVir()
 *
 * Description : Convert from physical address to virtual address if the operating system uses
 *               virtual memory
 *
 * Arguments   : x       Physical address to convert
 *
 * Returns     : The corresponding virtual address
 *
 * Note(s)     : None.
 **************************************************************************************************************
 */

void *USBH_OS_BusToVir(void *x)
{
	return (x);
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                           DELAY FUNCTIONS
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                           USBH_OS_DlyMS()
 *
 * Description : Delay the current task by specified delay.
 *
 * Argument(s) : dly       Delay, in milliseconds.
 *
 * Return(s)   : none.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

void USBH_OS_DlyMS(uint32_t dly)
{
	k_sleep(K_MSEC(dly));
}

/*
 *********************************************************************************************************
 *                                           USBH_OS_DlyUS()
 *
 * Description : Delay the current task by specified delay.
 *
 * Argument(s) : dly       Delay, in microseconds.
 *
 * Return(s)   : none.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

void USBH_OS_DlyUS(uint32_t dly)
{
	k_sleep(K_USEC(dly));
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                           MUTEX FUNCTIONS
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                        USBH_OS_MutexCreate()
 *
 * Description : Create a mutex.
 *
 * Argument(s) : p_mutex    Pointer to variable that will receive handle of the mutex.
 *
 * Return(s)   : USBH_ERR_NONE,              if successful.
 *               USBH_ERR_ALLOC,
 *               USBH_ERR_OS_SIGNAL_CREATE,
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_MutexCreate(struct k_mutex *p_mutex)
{
	k_mutex_init(p_mutex);

	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *                                         USBH_OS_MutexLock()
 *
 * Description : Acquire a mutex.
 *
 * Argument(s) : mutex     Mutex handle.
 *
 * Return(s)   : USBH_ERR_NONE,          if successful.
 *               USBH_ERR_OS_TIMEOUT,    if timeout.
 *               USBH_ERR_OS_FAIL,       otherwise.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_MutexLock(struct k_mutex mutex)
{
	int err = k_mutex_lock(&mutex, K_NO_WAIT);

	if (err != 0) {
		return USBH_ERR_OS_FAIL;
	}
	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *                                        USBH_OS_MutexUnlock()
 *
 * Description : Releases a mutex.
 *
 * Argument(s) : mutex     Mutex handle.
 *
 * Return(s)   : USBH_ERR_NONE,      if successful.
 *               USBH_ERR_OS_FAIL,   otherwise.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_MutexUnlock(struct k_mutex mutex)
{
	int err = k_mutex_unlock(&mutex);

	if (err != 0) {
		return USBH_ERR_OS_FAIL;
	}
	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *                                        USBH_OS_MutexDestroy()
 *
 * Description : Destroys a mutex only when no task is pending on the mutex.
 *
 * Argument(s) : mutex     Mutex handle.
 *
 * Return(s)   : USBH_ERR_NONE,      if successful.
 *               USBH_ERR_OS_FAIL,   if mutex delete failed.
 *               USBH_ERR_FREE,      if mutex free failed.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_MutexDestroy(struct k_mutex mutex)
{
	(void)mutex;

	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                         SEMAPHORE FUNCTIONS
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                         USBH_OS_SemCreate()
 *
 * Description : Create semaphore with given count.
 *
 * Argument(s) : p_sem      Pointer that will receive handle for managing semaphore.

 *               cnt        Value with which the semaphore will be initialized.
 *
 * Return(s)   : USBH_ERR_NONE,              if successful.
 *               USBH_ERR_OS_SIGNAL_CREATE,
 *               USBH_ERR_ALLOC,
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_SemCreate(struct k_sem *p_sem, uint32_t cnt)
{
	int err = k_sem_init(p_sem, cnt, USBH_OS_SEM_REQUIRED);

	if (err != 0) {
		return USBH_ERR_OS_FAIL;
	}

	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *                                         USBH_OS_SemDestroy()
 *
 * Description : Destroy a semphore if no tasks are waiting on it.
 *
 * Argument(s) : sem       Semaphore handle.
 *
 * Return(s)   : none.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_SemDestroy(struct k_sem sem)
{
	k_sem_reset(&sem);

	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *                                          USBH_OS_SemWait()
 *
 * Description : Wait on a semaphore to become available.
 *
 * Argument(s) : sem       Semaphore handle.
 *
 *               timeout   Timeout.
 *
 * Return(s)   : USBH_ERR_NONE,          if successful.
 *               USBH_ERR_OS_TIMEOUT,    if timeout error.
 *               USBH_ERR_OS_ABORT,
 *               USBH_ERR_OS_FAIL,
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_SemWait(struct k_sem *sem, uint32_t timeout)
{
	int err = 0;
	int ret = USBH_ERR_NONE;

	if (timeout == 0) {
		err = k_sem_take(sem, K_FOREVER);
	} else   {
		err = k_sem_take(sem, K_MSEC(timeout));
	}

	if (err == EAGAIN) {
		ret = USBH_ERR_OS_TIMEOUT;
	} else if (err != 0) {
		ret = USBH_ERR_OS_FAIL;
	}
	return ret;
}

/*
 *********************************************************************************************************
 *                                       USBH_OS_SemWaitAbort()
 *
 * Description : Resume all tasks waiting on specified semaphore.
 *
 * Argument(s) : sem       Semaphore handle.
 *
 * Return(s)   : USBH_ERR_NONE,          if successful.
 *               USBH_ERR_OS_TIMEOUT,    if timeout error.
 *               USBH_ERR_OS_ABORT,
 *               USBH_ERR_OS_FAIL,
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_SemWaitAbort(struct k_sem *sem)
{
	k_sem_reset(sem);
	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *                                          USBH_OS_SemPost()
 *
 * Description : Post a semaphore.
 *
 * Argument(s) : sem       Semaphore handle.
 *
 * Return(s)   : USBH_ERR_NONE,      if successful.
 *               USBH_ERR_OS_FAIL,   otherwise.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_SemPost(struct k_sem *sem)
{
	k_sem_give(sem);
	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                          THREAD FUNCTIONS
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                        USBH_OS_ThreadCreate()
 *
 * Description : Create a thread (or task).
 *
 * Argument(s) : p_name          Pointer to name to assign to thread.
 *
 *               prio            Priority of the thread to be created.
 *
 *               thread_fnct     Pointer to the function that will be executed in this thread.
 *
 *               p_data          Pointer to the data that is passed to the thread function.
 *
 *               p_stk           Pointer to the beginning of the stack used by the thread.
 *
 *               stk_size        Size of the stack.
 *
 *               p_thread        Pointer that will receive handle for managing thread.
 *
 * Return(s)   : USBH_ERR_NONE,               if successful.
 *               USBH_ERR_ALLOC,
 *               USBH_ERR_OS_TASK_CREATE,
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_TaskCreate(char *p_name, uint32_t prio,
			    USBH_TASK_FNCT task_fnct, void *p_data,
			    uint32_t *p_stk, uint32_t stk_size,
			    struct k_thread *p_task)
{
	return (USBH_ERR_NONE);
}

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                            MESSAGE QUEUE
 *********************************************************************************************************
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                       USBH_OS_MsgQueueCreate()
 *
 * Description : Create a message queue.
 *
 * Argument(s) : p_start         Pointer to the base address of the message queue storage area.
 *
 *               size            Number of elements in the storage area.
 *
 *               p_err           Pointer to variable that will receive the return error code from this function:
 *
 *                                   USBH_ERR_NONE               Message queue created.
 *                                   USBH_ERR_ALLOC              Message queue creation failed.
 *                                   USBH_ERR_OS_SIGNAL_CREATE
 *
 * Return(s)   : The handle of the message queue.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

// USBH_HQUEUE USBH_OS_MsgQueueCreate(void **p_start, CPU_INT16U size,
// 								   USBH_ERR *p_err)
// {
// 	(void)p_start;
// 	(void)size;

// 	*p_err = USBH_ERR_NONE;

// 	return ((USBH_HQUEUE)0);
// }

/*
 *********************************************************************************************************
 *                                        USBH_OS_MsgQueuePut()
 *
 * Description : Post a message to a message queue.
 *
 * Argument(s) : msg_q     Message queue handle.
 *
 *               p_msg     Pointer to the message to send.
 *
 * Return(s)   : USBH_ERR_NONE,      if successful.
 *               USBH_ERR_OS_FAIL,   otherwise.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

USBH_ERR USBH_OS_MsgQueuePut(struct k_msgq *msg_q, void *p_msg)
{
	int err = k_msgq_put(msg_q, p_msg, K_NO_WAIT);

	if (err == 0) {
		err = USBH_ERR_NONE;
	} else   {
		err = USBH_ERR_OS_FAIL;
	}

	return err;
}

/*
 *********************************************************************************************************
 *                                        USBH_OS_MsgQueueGet()
 *
 * Description : Get a message from a message queue and blocks forever until a message is posted in the
 *               queue.
 *
 * Argument(s) : msg_q     Message queue handle.
 *
 *               timeout   Time to wait for a message, in milliseconds.
 *
 *               p_err     Pointer to variable that will receive the return error code from this function :
 *
 *                         USBH_ERR_NONE,           if successful.
 *                         USBH_ERR_OS_TIMEOUT,
 *                         USBH_ERR_OS_ABORT,
 *                         USBH_ERR_OS_FAIL,
 *
 * Return(s)   : Pointer to message, if successful.
 *               Pointer to null,    otherwise.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

void USBH_OS_MsgQueueGet(struct k_msgq *msg_q, uint32_t timeout,
			 USBH_ERR *p_err, void *p_data)
{
	int err = 0;

	if (timeout == 0) {
		err = k_msgq_get(msg_q, p_data, K_FOREVER);
	} else   {
		err = k_msgq_get(msg_q, p_data, K_MSEC(timeout));
	}
	if (err == 0) {
		*p_err = USBH_ERR_NONE;
	} else if (err == EAGAIN || err == ENOMSG) {
		*p_err = USBH_ERR_OS_TIMEOUT;
	} else   {
		*p_err = USBH_ERR_OS_FAIL;
	}
}

// /*
// *********************************************************************************************************
// *                                         CPU_CntTrailZeros()
// *
// * Description : Count the number of contiguous, least-significant, trailing zero bits in a data value.
// *
// * Argument(s) : val         Data value to count trailing zero bits.
// *
// * Return(s)   : Number of contiguous, least-significant, trailing zero bits in 'val'.
// *
// * Note(s)     : (1) (a) Supports the following data value sizes :
// *
// *                       (1)  8-bits
// *                       (2) 16-bits
// *                       (3) 32-bits
// *                       (4) 64-bits
// *
// *                       See also 'cpu_def.h  CPU WORD CONFIGURATION  Note #1'.
// *
// *                   (b) (1) For  8-bit values :
// *
// *                                  b07  b06  b05  b04  b03  b02  b01  b00    # Trailing Zeros
// *                                  ---  ---  ---  ---  ---  ---  ---  ---    ----------------
// *                                   x    x    x    x    x    x    x    1            0
// *                                   x    x    x    x    x    x    1    0            1
// *                                   x    x    x    x    x    1    0    0            2
// *                                   x    x    x    x    1    0    0    0            3
// *                                   x    x    x    1    0    0    0    0            4
// *                                   x    x    1    0    0    0    0    0            5
// *                                   x    1    0    0    0    0    0    0            6
// *                                   1    0    0    0    0    0    0    0            7
// *                                   0    0    0    0    0    0    0    0            8
// *
// *
// *                       (2) For 16-bit values :
// *
// *                             b15  b14  b13  b12  b11  ...  b02  b01  b00    # Trailing Zeros
// *                             ---  ---  ---  ---  ---       ---  ---  ---    ----------------
// *                              x    x    x    x    x         x    x    1            0
// *                              x    x    x    x    x         x    1    0            1
// *                              x    x    x    x    x         1    0    0            2
// *                              :    :    :    :    :         :    :    :            :
// *                              :    :    :    :    :         :    :    :            :
// *                              x    x    x    x    1         0    0    0           11
// *                              x    x    x    1    0         0    0    0           12
// *                              x    x    1    0    0         0    0    0           13
// *                              x    1    0    0    0         0    0    0           14
// *                              1    0    0    0    0         0    0    0           15
// *                              0    0    0    0    0         0    0    0           16
// *
// *
// *                       (3) For 32-bit values :
// *
// *                             b31  b30  b29  b28  b27  ...  b02  b01  b00    # Trailing Zeros
// *                             ---  ---  ---  ---  ---       ---  ---  ---    ----------------
// *                              x    x    x    x    x         x    x    1            0
// *                              x    x    x    x    x         x    1    0            1
// *                              x    x    x    x    x         1    0    0            2
// *                              :    :    :    :    :         :    :    :            :
// *                              :    :    :    :    :         :    :    :            :
// *                              x    x    x    x    1         0    0    0           27
// *                              x    x    x    1    0         0    0    0           28
// *                              x    x    1    0    0         0    0    0           29
// *                              x    1    0    0    0         0    0    0           30
// *                              1    0    0    0    0         0    0    0           31
// *                              0    0    0    0    0         0    0    0           32
// *
// *
// *                       (4) For 64-bit values :
// *
// *                             b63  b62  b61  b60  b59  ...  b02  b01  b00    # Trailing Zeros
// *                             ---  ---  ---  ---  ---       ---  ---  ---    ----------------
// *                              x    x    x    x    x         x    x    1            0
// *                              x    x    x    x    x         x    1    0            1
// *                              x    x    x    x    x         1    0    0            2
// *                              :    :    :    :    :         :    :    :            :
// *                              :    :    :    :    :         :    :    :            :
// *                              x    x    x    x    1         0    0    0           59
// *                              x    x    x    1    0         0    0    0           60
// *                              x    x    1    0    0         0    0    0           61
// *                              x    1    0    0    0         0    0    0           62
// *                              1    0    0    0    0         0    0    0           63
// *                              0    0    0    0    0         0    0    0           64
// *
// *               (2) For non-zero values, the returned number of contiguous, least-significant, trailing
// *                   zero bits is also equivalent to the bit position of the least-significant set bit.
// *
// *               (3) 'val' SHOULD be validated for non-'0' PRIOR to all other counting zero calculations :
// *
// *                   (a) CPU_CntTrailZeros()'s final conditional statement calculates 'val's number of
// *                       trailing zeros based on its return data size, 'CPU_CFG_DATA_SIZE', & 'val's
// *                       calculated number of lead zeros ONLY if the initial 'val' is non-'0' :
// *
// *                           if (val != 0u) {
// *                               nbr_trail_zeros = ((CPU_CFG_DATA_SIZE * DEF_OCTET_NBR_BITS) - 1u) - nbr_lead_zeros;
// *                           } else {
// *                               nbr_trail_zeros = nbr_lead_zeros;
// *                           }
// *
// *                       Therefore, initially validating all non-'0' values avoids having to conditionally
// *                       execute the final 'if' statement.
// *********************************************************************************************************
// */

// CPU_DATA CPU_CntTrailZeros(CPU_DATA val)
// {
// 	CPU_DATA val_bit_mask;
// 	CPU_DATA nbr_lead_zeros;
// 	CPU_DATA nbr_trail_zeros;

// 	if (val == 0u)
// 	{ /* Rtn ALL val bits as zero'd (see Note #3).            */
// 		return ((CPU_DATA)(CPU_CFG_DATA_SIZE * DEF_OCTET_NBR_BITS));
// 	}

// 	val_bit_mask = val & ((CPU_DATA)~val + 1u);		 /* Zero/clr all bits EXCEPT least-sig set bit.          */
// 	nbr_lead_zeros = CPU_CntLeadZeros(val_bit_mask); /* Cnt  nbr lead  0s.                                   */
// 													 /* Calc nbr trail 0s = (nbr val bits - 1) - nbr lead 0s.*/
// 	nbr_trail_zeros = ((CPU_DATA)((CPU_CFG_DATA_SIZE * DEF_OCTET_NBR_BITS) - 1u) - nbr_lead_zeros);

// 	return (nbr_trail_zeros);
// }


