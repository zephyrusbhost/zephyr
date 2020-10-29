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
*                        USB HOST DRIVER BOARD SUPPORT PACKAGE (BSP) FUNCTIONS
*
*                                              TEMPLATE
*
* Filename : bsp_usbh_template.c
* Version  : V3.42.00
*********************************************************************************************************
*/

/*
**************************************************************************************************************
*                                            INCLUDE FILES
**************************************************************************************************************
*/

#include "bsp_usbh_template.h"
#include <usbh_cpu.h>

#include <soc.h>
#include <drivers/pinmux.h>

#include <usbh_cfg.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(bsp);
/*
**************************************************************************************************************
*                                            LOCAL DEFINES
**************************************************************************************************************
*/
#define CKGR_UCKR_UPLLEN (0x1u << 16) /**< \brief (CKGR_UCKR) UTMI PLL Enable */

/*
**************************************************************************************************************
*                                           LOCAL CONSTANTS
**************************************************************************************************************
*/

/*
**************************************************************************************************************
*                                          LOCAL DATA TYPES
**************************************************************************************************************
*/

/*
**************************************************************************************************************
*                                            LOCAL TABLES
**************************************************************************************************************
*/

/*
**************************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
**************************************************************************************************************
*/

static USBH_HC_DRV *USBH_HC_Template_DrvPtr;

static CPU_FNCT_PTR BSP_USBH_Template_ISR_Ptr;

/*
**************************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
**************************************************************************************************************
*/

static void BSP_USBH_samr21_Init(USBH_HC_DRV *p_drv, USBH_ERR *p_err);

static void BSP_USBH_samr21_ISR_Register(CPU_FNCT_PTR isr_fnct,
										   USBH_ERR *p_err);

static void BSP_USBH_samr21_ISR_Unregister(USBH_ERR *p_err);

// static void BSP_USBH_samr21_IntHandler(void);

/*
*********************************************************************************************************
*                                    USB HOST DRIVER BSP INTERFACE
*********************************************************************************************************
*/

USBH_HC_BSP_API USBH_DrvBSP_Template = {BSP_USBH_samr21_Init,
										BSP_USBH_samr21_ISR_Register,
										BSP_USBH_samr21_ISR_Unregister};

/*
**************************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
**************************************************************************************************************
*/

/*
**************************************************************************************************************
**************************************************************************************************************
*                                           LOCAL FUNCTION
**************************************************************************************************************
**************************************************************************************************************
*/

/*
*********************************************************************************************************
*                                      BSP_USBH_Template_Init()
*
* Description : This function performs board specific initialization of USB host controller.
*
* Argument(s) : p_drv    Pointer to host controller driver structure.
*
*               p_err    Pointer to variable that will receive the return error code from this function
*
*                            USBH_ERR_NONE    BSP init successfull.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/
static void BSP_USBH_samr21_Init(USBH_HC_DRV *p_drv, USBH_ERR *p_err)
{
	LOG_INF("bsp init");
	// USBH_HC_Template_DrvPtr = p_drv;


	// /* Enable the clock in PM */
	// PM->AHBMASK.bit.USB_ = 1;
	// PM->APBBMASK.bit.USB_ = 1;

	// /* Enable the GCLK */
	// GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_USB | GCLK_CLKCTRL_GEN_GCLK0 |
	// 					GCLK_CLKCTRL_CLKEN;

	// while (GCLK->STATUS.bit.SYNCBUSY)
	// {
	// }
	// struct device *muxa = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_a)));
	// pinmux_pin_set(muxa, 25, PINMUX_FUNC_G);
	// pinmux_pin_set(muxa, 24, PINMUX_FUNC_G);
	// pinmux_pin_set(muxa, 23, PINMUX_FUNC_G);

	// /* $$$$ This function performs all operations that the host controller cannot do. Typical operations are: */

	// /* $$$$ Enable host control registers and bus clock [mandatory]. */
	// /* $$$$ Configure main USB host interrupt(s) in interrupt controller (e.g. registering BSP ISR) [mandatory]. */
	// /* $$$$ Configure I/O pins [if necessary]. */
	// *p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                  BSP_USBH_Template_ISR_Register()
*
* Description : Registers Interrupt Service Routine.
*
* Argument(s) : isr_fnct    Host controller ISR address.
*
*               p_err       Pointer to variable that will receive the return error code from this function
*
*                               USBH_ERR_NONE    ISR registered successfully.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static void BSP_USBH_samr21_ISR_Register(CPU_FNCT_PTR isr_fnct,
										   USBH_ERR *p_err)
{
	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                 BSP_USBH_Template_ISR_Unregister()
*
* Description : Unregisters Interrupt Service Routine.
*
* Argument(s) : p_err    Pointer to variable that will receive the return error code from this function
*
*                            USBH_ERR_NONE    ISR unregistered successfully.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static void BSP_USBH_samr21_ISR_Unregister(USBH_ERR *p_err)
{
	BSP_USBH_Template_ISR_Ptr = (CPU_FNCT_PTR)0;

	*p_err = USBH_ERR_NONE;
}

/*
*********************************************************************************************************
*                                   BSP_USBH_Template_IntHandler()
*
* Description : USB host interrupt handler.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

// static void BSP_USBH_samr21_IntHandler(void)
// {
// 	if (BSP_USBH_Template_ISR_Ptr != (CPU_FNCT_PTR)0)
// 	{
// 		BSP_USBH_Template_ISR_Ptr((void *)USBH_HC_Template_DrvPtr);
// 	}
// }

/*
**************************************************************************************************************
*                                                  END
**************************************************************************************************************
*/
