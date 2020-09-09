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
 *                                        USB HOST HUB OPERATIONS
 *
 * Filename : usbh_hub.h
 * Version  : V3.42.00
 *********************************************************************************************************
 */


#ifndef  USBH_HUB_CLASS
#define  USBH_HUB_CLASS

#include  <usbh_core.h>

extern const struct usbh_class_drv USBH_HUB_Drv;


int    usbh_hub_port_en(struct usbh_hub_dev   *p_hub_dev,
			uint16_t port_nbr);

int    usbh_hub_port_dis(struct usbh_hub_dev   *p_hub_dev,
			 uint16_t port_nbr);

int    usbh_hub_port_suspend_set(struct usbh_hub_dev   *p_hub_dev,
				 uint16_t port_nbr);

void        usbh_hub_class_notify(void           *p_class_dev,
				  uint8_t state,
				  void           *p_ctx);

uint32_t  usbh_rh_ctrl_req(struct usbh_hc        *p_hc,
			   uint8_t b_req,
			   uint8_t bm_req_type,
			   uint16_t w_val,
			   uint16_t w_ix,
			   void           *p_buf,
			   uint32_t buf_len,
			   int       *p_err);

void        usbh_rh_event(struct usbh_dev       *p_dev);

void        usbh_hub_parse_hub_desc(struct usbh_hub_desc  *p_hub_desc,
				    void           *p_buf_src);

void        usbh_hub_fmt_hub_desc(struct usbh_hub_desc  *p_hub_desc,
				  void           *p_buf_dest);

void        usbh_hub_event_task(void           *p_arg, void *p_arg2, void *p_arg3);


#endif
