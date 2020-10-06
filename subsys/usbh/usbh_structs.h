#ifndef ZEPHYR_USBH_STRUCTS_H_
#define ZEPHYR_USBH_STRUCTS_H_

#include <zephyr/types.h>
#include <usbh_cfg.h>

#define  USBH_MAX_NBR_DEVS                          USBH_CFG_MAX_NBR_DEVS + USBH_CFG_MAX_NBR_HC
#define  USBH_LEN_DESC_DEV                              0x12u


enum usbh_device_speed {
	USBH_UNKNOWN_SPEED = 0,                 /* enumerating */
	USBH_LOW_SPEED, USBH_FULL_SPEED,        /* usb 1.1 */
	USBH_HIGH_SPEED,                        /* usb 2.0 */
};


/*
 *********************************************************************************************************
 *                                      HUB PORT STATUS DATA TYPE
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 11.24.2.7.
 *********************************************************************************************************
 */

struct  usbh_hub_port_status {
	uint16_t w_port_status;
	uint16_t w_port_change;
};


/*
 *********************************************************************************************************
 *                                           HUB DESCRIPTOR
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 11.23.2.1.
 *********************************************************************************************************
 */
struct  usbh_hub_desc {
	uint8_t b_desc_length;
	uint8_t b_desc_type;
	uint8_t b_nbr_ports;
	uint16_t w_hub_characteristics;
	uint8_t b_pwr_on_to_pwr_good;
	uint8_t b_hub_contr_current;
	uint8_t device_removable;
	uint32_t port_pwr_ctrl_mask[USBH_CFG_MAX_HUB_PORTS];
};


/*
 *********************************************************************************************************
 *                                             HUB STATUS
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 11.24.2.6.
 *********************************************************************************************************
 */

struct  usbh_hub_status {
	uint16_t w_hub_status;
	uint16_t w_hub_change;
};


/*
 *********************************************************************************************************
   -                                            SETUP REQUEST
   -
   - Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 9.3, Table 9-2.
 *********************************************************************************************************
 */

struct  usbh_setup_req {
	uint8_t bm_request_type;
	uint8_t b_request;
	uint16_t w_value;
	uint16_t w_index;
	uint16_t w_length;
};


/*
 *********************************************************************************************************
 *                                          DESCRIPTOR HEADER
 *********************************************************************************************************
 */

struct  usbh_desc_hdr {
	uint8_t b_length;
	uint8_t b_desc_type;
};


/*
 *********************************************************************************************************
 *                                          DEVICE DESCRIPTOR
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 9.6.1, Table 9-8.
 *********************************************************************************************************
 */

struct  usbh_dev_desc {
	uint8_t b_length;
	uint8_t b_desc_type;
	uint16_t bcd_usb;
	uint8_t b_device_class;
	uint8_t b_device_sub_class;
	uint8_t b_device_protocol;
	uint8_t b_max_packet_size_zero;
	uint16_t id_vendor;
	uint16_t id_product;
	uint16_t bcd_device;
	uint8_t i_manufacturer;
	uint8_t i_product;
	uint8_t i_serial_number;
	uint8_t b_nbr_configs;
};


/*
 *********************************************************************************************************
 *                                     DEVICE QUALIFIER DESCRIPTOR
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 9.6.2, Table 9-9.
 *********************************************************************************************************
 */

struct  usbh_dev_qualifier_desc {
	uint8_t b_length;
	uint8_t b_desc_type;
	uint16_t bcd_usb;
	uint8_t b_device_class;
	uint8_t b_device_sub_class;
	uint8_t b_device_protocol;
	uint8_t b_max_packet_size_zero;
	uint8_t b_nbr_configs;
	uint8_t b_reserved;
};


/*
 *********************************************************************************************************
 *                                      CONFIGURATION DESCRIPTOR
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 9.6.3, Table 9-10.
 *********************************************************************************************************
 */

struct  usbh_cfg_desc {
	uint8_t b_length;
	uint8_t b_desc_type;
	uint16_t w_total_length;
	uint8_t b_nbr_interfaces;
	uint8_t b_cfg_value;
	uint8_t i_cfg;
	uint8_t bm_attributes;
	uint8_t b_max_pwr;
};


/*
 *********************************************************************************************************
 *                                OTHER SPEED CONFIGURATION DESCRIPTOR
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 9.6.4, Table 9-11.
 *********************************************************************************************************
 */

struct  usbh_other_spd_cfg_desc {
	uint8_t b_length;
	uint8_t b_desc_type;
	uint16_t w_total_length;
	uint8_t b_nbr_interfaces;
	uint8_t b_cfg_value;
	uint8_t i_cfg;
	uint8_t bm_attributes;
	uint8_t b_max_pwr;
};


/*
 *********************************************************************************************************
 *                                        INTERFACE DESCRIPTOR
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 9.6.5, Table 9-12.
 *********************************************************************************************************
 */

struct  usbh_if_desc {
	uint8_t b_length;
	uint8_t b_desc_type;
	uint8_t b_if_nbr;
	uint8_t b_alt_setting;
	uint8_t b_nbr_endpoints;
	uint8_t b_if_class;
	uint8_t b_if_sub_class;
	uint8_t b_if_protocol;
	uint8_t i_interface;
};


/*
 *********************************************************************************************************
 *                                  INTERFACE ASSOCIATION DESCRIPTOR
 *
 * Note(s) : (1) See 'www.usb.org/developers/doc/InterfaceAssociationDescriptor_ecn.pdf', Section 9.X.Y, Table 9-Z.
 *********************************************************************************************************
 */

struct  usbh_if_association_desc {
	uint8_t b_length;
	uint8_t b_desc_type;
	uint8_t b_first_if;
	uint8_t b_if_cnt;
	uint8_t b_fnct_class;
	uint8_t b_fnct_sub_class;
	uint8_t b_fnct_protocol;
	uint8_t i_function;
};


/*
 *********************************************************************************************************
 *                                         ENDPOINT DESCRIPTOR
 *
 * Note(s) : (1) See 'Universal Serial Bus Specification Revision 2.0', Section 9.6.6, Table 9-14.
 *********************************************************************************************************
 */

struct  usbh_ep_desc {
	uint8_t b_length;
	uint8_t b_desc_type;
	uint8_t b_endpoint_address;
	uint8_t bm_attributes;
	uint16_t w_max_packet_size;
	uint8_t b_interval;
	uint8_t b_refresh;
	uint8_t b_sync_address;
};


/*
 *********************************************************************************************************
 *                                           OTG DESCRIPTOR
 *
 * Note(s) : (1) See 'On-The-Go Specification Revision 1.3', Section 6.4, Table 6-1.
 *********************************************************************************************************
 */

struct  usbh_otg_desc {
	uint8_t b_length;
	uint8_t b_desc_type;
	uint8_t bm_attributes;
};


/*
 *********************************************************************************************************
 *                                       ISOCHRONOUS DESCRIPTOR
 *********************************************************************************************************
 */

struct  usbh_isoc_desc {
	uint8_t  *buf_ptr;
	uint32_t buf_len;
	uint32_t start_frm;
	uint32_t nbr_frm;
	uint16_t  *frm_len;
	int    *frm_err;
};


/*
 *********************************************************************************************************
 *                                 USB REQUEST BLOCK (urb) INFORMATION
 *********************************************************************************************************
 */

struct  usbh_urb {
	volatile uint8_t state;                                 /* state of urb.                                        */
	struct usbh_ep         *ep_ptr;                         /* EP the urb belongs to.                               */
	volatile int err;                                       /* The status of urb completion.                        */

	void            *userbuf_ptr;                           /* Ptr to buf supplied by app.                          */
	uint32_t uberbuf_len;                                   /* Buf len in bytes.                                    */
	void            *dma_buf_ptr;                           /* DMA buf ptr used by DMA HW.                          */
	int32_t dma_buf_len;                                    /* DMA buf len.                                         */
	uint32_t xfer_len;                                      /* Actual len xfer'd by ctrlr.                          */

	struct usbh_isoc_desc  *isoc_desc_ptr;                  /* Isoc xfer desc.                                      */

	void            *fnct_ptr;                              /* Fnct ptr, called when I/O is completed.              */
	void            *fnct_arg_ptr;                          /* Fnct context.                                        */

	void            *arg_ptr;                               /* HCD private data.                                    */

	uint8_t token;                                          /* token (SETUP, IN, or OUT).                           */

	bool urb_done_signal;
	struct usbh_urb        *async_urb_nxt_ptr;                      /* Ptr to next urb (if any).                            */
	struct usbh_urb        *nxt_ptr;                                /* Used for urb chained list in async task.             */

	struct k_sem sem;                                               /* sem to wait on I/O completion.                       */
};


/*
 *********************************************************************************************************
 *                                        ENDPOINT INFORMATION
 *********************************************************************************************************
 */

struct  usbh_ep {
	enum usbh_device_speed dev_spd;                         /* USB dev spd.                                         */
	uint8_t dev_addr;                                       /* USB dev addr.                                        */
	struct usbh_dev      *dev_ptr;                          /* Ptr to USB dev struct.                               */
	struct usbh_ep_desc desc;                               /* EP desc.                                             */
	uint16_t interval;                                      /* EP interval.                                         */
	uint32_t hc_ref_frame;                                  /* Initial HC ref frame nbr.                            */
	void          *arg_ptr;                                 /* HCD private data.                                    */
	struct usbh_urb urb;                                    /* urb used for data xfer on this endpoint.             */
	struct k_mutex mutex;                                   /* mutex for I/O access serialization on this EP.       */
	bool is_open;                                           /* EP state.                                            */
	uint32_t xfer_nbr_in_prog;                              /* nbr of urb(s) in progress. Used for async omm.       */
	uint8_t data_pid;                                       /* EP Data Toggle PID tracker.                          */
};


/*
 *********************************************************************************************************
 *                                        INTERFACE INFORMATION
 *********************************************************************************************************
 */

struct  usbh_if {
	struct usbh_dev            *dev_ptr;                            /* Ptr to USB dev.                                      */
	uint8_t alt_ix_sel;                                             /* Selected alternate setting ix.                       */
	void                *class_dev_ptr;                             /* Ptr to class dev created by class drv.               */
	struct usbh_class_drv_reg  *class_drv_reg_ptr;                  /* Ptr to class drv registered for this IF.             */
	uint8_t          *if_data_ptr;                                  /* Buf pointer containing IF data.                      */
	uint16_t if_data_len;                                           /* Buf len.                                             */
};


/*
 *********************************************************************************************************
 *                                      CONFIGURATION INFORMATION
 *********************************************************************************************************
 */

struct  usbh_cfg {
	uint8_t cfg_data[USBH_CFG_MAX_CFG_DATA_LEN];            /* Buf containing cfg desc data.                        */
	uint16_t cfg_data_len;                                  /* Cfg desc data len.                                   */
	struct usbh_if if_list[USBH_CFG_MAX_NBR_IFS];           /* Device IFs.                                          */
};


/*
 *********************************************************************************************************
 *                                         DEVICE INFORMATION
 *********************************************************************************************************
 */

struct  usbh_dev {
	struct usbh_hc             *hc_ptr;                     /* Ptr to HC struct.                                    */
	uint8_t dev_addr;                                       /* USB dev addr assigned by host.                       */
	enum usbh_device_speed dev_spd;                         /* Dev spd (low, full or high).                         */
	struct usbh_ep dflt_ep;                                 /* Dflt ctrl EP.                                        */
	struct k_mutex dflt_ep_mutex;                           /* Dev dflt EP mutex.                                   */
	uint16_t lang_id;                                       /* Language ID used by the str desc.                    */
	void                *class_dev_ptr;                     /* Ptr to class dev created by class drv.               */
	struct usbh_class_drv_reg  *class_drv_reg_ptr;          /* Ptr to class drv managing this dev.                  */
	uint8_t dev_desc[USBH_LEN_DESC_DEV];                    /* Dev desc.                                            */
	struct usbh_cfg cfg_list[USBH_CFG_MAX_NBR_CFGS];        /* Dev cfg.                                             */
	uint8_t sel_cfg;                                        /* Selected dev cfg nbr.                                */
	struct usbh_dev            *hub_dev_ptr;                /* Ptr to up stream hub dev struct.                     */
	uint32_t port_nbr;                                      /* Port nbr to which this dev is connected.             */
	bool is_root_hub;                                       /* Indicate if this is a RH dev.                        */
	struct usbh_hub_dev        *hub_hs_ptr;                 /* Ptr to prev HS Hub.                                  */
};


/*
 *********************************************************************************************************
 *                                             HUB DEVICE
 *********************************************************************************************************
 */

struct  usbh_hub_dev {
	struct usbh_ep intr_ep;                                         /* Intr EP to recv events from hub.                     */
	struct usbh_hub_desc desc;                                      /* Hub desc.                                            */
	struct usbh_dev       *dev_ptr_list[USBH_CFG_MAX_HUB_PORTS];    /* Ptrs to USB devs connected to this hub.              */
	struct usbh_dev       *dev_ptr;                                 /* USB dev ptr of the hub IF.                           */
	struct usbh_if        *if_ptr;                                  /* HUB IF ptr.                                          */
	uint8_t hub_intr_buf[64];                                       /* Buf to recv hub events.                              */
	uint32_t err_cnt;
	uint8_t state;
	uint8_t ref_cnt;
	struct usbh_hub_dev   *nxt_ptr;
	uint8_t conn_cnt;                                     /* Re-connection counter                                */
};


/*
 *********************************************************************************************************
 *                                 HOST CONTROLLER DRIVER INFORMATION
 *********************************************************************************************************
 */

struct  usbh_hc_drv {
	uint8_t nbr;                                            /* HC nbr.                                              */
	void             *data_ptr;                             /* Drv's data.                                          */
	struct usbh_dev         *rh_dev_ptr;                    /* Ptr to RH dev struct.                                */
	const struct usbh_hc_drv_api  *api_ptr;                 /* Ptr to HC drv API struct.                            */
	const struct usbh_hc_rh_api   *rh_api_ptr;              /* Ptr to RH drv API struct.                            */
	const struct usbh_hc_bsp_api  *bsp_api_ptr;             /* Ptr to HC BSP API struct.                            */
};



/*
 *********************************************************************************************************
 *                                HOST CONTROLLER INFORMATION DATA TYPE
 *********************************************************************************************************
 */

struct usbh_hc {
	struct usbh_hc_drv hc_drv;                              /* Host Controller driver (HCD) info.                   */
	struct usbh_host     *host_ptr;                         /* Host structure.                                      */
	struct usbh_hub_dev  *rh_class_dev_ptr;                 /* Root Hub class device pointer.                       */
	struct k_mutex hcd_mutex;                               /* mutex to sync access to HCD.                         */
	bool is_vir_rh;                                         /* Indicate if RH is virtual.                           */
};


/*
 *********************************************************************************************************
 *                                     HOST INFORMATION DATA TYPE
 *********************************************************************************************************
 */

struct  usbh_host {
	uint8_t state;                                  /* state of USB host stack.                             */

	struct usbh_dev dev_list[USBH_MAX_NBR_DEVS];    /* List of USB dev connected.                           */
	int8_t dev_cnt;                                 /* Pool for mem mgmt of USB devs.                       */
	int8_t isoc_cnt;
	struct usbh_isoc_desc isoc_desc[USBH_CFG_MAX_ISOC_DESC];
	struct k_mem_pool async_urb_pool;                               /* Pool of extra urb when using async comm.             */

	struct usbh_hc hc_tbl[USBH_CFG_MAX_NBR_HC];                     /* Array of HC structs.                                 */
	uint8_t hc_nbr_next;

	struct k_thread h_async_task;                                           /* Async task handle.                                   */
	struct k_thread h_hub_task;                                             /* Hub event task handle.                               */
};

#endif /*  ZEPHYR_USBH_STRUCTS_H_ */

