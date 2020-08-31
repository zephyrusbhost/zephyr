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

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#define USBH_HUB_MODULE
#define MICRIUM_SOURCE
#include "usbh_hub.h"
#include "usbh_core.h"
#include "usbh_class.h"
#include <sys/byteorder.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(hub);
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
K_MEM_POOL_DEFINE(USBH_HUB_Pool, sizeof(struct usbh_hub_dev),
		  sizeof(struct usbh_hub_dev), USBH_CFG_MAX_HUBS,
		  sizeof(uint32_t));

/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/

/*
********************************************************************************************************
*                                     ROOT HUB DEVICE DESCRIPTOR
********************************************************************************************************
*/

static const uint8_t USBH_HUB_RH_DevDesc[18] = {
    USBH_LEN_DESC_DEV,              /* bLength                                              */
    USBH_DESC_TYPE_DEV,             /* bDescriptorType: Device                              */
    0x10u, 0x01u,                   /* bcdUSB: v1.1                                         */
    USBH_CLASS_CODE_HUB,            /* bDeviceClass: HUB_CLASSCODE                          */
    USBH_SUBCLASS_CODE_USE_IF_DESC, /* bDeviceSubClass                                      */
    USBH_PROTOCOL_CODE_USE_IF_DESC, /* bDeviceProtocol                                      */
    0x40u,                          /* bMaxPacketSize0: 64 Bytes                            */
    0x00u, 0x00u,                   /* idVendor                                             */
    0x00u, 0x00u,                   /* idProduct                                            */
    0x00u, 0x00u,                   /* bcdDevice                                            */
    0x00u,                          /* iManufacturer                                        */
    0x00u,                          /* iProduct                                             */
    0x00u,                          /* iSerialNumber                                        */
    0x01u                           /* bNumConfigurations                                   */
};

/*
********************************************************************************************************
*                                  ROOT HUB CONFIGURATION DESCRIPTOR
********************************************************************************************************
*/

static const uint8_t USBH_HUB_RH_FS_CfgDesc[] = {
    /* ------------- CONFIGURATION DESCRIPTOR ------------- */
    USBH_LEN_DESC_CFG,  /* bLength                                              */
    USBH_DESC_TYPE_CFG, /* bDescriptorType CONFIGURATION                        */
    0x19u, 0x00u,       /* le16 wTotalLength                                    */
    0x01u,              /* bNumInterfaces                                       */
    0x01u,              /* bConfigurationValue                                  */
    0x00u,              /* iConfiguration                                       */
    0xC0u,              /* bmAttributes -> Self-powered|Remote wakeup           */
    0x00u,              /* bMaxPower                                            */

    /* --------------- INTERFACE DESCRIPTOR --------------- */
    USBH_LEN_DESC_IF,    /* bLength                                              */
    USBH_DESC_TYPE_IF,   /* bDescriptorType: Interface                           */
    0x00u,               /* bInterfaceNumber                                     */
    0x00u,               /* bAlternateSetting                                    */
    0x01u,               /* bNumEndpoints                                        */
    USBH_CLASS_CODE_HUB, /* bInterfaceClass HUB_CLASSCODE                        */
    0x00u,               /* bInterfaceSubClass                                   */
    0x00u,               /* bInterfaceProtocol                                   */
    0x00u,               /* iInterface                                           */

    /* --------------- ENDPOINT DESCRIPTOR ---------------- */
    USBH_LEN_DESC_EP,  /* bLength                                              */
    USBH_DESC_TYPE_EP, /* bDescriptorType: Endpoint                            */
    0x81u,             /* bEndpointAddress: IN Endpoint 1                      */
    0x03u,             /* bmAttributes Interrupt                               */
    0x08u, 0x00u,      /* wMaxPacketSize                                       */
    0x1u               /* bInterval                                            */
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

static const uint8_t USBH_HUB_RH_LangID[] = {
    0x04u,
    USBH_DESC_TYPE_STR,
    0x09u, 0x04u, /* Identifer for English (United States). See Note #1.  */
};

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

static uint8_t USBH_HUB_DescBuf[USBH_HUB_MAX_DESC_LEN];
static struct usbh_hub_dev USBH_HUB_Arr[USBH_CFG_MAX_HUBS];
static int8_t HubCount = USBH_CFG_MAX_HUBS;
static volatile struct usbh_hub_dev *USBH_HUB_HeadPtr;
static volatile struct usbh_hub_dev *USBH_HUB_TailPtr;
static struct k_sem USBH_HUB_EventSem;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void USBH_HUB_GlobalInit(USBH_ERR *p_err);

static void *USBH_HUB_IF_Probe(struct usbh_dev *p_dev,
                               struct usbh_if *p_if,
                               USBH_ERR *p_err);

static void USBH_HUB_Suspend(void *p_class_dev);

static void USBH_HUB_Resume(void *p_class_dev);

static void USBH_HUB_Disconn(void *p_class_dev);

static USBH_ERR USBH_HUB_Init(struct usbh_hub_dev *p_hub_dev);

static void USBH_HUB_Uninit(struct usbh_hub_dev *p_hub_dev);

static USBH_ERR USBH_HUB_EP_Open(struct usbh_hub_dev *p_hub_dev);

static void USBH_HUB_EP_Close(struct usbh_hub_dev *p_hub_dev);

static USBH_ERR USBH_HUB_EventReq(struct usbh_hub_dev *p_hub_dev);

static void USBH_HUB_ISR(struct usbh_ep *p_ep,
                         void *p_buf,
                         uint32_t buf_len,
                         uint32_t xfer_len,
                         void *p_arg,
                         USBH_ERR err);

static void USBH_HUB_EventProcess(void);

static USBH_ERR USBH_HUB_DescGet(struct usbh_hub_dev *p_hub_dev);

static USBH_ERR USBH_HUB_PortsInit(struct usbh_hub_dev *p_hub_dev);

static USBH_ERR USBH_HUB_PortStatusGet(struct usbh_hub_dev *p_hub_dev,
                                       uint16_t port_nbr,
                                       struct usbh_hub_port_status *p_port_status);

static USBH_ERR USBH_HUB_PortResetSet(struct usbh_hub_dev *p_hub_dev,
                                      uint16_t port_nbr);

static USBH_ERR USBH_HUB_PortRstChngClr(struct usbh_hub_dev *p_hub_dev,
                                        uint16_t port_nbr);

static USBH_ERR USBH_HUB_PortEnChngClr(struct usbh_hub_dev *p_hub_dev,
                                       uint16_t port_nbr);

static USBH_ERR USBH_HUB_PortConnChngClr(struct usbh_hub_dev *p_hub_dev,
                                         uint16_t port_nbr);

static USBH_ERR USBH_HUB_PortPwrSet(struct usbh_hub_dev *p_hub_dev,
                                    uint16_t port_nbr);

static USBH_ERR USBH_HUB_PortSuspendClr(struct usbh_hub_dev *p_hub_dev,
                                        uint16_t port_nbr);

static USBH_ERR USBH_HUB_PortEnClr(struct usbh_hub_dev *p_hub_dev,
                                   uint16_t port_nbr);

static USBH_ERR USBH_HUB_PortEnSet(struct usbh_hub_dev *p_hub_dev,
                                   uint16_t port_nbr);

static void USBH_HUB_Clr(struct usbh_hub_dev *p_hub_dev);

static USBH_ERR USBH_HUB_RefAdd(struct usbh_hub_dev *p_hub_dev);

static USBH_ERR USBH_HUB_RefRel(struct usbh_hub_dev *p_hub_dev);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                          HUB CLASS DRIVER
*********************************************************************************************************
*/

const struct usbh_class_drv USBH_HUB_Drv = {
    (uint8_t *)"HUB",
    USBH_HUB_GlobalInit,
    0,
    USBH_HUB_IF_Probe,
    USBH_HUB_Suspend,
    USBH_HUB_Resume,
    USBH_HUB_Disconn};

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
    (void)p_arg;

    while (true)
    {
        k_sem_take(&USBH_HUB_EventSem, K_FOREVER);
        USBH_HUB_EventProcess();
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
* Return(s)   : USBH_ERR_NONE,                          If port is successfully disabled.
*               USBH_ERR_INVALID_ARG,                   If invalid parameter passed to 'p_hub_dev'.
*
*                                                       ----- RETURNED BY USBH_HUB_PortEnClr() : -----
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

USBH_ERR usbh_hub_port_dis(struct usbh_hub_dev *p_hub_dev,
                           uint16_t port_nbr)
{
    USBH_ERR err;

    if (p_hub_dev == (struct usbh_hub_dev *)0)
    {
        return (USBH_ERR_INVALID_ARG);
    }

    err = USBH_HUB_PortEnClr(p_hub_dev, port_nbr);

    return (err);
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
*                                                       ----- RETURNED BY USBH_HUB_PortEnSet() : -----
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

USBH_ERR usbh_hub_port_en(struct usbh_hub_dev *p_hub_dev,
                          uint16_t port_nbr)
{
    USBH_ERR err;

    if (p_hub_dev == (struct usbh_hub_dev *)0)
    {
        return (USBH_ERR_INVALID_ARG);
    }

    err = USBH_HUB_PortEnSet(p_hub_dev, port_nbr);

    return (err);
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
*                                        USBH_HUB_GlobalInit()
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

static void USBH_HUB_GlobalInit(USBH_ERR *p_err)
{
    uint8_t hub_ix;

    for (hub_ix = 0u; hub_ix < USBH_CFG_MAX_HUBS; hub_ix++)
    { /* Clr all HUB dev structs.                             */
        USBH_HUB_Clr(&USBH_HUB_Arr[hub_ix]);
    }
    HubCount = (USBH_CFG_MAX_HUBS - 1);

    *p_err = k_sem_init(&USBH_HUB_EventSem, 0u, USBH_OS_SEM_REQUIRED);

    USBH_HUB_HeadPtr = (struct usbh_hub_dev *)0;
    USBH_HUB_TailPtr = (struct usbh_hub_dev *)0;

    memset(USBH_HUB_DescBuf, 0, USBH_HUB_MAX_DESC_LEN);
}

/*
*********************************************************************************************************
*                                         USBH_HUB_IF_Probe()
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
*                                                           ----- RETURNED BY USBH_HUB_Init() : -----
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

static void *USBH_HUB_IF_Probe(struct usbh_dev *p_dev,
                               struct usbh_if *p_if,
                               USBH_ERR *p_err)
{
    LOG_INF("hub if");
    struct usbh_hub_dev *p_hub_dev;
    struct usbh_if_desc if_desc;

    p_hub_dev = (struct usbh_hub_dev *)0;
    *p_err = usbh_if_desc_get(p_if, /* Get IF desc.                                         */
                             0u,
                             &if_desc);
    if (*p_err != USBH_ERR_NONE)
    {
        return ((void *)0);
    }

    if (if_desc.bInterfaceClass == USBH_CLASS_CODE_HUB)
    { /* If IF is HUB, alloc HUB dev.                         */
        if (HubCount < 0)
        {
            *p_err = USBH_ERR_DEV_ALLOC;
            return ((void *)0);
        }
        else
        {
            p_hub_dev = &USBH_HUB_Arr[HubCount--];
        }

        USBH_HUB_Clr(p_hub_dev);
        USBH_HUB_RefAdd(p_hub_dev);

        p_hub_dev->State = (uint8_t)USBH_CLASS_DEV_STATE_CONN;
        p_hub_dev->DevPtr = p_dev;
        p_hub_dev->IF_Ptr = p_if;
        p_hub_dev->ErrCnt = 0u;

        if ((p_dev->IsRootHub == true) &&
            (p_dev->HC_Ptr->IsVirRootHub == true))
        {
            p_dev->HC_Ptr->RH_ClassDevPtr = p_hub_dev;
        }

        *p_err = USBH_HUB_Init(p_hub_dev); /* Init HUB.                                            */
        if (*p_err != USBH_ERR_NONE)
        {
            USBH_HUB_RefRel(p_hub_dev);
        }
    }
    else
    {
        *p_err = USBH_ERR_CLASS_DRV_NOT_FOUND;
    }

    return ((void *)p_hub_dev);
}

/*
*********************************************************************************************************
*                                         USBH_HUB_Suspend()
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

static void USBH_HUB_Suspend(void *p_class_dev)
{
    uint16_t nbr_ports;
    uint16_t port_ix;
    struct usbh_dev *p_dev;
    struct usbh_hub_dev *p_hub_dev;

    p_hub_dev = (struct usbh_hub_dev *)p_class_dev;
    nbr_ports = MIN(p_hub_dev->Desc.bNbrPorts, USBH_CFG_MAX_HUB_PORTS);

    for (port_ix = 0u; port_ix < nbr_ports; port_ix++)
    {
        p_dev = (struct usbh_dev *)p_hub_dev->DevPtrList[port_ix];

        if (p_dev != (struct usbh_dev *)0)
        {
            usbh_class_suspend(p_dev);
        }
    }
}

/*
*********************************************************************************************************
*                                          USBH_HUB_Resume()
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

static void USBH_HUB_Resume(void *p_class_dev)
{
    uint16_t nbr_ports;
    uint16_t port_ix;
    struct usbh_dev *p_dev;
    struct usbh_hub_dev *p_hub_dev;
    struct usbh_hub_port_status port_status;

    p_hub_dev = (struct usbh_hub_dev *)p_class_dev;
    nbr_ports = MIN(p_hub_dev->Desc.bNbrPorts, USBH_CFG_MAX_HUB_PORTS);

    for (port_ix = 0u; port_ix < nbr_ports; port_ix++)
    {
        USBH_HUB_PortSuspendClr(p_hub_dev, port_ix + 1u); /* En resume signaling on port.                         */
    }

    k_sleep(K_MSEC(20u + 12u)); /* See Note (1).                                        */

    for (port_ix = 0u; port_ix < nbr_ports; port_ix++)
    {
        p_dev = p_hub_dev->DevPtrList[port_ix];

        if (p_dev != (struct usbh_dev *)0)
        {
            usbh_class_resume(p_dev);
        }
        else
        { /* Get port status info.                                */
            (void)USBH_HUB_PortStatusGet(p_hub_dev,
                                         port_ix + 1u,
                                         &port_status);

            if ((port_status.wPortStatus & USBH_HUB_STATUS_PORT_CONN) != 0u)
            {
                (void)USBH_HUB_PortResetSet(p_hub_dev,
                                            port_ix + 1u);
            }
        }
    }
}

/*
*********************************************************************************************************
*                                         USBH_HUB_Disconn()
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

static void USBH_HUB_Disconn(void *p_class_dev)
{
    struct usbh_hub_dev *p_hub_dev;

    p_hub_dev = (struct usbh_hub_dev *)p_class_dev;
    p_hub_dev->State = USBH_CLASS_DEV_STATE_DISCONN;

    USBH_HUB_Uninit(p_hub_dev);
    USBH_HUB_RefRel(p_hub_dev);
}

/*
*********************************************************************************************************
*                                           USBH_HUB_Init()
*
* Description : Opens the endpoints, reads hub descriptor, initializes ports and submits request to
*               start receiving hub events.
*
* Argument(s) : p_hub_dev       Pointer to hub structure.
*
* Return(s)   : USBH_ERR_NONE,                          If hub is successfully initialized.
*
*                                                       ----- RETURNED BY USBH_HUB_DescGet() : -----
*               USBH_ERR_DESC_INVALID,                  if hub descriptor is invalid.
*               USBH_ERR_UNKNOWN,                       Unknown error occurred.
*               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
*               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
*               USBH_ERR_HC_IO,                         Root hub input/output error.
*               USBH_ERR_EP_STALL,                      Root hub does not support request.
*               Host controller drivers error code,     Otherwise.
*
*                                                       ----- RETURNED BY USBH_HUB_EP_Open() : -----
*               USBH_ERR_EP_ALLOC,                      If USBH_CFG_MAX_NBR_EPS reached.
*               USBH_ERR_EP_NOT_FOUND,                  If endpoint with given type and direction not found.
*               USBH_ERR_OS_SIGNAL_CREATE,              if mutex or semaphore creation failed.
*               Host controller drivers error,          Otherwise.
*
*                                                       ----- RETURNED BY USBH_HUB_PortsInit() : -----
*               USBH_ERR_UNKNOWN,                       Unknown error occurred.
*               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
*               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
*               USBH_ERR_HC_IO,                         Root hub input/output error.
*               USBH_ERR_EP_STALL,                      Root hub does not support request.
*               Host controller drivers error code,     Otherwise.
*
*                                                       ----- RETURNED BY USBH_HUB_EventReq() : -----
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

static USBH_ERR USBH_HUB_Init(struct usbh_hub_dev *p_hub_dev)
{
    USBH_ERR err;

    err = USBH_HUB_EP_Open(p_hub_dev); /* Open intr EP.                                        */
    if (err != USBH_ERR_NONE)
    {
        return (err);
    }

    err = USBH_HUB_DescGet(p_hub_dev); /* Get hub desc.                                        */
    if (err != USBH_ERR_NONE)
    {
        return (err);
    }

    err = USBH_HUB_PortsInit(p_hub_dev); /* Init hub ports.                                      */
    if (err != USBH_ERR_NONE)
    {
        return (err);
    }

    err = USBH_HUB_EventReq(p_hub_dev); /* Start receiving hub evts.                            */

    return (err);
}

/*
*********************************************************************************************************
*                                          USBH_HUB_Uninit()
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

static void USBH_HUB_Uninit(struct usbh_hub_dev *p_hub_dev)
{
    uint16_t nbr_ports;
    uint16_t port_ix;
    struct usbh_dev *p_dev;

    USBH_HUB_EP_Close(p_hub_dev);
    nbr_ports = MIN(p_hub_dev->Desc.bNbrPorts, USBH_CFG_MAX_HUB_PORTS);

    for (port_ix = 0u; port_ix < nbr_ports; port_ix++)
    {
        p_dev = (struct usbh_dev *)p_hub_dev->DevPtrList[port_ix];

        if (p_dev != (struct usbh_dev *)0)
        {
            usbh_dev_disconn(p_dev);
            p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount++;

            p_hub_dev->DevPtrList[port_ix] = (struct usbh_dev *)0;
        }
    }
}

/*
*********************************************************************************************************
*                                         USBH_HUB_EP_Open()
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

static USBH_ERR USBH_HUB_EP_Open(struct usbh_hub_dev *p_hub_dev)
{
    USBH_ERR err;
    struct usbh_dev *p_dev;
    struct usbh_if *p_if;

    p_dev = p_hub_dev->DevPtr;
    p_if = p_hub_dev->IF_Ptr;

    err = usbh_intr_in_open(p_dev, /* Find and open hub intr EP.                           */
                          p_if,
                          &p_hub_dev->IntrEP);

    return (err);
}

/*
*********************************************************************************************************
*                                         USBH_HUB_EP_Close()
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

static void USBH_HUB_EP_Close(struct usbh_hub_dev *p_hub_dev)
{
    usbh_ep_close(&p_hub_dev->IntrEP); /* Close hub intr EP.                                   */
}

/*
*********************************************************************************************************
*                                         USBH_HUB_EventReq()
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

static USBH_ERR USBH_HUB_EventReq(struct usbh_hub_dev *p_hub_dev)
{
    uint32_t len;
    bool valid;
    const struct usbh_hc_rh_api *p_rh_api;
    USBH_ERR err;
    struct usbh_dev *p_dev;

    p_dev = p_hub_dev->DevPtr;

    if ((p_dev->IsRootHub == true) && /* Chk if RH fncts are supported before calling HCD.    */
        (p_dev->HC_Ptr->IsVirRootHub == true))
    {

        p_rh_api = p_dev->HC_Ptr->HC_Drv.RH_API_Ptr;
        valid = p_rh_api->IntEn(&p_dev->HC_Ptr->HC_Drv);
        if (valid != 1)
        {
            return (USBH_ERR_HC_IO);
        }
        else
        {
            return (USBH_ERR_NONE);
        }
    }

    len = (p_hub_dev->Desc.bNbrPorts / 8u) + 1u; /* See Note (1).                                        */
    err = usbh_intr_rx_async(&p_hub_dev->IntrEP,   /* Start receiving hub events.                          */
                           (void *)p_hub_dev->HubIntrBuf,
                           len,
                           USBH_HUB_ISR,
                           (void *)p_hub_dev);
    return (err);
}

/*
*********************************************************************************************************
*                                           USBH_HUB_ISR()
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

static void USBH_HUB_ISR(struct usbh_ep *p_ep,
                         void *p_buf,
                         uint32_t buf_len,
                         uint32_t xfer_len,
                         void *p_arg,
                         USBH_ERR err)
{
    struct usbh_hub_dev *p_hub_dev;
    int key;

    (void)buf_len;
    (void)p_buf;
    (void)p_ep;
    (void)xfer_len;

    p_hub_dev = (struct usbh_hub_dev *)p_arg;

    if (err != USBH_ERR_NONE)
    {
        if (p_hub_dev->State == USBH_CLASS_DEV_STATE_CONN)
        {
            if (p_hub_dev->ErrCnt < 3u)
            {
                LOG_ERR("USBH_HUB_ISR() fails. Err=%d errcnt=%d\r\n",
                        err,
                        p_hub_dev->ErrCnt);

                p_hub_dev->ErrCnt++;
                USBH_HUB_EventReq(p_hub_dev); /* Retry URB.                                           */
            }
        }
        return;
    }

    p_hub_dev->ErrCnt = 0u;

    USBH_HUB_RefAdd(p_hub_dev);

    key = irq_lock();
    if (USBH_HUB_HeadPtr == (struct usbh_hub_dev *)0)
    {
        USBH_HUB_HeadPtr = USBH_HUB_TailPtr = p_hub_dev;
    }
    else
    {
        USBH_HUB_TailPtr->NxtPtr = p_hub_dev;
        USBH_HUB_TailPtr = p_hub_dev;
    }
    irq_unlock(key);

    k_sem_give(&USBH_HUB_EventSem);
}

/*
*********************************************************************************************************
*                                       USBH_HUB_EventProcess()
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

static void USBH_HUB_EventProcess(void)
{
    uint16_t nbr_ports;
    uint16_t port_nbr;
    enum usbh_device_speed dev_spd;
    struct usbh_hub_dev *p_hub_dev;
    struct usbh_hub_port_status port_status;
    struct usbh_dev *p_dev;
    USBH_ERR err;
    int key;

    key = irq_lock();
    p_hub_dev = (struct usbh_hub_dev *)USBH_HUB_HeadPtr;

    if (USBH_HUB_HeadPtr == USBH_HUB_TailPtr)
    {
        USBH_HUB_HeadPtr = (struct usbh_hub_dev *)0;
        USBH_HUB_TailPtr = (struct usbh_hub_dev *)0;
    }
    else
    {
        USBH_HUB_HeadPtr = USBH_HUB_HeadPtr->NxtPtr;
    }
    irq_unlock(key);

    if (p_hub_dev == (struct usbh_hub_dev *)0)
    {
        return;
    }

    if (p_hub_dev->State == USBH_CLASS_DEV_STATE_DISCONN)
    {
        LOG_DBG("device state disconnected");
        err = USBH_HUB_RefRel(p_hub_dev);
        if (err != USBH_ERR_NONE)
        {
            USBH_PRINT_ERR(err);
        }
        return;
    }

    port_nbr = 1u;
    nbr_ports = MIN(p_hub_dev->Desc.bNbrPorts, USBH_CFG_MAX_HUB_PORTS);

    while (port_nbr <= nbr_ports)
    {

        err = USBH_HUB_PortStatusGet(p_hub_dev, /* Get port status info..                               */
                                     port_nbr,
                                     &port_status);
        if (err != USBH_ERR_NONE)
        {
            break;
        }
        /* ------------- CONNECTION STATUS CHANGE ------------- */
        if (DEF_BIT_IS_SET(port_status.wPortChange, USBH_HUB_STATUS_C_PORT_CONN) == true)
        {
            LOG_DBG("connection status change");
            err = USBH_HUB_PortConnChngClr(p_hub_dev, /* Clr port conn chng.                                  */
                                           port_nbr);
            if (err != USBH_ERR_NONE)
            {
                break;
            }
            /* -------------- DEV HAS BEEN CONNECTED -------------- */
            if (DEF_BIT_IS_SET(port_status.wPortStatus, USBH_HUB_STATUS_PORT_CONN) == true)
            {

                LOG_DBG("Port %d : Device Connected.\r\n", port_nbr);

                p_hub_dev->ConnCnt = 0; /* Reset re-connection counter                          */
                p_dev = p_hub_dev->DevPtrList[port_nbr - 1u];
                if (p_dev != (struct usbh_dev *)0)
                {
                    usbh_dev_disconn(p_dev);
                    p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount++;
                    p_hub_dev->DevPtrList[port_nbr - 1u] = (struct usbh_dev *)0;
                }

                k_sleep(K_MSEC(100u));                   /* See Notes #1.                                        */
                err = USBH_HUB_PortResetSet(p_hub_dev, /* Apply port reset.                                    */
                                            port_nbr);
                if (err != USBH_ERR_NONE)
                {
                    break;
                }

                k_sleep(K_MSEC(USBH_HUB_DLY_DEV_RESET)); /* See Notes #2.                                        */
                continue;                              /* Handle port reset status change.                     */
            }
            else
            { /* --------------- DEV HAS BEEN REMOVED --------------- */
                LOG_DBG("device has been removed");
                k_sleep(K_MSEC(10u)); /* Wait for any pending I/O xfer to rtn err.            */

                p_dev = p_hub_dev->DevPtrList[port_nbr - 1u];

                if (p_dev != (struct usbh_dev *)0)
                {
                    usbh_dev_disconn(p_dev);
                    p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount++;

                    p_hub_dev->DevPtrList[port_nbr - 1u] = (struct usbh_dev *)0;
                }
            }
        }
        /* ------------- PORT RESET STATUS CHANGE ------------- */
        if (DEF_BIT_IS_SET(port_status.wPortChange, USBH_HUB_STATUS_C_PORT_RESET) == true)
        {
            err = USBH_HUB_PortRstChngClr(p_hub_dev, port_nbr);
            if (err != USBH_ERR_NONE)
            {
                break;
            }
            /* Dev has been connected.                              */
            if (DEF_BIT_IS_SET(port_status.wPortStatus, USBH_HUB_STATUS_PORT_CONN) == true)
            {

                err = USBH_HUB_PortStatusGet(p_hub_dev, /* Get port status info.                                */
                                             port_nbr,
                                             &port_status);
                if (err != USBH_ERR_NONE)
                {
                    break;
                }

                /* Determine dev spd.                                   */
                if (DEF_BIT_IS_SET(port_status.wPortStatus, USBH_HUB_STATUS_PORT_LOW_SPD) == true)
                {
                    dev_spd = USBH_LOW_SPEED;
                }
                else if (DEF_BIT_IS_SET(port_status.wPortStatus, USBH_HUB_STATUS_PORT_HIGH_SPD) == true)
                {
                    dev_spd = USBH_HIGH_SPEED;
                }
                else
                {
                    dev_spd = USBH_FULL_SPEED;
                }

                LOG_DBG("Port %d : Port Reset complete, device speed is %s\r\n", port_nbr,
                        (dev_spd == USBH_LOW_SPEED) ? "LOW Speed(1.5 Mb/Sec)" : (dev_spd == USBH_FULL_SPEED) ? "FULL Speed(12 Mb/Sec)" : "HIGH Speed(480 Mb/Sec)");

                p_dev = p_hub_dev->DevPtrList[port_nbr - 1u];

                if (p_dev != (struct usbh_dev *)0)
                {
                    continue;
                }

                if (p_hub_dev->DevPtr->HC_Ptr->HostPtr->State == USBH_HOST_STATE_SUSPENDED)
                {
                    continue;
                }
                if (p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount < 0)
                {
                    usbh_hub_port_dis(p_hub_dev, port_nbr);
                    USBH_HUB_RefRel(p_hub_dev);
                    USBH_HUB_EventReq(p_hub_dev); /* Retry URB.                                           */

                    return;
                }
                else
                {
                    p_dev = &p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevList[p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount--];
                }

                p_dev->DevSpd = dev_spd;
                p_dev->HubDevPtr = p_hub_dev->DevPtr;
                p_dev->PortNbr = port_nbr;
                p_dev->HC_Ptr = p_hub_dev->DevPtr->HC_Ptr;

                if (dev_spd == USBH_HIGH_SPEED)
                {
                    p_dev->HubHS_Ptr = p_hub_dev;
                }
                else
                {
                    if (p_hub_dev->IntrEP.DevSpd == USBH_HIGH_SPEED)
                    {
                        p_dev->HubHS_Ptr = p_hub_dev;
                    }
                    else
                    {
                        p_dev->HubHS_Ptr = p_hub_dev->DevPtr->HubHS_Ptr;
                    }
                }

                k_sleep(K_MSEC(50u));
                err = usbh_dev_conn(p_dev); /* Conn dev.                                            */
                if (err != USBH_ERR_NONE)
                {
                    usbh_hub_port_dis(p_hub_dev, port_nbr);
                    usbh_dev_disconn(p_dev);

                    p_hub_dev->DevPtr->HC_Ptr->HostPtr->DevCount++;

                    if (p_hub_dev->ConnCnt < USBH_CFG_MAX_NUM_DEV_RECONN)
                    {
                        /*This condition may happen due to EP_STALL return      */
                        err = USBH_HUB_PortResetSet(p_hub_dev, /* Apply port reset.                                    */
                                                    port_nbr);
                        if (err != USBH_ERR_NONE)
                        {
                            break;
                        }

                        k_sleep(K_MSEC(USBH_HUB_DLY_DEV_RESET)); /* See Notes #2.                                        */
                        p_hub_dev->ConnCnt++;
                        ;
                        continue; /* Handle port reset status change.                     */
                    }
                    else
                    {
                        p_hub_dev->DevPtrList[port_nbr - 1u] = (struct usbh_dev *)0;
                    }
                }
                else
                {
                    p_hub_dev->DevPtrList[port_nbr - 1u] = p_dev;
                }
            }
        }
        /* ------------ PORT ENABLE STATUS CHANGE ------------- */
        if (DEF_BIT_IS_SET(port_status.wPortChange, USBH_HUB_STATUS_C_PORT_EN) == true)
        {
            err = USBH_HUB_PortEnChngClr(p_hub_dev, port_nbr);
            if (err != USBH_ERR_NONE)
            {
                break;
            }
        }
        port_nbr++;
    }
    USBH_HUB_EventReq(p_hub_dev); /* Retry URB.                                           */

    USBH_HUB_RefRel(p_hub_dev);
}

/*
*********************************************************************************************************
*                                         USBH_HUB_DescGet()
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

static USBH_ERR USBH_HUB_DescGet(struct usbh_hub_dev *p_hub_dev)
{
    USBH_ERR err;
    uint32_t len;
    uint32_t i;
    struct usbh_desc_hdr hdr;

    for (i = 0u; i < USBH_CFG_STD_REQ_RETRY; i++)
    { /* Attempt to get desc hdr 3 times.                     */
        len = usbh_ctrl_rx(p_hub_dev->DevPtr,
                          USBH_REQ_GET_DESC, /* See Note (1).                                        */
                          (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_TYPE_CLASS),
                          (USBH_HUB_DESC_TYPE_HUB << 8u),
                          0u,
                          (void *)&hdr,
                          USBH_LEN_DESC_HDR,
                          USBH_HUB_TIMEOUT,
                          &err);
        if ((err == USBH_ERR_EP_STALL) ||
            (len == 0u))
        {
            usbh_ep_reset(p_hub_dev->DevPtr,
                          (struct usbh_ep *)0);
        }
        else
        {
            break;
        }
    }

    if (len != USBH_LEN_DESC_HDR)
    {
        return (USBH_ERR_DESC_INVALID);
    }

    if ((hdr.bLength == 0u) ||
        (hdr.bLength > USBH_HUB_MAX_DESC_LEN) ||
        (hdr.bDescriptorType != USBH_HUB_DESC_TYPE_HUB))
    {
        return (USBH_ERR_DESC_INVALID);
    }

    for (i = 0u; i < USBH_CFG_STD_REQ_RETRY; i++)
    { /* Attempt to get full desc 3 times.                    */
        len = usbh_ctrl_rx(p_hub_dev->DevPtr,
                          USBH_REQ_GET_DESC,
                          (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_TYPE_CLASS),
                          (USBH_HUB_DESC_TYPE_HUB << 8),
                          0u,
                          (void *)USBH_HUB_DescBuf,
                          hdr.bLength,
                          USBH_HUB_TIMEOUT,
                          &err);
        if ((err == USBH_ERR_EP_STALL) ||
            (len < hdr.bLength))
        {
            usbh_ep_reset(p_hub_dev->DevPtr,
                          (struct usbh_ep *)0);
        }
        else
        {
            break;
        }
    }

    usbh_hub_parse_hub_desc(&p_hub_dev->Desc,
                          USBH_HUB_DescBuf);

    if (p_hub_dev->Desc.bNbrPorts > USBH_CFG_MAX_HUB_PORTS)
    { /* Warns limit on hub port nbr to max cfg'd.            */
        LOG_WRN("Only ports [1..%d] are active.\r\n", USBH_CFG_MAX_HUB_PORTS);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                        USBH_HUB_PortsInit()
*
* Description : Enable power on each hub port & initialize device on each port.
*
* Argument(s) : p_hub_dev       Pointer to the hub.
*
* Return(s)   : USBH_ERR_NONE,                          if ports were successfully initialized.
*
*                                                       ----- RETURNED BY USBH_HUB_PortPwrSet() : -----
*               USBH_ERR_UNKNOWN,                       Unknown error occurred.
*               USBH_ERR_INVALID_ARG,                   Invalid argument passed to 'p_ep'.
*               USBH_ERR_EP_INVALID_STATE,              Endpoint is not opened.
*               USBH_ERR_HC_IO,                         Root hub input/output error.
*               USBH_ERR_EP_STALL,                      Root hub does not support request.
*               Host controller drivers error code,     Otherwise.
*
* Note(s)     : (1) USB2.0 specification states that the host must wait for (bPwrOn2PwrGood * 2) ms
*                   before accessing a powered port. See section 11.23.2.1, bPwrOn2PwrGood field in hub
*                   descriptor.
*********************************************************************************************************
*/

static USBH_ERR USBH_HUB_PortsInit(struct usbh_hub_dev *p_hub_dev)
{
    USBH_ERR err;
    uint32_t i;
    uint16_t nbr_ports;

    nbr_ports = MIN(p_hub_dev->Desc.bNbrPorts, USBH_CFG_MAX_HUB_PORTS);

    for (i = 0u; i < nbr_ports; i++)
    {
        err = USBH_HUB_PortPwrSet(p_hub_dev, i + 1u); /* Set port pwr.                                      */

        if (err != USBH_ERR_NONE)
        {
            LOG_ERR("PortPwrSet error");
            USBH_PRINT_ERR(err);
            return (err);
        }
        k_sleep(K_MSEC(p_hub_dev->Desc.bPwrOn2PwrGood * 2u)); /* See Note (1).                                       */
    }
    return (USBH_ERR_NONE);
}

/*
*********************************************************************************************************
*                                      USBH_HUB_PortStatusGet()
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

static USBH_ERR USBH_HUB_PortStatusGet(struct usbh_hub_dev *p_hub_dev,
                                       uint16_t port_nbr,
                                       struct usbh_hub_port_status *p_port_status)
{
    uint8_t *p_buf;
    struct usbh_hub_port_status port_status;
    USBH_ERR err;

    p_buf = (uint8_t *)&port_status;

    (void)usbh_ctrl_rx(p_hub_dev->DevPtr, /* See Note (1).                                        */
                      USBH_REQ_GET_STATUS,
                      (USBH_REQ_DIR_DEV_TO_HOST | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      0u,
                      port_nbr,
                      (void *)p_buf,
                      USBH_HUB_LEN_HUB_PORT_STATUS,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }
    else
    {
        p_port_status->wPortStatus = sys_get_le16((uint8_t *)&port_status.wPortStatus);
        p_port_status->wPortChange = sys_get_le16((uint8_t *)&port_status.wPortChange);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                       USBH_HUB_PortResetSet()
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

static USBH_ERR USBH_HUB_PortResetSet(struct usbh_hub_dev *p_hub_dev,
                                      uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note (1).                                        */
                      USBH_REQ_SET_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_PORT_RESET,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                      USBH_HUB_PortRstChngClr()
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

static USBH_ERR USBH_HUB_PortRstChngClr(struct usbh_hub_dev *p_hub_dev,
                                        uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note (1).                                        */
                      USBH_REQ_CLR_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_C_PORT_RESET,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                      USBH_HUB_PortEnChngClr()
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

static USBH_ERR USBH_HUB_PortEnChngClr(struct usbh_hub_dev *p_hub_dev,
                                       uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note (1).                                        */
                      USBH_REQ_CLR_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_C_PORT_EN,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                     USBH_HUB_PortConnChngClr()
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

static USBH_ERR USBH_HUB_PortConnChngClr(struct usbh_hub_dev *p_hub_dev,
                                         uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
                      USBH_REQ_CLR_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_C_PORT_CONN,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                         USBH_HUB_PortPwrSet()
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

static USBH_ERR USBH_HUB_PortPwrSet(struct usbh_hub_dev *p_hub_dev,
                                    uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
                      USBH_REQ_SET_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_PORT_PWR,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                      USBH_HUB_PortSuspendClr()
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

static USBH_ERR USBH_HUB_PortSuspendClr(struct usbh_hub_dev *p_hub_dev,
                                        uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
                      USBH_REQ_CLR_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_C_PORT_SUSPEND,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                        USBH_HUB_PortEnClr()
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

static USBH_ERR USBH_HUB_PortEnClr(struct usbh_hub_dev *p_hub_dev,
                                   uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
                      USBH_REQ_CLR_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_PORT_EN,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                        USBH_HUB_PortEnSet()
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

static USBH_ERR USBH_HUB_PortEnSet(struct usbh_hub_dev *p_hub_dev,
                                   uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
                      USBH_REQ_SET_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_PORT_EN,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
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

USBH_ERR usbh_hub_port_suspend_set(struct usbh_hub_dev *p_hub_dev,
                                   uint16_t port_nbr)
{
    USBH_ERR err;

    (void)usbh_ctrl_tx(p_hub_dev->DevPtr, /* See Note #1.                                         */
                      USBH_REQ_SET_FEATURE,
                      (USBH_REQ_DIR_HOST_TO_DEV | USBH_REQ_TYPE_CLASS | USBH_REQ_RECIPIENT_OTHER),
                      USBH_HUB_FEATURE_SEL_PORT_SUSPEND,
                      port_nbr,
                      (void *)0,
                      0u,
                      USBH_HUB_TIMEOUT,
                      &err);
    if (err != USBH_ERR_NONE)
    {
        usbh_ep_reset(p_hub_dev->DevPtr, (struct usbh_ep *)0);
    }

    return (err);
}

/*
*********************************************************************************************************
*                                           USBH_HUB_Clr()
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

static void USBH_HUB_Clr(struct usbh_hub_dev *p_hub_dev)
{
    uint8_t dev_ix;

    p_hub_dev->DevPtr = (struct usbh_dev *)0;
    p_hub_dev->IF_Ptr = (struct usbh_if *)0;
    /* Clr dev ptr lst.                                     */
    for (dev_ix = 0u; dev_ix < USBH_CFG_MAX_HUB_PORTS; dev_ix++)
    {
        p_hub_dev->DevPtrList[dev_ix] = (struct usbh_dev *)0;
    }

    p_hub_dev->RefCnt = 0u;
    p_hub_dev->State = USBH_CLASS_DEV_STATE_NONE;
    p_hub_dev->NxtPtr = 0u;
}

/*
*********************************************************************************************************
*                                          USBH_HUB_RefAdd()
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

static USBH_ERR USBH_HUB_RefAdd(struct usbh_hub_dev *p_hub_dev)
{
    int key;

    if (p_hub_dev == (struct usbh_hub_dev *)0)
    {
        return (USBH_ERR_INVALID_ARG);
    }

    key = irq_lock();
    p_hub_dev->RefCnt++; /* Increment access ref cnt to hub dev.                 */

    irq_unlock(key);
    return (USBH_ERR_NONE);
}

/*
*********************************************************************************************************
*                                          USBH_HUB_RefRel()
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

static USBH_ERR USBH_HUB_RefRel(struct usbh_hub_dev *p_hub_dev)
{
    int key;

    if (p_hub_dev == (struct usbh_hub_dev *)0)
    {
        return (USBH_ERR_INVALID_ARG);
    }

    key = irq_lock();
    if (p_hub_dev->RefCnt > 0u)
    {
        p_hub_dev->RefCnt--; /* Decrement access ref cnt to hub dev.                 */

        if (p_hub_dev->RefCnt == 0u)
        {
            HubCount++;
        }
    }
    irq_unlock(key);

    return (USBH_ERR_NONE);
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
                               USBH_ERR *p_err)
{
    uint32_t len;
    struct usbh_hc_drv *p_hc_drv;
    const struct usbh_hc_rh_api *p_hc_rh_api;
    bool valid;

    p_hc_drv = &p_hc->HC_Drv;
    p_hc_rh_api = p_hc_drv->RH_API_Ptr;
    *p_err = USBH_ERR_NONE;
    len = 0u;
    valid = 1;

    switch (b_req)
    {
    case USBH_REQ_GET_STATUS:
        /* Only port status is supported.                       */
        if ((bm_req_type & USBH_REQ_RECIPIENT_OTHER) == USBH_REQ_RECIPIENT_OTHER)
        {
            valid = p_hc_rh_api->PortStatusGet(p_hc_drv,
                                               w_ix,
                                               (struct usbh_hub_port_status *)p_buf);
        }
        else
        {
            len = buf_len;
            memset(p_buf, 0, len); /* Return 0 for other reqs.                             */
        }
        break;

    case USBH_REQ_CLR_FEATURE:
        switch (w_val)
        {
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
            *p_err = USBH_ERR_EP_STALL;
            break;

        default:
            break;
        }
        break;

    case USBH_REQ_SET_FEATURE:
        switch (w_val)
        {
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
            *p_err = USBH_ERR_EP_STALL;
            break;

        default:
            break;
        }
        break;

    case USBH_REQ_SET_ADDR:
        break;

    case USBH_REQ_GET_DESC:
        switch (w_val >> 8u)
        { /* Desc type.                                           */
        case USBH_DESC_TYPE_DEV:
            if (buf_len > sizeof(USBH_HUB_RH_DevDesc))
            {
                len = sizeof(USBH_HUB_RH_DevDesc);
            }
            else
            {
                len = buf_len;
            }

            memcpy((void *)p_buf,
                     (void *)USBH_HUB_RH_DevDesc,
                     len);
            break;

        case USBH_DESC_TYPE_CFG: /* Return cfg desc.                                     */
            if (buf_len > sizeof(USBH_HUB_RH_FS_CfgDesc))
            {
                len = sizeof(USBH_HUB_RH_FS_CfgDesc);
            }
            else
            {
                len = buf_len;
            }
            memcpy((void *)p_buf,
                     (void *)USBH_HUB_RH_FS_CfgDesc,
                     len);
            break;

        case USBH_HUB_DESC_TYPE_HUB: /* Return hub desc.                                     */
            len = buf_len;
            valid = p_hc_rh_api->HubDescGet(p_hc_drv,
                                            (struct usbh_hub_desc *)p_buf,
                                            len);
            break;

        case USBH_DESC_TYPE_STR:

            if ((w_val & 0x00FF) == 0u)
            {
                if (buf_len > sizeof(USBH_HUB_RH_LangID))
                {
                    len = sizeof(USBH_HUB_RH_LangID);
                }
                else
                {
                    len = buf_len;
                }
                memcpy((void *)p_buf,
                         (void *)USBH_HUB_RH_LangID,
                         len);
            }
            else
            {
                *p_err = USBH_ERR_EP_STALL;
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
        *p_err = USBH_ERR_EP_STALL;
        break;

    default:
        break;
    }

    if ((valid != 1) &&
        (*p_err == USBH_ERR_NONE))
    {
        *p_err = USBH_ERR_HC_IO;
    }

    return (len);
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
    if (p_hub_dev == (struct usbh_hub_dev *)0)
    {
        (void)p_rh_drv_api->IntDis(p_hc_drv);
        return;
    }

    (void)p_rh_drv_api->IntDis(p_hc_drv);
    LOG_DBG("RefAdd");
    USBH_HUB_RefAdd(p_hub_dev);

    key = irq_lock();
    if (USBH_HUB_HeadPtr == (struct usbh_hub_dev *)0)
    {
        USBH_HUB_HeadPtr = p_hub_dev;
        USBH_HUB_TailPtr = p_hub_dev;
    }
    else
    {
        USBH_HUB_TailPtr->NxtPtr = p_hub_dev;
        USBH_HUB_TailPtr = p_hub_dev;
    }
    irq_unlock(key);
    k_sem_give(&USBH_HUB_EventSem);
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

    (void)p_ctx;

    p_hub_dev = (struct usbh_hub_dev *)p_class_dev;
    p_dev = p_hub_dev->DevPtr;

    if (p_dev->IsRootHub == true)
    { /* If RH, return immediately.                           */
        return;
    }

    switch (state)
    { /* External hub has been identified.                    */
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

    p_hub_desc->bDescLength = p_buf_src_desc->bDescLength;
    p_hub_desc->bDescriptorType = p_buf_src_desc->bDescriptorType;
    p_hub_desc->bNbrPorts = p_buf_src_desc->bNbrPorts;
    p_hub_desc->wHubCharacteristics = sys_get_le16((uint8_t *)&p_buf_src_desc->wHubCharacteristics);
    p_hub_desc->bPwrOn2PwrGood = p_buf_src_desc->bPwrOn2PwrGood;
    p_hub_desc->bHubContrCurrent = p_buf_src_desc->bHubContrCurrent;
    p_hub_desc->DeviceRemovable = p_buf_src_desc->DeviceRemovable;

    for (i = 0u; i < USBH_CFG_MAX_HUB_PORTS; i++)
    {
        p_hub_desc->PortPwrCtrlMask[i] = sys_get_le32((uint8_t *)&p_buf_src_desc->PortPwrCtrlMask[i]);
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

    p_buf_dest_desc->bDescLength = p_hub_desc->bDescLength;
    p_buf_dest_desc->bDescriptorType = p_hub_desc->bDescriptorType;
    p_buf_dest_desc->bNbrPorts = p_hub_desc->bNbrPorts;
    p_buf_dest_desc->wHubCharacteristics = sys_get_le16((uint8_t *)&p_hub_desc->wHubCharacteristics);
    p_buf_dest_desc->bHubContrCurrent = p_hub_desc->bHubContrCurrent;
    p_buf_dest_desc->DeviceRemovable = p_hub_desc->DeviceRemovable;

    memcpy(&p_buf_dest_desc->bPwrOn2PwrGood, &p_hub_desc->bPwrOn2PwrGood, sizeof(uint8_t));
    memcpy(&p_buf_dest_desc->PortPwrCtrlMask[0], &p_hub_desc->PortPwrCtrlMask[0], (sizeof(uint32_t) * USBH_CFG_MAX_HUB_PORTS));
}
