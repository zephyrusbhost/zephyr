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

#include "usbh_class.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(usbh_class);

struct usbh_class_drv_reg usbh_class_drv_list[USBH_CFG_MAX_NBR_CLASS_DRVS];

static USBH_ERR usbh_class_probe_dev(struct usbh_dev *p_dev);

static USBH_ERR usbh_class_probe_if(struct usbh_dev *p_dev,
                                  struct usbh_if *p_if);

static void USBH_ClassNotify(struct usbh_dev *p_dev,
                             struct usbh_if *p_if,
                             void *p_class_dev,
                             uint8_t is_conn);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                          GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                         USBH_ClassDrvReg()
*
* Description : Registers a class driver to the USB host stack.
*
* Argument(s) : p_class_drv          Pointer to the class driver.
*
*               class_notify_fnct    Class notification function pointer.
*
*               p_class_notify_ctx   Class notification function context pointer.
*
* Return(s)   : USBH_ERR_NONE               If the class driver is registered.
*               USBH_ERR_INVALID_ARG        If invalid argument(s) passed to 'p_host'/ 'p_class_drv'.
*               USBH_ERR_CLASS_DRV_ALLOC    If maximum class driver limit reached.
*
* Note(s)     : None.
*********************************************************************************************************
*/

USBH_ERR usbh_class_drv_reg(const struct usbh_class_drv *p_class_drv,
                            USBH_CLASS_NOTIFY_FNCT class_notify_fnct,
                            void *p_class_notify_ctx)
{
    uint32_t ix;
    USBH_ERR err;
    int key;

    if (p_class_drv == (const struct usbh_class_drv *)0)
    {
        return (USBH_ERR_INVALID_ARG);
    }

    if (p_class_drv->NamePtr == (uint8_t *)0)
    {
        return (USBH_ERR_INVALID_ARG);
    }

    if ((p_class_drv->ProbeDev == 0) &&
        (p_class_drv->ProbeIF == 0))
    {
        return (USBH_ERR_INVALID_ARG);
    }

    key = irq_lock();
    for (ix = 0u; ix < USBH_CFG_MAX_NBR_CLASS_DRVS; ix++)
    { /* Find first empty element in the class drv list.      */

        if (usbh_class_drv_list[ix].InUse == 0u)
        { /* Insert class drv if it is empty location.            */
            usbh_class_drv_list[ix].ClassDrvPtr = p_class_drv;
            usbh_class_drv_list[ix].NotifyFnctPtr = class_notify_fnct;
            usbh_class_drv_list[ix].NotifyArgPtr = p_class_notify_ctx;
            usbh_class_drv_list[ix].InUse = 1u;
            break;
        }
    }
    irq_unlock(key);

    if (ix >= USBH_CFG_MAX_NBR_CLASS_DRVS)
    { /* List is full.                                        */
        return (USBH_ERR_CLASS_DRV_ALLOC);
    }

    p_class_drv->GlobalInit(&err);

    return (err);
}

/*
*********************************************************************************************************
*                                        USBH_ClassDrvUnreg()
*
* Description : Unregisters a class driver from the USB host stack.
*
* Argument(s) : p_class_drv     Pointer to the class driver.
*
* Return(s)   : USBH_ERR_NONE                   If the class driver is unregistered.
*               USBH_ERR_INVALID_ARG            If invalid argument(s) passed to 'p_host'/ 'p_class_drv'.
*               USBH_ERR_CLASS_DRV_NOT_FOUND    If no class driver found.
*
* Note(s)     : None.
*********************************************************************************************************
*/

USBH_ERR usbh_class_drv_unreg(const struct usbh_class_drv *p_class_drv)
{
    uint32_t ix;
    int key;

    if (p_class_drv == (const struct usbh_class_drv *)0)
    {
        return (USBH_ERR_INVALID_ARG);
    }

    key = irq_lock();
    for (ix = 0u; ix < USBH_CFG_MAX_NBR_CLASS_DRVS; ix++)
    { /* Find the element in the class driver list.           */

        if ((usbh_class_drv_list[ix].InUse != 0u) &&
            (usbh_class_drv_list[ix].ClassDrvPtr == p_class_drv))
        {

            usbh_class_drv_list[ix].ClassDrvPtr = (const struct usbh_class_drv *)0;
            usbh_class_drv_list[ix].NotifyFnctPtr = (USBH_CLASS_NOTIFY_FNCT)0;
            usbh_class_drv_list[ix].NotifyArgPtr = (void *)0;
            usbh_class_drv_list[ix].InUse = 0u;
            break;
        }
    }
    irq_unlock(key);

    if (ix >= USBH_CFG_MAX_NBR_CLASS_DRVS)
    {
        return (USBH_ERR_CLASS_DRV_NOT_FOUND);
    }

    return (USBH_ERR_NONE);
}

/*
*********************************************************************************************************
*                                         USBH_ClassSuspend()
*
* Description : Suspend all class drivers associated to the device.
*
* Argument(s) : p_dev       Pointer to USB device.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

void usbh_class_suspend(struct usbh_dev *p_dev)
{
    uint8_t if_ix;
    uint8_t nbr_ifs;
    struct usbh_cfg *p_cfg;
    struct usbh_if *p_if;
    const struct usbh_class_drv *p_class_drv;

    if ((p_dev->ClassDevPtr != (void *)0) && /* If class drv is present at dev level.                */
        (p_dev->ClassDrvRegPtr != (struct usbh_class_drv_reg *)0))
    {
        p_class_drv = p_dev->ClassDrvRegPtr->ClassDrvPtr;
        /* Chk if class drv is present at dev level.            */
        if ((p_class_drv != (const struct usbh_class_drv *)0) &&
            (p_class_drv->Suspend != (void *)0))
        {
            p_class_drv->Suspend(p_dev->ClassDevPtr);
            return;
        }
    }

    p_cfg = usbh_cfg_get(p_dev, 0u); /* Get first cfg.                                       */
    nbr_ifs = usbh_cfg_if_nbr_get(p_cfg);

    for (if_ix = 0u; if_ix < nbr_ifs; if_ix++)
    {
        p_if = usbu_if_get(p_cfg, if_ix);
        if (p_if == (struct usbh_if *)0)
        {
            return;
        }

        if ((p_if->ClassDevPtr != (void *)0) &&
            (p_if->ClassDrvRegPtr != (struct usbh_class_drv_reg *)0))
        {
            p_class_drv = p_if->ClassDrvRegPtr->ClassDrvPtr;

            if ((p_class_drv != (const struct usbh_class_drv *)0) &&
                (p_class_drv->Suspend != (void *)0))
            {
                p_class_drv->Suspend(p_if->ClassDevPtr);
                return;
            }
        }
    }
}

/*
*********************************************************************************************************
*                                          USBH_ClassResume()
*
* Description : Resume all class drivers associated to the device.
*
* Argument(s) : p_dev   Pointer to USB device.
*
* Return(s)   : None
*
* Note(s)     : None
*********************************************************************************************************
*/

void usbh_class_resume(struct usbh_dev *p_dev)
{
    uint8_t if_ix;
    uint8_t nbr_ifs;
    struct usbh_cfg *p_cfg;
    struct usbh_if *p_if;
    const struct usbh_class_drv *p_class_drv;

    if ((p_dev->ClassDevPtr != (void *)0) && /* If class drv is present at dev level.                */
        (p_dev->ClassDrvRegPtr != (struct usbh_class_drv_reg *)0))
    {
        p_class_drv = p_dev->ClassDrvRegPtr->ClassDrvPtr;
        /* Chk if class drv is present at dev level.            */
        if ((p_class_drv != (const struct usbh_class_drv *)0) &&
            (p_class_drv->Resume != (void *)0))
        {
            p_class_drv->Resume(p_dev->ClassDevPtr);
            return;
        }
    }

    p_cfg = usbh_cfg_get(p_dev, 0u); /* Get 1st cfg.                                         */
    nbr_ifs = usbh_cfg_if_nbr_get(p_cfg);

    for (if_ix = 0u; if_ix < nbr_ifs; if_ix++)
    {
        p_if = usbu_if_get(p_cfg, if_ix);
        if (p_if == (struct usbh_if *)0)
        {
            return;
        }

        if ((p_if->ClassDevPtr != (void *)0) &&
            (p_if->ClassDrvRegPtr != (struct usbh_class_drv_reg *)0))
        {
            p_class_drv = p_if->ClassDrvRegPtr->ClassDrvPtr;

            if ((p_class_drv != (const struct usbh_class_drv *)0) &&
                (p_class_drv->Resume != (void *)0))
            {
                p_class_drv->Resume(p_if->ClassDevPtr);
                return;
            }
        }
    }
}

/*
*********************************************************************************************************
*                                         USBH_ClassDrvConn()
*
* Description : Once a device is connected, attempts to find a class driver matching the device descriptor.
*               If no class driver matches the device descriptor, it will attempt to find a class driver
*               for each interface present in the active configuration.
*
* Argument(s) : p_dev   Pointer to USB device.
*
* Return(s)   : USBH_ERR_NONE                           If a class driver was found.
*               USBH_ERR_DRIVER_NOT_FOUND               If no class driver was found.
*
*                                                       ----- RETURNED BY USBH_CfgSet() : -----
*               USBH_ERR_UNKNOWN                        Unknown error occurred.
*               USBH_ERR_INVALID_ARG                    Invalid argument passed to 'p_ep'.
*               USBH_ERR_EP_INVALID_STATE               Endpoint is not opened.
*               USBH_ERR_HC_IO,                         Root hub input/output error.
*               USBH_ERR_EP_STALL,                      Root hub does not support request.
*               Host controller drivers error code,     Otherwise.
*
*                                                       ----- RETURNED BY USBH_IF_Get() : -----
*               USBH_ERR_NULL_PTR                       If a null pointer was set for 'p_if'.
*               Host controller drivers error code,     Otherwise.
*
* Note(s)     : None.
*********************************************************************************************************
*/

USBH_ERR usbh_class_drv_conn(struct usbh_dev *p_dev)
{
    uint8_t if_ix;
    uint8_t nbr_if;
    bool drv_found;
    USBH_ERR err;
    struct usbh_cfg *p_cfg;
    struct usbh_if *p_if;

    LOG_DBG("ProbeDev");
    err = usbh_class_probe_dev(p_dev);
    if (err == USBH_ERR_NONE)
    {
        p_if = (struct usbh_if *)0;

        LOG_DBG("ClassNotify");
        USBH_ClassNotify(p_dev, /* Find a class drv matching dev desc.                  */
                         p_if,
                         p_dev->ClassDevPtr,
                         USBH_CLASS_DEV_STATE_CONN);
        return (USBH_ERR_NONE);
    }
    else if ((err != USBH_ERR_CLASS_DRV_NOT_FOUND) &&
             (err != USBH_ERR_CLASS_PROBE_FAIL))
    {

        LOG_ERR("ERROR: Probe class driver. #%d\r\n", err);
    }
    else
    {
        /* Empty Else Statement                                 */
    }

    LOG_DBG("CfgSet");
    err = usbh_cfg_set(p_dev, 1u); /* Select first cfg.                                    */
    if (err != USBH_ERR_NONE)
    {
        return (err);
    }

    drv_found = false;
    LOG_DBG("CfgGet");
    p_cfg = usbh_cfg_get(p_dev, (p_dev->SelCfg - 1)); /* Get active cfg struct.                               */
    LOG_DBG("CfgIF_NbrGet");
    nbr_if = usbh_cfg_if_nbr_get(p_cfg);

    for (if_ix = 0u; if_ix < nbr_if; if_ix++)
    { /* For all IFs present in cfg.                          */
        LOG_DBG("IF_Get");
        p_if = usbu_if_get(p_cfg, if_ix);
        if (p_if == (struct usbh_if *)0)
        {
            return (USBH_ERR_NULL_PTR);
        }
        LOG_DBG("ProbeIF");
        err = usbh_class_probe_if(p_dev, p_if); /* Find class driver matching IF.                       */
        if (err == USBH_ERR_NONE)
        {
            drv_found = true;
        }
        else if (err != USBH_ERR_CLASS_DRV_NOT_FOUND)
        {
            LOG_ERR("ERROR: Probe class driver. #%d\r\n", err);
        }
        else
        {
            /* Empty Else Statement                                 */
        }
    }
    if (drv_found == false)
    {
        LOG_ERR("No Class Driver Found.\r\n");
        return (err);
    }

    for (if_ix = 0u; if_ix < nbr_if; if_ix++)
    { /* For all IFs present in this cfg, notify app.         */
        LOG_DBG("IF_Get");

        p_if = usbu_if_get(p_cfg, if_ix);
        if (p_if == (struct usbh_if *)0)
        {
            return (USBH_ERR_NULL_PTR);
        }

        if (p_if->ClassDevPtr != 0)
        {
            LOG_INF("ClassNotify");

            USBH_ClassNotify(p_dev,
                             p_if,
                             p_if->ClassDevPtr,
                             USBH_CLASS_DEV_STATE_CONN);
        }
    }

    return (USBH_ERR_NONE);
}

/*
*********************************************************************************************************
*                                       USBH_ClassDrvDisconn()
*
* Description : Disconnect all the class drivers associated to the specified USB device.
*
* Argument(s) : p_dev   Pointer to USB device.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

void usbh_class_drv_disconn(struct usbh_dev *p_dev)
{
    LOG_DBG("disconnect class driver");
    uint8_t if_ix;
    uint8_t nbr_ifs;
    struct usbh_cfg *p_cfg;
    struct usbh_if *p_if = (struct usbh_if *)0;
    const struct usbh_class_drv *p_class_drv;

    if ((p_dev->ClassDevPtr != (void *)0) && /* If class drv is present at dev level.                */
        (p_dev->ClassDrvRegPtr != (struct usbh_class_drv_reg *)0))
    {
        p_class_drv = p_dev->ClassDrvRegPtr->ClassDrvPtr;

        if ((p_class_drv != 0) &&
            (p_class_drv->Disconn != 0))
        {
            LOG_DBG("notify class");
            USBH_ClassNotify(p_dev,
                             p_if,
                             p_dev->ClassDevPtr,
                             USBH_CLASS_DEV_STATE_DISCONN);

            p_class_drv->Disconn((void *)p_dev->ClassDevPtr);
        }

        p_dev->ClassDrvRegPtr = (struct usbh_class_drv_reg *)0;
        p_dev->ClassDevPtr = (void *)0;
        return;
    }

    p_cfg = usbh_cfg_get(p_dev, 0u); /* Get first cfg.                                       */
    nbr_ifs = usbh_cfg_if_nbr_get(p_cfg);
    for (if_ix = 0u; if_ix < nbr_ifs; if_ix++)
    { /* For all IFs present in first cfg.                    */
        p_if = usbu_if_get(p_cfg, if_ix);
        if (p_if == (struct usbh_if *)0)
        {
            return;
        }

        if ((p_if->ClassDevPtr != (void *)0) &&
            (p_if->ClassDrvRegPtr != (struct usbh_class_drv_reg *)0))
        {
            p_class_drv = p_if->ClassDrvRegPtr->ClassDrvPtr;

            if ((p_class_drv != 0) &&
                (p_class_drv->Disconn != 0))
            {
                LOG_DBG("notify class");
                USBH_ClassNotify(p_dev,
                                 p_if,
                                 p_if->ClassDevPtr,
                                 USBH_CLASS_DEV_STATE_DISCONN);

                /* Disconnect the class driver.                         */
                p_class_drv->Disconn((void *)p_if->ClassDevPtr);
            }

            p_if->ClassDrvRegPtr = (struct usbh_class_drv_reg *)0;
            p_if->ClassDevPtr = (void *)0;
        }
    }
}

/*
*********************************************************************************************************
*********************************************************************************************************
*                                            LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                         USBH_ClassProbeDev()
*
* Description : Find a class driver matching device descriptor of the USB device.
*
* Argument(s) : p_dev   Pointer to USB device.
*
* Return(s)   : USBH_ERR_NONE                   If class driver for this device was found.
*               USBH_ERR_CLASS_DRV_NOT_FOUND    If class driver for this device was not found.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static USBH_ERR usbh_class_probe_dev(struct usbh_dev *p_dev)
{
    uint32_t ix;
    const struct usbh_class_drv *p_class_drv;
    USBH_ERR err;
    void *p_class_dev;

    err = USBH_ERR_CLASS_DRV_NOT_FOUND;

    for (ix = 0u; ix < USBH_CFG_MAX_NBR_CLASS_DRVS; ix++)
    { /* For each class drv present in list.                  */

        if (usbh_class_drv_list[ix].InUse != 0)
        {
            p_class_drv = usbh_class_drv_list[ix].ClassDrvPtr;

            if (p_class_drv->ProbeDev != (void *)0)
            {
                p_dev->ClassDrvRegPtr = &usbh_class_drv_list[ix];
                p_class_dev = p_class_drv->ProbeDev(p_dev, &err);

                if (err == USBH_ERR_NONE)
                {
                    p_dev->ClassDevPtr = p_class_dev; /* Drv found, store class dev ptr.                      */
                    return (err);
                }
                p_dev->ClassDrvRegPtr = (struct usbh_class_drv_reg *)0;
            }
        }
    }

    return (err);
}

/*
*********************************************************************************************************
*                                         USBH_ClassProbeIF()
*
* Description : Finds a class driver matching interface descriptor of an interface.
*
* Argument(s) : p_dev   Pointer to USB device.
*
*               p_if    Pointer to USB interface.
*
* Return(s)   : USBH_ERR_NONE                   If class driver for this interface was found.
*               USBH_ERR_CLASS_DRV_NOT_FOUND    If class driver for this interface was not found.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static USBH_ERR usbh_class_probe_if(struct usbh_dev *p_dev,
                                  struct usbh_if *p_if)
{
    LOG_DBG("ClassProbeIF");
    uint32_t ix;
    const struct usbh_class_drv *p_class_drv;
    void *p_class_dev;
    USBH_ERR err;

    err = USBH_ERR_CLASS_DRV_NOT_FOUND;
    for (ix = 0u; ix < USBH_CFG_MAX_NBR_CLASS_DRVS; ix++)
    { /* Search drv list for matching IF class.
                 */
        if (usbh_class_drv_list[ix].InUse != 0u)
        {
            p_class_drv = usbh_class_drv_list[ix].ClassDrvPtr;

            if (p_class_drv->ProbeIF != (void *)0)
            {
                p_if->ClassDrvRegPtr = &usbh_class_drv_list[ix];
                p_class_dev = p_class_drv->ProbeIF(p_dev, p_if, &err);

                if (err == USBH_ERR_NONE)
                {
                    p_if->ClassDevPtr = p_class_dev; /* Drv found, store class dev ptr.                      */
                    return (err);
                }
                p_if->ClassDrvRegPtr = (struct usbh_class_drv_reg *)0;
            }
        }
    }

    return (err);
}

/*
*********************************************************************************************************
*                                          USBH_ClassNotify()
*
* Description : Notifies application about connection and disconnection events.
*
* Argument(s) : p_dev           Pointer to USB device.
*
*               p_if            Pointer to USB interface.
*
*               p_class_dev     Pointer to class device structure.
*
*               is_conn         Flag indicating connection or disconnection.
*
* Return(s)   : None.
*
* Note(s)     : None
*********************************************************************************************************
*/

static void USBH_ClassNotify(struct usbh_dev *p_dev,
                             struct usbh_if *p_if,
                             void *p_class_dev,
                             uint8_t is_conn)
{
    struct usbh_class_drv_reg *p_class_drv_reg;

    p_class_drv_reg = p_dev->ClassDrvRegPtr;

    if (p_class_drv_reg == (struct usbh_class_drv_reg *)0)
    {
        p_class_drv_reg = p_if->ClassDrvRegPtr;
    }

    if (p_class_drv_reg->NotifyFnctPtr != (USBH_CLASS_NOTIFY_FNCT)0)
    {
        p_class_drv_reg->NotifyFnctPtr(p_class_dev, /* Call app notification callback fnct.                 */
                                       is_conn,
                                       p_class_drv_reg->NotifyArgPtr);
    }
}
