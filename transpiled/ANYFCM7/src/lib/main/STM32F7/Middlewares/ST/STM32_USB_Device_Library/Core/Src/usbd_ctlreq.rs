use ::libc;
extern "C" {
    #[no_mangle]
    fn USBD_SetClassConfig(pdev: *mut USBD_HandleTypeDef, cfgidx: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_ClrClassConfig(pdev: *mut USBD_HandleTypeDef, cfgidx: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_StallEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_ClearStallEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_IsStallEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
     -> uint8_t;
    /* *
  ******************************************************************************
  * @file    usbd_ioreq.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header file for the usbd_ioreq.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
    /* Define to prevent recursive inclusion -------------------------------------*/
    /* Includes ------------------------------------------------------------------*/
    /* * @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
    /* * @defgroup USBD_IOREQ
  * @brief header file for the usbd_ioreq.c file
  * @{
  */
    /* * @defgroup USBD_IOREQ_Exported_Defines
  * @{
  */ 
/* *
  * @}
  */
    /* * @defgroup USBD_IOREQ_Exported_Types
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USBD_IOREQ_Exported_Macros
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USBD_IOREQ_Exported_Variables
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USBD_IOREQ_Exported_FunctionsPrototype
  * @{
  */
    #[no_mangle]
    fn USBD_CtlSendStatus(pdev: *mut USBD_HandleTypeDef)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_SetUSBAddress(pdev: *mut USBD_HandleTypeDef, dev_addr: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_CtlSendData(pdev: *mut USBD_HandleTypeDef, buf: *mut uint8_t,
                        len: uint16_t) -> USBD_StatusTypeDef;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/*  Device Status */
/*  EP0 State */
/* *
  * @}
  */
/* * @defgroup USBD_DEF_Exported_TypesDefinitions
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usb_setup_req {
    pub bmRequest: uint8_t,
    pub bRequest: uint8_t,
    pub wValue: uint16_t,
    pub wIndex: uint16_t,
    pub wLength: uint16_t,
}
pub type USBD_SetupReqTypedef = usb_setup_req;
/* Control Endpoints*/
/* Class Specific Endpoints*/
/* Following USB Device Speed */
/* Following USB Device status */
/* USB Device descriptors structure */
/* USB Device handle structure */
/* USB Device handle structure */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USBD_HandleTypeDef {
    pub id: uint8_t,
    pub dev_config: uint32_t,
    pub dev_default_config: uint32_t,
    pub dev_config_status: uint32_t,
    pub dev_speed: USBD_SpeedTypeDef,
    pub ep_in: [USBD_EndpointTypeDef; 15],
    pub ep_out: [USBD_EndpointTypeDef; 15],
    pub ep0_state: uint32_t,
    pub ep0_data_len: uint32_t,
    pub dev_state: uint8_t,
    pub dev_old_state: uint8_t,
    pub dev_address: uint8_t,
    pub dev_connection_status: uint8_t,
    pub dev_test_mode: uint8_t,
    pub dev_remote_wakeup: uint32_t,
    pub request: USBD_SetupReqTypedef,
    pub pDesc: *mut USBD_DescriptorsTypeDef,
    pub pClass: *mut USBD_ClassTypeDef,
    pub pCDC_ClassData: *mut libc::c_void,
    pub pCDC_UserData: *mut libc::c_void,
    pub pHID_ClassData: *mut libc::c_void,
    pub pHID_UserData: *mut libc::c_void,
    pub pMSC_ClassData: *mut libc::c_void,
    pub pMSC_UserData: *mut libc::c_void,
    pub pData: *mut libc::c_void,
}
pub type USBD_ClassTypeDef = _Device_cb;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _Device_cb {
    pub Init: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                          _: uint8_t) -> uint8_t>,
    pub DeInit: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                            _: uint8_t) -> uint8_t>,
    pub Setup: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                           _: *mut USBD_SetupReqTypedef)
                          -> uint8_t>,
    pub EP0_TxSent: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                               -> uint8_t>,
    pub EP0_RxReady: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                                -> uint8_t>,
    pub DataIn: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                            _: uint8_t) -> uint8_t>,
    pub DataOut: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                             _: uint8_t) -> uint8_t>,
    pub SOF: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                        -> uint8_t>,
    pub IsoINIncomplete: Option<unsafe extern "C" fn(_:
                                                         *mut _USBD_HandleTypeDef,
                                                     _: uint8_t) -> uint8_t>,
    pub IsoOUTIncomplete: Option<unsafe extern "C" fn(_:
                                                          *mut _USBD_HandleTypeDef,
                                                      _: uint8_t) -> uint8_t>,
    pub GetHSConfigDescriptor: Option<unsafe extern "C" fn(_: *mut uint16_t)
                                          -> *mut uint8_t>,
    pub GetFSConfigDescriptor: Option<unsafe extern "C" fn(_: *mut uint16_t)
                                          -> *mut uint8_t>,
    pub GetOtherSpeedConfigDescriptor: Option<unsafe extern "C" fn(_:
                                                                       *mut uint16_t)
                                                  -> *mut uint8_t>,
    pub GetDeviceQualifierDescriptor: Option<unsafe extern "C" fn(_:
                                                                      *mut uint16_t)
                                                 -> *mut uint8_t>,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_DescriptorsTypeDef {
    pub GetDeviceDescriptor: Option<unsafe extern "C" fn(_: USBD_SpeedTypeDef,
                                                         _: *mut uint16_t)
                                        -> *mut uint8_t>,
    pub GetLangIDStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                USBD_SpeedTypeDef,
                                                            _: *mut uint16_t)
                                           -> *mut uint8_t>,
    pub GetManufacturerStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                      USBD_SpeedTypeDef,
                                                                  _:
                                                                      *mut uint16_t)
                                                 -> *mut uint8_t>,
    pub GetProductStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                 USBD_SpeedTypeDef,
                                                             _: *mut uint16_t)
                                            -> *mut uint8_t>,
    pub GetSerialStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                USBD_SpeedTypeDef,
                                                            _: *mut uint16_t)
                                           -> *mut uint8_t>,
    pub GetConfigurationStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                       USBD_SpeedTypeDef,
                                                                   _:
                                                                       *mut uint16_t)
                                                  -> *mut uint8_t>,
    pub GetInterfaceStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                   USBD_SpeedTypeDef,
                                                               _:
                                                                   *mut uint16_t)
                                              -> *mut uint8_t>,
}
pub type USBD_SpeedTypeDef = libc::c_uint;
pub const USBD_SPEED_LOW: USBD_SpeedTypeDef = 2;
pub const USBD_SPEED_FULL: USBD_SpeedTypeDef = 1;
pub const USBD_SPEED_HIGH: USBD_SpeedTypeDef = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_EndpointTypeDef {
    pub status: uint32_t,
    pub total_length: uint32_t,
    pub rem_length: uint32_t,
    pub maxpacket: uint32_t,
}
pub type USBD_StatusTypeDef = libc::c_uint;
pub const USBD_FAIL: USBD_StatusTypeDef = 2;
pub const USBD_BUSY: USBD_StatusTypeDef = 1;
pub const USBD_OK: USBD_StatusTypeDef = 0;
pub type USBD_HandleTypeDef = _USBD_HandleTypeDef;
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Private_Functions
  * @{
  */
/* *
* @brief  USBD_StdDevReq
*         Handle standard usb device requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_StdDevReq(mut pdev: *mut USBD_HandleTypeDef,
                                        mut req: *mut USBD_SetupReqTypedef)
 -> USBD_StatusTypeDef {
    let mut ret: USBD_StatusTypeDef = USBD_OK;
    match (*req).bRequest as libc::c_int {
        6 => { USBD_GetDescriptor(pdev, req); }
        5 => { USBD_SetAddress(pdev, req); }
        9 => { USBD_SetConfig(pdev, req); }
        8 => { USBD_GetConfig(pdev, req); }
        0 => { USBD_GetStatus(pdev, req); }
        3 => { USBD_SetFeature(pdev, req); }
        1 => { USBD_ClrFeature(pdev, req); }
        _ => { USBD_CtlError(pdev, req); }
    }
    return ret;
}
/* *
* @brief  USBD_StdItfReq
*         Handle standard usb interface requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_StdItfReq(mut pdev: *mut USBD_HandleTypeDef,
                                        mut req: *mut USBD_SetupReqTypedef)
 -> USBD_StatusTypeDef {
    let mut ret: USBD_StatusTypeDef = USBD_OK;
    match (*pdev).dev_state as libc::c_int {
        3 => {
            if ((*req).wIndex as libc::c_int & 0xff as libc::c_int) as uint8_t
                   as libc::c_int <= 3 as libc::c_int {
                (*(*pdev).pClass).Setup.expect("non-null function pointer")(pdev,
                                                                            req);
                if (*req).wLength as libc::c_int == 0 as libc::c_int &&
                       ret as libc::c_uint ==
                           USBD_OK as libc::c_int as libc::c_uint {
                    USBD_CtlSendStatus(pdev);
                }
            } else { USBD_CtlError(pdev, req); }
        }
        _ => { USBD_CtlError(pdev, req); }
    }
    return USBD_OK;
}
/* *
* @brief  USBD_StdEPReq
*         Handle standard usb endpoint requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_StdEPReq(mut pdev: *mut USBD_HandleTypeDef,
                                       mut req: *mut USBD_SetupReqTypedef)
 -> USBD_StatusTypeDef {
    let mut ep_addr: uint8_t = 0;
    let mut ret: USBD_StatusTypeDef = USBD_OK;
    let mut pep: *mut USBD_EndpointTypeDef = 0 as *mut USBD_EndpointTypeDef;
    ep_addr = ((*req).wIndex as libc::c_int & 0xff as libc::c_int) as uint8_t;
    /* Check if it is a class request */
    if (*req).bmRequest as libc::c_int & 0x60 as libc::c_int ==
           0x20 as libc::c_int {
        (*(*pdev).pClass).Setup.expect("non-null function pointer")(pdev,
                                                                    req);
        return USBD_OK
    }
    match (*req).bRequest as libc::c_int {
        3 => {
            match (*pdev).dev_state as libc::c_int {
                2 => {
                    if ep_addr as libc::c_int != 0 as libc::c_int &&
                           ep_addr as libc::c_int != 0x80 as libc::c_int {
                        USBD_LL_StallEP(pdev, ep_addr);
                    }
                }
                3 => {
                    if (*req).wValue as libc::c_int == 0 as libc::c_int {
                        if ep_addr as libc::c_int != 0 as libc::c_int &&
                               ep_addr as libc::c_int != 0x80 as libc::c_int {
                            USBD_LL_StallEP(pdev, ep_addr);
                        }
                    }
                    (*(*pdev).pClass).Setup.expect("non-null function pointer")(pdev,
                                                                                req);
                    USBD_CtlSendStatus(pdev);
                }
                _ => { USBD_CtlError(pdev, req); }
            }
        }
        1 => {
            match (*pdev).dev_state as libc::c_int {
                2 => {
                    if ep_addr as libc::c_int != 0 as libc::c_int &&
                           ep_addr as libc::c_int != 0x80 as libc::c_int {
                        USBD_LL_StallEP(pdev, ep_addr);
                    }
                }
                3 => {
                    if (*req).wValue as libc::c_int == 0 as libc::c_int {
                        if ep_addr as libc::c_int & 0x7f as libc::c_int !=
                               0 as libc::c_int {
                            USBD_LL_ClearStallEP(pdev, ep_addr);
                            (*(*pdev).pClass).Setup.expect("non-null function pointer")(pdev,
                                                                                        req);
                        }
                        USBD_CtlSendStatus(pdev);
                    }
                }
                _ => { USBD_CtlError(pdev, req); }
            }
        }
        0 => {
            match (*pdev).dev_state as libc::c_int {
                2 => {
                    if ep_addr as libc::c_int & 0x7f as libc::c_int !=
                           0 as libc::c_int {
                        USBD_LL_StallEP(pdev, ep_addr);
                    }
                }
                3 => {
                    pep =
                        if ep_addr as libc::c_int & 0x80 as libc::c_int ==
                               0x80 as libc::c_int {
                            &mut *(*pdev).ep_in.as_mut_ptr().offset((ep_addr
                                                                         as
                                                                         libc::c_int
                                                                         &
                                                                         0x7f
                                                                             as
                                                                             libc::c_int)
                                                                        as
                                                                        isize)
                                as *mut USBD_EndpointTypeDef
                        } else {
                            &mut *(*pdev).ep_out.as_mut_ptr().offset((ep_addr
                                                                          as
                                                                          libc::c_int
                                                                          &
                                                                          0x7f
                                                                              as
                                                                              libc::c_int)
                                                                         as
                                                                         isize)
                                as *mut USBD_EndpointTypeDef
                        };
                    if USBD_LL_IsStallEP(pdev, ep_addr) != 0 {
                        (*pep).status = 0x1 as libc::c_int as uint32_t
                    } else { (*pep).status = 0 as libc::c_int as uint32_t }
                    USBD_CtlSendData(pdev,
                                     &mut (*pep).status as *mut uint32_t as
                                         *mut uint8_t,
                                     2 as libc::c_int as uint16_t);
                }
                _ => { USBD_CtlError(pdev, req); }
            }
        }
        _ => { }
    }
    return ret;
}
/* *
  ******************************************************************************
  * @file    usbd_req.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015 
  * @brief   This file provides the standard USB requests following chapter 9.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32_USBD_STATE_DEVICE_LIBRARY
  * @{
  */
/* * @defgroup USBD_REQ 
  * @brief USB standard requests module
  * @{
  */
/* * @defgroup USBD_REQ_Private_TypesDefinitions
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Private_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Private_Macros
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Private_Variables
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Private_FunctionPrototypes
  * @{
  */
/* *
* @brief  USBD_GetDescriptor
*         Handle Get Descriptor requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
unsafe extern "C" fn USBD_GetDescriptor(mut pdev: *mut USBD_HandleTypeDef,
                                        mut req: *mut USBD_SetupReqTypedef) {
    let mut len: uint16_t = 0;
    let mut pbuf: *mut uint8_t = 0 as *mut uint8_t;
    match (*req).wValue as libc::c_int >> 8 as libc::c_int {
        1 => {
            pbuf =
                (*(*pdev).pDesc).GetDeviceDescriptor.expect("non-null function pointer")((*pdev).dev_speed,
                                                                                         &mut len)
        }
        2 => {
            if (*pdev).dev_speed as libc::c_uint ==
                   USBD_SPEED_HIGH as libc::c_int as libc::c_uint {
                pbuf =
                    (*(*pdev).pClass).GetHSConfigDescriptor.expect("non-null function pointer")(&mut len);
                *pbuf.offset(1 as libc::c_int as isize) =
                    2 as libc::c_int as uint8_t
            } else {
                pbuf =
                    (*(*pdev).pClass).GetFSConfigDescriptor.expect("non-null function pointer")(&mut len);
                *pbuf.offset(1 as libc::c_int as isize) =
                    2 as libc::c_int as uint8_t
            }
        }
        3 => {
            match (*req).wValue as uint8_t as libc::c_int {
                0 => {
                    pbuf =
                        (*(*pdev).pDesc).GetLangIDStrDescriptor.expect("non-null function pointer")((*pdev).dev_speed,
                                                                                                    &mut len)
                }
                1 => {
                    pbuf =
                        (*(*pdev).pDesc).GetManufacturerStrDescriptor.expect("non-null function pointer")((*pdev).dev_speed,
                                                                                                          &mut len)
                }
                2 => {
                    pbuf =
                        (*(*pdev).pDesc).GetProductStrDescriptor.expect("non-null function pointer")((*pdev).dev_speed,
                                                                                                     &mut len)
                }
                3 => {
                    pbuf =
                        (*(*pdev).pDesc).GetSerialStrDescriptor.expect("non-null function pointer")((*pdev).dev_speed,
                                                                                                    &mut len)
                }
                4 => {
                    pbuf =
                        (*(*pdev).pDesc).GetConfigurationStrDescriptor.expect("non-null function pointer")((*pdev).dev_speed,
                                                                                                           &mut len)
                }
                5 => {
                    pbuf =
                        (*(*pdev).pDesc).GetInterfaceStrDescriptor.expect("non-null function pointer")((*pdev).dev_speed,
                                                                                                       &mut len)
                }
                _ => { USBD_CtlError(pdev, req); return }
            }
        }
        6 => {
            if (*pdev).dev_speed as libc::c_uint ==
                   USBD_SPEED_HIGH as libc::c_int as libc::c_uint {
                pbuf =
                    (*(*pdev).pClass).GetDeviceQualifierDescriptor.expect("non-null function pointer")(&mut len)
            } else { USBD_CtlError(pdev, req); return }
        }
        7 => {
            if (*pdev).dev_speed as libc::c_uint ==
                   USBD_SPEED_HIGH as libc::c_int as libc::c_uint {
                pbuf =
                    (*(*pdev).pClass).GetOtherSpeedConfigDescriptor.expect("non-null function pointer")(&mut len);
                *pbuf.offset(1 as libc::c_int as isize) =
                    7 as libc::c_int as uint8_t
            } else { USBD_CtlError(pdev, req); return }
        }
        _ => { USBD_CtlError(pdev, req); return }
    }
    if len as libc::c_int != 0 as libc::c_int &&
           (*req).wLength as libc::c_int != 0 as libc::c_int {
        len =
            if (len as libc::c_int) < (*req).wLength as libc::c_int {
                len as libc::c_int
            } else { (*req).wLength as libc::c_int } as uint16_t;
        USBD_CtlSendData(pdev, pbuf, len);
    };
}
/* *
* @brief  USBD_SetAddress
*         Set device address
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
unsafe extern "C" fn USBD_SetAddress(mut pdev: *mut USBD_HandleTypeDef,
                                     mut req: *mut USBD_SetupReqTypedef) {
    let mut dev_addr: uint8_t = 0;
    if (*req).wIndex as libc::c_int == 0 as libc::c_int &&
           (*req).wLength as libc::c_int == 0 as libc::c_int {
        dev_addr =
            ((*req).wValue as uint8_t as libc::c_int & 0x7f as libc::c_int) as
                uint8_t;
        if (*pdev).dev_state as libc::c_int == 3 as libc::c_int {
            USBD_CtlError(pdev, req);
        } else {
            (*pdev).dev_address = dev_addr;
            USBD_LL_SetUSBAddress(pdev, dev_addr);
            USBD_CtlSendStatus(pdev);
            if dev_addr as libc::c_int != 0 as libc::c_int {
                (*pdev).dev_state = 2 as libc::c_int as uint8_t
            } else { (*pdev).dev_state = 1 as libc::c_int as uint8_t }
        }
    } else { USBD_CtlError(pdev, req); };
}
/* *
* @brief  USBD_SetConfig
*         Handle Set device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
unsafe extern "C" fn USBD_SetConfig(mut pdev: *mut USBD_HandleTypeDef,
                                    mut req: *mut USBD_SetupReqTypedef) {
    static mut cfgidx: uint8_t = 0;
    cfgidx = (*req).wValue as uint8_t;
    if cfgidx as libc::c_int > 1 as libc::c_int {
        USBD_CtlError(pdev, req);
    } else {
        match (*pdev).dev_state as libc::c_int {
            2 => {
                if cfgidx != 0 {
                    (*pdev).dev_config = cfgidx as uint32_t;
                    (*pdev).dev_state = 3 as libc::c_int as uint8_t;
                    if USBD_SetClassConfig(pdev, cfgidx) as libc::c_uint ==
                           USBD_FAIL as libc::c_int as libc::c_uint {
                        USBD_CtlError(pdev, req);
                        return
                    }
                    USBD_CtlSendStatus(pdev);
                } else { USBD_CtlSendStatus(pdev); }
            }
            3 => {
                if cfgidx as libc::c_int == 0 as libc::c_int {
                    (*pdev).dev_state = 2 as libc::c_int as uint8_t;
                    (*pdev).dev_config = cfgidx as uint32_t;
                    USBD_ClrClassConfig(pdev, cfgidx);
                    USBD_CtlSendStatus(pdev);
                } else if cfgidx as libc::c_uint != (*pdev).dev_config {
                    /* Clear old configuration */
                    USBD_ClrClassConfig(pdev, (*pdev).dev_config as uint8_t);
                    /* set new configuration */
                    (*pdev).dev_config = cfgidx as uint32_t;
                    if USBD_SetClassConfig(pdev, cfgidx) as libc::c_uint ==
                           USBD_FAIL as libc::c_int as libc::c_uint {
                        USBD_CtlError(pdev, req);
                        return
                    }
                    USBD_CtlSendStatus(pdev);
                } else { USBD_CtlSendStatus(pdev); }
            }
            _ => { USBD_CtlError(pdev, req); }
        }
    };
}
/* *
* @brief  USBD_GetConfig
*         Handle Get device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
unsafe extern "C" fn USBD_GetConfig(mut pdev: *mut USBD_HandleTypeDef,
                                    mut req: *mut USBD_SetupReqTypedef) {
    if (*req).wLength as libc::c_int != 1 as libc::c_int {
        USBD_CtlError(pdev, req);
    } else {
        match (*pdev).dev_state as libc::c_int {
            2 => {
                (*pdev).dev_default_config = 0 as libc::c_int as uint32_t;
                USBD_CtlSendData(pdev,
                                 &mut (*pdev).dev_default_config as
                                     *mut uint32_t as *mut uint8_t,
                                 1 as libc::c_int as uint16_t);
            }
            3 => {
                USBD_CtlSendData(pdev,
                                 &mut (*pdev).dev_config as *mut uint32_t as
                                     *mut uint8_t,
                                 1 as libc::c_int as uint16_t);
            }
            _ => { USBD_CtlError(pdev, req); }
        }
    };
}
/* *
* @brief  USBD_GetStatus
*         Handle Get Status request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
unsafe extern "C" fn USBD_GetStatus(mut pdev: *mut USBD_HandleTypeDef,
                                    mut req: *mut USBD_SetupReqTypedef) {
    match (*pdev).dev_state as libc::c_int {
        2 | 3 => {
            (*pdev).dev_config_status = 1 as libc::c_int as uint32_t;
            if (*pdev).dev_remote_wakeup != 0 {
                (*pdev).dev_config_status |= 2 as libc::c_int as libc::c_uint
            }
            USBD_CtlSendData(pdev,
                             &mut (*pdev).dev_config_status as *mut uint32_t
                                 as *mut uint8_t,
                             2 as libc::c_int as uint16_t);
        }
        _ => { USBD_CtlError(pdev, req); }
    };
}
/* *
* @brief  USBD_SetFeature
*         Handle Set device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
unsafe extern "C" fn USBD_SetFeature(mut pdev: *mut USBD_HandleTypeDef,
                                     mut req: *mut USBD_SetupReqTypedef) {
    if (*req).wValue as libc::c_int == 1 as libc::c_int {
        (*pdev).dev_remote_wakeup = 1 as libc::c_int as uint32_t;
        (*(*pdev).pClass).Setup.expect("non-null function pointer")(pdev,
                                                                    req);
        USBD_CtlSendStatus(pdev);
    };
}
/* *
* @brief  USBD_ClrFeature
*         Handle clear device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
unsafe extern "C" fn USBD_ClrFeature(mut pdev: *mut USBD_HandleTypeDef,
                                     mut req: *mut USBD_SetupReqTypedef) {
    match (*pdev).dev_state as libc::c_int {
        2 | 3 => {
            if (*req).wValue as libc::c_int == 1 as libc::c_int {
                (*pdev).dev_remote_wakeup = 0 as libc::c_int as uint32_t;
                (*(*pdev).pClass).Setup.expect("non-null function pointer")(pdev,
                                                                            req);
                USBD_CtlSendStatus(pdev);
            }
        }
        _ => { USBD_CtlError(pdev, req); }
    };
}
/* *
* @brief  USBD_ParseSetupRequest 
*         Copy buffer into setup structure
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_ParseSetupRequest(mut req:
                                                    *mut USBD_SetupReqTypedef,
                                                mut pdata: *mut uint8_t) {
    (*req).bmRequest = *pdata;
    (*req).bRequest = *pdata.offset(1 as libc::c_int as isize);
    (*req).wValue =
        (*pdata.offset(2 as libc::c_int as isize) as uint16_t as libc::c_int +
             ((*pdata.offset(2 as libc::c_int as
                                 isize).offset(1 as libc::c_int as isize) as
                   uint16_t as libc::c_int) << 8 as libc::c_int)) as uint16_t;
    (*req).wIndex =
        (*pdata.offset(4 as libc::c_int as isize) as uint16_t as libc::c_int +
             ((*pdata.offset(4 as libc::c_int as
                                 isize).offset(1 as libc::c_int as isize) as
                   uint16_t as libc::c_int) << 8 as libc::c_int)) as uint16_t;
    (*req).wLength =
        (*pdata.offset(6 as libc::c_int as isize) as uint16_t as libc::c_int +
             ((*pdata.offset(6 as libc::c_int as
                                 isize).offset(1 as libc::c_int as isize) as
                   uint16_t as libc::c_int) << 8 as libc::c_int)) as uint16_t;
}
/* *
* @brief  USBD_CtlError 
*         Handle USB low level Error
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_CtlError(mut pdev: *mut USBD_HandleTypeDef,
                                       mut req: *mut USBD_SetupReqTypedef) {
    USBD_LL_StallEP(pdev, 0x80 as libc::c_int as uint8_t);
    USBD_LL_StallEP(pdev, 0 as libc::c_int as uint8_t);
}
/* *
  ******************************************************************************
  * @file    usbd_req.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header file for the usbd_req.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
/* * @defgroup USBD_REQ
  * @brief header file for the usbd_req.c file
  * @{
  */
/* * @defgroup USBD_REQ_Exported_Defines
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Exported_Types
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Exported_Macros
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Exported_Variables
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_REQ_Exported_FunctionsPrototype
  * @{
  */
/* *
  * @brief  USBD_GetString
  *         Convert Ascii string into unicode one
  * @param  desc : descriptor buffer
  * @param  unicode : Formatted string buffer (unicode)
  * @param  len : descriptor length
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_GetString(mut desc: *mut uint8_t,
                                        mut unicode: *mut uint8_t,
                                        mut len: *mut uint16_t) {
    let mut idx: uint8_t = 0 as libc::c_int as uint8_t;
    if !desc.is_null() {
        *len =
            (USBD_GetLen(desc) as libc::c_int * 2 as libc::c_int +
                 2 as libc::c_int) as uint16_t;
        let fresh0 = idx;
        idx = idx.wrapping_add(1);
        *unicode.offset(fresh0 as isize) = *len as uint8_t;
        let fresh1 = idx;
        idx = idx.wrapping_add(1);
        *unicode.offset(fresh1 as isize) = 3 as libc::c_int as uint8_t;
        while *desc as libc::c_int != '\u{0}' as i32 {
            let fresh2 = desc;
            desc = desc.offset(1);
            let fresh3 = idx;
            idx = idx.wrapping_add(1);
            *unicode.offset(fresh3 as isize) = *fresh2;
            let fresh4 = idx;
            idx = idx.wrapping_add(1);
            *unicode.offset(fresh4 as isize) = 0 as libc::c_int as uint8_t
        }
    };
}
/* *
  * @brief  USBD_GetLen
  *         return the string length
   * @param  buf : pointer to the ascii string buffer
  * @retval string length
  */
unsafe extern "C" fn USBD_GetLen(mut buf: *mut uint8_t) -> uint8_t {
    let mut len: uint8_t = 0 as libc::c_int as uint8_t;
    while *buf as libc::c_int != '\u{0}' as i32 {
        len = len.wrapping_add(1);
        buf = buf.offset(1)
    }
    return len;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
