use ::libc;
extern "C" {
    #[no_mangle]
    fn USBD_CtlContinueSendData(pdev: *mut USBD_HandleTypeDef,
                                pbuf: *mut uint8_t, len: uint16_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_CtlContinueRx(pdev: *mut USBD_HandleTypeDef, pbuf: *mut uint8_t,
                          len: uint16_t) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_CtlSendStatus(pdev: *mut USBD_HandleTypeDef)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_CtlReceiveStatus(pdev: *mut USBD_HandleTypeDef)
     -> USBD_StatusTypeDef;
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
    #[no_mangle]
    fn USBD_StdDevReq(pdev: *mut USBD_HandleTypeDef,
                      req: *mut USBD_SetupReqTypedef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_StdItfReq(pdev: *mut USBD_HandleTypeDef,
                      req: *mut USBD_SetupReqTypedef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_StdEPReq(pdev: *mut USBD_HandleTypeDef,
                     req: *mut USBD_SetupReqTypedef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_ParseSetupRequest(req: *mut USBD_SetupReqTypedef,
                              pdata: *mut uint8_t);
    /* USBD Low Level Driver */
    #[no_mangle]
    fn USBD_LL_Init(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_DeInit(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_Stop(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_Start(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_StallEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_PrepareReceive(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t,
                              pbuf: *mut uint8_t, size: uint16_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_OpenEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t,
                      ep_type: uint8_t, ep_mps: uint16_t)
     -> USBD_StatusTypeDef;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    usbd_def.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   General defines for the usb device library
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
/* * @addtogroup STM32_USBD_DEVICE_LIBRARY
  * @{
  */
/* * @defgroup USB_DEF
  * @brief general defines for the usb device library file
  * @{
  */
/* * @defgroup USB_DEF_Exported_Defines
  * @{
  */
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
  ******************************************************************************
  * @file    usbd_core.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the USBD core functions.
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
/* * @addtogroup STM32_USBD_DEVICE_LIBRARY
* @{
*/
/* * @defgroup USBD_CORE
* @brief usbd core module
* @{
*/
/* * @defgroup USBD_CORE_Private_TypesDefinitions
* @{
*/
/* *
* @}
*/
/* * @defgroup USBD_CORE_Private_Defines
* @{
*/
/* *
* @}
*/
/* * @defgroup USBD_CORE_Private_Macros
* @{
*/
/* *
* @}
*/
/* * @defgroup USBD_CORE_Private_FunctionPrototypes
* @{
*/
/* *
* @}
*/
/* * @defgroup USBD_CORE_Private_Variables
* @{
*/
/* *
* @}
*/
/* * @defgroup USBD_CORE_Private_Functions
* @{
*/
/* *
* @brief  USBD_Init
*         Initializes the device stack and load the class driver
* @param  pdev: device instance
* @param  pdesc: Descriptor structure address
* @param  id: Low level core index
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_Init(mut pdev: *mut USBD_HandleTypeDef,
                                   mut pdesc: *mut USBD_DescriptorsTypeDef,
                                   mut id: uint8_t) -> USBD_StatusTypeDef {
    /* Check whether the USB Host handle is valid */
    if pdev.is_null() { return USBD_FAIL }
    /* Unlink previous class*/
    if !(*pdev).pClass.is_null() {
        (*pdev).pClass = 0 as *mut USBD_ClassTypeDef
    }
    /* Assign USBD Descriptors */
    if !pdesc.is_null() { (*pdev).pDesc = pdesc }
    /* Set Device initial State */
    (*pdev).dev_state = 1 as libc::c_int as uint8_t;
    (*pdev).id = id;
    /* Initialize low level driver */
    USBD_LL_Init(pdev);
    return USBD_OK;
}
/* *
* @brief  USBD_DeInit
*         Re-Initialize th device library
* @param  pdev: device instance
* @retval status: status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_DeInit(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    /* Set Default State */
    (*pdev).dev_state = 1 as libc::c_int as uint8_t;
    /* Free Class Resources */
    (*(*pdev).pClass).DeInit.expect("non-null function pointer")(pdev,
                                                                 (*pdev).dev_config
                                                                     as
                                                                     uint8_t);
    /* Stop the low level driver  */
    USBD_LL_Stop(pdev);
    /* Initialize low level driver */
    USBD_LL_DeInit(pdev);
    return USBD_OK;
}
/* *
  * @brief  USBD_RegisterClass
  *         Link class driver to Device Core.
  * @param  pDevice : Device Handle
  * @param  pclass: Class handle
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_RegisterClass(mut pdev: *mut USBD_HandleTypeDef,
                                            mut pclass:
                                                *mut USBD_ClassTypeDef)
 -> USBD_StatusTypeDef {
    let mut status: USBD_StatusTypeDef = USBD_OK;
    if !pclass.is_null() {
        /* link the class to the USB Device handle */
        (*pdev).pClass = pclass;
        status = USBD_OK
    } else { status = USBD_FAIL }
    return status;
}
/* *
  * @brief  USBD_Start
  *         Start the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_Start(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    /* Start the low level driver  */
    USBD_LL_Start(pdev);
    return USBD_OK;
}
/* *
  * @brief  USBD_Stop
  *         Stop the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_Stop(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    /* Free Class Resources */
    (*(*pdev).pClass).DeInit.expect("non-null function pointer")(pdev,
                                                                 (*pdev).dev_config
                                                                     as
                                                                     uint8_t);
    /* Stop the low level driver  */
    USBD_LL_Stop(pdev);
    return USBD_OK;
}
/* *
* @brief  USBD_RunTestMode
*         Launch test mode process
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_RunTestMode(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    return USBD_OK;
}
/* *
* @brief  USBD_SetClassConfig
*        Configure device and start the interface
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_SetClassConfig(mut pdev:
                                                 *mut USBD_HandleTypeDef,
                                             mut cfgidx: uint8_t)
 -> USBD_StatusTypeDef {
    let mut ret: USBD_StatusTypeDef = USBD_FAIL;
    if !(*pdev).pClass.is_null() {
        /* Set configuration  and Start the Class*/
        if (*(*pdev).pClass).Init.expect("non-null function pointer")(pdev,
                                                                      cfgidx)
               as libc::c_int == 0 as libc::c_int {
            ret = USBD_OK
        }
    }
    return ret;
}
/* *
* @brief  USBD_ClrClassConfig
*         Clear current configuration
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status: USBD_StatusTypeDef
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_ClrClassConfig(mut pdev:
                                                 *mut USBD_HandleTypeDef,
                                             mut cfgidx: uint8_t)
 -> USBD_StatusTypeDef {
    /* Clear configuration  and De-initialize the Class process*/
    (*(*pdev).pClass).DeInit.expect("non-null function pointer")(pdev,
                                                                 cfgidx);
    return USBD_OK;
}
/* *
* @brief  USBD_SetupStage
*         Handle the setup stage
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_SetupStage(mut pdev: *mut USBD_HandleTypeDef,
                                            mut psetup: *mut uint8_t)
 -> USBD_StatusTypeDef {
    USBD_ParseSetupRequest(&mut (*pdev).request, psetup);
    (*pdev).ep0_state = 1 as libc::c_int as uint32_t;
    (*pdev).ep0_data_len = (*pdev).request.wLength as uint32_t;
    match (*pdev).request.bmRequest as libc::c_int & 0x1f as libc::c_int {
        0 => { USBD_StdDevReq(pdev, &mut (*pdev).request); }
        1 => { USBD_StdItfReq(pdev, &mut (*pdev).request); }
        2 => { USBD_StdEPReq(pdev, &mut (*pdev).request); }
        _ => {
            USBD_LL_StallEP(pdev,
                            ((*pdev).request.bmRequest as libc::c_int &
                                 0x80 as libc::c_int) as uint8_t);
        }
    }
    return USBD_OK;
}
/* *
* @brief  USBD_DataOutStage
*         Handle data OUT stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_DataOutStage(mut pdev:
                                                  *mut USBD_HandleTypeDef,
                                              mut epnum: uint8_t,
                                              mut pdata: *mut uint8_t)
 -> USBD_StatusTypeDef {
    let mut pep: *mut USBD_EndpointTypeDef = 0 as *mut USBD_EndpointTypeDef;
    if epnum as libc::c_int == 0 as libc::c_int {
        pep =
            &mut *(*pdev).ep_out.as_mut_ptr().offset(0 as libc::c_int as
                                                         isize) as
                *mut USBD_EndpointTypeDef;
        if (*pdev).ep0_state == 3 as libc::c_int as libc::c_uint {
            if (*pep).rem_length > (*pep).maxpacket {
                (*pep).rem_length =
                    ((*pep).rem_length as
                         libc::c_uint).wrapping_sub((*pep).maxpacket) as
                        uint32_t as uint32_t;
                USBD_CtlContinueRx(pdev, pdata,
                                   if (*pep).rem_length < (*pep).maxpacket {
                                       (*pep).rem_length
                                   } else { (*pep).maxpacket } as uint16_t);
            } else {
                if (*(*pdev).pClass).EP0_RxReady.is_some() &&
                       (*pdev).dev_state as libc::c_int == 3 as libc::c_int {
                    (*(*pdev).pClass).EP0_RxReady.expect("non-null function pointer")(pdev);
                }
                USBD_CtlSendStatus(pdev);
            }
        }
    } else if (*(*pdev).pClass).DataOut.is_some() &&
                  (*pdev).dev_state as libc::c_int == 3 as libc::c_int {
        (*(*pdev).pClass).DataOut.expect("non-null function pointer")(pdev,
                                                                      epnum);
    }
    return USBD_OK;
}
/* *
* @brief  USBD_DataInStage
*         Handle data in stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_DataInStage(mut pdev:
                                                 *mut USBD_HandleTypeDef,
                                             mut epnum: uint8_t,
                                             mut pdata: *mut uint8_t)
 -> USBD_StatusTypeDef {
    let mut pep: *mut USBD_EndpointTypeDef = 0 as *mut USBD_EndpointTypeDef;
    if epnum as libc::c_int == 0 as libc::c_int {
        pep =
            &mut *(*pdev).ep_in.as_mut_ptr().offset(0 as libc::c_int as isize)
                as *mut USBD_EndpointTypeDef;
        if (*pdev).ep0_state == 2 as libc::c_int as libc::c_uint {
            if (*pep).rem_length > (*pep).maxpacket {
                (*pep).rem_length =
                    ((*pep).rem_length as
                         libc::c_uint).wrapping_sub((*pep).maxpacket) as
                        uint32_t as uint32_t;
                USBD_CtlContinueSendData(pdev, pdata,
                                         (*pep).rem_length as uint16_t);
                /* Prepare endpoint for premature end of transfer */
                USBD_LL_PrepareReceive(pdev, 0 as libc::c_int as uint8_t,
                                       0 as *mut uint8_t,
                                       0 as libc::c_int as uint16_t);
            } else if (*pep).total_length.wrapping_rem((*pep).maxpacket) ==
                          0 as libc::c_int as libc::c_uint &&
                          (*pep).total_length >= (*pep).maxpacket &&
                          (*pep).total_length < (*pdev).ep0_data_len {
                USBD_CtlContinueSendData(pdev, 0 as *mut uint8_t,
                                         0 as libc::c_int as uint16_t);
                (*pdev).ep0_data_len = 0 as libc::c_int as uint32_t;
                /* last packet is MPS multiple, so send ZLP packet */
                /* Prepare endpoint for premature end of transfer */
                USBD_LL_PrepareReceive(pdev, 0 as libc::c_int as uint8_t,
                                       0 as *mut uint8_t,
                                       0 as libc::c_int as uint16_t);
            } else {
                if (*(*pdev).pClass).EP0_TxSent.is_some() &&
                       (*pdev).dev_state as libc::c_int == 3 as libc::c_int {
                    (*(*pdev).pClass).EP0_TxSent.expect("non-null function pointer")(pdev);
                }
                USBD_CtlReceiveStatus(pdev);
            }
        }
        if (*pdev).dev_test_mode as libc::c_int == 1 as libc::c_int {
            USBD_RunTestMode(pdev);
            (*pdev).dev_test_mode = 0 as libc::c_int as uint8_t
        }
    } else if (*(*pdev).pClass).DataIn.is_some() &&
                  (*pdev).dev_state as libc::c_int == 3 as libc::c_int {
        (*(*pdev).pClass).DataIn.expect("non-null function pointer")(pdev,
                                                                     epnum);
    }
    return USBD_OK;
}
/* *
* @brief  USBD_LL_Reset
*         Handle Reset event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_Reset(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    /* Open EP0 OUT */
    USBD_LL_OpenEP(pdev, 0 as libc::c_int as uint8_t,
                   0 as libc::c_int as uint8_t,
                   64 as libc::c_int as uint16_t);
    (*pdev).ep_out[0 as libc::c_int as usize].maxpacket =
        64 as libc::c_int as uint32_t;
    /* Open EP0 IN */
    USBD_LL_OpenEP(pdev, 0x80 as libc::c_int as uint8_t,
                   0 as libc::c_int as uint8_t,
                   64 as libc::c_int as uint16_t);
    (*pdev).ep_in[0 as libc::c_int as usize].maxpacket =
        64 as libc::c_int as uint32_t;
    /* Upon Reset call user call back */
    (*pdev).dev_state = 1 as libc::c_int as uint8_t;
    if !(*pdev).pCDC_ClassData.is_null() || !(*pdev).pHID_ClassData.is_null()
           || !(*pdev).pMSC_ClassData.is_null() {
        (*(*pdev).pClass).DeInit.expect("non-null function pointer")(pdev,
                                                                     (*pdev).dev_config
                                                                         as
                                                                         uint8_t);
    }
    return USBD_OK;
}
/* *
* @brief  USBD_LL_Reset
*         Handle Reset event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_SetSpeed(mut pdev: *mut USBD_HandleTypeDef,
                                          mut speed: USBD_SpeedTypeDef)
 -> USBD_StatusTypeDef {
    (*pdev).dev_speed = speed;
    return USBD_OK;
}
/* *
* @brief  USBD_Suspend
*         Handle Suspend event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_Suspend(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    (*pdev).dev_old_state = (*pdev).dev_state;
    (*pdev).dev_state = 4 as libc::c_int as uint8_t;
    return USBD_OK;
}
/* *
* @brief  USBD_Resume
*         Handle Resume event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_Resume(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    (*pdev).dev_state = (*pdev).dev_old_state;
    return USBD_OK;
}
/* *
* @brief  USBD_SOF
*         Handle SOF event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_SOF(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    if (*pdev).dev_state as libc::c_int == 3 as libc::c_int {
        if (*(*pdev).pClass).SOF.is_some() {
            (*(*pdev).pClass).SOF.expect("non-null function pointer")(pdev);
        }
    }
    return USBD_OK;
}
/* *
* @brief  USBD_IsoINIncomplete
*         Handle iso in incomplete event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_IsoINIncomplete(mut pdev:
                                                     *mut USBD_HandleTypeDef,
                                                 mut epnum: uint8_t)
 -> USBD_StatusTypeDef {
    return USBD_OK;
}
/* *
* @brief  USBD_IsoOUTIncomplete
*         Handle iso out incomplete event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_IsoOUTIncomplete(mut pdev:
                                                      *mut USBD_HandleTypeDef,
                                                  mut epnum: uint8_t)
 -> USBD_StatusTypeDef {
    return USBD_OK;
}
/* *
* @brief  USBD_DevConnected
*         Handle device connection event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_DevConnected(mut pdev:
                                                  *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    return USBD_OK;
}
/* *
  ******************************************************************************
  * @file    usbd_core.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header file for usbd_core.c file
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
/* * @defgroup USBD_CORE
  * @brief This file is the Header file for usbd_core.c file
  * @{
  */
/* * @defgroup USBD_CORE_Exported_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_FunctionsPrototype
  * @{
  */
/* *
* @brief  USBD_DevDisconnected
*         Handle device disconnection event
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_DevDisconnected(mut pdev:
                                                     *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    /* Free Class Resources */
    (*pdev).dev_state = 1 as libc::c_int as uint8_t;
    (*(*pdev).pClass).DeInit.expect("non-null function pointer")(pdev,
                                                                 (*pdev).dev_config
                                                                     as
                                                                     uint8_t);
    return USBD_OK;
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
