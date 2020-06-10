use ::libc;
extern "C" {
    #[no_mangle]
    fn malloc(_: libc::c_ulong) -> *mut libc::c_void;
    #[no_mangle]
    fn free(__ptr: *mut libc::c_void);
    #[no_mangle]
    fn USBD_CtlError(pdev: *mut USBD_HandleTypeDef,
                     req: *mut USBD_SetupReqTypedef);
    #[no_mangle]
    fn USBD_LL_OpenEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t,
                      ep_type: uint8_t, ep_mps: uint16_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_CloseEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_Transmit(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t,
                        pbuf: *mut uint8_t, size: uint16_t)
     -> USBD_StatusTypeDef;
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
    fn USBD_CtlSendData(pdev: *mut USBD_HandleTypeDef, buf: *mut uint8_t,
                        len: uint16_t) -> USBD_StatusTypeDef;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub type HID_StateTypeDef = libc::c_uint;
pub const HID_BUSY: HID_StateTypeDef = 1;
pub const HID_IDLE: HID_StateTypeDef = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_HID_HandleTypeDef {
    pub Protocol: uint32_t,
    pub IdleState: uint32_t,
    pub AltSetting: uint32_t,
    pub state: HID_StateTypeDef,
}
/* *
  * @}
  */
/* * @defgroup USBD_HID_Private_Variables
  * @{
  */
#[no_mangle]
pub static mut USBD_HID: USBD_ClassTypeDef =
    unsafe {
        {
            let mut init =
                _Device_cb{Init:
                               Some(USBD_HID_Init as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           DeInit:
                               Some(USBD_HID_DeInit as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           Setup:
                               Some(USBD_HID_Setup as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _:
                                                                 *mut USBD_SetupReqTypedef)
                                            -> uint8_t),
                           EP0_TxSent: None,
                           EP0_RxReady: None,
                           DataIn:
                               Some(USBD_HID_DataIn as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           DataOut: None,
                           SOF: None,
                           IsoINIncomplete: None,
                           IsoOUTIncomplete: None,
                           GetHSConfigDescriptor:
                               Some(USBD_HID_GetCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetFSConfigDescriptor:
                               Some(USBD_HID_GetCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetOtherSpeedConfigDescriptor:
                               Some(USBD_HID_GetCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetDeviceQualifierDescriptor:
                               Some(USBD_HID_GetDeviceQualifierDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),};
            init
        }
    };
/* USB HID device Configuration Descriptor */
static mut USBD_HID_CfgDesc: [uint8_t; 34] =
    [0x9 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     34 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xe0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x21 as libc::c_int as uint8_t,
     0x11 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x22 as libc::c_int as uint8_t, 38 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x83 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t];
/* USB HID device Configuration Descriptor */
static mut USBD_HID_Desc: [uint8_t; 9] =
    [0x9 as libc::c_int as uint8_t, 0x21 as libc::c_int as uint8_t,
     0x11 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x22 as libc::c_int as uint8_t, 38 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t];
/* USB Standard Device Descriptor */
static mut USBD_HID_DeviceQualifierDesc: [uint8_t; 10] =
    [0xa as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
static mut HID_MOUSE_ReportDesc: [uint8_t; 38] =
    [0x5 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0xa1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0xa1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x30 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x31 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x35 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x34 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x38 as libc::c_int as uint8_t,
     0x15 as libc::c_int as uint8_t, 0x81 as libc::c_int as uint8_t,
     0x25 as libc::c_int as uint8_t, 0x7f as libc::c_int as uint8_t,
     0x75 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
     0x95 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
     0x81 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0xc0 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t];
/* *
  ******************************************************************************
  * @file    usbd_hid.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                HID Class  Description
  *          ===================================================================
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse protocol
  *             - Usage Page : Generic Desktop
  *             - Usage : Joystick
  *             - Collection : Application
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
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
/* * @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
/* * @defgroup USBD_HID
  * @brief usbd core module
  * @{
  */
/* * @defgroup USBD_HID_Private_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_HID_Private_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_HID_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_HID_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_HID_Private_Functions
  * @{
  */
/* *
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
unsafe extern "C" fn USBD_HID_Init(mut pdev: *mut USBD_HandleTypeDef,
                                   mut cfgidx: uint8_t) -> uint8_t {
    let mut ret: uint8_t = 0 as libc::c_int as uint8_t;
    /* Open EP IN */
    USBD_LL_OpenEP(pdev, 0x83 as libc::c_int as uint8_t,
                   3 as libc::c_int as uint8_t,
                   0x8 as libc::c_int as uint16_t);
    (*pdev).pHID_ClassData =
        malloc(::core::mem::size_of::<USBD_HID_HandleTypeDef>() as
                   libc::c_ulong);
    if (*pdev).pHID_ClassData.is_null() {
        ret = 1 as libc::c_int as uint8_t
    } else {
        (*((*pdev).pHID_ClassData as *mut USBD_HID_HandleTypeDef)).state =
            HID_IDLE
    }
    return ret;
}
/* *
  * @brief  USBD_HID_Init
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
unsafe extern "C" fn USBD_HID_DeInit(mut pdev: *mut USBD_HandleTypeDef,
                                     mut cfgidx: uint8_t) -> uint8_t {
    /* Close HID EPs */
    USBD_LL_CloseEP(pdev, 0x83 as libc::c_int as uint8_t);
    /* FRee allocated memory */
    if !(*pdev).pHID_ClassData.is_null() {
        free((*pdev).pHID_ClassData);
        (*pdev).pHID_ClassData = 0 as *mut libc::c_void
    }
    return USBD_OK as libc::c_int as uint8_t;
}
/* *
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
unsafe extern "C" fn USBD_HID_Setup(mut pdev: *mut USBD_HandleTypeDef,
                                    mut req: *mut USBD_SetupReqTypedef)
 -> uint8_t {
    let mut len: uint16_t = 0 as libc::c_int as uint16_t;
    let mut pbuf: *mut uint8_t = 0 as *mut uint8_t;
    let mut hhid: *mut USBD_HID_HandleTypeDef =
        (*pdev).pHID_ClassData as *mut USBD_HID_HandleTypeDef;
    match (*req).bmRequest as libc::c_int & 0x60 as libc::c_int {
        32 => {
            match (*req).bRequest as libc::c_int {
                11 => {
                    (*hhid).Protocol = (*req).wValue as uint8_t as uint32_t
                }
                3 => {
                    USBD_CtlSendData(pdev,
                                     &mut (*hhid).Protocol as *mut uint32_t as
                                         *mut uint8_t,
                                     1 as libc::c_int as uint16_t);
                }
                10 => {
                    (*hhid).IdleState =
                        ((*req).wValue as libc::c_int >> 8 as libc::c_int) as
                            uint8_t as uint32_t
                }
                2 => {
                    USBD_CtlSendData(pdev,
                                     &mut (*hhid).IdleState as *mut uint32_t
                                         as *mut uint8_t,
                                     1 as libc::c_int as uint16_t);
                }
                _ => {
                    USBD_CtlError(pdev, req);
                    return USBD_FAIL as libc::c_int as uint8_t
                }
            }
        }
        0 => {
            match (*req).bRequest as libc::c_int {
                6 => {
                    if (*req).wValue as libc::c_int >> 8 as libc::c_int ==
                           0x22 as libc::c_int {
                        len =
                            if (38 as libc::c_int) <
                                   (*req).wLength as libc::c_int {
                                38 as libc::c_int
                            } else { (*req).wLength as libc::c_int } as
                                uint16_t;
                        pbuf = HID_MOUSE_ReportDesc.as_mut_ptr()
                    } else if (*req).wValue as libc::c_int >> 8 as libc::c_int
                                  == 0x21 as libc::c_int {
                        pbuf = USBD_HID_Desc.as_mut_ptr();
                        len =
                            if (9 as libc::c_int) <
                                   (*req).wLength as libc::c_int {
                                9 as libc::c_int
                            } else { (*req).wLength as libc::c_int } as
                                uint16_t
                    }
                    USBD_CtlSendData(pdev, pbuf, len);
                }
                10 => {
                    USBD_CtlSendData(pdev,
                                     &mut (*hhid).AltSetting as *mut uint32_t
                                         as *mut uint8_t,
                                     1 as libc::c_int as uint16_t);
                }
                11 => {
                    (*hhid).AltSetting = (*req).wValue as uint8_t as uint32_t
                }
                _ => { }
            }
        }
        _ => { }
    }
    return USBD_OK as libc::c_int as uint8_t;
}
/* *
  * @brief  USBD_HID_SendReport
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_HID_SendReport(mut pdev:
                                                 *mut USBD_HandleTypeDef,
                                             mut report: *mut uint8_t,
                                             mut len: uint16_t) -> uint8_t {
    let mut hhid: *mut USBD_HID_HandleTypeDef =
        (*pdev).pHID_ClassData as *mut USBD_HID_HandleTypeDef;
    if (*pdev).dev_state as libc::c_int == 3 as libc::c_int {
        if (*hhid).state as libc::c_uint ==
               HID_IDLE as libc::c_int as libc::c_uint {
            (*hhid).state = HID_BUSY;
            USBD_LL_Transmit(pdev, 0x83 as libc::c_int as uint8_t, report,
                             len);
        }
    }
    return USBD_OK as libc::c_int as uint8_t;
}
/* *
  ******************************************************************************
  * @file    usbd_hid.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header file for the usbd_hid_core.c file.
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
/* * @defgroup USBD_HID
  * @brief This file is the Header file for usbd_hid.c
  * @{
  */
/* * @defgroup USBD_HID_Exported_Defines
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
/* * @defgroup USB_CORE_Exported_Functions
  * @{
  */
/* *
  * @brief  USBD_HID_GetPollingInterval
  *         return polling interval from endpoint descriptor
  * @param  pdev: device instance
  * @retval polling interval
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_HID_GetPollingInterval(mut pdev:
                                                         *mut USBD_HandleTypeDef)
 -> uint32_t {
    let mut polling_interval: uint32_t = 0 as libc::c_int as uint32_t;
    /* HIGH-speed endpoints */
    if (*pdev).dev_speed as libc::c_uint ==
           USBD_SPEED_HIGH as libc::c_int as libc::c_uint {
        /* Sets the data transfer polling interval for high speed transfers.
    Values between 1..16 are allowed. Values correspond to interval
    of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
        polling_interval =
            (((1 as libc::c_int) << 0x7 as libc::c_int - 1 as libc::c_int) /
                 8 as libc::c_int) as uint32_t
    } else {
        /* LOW and FULL-speed endpoints */
        /* Sets the data transfer polling interval for low and full
    speed transfers */
        polling_interval = 0xa as libc::c_int as uint32_t
    }
    return polling_interval;
}
/* *
  * @brief  USBD_HID_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
unsafe extern "C" fn USBD_HID_GetCfgDesc(mut length: *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 34]>() as libc::c_ulong as uint16_t;
    return USBD_HID_CfgDesc.as_mut_ptr();
}
/* *
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
unsafe extern "C" fn USBD_HID_DataIn(mut pdev: *mut USBD_HandleTypeDef,
                                     mut epnum: uint8_t) -> uint8_t {
    /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
    (*((*pdev).pHID_ClassData as *mut USBD_HID_HandleTypeDef)).state =
        HID_IDLE;
    return USBD_OK as libc::c_int as uint8_t;
}
/* *
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
unsafe extern "C" fn USBD_HID_GetDeviceQualifierDesc(mut length:
                                                         *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 10]>() as libc::c_ulong as uint16_t;
    return USBD_HID_DeviceQualifierDesc.as_mut_ptr();
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
