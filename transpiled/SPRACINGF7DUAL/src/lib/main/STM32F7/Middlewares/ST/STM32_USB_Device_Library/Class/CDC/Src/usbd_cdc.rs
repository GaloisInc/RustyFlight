use ::libc;
extern "C" {
    #[no_mangle]
    fn malloc(_: libc::c_ulong) -> *mut libc::c_void;
    #[no_mangle]
    fn free(__ptr: *mut libc::c_void);
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
    #[no_mangle]
    fn USBD_LL_PrepareReceive(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t,
                              pbuf: *mut uint8_t, size: uint16_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_GetRxDataSize(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
     -> uint32_t;
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
    #[no_mangle]
    fn USBD_CtlPrepareRx(pdev: *mut USBD_HandleTypeDef, pbuf: *mut uint8_t,
                         len: uint16_t) -> USBD_StatusTypeDef;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USBD_CDC_Itf {
    pub Init: Option<unsafe extern "C" fn() -> int8_t>,
    pub DeInit: Option<unsafe extern "C" fn() -> int8_t>,
    pub Control: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                             _: uint16_t) -> int8_t>,
    pub Receive: Option<unsafe extern "C" fn(_: *mut uint8_t,
                                             _: *mut uint32_t) -> int8_t>,
}
pub type USBD_CDC_ItfTypeDef = _USBD_CDC_Itf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_CDC_HandleTypeDef {
    pub data: [uint32_t; 128],
    pub CmdOpCode: uint8_t,
    pub CmdLength: uint8_t,
    pub RxBuffer: *mut uint8_t,
    pub TxBuffer: *mut uint8_t,
    pub RxLength: uint32_t,
    pub TxLength: uint32_t,
    pub TxState: uint32_t,
    pub RxState: uint32_t,
}
/* USB Standard Device Descriptor */
static mut USBD_CDC_DeviceQualifierDesc: [uint8_t; 10] =
    [0xa as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
/* *
  * @}
  */
/* * @defgroup USBD_CDC_Private_Variables
  * @{
  */
/* CDC interface class callbacks structure */
#[no_mangle]
pub static mut USBD_CDC: USBD_ClassTypeDef =
    unsafe {
        {
            let mut init =
                _Device_cb{Init:
                               Some(USBD_CDC_Init as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           DeInit:
                               Some(USBD_CDC_DeInit as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           Setup:
                               Some(USBD_CDC_Setup as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _:
                                                                 *mut USBD_SetupReqTypedef)
                                            -> uint8_t),
                           EP0_TxSent: None,
                           EP0_RxReady:
                               Some(USBD_CDC_EP0_RxReady as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef)
                                            -> uint8_t),
                           DataIn:
                               Some(USBD_CDC_DataIn as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           DataOut:
                               Some(USBD_CDC_DataOut as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           SOF: None,
                           IsoINIncomplete: None,
                           IsoOUTIncomplete: None,
                           GetHSConfigDescriptor:
                               Some(USBD_CDC_GetHSCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetFSConfigDescriptor:
                               Some(USBD_CDC_GetFSCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetOtherSpeedConfigDescriptor:
                               Some(USBD_CDC_GetOtherSpeedCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetDeviceQualifierDescriptor:
                               Some(USBD_CDC_GetDeviceQualifierDescriptor as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),};
            init
        }
    };
/* USB CDC device Configuration Descriptor */
#[no_mangle]
pub static mut USBD_CDC_CfgHSDesc: [uint8_t; 67] =
    [0x9 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     67 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x24 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t,
     (8 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((8 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0x10 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t,
     (512 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((512 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x81 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t,
     (512 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((512 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t];
/* USB CDC device Configuration Descriptor */
#[no_mangle]
pub static mut USBD_CDC_CfgFSDesc: [uint8_t; 67] =
    [0x9 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     67 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x24 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t,
     (8 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((8 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0x10 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t,
     (64 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((64 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x81 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t,
     (64 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((64 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut USBD_CDC_OtherSpeedCfgDesc: [uint8_t; 67] =
    [0x9 as libc::c_int as uint8_t, 7 as libc::c_int as uint8_t,
     67 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x24 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t,
     (8 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((8 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0xff as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x7 as libc::c_int as uint8_t, 5 as libc::c_int as uint8_t,
     0x81 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x40 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t];
/* *
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                CDC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
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
/* * @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */
/* * @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_CDC_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CDC_Private_Functions
  * @{
  */
/* *
  * @brief  USBD_CDC_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
unsafe extern "C" fn USBD_CDC_Init(mut pdev: *mut USBD_HandleTypeDef,
                                   mut cfgidx: uint8_t) -> uint8_t {
    let mut ret: uint8_t = 0 as libc::c_int as uint8_t;
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        0 as *mut USBD_CDC_HandleTypeDef;
    if (*pdev).dev_speed as libc::c_uint ==
           USBD_SPEED_HIGH as libc::c_int as libc::c_uint {
        /* Open EP IN */
        USBD_LL_OpenEP(pdev, 0x81 as libc::c_int as uint8_t,
                       2 as libc::c_int as uint8_t,
                       512 as libc::c_int as uint16_t);
        /* Open EP OUT */
        USBD_LL_OpenEP(pdev, 0x1 as libc::c_int as uint8_t,
                       2 as libc::c_int as uint8_t,
                       512 as libc::c_int as uint16_t);
    } else {
        /* Open EP IN */
        USBD_LL_OpenEP(pdev, 0x81 as libc::c_int as uint8_t,
                       2 as libc::c_int as uint8_t,
                       64 as libc::c_int as uint16_t);
        /* Open EP OUT */
        USBD_LL_OpenEP(pdev, 0x1 as libc::c_int as uint8_t,
                       2 as libc::c_int as uint8_t,
                       64 as libc::c_int as uint16_t);
    }
    /* Open Command IN EP */
    USBD_LL_OpenEP(pdev, 0x82 as libc::c_int as uint8_t,
                   3 as libc::c_int as uint8_t, 8 as libc::c_int as uint16_t);
    (*pdev).pCDC_ClassData =
        malloc(::core::mem::size_of::<USBD_CDC_HandleTypeDef>() as
                   libc::c_ulong);
    if (*pdev).pCDC_ClassData.is_null() {
        ret = 1 as libc::c_int as uint8_t
    } else {
        hcdc = (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
        /* Init  physical Interface components */
        (*((*pdev).pCDC_UserData as
               *mut USBD_CDC_ItfTypeDef)).Init.expect("non-null function pointer")();
        /* Init Xfer states */
        ::core::ptr::write_volatile(&mut (*hcdc).TxState as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*hcdc).RxState as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        if (*pdev).dev_speed as libc::c_uint ==
               USBD_SPEED_HIGH as libc::c_int as libc::c_uint {
            /* Prepare Out endpoint to receive next packet */
            USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                                   (*hcdc).RxBuffer,
                                   512 as libc::c_int as uint16_t);
        } else {
            /* Prepare Out endpoint to receive next packet */
            USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                                   (*hcdc).RxBuffer,
                                   64 as libc::c_int as uint16_t);
        }
    }
    return ret;
}
/* *
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
unsafe extern "C" fn USBD_CDC_DeInit(mut pdev: *mut USBD_HandleTypeDef,
                                     mut cfgidx: uint8_t) -> uint8_t {
    let mut ret: uint8_t = 0 as libc::c_int as uint8_t;
    /* Open EP IN */
    USBD_LL_CloseEP(pdev, 0x81 as libc::c_int as uint8_t);
    /* Open EP OUT */
    USBD_LL_CloseEP(pdev, 0x1 as libc::c_int as uint8_t);
    /* Open Command IN EP */
    USBD_LL_CloseEP(pdev, 0x82 as libc::c_int as uint8_t);
    /* DeInit  physical Interface components */
    if !(*pdev).pCDC_ClassData.is_null() {
        (*((*pdev).pCDC_UserData as
               *mut USBD_CDC_ItfTypeDef)).DeInit.expect("non-null function pointer")();
        free((*pdev).pCDC_ClassData);
        (*pdev).pCDC_ClassData = 0 as *mut libc::c_void
    }
    return ret;
}
/* *
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
unsafe extern "C" fn USBD_CDC_Setup(mut pdev: *mut USBD_HandleTypeDef,
                                    mut req: *mut USBD_SetupReqTypedef)
 -> uint8_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    static mut ifalt: uint8_t = 0 as libc::c_int as uint8_t;
    match (*req).bmRequest as libc::c_int & 0x60 as libc::c_int {
        32 => {
            if (*req).wLength != 0 {
                if (*req).bmRequest as libc::c_int & 0x80 as libc::c_int != 0
                   {
                    (*((*pdev).pCDC_UserData as
                           *mut USBD_CDC_ItfTypeDef)).Control.expect("non-null function pointer")((*req).bRequest,
                                                                                                  (*hcdc).data.as_mut_ptr()
                                                                                                      as
                                                                                                      *mut uint8_t,
                                                                                                  (*req).wLength);
                    USBD_CtlSendData(pdev,
                                     (*hcdc).data.as_mut_ptr() as
                                         *mut uint8_t, (*req).wLength);
                } else {
                    (*hcdc).CmdOpCode = (*req).bRequest;
                    (*hcdc).CmdLength = (*req).wLength as uint8_t;
                    USBD_CtlPrepareRx(pdev,
                                      (*hcdc).data.as_mut_ptr() as
                                          *mut uint8_t, (*req).wLength);
                }
            } else {
                (*((*pdev).pCDC_UserData as
                       *mut USBD_CDC_ItfTypeDef)).Control.expect("non-null function pointer")((*req).bRequest,
                                                                                              req
                                                                                                  as
                                                                                                  *mut uint8_t,
                                                                                              0
                                                                                                  as
                                                                                                  libc::c_int
                                                                                                  as
                                                                                                  uint16_t);
            }
        }
        0 => {
            match (*req).bRequest as libc::c_int {
                10 => {
                    USBD_CtlSendData(pdev, &mut ifalt,
                                     1 as libc::c_int as uint16_t);
                }
                11 | _ => { }
            }
        }
        _ => { }
    }
    return USBD_OK as libc::c_int as uint8_t;
}
/* *
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
unsafe extern "C" fn USBD_CDC_DataIn(mut pdev: *mut USBD_HandleTypeDef,
                                     mut epnum: uint8_t) -> uint8_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    if !(*pdev).pCDC_ClassData.is_null() {
        ::core::ptr::write_volatile(&mut (*hcdc).TxState as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        return USBD_OK as libc::c_int as uint8_t
    } else { return USBD_FAIL as libc::c_int as uint8_t };
}
/* *
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
unsafe extern "C" fn USBD_CDC_DataOut(mut pdev: *mut USBD_HandleTypeDef,
                                      mut epnum: uint8_t) -> uint8_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    /* Get the received data length */
    (*hcdc).RxLength = USBD_LL_GetRxDataSize(pdev, epnum);
    /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */
    if !(*pdev).pCDC_ClassData.is_null() {
        (*((*pdev).pCDC_UserData as
               *mut USBD_CDC_ItfTypeDef)).Receive.expect("non-null function pointer")((*hcdc).RxBuffer,
                                                                                      &mut (*hcdc).RxLength);
        return USBD_OK as libc::c_int as uint8_t
    } else { return USBD_FAIL as libc::c_int as uint8_t };
}
/* *
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
unsafe extern "C" fn USBD_CDC_EP0_RxReady(mut pdev: *mut USBD_HandleTypeDef)
 -> uint8_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    if !(*pdev).pCDC_UserData.is_null() &&
           (*hcdc).CmdOpCode as libc::c_int != 0xff as libc::c_int {
        (*((*pdev).pCDC_UserData as
               *mut USBD_CDC_ItfTypeDef)).Control.expect("non-null function pointer")((*hcdc).CmdOpCode,
                                                                                      (*hcdc).data.as_mut_ptr()
                                                                                          as
                                                                                          *mut uint8_t,
                                                                                      (*hcdc).CmdLength
                                                                                          as
                                                                                          uint16_t);
        (*hcdc).CmdOpCode = 0xff as libc::c_int as uint8_t
    }
    return USBD_OK as libc::c_int as uint8_t;
}
/* *
  * @brief  USBD_CDC_GetFSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
unsafe extern "C" fn USBD_CDC_GetFSCfgDesc(mut length: *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 67]>() as libc::c_ulong as uint16_t;
    return USBD_CDC_CfgFSDesc.as_mut_ptr();
}
/* *
  * @brief  USBD_CDC_GetHSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
unsafe extern "C" fn USBD_CDC_GetHSCfgDesc(mut length: *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 67]>() as libc::c_ulong as uint16_t;
    return USBD_CDC_CfgHSDesc.as_mut_ptr();
}
/* *
  * @brief  USBD_CDC_GetCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
unsafe extern "C" fn USBD_CDC_GetOtherSpeedCfgDesc(mut length: *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 67]>() as libc::c_ulong as uint16_t;
    return USBD_CDC_OtherSpeedCfgDesc.as_mut_ptr();
}
/* *
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_CDC_GetDeviceQualifierDescriptor(mut length:
                                                                   *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 10]>() as libc::c_ulong as uint16_t;
    return USBD_CDC_DeviceQualifierDesc.as_mut_ptr();
}
/* *
* @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_CDC_RegisterInterface(mut pdev:
                                                        *mut USBD_HandleTypeDef,
                                                    mut fops:
                                                        *mut USBD_CDC_ItfTypeDef)
 -> uint8_t {
    let mut ret: uint8_t = USBD_FAIL as libc::c_int as uint8_t;
    if !fops.is_null() {
        (*pdev).pCDC_UserData = fops as *mut libc::c_void;
        ret = USBD_OK as libc::c_int as uint8_t
    }
    return ret;
}
/* *
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_CDC_SetTxBuffer(mut pdev:
                                                  *mut USBD_HandleTypeDef,
                                              mut pbuff: *mut uint8_t,
                                              mut length: uint16_t)
 -> uint8_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    (*hcdc).TxBuffer = pbuff;
    (*hcdc).TxLength = length as uint32_t;
    return USBD_OK as libc::c_int as uint8_t;
}
/* *
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_CDC_SetRxBuffer(mut pdev:
                                                  *mut USBD_HandleTypeDef,
                                              mut pbuff: *mut uint8_t)
 -> uint8_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    (*hcdc).RxBuffer = pbuff;
    return USBD_OK as libc::c_int as uint8_t;
}
/* *
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_CDC_TransmitPacket(mut pdev:
                                                     *mut USBD_HandleTypeDef)
 -> uint8_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    if !(*pdev).pCDC_ClassData.is_null() {
        if (*hcdc).TxState == 0 as libc::c_int as libc::c_uint {
            /* Tx Transfer in progress */
            ::core::ptr::write_volatile(&mut (*hcdc).TxState as *mut uint32_t,
                                        1 as libc::c_int as uint32_t);
            /* Transmit next packet */
            USBD_LL_Transmit(pdev, 0x81 as libc::c_int as uint8_t,
                             (*hcdc).TxBuffer, (*hcdc).TxLength as uint16_t);
            return USBD_OK as libc::c_int as uint8_t
        } else { return USBD_BUSY as libc::c_int as uint8_t }
    } else { return USBD_FAIL as libc::c_int as uint8_t };
}
/* *
  ******************************************************************************
  * @file    usbd_cdc.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_cdc.c file.
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
/* * @defgroup usbd_cdc
  * @brief This file is the Header file for usbd_cdc.c
  * @{
  */
/* * @defgroup usbd_cdc_Exported_Defines
  * @{
  */
/* EP1 for data IN */
/* EP1 for data OUT */
/* EP2 for CDC commands */
/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
/* Endpoint IN & OUT Packet size */
/* Endpoint IN & OUT Packet size */
/* Control Endpoint Packet size */
/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
/* Force 32bits alignment */
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
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_CDC_ReceivePacket(mut pdev:
                                                    *mut USBD_HandleTypeDef)
 -> uint8_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        (*pdev).pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    /* Suspend or Resume USB Out process */
    if !(*pdev).pCDC_ClassData.is_null() {
        if (*pdev).dev_speed as libc::c_uint ==
               USBD_SPEED_HIGH as libc::c_int as libc::c_uint {
            /* Prepare Out endpoint to receive next packet */
            USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                                   (*hcdc).RxBuffer,
                                   512 as libc::c_int as uint16_t);
        } else {
            /* Prepare Out endpoint to receive next packet */
            USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                                   (*hcdc).RxBuffer,
                                   64 as libc::c_int as uint16_t);
        }
        return USBD_OK as libc::c_int as uint8_t
    } else { return USBD_FAIL as libc::c_int as uint8_t };
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
