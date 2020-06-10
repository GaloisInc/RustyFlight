use ::libc;
extern "C" {
    #[no_mangle]
    fn malloc(_: libc::c_ulong) -> *mut libc::c_void;
    #[no_mangle]
    fn free(__ptr: *mut libc::c_void);
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
    fn USBD_LL_FlushEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
     -> USBD_StatusTypeDef;
    /* *
  * @}
  */
    /* * @defgroup USBD_CORE_Exported_Types
  * @{
  */
    /* *
  * @}
  */ 
/* * @defgroup USBD_CORE_Exported_FunctionsPrototypes
  * @{
  */
    #[no_mangle]
    fn MSC_BOT_Init(pdev: *mut USBD_HandleTypeDef);
    #[no_mangle]
    fn MSC_BOT_Reset(pdev: *mut USBD_HandleTypeDef);
    #[no_mangle]
    fn MSC_BOT_DeInit(pdev: *mut USBD_HandleTypeDef);
    #[no_mangle]
    fn MSC_BOT_DataIn(pdev: *mut USBD_HandleTypeDef, epnum: uint8_t);
    #[no_mangle]
    fn MSC_BOT_DataOut(pdev: *mut USBD_HandleTypeDef, epnum: uint8_t);
    #[no_mangle]
    fn MSC_BOT_CplClrFeature(pdev: *mut USBD_HandleTypeDef, epnum: uint8_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
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
  ******************************************************************************
  * @file    usbd_msc_bot.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for the usbd_msc_bot.c file
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
/* * @defgroup MSC_BOT
  * @brief This file is the Header file for usbd_msc_bot.c
  * @{
  */
/* * @defgroup USBD_CORE_Exported_Defines
  * @{
  */
/* Idle state */
/* Data Out state */
/* Data In state */
/* Last Data In Last */
/* Send Immediate data */
/* No data Stage */
/* CSW Status Definitions */
/* BOT Status */
/* *
  * @}
  */
/* * @defgroup MSC_CORE_Private_TypesDefinitions
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_MSC_BOT_CBWTypeDef {
    pub dSignature: uint32_t,
    pub dTag: uint32_t,
    pub dDataLength: uint32_t,
    pub bmFlags: uint8_t,
    pub bLUN: uint8_t,
    pub bCBLength: uint8_t,
    pub CB: [uint8_t; 16],
    pub ReservedForAlign: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_MSC_BOT_CSWTypeDef {
    pub dSignature: uint32_t,
    pub dTag: uint32_t,
    pub dDataResidue: uint32_t,
    pub bStatus: uint8_t,
    pub ReservedForAlign: [uint8_t; 3],
}
/* *
  * @}
  */
/* * @defgroup USBD_SCSI_Exported_TypesDefinitions
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _SENSE_ITEM {
    pub Skey: libc::c_char,
    pub w: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub b: _ASCs,
    pub ASC: libc::c_uint,
    pub pData: *mut libc::c_char,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _ASCs {
    pub ASC: libc::c_char,
    pub ASCQ: libc::c_char,
}
pub type USBD_SCSI_SenseTypeDef = _SENSE_ITEM;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USBD_STORAGE {
    pub Init: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub GetCapacity: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint32_t,
                                                 _: *mut uint16_t) -> int8_t>,
    pub IsReady: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub IsWriteProtected: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub Read: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                          _: uint32_t, _: uint16_t)
                         -> int8_t>,
    pub Write: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                           _: uint32_t, _: uint16_t)
                          -> int8_t>,
    pub GetMaxLun: Option<unsafe extern "C" fn() -> int8_t>,
    pub pInquiry: *mut int8_t,
}
pub type USBD_StorageTypeDef = _USBD_STORAGE;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_MSC_BOT_HandleTypeDef {
    pub max_lun: uint32_t,
    pub interface: uint32_t,
    pub bot_state: uint8_t,
    pub bot_status: uint8_t,
    pub bot_data_length: uint16_t,
    pub bot_data: [uint8_t; 512],
    pub cbw: USBD_MSC_BOT_CBWTypeDef,
    pub csw: USBD_MSC_BOT_CSWTypeDef,
    pub scsi_sense: [USBD_SCSI_SenseTypeDef; 4],
    pub scsi_sense_head: uint8_t,
    pub scsi_sense_tail: uint8_t,
    pub scsi_blk_size: uint16_t,
    pub scsi_blk_nbr: uint32_t,
    pub scsi_blk_addr: uint32_t,
    pub scsi_blk_len: uint32_t,
}
/* *
  * @}
  */
/* * @defgroup MSC_CORE_Private_Variables
  * @{
  */
#[no_mangle]
pub static mut USBD_MSC: USBD_ClassTypeDef =
    unsafe {
        {
            let mut init =
                _Device_cb{Init:
                               Some(USBD_MSC_Init as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           DeInit:
                               Some(USBD_MSC_DeInit as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           Setup:
                               Some(USBD_MSC_Setup as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _:
                                                                 *mut USBD_SetupReqTypedef)
                                            -> uint8_t),
                           EP0_TxSent: None,
                           EP0_RxReady: None,
                           DataIn:
                               Some(USBD_MSC_DataIn as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           DataOut:
                               Some(USBD_MSC_DataOut as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           SOF: None,
                           IsoINIncomplete: None,
                           IsoOUTIncomplete: None,
                           GetHSConfigDescriptor:
                               Some(USBD_MSC_GetHSCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetFSConfigDescriptor:
                               Some(USBD_MSC_GetFSCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetOtherSpeedConfigDescriptor:
                               Some(USBD_MSC_GetOtherSpeedCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetDeviceQualifierDescriptor:
                               Some(USBD_MSC_GetDeviceQualifierDescriptor as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),};
            init
        }
    };
/* USB Mass storage device Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
#[no_mangle]
pub static mut USBD_MSC_CfgHSDesc: [uint8_t; 32] =
    [0x9 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     32 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x50 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x7 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x81 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     (0x200 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x200 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t,
     (0x200 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x200 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t];
/* USB Mass storage device Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
#[no_mangle]
pub static mut USBD_MSC_CfgFSDesc: [uint8_t; 32] =
    [0x9 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     32 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x50 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x7 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x81 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     (0x40 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x40 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t,
     (0x40 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x40 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut USBD_MSC_OtherSpeedCfgDesc: [uint8_t; 32] =
    [0x9 as libc::c_int as uint8_t, 7 as libc::c_int as uint8_t,
     32 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x50 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x7 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x81 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x40 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
/* USB Standard Device Descriptor */
#[no_mangle]
pub static mut USBD_MSC_DeviceQualifierDesc: [uint8_t; 10] =
    [0xa as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
/* *
  ******************************************************************************
  * @file    usbd_msc.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the MSC core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                MSC Class  Description
  *          ===================================================================
  *           This module manages the MSC class V1.0 following the "Universal
  *           Serial Bus Mass Storage Class (MSC) Bulk-Only Transport (BOT) Version 1.0
  *           Sep. 31, 1999".
  *           This driver implements the following aspects of the specification:
  *             - Bulk-Only Transport protocol
  *             - Subclass : SCSI transparent command set (ref. SCSI Primary Commands - 3 (SPC-3))
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
/* * @defgroup MSC_CORE
  * @brief Mass storage core module
  * @{
  */
/* * @defgroup MSC_CORE_Private_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_CORE_Private_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_CORE_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_CORE_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_CORE_Private_Functions
  * @{
  */
/* *
  * @brief  USBD_MSC_Init
  *         Initialize  the mass storage configuration
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_Init(mut pdev: *mut USBD_HandleTypeDef,
                                       mut cfgidx: uint8_t) -> uint8_t {
    let mut ret: int16_t = 0 as libc::c_int as int16_t;
    if (*pdev).dev_speed as libc::c_uint ==
           USBD_SPEED_HIGH as libc::c_int as libc::c_uint {
        /* Open EP OUT */
        USBD_LL_OpenEP(pdev, 0x1 as libc::c_int as uint8_t,
                       2 as libc::c_int as uint8_t,
                       0x200 as libc::c_int as uint16_t);
        /* Open EP IN */
        USBD_LL_OpenEP(pdev, 0x81 as libc::c_int as uint8_t,
                       2 as libc::c_int as uint8_t,
                       0x200 as libc::c_int as uint16_t);
    } else {
        /* Open EP OUT */
        USBD_LL_OpenEP(pdev, 0x1 as libc::c_int as uint8_t,
                       2 as libc::c_int as uint8_t,
                       0x40 as libc::c_int as uint16_t);
        /* Open EP IN */
        USBD_LL_OpenEP(pdev, 0x81 as libc::c_int as uint8_t,
                       2 as libc::c_int as uint8_t,
                       0x40 as libc::c_int as uint16_t);
    }
    (*pdev).pMSC_ClassData =
        malloc(::core::mem::size_of::<USBD_MSC_BOT_HandleTypeDef>() as
                   libc::c_ulong);
    if (*pdev).pMSC_ClassData.is_null() {
        ret = 1 as libc::c_int as int16_t
    } else {
        /* Init the BOT  layer */
        MSC_BOT_Init(pdev);
        ret = 0 as libc::c_int as int16_t
    }
    return ret as uint8_t;
}
/* *
  * @brief  USBD_MSC_DeInit
  *         DeInitilaize  the mass storage configuration
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_DeInit(mut pdev: *mut USBD_HandleTypeDef,
                                         mut cfgidx: uint8_t) -> uint8_t {
    /* Close MSC EPs */
    USBD_LL_CloseEP(pdev, 0x1 as libc::c_int as uint8_t);
    /* Open EP IN */
    USBD_LL_CloseEP(pdev, 0x81 as libc::c_int as uint8_t);
    /* De-Init the BOT layer */
    MSC_BOT_DeInit(pdev);
    /* Free MSC Class Resources */
    if !(*pdev).pMSC_ClassData.is_null() {
        free((*pdev).pMSC_ClassData);
        (*pdev).pMSC_ClassData = 0 as *mut libc::c_void
    }
    return 0 as libc::c_int as uint8_t;
}
/* *
* @brief  USBD_MSC_Setup
*         Handle the MSC specific requests
* @param  pdev: device instance
* @param  req: USB request
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_Setup(mut pdev: *mut USBD_HandleTypeDef,
                                        mut req: *mut USBD_SetupReqTypedef)
 -> uint8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    match (*req).bmRequest as libc::c_int & 0x60 as libc::c_int {
        32 => {
            /* Class request */
            match (*req).bRequest as libc::c_int {
                254 => {
                    if (*req).wValue as libc::c_int == 0 as libc::c_int &&
                           (*req).wLength as libc::c_int == 1 as libc::c_int
                           &&
                           (*req).bmRequest as libc::c_int &
                               0x80 as libc::c_int == 0x80 as libc::c_int {
                        (*hmsc).max_lun =
                            (*((*pdev).pMSC_UserData as
                                   *mut USBD_StorageTypeDef)).GetMaxLun.expect("non-null function pointer")()
                                as uint32_t;
                        USBD_CtlSendData(pdev,
                                         &mut (*hmsc).max_lun as *mut uint32_t
                                             as *mut uint8_t,
                                         1 as libc::c_int as uint16_t);
                    } else {
                        USBD_CtlError(pdev, req);
                        return USBD_FAIL as libc::c_int as uint8_t
                    }
                }
                255 => {
                    if (*req).wValue as libc::c_int == 0 as libc::c_int &&
                           (*req).wLength as libc::c_int == 0 as libc::c_int
                           &&
                           (*req).bmRequest as libc::c_int &
                               0x80 as libc::c_int != 0x80 as libc::c_int {
                        MSC_BOT_Reset(pdev);
                    } else {
                        USBD_CtlError(pdev, req);
                        return USBD_FAIL as libc::c_int as uint8_t
                    }
                }
                _ => {
                    USBD_CtlError(pdev, req);
                    return USBD_FAIL as libc::c_int as uint8_t
                }
            }
        }
        0 => {
            /* Interface & Endpoint request */
            match (*req).bRequest as libc::c_int {
                10 => {
                    USBD_CtlSendData(pdev,
                                     &mut (*hmsc).interface as *mut uint32_t
                                         as *mut uint8_t,
                                     1 as libc::c_int as uint16_t);
                }
                11 => {
                    (*hmsc).interface = (*req).wValue as uint8_t as uint32_t
                }
                1 => {
                    /* Flush the FIFO and Clear the stall status */
                    USBD_LL_FlushEP(pdev, (*req).wIndex as uint8_t);
                    /* Reactivate the EP */
                    USBD_LL_CloseEP(pdev, (*req).wIndex as uint8_t);
                    if (*req).wIndex as uint8_t as libc::c_int &
                           0x80 as libc::c_int == 0x80 as libc::c_int {
                        if (*pdev).dev_speed as libc::c_uint ==
                               USBD_SPEED_HIGH as libc::c_int as libc::c_uint
                           {
                            /* Open EP IN */
                            USBD_LL_OpenEP(pdev,
                                           0x81 as libc::c_int as uint8_t,
                                           2 as libc::c_int as uint8_t,
                                           0x200 as libc::c_int as uint16_t);
                        } else {
                            /* Open EP IN */
                            USBD_LL_OpenEP(pdev,
                                           0x81 as libc::c_int as uint8_t,
                                           2 as libc::c_int as uint8_t,
                                           0x40 as libc::c_int as uint16_t);
                        }
                    } else if (*pdev).dev_speed as libc::c_uint ==
                                  USBD_SPEED_HIGH as libc::c_int as
                                      libc::c_uint {
                        /* Open EP IN */
                        USBD_LL_OpenEP(pdev, 0x1 as libc::c_int as uint8_t,
                                       2 as libc::c_int as uint8_t,
                                       0x200 as libc::c_int as uint16_t);
                    } else {
                        /* Open EP IN */
                        USBD_LL_OpenEP(pdev, 0x1 as libc::c_int as uint8_t,
                                       2 as libc::c_int as uint8_t,
                                       0x40 as libc::c_int as uint16_t);
                    }
                    /* Handle BOT error */
                    MSC_BOT_CplClrFeature(pdev, (*req).wIndex as uint8_t);
                }
                _ => { }
            }
        }
        _ => { }
    }
    return 0 as libc::c_int as uint8_t;
}
/* *
* @brief  USBD_MSC_DataIn
*         handle data IN Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_DataIn(mut pdev: *mut USBD_HandleTypeDef,
                                         mut epnum: uint8_t) -> uint8_t {
    MSC_BOT_DataIn(pdev, epnum);
    return 0 as libc::c_int as uint8_t;
}
/* *
* @brief  USBD_MSC_DataOut
*         handle data OUT Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_DataOut(mut pdev: *mut USBD_HandleTypeDef,
                                          mut epnum: uint8_t) -> uint8_t {
    MSC_BOT_DataOut(pdev, epnum);
    return 0 as libc::c_int as uint8_t;
}
/* *
* @brief  USBD_MSC_GetHSCfgDesc
*         return configuration descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_GetHSCfgDesc(mut length: *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 32]>() as libc::c_ulong as uint16_t;
    return USBD_MSC_CfgHSDesc.as_mut_ptr();
}
/* *
* @brief  USBD_MSC_GetFSCfgDesc
*         return configuration descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_GetFSCfgDesc(mut length: *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 32]>() as libc::c_ulong as uint16_t;
    return USBD_MSC_CfgFSDesc.as_mut_ptr();
}
/* *
* @brief  USBD_MSC_GetOtherSpeedCfgDesc
*         return other speed configuration descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_GetOtherSpeedCfgDesc(mut length:
                                                           *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 32]>() as libc::c_ulong as uint16_t;
    return USBD_MSC_OtherSpeedCfgDesc.as_mut_ptr();
}
/* *
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_GetDeviceQualifierDescriptor(mut length:
                                                                   *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 10]>() as libc::c_ulong as uint16_t;
    return USBD_MSC_DeviceQualifierDesc.as_mut_ptr();
}
/* *
  ******************************************************************************
  * @file    usbd_msc.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for the usbd_msc.c file
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
/* * @addtogroup USBD_MSC_BOT
  * @{
  */
/* * @defgroup USBD_MSC
  * @brief This file is the Header file for usbd_msc.c
  * @{
  */
/* * @defgroup USBD_BOT_Exported_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USB_CORE_Exported_Types
  * @{
  */
/* Structure for MSC process */
/* *
* @brief  USBD_MSC_RegisterStorage
* @param  fops: storage callback
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_MSC_RegisterStorage(mut pdev:
                                                      *mut USBD_HandleTypeDef,
                                                  mut fops:
                                                      *mut USBD_StorageTypeDef)
 -> uint8_t {
    if !fops.is_null() { (*pdev).pMSC_UserData = fops as *mut libc::c_void }
    return 0 as libc::c_int as uint8_t;
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
