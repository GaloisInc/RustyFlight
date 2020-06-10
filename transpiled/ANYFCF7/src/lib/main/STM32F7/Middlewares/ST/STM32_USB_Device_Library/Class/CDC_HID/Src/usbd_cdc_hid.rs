use ::libc;
extern "C" {
    /* * @defgroup USBD_CORE_Exported_Macros
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USBD_CORE_Exported_Variables
  * @{
  */
    #[no_mangle]
    static mut USBD_CDC: USBD_ClassTypeDef;
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
    #[no_mangle]
    static mut USBD_HID: USBD_ClassTypeDef;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
/* Control Endpoints*/
/* Class Specific Endpoints*/
/* Following USB Device Speed */
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
pub type C2RustUnnamed = libc::c_uint;
pub const USBD_FAIL: C2RustUnnamed = 2;
pub const USBD_BUSY: C2RustUnnamed = 1;
pub const USBD_OK: C2RustUnnamed = 0;
pub type USBD_HandleTypeDef = _USBD_HandleTypeDef;
#[no_mangle]
pub static mut USBD_HID_CDC_DeviceDescriptor: [uint8_t; 18] =
    [0x12 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0xef as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 64 as libc::c_uint as uint8_t,
     (0x483 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x483 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, (0x3256 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x3256 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t];
static mut USBD_HID_CDC_CfgDesc: [uint8_t; 100] =
    [0x9 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     (34 as libc::c_int - 9 as libc::c_int + 67 as libc::c_int +
          8 as libc::c_int) as uint8_t, 0 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x21 as libc::c_int as uint8_t,
     0x11 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x22 as libc::c_int as uint8_t, 38 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 0x83 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0xb as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x24 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x10 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
     0x24 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x24 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x7 as libc::c_int as uint8_t, 5 as libc::c_int as uint8_t,
     0x82 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     (8 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((8 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0xff as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
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
/* USB Standard Device Descriptor */
static mut USBD_HID_CDC_DeviceQualifierDesc: [uint8_t; 10] =
    [0xa as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
//Will be NULL Callback because it's unused
/* CDC interface class callbacks structure */
#[no_mangle]
pub static mut USBD_HID_CDC: USBD_ClassTypeDef =
    unsafe {
        {
            let mut init =
                _Device_cb{Init:
                               Some(USBD_HID_CDC_Init as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           DeInit:
                               Some(USBD_HID_CDC_DeInit as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           Setup:
                               Some(USBD_HID_CDC_Setup as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _:
                                                                 *mut USBD_SetupReqTypedef)
                                            -> uint8_t),
                           EP0_TxSent: None,
                           EP0_RxReady:
                               Some(USBD_HID_CDC_EP0_RxReady as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef)
                                            -> uint8_t),
                           DataIn:
                               Some(USBD_HID_CDC_DataIn as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           DataOut:
                               Some(USBD_HID_CDC_DataOut as
                                        unsafe extern "C" fn(_:
                                                                 *mut USBD_HandleTypeDef,
                                                             _: uint8_t)
                                            -> uint8_t),
                           SOF: None,
                           IsoINIncomplete: None,
                           IsoOUTIncomplete: None,
                           GetHSConfigDescriptor: None,
                           GetFSConfigDescriptor:
                               Some(USBD_HID_CDC_GetFSCfgDesc as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),
                           GetOtherSpeedConfigDescriptor: None,
                           GetDeviceQualifierDescriptor:
                               Some(USBD_HID_CDC_GetDeviceQualifierDescriptor
                                        as
                                        unsafe extern "C" fn(_: *mut uint16_t)
                                            -> *mut uint8_t),};
            init
        }
    };
/* Wrapper related callbacks */
unsafe extern "C" fn USBD_HID_CDC_Init(mut pdev: *mut USBD_HandleTypeDef,
                                       mut cfgidx: uint8_t) -> uint8_t {
    //Init CDC
    USBD_CDC.Init.expect("non-null function pointer")(pdev, cfgidx);
    //Init HID
    USBD_HID.Init.expect("non-null function pointer")(pdev, cfgidx);
    return USBD_OK as libc::c_int as uint8_t;
}
unsafe extern "C" fn USBD_HID_CDC_DeInit(mut pdev: *mut USBD_HandleTypeDef,
                                         mut cfgidx: uint8_t) -> uint8_t {
    //DeInit CDC
    USBD_CDC.DeInit.expect("non-null function pointer")(pdev, cfgidx);
    //DeInit HID
    USBD_HID.DeInit.expect("non-null function pointer")(pdev, cfgidx);
    return USBD_OK as libc::c_int as uint8_t;
}
/* Control Endpoints*/
unsafe extern "C" fn USBD_HID_CDC_Setup(mut pdev: *mut USBD_HandleTypeDef,
                                        mut req: *mut USBD_SetupReqTypedef)
 -> uint8_t {
    match (*req).bmRequest as libc::c_int & 0x3 as libc::c_int {
        1 => {
            if (*req).wIndex as libc::c_int == 0 as libc::c_int {
                return USBD_HID.Setup.expect("non-null function pointer")(pdev,
                                                                          req)
            } else {
                return USBD_CDC.Setup.expect("non-null function pointer")(pdev,
                                                                          req)
            }
        }
        2 => {
            if (*req).wIndex as libc::c_int == 0x83 as libc::c_int {
                return USBD_HID.Setup.expect("non-null function pointer")(pdev,
                                                                          req)
            } else {
                return USBD_CDC.Setup.expect("non-null function pointer")(pdev,
                                                                          req)
            }
        }
        _ => { }
    }
    return USBD_OK as libc::c_int as uint8_t;
}
unsafe extern "C" fn USBD_HID_CDC_EP0_RxReady(mut pdev:
                                                  *mut USBD_HandleTypeDef)
 -> uint8_t {
    return USBD_CDC.EP0_RxReady.expect("non-null function pointer")(pdev);
}
/* Class Specific Endpoints*/
unsafe extern "C" fn USBD_HID_CDC_DataIn(mut pdev: *mut USBD_HandleTypeDef,
                                         mut epnum: uint8_t) -> uint8_t {
    if epnum as libc::c_int == 0x81 as libc::c_int & !(0x80 as libc::c_int) {
        return USBD_CDC.DataIn.expect("non-null function pointer")(pdev,
                                                                   epnum)
    } else {
        return USBD_HID.DataIn.expect("non-null function pointer")(pdev,
                                                                   epnum)
    };
}
unsafe extern "C" fn USBD_HID_CDC_DataOut(mut pdev: *mut USBD_HandleTypeDef,
                                          mut epnum: uint8_t) -> uint8_t {
    if epnum as libc::c_int == 0x1 as libc::c_int & !(0x80 as libc::c_int) {
        return USBD_CDC.DataOut.expect("non-null function pointer")(pdev,
                                                                    epnum)
    }
    return USBD_OK as libc::c_int as uint8_t;
}
unsafe extern "C" fn USBD_HID_CDC_GetFSCfgDesc(mut length: *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 100]>() as libc::c_ulong as uint16_t;
    return USBD_HID_CDC_CfgDesc.as_mut_ptr();
}
#[no_mangle]
pub unsafe extern "C" fn USBD_HID_CDC_GetDeviceQualifierDescriptor(mut length:
                                                                       *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 10]>() as libc::c_ulong as uint16_t;
    return USBD_HID_CDC_DeviceQualifierDesc.as_mut_ptr();
}
