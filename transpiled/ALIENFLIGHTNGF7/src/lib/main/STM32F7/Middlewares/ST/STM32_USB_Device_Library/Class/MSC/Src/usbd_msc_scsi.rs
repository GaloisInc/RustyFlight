use ::libc;
extern "C" {
    #[no_mangle]
    fn MSC_BOT_SendCSW(pdev: *mut USBD_HandleTypeDef, CSW_Status: uint8_t);
    #[no_mangle]
    fn USBD_LL_PrepareReceive(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t,
                              pbuf: *mut uint8_t, size: uint16_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_Transmit(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t,
                        pbuf: *mut uint8_t, size: uint16_t)
     -> USBD_StatusTypeDef;
    /* *
  * @}
  */
    /* * @defgroup USBD_INFO_Exported_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
    /* * @defgroup USBD_INFO_Exported_Macros
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USBD_INFO_Exported_Variables
  * @{
  */
    #[no_mangle]
    static MSC_Page00_Inquiry_Data: [uint8_t; 0];
    #[no_mangle]
    static MSC_Mode_Sense6_data: [uint8_t; 0];
    #[no_mangle]
    static MSC_Mode_Sense10_data: [uint8_t; 0];
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
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
/* Following USB Device status */
/* USB Device descriptors structure */
/* USB Device handle structure */
/* USB Device handle structure */
pub type USBD_HandleTypeDef = _USBD_HandleTypeDef;
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
/* This is stupid, any nice solution to handle multiple interfaces
   * would be much apriciated. Or at least a flow how this should be rewritten instead.
   */
/* *
  * @}
  */
/* * @defgroup USBD_SCSI_Exported_TypesDefinitions
  * @{
  */
pub type USBD_SCSI_SenseTypeDef = _SENSE_ITEM;
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
/* * @defgroup USB_CORE_Exported_Types
  * @{
  */
pub type USBD_StorageTypeDef = _USBD_STORAGE;
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
/* *
  * @}
  */
/* * @defgroup USBD_SCSI_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_SCSI_Exported_Variables
  * @{
  */
/* *
  * @}
  */ 
/* * @defgroup USBD_SCSI_Exported_FunctionsPrototype
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_SCSI_Private_Functions
  * @{
  */
/* *
* @brief  SCSI_ProcessCmd
*         Process SCSI commands
* @param  pdev: device instance
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn SCSI_ProcessCmd(mut pdev: *mut USBD_HandleTypeDef,
                                         mut lun: uint8_t,
                                         mut params: *mut uint8_t) -> int8_t {
    match *params.offset(0 as libc::c_int as isize) as libc::c_int {
        0 => { return SCSI_TestUnitReady(pdev, lun, params) }
        3 => { return SCSI_RequestSense(pdev, lun, params) }
        18 => { return SCSI_Inquiry(pdev, lun, params) }
        27 => { return SCSI_StartStopUnit(pdev, lun, params) }
        30 => { return SCSI_StartStopUnit(pdev, lun, params) }
        26 => { return SCSI_ModeSense6(pdev, lun, params) }
        90 => { return SCSI_ModeSense10(pdev, lun, params) }
        35 => { return SCSI_ReadFormatCapacity(pdev, lun, params) }
        37 => { return SCSI_ReadCapacity10(pdev, lun, params) }
        40 => { return SCSI_Read10(pdev, lun, params) }
        42 => { return SCSI_Write10(pdev, lun, params) }
        47 => { return SCSI_Verify10(pdev, lun, params) }
        _ => {
            SCSI_SenseCode(pdev, lun, 5 as libc::c_int as uint8_t,
                           0x20 as libc::c_int as uint8_t);
            return -(1 as libc::c_int) as int8_t
        }
    };
}
/* *
  ******************************************************************************
  * @file    usbd_msc_scsi.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the USBD SCSI layer functions.
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
/* * @defgroup MSC_SCSI
  * @brief Mass storage SCSI layer module
  * @{
  */
/* * @defgroup MSC_SCSI_Private_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_SCSI_Private_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_SCSI_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_SCSI_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_SCSI_Private_FunctionPrototypes
  * @{
  */
/* *
* @brief  SCSI_TestUnitReady
*         Process SCSI Test Unit Ready Command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_TestUnitReady(mut pdev: *mut USBD_HandleTypeDef,
                                        mut lun: uint8_t,
                                        mut params: *mut uint8_t) -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    /* case 9 : Hi > D0 */
    if (*hmsc).cbw.dDataLength != 0 as libc::c_int as libc::c_uint {
        SCSI_SenseCode(pdev, (*hmsc).cbw.bLUN, 5 as libc::c_int as uint8_t,
                       0x20 as libc::c_int as uint8_t);
        return -(1 as libc::c_int) as int8_t
    }
    if (*((*pdev).pMSC_UserData as
              *mut USBD_StorageTypeDef)).IsReady.expect("non-null function pointer")(lun)
           as libc::c_int != 0 as libc::c_int {
        SCSI_SenseCode(pdev, lun, 2 as libc::c_int as uint8_t,
                       0x3a as libc::c_int as uint8_t);
        (*hmsc).bot_state = 5 as libc::c_int as uint8_t;
        return -(1 as libc::c_int) as int8_t
    }
    (*hmsc).bot_data_length = 0 as libc::c_int as uint16_t;
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_Inquiry
*         Process Inquiry command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_Inquiry(mut pdev: *mut USBD_HandleTypeDef,
                                  mut lun: uint8_t, mut params: *mut uint8_t)
 -> int8_t {
    let mut pPage: *mut uint8_t = 0 as *mut uint8_t;
    let mut len: uint16_t = 0;
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    if *params.offset(1 as libc::c_int as isize) as libc::c_int &
           0x1 as libc::c_int != 0 {
        /*Evpd is set*/
        pPage = MSC_Page00_Inquiry_Data.as_ptr() as *mut uint8_t;
        len = 7 as libc::c_int as uint16_t
    } else {
        pPage =
            &mut *(*((*pdev).pMSC_UserData as
                         *mut USBD_StorageTypeDef)).pInquiry.offset((lun as
                                                                         libc::c_int
                                                                         *
                                                                         0x24
                                                                             as
                                                                             libc::c_int)
                                                                        as
                                                                        isize)
                as *mut int8_t as *mut uint8_t;
        len =
            (*pPage.offset(4 as libc::c_int as isize) as libc::c_int +
                 5 as libc::c_int) as uint16_t;
        if *params.offset(4 as libc::c_int as isize) as libc::c_int <=
               len as libc::c_int {
            len = *params.offset(4 as libc::c_int as isize) as uint16_t
        }
    }
    (*hmsc).bot_data_length = len;
    while len != 0 {
        len = len.wrapping_sub(1);
        (*hmsc).bot_data[len as usize] = *pPage.offset(len as isize)
    }
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_ReadCapacity10
*         Process Read Capacity 10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_ReadCapacity10(mut pdev: *mut USBD_HandleTypeDef,
                                         mut lun: uint8_t,
                                         mut params: *mut uint8_t) -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    if (*((*pdev).pMSC_UserData as
              *mut USBD_StorageTypeDef)).GetCapacity.expect("non-null function pointer")(lun,
                                                                                         &mut (*hmsc).scsi_blk_nbr,
                                                                                         &mut (*hmsc).scsi_blk_size)
           as libc::c_int != 0 as libc::c_int {
        SCSI_SenseCode(pdev, lun, 2 as libc::c_int as uint8_t,
                       0x3a as libc::c_int as uint8_t);
        return -(1 as libc::c_int) as int8_t
    } else {
        (*hmsc).bot_data[0 as libc::c_int as usize] =
            ((*hmsc).scsi_blk_nbr.wrapping_sub(1 as libc::c_int as
                                                   libc::c_uint) >>
                 24 as libc::c_int) as uint8_t;
        (*hmsc).bot_data[1 as libc::c_int as usize] =
            ((*hmsc).scsi_blk_nbr.wrapping_sub(1 as libc::c_int as
                                                   libc::c_uint) >>
                 16 as libc::c_int) as uint8_t;
        (*hmsc).bot_data[2 as libc::c_int as usize] =
            ((*hmsc).scsi_blk_nbr.wrapping_sub(1 as libc::c_int as
                                                   libc::c_uint) >>
                 8 as libc::c_int) as uint8_t;
        (*hmsc).bot_data[3 as libc::c_int as usize] =
            (*hmsc).scsi_blk_nbr.wrapping_sub(1 as libc::c_int as
                                                  libc::c_uint) as uint8_t;
        (*hmsc).bot_data[4 as libc::c_int as usize] =
            ((*hmsc).scsi_blk_size as libc::c_int >> 24 as libc::c_int) as
                uint8_t;
        (*hmsc).bot_data[5 as libc::c_int as usize] =
            ((*hmsc).scsi_blk_size as libc::c_int >> 16 as libc::c_int) as
                uint8_t;
        (*hmsc).bot_data[6 as libc::c_int as usize] =
            ((*hmsc).scsi_blk_size as libc::c_int >> 8 as libc::c_int) as
                uint8_t;
        (*hmsc).bot_data[7 as libc::c_int as usize] =
            (*hmsc).scsi_blk_size as uint8_t;
        (*hmsc).bot_data_length = 8 as libc::c_int as uint16_t;
        return 0 as libc::c_int as int8_t
    };
}
/* *
* @brief  SCSI_ReadFormatCapacity
*         Process Read Format Capacity command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_ReadFormatCapacity(mut pdev:
                                                 *mut USBD_HandleTypeDef,
                                             mut lun: uint8_t,
                                             mut params: *mut uint8_t)
 -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    let mut blk_size: uint16_t = 0;
    let mut blk_nbr: uint32_t = 0;
    let mut i: uint16_t = 0;
    i = 0 as libc::c_int as uint16_t;
    while (i as libc::c_int) < 12 as libc::c_int {
        (*hmsc).bot_data[i as usize] = 0 as libc::c_int as uint8_t;
        i = i.wrapping_add(1)
    }
    if (*((*pdev).pMSC_UserData as
              *mut USBD_StorageTypeDef)).GetCapacity.expect("non-null function pointer")(lun,
                                                                                         &mut blk_nbr,
                                                                                         &mut blk_size)
           as libc::c_int != 0 as libc::c_int {
        SCSI_SenseCode(pdev, lun, 2 as libc::c_int as uint8_t,
                       0x3a as libc::c_int as uint8_t);
        return -(1 as libc::c_int) as int8_t
    } else {
        (*hmsc).bot_data[3 as libc::c_int as usize] =
            0x8 as libc::c_int as uint8_t;
        (*hmsc).bot_data[4 as libc::c_int as usize] =
            (blk_nbr.wrapping_sub(1 as libc::c_int as libc::c_uint) >>
                 24 as libc::c_int) as uint8_t;
        (*hmsc).bot_data[5 as libc::c_int as usize] =
            (blk_nbr.wrapping_sub(1 as libc::c_int as libc::c_uint) >>
                 16 as libc::c_int) as uint8_t;
        (*hmsc).bot_data[6 as libc::c_int as usize] =
            (blk_nbr.wrapping_sub(1 as libc::c_int as libc::c_uint) >>
                 8 as libc::c_int) as uint8_t;
        (*hmsc).bot_data[7 as libc::c_int as usize] =
            blk_nbr.wrapping_sub(1 as libc::c_int as libc::c_uint) as uint8_t;
        (*hmsc).bot_data[8 as libc::c_int as usize] =
            0x2 as libc::c_int as uint8_t;
        (*hmsc).bot_data[9 as libc::c_int as usize] =
            (blk_size as libc::c_int >> 16 as libc::c_int) as uint8_t;
        (*hmsc).bot_data[10 as libc::c_int as usize] =
            (blk_size as libc::c_int >> 8 as libc::c_int) as uint8_t;
        (*hmsc).bot_data[11 as libc::c_int as usize] = blk_size as uint8_t;
        (*hmsc).bot_data_length = 12 as libc::c_int as uint16_t;
        return 0 as libc::c_int as int8_t
    };
}
/* *
* @brief  SCSI_ModeSense6
*         Process Mode Sense6 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_ModeSense6(mut pdev: *mut USBD_HandleTypeDef,
                                     mut lun: uint8_t,
                                     mut params: *mut uint8_t) -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    let mut len: uint16_t = 8 as libc::c_int as uint16_t;
    (*hmsc).bot_data_length = len;
    while len != 0 {
        len = len.wrapping_sub(1);
        (*hmsc).bot_data[len as usize] =
            *MSC_Mode_Sense6_data.as_ptr().offset(len as isize)
    }
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_ModeSense10
*         Process Mode Sense10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_ModeSense10(mut pdev: *mut USBD_HandleTypeDef,
                                      mut lun: uint8_t,
                                      mut params: *mut uint8_t) -> int8_t {
    let mut len: uint16_t = 8 as libc::c_int as uint16_t;
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    (*hmsc).bot_data_length = len;
    while len != 0 {
        len = len.wrapping_sub(1);
        (*hmsc).bot_data[len as usize] =
            *MSC_Mode_Sense10_data.as_ptr().offset(len as isize)
    }
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_RequestSense
*         Process Request Sense command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_RequestSense(mut pdev: *mut USBD_HandleTypeDef,
                                       mut lun: uint8_t,
                                       mut params: *mut uint8_t) -> int8_t {
    let mut i: uint8_t = 0;
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    i = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 0x12 as libc::c_int {
        (*hmsc).bot_data[i as usize] = 0 as libc::c_int as uint8_t;
        i = i.wrapping_add(1)
    }
    (*hmsc).bot_data[0 as libc::c_int as usize] =
        0x70 as libc::c_int as uint8_t;
    (*hmsc).bot_data[7 as libc::c_int as usize] =
        (0x12 as libc::c_int - 6 as libc::c_int) as uint8_t;
    if (*hmsc).scsi_sense_head as libc::c_int !=
           (*hmsc).scsi_sense_tail as libc::c_int {
        (*hmsc).bot_data[2 as libc::c_int as usize] =
            (*hmsc).scsi_sense[(*hmsc).scsi_sense_head as usize].Skey as
                uint8_t;
        (*hmsc).bot_data[12 as libc::c_int as usize] =
            (*hmsc).scsi_sense[(*hmsc).scsi_sense_head as usize].w.b.ASCQ as
                uint8_t;
        (*hmsc).bot_data[13 as libc::c_int as usize] =
            (*hmsc).scsi_sense[(*hmsc).scsi_sense_head as usize].w.b.ASC as
                uint8_t;
        (*hmsc).scsi_sense_head = (*hmsc).scsi_sense_head.wrapping_add(1);
        if (*hmsc).scsi_sense_head as libc::c_int == 4 as libc::c_int {
            (*hmsc).scsi_sense_head = 0 as libc::c_int as uint8_t
        }
    }
    (*hmsc).bot_data_length = 0x12 as libc::c_int as uint16_t;
    if *params.offset(4 as libc::c_int as isize) as libc::c_int <=
           0x12 as libc::c_int {
        (*hmsc).bot_data_length =
            *params.offset(4 as libc::c_int as isize) as uint16_t
    }
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_SenseCode
*         Load the last error code in the error list
* @param  lun: Logical unit number
* @param  sKey: Sense Key
* @param  ASC: Additional Sense Key
* @retval none

*/
#[no_mangle]
pub unsafe extern "C" fn SCSI_SenseCode(mut pdev: *mut USBD_HandleTypeDef,
                                        mut lun: uint8_t, mut sKey: uint8_t,
                                        mut ASC: uint8_t) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    (*hmsc).scsi_sense[(*hmsc).scsi_sense_tail as usize].Skey =
        sKey as libc::c_char;
    (*hmsc).scsi_sense[(*hmsc).scsi_sense_tail as usize].w.ASC =
        ((ASC as libc::c_int) << 8 as libc::c_int) as libc::c_uint;
    (*hmsc).scsi_sense_tail = (*hmsc).scsi_sense_tail.wrapping_add(1);
    if (*hmsc).scsi_sense_tail as libc::c_int == 4 as libc::c_int {
        (*hmsc).scsi_sense_tail = 0 as libc::c_int as uint8_t
    };
}
/* *
* @brief  SCSI_StartStopUnit
*         Process Start Stop Unit command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_StartStopUnit(mut pdev: *mut USBD_HandleTypeDef,
                                        mut lun: uint8_t,
                                        mut params: *mut uint8_t) -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    (*hmsc).bot_data_length = 0 as libc::c_int as uint16_t;
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_Read10
*         Process Read10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_Read10(mut pdev: *mut USBD_HandleTypeDef,
                                 mut lun: uint8_t, mut params: *mut uint8_t)
 -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    if (*hmsc).bot_state as libc::c_int == 0 as libc::c_int {
        /* Idle */
        /* case 10 : Ho <> Di */
        if (*hmsc).cbw.bmFlags as libc::c_int & 0x80 as libc::c_int !=
               0x80 as libc::c_int {
            SCSI_SenseCode(pdev, (*hmsc).cbw.bLUN,
                           5 as libc::c_int as uint8_t,
                           0x20 as libc::c_int as uint8_t);
            return -(1 as libc::c_int) as int8_t
        }
        if (*((*pdev).pMSC_UserData as
                  *mut USBD_StorageTypeDef)).IsReady.expect("non-null function pointer")(lun)
               as libc::c_int != 0 as libc::c_int {
            SCSI_SenseCode(pdev, lun, 2 as libc::c_int as uint8_t,
                           0x3a as libc::c_int as uint8_t);
            return -(1 as libc::c_int) as int8_t
        }
        (*hmsc).scsi_blk_addr =
            ((*params.offset(2 as libc::c_int as isize) as libc::c_int) <<
                 24 as libc::c_int |
                 (*params.offset(3 as libc::c_int as isize) as libc::c_int) <<
                     16 as libc::c_int |
                 (*params.offset(4 as libc::c_int as isize) as libc::c_int) <<
                     8 as libc::c_int |
                 *params.offset(5 as libc::c_int as isize) as libc::c_int) as
                uint32_t;
        (*hmsc).scsi_blk_len =
            ((*params.offset(7 as libc::c_int as isize) as libc::c_int) <<
                 8 as libc::c_int |
                 *params.offset(8 as libc::c_int as isize) as libc::c_int) as
                uint32_t;
        if (SCSI_CheckAddressRange(pdev, lun, (*hmsc).scsi_blk_addr,
                                   (*hmsc).scsi_blk_len as uint16_t) as
                libc::c_int) < 0 as libc::c_int {
            return -(1 as libc::c_int) as int8_t
            /* error */
        }
        (*hmsc).bot_state = 2 as libc::c_int as uint8_t;
        (*hmsc).scsi_blk_addr =
            ((*hmsc).scsi_blk_addr as
                 libc::c_uint).wrapping_mul((*hmsc).scsi_blk_size as
                                                libc::c_uint) as uint32_t as
                uint32_t;
        (*hmsc).scsi_blk_len =
            ((*hmsc).scsi_blk_len as
                 libc::c_uint).wrapping_mul((*hmsc).scsi_blk_size as
                                                libc::c_uint) as uint32_t as
                uint32_t;
        if (*hmsc).cbw.dDataLength != (*hmsc).scsi_blk_len {
            SCSI_SenseCode(pdev, (*hmsc).cbw.bLUN,
                           5 as libc::c_int as uint8_t,
                           0x20 as libc::c_int as uint8_t);
            return -(1 as libc::c_int) as int8_t
        }
    }
    (*hmsc).bot_data_length = 512 as libc::c_int as uint16_t;
    return SCSI_ProcessRead(pdev, lun);
}
/* cases 4,5 : Hi <> Dn */
/* *
* @brief  SCSI_Write10
*         Process Write10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_Write10(mut pdev: *mut USBD_HandleTypeDef,
                                  mut lun: uint8_t, mut params: *mut uint8_t)
 -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    if (*hmsc).bot_state as libc::c_int == 0 as libc::c_int {
        /* Idle */
        /* case 8 : Hi <> Do */
        if (*hmsc).cbw.bmFlags as libc::c_int & 0x80 as libc::c_int ==
               0x80 as libc::c_int {
            SCSI_SenseCode(pdev, (*hmsc).cbw.bLUN,
                           5 as libc::c_int as uint8_t,
                           0x20 as libc::c_int as uint8_t);
            return -(1 as libc::c_int) as int8_t
        }
        if (*((*pdev).pMSC_UserData as
                  *mut USBD_StorageTypeDef)).IsReady.expect("non-null function pointer")(lun)
               as libc::c_int != 0 as libc::c_int {
            SCSI_SenseCode(pdev, lun, 2 as libc::c_int as uint8_t,
                           0x3a as libc::c_int as uint8_t);
            return -(1 as libc::c_int) as int8_t
        }
        if (*((*pdev).pMSC_UserData as
                  *mut USBD_StorageTypeDef)).IsWriteProtected.expect("non-null function pointer")(lun)
               as libc::c_int != 0 as libc::c_int {
            SCSI_SenseCode(pdev, lun, 2 as libc::c_int as uint8_t,
                           0x27 as libc::c_int as uint8_t);
            return -(1 as libc::c_int) as int8_t
        }
        (*hmsc).scsi_blk_addr =
            ((*params.offset(2 as libc::c_int as isize) as libc::c_int) <<
                 24 as libc::c_int |
                 (*params.offset(3 as libc::c_int as isize) as libc::c_int) <<
                     16 as libc::c_int |
                 (*params.offset(4 as libc::c_int as isize) as libc::c_int) <<
                     8 as libc::c_int |
                 *params.offset(5 as libc::c_int as isize) as libc::c_int) as
                uint32_t;
        (*hmsc).scsi_blk_len =
            ((*params.offset(7 as libc::c_int as isize) as libc::c_int) <<
                 8 as libc::c_int |
                 *params.offset(8 as libc::c_int as isize) as libc::c_int) as
                uint32_t;
        if (SCSI_CheckAddressRange(pdev, lun, (*hmsc).scsi_blk_addr,
                                   (*hmsc).scsi_blk_len as uint16_t) as
                libc::c_int) < 0 as libc::c_int {
            return -(1 as libc::c_int) as int8_t
            /* Check whether Media is ready */
            /* Check If media is write-protected */
            /* error */
        }
        (*hmsc).scsi_blk_addr =
            ((*hmsc).scsi_blk_addr as
                 libc::c_uint).wrapping_mul((*hmsc).scsi_blk_size as
                                                libc::c_uint) as uint32_t as
                uint32_t;
        (*hmsc).scsi_blk_len =
            ((*hmsc).scsi_blk_len as
                 libc::c_uint).wrapping_mul((*hmsc).scsi_blk_size as
                                                libc::c_uint) as uint32_t as
                uint32_t;
        if (*hmsc).cbw.dDataLength != (*hmsc).scsi_blk_len {
            SCSI_SenseCode(pdev, (*hmsc).cbw.bLUN,
                           5 as libc::c_int as uint8_t,
                           0x20 as libc::c_int as uint8_t);
            return -(1 as libc::c_int) as int8_t
        }
        (*hmsc).bot_state = 1 as libc::c_int as uint8_t;
        USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                               (*hmsc).bot_data.as_mut_ptr(),
                               if (*hmsc).scsi_blk_len <
                                      512 as libc::c_int as libc::c_uint {
                                   (*hmsc).scsi_blk_len
                               } else { 512 as libc::c_int as libc::c_uint }
                                   as uint16_t);
    } else {
        /* check if LBA address is in the right range */
        /* cases 3,11,13 : Hn,Ho <> D0 */
        /* Prepare EP to receive first data packet */
        /* Write Process ongoing */
        return SCSI_ProcessWrite(pdev, lun)
    }
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_Verify10
*         Process Verify10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
unsafe extern "C" fn SCSI_Verify10(mut pdev: *mut USBD_HandleTypeDef,
                                   mut lun: uint8_t, mut params: *mut uint8_t)
 -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    if *params.offset(1 as libc::c_int as isize) as libc::c_int &
           0x2 as libc::c_int == 0x2 as libc::c_int {
        SCSI_SenseCode(pdev, lun, 5 as libc::c_int as uint8_t,
                       0x24 as libc::c_int as uint8_t);
        return -(1 as libc::c_int) as int8_t
        /* Error, Verify Mode Not supported*/
    }
    if (SCSI_CheckAddressRange(pdev, lun, (*hmsc).scsi_blk_addr,
                               (*hmsc).scsi_blk_len as uint16_t) as
            libc::c_int) < 0 as libc::c_int {
        return -(1 as libc::c_int) as int8_t
        /* error */
    }
    (*hmsc).bot_data_length = 0 as libc::c_int as uint16_t;
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_CheckAddressRange
*         Check address range
* @param  lun: Logical unit number
* @param  blk_offset: first block address
* @param  blk_nbr: number of block to be processed
* @retval status
*/
unsafe extern "C" fn SCSI_CheckAddressRange(mut pdev: *mut USBD_HandleTypeDef,
                                            mut lun: uint8_t,
                                            mut blk_offset: uint32_t,
                                            mut blk_nbr: uint16_t) -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    if blk_offset.wrapping_add(blk_nbr as libc::c_uint) > (*hmsc).scsi_blk_nbr
       {
        SCSI_SenseCode(pdev, lun, 5 as libc::c_int as uint8_t,
                       0x21 as libc::c_int as uint8_t);
        return -(1 as libc::c_int) as int8_t
    }
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_ProcessRead
*         Handle Read Process
* @param  lun: Logical unit number
* @retval status
*/
unsafe extern "C" fn SCSI_ProcessRead(mut pdev: *mut USBD_HandleTypeDef,
                                      mut lun: uint8_t) -> int8_t {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    let mut len: uint32_t = 0;
    len =
        if (*hmsc).scsi_blk_len < 512 as libc::c_int as libc::c_uint {
            (*hmsc).scsi_blk_len
        } else { 512 as libc::c_int as libc::c_uint };
    if ((*((*pdev).pMSC_UserData as
               *mut USBD_StorageTypeDef)).Read.expect("non-null function pointer")(lun,
                                                                                   (*hmsc).bot_data.as_mut_ptr(),
                                                                                   (*hmsc).scsi_blk_addr.wrapping_div((*hmsc).scsi_blk_size
                                                                                                                          as
                                                                                                                          libc::c_uint),
                                                                                   len.wrapping_div((*hmsc).scsi_blk_size
                                                                                                        as
                                                                                                        libc::c_uint)
                                                                                       as
                                                                                       uint16_t)
            as libc::c_int) < 0 as libc::c_int {
        SCSI_SenseCode(pdev, lun, 4 as libc::c_int as uint8_t,
                       0x11 as libc::c_int as uint8_t);
        return -(1 as libc::c_int) as int8_t
    }
    USBD_LL_Transmit(pdev, 0x81 as libc::c_int as uint8_t,
                     (*hmsc).bot_data.as_mut_ptr(), len as uint16_t);
    (*hmsc).scsi_blk_addr =
        ((*hmsc).scsi_blk_addr as libc::c_uint).wrapping_add(len) as uint32_t
            as uint32_t;
    (*hmsc).scsi_blk_len =
        ((*hmsc).scsi_blk_len as libc::c_uint).wrapping_sub(len) as uint32_t
            as uint32_t;
    /* case 6 : Hi = Di */
    (*hmsc).csw.dDataResidue =
        ((*hmsc).csw.dDataResidue as libc::c_uint).wrapping_sub(len) as
            uint32_t as uint32_t;
    if (*hmsc).scsi_blk_len == 0 as libc::c_int as libc::c_uint {
        (*hmsc).bot_state = 3 as libc::c_int as uint8_t
    }
    return 0 as libc::c_int as int8_t;
}
/* *
* @brief  SCSI_ProcessWrite
*         Handle Write Process
* @param  lun: Logical unit number
* @retval status
*/
unsafe extern "C" fn SCSI_ProcessWrite(mut pdev: *mut USBD_HandleTypeDef,
                                       mut lun: uint8_t) -> int8_t {
    let mut len: uint32_t = 0;
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    len =
        if (*hmsc).scsi_blk_len < 512 as libc::c_int as libc::c_uint {
            (*hmsc).scsi_blk_len
        } else { 512 as libc::c_int as libc::c_uint };
    if ((*((*pdev).pMSC_UserData as
               *mut USBD_StorageTypeDef)).Write.expect("non-null function pointer")(lun,
                                                                                    (*hmsc).bot_data.as_mut_ptr(),
                                                                                    (*hmsc).scsi_blk_addr.wrapping_div((*hmsc).scsi_blk_size
                                                                                                                           as
                                                                                                                           libc::c_uint),
                                                                                    len.wrapping_div((*hmsc).scsi_blk_size
                                                                                                         as
                                                                                                         libc::c_uint)
                                                                                        as
                                                                                        uint16_t)
            as libc::c_int) < 0 as libc::c_int {
        SCSI_SenseCode(pdev, lun, 4 as libc::c_int as uint8_t,
                       0x3 as libc::c_int as uint8_t);
        return -(1 as libc::c_int) as int8_t
    }
    (*hmsc).scsi_blk_addr =
        ((*hmsc).scsi_blk_addr as libc::c_uint).wrapping_add(len) as uint32_t
            as uint32_t;
    (*hmsc).scsi_blk_len =
        ((*hmsc).scsi_blk_len as libc::c_uint).wrapping_sub(len) as uint32_t
            as uint32_t;
    /* case 12 : Ho = Do */
    (*hmsc).csw.dDataResidue =
        ((*hmsc).csw.dDataResidue as libc::c_uint).wrapping_sub(len) as
            uint32_t as uint32_t;
    if (*hmsc).scsi_blk_len == 0 as libc::c_int as libc::c_uint {
        MSC_BOT_SendCSW(pdev, 0 as libc::c_int as uint8_t);
    } else {
        /* Prepare EP to Receive next packet */
        USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                               (*hmsc).bot_data.as_mut_ptr(),
                               if (*hmsc).scsi_blk_len <
                                      512 as libc::c_int as libc::c_uint {
                                   (*hmsc).scsi_blk_len
                               } else { 512 as libc::c_int as libc::c_uint }
                                   as uint16_t);
    }
    return 0 as libc::c_int as int8_t;
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
