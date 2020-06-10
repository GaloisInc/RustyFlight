use ::libc;
extern "C" {
    #[no_mangle]
    fn USBD_LL_FlushEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_StallEP(pdev: *mut USBD_HandleTypeDef, ep_addr: uint8_t)
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
    #[no_mangle]
    fn SCSI_ProcessCmd(pdev: *mut USBD_HandleTypeDef, lun: uint8_t,
                       cmd: *mut uint8_t) -> int8_t;
    #[no_mangle]
    fn SCSI_SenseCode(pdev: *mut USBD_HandleTypeDef, lun: uint8_t,
                      sKey: uint8_t, ASC: uint8_t);
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
/* * @defgroup USBD_SCSI_Exported_TypesDefinitions
  * @{
  */
pub type USBD_SCSI_SenseTypeDef = _SENSE_ITEM;
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
/* * @defgroup MSC_BOT_Private_Functions
  * @{
  */
/* *
* @brief  MSC_BOT_Init
*         Initialize the BOT Process
* @param  pdev: device instance
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn MSC_BOT_Init(mut pdev: *mut USBD_HandleTypeDef) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    (*hmsc).bot_state = 0 as libc::c_int as uint8_t;
    (*hmsc).bot_status = 0 as libc::c_int as uint8_t;
    (*hmsc).scsi_sense_tail = 0 as libc::c_int as uint8_t;
    (*hmsc).scsi_sense_head = 0 as libc::c_int as uint8_t;
    (*((*pdev).pMSC_UserData as
           *mut USBD_StorageTypeDef)).Init.expect("non-null function pointer")(0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint8_t);
    USBD_LL_FlushEP(pdev, 0x1 as libc::c_int as uint8_t);
    USBD_LL_FlushEP(pdev, 0x81 as libc::c_int as uint8_t);
    /* Prapare EP to Receive First BOT Cmd */
    USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                           &mut (*hmsc).cbw as *mut USBD_MSC_BOT_CBWTypeDef as
                               *mut uint8_t, 31 as libc::c_int as uint16_t);
}
/* *
* @brief  MSC_BOT_Reset
*         Reset the BOT Machine
* @param  pdev: device instance
* @retval  None
*/
#[no_mangle]
pub unsafe extern "C" fn MSC_BOT_Reset(mut pdev: *mut USBD_HandleTypeDef) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    (*hmsc).bot_state = 0 as libc::c_int as uint8_t;
    (*hmsc).bot_status = 1 as libc::c_int as uint8_t;
    /* Prapare EP to Receive First BOT Cmd */
    USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                           &mut (*hmsc).cbw as *mut USBD_MSC_BOT_CBWTypeDef as
                               *mut uint8_t, 31 as libc::c_int as uint16_t);
}
/* *
* @brief  MSC_BOT_DeInit
*         Deinitialize the BOT Machine
* @param  pdev: device instance
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn MSC_BOT_DeInit(mut pdev: *mut USBD_HandleTypeDef) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    (*hmsc).bot_state = 0 as libc::c_int as uint8_t;
}
/* *
* @brief  MSC_BOT_DataIn
*         Handle BOT IN data stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn MSC_BOT_DataIn(mut pdev: *mut USBD_HandleTypeDef,
                                        mut epnum: uint8_t) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    match (*hmsc).bot_state as libc::c_int {
        2 => {
            if (SCSI_ProcessCmd(pdev, (*hmsc).cbw.bLUN,
                                &mut *(*hmsc).cbw.CB.as_mut_ptr().offset(0 as
                                                                             libc::c_int
                                                                             as
                                                                             isize))
                    as libc::c_int) < 0 as libc::c_int {
                MSC_BOT_SendCSW(pdev, 0x1 as libc::c_int as uint8_t);
            }
        }
        4 | 3 => { MSC_BOT_SendCSW(pdev, 0 as libc::c_int as uint8_t); }
        _ => { }
    };
}
/* *
* @brief  MSC_BOT_DataOut
*         Process MSC OUT data
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn MSC_BOT_DataOut(mut pdev: *mut USBD_HandleTypeDef,
                                         mut epnum: uint8_t) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    match (*hmsc).bot_state as libc::c_int {
        0 => { MSC_BOT_CBW_Decode(pdev); }
        1 => {
            if (SCSI_ProcessCmd(pdev, (*hmsc).cbw.bLUN,
                                &mut *(*hmsc).cbw.CB.as_mut_ptr().offset(0 as
                                                                             libc::c_int
                                                                             as
                                                                             isize))
                    as libc::c_int) < 0 as libc::c_int {
                MSC_BOT_SendCSW(pdev, 0x1 as libc::c_int as uint8_t);
            }
        }
        _ => { }
    };
}
/* *
  ******************************************************************************
  * @file    usbd_msc_bot.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the BOT protocol core functions.
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
/* * @defgroup MSC_BOT
  * @brief BOT protocol module
  * @{
  */
/* * @defgroup MSC_BOT_Private_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_BOT_Private_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_BOT_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_BOT_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MSC_BOT_Private_FunctionPrototypes
  * @{
  */
/* *
* @brief  MSC_BOT_CBW_Decode
*         Decode the CBW command and set the BOT state machine accordingly
* @param  pdev: device instance
* @retval None
*/
unsafe extern "C" fn MSC_BOT_CBW_Decode(mut pdev: *mut USBD_HandleTypeDef) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    (*hmsc).csw.dTag = (*hmsc).cbw.dTag;
    (*hmsc).csw.dDataResidue = (*hmsc).cbw.dDataLength;
    if USBD_LL_GetRxDataSize(pdev, 0x1 as libc::c_int as uint8_t) !=
           31 as libc::c_int as libc::c_uint ||
           (*hmsc).cbw.dSignature != 0x43425355 as libc::c_int as libc::c_uint
           || (*hmsc).cbw.bLUN as libc::c_int > 1 as libc::c_int ||
           ((*hmsc).cbw.bCBLength as libc::c_int) < 1 as libc::c_int ||
           (*hmsc).cbw.bCBLength as libc::c_int > 16 as libc::c_int {
        SCSI_SenseCode(pdev, (*hmsc).cbw.bLUN, 5 as libc::c_int as uint8_t,
                       0x20 as libc::c_int as uint8_t);
        (*hmsc).bot_status = 2 as libc::c_int as uint8_t;
        MSC_BOT_Abort(pdev);
    } else if (SCSI_ProcessCmd(pdev, (*hmsc).cbw.bLUN,
                               &mut *(*hmsc).cbw.CB.as_mut_ptr().offset(0 as
                                                                            libc::c_int
                                                                            as
                                                                            isize))
                   as libc::c_int) < 0 as libc::c_int {
        if (*hmsc).bot_state as libc::c_int == 5 as libc::c_int {
            MSC_BOT_SendCSW(pdev, 0x1 as libc::c_int as uint8_t);
        } else { MSC_BOT_Abort(pdev); }
    } else if (*hmsc).bot_state as libc::c_int != 2 as libc::c_int &&
                  (*hmsc).bot_state as libc::c_int != 1 as libc::c_int &&
                  (*hmsc).bot_state as libc::c_int != 3 as libc::c_int {
        if (*hmsc).bot_data_length as libc::c_int > 0 as libc::c_int {
            MSC_BOT_SendData(pdev, (*hmsc).bot_data.as_mut_ptr(),
                             (*hmsc).bot_data_length);
        } else if (*hmsc).bot_data_length as libc::c_int == 0 as libc::c_int {
            MSC_BOT_SendCSW(pdev, 0 as libc::c_int as uint8_t);
        }
    };
}
/*Burst xfer handled internally*/
/* *
* @brief  MSC_BOT_SendData
*         Send the requested data
* @param  pdev: device instance
* @param  buf: pointer to data buffer
* @param  len: Data Length
* @retval None
*/
unsafe extern "C" fn MSC_BOT_SendData(mut pdev: *mut USBD_HandleTypeDef,
                                      mut buf: *mut uint8_t,
                                      mut len: uint16_t) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    len =
        if (*hmsc).cbw.dDataLength < len as libc::c_uint {
            (*hmsc).cbw.dDataLength
        } else { len as libc::c_uint } as uint16_t;
    (*hmsc).csw.dDataResidue =
        ((*hmsc).csw.dDataResidue as
             libc::c_uint).wrapping_sub(len as libc::c_uint) as uint32_t as
            uint32_t;
    (*hmsc).csw.bStatus = 0 as libc::c_int as uint8_t;
    (*hmsc).bot_state = 4 as libc::c_int as uint8_t;
    USBD_LL_Transmit(pdev, 0x81 as libc::c_int as uint8_t, buf, len);
}
/* *
* @brief  MSC_BOT_SendCSW
*         Send the Command Status Wrapper
* @param  pdev: device instance
* @param  status : CSW status
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn MSC_BOT_SendCSW(mut pdev: *mut USBD_HandleTypeDef,
                                         mut CSW_Status: uint8_t) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    (*hmsc).csw.dSignature = 0x53425355 as libc::c_int as uint32_t;
    (*hmsc).csw.bStatus = CSW_Status;
    (*hmsc).bot_state = 0 as libc::c_int as uint8_t;
    USBD_LL_Transmit(pdev, 0x81 as libc::c_int as uint8_t,
                     &mut (*hmsc).csw as *mut USBD_MSC_BOT_CSWTypeDef as
                         *mut uint8_t, 13 as libc::c_int as uint16_t);
    /* Prepare EP to Receive next Cmd */
    USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                           &mut (*hmsc).cbw as *mut USBD_MSC_BOT_CBWTypeDef as
                               *mut uint8_t, 31 as libc::c_int as uint16_t);
}
/* *
* @brief  MSC_BOT_Abort
*         Abort the current transfer
* @param  pdev: device instance
* @retval status
*/
unsafe extern "C" fn MSC_BOT_Abort(mut pdev: *mut USBD_HandleTypeDef) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    if (*hmsc).cbw.bmFlags as libc::c_int == 0 as libc::c_int &&
           (*hmsc).cbw.dDataLength != 0 as libc::c_int as libc::c_uint &&
           (*hmsc).bot_status as libc::c_int == 0 as libc::c_int {
        USBD_LL_StallEP(pdev, 0x1 as libc::c_int as uint8_t);
    }
    USBD_LL_StallEP(pdev, 0x81 as libc::c_int as uint8_t);
    if (*hmsc).bot_status as libc::c_int == 2 as libc::c_int {
        USBD_LL_PrepareReceive(pdev, 0x1 as libc::c_int as uint8_t,
                               &mut (*hmsc).cbw as
                                   *mut USBD_MSC_BOT_CBWTypeDef as
                                   *mut uint8_t,
                               31 as libc::c_int as uint16_t);
    };
}
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
/* *
* @brief  MSC_BOT_CplClrFeature
*         Complete the clear feature request
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn MSC_BOT_CplClrFeature(mut pdev:
                                                   *mut USBD_HandleTypeDef,
                                               mut epnum: uint8_t) {
    let mut hmsc: *mut USBD_MSC_BOT_HandleTypeDef =
        (*pdev).pMSC_ClassData as *mut USBD_MSC_BOT_HandleTypeDef;
    if (*hmsc).bot_status as libc::c_int == 2 as libc::c_int {
        /* Bad CBW Signature */
        USBD_LL_StallEP(pdev, 0x81 as libc::c_int as uint8_t);
        (*hmsc).bot_status = 0 as libc::c_int as uint8_t
    } else if epnum as libc::c_int & 0x80 as libc::c_int ==
                  0x80 as libc::c_int &&
                  (*hmsc).bot_status as libc::c_int != 1 as libc::c_int {
        MSC_BOT_SendCSW(pdev, 0x1 as libc::c_int as uint8_t);
    };
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
