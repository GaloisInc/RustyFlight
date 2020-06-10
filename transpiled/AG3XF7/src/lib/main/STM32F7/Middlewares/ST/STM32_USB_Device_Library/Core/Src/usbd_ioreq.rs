use ::libc;
extern "C" {
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
  * @file    usbd_ioreq.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the IO requests APIs for control endpoints.
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
/* * @defgroup USBD_IOREQ 
  * @brief control I/O requests module
  * @{
  */
/* * @defgroup USBD_IOREQ_Private_TypesDefinitions
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_IOREQ_Private_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_IOREQ_Private_Macros
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_IOREQ_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_IOREQ_Private_FunctionPrototypes
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup USBD_IOREQ_Private_Functions
  * @{
  */
/* *
* @brief  USBD_CtlSendData
*         send data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be sent
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_CtlSendData(mut pdev: *mut USBD_HandleTypeDef,
                                          mut pbuf: *mut uint8_t,
                                          mut len: uint16_t)
 -> USBD_StatusTypeDef {
    /* Set EP0 State */
    (*pdev).ep0_state = 2 as libc::c_int as uint32_t;
    (*pdev).ep_in[0 as libc::c_int as usize].total_length = len as uint32_t;
    (*pdev).ep_in[0 as libc::c_int as usize].rem_length = len as uint32_t;
    /* Start the transfer */
    USBD_LL_Transmit(pdev, 0 as libc::c_int as uint8_t, pbuf, len);
    return USBD_OK;
}
/* *
* @brief  USBD_CtlContinueSendData
*         continue sending data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be sent
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_CtlContinueSendData(mut pdev:
                                                      *mut USBD_HandleTypeDef,
                                                  mut pbuf: *mut uint8_t,
                                                  mut len: uint16_t)
 -> USBD_StatusTypeDef {
    /* Start the next transfer */
    USBD_LL_Transmit(pdev, 0 as libc::c_int as uint8_t, pbuf, len);
    return USBD_OK;
}
/* *
* @brief  USBD_CtlPrepareRx
*         receive data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be received
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_CtlPrepareRx(mut pdev: *mut USBD_HandleTypeDef,
                                           mut pbuf: *mut uint8_t,
                                           mut len: uint16_t)
 -> USBD_StatusTypeDef {
    /* Set EP0 State */
    (*pdev).ep0_state = 3 as libc::c_int as uint32_t;
    (*pdev).ep_out[0 as libc::c_int as usize].total_length = len as uint32_t;
    (*pdev).ep_out[0 as libc::c_int as usize].rem_length = len as uint32_t;
    /* Start the transfer */
    USBD_LL_PrepareReceive(pdev, 0 as libc::c_int as uint8_t, pbuf, len);
    return USBD_OK;
}
/* *
* @brief  USBD_CtlContinueRx
*         continue receive data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be received
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_CtlContinueRx(mut pdev: *mut USBD_HandleTypeDef,
                                            mut pbuf: *mut uint8_t,
                                            mut len: uint16_t)
 -> USBD_StatusTypeDef {
    USBD_LL_PrepareReceive(pdev, 0 as libc::c_int as uint8_t, pbuf, len);
    return USBD_OK;
}
/* *
* @brief  USBD_CtlSendStatus
*         send zero lzngth packet on the ctl pipe
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_CtlSendStatus(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    /* Set EP0 State */
    (*pdev).ep0_state = 4 as libc::c_int as uint32_t;
    /* Start the transfer */
    USBD_LL_Transmit(pdev, 0 as libc::c_int as uint8_t, 0 as *mut uint8_t,
                     0 as libc::c_int as uint16_t);
    return USBD_OK;
}
/* *
* @brief  USBD_CtlReceiveStatus
*         receive zero lzngth packet on the ctl pipe
* @param  pdev: device instance
* @retval status
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_CtlReceiveStatus(mut pdev:
                                                   *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    /* Set EP0 State */
    (*pdev).ep0_state = 5 as libc::c_int as uint32_t;
    /* Start the transfer */
    USBD_LL_PrepareReceive(pdev, 0 as libc::c_int as uint8_t,
                           0 as *mut uint8_t, 0 as libc::c_int as uint16_t);
    return USBD_OK;
}
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
/* *
* @brief  USBD_GetRxCount
*         returns the received data length
* @param  pdev: device instance
* @param  ep_addr: endpoint address
* @retval Rx Data blength
*/
#[no_mangle]
pub unsafe extern "C" fn USBD_GetRxCount(mut pdev: *mut USBD_HandleTypeDef,
                                         mut ep_addr: uint8_t) -> uint16_t {
    return USBD_LL_GetRxDataSize(pdev, ep_addr) as uint16_t;
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
