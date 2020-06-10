use ::libc;
extern "C" {
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    /* *
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Inc/usbd_cdc_interface.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-April-2016
  * @brief   Header for usbd_cdc_interface.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
    /* Define to prevent recursive inclusion -------------------------------------*/
    /* Includes ------------------------------------------------------------------*/
    /* Definition for TIMx clock resources */
    /* Periodically, the state of the buffer "UserTxBuffer" is checked.
   The period depends on CDC_POLLING_INTERVAL */
    /* in ms. The max is 65 and the min is 1 */
    /* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
    #[no_mangle]
    fn CDC_SetBaudRateCb(cb:
                             Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                                         _: uint32_t) -> ()>,
                         context: *mut libc::c_void);
    #[no_mangle]
    fn CDC_SetCtrlLineStateCb(cb:
                                  Option<unsafe extern "C" fn(_:
                                                                  *mut libc::c_void,
                                                              _: uint16_t)
                                             -> ()>,
                              context: *mut libc::c_void);
    #[no_mangle]
    fn CDC_BaudRate() -> uint32_t;
    #[no_mangle]
    fn usbIsConnected() -> uint8_t;
    #[no_mangle]
    fn usbIsConfigured() -> uint8_t;
    #[no_mangle]
    fn CDC_Receive_BytesAvailable() -> uint32_t;
    #[no_mangle]
    fn CDC_Receive_DATA(recvBuf: *mut uint8_t, len: uint32_t) -> uint32_t;
    #[no_mangle]
    fn CDC_Send_FreeBytes() -> uint32_t;
    #[no_mangle]
    fn CDC_Send_DATA(ptrBuffer: *const uint8_t, sendLength: uint32_t)
     -> uint32_t;
    #[no_mangle]
    static mut USBD_CDC_fops: USBD_CDC_ItfTypeDef;
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
    #[no_mangle]
    fn USBD_Init(pdev: *mut USBD_HandleTypeDef,
                 pdesc: *mut USBD_DescriptorsTypeDef, id: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_Start(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_RegisterClass(pdev: *mut USBD_HandleTypeDef,
                          pclass: *mut USBD_ClassTypeDef)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    static mut USBD_CDC: USBD_ClassTypeDef;
    /* *
  * @}
  */
    /* * @defgroup USB_CORE_Exported_Functions
  * @{
  */
    #[no_mangle]
    fn USBD_CDC_RegisterInterface(pdev: *mut USBD_HandleTypeDef,
                                  fops: *mut USBD_CDC_ItfTypeDef) -> uint8_t;
    /* *
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Inc/usbd_desc.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-April-2016
  * @brief   Header for usbd_desc.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
    /* Define to prevent recursive inclusion -------------------------------------*/
    /* Includes ------------------------------------------------------------------*/
    /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    #[no_mangle]
    static mut VCP_Desc: USBD_DescriptorsTypeDef;
    /*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
    #[no_mangle]
    fn usbGenerateDisconnectPulse();
    /*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */
    #[no_mangle]
    static mut USBD_HID_CDC: USBD_ClassTypeDef;
    #[no_mangle]
    static mut usbDevConfig_System: usbDev_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
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
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type USB_DEV = libc::c_uint;
pub const COMPOSITE: USB_DEV = 1;
pub const DEFAULT: USB_DEV = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usbDev_s {
    pub type_0: uint8_t,
    pub mscButtonPin: ioTag_t,
    pub mscButtonUsePullup: uint8_t,
}
pub type usbDev_t = usbDev_s;
// millisecond time
pub type timeMs_t = uint32_t;
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vcpPort_t {
    pub port: serialPort_t,
    pub txBuf: [uint8_t; 20],
    pub txAt: uint8_t,
    pub buffering: bool,
}
#[inline]
unsafe extern "C" fn usbDevConfig() -> *const usbDev_t {
    return &mut usbDevConfig_System;
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#[no_mangle]
pub static mut USBD_Device: USBD_HandleTypeDef =
    USBD_HandleTypeDef{id: 0,
                       dev_config: 0,
                       dev_default_config: 0,
                       dev_config_status: 0,
                       dev_speed: USBD_SPEED_HIGH,
                       ep_in:
                           [USBD_EndpointTypeDef{status: 0,
                                                 total_length: 0,
                                                 rem_length: 0,
                                                 maxpacket: 0,}; 15],
                       ep_out:
                           [USBD_EndpointTypeDef{status: 0,
                                                 total_length: 0,
                                                 rem_length: 0,
                                                 maxpacket: 0,}; 15],
                       ep0_state: 0,
                       ep0_data_len: 0,
                       dev_state: 0,
                       dev_old_state: 0,
                       dev_address: 0,
                       dev_connection_status: 0,
                       dev_test_mode: 0,
                       dev_remote_wakeup: 0,
                       request:
                           USBD_SetupReqTypedef{bmRequest: 0,
                                                bRequest: 0,
                                                wValue: 0,
                                                wIndex: 0,
                                                wLength: 0,},
                       pDesc:
                           0 as *const USBD_DescriptorsTypeDef as
                               *mut USBD_DescriptorsTypeDef,
                       pClass:
                           0 as *const USBD_ClassTypeDef as
                               *mut USBD_ClassTypeDef,
                       pCDC_ClassData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pCDC_UserData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pHID_ClassData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pHID_UserData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pMSC_ClassData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pMSC_UserData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pData: 0 as *const libc::c_void as *mut libc::c_void,};
static mut vcpPort: vcpPort_t =
    vcpPort_t{port:
                  serialPort_t{vTable: 0 as *const serialPortVTable,
                               mode: 0 as portMode_e,
                               options: SERIAL_NOT_INVERTED,
                               baudRate: 0,
                               rxBufferSize: 0,
                               txBufferSize: 0,
                               rxBuffer: 0 as *const uint8_t as *mut uint8_t,
                               txBuffer: 0 as *const uint8_t as *mut uint8_t,
                               rxBufferHead: 0,
                               rxBufferTail: 0,
                               txBufferHead: 0,
                               txBufferTail: 0,
                               rxCallback: None,
                               rxCallbackData:
                                   0 as *const libc::c_void as
                                       *mut libc::c_void,
                               identifier: 0,},
              txBuf: [0; 20],
              txAt: 0,
              buffering: false,};
unsafe extern "C" fn usbVcpSetBaudRate(mut instance: *mut serialPort_t,
                                       mut baudRate: uint32_t) {
    // TODO implement
}
unsafe extern "C" fn usbVcpSetMode(mut instance: *mut serialPort_t,
                                   mut mode: portMode_e) {
    // TODO implement
}
unsafe extern "C" fn usbVcpSetCtrlLineStateCb(mut instance: *mut serialPort_t,
                                              mut cb:
                                                  Option<unsafe extern "C" fn(_:
                                                                                  *mut libc::c_void,
                                                                              _:
                                                                                  uint16_t)
                                                             -> ()>,
                                              mut context:
                                                  *mut libc::c_void) {
    // Register upper driver control line state callback routine with USB driver
    CDC_SetCtrlLineStateCb(cb, context);
}
unsafe extern "C" fn usbVcpSetBaudRateCb(mut instance: *mut serialPort_t,
                                         mut cb:
                                             Option<unsafe extern "C" fn(_:
                                                                             *mut serialPort_t,
                                                                         _:
                                                                             uint32_t)
                                                        -> ()>,
                                         mut context: *mut serialPort_t) {
    // Register upper driver baud rate callback routine with USB driver
    CDC_SetBaudRateCb(::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                               *mut serialPort_t,
                                                                           _:
                                                                               uint32_t)
                                                          -> ()>,
                                               Option<unsafe extern "C" fn(_:
                                                                               *mut libc::c_void,
                                                                           _:
                                                                               uint32_t)
                                                          -> ()>>(cb),
                      context as *mut libc::c_void);
}
unsafe extern "C" fn isUsbVcpTransmitBufferEmpty(mut instance:
                                                     *const serialPort_t)
 -> bool {
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn usbVcpAvailable(mut instance: *const serialPort_t)
 -> uint32_t {
    return CDC_Receive_BytesAvailable();
}
unsafe extern "C" fn usbVcpRead(mut instance: *mut serialPort_t) -> uint8_t {
    let mut buf: [uint8_t; 1] = [0; 1];
    loop  {
        if CDC_Receive_DATA(buf.as_mut_ptr(), 1 as libc::c_int as uint32_t) !=
               0 {
            return buf[0 as libc::c_int as usize]
        }
    };
}
unsafe extern "C" fn usbVcpWriteBuf(mut instance: *mut serialPort_t,
                                    mut data: *const libc::c_void,
                                    mut count: libc::c_int) {
    if !(usbIsConnected() as libc::c_int != 0 &&
             usbIsConfigured() as libc::c_int != 0) {
        return
    }
    let mut start: uint32_t = millis();
    let mut p: *const uint8_t = data as *const uint8_t;
    while count > 0 as libc::c_int {
        let mut txed: uint32_t = CDC_Send_DATA(p, count as uint32_t);
        count =
            (count as libc::c_uint).wrapping_sub(txed) as libc::c_int as
                libc::c_int;
        p = p.offset(txed as isize);
        if millis().wrapping_sub(start) > 50 as libc::c_int as libc::c_uint {
            break ;
        }
    };
}
unsafe extern "C" fn usbVcpFlush(mut port: *mut vcpPort_t) -> bool {
    let mut count: uint32_t = (*port).txAt as uint32_t;
    (*port).txAt = 0 as libc::c_int as uint8_t;
    if count == 0 as libc::c_int as libc::c_uint {
        return 1 as libc::c_int != 0
    }
    if usbIsConnected() == 0 || usbIsConfigured() == 0 {
        return 0 as libc::c_int != 0
    }
    let mut start: uint32_t = millis();
    let mut p: *mut uint8_t = (*port).txBuf.as_mut_ptr();
    while count > 0 as libc::c_int as libc::c_uint {
        let mut txed: uint32_t = CDC_Send_DATA(p, count);
        count =
            (count as libc::c_uint).wrapping_sub(txed) as uint32_t as
                uint32_t;
        p = p.offset(txed as isize);
        if millis().wrapping_sub(start) > 50 as libc::c_int as libc::c_uint {
            break ;
        }
    }
    return count == 0 as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn usbVcpWrite(mut instance: *mut serialPort_t,
                                 mut c: uint8_t) {
    let mut port: *mut vcpPort_t =
        ({
             let mut __mptr: *const serialPort_t = instance;
             (__mptr as
                  *mut libc::c_char).offset(-(0 as libc::c_ulong as isize)) as
                 *mut vcpPort_t
         });
    let fresh0 = (*port).txAt;
    (*port).txAt = (*port).txAt.wrapping_add(1);
    (*port).txBuf[fresh0 as usize] = c;
    if !(*port).buffering ||
           (*port).txAt as libc::c_ulong >=
               (::core::mem::size_of::<[uint8_t; 20]>() as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                    as libc::c_ulong) {
        usbVcpFlush(port);
    };
}
unsafe extern "C" fn usbVcpBeginWrite(mut instance: *mut serialPort_t) {
    let mut port: *mut vcpPort_t =
        ({
             let mut __mptr: *const serialPort_t = instance;
             (__mptr as
                  *mut libc::c_char).offset(-(0 as libc::c_ulong as isize)) as
                 *mut vcpPort_t
         });
    (*port).buffering = 1 as libc::c_int != 0;
}
unsafe extern "C" fn usbTxBytesFree(mut instance: *const serialPort_t)
 -> uint32_t {
    return CDC_Send_FreeBytes();
}
unsafe extern "C" fn usbVcpEndWrite(mut instance: *mut serialPort_t) {
    let mut port: *mut vcpPort_t =
        ({
             let mut __mptr: *const serialPort_t = instance;
             (__mptr as
                  *mut libc::c_char).offset(-(0 as libc::c_ulong as isize)) as
                 *mut vcpPort_t
         });
    (*port).buffering = 0 as libc::c_int != 0;
    usbVcpFlush(port);
}
static mut usbVTable: [serialPortVTable; 1] =
    unsafe {
        [{
             let mut init =
                 serialPortVTable{serialWrite:
                                      Some(usbVcpWrite as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint8_t)
                                                   -> ()),
                                  serialTotalRxWaiting:
                                      Some(usbVcpAvailable as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialTotalTxFree:
                                      Some(usbTxBytesFree as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialRead:
                                      Some(usbVcpRead as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> uint8_t),
                                  serialSetBaudRate:
                                      Some(usbVcpSetBaudRate as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint32_t)
                                                   -> ()),
                                  isSerialTransmitBufferEmpty:
                                      Some(isUsbVcpTransmitBufferEmpty as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> bool),
                                  setMode:
                                      Some(usbVcpSetMode as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        portMode_e)
                                                   -> ()),
                                  setCtrlLineStateCb:
                                      Some(usbVcpSetCtrlLineStateCb as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        Option<unsafe extern "C" fn(_:
                                                                                                        *mut libc::c_void,
                                                                                                    _:
                                                                                                        uint16_t)
                                                                                   ->
                                                                                       ()>,
                                                                    _:
                                                                        *mut libc::c_void)
                                                   -> ()),
                                  setBaudRateCb:
                                      Some(usbVcpSetBaudRateCb as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        Option<unsafe extern "C" fn(_:
                                                                                                        *mut serialPort_t,
                                                                                                    _:
                                                                                                        uint32_t)
                                                                                   ->
                                                                                       ()>,
                                                                    _:
                                                                        *mut serialPort_t)
                                                   -> ()),
                                  writeBuf:
                                      Some(usbVcpWriteBuf as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        *const libc::c_void,
                                                                    _:
                                                                        libc::c_int)
                                                   -> ()),
                                  beginWrite:
                                      Some(usbVcpBeginWrite as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> ()),
                                  endWrite:
                                      Some(usbVcpEndWrite as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> ()),};
             init
         }]
    };
#[no_mangle]
pub unsafe extern "C" fn usbVcpOpen() -> *mut serialPort_t {
    let mut s: *mut vcpPort_t = 0 as *mut vcpPort_t;
    usbGenerateDisconnectPulse();
    IOInit(IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) <<
                           4 as libc::c_int | 11 as libc::c_int) as ioTag_t),
           OWNER_USB, 0 as libc::c_int as uint8_t);
    IOInit(IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) <<
                           4 as libc::c_int | 12 as libc::c_int) as ioTag_t),
           OWNER_USB, 0 as libc::c_int as uint8_t);
    /* Init Device Library */
    USBD_Init(&mut USBD_Device, &mut VCP_Desc, 0 as libc::c_int as uint8_t);
    /* Add Supported Class */
    if (*usbDevConfig()).type_0 as libc::c_int == COMPOSITE as libc::c_int {
        USBD_RegisterClass(&mut USBD_Device, &mut USBD_HID_CDC);
    } else { USBD_RegisterClass(&mut USBD_Device, &mut USBD_CDC); }
    /* HID Interface doesn't have any callbacks... */
    /* Add CDC Interface Class */
    USBD_CDC_RegisterInterface(&mut USBD_Device, &mut USBD_CDC_fops);
    /* Start Device Process */
    USBD_Start(&mut USBD_Device);
    s = &mut vcpPort;
    (*s).port.vTable = usbVTable.as_ptr();
    return s as *mut serialPort_t;
}
#[no_mangle]
pub unsafe extern "C" fn usbVcpGetBaudRate(mut instance: *mut serialPort_t)
 -> uint32_t {
    return CDC_BaudRate();
}
#[no_mangle]
pub unsafe extern "C" fn usbVcpIsConnected() -> uint8_t {
    return usbIsConnected();
}
