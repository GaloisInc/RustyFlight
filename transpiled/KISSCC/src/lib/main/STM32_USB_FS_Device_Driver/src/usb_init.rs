use ::libc;
extern "C" {
    #[no_mangle]
    static mut User_Standard_Requests: USER_STANDARD_REQUESTS;
    #[no_mangle]
    static mut Device_Property: DEVICE_PROP;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type _RESULT = libc::c_uint;
pub const USB_NOT_READY: _RESULT = 3;
pub const USB_UNSUPPORT: _RESULT = 2;
pub const USB_ERROR: _RESULT = 1;
pub const USB_SUCCESS: _RESULT = 0;
pub type RESULT = _RESULT;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _ENDPOINT_INFO {
    pub Usb_wLength: uint16_t,
    pub Usb_wOffset: uint16_t,
    pub PacketSize: uint16_t,
    pub CopyData: Option<unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t>,
}
pub type ENDPOINT_INFO = _ENDPOINT_INFO;
#[derive(Copy, Clone)]
#[repr(C)]
pub union uint16_t_uint8_t {
    pub w: uint16_t,
    pub bw: BW,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct BW {
    pub bb1: uint8_t,
    pub bb0: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _DEVICE_INFO {
    pub USBbmRequestType: uint8_t,
    pub USBbRequest: uint8_t,
    pub USBwValues: uint16_t_uint8_t,
    pub USBwIndexs: uint16_t_uint8_t,
    pub USBwLengths: uint16_t_uint8_t,
    pub ControlState: uint8_t,
    pub Current_Feature: uint8_t,
    pub Current_Configuration: uint8_t,
    pub Current_Interface: uint8_t,
    pub Current_AlternateSetting: uint8_t,
    pub Ctrl_Info: ENDPOINT_INFO,
}
pub type DEVICE_INFO = _DEVICE_INFO;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _DEVICE_PROP {
    pub Init: Option<unsafe extern "C" fn() -> ()>,
    pub Reset: Option<unsafe extern "C" fn() -> ()>,
    pub Process_Status_IN: Option<unsafe extern "C" fn() -> ()>,
    pub Process_Status_OUT: Option<unsafe extern "C" fn() -> ()>,
    pub Class_Data_Setup: Option<unsafe extern "C" fn(_: uint8_t) -> RESULT>,
    pub Class_NoData_Setup: Option<unsafe extern "C" fn(_: uint8_t)
                                       -> RESULT>,
    pub Class_Get_Interface_Setting: Option<unsafe extern "C" fn(_: uint8_t,
                                                                 _: uint8_t)
                                                -> RESULT>,
    pub GetDeviceDescriptor: Option<unsafe extern "C" fn(_: uint16_t)
                                        -> *mut uint8_t>,
    pub GetConfigDescriptor: Option<unsafe extern "C" fn(_: uint16_t)
                                        -> *mut uint8_t>,
    pub GetStringDescriptor: Option<unsafe extern "C" fn(_: uint16_t)
                                        -> *mut uint8_t>,
    pub RxEP_buffer: *mut libc::c_void,
    pub MaxPacketSize: uint8_t,
}
pub type DEVICE_PROP = _DEVICE_PROP;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USER_STANDARD_REQUESTS {
    pub User_GetConfiguration: Option<unsafe extern "C" fn() -> ()>,
    pub User_SetConfiguration: Option<unsafe extern "C" fn() -> ()>,
    pub User_GetInterface: Option<unsafe extern "C" fn() -> ()>,
    pub User_SetInterface: Option<unsafe extern "C" fn() -> ()>,
    pub User_GetStatus: Option<unsafe extern "C" fn() -> ()>,
    pub User_ClearFeature: Option<unsafe extern "C" fn() -> ()>,
    pub User_SetEndPointFeature: Option<unsafe extern "C" fn() -> ()>,
    pub User_SetDeviceFeature: Option<unsafe extern "C" fn() -> ()>,
    pub User_SetDeviceAddress: Option<unsafe extern "C" fn() -> ()>,
}
pub type USER_STANDARD_REQUESTS = _USER_STANDARD_REQUESTS;
/* *
  ******************************************************************************
  * @file    usb_init.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Initialization routines & global variables
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*  The number of current endpoint, it will be used to specify an endpoint */
#[no_mangle]
pub static mut EPindex: uint8_t = 0;
/*  The number of current device, it is an index to the Device_Table */
/* uint8_t	Device_no; */
/*  Points to the DEVICE_INFO structure of current device */
/*  The purpose of this register is to speed up the execution */
#[no_mangle]
pub static mut pInformation: *mut DEVICE_INFO =
    0 as *const DEVICE_INFO as *mut DEVICE_INFO;
/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */
#[no_mangle]
pub static mut pProperty: *mut DEVICE_PROP =
    0 as *const DEVICE_PROP as *mut DEVICE_PROP;
/*  Temporary save the state of Rx & Tx status. */
/*  Whenever the Rx or Tx state is changed, its value is saved */
/*  in this variable first and will be set to the EPRB or EPRA */
/*  at the end of interrupt process */
#[no_mangle]
pub static mut SaveState: uint16_t = 0;
#[no_mangle]
pub static mut wInterrupt_Mask: uint16_t = 0;
#[no_mangle]
pub static mut Device_Info: DEVICE_INFO =
    DEVICE_INFO{USBbmRequestType: 0,
                USBbRequest: 0,
                USBwValues: uint16_t_uint8_t{w: 0,},
                USBwIndexs: uint16_t_uint8_t{w: 0,},
                USBwLengths: uint16_t_uint8_t{w: 0,},
                ControlState: 0,
                Current_Feature: 0,
                Current_Configuration: 0,
                Current_Interface: 0,
                Current_AlternateSetting: 0,
                Ctrl_Info:
                    ENDPOINT_INFO{Usb_wLength: 0,
                                  Usb_wOffset: 0,
                                  PacketSize: 0,
                                  CopyData: None,},};
#[no_mangle]
pub static mut pUser_Standard_Requests: *mut USER_STANDARD_REQUESTS =
    0 as *const USER_STANDARD_REQUESTS as *mut USER_STANDARD_REQUESTS;
/* *
  ******************************************************************************
  * @file    usb_init.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Initialization routines & global variables
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
* Function Name  : USB_Init
* Description    : USB system initialization
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_Init() {
    pInformation = &mut Device_Info;
    (*pInformation).ControlState = 2 as libc::c_int as uint8_t;
    pProperty = &mut Device_Property;
    pUser_Standard_Requests = &mut User_Standard_Requests;
    /* Initialize devices one by one */
    (*pProperty).Init.expect("non-null function pointer")();
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
