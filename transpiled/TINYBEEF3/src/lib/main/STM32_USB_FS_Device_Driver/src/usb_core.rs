use ::libc;
extern "C" {
    #[no_mangle]
    fn SetEPTxStatus(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn ClearDTOG_RX(_: uint8_t);
    #[no_mangle]
    fn ClearDTOG_TX(_: uint8_t);
    #[no_mangle]
    fn GetEPTxAddr(_: uint8_t) -> uint16_t;
    #[no_mangle]
    fn GetEPRxAddr(_: uint8_t) -> uint16_t;
    #[no_mangle]
    fn SetEPTxCount(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPRxCount(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn ByteSwap(_: uint16_t) -> uint16_t;
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
    /* External variables --------------------------------------------------------*/
/*  The number of current endpoint, it will be used to specify an endpoint */
    /*  The number of current device, it is an index to the Device_Table */
/*extern uint8_t	Device_no; */
/*  Points to the DEVICE_INFO structure of current device */
/*  The purpose of this register is to speed up the execution */
    #[no_mangle]
    static mut pInformation: *mut DEVICE_INFO;
    /* cells saving status during interrupt servicing */
    #[no_mangle]
    static mut SaveTState: uint16_t;
    #[no_mangle]
    static mut SaveRState: uint16_t;
    #[no_mangle]
    static mut Device_Property: DEVICE_PROP;
    /* *
  ******************************************************************************
  * @file    usb_mem.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Utility prototypes functions for memory/PMA transfers
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
    #[no_mangle]
    fn UserToPMABufferCopy(pbUsrBuf: *const uint8_t, wPMABufAddr: uint16_t,
                           wNBytes: uint16_t);
    /*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */
    #[no_mangle]
    static mut pProperty: *mut DEVICE_PROP;
    /*  Temporary save the state of Rx & Tx status. */
/*  Whenever the Rx or Tx state is changed, its value is saved */
/*  in this variable first and will be set to the EPRB or EPRA */
/*  at the end of interrupt process */
    #[no_mangle]
    static mut pUser_Standard_Requests: *mut USER_STANDARD_REQUESTS;
    #[no_mangle]
    static mut Device_Table: DEVICE;
    #[no_mangle]
    fn PMAToUserBufferCopy(pbUsrBuf: *mut uint8_t, wPMABufAddr: uint16_t,
                           wNBytes: uint16_t);
}
/* *
  ******************************************************************************
  * @file    usb_type.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Type definitions used by the USB Library
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
pub type boolean = libc::c_uint;
pub const TRUE: boolean = 1;
pub const FALSE: boolean = 0;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    usb_def.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Definitions related to USB Core
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
pub type _RECIPIENT_TYPE = libc::c_uint;
/* Recipient endpoint */
pub const OTHER_RECIPIENT: _RECIPIENT_TYPE = 3;
/* Recipient interface */
pub const ENDPOINT_RECIPIENT: _RECIPIENT_TYPE = 2;
/* Recipient device */
pub const INTERFACE_RECIPIENT: _RECIPIENT_TYPE = 1;
pub const DEVICE_RECIPIENT: _RECIPIENT_TYPE = 0;
pub type _STANDARD_REQUESTS = libc::c_uint;
/* Total number of Standard request */
pub const SYNCH_FRAME: _STANDARD_REQUESTS = 12;
pub const TOTAL_sREQUEST: _STANDARD_REQUESTS = 12;
pub const SET_INTERFACE: _STANDARD_REQUESTS = 11;
pub const GET_INTERFACE: _STANDARD_REQUESTS = 10;
pub const SET_CONFIGURATION: _STANDARD_REQUESTS = 9;
pub const GET_CONFIGURATION: _STANDARD_REQUESTS = 8;
pub const SET_DESCRIPTOR: _STANDARD_REQUESTS = 7;
pub const GET_DESCRIPTOR: _STANDARD_REQUESTS = 6;
pub const SET_ADDRESS: _STANDARD_REQUESTS = 5;
pub const RESERVED2: _STANDARD_REQUESTS = 4;
pub const SET_FEATURE: _STANDARD_REQUESTS = 3;
pub const RESERVED1: _STANDARD_REQUESTS = 2;
pub const CLEAR_FEATURE: _STANDARD_REQUESTS = 1;
pub const GET_STATUS: _STANDARD_REQUESTS = 0;
/* Definition of "USBwValue" */
pub type _DESCRIPTOR_TYPE = libc::c_uint;
pub const ENDPOINT_DESCRIPTOR: _DESCRIPTOR_TYPE = 5;
pub const INTERFACE_DESCRIPTOR: _DESCRIPTOR_TYPE = 4;
pub const STRING_DESCRIPTOR: _DESCRIPTOR_TYPE = 3;
pub const CONFIG_DESCRIPTOR: _DESCRIPTOR_TYPE = 2;
pub const DEVICE_DESCRIPTOR: _DESCRIPTOR_TYPE = 1;
/* Feature selector of a SET_FEATURE or CLEAR_FEATURE */
pub type _FEATURE_SELECTOR = libc::c_uint;
pub const DEVICE_REMOTE_WAKEUP: _FEATURE_SELECTOR = 1;
pub const ENDPOINT_STALL: _FEATURE_SELECTOR = 0;
pub type _CONTROL_STATE = libc::c_uint;
pub const PAUSE: _CONTROL_STATE = 9;
pub const STALLED: _CONTROL_STATE = 8;
pub const WAIT_STATUS_OUT: _CONTROL_STATE = 7;
pub const WAIT_STATUS_IN: _CONTROL_STATE = 6;
pub const LAST_OUT_DATA: _CONTROL_STATE = 5;
pub const LAST_IN_DATA: _CONTROL_STATE = 4;
pub const OUT_DATA: _CONTROL_STATE = 3;
pub const IN_DATA: _CONTROL_STATE = 2;
pub const SETTING_UP: _CONTROL_STATE = 1;
pub const WAIT_SETUP: _CONTROL_STATE = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OneDescriptor {
    pub Descriptor: *mut uint8_t,
    pub Descriptor_Size: uint16_t,
}
pub type ONE_DESCRIPTOR = OneDescriptor;
pub type PONE_DESCRIPTOR = *mut OneDescriptor;
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
pub struct _DEVICE {
    pub Total_Endpoint: uint8_t,
    pub Total_Configuration: uint8_t,
}
pub type DEVICE = _DEVICE;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub b: *mut uint8_t,
    pub w: *mut uint16_t,
}
/* *
  ******************************************************************************
  * @file    usb_core.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Standard protocol processing (USB v2.0)
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
/* Reverse bb0 & bb1 */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#[no_mangle]
pub static mut StatusInfo: uint16_t_uint8_t = uint16_t_uint8_t{w: 0,};
#[no_mangle]
pub static mut Data_Mul_MaxPacketSize: boolean = FALSE;
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
* Function Name  : Standard_GetConfiguration.
* Description    : Return the current configuration variable address.
* Input          : Length - How many bytes are needed.
* Output         : None.
* Return         : Return 1 , if the request is invalid when "Length" is 0.
*                  Return "Buffer" if the "Length" is not 0.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_GetConfiguration(mut Length: uint16_t)
 -> *mut uint8_t {
    if Length as libc::c_int == 0 as libc::c_int {
        (*pInformation).Ctrl_Info.Usb_wLength =
            ::core::mem::size_of::<uint8_t>() as libc::c_ulong as uint16_t;
        return 0 as *mut uint8_t
    }
    (*pUser_Standard_Requests).User_GetConfiguration.expect("non-null function pointer")();
    return &mut (*pInformation).Current_Configuration as *mut uint8_t;
}
/* ******************************************************************************
* Function Name  : Standard_SetConfiguration.
* Description    : This routine is called to set the configuration value
*                  Then each class should configure device itself.
* Input          : None.
* Output         : None.
* Return         : Return USB_SUCCESS, if the request is performed.
*                  Return USB_UNSUPPORT, if the request is invalid.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_SetConfiguration() -> RESULT {
    if (*pInformation).USBwValues.bw.bb0 as libc::c_int <=
           Device_Table.Total_Configuration as libc::c_int &&
           (*pInformation).USBwValues.bw.bb1 as libc::c_int ==
               0 as libc::c_int &&
           (*pInformation).USBwIndexs.w as libc::c_int == 0 as libc::c_int {
        /*call Back usb spec 2.0*/
        (*pInformation).Current_Configuration =
            (*pInformation).USBwValues.bw.bb0;
        (*pUser_Standard_Requests).User_SetConfiguration.expect("non-null function pointer")();
        return USB_SUCCESS
    } else { return USB_UNSUPPORT };
}
/* ******************************************************************************
* Function Name  : Standard_GetInterface.
* Description    : Return the Alternate Setting of the current interface.
* Input          : Length - How many bytes are needed.
* Output         : None.
* Return         : Return 0, if the request is invalid when "Length" is 0.
*                  Return "Buffer" if the "Length" is not 0.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_GetInterface(mut Length: uint16_t)
 -> *mut uint8_t {
    if Length as libc::c_int == 0 as libc::c_int {
        (*pInformation).Ctrl_Info.Usb_wLength =
            ::core::mem::size_of::<uint8_t>() as libc::c_ulong as uint16_t;
        return 0 as *mut uint8_t
    }
    (*pUser_Standard_Requests).User_GetInterface.expect("non-null function pointer")();
    return &mut (*pInformation).Current_AlternateSetting as *mut uint8_t;
}
/* ******************************************************************************
* Function Name  : Standard_SetInterface.
* Description    : This routine is called to set the interface.
*                  Then each class should configure the interface them self.
* Input          : None.
* Output         : None.
* Return         : - Return USB_SUCCESS, if the request is performed.
*                  - Return USB_UNSUPPORT, if the request is invalid.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_SetInterface() -> RESULT {
    let mut Re: RESULT = USB_SUCCESS;
    /*Test if the specified Interface and Alternate Setting are supported by
    the application Firmware*/
    Re =
        Some((*pProperty).Class_Get_Interface_Setting.expect("non-null function pointer")).expect("non-null function pointer")((*pInformation).USBwIndexs.bw.bb0,
                                                                                                                               (*pInformation).USBwValues.bw.bb0);
    if (*pInformation).Current_Configuration as libc::c_int !=
           0 as libc::c_int {
        if Re as libc::c_uint != USB_SUCCESS as libc::c_int as libc::c_uint ||
               (*pInformation).USBwIndexs.bw.bb1 as libc::c_int !=
                   0 as libc::c_int ||
               (*pInformation).USBwValues.bw.bb1 as libc::c_int !=
                   0 as libc::c_int {
            return USB_UNSUPPORT
        } else {
            if Re as libc::c_uint ==
                   USB_SUCCESS as libc::c_int as libc::c_uint {
                (*pUser_Standard_Requests).User_SetInterface.expect("non-null function pointer")();
                (*pInformation).Current_Interface =
                    (*pInformation).USBwIndexs.bw.bb0;
                (*pInformation).Current_AlternateSetting =
                    (*pInformation).USBwValues.bw.bb0;
                return USB_SUCCESS
            }
        }
    }
    return USB_UNSUPPORT;
}
/* ******************************************************************************
* Function Name  : Standard_GetStatus.
* Description    : Copy the device request data to "StatusInfo buffer".
* Input          : - Length - How many bytes are needed.
* Output         : None.
* Return         : Return 0, if the request is at end of data block,
*                  or is invalid when "Length" is 0.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_GetStatus(mut Length: uint16_t)
 -> *mut uint8_t {
    if Length as libc::c_int == 0 as libc::c_int {
        (*pInformation).Ctrl_Info.Usb_wLength = 2 as libc::c_int as uint16_t;
        return 0 as *mut uint8_t
    }
    /* Reset Status Information */
    StatusInfo.w = 0 as libc::c_int as uint16_t;
    if (*pInformation).USBbmRequestType as libc::c_int &
           (0x60 as libc::c_int | 0x1f as libc::c_int) ==
           0 as libc::c_int | DEVICE_RECIPIENT as libc::c_int {
        /*Get Device Status */
        let mut Feature: uint8_t = (*pInformation).Current_Feature;
        /* Remote Wakeup enabled */
        if Feature as libc::c_int & (1 as libc::c_int) << 5 as libc::c_int !=
               0 {
            StatusInfo.bw.bb1 =
                (StatusInfo.bw.bb1 as libc::c_int |
                     (1 as libc::c_int) << 1 as libc::c_int) as uint8_t
        } else {
            StatusInfo.bw.bb1 =
                (StatusInfo.bw.bb1 as libc::c_int &
                     ((1 as libc::c_int) << 1 as libc::c_int ^
                          255 as libc::c_int)) as uint8_t
        }
        /* Bus-powered */
        if Feature as libc::c_int & (1 as libc::c_int) << 6 as libc::c_int !=
               0 {
            StatusInfo.bw.bb1 =
                (StatusInfo.bw.bb1 as libc::c_int |
                     (1 as libc::c_int) << 0 as libc::c_int) as uint8_t
        } else {
            /* Self-powered */
            StatusInfo.bw.bb1 =
                (StatusInfo.bw.bb1 as libc::c_int &
                     ((1 as libc::c_int) << 0 as libc::c_int ^
                          255 as libc::c_int)) as uint8_t
        }
    } else if (*pInformation).USBbmRequestType as libc::c_int &
                  (0x60 as libc::c_int | 0x1f as libc::c_int) ==
                  0 as libc::c_int | INTERFACE_RECIPIENT as libc::c_int {
        return &mut StatusInfo as *mut uint16_t_uint8_t as *mut uint8_t
    } else {
        /*Interface Status*/
        /*Get EndPoint Status*/
        if (*pInformation).USBbmRequestType as libc::c_int &
               (0x60 as libc::c_int | 0x1f as libc::c_int) ==
               0 as libc::c_int | ENDPOINT_RECIPIENT as libc::c_int {
            let mut Related_Endpoint: uint8_t = 0;
            let mut wIndex0: uint8_t = (*pInformation).USBwIndexs.bw.bb0;
            Related_Endpoint =
                (wIndex0 as libc::c_int & 0xf as libc::c_int) as uint8_t;
            if wIndex0 as libc::c_int & (1 as libc::c_int) << 7 as libc::c_int
                   != 0 {
                /* IN endpoint */
                if *(0x40005c00 as libc::c_long as
                         *mut libc::c_uint).offset(Related_Endpoint as
                                                       libc::c_int as isize)
                       as uint16_t as libc::c_int & 0x30 as libc::c_int ==
                       0x10 as libc::c_int {
                    StatusInfo.bw.bb1 =
                        (StatusInfo.bw.bb1 as libc::c_int |
                             (1 as libc::c_int) << 0 as libc::c_int) as
                            uint8_t
                    /* IN Endpoint stalled */
                }
            } else if *(0x40005c00 as libc::c_long as
                            *mut libc::c_uint).offset(Related_Endpoint as
                                                          libc::c_int as
                                                          isize) as uint16_t
                          as libc::c_int & 0x3000 as libc::c_int ==
                          0x1000 as libc::c_int {
                StatusInfo.bw.bb1 =
                    (StatusInfo.bw.bb1 as libc::c_int |
                         (1 as libc::c_int) << 0 as libc::c_int) as uint8_t
                /* OUT endpoint */
                /* OUT Endpoint stalled */
            }
        } else { return 0 as *mut uint8_t }
    }
    (*pUser_Standard_Requests).User_GetStatus.expect("non-null function pointer")();
    return &mut StatusInfo as *mut uint16_t_uint8_t as *mut uint8_t;
}
/* ******************************************************************************
* Function Name  : Standard_ClearFeature.
* Description    : Clear or disable a specific feature.
* Input          : None.
* Output         : None.
* Return         : - Return USB_SUCCESS, if the request is performed.
*                  - Return USB_UNSUPPORT, if the request is invalid.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_ClearFeature() -> RESULT {
    let mut Type_Rec: uint32_t =
        ((*pInformation).USBbmRequestType as libc::c_int &
             (0x60 as libc::c_int | 0x1f as libc::c_int)) as uint32_t;
    let mut Status: uint32_t = 0;
    if Type_Rec ==
           (0 as libc::c_int | DEVICE_RECIPIENT as libc::c_int) as
               libc::c_uint {
        /*Device Clear Feature*/
        (*pInformation).Current_Feature =
            ((*pInformation).Current_Feature as libc::c_int &
                 ((1 as libc::c_int) << 5 as libc::c_int ^
                      255 as libc::c_int)) as uint8_t;
        return USB_SUCCESS
    } else {
        if Type_Rec ==
               (0 as libc::c_int | ENDPOINT_RECIPIENT as libc::c_int) as
                   libc::c_uint {
            /*EndPoint Clear Feature*/
            let mut pDev: *mut DEVICE = 0 as *mut DEVICE;
            let mut Related_Endpoint: uint32_t = 0;
            let mut wIndex0: uint32_t = 0;
            let mut rEP: uint32_t = 0;
            if (*pInformation).USBwValues.w as libc::c_int !=
                   ENDPOINT_STALL as libc::c_int ||
                   (*pInformation).USBwIndexs.bw.bb1 as libc::c_int !=
                       0 as libc::c_int {
                return USB_UNSUPPORT
            }
            pDev = &mut Device_Table;
            wIndex0 = (*pInformation).USBwIndexs.bw.bb0 as uint32_t;
            rEP = wIndex0 & !(0x80 as libc::c_int) as libc::c_uint;
            Related_Endpoint =
                (0 as libc::c_int as uint8_t as
                     libc::c_uint).wrapping_add(rEP);
            if (*pInformation).USBwIndexs.bw.bb0 as libc::c_int &
                   (1 as libc::c_int) << 7 as libc::c_int != 0 {
                /*Get Status of endpoint & stall the request if the related_ENdpoint
      is Disabled*/
                Status =
                    (*(0x40005c00 as libc::c_long as
                           *mut libc::c_uint).offset(Related_Endpoint as
                                                         isize) as uint16_t as
                         libc::c_int & 0x30 as libc::c_int) as uint32_t
            } else {
                Status =
                    (*(0x40005c00 as libc::c_long as
                           *mut libc::c_uint).offset(Related_Endpoint as
                                                         isize) as uint16_t as
                         libc::c_int & 0x3000 as libc::c_int) as uint32_t
            }
            if rEP >= (*pDev).Total_Endpoint as libc::c_uint ||
                   Status == 0 as libc::c_int as libc::c_uint ||
                   (*pInformation).Current_Configuration as libc::c_int ==
                       0 as libc::c_int {
                return USB_UNSUPPORT
            }
            if wIndex0 & 0x80 as libc::c_int as libc::c_uint != 0 {
                /* IN endpoint */
                if *(0x40005c00 as libc::c_long as
                         *mut libc::c_uint).offset(Related_Endpoint as isize)
                       as uint16_t as libc::c_int & 0x30 as libc::c_int ==
                       0x10 as libc::c_int {
                    ClearDTOG_TX(Related_Endpoint as uint8_t);
                    SetEPTxStatus(Related_Endpoint as uint8_t,
                                  0x30 as libc::c_int as uint16_t);
                }
            } else if *(0x40005c00 as libc::c_long as
                            *mut libc::c_uint).offset(Related_Endpoint as
                                                          isize) as uint16_t
                          as libc::c_int & 0x3000 as libc::c_int ==
                          0x1000 as libc::c_int {
                if Related_Endpoint ==
                       0 as libc::c_int as uint8_t as libc::c_uint {
                    /* OUT endpoint */
                    /* After clear the STALL, enable the default endpoint receiver */
                    SetEPRxCount(Related_Endpoint as uint8_t,
                                 Device_Property.MaxPacketSize as uint16_t);
                    let mut _wRegVal: uint16_t = 0;
                    _wRegVal =
                        (*(0x40005c00 as libc::c_long as
                               *mut libc::c_uint).offset(Related_Endpoint as
                                                             isize) as
                             uint16_t as libc::c_int &
                             (0x3000 as libc::c_int |
                                  (0x8000 as libc::c_int |
                                       0x800 as libc::c_int |
                                       0x600 as libc::c_int |
                                       0x100 as libc::c_int |
                                       0x80 as libc::c_int |
                                       0xf as libc::c_int))) as uint16_t;
                    if 0x1000 as libc::c_int & 0x3000 as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal =
                            (_wRegVal as libc::c_int ^ 0x1000 as libc::c_int)
                                as uint16_t
                    }
                    if 0x2000 as libc::c_int & 0x3000 as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal =
                            (_wRegVal as libc::c_int ^ 0x2000 as libc::c_int)
                                as uint16_t
                    }
                    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                     *mut libc::c_uint).offset(Related_Endpoint
                                                                                   as
                                                                                   isize),
                                                (_wRegVal as libc::c_int |
                                                     0x8000 as libc::c_int |
                                                     0x80 as libc::c_int) as
                                                    uint16_t as libc::c_uint)
                } else {
                    ClearDTOG_RX(Related_Endpoint as uint8_t);
                    let mut _wRegVal_0: uint16_t = 0;
                    _wRegVal_0 =
                        (*(0x40005c00 as libc::c_long as
                               *mut libc::c_uint).offset(Related_Endpoint as
                                                             isize) as
                             uint16_t as libc::c_int &
                             (0x3000 as libc::c_int |
                                  (0x8000 as libc::c_int |
                                       0x800 as libc::c_int |
                                       0x600 as libc::c_int |
                                       0x100 as libc::c_int |
                                       0x80 as libc::c_int |
                                       0xf as libc::c_int))) as uint16_t;
                    if 0x1000 as libc::c_int & 0x3000 as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_0 =
                            (_wRegVal_0 as libc::c_int ^
                                 0x1000 as libc::c_int) as uint16_t
                    }
                    if 0x2000 as libc::c_int & 0x3000 as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_0 =
                            (_wRegVal_0 as libc::c_int ^
                                 0x2000 as libc::c_int) as uint16_t
                    }
                    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                     *mut libc::c_uint).offset(Related_Endpoint
                                                                                   as
                                                                                   isize),
                                                (_wRegVal_0 as libc::c_int |
                                                     0x8000 as libc::c_int |
                                                     0x80 as libc::c_int) as
                                                    uint16_t as libc::c_uint)
                }
            }
            (*pUser_Standard_Requests).User_ClearFeature.expect("non-null function pointer")();
            return USB_SUCCESS
        }
    }
    return USB_UNSUPPORT;
}
/* ******************************************************************************
* Function Name  : Standard_SetEndPointFeature
* Description    : Set or enable a specific feature of EndPoint
* Input          : None.
* Output         : None.
* Return         : - Return USB_SUCCESS, if the request is performed.
*                  - Return USB_UNSUPPORT, if the request is invalid.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_SetEndPointFeature() -> RESULT {
    let mut wIndex0: uint32_t = 0;
    let mut Related_Endpoint: uint32_t = 0;
    let mut rEP: uint32_t = 0;
    let mut Status: uint32_t = 0;
    wIndex0 = (*pInformation).USBwIndexs.bw.bb0 as uint32_t;
    rEP = wIndex0 & !(0x80 as libc::c_int) as libc::c_uint;
    Related_Endpoint =
        (0 as libc::c_int as uint8_t as libc::c_uint).wrapping_add(rEP);
    if (*pInformation).USBwIndexs.bw.bb0 as libc::c_int &
           (1 as libc::c_int) << 7 as libc::c_int != 0 {
        /* get Status of endpoint & stall the request if the related_ENdpoint
    is Disabled*/
        Status =
            (*(0x40005c00 as libc::c_long as
                   *mut libc::c_uint).offset(Related_Endpoint as isize) as
                 uint16_t as libc::c_int & 0x30 as libc::c_int) as uint32_t
    } else {
        Status =
            (*(0x40005c00 as libc::c_long as
                   *mut libc::c_uint).offset(Related_Endpoint as isize) as
                 uint16_t as libc::c_int & 0x3000 as libc::c_int) as uint32_t
    }
    if Related_Endpoint >= Device_Table.Total_Endpoint as libc::c_uint ||
           (*pInformation).USBwValues.w as libc::c_int != 0 as libc::c_int ||
           Status == 0 as libc::c_int as libc::c_uint ||
           (*pInformation).Current_Configuration as libc::c_int ==
               0 as libc::c_int {
        return USB_UNSUPPORT
    } else {
        if wIndex0 & 0x80 as libc::c_int as libc::c_uint != 0 {
            /* IN endpoint */
            let mut _wRegVal: uint16_t = 0;
            _wRegVal =
                (*(0x40005c00 as libc::c_long as
                       *mut libc::c_uint).offset(Related_Endpoint as isize) as
                     uint16_t as libc::c_int &
                     (0x30 as libc::c_int |
                          (0x8000 as libc::c_int | 0x800 as libc::c_int |
                               0x600 as libc::c_int | 0x100 as libc::c_int |
                               0x80 as libc::c_int | 0xf as libc::c_int))) as
                    uint16_t;
            if 0x10 as libc::c_int & 0x10 as libc::c_int != 0 as libc::c_int {
                _wRegVal =
                    (_wRegVal as libc::c_int ^ 0x10 as libc::c_int) as
                        uint16_t
            }
            if 0x20 as libc::c_int & 0x10 as libc::c_int != 0 as libc::c_int {
                _wRegVal =
                    (_wRegVal as libc::c_int ^ 0x20 as libc::c_int) as
                        uint16_t
            }
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                             *mut libc::c_uint).offset(Related_Endpoint
                                                                           as
                                                                           isize),
                                        (_wRegVal as libc::c_int |
                                             0x8000 as libc::c_int |
                                             0x80 as libc::c_int) as uint16_t
                                            as libc::c_uint)
        } else {
            /* OUT endpoint */
            let mut _wRegVal_0: uint16_t = 0;
            _wRegVal_0 =
                (*(0x40005c00 as libc::c_long as
                       *mut libc::c_uint).offset(Related_Endpoint as isize) as
                     uint16_t as libc::c_int &
                     (0x3000 as libc::c_int |
                          (0x8000 as libc::c_int | 0x800 as libc::c_int |
                               0x600 as libc::c_int | 0x100 as libc::c_int |
                               0x80 as libc::c_int | 0xf as libc::c_int))) as
                    uint16_t;
            if 0x1000 as libc::c_int & 0x1000 as libc::c_int !=
                   0 as libc::c_int {
                _wRegVal_0 =
                    (_wRegVal_0 as libc::c_int ^ 0x1000 as libc::c_int) as
                        uint16_t
            }
            if 0x2000 as libc::c_int & 0x1000 as libc::c_int !=
                   0 as libc::c_int {
                _wRegVal_0 =
                    (_wRegVal_0 as libc::c_int ^ 0x2000 as libc::c_int) as
                        uint16_t
            }
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                             *mut libc::c_uint).offset(Related_Endpoint
                                                                           as
                                                                           isize),
                                        (_wRegVal_0 as libc::c_int |
                                             0x8000 as libc::c_int |
                                             0x80 as libc::c_int) as uint16_t
                                            as libc::c_uint)
        }
    }
    (*pUser_Standard_Requests).User_SetEndPointFeature.expect("non-null function pointer")();
    return USB_SUCCESS;
}
/* ******************************************************************************
* Function Name  : Standard_SetDeviceFeature.
* Description    : Set or enable a specific feature of Device.
* Input          : None.
* Output         : None.
* Return         : - Return USB_SUCCESS, if the request is performed.
*                  - Return USB_UNSUPPORT, if the request is invalid.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_SetDeviceFeature() -> RESULT {
    (*pInformation).Current_Feature =
        ((*pInformation).Current_Feature as libc::c_int |
             (1 as libc::c_int) << 5 as libc::c_int) as uint8_t;
    (*pUser_Standard_Requests).User_SetDeviceFeature.expect("non-null function pointer")();
    return USB_SUCCESS;
}
/* ******************************************************************************
* Function Name  : Standard_GetDescriptorData.
* Description    : Standard_GetDescriptorData is used for descriptors transfer.
*                : This routine is used for the descriptors resident in Flash
*                  or RAM
*                  pDesc can be in either Flash or RAM
*                  The purpose of this routine is to have a versatile way to
*                  response descriptors request. It allows user to generate
*                  certain descriptors with software or read descriptors from
*                  external storage part by part.
* Input          : - Length - Length of the data in this transfer.
*                  - pDesc - A pointer points to descriptor struct.
*                  The structure gives the initial address of the descriptor and
*                  its original size.
* Output         : None.
* Return         : Address of a part of the descriptor pointed by the Usb_
*                  wOffset The buffer pointed by this address contains at least
*                  Length bytes.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Standard_GetDescriptorData(mut Length: uint16_t,
                                                    mut pDesc:
                                                        *mut ONE_DESCRIPTOR)
 -> *mut uint8_t {
    let mut wOffset: uint32_t = 0;
    wOffset = (*pInformation).Ctrl_Info.Usb_wOffset as uint32_t;
    if Length as libc::c_int == 0 as libc::c_int {
        (*pInformation).Ctrl_Info.Usb_wLength =
            ((*pDesc).Descriptor_Size as libc::c_uint).wrapping_sub(wOffset)
                as uint16_t;
        return 0 as *mut uint8_t
    }
    return (*pDesc).Descriptor.offset(wOffset as isize);
}
/* Private function prototypes -----------------------------------------------*/
/* ******************************************************************************
* Function Name  : DataStageOut.
* Description    : Data stage of a Control Write Transfer.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn DataStageOut() {
    let mut pEPinfo: *mut ENDPOINT_INFO =
        &mut (*pInformation).Ctrl_Info; /* re-enable for next data reception */
    let mut save_rLength: uint32_t = 0;
    save_rLength = (*pEPinfo).Usb_wLength as uint32_t;
    if (*pEPinfo).CopyData.is_some() && save_rLength != 0 {
        let mut Buffer: *mut uint8_t = 0 as *mut uint8_t;
        let mut Length: uint32_t = 0;
        Length = (*pEPinfo).PacketSize as uint32_t;
        if Length > save_rLength { Length = save_rLength }
        Buffer =
            Some((*pEPinfo).CopyData.expect("non-null function pointer")).expect("non-null function pointer")(Length
                                                                                                                  as
                                                                                                                  uint16_t);
        (*pEPinfo).Usb_wLength =
            ((*pEPinfo).Usb_wLength as libc::c_uint).wrapping_sub(Length) as
                uint16_t as uint16_t;
        (*pEPinfo).Usb_wOffset =
            ((*pEPinfo).Usb_wOffset as libc::c_uint).wrapping_add(Length) as
                uint16_t as uint16_t;
        PMAToUserBufferCopy(Buffer, GetEPRxAddr(0 as libc::c_int as uint8_t),
                            Length as uint16_t);
    }
    if (*pEPinfo).Usb_wLength as libc::c_int != 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut SaveRState as *mut uint16_t,
                                    0x3000 as libc::c_int as uint16_t);
        SetEPTxCount(0 as libc::c_int as uint8_t,
                     0 as libc::c_int as uint16_t);
        ::core::ptr::write_volatile(&mut SaveTState as *mut uint16_t,
                                    0x30 as libc::c_int as uint16_t)
        /* Expect the host to abort the data OUT stage */
    }
    /* Set the next State*/
    if (*pEPinfo).Usb_wLength as libc::c_int >=
           (*pEPinfo).PacketSize as libc::c_int {
        (*pInformation).ControlState = OUT_DATA as libc::c_int as uint8_t
    } else if (*pEPinfo).Usb_wLength as libc::c_int > 0 as libc::c_int {
        (*pInformation).ControlState = LAST_OUT_DATA as libc::c_int as uint8_t
    } else if (*pEPinfo).Usb_wLength as libc::c_int == 0 as libc::c_int {
        (*pInformation).ControlState =
            WAIT_STATUS_IN as libc::c_int as uint8_t;
        ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                               0x50 as libc::c_int as
                                                   libc::c_long) as
                                              *mut libc::c_uint) as uint16_t
                                            as libc::c_int &
                                            !(0x7 as libc::c_int)) +
                                           0 as libc::c_int as uint8_t as
                                               libc::c_int * 8 as libc::c_int
                                           + 2 as libc::c_int) *
                                          2 as libc::c_int) as libc::c_long +
                                         0x40006000 as libc::c_long) as
                                        *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut SaveTState as *mut uint16_t,
                                    0x30 as libc::c_int as uint16_t)
    };
}
/* ******************************************************************************
* Function Name  : DataStageIn.
* Description    : Data stage of a Control Read Transfer.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn DataStageIn() {
    let mut pEPinfo: *mut ENDPOINT_INFO = &mut (*pInformation).Ctrl_Info;
    let mut save_wLength: uint32_t = (*pEPinfo).Usb_wLength as uint32_t;
    let mut ControlState: uint32_t = (*pInformation).ControlState as uint32_t;
    let mut DataBuffer: *mut uint8_t = 0 as *mut uint8_t;
    let mut Length: uint32_t = 0;
    if save_wLength == 0 as libc::c_int as libc::c_uint &&
           ControlState == LAST_IN_DATA as libc::c_int as libc::c_uint {
        if Data_Mul_MaxPacketSize as libc::c_uint ==
               TRUE as libc::c_int as libc::c_uint {
            /* No more data to send and empty packet */
            ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                                   0x50 as libc::c_int as
                                                       libc::c_long) as
                                                  *mut libc::c_uint) as
                                                uint16_t as libc::c_int &
                                                !(0x7 as libc::c_int)) +
                                               0 as libc::c_int as uint8_t as
                                                   libc::c_int *
                                                   8 as libc::c_int +
                                               2 as libc::c_int) *
                                              2 as libc::c_int) as
                                             libc::c_long +
                                             0x40006000 as libc::c_long) as
                                            *mut uint32_t,
                                        0 as libc::c_int as uint32_t);
            ::core::ptr::write_volatile(&mut SaveTState as *mut uint16_t,
                                        0x30 as libc::c_int as uint16_t);
            ControlState = LAST_IN_DATA as libc::c_int as uint32_t;
            Data_Mul_MaxPacketSize = FALSE
        } else {
            /* No more data to send so STALL the TX Status*/
            ControlState =
                WAIT_STATUS_OUT as libc::c_int as
                    uint32_t; /* Expect the host to abort the data IN stage */
            ::core::ptr::write_volatile(&mut SaveTState as *mut uint16_t,
                                        0x10 as libc::c_int as uint16_t)
        }
    } else {
        Length = (*pEPinfo).PacketSize as uint32_t;
        ControlState =
            if save_wLength <= Length {
                LAST_IN_DATA as libc::c_int
            } else { IN_DATA as libc::c_int } as uint32_t;
        if Length > save_wLength { Length = save_wLength }
        DataBuffer =
            Some((*pEPinfo).CopyData.expect("non-null function pointer")).expect("non-null function pointer")(Length
                                                                                                                  as
                                                                                                                  uint16_t);
        UserToPMABufferCopy(DataBuffer,
                            GetEPTxAddr(0 as libc::c_int as uint8_t),
                            Length as uint16_t);
        SetEPTxCount(0 as libc::c_int as uint8_t, Length as uint16_t);
        (*pEPinfo).Usb_wLength =
            ((*pEPinfo).Usb_wLength as libc::c_uint).wrapping_sub(Length) as
                uint16_t as uint16_t;
        (*pEPinfo).Usb_wOffset =
            ((*pEPinfo).Usb_wOffset as libc::c_uint).wrapping_add(Length) as
                uint16_t as uint16_t;
        ::core::ptr::write_volatile(&mut SaveTState as *mut uint16_t,
                                    0x30 as libc::c_int as uint16_t);
        ::core::ptr::write_volatile(&mut SaveRState as *mut uint16_t,
                                    0x3000 as libc::c_int as uint16_t)
    }
    (*pInformation).ControlState = ControlState as uint8_t;
}
/* ******************************************************************************
* Function Name  : NoData_Setup0.
* Description    : Proceed the processing of setup request without data stage.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn NoData_Setup0() {
    let mut current_block: u64;
    let mut Result: RESULT = USB_UNSUPPORT;
    let mut RequestNo: uint32_t = (*pInformation).USBbRequest as uint32_t;
    let mut ControlState: uint32_t = 0;
    if (*pInformation).USBbmRequestType as libc::c_int &
           (0x60 as libc::c_int | 0x1f as libc::c_int) ==
           0 as libc::c_int | DEVICE_RECIPIENT as libc::c_int {
        /* Device Request*/
    /* SET_CONFIGURATION*/
        if RequestNo == SET_CONFIGURATION as libc::c_int as libc::c_uint {
            Result = Standard_SetConfiguration();
            current_block = 13131896068329595644;
        } else if RequestNo == SET_ADDRESS as libc::c_int as libc::c_uint {
            if (*pInformation).USBwValues.bw.bb0 as libc::c_int >
                   127 as libc::c_int ||
                   (*pInformation).USBwValues.bw.bb1 as libc::c_int !=
                       0 as libc::c_int ||
                   (*pInformation).USBwIndexs.w as libc::c_int !=
                       0 as libc::c_int ||
                   (*pInformation).Current_Configuration as libc::c_int !=
                       0 as libc::c_int {
                /*SET ADDRESS*/
                /* Device Address should be 127 or less*/
                ControlState = STALLED as libc::c_int as uint32_t;
                current_block = 17452698571840859962;
            } else {
                Result = USB_SUCCESS;
                current_block = 13131896068329595644;
            }
        } else {
            /*SET FEATURE for Device*/
            if RequestNo == SET_FEATURE as libc::c_int as libc::c_uint {
                if (*pInformation).USBwValues.bw.bb0 as libc::c_int ==
                       DEVICE_REMOTE_WAKEUP as libc::c_int &&
                       (*pInformation).USBwIndexs.w as libc::c_int ==
                           0 as libc::c_int {
                    Result = Standard_SetDeviceFeature()
                } else { Result = USB_UNSUPPORT }
            } else if RequestNo ==
                          CLEAR_FEATURE as libc::c_int as libc::c_uint {
                if (*pInformation).USBwValues.bw.bb0 as libc::c_int ==
                       DEVICE_REMOTE_WAKEUP as libc::c_int &&
                       (*pInformation).USBwIndexs.w as libc::c_int ==
                           0 as libc::c_int &&
                       (*pInformation).Current_Feature as libc::c_int &
                           (1 as libc::c_int) << 5 as libc::c_int != 0 {
                    Result = Standard_ClearFeature()
                } else { Result = USB_UNSUPPORT }
            }
            current_block = 13131896068329595644;
        }
    } else {
        /*Clear FEATURE for Device */
        /* Interface Request*/
        if (*pInformation).USBbmRequestType as libc::c_int &
               (0x60 as libc::c_int | 0x1f as libc::c_int) ==
               0 as libc::c_int | INTERFACE_RECIPIENT as libc::c_int {
            /*SET INTERFACE*/
            if RequestNo == SET_INTERFACE as libc::c_int as libc::c_uint {
                Result = Standard_SetInterface()
            }
        } else if (*pInformation).USBbmRequestType as libc::c_int &
                      (0x60 as libc::c_int | 0x1f as libc::c_int) ==
                      0 as libc::c_int | ENDPOINT_RECIPIENT as libc::c_int {
            /* EndPoint Request*/
            /*CLEAR FEATURE for EndPoint*/
            if RequestNo == CLEAR_FEATURE as libc::c_int as libc::c_uint {
                Result = Standard_ClearFeature()
            } else if RequestNo == SET_FEATURE as libc::c_int as libc::c_uint
             {
                Result = Standard_SetEndPointFeature()
            }
        } else { Result = USB_UNSUPPORT }
        current_block = 13131896068329595644;
    }
    match current_block {
        13131896068329595644 => {
            if Result as libc::c_uint !=
                   USB_SUCCESS as libc::c_int as libc::c_uint {
                Result =
                    Some((*pProperty).Class_NoData_Setup.expect("non-null function pointer")).expect("non-null function pointer")(RequestNo
                                                                                                                                      as
                                                                                                                                      uint8_t);
                if Result as libc::c_uint ==
                       USB_NOT_READY as libc::c_int as libc::c_uint {
                    ControlState = PAUSE as libc::c_int as uint32_t;
                    current_block = 17452698571840859962;
                } else { current_block = 3938820862080741272; }
            } else { current_block = 3938820862080741272; }
            match current_block {
                17452698571840859962 => { }
                _ => {
                    if Result as libc::c_uint !=
                           USB_SUCCESS as libc::c_int as libc::c_uint {
                        ControlState = STALLED as libc::c_int as uint32_t
                    } else {
                        /* SET FEATURE for EndPoint*/
                        ControlState =
                            WAIT_STATUS_IN as libc::c_int as
                                uint32_t; /* After no data stage SETUP */
                        ::core::ptr::write_volatile(((((*((0x40005c00 as
                                                               libc::c_long +
                                                               0x50 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long)
                                                              as
                                                              *mut libc::c_uint)
                                                            as uint16_t as
                                                            libc::c_int &
                                                            !(0x7 as
                                                                  libc::c_int))
                                                           +
                                                           0 as libc::c_int as
                                                               uint8_t as
                                                               libc::c_int *
                                                               8 as
                                                                   libc::c_int
                                                           + 2 as libc::c_int)
                                                          * 2 as libc::c_int)
                                                         as libc::c_long +
                                                         0x40006000 as
                                                             libc::c_long) as
                                                        *mut uint32_t,
                                                    0 as libc::c_int as
                                                        uint32_t);
                        ::core::ptr::write_volatile(&mut SaveTState as
                                                        *mut uint16_t,
                                                    0x30 as libc::c_int as
                                                        uint16_t)
                    }
                }
            }
        }
        _ => { }
    }
    (*pInformation).ControlState = ControlState as uint8_t;
}
/* ******************************************************************************
* Function Name  : Data_Setup0.
* Description    : Proceed the processing of setup request with data stage.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn Data_Setup0() {
    let mut CopyRoutine:
            Option<unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t> = None;
    let mut Result: RESULT = USB_SUCCESS;
    let mut Request_No: uint32_t = (*pInformation).USBbRequest as uint32_t;
    let mut Related_Endpoint: uint32_t = 0;
    let mut Reserved: uint32_t = 0;
    let mut wOffset: uint32_t = 0;
    let mut Status: uint32_t = 0;
    CopyRoutine = None;
    wOffset = 0 as libc::c_int as uint32_t;
    /*GET DESCRIPTOR*/
    if Request_No == GET_DESCRIPTOR as libc::c_int as libc::c_uint {
        if (*pInformation).USBbmRequestType as libc::c_int &
               (0x60 as libc::c_int | 0x1f as libc::c_int) ==
               0 as libc::c_int | DEVICE_RECIPIENT as libc::c_int {
            let mut wValue1: uint8_t = (*pInformation).USBwValues.bw.bb1;
            if wValue1 as libc::c_int == DEVICE_DESCRIPTOR as libc::c_int {
                CopyRoutine = (*pProperty).GetDeviceDescriptor
            } else if wValue1 as libc::c_int ==
                          CONFIG_DESCRIPTOR as libc::c_int {
                CopyRoutine = (*pProperty).GetConfigDescriptor
            } else if wValue1 as libc::c_int ==
                          STRING_DESCRIPTOR as libc::c_int {
                CopyRoutine = (*pProperty).GetStringDescriptor
            }
            /* End of GET_DESCRIPTOR */
        }
    } else if Request_No == GET_STATUS as libc::c_int as libc::c_uint &&
                  (*pInformation).USBwValues.w as libc::c_int ==
                      0 as libc::c_int &&
                  (*pInformation).USBwLengths.w as libc::c_int ==
                      0x2 as libc::c_int &&
                  (*pInformation).USBwIndexs.bw.bb1 as libc::c_int ==
                      0 as libc::c_int {
        /*GET STATUS*/
        /* GET STATUS for Device*/
        if (*pInformation).USBbmRequestType as libc::c_int &
               (0x60 as libc::c_int | 0x1f as libc::c_int) ==
               0 as libc::c_int | DEVICE_RECIPIENT as libc::c_int &&
               (*pInformation).USBwIndexs.w as libc::c_int == 0 as libc::c_int
           {
            CopyRoutine =
                Some(Standard_GetStatus as
                         unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t)
        } else if (*pInformation).USBbmRequestType as libc::c_int &
                      (0x60 as libc::c_int | 0x1f as libc::c_int) ==
                      0 as libc::c_int | INTERFACE_RECIPIENT as libc::c_int {
            if Some((*pProperty).Class_Get_Interface_Setting.expect("non-null function pointer")).expect("non-null function pointer")((*pInformation).USBwIndexs.bw.bb0,
                                                                                                                                      0
                                                                                                                                          as
                                                                                                                                          libc::c_int
                                                                                                                                          as
                                                                                                                                          uint8_t)
                   as libc::c_uint ==
                   USB_SUCCESS as libc::c_int as libc::c_uint &&
                   (*pInformation).Current_Configuration as libc::c_int !=
                       0 as libc::c_int {
                CopyRoutine =
                    Some(Standard_GetStatus as
                             unsafe extern "C" fn(_: uint16_t)
                                 -> *mut uint8_t)
            }
        } else if (*pInformation).USBbmRequestType as libc::c_int &
                      (0x60 as libc::c_int | 0x1f as libc::c_int) ==
                      0 as libc::c_int | ENDPOINT_RECIPIENT as libc::c_int {
            Related_Endpoint =
                ((*pInformation).USBwIndexs.bw.bb0 as libc::c_int &
                     0xf as libc::c_int) as uint32_t;
            Reserved =
                ((*pInformation).USBwIndexs.bw.bb0 as libc::c_int &
                     0x70 as libc::c_int) as uint32_t;
            if (*pInformation).USBwIndexs.bw.bb0 as libc::c_int &
                   (1 as libc::c_int) << 7 as libc::c_int != 0 {
                /* GET STATUS for Interface*/
                /* GET STATUS for EndPoint*/
                /*Get Status of endpoint & stall the request if the related_ENdpoint
        is Disabled*/
                Status =
                    (*(0x40005c00 as libc::c_long as
                           *mut libc::c_uint).offset(Related_Endpoint as
                                                         isize) as uint16_t as
                         libc::c_int & 0x30 as libc::c_int) as uint32_t
            } else {
                Status =
                    (*(0x40005c00 as libc::c_long as
                           *mut libc::c_uint).offset(Related_Endpoint as
                                                         isize) as uint16_t as
                         libc::c_int & 0x3000 as libc::c_int) as uint32_t
            }
            if Related_Endpoint < Device_Table.Total_Endpoint as libc::c_uint
                   && Reserved == 0 as libc::c_int as libc::c_uint &&
                   Status != 0 as libc::c_int as libc::c_uint {
                CopyRoutine =
                    Some(Standard_GetStatus as
                             unsafe extern "C" fn(_: uint16_t)
                                 -> *mut uint8_t)
            }
        }
    } else if Request_No == GET_CONFIGURATION as libc::c_int as libc::c_uint {
        if (*pInformation).USBbmRequestType as libc::c_int &
               (0x60 as libc::c_int | 0x1f as libc::c_int) ==
               0 as libc::c_int | DEVICE_RECIPIENT as libc::c_int {
            CopyRoutine =
                Some(Standard_GetConfiguration as
                         unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t)
        }
    } else if Request_No == GET_INTERFACE as libc::c_int as libc::c_uint {
        if (*pInformation).USBbmRequestType as libc::c_int &
               (0x60 as libc::c_int | 0x1f as libc::c_int) ==
               0 as libc::c_int | INTERFACE_RECIPIENT as libc::c_int &&
               (*pInformation).Current_Configuration as libc::c_int !=
                   0 as libc::c_int &&
               (*pInformation).USBwValues.w as libc::c_int == 0 as libc::c_int
               &&
               (*pInformation).USBwIndexs.bw.bb1 as libc::c_int ==
                   0 as libc::c_int &&
               (*pInformation).USBwLengths.w as libc::c_int ==
                   0x1 as libc::c_int &&
               Some((*pProperty).Class_Get_Interface_Setting.expect("non-null function pointer")).expect("non-null function pointer")((*pInformation).USBwIndexs.bw.bb0,
                                                                                                                                      0
                                                                                                                                          as
                                                                                                                                          libc::c_int
                                                                                                                                          as
                                                                                                                                          uint8_t)
                   as libc::c_uint ==
                   USB_SUCCESS as libc::c_int as libc::c_uint {
            CopyRoutine =
                Some(Standard_GetInterface as
                         unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t)
        }
    }
    if CopyRoutine.is_some() {
        (*pInformation).Ctrl_Info.Usb_wOffset = wOffset as uint16_t;
        (*pInformation).Ctrl_Info.CopyData = CopyRoutine;
        /*GET CONFIGURATION*/
        /*GET INTERFACE*/
        /* sb in the original the cast to word was directly */
    /* now the cast is made step by step */
        Some(CopyRoutine.expect("non-null function pointer")).expect("non-null function pointer")(0
                                                                                                      as
                                                                                                      libc::c_int
                                                                                                      as
                                                                                                      uint16_t);
        Result = USB_SUCCESS
    } else {
        Result =
            Some((*pProperty).Class_Data_Setup.expect("non-null function pointer")).expect("non-null function pointer")((*pInformation).USBbRequest);
        if Result as libc::c_uint ==
               USB_NOT_READY as libc::c_int as libc::c_uint {
            (*pInformation).ControlState = PAUSE as libc::c_int as uint8_t;
            return
        }
    }
    if (*pInformation).Ctrl_Info.Usb_wLength as libc::c_int ==
           0xffff as libc::c_int {
        /* Data is not ready, wait it */
        (*pInformation).ControlState = PAUSE as libc::c_int as uint8_t;
        return
    }
    if Result as libc::c_uint == USB_UNSUPPORT as libc::c_int as libc::c_uint
           ||
           (*pInformation).Ctrl_Info.Usb_wLength as libc::c_int ==
               0 as libc::c_int {
        /* Unsupported request */
        (*pInformation).ControlState = STALLED as libc::c_int as uint8_t;
        return
    }
    if (*pInformation).USBbmRequestType as libc::c_int &
           (1 as libc::c_int) << 7 as libc::c_int != 0 {
        /* Device ==> Host */
        let mut wLength: uint32_t = (*pInformation).USBwLengths.w as uint32_t;
        /* Restrict the data length to be the one host asks for */
        if (*pInformation).Ctrl_Info.Usb_wLength as libc::c_uint > wLength {
            (*pInformation).Ctrl_Info.Usb_wLength = wLength as uint16_t
        } else if ((*pInformation).Ctrl_Info.Usb_wLength as libc::c_int) <
                      (*pInformation).USBwLengths.w as libc::c_int {
            if ((*pInformation).Ctrl_Info.Usb_wLength as libc::c_int) <
                   (*pProperty).MaxPacketSize as libc::c_int {
                Data_Mul_MaxPacketSize = FALSE
            } else if (*pInformation).Ctrl_Info.Usb_wLength as libc::c_int %
                          (*pProperty).MaxPacketSize as libc::c_int ==
                          0 as libc::c_int {
                Data_Mul_MaxPacketSize = TRUE
            }
        }
        (*pInformation).Ctrl_Info.PacketSize =
            (*pProperty).MaxPacketSize as uint16_t;
        DataStageIn();
    } else {
        (*pInformation).ControlState = OUT_DATA as libc::c_int as uint8_t;
        ::core::ptr::write_volatile(&mut SaveRState as *mut uint16_t,
                                    0x3000 as libc::c_int as uint16_t)
        /* enable for next data reception */
    };
}
/* ******************************************************************************
* Function Name  : Setup0_Process
* Description    : Get the device request data and dispatch to individual process.
* Input          : None.
* Output         : None.
* Return         : Post0_Process.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Setup0_Process() -> uint8_t {
    let mut pBuf: C2RustUnnamed =
        C2RustUnnamed{b: 0 as *mut uint8_t,}; /* *2 for 32 bits addr */
    let mut offset: uint16_t =
        1 as libc::c_int as uint16_t; /* bmRequestType */
    pBuf.b =
        ((*(((((*((0x40005c00 as libc::c_long +
                       0x50 as libc::c_int as libc::c_long) as
                      *mut libc::c_uint) as uint16_t as libc::c_int &
                    !(0x7 as libc::c_int)) +
                   0 as libc::c_int as uint8_t as libc::c_int *
                       8 as libc::c_int + 4 as libc::c_int) *
                  2 as libc::c_int) as libc::c_long +
                 0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t as
              libc::c_int * 2 as libc::c_int) as
             *mut uint8_t).offset(0x40006000 as libc::c_long as isize);
    if (*pInformation).ControlState as libc::c_int != PAUSE as libc::c_int {
        let fresh0 = pBuf.b;
        pBuf.b = pBuf.b.offset(1);
        (*pInformation).USBbmRequestType = *fresh0;
        /* wLength */
        let fresh1 = pBuf.b; /* bRequest */
        pBuf.b =
            pBuf.b.offset(1); /* word not accessed because of 32 bits addressing */
        (*pInformation).USBbRequest = *fresh1; /* wValue */
        pBuf.w =
            pBuf.w.offset(offset as libc::c_int as
                              isize); /* word not accessed because of 32 bits addressing */
        let fresh2 = pBuf.w; /* wIndex */
        pBuf.w =
            pBuf.w.offset(1); /* word not accessed because of 32 bits addressing */
        (*pInformation).USBwValues.w = ByteSwap(*fresh2);
        pBuf.w = pBuf.w.offset(offset as libc::c_int as isize);
        let fresh3 = pBuf.w;
        pBuf.w = pBuf.w.offset(1);
        (*pInformation).USBwIndexs.w = ByteSwap(*fresh3);
        pBuf.w = pBuf.w.offset(offset as libc::c_int as isize);
        (*pInformation).USBwLengths.w = *pBuf.w
    }
    (*pInformation).ControlState = SETTING_UP as libc::c_int as uint8_t;
    if (*pInformation).USBwLengths.w as libc::c_int == 0 as libc::c_int {
        /* Setup with no data stage */
        NoData_Setup0();
    } else {
        /* Setup with data stage */
        Data_Setup0();
    }
    return Post0_Process();
}
/* ******************************************************************************
* Function Name  : In0_Process
* Description    : Process the IN token on all default endpoint.
* Input          : None.
* Output         : None.
* Return         : Post0_Process.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn In0_Process() -> uint8_t {
    let mut ControlState: uint32_t = (*pInformation).ControlState as uint32_t;
    if ControlState == IN_DATA as libc::c_int as libc::c_uint ||
           ControlState == LAST_IN_DATA as libc::c_int as libc::c_uint {
        DataStageIn();
        /* ControlState may be changed outside the function */
        ControlState = (*pInformation).ControlState as uint32_t
    } else if ControlState == WAIT_STATUS_IN as libc::c_int as libc::c_uint {
        if (*pInformation).USBbRequest as libc::c_int ==
               SET_ADDRESS as libc::c_int &&
               (*pInformation).USBbmRequestType as libc::c_int &
                   (0x60 as libc::c_int | 0x1f as libc::c_int) ==
                   0 as libc::c_int | DEVICE_RECIPIENT as libc::c_int {
            SetDeviceAddress((*pInformation).USBwValues.bw.bb0);
            (*pUser_Standard_Requests).User_SetDeviceAddress.expect("non-null function pointer")();
        }
        Some((*pProperty).Process_Status_IN.expect("non-null function pointer")).expect("non-null function pointer")();
        ControlState = STALLED as libc::c_int as uint32_t
    } else { ControlState = STALLED as libc::c_int as uint32_t }
    (*pInformation).ControlState = ControlState as uint8_t;
    return Post0_Process();
}
/* ******************************************************************************
* Function Name  : Out0_Process
* Description    : Process the OUT token on all default endpoint.
* Input          : None.
* Output         : None.
* Return         : Post0_Process.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Out0_Process() -> uint8_t {
    let mut ControlState: uint32_t = (*pInformation).ControlState as uint32_t;
    if ControlState == IN_DATA as libc::c_int as libc::c_uint ||
           ControlState == LAST_IN_DATA as libc::c_int as libc::c_uint {
        /* host aborts the transfer before finish */
        ControlState = STALLED as libc::c_int as uint32_t
    } else if ControlState == OUT_DATA as libc::c_int as libc::c_uint ||
                  ControlState == LAST_OUT_DATA as libc::c_int as libc::c_uint
     {
        DataStageOut();
        ControlState = (*pInformation).ControlState as uint32_t
        /* may be changed outside the function */
    } else if ControlState == WAIT_STATUS_OUT as libc::c_int as libc::c_uint {
        Some((*pProperty).Process_Status_OUT.expect("non-null function pointer")).expect("non-null function pointer")();
        ControlState = STALLED as libc::c_int as uint32_t
    } else {
        /* Unexpect state, STALL the endpoint */
        ControlState = STALLED as libc::c_int as uint32_t
    }
    (*pInformation).ControlState = ControlState as uint8_t;
    return Post0_Process();
}
/* ******************************************************************************
* Function Name  : Post0_Process
* Description    : Stall the Endpoint 0 in case of error.
* Input          : None.
* Output         : None.
* Return         : - 0 if the control State is in PAUSE
*                  - 1 if not.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Post0_Process() -> uint8_t {
    SetEPRxCount(0 as libc::c_int as uint8_t,
                 Device_Property.MaxPacketSize as uint16_t);
    if (*pInformation).ControlState as libc::c_int == STALLED as libc::c_int {
        ::core::ptr::write_volatile(&mut SaveRState as *mut uint16_t,
                                    0x1000 as libc::c_int as uint16_t);
        ::core::ptr::write_volatile(&mut SaveTState as *mut uint16_t,
                                    0x10 as libc::c_int as uint16_t)
    }
    return ((*pInformation).ControlState as libc::c_int ==
                PAUSE as libc::c_int) as libc::c_int as uint8_t;
}
/* ******************************************************************************
* Function Name  : SetDeviceAddress.
* Description    : Set the device and all the used Endpoints addresses.
* Input          : - Val: device address.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetDeviceAddress(mut Val: uint8_t) {
    let mut i: uint32_t = 0;
    let mut nEP: uint32_t = Device_Table.Total_Endpoint as uint32_t;
    /* set address in every used endpoint */
    i = 0 as libc::c_int as uint32_t; /* for */
    while i < nEP {
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                         *mut libc::c_uint).offset(i as
                                                                       uint8_t
                                                                       as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                    (0x8000 as libc::c_int as uint16_t as
                                         libc::c_int | 0x80 as libc::c_int |
                                         *(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(i as
                                                                             uint8_t
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int) |
                                         i as uint8_t as libc::c_int) as
                                        libc::c_uint);
        i = i.wrapping_add(1)
    }
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x4c as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                (Val as uint16_t as libc::c_int |
                                     0x80 as libc::c_int) as libc::c_uint);
    /* set device address and enable function */
}
/* *
  ******************************************************************************
  * @file    usb_core.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Standard protocol processing functions prototypes
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
/* 0 */
/* 1 */
/* 2 */
/* 3 */
/* 4 */
/* 5 */
/* 7 */
/* 8 */
/* 9 */
/* 10 */
/* The state machine states of a control pipe */
/* All the request process routines return a value of this type
   If the return value is not SUCCESS or NOT_READY,
   the software will STALL the correspond endpoint */
/* Process successfully */
/* The process has not been finished, endpoint will be
                         NAK to further request */
/*-*-*-*-*-*-*-*-*-*-* Definitions for endpoint level -*-*-*-*-*-*-*-*-*-*-*-*/
/* When send data out of the device,
   CopyData() is used to get data buffer 'Length' bytes data
   if Length is 0,
    CopyData() returns the total length of the data
    if the request is not supported, returns 0
    (NEW Feature )
     if CopyData() returns -1, the calling routine should not proceed
     further and will resume the SETUP process by the class device
   if Length is not 0,
    CopyData() returns a pointer to indicate the data location
   Usb_wLength is the data remain to be sent,
   Usb_wOffset is the Offset of original data
  When receive data from the host,
   CopyData() is used to get user data buffer which is capable
   of Length bytes data to copy data from the endpoint buffer.
   if Length is 0,
    CopyData() returns the available data length,
   if Length is not 0,
    CopyData() returns user buffer address
   Usb_rLength is the data remain to be received,
   Usb_rPointer is the Offset of data buffer
  */
/*-*-*-*-*-*-*-*-*-*-*-* Definitions for device level -*-*-*-*-*-*-*-*-*-*-*-*/
/* Number of endpoints that are used */
/* Number of configuration available */
/* bmRequestType */
/* bRequest */
/* wValue */
/* wIndex */
/* wLength */
/* of type CONTROL_STATE */
/* Selected configuration */
/* Selected interface of current configuration */
/* Selected Alternate Setting of current
                                     interface*/
/* Initialize the device */
/* Reset routine of this device */
/* Device dependent process after the status stage */
/* Procedure of process on setup stage of a class specified request with data stage */
  /* All class specified requests with data stage are processed in Class_Data_Setup
   Class_Data_Setup()
    responses to check all special requests and fills ENDPOINT_INFO
    according to the request
    If IN tokens are expected, then wLength & wOffset will be filled
    with the total transferring bytes and the starting position
    If OUT tokens are expected, then rLength & rOffset will be filled
    with the total expected bytes and the starting position in the buffer

    If the request is valid, Class_Data_Setup returns SUCCESS, else UNSUPPORT

   CAUTION:
    Since GET_CONFIGURATION & GET_INTERFACE are highly related to
    the individual classes, they will be checked and processed here.
  */
/* Procedure of process on setup stage of a class specified request without data stage */
  /* All class specified requests without data stage are processed in Class_NoData_Setup
   Class_NoData_Setup
    responses to check all special requests and perform the request

   CAUTION:
    Since SET_CONFIGURATION & SET_INTERFACE are highly related to
    the individual classes, they will be checked and processed here.
  */
/*Class_Get_Interface_Setting
   This function is used by the file usb_core.c to test if the selected Interface
   and Alternate Setting (uint8_t Interface, uint8_t AlternateSetting) are supported by
   the application.
   This function is writing by user. It should return "SUCCESS" if the Interface
   and Alternate Setting are supported by the application or "UNSUPPORT" if they
   are not supported. */
/* This field is not used in current library version. It is kept only for 
   compatibility with previous versions */
/* Get Configuration */
/* Set Configuration */
/* Get Interface */
/* Set Interface */
/* Get Status */
/* Clear Feature */
/* Set Endpoint Feature */
/* Set Device Feature */
/* Set Device Address */
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* ******************************************************************************
* Function Name  : NOP_Process
* Description    : No operation function.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn NOP_Process() { }
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
