use ::libc;
extern "C" {
    #[no_mangle]
    fn USB_Interrupts_Disable();
    #[no_mangle]
    fn Get_SerialNum();
    #[no_mangle]
    fn SetBTABLE(_: uint16_t);
    #[no_mangle]
    fn SetEPType(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPTxStatus(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPRxStatus(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPRxValid(_: uint8_t);
    #[no_mangle]
    fn Clear_Status_Out(_: uint8_t);
    #[no_mangle]
    fn SetEPTxAddr(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPRxAddr(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPRxCount(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn Standard_GetDescriptorData(Length: uint16_t, pDesc: PONE_DESCRIPTOR)
     -> *mut uint8_t;
    #[no_mangle]
    fn SetDeviceAddress(_: uint8_t);
    #[no_mangle]
    fn NOP_Process();
    #[no_mangle]
    static mut pInformation: *mut DEVICE_INFO;
    #[no_mangle]
    static mut Device_Info: DEVICE_INFO;
    #[no_mangle]
    fn USB_SIL_Init() -> uint32_t;
    /* Exported functions ------------------------------------------------------- */
    #[no_mangle]
    static Virtual_Com_Port_DeviceDescriptor: [uint8_t; 18];
    #[no_mangle]
    static Virtual_Com_Port_ConfigDescriptor: [uint8_t; 67];
    #[no_mangle]
    static Virtual_Com_Port_StringLangID: [uint8_t; 4];
    #[no_mangle]
    static Virtual_Com_Port_StringVendor: [uint8_t; 38];
    #[no_mangle]
    static Virtual_Com_Port_StringProduct: [uint8_t; 50];
    #[no_mangle]
    static mut Virtual_Com_Port_StringSerial: [uint8_t; 26];
    #[no_mangle]
    fn PowerOn() -> RESULT;
    /* External variables --------------------------------------------------------*/
    #[no_mangle]
    static mut bDeviceState: uint32_t;
}
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
#[repr(C, packed)]
pub struct LINE_CODING {
    pub bitrate: uint32_t,
    pub format: uint8_t,
    pub paritytype: uint8_t,
    pub datatype: uint8_t,
}
pub const ATTACHED: _DEVICE_STATE = 1;
pub const UNCONNECTED: _DEVICE_STATE = 0;
pub const ADDRESSED: _DEVICE_STATE = 4;
pub const CONFIGURED: _DEVICE_STATE = 5;
pub type _DEVICE_STATE = libc::c_uint;
pub const SUSPENDED: _DEVICE_STATE = 3;
pub const POWERED: _DEVICE_STATE = 2;
/* *
 ******************************************************************************
 * @file    usb_prop.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   All processing related to Virtual Com Port Demo
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
#[no_mangle]
pub static mut Request: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut linecoding: LINE_CODING =
    {
        let mut init =
            LINE_CODING{bitrate: 115200 as libc::c_int as uint32_t,
                        format: 0 as libc::c_int as uint8_t,
                        paritytype: 0 as libc::c_int as uint8_t,
                        datatype: 0x8 as libc::c_int as uint8_t,};
        init
    };
/* -------------------------------------------------------------------------- */
/*  Structures initializations */
/* -------------------------------------------------------------------------- */
#[no_mangle]
pub static mut Device_Table: DEVICE =
    {
        let mut init =
            _DEVICE{Total_Endpoint: 4 as libc::c_int as uint8_t,
                    Total_Configuration: 1 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub static mut Device_Property: DEVICE_PROP =
    unsafe {
        {
            let mut init =
                _DEVICE_PROP{Init:
                                 Some(Virtual_Com_Port_init as
                                          unsafe extern "C" fn() -> ()),
                             Reset:
                                 Some(Virtual_Com_Port_Reset as
                                          unsafe extern "C" fn() -> ()),
                             Process_Status_IN:
                                 Some(Virtual_Com_Port_Status_In as
                                          unsafe extern "C" fn() -> ()),
                             Process_Status_OUT:
                                 Some(Virtual_Com_Port_Status_Out as
                                          unsafe extern "C" fn() -> ()),
                             Class_Data_Setup:
                                 Some(Virtual_Com_Port_Data_Setup as
                                          unsafe extern "C" fn(_: uint8_t)
                                              -> RESULT),
                             Class_NoData_Setup:
                                 Some(Virtual_Com_Port_NoData_Setup as
                                          unsafe extern "C" fn(_: uint8_t)
                                              -> RESULT),
                             Class_Get_Interface_Setting:
                                 Some(Virtual_Com_Port_Get_Interface_Setting
                                          as
                                          unsafe extern "C" fn(_: uint8_t,
                                                               _: uint8_t)
                                              -> RESULT),
                             GetDeviceDescriptor:
                                 Some(Virtual_Com_Port_GetDeviceDescriptor as
                                          unsafe extern "C" fn(_: uint16_t)
                                              -> *mut uint8_t),
                             GetConfigDescriptor:
                                 Some(Virtual_Com_Port_GetConfigDescriptor as
                                          unsafe extern "C" fn(_: uint16_t)
                                              -> *mut uint8_t),
                             GetStringDescriptor:
                                 Some(Virtual_Com_Port_GetStringDescriptor as
                                          unsafe extern "C" fn(_: uint16_t)
                                              -> *mut uint8_t),
                             RxEP_buffer:
                                 0 as *const libc::c_void as
                                     *mut libc::c_void,
                             MaxPacketSize: 0x40 as libc::c_int as uint8_t,};
            init
        }
    };
#[no_mangle]
pub static mut User_Standard_Requests: USER_STANDARD_REQUESTS =
    unsafe {
        {
            let mut init =
                _USER_STANDARD_REQUESTS{User_GetConfiguration:
                                            Some(NOP_Process as
                                                     unsafe extern "C" fn()
                                                         -> ()),
                                        User_SetConfiguration:
                                            Some(Virtual_Com_Port_SetConfiguration
                                                     as
                                                     unsafe extern "C" fn()
                                                         -> ()),
                                        User_GetInterface:
                                            Some(NOP_Process as
                                                     unsafe extern "C" fn()
                                                         -> ()),
                                        User_SetInterface:
                                            Some(NOP_Process as
                                                     unsafe extern "C" fn()
                                                         -> ()),
                                        User_GetStatus:
                                            Some(NOP_Process as
                                                     unsafe extern "C" fn()
                                                         -> ()),
                                        User_ClearFeature:
                                            Some(NOP_Process as
                                                     unsafe extern "C" fn()
                                                         -> ()),
                                        User_SetEndPointFeature:
                                            Some(NOP_Process as
                                                     unsafe extern "C" fn()
                                                         -> ()),
                                        User_SetDeviceFeature:
                                            Some(NOP_Process as
                                                     unsafe extern "C" fn()
                                                         -> ()),
                                        User_SetDeviceAddress:
                                            Some(Virtual_Com_Port_SetDeviceAddress
                                                     as
                                                     unsafe extern "C" fn()
                                                         -> ()),};
            init
        }
    };
#[no_mangle]
pub static mut Device_Descriptor: ONE_DESCRIPTOR =
    unsafe {
        {
            let mut init =
                OneDescriptor{Descriptor:
                                  Virtual_Com_Port_DeviceDescriptor.as_ptr()
                                      as *mut uint8_t,
                              Descriptor_Size:
                                  18 as libc::c_int as uint16_t,};
            init
        }
    };
#[no_mangle]
pub static mut Config_Descriptor: ONE_DESCRIPTOR =
    unsafe {
        {
            let mut init =
                OneDescriptor{Descriptor:
                                  Virtual_Com_Port_ConfigDescriptor.as_ptr()
                                      as *mut uint8_t,
                              Descriptor_Size:
                                  67 as libc::c_int as uint16_t,};
            init
        }
    };
#[no_mangle]
pub static mut String_Descriptor: [ONE_DESCRIPTOR; 4] =
    unsafe {
        [{
             let mut init =
                 OneDescriptor{Descriptor:
                                   Virtual_Com_Port_StringLangID.as_ptr() as
                                       *mut uint8_t,
                               Descriptor_Size:
                                   4 as libc::c_int as uint16_t,};
             init
         },
         {
             let mut init =
                 OneDescriptor{Descriptor:
                                   Virtual_Com_Port_StringVendor.as_ptr() as
                                       *mut uint8_t,
                               Descriptor_Size:
                                   38 as libc::c_int as uint16_t,};
             init
         },
         {
             let mut init =
                 OneDescriptor{Descriptor:
                                   Virtual_Com_Port_StringProduct.as_ptr() as
                                       *mut uint8_t,
                               Descriptor_Size:
                                   50 as libc::c_int as uint16_t,};
             init
         },
         {
             let mut init =
                 OneDescriptor{Descriptor:
                                   Virtual_Com_Port_StringSerial.as_ptr() as
                                       *mut _,
                               Descriptor_Size:
                                   26 as libc::c_int as uint16_t,};
             init
         }]
    };
/* Exported functions ------------------------------------------------------- */
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_init.
 * Description    : Virtual COM Port Mouse init routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_init() {
    /* Make absolutly sure interrupts are disabled. */
    USB_Interrupts_Disable();
    /* Update the serial number string descriptor with the data from the unique
     ID*/
    Get_SerialNum();
    (*pInformation).Current_Configuration = 0 as libc::c_int as uint8_t;
    /* Connect the device */
    PowerOn();
    /* Perform basic device initialization operations */
    USB_SIL_Init();
    ::core::ptr::write_volatile(&mut bDeviceState as *mut uint32_t,
                                UNCONNECTED as libc::c_int as uint32_t);
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_Reset
 * Description    : Virtual_Com_Port Mouse reset routine
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_Reset() {
    /* Set Virtual_Com_Port DEVICE as not configured */
    (*pInformation).Current_Configuration = 0 as libc::c_int as uint8_t;
    /* Current Feature initialization */
    (*pInformation).Current_Feature =
        Virtual_Com_Port_ConfigDescriptor[7 as libc::c_int as usize];
    /* Set Virtual_Com_Port DEVICE with the default Interface*/
    (*pInformation).Current_Interface = 0 as libc::c_int as uint8_t;
    SetBTABLE(0 as libc::c_int as uint16_t);
    /* Initialize Endpoint 0 */
    SetEPType(0 as libc::c_int as uint8_t, 0x200 as libc::c_int as uint16_t);
    SetEPTxStatus(0 as libc::c_int as uint8_t,
                  0x10 as libc::c_int as uint16_t);
    SetEPRxAddr(0 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint16_t);
    SetEPTxAddr(0 as libc::c_int as uint8_t, 0x80 as libc::c_int as uint16_t);
    Clear_Status_Out(0 as libc::c_int as uint8_t);
    SetEPRxCount(0 as libc::c_int as uint8_t,
                 Device_Property.MaxPacketSize as uint16_t);
    SetEPRxValid(0 as libc::c_int as uint8_t);
    /* Initialize Endpoint 1 */
    SetEPType(1 as libc::c_int as uint8_t, 0 as libc::c_int as uint16_t);
    SetEPTxAddr(1 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint16_t);
    SetEPTxStatus(1 as libc::c_int as uint8_t,
                  0x20 as libc::c_int as uint16_t);
    SetEPRxStatus(1 as libc::c_int as uint8_t, 0 as libc::c_int as uint16_t);
    /* Initialize Endpoint 2 */
    SetEPType(2 as libc::c_int as uint8_t, 0x600 as libc::c_int as uint16_t);
    SetEPTxAddr(2 as libc::c_int as uint8_t,
                0x100 as libc::c_int as uint16_t);
    SetEPRxStatus(2 as libc::c_int as uint8_t, 0 as libc::c_int as uint16_t);
    SetEPTxStatus(2 as libc::c_int as uint8_t,
                  0x20 as libc::c_int as uint16_t);
    /* Initialize Endpoint 3 */
    SetEPType(3 as libc::c_int as uint8_t, 0 as libc::c_int as uint16_t);
    SetEPRxAddr(3 as libc::c_int as uint8_t,
                0x110 as libc::c_int as uint16_t);
    SetEPRxCount(3 as libc::c_int as uint8_t, 64 as libc::c_int as uint16_t);
    SetEPRxStatus(3 as libc::c_int as uint8_t,
                  0x3000 as libc::c_int as uint16_t);
    SetEPTxStatus(3 as libc::c_int as uint8_t, 0 as libc::c_int as uint16_t);
    /* Set this device to response on default address */
    SetDeviceAddress(0 as libc::c_int as uint8_t);
    ::core::ptr::write_volatile(&mut bDeviceState as *mut uint32_t,
                                ATTACHED as libc::c_int as uint32_t);
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_SetConfiguration.
 * Description    : Update the device state to configured.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_SetConfiguration() {
    let mut pInfo: *mut DEVICE_INFO = &mut Device_Info;
    if (*pInfo).Current_Configuration as libc::c_int != 0 as libc::c_int {
        /* Device configured */
        ::core::ptr::write_volatile(&mut bDeviceState as *mut uint32_t,
                                    CONFIGURED as libc::c_int as uint32_t)
    };
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_SetConfiguration.
 * Description    : Update the device state to addressed.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_SetDeviceAddress() {
    ::core::ptr::write_volatile(&mut bDeviceState as *mut uint32_t,
                                ADDRESSED as libc::c_int as uint32_t);
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_Status_In.
 * Description    : Virtual COM Port Status In Routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_Status_In() {
    if Request as libc::c_int == 0x20 as libc::c_int {
        Request = 0 as libc::c_int as uint8_t
    };
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_Status_Out
 * Description    : Virtual COM Port Status OUT Routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_Status_Out() { }
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_Data_Setup
 * Description    : handle the data class specific requests
 * Input          : Request Nb.
 * Output         : None.
 * Return         : USB_UNSUPPORT or USB_SUCCESS.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_Data_Setup(mut RequestNo: uint8_t)
 -> RESULT {
    let mut CopyRoutine:
            Option<unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t> = None;
    CopyRoutine = None;
    if RequestNo as libc::c_int == 0x21 as libc::c_int {
        if (*pInformation).USBbmRequestType as libc::c_int &
               (0x60 as libc::c_int | 0x1f as libc::c_int) ==
               0x20 as libc::c_int | INTERFACE_RECIPIENT as libc::c_int {
            CopyRoutine =
                Some(Virtual_Com_Port_GetLineCoding as
                         unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t)
        }
    } else if RequestNo as libc::c_int == 0x20 as libc::c_int {
        if (*pInformation).USBbmRequestType as libc::c_int &
               (0x60 as libc::c_int | 0x1f as libc::c_int) ==
               0x20 as libc::c_int | INTERFACE_RECIPIENT as libc::c_int {
            CopyRoutine =
                Some(Virtual_Com_Port_SetLineCoding as
                         unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t)
        }
        Request = 0x20 as libc::c_int as uint8_t
    }
    if CopyRoutine.is_none() { return USB_UNSUPPORT }
    (*pInformation).Ctrl_Info.CopyData = CopyRoutine;
    (*pInformation).Ctrl_Info.Usb_wOffset = 0 as libc::c_int as uint16_t;
    Some(CopyRoutine.expect("non-null function pointer")).expect("non-null function pointer")(0
                                                                                                  as
                                                                                                  libc::c_int
                                                                                                  as
                                                                                                  uint16_t);
    return USB_SUCCESS;
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_NoData_Setup.
 * Description    : handle the no data class specific requests.
 * Input          : Request Nb.
 * Output         : None.
 * Return         : USB_UNSUPPORT or USB_SUCCESS.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_NoData_Setup(mut RequestNo: uint8_t)
 -> RESULT {
    if (*pInformation).USBbmRequestType as libc::c_int &
           (0x60 as libc::c_int | 0x1f as libc::c_int) ==
           0x20 as libc::c_int | INTERFACE_RECIPIENT as libc::c_int {
        if RequestNo as libc::c_int == 0x2 as libc::c_int {
            return USB_SUCCESS
        } else {
            if RequestNo as libc::c_int == 0x22 as libc::c_int {
                return USB_SUCCESS
            }
        }
    }
    return USB_UNSUPPORT;
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_GetDeviceDescriptor.
 * Description    : Gets the device descriptor.
 * Input          : Length.
 * Output         : None.
 * Return         : The address of the device descriptor.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_GetDeviceDescriptor(mut Length:
                                                                  uint16_t)
 -> *mut uint8_t {
    return Standard_GetDescriptorData(Length, &mut Device_Descriptor);
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_GetConfigDescriptor.
 * Description    : get the configuration descriptor.
 * Input          : Length.
 * Output         : None.
 * Return         : The address of the configuration descriptor.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_GetConfigDescriptor(mut Length:
                                                                  uint16_t)
 -> *mut uint8_t {
    return Standard_GetDescriptorData(Length, &mut Config_Descriptor);
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_GetStringDescriptor
 * Description    : Gets the string descriptors according to the needed index
 * Input          : Length.
 * Output         : None.
 * Return         : The address of the string descriptors.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_GetStringDescriptor(mut Length:
                                                                  uint16_t)
 -> *mut uint8_t {
    let mut wValue0: uint8_t = (*pInformation).USBwValues.bw.bb0;
    if wValue0 as libc::c_int > 4 as libc::c_int {
        return 0 as *mut uint8_t
    } else {
        return Standard_GetDescriptorData(Length,
                                          &mut *String_Descriptor.as_mut_ptr().offset(wValue0
                                                                                          as
                                                                                          isize))
    };
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_Get_Interface_Setting.
 * Description    : test the interface and the alternate setting according to the
 *                  supported one.
 * Input1         : uint8_t: Interface : interface number.
 * Input2         : uint8_t: AlternateSetting : Alternate Setting number.
 * Output         : None.
 * Return         : The address of the string descriptors.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_Get_Interface_Setting(mut Interface:
                                                                    uint8_t,
                                                                mut AlternateSetting:
                                                                    uint8_t)
 -> RESULT {
    if AlternateSetting as libc::c_int > 0 as libc::c_int {
        return USB_UNSUPPORT
    } else {
        if Interface as libc::c_int > 1 as libc::c_int {
            return USB_UNSUPPORT
        }
    }
    return USB_SUCCESS;
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_GetLineCoding.
 * Description    : send the linecoding structure to the PC host.
 * Input          : Length.
 * Output         : None.
 * Return         : Linecoding structure base address.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_GetLineCoding(mut Length: uint16_t)
 -> *mut uint8_t {
    if Length as libc::c_int == 0 as libc::c_int {
        (*pInformation).Ctrl_Info.Usb_wLength =
            ::core::mem::size_of::<LINE_CODING>() as libc::c_ulong as
                uint16_t;
        return 0 as *mut uint8_t
    }
    return &mut linecoding as *mut LINE_CODING as *mut uint8_t;
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_SetLineCoding.
 * Description    : Set the linecoding structure fields.
 * Input          : Length.
 * Output         : None.
 * Return         : Linecoding structure base address.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_SetLineCoding(mut Length: uint16_t)
 -> *mut uint8_t {
    if Length as libc::c_int == 0 as libc::c_int {
        (*pInformation).Ctrl_Info.Usb_wLength =
            ::core::mem::size_of::<LINE_CODING>() as libc::c_ulong as
                uint16_t;
        return 0 as *mut uint8_t
    }
    return &mut linecoding as *mut LINE_CODING as *mut uint8_t;
}
/* ******************************************************************************
 * Function Name  : Virtual_Com_Port_GetBaudRate.
 * Description    : Get the current baudrate
 * Input          : None.
 * Output         : None.
 * Return         : baudrate in bps
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Virtual_Com_Port_GetBaudRate() -> uint32_t {
    return linecoding.bitrate;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
