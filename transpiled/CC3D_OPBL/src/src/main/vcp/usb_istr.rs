use ::libc;
extern "C" {
    #[no_mangle]
    fn NOP_Process();
    #[no_mangle]
    static mut Device_Property: DEVICE_PROP;
    #[no_mangle]
    static mut wInterrupt_Mask: uint16_t;
    /* *
  ******************************************************************************
  * @file    usb_int.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Endpoint CTR (Low and High) interrupt's service routines prototypes
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
    fn CTR_LP();
    #[no_mangle]
    fn Resume(eResumeSetVal: RESUME_STATE);
    /* function prototypes Automatically built defining related macros */
    #[no_mangle]
    fn EP1_IN_Callback();
    #[no_mangle]
    fn EP3_OUT_Callback();
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* All the request process routines return a value of this type
   If the return value is not SUCCESS or NOT_READY,
   the software will STALL the correspond endpoint */
pub type _RESULT = libc::c_uint;
pub const USB_NOT_READY: _RESULT = 3;
pub const USB_UNSUPPORT: _RESULT = 2;
/* The process has not been finished, endpoint will be
                         NAK to further request */
/* Process successfully */
pub const USB_ERROR: _RESULT = 1;
pub const USB_SUCCESS: _RESULT = 0;
pub type RESULT = _RESULT;
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
pub type _RESUME_STATE = libc::c_uint;
pub const RESUME_ESOF: _RESUME_STATE = 7;
pub const RESUME_OFF: _RESUME_STATE = 6;
pub const RESUME_ON: _RESUME_STATE = 5;
pub const RESUME_START: _RESUME_STATE = 4;
pub const RESUME_WAIT: _RESUME_STATE = 3;
pub const RESUME_LATER: _RESUME_STATE = 2;
pub const RESUME_INTERNAL: _RESUME_STATE = 1;
pub const RESUME_EXTERNAL: _RESUME_STATE = 0;
/* *
 ******************************************************************************
 * @file    usb_pwr.h
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Connection/disconnection & power management header
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
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
pub type RESUME_STATE = _RESUME_STATE;
/* *
 ******************************************************************************
 * @file    usb_istr.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   ISTR events interrupt service routines
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
pub static mut wIstr: uint16_t = 0;
/* ISTR register last read value */
#[no_mangle]
pub static mut bIntPackSOF: uint8_t = 0 as libc::c_int as uint8_t;
/* SOFs received between 2 consecutive packets */
#[no_mangle]
pub static mut esof_counter: uint32_t = 0 as libc::c_int as uint32_t;
/* expected SOF counter */
#[no_mangle]
pub static mut wCNTR: uint32_t = 0 as libc::c_int as uint32_t;
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* function pointers to non-control endpoints service routines */
#[no_mangle]
pub static mut pEpInt_IN: [Option<unsafe extern "C" fn() -> ()>; 7] =
    unsafe {
        [Some(EP1_IN_Callback as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ())]
    };
#[no_mangle]
pub static mut pEpInt_OUT: [Option<unsafe extern "C" fn() -> ()>; 7] =
    unsafe {
        [Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(EP3_OUT_Callback as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ()),
         Some(NOP_Process as unsafe extern "C" fn() -> ())]
    };
/* *
 ******************************************************************************
 * @file    usb_istr.h
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   This file includes the peripherals header files in the user application.
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
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* ******************************************************************************
 * Function Name  : USB_Istr
 * Description    : ISTR events interrupt service routine
 * Input          :
 * Output         :
 * Return         :
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_Istr() {
    ::core::ptr::write_volatile(&mut wIstr as *mut uint16_t,
                                *((0x40005c00 as libc::c_long +
                                       0x44 as libc::c_int as libc::c_long) as
                                      *mut libc::c_uint) as uint16_t);
    if wIstr as libc::c_int & 0x200 as libc::c_int &
           wInterrupt_Mask as libc::c_int != 0 {
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                         0x44 as libc::c_int as libc::c_long)
                                        as *mut libc::c_uint,
                                    !(0x200 as libc::c_int) as uint16_t as
                                        libc::c_uint);
        ::core::ptr::write_volatile(&mut bIntPackSOF as *mut uint8_t,
                                    ::core::ptr::read_volatile::<uint8_t>(&bIntPackSOF
                                                                              as
                                                                              *const uint8_t).wrapping_add(1))
    }
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
    if wIstr as libc::c_int & 0x8000 as libc::c_int &
           wInterrupt_Mask as libc::c_int != 0 {
        /* servicing of the endpoint correct transfer interrupt */
        /* clear of the CTR flag into the sub */
        CTR_LP();
    }
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
    if wIstr as libc::c_int & 0x400 as libc::c_int &
           wInterrupt_Mask as libc::c_int != 0 {
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                         0x44 as libc::c_int as libc::c_long)
                                        as *mut libc::c_uint,
                                    !(0x400 as libc::c_int) as uint16_t as
                                        libc::c_uint);
        Device_Property.Reset.expect("non-null function pointer")();
    }
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
    if wIstr as libc::c_int & 0x1000 as libc::c_int &
           wInterrupt_Mask as libc::c_int != 0 {
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                         0x44 as libc::c_int as libc::c_long)
                                        as *mut libc::c_uint,
                                    !(0x1000 as libc::c_int) as uint16_t as
                                        libc::c_uint);
        Resume(RESUME_EXTERNAL);
    };
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USB_Istr */
