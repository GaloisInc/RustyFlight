use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type uint8_t = __uint8_t;
/* *
 ******************************************************************************
 * @file    usb_desc.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Descriptors for Virtual Com Port Demo
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
/* USB Standard Device Descriptor */
#[no_mangle]
pub static mut Virtual_Com_Port_DeviceDescriptor: [uint8_t; 18] =
    [0x12 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
     0x83 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
     0x40 as libc::c_int as uint8_t, 0x57 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut Virtual_Com_Port_ConfigDescriptor: [uint8_t; 67] =
    [0x9 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     67 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xc0 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x24 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x82 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 8 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 64 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x7 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x81 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     64 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t];
/* USB String Descriptors */
#[no_mangle]
pub static mut Virtual_Com_Port_StringLangID: [uint8_t; 4] =
    [4 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0x9 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut Virtual_Com_Port_StringVendor: [uint8_t; 38] =
    [38 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     'S' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'T' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'M' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'i' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'c' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'r' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'o' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'e' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'l' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'e' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'c' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     't' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'r' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'o' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'n' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'i' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'c' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     's' as i32 as uint8_t, 0 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut Virtual_Com_Port_StringProduct: [uint8_t; 50] =
    [50 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     'S' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'T' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'M' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     '3' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     '2' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     ' ' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'V' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'i' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'r' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     't' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'u' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'a' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'l' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     ' ' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'C' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'O' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'M' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     ' ' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'P' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'o' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'r' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     't' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     ' ' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     ' ' as i32 as uint8_t, 0 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut Virtual_Com_Port_StringSerial: [uint8_t; 26] =
    [26 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     'S' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'T' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     'M' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     '3' as i32 as uint8_t, 0 as libc::c_int as uint8_t,
     '2' as i32 as uint8_t, 0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0];
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
