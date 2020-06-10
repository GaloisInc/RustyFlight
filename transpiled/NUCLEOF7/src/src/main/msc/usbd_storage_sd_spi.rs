use ::libc;
extern "C" {
    #[no_mangle]
    fn sdcard_init(config: *const sdcardConfig_t);
    #[no_mangle]
    fn sdcard_readBlock(blockIndex: uint32_t, buffer: *mut uint8_t,
                        callback: sdcard_operationCompleteCallback_c,
                        callbackData: uint32_t) -> bool;
    #[no_mangle]
    fn sdcard_writeBlock(blockIndex: uint32_t, buffer: *mut uint8_t,
                         callback: sdcard_operationCompleteCallback_c,
                         callbackData: uint32_t) -> sdcardOperationStatus_e;
    #[no_mangle]
    fn sdcard_getMetadata() -> *const sdcardMetadata_t;
    #[no_mangle]
    fn sdcard_poll() -> bool;
    #[no_mangle]
    static mut sdcardConfig_System: sdcardConfig_t;
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
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
  * @file    usbd_msc.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for the usbd_msc.c file
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
/* * @addtogroup USBD_MSC_BOT
  * @{
  */
/* * @defgroup USBD_MSC
  * @brief This file is the Header file for usbd_msc.c
  * @{
  */
/* * @defgroup USBD_BOT_Exported_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USB_CORE_Exported_Types
  * @{
  */
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
pub type USBD_StorageTypeDef = _USBD_STORAGE;
pub const SDCARD_OPERATION_IN_PROGRESS: sdcardOperationStatus_e = 0;
pub type sdcardOperationStatus_e = libc::c_uint;
pub const SDCARD_OPERATION_FAILURE: sdcardOperationStatus_e = 3;
pub const SDCARD_OPERATION_SUCCESS: sdcardOperationStatus_e = 2;
pub const SDCARD_OPERATION_BUSY: sdcardOperationStatus_e = 1;
pub type sdcard_operationCompleteCallback_c
    =
    Option<unsafe extern "C" fn(_: sdcardBlockOperation_e, _: uint32_t,
                                _: *mut uint8_t, _: uint32_t) -> ()>;
pub type sdcardBlockOperation_e = libc::c_uint;
pub const SDCARD_BLOCK_OPERATION_ERASE: sdcardBlockOperation_e = 2;
pub const SDCARD_BLOCK_OPERATION_WRITE: sdcardBlockOperation_e = 1;
pub const SDCARD_BLOCK_OPERATION_READ: sdcardBlockOperation_e = 0;
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
pub type sdcardMetadata_t = sdcardMetadata_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardMetadata_s {
    pub numBlocks: uint32_t,
    pub oemID: uint16_t,
    pub manufacturerID: uint8_t,
    pub productName: [libc::c_char; 5],
    pub productSerial: uint32_t,
    pub productRevisionMajor: uint8_t,
    pub productRevisionMinor: uint8_t,
    pub productionYear: uint16_t,
    pub productionMonth: uint8_t,
}
pub type sdcardConfig_t = sdcardConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardConfig_s {
    pub useDma: uint8_t,
    pub enabled: uint8_t,
    pub device: uint8_t,
    pub cardDetectTag: ioTag_t,
    pub chipSelectTag: ioTag_t,
    pub cardDetectInverted: uint8_t,
    pub dmaIdentifier: uint8_t,
    pub dmaChannel: uint8_t,
}
/* Card capacity in 512-byte blocks*/
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
#[inline]
unsafe extern "C" fn sdcardConfig() -> *const sdcardConfig_t {
    return &mut sdcardConfig_System;
}
/* USB Mass storage Standard Inquiry Data */
static mut STORAGE_Inquirydata: [uint8_t; 36] =
    [0 as libc::c_int as uint8_t, 0x80 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     (0x24 as libc::c_int - 5 as libc::c_int) as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 'S' as i32 as uint8_t,
     'T' as i32 as uint8_t, 'M' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, 'P' as i32 as uint8_t, 'r' as i32 as uint8_t,
     'o' as i32 as uint8_t, 'd' as i32 as uint8_t, 'u' as i32 as uint8_t,
     't' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, '0' as i32 as uint8_t,
     '.' as i32 as uint8_t, '0' as i32 as uint8_t, '1' as i32 as uint8_t];
#[no_mangle]
pub static mut USBD_MSC_MICRO_SDIO_fops: USBD_StorageTypeDef =
    unsafe {
        {
            let mut init =
                _USBD_STORAGE{Init:
                                  Some(STORAGE_Init as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              GetCapacity:
                                  Some(STORAGE_GetCapacity as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint32_t,
                                                                _:
                                                                    *mut uint16_t)
                                               -> int8_t),
                              IsReady:
                                  Some(STORAGE_IsReady as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              IsWriteProtected:
                                  Some(STORAGE_IsWriteProtected as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              Read:
                                  Some(STORAGE_Read as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _: uint32_t,
                                                                _: uint16_t)
                                               -> int8_t),
                              Write:
                                  Some(STORAGE_Write as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _: uint32_t,
                                                                _: uint16_t)
                                               -> int8_t),
                              GetMaxLun:
                                  Some(STORAGE_GetMaxLun as
                                           unsafe extern "C" fn() -> int8_t),
                              pInquiry:
                                  STORAGE_Inquirydata.as_ptr() as *mut _ as
                                      *mut int8_t,};
            init
        }
    };
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_Init(mut lun: uint8_t) -> int8_t {
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    sdcard_init(sdcardConfig());
    while sdcard_poll() as libc::c_int == 0 as libc::c_int { }
    ledSet(0 as libc::c_int, 1 as libc::c_int != 0);
    return 0 as libc::c_int as int8_t;
}
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_GetCapacity(mut lun: uint8_t,
                                         mut block_num: *mut uint32_t,
                                         mut block_size: *mut uint16_t)
 -> int8_t {
    *block_num = (*sdcard_getMetadata()).numBlocks;
    *block_size = 512 as libc::c_int as uint16_t;
    return 0 as libc::c_int as int8_t;
}
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_IsReady(mut lun: uint8_t) -> int8_t {
    let mut ret: int8_t = -(1 as libc::c_int) as int8_t;
    if sdcard_poll() { ret = 0 as libc::c_int as int8_t }
    return ret;
}
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_IsWriteProtected(mut lun: uint8_t) -> int8_t {
    return 0 as libc::c_int as int8_t;
}
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_Read(mut lun: uint8_t, mut buf: *mut uint8_t,
                                  mut blk_addr: uint32_t,
                                  mut blk_len: uint16_t) -> int8_t {
    ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < blk_len as libc::c_int {
        while sdcard_readBlock(blk_addr.wrapping_add(i as libc::c_uint),
                               buf.offset((512 as libc::c_int * i) as isize),
                               None, 0 as *mut libc::c_void as uint32_t) as
                  libc::c_int == 0 as libc::c_int {
        }
        while sdcard_poll() as libc::c_int == 0 as libc::c_int { }
        i += 1
    }
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    return 0 as libc::c_int as int8_t;
}
/* ******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_Write(mut lun: uint8_t, mut buf: *mut uint8_t,
                                   mut blk_addr: uint32_t,
                                   mut blk_len: uint16_t) -> int8_t {
    ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < blk_len as libc::c_int {
        while sdcard_writeBlock(blk_addr.wrapping_add(i as libc::c_uint),
                                buf.offset((i * 512 as libc::c_int) as isize),
                                None, 0 as *mut libc::c_void as uint32_t) as
                  libc::c_uint !=
                  SDCARD_OPERATION_IN_PROGRESS as libc::c_int as libc::c_uint
              {
            sdcard_poll();
        }
        while sdcard_poll() as libc::c_int == 0 as libc::c_int { }
        i += 1
    }
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    return 0 as libc::c_int as int8_t;
}
/* ******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_GetMaxLun() -> int8_t {
    return (1 as libc::c_int - 1 as libc::c_int) as int8_t;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
