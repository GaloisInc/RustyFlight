use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialTxBytesFree(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn serialBeginWrite(instance: *mut serialPort_t);
    #[no_mangle]
    fn serialEndWrite(instance: *mut serialPort_t);
    #[no_mangle]
    fn pwmGetMotors() -> *mut pwmOutputPort_t;
    #[no_mangle]
    fn pwmDisableMotors();
    #[no_mangle]
    fn pwmEnableMotors();
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    fn beeperSilence();
    #[no_mangle]
    fn BL_ConnectEx(pDeviceInfo: *mut uint8_32_u) -> uint8_t;
    #[no_mangle]
    fn BL_SendCMDKeepAlive() -> uint8_t;
    #[no_mangle]
    fn BL_PageErase(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn BL_ReadEEprom(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn BL_WriteEEprom(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn BL_WriteFlash(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn BL_ReadFlash(interface_mode: uint8_t, pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn BL_VerifyFlash(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn BL_SendCMDRunRestartBootloader(pDeviceInfo: *mut uint8_32_u);
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
    /*
 * Author: 4712
*/
    #[no_mangle]
    fn Stk_SignOn() -> uint8_t;
    #[no_mangle]
    fn Stk_ConnectEx(pDeviceInfo: *mut uint8_32_u) -> uint8_t;
    #[no_mangle]
    fn Stk_ReadEEprom(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn Stk_WriteEEprom(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn Stk_ReadFlash(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn Stk_WriteFlash(pMem: *mut ioMem_t) -> uint8_t;
    #[no_mangle]
    fn Stk_Chip_Erase() -> uint8_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief TIM
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint16_t,
    pub RESERVED9: uint16_t,
    pub ARR: uint32_t,
    pub RCR: uint16_t,
    pub RESERVED10: uint16_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED13: uint16_t,
    pub OR: uint16_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
    /* !< TIM capture/compare register 4,      Address offset: 0x5C */
}
/* *
  ******************************************************************************
  * @file    stm32f30x_gpio.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup Configuration_Mode_enumeration 
  * @{
  */
pub type C2RustUnnamed = libc::c_uint;
/* !< GPIO Analog In/Out Mode      */
/* !< GPIO Alternate function Mode */
pub const GPIO_Mode_AN: C2RustUnnamed = 3;
/* !< GPIO Output Mode */
pub const GPIO_Mode_AF: C2RustUnnamed = 2;
/* !< GPIO Input Mode */
pub const GPIO_Mode_OUT: C2RustUnnamed = 1;
pub const GPIO_Mode_IN: C2RustUnnamed = 0;
/* *
  * @}
  */
/* * @defgroup Output_type_enumeration
  * @{
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_OType_OD: C2RustUnnamed_0 = 1;
pub const GPIO_OType_PP: C2RustUnnamed_0 = 0;
/* *
  * @}
  */
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_1 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_1 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_1 = 0;
pub type IO_t = *mut libc::c_void;
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
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
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timerChannel_t {
    pub ccr: *mut timCCR_t,
    pub tim: *mut TIM_TypeDef,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pwmOutputPort_t {
    pub channel: timerChannel_t,
    pub pulseScale: libc::c_float,
    pub pulseOffset: libc::c_float,
    pub forceOverflow: bool,
    pub enabled: bool,
    pub io: IO_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct escHardware_t {
    pub io: IO_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct ioMem_s {
    pub D_NUM_BYTES: uint8_t,
    pub D_FLASH_ADDR_H: uint8_t,
    pub D_FLASH_ADDR_L: uint8_t,
    pub D_PTR_I: *mut uint8_t,
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
/*
 * Author: 4712
*/
pub type ioMem_t = ioMem_s;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union uint8_16_u {
    pub bytes: [uint8_t; 2],
    pub word: uint16_t,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union uint8_32_u {
    pub bytes: [uint8_t; 4],
    pub words: [uint16_t; 2],
    pub dword: uint32_t,
}
static mut escCount: uint8_t = 0;
#[no_mangle]
pub static mut escHardware: [escHardware_t; 8] =
    [escHardware_t{io: 0 as *const libc::c_void as *mut libc::c_void,}; 8];
#[no_mangle]
pub static mut selected_esc: uint8_t = 0;
#[no_mangle]
pub static mut DeviceInfo: uint8_32_u = uint8_32_u{bytes: [0; 4],};
#[inline]
unsafe extern "C" fn isMcuConnected() -> bool {
    return DeviceInfo.bytes[0] as libc::c_int > 0i32;
}
#[inline]
unsafe extern "C" fn setEscHi(mut selEsc: uint8_t) {
    IOHi(escHardware[selEsc as usize].io);
}
#[inline]
unsafe extern "C" fn setEscLo(mut selEsc: uint8_t) {
    IOLo(escHardware[selEsc as usize].io);
}
#[inline]
unsafe extern "C" fn setEscInput(mut selEsc: uint8_t) {
    IOConfigGPIO(escHardware[selEsc as usize].io,
                 (GPIO_Mode_IN as libc::c_int | 0i32 << 2i32 | 0i32 << 4i32 |
                      (GPIO_PuPd_UP as libc::c_int) << 5i32) as ioConfig_t);
}
//extern uint8_32_u DeviceInfo;
#[no_mangle]
pub unsafe extern "C" fn esc4wayInit() -> uint8_t {
    // StopPwmAllMotors();
    pwmDisableMotors();
    escCount = 0i32 as uint8_t;
    memset(&mut escHardware as *mut [escHardware_t; 8] as *mut libc::c_void,
           0i32,
           ::core::mem::size_of::<[escHardware_t; 8]>() as libc::c_ulong);
    let mut pwmMotors: *mut pwmOutputPort_t = pwmGetMotors();
    let mut i: uint8_t = 0i32 as uint8_t;
    while (i as libc::c_int) < 8i32 {
        if (*pwmMotors.offset(i as isize)).enabled {
            if !(*pwmMotors.offset(i as isize)).io.is_null() {
                escHardware[escCount as usize].io =
                    (*pwmMotors.offset(i as isize)).io;
                setEscInput(escCount);
                setEscHi(escCount);
                escCount = escCount.wrapping_add(1)
            }
        }
        ::core::ptr::write_volatile(&mut i as *mut uint8_t,
                                    ::core::ptr::read_volatile::<uint8_t>(&i
                                                                              as
                                                                              *const uint8_t).wrapping_add(1))
    }
    return escCount;
}
#[no_mangle]
pub unsafe extern "C" fn esc4wayRelease() {
    while escCount as libc::c_int > 0i32 {
        escCount = escCount.wrapping_sub(1);
        IOConfigGPIO(escHardware[escCount as usize].io,
                     (GPIO_Mode_AF as libc::c_int | 0i32 << 2i32 |
                          (GPIO_OType_PP as libc::c_int) << 4i32 |
                          (GPIO_PuPd_NOPULL as libc::c_int) << 5i32) as
                         ioConfig_t);
        setEscLo(escCount);
    }
    pwmEnableMotors();
}
/* Copyright (c) 2002, 2003, 2004  Marek Michalkiewicz
   Copyright (c) 2005, 2007 Joerg Wunsch
   Copyright (c) 2013 Dave Hylands
   Copyright (c) 2013 Frederic Nadeau
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */
#[no_mangle]
pub unsafe extern "C" fn _crc_xmodem_update(mut crc: uint16_t,
                                            mut data: uint8_t) -> uint16_t {
    let mut i: libc::c_int = 0;
    crc =
        (crc as libc::c_int ^ (data as uint16_t as libc::c_int) << 8i32) as
            uint16_t;
    i = 0i32;
    while i < 8i32 {
        if crc as libc::c_int & 0x8000i32 != 0 {
            crc = ((crc as libc::c_int) << 1i32 ^ 0x1021i32) as uint16_t
        } else { crc = ((crc as libc::c_int) << 1i32) as uint16_t }
        i += 1
    }
    return crc;
}
// * End copyright
static mut CurrentInterfaceMode: uint8_t = 0;
unsafe extern "C" fn Connect(mut pDeviceInfo: *mut uint8_32_u) -> uint8_t {
    let mut I: uint8_t = 0i32 as uint8_t;
    while (I as libc::c_int) < 3i32 {
        if CurrentInterfaceMode as libc::c_int != 4i32 &&
               Stk_ConnectEx(pDeviceInfo) as libc::c_int != 0 &&
               ((*pDeviceInfo).words[0] as libc::c_int == 0x9307i32 ||
                    (*pDeviceInfo).words[0] as libc::c_int == 0x930ai32 ||
                    (*pDeviceInfo).words[0] as libc::c_int == 0x930fi32 ||
                    (*pDeviceInfo).words[0] as libc::c_int == 0x940bi32) {
            CurrentInterfaceMode = 3i32 as uint8_t;
            return 1i32 as uint8_t
        } else {
            if BL_ConnectEx(pDeviceInfo) != 0 {
                if (*pDeviceInfo).words[0] as libc::c_int == 0xf310i32 ||
                       (*pDeviceInfo).words[0] as libc::c_int == 0xf330i32 ||
                       (*pDeviceInfo).words[0] as libc::c_int == 0xf410i32 ||
                       (*pDeviceInfo).words[0] as libc::c_int == 0xf390i32 ||
                       (*pDeviceInfo).words[0] as libc::c_int == 0xf850i32 ||
                       (*pDeviceInfo).words[0] as libc::c_int == 0xe8b1i32 ||
                       (*pDeviceInfo).words[0] as libc::c_int == 0xe8b2i32 {
                    CurrentInterfaceMode = 1i32 as uint8_t;
                    return 1i32 as uint8_t
                } else {
                    if (*pDeviceInfo).words[0] as libc::c_int == 0x9307i32 ||
                           (*pDeviceInfo).words[0] as libc::c_int == 0x930ai32
                           ||
                           (*pDeviceInfo).words[0] as libc::c_int == 0x930fi32
                           ||
                           (*pDeviceInfo).words[0] as libc::c_int == 0x940bi32
                       {
                        CurrentInterfaceMode = 2i32 as uint8_t;
                        return 1i32 as uint8_t
                    } else {
                        if (*pDeviceInfo).words[0] as libc::c_int == 0x1f06i32
                               ||
                               (*pDeviceInfo).words[0] as libc::c_int ==
                                   0x3306i32 ||
                               (*pDeviceInfo).words[0] as libc::c_int ==
                                   0x3406i32 ||
                               (*pDeviceInfo).words[0] as libc::c_int ==
                                   0x3506i32 {
                            CurrentInterfaceMode = 4i32 as uint8_t;
                            return 1i32 as uint8_t
                        }
                    }
                }
            }
        }
        I = I.wrapping_add(1)
    }
    return 0i32 as uint8_t;
}
static mut port: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
unsafe extern "C" fn ReadByte() -> uint8_t {
    // need timeout?
    while serialRxBytesWaiting(port) == 0 { }
    return serialRead(port);
}
static mut CRC_in: uint8_16_u = uint8_16_u{bytes: [0; 2],};
unsafe extern "C" fn ReadByteCrc() -> uint8_t {
    let mut b: uint8_t = ReadByte();
    CRC_in.word = _crc_xmodem_update(CRC_in.word, b);
    return b;
}
unsafe extern "C" fn WriteByte(mut b: uint8_t) { serialWrite(port, b); }
static mut CRCout: uint8_16_u = uint8_16_u{bytes: [0; 2],};
unsafe extern "C" fn WriteByteCrc(mut b: uint8_t) {
    WriteByte(b);
    CRCout.word = _crc_xmodem_update(CRCout.word, b);
}
#[no_mangle]
pub unsafe extern "C" fn esc4wayProcess(mut mspPort: *mut serialPort_t) {
    let mut ParamBuf: [uint8_t; 256] = [0; 256];
    let mut ESC: uint8_t = 0;
    let mut I_PARAM_LEN: uint8_t = 0;
    let mut CMD: uint8_t = 0;
    let mut ACK_OUT: uint8_t = 0;
    let mut CRC_check: uint8_16_u = uint8_16_u{bytes: [0; 2],};
    let mut Dummy: uint8_16_u = uint8_16_u{bytes: [0; 2],};
    let mut O_PARAM_LEN: uint8_t = 0;
    let mut O_PARAM: *mut uint8_t = 0 as *mut uint8_t;
    let mut InBuff: *mut uint8_t = 0 as *mut uint8_t;
    let mut ioMem: ioMem_t =
        ioMem_t{D_NUM_BYTES: 0,
                D_FLASH_ADDR_H: 0,
                D_FLASH_ADDR_L: 0,
                D_PTR_I: 0 as *mut uint8_t,};
    port = mspPort;
    // Start here  with UART Main loop
    // fix for buzzer often starts beeping continuously when the ESCs are read
    // switch beeper silent here
    beeperSilence();
    let mut isExitScheduled: bool = 0i32 != 0;
    loop  {
        loop 
             // restart looking for new sequence from host
             {
            CRC_in.word = 0i32 as uint16_t;
            ESC = ReadByteCrc();
            if !(ESC as libc::c_int != 0x2fi32) { break ; }
        }
        ledSet(0i32, 1i32 != 0);
        Dummy.word = 0i32 as uint16_t;
        O_PARAM = &mut *Dummy.bytes.as_mut_ptr().offset(0) as *mut uint8_t;
        O_PARAM_LEN = 1i32 as uint8_t;
        CMD = ReadByteCrc();
        ioMem.D_FLASH_ADDR_H = ReadByteCrc();
        ioMem.D_FLASH_ADDR_L = ReadByteCrc();
        I_PARAM_LEN = ReadByteCrc();
        InBuff = ParamBuf.as_mut_ptr();
        let mut i: uint8_t = I_PARAM_LEN;
        loop  {
            *InBuff = ReadByteCrc();
            InBuff = InBuff.offset(1);
            i = i.wrapping_sub(1);
            if !(i as libc::c_int != 0i32) { break ; }
        }
        CRC_check.bytes[1] = ReadByte();
        CRC_check.bytes[0] = ReadByte();
        if CRC_check.word as libc::c_int == CRC_in.word as libc::c_int {
            ACK_OUT = 0i32 as uint8_t
        } else { ACK_OUT = 0x3i32 as uint8_t }
        ledSet(0i32, 1i32 != 0);
        if ACK_OUT as libc::c_int == 0i32 {
            // wtf.D_FLASH_ADDR_H=Adress_H;
            // wtf.D_FLASH_ADDR_L=Adress_L;
            ioMem.D_PTR_I = ParamBuf.as_mut_ptr();
            match CMD as libc::c_int {
                48 => {
                    // ******* Interface related stuff *******
                    if isMcuConnected() {
                        match CurrentInterfaceMode as libc::c_int {
                            2 | 1 | 4 => {
                                if BL_SendCMDKeepAlive() == 0 {
                                    // SetStateDisconnected() included
                                    ACK_OUT = 0xfi32 as uint8_t
                                }
                            }
                            3 => {
                                if Stk_SignOn() == 0 {
                                    // SetStateDisconnected();
                                    ACK_OUT = 0xfi32 as uint8_t
                                }
                            }
                            _ => { ACK_OUT = 0xfi32 as uint8_t }
                        }
                        if ACK_OUT as libc::c_int != 0i32 {
                            DeviceInfo.words[0] = 0i32 as uint16_t
                        }
                    }
                }
                49 => {
                    // Only interface itself, no matter what Device
                    Dummy.bytes[0] = 107i32 as uint8_t
                }
                50 => {
                    // Only interface itself, no matter what Device
                    // O_PARAM_LEN=16;
                    O_PARAM_LEN =
                        strlen(b"m4wFCIntf\x00" as *const u8 as
                                   *const libc::c_char) as uint8_t;
                    O_PARAM =
                        b"m4wFCIntf\x00" as *const u8 as *const libc::c_char
                            as *mut uint8_t
                }
                51 => {
                    // Only interface itself, no matter what Device
                    // Dummy = iUart_res_InterfVersion;
                    O_PARAM_LEN = 2i32 as uint8_t;
                    Dummy.bytes[0] =
                        ((20i32 * 1000i32 +
                              0i32 as uint8_t as libc::c_int * 100i32 +
                              0o2i32 as uint8_t as libc::c_int) as uint16_t as
                             libc::c_int / 100i32) as uint8_t;
                    Dummy.bytes[1] =
                        ((20i32 * 1000i32 +
                              0i32 as uint8_t as libc::c_int * 100i32 +
                              0o2i32 as uint8_t as libc::c_int) as uint16_t as
                             libc::c_int % 100i32) as uint8_t
                }
                52 => { isExitScheduled = 1i32 != 0 }
                63 => {
                    if ParamBuf[0] as libc::c_int <= 4i32 &&
                           ParamBuf[0] as libc::c_int >= 1i32 {
                        CurrentInterfaceMode = ParamBuf[0]
                    } else { ACK_OUT = 0x9i32 as uint8_t }
                }
                53 => {
                    if (ParamBuf[0] as libc::c_int) < escCount as libc::c_int
                       {
                        // Channel may change here
                        selected_esc = ParamBuf[0];
                        match CurrentInterfaceMode as libc::c_int {
                            1 | 2 | 4 => {
                                BL_SendCMDRunRestartBootloader(&mut DeviceInfo);
                            }
                            3 | _ => { }
                        }
                        DeviceInfo.words[0] = 0i32 as uint16_t
                    } else { ACK_OUT = 0x8i32 as uint8_t }
                }
                55 => {
                    DeviceInfo.words[0] = 0i32 as uint16_t;
                    if (ParamBuf[0] as libc::c_int) < escCount as libc::c_int
                       {
                        //Channel may change here
                        //ESC_LO or ESC_HI; Halt state for prev channel
                        selected_esc = ParamBuf[0]; //4
                        O_PARAM_LEN = 4i32 as uint8_t;
                        O_PARAM =
                            &mut DeviceInfo as *mut uint8_32_u as
                                *mut uint8_t;
                        if Connect(&mut DeviceInfo) != 0 {
                            DeviceInfo.bytes[3] = CurrentInterfaceMode
                        } else {
                            DeviceInfo.words[0] = 0i32 as uint16_t;
                            ACK_OUT = 0xfi32 as uint8_t
                        }
                    } else { ACK_OUT = 0x8i32 as uint8_t }
                }
                56 => {
                    match CurrentInterfaceMode as libc::c_int {
                        3 => {
                            if Stk_Chip_Erase() == 0 {
                                ACK_OUT = 0xfi32 as uint8_t
                            }
                        }
                        _ => { ACK_OUT = 0x2i32 as uint8_t }
                    }
                }
                57 => {
                    match CurrentInterfaceMode as libc::c_int {
                        1 | 4 => {
                            Dummy.bytes[0] = ParamBuf[0];
                            if CurrentInterfaceMode as libc::c_int == 4i32 {
                                // Address =Page * 1024
                                ioMem.D_FLASH_ADDR_H =
                                    ((Dummy.bytes[0] as libc::c_int) << 2i32)
                                        as uint8_t
                            } else {
                                // Address =Page * 512
                                ioMem.D_FLASH_ADDR_H =
                                    ((Dummy.bytes[0] as libc::c_int) << 1i32)
                                        as uint8_t
                            }
                            ioMem.D_FLASH_ADDR_L = 0i32 as uint8_t;
                            if BL_PageErase(&mut ioMem) == 0 {
                                ACK_OUT = 0xfi32 as uint8_t
                            }
                        }
                        _ => { ACK_OUT = 0x2i32 as uint8_t }
                    }
                }
                58 => {
                    //*** Device Memory Read Ops ***
                    ioMem.D_NUM_BYTES = ParamBuf[0];
                    /*
                    wtf.D_FLASH_ADDR_H=Adress_H;
                    wtf.D_FLASH_ADDR_L=Adress_L;
                    wtf.D_PTR_I = BUF_I;
                    */
                    match CurrentInterfaceMode as libc::c_int {
                        1 | 2 | 4 => {
                            if BL_ReadFlash(CurrentInterfaceMode, &mut ioMem)
                                   == 0 {
                                ACK_OUT = 0xfi32 as uint8_t
                            }
                        }
                        3 => {
                            if Stk_ReadFlash(&mut ioMem) == 0 {
                                ACK_OUT = 0xfi32 as uint8_t
                            }
                        }
                        _ => { ACK_OUT = 0x2i32 as uint8_t }
                    }
                    if ACK_OUT as libc::c_int == 0i32 {
                        O_PARAM_LEN = ioMem.D_NUM_BYTES;
                        O_PARAM =
                            &mut ParamBuf as *mut [uint8_t; 256] as
                                *mut uint8_t
                    }
                }
                61 => {
                    ioMem.D_NUM_BYTES = ParamBuf[0];
                    /*
                    wtf.D_FLASH_ADDR_H = Adress_H;
                    wtf.D_FLASH_ADDR_L = Adress_L;
                    wtf.D_PTR_I = BUF_I;
                    */
                    match CurrentInterfaceMode as libc::c_int {
                        2 => {
                            if BL_ReadEEprom(&mut ioMem) == 0 {
                                ACK_OUT = 0xfi32 as uint8_t
                            }
                        }
                        3 => {
                            if Stk_ReadEEprom(&mut ioMem) == 0 {
                                ACK_OUT = 0xfi32 as uint8_t
                            }
                        }
                        _ => { ACK_OUT = 0x2i32 as uint8_t }
                    }
                    if ACK_OUT as libc::c_int == 0i32 {
                        O_PARAM_LEN = ioMem.D_NUM_BYTES;
                        O_PARAM =
                            &mut ParamBuf as *mut [uint8_t; 256] as
                                *mut uint8_t
                    }
                }
                59 => {
                    //*** Device Memory Write Ops ***
                    ioMem.D_NUM_BYTES = I_PARAM_LEN;
                    /*
                    wtf.D_FLASH_ADDR_H=Adress_H;
                    wtf.D_FLASH_ADDR_L=Adress_L;
                    wtf.D_PTR_I = BUF_I;
                    */
                    match CurrentInterfaceMode as libc::c_int {
                        1 | 2 | 4 => {
                            if BL_WriteFlash(&mut ioMem) == 0 {
                                ACK_OUT = 0xfi32 as uint8_t
                            }
                        }
                        3 => {
                            if Stk_WriteFlash(&mut ioMem) == 0 {
                                ACK_OUT = 0xfi32 as uint8_t
                            }
                        }
                        _ => { }
                    }
                }
                62 => {
                    ioMem.D_NUM_BYTES = I_PARAM_LEN;
                    ACK_OUT = 0xfi32 as uint8_t;
                    /*
                    wtf.D_FLASH_ADDR_H=Adress_H;
                    wtf.D_FLASH_ADDR_L=Adress_L;
                    wtf.D_PTR_I = BUF_I;
                    */
                    match CurrentInterfaceMode as libc::c_int {
                        1 => { ACK_OUT = 0x2i32 as uint8_t }
                        2 => {
                            if BL_WriteEEprom(&mut ioMem) != 0 {
                                ACK_OUT = 0i32 as uint8_t
                            }
                        }
                        3 => {
                            if Stk_WriteEEprom(&mut ioMem) != 0 {
                                ACK_OUT = 0i32 as uint8_t
                            }
                        }
                        _ => { }
                    }
                }
                64 => {
                    //*** Device Memory Verify Ops ***
                    match CurrentInterfaceMode as libc::c_int {
                        4 => {
                            ioMem.D_NUM_BYTES = I_PARAM_LEN;
                            /*
                            wtf.D_FLASH_ADDR_H=Adress_H;
                            wtf.D_FLASH_ADDR_L=Adress_L;
                            wtf.D_PTR_I = BUF_I;
                            */
                            ACK_OUT = BL_VerifyFlash(&mut ioMem);
                            match ACK_OUT as libc::c_int {
                                48 => { ACK_OUT = 0i32 as uint8_t }
                                192 => { ACK_OUT = 0x4i32 as uint8_t }
                                _ => { ACK_OUT = 0xfi32 as uint8_t }
                            }
                        }
                        _ => { ACK_OUT = 0x2i32 as uint8_t }
                    }
                }
                _ => { ACK_OUT = 0x2i32 as uint8_t }
            }
        }
        CRCout.word = 0i32 as uint16_t;
        ledSet(0i32, 0i32 != 0);
        serialBeginWrite(port);
        WriteByteCrc(0x2ei32 as uint8_t);
        WriteByteCrc(CMD);
        WriteByteCrc(ioMem.D_FLASH_ADDR_H);
        WriteByteCrc(ioMem.D_FLASH_ADDR_L);
        WriteByteCrc(O_PARAM_LEN);
        i = O_PARAM_LEN;
        loop  {
            while serialTxBytesFree(port) == 0 { }
            WriteByteCrc(*O_PARAM);
            O_PARAM = O_PARAM.offset(1);
            i = i.wrapping_sub(1);
            if !(i as libc::c_int > 0i32) { break ; }
        }
        WriteByteCrc(ACK_OUT);
        WriteByte(CRCout.bytes[1]);
        WriteByte(CRCout.bytes[0]);
        serialEndWrite(port);
        ledSet(0i32, 0i32 != 0);
        if isExitScheduled { esc4wayRelease(); return }
    };
}
