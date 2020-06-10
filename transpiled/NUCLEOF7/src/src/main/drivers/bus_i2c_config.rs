use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    static i2cHardware: [i2cHardware_t; 0];
    #[no_mangle]
    static mut i2cDevice: [i2cDevice_t; 0];
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
    // preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred
    // expand pinid to to ioTag_t
    //speed is packed inside modebits 5 and 2,
    // declare available IO pins. Available pins are specified per target
    // unimplemented
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub OAR1: uint32_t,
    pub OAR2: uint32_t,
    pub TIMINGR: uint32_t,
    pub TIMEOUTR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub PECR: uint32_t,
    pub RXDR: uint32_t,
    pub TXDR: uint32_t,
}
pub type HAL_StatusTypeDef = libc::c_uint;
pub const HAL_TIMEOUT: HAL_StatusTypeDef = 3;
pub const HAL_BUSY: HAL_StatusTypeDef = 2;
pub const HAL_ERROR: HAL_StatusTypeDef = 1;
pub const HAL_OK: HAL_StatusTypeDef = 0;
pub type HAL_LockTypeDef = libc::c_uint;
pub const HAL_LOCKED: HAL_LockTypeDef = 1;
pub const HAL_UNLOCKED: HAL_LockTypeDef = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub Channel: uint32_t,
    pub Direction: uint32_t,
    pub PeriphInc: uint32_t,
    pub MemInc: uint32_t,
    pub PeriphDataAlignment: uint32_t,
    pub MemDataAlignment: uint32_t,
    pub Mode: uint32_t,
    pub Priority: uint32_t,
    pub FIFOMode: uint32_t,
    pub FIFOThreshold: uint32_t,
    pub MemBurst: uint32_t,
    pub PeriphBurst: uint32_t,
}
pub type HAL_DMA_StateTypeDef = libc::c_uint;
pub const HAL_DMA_STATE_ABORT: HAL_DMA_StateTypeDef = 5;
pub const HAL_DMA_STATE_ERROR: HAL_DMA_StateTypeDef = 4;
pub const HAL_DMA_STATE_TIMEOUT: HAL_DMA_StateTypeDef = 3;
pub const HAL_DMA_STATE_BUSY: HAL_DMA_StateTypeDef = 2;
pub const HAL_DMA_STATE_READY: HAL_DMA_StateTypeDef = 1;
pub const HAL_DMA_STATE_RESET: HAL_DMA_StateTypeDef = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __DMA_HandleTypeDef {
    pub Instance: *mut DMA_Stream_TypeDef,
    pub Init: DMA_InitTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_DMA_StateTypeDef,
    pub Parent: *mut libc::c_void,
    pub XferCpltCallback: Option<unsafe extern "C" fn(_:
                                                          *mut __DMA_HandleTypeDef)
                                     -> ()>,
    pub XferHalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                              *mut __DMA_HandleTypeDef)
                                         -> ()>,
    pub XferM1CpltCallback: Option<unsafe extern "C" fn(_:
                                                            *mut __DMA_HandleTypeDef)
                                       -> ()>,
    pub XferM1HalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                                *mut __DMA_HandleTypeDef)
                                           -> ()>,
    pub XferErrorCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub XferAbortCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub ErrorCode: uint32_t,
    pub StreamBaseAddress: uint32_t,
    pub StreamIndex: uint32_t,
}
pub type DMA_HandleTypeDef = __DMA_HandleTypeDef;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_i2c.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of I2C HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup I2C
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup I2C_Exported_Types I2C Exported Types
  * @{
  */
/* * @defgroup I2C_Configuration_Structure_definition I2C Configuration Structure definition
  * @brief  I2C Configuration Structure definition  
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_InitTypeDef {
    pub Timing: uint32_t,
    pub OwnAddress1: uint32_t,
    pub AddressingMode: uint32_t,
    pub DualAddressMode: uint32_t,
    pub OwnAddress2: uint32_t,
    pub OwnAddress2Masks: uint32_t,
    pub GeneralCallMode: uint32_t,
    pub NoStretchMode: uint32_t,
}
/* * 
  * @}
  */
/* * @defgroup HAL_state_structure_definition HAL state structure definition
  * @brief  HAL State structure definition
  * @note  HAL I2C State value coding follow below described bitmap :\n
  *          b7-b6  Error information\n
  *             00 : No Error\n
  *             01 : Abort (Abort user request on going)\n
  *             10 : Timeout\n
  *             11 : Error\n
  *          b5     IP initilisation status\n
  *             0  : Reset (IP not initialized)\n
  *             1  : Init done (IP initialized and ready to use. HAL I2C Init function called)\n
  *          b4     (not used)\n
  *             x  : Should be set to 0\n
  *          b3\n
  *             0  : Ready or Busy (No Listen mode ongoing)\n
  *             1  : Listen (IP in Address Listen Mode)\n
  *          b2     Intrinsic process state\n
  *             0  : Ready\n
  *             1  : Busy (IP busy with some configuration or internal operations)\n
  *          b1     Rx state\n
  *             0  : Ready (no Rx operation ongoing)\n
  *             1  : Busy (Rx operation ongoing)\n
  *          b0     Tx state\n
  *             0  : Ready (no Tx operation ongoing)\n
  *             1  : Busy (Tx operation ongoing)
  * @{
  */
pub type HAL_I2C_StateTypeDef = libc::c_uint;
/* !< Error                                     */
/* !< Timeout state                             */
pub const HAL_I2C_STATE_ERROR: HAL_I2C_StateTypeDef = 224;
/* !< Abort user request ongoing                */
pub const HAL_I2C_STATE_TIMEOUT: HAL_I2C_StateTypeDef = 160;
/* !< Address Listen Mode and Data Reception
                                                 process is ongoing                         */
pub const HAL_I2C_STATE_ABORT: HAL_I2C_StateTypeDef = 96;
/* !< Address Listen Mode and Data Transmission
                                                 process is ongoing                         */
pub const HAL_I2C_STATE_BUSY_RX_LISTEN: HAL_I2C_StateTypeDef = 42;
/* !< Address Listen Mode is ongoing            */
pub const HAL_I2C_STATE_BUSY_TX_LISTEN: HAL_I2C_StateTypeDef = 41;
/* !< Data Reception process is ongoing         */
pub const HAL_I2C_STATE_LISTEN: HAL_I2C_StateTypeDef = 40;
/* !< Data Transmission process is ongoing      */
pub const HAL_I2C_STATE_BUSY_RX: HAL_I2C_StateTypeDef = 34;
/* !< An internal process is ongoing            */
pub const HAL_I2C_STATE_BUSY_TX: HAL_I2C_StateTypeDef = 33;
/* !< Peripheral Initialized and ready for use  */
pub const HAL_I2C_STATE_BUSY: HAL_I2C_StateTypeDef = 36;
/* !< Peripheral is not yet Initialized         */
pub const HAL_I2C_STATE_READY: HAL_I2C_StateTypeDef = 32;
pub const HAL_I2C_STATE_RESET: HAL_I2C_StateTypeDef = 0;
/* *
  * @}
  */
/* * @defgroup HAL_mode_structure_definition HAL mode structure definition
  * @brief  HAL Mode structure definition
  * @note  HAL I2C Mode value coding follow below described bitmap :\n
  *          b7     (not used)\n
  *             x  : Should be set to 0\n
  *          b6\n
  *             0  : None\n
  *             1  : Memory (HAL I2C communication is in Memory Mode)\n
  *          b5\n
  *             0  : None\n
  *             1  : Slave (HAL I2C communication is in Slave Mode)\n
  *          b4\n
  *             0  : None\n
  *             1  : Master (HAL I2C communication is in Master Mode)\n
  *          b3-b2-b1-b0  (not used)\n
  *             xxxx : Should be set to 0000
  * @{
  */
pub type HAL_I2C_ModeTypeDef = libc::c_uint;
/* !< I2C communication is in Memory Mode       */
/* !< I2C communication is in Slave Mode        */
pub const HAL_I2C_MODE_MEM: HAL_I2C_ModeTypeDef = 64;
/* !< I2C communication is in Master Mode       */
pub const HAL_I2C_MODE_SLAVE: HAL_I2C_ModeTypeDef = 32;
/* !< No I2C communication on going             */
pub const HAL_I2C_MODE_MASTER: HAL_I2C_ModeTypeDef = 16;
pub const HAL_I2C_MODE_NONE: HAL_I2C_ModeTypeDef = 0;
/* * 
  * @}
  */
/* * @defgroup I2C_Error_Code_definition I2C Error Code definition
  * @brief  I2C Error Code definition
  * @{
  */
/* !< No error              */
/* !< BERR error            */
/* !< ARLO error            */
/* !< ACKF error            */
/* !< OVR error             */
/* !< DMA transfer error    */
/* !< Timeout error         */
/* !< Size Management error */
/* *
  * @}
  */
/* * @defgroup I2C_handle_Structure_definition I2C handle Structure definition
  * @brief  I2C handle Structure definition
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __I2C_HandleTypeDef {
    pub Instance: *mut I2C_TypeDef,
    pub Init: I2C_InitTypeDef,
    pub pBuffPtr: *mut uint8_t,
    pub XferSize: uint16_t,
    pub XferCount: uint16_t,
    pub XferOptions: uint32_t,
    pub PreviousState: uint32_t,
    pub XferISR: Option<unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                             _: uint32_t, _: uint32_t)
                            -> HAL_StatusTypeDef>,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_I2C_StateTypeDef,
    pub Mode: HAL_I2C_ModeTypeDef,
    pub ErrorCode: uint32_t,
    pub AddrEventCount: uint32_t,
}
pub type I2C_HandleTypeDef = __I2C_HandleTypeDef;
/* !< I2C Address Event counter                 */
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
pub type rccPeriphTag_t = uint8_t;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cConfig_s {
    pub ioTagScl: ioTag_t,
    pub ioTagSda: ioTag_t,
    pub overClock: bool,
    pub pullUp: bool,
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
pub type i2cConfig_t = i2cConfig_s;
pub type i2cHardware_t = i2cHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cHardware_s {
    pub device: I2CDevice,
    pub reg: *mut I2C_TypeDef,
    pub sclPins: [i2cPinDef_t; 4],
    pub sdaPins: [i2cPinDef_t; 4],
    pub rcc: rccPeriphTag_t,
    pub ev_irq: uint8_t,
    pub er_irq: uint8_t,
}
pub type i2cPinDef_t = i2cPinDef_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cPinDef_s {
    pub ioTag: ioTag_t,
}
pub type i2cDevice_t = i2cDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cDevice_s {
    pub hardware: *const i2cHardware_t,
    pub reg: *mut I2C_TypeDef,
    pub scl: IO_t,
    pub sda: IO_t,
    pub overClock: bool,
    pub pullUp: bool,
    pub handle: I2C_HandleTypeDef,
}
// MCU/Driver dependent member follows
// Macros to convert between CLI bus number and I2CDevice.
// I2C device address range in 7-bit address mode
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
 * Created by jflyper
 */
// Backward compatibility for overclocking and internal pullup.
// These will eventually be configurable through PG-based configurator
// (and/or probably through some cli extension).
#[no_mangle]
pub unsafe extern "C" fn i2cHardwareConfigure(mut i2cConfig:
                                                  *const i2cConfig_t) {
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < 4 as libc::c_int {
        let mut hardware: *const i2cHardware_t =
            &*i2cHardware.as_ptr().offset(index as isize) as
                *const i2cHardware_t;
        if !(*hardware).reg.is_null() {
            let mut device: I2CDevice = (*hardware).device;
            let mut pDev: *mut i2cDevice_t =
                &mut *i2cDevice.as_mut_ptr().offset(device as isize) as
                    *mut i2cDevice_t;
            memset(pDev as *mut libc::c_void, 0 as libc::c_int,
                   ::core::mem::size_of::<i2cDevice_t>() as libc::c_ulong);
            let mut pindex: libc::c_int = 0 as libc::c_int;
            while pindex < 4 as libc::c_int {
                if (*i2cConfig.offset(device as isize)).ioTagScl as
                       libc::c_int ==
                       (*hardware).sclPins[pindex as usize].ioTag as
                           libc::c_int {
                    (*pDev).scl =
                        IOGetByTag((*i2cConfig.offset(device as
                                                          isize)).ioTagScl)
                }
                if (*i2cConfig.offset(device as isize)).ioTagSda as
                       libc::c_int ==
                       (*hardware).sdaPins[pindex as usize].ioTag as
                           libc::c_int {
                    (*pDev).sda =
                        IOGetByTag((*i2cConfig.offset(device as
                                                          isize)).ioTagSda)
                }
                pindex += 1
            }
            if !(*pDev).scl.is_null() && !(*pDev).sda.is_null() {
                (*pDev).hardware = hardware;
                (*pDev).reg = (*hardware).reg;
                (*pDev).overClock =
                    (*i2cConfig.offset(device as isize)).overClock;
                (*pDev).pullUp = (*i2cConfig.offset(device as isize)).pullUp
            }
        }
        index += 1
    };
}
// defined(USE_I2C) && !defined(USE_SOFT_I2C)
