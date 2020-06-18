use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    static i2cHardware: [i2cHardware_t; 0];
    #[no_mangle]
    static mut i2cDevice: [i2cDevice_t; 0];
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief Inter-integrated Circuit Interface
  */
#[derive ( Copy, Clone )]
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
    /* !< I2C Transmit data register,        Address offset: 0x28 */
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
// Macros to convert between CLI bus number and I2CDevice.
// I2C device address range in 7-bit address mode
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct i2cConfig_s {
    pub ioTagScl: ioTag_t,
    pub ioTagSda: ioTag_t,
    pub overClock: bool,
    pub pullUp: bool,
}
pub type i2cConfig_t = i2cConfig_s;
pub type i2cHardware_t = i2cHardware_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct i2cHardware_s {
    pub device: I2CDevice,
    pub reg: *mut I2C_TypeDef,
    pub sclPins: [i2cPinDef_t; 4],
    pub sdaPins: [i2cPinDef_t; 4],
    pub rcc: rccPeriphTag_t,
}
pub type i2cPinDef_t = i2cPinDef_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct i2cPinDef_s {
    pub ioTag: ioTag_t,
}
pub type i2cDevice_t = i2cDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct i2cDevice_s {
    pub hardware: *const i2cHardware_t,
    pub reg: *mut I2C_TypeDef,
    pub scl: IO_t,
    pub sda: IO_t,
    pub overClock: bool,
    pub pullUp: bool,
}
// MCU/Driver dependent member follows
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
    let mut index: libc::c_int = 0i32;
    while index < 2i32 {
        let mut hardware: *const i2cHardware_t =
            &*i2cHardware.as_ptr().offset(index as isize) as
                *const i2cHardware_t;
        if !(*hardware).reg.is_null() {
            let mut device: I2CDevice = (*hardware).device;
            let mut pDev: *mut i2cDevice_t =
                &mut *i2cDevice.as_mut_ptr().offset(device as isize) as
                    *mut i2cDevice_t;
            memset(pDev as *mut libc::c_void, 0i32,
                   ::core::mem::size_of::<i2cDevice_t>() as libc::c_ulong);
            let mut pindex: libc::c_int = 0i32;
            while pindex < 4i32 {
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
