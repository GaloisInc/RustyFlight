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
    // mode is using only bits 6-2
    // declare available IO pins. Available pins are specified per target
    // unimplemented
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
/* * 
  * @brief Inter Integrated Circuit Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub OAR1: uint16_t,
    pub RESERVED2: uint16_t,
    pub OAR2: uint16_t,
    pub RESERVED3: uint16_t,
    pub DR: uint16_t,
    pub RESERVED4: uint16_t,
    pub SR1: uint16_t,
    pub RESERVED5: uint16_t,
    pub SR2: uint16_t,
    pub RESERVED6: uint16_t,
    pub CCR: uint16_t,
    pub RESERVED7: uint16_t,
    pub TRISE: uint16_t,
    pub RESERVED8: uint16_t,
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
    pub state: i2cState_t,
}
pub type i2cState_t = i2cState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cState_s {
    pub error: bool,
    pub busy: bool,
    pub addr: uint8_t,
    pub reg: uint8_t,
    pub bytes: uint8_t,
    pub writing: uint8_t,
    pub reading: uint8_t,
    pub write_p: *mut uint8_t,
    pub read_p: *mut uint8_t,
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
    while index < 2 as libc::c_int {
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
