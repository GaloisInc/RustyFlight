use ::libc;
extern "C" {
    // Macros to convert between CLI bus number and I2CDevice.
    // I2C device address range in 7-bit address mode
    #[no_mangle]
    fn i2cHardwareConfigure(i2cConfig_0: *const i2cConfig_s);
    #[no_mangle]
    fn i2cInit(device: I2CDevice);
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
    #[no_mangle]
    fn spiPreInit();
    #[no_mangle]
    fn spiInit(device: SPIDevice) -> bool;
    #[no_mangle]
    fn spiPinConfigure(pConfig: *const spiPinConfig_s);
    #[no_mangle]
    static mut hardwareRevision: uint8_t;
    #[no_mangle]
    static mut i2cConfig_SystemArray: [i2cConfig_t; 2];
    #[no_mangle]
    static mut spiPinConfig_SystemArray: [spiPinConfig_t; 3];
}
pub type __uint8_t = libc::c_uchar;
pub type uint8_t = __uint8_t;
pub type ioTag_t = uint8_t;
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
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
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
pub type spiPinConfig_t = spiPinConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct spiPinConfig_s {
    pub ioTagSck: ioTag_t,
    pub ioTagMiso: ioTag_t,
    pub ioTagMosi: ioTag_t,
}
pub const AFF3_REV_2: awf3HardwareRevision_t = 2;
pub type awf3HardwareRevision_t = libc::c_uint;
pub const AFF3_REV_1: awf3HardwareRevision_t = 1;
pub const AFF3_UNKNOWN: awf3HardwareRevision_t = 0;
#[inline]
unsafe extern "C" fn i2cConfig(mut _index: libc::c_int)
 -> *const i2cConfig_t {
    return &mut *i2cConfig_SystemArray.as_mut_ptr().offset(_index as isize) as
               *mut i2cConfig_t;
}
#[inline]
unsafe extern "C" fn spiPinConfig(mut _index: libc::c_int)
 -> *const spiPinConfig_t {
    return &mut *spiPinConfig_SystemArray.as_mut_ptr().offset(_index as isize)
               as *mut spiPinConfig_t;
}
#[no_mangle]
pub unsafe extern "C" fn targetBusInit() {
    if hardwareRevision as libc::c_int == AFF3_REV_2 as libc::c_int {
        spiPinConfigure(spiPinConfig(0 as libc::c_int));
        spiPreInit();
        spiInit(SPIDEV_3);
    }
    i2cHardwareConfigure(i2cConfig(0 as libc::c_int));
    i2cInit(I2CDEV_2);
}
