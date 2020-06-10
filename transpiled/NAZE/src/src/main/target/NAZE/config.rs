use ::libc;
extern "C" {
    #[no_mangle]
    static mut accelerometerConfig_System: accelerometerConfig_t;
    #[no_mangle]
    static mut compassConfig_System: compassConfig_t;
    #[no_mangle]
    static mut beeperDevConfig_System: beeperDevConfig_t;
    #[no_mangle]
    static mut flashConfig_System: flashConfig_t;
    #[no_mangle]
    static mut hardwareRevision: uint8_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type ioTag_t = uint8_t;
pub type compassConfig_t = compassConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct compassConfig_s {
    pub mag_declination: int16_t,
    pub mag_align: sensor_align_e,
    pub mag_hardware: uint8_t,
    pub mag_bustype: uint8_t,
    pub mag_i2c_device: uint8_t,
    pub mag_i2c_address: uint8_t,
    pub mag_spi_device: uint8_t,
    pub mag_spi_csn: ioTag_t,
    pub interruptTag: ioTag_t,
    pub magZero: flightDynamicsTrims_t,
}
pub type flightDynamicsTrims_t = flightDynamicsTrims_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union flightDynamicsTrims_u {
    pub raw: [int16_t; 3],
    pub values: flightDynamicsTrims_def_t,
}
pub type flightDynamicsTrims_def_t = int16_flightDynamicsTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct int16_flightDynamicsTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
// Get your magnetic decliniation from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
// mag alignment
// Which mag hardware to use on boards with more than one device
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
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
// driver-provided alignment
pub const CW0_DEG: sensor_align_e = 1;
pub const ALIGN_DEFAULT: sensor_align_e = 0;
pub const NAZE32_REV5: nazeHardwareRevision_t = 2;
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
pub type flashConfig_t = flashConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flashConfig_s {
    pub csTag: ioTag_t,
    pub spiDevice: uint8_t,
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
pub type beeperDevConfig_t = beeperDevConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct beeperDevConfig_s {
    pub ioTag: ioTag_t,
    pub isInverted: uint8_t,
    pub isOpenDrain: uint8_t,
    pub frequency: uint16_t,
}
pub const ACC_NONE: C2RustUnnamed = 1;
pub type accelerometerConfig_t = accelerometerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accelerometerConfig_s {
    pub acc_lpf_hz: uint16_t,
    pub acc_align: sensor_align_e,
    pub acc_hardware: uint8_t,
    pub acc_high_fsr: bool,
    pub accZero: flightDynamicsTrims_t,
    pub accelerometerTrims: rollAndPitchTrims_t,
}
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union rollAndPitchTrims_u {
    pub raw: [int16_t; 2],
    pub values: rollAndPitchTrims_t_def,
}
pub type rollAndPitchTrims_t_def = rollAndPitchTrims_s;
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
// Type of accelerometer used/detected
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
}
pub const ACC_ADXL345: C2RustUnnamed = 2;
pub type C2RustUnnamed = libc::c_uint;
pub const ACC_FAKE: C2RustUnnamed = 16;
pub const ACC_BMI160: C2RustUnnamed = 15;
pub const ACC_ICM20689: C2RustUnnamed = 14;
pub const ACC_ICM20649: C2RustUnnamed = 13;
pub const ACC_ICM20608G: C2RustUnnamed = 12;
pub const ACC_ICM20602: C2RustUnnamed = 11;
pub const ACC_ICM20601: C2RustUnnamed = 10;
pub const ACC_MPU9250: C2RustUnnamed = 9;
pub const ACC_MPU6500: C2RustUnnamed = 8;
pub const ACC_MPU6000: C2RustUnnamed = 7;
pub const ACC_LSM303DLHC: C2RustUnnamed = 6;
pub const ACC_BMA280: C2RustUnnamed = 5;
pub const ACC_MMA8452: C2RustUnnamed = 4;
pub const ACC_MPU6050: C2RustUnnamed = 3;
pub const ACC_DEFAULT: C2RustUnnamed = 0;
pub type nazeHardwareRevision_t = libc::c_uint;
pub const NAZE32_SP: nazeHardwareRevision_t = 3;
pub const NAZE32: nazeHardwareRevision_t = 1;
pub const UNKNOWN: nazeHardwareRevision_t = 0;
#[inline]
unsafe extern "C" fn accelerometerConfigMutable()
 -> *mut accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn accelerometerConfig() -> *const accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn compassConfigMutable() -> *mut compassConfig_t {
    return &mut compassConfig_System;
}
#[inline]
unsafe extern "C" fn beeperDevConfigMutable() -> *mut beeperDevConfig_t {
    return &mut beeperDevConfig_System;
}
#[inline]
unsafe extern "C" fn flashConfigMutable() -> *mut flashConfig_t {
    return &mut flashConfig_System;
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
#[no_mangle]
pub unsafe extern "C" fn targetConfiguration() {
    if hardwareRevision as libc::c_int >= NAZE32_REV5 as libc::c_int {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        (*beeperDevConfigMutable()).isOpenDrain = 0 as libc::c_int as uint8_t;
        (*beeperDevConfigMutable()).isInverted = 1 as libc::c_int as uint8_t
    } else {
        (*beeperDevConfigMutable()).isOpenDrain = 1 as libc::c_int as uint8_t;
        (*beeperDevConfigMutable()).isInverted = 0 as libc::c_int as uint8_t;
        (*flashConfigMutable()).csTag = 0 as libc::c_int as ioTag_t
    }
    if (hardwareRevision as libc::c_int) < NAZE32_REV5 as libc::c_int {
        (*compassConfigMutable()).interruptTag =
            ((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
                 12 as libc::c_int) as ioTag_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn targetValidateConfiguration() {
    if (hardwareRevision as libc::c_int) < NAZE32_REV5 as libc::c_int &&
           (*accelerometerConfig()).acc_hardware as libc::c_int ==
               ACC_ADXL345 as libc::c_int {
        (*accelerometerConfigMutable()).acc_hardware =
            ACC_NONE as libc::c_int as uint8_t
    };
}
