use ::libc;
extern "C" {
    #[no_mangle]
    static mut gyro: gyro_t;
    #[no_mangle]
    fn gyroInit() -> bool;
    #[no_mangle]
    fn accInit(gyroTargetLooptime: uint32_t) -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub const RANGEFINDER_NONE: C2RustUnnamed_3 = 0;
pub const MAG_NONE: C2RustUnnamed_2 = 1;
pub const BARO_NONE: C2RustUnnamed_1 = 1;
pub const ACC_NONE: C2RustUnnamed_0 = 1;
pub const GYRO_NONE: C2RustUnnamed = 0;
pub type C2RustUnnamed = libc::c_uint;
pub const GYRO_FAKE: C2RustUnnamed = 15;
pub const GYRO_BMI160: C2RustUnnamed = 14;
pub const GYRO_ICM20689: C2RustUnnamed = 13;
pub const GYRO_ICM20649: C2RustUnnamed = 12;
pub const GYRO_ICM20608G: C2RustUnnamed = 11;
pub const GYRO_ICM20602: C2RustUnnamed = 10;
pub const GYRO_ICM20601: C2RustUnnamed = 9;
pub const GYRO_MPU9250: C2RustUnnamed = 8;
pub const GYRO_MPU6500: C2RustUnnamed = 7;
pub const GYRO_MPU6000: C2RustUnnamed = 6;
pub const GYRO_L3GD20: C2RustUnnamed = 5;
pub const GYRO_MPU3050: C2RustUnnamed = 4;
pub const GYRO_L3G4200D: C2RustUnnamed = 3;
pub const GYRO_MPU6050: C2RustUnnamed = 2;
pub const GYRO_DEFAULT: C2RustUnnamed = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
}
pub type gyro_t = gyro_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const ACC_FAKE: C2RustUnnamed_0 = 16;
pub const ACC_BMI160: C2RustUnnamed_0 = 15;
pub const ACC_ICM20689: C2RustUnnamed_0 = 14;
pub const ACC_ICM20649: C2RustUnnamed_0 = 13;
pub const ACC_ICM20608G: C2RustUnnamed_0 = 12;
pub const ACC_ICM20602: C2RustUnnamed_0 = 11;
pub const ACC_ICM20601: C2RustUnnamed_0 = 10;
pub const ACC_MPU9250: C2RustUnnamed_0 = 9;
pub const ACC_MPU6500: C2RustUnnamed_0 = 8;
pub const ACC_MPU6000: C2RustUnnamed_0 = 7;
pub const ACC_LSM303DLHC: C2RustUnnamed_0 = 6;
pub const ACC_BMA280: C2RustUnnamed_0 = 5;
pub const ACC_MMA8452: C2RustUnnamed_0 = 4;
pub const ACC_MPU6050: C2RustUnnamed_0 = 3;
pub const ACC_ADXL345: C2RustUnnamed_0 = 2;
pub const ACC_DEFAULT: C2RustUnnamed_0 = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const BARO_QMP6988: C2RustUnnamed_1 = 6;
pub const BARO_LPS: C2RustUnnamed_1 = 5;
pub const BARO_BMP280: C2RustUnnamed_1 = 4;
pub const BARO_MS5611: C2RustUnnamed_1 = 3;
pub const BARO_BMP085: C2RustUnnamed_1 = 2;
pub const BARO_DEFAULT: C2RustUnnamed_1 = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const MAG_QMC5883: C2RustUnnamed_2 = 5;
pub const MAG_AK8963: C2RustUnnamed_2 = 4;
pub const MAG_AK8975: C2RustUnnamed_2 = 3;
pub const MAG_HMC5883: C2RustUnnamed_2 = 2;
pub const MAG_DEFAULT: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const RANGEFINDER_UIB: C2RustUnnamed_3 = 7;
pub const RANGEFINDER_VL53L0X: C2RustUnnamed_3 = 6;
pub const RANGEFINDER_HCSR04I2C: C2RustUnnamed_3 = 5;
pub const RANGEFINDER_SRF10: C2RustUnnamed_3 = 4;
pub const RANGEFINDER_TF02: C2RustUnnamed_3 = 3;
pub const RANGEFINDER_TFMINI: C2RustUnnamed_3 = 2;
pub const RANGEFINDER_HCSR04: C2RustUnnamed_3 = 1;
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
// requestedSensors is not actually used
#[no_mangle]
pub static mut requestedSensors: [uint8_t; 5] =
    [GYRO_NONE as libc::c_int as uint8_t, ACC_NONE as libc::c_int as uint8_t,
     BARO_NONE as libc::c_int as uint8_t, MAG_NONE as libc::c_int as uint8_t,
     RANGEFINDER_NONE as libc::c_int as uint8_t];
#[no_mangle]
pub static mut detectedSensors: [uint8_t; 5] =
    [GYRO_NONE as libc::c_int as uint8_t, ACC_NONE as libc::c_int as uint8_t,
     BARO_NONE as libc::c_int as uint8_t, MAG_NONE as libc::c_int as uint8_t,
     RANGEFINDER_NONE as libc::c_int as uint8_t];
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
pub unsafe extern "C" fn sensorsAutodetect() -> bool {
    // gyro must be initialised before accelerometer
    let mut gyroDetected: bool = gyroInit();
    if gyroDetected { accInit(gyro.targetLooptime); }
    return gyroDetected;
}
