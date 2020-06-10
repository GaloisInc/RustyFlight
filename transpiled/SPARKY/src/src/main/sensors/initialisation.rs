use ::libc;
extern "C" {
    #[no_mangle]
    static mut gyro: gyro_t;
    #[no_mangle]
    fn gyroInit() -> bool;
    #[no_mangle]
    fn accInit(gyroTargetLooptime: uint32_t) -> bool;
    #[no_mangle]
    static mut barometerConfig_System: barometerConfig_t;
    #[no_mangle]
    static mut baro: baro_t;
    #[no_mangle]
    fn baroDetect(dev: *mut baroDev_t, baroHardwareToUse: baroSensor_e)
     -> bool;
    #[no_mangle]
    fn compassInit() -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief Serial Peripheral Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
}
pub const RANGEFINDER_NONE: C2RustUnnamed_3 = 0;
pub const MAG_NONE: C2RustUnnamed_2 = 1;
pub const BARO_NONE: baroSensor_e = 1;
pub const ACC_NONE: C2RustUnnamed_1 = 1;
pub const GYRO_NONE: C2RustUnnamed_0 = 0;
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GYRO_FAKE: C2RustUnnamed_0 = 15;
pub const GYRO_BMI160: C2RustUnnamed_0 = 14;
pub const GYRO_ICM20689: C2RustUnnamed_0 = 13;
pub const GYRO_ICM20649: C2RustUnnamed_0 = 12;
pub const GYRO_ICM20608G: C2RustUnnamed_0 = 11;
pub const GYRO_ICM20602: C2RustUnnamed_0 = 10;
pub const GYRO_ICM20601: C2RustUnnamed_0 = 9;
pub const GYRO_MPU9250: C2RustUnnamed_0 = 8;
pub const GYRO_MPU6500: C2RustUnnamed_0 = 7;
pub const GYRO_MPU6000: C2RustUnnamed_0 = 6;
pub const GYRO_L3GD20: C2RustUnnamed_0 = 5;
pub const GYRO_MPU3050: C2RustUnnamed_0 = 4;
pub const GYRO_L3G4200D: C2RustUnnamed_0 = 3;
pub const GYRO_MPU6050: C2RustUnnamed_0 = 2;
pub const GYRO_DEFAULT: C2RustUnnamed_0 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
}
pub type gyro_t = gyro_s;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const ACC_FAKE: C2RustUnnamed_1 = 16;
pub const ACC_BMI160: C2RustUnnamed_1 = 15;
pub const ACC_ICM20689: C2RustUnnamed_1 = 14;
pub const ACC_ICM20649: C2RustUnnamed_1 = 13;
pub const ACC_ICM20608G: C2RustUnnamed_1 = 12;
pub const ACC_ICM20602: C2RustUnnamed_1 = 11;
pub const ACC_ICM20601: C2RustUnnamed_1 = 10;
pub const ACC_MPU9250: C2RustUnnamed_1 = 9;
pub const ACC_MPU6500: C2RustUnnamed_1 = 8;
pub const ACC_MPU6000: C2RustUnnamed_1 = 7;
pub const ACC_LSM303DLHC: C2RustUnnamed_1 = 6;
pub const ACC_BMA280: C2RustUnnamed_1 = 5;
pub const ACC_MMA8452: C2RustUnnamed_1 = 4;
pub const ACC_MPU6050: C2RustUnnamed_1 = 3;
pub const ACC_ADXL345: C2RustUnnamed_1 = 2;
pub const ACC_DEFAULT: C2RustUnnamed_1 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baroDev_s {
    pub busdev: busDevice_t,
    pub ut_delay: uint16_t,
    pub up_delay: uint16_t,
    pub start_ut: baroOpFuncPtr,
    pub get_ut: baroOpFuncPtr,
    pub start_up: baroOpFuncPtr,
    pub get_up: baroOpFuncPtr,
    pub calculate: baroCalculateFuncPtr,
}
// baro start operation
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
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
// XXX
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
// baro calculation (filled params are pressure and temperature)
pub type baroDev_t = baroDev_s;
pub type baroSensor_e = libc::c_uint;
pub const BARO_QMP6988: baroSensor_e = 6;
pub const BARO_LPS: baroSensor_e = 5;
pub const BARO_BMP280: baroSensor_e = 4;
pub const BARO_MS5611: baroSensor_e = 3;
pub const BARO_BMP085: baroSensor_e = 2;
pub const BARO_DEFAULT: baroSensor_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct barometerConfig_s {
    pub baro_bustype: uint8_t,
    pub baro_spi_device: uint8_t,
    pub baro_spi_csn: ioTag_t,
    pub baro_i2c_device: uint8_t,
    pub baro_i2c_address: uint8_t,
    pub baro_hardware: uint8_t,
    pub baro_sample_count: uint8_t,
    pub baro_noise_lpf: uint16_t,
    pub baro_cf_vel: uint16_t,
    pub baro_cf_alt: uint16_t,
}
pub type barometerConfig_t = barometerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baro_s {
    pub dev: baroDev_t,
    pub BaroAlt: int32_t,
    pub baroTemperature: int32_t,
    pub baroPressure: int32_t,
}
pub type baro_t = baro_s;
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
#[inline]
unsafe extern "C" fn barometerConfig() -> *const barometerConfig_t {
    return &mut barometerConfig_System;
}
// Also used as XCLR (positive logic) for BMP085
// Barometer hardware to use
// size of baro filter array
// additional LPF to reduce baro noise
// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
// apply CF to use ACC for height estimation
// Use temperature for telemetry
// Use pressure for telemetry
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
    compassInit();
    baroDetect(&mut baro.dev,
               (*barometerConfig()).baro_hardware as baroSensor_e);
    return gyroDetected;
}
