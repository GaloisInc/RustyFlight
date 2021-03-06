use core;
use libc;
extern "C" {
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
    pub type filter_s;
    #[no_mangle]
    fn fabsf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn abs(_: libc::c_int) -> libc::c_int;
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
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn devClear(dev: *mut stdev_t);
    #[no_mangle]
    fn devPush(dev: *mut stdev_t, x: libc::c_float);
    #[no_mangle]
    fn devStandardDeviation(dev: *mut stdev_t) -> libc::c_float;
    #[no_mangle]
    fn nullFilterApply(filter: *mut filter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn biquadFilterInitLPF(filter: *mut biquadFilter_t,
                           filterFreq: libc::c_float, refreshRate: uint32_t);
    #[no_mangle]
    fn biquadFilterInit(filter: *mut biquadFilter_t,
                        filterFreq: libc::c_float, refreshRate: uint32_t,
                        Q: libc::c_float, filterType: biquadFilterType_e);
    #[no_mangle]
    fn biquadFilterApply(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn filterGetNotchQ(centerFreq: libc::c_float, cutoffFreq: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn pt1FilterGain(f_cut: uint16_t, dT: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn pt1FilterInit(filter: *mut pt1Filter_t, k: libc::c_float);
    #[no_mangle]
    fn pt1FilterApply(filter: *mut pt1Filter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn mpuGyroReadRegister(bus: *const busDevice_t, reg: uint8_t) -> uint8_t;
    #[no_mangle]
    fn fakeGyroDetect(gyro_0: *mut gyroDev_s) -> bool;
    #[no_mangle]
    fn gyroSetSampleRate(gyro_0: *mut gyroDev_t, lpf: uint8_t,
                         gyroSyncDenominator: uint8_t, gyro_use_32khz: bool)
     -> uint32_t;
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    fn sensorsSet(mask: uint32_t);
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    #[no_mangle]
    fn schedulerResetTaskStatistics(taskId: cfTaskId_e);
    #[no_mangle]
    fn alignSensors(dest: *mut libc::c_float, rotation: uint8_t);
    #[no_mangle]
    static mut detectedSensors: [uint8_t; 5];
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __pthread_internal_list {
    pub __prev: *mut __pthread_internal_list,
    pub __next: *mut __pthread_internal_list,
}
pub type __pthread_list_t = __pthread_internal_list;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __pthread_mutex_s {
    pub __lock: libc::c_int,
    pub __count: libc::c_uint,
    pub __owner: libc::c_int,
    pub __nusers: libc::c_uint,
    pub __kind: libc::c_int,
    pub __spins: libc::c_short,
    pub __elision: libc::c_short,
    pub __list: __pthread_list_t,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union pthread_mutex_t {
    pub __data: __pthread_mutex_s,
    pub __size: [libc::c_char; 40],
    pub __align: libc::c_long,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SPI_TypeDef {
    pub test: *mut libc::c_void,
}
pub type C2RustUnnamed = libc::c_uint;
pub const DEBUG_COUNT: C2RustUnnamed = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed = 38;
pub const DEBUG_RTH: C2RustUnnamed = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed = 36;
pub const DEBUG_USB: C2RustUnnamed = 35;
pub const DEBUG_CURRENT: C2RustUnnamed = 34;
pub const DEBUG_SDIO: C2RustUnnamed = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed = 28;
pub const DEBUG_FPORT: C2RustUnnamed = 27;
pub const DEBUG_SBUS: C2RustUnnamed = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed = 16;
pub const DEBUG_FFT: C2RustUnnamed = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed = 12;
pub const DEBUG_STACK: C2RustUnnamed = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed = 3;
pub const DEBUG_BATTERY: C2RustUnnamed = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed = 1;
pub const DEBUG_NONE: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const Z: C2RustUnnamed_0 = 2;
pub const Y: C2RustUnnamed_0 = 1;
pub const X: C2RustUnnamed_0 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct stdev_s {
    pub m_oldM: libc::c_float,
    pub m_newM: libc::c_float,
    pub m_oldS: libc::c_float,
    pub m_newS: libc::c_float,
    pub m_n: libc::c_int,
}
pub type stdev_t = stdev_s;
pub type filter_t = filter_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pt1Filter_s {
    pub state: libc::c_float,
    pub k: libc::c_float,
}
pub type pt1Filter_t = pt1Filter_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct biquadFilter_s {
    pub b0: libc::c_float,
    pub b1: libc::c_float,
    pub b2: libc::c_float,
    pub a1: libc::c_float,
    pub a2: libc::c_float,
    pub x1: libc::c_float,
    pub x2: libc::c_float,
    pub y1: libc::c_float,
    pub y2: libc::c_float,
}
/* this holds the data required to update samples thru a filter */
pub type biquadFilter_t = biquadFilter_s;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FILTER_BIQUAD: C2RustUnnamed_1 = 1;
pub const FILTER_PT1: C2RustUnnamed_1 = 0;
pub type biquadFilterType_e = libc::c_uint;
pub const FILTER_BPF: biquadFilterType_e = 2;
pub const FILTER_NOTCH: biquadFilterType_e = 1;
pub const FILTER_LPF: biquadFilterType_e = 0;
pub type filterApplyFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
               -> libc::c_float>;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_2 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_2 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_2 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_2 = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_3,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_3 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
}
pub type pgRegistry_t = pgRegistry_s;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
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
pub type IO_t = *mut libc::c_void;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ()>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
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
pub type extiCallbackRec_t = extiCallbackRec_s;
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
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_4,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_4 {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
// Slave I2C on SPI master
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
pub type mpuDetectionResult_t = mpuDetectionResult_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
}
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
pub type mpuSensor_e = libc::c_uint;
pub const BMI_160_SPI: mpuSensor_e = 12;
pub const ICM_20689_SPI: mpuSensor_e = 11;
pub const ICM_20649_SPI: mpuSensor_e = 10;
pub const ICM_20608_SPI: mpuSensor_e = 9;
pub const ICM_20602_SPI: mpuSensor_e = 8;
pub const ICM_20601_SPI: mpuSensor_e = 7;
pub const MPU_9250_SPI: mpuSensor_e = 6;
pub const MPU_65xx_SPI: mpuSensor_e = 5;
pub const MPU_65xx_I2C: mpuSensor_e = 4;
pub const MPU_60x0_SPI: mpuSensor_e = 3;
pub const MPU_60x0: mpuSensor_e = 2;
pub const MPU_3050: mpuSensor_e = 1;
pub const MPU_NONE: mpuSensor_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gyroDev_s {
    pub lock: pthread_mutex_t,
    pub initFn: sensorGyroInitFuncPtr,
    pub readFn: sensorGyroReadFuncPtr,
    pub temperatureFn: sensorGyroReadDataFuncPtr,
    pub exti: extiCallbackRec_t,
    pub bus: busDevice_t,
    pub scale: libc::c_float,
    pub gyroZero: [libc::c_float; 3],
    pub gyroADC: [libc::c_float; 3],
    pub gyroADCf: [libc::c_float; 3],
    pub gyroADCRawPrevious: [int32_t; 3],
    pub gyroADCRaw: [int16_t; 3],
    pub temperature: int16_t,
    pub mpuConfiguration: mpuConfiguration_t,
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub gyroAlign: sensor_align_e,
    pub gyroRateKHz: gyroRateKHz_e,
    pub dataReady: bool,
    pub gyro_high_fsr: bool,
    pub hardware_lpf: uint8_t,
    pub hardware_32khz_lpf: uint8_t,
    pub mpuDividerDrops: uint8_t,
    pub mpuIntExtiTag: ioTag_t,
    pub gyroHasOverflowProtection: uint8_t,
    pub gyroHardware: gyroSensor_e,
}
pub type gyroSensor_e = libc::c_uint;
pub const GYRO_FAKE: gyroSensor_e = 15;
pub const GYRO_BMI160: gyroSensor_e = 14;
pub const GYRO_ICM20689: gyroSensor_e = 13;
pub const GYRO_ICM20649: gyroSensor_e = 12;
pub const GYRO_ICM20608G: gyroSensor_e = 11;
pub const GYRO_ICM20602: gyroSensor_e = 10;
pub const GYRO_ICM20601: gyroSensor_e = 9;
pub const GYRO_MPU9250: gyroSensor_e = 8;
pub const GYRO_MPU6500: gyroSensor_e = 7;
pub const GYRO_MPU6000: gyroSensor_e = 6;
pub const GYRO_L3GD20: gyroSensor_e = 5;
pub const GYRO_MPU3050: gyroSensor_e = 4;
pub const GYRO_L3G4200D: gyroSensor_e = 3;
pub const GYRO_MPU6050: gyroSensor_e = 2;
pub const GYRO_DEFAULT: gyroSensor_e = 1;
pub const GYRO_NONE: gyroSensor_e = 0;
pub type gyroRateKHz_e = libc::c_uint;
pub const GYRO_RATE_32_kHz: gyroRateKHz_e = 5;
pub const GYRO_RATE_9_kHz: gyroRateKHz_e = 4;
pub const GYRO_RATE_8_kHz: gyroRateKHz_e = 3;
pub const GYRO_RATE_3200_Hz: gyroRateKHz_e = 2;
pub const GYRO_RATE_1100_Hz: gyroRateKHz_e = 1;
pub const GYRO_RATE_1_kHz: gyroRateKHz_e = 0;
pub type mpuConfiguration_t = mpuConfiguration_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mpuConfiguration_s {
    pub resetFn: mpuResetFnPtr,
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
//#define DEBUG_MPU_DATA_READY_INTERRUPT
// MPU6050
// MPU3050, 6000 and 6050
// RA = Register Address
//[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
//[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
//[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
//[7:0] X_FINE_GAIN
//[7:0] Y_FINE_GAIN
//[7:0] Z_FINE_GAIN
//[15:0] XA_OFFS
//[15:0] YA_OFFS
//[15:0] ZA_OFFS
// Product ID Register
//[15:0] XG_OFFS_USR
//[15:0] YG_OFFS_USR
//[15:0] ZG_OFFS_USR
// RF = Register Flag
pub type mpuResetFnPtr = Option<unsafe extern "C" fn() -> ()>;
pub type sensorGyroReadDataFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s, _: *mut int16_t) -> bool>;
pub type sensorGyroReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s) -> bool>;
pub type sensorGyroInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s) -> ()>;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const GYRO_OVERFLOW_Z: C2RustUnnamed_5 = 4;
pub const GYRO_OVERFLOW_Y: C2RustUnnamed_5 = 2;
pub const GYRO_OVERFLOW_X: C2RustUnnamed_5 = 1;
pub const GYRO_OVERFLOW_NONE: C2RustUnnamed_5 = 0;
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
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
}
pub type gyro_t = gyro_s;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const GYRO_OVERFLOW_CHECK_ALL_AXES: C2RustUnnamed_6 = 2;
pub const GYRO_OVERFLOW_CHECK_YAW: C2RustUnnamed_6 = 1;
pub const GYRO_OVERFLOW_CHECK_NONE: C2RustUnnamed_6 = 0;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const FILTER_LOWPASS2: C2RustUnnamed_7 = 1;
pub const FILTER_LOWPASS: C2RustUnnamed_7 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gyroConfig_s {
    pub gyro_align: uint8_t,
    pub gyroMovementCalibrationThreshold: uint8_t,
    pub gyro_sync_denom: uint8_t,
    pub gyro_hardware_lpf: uint8_t,
    pub gyro_32khz_hardware_lpf: uint8_t,
    pub gyro_high_fsr: uint8_t,
    pub gyro_use_32khz: uint8_t,
    pub gyro_to_use: uint8_t,
    pub gyro_lowpass_hz: uint16_t,
    pub gyro_lowpass2_hz: uint16_t,
    pub gyro_soft_notch_hz_1: uint16_t,
    pub gyro_soft_notch_cutoff_1: uint16_t,
    pub gyro_soft_notch_hz_2: uint16_t,
    pub gyro_soft_notch_cutoff_2: uint16_t,
    pub gyro_offset_yaw: int16_t,
    pub checkOverflow: uint8_t,
    pub gyro_lowpass_type: uint8_t,
    pub gyro_lowpass2_type: uint8_t,
    pub yaw_spin_recovery: uint8_t,
    pub yaw_spin_threshold: int16_t,
    pub gyroCalibrationDuration: uint16_t,
    pub dyn_notch_quality: uint8_t,
    pub dyn_notch_width_percent: uint8_t,
}
pub type gyroConfig_t = gyroConfig_s;
pub type gyroDev_t = gyroDev_s;
pub type gyroSensor_t = gyroSensor_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gyroSensor_s {
    pub gyroDev: gyroDev_t,
    pub calibration: gyroCalibration_t,
    pub lowpassFilterApplyFn: filterApplyFnPtr,
    pub lowpassFilter: [gyroLowpassFilter_t; 3],
    pub lowpass2FilterApplyFn: filterApplyFnPtr,
    pub lowpass2Filter: [gyroLowpassFilter_t; 3],
    pub notchFilter1ApplyFn: filterApplyFnPtr,
    pub notchFilter1: [biquadFilter_t; 3],
    pub notchFilter2ApplyFn: filterApplyFnPtr,
    pub notchFilter2: [biquadFilter_t; 3],
    pub notchFilterDynApplyFn: filterApplyFnPtr,
    pub notchFilterDyn: [biquadFilter_t; 3],
    pub overflowTimeUs: timeUs_t,
    pub overflowDetected: bool,
    pub yawSpinTimeUs: timeUs_t,
    pub yawSpinDetected: bool,
}
pub type gyroLowpassFilter_t = gyroLowpassFilter_u;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union gyroLowpassFilter_u {
    pub pt1FilterState: pt1Filter_t,
    pub biquadFilterState: biquadFilter_t,
}
pub type gyroCalibration_t = gyroCalibration_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gyroCalibration_s {
    pub sum: [libc::c_float; 3],
    pub var: [stdev_t; 3],
    pub cyclesRemaining: int32_t,
}
pub const SENSOR_GYRO: C2RustUnnamed_9 = 1;
pub const SENSOR_INDEX_GYRO: C2RustUnnamed_8 = 0;
pub type beeperMode_e = libc::c_uint;
pub const BEEPER_ALL: beeperMode_e = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: beeperMode_e = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: beeperMode_e = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: beeperMode_e = 21;
pub const BEEPER_CRASH_FLIP_MODE: beeperMode_e = 20;
pub const BEEPER_BLACKBOX_ERASE: beeperMode_e = 19;
pub const BEEPER_USB: beeperMode_e = 18;
pub const BEEPER_SYSTEM_INIT: beeperMode_e = 17;
pub const BEEPER_ARMED: beeperMode_e = 16;
pub const BEEPER_DISARM_REPEAT: beeperMode_e = 15;
pub const BEEPER_MULTI_BEEPS: beeperMode_e = 14;
pub const BEEPER_READY_BEEP: beeperMode_e = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: beeperMode_e = 12;
pub const BEEPER_ACC_CALIBRATION: beeperMode_e = 11;
pub const BEEPER_RX_SET: beeperMode_e = 10;
pub const BEEPER_GPS_STATUS: beeperMode_e = 9;
pub const BEEPER_BAT_LOW: beeperMode_e = 8;
pub const BEEPER_BAT_CRIT_LOW: beeperMode_e = 7;
pub const BEEPER_ARMING_GPS_FIX: beeperMode_e = 6;
pub const BEEPER_ARMING: beeperMode_e = 5;
pub const BEEPER_DISARMING: beeperMode_e = 4;
pub const BEEPER_RX_LOST_LANDING: beeperMode_e = 3;
pub const BEEPER_RX_LOST: beeperMode_e = 2;
pub const BEEPER_GYRO_CALIBRATED: beeperMode_e = 1;
pub const BEEPER_SILENCE: beeperMode_e = 0;
pub const ARMING_DISABLED_CALIBRATING: armingDisableFlags_e = 2048;
pub type armingDisableFlags_e = libc::c_uint;
pub const ARMING_DISABLED_ARM_SWITCH: armingDisableFlags_e = 524288;
pub const ARMING_DISABLED_GPS: armingDisableFlags_e = 262144;
pub const ARMING_DISABLED_PARALYZE: armingDisableFlags_e = 131072;
pub const ARMING_DISABLED_MSP: armingDisableFlags_e = 65536;
pub const ARMING_DISABLED_BST: armingDisableFlags_e = 32768;
pub const ARMING_DISABLED_OSD_MENU: armingDisableFlags_e = 16384;
pub const ARMING_DISABLED_CMS_MENU: armingDisableFlags_e = 8192;
pub const ARMING_DISABLED_CLI: armingDisableFlags_e = 4096;
pub const ARMING_DISABLED_LOAD: armingDisableFlags_e = 1024;
pub const ARMING_DISABLED_NOPREARM: armingDisableFlags_e = 512;
pub const ARMING_DISABLED_BOOT_GRACE_TIME: armingDisableFlags_e = 256;
pub const ARMING_DISABLED_ANGLE: armingDisableFlags_e = 128;
pub const ARMING_DISABLED_THROTTLE: armingDisableFlags_e = 64;
pub const ARMING_DISABLED_RUNAWAY_TAKEOFF: armingDisableFlags_e = 32;
pub const ARMING_DISABLED_BOXFAILSAFE: armingDisableFlags_e = 16;
pub const ARMING_DISABLED_BAD_RX_RECOVERY: armingDisableFlags_e = 8;
pub const ARMING_DISABLED_RX_FAILSAFE: armingDisableFlags_e = 4;
pub const ARMING_DISABLED_FAILSAFE: armingDisableFlags_e = 2;
pub const ARMING_DISABLED_NO_GYRO: armingDisableFlags_e = 1;
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 19;
pub const TASK_NONE: cfTaskId_e = 18;
pub const TASK_COUNT: cfTaskId_e = 18;
pub const TASK_PINIOBOX: cfTaskId_e = 17;
pub const TASK_RCDEVICE: cfTaskId_e = 16;
pub const TASK_TELEMETRY: cfTaskId_e = 15;
pub const TASK_ALTITUDE: cfTaskId_e = 14;
pub const TASK_BARO: cfTaskId_e = 13;
pub const TASK_COMPASS: cfTaskId_e = 12;
pub const TASK_GPS: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const SENSOR_INDEX_COUNT: C2RustUnnamed_8 = 5;
pub const SENSOR_INDEX_RANGEFINDER: C2RustUnnamed_8 = 4;
pub const SENSOR_INDEX_MAG: C2RustUnnamed_8 = 3;
pub const SENSOR_INDEX_BARO: C2RustUnnamed_8 = 2;
pub const SENSOR_INDEX_ACC: C2RustUnnamed_8 = 1;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_9 = 64;
pub const SENSOR_GPS: C2RustUnnamed_9 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_9 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_9 = 16;
pub const SENSOR_MAG: C2RustUnnamed_9 = 8;
pub const SENSOR_BARO: C2RustUnnamed_9 = 4;
pub const SENSOR_ACC: C2RustUnnamed_9 = 2;
// lowpass gyro soft filter
// lowpass2 gyro soft filter
// notch filters
// overflow and recovery
// USE_YAW_SPIN_RECOVERY
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
// SITL (software in the loop) simulator
// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC
// file name to save config
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
#[inline]
unsafe extern "C" fn gyroConfig() -> *const gyroConfig_t {
    return &mut gyroConfig_System;
}
#[inline]
unsafe extern "C" fn gyroConfigMutable() -> *mut gyroConfig_t {
    return &mut gyroConfig_System;
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
pub static mut gyro: gyro_t = gyro_t{targetLooptime: 0, gyroADCf: [0.; 3],};
static mut gyroDebugMode: uint8_t = 0;
static mut gyroToUse: uint8_t = 0i32 as uint8_t;
static mut overflowAxisMask: uint8_t = 0;
static mut accumulatedMeasurements: [libc::c_float; 3] = [0.; 3];
static mut gyroPrevious: [libc::c_float; 3] = [0.; 3];
static mut accumulatedMeasurementTimeUs: timeUs_t = 0;
static mut accumulationLastTimeSampledUs: timeUs_t = 0;
static mut gyroHasOverflowProtection: bool = 1i32 != 0;
#[no_mangle]
pub static mut firstArmingCalibrationWasStarted: bool = 0i32 != 0;
static mut gyroSensor1: gyroSensor_t =
    gyroSensor_t{gyroDev:
                     gyroDev_t{lock:
                                   pthread_mutex_t{__data:
                                                       __pthread_mutex_s{__lock:
                                                                             0,
                                                                         __count:
                                                                             0,
                                                                         __owner:
                                                                             0,
                                                                         __nusers:
                                                                             0,
                                                                         __kind:
                                                                             0,
                                                                         __spins:
                                                                             0,
                                                                         __elision:
                                                                             0,
                                                                         __list:
                                                                             __pthread_list_t{__prev:
                                                                                                  0
                                                                                                      as
                                                                                                      *const __pthread_internal_list
                                                                                                      as
                                                                                                      *mut __pthread_internal_list,
                                                                                              __next:
                                                                                                  0
                                                                                                      as
                                                                                                      *const __pthread_internal_list
                                                                                                      as
                                                                                                      *mut __pthread_internal_list,},},},
                               initFn: None,
                               readFn: None,
                               temperatureFn: None,
                               exti: extiCallbackRec_t{fn_0: None,},
                               bus:
                                   busDevice_t{bustype: BUSTYPE_NONE,
                                               busdev_u:
                                                   C2RustUnnamed_4{spi:
                                                                       deviceSpi_s{instance:
                                                                                       0
                                                                                           as
                                                                                           *const SPI_TypeDef
                                                                                           as
                                                                                           *mut SPI_TypeDef,
                                                                                   csnPin:
                                                                                       0
                                                                                           as
                                                                                           *const libc::c_void
                                                                                           as
                                                                                           *mut libc::c_void,},},},
                               scale: 0.,
                               gyroZero: [0.; 3],
                               gyroADC: [0.; 3],
                               gyroADCf: [0.; 3],
                               gyroADCRawPrevious: [0; 3],
                               gyroADCRaw: [0; 3],
                               temperature: 0,
                               mpuConfiguration:
                                   mpuConfiguration_t{resetFn: None,},
                               mpuDetectionResult:
                                   mpuDetectionResult_t{sensor: MPU_NONE,
                                                        resolution:
                                                            MPU_HALF_RESOLUTION,},
                               gyroAlign: ALIGN_DEFAULT,
                               gyroRateKHz: GYRO_RATE_1_kHz,
                               dataReady: false,
                               gyro_high_fsr: false,
                               hardware_lpf: 0,
                               hardware_32khz_lpf: 0,
                               mpuDividerDrops: 0,
                               mpuIntExtiTag: 0,
                               gyroHasOverflowProtection: 0,
                               gyroHardware: GYRO_NONE,},
                 calibration:
                     gyroCalibration_t{sum: [0.; 3],
                                       var:
                                           [stdev_t{m_oldM: 0.,
                                                    m_newM: 0.,
                                                    m_oldS: 0.,
                                                    m_newS: 0.,
                                                    m_n: 0,}; 3],
                                       cyclesRemaining: 0,},
                 lowpassFilterApplyFn: None,
                 lowpassFilter:
                     [gyroLowpassFilter_u{pt1FilterState:
                                              pt1Filter_t{state: 0.,
                                                          k: 0.,},}; 3],
                 lowpass2FilterApplyFn: None,
                 lowpass2Filter:
                     [gyroLowpassFilter_u{pt1FilterState:
                                              pt1Filter_t{state: 0.,
                                                          k: 0.,},}; 3],
                 notchFilter1ApplyFn: None,
                 notchFilter1:
                     [biquadFilter_t{b0: 0.,
                                     b1: 0.,
                                     b2: 0.,
                                     a1: 0.,
                                     a2: 0.,
                                     x1: 0.,
                                     x2: 0.,
                                     y1: 0.,
                                     y2: 0.,}; 3],
                 notchFilter2ApplyFn: None,
                 notchFilter2:
                     [biquadFilter_t{b0: 0.,
                                     b1: 0.,
                                     b2: 0.,
                                     a1: 0.,
                                     a2: 0.,
                                     x1: 0.,
                                     x2: 0.,
                                     y1: 0.,
                                     y2: 0.,}; 3],
                 notchFilterDynApplyFn: None,
                 notchFilterDyn:
                     [biquadFilter_t{b0: 0.,
                                     b1: 0.,
                                     b2: 0.,
                                     a1: 0.,
                                     a2: 0.,
                                     x1: 0.,
                                     x2: 0.,
                                     y1: 0.,
                                     y2: 0.,}; 3],
                 overflowTimeUs: 0,
                 overflowDetected: false,
                 yawSpinTimeUs: 0,
                 yawSpinDetected: false,};
// 92.5% full scale (1850dps for 2000dps gyro)
#[no_mangle]
pub static mut gyroConfig_System: gyroConfig_t =
    gyroConfig_t{gyro_align: 0,
                 gyroMovementCalibrationThreshold: 0,
                 gyro_sync_denom: 0,
                 gyro_hardware_lpf: 0,
                 gyro_32khz_hardware_lpf: 0,
                 gyro_high_fsr: 0,
                 gyro_use_32khz: 0,
                 gyro_to_use: 0,
                 gyro_lowpass_hz: 0,
                 gyro_lowpass2_hz: 0,
                 gyro_soft_notch_hz_1: 0,
                 gyro_soft_notch_cutoff_1: 0,
                 gyro_soft_notch_hz_2: 0,
                 gyro_soft_notch_cutoff_2: 0,
                 gyro_offset_yaw: 0,
                 checkOverflow: 0,
                 gyro_lowpass_type: 0,
                 gyro_lowpass2_type: 0,
                 yaw_spin_recovery: 0,
                 yaw_spin_threshold: 0,
                 gyroCalibrationDuration: 0,
                 dyn_notch_quality: 0,
                 dyn_notch_width_percent: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut gyroConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (10i32 | 4i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<gyroConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &gyroConfig_System as *const gyroConfig_t as
                                     *mut gyroConfig_t as *mut uint8_t,
                             copy:
                                 &gyroConfig_Copy as *const gyroConfig_t as
                                     *mut gyroConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_3{ptr:
                                                     &pgResetTemplate_gyroConfig
                                                         as
                                                         *const gyroConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut gyroConfig_Copy: gyroConfig_t =
    gyroConfig_t{gyro_align: 0,
                 gyroMovementCalibrationThreshold: 0,
                 gyro_sync_denom: 0,
                 gyro_hardware_lpf: 0,
                 gyro_32khz_hardware_lpf: 0,
                 gyro_high_fsr: 0,
                 gyro_use_32khz: 0,
                 gyro_to_use: 0,
                 gyro_lowpass_hz: 0,
                 gyro_lowpass2_hz: 0,
                 gyro_soft_notch_hz_1: 0,
                 gyro_soft_notch_cutoff_1: 0,
                 gyro_soft_notch_hz_2: 0,
                 gyro_soft_notch_cutoff_2: 0,
                 gyro_offset_yaw: 0,
                 checkOverflow: 0,
                 gyro_lowpass_type: 0,
                 gyro_lowpass2_type: 0,
                 yaw_spin_recovery: 0,
                 yaw_spin_threshold: 0,
                 gyroCalibrationDuration: 0,
                 dyn_notch_quality: 0,
                 dyn_notch_width_percent: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_gyroConfig: gyroConfig_t =
    {
        let mut init =
            gyroConfig_s{gyro_align: ALIGN_DEFAULT as libc::c_int as uint8_t,
                         gyroMovementCalibrationThreshold: 48i32 as uint8_t,
                         gyro_sync_denom: 3i32 as uint8_t,
                         gyro_hardware_lpf: 0i32 as uint8_t,
                         gyro_32khz_hardware_lpf: 0i32 as uint8_t,
                         gyro_high_fsr: 0i32 as uint8_t,
                         gyro_use_32khz: 0i32 as uint8_t,
                         gyro_to_use: 0i32 as uint8_t,
                         gyro_lowpass_hz: 100i32 as uint16_t,
                         gyro_lowpass2_hz: 300i32 as uint16_t,
                         gyro_soft_notch_hz_1: 0i32 as uint16_t,
                         gyro_soft_notch_cutoff_1: 0i32 as uint16_t,
                         gyro_soft_notch_hz_2: 0i32 as uint16_t,
                         gyro_soft_notch_cutoff_2: 0i32 as uint16_t,
                         gyro_offset_yaw: 0i32 as int16_t,
                         checkOverflow:
                             GYRO_OVERFLOW_CHECK_ALL_AXES as libc::c_int as
                                 uint8_t,
                         gyro_lowpass_type:
                             FILTER_PT1 as libc::c_int as uint8_t,
                         gyro_lowpass2_type:
                             FILTER_PT1 as libc::c_int as uint8_t,
                         yaw_spin_recovery: 1i32 as uint8_t,
                         yaw_spin_threshold: 1950i32 as int16_t,
                         gyroCalibrationDuration: 125i32 as uint16_t,
                         dyn_notch_quality: 70i32 as uint8_t,
                         dyn_notch_width_percent: 50i32 as uint8_t,};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn gyroSensorBus() -> *const busDevice_t {
    return &mut gyroSensor1.gyroDev.bus;
}
#[no_mangle]
pub unsafe extern "C" fn gyroSensorBusByDevice(mut whichSensor: uint8_t)
 -> *const busDevice_t {
    return &mut gyroSensor1.gyroDev.bus;
}
// USE_GYRO_REGISTER_DUMP
#[no_mangle]
pub unsafe extern "C" fn gyroMpuConfiguration() -> *const mpuConfiguration_s {
    return &mut gyroSensor1.gyroDev.mpuConfiguration;
}
#[no_mangle]
pub unsafe extern "C" fn gyroMpuDetectionResult()
 -> *const mpuDetectionResult_s {
    return &mut gyroSensor1.gyroDev.mpuDetectionResult;
}
unsafe extern "C" fn gyroDetect(mut dev: *mut gyroDev_t) -> gyroSensor_e {
    let mut gyroHardware: gyroSensor_e = GYRO_DEFAULT;
    let mut current_block_5: u64;
    match gyroHardware as libc::c_uint {
        1 | 15 => {
            if fakeGyroDetect(dev) {
                gyroHardware = GYRO_FAKE;
                current_block_5 = 11650488183268122163;
            } else { current_block_5 = 2879091210037175961; }
        }
        _ => { current_block_5 = 2879091210037175961; }
    }
    match current_block_5 {
        2879091210037175961 => { gyroHardware = GYRO_NONE }
        _ => { }
    }
    if gyroHardware as libc::c_uint !=
           GYRO_NONE as libc::c_int as libc::c_uint {
        detectedSensors[SENSOR_INDEX_GYRO as libc::c_int as usize] =
            gyroHardware as uint8_t;
        sensorsSet(SENSOR_GYRO as libc::c_int as uint32_t);
    }
    return gyroHardware;
}
unsafe extern "C" fn gyroInitSensor(mut gyroSensor: *mut gyroSensor_t)
 -> bool {
    (*gyroSensor).gyroDev.gyro_high_fsr = (*gyroConfig()).gyro_high_fsr != 0;
    let gyroHardware: gyroSensor_e = gyroDetect(&mut (*gyroSensor).gyroDev);
    (*gyroSensor).gyroDev.gyroHardware = gyroHardware;
    if gyroHardware as libc::c_uint ==
           GYRO_NONE as libc::c_int as libc::c_uint {
        return 0i32 != 0
    }
    match gyroHardware as libc::c_uint {
        7 | 8 | 9 | 10 | 11 | 13 => { }
        _ => {
            // gyro does not support 32kHz
            (*gyroConfigMutable()).gyro_use_32khz = 0i32 as uint8_t
        }
    }
    // Must set gyro targetLooptime before gyroDev.init and initialisation of filters
    gyro.targetLooptime =
        gyroSetSampleRate(&mut (*gyroSensor).gyroDev,
                          (*gyroConfig()).gyro_hardware_lpf,
                          (*gyroConfig()).gyro_sync_denom,
                          (*gyroConfig()).gyro_use_32khz != 0);
    (*gyroSensor).gyroDev.hardware_lpf = (*gyroConfig()).gyro_hardware_lpf;
    (*gyroSensor).gyroDev.hardware_32khz_lpf =
        (*gyroConfig()).gyro_32khz_hardware_lpf;
    (*gyroSensor).gyroDev.initFn.expect("non-null function pointer")(&mut (*gyroSensor).gyroDev);
    if (*gyroConfig()).gyro_align as libc::c_int !=
           ALIGN_DEFAULT as libc::c_int {
        (*gyroSensor).gyroDev.gyroAlign =
            (*gyroConfig()).gyro_align as sensor_align_e
    }
    // As new gyros are supported, be sure to add them below based on whether they are subject to the overflow/inversion bug
    // Any gyro not explicitly defined will default to not having built-in overflow protection as a safe alternative.
    match gyroHardware as libc::c_uint {
        0 | 1 | 15 | 2 | 3 | 4 | 5 | 14 | 6 | 7 | 8 => {
            // Won't ever actually get here, but included to account for all gyro types
            (*gyroSensor).gyroDev.gyroHasOverflowProtection = 1i32 as uint8_t
        }
        9 | 10 | 11 | 12 | 13 => {
            // we don't actually know if this is affected, but as there are currently no flight controllers using it we err on the side of caution
            (*gyroSensor).gyroDev.gyroHasOverflowProtection = 0i32 as uint8_t
        }
        _ => {
            (*gyroSensor).gyroDev.gyroHasOverflowProtection = 0i32 as uint8_t
        }
    } // default catch for newly added gyros until proven to be unaffected
    gyroInitSensorFilters(gyroSensor);
    return 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn gyroInit() -> bool {
    if (*gyroConfig()).checkOverflow as libc::c_int ==
           GYRO_OVERFLOW_CHECK_YAW as libc::c_int {
        overflowAxisMask = GYRO_OVERFLOW_Z as libc::c_int as uint8_t
    } else if (*gyroConfig()).checkOverflow as libc::c_int ==
                  GYRO_OVERFLOW_CHECK_ALL_AXES as libc::c_int {
        overflowAxisMask =
            (GYRO_OVERFLOW_X as libc::c_int | GYRO_OVERFLOW_Y as libc::c_int |
                 GYRO_OVERFLOW_Z as libc::c_int) as uint8_t
    } else { overflowAxisMask = 0i32 as uint8_t }
    match debugMode as libc::c_int {
        15 | 17 | 19 | 6 | 3 => { gyroDebugMode = debugMode }
        _ => {
            // debugMode is not gyro-related
            gyroDebugMode = DEBUG_NONE as libc::c_int as uint8_t
        }
    }
    firstArmingCalibrationWasStarted = 0i32 != 0;
    let mut ret: bool = 0i32 != 0;
    gyroToUse = (*gyroConfig()).gyro_to_use;
    gyroSensor1.gyroDev.gyroAlign = ALIGN_DEFAULT;
    gyroSensor1.gyroDev.mpuIntExtiTag = 0i32 as ioTag_t;
    // GYRO_1_EXTI_PIN
    // USE_DUAL_GYRO
    ret = gyroInitSensor(&mut gyroSensor1);
    gyroHasOverflowProtection =
        gyroHasOverflowProtection as libc::c_int != 0 &&
            gyroSensor1.gyroDev.gyroHasOverflowProtection as libc::c_int != 0;
    // USE_DUAL_GYRO
    // USE_DUAL_GYRO
    // USE_DUAL_GYRO
    return ret;
}
unsafe extern "C" fn gyroInitLowpassFilterLpf(mut gyroSensor:
                                                  *mut gyroSensor_t,
                                              mut slot: libc::c_int,
                                              mut type_0: libc::c_int,
                                              mut lpfHz: uint16_t) {
    let mut lowpassFilterApplyFn: *mut filterApplyFnPtr =
        0 as *mut filterApplyFnPtr;
    let mut lowpassFilter: *mut gyroLowpassFilter_t =
        0 as *mut gyroLowpassFilter_t;
    match slot {
        0 => {
            lowpassFilterApplyFn = &mut (*gyroSensor).lowpassFilterApplyFn;
            lowpassFilter = (*gyroSensor).lowpassFilter.as_mut_ptr()
        }
        1 => {
            lowpassFilterApplyFn = &mut (*gyroSensor).lowpass2FilterApplyFn;
            lowpassFilter = (*gyroSensor).lowpass2Filter.as_mut_ptr()
        }
        _ => { return }
    }
    // Establish some common constants
    let gyroFrequencyNyquist: uint32_t =
        ((1000000i32 / 2i32) as
             libc::c_uint).wrapping_div(gyro.targetLooptime);
    let gyroDt: libc::c_float =
        gyro.targetLooptime as libc::c_float * 1e-6f32;
    // Gain could be calculated a little later as it is specific to the pt1/bqrcf2/fkf branches
    let gain: libc::c_float = pt1FilterGain(lpfHz, gyroDt);
    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn =
        Some(nullFilterApply as
                 unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                     -> libc::c_float);
    // If lowpass cutoff has been specified and is less than the Nyquist frequency
    if lpfHz as libc::c_int != 0 &&
           lpfHz as libc::c_uint <= gyroFrequencyNyquist {
        match type_0 {
            0 => {
                *lowpassFilterApplyFn =
                    ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                             *mut pt1Filter_t,
                                                                         _:
                                                                             libc::c_float)
                                                        -> libc::c_float>,
                                             filterApplyFnPtr>(Some(pt1FilterApply
                                                                        as
                                                                        unsafe extern "C" fn(_:
                                                                                                 *mut pt1Filter_t,
                                                                                             _:
                                                                                                 libc::c_float)
                                                                            ->
                                                                                libc::c_float));
                let mut axis: libc::c_int = 0i32;
                while axis < 3i32 {
                    pt1FilterInit(&mut (*lowpassFilter.offset(axis as
                                                                  isize)).pt1FilterState,
                                  gain);
                    axis += 1
                }
            }
            1 => {
                *lowpassFilterApplyFn =
                    ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                             *mut biquadFilter_t,
                                                                         _:
                                                                             libc::c_float)
                                                        -> libc::c_float>,
                                             filterApplyFnPtr>(Some(biquadFilterApply
                                                                        as
                                                                        unsafe extern "C" fn(_:
                                                                                                 *mut biquadFilter_t,
                                                                                             _:
                                                                                                 libc::c_float)
                                                                            ->
                                                                                libc::c_float));
                let mut axis_0: libc::c_int = 0i32;
                while axis_0 < 3i32 {
                    biquadFilterInitLPF(&mut (*lowpassFilter.offset(axis_0 as
                                                                        isize)).biquadFilterState,
                                        lpfHz as libc::c_float,
                                        gyro.targetLooptime);
                    axis_0 += 1
                }
            }
            _ => { }
        }
    };
}
unsafe extern "C" fn calculateNyquistAdjustedNotchHz(mut notchHz: uint16_t,
                                                     mut notchCutoffHz:
                                                         uint16_t)
 -> uint16_t {
    let gyroFrequencyNyquist: uint32_t =
        ((1000000i32 / 2i32) as
             libc::c_uint).wrapping_div(gyro.targetLooptime);
    if notchHz as libc::c_uint > gyroFrequencyNyquist {
        if (notchCutoffHz as libc::c_uint) < gyroFrequencyNyquist {
            notchHz = gyroFrequencyNyquist as uint16_t
        } else { notchHz = 0i32 as uint16_t }
    }
    return notchHz;
}
unsafe extern "C" fn gyroInitFilterNotch1(mut gyroSensor: *mut gyroSensor_t,
                                          mut notchHz: uint16_t,
                                          mut notchCutoffHz: uint16_t) {
    (*gyroSensor).notchFilter1ApplyFn =
        Some(nullFilterApply as
                 unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                     -> libc::c_float);
    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);
    if notchHz as libc::c_int != 0i32 && notchCutoffHz as libc::c_int != 0i32
       {
        (*gyroSensor).notchFilter1ApplyFn =
            ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                     *mut biquadFilter_t,
                                                                 _:
                                                                     libc::c_float)
                                                -> libc::c_float>,
                                     filterApplyFnPtr>(Some(biquadFilterApply
                                                                as
                                                                unsafe extern "C" fn(_:
                                                                                         *mut biquadFilter_t,
                                                                                     _:
                                                                                         libc::c_float)
                                                                    ->
                                                                        libc::c_float));
        let notchQ: libc::c_float =
            filterGetNotchQ(notchHz as libc::c_float,
                            notchCutoffHz as libc::c_float);
        let mut axis: libc::c_int = 0i32;
        while axis < 3i32 {
            biquadFilterInit(&mut *(*gyroSensor).notchFilter1.as_mut_ptr().offset(axis
                                                                                      as
                                                                                      isize),
                             notchHz as libc::c_float, gyro.targetLooptime,
                             notchQ, FILTER_NOTCH);
            axis += 1
        }
    };
}
unsafe extern "C" fn gyroInitFilterNotch2(mut gyroSensor: *mut gyroSensor_t,
                                          mut notchHz: uint16_t,
                                          mut notchCutoffHz: uint16_t) {
    (*gyroSensor).notchFilter2ApplyFn =
        Some(nullFilterApply as
                 unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                     -> libc::c_float);
    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);
    if notchHz as libc::c_int != 0i32 && notchCutoffHz as libc::c_int != 0i32
       {
        (*gyroSensor).notchFilter2ApplyFn =
            ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                     *mut biquadFilter_t,
                                                                 _:
                                                                     libc::c_float)
                                                -> libc::c_float>,
                                     filterApplyFnPtr>(Some(biquadFilterApply
                                                                as
                                                                unsafe extern "C" fn(_:
                                                                                         *mut biquadFilter_t,
                                                                                     _:
                                                                                         libc::c_float)
                                                                    ->
                                                                        libc::c_float));
        let notchQ: libc::c_float =
            filterGetNotchQ(notchHz as libc::c_float,
                            notchCutoffHz as libc::c_float);
        let mut axis: libc::c_int = 0i32;
        while axis < 3i32 {
            biquadFilterInit(&mut *(*gyroSensor).notchFilter2.as_mut_ptr().offset(axis
                                                                                      as
                                                                                      isize),
                             notchHz as libc::c_float, gyro.targetLooptime,
                             notchQ, FILTER_NOTCH);
            axis += 1
        }
    };
}
unsafe extern "C" fn gyroInitSensorFilters(mut gyroSensor:
                                               *mut gyroSensor_t) {
    gyroInitLowpassFilterLpf(gyroSensor, FILTER_LOWPASS as libc::c_int,
                             (*gyroConfig()).gyro_lowpass_type as libc::c_int,
                             (*gyroConfig()).gyro_lowpass_hz);
    gyroInitLowpassFilterLpf(gyroSensor, FILTER_LOWPASS2 as libc::c_int,
                             (*gyroConfig()).gyro_lowpass2_type as
                                 libc::c_int,
                             (*gyroConfig()).gyro_lowpass2_hz);
    gyroInitFilterNotch1(gyroSensor, (*gyroConfig()).gyro_soft_notch_hz_1,
                         (*gyroConfig()).gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(gyroSensor, (*gyroConfig()).gyro_soft_notch_hz_2,
                         (*gyroConfig()).gyro_soft_notch_cutoff_2);
}
#[no_mangle]
pub unsafe extern "C" fn gyroInitFilters() {
    gyroInitSensorFilters(&mut gyroSensor1);
}
#[no_mangle]
pub unsafe extern "C" fn isGyroSensorCalibrationComplete(mut gyroSensor:
                                                             *const gyroSensor_t)
 -> bool {
    return (*gyroSensor).calibration.cyclesRemaining == 0i32;
}
#[no_mangle]
pub unsafe extern "C" fn isGyroCalibrationComplete() -> bool {
    return isGyroSensorCalibrationComplete(&mut gyroSensor1);
}
unsafe extern "C" fn isOnFinalGyroCalibrationCycle(mut gyroCalibration:
                                                       *const gyroCalibration_t)
 -> bool {
    return (*gyroCalibration).cyclesRemaining == 1i32;
}
unsafe extern "C" fn gyroCalculateCalibratingCycles() -> int32_t {
    return (((*gyroConfig()).gyroCalibrationDuration as libc::c_int *
                 10000i32) as libc::c_uint).wrapping_div(gyro.targetLooptime)
               as int32_t;
}
unsafe extern "C" fn isOnFirstGyroCalibrationCycle(mut gyroCalibration:
                                                       *const gyroCalibration_t)
 -> bool {
    return (*gyroCalibration).cyclesRemaining ==
               gyroCalculateCalibratingCycles();
}
unsafe extern "C" fn gyroSetCalibrationCycles(mut gyroSensor:
                                                  *mut gyroSensor_t) {
    (*gyroSensor).calibration.cyclesRemaining =
        gyroCalculateCalibratingCycles();
}
#[no_mangle]
pub unsafe extern "C" fn gyroStartCalibration(mut isFirstArmingCalibration:
                                                  bool) {
    if !(isFirstArmingCalibration as libc::c_int != 0 &&
             firstArmingCalibrationWasStarted as libc::c_int != 0) {
        gyroSetCalibrationCycles(&mut gyroSensor1);
        if isFirstArmingCalibration {
            firstArmingCalibrationWasStarted = 1i32 != 0
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn isFirstArmingGyroCalibrationRunning() -> bool {
    return firstArmingCalibrationWasStarted as libc::c_int != 0 &&
               !isGyroCalibrationComplete();
}
unsafe extern "C" fn performGyroCalibration(mut gyroSensor: *mut gyroSensor_t,
                                            mut gyroMovementCalibrationThreshold:
                                                uint8_t) {
    let mut axis: libc::c_int = 0i32;
    while axis < 3i32 {
        // Reset g[axis] at start of calibration
        if isOnFirstGyroCalibrationCycle(&mut (*gyroSensor).calibration) {
            (*gyroSensor).calibration.sum[axis as usize] = 0.0f32;
            devClear(&mut *(*gyroSensor).calibration.var.as_mut_ptr().offset(axis
                                                                                 as
                                                                                 isize));
            // gyroZero is set to zero until calibration complete
            (*gyroSensor).gyroDev.gyroZero[axis as usize] = 0.0f32
        }
        // Sum up CALIBRATING_GYRO_TIME_US readings
        (*gyroSensor).calibration.sum[axis as usize] +=
            (*gyroSensor).gyroDev.gyroADCRaw[axis as usize] as libc::c_int as
                libc::c_float;
        devPush(&mut *(*gyroSensor).calibration.var.as_mut_ptr().offset(axis
                                                                            as
                                                                            isize),
                (*gyroSensor).gyroDev.gyroADCRaw[axis as usize] as
                    libc::c_float);
        if isOnFinalGyroCalibrationCycle(&mut (*gyroSensor).calibration) {
            let stddev: libc::c_float =
                devStandardDeviation(&mut *(*gyroSensor).calibration.var.as_mut_ptr().offset(axis
                                                                                                 as
                                                                                                 isize));
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            if axis == X as libc::c_int {
                if debugMode as libc::c_int == DEBUG_GYRO_RAW as libc::c_int {
                    debug[3] = lrintf(stddev) as int16_t
                }
            }
            // check deviation and startover in case the model was moved
            if gyroMovementCalibrationThreshold as libc::c_int != 0 &&
                   stddev >
                       gyroMovementCalibrationThreshold as libc::c_int as
                           libc::c_float {
                gyroSetCalibrationCycles(gyroSensor);
                return
            }
            // please take care with exotic boardalignment !!
            (*gyroSensor).gyroDev.gyroZero[axis as usize] =
                (*gyroSensor).calibration.sum[axis as usize] /
                    gyroCalculateCalibratingCycles() as
                        libc::c_float; // so calibration cycles do not pollute tasks statistics
            if axis == Z as libc::c_int {
                (*gyroSensor).gyroDev.gyroZero[axis as usize] -=
                    (*gyroConfig()).gyro_offset_yaw as libc::c_float /
                        100i32 as libc::c_float
            }
        }
        axis += 1
    }
    if isOnFinalGyroCalibrationCycle(&mut (*gyroSensor).calibration) {
        schedulerResetTaskStatistics(TASK_SELF);
        if !firstArmingCalibrationWasStarted ||
               getArmingDisableFlags() as libc::c_uint &
                   !(ARMING_DISABLED_CALIBRATING as libc::c_int) as
                       libc::c_uint == 0i32 as libc::c_uint {
            beeper(BEEPER_GYRO_CALIBRATED);
        }
    }
    (*gyroSensor).calibration.cyclesRemaining -= 1;
}
unsafe extern "C" fn handleOverflow(mut gyroSensor: *mut gyroSensor_t,
                                    mut currentTimeUs: timeUs_t) {
    let gyroOverflowResetRate: libc::c_float =
        30340i32 as libc::c_float * (*gyroSensor).gyroDev.scale;
    if (abs(gyro.gyroADCf[X as libc::c_int as usize] as libc::c_int) as
            libc::c_float) < gyroOverflowResetRate &&
           (abs(gyro.gyroADCf[Y as libc::c_int as usize] as libc::c_int) as
                libc::c_float) < gyroOverflowResetRate &&
           (abs(gyro.gyroADCf[Z as libc::c_int as usize] as libc::c_int) as
                libc::c_float) < gyroOverflowResetRate {
        // if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
        // reset requires good OK values on all axes
        if cmpTimeUs(currentTimeUs, (*gyroSensor).overflowTimeUs) > 50000i32 {
            (*gyroSensor).overflowDetected = 0i32 != 0
        }
    } else {
        // not a consecutive OK value, so reset the overflow time
        (*gyroSensor).overflowTimeUs = currentTimeUs
    };
}
unsafe extern "C" fn checkForOverflow(mut gyroSensor: *mut gyroSensor_t,
                                      mut currentTimeUs: timeUs_t) {
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (*gyroSensor).overflowDetected {
        handleOverflow(gyroSensor, currentTimeUs);
    };
}
// USE_GYRO_OVERFLOW_CHECK
unsafe extern "C" fn handleYawSpin(mut gyroSensor: *mut gyroSensor_t,
                                   mut currentTimeUs: timeUs_t) {
    let yawSpinResetRate: libc::c_float =
        (*gyroConfig()).yaw_spin_threshold as libc::c_int as libc::c_float -
            100.0f32;
    if (abs(gyro.gyroADCf[Z as libc::c_int as usize] as libc::c_int) as
            libc::c_float) < yawSpinResetRate {
        // testing whether 20ms of consecutive OK gyro yaw values is enough
        if cmpTimeUs(currentTimeUs, (*gyroSensor).yawSpinTimeUs) > 20000i32 {
            (*gyroSensor).yawSpinDetected = 0i32 != 0
        }
    } else {
        // reset the yaw spin time
        (*gyroSensor).yawSpinTimeUs = currentTimeUs
    };
}
unsafe extern "C" fn checkForYawSpin(mut gyroSensor: *mut gyroSensor_t,
                                     mut currentTimeUs: timeUs_t) {
    // if not in overflow mode, handle yaw spins above threshold
    if (*gyroSensor).overflowDetected {
        (*gyroSensor).yawSpinDetected = 0i32 != 0;
        return
    }
    // USE_GYRO_OVERFLOW_CHECK
    if (*gyroSensor).yawSpinDetected {
        handleYawSpin(gyroSensor, currentTimeUs);
    };
}
unsafe extern "C" fn filterGyro(mut gyroSensor: *mut gyroSensor_t,
                                mut sampleDeltaUs: timeDelta_t) {
    let mut axis: libc::c_int = 0i32;
    while axis < 3i32 {
        // scale gyro output to degrees per second
        // scale gyro output to degrees per second
        let mut gyroADCf: libc::c_float =
            (*gyroSensor).gyroDev.gyroADC[axis as usize] *
                (*gyroSensor).gyroDev.scale;
        // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
        // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
        // apply static notch filters and software lowpass filters
        // apply static notch filters and software lowpass filters
        gyroADCf =
            (*gyroSensor).notchFilter1ApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilter1.as_mut_ptr().offset(axis
                                                                                                                                           as
                                                                                                                                           isize)
                                                                                      as
                                                                                      *mut biquadFilter_t
                                                                                      as
                                                                                      *mut filter_t,
                                                                                  gyroADCf);
        gyroADCf =
            (*gyroSensor).notchFilter2ApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilter2.as_mut_ptr().offset(axis
                                                                                                                                           as
                                                                                                                                           isize)
                                                                                      as
                                                                                      *mut biquadFilter_t
                                                                                      as
                                                                                      *mut filter_t,
                                                                                  gyroADCf);
        gyroADCf =
            (*gyroSensor).lowpassFilterApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).lowpassFilter.as_mut_ptr().offset(axis
                                                                                                                                             as
                                                                                                                                             isize)
                                                                                       as
                                                                                       *mut gyroLowpassFilter_t
                                                                                       as
                                                                                       *mut filter_t,
                                                                                   gyroADCf);
        gyroADCf =
            (*gyroSensor).lowpass2FilterApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).lowpass2Filter.as_mut_ptr().offset(axis
                                                                                                                                               as
                                                                                                                                               isize)
                                                                                        as
                                                                                        *mut gyroLowpassFilter_t
                                                                                        as
                                                                                        *mut filter_t,
                                                                                    gyroADCf);
        (*gyroSensor).gyroDev.gyroADCf[axis as usize] = gyroADCf;
        if !(*gyroSensor).overflowDetected {
            // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
            // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
            // integrate using trapezium rule to avoid bias
            // integrate using trapezium rule to avoid bias
            accumulatedMeasurements[axis as usize] +=
                0.5f32 * (gyroPrevious[axis as usize] + gyroADCf) *
                    sampleDeltaUs as libc::c_float;
            gyroPrevious[axis as usize] = gyroADCf
        }
        axis += 1
    };
}
unsafe extern "C" fn filterGyroDebug(mut gyroSensor: *mut gyroSensor_t,
                                     mut sampleDeltaUs: timeDelta_t) {
    let mut axis: libc::c_int = 0i32;
    while axis < 3i32 {
        if debugMode as libc::c_int == DEBUG_GYRO_RAW as libc::c_int {
            debug[axis as usize] =
                (*gyroSensor).gyroDev.gyroADCRaw[axis as usize]
        }
        let mut gyroADCf: libc::c_float =
            (*gyroSensor).gyroDev.gyroADC[axis as usize] *
                (*gyroSensor).gyroDev.scale;
        if debugMode as libc::c_int == DEBUG_GYRO_SCALED as libc::c_int {
            debug[axis as usize] = lrintf(gyroADCf) as int16_t
        }
        gyroADCf =
            (*gyroSensor).notchFilter1ApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilter1.as_mut_ptr().offset(axis
                                                                                                                                           as
                                                                                                                                           isize)
                                                                                      as
                                                                                      *mut biquadFilter_t
                                                                                      as
                                                                                      *mut filter_t,
                                                                                  gyroADCf);
        gyroADCf =
            (*gyroSensor).notchFilter2ApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).notchFilter2.as_mut_ptr().offset(axis
                                                                                                                                           as
                                                                                                                                           isize)
                                                                                      as
                                                                                      *mut biquadFilter_t
                                                                                      as
                                                                                      *mut filter_t,
                                                                                  gyroADCf);
        gyroADCf =
            (*gyroSensor).lowpassFilterApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).lowpassFilter.as_mut_ptr().offset(axis
                                                                                                                                             as
                                                                                                                                             isize)
                                                                                       as
                                                                                       *mut gyroLowpassFilter_t
                                                                                       as
                                                                                       *mut filter_t,
                                                                                   gyroADCf);
        gyroADCf =
            (*gyroSensor).lowpass2FilterApplyFn.expect("non-null function pointer")(&mut *(*gyroSensor).lowpass2Filter.as_mut_ptr().offset(axis
                                                                                                                                               as
                                                                                                                                               isize)
                                                                                        as
                                                                                        *mut gyroLowpassFilter_t
                                                                                        as
                                                                                        *mut filter_t,
                                                                                    gyroADCf);
        if debugMode as libc::c_int == DEBUG_GYRO_FILTERED as libc::c_int {
            debug[axis as usize] = lrintf(gyroADCf) as int16_t
        }
        (*gyroSensor).gyroDev.gyroADCf[axis as usize] = gyroADCf;
        if !(*gyroSensor).overflowDetected {
            accumulatedMeasurements[axis as usize] +=
                0.5f32 * (gyroPrevious[axis as usize] + gyroADCf) *
                    sampleDeltaUs as libc::c_float;
            gyroPrevious[axis as usize] = gyroADCf
        }
        axis += 1
    };
}
// USE_YAW_SPIN_RECOVERY
unsafe extern "C" fn gyroUpdateSensor(mut gyroSensor: *mut gyroSensor_t,
                                      mut currentTimeUs: timeUs_t) {
    if !(*gyroSensor).gyroDev.readFn.expect("non-null function pointer")(&mut (*gyroSensor).gyroDev)
       {
        return
    }
    (*gyroSensor).gyroDev.dataReady = 0i32 != 0;
    if isGyroSensorCalibrationComplete(gyroSensor) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations
        (*gyroSensor).gyroDev.gyroADC[X as libc::c_int as usize] =
            (*gyroSensor).gyroDev.gyroADCRaw[X as libc::c_int as usize] as
                libc::c_int as libc::c_float -
                (*gyroSensor).gyroDev.gyroZero[X as libc::c_int as usize];
        (*gyroSensor).gyroDev.gyroADC[Y as libc::c_int as usize] =
            (*gyroSensor).gyroDev.gyroADCRaw[Y as libc::c_int as usize] as
                libc::c_int as libc::c_float -
                (*gyroSensor).gyroDev.gyroZero[Y as libc::c_int as usize];
        (*gyroSensor).gyroDev.gyroADC[Z as libc::c_int as usize] =
            (*gyroSensor).gyroDev.gyroADCRaw[Z as libc::c_int as usize] as
                libc::c_int as libc::c_float -
                (*gyroSensor).gyroDev.gyroZero[Z as libc::c_int as usize];
        alignSensors((*gyroSensor).gyroDev.gyroADC.as_mut_ptr(),
                     (*gyroSensor).gyroDev.gyroAlign as uint8_t);
    } else {
        performGyroCalibration(gyroSensor,
                               (*gyroConfig()).gyroMovementCalibrationThreshold);
        // still calibrating, so no need to further process gyro data
        return
    }
    let sampleDeltaUs: timeDelta_t =
        currentTimeUs.wrapping_sub(accumulationLastTimeSampledUs) as
            timeDelta_t;
    accumulationLastTimeSampledUs = currentTimeUs;
    accumulatedMeasurementTimeUs =
        (accumulatedMeasurementTimeUs as
             libc::c_uint).wrapping_add(sampleDeltaUs as libc::c_uint) as
            timeUs_t as timeUs_t;
    if (*gyroConfig()).checkOverflow as libc::c_int != 0 &&
           !gyroHasOverflowProtection {
        checkForOverflow(gyroSensor, currentTimeUs);
    }
    if (*gyroConfig()).yaw_spin_recovery != 0 {
        checkForYawSpin(gyroSensor, currentTimeUs);
    }
    if gyroDebugMode as libc::c_int == DEBUG_NONE as libc::c_int {
        filterGyro(gyroSensor, sampleDeltaUs);
    } else { filterGyroDebug(gyroSensor, sampleDeltaUs); };
}
#[no_mangle]
pub unsafe extern "C" fn gyroUpdate(mut currentTimeUs: timeUs_t) {
    gyroUpdateSensor(&mut gyroSensor1, currentTimeUs);
    gyro.gyroADCf[X as libc::c_int as usize] =
        gyroSensor1.gyroDev.gyroADCf[X as libc::c_int as usize];
    gyro.gyroADCf[Y as libc::c_int as usize] =
        gyroSensor1.gyroDev.gyroADCf[Y as libc::c_int as usize];
    gyro.gyroADCf[Z as libc::c_int as usize] =
        gyroSensor1.gyroDev.gyroADCf[Z as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn gyroGetAccumulationAverage(mut accumulationAverage:
                                                        *mut libc::c_float)
 -> bool {
    if accumulatedMeasurementTimeUs > 0i32 as libc::c_uint {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        let mut axis: libc::c_int = 0i32;
        while axis < 3i32 {
            *accumulationAverage.offset(axis as isize) =
                accumulatedMeasurements[axis as usize] /
                    accumulatedMeasurementTimeUs as libc::c_float;
            accumulatedMeasurements[axis as usize] = 0.0f32;
            axis += 1
        }
        accumulatedMeasurementTimeUs = 0i32 as timeUs_t;
        return 1i32 != 0
    } else {
        let mut axis_0: libc::c_int = 0i32;
        while axis_0 < 3i32 {
            *accumulationAverage.offset(axis_0 as isize) = 0.0f32;
            axis_0 += 1
        }
        return 0i32 != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn gyroReadTemperature() {
    if gyroSensor1.gyroDev.temperatureFn.is_some() {
        gyroSensor1.gyroDev.temperatureFn.expect("non-null function pointer")(&mut gyroSensor1.gyroDev,
                                                                              &mut gyroSensor1.gyroDev.temperature);
    };
}
#[no_mangle]
pub unsafe extern "C" fn gyroGetTemperature() -> int16_t {
    return gyroSensor1.gyroDev.temperature;
}
#[no_mangle]
pub unsafe extern "C" fn gyroRateDps(mut axis: libc::c_int) -> int16_t {
    return lrintf(gyro.gyroADCf[axis as usize] / gyroSensor1.gyroDev.scale) as
               int16_t;
}
#[no_mangle]
pub unsafe extern "C" fn gyroOverflowDetected() -> bool {
    return gyroSensor1.overflowDetected;
}
#[no_mangle]
pub unsafe extern "C" fn gyroYawSpinDetected() -> bool {
    return gyroSensor1.yawSpinDetected;
}
// USE_YAW_SPIN_RECOVERY
#[no_mangle]
pub unsafe extern "C" fn gyroAbsRateDps(mut axis: libc::c_int) -> uint16_t {
    return fabsf(gyro.gyroADCf[axis as usize]) as uint16_t;
}
// gyro alignment
// people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
// Gyro sample divider
// gyro DLPF setting
// gyro 32khz DLPF setting
// Lowpass primary/secondary
// Gyro calibration duration in 1/100 second
// bandpass quality factor, 100 for steep sided bandpass
#[no_mangle]
pub unsafe extern "C" fn gyroReadRegister(mut whichSensor: uint8_t,
                                          mut reg: uint8_t) -> uint8_t {
    return mpuGyroReadRegister(gyroSensorBusByDevice(whichSensor), reg);
}
// USE_GYRO_REGISTER_DUMP
