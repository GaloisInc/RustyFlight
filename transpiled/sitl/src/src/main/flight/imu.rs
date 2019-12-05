use core;
use libc;
extern "C" {
    #[no_mangle]
    fn sqrtf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn fabsf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn degreesToRadians(degrees: int16_t) -> libc::c_float;
    #[no_mangle]
    fn atan2_approx(y: libc::c_float, x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn acos_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn sin_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn cos_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn mixerSetThrottleAngleCorrection(correctionValue: libc::c_int);
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    fn pthread_mutex_init(__mutex: *mut pthread_mutex_t,
                          __mutexattr: *const pthread_mutexattr_t)
     -> libc::c_int;
    #[no_mangle]
    fn pthread_mutex_lock(__mutex: *mut pthread_mutex_t) -> libc::c_int;
    #[no_mangle]
    fn pthread_mutex_unlock(__mutex: *mut pthread_mutex_t) -> libc::c_int;
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    fn compassIsHealthy() -> bool;
    #[no_mangle]
    fn printf(_: *const libc::c_char, _: ...) -> libc::c_int;
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
pub struct SPI_TypeDef {
    pub test: *mut libc::c_void,
}
pub type C2RustUnnamed = libc::c_uint;
pub const Z: C2RustUnnamed = 2;
pub const Y: C2RustUnnamed = 1;
pub const X: C2RustUnnamed = 0;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_0 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_0 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_0 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_0 = 4095;
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
    pub reset: C2RustUnnamed_1,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_1 {
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
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_2 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_2 = 2;
pub const ARMED: C2RustUnnamed_2 = 1;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_3 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_3 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_3 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_3 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_3 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_3 = 16;
pub const BARO_MODE: C2RustUnnamed_3 = 8;
pub const MAG_MODE: C2RustUnnamed_3 = 4;
pub const HORIZON_MODE: C2RustUnnamed_3 = 2;
pub const ANGLE_MODE: C2RustUnnamed_3 = 1;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_4 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_4 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_4 = 4;
pub const GPS_FIX: C2RustUnnamed_4 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_4 = 1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct fp_vector {
    pub X: libc::c_float,
    pub Y: libc::c_float,
    pub Z: libc::c_float,
}
// Floating point 3 vector.
pub type t_fp_vector_def = fp_vector;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct quaternion {
    pub w: libc::c_float,
    pub x: libc::c_float,
    pub y: libc::c_float,
    pub z: libc::c_float,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct quaternionProducts {
    pub ww: libc::c_float,
    pub wx: libc::c_float,
    pub wy: libc::c_float,
    pub wz: libc::c_float,
    pub xx: libc::c_float,
    pub xy: libc::c_float,
    pub xz: libc::c_float,
    pub yy: libc::c_float,
    pub yz: libc::c_float,
    pub zz: libc::c_float,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_5,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct C2RustUnnamed_5 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct accDeadband_s {
    pub xy: uint8_t,
    pub z: uint8_t,
}
pub type accDeadband_t = accDeadband_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct imuConfig_s {
    pub dcm_kp: uint16_t,
    pub dcm_ki: uint16_t,
    pub small_angle: uint8_t,
    pub acc_unarmedcal: uint8_t,
    pub accDeadband: accDeadband_t,
}
pub type imuConfig_t = imuConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct imuRuntimeConfig_s {
    pub dcm_ki: libc::c_float,
    pub dcm_kp: libc::c_float,
    pub acc_unarmedcal: uint8_t,
    pub small_angle: uint8_t,
    pub accDeadband: accDeadband_t,
}
pub type imuRuntimeConfig_t = imuRuntimeConfig_s;
pub type acc_t = acc_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub type accDev_t = accDev_s;
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
// initialize function
// read 3 axis data function
// read temperature if available
// scalefactor
// gyro data after calibration and alignment
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct accDev_s {
    pub lock: pthread_mutex_t,
    pub initFn: sensorAccInitFuncPtr,
    pub readFn: sensorAccReadFuncPtr,
    pub bus: busDevice_t,
    pub acc_1G: uint16_t,
    pub ADCRaw: [int16_t; 3],
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub accAlign: sensor_align_e,
    pub dataReady: bool,
    pub acc_high_fsr: bool,
    pub revisionCode: libc::c_char,
    pub filler: [uint8_t; 2],
}
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
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
pub type busDevice_t = busDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_6,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_6 {
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
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
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union pthread_mutex_t {
    pub __data: __pthread_mutex_s,
    pub __size: [libc::c_char; 40],
    pub __align: libc::c_long,
}
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
pub type __pthread_list_t = __pthread_internal_list;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __pthread_internal_list {
    pub __prev: *mut __pthread_internal_list,
    pub __next: *mut __pthread_internal_list,
}
pub type gpsSolutionData_t = gpsSolutionData_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gpsSolutionData_s {
    pub llh: gpsLocation_t,
    pub groundSpeed: uint16_t,
    pub groundCourse: uint16_t,
    pub hdop: uint16_t,
    pub numSat: uint8_t,
}
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
/* LLH Location in NEU axis system */
pub type gpsLocation_t = gpsLocation_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
pub const SENSOR_GPS: C2RustUnnamed_7 = 32;
pub const SENSOR_MAG: C2RustUnnamed_7 = 8;
pub const SENSOR_ACC: C2RustUnnamed_7 = 2;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union pthread_mutexattr_t {
    pub __size: [libc::c_char; 4],
    pub __align: libc::c_int,
}
pub type C2RustUnnamed_7 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_7 = 64;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_7 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_7 = 16;
pub const SENSOR_BARO: C2RustUnnamed_7 = 4;
pub const SENSOR_GYRO: C2RustUnnamed_7 = 1;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[inline]
unsafe extern "C" fn imuConfig() -> *const imuConfig_t {
    return &mut imuConfig_System;
}
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
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
// Inertial Measurement Unit (IMU)
static mut imuUpdateLock: pthread_mutex_t =
    pthread_mutex_t{__data:
                        __pthread_mutex_s{__lock: 0,
                                          __count: 0,
                                          __owner: 0,
                                          __nusers: 0,
                                          __kind: 0,
                                          __spins: 0,
                                          __elision: 0,
                                          __list:
                                              __pthread_list_t{__prev:
                                                                   0 as
                                                                       *const __pthread_internal_list
                                                                       as
                                                                       *mut __pthread_internal_list,
                                                               __next:
                                                                   0 as
                                                                       *const __pthread_internal_list
                                                                       as
                                                                       *mut __pthread_internal_list,},},};
// 500cm/s minimum groundspeed for a gps heading to be considered valid
#[no_mangle]
pub static mut accSum: [int32_t; 3] = [0; 3];
#[no_mangle]
pub static mut accAverage: [libc::c_float; 3] = [0.; 3];
#[no_mangle]
pub static mut accTimeSum: uint32_t = 0i32 as uint32_t;
// keep track for integration of acc
#[no_mangle]
pub static mut accSumCount: libc::c_int = 0i32;
#[no_mangle]
pub static mut accVelScale: libc::c_float = 0.;
#[no_mangle]
pub static mut canUseGPSHeading: bool = 1i32 != 0;
static mut throttleAngleScale: libc::c_float = 0.;
static mut throttleAngleValue: libc::c_int = 0;
static mut fc_acc: libc::c_float = 0.;
static mut smallAngleCosZ: libc::c_float = 0i32 as libc::c_float;
static mut imuRuntimeConfig: imuRuntimeConfig_t =
    imuRuntimeConfig_t{dcm_ki: 0.,
                       dcm_kp: 0.,
                       acc_unarmedcal: 0,
                       small_angle: 0,
                       accDeadband: accDeadband_t{xy: 0, z: 0,},};
static mut rMat: [[libc::c_float; 3]; 3] = [[0.; 3]; 3];
// quaternion of sensor frame relative to earth frame
static mut q: quaternion =
    {
        let mut init =
            quaternion{w: 1i32 as libc::c_float,
                       x: 0i32 as libc::c_float,
                       y: 0i32 as libc::c_float,
                       z: 0i32 as libc::c_float,};
        init
    };
static mut qP: quaternionProducts =
    {
        let mut init =
            quaternionProducts{ww: 1i32 as libc::c_float,
                               wx: 0i32 as libc::c_float,
                               wy: 0i32 as libc::c_float,
                               wz: 0i32 as libc::c_float,
                               xx: 0i32 as libc::c_float,
                               xy: 0i32 as libc::c_float,
                               xz: 0i32 as libc::c_float,
                               yy: 0i32 as libc::c_float,
                               yz: 0i32 as libc::c_float,
                               zz: 0i32 as libc::c_float,};
        init
    };
// headfree quaternions
#[no_mangle]
pub static mut headfree: quaternion =
    {
        let mut init =
            quaternion{w: 1i32 as libc::c_float,
                       x: 0i32 as libc::c_float,
                       y: 0i32 as libc::c_float,
                       z: 0i32 as libc::c_float,};
        init
    };
#[no_mangle]
pub static mut offset: quaternion =
    {
        let mut init =
            quaternion{w: 1i32 as libc::c_float,
                       x: 0i32 as libc::c_float,
                       y: 0i32 as libc::c_float,
                       z: 0i32 as libc::c_float,};
        init
    };
// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
#[no_mangle]
pub static mut attitude: attitudeEulerAngles_t =
    attitudeEulerAngles_t{raw:
                              [0i32 as int16_t, 0i32 as int16_t,
                               0i32 as int16_t],};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut imuConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (22i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<imuConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &imuConfig_System as *const imuConfig_t as
                                     *mut imuConfig_t as *mut uint8_t,
                             copy:
                                 &imuConfig_Copy as *const imuConfig_t as
                                     *mut imuConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_imuConfig
                                                         as *const imuConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut imuConfig_System: imuConfig_t =
    imuConfig_t{dcm_kp: 0,
                dcm_ki: 0,
                small_angle: 0,
                acc_unarmedcal: 0,
                accDeadband: accDeadband_t{xy: 0, z: 0,},};
#[no_mangle]
pub static mut imuConfig_Copy: imuConfig_t =
    imuConfig_t{dcm_kp: 0,
                dcm_ki: 0,
                small_angle: 0,
                acc_unarmedcal: 0,
                accDeadband: accDeadband_t{xy: 0, z: 0,},};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_imuConfig: imuConfig_t =
    {
        let mut init =
            imuConfig_s{dcm_kp: 2500i32 as uint16_t,
                        dcm_ki: 0i32 as uint16_t,
                        small_angle: 25i32 as uint8_t,
                        acc_unarmedcal: 1i32 as uint8_t,
                        accDeadband:
                            {
                                let mut init =
                                    accDeadband_s{xy: 40i32 as uint8_t,
                                                  z: 40i32 as uint8_t,};
                                init
                            },};
        init
    };
unsafe extern "C" fn imuComputeRotationMatrix() {
    imuQuaternionComputeProducts(&mut q, &mut qP);
    rMat[0][0] = 1.0f32 - 2.0f32 * qP.yy - 2.0f32 * qP.zz;
    rMat[0][1] = 2.0f32 * (qP.xy + -qP.wz);
    rMat[0][2] = 2.0f32 * (qP.xz - -qP.wy);
    rMat[1][0] = 2.0f32 * (qP.xy - -qP.wz);
    rMat[1][1] = 1.0f32 - 2.0f32 * qP.xx - 2.0f32 * qP.zz;
    rMat[1][2] = 2.0f32 * (qP.yz + -qP.wx);
    rMat[2][0] = 2.0f32 * (qP.xz + -qP.wy);
    rMat[2][1] = 2.0f32 * (qP.yz - -qP.wx);
    rMat[2][2] = 1.0f32 - 2.0f32 * qP.xx - 2.0f32 * qP.yy;
    rMat[1][0] = -2.0f32 * (qP.xy - -qP.wz);
    rMat[2][0] = -2.0f32 * (qP.xz + -qP.wy);
}
/*
* Calculate RC time constant used in the accZ lpf.
*/
unsafe extern "C" fn calculateAccZLowPassFilterRCTimeConstant(mut accz_lpf_cutoff:
                                                                  libc::c_float)
 -> libc::c_float {
    return 0.5f32 / (3.14159265358979323846f32 * accz_lpf_cutoff);
}
unsafe extern "C" fn calculateThrottleAngleScale(mut throttle_correction_angle:
                                                     uint16_t)
 -> libc::c_float {
    return 1800.0f32 / 3.14159265358979323846f32 *
               (900.0f32 /
                    throttle_correction_angle as libc::c_int as
                        libc::c_float);
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
// Exported symbols
// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
// set the acc deadband for xy-Axis
// set the acc deadband for z-Axis, this ignores small accelerations
// DCM filter proportional gain ( x 10000)
// DCM filter integral gain ( x 10000)
// turn automatic acc compensation on/off
#[no_mangle]
pub unsafe extern "C" fn imuConfigure(mut throttle_correction_angle: uint16_t,
                                      mut throttle_correction_value:
                                          uint8_t) {
    imuRuntimeConfig.dcm_kp =
        (*imuConfig()).dcm_kp as libc::c_int as libc::c_float /
            10000.0f32; // Set to fix value
    imuRuntimeConfig.dcm_ki =
        (*imuConfig()).dcm_ki as libc::c_int as libc::c_float /
            10000.0f32; // integral error terms scaled by Ki
    imuRuntimeConfig.acc_unarmedcal = (*imuConfig()).acc_unarmedcal;
    imuRuntimeConfig.small_angle = (*imuConfig()).small_angle;
    fc_acc = calculateAccZLowPassFilterRCTimeConstant(5.0f32);
    throttleAngleScale =
        calculateThrottleAngleScale(throttle_correction_angle);
    throttleAngleValue = throttle_correction_value as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn imuInit() {
    smallAngleCosZ =
        cos_approx(degreesToRadians(imuRuntimeConfig.small_angle as int16_t));
    accVelScale =
        9.80665f32 / acc.dev.acc_1G as libc::c_int as libc::c_float /
            10000.0f32;
    canUseGPSHeading = 1i32 != 0;
    imuComputeRotationMatrix();
    if pthread_mutex_init(&mut imuUpdateLock, 0 as *const pthread_mutexattr_t)
           != 0i32 {
        printf(b"Create imuUpdateLock error!\n\x00" as *const u8 as
                   *const libc::c_char);
    };
}
#[no_mangle]
pub unsafe extern "C" fn imuResetAccelerationSum() {
    accSum[0] = 0i32;
    accSum[1] = 0i32;
    accSum[2] = 0i32;
    accSumCount = 0i32;
    accTimeSum = 0i32 as uint32_t;
}
unsafe extern "C" fn invSqrt(mut x: libc::c_float) -> libc::c_float {
    return 1.0f32 / sqrtf(x);
}
unsafe extern "C" fn imuMahonyAHRSupdate(mut dt: libc::c_float,
                                         mut gx: libc::c_float,
                                         mut gy: libc::c_float,
                                         mut gz: libc::c_float,
                                         mut useAcc: bool,
                                         mut ax: libc::c_float,
                                         mut ay: libc::c_float,
                                         mut az: libc::c_float,
                                         mut useMag: bool,
                                         mut mx: libc::c_float,
                                         mut my: libc::c_float,
                                         mut mz: libc::c_float,
                                         mut useCOG: bool,
                                         mut courseOverGround: libc::c_float,
                                         dcmKpGain: libc::c_float) {
    static mut integralFBx: libc::c_float = 0.0f32;
    static mut integralFBy: libc::c_float = 0.0f32;
    static mut integralFBz: libc::c_float = 0.0f32;
    // Calculate general spin rate (rad/s)
    let spin_rate: libc::c_float = sqrtf(gx * gx + gy * gy + gz * gz);
    // Use raw heading error (from GPS or whatever else)
    let mut ex: libc::c_float = 0i32 as libc::c_float;
    let mut ey: libc::c_float = 0i32 as libc::c_float;
    let mut ez: libc::c_float = 0i32 as libc::c_float;
    if useCOG {
        while courseOverGround > 3.14159265358979323846f32 {
            courseOverGround -= 2.0f32 * 3.14159265358979323846f32
        }
        while courseOverGround < -3.14159265358979323846f32 {
            courseOverGround += 2.0f32 * 3.14159265358979323846f32
        }
        let ez_ef: libc::c_float =
            -sin_approx(courseOverGround) * rMat[0][0] -
                cos_approx(courseOverGround) * rMat[1][0];
        ex = rMat[2][0] * ez_ef;
        ey = rMat[2][1] * ez_ef;
        ez = rMat[2][2] * ez_ef
    }
    // Use measured magnetic field vector
    let mut recipMagNorm: libc::c_float = mx * mx + my * my + mz * mz;
    if useMag as libc::c_int != 0 && recipMagNorm > 0.01f32 {
        // Normalise magnetometer measurement
        recipMagNorm = invSqrt(recipMagNorm);
        mx *= recipMagNorm;
        my *= recipMagNorm;
        mz *= recipMagNorm;
        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles
        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        let hx: libc::c_float =
            rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        let hy: libc::c_float =
            rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        let bx: libc::c_float = sqrtf(hx * hx + hy * hy);
        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        let ez_ef_0: libc::c_float = -(hy * bx);
        // Rotate mag error vector back to BF and accumulate
        ex += rMat[2][0] * ez_ef_0;
        ey += rMat[2][1] * ez_ef_0;
        ez += rMat[2][2] * ez_ef_0
    }
    // Use measured acceleration vector
    let mut recipAccNorm: libc::c_float = ax * ax + ay * ay + az * az;
    if useAcc as libc::c_int != 0 && recipAccNorm > 0.01f32 {
        // Normalise accelerometer measurement
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;
        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += ay * rMat[2][2] - az * rMat[2][1];
        ey += az * rMat[2][0] - ax * rMat[2][2];
        ez += ax * rMat[2][1] - ay * rMat[2][0]
    }
    // Compute and apply integral feedback if enabled
    if imuRuntimeConfig.dcm_ki > 0.0f32 {
        // Stop integrating if spinning beyond the certain limit
        if spin_rate < 20i32 as libc::c_float * 0.0174532925f32 {
            let dcmKiGain: libc::c_float =
                imuRuntimeConfig.dcm_ki; // integral error scaled by Ki
            integralFBx += dcmKiGain * ex * dt; // prevent integral windup
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt
        }
    } else {
        integralFBx = 0.0f32;
        integralFBy = 0.0f32;
        integralFBz = 0.0f32
    }
    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;
    // Integrate rate of change of quaternion
    gx *= 0.5f32 * dt;
    gy *= 0.5f32 * dt;
    gz *= 0.5f32 * dt;
    let mut buffer: quaternion = quaternion{w: 0., x: 0., y: 0., z: 0.,};
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;
    q.w += -buffer.x * gx - buffer.y * gy - buffer.z * gz;
    q.x += buffer.w * gx + buffer.y * gz - buffer.z * gy;
    q.y += buffer.w * gy - buffer.x * gz + buffer.z * gx;
    q.z += buffer.w * gz + buffer.x * gy - buffer.y * gx;
    // Normalise quaternion
    let mut recipNorm: libc::c_float =
        invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}
unsafe extern "C" fn imuUpdateEulerAngles() {
    let mut buffer: quaternionProducts =
        quaternionProducts{ww: 0.,
                           wx: 0.,
                           wy: 0.,
                           wz: 0.,
                           xx: 0.,
                           xy: 0.,
                           xz: 0.,
                           yy: 0.,
                           yz: 0.,
                           zz: 0.,};
    if flightModeFlags as libc::c_int & HEADFREE_MODE as libc::c_int != 0 {
        imuQuaternionComputeProducts(&mut headfree, &mut buffer);
        attitude.values.roll =
            lrintf(atan2_approx(2.0f32 * (buffer.wx + buffer.yz),
                                1.0f32 - 2.0f32 * (buffer.xx + buffer.yy)) *
                       (1800.0f32 / 3.14159265358979323846f32)) as int16_t;
        attitude.values.pitch =
            lrintf((0.5f32 * 3.14159265358979323846f32 -
                        acos_approx(2.0f32 * (buffer.wy - buffer.xz))) *
                       (1800.0f32 / 3.14159265358979323846f32)) as int16_t;
        attitude.values.yaw =
            lrintf(-atan2_approx(2.0f32 * (buffer.wz + buffer.xy),
                                 1.0f32 - 2.0f32 * (buffer.yy + buffer.zz)) *
                       (1800.0f32 / 3.14159265358979323846f32)) as int16_t
    } else {
        attitude.values.roll =
            lrintf(atan2_approx(rMat[2][1], rMat[2][2]) *
                       (1800.0f32 / 3.14159265358979323846f32)) as int16_t;
        attitude.values.pitch =
            lrintf((0.5f32 * 3.14159265358979323846f32 -
                        acos_approx(-rMat[2][0])) *
                       (1800.0f32 / 3.14159265358979323846f32)) as int16_t;
        attitude.values.yaw =
            lrintf(-atan2_approx(rMat[1][0], rMat[0][0]) *
                       (1800.0f32 / 3.14159265358979323846f32)) as int16_t
    }
    if (attitude.values.yaw as libc::c_int) < 0i32 {
        attitude.values.yaw =
            (attitude.values.yaw as libc::c_int + 3600i32) as int16_t
    }
    // Update small angle state
    if rMat[2][2] > smallAngleCosZ {
        stateFlags =
            (stateFlags as libc::c_int | SMALL_ANGLE as libc::c_int) as
                uint8_t
    } else {
        stateFlags =
            (stateFlags as libc::c_int & !(SMALL_ANGLE as libc::c_int)) as
                uint8_t
    };
}
unsafe extern "C" fn imuIsAccelerometerHealthy(mut accAverage_0:
                                                   *mut libc::c_float)
 -> bool {
    let mut accMagnitude: libc::c_float = 0i32 as libc::c_float;
    let mut axis: libc::c_int = 0i32;
    while axis < 3i32 {
        let a: libc::c_float = *accAverage_0.offset(axis as isize);
        accMagnitude += a * a;
        axis += 1
    }
    accMagnitude =
        accMagnitude * 100i32 as libc::c_float /
            (acc.dev.acc_1G as int32_t * acc.dev.acc_1G as int32_t) as
                libc::c_float;
    // Accept accel readings only in range 0.90g - 1.10g
    return (81i32 as libc::c_float) < accMagnitude &&
               accMagnitude < 121i32 as libc::c_float;
}
// Calculate the dcmKpGain to use. When armed, the gain is imuRuntimeConfig.dcm_kp * 1.0 scaling.
// When disarmed after initial boot, the scaling is set to 10.0 for the first 20 seconds to speed up initial convergence.
// After disarming we want to quickly reestablish convergence to deal with the attitude estimation being incorrect due to a crash.
//   - wait for a 250ms period of low gyro activity to ensure the craft is not moving
//   - use a large dcmKpGain value for 500ms to allow the attitude estimate to quickly converge
//   - reset the gain back to the standard setting
#[no_mangle]
pub unsafe extern "C" fn imuCalcKpGain(mut currentTimeUs: timeUs_t,
                                       mut useAcc: bool,
                                       mut gyroAverage: *mut libc::c_float)
 -> libc::c_float {
    static mut lastArmState: bool = 0i32 != 0;
    static mut gyroQuietPeriodTimeEnd: timeUs_t = 0i32 as timeUs_t;
    static mut attitudeResetTimeEnd: timeUs_t = 0i32 as timeUs_t;
    static mut attitudeResetCompleted: bool = 0i32 != 0;
    let mut ret: libc::c_float = 0.;
    let mut attitudeResetActive: bool = 0i32 != 0;
    let armState: bool =
        armingFlags as libc::c_int & ARMED as libc::c_int != 0;
    if !armState {
        if lastArmState {
            // Just disarmed; start the gyro quiet period
            gyroQuietPeriodTimeEnd =
                currentTimeUs.wrapping_add(250000i32 as libc::c_uint);
            attitudeResetTimeEnd = 0i32 as timeUs_t;
            attitudeResetCompleted = 0i32 != 0
        }
        // If gyro activity exceeds the threshold then restart the quiet period.
        // Also, if the attitude reset has been complete and there is subsequent gyro activity then
        // start the reset cycle again. This addresses the case where the pilot rights the craft after a crash.
        if attitudeResetTimeEnd > 0i32 as libc::c_uint ||
               gyroQuietPeriodTimeEnd > 0i32 as libc::c_uint ||
               attitudeResetCompleted as libc::c_int != 0 {
            if fabsf(*gyroAverage.offset(X as libc::c_int as isize)) >
                   15i32 as libc::c_float ||
                   fabsf(*gyroAverage.offset(Y as libc::c_int as isize)) >
                       15i32 as libc::c_float ||
                   fabsf(*gyroAverage.offset(Z as libc::c_int as isize)) >
                       15i32 as libc::c_float || !useAcc {
                gyroQuietPeriodTimeEnd =
                    currentTimeUs.wrapping_add(250000i32 as libc::c_uint);
                attitudeResetTimeEnd = 0i32 as timeUs_t
            }
        }
        if attitudeResetTimeEnd > 0i32 as libc::c_uint {
            // Resetting the attitude estimation
            if currentTimeUs >= attitudeResetTimeEnd {
                gyroQuietPeriodTimeEnd = 0i32 as timeUs_t;
                attitudeResetTimeEnd = 0i32 as timeUs_t;
                attitudeResetCompleted = 1i32 != 0
            } else { attitudeResetActive = 1i32 != 0 }
        } else if gyroQuietPeriodTimeEnd > 0i32 as libc::c_uint &&
                      currentTimeUs >= gyroQuietPeriodTimeEnd {
            // Start the high gain period to bring the estimation into convergence
            attitudeResetTimeEnd =
                currentTimeUs.wrapping_add(500000i32 as libc::c_uint);
            gyroQuietPeriodTimeEnd = 0i32 as timeUs_t
        }
    }
    lastArmState = armState;
    if attitudeResetActive {
        ret = 25.0f64 as libc::c_float
    } else {
        ret = imuRuntimeConfig.dcm_kp;
        if !armState {
            ret = ret * 10.0f32
            // Scale the kP to generally converge faster when disarmed.
        }
    } // Whether or not correct yaw via imuMahonyAHRSupdate from our ground course
    return ret; // To be used when useCOG is true.  Stored in Radians
}
unsafe extern "C" fn imuCalculateEstimatedAttitude(mut currentTimeUs:
                                                       timeUs_t) {
    static mut previousIMUUpdateTime: timeUs_t = 0;
    let mut useAcc: bool = 0i32 != 0;
    let mut useMag: bool = 0i32 != 0;
    let mut useCOG: bool = 0i32 != 0;
    let mut courseOverGround: libc::c_float = 0i32 as libc::c_float;
    let deltaT: timeDelta_t =
        currentTimeUs.wrapping_sub(previousIMUUpdateTime) as timeDelta_t;
    previousIMUUpdateTime = currentTimeUs;
    if sensors(SENSOR_MAG as libc::c_int as uint32_t) as libc::c_int != 0 &&
           compassIsHealthy() as libc::c_int != 0 {
        useMag = 1i32 != 0
    }
    if !useMag &&
           sensors(SENSOR_GPS as libc::c_int as uint32_t) as libc::c_int != 0
           && stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 &&
           gpsSol.numSat as libc::c_int >= 5i32 &&
           gpsSol.groundSpeed as libc::c_int >= 500i32 {
        // Use GPS course over ground to correct attitude.values.yaw
        if stateFlags as libc::c_int & FIXED_WING as libc::c_int != 0 {
            courseOverGround =
                gpsSol.groundCourse as libc::c_int as libc::c_float / 10.0f32
                    * 0.0174532925f32;
            useCOG = 1i32 != 0
        } else if canUseGPSHeading {
            courseOverGround =
                gpsSol.groundCourse as libc::c_int as libc::c_float / 10.0f32
                    * 0.0174532925f32;
            useCOG = 1i32 != 0
        }
        if useCOG as libc::c_int != 0 &&
               shouldInitializeGPSHeading() as libc::c_int != 0 {
            // If GPS rescue mode is active and we can use it, go for it.  When we're close to home we will
            // probably stop re calculating GPS heading data.  Other future modes can also use this extern
            // Reset our reference and reinitialize quaternion.  This will likely ideally happen more than once per flight, but for now,
            // shouldInitializeGPSHeading() returns true only once.
            imuComputeQuaternionFromRPY(&mut qP, attitude.values.roll,
                                        attitude.values.pitch,
                                        gpsSol.groundCourse as int16_t);
            useCOG = 0i32 != 0
            // Don't use the COG when we first reinitialize.  Next time around though, yes.
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn calculateThrottleAngleCorrection() -> libc::c_int {
    /*
    * Use 0 as the throttle angle correction if we are inverted, vertical or with a
    * small angle < 0.86 deg
    * TODO: Define this small angle in config.
    */
    if rMat[2][2] <= 0.015f32 { return 0i32 }
    let mut angle: libc::c_int =
        lrintf(acos_approx(rMat[2][2]) * throttleAngleScale) as libc::c_int;
    if angle > 900i32 { angle = 900i32 }
    return lrintf(throttleAngleValue as libc::c_float *
                      sin_approx(angle as libc::c_float /
                                     (900.0f32 * 3.14159265358979323846f32 /
                                          2.0f32))) as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn imuUpdateAttitude(mut currentTimeUs: timeUs_t) {
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) as libc::c_int != 0 &&
           acc.isAccelUpdatedAtLeastOnce as libc::c_int != 0 {
        pthread_mutex_lock(&mut imuUpdateLock);
        imuCalculateEstimatedAttitude(currentTimeUs);
        pthread_mutex_unlock(&mut imuUpdateLock);
        // Update the throttle correction for angle and supply it to the mixer
        let mut throttleAngleCorrection: libc::c_int = 0i32;
        if throttleAngleValue != 0 &&
               (flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int !=
                    0 ||
                    flightModeFlags as libc::c_int &
                        HORIZON_MODE as libc::c_int != 0) &&
               armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            throttleAngleCorrection = calculateThrottleAngleCorrection()
        }
        mixerSetThrottleAngleCorrection(throttleAngleCorrection);
    } else {
        acc.accADC[X as libc::c_int as usize] = 0i32 as libc::c_float;
        acc.accADC[Y as libc::c_int as usize] = 0i32 as libc::c_float;
        acc.accADC[Z as libc::c_int as usize] = 0i32 as libc::c_float
    };
}
// in deg
#[no_mangle]
pub unsafe extern "C" fn shouldInitializeGPSHeading() -> bool {
    static mut initialized: bool = 0i32 != 0;
    if !initialized { initialized = 1i32 != 0; return 1i32 != 0 }
    return 0i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn getCosTiltAngle() -> libc::c_float {
    return rMat[2][2];
}
#[no_mangle]
pub unsafe extern "C" fn getQuaternion(mut quat: *mut quaternion) {
    (*quat).w = q.w;
    (*quat).x = q.x;
    (*quat).y = q.y;
    (*quat).z = q.z;
}
#[no_mangle]
pub unsafe extern "C" fn imuComputeQuaternionFromRPY(mut quatProd:
                                                         *mut quaternionProducts,
                                                     mut initialRoll: int16_t,
                                                     mut initialPitch:
                                                         int16_t,
                                                     mut initialYaw:
                                                         int16_t) {
    if initialRoll as libc::c_int > 1800i32 {
        initialRoll = (initialRoll as libc::c_int - 3600i32) as int16_t
    }
    if initialPitch as libc::c_int > 1800i32 {
        initialPitch = (initialPitch as libc::c_int - 3600i32) as int16_t
    }
    if initialYaw as libc::c_int > 1800i32 {
        initialYaw = (initialYaw as libc::c_int - 3600i32) as int16_t
    }
    let cosRoll: libc::c_float =
        cos_approx(initialRoll as libc::c_int as libc::c_float / 10.0f32 *
                       0.0174532925f32 * 0.5f32);
    let sinRoll: libc::c_float =
        sin_approx(initialRoll as libc::c_int as libc::c_float / 10.0f32 *
                       0.0174532925f32 * 0.5f32);
    let cosPitch: libc::c_float =
        cos_approx(initialPitch as libc::c_int as libc::c_float / 10.0f32 *
                       0.0174532925f32 * 0.5f32);
    let sinPitch: libc::c_float =
        sin_approx(initialPitch as libc::c_int as libc::c_float / 10.0f32 *
                       0.0174532925f32 * 0.5f32);
    let cosYaw: libc::c_float =
        cos_approx(-(initialYaw as libc::c_int) as libc::c_float / 10.0f32 *
                       0.0174532925f32 * 0.5f32);
    let sinYaw: libc::c_float =
        sin_approx(-(initialYaw as libc::c_int) as libc::c_float / 10.0f32 *
                       0.0174532925f32 * 0.5f32);
    let q0: libc::c_float =
        cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    let q1: libc::c_float =
        sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    let q2: libc::c_float =
        cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    let q3: libc::c_float =
        cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    (*quatProd).xx = q1 * q1;
    (*quatProd).yy = q2 * q2;
    (*quatProd).zz = q3 * q3;
    (*quatProd).xy = q1 * q2;
    (*quatProd).xz = q1 * q3;
    (*quatProd).yz = q2 * q3;
    (*quatProd).wx = q0 * q1;
    (*quatProd).wy = q0 * q2;
    (*quatProd).wz = q0 * q3;
    imuComputeRotationMatrix();
}
#[no_mangle]
pub unsafe extern "C" fn imuSetAttitudeRPY(mut roll: libc::c_float,
                                           mut pitch: libc::c_float,
                                           mut yaw: libc::c_float) {
    pthread_mutex_lock(&mut imuUpdateLock);
    attitude.values.roll = (roll * 10i32 as libc::c_float) as int16_t;
    attitude.values.pitch = (pitch * 10i32 as libc::c_float) as int16_t;
    attitude.values.yaw = (yaw * 10i32 as libc::c_float) as int16_t;
    pthread_mutex_unlock(&mut imuUpdateLock);
}
#[no_mangle]
pub unsafe extern "C" fn imuSetAttitudeQuat(mut w: libc::c_float,
                                            mut x: libc::c_float,
                                            mut y: libc::c_float,
                                            mut z: libc::c_float) {
    pthread_mutex_lock(&mut imuUpdateLock);
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    imuComputeRotationMatrix();
    imuUpdateEulerAngles();
    pthread_mutex_unlock(&mut imuUpdateLock);
}
#[no_mangle]
pub unsafe extern "C" fn imuQuaternionComputeProducts(mut quat:
                                                          *mut quaternion,
                                                      mut quatProd:
                                                          *mut quaternionProducts) {
    (*quatProd).ww = (*quat).w * (*quat).w;
    (*quatProd).wx = (*quat).w * (*quat).x;
    (*quatProd).wy = (*quat).w * (*quat).y;
    (*quatProd).wz = (*quat).w * (*quat).z;
    (*quatProd).xx = (*quat).x * (*quat).x;
    (*quatProd).xy = (*quat).x * (*quat).y;
    (*quatProd).xz = (*quat).x * (*quat).z;
    (*quatProd).yy = (*quat).y * (*quat).y;
    (*quatProd).yz = (*quat).y * (*quat).z;
    (*quatProd).zz = (*quat).z * (*quat).z;
}
#[no_mangle]
pub unsafe extern "C" fn imuQuaternionHeadfreeOffsetSet() -> bool {
    if ({
            let mut _x: int16_t = attitude.values.roll;
            (if _x as libc::c_int > 0i32 {
                 _x as libc::c_int
             } else { -(_x as libc::c_int) })
        }) < 450i32 &&
           ({
                let mut _x: int16_t = attitude.values.pitch;
                (if _x as libc::c_int > 0i32 {
                     _x as libc::c_int
                 } else { -(_x as libc::c_int) })
            }) < 450i32 {
        let yaw: libc::c_float =
            -atan2_approx(2.0f32 * (qP.wz + qP.xy),
                          1.0f32 - 2.0f32 * (qP.yy + qP.zz));
        offset.w = cos_approx(yaw / 2i32 as libc::c_float);
        offset.x = 0i32 as libc::c_float;
        offset.y = 0i32 as libc::c_float;
        offset.z = sin_approx(yaw / 2i32 as libc::c_float);
        return 1i32 != 0
    } else { return 0i32 != 0 };
}
#[no_mangle]
pub unsafe extern "C" fn imuQuaternionMultiplication(mut q1: *mut quaternion,
                                                     mut q2: *mut quaternion,
                                                     mut result:
                                                         *mut quaternion) {
    let A: libc::c_float = ((*q1).w + (*q1).x) * ((*q2).w + (*q2).x);
    let B: libc::c_float = ((*q1).z - (*q1).y) * ((*q2).y - (*q2).z);
    let C: libc::c_float = ((*q1).w - (*q1).x) * ((*q2).y + (*q2).z);
    let D: libc::c_float = ((*q1).y + (*q1).z) * ((*q2).w - (*q2).x);
    let E: libc::c_float = ((*q1).x + (*q1).z) * ((*q2).x + (*q2).y);
    let F: libc::c_float = ((*q1).x - (*q1).z) * ((*q2).x - (*q2).y);
    let G: libc::c_float = ((*q1).w + (*q1).y) * ((*q2).w - (*q2).z);
    let H: libc::c_float = ((*q1).w - (*q1).y) * ((*q2).w + (*q2).z);
    (*result).w = B + (-E - F + G + H) / 2.0f32;
    (*result).x = A - (E + F + G + H) / 2.0f32;
    (*result).y = C + (E - F + G - H) / 2.0f32;
    (*result).z = D + (E - F - G + H) / 2.0f32;
}
#[no_mangle]
pub unsafe extern "C" fn imuQuaternionHeadfreeTransformVectorEarthToBody(mut v:
                                                                             *mut t_fp_vector_def) {
    let mut buffer: quaternionProducts =
        quaternionProducts{ww: 0.,
                           wx: 0.,
                           wy: 0.,
                           wz: 0.,
                           xx: 0.,
                           xy: 0.,
                           xz: 0.,
                           yy: 0.,
                           yz: 0.,
                           zz: 0.,};
    imuQuaternionMultiplication(&mut offset, &mut q, &mut headfree);
    imuQuaternionComputeProducts(&mut headfree, &mut buffer);
    let x: libc::c_float =
        (buffer.ww + buffer.xx - buffer.yy - buffer.zz) * (*v).X +
            2.0f32 * (buffer.xy + buffer.wz) * (*v).Y +
            2.0f32 * (buffer.xz - buffer.wy) * (*v).Z;
    let y: libc::c_float =
        2.0f32 * (buffer.xy - buffer.wz) * (*v).X +
            (buffer.ww - buffer.xx + buffer.yy - buffer.zz) * (*v).Y +
            2.0f32 * (buffer.yz + buffer.wx) * (*v).Z;
    let z: libc::c_float =
        2.0f32 * (buffer.xz + buffer.wy) * (*v).X +
            2.0f32 * (buffer.yz - buffer.wx) * (*v).Y +
            (buffer.ww - buffer.xx - buffer.yy + buffer.zz) * (*v).Z;
    (*v).X = x;
    (*v).Y = y;
    (*v).Z = z;
}
