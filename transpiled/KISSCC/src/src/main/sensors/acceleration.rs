use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    fn biquadFilterInitLPF(filter: *mut biquadFilter_t,
                           filterFreq: libc::c_float, refreshRate: uint32_t);
    #[no_mangle]
    fn biquadFilterApply(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn gyroMpuDetectionResult() -> *const mpuDetectionResult_s;
    #[no_mangle]
    fn gyroSensorBus() -> *const busDevice_t;
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
    fn mpu6050AccDetect(acc_0: *mut accDev_t) -> bool;
    #[no_mangle]
    fn saveConfigAndNotify();
    #[no_mangle]
    fn sensorsSet(mask: uint32_t);
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    #[no_mangle]
    static mut detectedSensors: [uint8_t; 5];
    #[no_mangle]
    fn alignSensors(dest: *mut libc::c_float, rotation: uint8_t);
    // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
    #[no_mangle]
    static mut InflightcalibratingA: uint16_t;
    #[no_mangle]
    static mut AccInflightCalibrationMeasurementDone: bool;
    #[no_mangle]
    static mut AccInflightCalibrationSavetoEEProm: bool;
    #[no_mangle]
    static mut AccInflightCalibrationActive: bool;
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
#[derive(Copy, Clone)]
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_1 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_1 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_1 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_1 = 4095;
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_2,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_2 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_3 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_3 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_3 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_3 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_3 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_3 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_3 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_3 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_3 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_3 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_3 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_3 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_3 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_3 = 8192;
pub const FEATURE_3D: C2RustUnnamed_3 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_3 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_3 = 512;
pub const FEATURE_GPS: C2RustUnnamed_3 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_3 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_3 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_3 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_3 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_3 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_3 = 1;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_4,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_4 {
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accDev_s {
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
pub type mpuDetectionResult_t = mpuDetectionResult_s;
#[derive(Copy, Clone)]
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
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
// microsecond time
pub type timeUs_t = uint32_t;
pub type accDev_t = accDev_s;
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
pub type C2RustUnnamed_5 = libc::c_uint;
pub const SENSOR_INDEX_COUNT: C2RustUnnamed_5 = 5;
pub const SENSOR_INDEX_RANGEFINDER: C2RustUnnamed_5 = 4;
pub const SENSOR_INDEX_MAG: C2RustUnnamed_5 = 3;
pub const SENSOR_INDEX_BARO: C2RustUnnamed_5 = 2;
pub const SENSOR_INDEX_ACC: C2RustUnnamed_5 = 1;
pub const SENSOR_INDEX_GYRO: C2RustUnnamed_5 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct int16_flightDynamicsTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type flightDynamicsTrims_def_t = int16_flightDynamicsTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union flightDynamicsTrims_u {
    pub raw: [int16_t; 3],
    pub values: flightDynamicsTrims_def_t,
}
pub type flightDynamicsTrims_t = flightDynamicsTrims_u;
// 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
pub type C2RustUnnamed_6 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_6 = 64;
pub const SENSOR_GPS: C2RustUnnamed_6 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_6 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_6 = 16;
pub const SENSOR_MAG: C2RustUnnamed_6 = 8;
pub const SENSOR_BARO: C2RustUnnamed_6 = 4;
// always present
pub const SENSOR_ACC: C2RustUnnamed_6 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_6 = 1;
pub type accelerationSensor_e = libc::c_uint;
pub const ACC_FAKE: accelerationSensor_e = 16;
pub const ACC_BMI160: accelerationSensor_e = 15;
pub const ACC_ICM20689: accelerationSensor_e = 14;
pub const ACC_ICM20649: accelerationSensor_e = 13;
pub const ACC_ICM20608G: accelerationSensor_e = 12;
pub const ACC_ICM20602: accelerationSensor_e = 11;
pub const ACC_ICM20601: accelerationSensor_e = 10;
pub const ACC_MPU9250: accelerationSensor_e = 9;
pub const ACC_MPU6500: accelerationSensor_e = 8;
pub const ACC_MPU6000: accelerationSensor_e = 7;
pub const ACC_LSM303DLHC: accelerationSensor_e = 6;
pub const ACC_BMA280: accelerationSensor_e = 5;
pub const ACC_MMA8452: accelerationSensor_e = 4;
pub const ACC_MPU6050: accelerationSensor_e = 3;
pub const ACC_ADXL345: accelerationSensor_e = 2;
pub const ACC_NONE: accelerationSensor_e = 1;
pub const ACC_DEFAULT: accelerationSensor_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub type acc_t = acc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
}
pub type rollAndPitchTrims_t_def = rollAndPitchTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union rollAndPitchTrims_u {
    pub raw: [int16_t; 2],
    pub values: rollAndPitchTrims_t_def,
}
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
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
pub type accelerometerConfig_t = accelerometerConfig_s;
#[inline]
unsafe extern "C" fn accelerometerConfig() -> *const accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn accelerometerConfigMutable()
 -> *mut accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
// cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
// acc alignment
// Which acc hardware to use on boards with more than one device
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
pub static mut acc: acc_t =
    acc_t{dev:
              accDev_t{initFn: None,
                       readFn: None,
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
                       acc_1G: 0,
                       ADCRaw: [0; 3],
                       mpuDetectionResult:
                           mpuDetectionResult_t{sensor: MPU_NONE,
                                                resolution:
                                                    MPU_HALF_RESOLUTION,},
                       accAlign: ALIGN_DEFAULT,
                       dataReady: false,
                       acc_high_fsr: false,
                       revisionCode: 0,
                       filler: [0; 2],},
          accSamplingInterval: 0,
          accADC: [0.; 3],
          isAccelUpdatedAtLeastOnce: false,};
// acc access functions
static mut accumulatedMeasurements: [libc::c_float; 3] = [0.; 3];
static mut accumulatedMeasurementCount: libc::c_int = 0;
static mut calibratingA: uint16_t = 0 as libc::c_int as uint16_t;
static mut accelerationTrims: *mut flightDynamicsTrims_t =
    0 as *const flightDynamicsTrims_t as *mut flightDynamicsTrims_t;
static mut accLpfCutHz: uint16_t = 0 as libc::c_int as uint16_t;
static mut accFilter: [biquadFilter_t; 3] =
    [biquadFilter_t{b0: 0.,
                    b1: 0.,
                    b2: 0.,
                    a1: 0.,
                    a2: 0.,
                    x1: 0.,
                    x2: 0.,
                    y1: 0.,
                    y2: 0.,}; 3];
#[no_mangle]
pub static mut accelerometerConfig_System: accelerometerConfig_t =
    accelerometerConfig_t{acc_lpf_hz: 0,
                          acc_align: ALIGN_DEFAULT,
                          acc_hardware: 0,
                          acc_high_fsr: false,
                          accZero: flightDynamicsTrims_u{raw: [0; 3],},
                          accelerometerTrims:
                              rollAndPitchTrims_u{raw: [0; 2],},};
#[no_mangle]
pub static mut accelerometerConfig_Copy: accelerometerConfig_t =
    accelerometerConfig_t{acc_lpf_hz: 0,
                          acc_align: ALIGN_DEFAULT,
                          acc_hardware: 0,
                          acc_high_fsr: false,
                          accZero: flightDynamicsTrims_u{raw: [0; 3],},
                          accelerometerTrims:
                              rollAndPitchTrims_u{raw: [0; 2],},};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut accelerometerConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (35 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<accelerometerConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &accelerometerConfig_System as
                                     *const accelerometerConfig_t as
                                     *mut accelerometerConfig_t as
                                     *mut uint8_t,
                             copy:
                                 &accelerometerConfig_Copy as
                                     *const accelerometerConfig_t as
                                     *mut accelerometerConfig_t as
                                     *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_2{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut accelerometerConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_accelerometerConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut accelerometerConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn resetRollAndPitchTrims(mut rollAndPitchTrims:
                                                    *mut rollAndPitchTrims_t) {
    *rollAndPitchTrims =
        rollAndPitchTrims_u{values:
                                {
                                    let mut init =
                                        rollAndPitchTrims_s{roll:
                                                                0 as
                                                                    libc::c_int
                                                                    as
                                                                    int16_t,
                                                            pitch:
                                                                0 as
                                                                    libc::c_int
                                                                    as
                                                                    int16_t,};
                                    init
                                },};
}
#[no_mangle]
pub unsafe extern "C" fn accResetRollAndPitchTrims() {
    resetRollAndPitchTrims(&mut (*(accelerometerConfigMutable as
                                       unsafe extern "C" fn()
                                           ->
                                               *mut accelerometerConfig_t)()).accelerometerTrims);
}
unsafe extern "C" fn resetFlightDynamicsTrims(mut accZero:
                                                  *mut flightDynamicsTrims_t) {
    (*accZero).values.roll = 0 as libc::c_int as int16_t;
    (*accZero).values.pitch = 0 as libc::c_int as int16_t;
    (*accZero).values.yaw = 0 as libc::c_int as int16_t;
}
#[no_mangle]
pub unsafe extern "C" fn accResetFlightDynamicsTrims() {
    resetFlightDynamicsTrims(&mut (*(accelerometerConfigMutable as
                                         unsafe extern "C" fn()
                                             ->
                                                 *mut accelerometerConfig_t)()).accZero);
}
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_accelerometerConfig(mut instance:
                                                           *mut accelerometerConfig_t) {
    *instance =
        {
            let mut init =
                accelerometerConfig_s{acc_lpf_hz:
                                          10 as libc::c_int as uint16_t,
                                      acc_align: ALIGN_DEFAULT,
                                      acc_hardware:
                                          ACC_DEFAULT as libc::c_int as
                                              uint8_t,
                                      acc_high_fsr: 0 as libc::c_int != 0,
                                      accZero:
                                          flightDynamicsTrims_u{raw: [0; 3],},
                                      accelerometerTrims:
                                          rollAndPitchTrims_u{raw: [0; 2],},};
            init
        };
    resetRollAndPitchTrims(&mut (*instance).accelerometerTrims);
    resetFlightDynamicsTrims(&mut (*instance).accZero);
}
#[no_mangle]
pub unsafe extern "C" fn accDetect(mut dev: *mut accDev_t,
                                   mut accHardwareToUse: accelerationSensor_e)
 -> bool {
    let mut accHardware: accelerationSensor_e = ACC_NONE;
    loop  {
        let mut current_block_8: u64;
        match accHardwareToUse as libc::c_uint {
            0 | 3 => {
                // MPU6050
                if mpu6050AccDetect(dev) {
                    (*dev).accAlign = CW90_DEG;
                    accHardware = ACC_MPU6050;
                    current_block_8 = 1054647088692577877;
                } else { current_block_8 = 13981425830553615451; }
            }
            8 | 10 | 11 | 12 | 1 | _ => {
                current_block_8 = 13981425830553615451;
            }
        }
        match current_block_8 {
            13981425830553615451 => {
                // disable ACC
                accHardware = ACC_NONE
            }
            _ => { }
        }
        // Found anything? Check if error or ACC is really missing.
        if !(accHardware as libc::c_uint ==
                 ACC_NONE as libc::c_int as libc::c_uint &&
                 accHardwareToUse as libc::c_uint !=
                     ACC_DEFAULT as libc::c_int as libc::c_uint &&
                 accHardwareToUse as libc::c_uint !=
                     ACC_NONE as libc::c_int as libc::c_uint) {
            break ;
        }
        // Nothing was found and we have a forced sensor that isn't present.
        accHardwareToUse = ACC_DEFAULT
    }
    if accHardware as libc::c_uint == ACC_NONE as libc::c_int as libc::c_uint
       {
        return 0 as libc::c_int != 0
    }
    detectedSensors[SENSOR_INDEX_ACC as libc::c_int as usize] =
        accHardware as uint8_t;
    sensorsSet(SENSOR_ACC as libc::c_int as uint32_t);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn accInit(mut gyroSamplingInverval: uint32_t) -> bool {
    memset(&mut acc as *mut acc_t as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<acc_t>() as libc::c_ulong);
    // copy over the common gyro mpu settings
    acc.dev.bus = *gyroSensorBus(); // set default
    acc.dev.mpuDetectionResult =
        *gyroMpuDetectionResult(); // driver initialisation
    acc.dev.acc_high_fsr = (*accelerometerConfig()).acc_high_fsr;
    acc.dev.accAlign = ALIGN_DEFAULT;
    if !accDetect(&mut acc.dev,
                  (*accelerometerConfig()).acc_hardware as
                      accelerationSensor_e) {
        return 0 as libc::c_int != 0
    }
    acc.dev.acc_1G = 256 as libc::c_int as uint16_t;
    acc.dev.initFn.expect("non-null function pointer")(&mut acc.dev);
    // set the acc sampling interval according to the gyro sampling interval
    match gyroSamplingInverval {
        500 | 375 | 250 | 125 => {
            // Switch statement kept in place to change acc sampling interval in the future
            acc.accSamplingInterval = 1000 as libc::c_int as uint32_t
        }
        1000 | _ => {
            acc.accSamplingInterval = 1000 as libc::c_int as uint32_t
        }
    }
    if accLpfCutHz != 0 {
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            biquadFilterInitLPF(&mut *accFilter.as_mut_ptr().offset(axis as
                                                                        isize),
                                accLpfCutHz as libc::c_float,
                                acc.accSamplingInterval);
            axis += 1
        }
    }
    if (*accelerometerConfig()).acc_align as libc::c_uint !=
           ALIGN_DEFAULT as libc::c_int as libc::c_uint {
        acc.dev.accAlign = (*accelerometerConfig()).acc_align
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn accSetCalibrationCycles(mut calibrationCyclesRequired:
                                                     uint16_t) {
    calibratingA = calibrationCyclesRequired;
}
#[no_mangle]
pub unsafe extern "C" fn accIsCalibrationComplete() -> bool {
    return calibratingA as libc::c_int == 0 as libc::c_int;
}
unsafe extern "C" fn isOnFinalAccelerationCalibrationCycle() -> bool {
    return calibratingA as libc::c_int == 1 as libc::c_int;
}
unsafe extern "C" fn isOnFirstAccelerationCalibrationCycle() -> bool {
    return calibratingA as libc::c_int == 400 as libc::c_int;
}
unsafe extern "C" fn performAcclerationCalibration(mut rollAndPitchTrims:
                                                       *mut rollAndPitchTrims_t) {
    static mut a: [int32_t; 3] = [0; 3];
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        // Reset a[axis] at start of calibration
        if isOnFirstAccelerationCalibrationCycle() {
            a[axis as usize] = 0 as libc::c_int
        }
        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis as usize] =
            (a[axis as usize] as libc::c_float + acc.accADC[axis as usize]) as
                int32_t;
        // Reset global variables to prevent other code from using un-calibrated data
        acc.accADC[axis as usize] = 0 as libc::c_int as libc::c_float;
        (*accelerationTrims).raw[axis as usize] = 0 as libc::c_int as int16_t;
        axis += 1
    }
    if isOnFinalAccelerationCalibrationCycle() {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        (*accelerationTrims).raw[X as libc::c_int as usize] =
            ((a[X as libc::c_int as usize] +
                  400 as libc::c_int / 2 as libc::c_int) / 400 as libc::c_int)
                as int16_t;
        (*accelerationTrims).raw[Y as libc::c_int as usize] =
            ((a[Y as libc::c_int as usize] +
                  400 as libc::c_int / 2 as libc::c_int) / 400 as libc::c_int)
                as int16_t;
        (*accelerationTrims).raw[Z as libc::c_int as usize] =
            ((a[Z as libc::c_int as usize] +
                  400 as libc::c_int / 2 as libc::c_int) / 400 as libc::c_int
                 - acc.dev.acc_1G as libc::c_int) as int16_t;
        resetRollAndPitchTrims(rollAndPitchTrims);
        saveConfigAndNotify();
    }
    calibratingA = calibratingA.wrapping_sub(1);
}
unsafe extern "C" fn performInflightAccelerationCalibration(mut rollAndPitchTrims:
                                                                *mut rollAndPitchTrims_t) {
    static mut b: [int32_t; 3] = [0; 3];
    static mut accZero_saved: [int16_t; 3] =
        [0 as libc::c_int as int16_t, 0 as libc::c_int as int16_t,
         0 as libc::c_int as int16_t];
    static mut angleTrim_saved: rollAndPitchTrims_t =
        rollAndPitchTrims_u{raw:
                                [0 as libc::c_int as int16_t,
                                 0 as libc::c_int as int16_t],};
    // Saving old zeropoints before measurement
    if InflightcalibratingA as libc::c_int == 50 as libc::c_int {
        accZero_saved[X as libc::c_int as usize] =
            (*accelerationTrims).raw[X as libc::c_int as usize];
        accZero_saved[Y as libc::c_int as usize] =
            (*accelerationTrims).raw[Y as libc::c_int as usize];
        accZero_saved[Z as libc::c_int as usize] =
            (*accelerationTrims).raw[Z as libc::c_int as usize];
        angleTrim_saved.values.roll = (*rollAndPitchTrims).values.roll;
        angleTrim_saved.values.pitch = (*rollAndPitchTrims).values.pitch
    }
    if InflightcalibratingA as libc::c_int > 0 as libc::c_int {
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            // Reset a[axis] at start of calibration
            if InflightcalibratingA as libc::c_int == 50 as libc::c_int {
                b[axis as usize] = 0 as libc::c_int
            }
            // Sum up 50 readings
            b[axis as usize] =
                (b[axis as usize] as libc::c_float +
                     acc.accADC[axis as usize]) as int32_t;
            // Clear global variables for next reading
            acc.accADC[axis as usize] = 0 as libc::c_int as libc::c_float;
            (*accelerationTrims).raw[axis as usize] =
                0 as libc::c_int as int16_t;
            axis += 1
        }
        // all values are measured
        if InflightcalibratingA as libc::c_int == 1 as libc::c_int {
            AccInflightCalibrationActive =
                0 as libc::c_int != 0; // indicate end of calibration
            AccInflightCalibrationMeasurementDone = 1 as libc::c_int != 0;
            beeper(BEEPER_ACC_CALIBRATION);
            // recover saved values to maintain current flight behaviour until new values are transferred
            (*accelerationTrims).raw[X as libc::c_int as usize] =
                accZero_saved[X as libc::c_int as usize];
            (*accelerationTrims).raw[Y as libc::c_int as usize] =
                accZero_saved[Y as libc::c_int as usize];
            (*accelerationTrims).raw[Z as libc::c_int as usize] =
                accZero_saved[Z as libc::c_int as usize];
            (*rollAndPitchTrims).values.roll = angleTrim_saved.values.roll;
            (*rollAndPitchTrims).values.pitch = angleTrim_saved.values.pitch
        }
        InflightcalibratingA = InflightcalibratingA.wrapping_sub(1)
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if AccInflightCalibrationSavetoEEProm {
        // the aircraft is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm =
            0 as libc::c_int != 0; // for nunchuck 200=1G
        (*accelerationTrims).raw[X as libc::c_int as usize] =
            (b[X as libc::c_int as usize] / 50 as libc::c_int) as int16_t;
        (*accelerationTrims).raw[Y as libc::c_int as usize] =
            (b[Y as libc::c_int as usize] / 50 as libc::c_int) as int16_t;
        (*accelerationTrims).raw[Z as libc::c_int as usize] =
            (b[Z as libc::c_int as usize] / 50 as libc::c_int -
                 acc.dev.acc_1G as libc::c_int) as int16_t;
        resetRollAndPitchTrims(rollAndPitchTrims);
        saveConfigAndNotify();
    };
}
unsafe extern "C" fn applyAccelerationTrims(mut accelerationTrims_0:
                                                *const flightDynamicsTrims_t) {
    acc.accADC[X as libc::c_int as usize] -=
        (*accelerationTrims_0).raw[X as libc::c_int as usize] as libc::c_int
            as libc::c_float;
    acc.accADC[Y as libc::c_int as usize] -=
        (*accelerationTrims_0).raw[Y as libc::c_int as usize] as libc::c_int
            as libc::c_float;
    acc.accADC[Z as libc::c_int as usize] -=
        (*accelerationTrims_0).raw[Z as libc::c_int as usize] as libc::c_int
            as libc::c_float;
}
#[no_mangle]
pub unsafe extern "C" fn accUpdate(mut currentTimeUs: timeUs_t,
                                   mut rollAndPitchTrims:
                                       *mut rollAndPitchTrims_t) {
    if !acc.dev.readFn.expect("non-null function pointer")(&mut acc.dev) {
        return
    }
    acc.isAccelUpdatedAtLeastOnce = 1 as libc::c_int != 0;
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        if debugMode as libc::c_int == DEBUG_ACCELEROMETER as libc::c_int {
            debug[axis as usize] = acc.dev.ADCRaw[axis as usize]
        }
        acc.accADC[axis as usize] =
            acc.dev.ADCRaw[axis as usize] as libc::c_float;
        axis += 1
    }
    if accLpfCutHz != 0 {
        let mut axis_0: libc::c_int = 0 as libc::c_int;
        while axis_0 < 3 as libc::c_int {
            acc.accADC[axis_0 as usize] =
                biquadFilterApply(&mut *accFilter.as_mut_ptr().offset(axis_0
                                                                          as
                                                                          isize),
                                  acc.accADC[axis_0 as usize]);
            axis_0 += 1
        }
    }
    alignSensors(acc.accADC.as_mut_ptr(), acc.dev.accAlign as uint8_t);
    if !accIsCalibrationComplete() {
        performAcclerationCalibration(rollAndPitchTrims);
    }
    if feature(FEATURE_INFLIGHT_ACC_CAL as libc::c_int as uint32_t) {
        performInflightAccelerationCalibration(rollAndPitchTrims);
    }
    applyAccelerationTrims(accelerationTrims);
    accumulatedMeasurementCount += 1;
    let mut axis_1: libc::c_int = 0 as libc::c_int;
    while axis_1 < 3 as libc::c_int {
        accumulatedMeasurements[axis_1 as usize] +=
            acc.accADC[axis_1 as usize];
        axis_1 += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn accGetAccumulationAverage(mut accumulationAverage:
                                                       *mut libc::c_float)
 -> bool {
    if accumulatedMeasurementCount > 0 as libc::c_int {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            *accumulationAverage.offset(axis as isize) =
                accumulatedMeasurements[axis as usize] /
                    accumulatedMeasurementCount as libc::c_float;
            accumulatedMeasurements[axis as usize] = 0.0f32;
            axis += 1
        }
        accumulatedMeasurementCount = 0 as libc::c_int;
        return 1 as libc::c_int != 0
    } else {
        let mut axis_0: libc::c_int = 0 as libc::c_int;
        while axis_0 < 3 as libc::c_int {
            *accumulationAverage.offset(axis_0 as isize) = 0.0f32;
            axis_0 += 1
        }
        return 0 as libc::c_int != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn setAccelerationTrims(mut accelerationTrimsToUse:
                                                  *mut flightDynamicsTrims_t) {
    accelerationTrims = accelerationTrimsToUse;
}
#[no_mangle]
pub unsafe extern "C" fn accInitFilters() {
    accLpfCutHz = (*accelerometerConfig()).acc_lpf_hz;
    if acc.accSamplingInterval != 0 {
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            biquadFilterInitLPF(&mut *accFilter.as_mut_ptr().offset(axis as
                                                                        isize),
                                accLpfCutHz as libc::c_float,
                                acc.accSamplingInterval);
            axis += 1
        }
    };
}
