use ::libc;
extern "C" {
    #[no_mangle]
    static debugModeNames: [*const libc::c_char; 44];
    #[no_mangle]
    static currentMeterSourceNames: [*const libc::c_char; 5];
    #[no_mangle]
    static voltageMeterSourceNames: [*const libc::c_char; 3];
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type pgn_t = uint16_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
}
pub type displayPort_t = displayPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
}
pub type ADCDevice = libc::c_int;
pub const ADCDEV_COUNT: ADCDevice = 3;
pub const ADCDEV_3: ADCDevice = 2;
pub const ADCDEV_2: ADCDevice = 1;
pub const ADCDEV_1: ADCDevice = 0;
pub const ADCINVALID: ADCDevice = -1;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub const THROTTLE: rc_alias = 3;
pub const YAW: rc_alias = 2;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
pub type C2RustUnnamed = libc::c_uint;
pub const DSHOT_CMD_MAX: C2RustUnnamed = 47;
pub const DSHOT_CMD_SILENT_MODE_ON_OFF: C2RustUnnamed = 31;
pub const DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF: C2RustUnnamed = 30;
pub const DSHOT_CMD_LED3_OFF: C2RustUnnamed = 29;
pub const DSHOT_CMD_LED2_OFF: C2RustUnnamed = 28;
pub const DSHOT_CMD_LED1_OFF: C2RustUnnamed = 27;
pub const DSHOT_CMD_LED0_OFF: C2RustUnnamed = 26;
pub const DSHOT_CMD_LED3_ON: C2RustUnnamed = 25;
pub const DSHOT_CMD_LED2_ON: C2RustUnnamed = 24;
pub const DSHOT_CMD_LED1_ON: C2RustUnnamed = 23;
pub const DSHOT_CMD_LED0_ON: C2RustUnnamed = 22;
pub const DSHOT_CMD_SPIN_DIRECTION_REVERSED: C2RustUnnamed = 21;
pub const DSHOT_CMD_SPIN_DIRECTION_NORMAL: C2RustUnnamed = 20;
pub const DSHOT_CMD_SAVE_SETTINGS: C2RustUnnamed = 12;
pub const DSHOT_CMD_SETTINGS_REQUEST: C2RustUnnamed = 11;
pub const DSHOT_CMD_3D_MODE_ON: C2RustUnnamed = 10;
pub const DSHOT_CMD_3D_MODE_OFF: C2RustUnnamed = 9;
pub const DSHOT_CMD_SPIN_DIRECTION_2: C2RustUnnamed = 8;
pub const DSHOT_CMD_SPIN_DIRECTION_1: C2RustUnnamed = 7;
pub const DSHOT_CMD_ESC_INFO: C2RustUnnamed = 6;
pub const DSHOT_CMD_BEACON5: C2RustUnnamed = 5;
pub const DSHOT_CMD_BEACON4: C2RustUnnamed = 4;
pub const DSHOT_CMD_BEACON3: C2RustUnnamed = 3;
pub const DSHOT_CMD_BEACON2: C2RustUnnamed = 2;
pub const DSHOT_CMD_BEACON1: C2RustUnnamed = 1;
pub const DSHOT_CMD_MOTOR_STOP: C2RustUnnamed = 0;
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
// These must be consecutive, see 'reversedSources'
pub type C2RustUnnamed_0 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_0 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_0 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_0 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_0 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_0 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_0 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_0 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_0 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_0 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_0 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_0 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_0 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_0 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_0 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_0 = 0;
pub type lookupTableIndex_e = libc::c_uint;
pub const LOOKUP_TABLE_COUNT: lookupTableIndex_e = 44;
pub const TABLE_RC_SMOOTHING_DERIVATIVE_TYPE: lookupTableIndex_e = 43;
pub const TABLE_RC_SMOOTHING_INPUT_TYPE: lookupTableIndex_e = 42;
pub const TABLE_RC_SMOOTHING_DEBUG: lookupTableIndex_e = 41;
pub const TABLE_RC_SMOOTHING_TYPE: lookupTableIndex_e = 40;
pub const TABLE_ACRO_TRAINER_DEBUG: lookupTableIndex_e = 39;
pub const TABLE_ITERM_RELAX_TYPE: lookupTableIndex_e = 38;
pub const TABLE_ITERM_RELAX: lookupTableIndex_e = 37;
pub const TABLE_VIDEO_SYSTEM: lookupTableIndex_e = 36;
pub const TABLE_THROTTLE_LIMIT_TYPE: lookupTableIndex_e = 35;
pub const TABLE_GYRO: lookupTableIndex_e = 34;
pub const TABLE_RGB_GRB: lookupTableIndex_e = 33;
pub const TABLE_OVERCLOCK: lookupTableIndex_e = 32;
pub const TABLE_RATES_TYPE: lookupTableIndex_e = 31;
pub const TABLE_GYRO_OVERFLOW_CHECK: lookupTableIndex_e = 30;
pub const TABLE_MAX7456_CLOCK: lookupTableIndex_e = 29;
pub const TABLE_BUS_TYPE: lookupTableIndex_e = 28;
pub const TABLE_CAMERA_CONTROL_MODE: lookupTableIndex_e = 27;
pub const TABLE_CRASH_RECOVERY: lookupTableIndex_e = 26;
pub const TABLE_FAILSAFE_SWITCH_MODE: lookupTableIndex_e = 25;
pub const TABLE_FAILSAFE: lookupTableIndex_e = 24;
pub const TABLE_ANTI_GRAVITY_MODE: lookupTableIndex_e = 23;
pub const TABLE_DTERM_LOWPASS_TYPE: lookupTableIndex_e = 22;
pub const TABLE_LOWPASS_TYPE: lookupTableIndex_e = 21;
pub const TABLE_RC_INTERPOLATION_CHANNELS: lookupTableIndex_e = 20;
pub const TABLE_RC_INTERPOLATION: lookupTableIndex_e = 19;
pub const TABLE_MOTOR_PWM_PROTOCOL: lookupTableIndex_e = 18;
pub const TABLE_DEBUG: lookupTableIndex_e = 17;
pub const TABLE_MAG_HARDWARE: lookupTableIndex_e = 16;
pub const TABLE_BARO_HARDWARE: lookupTableIndex_e = 15;
pub const TABLE_ACC_HARDWARE: lookupTableIndex_e = 14;
pub const TABLE_GYRO_32KHZ_HARDWARE_LPF: lookupTableIndex_e = 13;
pub const TABLE_GYRO_HARDWARE_LPF: lookupTableIndex_e = 12;
pub const TABLE_SERIAL_RX: lookupTableIndex_e = 11;
pub const TABLE_GIMBAL_MODE: lookupTableIndex_e = 10;
pub const TABLE_VOLTAGE_METER: lookupTableIndex_e = 9;
pub const TABLE_CURRENT_METER: lookupTableIndex_e = 8;
pub const TABLE_BLACKBOX_MODE: lookupTableIndex_e = 7;
pub const TABLE_BLACKBOX_DEVICE: lookupTableIndex_e = 6;
pub const TABLE_GPS_RESCUE: lookupTableIndex_e = 5;
pub const TABLE_GPS_SBAS_MODE: lookupTableIndex_e = 4;
pub const TABLE_GPS_PROVIDER: lookupTableIndex_e = 3;
pub const TABLE_ALIGNMENT: lookupTableIndex_e = 2;
pub const TABLE_UNIT: lookupTableIndex_e = 1;
pub const TABLE_OFF_ON: lookupTableIndex_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct lookupTableEntry_s {
    pub values: *const *const libc::c_char,
    pub valueCount: uint8_t,
}
pub type lookupTableEntry_t = lookupTableEntry_s;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const MODE_BITSET: C2RustUnnamed_1 = 96;
pub const MODE_ARRAY: C2RustUnnamed_1 = 64;
pub const MODE_LOOKUP: C2RustUnnamed_1 = 32;
pub const MODE_DIRECT: C2RustUnnamed_1 = 0;
pub const PROFILE_RATE_VALUE: C2RustUnnamed_1 = 16;
pub const PROFILE_VALUE: C2RustUnnamed_1 = 8;
pub const MASTER_VALUE: C2RustUnnamed_1 = 0;
pub const VAR_UINT32: C2RustUnnamed_1 = 4;
pub const VAR_INT16: C2RustUnnamed_1 = 3;
pub const VAR_UINT16: C2RustUnnamed_1 = 2;
pub const VAR_INT8: C2RustUnnamed_1 = 1;
pub const VAR_UINT8: C2RustUnnamed_1 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cliMinMaxConfig_s {
    pub min: int16_t,
    pub max: int16_t,
}
pub type cliMinMaxConfig_t = cliMinMaxConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cliLookupTableConfig_s {
    pub tableIndex: lookupTableIndex_e,
}
pub type cliLookupTableConfig_t = cliLookupTableConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cliArrayLengthConfig_s {
    pub length: uint8_t,
}
pub type cliArrayLengthConfig_t = cliArrayLengthConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union cliValueConfig_t {
    pub lookup: cliLookupTableConfig_t,
    pub minmax: cliMinMaxConfig_t,
    pub array: cliArrayLengthConfig_t,
    pub bitpos: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct clivalue_s {
    pub name: *const libc::c_char,
    pub type_0: uint8_t,
    pub config: cliValueConfig_t,
    pub pgn: pgn_t,
    pub offset: uint16_t,
}
pub type clivalue_t = clivalue_s;
pub const OSD_STAT_BLACKBOX_NUMBER: C2RustUnnamed_2 = 13;
pub const OSD_STAT_BLACKBOX: C2RustUnnamed_2 = 12;
pub const OSD_STAT_MAX_ALTITUDE: C2RustUnnamed_2 = 11;
pub const OSD_STAT_USED_MAH: C2RustUnnamed_2 = 10;
pub const OSD_STAT_MAX_CURRENT: C2RustUnnamed_2 = 9;
pub const OSD_STAT_MIN_RSSI: C2RustUnnamed_2 = 8;
pub const OSD_STAT_BATTERY: C2RustUnnamed_2 = 7;
pub const OSD_STAT_END_BATTERY: C2RustUnnamed_2 = 6;
pub const OSD_STAT_MIN_BATTERY: C2RustUnnamed_2 = 5;
pub const OSD_STAT_MAX_DISTANCE: C2RustUnnamed_2 = 4;
pub const OSD_STAT_MAX_SPEED: C2RustUnnamed_2 = 3;
pub const OSD_STAT_TIMER_2: C2RustUnnamed_2 = 2;
pub const OSD_STAT_TIMER_1: C2RustUnnamed_2 = 1;
pub const OSD_STAT_RTC_DATE_TIME: C2RustUnnamed_2 = 0;
pub const OSD_WARNING_CORE_TEMPERATURE: C2RustUnnamed_3 = 7;
pub const OSD_WARNING_ESC_FAIL: C2RustUnnamed_3 = 6;
pub const OSD_WARNING_CRASH_FLIP: C2RustUnnamed_3 = 5;
pub const OSD_WARNING_VISUAL_BEEPER: C2RustUnnamed_3 = 4;
pub const OSD_WARNING_BATTERY_CRITICAL: C2RustUnnamed_3 = 3;
pub const OSD_WARNING_BATTERY_WARNING: C2RustUnnamed_3 = 2;
pub const OSD_WARNING_BATTERY_NOT_FULL: C2RustUnnamed_3 = 1;
pub const OSD_WARNING_ARMING_DISABLE: C2RustUnnamed_3 = 0;
pub const FRSKY_VFAS_PRECISION_HIGH: C2RustUnnamed_4 = 1;
pub const FRSKY_VFAS_PRECISION_LOW: C2RustUnnamed_4 = 0;
pub const FRSKY_FORMAT_NMEA: C2RustUnnamed_5 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const OSD_STAT_COUNT: C2RustUnnamed_2 = 14;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const OSD_WARNING_COUNT: C2RustUnnamed_3 = 9;
pub const OSD_WARNING_RC_SMOOTHING: C2RustUnnamed_3 = 8;
pub type C2RustUnnamed_4 = libc::c_uint;
// see cliValueFlag_e
// 0 - 359
// 0 - 255
// 0 - 255
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
 * telemetry.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */
pub type C2RustUnnamed_5 = libc::c_uint;
pub const FRSKY_FORMAT_DMS: C2RustUnnamed_5 = 0;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_0 = INPUT_STABILIZED_ROLL;
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
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
// Sensor names (used in lookup tables for *_hardware settings and in status command output)
// sync with accelerationSensor_e
#[no_mangle]
pub static mut lookupTableAccHardware: [*const libc::c_char; 17] =
    [b"AUTO\x00" as *const u8 as *const libc::c_char,
     b"NONE\x00" as *const u8 as *const libc::c_char,
     b"ADXL345\x00" as *const u8 as *const libc::c_char,
     b"MPU6050\x00" as *const u8 as *const libc::c_char,
     b"MMA8452\x00" as *const u8 as *const libc::c_char,
     b"BMA280\x00" as *const u8 as *const libc::c_char,
     b"LSM303DLHC\x00" as *const u8 as *const libc::c_char,
     b"MPU6000\x00" as *const u8 as *const libc::c_char,
     b"MPU6500\x00" as *const u8 as *const libc::c_char,
     b"MPU9250\x00" as *const u8 as *const libc::c_char,
     b"ICM20601\x00" as *const u8 as *const libc::c_char,
     b"ICM20602\x00" as *const u8 as *const libc::c_char,
     b"ICM20608G\x00" as *const u8 as *const libc::c_char,
     b"ICM20649\x00" as *const u8 as *const libc::c_char,
     b"ICM20689\x00" as *const u8 as *const libc::c_char,
     b"BMI160\x00" as *const u8 as *const libc::c_char,
     b"FAKE\x00" as *const u8 as *const libc::c_char];
// sync with gyroSensor_e
#[no_mangle]
pub static mut lookupTableGyroHardware: [*const libc::c_char; 16] =
    [b"AUTO\x00" as *const u8 as *const libc::c_char,
     b"NONE\x00" as *const u8 as *const libc::c_char,
     b"MPU6050\x00" as *const u8 as *const libc::c_char,
     b"L3G4200D\x00" as *const u8 as *const libc::c_char,
     b"MPU3050\x00" as *const u8 as *const libc::c_char,
     b"L3GD20\x00" as *const u8 as *const libc::c_char,
     b"MPU6000\x00" as *const u8 as *const libc::c_char,
     b"MPU6500\x00" as *const u8 as *const libc::c_char,
     b"MPU9250\x00" as *const u8 as *const libc::c_char,
     b"ICM20601\x00" as *const u8 as *const libc::c_char,
     b"ICM20602\x00" as *const u8 as *const libc::c_char,
     b"ICM20608G\x00" as *const u8 as *const libc::c_char,
     b"ICM20649\x00" as *const u8 as *const libc::c_char,
     b"ICM20689\x00" as *const u8 as *const libc::c_char,
     b"BMI160\x00" as *const u8 as *const libc::c_char,
     b"FAKE\x00" as *const u8 as *const libc::c_char];
// sync with baroSensor_e
#[no_mangle]
pub static mut lookupTableBaroHardware: [*const libc::c_char; 7] =
    [b"AUTO\x00" as *const u8 as *const libc::c_char,
     b"NONE\x00" as *const u8 as *const libc::c_char,
     b"BMP085\x00" as *const u8 as *const libc::c_char,
     b"MS5611\x00" as *const u8 as *const libc::c_char,
     b"BMP280\x00" as *const u8 as *const libc::c_char,
     b"LPS\x00" as *const u8 as *const libc::c_char,
     b"QMP6988\x00" as *const u8 as *const libc::c_char];
// sync with magSensor_e
#[no_mangle]
pub static mut lookupTableMagHardware: [*const libc::c_char; 6] =
    [b"AUTO\x00" as *const u8 as *const libc::c_char,
     b"NONE\x00" as *const u8 as *const libc::c_char,
     b"HMC5883\x00" as *const u8 as *const libc::c_char,
     b"AK8975\x00" as *const u8 as *const libc::c_char,
     b"AK8963\x00" as *const u8 as *const libc::c_char,
     b"QMC5883\x00" as *const u8 as *const libc::c_char];
#[no_mangle]
pub static mut lookupTableRangefinderHardware: [*const libc::c_char; 4] =
    [b"NONE\x00" as *const u8 as *const libc::c_char,
     b"HCSR04\x00" as *const u8 as *const libc::c_char,
     b"TFMINI\x00" as *const u8 as *const libc::c_char,
     b"TF02\x00" as *const u8 as *const libc::c_char];
static mut lookupTableOffOn: [*const libc::c_char; 2] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"ON\x00" as *const u8 as *const libc::c_char];
static mut lookupTableCrashRecovery: [*const libc::c_char; 3] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"ON\x00" as *const u8 as *const libc::c_char,
     b"BEEP\x00" as *const u8 as *const libc::c_char];
static mut lookupTableUnit: [*const libc::c_char; 2] =
    [b"IMPERIAL\x00" as *const u8 as *const libc::c_char,
     b"METRIC\x00" as *const u8 as *const libc::c_char];
static mut lookupTableAlignment: [*const libc::c_char; 9] =
    [b"DEFAULT\x00" as *const u8 as *const libc::c_char,
     b"CW0\x00" as *const u8 as *const libc::c_char,
     b"CW90\x00" as *const u8 as *const libc::c_char,
     b"CW180\x00" as *const u8 as *const libc::c_char,
     b"CW270\x00" as *const u8 as *const libc::c_char,
     b"CW0FLIP\x00" as *const u8 as *const libc::c_char,
     b"CW90FLIP\x00" as *const u8 as *const libc::c_char,
     b"CW180FLIP\x00" as *const u8 as *const libc::c_char,
     b"CW270FLIP\x00" as *const u8 as *const libc::c_char];
static mut lookupTableGyro: [*const libc::c_char; 3] =
    [b"FIRST\x00" as *const u8 as *const libc::c_char,
     b"SECOND\x00" as *const u8 as *const libc::c_char,
     b"BOTH\x00" as *const u8 as *const libc::c_char];
static mut lookupTableGPSProvider: [*const libc::c_char; 2] =
    [b"NMEA\x00" as *const u8 as *const libc::c_char,
     b"UBLOX\x00" as *const u8 as *const libc::c_char];
static mut lookupTableGPSSBASMode: [*const libc::c_char; 5] =
    [b"AUTO\x00" as *const u8 as *const libc::c_char,
     b"EGNOS\x00" as *const u8 as *const libc::c_char,
     b"WAAS\x00" as *const u8 as *const libc::c_char,
     b"MSAS\x00" as *const u8 as *const libc::c_char,
     b"GAGAN\x00" as *const u8 as *const libc::c_char];
static mut lookupTableGimbalMode: [*const libc::c_char; 2] =
    [b"NORMAL\x00" as *const u8 as *const libc::c_char,
     b"MIXTILT\x00" as *const u8 as *const libc::c_char];
static mut lookupTableBlackboxDevice: [*const libc::c_char; 4] =
    [b"NONE\x00" as *const u8 as *const libc::c_char,
     b"SPIFLASH\x00" as *const u8 as *const libc::c_char,
     b"SDCARD\x00" as *const u8 as *const libc::c_char,
     b"SERIAL\x00" as *const u8 as *const libc::c_char];
static mut lookupTableBlackboxMode: [*const libc::c_char; 3] =
    [b"NORMAL\x00" as *const u8 as *const libc::c_char,
     b"MOTOR_TEST\x00" as *const u8 as *const libc::c_char,
     b"ALWAYS\x00" as *const u8 as *const libc::c_char];
static mut lookupTableSerialRX: [*const libc::c_char; 13] =
    [b"SPEK1024\x00" as *const u8 as *const libc::c_char,
     b"SPEK2048\x00" as *const u8 as *const libc::c_char,
     b"SBUS\x00" as *const u8 as *const libc::c_char,
     b"SUMD\x00" as *const u8 as *const libc::c_char,
     b"SUMH\x00" as *const u8 as *const libc::c_char,
     b"XB-B\x00" as *const u8 as *const libc::c_char,
     b"XB-B-RJ01\x00" as *const u8 as *const libc::c_char,
     b"IBUS\x00" as *const u8 as *const libc::c_char,
     b"JETIEXBUS\x00" as *const u8 as *const libc::c_char,
     b"CRSF\x00" as *const u8 as *const libc::c_char,
     b"SRXL\x00" as *const u8 as *const libc::c_char,
     b"CUSTOM\x00" as *const u8 as *const libc::c_char,
     b"FPORT\x00" as *const u8 as *const libc::c_char];
static mut lookupTableGyroHardwareLpf: [*const libc::c_char; 3] =
    [b"NORMAL\x00" as *const u8 as *const libc::c_char,
     b"EXPERIMENTAL\x00" as *const u8 as *const libc::c_char,
     b"1KHZ_SAMPLING\x00" as *const u8 as *const libc::c_char];
static mut lookupTableGyro32khzHardwareLpf: [*const libc::c_char; 2] =
    [b"NORMAL\x00" as *const u8 as *const libc::c_char,
     b"EXPERIMENTAL\x00" as *const u8 as *const libc::c_char];
static mut lookupTableCameraControlMode: [*const libc::c_char; 3] =
    [b"HARDWARE_PWM\x00" as *const u8 as *const libc::c_char,
     b"SOFTWARE_PWM\x00" as *const u8 as *const libc::c_char,
     b"DAC\x00" as *const u8 as *const libc::c_char];
static mut lookupTablePwmProtocol: [*const libc::c_char; 10] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"ONESHOT125\x00" as *const u8 as *const libc::c_char,
     b"ONESHOT42\x00" as *const u8 as *const libc::c_char,
     b"MULTISHOT\x00" as *const u8 as *const libc::c_char,
     b"BRUSHED\x00" as *const u8 as *const libc::c_char,
     b"DSHOT150\x00" as *const u8 as *const libc::c_char,
     b"DSHOT300\x00" as *const u8 as *const libc::c_char,
     b"DSHOT600\x00" as *const u8 as *const libc::c_char,
     b"DSHOT1200\x00" as *const u8 as *const libc::c_char,
     b"PROSHOT1000\x00" as *const u8 as *const libc::c_char];
static mut lookupTableRcInterpolation: [*const libc::c_char; 4] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"PRESET\x00" as *const u8 as *const libc::c_char,
     b"AUTO\x00" as *const u8 as *const libc::c_char,
     b"MANUAL\x00" as *const u8 as *const libc::c_char];
static mut lookupTableRcInterpolationChannels: [*const libc::c_char; 5] =
    [b"RP\x00" as *const u8 as *const libc::c_char,
     b"RPY\x00" as *const u8 as *const libc::c_char,
     b"RPYT\x00" as *const u8 as *const libc::c_char,
     b"T\x00" as *const u8 as *const libc::c_char,
     b"RPT\x00" as *const u8 as *const libc::c_char];
static mut lookupTableLowpassType: [*const libc::c_char; 2] =
    [b"PT1\x00" as *const u8 as *const libc::c_char,
     b"BIQUAD\x00" as *const u8 as *const libc::c_char];
static mut lookupTableDtermLowpassType: [*const libc::c_char; 2] =
    [b"PT1\x00" as *const u8 as *const libc::c_char,
     b"BIQUAD\x00" as *const u8 as *const libc::c_char];
static mut lookupTableAntiGravityMode: [*const libc::c_char; 2] =
    [b"SMOOTH\x00" as *const u8 as *const libc::c_char,
     b"STEP\x00" as *const u8 as *const libc::c_char];
static mut lookupTableFailsafe: [*const libc::c_char; 3] =
    [b"AUTO-LAND\x00" as *const u8 as *const libc::c_char,
     b"DROP\x00" as *const u8 as *const libc::c_char,
     b"GPS-RESCUE\x00" as *const u8 as *const libc::c_char];
static mut lookupTableFailsafeSwitchMode: [*const libc::c_char; 3] =
    [b"STAGE1\x00" as *const u8 as *const libc::c_char,
     b"KILL\x00" as *const u8 as *const libc::c_char,
     b"STAGE2\x00" as *const u8 as *const libc::c_char];
static mut lookupTableBusType: [*const libc::c_char; 4] =
    [b"NONE\x00" as *const u8 as *const libc::c_char,
     b"I2C\x00" as *const u8 as *const libc::c_char,
     b"SPI\x00" as *const u8 as *const libc::c_char,
     b"SLAVE\x00" as *const u8 as *const libc::c_char];
static mut lookupTableMax7456Clock: [*const libc::c_char; 3] =
    [b"HALF\x00" as *const u8 as *const libc::c_char,
     b"DEFAULT\x00" as *const u8 as *const libc::c_char,
     b"FULL\x00" as *const u8 as *const libc::c_char];
static mut lookupTableGyroOverflowCheck: [*const libc::c_char; 3] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"YAW\x00" as *const u8 as *const libc::c_char,
     b"ALL\x00" as *const u8 as *const libc::c_char];
static mut lookupTableRatesType: [*const libc::c_char; 2] =
    [b"BETAFLIGHT\x00" as *const u8 as *const libc::c_char,
     b"RACEFLIGHT\x00" as *const u8 as *const libc::c_char];
static mut lookupOverclock: [*const libc::c_char; 2] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"240MHZ\x00" as *const u8 as *const libc::c_char];
static mut lookupLedStripFormatRGB: [*const libc::c_char; 2] =
    [b"GRB\x00" as *const u8 as *const libc::c_char,
     b"RGB\x00" as *const u8 as *const libc::c_char];
static mut lookupTableThrottleLimitType: [*const libc::c_char; 3] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"SCALE\x00" as *const u8 as *const libc::c_char,
     b"CLIP\x00" as *const u8 as *const libc::c_char];
static mut lookupTableRescueSanityType: [*const libc::c_char; 3] =
    [b"RESCUE_SANITY_OFF\x00" as *const u8 as *const libc::c_char,
     b"RESCUE_SANITY_ON\x00" as *const u8 as *const libc::c_char,
     b"RESCUE_SANITY_FS_ONLY\x00" as *const u8 as *const libc::c_char];
static mut lookupTableVideoSystem: [*const libc::c_char; 3] =
    [b"AUTO\x00" as *const u8 as *const libc::c_char,
     b"PAL\x00" as *const u8 as *const libc::c_char,
     b"NTSC\x00" as *const u8 as *const libc::c_char];
// USE_MAX7456
static mut lookupTableItermRelax: [*const libc::c_char; 5] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"RP\x00" as *const u8 as *const libc::c_char,
     b"RPY\x00" as *const u8 as *const libc::c_char,
     b"RP_INC\x00" as *const u8 as *const libc::c_char,
     b"RPY_INC\x00" as *const u8 as *const libc::c_char];
static mut lookupTableItermRelaxType: [*const libc::c_char; 2] =
    [b"GYRO\x00" as *const u8 as *const libc::c_char,
     b"SETPOINT\x00" as *const u8 as *const libc::c_char];
static mut lookupTableAcroTrainerDebug: [*const libc::c_char; 2] =
    [b"ROLL\x00" as *const u8 as *const libc::c_char,
     b"PITCH\x00" as *const u8 as *const libc::c_char];
// USE_ACRO_TRAINER
static mut lookupTableRcSmoothingType: [*const libc::c_char; 2] =
    [b"INTERPOLATION\x00" as *const u8 as *const libc::c_char,
     b"FILTER\x00" as *const u8 as *const libc::c_char];
static mut lookupTableRcSmoothingDebug: [*const libc::c_char; 4] =
    [b"ROLL\x00" as *const u8 as *const libc::c_char,
     b"PITCH\x00" as *const u8 as *const libc::c_char,
     b"YAW\x00" as *const u8 as *const libc::c_char,
     b"THROTTLE\x00" as *const u8 as *const libc::c_char];
static mut lookupTableRcSmoothingInputType: [*const libc::c_char; 2] =
    [b"PT1\x00" as *const u8 as *const libc::c_char,
     b"BIQUAD\x00" as *const u8 as *const libc::c_char];
static mut lookupTableRcSmoothingDerivativeType: [*const libc::c_char; 3] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"PT1\x00" as *const u8 as *const libc::c_char,
     b"BIQUAD\x00" as *const u8 as *const libc::c_char];
// USE_RC_SMOOTHING_FILTER
// Initialized in run_static_initializers
#[no_mangle]
pub static mut lookupTables: [lookupTableEntry_t; 44] =
    [lookupTableEntry_t{values: 0 as *const *const libc::c_char,
                        valueCount: 0,}; 44];
#[no_mangle]
pub static mut valueTable: [clivalue_t; 383] =
    [{
         let mut init =
             clivalue_s{name:
                            b"align_gyro\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_ALIGNMENT,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_hardware_lpf\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_GYRO_HARDWARE_LPF,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_32khz_hardware_lpf\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_GYRO_32KHZ_HARDWARE_LPF,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_sync_denom\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_lowpass_type\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_LOWPASS_TYPE,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 23 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_lowpass_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_lowpass2_type\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_LOWPASS_TYPE,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 24 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_lowpass2_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_notch1_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_notch1_cutoff\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 14 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_notch2_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_notch2_cutoff\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_calib_duration\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               3000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 28 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_calib_noise_limit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_offset_yaw\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(1000
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 20 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_overflow_detect\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_GYRO_OVERFLOW_CHECK,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 22 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_spin_recovery\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 25 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_spin_threshold\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1950
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 26 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_use_32khz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_to_use\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_GYRO,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 7 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dyn_notch_quality\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               70
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 30 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dyn_notch_width_percent\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               99
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10 as libc::c_int as pgn_t,
                        offset: 31 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"align_acc\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_ALIGNMENT,};
                                                     init
                                                 },},
                        pgn: 35 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acc_hardware\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_ACC_HARDWARE,};
                                                     init
                                                 },},
                        pgn: 35 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acc_lpf_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               400
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 35 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acc_trim_pitch\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(300
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               300
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 35 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acc_trim_roll\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(300
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               300
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 35 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acc_calibration\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_ARRAY as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{array:
                                                 {
                                                     let mut init =
                                                         cliArrayLengthConfig_s{length:
                                                                                    3
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 35 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"align_mag\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_ALIGNMENT,};
                                                     init
                                                 },},
                        pgn: 40 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"mag_bustype\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_BUS_TYPE,};
                                                     init
                                                 },},
                        pgn: 40 as libc::c_int as pgn_t,
                        offset: 9 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"mag_i2c_device\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               4
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 40 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"mag_i2c_address\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               119
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 40 as libc::c_int as pgn_t,
                        offset: 11 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"mag_spi_device\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               4
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 40 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"mag_hardware\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_MAG_HARDWARE,};
                                                     init
                                                 },},
                        pgn: 40 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"mag_declination\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(18000
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               18000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 40 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"mag_calibration\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_ARRAY as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{array:
                                                 {
                                                     let mut init =
                                                         cliArrayLengthConfig_s{length:
                                                                                    3
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 40 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_bustype\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_BUS_TYPE,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_spi_device\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_i2c_device\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_i2c_address\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               119
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_hardware\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_BARO_HARDWARE,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 5 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_tab_size\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               48
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_noise_lpf\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_cf_vel\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"baro_cf_alt\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"mid_rc\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1700
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"min_check\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 20 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"max_check\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 22 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rssi_channel\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               18
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 15 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rssi_src_frame_errors\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 35 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rssi_scale\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rssi_offset\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(100
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 36 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rssi_invert\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 17 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_interp\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_RC_INTERPOLATION,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 24 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_interp_ch\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_RC_INTERPOLATION_CHANNELS,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 25 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_interp_int\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 26 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_smoothing_type\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_RC_SMOOTHING_TYPE,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 37 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_smoothing_input_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 38 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_smoothing_derivative_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 39 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_smoothing_debug_axis\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_RC_SMOOTHING_DEBUG,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 40 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_smoothing_input_type\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_RC_SMOOTHING_INPUT_TYPE,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 41 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rc_smoothing_derivative_type\x00" as *const u8
                                as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_RC_SMOOTHING_DERIVATIVE_TYPE,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 42 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"fpv_mix_degrees\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 27 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"max_aux_channels\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (18
                                                                                    as
                                                                                    libc::c_int
                                                                                    -
                                                                                    4
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 34 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"serialrx_provider\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_SERIAL_RX,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"serialrx_inverted\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 9 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"spektrum_sat_bind\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 13 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"spektrum_sat_bind_autoreset\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 14 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"airmode_start_throttle_percent\x00" as *const u8
                                as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 28 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rx_min_usec\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 30 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rx_max_usec\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 32 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"serialrx_halfduplex\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 24 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"adc_device\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               ADCDEV_COUNT
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 510 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"input_filtering_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 508 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"blackbox_p_ratio\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32767
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 5 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"blackbox_device\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_BLACKBOX_DEVICE,};
                                                     init
                                                 },},
                        pgn: 5 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"blackbox_record_acc\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 5 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"blackbox_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_BLACKBOX_MODE,};
                                                     init
                                                 },},
                        pgn: 5 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"min_throttle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"max_throttle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"min_command\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 20 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dshot_idle_value\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 14 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dshot_burst\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 5 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"use_unsynced_pwm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"motor_pwm_protocol\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_MOTOR_PWM_PROTOCOL,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"motor_pwm_rate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"motor_pwm_inversion\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"motor_poles\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               4
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6 as libc::c_int as pgn_t,
                        offset: 22 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"thr_corr_value\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               150
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 39 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"thr_corr_angle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               900
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 39 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"failsafe_delay\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 1 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"failsafe_off_delay\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 1 as libc::c_int as pgn_t,
                        offset: 5 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"failsafe_throttle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 1 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"failsafe_switch_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_FAILSAFE_SWITCH_MODE,};
                                                     init
                                                 },},
                        pgn: 1 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"failsafe_throttle_low_delay\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               300
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 1 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"failsafe_procedure\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_FAILSAFE,};
                                                     init
                                                 },},
                        pgn: 1 as libc::c_int as pgn_t,
                        offset: 7 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"align_board_roll\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(180
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               360
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 2 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"align_board_pitch\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(180
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               360
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 2 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"align_board_yaw\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(180
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               360
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 2 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gimbal_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_GIMBAL_MODE,};
                                                     init
                                                 },},
                        pgn: 3 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"bat_capacity\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               20000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_max_cell_voltage\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_full_cell_voltage\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 22 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_min_cell_voltage\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_warning_cell_voltage\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_hysteresis\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 21 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"current_meter\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_CURRENT_METER,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"battery_meter\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_VOLTAGE_METER,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_detect_cell_voltage\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"use_vbat_alerts\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"use_cbat_alerts\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 19 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"cbat_alert_percent\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 20 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_cutoff_percent\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_scale\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 258 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_divider\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 258 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_multiplier\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 258 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"ibata_scale\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(16000
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 256 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"ibata_offset\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(32000
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 256 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"ibatv_scale\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(16000
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 257 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"ibatv_offset\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 257 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"beeper_inversion\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 503 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"beeper_od\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 503 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"beeper_frequency\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 503 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"beeper_dshot_beacon_tone\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               DSHOT_CMD_BEACON5
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 502 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_motors_reversed\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 20 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crashflip_motor_percent\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 20 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"3d_deadband_low\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (1000
                                                                                    as
                                                                                    libc::c_int
                                                                                    +
                                                                                    (2000
                                                                                         as
                                                                                         libc::c_int
                                                                                         -
                                                                                         1000
                                                                                             as
                                                                                             libc::c_int)
                                                                                        /
                                                                                        2
                                                                                            as
                                                                                            libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"3d_deadband_high\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               (1000
                                                                                    as
                                                                                    libc::c_int
                                                                                    +
                                                                                    (2000
                                                                                         as
                                                                                         libc::c_int
                                                                                         -
                                                                                         1000
                                                                                             as
                                                                                             libc::c_int)
                                                                                        /
                                                                                        2
                                                                                            as
                                                                                            libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"3d_neutral\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"3d_deadband_throttle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"3d_limit_low\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (1000
                                                                                    as
                                                                                    libc::c_int
                                                                                    +
                                                                                    (2000
                                                                                         as
                                                                                         libc::c_int
                                                                                         -
                                                                                         1000
                                                                                             as
                                                                                             libc::c_int)
                                                                                        /
                                                                                        2
                                                                                            as
                                                                                            libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"3d_limit_high\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               (1000
                                                                                    as
                                                                                    libc::c_int
                                                                                    +
                                                                                    (2000
                                                                                         as
                                                                                         libc::c_int
                                                                                         -
                                                                                         1000
                                                                                             as
                                                                                             libc::c_int)
                                                                                        /
                                                                                        2
                                                                                            as
                                                                                            libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"3d_switched_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 26 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"servo_center_pulse\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 52 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"servo_pwm_rate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               498
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 52 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"servo_lowpass_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               400
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 52 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"tri_unarmed_servo\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 52 as libc::c_int as pgn_t,
                        offset: 14 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"channel_forwarding_start\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               AUX1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               18
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 52 as libc::c_int as pgn_t,
                        offset: 15 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"thr_mid\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"thr_expo\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rates_type\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_RATES_TYPE,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"roll_rc_rate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pitch_rc_rate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_rc_rate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 5 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"roll_expo\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pitch_expo\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 7 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_expo\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"roll_srate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 9 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pitch_srate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_srate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 11 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"tpa_rate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"tpa_breakpoint\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               750
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 14 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"throttle_limit_type\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_THROTTLE_LIMIT_TYPE,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"throttle_limit_percent\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_RATE_VALUE as libc::c_int) as
                                uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               25
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12 as libc::c_int as pgn_t,
                        offset: 17 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"reboot_character\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               48
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               126
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 13 as libc::c_int as pgn_t,
                        offset: 86 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"serial_update_rate_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 13 as libc::c_int as pgn_t,
                        offset: 84 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"accxy_deadband\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"accz_deadband\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22 as libc::c_int as pgn_t,
                        offset: 7 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acc_unarmedcal\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 22 as libc::c_int as pgn_t,
                        offset: 5 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"imu_dcm_kp\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"imu_dcm_ki\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"small_angle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               180
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"auto_disarm_delay\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               60
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 16 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gyro_cal_on_first_arm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 16 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_provider\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_GPS_PROVIDER,};
                                                     init
                                                 },},
                        pgn: 30 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_sbas_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_GPS_SBAS_MODE,};
                                                     init
                                                 },},
                        pgn: 30 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_auto_config\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 30 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_auto_baud\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 30 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_ublox_use_galileo\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 30 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_angle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_initial_alt\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               20
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_descent_dist\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               30
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_ground_speed\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               30
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               3000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_throttle_p\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_throttle_i\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_throttle_d\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_velocity_p\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 22 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_velocity_i\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 24 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_velocity_d\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 26 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_yaw_p\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 14 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_throttle_min\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_throttle_max\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_throttle_hover\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 20 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_sanity_checks\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_GPS_RESCUE,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 32 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"gps_rescue_min_sats\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 55 as libc::c_int as pgn_t,
                        offset: 28 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"deadband\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 25 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_deadband\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 25 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_control_reversed\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 25 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pid_process_denom\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 504 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"runaway_takeoff_prevention\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 504 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"runaway_takeoff_deactivate_delay\x00" as
                                *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 504 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"runaway_takeoff_deactivate_throttle_percent\x00"
                                as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 504 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dterm_lowpass_type\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_DTERM_LOWPASS_TYPE,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 38 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dterm_lowpass_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dterm_lowpass2_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 76 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dterm_notch_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dterm_notch_cutoff\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vbat_pid_gain\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 70 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pid_at_min_throttle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 44 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"anti_gravity_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_ANTI_GRAVITY_MODE,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 48 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"anti_gravity_threshold\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               20
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 50 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"anti_gravity_gain\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               30000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 52 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"feedforward_transition\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 71 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acc_limit_yaw\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 54 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acc_limit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 56 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_dthreshold\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 58 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_gthreshold\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 60 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_setpoint_threshold\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 62 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_time\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 64 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_delay\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 66 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_recovery_angle\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               30
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 68 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_recovery_rate\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 69 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_limit_yaw\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 72 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"crash_recovery\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_CRASH_RECOVERY,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 78 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"iterm_rotation\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 81 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"smart_feedforward\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 82 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"iterm_relax\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_ITERM_RELAX,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 85 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"iterm_relax_type\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_ITERM_RELAX_TYPE,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 83 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"iterm_relax_cutoff\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 84 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"iterm_windup\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               30
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 39 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"iterm_limit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 74 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pidsum_limit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 40 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pidsum_limit_yaw\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 42 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"yaw_lowpass_hz\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"throttle_boost\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 79 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"throttle_boost_cutoff\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               5
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 80 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acro_trainer_angle_limit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               80
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 86 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acro_trainer_lookahead_ms\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 90 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acro_trainer_debug_axis\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_ACRO_TRAINER_DEBUG,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 87 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"acro_trainer_gain\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               25
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 88 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"p_pitch\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 14 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"i_pitch\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 15 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"d_pitch\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"f_pitch\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"p_roll\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"i_roll\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 9 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"d_roll\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"f_roll\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"p_yaw\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 20 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"i_yaw\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 21 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"d_yaw\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 22 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"f_yaw\x00" as *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 24 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"p_level\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 26 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"i_level\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 27 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"d_level\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 28 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"level_limit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               90
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 45 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"horizon_tilt_effect\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               250
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 46 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"horizon_tilt_expert_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 47 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"abs_control_gain\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               20
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 92 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"abs_control_limit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 93 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"abs_control_error_limit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 PROFILE_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               45
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14 as libc::c_int as pgn_t,
                        offset: 94 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"tlm_inverted\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"tlm_halfduplex\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 5 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"frsky_default_lat\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(9000
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               9000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"frsky_default_long\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(18000
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               18000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"frsky_gps_format\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               FRSKY_FORMAT_NMEA
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"frsky_unit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_UNIT,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"frsky_vfas_precision\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               FRSKY_VFAS_PRECISION_LOW
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               FRSKY_VFAS_PRECISION_HIGH
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"hott_alarm_int\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               120
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 17 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pid_in_tlm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"report_cell_voltage\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 19 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"ibus_sensor\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_ARRAY as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{array:
                                                 {
                                                     let mut init =
                                                         cliArrayLengthConfig_s{length:
                                                                                    15
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 20 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"smartport_use_extra_sensors\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 31 as libc::c_int as pgn_t,
                        offset: 35 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"ledstrip_visual_beeper\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 27 as libc::c_int as pgn_t,
                        offset: 239 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"ledstrip_grb_rgb\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_RGB_GRB,};
                                                     init
                                                 },},
                        pgn: 27 as libc::c_int as pgn_t,
                        offset: 244 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"sdcard_dma\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 511 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_units\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_UNIT,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 92 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warn_arming_disable\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_WARNING_ARMING_DISABLE as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 100 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warn_batt_not_full\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_WARNING_BATTERY_NOT_FULL
                                                     as libc::c_int as
                                                     uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 100 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warn_batt_warning\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_WARNING_BATTERY_WARNING
                                                     as libc::c_int as
                                                     uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 100 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warn_batt_critical\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_WARNING_BATTERY_CRITICAL
                                                     as libc::c_int as
                                                     uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 100 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warn_visual_beeper\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_WARNING_VISUAL_BEEPER as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 100 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warn_crash_flip\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_WARNING_CRASH_FLIP as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 100 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warn_esc_fail\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_WARNING_ESC_FAIL as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 100 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warn_core_temp\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_WARNING_CORE_TEMPERATURE
                                                     as libc::c_int as
                                                     uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 100 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_rssi_alarm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 90 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_cap_alarm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               20000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 86 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_alt_alarm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               10000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 88 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_esc_temp_alarm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(128
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               127
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 108 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_esc_rpm_alarm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(1
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32767
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 110 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_esc_current_alarm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(1
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32767
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 112 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_core_temp_alarm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 114 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_ah_max_pit\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               90
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 102 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_ah_max_rol\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               90
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 103 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_tim1\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32767
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 96 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_tim2\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32767
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 98 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_vbat_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_rssi_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_tim_1_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 10 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_tim_2_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 12 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_remaining_time_estimate_pos\x00" as
                                *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 74 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_flymode_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 14 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_anti_gravity_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 82 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_g_force_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 84 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_throttle_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 18 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_vtx_channel_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 20 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_crosshairs_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_ah_sbar_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_ah_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_current_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 22 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_mah_drawn_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 24 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_craft_name_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 16 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_gps_speed_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 26 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_gps_lon_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 46 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_gps_lat_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 48 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_gps_sats_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 28 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_home_dir_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 60 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_home_dist_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 62 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_compass_bar_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 68 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_altitude_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 30 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_pid_roll_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 32 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_pid_pitch_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 34 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_pid_yaw_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 36 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_debug_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 50 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_power_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 38 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_pidrate_profile_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 40 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_warnings_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 42 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_avg_cell_voltage_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 44 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_pit_ang_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 52 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_rol_ang_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 54 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_battery_usage_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 56 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_disarmed_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 58 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_nheading_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 64 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_nvario_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 66 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_esc_tmp_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 70 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_esc_rpm_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 72 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_rtc_date_time_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 76 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_adjustment_range_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 78 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_core_temp_pos\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (0x800
                                                                                    as
                                                                                    libc::c_int
                                                                                    |
                                                                                    0x3ff
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 80 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_rtc_date_time\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_RTC_DATE_TIME as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_tim_1\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_TIMER_1 as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_tim_2\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_TIMER_2 as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_max_spd\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_MAX_SPEED as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_max_dist\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_MAX_DISTANCE as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_min_batt\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_MIN_BATTERY as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_endbatt\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_END_BATTERY as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_battery\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_BATTERY as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_min_rssi\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_MIN_RSSI as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_max_curr\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_MAX_CURRENT as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_used_mah\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_USED_MAH as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_max_alt\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_MAX_ALTITUDE as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_bbox\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_BLACKBOX as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"osd_stat_bb_no\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_BITSET as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{bitpos:
                                                 OSD_STAT_BLACKBOX_NUMBER as
                                                     libc::c_int as uint8_t,},
                        pgn: 501 as libc::c_int as pgn_t,
                        offset: 104 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"task_statistics\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 18 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"debug_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_DEBUG,};
                                                     init
                                                 },},
                        pgn: 18 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rate_6pos_switch\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 18 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"cpu_overclock\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OVERCLOCK,};
                                                     init
                                                 },},
                        pgn: 18 as libc::c_int as pgn_t,
                        offset: 5 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pwr_on_arm_grace\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               30
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 18 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vtx_band\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 259 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vtx_channel\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               8
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 259 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vtx_power\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (5
                                                                                    as
                                                                                    libc::c_int
                                                                                    -
                                                                                    1
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 259 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vtx_low_power_disarm\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 259 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vtx_freq\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5999
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 259 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vtx_pit_mode_freq\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5999
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 259 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vtx_halfduplex\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 515 as libc::c_int as pgn_t,
                        offset: 50 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vcd_video_system\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_VIDEO_SYSTEM,};
                                                     init
                                                 },},
                        pgn: 514 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vcd_h_offset\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(32
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               31
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 514 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"vcd_v_offset\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(15
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 514 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"max7456_clock\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_MAX7456_CLOCK,};
                                                     init
                                                 },},
                        pgn: 524 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"max7456_spi_bus\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               4
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 524 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"displayport_msp_col_adjust\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(6
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 512 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"displayport_msp_row_adjust\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(3
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 512 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"displayport_max7456_col_adjust\x00" as *const u8
                                as *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(6
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 513 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"displayport_max7456_row_adjust\x00" as *const u8
                                as *const libc::c_char,
                        type_0:
                            (VAR_INT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               -(3
                                                                                     as
                                                                                     libc::c_int)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 513 as libc::c_int as pgn_t,
                        offset: 1 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"displayport_max7456_inv\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 513 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"displayport_max7456_blk\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               3
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 513 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"displayport_max7456_wht\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               3
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 513 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"esc_sensor_halfduplex\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 517 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"esc_sensor_current_offset\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 517 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"led_inversion\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (((1
                                                                                      as
                                                                                      libc::c_int)
                                                                                     <<
                                                                                     3
                                                                                         as
                                                                                         libc::c_int)
                                                                                    -
                                                                                    1
                                                                                        as
                                                                                        libc::c_int)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 505 as libc::c_int as pgn_t,
                        offset: 3 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dashboard_i2c_bus\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               4
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 519 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"dashboard_i2c_addr\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               8
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               119
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 519 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"camera_control_mode\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_CAMERA_CONTROL_MODE,};
                                                     init
                                                 },},
                        pgn: 522 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"camera_control_ref_voltage\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               200
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               400
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 522 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"camera_control_key_delay\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               100
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 522 as libc::c_int as pgn_t,
                        offset: 6 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"camera_control_internal_resistance\x00" as
                                *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT16 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 522 as libc::c_int as pgn_t,
                        offset: 8 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"camera_control_inverted\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 522 as libc::c_int as pgn_t,
                        offset: 11 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pinio_config\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_ARRAY as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{array:
                                                 {
                                                     let mut init =
                                                         cliArrayLengthConfig_s{length:
                                                                                    4
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 529 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"pinio_box\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_ARRAY as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{array:
                                                 {
                                                     let mut init =
                                                         cliArrayLengthConfig_s{length:
                                                                                    4
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 530 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"usb_hid_cdc\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 531 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"usb_msc_pin_pullup\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int |
                                 MODE_LOOKUP as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{lookup:
                                                 {
                                                     let mut init =
                                                         cliLookupTableConfig_s{tableIndex:
                                                                                    TABLE_OFF_ON,};
                                                     init
                                                 },},
                        pgn: 531 as libc::c_int as pgn_t,
                        offset: 2 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rcdevice_init_dev_attempts\x00" as *const u8 as
                                *const libc::c_char,
                        type_0:
                            (VAR_UINT8 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               10
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 539 as libc::c_int as pgn_t,
                        offset: 0 as libc::c_ulong as uint16_t,};
         init
     },
     {
         let mut init =
             clivalue_s{name:
                            b"rcdevice_init_dev_attempt_interval\x00" as
                                *const u8 as *const libc::c_char,
                        type_0:
                            (VAR_UINT32 as libc::c_int |
                                 MASTER_VALUE as libc::c_int) as uint8_t,
                        config:
                            cliValueConfig_t{minmax:
                                                 {
                                                     let mut init =
                                                         cliMinMaxConfig_s{min:
                                                                               500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 539 as libc::c_int as pgn_t,
                        offset: 4 as libc::c_ulong as uint16_t,};
         init
     }];
// Initialized in run_static_initializers
#[no_mangle]
pub static mut valueTableEntryCount: uint16_t = 0;
#[no_mangle]
pub unsafe extern "C" fn settingsBuildCheck() { }
unsafe extern "C" fn run_static_initializers() {
    lookupTables =
        [{
             let mut init =
                 lookupTableEntry_s{values: lookupTableOffOn.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableUnit.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableAlignment.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 9]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableGPSProvider.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableGPSSBASMode.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 5]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableRescueSanityType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableBlackboxDevice.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 4]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableBlackboxMode.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: currentMeterSourceNames.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 5]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: voltageMeterSourceNames.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableGimbalMode.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableSerialRX.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 13]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableGyroHardwareLpf.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableGyro32khzHardwareLpf.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableAccHardware.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 17]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableBaroHardware.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 7]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableMagHardware.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 6]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: debugModeNames.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 44]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTablePwmProtocol.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 10]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableRcInterpolation.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 4]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableRcInterpolationChannels.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 5]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableLowpassType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableDtermLowpassType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableAntiGravityMode.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableFailsafe.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableFailsafeSwitchMode.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableCrashRecovery.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableCameraControlMode.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableBusType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 4]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableMax7456Clock.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableGyroOverflowCheck.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableRatesType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupOverclock.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupLedStripFormatRGB.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableGyro.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableThrottleLimitType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableVideoSystem.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values: lookupTableItermRelax.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 5]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableItermRelaxType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableAcroTrainerDebug.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableRcSmoothingType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableRcSmoothingDebug.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 4]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableRcSmoothingInputType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 2]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         },
         {
             let mut init =
                 lookupTableEntry_s{values:
                                        lookupTableRcSmoothingDerivativeType.as_ptr(),
                                    valueCount:
                                        (::core::mem::size_of::<[*const libc::c_char; 3]>()
                                             as
                                             libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                             as
                                                                             libc::c_ulong)
                                            as uint8_t,};
             init
         }];
    valueTableEntryCount =
        (::core::mem::size_of::<[clivalue_t; 383]>() as
             libc::c_ulong).wrapping_div(::core::mem::size_of::<clivalue_t>()
                                             as libc::c_ulong) as uint16_t
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
