use core;
use libc;
extern "C" {
    #[no_mangle]
    static debugModeNames: [*const libc::c_char; 44];
    #[no_mangle]
    static voltageMeterSourceNames: [*const libc::c_char; 3];
    #[no_mangle]
    static currentMeterSourceNames: [*const libc::c_char; 5];
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
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
pub type C2RustUnnamed = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed = 7;
pub const INPUT_RC_YAW: C2RustUnnamed = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed = 0;
pub type lookupTableIndex_e = libc::c_uint;
pub const LOOKUP_TABLE_COUNT: lookupTableIndex_e = 35;
pub const TABLE_RC_SMOOTHING_DERIVATIVE_TYPE: lookupTableIndex_e = 34;
pub const TABLE_RC_SMOOTHING_INPUT_TYPE: lookupTableIndex_e = 33;
pub const TABLE_RC_SMOOTHING_DEBUG: lookupTableIndex_e = 32;
pub const TABLE_RC_SMOOTHING_TYPE: lookupTableIndex_e = 31;
pub const TABLE_ACRO_TRAINER_DEBUG: lookupTableIndex_e = 30;
pub const TABLE_ITERM_RELAX_TYPE: lookupTableIndex_e = 29;
pub const TABLE_ITERM_RELAX: lookupTableIndex_e = 28;
pub const TABLE_THROTTLE_LIMIT_TYPE: lookupTableIndex_e = 27;
pub const TABLE_RATES_TYPE: lookupTableIndex_e = 26;
pub const TABLE_GYRO_OVERFLOW_CHECK: lookupTableIndex_e = 25;
pub const TABLE_BUS_TYPE: lookupTableIndex_e = 24;
pub const TABLE_CRASH_RECOVERY: lookupTableIndex_e = 23;
pub const TABLE_FAILSAFE_SWITCH_MODE: lookupTableIndex_e = 22;
pub const TABLE_FAILSAFE: lookupTableIndex_e = 21;
pub const TABLE_ANTI_GRAVITY_MODE: lookupTableIndex_e = 20;
pub const TABLE_DTERM_LOWPASS_TYPE: lookupTableIndex_e = 19;
pub const TABLE_LOWPASS_TYPE: lookupTableIndex_e = 18;
pub const TABLE_RC_INTERPOLATION_CHANNELS: lookupTableIndex_e = 17;
pub const TABLE_RC_INTERPOLATION: lookupTableIndex_e = 16;
pub const TABLE_MOTOR_PWM_PROTOCOL: lookupTableIndex_e = 15;
pub const TABLE_DEBUG: lookupTableIndex_e = 14;
pub const TABLE_MAG_HARDWARE: lookupTableIndex_e = 13;
pub const TABLE_BARO_HARDWARE: lookupTableIndex_e = 12;
pub const TABLE_ACC_HARDWARE: lookupTableIndex_e = 11;
pub const TABLE_GYRO_HARDWARE_LPF: lookupTableIndex_e = 10;
pub const TABLE_GIMBAL_MODE: lookupTableIndex_e = 9;
pub const TABLE_VOLTAGE_METER: lookupTableIndex_e = 8;
pub const TABLE_CURRENT_METER: lookupTableIndex_e = 7;
pub const TABLE_BLACKBOX_MODE: lookupTableIndex_e = 6;
pub const TABLE_BLACKBOX_DEVICE: lookupTableIndex_e = 5;
pub const TABLE_GPS_SBAS_MODE: lookupTableIndex_e = 4;
pub const TABLE_GPS_PROVIDER: lookupTableIndex_e = 3;
pub const TABLE_ALIGNMENT: lookupTableIndex_e = 2;
pub const TABLE_UNIT: lookupTableIndex_e = 1;
pub const TABLE_OFF_ON: lookupTableIndex_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct lookupTableEntry_s {
    pub values: *const *const libc::c_char,
    pub valueCount: uint8_t,
}
pub type lookupTableEntry_t = lookupTableEntry_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const MODE_BITSET: C2RustUnnamed_0 = 96;
pub const MODE_ARRAY: C2RustUnnamed_0 = 64;
pub const MODE_LOOKUP: C2RustUnnamed_0 = 32;
pub const MODE_DIRECT: C2RustUnnamed_0 = 0;
pub const PROFILE_RATE_VALUE: C2RustUnnamed_0 = 16;
pub const PROFILE_VALUE: C2RustUnnamed_0 = 8;
pub const MASTER_VALUE: C2RustUnnamed_0 = 0;
pub const VAR_UINT32: C2RustUnnamed_0 = 4;
pub const VAR_INT16: C2RustUnnamed_0 = 3;
pub const VAR_UINT16: C2RustUnnamed_0 = 2;
pub const VAR_INT8: C2RustUnnamed_0 = 1;
pub const VAR_UINT8: C2RustUnnamed_0 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cliMinMaxConfig_s {
    pub min: int16_t,
    pub max: int16_t,
}
pub type cliMinMaxConfig_t = cliMinMaxConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cliLookupTableConfig_s {
    pub tableIndex: lookupTableIndex_e,
}
pub type cliLookupTableConfig_t = cliLookupTableConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct cliArrayLengthConfig_s {
    pub length: uint8_t,
}
pub type cliArrayLengthConfig_t = cliArrayLengthConfig_s;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union cliValueConfig_t {
    pub lookup: cliLookupTableConfig_t,
    pub minmax: cliMinMaxConfig_t,
    pub array: cliArrayLengthConfig_t,
    pub bitpos: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C, packed)]
pub struct clivalue_s {
    pub name: *const libc::c_char,
    pub type_0: uint8_t,
    pub config: cliValueConfig_t,
    pub pgn: pgn_t,
    pub offset: uint16_t,
}
pub type clivalue_t = clivalue_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed = INPUT_STABILIZED_ROLL;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
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
static mut lookupTableGyroHardwareLpf: [*const libc::c_char; 3] =
    [b"NORMAL\x00" as *const u8 as *const libc::c_char,
     b"EXPERIMENTAL\x00" as *const u8 as *const libc::c_char,
     b"1KHZ_SAMPLING\x00" as *const u8 as *const libc::c_char];
static mut lookupTablePwmProtocol: [*const libc::c_char; 5] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"ONESHOT125\x00" as *const u8 as *const libc::c_char,
     b"ONESHOT42\x00" as *const u8 as *const libc::c_char,
     b"MULTISHOT\x00" as *const u8 as *const libc::c_char,
     b"BRUSHED\x00" as *const u8 as *const libc::c_char];
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
static mut lookupTableGyroOverflowCheck: [*const libc::c_char; 3] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"YAW\x00" as *const u8 as *const libc::c_char,
     b"ALL\x00" as *const u8 as *const libc::c_char];
static mut lookupTableRatesType: [*const libc::c_char; 2] =
    [b"BETAFLIGHT\x00" as *const u8 as *const libc::c_char,
     b"RACEFLIGHT\x00" as *const u8 as *const libc::c_char];
static mut lookupTableThrottleLimitType: [*const libc::c_char; 3] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"SCALE\x00" as *const u8 as *const libc::c_char,
     b"CLIP\x00" as *const u8 as *const libc::c_char];
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
pub static mut lookupTables: [lookupTableEntry_t; 35] =
    [lookupTableEntry_t{values: 0 as *const *const libc::c_char,
                        valueCount: 0,}; 35];
#[no_mangle]
pub static mut valueTable: [clivalue_t; 234] =
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
                        pgn: 10i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                        pgn: 10i32 as pgn_t,
                        offset: 3u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                        pgn: 10i32 as pgn_t,
                        offset: 23u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                        pgn: 10i32 as pgn_t,
                        offset: 24u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 10u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 14u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 18u64 as uint16_t,};
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
                                                                               50i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               3000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 28u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                                                                               -1000i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 20u64 as uint16_t,};
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
                        pgn: 10i32 as pgn_t,
                        offset: 22u64 as uint16_t,};
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
                        pgn: 10i32 as pgn_t,
                        offset: 25u64 as uint16_t,};
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
                                                                               500i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1950i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 10i32 as pgn_t,
                        offset: 26u64 as uint16_t,};
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
                        pgn: 35i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                        pgn: 35i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               400i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 35i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               -300i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               300i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 35i32 as pgn_t,
                        offset: 18u64 as uint16_t,};
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
                                                                               -300i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               300i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 35i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                                                                                    3i32
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 35i32 as pgn_t,
                        offset: 10u64 as uint16_t,};
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
                        pgn: 40i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                        pgn: 40i32 as pgn_t,
                        offset: 9u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               4i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 40i32 as pgn_t,
                        offset: 10u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               119i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 40i32 as pgn_t,
                        offset: 11u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               4i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 40i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                        pgn: 40i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                                                                               -18000i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               18000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 40i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                                    3i32
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 40i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                        pgn: 38i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38i32 as pgn_t,
                        offset: 3u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               119i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                        pgn: 38i32 as pgn_t,
                        offset: 5u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               48i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38i32 as pgn_t,
                        offset: 6u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38i32 as pgn_t,
                        offset: 10u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 38i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                                                                               1200i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1700i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 18u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 20u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 22u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               18i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 15u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 35u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                                                                               -100i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 36u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 17u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 24u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 25u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 26u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 37u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 38u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 39u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 40u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 41u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 42u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 27u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (18i32
                                                                                    -
                                                                                    4i32)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 34u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 28u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 30u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 24i32 as pgn_t,
                        offset: 32u64 as uint16_t,};
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
                        pgn: 24i32 as pgn_t,
                        offset: 10u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32767i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 5i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                        pgn: 5i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                        pgn: 5i32 as pgn_t,
                        offset: 3u64 as uint16_t,};
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
                        pgn: 5i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6i32 as pgn_t,
                        offset: 18u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6i32 as pgn_t,
                        offset: 20u64 as uint16_t,};
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
                        pgn: 6i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                        pgn: 6i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               200i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                        pgn: 6i32 as pgn_t,
                        offset: 3u64 as uint16_t,};
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
                                                                               4i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 6i32 as pgn_t,
                        offset: 22u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               150i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 39i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               900i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 39i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 1i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 1i32 as pgn_t,
                        offset: 5u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 1i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                        pgn: 1i32 as pgn_t,
                        offset: 6u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               300i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 1i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                        pgn: 1i32 as pgn_t,
                        offset: 7u64 as uint16_t,};
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
                                                                               -180i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               360i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 2i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               -180i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               360i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 2i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               -180i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               360i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 2i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                        pgn: 3i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               20000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                                                                               10i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               10i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 22u64 as uint16_t,};
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
                                                                               10i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                                                                               10i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 21u64 as uint16_t,};
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
                        pgn: 11i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                        pgn: 11i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 3u64 as uint16_t,};
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
                        pgn: 11i32 as pgn_t,
                        offset: 18u64 as uint16_t,};
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
                        pgn: 11i32 as pgn_t,
                        offset: 19u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 20u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 11i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 258i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 258i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 258i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               -16000i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 256i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               -32000i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 256i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               -16000i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 257i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 257i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                        pgn: 20i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 20i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (1000i32
                                                                                    +
                                                                                    (2000i32
                                                                                         -
                                                                                         1000i32)
                                                                                        /
                                                                                        2i32)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               (1000i32
                                                                                    +
                                                                                    (2000i32
                                                                                         -
                                                                                         1000i32)
                                                                                        /
                                                                                        2i32)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26i32 as pgn_t,
                        offset: 6u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               (1000i32
                                                                                    +
                                                                                    (2000i32
                                                                                         -
                                                                                         1000i32)
                                                                                        /
                                                                                        2i32)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                                                                               (1000i32
                                                                                    +
                                                                                    (2000i32
                                                                                         -
                                                                                         1000i32)
                                                                                        /
                                                                                        2i32)
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 26i32 as pgn_t,
                        offset: 10u64 as uint16_t,};
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
                        pgn: 26i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 52i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               50i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               498i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 52i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               400i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 52i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                        pgn: 52i32 as pgn_t,
                        offset: 14u64 as uint16_t,};
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
                                                                               18i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 52i32 as pgn_t,
                        offset: 15u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                        pgn: 12i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 3u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 5u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 6u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 7u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 9u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 10u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 11u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                                                                               750i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 14u64 as uint16_t,};
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
                        pgn: 12i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                                                                               25i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 12i32 as pgn_t,
                        offset: 17u64 as uint16_t,};
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
                                                                               48i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               126i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 13i32 as pgn_t,
                        offset: 98u64 as uint16_t,};
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
                                                                               100i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 13i32 as pgn_t,
                        offset: 96u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22i32 as pgn_t,
                        offset: 6u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22i32 as pgn_t,
                        offset: 7u64 as uint16_t,};
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
                        pgn: 22i32 as pgn_t,
                        offset: 5u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               180i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 22i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               60i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 16i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                        pgn: 16i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                        pgn: 30i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                        pgn: 30i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                        pgn: 30i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                        pgn: 30i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                        pgn: 30i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               32i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 25i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 25i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                        pgn: 25i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 504i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                        pgn: 504i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                                                                               100i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 504i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 504i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 38u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 76u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               16000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 6u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 70u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 44u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 48u64 as uint16_t,};
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
                                                                               20i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 50u64 as uint16_t,};
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
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               30000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 52u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 71u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 54u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 56u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 58u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 60u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 62u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 64u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 66u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               30i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 68u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 69u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 72u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 78u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 81u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 82u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 85u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 83u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 84u64 as uint16_t,};
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
                                                                               30i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 39u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 74u64 as uint16_t,};
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
                                                                               100i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 40u64 as uint16_t,};
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
                                                                               100i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               1000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 42u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               500i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               100i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 79u64 as uint16_t,};
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
                                                                               5i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               50i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 80u64 as uint16_t,};
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
                                                                               10i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               80i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 86u64 as uint16_t,};
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
                                                                               10i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 90u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 87u64 as uint16_t,};
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
                                                                               25i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 88u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 14u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 15u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 16u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 18u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 8u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 9u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 10u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 12u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 20u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 21u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 22u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               2000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 24u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 26u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 27u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               200i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 28u64 as uint16_t,};
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
                                                                               10i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               90i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 45u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               250i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 46u64 as uint16_t,};
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
                        pgn: 14i32 as pgn_t,
                        offset: 47u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               20i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 92u64 as uint16_t,};
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
                                                                               10i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               255i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 93u64 as uint16_t,};
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
                                                                               1i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               45i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 14i32 as pgn_t,
                        offset: 94u64 as uint16_t,};
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
                        pgn: 31i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                        pgn: 31i32 as pgn_t,
                        offset: 5u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               120i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 31i32 as pgn_t,
                        offset: 17u64 as uint16_t,};
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
                        pgn: 31i32 as pgn_t,
                        offset: 18u64 as uint16_t,};
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
                        pgn: 31i32 as pgn_t,
                        offset: 19u64 as uint16_t,};
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
                        pgn: 18i32 as pgn_t,
                        offset: 3u64 as uint16_t,};
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
                        pgn: 18i32 as pgn_t,
                        offset: 2u64 as uint16_t,};
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
                        pgn: 18i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               30i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 18i32 as pgn_t,
                        offset: 6u64 as uint16_t,};
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
                                                                               -6i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               0i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 512i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               -3i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               0i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 512i32 as pgn_t,
                        offset: 1u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               ((1i32
                                                                                     <<
                                                                                     3i32)
                                                                                    -
                                                                                    1i32)
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 505i32 as pgn_t,
                        offset: 3u64 as uint16_t,};
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
                                                                                    4i32
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 529i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
                                                                                    4i32
                                                                                        as
                                                                                        uint8_t,};
                                                     init
                                                 },},
                        pgn: 530i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               0i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               10i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 539i32 as pgn_t,
                        offset: 0u64 as uint16_t,};
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
                                                                               500i32
                                                                                   as
                                                                                   int16_t,
                                                                           max:
                                                                               5000i32
                                                                                   as
                                                                                   int16_t,};
                                                     init
                                                 },},
                        pgn: 539i32 as pgn_t,
                        offset: 4u64 as uint16_t,};
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
        (::core::mem::size_of::<[clivalue_t; 234]>() as
             libc::c_ulong).wrapping_div(::core::mem::size_of::<clivalue_t>()
                                             as libc::c_ulong) as uint16_t
}
#[used]
#[cfg_attr ( target_os = "linux", link_section = ".init_array" )]
#[cfg_attr ( target_os = "windows", link_section = ".CRT$XIB" )]
#[cfg_attr ( target_os = "macos", link_section = "__DATA,__mod_init_func" )]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
