use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strcat(_: *mut libc::c_char, _: *const libc::c_char)
     -> *mut libc::c_char;
    #[no_mangle]
    static mut blackboxConfig_System: blackboxConfig_t;
    #[no_mangle]
    fn blackboxGetLogNumber() -> libc::c_uint;
    #[no_mangle]
    fn displayIsGrabbed(instance: *const displayPort_t) -> bool;
    #[no_mangle]
    fn displayClearScreen(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayDrawScreen(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayHeartbeat(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayResync(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayWrite(instance: *mut displayPort_t, x: uint8_t, y: uint8_t,
                    s: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn displayWriteChar(instance: *mut displayPort_t, x: uint8_t, y: uint8_t,
                        c: uint8_t) -> libc::c_int;
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn itoa(i: libc::c_int, a: *mut libc::c_char, r: libc::c_int)
     -> *mut libc::c_char;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn isTryingToArm() -> bool;
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn getEstimatedAltitude() -> int32_t;
    #[no_mangle]
    fn isBeeperOn() -> bool;
    #[no_mangle]
    fn flashfsGetOffset() -> uint32_t;
    #[no_mangle]
    fn flashfsGetGeometry() -> *const flashGeometry_s;
    #[no_mangle]
    fn flashfsIsSupported() -> bool;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn getRssiPercent() -> uint8_t;
    #[no_mangle]
    fn getBatteryState() -> batteryState_e;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn getEscSensorData(motorNumber: uint8_t) -> *mut escSensorData_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
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
// parameter group registry flags
// documentary
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
    pub reset: C2RustUnnamed_0,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
}
pub type pgRegistry_t = pgRegistry_s;
/* base */
/* size */
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
// millisecond time
// microsecond time
pub type timeUs_t = uint32_t;
pub type BlackboxDevice = libc::c_uint;
pub const BLACKBOX_DEVICE_SERIAL: BlackboxDevice = 3;
pub const BLACKBOX_DEVICE_FLASH: BlackboxDevice = 1;
pub const BLACKBOX_DEVICE_NONE: BlackboxDevice = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
// I-frame interval / P-frame interval
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_1 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_1 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_1 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_1 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_1 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_1 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_1 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_1 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_1 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_1 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_1 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_1 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_1 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_1 = 8192;
pub const FEATURE_3D: C2RustUnnamed_1 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_1 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_1 = 512;
pub const FEATURE_GPS: C2RustUnnamed_1 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_1 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_1 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_1 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_1 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_1 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_1 = 1;
pub type flashType_e = libc::c_uint;
pub const FLASH_TYPE_NAND: flashType_e = 1;
pub const FLASH_TYPE_NOR: flashType_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flashGeometry_s {
    pub sectors: uint16_t,
    pub pageSize: uint16_t,
    pub sectorSize: uint32_t,
    pub totalSize: uint32_t,
    pub pagesPerSector: uint16_t,
    pub flashType: flashType_e,
}
pub type flashGeometry_t = flashGeometry_s;
pub type boxId_e = libc::c_uint;
pub const CHECKBOX_ITEM_COUNT: boxId_e = 41;
pub const BOXACROTRAINER: boxId_e = 40;
pub const BOXPIDAUDIO: boxId_e = 39;
pub const BOXUSER4: boxId_e = 38;
pub const BOXUSER3: boxId_e = 37;
pub const BOXUSER2: boxId_e = 36;
pub const BOXUSER1: boxId_e = 35;
pub const BOXPARALYZE: boxId_e = 34;
pub const BOXVTXPITMODE: boxId_e = 33;
pub const BOXBEEPGPSCOUNT: boxId_e = 32;
pub const BOXPREARM: boxId_e = 31;
pub const BOXFLIPOVERAFTERCRASH: boxId_e = 30;
pub const BOXCAMERA3: boxId_e = 29;
pub const BOXCAMERA2: boxId_e = 28;
pub const BOXCAMERA1: boxId_e = 27;
pub const BOXBLACKBOXERASE: boxId_e = 26;
pub const BOXFPVANGLEMIX: boxId_e = 25;
pub const BOX3D: boxId_e = 24;
pub const BOXAIRMODE: boxId_e = 23;
pub const BOXBLACKBOX: boxId_e = 22;
pub const BOXSERVO3: boxId_e = 21;
pub const BOXSERVO2: boxId_e = 20;
pub const BOXSERVO1: boxId_e = 19;
pub const BOXTELEMETRY: boxId_e = 18;
pub const BOXOSD: boxId_e = 17;
pub const BOXCALIB: boxId_e = 16;
pub const BOXLEDLOW: boxId_e = 15;
pub const BOXBEEPERON: boxId_e = 14;
pub const BOXCAMSTAB: boxId_e = 13;
pub const BOXHEADADJ: boxId_e = 12;
pub const BOXANTIGRAVITY: boxId_e = 11;
pub const BOXID_FLIGHTMODE_LAST: boxId_e = 10;
pub const BOXGPSRESCUE: boxId_e = 10;
pub const BOXFAILSAFE: boxId_e = 9;
pub const BOXPASSTHRU: boxId_e = 8;
pub const BOXHEADFREE: boxId_e = 7;
pub const BOXGPSHOLD: boxId_e = 6;
pub const BOXGPSHOME: boxId_e = 5;
pub const BOXBARO: boxId_e = 4;
pub const BOXMAG: boxId_e = 3;
pub const BOXHORIZON: boxId_e = 2;
pub const BOXANGLE: boxId_e = 1;
pub const BOXARM: boxId_e = 0;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_2 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_2 = 2;
pub const ARMED: C2RustUnnamed_2 = 1;
pub type armingDisableFlags_e = libc::c_uint;
pub const ARMING_DISABLED_ARM_SWITCH: armingDisableFlags_e = 524288;
pub const ARMING_DISABLED_GPS: armingDisableFlags_e = 262144;
pub const ARMING_DISABLED_PARALYZE: armingDisableFlags_e = 131072;
pub const ARMING_DISABLED_MSP: armingDisableFlags_e = 65536;
pub const ARMING_DISABLED_BST: armingDisableFlags_e = 32768;
pub const ARMING_DISABLED_OSD_MENU: armingDisableFlags_e = 16384;
pub const ARMING_DISABLED_CMS_MENU: armingDisableFlags_e = 8192;
pub const ARMING_DISABLED_CLI: armingDisableFlags_e = 4096;
pub const ARMING_DISABLED_CALIBRATING: armingDisableFlags_e = 2048;
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_3 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_3 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_3 = 4;
pub const GPS_FIX: C2RustUnnamed_3 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_3 = 1;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const OSD_ITEM_COUNT: C2RustUnnamed_4 = 43;
pub const OSD_G_FORCE: C2RustUnnamed_4 = 42;
pub const OSD_ANTI_GRAVITY: C2RustUnnamed_4 = 41;
pub const OSD_CORE_TEMPERATURE: C2RustUnnamed_4 = 40;
pub const OSD_ADJUSTMENT_RANGE: C2RustUnnamed_4 = 39;
pub const OSD_RTC_DATETIME: C2RustUnnamed_4 = 38;
pub const OSD_REMAINING_TIME_ESTIMATE: C2RustUnnamed_4 = 37;
pub const OSD_ESC_RPM: C2RustUnnamed_4 = 36;
pub const OSD_ESC_TMP: C2RustUnnamed_4 = 35;
pub const OSD_COMPASS_BAR: C2RustUnnamed_4 = 34;
pub const OSD_NUMERICAL_VARIO: C2RustUnnamed_4 = 33;
pub const OSD_NUMERICAL_HEADING: C2RustUnnamed_4 = 32;
pub const OSD_HOME_DIST: C2RustUnnamed_4 = 31;
pub const OSD_HOME_DIR: C2RustUnnamed_4 = 30;
pub const OSD_DISARMED: C2RustUnnamed_4 = 29;
pub const OSD_MAIN_BATT_USAGE: C2RustUnnamed_4 = 28;
pub const OSD_ROLL_ANGLE: C2RustUnnamed_4 = 27;
pub const OSD_PITCH_ANGLE: C2RustUnnamed_4 = 26;
pub const OSD_DEBUG: C2RustUnnamed_4 = 25;
pub const OSD_GPS_LAT: C2RustUnnamed_4 = 24;
pub const OSD_GPS_LON: C2RustUnnamed_4 = 23;
pub const OSD_AVG_CELL_VOLTAGE: C2RustUnnamed_4 = 22;
pub const OSD_WARNINGS: C2RustUnnamed_4 = 21;
pub const OSD_PIDRATE_PROFILE: C2RustUnnamed_4 = 20;
pub const OSD_POWER: C2RustUnnamed_4 = 19;
pub const OSD_YAW_PIDS: C2RustUnnamed_4 = 18;
pub const OSD_PITCH_PIDS: C2RustUnnamed_4 = 17;
pub const OSD_ROLL_PIDS: C2RustUnnamed_4 = 16;
pub const OSD_ALTITUDE: C2RustUnnamed_4 = 15;
pub const OSD_GPS_SATS: C2RustUnnamed_4 = 14;
pub const OSD_GPS_SPEED: C2RustUnnamed_4 = 13;
pub const OSD_MAH_DRAWN: C2RustUnnamed_4 = 12;
pub const OSD_CURRENT_DRAW: C2RustUnnamed_4 = 11;
pub const OSD_VTX_CHANNEL: C2RustUnnamed_4 = 10;
pub const OSD_THROTTLE_POS: C2RustUnnamed_4 = 9;
pub const OSD_CRAFT_NAME: C2RustUnnamed_4 = 8;
pub const OSD_FLYMODE: C2RustUnnamed_4 = 7;
pub const OSD_ITEM_TIMER_2: C2RustUnnamed_4 = 6;
pub const OSD_ITEM_TIMER_1: C2RustUnnamed_4 = 5;
pub const OSD_HORIZON_SIDEBARS: C2RustUnnamed_4 = 4;
pub const OSD_ARTIFICIAL_HORIZON: C2RustUnnamed_4 = 3;
pub const OSD_CROSSHAIRS: C2RustUnnamed_4 = 2;
pub const OSD_MAIN_BATT_VOLTAGE: C2RustUnnamed_4 = 1;
pub const OSD_RSSI_VALUE: C2RustUnnamed_4 = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const OSD_STAT_COUNT: C2RustUnnamed_5 = 14;
pub const OSD_STAT_BLACKBOX_NUMBER: C2RustUnnamed_5 = 13;
pub const OSD_STAT_BLACKBOX: C2RustUnnamed_5 = 12;
pub const OSD_STAT_MAX_ALTITUDE: C2RustUnnamed_5 = 11;
pub const OSD_STAT_USED_MAH: C2RustUnnamed_5 = 10;
pub const OSD_STAT_MAX_CURRENT: C2RustUnnamed_5 = 9;
pub const OSD_STAT_MIN_RSSI: C2RustUnnamed_5 = 8;
pub const OSD_STAT_BATTERY: C2RustUnnamed_5 = 7;
pub const OSD_STAT_END_BATTERY: C2RustUnnamed_5 = 6;
pub const OSD_STAT_MIN_BATTERY: C2RustUnnamed_5 = 5;
pub const OSD_STAT_MAX_DISTANCE: C2RustUnnamed_5 = 4;
pub const OSD_STAT_MAX_SPEED: C2RustUnnamed_5 = 3;
pub const OSD_STAT_TIMER_2: C2RustUnnamed_5 = 2;
pub const OSD_STAT_TIMER_1: C2RustUnnamed_5 = 1;
pub const OSD_STAT_RTC_DATE_TIME: C2RustUnnamed_5 = 0;
pub type osd_unit_e = libc::c_uint;
pub const OSD_UNIT_METRIC: osd_unit_e = 1;
pub const OSD_UNIT_IMPERIAL: osd_unit_e = 0;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const OSD_TIMER_COUNT: C2RustUnnamed_6 = 2;
pub const OSD_TIMER_2: C2RustUnnamed_6 = 1;
pub const OSD_TIMER_1: C2RustUnnamed_6 = 0;
pub type osd_timer_source_e = libc::c_uint;
pub const OSD_TIMER_SRC_COUNT: osd_timer_source_e = 3;
pub const OSD_TIMER_SRC_LAST_ARMED: osd_timer_source_e = 2;
pub const OSD_TIMER_SRC_TOTAL_ARMED: osd_timer_source_e = 1;
pub const OSD_TIMER_SRC_ON: osd_timer_source_e = 0;
pub type osd_timer_precision_e = libc::c_uint;
pub const OSD_TIMER_PREC_COUNT: osd_timer_precision_e = 2;
pub const OSD_TIMER_PREC_HUNDREDTHS: osd_timer_precision_e = 1;
pub const OSD_TIMER_PREC_SECOND: osd_timer_precision_e = 0;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const OSD_WARNING_COUNT: C2RustUnnamed_7 = 9;
pub const OSD_WARNING_RC_SMOOTHING: C2RustUnnamed_7 = 8;
pub const OSD_WARNING_CORE_TEMPERATURE: C2RustUnnamed_7 = 7;
pub const OSD_WARNING_ESC_FAIL: C2RustUnnamed_7 = 6;
pub const OSD_WARNING_CRASH_FLIP: C2RustUnnamed_7 = 5;
pub const OSD_WARNING_VISUAL_BEEPER: C2RustUnnamed_7 = 4;
pub const OSD_WARNING_BATTERY_CRITICAL: C2RustUnnamed_7 = 3;
pub const OSD_WARNING_BATTERY_WARNING: C2RustUnnamed_7 = 2;
pub const OSD_WARNING_BATTERY_NOT_FULL: C2RustUnnamed_7 = 1;
pub const OSD_WARNING_ARMING_DISABLE: C2RustUnnamed_7 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct osdConfig_s {
    pub item_pos: [uint16_t; 43],
    pub cap_alarm: uint16_t,
    pub alt_alarm: uint16_t,
    pub rssi_alarm: uint8_t,
    pub units: osd_unit_e,
    pub timers: [uint16_t; 2],
    pub enabledWarnings: uint16_t,
    pub ahMaxPitch: uint8_t,
    pub ahMaxRoll: uint8_t,
    pub enabled_stats: uint32_t,
    pub esc_temp_alarm: int8_t,
    pub esc_rpm_alarm: int16_t,
    pub esc_current_alarm: int16_t,
    pub core_temp_alarm: uint8_t,
}
pub type osdConfig_t = osdConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct escSensorData_t {
    pub dataAge: uint8_t,
    pub temperature: int8_t,
    pub voltage: int16_t,
    pub current: int32_t,
    pub consumption: int32_t,
    pub rpm: int16_t,
}
pub type statistic_t = statistic_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct statistic_s {
    pub armed_time: timeUs_t,
    pub max_speed: int16_t,
    pub min_voltage: int16_t,
    pub max_current: int16_t,
    pub min_rssi: int16_t,
    pub max_altitude: int32_t,
    pub max_distance: int16_t,
}
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub type batteryConfig_t = batteryConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct batteryConfig_s {
    pub vbatmaxcellvoltage: uint8_t,
    pub vbatmincellvoltage: uint8_t,
    pub vbatwarningcellvoltage: uint8_t,
    pub vbatnotpresentcellvoltage: uint8_t,
    pub lvcPercentage: uint8_t,
    pub voltageMeterSource: voltageMeterSource_e,
    pub currentMeterSource: currentMeterSource_e,
    pub batteryCapacity: uint16_t,
    pub useVBatAlerts: bool,
    pub useConsumptionAlerts: bool,
    pub consumptionWarningPercentage: uint8_t,
    pub vbathysteresis: uint8_t,
    pub vbatfullcellvoltage: uint8_t,
}
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const BATTERY_OK: batteryState_e = 0;
#[inline]
unsafe extern "C" fn cmp32(mut a: uint32_t, mut b: uint32_t) -> int32_t {
    return a.wrapping_sub(b) as int32_t;
}
#[inline]
unsafe extern "C" fn blackboxConfig() -> *const blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn osdConfig() -> *const osdConfig_t {
    return &mut osdConfig_System;
}
#[inline]
unsafe extern "C" fn osdConfigMutable() -> *mut osdConfig_t {
    return &mut osdConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[no_mangle]
pub static mut osdTimerSourceNames: [*const libc::c_char; 3] =
    [b"ON TIME  \x00" as *const u8 as *const libc::c_char,
     b"TOTAL ARM\x00" as *const u8 as *const libc::c_char,
     b"LAST ARM \x00" as *const u8 as *const libc::c_char];
// Alarms
// /10
// /10
// Blink control
static mut blinkState: bool = 1i32 != 0;
static mut showVisualBeeper: bool = 0i32 != 0;
static mut blinkBits: [uint32_t; 2] = [0; 2];
// Things in both OSD and CMS
static mut flyTime: timeUs_t = 0i32 as timeUs_t;
static mut stats: statistic_t =
    statistic_t{armed_time: 0,
                max_speed: 0,
                min_voltage: 0,
                max_current: 0,
                min_rssi: 0,
                max_altitude: 0,
                max_distance: 0,};
#[no_mangle]
pub static mut resumeRefreshAt: timeUs_t = 0i32 as timeUs_t;
static mut armState: uint8_t = 0;
static mut lastArmState: bool = false;
static mut osdDisplayPort: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
static mut escDataCombined: *mut escSensorData_t =
    0 as *const escSensorData_t as *mut escSensorData_t;
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut osdConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (501i32 | 3i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<osdConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &osdConfig_System as *const osdConfig_t as
                                     *mut osdConfig_t as *mut uint8_t,
                             copy:
                                 &osdConfig_Copy as *const osdConfig_t as
                                     *mut osdConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut osdConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<unsafe extern "C" fn(_:
                                                                                                              *mut libc::c_void,
                                                                                                          _:
                                                                                                              libc::c_int)
                                                                                         ->
                                                                                             ()>>(Some(pgResetFn_osdConfig
                                                                                                           as
                                                                                                           unsafe extern "C" fn(_:
                                                                                                                                    *mut osdConfig_t)
                                                                                                               ->
                                                                                                                   ())),},};
            init
        }
    };
#[no_mangle]
pub static mut osdConfig_System: osdConfig_t =
    osdConfig_t{item_pos: [0; 43],
                cap_alarm: 0,
                alt_alarm: 0,
                rssi_alarm: 0,
                units: OSD_UNIT_IMPERIAL,
                timers: [0; 2],
                enabledWarnings: 0,
                ahMaxPitch: 0,
                ahMaxRoll: 0,
                enabled_stats: 0,
                esc_temp_alarm: 0,
                esc_rpm_alarm: 0,
                esc_current_alarm: 0,
                core_temp_alarm: 0,};
#[no_mangle]
pub static mut osdConfig_Copy: osdConfig_t =
    osdConfig_t{item_pos: [0; 43],
                cap_alarm: 0,
                alt_alarm: 0,
                rssi_alarm: 0,
                units: OSD_UNIT_IMPERIAL,
                timers: [0; 2],
                enabledWarnings: 0,
                ahMaxPitch: 0,
                ahMaxRoll: 0,
                enabled_stats: 0,
                esc_temp_alarm: 0,
                esc_rpm_alarm: 0,
                esc_current_alarm: 0,
                core_temp_alarm: 0,};
/* *
 * Gets the correct altitude symbol for the current unit system
 */
unsafe extern "C" fn osdGetMetersToSelectedUnitSymbol() -> libc::c_char {
    match (*osdConfig()).units as libc::c_uint {
        0 => { return 0xfi32 as libc::c_char }
        _ => { return 0xci32 as libc::c_char }
    };
}
/* *
 * Converts altitude based on the current unit system.
 * @param meters Value in meters to convert
 */
unsafe extern "C" fn osdGetMetersToSelectedUnit(mut meters: int32_t)
 -> int32_t {
    match (*osdConfig()).units as libc::c_uint {
        0 => {
            return meters * 328i32 / 100i32
            // Already in metre / 100
        }
        _ => { return meters }
    };
}
unsafe extern "C" fn osdFormatAltitudeString(mut buff: *mut libc::c_char,
                                             mut altitude: libc::c_int) {
    let alt: libc::c_int = osdGetMetersToSelectedUnit(altitude) / 10i32;
    tfp_sprintf(buff, b"%5d %c\x00" as *const u8 as *const libc::c_char, alt,
                osdGetMetersToSelectedUnitSymbol() as libc::c_int);
    *buff.offset(5) = *buff.offset(4);
    *buff.offset(4) = '.' as i32 as libc::c_char;
}
unsafe extern "C" fn osdGetTimerSymbol(mut src: osd_timer_source_e)
 -> libc::c_char {
    match src as libc::c_uint {
        0 => { return 0x9bi32 as libc::c_char }
        1 | 2 => { return 0x9ci32 as libc::c_char }
        _ => { return ' ' as i32 as libc::c_char }
    };
}
unsafe extern "C" fn osdGetTimerValue(mut src: osd_timer_source_e)
 -> timeUs_t {
    match src as libc::c_uint {
        0 => { return micros() }
        1 => { return flyTime }
        2 => { return stats.armed_time }
        _ => { return 0i32 as timeUs_t }
    };
}
unsafe extern "C" fn osdFormatTime(mut buff: *mut libc::c_char,
                                   mut precision: osd_timer_precision_e,
                                   mut time: timeUs_t) {
    let mut seconds: libc::c_int =
        time.wrapping_div(1000000i32 as libc::c_uint) as libc::c_int;
    let minutes: libc::c_int = seconds / 60i32;
    seconds = seconds % 60i32;
    match precision as libc::c_uint {
        1 => {
            let hundredths: libc::c_int =
                time.wrapping_div(10000i32 as
                                      libc::c_uint).wrapping_rem(100i32 as
                                                                     libc::c_uint)
                    as libc::c_int;
            tfp_sprintf(buff,
                        b"%02d:%02d.%02d\x00" as *const u8 as
                            *const libc::c_char, minutes, seconds,
                        hundredths);
        }
        0 | _ => {
            tfp_sprintf(buff,
                        b"%02d:%02d\x00" as *const u8 as *const libc::c_char,
                        minutes, seconds);
        }
    };
}
unsafe extern "C" fn osdFormatTimer(mut buff: *mut libc::c_char,
                                    mut showSymbol: bool,
                                    mut usePrecision: bool,
                                    mut timerIndex: libc::c_int) {
    let timer: uint16_t = (*osdConfig()).timers[timerIndex as usize];
    let src: uint8_t = (timer as libc::c_int & 0xfi32) as uint8_t;
    if showSymbol {
        let fresh0 = buff;
        buff = buff.offset(1);
        *fresh0 = osdGetTimerSymbol(src as osd_timer_source_e)
    }
    osdFormatTime(buff,
                  if usePrecision as libc::c_int != 0 {
                      (timer as libc::c_int >> 4i32) & 0xfi32
                  } else { OSD_TIMER_PREC_SECOND as libc::c_int } as
                      osd_timer_precision_e,
                  osdGetTimerValue(src as osd_timer_source_e));
}
#[no_mangle]
pub unsafe extern "C" fn osdStatSetState(mut statIndex: uint8_t,
                                         mut enabled: bool) {
    if enabled {
        let ref mut fresh1 = (*osdConfigMutable()).enabled_stats;
        *fresh1 |= (1i32 << statIndex as libc::c_int) as libc::c_uint
    } else {
        let ref mut fresh2 = (*osdConfigMutable()).enabled_stats;
        *fresh2 &= !(1i32 << statIndex as libc::c_int) as libc::c_uint
    };
}
#[no_mangle]
pub unsafe extern "C" fn osdStatGetState(mut statIndex: uint8_t) -> bool {
    return (*osdConfig()).enabled_stats &
               (1i32 << statIndex as libc::c_int) as libc::c_uint != 0;
}
#[no_mangle]
pub unsafe extern "C" fn osdWarnSetState(mut warningIndex: uint8_t,
                                         mut enabled: bool) {
    if enabled {
        let ref mut fresh3 = (*osdConfigMutable()).enabledWarnings;
        *fresh3 =
            (*fresh3 as libc::c_int | 1i32 << warningIndex as libc::c_int) as
                uint16_t
    } else {
        let ref mut fresh4 = (*osdConfigMutable()).enabledWarnings;
        *fresh4 =
            (*fresh4 as libc::c_int & !(1i32 << warningIndex as libc::c_int))
                as uint16_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn osdWarnGetState(mut warningIndex: uint8_t) -> bool {
    return (*osdConfig()).enabledWarnings as libc::c_int &
               1i32 << warningIndex as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_osdConfig(mut osdConfig_0:
                                                 *mut osdConfig_t) {
    (*osdConfig_0).item_pos[OSD_RSSI_VALUE as libc::c_int as usize] =
        (8i32 & (1i32 << 5i32) - 1i32 | (1i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_MAIN_BATT_VOLTAGE as libc::c_int as usize] =
        (12i32 & (1i32 << 5i32) - 1i32 |
             (1i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_CROSSHAIRS as libc::c_int as usize] =
        (8i32 & (1i32 << 5i32) - 1i32 | (6i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ARTIFICIAL_HORIZON as libc::c_int as usize] =
        (8i32 & (1i32 << 5i32) - 1i32 | (6i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_HORIZON_SIDEBARS as libc::c_int as usize] =
        (8i32 & (1i32 << 5i32) - 1i32 | (6i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ITEM_TIMER_1 as libc::c_int as usize] =
        (22i32 & (1i32 << 5i32) - 1i32 |
             (1i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ITEM_TIMER_2 as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 | (1i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_FLYMODE as libc::c_int as usize] =
        (13i32 & (1i32 << 5i32) - 1i32 |
             (10i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_CRAFT_NAME as libc::c_int as usize] =
        (10i32 & (1i32 << 5i32) - 1i32 |
             (11i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_THROTTLE_POS as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 | (7i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_VTX_CHANNEL as libc::c_int as usize] =
        (24i32 & (1i32 << 5i32) - 1i32 |
             (11i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_CURRENT_DRAW as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 |
             (12i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_MAH_DRAWN as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 |
             (11i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_GPS_SPEED as libc::c_int as usize] =
        (25i32 & (1i32 << 5i32) - 1i32 |
             (6i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_GPS_SATS as libc::c_int as usize] =
        (19i32 & (1i32 << 5i32) - 1i32 |
             (1i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ALTITUDE as libc::c_int as usize] =
        (24i32 & (1i32 << 5i32) - 1i32 |
             (7i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ROLL_PIDS as libc::c_int as usize] =
        (7i32 & (1i32 << 5i32) - 1i32 |
             (13i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_PITCH_PIDS as libc::c_int as usize] =
        (7i32 & (1i32 << 5i32) - 1i32 |
             (14i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_YAW_PIDS as libc::c_int as usize] =
        (7i32 & (1i32 << 5i32) - 1i32 |
             (15i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_POWER as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 |
             (10i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_PIDRATE_PROFILE as libc::c_int as usize] =
        (25i32 & (1i32 << 5i32) - 1i32 |
             (10i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_AVG_CELL_VOLTAGE as libc::c_int as usize] =
        (12i32 & (1i32 << 5i32) - 1i32 |
             (0i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_DEBUG as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 |
             (0i32 & (1i32 << 5i32) - 1i32) << 5i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_PITCH_ANGLE as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 | (8i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ROLL_ANGLE as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 | (9i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_GPS_LAT as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 | (0i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_GPS_LON as libc::c_int as usize] =
        (18i32 & (1i32 << 5i32) - 1i32 |
             (0i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_HOME_DIST as libc::c_int as usize] =
        (15i32 & (1i32 << 5i32) - 1i32 |
             (9i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_HOME_DIR as libc::c_int as usize] =
        (14i32 & (1i32 << 5i32) - 1i32 |
             (9i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_COMPASS_BAR as libc::c_int as usize] =
        (10i32 & (1i32 << 5i32) - 1i32 |
             (8i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_MAIN_BATT_USAGE as libc::c_int as usize] =
        (8i32 & (1i32 << 5i32) - 1i32 |
             (12i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_DISARMED as libc::c_int as usize] =
        (11i32 & (1i32 << 5i32) - 1i32 |
             (4i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_NUMERICAL_HEADING as libc::c_int as usize] =
        (24i32 & (1i32 << 5i32) - 1i32 |
             (9i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_NUMERICAL_VARIO as libc::c_int as usize] =
        (24i32 & (1i32 << 5i32) - 1i32 |
             (8i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ESC_TMP as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 | (5i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ESC_RPM as libc::c_int as usize] =
        (1i32 & (1i32 << 5i32) - 1i32 | (6i32 & (1i32 << 5i32) - 1i32) << 5i32
             | 0x800i32) as uint16_t;
    // Always enable warnings elements by default
    (*osdConfig_0).item_pos[OSD_WARNINGS as libc::c_int as usize] =
        (9i32 & (1i32 << 5i32) - 1i32 |
             (10i32 & (1i32 << 5i32) - 1i32) << 5i32 | 0x800i32) as uint16_t;
    // Default to old fixed positions for these elements
    (*osdConfig_0).item_pos[OSD_CROSSHAIRS as libc::c_int as usize] =
        (13i32 & (1i32 << 5i32) - 1i32 |
             (6i32 & (1i32 << 5i32) - 1i32) << 5i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ARTIFICIAL_HORIZON as libc::c_int as usize] =
        (14i32 & (1i32 << 5i32) - 1i32 |
             (2i32 & (1i32 << 5i32) - 1i32) << 5i32) as uint16_t;
    (*osdConfig_0).item_pos[OSD_HORIZON_SIDEBARS as libc::c_int as usize] =
        (14i32 & (1i32 << 5i32) - 1i32 |
             (6i32 & (1i32 << 5i32) - 1i32) << 5i32) as uint16_t;
    // Enable the default stats
    (*osdConfig_0).enabled_stats =
        0i32 as uint32_t; // reset all to off and enable only a few initially
    osdStatSetState(OSD_STAT_MAX_SPEED as libc::c_int as uint8_t, 1i32 != 0);
    osdStatSetState(OSD_STAT_MIN_BATTERY as libc::c_int as uint8_t,
                    1i32 != 0);
    osdStatSetState(OSD_STAT_MIN_RSSI as libc::c_int as uint8_t, 1i32 != 0);
    osdStatSetState(OSD_STAT_MAX_CURRENT as libc::c_int as uint8_t,
                    1i32 != 0);
    osdStatSetState(OSD_STAT_USED_MAH as libc::c_int as uint8_t, 1i32 != 0);
    osdStatSetState(OSD_STAT_BLACKBOX as libc::c_int as uint8_t, 1i32 != 0);
    osdStatSetState(OSD_STAT_BLACKBOX_NUMBER as libc::c_int as uint8_t,
                    1i32 != 0);
    osdStatSetState(OSD_STAT_TIMER_2 as libc::c_int as uint8_t, 1i32 != 0);
    (*osdConfig_0).units = OSD_UNIT_METRIC;
    // Enable all warnings by default
    let mut i: libc::c_int = 0i32; // meters or feet depend on configuration
    while i < OSD_WARNING_COUNT as libc::c_int {
        osdWarnSetState(i as uint8_t, 1i32 != 0); // off by default
        i += 1
    } // off by default
    (*osdConfig_0).timers[OSD_TIMER_1 as libc::c_int as usize] =
        (OSD_TIMER_SRC_ON as libc::c_int & 0xfi32 |
             (OSD_TIMER_PREC_SECOND as libc::c_int & 0xfi32) << 4i32 |
             (10i32 & 0xffi32) << 8i32) as uint16_t; // off by default
    (*osdConfig_0).timers[OSD_TIMER_2 as libc::c_int as usize] =
        (OSD_TIMER_SRC_TOTAL_ARMED as libc::c_int & 0xfi32 |
             (OSD_TIMER_PREC_SECOND as libc::c_int & 0xfi32) << 4i32 |
             (10i32 & 0xffi32) << 8i32) as
            uint16_t; // a temperature above 70C should produce a warning, lockups have been reported above 80C
    (*osdConfig_0).rssi_alarm = 20i32 as uint8_t; // 20 degrees
    (*osdConfig_0).cap_alarm = 2200i32 as uint16_t;
    (*osdConfig_0).alt_alarm = 100i32 as uint16_t;
    (*osdConfig_0).esc_temp_alarm = -128i32 as int8_t;
    (*osdConfig_0).esc_rpm_alarm = -1i32 as int16_t;
    (*osdConfig_0).esc_current_alarm = -1i32 as int16_t;
    (*osdConfig_0).core_temp_alarm = 70i32 as uint8_t;
    (*osdConfig_0).ahMaxPitch = 20i32 as uint8_t;
    (*osdConfig_0).ahMaxRoll = 40i32 as uint8_t;
    // 40 degrees
}
unsafe extern "C" fn osdDrawLogo(mut x: libc::c_int, mut y: libc::c_int) {
    // display logo and help
    let mut fontOffset: libc::c_int = 160i32;
    let mut row: libc::c_int = 0i32;
    while row < 4i32 {
        let mut column: libc::c_int = 0i32;
        while column < 24i32 {
            if fontOffset <= 0xffi32 {
                let fresh5 = fontOffset;
                fontOffset = fontOffset + 1;
                displayWriteChar(osdDisplayPort, (x + column) as uint8_t,
                                 (y + row) as uint8_t, fresh5 as uint8_t);
            }
            column += 1
        }
        row += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn osdInit(mut osdDisplayPortToUse:
                                     *mut displayPort_t) {
    if osdDisplayPortToUse.is_null() { return }
    osdDisplayPort = osdDisplayPortToUse;
    armState = (armingFlags as libc::c_int & ARMED as libc::c_int) as uint8_t;
    memset(blinkBits.as_mut_ptr() as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<[uint32_t; 2]>() as libc::c_ulong);
    displayClearScreen(osdDisplayPort);
    osdDrawLogo(3i32, 1i32);
    let mut string_buffer: [libc::c_char; 30] = [0; 30];
    tfp_sprintf(string_buffer.as_mut_ptr(),
                b"V%s\x00" as *const u8 as *const libc::c_char,
                b"2.5.0\x00" as *const u8 as *const libc::c_char);
    displayWrite(osdDisplayPort, 20i32 as uint8_t, 6i32 as uint8_t,
                 string_buffer.as_mut_ptr());
    displayResync(osdDisplayPort);
    resumeRefreshAt =
        micros().wrapping_add((4i32 * 1000i32 * 1000i32) as libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn osdInitialized() -> bool {
    return !osdDisplayPort.is_null();
}
#[no_mangle]
pub unsafe extern "C" fn osdUpdateAlarms() {
    // This is overdone?
    let mut alt: int32_t =
        osdGetMetersToSelectedUnit(getEstimatedAltitude()) / 100i32;
    if (getRssiPercent() as libc::c_int) <
           (*osdConfig()).rssi_alarm as libc::c_int {
        blinkBits[(OSD_RSSI_VALUE as libc::c_int / 32i32) as usize] |=
            (1i32 << OSD_RSSI_VALUE as libc::c_int % 32i32) as libc::c_uint
    } else {
        blinkBits[(OSD_RSSI_VALUE as libc::c_int / 32i32) as usize] &=
            !(1i32 << OSD_RSSI_VALUE as libc::c_int % 32i32) as libc::c_uint
    }
    // Determine if the OSD_WARNINGS should blink
    if getBatteryState() as libc::c_uint !=
           BATTERY_OK as libc::c_int as libc::c_uint &&
           (osdWarnGetState(OSD_WARNING_BATTERY_CRITICAL as libc::c_int as
                                uint8_t) as libc::c_int != 0 ||
                osdWarnGetState(OSD_WARNING_BATTERY_WARNING as libc::c_int as
                                    uint8_t) as libc::c_int != 0) &&
           !isTryingToArm() {
        blinkBits[(OSD_WARNINGS as libc::c_int / 32i32) as usize] |=
            (1i32 << OSD_WARNINGS as libc::c_int % 32i32) as libc::c_uint
    } else {
        blinkBits[(OSD_WARNINGS as libc::c_int / 32i32) as usize] &=
            !(1i32 << OSD_WARNINGS as libc::c_int % 32i32) as libc::c_uint
    } // convert from minutes to us
    if getBatteryState() as libc::c_uint ==
           BATTERY_OK as libc::c_int as libc::c_uint {
        blinkBits[(OSD_MAIN_BATT_VOLTAGE as libc::c_int / 32i32) as usize] &=
            !(1i32 << OSD_MAIN_BATT_VOLTAGE as libc::c_int % 32i32) as
                libc::c_uint;
        blinkBits[(OSD_AVG_CELL_VOLTAGE as libc::c_int / 32i32) as usize] &=
            !(1i32 << OSD_AVG_CELL_VOLTAGE as libc::c_int % 32i32) as
                libc::c_uint
    } else {
        blinkBits[(OSD_MAIN_BATT_VOLTAGE as libc::c_int / 32i32) as usize] |=
            (1i32 << OSD_MAIN_BATT_VOLTAGE as libc::c_int % 32i32) as
                libc::c_uint;
        blinkBits[(OSD_AVG_CELL_VOLTAGE as libc::c_int / 32i32) as usize] |=
            (1i32 << OSD_AVG_CELL_VOLTAGE as libc::c_int % 32i32) as
                libc::c_uint
    }
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int == 0i32 {
        blinkBits[(OSD_GPS_SATS as libc::c_int / 32i32) as usize] |=
            (1i32 << OSD_GPS_SATS as libc::c_int % 32i32) as libc::c_uint
    } else {
        blinkBits[(OSD_GPS_SATS as libc::c_int / 32i32) as usize] &=
            !(1i32 << OSD_GPS_SATS as libc::c_int % 32i32) as libc::c_uint
    }
    let mut i: libc::c_int = 0i32;
    while i < OSD_TIMER_COUNT as libc::c_int {
        let timer: uint16_t = (*osdConfig()).timers[i as usize];
        let time: timeUs_t =
            osdGetTimerValue((timer as libc::c_int & 0xfi32) as
                                 osd_timer_source_e);
        let alarmTime: timeUs_t =
            ((timer as libc::c_int >> 8i32 & 0xffi32) * 60000000i32) as
                timeUs_t;
        if alarmTime != 0i32 as libc::c_uint && time >= alarmTime {
            blinkBits[((OSD_ITEM_TIMER_1 as libc::c_int + i) / 32i32) as
                          usize] |=
                (1i32 << (OSD_ITEM_TIMER_1 as libc::c_int + i) % 32i32) as
                    libc::c_uint
        } else {
            blinkBits[((OSD_ITEM_TIMER_1 as libc::c_int + i) / 32i32) as
                          usize] &=
                !(1i32 << (OSD_ITEM_TIMER_1 as libc::c_int + i) % 32i32) as
                    libc::c_uint
        }
        i += 1
    }
    if getMAhDrawn() >= (*osdConfig()).cap_alarm as libc::c_int {
        blinkBits[(OSD_MAH_DRAWN as libc::c_int / 32i32) as usize] |=
            (1i32 << OSD_MAH_DRAWN as libc::c_int % 32i32) as libc::c_uint;
        blinkBits[(OSD_MAIN_BATT_USAGE as libc::c_int / 32i32) as usize] |=
            (1i32 << OSD_MAIN_BATT_USAGE as libc::c_int % 32i32) as
                libc::c_uint;
        blinkBits[(OSD_REMAINING_TIME_ESTIMATE as libc::c_int / 32i32) as
                      usize] |=
            (1i32 << OSD_REMAINING_TIME_ESTIMATE as libc::c_int % 32i32) as
                libc::c_uint
    } else {
        blinkBits[(OSD_MAH_DRAWN as libc::c_int / 32i32) as usize] &=
            !(1i32 << OSD_MAH_DRAWN as libc::c_int % 32i32) as libc::c_uint;
        blinkBits[(OSD_MAIN_BATT_USAGE as libc::c_int / 32i32) as usize] &=
            !(1i32 << OSD_MAIN_BATT_USAGE as libc::c_int % 32i32) as
                libc::c_uint;
        blinkBits[(OSD_REMAINING_TIME_ESTIMATE as libc::c_int / 32i32) as
                      usize] &=
            !(1i32 << OSD_REMAINING_TIME_ESTIMATE as libc::c_int % 32i32) as
                libc::c_uint
    }
    if alt >= (*osdConfig()).alt_alarm as libc::c_int {
        blinkBits[(OSD_ALTITUDE as libc::c_int / 32i32) as usize] |=
            (1i32 << OSD_ALTITUDE as libc::c_int % 32i32) as libc::c_uint
    } else {
        blinkBits[(OSD_ALTITUDE as libc::c_int / 32i32) as usize] &=
            !(1i32 << OSD_ALTITUDE as libc::c_int % 32i32) as libc::c_uint
    }
    if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
        // This works because the combined ESC data contains the maximum temperature seen amongst all ESCs
        if (*osdConfig()).esc_temp_alarm as libc::c_int != -128i32 &&
               (*escDataCombined).temperature as libc::c_int >=
                   (*osdConfig()).esc_temp_alarm as libc::c_int {
            blinkBits[(OSD_ESC_TMP as libc::c_int / 32i32) as usize] |=
                (1i32 << OSD_ESC_TMP as libc::c_int % 32i32) as libc::c_uint
        } else {
            blinkBits[(OSD_ESC_TMP as libc::c_int / 32i32) as usize] &=
                !(1i32 << OSD_ESC_TMP as libc::c_int % 32i32) as libc::c_uint
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn osdResetAlarms() {
    blinkBits[(OSD_RSSI_VALUE as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_RSSI_VALUE as libc::c_int % 32i32) as libc::c_uint;
    blinkBits[(OSD_MAIN_BATT_VOLTAGE as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_MAIN_BATT_VOLTAGE as libc::c_int % 32i32) as
            libc::c_uint;
    blinkBits[(OSD_WARNINGS as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_WARNINGS as libc::c_int % 32i32) as libc::c_uint;
    blinkBits[(OSD_GPS_SATS as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_GPS_SATS as libc::c_int % 32i32) as libc::c_uint;
    blinkBits[(OSD_MAH_DRAWN as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_MAH_DRAWN as libc::c_int % 32i32) as libc::c_uint;
    blinkBits[(OSD_ALTITUDE as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_ALTITUDE as libc::c_int % 32i32) as libc::c_uint;
    blinkBits[(OSD_AVG_CELL_VOLTAGE as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_AVG_CELL_VOLTAGE as libc::c_int % 32i32) as
            libc::c_uint;
    blinkBits[(OSD_MAIN_BATT_USAGE as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_MAIN_BATT_USAGE as libc::c_int % 32i32) as libc::c_uint;
    blinkBits[(OSD_ITEM_TIMER_1 as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_ITEM_TIMER_1 as libc::c_int % 32i32) as libc::c_uint;
    blinkBits[(OSD_ITEM_TIMER_2 as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_ITEM_TIMER_2 as libc::c_int % 32i32) as libc::c_uint;
    blinkBits[(OSD_REMAINING_TIME_ESTIMATE as libc::c_int / 32i32) as usize]
        &=
        !(1i32 << OSD_REMAINING_TIME_ESTIMATE as libc::c_int % 32i32) as
            libc::c_uint;
    blinkBits[(OSD_ESC_TMP as libc::c_int / 32i32) as usize] &=
        !(1i32 << OSD_ESC_TMP as libc::c_int % 32i32) as libc::c_uint;
}
unsafe extern "C" fn osdResetStats() {
    stats.max_current = 0i32 as int16_t;
    stats.max_speed = 0i32 as int16_t;
    stats.min_voltage = 500i32 as int16_t;
    stats.min_rssi = 99i32 as int16_t;
    stats.max_altitude = 0i32;
    stats.max_distance = 0i32 as int16_t;
    stats.armed_time = 0i32 as timeUs_t;
}
unsafe extern "C" fn osdUpdateStats() {
    let mut value: int16_t = 0i32 as int16_t;
    if (stats.max_speed as libc::c_int) < value as libc::c_int {
        stats.max_speed = value
    }
    value = getBatteryVoltage() as int16_t;
    if stats.min_voltage as libc::c_int > value as libc::c_int {
        stats.min_voltage = value
    }
    value = (getAmperage() / 100i32) as int16_t;
    if (stats.max_current as libc::c_int) < value as libc::c_int {
        stats.max_current = value
    }
    value = getRssiPercent() as int16_t;
    if stats.min_rssi as libc::c_int > value as libc::c_int {
        stats.min_rssi = value
    }
    let mut altitude: libc::c_int = getEstimatedAltitude();
    if stats.max_altitude < altitude { stats.max_altitude = altitude };
}
unsafe extern "C" fn osdGetBlackboxStatusString(mut buff: *mut libc::c_char) {
    let mut storageDeviceIsWorking: bool = 0i32 != 0;
    let mut storageUsed: uint32_t = 0i32 as uint32_t;
    let mut storageTotal: uint32_t = 0i32 as uint32_t;
    match (*blackboxConfig()).device as libc::c_int {
        1 => {
            storageDeviceIsWorking = flashfsIsSupported();
            if storageDeviceIsWorking {
                let mut geometry: *const flashGeometry_t =
                    flashfsGetGeometry();
                storageTotal =
                    (*geometry).totalSize.wrapping_div(1024i32 as
                                                           libc::c_uint);
                storageUsed =
                    flashfsGetOffset().wrapping_div(1024i32 as libc::c_uint)
            }
        }
        _ => { }
    }
    if storageDeviceIsWorking {
        let storageUsedPercent: uint16_t =
            storageUsed.wrapping_mul(100i32 as
                                         libc::c_uint).wrapping_div(storageTotal)
                as uint16_t;
        tfp_sprintf(buff, b"%d%%\x00" as *const u8 as *const libc::c_char,
                    storageUsedPercent as libc::c_int);
    } else {
        tfp_sprintf(buff, b"FAULT\x00" as *const u8 as *const libc::c_char);
    };
}
unsafe extern "C" fn osdDisplayStatisticLabel(mut y: uint8_t,
                                              mut text: *const libc::c_char,
                                              mut value:
                                                  *const libc::c_char) {
    displayWrite(osdDisplayPort, 2i32 as uint8_t, y, text);
    displayWrite(osdDisplayPort, 20i32 as uint8_t, y,
                 b":\x00" as *const u8 as *const libc::c_char);
    displayWrite(osdDisplayPort, 22i32 as uint8_t, y, value);
}
/*
 * Test if there's some stat enabled
 */
unsafe extern "C" fn isSomeStatEnabled() -> bool {
    return (*osdConfig()).enabled_stats != 0i32 as libc::c_uint;
}
// *** IMPORTANT ***
// The order of the OSD stats as displayed on-screen must match the osd_stats_e enumeration.
// This is because the fields are presented in the configurator in the order of the enumeration
// and we want the configuration order to match the on-screen display order.  If you change the
// display order you *must* update the osd_stats_e enumeration to match. Additionally the
// changes to the stats display order *must* be implemented in the configurator otherwise the
// stats selections will not be populated correctly and the settings will become corrupted.
unsafe extern "C" fn osdShowStats(mut endBatteryVoltage: uint16_t) {
    let mut top: uint8_t = 2i32 as uint8_t;
    let mut buff: [libc::c_char; 32] = [0; 32];
    displayClearScreen(osdDisplayPort);
    let fresh6 = top;
    top = top.wrapping_add(1);
    displayWrite(osdDisplayPort, 2i32 as uint8_t, fresh6,
                 b"  --- STATS ---\x00" as *const u8 as *const libc::c_char);
    if osdStatGetState(OSD_STAT_RTC_DATE_TIME as libc::c_int as uint8_t) {
        let mut success: bool = 0i32 != 0;
        if !success {
            tfp_sprintf(buff.as_mut_ptr(),
                        b"NO RTC\x00" as *const u8 as *const libc::c_char);
        }
        let fresh7 = top;
        top = top.wrapping_add(1);
        displayWrite(osdDisplayPort, 2i32 as uint8_t, fresh7,
                     buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_TIMER_1 as libc::c_int as uint8_t) {
        osdFormatTimer(buff.as_mut_ptr(), 0i32 != 0,
                       if (*osdConfig()).timers[OSD_TIMER_1 as libc::c_int as
                                                    usize] as libc::c_int &
                              0xfi32 == OSD_TIMER_SRC_ON as libc::c_int {
                           0i32
                       } else { 1i32 } != 0, OSD_TIMER_1 as libc::c_int);
        let fresh8 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh8,
                                 osdTimerSourceNames[((*osdConfig()).timers[OSD_TIMER_1
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                usize]
                                                          as libc::c_int &
                                                          0xfi32) as usize],
                                 buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_TIMER_2 as libc::c_int as uint8_t) {
        osdFormatTimer(buff.as_mut_ptr(), 0i32 != 0,
                       if (*osdConfig()).timers[OSD_TIMER_2 as libc::c_int as
                                                    usize] as libc::c_int &
                              0xfi32 == OSD_TIMER_SRC_ON as libc::c_int {
                           0i32
                       } else { 1i32 } != 0, OSD_TIMER_2 as libc::c_int);
        let fresh9 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh9,
                                 osdTimerSourceNames[((*osdConfig()).timers[OSD_TIMER_2
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                usize]
                                                          as libc::c_int &
                                                          0xfi32) as usize],
                                 buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_MAX_SPEED as libc::c_int as uint8_t) as
           libc::c_int != 0 &&
           stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
        itoa(stats.max_speed as libc::c_int, buff.as_mut_ptr(), 10i32);
        let fresh10 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh10,
                                 b"MAX SPEED\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_MAX_DISTANCE as libc::c_int as uint8_t) {
        tfp_sprintf(buff.as_mut_ptr(),
                    b"%d%c\x00" as *const u8 as *const libc::c_char,
                    osdGetMetersToSelectedUnit(stats.max_distance as int32_t),
                    osdGetMetersToSelectedUnitSymbol() as libc::c_int);
        let fresh11 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh11,
                                 b"MAX DISTANCE\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_MIN_BATTERY as libc::c_int as uint8_t) {
        tfp_sprintf(buff.as_mut_ptr(),
                    b"%d.%1d%c\x00" as *const u8 as *const libc::c_char,
                    stats.min_voltage as libc::c_int / 10i32,
                    stats.min_voltage as libc::c_int % 10i32, 0x6i32);
        let fresh12 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh12,
                                 b"MIN BATTERY\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_END_BATTERY as libc::c_int as uint8_t) {
        tfp_sprintf(buff.as_mut_ptr(),
                    b"%d.%1d%c\x00" as *const u8 as *const libc::c_char,
                    endBatteryVoltage as libc::c_int / 10i32,
                    endBatteryVoltage as libc::c_int % 10i32, 0x6i32);
        let fresh13 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh13,
                                 b"END BATTERY\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_BATTERY as libc::c_int as uint8_t) {
        tfp_sprintf(buff.as_mut_ptr(),
                    b"%d.%1d%c\x00" as *const u8 as *const libc::c_char,
                    getBatteryVoltage() as libc::c_int / 10i32,
                    getBatteryVoltage() as libc::c_int % 10i32, 0x6i32);
        let fresh14 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh14,
                                 b"BATTERY\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_MIN_RSSI as libc::c_int as uint8_t) {
        itoa(stats.min_rssi as libc::c_int, buff.as_mut_ptr(), 10i32);
        strcat(buff.as_mut_ptr(),
               b"%\x00" as *const u8 as *const libc::c_char);
        let fresh15 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh15,
                                 b"MIN RSSI\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if (*batteryConfig()).currentMeterSource as libc::c_uint !=
           CURRENT_METER_NONE as libc::c_int as libc::c_uint {
        if osdStatGetState(OSD_STAT_MAX_CURRENT as libc::c_int as uint8_t) {
            itoa(stats.max_current as libc::c_int, buff.as_mut_ptr(), 10i32);
            strcat(buff.as_mut_ptr(),
                   b"A\x00" as *const u8 as *const libc::c_char);
            let fresh16 = top;
            top = top.wrapping_add(1);
            osdDisplayStatisticLabel(fresh16,
                                     b"MAX CURRENT\x00" as *const u8 as
                                         *const libc::c_char,
                                     buff.as_mut_ptr());
        }
        if osdStatGetState(OSD_STAT_USED_MAH as libc::c_int as uint8_t) {
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%d%c\x00" as *const u8 as *const libc::c_char,
                        getMAhDrawn(), 0x7i32);
            let fresh17 = top;
            top = top.wrapping_add(1);
            osdDisplayStatisticLabel(fresh17,
                                     b"USED MAH\x00" as *const u8 as
                                         *const libc::c_char,
                                     buff.as_mut_ptr());
        }
    }
    if osdStatGetState(OSD_STAT_MAX_ALTITUDE as libc::c_int as uint8_t) {
        osdFormatAltitudeString(buff.as_mut_ptr(), stats.max_altitude);
        let fresh18 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh18,
                                 b"MAX ALTITUDE\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_BLACKBOX as libc::c_int as uint8_t) as
           libc::c_int != 0 && (*blackboxConfig()).device as libc::c_int != 0
           &&
           (*blackboxConfig()).device as libc::c_int !=
               BLACKBOX_DEVICE_SERIAL as libc::c_int {
        osdGetBlackboxStatusString(buff.as_mut_ptr());
        let fresh19 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh19,
                                 b"BLACKBOX\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_BLACKBOX_NUMBER as libc::c_int as uint8_t) as
           libc::c_int != 0 && (*blackboxConfig()).device as libc::c_int != 0
           &&
           (*blackboxConfig()).device as libc::c_int !=
               BLACKBOX_DEVICE_SERIAL as libc::c_int {
        itoa(blackboxGetLogNumber() as libc::c_int, buff.as_mut_ptr(), 10i32);
        let fresh20 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh20,
                                 b"BB LOG NUM\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    };
}
unsafe extern "C" fn osdShowArmed() {
    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 12i32 as uint8_t, 7i32 as uint8_t,
                 b"ARMED\x00" as *const u8 as *const libc::c_char);
}
unsafe extern "C" fn osdRefresh(mut currentTimeUs: timeUs_t) {
    static mut lastTimeUs: timeUs_t = 0i32 as timeUs_t;
    static mut osdStatsEnabled: bool = 0i32 != 0;
    static mut osdStatsVisible: bool = 0i32 != 0;
    static mut osdStatsRefreshTimeUs: timeUs_t = 0;
    static mut endBatteryVoltage: uint16_t = 0;
    // detect arm/disarm
    if armState as libc::c_int !=
           armingFlags as libc::c_int & ARMED as libc::c_int {
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            osdStatsEnabled = 0i32 != 0;
            osdStatsVisible = 0i32 != 0;
            osdResetStats();
            osdShowArmed();
            resumeRefreshAt =
                currentTimeUs.wrapping_add((1000i32 * 1000i32 / 2i32) as
                                               libc::c_uint)
        } else if isSomeStatEnabled() as libc::c_int != 0 &&
                      (getArmingDisableFlags() as libc::c_uint &
                           ARMING_DISABLED_RUNAWAY_TAKEOFF as libc::c_int as
                               libc::c_uint == 0 ||
                           (*osdConfig()).item_pos[OSD_WARNINGS as libc::c_int
                                                       as usize] as
                               libc::c_int & 0x800i32 == 0) {
            // suppress stats if runaway takeoff triggered disarm and WARNINGS element is visible
            osdStatsEnabled = 1i32 != 0;
            resumeRefreshAt =
                currentTimeUs.wrapping_add((60i32 * 1000i32 * 1000i32) as
                                               libc::c_uint);
            endBatteryVoltage = getBatteryVoltage()
        }
        armState =
            (armingFlags as libc::c_int & ARMED as libc::c_int) as uint8_t
    }
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        osdUpdateStats();
        let mut deltaT: timeUs_t = currentTimeUs.wrapping_sub(lastTimeUs);
        flyTime =
            (flyTime as libc::c_uint).wrapping_add(deltaT) as timeUs_t as
                timeUs_t;
        stats.armed_time =
            (stats.armed_time as libc::c_uint).wrapping_add(deltaT) as
                timeUs_t as timeUs_t
    } else if osdStatsEnabled {
        // handle showing/hiding stats based on OSD disable switch position
        if displayIsGrabbed(osdDisplayPort) {
            osdStatsEnabled = 0i32 != 0;
            resumeRefreshAt = 0i32 as timeUs_t;
            stats.armed_time = 0i32 as timeUs_t
        } else if IS_RC_MODE_ACTIVE(BOXOSD) as libc::c_int != 0 &&
                      osdStatsVisible as libc::c_int != 0 {
            osdStatsVisible = 0i32 != 0;
            displayClearScreen(osdDisplayPort);
        } else if !IS_RC_MODE_ACTIVE(BOXOSD) {
            if !osdStatsVisible {
                osdStatsVisible = 1i32 != 0;
                osdStatsRefreshTimeUs = 0i32 as timeUs_t
            }
            if currentTimeUs >= osdStatsRefreshTimeUs {
                osdStatsRefreshTimeUs =
                    currentTimeUs.wrapping_add((1000i32 * 1000i32) as
                                                   libc::c_uint);
                osdShowStats(endBatteryVoltage);
            }
        }
    }
    lastTimeUs = currentTimeUs;
    if resumeRefreshAt != 0 {
        if cmp32(currentTimeUs, resumeRefreshAt) < 0i32 {
            // in timeout period, check sticks for activity to resume display.
            if rcData[THROTTLE as libc::c_int as usize] as libc::c_int >
                   1750i32 ||
                   rcData[PITCH as libc::c_int as usize] as libc::c_int >
                       1750i32 {
                resumeRefreshAt = currentTimeUs
            }
            displayHeartbeat(osdDisplayPort);
            return
        } else {
            displayClearScreen(osdDisplayPort);
            resumeRefreshAt = 0i32 as timeUs_t;
            osdStatsEnabled = 0i32 != 0;
            stats.armed_time = 0i32 as timeUs_t
        }
    }
    blinkState =
        currentTimeUs.wrapping_div(200000i32 as
                                       libc::c_uint).wrapping_rem(2i32 as
                                                                      libc::c_uint)
            != 0;
    if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
        escDataCombined = getEscSensorData(255i32 as uint8_t)
    }
    lastArmState = armingFlags as libc::c_int & ARMED as libc::c_int != 0;
}
/*
 * Called periodically by the scheduler
 */
#[no_mangle]
pub unsafe extern "C" fn osdUpdate(mut currentTimeUs: timeUs_t) {
    static mut counter: uint32_t = 0i32 as uint32_t;
    if isBeeperOn() { showVisualBeeper = 1i32 != 0 }
    // MAX7456_DMA_CHANNEL_TX
    static mut idlecounter: uint32_t = 0i32 as uint32_t;
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        let fresh21 = idlecounter;
        idlecounter = idlecounter.wrapping_add(1);
        if fresh21.wrapping_rem(4i32 as libc::c_uint) != 0i32 as libc::c_uint
           {
            return
        }
    }
    // redraw values in buffer
    // MWOSD @ 115200 baud (
    if counter.wrapping_rem(10i32 as libc::c_uint) == 0i32 as libc::c_uint {
        osdRefresh(currentTimeUs);
        showVisualBeeper = 0i32 != 0
    } else {
        // rest of time redraw screen 10 chars per idle so it doesn't lock the main idle
        displayDrawScreen(osdDisplayPort);
    }
    counter = counter.wrapping_add(1);
}
// USE_OSD
