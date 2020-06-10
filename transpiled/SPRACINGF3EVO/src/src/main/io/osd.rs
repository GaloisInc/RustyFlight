use ::libc;
extern "C" {
    #[no_mangle]
    fn abs(_: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strcpy(_: *mut libc::c_char, _: *const libc::c_char)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strcat(_: *mut libc::c_char, _: *const libc::c_char)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn toupper(_: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn sqrtf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn ceilf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    static mut blackboxConfig_System: blackboxConfig_t;
    #[no_mangle]
    fn blackboxGetLogNumber() -> libc::c_uint;
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
    fn displayIsGrabbed(instance: *const displayPort_t) -> bool;
    #[no_mangle]
    fn displayClearScreen(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayDrawScreen(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayWrite(instance: *mut displayPort_t, x: uint8_t, y: uint8_t,
                    s: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn displayWriteChar(instance: *mut displayPort_t, x: uint8_t, y: uint8_t,
                        c: uint8_t) -> libc::c_int;
    // Device management
    #[no_mangle]
    fn cmsDisplayPortRegister(pDisplay: *mut displayPort_t) -> bool;
    #[no_mangle]
    fn displayHeartbeat(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayResync(instance: *mut displayPort_t);
    #[no_mangle]
    fn scaleRange(x: libc::c_int, srcFrom: libc::c_int, srcTo: libc::c_int,
                  destFrom: libc::c_int, destTo: libc::c_int) -> libc::c_int;
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
    fn sdcard_isInserted() -> bool;
    #[no_mangle]
    fn sdcard_isFunctional() -> bool;
    #[no_mangle]
    fn sdcard_getMetadata() -> *const sdcardMetadata_t;
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    static mut pilotConfig_System: pilotConfig_t;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    fn getCurrentPidProfileIndex() -> uint8_t;
    #[no_mangle]
    fn getCurrentControlRateProfileIndex() -> uint8_t;
    #[no_mangle]
    fn isFlipOverAfterCrashMode() -> bool;
    #[no_mangle]
    fn isTryingToArm() -> bool;
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    fn isAirmodeActive() -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut armingDisableFlagNames: [*const libc::c_char; 20];
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn unsetArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn isArmingDisabled() -> bool;
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn getEstimatedAltitude() -> int32_t;
    #[no_mangle]
    fn getEstimatedVario() -> int16_t;
    #[no_mangle]
    static mut accAverage: [libc::c_float; 3];
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    fn pidOsdAntiGravityActive() -> bool;
    #[no_mangle]
    fn afatfs_getContiguousFreeSpace() -> uint32_t;
    #[no_mangle]
    fn afatfs_getFilesystemState() -> afatfsFilesystemState_e;
    #[no_mangle]
    fn isBeeperOn() -> bool;
    #[no_mangle]
    fn getLastDshotBeaconCommandTimeUs() -> timeUs_t;
    #[no_mangle]
    fn vtxCommonDevice() -> *mut vtxDevice_t;
    #[no_mangle]
    fn vtxCommonGetPowerIndex(vtxDevice: *const vtxDevice_t,
                              pIndex: *mut uint8_t) -> bool;
    #[no_mangle]
    static vtx58ChannelNames: [*const libc::c_char; 0];
    #[no_mangle]
    static vtx58BandLetter: [libc::c_char; 0];
    #[no_mangle]
    static mut vtxSettingsConfig_System: vtxSettingsConfig_t;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn getRssi() -> uint16_t;
    #[no_mangle]
    fn getRssiPercent() -> uint8_t;
    #[no_mangle]
    static mut acc: acc_t;
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
    // voltage
    // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
    // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
    // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
    // Percentage of throttle when lvc is triggered
    // source of battery voltage meter used, either ADC or ESC
    // current
    // source of battery current meter used, either ADC, Virtual or ESC
    // mAh
    // warnings / alerts
    // Issue alerts based on VBat readings
    // Issue alerts based on total power consumption
    // Percentage of remaining capacity that should trigger a battery warning
    // hysteresis for alarm, default 1 = 0.1V
    // Cell voltage at which the battery is deemed to be "full" 0.1V units, default is 41 (4.1V)
    #[no_mangle]
    fn getBatteryState() -> batteryState_e;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn getBatteryAverageCellVoltage() -> uint16_t;
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getEscSensorData(motorNumber: uint8_t) -> *mut escSensorData_t;
    #[no_mangle]
    fn calcEscRpm(erpm: libc::c_int) -> libc::c_int;
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
pub type size_t = libc::c_ulong;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
/* base */
/* size */
// microsecond time
pub type timeUs_t = uint32_t;
pub type BlackboxDevice = libc::c_uint;
pub const BLACKBOX_DEVICE_SERIAL: BlackboxDevice = 3;
pub const BLACKBOX_DEVICE_SDCARD: BlackboxDevice = 2;
pub const BLACKBOX_DEVICE_NONE: BlackboxDevice = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
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
pub type IO_t = *mut libc::c_void;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardMetadata_s {
    pub numBlocks: uint32_t,
    pub oemID: uint16_t,
    pub manufacturerID: uint8_t,
    pub productName: [libc::c_char; 5],
    pub productSerial: uint32_t,
    pub productRevisionMajor: uint8_t,
    pub productRevisionMinor: uint8_t,
    pub productionYear: uint16_t,
    pub productionMonth: uint8_t,
}
// CMS state
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
pub type sdcardMetadata_t = sdcardMetadata_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pilotConfig_s {
    pub name: [libc::c_char; 17],
}
pub type pilotConfig_t = pilotConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidProfile_s {
    pub yaw_lowpass_hz: uint16_t,
    pub dterm_lowpass_hz: uint16_t,
    pub dterm_notch_hz: uint16_t,
    pub dterm_notch_cutoff: uint16_t,
    pub pid: [pidf_t; 5],
    pub dterm_filter_type: uint8_t,
    pub itermWindupPointPercent: uint8_t,
    pub pidSumLimit: uint16_t,
    pub pidSumLimitYaw: uint16_t,
    pub pidAtMinThrottle: uint8_t,
    pub levelAngleLimit: uint8_t,
    pub horizon_tilt_effect: uint8_t,
    pub horizon_tilt_expert_mode: uint8_t,
    pub antiGravityMode: uint8_t,
    pub itermThrottleThreshold: uint16_t,
    pub itermAcceleratorGain: uint16_t,
    pub yawRateAccelLimit: uint16_t,
    pub rateAccelLimit: uint16_t,
    pub crash_dthreshold: uint16_t,
    pub crash_gthreshold: uint16_t,
    pub crash_setpoint_threshold: uint16_t,
    pub crash_time: uint16_t,
    pub crash_delay: uint16_t,
    pub crash_recovery_angle: uint8_t,
    pub crash_recovery_rate: uint8_t,
    pub vbatPidCompensation: uint8_t,
    pub feedForwardTransition: uint8_t,
    pub crash_limit_yaw: uint16_t,
    pub itermLimit: uint16_t,
    pub dterm_lowpass2_hz: uint16_t,
    pub crash_recovery: uint8_t,
    pub throttle_boost: uint8_t,
    pub throttle_boost_cutoff: uint8_t,
    pub iterm_rotation: uint8_t,
    pub smart_feedforward: uint8_t,
    pub iterm_relax_type: uint8_t,
    pub iterm_relax_cutoff: uint8_t,
    pub iterm_relax: uint8_t,
    pub acro_trainer_angle_limit: uint8_t,
    pub acro_trainer_debug_axis: uint8_t,
    pub acro_trainer_gain: uint8_t,
    pub acro_trainer_lookahead_ms: uint16_t,
    pub abs_control_gain: uint8_t,
    pub abs_control_limit: uint8_t,
    pub abs_control_error_limit: uint8_t,
}
pub type pidf_t = pidf_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidf_s {
    pub P: uint8_t,
    pub I: uint8_t,
    pub D: uint8_t,
    pub F: uint16_t,
}
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
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_5,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_5 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type C2RustUnnamed_6 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_6 = 5;
pub const PID_MAG: C2RustUnnamed_6 = 4;
pub const PID_LEVEL: C2RustUnnamed_6 = 3;
pub const PID_YAW: C2RustUnnamed_6 = 2;
pub const PID_PITCH: C2RustUnnamed_6 = 1;
pub const PID_ROLL: C2RustUnnamed_6 = 0;
pub type afatfsFilesystemState_e = libc::c_uint;
pub const AFATFS_FILESYSTEM_STATE_READY: afatfsFilesystemState_e = 3;
pub const AFATFS_FILESYSTEM_STATE_INITIALIZATION: afatfsFilesystemState_e = 2;
pub const AFATFS_FILESYSTEM_STATE_FATAL: afatfsFilesystemState_e = 1;
pub const AFATFS_FILESYSTEM_STATE_UNKNOWN: afatfsFilesystemState_e = 0;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const OSD_ITEM_COUNT: C2RustUnnamed_7 = 43;
pub const OSD_G_FORCE: C2RustUnnamed_7 = 42;
pub const OSD_ANTI_GRAVITY: C2RustUnnamed_7 = 41;
pub const OSD_CORE_TEMPERATURE: C2RustUnnamed_7 = 40;
pub const OSD_ADJUSTMENT_RANGE: C2RustUnnamed_7 = 39;
pub const OSD_RTC_DATETIME: C2RustUnnamed_7 = 38;
pub const OSD_REMAINING_TIME_ESTIMATE: C2RustUnnamed_7 = 37;
pub const OSD_ESC_RPM: C2RustUnnamed_7 = 36;
pub const OSD_ESC_TMP: C2RustUnnamed_7 = 35;
pub const OSD_COMPASS_BAR: C2RustUnnamed_7 = 34;
pub const OSD_NUMERICAL_VARIO: C2RustUnnamed_7 = 33;
pub const OSD_NUMERICAL_HEADING: C2RustUnnamed_7 = 32;
pub const OSD_HOME_DIST: C2RustUnnamed_7 = 31;
pub const OSD_HOME_DIR: C2RustUnnamed_7 = 30;
pub const OSD_DISARMED: C2RustUnnamed_7 = 29;
pub const OSD_MAIN_BATT_USAGE: C2RustUnnamed_7 = 28;
pub const OSD_ROLL_ANGLE: C2RustUnnamed_7 = 27;
pub const OSD_PITCH_ANGLE: C2RustUnnamed_7 = 26;
pub const OSD_DEBUG: C2RustUnnamed_7 = 25;
pub const OSD_GPS_LAT: C2RustUnnamed_7 = 24;
pub const OSD_GPS_LON: C2RustUnnamed_7 = 23;
pub const OSD_AVG_CELL_VOLTAGE: C2RustUnnamed_7 = 22;
pub const OSD_WARNINGS: C2RustUnnamed_7 = 21;
pub const OSD_PIDRATE_PROFILE: C2RustUnnamed_7 = 20;
pub const OSD_POWER: C2RustUnnamed_7 = 19;
pub const OSD_YAW_PIDS: C2RustUnnamed_7 = 18;
pub const OSD_PITCH_PIDS: C2RustUnnamed_7 = 17;
pub const OSD_ROLL_PIDS: C2RustUnnamed_7 = 16;
pub const OSD_ALTITUDE: C2RustUnnamed_7 = 15;
pub const OSD_GPS_SATS: C2RustUnnamed_7 = 14;
pub const OSD_GPS_SPEED: C2RustUnnamed_7 = 13;
pub const OSD_MAH_DRAWN: C2RustUnnamed_7 = 12;
pub const OSD_CURRENT_DRAW: C2RustUnnamed_7 = 11;
pub const OSD_VTX_CHANNEL: C2RustUnnamed_7 = 10;
pub const OSD_THROTTLE_POS: C2RustUnnamed_7 = 9;
pub const OSD_CRAFT_NAME: C2RustUnnamed_7 = 8;
pub const OSD_FLYMODE: C2RustUnnamed_7 = 7;
pub const OSD_ITEM_TIMER_2: C2RustUnnamed_7 = 6;
pub const OSD_ITEM_TIMER_1: C2RustUnnamed_7 = 5;
pub const OSD_HORIZON_SIDEBARS: C2RustUnnamed_7 = 4;
pub const OSD_ARTIFICIAL_HORIZON: C2RustUnnamed_7 = 3;
pub const OSD_CROSSHAIRS: C2RustUnnamed_7 = 2;
pub const OSD_MAIN_BATT_VOLTAGE: C2RustUnnamed_7 = 1;
pub const OSD_RSSI_VALUE: C2RustUnnamed_7 = 0;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const OSD_STAT_COUNT: C2RustUnnamed_8 = 14;
pub const OSD_STAT_BLACKBOX_NUMBER: C2RustUnnamed_8 = 13;
pub const OSD_STAT_BLACKBOX: C2RustUnnamed_8 = 12;
pub const OSD_STAT_MAX_ALTITUDE: C2RustUnnamed_8 = 11;
pub const OSD_STAT_USED_MAH: C2RustUnnamed_8 = 10;
pub const OSD_STAT_MAX_CURRENT: C2RustUnnamed_8 = 9;
pub const OSD_STAT_MIN_RSSI: C2RustUnnamed_8 = 8;
pub const OSD_STAT_BATTERY: C2RustUnnamed_8 = 7;
pub const OSD_STAT_END_BATTERY: C2RustUnnamed_8 = 6;
pub const OSD_STAT_MIN_BATTERY: C2RustUnnamed_8 = 5;
pub const OSD_STAT_MAX_DISTANCE: C2RustUnnamed_8 = 4;
pub const OSD_STAT_MAX_SPEED: C2RustUnnamed_8 = 3;
pub const OSD_STAT_TIMER_2: C2RustUnnamed_8 = 2;
pub const OSD_STAT_TIMER_1: C2RustUnnamed_8 = 1;
pub const OSD_STAT_RTC_DATE_TIME: C2RustUnnamed_8 = 0;
pub type osd_unit_e = libc::c_uint;
pub const OSD_UNIT_METRIC: osd_unit_e = 1;
pub const OSD_UNIT_IMPERIAL: osd_unit_e = 0;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const OSD_TIMER_COUNT: C2RustUnnamed_9 = 2;
pub const OSD_TIMER_2: C2RustUnnamed_9 = 1;
pub const OSD_TIMER_1: C2RustUnnamed_9 = 0;
pub type osd_timer_source_e = libc::c_uint;
pub const OSD_TIMER_SRC_COUNT: osd_timer_source_e = 3;
pub const OSD_TIMER_SRC_LAST_ARMED: osd_timer_source_e = 2;
pub const OSD_TIMER_SRC_TOTAL_ARMED: osd_timer_source_e = 1;
pub const OSD_TIMER_SRC_ON: osd_timer_source_e = 0;
pub type osd_timer_precision_e = libc::c_uint;
pub const OSD_TIMER_PREC_COUNT: osd_timer_precision_e = 2;
pub const OSD_TIMER_PREC_HUNDREDTHS: osd_timer_precision_e = 1;
pub const OSD_TIMER_PREC_SECOND: osd_timer_precision_e = 0;
pub type C2RustUnnamed_10 = libc::c_uint;
pub const OSD_WARNING_COUNT: C2RustUnnamed_10 = 9;
pub const OSD_WARNING_RC_SMOOTHING: C2RustUnnamed_10 = 8;
pub const OSD_WARNING_CORE_TEMPERATURE: C2RustUnnamed_10 = 7;
pub const OSD_WARNING_ESC_FAIL: C2RustUnnamed_10 = 6;
pub const OSD_WARNING_CRASH_FLIP: C2RustUnnamed_10 = 5;
pub const OSD_WARNING_VISUAL_BEEPER: C2RustUnnamed_10 = 4;
pub const OSD_WARNING_BATTERY_CRITICAL: C2RustUnnamed_10 = 3;
pub const OSD_WARNING_BATTERY_WARNING: C2RustUnnamed_10 = 2;
pub const OSD_WARNING_BATTERY_NOT_FULL: C2RustUnnamed_10 = 1;
pub const OSD_WARNING_ARMING_DISABLE: C2RustUnnamed_10 = 0;
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct escSensorData_t {
    pub dataAge: uint8_t,
    pub temperature: int8_t,
    pub voltage: int16_t,
    pub current: int32_t,
    pub consumption: int32_t,
    pub rpm: int16_t,
}
pub type batteryConfig_t = batteryConfig_s;
#[derive(Copy, Clone)]
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
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const BATTERY_OK: batteryState_e = 0;
pub type assert_failed_osd_warnings_size_exceeds_buffer_size
    =
    [libc::c_char; 1];
pub type accDev_t = accDev_s;
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
pub type busDevice_t = busDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_11,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_11 {
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
/* Card capacity in 512-byte blocks*/
// Alarms
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
#[derive(Copy, Clone)]
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
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
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
// driver-provided alignment
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
pub type acc_t = acc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
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
pub type vtxSettingsConfig_t = vtxSettingsConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxSettingsConfig_s {
    pub band: uint8_t,
    pub channel: uint8_t,
    pub power: uint8_t,
    pub freq: uint16_t,
    pub pitModeFreq: uint16_t,
    pub lowPowerDisarm: uint8_t,
}
pub type vtxDevice_t = vtxDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxDevice_s {
    pub vTable: *const vtxVTable_s,
    pub capability: vtxDeviceCapability_t,
    pub frequencyTable: *mut uint16_t,
    pub bandNames: *mut *mut libc::c_char,
    pub channelNames: *mut *mut libc::c_char,
    pub powerNames: *mut *mut libc::c_char,
    pub frequency: uint16_t,
    pub band: uint8_t,
    pub channel: uint8_t,
    pub powerIndex: uint8_t,
    pub pitMode: uint8_t,
}
// 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
// 1-8
// 0 = lowest
// sets freq in MHz if band=0
// sets out-of-range pitmode frequency
// min power while disarmed
// Array of [bandCount][channelCount]
// char *bandNames[bandCount]
// char *channelNames[channelCount]
// char *powerNames[powerCount]
// Band = 1, 1-based
// CH1 = 1, 1-based
// Lowest/Off = 0
// 0 = non-PIT, 1 = PIT
// VTX magic numbers
// RTC6705 RF Power index "---", 25 or 200 mW
// SmartAudio "---", 25, 200, 500, 800 mW
// Tramp "---", 25, 100, 200, 400, 600 mW
pub type vtxDeviceCapability_t = vtxDeviceCapability_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxDeviceCapability_s {
    pub bandCount: uint8_t,
    pub channelCount: uint8_t,
    pub powerCount: uint8_t,
    pub filler: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxVTable_s {
    pub process: Option<unsafe extern "C" fn(_: *mut vtxDevice_t, _: timeUs_t)
                            -> ()>,
    pub getDeviceType: Option<unsafe extern "C" fn(_: *const vtxDevice_t)
                                  -> vtxDevType_e>,
    pub isReady: Option<unsafe extern "C" fn(_: *const vtxDevice_t) -> bool>,
    pub setBandAndChannel: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                       _: uint8_t, _: uint8_t)
                                      -> ()>,
    pub setPowerByIndex: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                     _: uint8_t) -> ()>,
    pub setPitMode: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                _: uint8_t) -> ()>,
    pub setFrequency: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                  _: uint16_t) -> ()>,
    pub getBandAndChannel: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                       _: *mut uint8_t,
                                                       _: *mut uint8_t)
                                      -> bool>,
    pub getPowerIndex: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                   _: *mut uint8_t) -> bool>,
    pub getPitMode: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                _: *mut uint8_t) -> bool>,
    pub getFrequency: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                  _: *mut uint16_t) -> bool>,
}
pub type vtxDevType_e = libc::c_uint;
pub const VTXDEV_UNKNOWN: vtxDevType_e = 255;
pub const VTXDEV_TRAMP: vtxDevType_e = 4;
pub const VTXDEV_SMARTAUDIO: vtxDevType_e = 3;
pub const VTXDEV_RTC6705: vtxDevType_e = 1;
pub const VTXDEV_UNSUPPORTED: vtxDevType_e = 0;
pub type statistic_t = statistic_s;
#[derive(Copy, Clone)]
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
pub const SENSOR_ACC: C2RustUnnamed_12 = 2;
pub type C2RustUnnamed_12 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_12 = 64;
pub const SENSOR_GPS: C2RustUnnamed_12 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_12 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_12 = 16;
pub const SENSOR_MAG: C2RustUnnamed_12 = 8;
pub const SENSOR_BARO: C2RustUnnamed_12 = 4;
pub const SENSOR_GYRO: C2RustUnnamed_12 = 1;
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
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn pilotConfig() -> *const pilotConfig_t {
    return &mut pilotConfig_System;
}
#[inline]
unsafe extern "C" fn osdConfig() -> *const osdConfig_t {
    return &mut osdConfig_System;
}
#[inline]
unsafe extern "C" fn osdConfigMutable() -> *mut osdConfig_t {
    return &mut osdConfig_System;
}
#[inline]
unsafe extern "C" fn vtxSettingsConfig() -> *const vtxSettingsConfig_t {
    return &mut vtxSettingsConfig_System;
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
// /10
// /10
// Blink control
static mut blinkState: bool = 1 as libc::c_int != 0;
static mut showVisualBeeper: bool = 0 as libc::c_int != 0;
static mut blinkBits: [uint32_t; 2] = [0; 2];
// Things in both OSD and CMS
static mut flyTime: timeUs_t = 0 as libc::c_int as timeUs_t;
static mut stats: statistic_t =
    statistic_t{armed_time: 0,
                max_speed: 0,
                min_voltage: 0,
                max_current: 0,
                min_rssi: 0,
                max_altitude: 0,
                max_distance: 0,};
#[no_mangle]
pub static mut resumeRefreshAt: timeUs_t = 0 as libc::c_int as timeUs_t;
static mut armState: uint8_t = 0;
static mut lastArmState: bool = false;
static mut osdDisplayPort: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
static mut escDataCombined: *mut escSensorData_t =
    0 as *const escSensorData_t as *mut escSensorData_t;
static mut compassBar: [libc::c_char; 24] =
    [0x1b as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x1c as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x18 as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x1c as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x1a as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x1c as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x19 as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x1c as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x1b as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x1c as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x18 as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x1c as libc::c_int as libc::c_char,
     0x1d as libc::c_int as libc::c_char];
static mut osdElementDisplayOrder: [uint8_t; 30] =
    [OSD_MAIN_BATT_VOLTAGE as libc::c_int as uint8_t,
     OSD_RSSI_VALUE as libc::c_int as uint8_t,
     OSD_CROSSHAIRS as libc::c_int as uint8_t,
     OSD_HORIZON_SIDEBARS as libc::c_int as uint8_t,
     OSD_ITEM_TIMER_1 as libc::c_int as uint8_t,
     OSD_ITEM_TIMER_2 as libc::c_int as uint8_t,
     OSD_REMAINING_TIME_ESTIMATE as libc::c_int as uint8_t,
     OSD_FLYMODE as libc::c_int as uint8_t,
     OSD_THROTTLE_POS as libc::c_int as uint8_t,
     OSD_VTX_CHANNEL as libc::c_int as uint8_t,
     OSD_CURRENT_DRAW as libc::c_int as uint8_t,
     OSD_MAH_DRAWN as libc::c_int as uint8_t,
     OSD_CRAFT_NAME as libc::c_int as uint8_t,
     OSD_ALTITUDE as libc::c_int as uint8_t,
     OSD_ROLL_PIDS as libc::c_int as uint8_t,
     OSD_PITCH_PIDS as libc::c_int as uint8_t,
     OSD_YAW_PIDS as libc::c_int as uint8_t,
     OSD_POWER as libc::c_int as uint8_t,
     OSD_PIDRATE_PROFILE as libc::c_int as uint8_t,
     OSD_WARNINGS as libc::c_int as uint8_t,
     OSD_AVG_CELL_VOLTAGE as libc::c_int as uint8_t,
     OSD_DEBUG as libc::c_int as uint8_t,
     OSD_PITCH_ANGLE as libc::c_int as uint8_t,
     OSD_ROLL_ANGLE as libc::c_int as uint8_t,
     OSD_MAIN_BATT_USAGE as libc::c_int as uint8_t,
     OSD_DISARMED as libc::c_int as uint8_t,
     OSD_NUMERICAL_HEADING as libc::c_int as uint8_t,
     OSD_NUMERICAL_VARIO as libc::c_int as uint8_t,
     OSD_COMPASS_BAR as libc::c_int as uint8_t,
     OSD_ANTI_GRAVITY as libc::c_int as uint8_t];
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
#[link_section = ".pg_registry"]
#[used]
pub static mut osdConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (501 as libc::c_int |
                                      (3 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
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
                                                                              Option<pgResetFunc>>(Some(pgResetFn_osdConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut osdConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
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
        0 => { return 0xf as libc::c_int as libc::c_char }
        _ => { return 0xc as libc::c_int as libc::c_char }
    };
}
/* *
 * Gets average battery cell voltage in 0.01V units.
 */
unsafe extern "C" fn osdGetBatteryAverageCellVoltage() -> libc::c_int {
    return getBatteryVoltage() as libc::c_int * 10 as libc::c_int /
               getBatteryCellCount() as libc::c_int;
}
unsafe extern "C" fn osdGetBatterySymbol(mut cellVoltage: libc::c_int)
 -> libc::c_char {
    if getBatteryState() as libc::c_uint ==
           BATTERY_CRITICAL as libc::c_int as libc::c_uint {
        return 0x97 as libc::c_int as libc::c_char
        // FIXME: currently the BAT- symbol, ideally replace with a battery with exclamation mark
    } else {
        // Calculate a symbol offset using cell voltage over full cell voltage range
        let symOffset: libc::c_int =
            scaleRange(cellVoltage,
                       (*batteryConfig()).vbatmincellvoltage as libc::c_int *
                           10 as libc::c_int,
                       (*batteryConfig()).vbatmaxcellvoltage as libc::c_int *
                           10 as libc::c_int, 0 as libc::c_int,
                       7 as libc::c_int);
        return (0x96 as libc::c_int -
                    constrain(symOffset, 0 as libc::c_int, 6 as libc::c_int))
                   as libc::c_char
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
            return meters * 328 as libc::c_int / 100 as libc::c_int
            // Already in metre / 100
        }
        _ => { return meters }
    }; // Ensure positive value
}
unsafe extern "C" fn osdConvertTemperatureToSelectedUnit(mut tempInDeciDegrees:
                                                             libc::c_int)
 -> libc::c_int {
    match (*osdConfig()).units as libc::c_uint {
        0 => {
            return tempInDeciDegrees * 9 as libc::c_int / 5 as libc::c_int +
                       320 as libc::c_int
        }
        _ => { return tempInDeciDegrees }
    };
}
unsafe extern "C" fn osdGetTemperatureSymbolForSelectedUnit()
 -> libc::c_char {
    match (*osdConfig()).units as libc::c_uint {
        0 => { return 'F' as i32 as libc::c_char }
        _ => { return 'C' as i32 as libc::c_char }
    };
}
unsafe extern "C" fn osdFormatAltitudeString(mut buff: *mut libc::c_char,
                                             mut altitude: libc::c_int) {
    let alt: libc::c_int =
        osdGetMetersToSelectedUnit(altitude) / 10 as libc::c_int;
    tfp_sprintf(buff, b"%5d %c\x00" as *const u8 as *const libc::c_char, alt,
                osdGetMetersToSelectedUnitSymbol() as libc::c_int);
    *buff.offset(5 as libc::c_int as isize) =
        *buff.offset(4 as libc::c_int as isize);
    *buff.offset(4 as libc::c_int as isize) = '.' as i32 as libc::c_char;
}
unsafe extern "C" fn osdFormatPID(mut buff: *mut libc::c_char,
                                  mut label: *const libc::c_char,
                                  mut pid: *const pidf_t) {
    tfp_sprintf(buff,
                b"%s %3d %3d %3d\x00" as *const u8 as *const libc::c_char,
                label, (*pid).P as libc::c_int, (*pid).I as libc::c_int,
                (*pid).D as libc::c_int);
}
unsafe extern "C" fn osdGetHeadingIntoDiscreteDirections(mut heading:
                                                             libc::c_int,
                                                         mut directions:
                                                             libc::c_uint)
 -> uint8_t {
    heading += 360 as libc::c_int;
    // Split input heading 0..359 into sectors 0..(directions-1), but offset
    // by half a sector so that sector 0 gets centered around heading 0.
    // We multiply heading by directions to not loose precision in divisions
    // In this way each segment will be a FULL_CIRCLE length
    let mut direction: libc::c_int =
        (heading as
             libc::c_uint).wrapping_mul(directions).wrapping_add((360 as
                                                                      libc::c_int
                                                                      /
                                                                      2 as
                                                                          libc::c_int)
                                                                     as
                                                                     libc::c_uint).wrapping_div(360
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
            as libc::c_int; // scale with rounding
    direction =
        (direction as libc::c_uint).wrapping_rem(directions) as libc::c_int as
            libc::c_int; // normalize
    return direction as uint8_t;
    // return segment number
}
unsafe extern "C" fn osdGetDirectionSymbolFromHeading(mut heading:
                                                          libc::c_int)
 -> uint8_t {
    heading =
        osdGetHeadingIntoDiscreteDirections(heading,
                                            16 as libc::c_int as libc::c_uint)
            as libc::c_int;
    // Now heading has a heading with Up=0, Right=4, Down=8 and Left=12
    // Our symbols are Down=0, Right=4, Up=8 and Left=12
    // There're 16 arrow symbols. Transform it.
    heading = 16 as libc::c_int - heading;
    heading = (heading + 8 as libc::c_int) % 16 as libc::c_int;
    return (0x60 as libc::c_int + heading) as uint8_t;
}
unsafe extern "C" fn osdGetTimerSymbol(mut src: osd_timer_source_e)
 -> libc::c_char {
    match src as libc::c_uint {
        0 => { return 0x9b as libc::c_int as libc::c_char }
        1 | 2 => { return 0x9c as libc::c_int as libc::c_char }
        _ => { return ' ' as i32 as libc::c_char }
    };
}
unsafe extern "C" fn osdGetTimerValue(mut src: osd_timer_source_e)
 -> timeUs_t {
    match src as libc::c_uint {
        0 => { return micros() }
        1 => { return flyTime }
        2 => { return stats.armed_time }
        _ => { return 0 as libc::c_int as timeUs_t }
    };
}
unsafe extern "C" fn osdFormatTime(mut buff: *mut libc::c_char,
                                   mut precision: osd_timer_precision_e,
                                   mut time: timeUs_t) {
    let mut seconds: libc::c_int =
        time.wrapping_div(1000000 as libc::c_int as libc::c_uint) as
            libc::c_int;
    let minutes: libc::c_int = seconds / 60 as libc::c_int;
    seconds = seconds % 60 as libc::c_int;
    match precision as libc::c_uint {
        1 => {
            let hundredths: libc::c_int =
                time.wrapping_div(10000 as libc::c_int as
                                      libc::c_uint).wrapping_rem(100 as
                                                                     libc::c_int
                                                                     as
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
    let src: uint8_t = (timer as libc::c_int & 0xf as libc::c_int) as uint8_t;
    if showSymbol {
        let fresh0 = buff;
        buff = buff.offset(1);
        *fresh0 = osdGetTimerSymbol(src as osd_timer_source_e)
    }
    osdFormatTime(buff,
                  if usePrecision as libc::c_int != 0 {
                      (timer as libc::c_int >> 4 as libc::c_int) &
                          0xf as libc::c_int
                  } else { OSD_TIMER_PREC_SECOND as libc::c_int } as
                      osd_timer_precision_e,
                  osdGetTimerValue(src as osd_timer_source_e));
}
// USE_GPS
unsafe extern "C" fn osdFormatMessage(mut buff: *mut libc::c_char,
                                      mut size: size_t,
                                      mut message: *const libc::c_char) {
    memset(buff as *mut libc::c_void, 0x20 as libc::c_int, size);
    if !message.is_null() {
        memcpy(buff as *mut libc::c_void, message as *const libc::c_void,
               strlen(message));
    }
    // Ensure buff is zero terminated
    *buff.offset(size.wrapping_sub(1 as libc::c_int as libc::c_ulong) as
                     isize) = '\u{0}' as i32 as libc::c_char; // change range
}
#[no_mangle]
pub unsafe extern "C" fn osdStatSetState(mut statIndex: uint8_t,
                                         mut enabled: bool) {
    if enabled {
        let ref mut fresh1 = (*osdConfigMutable()).enabled_stats;
        *fresh1 |=
            ((1 as libc::c_int) << statIndex as libc::c_int) as libc::c_uint
    } else {
        let ref mut fresh2 = (*osdConfigMutable()).enabled_stats;
        *fresh2 &=
            !((1 as libc::c_int) << statIndex as libc::c_int) as libc::c_uint
    };
}
#[no_mangle]
pub unsafe extern "C" fn osdStatGetState(mut statIndex: uint8_t) -> bool {
    return (*osdConfig()).enabled_stats &
               ((1 as libc::c_int) << statIndex as libc::c_int) as
                   libc::c_uint != 0;
}
#[no_mangle]
pub unsafe extern "C" fn osdWarnSetState(mut warningIndex: uint8_t,
                                         mut enabled: bool) {
    if enabled {
        let ref mut fresh3 = (*osdConfigMutable()).enabledWarnings;
        *fresh3 =
            (*fresh3 as libc::c_int |
                 (1 as libc::c_int) << warningIndex as libc::c_int) as
                uint16_t
    } else {
        let ref mut fresh4 = (*osdConfigMutable()).enabledWarnings;
        *fresh4 =
            (*fresh4 as libc::c_int &
                 !((1 as libc::c_int) << warningIndex as libc::c_int)) as
                uint16_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn osdWarnGetState(mut warningIndex: uint8_t) -> bool {
    return (*osdConfig()).enabledWarnings as libc::c_int &
               (1 as libc::c_int) << warningIndex as libc::c_int != 0;
}
unsafe extern "C" fn osdDrawSingleElement(mut item: uint8_t) -> bool {
    if (*osdConfig()).item_pos[item as usize] as libc::c_int &
           0x800 as libc::c_int == 0 ||
           blinkBits[(item as libc::c_int / 32 as libc::c_int) as usize] &
               ((1 as libc::c_int) << item as libc::c_int % 32 as libc::c_int)
                   as libc::c_uint != 0 && blinkState as libc::c_int != 0 {
        return 0 as libc::c_int != 0
    }
    let mut elemPosX: uint8_t =
        ((*osdConfig()).item_pos[item as usize] as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int) as
            uint8_t;
    let mut elemPosY: uint8_t =
        ((*osdConfig()).item_pos[item as usize] as libc::c_int >>
             5 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int) as
            uint8_t;
    let mut buff: [libc::c_char; 32] =
        *::core::mem::transmute::<&[u8; 32],
                                  &mut [libc::c_char; 32]>(b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00");
    let mut current_block_159: u64;
    match item as libc::c_int {
        0 => {
            let mut osdRssi: uint16_t =
                (getRssi() as libc::c_int * 100 as libc::c_int /
                     1024 as libc::c_int) as uint16_t;
            if osdRssi as libc::c_int >= 100 as libc::c_int {
                osdRssi = 99 as libc::c_int as uint16_t
            }
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%c%2d\x00" as *const u8 as *const libc::c_char,
                        0x1 as libc::c_int, osdRssi as libc::c_int);
        }
        1 => {
            buff[0 as libc::c_int as usize] =
                osdGetBatterySymbol(osdGetBatteryAverageCellVoltage());
            tfp_sprintf(buff.as_mut_ptr().offset(1 as libc::c_int as isize),
                        b"%2d.%1d%c\x00" as *const u8 as *const libc::c_char,
                        getBatteryVoltage() as libc::c_int /
                            10 as libc::c_int,
                        getBatteryVoltage() as libc::c_int %
                            10 as libc::c_int, 0x6 as libc::c_int);
        }
        11 => {
            let amperage: int32_t = getAmperage();
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%3d.%02d%c\x00" as *const u8 as *const libc::c_char,
                        abs(amperage) / 100 as libc::c_int,
                        abs(amperage) % 100 as libc::c_int,
                        0x9a as libc::c_int);
        }
        12 => {
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%4d%c\x00" as *const u8 as *const libc::c_char,
                        getMAhDrawn(), 0x7 as libc::c_int);
        }
        34 => {
            // GPS
            memcpy(buff.as_mut_ptr() as *mut libc::c_void,
                   compassBar.as_ptr().offset(osdGetHeadingIntoDiscreteDirections(attitude.values.yaw
                                                                                      as
                                                                                      libc::c_int
                                                                                      /
                                                                                      10
                                                                                          as
                                                                                          libc::c_int,
                                                                                  16
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                                  as libc::c_int as isize) as
                       *const libc::c_void,
                   9 as libc::c_int as libc::c_ulong);
            buff[9 as libc::c_int as usize] = 0 as libc::c_int as libc::c_char
        }
        15 => {
            osdFormatAltitudeString(buff.as_mut_ptr(),
                                    getEstimatedAltitude());
        }
        5 | 6 => {
            osdFormatTimer(buff.as_mut_ptr(), 1 as libc::c_int != 0,
                           1 as libc::c_int != 0,
                           item as libc::c_int -
                               OSD_ITEM_TIMER_1 as libc::c_int);
        }
        37 => {
            let mAhDrawn: libc::c_int = getMAhDrawn();
            let remaining_time: libc::c_int =
                (((*osdConfig()).cap_alarm as libc::c_int - mAhDrawn) as
                     libc::c_float * flyTime as libc::c_float /
                     mAhDrawn as libc::c_float) as libc::c_int;
            if (mAhDrawn as libc::c_double) <
                   0.1f64 *
                       (*osdConfig()).cap_alarm as libc::c_int as
                           libc::c_double {
                tfp_sprintf(buff.as_mut_ptr(),
                            b"--:--\x00" as *const u8 as *const libc::c_char);
            } else if mAhDrawn > (*osdConfig()).cap_alarm as libc::c_int {
                tfp_sprintf(buff.as_mut_ptr(),
                            b"00:00\x00" as *const u8 as *const libc::c_char);
            } else {
                osdFormatTime(buff.as_mut_ptr(), OSD_TIMER_PREC_SECOND,
                              remaining_time as timeUs_t);
            }
        }
        7 => {
            if flightModeFlags as libc::c_int & FAILSAFE_MODE as libc::c_int
                   != 0 {
                strcpy(buff.as_mut_ptr(),
                       b"!FS!\x00" as *const u8 as *const libc::c_char);
            } else if flightModeFlags as libc::c_int &
                          ANGLE_MODE as libc::c_int != 0 {
                strcpy(buff.as_mut_ptr(),
                       b"STAB\x00" as *const u8 as *const libc::c_char);
            } else if flightModeFlags as libc::c_int &
                          HORIZON_MODE as libc::c_int != 0 {
                strcpy(buff.as_mut_ptr(),
                       b"HOR \x00" as *const u8 as *const libc::c_char);
            } else if flightModeFlags as libc::c_int &
                          GPS_RESCUE_MODE as libc::c_int != 0 {
                strcpy(buff.as_mut_ptr(),
                       b"RESC\x00" as *const u8 as *const libc::c_char);
            } else if IS_RC_MODE_ACTIVE(BOXACROTRAINER) {
                strcpy(buff.as_mut_ptr(),
                       b"ATRN\x00" as *const u8 as *const libc::c_char);
            } else if isAirmodeActive() {
                strcpy(buff.as_mut_ptr(),
                       b"AIR \x00" as *const u8 as *const libc::c_char);
            } else {
                strcpy(buff.as_mut_ptr(),
                       b"ACRO\x00" as *const u8 as *const libc::c_char);
            }
        }
        41 => {
            if pidOsdAntiGravityActive() {
                strcpy(buff.as_mut_ptr(),
                       b"AG\x00" as *const u8 as *const libc::c_char);
            }
        }
        8 => {
            // This does not strictly support iterative updating if the craft name changes at run time. But since the craft name is not supposed to be changing this should not matter, and blanking the entire length of the craft name string on update will make it impossible to configure elements to be displayed on the right hand side of the craft name.
        //TODO: When iterative updating is implemented, change this so the craft name is only printed once whenever the OSD 'flight' screen is entered.
            if strlen((*pilotConfig()).name.as_ptr()) ==
                   0 as libc::c_int as libc::c_ulong {
                strcpy(buff.as_mut_ptr(),
                       b"CRAFT_NAME\x00" as *const u8 as *const libc::c_char);
            } else {
                let mut i: libc::c_uint = 0;
                i = 0 as libc::c_int as libc::c_uint;
                while i < 16 as libc::c_uint {
                    if !((*pilotConfig()).name[i as usize] != 0) { break ; }
                    buff[i as usize] =
                        toupper((*pilotConfig()).name[i as usize] as
                                    libc::c_uchar as libc::c_int) as
                            libc::c_char;
                    i = i.wrapping_add(1)
                }
                buff[i as usize] = '\u{0}' as i32 as libc::c_char
            }
        }
        9 => {
            buff[0 as libc::c_int as usize] =
                0x4 as libc::c_int as libc::c_char;
            buff[1 as libc::c_int as usize] =
                0x5 as libc::c_int as libc::c_char;
            tfp_sprintf(buff.as_mut_ptr().offset(2 as libc::c_int as isize),
                        b"%3d\x00" as *const u8 as *const libc::c_char,
                        (constrain(rcData[THROTTLE as libc::c_int as usize] as
                                       libc::c_int, 1000 as libc::c_int,
                                   2000 as libc::c_int) - 1000 as libc::c_int)
                            * 100 as libc::c_int /
                            (2000 as libc::c_int - 1000 as libc::c_int));
        }
        10 => {
            let vtxBandLetter: libc::c_char =
                *vtx58BandLetter.as_ptr().offset((*vtxSettingsConfig()).band
                                                     as isize);
            let mut vtxChannelName: *const libc::c_char =
                *vtx58ChannelNames.as_ptr().offset((*vtxSettingsConfig()).channel
                                                       as isize);
            let mut vtxPower: uint8_t = (*vtxSettingsConfig()).power;
            let mut vtxDevice: *const vtxDevice_t = vtxCommonDevice();
            if !vtxDevice.is_null() &&
                   (*vtxSettingsConfig()).lowPowerDisarm as libc::c_int != 0 {
                vtxCommonGetPowerIndex(vtxDevice, &mut vtxPower);
            }
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%c:%s:%1d\x00" as *const u8 as *const libc::c_char,
                        vtxBandLetter as libc::c_int, vtxChannelName,
                        vtxPower as libc::c_int);
        }
        2 => {
            buff[0 as libc::c_int as usize] =
                0x26 as libc::c_int as libc::c_char;
            buff[1 as libc::c_int as usize] =
                0x7e as libc::c_int as libc::c_char;
            buff[2 as libc::c_int as usize] =
                0x27 as libc::c_int as libc::c_char;
            buff[3 as libc::c_int as usize] = 0 as libc::c_int as libc::c_char
        }
        3 => {
            // Get pitch and roll limits in tenths of degrees
            let maxPitch: libc::c_int =
                (*osdConfig()).ahMaxPitch as libc::c_int * 10 as libc::c_int;
            let maxRoll: libc::c_int =
                (*osdConfig()).ahMaxRoll as libc::c_int * 10 as libc::c_int;
            let rollAngle: libc::c_int =
                constrain(attitude.values.roll as libc::c_int, -maxRoll,
                          maxRoll);
            let mut pitchAngle: libc::c_int =
                constrain(attitude.values.pitch as libc::c_int, -maxPitch,
                          maxPitch);
            // Convert pitchAngle to y compensation value
            // (maxPitch / 25) divisor matches previous settings of fixed divisor of 8 and fixed max AHI pitch angle of 20.0 degrees
            pitchAngle =
                pitchAngle * 25 as libc::c_int / maxPitch -
                    41 as libc::c_int; // 41 = 4 * AH_SYMBOL_COUNT + 5
            let mut x: libc::c_int = -(4 as libc::c_int);
            while x <= 4 as libc::c_int {
                let y: libc::c_int =
                    -rollAngle * x / 64 as libc::c_int - pitchAngle;
                if y >= 0 as libc::c_int && y <= 81 as libc::c_int {
                    displayWriteChar(osdDisplayPort,
                                     (elemPosX as libc::c_int + x) as uint8_t,
                                     (elemPosY as libc::c_int +
                                          y / 9 as libc::c_int) as uint8_t,
                                     (0x80 as libc::c_int +
                                          y % 9 as libc::c_int) as uint8_t);
                }
                x += 1
            }
            return 1 as libc::c_int != 0
        }
        4 => {
            // Draw AH sides
            let hudwidth: int8_t = 7 as libc::c_int as int8_t;
            let hudheight: int8_t = 3 as libc::c_int as int8_t;
            let mut y_0: libc::c_int = -(hudheight as libc::c_int);
            while y_0 <= hudheight as libc::c_int {
                displayWriteChar(osdDisplayPort,
                                 (elemPosX as libc::c_int -
                                      hudwidth as libc::c_int) as uint8_t,
                                 (elemPosY as libc::c_int + y_0) as uint8_t,
                                 0x13 as libc::c_int as uint8_t);
                displayWriteChar(osdDisplayPort,
                                 (elemPosX as libc::c_int +
                                      hudwidth as libc::c_int) as uint8_t,
                                 (elemPosY as libc::c_int + y_0) as uint8_t,
                                 0x13 as libc::c_int as uint8_t);
                y_0 += 1
            }
            // AH level indicators
            displayWriteChar(osdDisplayPort,
                             (elemPosX as libc::c_int -
                                  hudwidth as libc::c_int + 1 as libc::c_int)
                                 as uint8_t, elemPosY,
                             0x3 as libc::c_int as uint8_t);
            displayWriteChar(osdDisplayPort,
                             (elemPosX as libc::c_int +
                                  hudwidth as libc::c_int - 1 as libc::c_int)
                                 as uint8_t, elemPosY,
                             0x2 as libc::c_int as uint8_t);
            return 1 as libc::c_int != 0
        }
        42 => {
            let mut osdGForce: libc::c_float =
                0 as libc::c_int as libc::c_float;
            let mut axis: libc::c_int = 0 as libc::c_int;
            while axis < 3 as libc::c_int {
                let a: libc::c_float = accAverage[axis as usize];
                osdGForce += a * a;
                axis += 1
            }
            osdGForce =
                sqrtf(osdGForce) /
                    acc.dev.acc_1G as libc::c_int as libc::c_float;
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%01d.%01dG\x00" as *const u8 as *const libc::c_char,
                        osdGForce as libc::c_int,
                        (osdGForce * 10 as libc::c_int as libc::c_float) as
                            libc::c_int % 10 as libc::c_int);
        }
        16 => {
            osdFormatPID(buff.as_mut_ptr(),
                         b"ROL\x00" as *const u8 as *const libc::c_char,
                         &mut *(*currentPidProfile).pid.as_mut_ptr().offset(PID_ROLL
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize));
        }
        17 => {
            osdFormatPID(buff.as_mut_ptr(),
                         b"PIT\x00" as *const u8 as *const libc::c_char,
                         &mut *(*currentPidProfile).pid.as_mut_ptr().offset(PID_PITCH
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize));
        }
        18 => {
            osdFormatPID(buff.as_mut_ptr(),
                         b"YAW\x00" as *const u8 as *const libc::c_char,
                         &mut *(*currentPidProfile).pid.as_mut_ptr().offset(PID_YAW
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize));
        }
        19 => {
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%4dW\x00" as *const u8 as *const libc::c_char,
                        getAmperage() * getBatteryVoltage() as libc::c_int /
                            1000 as libc::c_int);
        }
        20 => {
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%d-%d\x00" as *const u8 as *const libc::c_char,
                        getCurrentPidProfileIndex() as libc::c_int +
                            1 as libc::c_int,
                        getCurrentControlRateProfileIndex() as libc::c_int +
                            1 as libc::c_int);
        }
        21 => {
            let batteryState: batteryState_e = getBatteryState();
            if isTryingToArm() as libc::c_int != 0 &&
                   armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                let mut armingDelayTime: libc::c_int =
                    (getLastDshotBeaconCommandTimeUs().wrapping_add(1200000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_sub(micros())
                         as libc::c_double / 1e5f64) as libc::c_int;
                if armingDelayTime < 0 as libc::c_int {
                    armingDelayTime = 0 as libc::c_int
                }
                if armingDelayTime as libc::c_double >=
                       1200000 as libc::c_int as libc::c_double / 1e5f64 -
                           5 as libc::c_int as libc::c_double {
                    osdFormatMessage(buff.as_mut_ptr(),
                                     (11 as libc::c_int + 1 as libc::c_int) as
                                         size_t,
                                     b" BEACON ON\x00" as *const u8 as
                                         *const libc::c_char);
                    // Display this message for the first 0.5 seconds
                } else {
                    let mut armingDelayMessage: [libc::c_char; 12] = [0; 12];
                    tfp_sprintf(armingDelayMessage.as_mut_ptr(),
                                b"ARM IN %d.%d\x00" as *const u8 as
                                    *const libc::c_char,
                                armingDelayTime / 10 as libc::c_int,
                                armingDelayTime % 10 as libc::c_int);
                    osdFormatMessage(buff.as_mut_ptr(),
                                     (11 as libc::c_int + 1 as libc::c_int) as
                                         size_t,
                                     armingDelayMessage.as_mut_ptr());
                }
            } else if osdWarnGetState(OSD_WARNING_BATTERY_CRITICAL as
                                          libc::c_int as uint8_t) as
                          libc::c_int != 0 &&
                          batteryState as libc::c_uint ==
                              BATTERY_CRITICAL as libc::c_int as libc::c_uint
             {
                osdFormatMessage(buff.as_mut_ptr(),
                                 (11 as libc::c_int + 1 as libc::c_int) as
                                     size_t,
                                 b" LAND NOW\x00" as *const u8 as
                                     *const libc::c_char);
            } else {
                // Show warning if we lose motor output, the ESC is overheating or excessive current draw
                if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) as
                       libc::c_int != 0 &&
                       osdWarnGetState(OSD_WARNING_ESC_FAIL as libc::c_int as
                                           uint8_t) as libc::c_int != 0 {
                    let mut escWarningMsg: [libc::c_char; 12] = [0; 12];
                    let mut pos: libc::c_uint =
                        0 as libc::c_int as libc::c_uint;
                    let mut title: *const libc::c_char =
                        b"ESC\x00" as *const u8 as *const libc::c_char;
                    // center justify message
                    while (pos as libc::c_ulong) <
                              (11 as libc::c_int as
                                   libc::c_ulong).wrapping_sub(strlen(title).wrapping_add(getMotorCount()
                                                                                              as
                                                                                              libc::c_ulong)).wrapping_div(2
                                                                                                                               as
                                                                                                                               libc::c_int
                                                                                                                               as
                                                                                                                               libc::c_ulong)
                          {
                        let fresh5 = pos;
                        pos = pos.wrapping_add(1);
                        escWarningMsg[fresh5 as usize] =
                            ' ' as i32 as libc::c_char
                    }
                    strcpy(escWarningMsg.as_mut_ptr().offset(pos as isize),
                           title);
                    pos =
                        (pos as libc::c_ulong).wrapping_add(strlen(title)) as
                            libc::c_uint as libc::c_uint;
                    let mut i_0: libc::c_uint =
                        0 as libc::c_int as libc::c_uint;
                    let mut escWarningCount: libc::c_uint =
                        0 as libc::c_int as libc::c_uint;
                    while i_0 < getMotorCount() as libc::c_uint &&
                              pos <
                                  (11 as libc::c_int + 1 as libc::c_int -
                                       1 as libc::c_int) as libc::c_uint {
                        let mut escData: *mut escSensorData_t =
                            getEscSensorData(i_0 as uint8_t);
                        let motorNumber: libc::c_char =
                            ('1' as i32 as libc::c_uint).wrapping_add(i_0) as
                                libc::c_char;
                        // if everything is OK just display motor number else R, T or C
                        let mut warnFlag: libc::c_char = motorNumber;
                        if armingFlags as libc::c_int & ARMED as libc::c_int
                               != 0 &&
                               (*osdConfig()).esc_rpm_alarm as libc::c_int !=
                                   -(1 as libc::c_int) &&
                               calcEscRpm((*escData).rpm as libc::c_int) <=
                                   (*osdConfig()).esc_rpm_alarm as libc::c_int
                           {
                            warnFlag = 'R' as i32 as libc::c_char
                        }
                        if (*osdConfig()).esc_temp_alarm as libc::c_int !=
                               -(128 as libc::c_int) &&
                               (*escData).temperature as libc::c_int >=
                                   (*osdConfig()).esc_temp_alarm as
                                       libc::c_int {
                            warnFlag = 'T' as i32 as libc::c_char
                        }
                        if armingFlags as libc::c_int & ARMED as libc::c_int
                               != 0 &&
                               (*osdConfig()).esc_current_alarm as libc::c_int
                                   != -(1 as libc::c_int) &&
                               (*escData).current >=
                                   (*osdConfig()).esc_current_alarm as
                                       libc::c_int {
                            warnFlag = 'C' as i32 as libc::c_char
                        }
                        let fresh6 = pos;
                        pos = pos.wrapping_add(1);
                        escWarningMsg[fresh6 as usize] = warnFlag;
                        if warnFlag as libc::c_int !=
                               motorNumber as libc::c_int {
                            escWarningCount = escWarningCount.wrapping_add(1)
                        }
                        i_0 = i_0.wrapping_add(1)
                    }
                    escWarningMsg[pos as usize] =
                        '\u{0}' as i32 as libc::c_char;
                    if escWarningCount > 0 as libc::c_int as libc::c_uint {
                        osdFormatMessage(buff.as_mut_ptr(),
                                         (11 as libc::c_int +
                                              1 as libc::c_int) as size_t,
                                         escWarningMsg.as_mut_ptr());
                        current_block_159 = 14285224772872304909;
                    } else { current_block_159 = 1606838685604807701; }
                } else { current_block_159 = 1606838685604807701; }
                match current_block_159 {
                    14285224772872304909 => { }
                    _ =>
                    // Warn when in flip over after crash mode
                    {
                        if osdWarnGetState(OSD_WARNING_CRASH_FLIP as
                                               libc::c_int as uint8_t) as
                               libc::c_int != 0 &&
                               isFlipOverAfterCrashMode() as libc::c_int != 0
                           {
                            osdFormatMessage(buff.as_mut_ptr(),
                                             (11 as libc::c_int +
                                                  1 as libc::c_int) as size_t,
                                             b"CRASH FLIP\x00" as *const u8 as
                                                 *const libc::c_char);
                        } else if osdWarnGetState(OSD_WARNING_ARMING_DISABLE
                                                      as libc::c_int as
                                                      uint8_t) as libc::c_int
                                      != 0 &&
                                      IS_RC_MODE_ACTIVE(BOXARM) as libc::c_int
                                          != 0 &&
                                      isArmingDisabled() as libc::c_int != 0 {
                            let flags: armingDisableFlags_e =
                                getArmingDisableFlags();
                            let mut i_1: libc::c_int = 0 as libc::c_int;
                            while i_1 < 20 as libc::c_int {
                                if flags as libc::c_uint &
                                       ((1 as libc::c_int) << i_1) as
                                           libc::c_uint != 0 {
                                    osdFormatMessage(buff.as_mut_ptr(),
                                                     (11 as libc::c_int +
                                                          1 as libc::c_int) as
                                                         size_t,
                                                     armingDisableFlagNames[i_1
                                                                                as
                                                                                usize]);
                                    break ;
                                } else { i_1 += 1 }
                            }
                        } else if osdWarnGetState(OSD_WARNING_BATTERY_WARNING
                                                      as libc::c_int as
                                                      uint8_t) as libc::c_int
                                      != 0 &&
                                      batteryState as libc::c_uint ==
                                          BATTERY_WARNING as libc::c_int as
                                              libc::c_uint {
                            osdFormatMessage(buff.as_mut_ptr(),
                                             (11 as libc::c_int +
                                                  1 as libc::c_int) as size_t,
                                             b"LOW BATTERY\x00" as *const u8
                                                 as *const libc::c_char);
                        } else if osdWarnGetState(OSD_WARNING_BATTERY_NOT_FULL
                                                      as libc::c_int as
                                                      uint8_t) as libc::c_int
                                      != 0 &&
                                      armingFlags as libc::c_int &
                                          WAS_EVER_ARMED as libc::c_int == 0
                                      &&
                                      getBatteryState() as libc::c_uint ==
                                          BATTERY_OK as libc::c_int as
                                              libc::c_uint &&
                                      (getBatteryAverageCellVoltage() as
                                           libc::c_int) <
                                          (*batteryConfig()).vbatfullcellvoltage
                                              as libc::c_int {
                            osdFormatMessage(buff.as_mut_ptr(),
                                             (11 as libc::c_int +
                                                  1 as libc::c_int) as size_t,
                                             b"BATT < FULL\x00" as *const u8
                                                 as *const libc::c_char);
                        } else if osdWarnGetState(OSD_WARNING_VISUAL_BEEPER as
                                                      libc::c_int as uint8_t)
                                      as libc::c_int != 0 &&
                                      showVisualBeeper as libc::c_int != 0 {
                            osdFormatMessage(buff.as_mut_ptr(),
                                             (11 as libc::c_int +
                                                  1 as libc::c_int) as size_t,
                                             b"  * * * *\x00" as *const u8 as
                                                 *const libc::c_char);
                        } else {
                            osdFormatMessage(buff.as_mut_ptr(),
                                             (11 as libc::c_int +
                                                  1 as libc::c_int) as size_t,
                                             0 as *const libc::c_char);
                        }
                    }
                }
            }
        }
        22 => {
            let cellV: libc::c_int = osdGetBatteryAverageCellVoltage();
            buff[0 as libc::c_int as usize] = osdGetBatterySymbol(cellV);
            tfp_sprintf(buff.as_mut_ptr().offset(1 as libc::c_int as isize),
                        b"%d.%02d%c\x00" as *const u8 as *const libc::c_char,
                        cellV / 100 as libc::c_int,
                        cellV % 100 as libc::c_int, 0x6 as libc::c_int);
        }
        25 => {
            tfp_sprintf(buff.as_mut_ptr(),
                        b"DBG %5d %5d %5d %5d\x00" as *const u8 as
                            *const libc::c_char,
                        debug[0 as libc::c_int as usize] as libc::c_int,
                        debug[1 as libc::c_int as usize] as libc::c_int,
                        debug[2 as libc::c_int as usize] as libc::c_int,
                        debug[3 as libc::c_int as usize] as libc::c_int);
        }
        26 | 27 => {
            let angle: libc::c_int =
                if item as libc::c_int == OSD_PITCH_ANGLE as libc::c_int {
                    attitude.values.pitch as libc::c_int
                } else { attitude.values.roll as libc::c_int };
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%c%02d.%01d\x00" as *const u8 as
                            *const libc::c_char,
                        if angle < 0 as libc::c_int {
                            '-' as i32
                        } else { ' ' as i32 }, abs(angle / 10 as libc::c_int),
                        abs(angle % 10 as libc::c_int));
        }
        28 => {
            // Show most severe reason for arming being disabled
            // Show warning if battery is not fresh
            // Visual beeper
            // Set length of indicator bar
            // Use an odd number so the bar can be centered.
            // Calculate constrained value
            let value: libc::c_float =
                constrain((*batteryConfig()).batteryCapacity as libc::c_int -
                              getMAhDrawn(), 0 as libc::c_int,
                          (*batteryConfig()).batteryCapacity as libc::c_int)
                    as libc::c_float;
            // Calculate mAh used progress
            let mAhUsedProgress: uint8_t =
                ceilf(value /
                          ((*batteryConfig()).batteryCapacity as libc::c_int /
                               11 as libc::c_int) as libc::c_float) as
                    uint8_t;
            // Create empty battery indicator bar
            buff[0 as libc::c_int as usize] =
                0x8a as libc::c_int as libc::c_char;
            let mut i_2: libc::c_int = 1 as libc::c_int;
            while i_2 <= 11 as libc::c_int {
                buff[i_2 as usize] =
                    if i_2 <= mAhUsedProgress as libc::c_int {
                        0x8b as libc::c_int
                    } else { 0x8d as libc::c_int } as libc::c_char;
                i_2 += 1
            }
            buff[(11 as libc::c_int + 1 as libc::c_int) as usize] =
                0x8f as libc::c_int as libc::c_char;
            if mAhUsedProgress as libc::c_int > 0 as libc::c_int &&
                   (mAhUsedProgress as libc::c_int) < 11 as libc::c_int {
                buff[(1 as libc::c_int + mAhUsedProgress as libc::c_int) as
                         usize] = 0x8e as libc::c_int as libc::c_char
            }
            buff[(11 as libc::c_int + 2 as libc::c_int) as usize] =
                '\u{0}' as i32 as libc::c_char
        }
        29 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                tfp_sprintf(buff.as_mut_ptr(),
                            b"DISARMED\x00" as *const u8 as
                                *const libc::c_char);
            } else if !lastArmState {
                // previously disarmed - blank out the message one time
                tfp_sprintf(buff.as_mut_ptr(),
                            b"        \x00" as *const u8 as
                                *const libc::c_char);
            }
        }
        32 => {
            let heading: libc::c_int =
                attitude.values.yaw as libc::c_int / 10 as libc::c_int;
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%c%03d\x00" as *const u8 as *const libc::c_char,
                        osdGetDirectionSymbolFromHeading(heading) as
                            libc::c_int, heading);
        }
        33 => {
            let verticalSpeed: libc::c_int =
                osdGetMetersToSelectedUnit(getEstimatedVario() as int32_t);
            let directionSymbol: libc::c_char =
                if verticalSpeed < 0 as libc::c_int {
                    0x60 as libc::c_int
                } else { 0x68 as libc::c_int } as libc::c_char;
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%c%01d.%01d\x00" as *const u8 as
                            *const libc::c_char,
                        directionSymbol as libc::c_int,
                        abs(verticalSpeed / 100 as libc::c_int),
                        abs(verticalSpeed % 100 as libc::c_int /
                                10 as libc::c_int));
        }
        35 => {
            if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
                tfp_sprintf(buff.as_mut_ptr(),
                            b"%3d%c\x00" as *const u8 as *const libc::c_char,
                            osdConvertTemperatureToSelectedUnit((*escDataCombined).temperature
                                                                    as
                                                                    libc::c_int
                                                                    *
                                                                    10 as
                                                                        libc::c_int)
                                / 10 as libc::c_int,
                            osdGetTemperatureSymbolForSelectedUnit() as
                                libc::c_int);
            }
        }
        36 => {
            if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
                tfp_sprintf(buff.as_mut_ptr(),
                            b"%5d\x00" as *const u8 as *const libc::c_char,
                            if escDataCombined.is_null() {
                                0 as libc::c_int
                            } else {
                                calcEscRpm((*escDataCombined).rpm as
                                               libc::c_int)
                            });
            }
        }
        _ => { return 0 as libc::c_int != 0 }
    }
    displayWrite(osdDisplayPort, elemPosX, elemPosY, buff.as_mut_ptr());
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn osdDrawElements() {
    displayClearScreen(osdDisplayPort);
    // Hide OSD when OSDSW mode is active
    if IS_RC_MODE_ACTIVE(BOXOSD) { return }
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        osdDrawSingleElement(OSD_ARTIFICIAL_HORIZON as libc::c_int as
                                 uint8_t);
        osdDrawSingleElement(OSD_G_FORCE as libc::c_int as uint8_t);
    }
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (i as libc::c_ulong) <
              ::core::mem::size_of::<[uint8_t; 30]>() as libc::c_ulong {
        osdDrawSingleElement(osdElementDisplayOrder[i as usize]);
        i = i.wrapping_add(1)
    }
    // GPS
    if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
        osdDrawSingleElement(OSD_ESC_TMP as libc::c_int as uint8_t);
        osdDrawSingleElement(OSD_ESC_RPM as libc::c_int as uint8_t);
    };
}
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_osdConfig(mut osdConfig_0:
                                                 *mut osdConfig_t) {
    (*osdConfig_0).item_pos[OSD_RSSI_VALUE as libc::c_int as usize] =
        (8 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (1 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_MAIN_BATT_VOLTAGE as libc::c_int as usize] =
        (12 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (1 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_CROSSHAIRS as libc::c_int as usize] =
        (8 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (6 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ARTIFICIAL_HORIZON as libc::c_int as usize] =
        (8 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (6 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_HORIZON_SIDEBARS as libc::c_int as usize] =
        (8 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (6 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ITEM_TIMER_1 as libc::c_int as usize] =
        (22 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (1 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ITEM_TIMER_2 as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (1 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_FLYMODE as libc::c_int as usize] =
        (13 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (10 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_CRAFT_NAME as libc::c_int as usize] =
        (10 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (11 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_THROTTLE_POS as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (7 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_VTX_CHANNEL as libc::c_int as usize] =
        (24 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (11 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_CURRENT_DRAW as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (12 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_MAH_DRAWN as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (11 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_GPS_SPEED as libc::c_int as usize] =
        (25 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (6 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_GPS_SATS as libc::c_int as usize] =
        (19 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (1 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ALTITUDE as libc::c_int as usize] =
        (24 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (7 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ROLL_PIDS as libc::c_int as usize] =
        (7 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (13 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_PITCH_PIDS as libc::c_int as usize] =
        (7 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (14 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_YAW_PIDS as libc::c_int as usize] =
        (7 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (15 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_POWER as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (10 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_PIDRATE_PROFILE as libc::c_int as usize] =
        (25 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (10 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_AVG_CELL_VOLTAGE as libc::c_int as usize] =
        (12 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (0 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_DEBUG as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (0 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_PITCH_ANGLE as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (8 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ROLL_ANGLE as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (9 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_GPS_LAT as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (0 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_GPS_LON as libc::c_int as usize] =
        (18 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (0 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_HOME_DIST as libc::c_int as usize] =
        (15 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (9 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_HOME_DIR as libc::c_int as usize] =
        (14 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (9 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_COMPASS_BAR as libc::c_int as usize] =
        (10 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (8 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_MAIN_BATT_USAGE as libc::c_int as usize] =
        (8 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (12 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_DISARMED as libc::c_int as usize] =
        (11 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (4 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_NUMERICAL_HEADING as libc::c_int as usize] =
        (24 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (9 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_NUMERICAL_VARIO as libc::c_int as usize] =
        (24 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (8 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ESC_TMP as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (5 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ESC_RPM as libc::c_int as usize] =
        (1 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (6 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    // Always enable warnings elements by default
    (*osdConfig_0).item_pos[OSD_WARNINGS as libc::c_int as usize] =
        (9 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (10 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    // Default to old fixed positions for these elements
    (*osdConfig_0).item_pos[OSD_CROSSHAIRS as libc::c_int as usize] =
        (13 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (6 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_ARTIFICIAL_HORIZON as libc::c_int as usize] =
        (14 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (2 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int) as uint16_t;
    (*osdConfig_0).item_pos[OSD_HORIZON_SIDEBARS as libc::c_int as usize] =
        (14 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (6 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int) as uint16_t;
    // Enable the default stats
    (*osdConfig_0).enabled_stats =
        0 as libc::c_int as
            uint32_t; // reset all to off and enable only a few initially
    osdStatSetState(OSD_STAT_MAX_SPEED as libc::c_int as uint8_t,
                    1 as libc::c_int != 0);
    osdStatSetState(OSD_STAT_MIN_BATTERY as libc::c_int as uint8_t,
                    1 as libc::c_int != 0);
    osdStatSetState(OSD_STAT_MIN_RSSI as libc::c_int as uint8_t,
                    1 as libc::c_int != 0);
    osdStatSetState(OSD_STAT_MAX_CURRENT as libc::c_int as uint8_t,
                    1 as libc::c_int != 0);
    osdStatSetState(OSD_STAT_USED_MAH as libc::c_int as uint8_t,
                    1 as libc::c_int != 0);
    osdStatSetState(OSD_STAT_BLACKBOX as libc::c_int as uint8_t,
                    1 as libc::c_int != 0);
    osdStatSetState(OSD_STAT_BLACKBOX_NUMBER as libc::c_int as uint8_t,
                    1 as libc::c_int != 0);
    osdStatSetState(OSD_STAT_TIMER_2 as libc::c_int as uint8_t,
                    1 as libc::c_int != 0);
    (*osdConfig_0).units = OSD_UNIT_METRIC;
    // Enable all warnings by default
    let mut i: libc::c_int =
        0 as libc::c_int; // meters or feet depend on configuration
    while i < OSD_WARNING_COUNT as libc::c_int {
        osdWarnSetState(i as uint8_t,
                        1 as libc::c_int != 0); // off by default
        i += 1
    } // off by default
    (*osdConfig_0).timers[OSD_TIMER_1 as libc::c_int as usize] =
        (OSD_TIMER_SRC_ON as libc::c_int & 0xf as libc::c_int |
             (OSD_TIMER_PREC_SECOND as libc::c_int & 0xf as libc::c_int) <<
                 4 as libc::c_int |
             (10 as libc::c_int & 0xff as libc::c_int) << 8 as libc::c_int) as
            uint16_t; // off by default
    (*osdConfig_0).timers[OSD_TIMER_2 as libc::c_int as usize] =
        (OSD_TIMER_SRC_TOTAL_ARMED as libc::c_int & 0xf as libc::c_int |
             (OSD_TIMER_PREC_SECOND as libc::c_int & 0xf as libc::c_int) <<
                 4 as libc::c_int |
             (10 as libc::c_int & 0xff as libc::c_int) << 8 as libc::c_int) as
            uint16_t; // a temperature above 70C should produce a warning, lockups have been reported above 80C
    (*osdConfig_0).rssi_alarm = 20 as libc::c_int as uint8_t; // 20 degrees
    (*osdConfig_0).cap_alarm = 2200 as libc::c_int as uint16_t;
    (*osdConfig_0).alt_alarm = 100 as libc::c_int as uint16_t;
    (*osdConfig_0).esc_temp_alarm = -(128 as libc::c_int) as int8_t;
    (*osdConfig_0).esc_rpm_alarm = -(1 as libc::c_int) as int16_t;
    (*osdConfig_0).esc_current_alarm = -(1 as libc::c_int) as int16_t;
    (*osdConfig_0).core_temp_alarm = 70 as libc::c_int as uint8_t;
    (*osdConfig_0).ahMaxPitch = 20 as libc::c_int as uint8_t;
    (*osdConfig_0).ahMaxRoll = 40 as libc::c_int as uint8_t;
    // 40 degrees
}
unsafe extern "C" fn osdDrawLogo(mut x: libc::c_int, mut y: libc::c_int) {
    // display logo and help
    let mut fontOffset: libc::c_int = 160 as libc::c_int;
    let mut row: libc::c_int = 0 as libc::c_int;
    while row < 4 as libc::c_int {
        let mut column: libc::c_int = 0 as libc::c_int;
        while column < 24 as libc::c_int {
            if fontOffset <= 0xff as libc::c_int {
                let fresh7 = fontOffset;
                fontOffset = fontOffset + 1;
                displayWriteChar(osdDisplayPort, (x + column) as uint8_t,
                                 (y + row) as uint8_t, fresh7 as uint8_t);
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
    cmsDisplayPortRegister(osdDisplayPort);
    armState = (armingFlags as libc::c_int & ARMED as libc::c_int) as uint8_t;
    memset(blinkBits.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[uint32_t; 2]>() as libc::c_ulong);
    displayClearScreen(osdDisplayPort);
    osdDrawLogo(3 as libc::c_int, 1 as libc::c_int);
    let mut string_buffer: [libc::c_char; 30] = [0; 30];
    tfp_sprintf(string_buffer.as_mut_ptr(),
                b"V%s\x00" as *const u8 as *const libc::c_char,
                b"2.5.0\x00" as *const u8 as *const libc::c_char);
    displayWrite(osdDisplayPort, 20 as libc::c_int as uint8_t,
                 6 as libc::c_int as uint8_t, string_buffer.as_mut_ptr());
    displayWrite(osdDisplayPort, 7 as libc::c_int as uint8_t,
                 8 as libc::c_int as uint8_t,
                 b"MENU:THR MID\x00" as *const u8 as *const libc::c_char);
    displayWrite(osdDisplayPort, 11 as libc::c_int as uint8_t,
                 9 as libc::c_int as uint8_t,
                 b"+ YAW LEFT\x00" as *const u8 as *const libc::c_char);
    displayWrite(osdDisplayPort, 11 as libc::c_int as uint8_t,
                 10 as libc::c_int as uint8_t,
                 b"+ PITCH UP\x00" as *const u8 as *const libc::c_char);
    displayResync(osdDisplayPort);
    resumeRefreshAt =
        micros().wrapping_add((4 as libc::c_int * 1000 as libc::c_int *
                                   1000 as libc::c_int) as libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn osdInitialized() -> bool {
    return !osdDisplayPort.is_null();
}
#[no_mangle]
pub unsafe extern "C" fn osdUpdateAlarms() {
    // This is overdone?
    let mut alt: int32_t =
        osdGetMetersToSelectedUnit(getEstimatedAltitude()) /
            100 as libc::c_int;
    if (getRssiPercent() as libc::c_int) <
           (*osdConfig()).rssi_alarm as libc::c_int {
        blinkBits[(OSD_RSSI_VALUE as libc::c_int / 32 as libc::c_int) as
                      usize] |=
            ((1 as libc::c_int) <<
                 OSD_RSSI_VALUE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    } else {
        blinkBits[(OSD_RSSI_VALUE as libc::c_int / 32 as libc::c_int) as
                      usize] &=
            !((1 as libc::c_int) <<
                  OSD_RSSI_VALUE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    }
    // Determine if the OSD_WARNINGS should blink
    if getBatteryState() as libc::c_uint !=
           BATTERY_OK as libc::c_int as libc::c_uint &&
           (osdWarnGetState(OSD_WARNING_BATTERY_CRITICAL as libc::c_int as
                                uint8_t) as libc::c_int != 0 ||
                osdWarnGetState(OSD_WARNING_BATTERY_WARNING as libc::c_int as
                                    uint8_t) as libc::c_int != 0) &&
           !isTryingToArm() {
        blinkBits[(OSD_WARNINGS as libc::c_int / 32 as libc::c_int) as usize]
            |=
            ((1 as libc::c_int) <<
                 OSD_WARNINGS as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    } else {
        blinkBits[(OSD_WARNINGS as libc::c_int / 32 as libc::c_int) as usize]
            &=
            !((1 as libc::c_int) <<
                  OSD_WARNINGS as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    } // convert from minutes to us
    if getBatteryState() as libc::c_uint ==
           BATTERY_OK as libc::c_int as libc::c_uint {
        blinkBits[(OSD_MAIN_BATT_VOLTAGE as libc::c_int / 32 as libc::c_int)
                      as usize] &=
            !((1 as libc::c_int) <<
                  OSD_MAIN_BATT_VOLTAGE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint;
        blinkBits[(OSD_AVG_CELL_VOLTAGE as libc::c_int / 32 as libc::c_int) as
                      usize] &=
            !((1 as libc::c_int) <<
                  OSD_AVG_CELL_VOLTAGE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    } else {
        blinkBits[(OSD_MAIN_BATT_VOLTAGE as libc::c_int / 32 as libc::c_int)
                      as usize] |=
            ((1 as libc::c_int) <<
                 OSD_MAIN_BATT_VOLTAGE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint;
        blinkBits[(OSD_AVG_CELL_VOLTAGE as libc::c_int / 32 as libc::c_int) as
                      usize] |=
            ((1 as libc::c_int) <<
                 OSD_AVG_CELL_VOLTAGE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    }
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int == 0 as libc::c_int
       {
        blinkBits[(OSD_GPS_SATS as libc::c_int / 32 as libc::c_int) as usize]
            |=
            ((1 as libc::c_int) <<
                 OSD_GPS_SATS as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    } else {
        blinkBits[(OSD_GPS_SATS as libc::c_int / 32 as libc::c_int) as usize]
            &=
            !((1 as libc::c_int) <<
                  OSD_GPS_SATS as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    }
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < OSD_TIMER_COUNT as libc::c_int {
        let timer: uint16_t = (*osdConfig()).timers[i as usize];
        let time: timeUs_t =
            osdGetTimerValue((timer as libc::c_int & 0xf as libc::c_int) as
                                 osd_timer_source_e);
        let alarmTime: timeUs_t =
            ((timer as libc::c_int >> 8 as libc::c_int & 0xff as libc::c_int)
                 * 60000000 as libc::c_int) as timeUs_t;
        if alarmTime != 0 as libc::c_int as libc::c_uint && time >= alarmTime
           {
            blinkBits[((OSD_ITEM_TIMER_1 as libc::c_int + i) /
                           32 as libc::c_int) as usize] |=
                ((1 as libc::c_int) <<
                     (OSD_ITEM_TIMER_1 as libc::c_int + i) %
                         32 as libc::c_int) as libc::c_uint
        } else {
            blinkBits[((OSD_ITEM_TIMER_1 as libc::c_int + i) /
                           32 as libc::c_int) as usize] &=
                !((1 as libc::c_int) <<
                      (OSD_ITEM_TIMER_1 as libc::c_int + i) %
                          32 as libc::c_int) as libc::c_uint
        }
        i += 1
    }
    if getMAhDrawn() >= (*osdConfig()).cap_alarm as libc::c_int {
        blinkBits[(OSD_MAH_DRAWN as libc::c_int / 32 as libc::c_int) as usize]
            |=
            ((1 as libc::c_int) <<
                 OSD_MAH_DRAWN as libc::c_int % 32 as libc::c_int) as
                libc::c_uint;
        blinkBits[(OSD_MAIN_BATT_USAGE as libc::c_int / 32 as libc::c_int) as
                      usize] |=
            ((1 as libc::c_int) <<
                 OSD_MAIN_BATT_USAGE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint;
        blinkBits[(OSD_REMAINING_TIME_ESTIMATE as libc::c_int /
                       32 as libc::c_int) as usize] |=
            ((1 as libc::c_int) <<
                 OSD_REMAINING_TIME_ESTIMATE as libc::c_int %
                     32 as libc::c_int) as libc::c_uint
    } else {
        blinkBits[(OSD_MAH_DRAWN as libc::c_int / 32 as libc::c_int) as usize]
            &=
            !((1 as libc::c_int) <<
                  OSD_MAH_DRAWN as libc::c_int % 32 as libc::c_int) as
                libc::c_uint;
        blinkBits[(OSD_MAIN_BATT_USAGE as libc::c_int / 32 as libc::c_int) as
                      usize] &=
            !((1 as libc::c_int) <<
                  OSD_MAIN_BATT_USAGE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint;
        blinkBits[(OSD_REMAINING_TIME_ESTIMATE as libc::c_int /
                       32 as libc::c_int) as usize] &=
            !((1 as libc::c_int) <<
                  OSD_REMAINING_TIME_ESTIMATE as libc::c_int %
                      32 as libc::c_int) as libc::c_uint
    }
    if alt >= (*osdConfig()).alt_alarm as libc::c_int {
        blinkBits[(OSD_ALTITUDE as libc::c_int / 32 as libc::c_int) as usize]
            |=
            ((1 as libc::c_int) <<
                 OSD_ALTITUDE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    } else {
        blinkBits[(OSD_ALTITUDE as libc::c_int / 32 as libc::c_int) as usize]
            &=
            !((1 as libc::c_int) <<
                  OSD_ALTITUDE as libc::c_int % 32 as libc::c_int) as
                libc::c_uint
    }
    if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
        // This works because the combined ESC data contains the maximum temperature seen amongst all ESCs
        if (*osdConfig()).esc_temp_alarm as libc::c_int !=
               -(128 as libc::c_int) &&
               (*escDataCombined).temperature as libc::c_int >=
                   (*osdConfig()).esc_temp_alarm as libc::c_int {
            blinkBits[(OSD_ESC_TMP as libc::c_int / 32 as libc::c_int) as
                          usize] |=
                ((1 as libc::c_int) <<
                     OSD_ESC_TMP as libc::c_int % 32 as libc::c_int) as
                    libc::c_uint
        } else {
            blinkBits[(OSD_ESC_TMP as libc::c_int / 32 as libc::c_int) as
                          usize] &=
                !((1 as libc::c_int) <<
                      OSD_ESC_TMP as libc::c_int % 32 as libc::c_int) as
                    libc::c_uint
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn osdResetAlarms() {
    blinkBits[(OSD_RSSI_VALUE as libc::c_int / 32 as libc::c_int) as usize] &=
        !((1 as libc::c_int) <<
              OSD_RSSI_VALUE as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_MAIN_BATT_VOLTAGE as libc::c_int / 32 as libc::c_int) as
                  usize] &=
        !((1 as libc::c_int) <<
              OSD_MAIN_BATT_VOLTAGE as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_WARNINGS as libc::c_int / 32 as libc::c_int) as usize] &=
        !((1 as libc::c_int) <<
              OSD_WARNINGS as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_GPS_SATS as libc::c_int / 32 as libc::c_int) as usize] &=
        !((1 as libc::c_int) <<
              OSD_GPS_SATS as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_MAH_DRAWN as libc::c_int / 32 as libc::c_int) as usize] &=
        !((1 as libc::c_int) <<
              OSD_MAH_DRAWN as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_ALTITUDE as libc::c_int / 32 as libc::c_int) as usize] &=
        !((1 as libc::c_int) <<
              OSD_ALTITUDE as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_AVG_CELL_VOLTAGE as libc::c_int / 32 as libc::c_int) as
                  usize] &=
        !((1 as libc::c_int) <<
              OSD_AVG_CELL_VOLTAGE as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_MAIN_BATT_USAGE as libc::c_int / 32 as libc::c_int) as
                  usize] &=
        !((1 as libc::c_int) <<
              OSD_MAIN_BATT_USAGE as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_ITEM_TIMER_1 as libc::c_int / 32 as libc::c_int) as usize]
        &=
        !((1 as libc::c_int) <<
              OSD_ITEM_TIMER_1 as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_ITEM_TIMER_2 as libc::c_int / 32 as libc::c_int) as usize]
        &=
        !((1 as libc::c_int) <<
              OSD_ITEM_TIMER_2 as libc::c_int % 32 as libc::c_int) as
            libc::c_uint;
    blinkBits[(OSD_REMAINING_TIME_ESTIMATE as libc::c_int / 32 as libc::c_int)
                  as usize] &=
        !((1 as libc::c_int) <<
              OSD_REMAINING_TIME_ESTIMATE as libc::c_int % 32 as libc::c_int)
            as libc::c_uint;
    blinkBits[(OSD_ESC_TMP as libc::c_int / 32 as libc::c_int) as usize] &=
        !((1 as libc::c_int) <<
              OSD_ESC_TMP as libc::c_int % 32 as libc::c_int) as libc::c_uint;
}
unsafe extern "C" fn osdResetStats() {
    stats.max_current = 0 as libc::c_int as int16_t;
    stats.max_speed = 0 as libc::c_int as int16_t;
    stats.min_voltage = 500 as libc::c_int as int16_t;
    stats.min_rssi = 99 as libc::c_int as int16_t;
    stats.max_altitude = 0 as libc::c_int;
    stats.max_distance = 0 as libc::c_int as int16_t;
    stats.armed_time = 0 as libc::c_int as timeUs_t;
}
unsafe extern "C" fn osdUpdateStats() {
    let mut value: int16_t = 0 as libc::c_int as int16_t;
    if (stats.max_speed as libc::c_int) < value as libc::c_int {
        stats.max_speed = value
    }
    value = getBatteryVoltage() as int16_t;
    if stats.min_voltage as libc::c_int > value as libc::c_int {
        stats.min_voltage = value
    }
    value = (getAmperage() / 100 as libc::c_int) as int16_t;
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
    let mut storageDeviceIsWorking: bool = 0 as libc::c_int != 0;
    let mut storageUsed: uint32_t = 0 as libc::c_int as uint32_t;
    let mut storageTotal: uint32_t = 0 as libc::c_int as uint32_t;
    match (*blackboxConfig()).device as libc::c_int {
        2 => {
            storageDeviceIsWorking =
                sdcard_isInserted() as libc::c_int != 0 &&
                    sdcard_isFunctional() as libc::c_int != 0 &&
                    afatfs_getFilesystemState() as libc::c_uint ==
                        AFATFS_FILESYSTEM_STATE_READY as libc::c_int as
                            libc::c_uint;
            if storageDeviceIsWorking {
                storageTotal =
                    (*sdcard_getMetadata()).numBlocks.wrapping_div(2000 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint);
                storageUsed =
                    storageTotal.wrapping_sub(afatfs_getContiguousFreeSpace().wrapping_div(1024000
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               libc::c_uint))
            }
        }
        _ => { }
    }
    if storageDeviceIsWorking {
        let storageUsedPercent: uint16_t =
            storageUsed.wrapping_mul(100 as libc::c_int as
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
    displayWrite(osdDisplayPort, 2 as libc::c_int as uint8_t, y, text);
    displayWrite(osdDisplayPort, 20 as libc::c_int as uint8_t, y,
                 b":\x00" as *const u8 as *const libc::c_char);
    displayWrite(osdDisplayPort, 22 as libc::c_int as uint8_t, y, value);
}
/*
 * Test if there's some stat enabled
 */
unsafe extern "C" fn isSomeStatEnabled() -> bool {
    return (*osdConfig()).enabled_stats != 0 as libc::c_int as libc::c_uint;
}
// *** IMPORTANT ***
// The order of the OSD stats as displayed on-screen must match the osd_stats_e enumeration.
// This is because the fields are presented in the configurator in the order of the enumeration
// and we want the configuration order to match the on-screen display order.  If you change the
// display order you *must* update the osd_stats_e enumeration to match. Additionally the
// changes to the stats display order *must* be implemented in the configurator otherwise the
// stats selections will not be populated correctly and the settings will become corrupted.
unsafe extern "C" fn osdShowStats(mut endBatteryVoltage: uint16_t) {
    let mut top: uint8_t = 2 as libc::c_int as uint8_t;
    let mut buff: [libc::c_char; 32] = [0; 32];
    displayClearScreen(osdDisplayPort);
    let fresh8 = top;
    top = top.wrapping_add(1);
    displayWrite(osdDisplayPort, 2 as libc::c_int as uint8_t, fresh8,
                 b"  --- STATS ---\x00" as *const u8 as *const libc::c_char);
    if osdStatGetState(OSD_STAT_RTC_DATE_TIME as libc::c_int as uint8_t) {
        let mut success: bool = 0 as libc::c_int != 0;
        if !success {
            tfp_sprintf(buff.as_mut_ptr(),
                        b"NO RTC\x00" as *const u8 as *const libc::c_char);
        }
        let fresh9 = top;
        top = top.wrapping_add(1);
        displayWrite(osdDisplayPort, 2 as libc::c_int as uint8_t, fresh9,
                     buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_TIMER_1 as libc::c_int as uint8_t) {
        osdFormatTimer(buff.as_mut_ptr(), 0 as libc::c_int != 0,
                       if (*osdConfig()).timers[OSD_TIMER_1 as libc::c_int as
                                                    usize] as libc::c_int &
                              0xf as libc::c_int ==
                              OSD_TIMER_SRC_ON as libc::c_int {
                           0 as libc::c_int
                       } else { 1 as libc::c_int } != 0,
                       OSD_TIMER_1 as libc::c_int);
        let fresh10 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh10,
                                 osdTimerSourceNames[((*osdConfig()).timers[OSD_TIMER_1
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                usize]
                                                          as libc::c_int &
                                                          0xf as libc::c_int)
                                                         as usize],
                                 buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_TIMER_2 as libc::c_int as uint8_t) {
        osdFormatTimer(buff.as_mut_ptr(), 0 as libc::c_int != 0,
                       if (*osdConfig()).timers[OSD_TIMER_2 as libc::c_int as
                                                    usize] as libc::c_int &
                              0xf as libc::c_int ==
                              OSD_TIMER_SRC_ON as libc::c_int {
                           0 as libc::c_int
                       } else { 1 as libc::c_int } != 0,
                       OSD_TIMER_2 as libc::c_int);
        let fresh11 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh11,
                                 osdTimerSourceNames[((*osdConfig()).timers[OSD_TIMER_2
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                usize]
                                                          as libc::c_int &
                                                          0xf as libc::c_int)
                                                         as usize],
                                 buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_MAX_SPEED as libc::c_int as uint8_t) as
           libc::c_int != 0 &&
           stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
        itoa(stats.max_speed as libc::c_int, buff.as_mut_ptr(),
             10 as libc::c_int);
        let fresh12 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh12,
                                 b"MAX SPEED\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_MAX_DISTANCE as libc::c_int as uint8_t) {
        tfp_sprintf(buff.as_mut_ptr(),
                    b"%d%c\x00" as *const u8 as *const libc::c_char,
                    osdGetMetersToSelectedUnit(stats.max_distance as int32_t),
                    osdGetMetersToSelectedUnitSymbol() as libc::c_int);
        let fresh13 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh13,
                                 b"MAX DISTANCE\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_MIN_BATTERY as libc::c_int as uint8_t) {
        tfp_sprintf(buff.as_mut_ptr(),
                    b"%d.%1d%c\x00" as *const u8 as *const libc::c_char,
                    stats.min_voltage as libc::c_int / 10 as libc::c_int,
                    stats.min_voltage as libc::c_int % 10 as libc::c_int,
                    0x6 as libc::c_int);
        let fresh14 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh14,
                                 b"MIN BATTERY\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_END_BATTERY as libc::c_int as uint8_t) {
        tfp_sprintf(buff.as_mut_ptr(),
                    b"%d.%1d%c\x00" as *const u8 as *const libc::c_char,
                    endBatteryVoltage as libc::c_int / 10 as libc::c_int,
                    endBatteryVoltage as libc::c_int % 10 as libc::c_int,
                    0x6 as libc::c_int);
        let fresh15 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh15,
                                 b"END BATTERY\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_BATTERY as libc::c_int as uint8_t) {
        tfp_sprintf(buff.as_mut_ptr(),
                    b"%d.%1d%c\x00" as *const u8 as *const libc::c_char,
                    getBatteryVoltage() as libc::c_int / 10 as libc::c_int,
                    getBatteryVoltage() as libc::c_int % 10 as libc::c_int,
                    0x6 as libc::c_int);
        let fresh16 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh16,
                                 b"BATTERY\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_MIN_RSSI as libc::c_int as uint8_t) {
        itoa(stats.min_rssi as libc::c_int, buff.as_mut_ptr(),
             10 as libc::c_int);
        strcat(buff.as_mut_ptr(),
               b"%\x00" as *const u8 as *const libc::c_char);
        let fresh17 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh17,
                                 b"MIN RSSI\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if (*batteryConfig()).currentMeterSource as libc::c_uint !=
           CURRENT_METER_NONE as libc::c_int as libc::c_uint {
        if osdStatGetState(OSD_STAT_MAX_CURRENT as libc::c_int as uint8_t) {
            itoa(stats.max_current as libc::c_int, buff.as_mut_ptr(),
                 10 as libc::c_int);
            strcat(buff.as_mut_ptr(),
                   b"A\x00" as *const u8 as *const libc::c_char);
            let fresh18 = top;
            top = top.wrapping_add(1);
            osdDisplayStatisticLabel(fresh18,
                                     b"MAX CURRENT\x00" as *const u8 as
                                         *const libc::c_char,
                                     buff.as_mut_ptr());
        }
        if osdStatGetState(OSD_STAT_USED_MAH as libc::c_int as uint8_t) {
            tfp_sprintf(buff.as_mut_ptr(),
                        b"%d%c\x00" as *const u8 as *const libc::c_char,
                        getMAhDrawn(), 0x7 as libc::c_int);
            let fresh19 = top;
            top = top.wrapping_add(1);
            osdDisplayStatisticLabel(fresh19,
                                     b"USED MAH\x00" as *const u8 as
                                         *const libc::c_char,
                                     buff.as_mut_ptr());
        }
    }
    if osdStatGetState(OSD_STAT_MAX_ALTITUDE as libc::c_int as uint8_t) {
        osdFormatAltitudeString(buff.as_mut_ptr(), stats.max_altitude);
        let fresh20 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh20,
                                 b"MAX ALTITUDE\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_BLACKBOX as libc::c_int as uint8_t) as
           libc::c_int != 0 && (*blackboxConfig()).device as libc::c_int != 0
           &&
           (*blackboxConfig()).device as libc::c_int !=
               BLACKBOX_DEVICE_SERIAL as libc::c_int {
        osdGetBlackboxStatusString(buff.as_mut_ptr());
        let fresh21 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh21,
                                 b"BLACKBOX\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    }
    if osdStatGetState(OSD_STAT_BLACKBOX_NUMBER as libc::c_int as uint8_t) as
           libc::c_int != 0 && (*blackboxConfig()).device as libc::c_int != 0
           &&
           (*blackboxConfig()).device as libc::c_int !=
               BLACKBOX_DEVICE_SERIAL as libc::c_int {
        itoa(blackboxGetLogNumber() as libc::c_int, buff.as_mut_ptr(),
             10 as libc::c_int);
        let fresh22 = top;
        top = top.wrapping_add(1);
        osdDisplayStatisticLabel(fresh22,
                                 b"BB LOG NUM\x00" as *const u8 as
                                     *const libc::c_char, buff.as_mut_ptr());
    };
}
unsafe extern "C" fn osdShowArmed() {
    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 12 as libc::c_int as uint8_t,
                 7 as libc::c_int as uint8_t,
                 b"ARMED\x00" as *const u8 as *const libc::c_char);
}
unsafe extern "C" fn osdRefresh(mut currentTimeUs: timeUs_t) {
    static mut lastTimeUs: timeUs_t = 0 as libc::c_int as timeUs_t;
    static mut osdStatsEnabled: bool = 0 as libc::c_int != 0;
    static mut osdStatsVisible: bool = 0 as libc::c_int != 0;
    static mut osdStatsRefreshTimeUs: timeUs_t = 0;
    static mut endBatteryVoltage: uint16_t = 0;
    // detect arm/disarm
    if armState as libc::c_int !=
           armingFlags as libc::c_int & ARMED as libc::c_int {
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            osdStatsEnabled = 0 as libc::c_int != 0;
            osdStatsVisible = 0 as libc::c_int != 0;
            osdResetStats();
            osdShowArmed();
            resumeRefreshAt =
                currentTimeUs.wrapping_add((1000 as libc::c_int *
                                                1000 as libc::c_int /
                                                2 as libc::c_int) as
                                               libc::c_uint)
        } else if isSomeStatEnabled() as libc::c_int != 0 &&
                      (getArmingDisableFlags() as libc::c_uint &
                           ARMING_DISABLED_RUNAWAY_TAKEOFF as libc::c_int as
                               libc::c_uint == 0 ||
                           (*osdConfig()).item_pos[OSD_WARNINGS as libc::c_int
                                                       as usize] as
                               libc::c_int & 0x800 as libc::c_int == 0) {
            // suppress stats if runaway takeoff triggered disarm and WARNINGS element is visible
            osdStatsEnabled = 1 as libc::c_int != 0;
            resumeRefreshAt =
                currentTimeUs.wrapping_add((60 as libc::c_int *
                                                1000 as libc::c_int *
                                                1000 as libc::c_int) as
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
            osdStatsEnabled = 0 as libc::c_int != 0;
            resumeRefreshAt = 0 as libc::c_int as timeUs_t;
            stats.armed_time = 0 as libc::c_int as timeUs_t
        } else if IS_RC_MODE_ACTIVE(BOXOSD) as libc::c_int != 0 &&
                      osdStatsVisible as libc::c_int != 0 {
            osdStatsVisible = 0 as libc::c_int != 0;
            displayClearScreen(osdDisplayPort);
        } else if !IS_RC_MODE_ACTIVE(BOXOSD) {
            if !osdStatsVisible {
                osdStatsVisible = 1 as libc::c_int != 0;
                osdStatsRefreshTimeUs = 0 as libc::c_int as timeUs_t
            }
            if currentTimeUs >= osdStatsRefreshTimeUs {
                osdStatsRefreshTimeUs =
                    currentTimeUs.wrapping_add((1000 as libc::c_int *
                                                    1000 as libc::c_int) as
                                                   libc::c_uint);
                osdShowStats(endBatteryVoltage);
            }
        }
    }
    lastTimeUs = currentTimeUs;
    if resumeRefreshAt != 0 {
        if cmp32(currentTimeUs, resumeRefreshAt) < 0 as libc::c_int {
            // in timeout period, check sticks for activity to resume display.
            if rcData[THROTTLE as libc::c_int as usize] as libc::c_int >
                   1750 as libc::c_int ||
                   rcData[PITCH as libc::c_int as usize] as libc::c_int >
                       1750 as libc::c_int {
                resumeRefreshAt = currentTimeUs
            }
            displayHeartbeat(osdDisplayPort);
            return
        } else {
            displayClearScreen(osdDisplayPort);
            resumeRefreshAt = 0 as libc::c_int as timeUs_t;
            osdStatsEnabled = 0 as libc::c_int != 0;
            stats.armed_time = 0 as libc::c_int as timeUs_t
        }
    }
    blinkState =
        currentTimeUs.wrapping_div(200000 as libc::c_int as
                                       libc::c_uint).wrapping_rem(2 as
                                                                      libc::c_int
                                                                      as
                                                                      libc::c_uint)
            != 0;
    if feature(FEATURE_ESC_SENSOR as libc::c_int as uint32_t) {
        escDataCombined = getEscSensorData(255 as libc::c_int as uint8_t)
    }
    if !displayIsGrabbed(osdDisplayPort) {
        osdUpdateAlarms();
        osdDrawElements();
        displayHeartbeat(osdDisplayPort);
    }
    lastArmState = armingFlags as libc::c_int & ARMED as libc::c_int != 0;
}
/*
 * Called periodically by the scheduler
 */
#[no_mangle]
pub unsafe extern "C" fn osdUpdate(mut currentTimeUs: timeUs_t) {
    static mut counter: uint32_t = 0 as libc::c_int as uint32_t;
    if isBeeperOn() { showVisualBeeper = 1 as libc::c_int != 0 }
    // MAX7456_DMA_CHANNEL_TX
    // redraw values in buffer
    // MWOSD @ 115200 baud (
    if counter.wrapping_rem(10 as libc::c_int as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        osdRefresh(currentTimeUs);
        showVisualBeeper = 0 as libc::c_int != 0
    } else {
        // rest of time redraw screen 10 chars per idle so it doesn't lock the main idle
        displayDrawScreen(osdDisplayPort);
    }
    counter = counter.wrapping_add(1);
    // do not allow ARM if we are in menu
    if displayIsGrabbed(osdDisplayPort) {
        setArmingDisabled(ARMING_DISABLED_OSD_MENU);
    } else { unsetArmingDisabled(ARMING_DISABLED_OSD_MENU); };
}
// USE_OSD
