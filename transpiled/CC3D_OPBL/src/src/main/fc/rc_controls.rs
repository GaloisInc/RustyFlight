use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn saveConfigAndNotify();
    #[no_mangle]
    fn changePidProfile(pidProfileIndex: uint8_t);
    #[no_mangle]
    fn changeControlRateProfile(profileIndex: uint8_t);
    #[no_mangle]
    fn applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrimsDelta:
                                               *mut rollAndPitchTrims_u);
    #[no_mangle]
    fn handleInflightCalibrationStickPosition();
    #[no_mangle]
    fn resetArmingDisabled();
    #[no_mangle]
    fn disarm();
    #[no_mangle]
    fn tryArm();
    #[no_mangle]
    fn isTryingToArm() -> bool;
    #[no_mangle]
    fn resetTryingToArm();
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn unsetArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    fn isModeActivationConditionPresent(modeId: boxId_e) -> bool;
    #[no_mangle]
    fn gyroStartCalibration(isFirstArmingCalibration: bool);
    // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    // acc alignment
    // Which acc hardware to use on boards with more than one device
    #[no_mangle]
    fn accSetCalibrationCycles(calibrationCyclesRequired: uint16_t);
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn rxIsReceivingSignal() -> bool;
    #[no_mangle]
    fn getTaskDeltaTime(taskId: cfTaskId_e) -> timeDelta_t;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxConfig_s {
    pub rcmap: [uint8_t; 8],
    pub serialrx_provider: uint8_t,
    pub serialrx_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub spektrum_bind_pin_override_ioTag: ioTag_t,
    pub spektrum_bind_plug_ioTag: ioTag_t,
    pub spektrum_sat_bind: uint8_t,
    pub spektrum_sat_bind_autoreset: uint8_t,
    pub rssi_channel: uint8_t,
    pub rssi_scale: uint8_t,
    pub rssi_invert: uint8_t,
    pub midrc: uint16_t,
    pub mincheck: uint16_t,
    pub maxcheck: uint16_t,
    pub rcInterpolation: uint8_t,
    pub rcInterpolationChannels: uint8_t,
    pub rcInterpolationInterval: uint8_t,
    pub fpvCamAngleDegrees: uint8_t,
    pub airModeActivateThreshold: uint8_t,
    pub rx_min_usec: uint16_t,
    pub rx_max_usec: uint16_t,
    pub max_aux_channel: uint8_t,
    pub rssi_src_frame_errors: uint8_t,
    pub rssi_offset: int8_t,
    pub rc_smoothing_type: uint8_t,
    pub rc_smoothing_input_cutoff: uint8_t,
    pub rc_smoothing_derivative_cutoff: uint8_t,
    pub rc_smoothing_debug_axis: uint8_t,
    pub rc_smoothing_input_type: uint8_t,
    pub rc_smoothing_derivative_type: uint8_t,
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
pub type rxConfig_t = rxConfig_s;
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
// mapping of radio channels to internal RPYTA+ order
// type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
// invert the serial RX protocol compared to it's default setting
// allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
// number of bind pulses for Spektrum satellite receivers
// whenever we will reset (exit) binding mode after hard reboot
// Some radios have not a neutral point centered on 1500. can be changed here
// minimum rc end
// maximum rc end
// Camera angle to be scaled into rc commands
// Throttle setpoint percent where airmode gets activated
// true to use frame drop flags in the rx protocol
// offset applied to the RSSI value before it is returned
// Determines the smoothing algorithm to use: INTERPOLATION or FILTER
// Filter cutoff frequency for the input filter (0 = auto)
// Filter cutoff frequency for the setpoint weight derivative filter (0 = auto)
// Axis to log as debug values when debug_mode = RC_SMOOTHING
// Input filter type (0 = PT1, 1 = BIQUAD)
// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
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
#[derive(Copy, Clone)]
#[repr(C)]
pub union rollAndPitchTrims_u {
    pub raw: [int16_t; 2],
    pub values: rollAndPitchTrims_t_def,
}
pub type rollAndPitchTrims_t_def = rollAndPitchTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
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
pub type throttleStatus_e = libc::c_uint;
pub const THROTTLE_HIGH: throttleStatus_e = 1;
pub const THROTTLE_LOW: throttleStatus_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcControlsConfig_s {
    pub deadband: uint8_t,
    pub yaw_deadband: uint8_t,
    pub alt_hold_deadband: uint8_t,
    pub alt_hold_fast_change: uint8_t,
    pub yaw_control_reversed: bool,
}
pub type rcControlsConfig_t = rcControlsConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flight3DConfig_s {
    pub deadband3d_low: uint16_t,
    pub deadband3d_high: uint16_t,
    pub neutral3d: uint16_t,
    pub deadband3d_throttle: uint16_t,
    pub limit3d_low: uint16_t,
    pub limit3d_high: uint16_t,
    pub switched_mode3d: uint8_t,
}
pub type flight3DConfig_t = flight3DConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct armingConfig_s {
    pub gyro_cal_on_first_arm: uint8_t,
    pub auto_disarm_delay: uint8_t,
}
pub type armingConfig_t = armingConfig_s;
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
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
pub const HORIZON_MODE: C2RustUnnamed_3 = 2;
pub const ANGLE_MODE: C2RustUnnamed_3 = 1;
pub const CALIBRATE_MAG: C2RustUnnamed_4 = 4;
pub const ARMING_DISABLED_RUNAWAY_TAKEOFF: armingDisableFlags_e = 32;
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
pub const ARMING_DISABLED_BOXFAILSAFE: armingDisableFlags_e = 16;
pub const ARMING_DISABLED_BAD_RX_RECOVERY: armingDisableFlags_e = 8;
pub const ARMING_DISABLED_RX_FAILSAFE: armingDisableFlags_e = 4;
pub const ARMING_DISABLED_FAILSAFE: armingDisableFlags_e = 2;
pub const ARMING_DISABLED_NO_GYRO: armingDisableFlags_e = 1;
pub const ARMED: C2RustUnnamed_2 = 1;
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
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 15;
pub const TASK_NONE: cfTaskId_e = 14;
pub const TASK_COUNT: cfTaskId_e = 14;
pub const TASK_LEDSTRIP: cfTaskId_e = 13;
pub const TASK_TELEMETRY: cfTaskId_e = 12;
pub const TASK_BEEPER: cfTaskId_e = 11;
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
pub type pidProfile_t = pidProfile_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_2 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_2 = 2;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_3 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_3 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_3 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_3 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_3 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_3 = 16;
pub const BARO_MODE: C2RustUnnamed_3 = 8;
pub const MAG_MODE: C2RustUnnamed_3 = 4;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_4 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_4 = 8;
pub const GPS_FIX: C2RustUnnamed_4 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_4 = 1;
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn flight3DConfig() -> *const flight3DConfig_t {
    return &mut flight3DConfig_System;
}
// CMS state
// Additional yaw filter when yaw axis too noisy
// Delta Filter in hz
// Biquad dterm notch hz
// Biquad dterm notch low cutoff
// Filter selection for dterm
// Experimental ITerm windup threshold, percent motor saturation
// Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
// Max angle in degrees in level mode
// inclination factor for Horizon mode
// OFF or ON
// Betaflight PID controller parameters
// type of anti gravity method
// max allowed throttle delta before iterm accelerated in ms
// Iterm Accelerator Gain when itermThrottlethreshold is hit
// yaw accel limiter for deg/sec/ms
// accel limiter roll/pitch deg/sec/ms
// dterm crash value
// gyro crash value
// setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
// ms
// ms
// degrees
// degree/second
// Scale PIDsum to battery voltage
// Feed forward weight transition
// limits yaw errorRate, so crashes don't cause huge throttle increase
// Extra PT1 Filter on D in hz
// off, on, on and beeps when it is in crash recovery mode
// how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
// Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
// rotates iterm to translate world errors to local coordinate system
// takes only the larger of P and the D weight feed forward term if they have the same sign.
// Specifies type of relax algorithm
// This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
// Enable iterm suppression during stick input
// Acro trainer roll/pitch angle limit in degrees
// The axis for which record debugging values are captured 0=roll, 1=pitch
// The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
// The lookahead window in milliseconds used to reduce overshoot
// How strongly should the absolute accumulated error be corrected for
// Limit to the correction
// Limit to the accumulated error
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
static mut pidProfile: *mut pidProfile_t =
    0 as *const pidProfile_t as *mut pidProfile_t;
// true if arming is done via the sticks (as opposed to a switch)
static mut isUsingSticksToArm: bool = 1 as libc::c_int != 0;
#[no_mangle]
pub static mut rcCommand: [libc::c_float; 4] = [0.; 4];
// interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut rcControlsConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (25 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<rcControlsConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &rcControlsConfig_System as
                                     *const rcControlsConfig_t as
                                     *mut rcControlsConfig_t as *mut uint8_t,
                             copy:
                                 &rcControlsConfig_Copy as
                                     *const rcControlsConfig_t as
                                     *mut rcControlsConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_rcControlsConfig
                                                         as
                                                         *const rcControlsConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut rcControlsConfig_Copy: rcControlsConfig_t =
    rcControlsConfig_t{deadband: 0,
                       yaw_deadband: 0,
                       alt_hold_deadband: 0,
                       alt_hold_fast_change: 0,
                       yaw_control_reversed: false,};
#[no_mangle]
pub static mut rcControlsConfig_System: rcControlsConfig_t =
    rcControlsConfig_t{deadband: 0,
                       yaw_deadband: 0,
                       alt_hold_deadband: 0,
                       alt_hold_fast_change: 0,
                       yaw_control_reversed: false,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_rcControlsConfig: rcControlsConfig_t =
    {
        let mut init =
            rcControlsConfig_s{deadband: 0 as libc::c_int as uint8_t,
                               yaw_deadband: 0 as libc::c_int as uint8_t,
                               alt_hold_deadband:
                                   40 as libc::c_int as uint8_t,
                               alt_hold_fast_change:
                                   1 as libc::c_int as uint8_t,
                               yaw_control_reversed: 0 as libc::c_int != 0,};
        init
    };
#[no_mangle]
pub static mut armingConfig_System: armingConfig_t =
    armingConfig_t{gyro_cal_on_first_arm: 0, auto_disarm_delay: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut armingConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (16 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<armingConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &armingConfig_System as *const armingConfig_t
                                     as *mut armingConfig_t as *mut uint8_t,
                             copy:
                                 &armingConfig_Copy as *const armingConfig_t
                                     as *mut armingConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_armingConfig
                                                         as
                                                         *const armingConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut armingConfig_Copy: armingConfig_t =
    armingConfig_t{gyro_cal_on_first_arm: 0, auto_disarm_delay: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_armingConfig: armingConfig_t =
    {
        let mut init =
            armingConfig_s{gyro_cal_on_first_arm: 0 as libc::c_int as uint8_t,
                           auto_disarm_delay: 5 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub static mut flight3DConfig_System: flight3DConfig_t =
    flight3DConfig_t{deadband3d_low: 0,
                     deadband3d_high: 0,
                     neutral3d: 0,
                     deadband3d_throttle: 0,
                     limit3d_low: 0,
                     limit3d_high: 0,
                     switched_mode3d: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut flight3DConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (26 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<flight3DConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &flight3DConfig_System as
                                     *const flight3DConfig_t as
                                     *mut flight3DConfig_t as *mut uint8_t,
                             copy:
                                 &flight3DConfig_Copy as
                                     *const flight3DConfig_t as
                                     *mut flight3DConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_flight3DConfig
                                                         as
                                                         *const flight3DConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut flight3DConfig_Copy: flight3DConfig_t =
    flight3DConfig_t{deadband3d_low: 0,
                     deadband3d_high: 0,
                     neutral3d: 0,
                     deadband3d_throttle: 0,
                     limit3d_low: 0,
                     limit3d_high: 0,
                     switched_mode3d: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_flight3DConfig: flight3DConfig_t =
    {
        let mut init =
            flight3DConfig_s{deadband3d_low: 1406 as libc::c_int as uint16_t,
                             deadband3d_high: 1514 as libc::c_int as uint16_t,
                             neutral3d: 1460 as libc::c_int as uint16_t,
                             deadband3d_throttle:
                                 50 as libc::c_int as uint16_t,
                             limit3d_low: 1000 as libc::c_int as uint16_t,
                             limit3d_high: 2000 as libc::c_int as uint16_t,
                             switched_mode3d: 0 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn isUsingSticksForArming() -> bool {
    return isUsingSticksToArm;
}
#[no_mangle]
pub unsafe extern "C" fn areSticksInApModePosition(mut ap_mode: uint16_t)
 -> bool {
    return ({
                let mut _x: libc::c_float =
                    rcCommand[ROLL as libc::c_int as usize];
                (if _x > 0 as libc::c_int as libc::c_float {
                     _x
                 } else { -_x })
            }) < ap_mode as libc::c_int as libc::c_float &&
               ({
                    let mut _x: libc::c_float =
                        rcCommand[PITCH as libc::c_int as usize];
                    (if _x > 0 as libc::c_int as libc::c_float {
                         _x
                     } else { -_x })
                }) < ap_mode as libc::c_int as libc::c_float;
}
// introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
// introduce a deadband around the stick center for yaw axis. Must be greater than zero.
// defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
// when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
// invert control direction of yaw
// min 3d value
// max 3d value
// center 3d value
// default throttle deadband from MIDRC
// pwm output value for max negative thrust
// pwm output value for max positive thrust
// enable '3D Switched Mode'
// allow disarm/arm on throttle down + roll left/right
// allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
#[no_mangle]
pub unsafe extern "C" fn calculateThrottleStatus() -> throttleStatus_e {
    if feature(FEATURE_3D as libc::c_int as uint32_t) {
        if IS_RC_MODE_ACTIVE(BOX3D) as libc::c_int != 0 ||
               (*flight3DConfig()).switched_mode3d as libc::c_int != 0 {
            if (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
                   (*rxConfig()).mincheck as libc::c_int {
                return THROTTLE_LOW
            }
        } else if rcData[THROTTLE as libc::c_int as usize] as libc::c_int >
                      (*rxConfig()).midrc as libc::c_int -
                          (*flight3DConfig()).deadband3d_throttle as
                              libc::c_int &&
                      (rcData[THROTTLE as libc::c_int as usize] as
                           libc::c_int) <
                          (*rxConfig()).midrc as libc::c_int +
                              (*flight3DConfig()).deadband3d_throttle as
                                  libc::c_int {
            return THROTTLE_LOW
        }
    } else if (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
                  (*rxConfig()).mincheck as libc::c_int {
        return THROTTLE_LOW
    }
    return THROTTLE_HIGH;
}
#[no_mangle]
pub unsafe extern "C" fn processRcStickPositions() {
    // time the sticks are maintained
    static mut rcDelayMs: int16_t = 0;
    // hold sticks position for command combos
    static mut rcSticks: uint8_t = 0;
    // an extra guard for disarming through switch to prevent that one frame can disarm it
    static mut rcDisarmTicks: uint8_t = 0;
    static mut doNotRepeat: bool = false;
    // checking sticks positions
    let mut stTmp: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 4 as libc::c_int {
        stTmp = (stTmp as libc::c_int >> 2 as libc::c_int) as uint8_t;
        if rcData[i as usize] as libc::c_int >
               (*rxConfig()).mincheck as libc::c_int {
            stTmp = (stTmp as libc::c_int | 0x80 as libc::c_int) as uint8_t
            // check for MIN
        }
        if (rcData[i as usize] as libc::c_int) <
               (*rxConfig()).maxcheck as libc::c_int {
            stTmp = (stTmp as libc::c_int | 0x40 as libc::c_int) as uint8_t
            // check for MAX
        }
        i += 1
    }
    if stTmp as libc::c_int == rcSticks as libc::c_int {
        if rcDelayMs as libc::c_int <=
               32767 as libc::c_int -
                   getTaskDeltaTime(TASK_SELF) / 1000 as libc::c_int {
            rcDelayMs =
                (rcDelayMs as libc::c_int +
                     getTaskDeltaTime(TASK_SELF) / 1000 as libc::c_int) as
                    int16_t
        }
    } else {
        rcDelayMs = 0 as libc::c_int as int16_t;
        doNotRepeat = 0 as libc::c_int != 0
    }
    rcSticks = stTmp;
    // perform actions
    if !isUsingSticksToArm {
        if IS_RC_MODE_ACTIVE(BOXARM) {
            rcDisarmTicks = 0 as libc::c_int as uint8_t;
            // Arming via ARM BOX
            tryArm();
        } else {
            resetTryingToArm();
            // Disarming via ARM BOX
            resetArmingDisabled();
            if armingFlags as libc::c_int & ARMED as libc::c_int != 0 &&
                   rxIsReceivingSignal() as libc::c_int != 0 &&
                   !failsafeIsActive() {
                rcDisarmTicks = rcDisarmTicks.wrapping_add(1);
                if rcDisarmTicks as libc::c_int > 3 as libc::c_int {
                    disarm();
                }
            }
        }
    } else if rcSticks as libc::c_int ==
                  ((1 as libc::c_int) <<
                       2 as libc::c_int * THROTTLE as libc::c_int) +
                      ((1 as libc::c_int) <<
                           2 as libc::c_int * YAW as libc::c_int) +
                      ((3 as libc::c_int) <<
                           2 as libc::c_int * PITCH as libc::c_int) +
                      ((3 as libc::c_int) <<
                           2 as libc::c_int * ROLL as libc::c_int) {
        if rcDelayMs as libc::c_int >= 500 as libc::c_int && !doNotRepeat {
            doNotRepeat = 1 as libc::c_int != 0;
            // Disarm on throttle down + yaw
            resetTryingToArm(); // sound tone while stick held
            if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                disarm();
            } else {
                beeper(BEEPER_DISARM_REPEAT);
                rcDelayMs =
                    (rcDelayMs as libc::c_int - 250 as libc::c_int) as
                        int16_t;
                doNotRepeat = 0 as libc::c_int != 0;
                // disarm tone will repeat
                // Unset the ARMING_DISABLED_RUNAWAY_TAKEOFF arming disabled flag that might have been set
                // by a runaway pidSum detection auto-disarm.
                // This forces the pilot to explicitly perform a disarm sequence (even though we're implicitly disarmed)
                // before they're able to rearm
                unsetArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
            }
        }
        return
    } else {
        if rcSticks as libc::c_int ==
               ((1 as libc::c_int) <<
                    2 as libc::c_int * THROTTLE as libc::c_int) +
                   ((2 as libc::c_int) <<
                        2 as libc::c_int * YAW as libc::c_int) +
                   ((3 as libc::c_int) <<
                        2 as libc::c_int * PITCH as libc::c_int) +
                   ((3 as libc::c_int) <<
                        2 as libc::c_int * ROLL as libc::c_int) {
            if rcDelayMs as libc::c_int >= 500 as libc::c_int && !doNotRepeat
               {
                doNotRepeat = 1 as libc::c_int != 0;
                if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                    // Arm via YAW
                    tryArm();
                    if isTryingToArm() { doNotRepeat = 0 as libc::c_int != 0 }
                } else { resetArmingDisabled(); }
            }
            return
        } else { resetTryingToArm(); }
    }
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 ||
           doNotRepeat as libc::c_int != 0 ||
           rcDelayMs as libc::c_int <= 50 as libc::c_int ||
           getArmingDisableFlags() as libc::c_uint &
               ARMING_DISABLED_RUNAWAY_TAKEOFF as libc::c_int as libc::c_uint
               != 0 {
        return
    }
    doNotRepeat = 1 as libc::c_int != 0;
    // actions during not armed
    if rcSticks as libc::c_int ==
           ((1 as libc::c_int) << 2 as libc::c_int * THROTTLE as libc::c_int)
               + ((1 as libc::c_int) << 2 as libc::c_int * YAW as libc::c_int)
               +
               ((1 as libc::c_int) << 2 as libc::c_int * PITCH as libc::c_int)
               +
               ((3 as libc::c_int) << 2 as libc::c_int * ROLL as libc::c_int)
       {
        // GYRO calibration
        gyroStartCalibration(0 as libc::c_int != 0);
        return
    }
    if feature(FEATURE_INFLIGHT_ACC_CAL as libc::c_int as uint32_t) as
           libc::c_int != 0 &&
           rcSticks as libc::c_int ==
               ((1 as libc::c_int) <<
                    2 as libc::c_int * THROTTLE as libc::c_int) +
                   ((1 as libc::c_int) <<
                        2 as libc::c_int * YAW as libc::c_int) +
                   ((2 as libc::c_int) <<
                        2 as libc::c_int * PITCH as libc::c_int) +
                   ((2 as libc::c_int) <<
                        2 as libc::c_int * ROLL as libc::c_int) {
        // Inflight ACC Calibration
        handleInflightCalibrationStickPosition();
        return
    }
    // Change PID profile
    match rcSticks as libc::c_int {
        93 => {
            // ROLL left -> PID profile 1
            changePidProfile(0 as libc::c_int as uint8_t);
            return
        }
        91 => {
            // PITCH up -> PID profile 2
            changePidProfile(1 as libc::c_int as uint8_t);
            return
        }
        94 => {
            // ROLL right -> PID profile 3
            changePidProfile(2 as libc::c_int as uint8_t);
            return
        }
        _ => { }
    }
    if rcSticks as libc::c_int ==
           ((1 as libc::c_int) << 2 as libc::c_int * THROTTLE as libc::c_int)
               + ((1 as libc::c_int) << 2 as libc::c_int * YAW as libc::c_int)
               +
               ((1 as libc::c_int) << 2 as libc::c_int * PITCH as libc::c_int)
               +
               ((2 as libc::c_int) << 2 as libc::c_int * ROLL as libc::c_int)
       {
        saveConfigAndNotify();
    }
    if rcSticks as libc::c_int ==
           ((2 as libc::c_int) << 2 as libc::c_int * THROTTLE as libc::c_int)
               + ((1 as libc::c_int) << 2 as libc::c_int * YAW as libc::c_int)
               +
               ((1 as libc::c_int) << 2 as libc::c_int * PITCH as libc::c_int)
               +
               ((3 as libc::c_int) << 2 as libc::c_int * ROLL as libc::c_int)
       {
        // Calibrating Acc
        accSetCalibrationCycles(400 as libc::c_int as uint16_t);
        return
    }
    if rcSticks as libc::c_int ==
           ((2 as libc::c_int) << 2 as libc::c_int * THROTTLE as libc::c_int)
               + ((2 as libc::c_int) << 2 as libc::c_int * YAW as libc::c_int)
               +
               ((1 as libc::c_int) << 2 as libc::c_int * PITCH as libc::c_int)
               +
               ((3 as libc::c_int) << 2 as libc::c_int * ROLL as libc::c_int)
       {
        // Calibrating Mag
        stateFlags =
            (stateFlags as libc::c_int | CALIBRATE_MAG as libc::c_int) as
                uint8_t;
        return
    }
    if flightModeFlags as libc::c_int &
           (ANGLE_MODE as libc::c_int | HORIZON_MODE as libc::c_int) != 0 {
        // in ANGLE or HORIZON mode, so use sticks to apply accelerometer trims
        let mut accelerometerTrimsDelta: rollAndPitchTrims_t =
            rollAndPitchTrims_u{raw: [0; 2],};
        memset(&mut accelerometerTrimsDelta as *mut rollAndPitchTrims_t as
                   *mut libc::c_void, 0 as libc::c_int,
               ::core::mem::size_of::<rollAndPitchTrims_t>() as
                   libc::c_ulong);
        let mut shouldApplyRollAndPitchTrimDelta: bool =
            0 as libc::c_int != 0;
        match rcSticks as libc::c_int {
            187 => {
                accelerometerTrimsDelta.values.pitch =
                    2 as libc::c_int as int16_t;
                shouldApplyRollAndPitchTrimDelta = 1 as libc::c_int != 0
            }
            183 => {
                accelerometerTrimsDelta.values.pitch =
                    -(2 as libc::c_int) as int16_t;
                shouldApplyRollAndPitchTrimDelta = 1 as libc::c_int != 0
            }
            190 => {
                accelerometerTrimsDelta.values.roll =
                    2 as libc::c_int as int16_t;
                shouldApplyRollAndPitchTrimDelta = 1 as libc::c_int != 0
            }
            189 => {
                accelerometerTrimsDelta.values.roll =
                    -(2 as libc::c_int) as int16_t;
                shouldApplyRollAndPitchTrimDelta = 1 as libc::c_int != 0
            }
            _ => { }
        }
        if shouldApplyRollAndPitchTrimDelta {
            applyAndSaveAccelerometerTrimsDelta(&mut accelerometerTrimsDelta);
            rcDelayMs =
                (rcDelayMs as libc::c_int - 250 as libc::c_int) as int16_t;
            doNotRepeat = 0 as libc::c_int != 0;
            return
        }
    } else {
        // in ACRO mode, so use sticks to change RATE profile
        match rcSticks as libc::c_int {
            187 => {
                changeControlRateProfile(0 as libc::c_int as uint8_t);
                return
            }
            183 => {
                changeControlRateProfile(1 as libc::c_int as uint8_t);
                return
            }
            190 => {
                changeControlRateProfile(2 as libc::c_int as uint8_t);
                return
            }
            189 => {
                changeControlRateProfile(3 as libc::c_int as uint8_t);
                return
            }
            _ => { }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn getRcStickDeflection(mut axis: int32_t,
                                              mut midrc: uint16_t)
 -> int32_t {
    return ({
                let mut _a: libc::c_int =
                    ({
                         let mut _x: libc::c_int =
                             rcData[axis as usize] as libc::c_int -
                                 midrc as libc::c_int;
                         if _x > 0 as libc::c_int { _x } else { -_x }
                     });
                let mut _b: libc::c_int = 500 as libc::c_int;
                if _a < _b { _a } else { _b }
            });
}
#[no_mangle]
pub unsafe extern "C" fn useRcControlsConfig(mut pidProfileToUse:
                                                 *mut pidProfile_t) {
    pidProfile = pidProfileToUse;
    isUsingSticksToArm = !isModeActivationConditionPresent(BOXARM);
}
