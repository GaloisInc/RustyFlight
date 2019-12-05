use core;
use libc;
extern "C" {
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn isEEPROMStructureValid() -> bool;
    #[no_mangle]
    fn loadEEPROM() -> bool;
    #[no_mangle]
    fn writeConfigToEEPROM();
    #[no_mangle]
    fn featureConfigured(mask: uint32_t) -> bool;
    #[no_mangle]
    fn featureSet(mask: uint32_t);
    #[no_mangle]
    fn featureClear(mask: uint32_t);
    #[no_mangle]
    fn pgResetAll();
    #[no_mangle]
    fn loadControlRateProfile();
    #[no_mangle]
    static mut throttleCorrectionConfig_System: throttleCorrectionConfig_t;
    #[no_mangle]
    fn initRcProcessing();
    #[no_mangle]
    fn rcSmoothingIsEnabled() -> bool;
    #[no_mangle]
    fn isModeActivationConditionPresent(modeId: boxId_e) -> bool;
    #[no_mangle]
    fn removeModeActivationCondition(modeId: boxId_e);
    // enough for 4 x 3position switches / 4 aux channel
    #[no_mangle]
    fn resetAdjustmentStates();
    #[no_mangle]
    fn useAdjustmentConfig(pidProfileToUse: *mut pidProfile_s);
    #[no_mangle]
    fn useRcControlsConfig(pidProfileToUse: *mut pidProfile_s);
    #[no_mangle]
    static mut failsafeConfig_System: failsafeConfig_t;
    #[no_mangle]
    fn failsafeReset();
    #[no_mangle]
    fn imuConfigure(throttle_correction_angle: uint16_t,
                    throttle_correction_value: uint8_t);
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mixers: [mixer_t; 0];
    #[no_mangle]
    static mut pidProfiles_SystemArray: [pidProfile_t; 3];
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
    #[no_mangle]
    fn pidInit(pidProfile: *const pidProfile_t);
    #[no_mangle]
    static servoMixers: [mixerRules_t; 0];
    #[no_mangle]
    fn beeperConfirmationBeeps(beepCount: uint8_t);
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
    // 1
    // 2
    // 4
    // 8
    // 16
    // 32
    // 64
    // 128
    // 512
    // 1024
    // 2048
    // 4096
    // 8192
    // 16384
    // 32768
    // serial port identifiers are now fixed, these values are used by MSP commands.
    //
// runtime
//
    //
// configuration
//
    // not used for all telemetry systems, e.g. HoTT only works at 19200.
    // which byte is used to reboot. Default 'R', could be changed carefully to something else.
    //
// configuration
//
    #[no_mangle]
    fn isSerialConfigValid(serialConfig_0: *const serialConfig_t) -> bool;
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn pgResetFn_serialConfig(serialConfig_0: *mut serialConfig_t);
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn suspendRxSignal();
    #[no_mangle]
    fn resumeRxSignal();
    #[no_mangle]
    fn setAccelerationTrims(accelerationTrimsToUse:
                                *mut flightDynamicsTrims_u);
    #[no_mangle]
    static mut accelerometerConfig_System: accelerometerConfig_t;
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
    #[no_mangle]
    fn accInitFilters();
    #[no_mangle]
    fn gyroMpuDetectionResult() -> *const mpuDetectionResult_s;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_0 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_0 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_0 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_0 = 4095;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_2 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_2 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_2 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_2 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_2 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_2 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_2 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_2 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_2 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_2 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_2 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_2 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_2 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_2 = 8192;
pub const FEATURE_3D: C2RustUnnamed_2 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_2 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_2 = 512;
pub const FEATURE_GPS: C2RustUnnamed_2 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_2 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_2 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_2 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_2 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_2 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_2 = 1;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pilotConfig_s {
    pub name: [libc::c_char; 17],
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
pub type pilotConfig_t = pilotConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct systemConfig_s {
    pub pidProfileIndex: uint8_t,
    pub activeRateProfile: uint8_t,
    pub debug_mode: uint8_t,
    pub task_statistics: uint8_t,
    pub rateProfile6PosSwitch: uint8_t,
    pub cpu_overclock: uint8_t,
    pub powerOnArmingGraceTime: uint8_t,
    pub boardIdentifier: [libc::c_char; 6],
}
pub type systemConfig_t = systemConfig_s;
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pidf_s {
    pub P: uint8_t,
    pub I: uint8_t,
    pub D: uint8_t,
    pub F: uint16_t,
}
pub type pidProfile_t = pidProfile_s;
pub type throttleCorrectionConfig_t = throttleCorrectionConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct throttleCorrectionConfig_s {
    pub throttle_correction_angle: uint16_t,
    pub throttle_correction_value: uint8_t,
}
// in seconds
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
pub type flightDynamicsTrims_t = flightDynamicsTrims_u;
// cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
// acc alignment
// Which acc hardware to use on boards with more than one device
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union flightDynamicsTrims_u {
    pub raw: [int16_t; 3],
    pub values: flightDynamicsTrims_def_t,
}
pub type flightDynamicsTrims_def_t = int16_flightDynamicsTrims_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct int16_flightDynamicsTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type accelerometerConfig_t = accelerometerConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct accelerometerConfig_s {
    pub acc_lpf_hz: uint16_t,
    pub acc_align: sensor_align_e,
    pub acc_hardware: uint8_t,
    pub acc_high_fsr: bool,
    pub accZero: flightDynamicsTrims_t,
    pub accelerometerTrims: rollAndPitchTrims_t,
}
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union rollAndPitchTrims_u {
    pub raw: [int16_t; 2],
    pub values: rollAndPitchTrims_t_def,
}
pub type rollAndPitchTrims_t_def = rollAndPitchTrims_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
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
pub const FAILSAFE_PROCEDURE_DROP_IT: C2RustUnnamed_4 = 1;
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
// millis
// millis
pub type failsafeConfig_t = failsafeConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct failsafeConfig_s {
    pub failsafe_throttle: uint16_t,
    pub failsafe_throttle_low_delay: uint16_t,
    pub failsafe_delay: uint8_t,
    pub failsafe_off_delay: uint8_t,
    pub failsafe_switch_mode: uint8_t,
    pub failsafe_procedure: uint8_t,
}
pub const FAILSAFE_PROCEDURE_GPS_RESCUE: C2RustUnnamed_4 = 2;
pub const INTERPOLATION_CHANNELS_RPT: C2RustUnnamed_3 = 4;
pub type rxConfig_t = rxConfig_s;
#[derive ( Copy, Clone )]
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
pub const INTERPOLATION_CHANNELS_T: C2RustUnnamed_3 = 3;
pub const INTERPOLATION_CHANNELS_RPYT: C2RustUnnamed_3 = 2;
pub const PID_YAW: C2RustUnnamed_6 = 2;
pub const INTERPOLATION_CHANNELS_RPY: C2RustUnnamed_3 = 1;
pub const PID_PITCH: C2RustUnnamed_6 = 1;
pub const PID_ROLL: C2RustUnnamed_6 = 0;
pub type gyroConfig_t = gyroConfig_s;
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
pub type pidConfig_t = pidConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
pub type motorDevConfig_t = motorDevConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct motorDevConfig_s {
    pub motorPwmRate: uint16_t,
    pub motorPwmProtocol: uint8_t,
    pub motorPwmInversion: uint8_t,
    pub useUnsyncedPwm: uint8_t,
    pub useBurstDshot: uint8_t,
    pub ioTags: [ioTag_t; 8],
}
pub type motorConfig_t = motorConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct motorConfig_s {
    pub dev: motorDevConfig_t,
    pub digitalIdleOffsetValue: uint16_t,
    pub minthrottle: uint16_t,
    pub maxthrottle: uint16_t,
    pub mincommand: uint16_t,
    pub motorPoleCount: uint8_t,
}
pub const PWM_TYPE_STANDARD: C2RustUnnamed_5 = 0;
pub const PWM_TYPE_BRUSHED: C2RustUnnamed_5 = 4;
pub const PWM_TYPE_ONESHOT42: C2RustUnnamed_5 = 2;
pub const PWM_TYPE_ONESHOT125: C2RustUnnamed_5 = 1;
pub const ICM_20649_SPI: mpuSensor_e = 10;
pub type mpuSensor_e = libc::c_uint;
pub const BMI_160_SPI: mpuSensor_e = 12;
pub const ICM_20689_SPI: mpuSensor_e = 11;
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
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
}
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
pub type serialPortConfig_t = serialPortConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPortConfig_s {
    pub functionMask: uint16_t,
    pub identifier: serialPortIdentifier_e,
    pub msp_baudrateIndex: uint8_t,
    pub gps_baudrateIndex: uint8_t,
    pub blackbox_baudrateIndex: uint8_t,
    pub telemetry_baudrateIndex: uint8_t,
}
pub type serialPortIdentifier_e = libc::c_int;
pub const SERIAL_PORT_SOFTSERIAL2: serialPortIdentifier_e = 31;
pub const SERIAL_PORT_SOFTSERIAL1: serialPortIdentifier_e = 30;
pub const SERIAL_PORT_USB_VCP: serialPortIdentifier_e = 20;
pub const SERIAL_PORT_USART8: serialPortIdentifier_e = 7;
pub const SERIAL_PORT_USART7: serialPortIdentifier_e = 6;
pub const SERIAL_PORT_USART6: serialPortIdentifier_e = 5;
pub const SERIAL_PORT_UART5: serialPortIdentifier_e = 4;
pub const SERIAL_PORT_UART4: serialPortIdentifier_e = 3;
pub const SERIAL_PORT_USART3: serialPortIdentifier_e = 2;
pub const SERIAL_PORT_USART2: serialPortIdentifier_e = 1;
pub const SERIAL_PORT_USART1: serialPortIdentifier_e = 0;
pub const SERIAL_PORT_NONE: serialPortIdentifier_e = -1;
pub type serialPortFunction_e = libc::c_uint;
pub const FUNCTION_LIDAR_TF: serialPortFunction_e = 32768;
pub const FUNCTION_RCDEVICE: serialPortFunction_e = 16384;
pub const FUNCTION_VTX_TRAMP: serialPortFunction_e = 8192;
pub const FUNCTION_TELEMETRY_IBUS: serialPortFunction_e = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: serialPortFunction_e = 2048;
pub const FUNCTION_ESC_SENSOR: serialPortFunction_e = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: serialPortFunction_e = 512;
pub const FUNCTION_BLACKBOX: serialPortFunction_e = 128;
pub const FUNCTION_RX_SERIAL: serialPortFunction_e = 64;
pub const FUNCTION_TELEMETRY_SMARTPORT: serialPortFunction_e = 32;
pub const FUNCTION_TELEMETRY_LTM: serialPortFunction_e = 16;
pub const FUNCTION_TELEMETRY_HOTT: serialPortFunction_e = 8;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: serialPortFunction_e = 4;
pub const FUNCTION_GPS: serialPortFunction_e = 2;
pub const FUNCTION_MSP: serialPortFunction_e = 1;
pub const FUNCTION_NONE: serialPortFunction_e = 0;
pub type serialConfig_t = serialConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 8],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
pub const MIXER_CUSTOM_AIRPLANE: mixerMode = 24;
pub type mixerConfig_t = mixerConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
// Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
// Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
// Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
// Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
// failsafe switch action is 0: stage1 (identical to rc link loss), 1: disarms instantly, 2: stage2
// selected full failsafe procedure is 0: auto-landing, 1: Drop it
// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
// Processing denominator for PID controller vs gyro sampling rate
// off, on - enables pidsum runaway disarm logic
// delay in ms for "in-flight" conditions before deactivation (successful flight)
// minimum throttle percent required during deactivation phase
// Idle value for DShot protocol, full motor output = 10000
// Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
// This is the maximum value for the ESCs at full power this value can be increased up to 2000
// This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
// Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
// Custom mixer configuration
pub type mixerRules_t = mixerRules_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mixerRules_s {
    pub servoRuleCount: uint8_t,
    pub rule: *const servoMixer_t,
}
// FIXME rename to servoChannel_e
pub type servoMixer_t = servoMixer_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct servoMixer_s {
    pub targetChannel: uint8_t,
    pub inputSource: uint8_t,
    pub rate: int8_t,
    pub speed: uint8_t,
    pub min: int8_t,
    pub max: int8_t,
    pub box_0: uint8_t,
}
// servo that receives the output of the rule
// input channel for this rule
// range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
// reduces the speed of the rule, 0=unlimited speed
// lower bound of rule range [0;100]% of servo max-min
// lower bound of rule range [0;100]% of servo max-min
// active rule if box is enabled, range [0;3], 0=no box, 1=BOXSERVO1, 2=BOXSERVO2, 3=BOXSERVO3
// Digital protocol has fixed values
// Note: this is called MultiType/MULTITYPE_* in baseflight.
pub type mixerMode_e = mixerMode;
pub type mixerMode = libc::c_uint;
pub const MIXER_QUADX_1234: mixerMode = 26;
pub const MIXER_CUSTOM_TRI: mixerMode = 25;
pub const MIXER_CUSTOM: mixerMode = 23;
pub const MIXER_ATAIL4: mixerMode = 22;
pub const MIXER_SINGLECOPTER: mixerMode = 21;
pub const MIXER_DUALCOPTER: mixerMode = 20;
pub const MIXER_RX_TO_SERVO: mixerMode = 19;
pub const MIXER_HEX6H: mixerMode = 18;
pub const MIXER_VTAIL4: mixerMode = 17;
pub const MIXER_HELI_90_DEG: mixerMode = 16;
pub const MIXER_HELI_120_CCPM: mixerMode = 15;
pub const MIXER_AIRPLANE: mixerMode = 14;
pub const MIXER_OCTOFLATX: mixerMode = 13;
pub const MIXER_OCTOFLATP: mixerMode = 12;
pub const MIXER_OCTOX8: mixerMode = 11;
pub const MIXER_HEX6X: mixerMode = 10;
pub const MIXER_Y4: mixerMode = 9;
pub const MIXER_FLYING_WING: mixerMode = 8;
pub const MIXER_HEX6: mixerMode = 7;
pub const MIXER_Y6: mixerMode = 6;
pub const MIXER_GIMBAL: mixerMode = 5;
pub const MIXER_BICOPTER: mixerMode = 4;
pub const MIXER_QUADX: mixerMode = 3;
pub const MIXER_QUADP: mixerMode = 2;
pub const MIXER_TRI: mixerMode = 1;
// airplane / singlecopter / dualcopter (not yet properly supported)
// PPM -> servo relay
// Custom mixer configuration
pub type mixer_t = mixer_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mixer_s {
    pub motorCount: uint8_t,
    pub useServo: uint8_t,
    pub motor: *const motorMixer_t,
}
// Custom mixer data per motor
pub type motorMixer_t = motorMixer_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct motorMixer_s {
    pub throttle: libc::c_float,
    pub roll: libc::c_float,
    pub pitch: libc::c_float,
    pub yaw: libc::c_float,
}
pub type C2RustUnnamed_3 = libc::c_uint;
pub const INTERPOLATION_CHANNELS_RP: C2RustUnnamed_3 = 0;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FAILSAFE_PROCEDURE_AUTO_LANDING: C2RustUnnamed_4 = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const PWM_TYPE_MAX: C2RustUnnamed_5 = 5;
pub const PWM_TYPE_MULTISHOT: C2RustUnnamed_5 = 3;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_6 = 5;
pub const PID_MAG: C2RustUnnamed_6 = 4;
pub const PID_LEVEL: C2RustUnnamed_6 = 3;
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
pub type C2RustUnnamed_7 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_7 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_7 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_7 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_7 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_7 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_7 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_7 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_7 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_7 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_7 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_7 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_7 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_7 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_7 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_7 = 0;
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
// 0 - 359
// 0 - 255
// 0 - 255
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[inline]
unsafe extern "C" fn systemConfigMutable() -> *mut systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn throttleCorrectionConfig()
 -> *const throttleCorrectionConfig_t {
    return &mut throttleCorrectionConfig_System;
}
#[inline]
unsafe extern "C" fn failsafeConfig() -> *const failsafeConfig_t {
    return &mut failsafeConfig_System;
}
#[inline]
unsafe extern "C" fn failsafeConfigMutable() -> *mut failsafeConfig_t {
    return &mut failsafeConfig_System;
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn mixerConfigMutable() -> *mut mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfigMutable() -> *mut motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn pidProfilesMutable(mut _index: libc::c_int)
 -> *mut pidProfile_t {
    return &mut *pidProfiles_SystemArray.as_mut_ptr().offset(_index as isize)
               as *mut pidProfile_t;
}
#[inline]
unsafe extern "C" fn pidConfigMutable() -> *mut pidConfig_t {
    return &mut pidConfig_System;
}
#[inline]
unsafe extern "C" fn pidConfig() -> *const pidConfig_t {
    return &mut pidConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_7 = INPUT_STABILIZED_ROLL;
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
#[inline]
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn serialConfigMutable() -> *mut serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfigMutable() -> *mut rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn accelerometerConfigMutable()
 -> *mut accelerometerConfig_t {
    return &mut accelerometerConfig_System;
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
pub static mut currentPidProfile: *mut pidProfile_t =
    0 as *const pidProfile_t as *mut pidProfile_t;
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut pilotConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (47i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<pilotConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &pilotConfig_System as *const pilotConfig_t
                                     as *mut pilotConfig_t as *mut uint8_t,
                             copy:
                                 &pilotConfig_Copy as *const pilotConfig_t as
                                     *mut pilotConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_pilotConfig
                                                         as
                                                         *const pilotConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut pilotConfig_System: pilotConfig_t =
    pilotConfig_t{name: [0; 17],};
#[no_mangle]
pub static mut pilotConfig_Copy: pilotConfig_t =
    pilotConfig_t{name: [0; 17],};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_pilotConfig: pilotConfig_t =
    {
        let mut init =
            pilotConfig_s{name:
                              [0i32 as libc::c_char, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0],};
        init
    };
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut systemConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (18i32 | 2i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<systemConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &systemConfig_System as *const systemConfig_t
                                     as *mut systemConfig_t as *mut uint8_t,
                             copy:
                                 &systemConfig_Copy as *const systemConfig_t
                                     as *mut systemConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_systemConfig
                                                         as
                                                         *const systemConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut systemConfig_Copy: systemConfig_t =
    systemConfig_t{pidProfileIndex: 0,
                   activeRateProfile: 0,
                   debug_mode: 0,
                   task_statistics: 0,
                   rateProfile6PosSwitch: 0,
                   cpu_overclock: 0,
                   powerOnArmingGraceTime: 0,
                   boardIdentifier: [0; 6],};
#[no_mangle]
pub static mut systemConfig_System: systemConfig_t =
    systemConfig_t{pidProfileIndex: 0,
                   activeRateProfile: 0,
                   debug_mode: 0,
                   task_statistics: 0,
                   rateProfile6PosSwitch: 0,
                   cpu_overclock: 0,
                   powerOnArmingGraceTime: 0,
                   boardIdentifier: [0; 6],};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_systemConfig: systemConfig_t =
    {
        let mut init =
            systemConfig_s{pidProfileIndex: 0i32 as uint8_t,
                           activeRateProfile: 0i32 as uint8_t,
                           debug_mode: DEBUG_NONE as libc::c_int as uint8_t,
                           task_statistics: 1i32 as uint8_t,
                           rateProfile6PosSwitch: 0,
                           cpu_overclock: 0i32 as uint8_t,
                           powerOnArmingGraceTime: 5i32 as uint8_t,
                           boardIdentifier: [83, 73, 84, 76, 0, 0],};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn getCurrentPidProfileIndex() -> uint8_t {
    return (*systemConfig()).pidProfileIndex;
}
unsafe extern "C" fn loadPidProfile() {
    currentPidProfile =
        pidProfilesMutable((*systemConfig()).pidProfileIndex as libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn getCurrentControlRateProfileIndex() -> uint8_t {
    return (*systemConfig()).activeRateProfile;
}
#[no_mangle]
pub unsafe extern "C" fn getCurrentMinthrottle() -> uint16_t {
    return (*motorConfig()).minthrottle;
}
// USE_OSD_SLAVE
#[no_mangle]
pub unsafe extern "C" fn resetConfigs() { pgResetAll(); }
unsafe extern "C" fn activateConfig() {
    loadPidProfile();
    loadControlRateProfile();
    initRcProcessing();
    resetAdjustmentStates();
    pidInit(currentPidProfile);
    useRcControlsConfig(currentPidProfile);
    useAdjustmentConfig(currentPidProfile);
    failsafeReset();
    setAccelerationTrims(&mut (*(accelerometerConfigMutable as
                                     unsafe extern "C" fn()
                                         ->
                                             *mut accelerometerConfig_t)()).accZero);
    accInitFilters();
    imuConfigure((*throttleCorrectionConfig()).throttle_correction_angle,
                 (*throttleCorrectionConfig()).throttle_correction_value);
    // USE_OSD_SLAVE
}
unsafe extern "C" fn validateAndFixConfig() {
    // Reset unsupported mixer mode to default.
    // This check will be gone when motor/servo mixers are loaded dynamically
    // by configurator as a part of configuration procedure.
    let mut mixerMode: mixerMode_e =
        (*mixerConfigMutable()).mixerMode as mixerMode_e;
    if !(mixerMode as libc::c_uint ==
             MIXER_CUSTOM as libc::c_int as libc::c_uint ||
             mixerMode as libc::c_uint ==
                 MIXER_CUSTOM_AIRPLANE as libc::c_int as libc::c_uint ||
             mixerMode as libc::c_uint ==
                 MIXER_CUSTOM_TRI as libc::c_int as libc::c_uint) {
        if (*mixers.as_ptr().offset(mixerMode as isize)).motorCount as
               libc::c_int != 0 &&
               (*mixers.as_ptr().offset(mixerMode as isize)).motor.is_null() {
            (*mixerConfigMutable()).mixerMode =
                MIXER_CUSTOM as libc::c_int as uint8_t
        }
        if (*mixers.as_ptr().offset(mixerMode as isize)).useServo as
               libc::c_int != 0 &&
               (*servoMixers.as_ptr().offset(mixerMode as
                                                 isize)).servoRuleCount as
                   libc::c_int == 0i32 {
            (*mixerConfigMutable()).mixerMode =
                MIXER_CUSTOM_AIRPLANE as libc::c_int as uint8_t
        }
    }
    if !isSerialConfigValid(serialConfig()) {
        pgResetFn_serialConfig(serialConfigMutable());
    }
    if findSerialPortConfig(FUNCTION_GPS).is_null() && 1i32 != 0 {
        featureClear(FEATURE_GPS as libc::c_int as uint32_t);
    }
    if (*systemConfig()).activeRateProfile as libc::c_int >= 6i32 {
        (*systemConfigMutable()).activeRateProfile = 0i32 as uint8_t
    }
    loadControlRateProfile();
    if (*systemConfig()).pidProfileIndex as libc::c_int >= 3i32 {
        (*systemConfigMutable()).pidProfileIndex = 0i32 as uint8_t
    }
    loadPidProfile();
    // Prevent invalid notch cutoff
    if (*currentPidProfile).dterm_notch_cutoff as libc::c_int >=
           (*currentPidProfile).dterm_notch_hz as libc::c_int {
        (*currentPidProfile).dterm_notch_hz = 0i32 as uint16_t
    }
    if (*motorConfig()).dev.motorPwmProtocol as libc::c_int ==
           PWM_TYPE_BRUSHED as libc::c_int {
        featureClear(FEATURE_3D as libc::c_int as uint32_t);
        if ((*motorConfig()).mincommand as libc::c_int) < 1000i32 {
            (*motorConfigMutable()).mincommand = 1000i32 as uint16_t
        }
    }
    if (*motorConfig()).dev.motorPwmProtocol as libc::c_int ==
           PWM_TYPE_STANDARD as libc::c_int &&
           (*motorConfig()).dev.motorPwmRate as libc::c_int > 480i32 {
        (*motorConfigMutable()).dev.motorPwmRate = 480i32 as uint16_t
    }
    validateAndFixGyroConfig();
    if !(featureConfigured(FEATURE_RX_PARALLEL_PWM as libc::c_int as uint32_t)
             as libc::c_int != 0 ||
             featureConfigured(FEATURE_RX_PPM as libc::c_int as uint32_t) as
                 libc::c_int != 0 ||
             featureConfigured(FEATURE_RX_SERIAL as libc::c_int as uint32_t)
                 as libc::c_int != 0 ||
             featureConfigured(FEATURE_RX_MSP as libc::c_int as uint32_t) as
                 libc::c_int != 0 ||
             featureConfigured(FEATURE_RX_SPI as libc::c_int as uint32_t) as
                 libc::c_int != 0) {
        featureSet(FEATURE_RX_MSP as libc::c_int as uint32_t);
    }
    if featureConfigured(FEATURE_RX_PPM as libc::c_int as uint32_t) {
        featureClear((FEATURE_RX_SERIAL as libc::c_int |
                          FEATURE_RX_PARALLEL_PWM as libc::c_int |
                          FEATURE_RX_MSP as libc::c_int |
                          FEATURE_RX_SPI as libc::c_int) as uint32_t);
    }
    if featureConfigured(FEATURE_RX_MSP as libc::c_int as uint32_t) {
        featureClear((FEATURE_RX_SERIAL as libc::c_int |
                          FEATURE_RX_PARALLEL_PWM as libc::c_int |
                          FEATURE_RX_PPM as libc::c_int |
                          FEATURE_RX_SPI as libc::c_int) as uint32_t);
    }
    if featureConfigured(FEATURE_RX_SERIAL as libc::c_int as uint32_t) {
        featureClear((FEATURE_RX_PARALLEL_PWM as libc::c_int |
                          FEATURE_RX_MSP as libc::c_int |
                          FEATURE_RX_PPM as libc::c_int |
                          FEATURE_RX_SPI as libc::c_int) as uint32_t);
    }
    // USE_RX_SPI
    if featureConfigured(FEATURE_RX_PARALLEL_PWM as libc::c_int as uint32_t) {
        featureClear((FEATURE_RX_SERIAL as libc::c_int |
                          FEATURE_RX_MSP as libc::c_int |
                          FEATURE_RX_PPM as libc::c_int |
                          FEATURE_RX_SPI as libc::c_int) as uint32_t);
    }
    // USE_SOFTSPI
    if (*rxConfigMutable()).rssi_channel != 0 {
        (*rxConfigMutable()).rssi_src_frame_errors = 0i32 as uint8_t
    }
    if !rcSmoothingIsEnabled() ||
           (*rxConfig()).rcInterpolationChannels as libc::c_int ==
               INTERPOLATION_CHANNELS_T as libc::c_int {
        let mut i: libc::c_uint = 0i32 as libc::c_uint;
        while i < 3i32 as libc::c_uint {
            (*pidProfilesMutable(i as
                                     libc::c_int)).pid[PID_ROLL as libc::c_int
                                                           as usize].F =
                0i32 as uint16_t;
            (*pidProfilesMutable(i as
                                     libc::c_int)).pid[PID_PITCH as
                                                           libc::c_int as
                                                           usize].F =
                0i32 as uint16_t;
            i = i.wrapping_add(1)
        }
    }
    if !rcSmoothingIsEnabled() ||
           (*rxConfig()).rcInterpolationChannels as libc::c_int !=
               INTERPOLATION_CHANNELS_RPY as libc::c_int &&
               (*rxConfig()).rcInterpolationChannels as libc::c_int !=
                   INTERPOLATION_CHANNELS_RPYT as libc::c_int {
        let mut i_0: libc::c_uint = 0i32 as libc::c_uint;
        while i_0 < 3i32 as libc::c_uint {
            (*pidProfilesMutable(i_0 as
                                     libc::c_int)).pid[PID_YAW as libc::c_int
                                                           as usize].F =
                0i32 as uint16_t;
            i_0 = i_0.wrapping_add(1)
        }
    }
    if !rcSmoothingIsEnabled() ||
           !((*rxConfig()).rcInterpolationChannels as libc::c_int ==
                 INTERPOLATION_CHANNELS_RPYT as libc::c_int ||
                 (*rxConfig()).rcInterpolationChannels as libc::c_int ==
                     INTERPOLATION_CHANNELS_T as libc::c_int ||
                 (*rxConfig()).rcInterpolationChannels as libc::c_int ==
                     INTERPOLATION_CHANNELS_RPT as libc::c_int) {
        let mut i_1: libc::c_uint = 0i32 as libc::c_uint;
        while i_1 < 3i32 as libc::c_uint {
            (*pidProfilesMutable(i_1 as libc::c_int)).throttle_boost =
                0i32 as uint8_t;
            i_1 = i_1.wrapping_add(1)
        }
    }
    if featureConfigured(FEATURE_3D as libc::c_int as uint32_t) as libc::c_int
           != 0 || !featureConfigured(FEATURE_GPS as libc::c_int as uint32_t)
           || 1i32 != 0 {
        if (*failsafeConfig()).failsafe_procedure as libc::c_int ==
               FAILSAFE_PROCEDURE_GPS_RESCUE as libc::c_int {
            (*failsafeConfigMutable()).failsafe_procedure =
                FAILSAFE_PROCEDURE_DROP_IT as libc::c_int as uint8_t
        }
        if isModeActivationConditionPresent(BOXGPSRESCUE) {
            removeModeActivationCondition(BOXGPSRESCUE);
        }
    }
    // USE_OSD_SLAVE
    // clear features that are not supported.
// I have kept them all here in one place, some could be moved to sections of code above.
    featureClear(FEATURE_RX_PPM as libc::c_int as uint32_t);
    featureClear(FEATURE_RX_SERIAL as libc::c_int as uint32_t);
    featureClear(FEATURE_SOFTSERIAL as libc::c_int as uint32_t);
    featureClear(FEATURE_RANGEFINDER as libc::c_int as uint32_t);
    featureClear(FEATURE_RX_PARALLEL_PWM as libc::c_int as uint32_t);
    featureClear(FEATURE_LED_STRIP as libc::c_int as uint32_t);
    featureClear(FEATURE_DASHBOARD as libc::c_int as uint32_t);
    featureClear(FEATURE_OSD as libc::c_int as uint32_t);
    featureClear(FEATURE_TRANSPONDER as libc::c_int as uint32_t);
    featureClear(FEATURE_RX_SPI as libc::c_int as uint32_t);
    featureClear(FEATURE_SOFTSPI as libc::c_int as uint32_t);
    featureClear(FEATURE_ESC_SENSOR as libc::c_int as uint32_t);
    featureClear(FEATURE_DYNAMIC_FILTER as libc::c_int as uint32_t);
    featureClear(FEATURE_RSSI_ADC as libc::c_int as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn validateAndFixGyroConfig() {
    // Prevent invalid notch cutoff
    if (*gyroConfig()).gyro_soft_notch_cutoff_1 as libc::c_int >=
           (*gyroConfig()).gyro_soft_notch_hz_1 as libc::c_int {
        (*gyroConfigMutable()).gyro_soft_notch_hz_1 = 0i32 as uint16_t
    } // When gyro set to 1khz always set pid speed 1:1 to sampling speed
    if (*gyroConfig()).gyro_soft_notch_cutoff_2 as libc::c_int >=
           (*gyroConfig()).gyro_soft_notch_hz_2 as libc::c_int {
        (*gyroConfigMutable()).gyro_soft_notch_hz_2 = 0i32 as uint16_t
    }
    if (*gyroConfig()).gyro_hardware_lpf as libc::c_int != 0i32 &&
           (*gyroConfig()).gyro_hardware_lpf as libc::c_int != 1i32 {
        (*pidConfigMutable()).pid_process_denom = 1i32 as uint8_t;
        (*gyroConfigMutable()).gyro_sync_denom = 1i32 as uint8_t;
        (*gyroConfigMutable()).gyro_use_32khz = 0i32 as uint8_t
    }
    ((*gyroConfig()).gyro_use_32khz) != 0;
    let mut samplingTime: libc::c_float = 0.;
    match (*gyroMpuDetectionResult()).sensor as libc::c_uint {
        10 => { samplingTime = 1.0f32 / 9000.0f32 }
        12 => { samplingTime = 0.0003125f32 }
        _ => { samplingTime = 0.000125f32 }
    }
    if (*gyroConfig()).gyro_hardware_lpf as libc::c_int != 0i32 &&
           (*gyroConfig()).gyro_hardware_lpf as libc::c_int != 1i32 {
        match (*gyroMpuDetectionResult()).sensor as libc::c_uint {
            10 => { samplingTime = 1.0f32 / 1100.0f32 }
            _ => { samplingTime = 0.001f32 }
        }
    }
    if (*gyroConfig()).gyro_use_32khz != 0 {
        samplingTime = 0.00003125f64 as libc::c_float
    }
    // check for looptime restrictions based on motor protocol. Motor times have safety margin
    let mut motorUpdateRestriction: libc::c_float = 0.;
    match (*motorConfig()).dev.motorPwmProtocol as libc::c_int {
        0 => { motorUpdateRestriction = 1.0f32 / 480i32 as libc::c_float }
        1 => { motorUpdateRestriction = 0.0005f32 }
        2 => { motorUpdateRestriction = 0.0001f32 }
        _ => { motorUpdateRestriction = 0.00003125f32 }
    }
    if (*motorConfig()).dev.useUnsyncedPwm != 0 {
        // Prevent overriding the max rate of motors
        if (*motorConfig()).dev.motorPwmProtocol as libc::c_int <=
               PWM_TYPE_BRUSHED as libc::c_int &&
               (*motorConfig()).dev.motorPwmProtocol as libc::c_int !=
                   PWM_TYPE_STANDARD as libc::c_int {
            let maxEscRate: uint32_t =
                lrintf(1.0f32 / motorUpdateRestriction) as uint32_t;
            (*motorConfigMutable()).dev.motorPwmRate =
                ({
                     let _a: uint16_t = (*motorConfig()).dev.motorPwmRate;
                     let _b: uint32_t = maxEscRate;
                     if (_a as libc::c_uint) < _b {
                         _a as libc::c_uint
                     } else { _b }
                 }) as uint16_t
        }
    } else {
        let pidLooptime: libc::c_float =
            samplingTime *
                (*gyroConfig()).gyro_sync_denom as libc::c_int as
                    libc::c_float *
                (*pidConfig()).pid_process_denom as libc::c_int as
                    libc::c_float;
        if pidLooptime < motorUpdateRestriction {
            let minPidProcessDenom: uint8_t =
                constrain((motorUpdateRestriction /
                               (samplingTime *
                                    (*gyroConfig()).gyro_sync_denom as
                                        libc::c_int as libc::c_float)) as
                              libc::c_int, 1i32, 16i32) as uint8_t;
            (*pidConfigMutable()).pid_process_denom =
                ({
                     let mut _a: uint8_t =
                         (*pidConfigMutable()).pid_process_denom;
                     let _b: uint8_t = minPidProcessDenom;
                     if _a as libc::c_int > _b as libc::c_int {
                         _a as libc::c_int
                     } else { _b as libc::c_int }
                 }) as uint8_t
        }
    };
}
// USE_OSD_SLAVE
#[no_mangle]
pub unsafe extern "C" fn readEEPROM() -> bool {
    suspendRxSignal();
    // Sanity check, read flash
    let mut success: bool = loadEEPROM();
    validateAndFixConfig();
    activateConfig();
    resumeRxSignal();
    return success;
}
#[no_mangle]
pub unsafe extern "C" fn writeEEPROM() {
    validateAndFixConfig();
    suspendRxSignal();
    writeConfigToEEPROM();
    resumeRxSignal();
}
#[no_mangle]
pub unsafe extern "C" fn resetEEPROM() {
    resetConfigs();
    writeEEPROM();
    activateConfig();
}
#[no_mangle]
pub unsafe extern "C" fn ensureEEPROMStructureIsValid() {
    if isEEPROMStructureValid() { return }
    resetEEPROM();
}
#[no_mangle]
pub unsafe extern "C" fn saveConfigAndNotify() {
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(1i32 as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn changePidProfile(mut pidProfileIndex: uint8_t) {
    if (pidProfileIndex as libc::c_int) < 3i32 {
        (*systemConfigMutable()).pidProfileIndex = pidProfileIndex;
        loadPidProfile();
    }
    beeperConfirmationBeeps((pidProfileIndex as libc::c_int + 1i32) as
                                uint8_t);
}
