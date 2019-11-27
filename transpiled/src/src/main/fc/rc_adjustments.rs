use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    static mut blackboxConfig_System: blackboxConfig_t;
    #[no_mangle]
    fn blackboxLogEvent(event: FlightLogEvent,
                        data: *mut flightLogEventData_u);
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn pidInitConfig(pidProfile_0: *const pidProfile_t);
    #[no_mangle]
    fn beeperConfirmationBeeps(beepCount: uint8_t);
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    fn getCurrentControlRateProfileIndex() -> uint8_t;
    #[no_mangle]
    fn changeControlRateProfile(profileIndex: uint8_t);
    #[no_mangle]
    fn isRangeActive(auxChannelIndex: uint8_t, range: *const channelRange_t)
     -> bool;
    #[no_mangle]
    fn initRcProcessing();
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn rxIsReceivingSignal() -> bool;
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
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
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
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// millisecond time
pub type timeMs_t = uint32_t;
pub type FlightLogEvent = libc::c_uint;
pub const FLIGHT_LOG_EVENT_LOG_END: FlightLogEvent = 255;
pub const FLIGHT_LOG_EVENT_FLIGHTMODE: FlightLogEvent = 30;
pub const FLIGHT_LOG_EVENT_LOGGING_RESUME: FlightLogEvent = 14;
pub const FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT: FlightLogEvent = 13;
pub const FLIGHT_LOG_EVENT_SYNC_BEEP: FlightLogEvent = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union flightLogEventData_u {
    pub syncBeep: flightLogEvent_syncBeep_t,
    pub flightMode: flightLogEvent_flightMode_t,
    pub inflightAdjustment: flightLogEvent_inflightAdjustment_t,
    pub loggingResume: flightLogEvent_loggingResume_t,
}
pub type flightLogEvent_loggingResume_t = flightLogEvent_loggingResume_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flightLogEvent_loggingResume_s {
    pub logIteration: uint32_t,
    pub currentTime: uint32_t,
}
pub type flightLogEvent_inflightAdjustment_t
    =
    flightLogEvent_inflightAdjustment_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flightLogEvent_inflightAdjustment_s {
    pub newValue: int32_t,
    pub newFloatValue: libc::c_float,
    pub adjustmentFunction: uint8_t,
    pub floatFlag: bool,
}
pub type flightLogEvent_flightMode_t = flightLogEvent_flightMode_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flightLogEvent_flightMode_s {
    pub flags: uint32_t,
    pub lastFlags: uint32_t,
}
pub type flightLogEvent_syncBeep_t = flightLogEvent_syncBeep_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flightLogEvent_syncBeep_s {
    pub time: uint32_t,
}
pub type flightLogEventData_t = flightLogEventData_u;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FD_YAW: C2RustUnnamed_1 = 2;
pub const FD_PITCH: C2RustUnnamed_1 = 1;
pub const FD_ROLL: C2RustUnnamed_1 = 0;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_2 = 5;
pub const PID_MAG: C2RustUnnamed_2 = 4;
pub const PID_LEVEL: C2RustUnnamed_2 = 3;
pub const PID_YAW: C2RustUnnamed_2 = 2;
pub const PID_PITCH: C2RustUnnamed_2 = 1;
pub const PID_ROLL: C2RustUnnamed_2 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pidf_s {
    pub P: uint8_t,
    pub I: uint8_t,
    pub D: uint8_t,
    pub F: uint16_t,
}
pub type pidf_t = pidf_s;
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
pub type pidProfile_t = pidProfile_s;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const PID_AUDIO_PIDSUM_XY: C2RustUnnamed_3 = 3;
pub const PID_AUDIO_PIDSUM_Y: C2RustUnnamed_3 = 2;
pub const PID_AUDIO_PIDSUM_X: C2RustUnnamed_3 = 1;
pub const PID_AUDIO_OFF: C2RustUnnamed_3 = 0;
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
pub struct controlRateConfig_s {
    pub thrMid8: uint8_t,
    pub thrExpo8: uint8_t,
    pub rates_type: uint8_t,
    pub rcRates: [uint8_t; 3],
    pub rcExpo: [uint8_t; 3],
    pub rates: [uint8_t; 3],
    pub dynThrPID: uint8_t,
    pub tpa_breakpoint: uint16_t,
    pub throttle_limit_type: uint8_t,
    pub throttle_limit_percent: uint8_t,
}
pub type controlRateConfig_t = controlRateConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
pub type channelRange_t = channelRange_s;
pub type adjustmentFunction_e = libc::c_uint;
pub const ADJUSTMENT_FUNCTION_COUNT: adjustmentFunction_e = 33;
pub const ADJUSTMENT_YAW_F: adjustmentFunction_e = 32;
pub const ADJUSTMENT_ROLL_F: adjustmentFunction_e = 31;
pub const ADJUSTMENT_PITCH_F: adjustmentFunction_e = 30;
pub const ADJUSTMENT_PID_AUDIO: adjustmentFunction_e = 29;
pub const ADJUSTMENT_PITCH_RC_EXPO: adjustmentFunction_e = 28;
pub const ADJUSTMENT_ROLL_RC_EXPO: adjustmentFunction_e = 27;
pub const ADJUSTMENT_PITCH_RC_RATE: adjustmentFunction_e = 26;
pub const ADJUSTMENT_ROLL_RC_RATE: adjustmentFunction_e = 25;
pub const ADJUSTMENT_HORIZON_STRENGTH: adjustmentFunction_e = 24;
pub const ADJUSTMENT_FEEDFORWARD_TRANSITION: adjustmentFunction_e = 23;
pub const ADJUSTMENT_PITCH_ROLL_F: adjustmentFunction_e = 22;
pub const ADJUSTMENT_RC_RATE_YAW: adjustmentFunction_e = 21;
pub const ADJUSTMENT_ROLL_D: adjustmentFunction_e = 20;
pub const ADJUSTMENT_ROLL_I: adjustmentFunction_e = 19;
pub const ADJUSTMENT_ROLL_P: adjustmentFunction_e = 18;
pub const ADJUSTMENT_PITCH_D: adjustmentFunction_e = 17;
pub const ADJUSTMENT_PITCH_I: adjustmentFunction_e = 16;
pub const ADJUSTMENT_PITCH_P: adjustmentFunction_e = 15;
pub const ADJUSTMENT_ROLL_RATE: adjustmentFunction_e = 14;
pub const ADJUSTMENT_PITCH_RATE: adjustmentFunction_e = 13;
pub const ADJUSTMENT_RATE_PROFILE: adjustmentFunction_e = 12;
pub const ADJUSTMENT_YAW_D: adjustmentFunction_e = 11;
pub const ADJUSTMENT_YAW_I: adjustmentFunction_e = 10;
pub const ADJUSTMENT_YAW_P: adjustmentFunction_e = 9;
pub const ADJUSTMENT_PITCH_ROLL_D: adjustmentFunction_e = 8;
pub const ADJUSTMENT_PITCH_ROLL_I: adjustmentFunction_e = 7;
pub const ADJUSTMENT_PITCH_ROLL_P: adjustmentFunction_e = 6;
pub const ADJUSTMENT_YAW_RATE: adjustmentFunction_e = 5;
pub const ADJUSTMENT_PITCH_ROLL_RATE: adjustmentFunction_e = 4;
pub const ADJUSTMENT_THROTTLE_EXPO: adjustmentFunction_e = 3;
pub const ADJUSTMENT_RC_EXPO: adjustmentFunction_e = 2;
pub const ADJUSTMENT_RC_RATE: adjustmentFunction_e = 1;
pub const ADJUSTMENT_NONE: adjustmentFunction_e = 0;
pub type adjustmentMode_e = libc::c_uint;
pub const ADJUSTMENT_MODE_SELECT: adjustmentMode_e = 1;
pub const ADJUSTMENT_MODE_STEP: adjustmentMode_e = 0;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union adjustmentConfig_u {
    pub step: uint8_t,
    pub switchPositions: uint8_t,
}
pub type adjustmentData_t = adjustmentConfig_u;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct adjustmentConfig_s {
    pub adjustmentFunction: adjustmentFunction_e,
    pub mode: adjustmentMode_e,
    pub data: adjustmentData_t,
}
pub type adjustmentConfig_t = adjustmentConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct adjustmentRange_s {
    pub auxChannelIndex: uint8_t,
    pub range: channelRange_t,
    pub adjustmentFunction: uint8_t,
    pub auxSwitchChannelIndex: uint8_t,
    pub adjustmentIndex: uint8_t,
    pub adjustmentCenter: uint16_t,
    pub adjustmentScale: uint16_t,
}
pub type adjustmentRange_t = adjustmentRange_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct adjustmentState_s {
    pub auxChannelIndex: uint8_t,
    pub config: *const adjustmentConfig_t,
    pub timeoutAt: uint32_t,
}
pub type adjustmentState_t = adjustmentState_s;
#[inline]
unsafe extern "C" fn cmp32(mut a: uint32_t, mut b: uint32_t) -> int32_t {
    return a.wrapping_sub(b) as int32_t;
}
#[inline]
unsafe extern "C" fn blackboxConfig() -> *const blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn adjustmentRanges(mut _index: libc::c_int)
 -> *const adjustmentRange_t {
    return &mut *adjustmentRanges_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut adjustmentRange_t;
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
// in seconds
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// when aux channel is in range...
// ..then apply the adjustment function to the auxSwitchChannel ...
// ... via slot
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
pub static mut adjustmentRanges_SystemArray: [adjustmentRange_t; 15] =
    [adjustmentRange_t{auxChannelIndex: 0,
                       range: channelRange_t{startStep: 0, endStep: 0,},
                       adjustmentFunction: 0,
                       auxSwitchChannelIndex: 0,
                       adjustmentIndex: 0,
                       adjustmentCenter: 0,
                       adjustmentScale: 0,}; 15];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut adjustmentRanges_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_0{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
#[no_mangle]
pub static mut adjustmentRanges_CopyArray: [adjustmentRange_t; 15] =
    [adjustmentRange_t{auxChannelIndex: 0,
                       range: channelRange_t{startStep: 0, endStep: 0,},
                       adjustmentFunction: 0,
                       auxSwitchChannelIndex: 0,
                       adjustmentIndex: 0,
                       adjustmentCenter: 0,
                       adjustmentScale: 0,}; 15];
#[no_mangle]
pub static mut pidAudioPositionToModeMap: [uint8_t; 7] =
    [PID_AUDIO_PIDSUM_X as libc::c_int as uint8_t,
     PID_AUDIO_PIDSUM_Y as libc::c_int as uint8_t,
     PID_AUDIO_PIDSUM_XY as libc::c_int as uint8_t,
     PID_AUDIO_OFF as libc::c_int as uint8_t,
     PID_AUDIO_OFF as libc::c_int as uint8_t,
     PID_AUDIO_OFF as libc::c_int as uint8_t,
     PID_AUDIO_OFF as libc::c_int as uint8_t];
static mut pidProfile: *mut pidProfile_t =
    0 as *const pidProfile_t as *mut pidProfile_t;
unsafe extern "C" fn blackboxLogInflightAdjustmentEvent(mut adjustmentFunction:
                                                            adjustmentFunction_e,
                                                        mut newValue:
                                                            int32_t) {
    if (*blackboxConfig()).device != 0 {
        let mut eventData: flightLogEvent_inflightAdjustment_t =
            flightLogEvent_inflightAdjustment_t{newValue: 0,
                                                newFloatValue: 0.,
                                                adjustmentFunction: 0,
                                                floatFlag: false,};
        eventData.adjustmentFunction = adjustmentFunction as uint8_t;
        eventData.newValue = newValue;
        eventData.floatFlag = 0i32 != 0;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT,
                         &mut eventData as
                             *mut flightLogEvent_inflightAdjustment_t as
                             *mut flightLogEventData_t);
    };
}
static mut adjustmentStateMask: uint8_t = 0i32 as uint8_t;
// sync with adjustmentFunction_e
// Initialized in run_static_initializers
static mut defaultAdjustmentConfigs: [adjustmentConfig_t; 32] =
    [adjustmentConfig_t{adjustmentFunction: ADJUSTMENT_NONE,
                        mode: ADJUSTMENT_MODE_STEP,
                        data: adjustmentConfig_u{step: 0,},}; 32];
static mut adjustmentStates: [adjustmentState_t; 4] =
    [adjustmentState_t{auxChannelIndex: 0,
                       config: 0 as *const adjustmentConfig_t,
                       timeoutAt: 0,}; 4];
unsafe extern "C" fn configureAdjustment(mut index: uint8_t,
                                         mut auxSwitchChannelIndex: uint8_t,
                                         mut adjustmentConfig:
                                             *const adjustmentConfig_t) {
    let mut adjustmentState: *mut adjustmentState_t =
        &mut *adjustmentStates.as_mut_ptr().offset(index as isize) as
            *mut adjustmentState_t;
    if (*adjustmentState).config == adjustmentConfig {
        // already configured
        return
    } // FIXME magic numbers repeated in cli.c
    (*adjustmentState).auxChannelIndex =
        auxSwitchChannelIndex; // FIXME magic numbers repeated in cli.c
    (*adjustmentState).config = adjustmentConfig;
    (*adjustmentState).timeoutAt = 0i32 as uint32_t;
    adjustmentStateMask =
        (adjustmentStateMask as libc::c_int & !(1i32 << index as libc::c_int))
            as uint8_t;
}
unsafe extern "C" fn applyStepAdjustment(mut controlRateConfig:
                                             *mut controlRateConfig_t,
                                         mut adjustmentFunction: uint8_t,
                                         mut delta: libc::c_int)
 -> libc::c_int {
    beeperConfirmationBeeps(if delta > 0i32 { 2i32 } else { 1i32 } as
                                uint8_t);
    let mut newValue: libc::c_int = 0;
    let mut current_block_82: u64;
    match adjustmentFunction as libc::c_int {
        1 | 25 => {
            newValue =
                constrain((*controlRateConfig).rcRates[FD_ROLL as libc::c_int
                                                           as usize] as
                              libc::c_int + delta, 0i32, 255i32);
            (*controlRateConfig).rcRates[FD_ROLL as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_RATE,
                                               newValue);
            if adjustmentFunction as libc::c_int ==
                   ADJUSTMENT_ROLL_RC_RATE as libc::c_int {
                current_block_82 = 15447629348493591490;
            } else { current_block_82 = 15932841215234547859; }
        }
        26 => { current_block_82 = 15932841215234547859; }
        2 | 27 => {
            newValue =
                constrain((*controlRateConfig).rcExpo[FD_ROLL as libc::c_int
                                                          as usize] as
                              libc::c_int + delta, 0i32, 100i32);
            (*controlRateConfig).rcExpo[FD_ROLL as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_EXPO,
                                               newValue);
            if adjustmentFunction as libc::c_int ==
                   ADJUSTMENT_ROLL_RC_EXPO as libc::c_int {
                current_block_82 = 15447629348493591490;
            } else { current_block_82 = 8697802668005419385; }
        }
        28 => { current_block_82 = 8697802668005419385; }
        3 => {
            newValue =
                constrain((*controlRateConfig).thrExpo8 as libc::c_int +
                              delta, 0i32, 100i32);
            (*controlRateConfig).thrExpo8 = newValue as uint8_t;
            initRcProcessing();
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_THROTTLE_EXPO,
                                               newValue);
            current_block_82 = 15447629348493591490;
        }
        4 | 13 => {
            newValue =
                constrain((*controlRateConfig).rates[FD_PITCH as libc::c_int
                                                         as usize] as
                              libc::c_int + delta, 0i32, 255i32);
            (*controlRateConfig).rates[FD_PITCH as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RATE,
                                               newValue);
            if adjustmentFunction as libc::c_int ==
                   ADJUSTMENT_PITCH_RATE as libc::c_int {
                current_block_82 = 15447629348493591490;
            } else { current_block_82 = 2128805812479287575; }
        }
        14 => { current_block_82 = 2128805812479287575; }
        5 => {
            newValue =
                constrain((*controlRateConfig).rates[FD_YAW as libc::c_int as
                                                         usize] as libc::c_int
                              + delta, 0i32, 255i32);
            (*controlRateConfig).rates[FD_YAW as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_RATE, newValue);
            current_block_82 = 15447629348493591490;
        }
        6 | 15 => {
            newValue =
                constrain((*pidProfile).pid[PID_PITCH as libc::c_int as
                                                usize].P as libc::c_int +
                              delta, 0i32, 200i32);
            (*pidProfile).pid[PID_PITCH as libc::c_int as usize].P =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_P, newValue);
            if adjustmentFunction as libc::c_int ==
                   ADJUSTMENT_PITCH_P as libc::c_int {
                current_block_82 = 15447629348493591490;
            } else {
                // fall through for combined ADJUSTMENT_PITCH_ROLL_P
                current_block_82 =
                    10622320711346489414; // FIXME magic numbers repeated in cli.c
            }
        }
        18 => { current_block_82 = 10622320711346489414; }
        7 | 16 => {
            newValue =
                constrain((*pidProfile).pid[PID_PITCH as libc::c_int as
                                                usize].I as libc::c_int +
                              delta, 0i32, 200i32);
            (*pidProfile).pid[PID_PITCH as libc::c_int as usize].I =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_I, newValue);
            if adjustmentFunction as libc::c_int ==
                   ADJUSTMENT_PITCH_I as libc::c_int {
                current_block_82 = 15447629348493591490;
            } else {
                // fall through for combined ADJUSTMENT_PITCH_ROLL_I
                current_block_82 =
                    14549389285293717636; // FIXME magic numbers repeated in cli.c
            }
        }
        19 => { current_block_82 = 14549389285293717636; }
        8 | 17 => {
            newValue =
                constrain((*pidProfile).pid[PID_PITCH as libc::c_int as
                                                usize].D as libc::c_int +
                              delta, 0i32, 200i32);
            (*pidProfile).pid[PID_PITCH as libc::c_int as usize].D =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_D, newValue);
            if adjustmentFunction as libc::c_int ==
                   ADJUSTMENT_PITCH_D as libc::c_int {
                current_block_82 = 15447629348493591490;
            } else {
                // fall through for combined ADJUSTMENT_PITCH_ROLL_D
                current_block_82 =
                    3185416327625189411; // FIXME magic numbers repeated in cli.c
            }
        }
        20 => {
            current_block_82 =
                3185416327625189411; // FIXME magic numbers repeated in cli.c
        }
        9 => {
            newValue =
                constrain((*pidProfile).pid[PID_YAW as libc::c_int as usize].P
                              as libc::c_int + delta, 0i32,
                          200i32); // FIXME magic numbers repeated in cli.c
            (*pidProfile).pid[PID_YAW as libc::c_int as usize].P =
                newValue as uint8_t; // FIXME magic numbers repeated in cli.c
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_P, newValue);
            current_block_82 = 15447629348493591490;
        }
        10 => {
            newValue =
                constrain((*pidProfile).pid[PID_YAW as libc::c_int as usize].I
                              as libc::c_int + delta, 0i32, 200i32);
            (*pidProfile).pid[PID_YAW as libc::c_int as usize].I =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_I, newValue);
            current_block_82 = 15447629348493591490;
        }
        11 => {
            newValue =
                constrain((*pidProfile).pid[PID_YAW as libc::c_int as usize].D
                              as libc::c_int + delta, 0i32, 200i32);
            (*pidProfile).pid[PID_YAW as libc::c_int as usize].D =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_D, newValue);
            current_block_82 = 15447629348493591490;
        }
        21 => {
            newValue =
                constrain((*controlRateConfig).rcRates[FD_YAW as libc::c_int
                                                           as usize] as
                              libc::c_int + delta, 0i32, 255i32);
            (*controlRateConfig).rcRates[FD_YAW as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RC_RATE_YAW,
                                               newValue);
            current_block_82 = 15447629348493591490;
        }
        22 | 30 => {
            newValue =
                constrain((*pidProfile).pid[PID_PITCH as libc::c_int as
                                                usize].F as libc::c_int +
                              delta, 0i32, 2000i32);
            (*pidProfile).pid[PID_PITCH as libc::c_int as usize].F =
                newValue as uint16_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_F, newValue);
            if adjustmentFunction as libc::c_int ==
                   ADJUSTMENT_PITCH_F as libc::c_int {
                current_block_82 = 15447629348493591490;
            } else { current_block_82 = 3887013524721631589; }
        }
        31 => { current_block_82 = 3887013524721631589; }
        32 => {
            newValue =
                constrain((*pidProfile).pid[PID_YAW as libc::c_int as usize].F
                              as libc::c_int + delta, 0i32, 2000i32);
            (*pidProfile).pid[PID_YAW as libc::c_int as usize].F =
                newValue as uint16_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_F, newValue);
            current_block_82 = 15447629348493591490;
        }
        23 => {
            newValue =
                constrain((*pidProfile).feedForwardTransition as libc::c_int +
                              delta, 1i32, 100i32);
            (*pidProfile).feedForwardTransition = newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_FEEDFORWARD_TRANSITION,
                                               newValue);
            current_block_82 = 15447629348493591490;
        }
        _ => { newValue = -1i32; current_block_82 = 15447629348493591490; }
    }
    match current_block_82 {
        3887013524721631589 =>
        // fall through for combined ADJUSTMENT_PITCH_ROLL_F
        {
            newValue =
                constrain((*pidProfile).pid[PID_ROLL as libc::c_int as
                                                usize].F as libc::c_int +
                              delta, 0i32,
                          2000i32); // FIXME magic numbers repeated in cli.c
            (*pidProfile).pid[PID_ROLL as libc::c_int as usize].F =
                newValue as uint16_t; // FIXME magic numbers repeated in cli.c
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_F,
                                               newValue); // FIXME magic numbers repeated in cli.c
        }
        3185416327625189411 => {
            newValue =
                constrain((*pidProfile).pid[PID_ROLL as libc::c_int as
                                                usize].D as libc::c_int +
                              delta, 0i32, 200i32);
            (*pidProfile).pid[PID_ROLL as libc::c_int as usize].D =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_D, newValue);
        }
        14549389285293717636 => {
            newValue =
                constrain((*pidProfile).pid[PID_ROLL as libc::c_int as
                                                usize].I as libc::c_int +
                              delta, 0i32, 200i32);
            (*pidProfile).pid[PID_ROLL as libc::c_int as usize].I =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_I, newValue);
        }
        10622320711346489414 => {
            newValue =
                constrain((*pidProfile).pid[PID_ROLL as libc::c_int as
                                                usize].P as libc::c_int +
                              delta, 0i32, 200i32);
            (*pidProfile).pid[PID_ROLL as libc::c_int as usize].P =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_P, newValue);
        }
        2128805812479287575 =>
        // fall through for combined ADJUSTMENT_PITCH_ROLL_RATE
        {
            newValue =
                constrain((*controlRateConfig).rates[FD_ROLL as libc::c_int as
                                                         usize] as libc::c_int
                              + delta, 0i32, 255i32);
            (*controlRateConfig).rates[FD_ROLL as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RATE,
                                               newValue);
        }
        8697802668005419385 =>
        // fall through for combined ADJUSTMENT_RC_EXPO
        {
            newValue =
                constrain((*controlRateConfig).rcExpo[FD_PITCH as libc::c_int
                                                          as usize] as
                              libc::c_int + delta, 0i32, 100i32);
            (*controlRateConfig).rcExpo[FD_PITCH as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_EXPO,
                                               newValue);
        }
        15932841215234547859 =>
        // fall through for combined ADJUSTMENT_RC_EXPO
        {
            newValue =
                constrain((*controlRateConfig).rcRates[FD_PITCH as libc::c_int
                                                           as usize] as
                              libc::c_int + delta, 0i32,
                          255i32); // FIXME magic numbers repeated in cli.c
            (*controlRateConfig).rcRates[FD_PITCH as libc::c_int as usize] =
                newValue as uint8_t; // FIXME magic numbers repeated in cli.c
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_RATE,
                                               newValue);
        }
        _ => { }
    }
    return newValue;
}
unsafe extern "C" fn applyAbsoluteAdjustment(mut controlRateConfig:
                                                 *mut controlRateConfig_t,
                                             mut adjustmentFunction:
                                                 adjustmentFunction_e,
                                             mut value: libc::c_int)
 -> libc::c_int {
    let mut newValue: libc::c_int = 0;
    if controlRateConfig.is_null() || pidProfile.is_null() { return 0i32 }
    let mut current_block_84: u64;
    match adjustmentFunction as libc::c_uint {
        1 | 25 => {
            newValue = constrain(value, 0i32, 255i32);
            (*controlRateConfig).rcRates[FD_ROLL as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_RATE,
                                               newValue);
            if adjustmentFunction as libc::c_uint ==
                   ADJUSTMENT_ROLL_RC_RATE as libc::c_int as libc::c_uint {
                current_block_84 = 5265702136860997526;
            } else { current_block_84 = 67936954847564801; }
        }
        26 => { current_block_84 = 67936954847564801; }
        2 | 27 => {
            newValue = constrain(value, 0i32, 100i32);
            (*controlRateConfig).rcExpo[FD_ROLL as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_EXPO,
                                               newValue);
            if adjustmentFunction as libc::c_uint ==
                   ADJUSTMENT_ROLL_RC_EXPO as libc::c_int as libc::c_uint {
                current_block_84 = 5265702136860997526;
            } else { current_block_84 = 13474875856152572999; }
        }
        28 => { current_block_84 = 13474875856152572999; }
        3 => {
            newValue = constrain(value, 0i32, 100i32);
            (*controlRateConfig).thrExpo8 = newValue as uint8_t;
            initRcProcessing();
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_THROTTLE_EXPO,
                                               newValue);
            current_block_84 = 5265702136860997526;
        }
        4 | 13 => {
            newValue = constrain(value, 0i32, 255i32);
            (*controlRateConfig).rates[FD_PITCH as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RATE,
                                               newValue);
            if adjustmentFunction as libc::c_uint ==
                   ADJUSTMENT_PITCH_RATE as libc::c_int as libc::c_uint {
                current_block_84 = 5265702136860997526;
            } else { current_block_84 = 10434922677118397198; }
        }
        14 => { current_block_84 = 10434922677118397198; }
        5 => {
            newValue = constrain(value, 0i32, 255i32);
            (*controlRateConfig).rates[FD_YAW as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_RATE, newValue);
            current_block_84 = 5265702136860997526;
        }
        6 | 15 => {
            newValue = constrain(value, 0i32, 200i32);
            (*pidProfile).pid[PID_PITCH as libc::c_int as usize].P =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_P, newValue);
            if adjustmentFunction as libc::c_uint ==
                   ADJUSTMENT_PITCH_P as libc::c_int as libc::c_uint {
                current_block_84 = 5265702136860997526;
            } else {
                // fall through for combined ADJUSTMENT_PITCH_ROLL_P
                current_block_84 =
                    17331341238949182750; // FIXME magic numbers repeated in cli.c
            }
        }
        18 => { current_block_84 = 17331341238949182750; }
        7 | 16 => {
            newValue = constrain(value, 0i32, 200i32);
            (*pidProfile).pid[PID_PITCH as libc::c_int as usize].I =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_I, newValue);
            if adjustmentFunction as libc::c_uint ==
                   ADJUSTMENT_PITCH_I as libc::c_int as libc::c_uint {
                current_block_84 = 5265702136860997526;
            } else {
                // fall through for combined ADJUSTMENT_PITCH_ROLL_I
                current_block_84 =
                    10290786027229383966; // FIXME magic numbers repeated in cli.c
            }
        }
        19 => { current_block_84 = 10290786027229383966; }
        8 | 17 => {
            newValue = constrain(value, 0i32, 200i32);
            (*pidProfile).pid[PID_PITCH as libc::c_int as usize].D =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_D, newValue);
            if adjustmentFunction as libc::c_uint ==
                   ADJUSTMENT_PITCH_D as libc::c_int as libc::c_uint {
                current_block_84 = 5265702136860997526;
            } else {
                // fall through for combined ADJUSTMENT_PITCH_ROLL_D
                current_block_84 =
                    16756648847763220407; // FIXME magic numbers repeated in cli.c
            }
        }
        20 => {
            current_block_84 =
                16756648847763220407; // FIXME magic numbers repeated in cli.c
        }
        9 => {
            newValue =
                constrain(value, 0i32,
                          200i32); // FIXME magic numbers repeated in cli.c
            (*pidProfile).pid[PID_YAW as libc::c_int as usize].P =
                newValue as uint8_t; // FIXME magic numbers repeated in cli.c
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_P, newValue);
            current_block_84 = 5265702136860997526;
        }
        10 => {
            newValue = constrain(value, 0i32, 200i32);
            (*pidProfile).pid[PID_YAW as libc::c_int as usize].I =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_I, newValue);
            current_block_84 = 5265702136860997526;
        }
        11 => {
            newValue = constrain(value, 0i32, 200i32);
            (*pidProfile).pid[PID_YAW as libc::c_int as usize].D =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_D, newValue);
            current_block_84 = 5265702136860997526;
        }
        21 => {
            newValue = constrain(value, 0i32, 255i32);
            (*controlRateConfig).rcRates[FD_YAW as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RC_RATE_YAW,
                                               newValue);
            current_block_84 = 5265702136860997526;
        }
        22 | 30 => {
            newValue = constrain(value, 0i32, 2000i32);
            (*pidProfile).pid[PID_PITCH as libc::c_int as usize].F =
                newValue as uint16_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_F, newValue);
            if adjustmentFunction as libc::c_uint ==
                   ADJUSTMENT_PITCH_F as libc::c_int as libc::c_uint {
                current_block_84 = 5265702136860997526;
            } else { current_block_84 = 3750153672556407072; }
        }
        31 => { current_block_84 = 3750153672556407072; }
        32 => {
            newValue = constrain(value, 0i32, 2000i32);
            (*pidProfile).pid[PID_YAW as libc::c_int as usize].F =
                newValue as uint16_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_F, newValue);
            current_block_84 = 5265702136860997526;
        }
        23 => {
            newValue = constrain(value, 1i32, 100i32);
            (*pidProfile).feedForwardTransition = newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_FEEDFORWARD_TRANSITION,
                                               newValue);
            current_block_84 = 5265702136860997526;
        }
        _ => { newValue = -1i32; current_block_84 = 5265702136860997526; }
    }
    match current_block_84 {
        3750153672556407072 =>
        // fall through for combined ADJUSTMENT_PITCH_ROLL_F
        {
            newValue =
                constrain(value, 0i32,
                          2000i32); // FIXME magic numbers repeated in cli.c
            (*pidProfile).pid[PID_ROLL as libc::c_int as usize].F =
                newValue as uint16_t; // FIXME magic numbers repeated in cli.c
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_F,
                                               newValue); // FIXME magic numbers repeated in cli.c
        }
        16756648847763220407 => {
            newValue = constrain(value, 0i32, 200i32);
            (*pidProfile).pid[PID_ROLL as libc::c_int as usize].D =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_D, newValue);
        }
        10290786027229383966 => {
            newValue = constrain(value, 0i32, 200i32);
            (*pidProfile).pid[PID_ROLL as libc::c_int as usize].I =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_I, newValue);
        }
        17331341238949182750 => {
            newValue = constrain(value, 0i32, 200i32);
            (*pidProfile).pid[PID_ROLL as libc::c_int as usize].P =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_P, newValue);
        }
        10434922677118397198 =>
        // fall through for combined ADJUSTMENT_PITCH_ROLL_RATE
        {
            newValue = constrain(value, 0i32, 255i32);
            (*controlRateConfig).rates[FD_ROLL as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RATE,
                                               newValue);
        }
        13474875856152572999 =>
        // fall through for combined ADJUSTMENT_RC_EXPO
        {
            newValue = constrain(value, 0i32, 100i32);
            (*controlRateConfig).rcExpo[FD_PITCH as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_EXPO,
                                               newValue);
        }
        67936954847564801 =>
        // fall through for combined ADJUSTMENT_RC_EXPO
        {
            newValue =
                constrain(value, 0i32,
                          255i32); // FIXME magic numbers repeated in serial_cli.c
            (*controlRateConfig).rcRates[FD_PITCH as libc::c_int as usize] =
                newValue as uint8_t;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_RATE,
                                               newValue);
        }
        _ => { }
    }
    return newValue;
}
unsafe extern "C" fn applySelectAdjustment(mut adjustmentFunction:
                                               adjustmentFunction_e,
                                           mut position: uint8_t) -> uint8_t {
    let mut beeps: uint8_t = 0i32 as uint8_t;
    match adjustmentFunction as libc::c_uint {
        12 => {
            if getCurrentControlRateProfileIndex() as libc::c_int !=
                   position as libc::c_int {
                changeControlRateProfile(position);
                blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RATE_PROFILE,
                                                   position as int32_t);
                beeps = (position as libc::c_int + 1i32) as uint8_t
            }
        }
        24 => {
            let mut newValue: uint8_t =
                constrain(position as libc::c_int, 0i32, 200i32) as uint8_t;
            if (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].D as
                   libc::c_int != newValue as libc::c_int {
                beeps =
                    ((newValue as libc::c_int -
                          (*pidProfile).pid[PID_LEVEL as libc::c_int as
                                                usize].D as libc::c_int) /
                         8i32 + 1i32) as uint8_t;
                (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].D =
                    newValue;
                blackboxLogInflightAdjustmentEvent(ADJUSTMENT_HORIZON_STRENGTH,
                                                   position as int32_t);
            }
        }
        29 | _ => { }
    }
    if beeps != 0 { beeperConfirmationBeeps(beeps); }
    return position;
}
#[no_mangle]
pub unsafe extern "C" fn processRcAdjustments(mut controlRateConfig:
                                                  *mut controlRateConfig_t) {
    let now: uint32_t = millis();
    let mut newValue: libc::c_int = -1i32;
    let canUseRxData: bool = rxIsReceivingSignal();
    let mut current_block_19: u64;
    // Process Increment/Decrement adjustments
    let mut adjustmentIndex: libc::c_int = 0i32;
    while adjustmentIndex < 4i32 {
        let mut adjustmentState: *mut adjustmentState_t =
            &mut *adjustmentStates.as_mut_ptr().offset(adjustmentIndex as
                                                           isize) as
                *mut adjustmentState_t;
        if !(*adjustmentState).config.is_null() {
            let adjustmentFunction: adjustmentFunction_e =
                (*(*adjustmentState).config).adjustmentFunction;
            if !(adjustmentFunction as libc::c_uint ==
                     ADJUSTMENT_NONE as libc::c_int as libc::c_uint) {
                if cmp32(now, (*adjustmentState).timeoutAt) >= 0i32 {
                    (*adjustmentState).timeoutAt =
                        now.wrapping_add((1000i32 / 2i32) as libc::c_uint);
                    adjustmentStateMask =
                        (adjustmentStateMask as libc::c_int &
                             !(1i32 << adjustmentIndex)) as uint8_t
                }
                if canUseRxData {
                    let channelIndex: uint8_t =
                        (4i32 +
                             (*adjustmentState).auxChannelIndex as
                                 libc::c_int) as uint8_t;
                    if (*(*adjustmentState).config).mode as libc::c_uint ==
                           ADJUSTMENT_MODE_STEP as libc::c_int as libc::c_uint
                       {
                        let mut delta: libc::c_int = 0;
                        if rcData[channelIndex as usize] as libc::c_int >
                               (*rxConfig()).midrc as libc::c_int + 200i32 {
                            delta =
                                (*(*adjustmentState).config).data.step as
                                    libc::c_int;
                            current_block_19 = 11298138898191919651;
                        } else if (rcData[channelIndex as usize] as
                                       libc::c_int) <
                                      (*rxConfig()).midrc as libc::c_int -
                                          200i32 {
                            delta =
                                -((*(*adjustmentState).config).data.step as
                                      libc::c_int);
                            current_block_19 = 11298138898191919651;
                        } else {
                            // returning the switch to the middle immediately resets the ready state
                            adjustmentStateMask =
                                (adjustmentStateMask as libc::c_int &
                                     !(1i32 << adjustmentIndex)) as uint8_t;
                            (*adjustmentState).timeoutAt =
                                now.wrapping_add((1000i32 / 2i32) as
                                                     libc::c_uint);
                            current_block_19 = 14916268686031723178;
                        }
                        match current_block_19 {
                            14916268686031723178 => { }
                            _ => {
                                if adjustmentStateMask as libc::c_int &
                                       1i32 << adjustmentIndex != 0 {
                                    current_block_19 = 14916268686031723178;
                                } else {
                                    newValue =
                                        applyStepAdjustment(controlRateConfig,
                                                            adjustmentFunction
                                                                as uint8_t,
                                                            delta);
                                    pidInitConfig(pidProfile);
                                    current_block_19 = 17184638872671510253;
                                }
                            }
                        }
                    } else {
                        if (*(*adjustmentState).config).mode as libc::c_uint
                               ==
                               ADJUSTMENT_MODE_SELECT as libc::c_int as
                                   libc::c_uint {
                            let mut switchPositions: libc::c_int =
                                (*(*adjustmentState).config).data.switchPositions
                                    as libc::c_int;
                            if adjustmentFunction as libc::c_uint ==
                                   ADJUSTMENT_RATE_PROFILE as libc::c_int as
                                       libc::c_uint &&
                                   (*systemConfig()).rateProfile6PosSwitch as
                                       libc::c_int != 0 {
                                switchPositions = 6i32
                            }
                            let rangeWidth: uint16_t =
                                ((2100i32 - 900i32) / switchPositions) as
                                    uint16_t;
                            let position: uint8_t =
                                ((constrain(rcData[channelIndex as usize] as
                                                libc::c_int, 900i32,
                                            2100i32 - 1i32) - 900i32) /
                                     rangeWidth as libc::c_int) as uint8_t;
                            newValue =
                                applySelectAdjustment(adjustmentFunction,
                                                      position) as libc::c_int
                        }
                        current_block_19 = 17184638872671510253;
                    }
                    match current_block_19 {
                        14916268686031723178 => { }
                        _ => {
                            adjustmentStateMask =
                                (adjustmentStateMask as libc::c_int |
                                     1i32 << adjustmentIndex) as uint8_t
                        }
                    }
                }
            }
        }
        adjustmentIndex += 1
    }
    // Process Absolute adjustments
    let mut index: libc::c_int = 0i32;
    while index < 15i32 {
        static mut lastRcData: [int16_t; 15] =
            [0i32 as int16_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let adjustmentRange: *const adjustmentRange_t =
            adjustmentRanges(index);
        let channelIndex_0: uint8_t =
            (4i32 + (*adjustmentRange).auxSwitchChannelIndex as libc::c_int)
                as uint8_t;
        let mut adjustmentConfig: *const adjustmentConfig_t =
            &*defaultAdjustmentConfigs.as_ptr().offset(((*adjustmentRange).adjustmentFunction
                                                            as libc::c_int -
                                                            1i32) as isize) as
                *const adjustmentConfig_t;
        // If setting is defined for step adjustment and center value has been specified, apply values directly (scaled) from aux channel
        if rcData[channelIndex_0 as usize] as libc::c_int !=
               lastRcData[index as usize] as libc::c_int &&
               (*adjustmentRange).adjustmentCenter as libc::c_int != 0 &&
               (*adjustmentConfig).mode as libc::c_uint ==
                   ADJUSTMENT_MODE_STEP as libc::c_int as libc::c_uint &&
               isRangeActive((*adjustmentRange).auxChannelIndex,
                             &(*adjustmentRange).range) as libc::c_int != 0 {
            let mut value: libc::c_int =
                (rcData[channelIndex_0 as usize] as libc::c_int -
                     (1000i32 + (2000i32 - 1000i32) / 2i32)) *
                    (*adjustmentRange).adjustmentScale as libc::c_int /
                    (1000i32 + (2000i32 - 1000i32) / 2i32 - 1000i32) +
                    (*adjustmentRange).adjustmentCenter as libc::c_int;
            lastRcData[index as usize] = rcData[channelIndex_0 as usize];
            applyAbsoluteAdjustment(controlRateConfig,
                                    (*adjustmentRange).adjustmentFunction as
                                        adjustmentFunction_e, value);
            pidInitConfig(pidProfile);
        }
        index += 1
    };
}
// enough for 4 x 3position switches / 4 aux channel
#[no_mangle]
pub unsafe extern "C" fn resetAdjustmentStates() {
    memset(adjustmentStates.as_mut_ptr() as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<[adjustmentState_t; 4]>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn updateAdjustmentStates() {
    let mut index: libc::c_int = 0i32;
    while index < 15i32 {
        let adjustmentRange: *const adjustmentRange_t =
            adjustmentRanges(index);
        // Only use slots if center value has not been specified, otherwise apply values directly (scaled) from aux channel
        if isRangeActive((*adjustmentRange).auxChannelIndex,
                         &(*adjustmentRange).range) as libc::c_int != 0 &&
               (*adjustmentRange).adjustmentCenter as libc::c_int == 0i32 {
            let mut adjustmentConfig: *const adjustmentConfig_t =
                &*defaultAdjustmentConfigs.as_ptr().offset(((*adjustmentRange).adjustmentFunction
                                                                as libc::c_int
                                                                - 1i32) as
                                                               isize) as
                    *const adjustmentConfig_t;
            configureAdjustment((*adjustmentRange).adjustmentIndex,
                                (*adjustmentRange).auxSwitchChannelIndex,
                                adjustmentConfig);
        }
        index += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn useAdjustmentConfig(mut pidProfileToUse:
                                                 *mut pidProfile_t) {
    pidProfile = pidProfileToUse;
}
unsafe extern "C" fn run_static_initializers() {
    adjustmentRanges_Registry =
        {
            let mut init =
                pgRegistry_s{pgn: (37i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 ((::core::mem::size_of::<adjustmentRange_t>()
                                       as
                                       libc::c_ulong).wrapping_mul(15i32 as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut adjustmentRanges_SystemArray as
                                     *mut [adjustmentRange_t; 15] as
                                     *mut uint8_t,
                             copy:
                                 &mut adjustmentRanges_CopyArray as
                                     *mut [adjustmentRange_t; 15] as
                                     *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     0 as
                                                         *mut libc::c_void,},};
            init
        };
    defaultAdjustmentConfigs =
        [{
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_RC_RATE,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_RC_EXPO,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_THROTTLE_EXPO,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_PITCH_ROLL_RATE,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_YAW_RATE,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_PITCH_ROLL_P,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_PITCH_ROLL_I,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_PITCH_ROLL_D,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_YAW_P,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_YAW_I,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_YAW_D,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_RATE_PROFILE,
                                    mode: ADJUSTMENT_MODE_SELECT,
                                    data:
                                        adjustmentConfig_u{switchPositions:
                                                               3i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_PITCH_RATE,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_ROLL_RATE,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_PITCH_P,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_PITCH_I,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_PITCH_D,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_ROLL_P,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_ROLL_I,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_ROLL_D,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_RC_RATE_YAW,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_PITCH_ROLL_F,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_FEEDFORWARD_TRANSITION,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction:
                                        ADJUSTMENT_HORIZON_STRENGTH,
                                    mode: ADJUSTMENT_MODE_SELECT,
                                    data:
                                        adjustmentConfig_u{switchPositions:
                                                               255i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_PID_AUDIO,
                                    mode: ADJUSTMENT_MODE_SELECT,
                                    data:
                                        adjustmentConfig_u{switchPositions:
                                                               (::core::mem::size_of::<[uint8_t; 7]>()
                                                                    as
                                                                    libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                                                    as
                                                                                                    libc::c_ulong)
                                                                   as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_PITCH_F,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_ROLL_F,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         {
             let mut init =
                 adjustmentConfig_s{adjustmentFunction: ADJUSTMENT_YAW_F,
                                    mode: ADJUSTMENT_MODE_STEP,
                                    data:
                                        adjustmentConfig_u{step:
                                                               1i32 as
                                                                   uint8_t,},};
             init
         },
         adjustmentConfig_t{adjustmentFunction: ADJUSTMENT_NONE,
                            mode: ADJUSTMENT_MODE_STEP,
                            data: adjustmentConfig_u{step: 0,},},
         adjustmentConfig_t{adjustmentFunction: ADJUSTMENT_NONE,
                            mode: ADJUSTMENT_MODE_STEP,
                            data: adjustmentConfig_u{step: 0,},},
         adjustmentConfig_t{adjustmentFunction: ADJUSTMENT_NONE,
                            mode: ADJUSTMENT_MODE_STEP,
                            data: adjustmentConfig_u{step: 0,},},
         adjustmentConfig_t{adjustmentFunction: ADJUSTMENT_NONE,
                            mode: ADJUSTMENT_MODE_STEP,
                            data: adjustmentConfig_u{step: 0,},}]
}
#[used]
#[cfg_attr ( target_os = "linux", link_section = ".init_array" )]
#[cfg_attr ( target_os = "windows", link_section = ".CRT$XIB" )]
#[cfg_attr ( target_os = "macos", link_section = "__DATA,__mod_init_func" )]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
