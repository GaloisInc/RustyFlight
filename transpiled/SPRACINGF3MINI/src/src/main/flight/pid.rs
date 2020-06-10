use ::libc;
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
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn fabsf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
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
    fn fapplyDeadband(value: libc::c_float, deadband: libc::c_float)
     -> libc::c_float;
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
    fn biquadFilterUpdateLPF(filter: *mut biquadFilter_t,
                             filterFreq: libc::c_float,
                             refreshRate: uint32_t);
    #[no_mangle]
    fn biquadFilterApplyDF1(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn biquadFilterApply(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn filterGetNotchQ(centerFreq: libc::c_float, cutoffFreq: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn pt1FilterGain(f_cut: uint16_t, dT_0: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn pt1FilterInit(filter: *mut pt1Filter_t, k: libc::c_float);
    #[no_mangle]
    fn pt1FilterUpdateCutoff(filter: *mut pt1Filter_t, k: libc::c_float);
    #[no_mangle]
    fn pt1FilterApply(filter: *mut pt1Filter_t, input: libc::c_float)
     -> libc::c_float;
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
    fn systemBeep(on: bool);
    #[no_mangle]
    fn getSetpointRate(axis: libc::c_int) -> libc::c_float;
    #[no_mangle]
    fn getRcDeflection(axis: libc::c_int) -> libc::c_float;
    #[no_mangle]
    fn getRcDeflectionAbs(axis: libc::c_int) -> libc::c_float;
    #[no_mangle]
    fn getThrottlePIDAttenuation() -> libc::c_float;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn getMotorMixRange() -> libc::c_float;
    #[no_mangle]
    fn mixerIsOutputSaturated(axis: libc::c_int, errorRate: libc::c_float)
     -> bool;
    #[no_mangle]
    static mut gyro: gyro_t;
    #[no_mangle]
    fn gyroYawSpinDetected() -> bool;
    #[no_mangle]
    fn gyroOverflowDetected() -> bool;
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
pub const FD_YAW: C2RustUnnamed_0 = 2;
pub const FD_PITCH: C2RustUnnamed_0 = 1;
pub const FD_ROLL: C2RustUnnamed_0 = 0;
pub type angle_index_t = libc::c_uint;
pub const AI_PITCH: angle_index_t = 1;
pub const AI_ROLL: angle_index_t = 0;
pub type filter_t = filter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pt1Filter_s {
    pub state: libc::c_float,
    pub k: libc::c_float,
}
pub type pt1Filter_t = pt1Filter_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_3 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
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
// Type of accelerometer used/detected
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
pub type C2RustUnnamed_4 = libc::c_uint;
pub const RC_SMOOTHING_DERIVATIVE_BIQUAD: C2RustUnnamed_4 = 2;
pub const RC_SMOOTHING_DERIVATIVE_PT1: C2RustUnnamed_4 = 1;
pub const RC_SMOOTHING_DERIVATIVE_OFF: C2RustUnnamed_4 = 0;
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
pub type C2RustUnnamed_5 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_5 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_5 = 2;
pub const ARMED: C2RustUnnamed_5 = 1;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_6 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_6 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_6 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_6 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_6 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_6 = 16;
pub const BARO_MODE: C2RustUnnamed_6 = 8;
pub const MAG_MODE: C2RustUnnamed_6 = 4;
pub const HORIZON_MODE: C2RustUnnamed_6 = 2;
pub const ANGLE_MODE: C2RustUnnamed_6 = 1;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_7 = 5;
pub const PID_MAG: C2RustUnnamed_7 = 4;
pub const PID_LEVEL: C2RustUnnamed_7 = 3;
pub const PID_YAW: C2RustUnnamed_7 = 2;
pub const PID_PITCH: C2RustUnnamed_7 = 1;
pub const PID_ROLL: C2RustUnnamed_7 = 0;
pub type pidStabilisationState_e = libc::c_uint;
pub const PID_STABILISATION_ON: pidStabilisationState_e = 1;
pub const PID_STABILISATION_OFF: pidStabilisationState_e = 0;
pub type pidCrashRecovery_e = libc::c_uint;
pub const PID_CRASH_RECOVERY_BEEP: pidCrashRecovery_e = 2;
pub const PID_CRASH_RECOVERY_ON: pidCrashRecovery_e = 1;
pub const PID_CRASH_RECOVERY_OFF: pidCrashRecovery_e = 0;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const ANTI_GRAVITY_STEP: C2RustUnnamed_8 = 1;
pub const ANTI_GRAVITY_SMOOTH: C2RustUnnamed_8 = 0;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const ITERM_RELAX_RPY_INC: C2RustUnnamed_9 = 4;
pub const ITERM_RELAX_RP_INC: C2RustUnnamed_9 = 3;
pub const ITERM_RELAX_RPY: C2RustUnnamed_9 = 2;
pub const ITERM_RELAX_RP: C2RustUnnamed_9 = 1;
pub const ITERM_RELAX_OFF: C2RustUnnamed_9 = 0;
pub type C2RustUnnamed_10 = libc::c_uint;
pub const ITERM_RELAX_SETPOINT: C2RustUnnamed_10 = 1;
pub const ITERM_RELAX_GYRO: C2RustUnnamed_10 = 0;
pub type pidProfile_t = pidProfile_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
pub type pidConfig_t = pidConfig_s;
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
pub type pidAxisData_t = pidAxisData_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidAxisData_s {
    pub P: libc::c_float,
    pub I: libc::c_float,
    pub D: libc::c_float,
    pub F: libc::c_float,
    pub Sum: libc::c_float,
}
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
// Processing denominator for PID controller vs gyro sampling rate
// off, on - enables pidsum runaway disarm logic
// delay in ms for "in-flight" conditions before deactivation (successful flight)
// minimum throttle percent required during deactivation phase
// USE_RC_SMOOTHING_FILTER
pub type pidCoefficient_t = pidCoefficient_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidCoefficient_s {
    pub Kp: libc::c_float,
    pub Ki: libc::c_float,
    pub Kd: libc::c_float,
    pub Kf: libc::c_float,
}
pub type gyro_t = gyro_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_11,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_11 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub const SENSOR_ACC: C2RustUnnamed_12 = 2;
pub type dtermLowpass_t = dtermLowpass_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union dtermLowpass_u {
    pub pt1Filter: pt1Filter_t,
    pub biquadFilter: biquadFilter_t,
}
pub type C2RustUnnamed_12 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_12 = 64;
pub const SENSOR_GPS: C2RustUnnamed_12 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_12 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_12 = 16;
pub const SENSOR_MAG: C2RustUnnamed_12 = 8;
pub const SENSOR_BARO: C2RustUnnamed_12 = 4;
pub const SENSOR_GYRO: C2RustUnnamed_12 = 1;
#[inline]
unsafe extern "C" fn constrainf(mut amt: libc::c_float,
                                mut low: libc::c_float,
                                mut high: libc::c_float) -> libc::c_float {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
#[inline]
unsafe extern "C" fn pidProfilesMutable(mut _index: libc::c_int)
 -> *mut pidProfile_t {
    return &mut *pidProfiles_SystemArray.as_mut_ptr().offset(_index as isize)
               as *mut pidProfile_t;
}
#[inline]
unsafe extern "C" fn pidConfig() -> *const pidConfig_t {
    return &mut pidConfig_System;
}
#[no_mangle]
pub static mut pidNames: [libc::c_char; 26] =
    [82, 79, 76, 76, 59, 80, 73, 84, 67, 72, 59, 89, 65, 87, 59, 76, 69, 86,
     69, 76, 59, 77, 65, 71, 59, 0];
#[no_mangle]
pub static mut targetPidLooptime: uint32_t = 0;
#[no_mangle]
pub static mut pidData: [pidAxisData_t; 3] =
    [pidAxisData_t{P: 0., I: 0., D: 0., F: 0., Sum: 0.,}; 3];
static mut pidStabilisationEnabled: bool = false;
static mut inCrashRecoveryMode: bool = 0 as libc::c_int != 0;
static mut dT: libc::c_float = 0.;
static mut pidFrequency: libc::c_float = 0.;
static mut antiGravityMode: uint8_t = 0;
static mut antiGravityThrottleHpf: libc::c_float = 0.;
static mut itermAcceleratorGain: uint16_t = 0;
static mut antiGravityOsdCutoff: libc::c_float = 1.0f32;
static mut antiGravityEnabled: bool = false;
#[no_mangle]
pub static mut pidConfig_System: pidConfig_t =
    pidConfig_t{pid_process_denom: 0,
                runaway_takeoff_prevention: 0,
                runaway_takeoff_deactivate_delay: 0,
                runaway_takeoff_deactivate_throttle: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut pidConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (504 as libc::c_int |
                                      (2 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<pidConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &pidConfig_System as *const pidConfig_t as
                                     *mut pidConfig_t as *mut uint8_t,
                             copy:
                                 &pidConfig_Copy as *const pidConfig_t as
                                     *mut pidConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_3{ptr:
                                                     &pgResetTemplate_pidConfig
                                                         as *const pidConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut pidConfig_Copy: pidConfig_t =
    pidConfig_t{pid_process_denom: 0,
                runaway_takeoff_prevention: 0,
                runaway_takeoff_deactivate_delay: 0,
                runaway_takeoff_deactivate_throttle: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_pidConfig: pidConfig_t =
    {
        let mut init =
            pidConfig_s{pid_process_denom: 2 as libc::c_int as uint8_t,
                        runaway_takeoff_prevention:
                            1 as libc::c_int as uint8_t,
                        runaway_takeoff_deactivate_delay:
                            500 as libc::c_int as uint16_t,
                        runaway_takeoff_deactivate_throttle:
                            25 as libc::c_int as uint8_t,};
        init
    };
// The anti gravity throttle highpass filter cutoff
#[no_mangle]
pub static mut pidProfiles_SystemArray: [pidProfile_t; 3] =
    [pidProfile_t{yaw_lowpass_hz: 0,
                  dterm_lowpass_hz: 0,
                  dterm_notch_hz: 0,
                  dterm_notch_cutoff: 0,
                  pid: [pidf_t{P: 0, I: 0, D: 0, F: 0,}; 5],
                  dterm_filter_type: 0,
                  itermWindupPointPercent: 0,
                  pidSumLimit: 0,
                  pidSumLimitYaw: 0,
                  pidAtMinThrottle: 0,
                  levelAngleLimit: 0,
                  horizon_tilt_effect: 0,
                  horizon_tilt_expert_mode: 0,
                  antiGravityMode: 0,
                  itermThrottleThreshold: 0,
                  itermAcceleratorGain: 0,
                  yawRateAccelLimit: 0,
                  rateAccelLimit: 0,
                  crash_dthreshold: 0,
                  crash_gthreshold: 0,
                  crash_setpoint_threshold: 0,
                  crash_time: 0,
                  crash_delay: 0,
                  crash_recovery_angle: 0,
                  crash_recovery_rate: 0,
                  vbatPidCompensation: 0,
                  feedForwardTransition: 0,
                  crash_limit_yaw: 0,
                  itermLimit: 0,
                  dterm_lowpass2_hz: 0,
                  crash_recovery: 0,
                  throttle_boost: 0,
                  throttle_boost_cutoff: 0,
                  iterm_rotation: 0,
                  smart_feedforward: 0,
                  iterm_relax_type: 0,
                  iterm_relax_cutoff: 0,
                  iterm_relax: 0,
                  acro_trainer_angle_limit: 0,
                  acro_trainer_debug_axis: 0,
                  acro_trainer_gain: 0,
                  acro_trainer_lookahead_ms: 0,
                  abs_control_gain: 0,
                  abs_control_limit: 0,
                  abs_control_error_limit: 0,}; 3];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut pidProfiles_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_3{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
#[no_mangle]
pub static mut pidProfiles_CopyArray: [pidProfile_t; 3] =
    [pidProfile_t{yaw_lowpass_hz: 0,
                  dterm_lowpass_hz: 0,
                  dterm_notch_hz: 0,
                  dterm_notch_cutoff: 0,
                  pid: [pidf_t{P: 0, I: 0, D: 0, F: 0,}; 5],
                  dterm_filter_type: 0,
                  itermWindupPointPercent: 0,
                  pidSumLimit: 0,
                  pidSumLimitYaw: 0,
                  pidAtMinThrottle: 0,
                  levelAngleLimit: 0,
                  horizon_tilt_effect: 0,
                  horizon_tilt_expert_mode: 0,
                  antiGravityMode: 0,
                  itermThrottleThreshold: 0,
                  itermAcceleratorGain: 0,
                  yawRateAccelLimit: 0,
                  rateAccelLimit: 0,
                  crash_dthreshold: 0,
                  crash_gthreshold: 0,
                  crash_setpoint_threshold: 0,
                  crash_time: 0,
                  crash_delay: 0,
                  crash_recovery_angle: 0,
                  crash_recovery_rate: 0,
                  vbatPidCompensation: 0,
                  feedForwardTransition: 0,
                  crash_limit_yaw: 0,
                  itermLimit: 0,
                  dterm_lowpass2_hz: 0,
                  crash_recovery: 0,
                  throttle_boost: 0,
                  throttle_boost_cutoff: 0,
                  iterm_rotation: 0,
                  smart_feedforward: 0,
                  iterm_relax_type: 0,
                  iterm_relax_cutoff: 0,
                  iterm_relax: 0,
                  acro_trainer_angle_limit: 0,
                  acro_trainer_debug_axis: 0,
                  acro_trainer_gain: 0,
                  acro_trainer_lookahead_ms: 0,
                  abs_control_gain: 0,
                  abs_control_limit: 0,
                  abs_control_error_limit: 0,}; 3];
#[no_mangle]
pub unsafe extern "C" fn resetPidProfile(mut pidProfile: *mut pidProfile_t) {
    static mut _reset_template_171: pidProfile_t =
        {
            let mut init =
                pidProfile_s{yaw_lowpass_hz: 0 as libc::c_int as uint16_t,
                             dterm_lowpass_hz: 100 as libc::c_int as uint16_t,
                             dterm_notch_hz: 0 as libc::c_int as uint16_t,
                             dterm_notch_cutoff:
                                 160 as libc::c_int as uint16_t,
                             pid:
                                 [{
                                      let mut init =
                                          pidf_s{P:
                                                     46 as libc::c_int as
                                                         uint8_t,
                                                 I:
                                                     45 as libc::c_int as
                                                         uint8_t,
                                                 D:
                                                     25 as libc::c_int as
                                                         uint8_t,
                                                 F:
                                                     60 as libc::c_int as
                                                         uint16_t,};
                                      init
                                  },
                                  {
                                      let mut init =
                                          pidf_s{P:
                                                     50 as libc::c_int as
                                                         uint8_t,
                                                 I:
                                                     50 as libc::c_int as
                                                         uint8_t,
                                                 D:
                                                     27 as libc::c_int as
                                                         uint8_t,
                                                 F:
                                                     60 as libc::c_int as
                                                         uint16_t,};
                                      init
                                  },
                                  {
                                      let mut init =
                                          pidf_s{P:
                                                     65 as libc::c_int as
                                                         uint8_t,
                                                 I:
                                                     45 as libc::c_int as
                                                         uint8_t,
                                                 D:
                                                     0 as libc::c_int as
                                                         uint8_t,
                                                 F:
                                                     60 as libc::c_int as
                                                         uint16_t,};
                                      init
                                  },
                                  {
                                      let mut init =
                                          pidf_s{P:
                                                     50 as libc::c_int as
                                                         uint8_t,
                                                 I:
                                                     50 as libc::c_int as
                                                         uint8_t,
                                                 D:
                                                     75 as libc::c_int as
                                                         uint8_t,
                                                 F:
                                                     0 as libc::c_int as
                                                         uint16_t,};
                                      init
                                  },
                                  {
                                      let mut init =
                                          pidf_s{P:
                                                     40 as libc::c_int as
                                                         uint8_t,
                                                 I:
                                                     0 as libc::c_int as
                                                         uint8_t,
                                                 D:
                                                     0 as libc::c_int as
                                                         uint8_t,
                                                 F:
                                                     0 as libc::c_int as
                                                         uint16_t,};
                                      init
                                  }],
                             dterm_filter_type:
                                 FILTER_PT1 as libc::c_int as uint8_t,
                             itermWindupPointPercent:
                                 40 as libc::c_int as uint8_t,
                             pidSumLimit: 500 as libc::c_int as uint16_t,
                             pidSumLimitYaw: 400 as libc::c_int as uint16_t,
                             pidAtMinThrottle:
                                 PID_STABILISATION_ON as libc::c_int as
                                     uint8_t,
                             levelAngleLimit: 55 as libc::c_int as uint8_t,
                             horizon_tilt_effect:
                                 75 as libc::c_int as uint8_t,
                             horizon_tilt_expert_mode:
                                 0 as libc::c_int as uint8_t,
                             antiGravityMode:
                                 ANTI_GRAVITY_SMOOTH as libc::c_int as
                                     uint8_t,
                             itermThrottleThreshold:
                                 250 as libc::c_int as uint16_t,
                             itermAcceleratorGain:
                                 5000 as libc::c_int as uint16_t,
                             yawRateAccelLimit:
                                 100 as libc::c_int as uint16_t,
                             rateAccelLimit: 0 as libc::c_int as uint16_t,
                             crash_dthreshold: 50 as libc::c_int as uint16_t,
                             crash_gthreshold: 400 as libc::c_int as uint16_t,
                             crash_setpoint_threshold:
                                 350 as libc::c_int as uint16_t,
                             crash_time: 500 as libc::c_int as uint16_t,
                             crash_delay: 0 as libc::c_int as uint16_t,
                             crash_recovery_angle:
                                 10 as libc::c_int as uint8_t,
                             crash_recovery_rate:
                                 100 as libc::c_int as uint8_t,
                             vbatPidCompensation: 0 as libc::c_int as uint8_t,
                             feedForwardTransition:
                                 0 as libc::c_int as uint8_t,
                             crash_limit_yaw: 200 as libc::c_int as uint16_t,
                             itermLimit: 150 as libc::c_int as uint16_t,
                             dterm_lowpass2_hz:
                                 200 as libc::c_int as uint16_t,
                             crash_recovery:
                                 PID_CRASH_RECOVERY_OFF as libc::c_int as
                                     uint8_t,
                             throttle_boost: 5 as libc::c_int as uint8_t,
                             throttle_boost_cutoff:
                                 15 as libc::c_int as uint8_t,
                             iterm_rotation: 1 as libc::c_int as uint8_t,
                             smart_feedforward: 0 as libc::c_int as uint8_t,
                             iterm_relax_type:
                                 ITERM_RELAX_GYRO as libc::c_int as uint8_t,
                             iterm_relax_cutoff: 11 as libc::c_int as uint8_t,
                             iterm_relax:
                                 ITERM_RELAX_OFF as libc::c_int as uint8_t,
                             acro_trainer_angle_limit:
                                 20 as libc::c_int as uint8_t,
                             acro_trainer_debug_axis:
                                 FD_ROLL as libc::c_int as uint8_t,
                             acro_trainer_gain: 75 as libc::c_int as uint8_t,
                             acro_trainer_lookahead_ms:
                                 50 as libc::c_int as uint16_t,
                             abs_control_gain: 0 as libc::c_int as uint8_t,
                             abs_control_limit: 90 as libc::c_int as uint8_t,
                             abs_control_error_limit:
                                 20 as libc::c_int as uint8_t,};
            init
        };
    memcpy(pidProfile as *mut libc::c_void,
           &_reset_template_171 as *const pidProfile_t as *const libc::c_void,
           ::core::mem::size_of::<pidProfile_t>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_pidProfiles(mut pidProfiles:
                                                   *mut pidProfile_t) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        resetPidProfile(&mut *pidProfiles.offset(i as isize));
        i += 1
    };
}
unsafe extern "C" fn pidSetTargetLooptime(mut pidLooptime: uint32_t) {
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime as libc::c_float * 1e-6f32;
    pidFrequency = 1.0f32 / dT;
}
static mut itermAccelerator: libc::c_float = 1.0f32;
#[no_mangle]
pub unsafe extern "C" fn pidSetItermAccelerator(mut newItermAccelerator:
                                                    libc::c_float) {
    itermAccelerator = newItermAccelerator;
}
#[no_mangle]
pub unsafe extern "C" fn pidOsdAntiGravityActive() -> bool {
    return itermAccelerator > antiGravityOsdCutoff;
}
#[no_mangle]
pub unsafe extern "C" fn pidStabilisationState(mut pidControllerState:
                                                   pidStabilisationState_e) {
    pidStabilisationEnabled =
        if pidControllerState as libc::c_uint ==
               PID_STABILISATION_ON as libc::c_int as libc::c_uint {
            1 as libc::c_int
        } else { 0 as libc::c_int } != 0;
}
#[no_mangle]
pub static mut rcAliasToAngleIndexMap: [angle_index_t; 2] =
    [AI_ROLL, AI_PITCH];
static mut dtermNotchApplyFn: filterApplyFnPtr = None;
static mut dtermNotch: [biquadFilter_t; 3] =
    [biquadFilter_t{b0: 0.,
                    b1: 0.,
                    b2: 0.,
                    a1: 0.,
                    a2: 0.,
                    x1: 0.,
                    x2: 0.,
                    y1: 0.,
                    y2: 0.,}; 3];
static mut dtermLowpassApplyFn: filterApplyFnPtr = None;
static mut dtermLowpass: [dtermLowpass_t; 3] =
    [dtermLowpass_u{pt1Filter: pt1Filter_t{state: 0., k: 0.,},}; 3];
static mut dtermLowpass2ApplyFn: filterApplyFnPtr = None;
static mut dtermLowpass2: [pt1Filter_t; 3] =
    [pt1Filter_t{state: 0., k: 0.,}; 3];
static mut ptermYawLowpassApplyFn: filterApplyFnPtr = None;
static mut ptermYawLowpass: pt1Filter_t = pt1Filter_t{state: 0., k: 0.,};
static mut windupLpf: [pt1Filter_t; 3] = [pt1Filter_t{state: 0., k: 0.,}; 3];
static mut itermRelax: uint8_t = 0;
static mut itermRelaxType: uint8_t = 0;
static mut itermRelaxCutoff: uint8_t = 0;
static mut setpointDerivativePt1: [pt1Filter_t; 3] =
    [pt1Filter_t{state: 0., k: 0.,}; 3];
static mut setpointDerivativeBiquad: [biquadFilter_t; 3] =
    [biquadFilter_t{b0: 0.,
                    b1: 0.,
                    b2: 0.,
                    a1: 0.,
                    a2: 0.,
                    x1: 0.,
                    x2: 0.,
                    y1: 0.,
                    y2: 0.,}; 3];
static mut setpointDerivativeLpfInitialized: bool = false;
static mut rcSmoothingDebugAxis: uint8_t = 0;
static mut rcSmoothingFilterType: uint8_t = 0;
// USE_RC_SMOOTHING_FILTER
static mut antiGravityThrottleLpf: pt1Filter_t =
    pt1Filter_t{state: 0., k: 0.,};
#[no_mangle]
pub unsafe extern "C" fn pidInitFilters(mut pidProfile: *const pidProfile_t) {
    // ensure yaw axis is 2
    if targetPidLooptime == 0 as libc::c_int as libc::c_uint {
        // no looptime set, so set all the filters to null
        dtermNotchApplyFn =
            Some(nullFilterApply as
                     unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                         -> libc::c_float); // No rounding needed
        dtermLowpassApplyFn =
            Some(nullFilterApply as
                     unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                         -> libc::c_float);
        ptermYawLowpassApplyFn =
            Some(nullFilterApply as
                     unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                         -> libc::c_float);
        return
    }
    let pidFrequencyNyquist: uint32_t =
        (pidFrequency / 2 as libc::c_int as libc::c_float) as uint32_t;
    let mut dTermNotchHz: uint16_t = 0;
    if (*pidProfile).dterm_notch_hz as libc::c_uint <= pidFrequencyNyquist {
        dTermNotchHz = (*pidProfile).dterm_notch_hz
    } else if ((*pidProfile).dterm_notch_cutoff as libc::c_uint) <
                  pidFrequencyNyquist {
        dTermNotchHz = pidFrequencyNyquist as uint16_t
    } else { dTermNotchHz = 0 as libc::c_int as uint16_t }
    if dTermNotchHz as libc::c_int != 0 as libc::c_int &&
           (*pidProfile).dterm_notch_cutoff as libc::c_int != 0 as libc::c_int
       {
        dtermNotchApplyFn =
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
            filterGetNotchQ(dTermNotchHz as libc::c_float,
                            (*pidProfile).dterm_notch_cutoff as
                                libc::c_float);
        let mut axis: libc::c_int = FD_ROLL as libc::c_int;
        while axis <= FD_YAW as libc::c_int {
            biquadFilterInit(&mut *dtermNotch.as_mut_ptr().offset(axis as
                                                                      isize),
                             dTermNotchHz as libc::c_float, targetPidLooptime,
                             notchQ, FILTER_NOTCH);
            axis += 1
        }
    } else {
        dtermNotchApplyFn =
            Some(nullFilterApply as
                     unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                         -> libc::c_float)
    }
    //2nd Dterm Lowpass Filter
    if (*pidProfile).dterm_lowpass2_hz as libc::c_int == 0 as libc::c_int ||
           (*pidProfile).dterm_lowpass2_hz as libc::c_uint >
               pidFrequencyNyquist {
        dtermLowpass2ApplyFn =
            Some(nullFilterApply as
                     unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                         -> libc::c_float)
    } else {
        dtermLowpass2ApplyFn =
            ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                     *mut pt1Filter_t,
                                                                 _:
                                                                     libc::c_float)
                                                -> libc::c_float>,
                                     filterApplyFnPtr>(Some(pt1FilterApply as
                                                                unsafe extern "C" fn(_:
                                                                                         *mut pt1Filter_t,
                                                                                     _:
                                                                                         libc::c_float)
                                                                    ->
                                                                        libc::c_float));
        let mut axis_0: libc::c_int = FD_ROLL as libc::c_int;
        while axis_0 <= FD_YAW as libc::c_int {
            pt1FilterInit(&mut *dtermLowpass2.as_mut_ptr().offset(axis_0 as
                                                                      isize),
                          pt1FilterGain((*pidProfile).dterm_lowpass2_hz, dT));
            axis_0 += 1
        }
    }
    if (*pidProfile).dterm_lowpass_hz as libc::c_int == 0 as libc::c_int ||
           (*pidProfile).dterm_lowpass_hz as libc::c_uint >
               pidFrequencyNyquist {
        dtermLowpassApplyFn =
            Some(nullFilterApply as
                     unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                         -> libc::c_float)
    } else {
        match (*pidProfile).dterm_filter_type as libc::c_int {
            0 => {
                dtermLowpassApplyFn =
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
                let mut axis_1: libc::c_int = FD_ROLL as libc::c_int;
                while axis_1 <= FD_YAW as libc::c_int {
                    pt1FilterInit(&mut (*dtermLowpass.as_mut_ptr().offset(axis_1
                                                                              as
                                                                              isize)).pt1Filter,
                                  pt1FilterGain((*pidProfile).dterm_lowpass_hz,
                                                dT));
                    axis_1 += 1
                }
            }
            1 => {
                dtermLowpassApplyFn =
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
                let mut axis_2: libc::c_int = FD_ROLL as libc::c_int;
                while axis_2 <= FD_YAW as libc::c_int {
                    biquadFilterInitLPF(&mut (*dtermLowpass.as_mut_ptr().offset(axis_2
                                                                                    as
                                                                                    isize)).biquadFilter,
                                        (*pidProfile).dterm_lowpass_hz as
                                            libc::c_float, targetPidLooptime);
                    axis_2 += 1
                }
            }
            _ => {
                dtermLowpassApplyFn =
                    Some(nullFilterApply as
                             unsafe extern "C" fn(_: *mut filter_t,
                                                  _: libc::c_float)
                                 -> libc::c_float)
            }
        }
    }
    if (*pidProfile).yaw_lowpass_hz as libc::c_int == 0 as libc::c_int ||
           (*pidProfile).yaw_lowpass_hz as libc::c_uint > pidFrequencyNyquist
       {
        ptermYawLowpassApplyFn =
            Some(nullFilterApply as
                     unsafe extern "C" fn(_: *mut filter_t, _: libc::c_float)
                         -> libc::c_float)
    } else {
        ptermYawLowpassApplyFn =
            ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                     *mut pt1Filter_t,
                                                                 _:
                                                                     libc::c_float)
                                                -> libc::c_float>,
                                     filterApplyFnPtr>(Some(pt1FilterApply as
                                                                unsafe extern "C" fn(_:
                                                                                         *mut pt1Filter_t,
                                                                                     _:
                                                                                         libc::c_float)
                                                                    ->
                                                                        libc::c_float));
        pt1FilterInit(&mut ptermYawLowpass,
                      pt1FilterGain((*pidProfile).yaw_lowpass_hz, dT));
    }
    pt1FilterInit(&mut throttleLpf,
                  pt1FilterGain((*pidProfile).throttle_boost_cutoff as
                                    uint16_t, dT));
    if itermRelax != 0 {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < 3 as libc::c_int {
            pt1FilterInit(&mut *windupLpf.as_mut_ptr().offset(i as isize),
                          pt1FilterGain(itermRelaxCutoff as uint16_t, dT));
            i += 1
        }
    }
    pt1FilterInit(&mut antiGravityThrottleLpf,
                  pt1FilterGain(15 as libc::c_int as uint16_t, dT));
}
#[no_mangle]
pub unsafe extern "C" fn pidInitSetpointDerivativeLpf(mut filterCutoff:
                                                          uint16_t,
                                                      mut debugAxis: uint8_t,
                                                      mut filterType:
                                                          uint8_t) {
    rcSmoothingDebugAxis = debugAxis;
    rcSmoothingFilterType = filterType;
    if filterCutoff as libc::c_int > 0 as libc::c_int &&
           rcSmoothingFilterType as libc::c_int !=
               RC_SMOOTHING_DERIVATIVE_OFF as libc::c_int {
        setpointDerivativeLpfInitialized = 1 as libc::c_int != 0;
        let mut axis: libc::c_int = FD_ROLL as libc::c_int;
        while axis <= FD_YAW as libc::c_int {
            match rcSmoothingFilterType as libc::c_int {
                1 => {
                    pt1FilterInit(&mut *setpointDerivativePt1.as_mut_ptr().offset(axis
                                                                                      as
                                                                                      isize),
                                  pt1FilterGain(filterCutoff, dT));
                }
                2 => {
                    biquadFilterInitLPF(&mut *setpointDerivativeBiquad.as_mut_ptr().offset(axis
                                                                                               as
                                                                                               isize),
                                        filterCutoff as libc::c_float,
                                        targetPidLooptime);
                }
                _ => { }
            }
            axis += 1
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn pidUpdateSetpointDerivativeLpf(mut filterCutoff:
                                                            uint16_t) {
    if filterCutoff as libc::c_int > 0 as libc::c_int &&
           rcSmoothingFilterType as libc::c_int !=
               RC_SMOOTHING_DERIVATIVE_OFF as libc::c_int {
        let mut axis: libc::c_int = FD_ROLL as libc::c_int;
        while axis <= FD_YAW as libc::c_int {
            match rcSmoothingFilterType as libc::c_int {
                1 => {
                    pt1FilterUpdateCutoff(&mut *setpointDerivativePt1.as_mut_ptr().offset(axis
                                                                                              as
                                                                                              isize),
                                          pt1FilterGain(filterCutoff, dT));
                }
                2 => {
                    biquadFilterUpdateLPF(&mut *setpointDerivativeBiquad.as_mut_ptr().offset(axis
                                                                                                 as
                                                                                                 isize),
                                          filterCutoff as libc::c_float,
                                          targetPidLooptime);
                }
                _ => { }
            }
            axis += 1
        }
    };
}
static mut pidCoefficient: [pidCoefficient_t; 3] =
    [pidCoefficient_t{Kp: 0., Ki: 0., Kd: 0., Kf: 0.,}; 3];
static mut maxVelocity: [libc::c_float; 3] = [0.; 3];
static mut feedForwardTransition: libc::c_float = 0.;
static mut levelGain: libc::c_float = 0.;
static mut horizonGain: libc::c_float = 0.;
static mut horizonTransition: libc::c_float = 0.;
static mut horizonCutoffDegrees: libc::c_float = 0.;
static mut horizonFactorRatio: libc::c_float = 0.;
static mut ITermWindupPointInv: libc::c_float = 0.;
static mut horizonTiltExpertMode: uint8_t = 0;
static mut crashTimeLimitUs: timeDelta_t = 0;
static mut crashTimeDelayUs: timeDelta_t = 0;
static mut crashRecoveryAngleDeciDegrees: int32_t = 0;
static mut crashRecoveryRate: libc::c_float = 0.;
static mut crashDtermThreshold: libc::c_float = 0.;
static mut crashGyroThreshold: libc::c_float = 0.;
static mut crashSetpointThreshold: libc::c_float = 0.;
static mut crashLimitYaw: libc::c_float = 0.;
static mut itermLimit: libc::c_float = 0.;
#[no_mangle]
pub static mut throttleBoost: libc::c_float = 0.;
#[no_mangle]
pub static mut throttleLpf: pt1Filter_t = pt1Filter_t{state: 0., k: 0.,};
static mut itermRotation: bool = false;
static mut smartFeedforward: bool = false;
#[no_mangle]
pub unsafe extern "C" fn pidResetITerm() {
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        pidData[axis as usize].I = 0.0f32;
        axis += 1
    };
}
static mut acroTrainerAngleLimit: libc::c_float = 0.;
static mut acroTrainerLookaheadTime: libc::c_float = 0.;
static mut acroTrainerDebugAxis: uint8_t = 0;
static mut acroTrainerActive: bool = false;
static mut acroTrainerAxisState: [libc::c_int; 2] = [0; 2];
// only need roll and pitch
static mut acroTrainerGain: libc::c_float = 0.;
// USE_ACRO_TRAINER
#[no_mangle]
pub unsafe extern "C" fn pidUpdateAntiGravityThrottleFilter(mut throttle:
                                                                libc::c_float) {
    if antiGravityMode as libc::c_int == ANTI_GRAVITY_SMOOTH as libc::c_int {
        antiGravityThrottleHpf =
            throttle - pt1FilterApply(&mut antiGravityThrottleLpf, throttle)
    };
}
#[no_mangle]
pub unsafe extern "C" fn pidInitConfig(mut pidProfile: *const pidProfile_t) {
    if (*pidProfile).feedForwardTransition as libc::c_int == 0 as libc::c_int
       {
        feedForwardTransition = 0 as libc::c_int as libc::c_float
    } else {
        feedForwardTransition =
            100.0f32 /
                (*pidProfile).feedForwardTransition as libc::c_int as
                    libc::c_float
    }
    let mut axis: libc::c_int = FD_ROLL as libc::c_int;
    while axis <= FD_YAW as libc::c_int {
        pidCoefficient[axis as usize].Kp =
            0.032029f32 *
                (*pidProfile).pid[axis as usize].P as libc::c_int as
                    libc::c_float;
        pidCoefficient[axis as usize].Ki =
            0.244381f32 *
                (*pidProfile).pid[axis as usize].I as libc::c_int as
                    libc::c_float;
        pidCoefficient[axis as usize].Kd =
            0.000529f32 *
                (*pidProfile).pid[axis as usize].D as libc::c_int as
                    libc::c_float;
        pidCoefficient[axis as usize].Kf =
            0.013754f32 *
                ((*pidProfile).pid[axis as usize].F as libc::c_int as
                     libc::c_float / 100.0f32);
        axis += 1
    }
    levelGain =
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].P as libc::c_int
            as libc::c_float / 10.0f32;
    horizonGain =
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].I as libc::c_int
            as libc::c_float / 10.0f32;
    horizonTransition =
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].D as
            libc::c_float;
    horizonTiltExpertMode = (*pidProfile).horizon_tilt_expert_mode;
    horizonCutoffDegrees =
        (175 as libc::c_int -
             (*pidProfile).horizon_tilt_effect as libc::c_int) as
            libc::c_float * 1.8f32;
    horizonFactorRatio =
        (100 as libc::c_int -
             (*pidProfile).horizon_tilt_effect as libc::c_int) as
            libc::c_float * 0.01f32;
    maxVelocity[FD_PITCH as libc::c_int as usize] =
        ((*pidProfile).rateAccelLimit as libc::c_int * 100 as libc::c_int) as
            libc::c_float * dT;
    maxVelocity[FD_ROLL as libc::c_int as usize] =
        maxVelocity[FD_PITCH as libc::c_int as usize];
    maxVelocity[FD_YAW as libc::c_int as usize] =
        ((*pidProfile).yawRateAccelLimit as libc::c_int * 100 as libc::c_int)
            as libc::c_float * dT;
    let ITermWindupPoint: libc::c_float =
        (*pidProfile).itermWindupPointPercent as libc::c_float / 100.0f32;
    ITermWindupPointInv = 1.0f32 / (1.0f32 - ITermWindupPoint);
    itermAcceleratorGain = (*pidProfile).itermAcceleratorGain;
    crashTimeLimitUs =
        (*pidProfile).crash_time as libc::c_int * 1000 as libc::c_int;
    crashTimeDelayUs =
        (*pidProfile).crash_delay as libc::c_int * 1000 as libc::c_int;
    crashRecoveryAngleDeciDegrees =
        (*pidProfile).crash_recovery_angle as libc::c_int * 10 as libc::c_int;
    crashRecoveryRate = (*pidProfile).crash_recovery_rate as libc::c_float;
    crashGyroThreshold = (*pidProfile).crash_gthreshold as libc::c_float;
    crashDtermThreshold = (*pidProfile).crash_dthreshold as libc::c_float;
    crashSetpointThreshold =
        (*pidProfile).crash_setpoint_threshold as libc::c_float;
    crashLimitYaw = (*pidProfile).crash_limit_yaw as libc::c_float;
    itermLimit = (*pidProfile).itermLimit as libc::c_float;
    throttleBoost =
        (*pidProfile).throttle_boost as libc::c_int as libc::c_float * 0.1f32;
    itermRotation = (*pidProfile).iterm_rotation != 0;
    antiGravityMode = (*pidProfile).antiGravityMode;
    // Calculate the anti-gravity value that will trigger the OSD display.
    // For classic AG it's either 1.0 for off and > 1.0 for on.
    // For the new AG it's a continuous floating value so we want to trigger the OSD
    // display when it exceeds 25% of its possible range. This gives a useful indication
    // of AG activity without excessive display.
    antiGravityOsdCutoff = 1.0f32;
    if antiGravityMode as libc::c_int == ANTI_GRAVITY_SMOOTH as libc::c_int {
        antiGravityOsdCutoff +=
            (itermAcceleratorGain as libc::c_int - 1000 as libc::c_int) as
                libc::c_float / 1000.0f32 * 0.25f32
    }
    smartFeedforward = (*pidProfile).smart_feedforward != 0;
    itermRelax = (*pidProfile).iterm_relax;
    itermRelaxType = (*pidProfile).iterm_relax_type;
    itermRelaxCutoff = (*pidProfile).iterm_relax_cutoff;
    acroTrainerAngleLimit =
        (*pidProfile).acro_trainer_angle_limit as libc::c_float;
    acroTrainerLookaheadTime =
        (*pidProfile).acro_trainer_lookahead_ms as libc::c_float / 1000.0f32;
    acroTrainerDebugAxis = (*pidProfile).acro_trainer_debug_axis;
    acroTrainerGain =
        (*pidProfile).acro_trainer_gain as libc::c_float / 10.0f32;
    // USE_ACRO_TRAINER
}
#[no_mangle]
pub unsafe extern "C" fn pidInit(mut pidProfile: *const pidProfile_t) {
    pidSetTargetLooptime(gyro.targetLooptime.wrapping_mul((*pidConfig()).pid_process_denom
                                                              as
                                                              libc::c_uint)); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
}
#[no_mangle]
pub unsafe extern "C" fn pidAcroTrainerInit() {
    acroTrainerAxisState[FD_ROLL as libc::c_int as usize] = 0 as libc::c_int;
    acroTrainerAxisState[FD_PITCH as libc::c_int as usize] = 0 as libc::c_int;
}
// USE_ACRO_TRAINER
#[no_mangle]
pub unsafe extern "C" fn pidCopyProfile(mut dstPidProfileIndex: uint8_t,
                                        mut srcPidProfileIndex: uint8_t) {
    if (dstPidProfileIndex as libc::c_int) <
           3 as libc::c_int - 1 as libc::c_int &&
           (srcPidProfileIndex as libc::c_int) <
               3 as libc::c_int - 1 as libc::c_int &&
           dstPidProfileIndex as libc::c_int !=
               srcPidProfileIndex as libc::c_int {
        memcpy(pidProfilesMutable(dstPidProfileIndex as libc::c_int) as
                   *mut libc::c_void,
               pidProfilesMutable(srcPidProfileIndex as libc::c_int) as
                   *const libc::c_void,
               ::core::mem::size_of::<pidProfile_t>() as libc::c_ulong);
    };
}
// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
unsafe extern "C" fn calcHorizonLevelStrength() -> libc::c_float {
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    let mut horizonLevelStrength: libc::c_float =
        1.0f32 -
            ({
                 let mut _a: libc::c_float =
                     getRcDeflectionAbs(FD_ROLL as libc::c_int);
                 let mut _b: libc::c_float =
                     getRcDeflectionAbs(FD_PITCH as libc::c_int);
                 (if _a > _b { _a } else { _b })
             });
    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    let currentInclination: libc::c_float =
        ({
             let mut _a: libc::c_int =
                 ({
                      let mut _x: int16_t = attitude.values.roll;
                      (if _x as libc::c_int > 0 as libc::c_int {
                           _x as libc::c_int
                       } else { -(_x as libc::c_int) })
                  });
             let mut _b: libc::c_int =
                 ({
                      let mut _x: int16_t = attitude.values.pitch;
                      (if _x as libc::c_int > 0 as libc::c_int {
                           _x as libc::c_int
                       } else { -(_x as libc::c_int) })
                  });
             (if _a > _b { _a } else { _b })
         }) as libc::c_float / 10.0f32;
    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if horizonTiltExpertMode != 0 {
        if horizonTransition > 0 as libc::c_int as libc::c_float &&
               horizonCutoffDegrees > 0 as libc::c_int as libc::c_float {
            let inclinationLevelRatio: libc::c_float =
                constrainf((horizonCutoffDegrees - currentInclination) /
                               horizonCutoffDegrees,
                           0 as libc::c_int as libc::c_float,
                           1 as libc::c_int as
                               libc::c_float); // horizon_tilt_expert_mode = 0 (leveling always active when sticks centered)
            horizonLevelStrength =
                (horizonLevelStrength - 1 as libc::c_int as libc::c_float) *
                    100 as libc::c_int as libc::c_float / horizonTransition +
                    1 as libc::c_int as libc::c_float;
            horizonLevelStrength *= inclinationLevelRatio
        } else { horizonLevelStrength = 0 as libc::c_int as libc::c_float }
    } else {
        let mut sensitFact: libc::c_float =
            0.; // d_level=0 or horizon_tilt_effect>=175 means no leveling
        if horizonFactorRatio < 1.01f32 {
            let inclinationLevelRatio_0: libc::c_float =
                (180 as libc::c_int as libc::c_float - currentInclination) /
                    180 as libc::c_int as libc::c_float *
                    (1.0f32 - horizonFactorRatio) + horizonFactorRatio;
            // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            // apply configured horizon sensitivity:
                // when stick is near center (horizonLevelStrength ~= 1.0)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0.0)
                //  H_sensitivity value has more effect:
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            sensitFact = horizonTransition * inclinationLevelRatio_0
        } else {
            sensitFact = horizonTransition
        } // horizonTiltEffect=0 for "old" functionality
        if sensitFact <= 0 as libc::c_int as libc::c_float {
            // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            // apply ratio to configured horizon sensitivity:
            // zero means no leveling
            horizonLevelStrength = 0 as libc::c_int as libc::c_float
        } else {
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  sensitFact value has more effect:
            horizonLevelStrength =
                (horizonLevelStrength - 1 as libc::c_int as libc::c_float) *
                    (100 as libc::c_int as libc::c_float / sensitFact) +
                    1 as libc::c_int as libc::c_float
        }
    }
    return constrainf(horizonLevelStrength, 0 as libc::c_int as libc::c_float,
                      1 as libc::c_int as libc::c_float);
}
unsafe extern "C" fn pidLevel(mut axis: libc::c_int,
                              mut pidProfile: *const pidProfile_t,
                              mut angleTrim: *const rollAndPitchTrims_t,
                              mut currentPidSetpoint: libc::c_float)
 -> libc::c_float {
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    let mut angle: libc::c_float =
        (*pidProfile).levelAngleLimit as libc::c_int as libc::c_float *
            getRcDeflection(axis);
    angle =
        constrainf(angle,
                   -((*pidProfile).levelAngleLimit as libc::c_int) as
                       libc::c_float,
                   (*pidProfile).levelAngleLimit as libc::c_float);
    let errorAngle: libc::c_float =
        angle -
            (attitude.raw[axis as usize] as libc::c_int -
                 (*angleTrim).raw[axis as usize] as libc::c_int) as
                libc::c_float / 10.0f32;
    if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int != 0 ||
           flightModeFlags as libc::c_int & GPS_RESCUE_MODE as libc::c_int !=
               0 {
        // ANGLE mode - control is angle based
        currentPidSetpoint = errorAngle * levelGain
    } else {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        let horizonLevelStrength: libc::c_float = calcHorizonLevelStrength();
        currentPidSetpoint =
            currentPidSetpoint +
                errorAngle * horizonGain * horizonLevelStrength
    }
    return currentPidSetpoint;
}
unsafe extern "C" fn accelerationLimit(mut axis: libc::c_int,
                                       mut currentPidSetpoint: libc::c_float)
 -> libc::c_float {
    static mut previousSetpoint: [libc::c_float; 3] = [0.; 3];
    let currentVelocity: libc::c_float =
        currentPidSetpoint - previousSetpoint[axis as usize];
    if ({
            let _x: libc::c_float = currentVelocity;
            (if _x > 0 as libc::c_int as libc::c_float { _x } else { -_x })
        }) > maxVelocity[axis as usize] {
        currentPidSetpoint =
            if currentVelocity > 0 as libc::c_int as libc::c_float {
                (previousSetpoint[axis as usize]) + maxVelocity[axis as usize]
            } else {
                (previousSetpoint[axis as usize]) - maxVelocity[axis as usize]
            }
    }
    previousSetpoint[axis as usize] = currentPidSetpoint;
    return currentPidSetpoint;
}
static mut crashDetectedAtUs: timeUs_t = 0;
unsafe extern "C" fn handleCrashRecovery(crash_recovery: pidCrashRecovery_e,
                                         mut angleTrim:
                                             *const rollAndPitchTrims_t,
                                         axis: libc::c_int,
                                         currentTimeUs: timeUs_t,
                                         gyroRate: libc::c_float,
                                         mut currentPidSetpoint:
                                             *mut libc::c_float,
                                         mut errorRate: *mut libc::c_float) {
    if inCrashRecoveryMode as libc::c_int != 0 &&
           cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeDelayUs {
        if crash_recovery as libc::c_uint ==
               PID_CRASH_RECOVERY_BEEP as libc::c_int as libc::c_uint {
            systemBeep(1 as libc::c_int != 0);
        }
        if axis == FD_YAW as libc::c_int {
            *errorRate = constrainf(*errorRate, -crashLimitYaw, crashLimitYaw)
        } else if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            // errorAngle is deviation from horizontal
            let errorAngle: libc::c_float =
                -(attitude.raw[axis as usize] as libc::c_int -
                      (*angleTrim).raw[axis as usize] as libc::c_int) as
                    libc::c_float / 10.0f32;
            *currentPidSetpoint = errorAngle * levelGain;
            *errorRate = *currentPidSetpoint - gyroRate
        }
        // reset ITerm, since accumulated error before crash is now meaningless
        // and ITerm windup during crash recovery can be extreme, especially on yaw axis
        pidData[axis as usize].I = 0.0f32;
        if cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeLimitUs ||
               getMotorMixRange() < 1.0f32 &&
                   ({
                        let mut _x: libc::c_float =
                            gyro.gyroADCf[FD_ROLL as libc::c_int as usize];
                        (if _x > 0 as libc::c_int as libc::c_float {
                             _x
                         } else { -_x })
                    }) < crashRecoveryRate &&
                   ({
                        let mut _x: libc::c_float =
                            gyro.gyroADCf[FD_PITCH as libc::c_int as usize];
                        (if _x > 0 as libc::c_int as libc::c_float {
                             _x
                         } else { -_x })
                    }) < crashRecoveryRate &&
                   ({
                        let mut _x: libc::c_float =
                            gyro.gyroADCf[FD_YAW as libc::c_int as usize];
                        (if _x > 0 as libc::c_int as libc::c_float {
                             _x
                         } else { -_x })
                    }) < crashRecoveryRate {
            if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
                // check aircraft nearly level
                if ({
                        let mut _x: libc::c_int =
                            attitude.raw[FD_ROLL as libc::c_int as usize] as
                                libc::c_int -
                                (*angleTrim).raw[FD_ROLL as libc::c_int as
                                                     usize] as libc::c_int;
                        (if _x > 0 as libc::c_int { _x } else { -_x })
                    }) < crashRecoveryAngleDeciDegrees &&
                       ({
                            let mut _x: libc::c_int =
                                attitude.raw[FD_PITCH as libc::c_int as usize]
                                    as libc::c_int -
                                    (*angleTrim).raw[FD_PITCH as libc::c_int
                                                         as usize] as
                                        libc::c_int;
                            (if _x > 0 as libc::c_int { _x } else { -_x })
                        }) < crashRecoveryAngleDeciDegrees {
                    inCrashRecoveryMode = 0 as libc::c_int != 0;
                    systemBeep(0 as libc::c_int != 0);
                }
            } else {
                inCrashRecoveryMode = 0 as libc::c_int != 0;
                systemBeep(0 as libc::c_int != 0);
            }
        }
    };
}
unsafe extern "C" fn detectAndSetCrashRecovery(crash_recovery:
                                                   pidCrashRecovery_e,
                                               axis: libc::c_int,
                                               currentTimeUs: timeUs_t,
                                               delta: libc::c_float,
                                               errorRate: libc::c_float) {
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if (crash_recovery as libc::c_uint != 0 ||
            flightModeFlags as libc::c_int & GPS_RESCUE_MODE as libc::c_int !=
                0) && !gyroOverflowDetected() {
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            if getMotorMixRange() >= 1.0f32 && !inCrashRecoveryMode &&
                   ({
                        let _x: libc::c_float = delta;
                        (if _x > 0 as libc::c_int as libc::c_float {
                             _x
                         } else { -_x })
                    }) > crashDtermThreshold &&
                   ({
                        let _x: libc::c_float = errorRate;
                        (if _x > 0 as libc::c_int as libc::c_float {
                             _x
                         } else { -_x })
                    }) > crashGyroThreshold &&
                   ({
                        let mut _x: libc::c_float = getSetpointRate(axis);
                        (if _x > 0 as libc::c_int as libc::c_float {
                             _x
                         } else { -_x })
                    }) < crashSetpointThreshold {
                inCrashRecoveryMode = 1 as libc::c_int != 0;
                crashDetectedAtUs = currentTimeUs
            }
            if inCrashRecoveryMode as libc::c_int != 0 &&
                   cmpTimeUs(currentTimeUs, crashDetectedAtUs) <
                       crashTimeDelayUs &&
                   (({
                         let _x: libc::c_float = errorRate;
                         (if _x > 0 as libc::c_int as libc::c_float {
                              _x
                          } else { -_x })
                     }) < crashGyroThreshold ||
                        ({
                             let mut _x: libc::c_float =
                                 getSetpointRate(axis);
                             (if _x > 0 as libc::c_int as libc::c_float {
                                  _x
                              } else { -_x })
                         }) > crashSetpointThreshold) {
                inCrashRecoveryMode = 0 as libc::c_int != 0;
                systemBeep(0 as libc::c_int != 0);
            }
        } else if inCrashRecoveryMode {
            inCrashRecoveryMode = 0 as libc::c_int != 0;
            systemBeep(0 as libc::c_int != 0);
        }
    };
}
unsafe extern "C" fn rotateVector(mut v: *mut libc::c_float,
                                  mut rotation: *mut libc::c_float) {
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        let mut i_1: libc::c_int = (i + 1 as libc::c_int) % 3 as libc::c_int;
        let mut i_2: libc::c_int = (i + 2 as libc::c_int) % 3 as libc::c_int;
        let mut newV: libc::c_float =
            *v.offset(i_1 as isize) +
                *v.offset(i_2 as isize) * *rotation.offset(i as isize);
        *v.offset(i_2 as isize) -=
            *v.offset(i_1 as isize) * *rotation.offset(i as isize);
        *v.offset(i_1 as isize) = newV;
        i += 1
    };
}
unsafe extern "C" fn rotateITermAndAxisError() {
    if itermRotation {
        let gyroToAngle: libc::c_float =
            dT * (3.14159265358979323846f32 / 180.0f32);
        let mut rotationRads: [libc::c_float; 3] = [0.; 3];
        let mut i: libc::c_int = FD_ROLL as libc::c_int;
        while i <= FD_YAW as libc::c_int {
            rotationRads[i as usize] =
                gyro.gyroADCf[i as usize] * gyroToAngle;
            i += 1
        }
        if itermRotation {
            let mut v: [libc::c_float; 3] = [0.; 3];
            let mut i_0: libc::c_int = 0 as libc::c_int;
            while i_0 < 3 as libc::c_int {
                v[i_0 as usize] = pidData[i_0 as usize].I;
                i_0 += 1
            }
            rotateVector(v.as_mut_ptr(), rotationRads.as_mut_ptr());
            let mut i_1: libc::c_int = 0 as libc::c_int;
            while i_1 < 3 as libc::c_int {
                pidData[i_1 as usize].I = v[i_1 as usize];
                i_1 += 1
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn acroTrainerSign(mut x: libc::c_float)
 -> libc::c_int {
    return if x > 0 as libc::c_int as libc::c_float {
               1 as libc::c_int
           } else { -(1 as libc::c_int) };
}
// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint
// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
unsafe extern "C" fn applyAcroTrainer(mut axis: libc::c_int,
                                      mut angleTrim:
                                          *const rollAndPitchTrims_t,
                                      mut setPoint: libc::c_float)
 -> libc::c_float {
    let mut ret: libc::c_float = setPoint;
    if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int == 0 &&
           flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int == 0
           &&
           flightModeFlags as libc::c_int & GPS_RESCUE_MODE as libc::c_int ==
               0 {
        let mut resetIterm: bool = 0 as libc::c_int != 0;
        let mut projectedAngle: libc::c_float =
            0 as libc::c_int as libc::c_float;
        let setpointSign: libc::c_int = acroTrainerSign(setPoint);
        let currentAngle: libc::c_float =
            (attitude.raw[axis as usize] as libc::c_int -
                 (*angleTrim).raw[axis as usize] as libc::c_int) as
                libc::c_float / 10.0f32;
        let angleSign: libc::c_int = acroTrainerSign(currentAngle);
        if acroTrainerAxisState[axis as usize] != 0 as libc::c_int &&
               acroTrainerAxisState[axis as usize] != setpointSign {
            // stick has reversed - stop limiting
            acroTrainerAxisState[axis as usize] = 0 as libc::c_int
        }
        // Limit and correct the angle when it exceeds the limit
        if fabsf(currentAngle) > acroTrainerAngleLimit &&
               acroTrainerAxisState[axis as usize] == 0 as libc::c_int {
            if angleSign == setpointSign {
                acroTrainerAxisState[axis as usize] = angleSign;
                resetIterm = 1 as libc::c_int != 0
            }
        }
        if acroTrainerAxisState[axis as usize] != 0 as libc::c_int {
            ret =
                constrainf((acroTrainerAngleLimit * angleSign as libc::c_float
                                - currentAngle) * acroTrainerGain, -1000.0f32,
                           1000.0f32)
        } else {
            // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            let mut checkInterval: libc::c_float =
                constrainf(fabsf(gyro.gyroADCf[axis as usize]) / 500.0f32,
                           0.0f32, 1.0f32) * acroTrainerLookaheadTime;
            projectedAngle =
                gyro.gyroADCf[axis as usize] * checkInterval + currentAngle;
            let projectedAngleSign: libc::c_int =
                acroTrainerSign(projectedAngle);
            if fabsf(projectedAngle) > acroTrainerAngleLimit &&
                   projectedAngleSign == setpointSign {
                ret =
                    (acroTrainerAngleLimit *
                         projectedAngleSign as libc::c_float - projectedAngle)
                        * acroTrainerGain;
                resetIterm = 1 as libc::c_int != 0
            }
        }
        if resetIterm {
            pidData[axis as usize].I = 0 as libc::c_int as libc::c_float
        }
        if axis == acroTrainerDebugAxis as libc::c_int {
            if debugMode as libc::c_int == DEBUG_ACRO_TRAINER as libc::c_int {
                debug[0 as libc::c_int as usize] =
                    lrintf(currentAngle * 10.0f32) as int16_t
            }
            if debugMode as libc::c_int == DEBUG_ACRO_TRAINER as libc::c_int {
                debug[1 as libc::c_int as usize] =
                    acroTrainerAxisState[axis as usize] as int16_t
            }
            if debugMode as libc::c_int == DEBUG_ACRO_TRAINER as libc::c_int {
                debug[2 as libc::c_int as usize] = lrintf(ret) as int16_t
            }
            if debugMode as libc::c_int == DEBUG_ACRO_TRAINER as libc::c_int {
                debug[3 as libc::c_int as usize] =
                    lrintf(projectedAngle * 10.0f32) as int16_t
            }
        }
    }
    return ret;
}
// USE_ACRO_TRAINER
#[no_mangle]
pub unsafe extern "C" fn applyRcSmoothingDerivativeFilter(mut axis:
                                                              libc::c_int,
                                                          mut pidSetpointDelta:
                                                              libc::c_float)
 -> libc::c_float {
    let mut ret: libc::c_float = pidSetpointDelta;
    if axis == rcSmoothingDebugAxis as libc::c_int {
        if debugMode as libc::c_int == DEBUG_RC_SMOOTHING as libc::c_int {
            debug[1 as libc::c_int as usize] =
                lrintf(pidSetpointDelta * 100.0f32) as int16_t
        }
    }
    if setpointDerivativeLpfInitialized {
        match rcSmoothingFilterType as libc::c_int {
            1 => {
                ret =
                    pt1FilterApply(&mut *setpointDerivativePt1.as_mut_ptr().offset(axis
                                                                                       as
                                                                                       isize),
                                   pidSetpointDelta)
            }
            2 => {
                ret =
                    biquadFilterApplyDF1(&mut *setpointDerivativeBiquad.as_mut_ptr().offset(axis
                                                                                                as
                                                                                                isize),
                                         pidSetpointDelta)
            }
            _ => { }
        }
        if axis == rcSmoothingDebugAxis as libc::c_int {
            if debugMode as libc::c_int == DEBUG_RC_SMOOTHING as libc::c_int {
                debug[2 as libc::c_int as usize] =
                    lrintf(ret * 100.0f32) as int16_t
            }
        }
    }
    return ret;
}
// USE_RC_SMOOTHING_FILTER
#[no_mangle]
pub unsafe extern "C" fn applySmartFeedforward(mut axis: libc::c_int) {
    if smartFeedforward {
        if pidData[axis as usize].P * pidData[axis as usize].F >
               0 as libc::c_int as libc::c_float {
            if ({
                    let mut _x: libc::c_float = pidData[axis as usize].F;
                    (if _x > 0 as libc::c_int as libc::c_float {
                         _x
                     } else { -_x })
                }) >
                   ({
                        let mut _x: libc::c_float = pidData[axis as usize].P;
                        (if _x > 0 as libc::c_int as libc::c_float {
                             _x
                         } else { -_x })
                    }) {
                pidData[axis as usize].P = 0 as libc::c_int as libc::c_float
            } else {
                pidData[axis as usize].F = 0 as libc::c_int as libc::c_float
            }
        }
    };
}
// USE_SMART_FEEDFORWARD
// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
#[no_mangle]
pub unsafe extern "C" fn pidController(mut pidProfile: *const pidProfile_t,
                                       mut angleTrim:
                                           *const rollAndPitchTrims_t,
                                       mut currentTimeUs: timeUs_t) {
    static mut previousGyroRateDterm: [libc::c_float; 3] = [0.; 3];
    static mut previousPidSetpoint: [libc::c_float; 3] = [0.; 3];
    let tpaFactor: libc::c_float = getThrottlePIDAttenuation();
    let motorMixRange: libc::c_float = getMotorMixRange();
    let yawSpinActive: bool = gyroYawSpinDetected();
    // Dynamic i component,
    if antiGravityMode as libc::c_int == ANTI_GRAVITY_SMOOTH as libc::c_int &&
           antiGravityEnabled as libc::c_int != 0 {
        itermAccelerator =
            1 as libc::c_int as libc::c_float +
                fabsf(antiGravityThrottleHpf) * 0.01f32 *
                    (itermAcceleratorGain as libc::c_int -
                         1000 as libc::c_int) as libc::c_float;
        if debugMode as libc::c_int == DEBUG_ANTI_GRAVITY as libc::c_int {
            debug[1 as libc::c_int as usize] =
                lrintf(antiGravityThrottleHpf *
                           1000 as libc::c_int as libc::c_float) as int16_t
        }
    }
    if debugMode as libc::c_int == DEBUG_ANTI_GRAVITY as libc::c_int {
        debug[0 as libc::c_int as usize] =
            lrintf(itermAccelerator * 1000 as libc::c_int as libc::c_float) as
                int16_t
    }
    // gradually scale back integration when above windup point
    let dynCi: libc::c_float =
        ({
             let mut _a: libc::c_float =
                 (1.0f32 - motorMixRange) * ITermWindupPointInv;
             let mut _b: libc::c_float = 1.0f32;
             (if _a < _b { _a } else { _b })
         }) * dT * itermAccelerator;
    // Precalculate gyro deta for D-term here, this allows loop unrolling
    let mut gyroRateDterm: [libc::c_float; 3] = [0.; 3];
    let mut axis: libc::c_int = FD_ROLL as libc::c_int;
    while axis <= FD_YAW as libc::c_int {
        gyroRateDterm[axis as usize] =
            dtermNotchApplyFn.expect("non-null function pointer")(&mut *dtermNotch.as_mut_ptr().offset(axis
                                                                                                           as
                                                                                                           isize)
                                                                      as
                                                                      *mut biquadFilter_t
                                                                      as
                                                                      *mut filter_t,
                                                                  gyro.gyroADCf[axis
                                                                                    as
                                                                                    usize]);
        gyroRateDterm[axis as usize] =
            dtermLowpassApplyFn.expect("non-null function pointer")(&mut *dtermLowpass.as_mut_ptr().offset(axis
                                                                                                               as
                                                                                                               isize)
                                                                        as
                                                                        *mut dtermLowpass_t
                                                                        as
                                                                        *mut filter_t,
                                                                    gyroRateDterm[axis
                                                                                      as
                                                                                      usize]);
        gyroRateDterm[axis as usize] =
            dtermLowpass2ApplyFn.expect("non-null function pointer")(&mut *dtermLowpass2.as_mut_ptr().offset(axis
                                                                                                                 as
                                                                                                                 isize)
                                                                         as
                                                                         *mut pt1Filter_t
                                                                         as
                                                                         *mut filter_t,
                                                                     gyroRateDterm[axis
                                                                                       as
                                                                                       usize]);
        axis += 1
    }
    rotateITermAndAxisError();
    // ----------PID controller----------
    let mut axis_0: libc::c_int = FD_ROLL as libc::c_int;
    while axis_0 <= FD_YAW as libc::c_int {
        let mut currentPidSetpoint: libc::c_float = getSetpointRate(axis_0);
        if maxVelocity[axis_0 as usize] != 0. {
            currentPidSetpoint = accelerationLimit(axis_0, currentPidSetpoint)
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        if (flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int != 0 ||
                flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int
                    != 0 ||
                flightModeFlags as libc::c_int &
                    GPS_RESCUE_MODE as libc::c_int != 0) &&
               axis_0 != FD_YAW as libc::c_int {
            currentPidSetpoint =
                pidLevel(axis_0, pidProfile, angleTrim, currentPidSetpoint)
        }
        if axis_0 != FD_YAW as libc::c_int &&
               acroTrainerActive as libc::c_int != 0 && !inCrashRecoveryMode {
            currentPidSetpoint =
                applyAcroTrainer(axis_0, angleTrim, currentPidSetpoint)
        }
        // USE_ACRO_TRAINER
        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
        if axis_0 == FD_YAW as libc::c_int &&
               yawSpinActive as libc::c_int != 0 {
            currentPidSetpoint = 0.0f32
        }
        // USE_YAW_SPIN_RECOVERY
        // -----calculate error rate
        let gyroRate: libc::c_float =
            gyro.gyroADCf[axis_0 as
                              usize]; // Process variable from gyro output in deg/sec
        let ITerm: libc::c_float = pidData[axis_0 as usize].I;
        let mut itermErrorRate: libc::c_float = currentPidSetpoint - gyroRate;
        if itermRelax as libc::c_int != 0 &&
               (axis_0 < FD_YAW as libc::c_int ||
                    itermRelax as libc::c_int ==
                        ITERM_RELAX_RPY as libc::c_int ||
                    itermRelax as libc::c_int ==
                        ITERM_RELAX_RPY_INC as libc::c_int) {
            let setpointLpf: libc::c_float =
                pt1FilterApply(&mut *windupLpf.as_mut_ptr().offset(axis_0 as
                                                                       isize),
                               currentPidSetpoint);
            let setpointHpf: libc::c_float =
                fabsf(currentPidSetpoint - setpointLpf);
            let itermRelaxFactor: libc::c_float =
                1 as libc::c_int as libc::c_float - setpointHpf / 30.0f32;
            let isDecreasingI: bool =
                ITerm > 0 as libc::c_int as libc::c_float &&
                    itermErrorRate < 0 as libc::c_int as libc::c_float ||
                    ITerm < 0 as libc::c_int as libc::c_float &&
                        itermErrorRate > 0 as libc::c_int as libc::c_float;
            if !(itermRelax as libc::c_int >=
                     ITERM_RELAX_RP_INC as libc::c_int &&
                     isDecreasingI as libc::c_int != 0) {
                if itermRelaxType as libc::c_int ==
                       ITERM_RELAX_SETPOINT as libc::c_int &&
                       setpointHpf < 30 as libc::c_int as libc::c_float {
                    itermErrorRate *= itermRelaxFactor
                } else if itermRelaxType as libc::c_int ==
                              ITERM_RELAX_GYRO as libc::c_int {
                    itermErrorRate =
                        fapplyDeadband(setpointLpf - gyroRate, setpointHpf)
                } else { itermErrorRate = 0.0f32 }
            }
            if axis_0 == FD_ROLL as libc::c_int {
                if debugMode as libc::c_int ==
                       DEBUG_ITERM_RELAX as libc::c_int {
                    debug[0 as libc::c_int as usize] =
                        lrintf(setpointHpf) as int16_t
                }
                if debugMode as libc::c_int ==
                       DEBUG_ITERM_RELAX as libc::c_int {
                    debug[1 as libc::c_int as usize] =
                        lrintf(itermRelaxFactor * 100.0f32) as int16_t
                }
                if debugMode as libc::c_int ==
                       DEBUG_ITERM_RELAX as libc::c_int {
                    debug[2 as libc::c_int as usize] =
                        lrintf(itermErrorRate) as int16_t
                }
            }
            // USE_ABSOLUTE_CONTROL             
        } // r - y
        let mut errorRate: libc::c_float = currentPidSetpoint - gyroRate;
        handleCrashRecovery((*pidProfile).crash_recovery as
                                pidCrashRecovery_e, angleTrim, axis_0,
                            currentTimeUs, gyroRate, &mut currentPidSetpoint,
                            &mut errorRate);
        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).
        // -----calculate P component and add Dynamic Part based on stick input
        pidData[axis_0 as usize].P =
            pidCoefficient[axis_0 as usize].Kp * errorRate * tpaFactor;
        if axis_0 == FD_YAW as libc::c_int {
            pidData[axis_0 as usize].P =
                ptermYawLowpassApplyFn.expect("non-null function pointer")(&mut ptermYawLowpass
                                                                               as
                                                                               *mut pt1Filter_t
                                                                               as
                                                                               *mut filter_t,
                                                                           pidData[axis_0
                                                                                       as
                                                                                       usize].P)
        }
        // -----calculate I component
        let ITermNew: libc::c_float =
            constrainf(ITerm +
                           pidCoefficient[axis_0 as usize].Ki * itermErrorRate
                               * dynCi, -itermLimit, itermLimit);
        let outputSaturated: bool = mixerIsOutputSaturated(axis_0, errorRate);
        if outputSaturated as libc::c_int == 0 as libc::c_int ||
               ({
                    let _x: libc::c_float = ITermNew;
                    (if _x > 0 as libc::c_int as libc::c_float {
                         _x
                     } else { -_x })
                }) <
                   ({
                        let _x: libc::c_float = ITerm;
                        (if _x > 0 as libc::c_int as libc::c_float {
                             _x
                         } else { -_x })
                    }) {
            // Only increase ITerm if output is not saturated
            pidData[axis_0 as usize].I = ITermNew
        }
        // -----calculate D component
        if pidCoefficient[axis_0 as usize].Kd >
               0 as libc::c_int as libc::c_float {
            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            let delta: libc::c_float =
                -(gyroRateDterm[axis_0 as usize] -
                      previousGyroRateDterm[axis_0 as usize]) * pidFrequency;
            detectAndSetCrashRecovery((*pidProfile).crash_recovery as
                                          pidCrashRecovery_e, axis_0,
                                      currentTimeUs, delta, errorRate);
            pidData[axis_0 as usize].D =
                pidCoefficient[axis_0 as usize].Kd * delta * tpaFactor
        } else {
            pidData[axis_0 as usize].D = 0 as libc::c_int as libc::c_float
        }
        previousGyroRateDterm[axis_0 as usize] =
            gyroRateDterm[axis_0 as usize];
        // -----calculate feedforward component
        // Only enable feedforward for rate mode
        let feedforwardGain: libc::c_float =
            if flightModeFlags as libc::c_int != 0 {
                0.0f32
            } else { pidCoefficient[axis_0 as usize].Kf };
        if feedforwardGain > 0 as libc::c_int as libc::c_float {
            // no transition if feedForwardTransition == 0
            let mut transition: libc::c_float =
                if feedForwardTransition > 0 as libc::c_int as libc::c_float {
                    ({
                         let mut _a: libc::c_float = 1.0f32;
                         let mut _b: libc::c_float =
                             getRcDeflectionAbs(axis_0) *
                                 feedForwardTransition;
                         if _a < _b { _a } else { _b }
                     })
                } else { 1 as libc::c_int as libc::c_float };
            let mut pidSetpointDelta: libc::c_float =
                currentPidSetpoint - previousPidSetpoint[axis_0 as usize];
            pidSetpointDelta =
                applyRcSmoothingDerivativeFilter(axis_0, pidSetpointDelta);
            // USE_RC_SMOOTHING_FILTER
            pidData[axis_0 as usize].F =
                feedforwardGain * transition * pidSetpointDelta *
                    pidFrequency; // in yaw spin always disable I
            applySmartFeedforward(axis_0);
        } else {
            pidData[axis_0 as usize].F = 0 as libc::c_int as libc::c_float
        }
        previousPidSetpoint[axis_0 as usize] = currentPidSetpoint;
        if yawSpinActive {
            pidData[axis_0 as usize].I = 0 as libc::c_int as libc::c_float;
            if axis_0 <= FD_PITCH as libc::c_int {
                // zero PIDs on pitch and roll leaving yaw P to correct spin 
                pidData[axis_0 as usize].P =
                    0 as libc::c_int as libc::c_float;
                pidData[axis_0 as usize].D =
                    0 as libc::c_int as libc::c_float;
                pidData[axis_0 as usize].F = 0 as libc::c_int as libc::c_float
            }
        }
        // USE_YAW_SPIN_RECOVERY
        // calculating the PID sum
        pidData[axis_0 as usize].Sum =
            pidData[axis_0 as usize].P + pidData[axis_0 as usize].I +
                pidData[axis_0 as usize].D + pidData[axis_0 as usize].F;
        axis_0 += 1
    }
    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if !pidStabilisationEnabled || gyroOverflowDetected() as libc::c_int != 0
       {
        let mut axis_1: libc::c_int = FD_ROLL as libc::c_int;
        while axis_1 <= FD_YAW as libc::c_int {
            pidData[axis_1 as usize].P = 0 as libc::c_int as libc::c_float;
            pidData[axis_1 as usize].I = 0 as libc::c_int as libc::c_float;
            pidData[axis_1 as usize].D = 0 as libc::c_int as libc::c_float;
            pidData[axis_1 as usize].F = 0 as libc::c_int as libc::c_float;
            pidData[axis_1 as usize].Sum = 0 as libc::c_int as libc::c_float;
            axis_1 += 1
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn crashRecoveryModeActive() -> bool {
    return inCrashRecoveryMode;
}
#[no_mangle]
pub unsafe extern "C" fn pidSetAcroTrainerState(mut newState: bool) {
    if acroTrainerActive as libc::c_int != newState as libc::c_int {
        if newState { pidAcroTrainerInit(); }
        acroTrainerActive = newState
    };
}
// USE_ACRO_TRAINER
#[no_mangle]
pub unsafe extern "C" fn pidSetAntiGravityState(mut newState: bool) {
    if newState as libc::c_int != antiGravityEnabled as libc::c_int {
        // reset the accelerator on state changes
        itermAccelerator = 1.0f32
    }
    antiGravityEnabled = newState;
}
#[no_mangle]
pub unsafe extern "C" fn pidAntiGravityEnabled() -> bool {
    return antiGravityEnabled;
}
unsafe extern "C" fn run_static_initializers() {
    pidProfiles_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (14 as libc::c_int |
                                      (5 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<pidProfile_t>() as
                                       libc::c_ulong).wrapping_mul(3 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut pidProfiles_SystemArray as
                                     *mut [pidProfile_t; 3] as *mut uint8_t,
                             copy:
                                 &mut pidProfiles_CopyArray as
                                     *mut [pidProfile_t; 3] as *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_3{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut pidProfile_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_pidProfiles
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut pidProfile_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
