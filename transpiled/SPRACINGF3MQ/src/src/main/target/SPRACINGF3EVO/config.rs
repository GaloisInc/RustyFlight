use ::libc;
extern "C" {
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut pidProfiles_SystemArray: [pidProfile_t; 3];
    #[no_mangle]
    static mut sdcardConfig_System: sdcardConfig_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const FD_YAW: C2RustUnnamed = 2;
pub const FD_PITCH: C2RustUnnamed = 1;
pub const FD_ROLL: C2RustUnnamed = 0;
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
pub struct motorDevConfig_s {
    pub motorPwmRate: uint16_t,
    pub motorPwmProtocol: uint8_t,
    pub motorPwmInversion: uint8_t,
    pub useUnsyncedPwm: uint8_t,
    pub useBurstDshot: uint8_t,
    pub ioTags: [ioTag_t; 8],
}
pub type motorDevConfig_t = motorDevConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorConfig_s {
    pub dev: motorDevConfig_t,
    pub digitalIdleOffsetValue: uint16_t,
    pub minthrottle: uint16_t,
    pub maxthrottle: uint16_t,
    pub mincommand: uint16_t,
    pub motorPoleCount: uint8_t,
}
pub type motorConfig_t = motorConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidf_s {
    pub P: uint8_t,
    pub I: uint8_t,
    pub D: uint8_t,
    pub F: uint16_t,
}
pub type pidf_t = pidf_s;
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
pub type pidProfile_t = pidProfile_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardConfig_s {
    pub useDma: uint8_t,
    pub enabled: uint8_t,
    pub device: uint8_t,
    pub cardDetectTag: ioTag_t,
    pub chipSelectTag: ioTag_t,
    pub cardDetectInverted: uint8_t,
    pub dmaIdentifier: uint8_t,
    pub dmaChannel: uint8_t,
}
pub type sdcardConfig_t = sdcardConfig_s;
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
unsafe extern "C" fn sdcardConfigMutable() -> *mut sdcardConfig_t {
    return &mut sdcardConfig_System;
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
// 32kHz
#[no_mangle]
pub unsafe extern "C" fn targetConfiguration() {
    // Temporary workaround: Disable SDCard DMA by default since it causes errors on this target
    (*sdcardConfigMutable()).useDma = 0 as libc::c_int as uint8_t;
    (*motorConfigMutable()).dev.motorPwmRate =
        32000 as libc::c_int as uint16_t;
    let mut pidProfileIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (pidProfileIndex as libc::c_int) < 3 as libc::c_int {
        let mut pidProfile: *mut pidProfile_t =
            pidProfilesMutable(pidProfileIndex as libc::c_int);
        (*pidProfile).pid[FD_ROLL as libc::c_int as usize].P =
            90 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_ROLL as libc::c_int as usize].I =
            44 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_ROLL as libc::c_int as usize].D =
            60 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_PITCH as libc::c_int as usize].P =
            90 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_PITCH as libc::c_int as usize].I =
            44 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_PITCH as libc::c_int as usize].D =
            60 as libc::c_int as uint8_t;
        pidProfileIndex = pidProfileIndex.wrapping_add(1)
    };
}
