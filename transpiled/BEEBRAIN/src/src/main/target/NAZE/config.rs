use ::libc;
extern "C" {
    #[no_mangle]
    static mut controlRateProfiles_SystemArray: [controlRateConfig_t; 6];
    #[no_mangle]
    static mut failsafeConfig_System: failsafeConfig_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut pidProfiles_SystemArray: [pidProfile_t; 3];
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
    // gyro alignment
    // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    // Gyro sample divider
    // gyro DLPF setting
    // gyro 32khz DLPF setting
    // Lowpass primary/secondary
    // Gyro calibration duration in 1/100 second
    // bandpass quality factor, 100 for steep sided bandpass
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
    #[no_mangle]
    static mut accelerometerConfig_System: accelerometerConfig_t;
    #[no_mangle]
    static mut compassConfig_System: compassConfig_t;
    #[no_mangle]
    static mut hardwareRevision: uint8_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type int16_t = __int16_t;
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
pub type compassConfig_t = compassConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct compassConfig_s {
    pub mag_declination: int16_t,
    pub mag_align: sensor_align_e,
    pub mag_hardware: uint8_t,
    pub mag_bustype: uint8_t,
    pub mag_i2c_device: uint8_t,
    pub mag_i2c_address: uint8_t,
    pub mag_spi_device: uint8_t,
    pub mag_spi_csn: ioTag_t,
    pub interruptTag: ioTag_t,
    pub magZero: flightDynamicsTrims_t,
}
pub type flightDynamicsTrims_t = flightDynamicsTrims_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union flightDynamicsTrims_u {
    pub raw: [int16_t; 3],
    pub values: flightDynamicsTrims_def_t,
}
pub type flightDynamicsTrims_def_t = int16_flightDynamicsTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct int16_flightDynamicsTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
// Get your magnetic decliniation from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
// mag alignment
// Which mag hardware to use on boards with more than one device
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
pub const NAZE32_REV5: nazeHardwareRevision_t = 2;
pub type controlRateConfig_t = controlRateConfig_s;
#[derive(Copy, Clone)]
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
pub type pidProfile_t = pidProfile_s;
pub const PID_ROLL: C2RustUnnamed_0 = 0;
pub const PID_PITCH: C2RustUnnamed_0 = 1;
pub const PID_LEVEL: C2RustUnnamed_0 = 3;
pub const PID_YAW: C2RustUnnamed_0 = 2;
pub type gyroConfig_t = gyroConfig_s;
#[derive(Copy, Clone)]
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
pub type motorConfig_t = motorConfig_s;
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
pub type motorDevConfig_t = motorDevConfig_s;
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
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
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
// Idle value for DShot protocol, full motor output = 10000
// Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
// This is the maximum value for the ESCs at full power this value can be increased up to 2000
// This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
// Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct failsafeConfig_s {
    pub failsafe_throttle: uint16_t,
    pub failsafe_throttle_low_delay: uint16_t,
    pub failsafe_delay: uint8_t,
    pub failsafe_off_delay: uint8_t,
    pub failsafe_switch_mode: uint8_t,
    pub failsafe_procedure: uint8_t,
}
pub const ACC_NONE: C2RustUnnamed_1 = 1;
pub type accelerometerConfig_t = accelerometerConfig_s;
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
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union rollAndPitchTrims_u {
    pub raw: [int16_t; 2],
    pub values: rollAndPitchTrims_t_def,
}
pub type rollAndPitchTrims_t_def = rollAndPitchTrims_s;
// Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
// Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
// Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
// Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
// failsafe switch action is 0: stage1 (identical to rc link loss), 1: disarms instantly, 2: stage2
// selected full failsafe procedure is 0: auto-landing, 1: Drop it
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
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
}
pub const ACC_ADXL345: C2RustUnnamed_1 = 2;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_0 = 5;
pub const PID_MAG: C2RustUnnamed_0 = 4;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const ACC_FAKE: C2RustUnnamed_1 = 16;
pub const ACC_BMI160: C2RustUnnamed_1 = 15;
pub const ACC_ICM20689: C2RustUnnamed_1 = 14;
pub const ACC_ICM20649: C2RustUnnamed_1 = 13;
pub const ACC_ICM20608G: C2RustUnnamed_1 = 12;
pub const ACC_ICM20602: C2RustUnnamed_1 = 11;
pub const ACC_ICM20601: C2RustUnnamed_1 = 10;
pub const ACC_MPU9250: C2RustUnnamed_1 = 9;
pub const ACC_MPU6500: C2RustUnnamed_1 = 8;
pub const ACC_MPU6000: C2RustUnnamed_1 = 7;
pub const ACC_LSM303DLHC: C2RustUnnamed_1 = 6;
pub const ACC_BMA280: C2RustUnnamed_1 = 5;
pub const ACC_MMA8452: C2RustUnnamed_1 = 4;
pub const ACC_MPU6050: C2RustUnnamed_1 = 3;
pub const ACC_DEFAULT: C2RustUnnamed_1 = 0;
pub type nazeHardwareRevision_t = libc::c_uint;
pub const NAZE32_SP: nazeHardwareRevision_t = 3;
pub const NAZE32: nazeHardwareRevision_t = 1;
pub const UNKNOWN: nazeHardwareRevision_t = 0;
#[inline]
unsafe extern "C" fn controlRateProfilesMutable(mut _index: libc::c_int)
 -> *mut controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
}
#[inline]
unsafe extern "C" fn failsafeConfigMutable() -> *mut failsafeConfig_t {
    return &mut failsafeConfig_System;
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
unsafe extern "C" fn gyroConfigMutable() -> *mut gyroConfig_t {
    return &mut gyroConfig_System;
}
#[inline]
unsafe extern "C" fn accelerometerConfig() -> *const accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn accelerometerConfigMutable()
 -> *mut accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn compassConfigMutable() -> *mut compassConfig_t {
    return &mut compassConfig_System;
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
pub unsafe extern "C" fn targetConfiguration() {
    // alternative defaults settings for Beebrain target
    (*motorConfigMutable()).dev.motorPwmRate =
        4000 as libc::c_int as uint16_t;
    (*failsafeConfigMutable()).failsafe_delay = 2 as libc::c_int as uint8_t;
    (*failsafeConfigMutable()).failsafe_off_delay =
        0 as libc::c_int as uint8_t;
    (*motorConfigMutable()).minthrottle = 1049 as libc::c_int as uint16_t;
    (*gyroConfigMutable()).gyro_hardware_lpf = 2 as libc::c_int as uint8_t;
    (*gyroConfigMutable()).gyro_lowpass_hz = 100 as libc::c_int as uint16_t;
    (*gyroConfigMutable()).gyro_soft_notch_hz_1 =
        0 as libc::c_int as uint16_t;
    (*gyroConfigMutable()).gyro_soft_notch_hz_2 =
        0 as libc::c_int as uint16_t;
    /*for (int channel = 0; channel < NON_AUX_CHANNEL_COUNT; channel++) {
        rxChannelRangeConfigsMutable(channel)->min = 1180;
        rxChannelRangeConfigsMutable(channel)->max = 1860;
    }*/
    let mut pidProfileIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (pidProfileIndex as libc::c_int) < 3 as libc::c_int {
        let mut pidProfile: *mut pidProfile_t =
            pidProfilesMutable(pidProfileIndex as libc::c_int);
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].P =
            60 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].I =
            70 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].D =
            17 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].P =
            80 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].I =
            90 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].D =
            18 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_YAW as libc::c_int as usize].P =
            200 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_YAW as libc::c_int as usize].I =
            45 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].P =
            30 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].D =
            30 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].F =
            200 as libc::c_int as uint16_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].F =
            200 as libc::c_int as uint16_t;
        (*pidProfile).feedForwardTransition = 50 as libc::c_int as uint8_t;
        pidProfileIndex = pidProfileIndex.wrapping_add(1)
    }
    let mut rateProfileIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (rateProfileIndex as libc::c_int) < 6 as libc::c_int {
        let mut controlRateConfig: *mut controlRateConfig_t =
            controlRateProfilesMutable(rateProfileIndex as libc::c_int);
        (*controlRateConfig).rcRates[FD_ROLL as libc::c_int as usize] =
            100 as libc::c_int as uint8_t;
        (*controlRateConfig).rcRates[FD_PITCH as libc::c_int as usize] =
            100 as libc::c_int as uint8_t;
        (*controlRateConfig).rcRates[FD_YAW as libc::c_int as usize] =
            110 as libc::c_int as uint8_t;
        (*controlRateConfig).rcExpo[FD_ROLL as libc::c_int as usize] =
            0 as libc::c_int as uint8_t;
        (*controlRateConfig).rcExpo[FD_PITCH as libc::c_int as usize] =
            0 as libc::c_int as uint8_t;
        (*controlRateConfig).rates[FD_ROLL as libc::c_int as usize] =
            77 as libc::c_int as uint8_t;
        (*controlRateConfig).rates[FD_PITCH as libc::c_int as usize] =
            77 as libc::c_int as uint8_t;
        (*controlRateConfig).rates[FD_YAW as libc::c_int as usize] =
            80 as libc::c_int as uint8_t;
        rateProfileIndex = rateProfileIndex.wrapping_add(1)
    }
    if (hardwareRevision as libc::c_int) < NAZE32_REV5 as libc::c_int {
        (*compassConfigMutable()).interruptTag =
            ((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
                 12 as libc::c_int) as ioTag_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn targetValidateConfiguration() {
    if (hardwareRevision as libc::c_int) < NAZE32_REV5 as libc::c_int &&
           (*accelerometerConfig()).acc_hardware as libc::c_int ==
               ACC_ADXL345 as libc::c_int {
        (*accelerometerConfigMutable()).acc_hardware =
            ACC_NONE as libc::c_int as uint8_t
    };
}
