use ::libc;
extern "C" {
    #[no_mangle]
    fn sqrtf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn pt1FilterApply(filter: *mut pt1Filter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn scaleRange(x: libc::c_int, srcFrom: libc::c_int, srcTo: libc::c_int,
                  destFrom: libc::c_int, destTo: libc::c_int) -> libc::c_int;
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
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    // function pointer used to encode a digital motor value into the DMA buffer representation
    #[no_mangle]
    fn pwmAreMotorsEnabled() -> bool;
    #[no_mangle]
    fn pwmCompleteMotorUpdate(motorCount_0: uint8_t);
    #[no_mangle]
    fn isMotorProtocolDshot() -> bool;
    #[no_mangle]
    fn pwmWriteMotor(index: uint8_t, value: libc::c_float);
    #[no_mangle]
    fn pwmShutdownPulsesForAllMotors(motorCount_0: uint8_t);
    #[no_mangle]
    fn timerioTagGetByUsage(usageFlag: timerUsageFlag_e, index: uint8_t)
     -> ioTag_t;
    #[no_mangle]
    static mut hardwareMotorType: uint8_t;
    #[no_mangle]
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    static mut currentControlRateProfile: *mut controlRateConfig_t;
    // (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    #[no_mangle]
    static mut flight3DConfig_System: flight3DConfig_t;
    #[no_mangle]
    fn isAirmodeActive() -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn isFlipOverAfterCrashMode() -> bool;
    #[no_mangle]
    fn getRcDeflection(axis: libc::c_int) -> libc::c_float;
    #[no_mangle]
    fn getRcDeflectionAbs(axis: libc::c_int) -> libc::c_float;
    #[no_mangle]
    fn isMotorsReversed() -> bool;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
    #[no_mangle]
    fn mixerTricopterIsServoSaturated(errorRate: libc::c_float) -> bool;
    #[no_mangle]
    fn mixerTricopterInit();
    #[no_mangle]
    fn mixerTricopterMotorCorrection(motor_0: libc::c_int) -> libc::c_float;
    #[no_mangle]
    static mut pidData: [pidAxisData_t; 3];
    #[no_mangle]
    static mut throttleBoost: libc::c_float;
    #[no_mangle]
    static mut throttleLpf: pt1Filter_t;
    #[no_mangle]
    fn pidResetITerm();
    #[no_mangle]
    fn pidUpdateAntiGravityThrottleFilter(throttle_0: libc::c_float);
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn calculateVbatPidCompensation() -> libc::c_float;
    #[no_mangle]
    fn gyroYawSpinDetected() -> bool;
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
pub const FD_YAW: C2RustUnnamed = 2;
pub const FD_PITCH: C2RustUnnamed = 1;
pub const FD_ROLL: C2RustUnnamed = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pt1Filter_s {
    pub state: libc::c_float,
    pub k: libc::c_float,
}
pub type pt1Filter_t = pt1Filter_s;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_0 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_0 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_0 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_0 = 4095;
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
    pub reset: C2RustUnnamed_1,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
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
pub type rxConfig_t = rxConfig_s;
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const PWM_TYPE_MAX: C2RustUnnamed_3 = 10;
pub const PWM_TYPE_PROSHOT1000: C2RustUnnamed_3 = 9;
pub const PWM_TYPE_DSHOT1200: C2RustUnnamed_3 = 8;
pub const PWM_TYPE_DSHOT600: C2RustUnnamed_3 = 7;
pub const PWM_TYPE_DSHOT300: C2RustUnnamed_3 = 6;
pub const PWM_TYPE_DSHOT150: C2RustUnnamed_3 = 5;
pub const PWM_TYPE_BRUSHED: C2RustUnnamed_3 = 4;
pub const PWM_TYPE_MULTISHOT: C2RustUnnamed_3 = 3;
pub const PWM_TYPE_ONESHOT42: C2RustUnnamed_3 = 2;
pub const PWM_TYPE_ONESHOT125: C2RustUnnamed_3 = 1;
pub const PWM_TYPE_STANDARD: C2RustUnnamed_3 = 0;
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
//CAVEAT: This is used in the `motorConfig_t` parameter group, so the parameter group constraints apply
pub type motorDevConfig_t = motorDevConfig_s;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const MOTOR_BRUSHLESS: C2RustUnnamed_4 = 2;
pub const MOTOR_BRUSHED: C2RustUnnamed_4 = 1;
pub const MOTOR_UNKNOWN: C2RustUnnamed_4 = 0;
// The update rate of motor outputs (50-498Hz)
// Pwm Protocol
// Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
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
pub const THROTTLE_LIMIT_TYPE_CLIP: C2RustUnnamed_5 = 2;
pub const THROTTLE_LIMIT_TYPE_SCALE: C2RustUnnamed_5 = 1;
pub const THROTTLE_LIMIT_TYPE_OFF: C2RustUnnamed_5 = 0;
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
pub type controlRateConfig_t = controlRateConfig_s;
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
pub type C2RustUnnamed_6 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_6 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_6 = 2;
pub const ARMED: C2RustUnnamed_6 = 1;
pub type mixerMode = libc::c_uint;
pub const MIXER_QUADX_1234: mixerMode = 26;
pub const MIXER_CUSTOM_TRI: mixerMode = 25;
pub const MIXER_CUSTOM_AIRPLANE: mixerMode = 24;
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
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// min 3d value
// max 3d value
// center 3d value
// default throttle deadband from MIDRC
// pwm output value for max negative thrust
// pwm output value for max positive thrust
// enable '3D Switched Mode'
// Note: this is called MultiType/MULTITYPE_* in baseflight.
pub type mixerMode_e = mixerMode;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorMixer_s {
    pub throttle: libc::c_float,
    pub roll: libc::c_float,
    pub pitch: libc::c_float,
    pub yaw: libc::c_float,
}
// airplane / singlecopter / dualcopter (not yet properly supported)
// PPM -> servo relay
// Custom mixer data per motor
pub type motorMixer_t = motorMixer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixer_s {
    pub motorCount: uint8_t,
    pub useServo: uint8_t,
    pub motor: *const motorMixer_t,
}
// Custom mixer configuration
pub type mixer_t = mixer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
pub type mixerConfig_t = mixerConfig_s;
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
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn constrainf(mut amt: libc::c_float,
                                mut low: libc::c_float,
                                mut high: libc::c_float) -> libc::c_float {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn flight3DConfig() -> *const flight3DConfig_t {
    return &mut flight3DConfig_System;
}
#[inline]
unsafe extern "C" fn flight3DConfigMutable() -> *mut flight3DConfig_t {
    return &mut flight3DConfig_System;
}
#[inline]
unsafe extern "C" fn customMotorMixer(mut _index: libc::c_int)
 -> *const motorMixer_t {
    return &mut *customMotorMixer_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut motorMixer_t;
}
#[inline]
unsafe extern "C" fn mixerConfig() -> *const mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
}
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
#[no_mangle]
pub static mut mixerConfig_Copy: mixerConfig_t =
    mixerConfig_t{mixerMode: 0,
                  yaw_motors_reversed: false,
                  crashflip_motor_percent: 0,};
#[no_mangle]
pub static mut mixerConfig_System: mixerConfig_t =
    mixerConfig_t{mixerMode: 0,
                  yaw_motors_reversed: false,
                  crashflip_motor_percent: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut mixerConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (20 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<mixerConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mixerConfig_System as *const mixerConfig_t
                                     as *mut mixerConfig_t as *mut uint8_t,
                             copy:
                                 &mixerConfig_Copy as *const mixerConfig_t as
                                     *mut mixerConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_mixerConfig
                                                         as
                                                         *const mixerConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_mixerConfig: mixerConfig_t =
    {
        let mut init =
            mixerConfig_s{mixerMode: MIXER_QUADX as libc::c_int as uint8_t,
                          yaw_motors_reversed: 0 as libc::c_int != 0,
                          crashflip_motor_percent:
                              0 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub static mut motorConfig_Copy: motorConfig_t =
    motorConfig_t{dev:
                      motorDevConfig_t{motorPwmRate: 0,
                                       motorPwmProtocol: 0,
                                       motorPwmInversion: 0,
                                       useUnsyncedPwm: 0,
                                       useBurstDshot: 0,
                                       ioTags: [0; 8],},
                  digitalIdleOffsetValue: 0,
                  minthrottle: 0,
                  maxthrottle: 0,
                  mincommand: 0,
                  motorPoleCount: 0,};
#[no_mangle]
pub static mut motorConfig_System: motorConfig_t =
    motorConfig_t{dev:
                      motorDevConfig_t{motorPwmRate: 0,
                                       motorPwmProtocol: 0,
                                       motorPwmInversion: 0,
                                       useUnsyncedPwm: 0,
                                       useBurstDshot: 0,
                                       ioTags: [0; 8],},
                  digitalIdleOffsetValue: 0,
                  minthrottle: 0,
                  maxthrottle: 0,
                  mincommand: 0,
                  motorPoleCount: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut motorConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (6 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<motorConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &motorConfig_System as *const motorConfig_t
                                     as *mut motorConfig_t as *mut uint8_t,
                             copy:
                                 &motorConfig_Copy as *const motorConfig_t as
                                     *mut motorConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut motorConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_motorConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut motorConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_motorConfig(mut motorConfig_0:
                                                   *mut motorConfig_t) {
    if hardwareMotorType as libc::c_int == MOTOR_BRUSHED as libc::c_int {
        (*motorConfig_0).minthrottle = 1000 as libc::c_int as uint16_t;
        (*motorConfig_0).dev.motorPwmRate = 16000 as libc::c_int as uint16_t;
        (*motorConfig_0).dev.motorPwmProtocol =
            PWM_TYPE_BRUSHED as libc::c_int as uint8_t;
        (*motorConfig_0).dev.useUnsyncedPwm = 1 as libc::c_int as uint8_t
    } else {
        (*motorConfig_0).minthrottle = 1070 as libc::c_int as uint16_t;
        (*motorConfig_0).dev.motorPwmRate = 480 as libc::c_int as uint16_t;
        (*motorConfig_0).dev.motorPwmProtocol =
            PWM_TYPE_ONESHOT125 as libc::c_int as uint8_t
    }
    (*motorConfig_0).maxthrottle = 2000 as libc::c_int as uint16_t;
    (*motorConfig_0).mincommand = 1000 as libc::c_int as uint16_t;
    (*motorConfig_0).digitalIdleOffsetValue = 450 as libc::c_int as uint16_t;
    (*motorConfig_0).dev.useBurstDshot = 0 as libc::c_int as uint8_t;
    let mut motorIndex: libc::c_int = 0 as libc::c_int;
    while motorIndex < 8 as libc::c_int {
        (*motorConfig_0).dev.ioTags[motorIndex as usize] =
            timerioTagGetByUsage(TIM_USE_MOTOR, motorIndex as uint8_t);
        motorIndex += 1
    }
    (*motorConfig_0).motorPoleCount = 14 as libc::c_int as uint8_t;
    // Most brushes motors that we use are 14 poles
}
#[no_mangle]
pub static mut customMotorMixer_CopyArray: [motorMixer_t; 8] =
    [motorMixer_t{throttle: 0., roll: 0., pitch: 0., yaw: 0.,}; 8];
#[no_mangle]
pub static mut customMotorMixer_SystemArray: [motorMixer_t; 8] =
    [motorMixer_t{throttle: 0., roll: 0., pitch: 0., yaw: 0.,}; 8];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut customMotorMixer_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_1{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
static mut motorCount: uint8_t = 0;
static mut motorMixRange: libc::c_float = 0.;
#[no_mangle]
pub static mut motor: [libc::c_float; 8] = [0.; 8];
#[no_mangle]
pub static mut motor_disarmed: [libc::c_float; 8] = [0.; 8];
#[no_mangle]
pub static mut currentMixerMode: mixerMode_e = 0 as mixerMode_e;
static mut currentMixer: [motorMixer_t; 8] =
    [motorMixer_t{throttle: 0., roll: 0., pitch: 0., yaw: 0.,}; 8];
static mut throttleAngleCorrection: libc::c_int = 0;
static mut mixerQuadX: [motorMixer_t; 4] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -1.0f32,
                          pitch: 1.0f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -1.0f32,
                          pitch: -1.0f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 1.0f32,
                          pitch: 1.0f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 1.0f32,
                          pitch: -1.0f32,
                          yaw: -1.0f32,};
         init
     }];
static mut mixerTricopter: [motorMixer_t; 3] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.0f32,
                          pitch: 1.333333f32,
                          yaw: 0.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -1.0f32,
                          pitch: -0.666667f32,
                          yaw: 0.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 1.0f32,
                          pitch: -0.666667f32,
                          yaw: 0.0f32,};
         init
     }];
static mut mixerQuadP: [motorMixer_t; 4] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.0f32,
                          pitch: 1.0f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -1.0f32,
                          pitch: 0.0f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 1.0f32,
                          pitch: 0.0f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.0f32,
                          pitch: -1.0f32,
                          yaw: -1.0f32,};
         init
     }];
static mut mixerY4: [motorMixer_t; 4] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.0f32,
                          pitch: 1.0f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -1.0f32,
                          pitch: -1.0f32,
                          yaw: 0.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.0f32,
                          pitch: 1.0f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 1.0f32,
                          pitch: -1.0f32,
                          yaw: 0.0f32,};
         init
     }];
static mut mixerHex6X: [motorMixer_t; 6] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -0.5f32,
                          pitch: 0.866025f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -0.5f32,
                          pitch: -0.866025f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.5f32,
                          pitch: 0.866025f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.5f32,
                          pitch: -0.866025f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -1.0f32,
                          pitch: 0.0f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 1.0f32,
                          pitch: 0.0f32,
                          yaw: 1.0f32,};
         init
     }];
static mut mixerVtail4: [motorMixer_t; 4] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -0.58f32,
                          pitch: 0.58f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -0.46f32,
                          pitch: -0.39f32,
                          yaw: -0.5f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.58f32,
                          pitch: 0.58f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.46f32,
                          pitch: -0.39f32,
                          yaw: 0.5f32,};
         init
     }];
static mut mixerAtail4: [motorMixer_t; 4] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -0.58f32,
                          pitch: 0.58f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -0.46f32,
                          pitch: -0.39f32,
                          yaw: 0.5f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.58f32,
                          pitch: 0.58f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.46f32,
                          pitch: -0.39f32,
                          yaw: -0.5f32,};
         init
     }];
static mut mixerSingleProp: [motorMixer_t; 1] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 0.0f32,
                          pitch: 0.0f32,
                          yaw: 0.0f32,};
         init
     }];
static mut mixerQuadX1234: [motorMixer_t; 4] =
    [{
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 1.0f32,
                          pitch: -1.0f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -1.0f32,
                          pitch: -1.0f32,
                          yaw: 1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: -1.0f32,
                          pitch: 1.0f32,
                          yaw: -1.0f32,};
         init
     },
     {
         let mut init =
             motorMixer_s{throttle: 1.0f32,
                          roll: 1.0f32,
                          pitch: 1.0f32,
                          yaw: 1.0f32,};
         init
     }];
// Keep synced with mixerMode_e
#[no_mangle]
pub static mut mixers: [mixer_t; 27] =
    unsafe {
        [{
             let mut init =
                 mixer_s{motorCount: 0 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 3 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: mixerTricopter.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 4 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: mixerQuadP.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 4 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: mixerQuadX.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 2 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 0 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 6 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 6 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 1 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: mixerSingleProp.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 4 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: mixerY4.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 6 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: mixerHex6X.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 8 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 8 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 8 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 1 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: mixerSingleProp.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 1 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: mixerSingleProp.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 0 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 4 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: mixerVtail4.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 6 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 0 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 2 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 1 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 4 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: mixerAtail4.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 0 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 2 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 3 as libc::c_int as uint8_t,
                         useServo: 1 as libc::c_int as uint8_t,
                         motor: 0 as *const motorMixer_t,};
             init
         },
         {
             let mut init =
                 mixer_s{motorCount: 4 as libc::c_int as uint8_t,
                         useServo: 0 as libc::c_int as uint8_t,
                         motor: mixerQuadX1234.as_ptr(),};
             init
         }]
    };
// !USE_QUAD_MIXER_ONLY
#[no_mangle]
pub static mut motorOutputHigh: libc::c_float = 0.;
#[no_mangle]
pub static mut motorOutputLow: libc::c_float = 0.;
static mut disarmMotorOutput: libc::c_float = 0.;
static mut deadbandMotor3dHigh: libc::c_float = 0.;
static mut deadbandMotor3dLow: libc::c_float = 0.;
static mut rcCommand3dDeadBandLow: uint16_t = 0;
static mut rcCommand3dDeadBandHigh: uint16_t = 0;
static mut rcCommandThrottleRange: libc::c_float = 0.;
static mut rcCommandThrottleRange3dLow: libc::c_float = 0.;
static mut rcCommandThrottleRange3dHigh: libc::c_float = 0.;
#[no_mangle]
pub unsafe extern "C" fn getMotorCount() -> uint8_t { return motorCount; }
#[no_mangle]
pub unsafe extern "C" fn getMotorMixRange() -> libc::c_float {
    return motorMixRange;
}
#[no_mangle]
pub unsafe extern "C" fn areMotorsRunning() -> bool {
    let mut motorsRunning: bool = 0 as libc::c_int != 0;
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        motorsRunning = 1 as libc::c_int != 0
    } else {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < motorCount as libc::c_int {
            if motor_disarmed[i as usize] != disarmMotorOutput {
                motorsRunning = 1 as libc::c_int != 0;
                break ;
            } else { i += 1 }
        }
    }
    return motorsRunning;
}
#[no_mangle]
pub unsafe extern "C" fn mixerIsTricopter() -> bool {
    return currentMixerMode as libc::c_uint ==
               MIXER_TRI as libc::c_int as libc::c_uint ||
               currentMixerMode as libc::c_uint ==
                   MIXER_CUSTOM_TRI as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn mixerIsOutputSaturated(mut axis: libc::c_int,
                                                mut errorRate: libc::c_float)
 -> bool {
    if axis == FD_YAW as libc::c_int && mixerIsTricopter() as libc::c_int != 0
       {
        return mixerTricopterIsServoSaturated(errorRate)
    }
    return motorMixRange >= 1.0f32;
}
// All PWM motor scaling is done to standard PWM range of 1000-2000 for easier tick conversion with legacy code / configurator
// DSHOT scaling is done to the actual dshot range
#[no_mangle]
pub unsafe extern "C" fn initEscEndpoints() {
    // Can't use 'isMotorProtocolDshot()' here since motors haven't been initialised yet
    match (*motorConfig()).dev.motorPwmProtocol as libc::c_int {
        9 | 8 | 7 | 6 | 5 => {
            disarmMotorOutput = 0 as libc::c_int as libc::c_float;
            if feature(FEATURE_3D as libc::c_int as uint32_t) {
                motorOutputLow =
                    48 as libc::c_int as libc::c_float +
                        (1047 as libc::c_int - 48 as libc::c_int) as
                            libc::c_float / 100.0f32 *
                            (0.01f32 *
                                 (*motorConfig()).digitalIdleOffsetValue as
                                     libc::c_int as libc::c_float)
            } else {
                motorOutputLow =
                    48 as libc::c_int as libc::c_float +
                        (2047 as libc::c_int - 48 as libc::c_int) as
                            libc::c_float / 100.0f32 *
                            (0.01f32 *
                                 (*motorConfig()).digitalIdleOffsetValue as
                                     libc::c_int as libc::c_float)
            }
            motorOutputHigh = 2047 as libc::c_int as libc::c_float;
            deadbandMotor3dHigh =
                1048 as libc::c_int as libc::c_float +
                    (2047 as libc::c_int - 1048 as libc::c_int) as
                        libc::c_float / 100.0f32 *
                        (0.01f32 *
                             (*motorConfig()).digitalIdleOffsetValue as
                                 libc::c_int as libc::c_float);
            deadbandMotor3dLow = 1047 as libc::c_int as libc::c_float
        }
        _ => {
            if feature(FEATURE_3D as libc::c_int as uint32_t) {
                disarmMotorOutput =
                    (*flight3DConfig()).neutral3d as libc::c_float;
                motorOutputLow =
                    (*flight3DConfig()).limit3d_low as libc::c_float;
                motorOutputHigh =
                    (*flight3DConfig()).limit3d_high as libc::c_float;
                deadbandMotor3dHigh =
                    (*flight3DConfig()).deadband3d_high as libc::c_float;
                deadbandMotor3dLow =
                    (*flight3DConfig()).deadband3d_low as libc::c_float
            } else {
                disarmMotorOutput =
                    (*motorConfig()).mincommand as libc::c_float;
                motorOutputLow =
                    (*motorConfig()).minthrottle as libc::c_float;
                motorOutputHigh =
                    (*motorConfig()).maxthrottle as libc::c_float
            }
        }
    }
    rcCommandThrottleRange =
        (2000 as libc::c_int - (*rxConfig()).mincheck as libc::c_int) as
            libc::c_float;
    rcCommand3dDeadBandLow =
        ((*rxConfig()).midrc as libc::c_int -
             (*flight3DConfig()).deadband3d_throttle as libc::c_int) as
            uint16_t;
    rcCommand3dDeadBandHigh =
        ((*rxConfig()).midrc as libc::c_int +
             (*flight3DConfig()).deadband3d_throttle as libc::c_int) as
            uint16_t;
    rcCommandThrottleRange3dLow =
        (rcCommand3dDeadBandLow as libc::c_int - 1000 as libc::c_int) as
            libc::c_float;
    rcCommandThrottleRange3dHigh =
        (2000 as libc::c_int - rcCommand3dDeadBandHigh as libc::c_int) as
            libc::c_float;
}
#[no_mangle]
pub unsafe extern "C" fn mixerInit(mut mixerMode: mixerMode_e) {
    currentMixerMode = mixerMode;
    initEscEndpoints();
    if mixerIsTricopter() { mixerTricopterInit(); };
}
#[no_mangle]
pub unsafe extern "C" fn mixerConfigureOutput() {
    motorCount = 0 as libc::c_int as uint8_t;
    if currentMixerMode as libc::c_uint ==
           MIXER_CUSTOM as libc::c_int as libc::c_uint ||
           currentMixerMode as libc::c_uint ==
               MIXER_CUSTOM_TRI as libc::c_int as libc::c_uint ||
           currentMixerMode as libc::c_uint ==
               MIXER_CUSTOM_AIRPLANE as libc::c_int as libc::c_uint {
        // load custom mixer into currentMixer
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < 8 as libc::c_int {
            // check if done
            if (*customMotorMixer(i)).throttle == 0.0f32 { break ; }
            currentMixer[i as usize] = *customMotorMixer(i);
            motorCount = motorCount.wrapping_add(1);
            i += 1
        }
    } else {
        motorCount = mixers[currentMixerMode as usize].motorCount;
        if motorCount as libc::c_int > 8 as libc::c_int {
            motorCount = 8 as libc::c_int as uint8_t
        }
        // copy motor-based mixers
        if !mixers[currentMixerMode as usize].motor.is_null() {
            let mut i_0: libc::c_int = 0 as libc::c_int;
            while i_0 < motorCount as libc::c_int {
                currentMixer[i_0 as usize] =
                    *mixers[currentMixerMode as
                                usize].motor.offset(i_0 as isize);
                i_0 += 1
            }
        }
    }
    mixerResetDisarmedMotors();
}
#[no_mangle]
pub unsafe extern "C" fn mixerLoadMix(mut index: libc::c_int,
                                      mut customMixers: *mut motorMixer_t) {
    // we're 1-based
    index += 1;
    // clear existing
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        (*customMixers.offset(i as isize)).throttle = 0.0f32;
        i += 1
    }
    // do we have anything here to begin with?
    if !mixers[index as usize].motor.is_null() {
        let mut i_0: libc::c_int = 0 as libc::c_int;
        while i_0 < mixers[index as usize].motorCount as libc::c_int {
            *customMixers.offset(i_0 as isize) =
                *mixers[index as usize].motor.offset(i_0 as isize);
            i_0 += 1
        }
    };
}
// USE_QUAD_MIXER_ONLY
#[no_mangle]
pub unsafe extern "C" fn mixerResetDisarmedMotors() {
    // set disarmed motor values
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        motor_disarmed[i as usize] = disarmMotorOutput;
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn writeMotors() {
    if pwmAreMotorsEnabled() {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < motorCount as libc::c_int {
            pwmWriteMotor(i as uint8_t, motor[i as usize]);
            i += 1
        }
        pwmCompleteMotorUpdate(motorCount);
    };
}
unsafe extern "C" fn writeAllMotors(mut mc: int16_t) {
    // Sends commands to all motors
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < motorCount as libc::c_int {
        motor[i as usize] = mc as libc::c_float;
        i += 1
    }
    writeMotors();
}
#[no_mangle]
pub unsafe extern "C" fn stopMotors() {
    writeAllMotors(disarmMotorOutput as int16_t);
    delay(50 as libc::c_int as timeMs_t);
    // give the timers and ESCs a chance to react.
}
#[no_mangle]
pub unsafe extern "C" fn stopPwmAllMotors() {
    pwmShutdownPulsesForAllMotors(motorCount); // Store the last throttle direction for deadband transitions
    delayMicroseconds(1500 as libc::c_int as
                          timeUs_t); // time when motors last reversed in 3D mode
}
static mut throttle: libc::c_float = 0 as libc::c_int as libc::c_float;
static mut motorOutputMin: libc::c_float = 0.;
static mut motorRangeMin: libc::c_float = 0.;
static mut motorRangeMax: libc::c_float = 0.;
static mut motorOutputRange: libc::c_float = 0.;
static mut motorOutputMixSign: int8_t = 0;
unsafe extern "C" fn calculateThrottleAndCurrentMotorEndpoints(mut currentTimeUs:
                                                                   timeUs_t) {
    static mut rcThrottlePrevious: uint16_t = 0 as libc::c_int as uint16_t;
    static mut reversalTimeUs: timeUs_t = 0 as libc::c_int as timeUs_t;
    let mut currentThrottleInputRange: libc::c_float =
        0 as libc::c_int as libc::c_float;
    if feature(FEATURE_3D as libc::c_int as uint32_t) {
        if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
            rcThrottlePrevious = (*rxConfig()).midrc
            // When disarmed set to mid_rc. It always results in positive direction after arming.
        }
        if rcCommand[THROTTLE as libc::c_int as usize] <=
               rcCommand3dDeadBandLow as libc::c_int as libc::c_float {
            // INVERTED
            motorRangeMin = motorOutputLow;
            motorRangeMax = deadbandMotor3dLow;
            if isMotorProtocolDshot() {
                motorOutputMin = motorOutputLow;
                motorOutputRange = deadbandMotor3dLow - motorOutputLow
            } else {
                motorOutputMin = deadbandMotor3dLow;
                motorOutputRange = motorOutputLow - deadbandMotor3dLow
            }
            if motorOutputMixSign as libc::c_int != -(1 as libc::c_int) {
                reversalTimeUs = currentTimeUs
            }
            motorOutputMixSign = -(1 as libc::c_int) as int8_t;
            rcThrottlePrevious =
                rcCommand[THROTTLE as libc::c_int as usize] as uint16_t;
            throttle =
                rcCommand3dDeadBandLow as libc::c_int as libc::c_float -
                    rcCommand[THROTTLE as libc::c_int as usize];
            currentThrottleInputRange = rcCommandThrottleRange3dLow
        } else if rcCommand[THROTTLE as libc::c_int as usize] >=
                      rcCommand3dDeadBandHigh as libc::c_int as libc::c_float
         {
            // NORMAL
            motorRangeMin = deadbandMotor3dHigh;
            motorRangeMax = motorOutputHigh;
            motorOutputMin = deadbandMotor3dHigh;
            motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
            if motorOutputMixSign as libc::c_int != 1 as libc::c_int {
                reversalTimeUs = currentTimeUs
            }
            motorOutputMixSign = 1 as libc::c_int as int8_t;
            rcThrottlePrevious =
                rcCommand[THROTTLE as libc::c_int as usize] as uint16_t;
            throttle =
                rcCommand[THROTTLE as libc::c_int as usize] -
                    rcCommand3dDeadBandHigh as libc::c_int as libc::c_float;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh
        } else if rcThrottlePrevious as libc::c_int <=
                      rcCommand3dDeadBandLow as libc::c_int &&
                      (*flight3DConfigMutable()).switched_mode3d == 0 ||
                      isMotorsReversed() as libc::c_int != 0 {
            // INVERTED_TO_DEADBAND
            motorRangeMin = motorOutputLow;
            motorRangeMax = deadbandMotor3dLow;
            if isMotorProtocolDshot() {
                motorOutputMin = motorOutputLow;
                motorOutputRange = deadbandMotor3dLow - motorOutputLow
            } else {
                motorOutputMin = deadbandMotor3dLow;
                motorOutputRange = motorOutputLow - deadbandMotor3dLow
            }
            if motorOutputMixSign as libc::c_int != -(1 as libc::c_int) {
                reversalTimeUs = currentTimeUs
            }
            motorOutputMixSign = -(1 as libc::c_int) as int8_t;
            throttle = 0 as libc::c_int as libc::c_float;
            currentThrottleInputRange = rcCommandThrottleRange3dLow
        } else {
            // NORMAL_TO_DEADBAND
            motorRangeMin = deadbandMotor3dHigh;
            motorRangeMax = motorOutputHigh;
            motorOutputMin = deadbandMotor3dHigh;
            motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
            if motorOutputMixSign as libc::c_int != 1 as libc::c_int {
                reversalTimeUs = currentTimeUs
            }
            motorOutputMixSign = 1 as libc::c_int as int8_t;
            throttle = 0 as libc::c_int as libc::c_float;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh
        }
        if currentTimeUs.wrapping_sub(reversalTimeUs) <
               250000 as libc::c_int as libc::c_uint {
            // keep ITerm zero for 250ms after motor reversal
            pidResetITerm();
        }
    } else {
        throttle =
            rcCommand[THROTTLE as libc::c_int as usize] -
                (*rxConfig()).mincheck as libc::c_int as libc::c_float +
                throttleAngleCorrection as libc::c_float;
        currentThrottleInputRange = rcCommandThrottleRange;
        motorRangeMin = motorOutputLow;
        motorRangeMax = motorOutputHigh;
        motorOutputMin = motorOutputLow;
        motorOutputRange = motorOutputHigh - motorOutputLow;
        motorOutputMixSign = 1 as libc::c_int as int8_t
    }
    throttle =
        constrainf(throttle / currentThrottleInputRange, 0.0f32, 1.0f32);
}
unsafe extern "C" fn applyFlipOverAfterCrashModeToMotors() {
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        let mut stickDeflectionPitchAbs: libc::c_float =
            getRcDeflectionAbs(FD_PITCH as libc::c_int);
        let mut stickDeflectionRollAbs: libc::c_float =
            getRcDeflectionAbs(FD_ROLL as libc::c_int);
        let mut stickDeflectionYawAbs: libc::c_float =
            getRcDeflectionAbs(FD_YAW as libc::c_int);
        let mut signPitch: libc::c_float =
            if getRcDeflection(FD_PITCH as libc::c_int) <
                   0 as libc::c_int as libc::c_float {
                1 as libc::c_int
            } else { -(1 as libc::c_int) } as libc::c_float;
        let mut signRoll: libc::c_float =
            if getRcDeflection(FD_ROLL as libc::c_int) <
                   0 as libc::c_int as libc::c_float {
                1 as libc::c_int
            } else { -(1 as libc::c_int) } as libc::c_float;
        let mut signYaw: libc::c_float =
            ((if getRcDeflection(FD_YAW as libc::c_int) <
                     0 as libc::c_int as libc::c_float {
                  1 as libc::c_int
              } else { -(1 as libc::c_int) }) *
                 (if (*mixerConfig()).yaw_motors_reversed as libc::c_int != 0
                     {
                      1 as libc::c_int
                  } else { -(1 as libc::c_int) })) as libc::c_float;
        let mut stickDeflectionLength: libc::c_float =
            sqrtf(stickDeflectionPitchAbs * stickDeflectionPitchAbs +
                      stickDeflectionRollAbs * stickDeflectionRollAbs);
        if stickDeflectionYawAbs >
               ({
                    let mut _a: libc::c_float = stickDeflectionPitchAbs;
                    let mut _b: libc::c_float = stickDeflectionRollAbs;
                    (if _a > _b { _a } else { _b })
                }) {
            // If yaw is the dominant, disable pitch and roll
            stickDeflectionLength = stickDeflectionYawAbs;
            signRoll = 0 as libc::c_int as libc::c_float;
            signPitch = 0 as libc::c_int as libc::c_float
        } else {
            // If pitch/roll dominant, disable yaw
            signYaw = 0 as libc::c_int as libc::c_float
        } // cos(PI/6.0f)
        let mut cosPhi: libc::c_float =
            (stickDeflectionPitchAbs + stickDeflectionRollAbs) /
                (sqrtf(2.0f32) * stickDeflectionLength);
        let cosThreshold: libc::c_float = sqrtf(3.0f32) / 2.0f32;
        if cosPhi < cosThreshold {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if stickDeflectionRollAbs > stickDeflectionPitchAbs {
                signPitch = 0 as libc::c_int as libc::c_float
            } else { signRoll = 0 as libc::c_int as libc::c_float }
        }
        // Apply a reasonable amount of stick deadband
        let flipStickRange: libc::c_float = 1.0f32 - 0.15f32;
        let mut flipPower: libc::c_float =
            ({
                 let mut _a: libc::c_float = 0.0f32;
                 let mut _b: libc::c_float = stickDeflectionLength - 0.15f32;
                 (if _a > _b { _a } else { _b })
             }) / flipStickRange;
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < motorCount as libc::c_int {
            let mut motorOutput: libc::c_float =
                signPitch * currentMixer[i as usize].pitch +
                    signRoll * currentMixer[i as usize].roll +
                    signYaw * currentMixer[i as usize].yaw;
            if motorOutput < 0 as libc::c_int as libc::c_float {
                if (*mixerConfig()).crashflip_motor_percent as libc::c_int >
                       0 as libc::c_int {
                    motorOutput =
                        -motorOutput *
                            (*mixerConfig()).crashflip_motor_percent as
                                libc::c_float / 100.0f32
                } else { motorOutput = disarmMotorOutput }
            }
            motorOutput =
                ({
                     let mut _a: libc::c_float = 1.0f32;
                     let mut _b: libc::c_float = flipPower * motorOutput;
                     if _a < _b { _a } else { _b }
                 });
            motorOutput = motorOutputMin + motorOutput * motorOutputRange;
            // Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
            motorOutput =
                if motorOutput <
                       motorOutputMin + 20 as libc::c_int as libc::c_float {
                    disarmMotorOutput
                } else { (motorOutput) - 20 as libc::c_int as libc::c_float };
            motor[i as usize] = motorOutput;
            i += 1
        }
    } else {
        // Disarmed mode
        let mut i_0: libc::c_int = 0 as libc::c_int;
        while i_0 < motorCount as libc::c_int {
            motor[i_0 as usize] = motor_disarmed[i_0 as usize];
            i_0 += 1
        }
    };
}
unsafe extern "C" fn applyMixToMotors(mut motorMix: *mut libc::c_float) {
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < motorCount as libc::c_int {
        let mut motorOutput: libc::c_float =
            motorOutputMin +
                motorOutputRange *
                    (motorOutputMixSign as libc::c_int as libc::c_float *
                         *motorMix.offset(i as isize) +
                         throttle * currentMixer[i as usize].throttle);
        if mixerIsTricopter() {
            motorOutput += mixerTricopterMotorCorrection(i)
        }
        if failsafeIsActive() {
            if isMotorProtocolDshot() {
                motorOutput =
                    if motorOutput < motorRangeMin {
                        disarmMotorOutput
                    } else { motorOutput }
                // Prevent getting into special reserved range
            }
            motorOutput =
                constrain(motorOutput as libc::c_int,
                          disarmMotorOutput as libc::c_int,
                          motorRangeMax as libc::c_int) as libc::c_float
        } else {
            motorOutput =
                constrain(motorOutput as libc::c_int,
                          motorRangeMin as libc::c_int,
                          motorRangeMax as libc::c_int) as libc::c_float
        }
        // Motor stop handling
        if feature(FEATURE_MOTOR_STOP as libc::c_int as uint32_t) as
               libc::c_int != 0 &&
               armingFlags as libc::c_int & ARMED as libc::c_int != 0 &&
               !feature(FEATURE_3D as libc::c_int as uint32_t) &&
               !isAirmodeActive() {
            if (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
                   (*rxConfig()).mincheck as libc::c_int {
                motorOutput = disarmMotorOutput
            }
        }
        motor[i as usize] = motorOutput;
        i += 1
    }
    // Disarmed mode
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        let mut i_0: libc::c_int = 0 as libc::c_int;
        while i_0 < motorCount as libc::c_int {
            motor[i_0 as usize] = motor_disarmed[i_0 as usize];
            i_0 += 1
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn applyThrottleLimit(mut throttle_0: libc::c_float)
 -> libc::c_float {
    if ((*currentControlRateProfile).throttle_limit_percent as libc::c_int) <
           100 as libc::c_int {
        let throttleLimitFactor: libc::c_float =
            (*currentControlRateProfile).throttle_limit_percent as libc::c_int
                as libc::c_float / 100.0f32;
        match (*currentControlRateProfile).throttle_limit_type as libc::c_int
            {
            1 => { return throttle_0 * throttleLimitFactor }
            2 => {
                return ({
                            let mut _a: libc::c_float = throttle_0;
                            let _b: libc::c_float = throttleLimitFactor;
                            if _a < _b { _a } else { _b }
                        })
            }
            _ => { }
        }
    }
    return throttle_0;
}
#[no_mangle]
pub unsafe extern "C" fn mixTable(mut currentTimeUs: timeUs_t,
                                  mut vbatPidCompensation: uint8_t) {
    if isFlipOverAfterCrashMode() {
        applyFlipOverAfterCrashModeToMotors();
        return
    }
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);
    // Calculate and Limit the PID sum
    let scaledAxisPidRoll: libc::c_float =
        constrainf(pidData[FD_ROLL as libc::c_int as usize].Sum,
                   -((*currentPidProfile).pidSumLimit as libc::c_int) as
                       libc::c_float,
                   (*currentPidProfile).pidSumLimit as libc::c_float) /
            1000.0f32;
    let scaledAxisPidPitch: libc::c_float =
        constrainf(pidData[FD_PITCH as libc::c_int as usize].Sum,
                   -((*currentPidProfile).pidSumLimit as libc::c_int) as
                       libc::c_float,
                   (*currentPidProfile).pidSumLimit as libc::c_float) /
            1000.0f32;
    let mut yawPidSumLimit: uint16_t = (*currentPidProfile).pidSumLimitYaw;
    let yawSpinDetected: bool = gyroYawSpinDetected();
    if yawSpinDetected {
        yawPidSumLimit = 1000 as libc::c_int as uint16_t
        // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
    // USE_YAW_SPIN_RECOVERY
    let mut scaledAxisPidYaw: libc::c_float =
        constrainf(pidData[FD_YAW as libc::c_int as usize].Sum,
                   -(yawPidSumLimit as libc::c_int) as libc::c_float,
                   yawPidSumLimit as libc::c_float) / 1000.0f32;
    if !(*mixerConfig()).yaw_motors_reversed {
        scaledAxisPidYaw = -scaledAxisPidYaw
    }
    // Calculate voltage compensation
    let vbatCompensationFactor: libc::c_float =
        if vbatPidCompensation as libc::c_int != 0 {
            calculateVbatPidCompensation()
        } else { 1.0f32 };
    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (*currentControlRateProfile).throttle_limit_type as libc::c_int !=
           THROTTLE_LIMIT_TYPE_OFF as libc::c_int {
        throttle = applyThrottleLimit(throttle)
    }
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if yawSpinDetected as libc::c_int != 0 && !isAirmodeActive() {
        throttle = 0.5f32
        // 
    }
    // USE_YAW_SPIN_RECOVERY
    // Find roll/pitch/yaw desired output
    let mut motorMix: [libc::c_float; 8] =
        [0.; 8]; // Add voltage compensation
    let mut motorMixMax: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut motorMixMin: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < motorCount as libc::c_int {
        let mut mix: libc::c_float =
            scaledAxisPidRoll * currentMixer[i as usize].roll +
                scaledAxisPidPitch * currentMixer[i as usize].pitch +
                scaledAxisPidYaw * currentMixer[i as usize].yaw;
        mix *= vbatCompensationFactor;
        if mix > motorMixMax {
            motorMixMax = mix
        } else if mix < motorMixMin { motorMixMin = mix }
        motorMix[i as usize] = mix;
        i += 1
    }
    pidUpdateAntiGravityThrottleFilter(throttle);
    if throttleBoost > 0.0f32 {
        let throttleHpf: libc::c_float =
            throttle - pt1FilterApply(&mut throttleLpf, throttle);
        throttle =
            constrainf(throttle + throttleBoost * throttleHpf, 0.0f32, 1.0f32)
    }
    motorMixRange = motorMixMax - motorMixMin;
    if motorMixRange > 1.0f32 {
        let mut i_0: libc::c_int = 0 as libc::c_int;
        while i_0 < motorCount as libc::c_int {
            motorMix[i_0 as usize] /= motorMixRange;
            i_0 += 1
        }
        // Get the maximum correction by setting offset to center when airmode enabled
        if isAirmodeActive() { throttle = 0.5f32 }
    } else if isAirmodeActive() as libc::c_int != 0 || throttle > 0.5f32 {
        // Only automatically adjust throttle when airmode enabled. Airmode logic is always active on high throttle
        let throttleLimitOffset: libc::c_float = motorMixRange / 2.0f32;
        throttle =
            constrainf(throttle, 0.0f32 + throttleLimitOffset,
                       1.0f32 - throttleLimitOffset)
    }
    // Apply the mix to motor endpoints
    applyMixToMotors(motorMix.as_mut_ptr());
}
#[no_mangle]
pub unsafe extern "C" fn convertExternalToMotor(mut externalValue: uint16_t)
 -> libc::c_float {
    let mut motorValue: uint16_t = 0;
    match isMotorProtocolDshot() as libc::c_int {
        1 => {
            externalValue =
                constrain(externalValue as libc::c_int, 1000 as libc::c_int,
                          2000 as libc::c_int) as uint16_t;
            if feature(FEATURE_3D as libc::c_int as uint32_t) {
                if externalValue as libc::c_int == 1500 as libc::c_int {
                    motorValue = 0 as libc::c_int as uint16_t
                } else if (externalValue as libc::c_int) < 1500 as libc::c_int
                 {
                    motorValue =
                        scaleRange(externalValue as libc::c_int,
                                   1000 as libc::c_int,
                                   1500 as libc::c_int - 1 as libc::c_int,
                                   1047 as libc::c_int, 48 as libc::c_int) as
                            uint16_t
                } else {
                    motorValue =
                        scaleRange(externalValue as libc::c_int,
                                   1500 as libc::c_int + 1 as libc::c_int,
                                   2000 as libc::c_int, 1048 as libc::c_int,
                                   2047 as libc::c_int) as uint16_t
                }
            } else {
                motorValue =
                    if externalValue as libc::c_int == 1000 as libc::c_int {
                        0 as libc::c_int
                    } else {
                        scaleRange(externalValue as libc::c_int,
                                   1000 as libc::c_int + 1 as libc::c_int,
                                   2000 as libc::c_int, 48 as libc::c_int,
                                   2047 as libc::c_int)
                    } as uint16_t
            }
        }
        0 | _ => { motorValue = externalValue }
    }
    return motorValue as libc::c_float;
}
#[no_mangle]
pub unsafe extern "C" fn convertMotorToExternal(mut motorValue: libc::c_float)
 -> uint16_t {
    let mut externalValue: uint16_t = 0;
    match isMotorProtocolDshot() as libc::c_int {
        1 => {
            if feature(FEATURE_3D as libc::c_int as uint32_t) {
                if motorValue == 0 as libc::c_int as libc::c_float ||
                       motorValue < 48 as libc::c_int as libc::c_float {
                    externalValue = 1500 as libc::c_int as uint16_t
                } else if motorValue <= 1047 as libc::c_int as libc::c_float {
                    externalValue =
                        scaleRange(motorValue as libc::c_int,
                                   48 as libc::c_int, 1047 as libc::c_int,
                                   1500 as libc::c_int - 1 as libc::c_int,
                                   1000 as libc::c_int) as uint16_t
                } else {
                    externalValue =
                        scaleRange(motorValue as libc::c_int,
                                   1048 as libc::c_int, 2047 as libc::c_int,
                                   1500 as libc::c_int + 1 as libc::c_int,
                                   2000 as libc::c_int) as uint16_t
                }
            } else {
                externalValue =
                    if motorValue < 48 as libc::c_int as libc::c_float {
                        1000 as libc::c_int
                    } else {
                        scaleRange(motorValue as libc::c_int,
                                   48 as libc::c_int, 2047 as libc::c_int,
                                   1000 as libc::c_int + 1 as libc::c_int,
                                   2000 as libc::c_int)
                    } as uint16_t
            }
        }
        0 | _ => { externalValue = motorValue as uint16_t }
    }
    return externalValue;
}
#[no_mangle]
pub unsafe extern "C" fn mixerSetThrottleAngleCorrection(mut correctionValue:
                                                             libc::c_int) {
    throttleAngleCorrection = correctionValue;
}
unsafe extern "C" fn run_static_initializers() {
    customMotorMixer_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (4 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<motorMixer_t>() as
                                       libc::c_ulong).wrapping_mul(8 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut customMotorMixer_SystemArray as
                                     *mut [motorMixer_t; 8] as *mut uint8_t,
                             copy:
                                 &mut customMotorMixer_CopyArray as
                                     *mut [motorMixer_t; 8] as *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     0 as
                                                         *mut libc::c_void,},};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
