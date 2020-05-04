use ::libc;
extern "C" {
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
    fn sin_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn cos_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn qMultiply(q: fix12_t, input: int16_t) -> int16_t;
    #[no_mangle]
    fn qConstruct(num: int16_t, den: int16_t) -> fix12_t;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    static mut currentControlRateProfile: *mut controlRateConfig_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut isRXDataNew: bool;
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    // 2nd order Butterworth section
    #[no_mangle]
    fn biquadFilterApplyDF1(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn pt1FilterApply(filter: *mut pt1Filter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn biquadFilterUpdateLPF(filter: *mut biquadFilter_t,
                             filterFreq: libc::c_float,
                             refreshRate: uint32_t);
    #[no_mangle]
    fn biquadFilterInitLPF(filter: *mut biquadFilter_t,
                           filterFreq: libc::c_float, refreshRate: uint32_t);
    #[no_mangle]
    fn pt1FilterGain(f_cut: uint16_t, dT: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn pt1FilterUpdateCutoff(filter: *mut pt1Filter_t, k: libc::c_float);
    #[no_mangle]
    fn pt1FilterInit(filter: *mut pt1Filter_t, k: libc::c_float);
    #[no_mangle]
    static mut rcControlsConfig_System: rcControlsConfig_t;
    #[no_mangle]
    static mut flight3DConfig_System: flight3DConfig_t;
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
    #[no_mangle]
    fn imuQuaternionHeadfreeTransformVectorEarthToBody(v:
                                                           *mut t_fp_vector_def);
    #[no_mangle]
    static mut targetPidLooptime: uint32_t;
    #[no_mangle]
    fn pidSetItermAccelerator(newItermAccelerator: libc::c_float);
    #[no_mangle]
    fn pidInitSetpointDerivativeLpf(filterCutoff: uint16_t,
                                    debugAxis: uint8_t, filterType: uint8_t);
    #[no_mangle]
    fn pidUpdateSetpointDerivativeLpf(filterCutoff: uint16_t);
    #[no_mangle]
    fn pidAntiGravityEnabled() -> bool;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn rxIsReceivingSignal() -> bool;
    #[no_mangle]
    fn rxGetRefreshRate() -> uint16_t;
    #[no_mangle]
    fn getLowVoltageCutoff() -> *const lowVoltageCutoff_t;
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
pub type fix12_t = int32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct fp_vector {
    pub X: libc::c_float,
    pub Y: libc::c_float,
    pub Z: libc::c_float,
}
// Floating point 3 vector.
pub type t_fp_vector_def = fp_vector;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const RATES_TYPE_RACEFLIGHT: C2RustUnnamed_2 = 1;
pub const RATES_TYPE_BETAFLIGHT: C2RustUnnamed_2 = 0;
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
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const INTERPOLATION_CHANNELS_RPT: C2RustUnnamed_3 = 4;
pub const INTERPOLATION_CHANNELS_T: C2RustUnnamed_3 = 3;
pub const INTERPOLATION_CHANNELS_RPYT: C2RustUnnamed_3 = 2;
pub const INTERPOLATION_CHANNELS_RPY: C2RustUnnamed_3 = 1;
pub const INTERPOLATION_CHANNELS_RP: C2RustUnnamed_3 = 0;
pub const ROLL: rc_alias = 0;
pub const YAW: rc_alias = 2;
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
pub type ioTag_t = uint8_t;
pub const HEADFREE_MODE: C2RustUnnamed_9 = 64;
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
pub type applyRatesFn
    =
    unsafe extern "C" fn(_: libc::c_int, _: libc::c_float, _: libc::c_float)
        -> libc::c_float;
pub const THROTTLE: rc_alias = 3;
pub const RC_SMOOTHING_DEFAULT: C2RustUnnamed_4 = 1;
pub const RC_SMOOTHING_OFF: C2RustUnnamed_4 = 0;
pub const RC_SMOOTHING_MANUAL: C2RustUnnamed_4 = 3;
pub const RC_SMOOTHING_AUTO: C2RustUnnamed_4 = 2;
pub const RC_SMOOTHING_TYPE_INTERPOLATION: C2RustUnnamed_5 = 0;
/* this holds the data required to update samples thru a filter */
pub type biquadFilter_t = biquadFilter_s;
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
pub type rcSmoothingFilterTypes_t = rcSmoothingFilterTypes_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union rcSmoothingFilterTypes_u {
    pub pt1Filter: pt1Filter_t,
    pub biquadFilter: biquadFilter_t,
}
pub type pt1Filter_t = pt1Filter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pt1Filter_s {
    pub state: libc::c_float,
    pub k: libc::c_float,
}
pub type rcSmoothingFilter_t = rcSmoothingFilter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcSmoothingFilter_s {
    pub filterInitialized: bool,
    pub filter: [rcSmoothingFilterTypes_t; 4],
    pub inputCutoffFrequency: uint16_t,
    pub derivativeCutoffFrequency: uint16_t,
    pub averageFrameTimeUs: libc::c_int,
    pub training: rcSmoothingFilterTraining_t,
}
pub type rcSmoothingFilterTraining_t = rcSmoothingFilterTraining_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcSmoothingFilterTraining_s {
    pub sum: libc::c_float,
    pub count: libc::c_int,
    pub min: uint16_t,
    pub max: uint16_t,
}
pub const RC_SMOOTHING_INPUT_BIQUAD: C2RustUnnamed_6 = 1;
pub const RC_SMOOTHING_INPUT_PT1: C2RustUnnamed_6 = 0;
pub const RC_SMOOTHING_DERIVATIVE_PT1: C2RustUnnamed_7 = 1;
pub const RC_SMOOTHING_DERIVATIVE_OFF: C2RustUnnamed_7 = 0;
pub const RC_SMOOTHING_TYPE_FILTER: C2RustUnnamed_5 = 1;
pub const ANTI_GRAVITY_STEP: C2RustUnnamed_10 = 1;
pub const GPS_RESCUE_MODE: C2RustUnnamed_9 = 2048;
pub const HORIZON_MODE: C2RustUnnamed_9 = 2;
pub const ANGLE_MODE: C2RustUnnamed_9 = 1;
pub const PITCH: rc_alias = 1;
pub type flight3DConfig_t = flight3DConfig_s;
// enable '3D Switched Mode'
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
// (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
// introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
// introduce a deadband around the stick center for yaw axis. Must be greater than zero.
// defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
// when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
// invert control direction of yaw
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
pub type lowVoltageCutoff_t = lowVoltageCutoff_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct lowVoltageCutoff_s {
    pub enabled: bool,
    pub percentage: uint8_t,
    pub startTime: timeUs_t,
}
pub type rcControlsConfig_t = rcControlsConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcControlsConfig_s {
    pub deadband: uint8_t,
    pub yaw_deadband: uint8_t,
    pub alt_hold_deadband: uint8_t,
    pub alt_hold_fast_change: uint8_t,
    pub yaw_control_reversed: bool,
}
pub const THROTTLE_FLAG: C2RustUnnamed_11 = 8;
pub const PITCH_FLAG: C2RustUnnamed_11 = 2;
pub const ROLL_FLAG: C2RustUnnamed_11 = 1;
pub const YAW_FLAG: C2RustUnnamed_11 = 4;
pub const RC_SMOOTHING_VALUE_AVERAGE_FRAME: C2RustUnnamed_8 = 2;
pub const RC_SMOOTHING_VALUE_DERIVATIVE_ACTIVE: C2RustUnnamed_8 = 1;
pub const RC_SMOOTHING_VALUE_INPUT_ACTIVE: C2RustUnnamed_8 = 0;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub type C2RustUnnamed_4 = libc::c_uint;
pub type C2RustUnnamed_5 = libc::c_uint;
pub type C2RustUnnamed_6 = libc::c_uint;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const RC_SMOOTHING_DERIVATIVE_BIQUAD: C2RustUnnamed_7 = 2;
pub type C2RustUnnamed_8 = libc::c_uint;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const FAILSAFE_MODE: C2RustUnnamed_9 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_9 = 256;
pub const GPS_HOLD_MODE: C2RustUnnamed_9 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_9 = 16;
pub const BARO_MODE: C2RustUnnamed_9 = 8;
pub const MAG_MODE: C2RustUnnamed_9 = 4;
pub type C2RustUnnamed_10 = libc::c_uint;
pub const ANTI_GRAVITY_SMOOTH: C2RustUnnamed_10 = 0;
pub type C2RustUnnamed_11 = libc::c_uint;
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
// SITL (software in the loop) simulator
// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC
// file name to save config
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
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
unsafe extern "C" fn rcControlsConfig() -> *const rcControlsConfig_t {
    return &mut rcControlsConfig_System;
}
#[inline]
unsafe extern "C" fn flight3DConfig() -> *const flight3DConfig_t {
    return &mut flight3DConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
static mut setpointRate: [libc::c_float; 3] = [0.; 3];
static mut rcDeflection: [libc::c_float; 3] = [0.; 3];
static mut rcDeflectionAbs: [libc::c_float; 3] = [0.; 3];
static mut throttlePIDAttenuation: libc::c_float = 0.;
static mut reverseMotors: bool = 0 as libc::c_int != 0;
static mut applyRates: Option<applyRatesFn> = None;
#[no_mangle]
pub static mut currentRxRefreshRate: uint16_t = 0;
#[no_mangle]
pub static mut interpolationChannels: uint8_t = 0;
// 50ms or 20hz
static mut rcSmoothingData: rcSmoothingFilter_t =
    rcSmoothingFilter_t{filterInitialized: false,
                        filter:
                            [rcSmoothingFilterTypes_u{pt1Filter:
                                                          pt1Filter_t{state:
                                                                          0.,
                                                                      k:
                                                                          0.,},};
                                4],
                        inputCutoffFrequency: 0,
                        derivativeCutoffFrequency: 0,
                        averageFrameTimeUs: 0,
                        training:
                            rcSmoothingFilterTraining_t{sum: 0.,
                                                        count: 0,
                                                        min: 0,
                                                        max: 0,},};
// USE_RC_SMOOTHING_FILTER
#[no_mangle]
pub unsafe extern "C" fn getSetpointRate(mut axis: libc::c_int)
 -> libc::c_float {
    return setpointRate[axis as usize];
}
#[no_mangle]
pub unsafe extern "C" fn getRcDeflection(mut axis: libc::c_int)
 -> libc::c_float {
    return rcDeflection[axis as usize];
}
#[no_mangle]
pub unsafe extern "C" fn getRcDeflectionAbs(mut axis: libc::c_int)
 -> libc::c_float {
    return rcDeflectionAbs[axis as usize];
}
#[no_mangle]
pub unsafe extern "C" fn getThrottlePIDAttenuation() -> libc::c_float {
    return throttlePIDAttenuation;
}
static mut lookupThrottleRC: [int16_t; 12] = [0; 12];
// lookup table for expo & mid THROTTLE
unsafe extern "C" fn rcLookupThrottle(mut tmp: int32_t) -> int16_t {
    let tmp2: int32_t = tmp / 100 as libc::c_int;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return (lookupThrottleRC[tmp2 as usize] as libc::c_int +
                (tmp - tmp2 * 100 as libc::c_int) *
                    (lookupThrottleRC[(tmp2 + 1 as libc::c_int) as usize] as
                         libc::c_int -
                         lookupThrottleRC[tmp2 as usize] as libc::c_int) /
                    100 as libc::c_int) as int16_t;
}
#[no_mangle]
pub unsafe extern "C" fn applyBetaflightRates(axis: libc::c_int,
                                              mut rcCommandf: libc::c_float,
                                              rcCommandfAbs: libc::c_float)
 -> libc::c_float {
    if (*currentControlRateProfile).rcExpo[axis as usize] != 0 {
        let expof: libc::c_float =
            (*currentControlRateProfile).rcExpo[axis as usize] as libc::c_int
                as libc::c_float / 100.0f32;
        rcCommandf =
            rcCommandf * (rcCommandfAbs * rcCommandfAbs * rcCommandfAbs) *
                expof +
                rcCommandf * (1 as libc::c_int as libc::c_float - expof)
    }
    let mut rcRate: libc::c_float =
        (*currentControlRateProfile).rcRates[axis as usize] as libc::c_int as
            libc::c_float / 100.0f32;
    if rcRate > 2.0f32 { rcRate += 14.54f32 * (rcRate - 2.0f32) }
    let mut angleRate: libc::c_float = 200.0f32 * rcRate * rcCommandf;
    if (*currentControlRateProfile).rates[axis as usize] != 0 {
        let rcSuperfactor: libc::c_float =
            1.0f32 /
                constrainf(1.0f32 -
                               rcCommandfAbs *
                                   ((*currentControlRateProfile).rates[axis as
                                                                           usize]
                                        as libc::c_int as libc::c_float /
                                        100.0f32), 0.01f32, 1.00f32);
        angleRate *= rcSuperfactor
    }
    return angleRate;
}
#[no_mangle]
pub unsafe extern "C" fn applyRaceFlightRates(axis: libc::c_int,
                                              mut rcCommandf: libc::c_float,
                                              rcCommandfAbs: libc::c_float)
 -> libc::c_float {
    // -1.0 to 1.0 ranged and curved
    rcCommandf =
        (1.0f32 +
             0.01f32 *
                 (*currentControlRateProfile).rcExpo[axis as usize] as
                     libc::c_int as libc::c_float *
                 (rcCommandf * rcCommandf - 1.0f32)) * rcCommandf;
    // convert to -2000 to 2000 range using acro+ modifier
    let mut angleRate: libc::c_float =
        10.0f32 *
            (*currentControlRateProfile).rcRates[axis as usize] as libc::c_int
                as libc::c_float * rcCommandf;
    angleRate =
        angleRate *
            (1 as libc::c_int as libc::c_float +
                 rcCommandfAbs *
                     (*currentControlRateProfile).rates[axis as usize] as
                         libc::c_float * 0.01f32);
    return angleRate;
}
unsafe extern "C" fn calculateSetpointRate(mut axis: libc::c_int) {
    let mut angleRate: libc::c_float = 0.;
    // scale rcCommandf to range [-1.0, 1.0]
    let mut rcCommandf: libc::c_float =
        rcCommand[axis as usize] /
            500.0f32; // Rate limit protection (deg/sec)
    rcDeflection[axis as usize] = rcCommandf;
    let rcCommandfAbs: libc::c_float =
        ({
             let mut _x: libc::c_float = rcCommandf;
             if _x > 0 as libc::c_int as libc::c_float { _x } else { -_x }
         });
    rcDeflectionAbs[axis as usize] = rcCommandfAbs;
    angleRate =
        applyRates.expect("non-null function pointer")(axis, rcCommandf,
                                                       rcCommandfAbs);
    setpointRate[axis as usize] =
        constrainf(angleRate, -1998.0f32, 1998.0f32);
    if debugMode as libc::c_int == DEBUG_ANGLERATE as libc::c_int {
        debug[axis as usize] = angleRate as int16_t
    };
}
unsafe extern "C" fn scaleRcCommandToFpvCamAngle() {
    //recalculate sin/cos only when rxConfig()->fpvCamAngleDegrees changed
    static mut lastFpvCamAngleDegrees: uint8_t = 0 as libc::c_int as uint8_t;
    static mut cosFactor: libc::c_float = 1.0f64 as libc::c_float;
    static mut sinFactor: libc::c_float = 0.0f64 as libc::c_float;
    if lastFpvCamAngleDegrees as libc::c_int !=
           (*rxConfig()).fpvCamAngleDegrees as libc::c_int {
        lastFpvCamAngleDegrees = (*rxConfig()).fpvCamAngleDegrees;
        cosFactor =
            cos_approx((*rxConfig()).fpvCamAngleDegrees as libc::c_int as
                           libc::c_float *
                           (3.14159265358979323846f32 / 180.0f32));
        sinFactor =
            sin_approx((*rxConfig()).fpvCamAngleDegrees as libc::c_int as
                           libc::c_float *
                           (3.14159265358979323846f32 / 180.0f32))
    }
    let mut roll: libc::c_float = setpointRate[ROLL as libc::c_int as usize];
    let mut yaw: libc::c_float = setpointRate[YAW as libc::c_int as usize];
    setpointRate[ROLL as libc::c_int as usize] =
        constrainf(roll * cosFactor - yaw * sinFactor, -1998.0f32, 1998.0f32);
    setpointRate[YAW as libc::c_int as usize] =
        constrainf(yaw * cosFactor + roll * sinFactor, -1998.0f32, 1998.0f32);
}
unsafe extern "C" fn checkForThrottleErrorResetState(mut rxRefreshRate:
                                                         uint16_t) {
    static mut index: libc::c_int = 0;
    static mut rcCommandThrottlePrevious: [int16_t; 20] = [0; 20];
    let rxRefreshRateMs: libc::c_int =
        rxRefreshRate as libc::c_int / 1000 as libc::c_int;
    let indexMax: libc::c_int =
        constrain(100 as libc::c_int / rxRefreshRateMs, 1 as libc::c_int,
                  20 as libc::c_int);
    let throttleVelocityThreshold: int16_t =
        if feature(FEATURE_3D as libc::c_int as uint32_t) as libc::c_int != 0
           {
            ((*currentPidProfile).itermThrottleThreshold as libc::c_int) /
                2 as libc::c_int
        } else { (*currentPidProfile).itermThrottleThreshold as libc::c_int }
            as int16_t;
    let fresh0 = index;
    index = index + 1;
    rcCommandThrottlePrevious[fresh0 as usize] =
        rcCommand[THROTTLE as libc::c_int as usize] as int16_t;
    if index >= indexMax { index = 0 as libc::c_int }
    let rcCommandSpeed: int16_t =
        (rcCommand[THROTTLE as libc::c_int as usize] -
             rcCommandThrottlePrevious[index as usize] as libc::c_int as
                 libc::c_float) as int16_t;
    if (*currentPidProfile).antiGravityMode as libc::c_int ==
           ANTI_GRAVITY_STEP as libc::c_int {
        if ({
                let _x: int16_t = rcCommandSpeed;
                (if _x as libc::c_int > 0 as libc::c_int {
                     _x as libc::c_int
                 } else { -(_x as libc::c_int) })
            }) > throttleVelocityThreshold as libc::c_int {
            pidSetItermAccelerator(0.001f32 *
                                       (*currentPidProfile).itermAcceleratorGain
                                           as libc::c_int as libc::c_float);
        } else { pidSetItermAccelerator(1.0f32); }
    };
}
#[no_mangle]
pub unsafe extern "C" fn processRcInterpolation() -> uint8_t {
    static mut rcCommandInterp: [libc::c_float; 4] = [0.; 4];
    static mut rcStepSize: [libc::c_float; 4] = [0.; 4];
    static mut rcInterpolationStepCount: int16_t = 0;
    let mut rxRefreshRate: uint16_t = 0;
    let mut updatedChannel: uint8_t = 0 as libc::c_int as uint8_t;
    if (*rxConfig()).rcInterpolation != 0 {
        // Set RC refresh rate for sampling and channels to filter
        match (*rxConfig()).rcInterpolation as libc::c_int {
            2 => {
                rxRefreshRate =
                    (currentRxRefreshRate as libc::c_int +
                         1000 as libc::c_int) as uint16_t
            }
            3 => {
                rxRefreshRate =
                    (1000 as libc::c_int *
                         (*rxConfig()).rcInterpolationInterval as libc::c_int)
                        as uint16_t
            }
            0 | 1 | _ => { rxRefreshRate = rxGetRefreshRate() }
        } // Add slight overhead to prevent ramps
        if isRXDataNew as libc::c_int != 0 &&
               rxRefreshRate as libc::c_int > 0 as libc::c_int {
            rcInterpolationStepCount =
                (rxRefreshRate as
                     libc::c_uint).wrapping_div(targetPidLooptime) as int16_t;
            let mut channel: libc::c_int = 0 as libc::c_int;
            while channel < THROTTLE as libc::c_int + 1 as libc::c_int {
                if (1 as libc::c_int) << channel &
                       interpolationChannels as libc::c_int != 0 {
                    rcStepSize[channel as usize] =
                        (rcCommand[channel as usize] -
                             rcCommandInterp[channel as usize]) /
                            rcInterpolationStepCount as libc::c_float
                }
                channel += 1
            }
            if debugMode as libc::c_int ==
                   DEBUG_RC_INTERPOLATION as libc::c_int {
                debug[0 as libc::c_int as usize] =
                    lrintf(rcCommand[0 as libc::c_int as usize]) as int16_t
            }
            if debugMode as libc::c_int ==
                   DEBUG_RC_INTERPOLATION as libc::c_int {
                debug[1 as libc::c_int as usize] =
                    lrintf((currentRxRefreshRate as libc::c_int /
                                1000 as libc::c_int) as libc::c_float) as
                        int16_t
            }
        } else { rcInterpolationStepCount -= 1 }
        // Interpolate steps of rcCommand
        if rcInterpolationStepCount as libc::c_int > 0 as libc::c_int {
            updatedChannel = 0 as libc::c_int as uint8_t;
            while (updatedChannel as libc::c_int) <
                      THROTTLE as libc::c_int + 1 as libc::c_int {
                if (1 as libc::c_int) << updatedChannel as libc::c_int &
                       interpolationChannels as libc::c_int != 0 {
                    rcCommandInterp[updatedChannel as usize] +=
                        rcStepSize[updatedChannel as usize];
                    rcCommand[updatedChannel as usize] =
                        rcCommandInterp[updatedChannel as usize]
                }
                updatedChannel = updatedChannel.wrapping_add(1)
            }
        }
    } else {
        rcInterpolationStepCount = 0 as libc::c_int as int16_t
        // reset factor in case of level modes flip flopping
    }
    if debugMode as libc::c_int == DEBUG_RC_INTERPOLATION as libc::c_int {
        debug[2 as libc::c_int as usize] = rcInterpolationStepCount
    }
    return updatedChannel;
}
// Determine a cutoff frequency based on filter type and the calculated
// average rx frame time
#[no_mangle]
pub unsafe extern "C" fn calcRcSmoothingCutoff(mut avgRxFrameTimeUs:
                                                   libc::c_int, mut pt1: bool)
 -> libc::c_int {
    if avgRxFrameTimeUs > 0 as libc::c_int {
        let mut cutoff: libc::c_float =
            1 as libc::c_int as libc::c_float /
                (avgRxFrameTimeUs as libc::c_float * 1e-6f32) /
                2 as libc::c_int as
                    libc::c_float; // calculate the nyquist frequency
        cutoff =
            cutoff * 0.90f32; // Use 90% of the calculated nyquist frequency
        if pt1 {
            cutoff = cutoff * cutoff / 80 as libc::c_int as libc::c_float
            // convert to a cutoff for pt1 that has similar characteristics
        }
        return lrintf(cutoff) as libc::c_int
    } else { return 0 as libc::c_int };
}
// Preforms a reasonableness check on the rx frame time to avoid bad data
// skewing the average.
#[no_mangle]
pub unsafe extern "C" fn rcSmoothingRxRateValid(mut currentRxRefreshRate_0:
                                                    libc::c_int) -> bool {
    return currentRxRefreshRate_0 >= 1000 as libc::c_int &&
               currentRxRefreshRate_0 <= 50000 as libc::c_int;
}
// Initialize or update the filters base on either the manually selected cutoff, or
// the auto-calculated cutoff frequency based on detected rx frame rate.
#[no_mangle]
pub unsafe extern "C" fn rcSmoothingSetFilterCutoffs(mut smoothingData:
                                                         *mut rcSmoothingFilter_t) {
    let dT: libc::c_float = targetPidLooptime as libc::c_float * 1e-6f32;
    let mut oldCutoff: uint16_t = (*smoothingData).inputCutoffFrequency;
    if (*rxConfig()).rc_smoothing_input_cutoff as libc::c_int ==
           0 as libc::c_int {
        (*smoothingData).inputCutoffFrequency =
            calcRcSmoothingCutoff((*smoothingData).averageFrameTimeUs,
                                  (*rxConfig()).rc_smoothing_input_type as
                                      libc::c_int ==
                                      RC_SMOOTHING_INPUT_PT1 as libc::c_int)
                as uint16_t
    }
    // initialize or update the input filter
    if (*smoothingData).inputCutoffFrequency as libc::c_int !=
           oldCutoff as libc::c_int || !(*smoothingData).filterInitialized {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < THROTTLE as libc::c_int + 1 as libc::c_int {
            if (1 as libc::c_int) << i & interpolationChannels as libc::c_int
                   != 0 {
                // only update channels specified by rc_interp_ch
                match (*rxConfig()).rc_smoothing_input_type as libc::c_int {
                    0 => {
                        if !(*smoothingData).filterInitialized {
                            pt1FilterInit(&mut *(*smoothingData).filter.as_mut_ptr().offset(i
                                                                                                as
                                                                                                isize)
                                              as *mut rcSmoothingFilterTypes_t
                                              as *mut pt1Filter_t,
                                          pt1FilterGain((*smoothingData).inputCutoffFrequency,
                                                        dT));
                        } else {
                            pt1FilterUpdateCutoff(&mut *(*smoothingData).filter.as_mut_ptr().offset(i
                                                                                                        as
                                                                                                        isize)
                                                      as
                                                      *mut rcSmoothingFilterTypes_t
                                                      as *mut pt1Filter_t,
                                                  pt1FilterGain((*smoothingData).inputCutoffFrequency,
                                                                dT));
                        }
                    }
                    1 | _ => {
                        if !(*smoothingData).filterInitialized {
                            biquadFilterInitLPF(&mut *(*smoothingData).filter.as_mut_ptr().offset(i
                                                                                                      as
                                                                                                      isize)
                                                    as
                                                    *mut rcSmoothingFilterTypes_t
                                                    as *mut biquadFilter_t,
                                                (*smoothingData).inputCutoffFrequency
                                                    as libc::c_float,
                                                targetPidLooptime);
                        } else {
                            biquadFilterUpdateLPF(&mut *(*smoothingData).filter.as_mut_ptr().offset(i
                                                                                                        as
                                                                                                        isize)
                                                      as
                                                      *mut rcSmoothingFilterTypes_t
                                                      as *mut biquadFilter_t,
                                                  (*smoothingData).inputCutoffFrequency
                                                      as libc::c_float,
                                                  targetPidLooptime);
                        }
                    }
                }
            }
            i += 1
        }
    }
    // update or initialize the derivative filter
    oldCutoff = (*smoothingData).derivativeCutoffFrequency;
    if (*rxConfig()).rc_smoothing_derivative_cutoff as libc::c_int ==
           0 as libc::c_int &&
           (*rxConfig()).rc_smoothing_derivative_type as libc::c_int !=
               RC_SMOOTHING_DERIVATIVE_OFF as libc::c_int {
        (*smoothingData).derivativeCutoffFrequency =
            calcRcSmoothingCutoff((*smoothingData).averageFrameTimeUs,
                                  (*rxConfig()).rc_smoothing_derivative_type
                                      as libc::c_int ==
                                      RC_SMOOTHING_DERIVATIVE_PT1 as
                                          libc::c_int) as uint16_t
    }
    if !(*smoothingData).filterInitialized {
        pidInitSetpointDerivativeLpf((*smoothingData).derivativeCutoffFrequency,
                                     (*rxConfig()).rc_smoothing_debug_axis,
                                     (*rxConfig()).rc_smoothing_derivative_type);
    } else if (*smoothingData).derivativeCutoffFrequency as libc::c_int !=
                  oldCutoff as libc::c_int {
        pidUpdateSetpointDerivativeLpf((*smoothingData).derivativeCutoffFrequency);
    };
}
#[no_mangle]
pub unsafe extern "C" fn rcSmoothingResetAccumulation(mut smoothingData:
                                                          *mut rcSmoothingFilter_t) {
    (*smoothingData).training.sum = 0 as libc::c_int as libc::c_float;
    (*smoothingData).training.count = 0 as libc::c_int;
    (*smoothingData).training.min = 65535 as libc::c_int as uint16_t;
    (*smoothingData).training.max = 0 as libc::c_int as uint16_t;
}
// Accumulate the rx frame time samples. Once we've collected enough samples calculate the
// average and return true.
#[no_mangle]
pub unsafe extern "C" fn rcSmoothingAccumulateSample(mut smoothingData:
                                                         *mut rcSmoothingFilter_t,
                                                     mut rxFrameTimeUs:
                                                         libc::c_int)
 -> bool {
    (*smoothingData).training.sum += rxFrameTimeUs as libc::c_float;
    (*smoothingData).training.count += 1;
    (*smoothingData).training.max =
        ({
             let mut _a: uint16_t = (*smoothingData).training.max;
             let mut _b: libc::c_int = rxFrameTimeUs;
             if _a as libc::c_int > _b { _a as libc::c_int } else { _b }
         }) as uint16_t;
    (*smoothingData).training.min =
        ({
             let mut _a: uint16_t = (*smoothingData).training.min;
             let mut _b: libc::c_int = rxFrameTimeUs;
             if (_a as libc::c_int) < _b { _a as libc::c_int } else { _b }
         }) as uint16_t;
    // if we've collected enough samples then calculate the average and reset the accumulation
    if (*smoothingData).training.count >= 50 as libc::c_int {
        (*smoothingData).training.sum =
            (*smoothingData).training.sum -
                (*smoothingData).training.min as libc::c_int as libc::c_float
                -
                (*smoothingData).training.max as libc::c_int as
                    libc::c_float; // Throw out high and low samples
        (*smoothingData).averageFrameTimeUs =
            lrintf((*smoothingData).training.sum /
                       ((*smoothingData).training.count - 2 as libc::c_int) as
                           libc::c_float) as libc::c_int;
        rcSmoothingResetAccumulation(smoothingData);
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
// Determine if we need to caclulate filter cutoffs. If not then we can avoid
// examining the rx frame times completely 
#[no_mangle]
pub unsafe extern "C" fn rcSmoothingAutoCalculate() -> bool {
    let mut ret: bool = 0 as libc::c_int != 0;
    // if the input cutoff is 0 (auto) then we need to calculate cutoffs
    if (*rxConfig()).rc_smoothing_input_cutoff as libc::c_int ==
           0 as libc::c_int {
        ret = 1 as libc::c_int != 0
    }
    // if the derivative type isn't OFF and the cutoff is 0 then we need to calculate
    if (*rxConfig()).rc_smoothing_derivative_type as libc::c_int !=
           RC_SMOOTHING_DERIVATIVE_OFF as libc::c_int {
        if (*rxConfig()).rc_smoothing_derivative_cutoff as libc::c_int ==
               0 as libc::c_int {
            ret = 1 as libc::c_int != 0
        }
    }
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn processRcSmoothingFilter() -> uint8_t {
    let mut updatedChannel: uint8_t = 0 as libc::c_int as uint8_t;
    static mut lastRxData: [libc::c_float; 4] = [0.; 4];
    static mut initialized: bool = false;
    static mut validRxFrameTimeMs: timeMs_t = 0;
    static mut calculateCutoffs: bool = false;
    // first call initialization
    if !initialized {
        initialized = 1 as libc::c_int != 0;
        rcSmoothingData.filterInitialized = 0 as libc::c_int != 0;
        rcSmoothingData.averageFrameTimeUs = 0 as libc::c_int;
        rcSmoothingResetAccumulation(&mut rcSmoothingData);
        rcSmoothingData.inputCutoffFrequency =
            (*rxConfig()).rc_smoothing_input_cutoff as uint16_t;
        if (*rxConfig()).rc_smoothing_derivative_type as libc::c_int !=
               RC_SMOOTHING_DERIVATIVE_OFF as libc::c_int {
            rcSmoothingData.derivativeCutoffFrequency =
                (*rxConfig()).rc_smoothing_derivative_cutoff as uint16_t
        }
        calculateCutoffs = rcSmoothingAutoCalculate();
        // if we don't need to calculate cutoffs dynamically then the filters can be initialized now
        if !calculateCutoffs {
            rcSmoothingSetFilterCutoffs(&mut rcSmoothingData);
            rcSmoothingData.filterInitialized = 1 as libc::c_int != 0
        }
    }
    if isRXDataNew {
        // store the new raw channel values
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < THROTTLE as libc::c_int + 1 as libc::c_int {
            if (1 as libc::c_int) << i & interpolationChannels as libc::c_int
                   != 0 {
                lastRxData[i as usize] = rcCommand[i as usize]
            }
            i += 1
        }
        // for dynamically calculated filters we need to examine each rx frame interval
        if calculateCutoffs {
            let currentTimeMs: timeMs_t = millis();
            let mut sampleState: libc::c_int = 0 as libc::c_int;
            // If the filter cutoffs are set to auto and we have good rx data, then determine the average rx frame rate
            // and use that to calculate the filter cutoff frequencies
            if currentTimeMs > 5000 as libc::c_int as libc::c_uint &&
                   targetPidLooptime > 0 as libc::c_int as libc::c_uint {
                // skip during FC initialization
                if rxIsReceivingSignal() as libc::c_int != 0 &&
                       rcSmoothingRxRateValid(currentRxRefreshRate as
                                                  libc::c_int) as libc::c_int
                           != 0 {
                    // set the guard time expiration if it's not set
                    if validRxFrameTimeMs == 0 as libc::c_int as libc::c_uint
                       {
                        validRxFrameTimeMs =
                            currentTimeMs.wrapping_add((if rcSmoothingData.filterInitialized
                                                               as libc::c_int
                                                               != 0 {
                                                            2000 as
                                                                libc::c_int
                                                        } else {
                                                            1000 as
                                                                libc::c_int
                                                        }) as libc::c_uint)
                    } else { sampleState = 1 as libc::c_int }
                    // if the guard time has expired then process the rx frame time
                    if currentTimeMs > validRxFrameTimeMs {
                        sampleState = 2 as libc::c_int;
                        let mut accumulateSample: bool =
                            1 as libc::c_int != 0;
                        // During initial training process all samples.
                        // During retraining check samples to determine if they vary by more than the limit percentage.
                        if rcSmoothingData.filterInitialized {
                            let percentChange: libc::c_float =
                                ({
                                     let mut _x: libc::c_int =
                                         currentRxRefreshRate as libc::c_int -
                                             rcSmoothingData.averageFrameTimeUs;
                                     (if _x > 0 as libc::c_int {
                                          _x
                                      } else { -_x })
                                 }) as libc::c_float /
                                    rcSmoothingData.averageFrameTimeUs as
                                        libc::c_float *
                                    100 as libc::c_int as libc::c_float;
                            if percentChange <
                                   20 as libc::c_int as libc::c_float {
                                // We received a sample that wasn't more than the limit percent so reset the accumulation
                                // During retraining we need a contiguous block of samples that are all significantly different than the current average
                                rcSmoothingResetAccumulation(&mut rcSmoothingData);
                                accumulateSample = 0 as libc::c_int != 0
                            }
                        }
                        // accumlate the sample into the average
                        if accumulateSample {
                            if rcSmoothingAccumulateSample(&mut rcSmoothingData,
                                                           currentRxRefreshRate
                                                               as libc::c_int)
                               {
                                // the required number of samples were collected so set the filter cutoffs
                                rcSmoothingSetFilterCutoffs(&mut rcSmoothingData);
                                rcSmoothingData.filterInitialized =
                                    1 as libc::c_int != 0;
                                validRxFrameTimeMs =
                                    0 as libc::c_int as timeMs_t
                            }
                        }
                    }
                } else {
                    // we have either stopped receiving rx samples (failsafe?) or the sample time is unreasonable so reset the accumulation
                    rcSmoothingResetAccumulation(&mut rcSmoothingData);
                }
            }
            // rx frame rate training blackbox debugging
            if debugMode as libc::c_int ==
                   DEBUG_RC_SMOOTHING_RATE as libc::c_int {
                if debugMode as libc::c_int ==
                       DEBUG_RC_SMOOTHING_RATE as libc::c_int {
                    debug[0 as libc::c_int as usize] =
                        currentRxRefreshRate as int16_t
                }
                // indicates whether guard time is active
                if debugMode as libc::c_int ==
                       DEBUG_RC_SMOOTHING_RATE as libc::c_int {
                    debug[1 as libc::c_int as usize] =
                        rcSmoothingData.training.count as int16_t
                }
                if debugMode as libc::c_int ==
                       DEBUG_RC_SMOOTHING_RATE as libc::c_int {
                    debug[2 as libc::c_int as usize] =
                        rcSmoothingData.averageFrameTimeUs as int16_t
                }
                if debugMode as libc::c_int ==
                       DEBUG_RC_SMOOTHING_RATE as libc::c_int {
                    debug[3 as libc::c_int as usize] = sampleState as int16_t
                }
            }
        }
    }
    if rcSmoothingData.filterInitialized as libc::c_int != 0 &&
           debugMode as libc::c_int == DEBUG_RC_SMOOTHING as libc::c_int {
        // log each rx frame interval
        // log the training step count
        // the current calculated average
        // after training has completed then log the raw rc channel and the calculated
        // average rx frame rate that was used to calculate the automatic filter cutoffs
        if debugMode as libc::c_int == DEBUG_RC_SMOOTHING as libc::c_int {
            debug[0 as libc::c_int as usize] =
                lrintf(lastRxData[(*rxConfig()).rc_smoothing_debug_axis as
                                      usize]) as int16_t
        }
        if debugMode as libc::c_int == DEBUG_RC_SMOOTHING as libc::c_int {
            debug[3 as libc::c_int as usize] =
                rcSmoothingData.averageFrameTimeUs as int16_t
        }
    }
    // each pid loop continue to apply the last received channel value to the filter
    updatedChannel = 0 as libc::c_int as uint8_t;
    while (updatedChannel as libc::c_int) <
              THROTTLE as libc::c_int + 1 as libc::c_int {
        if (1 as libc::c_int) << updatedChannel as libc::c_int &
               interpolationChannels as libc::c_int != 0 {
            // only smooth selected channels base on the rc_interp_ch value
            if rcSmoothingData.filterInitialized {
                match (*rxConfig()).rc_smoothing_input_type as libc::c_int {
                    0 => {
                        rcCommand[updatedChannel as usize] =
                            pt1FilterApply(&mut *rcSmoothingData.filter.as_mut_ptr().offset(updatedChannel
                                                                                                as
                                                                                                isize)
                                               as
                                               *mut rcSmoothingFilterTypes_t
                                               as *mut pt1Filter_t,
                                           lastRxData[updatedChannel as
                                                          usize])
                    }
                    1 | _ => {
                        rcCommand[updatedChannel as usize] =
                            biquadFilterApplyDF1(&mut *rcSmoothingData.filter.as_mut_ptr().offset(updatedChannel
                                                                                                      as
                                                                                                      isize)
                                                     as
                                                     *mut rcSmoothingFilterTypes_t
                                                     as *mut biquadFilter_t,
                                                 lastRxData[updatedChannel as
                                                                usize])
                    }
                }
            } else {
                // If filter isn't initialized yet then use the actual unsmoothed rx channel data
                rcCommand[updatedChannel as usize] =
                    lastRxData[updatedChannel as usize]
            }
        }
        updatedChannel = updatedChannel.wrapping_add(1)
    }
    return interpolationChannels;
}
// USE_RC_SMOOTHING_FILTER
#[no_mangle]
pub unsafe extern "C" fn processRcCommand() {
    let mut updatedChannel: uint8_t = 0;
    if isRXDataNew as libc::c_int != 0 &&
           pidAntiGravityEnabled() as libc::c_int != 0 {
        checkForThrottleErrorResetState(currentRxRefreshRate);
    }
    match (*rxConfig()).rc_smoothing_type as libc::c_int {
        1 => { updatedChannel = processRcSmoothingFilter() }
        0 | _ => {
            // USE_RC_SMOOTHING_FILTER
            updatedChannel = processRcInterpolation()
        }
    } // throttle channel doesn't require rate calculation
    if isRXDataNew as libc::c_int != 0 || updatedChannel as libc::c_int != 0 {
        let maxUpdatedAxis: uint8_t =
            if isRXDataNew as libc::c_int != 0 {
                FD_YAW as libc::c_int
            } else {
                ({
                     let mut _a: uint8_t = updatedChannel;
                     let mut _b: libc::c_int = FD_YAW as libc::c_int;
                     if (_a as libc::c_int) < _b {
                         _a as libc::c_int
                     } else { _b }
                 })
            } as uint8_t;
        let mut axis: libc::c_int = FD_ROLL as libc::c_int;
        while axis <= maxUpdatedAxis as libc::c_int {
            calculateSetpointRate(axis);
            axis += 1
        }
        if debugMode as libc::c_int == DEBUG_RC_INTERPOLATION as libc::c_int {
            debug[3 as libc::c_int as usize] =
                setpointRate[0 as libc::c_int as usize] as int16_t
        }
        // Scaling of AngleRate to camera angle (Mixing Roll and Yaw)
        if (*rxConfig()).fpvCamAngleDegrees as libc::c_int != 0 &&
               IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) as libc::c_int != 0 &&
               flightModeFlags as libc::c_int & HEADFREE_MODE as libc::c_int
                   == 0 {
            scaleRcCommandToFpvCamAngle();
        }
    }
    if isRXDataNew { isRXDataNew = 0 as libc::c_int != 0 };
}
#[no_mangle]
pub unsafe extern "C" fn updateRcCommands() {
    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    let mut prop: int32_t = 0;
    if (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
           (*currentControlRateProfile).tpa_breakpoint as libc::c_int {
        prop = 100 as libc::c_int;
        throttlePIDAttenuation = 1.0f32
    } else {
        if (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
               2000 as libc::c_int {
            prop =
                100 as libc::c_int -
                    (*currentControlRateProfile).dynThrPID as uint16_t as
                        libc::c_int *
                        (rcData[THROTTLE as libc::c_int as usize] as
                             libc::c_int -
                             (*currentControlRateProfile).tpa_breakpoint as
                                 libc::c_int) /
                        (2000 as libc::c_int -
                             (*currentControlRateProfile).tpa_breakpoint as
                                 libc::c_int)
        } else {
            prop =
                100 as libc::c_int -
                    (*currentControlRateProfile).dynThrPID as libc::c_int
        }
        throttlePIDAttenuation = prop as libc::c_float / 100.0f32
    }
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        // non coupled PID reduction scaler used in PID controller 1 and PID controller 2.
        let mut tmp: int32_t =
            ({
                 let mut _a: libc::c_int =
                     ({
                          let mut _x: libc::c_int =
                              rcData[axis as usize] as libc::c_int -
                                  (*rxConfig()).midrc as libc::c_int;
                          if _x > 0 as libc::c_int { _x } else { -_x }
                      });
                 let mut _b: libc::c_int = 500 as libc::c_int;
                 if _a < _b { _a } else { _b }
             });
        if axis == ROLL as libc::c_int || axis == PITCH as libc::c_int {
            if tmp > (*rcControlsConfig()).deadband as libc::c_int {
                tmp -= (*rcControlsConfig()).deadband as libc::c_int
            } else { tmp = 0 as libc::c_int }
            rcCommand[axis as usize] = tmp as libc::c_float
        } else {
            if tmp > (*rcControlsConfig()).yaw_deadband as libc::c_int {
                tmp -= (*rcControlsConfig()).yaw_deadband as libc::c_int
            } else { tmp = 0 as libc::c_int }
            rcCommand[axis as usize] =
                (tmp *
                     -(if (*rcControlsConfig()).yaw_control_reversed as
                              libc::c_int != 0 {
                           -(1 as libc::c_int)
                       } else { 1 as libc::c_int })) as libc::c_float
        }
        if (rcData[axis as usize] as libc::c_int) <
               (*rxConfig()).midrc as libc::c_int {
            rcCommand[axis as usize] = -rcCommand[axis as usize]
        }
        axis += 1
    }
    let mut tmp_0: int32_t = 0;
    if feature(FEATURE_3D as libc::c_int as uint32_t) {
        tmp_0 =
            constrain(rcData[THROTTLE as libc::c_int as usize] as libc::c_int,
                      1000 as libc::c_int, 2000 as libc::c_int);
        tmp_0 = (tmp_0 - 1000 as libc::c_int) as uint32_t as int32_t
    } else {
        tmp_0 =
            constrain(rcData[THROTTLE as libc::c_int as usize] as libc::c_int,
                      (*rxConfig()).mincheck as libc::c_int,
                      2000 as libc::c_int);
        tmp_0 =
            ((tmp_0 - (*rxConfig()).mincheck as libc::c_int) as
                 uint32_t).wrapping_mul(1000 as libc::c_int as
                                            libc::c_uint).wrapping_div((2000
                                                                            as
                                                                            libc::c_int
                                                                            -
                                                                            (*rxConfig()).mincheck
                                                                                as
                                                                                libc::c_int)
                                                                           as
                                                                           libc::c_uint)
                as int32_t
    }
    if (*getLowVoltageCutoff()).enabled {
        tmp_0 =
            tmp_0 * (*getLowVoltageCutoff()).percentage as libc::c_int /
                100 as libc::c_int
    }
    rcCommand[THROTTLE as libc::c_int as usize] =
        rcLookupThrottle(tmp_0) as libc::c_float;
    if feature(FEATURE_3D as libc::c_int as uint32_t) as libc::c_int != 0 &&
           !failsafeIsActive() {
        if (*flight3DConfig()).switched_mode3d == 0 {
            if IS_RC_MODE_ACTIVE(BOX3D) {
                let mut throttleScaler: fix12_t =
                    qConstruct((rcCommand[THROTTLE as libc::c_int as usize] -
                                    1000 as libc::c_int as libc::c_float) as
                                   int16_t, 1000 as libc::c_int as int16_t);
                rcCommand[THROTTLE as libc::c_int as usize] =
                    ((*rxConfig()).midrc as libc::c_int +
                         qMultiply(throttleScaler,
                                   (2000 as libc::c_int -
                                        (*rxConfig()).midrc as libc::c_int) as
                                       int16_t) as libc::c_int) as
                        libc::c_float
            }
        } else if IS_RC_MODE_ACTIVE(BOX3D) {
            reverseMotors = 1 as libc::c_int != 0;
            let mut throttleScaler_0: fix12_t =
                qConstruct((rcCommand[THROTTLE as libc::c_int as usize] -
                                1000 as libc::c_int as libc::c_float) as
                               int16_t, 1000 as libc::c_int as int16_t);
            rcCommand[THROTTLE as libc::c_int as usize] =
                ((*rxConfig()).midrc as libc::c_int +
                     qMultiply(throttleScaler_0,
                               (1000 as libc::c_int -
                                    (*rxConfig()).midrc as libc::c_int) as
                                   int16_t) as libc::c_int) as libc::c_float
        } else {
            reverseMotors = 0 as libc::c_int != 0;
            let mut throttleScaler_1: fix12_t =
                qConstruct((rcCommand[THROTTLE as libc::c_int as usize] -
                                1000 as libc::c_int as libc::c_float) as
                               int16_t, 1000 as libc::c_int as int16_t);
            rcCommand[THROTTLE as libc::c_int as usize] =
                ((*rxConfig()).midrc as libc::c_int +
                     qMultiply(throttleScaler_1,
                               (2000 as libc::c_int -
                                    (*rxConfig()).midrc as libc::c_int) as
                                   int16_t) as libc::c_int) as libc::c_float
        }
    }
    if flightModeFlags as libc::c_int & HEADFREE_MODE as libc::c_int != 0 {
        static mut rcCommandBuff: t_fp_vector_def =
            t_fp_vector_def{X: 0., Y: 0., Z: 0.,};
        rcCommandBuff.X = rcCommand[ROLL as libc::c_int as usize];
        rcCommandBuff.Y = rcCommand[PITCH as libc::c_int as usize];
        if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int == 0 &&
               flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int ==
                   0 &&
               flightModeFlags as libc::c_int & GPS_RESCUE_MODE as libc::c_int
                   == 0 {
            rcCommandBuff.Z = rcCommand[YAW as libc::c_int as usize]
        } else { rcCommandBuff.Z = 0 as libc::c_int as libc::c_float }
        imuQuaternionHeadfreeTransformVectorEarthToBody(&mut rcCommandBuff);
        rcCommand[ROLL as libc::c_int as usize] = rcCommandBuff.X;
        rcCommand[PITCH as libc::c_int as usize] = rcCommandBuff.Y;
        if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int == 0 &&
               flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int ==
                   0 &&
               flightModeFlags as libc::c_int & GPS_RESCUE_MODE as libc::c_int
                   == 0 {
            rcCommand[YAW as libc::c_int as usize] = rcCommandBuff.Z
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn resetYawAxis() {
    rcCommand[YAW as libc::c_int as usize] =
        0 as libc::c_int as libc::c_float;
    setpointRate[YAW as libc::c_int as usize] =
        0 as libc::c_int as libc::c_float;
}
#[no_mangle]
pub unsafe extern "C" fn isMotorsReversed() -> bool { return reverseMotors; }
#[no_mangle]
pub unsafe extern "C" fn initRcProcessing() {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 12 as libc::c_int {
        let tmp: int16_t =
            (10 as libc::c_int * i -
                 (*currentControlRateProfile).thrMid8 as libc::c_int) as
                int16_t;
        let mut y: uint8_t = 1 as libc::c_int as uint8_t;
        if tmp as libc::c_int > 0 as libc::c_int {
            y =
                (100 as libc::c_int -
                     (*currentControlRateProfile).thrMid8 as libc::c_int) as
                    uint8_t
        }
        if (tmp as libc::c_int) < 0 as libc::c_int {
            y = (*currentControlRateProfile).thrMid8
        }
        lookupThrottleRC[i as usize] =
            (10 as libc::c_int *
                 (*currentControlRateProfile).thrMid8 as libc::c_int +
                 tmp as libc::c_int *
                     (100 as libc::c_int -
                          (*currentControlRateProfile).thrExpo8 as libc::c_int
                          +
                          (*currentControlRateProfile).thrExpo8 as int32_t *
                              (tmp as libc::c_int * tmp as libc::c_int) /
                              (y as libc::c_int * y as libc::c_int)) /
                     10 as libc::c_int) as int16_t;
        lookupThrottleRC[i as usize] =
            (1000 as libc::c_int +
                 (2000 as libc::c_int - 1000 as libc::c_int) *
                     lookupThrottleRC[i as usize] as libc::c_int /
                     1000 as libc::c_int) as int16_t;
        i += 1
        // [MINTHROTTLE;MAXTHROTTLE]
    }
    match (*currentControlRateProfile).rates_type as libc::c_int {
        1 => {
            applyRates =
                Some(applyRaceFlightRates as
                         unsafe extern "C" fn(_: libc::c_int,
                                              _: libc::c_float,
                                              _: libc::c_float)
                             -> libc::c_float)
        }
        0 | _ => {
            applyRates =
                Some(applyBetaflightRates as
                         unsafe extern "C" fn(_: libc::c_int,
                                              _: libc::c_float,
                                              _: libc::c_float)
                             -> libc::c_float)
        }
    }
    interpolationChannels = 0 as libc::c_int as uint8_t;
    let mut current_block_22: u64;
    match (*rxConfig()).rcInterpolationChannels as libc::c_int {
        2 => {
            interpolationChannels =
                (interpolationChannels as libc::c_int |
                     THROTTLE_FLAG as libc::c_int) as uint8_t;
            current_block_22 = 4756526906437228406;
        }
        1 => { current_block_22 = 4756526906437228406; }
        0 => { current_block_22 = 10894603820806451092; }
        4 => {
            interpolationChannels =
                (interpolationChannels as libc::c_int |
                     (ROLL_FLAG as libc::c_int | PITCH_FLAG as libc::c_int))
                    as uint8_t;
            current_block_22 = 6831550151654322379;
        }
        3 => { current_block_22 = 6831550151654322379; }
        _ => { current_block_22 = 5689316957504528238; }
    }
    match current_block_22 {
        4756526906437228406 => {
            interpolationChannels =
                (interpolationChannels as libc::c_int |
                     YAW_FLAG as libc::c_int) as uint8_t;
            current_block_22 = 10894603820806451092;
        }
        6831550151654322379 => {
            interpolationChannels =
                (interpolationChannels as libc::c_int |
                     THROTTLE_FLAG as libc::c_int) as uint8_t;
            current_block_22 = 5689316957504528238;
        }
        _ => { }
    }
    match current_block_22 {
        10894603820806451092 => {
            interpolationChannels =
                (interpolationChannels as libc::c_int |
                     (ROLL_FLAG as libc::c_int | PITCH_FLAG as libc::c_int))
                    as uint8_t
        }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn rcSmoothingIsEnabled() -> bool {
    return !((*rxConfig()).rc_smoothing_type as libc::c_int ==
                 RC_SMOOTHING_TYPE_INTERPOLATION as libc::c_int &&
                 (*rxConfig()).rcInterpolation as libc::c_int ==
                     RC_SMOOTHING_OFF as libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn rcSmoothingGetValue(mut whichValue: libc::c_int)
 -> libc::c_int {
    match whichValue {
        0 => { return rcSmoothingData.inputCutoffFrequency as libc::c_int }
        1 => {
            return rcSmoothingData.derivativeCutoffFrequency as libc::c_int
        }
        2 => { return rcSmoothingData.averageFrameTimeUs }
        _ => { return 0 as libc::c_int }
    };
}
#[no_mangle]
pub unsafe extern "C" fn rcSmoothingInitializationComplete() -> bool {
    return (*rxConfig()).rc_smoothing_type as libc::c_int !=
               RC_SMOOTHING_TYPE_FILTER as libc::c_int ||
               rcSmoothingData.filterInitialized as libc::c_int != 0;
}
// USE_RC_SMOOTHING_FILTER
