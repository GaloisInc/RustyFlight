use ::libc;
extern "C" {
    #[no_mangle]
    static mut blackboxConfig_System: blackboxConfig_t;
    #[no_mangle]
    fn featureSet(mask: uint32_t);
    #[no_mangle]
    static mut hardwareMotorType: uint8_t;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    static mut controlRateProfiles_SystemArray: [controlRateConfig_t; 6];
    #[no_mangle]
    static mut imuConfig_System: imuConfig_t;
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut pidProfiles_SystemArray: [pidProfile_t; 3];
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn parseRcChannels(input: *const libc::c_char, rxConfig: *mut rxConfig_s);
    #[no_mangle]
    static mut barometerConfig_System: barometerConfig_t;
    #[no_mangle]
    static mut compassConfig_System: compassConfig_t;
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
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
// Add new event type for flight mode status.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
pub type C2RustUnnamed = libc::c_uint;
pub const FD_YAW: C2RustUnnamed = 2;
pub const FD_PITCH: C2RustUnnamed = 1;
pub const FD_ROLL: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const FILTER_BIQUAD: C2RustUnnamed_0 = 1;
pub const FILTER_PT1: C2RustUnnamed_0 = 0;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const MOTOR_BRUSHLESS: C2RustUnnamed_2 = 2;
pub const MOTOR_BRUSHED: C2RustUnnamed_2 = 1;
pub const MOTOR_UNKNOWN: C2RustUnnamed_2 = 0;
#[derive(Copy, Clone)]
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
pub const THROTTLE_LIMIT_TYPE_CLIP: C2RustUnnamed_3 = 2;
pub type pidProfile_t = pidProfile_s;
pub const PID_ROLL: C2RustUnnamed_5 = 0;
pub const PID_PITCH: C2RustUnnamed_5 = 1;
pub const PID_LEVEL: C2RustUnnamed_5 = 3;
pub const PID_YAW: C2RustUnnamed_5 = 2;
pub type pidConfig_t = pidConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
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
pub const MAG_NONE: C2RustUnnamed_8 = 1;
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
// in seconds
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
// Processing denominator for PID controller vs gyro sampling rate
// off, on - enables pidsum runaway disarm logic
// delay in ms for "in-flight" conditions before deactivation (successful flight)
// minimum throttle percent required during deactivation phase
// gyro alignment
// people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
// Gyro sample divider
// gyro DLPF setting
// gyro 32khz DLPF setting
// Lowpass primary/secondary
// Gyro calibration duration in 1/100 second
// bandpass quality factor, 100 for steep sided bandpass
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
pub const BARO_NONE: C2RustUnnamed_7 = 1;
pub type barometerConfig_t = barometerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct barometerConfig_s {
    pub baro_bustype: uint8_t,
    pub baro_spi_device: uint8_t,
    pub baro_spi_csn: ioTag_t,
    pub baro_i2c_device: uint8_t,
    pub baro_i2c_address: uint8_t,
    pub baro_hardware: uint8_t,
    pub baro_sample_count: uint8_t,
    pub baro_noise_lpf: uint16_t,
    pub baro_cf_vel: uint16_t,
    pub baro_cf_alt: uint16_t,
}
pub type imuConfig_t = imuConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct imuConfig_s {
    pub dcm_kp: uint16_t,
    pub dcm_ki: uint16_t,
    pub small_angle: uint8_t,
    pub acc_unarmedcal: uint8_t,
    pub accDeadband: accDeadband_t,
}
pub type accDeadband_t = accDeadband_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accDeadband_s {
    pub xy: uint8_t,
    pub z: uint8_t,
}
pub type mixerConfig_t = mixerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
// Also used as XCLR (positive logic) for BMP085
// Barometer hardware to use
// size of baro filter array
// additional LPF to reduce baro noise
// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
// apply CF to use ACC for height estimation
// DCM filter proportional gain ( x 10000)
// DCM filter integral gain ( x 10000)
// turn automatic acc compensation on/off
// set the acc deadband for xy-Axis
// set the acc deadband for z-Axis, this ignores small accelerations
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
pub const RC_SMOOTHING_MANUAL: C2RustUnnamed_4 = 3;
pub const SERIALRX_SPEKTRUM2048: C2RustUnnamed_6 = 1;
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const THROTTLE_LIMIT_TYPE_SCALE: C2RustUnnamed_3 = 1;
pub const THROTTLE_LIMIT_TYPE_OFF: C2RustUnnamed_3 = 0;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const RC_SMOOTHING_AUTO: C2RustUnnamed_4 = 2;
pub const RC_SMOOTHING_DEFAULT: C2RustUnnamed_4 = 1;
pub const RC_SMOOTHING_OFF: C2RustUnnamed_4 = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_5 = 5;
pub const PID_MAG: C2RustUnnamed_5 = 4;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const SERIALRX_FPORT: C2RustUnnamed_6 = 12;
pub const SERIALRX_TARGET_CUSTOM: C2RustUnnamed_6 = 11;
pub const SERIALRX_SRXL: C2RustUnnamed_6 = 10;
pub const SERIALRX_CRSF: C2RustUnnamed_6 = 9;
pub const SERIALRX_JETIEXBUS: C2RustUnnamed_6 = 8;
pub const SERIALRX_IBUS: C2RustUnnamed_6 = 7;
pub const SERIALRX_XBUS_MODE_B_RJ01: C2RustUnnamed_6 = 6;
pub const SERIALRX_XBUS_MODE_B: C2RustUnnamed_6 = 5;
pub const SERIALRX_SUMH: C2RustUnnamed_6 = 4;
pub const SERIALRX_SUMD: C2RustUnnamed_6 = 3;
pub const SERIALRX_SBUS: C2RustUnnamed_6 = 2;
pub const SERIALRX_SPEKTRUM1024: C2RustUnnamed_6 = 0;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const BARO_QMP6988: C2RustUnnamed_7 = 6;
pub const BARO_LPS: C2RustUnnamed_7 = 5;
pub const BARO_BMP280: C2RustUnnamed_7 = 4;
pub const BARO_MS5611: C2RustUnnamed_7 = 3;
pub const BARO_BMP085: C2RustUnnamed_7 = 2;
pub const BARO_DEFAULT: C2RustUnnamed_7 = 0;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const MAG_QMC5883: C2RustUnnamed_8 = 5;
pub const MAG_AK8963: C2RustUnnamed_8 = 4;
pub const MAG_AK8975: C2RustUnnamed_8 = 3;
pub const MAG_HMC5883: C2RustUnnamed_8 = 2;
pub const MAG_DEFAULT: C2RustUnnamed_8 = 0;
#[inline]
unsafe extern "C" fn blackboxConfigMutable() -> *mut blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfigMutable() -> *mut systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn controlRateProfilesMutable(mut _index: libc::c_int)
 -> *mut controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
}
#[inline]
unsafe extern "C" fn imuConfigMutable() -> *mut imuConfig_t {
    return &mut imuConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfigMutable() -> *mut motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn mixerConfigMutable() -> *mut mixerConfig_t {
    return &mut mixerConfig_System;
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
unsafe extern "C" fn rxConfigMutable() -> *mut rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn barometerConfigMutable() -> *mut barometerConfig_t {
    return &mut barometerConfig_System;
}
#[inline]
unsafe extern "C" fn compassConfigMutable() -> *mut compassConfig_t {
    return &mut compassConfig_System;
}
#[inline]
unsafe extern "C" fn gyroConfigMutable() -> *mut gyroConfig_t {
    return &mut gyroConfig_System;
}
#[no_mangle]
pub unsafe extern "C" fn targetConfiguration() {
    if hardwareMotorType as libc::c_int == MOTOR_BRUSHED as libc::c_int {
        (*motorConfigMutable()).dev.motorPwmRate =
            32000 as libc::c_int as uint16_t;
        (*motorConfigMutable()).minthrottle = 1050 as libc::c_int as uint16_t
        // The update rate of motor outputs (50-498Hz)
        // Pwm Protocol
        // Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
        // for 6mm and 7mm brushed
    }
    /* Default to Spektrum */
    (*rxConfigMutable()).serialrx_provider =
        SERIALRX_SPEKTRUM2048 as libc::c_int as
            uint8_t; // all DSM* except DSM2 22ms
    (*rxConfigMutable()).spektrum_sat_bind =
        5 as libc::c_int as uint8_t; // DSM2 11ms
    (*rxConfigMutable()).spektrum_sat_bind_autoreset =
        1 as libc::c_int as uint8_t;
    (*rxConfigMutable()).mincheck = 1025 as libc::c_int as uint16_t;
    (*rxConfigMutable()).rcInterpolation =
        RC_SMOOTHING_MANUAL as libc::c_int as uint8_t;
    (*rxConfigMutable()).rcInterpolationInterval =
        14 as libc::c_int as uint8_t;
    parseRcChannels(b"TAER1234\x00" as *const u8 as *const libc::c_char,
                    rxConfigMutable());
    (*mixerConfigMutable()).yaw_motors_reversed = 1 as libc::c_int != 0;
    (*imuConfigMutable()).small_angle = 180 as libc::c_int as uint8_t;
    (*blackboxConfigMutable()).p_ratio = 128 as libc::c_int as uint16_t;
    /* Breadboard-specific settings for development purposes only
     */
    (*barometerConfigMutable()).baro_hardware =
        BARO_NONE as libc::c_int as uint8_t; //216MHZ
    (*compassConfigMutable()).mag_hardware =
        MAG_NONE as libc::c_int as uint8_t;
    (*systemConfigMutable()).cpu_overclock = 2 as libc::c_int as uint8_t;
    /* Default to 32kHz enabled at 16/16 */
    (*gyroConfigMutable()).gyro_use_32khz =
        1 as libc::c_int as uint8_t; // enable 32kHz sampling
    (*gyroConfigMutable()).gyroMovementCalibrationThreshold =
        200 as libc::c_int as uint8_t; // aka moron_threshold
    (*gyroConfigMutable()).gyro_sync_denom =
        2 as libc::c_int as uint8_t; // 16kHz gyro
    (*pidConfigMutable()).pid_process_denom =
        1 as libc::c_int as uint8_t; // 16kHz PID
    (*gyroConfigMutable()).gyro_lowpass2_hz = 751 as libc::c_int as uint16_t;
    (*pidConfigMutable()).runaway_takeoff_prevention =
        0 as libc::c_int as uint8_t;
    featureSet(((FEATURE_DYNAMIC_FILTER as libc::c_int |
                     FEATURE_AIRMODE as libc::c_int |
                     FEATURE_ANTI_GRAVITY as libc::c_int) ^
                    FEATURE_RX_PARALLEL_PWM as libc::c_int) as uint32_t);
    /* AlienWhoop PIDs tested with 6mm and 7mm motors on most frames */
    let mut pidProfileIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (pidProfileIndex as libc::c_int) < 3 as libc::c_int {
        let mut pidProfile: *mut pidProfile_t =
            pidProfilesMutable(pidProfileIndex as libc::c_int);
        (*pidProfile).pidSumLimit = 1000 as libc::c_int as uint16_t;
        (*pidProfile).pidSumLimitYaw = 1000 as libc::c_int as uint16_t;
        /* AlienWhoop PIDs tested with 6mm and 7mm motors on most frames */
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].P =
            115 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].I =
            75 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].D =
            95 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].P =
            110 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].I =
            75 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].D =
            85 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_YAW as libc::c_int as usize].P =
            220 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_YAW as libc::c_int as usize].I =
            75 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_YAW as libc::c_int as usize].D =
            20 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].P =
            65 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].I =
            65 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].D =
            55 as libc::c_int as uint8_t;
        /* Setpoints */
        (*pidProfile).dterm_filter_type =
            FILTER_BIQUAD as libc::c_int as uint8_t;
        (*pidProfile).dterm_notch_hz = 0 as libc::c_int as uint16_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].F =
            100 as libc::c_int as uint16_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].F =
            100 as libc::c_int as uint16_t;
        (*pidProfile).feedForwardTransition = 0 as libc::c_int as uint8_t;
        /* Anti-Gravity */
        (*pidProfile).itermThrottleThreshold = 500 as libc::c_int as uint16_t;
        (*pidProfile).itermAcceleratorGain = 5000 as libc::c_int as uint16_t;
        (*pidProfile).levelAngleLimit = 65 as libc::c_int as uint8_t;
        pidProfileIndex = pidProfileIndex.wrapping_add(1)
    }
    let mut rateProfileIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (rateProfileIndex as libc::c_int) < 6 as libc::c_int {
        let mut controlRateConfig: *mut controlRateConfig_t =
            controlRateProfilesMutable(rateProfileIndex as libc::c_int);
        // 20% throttle expo
        (*controlRateConfig).rcRates[FD_ROLL as libc::c_int as usize] =
            218 as libc::c_int as uint8_t;
        (*controlRateConfig).rcRates[FD_PITCH as libc::c_int as usize] =
            218 as libc::c_int as uint8_t;
        (*controlRateConfig).rcRates[FD_YAW as libc::c_int as usize] =
            218 as libc::c_int as uint8_t;
        (*controlRateConfig).rcExpo[FD_ROLL as libc::c_int as usize] =
            45 as libc::c_int as uint8_t;
        (*controlRateConfig).rcExpo[FD_PITCH as libc::c_int as usize] =
            45 as libc::c_int as uint8_t;
        (*controlRateConfig).rcExpo[FD_YAW as libc::c_int as usize] =
            45 as libc::c_int as uint8_t;
        (*controlRateConfig).rates[FD_ROLL as libc::c_int as usize] =
            0 as libc::c_int as uint8_t;
        (*controlRateConfig).rates[FD_PITCH as libc::c_int as usize] =
            0 as libc::c_int as uint8_t;
        (*controlRateConfig).rates[FD_YAW as libc::c_int as usize] =
            0 as libc::c_int as uint8_t;
        (*controlRateConfig).dynThrPID = 0 as libc::c_int as uint8_t;
        (*controlRateConfig).tpa_breakpoint = 1600 as libc::c_int as uint16_t;
        (*controlRateConfig).throttle_limit_type =
            THROTTLE_LIMIT_TYPE_CLIP as libc::c_int as uint8_t;
        (*controlRateConfig).thrExpo8 = 20 as libc::c_int as uint8_t;
        rateProfileIndex = rateProfileIndex.wrapping_add(1)
    };
}
/* RC Rates */
/* Classic Expo */
/* Super Expo Rates */
/* Throttle PID Attenuation (TPA) */
// tpa_rate off
/* Force the clipping mixer at 100% seems better for brushed than default (off) and scaling)? */
//controlRateConfig->throttle_limit_percent = 100;
