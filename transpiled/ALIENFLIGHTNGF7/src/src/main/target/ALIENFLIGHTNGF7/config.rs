use ::libc;
extern "C" {
    #[no_mangle]
    fn featureSet(mask: uint32_t);
    #[no_mangle]
    static mut hardwareMotorType: uint8_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut customMotorMixer_SystemArray: [motorMixer_t; 8];
    #[no_mangle]
    static mut pidProfiles_SystemArray: [pidProfile_t; 3];
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn findSerialPortIndexByIdentifier(identifier: serialPortIdentifier_e)
     -> libc::c_int;
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    static mut hardwareRevision: uint8_t;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_0 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_0 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_0 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_0 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_0 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_0 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_0 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_0 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_0 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_0 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_0 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_0 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_0 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_0 = 8192;
pub const FEATURE_3D: C2RustUnnamed_0 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_0 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_0 = 512;
pub const FEATURE_GPS: C2RustUnnamed_0 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_0 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_0 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_0 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_0 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_0 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_0 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const MOTOR_BRUSHLESS: C2RustUnnamed_1 = 2;
pub const MOTOR_BRUSHED: C2RustUnnamed_1 = 1;
pub const MOTOR_UNKNOWN: C2RustUnnamed_1 = 0;
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
pub type motorMixer_t = motorMixer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorMixer_s {
    pub throttle: libc::c_float,
    pub roll: libc::c_float,
    pub pitch: libc::c_float,
    pub yaw: libc::c_float,
}
pub type pidProfile_t = pidProfile_s;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
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
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub type telemetryConfig_t = telemetryConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct telemetryConfig_s {
    pub gpsNoFixLatitude: int16_t,
    pub gpsNoFixLongitude: int16_t,
    pub telemetry_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub frsky_coordinate_format: frskyGpsCoordFormat_e,
    pub frsky_unit: frskyUnit_e,
    pub frsky_vfas_precision: uint8_t,
    pub hottAlarmSoundInterval: uint8_t,
    pub pidValuesAsTelemetry: uint8_t,
    pub report_cell_voltage: uint8_t,
    pub flysky_sensors: [uint8_t; 15],
    pub smartport_use_extra_sensors: uint8_t,
}
pub type frskyUnit_e = libc::c_uint;
pub const FRSKY_UNIT_IMPERIALS: frskyUnit_e = 1;
pub const FRSKY_UNIT_METRICS: frskyUnit_e = 0;
pub type frskyGpsCoordFormat_e = libc::c_uint;
pub const FRSKY_FORMAT_NMEA: frskyGpsCoordFormat_e = 1;
pub const FRSKY_FORMAT_DMS: frskyGpsCoordFormat_e = 0;
pub const FUNCTION_RX_SERIAL: C2RustUnnamed_3 = 64;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: C2RustUnnamed_3 = 4;
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
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
#[derive(Copy, Clone)]
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
pub type serialConfig_t = serialConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 7],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
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
pub const SERIALRX_SBUS: C2RustUnnamed_2 = 2;
pub const SERIALRX_SPEKTRUM2048: C2RustUnnamed_2 = 1;
pub const AFF7_REV_1: awf7HardwareRevision_t = 1;
pub type pidConfig_t = pidConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
pub type motorDevConfig_t = motorDevConfig_s;
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
// Processing denominator for PID controller vs gyro sampling rate
// off, on - enables pidsum runaway disarm logic
// delay in ms for "in-flight" conditions before deactivation (successful flight)
// minimum throttle percent required during deactivation phase
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
/*
  DshotSettingRequest (KISS24). Spin direction, 3d and save Settings reqire 10 requests.. and the TLM Byte must always be high if 1-47 are used to send settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> negative direction
  1048 (low) - 2047 (high) -> positive direction
 */
// V2 includes settings
// Currently not implemented
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// KISS audio Stream mode on/Off
// KISS silent Mode on/Off
// 1uS
// 4uS
/* resolution + frame reset (2us) */
/* resolution + frame reset (2us) */
// function pointer used to write motors
// function pointer used after motors are written
//CAVEAT: This is used in the `motorConfig_t` parameter group, so the parameter group constraints apply
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SERIALRX_FPORT: C2RustUnnamed_2 = 12;
pub const SERIALRX_TARGET_CUSTOM: C2RustUnnamed_2 = 11;
pub const SERIALRX_SRXL: C2RustUnnamed_2 = 10;
pub const SERIALRX_CRSF: C2RustUnnamed_2 = 9;
pub const SERIALRX_JETIEXBUS: C2RustUnnamed_2 = 8;
pub const SERIALRX_IBUS: C2RustUnnamed_2 = 7;
pub const SERIALRX_XBUS_MODE_B_RJ01: C2RustUnnamed_2 = 6;
pub const SERIALRX_XBUS_MODE_B: C2RustUnnamed_2 = 5;
pub const SERIALRX_SUMH: C2RustUnnamed_2 = 4;
pub const SERIALRX_SUMD: C2RustUnnamed_2 = 3;
pub const SERIALRX_SPEKTRUM1024: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FUNCTION_LIDAR_TF: C2RustUnnamed_3 = 32768;
pub const FUNCTION_RCDEVICE: C2RustUnnamed_3 = 16384;
pub const FUNCTION_VTX_TRAMP: C2RustUnnamed_3 = 8192;
pub const FUNCTION_TELEMETRY_IBUS: C2RustUnnamed_3 = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: C2RustUnnamed_3 = 2048;
pub const FUNCTION_ESC_SENSOR: C2RustUnnamed_3 = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: C2RustUnnamed_3 = 512;
pub const FUNCTION_BLACKBOX: C2RustUnnamed_3 = 128;
pub const FUNCTION_TELEMETRY_SMARTPORT: C2RustUnnamed_3 = 32;
pub const FUNCTION_TELEMETRY_LTM: C2RustUnnamed_3 = 16;
pub const FUNCTION_TELEMETRY_HOTT: C2RustUnnamed_3 = 8;
pub const FUNCTION_GPS: C2RustUnnamed_3 = 2;
pub const FUNCTION_MSP: C2RustUnnamed_3 = 1;
pub const FUNCTION_NONE: C2RustUnnamed_3 = 0;
pub type awf7HardwareRevision_t = libc::c_uint;
pub const AFF7_REV_2: awf7HardwareRevision_t = 2;
pub const AFF7_UNKNOWN: awf7HardwareRevision_t = 0;
#[inline]
unsafe extern "C" fn motorConfigMutable() -> *mut motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn customMotorMixerMutable(mut _index: libc::c_int)
 -> *mut motorMixer_t {
    return &mut *customMotorMixer_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut motorMixer_t;
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
unsafe extern "C" fn serialConfigMutable() -> *mut serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn telemetryConfigMutable() -> *mut telemetryConfig_t {
    return &mut telemetryConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfigMutable() -> *mut batteryConfig_t {
    return &mut batteryConfig_System;
}
// 32kHz
// alternative defaults settings for AlienFlight targets
#[no_mangle]
pub unsafe extern "C" fn targetConfiguration() {
    if hardwareMotorType as libc::c_int == MOTOR_BRUSHED as libc::c_int {
        (*motorConfigMutable()).dev.motorPwmRate =
            32000 as libc::c_int as uint16_t; // REAR_R
        (*pidConfigMutable()).pid_process_denom = 1 as libc::c_int as uint8_t
    } // FRONT_R
    if hardwareRevision as libc::c_int == AFF7_REV_1 as libc::c_int {
        (*rxConfigMutable()).serialrx_provider =
            SERIALRX_SPEKTRUM2048 as libc::c_int as uint8_t; // REAR_L
        (*rxConfigMutable()).spektrum_sat_bind =
            5 as libc::c_int as uint8_t; // FRONT_L
        (*rxConfigMutable()).spektrum_sat_bind_autoreset =
            1 as libc::c_int as uint8_t
    } else {
        (*rxConfigMutable()).serialrx_provider =
            SERIALRX_SBUS as libc::c_int as uint8_t; // MIDFRONT_R
        (*rxConfigMutable()).serialrx_inverted =
            1 as libc::c_int as uint8_t; // MIDFRONT_L
        (*serialConfigMutable()).portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART2)
                                                 as usize].functionMask =
            (FUNCTION_TELEMETRY_FRSKY_HUB as libc::c_int |
                 FUNCTION_RX_SERIAL as libc::c_int) as uint16_t; // MIDREAR_R
        (*telemetryConfigMutable()).telemetry_inverted =
            0 as libc::c_int as uint8_t;
        (*batteryConfigMutable()).voltageMeterSource = VOLTAGE_METER_ADC;
        (*batteryConfigMutable()).currentMeterSource = CURRENT_METER_ADC;
        featureSet(FEATURE_TELEMETRY as libc::c_int as uint32_t);
    }
    let mut pidProfileIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (pidProfileIndex as libc::c_int) < 3 as libc::c_int {
        let mut pidProfile: *mut pidProfile_t =
            pidProfilesMutable(pidProfileIndex as libc::c_int);
        (*pidProfile).pid[FD_ROLL as libc::c_int as usize].P =
            53 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_ROLL as libc::c_int as usize].I =
            45 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_ROLL as libc::c_int as usize].D =
            52 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_PITCH as libc::c_int as usize].P =
            53 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_PITCH as libc::c_int as usize].I =
            45 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_PITCH as libc::c_int as usize].D =
            52 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_YAW as libc::c_int as usize].P =
            64 as libc::c_int as uint8_t;
        (*pidProfile).pid[FD_YAW as libc::c_int as usize].D =
            18 as libc::c_int as uint8_t;
        pidProfileIndex = pidProfileIndex.wrapping_add(1)
    }
    *customMotorMixerMutable(0 as libc::c_int) =
        {
            let mut init =
                motorMixer_s{throttle: 1.0f32,
                             roll: -0.414178f32,
                             pitch: 1.0f32,
                             yaw: -1.0f32,};
            init
        };
    *customMotorMixerMutable(1 as libc::c_int) =
        {
            let mut init =
                motorMixer_s{throttle: 1.0f32,
                             roll: -0.414178f32,
                             pitch: -1.0f32,
                             yaw: 1.0f32,};
            init
        };
    *customMotorMixerMutable(2 as libc::c_int) =
        {
            let mut init =
                motorMixer_s{throttle: 1.0f32,
                             roll: 0.414178f32,
                             pitch: 1.0f32,
                             yaw: 1.0f32,};
            init
        };
    *customMotorMixerMutable(3 as libc::c_int) =
        {
            let mut init =
                motorMixer_s{throttle: 1.0f32,
                             roll: 0.414178f32,
                             pitch: -1.0f32,
                             yaw: -1.0f32,};
            init
        };
    *customMotorMixerMutable(4 as libc::c_int) =
        {
            let mut init =
                motorMixer_s{throttle: 1.0f32,
                             roll: -1.0f32,
                             pitch: -0.414178f32,
                             yaw: -1.0f32,};
            init
        };
    *customMotorMixerMutable(5 as libc::c_int) =
        {
            let mut init =
                motorMixer_s{throttle: 1.0f32,
                             roll: 1.0f32,
                             pitch: -0.414178f32,
                             yaw: 1.0f32,};
            init
        };
    *customMotorMixerMutable(6 as libc::c_int) =
        {
            let mut init =
                motorMixer_s{throttle: 1.0f32,
                             roll: -1.0f32,
                             pitch: 0.414178f32,
                             yaw: 1.0f32,};
            init
        };
    *customMotorMixerMutable(7 as libc::c_int) =
        {
            let mut init =
                motorMixer_s{throttle: 1.0f32,
                             roll: 1.0f32,
                             pitch: 0.414178f32,
                             yaw: -1.0f32,};
            init
        };
    // MIDREAR_L
}
