use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn biquadFilterInitLPF(filter: *mut biquadFilter_t,
                           filterFreq: libc::c_float, refreshRate: uint32_t);
    #[no_mangle]
    fn biquadFilterApply(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn scaleRange(x: libc::c_int, srcFrom: libc::c_int, srcTo: libc::c_int,
                  destFrom: libc::c_int, destTo: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn pwmWriteServo(index: uint8_t, value: libc::c_float);
    #[no_mangle]
    fn timerioTagGetByUsage(usageFlag: timerUsageFlag_e, index: uint8_t)
     -> ioTag_t;
    // (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    static mixers: [mixer_t; 0];
    #[no_mangle]
    static mut motor: [libc::c_float; 8];
    #[no_mangle]
    fn mixerIsTricopter() -> bool;
    #[no_mangle]
    static mut pidData: [pidAxisData_t; 3];
    #[no_mangle]
    static mut targetPidLooptime: uint32_t;
    // tricopter specific
    #[no_mangle]
    fn servosTricopterInit();
    #[no_mangle]
    fn servosTricopterMixer();
    #[no_mangle]
    fn servosTricopterIsEnabledServoUnarmed() -> bool;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    static mut rxRuntimeConfig: rxRuntimeConfig_t;
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
    static mut currentMixerMode: mixerMode_e;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoDevConfig_s {
    pub servoCenterPulse: uint16_t,
    pub servoPwmRate: uint16_t,
    pub ioTags: [ioTag_t; 8],
}
pub type servoDevConfig_t = servoDevConfig_s;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_2 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_2 = 2;
pub const ARMED: C2RustUnnamed_2 = 1;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_3 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_3 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_3 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_3 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_3 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_3 = 16;
pub const BARO_MODE: C2RustUnnamed_3 = 8;
pub const MAG_MODE: C2RustUnnamed_3 = 4;
pub const HORIZON_MODE: C2RustUnnamed_3 = 2;
pub const ANGLE_MODE: C2RustUnnamed_3 = 1;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_4 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_4 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_4 = 4;
pub const GPS_FIX: C2RustUnnamed_4 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_4 = 1;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const FD_YAW: C2RustUnnamed_5 = 2;
pub const FD_PITCH: C2RustUnnamed_5 = 1;
pub const FD_ROLL: C2RustUnnamed_5 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_6,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_6 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
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
// Digital protocol has fixed values
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
pub struct pidAxisData_s {
    pub P: libc::c_float,
    pub I: libc::c_float,
    pub D: libc::c_float,
    pub F: libc::c_float,
    pub Sum: libc::c_float,
}
pub type pidAxisData_t = pidAxisData_s;
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
pub type servoIndex_e = libc::c_uint;
pub const SERVO_HELI_RUD: servoIndex_e = 3;
pub const SERVO_HELI_TOP: servoIndex_e = 2;
pub const SERVO_HELI_RIGHT: servoIndex_e = 1;
pub const SERVO_HELI_LEFT: servoIndex_e = 0;
pub const SERVO_SINGLECOPTER_4: servoIndex_e = 6;
pub const SERVO_SINGLECOPTER_3: servoIndex_e = 5;
pub const SERVO_SINGLECOPTER_2: servoIndex_e = 4;
pub const SERVO_SINGLECOPTER_1: servoIndex_e = 3;
pub const SERVO_DUALCOPTER_RIGHT: servoIndex_e = 5;
pub const SERVO_DUALCOPTER_LEFT: servoIndex_e = 4;
pub const SERVO_BICOPTER_RIGHT: servoIndex_e = 5;
pub const SERVO_BICOPTER_LEFT: servoIndex_e = 4;
pub const SERVO_THROTTLE: servoIndex_e = 7;
pub const SERVO_ELEVATOR: servoIndex_e = 6;
pub const SERVO_RUDDER: servoIndex_e = 5;
pub const SERVO_FLAPPERON_2: servoIndex_e = 4;
pub const SERVO_FLAPPERON_1: servoIndex_e = 3;
pub const SERVO_FLAPS: servoIndex_e = 2;
pub const SERVO_GIMBAL_ROLL: servoIndex_e = 1;
pub const SERVO_GIMBAL_PITCH: servoIndex_e = 0;
#[derive(Copy, Clone)]
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
pub type servoMixer_t = servoMixer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixerRules_s {
    pub servoRuleCount: uint8_t,
    pub rule: *const servoMixer_t,
}
// servo that receives the output of the rule
// input channel for this rule
// range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
// reduces the speed of the rule, 0=unlimited speed
// lower bound of rule range [0;100]% of servo max-min
// lower bound of rule range [0;100]% of servo max-min
// active rule if box is enabled, range [0;3], 0=no box, 1=BOXSERVO1, 2=BOXSERVO2, 3=BOXSERVO3
// Custom mixer configuration
pub type mixerRules_t = mixerRules_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoParam_s {
    pub reversedSources: uint32_t,
    pub min: int16_t,
    pub max: int16_t,
    pub middle: int16_t,
    pub rate: int8_t,
    pub forwardFromChannel: int8_t,
}
pub type servoParam_t = servoParam_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoConfig_s {
    pub dev: servoDevConfig_t,
    pub servo_lowpass_freq: uint16_t,
    pub tri_unarmed_servo: uint8_t,
    pub channelForwardingStartChannel: uint8_t,
}
pub type servoConfig_t = servoConfig_s;
pub const GIMBAL_MODE_MIXTILT: C2RustUnnamed_8 = 1;
pub type gimbalConfig_t = gimbalConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gimbalConfig_s {
    pub mode: uint8_t,
}
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxRuntimeConfig_s {
    pub channelCount: uint8_t,
    pub rxRefreshRate: uint16_t,
    pub rcReadRawFn: rcReadRawDataFnPtr,
    pub rcFrameStatusFn: rcFrameStatusFnPtr,
    pub rcProcessFrameFn: rcProcessFrameFnPtr,
    pub channelData: *mut uint16_t,
    pub frameData: *mut libc::c_void,
}
pub type rcProcessFrameFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s) -> bool>;
// the direction of servo movement for each input source of the servo mixer, bit set=inverted
// servo min
// servo max
// servo middle
// range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
// RX channel index, 0 based.  See CHANNEL_FORWARDING_DISABLED
// lowpass servo filter frequency selection; 1/1000ths of loop freq
// send tail servo correction pulses even when unarmed
// number of RC channels as reported by current input driver
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const GIMBAL_MODE_NORMAL: C2RustUnnamed_8 = 0;
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
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_7 = INPUT_STABILIZED_ROLL;
#[inline]
unsafe extern "C" fn customServoMixersMutable(mut _index: libc::c_int)
 -> *mut servoMixer_t {
    return &mut *customServoMixers_SystemArray.as_mut_ptr().offset(_index as
                                                                       isize)
               as *mut servoMixer_t;
}
#[inline]
unsafe extern "C" fn customServoMixers(mut _index: libc::c_int)
 -> *const servoMixer_t {
    return &mut *customServoMixers_SystemArray.as_mut_ptr().offset(_index as
                                                                       isize)
               as *mut servoMixer_t;
}
#[inline]
unsafe extern "C" fn servoParams(mut _index: libc::c_int)
 -> *const servoParam_t {
    return &mut *servoParams_SystemArray.as_mut_ptr().offset(_index as isize)
               as *mut servoParam_t;
}
#[inline]
unsafe extern "C" fn servoConfig() -> *const servoConfig_t {
    return &mut servoConfig_System;
}
#[inline]
unsafe extern "C" fn gimbalConfig() -> *const gimbalConfig_t {
    return &mut gimbalConfig_System;
}
#[no_mangle]
pub static mut servoConfig_System: servoConfig_t =
    servoConfig_t{dev:
                      servoDevConfig_t{servoCenterPulse: 0,
                                       servoPwmRate: 0,
                                       ioTags: [0; 8],},
                  servo_lowpass_freq: 0,
                  tri_unarmed_servo: 0,
                  channelForwardingStartChannel: 0,};
#[no_mangle]
pub static mut servoConfig_Copy: servoConfig_t =
    servoConfig_t{dev:
                      servoDevConfig_t{servoCenterPulse: 0,
                                       servoPwmRate: 0,
                                       ioTags: [0; 8],},
                  servo_lowpass_freq: 0,
                  tri_unarmed_servo: 0,
                  channelForwardingStartChannel: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut servoConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (52 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<servoConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &servoConfig_System as *const servoConfig_t
                                     as *mut servoConfig_t as *mut uint8_t,
                             copy:
                                 &servoConfig_Copy as *const servoConfig_t as
                                     *mut servoConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut servoConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_servoConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut servoConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_servoConfig(mut servoConfig_0:
                                                   *mut servoConfig_t) {
    (*servoConfig_0).dev.servoCenterPulse = 1500 as libc::c_int as uint16_t;
    (*servoConfig_0).dev.servoPwmRate = 50 as libc::c_int as uint16_t;
    (*servoConfig_0).tri_unarmed_servo = 1 as libc::c_int as uint8_t;
    (*servoConfig_0).servo_lowpass_freq = 0 as libc::c_int as uint16_t;
    (*servoConfig_0).channelForwardingStartChannel =
        AUX1 as libc::c_int as uint8_t;
    let mut servoIndex: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while servoIndex < 8 as libc::c_int as libc::c_uint {
        (*servoConfig_0).dev.ioTags[servoIndex as usize] =
            timerioTagGetByUsage(TIM_USE_SERVO, servoIndex as uint8_t);
        servoIndex = servoIndex.wrapping_add(1)
    };
}
#[no_mangle]
pub static mut customServoMixers_SystemArray: [servoMixer_t; 16] =
    [servoMixer_t{targetChannel: 0,
                  inputSource: 0,
                  rate: 0,
                  speed: 0,
                  min: 0,
                  max: 0,
                  box_0: 0,}; 16];
#[no_mangle]
pub static mut customServoMixers_CopyArray: [servoMixer_t; 16] =
    [servoMixer_t{targetChannel: 0,
                  inputSource: 0,
                  rate: 0,
                  speed: 0,
                  min: 0,
                  max: 0,
                  box_0: 0,}; 16];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut customServoMixers_Registry: pgRegistry_t =
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
pub static mut servoParams_SystemArray: [servoParam_t; 8] =
    [servoParam_t{reversedSources: 0,
                  min: 0,
                  max: 0,
                  middle: 0,
                  rate: 0,
                  forwardFromChannel: 0,}; 8];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut servoParams_Registry: pgRegistry_t =
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
pub static mut servoParams_CopyArray: [servoParam_t; 8] =
    [servoParam_t{reversedSources: 0,
                  min: 0,
                  max: 0,
                  middle: 0,
                  rate: 0,
                  forwardFromChannel: 0,}; 8];
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_servoParams(mut instance:
                                                   *mut servoParam_t) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        static mut _reset_template_88: servoParam_t =
            {
                let mut init =
                    servoParam_s{reversedSources: 0,
                                 min: 1000 as libc::c_int as int16_t,
                                 max: 2000 as libc::c_int as int16_t,
                                 middle: 1500 as libc::c_int as int16_t,
                                 rate: 100 as libc::c_int as int8_t,
                                 forwardFromChannel:
                                     0xff as libc::c_int as uint8_t as
                                         int8_t,};
                init
            };
        memcpy(&mut *instance.offset(i as isize) as *mut servoParam_t as
                   *mut libc::c_void,
               &_reset_template_88 as *const servoParam_t as
                   *const libc::c_void,
               ::core::mem::size_of::<servoParam_t>() as libc::c_ulong);
        i += 1
    };
}
// no template required since default is zero
#[no_mangle]
pub static mut gimbalConfig_System: gimbalConfig_t = gimbalConfig_t{mode: 0,};
#[no_mangle]
pub static mut gimbalConfig_Copy: gimbalConfig_t = gimbalConfig_t{mode: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut gimbalConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (3 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<gimbalConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &gimbalConfig_System as *const gimbalConfig_t
                                     as *mut gimbalConfig_t as *mut uint8_t,
                             copy:
                                 &gimbalConfig_Copy as *const gimbalConfig_t
                                     as *mut gimbalConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     0 as *const libc::c_void
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut servo: [int16_t; 8] = [0; 8];
static mut servoRuleCount: uint8_t = 0 as libc::c_int as uint8_t;
static mut currentServoMixer: [servoMixer_t; 16] =
    [servoMixer_t{targetChannel: 0,
                  inputSource: 0,
                  rate: 0,
                  speed: 0,
                  min: 0,
                  max: 0,
                  box_0: 0,}; 16];
static mut useServo: libc::c_int = 0;
// mixer rule format servo, input, rate, speed, min, max, box
static mut servoMixerAirplane: [servoMixer_t; 5] =
    [{
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_FLAPPERON_1 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_FLAPPERON_2 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_RUDDER as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_ELEVATOR as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_THROTTLE as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_THROTTLE as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     }];
static mut servoMixerFlyingWing: [servoMixer_t; 5] =
    [{
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_FLAPPERON_1 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_FLAPPERON_1 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_FLAPPERON_2 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: -(100 as libc::c_int) as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_FLAPPERON_2 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_THROTTLE as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_THROTTLE as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     }];
static mut servoMixerTri: [servoMixer_t; 1] =
    [{
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_RUDDER as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     }];
static mut servoMixerBI: [servoMixer_t; 4] =
    [{
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_BICOPTER_LEFT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_BICOPTER_LEFT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: -(100 as libc::c_int) as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_BICOPTER_RIGHT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_BICOPTER_RIGHT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     }];
static mut servoMixerDual: [servoMixer_t; 2] =
    [{
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_DUALCOPTER_LEFT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_DUALCOPTER_RIGHT as libc::c_int as
                                  uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     }];
static mut servoMixerSingle: [servoMixer_t; 8] =
    [{
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_SINGLECOPTER_1 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_SINGLECOPTER_1 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_SINGLECOPTER_2 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_SINGLECOPTER_2 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_SINGLECOPTER_3 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_SINGLECOPTER_3 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_SINGLECOPTER_4 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_SINGLECOPTER_4 as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     }];
static mut servoMixerHeli: [servoMixer_t; 9] =
    [{
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_LEFT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: -(50 as libc::c_int) as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_LEFT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: -(87 as libc::c_int) as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_LEFT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_RC_AUX1 as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_RIGHT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: -(50 as libc::c_int) as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_RIGHT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_ROLL as libc::c_int as uint8_t,
                          rate: 87 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_RIGHT as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_RC_AUX1 as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_TOP as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_PITCH as libc::c_int as
                                  uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_TOP as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_RC_AUX1 as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_HELI_RUD as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_STABILIZED_YAW as libc::c_int as uint8_t,
                          rate: 100 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     }];
// USE_UNCOMMON_MIXERS
static mut servoMixerGimbal: [servoMixer_t; 2] =
    [{
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_GIMBAL_PITCH as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_GIMBAL_PITCH as libc::c_int as uint8_t,
                          rate: 125 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             servoMixer_s{targetChannel:
                              SERVO_GIMBAL_ROLL as libc::c_int as uint8_t,
                          inputSource:
                              INPUT_GIMBAL_ROLL as libc::c_int as uint8_t,
                          rate: 125 as libc::c_int as int8_t,
                          speed: 0 as libc::c_int as uint8_t,
                          min: 0 as libc::c_int as int8_t,
                          max: 100 as libc::c_int as int8_t,
                          box_0: 0 as libc::c_int as uint8_t,};
         init
     }];
// Initialized in run_static_initializers
#[no_mangle]
pub static mut servoMixers: [mixerRules_t; 27] =
    [mixerRules_t{servoRuleCount: 0, rule: 0 as *const servoMixer_t,}; 27];
#[no_mangle]
pub unsafe extern "C" fn determineServoMiddleOrForwardFromChannel(mut servoIndex:
                                                                      servoIndex_e)
 -> int16_t {
    let channelToForwardFrom: uint8_t =
        (*servoParams(servoIndex as libc::c_int)).forwardFromChannel as
            uint8_t;
    if channelToForwardFrom as libc::c_int !=
           0xff as libc::c_int as uint8_t as libc::c_int &&
           (channelToForwardFrom as libc::c_int) <
               rxRuntimeConfig.channelCount as libc::c_int {
        return rcData[channelToForwardFrom as usize]
    }
    return (*servoParams(servoIndex as libc::c_int)).middle;
}
#[no_mangle]
pub unsafe extern "C" fn servoDirection(mut servoIndex: libc::c_int,
                                        mut inputSource: libc::c_int)
 -> libc::c_int {
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (*servoParams(servoIndex)).reversedSources &
           ((1 as libc::c_int) << inputSource) as libc::c_uint != 0 {
        return -(1 as libc::c_int)
    } else { return 1 as libc::c_int };
}
#[no_mangle]
pub unsafe extern "C" fn servosInit() {
    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo =
        (*mixers.as_ptr().offset(currentMixerMode as isize)).useServo as
            libc::c_int;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if feature(FEATURE_SERVO_TILT as libc::c_int as uint32_t) as libc::c_int
           != 0 ||
           feature(FEATURE_CHANNEL_FORWARDING as libc::c_int as uint32_t) as
               libc::c_int != 0 {
        useServo = 1 as libc::c_int
    }
    // give all servos a default command
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 8 as libc::c_int {
        servo[i as usize] = 1500 as libc::c_int as int16_t;
        i = i.wrapping_add(1)
    }
    if mixerIsTricopter() { servosTricopterInit(); };
}
#[no_mangle]
pub unsafe extern "C" fn loadCustomServoMixer() {
    // reset settings
    servoRuleCount = 0 as libc::c_int as uint8_t;
    memset(currentServoMixer.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<[servoMixer_t; 16]>() as libc::c_ulong);
    // load custom mixer into currentServoMixer
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 2 as libc::c_int * 8 as libc::c_int {
        // check if done
        if (*customServoMixers(i)).rate as libc::c_int == 0 as libc::c_int {
            break ;
        }
        currentServoMixer[i as usize] = *customServoMixers(i);
        servoRuleCount = servoRuleCount.wrapping_add(1);
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn servoConfigureOutput() {
    if useServo != 0 {
        servoRuleCount =
            servoMixers[currentMixerMode as usize].servoRuleCount;
        if !servoMixers[currentMixerMode as usize].rule.is_null() {
            let mut i: libc::c_int = 0 as libc::c_int;
            while i < servoRuleCount as libc::c_int {
                currentServoMixer[i as usize] =
                    *servoMixers[currentMixerMode as
                                     usize].rule.offset(i as isize);
                i += 1
            }
        }
    }
    // set flag that we're on something with wings
    if currentMixerMode as libc::c_uint ==
           MIXER_FLYING_WING as libc::c_int as libc::c_uint ||
           currentMixerMode as libc::c_uint ==
               MIXER_AIRPLANE as libc::c_int as libc::c_uint ||
           currentMixerMode as libc::c_uint ==
               MIXER_CUSTOM_AIRPLANE as libc::c_int as libc::c_uint {
        stateFlags =
            (stateFlags as libc::c_int | FIXED_WING as libc::c_int) as
                uint8_t;
        if currentMixerMode as libc::c_uint ==
               MIXER_CUSTOM_AIRPLANE as libc::c_int as libc::c_uint {
            loadCustomServoMixer();
        }
    } else {
        stateFlags =
            (stateFlags as libc::c_int & !(FIXED_WING as libc::c_int)) as
                uint8_t;
        if currentMixerMode as libc::c_uint ==
               MIXER_CUSTOM_TRI as libc::c_int as libc::c_uint {
            loadCustomServoMixer();
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn servoMixerLoadMix(mut index: libc::c_int) {
    // we're 1-based
    index += 1;
    // clear existing
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 2 as libc::c_int * 8 as libc::c_int {
        let ref mut fresh0 = (*customServoMixersMutable(i)).box_0;
        *fresh0 = 0 as libc::c_int as uint8_t;
        let ref mut fresh1 = (*customServoMixersMutable(i)).rate;
        *fresh1 = *fresh0 as int8_t;
        let ref mut fresh2 = (*customServoMixersMutable(i)).inputSource;
        *fresh2 = *fresh1 as uint8_t;
        (*customServoMixersMutable(i)).targetChannel = *fresh2;
        i += 1
    }
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < servoMixers[index as usize].servoRuleCount as libc::c_int {
        *customServoMixersMutable(i_0) =
            *servoMixers[index as usize].rule.offset(i_0 as isize);
        i_0 += 1
    };
}
unsafe extern "C" fn forwardAuxChannelsToServos(mut firstServoIndex:
                                                    uint8_t) {
    // start forwarding from this channel
    let mut channelOffset: libc::c_int =
        (*servoConfig()).channelForwardingStartChannel as libc::c_int;
    let mut servoOffset: libc::c_int = 0 as libc::c_int;
    while servoOffset < 18 as libc::c_int - 4 as libc::c_int &&
              channelOffset < 18 as libc::c_int {
        let fresh3 = channelOffset;
        channelOffset = channelOffset + 1;
        pwmWriteServo((firstServoIndex as libc::c_int + servoOffset) as
                          uint8_t, rcData[fresh3 as usize] as libc::c_float);
        servoOffset += 1
    };
}
unsafe extern "C" fn updateGimbalServos(mut firstServoIndex: uint8_t) {
    pwmWriteServo((firstServoIndex as libc::c_int + 0 as libc::c_int) as
                      uint8_t,
                  servo[SERVO_GIMBAL_PITCH as libc::c_int as usize] as
                      libc::c_float);
    pwmWriteServo((firstServoIndex as libc::c_int + 1 as libc::c_int) as
                      uint8_t,
                  servo[SERVO_GIMBAL_ROLL as libc::c_int as usize] as
                      libc::c_float);
}
#[no_mangle]
pub unsafe extern "C" fn writeServos() {
    servoTable();
    filterServos();
    let mut servoIndex: uint8_t = 0 as libc::c_int as uint8_t;
    match currentMixerMode as libc::c_uint {
        1 | 25 => {
            if servosTricopterIsEnabledServoUnarmed() {
                // if unarmed flag set, we always move servo
                let fresh4 = servoIndex;
                servoIndex = servoIndex.wrapping_add(1);
                pwmWriteServo(fresh4,
                              servo[SERVO_RUDDER as libc::c_int as usize] as
                                  libc::c_float);
            } else if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                let fresh5 = servoIndex;
                servoIndex = servoIndex.wrapping_add(1);
                pwmWriteServo(fresh5,
                              servo[SERVO_RUDDER as libc::c_int as usize] as
                                  libc::c_float);
            } else {
                let fresh6 = servoIndex;
                servoIndex = servoIndex.wrapping_add(1);
                pwmWriteServo(fresh6, 0 as libc::c_int as libc::c_float);
            }
        }
        8 => {
            let fresh7 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh7,
                          servo[SERVO_FLAPPERON_1 as libc::c_int as usize] as
                              libc::c_float);
            let fresh8 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh8,
                          servo[SERVO_FLAPPERON_2 as libc::c_int as usize] as
                              libc::c_float);
        }
        24 | 14 => {
            let mut i: libc::c_int = SERVO_FLAPS as libc::c_int;
            while i <= SERVO_THROTTLE as libc::c_int {
                let fresh9 = servoIndex;
                servoIndex = servoIndex.wrapping_add(1);
                pwmWriteServo(fresh9, servo[i as usize] as libc::c_float);
                i += 1
            }
        }
        4 => {
            let fresh10 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh10,
                          servo[SERVO_BICOPTER_LEFT as libc::c_int as usize]
                              as libc::c_float);
            let fresh11 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh11,
                          servo[SERVO_BICOPTER_RIGHT as libc::c_int as usize]
                              as libc::c_float);
        }
        15 => {
            let fresh12 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh12,
                          servo[SERVO_HELI_LEFT as libc::c_int as usize] as
                              libc::c_float);
            let fresh13 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh13,
                          servo[SERVO_HELI_RIGHT as libc::c_int as usize] as
                              libc::c_float);
            let fresh14 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh14,
                          servo[SERVO_HELI_TOP as libc::c_int as usize] as
                              libc::c_float);
            let fresh15 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh15,
                          servo[SERVO_HELI_RUD as libc::c_int as usize] as
                              libc::c_float);
        }
        20 => {
            let fresh16 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh16,
                          servo[SERVO_DUALCOPTER_LEFT as libc::c_int as usize]
                              as libc::c_float);
            let fresh17 = servoIndex;
            servoIndex = servoIndex.wrapping_add(1);
            pwmWriteServo(fresh17,
                          servo[SERVO_DUALCOPTER_RIGHT as libc::c_int as
                                    usize] as libc::c_float);
        }
        21 => {
            let mut i_0: libc::c_int = SERVO_SINGLECOPTER_1 as libc::c_int;
            while i_0 <= SERVO_SINGLECOPTER_4 as libc::c_int {
                let fresh18 = servoIndex;
                servoIndex = servoIndex.wrapping_add(1);
                pwmWriteServo(fresh18, servo[i_0 as usize] as libc::c_float);
                i_0 += 1
            }
        }
        19 => {
            // otherwise, only move servo when copter is armed
            // kill servo signal completely.
            // USE_UNCOMMON_MIXERS
            let mut i_1: libc::c_int = 0 as libc::c_int;
            while i_1 < 8 as libc::c_int {
                let fresh19 = servoIndex;
                servoIndex = servoIndex.wrapping_add(1);
                pwmWriteServo(fresh19, servo[i_1 as usize] as libc::c_float);
                i_1 += 1
            }
        }
        _ => { }
    }
    // Two servos for SERVO_TILT, if enabled
    if feature(FEATURE_SERVO_TILT as libc::c_int as uint32_t) as libc::c_int
           != 0 ||
           currentMixerMode as libc::c_uint ==
               MIXER_GIMBAL as libc::c_int as libc::c_uint {
        updateGimbalServos(servoIndex);
        servoIndex = (servoIndex as libc::c_int + 2 as libc::c_int) as uint8_t
    }
    // forward AUX to remaining servo outputs (not constrained)
    if feature(FEATURE_CHANNEL_FORWARDING as libc::c_int as uint32_t) {
        forwardAuxChannelsToServos(servoIndex); // Range [-500:+500]
        servoIndex =
            (servoIndex as libc::c_int +
                 (18 as libc::c_int - 4 as libc::c_int)) as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn servoMixer() {
    let mut input: [int16_t; 14] = [0; 14];
    static mut currentOutput: [int16_t; 16] = [0; 16];
    if flightModeFlags as libc::c_int & PASSTHRU_MODE as libc::c_int != 0 {
        // Direct passthru from RX
        input[INPUT_STABILIZED_ROLL as libc::c_int as usize] =
            rcCommand[ROLL as libc::c_int as usize] as int16_t;
        input[INPUT_STABILIZED_PITCH as libc::c_int as usize] =
            rcCommand[PITCH as libc::c_int as usize] as int16_t;
        input[INPUT_STABILIZED_YAW as libc::c_int as usize] =
            rcCommand[YAW as libc::c_int as usize] as int16_t
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        input[INPUT_STABILIZED_ROLL as libc::c_int as usize] =
            (pidData[FD_ROLL as libc::c_int as usize].Sum * 0.7f32) as
                int16_t;
        input[INPUT_STABILIZED_PITCH as libc::c_int as usize] =
            (pidData[FD_PITCH as libc::c_int as usize].Sum * 0.7f32) as
                int16_t;
        input[INPUT_STABILIZED_YAW as libc::c_int as usize] =
            (pidData[FD_YAW as libc::c_int as usize].Sum * 0.7f32) as int16_t;
        // Reverse yaw servo when inverted in 3D mode
        if feature(FEATURE_3D as libc::c_int as uint32_t) as libc::c_int != 0
               &&
               (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
                   (*rxConfig()).midrc as libc::c_int {
            input[INPUT_STABILIZED_YAW as libc::c_int as usize] =
                (input[INPUT_STABILIZED_YAW as libc::c_int as usize] as
                     libc::c_int * -(1 as libc::c_int)) as int16_t
        }
    } // Since it derives from rcCommand or mincommand and must be [-500:+500]
    input[INPUT_GIMBAL_PITCH as libc::c_int as usize] =
        scaleRange(attitude.values.pitch as libc::c_int,
                   -(1800 as libc::c_int), 1800 as libc::c_int,
                   -(500 as libc::c_int), 500 as libc::c_int) as int16_t;
    input[INPUT_GIMBAL_ROLL as libc::c_int as usize] =
        scaleRange(attitude.values.roll as libc::c_int,
                   -(1800 as libc::c_int), 1800 as libc::c_int,
                   -(500 as libc::c_int), 500 as libc::c_int) as int16_t;
    input[INPUT_STABILIZED_THROTTLE as libc::c_int as usize] =
        (motor[0 as libc::c_int as usize] -
             1000 as libc::c_int as libc::c_float -
             500 as libc::c_int as libc::c_float) as int16_t;
    // center the RC input value around the RC middle value
    // by subtracting the RC middle value from the RC input value, we get:
    // data - middle = input
    // 2000 - 1500 = +500
    // 1500 - 1500 = 0
    // 1000 - 1500 = -500
    input[INPUT_RC_ROLL as libc::c_int as usize] =
        (rcData[ROLL as libc::c_int as usize] as libc::c_int -
             (*rxConfig()).midrc as libc::c_int) as int16_t;
    input[INPUT_RC_PITCH as libc::c_int as usize] =
        (rcData[PITCH as libc::c_int as usize] as libc::c_int -
             (*rxConfig()).midrc as libc::c_int) as int16_t;
    input[INPUT_RC_YAW as libc::c_int as usize] =
        (rcData[YAW as libc::c_int as usize] as libc::c_int -
             (*rxConfig()).midrc as libc::c_int) as int16_t;
    input[INPUT_RC_THROTTLE as libc::c_int as usize] =
        (rcData[THROTTLE as libc::c_int as usize] as libc::c_int -
             (*rxConfig()).midrc as libc::c_int) as int16_t;
    input[INPUT_RC_AUX1 as libc::c_int as usize] =
        (rcData[AUX1 as libc::c_int as usize] as libc::c_int -
             (*rxConfig()).midrc as libc::c_int) as int16_t;
    input[INPUT_RC_AUX2 as libc::c_int as usize] =
        (rcData[AUX2 as libc::c_int as usize] as libc::c_int -
             (*rxConfig()).midrc as libc::c_int) as int16_t;
    input[INPUT_RC_AUX3 as libc::c_int as usize] =
        (rcData[AUX3 as libc::c_int as usize] as libc::c_int -
             (*rxConfig()).midrc as libc::c_int) as int16_t;
    input[INPUT_RC_AUX4 as libc::c_int as usize] =
        (rcData[AUX4 as libc::c_int as usize] as libc::c_int -
             (*rxConfig()).midrc as libc::c_int) as int16_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        servo[i as usize] = 0 as libc::c_int as int16_t;
        i += 1
    }
    // mix servos according to rules
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < servoRuleCount as libc::c_int {
        // consider rule if no box assigned or box is active
        if currentServoMixer[i_0 as usize].box_0 as libc::c_int ==
               0 as libc::c_int ||
               IS_RC_MODE_ACTIVE((BOXSERVO1 as libc::c_int +
                                      currentServoMixer[i_0 as usize].box_0 as
                                          libc::c_int - 1 as libc::c_int) as
                                     boxId_e) as libc::c_int != 0 {
            let mut target: uint8_t =
                currentServoMixer[i_0 as usize].targetChannel;
            let mut from: uint8_t =
                currentServoMixer[i_0 as usize].inputSource;
            let mut servo_width: uint16_t =
                ((*servoParams(target as libc::c_int)).max as libc::c_int -
                     (*servoParams(target as libc::c_int)).min as libc::c_int)
                    as uint16_t;
            let mut min: int16_t =
                (currentServoMixer[i_0 as usize].min as libc::c_int *
                     servo_width as libc::c_int / 100 as libc::c_int -
                     servo_width as libc::c_int / 2 as libc::c_int) as
                    int16_t;
            let mut max: int16_t =
                (currentServoMixer[i_0 as usize].max as libc::c_int *
                     servo_width as libc::c_int / 100 as libc::c_int -
                     servo_width as libc::c_int / 2 as libc::c_int) as
                    int16_t;
            if currentServoMixer[i_0 as usize].speed as libc::c_int ==
                   0 as libc::c_int {
                currentOutput[i_0 as usize] = input[from as usize]
            } else if (currentOutput[i_0 as usize] as libc::c_int) <
                          input[from as usize] as libc::c_int {
                currentOutput[i_0 as usize] =
                    constrain(currentOutput[i_0 as usize] as libc::c_int +
                                  currentServoMixer[i_0 as usize].speed as
                                      libc::c_int,
                              currentOutput[i_0 as usize] as libc::c_int,
                              input[from as usize] as libc::c_int) as int16_t
            } else if currentOutput[i_0 as usize] as libc::c_int >
                          input[from as usize] as libc::c_int {
                currentOutput[i_0 as usize] =
                    constrain(currentOutput[i_0 as usize] as libc::c_int -
                                  currentServoMixer[i_0 as usize].speed as
                                      libc::c_int,
                              input[from as usize] as libc::c_int,
                              currentOutput[i_0 as usize] as libc::c_int) as
                        int16_t
            }
            servo[target as usize] =
                (servo[target as usize] as libc::c_int +
                     servoDirection(target as libc::c_int,
                                    from as libc::c_int) *
                         constrain(currentOutput[i_0 as usize] as int32_t *
                                       currentServoMixer[i_0 as usize].rate as
                                           libc::c_int / 100 as libc::c_int,
                                   min as libc::c_int, max as libc::c_int)) as
                    int16_t
        } else { currentOutput[i_0 as usize] = 0 as libc::c_int as int16_t }
        i_0 += 1
    }
    let mut i_1: libc::c_int = 0 as libc::c_int;
    while i_1 < 8 as libc::c_int {
        servo[i_1 as usize] =
            (((*servoParams(i_1)).rate as int32_t *
                  servo[i_1 as usize] as libc::c_int) as libc::c_long /
                 100 as libc::c_long) as int16_t;
        servo[i_1 as usize] =
            (servo[i_1 as usize] as libc::c_int +
                 determineServoMiddleOrForwardFromChannel(i_1 as servoIndex_e)
                     as libc::c_int) as int16_t;
        i_1 += 1
    };
}
unsafe extern "C" fn servoTable() {
    // airplane / servo mixes
    match currentMixerMode as libc::c_uint {
        25 | 1 => { servosTricopterMixer(); }
        24 | 8 | 14 | 4 | 20 | 21 | 15 | 5 => { servoMixer(); }
        19 => {
            let mut i: libc::c_int = 0 as libc::c_int;
            while i <
                      ({
                           let mut _a: libc::c_int = 8 as libc::c_int;
                           let mut _b: libc::c_int = 18 as libc::c_int;
                           (if _a < _b { _a } else { _b })
                       }) {
                servo[i as usize] = rcData[i as usize];
                i += 1
            }
        }
        _ => { }
    }
    // camera stabilization
    if feature(FEATURE_SERVO_TILT as libc::c_int as uint32_t) {
        // center at fixed position, or vary either pitch or roll by RC channel
        servo[SERVO_GIMBAL_PITCH as libc::c_int as usize] =
            determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
        servo[SERVO_GIMBAL_ROLL as libc::c_int as usize] =
            determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);
        if IS_RC_MODE_ACTIVE(BOXCAMSTAB) {
            if (*gimbalConfig()).mode as libc::c_int ==
                   GIMBAL_MODE_MIXTILT as libc::c_int {
                servo[SERVO_GIMBAL_PITCH as libc::c_int as usize] =
                    (servo[SERVO_GIMBAL_PITCH as libc::c_int as usize] as
                         libc::c_int -
                         (-((*servoParams(SERVO_GIMBAL_PITCH as
                                              libc::c_int)).rate as int32_t) *
                              attitude.values.pitch as libc::c_int /
                              50 as libc::c_int -
                              (*servoParams(SERVO_GIMBAL_ROLL as
                                                libc::c_int)).rate as int32_t
                                  * attitude.values.roll as libc::c_int /
                                  50 as libc::c_int)) as int16_t;
                servo[SERVO_GIMBAL_ROLL as libc::c_int as usize] =
                    (servo[SERVO_GIMBAL_ROLL as libc::c_int as usize] as
                         libc::c_int +
                         (-((*servoParams(SERVO_GIMBAL_PITCH as
                                              libc::c_int)).rate as int32_t) *
                              attitude.values.pitch as libc::c_int /
                              50 as libc::c_int +
                              (*servoParams(SERVO_GIMBAL_ROLL as
                                                libc::c_int)).rate as int32_t
                                  * attitude.values.roll as libc::c_int /
                                  50 as libc::c_int)) as int16_t
            } else {
                servo[SERVO_GIMBAL_PITCH as libc::c_int as usize] =
                    (servo[SERVO_GIMBAL_PITCH as libc::c_int as usize] as
                         libc::c_int +
                         (*servoParams(SERVO_GIMBAL_PITCH as
                                           libc::c_int)).rate as int32_t *
                             attitude.values.pitch as libc::c_int /
                             50 as libc::c_int) as int16_t;
                servo[SERVO_GIMBAL_ROLL as libc::c_int as usize] =
                    (servo[SERVO_GIMBAL_ROLL as libc::c_int as usize] as
                         libc::c_int +
                         (*servoParams(SERVO_GIMBAL_ROLL as libc::c_int)).rate
                             as int32_t * attitude.values.roll as libc::c_int
                             / 50 as libc::c_int) as int16_t
            }
        }
    }
    // constrain servos
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < 8 as libc::c_int {
        servo[i_0 as usize] =
            constrain(servo[i_0 as usize] as libc::c_int,
                      (*servoParams(i_0)).min as libc::c_int,
                      (*servoParams(i_0)).max as libc::c_int) as int16_t;
        i_0 += 1
        // limit the values
    };
}
#[no_mangle]
pub unsafe extern "C" fn isMixerUsingServos() -> bool {
    return useServo != 0;
}
static mut servoFilter: [biquadFilter_t; 8] =
    [biquadFilter_t{b0: 0.,
                    b1: 0.,
                    b2: 0.,
                    a1: 0.,
                    a2: 0.,
                    x1: 0.,
                    x2: 0.,
                    y1: 0.,
                    y2: 0.,}; 8];
#[no_mangle]
pub unsafe extern "C" fn servosFilterInit() {
    if (*servoConfig()).servo_lowpass_freq != 0 {
        let mut servoIdx: libc::c_int = 0 as libc::c_int;
        while servoIdx < 8 as libc::c_int {
            biquadFilterInitLPF(&mut *servoFilter.as_mut_ptr().offset(servoIdx
                                                                          as
                                                                          isize),
                                (*servoConfig()).servo_lowpass_freq as
                                    libc::c_float, targetPidLooptime);
            servoIdx += 1
        }
    };
}
unsafe extern "C" fn filterServos() {
    if (*servoConfig()).servo_lowpass_freq != 0 {
        let mut servoIdx: libc::c_int = 0 as libc::c_int;
        while servoIdx < 8 as libc::c_int {
            servo[servoIdx as usize] =
                lrintf(biquadFilterApply(&mut *servoFilter.as_mut_ptr().offset(servoIdx
                                                                                   as
                                                                                   isize),
                                         servo[servoIdx as usize] as
                                             libc::c_float)) as int16_t;
            // Sanity check
            servo[servoIdx as usize] =
                constrain(servo[servoIdx as usize] as libc::c_int,
                          (*servoParams(servoIdx)).min as libc::c_int,
                          (*servoParams(servoIdx)).max as libc::c_int) as
                    int16_t;
            servoIdx += 1
        }
    };
}
unsafe extern "C" fn run_static_initializers() {
    customServoMixers_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (21 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<servoMixer_t>() as
                                       libc::c_ulong).wrapping_mul((2 as
                                                                        libc::c_int
                                                                        *
                                                                        8 as
                                                                            libc::c_int)
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut customServoMixers_SystemArray as
                                     *mut [servoMixer_t; 16] as *mut uint8_t,
                             copy:
                                 &mut customServoMixers_CopyArray as
                                     *mut [servoMixer_t; 16] as *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     0 as
                                                         *mut libc::c_void,},};
            init
        };
    servoParams_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (42 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<servoParam_t>() as
                                       libc::c_ulong).wrapping_mul(8 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut servoParams_SystemArray as
                                     *mut [servoParam_t; 8] as *mut uint8_t,
                             copy:
                                 &mut servoParams_CopyArray as
                                     *mut [servoParam_t; 8] as *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut servoParam_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_servoParams
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut servoParam_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        };
    servoMixers =
        [{
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount:
                                  (::core::mem::size_of::<[servoMixer_t; 1]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<servoMixer_t>()
                                                                       as
                                                                       libc::c_ulong)
                                      as uint8_t,
                              rule: servoMixerTri.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount:
                                  (::core::mem::size_of::<[servoMixer_t; 4]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<servoMixer_t>()
                                                                       as
                                                                       libc::c_ulong)
                                      as uint8_t,
                              rule: servoMixerBI.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount:
                                  (::core::mem::size_of::<[servoMixer_t; 2]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<servoMixer_t>()
                                                                       as
                                                                       libc::c_ulong)
                                      as uint8_t,
                              rule: servoMixerGimbal.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount:
                                  (::core::mem::size_of::<[servoMixer_t; 5]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<servoMixer_t>()
                                                                       as
                                                                       libc::c_ulong)
                                      as uint8_t,
                              rule: servoMixerFlyingWing.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount:
                                  (::core::mem::size_of::<[servoMixer_t; 5]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<servoMixer_t>()
                                                                       as
                                                                       libc::c_ulong)
                                      as uint8_t,
                              rule: servoMixerAirplane.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount:
                                  (::core::mem::size_of::<[servoMixer_t; 9]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<servoMixer_t>()
                                                                       as
                                                                       libc::c_ulong)
                                      as uint8_t,
                              rule: servoMixerHeli.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount:
                                  (::core::mem::size_of::<[servoMixer_t; 2]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<servoMixer_t>()
                                                                       as
                                                                       libc::c_ulong)
                                      as uint8_t,
                              rule: servoMixerDual.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount:
                                  (::core::mem::size_of::<[servoMixer_t; 8]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<servoMixer_t>()
                                                                       as
                                                                       libc::c_ulong)
                                      as uint8_t,
                              rule: servoMixerSingle.as_ptr(),};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         },
         {
             let mut init =
                 mixerRules_s{servoRuleCount: 0 as libc::c_int as uint8_t,
                              rule: 0 as *const servoMixer_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
// USE_SERVOS
