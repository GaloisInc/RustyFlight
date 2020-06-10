use ::libc;
extern "C" {
    #[no_mangle]
    fn strcpy(_: *mut libc::c_char, _: *const libc::c_char)
     -> *mut libc::c_char;
    #[no_mangle]
    static mut hardwareMotorType: uint8_t;
    #[no_mangle]
    static mut pilotConfig_System: pilotConfig_t;
    #[no_mangle]
    static mut controlRateProfiles_SystemArray: [controlRateConfig_t; 6];
    #[no_mangle]
    static mut modeActivationConditions_SystemArray:
           [modeActivationCondition_t; 20];
    #[no_mangle]
    static mut customMotorMixer_SystemArray: [motorMixer_t; 8];
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut pidProfiles_SystemArray: [pidProfile_t; 3];
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
    #[no_mangle]
    static mut vcdProfile_System: vcdProfile_t;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static mut rxFailsafeChannelConfigs_SystemArray:
           [rxFailsafeChannelConfig_t; 18];
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn findSerialPortIndexByIdentifier(identifier: serialPortIdentifier_e)
     -> libc::c_int;
    #[no_mangle]
    static mut osdConfig_System: osdConfig_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const MOTOR_BRUSHLESS: C2RustUnnamed_0 = 2;
pub const MOTOR_BRUSHED: C2RustUnnamed_0 = 1;
pub const MOTOR_UNKNOWN: C2RustUnnamed_0 = 0;
#[derive(Copy, Clone)]
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
pub const OSD_RSSI_VALUE: C2RustUnnamed_4 = 0;
pub type osdConfig_t = osdConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct osdConfig_s {
    pub item_pos: [uint16_t; 43],
    pub cap_alarm: uint16_t,
    pub alt_alarm: uint16_t,
    pub rssi_alarm: uint8_t,
    pub units: osd_unit_e,
    pub timers: [uint16_t; 2],
    pub enabledWarnings: uint16_t,
    pub ahMaxPitch: uint8_t,
    pub ahMaxRoll: uint8_t,
    pub enabled_stats: uint32_t,
    pub esc_temp_alarm: int8_t,
    pub esc_rpm_alarm: int16_t,
    pub esc_current_alarm: int16_t,
    pub core_temp_alarm: uint8_t,
}
pub type osd_unit_e = libc::c_uint;
pub const OSD_UNIT_METRIC: osd_unit_e = 1;
pub const OSD_UNIT_IMPERIAL: osd_unit_e = 0;
pub type rxFailsafeChannelConfig_t = rxFailsafeChannelConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxFailsafeChannelConfig_s {
    pub mode: uint8_t,
    pub step: uint8_t,
}
pub const RX_FAILSAFE_MODE_SET: C2RustUnnamed_2 = 2;
// Alarms
// See rxFailsafeChannelMode_e
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
pub const FUNCTION_RX_SERIAL: C2RustUnnamed_3 = 64;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: C2RustUnnamed_3 = 4;
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
    pub portConfigs: [serialPortConfig_t; 4],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
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
// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub type channelRange_t = channelRange_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
pub type modeActivationCondition_t = modeActivationCondition_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeActivationCondition_s {
    pub modeId: boxId_e,
    pub auxChannelIndex: uint8_t,
    pub range: channelRange_t,
    pub modeLogic: modeLogic_e,
    pub linkedTo: boxId_e,
}
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
pub type modeLogic_e = libc::c_uint;
pub const MODELOGIC_AND: modeLogic_e = 1;
pub const MODELOGIC_OR: modeLogic_e = 0;
pub const AUX2: rc_alias = 5;
pub const OSD_G_FORCE: C2RustUnnamed_4 = 42;
pub const OSD_ESC_RPM: C2RustUnnamed_4 = 36;
pub const OSD_ESC_TMP: C2RustUnnamed_4 = 35;
pub const OSD_NUMERICAL_VARIO: C2RustUnnamed_4 = 33;
pub const OSD_NUMERICAL_HEADING: C2RustUnnamed_4 = 32;
pub const OSD_DISARMED: C2RustUnnamed_4 = 29;
pub const OSD_MAIN_BATT_USAGE: C2RustUnnamed_4 = 28;
pub const OSD_ROLL_ANGLE: C2RustUnnamed_4 = 27;
pub const OSD_PITCH_ANGLE: C2RustUnnamed_4 = 26;
pub const OSD_AVG_CELL_VOLTAGE: C2RustUnnamed_4 = 22;
pub const OSD_WARNINGS: C2RustUnnamed_4 = 21;
pub const OSD_PIDRATE_PROFILE: C2RustUnnamed_4 = 20;
pub const OSD_POWER: C2RustUnnamed_4 = 19;
pub const OSD_DEBUG: C2RustUnnamed_4 = 25;
pub const OSD_YAW_PIDS: C2RustUnnamed_4 = 18;
pub const OSD_PITCH_PIDS: C2RustUnnamed_4 = 17;
pub const OSD_ROLL_PIDS: C2RustUnnamed_4 = 16;
pub const OSD_ALTITUDE: C2RustUnnamed_4 = 15;
pub const OSD_COMPASS_BAR: C2RustUnnamed_4 = 34;
pub const OSD_HOME_DIST: C2RustUnnamed_4 = 31;
pub const OSD_HOME_DIR: C2RustUnnamed_4 = 30;
pub const OSD_GPS_SATS: C2RustUnnamed_4 = 14;
pub const OSD_GPS_LAT: C2RustUnnamed_4 = 24;
pub const OSD_GPS_LON: C2RustUnnamed_4 = 23;
pub const OSD_GPS_SPEED: C2RustUnnamed_4 = 13;
pub const OSD_MAH_DRAWN: C2RustUnnamed_4 = 12;
pub const OSD_CURRENT_DRAW: C2RustUnnamed_4 = 11;
pub const OSD_ARTIFICIAL_HORIZON: C2RustUnnamed_4 = 3;
pub const OSD_HORIZON_SIDEBARS: C2RustUnnamed_4 = 4;
pub const OSD_CROSSHAIRS: C2RustUnnamed_4 = 2;
pub const OSD_THROTTLE_POS: C2RustUnnamed_4 = 9;
pub const OSD_ITEM_TIMER_1: C2RustUnnamed_4 = 5;
pub const OSD_VTX_CHANNEL: C2RustUnnamed_4 = 10;
pub const OSD_FLYMODE: C2RustUnnamed_4 = 7;
pub const OSD_ITEM_TIMER_2: C2RustUnnamed_4 = 6;
pub const OSD_MAIN_BATT_VOLTAGE: C2RustUnnamed_4 = 1;
pub const OSD_CRAFT_NAME: C2RustUnnamed_4 = 8;
pub const VIDEO_SYSTEM_NTSC: VIDEO_SYSTEMS = 2;
pub type vcdProfile_t = vcdProfile_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vcdProfile_s {
    pub video_system: uint8_t,
    pub h_offset: int8_t,
    pub v_offset: int8_t,
}
// Custom mixer data per motor
pub type motorMixer_t = motorMixer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorMixer_s {
    pub throttle: libc::c_float,
    pub roll: libc::c_float,
    pub pitch: libc::c_float,
    pub yaw: libc::c_float,
}
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
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
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
//
// meters
//
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
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
pub const PID_YAW: C2RustUnnamed_1 = 2;
pub type pidProfile_t = pidProfile_s;
pub const PID_PITCH: C2RustUnnamed_1 = 1;
pub const PID_ROLL: C2RustUnnamed_1 = 0;
pub type pidConfig_t = pidConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
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
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX1: rc_alias = 4;
pub const THROTTLE: rc_alias = 3;
pub const YAW: rc_alias = 2;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_1 = 5;
pub const PID_MAG: C2RustUnnamed_1 = 4;
pub const PID_LEVEL: C2RustUnnamed_1 = 3;
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
// Video Character Display parameters
pub type VIDEO_SYSTEMS = libc::c_uint;
pub const VIDEO_SYSTEM_PAL: VIDEO_SYSTEMS = 1;
pub const VIDEO_SYSTEM_AUTO: VIDEO_SYSTEMS = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const RX_FAILSAFE_MODE_INVALID: C2RustUnnamed_2 = 3;
pub const RX_FAILSAFE_MODE_HOLD: C2RustUnnamed_2 = 1;
pub const RX_FAILSAFE_MODE_AUTO: C2RustUnnamed_2 = 0;
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
pub type C2RustUnnamed_4 = libc::c_uint;
pub const OSD_ITEM_COUNT: C2RustUnnamed_4 = 43;
pub const OSD_ANTI_GRAVITY: C2RustUnnamed_4 = 41;
pub const OSD_CORE_TEMPERATURE: C2RustUnnamed_4 = 40;
pub const OSD_ADJUSTMENT_RANGE: C2RustUnnamed_4 = 39;
pub const OSD_RTC_DATETIME: C2RustUnnamed_4 = 38;
pub const OSD_REMAINING_TIME_ESTIMATE: C2RustUnnamed_4 = 37;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn pilotConfigMutable() -> *mut pilotConfig_t {
    return &mut pilotConfig_System;
}
#[inline]
unsafe extern "C" fn controlRateProfilesMutable(mut _index: libc::c_int)
 -> *mut controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
}
#[inline]
unsafe extern "C" fn modeActivationConditionsMutable(mut _index: libc::c_int)
 -> *mut modeActivationCondition_t {
    return &mut *modeActivationConditions_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut modeActivationCondition_t;
}
#[inline]
unsafe extern "C" fn customMotorMixerMutable(mut _index: libc::c_int)
 -> *mut motorMixer_t {
    return &mut *customMotorMixer_SystemArray.as_mut_ptr().offset(_index as
                                                                      isize)
               as *mut motorMixer_t;
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
unsafe extern "C" fn vcdProfileMutable() -> *mut vcdProfile_t {
    return &mut vcdProfile_System;
}
#[inline]
unsafe extern "C" fn rxConfigMutable() -> *mut rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn rxFailsafeChannelConfigsMutable(mut _index: libc::c_int)
 -> *mut rxFailsafeChannelConfig_t {
    return &mut *rxFailsafeChannelConfigs_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut rxFailsafeChannelConfig_t;
}
#[inline]
unsafe extern "C" fn serialConfigMutable() -> *mut serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn osdConfigMutable() -> *mut osdConfig_t {
    return &mut osdConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfigMutable() -> *mut batteryConfig_t {
    return &mut batteryConfig_System;
}
// 32kHz
#[no_mangle]
pub unsafe extern "C" fn targetConfiguration() {
    if hardwareMotorType as libc::c_int == MOTOR_BRUSHED as libc::c_int {
        (*motorConfigMutable()).dev.motorPwmRate =
            32000 as libc::c_int as uint16_t; // REAR_R
        (*motorConfigMutable()).minthrottle =
            1030 as libc::c_int as uint16_t; // FRONT_R
        (*pidConfigMutable()).pid_process_denom = 1 as libc::c_int as uint8_t
    } // REAR_L
    let mut pidProfileIndex: uint8_t = 0 as libc::c_int as uint8_t; // FRONT_L
    while (pidProfileIndex as libc::c_int) < 3 as libc::c_int {
        let mut pidProfile: *mut pidProfile_t =
            pidProfilesMutable(pidProfileIndex as libc::c_int);
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].P =
            86 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].I =
            50 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_ROLL as libc::c_int as usize].D =
            60 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].P =
            90 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].I =
            55 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_PITCH as libc::c_int as usize].D =
            60 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_YAW as libc::c_int as usize].P =
            123 as libc::c_int as uint8_t;
        (*pidProfile).pid[PID_YAW as libc::c_int as usize].I =
            75 as libc::c_int as uint8_t;
        pidProfileIndex = pidProfileIndex.wrapping_add(1)
    }
    let mut rateProfileIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (rateProfileIndex as libc::c_int) < 6 as libc::c_int {
        let mut controlRateConfig: *mut controlRateConfig_t =
            controlRateProfilesMutable(rateProfileIndex as libc::c_int);
        (*controlRateConfig).rcRates[FD_YAW as libc::c_int as usize] =
            120 as libc::c_int as uint8_t;
        (*controlRateConfig).rcExpo[FD_ROLL as libc::c_int as usize] =
            15 as libc::c_int as uint8_t;
        (*controlRateConfig).rcExpo[FD_PITCH as libc::c_int as usize] =
            15 as libc::c_int as uint8_t;
        (*controlRateConfig).rcExpo[FD_YAW as libc::c_int as usize] =
            15 as libc::c_int as uint8_t;
        (*controlRateConfig).rates[FD_ROLL as libc::c_int as usize] =
            85 as libc::c_int as uint8_t;
        (*controlRateConfig).rates[FD_PITCH as libc::c_int as usize] =
            85 as libc::c_int as uint8_t;
        rateProfileIndex = rateProfileIndex.wrapping_add(1)
    }
    (*batteryConfigMutable()).batteryCapacity =
        250 as libc::c_int as uint16_t;
    (*batteryConfigMutable()).vbatmincellvoltage =
        28 as libc::c_int as uint8_t;
    (*batteryConfigMutable()).vbatwarningcellvoltage =
        33 as libc::c_int as uint8_t;
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
    (*vcdProfileMutable()).video_system =
        VIDEO_SYSTEM_NTSC as libc::c_int as uint8_t;
    strcpy((*pilotConfigMutable()).name.as_mut_ptr(),
           b"BeeStorm\x00" as *const u8 as *const libc::c_char);
    (*osdConfigMutable()).cap_alarm = 250 as libc::c_int as uint16_t;
    (*osdConfigMutable()).item_pos[OSD_CRAFT_NAME as libc::c_int as usize] =
        (9 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (11 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfigMutable()).item_pos[OSD_MAIN_BATT_VOLTAGE as libc::c_int as
                                       usize] =
        (23 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (10 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfigMutable()).item_pos[OSD_ITEM_TIMER_2 as libc::c_int as usize] =
        (2 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (10 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfigMutable()).item_pos[OSD_FLYMODE as libc::c_int as usize] =
        (17 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (10 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    (*osdConfigMutable()).item_pos[OSD_VTX_CHANNEL as libc::c_int as usize] =
        (10 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (10 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
    let ref mut fresh0 =
        (*osdConfigMutable()).item_pos[OSD_RSSI_VALUE as libc::c_int as
                                           usize];
    *fresh0 = (*fresh0 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh1 =
        (*osdConfigMutable()).item_pos[OSD_ITEM_TIMER_1 as libc::c_int as
                                           usize];
    *fresh1 = (*fresh1 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh2 =
        (*osdConfigMutable()).item_pos[OSD_THROTTLE_POS as libc::c_int as
                                           usize];
    *fresh2 = (*fresh2 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh3 =
        (*osdConfigMutable()).item_pos[OSD_CROSSHAIRS as libc::c_int as
                                           usize];
    *fresh3 = (*fresh3 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh4 =
        (*osdConfigMutable()).item_pos[OSD_HORIZON_SIDEBARS as libc::c_int as
                                           usize];
    *fresh4 = (*fresh4 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh5 =
        (*osdConfigMutable()).item_pos[OSD_ARTIFICIAL_HORIZON as libc::c_int
                                           as usize];
    *fresh5 = (*fresh5 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh6 =
        (*osdConfigMutable()).item_pos[OSD_CURRENT_DRAW as libc::c_int as
                                           usize];
    *fresh6 = (*fresh6 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh7 =
        (*osdConfigMutable()).item_pos[OSD_MAH_DRAWN as libc::c_int as usize];
    *fresh7 = (*fresh7 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh8 =
        (*osdConfigMutable()).item_pos[OSD_GPS_SPEED as libc::c_int as usize];
    *fresh8 = (*fresh8 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh9 =
        (*osdConfigMutable()).item_pos[OSD_GPS_LON as libc::c_int as usize];
    *fresh9 = (*fresh9 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh10 =
        (*osdConfigMutable()).item_pos[OSD_GPS_LAT as libc::c_int as usize];
    *fresh10 =
        (*fresh10 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh11 =
        (*osdConfigMutable()).item_pos[OSD_GPS_SATS as libc::c_int as usize];
    *fresh11 =
        (*fresh11 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh12 =
        (*osdConfigMutable()).item_pos[OSD_HOME_DIR as libc::c_int as usize];
    *fresh12 =
        (*fresh12 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh13 =
        (*osdConfigMutable()).item_pos[OSD_HOME_DIST as libc::c_int as usize];
    *fresh13 =
        (*fresh13 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh14 =
        (*osdConfigMutable()).item_pos[OSD_COMPASS_BAR as libc::c_int as
                                           usize];
    *fresh14 =
        (*fresh14 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh15 =
        (*osdConfigMutable()).item_pos[OSD_ALTITUDE as libc::c_int as usize];
    *fresh15 =
        (*fresh15 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh16 =
        (*osdConfigMutable()).item_pos[OSD_ROLL_PIDS as libc::c_int as usize];
    *fresh16 =
        (*fresh16 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh17 =
        (*osdConfigMutable()).item_pos[OSD_PITCH_PIDS as libc::c_int as
                                           usize];
    *fresh17 =
        (*fresh17 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh18 =
        (*osdConfigMutable()).item_pos[OSD_YAW_PIDS as libc::c_int as usize];
    *fresh18 =
        (*fresh18 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh19 =
        (*osdConfigMutable()).item_pos[OSD_DEBUG as libc::c_int as usize];
    *fresh19 =
        (*fresh19 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh20 =
        (*osdConfigMutable()).item_pos[OSD_POWER as libc::c_int as usize];
    *fresh20 =
        (*fresh20 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh21 =
        (*osdConfigMutable()).item_pos[OSD_PIDRATE_PROFILE as libc::c_int as
                                           usize];
    *fresh21 =
        (*fresh21 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh22 =
        (*osdConfigMutable()).item_pos[OSD_WARNINGS as libc::c_int as usize];
    *fresh22 =
        (*fresh22 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh23 =
        (*osdConfigMutable()).item_pos[OSD_AVG_CELL_VOLTAGE as libc::c_int as
                                           usize];
    *fresh23 =
        (*fresh23 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh24 =
        (*osdConfigMutable()).item_pos[OSD_PITCH_ANGLE as libc::c_int as
                                           usize];
    *fresh24 =
        (*fresh24 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh25 =
        (*osdConfigMutable()).item_pos[OSD_ROLL_ANGLE as libc::c_int as
                                           usize];
    *fresh25 =
        (*fresh25 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh26 =
        (*osdConfigMutable()).item_pos[OSD_MAIN_BATT_USAGE as libc::c_int as
                                           usize];
    *fresh26 =
        (*fresh26 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh27 =
        (*osdConfigMutable()).item_pos[OSD_DISARMED as libc::c_int as usize];
    *fresh27 =
        (*fresh27 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh28 =
        (*osdConfigMutable()).item_pos[OSD_NUMERICAL_HEADING as libc::c_int as
                                           usize];
    *fresh28 =
        (*fresh28 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh29 =
        (*osdConfigMutable()).item_pos[OSD_NUMERICAL_VARIO as libc::c_int as
                                           usize];
    *fresh29 =
        (*fresh29 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh30 =
        (*osdConfigMutable()).item_pos[OSD_ESC_TMP as libc::c_int as usize];
    *fresh30 =
        (*fresh30 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh31 =
        (*osdConfigMutable()).item_pos[OSD_ESC_RPM as libc::c_int as usize];
    *fresh31 =
        (*fresh31 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    let ref mut fresh32 =
        (*osdConfigMutable()).item_pos[OSD_G_FORCE as libc::c_int as usize];
    *fresh32 =
        (*fresh32 as libc::c_int & !(0x800 as libc::c_int)) as uint16_t;
    (*modeActivationConditionsMutable(0 as libc::c_int)).modeId = BOXANGLE;
    (*modeActivationConditionsMutable(0 as libc::c_int)).auxChannelIndex =
        (AUX2 as libc::c_int - 4 as libc::c_int) as uint8_t;
    (*modeActivationConditionsMutable(0 as libc::c_int)).range.startStep =
        ((constrain(900 as libc::c_int, 900 as libc::c_int,
                    2100 as libc::c_int) - 900 as libc::c_int) /
             25 as libc::c_int) as uint8_t;
    (*modeActivationConditionsMutable(0 as libc::c_int)).range.endStep =
        ((constrain(2100 as libc::c_int, 900 as libc::c_int,
                    2100 as libc::c_int) - 900 as libc::c_int) /
             25 as libc::c_int) as uint8_t;
    // Frsky version
    (*serialConfigMutable()).portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART2)
                                             as usize].functionMask =
        (FUNCTION_TELEMETRY_FRSKY_HUB as libc::c_int |
             FUNCTION_RX_SERIAL as libc::c_int) as uint16_t;
    (*rxConfigMutable()).rssi_channel = 9 as libc::c_int as uint8_t;
    let mut channelFailsafeConfig: *mut rxFailsafeChannelConfig_t =
        rxFailsafeChannelConfigsMutable(9 as libc::c_int - 1 as libc::c_int);
    (*channelFailsafeConfig).mode =
        RX_FAILSAFE_MODE_SET as libc::c_int as uint8_t;
    (*channelFailsafeConfig).step =
        ((constrain(1000 as libc::c_int, 750 as libc::c_int,
                    2250 as libc::c_int) - 750 as libc::c_int) /
             25 as libc::c_int) as uint8_t;
    (*osdConfigMutable()).item_pos[OSD_RSSI_VALUE as libc::c_int as usize] =
        (2 as libc::c_int &
             ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int |
             (11 as libc::c_int &
                  ((1 as libc::c_int) << 5 as libc::c_int) - 1 as libc::c_int)
                 << 5 as libc::c_int | 0x800 as libc::c_int) as uint16_t;
}
