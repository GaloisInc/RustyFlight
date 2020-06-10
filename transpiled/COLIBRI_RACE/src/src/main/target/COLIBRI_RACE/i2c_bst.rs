use ::libc;
extern "C" {
    // lower case hexadecimal digits.
    #[no_mangle]
    static shortGitRevision: *const libc::c_char;
    #[no_mangle]
    static buildDate: *const libc::c_char;
    #[no_mangle]
    static buildTime: *const libc::c_char;
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
    #[no_mangle]
    fn i2cGetErrorCounter() -> uint16_t;
    // bootloader/IAP
    #[no_mangle]
    fn systemReset();
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    fn readEEPROM() -> bool;
    #[no_mangle]
    fn writeEEPROM();
    #[no_mangle]
    fn getCurrentPidProfileIndex() -> uint8_t;
    #[no_mangle]
    fn changePidProfile(pidProfileIndex: uint8_t);
    #[no_mangle]
    static mut currentControlRateProfile: *mut controlRateConfig_t;
    #[no_mangle]
    fn disarm();
    #[no_mangle]
    static mut modeActivationConditions_SystemArray:
           [modeActivationCondition_t; 20];
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut rcControlsConfig_System: rcControlsConfig_t;
    #[no_mangle]
    fn useRcControlsConfig(pidProfileToUse: *mut pidProfile_s);
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn unsetArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    static mut ledStripConfig_System: ledStripConfig_t;
    #[no_mangle]
    fn reevaluateLedConfig();
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn getTaskDeltaTime(taskId: cfTaskId_e) -> timeDelta_t;
    #[no_mangle]
    static mut voltageSensorADCConfig_SystemArray:
           [voltageSensorADCConfig_t; 1];
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn accSetCalibrationCycles(calibrationCyclesRequired: uint16_t);
    #[no_mangle]
    static mut compassConfig_System: compassConfig_t;
    #[no_mangle]
    static mut failsafeConfig_System: failsafeConfig_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    fn stopMotors();
    #[no_mangle]
    fn pidInitConfig(pidProfile: *const pidProfile_t);
    #[no_mangle]
    fn featureConfigured(mask: uint32_t) -> bool;
    #[no_mangle]
    fn featureSet(mask: uint32_t);
    #[no_mangle]
    fn featureClearAll();
    #[no_mangle]
    fn featureMask() -> uint32_t;
    #[no_mangle]
    fn bstTimeoutUserCallback() -> uint32_t;
    #[no_mangle]
    fn bstWriteBusy() -> bool;
    #[no_mangle]
    fn bstMasterWrite(data: *mut uint8_t) -> bool;
    #[no_mangle]
    fn bstMasterWriteLoop();
    //in message          no param
    #[no_mangle]
    static mut CRC8: uint8_t;
    #[no_mangle]
    static mut coreProReady: bool;
    #[no_mangle]
    static mut readData: [uint8_t; 128];
    #[no_mangle]
    static mut writeData: [uint8_t; 128];
    #[no_mangle]
    static mut cleanflight_data_ready: bool;
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
pub const FD_YAW: C2RustUnnamed = 2;
pub const FD_PITCH: C2RustUnnamed = 1;
pub const FD_ROLL: C2RustUnnamed = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
// 0 - 359
// 0 - 255
// 0 - 255
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
pub type gyroConfig_t = gyroConfig_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
pub type channelRange_t = channelRange_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeActivationCondition_s {
    pub modeId: boxId_e,
    pub auxChannelIndex: uint8_t,
    pub range: channelRange_t,
    pub modeLogic: modeLogic_e,
    pub linkedTo: boxId_e,
}
pub type modeActivationCondition_t = modeActivationCondition_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcControlsConfig_s {
    pub deadband: uint8_t,
    pub yaw_deadband: uint8_t,
    pub alt_hold_deadband: uint8_t,
    pub alt_hold_fast_change: uint8_t,
    pub yaw_control_reversed: bool,
}
pub type rcControlsConfig_t = rcControlsConfig_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_0 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_0 = 2;
pub const ARMED: C2RustUnnamed_0 = 1;
pub type armingDisableFlags_e = libc::c_uint;
pub const ARMING_DISABLED_ARM_SWITCH: armingDisableFlags_e = 524288;
pub const ARMING_DISABLED_GPS: armingDisableFlags_e = 262144;
pub const ARMING_DISABLED_PARALYZE: armingDisableFlags_e = 131072;
pub const ARMING_DISABLED_MSP: armingDisableFlags_e = 65536;
pub const ARMING_DISABLED_BST: armingDisableFlags_e = 32768;
pub const ARMING_DISABLED_OSD_MENU: armingDisableFlags_e = 16384;
pub const ARMING_DISABLED_CMS_MENU: armingDisableFlags_e = 8192;
pub const ARMING_DISABLED_CLI: armingDisableFlags_e = 4096;
pub const ARMING_DISABLED_CALIBRATING: armingDisableFlags_e = 2048;
pub const ARMING_DISABLED_LOAD: armingDisableFlags_e = 1024;
pub const ARMING_DISABLED_NOPREARM: armingDisableFlags_e = 512;
pub const ARMING_DISABLED_BOOT_GRACE_TIME: armingDisableFlags_e = 256;
pub const ARMING_DISABLED_ANGLE: armingDisableFlags_e = 128;
pub const ARMING_DISABLED_THROTTLE: armingDisableFlags_e = 64;
pub const ARMING_DISABLED_RUNAWAY_TAKEOFF: armingDisableFlags_e = 32;
pub const ARMING_DISABLED_BOXFAILSAFE: armingDisableFlags_e = 16;
pub const ARMING_DISABLED_BAD_RX_RECOVERY: armingDisableFlags_e = 8;
pub const ARMING_DISABLED_RX_FAILSAFE: armingDisableFlags_e = 4;
pub const ARMING_DISABLED_FAILSAFE: armingDisableFlags_e = 2;
pub const ARMING_DISABLED_NO_GYRO: armingDisableFlags_e = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_1 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_1 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_1 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_1 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_1 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_1 = 16;
pub const BARO_MODE: C2RustUnnamed_1 = 8;
pub const MAG_MODE: C2RustUnnamed_1 = 4;
pub const HORIZON_MODE: C2RustUnnamed_1 = 2;
pub const ANGLE_MODE: C2RustUnnamed_1 = 1;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_2 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_2 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_2 = 4;
pub const GPS_FIX: C2RustUnnamed_2 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_2 = 1;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FUNCTION_LIDAR_TF: C2RustUnnamed_3 = 32768;
pub const FUNCTION_RCDEVICE: C2RustUnnamed_3 = 16384;
pub const FUNCTION_VTX_TRAMP: C2RustUnnamed_3 = 8192;
pub const FUNCTION_TELEMETRY_IBUS: C2RustUnnamed_3 = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: C2RustUnnamed_3 = 2048;
pub const FUNCTION_ESC_SENSOR: C2RustUnnamed_3 = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: C2RustUnnamed_3 = 512;
pub const FUNCTION_BLACKBOX: C2RustUnnamed_3 = 128;
pub const FUNCTION_RX_SERIAL: C2RustUnnamed_3 = 64;
pub const FUNCTION_TELEMETRY_SMARTPORT: C2RustUnnamed_3 = 32;
pub const FUNCTION_TELEMETRY_LTM: C2RustUnnamed_3 = 16;
pub const FUNCTION_TELEMETRY_HOTT: C2RustUnnamed_3 = 8;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: C2RustUnnamed_3 = 4;
pub const FUNCTION_GPS: C2RustUnnamed_3 = 2;
pub const FUNCTION_MSP: C2RustUnnamed_3 = 1;
pub const FUNCTION_NONE: C2RustUnnamed_3 = 0;
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
// gyro alignment
// people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
// Gyro sample divider
// gyro DLPF setting
// gyro 32khz DLPF setting
// Lowpass primary/secondary
// Gyro calibration duration in 1/100 second
// bandpass quality factor, 100 for steep sided bandpass
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
// introduce a deadband around the stick center for yaw axis. Must be greater than zero.
// defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
// when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
// invert control direction of yaw
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 4],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
pub type serialConfig_t = serialConfig_s;
pub type ledStripFormatRGB_e = libc::c_uint;
pub const LED_RGB: ledStripFormatRGB_e = 1;
pub const LED_GRB: ledStripFormatRGB_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
pub type ledConfig_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ledStripConfig_s {
    pub ledConfigs: [ledConfig_t; 32],
    pub colors: [hsvColor_t; 16],
    pub modeColors: [modeColorIndexes_t; 6],
    pub specialColors: specialColorIndexes_t,
    pub ledstrip_visual_beeper: uint8_t,
    pub ledstrip_aux_channel: uint8_t,
    pub ioTag: ioTag_t,
    pub ledstrip_grb_rgb: ledStripFormatRGB_e,
}
pub type ledStripConfig_t = ledStripConfig_s;
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
// suppress LEDLOW mode if beeper is on
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
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 25;
pub const TASK_NONE: cfTaskId_e = 24;
pub const TASK_COUNT: cfTaskId_e = 24;
pub const TASK_PINIOBOX: cfTaskId_e = 23;
pub const TASK_RCDEVICE: cfTaskId_e = 22;
pub const TASK_CAMCTRL: cfTaskId_e = 21;
pub const TASK_VTXCTRL: cfTaskId_e = 20;
pub const TASK_CMS: cfTaskId_e = 19;
pub const TASK_ESC_SENSOR: cfTaskId_e = 18;
pub const TASK_BST_MASTER_PROCESS: cfTaskId_e = 17;
pub const TASK_LEDSTRIP: cfTaskId_e = 16;
pub const TASK_TELEMETRY: cfTaskId_e = 15;
pub const TASK_ALTITUDE: cfTaskId_e = 14;
pub const TASK_BARO: cfTaskId_e = 13;
pub const TASK_COMPASS: cfTaskId_e = 12;
pub const TASK_BEEPER: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct int16_flightDynamicsTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type flightDynamicsTrims_def_t = int16_flightDynamicsTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union flightDynamicsTrims_u {
    pub raw: [int16_t; 3],
    pub values: flightDynamicsTrims_def_t,
}
pub type flightDynamicsTrims_t = flightDynamicsTrims_u;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_4 = 64;
pub const SENSOR_GPS: C2RustUnnamed_4 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_4 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_4 = 16;
pub const SENSOR_MAG: C2RustUnnamed_4 = 8;
pub const SENSOR_BARO: C2RustUnnamed_4 = 4;
pub const SENSOR_ACC: C2RustUnnamed_4 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_4 = 1;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const VOLTAGE_SENSOR_ADC_5V: C2RustUnnamed_5 = 3;
pub const VOLTAGE_SENSOR_ADC_9V: C2RustUnnamed_5 = 2;
pub const VOLTAGE_SENSOR_ADC_12V: C2RustUnnamed_5 = 1;
pub const VOLTAGE_SENSOR_ADC_VBAT: C2RustUnnamed_5 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageSensorADCConfig_s {
    pub vbatscale: uint8_t,
    pub vbatresdivval: uint8_t,
    pub vbatresdivmultiplier: uint8_t,
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
// see also voltageMeterADCtoIDMap
pub type voltageSensorADCConfig_t = voltageSensorADCConfig_s;
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
pub type batteryConfig_t = batteryConfig_s;
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
pub type compassConfig_t = compassConfig_s;
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
// adjust this to match battery voltage to reported value
// resistor divider R2 (default NAZE 10(K))
// multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
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
// millis
// millis
pub type failsafeConfig_t = failsafeConfig_s;
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
// Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
// Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
// Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
// Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
// failsafe switch action is 0: stage1 (identical to rc link loss), 1: disarms instantly, 2: stage2
// selected full failsafe procedure is 0: auto-landing, 1: Drop it
//CAVEAT: This is used in the `motorConfig_t` parameter group, so the parameter group constraints apply
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
pub type C2RustUnnamed_7 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_7 = 5;
pub const PID_MAG: C2RustUnnamed_7 = 4;
pub const PID_LEVEL: C2RustUnnamed_7 = 3;
pub const PID_YAW: C2RustUnnamed_7 = 2;
pub const PID_PITCH: C2RustUnnamed_7 = 1;
pub const PID_ROLL: C2RustUnnamed_7 = 0;
pub type pidProfile_t = pidProfile_s;
// The update rate of motor outputs (50-498Hz)
// Pwm Protocol
// Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
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
// These must be consecutive, see 'reversedSources'
pub type C2RustUnnamed_8 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_8 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_8 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_8 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_8 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_8 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_8 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_8 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_8 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_8 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_8 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_8 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_8 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_8 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_8 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_8 = 0;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_9 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_9 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_9 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_9 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_9 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_9 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_9 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_9 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_9 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_9 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_9 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_9 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_9 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_9 = 8192;
pub const FEATURE_3D: C2RustUnnamed_9 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_9 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_9 = 512;
pub const FEATURE_GPS: C2RustUnnamed_9 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_9 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_9 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_9 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_9 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_9 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_9 = 1;
pub type box_t = box_e;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct box_e {
    pub boxId: uint8_t,
    pub boxName: *const libc::c_char,
    pub permanentId: uint8_t,
}
#[inline]
unsafe extern "C" fn gyroConfig() -> *const gyroConfig_t {
    return &mut gyroConfig_System;
}
#[inline]
unsafe extern "C" fn gyroConfigMutable() -> *mut gyroConfig_t {
    return &mut gyroConfig_System;
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
unsafe extern "C" fn modeActivationConditions(mut _index: libc::c_int)
 -> *const modeActivationCondition_t {
    return &mut *modeActivationConditions_SystemArray.as_mut_ptr().offset(_index
                                                                              as
                                                                              isize)
               as *mut modeActivationCondition_t;
}
#[inline]
unsafe extern "C" fn rcControlsConfigMutable() -> *mut rcControlsConfig_t {
    return &mut rcControlsConfig_System;
}
#[inline]
unsafe extern "C" fn rcControlsConfig() -> *const rcControlsConfig_t {
    return &mut rcControlsConfig_System;
}
#[inline]
unsafe extern "C" fn serialConfigMutable() -> *mut serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn ledStripConfigMutable() -> *mut ledStripConfig_t {
    return &mut ledStripConfig_System;
}
#[inline]
unsafe extern "C" fn ledStripConfig() -> *const ledStripConfig_t {
    return &mut ledStripConfig_System;
}
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
#[inline]
unsafe extern "C" fn rxConfigMutable() -> *mut rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfigMutable(mut _index: libc::c_int)
 -> *mut voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfig(mut _index: libc::c_int)
 -> *const voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
}
// see boxId_e
// GUI-readable box name
//
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
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfigMutable() -> *mut batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn compassConfigMutable() -> *mut compassConfig_t {
    return &mut compassConfig_System;
}
#[inline]
unsafe extern "C" fn compassConfig() -> *const compassConfig_t {
    return &mut compassConfig_System;
}
#[inline]
unsafe extern "C" fn failsafeConfigMutable() -> *mut failsafeConfig_t {
    return &mut failsafeConfig_System;
}
#[inline]
unsafe extern "C" fn failsafeConfig() -> *const failsafeConfig_t {
    return &mut failsafeConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfigMutable() -> *mut motorConfig_t {
    return &mut motorConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_8 = INPUT_STABILIZED_ROLL;
// this is calculated at startup based on enabled features.
static mut activeBoxIds: [uint8_t; 41] = [0; 41];
// this is the number of filled indexes in above array
static mut activeBoxIdCount: uint8_t = 0 as libc::c_int as uint8_t;
// from mixer.c
// cause reboot after BST processing complete
static mut isRebootScheduled: bool = 0 as libc::c_int != 0;
// FIXME remove ;'s
static mut boxes: [box_t; 42] =
    [{
         let mut init =
             box_e{boxId: BOXARM as libc::c_int as uint8_t,
                   boxName: b"ARM;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXANGLE as libc::c_int as uint8_t,
                   boxName: b"ANGLE;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 1 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXHORIZON as libc::c_int as uint8_t,
                   boxName:
                       b"HORIZON;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXBARO as libc::c_int as uint8_t,
                   boxName: b"BARO;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 3 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXMAG as libc::c_int as uint8_t,
                   boxName: b"MAG;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 5 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXHEADFREE as libc::c_int as uint8_t,
                   boxName:
                       b"HEADFREE;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 6 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXHEADADJ as libc::c_int as uint8_t,
                   boxName:
                       b"HEADADJ;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 7 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXCAMSTAB as libc::c_int as uint8_t,
                   boxName:
                       b"CAMSTAB;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 8 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXGPSHOME as libc::c_int as uint8_t,
                   boxName:
                       b"GPS HOME;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 10 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXGPSHOLD as libc::c_int as uint8_t,
                   boxName:
                       b"GPS HOLD;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 11 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXPASSTHRU as libc::c_int as uint8_t,
                   boxName:
                       b"PASSTHRU;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 12 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXBEEPERON as libc::c_int as uint8_t,
                   boxName:
                       b"BEEPER;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 13 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXLEDLOW as libc::c_int as uint8_t,
                   boxName:
                       b"LEDLOW;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 15 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXCALIB as libc::c_int as uint8_t,
                   boxName: b"CALIB;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 17 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXOSD as libc::c_int as uint8_t,
                   boxName:
                       b"OSD DISABLE SW;\x00" as *const u8 as
                           *const libc::c_char,
                   permanentId: 19 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXTELEMETRY as libc::c_int as uint8_t,
                   boxName:
                       b"TELEMETRY;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 20 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXSERVO1 as libc::c_int as uint8_t,
                   boxName:
                       b"SERVO1;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 23 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXSERVO2 as libc::c_int as uint8_t,
                   boxName:
                       b"SERVO2;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 24 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXSERVO3 as libc::c_int as uint8_t,
                   boxName:
                       b"SERVO3;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 25 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXBLACKBOX as libc::c_int as uint8_t,
                   boxName:
                       b"BLACKBOX;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 26 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: BOXFAILSAFE as libc::c_int as uint8_t,
                   boxName:
                       b"FAILSAFE;\x00" as *const u8 as *const libc::c_char,
                   permanentId: 27 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             box_e{boxId: CHECKBOX_ITEM_COUNT as libc::c_int as uint8_t,
                   boxName: 0 as *const libc::c_char,
                   permanentId: 0xff as libc::c_int as uint8_t,};
         init
     }, box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,},
     box_t{boxId: 0, boxName: 0 as *const libc::c_char, permanentId: 0,}];
/* ************************************************************************************************/
#[no_mangle]
pub static mut writeBufferPointer: uint8_t = 1 as libc::c_int as uint8_t;
unsafe extern "C" fn bstWrite8(mut data: uint8_t) {
    let fresh0 = writeBufferPointer;
    writeBufferPointer = writeBufferPointer.wrapping_add(1);
    writeData[fresh0 as usize] = data;
    writeData[0 as libc::c_int as usize] = writeBufferPointer;
}
unsafe extern "C" fn bstWrite16(mut data: uint16_t) {
    bstWrite8((data as libc::c_int >> 0 as libc::c_int) as uint8_t);
    bstWrite8((data as libc::c_int >> 8 as libc::c_int) as uint8_t);
}
unsafe extern "C" fn bstWrite32(mut data: uint32_t) {
    bstWrite16((data >> 0 as libc::c_int) as uint16_t);
    bstWrite16((data >> 16 as libc::c_int) as uint16_t);
}
#[no_mangle]
pub static mut readBufferPointer: uint8_t = 4 as libc::c_int as uint8_t;
unsafe extern "C" fn bstCurrentAddress() -> uint8_t {
    return readData[0 as libc::c_int as usize];
}
unsafe extern "C" fn bstRead8() -> uint8_t {
    let fresh1 = readBufferPointer;
    readBufferPointer = readBufferPointer.wrapping_add(1);
    return (readData[fresh1 as usize] as libc::c_int & 0xff as libc::c_int) as
               uint8_t;
}
unsafe extern "C" fn bstRead16() -> uint16_t {
    let mut t: uint16_t = bstRead8() as uint16_t;
    t =
        (t as libc::c_int +
             ((bstRead8() as uint16_t as libc::c_int) << 8 as libc::c_int)) as
            uint16_t;
    return t;
}
unsafe extern "C" fn bstRead32() -> uint32_t {
    let mut t: uint32_t = bstRead16() as uint32_t;
    t =
        (t as
             libc::c_uint).wrapping_add((bstRead16() as uint32_t) <<
                                            16 as libc::c_int) as uint32_t as
            uint32_t;
    return t;
}
unsafe extern "C" fn bstReadDataSize() -> uint8_t {
    return (readData[1 as libc::c_int as usize] as libc::c_int -
                5 as libc::c_int) as uint8_t;
}
unsafe extern "C" fn bstReadCRC() -> uint8_t {
    return readData[(readData[1 as libc::c_int as usize] as libc::c_int +
                         1 as libc::c_int) as usize];
}
unsafe extern "C" fn findBoxByPermenantId(mut permenantId: uint8_t)
 -> *const box_t {
    let mut boxIndex: uint8_t = 0;
    let mut candidate: *const box_t = 0 as *const box_t;
    boxIndex = 0 as libc::c_int as uint8_t;
    while (boxIndex as libc::c_ulong) <
              (::core::mem::size_of::<[box_t; 42]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<box_t>()
                                                   as libc::c_ulong) {
        candidate =
            &*boxes.as_ptr().offset(boxIndex as isize) as *const box_t;
        if (*candidate).permanentId as libc::c_int ==
               permenantId as libc::c_int {
            return candidate
        }
        boxIndex = boxIndex.wrapping_add(1)
    }
    return 0 as *const box_t;
}
unsafe extern "C" fn bstSlaveProcessFeedbackCommand(mut bstRequest: uint8_t)
 -> bool {
    let mut i: uint32_t = 0;
    let mut tmp: uint32_t = 0;
    let mut junk: uint32_t = 0;
    match bstRequest as libc::c_int {
        1 => {
            bstWrite8(0 as libc::c_int as uint8_t);
            bstWrite8(1 as libc::c_int as uint8_t);
            bstWrite8(13 as libc::c_int as uint8_t);
        }
        5 => {
            i = 0 as libc::c_int as uint32_t;
            while i < 11 as libc::c_int as libc::c_uint {
                bstWrite8(*buildDate.offset(i as isize) as uint8_t);
                i = i.wrapping_add(1)
            }
            i = 0 as libc::c_int as uint32_t;
            while i < 8 as libc::c_int as libc::c_uint {
                bstWrite8(*buildTime.offset(i as isize) as uint8_t);
                i = i.wrapping_add(1)
            }
            i = 0 as libc::c_int as uint32_t;
            while i < 7 as libc::c_int as libc::c_uint {
                bstWrite8(*shortGitRevision.offset(i as isize) as uint8_t);
                i = i.wrapping_add(1)
            }
        }
        101 => {
            bstWrite16(getTaskDeltaTime(TASK_GYROPID) as uint16_t);
            bstWrite16(i2cGetErrorCounter());
            bstWrite16((sensors(SENSOR_ACC as libc::c_int as uint32_t) as
                            libc::c_int |
                            (sensors(SENSOR_BARO as libc::c_int as uint32_t)
                                 as libc::c_int) << 1 as libc::c_int |
                            (sensors(SENSOR_MAG as libc::c_int as uint32_t) as
                                 libc::c_int) << 2 as libc::c_int |
                            (sensors(SENSOR_GPS as libc::c_int as uint32_t) as
                                 libc::c_int) << 3 as libc::c_int |
                            (sensors(SENSOR_RANGEFINDER as libc::c_int as
                                         uint32_t) as libc::c_int) <<
                                4 as libc::c_int) as uint16_t);
            // BST the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
            // Requires new Multiwii protocol version to fix
            // It would be preferable to setting the enabled bits based on BOXINDEX.
            junk = 0 as libc::c_int as uint32_t;
            tmp =
                ((if flightModeFlags as libc::c_int &
                         ANGLE_MODE as libc::c_int == 0 as libc::c_int {
                      0 as libc::c_int
                  } else { 1 as libc::c_int }) << BOXANGLE as libc::c_int |
                     (if flightModeFlags as libc::c_int &
                             HORIZON_MODE as libc::c_int == 0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXHORIZON as libc::c_int |
                     (if flightModeFlags as libc::c_int &
                             BARO_MODE as libc::c_int == 0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) << BOXBARO as libc::c_int |
                     (if flightModeFlags as libc::c_int &
                             MAG_MODE as libc::c_int == 0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) << BOXMAG as libc::c_int |
                     (if flightModeFlags as libc::c_int &
                             HEADFREE_MODE as libc::c_int == 0 as libc::c_int
                         {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXHEADFREE as libc::c_int |
                     (if IS_RC_MODE_ACTIVE(BOXHEADADJ) as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXHEADADJ as libc::c_int |
                     (if IS_RC_MODE_ACTIVE(BOXCAMSTAB) as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXCAMSTAB as libc::c_int |
                     (if flightModeFlags as libc::c_int &
                             GPS_HOME_MODE as libc::c_int == 0 as libc::c_int
                         {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXGPSHOME as libc::c_int |
                     (if flightModeFlags as libc::c_int &
                             GPS_HOLD_MODE as libc::c_int == 0 as libc::c_int
                         {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXGPSHOLD as libc::c_int |
                     (if flightModeFlags as libc::c_int &
                             PASSTHRU_MODE as libc::c_int == 0 as libc::c_int
                         {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXPASSTHRU as libc::c_int |
                     (if IS_RC_MODE_ACTIVE(BOXBEEPERON) as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXBEEPERON as libc::c_int |
                     (if IS_RC_MODE_ACTIVE(BOXLEDLOW) as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) << BOXLEDLOW as libc::c_int
                     |
                     (if IS_RC_MODE_ACTIVE(BOXCALIB) as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) << BOXCALIB as libc::c_int
                     |
                     (if IS_RC_MODE_ACTIVE(BOXOSD) as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) << BOXOSD as libc::c_int |
                     (if IS_RC_MODE_ACTIVE(BOXTELEMETRY) as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXTELEMETRY as libc::c_int |
                     (if armingFlags as libc::c_int & ARMED as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) << BOXARM as libc::c_int |
                     (if IS_RC_MODE_ACTIVE(BOXBLACKBOX) as libc::c_int ==
                             0 as libc::c_int {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXBLACKBOX as libc::c_int |
                     (if flightModeFlags as libc::c_int &
                             FAILSAFE_MODE as libc::c_int == 0 as libc::c_int
                         {
                          0 as libc::c_int
                      } else { 1 as libc::c_int }) <<
                         BOXFAILSAFE as libc::c_int) as uint32_t;
            i = 0 as libc::c_int as uint32_t;
            while i < activeBoxIdCount as libc::c_uint {
                let mut flag: libc::c_int =
                    (tmp &
                         ((1 as libc::c_int) <<
                              activeBoxIds[i as usize] as libc::c_int) as
                             libc::c_uint) as libc::c_int;
                if flag != 0 {
                    junk |= ((1 as libc::c_int) << i) as libc::c_uint
                }
                i = i.wrapping_add(1)
            }
            bstWrite32(junk);
            bstWrite8(getCurrentPidProfileIndex());
        }
        83 => { bstWrite16(getTaskDeltaTime(TASK_GYROPID) as uint16_t); }
        111 => {
            bstWrite8((*currentControlRateProfile).rcRates[FD_ROLL as
                                                               libc::c_int as
                                                               usize]);
            bstWrite8((*currentControlRateProfile).rcExpo[FD_ROLL as
                                                              libc::c_int as
                                                              usize]);
            i = 0 as libc::c_int as uint32_t;
            while i < 3 as libc::c_int as libc::c_uint {
                bstWrite8((*currentControlRateProfile).rates[i as usize]);
                i = i.wrapping_add(1)
                // R,P,Y see flight_dynamics_index_t
            } // gps_type
            bstWrite8((*currentControlRateProfile).dynThrPID); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
            bstWrite8((*currentControlRateProfile).thrMid8); // gps_ubx_sbas
            bstWrite8((*currentControlRateProfile).thrExpo8); // legacy - was multiwiiCurrentMeterOutput);
            bstWrite16((*currentControlRateProfile).tpa_breakpoint);
            bstWrite8((*currentControlRateProfile).rcExpo[FD_YAW as
                                                              libc::c_int as
                                                              usize]);
            bstWrite8((*currentControlRateProfile).rcRates[FD_YAW as
                                                               libc::c_int as
                                                               usize]);
        }
        112 => {
            i = 0 as libc::c_int as uint32_t;
            while i < PID_ITEM_COUNT as libc::c_int as libc::c_uint {
                bstWrite8((*currentPidProfile).pid[i as usize].P);
                bstWrite8((*currentPidProfile).pid[i as usize].I);
                bstWrite8((*currentPidProfile).pid[i as usize].D);
                i = i.wrapping_add(1)
            }
            pidInitConfig(currentPidProfile);
        }
        34 => {
            i = 0 as libc::c_int as uint32_t;
            while i < 20 as libc::c_int as libc::c_uint {
                let mut mac: *const modeActivationCondition_t =
                    modeActivationConditions(i as libc::c_int);
                let mut box_0: *const box_t =
                    &*boxes.as_ptr().offset((*mac).modeId as isize) as
                        *const box_t;
                bstWrite8((*box_0).permanentId);
                bstWrite8((*mac).auxChannelIndex);
                bstWrite8((*mac).range.startStep);
                bstWrite8((*mac).range.endStep);
                i = i.wrapping_add(1)
            }
        }
        114 => {
            bstWrite16((*rxConfig()).midrc);
            bstWrite16((*motorConfig()).minthrottle);
            bstWrite16((*motorConfig()).maxthrottle);
            bstWrite16((*motorConfig()).mincommand);
            bstWrite16((*failsafeConfig()).failsafe_throttle);
            bstWrite8(0 as libc::c_int as uint8_t);
            bstWrite8(0 as libc::c_int as uint8_t);
            bstWrite8(0 as libc::c_int as uint8_t);
            bstWrite8(0 as libc::c_int as uint8_t);
            bstWrite8((*rxConfig()).rssi_channel);
            bstWrite8(0 as libc::c_int as uint8_t);
            bstWrite16(((*compassConfig()).mag_declination as libc::c_int /
                            10 as libc::c_int) as uint16_t);
            bstWrite8((*voltageSensorADCConfig(VOLTAGE_SENSOR_ADC_VBAT as
                                                   libc::c_int)).vbatscale);
            bstWrite8((*batteryConfig()).vbatmincellvoltage);
            bstWrite8((*batteryConfig()).vbatmaxcellvoltage);
            bstWrite8((*batteryConfig()).vbatwarningcellvoltage);
        }
        36 => { bstWrite32(featureMask()); }
        44 => {
            bstWrite8((*rxConfig()).serialrx_provider);
            bstWrite16((*rxConfig()).maxcheck);
            bstWrite16((*rxConfig()).midrc);
            bstWrite16((*rxConfig()).mincheck);
            bstWrite8((*rxConfig()).spektrum_sat_bind);
            bstWrite16((*rxConfig()).rx_min_usec);
            bstWrite16((*rxConfig()).rx_max_usec);
        }
        64 => {
            i = 0 as libc::c_int as uint32_t;
            while i < 8 as libc::c_int as libc::c_uint {
                bstWrite8((*rxConfig()).rcmap[i as usize]);
                i = i.wrapping_add(1)
            }
        }
        46 => {
            i = 0 as libc::c_int as uint32_t;
            while i < 16 as libc::c_int as libc::c_uint {
                let mut color: *mut hsvColor_t =
                    &mut *(*(ledStripConfigMutable as
                                 unsafe extern "C" fn()
                                     ->
                                         *mut ledStripConfig_t)()).colors.as_mut_ptr().offset(i
                                                                                                  as
                                                                                                  isize)
                        as *mut hsvColor_t;
                bstWrite16((*color).h);
                bstWrite8((*color).s);
                bstWrite8((*color).v);
                i = i.wrapping_add(1)
            }
        }
        48 => {
            i = 0 as libc::c_int as uint32_t;
            while i < 32 as libc::c_int as libc::c_uint {
                let mut ledConfig: *const ledConfig_t =
                    &*(*(ledStripConfig as
                             unsafe extern "C" fn()
                                 ->
                                     *const ledStripConfig_t)()).ledConfigs.as_ptr().offset(i
                                                                                                as
                                                                                                isize)
                        as *const ledConfig_t;
                bstWrite32(*ledConfig);
                i = i.wrapping_add(1)
            }
        }
        72 => {
            bstWrite8((*rcControlsConfig()).alt_hold_deadband);
            bstWrite8((*rcControlsConfig()).alt_hold_fast_change);
            bstWrite8((*rcControlsConfig()).deadband);
            bstWrite8((*rcControlsConfig()).yaw_deadband);
        }
        74 => {
            match (*gyroConfig()).gyro_hardware_lpf as libc::c_int {
                2 => {
                    // Extra safety to prevent OSD setting corrupt values
                    bstWrite16(1 as libc::c_int as uint16_t);
                }
                _ => { bstWrite16(0 as libc::c_int as uint16_t); }
            }
        }
        _ => {
            // we do not know how to handle the (valid) message, indicate error BST
            return 0 as libc::c_int != 0
        }
    } // gps_type
    return 1 as libc::c_int != 0; // gps_baudrate
}
unsafe extern "C" fn bstSlaveProcessWriteCommand(mut bstWriteCommand: uint8_t)
 -> bool {
    let mut i: uint32_t = 0; // gps_ubx_sbas
    let mut tmp: uint16_t = 0; // legacy - was multiwiiCurrentMeterOutput
    let mut ret: bool =
        0x1 as libc::c_int != 0; // actual vbatscale as intended
    match bstWriteCommand as libc::c_int {
        210 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                changePidProfile(bstRead8()); // vbatlevel_warn1 in MWC2.3 GUI
            }
        }
        84 => {
            bstRead16(); // vbatlevel_warn2 in MWC2.3 GUI
        }
        202 => {
            i =
                0 as libc::c_int as
                    uint32_t; // vbatlevel when buzzer starts to alert
            while i < PID_ITEM_COUNT as libc::c_int as libc::c_uint {
                (*currentPidProfile).pid[i as usize].P =
                    bstRead8(); // features bitmap
                (*currentPidProfile).pid[i as usize].I = bstRead8();
                (*currentPidProfile).pid[i as usize].D = bstRead8();
                i = i.wrapping_add(1)
            }
        }
        204 => {
            if bstReadDataSize() as libc::c_int >= 10 as libc::c_int {
                let mut rate: uint8_t = 0;
                (*currentControlRateProfile).rcRates[FD_ROLL as libc::c_int as
                                                         usize] = bstRead8();
                (*currentControlRateProfile).rcExpo[FD_ROLL as libc::c_int as
                                                        usize] = bstRead8();
                i = 0 as libc::c_int as uint32_t;
                while i < 3 as libc::c_int as libc::c_uint {
                    (*currentControlRateProfile).rates[i as usize] =
                        bstRead8();
                    i = i.wrapping_add(1)
                }
                rate = bstRead8();
                (*currentControlRateProfile).dynThrPID =
                    ({
                         let mut _a: uint8_t = rate;
                         let mut _b: libc::c_int = 100 as libc::c_int;
                         if (_a as libc::c_int) < _b {
                             _a as libc::c_int
                         } else { _b }
                     }) as uint8_t;
                (*currentControlRateProfile).thrMid8 = bstRead8();
                (*currentControlRateProfile).thrExpo8 = bstRead8();
                (*currentControlRateProfile).tpa_breakpoint = bstRead16();
                if bstReadDataSize() as libc::c_int >= 11 as libc::c_int {
                    (*currentControlRateProfile).rcExpo[FD_YAW as libc::c_int
                                                            as usize] =
                        bstRead8()
                }
                if bstReadDataSize() as libc::c_int >= 12 as libc::c_int {
                    (*currentControlRateProfile).rcRates[FD_YAW as libc::c_int
                                                             as usize] =
                        bstRead8()
                }
            } else { ret = 0 as libc::c_int != 0 }
        }
        35 => {
            i = bstRead8() as uint32_t;
            if i < 20 as libc::c_int as libc::c_uint {
                let mut mac: *mut modeActivationCondition_t =
                    modeActivationConditionsMutable(i as libc::c_int);
                i = bstRead8() as uint32_t;
                let mut box_0: *const box_t =
                    findBoxByPermenantId(i as uint8_t);
                if !box_0.is_null() {
                    (*mac).modeId = (*box_0).boxId as boxId_e;
                    (*mac).auxChannelIndex = bstRead8();
                    (*mac).range.startStep = bstRead8();
                    (*mac).range.endStep = bstRead8();
                    useRcControlsConfig(currentPidProfile);
                } else { ret = 0 as libc::c_int != 0 }
            } else { ret = 0 as libc::c_int != 0 }
        }
        207 => {
            tmp = bstRead16();
            if (tmp as libc::c_int) < 1600 as libc::c_int &&
                   tmp as libc::c_int > 1400 as libc::c_int {
                (*rxConfigMutable()).midrc = tmp
            }
            (*motorConfigMutable()).minthrottle = bstRead16();
            (*motorConfigMutable()).maxthrottle = bstRead16();
            (*motorConfigMutable()).mincommand = bstRead16();
            (*failsafeConfigMutable()).failsafe_throttle = bstRead16();
            bstRead8();
            bstRead8();
            bstRead8();
            bstRead8();
            (*rxConfigMutable()).rssi_channel = bstRead8();
            bstRead8();
            (*compassConfigMutable()).mag_declination =
                (bstRead16() as libc::c_int * 10 as libc::c_int) as int16_t;
            (*voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT as
                                                libc::c_int)).vbatscale =
                bstRead8();
            (*batteryConfigMutable()).vbatmincellvoltage = bstRead8();
            (*batteryConfigMutable()).vbatmaxcellvoltage = bstRead8();
            (*batteryConfigMutable()).vbatwarningcellvoltage = bstRead8()
        }
        205 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                accSetCalibrationCycles(400 as libc::c_int as uint16_t);
            }
        }
        206 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
                stateFlags =
                    (stateFlags as libc::c_int | CALIBRATE_MAG as libc::c_int)
                        as uint8_t
            }
        }
        250 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                ret = 0 as libc::c_int != 0;
                bstWrite8(ret as uint8_t);
                return ret
            }
            writeEEPROM();
            readEEPROM();
        }
        37 => {
            featureClearAll();
            featureSet(bstRead32());
            if featureConfigured(FEATURE_RX_SERIAL as libc::c_int as uint32_t)
               {
                (*serialConfigMutable()).portConfigs[SERIAL_PORT_USART2 as
                                                         libc::c_int as
                                                         usize].functionMask =
                    FUNCTION_RX_SERIAL as libc::c_int as uint16_t
            } else {
                (*serialConfigMutable()).portConfigs[SERIAL_PORT_USART2 as
                                                         libc::c_int as
                                                         usize].functionMask =
                    FUNCTION_NONE as libc::c_int as uint16_t
            }
        }
        45 => {
            (*rxConfigMutable()).serialrx_provider = bstRead8();
            (*rxConfigMutable()).maxcheck = bstRead16();
            (*rxConfigMutable()).midrc = bstRead16();
            (*rxConfigMutable()).mincheck = bstRead16();
            (*rxConfigMutable()).spektrum_sat_bind = bstRead8();
            if bstReadDataSize() as libc::c_int > 8 as libc::c_int {
                (*rxConfigMutable()).rx_min_usec = bstRead16();
                (*rxConfigMutable()).rx_max_usec = bstRead16()
            }
        }
        65 => {
            i = 0 as libc::c_int as uint32_t;
            while i < 8 as libc::c_int as libc::c_uint {
                (*rxConfigMutable()).rcmap[i as usize] = bstRead8();
                i = i.wrapping_add(1)
            }
        }
        47 => {
            //for (i = 0; i < CONFIGURABLE_COLOR_COUNT; i++) {
            i = bstRead8() as uint32_t;
            let mut color: *mut hsvColor_t =
                &mut *(*(ledStripConfigMutable as
                             unsafe extern "C" fn()
                                 ->
                                     *mut ledStripConfig_t)()).colors.as_mut_ptr().offset(i
                                                                                              as
                                                                                              isize)
                    as *mut hsvColor_t;
            (*color).h = bstRead16();
            (*color).s = bstRead8();
            (*color).v = bstRead8()
        }
        49 => {
            i = bstRead8() as uint32_t;
            if i >= 32 as libc::c_int as libc::c_uint ||
                   bstReadDataSize() as libc::c_int !=
                       1 as libc::c_int + 4 as libc::c_int {
                ret = 0 as libc::c_int != 0
            } else {
                let mut ledConfig: *mut ledConfig_t =
                    &mut *(*(ledStripConfigMutable as
                                 unsafe extern "C" fn()
                                     ->
                                         *mut ledStripConfig_t)()).ledConfigs.as_mut_ptr().offset(i
                                                                                                      as
                                                                                                      isize)
                        as *mut ledConfig_t;
                *ledConfig = bstRead32();
                reevaluateLedConfig();
            }
        }
        68 => { isRebootScheduled = 1 as libc::c_int != 0 }
        70 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                disarm();
            }
            setArmingDisabled(ARMING_DISABLED_BST);
        }
        71 => { unsetArmingDisabled(ARMING_DISABLED_BST); }
        73 => {
            (*rcControlsConfigMutable()).alt_hold_deadband = bstRead8();
            (*rcControlsConfigMutable()).alt_hold_fast_change = bstRead8();
            (*rcControlsConfigMutable()).deadband = bstRead8();
            (*rcControlsConfigMutable()).yaw_deadband = bstRead8()
        }
        75 => {
            match bstRead16() as libc::c_int {
                1 => {
                    (*gyroConfigMutable()).gyro_hardware_lpf =
                        2 as libc::c_int as uint8_t
                }
                _ => {
                    (*gyroConfigMutable()).gyro_hardware_lpf =
                        0 as libc::c_int as uint8_t
                }
            }
        }
        _ => {
            // we do not know how to handle the (valid) message, indicate error BST
            ret = 0 as libc::c_int != 0
        }
    }
    bstWrite8(ret as uint8_t);
    if ret as libc::c_int == 0 as libc::c_int { return 0 as libc::c_int != 0 }
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn bstSlaveUSBCommandFeedback() -> bool 
 /*uint8_t bstFeedback*/
 {
    bstWrite8(0x5 as libc::c_int as uint8_t); //Sub CPU Device Info FRAME
    bstWrite8((sensors(SENSOR_ACC as libc::c_int as uint32_t) as libc::c_int |
                   (sensors(SENSOR_BARO as libc::c_int as uint32_t) as
                        libc::c_int) << 1 as libc::c_int |
                   (sensors(SENSOR_MAG as libc::c_int as uint32_t) as
                        libc::c_int) << 2 as libc::c_int |
                   (sensors(SENSOR_GPS as libc::c_int as uint32_t) as
                        libc::c_int) << 3 as libc::c_int |
                   (sensors(SENSOR_RANGEFINDER as libc::c_int as uint32_t) as
                        libc::c_int) << 4 as libc::c_int) as
                  uint8_t); //Firmware ID
    bstWrite8(0 as libc::c_int as uint8_t); //Firmware ID
    bstWrite8(0 as libc::c_int as uint8_t);
    bstWrite8(0 as libc::c_int as uint8_t);
    bstWrite8(2 as libc::c_int as uint8_t);
    bstWrite8(5 as libc::c_int as uint8_t);
    bstWrite8(0 as libc::c_int as uint8_t);
    bstWrite8(0 as libc::c_int as uint8_t);
    return 1 as libc::c_int != 0;
}
//micro-seconds
#[no_mangle]
pub static mut resetBstTimer: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut needResetCheck: bool = 1 as libc::c_int != 0;
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
pub unsafe extern "C" fn bstProcessInCommand() {
    readBufferPointer = 2 as libc::c_int as uint8_t;
    if bstCurrentAddress() as libc::c_int == 0xc8 as libc::c_int {
        if bstReadCRC() as libc::c_int == CRC8 as libc::c_int &&
               bstRead8() as libc::c_int == 0xa as libc::c_int {
            let mut i: uint8_t = 0;
            writeBufferPointer = 1 as libc::c_int as uint8_t;
            cleanflight_data_ready = 0 as libc::c_int != 0;
            i = 0 as libc::c_int as uint8_t;
            while (i as libc::c_int) < 128 as libc::c_int {
                writeData[i as usize] = 0 as libc::c_int as uint8_t;
                i = i.wrapping_add(1)
            }
            match bstRead8() as libc::c_int {
                4 => {
                    bstRead8();
                    if bstSlaveUSBCommandFeedback() {
                        ::core::ptr::write_volatile(&mut coreProReady as
                                                        *mut bool,
                                                    1 as libc::c_int != 0)
                    }
                }
                38 => {
                    bstWrite8(0x26 as libc::c_int as uint8_t);
                    bstSlaveProcessFeedbackCommand(bstRead8());
                }
                37 => {
                    bstWrite8(0x25 as libc::c_int as uint8_t);
                    bstSlaveProcessWriteCommand(bstRead8());
                }
                _ => { }
            }
            cleanflight_data_ready = 1 as libc::c_int != 0
        }
    } else if bstCurrentAddress() as libc::c_int == 0 as libc::c_int {
        if bstReadCRC() as libc::c_int == CRC8 as libc::c_int &&
               bstRead8() as libc::c_int == 0xb as libc::c_int {
            resetBstTimer = micros();
            needResetCheck = 1 as libc::c_int != 0
        }
    };
}
unsafe extern "C" fn resetBstChecker(mut currentTimeUs: timeUs_t) {
    if needResetCheck {
        if currentTimeUs as libc::c_double >=
               resetBstTimer as libc::c_double +
                   1.2f64 * 1000 as libc::c_int as libc::c_double *
                       1000 as libc::c_int as libc::c_double {
            bstTimeoutUserCallback();
            needResetCheck = 0 as libc::c_int != 0
        }
    };
}
static mut next02hzUpdateAt_1: uint32_t = 0 as libc::c_int as uint32_t;
static mut next20hzUpdateAt_1: uint32_t = 0 as libc::c_int as uint32_t;
static mut sendCounter: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn taskBstMasterProcess(mut currentTimeUs: timeUs_t) {
    if coreProReady {
        if currentTimeUs >= next02hzUpdateAt_1 && !bstWriteBusy() {
            writeFCModeToBST();
            next02hzUpdateAt_1 =
                currentTimeUs.wrapping_add((1000 as libc::c_int *
                                                1000 as libc::c_int /
                                                2 as libc::c_int) as
                                               libc::c_uint)
        }
        if currentTimeUs >= next20hzUpdateAt_1 && !bstWriteBusy() {
            if sendCounter as libc::c_int == 0 as libc::c_int {
                writeRCChannelToBST();
            } else if sendCounter as libc::c_int == 1 as libc::c_int {
                writeRollPitchYawToBST();
            }
            sendCounter = sendCounter.wrapping_add(1);
            if sendCounter as libc::c_int > 1 as libc::c_int {
                sendCounter = 0 as libc::c_int as uint8_t
            }
            next20hzUpdateAt_1 =
                currentTimeUs.wrapping_add((1000 as libc::c_int *
                                                1000 as libc::c_int /
                                                20 as libc::c_int) as
                                               libc::c_uint)
        }
    }
    bstMasterWriteLoop();
    if isRebootScheduled { stopMotors(); systemReset(); }
    resetBstChecker(currentTimeUs);
}
/* ************************************************************************************************/
static mut masterWriteBufferPointer: uint8_t = 0;
static mut masterWriteData: [uint8_t; 128] = [0; 128];
unsafe extern "C" fn bstMasterStartBuffer(mut address: uint8_t) {
    masterWriteData[0 as libc::c_int as usize] =
        address; //radiusHeading * 10000;
    masterWriteBufferPointer = 2 as libc::c_int as uint8_t;
}
unsafe extern "C" fn bstMasterWrite8(mut data: uint8_t) {
    let fresh2 = masterWriteBufferPointer;
    masterWriteBufferPointer = masterWriteBufferPointer.wrapping_add(1);
    masterWriteData[fresh2 as usize] = data;
    masterWriteData[1 as libc::c_int as usize] = masterWriteBufferPointer;
}
unsafe extern "C" fn bstMasterWrite16(mut data: uint16_t) {
    bstMasterWrite8((data as libc::c_int >> 8 as libc::c_int) as uint8_t);
    bstMasterWrite8((data as libc::c_int >> 0 as libc::c_int) as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn writeRollPitchYawToBST() -> bool {
    let mut X: int16_t =
        (-(attitude.values.pitch as libc::c_int) as libc::c_float *
             (3.14159265358979323846f32 / 1800.0f32) *
             10000 as libc::c_int as libc::c_float) as int16_t;
    let mut Y: int16_t =
        (attitude.values.roll as libc::c_int as libc::c_float *
             (3.14159265358979323846f32 / 1800.0f32) *
             10000 as libc::c_int as libc::c_float) as int16_t;
    let mut Z: int16_t = 0 as libc::c_int as int16_t;
    bstMasterStartBuffer(0 as libc::c_int as uint8_t);
    bstMasterWrite8(0x1e as libc::c_int as uint8_t);
    bstMasterWrite16(X as uint16_t);
    bstMasterWrite16(Y as uint16_t);
    bstMasterWrite16(Z as uint16_t);
    return bstMasterWrite(masterWriteData.as_mut_ptr());
}
#[no_mangle]
pub unsafe extern "C" fn writeRCChannelToBST() -> bool {
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    bstMasterStartBuffer(0 as libc::c_int as uint8_t);
    bstMasterWrite8(0x15 as libc::c_int as uint8_t);
    i = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 12 as libc::c_int - 1 as libc::c_int {
        bstMasterWrite16(rcData[i as usize] as uint16_t);
        i = i.wrapping_add(1)
    }
    return bstMasterWrite(masterWriteData.as_mut_ptr());
}
#[no_mangle]
pub unsafe extern "C" fn writeFCModeToBST() -> bool {
    let mut tmp: uint8_t = 0 as libc::c_int as uint8_t;
    tmp =
        ((if armingFlags as libc::c_int & ARMED as libc::c_int ==
                 0 as libc::c_int {
              0 as libc::c_int
          } else { 1 as libc::c_int }) |
             (if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int ==
                     0 as libc::c_int {
                  0 as libc::c_int
              } else { 1 as libc::c_int }) << 1 as libc::c_int |
             (if flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int
                     == 0 as libc::c_int {
                  0 as libc::c_int
              } else { 1 as libc::c_int }) << 2 as libc::c_int |
             (if flightModeFlags as libc::c_int & BARO_MODE as libc::c_int ==
                     0 as libc::c_int {
                  0 as libc::c_int
              } else { 1 as libc::c_int }) << 3 as libc::c_int |
             (if flightModeFlags as libc::c_int & MAG_MODE as libc::c_int ==
                     0 as libc::c_int {
                  0 as libc::c_int
              } else { 1 as libc::c_int }) << 4 as libc::c_int |
             (if IS_RC_MODE_ACTIVE(BOXAIRMODE) as libc::c_int ==
                     0 as libc::c_int {
                  0 as libc::c_int
              } else { 1 as libc::c_int }) << 5 as libc::c_int |
             (if flightModeFlags as libc::c_int & FAILSAFE_MODE as libc::c_int
                     == 0 as libc::c_int {
                  0 as libc::c_int
              } else { 1 as libc::c_int }) << 7 as libc::c_int) as uint8_t;
    bstMasterStartBuffer(0 as libc::c_int as uint8_t);
    bstMasterWrite8(0x20 as libc::c_int as uint8_t);
    bstMasterWrite8(tmp);
    bstMasterWrite8((sensors(SENSOR_ACC as libc::c_int as uint32_t) as
                         libc::c_int |
                         (sensors(SENSOR_BARO as libc::c_int as uint32_t) as
                              libc::c_int) << 1 as libc::c_int |
                         (sensors(SENSOR_MAG as libc::c_int as uint32_t) as
                              libc::c_int) << 2 as libc::c_int |
                         (sensors(SENSOR_GPS as libc::c_int as uint32_t) as
                              libc::c_int) << 3 as libc::c_int |
                         (sensors(SENSOR_RANGEFINDER as libc::c_int as
                                      uint32_t) as libc::c_int) <<
                             4 as libc::c_int) as uint8_t);
    return bstMasterWrite(masterWriteData.as_mut_ptr());
}
/* ************************************************************************************************/
