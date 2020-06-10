use ::libc;
extern "C" {
    #[no_mangle]
    fn strncasecmp(_: *const libc::c_char, _: *const libc::c_char,
                   _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn blackboxInit();
    #[no_mangle]
    fn printfSupportInit();
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
    fn isEEPROMVersionValid() -> bool;
    #[no_mangle]
    fn latchActiveFeatures();
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn EXTIInit();
    #[no_mangle]
    fn targetBusInit();
    #[no_mangle]
    fn gyroStartCalibration(isFirstArmingCalibration: bool);
    #[no_mangle]
    fn detectBrushedESC();
    #[no_mangle]
    fn motorDevInit(motorDevConfig: *const motorDevConfig_t,
                    idlePulse: uint16_t, motorCount: uint8_t);
    #[no_mangle]
    fn servoDevInit(servoDevConfig: *const servoDevConfig_t);
    #[no_mangle]
    fn timerInit();
    #[no_mangle]
    fn timerStart();
    #[no_mangle]
    fn pwmEnableMotors();
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn adcInit(config: *const adcConfig_s);
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
    // Maximum page size of all supported SPI flash devices.
// Used to detect flashfs allocation size being too small.
    // Count of the number of erasable blocks on the device
    // In bytes
    // This is just pagesPerSector * pageSize
    // This is just sectorSize * sectors
    #[no_mangle]
    fn flashInit(flashConfig_0: *const flashConfig_t) -> bool;
    #[no_mangle]
    static mut flashConfig_System: flashConfig_t;
    #[no_mangle]
    fn IOInitGlobal();
    #[no_mangle]
    static mut serialPinConfig_System: serialPinConfig_t;
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
    fn initInverters(serialPinConfigToUse: *const serialPinConfig_t);
    #[no_mangle]
    static mut statusLedConfig_System: statusLedConfig_t;
    // Helpful macros
    #[no_mangle]
    fn ledInit(statusLedConfig_0: *const statusLedConfig_t);
    #[no_mangle]
    fn ledToggle(led: libc::c_int);
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    fn ppmRxInit(ppmConfig_0: *const ppmConfig_s);
    #[no_mangle]
    fn pwmRxInit(pwmConfig_0: *const pwmConfig_s);
    #[no_mangle]
    fn uartPinConfigure(pSerialPinConfig: *const serialPinConfig_t);
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
    fn beeperInit(beeperDevConfig_0: *const beeperDevConfig_s);
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
    fn systemInit();
    // failure
    #[no_mangle]
    fn indicateFailure(mode: failureMode_e, repeatCount: libc::c_int);
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    fn initEEPROM();
    #[no_mangle]
    fn resetEEPROM();
    #[no_mangle]
    fn readEEPROM() -> bool;
    #[no_mangle]
    fn ensureEEPROMStructureIsValid();
    #[no_mangle]
    fn validateAndFixGyroConfig();
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
    // Prevent too long busy wait times
    #[no_mangle]
    fn fcTasksInit();
    #[no_mangle]
    static mut flight3DConfig_System: flight3DConfig_t;
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn cliInit(serialConfig_0: *const serialConfig_s);
    #[no_mangle]
    fn mspInit();
    #[no_mangle]
    fn mspSerialInit();
    #[no_mangle]
    static mut adcConfig_System: adcConfig_t;
    #[no_mangle]
    static mut beeperConfig_System: beeperConfig_t;
    #[no_mangle]
    static mut beeperDevConfig_System: beeperDevConfig_t;
    #[no_mangle]
    static mut ppmConfig_System: ppmConfig_t;
    #[no_mangle]
    static mut pwmConfig_System: pwmConfig_t;
    // !!TODO remove this extern, only needed once for channelCount
    #[no_mangle]
    fn rxInit();
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    //
// configuration
//
    #[no_mangle]
    fn serialInit(softserialEnabled: bool,
                  serialPortToDisable: serialPortIdentifier_e);
    #[no_mangle]
    fn flashfsInit();
    #[no_mangle]
    fn ledStripInit();
    #[no_mangle]
    fn ledStripEnable();
    #[no_mangle]
    fn accInitFilters();
    #[no_mangle]
    fn accSetCalibrationCycles(calibrationCyclesRequired: uint16_t);
    #[no_mangle]
    fn baroSetCalibrationCycles(calibrationCyclesRequired: uint16_t);
    #[no_mangle]
    fn batteryInit();
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    static mut boardAlignment_System: boardAlignment_t;
    #[no_mangle]
    fn initBoardAlignment(boardAlignment_0: *const boardAlignment_t);
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
    fn sensorsAutodetect() -> bool;
    #[no_mangle]
    fn telemetryInit();
    #[no_mangle]
    fn failsafeInit();
    #[no_mangle]
    fn imuInit();
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    fn mixerInit(mixerMode: mixerMode_e);
    #[no_mangle]
    fn mixerConfigureOutput();
    #[no_mangle]
    fn pidInit(pidProfile: *const pidProfile_t);
    #[no_mangle]
    static mut servoConfig_System: servoConfig_t;
    #[no_mangle]
    fn isMixerUsingServos() -> bool;
    #[no_mangle]
    fn servoConfigureOutput();
    #[no_mangle]
    fn servosInit();
    #[no_mangle]
    fn servosFilterInit();
    #[no_mangle]
    fn updateHardwareRevision();
    #[no_mangle]
    fn detectHardwareRevision();
    #[no_mangle]
    static mut debugMode: uint8_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
// millisecond time
pub type timeMs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
pub type C2RustUnnamed = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed = 1048576;
pub const FEATURE_OSD: C2RustUnnamed = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed = 8192;
pub const FEATURE_3D: C2RustUnnamed = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed = 512;
pub const FEATURE_GPS: C2RustUnnamed = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
}
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
pub type displayPort_t = displayPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
}
// CMS state
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
pub const PWM_TYPE_MAX: C2RustUnnamed_0 = 5;
pub const PWM_TYPE_BRUSHED: C2RustUnnamed_0 = 4;
pub const PWM_TYPE_MULTISHOT: C2RustUnnamed_0 = 3;
pub const PWM_TYPE_ONESHOT42: C2RustUnnamed_0 = 2;
pub const PWM_TYPE_ONESHOT125: C2RustUnnamed_0 = 1;
pub const PWM_TYPE_STANDARD: C2RustUnnamed_0 = 0;
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
//CAVEAT: This is used in the `motorConfig_t` parameter group, so the parameter group constraints apply
pub type motorDevConfig_t = motorDevConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoDevConfig_s {
    pub servoCenterPulse: uint16_t,
    pub servoPwmRate: uint16_t,
    pub ioTags: [ioTag_t; 8],
}
pub type servoDevConfig_t = servoDevConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcConfig_s {
    pub vbat: adcChannelConfig_t,
    pub rssi: adcChannelConfig_t,
    pub current: adcChannelConfig_t,
    pub external1: adcChannelConfig_t,
    pub device: int8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcChannelConfig_t {
    pub enabled: bool,
    pub ioTag: ioTag_t,
}
// The update rate of motor outputs (50-498Hz)
// Pwm Protocol
// Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
// PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
// This is the value for servos when they should be in the middle. e.g. 1500.
// The update rate of servo outputs (50-498Hz)
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
pub struct flashConfig_s {
    pub csTag: ioTag_t,
    pub spiDevice: uint8_t,
}
pub type flashConfig_t = flashConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPinConfig_s {
    pub ioTagTx: [ioTag_t; 12],
    pub ioTagRx: [ioTag_t; 12],
    pub ioTagInverter: [ioTag_t; 12],
}
pub type serialPinConfig_t = serialPinConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct statusLedConfig_s {
    pub ioTags: [ioTag_t; 3],
    pub inversion: uint8_t,
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
pub type statusLedConfig_t = statusLedConfig_s;
pub type inputFilteringMode_e = libc::c_uint;
pub const INPUT_FILTERING_ENABLED: inputFilteringMode_e = 1;
pub const INPUT_FILTERING_DISABLED: inputFilteringMode_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ppmConfig_s {
    pub ioTag: ioTag_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmConfig_s {
    pub ioTags: [ioTag_t; 8],
    pub inputFilteringMode: inputFilteringMode_e,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct beeperDevConfig_s {
    pub ioTag: ioTag_t,
    pub isInverted: uint8_t,
    pub isOpenDrain: uint8_t,
    pub frequency: uint16_t,
}
pub type failureMode_e = libc::c_uint;
pub const FAILURE_GYRO_INIT_FAILED: failureMode_e = 6;
pub const FAILURE_FLASH_WRITE_FAILED: failureMode_e = 5;
pub const FAILURE_INVALID_EEPROM_CONTENTS: failureMode_e = 4;
pub const FAILURE_ACC_INCOMPATIBLE: failureMode_e = 3;
pub const FAILURE_ACC_INIT: failureMode_e = 2;
pub const FAILURE_MISSING_ACC: failureMode_e = 1;
pub const FAILURE_DEVELOPER: failureMode_e = 0;
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const SYSTEM_STATE_READY: C2RustUnnamed_1 = 128;
pub const SYSTEM_STATE_TRANSPONDER_ENABLED: C2RustUnnamed_1 = 8;
pub const SYSTEM_STATE_MOTORS_READY: C2RustUnnamed_1 = 4;
pub const SYSTEM_STATE_SENSORS_READY: C2RustUnnamed_1 = 2;
pub const SYSTEM_STATE_CONFIG_LOADED: C2RustUnnamed_1 = 1;
pub const SYSTEM_STATE_INITIALISING: C2RustUnnamed_1 = 0;
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
pub const SMALL_ANGLE: C2RustUnnamed_2 = 8;
pub const MIXER_GIMBAL: mixerMode = 5;
pub type mixerConfig_t = mixerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
pub type serialConfig_t = serialConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 4],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
// in seconds
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
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
pub const BEEPER_SYSTEM_INIT: C2RustUnnamed_3 = 17;
// not used for all telemetry systems, e.g. HoTT only works at 19200.
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
pub type beeperConfig_t = beeperConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct beeperConfig_s {
    pub beeper_off_flags: uint32_t,
    pub dshotBeaconTone: uint8_t,
    pub dshotBeaconOffFlags: uint32_t,
}
pub type servoConfig_t = servoConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoConfig_s {
    pub dev: servoDevConfig_t,
    pub servo_lowpass_freq: uint16_t,
    pub tri_unarmed_servo: uint8_t,
    pub channelForwardingStartChannel: uint8_t,
}
pub type pidProfile_t = pidProfile_s;
// lowpass servo filter frequency selection; 1/1000ths of loop freq
// send tail servo correction pulses even when unarmed
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
pub type boardAlignment_t = boardAlignment_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct boardAlignment_s {
    pub rollDegrees: int32_t,
    pub pitchDegrees: int32_t,
    pub yawDegrees: int32_t,
}
pub type adcConfig_t = adcConfig_s;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
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
// ADCDevice
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
pub type beeperDevConfig_t = beeperDevConfig_s;
pub type pwmConfig_t = pwmConfig_s;
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
pub type ppmConfig_t = ppmConfig_s;
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
pub type flight3DConfig_t = flight3DConfig_s;
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
// Digital protocol has fixed values
// Note: this is called MultiType/MULTITYPE_* in baseflight.
pub type mixerMode_e = mixerMode;
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
pub const MIXER_BICOPTER: mixerMode = 4;
pub const MIXER_QUADX: mixerMode = 3;
pub const MIXER_QUADP: mixerMode = 2;
pub const MIXER_TRI: mixerMode = 1;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_2 = 16;
pub const CALIBRATE_MAG: C2RustUnnamed_2 = 4;
pub const GPS_FIX: C2RustUnnamed_2 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_2 = 1;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const BEEPER_ALL: C2RustUnnamed_3 = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: C2RustUnnamed_3 = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: C2RustUnnamed_3 = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: C2RustUnnamed_3 = 21;
pub const BEEPER_CRASH_FLIP_MODE: C2RustUnnamed_3 = 20;
pub const BEEPER_BLACKBOX_ERASE: C2RustUnnamed_3 = 19;
pub const BEEPER_USB: C2RustUnnamed_3 = 18;
pub const BEEPER_ARMED: C2RustUnnamed_3 = 16;
pub const BEEPER_DISARM_REPEAT: C2RustUnnamed_3 = 15;
pub const BEEPER_MULTI_BEEPS: C2RustUnnamed_3 = 14;
pub const BEEPER_READY_BEEP: C2RustUnnamed_3 = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: C2RustUnnamed_3 = 12;
pub const BEEPER_ACC_CALIBRATION: C2RustUnnamed_3 = 11;
pub const BEEPER_RX_SET: C2RustUnnamed_3 = 10;
pub const BEEPER_GPS_STATUS: C2RustUnnamed_3 = 9;
pub const BEEPER_BAT_LOW: C2RustUnnamed_3 = 8;
pub const BEEPER_BAT_CRIT_LOW: C2RustUnnamed_3 = 7;
pub const BEEPER_ARMING_GPS_FIX: C2RustUnnamed_3 = 6;
pub const BEEPER_ARMING: C2RustUnnamed_3 = 5;
pub const BEEPER_DISARMING: C2RustUnnamed_3 = 4;
pub const BEEPER_RX_LOST_LANDING: C2RustUnnamed_3 = 3;
pub const BEEPER_RX_LOST: C2RustUnnamed_3 = 2;
pub const BEEPER_GYRO_CALIBRATED: C2RustUnnamed_3 = 1;
pub const BEEPER_SILENCE: C2RustUnnamed_3 = 0;
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
// airplane / singlecopter / dualcopter (not yet properly supported)
// PPM -> servo relay
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
pub type C2RustUnnamed_4 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_4 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_4 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_4 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_4 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_4 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_4 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_4 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_4 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_4 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_4 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_4 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_4 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_4 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_4 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_4 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcdeviceSwitchState_s {
    pub isActivated: bool,
}
pub type rcdeviceSwitchState_t = rcdeviceSwitchState_s;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn flashConfig() -> *const flashConfig_t {
    return &mut flashConfig_System;
}
#[inline]
unsafe extern "C" fn serialPinConfig() -> *const serialPinConfig_t {
    return &mut serialPinConfig_System;
}
#[inline]
unsafe extern "C" fn statusLedConfig() -> *const statusLedConfig_t {
    return &mut statusLedConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn flight3DConfig() -> *const flight3DConfig_t {
    return &mut flight3DConfig_System;
}
#[inline]
unsafe extern "C" fn adcConfig() -> *const adcConfig_t {
    return &mut adcConfig_System;
}
#[inline]
unsafe extern "C" fn adcConfigMutable() -> *mut adcConfig_t {
    return &mut adcConfig_System;
}
#[inline]
unsafe extern "C" fn beeperConfig() -> *const beeperConfig_t {
    return &mut beeperConfig_System;
}
#[inline]
unsafe extern "C" fn beeperDevConfig() -> *const beeperDevConfig_t {
    return &mut beeperDevConfig_System;
}
#[inline]
unsafe extern "C" fn ppmConfig() -> *const ppmConfig_t {
    return &mut ppmConfig_System;
}
#[inline]
unsafe extern "C" fn pwmConfig() -> *const pwmConfig_t {
    return &mut pwmConfig_System;
}
#[no_mangle]
pub static mut srxlDisplayPort: displayPort_t =
    displayPort_t{vTable: 0 as *const displayPortVTable_s,
                  device: 0 as *const libc::c_void as *mut libc::c_void,
                  rows: 0,
                  cols: 0,
                  posX: 0,
                  posY: 0,
                  cleared: false,
                  cursorRow: 0,
                  grabCount: 0,};
#[inline]
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
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
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn boardAlignment() -> *const boardAlignment_t {
    return &mut boardAlignment_System;
}
#[inline]
unsafe extern "C" fn mixerConfig() -> *const mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_4 = INPUT_STABILIZED_ROLL;
#[inline]
unsafe extern "C" fn servoConfig() -> *const servoConfig_t {
    return &mut servoConfig_System;
}
// used for unit test
#[no_mangle]
pub static mut switchStates: [rcdeviceSwitchState_t; 3] =
    [rcdeviceSwitchState_t{isActivated: false,}; 3];
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
pub static mut systemState: uint8_t =
    SYSTEM_STATE_INITIALISING as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn processLoopback() { }
#[no_mangle]
pub unsafe extern "C" fn init() {
    printfSupportInit();
    systemInit();
    // initialize IO (needed for all IO operations)
    IOInitGlobal();
    detectHardwareRevision();
    detectBrushedESC();
    initEEPROM();
    ensureEEPROMStructureIsValid();
    let mut readSuccess: bool = readEEPROM();
    if !readSuccess || !isEEPROMVersionValid() ||
           strncasecmp((*systemConfig()).boardIdentifier.as_ptr(),
                       b"AFMN\x00" as *const u8 as *const libc::c_char,
                       ::core::mem::size_of::<[libc::c_char; 5]>() as
                           libc::c_ulong) != 0 {
        resetEEPROM();
    }
    systemState =
        (systemState as libc::c_int |
             SYSTEM_STATE_CONFIG_LOADED as libc::c_int) as uint8_t;
    //i2cSetOverclock(masterConfig.i2c_overclock);
    debugMode = (*systemConfig()).debug_mode;
    // Latch active features to be used for feature() in the remainder of init().
    latchActiveFeatures(); // timer must be initialized before any channel is allocated
    ledInit(statusLedConfig());
    ledSet(2 as libc::c_int, 1 as libc::c_int != 0);
    EXTIInit();
    delay(100 as libc::c_int as timeMs_t);
    timerInit();
    uartPinConfigure(serialPinConfig());
    serialInit(feature(FEATURE_SOFTSERIAL as libc::c_int as uint32_t),
               SERIAL_PORT_NONE);
    mixerInit((*mixerConfig()).mixerMode as mixerMode_e);
    mixerConfigureOutput();
    let mut idlePulse: uint16_t = (*motorConfig()).mincommand;
    if feature(FEATURE_3D as libc::c_int as uint32_t) {
        idlePulse = (*flight3DConfig()).neutral3d
    }
    if (*motorConfig()).dev.motorPwmProtocol as libc::c_int ==
           PWM_TYPE_BRUSHED as libc::c_int {
        idlePulse = 0 as libc::c_int as uint16_t
        // brushed motors
    }
    /* Motors needs to be initialized soon as posible because hardware initialization
     * may send spurious pulses to esc's causing their early initialization. Also ppm
     * receiver may share timer with motors so motors MUST be initialized here. */
    motorDevInit(&(*(motorConfig as
                         unsafe extern "C" fn()
                             -> *const motorConfig_t)()).dev, idlePulse,
                 getMotorCount());
    systemState =
        (systemState as libc::c_int |
             SYSTEM_STATE_MOTORS_READY as libc::c_int) as uint8_t;
    if feature(FEATURE_RX_PPM as libc::c_int as uint32_t) {
        ppmRxInit(ppmConfig());
    } else if feature(FEATURE_RX_PARALLEL_PWM as libc::c_int as uint32_t) {
        pwmRxInit(pwmConfig());
    }
    beeperInit(beeperDevConfig());
    /* temp until PGs are implemented. */
    initInverters(serialPinConfig());
    targetBusInit();
    // TARGET_BUS_INIT
    updateHardwareRevision();
    // XXX These kind of code should goto target/config.c?
// XXX And these no longer work properly as FEATURE_RANGEFINDER does control HCSR04 runtime configuration.
    (*adcConfigMutable()).vbat.enabled =
        (*batteryConfig()).voltageMeterSource as libc::c_uint ==
            VOLTAGE_METER_ADC as libc::c_int as libc::c_uint;
    (*adcConfigMutable()).current.enabled =
        (*batteryConfig()).currentMeterSource as libc::c_uint ==
            CURRENT_METER_ADC as libc::c_int as libc::c_uint;
    // The FrSky D SPI RX sends RSSI_ADC_PIN (if configured) as A2
    (*adcConfigMutable()).rssi.enabled =
        feature(FEATURE_RSSI_ADC as libc::c_int as uint32_t);
    adcInit(adcConfig());
    initBoardAlignment(boardAlignment());
    if !sensorsAutodetect() {
        // if gyro was not detected due to whatever reason, notify and don't arm.
        indicateFailure(FAILURE_MISSING_ACC, 2 as libc::c_int);
        setArmingDisabled(ARMING_DISABLED_NO_GYRO);
    }
    systemState =
        (systemState as libc::c_int |
             SYSTEM_STATE_SENSORS_READY as libc::c_int) as uint8_t;
    // gyro.targetLooptime set in sensorsAutodetect(),
    // so we are ready to call validateAndFixGyroConfig(), pidInit(), and setAccelerationFilter()
    validateAndFixGyroConfig();
    pidInit(currentPidProfile);
    accInitFilters();
    servosInit();
    servoConfigureOutput();
    if isMixerUsingServos() {
        //pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
        servoDevInit(&(*(servoConfig as
                             unsafe extern "C" fn()
                                 -> *const servoConfig_t)()).dev);
    }
    servosFilterInit();
    ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    ledSet(2 as libc::c_int, 0 as libc::c_int != 0);
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 10 as libc::c_int {
        ledToggle(1 as libc::c_int);
        ledToggle(0 as libc::c_int);
        delay(25 as libc::c_int as timeMs_t);
        if (*beeperConfig()).beeper_off_flags &
               ((1 as libc::c_int) <<
                    BEEPER_SYSTEM_INIT as libc::c_int - 1 as libc::c_int) as
                   libc::c_uint == 0 {
            systemBeep(1 as libc::c_int != 0);
        }
        delay(25 as libc::c_int as timeMs_t);
        systemBeep(0 as libc::c_int != 0);
        i += 1
    }
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    imuInit();
    mspInit();
    mspSerialInit();
    cliInit(serialConfig());
    failsafeInit();
    rxInit();
    /*
 * CMS, display devices and OSD
 */
    ledStripInit();
    if feature(FEATURE_LED_STRIP as libc::c_int as uint32_t) {
        ledStripEnable();
    }
    if feature(FEATURE_TELEMETRY as libc::c_int as uint32_t) {
        telemetryInit();
    }
    flashInit(flashConfig());
    flashfsInit();
    blackboxInit();
    if (*mixerConfig()).mixerMode as libc::c_int ==
           MIXER_GIMBAL as libc::c_int {
        accSetCalibrationCycles(400 as libc::c_int as uint16_t);
    }
    gyroStartCalibration(0 as libc::c_int != 0);
    baroSetCalibrationCycles(200 as libc::c_int as uint16_t);
    // VTX_CONTROL
    // start all timers
    // TODO - not implemented yet
    timerStart(); // always needs doing, regardless of features.
    stateFlags =
        (stateFlags as libc::c_int | SMALL_ANGLE as libc::c_int) as uint8_t;
    batteryInit();
    // USE_RCDEVICE
    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();
    pwmEnableMotors();
    setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
    fcTasksInit();
    systemState =
        (systemState as libc::c_int | SYSTEM_STATE_READY as libc::c_int) as
            uint8_t;
}