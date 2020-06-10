use ::libc;
extern "C" {
    #[no_mangle]
    fn ffs(__i: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn fabsf(_: libc::c_float) -> libc::c_float;
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
    static mut blackboxConfig_System: blackboxConfig_t;
    #[no_mangle]
    fn blackboxUpdate(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn blackboxFinish();
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
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
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
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
    fn gyroUpdate(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn accIsCalibrationComplete() -> bool;
    #[no_mangle]
    static mut accelerometerConfig_System: accelerometerConfig_t;
    #[no_mangle]
    fn gyroReadTemperature();
    #[no_mangle]
    fn isGyroCalibrationComplete() -> bool;
    #[no_mangle]
    fn isFirstArmingGyroCalibrationRunning() -> bool;
    #[no_mangle]
    fn gyroStartCalibration(isFirstArmingCalibration: bool);
    #[no_mangle]
    fn gyroAbsRateDps(axis: libc::c_int) -> uint16_t;
    #[no_mangle]
    fn isBaroCalibrationComplete() -> bool;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    fn saveConfigAndNotify();
    #[no_mangle]
    static mut currentControlRateProfile: *mut controlRateConfig_t;
    #[no_mangle]
    fn processRcCommand();
    #[no_mangle]
    fn resetYawAxis();
    #[no_mangle]
    fn rcSmoothingInitializationComplete() -> bool;
    // enough for 4 x 3position switches / 4 aux channel
    #[no_mangle]
    fn updateAdjustmentStates();
    #[no_mangle]
    fn processRcAdjustments(controlRateConfig: *mut controlRateConfig_s);
    #[no_mangle]
    fn isModeActivationConditionPresent(modeId: boxId_e) -> bool;
    #[no_mangle]
    fn updateActivatedModes();
    #[no_mangle]
    fn isAirmodeActive() -> bool;
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut rcControlsConfig_System: rcControlsConfig_t;
    #[no_mangle]
    static mut flight3DConfig_System: flight3DConfig_t;
    #[no_mangle]
    static mut armingConfig_System: armingConfig_t;
    #[no_mangle]
    fn calculateThrottleStatus() -> throttleStatus_e;
    #[no_mangle]
    fn processRcStickPositions();
    #[no_mangle]
    fn isUsingSticksForArming() -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn unsetArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn isArmingDisabled() -> bool;
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn enableFlightMode(mask: flightModeFlags_e) -> uint16_t;
    #[no_mangle]
    fn disableFlightMode(mask: flightModeFlags_e) -> uint16_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    // null when port unused.
    #[no_mangle]
    fn mspSerialAllocatePorts();
    #[no_mangle]
    fn mspSerialReleaseSharedTelemetryPorts();
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
    static mut cliMode: uint8_t;
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    #[no_mangle]
    fn beeperSilence();
    #[no_mangle]
    fn beeperWarningBeeps(beepCount: uint8_t);
    #[no_mangle]
    fn getLastDshotBeaconCommandTimeUs() -> timeUs_t;
    #[no_mangle]
    fn warningLedDisable();
    #[no_mangle]
    fn warningLedUpdate();
    #[no_mangle]
    fn warningLedFlash();
    #[no_mangle]
    fn vtxUpdateActivatedChannel();
    #[no_mangle]
    fn handleVTXControlButton();
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn rxIsReceivingSignal() -> bool;
    #[no_mangle]
    fn calculateRxChannelsAndUpdateFailsafe(currentTimeUs: timeUs_t) -> bool;
    #[no_mangle]
    fn updateRSSI(currentTimeUs: timeUs_t);
    #[no_mangle]
    static mut averageSystemLoadPercent: uint16_t;
    #[no_mangle]
    fn rescheduleTask(taskId: cfTaskId_e, newPeriodMicros: uint32_t);
    #[no_mangle]
    fn getTaskDeltaTime(taskId: cfTaskId_e) -> timeDelta_t;
    #[no_mangle]
    fn telemetryCheckState();
    #[no_mangle]
    fn failsafeStartMonitoring();
    #[no_mangle]
    fn failsafeUpdateState();
    #[no_mangle]
    fn failsafeIsMonitoring() -> bool;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
    #[no_mangle]
    fn imuQuaternionHeadfreeOffsetSet() -> bool;
    #[no_mangle]
    fn isMotorProtocolDshot() -> bool;
    #[no_mangle]
    fn pwmWriteDshotCommand(index: uint8_t, motorCount: uint8_t,
                            command: uint8_t, blocking: bool);
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    fn mixTable(currentTimeUs: timeUs_t, vbatPidCompensation: uint8_t);
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    fn writeMotors();
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
    #[no_mangle]
    fn pidController(pidProfile: *const pidProfile_t,
                     angleTrim: *const rollAndPitchTrims_u,
                     currentTimeUs: timeUs_t);
    #[no_mangle]
    static mut pidData: [pidAxisData_t; 3];
    #[no_mangle]
    static mut targetPidLooptime: uint32_t;
    #[no_mangle]
    fn pidResetITerm();
    #[no_mangle]
    fn pidStabilisationState(pidControllerState: pidStabilisationState_e);
    #[no_mangle]
    fn pidAcroTrainerInit();
    #[no_mangle]
    fn pidSetAcroTrainerState(newState: bool);
    #[no_mangle]
    fn pidSetAntiGravityState(newState: bool);
    #[no_mangle]
    static mut servoConfig_System: servoConfig_t;
    #[no_mangle]
    fn isMixerUsingServos() -> bool;
    #[no_mangle]
    fn writeServos();
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
// function that resets a single parameter group instance
/* base */
/* size */
pub type pgRegistry_t = pgRegistry_s;
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
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type BlackboxMode = libc::c_uint;
pub const BLACKBOX_MODE_ALWAYS_ON: BlackboxMode = 2;
pub const BLACKBOX_MODE_MOTOR_TEST: BlackboxMode = 1;
pub const BLACKBOX_MODE_NORMAL: BlackboxMode = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FD_YAW: C2RustUnnamed_2 = 2;
pub const FD_PITCH: C2RustUnnamed_2 = 1;
pub const FD_ROLL: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_3 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_3 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_3 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_3 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_3 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_3 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_3 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_3 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_3 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_3 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_3 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_3 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_3 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_3 = 8192;
pub const FEATURE_3D: C2RustUnnamed_3 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_3 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_3 = 512;
pub const FEATURE_GPS: C2RustUnnamed_3 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_3 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_3 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_3 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_3 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_3 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_3 = 1;
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
// 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
pub type C2RustUnnamed_4 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_4 = 64;
pub const SENSOR_GPS: C2RustUnnamed_4 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_4 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_4 = 16;
pub const SENSOR_MAG: C2RustUnnamed_4 = 8;
pub const SENSOR_BARO: C2RustUnnamed_4 = 4;
// always present
pub const SENSOR_ACC: C2RustUnnamed_4 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_4 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
}
pub type rollAndPitchTrims_t_def = rollAndPitchTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union rollAndPitchTrims_u {
    pub raw: [int16_t; 2],
    pub values: rollAndPitchTrims_t_def,
}
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
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
pub type accelerometerConfig_t = accelerometerConfig_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct throttleCorrectionConfig_s {
    pub throttle_correction_angle: uint16_t,
    pub throttle_correction_value: uint8_t,
}
// in seconds
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
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
pub type throttleCorrectionConfig_t = throttleCorrectionConfig_s;
pub type beeperMode_e = libc::c_uint;
pub const BEEPER_ALL: beeperMode_e = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: beeperMode_e = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: beeperMode_e = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: beeperMode_e = 21;
pub const BEEPER_CRASH_FLIP_MODE: beeperMode_e = 20;
pub const BEEPER_BLACKBOX_ERASE: beeperMode_e = 19;
pub const BEEPER_USB: beeperMode_e = 18;
pub const BEEPER_SYSTEM_INIT: beeperMode_e = 17;
pub const BEEPER_ARMED: beeperMode_e = 16;
pub const BEEPER_DISARM_REPEAT: beeperMode_e = 15;
pub const BEEPER_MULTI_BEEPS: beeperMode_e = 14;
pub const BEEPER_READY_BEEP: beeperMode_e = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: beeperMode_e = 12;
pub const BEEPER_ACC_CALIBRATION: beeperMode_e = 11;
pub const BEEPER_RX_SET: beeperMode_e = 10;
pub const BEEPER_GPS_STATUS: beeperMode_e = 9;
pub const BEEPER_BAT_LOW: beeperMode_e = 8;
pub const BEEPER_BAT_CRIT_LOW: beeperMode_e = 7;
pub const BEEPER_ARMING_GPS_FIX: beeperMode_e = 6;
pub const BEEPER_ARMING: beeperMode_e = 5;
pub const BEEPER_DISARMING: beeperMode_e = 4;
pub const BEEPER_RX_LOST_LANDING: beeperMode_e = 3;
pub const BEEPER_RX_LOST: beeperMode_e = 2;
pub const BEEPER_GYRO_CALIBRATED: beeperMode_e = 1;
pub const BEEPER_SILENCE: beeperMode_e = 0;
pub const ARMING_DISABLED_RUNAWAY_TAKEOFF: armingDisableFlags_e = 32;
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
pub const ARMING_DISABLED_BOXFAILSAFE: armingDisableFlags_e = 16;
pub const ARMING_DISABLED_BAD_RX_RECOVERY: armingDisableFlags_e = 8;
pub const ARMING_DISABLED_RX_FAILSAFE: armingDisableFlags_e = 4;
pub const ARMING_DISABLED_FAILSAFE: armingDisableFlags_e = 2;
pub const ARMING_DISABLED_NO_GYRO: armingDisableFlags_e = 1;
pub const DSHOT_CMD_SPIN_DIRECTION_NORMAL: C2RustUnnamed_7 = 20;
pub const ARMED: C2RustUnnamed_5 = 1;
pub const ARMING_DELAYED_DISARMED: C2RustUnnamed_9 = 0;
pub type armingConfig_t = armingConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct armingConfig_s {
    pub gyro_cal_on_first_arm: uint8_t,
    pub auto_disarm_delay: uint8_t,
}
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_5 = 4;
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
pub const WAS_EVER_ARMED: C2RustUnnamed_5 = 2;
pub const DSHOT_CMD_SPIN_DIRECTION_REVERSED: C2RustUnnamed_7 = 21;
pub const ARMING_DELAYED_CRASHFLIP: C2RustUnnamed_9 = 2;
pub const ARMING_DELAYED_NORMAL: C2RustUnnamed_9 = 1;
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
pub const SMALL_ANGLE: C2RustUnnamed_6 = 8;
pub const THROTTLE_LOW: throttleStatus_e = 0;
pub type throttleStatus_e = libc::c_uint;
pub const THROTTLE_HIGH: throttleStatus_e = 1;
pub type flightModeFlags_e = libc::c_uint;
pub const GPS_RESCUE_MODE: flightModeFlags_e = 2048;
pub const FAILSAFE_MODE: flightModeFlags_e = 1024;
pub const PASSTHRU_MODE: flightModeFlags_e = 256;
pub const HEADFREE_MODE: flightModeFlags_e = 64;
pub const GPS_HOLD_MODE: flightModeFlags_e = 32;
pub const GPS_HOME_MODE: flightModeFlags_e = 16;
pub const BARO_MODE: flightModeFlags_e = 8;
pub const MAG_MODE: flightModeFlags_e = 4;
pub const HORIZON_MODE: flightModeFlags_e = 2;
pub const ANGLE_MODE: flightModeFlags_e = 1;
pub const MIXER_AIRPLANE: mixerMode = 14;
pub type mixerConfig_t = mixerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
pub const MIXER_FLYING_WING: mixerMode = 8;
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 24;
pub const TASK_NONE: cfTaskId_e = 23;
pub const TASK_COUNT: cfTaskId_e = 23;
pub const TASK_PINIOBOX: cfTaskId_e = 22;
pub const TASK_RCDEVICE: cfTaskId_e = 21;
pub const TASK_CAMCTRL: cfTaskId_e = 20;
pub const TASK_VTXCTRL: cfTaskId_e = 19;
pub const TASK_CMS: cfTaskId_e = 18;
pub const TASK_ESC_SENSOR: cfTaskId_e = 17;
pub const TASK_OSD: cfTaskId_e = 16;
pub const TASK_LEDSTRIP: cfTaskId_e = 15;
pub const TASK_TELEMETRY: cfTaskId_e = 14;
pub const TASK_ALTITUDE: cfTaskId_e = 13;
pub const TASK_BARO: cfTaskId_e = 12;
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
pub const THROTTLE: rc_alias = 3;
pub const FIXED_WING: C2RustUnnamed_6 = 16;
pub type pidConfig_t = pidConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
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
pub type pidStabilisationState_e = libc::c_uint;
pub const PID_STABILISATION_ON: pidStabilisationState_e = 1;
pub const PID_STABILISATION_OFF: pidStabilisationState_e = 0;
pub type pidProfile_t = pidProfile_s;
pub type servoConfig_t = servoConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoConfig_s {
    pub dev: servoDevConfig_t,
    pub servo_lowpass_freq: uint16_t,
    pub tri_unarmed_servo: uint8_t,
    pub channelForwardingStartChannel: uint8_t,
}
pub type servoDevConfig_t = servoDevConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoDevConfig_s {
    pub servoCenterPulse: uint16_t,
    pub servoPwmRate: uint16_t,
    pub ioTags: [ioTag_t; 8],
}
pub const MIXER_CUSTOM_TRI: mixerMode = 25;
pub const MIXER_TRI: mixerMode = 1;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub const YAW: rc_alias = 2;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const CALIBRATE_MAG: C2RustUnnamed_6 = 4;
pub const GPS_FIX: C2RustUnnamed_6 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_6 = 1;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const DSHOT_CMD_MAX: C2RustUnnamed_7 = 47;
pub const DSHOT_CMD_SILENT_MODE_ON_OFF: C2RustUnnamed_7 = 31;
pub const DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF: C2RustUnnamed_7 = 30;
pub const DSHOT_CMD_LED3_OFF: C2RustUnnamed_7 = 29;
pub const DSHOT_CMD_LED2_OFF: C2RustUnnamed_7 = 28;
pub const DSHOT_CMD_LED1_OFF: C2RustUnnamed_7 = 27;
pub const DSHOT_CMD_LED0_OFF: C2RustUnnamed_7 = 26;
pub const DSHOT_CMD_LED3_ON: C2RustUnnamed_7 = 25;
pub const DSHOT_CMD_LED2_ON: C2RustUnnamed_7 = 24;
pub const DSHOT_CMD_LED1_ON: C2RustUnnamed_7 = 23;
pub const DSHOT_CMD_LED0_ON: C2RustUnnamed_7 = 22;
pub const DSHOT_CMD_SAVE_SETTINGS: C2RustUnnamed_7 = 12;
pub const DSHOT_CMD_SETTINGS_REQUEST: C2RustUnnamed_7 = 11;
pub const DSHOT_CMD_3D_MODE_ON: C2RustUnnamed_7 = 10;
pub const DSHOT_CMD_3D_MODE_OFF: C2RustUnnamed_7 = 9;
pub const DSHOT_CMD_SPIN_DIRECTION_2: C2RustUnnamed_7 = 8;
pub const DSHOT_CMD_SPIN_DIRECTION_1: C2RustUnnamed_7 = 7;
pub const DSHOT_CMD_ESC_INFO: C2RustUnnamed_7 = 6;
pub const DSHOT_CMD_BEACON5: C2RustUnnamed_7 = 5;
pub const DSHOT_CMD_BEACON4: C2RustUnnamed_7 = 4;
pub const DSHOT_CMD_BEACON3: C2RustUnnamed_7 = 3;
pub const DSHOT_CMD_BEACON2: C2RustUnnamed_7 = 2;
pub const DSHOT_CMD_BEACON1: C2RustUnnamed_7 = 1;
pub const DSHOT_CMD_MOTOR_STOP: C2RustUnnamed_7 = 0;
pub type mixerMode = libc::c_uint;
pub const MIXER_QUADX_1234: mixerMode = 26;
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
pub const MIXER_OCTOFLATX: mixerMode = 13;
pub const MIXER_OCTOFLATP: mixerMode = 12;
pub const MIXER_OCTOX8: mixerMode = 11;
pub const MIXER_HEX6X: mixerMode = 10;
pub const MIXER_Y4: mixerMode = 9;
pub const MIXER_HEX6: mixerMode = 7;
pub const MIXER_Y6: mixerMode = 6;
pub const MIXER_GIMBAL: mixerMode = 5;
pub const MIXER_BICOPTER: mixerMode = 4;
pub const MIXER_QUADX: mixerMode = 3;
pub const MIXER_QUADP: mixerMode = 2;
// the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
// the correction that will be applied at throttle_correction_angle.
// allow disarm/arm on throttle down + roll left/right
// allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
// min 3d value
// max 3d value
// center 3d value
// default throttle deadband from MIDRC
// pwm output value for max negative thrust
// pwm output value for max positive thrust
// enable '3D Switched Mode'
// Processing denominator for PID controller vs gyro sampling rate
// off, on - enables pidsum runaway disarm logic
// delay in ms for "in-flight" conditions before deactivation (successful flight)
// minimum throttle percent required during deactivation phase
// introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
// introduce a deadband around the stick center for yaw axis. Must be greater than zero.
// defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
// when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
// invert control direction of yaw
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
// lowpass servo filter frequency selection; 1/1000ths of loop freq
// send tail servo correction pulses even when unarmed
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
// I-frame interval / P-frame interval
#[inline]
unsafe extern "C" fn blackboxConfig() -> *const blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
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
#[inline]
unsafe extern "C" fn accelerometerConfigMutable()
 -> *mut accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn accelerometerConfig() -> *const accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
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
unsafe extern "C" fn armingConfig() -> *const armingConfig_t {
    return &mut armingConfig_System;
}
#[inline]
unsafe extern "C" fn mixerConfig() -> *const mixerConfig_t {
    return &mut mixerConfig_System;
}
#[inline]
unsafe extern "C" fn pidConfig() -> *const pidConfig_t {
    return &mut pidConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_8 = INPUT_STABILIZED_ROLL;
#[inline]
unsafe extern "C" fn servoConfig() -> *const servoConfig_t {
    return &mut servoConfig_System;
}
// High throttle limit to accelerate deactivation (halves the deactivation delay)
static mut flipOverAfterCrashMode: bool = 0 as libc::c_int != 0;
static mut disarmAt: uint32_t = 0;
// Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero
#[no_mangle]
pub static mut isRXDataNew: bool = false;
static mut lastArmingDisabledReason: libc::c_int = 0 as libc::c_int;
static mut lastDisarmTimeUs: timeUs_t = 0;
static mut tryingToArm: libc::c_int = ARMING_DELAYED_DISARMED as libc::c_int;
static mut runawayTakeoffDeactivateUs: timeUs_t =
    0 as libc::c_int as timeUs_t;
static mut runawayTakeoffAccumulatedUs: timeUs_t =
    0 as libc::c_int as timeUs_t;
static mut runawayTakeoffCheckDisabled: bool = 0 as libc::c_int != 0;
static mut runawayTakeoffTriggerUs: timeUs_t = 0 as libc::c_int as timeUs_t;
static mut runawayTakeoffTemporarilyDisabled: bool = 0 as libc::c_int != 0;
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut throttleCorrectionConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (39 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<throttleCorrectionConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &throttleCorrectionConfig_System as
                                     *const throttleCorrectionConfig_t as
                                     *mut throttleCorrectionConfig_t as
                                     *mut uint8_t,
                             copy:
                                 &throttleCorrectionConfig_Copy as
                                     *const throttleCorrectionConfig_t as
                                     *mut throttleCorrectionConfig_t as
                                     *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_throttleCorrectionConfig
                                                         as
                                                         *const throttleCorrectionConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut throttleCorrectionConfig_Copy: throttleCorrectionConfig_t =
    throttleCorrectionConfig_t{throttle_correction_angle: 0,
                               throttle_correction_value: 0,};
#[no_mangle]
pub static mut throttleCorrectionConfig_System: throttleCorrectionConfig_t =
    throttleCorrectionConfig_t{throttle_correction_angle: 0,
                               throttle_correction_value: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_throttleCorrectionConfig:
           throttleCorrectionConfig_t =
    {
        let mut init =
            throttleCorrectionConfig_s{throttle_correction_angle:
                                           800 as libc::c_int as uint16_t,
                                       throttle_correction_value:
                                           0 as libc::c_int as uint8_t,};
        init
    };
// could be 80.0 deg with atlhold or 45.0 for fpv
#[no_mangle]
pub unsafe extern "C" fn applyAndSaveAccelerometerTrimsDelta(mut rollAndPitchTrimsDelta:
                                                                 *mut rollAndPitchTrims_t) {
    let ref mut fresh0 =
        (*accelerometerConfigMutable()).accelerometerTrims.values.roll;
    *fresh0 =
        (*fresh0 as libc::c_int +
             (*rollAndPitchTrimsDelta).values.roll as libc::c_int) as int16_t;
    let ref mut fresh1 =
        (*accelerometerConfigMutable()).accelerometerTrims.values.pitch;
    *fresh1 =
        (*fresh1 as libc::c_int +
             (*rollAndPitchTrimsDelta).values.pitch as libc::c_int) as
            int16_t;
    saveConfigAndNotify();
}
unsafe extern "C" fn isCalibrating() -> bool {
    if sensors(SENSOR_BARO as libc::c_int as uint32_t) as libc::c_int != 0 &&
           !isBaroCalibrationComplete() {
        return 1 as libc::c_int != 0
    }
    // Note: compass calibration is handled completely differently, outside of the main loop, see f.CALIBRATE_MAG
    return !accIsCalibrationComplete() &&
               sensors(SENSOR_ACC as libc::c_int as uint32_t) as libc::c_int
                   != 0 || !isGyroCalibrationComplete();
}
#[no_mangle]
pub unsafe extern "C" fn resetArmingDisabled() {
    lastArmingDisabledReason = 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn updateArmingStatus() {
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        ledSet(0 as libc::c_int, 1 as libc::c_int != 0);
    } else {
        // Check if the power on arming grace time has elapsed
        if getArmingDisableFlags() as libc::c_uint &
               ARMING_DISABLED_BOOT_GRACE_TIME as libc::c_int as libc::c_uint
               != 0 &&
               millis() >=
                   ((*systemConfig()).powerOnArmingGraceTime as libc::c_int *
                        1000 as libc::c_int) as libc::c_uint {
            // If so, unset the grace time arming disable flag
            unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }
        // Clear the crash flip active status
        flipOverAfterCrashMode = 0 as libc::c_int != 0;
        // If switch is used for arming then check it is not defaulting to on when the RX link recovers from a fault
        if !isUsingSticksForArming() {
            static mut hadRx: bool = 0 as libc::c_int != 0;
            let haveRx: bool = rxIsReceivingSignal();
            let justGotRxBack: bool = !hadRx && haveRx as libc::c_int != 0;
            if justGotRxBack as libc::c_int != 0 &&
                   IS_RC_MODE_ACTIVE(BOXARM) as libc::c_int != 0 {
                // If the RX has just started to receive a signal again and the arm switch is on, apply arming restriction
                setArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            } else if haveRx as libc::c_int != 0 && !IS_RC_MODE_ACTIVE(BOXARM)
             {
                // If RX signal is OK and the arm switch is off, remove arming restriction
                unsetArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            }
            hadRx = haveRx
        }
        if IS_RC_MODE_ACTIVE(BOXFAILSAFE) {
            setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        } else { unsetArmingDisabled(ARMING_DISABLED_BOXFAILSAFE); }
        if calculateThrottleStatus() as libc::c_uint !=
               THROTTLE_LOW as libc::c_int as libc::c_uint {
            setArmingDisabled(ARMING_DISABLED_THROTTLE);
        } else { unsetArmingDisabled(ARMING_DISABLED_THROTTLE); }
        if stateFlags as libc::c_int & SMALL_ANGLE as libc::c_int == 0 &&
               !IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH) {
            setArmingDisabled(ARMING_DISABLED_ANGLE);
        } else { unsetArmingDisabled(ARMING_DISABLED_ANGLE); }
        if averageSystemLoadPercent as libc::c_int > 100 as libc::c_int {
            setArmingDisabled(ARMING_DISABLED_LOAD);
        } else { unsetArmingDisabled(ARMING_DISABLED_LOAD); }
        if isCalibrating() {
            setArmingDisabled(ARMING_DISABLED_CALIBRATING);
        } else { unsetArmingDisabled(ARMING_DISABLED_CALIBRATING); }
        if isModeActivationConditionPresent(BOXPREARM) {
            if IS_RC_MODE_ACTIVE(BOXPREARM) as libc::c_int != 0 &&
                   armingFlags as libc::c_int &
                       WAS_ARMED_WITH_PREARM as libc::c_int == 0 {
                unsetArmingDisabled(ARMING_DISABLED_NOPREARM);
            } else { setArmingDisabled(ARMING_DISABLED_NOPREARM); }
        }
        if IS_RC_MODE_ACTIVE(BOXPARALYZE) {
            setArmingDisabled(ARMING_DISABLED_PARALYZE);
        }
        if !isUsingSticksForArming() {
            /* Ignore ARMING_DISABLED_CALIBRATING if we are going to calibrate gyro on first arm */
            let mut ignoreGyro: bool =
                (*armingConfig()).gyro_cal_on_first_arm as libc::c_int != 0 &&
                    getArmingDisableFlags() as libc::c_uint &
                        !(ARMING_DISABLED_ARM_SWITCH as libc::c_int |
                              ARMING_DISABLED_CALIBRATING as libc::c_int) as
                            libc::c_uint == 0;
            /* Ignore ARMING_DISABLED_THROTTLE (once arm switch is on) if we are in 3D mode */
            let mut ignoreThrottle: bool =
                feature(FEATURE_3D as libc::c_int as uint32_t) as libc::c_int
                    != 0 && !IS_RC_MODE_ACTIVE(BOX3D) &&
                    (*flight3DConfig()).switched_mode3d == 0 &&
                    getArmingDisableFlags() as libc::c_uint &
                        !(ARMING_DISABLED_ARM_SWITCH as libc::c_int |
                              ARMING_DISABLED_THROTTLE as libc::c_int) as
                            libc::c_uint == 0;
            if !IS_RC_MODE_ACTIVE(BOXARM) {
                unsetArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
            }
            // If arming is disabled and the ARM switch is on
            if isArmingDisabled() as libc::c_int != 0 && !ignoreGyro &&
                   !ignoreThrottle &&
                   IS_RC_MODE_ACTIVE(BOXARM) as libc::c_int != 0 {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            } else if !IS_RC_MODE_ACTIVE(BOXARM) {
                unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            }
        }
        if isArmingDisabled() {
            warningLedFlash();
        } else { warningLedDisable(); }
        warningLedUpdate();
    };
}
#[no_mangle]
pub unsafe extern "C" fn disarm() {
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        armingFlags =
            (armingFlags as libc::c_int & !(ARMED as libc::c_int)) as uint8_t;
        lastDisarmTimeUs = micros();
        if (*blackboxConfig()).device as libc::c_int != 0 &&
               (*blackboxConfig()).mode as libc::c_int !=
                   BLACKBOX_MODE_ALWAYS_ON as libc::c_int {
            // Close the log upon disarm except when logging mode is ALWAYS ON
            blackboxFinish();
        }
        systemBeep(0 as libc::c_int != 0);
        if isMotorProtocolDshot() as libc::c_int != 0 &&
               flipOverAfterCrashMode as libc::c_int != 0 &&
               !feature(FEATURE_3D as libc::c_int as uint32_t) {
            pwmWriteDshotCommand(255 as libc::c_int as uint8_t,
                                 getMotorCount(),
                                 DSHOT_CMD_SPIN_DIRECTION_NORMAL as
                                     libc::c_int as uint8_t,
                                 0 as libc::c_int != 0);
        }
        flipOverAfterCrashMode = 0 as libc::c_int != 0;
        // if ARMING_DISABLED_RUNAWAY_TAKEOFF is set then we want to play it's beep pattern instead
        if getArmingDisableFlags() as libc::c_uint &
               ARMING_DISABLED_RUNAWAY_TAKEOFF as libc::c_int as libc::c_uint
               == 0 {
            beeper(BEEPER_DISARMING);
            // emit disarm tone
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn tryArm() {
    if (*armingConfig()).gyro_cal_on_first_arm != 0 {
        gyroStartCalibration(1 as libc::c_int != 0);
    }
    updateArmingStatus();
    if !isArmingDisabled() {
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 { return }
        if micros().wrapping_sub(getLastDshotBeaconCommandTimeUs()) <
               1200000 as libc::c_int as libc::c_uint {
            if tryingToArm == ARMING_DELAYED_DISARMED as libc::c_int {
                if isModeActivationConditionPresent(BOXFLIPOVERAFTERCRASH) as
                       libc::c_int != 0 &&
                       IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH) as libc::c_int
                           != 0 {
                    tryingToArm = ARMING_DELAYED_CRASHFLIP as libc::c_int
                } else { tryingToArm = ARMING_DELAYED_NORMAL as libc::c_int }
            }
            return
        }
        if isMotorProtocolDshot() as libc::c_int != 0 &&
               isModeActivationConditionPresent(BOXFLIPOVERAFTERCRASH) as
                   libc::c_int != 0 {
            if !(IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH) as libc::c_int != 0
                     ||
                     tryingToArm == ARMING_DELAYED_CRASHFLIP as libc::c_int) {
                flipOverAfterCrashMode = 0 as libc::c_int != 0;
                if !feature(FEATURE_3D as libc::c_int as uint32_t) {
                    pwmWriteDshotCommand(255 as libc::c_int as uint8_t,
                                         getMotorCount(),
                                         DSHOT_CMD_SPIN_DIRECTION_NORMAL as
                                             libc::c_int as uint8_t,
                                         0 as libc::c_int != 0);
                }
            } else {
                flipOverAfterCrashMode = 1 as libc::c_int != 0;
                runawayTakeoffCheckDisabled = 0 as libc::c_int != 0;
                if !feature(FEATURE_3D as libc::c_int as uint32_t) {
                    pwmWriteDshotCommand(255 as libc::c_int as uint8_t,
                                         getMotorCount(),
                                         DSHOT_CMD_SPIN_DIRECTION_REVERSED as
                                             libc::c_int as uint8_t,
                                         0 as libc::c_int != 0);
                }
            }
        }
        armingFlags =
            (armingFlags as libc::c_int | ARMED as libc::c_int) as uint8_t;
        armingFlags =
            (armingFlags as libc::c_int | WAS_EVER_ARMED as libc::c_int) as
                uint8_t;
        resetTryingToArm();
        pidAcroTrainerInit();
        // USE_ACRO_TRAINER
        if isModeActivationConditionPresent(BOXPREARM) {
            armingFlags =
                (armingFlags as libc::c_int |
                     WAS_ARMED_WITH_PREARM as libc::c_int) as uint8_t
        } // start disarm timeout, will be extended when throttle is nonzero
        imuQuaternionHeadfreeOffsetSet();
        disarmAt =
            millis().wrapping_add(((*armingConfig()).auto_disarm_delay as
                                       libc::c_int * 1000 as libc::c_int) as
                                      libc::c_uint);
        lastArmingDisabledReason = 0 as libc::c_int;
        //beep to indicate arming
        beeper(BEEPER_ARMING);
        runawayTakeoffDeactivateUs = 0 as libc::c_int as timeUs_t;
        runawayTakeoffAccumulatedUs = 0 as libc::c_int as timeUs_t;
        runawayTakeoffTriggerUs = 0 as libc::c_int as timeUs_t
    } else {
        resetTryingToArm();
        if !isFirstArmingGyroCalibrationRunning() {
            let mut armingDisabledReason: libc::c_int =
                ffs(getArmingDisableFlags() as libc::c_int);
            if lastArmingDisabledReason != armingDisabledReason {
                lastArmingDisabledReason = armingDisabledReason;
                beeperWarningBeeps(armingDisabledReason as uint8_t);
            }
        }
    };
}
// Automatic ACC Offset Calibration
#[no_mangle]
pub static mut AccInflightCalibrationArmed: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut AccInflightCalibrationMeasurementDone: bool =
    0 as libc::c_int != 0;
#[no_mangle]
pub static mut AccInflightCalibrationSavetoEEProm: bool =
    0 as libc::c_int != 0;
#[no_mangle]
pub static mut AccInflightCalibrationActive: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut InflightcalibratingA: uint16_t = 0 as libc::c_int as uint16_t;
#[no_mangle]
pub unsafe extern "C" fn handleInflightCalibrationStickPosition() {
    if AccInflightCalibrationMeasurementDone {
        // trigger saving into eeprom after landing
        AccInflightCalibrationMeasurementDone = 0 as libc::c_int != 0;
        AccInflightCalibrationSavetoEEProm = 1 as libc::c_int != 0
    } else {
        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
        if AccInflightCalibrationArmed {
            beeper(BEEPER_ACC_CALIBRATION);
        } else { beeper(BEEPER_ACC_CALIBRATION_FAIL); }
    };
}
unsafe extern "C" fn updateInflightCalibrationState() {
    if AccInflightCalibrationArmed as libc::c_int != 0 &&
           armingFlags as libc::c_int & ARMED as libc::c_int != 0 &&
           rcData[THROTTLE as libc::c_int as usize] as libc::c_int >
               (*rxConfig()).mincheck as libc::c_int &&
           !IS_RC_MODE_ACTIVE(BOXARM) {
        // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50 as libc::c_int as uint16_t;
        AccInflightCalibrationArmed = 0 as libc::c_int != 0
    }
    if IS_RC_MODE_ACTIVE(BOXCALIB) {
        // Use the Calib Option to activate : Calib = TRUE measurement started, Land and Calib = 0 measurement stored
        if !AccInflightCalibrationActive &&
               !AccInflightCalibrationMeasurementDone {
            InflightcalibratingA = 50 as libc::c_int as uint16_t
        }
        AccInflightCalibrationActive = 1 as libc::c_int != 0
    } else if AccInflightCalibrationMeasurementDone as libc::c_int != 0 &&
                  armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        AccInflightCalibrationMeasurementDone = 0 as libc::c_int != 0;
        AccInflightCalibrationSavetoEEProm = 1 as libc::c_int != 0
    };
}
unsafe extern "C" fn canUpdateVTX() -> bool { return 1 as libc::c_int != 0; }
// determine if the R/P/Y stick deflection exceeds the specified limit - integer math is good enough here.
#[no_mangle]
pub unsafe extern "C" fn areSticksActive(mut stickPercentLimit: uint8_t)
 -> bool {
    let mut axis: libc::c_int = FD_ROLL as libc::c_int;
    while axis <= FD_YAW as libc::c_int {
        let deadband: uint8_t =
            if axis == FD_YAW as libc::c_int {
                (*rcControlsConfig()).yaw_deadband as libc::c_int
            } else { (*rcControlsConfig()).deadband as libc::c_int } as
                uint8_t;
        let mut stickPercent: uint8_t = 0 as libc::c_int as uint8_t;
        if rcData[axis as usize] as libc::c_int >= 2000 as libc::c_int ||
               rcData[axis as usize] as libc::c_int <= 1000 as libc::c_int {
            stickPercent = 100 as libc::c_int as uint8_t
        } else if rcData[axis as usize] as libc::c_int >
                      (*rxConfig()).midrc as libc::c_int +
                          deadband as libc::c_int {
            stickPercent =
                ((rcData[axis as usize] as libc::c_int -
                      (*rxConfig()).midrc as libc::c_int -
                      deadband as libc::c_int) * 100 as libc::c_int /
                     (2000 as libc::c_int - (*rxConfig()).midrc as libc::c_int
                          - deadband as libc::c_int)) as uint8_t
        } else if (rcData[axis as usize] as libc::c_int) <
                      (*rxConfig()).midrc as libc::c_int -
                          deadband as libc::c_int {
            stickPercent =
                (((*rxConfig()).midrc as libc::c_int - deadband as libc::c_int
                      - rcData[axis as usize] as libc::c_int) *
                     100 as libc::c_int /
                     ((*rxConfig()).midrc as libc::c_int -
                          deadband as libc::c_int - 1000 as libc::c_int)) as
                    uint8_t
        }
        if stickPercent as libc::c_int >= stickPercentLimit as libc::c_int {
            return 1 as libc::c_int != 0
        }
        axis += 1
    }
    return 0 as libc::c_int != 0;
}
// allow temporarily disabling runaway takeoff prevention if we are connected
// to the configurator and the ARMING_DISABLED_MSP flag is cleared.
#[no_mangle]
pub unsafe extern "C" fn runawayTakeoffTemporaryDisable(mut disableFlag:
                                                            uint8_t) {
    runawayTakeoffTemporarilyDisabled = disableFlag != 0;
}
// calculate the throttle stick percent - integer math is good enough here.
#[no_mangle]
pub unsafe extern "C" fn calculateThrottlePercent() -> uint8_t {
    let mut ret: uint8_t = 0 as libc::c_int as uint8_t;
    if feature(FEATURE_3D as libc::c_int as uint32_t) as libc::c_int != 0 &&
           !IS_RC_MODE_ACTIVE(BOX3D) &&
           (*flight3DConfig()).switched_mode3d == 0 {
        if rcData[THROTTLE as libc::c_int as usize] as libc::c_int >=
               2000 as libc::c_int ||
               rcData[THROTTLE as libc::c_int as usize] as libc::c_int <=
                   1000 as libc::c_int {
            ret = 100 as libc::c_int as uint8_t
        } else if rcData[THROTTLE as libc::c_int as usize] as libc::c_int >
                      (*rxConfig()).midrc as libc::c_int +
                          (*flight3DConfig()).deadband3d_throttle as
                              libc::c_int {
            ret =
                ((rcData[THROTTLE as libc::c_int as usize] as libc::c_int -
                      (*rxConfig()).midrc as libc::c_int -
                      (*flight3DConfig()).deadband3d_throttle as libc::c_int)
                     * 100 as libc::c_int /
                     (2000 as libc::c_int - (*rxConfig()).midrc as libc::c_int
                          -
                          (*flight3DConfig()).deadband3d_throttle as
                              libc::c_int)) as uint8_t
        } else if (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
                      (*rxConfig()).midrc as libc::c_int -
                          (*flight3DConfig()).deadband3d_throttle as
                              libc::c_int {
            ret =
                (((*rxConfig()).midrc as libc::c_int -
                      (*flight3DConfig()).deadband3d_throttle as libc::c_int -
                      rcData[THROTTLE as libc::c_int as usize] as libc::c_int)
                     * 100 as libc::c_int /
                     ((*rxConfig()).midrc as libc::c_int -
                          (*flight3DConfig()).deadband3d_throttle as
                              libc::c_int - 1000 as libc::c_int)) as uint8_t
        }
    } else {
        ret =
            constrain((rcData[THROTTLE as libc::c_int as usize] as libc::c_int
                           - (*rxConfig()).mincheck as libc::c_int) *
                          100 as libc::c_int /
                          (2000 as libc::c_int -
                               (*rxConfig()).mincheck as libc::c_int),
                      0 as libc::c_int, 100 as libc::c_int) as uint8_t
    }
    return ret;
}
static mut airmodeIsActivated: bool = false;
#[no_mangle]
pub unsafe extern "C" fn isAirmodeActivated() -> bool {
    return airmodeIsActivated;
}
/*
 * processRx called from taskUpdateRxMain
 */
#[no_mangle]
pub unsafe extern "C" fn processRx(mut currentTimeUs: timeUs_t) -> bool {
    static mut armedBeeperOn: bool = 0 as libc::c_int != 0;
    if !calculateRxChannelsAndUpdateFailsafe(currentTimeUs) {
        return 0 as libc::c_int != 0
    }
    // in 3D mode, we need to be able to disarm by switch at any time
    if feature(FEATURE_3D as libc::c_int as uint32_t) {
        if !IS_RC_MODE_ACTIVE(BOXARM) { disarm(); }
    }
    updateRSSI(currentTimeUs);
    if currentTimeUs >
           (1000 as libc::c_int * 1000 as libc::c_int * 5 as libc::c_int) as
               libc::c_uint && !failsafeIsMonitoring() {
        failsafeStartMonitoring();
    }
    failsafeUpdateState();
    let throttleStatus: throttleStatus_e = calculateThrottleStatus();
    let throttlePercent: uint8_t = calculateThrottlePercent();
    if isAirmodeActive() as libc::c_int != 0 &&
           armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        if throttlePercent as libc::c_int >=
               (*rxConfig()).airModeActivateThreshold as libc::c_int {
            airmodeIsActivated = 1 as libc::c_int != 0
            // Prevent Iterm from being reset
        }
    } else { airmodeIsActivated = 0 as libc::c_int != 0 }
    /* In airmode Iterm should be prevented to grow when Low thottle and Roll + Pitch Centered.
     This is needed to prevent Iterm winding on the ground, but keep full stabilisation on 0 throttle while in air */
    if throttleStatus as libc::c_uint ==
           THROTTLE_LOW as libc::c_int as libc::c_uint && !airmodeIsActivated
       {
        pidResetITerm();
        if (*currentPidProfile).pidAtMinThrottle != 0 {
            pidStabilisationState(PID_STABILISATION_ON);
        } else { pidStabilisationState(PID_STABILISATION_OFF); }
    } else { pidStabilisationState(PID_STABILISATION_ON); }
    // If runaway_takeoff_prevention is enabled, accumulate the amount of time that throttle
    // is above runaway_takeoff_deactivate_throttle with the any of the R/P/Y sticks deflected
    // to at least runaway_takeoff_stick_percent percent while the pidSum on all axis is kept low.
    // Once the amount of accumulated time exceeds runaway_takeoff_deactivate_delay then disable
    // prevention for the remainder of the battery.
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 &&
           (*pidConfig()).runaway_takeoff_prevention as libc::c_int != 0 &&
           !runawayTakeoffCheckDisabled && !flipOverAfterCrashMode &&
           !runawayTakeoffTemporarilyDisabled &&
           stateFlags as libc::c_int & FIXED_WING as libc::c_int == 0 {
        // Determine if we're in "flight"
        //   - motors running
        //   - throttle over runaway_takeoff_deactivate_throttle_percent
        //   - sticks are active and have deflection greater than runaway_takeoff_deactivate_stick_percent
        //   - pidSum on all axis is less then runaway_takeoff_deactivate_pidlimit
        let mut inStableFlight: bool = 0 as libc::c_int != 0;
        if !feature(FEATURE_MOTOR_STOP as libc::c_int as uint32_t) ||
               isAirmodeActive() as libc::c_int != 0 ||
               throttleStatus as libc::c_uint !=
                   THROTTLE_LOW as libc::c_int as libc::c_uint {
            // are motors running?
            let lowThrottleLimit: uint8_t =
                (*pidConfig()).runaway_takeoff_deactivate_throttle;
            let midThrottleLimit: uint8_t =
                constrain(lowThrottleLimit as libc::c_int * 2 as libc::c_int,
                          lowThrottleLimit as libc::c_int * 2 as libc::c_int,
                          75 as libc::c_int) as uint8_t;
            if (throttlePercent as libc::c_int >=
                    lowThrottleLimit as libc::c_int &&
                    areSticksActive(15 as libc::c_int as uint8_t) as
                        libc::c_int != 0 ||
                    throttlePercent as libc::c_int >=
                        midThrottleLimit as libc::c_int) &&
                   fabsf(pidData[FD_PITCH as libc::c_int as usize].Sum) <
                       100 as libc::c_int as libc::c_float &&
                   fabsf(pidData[FD_ROLL as libc::c_int as usize].Sum) <
                       100 as libc::c_int as libc::c_float &&
                   fabsf(pidData[FD_YAW as libc::c_int as usize].Sum) <
                       100 as libc::c_int as libc::c_float {
                inStableFlight = 1 as libc::c_int != 0;
                if runawayTakeoffDeactivateUs ==
                       0 as libc::c_int as libc::c_uint {
                    runawayTakeoffDeactivateUs = currentTimeUs
                }
            }
        }
        // If we're in flight, then accumulate the time and deactivate once it exceeds runaway_takeoff_deactivate_delay milliseconds
        if inStableFlight {
            if runawayTakeoffDeactivateUs == 0 as libc::c_int as libc::c_uint
               {
                runawayTakeoffDeactivateUs = currentTimeUs
            }
            let mut deactivateDelay: uint16_t =
                (*pidConfig()).runaway_takeoff_deactivate_delay;
            // at high throttle levels reduce deactivation delay by 50%
            if throttlePercent as libc::c_int >= 75 as libc::c_int {
                deactivateDelay =
                    (deactivateDelay as libc::c_int / 2 as libc::c_int) as
                        uint16_t
            }
            if (cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs) as
                    libc::c_uint).wrapping_add(runawayTakeoffAccumulatedUs) >
                   (deactivateDelay as libc::c_int * 1000 as libc::c_int) as
                       libc::c_uint {
                runawayTakeoffCheckDisabled = 1 as libc::c_int != 0
            }
        } else {
            if runawayTakeoffDeactivateUs != 0 as libc::c_int as libc::c_uint
               {
                runawayTakeoffAccumulatedUs =
                    (runawayTakeoffAccumulatedUs as
                         libc::c_uint).wrapping_add(cmpTimeUs(currentTimeUs,
                                                              runawayTakeoffDeactivateUs)
                                                        as libc::c_uint) as
                        timeUs_t as timeUs_t
            }
            runawayTakeoffDeactivateUs = 0 as libc::c_int as timeUs_t
        }
        if runawayTakeoffDeactivateUs == 0 as libc::c_int as libc::c_uint {
            if debugMode as libc::c_int ==
                   DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
                debug[2 as libc::c_int as usize] = 0 as libc::c_int as int16_t
            }
            if debugMode as libc::c_int ==
                   DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
                debug[3 as libc::c_int as usize] =
                    runawayTakeoffAccumulatedUs.wrapping_div(1000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint)
                        as int16_t
            }
        } else {
            if debugMode as libc::c_int ==
                   DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
                debug[2 as libc::c_int as usize] = 1 as libc::c_int as int16_t
            }
            if debugMode as libc::c_int ==
                   DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
                debug[3 as libc::c_int as usize] =
                    (cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs) as
                         libc::c_uint).wrapping_add(runawayTakeoffAccumulatedUs).wrapping_div(1000
                                                                                                  as
                                                                                                  libc::c_int
                                                                                                  as
                                                                                                  libc::c_uint)
                        as int16_t
            }
        }
    } else {
        if debugMode as libc::c_int == DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
            debug[2 as libc::c_int as usize] = 0 as libc::c_int as int16_t
        }
        if debugMode as libc::c_int == DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
            debug[3 as libc::c_int as usize] = 0 as libc::c_int as int16_t
        }
    }
    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 &&
           feature(FEATURE_MOTOR_STOP as libc::c_int as uint32_t) as
               libc::c_int != 0 &&
           stateFlags as libc::c_int & FIXED_WING as libc::c_int == 0 &&
           !feature(FEATURE_3D as libc::c_int as uint32_t) &&
           !isAirmodeActive() {
        if isUsingSticksForArming() {
            if throttleStatus as libc::c_uint ==
                   THROTTLE_LOW as libc::c_int as libc::c_uint {
                if (*armingConfig()).auto_disarm_delay as libc::c_int !=
                       0 as libc::c_int &&
                       (disarmAt.wrapping_sub(millis()) as int32_t) <
                           0 as libc::c_int {
                    // auto-disarm configured and delay is over
                    disarm();
                    armedBeeperOn = 0 as libc::c_int != 0
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = 1 as libc::c_int != 0
                }
            } else {
                // throttle is not low
                if (*armingConfig()).auto_disarm_delay as libc::c_int !=
                       0 as libc::c_int {
                    // extend disarm time
                    disarmAt =
                        millis().wrapping_add(((*armingConfig()).auto_disarm_delay
                                                   as libc::c_int *
                                                   1000 as libc::c_int) as
                                                  libc::c_uint)
                }
                if armedBeeperOn {
                    beeperSilence();
                    armedBeeperOn = 0 as libc::c_int != 0
                }
            }
        } else if throttleStatus as libc::c_uint ==
                      THROTTLE_LOW as libc::c_int as libc::c_uint {
            beeper(BEEPER_ARMED);
            armedBeeperOn = 1 as libc::c_int != 0
        } else if armedBeeperOn {
            beeperSilence();
            armedBeeperOn = 0 as libc::c_int != 0
        }
    }
    processRcStickPositions();
    if feature(FEATURE_INFLIGHT_ACC_CAL as libc::c_int as uint32_t) {
        updateInflightCalibrationState();
    }
    updateActivatedModes();
    // arming is via AUX switch; beep while throttle low
    /* Enable beep warning when the crash flip mode is active */
    if isMotorProtocolDshot() as libc::c_int != 0 &&
           isModeActivationConditionPresent(BOXFLIPOVERAFTERCRASH) as
               libc::c_int != 0 &&
           IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH) as libc::c_int != 0 {
        beeper(BEEPER_CRASH_FLIP_MODE);
    }
    if cliMode == 0 {
        updateAdjustmentStates();
        processRcAdjustments(currentControlRateProfile);
    }
    let mut canUseHorizonMode: bool = 1 as libc::c_int != 0;
    if (IS_RC_MODE_ACTIVE(BOXANGLE) as libc::c_int != 0 ||
            failsafeIsActive() as libc::c_int != 0) &&
           sensors(SENSOR_ACC as libc::c_int as uint32_t) as libc::c_int != 0
       {
        // bumpless transfer to Level mode
        canUseHorizonMode = 0 as libc::c_int != 0;
        if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int == 0 {
            enableFlightMode(ANGLE_MODE);
        }
    } else {
        disableFlightMode(ANGLE_MODE);
        // failsafe support
    }
    if IS_RC_MODE_ACTIVE(BOXHORIZON) as libc::c_int != 0 &&
           canUseHorizonMode as libc::c_int != 0 {
        disableFlightMode(ANGLE_MODE);
        if flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int == 0 {
            enableFlightMode(HORIZON_MODE);
        }
    } else { disableFlightMode(HORIZON_MODE); }
    if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int != 0 ||
           flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int != 0 {
        ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
        // increase frequency of attitude task to reduce drift when in angle or horizon mode
        rescheduleTask(TASK_ATTITUDE,
                       (1000000 as libc::c_int / 500 as libc::c_int) as
                           uint32_t);
    } else {
        ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
        rescheduleTask(TASK_ATTITUDE,
                       (1000000 as libc::c_int / 100 as libc::c_int) as
                           uint32_t);
    }
    if !IS_RC_MODE_ACTIVE(BOXPREARM) &&
           armingFlags as libc::c_int & WAS_ARMED_WITH_PREARM as libc::c_int
               != 0 {
        armingFlags =
            (armingFlags as libc::c_int &
                 !(WAS_ARMED_WITH_PREARM as libc::c_int)) as uint8_t
    }
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) as libc::c_int != 0 ||
           sensors(SENSOR_MAG as libc::c_int as uint32_t) as libc::c_int != 0
       {
        if IS_RC_MODE_ACTIVE(BOXHEADFREE) {
            if flightModeFlags as libc::c_int & HEADFREE_MODE as libc::c_int
                   == 0 {
                enableFlightMode(HEADFREE_MODE);
            }
        } else { disableFlightMode(HEADFREE_MODE); }
        if IS_RC_MODE_ACTIVE(BOXHEADADJ) {
            if imuQuaternionHeadfreeOffsetSet() { beeper(BEEPER_RX_SET); }
        }
    }
    if IS_RC_MODE_ACTIVE(BOXPASSTHRU) {
        enableFlightMode(PASSTHRU_MODE);
    } else { disableFlightMode(PASSTHRU_MODE); }
    if (*mixerConfig()).mixerMode as libc::c_int ==
           MIXER_FLYING_WING as libc::c_int ||
           (*mixerConfig()).mixerMode as libc::c_int ==
               MIXER_AIRPLANE as libc::c_int {
        disableFlightMode(HEADFREE_MODE);
    }
    static mut sharedPortTelemetryEnabled: bool = 0 as libc::c_int != 0;
    if feature(FEATURE_TELEMETRY as libc::c_int as uint32_t) {
        let mut enableSharedPortTelemetry: bool =
            !isModeActivationConditionPresent(BOXTELEMETRY) &&
                armingFlags as libc::c_int & ARMED as libc::c_int != 0 ||
                isModeActivationConditionPresent(BOXTELEMETRY) as libc::c_int
                    != 0 &&
                    IS_RC_MODE_ACTIVE(BOXTELEMETRY) as libc::c_int != 0;
        if enableSharedPortTelemetry as libc::c_int != 0 &&
               !sharedPortTelemetryEnabled {
            mspSerialReleaseSharedTelemetryPorts();
            telemetryCheckState();
            sharedPortTelemetryEnabled = 1 as libc::c_int != 0
        } else if !enableSharedPortTelemetry &&
                      sharedPortTelemetryEnabled as libc::c_int != 0 {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();
            sharedPortTelemetryEnabled = 0 as libc::c_int != 0
        }
    }
    vtxUpdateActivatedChannel();
    if canUpdateVTX() { handleVTXControlButton(); }
    pidSetAcroTrainerState(IS_RC_MODE_ACTIVE(BOXACROTRAINER) as libc::c_int !=
                               0 &&
                               sensors(SENSOR_ACC as libc::c_int as uint32_t)
                                   as libc::c_int != 0);
    // USE_ACRO_TRAINER
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 &&
           !rcSmoothingInitializationComplete() {
        beeper(BEEPER_RC_SMOOTHING_INIT_FAIL);
    }
    pidSetAntiGravityState(IS_RC_MODE_ACTIVE(BOXANTIGRAVITY) as libc::c_int !=
                               0 ||
                               feature(FEATURE_ANTI_GRAVITY as libc::c_int as
                                           uint32_t) as libc::c_int != 0);
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn subTaskPidController(mut currentTimeUs: timeUs_t) {
    let mut startTime: uint32_t = 0 as libc::c_int as uint32_t;
    if debugMode as libc::c_int == DEBUG_PIDLOOP as libc::c_int {
        startTime = micros()
    }
    // PID - note this is function pointer set by setPIDController()
    pidController(currentPidProfile,
                  &(*(accelerometerConfig as
                          unsafe extern "C" fn()
                              ->
                                  *const accelerometerConfig_t)()).accelerometerTrims,
                  currentTimeUs);
    if debugMode as libc::c_int == DEBUG_PIDLOOP as libc::c_int {
        debug[1 as libc::c_int as usize] =
            micros().wrapping_sub(startTime) as int16_t
    }
    // Check to see if runaway takeoff detection is active (anti-taz), the pidSum is over the threshold,
    // and gyro rate for any axis is above the limit for at least the activate delay period.
    // If so, disarm for safety
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 &&
           stateFlags as libc::c_int & FIXED_WING as libc::c_int == 0 &&
           (*pidConfig()).runaway_takeoff_prevention as libc::c_int != 0 &&
           !runawayTakeoffCheckDisabled && !flipOverAfterCrashMode &&
           !runawayTakeoffTemporarilyDisabled &&
           (!feature(FEATURE_MOTOR_STOP as libc::c_int as uint32_t) ||
                isAirmodeActive() as libc::c_int != 0 ||
                calculateThrottleStatus() as libc::c_uint !=
                    THROTTLE_LOW as libc::c_int as libc::c_uint) {
        if (fabsf(pidData[FD_PITCH as libc::c_int as usize].Sum) >=
                600 as libc::c_int as libc::c_float ||
                fabsf(pidData[FD_ROLL as libc::c_int as usize].Sum) >=
                    600 as libc::c_int as libc::c_float ||
                fabsf(pidData[FD_YAW as libc::c_int as usize].Sum) >=
                    600 as libc::c_int as libc::c_float) &&
               (({
                     let mut _x: uint16_t =
                         gyroAbsRateDps(FD_PITCH as libc::c_int);
                     (if _x as libc::c_int > 0 as libc::c_int {
                          _x as libc::c_int
                      } else { -(_x as libc::c_int) })
                 }) > 15 as libc::c_int ||
                    ({
                         let mut _x: uint16_t =
                             gyroAbsRateDps(FD_ROLL as libc::c_int);
                         (if _x as libc::c_int > 0 as libc::c_int {
                              _x as libc::c_int
                          } else { -(_x as libc::c_int) })
                     }) > 15 as libc::c_int ||
                    ({
                         let mut _x: uint16_t =
                             gyroAbsRateDps(FD_YAW as libc::c_int);
                         (if _x as libc::c_int > 0 as libc::c_int {
                              _x as libc::c_int
                          } else { -(_x as libc::c_int) })
                     }) > 50 as libc::c_int) {
            if runawayTakeoffTriggerUs == 0 as libc::c_int as libc::c_uint {
                runawayTakeoffTriggerUs =
                    currentTimeUs.wrapping_add(75000 as libc::c_int as
                                                   libc::c_uint)
            } else if currentTimeUs > runawayTakeoffTriggerUs {
                setArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
                disarm();
            }
        } else { runawayTakeoffTriggerUs = 0 as libc::c_int as timeUs_t }
        if debugMode as libc::c_int == DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
            debug[0 as libc::c_int as usize] = 1 as libc::c_int as int16_t
        }
        if debugMode as libc::c_int == DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
            debug[1 as libc::c_int as usize] =
                if runawayTakeoffTriggerUs == 0 as libc::c_int as libc::c_uint
                   {
                    0 as libc::c_int
                } else { 1 as libc::c_int } as int16_t
        }
    } else {
        runawayTakeoffTriggerUs = 0 as libc::c_int as timeUs_t;
        if debugMode as libc::c_int == DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
            debug[0 as libc::c_int as usize] = 0 as libc::c_int as int16_t
        }
        if debugMode as libc::c_int == DEBUG_RUNAWAY_TAKEOFF as libc::c_int {
            debug[1 as libc::c_int as usize] = 0 as libc::c_int as int16_t
        }
    };
}
unsafe extern "C" fn subTaskPidSubprocesses(mut currentTimeUs: timeUs_t) {
    let mut startTime: uint32_t = 0 as libc::c_int as uint32_t;
    if debugMode as libc::c_int == DEBUG_PIDLOOP as libc::c_int {
        startTime = micros()
    }
    if cliMode == 0 && (*blackboxConfig()).device as libc::c_int != 0 {
        blackboxUpdate(currentTimeUs);
    }
    if debugMode as libc::c_int == DEBUG_PIDLOOP as libc::c_int {
        debug[3 as libc::c_int as usize] =
            micros().wrapping_sub(startTime) as int16_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn subTaskTelemetryPollSensors(mut currentTimeUs:
                                                         timeUs_t) {
    // Read out gyro temperature if used for telemmetry
    gyroReadTemperature();
}
unsafe extern "C" fn subTaskMotorUpdate(mut currentTimeUs: timeUs_t) {
    let mut startTime: uint32_t = 0 as libc::c_int as uint32_t;
    if debugMode as libc::c_int == DEBUG_CYCLETIME as libc::c_int {
        startTime = micros();
        static mut previousMotorUpdateTime: uint32_t = 0;
        let currentDeltaTime: uint32_t =
            startTime.wrapping_sub(previousMotorUpdateTime);
        debug[2 as libc::c_int as usize] = currentDeltaTime as int16_t;
        debug[3 as libc::c_int as usize] =
            currentDeltaTime.wrapping_sub(targetPidLooptime) as int16_t;
        previousMotorUpdateTime = startTime
    } else if debugMode as libc::c_int == DEBUG_PIDLOOP as libc::c_int {
        startTime = micros()
    }
    mixTable(currentTimeUs, (*currentPidProfile).vbatPidCompensation);
    // motor outputs are used as sources for servo mixing, so motors must be calculated using mixTable() before servos.
    if isMixerUsingServos() { writeServos(); }
    writeMotors();
    if debugMode as libc::c_int == DEBUG_PIDLOOP as libc::c_int {
        debug[2 as libc::c_int as usize] =
            micros().wrapping_sub(startTime) as int16_t
    };
}
unsafe extern "C" fn subTaskRcCommand(mut currentTimeUs: timeUs_t) {
    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if isUsingSticksForArming() as libc::c_int != 0 &&
           rcData[THROTTLE as libc::c_int as usize] as libc::c_int <=
               (*rxConfig()).mincheck as libc::c_int &&
           !(((*mixerConfig()).mixerMode as libc::c_int ==
                  MIXER_TRI as libc::c_int ||
                  (*mixerConfig()).mixerMode as libc::c_int ==
                      MIXER_CUSTOM_TRI as libc::c_int) &&
                 (*servoConfig()).tri_unarmed_servo as libc::c_int != 0) &&
           (*mixerConfig()).mixerMode as libc::c_int !=
               MIXER_AIRPLANE as libc::c_int &&
           (*mixerConfig()).mixerMode as libc::c_int !=
               MIXER_FLYING_WING as libc::c_int {
        resetYawAxis();
    }
    processRcCommand();
}
// Function for loop trigger
#[no_mangle]
pub unsafe extern "C" fn taskMainPidLoop(mut currentTimeUs: timeUs_t) {
    static mut pidUpdateCounter: uint32_t = 0 as libc::c_int as uint32_t;
    // DEBUG_PIDLOOP, timings for:
    // 0 - gyroUpdate()
    // 1 - subTaskPidController()
    // 2 - subTaskMotorUpdate()
    // 3 - subTaskPidSubprocesses()
    gyroUpdate(currentTimeUs);
    if debugMode as libc::c_int == DEBUG_PIDLOOP as libc::c_int {
        debug[0 as libc::c_int as usize] =
            micros().wrapping_sub(currentTimeUs) as int16_t
    }
    let fresh2 = pidUpdateCounter;
    pidUpdateCounter = pidUpdateCounter.wrapping_add(1);
    if fresh2.wrapping_rem((*pidConfig()).pid_process_denom as libc::c_uint)
           == 0 as libc::c_int as libc::c_uint {
        subTaskRcCommand(currentTimeUs);
        subTaskPidController(currentTimeUs);
        subTaskMotorUpdate(currentTimeUs);
        subTaskPidSubprocesses(currentTimeUs);
    }
    if debugMode as libc::c_int == DEBUG_CYCLETIME as libc::c_int {
        debug[0 as libc::c_int as usize] =
            getTaskDeltaTime(TASK_SELF) as int16_t;
        debug[1 as libc::c_int as usize] = averageSystemLoadPercent as int16_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn isFlipOverAfterCrashMode() -> bool {
    return flipOverAfterCrashMode;
}
#[no_mangle]
pub unsafe extern "C" fn getLastDisarmTimeUs() -> timeUs_t {
    return lastDisarmTimeUs;
}
#[no_mangle]
pub unsafe extern "C" fn isTryingToArm() -> bool {
    return tryingToArm != ARMING_DELAYED_DISARMED as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn resetTryingToArm() {
    tryingToArm = ARMING_DELAYED_DISARMED as libc::c_int;
}
