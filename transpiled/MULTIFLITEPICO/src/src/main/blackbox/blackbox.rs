use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
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
    // increment when a major release is made (big new feature, etc)
    // increment when a minor release is made (small new feature, change etc)
    // increment when a bug is fixed
    // lower case hexadecimal digits.
    // "MMM DD YYYY" MMM = Jan/Feb/...
    #[no_mangle]
    static buildTime: *const libc::c_char;
    #[no_mangle]
    static buildDate: *const libc::c_char;
    #[no_mangle]
    static shortGitRevision: *const libc::c_char;
    #[no_mangle]
    static targetName: *const libc::c_char;
    #[no_mangle]
    fn dateTimeFormatLocal(buf: *mut libc::c_char, dt: *mut dateTime_t)
     -> bool;
    #[no_mangle]
    fn rtcGetDateTime(dt: *mut dateTime_t) -> bool;
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
    fn blackboxPrintf(fmt: *const libc::c_char, _: ...) -> libc::c_int;
    #[no_mangle]
    fn blackboxPrintfHeaderLine(name: *const libc::c_char,
                                fmt: *const libc::c_char, _: ...);
    #[no_mangle]
    fn blackboxWriteUnsignedVB(value: uint32_t);
    #[no_mangle]
    fn blackboxWriteSignedVB(value: int32_t);
    #[no_mangle]
    fn blackboxWriteSignedVBArray(array: *mut int32_t, count: libc::c_int);
    #[no_mangle]
    fn blackboxWriteSigned16VBArray(array: *mut int16_t, count: libc::c_int);
    #[no_mangle]
    fn blackboxWriteTag2_3S32(values: *mut int32_t);
    #[no_mangle]
    fn blackboxWriteTag8_4S16(values: *mut int32_t);
    #[no_mangle]
    fn blackboxWriteTag8_8SVB(values: *mut int32_t, valueCount: libc::c_int);
    #[no_mangle]
    fn blackboxWriteFloat(value: libc::c_float);
    #[no_mangle]
    static mut blackboxHeaderBudget: int32_t;
    #[no_mangle]
    fn blackboxOpen();
    #[no_mangle]
    fn blackboxWrite(value: uint8_t);
    #[no_mangle]
    fn blackboxWriteString(s: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn blackboxDeviceFlush();
    #[no_mangle]
    fn blackboxDeviceFlushForce() -> bool;
    #[no_mangle]
    fn blackboxDeviceOpen() -> bool;
    #[no_mangle]
    fn blackboxDeviceClose();
    #[no_mangle]
    fn blackboxDeviceBeginLog() -> bool;
    #[no_mangle]
    fn blackboxDeviceEndLog(retainLog: bool) -> bool;
    #[no_mangle]
    fn isBlackboxDeviceFull() -> bool;
    #[no_mangle]
    fn blackboxReplenishHeaderBudget();
    #[no_mangle]
    fn blackboxDeviceReserveBufferSpace(bytes: int32_t)
     -> blackboxBufferReserveStatus_e;
    #[no_mangle]
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
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
    fn castFloatBytesToInt(f: libc::c_float) -> uint32_t;
    #[no_mangle]
    fn arraySubInt32(dest: *mut int32_t, array1: *mut int32_t,
                     array2: *mut int32_t, count: libc::c_int);
    #[no_mangle]
    static mut featureConfig_System: featureConfig_t;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
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
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut pilotConfig_System: pilotConfig_t;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    static mut controlRateProfiles_SystemArray: [controlRateConfig_t; 6];
    #[no_mangle]
    fn rcSmoothingGetValue(whichValue: libc::c_int) -> libc::c_int;
    // (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    #[no_mangle]
    static mut rcControlsConfig_System: rcControlsConfig_t;
    #[no_mangle]
    static mut armingConfig_System: armingConfig_t;
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    fn isModeActivationConditionPresent(modeId: boxId_e) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn failsafePhase() -> failsafePhase_e;
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut motor: [libc::c_float; 8];
    #[no_mangle]
    static mut motorOutputHigh: libc::c_float;
    #[no_mangle]
    static mut motorOutputLow: libc::c_float;
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    fn areMotorsRunning() -> bool;
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
    #[no_mangle]
    static mut pidData: [pidAxisData_t; 3];
    #[no_mangle]
    static mut targetPidLooptime: uint32_t;
    #[no_mangle]
    static mut servo: [int16_t; 8];
    #[no_mangle]
    fn getArmingBeepTimeMicros() -> uint32_t;
    #[no_mangle]
    fn findSharedSerialPort(functionMask: uint16_t,
                            sharedWithFunction: serialPortFunction_e)
     -> *mut serialPort_t;
    #[no_mangle]
    fn rxIsReceivingSignal() -> bool;
    #[no_mangle]
    fn rxAreFlightChannelsValid() -> bool;
    #[no_mangle]
    fn getRssi() -> uint16_t;
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    static mut gyro: gyro_t;
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
    #[no_mangle]
    static mut accelerometerConfig_System: accelerometerConfig_t;
    #[no_mangle]
    static mut barometerConfig_System: barometerConfig_t;
    #[no_mangle]
    static mut baro: baro_t;
    #[no_mangle]
    static mut voltageSensorADCConfig_SystemArray:
           [voltageSensorADCConfig_t; 1];
    // offset of the current sensor in mA
    #[no_mangle]
    static mut currentSensorADCConfig_System: currentSensorADCConfig_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn getAmperageLatest() -> int32_t;
    #[no_mangle]
    fn getBatteryVoltageLatest() -> uint16_t;
    #[no_mangle]
    static mut mag: mag_t;
    #[no_mangle]
    static mut compassConfig_System: compassConfig_t;
    #[no_mangle]
    fn rangefinderGetLatestAltitude() -> int32_t;
    // We pack this struct so that padding doesn't interfere with memcmp()
    //From rc_controls.c
    #[no_mangle]
    static mut rcModeActivationMask: boxBitmask_t;
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
pub type size_t = libc::c_ulong;
/* *
  * @brief Serial Peripheral Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
}
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
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
// parameter group registry flags
// documentary
// function that resets a single parameter group instance
/* base */
/* size */
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
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _dateTime_s {
    pub year: uint16_t,
    pub month: uint8_t,
    pub day: uint8_t,
    pub hours: uint8_t,
    pub minutes: uint8_t,
    pub seconds: uint8_t,
    pub millis: uint16_t,
}
pub type dateTime_t = _dateTime_s;
pub type BlackboxDevice = libc::c_uint;
pub const BLACKBOX_DEVICE_SERIAL: BlackboxDevice = 3;
pub const BLACKBOX_DEVICE_NONE: BlackboxDevice = 0;
pub type BlackboxMode = libc::c_uint;
pub const BLACKBOX_MODE_ALWAYS_ON: BlackboxMode = 2;
pub const BLACKBOX_MODE_MOTOR_TEST: BlackboxMode = 1;
pub const BLACKBOX_MODE_NORMAL: BlackboxMode = 0;
pub type FlightLogEvent = libc::c_uint;
pub const FLIGHT_LOG_EVENT_LOG_END: FlightLogEvent = 255;
pub const FLIGHT_LOG_EVENT_FLIGHTMODE: FlightLogEvent = 30;
pub const FLIGHT_LOG_EVENT_LOGGING_RESUME: FlightLogEvent = 14;
pub const FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT: FlightLogEvent = 13;
pub const FLIGHT_LOG_EVENT_SYNC_BEEP: FlightLogEvent = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union flightLogEventData_u {
    pub syncBeep: flightLogEvent_syncBeep_t,
    pub flightMode: flightLogEvent_flightMode_t,
    pub inflightAdjustment: flightLogEvent_inflightAdjustment_t,
    pub loggingResume: flightLogEvent_loggingResume_t,
}
pub type flightLogEvent_loggingResume_t = flightLogEvent_loggingResume_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flightLogEvent_loggingResume_s {
    pub logIteration: uint32_t,
    pub currentTime: uint32_t,
}
pub type flightLogEvent_inflightAdjustment_t
    =
    flightLogEvent_inflightAdjustment_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flightLogEvent_inflightAdjustment_s {
    pub newValue: int32_t,
    pub newFloatValue: libc::c_float,
    pub adjustmentFunction: uint8_t,
    pub floatFlag: bool,
}
pub type flightLogEvent_flightMode_t = flightLogEvent_flightMode_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flightLogEvent_flightMode_s {
    pub flags: uint32_t,
    pub lastFlags: uint32_t,
}
pub type flightLogEvent_syncBeep_t = flightLogEvent_syncBeep_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flightLogEvent_syncBeep_s {
    pub time: uint32_t,
}
pub type flightLogEventData_t = flightLogEventData_u;
pub const BLACKBOX_STATE_PAUSED: BlackboxState = 9;
pub type BlackboxState = libc::c_uint;
pub const BLACKBOX_STATE_ERASED: BlackboxState = 14;
pub const BLACKBOX_STATE_ERASING: BlackboxState = 13;
pub const BLACKBOX_STATE_START_ERASE: BlackboxState = 12;
pub const BLACKBOX_STATE_SHUTTING_DOWN: BlackboxState = 11;
pub const BLACKBOX_STATE_RUNNING: BlackboxState = 10;
pub const BLACKBOX_STATE_SEND_SYSINFO: BlackboxState = 8;
pub const BLACKBOX_STATE_SEND_SLOW_HEADER: BlackboxState = 7;
pub const BLACKBOX_STATE_SEND_GPS_G_HEADER: BlackboxState = 6;
pub const BLACKBOX_STATE_SEND_GPS_H_HEADER: BlackboxState = 5;
pub const BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER: BlackboxState = 4;
pub const BLACKBOX_STATE_SEND_HEADER: BlackboxState = 3;
pub const BLACKBOX_STATE_PREPARE_LOG_FILE: BlackboxState = 2;
pub const BLACKBOX_STATE_STOPPED: BlackboxState = 1;
pub const BLACKBOX_STATE_DISABLED: BlackboxState = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
    pub fieldIndex: libc::c_int,
    pub startTime: uint32_t,
}
// I-frame interval / P-frame interval
// New Event Data type
// New event data
// New event tracking of flight modes
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_2 {
    pub headerIndex: uint32_t,
    pub u: C2RustUnnamed_1,
}
// type to hold enough bits for CHECKBOX_ITEM_COUNT. Struct used for value-like behavior
pub type boxBitmask_t = boxBitmask_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct boxBitmask_s {
    pub bits: [uint32_t; 2],
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
pub type FlightLogFieldCondition = libc::c_uint;
pub const FLIGHT_LOG_FIELD_CONDITION_LAST: FlightLogFieldCondition = 22;
pub const FLIGHT_LOG_FIELD_CONDITION_FIRST: FlightLogFieldCondition = 0;
pub const FLIGHT_LOG_FIELD_CONDITION_NEVER: FlightLogFieldCondition = 22;
pub const FLIGHT_LOG_FIELD_CONDITION_DEBUG: FlightLogFieldCondition = 21;
pub const FLIGHT_LOG_FIELD_CONDITION_ACC: FlightLogFieldCondition = 20;
pub const FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
          FlightLogFieldCondition =
    19;
pub const FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2: FlightLogFieldCondition
          =
    18;
pub const FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1: FlightLogFieldCondition
          =
    17;
pub const FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0: FlightLogFieldCondition
          =
    16;
pub const FLIGHT_LOG_FIELD_CONDITION_RSSI: FlightLogFieldCondition = 15;
pub const FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER: FlightLogFieldCondition =
    14;
pub const FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC: FlightLogFieldCondition =
    13;
pub const FLIGHT_LOG_FIELD_CONDITION_VBAT: FlightLogFieldCondition = 12;
pub const FLIGHT_LOG_FIELD_CONDITION_BARO: FlightLogFieldCondition = 11;
pub const FLIGHT_LOG_FIELD_CONDITION_MAG: FlightLogFieldCondition = 10;
pub const FLIGHT_LOG_FIELD_CONDITION_TRICOPTER: FlightLogFieldCondition = 9;
pub const FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8:
          FlightLogFieldCondition =
    8;
pub const FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7:
          FlightLogFieldCondition =
    7;
pub const FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6:
          FlightLogFieldCondition =
    6;
pub const FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5:
          FlightLogFieldCondition =
    5;
pub const FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4:
          FlightLogFieldCondition =
    4;
pub const FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3:
          FlightLogFieldCondition =
    3;
pub const FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2:
          FlightLogFieldCondition =
    2;
pub const FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1:
          FlightLogFieldCondition =
    1;
pub const FLIGHT_LOG_FIELD_CONDITION_ALWAYS: FlightLogFieldCondition = 0;
pub const DEBUG_NONE: C2RustUnnamed_4 = 0;
pub const SENSOR_ACC: C2RustUnnamed_10 = 2;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_5 = 32768;
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
pub const SENSOR_RANGEFINDER: C2RustUnnamed_10 = 16;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
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
pub const SENSOR_BARO: C2RustUnnamed_10 = 4;
pub const SENSOR_MAG: C2RustUnnamed_10 = 8;
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
pub const MIXER_CUSTOM_TRI: mixerMode = 25;
pub type mixerConfig_t = mixerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
pub const MIXER_TRI: mixerMode = 1;
pub type blackboxMainState_t = blackboxMainState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxMainState_s {
    pub time: uint32_t,
    pub axisPID_P: [int32_t; 3],
    pub axisPID_I: [int32_t; 3],
    pub axisPID_D: [int32_t; 3],
    pub axisPID_F: [int32_t; 3],
    pub rcCommand: [int16_t; 4],
    pub gyroADC: [int16_t; 3],
    pub accADC: [int16_t; 3],
    pub debug: [int16_t; 4],
    pub motor: [int16_t; 8],
    pub servo: [int16_t; 8],
    pub vbatLatest: uint16_t,
    pub amperageLatest: int32_t,
    pub BaroAlt: int32_t,
    pub magADC: [int16_t; 3],
    pub surfaceRaw: int32_t,
    pub rssi: uint16_t,
}
pub type blackboxGpsState_t = blackboxGpsState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxGpsState_s {
    pub GPS_home: [int32_t; 2],
    pub GPS_coord: [int32_t; 2],
    pub GPS_numSat: uint8_t,
}
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
pub type serialPortFunction_e = libc::c_uint;
pub const FUNCTION_LIDAR_TF: serialPortFunction_e = 32768;
pub const FUNCTION_RCDEVICE: serialPortFunction_e = 16384;
pub const FUNCTION_VTX_TRAMP: serialPortFunction_e = 8192;
pub const FUNCTION_TELEMETRY_IBUS: serialPortFunction_e = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: serialPortFunction_e = 2048;
pub const FUNCTION_ESC_SENSOR: serialPortFunction_e = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: serialPortFunction_e = 512;
pub const FUNCTION_BLACKBOX: serialPortFunction_e = 128;
pub const FUNCTION_RX_SERIAL: serialPortFunction_e = 64;
pub const FUNCTION_TELEMETRY_SMARTPORT: serialPortFunction_e = 32;
pub const FUNCTION_TELEMETRY_LTM: serialPortFunction_e = 16;
pub const FUNCTION_TELEMETRY_HOTT: serialPortFunction_e = 8;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: serialPortFunction_e = 4;
pub const FUNCTION_GPS: serialPortFunction_e = 2;
pub const FUNCTION_MSP: serialPortFunction_e = 1;
pub const FUNCTION_NONE: serialPortFunction_e = 0;
pub const ARMED: C2RustUnnamed_7 = 1;
pub type baro_t = baro_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baro_s {
    pub dev: baroDev_t,
    pub BaroAlt: int32_t,
    pub baroTemperature: int32_t,
    pub baroPressure: int32_t,
}
// Use temperature for telemetry
// Use pressure for telemetry
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
// XXX
// baro start operation
// baro calculation (filled params are pressure and temperature)
pub type baroDev_t = baroDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baroDev_s {
    pub busdev: busDevice_t,
    pub ut_delay: uint16_t,
    pub up_delay: uint16_t,
    pub start_ut: baroOpFuncPtr,
    pub get_ut: baroOpFuncPtr,
    pub start_up: baroOpFuncPtr,
    pub get_up: baroOpFuncPtr,
    pub calculate: baroCalculateFuncPtr,
}
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
pub type busDevice_t = busDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_3 {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
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
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
pub type mag_t = mag_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mag_s {
    pub magADC: [libc::c_float; 3],
    pub magneticDeclination: libc::c_float,
}
pub type acc_t = acc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
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
// initialize function
// read 3 axis data function
// read temperature if available
// scalefactor
// gyro data after calibration and alignment
pub type accDev_t = accDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accDev_s {
    pub initFn: sensorAccInitFuncPtr,
    pub readFn: sensorAccReadFuncPtr,
    pub bus: busDevice_t,
    pub acc_1G: uint16_t,
    pub ADCRaw: [int16_t; 3],
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub accAlign: sensor_align_e,
    pub dataReady: bool,
    pub acc_high_fsr: bool,
    pub revisionCode: libc::c_char,
    pub filler: [uint8_t; 2],
}
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
pub type mpuDetectionResult_t = mpuDetectionResult_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
}
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
pub type mpuSensor_e = libc::c_uint;
pub const BMI_160_SPI: mpuSensor_e = 12;
pub const ICM_20689_SPI: mpuSensor_e = 11;
pub const ICM_20649_SPI: mpuSensor_e = 10;
pub const ICM_20608_SPI: mpuSensor_e = 9;
pub const ICM_20602_SPI: mpuSensor_e = 8;
pub const ICM_20601_SPI: mpuSensor_e = 7;
pub const MPU_9250_SPI: mpuSensor_e = 6;
pub const MPU_65xx_SPI: mpuSensor_e = 5;
pub const MPU_65xx_I2C: mpuSensor_e = 4;
pub const MPU_60x0_SPI: mpuSensor_e = 3;
pub const MPU_60x0: mpuSensor_e = 2;
pub const MPU_3050: mpuSensor_e = 1;
pub const MPU_NONE: mpuSensor_e = 0;
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
pub type gyro_t = gyro_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
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
// initialize function
// read 3 axis data function
// a revision code for the sensor, if known
// This data is updated really infrequently:
pub type blackboxSlowState_t = blackboxSlowState_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct blackboxSlowState_s {
    pub flightModeFlags: uint32_t,
    pub stateFlags: uint8_t,
    pub failsafePhase: uint8_t,
    pub rxSignalReceived: bool,
    pub rxFlightChannelsValid: bool,
}
pub type failsafePhase_e = libc::c_uint;
pub const FAILSAFE_GPS_RESCUE: failsafePhase_e = 6;
pub const FAILSAFE_RX_LOSS_RECOVERED: failsafePhase_e = 5;
pub const FAILSAFE_RX_LOSS_MONITORING: failsafePhase_e = 4;
pub const FAILSAFE_LANDED: failsafePhase_e = 3;
pub const FAILSAFE_LANDING: failsafePhase_e = 2;
pub const FAILSAFE_RX_LOSS_DETECTED: failsafePhase_e = 1;
pub const FAILSAFE_IDLE: failsafePhase_e = 0;
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
pub const THROTTLE: rc_alias = 3;
pub const RC_SMOOTHING_VALUE_AVERAGE_FRAME: C2RustUnnamed_6 = 2;
pub const RC_SMOOTHING_VALUE_DERIVATIVE_ACTIVE: C2RustUnnamed_6 = 1;
pub const RC_SMOOTHING_VALUE_INPUT_ACTIVE: C2RustUnnamed_6 = 0;
pub type featureConfig_t = featureConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct featureConfig_s {
    pub enabledFeatures: uint32_t,
}
pub type systemConfig_t = systemConfig_s;
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
pub type armingConfig_t = armingConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct armingConfig_s {
    pub gyro_cal_on_first_arm: uint8_t,
    pub auto_disarm_delay: uint8_t,
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
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
pub const PID_YAW: C2RustUnnamed_8 = 2;
pub const PID_PITCH: C2RustUnnamed_8 = 1;
pub const PID_ROLL: C2RustUnnamed_8 = 0;
pub const PID_MAG: C2RustUnnamed_8 = 4;
pub const PID_LEVEL: C2RustUnnamed_8 = 3;
pub const YAW: rc_alias = 2;
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
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
pub type pidConfig_t = pidConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
pub type currentSensorADCConfig_t = currentSensorADCConfig_s;
// extend this data size (from uint16_t)
// Idle value for DShot protocol, full motor output = 10000
// Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
// This is the maximum value for the ESCs at full power this value can be increased up to 2000
// This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
// Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
// in seconds
// allow disarm/arm on throttle down + roll left/right
// allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
// Get your magnetic decliniation from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
// mag alignment
// Which mag hardware to use on boards with more than one device
// Also used as XCLR (positive logic) for BMP085
// Barometer hardware to use
// size of baro filter array
// additional LPF to reduce baro noise
// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
// apply CF to use ACC for height estimation
// introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
// introduce a deadband around the stick center for yaw axis. Must be greater than zero.
// defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
// when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
// invert control direction of yaw
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// Processing denominator for PID controller vs gyro sampling rate
// off, on - enables pidsum runaway disarm logic
// delay in ms for "in-flight" conditions before deactivation (successful flight)
// minimum throttle percent required during deactivation phase
// WARNING - do not mix usage of CURRENT_SENSOR_* and CURRENT_METER_*, they are separate concerns.
// milliampere hours drawn from the battery since start
//
// Sensors
//
//
// ADC
//
// current read by current sensor in centiampere (1/100th A)
// current read by current sensor in centiampere (1/100th A) (unfiltered)
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentSensorADCConfig_s {
    pub scale: int16_t,
    pub offset: int16_t,
}
pub type voltageSensorADCConfig_t = voltageSensorADCConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageSensorADCConfig_s {
    pub vbatscale: uint8_t,
    pub vbatresdivval: uint8_t,
    pub vbatresdivmultiplier: uint8_t,
}
pub const VOLTAGE_SENSOR_ADC_VBAT: C2RustUnnamed_11 = 0;
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
pub struct pilotConfig_s {
    pub name: [libc::c_char; 17],
}
pub const BLACKBOX_RESERVE_SUCCESS: blackboxBufferReserveStatus_e = 0;
pub type blackboxBufferReserveStatus_e = libc::c_uint;
pub const BLACKBOX_RESERVE_PERMANENT_FAILURE: blackboxBufferReserveStatus_e =
    2;
pub const BLACKBOX_RESERVE_TEMPORARY_FAILURE: blackboxBufferReserveStatus_e =
    1;
pub type blackboxSimpleFieldDefinition_t = blackboxSimpleFieldDefinition_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxSimpleFieldDefinition_s {
    pub name: *const libc::c_char,
    pub fieldNameIndex: int8_t,
    pub isSigned: uint8_t,
    pub predict: uint8_t,
    pub encode: uint8_t,
}
pub const FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32: FlightLogFieldEncoding = 7;
pub const FLIGHT_LOG_FIELD_PREDICTOR_0: FlightLogFieldPredictor = 0;
pub const FLIGHT_LOG_FIELD_UNSIGNED: FlightLogFieldSign = 0;
pub const FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB: FlightLogFieldEncoding = 1;
/* All field definition structs should look like this (but with longer arrs): */
pub type blackboxFieldDefinition_t = blackboxFieldDefinition_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxFieldDefinition_s {
    pub name: *const libc::c_char,
    pub fieldNameIndex: int8_t,
    pub arr: [uint8_t; 1],
}
pub type blackboxDeltaFieldDefinition_t = blackboxDeltaFieldDefinition_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxDeltaFieldDefinition_s {
    pub name: *const libc::c_char,
    pub fieldNameIndex: int8_t,
    pub isSigned: uint8_t,
    pub Ipredict: uint8_t,
    pub Iencode: uint8_t,
    pub Ppredict: uint8_t,
    pub Pencode: uint8_t,
    pub condition: uint8_t,
}
pub const FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB: FlightLogFieldEncoding = 0;
pub const FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS: FlightLogFieldPredictor = 1;
pub const FLIGHT_LOG_FIELD_PREDICTOR_1500: FlightLogFieldPredictor = 8;
pub const FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2: FlightLogFieldPredictor = 3;
pub const FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0: FlightLogFieldPredictor = 5;
pub const FLIGHT_LOG_FIELD_PREDICTOR_MINMOTOR: FlightLogFieldPredictor = 11;
pub const FLIGHT_LOG_FIELD_SIGNED: FlightLogFieldSign = 1;
pub const FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB: FlightLogFieldEncoding = 6;
pub const FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT: FlightLogFieldEncoding = 3;
pub const FLIGHT_LOG_FIELD_PREDICTOR_VBATREF: FlightLogFieldPredictor = 9;
pub const FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16: FlightLogFieldEncoding = 8;
pub const FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE: FlightLogFieldPredictor = 4;
pub const FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE: FlightLogFieldPredictor =
    2;
pub const FLIGHT_LOG_FIELD_ENCODING_NULL: FlightLogFieldEncoding = 9;
pub const FLIGHT_LOG_FIELD_PREDICTOR_INC: FlightLogFieldPredictor = 6;
pub type FlightLogFieldPredictor = libc::c_uint;
pub const FLIGHT_LOG_FIELD_PREDICTOR_LAST_MAIN_FRAME_TIME:
          FlightLogFieldPredictor =
    10;
pub const FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD: FlightLogFieldPredictor = 7;
pub type FlightLogFieldEncoding = libc::c_uint;
pub const FLIGHT_LOG_FIELD_ENCODING_TAG2_3SVARIABLE: FlightLogFieldEncoding =
    10;
pub type FlightLogFieldSign = libc::c_uint;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const DEBUG_COUNT: C2RustUnnamed_4 = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed_4 = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed_4 = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed_4 = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed_4 = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed_4 = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed_4 = 38;
pub const DEBUG_RTH: C2RustUnnamed_4 = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed_4 = 36;
pub const DEBUG_USB: C2RustUnnamed_4 = 35;
pub const DEBUG_CURRENT: C2RustUnnamed_4 = 34;
pub const DEBUG_SDIO: C2RustUnnamed_4 = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed_4 = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed_4 = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed_4 = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed_4 = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed_4 = 28;
pub const DEBUG_FPORT: C2RustUnnamed_4 = 27;
pub const DEBUG_SBUS: C2RustUnnamed_4 = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed_4 = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed_4 = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed_4 = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed_4 = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed_4 = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed_4 = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed_4 = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed_4 = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed_4 = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed_4 = 16;
pub const DEBUG_FFT: C2RustUnnamed_4 = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed_4 = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed_4 = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed_4 = 12;
pub const DEBUG_STACK: C2RustUnnamed_4 = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed_4 = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed_4 = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed_4 = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed_4 = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed_4 = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed_4 = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed_4 = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed_4 = 3;
pub const DEBUG_BATTERY: C2RustUnnamed_4 = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed_4 = 1;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_5 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_5 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_5 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_5 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_5 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_5 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_5 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_5 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_5 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_5 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_5 = 65536;
pub const FEATURE_RX_MSP: C2RustUnnamed_5 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_5 = 8192;
pub const FEATURE_3D: C2RustUnnamed_5 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_5 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_5 = 512;
pub const FEATURE_GPS: C2RustUnnamed_5 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_5 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_5 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_5 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_5 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_5 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_5 = 1;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub type C2RustUnnamed_6 = libc::c_uint;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_7 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_7 = 2;
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
pub type C2RustUnnamed_8 = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed_8 = 5;
// If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
// Each member of this array will be the value to print for this field for the given header index
// Decide whether this field should appear in the log
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
pub type C2RustUnnamed_9 = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed_9 = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed_9 = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed_9 = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed_9 = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed_9 = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed_9 = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed_9 = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed_9 = 7;
pub const INPUT_RC_YAW: C2RustUnnamed_9 = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed_9 = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed_9 = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed_9 = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed_9 = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed_9 = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed_9 = 0;
pub type C2RustUnnamed_10 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_10 = 64;
pub const SENSOR_GPS: C2RustUnnamed_10 = 32;
pub const SENSOR_SONAR: C2RustUnnamed_10 = 16;
pub const SENSOR_GYRO: C2RustUnnamed_10 = 1;
pub type C2RustUnnamed_11 = libc::c_uint;
pub const VOLTAGE_SENSOR_ADC_5V: C2RustUnnamed_11 = 3;
pub const VOLTAGE_SENSOR_ADC_9V: C2RustUnnamed_11 = 2;
pub const VOLTAGE_SENSOR_ADC_12V: C2RustUnnamed_11 = 1;
#[inline]
unsafe extern "C" fn blackboxConfig() -> *const blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn blackboxConfigMutable() -> *mut blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn featureConfig() -> *const featureConfig_t {
    return &mut featureConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn pilotConfig() -> *const pilotConfig_t {
    return &mut pilotConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn controlRateProfiles(mut _index: libc::c_int)
 -> *const controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
}
#[inline]
unsafe extern "C" fn rcControlsConfig() -> *const rcControlsConfig_t {
    return &mut rcControlsConfig_System;
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
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn pidConfig() -> *const pidConfig_t {
    return &mut pidConfig_System;
}
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed_9 = INPUT_STABILIZED_ROLL;
#[inline]
unsafe extern "C" fn accelerometerConfig() -> *const accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn gyroConfig() -> *const gyroConfig_t {
    return &mut gyroConfig_System;
}
#[inline]
unsafe extern "C" fn barometerConfig() -> *const barometerConfig_t {
    return &mut barometerConfig_System;
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfig(mut _index: libc::c_int)
 -> *const voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
}
#[inline]
unsafe extern "C" fn currentSensorADCConfig()
 -> *const currentSensorADCConfig_t {
    return &mut currentSensorADCConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn compassConfig() -> *const compassConfig_t {
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
pub static mut blackboxConfig_Copy: blackboxConfig_t =
    blackboxConfig_t{p_ratio: 0, device: 0, record_acc: 0, mode: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut blackboxConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (5 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<blackboxConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &blackboxConfig_System as
                                     *const blackboxConfig_t as
                                     *mut blackboxConfig_t as *mut uint8_t,
                             copy:
                                 &blackboxConfig_Copy as
                                     *const blackboxConfig_t as
                                     *mut blackboxConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_blackboxConfig
                                                         as
                                                         *const blackboxConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut blackboxConfig_System: blackboxConfig_t =
    blackboxConfig_t{p_ratio: 0, device: 0, record_acc: 0, mode: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_blackboxConfig: blackboxConfig_t =
    {
        let mut init =
            blackboxConfig_s{p_ratio: 32 as libc::c_int as uint16_t,
                             device:
                                 BLACKBOX_DEVICE_SERIAL as libc::c_int as
                                     uint8_t,
                             record_acc: 1 as libc::c_int as uint8_t,
                             mode:
                                 BLACKBOX_MODE_NORMAL as libc::c_int as
                                     uint8_t,};
        init
    };
static mut blackboxHeader: [libc::c_char; 79] =
    [72, 32, 80, 114, 111, 100, 117, 99, 116, 58, 66, 108, 97, 99, 107, 98,
     111, 120, 32, 102, 108, 105, 103, 104, 116, 32, 100, 97, 116, 97, 32,
     114, 101, 99, 111, 114, 100, 101, 114, 32, 98, 121, 32, 78, 105, 99, 104,
     111, 108, 97, 115, 32, 83, 104, 101, 114, 108, 111, 99, 107, 10, 72, 32,
     68, 97, 116, 97, 32, 118, 101, 114, 115, 105, 111, 110, 58, 50, 10, 0];
static mut blackboxFieldHeaderNames: [*const libc::c_char; 6] =
    [b"name\x00" as *const u8 as *const libc::c_char,
     b"signed\x00" as *const u8 as *const libc::c_char,
     b"predictor\x00" as *const u8 as *const libc::c_char,
     b"encoding\x00" as *const u8 as *const libc::c_char,
     b"predictor\x00" as *const u8 as *const libc::c_char,
     b"encoding\x00" as *const u8 as *const libc::c_char];
/* *
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static mut blackboxMainFields: [blackboxDeltaFieldDefinition_t; 45] =
    [{
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"loopIteration\x00" as
                                                    *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                -(1 as libc::c_int) as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_INC
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_NULL
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"time\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                -(1 as libc::c_int) as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisP\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisP\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisP\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisI\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisI\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisI\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisD\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisD\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisD\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisF\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisF\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"axisF\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"rcCommand\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"rcCommand\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"rcCommand\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"rcCommand\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                3 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"vbatLatest\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                -(1 as libc::c_int) as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_VBATREF
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_VBAT
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"amperageLatest\x00" as
                                                    *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                -(1 as libc::c_int) as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"magADC\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_MAG
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"magADC\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_MAG
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"magADC\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_MAG
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"BaroAlt\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                -(1 as libc::c_int) as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_BARO
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"surfaceRaw\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                -(1 as libc::c_int) as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"rssi\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                -(1 as libc::c_int) as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_RSSI
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"gyroADC\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"gyroADC\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"gyroADC\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ALWAYS
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"accSmooth\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ACC
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"accSmooth\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ACC
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"accSmooth\x00" as *const u8
                                                    as *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_ACC
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"debug\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_DEBUG
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"debug\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_DEBUG
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"debug\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_DEBUG
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"debug\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                3 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_SIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_DEBUG
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"motor\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                0 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MINMOTOR
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"motor\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                1 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"motor\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                2 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"motor\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                3 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"motor\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                4 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"motor\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                5 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"motor\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                6 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"motor\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                7 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     },
     {
         let mut init =
             blackboxDeltaFieldDefinition_s{name:
                                                b"servo\x00" as *const u8 as
                                                    *const libc::c_char,
                                            fieldNameIndex:
                                                5 as libc::c_int as int8_t,
                                            isSigned:
                                                FLIGHT_LOG_FIELD_UNSIGNED as
                                                    libc::c_int as uint8_t,
                                            Ipredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_1500
                                                    as libc::c_int as uint8_t,
                                            Iencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            Ppredict:
                                                FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS
                                                    as libc::c_int as uint8_t,
                                            Pencode:
                                                FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB
                                                    as libc::c_int as uint8_t,
                                            condition:
                                                FLIGHT_LOG_FIELD_CONDITION_TRICOPTER
                                                    as libc::c_int as
                                                    uint8_t,};
         init
     }];
// Rarely-updated fields
static mut blackboxSlowFields: [blackboxSimpleFieldDefinition_t; 5] =
    [{
         let mut init =
             blackboxSimpleFieldDefinition_s{name:
                                                 b"flightModeFlags\x00" as
                                                     *const u8 as
                                                     *const libc::c_char,
                                             fieldNameIndex:
                                                 -(1 as libc::c_int) as
                                                     int8_t,
                                             isSigned:
                                                 FLIGHT_LOG_FIELD_UNSIGNED as
                                                     libc::c_int as uint8_t,
                                             predict:
                                                 FLIGHT_LOG_FIELD_PREDICTOR_0
                                                     as libc::c_int as
                                                     uint8_t,
                                             encode:
                                                 FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB
                                                     as libc::c_int as
                                                     uint8_t,};
         init
     },
     {
         let mut init =
             blackboxSimpleFieldDefinition_s{name:
                                                 b"stateFlags\x00" as
                                                     *const u8 as
                                                     *const libc::c_char,
                                             fieldNameIndex:
                                                 -(1 as libc::c_int) as
                                                     int8_t,
                                             isSigned:
                                                 FLIGHT_LOG_FIELD_UNSIGNED as
                                                     libc::c_int as uint8_t,
                                             predict:
                                                 FLIGHT_LOG_FIELD_PREDICTOR_0
                                                     as libc::c_int as
                                                     uint8_t,
                                             encode:
                                                 FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB
                                                     as libc::c_int as
                                                     uint8_t,};
         init
     },
     {
         let mut init =
             blackboxSimpleFieldDefinition_s{name:
                                                 b"failsafePhase\x00" as
                                                     *const u8 as
                                                     *const libc::c_char,
                                             fieldNameIndex:
                                                 -(1 as libc::c_int) as
                                                     int8_t,
                                             isSigned:
                                                 FLIGHT_LOG_FIELD_UNSIGNED as
                                                     libc::c_int as uint8_t,
                                             predict:
                                                 FLIGHT_LOG_FIELD_PREDICTOR_0
                                                     as libc::c_int as
                                                     uint8_t,
                                             encode:
                                                 FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32
                                                     as libc::c_int as
                                                     uint8_t,};
         init
     },
     {
         let mut init =
             blackboxSimpleFieldDefinition_s{name:
                                                 b"rxSignalReceived\x00" as
                                                     *const u8 as
                                                     *const libc::c_char,
                                             fieldNameIndex:
                                                 -(1 as libc::c_int) as
                                                     int8_t,
                                             isSigned:
                                                 FLIGHT_LOG_FIELD_UNSIGNED as
                                                     libc::c_int as uint8_t,
                                             predict:
                                                 FLIGHT_LOG_FIELD_PREDICTOR_0
                                                     as libc::c_int as
                                                     uint8_t,
                                             encode:
                                                 FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32
                                                     as libc::c_int as
                                                     uint8_t,};
         init
     },
     {
         let mut init =
             blackboxSimpleFieldDefinition_s{name:
                                                 b"rxFlightChannelsValid\x00"
                                                     as *const u8 as
                                                     *const libc::c_char,
                                             fieldNameIndex:
                                                 -(1 as libc::c_int) as
                                                     int8_t,
                                             isSigned:
                                                 FLIGHT_LOG_FIELD_UNSIGNED as
                                                     libc::c_int as uint8_t,
                                             predict:
                                                 FLIGHT_LOG_FIELD_PREDICTOR_0
                                                     as libc::c_int as
                                                     uint8_t,
                                             encode:
                                                 FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32
                                                     as libc::c_int as
                                                     uint8_t,};
         init
     }];
static mut blackboxState: BlackboxState = BLACKBOX_STATE_DISABLED;
static mut blackboxLastArmingBeep: uint32_t = 0 as libc::c_int as uint32_t;
static mut blackboxLastFlightModeFlags: uint32_t =
    0 as libc::c_int as uint32_t;
static mut xmitState: C2RustUnnamed_2 =
    C2RustUnnamed_2{headerIndex: 0, u: C2RustUnnamed_1{fieldIndex: 0,},};
// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
static mut blackboxConditionCache: uint32_t = 0;
static mut blackboxIteration: uint32_t = 0;
static mut blackboxLoopIndex: uint16_t = 0;
static mut blackboxPFrameIndex: uint16_t = 0;
static mut blackboxIFrameIndex: uint16_t = 0;
// number of flight loop iterations before logging I-frame
// typically 32 for 1kHz loop, 64 for 2kHz loop etc
static mut blackboxIInterval: int16_t = 0 as libc::c_int as int16_t;
// number of flight loop iterations before logging P-frame
static mut blackboxPInterval: int16_t = 0 as libc::c_int as int16_t;
static mut blackboxSInterval: int32_t = 0 as libc::c_int;
static mut blackboxSlowFrameIterationTimer: int32_t = 0;
static mut blackboxLoggedAnyFrames: bool = false;
/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static mut vbatReference: uint16_t = 0;
static mut gpsHistory: blackboxGpsState_t =
    blackboxGpsState_t{GPS_home: [0; 2], GPS_coord: [0; 2], GPS_numSat: 0,};
static mut slowHistory: blackboxSlowState_t =
    blackboxSlowState_t{flightModeFlags: 0,
                        stateFlags: 0,
                        failsafePhase: 0,
                        rxSignalReceived: false,
                        rxFlightChannelsValid: false,};
// Keep a history of length 2, plus a buffer for MW to store the new values into
static mut blackboxHistoryRing: [blackboxMainState_t; 3] =
    [blackboxMainState_t{time: 0,
                         axisPID_P: [0; 3],
                         axisPID_I: [0; 3],
                         axisPID_D: [0; 3],
                         axisPID_F: [0; 3],
                         rcCommand: [0; 4],
                         gyroADC: [0; 3],
                         accADC: [0; 3],
                         debug: [0; 4],
                         motor: [0; 8],
                         servo: [0; 8],
                         vbatLatest: 0,
                         amperageLatest: 0,
                         BaroAlt: 0,
                         magADC: [0; 3],
                         surfaceRaw: 0,
                         rssi: 0,}; 3];
// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static mut blackboxHistory: [*mut blackboxMainState_t; 3] =
    [0 as *const blackboxMainState_t as *mut blackboxMainState_t; 3];
static mut blackboxModeActivationConditionPresent: bool =
    0 as libc::c_int != 0;
/* *
 * Return true if it is safe to edit the Blackbox configuration.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxMayEditConfig() -> bool {
    return blackboxState as libc::c_uint <=
               BLACKBOX_STATE_STOPPED as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn blackboxIsOnlyLoggingIntraframes() -> bool {
    return (*blackboxConfig()).p_ratio as libc::c_int == 0 as libc::c_int;
}
unsafe extern "C" fn testBlackboxConditionUncached(mut condition:
                                                       FlightLogFieldCondition)
 -> bool {
    match condition as libc::c_uint {
        0 => { return 1 as libc::c_int != 0 }
        1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 => {
            return getMotorCount() as libc::c_uint >=
                       (condition as
                            libc::c_uint).wrapping_sub(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1
                                                           as libc::c_int as
                                                           libc::c_uint).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
        }
        9 => {
            return (*mixerConfig()).mixerMode as libc::c_int ==
                       MIXER_TRI as libc::c_int ||
                       (*mixerConfig()).mixerMode as libc::c_int ==
                           MIXER_CUSTOM_TRI as libc::c_int
        }
        16 | 17 | 18 => {
            return (*currentPidProfile).pid[(condition as
                                                 libc::c_uint).wrapping_sub(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as usize].D as libc::c_int !=
                       0 as libc::c_int
        }
        10 => { return sensors(SENSOR_MAG as libc::c_int as uint32_t) }
        11 => { return sensors(SENSOR_BARO as libc::c_int as uint32_t) }
        12 => {
            return (*batteryConfig()).voltageMeterSource as libc::c_uint !=
                       VOLTAGE_METER_NONE as libc::c_int as libc::c_uint
        }
        13 => {
            return (*batteryConfig()).currentMeterSource as libc::c_uint !=
                       CURRENT_METER_NONE as libc::c_int as libc::c_uint &&
                       (*batteryConfig()).currentMeterSource as libc::c_uint
                           !=
                           CURRENT_METER_VIRTUAL as libc::c_int as
                               libc::c_uint
        }
        14 => {
            return sensors(SENSOR_RANGEFINDER as libc::c_int as uint32_t)
        }
        15 => {
            return (*rxConfig()).rssi_channel as libc::c_int >
                       0 as libc::c_int ||
                       feature(FEATURE_RSSI_ADC as libc::c_int as uint32_t) as
                           libc::c_int != 0
        }
        19 => {
            return (*blackboxConfig()).p_ratio as libc::c_int !=
                       1 as libc::c_int
        }
        20 => {
            return sensors(SENSOR_ACC as libc::c_int as uint32_t) as
                       libc::c_int != 0 &&
                       (*blackboxConfig()).record_acc as libc::c_int != 0
        }
        21 => { return debugMode as libc::c_int != DEBUG_NONE as libc::c_int }
        22 => { return 0 as libc::c_int != 0 }
        _ => { return 0 as libc::c_int != 0 }
    };
}
unsafe extern "C" fn blackboxBuildConditionCache() {
    blackboxConditionCache = 0 as libc::c_int as uint32_t;
    let mut cond: FlightLogFieldCondition = FLIGHT_LOG_FIELD_CONDITION_FIRST;
    while cond as libc::c_uint <=
              FLIGHT_LOG_FIELD_CONDITION_LAST as libc::c_int as libc::c_uint {
        if testBlackboxConditionUncached(cond) {
            blackboxConditionCache |=
                ((1 as libc::c_int) << cond as libc::c_uint) as libc::c_uint
        }
        cond += 1
    };
}
unsafe extern "C" fn testBlackboxCondition(mut condition:
                                               FlightLogFieldCondition)
 -> bool {
    return blackboxConditionCache &
               ((1 as libc::c_int) << condition as libc::c_uint) as
                   libc::c_uint != 0 as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn blackboxSetState(mut newState: BlackboxState) {
    //Perform initial setup required for the new state
    match newState as libc::c_uint {
        2 => { blackboxLoggedAnyFrames = 0 as libc::c_int != 0 }
        3 => {
            blackboxHeaderBudget =
                0 as
                    libc::c_int; //Force a slow frame to be written on the first iteration
            xmitState.headerIndex = 0 as libc::c_int as uint32_t;
            xmitState.u.startTime = millis()
        }
        4 | 6 | 5 | 7 => {
            xmitState.headerIndex = 0 as libc::c_int as uint32_t;
            xmitState.u.fieldIndex = -(1 as libc::c_int)
        }
        8 => { xmitState.headerIndex = 0 as libc::c_int as uint32_t }
        10 => { blackboxSlowFrameIterationTimer = blackboxSInterval }
        11 => { xmitState.u.startTime = millis() }
        _ => { }
    }
    blackboxState = newState;
}
unsafe extern "C" fn writeIntraframe() {
    let mut blackboxCurrent: *mut blackboxMainState_t =
        blackboxHistory[0 as libc::c_int as usize];
    blackboxWrite('I' as i32 as uint8_t);
    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB((*blackboxCurrent).time);
    blackboxWriteSignedVBArray((*blackboxCurrent).axisPID_P.as_mut_ptr(),
                               3 as libc::c_int);
    blackboxWriteSignedVBArray((*blackboxCurrent).axisPID_I.as_mut_ptr(),
                               3 as libc::c_int);
    // Don't bother writing the current D term if the corresponding PID setting is zero
    let mut x: libc::c_int = 0 as libc::c_int;
    while x < 3 as libc::c_int {
        if testBlackboxCondition((FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0
                                      as libc::c_int + x) as
                                     FlightLogFieldCondition) {
            blackboxWriteSignedVB((*blackboxCurrent).axisPID_D[x as usize]);
        }
        x += 1
    }
    blackboxWriteSignedVBArray((*blackboxCurrent).axisPID_F.as_mut_ptr(),
                               3 as libc::c_int);
    // Write roll, pitch and yaw first:
    blackboxWriteSigned16VBArray((*blackboxCurrent).rcCommand.as_mut_ptr(),
                                 3 as libc::c_int);
    /*
     * Write the throttle separately from the rest of the RC data so we can apply a predictor to it.
     * Throttle lies in range [minthrottle..maxthrottle]:
     */
    blackboxWriteUnsignedVB(((*blackboxCurrent).rcCommand[THROTTLE as
                                                              libc::c_int as
                                                              usize] as
                                 libc::c_int -
                                 (*motorConfig()).minthrottle as libc::c_int)
                                as uint32_t);
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT) {
        /*
         * Our voltage is expected to decrease over the course of the flight, so store our difference from
         * the reference:
         *
         * Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
         */
        blackboxWriteUnsignedVB((vbatReference as libc::c_int -
                                     (*blackboxCurrent).vbatLatest as
                                         libc::c_int & 0x3fff as libc::c_int)
                                    as uint32_t);
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC) {
        // 12bit value directly from ADC
        blackboxWriteSignedVB((*blackboxCurrent).amperageLatest);
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG) {
        blackboxWriteSigned16VBArray((*blackboxCurrent).magADC.as_mut_ptr(),
                                     3 as libc::c_int);
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO) {
        blackboxWriteSignedVB((*blackboxCurrent).BaroAlt);
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER) {
        blackboxWriteSignedVB((*blackboxCurrent).surfaceRaw);
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI) {
        blackboxWriteUnsignedVB((*blackboxCurrent).rssi as uint32_t);
    }
    blackboxWriteSigned16VBArray((*blackboxCurrent).gyroADC.as_mut_ptr(),
                                 3 as libc::c_int);
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_ACC) {
        blackboxWriteSigned16VBArray((*blackboxCurrent).accADC.as_mut_ptr(),
                                     3 as libc::c_int);
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG) {
        blackboxWriteSigned16VBArray((*blackboxCurrent).debug.as_mut_ptr(),
                                     4 as libc::c_int);
    }
    //Motors can be below minimum output when disarmed, but that doesn't happen much
    blackboxWriteUnsignedVB(((*blackboxCurrent).motor[0 as libc::c_int as
                                                          usize] as
                                 libc::c_int as libc::c_float -
                                 motorOutputLow) as uint32_t);
    //Motors tend to be similar to each other so use the first motor's value as a predictor of the others
    let motorCount: libc::c_int = getMotorCount() as libc::c_int;
    let mut x_0: libc::c_int = 1 as libc::c_int;
    while x_0 < motorCount {
        blackboxWriteSignedVB((*blackboxCurrent).motor[x_0 as usize] as
                                  libc::c_int -
                                  (*blackboxCurrent).motor[0 as libc::c_int as
                                                               usize] as
                                      libc::c_int);
        x_0 += 1
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER) {
        //Assume the tail spends most of its time around the center
        blackboxWriteSignedVB((*blackboxCurrent).servo[5 as libc::c_int as
                                                           usize] as
                                  libc::c_int - 1500 as libc::c_int);
    }
    //Rotate our history buffers:
    //The current state becomes the new "before" state
    blackboxHistory[1 as libc::c_int as usize] =
        blackboxHistory[0 as libc::c_int as usize];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2 as libc::c_int as usize] =
        blackboxHistory[0 as libc::c_int as usize];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0 as libc::c_int as usize] =
        blackboxHistoryRing.as_mut_ptr().offset(((blackboxHistory[0 as
                                                                      libc::c_int
                                                                      as
                                                                      usize].wrapping_offset_from(blackboxHistoryRing.as_mut_ptr())
                                                      as libc::c_long +
                                                      1 as libc::c_int as
                                                          libc::c_long) %
                                                     3 as libc::c_int as
                                                         libc::c_long) as
                                                    isize);
    blackboxLoggedAnyFrames = 1 as libc::c_int != 0;
}
unsafe extern "C" fn blackboxWriteMainStateArrayUsingAveragePredictor(mut arrOffsetInHistory:
                                                                          libc::c_int,
                                                                      mut count:
                                                                          libc::c_int) {
    let mut curr: *mut int16_t =
        (blackboxHistory[0 as libc::c_int as usize] as
             *mut libc::c_char).offset(arrOffsetInHistory as isize) as
            *mut int16_t;
    let mut prev1: *mut int16_t =
        (blackboxHistory[1 as libc::c_int as usize] as
             *mut libc::c_char).offset(arrOffsetInHistory as isize) as
            *mut int16_t;
    let mut prev2: *mut int16_t =
        (blackboxHistory[2 as libc::c_int as usize] as
             *mut libc::c_char).offset(arrOffsetInHistory as isize) as
            *mut int16_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < count {
        // Predictor is the average of the previous two history states
        let mut predictor: int32_t =
            (*prev1.offset(i as isize) as libc::c_int +
                 *prev2.offset(i as isize) as libc::c_int) / 2 as libc::c_int;
        blackboxWriteSignedVB(*curr.offset(i as isize) as libc::c_int -
                                  predictor);
        i += 1
    };
}
unsafe extern "C" fn writeInterframe() {
    let mut blackboxCurrent: *mut blackboxMainState_t =
        blackboxHistory[0 as libc::c_int as usize];
    let mut blackboxLast: *mut blackboxMainState_t =
        blackboxHistory[1 as libc::c_int as usize];
    blackboxWrite('P' as i32 as uint8_t);
    //No need to store iteration count since its delta is always 1
    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((*blackboxHistory[0 as libc::c_int as
                                                usize]).time.wrapping_sub((2
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_mul((*blackboxHistory[1
                                                                                                                                as
                                                                                                                                libc::c_int
                                                                                                                                as
                                                                                                                                usize]).time)).wrapping_add((*blackboxHistory[2
                                                                                                                                                                                  as
                                                                                                                                                                                  libc::c_int
                                                                                                                                                                                  as
                                                                                                                                                                                  usize]).time)
                              as int32_t);
    let mut deltas: [int32_t; 8] = [0; 8];
    arraySubInt32(deltas.as_mut_ptr(),
                  (*blackboxCurrent).axisPID_P.as_mut_ptr(),
                  (*blackboxLast).axisPID_P.as_mut_ptr(), 3 as libc::c_int);
    blackboxWriteSignedVBArray(deltas.as_mut_ptr(), 3 as libc::c_int);
    /*
     * The PID I field changes very slowly, most of the time +-2, so use an encoding
     * that can pack all three fields into one byte in that situation.
     */
    arraySubInt32(deltas.as_mut_ptr(),
                  (*blackboxCurrent).axisPID_I.as_mut_ptr(),
                  (*blackboxLast).axisPID_I.as_mut_ptr(), 3 as libc::c_int);
    blackboxWriteTag2_3S32(deltas.as_mut_ptr());
    /*
     * The PID D term is frequently set to zero for yaw, which makes the result from the calculation
     * always zero. So don't bother recording D results when PID D terms are zero.
     */
    let mut x: libc::c_int = 0 as libc::c_int;
    while x < 3 as libc::c_int {
        if testBlackboxCondition((FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0
                                      as libc::c_int + x) as
                                     FlightLogFieldCondition) {
            blackboxWriteSignedVB((*blackboxCurrent).axisPID_D[x as usize] -
                                      (*blackboxLast).axisPID_D[x as usize]);
        }
        x += 1
    }
    arraySubInt32(deltas.as_mut_ptr(),
                  (*blackboxCurrent).axisPID_F.as_mut_ptr(),
                  (*blackboxLast).axisPID_F.as_mut_ptr(), 3 as libc::c_int);
    blackboxWriteSignedVBArray(deltas.as_mut_ptr(), 3 as libc::c_int);
    /*
     * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
     * can pack multiple values per byte:
     */
    let mut x_0: libc::c_int = 0 as libc::c_int;
    while x_0 < 4 as libc::c_int {
        deltas[x_0 as usize] =
            (*blackboxCurrent).rcCommand[x_0 as usize] as libc::c_int -
                (*blackboxLast).rcCommand[x_0 as usize] as libc::c_int;
        x_0 += 1
    }
    blackboxWriteTag8_4S16(deltas.as_mut_ptr());
    //Check for sensors that are updated periodically (so deltas are normally zero)
    let mut optionalFieldCount: libc::c_int = 0 as libc::c_int;
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT) {
        let fresh0 = optionalFieldCount;
        optionalFieldCount = optionalFieldCount + 1;
        deltas[fresh0 as usize] =
            (*blackboxCurrent).vbatLatest as int32_t -
                (*blackboxLast).vbatLatest as libc::c_int
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC) {
        let fresh1 = optionalFieldCount;
        optionalFieldCount = optionalFieldCount + 1;
        deltas[fresh1 as usize] =
            (*blackboxCurrent).amperageLatest - (*blackboxLast).amperageLatest
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG) {
        let mut x_1: libc::c_int = 0 as libc::c_int;
        while x_1 < 3 as libc::c_int {
            let fresh2 = optionalFieldCount;
            optionalFieldCount = optionalFieldCount + 1;
            deltas[fresh2 as usize] =
                (*blackboxCurrent).magADC[x_1 as usize] as libc::c_int -
                    (*blackboxLast).magADC[x_1 as usize] as libc::c_int;
            x_1 += 1
        }
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO) {
        let fresh3 = optionalFieldCount;
        optionalFieldCount = optionalFieldCount + 1;
        deltas[fresh3 as usize] =
            (*blackboxCurrent).BaroAlt - (*blackboxLast).BaroAlt
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER) {
        let fresh4 = optionalFieldCount;
        optionalFieldCount = optionalFieldCount + 1;
        deltas[fresh4 as usize] =
            (*blackboxCurrent).surfaceRaw - (*blackboxLast).surfaceRaw
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI) {
        let fresh5 = optionalFieldCount;
        optionalFieldCount = optionalFieldCount + 1;
        deltas[fresh5 as usize] =
            (*blackboxCurrent).rssi as int32_t -
                (*blackboxLast).rssi as libc::c_int
    }
    blackboxWriteTag8_8SVB(deltas.as_mut_ptr(), optionalFieldCount);
    //Since gyros, accs and motors are noisy, base their predictions on the average of the history:
    blackboxWriteMainStateArrayUsingAveragePredictor(60 as libc::c_ulong as
                                                         libc::c_int,
                                                     3 as libc::c_int);
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_ACC) {
        blackboxWriteMainStateArrayUsingAveragePredictor(66 as libc::c_ulong
                                                             as libc::c_int,
                                                         3 as libc::c_int);
    }
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG) {
        blackboxWriteMainStateArrayUsingAveragePredictor(72 as libc::c_ulong
                                                             as libc::c_int,
                                                         4 as libc::c_int);
    }
    blackboxWriteMainStateArrayUsingAveragePredictor(80 as libc::c_ulong as
                                                         libc::c_int,
                                                     getMotorCount() as
                                                         libc::c_int);
    if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER) {
        blackboxWriteSignedVB((*blackboxCurrent).servo[5 as libc::c_int as
                                                           usize] as
                                  libc::c_int -
                                  (*blackboxLast).servo[5 as libc::c_int as
                                                            usize] as
                                      libc::c_int);
    }
    //Rotate our history buffers
    blackboxHistory[2 as libc::c_int as usize] =
        blackboxHistory[1 as libc::c_int as usize];
    blackboxHistory[1 as libc::c_int as usize] =
        blackboxHistory[0 as libc::c_int as usize];
    blackboxHistory[0 as libc::c_int as usize] =
        blackboxHistoryRing.as_mut_ptr().offset(((blackboxHistory[0 as
                                                                      libc::c_int
                                                                      as
                                                                      usize].wrapping_offset_from(blackboxHistoryRing.as_mut_ptr())
                                                      as libc::c_long +
                                                      1 as libc::c_int as
                                                          libc::c_long) %
                                                     3 as libc::c_int as
                                                         libc::c_long) as
                                                    isize);
    blackboxLoggedAnyFrames = 1 as libc::c_int != 0;
}
/* Write the contents of the global "slowHistory" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
unsafe extern "C" fn writeSlowFrame() {
    let mut values: [int32_t; 3] = [0; 3];
    blackboxWrite('S' as i32 as uint8_t);
    blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
    blackboxWriteUnsignedVB(slowHistory.stateFlags as uint32_t);
    /*
     * Most of the time these three values will be able to pack into one byte for us:
     */
    values[0 as libc::c_int as usize] = slowHistory.failsafePhase as int32_t;
    values[1 as libc::c_int as usize] =
        if slowHistory.rxSignalReceived as libc::c_int != 0 {
            1 as libc::c_int
        } else { 0 as libc::c_int };
    values[2 as libc::c_int as usize] =
        if slowHistory.rxFlightChannelsValid as libc::c_int != 0 {
            1 as libc::c_int
        } else { 0 as libc::c_int };
    blackboxWriteTag2_3S32(values.as_mut_ptr());
    blackboxSlowFrameIterationTimer = 0 as libc::c_int;
}
/* *
 * Load rarely-changing values from the FC into the given structure
 */
unsafe extern "C" fn loadSlowState(mut slow: *mut blackboxSlowState_t) {
    memcpy(&mut (*slow).flightModeFlags as *mut uint32_t as *mut libc::c_void,
           &mut rcModeActivationMask as *mut boxBitmask_t as
               *const libc::c_void,
           ::core::mem::size_of::<uint32_t>() as
               libc::c_ulong); //was flightModeFlags;
    (*slow).stateFlags = stateFlags;
    (*slow).failsafePhase = failsafePhase() as uint8_t;
    (*slow).rxSignalReceived = rxIsReceivingSignal();
    (*slow).rxFlightChannelsValid = rxAreFlightChannelsValid();
}
/* *
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than blackboxSInterval logging iterations
 * since the field was last logged.
 */
unsafe extern "C" fn writeSlowFrameIfNeeded() -> bool {
    // Write the slow frame peridocially so it can be recovered if we ever lose sync
    let mut shouldWrite: bool =
        blackboxSlowFrameIterationTimer >= blackboxSInterval;
    if shouldWrite {
        loadSlowState(&mut slowHistory);
    } else {
        let mut newSlowState: blackboxSlowState_t =
            blackboxSlowState_t{flightModeFlags: 0,
                                stateFlags: 0,
                                failsafePhase: 0,
                                rxSignalReceived: false,
                                rxFlightChannelsValid: false,};
        loadSlowState(&mut newSlowState);
        // Only write a slow frame if it was different from the previous state
        if memcmp(&mut newSlowState as *mut blackboxSlowState_t as
                      *const libc::c_void,
                  &mut slowHistory as *mut blackboxSlowState_t as
                      *const libc::c_void,
                  ::core::mem::size_of::<blackboxSlowState_t>() as
                      libc::c_ulong) != 0 as libc::c_int {
            // Use the new state as our new history
            memcpy(&mut slowHistory as *mut blackboxSlowState_t as
                       *mut libc::c_void,
                   &mut newSlowState as *mut blackboxSlowState_t as
                       *const libc::c_void,
                   ::core::mem::size_of::<blackboxSlowState_t>() as
                       libc::c_ulong);
            shouldWrite = 1 as libc::c_int != 0
        }
    }
    if shouldWrite { writeSlowFrame(); }
    return shouldWrite;
}
#[no_mangle]
pub unsafe extern "C" fn blackboxValidateConfig() {
    // If we've chosen an unsupported device, change the device to serial
    match (*blackboxConfig()).device as libc::c_int {
        3 => { }
        _ => {
            (*blackboxConfigMutable()).device =
                BLACKBOX_DEVICE_SERIAL as libc::c_int as uint8_t
        }
    };
}
unsafe extern "C" fn blackboxResetIterationTimers() {
    blackboxIteration = 0 as libc::c_int as uint32_t;
    blackboxLoopIndex = 0 as libc::c_int as uint16_t;
    blackboxIFrameIndex = 0 as libc::c_int as uint16_t;
    blackboxPFrameIndex = 0 as libc::c_int as uint16_t;
    blackboxSlowFrameIterationTimer = 0 as libc::c_int;
}
/* *
 * Start Blackbox logging if it is not already running. Intended to be called upon arming.
 */
unsafe extern "C" fn blackboxStart() {
    blackboxValidateConfig();
    if !blackboxDeviceOpen() {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
        return
    }
    memset(&mut gpsHistory as *mut blackboxGpsState_t as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<blackboxGpsState_t>() as libc::c_ulong);
    blackboxHistory[0 as libc::c_int as usize] =
        &mut *blackboxHistoryRing.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
            *mut blackboxMainState_t;
    blackboxHistory[1 as libc::c_int as usize] =
        &mut *blackboxHistoryRing.as_mut_ptr().offset(1 as libc::c_int as
                                                          isize) as
            *mut blackboxMainState_t;
    blackboxHistory[2 as libc::c_int as usize] =
        &mut *blackboxHistoryRing.as_mut_ptr().offset(2 as libc::c_int as
                                                          isize) as
            *mut blackboxMainState_t;
    vbatReference = getBatteryVoltageLatest();
    //No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it
    /*
     * We use conditional tests to decide whether or not certain fields should be logged. Since our headers
     * must always agree with the logged data, the results of these tests must not change during logging. So
     * cache those now.
     */
    blackboxBuildConditionCache();
    blackboxModeActivationConditionPresent =
        isModeActivationConditionPresent(BOXBLACKBOX);
    blackboxResetIterationTimers();
    /*
     * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
     * it finally plays the beep for this arming event.
     */
    blackboxLastArmingBeep =
        getArmingBeepTimeMicros(); // record startup status
    memcpy(&mut blackboxLastFlightModeFlags as *mut uint32_t as
               *mut libc::c_void,
           &mut rcModeActivationMask as *mut boxBitmask_t as
               *const libc::c_void,
           ::core::mem::size_of::<uint32_t>() as libc::c_ulong);
    blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
}
/* *
 * Begin Blackbox shutdown.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxFinish() {
    let mut current_block_3: u64;
    match blackboxState as libc::c_uint {
        0 | 1 | 11 => { current_block_3 = 11006700562992250127; }
        10 | 9 => {
            blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END,
                             0 as *mut flightLogEventData_u);
            current_block_3 = 14343965764134602790;
        }
        _ => { current_block_3 = 14343965764134602790; }
    }
    match current_block_3 {
        14343965764134602790 => {
            blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
        }
        _ => { }
    };
}
/* *
 * Test Motors Blackbox Logging
 */
static mut startedLoggingInTestMode: bool = 0 as libc::c_int != 0;
unsafe extern "C" fn startInTestMode() {
    if !startedLoggingInTestMode {
        if (*blackboxConfig()).device as libc::c_int ==
               BLACKBOX_DEVICE_SERIAL as libc::c_int {
            let mut sharedBlackboxAndMspPort: *mut serialPort_t =
                findSharedSerialPort(FUNCTION_BLACKBOX as libc::c_int as
                                         uint16_t, FUNCTION_MSP);
            if !sharedBlackboxAndMspPort.is_null() {
                return
                // When in test mode, we cannot share the MSP and serial logger port!
            }
        }
        blackboxStart();
        startedLoggingInTestMode = 1 as libc::c_int != 0
    };
}
unsafe extern "C" fn stopInTestMode() {
    if startedLoggingInTestMode {
        blackboxFinish();
        startedLoggingInTestMode = 0 as libc::c_int != 0
    };
}
/* *
 * We are going to monitor the MSP_SET_MOTOR target variables motor_disarmed[] for values other than minthrottle
 * on reading a value (i.e. the user is testing the motors), then we enable test mode logging;
 * we monitor when the values return to minthrottle and start a delay timer (5 seconds); if
 * the test motors are left at minimum throttle for this delay timer, then we assume we are done testing and
 * shutdown the logger.
 *
 * Of course, after the 5 seconds and shutdown of the logger, the system will be re-enabled to allow the
 * test mode to trigger again; its just that the data will be in a second, third, fourth etc log file.
 */
unsafe extern "C" fn inMotorTestMode() -> bool {
    static mut resetTime: uint32_t =
        0 as libc::c_int as uint32_t; // add 5 seconds
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 &&
           areMotorsRunning() as libc::c_int != 0 {
        resetTime =
            millis().wrapping_add(5000 as libc::c_int as libc::c_uint);
        return 1 as libc::c_int != 0
    } else {
        // Monitor the duration at minimum
        return millis() < resetTime
    };
}
/* *
 * Fill the current state of the blackbox using values read from the flight controller
 */
unsafe extern "C" fn loadMainState(mut currentTimeUs: timeUs_t) {
    let mut blackboxCurrent: *mut blackboxMainState_t =
        blackboxHistory[0 as libc::c_int as usize];
    (*blackboxCurrent).time = currentTimeUs;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        (*blackboxCurrent).axisPID_P[i as usize] =
            pidData[i as usize].P as int32_t;
        (*blackboxCurrent).axisPID_I[i as usize] =
            pidData[i as usize].I as int32_t;
        (*blackboxCurrent).axisPID_D[i as usize] =
            pidData[i as usize].D as int32_t;
        (*blackboxCurrent).axisPID_F[i as usize] =
            pidData[i as usize].F as int32_t;
        (*blackboxCurrent).gyroADC[i as usize] =
            lrintf(gyro.gyroADCf[i as usize]) as int16_t;
        (*blackboxCurrent).accADC[i as usize] =
            lrintf(acc.accADC[i as usize]) as int16_t;
        (*blackboxCurrent).magADC[i as usize] =
            mag.magADC[i as usize] as int16_t;
        i += 1
    }
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < 4 as libc::c_int {
        (*blackboxCurrent).rcCommand[i_0 as usize] =
            rcCommand[i_0 as usize] as int16_t;
        i_0 += 1
    }
    let mut i_1: libc::c_int = 0 as libc::c_int;
    while i_1 < 4 as libc::c_int {
        (*blackboxCurrent).debug[i_1 as usize] = debug[i_1 as usize];
        i_1 += 1
    }
    let motorCount: libc::c_int = getMotorCount() as libc::c_int;
    let mut i_2: libc::c_int = 0 as libc::c_int;
    while i_2 < motorCount {
        (*blackboxCurrent).motor[i_2 as usize] =
            motor[i_2 as usize] as int16_t;
        i_2 += 1
    }
    (*blackboxCurrent).vbatLatest = getBatteryVoltageLatest();
    (*blackboxCurrent).amperageLatest = getAmperageLatest();
    (*blackboxCurrent).BaroAlt = baro.BaroAlt;
    // Store the raw sonar value without applying tilt correction
    (*blackboxCurrent).surfaceRaw = rangefinderGetLatestAltitude();
    (*blackboxCurrent).rssi = getRssi();
    //Tail servo for tricopters
    (*blackboxCurrent).servo[5 as libc::c_int as usize] =
        servo[5 as libc::c_int as usize];
    // UNIT_TEST
}
/* *
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * For all header types, provide a "mainFrameChar" which is the name for the field and will be used to refer to it in the
 * header (e.g. P, I etc). For blackboxDeltaField_t fields, also provide deltaFrameChar, otherwise set this to zero.
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
unsafe extern "C" fn sendFieldDefinition(mut mainFrameChar: libc::c_char,
                                         mut deltaFrameChar: libc::c_char,
                                         mut fieldDefinitions:
                                             *const libc::c_void,
                                         mut secondFieldDefinition:
                                             *const libc::c_void,
                                         mut fieldCount: libc::c_int,
                                         mut conditions: *const uint8_t,
                                         mut secondCondition: *const uint8_t)
 -> bool {
    let mut def: *const blackboxFieldDefinition_t =
        0 as *const blackboxFieldDefinition_t;
    let mut headerCount: libc::c_uint = 0;
    static mut needComma: bool = 0 as libc::c_int != 0;
    let mut definitionStride: size_t =
        (secondFieldDefinition as
             *mut libc::c_char).wrapping_offset_from(fieldDefinitions as
                                                         *mut libc::c_char) as
            libc::c_long as size_t;
    let mut conditionsStride: size_t =
        (secondCondition as
             *mut libc::c_char).wrapping_offset_from(conditions as
                                                         *mut libc::c_char) as
            libc::c_long as size_t;
    if deltaFrameChar != 0 {
        headerCount =
            (::core::mem::size_of::<[*const libc::c_char; 6]>() as
                 libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                 as libc::c_ulong) as
                libc::c_uint
    } else {
        headerCount =
            (::core::mem::size_of::<[*const libc::c_char; 6]>() as
                 libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                 as
                                                 libc::c_ulong).wrapping_sub(2
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_ulong)
                as libc::c_uint
    }
    /*
     * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
     * the whole header.
     */
    // On our first call we need to print the name of the header and a colon
    if xmitState.u.fieldIndex == -(1 as libc::c_int) {
        if xmitState.headerIndex >= headerCount {
            return 0 as libc::c_int != 0
            //Someone probably called us again after we had already completed transmission
        }
        let mut charsToBeWritten: uint32_t =
            strlen(b"H Field x :\x00" as *const u8 as
                       *const libc::c_char).wrapping_add(strlen(blackboxFieldHeaderNames[xmitState.headerIndex
                                                                                             as
                                                                                             usize]))
                as uint32_t;
        if blackboxDeviceReserveBufferSpace(charsToBeWritten as int32_t) as
               libc::c_uint !=
               BLACKBOX_RESERVE_SUCCESS as libc::c_int as libc::c_uint {
            return 1 as libc::c_int != 0
            // Try again later
        }
        blackboxHeaderBudget -=
            blackboxPrintf(b"H Field %c %s:\x00" as *const u8 as
                               *const libc::c_char,
                           if xmitState.headerIndex as libc::c_ulong >=
                                  (::core::mem::size_of::<[*const libc::c_char; 6]>()
                                       as
                                       libc::c_ulong).wrapping_div(::core::mem::size_of::<*const libc::c_char>()
                                                                       as
                                                                       libc::c_ulong).wrapping_sub(2
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_ulong)
                              {
                               deltaFrameChar as libc::c_int
                           } else { mainFrameChar as libc::c_int },
                           blackboxFieldHeaderNames[xmitState.headerIndex as
                                                        usize]);
        xmitState.u.fieldIndex += 1;
        needComma = 0 as libc::c_int != 0
    }
    // The longest we expect an integer to be as a string:
    let LONGEST_INTEGER_STRLEN: uint32_t = 2 as libc::c_int as uint32_t;
    while xmitState.u.fieldIndex < fieldCount {
        def =
            (fieldDefinitions as
                 *const libc::c_char).offset(definitionStride.wrapping_mul(xmitState.u.fieldIndex
                                                                               as
                                                                               libc::c_ulong)
                                                 as isize) as
                *const blackboxFieldDefinition_t;
        if conditions.is_null() ||
               testBlackboxCondition(*conditions.offset(conditionsStride.wrapping_mul(xmitState.u.fieldIndex
                                                                                          as
                                                                                          libc::c_ulong)
                                                            as isize) as
                                         FlightLogFieldCondition) as
                   libc::c_int != 0 {
            // First (over)estimate the length of the string we want to print
            let mut bytesToWrite: int32_t = 1 as libc::c_int; // Leading comma
            // The first header is a field name
            if xmitState.headerIndex == 0 as libc::c_int as libc::c_uint {
                bytesToWrite =
                    (bytesToWrite as
                         libc::c_ulong).wrapping_add(strlen((*def).name).wrapping_add(strlen(b"[]\x00"
                                                                                                 as
                                                                                                 *const u8
                                                                                                 as
                                                                                                 *const libc::c_char)).wrapping_add(LONGEST_INTEGER_STRLEN
                                                                                                                                        as
                                                                                                                                        libc::c_ulong))
                        as int32_t as int32_t
            } else {
                //The other headers are integers
                bytesToWrite =
                    (bytesToWrite as
                         libc::c_uint).wrapping_add(LONGEST_INTEGER_STRLEN) as
                        int32_t as int32_t
            }
            // Now perform the write if the buffer is large enough
            if blackboxDeviceReserveBufferSpace(bytesToWrite) as libc::c_uint
                   != BLACKBOX_RESERVE_SUCCESS as libc::c_int as libc::c_uint
               {
                // Ran out of space!
                return 1 as libc::c_int != 0
            }
            blackboxHeaderBudget -= bytesToWrite;
            if needComma {
                blackboxWrite(',' as i32 as uint8_t);
            } else { needComma = 1 as libc::c_int != 0 }
            // The first header is a field name
            if xmitState.headerIndex == 0 as libc::c_int as libc::c_uint {
                blackboxWriteString((*def).name);
                // Do we need to print an index in brackets after the name?
                if (*def).fieldNameIndex as libc::c_int != -(1 as libc::c_int)
                   {
                    blackboxPrintf(b"[%d]\x00" as *const u8 as
                                       *const libc::c_char,
                                   (*def).fieldNameIndex as libc::c_int);
                }
            } else {
                //The other headers are integers
                blackboxPrintf(b"%d\x00" as *const u8 as *const libc::c_char,
                               *(*def).arr.as_ptr().offset(xmitState.headerIndex.wrapping_sub(1
                                                                                                  as
                                                                                                  libc::c_int
                                                                                                  as
                                                                                                  libc::c_uint)
                                                               as isize) as
                                   libc::c_int);
            }
        }
        xmitState.u.fieldIndex += 1
    }
    // Did we complete this line?
    if xmitState.u.fieldIndex == fieldCount &&
           blackboxDeviceReserveBufferSpace(1 as libc::c_int) as libc::c_uint
               == BLACKBOX_RESERVE_SUCCESS as libc::c_int as libc::c_uint {
        blackboxHeaderBudget -= 1;
        blackboxWrite('\n' as i32 as uint8_t);
        xmitState.headerIndex = xmitState.headerIndex.wrapping_add(1);
        xmitState.u.fieldIndex = -(1 as libc::c_int)
    }
    return xmitState.headerIndex < headerCount;
}
// Buf must be at least FORMATTED_DATE_TIME_BUFSIZE
unsafe extern "C" fn blackboxGetStartDateTime(mut buf: *mut libc::c_char)
 -> *mut libc::c_char {
    let mut dt: dateTime_t =
        dateTime_t{year: 0,
                   month: 0,
                   day: 0,
                   hours: 0,
                   minutes: 0,
                   seconds: 0,
                   millis: 0,};
    // rtcGetDateTime will fill dt with 0000-01-01T00:00:00
    // when time is not known.
    rtcGetDateTime(&mut dt);
    dateTimeFormatLocal(buf, &mut dt);
    return buf;
}
/* *
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
unsafe extern "C" fn blackboxWriteSysinfo() -> bool {
    let motorOutputLowInt: uint16_t = lrintf(motorOutputLow) as uint16_t;
    let motorOutputHighInt: uint16_t = lrintf(motorOutputHigh) as uint16_t;
    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    if blackboxDeviceReserveBufferSpace(64 as libc::c_int) as libc::c_uint !=
           BLACKBOX_RESERVE_SUCCESS as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    let mut buf: [libc::c_char; 30] = [0; 30];
    let mut currentControlRateProfile: *const controlRateConfig_t =
        controlRateProfiles((*systemConfig()).activeRateProfile as
                                libc::c_int);
    match xmitState.headerIndex {
        0 => {
            blackboxPrintfHeaderLine(b"Firmware type\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%s\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"Cleanflight\x00" as *const u8 as
                                         *const libc::c_char);
        }
        1 => {
            blackboxPrintfHeaderLine(b"Firmware revision\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%s %s (%s) %s\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"Cleanflight\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"2.5.0\x00" as *const u8 as
                                         *const libc::c_char,
                                     shortGitRevision, targetName);
        }
        2 => {
            blackboxPrintfHeaderLine(b"Firmware date\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%s %s\x00" as *const u8 as
                                         *const libc::c_char, buildDate,
                                     buildTime);
        }
        3 => {
            blackboxPrintfHeaderLine(b"Log start datetime\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%s\x00" as *const u8 as
                                         *const libc::c_char,
                                     blackboxGetStartDateTime(buf.as_mut_ptr()));
        }
        4 => {
            blackboxPrintfHeaderLine(b"Craft name\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%s\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*pilotConfig()).name.as_ptr());
        }
        5 => {
            blackboxPrintfHeaderLine(b"I interval\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     blackboxIInterval as libc::c_int);
        }
        6 => {
            blackboxPrintfHeaderLine(b"P interval\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     blackboxPInterval as libc::c_int);
        }
        7 => {
            blackboxPrintfHeaderLine(b"P ratio\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*blackboxConfig()).p_ratio as
                                         libc::c_int);
        }
        8 => {
            blackboxPrintfHeaderLine(b"minthrottle\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*motorConfig()).minthrottle as
                                         libc::c_int);
        }
        9 => {
            blackboxPrintfHeaderLine(b"maxthrottle\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*motorConfig()).maxthrottle as
                                         libc::c_int);
        }
        10 => {
            blackboxPrintfHeaderLine(b"gyro_scale\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"0x%x\x00" as *const u8 as
                                         *const libc::c_char,
                                     castFloatBytesToInt(1.0f32));
        }
        11 => {
            blackboxPrintfHeaderLine(b"motorOutput\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     motorOutputLowInt as libc::c_int,
                                     motorOutputHighInt as libc::c_int);
        }
        12 => {
            blackboxPrintfHeaderLine(b"acc_1G\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%u\x00" as *const u8 as
                                         *const libc::c_char,
                                     acc.dev.acc_1G as libc::c_int);
        }
        13 => {
            if testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT) {
                blackboxPrintfHeaderLine(b"vbat_scale\x00" as *const u8 as
                                             *const libc::c_char,
                                         b"%u\x00" as *const u8 as
                                             *const libc::c_char,
                                         (*voltageSensorADCConfig(VOLTAGE_SENSOR_ADC_VBAT
                                                                      as
                                                                      libc::c_int)).vbatscale
                                             as libc::c_int);
            } else {
                xmitState.headerIndex =
                    (xmitState.headerIndex as
                         libc::c_uint).wrapping_add(2 as libc::c_int as
                                                        libc::c_uint) as
                        uint32_t as uint32_t
                // Skip the next two vbat fields too
            }
        }
        14 => {
            blackboxPrintfHeaderLine(b"vbatcellvoltage\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%u,%u,%u\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*batteryConfig()).vbatmincellvoltage as
                                         libc::c_int,
                                     (*batteryConfig()).vbatwarningcellvoltage
                                         as libc::c_int,
                                     (*batteryConfig()).vbatmaxcellvoltage as
                                         libc::c_int);
        }
        15 => {
            blackboxPrintfHeaderLine(b"vbatref\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%u\x00" as *const u8 as
                                         *const libc::c_char,
                                     vbatReference as libc::c_int);
        }
        16 => {
            if (*batteryConfig()).currentMeterSource as libc::c_uint ==
                   CURRENT_METER_ADC as libc::c_int as libc::c_uint {
                blackboxPrintfHeaderLine(b"currentSensor\x00" as *const u8 as
                                             *const libc::c_char,
                                         b"%d,%d\x00" as *const u8 as
                                             *const libc::c_char,
                                         (*currentSensorADCConfig()).offset as
                                             libc::c_int,
                                         (*currentSensorADCConfig()).scale as
                                             libc::c_int);
            }
        }
        17 => {
            blackboxPrintfHeaderLine(b"looptime\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     gyro.targetLooptime);
        }
        18 => {
            blackboxPrintfHeaderLine(b"gyro_sync_denom\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*gyroConfig()).gyro_sync_denom as
                                         libc::c_int);
        }
        19 => {
            blackboxPrintfHeaderLine(b"pid_process_denom\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*pidConfig()).pid_process_denom as
                                         libc::c_int);
        }
        20 => {
            blackboxPrintfHeaderLine(b"thr_mid\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentControlRateProfile).thrMid8 as
                                         libc::c_int);
        }
        21 => {
            blackboxPrintfHeaderLine(b"thr_expo\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentControlRateProfile).thrExpo8 as
                                         libc::c_int);
        }
        22 => {
            blackboxPrintfHeaderLine(b"tpa_rate\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentControlRateProfile).dynThrPID as
                                         libc::c_int);
        }
        23 => {
            blackboxPrintfHeaderLine(b"tpa_breakpoint\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentControlRateProfile).tpa_breakpoint
                                         as libc::c_int);
        }
        24 => {
            blackboxPrintfHeaderLine(b"rc_rates\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentControlRateProfile).rcRates[ROLL
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              usize]
                                         as libc::c_int,
                                     (*currentControlRateProfile).rcRates[PITCH
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              usize]
                                         as libc::c_int,
                                     (*currentControlRateProfile).rcRates[YAW
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              usize]
                                         as libc::c_int);
        }
        25 => {
            blackboxPrintfHeaderLine(b"rc_expo\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentControlRateProfile).rcExpo[ROLL
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             usize]
                                         as libc::c_int,
                                     (*currentControlRateProfile).rcExpo[PITCH
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             usize]
                                         as libc::c_int,
                                     (*currentControlRateProfile).rcExpo[YAW
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             usize]
                                         as libc::c_int);
        }
        26 => {
            blackboxPrintfHeaderLine(b"rates\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentControlRateProfile).rates[ROLL
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            usize]
                                         as libc::c_int,
                                     (*currentControlRateProfile).rates[PITCH
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            usize]
                                         as libc::c_int,
                                     (*currentControlRateProfile).rates[YAW as
                                                                            libc::c_int
                                                                            as
                                                                            usize]
                                         as libc::c_int);
        }
        27 => {
            blackboxPrintfHeaderLine(b"rollPID\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pid[PID_ROLL as
                                                                  libc::c_int
                                                                  as usize].P
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_ROLL as
                                                                  libc::c_int
                                                                  as usize].I
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_ROLL as
                                                                  libc::c_int
                                                                  as usize].D
                                         as libc::c_int);
        }
        28 => {
            blackboxPrintfHeaderLine(b"pitchPID\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pid[PID_PITCH as
                                                                  libc::c_int
                                                                  as usize].P
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_PITCH as
                                                                  libc::c_int
                                                                  as usize].I
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_PITCH as
                                                                  libc::c_int
                                                                  as usize].D
                                         as libc::c_int);
        }
        29 => {
            blackboxPrintfHeaderLine(b"yawPID\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pid[PID_YAW as
                                                                  libc::c_int
                                                                  as usize].P
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_YAW as
                                                                  libc::c_int
                                                                  as usize].I
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_YAW as
                                                                  libc::c_int
                                                                  as usize].D
                                         as libc::c_int);
        }
        30 => {
            blackboxPrintfHeaderLine(b"levelPID\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pid[PID_LEVEL as
                                                                  libc::c_int
                                                                  as usize].P
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_LEVEL as
                                                                  libc::c_int
                                                                  as usize].I
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_LEVEL as
                                                                  libc::c_int
                                                                  as usize].D
                                         as libc::c_int);
        }
        31 => {
            blackboxPrintfHeaderLine(b"magPID\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pid[PID_MAG as
                                                                  libc::c_int
                                                                  as usize].P
                                         as libc::c_int);
        }
        32 => {
            blackboxPrintfHeaderLine(b"dterm_filter_type\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).dterm_filter_type as
                                         libc::c_int);
        }
        33 => {
            blackboxPrintfHeaderLine(b"dterm_lowpass_hz\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).dterm_lowpass_hz as
                                         libc::c_int);
        }
        34 => {
            blackboxPrintfHeaderLine(b"dterm_lowpass2_hz\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).dterm_lowpass2_hz as
                                         libc::c_int);
        }
        35 => {
            blackboxPrintfHeaderLine(b"yaw_lowpass_hz\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).yaw_lowpass_hz as
                                         libc::c_int);
        }
        36 => {
            blackboxPrintfHeaderLine(b"dterm_notch_hz\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).dterm_notch_hz as
                                         libc::c_int);
        }
        37 => {
            blackboxPrintfHeaderLine(b"dterm_notch_cutoff\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).dterm_notch_cutoff
                                         as libc::c_int);
        }
        38 => {
            blackboxPrintfHeaderLine(b"iterm_windup\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).itermWindupPointPercent
                                         as libc::c_int);
        }
        39 => {
            blackboxPrintfHeaderLine(b"vbat_pid_gain\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).vbatPidCompensation
                                         as libc::c_int);
        }
        40 => {
            blackboxPrintfHeaderLine(b"pidAtMinThrottle\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pidAtMinThrottle as
                                         libc::c_int);
        }
        41 => {
            // Betaflight PID controller parameters
            blackboxPrintfHeaderLine(b"anti_gravity_mode\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).antiGravityMode as
                                         libc::c_int);
        }
        42 => {
            blackboxPrintfHeaderLine(b"anti_gravity_threshold\x00" as
                                         *const u8 as *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).itermThrottleThreshold
                                         as libc::c_int);
        }
        43 => {
            blackboxPrintfHeaderLine(b"anti_gravity_gain\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).itermAcceleratorGain
                                         as libc::c_int);
        }
        44 => {
            blackboxPrintfHeaderLine(b"feedforward_transition\x00" as
                                         *const u8 as *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).feedForwardTransition
                                         as libc::c_int);
        }
        45 => {
            blackboxPrintfHeaderLine(b"feedforward_weight\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pid[PID_ROLL as
                                                                  libc::c_int
                                                                  as usize].F
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_PITCH as
                                                                  libc::c_int
                                                                  as usize].F
                                         as libc::c_int,
                                     (*currentPidProfile).pid[PID_YAW as
                                                                  libc::c_int
                                                                  as usize].F
                                         as libc::c_int);
        }
        46 => {
            blackboxPrintfHeaderLine(b"acc_limit_yaw\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).yawRateAccelLimit as
                                         libc::c_int);
        }
        47 => {
            blackboxPrintfHeaderLine(b"acc_limit\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).rateAccelLimit as
                                         libc::c_int);
        }
        48 => {
            blackboxPrintfHeaderLine(b"pidsum_limit\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pidSumLimit as
                                         libc::c_int);
        }
        49 => {
            blackboxPrintfHeaderLine(b"pidsum_limit_yaw\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*currentPidProfile).pidSumLimitYaw as
                                         libc::c_int);
        }
        50 => {
            // End of Betaflight controller parameters
            blackboxPrintfHeaderLine(b"deadband\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rcControlsConfig()).deadband as
                                         libc::c_int);
        }
        51 => {
            blackboxPrintfHeaderLine(b"yaw_deadband\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rcControlsConfig()).yaw_deadband as
                                         libc::c_int);
        }
        52 => {
            blackboxPrintfHeaderLine(b"gyro_hardware_lpf\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*gyroConfig()).gyro_hardware_lpf as
                                         libc::c_int);
        }
        53 => {
            blackboxPrintfHeaderLine(b"gyro_lowpass_type\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*gyroConfig()).gyro_lowpass_type as
                                         libc::c_int);
        }
        54 => {
            blackboxPrintfHeaderLine(b"gyro_lowpass_hz\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*gyroConfig()).gyro_lowpass_hz as
                                         libc::c_int);
        }
        55 => {
            blackboxPrintfHeaderLine(b"gyro_lowpass2_type\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*gyroConfig()).gyro_lowpass2_type as
                                         libc::c_int);
        }
        56 => {
            blackboxPrintfHeaderLine(b"gyro_lowpass2_hz\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*gyroConfig()).gyro_lowpass2_hz as
                                         libc::c_int);
        }
        57 => {
            blackboxPrintfHeaderLine(b"gyro_notch_hz\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*gyroConfig()).gyro_soft_notch_hz_1 as
                                         libc::c_int,
                                     (*gyroConfig()).gyro_soft_notch_hz_2 as
                                         libc::c_int);
        }
        58 => {
            blackboxPrintfHeaderLine(b"gyro_notch_cutoff\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d,%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*gyroConfig()).gyro_soft_notch_cutoff_1
                                         as libc::c_int,
                                     (*gyroConfig()).gyro_soft_notch_cutoff_2
                                         as libc::c_int);
        }
        59 => {
            blackboxPrintfHeaderLine(b"acc_lpf_hz\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     ((*accelerometerConfig()).acc_lpf_hz as
                                          libc::c_int as libc::c_float *
                                          100.0f32) as libc::c_int);
        }
        60 => {
            blackboxPrintfHeaderLine(b"acc_hardware\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*accelerometerConfig()).acc_hardware as
                                         libc::c_int);
        }
        61 => {
            blackboxPrintfHeaderLine(b"baro_hardware\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*barometerConfig()).baro_hardware as
                                         libc::c_int);
        }
        62 => {
            blackboxPrintfHeaderLine(b"mag_hardware\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*compassConfig()).mag_hardware as
                                         libc::c_int);
        }
        63 => {
            blackboxPrintfHeaderLine(b"gyro_cal_on_first_arm\x00" as *const u8
                                         as *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*armingConfig()).gyro_cal_on_first_arm
                                         as libc::c_int);
        }
        64 => {
            blackboxPrintfHeaderLine(b"rc_interpolation\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rxConfig()).rcInterpolation as
                                         libc::c_int);
        }
        65 => {
            blackboxPrintfHeaderLine(b"rc_interpolation_interval\x00" as
                                         *const u8 as *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rxConfig()).rcInterpolationInterval as
                                         libc::c_int);
        }
        66 => {
            blackboxPrintfHeaderLine(b"airmode_activate_throttle\x00" as
                                         *const u8 as *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rxConfig()).airModeActivateThreshold as
                                         libc::c_int);
        }
        67 => {
            blackboxPrintfHeaderLine(b"serialrx_provider\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rxConfig()).serialrx_provider as
                                         libc::c_int);
        }
        68 => {
            blackboxPrintfHeaderLine(b"use_unsynced_pwm\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*motorConfig()).dev.useUnsyncedPwm as
                                         libc::c_int);
        }
        69 => {
            blackboxPrintfHeaderLine(b"motor_pwm_protocol\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*motorConfig()).dev.motorPwmProtocol as
                                         libc::c_int);
        }
        70 => {
            blackboxPrintfHeaderLine(b"motor_pwm_rate\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*motorConfig()).dev.motorPwmRate as
                                         libc::c_int);
        }
        71 => {
            blackboxPrintfHeaderLine(b"dshot_idle_value\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*motorConfig()).digitalIdleOffsetValue
                                         as libc::c_int);
        }
        72 => {
            blackboxPrintfHeaderLine(b"debug_mode\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*systemConfig()).debug_mode as
                                         libc::c_int);
        }
        73 => {
            blackboxPrintfHeaderLine(b"features\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*featureConfig()).enabledFeatures);
        }
        74 => {
            blackboxPrintfHeaderLine(b"rc_smoothing_type\x00" as *const u8 as
                                         *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rxConfig()).rc_smoothing_type as
                                         libc::c_int);
        }
        75 => {
            blackboxPrintfHeaderLine(b"rc_smoothing_debug_axis\x00" as
                                         *const u8 as *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rxConfig()).rc_smoothing_debug_axis as
                                         libc::c_int);
        }
        76 => {
            blackboxPrintfHeaderLine(b"rc_smoothing_cutoffs\x00" as *const u8
                                         as *const libc::c_char,
                                     b"%d, %d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rxConfig()).rc_smoothing_input_cutoff
                                         as libc::c_int,
                                     (*rxConfig()).rc_smoothing_derivative_cutoff
                                         as libc::c_int);
        }
        77 => {
            blackboxPrintfHeaderLine(b"rc_smoothing_filter_type\x00" as
                                         *const u8 as *const libc::c_char,
                                     b"%d, %d\x00" as *const u8 as
                                         *const libc::c_char,
                                     (*rxConfig()).rc_smoothing_input_type as
                                         libc::c_int,
                                     (*rxConfig()).rc_smoothing_derivative_type
                                         as libc::c_int);
        }
        78 => {
            blackboxPrintfHeaderLine(b"rc_smoothing_active_cutoffs\x00" as
                                         *const u8 as *const libc::c_char,
                                     b"%d, %d\x00" as *const u8 as
                                         *const libc::c_char,
                                     rcSmoothingGetValue(RC_SMOOTHING_VALUE_INPUT_ACTIVE
                                                             as libc::c_int),
                                     rcSmoothingGetValue(RC_SMOOTHING_VALUE_DERIVATIVE_ACTIVE
                                                             as libc::c_int));
        }
        79 => {
            blackboxPrintfHeaderLine(b"rc_smoothing_rx_average\x00" as
                                         *const u8 as *const libc::c_char,
                                     b"%d\x00" as *const u8 as
                                         *const libc::c_char,
                                     rcSmoothingGetValue(RC_SMOOTHING_VALUE_AVERAGE_FRAME
                                                             as libc::c_int));
        }
        _ => {
            // USE_RC_SMOOTHING_FILTER
            return 1 as libc::c_int != 0
        }
    }
    xmitState.headerIndex = xmitState.headerIndex.wrapping_add(1);
    // UNIT_TEST
    return 0 as libc::c_int != 0;
}
/* *
 * Write the given event to the log immediately
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxLogEvent(mut event: FlightLogEvent,
                                          mut data:
                                              *mut flightLogEventData_t) {
    // Only allow events to be logged after headers have been written
    if !(blackboxState as libc::c_uint ==
             BLACKBOX_STATE_RUNNING as libc::c_int as libc::c_uint ||
             blackboxState as libc::c_uint ==
                 BLACKBOX_STATE_PAUSED as libc::c_int as libc::c_uint) {
        return
    }
    //Shared header for event frames
    blackboxWrite('E' as i32 as uint8_t);
    blackboxWrite(event as uint8_t);
    //Now serialize the data for this specific frame type
    match event as libc::c_uint {
        0 => { blackboxWriteUnsignedVB((*data).syncBeep.time); }
        30 => {
            // New flightmode flags write
            blackboxWriteUnsignedVB((*data).flightMode.flags);
            blackboxWriteUnsignedVB((*data).flightMode.lastFlags);
        }
        13 => {
            if (*data).inflightAdjustment.floatFlag {
                blackboxWrite(((*data).inflightAdjustment.adjustmentFunction
                                   as libc::c_int + 128 as libc::c_int) as
                                  uint8_t);
                blackboxWriteFloat((*data).inflightAdjustment.newFloatValue);
            } else {
                blackboxWrite((*data).inflightAdjustment.adjustmentFunction);
                blackboxWriteSignedVB((*data).inflightAdjustment.newValue);
            }
        }
        14 => {
            blackboxWriteUnsignedVB((*data).loggingResume.logIteration);
            blackboxWriteUnsignedVB((*data).loggingResume.currentTime);
        }
        255 => {
            blackboxWriteString(b"End of log\x00" as *const u8 as
                                    *const libc::c_char);
            blackboxWrite(0 as libc::c_int as uint8_t);
        }
        _ => { }
    };
}
/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
unsafe extern "C" fn blackboxCheckAndLogArmingBeep() {
    // Use != so that we can still detect a change if the counter wraps
    if getArmingBeepTimeMicros() != blackboxLastArmingBeep {
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        let mut eventData: flightLogEvent_syncBeep_t =
            flightLogEvent_syncBeep_t{time: 0,};
        eventData.time = blackboxLastArmingBeep;
        blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP,
                         &mut eventData as *mut flightLogEvent_syncBeep_t as
                             *mut flightLogEventData_t);
    };
}
/* monitor the flight mode event status and trigger an event record if the state changes */
unsafe extern "C" fn blackboxCheckAndLogFlightMode() {
    // Use != so that we can still detect a change if the counter wraps
    if memcmp(&mut rcModeActivationMask as *mut boxBitmask_t as
                  *const libc::c_void,
              &mut blackboxLastFlightModeFlags as *mut uint32_t as
                  *const libc::c_void,
              ::core::mem::size_of::<uint32_t>() as libc::c_ulong) != 0 {
        let mut eventData: flightLogEvent_flightMode_t =
            flightLogEvent_flightMode_t{flags: 0,
                                        lastFlags:
                                            0,}; // Add new data for current flight mode flags
        eventData.lastFlags = blackboxLastFlightModeFlags;
        memcpy(&mut blackboxLastFlightModeFlags as *mut uint32_t as
                   *mut libc::c_void,
               &mut rcModeActivationMask as *mut boxBitmask_t as
                   *const libc::c_void,
               ::core::mem::size_of::<uint32_t>() as libc::c_ulong);
        memcpy(&mut eventData.flags as *mut uint32_t as *mut libc::c_void,
               &mut rcModeActivationMask as *mut boxBitmask_t as
                   *const libc::c_void,
               ::core::mem::size_of::<uint32_t>() as libc::c_ulong);
        blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE,
                         &mut eventData as *mut flightLogEvent_flightMode_t as
                             *mut flightLogEventData_t);
    };
}
unsafe extern "C" fn blackboxShouldLogPFrame() -> bool {
    return blackboxPFrameIndex as libc::c_int == 0 as libc::c_int &&
               (*blackboxConfig()).p_ratio as libc::c_int != 0 as libc::c_int;
}
unsafe extern "C" fn blackboxShouldLogIFrame() -> bool {
    return blackboxLoopIndex as libc::c_int == 0 as libc::c_int;
}
/*
 * If the GPS home point has been updated, or every 128 I-frames (~10 seconds), write the
 * GPS home position.
 *
 * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
 * still be interpreted correctly.
 */
// GPS
// Called once every FC loop in order to keep track of how many FC loop iterations have passed
unsafe extern "C" fn blackboxAdvanceIterationTimers() {
    blackboxSlowFrameIterationTimer += 1;
    blackboxIteration = blackboxIteration.wrapping_add(1);
    blackboxLoopIndex = blackboxLoopIndex.wrapping_add(1);
    if blackboxLoopIndex as libc::c_int >= blackboxIInterval as libc::c_int {
        blackboxLoopIndex = 0 as libc::c_int as uint16_t;
        blackboxIFrameIndex = blackboxIFrameIndex.wrapping_add(1);
        blackboxPFrameIndex = 0 as libc::c_int as uint16_t
    } else {
        blackboxPFrameIndex = blackboxPFrameIndex.wrapping_add(1);
        if blackboxPFrameIndex as libc::c_int >=
               blackboxPInterval as libc::c_int {
            blackboxPFrameIndex = 0 as libc::c_int as uint16_t
        }
    };
}
// Called once every FC loop in order to log the current state
unsafe extern "C" fn blackboxLogIteration(mut currentTimeUs: timeUs_t) {
    // Write a keyframe every blackboxIInterval frames so we can resynchronise upon missing frames
    if blackboxShouldLogIFrame() {
        /*
         * Don't log a slow frame if the slow data didn't change ("I" frames are already large enough without adding
         * an additional item to write at the same time). Unless we're *only* logging "I" frames, then we have no choice.
         */
        if blackboxIsOnlyLoggingIntraframes() {
            writeSlowFrameIfNeeded(); // Check for FlightMode status change event
        }
        loadMainState(currentTimeUs);
        writeIntraframe();
    } else {
        blackboxCheckAndLogArmingBeep();
        blackboxCheckAndLogFlightMode();
        if blackboxShouldLogPFrame() {
            /*
             * We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
             * So only log slow frames during loop iterations where we log a main frame.
             */
            writeSlowFrameIfNeeded();
            loadMainState(currentTimeUs);
            writeInterframe();
        }
    }
    //Flush every iteration so that our runtime variance is minimized
    blackboxDeviceFlush();
}
/* *
 * Call each flight loop iteration to perform blackbox logging.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxUpdate(mut currentTimeUs: timeUs_t) {
    match blackboxState as libc::c_uint {
        1 => {
            if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                blackboxOpen();
                blackboxStart();
            }
        }
        2 => {
            if blackboxDeviceBeginLog() {
                blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
            }
        }
        3 => {
            blackboxReplenishHeaderBudget();
            //On entry of this state, xmitState.headerIndex is 0 and startTime is intialised
            /*
         * Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit
         * buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
         */
            if millis() >
                   xmitState.u.startTime.wrapping_add(100 as libc::c_int as
                                                          libc::c_uint) {
                if blackboxDeviceReserveBufferSpace(64 as libc::c_int) as
                       libc::c_uint ==
                       BLACKBOX_RESERVE_SUCCESS as libc::c_int as libc::c_uint
                   {
                    let mut i: libc::c_int = 0 as libc::c_int;
                    while i < 64 as libc::c_int &&
                              blackboxHeader[xmitState.headerIndex as usize]
                                  as libc::c_int != '\u{0}' as i32 {
                        blackboxWrite(blackboxHeader[xmitState.headerIndex as
                                                         usize] as uint8_t);
                        blackboxHeaderBudget -= 1;
                        i += 1;
                        xmitState.headerIndex =
                            xmitState.headerIndex.wrapping_add(1)
                    }
                    if blackboxHeader[xmitState.headerIndex as usize] as
                           libc::c_int == '\u{0}' as i32 {
                        blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
                    }
                }
            }
        }
        4 => {
            blackboxReplenishHeaderBudget();
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if !sendFieldDefinition('I' as i32 as libc::c_char,
                                    'P' as i32 as libc::c_char,
                                    blackboxMainFields.as_ptr() as
                                        *const libc::c_void,
                                    blackboxMainFields.as_ptr().offset(1 as
                                                                           libc::c_int
                                                                           as
                                                                           isize)
                                        as *const libc::c_void,
                                    (::core::mem::size_of::<[blackboxDeltaFieldDefinition_t; 45]>()
                                         as
                                         libc::c_ulong).wrapping_div(::core::mem::size_of::<blackboxDeltaFieldDefinition_t>()
                                                                         as
                                                                         libc::c_ulong)
                                        as libc::c_int,
                                    &(*blackboxMainFields.as_ptr().offset(0 as
                                                                              libc::c_int
                                                                              as
                                                                              isize)).condition,
                                    &(*blackboxMainFields.as_ptr().offset(1 as
                                                                              libc::c_int
                                                                              as
                                                                              isize)).condition)
               {
                blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
            }
        }
        7 => {
            blackboxReplenishHeaderBudget();
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if !sendFieldDefinition('S' as i32 as libc::c_char,
                                    0 as libc::c_int as libc::c_char,
                                    blackboxSlowFields.as_ptr() as
                                        *const libc::c_void,
                                    blackboxSlowFields.as_ptr().offset(1 as
                                                                           libc::c_int
                                                                           as
                                                                           isize)
                                        as *const libc::c_void,
                                    (::core::mem::size_of::<[blackboxSimpleFieldDefinition_t; 5]>()
                                         as
                                         libc::c_ulong).wrapping_div(::core::mem::size_of::<blackboxSimpleFieldDefinition_t>()
                                                                         as
                                                                         libc::c_ulong)
                                        as libc::c_int, 0 as *const uint8_t,
                                    0 as *const uint8_t) {
                blackboxSetState(BLACKBOX_STATE_SEND_SYSINFO);
            }
        }
        8 => {
            blackboxReplenishHeaderBudget();
            //On entry of this state, xmitState.headerIndex is 0
            //Keep writing chunks of the system info headers until it returns true to signal completion
            if blackboxWriteSysinfo() {
                /*
             * Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
             * (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
             * could wipe out the end of the header if we weren't careful)
             */
                if blackboxDeviceFlushForce() {
                    blackboxSetState(BLACKBOX_STATE_RUNNING);
                }
            }
        }
        9 => {
            // Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
            if IS_RC_MODE_ACTIVE(BOXBLACKBOX) as libc::c_int != 0 &&
                   blackboxShouldLogIFrame() as libc::c_int != 0 {
                // Write a log entry so the decoder is aware that our large time/iteration skip is intended
                let mut resume: flightLogEvent_loggingResume_t =
                    flightLogEvent_loggingResume_t{logIteration: 0,
                                                   currentTime: 0,};
                resume.logIteration = blackboxIteration;
                resume.currentTime = currentTimeUs;
                blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME,
                                 &mut resume as
                                     *mut flightLogEvent_loggingResume_t as
                                     *mut flightLogEventData_t);
                blackboxSetState(BLACKBOX_STATE_RUNNING);
                blackboxLogIteration(currentTimeUs);
            }
            // Keep the logging timers ticking so our log iteration continues to advance
            blackboxAdvanceIterationTimers();
        }
        10 => {
            // On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0
        // Prevent the Pausing of the log on the mode switch if in Motor Test Mode
            if blackboxModeActivationConditionPresent as libc::c_int != 0 &&
                   !IS_RC_MODE_ACTIVE(BOXBLACKBOX) &&
                   !startedLoggingInTestMode {
                blackboxSetState(BLACKBOX_STATE_PAUSED);
            } else { blackboxLogIteration(currentTimeUs); }
            blackboxAdvanceIterationTimers();
        }
        11 => {
            //On entry of this state, startTime is set
        /*
         * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
         * since releasing the port clears the Tx buffer.
         *
         * Don't wait longer than it could possibly take if something funky happens.
         */
            if blackboxDeviceEndLog(blackboxLoggedAnyFrames) as libc::c_int !=
                   0 &&
                   (millis() >
                        xmitState.u.startTime.wrapping_add(200 as libc::c_int
                                                               as
                                                               libc::c_uint)
                        || blackboxDeviceFlushForce() as libc::c_int != 0) {
                blackboxDeviceClose();
                blackboxSetState(BLACKBOX_STATE_STOPPED);
            }
        }
        _ => { }
    }
    // Did we run out of room on the device? Stop!
    if isBlackboxDeviceFull() {
        blackboxSetState(BLACKBOX_STATE_STOPPED); // Only log in test mode if there is room!
        // ensure we reset the test mode flag if we stop due to full memory card
        if startedLoggingInTestMode {
            startedLoggingInTestMode = 0 as libc::c_int != 0
        }
    } else {
        match (*blackboxConfig()).mode as libc::c_int {
            1 => {
                // Handle Motor Test Mode
                if inMotorTestMode() {
                    if blackboxState as libc::c_uint ==
                           BLACKBOX_STATE_STOPPED as libc::c_int as
                               libc::c_uint {
                        startInTestMode();
                    }
                } else if blackboxState as libc::c_uint !=
                              BLACKBOX_STATE_STOPPED as libc::c_int as
                                  libc::c_uint {
                    stopInTestMode();
                }
            }
            2 => {
                if blackboxState as libc::c_uint ==
                       BLACKBOX_STATE_STOPPED as libc::c_int as libc::c_uint {
                    startInTestMode();
                }
            }
            0 | _ => { }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn blackboxCalculatePDenom(mut rateNum: libc::c_int,
                                                 mut rateDenom: libc::c_int)
 -> libc::c_int {
    return blackboxIInterval as libc::c_int * rateNum / rateDenom;
}
#[no_mangle]
pub unsafe extern "C" fn blackboxGetRateDenom() -> uint8_t {
    return blackboxPInterval as uint8_t;
}
/* *
 * Call during system startup to initialize the blackbox.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxInit() {
    blackboxResetIterationTimers();
    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    // targetPidLooptime is 1000 for 1kHz loop, 500 for 2kHz loop etc, targetPidLooptime is rounded for short looptimes
    if targetPidLooptime == 31 as libc::c_int as libc::c_uint {
        // rounded from 31.25us
        blackboxIInterval = 1024 as libc::c_int as int16_t
    } else if targetPidLooptime == 63 as libc::c_int as libc::c_uint {
        // rounded from 62.5us
        blackboxIInterval = 512 as libc::c_int as int16_t
    } else {
        blackboxIInterval =
            ((32 as libc::c_int * 1000 as libc::c_int) as
                 libc::c_uint).wrapping_div(targetPidLooptime) as uint16_t as
                int16_t
    }
    // by default p_ratio is 32 and a P-frame is written every 1ms
    // if p_ratio is zero then no P-frames are logged
    if (*blackboxConfig()).p_ratio as libc::c_int == 0 as libc::c_int {
        blackboxPInterval = 0 as libc::c_int as int16_t
        // blackboxPInterval not used when p_ratio is zero, so just set it to zero
    } else if (*blackboxConfig()).p_ratio as libc::c_int >
                  blackboxIInterval as libc::c_int &&
                  blackboxIInterval as libc::c_int >= 32 as libc::c_int {
        blackboxPInterval = 1 as libc::c_int as int16_t
    } else {
        blackboxPInterval =
            (blackboxIInterval as libc::c_int /
                 (*blackboxConfig()).p_ratio as libc::c_int) as int16_t
    }
    if (*blackboxConfig()).device != 0 {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    } else { blackboxSetState(BLACKBOX_STATE_DISABLED); }
    blackboxSInterval = blackboxIInterval as libc::c_int * 256 as libc::c_int;
    // S-frame is written every 256*32 = 8192ms, approx every 8 seconds
}
