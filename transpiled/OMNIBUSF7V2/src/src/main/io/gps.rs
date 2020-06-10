use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn sqrtf(_: libc::c_float) -> libc::c_float;
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
    fn GPS_coord_to_degrees(coordinateString: *const libc::c_char)
     -> uint32_t;
    #[no_mangle]
    fn cos_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn atan2_approx(y: libc::c_float, x: libc::c_float) -> libc::c_float;
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
    fn ledToggle(led: libc::c_int);
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
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn rtcHasTime() -> bool;
    #[no_mangle]
    fn rtcSet(t: *mut rtcTime_t) -> bool;
    #[no_mangle]
    fn rtcSetDateTime(dt: *mut dateTime_t) -> bool;
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn dashboardUpdate(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn dashboardShowFixedPage(pageId: pageId_e);
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn serialSetBaudRate(instance: *mut serialPort_t, baudRate: uint32_t);
    #[no_mangle]
    fn serialSetMode(instance: *mut serialPort_t, mode: portMode_e);
    #[no_mangle]
    fn isSerialTransmitBufferEmpty(instance: *const serialPort_t) -> bool;
    #[no_mangle]
    fn serialPrint(instance: *mut serialPort_t, str: *const libc::c_char);
    #[no_mangle]
    fn serialGetBaudRate(instance: *mut serialPort_t) -> uint32_t;
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
    // 1
    // 2
    // 4
    // 8
    // 16
    // 32
    // 64
    // 128
    // 512
    // 1024
    // 2048
    // 4096
    // 8192
    // 16384
    // 32768
    // serial port identifiers are now fixed, these values are used by MSP commands.
    //
// runtime
//
    //
// configuration
//
    // not used for all telemetry systems, e.g. HoTT only works at 19200.
    // which byte is used to reboot. Default 'R', could be changed carefully to something else.
    //
// configuration
//
    // !!TODO remove need for this
    //
// runtime
//
    #[no_mangle]
    fn openSerialPort(identifier: serialPortIdentifier_e,
                      function: serialPortFunction_e,
                      rxCallback: serialReceiveCallbackPtr,
                      rxCallbackData: *mut libc::c_void, baudrate: uint32_t,
                      mode: portMode_e, options: portOptions_e)
     -> *mut serialPort_t;
    #[no_mangle]
    static baudRates: [uint32_t; 0];
    //
// msp/cli/bootloader
//
    #[no_mangle]
    fn serialPassthrough(left: *mut serialPort_t, right: *mut serialPort_t,
                         leftC: Option<serialConsumer>,
                         rightC: Option<serialConsumer>);
    #[no_mangle]
    fn waitForSerialPortToFinishTransmitting(serialPort: *mut serialPort_t);
    #[no_mangle]
    fn lookupBaudRateIndex(baudRate: uint32_t) -> baudRate_e;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn sensorsSet(mask: uint32_t);
    #[no_mangle]
    fn sensorsClear(mask: uint32_t);
    #[no_mangle]
    fn updateGPSRescueState();
    #[no_mangle]
    fn rescueNewGpsData();
    #[no_mangle]
    fn gpsRescueIsConfigured() -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __int64_t = libc::c_long;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
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
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
// Milliseconds since Jan 1 1970
pub type rtcTime_t = int64_t;
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
pub type pageId_e = libc::c_uint;
pub const PAGE_COUNT: pageId_e = 9;
pub const PAGE_DEBUG: pageId_e = 8;
pub const PAGE_GPS: pageId_e = 7;
pub const PAGE_TASKS: pageId_e = 6;
pub const PAGE_PROFILE: pageId_e = 5;
pub const PAGE_RX: pageId_e = 4;
pub const PAGE_SENSORS: pageId_e = 3;
pub const PAGE_BATTERY: pageId_e = 2;
pub const PAGE_ARMED: pageId_e = 1;
pub const PAGE_WELCOME: pageId_e = 0;
pub type gpsProvider_e = libc::c_uint;
pub const GPS_UBLOX: gpsProvider_e = 1;
pub const GPS_NMEA: gpsProvider_e = 0;
pub type sbasMode_e = libc::c_uint;
pub const SBAS_GAGAN: sbasMode_e = 4;
pub const SBAS_MSAS: sbasMode_e = 3;
pub const SBAS_WAAS: sbasMode_e = 2;
pub const SBAS_EGNOS: sbasMode_e = 1;
pub const SBAS_AUTO: sbasMode_e = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const GPS_BAUDRATE_9600: C2RustUnnamed_2 = 4;
pub const GPS_BAUDRATE_19200: C2RustUnnamed_2 = 3;
pub const GPS_BAUDRATE_38400: C2RustUnnamed_2 = 2;
pub const GPS_BAUDRATE_57600: C2RustUnnamed_2 = 1;
pub const GPS_BAUDRATE_115200: C2RustUnnamed_2 = 0;
pub type gpsAutoConfig_e = libc::c_uint;
pub const GPS_AUTOCONFIG_ON: gpsAutoConfig_e = 1;
pub const GPS_AUTOCONFIG_OFF: gpsAutoConfig_e = 0;
pub type gpsAutoBaud_e = libc::c_uint;
pub const GPS_AUTOBAUD_ON: gpsAutoBaud_e = 1;
pub const GPS_AUTOBAUD_OFF: gpsAutoBaud_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsConfig_s {
    pub provider: gpsProvider_e,
    pub sbasMode: sbasMode_e,
    pub autoConfig: gpsAutoConfig_e,
    pub autoBaud: gpsAutoBaud_e,
    pub gps_ublox_use_galileo: uint8_t,
}
pub type gpsConfig_t = gpsConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
// full year
// 1-12
// 1-31
// 0-23
// 0-59
// 0-59
// 0-999
/* LLH Location in NEU axis system */
pub type gpsLocation_t = gpsLocation_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsSolutionData_s {
    pub llh: gpsLocation_t,
    pub groundSpeed: uint16_t,
    pub groundCourse: uint16_t,
    pub hdop: uint16_t,
    pub numSat: uint8_t,
}
pub type gpsSolutionData_t = gpsSolutionData_s;
pub type gpsMessageState_e = libc::c_uint;
pub const GPS_MESSAGE_STATE_ENTRY_COUNT: gpsMessageState_e = 4;
pub const GPS_MESSAGE_STATE_GALILEO: gpsMessageState_e = 3;
pub const GPS_MESSAGE_STATE_SBAS: gpsMessageState_e = 2;
pub const GPS_MESSAGE_STATE_INIT: gpsMessageState_e = 1;
pub const GPS_MESSAGE_STATE_IDLE: gpsMessageState_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsData_s {
    pub errors: uint32_t,
    pub timeouts: uint32_t,
    pub lastMessage: uint32_t,
    pub lastLastMessage: uint32_t,
    pub state_position: uint32_t,
    pub state_ts: uint32_t,
    pub state: uint8_t,
    pub baudrateIndex: uint8_t,
    pub messageState: gpsMessageState_e,
}
pub type gpsData_t = gpsData_s;
pub type navigationMode_e = libc::c_uint;
pub const NAV_MODE_WP: navigationMode_e = 2;
pub const NAV_MODE_POSHOLD: navigationMode_e = 1;
pub const NAV_MODE_NONE: navigationMode_e = 0;
pub type gpsState_e = libc::c_uint;
pub const GPS_LOST_COMMUNICATION: gpsState_e = 5;
pub const GPS_RECEIVING_DATA: gpsState_e = 4;
pub const GPS_CONFIGURE: gpsState_e = 3;
pub const GPS_CHANGE_BAUD: gpsState_e = 2;
pub const GPS_INITIALIZING: gpsState_e = 1;
pub const GPS_UNKNOWN: gpsState_e = 0;
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
// gps error counter - crc error/lost of data/sync etc..
// last time valid GPS data was received (millis)
// last-last valid GPS message. Used to calculate delta.
// incremental variable for loops
// timestamp for last state_position increment
// GPS thread state. Used for detecting cable disconnects and configuring attached devices
// index into auto-detecting or current baudrate
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
pub type gpsInitData_t = gpsInitData_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsInitData_s {
    pub index: uint8_t,
    pub baudrateIndex: uint8_t,
    pub ubx: *const libc::c_char,
    pub mtk: *const libc::c_char,
}
pub const BAUD_9600: baudRate_e = 1;
pub const BAUD_19200: baudRate_e = 2;
pub const BAUD_38400: baudRate_e = 3;
pub const BAUD_57600: baudRate_e = 4;
pub const BAUD_115200: baudRate_e = 5;
pub type serialConsumer = unsafe extern "C" fn(_: uint8_t) -> ();
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
pub const SENSOR_GPS: C2RustUnnamed_6 = 32;
pub const GPS_FIX: C2RustUnnamed_5 = 2;
pub type ubloxSbas_t = ubloxSbas_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ubloxSbas_s {
    pub mode: sbasMode_e,
    pub message: [uint8_t; 6],
}
pub type baudRate_e = libc::c_uint;
pub const BAUD_2470000: baudRate_e = 15;
pub const BAUD_2000000: baudRate_e = 14;
pub const BAUD_1500000: baudRate_e = 13;
pub const BAUD_1000000: baudRate_e = 12;
pub const BAUD_921600: baudRate_e = 11;
pub const BAUD_500000: baudRate_e = 10;
pub const BAUD_460800: baudRate_e = 9;
pub const BAUD_400000: baudRate_e = 8;
pub const BAUD_250000: baudRate_e = 7;
pub const BAUD_230400: baudRate_e = 6;
pub const BAUD_AUTO: baudRate_e = 0;
pub const GPS_FIX_HOME: C2RustUnnamed_5 = 1;
pub const ARMED: C2RustUnnamed_4 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ubx_nav_svinfo_channel {
    pub chn: uint8_t,
    pub svid: uint8_t,
    pub flags: uint8_t,
    pub quality: uint8_t,
    pub cno: uint8_t,
    pub elev: uint8_t,
    pub azim: int16_t,
    pub prRes: int32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ubx_nav_svinfo {
    pub time: uint32_t,
    pub numCh: uint8_t,
    pub globalFlags: uint8_t,
    pub reserved2: uint16_t,
    pub channel: [ubx_nav_svinfo_channel; 16],
}
// see baudRate_e
// Receive buffer
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_3 {
    pub posllh: ubx_nav_posllh,
    pub status: ubx_nav_status,
    pub solution: ubx_nav_solution,
    pub velned: ubx_nav_velned,
    pub svinfo: ubx_nav_svinfo,
    pub bytes: [uint8_t; 344],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ubx_nav_velned {
    pub time: uint32_t,
    pub ned_north: int32_t,
    pub ned_east: int32_t,
    pub ned_down: int32_t,
    pub speed_3d: uint32_t,
    pub speed_2d: uint32_t,
    pub heading_2d: int32_t,
    pub speed_accuracy: uint32_t,
    pub heading_accuracy: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ubx_nav_solution {
    pub time: uint32_t,
    pub time_nsec: int32_t,
    pub week: int16_t,
    pub fix_type: uint8_t,
    pub fix_status: uint8_t,
    pub ecef_x: int32_t,
    pub ecef_y: int32_t,
    pub ecef_z: int32_t,
    pub position_accuracy_3d: uint32_t,
    pub ecef_x_velocity: int32_t,
    pub ecef_y_velocity: int32_t,
    pub ecef_z_velocity: int32_t,
    pub speed_accuracy: uint32_t,
    pub position_DOP: uint16_t,
    pub res: uint8_t,
    pub satellites: uint8_t,
    pub res2: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ubx_nav_status {
    pub time: uint32_t,
    pub fix_type: uint8_t,
    pub fix_status: uint8_t,
    pub differential_status: uint8_t,
    pub res: uint8_t,
    pub time_to_first_fix: uint32_t,
    pub uptime: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ubx_nav_posllh {
    pub time: uint32_t,
    pub longitude: int32_t,
    pub latitude: int32_t,
    pub altitude_ellipsoid: int32_t,
    pub altitude_msl: int32_t,
    pub horizontal_accuracy: uint32_t,
    pub vertical_accuracy: uint32_t,
}
pub const MSG_SVINFO: C2RustUnnamed_7 = 48;
pub const MSG_VELNED: C2RustUnnamed_7 = 18;
pub const NAV_STATUS_TIME_WEEK_VALID: C2RustUnnamed_9 = 4;
pub const NAV_STATUS_TIME_SECOND_VALID: C2RustUnnamed_9 = 8;
pub const FIX_3D: C2RustUnnamed_8 = 3;
pub const NAV_STATUS_FIX_VALID: C2RustUnnamed_9 = 1;
pub const MSG_SOL: C2RustUnnamed_7 = 6;
pub const MSG_STATUS: C2RustUnnamed_7 = 3;
pub const MSG_POSLLH: C2RustUnnamed_7 = 2;
pub const PREAMBLE2: C2RustUnnamed_7 = 98;
pub const PREAMBLE1: C2RustUnnamed_7 = 181;
pub type gpsDataNmea_t = gpsDataNmea_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsDataNmea_s {
    pub latitude: int32_t,
    pub longitude: int32_t,
    pub numSat: uint8_t,
    pub altitude: int32_t,
    pub speed: uint16_t,
    pub hdop: uint16_t,
    pub ground_course: uint16_t,
    pub time: uint32_t,
    pub date: uint32_t,
}
pub type C2RustUnnamed_4 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_4 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_4 = 2;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_5 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_5 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_5 = 4;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_6 = 64;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_6 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_6 = 16;
pub const SENSOR_MAG: C2RustUnnamed_6 = 8;
pub const SENSOR_BARO: C2RustUnnamed_6 = 4;
pub const SENSOR_ACC: C2RustUnnamed_6 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_6 = 1;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const MSG_CFG_NAV_SETTINGS: C2RustUnnamed_7 = 36;
pub const MSG_CFG_SET_RATE: C2RustUnnamed_7 = 1;
pub const MSG_CFG_RATE: C2RustUnnamed_7 = 8;
pub const MSG_CFG_PRT: C2RustUnnamed_7 = 0;
pub const MSG_ACK_ACK: C2RustUnnamed_7 = 1;
pub const MSG_ACK_NACK: C2RustUnnamed_7 = 0;
pub const CLASS_CFG: C2RustUnnamed_7 = 6;
pub const CLASS_ACK: C2RustUnnamed_7 = 5;
pub const CLASS_NAV: C2RustUnnamed_7 = 1;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const FIX_TIME: C2RustUnnamed_8 = 5;
pub const FIX_GPS_DEAD_RECKONING: C2RustUnnamed_8 = 4;
pub const FIX_2D: C2RustUnnamed_8 = 2;
pub const FIX_DEAD_RECKONING: C2RustUnnamed_8 = 1;
pub const FIX_NONE: C2RustUnnamed_8 = 0;
pub type C2RustUnnamed_9 = libc::c_uint;
#[inline]
unsafe extern "C" fn gpsConfig() -> *const gpsConfig_t {
    return &mut gpsConfig_System;
}
#[no_mangle]
pub static mut gpsPacketLog: [libc::c_char; 21] = [0; 21];
static mut gpsPacketLogChar: *mut libc::c_char =
    unsafe { gpsPacketLog.as_ptr() as *mut _ };
// **********************
// GPS
// **********************
#[no_mangle]
pub static mut GPS_home: [int32_t; 2] = [0; 2];
#[no_mangle]
pub static mut GPS_distanceToHome: uint16_t = 0;
// distance to home point in meters
#[no_mangle]
pub static mut GPS_directionToHome: int16_t = 0;
// direction to home or hol point in degrees
#[no_mangle]
pub static mut dTnav: libc::c_float = 0.;
// Delta Time in milliseconds for navigation computations, updated with every good GPS read
#[no_mangle]
pub static mut actual_speed: [int16_t; 2] =
    [0 as libc::c_int as int16_t, 0 as libc::c_int as int16_t];
#[no_mangle]
pub static mut nav_takeoff_bearing: int16_t = 0;
#[no_mangle]
pub static mut nav_mode: navigationMode_e = NAV_MODE_NONE;
static mut GPS_filter_index: uint8_t = 0 as libc::c_int as uint8_t;
static mut GPS_filter: [[int32_t; 5]; 2] = [[0; 5]; 2];
static mut GPS_filter_sum: [int32_t; 2] = [0; 2];
static mut GPS_read: [int32_t; 2] = [0; 2];
static mut GPS_filtered: [int32_t; 2] = [0; 2];
static mut GPS_degree: [int32_t; 2] = [0; 2];
//the lat lon degree without any decimals (lat/10 000 000)
static mut fraction3: [uint16_t; 2] = [0; 2];
#[no_mangle]
pub static mut gpsSol: gpsSolutionData_t =
    gpsSolutionData_t{llh: gpsLocation_t{lat: 0, lon: 0, alt: 0,},
                      groundSpeed: 0,
                      groundCourse: 0,
                      hdop: 0,
                      numSat: 0,};
#[no_mangle]
pub static mut GPS_packetCount: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut GPS_svInfoReceivedCount: uint32_t =
    0 as libc::c_int as uint32_t;
// SV = Space Vehicle, counter increments each time SV info is received.
#[no_mangle]
pub static mut GPS_update: uint8_t = 0 as libc::c_int as uint8_t;
// it's a binary toggle to distinct a GPS position update
#[no_mangle]
pub static mut GPS_numCh: uint8_t = 0;
// Number of channels
#[no_mangle]
pub static mut GPS_svinfo_chn: [uint8_t; 16] = [0; 16];
// Channel number
#[no_mangle]
pub static mut GPS_svinfo_svid: [uint8_t; 16] = [0; 16];
// Satellite ID
#[no_mangle]
pub static mut GPS_svinfo_quality: [uint8_t; 16] = [0; 16];
// Bitfield Qualtity
#[no_mangle]
pub static mut GPS_svinfo_cno: [uint8_t; 16] = [0; 16];
static mut gpsPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
// NMEA will cycle through these until valid data is received
static mut gpsInitData: [gpsInitData_t; 5] =
    [{
         let mut init =
             gpsInitData_s{index:
                               GPS_BAUDRATE_115200 as libc::c_int as uint8_t,
                           baudrateIndex:
                               BAUD_115200 as libc::c_int as uint8_t,
                           ubx:
                               b"$PUBX,41,1,0003,0001,115200,0*1E\r\n\x00" as
                                   *const u8 as *const libc::c_char,
                           mtk:
                               b"$PMTK251,115200*1F\r\n\x00" as *const u8 as
                                   *const libc::c_char,};
         init
     },
     {
         let mut init =
             gpsInitData_s{index:
                               GPS_BAUDRATE_57600 as libc::c_int as uint8_t,
                           baudrateIndex:
                               BAUD_57600 as libc::c_int as uint8_t,
                           ubx:
                               b"$PUBX,41,1,0003,0001,57600,0*2D\r\n\x00" as
                                   *const u8 as *const libc::c_char,
                           mtk:
                               b"$PMTK251,57600*2C\r\n\x00" as *const u8 as
                                   *const libc::c_char,};
         init
     },
     {
         let mut init =
             gpsInitData_s{index:
                               GPS_BAUDRATE_38400 as libc::c_int as uint8_t,
                           baudrateIndex:
                               BAUD_38400 as libc::c_int as uint8_t,
                           ubx:
                               b"$PUBX,41,1,0003,0001,38400,0*26\r\n\x00" as
                                   *const u8 as *const libc::c_char,
                           mtk:
                               b"$PMTK251,38400*27\r\n\x00" as *const u8 as
                                   *const libc::c_char,};
         init
     },
     {
         let mut init =
             gpsInitData_s{index:
                               GPS_BAUDRATE_19200 as libc::c_int as uint8_t,
                           baudrateIndex:
                               BAUD_19200 as libc::c_int as uint8_t,
                           ubx:
                               b"$PUBX,41,1,0003,0001,19200,0*23\r\n\x00" as
                                   *const u8 as *const libc::c_char,
                           mtk:
                               b"$PMTK251,19200*22\r\n\x00" as *const u8 as
                                   *const libc::c_char,};
         init
     },
     {
         let mut init =
             gpsInitData_s{index: GPS_BAUDRATE_9600 as libc::c_int as uint8_t,
                           baudrateIndex: BAUD_9600 as libc::c_int as uint8_t,
                           ubx:
                               b"$PUBX,41,1,0003,0001,9600,0*16\r\n\x00" as
                                   *const u8 as *const libc::c_char,
                           mtk: b"\x00" as *const u8 as *const libc::c_char,};
         init
     }];
static mut ubloxInit: [uint8_t; 179] =
    [0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x24 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xff as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x10 as libc::c_int as uint8_t, 0x27 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xfa as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xfa as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x64 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x2c as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x3c as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xc8 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1c as libc::c_int as uint8_t, 0x6c as libc::c_int as uint8_t,
     0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xf0 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t,
     0x19 as libc::c_int as uint8_t, 0xb5 as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xf0 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xfd as libc::c_int as uint8_t, 0x15 as libc::c_int as uint8_t,
     0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xf0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xfb as libc::c_int as uint8_t,
     0x11 as libc::c_int as uint8_t, 0xb5 as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xf0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xfa as libc::c_int as uint8_t, 0xf as libc::c_int as uint8_t,
     0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xf0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xfc as libc::c_int as uint8_t,
     0x13 as libc::c_int as uint8_t, 0xb5 as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xf0 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0xfe as libc::c_int as uint8_t, 0x17 as libc::c_int as uint8_t,
     0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0xe as libc::c_int as uint8_t,
     0x47 as libc::c_int as uint8_t, 0xb5 as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0xf as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
     0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x12 as libc::c_int as uint8_t,
     0x4f as libc::c_int as uint8_t, 0xb5 as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x30 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x40 as libc::c_int as uint8_t, 0xa7 as libc::c_int as uint8_t,
     0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x12 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1e as libc::c_int as uint8_t,
     0x67 as libc::c_int as uint8_t, 0xb5 as libc::c_int as uint8_t,
     0x62 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xc8 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0xde as libc::c_int as uint8_t,
     0x6a as libc::c_int as uint8_t];
static mut ubloxSbasPrefix: [uint8_t; 10] =
    [0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x16 as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
// Note: these must be defined in the same order is sbasMode_e since no lookup table is used.
static mut ubloxSbas: [ubloxSbas_t; 5] =
    [{
         let mut init =
             ubloxSbas_s{mode: SBAS_AUTO,
                         message:
                             [0 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0x31 as libc::c_int as uint8_t,
                              0xe5 as libc::c_int as uint8_t],};
         init
     },
     {
         let mut init =
             ubloxSbas_s{mode: SBAS_EGNOS,
                         message:
                             [0x51 as libc::c_int as uint8_t,
                              0x8 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0x8a as libc::c_int as uint8_t,
                              0x41 as libc::c_int as uint8_t],};
         init
     },
     {
         let mut init =
             ubloxSbas_s{mode: SBAS_WAAS,
                         message:
                             [0x4 as libc::c_int as uint8_t,
                              0xe0 as libc::c_int as uint8_t,
                              0x4 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0x19 as libc::c_int as uint8_t,
                              0x9d as libc::c_int as uint8_t],};
         init
     },
     {
         let mut init =
             ubloxSbas_s{mode: SBAS_MSAS,
                         message:
                             [0 as libc::c_int as uint8_t,
                              0x2 as libc::c_int as uint8_t,
                              0x2 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0x35 as libc::c_int as uint8_t,
                              0xef as libc::c_int as uint8_t],};
         init
     },
     {
         let mut init =
             ubloxSbas_s{mode: SBAS_GAGAN,
                         message:
                             [0x80 as libc::c_int as uint8_t,
                              0x1 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0 as libc::c_int as uint8_t,
                              0xb2 as libc::c_int as uint8_t,
                              0xe8 as libc::c_int as uint8_t],};
         init
     }];
// Remove QZSS and add Galileo (only 3 GNSS systems supported simultaneously)
// Frame captured from uCenter
static mut ubloxGalileoInit: [uint8_t; 68] =
    [0xb5 as libc::c_int as uint8_t, 0x62 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x3e as libc::c_int as uint8_t,
     0x3c as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x20 as libc::c_int as uint8_t,
     0x20 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
     0x10 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
     0x10 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
     0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
     0x6 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
     0xe as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0x55 as libc::c_int as uint8_t, 0x47 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut gpsData: gpsData_t =
    gpsData_t{errors: 0,
              timeouts: 0,
              lastMessage: 0,
              lastLastMessage: 0,
              state_position: 0,
              state_ts: 0,
              state: 0,
              baudrateIndex: 0,
              messageState: GPS_MESSAGE_STATE_IDLE,};
#[no_mangle]
pub static mut gpsConfig_System: gpsConfig_t =
    gpsConfig_t{provider: GPS_NMEA,
                sbasMode: SBAS_AUTO,
                autoConfig: GPS_AUTOCONFIG_OFF,
                autoBaud: GPS_AUTOBAUD_OFF,
                gps_ublox_use_galileo: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut gpsConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (30 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<gpsConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &gpsConfig_System as *const gpsConfig_t as
                                     *mut gpsConfig_t as *mut uint8_t,
                             copy:
                                 &gpsConfig_Copy as *const gpsConfig_t as
                                     *mut gpsConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_gpsConfig
                                                         as *const gpsConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut gpsConfig_Copy: gpsConfig_t =
    gpsConfig_t{provider: GPS_NMEA,
                sbasMode: SBAS_AUTO,
                autoConfig: GPS_AUTOCONFIG_OFF,
                autoBaud: GPS_AUTOBAUD_OFF,
                gps_ublox_use_galileo: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_gpsConfig: gpsConfig_t =
    {
        let mut init =
            gpsConfig_s{provider: GPS_NMEA,
                        sbasMode: SBAS_AUTO,
                        autoConfig: GPS_AUTOCONFIG_ON,
                        autoBaud: GPS_AUTOBAUD_OFF,
                        gps_ublox_use_galileo: 0 as libc::c_int as uint8_t,};
        init
    };
unsafe extern "C" fn shiftPacketLog() {
    let mut i: uint32_t = 0;
    i =
        (::core::mem::size_of::<[libc::c_char; 21]>() as
             libc::c_ulong).wrapping_div(::core::mem::size_of::<libc::c_char>()
                                             as
                                             libc::c_ulong).wrapping_sub(1 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_ulong)
            as uint32_t;
    while i > 0 as libc::c_int as libc::c_uint {
        gpsPacketLog[i as usize] =
            gpsPacketLog[i.wrapping_sub(1 as libc::c_int as libc::c_uint) as
                             usize];
        i = i.wrapping_sub(1)
    };
}
unsafe extern "C" fn gpsSetState(mut state: gpsState_e) {
    gpsData.state = state as uint8_t;
    gpsData.state_position = 0 as libc::c_int as uint32_t;
    gpsData.state_ts = millis();
    gpsData.messageState = GPS_MESSAGE_STATE_IDLE;
}
// Navigation mode
// it's a binary toogle to distinct a GPS position update
// Number of channels
// Channel number
// Satellite ID
// Bitfield Qualtity
// Carrier to Noise Ratio (Signal Strength)
#[no_mangle]
pub unsafe extern "C" fn gpsInit() {
    gpsData.baudrateIndex = 0 as libc::c_int as uint8_t;
    gpsData.errors = 0 as libc::c_int as uint32_t;
    gpsData.timeouts = 0 as libc::c_int as uint32_t;
    memset(gpsPacketLog.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[libc::c_char; 21]>() as libc::c_ulong);
    // init gpsData structure. if we're not actually enabled, don't bother doing anything else
    gpsSetState(GPS_UNKNOWN);
    gpsData.lastMessage = millis();
    let mut gpsPortConfig: *mut serialPortConfig_t =
        findSerialPortConfig(FUNCTION_GPS);
    if gpsPortConfig.is_null() { return }
    while gpsInitData[gpsData.baudrateIndex as usize].baudrateIndex as
              libc::c_int != (*gpsPortConfig).gps_baudrateIndex as libc::c_int
          {
        gpsData.baudrateIndex = gpsData.baudrateIndex.wrapping_add(1);
        if !(gpsData.baudrateIndex as libc::c_ulong >=
                 (::core::mem::size_of::<[gpsInitData_t; 5]>() as
                      libc::c_ulong).wrapping_div(::core::mem::size_of::<gpsInitData_t>()
                                                      as libc::c_ulong)) {
            continue ;
        }
        gpsData.baudrateIndex = 0 as libc::c_int as uint8_t;
        break ;
    }
    let mut mode: portMode_e = MODE_RXTX;
    // no callback - buffer will be consumed in gpsUpdate()
    gpsPort =
        openSerialPort((*gpsPortConfig).identifier, FUNCTION_GPS, None,
                       0 as *mut libc::c_void,
                       *baudRates.as_ptr().offset(gpsInitData[gpsData.baudrateIndex
                                                                  as
                                                                  usize].baudrateIndex
                                                      as isize), mode,
                       SERIAL_NOT_INVERTED);
    if gpsPort.is_null() { return }
    // signal GPS "thread" to initialize when it gets to it
    gpsSetState(GPS_INITIALIZING);
}
#[no_mangle]
pub unsafe extern "C" fn gpsInitNmea() {
    let mut now: uint32_t = 0;
    match gpsData.state as libc::c_int {
        1 => {
            now = millis();
            if now.wrapping_sub(gpsData.state_ts) <
                   1000 as libc::c_int as libc::c_uint {
                return
            }
            gpsData.state_ts = now;
            if gpsData.state_position < 1 as libc::c_int as libc::c_uint {
                serialSetBaudRate(gpsPort, 4800 as libc::c_int as uint32_t);
                gpsData.state_position =
                    gpsData.state_position.wrapping_add(1)
            } else if gpsData.state_position <
                          2 as libc::c_int as libc::c_uint {
                // print our FIXED init string for the baudrate we want to be at
                serialPrint(gpsPort,
                            b"$PSRF100,1,115200,8,1,0*05\r\n\x00" as *const u8
                                as *const libc::c_char);
                gpsData.state_position =
                    gpsData.state_position.wrapping_add(1)
            } else {
                // we're now (hopefully) at the correct rate, next state will switch to it
                gpsSetState(GPS_CHANGE_BAUD);
            }
        }
        2 => {
            now = millis();
            if now.wrapping_sub(gpsData.state_ts) <
                   1000 as libc::c_int as libc::c_uint {
                return
            }
            gpsData.state_ts = now;
            if gpsData.state_position < 1 as libc::c_int as libc::c_uint {
                serialSetBaudRate(gpsPort,
                                  *baudRates.as_ptr().offset(gpsInitData[gpsData.baudrateIndex
                                                                             as
                                                                             usize].baudrateIndex
                                                                 as isize));
                gpsData.state_position =
                    gpsData.state_position.wrapping_add(1)
            } else if gpsData.state_position <
                          2 as libc::c_int as libc::c_uint {
                serialPrint(gpsPort,
                            b"$PSRF103,00,6,00,0*23\r\n\x00" as *const u8 as
                                *const libc::c_char);
                gpsData.state_position =
                    gpsData.state_position.wrapping_add(1)
            } else { gpsSetState(GPS_RECEIVING_DATA); }
        }
        _ => { }
    };
}
// USE_GPS_NMEA
#[no_mangle]
pub unsafe extern "C" fn gpsInitUblox() {
    let mut now: uint32_t = 0;
    // UBX will run at the serial port's baudrate, it shouldn't be "autodetected". So here we force it to that rate
    // Wait until GPS transmit buffer is empty
    if !isSerialTransmitBufferEmpty(gpsPort) { return }
    match gpsData.state as libc::c_int {
        1 => {
            now = millis();
            if now.wrapping_sub(gpsData.state_ts) <
                   200 as libc::c_int as libc::c_uint {
                return
            }
            if gpsData.state_position <
                   (GPS_BAUDRATE_9600 as libc::c_int + 1 as libc::c_int) as
                       libc::c_uint {
                // try different speed to INIT
                let mut newBaudRateIndex: baudRate_e =
                    gpsInitData[gpsData.state_position as usize].baudrateIndex
                        as baudRate_e;
                gpsData.state_ts = now;
                if lookupBaudRateIndex(serialGetBaudRate(gpsPort)) as
                       libc::c_uint != newBaudRateIndex as libc::c_uint {
                    // change the rate if needed and wait a little
                    serialSetBaudRate(gpsPort,
                                      *baudRates.as_ptr().offset(newBaudRateIndex
                                                                     as
                                                                     isize));
                    return
                }
                // print our FIXED init string for the baudrate we want to be at
                serialPrint(gpsPort,
                            gpsInitData[gpsData.baudrateIndex as usize].ubx);
                gpsData.state_position =
                    gpsData.state_position.wrapping_add(1)
            } else {
                // we're now (hopefully) at the correct rate, next state will switch to it
                gpsSetState(GPS_CHANGE_BAUD);
            }
        }
        2 => {
            serialSetBaudRate(gpsPort,
                              *baudRates.as_ptr().offset(gpsInitData[gpsData.baudrateIndex
                                                                         as
                                                                         usize].baudrateIndex
                                                             as isize));
            gpsSetState(GPS_CONFIGURE);
        }
        3 => {
            // Either use specific config file for GPS or let dynamically upload config
            if (*gpsConfig()).autoConfig as libc::c_uint ==
                   GPS_AUTOCONFIG_OFF as libc::c_int as libc::c_uint {
                gpsSetState(GPS_RECEIVING_DATA);
            } else {
                if gpsData.messageState as libc::c_uint ==
                       GPS_MESSAGE_STATE_IDLE as libc::c_int as libc::c_uint {
                    gpsData.messageState += 1
                }
                if gpsData.messageState as libc::c_uint ==
                       GPS_MESSAGE_STATE_INIT as libc::c_int as libc::c_uint {
                    if (gpsData.state_position as libc::c_ulong) <
                           ::core::mem::size_of::<[uint8_t; 179]>() as
                               libc::c_ulong {
                        serialWrite(gpsPort,
                                    ubloxInit[gpsData.state_position as
                                                  usize]);
                        gpsData.state_position =
                            gpsData.state_position.wrapping_add(1)
                    } else {
                        gpsData.state_position = 0 as libc::c_int as uint32_t;
                        gpsData.messageState += 1
                    }
                }
                if gpsData.messageState as libc::c_uint ==
                       GPS_MESSAGE_STATE_SBAS as libc::c_int as libc::c_uint {
                    if gpsData.state_position <
                           10 as libc::c_int as libc::c_uint {
                        serialWrite(gpsPort,
                                    ubloxSbasPrefix[gpsData.state_position as
                                                        usize]);
                        gpsData.state_position =
                            gpsData.state_position.wrapping_add(1)
                    } else if gpsData.state_position <
                                  (10 as libc::c_int + 6 as libc::c_int) as
                                      libc::c_uint {
                        serialWrite(gpsPort,
                                    ubloxSbas[(*gpsConfig()).sbasMode as
                                                  usize].message[gpsData.state_position.wrapping_sub(10
                                                                                                         as
                                                                                                         libc::c_int
                                                                                                         as
                                                                                                         libc::c_uint)
                                                                     as
                                                                     usize]);
                        gpsData.state_position =
                            gpsData.state_position.wrapping_add(1)
                    } else {
                        gpsData.state_position = 0 as libc::c_int as uint32_t;
                        gpsData.messageState += 1
                    }
                }
                if gpsData.messageState as libc::c_uint ==
                       GPS_MESSAGE_STATE_GALILEO as libc::c_int as
                           libc::c_uint {
                    if (*gpsConfig()).gps_ublox_use_galileo as libc::c_int !=
                           0 &&
                           (gpsData.state_position as libc::c_ulong) <
                               ::core::mem::size_of::<[uint8_t; 68]>() as
                                   libc::c_ulong {
                        serialWrite(gpsPort,
                                    ubloxGalileoInit[gpsData.state_position as
                                                         usize]);
                        gpsData.state_position =
                            gpsData.state_position.wrapping_add(1)
                    } else {
                        gpsData.state_position = 0 as libc::c_int as uint32_t;
                        gpsData.messageState += 1
                    }
                }
                if gpsData.messageState as libc::c_uint >=
                       GPS_MESSAGE_STATE_ENTRY_COUNT as libc::c_int as
                           libc::c_uint {
                    // ublox should be initialised, try receiving
                    gpsSetState(GPS_RECEIVING_DATA);
                }
            }
        }
        _ => { }
    };
}
// USE_GPS_UBLOX
#[no_mangle]
pub unsafe extern "C" fn gpsInitHardware() {
    match (*gpsConfig()).provider as libc::c_uint {
        0 => { gpsInitNmea(); }
        1 => { gpsInitUblox(); }
        _ => { }
    };
}
unsafe extern "C" fn updateGpsIndicator(mut currentTimeUs: timeUs_t) {
    static mut GPSLEDTime: uint32_t = 0;
    if currentTimeUs.wrapping_sub(GPSLEDTime) as int32_t >= 0 as libc::c_int
           && gpsSol.numSat as libc::c_int >= 5 as libc::c_int {
        GPSLEDTime =
            currentTimeUs.wrapping_add(150000 as libc::c_int as libc::c_uint);
        ledToggle(1 as libc::c_int);
    };
}
#[no_mangle]
pub unsafe extern "C" fn gpsUpdate(mut currentTimeUs: timeUs_t) {
    // read out available GPS bytes
    if !gpsPort.is_null() {
        while serialRxBytesWaiting(gpsPort) != 0 {
            gpsNewData(serialRead(gpsPort) as uint16_t);
        }
    }
    match gpsData.state as libc::c_int {
        1 | 2 | 3 => { gpsInitHardware(); }
        5 => {
            gpsData.timeouts = gpsData.timeouts.wrapping_add(1);
            if (*gpsConfig()).autoBaud as u64 != 0 {
                // try another rate
                gpsData.baudrateIndex = gpsData.baudrateIndex.wrapping_add(1);
                gpsData.baudrateIndex =
                    (gpsData.baudrateIndex as libc::c_int %
                         (GPS_BAUDRATE_9600 as libc::c_int +
                              1 as libc::c_int)) as uint8_t
            }
            gpsData.lastMessage = millis();
            gpsSol.numSat = 0 as libc::c_int as uint8_t;
            stateFlags =
                (stateFlags as libc::c_int & !(GPS_FIX as libc::c_int)) as
                    uint8_t;
            gpsSetState(GPS_INITIALIZING);
        }
        4 => {
            // check for no data/gps timeout/cable disconnection etc
            if millis().wrapping_sub(gpsData.lastMessage) >
                   2500 as libc::c_int as libc::c_uint {
                // remove GPS from capability
                sensorsClear(SENSOR_GPS as libc::c_int as uint32_t);
                gpsSetState(GPS_LOST_COMMUNICATION);
            }
        }
        0 | _ => { }
    }
    if sensors(SENSOR_GPS as libc::c_int as uint32_t) {
        updateGpsIndicator(currentTimeUs);
    }
    if gpsRescueIsConfigured() { updateGPSRescueState(); };
}
unsafe extern "C" fn gpsNewData(mut c: uint16_t) {
    if !gpsNewFrame(c as uint8_t) { return }
    // new data received and parsed, we're in business
    gpsData.lastLastMessage = gpsData.lastMessage;
    gpsData.lastMessage = millis();
    sensorsSet(SENSOR_GPS as libc::c_int as uint32_t);
    if GPS_update as libc::c_int == 1 as libc::c_int {
        GPS_update = 0 as libc::c_int as uint8_t
    } else { GPS_update = 1 as libc::c_int as uint8_t }
    onGpsNewData();
}
#[no_mangle]
pub unsafe extern "C" fn gpsNewFrame(mut c: uint8_t) -> bool {
    match (*gpsConfig()).provider as libc::c_uint {
        0 => {
            // NMEA
            return gpsNewFrameNMEA(c as libc::c_char)
        }
        1 => {
            // UBX binary
            return gpsNewFrameUBLOX(c)
        }
        _ => { }
    }
    return 0 as libc::c_int != 0;
}
// This code is used for parsing NMEA data
/* Alex optimization
  The latitude or longitude is coded this way in NMEA frames
  dm.f   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - f can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000

  EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
  resolution also increased precision of nav calculations
static uint32_t GPS_coord_to_degrees(char *coordinateString)
{
    char *p = s, *d = s;
    uint8_t min, deg = 0;
    uint16_t frac = 0, mult = 10000;

    while (*p) {                // parse the string until its end
        if (d != s) {
            frac += (*p - '0') * mult;  // calculate only fractional part on up to 5 digits  (d != s condition is true when the . is located)
            mult /= 10;
        }
        if (*p == '.')
            d = p;              // locate '.' char in the string
        p++;
    }
    if (p == s)
        return 0;
    while (s < d - 2) {
        deg *= 10;              // convert degrees : all chars before minutes ; for the first iteration, deg = 0
        deg += *(s++) - '0';
    }
    min = *(d - 1) - '0' + (*(d - 2) - '0') * 10;       // convert minutes : 2 previous char before '.'
    return deg * 10000000UL + (min * 100000UL + frac) * 10UL / 6;
}
*/
// helper functions
unsafe extern "C" fn grab_fields(mut src: *mut libc::c_char,
                                 mut mult: uint8_t) -> uint32_t {
    // convert string to uint32
    let mut i: uint32_t = 0;
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut isneg: libc::c_int = 0 as libc::c_int;
    i = 0 as libc::c_int as uint32_t;
    while *src.offset(i as isize) as libc::c_int != 0 as libc::c_int {
        if i == 0 as libc::c_int as libc::c_uint &&
               *src.offset(0 as libc::c_int as isize) as libc::c_int ==
                   '-' as i32 {
            // detect negative sign
            isneg = 1 as libc::c_int
            // jump to next character if the first one was a negative sign
        } else {
            if *src.offset(i as isize) as libc::c_int == '.' as i32 {
                i = i.wrapping_add(1);
                if mult as libc::c_int == 0 as libc::c_int { break ; }
                *src.offset(i.wrapping_add(mult as libc::c_uint) as isize) =
                    0 as libc::c_int as libc::c_char
            }
            tmp =
                (tmp as
                     libc::c_uint).wrapping_mul(10 as libc::c_int as
                                                    libc::c_uint) as uint32_t
                    as uint32_t;
            if *src.offset(i as isize) as libc::c_int >= '0' as i32 &&
                   *src.offset(i as isize) as libc::c_int <= '9' as i32 {
                tmp =
                    (tmp as
                         libc::c_uint).wrapping_add((*src.offset(i as isize)
                                                         as libc::c_int -
                                                         '0' as i32) as
                                                        libc::c_uint) as
                        uint32_t as uint32_t
            }
            if i >= 15 as libc::c_int as libc::c_uint {
                return 0 as libc::c_int as uint32_t
                // out of bounds
            }
        }
        i = i.wrapping_add(1)
    }
    return if isneg != 0 { tmp.wrapping_neg() } else { tmp };
    // handle negative altitudes
}
unsafe extern "C" fn gpsNewFrameNMEA(mut c: libc::c_char) -> bool {
    static mut gps_Msg: gpsDataNmea_t =
        gpsDataNmea_t{latitude: 0,
                      longitude: 0,
                      numSat: 0,
                      altitude: 0,
                      speed: 0,
                      hdop: 0,
                      ground_course: 0,
                      time: 0,
                      date: 0,};
    let mut frameOK: uint8_t = 0 as libc::c_int as uint8_t;
    static mut param: uint8_t = 0 as libc::c_int as uint8_t;
    static mut offset: uint8_t = 0 as libc::c_int as uint8_t;
    static mut parity: uint8_t = 0 as libc::c_int as uint8_t;
    static mut string: [libc::c_char; 15] = [0; 15];
    static mut checksum_param: uint8_t = 0;
    static mut gps_frame: uint8_t = 0 as libc::c_int as uint8_t;
    static mut svMessageNum: uint8_t = 0 as libc::c_int as uint8_t;
    let mut svSatNum: uint8_t = 0 as libc::c_int as uint8_t;
    let mut svPacketIdx: uint8_t = 0 as libc::c_int as uint8_t;
    let mut svSatParam: uint8_t = 0 as libc::c_int as uint8_t;
    match c as libc::c_int {
        36 => {
            param = 0 as libc::c_int as uint8_t;
            offset = 0 as libc::c_int as uint8_t;
            parity = 0 as libc::c_int as uint8_t
        }
        44 | 42 => {
            string[offset as usize] = 0 as libc::c_int as libc::c_char;
            if param as libc::c_int == 0 as libc::c_int {
                //frame identification
                gps_frame = 0 as libc::c_int as uint8_t;
                if string[0 as libc::c_int as usize] as libc::c_int ==
                       'G' as i32 &&
                       string[1 as libc::c_int as usize] as libc::c_int ==
                           'P' as i32 &&
                       string[2 as libc::c_int as usize] as libc::c_int ==
                           'G' as i32 &&
                       string[3 as libc::c_int as usize] as libc::c_int ==
                           'G' as i32 &&
                       string[4 as libc::c_int as usize] as libc::c_int ==
                           'A' as i32 {
                    gps_frame = 1 as libc::c_int as uint8_t
                }
                if string[0 as libc::c_int as usize] as libc::c_int ==
                       'G' as i32 &&
                       string[1 as libc::c_int as usize] as libc::c_int ==
                           'P' as i32 &&
                       string[2 as libc::c_int as usize] as libc::c_int ==
                           'R' as i32 &&
                       string[3 as libc::c_int as usize] as libc::c_int ==
                           'M' as i32 &&
                       string[4 as libc::c_int as usize] as libc::c_int ==
                           'C' as i32 {
                    gps_frame = 2 as libc::c_int as uint8_t
                }
                if string[0 as libc::c_int as usize] as libc::c_int ==
                       'G' as i32 &&
                       string[1 as libc::c_int as usize] as libc::c_int ==
                           'P' as i32 &&
                       string[2 as libc::c_int as usize] as libc::c_int ==
                           'G' as i32 &&
                       string[3 as libc::c_int as usize] as libc::c_int ==
                           'S' as i32 &&
                       string[4 as libc::c_int as usize] as libc::c_int ==
                           'V' as i32 {
                    gps_frame = 3 as libc::c_int as uint8_t
                }
            }
            match gps_frame as libc::c_int {
                1 => {
                    //************* GPGGA FRAME parsing
                    match param as libc::c_int {
                        2 => {
                            //          case 1:             // Time information
            //              break;
                            gps_Msg.latitude =
                                GPS_coord_to_degrees(string.as_mut_ptr()) as
                                    int32_t
                        }
                        3 => {
                            if string[0 as libc::c_int as usize] as
                                   libc::c_int == 'S' as i32 {
                                gps_Msg.latitude *= -(1 as libc::c_int)
                            }
                        }
                        4 => {
                            gps_Msg.longitude =
                                GPS_coord_to_degrees(string.as_mut_ptr()) as
                                    int32_t
                        }
                        5 => {
                            if string[0 as libc::c_int as usize] as
                                   libc::c_int == 'W' as i32 {
                                gps_Msg.longitude *= -(1 as libc::c_int)
                            }
                        }
                        6 => {
                            if string[0 as libc::c_int as usize] as
                                   libc::c_int > '0' as i32 {
                                stateFlags =
                                    (stateFlags as libc::c_int |
                                         GPS_FIX as libc::c_int) as uint8_t
                            } else {
                                stateFlags =
                                    (stateFlags as libc::c_int &
                                         !(GPS_FIX as libc::c_int)) as uint8_t
                            }
                        }
                        7 => {
                            gps_Msg.numSat =
                                grab_fields(string.as_mut_ptr(),
                                            0 as libc::c_int as uint8_t) as
                                    uint8_t
                        }
                        8 => {
                            gps_Msg.hdop =
                                grab_fields(string.as_mut_ptr(),
                                            1 as libc::c_int as
                                                uint8_t).wrapping_mul(100 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
                                    as uint16_t
                        }
                        9 => { // hdop
                            gps_Msg.altitude =
                                grab_fields(string.as_mut_ptr(),
                                            1 as libc::c_int as
                                                uint8_t).wrapping_mul(10 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
                                    as int32_t
                        }
                        _ => { }
                    }
                }
                2 =>
                { // altitude in centimeters. Note: NMEA delivers altitude with 1 or 3 decimals. It's safer to cut at 0.1m and multiply by 10
                    //************* GPRMC FRAME parsing
                    match param as libc::c_int {
                        1 => {
                            gps_Msg.time =
                                grab_fields(string.as_mut_ptr(),
                                            2 as libc::c_int as uint8_t)
                        }
                        7 => { // UTC time hhmmss.ss
                            gps_Msg.speed =
                                (grab_fields(string.as_mut_ptr(),
                                             1 as libc::c_int as uint8_t) as
                                     libc::c_long * 5144 as libc::c_long /
                                     1000 as libc::c_long) as uint16_t
                        }
                        8 => { // speed in cm/s added by Mis
                            gps_Msg.ground_course =
                                grab_fields(string.as_mut_ptr(),
                                            1 as libc::c_int as uint8_t) as
                                    uint16_t
                        }
                        9 => { // ground course deg * 10
                            gps_Msg.date =
                                grab_fields(string.as_mut_ptr(),
                                            0 as libc::c_int as uint8_t)
                        }
                        _ => { }
                    }
                }
                3 => {
                    match param as libc::c_int {
                        2 => { // date dd/mm/yy
                            /*case 1:
                            // Total number of messages of this type in this cycle
                            break; */
                            // Message number
                            svMessageNum =
                                grab_fields(string.as_mut_ptr(),
                                            0 as libc::c_int as uint8_t) as
                                    uint8_t
                        }
                        3 => {
                            // Total number of SVs visible
                            GPS_numCh =
                                grab_fields(string.as_mut_ptr(),
                                            0 as libc::c_int as uint8_t) as
                                    uint8_t
                        }
                        _ => { }
                    } // satellite number in packet, 1-4
                    if !((param as libc::c_int) < 4 as libc::c_int) {
                        svPacketIdx =
                            ((param as libc::c_int - 4 as libc::c_int) /
                                 4 as libc::c_int + 1 as libc::c_int) as
                                uint8_t; // global satellite number
                        svSatNum =
                            (svPacketIdx as libc::c_int +
                                 4 as libc::c_int *
                                     (svMessageNum as libc::c_int -
                                          1 as libc::c_int)) as
                                uint8_t; // parameter number for satellite
                        svSatParam =
                            (param as libc::c_int - 3 as libc::c_int -
                                 4 as libc::c_int *
                                     (svPacketIdx as libc::c_int -
                                          1 as libc::c_int)) as uint8_t;
                        if !(svSatNum as libc::c_int > 16 as libc::c_int) {
                            match svSatParam as libc::c_int {
                                1 => {
                                    // SV PRN number
                                    GPS_svinfo_chn[(svSatNum as libc::c_int -
                                                        1 as libc::c_int) as
                                                       usize] = svSatNum;
                                    GPS_svinfo_svid[(svSatNum as libc::c_int -
                                                         1 as libc::c_int) as
                                                        usize] =
                                        grab_fields(string.as_mut_ptr(),
                                                    0 as libc::c_int as
                                                        uint8_t) as uint8_t
                                }
                                4 => {
                                    /*case 2:
                            // Elevation, in degrees, 90 maximum
                            break;
                        case 3:
                            // Azimuth, degrees from True North, 000 through 359
                            break; */
                                    // SNR, 00 through 99 dB (null when not tracking)
                                    GPS_svinfo_cno[(svSatNum as libc::c_int -
                                                        1 as libc::c_int) as
                                                       usize] =
                                        grab_fields(string.as_mut_ptr(),
                                                    0 as libc::c_int as
                                                        uint8_t) as
                                            uint8_t; // only used by ublox
                                    GPS_svinfo_quality[(svSatNum as
                                                            libc::c_int -
                                                            1 as libc::c_int)
                                                           as usize] =
                                        0 as libc::c_int as uint8_t
                                }
                                _ => { }
                            }
                            GPS_svInfoReceivedCount =
                                GPS_svInfoReceivedCount.wrapping_add(1)
                        }
                    }
                }
                _ => { }
            }
            param = param.wrapping_add(1);
            offset = 0 as libc::c_int as uint8_t;
            if c as libc::c_int == '*' as i32 {
                checksum_param = 1 as libc::c_int as uint8_t
            } else {
                parity = (parity as libc::c_int ^ c as libc::c_int) as uint8_t
            }
        }
        13 | 10 => {
            if checksum_param != 0 {
                //parity checksum
                shiftPacketLog();
                let mut checksum: uint8_t =
                    (16 as libc::c_int *
                         (if string[0 as libc::c_int as usize] as libc::c_int
                                 >= 'A' as i32 {
                              (string[0 as libc::c_int as usize] as
                                   libc::c_int - 'A' as i32) +
                                  10 as libc::c_int
                          } else {
                              (string[0 as libc::c_int as usize] as
                                   libc::c_int) - '0' as i32
                          }) +
                         (if string[1 as libc::c_int as usize] as libc::c_int
                                 >= 'A' as i32 {
                              (string[1 as libc::c_int as usize] as
                                   libc::c_int - 'A' as i32) +
                                  10 as libc::c_int
                          } else {
                              (string[1 as libc::c_int as usize] as
                                   libc::c_int) - '0' as i32
                          })) as uint8_t;
                if checksum as libc::c_int == parity as libc::c_int {
                    *gpsPacketLogChar = '!' as i32 as libc::c_char;
                    GPS_packetCount = GPS_packetCount.wrapping_add(1);
                    match gps_frame as libc::c_int {
                        1 => {
                            *gpsPacketLogChar = 'g' as i32 as libc::c_char;
                            frameOK = 1 as libc::c_int as uint8_t;
                            if stateFlags as libc::c_int &
                                   GPS_FIX as libc::c_int != 0 {
                                gpsSol.llh.lat = gps_Msg.latitude;
                                gpsSol.llh.lon = gps_Msg.longitude;
                                gpsSol.numSat = gps_Msg.numSat;
                                gpsSol.llh.alt = gps_Msg.altitude;
                                gpsSol.hdop = gps_Msg.hdop
                            }
                        }
                        2 => {
                            *gpsPacketLogChar = 'r' as i32 as libc::c_char;
                            gpsSol.groundSpeed = gps_Msg.speed;
                            gpsSol.groundCourse = gps_Msg.ground_course;
                            // end switch
                            // This check will miss 00:00:00.00, but we shouldn't care - next report will be valid
                            if !rtcHasTime() &&
                                   gps_Msg.date !=
                                       0 as libc::c_int as libc::c_uint &&
                                   gps_Msg.time !=
                                       0 as libc::c_int as libc::c_uint {
                                let mut temp_time: dateTime_t =
                                    dateTime_t{year: 0,
                                               month: 0,
                                               day: 0,
                                               hours: 0,
                                               minutes: 0,
                                               seconds: 0,
                                               millis: 0,};
                                temp_time.year =
                                    gps_Msg.date.wrapping_rem(100 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint).wrapping_add(2000
                                                                                                 as
                                                                                                 libc::c_int
                                                                                                 as
                                                                                                 libc::c_uint)
                                        as uint16_t;
                                temp_time.month =
                                    gps_Msg.date.wrapping_div(100 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint).wrapping_rem(100
                                                                                                 as
                                                                                                 libc::c_int
                                                                                                 as
                                                                                                 libc::c_uint)
                                        as uint8_t;
                                temp_time.day =
                                    gps_Msg.date.wrapping_div(10000 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint).wrapping_rem(100
                                                                                                 as
                                                                                                 libc::c_int
                                                                                                 as
                                                                                                 libc::c_uint)
                                        as uint8_t;
                                temp_time.hours =
                                    gps_Msg.time.wrapping_div(1000000 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint).wrapping_rem(100
                                                                                                 as
                                                                                                 libc::c_int
                                                                                                 as
                                                                                                 libc::c_uint)
                                        as uint8_t;
                                temp_time.minutes =
                                    gps_Msg.time.wrapping_div(10000 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint).wrapping_rem(100
                                                                                                 as
                                                                                                 libc::c_int
                                                                                                 as
                                                                                                 libc::c_uint)
                                        as uint8_t;
                                temp_time.seconds =
                                    gps_Msg.time.wrapping_div(100 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint).wrapping_rem(100
                                                                                                 as
                                                                                                 libc::c_int
                                                                                                 as
                                                                                                 libc::c_uint)
                                        as uint8_t;
                                temp_time.millis =
                                    (gps_Msg.time &
                                         100 as libc::c_int as
                                             libc::c_uint).wrapping_mul(10 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                        as uint16_t;
                                rtcSetDateTime(&mut temp_time);
                            }
                        }
                        _ => { }
                    }
                } else { *gpsPacketLogChar = '?' as i32 as libc::c_char }
            }
            checksum_param = 0 as libc::c_int as uint8_t
        }
        _ => {
            if (offset as libc::c_int) < 15 as libc::c_int {
                let fresh0 = offset;
                offset = offset.wrapping_add(1);
                string[fresh0 as usize] = c
            }
            if checksum_param == 0 {
                parity = (parity as libc::c_int ^ c as libc::c_int) as uint8_t
            }
        }
    }
    return frameOK != 0;
}
#[no_mangle]
pub static mut ubx_protocol_bytes: C2RustUnnamed_7 = MSG_ACK_NACK;
#[no_mangle]
pub static mut ubs_nav_fix_type: C2RustUnnamed_8 = FIX_NONE;
#[no_mangle]
pub static mut ubx_nav_status_bits: C2RustUnnamed_9 = 0 as C2RustUnnamed_9;
// Packet checksum accumulators
static mut _ck_a: uint8_t = 0;
static mut _ck_b: uint8_t = 0;
// State machine state
static mut _skip_packet: bool = false;
static mut _step: uint8_t = 0;
static mut _msg_id: uint8_t = 0;
static mut _payload_length: uint16_t = 0;
static mut _payload_counter: uint16_t = 0;
static mut next_fix: bool = false;
static mut _class: uint8_t = 0;
// do we have new position information?
static mut _new_position: bool = false;
// do we have new speed information?
static mut _new_speed: bool = false;
static mut _buffer: C2RustUnnamed_3 =
    C2RustUnnamed_3{posllh:
                        ubx_nav_posllh{time: 0,
                                       longitude: 0,
                                       latitude: 0,
                                       altitude_ellipsoid: 0,
                                       altitude_msl: 0,
                                       horizontal_accuracy: 0,
                                       vertical_accuracy: 0,},};
#[no_mangle]
pub unsafe extern "C" fn _update_checksum(mut data: *mut uint8_t,
                                          mut len: uint8_t,
                                          mut ck_a: *mut uint8_t,
                                          mut ck_b: *mut uint8_t) {
    loop  {
        let fresh1 = len;
        len = len.wrapping_sub(1);
        if !(fresh1 != 0) { break ; }
        *ck_a = (*ck_a as libc::c_int + *data as libc::c_int) as uint8_t;
        *ck_b = (*ck_b as libc::c_int + *ck_a as libc::c_int) as uint8_t;
        data = data.offset(1)
    };
}
unsafe extern "C" fn UBLOX_parse_gps() -> bool {
    let mut i: uint32_t = 0;
    *gpsPacketLogChar = '!' as i32 as libc::c_char;
    match _msg_id as libc::c_int {
        2 => {
            *gpsPacketLogChar = 'P' as i32 as libc::c_char;
            //i2c_dataset.time                = _buffer.posllh.time;
            gpsSol.llh.lon = _buffer.posllh.longitude; //alt in cm
            gpsSol.llh.lat = _buffer.posllh.latitude;
            gpsSol.llh.alt = _buffer.posllh.altitude_msl / 10 as libc::c_int;
            if next_fix {
                stateFlags =
                    (stateFlags as libc::c_int | GPS_FIX as libc::c_int) as
                        uint8_t
            } else {
                stateFlags =
                    (stateFlags as libc::c_int & !(GPS_FIX as libc::c_int)) as
                        uint8_t
            }
            _new_position = 1 as libc::c_int != 0
        }
        3 => {
            *gpsPacketLogChar = 'S' as i32 as libc::c_char;
            next_fix =
                _buffer.status.fix_status as libc::c_int &
                    NAV_STATUS_FIX_VALID as libc::c_int != 0 &&
                    _buffer.status.fix_type as libc::c_int ==
                        FIX_3D as libc::c_int;
            if !next_fix {
                stateFlags =
                    (stateFlags as libc::c_int & !(GPS_FIX as libc::c_int)) as
                        uint8_t
            }
        }
        6 => {
            *gpsPacketLogChar = 'O' as i32 as libc::c_char;
            next_fix =
                _buffer.solution.fix_status as libc::c_int &
                    NAV_STATUS_FIX_VALID as libc::c_int != 0 &&
                    _buffer.solution.fix_type as libc::c_int ==
                        FIX_3D as libc::c_int;
            if !next_fix {
                stateFlags =
                    (stateFlags as libc::c_int & !(GPS_FIX as libc::c_int)) as
                        uint8_t
            }
            gpsSol.numSat = _buffer.solution.satellites;
            gpsSol.hdop = _buffer.solution.position_DOP;
            //set clock, when gps time is available
            if !rtcHasTime() &&
                   _buffer.solution.fix_status as libc::c_int &
                       NAV_STATUS_TIME_SECOND_VALID as libc::c_int != 0 &&
                   _buffer.solution.fix_status as libc::c_int &
                       NAV_STATUS_TIME_WEEK_VALID as libc::c_int != 0 {
                //calculate rtctime: week number * ms in a week + ms of week + fractions of second + offset to UNIX reference year - 18 leap seconds
                let mut temp_time: rtcTime_t =
                    ((_buffer.solution.week as int64_t *
                          7 as libc::c_int as libc::c_long *
                          24 as libc::c_int as libc::c_long *
                          60 as libc::c_int as libc::c_long *
                          60 as libc::c_int as libc::c_long *
                          1000 as libc::c_int as libc::c_long +
                          _buffer.solution.time as libc::c_long +
                          (_buffer.solution.time_nsec /
                               1000000 as libc::c_int) as libc::c_long) as
                         libc::c_longlong + 315964800000 as libc::c_longlong -
                         18000 as libc::c_int as libc::c_longlong) as
                        rtcTime_t;
                rtcSet(&mut temp_time);
            }
        }
        18 => {
            *gpsPacketLogChar = 'V' as i32 as libc::c_char;
            // speed_3d                        = _buffer.velned.speed_3d;  // cm/s
            gpsSol.groundSpeed = _buffer.velned.speed_2d as uint16_t; // cm/s
            gpsSol.groundCourse =
                (_buffer.velned.heading_2d / 10000 as libc::c_int) as
                    uint16_t; // Heading 2D deg * 100000 rescaled to deg * 10
            _new_speed = 1 as libc::c_int != 0
        }
        48 => {
            *gpsPacketLogChar = 'I' as i32 as libc::c_char;
            GPS_numCh = _buffer.svinfo.numCh;
            if GPS_numCh as libc::c_int > 16 as libc::c_int {
                GPS_numCh = 16 as libc::c_int as uint8_t
            }
            i = 0 as libc::c_int as uint32_t;
            while i < GPS_numCh as libc::c_uint {
                GPS_svinfo_chn[i as usize] =
                    _buffer.svinfo.channel[i as usize].chn;
                GPS_svinfo_svid[i as usize] =
                    _buffer.svinfo.channel[i as usize].svid;
                GPS_svinfo_quality[i as usize] =
                    _buffer.svinfo.channel[i as usize].quality;
                GPS_svinfo_cno[i as usize] =
                    _buffer.svinfo.channel[i as usize].cno;
                i = i.wrapping_add(1)
            }
            GPS_svInfoReceivedCount = GPS_svInfoReceivedCount.wrapping_add(1)
        }
        _ => { return 0 as libc::c_int != 0 }
    }
    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if _new_position as libc::c_int != 0 && _new_speed as libc::c_int != 0 {
        _new_position = 0 as libc::c_int != 0;
        _new_speed = _new_position;
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn gpsNewFrameUBLOX(mut data: uint8_t) -> bool {
    let mut parsed: bool = 0 as libc::c_int != 0;
    match _step as libc::c_int {
        0 => {
            // Sync char 1 (0xB5)
            if PREAMBLE1 as libc::c_int == data as libc::c_int {
                _skip_packet = 0 as libc::c_int != 0;
                _step = _step.wrapping_add(1)
            }
        }
        1 => {
            // Sync char 2 (0x62)
            if PREAMBLE2 as libc::c_int != data as libc::c_int {
                _step = 0 as libc::c_int as uint8_t
            } else { _step = _step.wrapping_add(1) }
        }
        2 => {
            // Class
            _step = _step.wrapping_add(1); // reset the checksum accumulators
            _class = data;
            _ck_a = data;
            _ck_b = _ck_a
        }
        3 => {
            // Id
            _step = _step.wrapping_add(1); // checksum byte
            _ck_a = (_ck_a as libc::c_int + data as libc::c_int) as uint8_t;
            _ck_b = (_ck_b as libc::c_int + _ck_a as libc::c_int) as uint8_t;
            _msg_id = data
        }
        4 => {
            // Payload length (part 1)
            _step = _step.wrapping_add(1); // checksum byte
            _ck_a =
                (_ck_a as libc::c_int + data as libc::c_int) as
                    uint8_t; // payload length low byte
            _ck_b = (_ck_b as libc::c_int + _ck_a as libc::c_int) as uint8_t;
            _payload_length = data as uint16_t
        }
        5 => {
            // Payload length (part 2)
            _step = _step.wrapping_add(1); // checksum byte
            _ck_a =
                (_ck_a as libc::c_int + data as libc::c_int) as
                    uint8_t; // prepare to receive payload
            _ck_b =
                (_ck_b as libc::c_int + _ck_a as libc::c_int) as
                    uint8_t; // checksum byte
            _payload_length =
                (_payload_length as libc::c_int +
                     ((data as libc::c_int) << 8 as libc::c_int) as uint16_t
                         as libc::c_int) as uint16_t; // bad checksum
            if _payload_length as libc::c_int > 344 as libc::c_int {
                _skip_packet = 1 as libc::c_int != 0
            }
            _payload_counter = 0 as libc::c_int as uint16_t;
            if _payload_length as libc::c_int == 0 as libc::c_int {
                _step = 7 as libc::c_int as uint8_t
            }
        }
        6 => {
            _ck_a = (_ck_a as libc::c_int + data as libc::c_int) as uint8_t;
            _ck_b = (_ck_b as libc::c_int + _ck_a as libc::c_int) as uint8_t;
            if (_payload_counter as libc::c_int) < 344 as libc::c_int {
                _buffer.bytes[_payload_counter as usize] = data
            }
            _payload_counter = _payload_counter.wrapping_add(1);
            if _payload_counter as libc::c_int >=
                   _payload_length as libc::c_int {
                _step = _step.wrapping_add(1)
            }
        }
        7 => {
            _step = _step.wrapping_add(1);
            if _ck_a as libc::c_int != data as libc::c_int {
                _skip_packet = 1 as libc::c_int != 0;
                gpsData.errors = gpsData.errors.wrapping_add(1)
            }
        }
        8 => {
            _step = 0 as libc::c_int as uint8_t;
            shiftPacketLog();
            if _ck_b as libc::c_int != data as libc::c_int {
                *gpsPacketLogChar = '?' as i32 as libc::c_char;
                gpsData.errors = gpsData.errors.wrapping_add(1)
                // bad checksum
            } else {
                GPS_packetCount = GPS_packetCount.wrapping_add(1);
                if _skip_packet {
                    *gpsPacketLogChar = '>' as i32 as libc::c_char
                } else if UBLOX_parse_gps() { parsed = 1 as libc::c_int != 0 }
            }
        }
        _ => { }
    }
    return parsed;
}
// USE_GPS_UBLOX
unsafe extern "C" fn gpsHandlePassthrough(mut data: uint8_t) {
    gpsNewData(data as uint16_t);
    if feature(FEATURE_DASHBOARD as libc::c_int as uint32_t) {
        dashboardUpdate(micros());
    };
}
#[no_mangle]
pub unsafe extern "C" fn gpsEnablePassthrough(mut gpsPassthroughPort:
                                                  *mut serialPort_t) {
    waitForSerialPortToFinishTransmitting(gpsPort);
    waitForSerialPortToFinishTransmitting(gpsPassthroughPort);
    if (*gpsPort).mode as libc::c_uint &
           MODE_TX as libc::c_int as libc::c_uint == 0 {
        serialSetMode(gpsPort,
                      ((*gpsPort).mode as libc::c_uint |
                           MODE_TX as libc::c_int as libc::c_uint) as
                          portMode_e);
    }
    if feature(FEATURE_DASHBOARD as libc::c_int as uint32_t) {
        dashboardShowFixedPage(PAGE_GPS);
    }
    serialPassthrough(gpsPort, gpsPassthroughPort,
                      Some(gpsHandlePassthrough as
                               unsafe extern "C" fn(_: uint8_t) -> ()), None);
}
#[no_mangle]
pub static mut GPS_scaleLonDown: libc::c_float = 1.0f32;
// this is used to offset the shrinking longitude as we go towards the poles
#[no_mangle]
pub unsafe extern "C" fn GPS_calc_longitude_scaling(mut lat: int32_t) {
    let mut rads: libc::c_float =
        ({
             let mut _x: libc::c_float =
                 lat as
                     libc::c_float; // need an initial value for distance and bearing calc
             (if _x > 0 as libc::c_int as libc::c_float { _x } else { -_x })
         }) / 10000000.0f32 * 0.0174532925f32;
    GPS_scaleLonDown = cos_approx(rads);
}
#[no_mangle]
pub unsafe extern "C" fn GPS_reset_home_position() {
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 &&
           gpsSol.numSat as libc::c_int >= 5 as libc::c_int {
        GPS_home[0 as libc::c_int as usize] = gpsSol.llh.lat;
        GPS_home[1 as libc::c_int as usize] = gpsSol.llh.lon;
        GPS_calc_longitude_scaling(gpsSol.llh.lat);
        // Set ground altitude
        stateFlags =
            (stateFlags as libc::c_int | GPS_FIX_HOME as libc::c_int) as
                uint8_t
    };
}
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
#[no_mangle]
pub unsafe extern "C" fn GPS_distance_cm_bearing(mut currentLat1:
                                                     *mut int32_t,
                                                 mut currentLon1:
                                                     *mut int32_t,
                                                 mut destinationLat2:
                                                     *mut int32_t,
                                                 mut destinationLon2:
                                                     *mut int32_t,
                                                 mut dist: *mut uint32_t,
                                                 mut bearing: *mut int32_t) {
    let mut dLat: libc::c_float =
        (*destinationLat2 - *currentLat1) as
            libc::c_float; // difference of latitude in 1/10 000 000 degrees
    let mut dLon: libc::c_float =
        (*destinationLon2 - *currentLon1) as libc::c_float *
            GPS_scaleLonDown; // Convert the output radians to 100xdeg
    *dist = (sqrtf(dLat * dLat + dLon * dLon) * 1.113195f32) as uint32_t;
    *bearing =
        (9000.0f32 + atan2_approx(-dLat, dLon) * 5729.57795f32) as int32_t;
    if *bearing < 0 as libc::c_int { *bearing += 36000 as libc::c_int };
}
#[no_mangle]
pub unsafe extern "C" fn GPS_calculateDistanceAndDirectionToHome() {
    if stateFlags as libc::c_int & GPS_FIX_HOME as libc::c_int != 0 {
        // If we don't have home set, do not display anything
        let mut dist: uint32_t = 0;
        let mut dir: int32_t = 0;
        GPS_distance_cm_bearing(&mut gpsSol.llh.lat, &mut gpsSol.llh.lon,
                                &mut *GPS_home.as_mut_ptr().offset(0 as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                &mut *GPS_home.as_mut_ptr().offset(1 as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                &mut dist, &mut dir);
        GPS_distanceToHome =
            dist.wrapping_div(100 as libc::c_int as libc::c_uint) as uint16_t;
        GPS_directionToHome = (dir / 100 as libc::c_int) as int16_t
    } else {
        GPS_distanceToHome = 0 as libc::c_int as uint16_t;
        GPS_directionToHome = 0 as libc::c_int as int16_t
    };
}
// //////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps position data
//
unsafe extern "C" fn GPS_calc_velocity() {
    static mut speed_old: [int16_t; 2] =
        [0 as libc::c_int as int16_t, 0 as libc::c_int as int16_t];
    static mut last_coord: [int32_t; 2] =
        [0 as libc::c_int, 0 as libc::c_int];
    static mut init: uint8_t = 0 as libc::c_int as uint8_t;
    if init != 0 {
        let mut tmp: libc::c_float = 1.0f32 / dTnav;
        actual_speed[1 as libc::c_int as usize] =
            ((gpsSol.llh.lon - last_coord[1 as libc::c_int as usize]) as
                 libc::c_float * GPS_scaleLonDown * tmp) as int16_t;
        actual_speed[0 as libc::c_int as usize] =
            ((gpsSol.llh.lat - last_coord[0 as libc::c_int as usize]) as
                 libc::c_float * tmp) as int16_t;
        actual_speed[1 as libc::c_int as usize] =
            ((actual_speed[1 as libc::c_int as usize] as libc::c_int +
                  speed_old[1 as libc::c_int as usize] as libc::c_int) /
                 2 as libc::c_int) as int16_t;
        actual_speed[0 as libc::c_int as usize] =
            ((actual_speed[0 as libc::c_int as usize] as libc::c_int +
                  speed_old[0 as libc::c_int as usize] as libc::c_int) /
                 2 as libc::c_int) as int16_t;
        speed_old[1 as libc::c_int as usize] =
            actual_speed[1 as libc::c_int as usize];
        speed_old[0 as libc::c_int as usize] =
            actual_speed[0 as libc::c_int as usize]
    }
    init = 1 as libc::c_int as uint8_t;
    last_coord[1 as libc::c_int as usize] = gpsSol.llh.lon;
    last_coord[0 as libc::c_int as usize] = gpsSol.llh.lat;
}
#[no_mangle]
pub unsafe extern "C" fn onGpsNewData() {
    if !(stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 &&
             gpsSol.numSat as libc::c_int >= 5 as libc::c_int) {
        return
    }
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        stateFlags =
            (stateFlags as libc::c_int & !(GPS_FIX_HOME as libc::c_int)) as
                uint8_t
    }
    if stateFlags as libc::c_int & GPS_FIX_HOME as libc::c_int == 0 &&
           armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        GPS_reset_home_position();
    }
    // Apply moving average filter to GPS data
    GPS_filter_index =
        ((GPS_filter_index as libc::c_int + 1 as libc::c_int) %
             5 as libc::c_int) as
            uint8_t; // latest unfiltered data is in GPS_latitude and GPS_longitude
    let mut axis: libc::c_int =
        0 as
            libc::c_int; // get the degree to assure the sum fits to the int32_t
    while axis < 2 as libc::c_int {
        GPS_read[axis as usize] =
            if axis == 0 as libc::c_int {
                gpsSol.llh.lat
            } else { gpsSol.llh.lon };
        GPS_degree[axis as usize] =
            GPS_read[axis as usize] / 10000000 as libc::c_int;
        // How close we are to a degree line ? its the first three digits from the fractions of degree
        // later we use it to Check if we are close to a degree line, if yes, disable averaging,
        fraction3[axis as usize] =
            ((GPS_read[axis as usize] -
                  GPS_degree[axis as usize] * 10000000 as libc::c_int) /
                 10000 as libc::c_int) as uint16_t;
        GPS_filter_sum[axis as usize] -=
            GPS_filter[axis as usize][GPS_filter_index as usize];
        GPS_filter[axis as usize][GPS_filter_index as usize] =
            GPS_read[axis as usize] -
                GPS_degree[axis as usize] * 10000000 as libc::c_int;
        GPS_filter_sum[axis as usize] +=
            GPS_filter[axis as usize][GPS_filter_index as usize];
        GPS_filtered[axis as usize] =
            GPS_filter_sum[axis as usize] / 5 as libc::c_int +
                GPS_degree[axis as usize] * 10000000 as libc::c_int;
        if nav_mode as libc::c_uint ==
               NAV_MODE_POSHOLD as libc::c_int as libc::c_uint {
            // we use gps averaging only in poshold mode...
            if fraction3[axis as usize] as libc::c_int > 1 as libc::c_int &&
                   (fraction3[axis as usize] as libc::c_int) <
                       999 as libc::c_int {
                if axis == 0 as libc::c_int {
                    gpsSol.llh.lat = GPS_filtered[0 as libc::c_int as usize]
                } else {
                    gpsSol.llh.lon = GPS_filtered[1 as libc::c_int as usize]
                }
            }
        }
        axis += 1
    }
    //
    // Calculate time delta for navigation loop, range 0-1.0f, in seconds
    //
    // Time for calculating x,y speed and navigation pids
    static mut nav_loopTimer: uint32_t = 0;
    dTnav = millis().wrapping_sub(nav_loopTimer) as libc::c_float / 1000.0f32;
    nav_loopTimer = millis();
    // prevent runup from bad GPS
    dTnav =
        ({
             let mut _a: libc::c_float = dTnav;
             let mut _b: libc::c_float = 1.0f32;
             if _a < _b { _a } else { _b }
         });
    GPS_calculateDistanceAndDirectionToHome();
    // calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
    GPS_calc_velocity();
    rescueNewGpsData();
}
