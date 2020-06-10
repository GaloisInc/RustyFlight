use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
    // disable pulls in BIDIR RX mode
    // Define known line control states which may be passed up by underlying serial driver callback
    // used by serial drivers to return frames to app
    // Specified baud rate may not be allowed by an implementation, use serialGetBaudRate to determine actual baud rate in use.
    // Optional functions used to buffer large writes.
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn serialSetMode(instance: *mut serialPort_t, mode: portMode_e);
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn getEstimatedAltitude() -> int32_t;
    #[no_mangle]
    fn getEstimatedVario() -> int16_t;
    #[no_mangle]
    static mut GPS_distanceToHome: uint16_t;
    // distance to home point in meters
    #[no_mangle]
    static mut GPS_directionToHome: int16_t;
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getBatteryState() -> batteryState_e;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn determinePortSharing(portConfig_0: *const serialPortConfig_t,
                            function: serialPortFunction_e) -> portSharing_e;
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
    fn closeSerialPort(serialPort: *mut serialPort_t);
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
 * telemetry.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    fn telemetryDetermineEnabledState(portSharing: portSharing_e) -> bool;
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
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
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
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
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
pub type serialPort_t = serialPort_s;
pub type C2RustUnnamed = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed = 16;
pub const SMALL_ANGLE: C2RustUnnamed = 8;
pub const CALIBRATE_MAG: C2RustUnnamed = 4;
pub const GPS_FIX: C2RustUnnamed = 2;
pub const GPS_FIX_HOME: C2RustUnnamed = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
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
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const BATTERY_OK: batteryState_e = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_0 = 64;
pub const SENSOR_GPS: C2RustUnnamed_0 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_0 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_0 = 16;
pub const SENSOR_MAG: C2RustUnnamed_0 = 8;
pub const SENSOR_BARO: C2RustUnnamed_0 = 4;
pub const SENSOR_ACC: C2RustUnnamed_0 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_0 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const HOTT_EAM_ALARM1_FLAG_MAIN_VOLTAGE: C2RustUnnamed_1 = 128;
pub const HOTT_EAM_ALARM1_FLAG_CURRENT: C2RustUnnamed_1 = 64;
pub const HOTT_EAM_ALARM1_FLAG_ALTITUDE: C2RustUnnamed_1 = 32;
pub const HOTT_EAM_ALARM1_FLAG_TEMPERATURE_2: C2RustUnnamed_1 = 16;
pub const HOTT_EAM_ALARM1_FLAG_TEMPERATURE_1: C2RustUnnamed_1 = 8;
pub const HOTT_EAM_ALARM1_FLAG_BATTERY_2: C2RustUnnamed_1 = 4;
pub const HOTT_EAM_ALARM1_FLAG_BATTERY_1: C2RustUnnamed_1 = 2;
pub const HOTT_EAM_ALARM1_FLAG_MAH: C2RustUnnamed_1 = 1;
pub const HOTT_EAM_ALARM1_FLAG_NONE: C2RustUnnamed_1 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HOTT_EAM_MSG_s {
    pub start_byte: uint8_t,
    pub eam_sensor_id: uint8_t,
    pub warning_beeps: uint8_t,
    pub sensor_id: uint8_t,
    pub alarm_invers1: uint8_t,
    pub alarm_invers2: uint8_t,
    pub cell1_L: uint8_t,
    pub cell2_L: uint8_t,
    pub cell3_L: uint8_t,
    pub cell4_L: uint8_t,
    pub cell5_L: uint8_t,
    pub cell6_L: uint8_t,
    pub cell7_L: uint8_t,
    pub cell1_H: uint8_t,
    pub cell2_H: uint8_t,
    pub cell3_H: uint8_t,
    pub cell4_H: uint8_t,
    pub cell5_H: uint8_t,
    pub cell6_H: uint8_t,
    pub cell7_H: uint8_t,
    pub batt1_voltage_L: uint8_t,
    pub batt1_voltage_H: uint8_t,
    pub batt2_voltage_L: uint8_t,
    pub batt2_voltage_H: uint8_t,
    pub temp1: uint8_t,
    pub temp2: uint8_t,
    pub altitude_L: uint8_t,
    pub altitude_H: uint8_t,
    pub current_L: uint8_t,
    pub current_H: uint8_t,
    pub main_voltage_L: uint8_t,
    pub main_voltage_H: uint8_t,
    pub batt_cap_L: uint8_t,
    pub batt_cap_H: uint8_t,
    pub climbrate_L: uint8_t,
    pub climbrate_H: uint8_t,
    pub climbrate3s: uint8_t,
    pub rpm_L: uint8_t,
    pub rpm_H: uint8_t,
    pub electric_min: uint8_t,
    pub electric_sec: uint8_t,
    pub speed_L: uint8_t,
    pub speed_H: uint8_t,
    pub stop_byte: uint8_t,
}
pub type HOTT_EAM_MSG_t = HOTT_EAM_MSG_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HOTT_GPS_MSG_s {
    pub start_byte: uint8_t,
    pub gps_sensor_id: uint8_t,
    pub warning_beeps: uint8_t,
    pub sensor_id: uint8_t,
    pub alarm_invers1: uint8_t,
    pub alarm_invers2: uint8_t,
    pub flight_direction: uint8_t,
    pub gps_speed_L: uint8_t,
    pub gps_speed_H: uint8_t,
    pub pos_NS: uint8_t,
    pub pos_NS_dm_L: uint8_t,
    pub pos_NS_dm_H: uint8_t,
    pub pos_NS_sec_L: uint8_t,
    pub pos_NS_sec_H: uint8_t,
    pub pos_EW: uint8_t,
    pub pos_EW_dm_L: uint8_t,
    pub pos_EW_dm_H: uint8_t,
    pub pos_EW_sec_L: uint8_t,
    pub pos_EW_sec_H: uint8_t,
    pub home_distance_L: uint8_t,
    pub home_distance_H: uint8_t,
    pub altitude_L: uint8_t,
    pub altitude_H: uint8_t,
    pub climbrate_L: uint8_t,
    pub climbrate_H: uint8_t,
    pub climbrate3s: uint8_t,
    pub gps_satelites: uint8_t,
    pub gps_fix_char: uint8_t,
    pub home_direction: uint8_t,
    pub angle_roll: uint8_t,
    pub angle_nick: uint8_t,
    pub angle_compass: uint8_t,
    pub gps_time_h: uint8_t,
    pub gps_time_m: uint8_t,
    pub gps_time_s: uint8_t,
    pub gps_time_sss: uint8_t,
    pub msl_altitude_L: uint8_t,
    pub msl_altitude_H: uint8_t,
    pub vibration: uint8_t,
    pub free_char1: uint8_t,
    pub free_char2: uint8_t,
    pub free_char3: uint8_t,
    pub version: uint8_t,
    pub stop_byte: uint8_t,
}
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
//#01 start uint8_t
//#02 EAM sensort id. constat value 0x8e
//#03 1=A 2=B ... or 'A' - 0x40 = 1
// Q    Min cell voltage sensor 1
                                // R    Min Battery 1 voltage sensor 1
                                // J    Max Battery 1 voltage sensor 1
                                // F    Mim temperature sensor 1
                                // H    Max temperature sensor 1
                                // S    Min cell voltage sensor 2
                                // K    Max cell voltage sensor 2
                                // G    Min temperature sensor 2
                                // I    Max temperature sensor 2
                                // W    Max current
                                // V    Max capacity mAh
                                // P    Min main power voltage
                                // X    Max main power voltage
                                // O    Min altitude
                                // Z    Max altitude
                                // C    (negative) sink rate m/sec to high
                                // B    (negative) sink rate m/3sec to high
                                // N    climb rate m/sec to high
                                // M    climb rate m/3sec to high
//#04 constant value 0xe0
//#05 alarm bitmask. Value is displayed inverted
//Bit#  Alarm field
                                // 0    mAh
                                // 1    Battery 1
                                // 2    Battery 2
                                // 3    Temperature 1
                                // 4    Temperature 2
                                // 5    Altitude
                                // 6    Current
                                // 7    Main power voltage
//#06 alarm bitmask. Value is displayed inverted
//Bit#  Alarm Field
                                // 0    m/s
                                // 1    m/3s
                                // 2    Altitude (duplicate?)
                                // 3    m/s (duplicate?)
                                // 4    m/3s (duplicate?)
                                // 5    unknown/unused
                                // 6    unknown/unused
                                // 7    "ON" sign/text msg active
//#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
//#08
//#09
//#10
//#11
//#12
//#13
//#14 cell 1 voltage high value. 0.02V steps, 124=2.48V
//#15
//#16
//#17
//#18
//#19
//#20
//#21 battery 1 voltage lower value in 100mv steps, 50=5V. optionally cell8_L value 0.02V steps
//#22
//#23 battery 2 voltage lower value in 100mv steps, 50=5V. optionally cell8_H value. 0.02V steps
//#24
//#25 Temperature sensor 1. 20=0�, 46=26� - offset of 20.
//#26 temperature sensor 2
//#27 Attitude lower value. unit: meters. Value of 500 = 0m
//#28
//#29 Current in 0.1 steps
//#30
//#31 Main power voltage (drive) in 0.1V steps
//#32
//#33 used battery capacity in 10mAh steps
//#34
//#35 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
//#36
//#37 climbrate in m/3sec. Value of 120 = 0m/3sec
//#38 RPM. Steps: 10 U/min
//#39
//#40 Electric minutes. Time does start, when motor current is > 3 A
//#41
//#42 (air?) speed in km/h. Steps 1km/h
//#43
//#44 stop uint8_t
//HoTT GPS Sensor response to Receiver (?!not?! Smartbox)
pub type HOTT_GPS_MSG_t = HOTT_GPS_MSG_s;
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
//#01 constant value 0x7c
//#02 constant value 0x8a
//#03 1=A 2=B ...
// A    Min Speed
                        // L    Max Speed
                        // O    Min Altitude
                        // Z    Max Altitude
                        // C    (negative) sink rate m/sec to high
                        // B    (negative) sink rate m/3sec to high
                        // N    climb rate m/sec to high
                        // M    climb rate m/3sec to high
                        // D    Max home distance
                        //
//#04 constant (?) value 0xa0
//#05
//TODO: more info
//#06  1 = No GPS signal
//TODO: more info
//#07 flight direction in 2 degreees/step (1 = 2degrees);
//08 km/h
//#09
//#10 north = 0, south = 1
//#11 degree minutes ie N48�39�988
//#12
//#13 position seconds
//#14
//#15 east = 0, west = 1
//#16 degree minutes ie. E9�25�9360
//#17
//#18 position seconds
//#19
//#20 meters
//#21
//#22 meters. Value of 500 = 0m
//#23
//#24 m/s 0.01m/s resolution. Value of 30000 = 0.00 m/s
//#25
//#26 climbrate in m/3s resolution, value of 120 = 0 m/3s
//#27 sat count
//#28 GPS fix character. display, 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix. Where appears this char???
//#29 direction from starting point to Model position (2 degree steps)
//#30 angle roll in 2 degree steps
//#31 angle in 2degree steps
//#32 angle in 2degree steps. 1 = 2�, 255 = - 2� (1 uint8_t) North = 0�
//#33 UTC time hours
//#34 UTC time minutes
//#35 UTC time seconds
//#36 UTC time milliseconds
//#37 mean sea level altitude
//#38
//#39 vibrations level in %
//#40 appears right to home distance
//#41 appears right to home direction
//#42 GPS ASCII D=DGPS 2=2D 3=3D -=No Fix
//#43
// 0    GPS Graupner #33600
                        // 1    Gyro Receiver
                        // 255 Mikrokopter
//#44 constant value 0x7d
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
pub const GPS_FIX_CHAR_2D: C2RustUnnamed_2 = 50;
pub const GPS_FIX_CHAR_3D: C2RustUnnamed_2 = 51;
pub const GPS_FIX_CHAR_NONE: C2RustUnnamed_2 = 45;
pub type portSharing_e = libc::c_uint;
pub const PORTSHARING_SHARED: portSharing_e = 2;
pub const PORTSHARING_NOT_SHARED: portSharing_e = 1;
pub const PORTSHARING_UNUSED: portSharing_e = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const GPS_FIX_CHAR_DGPS: C2RustUnnamed_2 = 68;
#[inline]
unsafe extern "C" fn telemetryConfig() -> *const telemetryConfig_t {
    return &mut telemetryConfig_System;
}
static mut lastHoTTRequestCheckAt: uint32_t = 0 as libc::c_int as uint32_t;
static mut lastMessagesPreparedAt: uint32_t = 0 as libc::c_int as uint32_t;
static mut lastHottAlarmSoundTime: uint32_t = 0 as libc::c_int as uint32_t;
static mut hottIsSending: bool = 0 as libc::c_int != 0;
static mut hottMsg: *mut uint8_t = 0 as *const uint8_t as *mut uint8_t;
static mut hottMsgRemainingBytesToSendCount: uint8_t = 0;
static mut hottMsgCrc: uint8_t = 0;
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// must be opened in RXTX so that TX and RX pins are allocated.
static mut hottPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut portConfig: *mut serialPortConfig_t =
    0 as *const serialPortConfig_t as *mut serialPortConfig_t;
static mut hottTelemetryEnabled: bool = 0 as libc::c_int != 0;
static mut hottPortSharing: portSharing_e = PORTSHARING_UNUSED;
static mut hottGPSMessage: HOTT_GPS_MSG_t =
    HOTT_GPS_MSG_t{start_byte: 0,
                   gps_sensor_id: 0,
                   warning_beeps: 0,
                   sensor_id: 0,
                   alarm_invers1: 0,
                   alarm_invers2: 0,
                   flight_direction: 0,
                   gps_speed_L: 0,
                   gps_speed_H: 0,
                   pos_NS: 0,
                   pos_NS_dm_L: 0,
                   pos_NS_dm_H: 0,
                   pos_NS_sec_L: 0,
                   pos_NS_sec_H: 0,
                   pos_EW: 0,
                   pos_EW_dm_L: 0,
                   pos_EW_dm_H: 0,
                   pos_EW_sec_L: 0,
                   pos_EW_sec_H: 0,
                   home_distance_L: 0,
                   home_distance_H: 0,
                   altitude_L: 0,
                   altitude_H: 0,
                   climbrate_L: 0,
                   climbrate_H: 0,
                   climbrate3s: 0,
                   gps_satelites: 0,
                   gps_fix_char: 0,
                   home_direction: 0,
                   angle_roll: 0,
                   angle_nick: 0,
                   angle_compass: 0,
                   gps_time_h: 0,
                   gps_time_m: 0,
                   gps_time_s: 0,
                   gps_time_sss: 0,
                   msl_altitude_L: 0,
                   msl_altitude_H: 0,
                   vibration: 0,
                   free_char1: 0,
                   free_char2: 0,
                   free_char3: 0,
                   version: 0,
                   stop_byte: 0,};
static mut hottEAMMessage: HOTT_EAM_MSG_t =
    HOTT_EAM_MSG_t{start_byte: 0,
                   eam_sensor_id: 0,
                   warning_beeps: 0,
                   sensor_id: 0,
                   alarm_invers1: 0,
                   alarm_invers2: 0,
                   cell1_L: 0,
                   cell2_L: 0,
                   cell3_L: 0,
                   cell4_L: 0,
                   cell5_L: 0,
                   cell6_L: 0,
                   cell7_L: 0,
                   cell1_H: 0,
                   cell2_H: 0,
                   cell3_H: 0,
                   cell4_H: 0,
                   cell5_H: 0,
                   cell6_H: 0,
                   cell7_H: 0,
                   batt1_voltage_L: 0,
                   batt1_voltage_H: 0,
                   batt2_voltage_L: 0,
                   batt2_voltage_H: 0,
                   temp1: 0,
                   temp2: 0,
                   altitude_L: 0,
                   altitude_H: 0,
                   current_L: 0,
                   current_H: 0,
                   main_voltage_L: 0,
                   main_voltage_H: 0,
                   batt_cap_L: 0,
                   batt_cap_H: 0,
                   climbrate_L: 0,
                   climbrate_H: 0,
                   climbrate3s: 0,
                   rpm_L: 0,
                   rpm_H: 0,
                   electric_min: 0,
                   electric_sec: 0,
                   speed_L: 0,
                   speed_H: 0,
                   stop_byte: 0,};
unsafe extern "C" fn initialiseEAMMessage(mut msg: *mut HOTT_EAM_MSG_t,
                                          mut size: size_t) {
    memset(msg as *mut libc::c_void, 0 as libc::c_int, size);
    (*msg).start_byte = 0x7c as libc::c_int as uint8_t;
    (*msg).eam_sensor_id = 0x8e as libc::c_int as uint8_t;
    (*msg).sensor_id = 0xe0 as libc::c_int as uint8_t;
    (*msg).stop_byte = 0x7d as libc::c_int as uint8_t;
}
unsafe extern "C" fn initialiseGPSMessage(mut msg: *mut HOTT_GPS_MSG_t,
                                          mut size: size_t) {
    memset(msg as *mut libc::c_void, 0 as libc::c_int, size);
    (*msg).start_byte = 0x7c as libc::c_int as uint8_t;
    (*msg).gps_sensor_id = 0x8a as libc::c_int as uint8_t;
    (*msg).sensor_id = 0xa0 as libc::c_int as uint8_t;
    (*msg).stop_byte = 0x7d as libc::c_int as uint8_t;
}
unsafe extern "C" fn initialiseMessages() {
    initialiseEAMMessage(&mut hottEAMMessage,
                         ::core::mem::size_of::<HOTT_EAM_MSG_t>() as
                             libc::c_ulong);
    initialiseGPSMessage(&mut hottGPSMessage,
                         ::core::mem::size_of::<HOTT_GPS_MSG_t>() as
                             libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn addGPSCoordinates(mut hottGPSMessage_0:
                                               *mut HOTT_GPS_MSG_t,
                                           mut latitude: int32_t,
                                           mut longitude: int32_t) {
    let mut deg: int16_t =
        (latitude as libc::c_long / 10000000 as libc::c_long) as int16_t;
    let mut sec: int32_t =
        ((latitude as libc::c_long -
              deg as libc::c_long * 10000000 as libc::c_long) *
             6 as libc::c_int as libc::c_long) as int32_t;
    let mut min: int8_t =
        (sec as libc::c_long / 1000000 as libc::c_long) as int8_t;
    sec =
        (sec as libc::c_long % 1000000 as libc::c_long / 100 as libc::c_long)
            as int32_t;
    let mut degMin: uint16_t =
        (deg as libc::c_long * 100 as libc::c_long + min as libc::c_long) as
            uint16_t;
    (*hottGPSMessage_0).pos_NS =
        (latitude < 0 as libc::c_int) as libc::c_int as uint8_t;
    (*hottGPSMessage_0).pos_NS_dm_L = degMin as uint8_t;
    (*hottGPSMessage_0).pos_NS_dm_H =
        (degMin as libc::c_int >> 8 as libc::c_int) as uint8_t;
    (*hottGPSMessage_0).pos_NS_sec_L = sec as uint8_t;
    (*hottGPSMessage_0).pos_NS_sec_H = (sec >> 8 as libc::c_int) as uint8_t;
    deg = (longitude as libc::c_long / 10000000 as libc::c_long) as int16_t;
    sec =
        ((longitude as libc::c_long -
              deg as libc::c_long * 10000000 as libc::c_long) *
             6 as libc::c_int as libc::c_long) as int32_t;
    min = (sec as libc::c_long / 1000000 as libc::c_long) as int8_t;
    sec =
        (sec as libc::c_long % 1000000 as libc::c_long / 100 as libc::c_long)
            as int32_t;
    degMin =
        (deg as libc::c_long * 100 as libc::c_long + min as libc::c_long) as
            uint16_t;
    (*hottGPSMessage_0).pos_EW =
        (longitude < 0 as libc::c_int) as libc::c_int as uint8_t;
    (*hottGPSMessage_0).pos_EW_dm_L = degMin as uint8_t;
    (*hottGPSMessage_0).pos_EW_dm_H =
        (degMin as libc::c_int >> 8 as libc::c_int) as uint8_t;
    (*hottGPSMessage_0).pos_EW_sec_L = sec as uint8_t;
    (*hottGPSMessage_0).pos_EW_sec_H = (sec >> 8 as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn hottPrepareGPSResponse(mut hottGPSMessage_0:
                                                    *mut HOTT_GPS_MSG_t) {
    (*hottGPSMessage_0).gps_satelites = gpsSol.numSat;
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int == 0 {
        (*hottGPSMessage_0).gps_fix_char =
            GPS_FIX_CHAR_NONE as libc::c_int as uint8_t;
        return
    }
    if gpsSol.numSat as libc::c_int >= 5 as libc::c_int {
        (*hottGPSMessage_0).gps_fix_char =
            GPS_FIX_CHAR_3D as libc::c_int as uint8_t
    } else {
        (*hottGPSMessage_0).gps_fix_char =
            GPS_FIX_CHAR_2D as libc::c_int as uint8_t
    }
    addGPSCoordinates(hottGPSMessage_0, gpsSol.llh.lat, gpsSol.llh.lon);
    // GPS Speed is returned in cm/s (from io/gps.c) and must be sent in km/h (Hott requirement)
    let speed: uint16_t =
        (gpsSol.groundSpeed as libc::c_int * 36 as libc::c_int /
             1000 as libc::c_int) as
            uint16_t; // gpsSol.llh.alt in m ; offset = 500 -> O m
    (*hottGPSMessage_0).gps_speed_L =
        (speed as libc::c_int & 0xff as libc::c_int) as uint8_t;
    (*hottGPSMessage_0).gps_speed_H =
        (speed as libc::c_int >> 8 as libc::c_int) as uint8_t;
    (*hottGPSMessage_0).home_distance_L =
        (GPS_distanceToHome as libc::c_int & 0xff as libc::c_int) as uint8_t;
    (*hottGPSMessage_0).home_distance_H =
        (GPS_distanceToHome as libc::c_int >> 8 as libc::c_int) as uint8_t;
    let mut altitude: uint16_t = gpsSol.llh.alt as uint16_t;
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int == 0 {
        altitude = getEstimatedAltitude() as uint16_t
    }
    let hottGpsAltitude: uint16_t =
        (altitude as libc::c_int / 100 as libc::c_int + 500 as libc::c_int) as
            uint16_t;
    (*hottGPSMessage_0).altitude_L =
        (hottGpsAltitude as libc::c_int & 0xff as libc::c_int) as uint8_t;
    (*hottGPSMessage_0).altitude_H =
        (hottGpsAltitude as libc::c_int >> 8 as libc::c_int) as uint8_t;
    (*hottGPSMessage_0).home_direction = GPS_directionToHome as uint8_t;
}
unsafe extern "C" fn shouldTriggerBatteryAlarmNow() -> bool {
    return millis().wrapping_sub(lastHottAlarmSoundTime) >=
               ((*telemetryConfig()).hottAlarmSoundInterval as libc::c_int *
                    1000 as libc::c_int) as libc::c_uint;
}
#[inline]
unsafe extern "C" fn updateAlarmBatteryStatus(mut hottEAMMessage_0:
                                                  *mut HOTT_EAM_MSG_t) {
    if shouldTriggerBatteryAlarmNow() {
        lastHottAlarmSoundTime = millis();
        let batteryState: batteryState_e = getBatteryState();
        if batteryState as libc::c_uint ==
               BATTERY_WARNING as libc::c_int as libc::c_uint ||
               batteryState as libc::c_uint ==
                   BATTERY_CRITICAL as libc::c_int as libc::c_uint {
            (*hottEAMMessage_0).warning_beeps =
                0x10 as libc::c_int as uint8_t;
            (*hottEAMMessage_0).alarm_invers1 =
                HOTT_EAM_ALARM1_FLAG_BATTERY_1 as libc::c_int as uint8_t
        } else {
            (*hottEAMMessage_0).warning_beeps =
                HOTT_EAM_ALARM1_FLAG_NONE as libc::c_int as uint8_t;
            (*hottEAMMessage_0).alarm_invers1 =
                HOTT_EAM_ALARM1_FLAG_NONE as libc::c_int as uint8_t
        }
    };
}
#[inline]
unsafe extern "C" fn hottEAMUpdateBattery(mut hottEAMMessage_0:
                                              *mut HOTT_EAM_MSG_t) {
    (*hottEAMMessage_0).main_voltage_L =
        (getBatteryVoltage() as libc::c_int & 0xff as libc::c_int) as uint8_t;
    (*hottEAMMessage_0).main_voltage_H =
        (getBatteryVoltage() as libc::c_int >> 8 as libc::c_int) as uint8_t;
    (*hottEAMMessage_0).batt1_voltage_L =
        (getBatteryVoltage() as libc::c_int & 0xff as libc::c_int) as uint8_t;
    (*hottEAMMessage_0).batt1_voltage_H =
        (getBatteryVoltage() as libc::c_int >> 8 as libc::c_int) as uint8_t;
    updateAlarmBatteryStatus(hottEAMMessage_0);
}
#[inline]
unsafe extern "C" fn hottEAMUpdateCurrentMeter(mut hottEAMMessage_0:
                                                   *mut HOTT_EAM_MSG_t) {
    let amp: int32_t = getAmperage() / 10 as libc::c_int;
    (*hottEAMMessage_0).current_L = (amp & 0xff as libc::c_int) as uint8_t;
    (*hottEAMMessage_0).current_H = (amp >> 8 as libc::c_int) as uint8_t;
}
#[inline]
unsafe extern "C" fn hottEAMUpdateBatteryDrawnCapacity(mut hottEAMMessage_0:
                                                           *mut HOTT_EAM_MSG_t) {
    let mAh: int32_t = getMAhDrawn() / 10 as libc::c_int;
    (*hottEAMMessage_0).batt_cap_L = (mAh & 0xff as libc::c_int) as uint8_t;
    (*hottEAMMessage_0).batt_cap_H = (mAh >> 8 as libc::c_int) as uint8_t;
}
#[inline]
unsafe extern "C" fn hottEAMUpdateAltitude(mut hottEAMMessage_0:
                                               *mut HOTT_EAM_MSG_t) {
    let hottEamAltitude: uint16_t =
        (getEstimatedAltitude() / 100 as libc::c_int + 500 as libc::c_int) as
            uint16_t;
    (*hottEAMMessage_0).altitude_L =
        (hottEamAltitude as libc::c_int & 0xff as libc::c_int) as uint8_t;
    (*hottEAMMessage_0).altitude_H =
        (hottEamAltitude as libc::c_int >> 8 as libc::c_int) as uint8_t;
}
#[inline]
unsafe extern "C" fn hottEAMUpdateClimbrate(mut hottEAMMessage_0:
                                                *mut HOTT_EAM_MSG_t) {
    let vario: int32_t = getEstimatedVario() as int32_t;
    (*hottEAMMessage_0).climbrate_L =
        (30000 as libc::c_int + vario & 0xff as libc::c_int) as uint8_t;
    (*hottEAMMessage_0).climbrate_H =
        (30000 as libc::c_int + vario >> 8 as libc::c_int) as uint8_t;
    (*hottEAMMessage_0).climbrate3s =
        (120 as libc::c_int + vario / 100 as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn hottPrepareEAMResponse(mut hottEAMMessage_0:
                                                    *mut HOTT_EAM_MSG_t) {
    // Reset alarms
    (*hottEAMMessage_0).warning_beeps = 0 as libc::c_int as uint8_t;
    (*hottEAMMessage_0).alarm_invers1 = 0 as libc::c_int as uint8_t;
    hottEAMUpdateBattery(hottEAMMessage_0);
    hottEAMUpdateCurrentMeter(hottEAMMessage_0);
    hottEAMUpdateBatteryDrawnCapacity(hottEAMMessage_0);
    hottEAMUpdateAltitude(hottEAMMessage_0);
    hottEAMUpdateClimbrate(hottEAMMessage_0);
}
unsafe extern "C" fn hottSerialWrite(mut c: uint8_t) {
    static mut serialWrites: uint8_t = 0 as libc::c_int as uint8_t;
    serialWrites = serialWrites.wrapping_add(1);
    serialWrite(hottPort, c);
}
#[no_mangle]
pub unsafe extern "C" fn freeHoTTTelemetryPort() {
    closeSerialPort(hottPort);
    hottPort = 0 as *mut serialPort_t;
    hottTelemetryEnabled = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn initHoTTTelemetry() {
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_HOTT);
    hottPortSharing =
        determinePortSharing(portConfig, FUNCTION_TELEMETRY_HOTT);
    initialiseMessages();
}
unsafe extern "C" fn flushHottRxBuffer() {
    while serialRxBytesWaiting(hottPort) > 0 as libc::c_int as libc::c_uint {
        serialRead(hottPort);
    };
}
unsafe extern "C" fn workAroundForHottTelemetryOnUsart(mut instance:
                                                           *mut serialPort_t,
                                                       mut mode: portMode_e) {
    closeSerialPort(hottPort);
    let mut portOptions: portOptions_e =
        if (*telemetryConfig()).telemetry_inverted as libc::c_int != 0 {
            SERIAL_INVERTED as libc::c_int
        } else { SERIAL_NOT_INVERTED as libc::c_int } as portOptions_e;
    if (*telemetryConfig()).halfDuplex != 0 {
        portOptions =
            ::core::mem::transmute::<libc::c_uint,
                                     portOptions_e>(portOptions as
                                                        libc::c_uint |
                                                        SERIAL_BIDIR as
                                                            libc::c_int as
                                                            libc::c_uint)
    }
    hottPort =
        openSerialPort((*instance).identifier as serialPortIdentifier_e,
                       FUNCTION_TELEMETRY_HOTT, None, 0 as *mut libc::c_void,
                       19200 as libc::c_int as uint32_t, mode, portOptions);
}
unsafe extern "C" fn hottIsUsingHardwareUART() -> bool {
    return !((*portConfig).identifier as libc::c_int ==
                 SERIAL_PORT_SOFTSERIAL1 as libc::c_int ||
                 (*portConfig).identifier as libc::c_int ==
                     SERIAL_PORT_SOFTSERIAL2 as libc::c_int);
}
unsafe extern "C" fn hottConfigurePortForTX() {
    // FIXME temorary workaround for HoTT not working on Hardware serial ports due to hardware/softserial serial port initialisation differences
    if hottIsUsingHardwareUART() {
        workAroundForHottTelemetryOnUsart(hottPort, MODE_TX);
    } else { serialSetMode(hottPort, MODE_TX); }
    hottIsSending = 1 as libc::c_int != 0;
    hottMsgCrc = 0 as libc::c_int as uint8_t;
}
unsafe extern "C" fn hottConfigurePortForRX() {
    // FIXME temorary workaround for HoTT not working on Hardware serial ports due to hardware/softserial serial port initialisation differences
    if hottIsUsingHardwareUART() {
        workAroundForHottTelemetryOnUsart(hottPort, MODE_RX);
    } else { serialSetMode(hottPort, MODE_RX); }
    hottMsg = 0 as *mut uint8_t;
    hottIsSending = 0 as libc::c_int != 0;
    flushHottRxBuffer();
}
#[no_mangle]
pub unsafe extern "C" fn configureHoTTTelemetryPort() {
    if portConfig.is_null() { return }
    let mut portOptions: portOptions_e = SERIAL_NOT_INVERTED;
    if (*telemetryConfig()).halfDuplex != 0 {
        portOptions =
            ::core::mem::transmute::<libc::c_uint,
                                     portOptions_e>(portOptions as
                                                        libc::c_uint |
                                                        SERIAL_BIDIR as
                                                            libc::c_int as
                                                            libc::c_uint)
    }
    hottPort =
        openSerialPort((*portConfig).identifier, FUNCTION_TELEMETRY_HOTT,
                       None, 0 as *mut libc::c_void,
                       19200 as libc::c_int as uint32_t, MODE_RXTX,
                       portOptions);
    if hottPort.is_null() { return }
    hottConfigurePortForRX();
    hottTelemetryEnabled = 1 as libc::c_int != 0;
}
unsafe extern "C" fn hottSendResponse(mut buffer: *mut uint8_t,
                                      mut length: libc::c_int) {
    if hottIsSending { return }
    hottMsg = buffer;
    hottMsgRemainingBytesToSendCount =
        (length as
             libc::c_ulong).wrapping_add(::core::mem::size_of::<uint8_t>() as
                                             libc::c_ulong) as uint8_t;
}
#[inline]
unsafe extern "C" fn hottSendGPSResponse() {
    hottSendResponse(&mut hottGPSMessage as *mut HOTT_GPS_MSG_t as
                         *mut uint8_t,
                     ::core::mem::size_of::<HOTT_GPS_MSG_t>() as libc::c_ulong
                         as libc::c_int);
}
#[inline]
unsafe extern "C" fn hottSendEAMResponse() {
    hottSendResponse(&mut hottEAMMessage as *mut HOTT_EAM_MSG_t as
                         *mut uint8_t,
                     ::core::mem::size_of::<HOTT_EAM_MSG_t>() as libc::c_ulong
                         as libc::c_int);
}
unsafe extern "C" fn hottPrepareMessages() {
    hottPrepareEAMResponse(&mut hottEAMMessage);
    hottPrepareGPSResponse(&mut hottGPSMessage);
}
unsafe extern "C" fn processBinaryModeRequest(mut address: uint8_t) {
    match address as libc::c_int {
        138 => {
            if sensors(SENSOR_GPS as libc::c_int as uint32_t) {
                hottSendGPSResponse();
            }
        }
        142 => { hottSendEAMResponse(); }
        _ => { }
    };
}
unsafe extern "C" fn hottCheckSerialData(mut currentMicros: uint32_t) {
    static mut lookingForRequest: bool = 1 as libc::c_int != 0;
    let bytesWaiting: uint8_t = serialRxBytesWaiting(hottPort) as uint8_t;
    if bytesWaiting as libc::c_int <= 1 as libc::c_int { return }
    if bytesWaiting as libc::c_int != 2 as libc::c_int {
        flushHottRxBuffer();
        lookingForRequest = 1 as libc::c_int != 0;
        return
    }
    if lookingForRequest {
        lastHoTTRequestCheckAt = currentMicros;
        lookingForRequest = 0 as libc::c_int != 0;
        return
    } else {
        let mut enoughTimePassed: bool =
            currentMicros.wrapping_sub(lastHoTTRequestCheckAt) >=
                4000 as libc::c_int as libc::c_uint;
        if !enoughTimePassed { return }
        lookingForRequest = 1 as libc::c_int != 0
    }
    let requestId: uint8_t = serialRead(hottPort);
    let address: uint8_t = serialRead(hottPort);
    if requestId as libc::c_int == 0 as libc::c_int ||
           requestId as libc::c_int == 0x80 as libc::c_int ||
           address as libc::c_int == 0x80 as libc::c_int {
        /*
     * FIXME the first byte of the HoTT request frame is ONLY either 0x80 (binary mode) or 0x7F (text mode).
     * The binary mode is read as 0x00 (error reading the upper bit) while the text mode is correctly decoded.
     * The (requestId == 0) test is a workaround for detecting the binary mode with no ambiguity as there is only
     * one other valid value (0x7F) for text mode.
     * The error reading for the upper bit should nevertheless be fixed
     */
        processBinaryModeRequest(address);
    };
}
unsafe extern "C" fn hottSendTelemetryData() {
    if !hottIsSending { hottConfigurePortForTX(); return }
    if hottMsgRemainingBytesToSendCount as libc::c_int == 0 as libc::c_int {
        hottConfigurePortForRX();
        return
    }
    hottMsgRemainingBytesToSendCount =
        hottMsgRemainingBytesToSendCount.wrapping_sub(1);
    if hottMsgRemainingBytesToSendCount as libc::c_int == 0 as libc::c_int {
        let fresh0 = hottMsgCrc;
        hottMsgCrc = hottMsgCrc.wrapping_add(1);
        hottSerialWrite(fresh0);
        return
    }
    hottMsgCrc =
        (hottMsgCrc as libc::c_int + *hottMsg as libc::c_int) as uint8_t;
    let fresh1 = hottMsg;
    hottMsg = hottMsg.offset(1);
    hottSerialWrite(*fresh1);
}
#[inline]
unsafe extern "C" fn shouldPrepareHoTTMessages(mut currentMicros: uint32_t)
 -> bool {
    return currentMicros.wrapping_sub(lastMessagesPreparedAt) >=
               (1000 as libc::c_int * 1000 as libc::c_int / 5 as libc::c_int)
                   as libc::c_uint;
}
#[inline]
unsafe extern "C" fn shouldCheckForHoTTRequest() -> bool {
    if hottIsSending { return 0 as libc::c_int != 0 }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn checkHoTTTelemetryState() {
    let newTelemetryEnabledValue: bool =
        telemetryDetermineEnabledState(hottPortSharing);
    if newTelemetryEnabledValue as libc::c_int ==
           hottTelemetryEnabled as libc::c_int {
        return
    }
    if newTelemetryEnabledValue {
        configureHoTTTelemetryPort();
    } else { freeHoTTTelemetryPort(); };
}
#[no_mangle]
pub unsafe extern "C" fn handleHoTTTelemetry(mut currentTimeUs: timeUs_t) {
    static mut serialTimer: timeUs_t = 0;
    if !hottTelemetryEnabled { return }
    if shouldPrepareHoTTMessages(currentTimeUs) {
        hottPrepareMessages();
        lastMessagesPreparedAt = currentTimeUs
    }
    if shouldCheckForHoTTRequest() { hottCheckSerialData(currentTimeUs); }
    if hottMsg.is_null() { return }
    if hottIsSending {
        if currentTimeUs.wrapping_sub(serialTimer) <
               3000 as libc::c_int as libc::c_uint {
            return
        }
    }
    hottSendTelemetryData();
    serialTimer = currentTimeUs;
}
