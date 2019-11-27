use core;
use libc;
extern "C" {
    #[no_mangle]
    fn scaleRange(x: libc::c_int, srcFrom: libc::c_int, srcTo: libc::c_int,
                  destFrom: libc::c_int, destTo: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    static baudRates: [uint32_t; 0];
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn determinePortSharing(portConfig_0: *const serialPortConfig_t,
                            function: serialPortFunction_e) -> portSharing_e;
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
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
    #[no_mangle]
    fn getRssi() -> uint16_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    static mut telemetrySharedPort: *mut serialPort_t;
    #[no_mangle]
    fn telemetryCheckRxPortShared(portConfig_0: *const serialPortConfig_t)
     -> bool;
    #[no_mangle]
    fn telemetryDetermineEnabledState(portSharing: portSharing_e) -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
pub type timeMs_t = uint32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed = 2;
pub const ARMED: C2RustUnnamed = 1;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_0 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_0 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_0 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_0 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_0 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_0 = 16;
pub const BARO_MODE: C2RustUnnamed_0 = 8;
pub const MAG_MODE: C2RustUnnamed_0 = 4;
pub const HORIZON_MODE: C2RustUnnamed_0 = 2;
pub const ANGLE_MODE: C2RustUnnamed_0 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_1 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_1 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_1 = 4;
pub const GPS_FIX: C2RustUnnamed_1 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_1 = 1;
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
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
pub type portSharing_e = libc::c_uint;
pub const PORTSHARING_SHARED: portSharing_e = 2;
pub const PORTSHARING_NOT_SHARED: portSharing_e = 1;
pub const PORTSHARING_UNUSED: portSharing_e = 0;
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
pub const BAUD_115200: baudRate_e = 5;
pub const BAUD_57600: baudRate_e = 4;
pub const BAUD_38400: baudRate_e = 3;
pub const BAUD_19200: baudRate_e = 2;
pub const BAUD_9600: baudRate_e = 1;
pub const BAUD_AUTO: baudRate_e = 0;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPortConfig_s {
    pub functionMask: uint16_t,
    pub identifier: serialPortIdentifier_e,
    pub msp_baudrateIndex: uint8_t,
    pub gps_baudrateIndex: uint8_t,
    pub blackbox_baudrateIndex: uint8_t,
    pub telemetry_baudrateIndex: uint8_t,
}
// 0 - 359
// 0 - 255
// 0 - 255
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_2,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct C2RustUnnamed_2 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type frskyGpsCoordFormat_e = libc::c_uint;
pub const FRSKY_FORMAT_NMEA: frskyGpsCoordFormat_e = 1;
pub const FRSKY_FORMAT_DMS: frskyGpsCoordFormat_e = 0;
pub type frskyUnit_e = libc::c_uint;
pub const FRSKY_UNIT_IMPERIALS: frskyUnit_e = 1;
pub const FRSKY_UNIT_METRICS: frskyUnit_e = 0;
#[derive ( Copy, Clone )]
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
pub type telemetryConfig_t = telemetryConfig_s;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
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
unsafe extern "C" fn telemetryConfig() -> *const telemetryConfig_t {
    return &mut telemetryConfig_System;
}
static mut ltmPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut portConfig: *mut serialPortConfig_t =
    0 as *const serialPortConfig_t as *mut serialPortConfig_t;
static mut ltmEnabled: bool = false;
static mut ltmPortSharing: portSharing_e = PORTSHARING_UNUSED;
static mut ltm_crc: uint8_t = 0;
unsafe extern "C" fn ltm_initialise_packet(mut ltm_id: uint8_t) {
    ltm_crc = 0i32 as uint8_t;
    serialWrite(ltmPort, '$' as i32 as uint8_t);
    serialWrite(ltmPort, 'T' as i32 as uint8_t);
    serialWrite(ltmPort, ltm_id);
}
unsafe extern "C" fn ltm_serialise_8(mut v: uint8_t) {
    serialWrite(ltmPort, v);
    ltm_crc = (ltm_crc as libc::c_int ^ v as libc::c_int) as uint8_t;
}
unsafe extern "C" fn ltm_serialise_16(mut v: uint16_t) {
    ltm_serialise_8(v as uint8_t);
    ltm_serialise_8((v as libc::c_int >> 8i32) as uint8_t);
}
unsafe extern "C" fn ltm_serialise_32(mut v: uint32_t) {
    ltm_serialise_8(v as uint8_t);
    ltm_serialise_8((v >> 8i32) as uint8_t);
    ltm_serialise_8((v >> 16i32) as uint8_t);
    ltm_serialise_8((v >> 24i32) as uint8_t);
}
unsafe extern "C" fn ltm_finalise() { serialWrite(ltmPort, ltm_crc); }
// not used for all telemetry systems, e.g. HoTT only works at 19200.
/*
 * GPS G-frame 5Hhz at > 2400 baud
 * LAT LON SPD ALT SAT/FIX
 */
unsafe extern "C" fn ltm_gframe() { }
/*
 * Sensors S-frame 5Hhz at > 2400 baud
 * VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD
 * Flight mode(0-19):
 *     0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon,
 *     4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
 *     8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints,
 *     11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe,
 *     15: LAND, 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
 */
unsafe extern "C" fn ltm_sframe() {
    let mut lt_flightmode: uint8_t = 0; // Rate mode
    let mut lt_statemode: uint8_t = 0; //vbat converted to mv
    if flightModeFlags as libc::c_int & PASSTHRU_MODE as libc::c_int != 0 {
        lt_flightmode = 0i32 as uint8_t
    } else if flightModeFlags as libc::c_int & GPS_HOME_MODE as libc::c_int !=
                  0 {
        lt_flightmode = 13i32 as uint8_t
    } else if flightModeFlags as libc::c_int & GPS_HOLD_MODE as libc::c_int !=
                  0 {
        lt_flightmode = 9i32 as uint8_t
    } else if flightModeFlags as libc::c_int & HEADFREE_MODE as libc::c_int !=
                  0 {
        lt_flightmode = 4i32 as uint8_t
    } else if flightModeFlags as libc::c_int & BARO_MODE as libc::c_int != 0 {
        lt_flightmode = 8i32 as uint8_t
    } else if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int != 0
     {
        lt_flightmode = 2i32 as uint8_t
    } else if flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int !=
                  0 {
        lt_flightmode = 3i32 as uint8_t
    } else { lt_flightmode = 1i32 as uint8_t } //  current, not implemented
    lt_statemode =
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            1i32
        } else { 0i32 } as uint8_t; // scaled RSSI (uchar)
    if failsafeIsActive() {
        lt_statemode = (lt_statemode as libc::c_int | 2i32) as uint8_t
    } // no airspeed
    ltm_initialise_packet('S' as i32 as uint8_t);
    ltm_serialise_16((getBatteryVoltage() as libc::c_int * 100i32) as
                         uint16_t);
    ltm_serialise_16(0i32 as uint16_t);
    ltm_serialise_8(constrain(scaleRange(getRssi() as libc::c_int, 0i32,
                                         1023i32, 0i32, 255i32), 0i32, 255i32)
                        as uint8_t);
    ltm_serialise_8(0i32 as uint8_t);
    ltm_serialise_8(((lt_flightmode as libc::c_int) << 2i32 |
                         lt_statemode as libc::c_int) as uint8_t);
    ltm_finalise();
}
/*
 * Attitude A-frame - 10 Hz at > 2400 baud
 *  PITCH ROLL HEADING
 */
unsafe extern "C" fn ltm_aframe() {
    ltm_initialise_packet('A' as i32 as uint8_t);
    ltm_serialise_16((attitude.values.pitch as libc::c_int / 10i32) as
                         uint16_t);
    ltm_serialise_16((attitude.values.roll as libc::c_int / 10i32) as
                         uint16_t);
    ltm_serialise_16((attitude.values.yaw as libc::c_int / 10i32) as
                         uint16_t);
    ltm_finalise();
}
/*
 * OSD additional data frame, 1 Hz rate
 *  This frame will be ignored by Ghettostation, but processed by GhettOSD if it is used as standalone onboard OSD
 *  home pos, home alt, direction to home
 */
unsafe extern "C" fn ltm_oframe() {
    ltm_initialise_packet('O' as i32 as
                              uint8_t); // Don't have GPS home altitude
    ltm_serialise_32(0i32 as uint32_t); // OSD always ON
    ltm_serialise_32(0i32 as uint32_t);
    ltm_serialise_32(0i32 as uint32_t);
    ltm_serialise_8(1i32 as uint8_t);
    ltm_serialise_8(if stateFlags as libc::c_int & GPS_FIX_HOME as libc::c_int
                           != 0 {
                        1i32
                    } else { 0i32 } as uint8_t);
    ltm_finalise();
}
unsafe extern "C" fn process_ltm() {
    static mut ltm_scheduler: uint8_t = 0;
    ltm_aframe();
    if ltm_scheduler as libc::c_int & 1i32 != 0 {
        ltm_gframe();
    } else { ltm_sframe(); }
    if ltm_scheduler as libc::c_int == 0i32 { ltm_oframe(); }
    ltm_scheduler = ltm_scheduler.wrapping_add(1);
    ltm_scheduler = (ltm_scheduler as libc::c_int % 10i32) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn handleLtmTelemetry() {
    static mut ltm_lastCycleTime: uint32_t = 0;
    let mut now: uint32_t = 0;
    if !ltmEnabled { return }
    if ltmPort.is_null() { return }
    now = millis();
    if now.wrapping_sub(ltm_lastCycleTime) >= 100i32 as libc::c_uint {
        process_ltm();
        ltm_lastCycleTime = now
    };
}
#[no_mangle]
pub unsafe extern "C" fn freeLtmTelemetryPort() {
    closeSerialPort(ltmPort);
    ltmPort = 0 as *mut serialPort_t;
    ltmEnabled = 0i32 != 0;
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
pub unsafe extern "C" fn initLtmTelemetry() {
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_LTM);
    ltmPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_LTM);
}
#[no_mangle]
pub unsafe extern "C" fn configureLtmTelemetryPort() {
    if portConfig.is_null() { return }
    let mut baudRateIndex: baudRate_e =
        (*portConfig).telemetry_baudrateIndex as baudRate_e;
    if baudRateIndex as libc::c_uint ==
           BAUD_AUTO as libc::c_int as libc::c_uint {
        baudRateIndex = BAUD_19200
    }
    ltmPort =
        openSerialPort((*portConfig).identifier, FUNCTION_TELEMETRY_LTM, None,
                       0 as *mut libc::c_void,
                       *baudRates.as_ptr().offset(baudRateIndex as isize),
                       MODE_TX,
                       if (*telemetryConfig()).telemetry_inverted as
                              libc::c_int != 0 {
                           SERIAL_INVERTED as libc::c_int
                       } else { SERIAL_NOT_INVERTED as libc::c_int } as
                           portOptions_e);
    if ltmPort.is_null() { return }
    ltmEnabled = 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn checkLtmTelemetryState() {
    if !portConfig.is_null() &&
           telemetryCheckRxPortShared(portConfig) as libc::c_int != 0 {
        if !ltmEnabled && !telemetrySharedPort.is_null() {
            ltmPort = telemetrySharedPort;
            ltmEnabled = 1i32 != 0
        }
    } else {
        let mut newTelemetryEnabledValue: bool =
            telemetryDetermineEnabledState(ltmPortSharing);
        if newTelemetryEnabledValue as libc::c_int ==
               ltmEnabled as libc::c_int {
            return
        }
        if newTelemetryEnabledValue {
            configureLtmTelemetryPort();
        } else { freeLtmTelemetryPort(); }
    };
}
