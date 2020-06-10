use ::libc;
extern "C" {
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    fn isModeActivationConditionPresent(modeId: boxId_e) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn handleFrSkyHubTelemetry(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn checkFrSkyHubTelemetryState();
    #[no_mangle]
    fn initFrSkyHubTelemetry() -> bool;
    #[no_mangle]
    fn handleHoTTTelemetry(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn checkHoTTTelemetryState();
    #[no_mangle]
    fn initHoTTTelemetry();
    #[no_mangle]
    fn initSmartPortTelemetry() -> bool;
    #[no_mangle]
    fn checkSmartPortTelemetryState();
    #[no_mangle]
    fn handleSmartPortTelemetry();
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
// Define known line control states which may be passed up by underlying serial driver callback
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
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
pub type portSharing_e = libc::c_uint;
pub const PORTSHARING_SHARED: portSharing_e = 2;
pub const PORTSHARING_NOT_SHARED: portSharing_e = 1;
pub const PORTSHARING_UNUSED: portSharing_e = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FUNCTION_LIDAR_TF: C2RustUnnamed_1 = 32768;
pub const FUNCTION_RCDEVICE: C2RustUnnamed_1 = 16384;
pub const FUNCTION_VTX_TRAMP: C2RustUnnamed_1 = 8192;
pub const FUNCTION_TELEMETRY_IBUS: C2RustUnnamed_1 = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: C2RustUnnamed_1 = 2048;
pub const FUNCTION_ESC_SENSOR: C2RustUnnamed_1 = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: C2RustUnnamed_1 = 512;
pub const FUNCTION_BLACKBOX: C2RustUnnamed_1 = 128;
pub const FUNCTION_RX_SERIAL: C2RustUnnamed_1 = 64;
pub const FUNCTION_TELEMETRY_SMARTPORT: C2RustUnnamed_1 = 32;
pub const FUNCTION_TELEMETRY_LTM: C2RustUnnamed_1 = 16;
pub const FUNCTION_TELEMETRY_HOTT: C2RustUnnamed_1 = 8;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: C2RustUnnamed_1 = 4;
pub const FUNCTION_GPS: C2RustUnnamed_1 = 2;
pub const FUNCTION_MSP: C2RustUnnamed_1 = 1;
pub const FUNCTION_NONE: C2RustUnnamed_1 = 0;
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
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
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
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const SERIALRX_FPORT: C2RustUnnamed_3 = 12;
pub const SERIALRX_TARGET_CUSTOM: C2RustUnnamed_3 = 11;
pub const SERIALRX_SRXL: C2RustUnnamed_3 = 10;
pub const SERIALRX_CRSF: C2RustUnnamed_3 = 9;
pub const SERIALRX_JETIEXBUS: C2RustUnnamed_3 = 8;
pub const SERIALRX_IBUS: C2RustUnnamed_3 = 7;
pub const SERIALRX_XBUS_MODE_B_RJ01: C2RustUnnamed_3 = 6;
pub const SERIALRX_XBUS_MODE_B: C2RustUnnamed_3 = 5;
pub const SERIALRX_SUMH: C2RustUnnamed_3 = 4;
pub const SERIALRX_SUMD: C2RustUnnamed_3 = 3;
pub const SERIALRX_SBUS: C2RustUnnamed_3 = 2;
pub const SERIALRX_SPEKTRUM2048: C2RustUnnamed_3 = 1;
pub const SERIALRX_SPEKTRUM1024: C2RustUnnamed_3 = 0;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const IBUS_SENSOR_TYPE_UNKNOWN: C2RustUnnamed_4 = 255;
pub const IBUS_SENSOR_TYPE_ALT_FLYSKY: C2RustUnnamed_4 = 249;
pub const IBUS_SENSOR_TYPE_ALT_MAX: C2RustUnnamed_4 = 132;
pub const IBUS_SENSOR_TYPE_ALT: C2RustUnnamed_4 = 131;
pub const IBUS_SENSOR_TYPE_GPS_ALT: C2RustUnnamed_4 = 130;
pub const IBUS_SENSOR_TYPE_GPS_LON: C2RustUnnamed_4 = 129;
pub const IBUS_SENSOR_TYPE_GPS_LAT: C2RustUnnamed_4 = 128;
pub const IBUS_SENSOR_TYPE_SPE: C2RustUnnamed_4 = 126;
pub const IBUS_SENSOR_TYPE_ODO2: C2RustUnnamed_4 = 125;
pub const IBUS_SENSOR_TYPE_ODO1: C2RustUnnamed_4 = 124;
pub const IBUS_SENSOR_TYPE_PRES: C2RustUnnamed_4 = 65;
pub const IBUS_SENSOR_TYPE_FLIGHT_MODE: C2RustUnnamed_4 = 22;
pub const IBUS_SENSOR_TYPE_ARMED: C2RustUnnamed_4 = 21;
pub const IBUS_SENSOR_TYPE_GPS_DIST: C2RustUnnamed_4 = 20;
pub const IBUS_SENSOR_TYPE_GROUND_SPEED: C2RustUnnamed_4 = 19;
pub const IBUS_SENSOR_TYPE_VERTICAL_SPEED: C2RustUnnamed_4 = 18;
pub const IBUS_SENSOR_TYPE_YAW: C2RustUnnamed_4 = 17;
pub const IBUS_SENSOR_TYPE_PITCH: C2RustUnnamed_4 = 16;
pub const IBUS_SENSOR_TYPE_ROLL: C2RustUnnamed_4 = 15;
pub const IBUS_SENSOR_TYPE_ACC_Z: C2RustUnnamed_4 = 14;
pub const IBUS_SENSOR_TYPE_ACC_Y: C2RustUnnamed_4 = 13;
pub const IBUS_SENSOR_TYPE_ACC_X: C2RustUnnamed_4 = 12;
pub const IBUS_SENSOR_TYPE_GPS_STATUS: C2RustUnnamed_4 = 11;
pub const IBUS_SENSOR_TYPE_COG: C2RustUnnamed_4 = 10;
pub const IBUS_SENSOR_TYPE_CLIMB_RATE: C2RustUnnamed_4 = 9;
pub const IBUS_SENSOR_TYPE_CMP_HEAD: C2RustUnnamed_4 = 8;
pub const IBUS_SENSOR_TYPE_RPM: C2RustUnnamed_4 = 7;
pub const IBUS_SENSOR_TYPE_FUEL: C2RustUnnamed_4 = 6;
pub const IBUS_SENSOR_TYPE_BAT_CURR: C2RustUnnamed_4 = 5;
pub const IBUS_SENSOR_TYPE_CELL: C2RustUnnamed_4 = 4;
pub const IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE: C2RustUnnamed_4 = 3;
pub const IBUS_SENSOR_TYPE_RPM_FLYSKY: C2RustUnnamed_4 = 2;
pub const IBUS_SENSOR_TYPE_TEMPERATURE: C2RustUnnamed_4 = 1;
pub const IBUS_SENSOR_TYPE_NONE: C2RustUnnamed_4 = 0;
pub type frskyGpsCoordFormat_e = libc::c_uint;
pub const FRSKY_FORMAT_NMEA: frskyGpsCoordFormat_e = 1;
pub const FRSKY_FORMAT_DMS: frskyGpsCoordFormat_e = 0;
pub type frskyUnit_e = libc::c_uint;
pub const FRSKY_UNIT_IMPERIALS: frskyUnit_e = 1;
pub const FRSKY_UNIT_METRICS: frskyUnit_e = 0;
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
pub type telemetryConfig_t = telemetryConfig_s;
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
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
pub static mut telemetryConfig_System: telemetryConfig_t =
    telemetryConfig_t{gpsNoFixLatitude: 0,
                      gpsNoFixLongitude: 0,
                      telemetry_inverted: 0,
                      halfDuplex: 0,
                      frsky_coordinate_format: FRSKY_FORMAT_DMS,
                      frsky_unit: FRSKY_UNIT_METRICS,
                      frsky_vfas_precision: 0,
                      hottAlarmSoundInterval: 0,
                      pidValuesAsTelemetry: 0,
                      report_cell_voltage: 0,
                      flysky_sensors: [0; 15],
                      smartport_use_extra_sensors: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut telemetryConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (31 as libc::c_int |
                                      (2 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<telemetryConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &telemetryConfig_System as
                                     *const telemetryConfig_t as
                                     *mut telemetryConfig_t as *mut uint8_t,
                             copy:
                                 &telemetryConfig_Copy as
                                     *const telemetryConfig_t as
                                     *mut telemetryConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_telemetryConfig
                                                         as
                                                         *const telemetryConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut telemetryConfig_Copy: telemetryConfig_t =
    telemetryConfig_t{gpsNoFixLatitude: 0,
                      gpsNoFixLongitude: 0,
                      telemetry_inverted: 0,
                      halfDuplex: 0,
                      frsky_coordinate_format: FRSKY_FORMAT_DMS,
                      frsky_unit: FRSKY_UNIT_METRICS,
                      frsky_vfas_precision: 0,
                      hottAlarmSoundInterval: 0,
                      pidValuesAsTelemetry: 0,
                      report_cell_voltage: 0,
                      flysky_sensors: [0; 15],
                      smartport_use_extra_sensors: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_telemetryConfig: telemetryConfig_t =
    {
        let mut init =
            telemetryConfig_s{gpsNoFixLatitude: 0 as libc::c_int as int16_t,
                              gpsNoFixLongitude: 0 as libc::c_int as int16_t,
                              telemetry_inverted: 0 as libc::c_int as uint8_t,
                              halfDuplex: 1 as libc::c_int as uint8_t,
                              frsky_coordinate_format: FRSKY_FORMAT_DMS,
                              frsky_unit: FRSKY_UNIT_METRICS,
                              frsky_vfas_precision:
                                  0 as libc::c_int as uint8_t,
                              hottAlarmSoundInterval:
                                  5 as libc::c_int as uint8_t,
                              pidValuesAsTelemetry:
                                  0 as libc::c_int as uint8_t,
                              report_cell_voltage:
                                  0 as libc::c_int as uint8_t,
                              flysky_sensors:
                                  [IBUS_SENSOR_TYPE_TEMPERATURE as libc::c_int
                                       as uint8_t,
                                   IBUS_SENSOR_TYPE_RPM_FLYSKY as libc::c_int
                                       as uint8_t,
                                   IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE as
                                       libc::c_int as uint8_t, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0, 0],
                              smartport_use_extra_sensors:
                                  0 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn telemetryInit() {
    initFrSkyHubTelemetry();
    initHoTTTelemetry();
    initSmartPortTelemetry();
    telemetryCheckState();
}
#[no_mangle]
pub unsafe extern "C" fn telemetryDetermineEnabledState(mut portSharing:
                                                            portSharing_e)
 -> bool {
    let mut enabled: bool =
        portSharing as libc::c_uint ==
            PORTSHARING_NOT_SHARED as libc::c_int as libc::c_uint;
    if portSharing as libc::c_uint ==
           PORTSHARING_SHARED as libc::c_int as libc::c_uint {
        if isModeActivationConditionPresent(BOXTELEMETRY) {
            enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY)
        } else {
            enabled = armingFlags as libc::c_int & ARMED as libc::c_int != 0
        }
    }
    return enabled;
}
#[no_mangle]
pub unsafe extern "C" fn telemetryCheckRxPortShared(mut portConfig:
                                                        *const serialPortConfig_t)
 -> bool {
    if (*portConfig).functionMask as libc::c_int &
           FUNCTION_RX_SERIAL as libc::c_int != 0 &&
           (*portConfig).functionMask as libc::c_int &
               (FUNCTION_TELEMETRY_FRSKY_HUB as libc::c_int |
                    FUNCTION_TELEMETRY_LTM as libc::c_int |
                    FUNCTION_TELEMETRY_MAVLINK as libc::c_int) != 0 &&
           ((*rxConfig()).serialrx_provider as libc::c_int ==
                SERIALRX_SPEKTRUM1024 as libc::c_int ||
                (*rxConfig()).serialrx_provider as libc::c_int ==
                    SERIALRX_SPEKTRUM2048 as libc::c_int ||
                (*rxConfig()).serialrx_provider as libc::c_int ==
                    SERIALRX_SBUS as libc::c_int ||
                (*rxConfig()).serialrx_provider as libc::c_int ==
                    SERIALRX_SUMD as libc::c_int ||
                (*rxConfig()).serialrx_provider as libc::c_int ==
                    SERIALRX_SUMH as libc::c_int ||
                (*rxConfig()).serialrx_provider as libc::c_int ==
                    SERIALRX_XBUS_MODE_B as libc::c_int ||
                (*rxConfig()).serialrx_provider as libc::c_int ==
                    SERIALRX_XBUS_MODE_B_RJ01 as libc::c_int ||
                (*rxConfig()).serialrx_provider as libc::c_int ==
                    SERIALRX_IBUS as libc::c_int) {
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub static mut telemetrySharedPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
#[no_mangle]
pub unsafe extern "C" fn telemetryCheckState() {
    checkFrSkyHubTelemetryState();
    checkHoTTTelemetryState();
    checkSmartPortTelemetryState();
}
#[no_mangle]
pub unsafe extern "C" fn telemetryProcess(mut currentTime: uint32_t) {
    handleFrSkyHubTelemetry(currentTime);
    handleHoTTTelemetry(currentTime);
    handleSmartPortTelemetry();
}
