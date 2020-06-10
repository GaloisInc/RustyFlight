use ::libc;
extern "C" {
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn findSerialPortIndexByIdentifier(identifier: serialPortIdentifier_e)
     -> libc::c_int;
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type ioTag_t = uint8_t;
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
pub const SERIALRX_SBUS: C2RustUnnamed_0 = 2;
pub const FUNCTION_RX_SERIAL: C2RustUnnamed = 64;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: C2RustUnnamed = 4;
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
    pub portConfigs: [serialPortConfig_t; 6],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
pub type C2RustUnnamed = libc::c_uint;
pub const FUNCTION_LIDAR_TF: C2RustUnnamed = 32768;
pub const FUNCTION_RCDEVICE: C2RustUnnamed = 16384;
pub const FUNCTION_VTX_TRAMP: C2RustUnnamed = 8192;
pub const FUNCTION_TELEMETRY_IBUS: C2RustUnnamed = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: C2RustUnnamed = 2048;
pub const FUNCTION_ESC_SENSOR: C2RustUnnamed = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: C2RustUnnamed = 512;
pub const FUNCTION_BLACKBOX: C2RustUnnamed = 128;
pub const FUNCTION_TELEMETRY_SMARTPORT: C2RustUnnamed = 32;
pub const FUNCTION_TELEMETRY_LTM: C2RustUnnamed = 16;
pub const FUNCTION_TELEMETRY_HOTT: C2RustUnnamed = 8;
pub const FUNCTION_GPS: C2RustUnnamed = 2;
pub const FUNCTION_MSP: C2RustUnnamed = 1;
pub const FUNCTION_NONE: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const SERIALRX_FPORT: C2RustUnnamed_0 = 12;
pub const SERIALRX_TARGET_CUSTOM: C2RustUnnamed_0 = 11;
pub const SERIALRX_SRXL: C2RustUnnamed_0 = 10;
pub const SERIALRX_CRSF: C2RustUnnamed_0 = 9;
pub const SERIALRX_JETIEXBUS: C2RustUnnamed_0 = 8;
pub const SERIALRX_IBUS: C2RustUnnamed_0 = 7;
pub const SERIALRX_XBUS_MODE_B_RJ01: C2RustUnnamed_0 = 6;
pub const SERIALRX_XBUS_MODE_B: C2RustUnnamed_0 = 5;
pub const SERIALRX_SUMH: C2RustUnnamed_0 = 4;
pub const SERIALRX_SUMD: C2RustUnnamed_0 = 3;
pub const SERIALRX_SPEKTRUM2048: C2RustUnnamed_0 = 1;
pub const SERIALRX_SPEKTRUM1024: C2RustUnnamed_0 = 0;
#[inline]
unsafe extern "C" fn serialConfigMutable() -> *mut serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfigMutable() -> *mut rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn telemetryConfigMutable() -> *mut telemetryConfig_t {
    return &mut telemetryConfig_System;
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
// set default settings to match our target
#[no_mangle]
pub unsafe extern "C" fn targetConfiguration() {
    // use the same uart for frsky telemetry and SBUS, both non inverted
    let index: libc::c_int =
        findSerialPortIndexByIdentifier(SERIAL_PORT_USART2);
    (*serialConfigMutable()).portConfigs[index as usize].functionMask =
        (FUNCTION_TELEMETRY_FRSKY_HUB as libc::c_int |
             FUNCTION_RX_SERIAL as libc::c_int) as uint16_t;
    (*rxConfigMutable()).serialrx_provider =
        SERIALRX_SBUS as libc::c_int as uint8_t;
    (*rxConfigMutable()).serialrx_inverted = 1 as libc::c_int as uint8_t;
    (*telemetryConfigMutable()).telemetry_inverted =
        1 as libc::c_int as uint8_t;
}
