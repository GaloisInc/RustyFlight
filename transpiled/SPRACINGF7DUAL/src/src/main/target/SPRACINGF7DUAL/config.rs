use ::libc;
extern "C" {
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    static mut barometerConfig_System: barometerConfig_t;
    #[no_mangle]
    fn targetSerialPortFunctionConfig(config: *mut targetSerialPortFunction_t,
                                      count: size_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type size_t = libc::c_ulong;
pub type ioTag_t = uint8_t;
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
pub type C2RustUnnamed = libc::c_uint;
pub const BARO_QMP6988: C2RustUnnamed = 6;
pub const BARO_LPS: C2RustUnnamed = 5;
pub const BARO_BMP280: C2RustUnnamed = 4;
pub const BARO_MS5611: C2RustUnnamed = 3;
pub const BARO_BMP085: C2RustUnnamed = 2;
pub const BARO_NONE: C2RustUnnamed = 1;
pub const BARO_DEFAULT: C2RustUnnamed = 0;
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
pub type barometerConfig_t = barometerConfig_s;
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
pub type targetSerialPortFunction_t = targetSerialPortFunction_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct targetSerialPortFunction_s {
    pub identifier: serialPortIdentifier_e,
    pub function: serialPortFunction_e,
}
#[inline]
unsafe extern "C" fn telemetryConfigMutable() -> *mut telemetryConfig_t {
    return &mut telemetryConfig_System;
}
#[inline]
unsafe extern "C" fn barometerConfigMutable() -> *mut barometerConfig_t {
    return &mut barometerConfig_System;
}
static mut targetSerialPortFunction: [targetSerialPortFunction_t; 1] =
    [{
         let mut init =
             targetSerialPortFunction_s{identifier: SERIAL_PORT_UART5,
                                        function:
                                            FUNCTION_TELEMETRY_SMARTPORT,};
         init
     }];
#[no_mangle]
pub unsafe extern "C" fn targetConfiguration() {
    (*barometerConfigMutable()).baro_hardware =
        BARO_DEFAULT as libc::c_int as uint8_t;
    targetSerialPortFunctionConfig(targetSerialPortFunction.as_mut_ptr(),
                                   (::core::mem::size_of::<[targetSerialPortFunction_t; 1]>()
                                        as
                                        libc::c_ulong).wrapping_div(::core::mem::size_of::<targetSerialPortFunction_t>()
                                                                        as
                                                                        libc::c_ulong));
    // change telemetry settings
    (*telemetryConfigMutable()).telemetry_inverted =
        1 as libc::c_int as uint8_t;
    (*telemetryConfigMutable()).halfDuplex = 1 as libc::c_int as uint8_t;
}
