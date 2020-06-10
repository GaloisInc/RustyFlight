use ::libc;
extern "C" {
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const FUNCTION_LIDAR_TF: C2RustUnnamed = 32768;
pub const FUNCTION_RCDEVICE: C2RustUnnamed = 16384;
pub const FUNCTION_VTX_TRAMP: C2RustUnnamed = 8192;
pub const FUNCTION_TELEMETRY_IBUS: C2RustUnnamed = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: C2RustUnnamed = 2048;
pub const FUNCTION_ESC_SENSOR: C2RustUnnamed = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: C2RustUnnamed = 512;
pub const FUNCTION_BLACKBOX: C2RustUnnamed = 128;
pub const FUNCTION_RX_SERIAL: C2RustUnnamed = 64;
pub const FUNCTION_TELEMETRY_SMARTPORT: C2RustUnnamed = 32;
pub const FUNCTION_TELEMETRY_LTM: C2RustUnnamed = 16;
pub const FUNCTION_TELEMETRY_HOTT: C2RustUnnamed = 8;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: C2RustUnnamed = 4;
pub const FUNCTION_GPS: C2RustUnnamed = 2;
pub const FUNCTION_MSP: C2RustUnnamed = 1;
pub const FUNCTION_NONE: C2RustUnnamed = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 3],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
pub type serialConfig_t = serialConfig_s;
#[inline]
unsafe extern "C" fn serialConfigMutable() -> *mut serialConfig_t {
    return &mut serialConfig_System;
}
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
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
pub unsafe extern "C" fn targetConfiguration() {
    (*serialConfigMutable()).portConfigs[1 as libc::c_int as
                                             usize].functionMask =
        FUNCTION_MSP as libc::c_int as uint16_t;
    // To connect to FC.
}
