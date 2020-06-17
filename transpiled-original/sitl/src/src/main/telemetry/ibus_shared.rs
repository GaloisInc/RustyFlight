use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
// SITL (software in the loop) simulator
// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC
// file name to save config
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
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
 * FlySky iBus telemetry implementation by CraigJPerry.
 * Unit tests and some additions by Unitware
 *
 * Many thanks to Dave Borthwick's iBus telemetry dongle converter for
 * PIC 12F1572 (also distributed under GPLv3) which was referenced to
 * clarify the protocol.
 */
// #include <string.h>
//#include "common/utils.h"
//defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
unsafe extern "C" fn calculateChecksum(mut ibusPacket: *const uint8_t)
 -> uint16_t {
    let mut checksum: uint16_t = 0xffffi32 as uint16_t;
    let mut dataSize: uint8_t =
        (*ibusPacket.offset(0) as libc::c_int - 2i32) as uint8_t;
    let mut i: libc::c_uint = 0i32 as libc::c_uint;
    while i < dataSize as libc::c_uint {
        checksum =
            (checksum as libc::c_int -
                 *ibusPacket.offset(i as isize) as libc::c_int) as uint16_t;
        i = i.wrapping_add(1)
    }
    return checksum;
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
/*
 * The ibus_shared module implements the ibus telemetry packet handling
 * which is shared between the ibus serial rx and the ibus telemetry.
 *
 * There is only one 'user' active at any time, serial rx will open the
 * serial port if both functions are enabled at the same time
 */
// Avg Cell voltage
// battery current A * 100
// remaining battery percentage / mah drawn otherwise or fuel level no unit!
// throttle value / battery capacity
//Heading  0..360 deg, 0=north 2bytes
//2 bytes m/s *100
//2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
//2 bytes
//2 bytes m/s *100 signed
//2 bytes m/s *100 signed
//2 bytes m/s *100 signed
//2 bytes deg *100 signed
//2 bytes deg *100 signed
//2 bytes deg *100 signed
//2 bytes m/s *100
//2 bytes m/s *100 different unit than build-in sensor
//2 bytes dist from home m unsigned
//2 bytes
//2 bytes
// Pressure
// Odometer1
// Odometer2
// Speed 2bytes km/h
//4bytes signed WGS84 in degrees * 1E7
//4bytes signed WGS84 in degrees * 1E7
//4bytes signed!!! GPS alt m*100
//4bytes signed!!! Alt m*100
//4bytes signed MaxAlt m*100
// Altitude 2 bytes signed in m
//defined(TELEMETRY_IBUS_EXTENDED)
//defined(TELEMETRY) && defined(TELEMETRY_IBUS)
#[no_mangle]
pub unsafe extern "C" fn isChecksumOkIa6b(mut ibusPacket: *const uint8_t,
                                          length: uint8_t) -> bool {
    let mut calculatedChecksum: uint16_t = calculateChecksum(ibusPacket);
    // Note that there's a byte order swap to little endian here
    return calculatedChecksum as libc::c_int >> 8i32 ==
               *ibusPacket.offset((length as libc::c_int - 1i32) as isize) as
                   libc::c_int &&
               calculatedChecksum as libc::c_int & 0xffi32 ==
                   *ibusPacket.offset((length as libc::c_int - 2i32) as isize)
                       as libc::c_int;
}
