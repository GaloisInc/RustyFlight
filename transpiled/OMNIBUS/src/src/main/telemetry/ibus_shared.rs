use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
    let mut checksum: uint16_t = 0xffff as libc::c_int as uint16_t;
    let mut dataSize: uint8_t =
        (*ibusPacket.offset(0 as libc::c_int as isize) as libc::c_int -
             2 as libc::c_int) as uint8_t;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
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
    return calculatedChecksum as libc::c_int >> 8 as libc::c_int ==
               *ibusPacket.offset((length as libc::c_int - 1 as libc::c_int)
                                      as isize) as libc::c_int &&
               calculatedChecksum as libc::c_int & 0xff as libc::c_int ==
                   *ibusPacket.offset((length as libc::c_int -
                                           2 as libc::c_int) as isize) as
                       libc::c_int;
}
