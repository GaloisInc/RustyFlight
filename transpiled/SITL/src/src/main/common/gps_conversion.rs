use ::libc;
extern "C" {
    #[no_mangle]
    fn isdigit(_: libc::c_int) -> libc::c_int;
}
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
#[no_mangle]
pub unsafe extern "C" fn GPS_coord_to_degrees(mut coordinateString:
                                                  *const libc::c_char)
 -> uint32_t {
    let mut fieldSeparator: *const libc::c_char = 0 as *const libc::c_char;
    let mut remainingString: *const libc::c_char = 0 as *const libc::c_char;
    let mut degress: uint8_t = 0 as libc::c_int as uint8_t;
    let mut minutes: uint8_t = 0 as libc::c_int as uint8_t;
    let mut fractionalMinutes: uint16_t = 0 as libc::c_int as uint16_t;
    let mut digitIndex: uint8_t = 0;
    // scan for decimal point or end of field
    fieldSeparator = coordinateString;
    while isdigit(*fieldSeparator as libc::c_uchar as libc::c_int) != 0 {
        if fieldSeparator >=
               coordinateString.offset(15 as libc::c_int as isize) {
            return 0 as libc::c_int as uint32_t
        }
        fieldSeparator = fieldSeparator.offset(1)
        // stop potential fail
    }
    remainingString = coordinateString;
    // convert degrees
    while fieldSeparator.wrapping_offset_from(remainingString) as libc::c_long
              > 2 as libc::c_int as libc::c_long {
        if degress != 0 {
            degress = (degress as libc::c_int * 10 as libc::c_int) as uint8_t
        }
        let fresh0 = remainingString;
        remainingString = remainingString.offset(1);
        degress =
            (degress as libc::c_int + (*fresh0 as libc::c_int - '0' as i32))
                as uint8_t
    }
    // convert minutes
    while fieldSeparator > remainingString {
        if minutes != 0 {
            minutes = (minutes as libc::c_int * 10 as libc::c_int) as uint8_t
        }
        let fresh1 = remainingString;
        remainingString = remainingString.offset(1);
        minutes =
            (minutes as libc::c_int + (*fresh1 as libc::c_int - '0' as i32))
                as uint8_t
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if *fieldSeparator as libc::c_int == '.' as i32 {
        remainingString = fieldSeparator.offset(1 as libc::c_int as isize);
        digitIndex = 0 as libc::c_int as uint8_t;
        while (digitIndex as libc::c_int) < 4 as libc::c_int {
            fractionalMinutes =
                (fractionalMinutes as libc::c_int * 10 as libc::c_int) as
                    uint16_t;
            if isdigit(*remainingString as libc::c_uchar as libc::c_int) != 0
               {
                let fresh2 = remainingString;
                remainingString = remainingString.offset(1);
                fractionalMinutes =
                    (fractionalMinutes as libc::c_int +
                         (*fresh2 as libc::c_int - '0' as i32)) as uint16_t
            }
            digitIndex = digitIndex.wrapping_add(1)
        }
    }
    return (degress as
                libc::c_ulong).wrapping_mul(10000000 as
                                                libc::c_ulong).wrapping_add((minutes
                                                                                 as
                                                                                 libc::c_ulong).wrapping_mul(1000000
                                                                                                                 as
                                                                                                                 libc::c_ulong).wrapping_add((fractionalMinutes
                                                                                                                                                  as
                                                                                                                                                  libc::c_ulong).wrapping_mul(100
                                                                                                                                                                                  as
                                                                                                                                                                                  libc::c_ulong)).wrapping_div(6
                                                                                                                                                                                                                   as
                                                                                                                                                                                                                   libc::c_int
                                                                                                                                                                                                                   as
                                                                                                                                                                                                                   libc::c_ulong))
               as uint32_t;
}
