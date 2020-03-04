#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type uint32_t = libc::c_uint;
/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
//#ifdef DEBUG_GPS_CONVERSION
// See http://en.wikipedia.org/wiki/Geographic_coordinate_conversion
#[no_mangle]
pub unsafe extern "C" fn TEST(mut GpsConversionTest: libc::c_int,
                              mut GPSCoordToDegrees_BadString: libc::c_int)
 -> libc::c_int {
    // expect
    let mut result: uint32_t =
        GPS_coord_to_degrees(b"diediedie\x00" as *const u8 as
                                 *const libc::c_char) as uint32_t;
    EXPECT_EQ(result, 0 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// expect
// expect
// overflowed without detection
// too many fractional digits
// largest value
// smallest value
