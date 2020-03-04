#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type int32_t = libc::c_int;
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
// See http://en.wikipedia.org/wiki/Geographic_coordinate_conversion
#[no_mangle]
pub static mut hottGPSMessage: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn getGPSMessageForTest() -> *mut libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn TEST(mut TelemetryHottTest: libc::c_int,
                              mut UpdateGPSCoordinates1: libc::c_int)
 -> libc::c_int {
    // given
    // Mayrhofen, Austria
    let mut longitude: uint32_t =
        GPS_coord_to_degrees(b"4710.5186\x00" as *const u8 as
                                 *const libc::c_char) as uint32_t;
    let mut latitude: uint32_t =
        GPS_coord_to_degrees(b"1151.4252\x00" as *const u8 as
                                 *const libc::c_char) as uint32_t;
    panic!("Reached end of non-void function without returning");
    // when
    // then
}
// given
// Hampstead Heath, London
    // 51.563886, -0.159960
// when
// then
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut TelemetryHottTest: libc::c_int,
                                mut UpdateGPSCoordinates3: libc::c_int)
 -> libc::c_int {
    // given
    let mut longitude: int32_t =
        -GPS_coord_to_degrees(b"17999.9999\x00" as *const u8 as
                                  *const libc::c_char);
    let mut latitude: int32_t =
        GPS_coord_to_degrees(b"8999.9999\x00" as *const u8 as
                                 *const libc::c_char);
    panic!("Reached end of non-void function without returning");
    // when
    // then
}
// direction to home or hol point in degrees
// distance to home point in meters
// STUBS
/*
TEST(TelemetryHottTest, PrepareGPSMessage_Altitude1m)
{
    // given
    HOTT_GPS_MSG_t *hottGPSMessage = getGPSMessageForTest();

    stateFlags = GPS_FIX;
    uint16_t altitudeInMeters = 1;
    GPS_altitude = altitudeInMeters * (1 / 0.1f); // 1 = 0.1m

    // when
    hottPrepareGPSResponse(hottGPSMessage);

    // then
    EXPECT_EQ((int16_t)(hottGPSMessage->altitude_H << 8 | hottGPSMessage->altitude_L), 1 + HOTT_GPS_ALTITUDE_OFFSET);
}
*/
