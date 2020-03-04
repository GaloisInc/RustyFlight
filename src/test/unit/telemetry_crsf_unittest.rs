#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type int16_t = libc::c_short;
pub type int32_t = libc::c_int;
pub type uint8_t = libc::c_uchar;
pub type uint16_t = libc::c_ushort;
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
#[no_mangle]
pub unsafe extern "C" fn crfsCrc(mut frame: *mut uint8_t,
                                 mut frameLen: libc::c_int) -> uint8_t {
    let mut crc: uint8_t = 0 as libc::c_int as uint8_t;
    let mut ii: libc::c_int = 2 as libc::c_int;
    while ii < frameLen - 1 as libc::c_int {
        crc =
            crc8_dvb_s2(crc as libc::c_int,
                        *frame.offset(ii as isize) as libc::c_int) as uint8_t;
        ii += 1
    }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn TEST(mut TelemetryCrsfTest: libc::c_int,
                              mut TestGPS: libc::c_int) -> libc::c_int {
    let mut frame: uint8_t = 0;
    let mut frameLen: libc::c_int = 0;
    // address
    // length
    // type
    let mut lattitude: int32_t = 0;
    EXPECT_EQ(0 as libc::c_int, lattitude);
    let mut longitude: int32_t = 0;
    EXPECT_EQ(0 as libc::c_int, longitude);
    let mut groundSpeed: uint16_t = 0;
    EXPECT_EQ(0 as libc::c_int, groundSpeed as libc::c_int);
    let mut GPSheading: uint16_t = 0;
    EXPECT_EQ(0 as libc::c_int, GPSheading as libc::c_int);
    let mut altitude: uint16_t = 0;
    EXPECT_EQ(1000 as libc::c_int, altitude as libc::c_int);
    let mut satelliteCount: uint8_t = 0;
    EXPECT_EQ(0 as libc::c_int, satelliteCount as libc::c_int);
    // altitude in cm
    // speed in 0.1m/s, 16.3 m/s = 58.68 km/h, so CRSF (km/h *10) value is 587
    // degrees * 10
    EXPECT_EQ(560000000 as libc::c_int, lattitude);
    EXPECT_EQ(1630000000 as libc::c_int, longitude);
    EXPECT_EQ(587 as libc::c_int, groundSpeed as libc::c_int);
    EXPECT_EQ(14790 as libc::c_int, GPSheading as libc::c_int);
    EXPECT_EQ(3345 as libc::c_int, altitude as libc::c_int);
    EXPECT_EQ(9 as libc::c_int, satelliteCount as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// 0.1V units
// address
// length
// type
// mV * 100
// mA * 100
// mAh
// percent
// 3.3V = 3300 mv
// = 29.60A = 29600mA - amperage is in 0.01A steps
// mV * 100
// mA * 100
// mAh
// percent
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut TelemetryCrsfTest: libc::c_int,
                                mut TestAttitude: libc::c_int)
 -> libc::c_int {
    let mut frame: uint8_t = 0;
    let mut frameLen: libc::c_int = 0;
    // address
    // length
    // type
    let mut pitch: int16_t = 0; // rad / 10000
    EXPECT_EQ(0 as libc::c_int, pitch as libc::c_int);
    let mut roll: int16_t = 0;
    EXPECT_EQ(0 as libc::c_int, roll as libc::c_int);
    let mut yaw: int16_t = 0;
    EXPECT_EQ(0 as libc::c_int, yaw as libc::c_int);
    // decidegrees == 1.183333232852155 rad
    // 2.609267231731523 rad
    //3.139847324337799 rad
    // rad / 10000
    EXPECT_EQ(11833 as libc::c_int, pitch as libc::c_int);
    EXPECT_EQ(26092 as libc::c_int, roll as libc::c_int);
    EXPECT_EQ(-(31398 as libc::c_int), yaw as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut TelemetryCrsfTest: libc::c_int,
                                mut TestFlightMode: libc::c_int)
 -> libc::c_int {
    let mut frame: uint8_t = 0;
    // nothing set, so ACRO mode
    let mut frameLen: libc::c_int = 0;
    EXPECT_EQ(5 as libc::c_int + 4 as libc::c_int, frameLen);
    // address
    // length
    // type
    EXPECT_EQ(5 as libc::c_int + 4 as libc::c_int, frameLen);
    // address
    // length
    // type
    EXPECT_EQ(4 as libc::c_int + 4 as libc::c_int, frameLen);
    // address
    // length
    // type
    EXPECT_EQ(4 as libc::c_int + 4 as libc::c_int, frameLen);
    panic!("Reached end of non-void function without returning");
    // address
    // length
    // type
}
// distance to home point in meters
// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
// see baudRate_e
// STUBS
