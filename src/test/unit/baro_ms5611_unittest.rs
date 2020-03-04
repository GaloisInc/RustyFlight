#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type int8_t = libc::c_schar;
pub type int32_t = libc::c_int;
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
pub unsafe extern "C" fn TEST(mut baroMS5611Test: libc::c_int,
                              mut TestValidMs5611Crc: libc::c_int)
 -> libc::c_int {
    // given
    let mut ms5611_prom: [uint16_t; 8] =
        [0x3132 as libc::c_int as uint16_t, 0x3334 as libc::c_int as uint16_t,
         0x3536 as libc::c_int as uint16_t, 0x3738 as libc::c_int as uint16_t,
         0x3940 as libc::c_int as uint16_t, 0x4142 as libc::c_int as uint16_t,
         0x4344 as libc::c_int as uint16_t,
         0x450b as libc::c_int as uint16_t];
    // when
    let mut result: int8_t = ms5611_crc(ms5611_prom.as_mut_ptr()) as int8_t;
    // then
    EXPECT_EQ(0 as libc::c_int, result as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// given
// when
// then
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut baroMS5611Test: libc::c_int,
                                mut TestMs5611AllZeroProm: libc::c_int)
 -> libc::c_int {
    // given
    let mut ms5611_prom: [uint16_t; 8] =
        [0 as libc::c_int as uint16_t, 0 as libc::c_int as uint16_t,
         0 as libc::c_int as uint16_t, 0 as libc::c_int as uint16_t,
         0 as libc::c_int as uint16_t, 0 as libc::c_int as uint16_t,
         0 as libc::c_int as uint16_t, 0 as libc::c_int as uint16_t];
    // when
    let mut result: int8_t = ms5611_crc(ms5611_prom.as_mut_ptr()) as int8_t;
    // then
    EXPECT_EQ(-(1 as libc::c_int), result as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut baroMS5611Test: libc::c_int,
                                mut TestMs5611AllOnesProm: libc::c_int)
 -> libc::c_int {
    // given
    let mut ms5611_prom: [uint16_t; 8] =
        [0xffff as libc::c_int as uint16_t, 0xffff as libc::c_int as uint16_t,
         0xffff as libc::c_int as uint16_t, 0xffff as libc::c_int as uint16_t,
         0xffff as libc::c_int as uint16_t, 0xffff as libc::c_int as uint16_t,
         0xffff as libc::c_int as uint16_t,
         0xffff as libc::c_int as uint16_t];
    // when
    let mut result: int8_t = ms5611_crc(ms5611_prom.as_mut_ptr()) as int8_t;
    // then
    EXPECT_EQ(-(1 as libc::c_int), result as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut baroMS5611Test: libc::c_int,
                                mut TestMs5611CalculatePressureGT20Deg:
                                    libc::c_int) -> libc::c_int {
    // given
    let mut pressure: int32_t = 0; // calibration data from MS5611 datasheet
    let mut temperature: int32_t = 0;
    let mut ms5611_c_test: [uint16_t; 8] =
        [0 as libc::c_int as uint16_t, 40127 as libc::c_int as uint16_t,
         36924 as libc::c_int as uint16_t, 23317 as libc::c_int as uint16_t,
         23282 as libc::c_int as uint16_t, 33464 as libc::c_int as uint16_t,
         28312 as libc::c_int as uint16_t, 0 as libc::c_int as uint16_t];
    // Digital pressure value from MS5611 datasheet
    // Digital temperature value from MS5611 datasheet
    // when
    ms5611_calculate(&mut pressure, &mut temperature);
    // then
    EXPECT_EQ(2007 as libc::c_int, temperature); // 20.07 deg C
    EXPECT_EQ(100009 as libc::c_int, pressure);
    panic!("Reached end of non-void function without returning");
    // 1000.09 mbar
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut baroMS5611Test: libc::c_int,
                                mut TestMs5611CalculatePressureLT20Deg:
                                    libc::c_int) -> libc::c_int {
    // given
    let mut pressure: int32_t = 0; // calibration data from MS5611 datasheet
    let mut temperature: int32_t = 0;
    let mut ms5611_c_test: [uint16_t; 8] =
        [0 as libc::c_int as uint16_t, 40127 as libc::c_int as uint16_t,
         36924 as libc::c_int as uint16_t, 23317 as libc::c_int as uint16_t,
         23282 as libc::c_int as uint16_t, 33464 as libc::c_int as uint16_t,
         28312 as libc::c_int as uint16_t, 0 as libc::c_int as uint16_t];
    // Digital pressure value from MS5611 datasheet
    // Digital temperature value
    // when
    ms5611_calculate(&mut pressure, &mut temperature);
    // then
    EXPECT_EQ(205 as libc::c_int, temperature); // 2.05 deg C
    EXPECT_EQ(96512 as libc::c_int, pressure);
    panic!("Reached end of non-void function without returning");
    // 965.12 mbar
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut baroMS5611Test: libc::c_int,
                                mut TestMs5611CalculatePressureLTMinus15Deg:
                                    libc::c_int) -> libc::c_int {
    // given
    let mut pressure: int32_t = 0; // calibration data from MS5611 datasheet
    let mut temperature: int32_t = 0;
    let mut ms5611_c_test: [uint16_t; 8] =
        [0 as libc::c_int as uint16_t, 40127 as libc::c_int as uint16_t,
         36924 as libc::c_int as uint16_t, 23317 as libc::c_int as uint16_t,
         23282 as libc::c_int as uint16_t, 33464 as libc::c_int as uint16_t,
         28312 as libc::c_int as uint16_t, 0 as libc::c_int as uint16_t];
    // Digital pressure value from MS5611 datasheet
    // Digital temperature value
    // when
    ms5611_calculate(&mut pressure, &mut temperature);
    // then
    EXPECT_EQ(-(2710 as libc::c_int), temperature); // -27.10 deg C
    EXPECT_EQ(90613 as libc::c_int, pressure);
    panic!("Reached end of non-void function without returning");
    // 906.13 mbar
}
// STUBS
