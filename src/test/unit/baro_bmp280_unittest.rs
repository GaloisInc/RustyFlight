#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type int32_t = libc::c_int;
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
/* calibration T1 data */
/* calibration T2 data */
/* calibration T3 data */
/* calibration P1 data */
/* calibration P2 data */
/* calibration P3 data */
/* calibration P4 data */
/* calibration P5 data */
/* calibration P6 data */
/* calibration P7 data */
/* calibration P8 data */
/* calibration P9 data */
/* calibration t_fine data */
#[no_mangle]
pub unsafe extern "C" fn TEST(mut baroBmp280Test: libc::c_int,
                              mut TestBmp280Calculate: libc::c_int)
 -> libc::c_int {
    // given
    let mut pressure: int32_t = 0;
    let mut temperature: int32_t = 0;
    // Digital pressure value
    // Digital temperature value
    // and
    // when
    bmp280_calculate(&mut pressure, &mut temperature);
    // then
    EXPECT_EQ(100653 as libc::c_int, pressure); // 100653 Pa
    EXPECT_EQ(2508 as libc::c_int, temperature);
    panic!("Reached end of non-void function without returning");
    // 25.08 degC
}
// given
// Digital pressure value
// Digital temperature value
// and
// when
// then
// 135385 Pa
// 25.08 degC
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut baroBmp280Test: libc::c_int,
                                mut TestBmp280CalculateZeroP: libc::c_int)
 -> libc::c_int {
    // given
    let mut pressure: int32_t = 0;
    let mut temperature: int32_t = 0;
    // Digital pressure value
    // Digital temperature value
    // and
    // when
    bmp280_calculate(&mut pressure, &mut temperature);
    // then
    EXPECT_EQ(0 as libc::c_int,
              pressure); // P1=0 trips pressure to 0 Pa, avoiding division by zero
    EXPECT_EQ(2508 as libc::c_int, temperature);
    panic!("Reached end of non-void function without returning");
    // 25.08 degC
}
// STUBS
