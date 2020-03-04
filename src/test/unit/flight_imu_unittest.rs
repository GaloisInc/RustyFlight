#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn sqrt(_: libc::c_double) -> libc::c_double;
}
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
pub static mut sqrt2over2: libc::c_float =
    unsafe {
        (sqrt(2 as libc::c_int as libc::c_double) / 2.0f32 as libc::c_double)
            as libc::c_float
    };
#[no_mangle]
pub unsafe extern "C" fn TEST(mut FlightImuTest: libc::c_int,
                              mut TestCalculateRotationMatrix: libc::c_int)
 -> libc::c_int {
    // No rotation
    imuComputeRotationMatrix();
    // 90 degrees around Z axis
    imuComputeRotationMatrix();
    // 60 degrees around X axis
    imuComputeRotationMatrix();
    panic!("Reached end of non-void function without returning");
}
// 45 degree yaw
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut FlightImuTest: libc::c_int,
                                mut TestSmallAngle: libc::c_int)
 -> libc::c_int {
    let r1: libc::c_float = 0.898f64 as libc::c_float;
    let r2: libc::c_float = 0.438f64 as libc::c_float;
    // given
    // and
    // when
    imuUpdateEulerAngles();
    // expect
    // given
    // when
    imuUpdateEulerAngles();
    // expect
    // given
    // when
    imuUpdateEulerAngles();
    panic!("Reached end of non-void function without returning");
    // expect
}
// STUBS
