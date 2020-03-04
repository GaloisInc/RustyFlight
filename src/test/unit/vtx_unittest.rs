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
#[no_mangle]
pub static mut simulationFeatureFlags: uint32_t =
    0 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut simulationTime: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut gyroCalibDone: libc::c_int = 0;
#[no_mangle]
pub static mut simulationHaveRx: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn TEST(mut VtxTest: libc::c_int,
                              mut PitMode: libc::c_int) -> libc::c_int {
    // given
    // and
    // expect
    // and
    // enable vtx pit mode
    // when
    updateActivatedModes();
    panic!("Reached end of non-void function without returning");
    // expect
}
// STUBS
