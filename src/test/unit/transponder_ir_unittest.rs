#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
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
pub unsafe extern "C" fn TEST(mut transponderTest: libc::c_int,
                              mut updateTransponderDMABufferArcitimer:
                                  libc::c_int) -> libc::c_int {
    //input
    let mut data: [uint8_t; 9] =
        [0x1f as libc::c_int as uint8_t, 0xfc as libc::c_int as uint8_t,
         0x8f as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
         0xf0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
         0xf8 as libc::c_int as uint8_t, 0x1f as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t];
    //excepted
    let mut excepted: uint8_t = 0;
    let mut transponderData: *mut uint8_t = data.as_mut_ptr();
    let mut i: uint16_t = 0;
    panic!("Reached end of non-void function without returning");
}
