#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type uint8_t = libc::c_uchar;
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
//#define DEBUG_LEDSTRIP
#[no_mangle]
pub unsafe extern "C" fn TEST(mut LedStripTest: libc::c_int,
                              mut parseLedStripConfig: libc::c_int)
 -> libc::c_int {
    // given
    // and
    static mut expectedLedStripConfig: libc::c_int = 0;
    // and
    let mut ledStripConfigCommands: [*const libc::c_char; 30] =
        [b"9,9:S:FW:0\x00" as *const u8 as *const libc::c_char,
         b"10,10:S:FW:0\x00" as *const u8 as *const libc::c_char,
         b"11,11:S:IA:0\x00" as *const u8 as *const libc::c_char,
         b"11,11:E:IA:0\x00" as *const u8 as *const libc::c_char,
         b"10,10:E:F:0\x00" as *const u8 as *const libc::c_char,
         b"10,5:S:F:0\x00" as *const u8 as *const libc::c_char,
         b"11,4:S:F:0\x00" as *const u8 as *const libc::c_char,
         b"12,3:S:IA:0\x00" as *const u8 as *const libc::c_char,
         b"12,2:N:IA:0\x00" as *const u8 as *const libc::c_char,
         b"11,1:N:F:0\x00" as *const u8 as *const libc::c_char,
         b"10,0:N:F:0\x00" as *const u8 as *const libc::c_char,
         b"7,0:N:FW:0\x00" as *const u8 as *const libc::c_char,
         b"6,0:N:CW:1\x00" as *const u8 as *const libc::c_char,
         b"5,0:N:CW:1\x00" as *const u8 as *const libc::c_char,
         b"4,0:N:FW:0\x00" as *const u8 as *const libc::c_char,
         b"2,0:N:F:0\x00" as *const u8 as *const libc::c_char,
         b"1,1:N:F:0\x00" as *const u8 as *const libc::c_char,
         b"0,2:N:IA:0\x00" as *const u8 as *const libc::c_char,
         b"0,3:W:IA:0\x00" as *const u8 as *const libc::c_char,
         b"1,4:W:F:0\x00" as *const u8 as *const libc::c_char,
         b"2,5:W:F:0\x00" as *const u8 as *const libc::c_char,
         b"1,10:W:F:0\x00" as *const u8 as *const libc::c_char,
         b"0,11:W:IA:0\x00" as *const u8 as *const libc::c_char,
         b"0,11:S:IA:0\x00" as *const u8 as *const libc::c_char,
         b"1,10:S:FW:0\x00" as *const u8 as *const libc::c_char,
         b"2,9:S:FW:0\x00" as *const u8 as *const libc::c_char,
         b"7,7::R:14\x00" as *const u8 as *const libc::c_char,
         b"8,7::R:15\x00" as *const u8 as *const libc::c_char,
         b"8,8::R:14\x00" as *const u8 as *const libc::c_char,
         b"7,8::R:15\x00" as *const u8 as *const libc::c_char];
    // when
    let mut index: uint8_t = 0 as libc::c_int as uint8_t;
    while (index as libc::c_ulong) <
              (::std::mem::size_of::<[*const libc::c_char; 30]>() as
                   libc::c_ulong).wrapping_div(::std::mem::size_of::<*const libc::c_char>()
                                                   as libc::c_ulong) {
        index = index.wrapping_add(1)
    }
    panic!("Reached end of non-void function without returning");
    // then
    // and
    // then
    // then
}
// when
// then
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut LedStripTest: libc::c_int,
                                mut smallestGrid: libc::c_int)
 -> libc::c_int {
    // given
    // and
    static mut testLedConfigs: [libc::c_int; 0] = [];
    // when
    reevaluateLedConfig();
    panic!("Reached end of non-void function without returning");
    // then
}
#[no_mangle]
pub static mut testColors: libc::c_int = 0;
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut ColorTest: libc::c_int,
                                mut parseColor: libc::c_int) -> libc::c_int {
    // given
    // and
    let expectedColors: [libc::c_int; 4] = [0; 4];
    let mut testColors_0: [*const libc::c_char; 4] =
        [b"0,0,0\x00" as *const u8 as *const libc::c_char,
         b"1,1,1\x00" as *const u8 as *const libc::c_char,
         b"359,255,255\x00" as *const u8 as *const libc::c_char,
         b"333,22,1\x00" as *const u8 as *const libc::c_char];
    //  H    S    V
    // when
    let mut index: uint8_t = 0 as libc::c_int as uint8_t;
    while (index as libc::c_int) < 4 as libc::c_int {
        index = index.wrapping_add(1)
    }
    // then
    let mut index_0: uint8_t = 0 as libc::c_int as uint8_t;
    while (index_0 as libc::c_int) < 4 as libc::c_int {
        index_0 = index_0.wrapping_add(1)
    }
    panic!("Reached end of non-void function without returning");
}
