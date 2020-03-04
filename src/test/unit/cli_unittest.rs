#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn printf(_: *const libc::c_char, _: ...) -> libc::c_int;
}
pub type int8_t = libc::c_schar;
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
pub unsafe extern "C" fn TEST(mut CLIUnittest: libc::c_int,
                              mut TestCliSet: libc::c_int) -> libc::c_int {
    cliSet(b"array_unit_test    =   123,  -3  , 1\x00" as *const u8 as
               *const libc::c_char as *mut libc::c_char);
    let cval: libc::c_int = 0;
    printf(b"\n===============================\n\x00" as *const u8 as
               *const libc::c_char);
    let mut data: *mut int8_t = 0 as *mut int8_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        printf(b"data[%d] = %d\n\x00" as *const u8 as *const libc::c_char, i,
               *data.offset(i as isize) as libc::c_int);
        i += 1
    }
    printf(b"\n===============================\n\x00" as *const u8 as
               *const libc::c_char);
    EXPECT_EQ(123 as libc::c_int,
              *data.offset(0 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(-(3 as libc::c_int),
              *data.offset(1 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(1 as libc::c_int,
              *data.offset(2 as libc::c_int as isize) as libc::c_int);
    panic!("Reached end of non-void function without returning");
    //cliGet((char *)"osd_item_vbat");
    //EXPECT_EQ(false, false);
}
// see baudRate_e
// STUBS
