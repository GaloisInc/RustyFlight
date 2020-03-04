#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
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
// Points to top entry of the current page
#[no_mangle]
pub unsafe extern "C" fn TEST(mut CMSUnittest: libc::c_int,
                              mut TestCmsDisplayPortRegister: libc::c_int)
 -> libc::c_int {
    cmsInit();
    let registered: bool = false;
    EXPECT_EQ(0 as libc::c_int, registered as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut CMSUnittest: libc::c_int,
                                mut TestCmsMenuExit0: libc::c_int)
 -> libc::c_int {
    cmsInit();
    cmsMenuOpen();
    let mut exit: libc::c_long = 0;
    EXPECT_EQ(0 as libc::c_int, exit);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut CMSUnittest: libc::c_int,
                                mut TestCmsMenuExit1: libc::c_int)
 -> libc::c_int {
    cmsInit();
    cmsMenuOpen();
    let mut exit: libc::c_long = 0;
    EXPECT_EQ(0 as libc::c_int, exit);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut CMSUnittest: libc::c_int,
                                mut TestCmsMenuBack: libc::c_int)
 -> libc::c_int {
    cmsInit();
    cmsMenuOpen();
    let mut exit: libc::c_long = 0;
    EXPECT_EQ(0 as libc::c_int, exit);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut CMSUnittest: libc::c_int,
                                mut TestCmsMenuKey: libc::c_int)
 -> libc::c_int {
    // msec
    // msec
    cmsInit();
    cmsMenuOpen();
    let mut result: uint16_t = 0;
    EXPECT_EQ(500 as libc::c_int, result as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// STUBS
