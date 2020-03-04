#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type uint8_t = libc::c_uchar;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct crsfMspFrame_s {
    pub deviceAddress: uint8_t,
    pub frameLength: uint8_t,
    pub type_0: uint8_t,
    pub destination: uint8_t,
    pub origin: uint8_t,
    pub payload: uint8_t,
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
//max frame size
pub type crsfMspFrame_t = crsfMspFrame_s;
#[no_mangle]
pub static mut crsfPidRequest: [uint8_t; 14] =
    [0 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
     0x7a as libc::c_int as uint8_t, 0xc8 as libc::c_int as uint8_t,
     0xea as libc::c_int as uint8_t, 0x30 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x70 as libc::c_int as uint8_t,
     0x70 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x69 as libc::c_int as uint8_t];
#[no_mangle]
pub unsafe extern "C" fn TEST(mut CrossFireMSPTest: libc::c_int,
                              mut RequestBufferTest: libc::c_int)
 -> libc::c_int {
    initSharedMsp(); // not done yet*/
    let mut framePtr: *const crsfMspFrame_t =
        crsfPidRequest.as_ptr() as *const crsfMspFrame_t; // not done yet
    let mut destination: *mut uint8_t = 0 as *mut uint8_t; // not done yet
    let mut origin: *mut uint8_t = 0 as *mut uint8_t; // not done yet
    let mut frameStart: *mut uint8_t = 0 as *mut uint8_t;
    let mut frameEnd: *mut uint8_t = 0 as *mut uint8_t;
    EXPECT_EQ(0xc8 as libc::c_int, *destination as libc::c_int);
    EXPECT_EQ(0xea as libc::c_int, *origin as libc::c_int);
    EXPECT_EQ(0x30 as libc::c_int, *frameStart as libc::c_int);
    EXPECT_EQ(0x69 as libc::c_int, *frameEnd as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub static mut crsfPidWrite1: [uint8_t; 13] =
    [0 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
     0x7c as libc::c_int as uint8_t, 0xc8 as libc::c_int as uint8_t,
     0xea as libc::c_int as uint8_t, 0x31 as libc::c_int as uint8_t,
     0x1e as libc::c_int as uint8_t, 0xca as libc::c_int as uint8_t,
     0x29 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
     0x1e as libc::c_int as uint8_t, 0x3a as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut crsfPidWrite2: [uint8_t; 13] =
    [0 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
     0x7c as libc::c_int as uint8_t, 0xc8 as libc::c_int as uint8_t,
     0xea as libc::c_int as uint8_t, 0x22 as libc::c_int as uint8_t,
     0x23 as libc::c_int as uint8_t, 0x46 as libc::c_int as uint8_t,
     0x2d as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut crsfPidWrite3: [uint8_t; 13] =
    [0 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
     0x7c as libc::c_int as uint8_t, 0xc8 as libc::c_int as uint8_t,
     0xea as libc::c_int as uint8_t, 0x23 as libc::c_int as uint8_t,
     0xf as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x22 as libc::c_int as uint8_t,
     0xe as libc::c_int as uint8_t, 0x35 as libc::c_int as uint8_t,
     0x19 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut crsfPidWrite4: [uint8_t; 13] =
    [0 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
     0x7c as libc::c_int as uint8_t, 0xc8 as libc::c_int as uint8_t,
     0xea as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
     0x21 as libc::c_int as uint8_t, 0x53 as libc::c_int as uint8_t,
     0x32 as libc::c_int as uint8_t, 0x32 as libc::c_int as uint8_t,
     0x4b as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut crsfPidWrite5: [uint8_t; 13] =
    [0 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
     0x7c as libc::c_int as uint8_t, 0xc8 as libc::c_int as uint8_t,
     0xea as libc::c_int as uint8_t, 0x25 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x37 as libc::c_int as uint8_t,
     0x37 as libc::c_int as uint8_t, 0x4b as libc::c_int as uint8_t,
     0xf8 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t];
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut CrossFireMSPTest: libc::c_int,
                                mut WriteResponseTest: libc::c_int)
 -> libc::c_int {
    initSharedMsp();
    let mut framePtr1: *const crsfMspFrame_t =
        crsfPidWrite1.as_ptr() as *const crsfMspFrame_t;
    let mut frameStart: *mut uint8_t = 0 as *mut uint8_t;
    let mut pending1: bool = false;
    EXPECT_FALSE(pending1 as libc::c_int);
    let mut framePtr2: *const crsfMspFrame_t =
        crsfPidWrite2.as_ptr() as *const crsfMspFrame_t;
    let mut frameStart2: *mut uint8_t = 0 as *mut uint8_t;
    let mut pending2: bool = false;
    EXPECT_FALSE(pending2 as libc::c_int);
    let mut framePtr3: *const crsfMspFrame_t =
        crsfPidWrite3.as_ptr() as *const crsfMspFrame_t;
    let mut frameStart3: *mut uint8_t = 0 as *mut uint8_t;
    let mut pending3: bool = false;
    EXPECT_FALSE(pending3 as libc::c_int);
    let mut framePtr4: *const crsfMspFrame_t =
        crsfPidWrite4.as_ptr() as *const crsfMspFrame_t;
    let mut frameStart4: *mut uint8_t = 0 as *mut uint8_t;
    let mut pending4: bool = false;
    EXPECT_FALSE(pending4 as libc::c_int);
    //EXPECT_EQ(0xB3,checksum);
    let mut framePtr5: *const crsfMspFrame_t =
        crsfPidWrite5.as_ptr() as *const crsfMspFrame_t;
    let mut frameStart5: *mut uint8_t = 0 as *mut uint8_t;
    let mut pending5: bool = false;
    EXPECT_TRUE(pending5 as libc::c_int);
    panic!("Reached end of non-void function without returning");
    // not done yet
}
#[no_mangle]
pub unsafe extern "C" fn testSendMspResponse(mut payload: *mut uint8_t) { }
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut CrossFireMSPTest: libc::c_int,
                                mut SendMspReply: libc::c_int)
 -> libc::c_int {
    initSharedMsp();
    let mut framePtr: *const crsfMspFrame_t =
        crsfPidRequest.as_ptr() as *const crsfMspFrame_t;
    let mut frameStart: *mut uint8_t = 0 as *mut uint8_t;
    let mut handled: bool = false;
    EXPECT_TRUE(handled as libc::c_int);
    let mut replyPending: bool =
        sendMspReply(64 as libc::c_int,
                     Some(testSendMspResponse as
                              unsafe extern "C" fn(_: *mut uint8_t) -> ())) !=
            0;
    EXPECT_FALSE(replyPending as libc::c_int);
    let mut ii: libc::c_uint = 1 as libc::c_int as libc::c_uint;
    while ii <= 30 as libc::c_int as libc::c_uint { ii = ii.wrapping_add(1) }
    panic!("Reached end of non-void function without returning");
    // CRC
}
// STUBS
