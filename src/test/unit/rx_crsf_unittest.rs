#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type uint8_t = libc::c_uchar;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct crsfRcChannelsFrame_s {
    pub deviceAddress: uint8_t,
    pub frameLength: uint8_t,
    pub type_0: uint8_t,
    pub payload: uint8_t,
}
pub type crsfRcChannelsFrame_t = crsfRcChannelsFrame_s;
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
// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
#[no_mangle]
pub static mut crc8tab: [libc::c_uchar; 256] =
    [0 as libc::c_int as libc::c_uchar, 0xd5 as libc::c_int as libc::c_uchar,
     0x7f as libc::c_int as libc::c_uchar,
     0xaa as libc::c_int as libc::c_uchar,
     0xfe as libc::c_int as libc::c_uchar,
     0x2b as libc::c_int as libc::c_uchar,
     0x81 as libc::c_int as libc::c_uchar,
     0x54 as libc::c_int as libc::c_uchar,
     0x29 as libc::c_int as libc::c_uchar,
     0xfc as libc::c_int as libc::c_uchar,
     0x56 as libc::c_int as libc::c_uchar,
     0x83 as libc::c_int as libc::c_uchar,
     0xd7 as libc::c_int as libc::c_uchar,
     0x2 as libc::c_int as libc::c_uchar,
     0xa8 as libc::c_int as libc::c_uchar,
     0x7d as libc::c_int as libc::c_uchar,
     0x52 as libc::c_int as libc::c_uchar,
     0x87 as libc::c_int as libc::c_uchar,
     0x2d as libc::c_int as libc::c_uchar,
     0xf8 as libc::c_int as libc::c_uchar,
     0xac as libc::c_int as libc::c_uchar,
     0x79 as libc::c_int as libc::c_uchar,
     0xd3 as libc::c_int as libc::c_uchar,
     0x6 as libc::c_int as libc::c_uchar,
     0x7b as libc::c_int as libc::c_uchar,
     0xae as libc::c_int as libc::c_uchar,
     0x4 as libc::c_int as libc::c_uchar,
     0xd1 as libc::c_int as libc::c_uchar,
     0x85 as libc::c_int as libc::c_uchar,
     0x50 as libc::c_int as libc::c_uchar,
     0xfa as libc::c_int as libc::c_uchar,
     0x2f as libc::c_int as libc::c_uchar,
     0xa4 as libc::c_int as libc::c_uchar,
     0x71 as libc::c_int as libc::c_uchar,
     0xdb as libc::c_int as libc::c_uchar,
     0xe as libc::c_int as libc::c_uchar,
     0x5a as libc::c_int as libc::c_uchar,
     0x8f as libc::c_int as libc::c_uchar,
     0x25 as libc::c_int as libc::c_uchar,
     0xf0 as libc::c_int as libc::c_uchar,
     0x8d as libc::c_int as libc::c_uchar,
     0x58 as libc::c_int as libc::c_uchar,
     0xf2 as libc::c_int as libc::c_uchar,
     0x27 as libc::c_int as libc::c_uchar,
     0x73 as libc::c_int as libc::c_uchar,
     0xa6 as libc::c_int as libc::c_uchar,
     0xc as libc::c_int as libc::c_uchar,
     0xd9 as libc::c_int as libc::c_uchar,
     0xf6 as libc::c_int as libc::c_uchar,
     0x23 as libc::c_int as libc::c_uchar,
     0x89 as libc::c_int as libc::c_uchar,
     0x5c as libc::c_int as libc::c_uchar,
     0x8 as libc::c_int as libc::c_uchar,
     0xdd as libc::c_int as libc::c_uchar,
     0x77 as libc::c_int as libc::c_uchar,
     0xa2 as libc::c_int as libc::c_uchar,
     0xdf as libc::c_int as libc::c_uchar,
     0xa as libc::c_int as libc::c_uchar,
     0xa0 as libc::c_int as libc::c_uchar,
     0x75 as libc::c_int as libc::c_uchar,
     0x21 as libc::c_int as libc::c_uchar,
     0xf4 as libc::c_int as libc::c_uchar,
     0x5e as libc::c_int as libc::c_uchar,
     0x8b as libc::c_int as libc::c_uchar,
     0x9d as libc::c_int as libc::c_uchar,
     0x48 as libc::c_int as libc::c_uchar,
     0xe2 as libc::c_int as libc::c_uchar,
     0x37 as libc::c_int as libc::c_uchar,
     0x63 as libc::c_int as libc::c_uchar,
     0xb6 as libc::c_int as libc::c_uchar,
     0x1c as libc::c_int as libc::c_uchar,
     0xc9 as libc::c_int as libc::c_uchar,
     0xb4 as libc::c_int as libc::c_uchar,
     0x61 as libc::c_int as libc::c_uchar,
     0xcb as libc::c_int as libc::c_uchar,
     0x1e as libc::c_int as libc::c_uchar,
     0x4a as libc::c_int as libc::c_uchar,
     0x9f as libc::c_int as libc::c_uchar,
     0x35 as libc::c_int as libc::c_uchar,
     0xe0 as libc::c_int as libc::c_uchar,
     0xcf as libc::c_int as libc::c_uchar,
     0x1a as libc::c_int as libc::c_uchar,
     0xb0 as libc::c_int as libc::c_uchar,
     0x65 as libc::c_int as libc::c_uchar,
     0x31 as libc::c_int as libc::c_uchar,
     0xe4 as libc::c_int as libc::c_uchar,
     0x4e as libc::c_int as libc::c_uchar,
     0x9b as libc::c_int as libc::c_uchar,
     0xe6 as libc::c_int as libc::c_uchar,
     0x33 as libc::c_int as libc::c_uchar,
     0x99 as libc::c_int as libc::c_uchar,
     0x4c as libc::c_int as libc::c_uchar,
     0x18 as libc::c_int as libc::c_uchar,
     0xcd as libc::c_int as libc::c_uchar,
     0x67 as libc::c_int as libc::c_uchar,
     0xb2 as libc::c_int as libc::c_uchar,
     0x39 as libc::c_int as libc::c_uchar,
     0xec as libc::c_int as libc::c_uchar,
     0x46 as libc::c_int as libc::c_uchar,
     0x93 as libc::c_int as libc::c_uchar,
     0xc7 as libc::c_int as libc::c_uchar,
     0x12 as libc::c_int as libc::c_uchar,
     0xb8 as libc::c_int as libc::c_uchar,
     0x6d as libc::c_int as libc::c_uchar,
     0x10 as libc::c_int as libc::c_uchar,
     0xc5 as libc::c_int as libc::c_uchar,
     0x6f as libc::c_int as libc::c_uchar,
     0xba as libc::c_int as libc::c_uchar,
     0xee as libc::c_int as libc::c_uchar,
     0x3b as libc::c_int as libc::c_uchar,
     0x91 as libc::c_int as libc::c_uchar,
     0x44 as libc::c_int as libc::c_uchar,
     0x6b as libc::c_int as libc::c_uchar,
     0xbe as libc::c_int as libc::c_uchar,
     0x14 as libc::c_int as libc::c_uchar,
     0xc1 as libc::c_int as libc::c_uchar,
     0x95 as libc::c_int as libc::c_uchar,
     0x40 as libc::c_int as libc::c_uchar,
     0xea as libc::c_int as libc::c_uchar,
     0x3f as libc::c_int as libc::c_uchar,
     0x42 as libc::c_int as libc::c_uchar,
     0x97 as libc::c_int as libc::c_uchar,
     0x3d as libc::c_int as libc::c_uchar,
     0xe8 as libc::c_int as libc::c_uchar,
     0xbc as libc::c_int as libc::c_uchar,
     0x69 as libc::c_int as libc::c_uchar,
     0xc3 as libc::c_int as libc::c_uchar,
     0x16 as libc::c_int as libc::c_uchar,
     0xef as libc::c_int as libc::c_uchar,
     0x3a as libc::c_int as libc::c_uchar,
     0x90 as libc::c_int as libc::c_uchar,
     0x45 as libc::c_int as libc::c_uchar,
     0x11 as libc::c_int as libc::c_uchar,
     0xc4 as libc::c_int as libc::c_uchar,
     0x6e as libc::c_int as libc::c_uchar,
     0xbb as libc::c_int as libc::c_uchar,
     0xc6 as libc::c_int as libc::c_uchar,
     0x13 as libc::c_int as libc::c_uchar,
     0xb9 as libc::c_int as libc::c_uchar,
     0x6c as libc::c_int as libc::c_uchar,
     0x38 as libc::c_int as libc::c_uchar,
     0xed as libc::c_int as libc::c_uchar,
     0x47 as libc::c_int as libc::c_uchar,
     0x92 as libc::c_int as libc::c_uchar,
     0xbd as libc::c_int as libc::c_uchar,
     0x68 as libc::c_int as libc::c_uchar,
     0xc2 as libc::c_int as libc::c_uchar,
     0x17 as libc::c_int as libc::c_uchar,
     0x43 as libc::c_int as libc::c_uchar,
     0x96 as libc::c_int as libc::c_uchar,
     0x3c as libc::c_int as libc::c_uchar,
     0xe9 as libc::c_int as libc::c_uchar,
     0x94 as libc::c_int as libc::c_uchar,
     0x41 as libc::c_int as libc::c_uchar,
     0xeb as libc::c_int as libc::c_uchar,
     0x3e as libc::c_int as libc::c_uchar,
     0x6a as libc::c_int as libc::c_uchar,
     0xbf as libc::c_int as libc::c_uchar,
     0x15 as libc::c_int as libc::c_uchar,
     0xc0 as libc::c_int as libc::c_uchar,
     0x4b as libc::c_int as libc::c_uchar,
     0x9e as libc::c_int as libc::c_uchar,
     0x34 as libc::c_int as libc::c_uchar,
     0xe1 as libc::c_int as libc::c_uchar,
     0xb5 as libc::c_int as libc::c_uchar,
     0x60 as libc::c_int as libc::c_uchar,
     0xca as libc::c_int as libc::c_uchar,
     0x1f as libc::c_int as libc::c_uchar,
     0x62 as libc::c_int as libc::c_uchar,
     0xb7 as libc::c_int as libc::c_uchar,
     0x1d as libc::c_int as libc::c_uchar,
     0xc8 as libc::c_int as libc::c_uchar,
     0x9c as libc::c_int as libc::c_uchar,
     0x49 as libc::c_int as libc::c_uchar,
     0xe3 as libc::c_int as libc::c_uchar,
     0x36 as libc::c_int as libc::c_uchar,
     0x19 as libc::c_int as libc::c_uchar,
     0xcc as libc::c_int as libc::c_uchar,
     0x66 as libc::c_int as libc::c_uchar,
     0xb3 as libc::c_int as libc::c_uchar,
     0xe7 as libc::c_int as libc::c_uchar,
     0x32 as libc::c_int as libc::c_uchar,
     0x98 as libc::c_int as libc::c_uchar,
     0x4d as libc::c_int as libc::c_uchar,
     0x30 as libc::c_int as libc::c_uchar,
     0xe5 as libc::c_int as libc::c_uchar,
     0x4f as libc::c_int as libc::c_uchar,
     0x9a as libc::c_int as libc::c_uchar,
     0xce as libc::c_int as libc::c_uchar,
     0x1b as libc::c_int as libc::c_uchar,
     0xb1 as libc::c_int as libc::c_uchar,
     0x64 as libc::c_int as libc::c_uchar,
     0x72 as libc::c_int as libc::c_uchar,
     0xa7 as libc::c_int as libc::c_uchar,
     0xd as libc::c_int as libc::c_uchar,
     0xd8 as libc::c_int as libc::c_uchar,
     0x8c as libc::c_int as libc::c_uchar,
     0x59 as libc::c_int as libc::c_uchar,
     0xf3 as libc::c_int as libc::c_uchar,
     0x26 as libc::c_int as libc::c_uchar,
     0x5b as libc::c_int as libc::c_uchar,
     0x8e as libc::c_int as libc::c_uchar,
     0x24 as libc::c_int as libc::c_uchar,
     0xf1 as libc::c_int as libc::c_uchar,
     0xa5 as libc::c_int as libc::c_uchar,
     0x70 as libc::c_int as libc::c_uchar,
     0xda as libc::c_int as libc::c_uchar,
     0xf as libc::c_int as libc::c_uchar,
     0x20 as libc::c_int as libc::c_uchar,
     0xf5 as libc::c_int as libc::c_uchar,
     0x5f as libc::c_int as libc::c_uchar,
     0x8a as libc::c_int as libc::c_uchar,
     0xde as libc::c_int as libc::c_uchar,
     0xb as libc::c_int as libc::c_uchar,
     0xa1 as libc::c_int as libc::c_uchar,
     0x74 as libc::c_int as libc::c_uchar,
     0x9 as libc::c_int as libc::c_uchar,
     0xdc as libc::c_int as libc::c_uchar,
     0x76 as libc::c_int as libc::c_uchar,
     0xa3 as libc::c_int as libc::c_uchar,
     0xf7 as libc::c_int as libc::c_uchar,
     0x22 as libc::c_int as libc::c_uchar,
     0x88 as libc::c_int as libc::c_uchar,
     0x5d as libc::c_int as libc::c_uchar,
     0xd6 as libc::c_int as libc::c_uchar,
     0x3 as libc::c_int as libc::c_uchar,
     0xa9 as libc::c_int as libc::c_uchar,
     0x7c as libc::c_int as libc::c_uchar,
     0x28 as libc::c_int as libc::c_uchar,
     0xfd as libc::c_int as libc::c_uchar,
     0x57 as libc::c_int as libc::c_uchar,
     0x82 as libc::c_int as libc::c_uchar,
     0xff as libc::c_int as libc::c_uchar,
     0x2a as libc::c_int as libc::c_uchar,
     0x80 as libc::c_int as libc::c_uchar,
     0x55 as libc::c_int as libc::c_uchar,
     0x1 as libc::c_int as libc::c_uchar,
     0xd4 as libc::c_int as libc::c_uchar,
     0x7e as libc::c_int as libc::c_uchar,
     0xab as libc::c_int as libc::c_uchar,
     0x84 as libc::c_int as libc::c_uchar,
     0x51 as libc::c_int as libc::c_uchar,
     0xfb as libc::c_int as libc::c_uchar,
     0x2e as libc::c_int as libc::c_uchar,
     0x7a as libc::c_int as libc::c_uchar,
     0xaf as libc::c_int as libc::c_uchar,
     0x5 as libc::c_int as libc::c_uchar,
     0xd0 as libc::c_int as libc::c_uchar,
     0xad as libc::c_int as libc::c_uchar,
     0x78 as libc::c_int as libc::c_uchar,
     0xd2 as libc::c_int as libc::c_uchar,
     0x7 as libc::c_int as libc::c_uchar,
     0x53 as libc::c_int as libc::c_uchar,
     0x86 as libc::c_int as libc::c_uchar,
     0x2c as libc::c_int as libc::c_uchar,
     0xf9 as libc::c_int as libc::c_uchar];
#[no_mangle]
pub unsafe extern "C" fn crc8_buf(mut ptr: *const uint8_t, mut len: uint8_t)
 -> uint8_t {
    let mut crc: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < len as libc::c_int {
        let fresh0 = ptr;
        ptr = ptr.offset(1);
        crc = crc8tab[(crc as libc::c_int ^ *fresh0 as libc::c_int) as usize];
        i = i.wrapping_add(1)
    }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn crc8_dvb_s2_buf(mut ptr: *const uint8_t,
                                         mut len: uint8_t) -> uint8_t {
    let mut crc: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < len as libc::c_int {
        let fresh1 = ptr;
        ptr = ptr.offset(1);
        crc =
            crc8_dvb_s2(crc as libc::c_int, *fresh1 as libc::c_int) as
                uint8_t;
        i = i.wrapping_add(1)
    }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn TEST(mut CrossFireTest: libc::c_int,
                              mut CRC: libc::c_int) -> libc::c_int {
    static mut buf1: [uint8_t; 27] =
        [97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110,
         111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 0];
    let mut crc1: uint8_t = 0 as libc::c_int as uint8_t;
    let mut crc2: uint8_t = 0 as libc::c_int as uint8_t;
    crc1 = crc8tab[1 as libc::c_int as usize];
    crc2 = crc8_dvb_s2(0 as libc::c_int, 1 as libc::c_int) as uint8_t;
    EXPECT_EQ(crc1 as libc::c_int, crc2 as libc::c_int);
    crc1 = crc8tab[2 as libc::c_int as usize];
    crc2 = crc8_dvb_s2(0 as libc::c_int, 2 as libc::c_int) as uint8_t;
    EXPECT_EQ(crc1 as libc::c_int, crc2 as libc::c_int);
    crc1 = crc8_buf(buf1.as_ptr(), 26 as libc::c_int as uint8_t);
    crc2 = crc8_dvb_s2_buf(buf1.as_ptr(), 26 as libc::c_int as uint8_t);
    EXPECT_EQ(crc1 as libc::c_int, crc2 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
/*
 * Frame is of form
 * <Device address> <Frame length> < Type>  <Payload> < CRC>
 * So RC channels frame is:
 * <0x00> <0x18> <0x16>  <22-bytes payload> < CRC>
 * 26 bytes altogther.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut CrossFireTest: libc::c_int,
                                mut TestCrsfFrameStatusUnpacking: libc::c_int)
 -> libc::c_int {
    // 16 11-bit channels packed into 22 bytes of data
    // bits 0-7
    // bits 8-15
    // bits 16-23
    // bits 24-31
    // bits 32-39  0101100.
    // bits 40-47  ....0001
    // bits 48-55  0.......
    // bits 56-64  11110000
    // bits 65-71  ......01
    // bits 72-79  011.....
    // bits 80-87  11100010
    let crc: uint8_t = crsfFrameCRC() as uint8_t;
    let status: uint8_t = crsfFrameStatus() as uint8_t;
    panic!("Reached end of non-void function without returning");
    //  172 = 0x0ac, 0001 0101100, bits 33-43
    //  992 = 0x3e0, 01 1110000 0, bits 55-65
    // 1811 = 0x713, 1110 0010 011, bits 77-87
}
#[no_mangle]
pub static mut capturedData: [uint8_t; 52] =
    [0 as libc::c_int as uint8_t, 0x18 as libc::c_int as uint8_t,
     0x16 as libc::c_int as uint8_t, 0xbd as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0x9f as libc::c_int as uint8_t,
     0xf4 as libc::c_int as uint8_t, 0xae as libc::c_int as uint8_t,
     0xf7 as libc::c_int as uint8_t, 0xbd as libc::c_int as uint8_t,
     0xef as libc::c_int as uint8_t, 0x7d as libc::c_int as uint8_t,
     0xef as libc::c_int as uint8_t, 0xfb as libc::c_int as uint8_t,
     0xad as libc::c_int as uint8_t, 0xfd as libc::c_int as uint8_t,
     0x45 as libc::c_int as uint8_t, 0x2b as libc::c_int as uint8_t,
     0x5a as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x6c as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x18 as libc::c_int as uint8_t,
     0x16 as libc::c_int as uint8_t, 0xbd as libc::c_int as uint8_t,
     0x8 as libc::c_int as uint8_t, 0x9f as libc::c_int as uint8_t,
     0xf4 as libc::c_int as uint8_t, 0xaa as libc::c_int as uint8_t,
     0xf7 as libc::c_int as uint8_t, 0xbd as libc::c_int as uint8_t,
     0xef as libc::c_int as uint8_t, 0x7d as libc::c_int as uint8_t,
     0xef as libc::c_int as uint8_t, 0xfb as libc::c_int as uint8_t,
     0xad as libc::c_int as uint8_t, 0xfd as libc::c_int as uint8_t,
     0x45 as libc::c_int as uint8_t, 0x2b as libc::c_int as uint8_t,
     0x5a as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x94 as libc::c_int as uint8_t];
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut CrossFireTest: libc::c_int,
                                mut TestCapturedData: libc::c_int)
 -> libc::c_int {
    //const int frameCount = sizeof(capturedData) / sizeof(crsfRcChannelsFrame_t);
    let mut framePtr: *const crsfRcChannelsFrame_t =
        capturedData.as_ptr() as *const crsfRcChannelsFrame_t;
    let mut status: uint8_t = crsfFrameStatus() as uint8_t;
    let mut crc: uint8_t = crsfFrameCRC() as uint8_t;
    framePtr = framePtr.offset(1);
    status = crsfFrameStatus() as uint8_t;
    crc = crsfFrameCRC() as uint8_t;
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut CrossFireTest: libc::c_int,
                                mut TestCrsfDataReceive: libc::c_int)
 -> libc::c_int {
    let mut pData: *const uint8_t = capturedData.as_ptr();
    let mut ii: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (ii as libc::c_ulong) <
              ::std::mem::size_of::<crsfRcChannelsFrame_t>() as libc::c_ulong
          {
        let fresh2 = pData;
        pData = pData.offset(1);
        crsfDataReceive(*fresh2 as libc::c_int);
        ii = ii.wrapping_add(1)
    }
    let mut crc: uint8_t = crsfFrameCRC() as uint8_t;
    panic!("Reached end of non-void function without returning");
}
// STUBS
