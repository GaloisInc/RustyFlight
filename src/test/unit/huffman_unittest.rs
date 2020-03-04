#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type int16_t = libc::c_short;
pub type uint8_t = libc::c_uchar;
pub type uint16_t = libc::c_ushort;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct huffmanTree_s {
    pub value: int16_t,
    pub codeLen: uint16_t,
    pub code: uint16_t,
}
// 256 characters plus EOF
pub type huffmanTree_t = huffmanTree_s;
static mut outBuf: [uint8_t; 128] = [0; 128];
static mut huffmanTree: [huffmanTree_t; 257] =
    [{
         let mut init =
             huffmanTree_s{value: 0 as libc::c_int as int16_t,
                           codeLen: 2 as libc::c_int as uint16_t,
                           code: 0x3 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x1 as libc::c_int as int16_t,
                           codeLen: 3 as libc::c_int as uint16_t,
                           code: 0x5 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x2 as libc::c_int as int16_t,
                           codeLen: 4 as libc::c_int as uint16_t,
                           code: 0x9 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x3 as libc::c_int as int16_t,
                           codeLen: 5 as libc::c_int as uint16_t,
                           code: 0x11 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x4 as libc::c_int as int16_t,
                           codeLen: 5 as libc::c_int as uint16_t,
                           code: 0x10 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x50 as libc::c_int as int16_t,
                           codeLen: 5 as libc::c_int as uint16_t,
                           code: 0xf as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x5 as libc::c_int as int16_t,
                           codeLen: 6 as libc::c_int as uint16_t,
                           code: 0x1d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x6 as libc::c_int as int16_t,
                           codeLen: 6 as libc::c_int as uint16_t,
                           code: 0x1c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x7 as libc::c_int as int16_t,
                           codeLen: 6 as libc::c_int as uint16_t,
                           code: 0x1b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x8 as libc::c_int as int16_t,
                           codeLen: 6 as libc::c_int as uint16_t,
                           code: 0x1a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x10 as libc::c_int as int16_t,
                           codeLen: 6 as libc::c_int as uint16_t,
                           code: 0x19 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x9 as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x31 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x30 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x2f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x2e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x2d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x2c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x2b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x11 as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x2a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x12 as libc::c_int as int16_t,
                           codeLen: 7 as libc::c_int as uint16_t,
                           code: 0x29 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x13 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x51 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x14 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x50 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x15 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x4f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x16 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x4e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x17 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x4d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x18 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x4c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x19 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x4b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x1a as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x4a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x1b as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x49 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x1c as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x48 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x1d as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x47 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x1e as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x46 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x1f as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x45 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x20 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x44 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x21 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x43 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x22 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x42 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x23 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x41 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x24 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x30 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x3f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x40 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x3e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf0 as libc::c_int as int16_t,
                           codeLen: 8 as libc::c_int as uint16_t,
                           code: 0x3d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x25 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x79 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x26 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x78 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x27 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x77 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x28 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x76 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x29 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x75 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x2a as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x74 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x2b as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x73 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x2c as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x72 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x2d as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x71 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x2e as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x70 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x2f as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x6f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x31 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x6e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x32 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x6d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x33 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x6c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x34 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x6b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x35 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x6a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x36 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x69 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x37 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x68 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x38 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x67 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x39 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x66 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x3a as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x65 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x3b as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x64 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x3c as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x63 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x3d as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x62 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x3e as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x61 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x3f as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x60 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x41 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x5f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x42 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x5e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x43 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x5d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x44 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x5c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x45 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x5b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x46 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x5a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x47 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x59 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x48 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x58 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x49 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x57 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x4c as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x56 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x4f as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x55 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x51 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x54 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x80 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x53 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe0 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x52 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf1 as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x51 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xff as libc::c_int as int16_t,
                           codeLen: 9 as libc::c_int as uint16_t,
                           code: 0x50 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x4a as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x9f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x4b as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x9e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x4d as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x9d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x4e as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x9c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x52 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x9b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x53 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x9a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x54 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x99 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x55 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x98 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x56 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x97 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x57 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x96 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x58 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x95 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x59 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x94 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x5a as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x93 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x5b as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x92 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x5c as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x91 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x5d as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x90 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x5e as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x8f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x5f as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x8e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x60 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x8d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x61 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x8c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x62 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x8b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x63 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x8a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x64 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x89 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x65 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x88 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x66 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x87 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x67 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x86 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x68 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x85 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x69 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x84 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x6a as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x83 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x6b as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x82 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x6c as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x81 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x6d as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x6e as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x7f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x6f as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x7e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x70 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x7d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x71 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x7c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x72 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x7b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x73 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x7a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x74 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x79 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x75 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x78 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x76 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x77 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x77 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x76 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x78 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x75 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x79 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x74 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x7a as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x73 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x7b as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x72 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x7c as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x71 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x7d as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x70 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x7e as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x6f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x7f as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x6e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x81 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x6d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x82 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x6c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x83 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x6b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x84 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x6a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x85 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x69 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x86 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x68 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x87 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x67 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x88 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x66 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x89 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x65 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x8a as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x64 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x8b as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x63 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x8c as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x62 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x8d as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x61 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x8e as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x60 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x8f as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x5f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x90 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x5e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x91 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x5d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x92 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x5c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x93 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x5b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x94 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x5a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x95 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x59 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x96 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x58 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x97 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x57 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x98 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x56 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x99 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x55 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x9a as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x54 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x9b as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x53 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x9c as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x52 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x9d as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x51 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x9e as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x50 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0x9f as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x4f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa0 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x4e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa1 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x4d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa2 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x4c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa3 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x4b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa4 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x4a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa5 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x49 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa6 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x48 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa7 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x47 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa8 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x46 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xa9 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x45 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xaa as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x44 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xab as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x43 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xac as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x42 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xad as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x41 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xae as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xaf as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x3f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb0 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x3e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb1 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x3d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb2 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x3c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb3 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x3b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb4 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x3a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb5 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x39 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb6 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x38 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb7 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x37 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb8 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x36 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xb9 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x35 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xba as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x34 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xbb as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x33 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xbc as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x32 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xbd as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x31 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xbe as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x30 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xbf as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x2f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc0 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x2e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc1 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x2d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc2 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x2c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc3 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x2b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc4 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x2a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc5 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x29 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc6 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x28 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc7 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x27 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc8 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x26 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xc9 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x25 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xca as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x24 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xcb as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x23 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xcc as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x22 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xcd as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x21 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xce as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x20 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xcf as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x1f as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd0 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x1e as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd1 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x1d as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd2 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x1c as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd3 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x1b as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd4 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x1a as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd6 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x19 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd7 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x18 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd8 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x17 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd9 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x16 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xda as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x15 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xdb as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x14 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xdc as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x13 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xde as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x12 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xdf as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x11 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe1 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0x10 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe2 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0xf as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe4 as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0xe as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xef as libc::c_int as int16_t,
                           codeLen: 10 as libc::c_int as uint16_t,
                           code: 0xd as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xd5 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x19 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xdd as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x18 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe3 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x17 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe5 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x16 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe6 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x15 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe7 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x14 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe8 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x13 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xe9 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x12 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xea as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x11 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xeb as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x10 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xec as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0xf as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xed as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0xe as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xee as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0xd as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf2 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0xc as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf3 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0xb as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf4 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0xa as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf5 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x9 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf6 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x8 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf7 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x7 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf8 as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x6 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xfa as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x5 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xfb as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x4 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xfc as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x3 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xfd as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x2 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xfe as libc::c_int as int16_t,
                           codeLen: 11 as libc::c_int as uint16_t,
                           code: 0x1 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: 0xf9 as libc::c_int as int16_t,
                           codeLen: 12 as libc::c_int as uint16_t,
                           code: 0x1 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTree_s{value: -(1 as libc::c_int) as int16_t,
                           codeLen: 12 as libc::c_int as uint16_t,
                           code: 0 as libc::c_int as uint16_t,};
         init
     }];
#[no_mangle]
pub static mut huffManLenIndex: [libc::c_int; 257] = [0; 257];
#[no_mangle]
pub unsafe extern "C" fn huffmanInitDecodeLenIndex() {
    // create an index of first code at each possible length
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 257 as libc::c_int {
        huffManLenIndex[ii as usize] = -(1 as libc::c_int);
        ii += 1
    }
    let mut ii_0: libc::c_int = 0 as libc::c_int;
    while ii_0 < 257 as libc::c_int {
        if huffManLenIndex[huffmanTree[ii_0 as usize].codeLen as usize] ==
               -(1 as libc::c_int) {
            huffManLenIndex[huffmanTree[ii_0 as usize].codeLen as usize] =
                ii_0
        }
        ii_0 += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn TEST(mut HuffmanUnittest: libc::c_int,
                              mut TestHuffmanEncode: libc::c_int)
 -> libc::c_int {
    let inBuf1: [uint8_t; 3] =
        [0 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
         1 as libc::c_int as uint8_t];
    // 11 101 101
    // 1110 1101
    // e    d
    let mut len: libc::c_int = 0;
    EXPECT_EQ(1 as libc::c_int, len);
    EXPECT_EQ(0xed as libc::c_int,
              outBuf[0 as libc::c_int as usize] as libc::c_int);
    let inBuf2: [uint8_t; 4] =
        [0 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
         2 as libc::c_int as uint8_t, 3 as libc::c_int as uint8_t];
    // 11 101 1001 10001
    // 1110 1100 1100 01
    // e    c    c    8
    EXPECT_EQ(2 as libc::c_int, len);
    EXPECT_EQ(0xec as libc::c_int,
              outBuf[0 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0xc4 as libc::c_int,
              outBuf[1 as libc::c_int as usize] as libc::c_int);
    let inBuf3: [uint8_t; 8] =
        [0 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
         2 as libc::c_int as uint8_t, 3 as libc::c_int as uint8_t,
         4 as libc::c_int as uint8_t, 5 as libc::c_int as uint8_t,
         6 as libc::c_int as uint8_t, 7 as libc::c_int as uint8_t];
    // 11 101 1001 10001 10000 011101 011100 011011
    // 1110 1100 1100 0110 0000 1110 1011 1000 1101 1
    // e    c    c    6    0    e    b    8    d    8
    EXPECT_EQ(5 as libc::c_int, len);
    EXPECT_EQ(0xec as libc::c_int,
              outBuf[0 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0xc6 as libc::c_int,
              outBuf[1 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0xe as libc::c_int,
              outBuf[2 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0xb8 as libc::c_int,
              outBuf[3 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0xd8 as libc::c_int,
              outBuf[4 as libc::c_int as usize] as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// 11 101 101
    // 1110 1101
    // e    d
// 11 101 1001 10001
    // 1110 1100 1100 01
    // e    c    c    8
// 11 101 1001 10001 10000 011101 011100 011011
    // 1110 1100 1100 0110 0000 1110 1011 1000 1101 1
    // e    c    c    6    0    e    b    8    d    8
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut HuffmanUnittest: libc::c_int,
                                mut TestHuffmanDecode: libc::c_int)
 -> libc::c_int {
    let mut len: libc::c_int = 0; // 11
    let inBuf1: [uint8_t; 1] = [0xc0 as libc::c_int as uint8_t]; // 11 101 101
    len =
        huffmanDecodeBuf(outBuf.as_mut_ptr(), 128 as libc::c_int,
                         inBuf1.as_ptr(), 1 as libc::c_int, 1 as libc::c_int,
                         huffmanTree.as_ptr());
    EXPECT_EQ(1 as libc::c_int, len);
    EXPECT_EQ(0 as libc::c_int,
              outBuf[0 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(-(1 as libc::c_int),
              huffManLenIndex[0 as libc::c_int as usize]);
    EXPECT_EQ(-(1 as libc::c_int),
              huffManLenIndex[1 as libc::c_int as usize]);
    EXPECT_EQ(0 as libc::c_int, huffManLenIndex[2 as libc::c_int as usize]);
    EXPECT_EQ(1 as libc::c_int, huffManLenIndex[3 as libc::c_int as usize]);
    EXPECT_EQ(2 as libc::c_int, huffManLenIndex[4 as libc::c_int as usize]);
    EXPECT_EQ(3 as libc::c_int, huffManLenIndex[5 as libc::c_int as usize]);
    EXPECT_EQ(6 as libc::c_int, huffManLenIndex[6 as libc::c_int as usize]);
    EXPECT_EQ(11 as libc::c_int, huffManLenIndex[7 as libc::c_int as usize]);
    let inBuf2: [uint8_t; 1] = [0xed as libc::c_int as uint8_t];
    len =
        huffmanDecodeBuf(outBuf.as_mut_ptr(), 128 as libc::c_int,
                         inBuf2.as_ptr(), 1 as libc::c_int, 3 as libc::c_int,
                         huffmanTree.as_ptr());
    EXPECT_EQ(3 as libc::c_int, len);
    EXPECT_EQ(0 as libc::c_int,
              outBuf[0 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x1 as libc::c_int,
              outBuf[1 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x1 as libc::c_int,
              outBuf[2 as libc::c_int as usize] as libc::c_int);
    let inBuf3: [uint8_t; 5] =
        [0xec as libc::c_int as uint8_t, 0xc6 as libc::c_int as uint8_t,
         0xe as libc::c_int as uint8_t, 0xb8 as libc::c_int as uint8_t,
         0xd8 as libc::c_int as uint8_t];
    len =
        huffmanDecodeBuf(outBuf.as_mut_ptr(), 128 as libc::c_int,
                         inBuf3.as_ptr(), 5 as libc::c_int, 8 as libc::c_int,
                         huffmanTree.as_ptr());
    EXPECT_EQ(8 as libc::c_int, len);
    EXPECT_EQ(0 as libc::c_int,
              outBuf[0 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x1 as libc::c_int,
              outBuf[1 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x2 as libc::c_int,
              outBuf[2 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x3 as libc::c_int,
              outBuf[3 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x4 as libc::c_int,
              outBuf[4 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x5 as libc::c_int,
              outBuf[5 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x6 as libc::c_int,
              outBuf[6 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(0x7 as libc::c_int,
              outBuf[7 as libc::c_int as usize] as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// STUBS
