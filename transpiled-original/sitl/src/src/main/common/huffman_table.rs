use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct huffmanTable_s {
    pub codeLen: uint8_t,
    pub code: uint16_t,
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// 256 characters plus EOF
pub type huffmanTable_t = huffmanTable_s;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * Huffman Table, used to compress a bytestream
 */
#[no_mangle]
pub static mut huffmanTable: [huffmanTable_t; 257] =
    [{
         let mut init =
             huffmanTable_s{codeLen: 2i32 as uint8_t,
                            code: 0xc000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 3i32 as uint8_t,
                            code: 0xa000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 4i32 as uint8_t,
                            code: 0x9000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 5i32 as uint8_t,
                            code: 0x8800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 5i32 as uint8_t,
                            code: 0x8000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6i32 as uint8_t,
                            code: 0x7400i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6i32 as uint8_t,
                            code: 0x7000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6i32 as uint8_t,
                            code: 0x6c00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6i32 as uint8_t,
                            code: 0x6800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x6200i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x6000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x5e00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x5c00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x5a00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x5800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x5600i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6i32 as uint8_t,
                            code: 0x6400i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x5400i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7i32 as uint8_t,
                            code: 0x5200i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x5100i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x5000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4f00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4e00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4d00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4c00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4b00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4a00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4900i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4700i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4600i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4500i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4400i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4300i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4200i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4100i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x4000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3c80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3c00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3b80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3b00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3a80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3a00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3980i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3900i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3880i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3780i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x3f00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3700i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3680i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3600i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3580i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3500i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3480i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3400i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3380i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3300i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3280i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3200i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3180i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3100i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3080i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x3000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x3e00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2f80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2f00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2e80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2e00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2d80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2d00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2c80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2c00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2b80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x27c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2780i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2b00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2740i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2700i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2a80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 5i32 as uint8_t,
                            code: 0x7800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2a00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x26c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2680i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2640i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2600i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x25c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2580i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2540i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2500i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x24c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2480i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2440i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2400i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x23c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2380i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2340i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2300i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x22c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2280i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2240i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2200i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x21c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2180i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2140i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2100i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x20c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2080i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2040i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x2000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1fc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1f80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1f40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1f00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1ec0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1e80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1e40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1e00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1dc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1d80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1d40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1d00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1cc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1c80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1c40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1c00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1bc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1b80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2980i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1b40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1b00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1ac0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1a80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1a40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1a00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x19c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1980i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1940i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1900i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x18c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1880i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1840i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x17c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1780i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1740i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1700i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x16c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1680i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1640i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1600i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x15c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1580i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1540i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1500i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x14c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1480i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1440i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1400i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x13c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1380i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1340i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1300i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x12c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1280i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1240i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1200i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x11c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1180i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1140i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1100i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x10c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1080i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1040i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x1000i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xfc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xf80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xf40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xf00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xec0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xe80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xe40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xe00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xdc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xd80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xd40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xd00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xcc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xc80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xc40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xc00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xbc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xb80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xb40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xb00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xac0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xa80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xa40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0xa00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x9c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x980i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x940i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x900i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x8c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x880i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x840i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x7c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x780i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x740i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x700i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x6c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x680i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x320i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x640i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x600i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x5c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x580i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x540i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x500i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x4c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x300i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x480i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x440i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2900i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x400i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x3c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x2e0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x380i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x2c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x2a0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x280i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x260i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x240i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x220i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x200i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x1e0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x1c0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x1a0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10i32 as uint8_t,
                            code: 0x340i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8i32 as uint8_t,
                            code: 0x3d00i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2880i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x180i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x160i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x140i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x120i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x100i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0xe0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0xc0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 12i32 as uint8_t,
                            code: 0x10i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0xa0i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x80i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x60i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x40i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11i32 as uint8_t,
                            code: 0x20i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9i32 as uint8_t,
                            code: 0x2800i32 as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 12i32 as uint8_t,
                            code: 0i32 as uint16_t,};
         init
     }];
