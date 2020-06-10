use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
#[derive(Copy, Clone)]
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
             huffmanTable_s{codeLen: 2 as libc::c_int as uint8_t,
                            code: 0xc000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 3 as libc::c_int as uint8_t,
                            code: 0xa000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 4 as libc::c_int as uint8_t,
                            code: 0x9000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 5 as libc::c_int as uint8_t,
                            code: 0x8800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 5 as libc::c_int as uint8_t,
                            code: 0x8000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6 as libc::c_int as uint8_t,
                            code: 0x7400 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6 as libc::c_int as uint8_t,
                            code: 0x7000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6 as libc::c_int as uint8_t,
                            code: 0x6c00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6 as libc::c_int as uint8_t,
                            code: 0x6800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x6200 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x6000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x5e00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x5c00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x5a00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x5800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x5600 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 6 as libc::c_int as uint8_t,
                            code: 0x6400 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x5400 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 7 as libc::c_int as uint8_t,
                            code: 0x5200 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x5100 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x5000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4f00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4e00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4d00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4c00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4b00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4a00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4900 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4700 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4600 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4500 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4400 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4300 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4200 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4100 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x4000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3c80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3c00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3b80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3b00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3a80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3a00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3980 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3900 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3880 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3780 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x3f00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3700 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3680 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3600 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3580 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3500 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3480 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3400 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3380 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3300 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3280 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3200 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3180 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3100 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3080 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x3000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x3e00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2f80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2f00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2e80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2e00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2d80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2d00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2c80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2c00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2b80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x27c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2780 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2b00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2740 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2700 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2a80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 5 as libc::c_int as uint8_t,
                            code: 0x7800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2a00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x26c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2680 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2640 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2600 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x25c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2580 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2540 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2500 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x24c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2480 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2440 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2400 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x23c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2380 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2340 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2300 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x22c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2280 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2240 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2200 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x21c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2180 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2140 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2100 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x20c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2080 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2040 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x2000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1fc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1f80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1f40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1f00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1ec0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1e80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1e40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1e00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1dc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1d80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1d40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1d00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1cc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1c80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1c40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1c00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1bc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1b80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2980 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1b40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1b00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1ac0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1a80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1a40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1a00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x19c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1980 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1940 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1900 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x18c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1880 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1840 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x17c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1780 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1740 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1700 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x16c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1680 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1640 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1600 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x15c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1580 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1540 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1500 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x14c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1480 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1440 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1400 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x13c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1380 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1340 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1300 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x12c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1280 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1240 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1200 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x11c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1180 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1140 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1100 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x10c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1080 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1040 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x1000 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xfc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xf80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xf40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xf00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xec0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xe80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xe40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xe00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xdc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xd80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xd40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xd00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xcc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xc80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xc40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xc00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xbc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xb80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xb40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xb00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xac0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xa80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xa40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0xa00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x9c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x980 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x940 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x900 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x8c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x880 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x840 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x7c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x780 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x740 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x700 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x6c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x680 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x320 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x640 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x600 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x5c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x580 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x540 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x500 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x4c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x300 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x480 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x440 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2900 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x400 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x3c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x2e0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x380 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x2c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x2a0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x280 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x260 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x240 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x220 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x200 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x1e0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x1c0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x1a0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 10 as libc::c_int as uint8_t,
                            code: 0x340 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 8 as libc::c_int as uint8_t,
                            code: 0x3d00 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2880 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x180 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x160 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x140 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x120 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x100 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0xe0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0xc0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 12 as libc::c_int as uint8_t,
                            code: 0x10 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0xa0 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x80 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x60 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x40 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 11 as libc::c_int as uint8_t,
                            code: 0x20 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 9 as libc::c_int as uint8_t,
                            code: 0x2800 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             huffmanTable_s{codeLen: 12 as libc::c_int as uint8_t,
                            code: 0 as libc::c_int as uint16_t,};
         init
     }];
