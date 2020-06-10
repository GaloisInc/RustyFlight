use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
// For v1 and Standard Capacity cards
// For v2 High Capacity cards
// Idle bit is set to 1 only when idle during intialization phase:
// These are worst-case timeouts defined for High Speed cards
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
/* *
 * Read a bitfield from an array of bits (the bit at index 0 being the most-significant bit of the first byte in
 * the buffer).
 */
#[no_mangle]
pub unsafe extern "C" fn readBitfield(mut buffer: *mut uint8_t,
                                      mut bitIndex: libc::c_uint,
                                      mut bitLen: libc::c_uint) -> uint32_t {
    let mut result: uint32_t = 0 as libc::c_int as uint32_t;
    let mut bitInByteOffset: libc::c_uint =
        bitIndex.wrapping_rem(8 as libc::c_int as libc::c_uint);
    let mut bufferByte: uint8_t = 0;
    buffer =
        buffer.offset(bitIndex.wrapping_div(8 as libc::c_int as libc::c_uint)
                          as isize);
    // Align the bitfield to be read to the top of the buffer
    bufferByte = ((*buffer as libc::c_int) << bitInByteOffset) as uint8_t;
    while bitLen > 0 as libc::c_int as libc::c_uint {
        let mut bitsThisLoop: libc::c_uint =
            ({
                 let mut _a: libc::c_uint =
                     (8 as libc::c_int as
                          libc::c_uint).wrapping_sub(bitInByteOffset);
                 let mut _b: libc::c_uint = bitLen;
                 if _a < _b { _a } else { _b }
             });
        result =
            result << bitsThisLoop |
                (bufferByte as libc::c_int >>
                     (8 as libc::c_int as
                          libc::c_uint).wrapping_sub(bitsThisLoop)) as
                    libc::c_uint;
        buffer = buffer.offset(1);
        bufferByte = *buffer;
        bitLen = bitLen.wrapping_sub(bitsThisLoop);
        bitInByteOffset = 0 as libc::c_int as libc::c_uint
    }
    return result;
}
