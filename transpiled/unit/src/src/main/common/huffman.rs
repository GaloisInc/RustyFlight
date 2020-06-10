use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct huffmanState_s {
    pub bytesWritten: uint16_t,
    pub outByte: *mut uint8_t,
    pub outBufLen: uint16_t,
    pub outBit: uint8_t,
}
pub type huffmanState_t = huffmanState_s;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
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
#[no_mangle]
pub unsafe extern "C" fn huffmanEncodeBuf(mut outBuf: *mut uint8_t,
                                          mut outBufLen: libc::c_int,
                                          mut inBuf: *const uint8_t,
                                          mut inLen: libc::c_int,
                                          mut huffmanTable:
                                              *const huffmanTable_t)
 -> libc::c_int {
    let mut ret: libc::c_int = 0 as libc::c_int;
    let mut outByte: *mut uint8_t = outBuf;
    *outByte = 0 as libc::c_int as uint8_t;
    let mut outBit: uint8_t = 0x80 as libc::c_int as uint8_t;
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < inLen {
        let huffCodeLen: libc::c_int =
            (*huffmanTable.offset(*inBuf as isize)).codeLen as libc::c_int;
        let huffCode: uint16_t = (*huffmanTable.offset(*inBuf as isize)).code;
        inBuf = inBuf.offset(1);
        let mut testBit: uint16_t = 0x8000 as libc::c_int as uint16_t;
        let mut jj: libc::c_int = 0 as libc::c_int;
        while jj < huffCodeLen {
            if huffCode as libc::c_int & testBit as libc::c_int != 0 {
                *outByte =
                    (*outByte as libc::c_int | outBit as libc::c_int) as
                        uint8_t
            }
            testBit =
                (testBit as libc::c_int >> 1 as libc::c_int) as uint16_t;
            outBit = (outBit as libc::c_int >> 1 as libc::c_int) as uint8_t;
            if outBit as libc::c_int == 0 as libc::c_int {
                outBit = 0x80 as libc::c_int as uint8_t;
                outByte = outByte.offset(1);
                *outByte = 0 as libc::c_int as uint8_t;
                ret += 1
            }
            if ret >= outBufLen && ii < inLen - 1 as libc::c_int &&
                   jj < huffCodeLen - 1 as libc::c_int {
                return -(1 as libc::c_int)
            }
            jj += 1
        }
        ii += 1
    }
    if outBit as libc::c_int != 0x80 as libc::c_int {
        // ensure last character in output buffer is counted
        ret += 1
    }
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn huffmanEncodeBufStreaming(mut state:
                                                       *mut huffmanState_t,
                                                   mut inBuf: *const uint8_t,
                                                   mut inLen: libc::c_int,
                                                   mut huffmanTable:
                                                       *const huffmanTable_t)
 -> libc::c_int {
    let mut savedOutBytePtr: *mut uint8_t = (*state).outByte;
    let mut savedOutByte: uint8_t = *savedOutBytePtr;
    let mut pos: *const uint8_t = inBuf;
    let mut end: *const uint8_t = inBuf.offset(inLen as isize);
    while pos < end {
        let huffCodeLen: libc::c_int =
            (*huffmanTable.offset(*pos as isize)).codeLen as libc::c_int;
        let huffCode: uint16_t = (*huffmanTable.offset(*pos as isize)).code;
        let mut testBit: uint16_t = 0x8000 as libc::c_int as uint16_t;
        let mut jj: libc::c_int = 0 as libc::c_int;
        while jj < huffCodeLen {
            if huffCode as libc::c_int & testBit as libc::c_int != 0 {
                *(*state).outByte =
                    (*(*state).outByte as libc::c_int |
                         (*state).outBit as libc::c_int) as uint8_t
            }
            testBit =
                (testBit as libc::c_int >> 1 as libc::c_int) as uint16_t;
            (*state).outBit =
                ((*state).outBit as libc::c_int >> 1 as libc::c_int) as
                    uint8_t;
            if (*state).outBit as libc::c_int == 0 as libc::c_int {
                (*state).outBit = 0x80 as libc::c_int as uint8_t;
                (*state).outByte = (*state).outByte.offset(1);
                *(*state).outByte = 0 as libc::c_int as uint8_t;
                (*state).bytesWritten = (*state).bytesWritten.wrapping_add(1)
            }
            // if buffer is filled and we haven't finished compressing
            if (*state).bytesWritten as libc::c_int >=
                   (*state).outBufLen as libc::c_int &&
                   (pos < end.offset(-(1 as libc::c_int as isize)) ||
                        jj < huffCodeLen - 1 as libc::c_int) {
                // restore savedOutByte
                *savedOutBytePtr = savedOutByte;
                return -(1 as libc::c_int)
            }
            jj += 1
        }
        pos = pos.offset(1)
    }
    return 0 as libc::c_int;
}
