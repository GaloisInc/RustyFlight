use core;
use libc;
extern "C" {
    /*
 * We want to limit how bursty our writes to the device are. Note that this will also restrict the maximum size of a
 * header write we can make:
 */
    /*
 * Ideally, each iteration in which we are logging headers would write a similar amount of data to the device as a
 * regular logging iteration. This way we won't hog the CPU by making a gigantic write:
 */
    #[no_mangle]
    static mut blackboxHeaderBudget: int32_t;
    #[no_mangle]
    fn blackboxWrite(value: uint8_t);
    #[no_mangle]
    fn blackboxWriteString(s: *const libc::c_char) -> libc::c_int;
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
    fn castFloatBytesToInt(f: libc::c_float) -> uint32_t;
    #[no_mangle]
    fn zigzagEncode(value: int32_t) -> uint32_t;
    #[no_mangle]
    fn tfp_format(putp: *mut libc::c_void,
                  putf:
                      Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                                  _: libc::c_char) -> ()>,
                  fmt: *const libc::c_char, va: ::core::ffi::VaList)
     -> libc::c_int;
}
pub type __builtin_va_list = [__va_list_tag; 1];
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __va_list_tag {
    pub gp_offset: libc::c_uint,
    pub fp_offset: libc::c_uint,
    pub overflow_arg_area: *mut libc::c_void,
    pub reg_save_area: *mut libc::c_void,
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type va_list = __builtin_va_list;
pub const BYTES_4: C2RustUnnamed = 3;
pub const BYTES_3: C2RustUnnamed = 2;
pub const BYTES_2: C2RustUnnamed = 1;
pub const BYTES_1: C2RustUnnamed = 0;
pub const BITS_2: C2RustUnnamed_0 = 0;
pub const BITS_32: C2RustUnnamed_0 = 3;
pub const BITS_6: C2RustUnnamed_0 = 2;
pub const BITS_4: C2RustUnnamed_0 = 1;
pub type C2RustUnnamed = libc::c_uint;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const BITS_2_0: C2RustUnnamed_2 = 0;
pub const BYTES_4_0: C2RustUnnamed_1 = 3;
pub const BYTES_3_0: C2RustUnnamed_1 = 2;
pub const BYTES_2_0: C2RustUnnamed_1 = 1;
pub const BYTES_1_0: C2RustUnnamed_1 = 0;
pub const BITS_32_0: C2RustUnnamed_2 = 3;
pub const BITS_877: C2RustUnnamed_2 = 2;
pub const BITS_554: C2RustUnnamed_2 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FIELD_16BIT: C2RustUnnamed_3 = 3;
pub const FIELD_8BIT: C2RustUnnamed_3 = 2;
pub const FIELD_4BIT: C2RustUnnamed_3 = 1;
pub const FIELD_ZERO: C2RustUnnamed_3 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
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
unsafe extern "C" fn _putc(mut p: *mut libc::c_void, mut c: libc::c_char) {
    blackboxWrite(c as uint8_t);
}
unsafe extern "C" fn blackboxPrintfv(mut fmt: *const libc::c_char,
                                     mut va: ::core::ffi::VaList)
 -> libc::c_int {
    return tfp_format(0 as *mut libc::c_void,
                      Some(_putc as
                               unsafe extern "C" fn(_: *mut libc::c_void,
                                                    _: libc::c_char) -> ()),
                      fmt, va.as_va_list());
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
//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
#[no_mangle]
pub unsafe extern "C" fn blackboxPrintf(mut fmt: *const libc::c_char,
                                        mut args: ...) -> libc::c_int {
    let mut va: ::core::ffi::VaListImpl;
    va = args.clone();
    let written: libc::c_int = blackboxPrintfv(fmt, va.as_va_list());
    return written;
}
/*
 * printf a Blackbox header line with a leading "H " and trailing "\n" added automatically. blackboxHeaderBudget is
 * decreased to account for the number of bytes written.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxPrintfHeaderLine(mut name:
                                                      *const libc::c_char,
                                                  mut fmt:
                                                      *const libc::c_char,
                                                  mut args: ...) {
    let mut va: ::core::ffi::VaListImpl;
    blackboxWrite('H' as i32 as uint8_t);
    blackboxWrite(' ' as i32 as uint8_t);
    blackboxWriteString(name);
    blackboxWrite(':' as i32 as uint8_t);
    va = args.clone();
    let written: libc::c_int = blackboxPrintfv(fmt, va.as_va_list());
    blackboxWrite('\n' as i32 as uint8_t);
    blackboxHeaderBudget -= written + 3i32;
}
/* *
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteUnsignedVB(mut value: uint32_t) {
    //While this isn't the final byte (we can only write 7 bits at a time)
    while value > 127i32 as libc::c_uint {
        blackboxWrite((value | 0x80i32 as libc::c_uint) as
                          uint8_t); // Set the high bit to mean "more bytes follow"
        value >>= 7i32
    }
    blackboxWrite(value as uint8_t);
}
/* *
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteSignedVB(mut value: int32_t) {
    //ZigZag encode to make the value always positive
    blackboxWriteUnsignedVB(zigzagEncode(value));
}
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteSignedVBArray(mut array: *mut int32_t,
                                                    mut count: libc::c_int) {
    let mut i: libc::c_int = 0i32;
    while i < count {
        blackboxWriteSignedVB(*array.offset(i as isize));
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteSigned16VBArray(mut array: *mut int16_t,
                                                      mut count:
                                                          libc::c_int) {
    let mut i: libc::c_int = 0i32;
    while i < count {
        blackboxWriteSignedVB(*array.offset(i as isize) as int32_t);
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteS16(mut value: int16_t) {
    blackboxWrite((value as libc::c_int & 0xffi32) as uint8_t);
    blackboxWrite((value as libc::c_int >> 8i32 & 0xffi32) as uint8_t);
}
/* *
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteTag2_3S32(mut values: *mut int32_t) {
    static mut NUM_FIELDS: libc::c_int = 3i32;
    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    let mut selector: libc::c_int = BITS_2 as libc::c_int;
    let mut selector2: libc::c_int = 0;
    /*
     * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
     * below:
     *
     * Selector possibilities
     *
     * 2 bits per field  ss11 2233,
     * 4 bits per field  ss00 1111 2222 3333
     * 6 bits per field  ss11 1111 0022 2222 0033 3333
     * 32 bits per field sstt tttt followed by fields of various byte counts
     */
    let mut x: libc::c_int = 0i32;
    while x < NUM_FIELDS {
        //Require more than 6 bits?
        if *values.offset(x as isize) >= 32i32 ||
               *values.offset(x as isize) < -32i32 {
            selector = BITS_32 as libc::c_int;
            break ;
        } else {
            //Require more than 4 bits?
            if *values.offset(x as isize) >= 8i32 ||
                   *values.offset(x as isize) < -8i32 {
                if selector < BITS_6 as libc::c_int {
                    selector = BITS_6 as libc::c_int
                }
            } else if *values.offset(x as isize) >= 2i32 ||
                          *values.offset(x as isize) < -2i32 {
                //Require more than 2 bits?
                if selector < BITS_4 as libc::c_int {
                    selector = BITS_4 as libc::c_int
                }
            }
            x += 1
        }
    }
    match selector {
        0 => {
            blackboxWrite((selector << 6i32 |
                               (*values.offset(0) & 0x3i32) << 4i32 |
                               (*values.offset(1) & 0x3i32) << 2i32 |
                               *values.offset(2) & 0x3i32) as uint8_t);
        }
        1 => {
            blackboxWrite((selector << 6i32 | *values.offset(0) & 0xfi32) as
                              uint8_t);
            blackboxWrite((*values.offset(1) << 4i32 |
                               *values.offset(2) & 0xfi32) as uint8_t);
        }
        2 => {
            blackboxWrite((selector << 6i32 | *values.offset(0) & 0x3fi32) as
                              uint8_t);
            blackboxWrite(*values.offset(1) as uint8_t);
            blackboxWrite(*values.offset(2) as uint8_t);
        }
        3 => {
            /*
         * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
         *
         * Selector2 field possibilities
         * 0 - 8 bits
         * 1 - 16 bits
         * 2 - 24 bits
         * 3 - 32 bits
         */
            selector2 = 0i32;
            //Encode in reverse order so the first field is in the low bits:
            let mut x_0: libc::c_int = NUM_FIELDS - 1i32;
            while x_0 >= 0i32 {
                selector2 <<= 2i32;
                if *values.offset(x_0 as isize) < 128i32 &&
                       *values.offset(x_0 as isize) >= -128i32 {
                    selector2 |= BYTES_1 as libc::c_int
                } else if *values.offset(x_0 as isize) < 32768i32 &&
                              *values.offset(x_0 as isize) >= -32768i32 {
                    selector2 |= BYTES_2 as libc::c_int
                } else if *values.offset(x_0 as isize) < 8388608i32 &&
                              *values.offset(x_0 as isize) >= -8388608i32 {
                    selector2 |= BYTES_3 as libc::c_int
                } else { selector2 |= BYTES_4 as libc::c_int }
                x_0 -= 1
            }
            //Write the selectors
            blackboxWrite((selector << 6i32 | selector2) as uint8_t);
            //And now the values according to the selectors we picked for them
            let mut x_1: libc::c_int = 0i32;
            while x_1 < NUM_FIELDS {
                match selector2 & 0x3i32 {
                    0 => {
                        blackboxWrite(*values.offset(x_1 as isize) as
                                          uint8_t);
                    }
                    1 => {
                        blackboxWrite(*values.offset(x_1 as isize) as
                                          uint8_t);
                        blackboxWrite((*values.offset(x_1 as isize) >> 8i32)
                                          as uint8_t);
                    }
                    2 => {
                        blackboxWrite(*values.offset(x_1 as isize) as
                                          uint8_t);
                        blackboxWrite((*values.offset(x_1 as isize) >> 8i32)
                                          as uint8_t);
                        blackboxWrite((*values.offset(x_1 as isize) >> 16i32)
                                          as uint8_t);
                    }
                    3 => {
                        blackboxWrite(*values.offset(x_1 as isize) as
                                          uint8_t);
                        blackboxWrite((*values.offset(x_1 as isize) >> 8i32)
                                          as uint8_t);
                        blackboxWrite((*values.offset(x_1 as isize) >> 16i32)
                                          as uint8_t);
                        blackboxWrite((*values.offset(x_1 as isize) >> 24i32)
                                          as uint8_t);
                    }
                    _ => { }
                }
                x_1 += 1;
                selector2 >>= 2i32
            }
        }
        _ => { }
    };
}
/* *
 * Write a 2 bit tag followed by 3 signed fields of 2, 554, 877 or 32 bits
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteTag2_3SVariable(mut values:
                                                          *mut int32_t)
 -> libc::c_int {
    static mut FIELD_COUNT: libc::c_int = 3i32;
    /*
     * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
     * below:
     *
     * Selector possibilities
     *
     * 2 bits per field  ss11 2233,
     * 554 bits per field  ss11 1112 2222 3333
     * 877 bits per field  ss11 1111 1122 2222 2333 3333
     * 32 bits per field sstt tttt followed by fields of various byte counts
     */
    let mut selector: libc::c_int = BITS_2_0 as libc::c_int;
    let mut selector2: libc::c_int = 0i32;
    // Require more than 877 bits?
    if *values.offset(0) >= 256i32 || *values.offset(0) < -256i32 ||
           *values.offset(1) >= 128i32 || *values.offset(1) < -128i32 ||
           *values.offset(2) >= 128i32 || *values.offset(2) < -128i32 {
        selector = BITS_32_0 as libc::c_int
        // Require more than 554 bits?
    } else if *values.offset(0) >= 16i32 || *values.offset(0) < -16i32 ||
                  *values.offset(1) >= 16i32 || *values.offset(1) < -16i32 ||
                  *values.offset(2) >= 8i32 || *values.offset(2) < -8i32 {
        selector = BITS_877 as libc::c_int
        // Require more than 2 bits?
    } else if *values.offset(0) >= 2i32 || *values.offset(0) < -2i32 ||
                  *values.offset(1) >= 2i32 || *values.offset(1) < -2i32 ||
                  *values.offset(2) >= 2i32 || *values.offset(2) < -2i32 {
        selector = BITS_554 as libc::c_int
    }
    match selector {
        0 => {
            blackboxWrite((selector << 6i32 |
                               (*values.offset(0) & 0x3i32) << 4i32 |
                               (*values.offset(1) & 0x3i32) << 2i32 |
                               *values.offset(2) & 0x3i32) as uint8_t);
        }
        1 => {
            // 554 bits per field  ss11 1112 2222 3333
            blackboxWrite((selector << 6i32 |
                               (*values.offset(0) & 0x1fi32) << 1i32 |
                               (*values.offset(1) & 0x1fi32) >> 4i32) as
                              uint8_t);
            blackboxWrite(((*values.offset(1) & 0xfi32) << 4i32 |
                               *values.offset(2) & 0xfi32) as uint8_t);
        }
        2 => {
            // 877 bits per field  ss11 1111 1122 2222 2333 3333
            blackboxWrite((selector << 6i32 |
                               (*values.offset(0) & 0xffi32) >> 2i32) as
                              uint8_t);
            blackboxWrite(((*values.offset(0) & 0x3i32) << 6i32 |
                               (*values.offset(1) & 0x7fi32) >> 1i32) as
                              uint8_t);
            blackboxWrite(((*values.offset(1) & 0x1i32) << 7i32 |
                               *values.offset(2) & 0x7fi32) as uint8_t);
        }
        3 => {
            /*
         * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
         *
         * Selector2 field possibilities
         * 0 - 8 bits
         * 1 - 16 bits
         * 2 - 24 bits
         * 3 - 32 bits
         */
            selector2 = 0i32;
            //Encode in reverse order so the first field is in the low bits:
            let mut x: libc::c_int = FIELD_COUNT - 1i32;
            while x >= 0i32 {
                selector2 <<= 2i32;
                if *values.offset(x as isize) < 128i32 &&
                       *values.offset(x as isize) >= -128i32 {
                    selector2 |= BYTES_1_0 as libc::c_int
                } else if *values.offset(x as isize) < 32768i32 &&
                              *values.offset(x as isize) >= -32768i32 {
                    selector2 |= BYTES_2_0 as libc::c_int
                } else if *values.offset(x as isize) < 8388608i32 &&
                              *values.offset(x as isize) >= -8388608i32 {
                    selector2 |= BYTES_3_0 as libc::c_int
                } else { selector2 |= BYTES_4_0 as libc::c_int }
                x -= 1
            }
            //Write the selectors
            blackboxWrite((selector << 6i32 | selector2) as uint8_t);
            //And now the values according to the selectors we picked for them
            let mut x_0: libc::c_int = 0i32;
            while x_0 < FIELD_COUNT {
                match selector2 & 0x3i32 {
                    0 => {
                        blackboxWrite(*values.offset(x_0 as isize) as
                                          uint8_t);
                    }
                    1 => {
                        blackboxWrite(*values.offset(x_0 as isize) as
                                          uint8_t);
                        blackboxWrite((*values.offset(x_0 as isize) >> 8i32)
                                          as uint8_t);
                    }
                    2 => {
                        blackboxWrite(*values.offset(x_0 as isize) as
                                          uint8_t);
                        blackboxWrite((*values.offset(x_0 as isize) >> 8i32)
                                          as uint8_t);
                        blackboxWrite((*values.offset(x_0 as isize) >> 16i32)
                                          as uint8_t);
                    }
                    3 => {
                        blackboxWrite(*values.offset(x_0 as isize) as
                                          uint8_t);
                        blackboxWrite((*values.offset(x_0 as isize) >> 8i32)
                                          as uint8_t);
                        blackboxWrite((*values.offset(x_0 as isize) >> 16i32)
                                          as uint8_t);
                        blackboxWrite((*values.offset(x_0 as isize) >> 24i32)
                                          as uint8_t);
                    }
                    _ => { }
                }
                x_0 += 1;
                selector2 >>= 2i32
            }
        }
        _ => { }
    }
    return selector;
}
/* *
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteTag8_4S16(mut values: *mut int32_t) {
    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    let mut selector: uint8_t = 0i32 as uint8_t;
    //Encode in reverse order so the first field is in the low bits:
    let mut x: libc::c_int = 3i32;
    while x >= 0i32 {
        selector = ((selector as libc::c_int) << 2i32) as uint8_t;
        if *values.offset(x as isize) == 0i32 {
            selector =
                (selector as libc::c_int | FIELD_ZERO as libc::c_int) as
                    uint8_t
        } else if *values.offset(x as isize) < 8i32 &&
                      *values.offset(x as isize) >= -8i32 {
            selector =
                (selector as libc::c_int | FIELD_4BIT as libc::c_int) as
                    uint8_t
        } else if *values.offset(x as isize) < 128i32 &&
                      *values.offset(x as isize) >= -128i32 {
            selector =
                (selector as libc::c_int | FIELD_8BIT as libc::c_int) as
                    uint8_t
        } else {
            selector =
                (selector as libc::c_int | FIELD_16BIT as libc::c_int) as
                    uint8_t
        }
        x -= 1
    }
    blackboxWrite(selector);
    let mut nibbleIndex: libc::c_int = 0i32;
    let mut buffer: uint8_t = 0i32 as uint8_t;
    let mut x_0: libc::c_int = 0i32;
    while x_0 < 4i32 {
        match selector as libc::c_int & 0x3i32 {
            1 => {
                if nibbleIndex == 0i32 {
                    //We fill high-bits first
                    buffer =
                        (*values.offset(x_0 as isize) << 4i32) as uint8_t;
                    nibbleIndex = 1i32
                } else {
                    blackboxWrite((buffer as libc::c_int |
                                       *values.offset(x_0 as isize) & 0xfi32)
                                      as uint8_t);
                    nibbleIndex = 0i32
                }
            }
            2 => {
                if nibbleIndex == 0i32 {
                    blackboxWrite(*values.offset(x_0 as isize) as uint8_t);
                } else {
                    //Write the high bits of the value first (mask to avoid sign extension)
                    blackboxWrite((buffer as libc::c_int |
                                       *values.offset(x_0 as isize) >> 4i32 &
                                           0xfi32) as uint8_t);
                    //Now put the leftover low bits into the top of the next buffer entry
                    buffer = (*values.offset(x_0 as isize) << 4i32) as uint8_t
                }
            }
            3 => {
                if nibbleIndex == 0i32 {
                    //Write high byte first
                    blackboxWrite((*values.offset(x_0 as isize) >> 8i32) as
                                      uint8_t);
                    blackboxWrite(*values.offset(x_0 as isize) as uint8_t);
                } else {
                    //First write the highest 4 bits
                    blackboxWrite((buffer as libc::c_int |
                                       *values.offset(x_0 as isize) >> 12i32 &
                                           0xfi32) as uint8_t);
                    // Then the middle 8
                    blackboxWrite((*values.offset(x_0 as isize) >> 4i32) as
                                      uint8_t);
                    //Only the smallest 4 bits are still left to write
                    buffer = (*values.offset(x_0 as isize) << 4i32) as uint8_t
                }
            }
            0 | _ => { }
        }
        x_0 += 1;
        selector = (selector as libc::c_int >> 2i32) as uint8_t
    }
    //Anything left over to write?
    if nibbleIndex == 1i32 { blackboxWrite(buffer); };
}
/* *
 * Write `valueCount` fields from `values` to the Blackbox using signed variable byte encoding. A 1-byte header is
 * written first which specifies which fields are non-zero (so this encoding is compact when most fields are zero).
 *
 * valueCount must be 8 or less.
 */
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteTag8_8SVB(mut values: *mut int32_t,
                                                mut valueCount: libc::c_int) {
    let mut header: uint8_t = 0;
    if valueCount > 0i32 {
        //If we're only writing one field then we can skip the header
        if valueCount == 1i32 {
            blackboxWriteSignedVB(*values.offset(0));
        } else {
            //First write a one-byte header that marks which fields are non-zero
            header = 0i32 as uint8_t;
            // First field should be in low bits of header
            let mut i: libc::c_int = valueCount - 1i32;
            while i >= 0i32 {
                header = ((header as libc::c_int) << 1i32) as uint8_t;
                if *values.offset(i as isize) != 0i32 {
                    header = (header as libc::c_int | 0x1i32) as uint8_t
                }
                i -= 1
            }
            blackboxWrite(header);
            let mut i_0: libc::c_int = 0i32;
            while i_0 < valueCount {
                if *values.offset(i_0 as isize) != 0i32 {
                    blackboxWriteSignedVB(*values.offset(i_0 as isize));
                }
                i_0 += 1
            }
        }
    };
}
/* * Write unsigned integer **/
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteU32(mut value: int32_t) {
    blackboxWrite((value & 0xffi32) as uint8_t);
    blackboxWrite((value >> 8i32 & 0xffi32) as uint8_t);
    blackboxWrite((value >> 16i32 & 0xffi32) as uint8_t);
    blackboxWrite((value >> 24i32 & 0xffi32) as uint8_t);
}
/* * Write float value in the integer form **/
#[no_mangle]
pub unsafe extern "C" fn blackboxWriteFloat(mut value: libc::c_float) {
    blackboxWriteU32(castFloatBytesToInt(value) as int32_t);
}
// BLACKBOX
