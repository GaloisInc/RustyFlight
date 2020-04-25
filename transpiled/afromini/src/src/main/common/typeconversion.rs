use core;
use libc;
extern "C" {
    #[no_mangle]
    fn strncpy(_: *mut libc::c_char, _: *const libc::c_char, _: libc::c_ulong)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strcat(_: *mut libc::c_char, _: *const libc::c_char)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
}
pub type __uint8_t = libc::c_uchar;
pub type __int32_t = libc::c_int;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
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
pub unsafe extern "C" fn uli2a(mut num: libc::c_ulong, mut base: libc::c_uint,
                               mut uc: libc::c_int,
                               mut bf: *mut libc::c_char) {
    let mut d: libc::c_uint = 1i32 as libc::c_uint;
    while num.wrapping_div(d as libc::c_ulong) >= base as libc::c_ulong {
        d = d.wrapping_mul(base)
    }
    while d != 0i32 as libc::c_uint {
        let mut dgt: libc::c_int =
            num.wrapping_div(d as libc::c_ulong) as libc::c_int;
        let fresh0 = bf;
        bf = bf.offset(1);
        *fresh0 =
            (dgt +
                 (if dgt < 10i32 {
                      '0' as i32
                  } else {
                      ((if uc != 0 { 'A' as i32 } else { 'a' as i32 })) -
                          10i32
                  })) as libc::c_char;
        // Next digit
        num = num.wrapping_rem(d as libc::c_ulong);
        d = d.wrapping_div(base)
    }
    *bf = 0i32 as libc::c_char;
}
#[no_mangle]
pub unsafe extern "C" fn li2a(mut num: libc::c_long,
                              mut bf: *mut libc::c_char) {
    if num < 0i32 as libc::c_long {
        num = -num;
        let fresh1 = bf;
        bf = bf.offset(1);
        *fresh1 = '-' as i32 as libc::c_char
    }
    uli2a(num as libc::c_ulong, 10i32 as libc::c_uint, 0i32, bf);
}
#[no_mangle]
pub unsafe extern "C" fn ui2a(mut num: libc::c_uint, mut base: libc::c_uint,
                              mut uc: libc::c_int,
                              mut bf: *mut libc::c_char) {
    let mut d: libc::c_uint = 1i32 as libc::c_uint;
    while num.wrapping_div(d) >= base { d = d.wrapping_mul(base) }
    while d != 0i32 as libc::c_uint {
        let mut dgt: libc::c_int = num.wrapping_div(d) as libc::c_int;
        let fresh2 = bf;
        bf = bf.offset(1);
        *fresh2 =
            (dgt +
                 (if dgt < 10i32 {
                      '0' as i32
                  } else {
                      ((if uc != 0 { 'A' as i32 } else { 'a' as i32 })) -
                          10i32
                  })) as libc::c_char;
        // Next digit
        num = num.wrapping_rem(d);
        d = d.wrapping_div(base)
    }
    *bf = 0i32 as libc::c_char;
}
#[no_mangle]
pub unsafe extern "C" fn i2a(mut num: libc::c_int,
                             mut bf: *mut libc::c_char) {
    if num < 0i32 {
        num = -num;
        let fresh3 = bf;
        bf = bf.offset(1);
        *fresh3 = '-' as i32 as libc::c_char
    }
    ui2a(num as libc::c_uint, 10i32 as libc::c_uint, 0i32, bf);
}
#[no_mangle]
pub unsafe extern "C" fn a2d(mut ch: libc::c_char) -> libc::c_int {
    if ch as libc::c_int >= '0' as i32 && ch as libc::c_int <= '9' as i32 {
        return ch as libc::c_int - '0' as i32
    } else if ch as libc::c_int >= 'a' as i32 &&
                  ch as libc::c_int <= 'f' as i32 {
        return ch as libc::c_int - 'a' as i32 + 10i32
    } else if ch as libc::c_int >= 'A' as i32 &&
                  ch as libc::c_int <= 'F' as i32 {
        return ch as libc::c_int - 'A' as i32 + 10i32
    } else { return -1i32 };
}
#[no_mangle]
pub unsafe extern "C" fn a2i(mut ch: libc::c_char,
                             mut src: *mut *const libc::c_char,
                             mut base: libc::c_int,
                             mut nump: *mut libc::c_int) -> libc::c_char {
    let mut p: *const libc::c_char = *src;
    let mut num: libc::c_int = 0i32;
    let mut digit: libc::c_int = 0;
    loop  {
        digit = a2d(ch);
        if !(digit >= 0i32) { break ; }
        if digit > base { break ; }
        num = num * base + digit;
        let fresh4 = p;
        p = p.offset(1);
        ch = *fresh4
    }
    *src = p;
    *nump = num;
    return ch;
}
/*
 ** The following two functions together make up an itoa()
 ** implementation. Function i2a() is a 'private' function
 ** called by the public itoa() function.
 **
 ** itoa() takes three arguments:
 **        1) the integer to be converted,
 **        2) a pointer to a character conversion buffer,
 **        3) the radix for the conversion
 **           which can range between 2 and 36 inclusive
 **           range errors on the radix default it to base10
 ** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
 */
unsafe extern "C" fn _i2a(mut i: libc::c_uint, mut a: *mut libc::c_char,
                          mut base: libc::c_uint) -> *mut libc::c_char {
    if i.wrapping_div(base) > 0i32 as libc::c_uint {
        a = _i2a(i.wrapping_div(base), a, base)
    }
    *a =
        (*::core::mem::transmute::<&[u8; 37],
                                   &[libc::c_char; 37]>(b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ\x00"))[i.wrapping_rem(base)
                                                                                                          as
                                                                                                          usize];
    return a.offset(1);
}
#[no_mangle]
pub unsafe extern "C" fn itoa(mut i: libc::c_int, mut a: *mut libc::c_char,
                              mut base: libc::c_int) -> *mut libc::c_char {
    if base < 2i32 || base > 36i32 { base = 10i32 }
    if i < 0i32 {
        *a = '-' as i32 as libc::c_char;
        *_i2a((i as libc::c_uint).wrapping_neg(), a.offset(1),
              base as libc::c_uint) = 0i32 as libc::c_char
    } else {
        *_i2a(i as libc::c_uint, a, base as libc::c_uint) =
            0i32 as libc::c_char
    }
    return a;
}
#[no_mangle]
pub unsafe extern "C" fn ftoa(mut x: libc::c_float,
                              mut floatString: *mut libc::c_char)
 -> *mut libc::c_char {
    let mut value: int32_t = 0;
    let mut intString1: [libc::c_char; 12] = [0; 12];
    let mut intString2: [libc::c_char; 12] =
        [0i32 as libc::c_char, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let mut decimalPoint: *mut libc::c_char =
        b".\x00" as *const u8 as *const libc::c_char as *mut libc::c_char;
    let mut dpLocation: uint8_t = 0;
    if x > 0i32 as libc::c_float {
        // Rounding for x.xxx display format
        x += 0.0005f32
    } else { x -= 0.0005f32 } // Convert float * 1000 to an integer
    value =
        (x * 1000.0f32) as int32_t; // Create string from abs of integer value
    itoa(({
              let mut _x: int32_t =
                  value; // Negative number, add a negative sign
              if _x > 0i32 { _x } else { -_x }
          }), intString1.as_mut_ptr(),
         10i32); // Positive number, add a pad space
    if value >= 0i32 {
        intString2[0] = ' ' as i32 as libc::c_char
    } else { intString2[0] = '-' as i32 as libc::c_char }
    if strlen(intString1.as_mut_ptr()) == 1i32 as libc::c_ulong {
        intString2[1] = '0' as i32 as libc::c_char;
        intString2[2] = '0' as i32 as libc::c_char;
        intString2[3] = '0' as i32 as libc::c_char;
        strcat(intString2.as_mut_ptr(), intString1.as_mut_ptr());
    } else if strlen(intString1.as_mut_ptr()) == 2i32 as libc::c_ulong {
        intString2[1] = '0' as i32 as libc::c_char;
        intString2[2] = '0' as i32 as libc::c_char;
        strcat(intString2.as_mut_ptr(), intString1.as_mut_ptr());
    } else if strlen(intString1.as_mut_ptr()) == 3i32 as libc::c_ulong {
        intString2[1] = '0' as i32 as libc::c_char;
        strcat(intString2.as_mut_ptr(), intString1.as_mut_ptr());
    } else { strcat(intString2.as_mut_ptr(), intString1.as_mut_ptr()); }
    dpLocation =
        strlen(intString2.as_mut_ptr()).wrapping_sub(3i32 as libc::c_ulong) as
            uint8_t;
    strncpy(floatString, intString2.as_mut_ptr(),
            dpLocation as libc::c_ulong);
    *floatString.offset(dpLocation as isize) = '\u{0}' as i32 as libc::c_char;
    strcat(floatString, decimalPoint);
    strcat(floatString,
           intString2.as_mut_ptr().offset(dpLocation as libc::c_int as
                                              isize));
    return floatString;
}
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
#[no_mangle]
pub unsafe extern "C" fn fastA2F(mut p: *const libc::c_char)
 -> libc::c_float {
    let mut frac: libc::c_int = 0i32;
    let mut sign: libc::c_float = 0.;
    let mut value: libc::c_float = 0.;
    let mut scale: libc::c_float = 0.;
    // Skip leading white space, if any.
    while *p as libc::c_int == ' ' as i32 || *p as libc::c_int == '\t' as i32
          {
        p = p.offset(1)
    }
    // Get sign, if any.
    sign = 1.0f32;
    if *p as libc::c_int == '-' as i32 {
        sign = -1.0f32;
        p = p.offset(1)
    } else if *p as libc::c_int == '+' as i32 { p = p.offset(1) }
    // Get digits before decimal point or exponent, if any.
    value = 0.0f32;
    while *p as libc::c_int >= '0' as i32 && *p as libc::c_int <= '9' as i32 {
        value =
            value * 10.0f32 +
                (*p as libc::c_int - '0' as i32) as libc::c_float;
        p = p.offset(1)
    }
    // Get digits after decimal point, if any.
    if *p as libc::c_int == '.' as i32 {
        let mut pow10: libc::c_float = 10.0f32;
        p = p.offset(1);
        while *p as libc::c_int >= '0' as i32 &&
                  *p as libc::c_int <= '9' as i32 {
            value +=
                (*p as libc::c_int - '0' as i32) as libc::c_float / pow10;
            pow10 *= 10.0f32;
            p = p.offset(1)
        }
    }
    // Handle exponent, if any.
    scale = 1.0f32;
    if *p as libc::c_int == 'e' as i32 || *p as libc::c_int == 'E' as i32 {
        let mut expon: libc::c_uint = 0;
        p = p.offset(1);
        // Get sign of exponent, if any.
        frac = 0i32;
        if *p as libc::c_int == '-' as i32 {
            frac = 1i32;
            p = p.offset(1)
        } else if *p as libc::c_int == '+' as i32 { p = p.offset(1) }
        // Get digits of exponent, if any.
        expon = 0i32 as libc::c_uint;
        while *p as libc::c_int >= '0' as i32 &&
                  *p as libc::c_int <= '9' as i32 {
            expon =
                expon.wrapping_mul(10i32 as
                                       libc::c_uint).wrapping_add((*p as
                                                                       libc::c_int
                                                                       -
                                                                       '0' as
                                                                           i32)
                                                                      as
                                                                      libc::c_uint);
            p = p.offset(1)
        }
        if expon > 308i32 as libc::c_uint { expon = 308i32 as libc::c_uint }
        // Calculate scaling factor.
        // while (expon >= 50) { scale *= 1E50f; expon -= 50; }
        while expon >= 8i32 as libc::c_uint {
            scale *= 1E8f32;
            expon = expon.wrapping_sub(8i32 as libc::c_uint)
        }
        while expon > 0i32 as libc::c_uint {
            scale *= 10.0f32;
            expon = expon.wrapping_sub(1i32 as libc::c_uint)
        }
    }
    // Return signed and scaled floating point result.
    return sign * (if frac != 0 { (value) / scale } else { (value) * scale });
}
