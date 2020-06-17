use core;
use libc;
extern "C" {
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
}
pub type size_t = libc::c_ulong;
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
 * Replacemnet for system header file <ctype.h> to avoid macro definitions
 * Functions are implemented in common/string_light.c
 */
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
pub unsafe extern "C" fn isalnum(mut c: libc::c_int) -> libc::c_int {
    return (isdigit(c) != 0 || isupper(c) != 0 || islower(c) != 0) as
               libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn isdigit(mut c: libc::c_int) -> libc::c_int {
    return (c >= '0' as i32 && c <= '9' as i32) as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn isupper(mut c: libc::c_int) -> libc::c_int {
    return (c >= 'A' as i32 && c <= 'Z' as i32) as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn islower(mut c: libc::c_int) -> libc::c_int {
    return (c >= 'a' as i32 && c <= 'z' as i32) as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn isspace(mut c: libc::c_int) -> libc::c_int {
    return (c == ' ' as i32 || c == '\t' as i32 || c == '\n' as i32 ||
                c == '\r' as i32 || c == '\u{c}' as i32 ||
                c == '\u{b}' as i32) as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn tolower(mut c: libc::c_int) -> libc::c_int {
    return if isupper(c) != 0 { (c - 'A' as i32) + 'a' as i32 } else { c };
}
#[no_mangle]
pub unsafe extern "C" fn toupper(mut c: libc::c_int) -> libc::c_int {
    return if islower(c) != 0 { (c - 'a' as i32) + 'A' as i32 } else { c };
}
#[no_mangle]
pub unsafe extern "C" fn strcasecmp(mut s1: *const libc::c_char,
                                    mut s2: *const libc::c_char)
 -> libc::c_int {
    return strncasecmp(s1, s2, -1i32 as size_t);
}
#[no_mangle]
pub unsafe extern "C" fn strncasecmp(mut s1: *const libc::c_char,
                                     mut s2: *const libc::c_char,
                                     mut n: size_t) -> libc::c_int {
    let mut ucs1: *const libc::c_uchar = s1 as *const libc::c_uchar;
    let mut ucs2: *const libc::c_uchar = s2 as *const libc::c_uchar;
    let mut d: libc::c_int = 0i32;
    while n != 0i32 as libc::c_ulong {
        let fresh0 = ucs1;
        ucs1 = ucs1.offset(1);
        let c1: libc::c_int = tolower(*fresh0 as libc::c_int);
        let fresh1 = ucs2;
        ucs2 = ucs2.offset(1);
        let c2: libc::c_int = tolower(*fresh1 as libc::c_int);
        d = c1 - c2;
        if d != 0i32 || c2 == '\u{0}' as i32 { break ; }
        n = n.wrapping_sub(1)
    }
    return d;
}
#[no_mangle]
pub unsafe extern "C" fn strcasestr(mut haystack: *const libc::c_char,
                                    mut needle: *const libc::c_char)
 -> *mut libc::c_char {
    let mut nLen: libc::c_int = strlen(needle) as libc::c_int;
    loop  {
        if strncasecmp(haystack, needle, nLen as libc::c_ulong) == 0 {
            return haystack as *mut libc::c_char
        }
        haystack = haystack.offset(1);
        if !(*haystack != 0) { break ; }
    }
    return 0 as *mut libc::c_char;
}
