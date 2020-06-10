use ::libc;
extern "C" {
    #[no_mangle]
    fn isspace(_: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn tolower(_: libc::c_int) -> libc::c_int;
}
/*  Copyright (C) 2002     Manuel Novoa III
 *  From my (incomplete) stdlib library for linux and (soon) elks.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Library General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, see
 *  <http://www.gnu.org/licenses/>.

 * Adapted for Betaflight by Petr Ledvina, 2018

 */
#[no_mangle]
pub unsafe extern "C" fn _strto_l(mut str: *const libc::c_char,
                                  mut endptr: *mut *mut libc::c_char,
                                  mut base: libc::c_int,
                                  mut sflag: libc::c_int) -> libc::c_ulong {
    let mut number: libc::c_ulong = 0;
    let mut cutoff: libc::c_ulong = 0;
    let mut fail_char: *const libc::c_char = 0 as *const libc::c_char;
    let mut negative: libc::c_uchar = 0;
    let mut digit: libc::c_uchar = 0;
    let mut cutoff_digit: libc::c_uchar = 0;
    fail_char = str;
    while isspace(*str as libc::c_int) != 0 {
        /* Skip leading whitespace. */
        str = str.offset(1)
    }
    /* Handle optional sign. */
    negative =
        0 as libc::c_int as
            libc::c_uchar; /* Fall through to increment str. */
    let mut current_block_8: u64;
    match *str as libc::c_int {
        45 => {
            negative = 1 as libc::c_int as libc::c_uchar;
            current_block_8 = 3310171364826293608;
        }
        43 => { current_block_8 = 3310171364826293608; }
        _ => { current_block_8 = 5399440093318478209; }
    }
    match current_block_8 {
        3310171364826293608 => { str = str.offset(1) }
        _ => { }
    }
    if base == 0 || base == 16 as libc::c_int || base == 2 as libc::c_int {
        /* Either dynamic (base = 0) or base with 0[xb] prefix. */
        if *str as libc::c_int == '0' as i32 {
            str = str.offset(1);
            fail_char = str;
            if (base == 0 || base == 16 as libc::c_int) &&
                   tolower(*str as libc::c_int) == 'x' as i32 {
                str = str.offset(1);
                base = 16 as libc::c_int
            } else if (base == 0 || base == 2 as libc::c_int) &&
                          tolower(*str as libc::c_int) == 'b' as i32 {
                str = str.offset(1);
                base = 2 as libc::c_int
            } else if base == 0 { base = 8 as libc::c_int }
        }
    }
    number = 0 as libc::c_int as libc::c_ulong;
    if ((base - 2 as libc::c_int) as libc::c_uint) <
           35 as libc::c_int as libc::c_uint {
        /* Legal base. */
        cutoff_digit =
            (9223372036854775807 as libc::c_long as
                 libc::c_ulong).wrapping_mul(2 as
                                                 libc::c_ulong).wrapping_add(1
                                                                                 as
                                                                                 libc::c_ulong).wrapping_rem(base
                                                                                                                 as
                                                                                                                 libc::c_ulong)
                as libc::c_uchar;
        cutoff =
            (9223372036854775807 as libc::c_long as
                 libc::c_ulong).wrapping_mul(2 as
                                                 libc::c_ulong).wrapping_add(1
                                                                                 as
                                                                                 libc::c_ulong).wrapping_div(base
                                                                                                                 as
                                                                                                                 libc::c_ulong);
        loop  {
            digit =
                if *str as libc::c_int - '0' as i32 <= 9 as libc::c_int {
                    (*str as libc::c_int) - '0' as i32
                } else if 0x20 as libc::c_int | *str as libc::c_int >=
                              'a' as i32 {
                    (0x20 as libc::c_int | *str as libc::c_int) -
                        ('a' as i32 - 10 as libc::c_int)
                } else { 40 as libc::c_int } as libc::c_uchar;
            if digit as libc::c_int >= base { break ; }
            str = str.offset(1);
            fail_char = str;
            if number > cutoff ||
                   number == cutoff &&
                       digit as libc::c_int > cutoff_digit as libc::c_int {
                number =
                    (9223372036854775807 as libc::c_long as
                         libc::c_ulong).wrapping_mul(2 as
                                                         libc::c_ulong).wrapping_add(1
                                                                                         as
                                                                                         libc::c_ulong);
                negative = (negative as libc::c_int & sflag) as libc::c_uchar
            } else {
                number =
                    number.wrapping_mul(base as
                                            libc::c_ulong).wrapping_add(digit
                                                                            as
                                                                            libc::c_ulong)
            }
        }
    }
    if !endptr.is_null() { *endptr = fail_char as *mut libc::c_char }
    let mut tmp: libc::c_ulong =
        if negative as libc::c_int != 0 {
            (-(1 as libc::c_int as libc::c_long +
                   (-(9223372036854775807 as libc::c_long) -
                        1 as libc::c_long)) as
                 libc::c_ulong).wrapping_add(1 as libc::c_int as
                                                 libc::c_ulong)
        } else { 9223372036854775807 as libc::c_long as libc::c_ulong };
    if sflag != 0 && number > tmp { number = tmp }
    return if negative as libc::c_int != 0 {
               -(number as libc::c_long) as libc::c_ulong
           } else { number };
}
#[no_mangle]
pub unsafe extern "C" fn strtol(mut str: *const libc::c_char,
                                mut endptr: *mut *mut libc::c_char,
                                mut base: libc::c_int) -> libc::c_long {
    return _strto_l(str, endptr, base, 1 as libc::c_int) as libc::c_long;
}
#[no_mangle]
pub unsafe extern "C" fn strtoul(mut str: *const libc::c_char,
                                 mut endptr: *mut *mut libc::c_char,
                                 mut base: libc::c_int) -> libc::c_ulong {
    return _strto_l(str, endptr, base, 0 as libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn atoi(mut str: *const libc::c_char) -> libc::c_int {
    return strtol(str, 0 as *mut *mut libc::c_char, 10 as libc::c_int) as
               libc::c_int;
}
