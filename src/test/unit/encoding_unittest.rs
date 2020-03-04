#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type int32_t = libc::c_int;
pub type uint32_t = libc::c_uint;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct zigzagEncodingExpectation_s {
    pub input: int32_t,
    pub expected: uint32_t,
}
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
pub type zigzagEncodingExpectation_t = zigzagEncodingExpectation_s;
#[no_mangle]
pub unsafe extern "C" fn TEST(mut EncodingTest: libc::c_int,
                              mut ZigzagEncodingTest: libc::c_int)
 -> libc::c_int {
    // given
    let mut expectations: [zigzagEncodingExpectation_t; 9] =
        [{
             let mut init =
                 zigzagEncodingExpectation_s{input: 0 as libc::c_int,
                                             expected:
                                                 0 as libc::c_int as
                                                     uint32_t,};
             init
         },
         {
             let mut init =
                 zigzagEncodingExpectation_s{input: -(1 as libc::c_int),
                                             expected:
                                                 1 as libc::c_int as
                                                     uint32_t,};
             init
         },
         {
             let mut init =
                 zigzagEncodingExpectation_s{input: 1 as libc::c_int,
                                             expected:
                                                 2 as libc::c_int as
                                                     uint32_t,};
             init
         },
         {
             let mut init =
                 zigzagEncodingExpectation_s{input: -(2 as libc::c_int),
                                             expected:
                                                 3 as libc::c_int as
                                                     uint32_t,};
             init
         },
         {
             let mut init =
                 zigzagEncodingExpectation_s{input: 2 as libc::c_int,
                                             expected:
                                                 4 as libc::c_int as
                                                     uint32_t,};
             init
         },
         {
             let mut init =
                 zigzagEncodingExpectation_s{input: 2147483646 as libc::c_int,
                                             expected:
                                                 4294967292 as libc::c_long as
                                                     uint32_t,};
             init
         },
         {
             let mut init =
                 zigzagEncodingExpectation_s{input:
                                                 -(2147483647 as libc::c_int),
                                             expected:
                                                 4294967293 as libc::c_long as
                                                     uint32_t,};
             init
         },
         {
             let mut init =
                 zigzagEncodingExpectation_s{input: 2147483647 as libc::c_int,
                                             expected:
                                                 4294967294 as libc::c_long as
                                                     uint32_t,};
             init
         },
         {
             let mut init =
                 zigzagEncodingExpectation_s{input:
                                                 -(2147483648 as libc::c_long)
                                                     as int32_t,
                                             expected:
                                                 4294967295 as libc::c_long as
                                                     uint32_t,};
             init
         }];
    let mut expectationCount: libc::c_int =
        (::std::mem::size_of::<[zigzagEncodingExpectation_t; 9]>() as
             libc::c_ulong).wrapping_div(::std::mem::size_of::<zigzagEncodingExpectation_t>()
                                             as libc::c_ulong) as libc::c_int;
    // expect
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < expectationCount {
        let mut expectation: *mut zigzagEncodingExpectation_t =
            &mut *expectations.as_mut_ptr().offset(i as isize) as
                *mut zigzagEncodingExpectation_t;
        EXPECT_EQ((*expectation).expected,
                  zigzagEncode((*expectation).input));
        i += 1
    }
    panic!("Reached end of non-void function without returning");
}
// STUBS
// expect
// Exponent should be in the top bits
// given
