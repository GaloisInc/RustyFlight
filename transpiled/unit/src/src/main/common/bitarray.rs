use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
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
pub unsafe extern "C" fn bitArrayGet(mut array: *const libc::c_void,
                                     mut bit: libc::c_uint) -> bool {
    return *(array as
                 *mut uint32_t).offset((bit as
                                            libc::c_ulong).wrapping_div((::core::mem::size_of::<uint32_t>()
                                                                             as
                                                                             libc::c_ulong).wrapping_mul(8
                                                                                                             as
                                                                                                             libc::c_int
                                                                                                             as
                                                                                                             libc::c_ulong))
                                           as isize) &
               ((1 as libc::c_int) <<
                    (bit as
                         libc::c_ulong).wrapping_rem((::core::mem::size_of::<uint32_t>()
                                                          as
                                                          libc::c_ulong).wrapping_mul(8
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_ulong)))
                   as libc::c_uint != 0;
}
#[no_mangle]
pub unsafe extern "C" fn bitArraySet(mut array: *mut libc::c_void,
                                     mut bit: libc::c_uint) {
    let ref mut fresh0 =
        *(array as
              *mut uint32_t).offset((bit as
                                         libc::c_ulong).wrapping_div((::core::mem::size_of::<uint32_t>()
                                                                          as
                                                                          libc::c_ulong).wrapping_mul(8
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_ulong))
                                        as isize);
    *fresh0 |=
        ((1 as libc::c_int) <<
             (bit as
                  libc::c_ulong).wrapping_rem((::core::mem::size_of::<uint32_t>()
                                                   as
                                                   libc::c_ulong).wrapping_mul(8
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_ulong)))
            as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn bitArrayClr(mut array: *mut libc::c_void,
                                     mut bit: libc::c_uint) {
    let ref mut fresh1 =
        *(array as
              *mut uint32_t).offset((bit as
                                         libc::c_ulong).wrapping_div((::core::mem::size_of::<uint32_t>()
                                                                          as
                                                                          libc::c_ulong).wrapping_mul(8
                                                                                                          as
                                                                                                          libc::c_int
                                                                                                          as
                                                                                                          libc::c_ulong))
                                        as isize);
    *fresh1 &=
        !((1 as libc::c_int) <<
              (bit as
                   libc::c_ulong).wrapping_rem((::core::mem::size_of::<uint32_t>()
                                                    as
                                                    libc::c_ulong).wrapping_mul(8
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_ulong)))
            as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn bitArrayXor(mut dest: *mut libc::c_void,
                                     mut size: size_t,
                                     mut op1: *mut libc::c_void,
                                     mut op2: *mut libc::c_void) {
    let mut i: size_t = 0 as libc::c_int as size_t;
    while i < size {
        *(dest as *mut uint8_t).offset(i as isize) =
            (*(op1 as *mut uint8_t).offset(i as isize) as libc::c_int ^
                 *(op2 as *mut uint8_t).offset(i as isize) as libc::c_int) as
                uint8_t;
        i = i.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn bitArrayCopy(mut array: *mut libc::c_void,
                                      mut from: libc::c_uint,
                                      mut to: libc::c_uint) {
    if bitArrayGet(array, from) {
        bitArraySet(array, to);
    } else { bitArrayClr(array, to); };
}
