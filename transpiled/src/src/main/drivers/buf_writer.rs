use ::libc;
pub type __uint8_t = libc::c_uchar;
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
// Called to flush the buffer.
pub type bufWrite_t
    =
    Option<unsafe extern "C" fn(_: *mut libc::c_void, _: *mut libc::c_void,
                                _: libc::c_int) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bufWriter_s {
    pub writer: bufWrite_t,
    pub arg: *mut libc::c_void,
    pub capacity: uint8_t,
    pub at: uint8_t,
    pub data: [uint8_t; 0],
}
pub type bufWriter_t = bufWriter_s;
// Initialise a block of memory as a buffered writer.
//
// b should be sizeof(bufWriter_t) + the number of bytes to buffer.
// total_size should be the total size of b.
//
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
pub unsafe extern "C" fn bufWriterInit(mut b: *mut uint8_t,
                                       mut total_size: libc::c_int,
                                       mut writer: bufWrite_t,
                                       mut arg: *mut libc::c_void)
 -> *mut bufWriter_t {
    let mut buf: *mut bufWriter_t = b as *mut bufWriter_t;
    (*buf).writer = writer;
    (*buf).arg = arg;
    (*buf).at = 0 as libc::c_int as uint8_t;
    (*buf).capacity =
        (total_size as
             libc::c_ulong).wrapping_sub(::core::mem::size_of::<bufWriter_t>()
                                             as libc::c_ulong) as uint8_t;
    return buf;
}
#[no_mangle]
pub unsafe extern "C" fn bufWriterAppend(mut b: *mut bufWriter_t,
                                         mut ch: uint8_t) {
    let fresh0 = (*b).at;
    (*b).at = (*b).at.wrapping_add(1);
    *(*b).data.as_mut_ptr().offset(fresh0 as isize) = ch;
    if (*b).at as libc::c_int >= (*b).capacity as libc::c_int {
        bufWriterFlush(b);
    };
}
#[no_mangle]
pub unsafe extern "C" fn bufWriterFlush(mut b: *mut bufWriter_t) {
    if (*b).at as libc::c_int != 0 as libc::c_int {
        (*b).writer.expect("non-null function pointer")((*b).arg,
                                                        (*b).data.as_mut_ptr()
                                                            as
                                                            *mut libc::c_void,
                                                        (*b).at as
                                                            libc::c_int);
        (*b).at = 0 as libc::c_int as uint8_t
    };
}
