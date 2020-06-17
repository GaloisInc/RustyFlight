use core;
use libc;
extern "C" {
    #[no_mangle]
    fn sbufWriteU8(dst: *mut sbuf_t, val: uint8_t);
    #[no_mangle]
    fn sbufWriteU16(dst: *mut sbuf_t, val: uint16_t);
    #[no_mangle]
    fn sbufPtr(buf: *mut sbuf_t) -> *mut uint8_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
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
// simple buffer-based serializer/deserializer without implicit size check
pub type sbuf_t = sbuf_s;
// data pointer must be first (sbuf_t* is equivalent to uint8_t **)
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
pub unsafe extern "C" fn crc16_ccitt(mut crc: uint16_t, mut a: libc::c_uchar)
 -> uint16_t {
    crc =
        (crc as libc::c_int ^ (a as uint16_t as libc::c_int) << 8i32) as
            uint16_t;
    let mut ii: libc::c_int = 0i32;
    while ii < 8i32 {
        if crc as libc::c_int & 0x8000i32 != 0 {
            crc = ((crc as libc::c_int) << 1i32 ^ 0x1021i32) as uint16_t
        } else { crc = ((crc as libc::c_int) << 1i32) as uint16_t }
        ii += 1
    }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn crc16_ccitt_update(mut crc: uint16_t,
                                            mut data: *const libc::c_void,
                                            mut length: uint32_t)
 -> uint16_t {
    let mut p: *const uint8_t = data as *const uint8_t;
    let mut pend: *const uint8_t = p.offset(length as isize);
    while p != pend { crc = crc16_ccitt(crc, *p); p = p.offset(1) }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn crc16_ccitt_sbuf_append(mut dst: *mut sbuf_t,
                                                 mut start: *mut uint8_t) {
    let mut crc: uint16_t = 0i32 as uint16_t;
    let end: *const uint8_t = sbufPtr(dst);
    let mut ptr: *const uint8_t = start;
    while ptr < end { crc = crc16_ccitt(crc, *ptr); ptr = ptr.offset(1) }
    sbufWriteU16(dst, crc);
}
#[no_mangle]
pub unsafe extern "C" fn crc8_dvb_s2(mut crc: uint8_t, mut a: libc::c_uchar)
 -> uint8_t {
    crc = (crc as libc::c_int ^ a as libc::c_int) as uint8_t;
    let mut ii: libc::c_int = 0i32;
    while ii < 8i32 {
        if crc as libc::c_int & 0x80i32 != 0 {
            crc = ((crc as libc::c_int) << 1i32 ^ 0xd5i32) as uint8_t
        } else { crc = ((crc as libc::c_int) << 1i32) as uint8_t }
        ii += 1
    }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn crc8_dvb_s2_update(mut crc: uint8_t,
                                            mut data: *const libc::c_void,
                                            mut length: uint32_t) -> uint8_t {
    let mut p: *const uint8_t = data as *const uint8_t;
    let mut pend: *const uint8_t = p.offset(length as isize);
    while p != pend { crc = crc8_dvb_s2(crc, *p); p = p.offset(1) }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn crc8_dvb_s2_sbuf_append(mut dst: *mut sbuf_t,
                                                 mut start: *mut uint8_t) {
    let mut crc: uint8_t = 0i32 as uint8_t;
    let end: *const uint8_t = (*dst).ptr;
    let mut ptr: *const uint8_t = start;
    while ptr < end { crc = crc8_dvb_s2(crc, *ptr); ptr = ptr.offset(1) }
    sbufWriteU8(dst, crc);
}
#[no_mangle]
pub unsafe extern "C" fn crc8_xor_update(mut crc: uint8_t,
                                         mut data: *const libc::c_void,
                                         mut length: uint32_t) -> uint8_t {
    let mut p: *const uint8_t = data as *const uint8_t;
    let mut pend: *const uint8_t = p.offset(length as isize);
    while p != pend {
        crc = (crc as libc::c_int ^ *p as libc::c_int) as uint8_t;
        p = p.offset(1)
    }
    return crc;
}
#[no_mangle]
pub unsafe extern "C" fn crc8_xor_sbuf_append(mut dst: *mut sbuf_t,
                                              mut start: *mut uint8_t) {
    let mut crc: uint8_t = 0i32 as uint8_t;
    let mut end: *const uint8_t = (*dst).ptr;
    let mut ptr: *mut uint8_t = start;
    while ptr < end as *mut uint8_t {
        crc = (crc as libc::c_int ^ *ptr as libc::c_int) as uint8_t;
        ptr = ptr.offset(1)
    }
    sbufWriteU8(dst, crc);
}
