use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
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
pub unsafe extern "C" fn sbufInit(mut sbuf: *mut sbuf_t,
                                  mut ptr: *mut uint8_t,
                                  mut end: *mut uint8_t) -> *mut sbuf_t {
    (*sbuf).ptr = ptr;
    (*sbuf).end = end;
    return sbuf;
}
#[no_mangle]
pub unsafe extern "C" fn sbufWriteU8(mut dst: *mut sbuf_t, mut val: uint8_t) {
    let fresh0 = (*dst).ptr;
    (*dst).ptr = (*dst).ptr.offset(1);
    *fresh0 = val;
}
#[no_mangle]
pub unsafe extern "C" fn sbufWriteU16(mut dst: *mut sbuf_t,
                                      mut val: uint16_t) {
    sbufWriteU8(dst, (val as libc::c_int >> 0i32) as uint8_t);
    sbufWriteU8(dst, (val as libc::c_int >> 8i32) as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn sbufWriteU32(mut dst: *mut sbuf_t,
                                      mut val: uint32_t) {
    sbufWriteU8(dst, (val >> 0i32) as uint8_t);
    sbufWriteU8(dst, (val >> 8i32) as uint8_t);
    sbufWriteU8(dst, (val >> 16i32) as uint8_t);
    sbufWriteU8(dst, (val >> 24i32) as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn sbufWriteU16BigEndian(mut dst: *mut sbuf_t,
                                               mut val: uint16_t) {
    sbufWriteU8(dst, (val as libc::c_int >> 8i32) as uint8_t);
    sbufWriteU8(dst, val as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn sbufWriteU32BigEndian(mut dst: *mut sbuf_t,
                                               mut val: uint32_t) {
    sbufWriteU8(dst, (val >> 24i32) as uint8_t);
    sbufWriteU8(dst, (val >> 16i32) as uint8_t);
    sbufWriteU8(dst, (val >> 8i32) as uint8_t);
    sbufWriteU8(dst, val as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn sbufFill(mut dst: *mut sbuf_t, mut data: uint8_t,
                                  mut len: libc::c_int) {
    memset((*dst).ptr as *mut libc::c_void, data as libc::c_int,
           len as libc::c_ulong);
    (*dst).ptr = (*dst).ptr.offset(len as isize);
}
#[no_mangle]
pub unsafe extern "C" fn sbufWriteData(mut dst: *mut sbuf_t,
                                       mut data: *const libc::c_void,
                                       mut len: libc::c_int) {
    memcpy((*dst).ptr as *mut libc::c_void, data, len as libc::c_ulong);
    (*dst).ptr = (*dst).ptr.offset(len as isize);
}
#[no_mangle]
pub unsafe extern "C" fn sbufWriteString(mut dst: *mut sbuf_t,
                                         mut string: *const libc::c_char) {
    sbufWriteData(dst, string as *const libc::c_void,
                  strlen(string) as libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn sbufWriteStringWithZeroTerminator(mut dst:
                                                               *mut sbuf_t,
                                                           mut string:
                                                               *const libc::c_char) {
    sbufWriteData(dst, string as *const libc::c_void,
                  strlen(string).wrapping_add(1i32 as libc::c_ulong) as
                      libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn sbufReadU8(mut src: *mut sbuf_t) -> uint8_t {
    let fresh1 = (*src).ptr;
    (*src).ptr = (*src).ptr.offset(1);
    return *fresh1;
}
#[no_mangle]
pub unsafe extern "C" fn sbufReadU16(mut src: *mut sbuf_t) -> uint16_t {
    let mut ret: uint16_t = 0;
    ret = sbufReadU8(src) as uint16_t;
    ret =
        (ret as libc::c_int | (sbufReadU8(src) as libc::c_int) << 8i32) as
            uint16_t;
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn sbufReadU32(mut src: *mut sbuf_t) -> uint32_t {
    let mut ret: uint32_t = 0;
    ret = sbufReadU8(src) as uint32_t;
    ret |= ((sbufReadU8(src) as libc::c_int) << 8i32) as libc::c_uint;
    ret |= ((sbufReadU8(src) as libc::c_int) << 16i32) as libc::c_uint;
    ret |= ((sbufReadU8(src) as libc::c_int) << 24i32) as libc::c_uint;
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn sbufReadData(mut src: *mut sbuf_t,
                                      mut data: *mut libc::c_void,
                                      mut len: libc::c_int) {
    memcpy(data, (*src).ptr as *const libc::c_void, len as libc::c_ulong);
}
// reader - return bytes remaining in buffer
// writer - return available space
#[no_mangle]
pub unsafe extern "C" fn sbufBytesRemaining(mut buf: *mut sbuf_t)
 -> libc::c_int {
    return (*buf).end.wrapping_offset_from((*buf).ptr) as libc::c_long as
               libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn sbufPtr(mut buf: *mut sbuf_t) -> *mut uint8_t {
    return (*buf).ptr;
}
#[no_mangle]
pub unsafe extern "C" fn sbufConstPtr(mut buf: *const sbuf_t)
 -> *const uint8_t {
    return (*buf).ptr;
}
// advance buffer pointer
// reader - skip data
// writer - commit written data
#[no_mangle]
pub unsafe extern "C" fn sbufAdvance(mut buf: *mut sbuf_t,
                                     mut size: libc::c_int) {
    (*buf).ptr = (*buf).ptr.offset(size as isize);
}
// modifies streambuf so that written data are prepared for reading
#[no_mangle]
pub unsafe extern "C" fn sbufSwitchToReader(mut buf: *mut sbuf_t,
                                            mut base: *mut uint8_t) {
    (*buf).end = (*buf).ptr;
    (*buf).ptr = base;
}
