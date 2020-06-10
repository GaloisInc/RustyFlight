use ::libc;
extern "C" {
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strncmp(_: *const libc::c_char, _: *const libc::c_char,
               _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn emfat_init(emfat_0: *mut emfat_t, label: *const libc::c_char,
                  entries_0: *mut emfat_entry_t) -> bool;
    #[no_mangle]
    fn flashfsGetSize() -> uint32_t;
    #[no_mangle]
    fn flashfsIdentifyStartOfFreeSpace() -> libc::c_int;
    #[no_mangle]
    fn flashfsReadAbs(offset: uint32_t, data: *mut uint8_t, len: libc::c_uint)
     -> libc::c_int;
}
pub type size_t = libc::c_ulong;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
/*
 * Derived from
 * https://github.com/fetisov/emfat/blob/master/project/emfat.c
 * version: 1.0 (4.01.2015)
 */
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 by Sergey Fetisov <fsenok@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct emfat_entry_s {
    pub name: *const libc::c_char,
    pub dir: bool,
    pub attr: uint8_t,
    pub level: libc::c_int,
    pub offset: uint32_t,
    pub curr_size: uint32_t,
    pub max_size: uint32_t,
    pub user_data: libc::c_long,
    pub cma_time: [uint32_t; 3],
    pub readcb: emfat_readcb_t,
    pub writecb: emfat_writecb_t,
    pub priv_0: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed {
    pub first_clust: uint32_t,
    pub last_clust: uint32_t,
    pub last_reserved: uint32_t,
    pub num_subentry: uint32_t,
    pub top: *mut emfat_entry_s,
    pub sub: *mut emfat_entry_s,
    pub next: *mut emfat_entry_s,
}
pub type emfat_writecb_t
    =
    Option<unsafe extern "C" fn(_: *const uint8_t, _: libc::c_int,
                                _: uint32_t, _: *mut emfat_entry_s) -> ()>;
pub type emfat_readcb_t
    =
    Option<unsafe extern "C" fn(_: *mut uint8_t, _: libc::c_int, _: uint32_t,
                                _: *mut emfat_entry_s) -> ()>;
pub type emfat_entry_t = emfat_entry_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct emfat_s {
    pub vol_size: uint64_t,
    pub disk_sectors: uint32_t,
    pub vol_label: *const libc::c_char,
    pub priv_0: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_0 {
    pub boot_lba: uint32_t,
    pub fsinfo_lba: uint32_t,
    pub fat1_lba: uint32_t,
    pub fat2_lba: uint32_t,
    pub root_lba: uint32_t,
    pub num_clust: uint32_t,
    pub free_clust: uint32_t,
    pub entries: *mut emfat_entry_t,
    pub last_entry: *mut emfat_entry_t,
    pub num_entries: libc::c_int,
}
pub type emfat_t = emfat_s;
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
 * Author: jflyper@github.com
 */
//#define USE_EMFAT_README
static mut autorun_file: [libc::c_char; 59] =
    [91, 97, 117, 116, 111, 114, 117, 110, 93, 13, 10, 105, 99, 111, 110, 61,
     105, 99, 111, 110, 46, 105, 99, 111, 13, 10, 108, 97, 98, 101, 108, 61,
     66, 101, 116, 97, 102, 108, 105, 103, 104, 116, 32, 79, 110, 98, 111, 97,
     114, 100, 32, 70, 108, 97, 115, 104, 13, 10, 0];
static mut icon_file: [libc::c_char; 2366] =
    [0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x1 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x1 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x18 as libc::c_int as libc::c_char, 0x18 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x1 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x20 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x28 as libc::c_int as libc::c_char, 0x9 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x16 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x28 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x18 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x30 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x1 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0x20 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0 as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xde as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xde as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xf4 as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xf4 as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd7 as libc::c_int as libc::c_char, 0xdd as libc::c_int as libc::c_char,
     0xe1 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xc7 as libc::c_int as libc::c_char, 0xc3 as libc::c_int as libc::c_char,
     0xc2 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xce as libc::c_int as libc::c_char, 0xce as libc::c_int as libc::c_char,
     0xce as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xe2 as libc::c_int as libc::c_char, 0xe4 as libc::c_int as libc::c_char,
     0xe5 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb0 as libc::c_int as libc::c_char, 0xad as libc::c_int as libc::c_char,
     0xad as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3a as libc::c_int as libc::c_char, 0x89 as libc::c_int as libc::c_char,
     0xa8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3 as libc::c_int as libc::c_char, 0xb5 as libc::c_int as libc::c_char,
     0xed as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x20 as libc::c_int as libc::c_char, 0x57 as libc::c_int as libc::c_char,
     0x6c as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x36 as libc::c_int as libc::c_char, 0x25 as libc::c_int as libc::c_char,
     0x1f as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x34 as libc::c_int as libc::c_char, 0x52 as libc::c_int as libc::c_char,
     0x5e as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x4f as libc::c_int as libc::c_char, 0x65 as libc::c_int as libc::c_char,
     0x6e as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x8a as libc::c_int as libc::c_char, 0x86 as libc::c_int as libc::c_char,
     0x84 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd8 as libc::c_int as libc::c_char, 0xd8 as libc::c_int as libc::c_char,
     0xd8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfa as libc::c_int as libc::c_char, 0xfa as libc::c_int as libc::c_char,
     0xfa as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb7 as libc::c_int as libc::c_char, 0xb2 as libc::c_int as libc::c_char,
     0xb1 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x2a as libc::c_int as libc::c_char, 0x1d as libc::c_int as libc::c_char,
     0x18 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x33 as libc::c_int as libc::c_char, 0x2a as libc::c_int as libc::c_char,
     0x26 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x25 as libc::c_int as libc::c_char, 0x68 as libc::c_int as libc::c_char,
     0x7f as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x16 as libc::c_int as libc::c_char, 0x90 as libc::c_int as libc::c_char,
     0xbe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3a as libc::c_int as libc::c_char, 0x38 as libc::c_int as libc::c_char,
     0x37 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x37 as libc::c_int as libc::c_char, 0x38 as libc::c_int as libc::c_char,
     0x38 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x31 as libc::c_int as libc::c_char, 0x35 as libc::c_int as libc::c_char,
     0x37 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x29 as libc::c_int as libc::c_char, 0x28 as libc::c_int as libc::c_char,
     0x27 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x34 as libc::c_int as libc::c_char, 0x34 as libc::c_int as libc::c_char,
     0x34 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x93 as libc::c_int as libc::c_char, 0x93 as libc::c_int as libc::c_char,
     0x93 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf3 as libc::c_int as libc::c_char, 0xf3 as libc::c_int as libc::c_char,
     0xf3 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xf8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x4f as libc::c_int as libc::c_char, 0xa0 as libc::c_int as libc::c_char,
     0xbf as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd as libc::c_int as libc::c_char, 0x8c as libc::c_int as libc::c_char,
     0xb8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x2c as libc::c_int as libc::c_char, 0x62 as libc::c_int as libc::c_char,
     0x76 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3e as libc::c_int as libc::c_char, 0x31 as libc::c_int as libc::c_char,
     0x2e as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x36 as libc::c_int as libc::c_char, 0x46 as libc::c_int as libc::c_char,
     0x4d as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x38 as libc::c_int as libc::c_char, 0x41 as libc::c_int as libc::c_char,
     0x44 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3b as libc::c_int as libc::c_char, 0x39 as libc::c_int as libc::c_char,
     0x38 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x37 as libc::c_int as libc::c_char, 0x36 as libc::c_int as libc::c_char,
     0x35 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x2c as libc::c_int as libc::c_char, 0x2c as libc::c_int as libc::c_char,
     0x2c as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x28 as libc::c_int as libc::c_char, 0x28 as libc::c_int as libc::c_char,
     0x28 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x83 as libc::c_int as libc::c_char, 0x83 as libc::c_int as libc::c_char,
     0x83 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x77 as libc::c_int as libc::c_char, 0x77 as libc::c_int as libc::c_char,
     0x77 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd5 as libc::c_int as libc::c_char, 0xd5 as libc::c_int as libc::c_char,
     0xd5 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x5f as libc::c_int as libc::c_char, 0x98 as libc::c_int as libc::c_char,
     0xad as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x1a as libc::c_int as libc::c_char, 0x5c as libc::c_int as libc::c_char,
     0x73 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x30 as libc::c_int as libc::c_char, 0x5a as libc::c_int as libc::c_char,
     0x6a as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x38 as libc::c_int as libc::c_char, 0x42 as libc::c_int as libc::c_char,
     0x46 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3c as libc::c_int as libc::c_char, 0x35 as libc::c_int as libc::c_char,
     0x33 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x39 as libc::c_int as libc::c_char, 0x38 as libc::c_int as libc::c_char,
     0x37 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x2b as libc::c_int as libc::c_char, 0x2b as libc::c_int as libc::c_char,
     0x2b as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3c as libc::c_int as libc::c_char, 0x3c as libc::c_int as libc::c_char,
     0x3c as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x83 as libc::c_int as libc::c_char, 0x83 as libc::c_int as libc::c_char,
     0x83 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xc2 as libc::c_int as libc::c_char, 0xc2 as libc::c_int as libc::c_char,
     0xc2 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd2 as libc::c_int as libc::c_char, 0xd2 as libc::c_int as libc::c_char,
     0xd2 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xa9 as libc::c_int as libc::c_char, 0xa9 as libc::c_int as libc::c_char,
     0xa9 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x86 as libc::c_int as libc::c_char, 0x86 as libc::c_int as libc::c_char,
     0x86 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xdf as libc::c_int as libc::c_char, 0xdf as libc::c_int as libc::c_char,
     0xdf as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xc3 as libc::c_int as libc::c_char, 0xba as libc::c_int as libc::c_char,
     0xb7 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x2c as libc::c_int as libc::c_char, 0x21 as libc::c_int as libc::c_char,
     0x1d as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x38 as libc::c_int as libc::c_char, 0x30 as libc::c_int as libc::c_char,
     0x2d as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3c as libc::c_int as libc::c_char, 0x3a as libc::c_int as libc::c_char,
     0x39 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x34 as libc::c_int as libc::c_char, 0x35 as libc::c_int as libc::c_char,
     0x35 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x2e as libc::c_int as libc::c_char, 0x2e as libc::c_int as libc::c_char,
     0x2e as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x78 as libc::c_int as libc::c_char, 0x78 as libc::c_int as libc::c_char,
     0x78 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xdf as libc::c_int as libc::c_char, 0xdf as libc::c_int as libc::c_char,
     0xdf as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xeb as libc::c_int as libc::c_char, 0xeb as libc::c_int as libc::c_char,
     0xeb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfa as libc::c_int as libc::c_char, 0xfa as libc::c_int as libc::c_char,
     0xfa as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb3 as libc::c_int as libc::c_char, 0xb0 as libc::c_int as libc::c_char,
     0xaf as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3d as libc::c_int as libc::c_char, 0x31 as libc::c_int as libc::c_char,
     0x2c as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x33 as libc::c_int as libc::c_char, 0x30 as libc::c_int as libc::c_char,
     0x2f as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x49 as libc::c_int as libc::c_char, 0x4a as libc::c_int as libc::c_char,
     0x4a as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xcb as libc::c_int as libc::c_char, 0xcb as libc::c_int as libc::c_char,
     0xcb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd3 as libc::c_int as libc::c_char, 0xe2 as libc::c_int as libc::c_char,
     0xe8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x1c as libc::c_int as libc::c_char, 0x85 as libc::c_int as libc::c_char,
     0xae as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x2f as libc::c_int as libc::c_char, 0x44 as libc::c_int as libc::c_char,
     0x4c as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x41 as libc::c_int as libc::c_char, 0x3c as libc::c_int as libc::c_char,
     0x3a as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x9a as libc::c_int as libc::c_char, 0x9a as libc::c_int as libc::c_char,
     0x9a as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xf9 as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xf9 as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf8 as libc::c_int as libc::c_char, 0xf8 as libc::c_int as libc::c_char,
     0xf8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf8 as libc::c_int as libc::c_char, 0xf8 as libc::c_int as libc::c_char,
     0xf8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf5 as libc::c_int as libc::c_char, 0xf5 as libc::c_int as libc::c_char,
     0xf5 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xf9 as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x8a as libc::c_int as libc::c_char, 0xd0 as libc::c_int as libc::c_char,
     0xea as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0xb0 as libc::c_int as libc::c_char,
     0xf7 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x36 as libc::c_int as libc::c_char, 0x49 as libc::c_int as libc::c_char,
     0x51 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x39 as libc::c_int as libc::c_char, 0x33 as libc::c_int as libc::c_char,
     0x31 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x1f as libc::c_int as libc::c_char, 0x1f as libc::c_int as libc::c_char,
     0x1f as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x98 as libc::c_int as libc::c_char, 0x98 as libc::c_int as libc::c_char,
     0x98 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xe2 as libc::c_int as libc::c_char, 0xe2 as libc::c_int as libc::c_char,
     0xe2 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x9b as libc::c_int as libc::c_char, 0x9b as libc::c_int as libc::c_char,
     0x9b as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x71 as libc::c_int as libc::c_char, 0x71 as libc::c_int as libc::c_char,
     0x71 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x72 as libc::c_int as libc::c_char, 0x72 as libc::c_int as libc::c_char,
     0x72 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x63 as libc::c_int as libc::c_char, 0x63 as libc::c_int as libc::c_char,
     0x63 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xef as libc::c_int as libc::c_char, 0xef as libc::c_int as libc::c_char,
     0xef as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf4 as libc::c_int as libc::c_char, 0xf4 as libc::c_int as libc::c_char,
     0xf4 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xae as libc::c_int as libc::c_char, 0xae as libc::c_int as libc::c_char,
     0xae as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x8c as libc::c_int as libc::c_char, 0x8c as libc::c_int as libc::c_char,
     0x8c as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x86 as libc::c_int as libc::c_char, 0x86 as libc::c_int as libc::c_char,
     0x86 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x84 as libc::c_int as libc::c_char, 0x84 as libc::c_int as libc::c_char,
     0x84 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x86 as libc::c_int as libc::c_char, 0x86 as libc::c_int as libc::c_char,
     0x86 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x91 as libc::c_int as libc::c_char, 0x8a as libc::c_int as libc::c_char,
     0x87 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x48 as libc::c_int as libc::c_char, 0x7a as libc::c_int as libc::c_char,
     0x8d as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0 as libc::c_int as libc::c_char, 0x9c as libc::c_int as libc::c_char,
     0xd7 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3a as libc::c_int as libc::c_char, 0x3e as libc::c_int as libc::c_char,
     0x3f as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x39 as libc::c_int as libc::c_char, 0x36 as libc::c_int as libc::c_char,
     0x35 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x35 as libc::c_int as libc::c_char, 0x35 as libc::c_int as libc::c_char,
     0x35 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x42 as libc::c_int as libc::c_char, 0x42 as libc::c_int as libc::c_char,
     0x42 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x3b as libc::c_int as libc::c_char, 0x3b as libc::c_int as libc::c_char,
     0x3b as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x31 as libc::c_int as libc::c_char, 0x31 as libc::c_int as libc::c_char,
     0x31 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x92 as libc::c_int as libc::c_char, 0x92 as libc::c_int as libc::c_char,
     0x92 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x90 as libc::c_int as libc::c_char, 0x90 as libc::c_int as libc::c_char,
     0x90 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x7b as libc::c_int as libc::c_char, 0x7b as libc::c_int as libc::c_char,
     0x7b as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xde as libc::c_int as libc::c_char, 0xde as libc::c_int as libc::c_char,
     0xde as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x80 as libc::c_int as libc::c_char, 0x80 as libc::c_int as libc::c_char,
     0x80 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb1 as libc::c_int as libc::c_char, 0xb1 as libc::c_int as libc::c_char,
     0xb1 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xe6 as libc::c_int as libc::c_char, 0xe6 as libc::c_int as libc::c_char,
     0xe6 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xf9 as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xea as libc::c_int as libc::c_char, 0xea as libc::c_int as libc::c_char,
     0xea as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x81 as libc::c_int as libc::c_char, 0x80 as libc::c_int as libc::c_char,
     0x80 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x49 as libc::c_int as libc::c_char, 0x42 as libc::c_int as libc::c_char,
     0x40 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x55 as libc::c_int as libc::c_char, 0x8e as libc::c_int as libc::c_char,
     0xa3 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x1f as libc::c_int as libc::c_char, 0x5b as libc::c_int as libc::c_char,
     0x72 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x35 as libc::c_int as libc::c_char, 0x3a as libc::c_int as libc::c_char,
     0x3c as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x43 as libc::c_int as libc::c_char, 0x41 as libc::c_int as libc::c_char,
     0x41 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x35 as libc::c_int as libc::c_char, 0x35 as libc::c_int as libc::c_char,
     0x35 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x2a as libc::c_int as libc::c_char, 0x2a as libc::c_int as libc::c_char,
     0x2a as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xbe as libc::c_int as libc::c_char, 0xbe as libc::c_int as libc::c_char,
     0xbe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xe8 as libc::c_int as libc::c_char, 0xe8 as libc::c_int as libc::c_char,
     0xe8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x6f as libc::c_int as libc::c_char, 0x6f as libc::c_int as libc::c_char,
     0x6f as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xe7 as libc::c_int as libc::c_char, 0xe7 as libc::c_int as libc::c_char,
     0xe7 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf5 as libc::c_int as libc::c_char, 0xf5 as libc::c_int as libc::c_char,
     0xf5 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x7b as libc::c_int as libc::c_char, 0x7b as libc::c_int as libc::c_char,
     0x7b as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xe0 as libc::c_int as libc::c_char, 0xe0 as libc::c_int as libc::c_char,
     0xe0 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf8 as libc::c_int as libc::c_char, 0xf8 as libc::c_int as libc::c_char,
     0xf8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb8 as libc::c_int as libc::c_char, 0xb8 as libc::c_int as libc::c_char,
     0xb8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x7d as libc::c_int as libc::c_char, 0x7d as libc::c_int as libc::c_char,
     0x7d as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xa3 as libc::c_int as libc::c_char, 0xa3 as libc::c_int as libc::c_char,
     0xa3 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd2 as libc::c_int as libc::c_char, 0xd2 as libc::c_int as libc::c_char,
     0xd2 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x87 as libc::c_int as libc::c_char, 0x7c as libc::c_int as libc::c_char,
     0x77 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x5a as libc::c_int as libc::c_char, 0x57 as libc::c_int as libc::c_char,
     0x55 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xca as libc::c_int as libc::c_char, 0xcb as libc::c_int as libc::c_char,
     0xcb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x82 as libc::c_int as libc::c_char, 0x82 as libc::c_int as libc::c_char,
     0x82 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x23 as libc::c_int as libc::c_char, 0x23 as libc::c_int as libc::c_char,
     0x23 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x38 as libc::c_int as libc::c_char, 0x38 as libc::c_int as libc::c_char,
     0x38 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x43 as libc::c_int as libc::c_char, 0x43 as libc::c_int as libc::c_char,
     0x43 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x5d as libc::c_int as libc::c_char, 0x5d as libc::c_int as libc::c_char,
     0x5d as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd9 as libc::c_int as libc::c_char, 0xd9 as libc::c_int as libc::c_char,
     0xd9 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xa1 as libc::c_int as libc::c_char, 0xa1 as libc::c_int as libc::c_char,
     0xa1 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb6 as libc::c_int as libc::c_char, 0xb6 as libc::c_int as libc::c_char,
     0xb6 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xdb as libc::c_int as libc::c_char, 0xdb as libc::c_int as libc::c_char,
     0xdb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb1 as libc::c_int as libc::c_char, 0xb1 as libc::c_int as libc::c_char,
     0xb1 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xc7 as libc::c_int as libc::c_char, 0xc7 as libc::c_int as libc::c_char,
     0xc7 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xea as libc::c_int as libc::c_char, 0xea as libc::c_int as libc::c_char,
     0xea as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xa6 as libc::c_int as libc::c_char, 0xa6 as libc::c_int as libc::c_char,
     0xa6 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x60 as libc::c_int as libc::c_char, 0x60 as libc::c_int as libc::c_char,
     0x60 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x9f as libc::c_int as libc::c_char, 0x9f as libc::c_int as libc::c_char,
     0x9f as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd1 as libc::c_int as libc::c_char, 0xd1 as libc::c_int as libc::c_char,
     0xd1 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x4e as libc::c_int as libc::c_char, 0x4e as libc::c_int as libc::c_char,
     0x4e as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x29 as libc::c_int as libc::c_char, 0x29 as libc::c_int as libc::c_char,
     0x29 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x73 as libc::c_int as libc::c_char, 0x73 as libc::c_int as libc::c_char,
     0x73 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xeb as libc::c_int as libc::c_char, 0xeb as libc::c_int as libc::c_char,
     0xeb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xec as libc::c_int as libc::c_char, 0xec as libc::c_int as libc::c_char,
     0xec as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x90 as libc::c_int as libc::c_char, 0x90 as libc::c_int as libc::c_char,
     0x90 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf3 as libc::c_int as libc::c_char, 0xf3 as libc::c_int as libc::c_char,
     0xf3 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf0 as libc::c_int as libc::c_char, 0xf0 as libc::c_int as libc::c_char,
     0xf0 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb7 as libc::c_int as libc::c_char, 0xb7 as libc::c_int as libc::c_char,
     0xb7 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x61 as libc::c_int as libc::c_char, 0x61 as libc::c_int as libc::c_char,
     0x61 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x89 as libc::c_int as libc::c_char, 0x89 as libc::c_int as libc::c_char,
     0x89 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xeb as libc::c_int as libc::c_char, 0xeb as libc::c_int as libc::c_char,
     0xeb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xdc as libc::c_int as libc::c_char, 0xdc as libc::c_int as libc::c_char,
     0xdc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x99 as libc::c_int as libc::c_char, 0x99 as libc::c_int as libc::c_char,
     0x99 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xbc as libc::c_int as libc::c_char, 0xbc as libc::c_int as libc::c_char,
     0xbc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0x98 as libc::c_int as libc::c_char, 0x98 as libc::c_int as libc::c_char,
     0x98 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x9c as libc::c_int as libc::c_char, 0x9c as libc::c_int as libc::c_char,
     0x9c as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xf9 as libc::c_int as libc::c_char,
     0xf9 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xc5 as libc::c_int as libc::c_char, 0xc5 as libc::c_int as libc::c_char,
     0xc5 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x66 as libc::c_int as libc::c_char, 0x66 as libc::c_int as libc::c_char,
     0x66 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x76 as libc::c_int as libc::c_char, 0x76 as libc::c_int as libc::c_char,
     0x76 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xdb as libc::c_int as libc::c_char, 0xdb as libc::c_int as libc::c_char,
     0xdb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xdd as libc::c_int as libc::c_char, 0xdd as libc::c_int as libc::c_char,
     0xdd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xcd as libc::c_int as libc::c_char, 0xcd as libc::c_int as libc::c_char,
     0xcd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xf7 as libc::c_int as libc::c_char, 0xf7 as libc::c_int as libc::c_char,
     0xf7 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xda as libc::c_int as libc::c_char, 0xda as libc::c_int as libc::c_char,
     0xda as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0x48 as libc::c_int as libc::c_char, 0x48 as libc::c_int as libc::c_char,
     0x48 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x75 as libc::c_int as libc::c_char, 0x75 as libc::c_int as libc::c_char,
     0x75 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xd5 as libc::c_int as libc::c_char, 0xd5 as libc::c_int as libc::c_char,
     0xd5 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x72 as libc::c_int as libc::c_char, 0x72 as libc::c_int as libc::c_char,
     0x72 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x6a as libc::c_int as libc::c_char, 0x6a as libc::c_int as libc::c_char,
     0x6a as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xc8 as libc::c_int as libc::c_char, 0xc8 as libc::c_int as libc::c_char,
     0xc8 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xeb as libc::c_int as libc::c_char, 0xeb as libc::c_int as libc::c_char,
     0xeb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xaf as libc::c_int as libc::c_char, 0xaf as libc::c_int as libc::c_char,
     0xaf as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb0 as libc::c_int as libc::c_char, 0xb0 as libc::c_int as libc::c_char,
     0xb0 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x8b as libc::c_int as libc::c_char, 0x8b as libc::c_int as libc::c_char,
     0x8b as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x60 as libc::c_int as libc::c_char, 0x60 as libc::c_int as libc::c_char,
     0x60 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xb3 as libc::c_int as libc::c_char, 0xb3 as libc::c_int as libc::c_char,
     0xb3 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0x66 as libc::c_int as libc::c_char, 0x66 as libc::c_int as libc::c_char,
     0x66 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0x54 as libc::c_int as libc::c_char, 0x54 as libc::c_int as libc::c_char,
     0x54 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xa0 as libc::c_int as libc::c_char, 0xa0 as libc::c_int as libc::c_char,
     0xa0 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfa as libc::c_int as libc::c_char, 0xfa as libc::c_int as libc::c_char,
     0xfa as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xce as libc::c_int as libc::c_char, 0xce as libc::c_int as libc::c_char,
     0xce as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0x98 as libc::c_int as libc::c_char, 0x98 as libc::c_int as libc::c_char,
     0x98 as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xef as libc::c_int as libc::c_char, 0xef as libc::c_int as libc::c_char,
     0xef as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xfe as libc::c_int as libc::c_char,
     0xfe as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xf6 as libc::c_int as libc::c_char, 0xf6 as libc::c_int as libc::c_char,
     0xf6 as libc::c_int as libc::c_char, 0xf4 as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xff as libc::c_int as libc::c_char, 0xff as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xf4 as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xde as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xfc as libc::c_int as libc::c_char,
     0xfc as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xfb as libc::c_int as libc::c_char,
     0xfb as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xee as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char, 0xfd as libc::c_int as libc::c_char,
     0xfd as libc::c_int as libc::c_char,
     0xde as libc::c_int as libc::c_char];
unsafe extern "C" fn memory_read_proc(mut dest: *mut uint8_t,
                                      mut size: libc::c_int,
                                      mut offset: uint32_t,
                                      mut entry: *mut emfat_entry_t) {
    let mut len: libc::c_int = 0;
    if offset > (*entry).curr_size { return }
    if offset.wrapping_add(size as libc::c_uint) > (*entry).curr_size {
        len = (*entry).curr_size.wrapping_sub(offset) as libc::c_int
    } else { len = size }
    memcpy(dest as *mut libc::c_void,
           &mut *((*entry).user_data as
                      *mut libc::c_char).offset(offset as isize) as
               *mut libc::c_char as *const libc::c_void,
           len as libc::c_ulong);
}
unsafe extern "C" fn bblog_read_proc(mut dest: *mut uint8_t,
                                     mut size: libc::c_int,
                                     mut offset: uint32_t,
                                     mut entry: *mut emfat_entry_t) {
    flashfsReadAbs(offset, dest, size as libc::c_uint);
}
// Initialized in run_static_initializers
static mut entriesPredefined: [emfat_entry_t; 4] =
    [emfat_entry_t{name: 0 as *const libc::c_char,
                   dir: false,
                   attr: 0,
                   level: 0,
                   offset: 0,
                   curr_size: 0,
                   max_size: 0,
                   user_data: 0,
                   cma_time: [0; 3],
                   readcb: None,
                   writecb: None,
                   priv_0:
                       C2RustUnnamed{first_clust: 0,
                                     last_clust: 0,
                                     last_reserved: 0,
                                     num_subentry: 0,
                                     top: 0 as *mut emfat_entry_s,
                                     sub: 0 as *mut emfat_entry_s,
                                     next: 0 as *mut emfat_entry_s,},}; 4];
static mut entries: [emfat_entry_t; 103] =
    [emfat_entry_t{name: 0 as *const libc::c_char,
                   dir: false,
                   attr: 0,
                   level: 0,
                   offset: 0,
                   curr_size: 0,
                   max_size: 0,
                   user_data: 0,
                   cma_time: [0; 3],
                   readcb: None,
                   writecb: None,
                   priv_0:
                       C2RustUnnamed{first_clust: 0,
                                     last_clust: 0,
                                     last_reserved: 0,
                                     num_subentry: 0,
                                     top: 0 as *mut emfat_entry_s,
                                     sub: 0 as *mut emfat_entry_s,
                                     next: 0 as *mut emfat_entry_s,},}; 103];
static mut logNames: [[libc::c_char; 11]; 100] = [[0; 11]; 100];
#[no_mangle]
pub static mut emfat: emfat_t =
    emfat_t{vol_size: 0,
            disk_sectors: 0,
            vol_label: 0 as *const libc::c_char,
            priv_0:
                C2RustUnnamed_0{boot_lba: 0,
                                fsinfo_lba: 0,
                                fat1_lba: 0,
                                fat2_lba: 0,
                                root_lba: 0,
                                num_clust: 0,
                                free_clust: 0,
                                entries:
                                    0 as *const emfat_entry_t as
                                        *mut emfat_entry_t,
                                last_entry:
                                    0 as *const emfat_entry_t as
                                        *mut emfat_entry_t,
                                num_entries: 0,},};
unsafe extern "C" fn emfat_add_log(mut entry: *mut emfat_entry_t,
                                   mut number: libc::c_int,
                                   mut offset: uint32_t, mut size: uint32_t) {
    tfp_sprintf(logNames[number as usize].as_mut_ptr(),
                b"BTFL_%03d.BBL\x00" as *const u8 as *const libc::c_char,
                number);
    (*entry).name = logNames[number as usize].as_mut_ptr();
    (*entry).level = 1 as libc::c_int;
    (*entry).offset = offset;
    (*entry).curr_size = size;
    (*entry).max_size = (*entry).curr_size;
    (*entry).cma_time[0 as libc::c_int as usize] =
        (((2018 as libc::c_int - 1980 as libc::c_int) << 9 as libc::c_int |
              (1 as libc::c_int) << 5 as libc::c_int | 1 as libc::c_int) <<
             16 as libc::c_int |
             ((13 as libc::c_int) << 11 as libc::c_int |
                  (0 as libc::c_int) << 5 as libc::c_int |
                  0 as libc::c_int >> 1 as libc::c_int)) as uint32_t;
    (*entry).cma_time[1 as libc::c_int as usize] =
        (((2018 as libc::c_int - 1980 as libc::c_int) << 9 as libc::c_int |
              (1 as libc::c_int) << 5 as libc::c_int | 1 as libc::c_int) <<
             16 as libc::c_int |
             ((13 as libc::c_int) << 11 as libc::c_int |
                  (0 as libc::c_int) << 5 as libc::c_int |
                  0 as libc::c_int >> 1 as libc::c_int)) as uint32_t;
    (*entry).cma_time[2 as libc::c_int as usize] =
        (((2018 as libc::c_int - 1980 as libc::c_int) << 9 as libc::c_int |
              (1 as libc::c_int) << 5 as libc::c_int | 1 as libc::c_int) <<
             16 as libc::c_int |
             ((13 as libc::c_int) << 11 as libc::c_int |
                  (0 as libc::c_int) << 5 as libc::c_int |
                  0 as libc::c_int >> 1 as libc::c_int)) as uint32_t;
    (*entry).readcb =
        Some(bblog_read_proc as
                 unsafe extern "C" fn(_: *mut uint8_t, _: libc::c_int,
                                      _: uint32_t, _: *mut emfat_entry_t)
                     -> ());
}
unsafe extern "C" fn emfat_find_log(mut entry: *mut emfat_entry_t,
                                    mut maxCount: libc::c_int) {
    let mut limit: uint32_t = flashfsIdentifyStartOfFreeSpace() as uint32_t;
    let mut lastOffset: uint32_t = 0 as libc::c_int as uint32_t;
    let mut currOffset: uint32_t = 0 as libc::c_int as uint32_t;
    let mut fileNumber: libc::c_int = 0 as libc::c_int;
    let mut buffer: [uint8_t; 18] = [0; 18];
    while currOffset < limit {
        // XXX 2048 = FREE_BLOCK_SIZE in io/flashfs.c
        flashfsReadAbs(currOffset, buffer.as_mut_ptr(),
                       18 as libc::c_int as libc::c_uint);
        if !(strncmp(buffer.as_mut_ptr() as *mut libc::c_char,
                     b"H Product:Blackbox\x00" as *const u8 as
                         *const libc::c_char,
                     18 as libc::c_int as libc::c_ulong) != 0) {
            if lastOffset != currOffset {
                emfat_add_log(entry, fileNumber, lastOffset,
                              currOffset.wrapping_sub(lastOffset));
                fileNumber += 1;
                if fileNumber == maxCount { break ; }
                entry = entry.offset(1)
            }
            lastOffset = currOffset
        }
        currOffset =
            (currOffset as
                 libc::c_uint).wrapping_add(2048 as libc::c_int as
                                                libc::c_uint) as uint32_t as
                uint32_t
    }
    if fileNumber != maxCount && lastOffset != currOffset {
        emfat_add_log(entry, fileNumber, lastOffset,
                      currOffset.wrapping_sub(lastOffset));
    };
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
#[no_mangle]
pub unsafe extern "C" fn emfat_init_files() {
    memset(entries.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[emfat_entry_t; 103]>() as libc::c_ulong);
    let mut i: size_t = 0 as libc::c_int as size_t;
    while i <
              (::core::mem::size_of::<[emfat_entry_t; 4]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<emfat_entry_t>()
                                                   as libc::c_ulong) {
        entries[i as usize] = entriesPredefined[i as usize];
        i = i.wrapping_add(1)
    }
    // Singleton
    let mut entry: *mut emfat_entry_t =
        &mut *entries.as_mut_ptr().offset((1 as libc::c_int + 1 as libc::c_int
                                               + 1 as libc::c_int +
                                               0 as libc::c_int) as isize) as
            *mut emfat_entry_t;
    (*entry).curr_size = flashfsIdentifyStartOfFreeSpace() as uint32_t;
    (*entry).max_size = flashfsGetSize();
    // Detect and list individual power cycle sessions
    emfat_find_log(&mut *entries.as_mut_ptr().offset((1 as libc::c_int +
                                                          1 as libc::c_int +
                                                          1 as libc::c_int +
                                                          0 as libc::c_int +
                                                          1 as libc::c_int) as
                                                         isize),
                   1 as libc::c_int + 1 as libc::c_int + 1 as libc::c_int +
                       0 as libc::c_int + 100 as libc::c_int -
                       (1 as libc::c_int + 1 as libc::c_int + 1 as libc::c_int
                            + 0 as libc::c_int + 1 as libc::c_int));
    emfat_init(&mut emfat, b"emfat\x00" as *const u8 as *const libc::c_char,
               entries.as_mut_ptr());
}
unsafe extern "C" fn run_static_initializers() {
    entriesPredefined =
        [{
             let mut init =
                 emfat_entry_s{name:
                                   b"\x00" as *const u8 as
                                       *const libc::c_char,
                               dir: 1 as libc::c_int != 0,
                               attr: 0 as libc::c_int as uint8_t,
                               level: 0 as libc::c_int,
                               offset: 0 as libc::c_int as uint32_t,
                               curr_size: 0 as libc::c_int as uint32_t,
                               max_size: 0 as libc::c_int as uint32_t,
                               user_data: 0 as libc::c_int as libc::c_long,
                               cma_time:
                                   [(((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t,
                                    (((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t,
                                    (((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t],
                               readcb: None,
                               writecb: None,
                               priv_0:
                                   {
                                       let mut init =
                                           C2RustUnnamed{first_clust:
                                                             0 as libc::c_int
                                                                 as uint32_t,
                                                         last_clust: 0,
                                                         last_reserved: 0,
                                                         num_subentry: 0,
                                                         top:
                                                             0 as
                                                                 *mut emfat_entry_s,
                                                         sub:
                                                             0 as
                                                                 *mut emfat_entry_s,
                                                         next:
                                                             0 as
                                                                 *mut emfat_entry_s,};
                                       init
                                   },};
             init
         },
         {
             let mut init =
                 emfat_entry_s{name:
                                   b"autorun.inf\x00" as *const u8 as
                                       *const libc::c_char,
                               dir: 0 as libc::c_int != 0,
                               attr: 0x2 as libc::c_int as uint8_t,
                               level: 1 as libc::c_int,
                               offset: 0 as libc::c_int as uint32_t,
                               curr_size:
                                   (::core::mem::size_of::<[libc::c_char; 59]>()
                                        as
                                        libc::c_ulong).wrapping_sub(1 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_ulong)
                                       as uint32_t,
                               max_size:
                                   (::core::mem::size_of::<[libc::c_char; 59]>()
                                        as
                                        libc::c_ulong).wrapping_sub(1 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_ulong)
                                       as uint32_t,
                               user_data:
                                   autorun_file.as_ptr() as libc::c_long,
                               cma_time:
                                   [(((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t,
                                    (((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t,
                                    (((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t],
                               readcb:
                                   Some(memory_read_proc as
                                            unsafe extern "C" fn(_:
                                                                     *mut uint8_t,
                                                                 _:
                                                                     libc::c_int,
                                                                 _: uint32_t,
                                                                 _:
                                                                     *mut emfat_entry_t)
                                                -> ()),
                               writecb: None,
                               priv_0:
                                   {
                                       let mut init =
                                           C2RustUnnamed{first_clust:
                                                             0 as libc::c_int
                                                                 as uint32_t,
                                                         last_clust: 0,
                                                         last_reserved: 0,
                                                         num_subentry: 0,
                                                         top:
                                                             0 as
                                                                 *mut emfat_entry_s,
                                                         sub:
                                                             0 as
                                                                 *mut emfat_entry_s,
                                                         next:
                                                             0 as
                                                                 *mut emfat_entry_s,};
                                       init
                                   },};
             init
         },
         {
             let mut init =
                 emfat_entry_s{name:
                                   b"icon.ico\x00" as *const u8 as
                                       *const libc::c_char,
                               dir: 0 as libc::c_int != 0,
                               attr: 0x2 as libc::c_int as uint8_t,
                               level: 1 as libc::c_int,
                               offset: 0 as libc::c_int as uint32_t,
                               curr_size:
                                   ::core::mem::size_of::<[libc::c_char; 2366]>()
                                       as libc::c_ulong as uint32_t,
                               max_size:
                                   ::core::mem::size_of::<[libc::c_char; 2366]>()
                                       as libc::c_ulong as uint32_t,
                               user_data: icon_file.as_ptr() as libc::c_long,
                               cma_time:
                                   [(((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t,
                                    (((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t,
                                    (((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t],
                               readcb:
                                   Some(memory_read_proc as
                                            unsafe extern "C" fn(_:
                                                                     *mut uint8_t,
                                                                 _:
                                                                     libc::c_int,
                                                                 _: uint32_t,
                                                                 _:
                                                                     *mut emfat_entry_t)
                                                -> ()),
                               writecb: None,
                               priv_0:
                                   {
                                       let mut init =
                                           C2RustUnnamed{first_clust:
                                                             0 as libc::c_int
                                                                 as uint32_t,
                                                         last_clust: 0,
                                                         last_reserved: 0,
                                                         num_subentry: 0,
                                                         top:
                                                             0 as
                                                                 *mut emfat_entry_s,
                                                         sub:
                                                             0 as
                                                                 *mut emfat_entry_s,
                                                         next:
                                                             0 as
                                                                 *mut emfat_entry_s,};
                                       init
                                   },};
             init
         },
         {
             let mut init =
                 emfat_entry_s{name:
                                   b"BTFL_ALL.BBL\x00" as *const u8 as
                                       *const libc::c_char,
                               dir: 0 as libc::c_int != 0,
                               attr: 0 as libc::c_int as uint8_t,
                               level: 1 as libc::c_int,
                               offset: 0 as libc::c_int as uint32_t,
                               curr_size: 0 as libc::c_int as uint32_t,
                               max_size: 0 as libc::c_int as uint32_t,
                               user_data: 0 as libc::c_int as libc::c_long,
                               cma_time:
                                   [(((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t,
                                    (((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t,
                                    (((2018 as libc::c_int -
                                           1980 as libc::c_int) <<
                                          9 as libc::c_int |
                                          (1 as libc::c_int) <<
                                              5 as libc::c_int |
                                          1 as libc::c_int) <<
                                         16 as libc::c_int |
                                         ((13 as libc::c_int) <<
                                              11 as libc::c_int |
                                              (0 as libc::c_int) <<
                                                  5 as libc::c_int |
                                              0 as libc::c_int >>
                                                  1 as libc::c_int)) as
                                        uint32_t],
                               readcb:
                                   Some(bblog_read_proc as
                                            unsafe extern "C" fn(_:
                                                                     *mut uint8_t,
                                                                 _:
                                                                     libc::c_int,
                                                                 _: uint32_t,
                                                                 _:
                                                                     *mut emfat_entry_t)
                                                -> ()),
                               writecb: None,
                               priv_0:
                                   {
                                       let mut init =
                                           C2RustUnnamed{first_clust:
                                                             0 as libc::c_int
                                                                 as uint32_t,
                                                         last_clust: 0,
                                                         last_reserved: 0,
                                                         num_subentry: 0,
                                                         top:
                                                             0 as
                                                                 *mut emfat_entry_s,
                                                         sub:
                                                             0 as
                                                                 *mut emfat_entry_s,
                                                         next:
                                                             0 as
                                                                 *mut emfat_entry_s,};
                                       init
                                   },};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
