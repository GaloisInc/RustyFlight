use ::libc;
extern "C" {
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn flashInit(flashConfig_0: *const flashConfig_t) -> bool;
    #[no_mangle]
    static mut flashConfig_System: flashConfig_t;
    #[no_mangle]
    fn flashfsInit();
    #[no_mangle]
    fn emfat_read(emfat_0: *mut emfat_t, data: *mut uint8_t, sector: uint32_t,
                  num_sectors: libc::c_int);
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
 *
 * Author: jflyper (https://github.com/jflyper)
 *
 */
    #[no_mangle]
    static mut emfat: emfat_t;
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
    fn emfat_init_files();
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// millisecond time
pub type timeMs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flashConfig_s {
    pub csTag: ioTag_t,
    pub spiDevice: uint8_t,
}
pub type flashConfig_t = flashConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USBD_STORAGE {
    pub Init: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub GetCapacity: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint32_t,
                                                 _: *mut uint16_t) -> int8_t>,
    pub IsReady: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub IsWriteProtected: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub Read: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                          _: uint32_t, _: uint16_t)
                         -> int8_t>,
    pub Write: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                           _: uint32_t, _: uint16_t)
                          -> int8_t>,
    pub GetMaxLun: Option<unsafe extern "C" fn() -> int8_t>,
    pub pInquiry: *mut int8_t,
}
pub type USBD_StorageTypeDef = _USBD_STORAGE;
pub type emfat_t = emfat_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct emfat_s {
    pub vol_size: uint64_t,
    pub disk_sectors: uint32_t,
    pub vol_label: *const libc::c_char,
    pub priv_0: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed {
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
pub type emfat_entry_t = emfat_entry_s;
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
    pub priv_0: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_0 {
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
/* *< create/mod/access time in unix format */
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
pub type emfat_readcb_t
    =
    Option<unsafe extern "C" fn(_: *mut uint8_t, _: libc::c_int, _: uint32_t,
                                _: *mut emfat_entry_s) -> ()>;
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
#[inline]
unsafe extern "C" fn flashConfig() -> *const flashConfig_t {
    return &mut flashConfig_System;
}
static mut STORAGE_Inquirydata: [uint8_t; 36] =
    [0 as libc::c_int as uint8_t, 0x80 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     (0x24 as libc::c_int - 5 as libc::c_int) as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 'B' as i32 as uint8_t,
     'E' as i32 as uint8_t, 'T' as i32 as uint8_t, 'A' as i32 as uint8_t,
     'F' as i32 as uint8_t, 'L' as i32 as uint8_t, 'T' as i32 as uint8_t,
     ' ' as i32 as uint8_t, 'O' as i32 as uint8_t, 'n' as i32 as uint8_t,
     'b' as i32 as uint8_t, 'o' as i32 as uint8_t, 'a' as i32 as uint8_t,
     'r' as i32 as uint8_t, 'd' as i32 as uint8_t, ' ' as i32 as uint8_t,
     'F' as i32 as uint8_t, 'l' as i32 as uint8_t, 'a' as i32 as uint8_t,
     's' as i32 as uint8_t, 'h' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t];
unsafe extern "C" fn STORAGE_Init(mut lun: uint8_t) -> int8_t {
    ledSet(0 as libc::c_int, 1 as libc::c_int != 0);
    flashInit(flashConfig());
    flashfsInit();
    emfat_init_files();
    delay(1000 as libc::c_int as timeMs_t);
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    return 0 as libc::c_int as int8_t;
}
unsafe extern "C" fn STORAGE_GetCapacity(mut lun: uint8_t,
                                         mut block_num: *mut uint32_t,
                                         mut block_size: *mut uint16_t)
 -> int8_t {
    *block_size = 512 as libc::c_int as uint16_t;
    *block_num = emfat.disk_sectors;
    return 0 as libc::c_int as int8_t;
}
unsafe extern "C" fn STORAGE_IsReady(mut lun: uint8_t) -> int8_t {
    return 0 as libc::c_int as int8_t;
}
unsafe extern "C" fn STORAGE_IsWriteProtected(mut lun: uint8_t) -> int8_t {
    return 1 as libc::c_int as int8_t;
}
unsafe extern "C" fn STORAGE_Read(mut lun: uint8_t, mut buf: *mut uint8_t,
                                  mut blk_addr: uint32_t,
                                  mut blk_len: uint16_t) -> int8_t 
 // nmber of blocks to be read
 {
    ledSet(0 as libc::c_int, 1 as libc::c_int != 0);
    emfat_read(&mut emfat, buf, blk_addr, blk_len as libc::c_int);
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    return 0 as libc::c_int as int8_t;
}
unsafe extern "C" fn STORAGE_Write(mut lun: uint8_t, mut buf: *mut uint8_t,
                                   mut blk_addr: uint32_t,
                                   mut blk_len: uint16_t) -> int8_t {
    return 1 as libc::c_int as int8_t;
}
unsafe extern "C" fn STORAGE_GetMaxLun() -> int8_t {
    return (1 as libc::c_int - 1 as libc::c_int) as int8_t;
}
#[no_mangle]
pub static mut USBD_MSC_EMFAT_fops: USBD_StorageTypeDef =
    unsafe {
        {
            let mut init =
                _USBD_STORAGE{Init:
                                  Some(STORAGE_Init as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              GetCapacity:
                                  Some(STORAGE_GetCapacity as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint32_t,
                                                                _:
                                                                    *mut uint16_t)
                                               -> int8_t),
                              IsReady:
                                  Some(STORAGE_IsReady as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              IsWriteProtected:
                                  Some(STORAGE_IsWriteProtected as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              Read:
                                  Some(STORAGE_Read as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _: uint32_t,
                                                                _: uint16_t)
                                               -> int8_t),
                              Write:
                                  Some(STORAGE_Write as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _: uint32_t,
                                                                _: uint16_t)
                                               -> int8_t),
                              GetMaxLun:
                                  Some(STORAGE_GetMaxLun as
                                           unsafe extern "C" fn() -> int8_t),
                              pInquiry:
                                  STORAGE_Inquirydata.as_ptr() as
                                      *mut int8_t,};
            init
        }
    };
