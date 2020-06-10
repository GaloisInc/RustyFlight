use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    // in seconds
    // rad/s -> range: +/- 8192; +/- 2000 deg/se
    // m/s/s NED, body frame -> sim 1G = 9.80665, FC 1G = 256
    //w, x, y, z
    // m/s, earth frame
    // meters, NED from origin
    // normal: [0.0, 1.0], 3D: [-1.0, 1.0]
    #[no_mangle]
    fn FLASH_Unlock();
    #[no_mangle]
    fn FLASH_Lock();
    #[no_mangle]
    fn FLASH_ErasePage(Page_Address: uintptr_t) -> FLASH_Status;
    #[no_mangle]
    fn FLASH_ProgramWord(addr: uintptr_t, Data: uint32_t) -> FLASH_Status;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type uintptr_t = libc::c_ulong;
pub type FLASH_Status = libc::c_uint;
pub const FLASH_TIMEOUT: FLASH_Status = 5;
pub const FLASH_COMPLETE: FLASH_Status = 4;
pub const FLASH_ERROR_WRP: FLASH_Status = 3;
pub const FLASH_ERROR_PG: FLASH_Status = 2;
pub const FLASH_BUSY: FLASH_Status = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct config_streamer_s {
    pub address: uintptr_t,
    pub size: libc::c_int,
    pub buffer: C2RustUnnamed,
    pub at: libc::c_int,
    pub err: libc::c_int,
    pub unlocked: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub b: [uint8_t; 4],
    pub w: uint32_t,
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
// Streams data out to the EEPROM, padding to the write size as
// needed, and updating the checksum as it goes.
pub type config_streamer_t = config_streamer_s;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[no_mangle]
pub unsafe extern "C" fn config_streamer_init(mut c: *mut config_streamer_t) {
    memset(c as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<config_streamer_t>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_start(mut c: *mut config_streamer_t,
                                               mut base: uintptr_t,
                                               mut size: libc::c_int) {
    // base must start at FLASH_PAGE_SIZE boundary
    (*c).address = base;
    (*c).size = size;
    if !(*c).unlocked {
        FLASH_Unlock();
        (*c).unlocked = 1 as libc::c_int != 0
    }
    // NOP
    (*c).err = 0 as libc::c_int;
}
unsafe extern "C" fn write_word(mut c: *mut config_streamer_t,
                                mut value: uint32_t) -> libc::c_int {
    if (*c).err != 0 as libc::c_int { return (*c).err }
    if (*c).address.wrapping_rem(0x400 as libc::c_int as libc::c_ulong) ==
           0 as libc::c_int as libc::c_ulong {
        let status: FLASH_Status = FLASH_ErasePage((*c).address);
        if status as libc::c_uint !=
               FLASH_COMPLETE as libc::c_int as libc::c_uint {
            return -(1 as libc::c_int)
        }
    }
    let status_0: FLASH_Status = FLASH_ProgramWord((*c).address, value);
    if status_0 as libc::c_uint !=
           FLASH_COMPLETE as libc::c_int as libc::c_uint {
        return -(2 as libc::c_int)
    }
    (*c).address =
        ((*c).address as
             libc::c_ulong).wrapping_add(::core::mem::size_of::<uint32_t>() as
                                             libc::c_ulong) as uintptr_t as
            uintptr_t;
    return 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_write(mut c: *mut config_streamer_t,
                                               mut p: *const uint8_t,
                                               mut size: uint32_t)
 -> libc::c_int {
    let mut pat: *const uint8_t = p;
    while pat != (p as *mut uint8_t).offset(size as isize) as *const uint8_t {
        let fresh0 = (*c).at;
        (*c).at = (*c).at + 1;
        (*c).buffer.b[fresh0 as usize] = *pat;
        if (*c).at as libc::c_ulong ==
               ::core::mem::size_of::<C2RustUnnamed>() as libc::c_ulong {
            (*c).err = write_word(c, (*c).buffer.w);
            (*c).at = 0 as libc::c_int
        }
        pat = pat.offset(1)
    }
    return (*c).err;
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_status(mut c: *mut config_streamer_t)
 -> libc::c_int {
    return (*c).err;
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_flush(mut c: *mut config_streamer_t)
 -> libc::c_int {
    if (*c).at != 0 as libc::c_int {
        memset((*c).buffer.b.as_mut_ptr().offset((*c).at as isize) as
                   *mut libc::c_void, 0 as libc::c_int,
               (::core::mem::size_of::<C2RustUnnamed>() as
                    libc::c_ulong).wrapping_sub((*c).at as libc::c_ulong));
        (*c).err = write_word(c, (*c).buffer.w);
        (*c).at = 0 as libc::c_int
    }
    return (*c).err;
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_finish(mut c: *mut config_streamer_t)
 -> libc::c_int {
    if (*c).unlocked { FLASH_Lock(); (*c).unlocked = 0 as libc::c_int != 0 }
    return (*c).err;
}
