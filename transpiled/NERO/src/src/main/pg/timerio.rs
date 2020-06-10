use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type ioTag_t = uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerIOConfig_s {
    pub ioTag: ioTag_t,
    pub index: uint8_t,
}
pub type timerIOConfig_t = timerIOConfig_s;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
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
pub static mut timerIOConfig_SystemArray: [timerIOConfig_t; 10] =
    [timerIOConfig_t{ioTag: 0, index: 0,}; 10];
#[no_mangle]
pub static mut timerIOConfig_CopyArray: [timerIOConfig_t; 10] =
    [timerIOConfig_t{ioTag: 0, index: 0,}; 10];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut timerIOConfig_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_0{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
unsafe extern "C" fn run_static_initializers() {
    timerIOConfig_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (534 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<timerIOConfig_t>()
                                       as
                                       libc::c_ulong).wrapping_mul(10 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut timerIOConfig_SystemArray as
                                     *mut [timerIOConfig_t; 10] as
                                     *mut uint8_t,
                             copy:
                                 &mut timerIOConfig_CopyArray as
                                     *mut [timerIOConfig_t; 10] as
                                     *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     0 as
                                                         *mut libc::c_void,},};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
