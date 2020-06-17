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
    static __pg_registry_start: [pgRegistry_t; 0];
    #[no_mangle]
    static __pg_registry_end: [pgRegistry_t; 0];
    #[no_mangle]
    static __pg_resetdata_start: [uint8_t; 0];
    #[no_mangle]
    static __pg_resetdata_end: [uint8_t; 0];
}
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
}
pub type pgRegistry_t = pgRegistry_s;
#[inline]
unsafe extern "C" fn pgN(mut reg: *const pgRegistry_t) -> uint16_t {
    return ((*reg).pgn as libc::c_int & PGR_PGN_MASK as libc::c_int) as
               uint16_t;
}
#[inline]
unsafe extern "C" fn pgVersion(mut reg: *const pgRegistry_t) -> uint8_t {
    return ((*reg).pgn as libc::c_int >> 12i32) as uint8_t;
}
#[inline]
unsafe extern "C" fn pgSize(mut reg: *const pgRegistry_t) -> uint16_t {
    return ((*reg).size as libc::c_int & PGR_SIZE_MASK as libc::c_int) as
               uint16_t;
}
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// Helper to iterate over the PG register.  Cheaper than a visitor style callback.
// Reset configuration to default (by name)
/* */
// Declare system config
/* */
// Declare system config array
/* */
// Register system config
/* Force external linkage for g++. Catch multi registration */
/* */
/* */
/* */
/* */
// Register system config array
/* */
/* */
/* */
// Emit reset defaults for config.
// Config must be registered with PG_REGISTER_<xxx>_WITH_RESET_TEMPLATE macro
/* */
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
pub unsafe extern "C" fn pgFind(mut pgn: pgn_t) -> *const pgRegistry_t {
    let mut reg: *const pgRegistry_t = __pg_registry_start.as_ptr();
    while reg < __pg_registry_end.as_ptr() {
        if pgN(reg) as libc::c_int == pgn as libc::c_int { return reg }
        reg = reg.offset(1)
    }
    return 0 as *const pgRegistry_t;
}
unsafe extern "C" fn pgOffset(mut reg: *const pgRegistry_t) -> *mut uint8_t {
    return (*reg).address;
}
#[no_mangle]
pub unsafe extern "C" fn pgResetInstance(mut reg: *const pgRegistry_t,
                                         mut base: *mut uint8_t) {
    let regSize: uint16_t = pgSize(reg);
    memset(base as *mut libc::c_void, 0i32, regSize as libc::c_ulong);
    if (*reg).reset.ptr >= __pg_resetdata_start.as_ptr() as *mut libc::c_void
           &&
           (*reg).reset.ptr < __pg_resetdata_end.as_ptr() as *mut libc::c_void
       {
        // pointer points to resetdata section, to it is data template
        memcpy(base as *mut libc::c_void, (*reg).reset.ptr,
               regSize as libc::c_ulong);
    } else if (*reg).reset.fn_0.is_some() {
        // reset function, call it
        (*reg).reset.fn_0.expect("non-null function pointer")(base as
                                                                  *mut libc::c_void,
                                                              regSize as
                                                                  libc::c_int);
    };
}
#[no_mangle]
pub unsafe extern "C" fn pgReset(mut reg: *const pgRegistry_t) {
    pgResetInstance(reg, pgOffset(reg));
}
#[no_mangle]
pub unsafe extern "C" fn pgResetCopy(mut copy: *mut libc::c_void,
                                     mut pgn: pgn_t) -> bool {
    let mut reg: *const pgRegistry_t = pgFind(pgn);
    if !reg.is_null() {
        pgResetInstance(reg, copy as *mut uint8_t);
        return 1i32 != 0
    }
    return 0i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pgLoad(mut reg: *const pgRegistry_t,
                                mut from: *const libc::c_void,
                                mut size: libc::c_int,
                                mut version: libc::c_int) -> bool {
    pgResetInstance(reg, pgOffset(reg));
    // restore only matching version, keep defaults otherwise
    if version == pgVersion(reg) as libc::c_int {
        let take: libc::c_int =
            ({
                 let mut _a: libc::c_int = size;
                 let mut _b: uint16_t = pgSize(reg);
                 if _a < _b as libc::c_int { _a } else { _b as libc::c_int }
             });
        memcpy(pgOffset(reg) as *mut libc::c_void, from,
               take as libc::c_ulong);
        return 1i32 != 0
    }
    return 0i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pgStore(mut reg: *const pgRegistry_t,
                                 mut to: *mut libc::c_void,
                                 mut size: libc::c_int) -> libc::c_int {
    let take: libc::c_int =
        ({
             let mut _a: libc::c_int = size;
             let mut _b: uint16_t = pgSize(reg);
             if _a < _b as libc::c_int { _a } else { _b as libc::c_int }
         });
    memcpy(to, pgOffset(reg) as *const libc::c_void, take as libc::c_ulong);
    return take;
}
#[no_mangle]
pub unsafe extern "C" fn pgResetAll() {
    let mut reg: *const pgRegistry_t = __pg_registry_start.as_ptr();
    while reg < __pg_registry_end.as_ptr() {
        pgReset(reg);
        reg = reg.offset(1)
    };
}
