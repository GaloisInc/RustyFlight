use core;
use libc;
extern "C" {
    #[no_mangle]
    fn strncpy(_: *mut libc::c_char, _: *const libc::c_char, _: libc::c_ulong)
     -> *mut libc::c_char;
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    static targetName: *const libc::c_char;
    #[no_mangle]
    fn getBoardName() -> *mut libc::c_char;
    #[no_mangle]
    fn getManufacturerId() -> *mut libc::c_char;
    #[no_mangle]
    fn boardInformationIsSet() -> bool;
    #[no_mangle]
    fn getSignature() -> *mut uint8_t;
    #[no_mangle]
    fn signatureIsSet() -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct boardConfig_s {
    pub signature: [uint8_t; 32],
    pub manufacturerId: [libc::c_char; 5],
    pub boardName: [libc::c_char; 21],
    pub boardInformationSet: uint8_t,
    pub signatureSet: uint8_t,
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
// Warning: This configuration is meant to be applied when loading the initial
// configuration for a generic board, and stay fixed after this, to enable
// identification of the hardware that this is running on.
// Do not modify this parameter group directly, use 'fc/board_info.h' instead.
pub type boardConfig_t = boardConfig_s;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
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
pub static mut boardConfig_System: boardConfig_t =
    boardConfig_t{signature: [0; 32],
                  manufacturerId: [0; 5],
                  boardName: [0; 21],
                  boardInformationSet: 0,
                  signatureSet: 0,};
#[no_mangle]
pub static mut boardConfig_Copy: boardConfig_t =
    boardConfig_t{signature: [0; 32],
                  manufacturerId: [0; 5],
                  boardName: [0; 21],
                  boardInformationSet: 0,
                  signatureSet: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut boardConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (538i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<boardConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &boardConfig_System as *const boardConfig_t
                                     as *mut boardConfig_t as *mut uint8_t,
                             copy:
                                 &boardConfig_Copy as *const boardConfig_t as
                                     *mut boardConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut boardConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<unsafe extern "C" fn(_:
                                                                                                              *mut libc::c_void,
                                                                                                          _:
                                                                                                              libc::c_int)
                                                                                         ->
                                                                                             ()>>(Some(pgResetFn_boardConfig
                                                                                                           as
                                                                                                           unsafe extern "C" fn(_:
                                                                                                                                    *mut boardConfig_t)
                                                                                                               ->
                                                                                                                   ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_boardConfig(mut boardConfig:
                                                   *mut boardConfig_t) {
    if boardInformationIsSet() {
        strncpy((*boardConfig).manufacturerId.as_mut_ptr(),
                getManufacturerId(), 4i32 as libc::c_ulong);
        strncpy((*boardConfig).boardName.as_mut_ptr(), getBoardName(),
                20i32 as libc::c_ulong);
        (*boardConfig).boardInformationSet = 1i32 as uint8_t
    } else {
        strncpy((*boardConfig).boardName.as_mut_ptr(), targetName,
                20i32 as libc::c_ulong);
        (*boardConfig).boardInformationSet = 1i32 as uint8_t
        // GENERIC_TARGET
    }
    if signatureIsSet() {
        memcpy((*boardConfig).signature.as_mut_ptr() as *mut libc::c_void,
               getSignature() as *const libc::c_void, 32i32 as libc::c_ulong);
        (*boardConfig).signatureSet = 1i32 as uint8_t
    } else { (*boardConfig).signatureSet = 0i32 as uint8_t };
}
// USE_BOARD_INFO:
