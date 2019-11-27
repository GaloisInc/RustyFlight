use core;
use libc;
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
// documentary
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
pub type timeMs_t = uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rcdeviceConfig_s {
    pub initDeviceAttempts: uint8_t,
    pub initDeviceAttemptInterval: timeMs_t,
}
pub type rcdeviceConfig_t = rcdeviceConfig_s;
/* base */
/* size */
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
pub static mut rcdeviceConfig_System: rcdeviceConfig_t =
    rcdeviceConfig_t{initDeviceAttempts: 0, initDeviceAttemptInterval: 0,};
#[no_mangle]
pub static mut rcdeviceConfig_Copy: rcdeviceConfig_t =
    rcdeviceConfig_t{initDeviceAttempts: 0, initDeviceAttemptInterval: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut rcdeviceConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (539i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<rcdeviceConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &rcdeviceConfig_System as
                                     *const rcdeviceConfig_t as
                                     *mut rcdeviceConfig_t as *mut uint8_t,
                             copy:
                                 &rcdeviceConfig_Copy as
                                     *const rcdeviceConfig_t as
                                     *mut rcdeviceConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut rcdeviceConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<unsafe extern "C" fn(_:
                                                                                                              *mut libc::c_void,
                                                                                                          _:
                                                                                                              libc::c_int)
                                                                                         ->
                                                                                             ()>>(Some(pgResetFn_rcdeviceConfig
                                                                                                           as
                                                                                                           unsafe extern "C" fn(_:
                                                                                                                                    *mut rcdeviceConfig_t)
                                                                                                               ->
                                                                                                                   ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_rcdeviceConfig(mut rcdeviceConfig:
                                                      *mut rcdeviceConfig_t) {
    (*rcdeviceConfig).initDeviceAttempts = 4i32 as uint8_t;
    (*rcdeviceConfig).initDeviceAttemptInterval = 1000i32 as timeMs_t;
}
