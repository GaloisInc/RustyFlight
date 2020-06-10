use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pinioBoxConfig_s {
    pub permanentId: [uint8_t; 4],
}
pub type pinioBoxConfig_t = pinioBoxConfig_s;
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
pub static mut pinioBoxConfig_System: pinioBoxConfig_t =
    pinioBoxConfig_t{permanentId: [0; 4],};
#[no_mangle]
pub static mut pinioBoxConfig_Copy: pinioBoxConfig_t =
    pinioBoxConfig_t{permanentId: [0; 4],};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut pinioBoxConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (530 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<pinioBoxConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &pinioBoxConfig_System as
                                     *const pinioBoxConfig_t as
                                     *mut pinioBoxConfig_t as *mut uint8_t,
                             copy:
                                 &pinioBoxConfig_Copy as
                                     *const pinioBoxConfig_t as
                                     *mut pinioBoxConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_pinioBoxConfig
                                                         as
                                                         *const pinioBoxConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_pinioBoxConfig: pinioBoxConfig_t =
    {
        let mut init =
            pinioBoxConfig_s{permanentId:
                                 [255 as libc::c_int as uint8_t,
                                  255 as libc::c_int as uint8_t,
                                  255 as libc::c_int as uint8_t,
                                  255 as libc::c_int as uint8_t],};
        init
    };
