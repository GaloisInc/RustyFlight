use core;
use libc;
extern "C" {
    #[no_mangle]
    fn timerioTagGetByUsage(usageFlag: timerUsageFlag_e, index: uint8_t)
     -> ioTag_t;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
pub type inputFilteringMode_e = libc::c_uint;
pub const INPUT_FILTERING_ENABLED: inputFilteringMode_e = 1;
pub const INPUT_FILTERING_DISABLED: inputFilteringMode_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct ppmConfig_s {
    pub ioTag: ioTag_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pwmConfig_s {
    pub ioTags: [ioTag_t; 8],
    pub inputFilteringMode: inputFilteringMode_e,
}
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
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
pub type ppmConfig_t = ppmConfig_s;
pub type pwmConfig_t = pwmConfig_s;
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
pub static mut pwmConfig_System: pwmConfig_t =
    pwmConfig_t{ioTags: [0; 8],
                inputFilteringMode: INPUT_FILTERING_DISABLED,};
#[no_mangle]
pub static mut pwmConfig_Copy: pwmConfig_t =
    pwmConfig_t{ioTags: [0; 8],
                inputFilteringMode: INPUT_FILTERING_DISABLED,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut pwmConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (508i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<pwmConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &pwmConfig_System as *const pwmConfig_t as
                                     *mut pwmConfig_t as *mut uint8_t,
                             copy:
                                 &pwmConfig_Copy as *const pwmConfig_t as
                                     *mut pwmConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut pwmConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<unsafe extern "C" fn(_:
                                                                                                              *mut libc::c_void,
                                                                                                          _:
                                                                                                              libc::c_int)
                                                                                         ->
                                                                                             ()>>(Some(pgResetFn_pwmConfig
                                                                                                           as
                                                                                                           unsafe extern "C" fn(_:
                                                                                                                                    *mut pwmConfig_t)
                                                                                                               ->
                                                                                                                   ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_pwmConfig(mut pwmConfig:
                                                 *mut pwmConfig_t) {
    (*pwmConfig).inputFilteringMode = INPUT_FILTERING_DISABLED;
    let mut inputIndex: libc::c_uint = 0i32 as libc::c_uint;
    while inputIndex < 8i32 as libc::c_uint {
        (*pwmConfig).ioTags[inputIndex as usize] =
            timerioTagGetByUsage(TIM_USE_PWM, inputIndex as uint8_t);
        inputIndex = inputIndex.wrapping_add(1)
    };
}
#[no_mangle]
pub static mut ppmConfig_Copy: ppmConfig_t = ppmConfig_t{ioTag: 0,};
#[no_mangle]
pub static mut ppmConfig_System: ppmConfig_t = ppmConfig_t{ioTag: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut ppmConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (507i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<ppmConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &ppmConfig_System as *const ppmConfig_t as
                                     *mut ppmConfig_t as *mut uint8_t,
                             copy:
                                 &ppmConfig_Copy as *const ppmConfig_t as
                                     *mut ppmConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut ppmConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<unsafe extern "C" fn(_:
                                                                                                              *mut libc::c_void,
                                                                                                          _:
                                                                                                              libc::c_int)
                                                                                         ->
                                                                                             ()>>(Some(pgResetFn_ppmConfig
                                                                                                           as
                                                                                                           unsafe extern "C" fn(_:
                                                                                                                                    *mut ppmConfig_t)
                                                                                                               ->
                                                                                                                   ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_ppmConfig(mut ppmConfig:
                                                 *mut ppmConfig_t) {
    (*ppmConfig).ioTag = timerioTagGetByUsage(TIM_USE_PPM, 0i32 as uint8_t);
}
