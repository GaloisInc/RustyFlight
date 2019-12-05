use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    fn initRcProcessing();
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
pub struct systemConfig_s {
    pub pidProfileIndex: uint8_t,
    pub activeRateProfile: uint8_t,
    pub debug_mode: uint8_t,
    pub task_statistics: uint8_t,
    pub rateProfile6PosSwitch: uint8_t,
    pub cpu_overclock: uint8_t,
    pub powerOnArmingGraceTime: uint8_t,
    pub boardIdentifier: [libc::c_char; 6],
}
pub type systemConfig_t = systemConfig_s;
pub type controlRateConfig_t = controlRateConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct controlRateConfig_s {
    pub thrMid8: uint8_t,
    pub thrExpo8: uint8_t,
    pub rates_type: uint8_t,
    pub rcRates: [uint8_t; 3],
    pub rcExpo: [uint8_t; 3],
    pub rates: [uint8_t; 3],
    pub dynThrPID: uint8_t,
    pub tpa_breakpoint: uint16_t,
    pub throttle_limit_type: uint8_t,
    pub throttle_limit_percent: uint8_t,
}
pub type C2RustUnnamed_1 = libc::c_uint;
pub const RATES_TYPE_RACEFLIGHT: C2RustUnnamed_1 = 1;
pub const RATES_TYPE_BETAFLIGHT: C2RustUnnamed_1 = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const THROTTLE_LIMIT_TYPE_CLIP: C2RustUnnamed_2 = 2;
pub const THROTTLE_LIMIT_TYPE_SCALE: C2RustUnnamed_2 = 1;
pub const THROTTLE_LIMIT_TYPE_OFF: C2RustUnnamed_2 = 0;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// in seconds
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
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
// SITL (software in the loop) simulator
// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC
// file name to save config
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfigMutable() -> *mut systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn controlRateProfilesMutable(mut _index: libc::c_int)
 -> *mut controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
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
pub static mut currentControlRateProfile: *mut controlRateConfig_t =
    0 as *const controlRateConfig_t as *mut controlRateConfig_t;
#[no_mangle]
pub static mut controlRateProfiles_CopyArray: [controlRateConfig_t; 6] =
    [controlRateConfig_t{thrMid8: 0,
                         thrExpo8: 0,
                         rates_type: 0,
                         rcRates: [0; 3],
                         rcExpo: [0; 3],
                         rates: [0; 3],
                         dynThrPID: 0,
                         tpa_breakpoint: 0,
                         throttle_limit_type: 0,
                         throttle_limit_percent: 0,}; 6];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut controlRateProfiles_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_0{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
#[no_mangle]
pub static mut controlRateProfiles_SystemArray: [controlRateConfig_t; 6] =
    [controlRateConfig_t{thrMid8: 0,
                         thrExpo8: 0,
                         rates_type: 0,
                         rcRates: [0; 3],
                         rcExpo: [0; 3],
                         rates: [0; 3],
                         dynThrPID: 0,
                         tpa_breakpoint: 0,
                         throttle_limit_type: 0,
                         throttle_limit_percent: 0,}; 6];
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_controlRateProfiles(mut controlRateConfig:
                                                           *mut controlRateConfig_t) {
    let mut i: libc::c_int = 0i32;
    while i < 6i32 {
        static mut _reset_template_61: controlRateConfig_t =
            {
                let mut init =
                    controlRateConfig_s{thrMid8: 50i32 as uint8_t,
                                        thrExpo8: 0i32 as uint8_t,
                                        rates_type:
                                            RATES_TYPE_BETAFLIGHT as
                                                libc::c_int as uint8_t,
                                        rcRates:
                                            [100i32 as uint8_t,
                                             100i32 as uint8_t,
                                             100i32 as uint8_t],
                                        rcExpo:
                                            [0i32 as uint8_t, 0i32 as uint8_t,
                                             0i32 as uint8_t],
                                        rates:
                                            [70i32 as uint8_t,
                                             70i32 as uint8_t,
                                             70i32 as uint8_t],
                                        dynThrPID: 10i32 as uint8_t,
                                        tpa_breakpoint: 1650i32 as uint16_t,
                                        throttle_limit_type:
                                            THROTTLE_LIMIT_TYPE_OFF as
                                                libc::c_int as uint8_t,
                                        throttle_limit_percent:
                                            100i32 as uint8_t,};
                init
            };
        memcpy(&mut *controlRateConfig.offset(i as isize) as
                   *mut controlRateConfig_t as *mut libc::c_void,
               &_reset_template_61 as *const controlRateConfig_t as
                   *const libc::c_void,
               ::core::mem::size_of::<controlRateConfig_t>() as
                   libc::c_ulong);
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn loadControlRateProfile() {
    currentControlRateProfile =
        controlRateProfilesMutable((*systemConfig()).activeRateProfile as
                                       libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn changeControlRateProfile(mut controlRateProfileIndex:
                                                      uint8_t) {
    if (controlRateProfileIndex as libc::c_int) < 6i32 {
        (*systemConfigMutable()).activeRateProfile = controlRateProfileIndex
    }
    loadControlRateProfile();
    initRcProcessing();
}
#[no_mangle]
pub unsafe extern "C" fn copyControlRateProfile(dstControlRateProfileIndex:
                                                    uint8_t,
                                                srcControlRateProfileIndex:
                                                    uint8_t) {
    if (dstControlRateProfileIndex as libc::c_int) < 6i32 - 1i32 &&
           (srcControlRateProfileIndex as libc::c_int) < 6i32 - 1i32 &&
           dstControlRateProfileIndex as libc::c_int !=
               srcControlRateProfileIndex as libc::c_int {
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex as
                                              libc::c_int) as
                   *mut libc::c_void,
               controlRateProfilesMutable(srcControlRateProfileIndex as
                                              libc::c_int) as
                   *const libc::c_void,
               ::core::mem::size_of::<controlRateConfig_t>() as
                   libc::c_ulong);
    };
}
unsafe extern "C" fn run_static_initializers() {
    controlRateProfiles_Registry =
        {
            let mut init =
                pgRegistry_s{pgn: (12i32 | 1i32 << 12i32) as pgn_t,
                             size:
                                 ((::core::mem::size_of::<controlRateConfig_t>()
                                       as
                                       libc::c_ulong).wrapping_mul(6i32 as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut controlRateProfiles_SystemArray as
                                     *mut [controlRateConfig_t; 6] as
                                     *mut uint8_t,
                             copy:
                                 &mut controlRateProfiles_CopyArray as
                                     *mut [controlRateConfig_t; 6] as
                                     *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut controlRateConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<unsafe extern "C" fn(_:
                                                                                                              *mut libc::c_void,
                                                                                                          _:
                                                                                                              libc::c_int)
                                                                                         ->
                                                                                             ()>>(Some(pgResetFn_controlRateProfiles
                                                                                                           as
                                                                                                           unsafe extern "C" fn(_:
                                                                                                                                    *mut controlRateConfig_t)
                                                                                                               ->
                                                                                                                   ())),},};
            init
        }
}
#[used]
#[cfg_attr ( target_os = "linux", link_section = ".init_array" )]
#[cfg_attr ( target_os = "windows", link_section = ".CRT$XIB" )]
#[cfg_attr ( target_os = "macos", link_section = "__DATA,__mod_init_func" )]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
