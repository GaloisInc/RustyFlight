use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type size_t = libc::c_ulong;
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
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct spiPinConfig_s {
    pub ioTagSck: ioTag_t,
    pub ioTagMiso: ioTag_t,
    pub ioTagMosi: ioTag_t,
}
pub type spiPinConfig_t = spiPinConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct spiCs_s {
    pub csnTag: ioTag_t,
}
pub type spiCs_t = spiCs_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct spiDefaultConfig_s {
    pub device: SPIDevice,
    pub sck: ioTag_t,
    pub miso: ioTag_t,
    pub mosi: ioTag_t,
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
pub type spiDefaultConfig_t = spiDefaultConfig_s;
#[no_mangle]
pub static mut spiDefaultConfig: [spiDefaultConfig_t; 3] =
    [{
         let mut init =
             spiDefaultConfig_s{device: SPIDEV_1,
                                sck:
                                    ((0 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 5 as libc::c_int)
                                        as ioTag_t,
                                miso:
                                    ((0 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 6 as libc::c_int)
                                        as ioTag_t,
                                mosi:
                                    ((0 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 7 as libc::c_int)
                                        as ioTag_t,};
         init
     },
     {
         let mut init =
             spiDefaultConfig_s{device: SPIDEV_2,
                                sck:
                                    ((1 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 13 as libc::c_int)
                                        as ioTag_t,
                                miso:
                                    ((1 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 14 as libc::c_int)
                                        as ioTag_t,
                                mosi:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 3 as libc::c_int)
                                        as ioTag_t,};
         init
     },
     {
         let mut init =
             spiDefaultConfig_s{device: SPIDEV_3,
                                sck:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 10 as libc::c_int)
                                        as ioTag_t,
                                miso:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 11 as libc::c_int)
                                        as ioTag_t,
                                mosi:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 12 as libc::c_int)
                                        as ioTag_t,};
         init
     }];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut spiPinConfig_Registry: pgRegistry_t =
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
pub static mut spiPinConfig_SystemArray: [spiPinConfig_t; 4] =
    [spiPinConfig_t{ioTagSck: 0, ioTagMiso: 0, ioTagMosi: 0,}; 4];
#[no_mangle]
pub static mut spiPinConfig_CopyArray: [spiPinConfig_t; 4] =
    [spiPinConfig_t{ioTagSck: 0, ioTagMiso: 0, ioTagMosi: 0,}; 4];
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_spiPinConfig(mut spiPinConfig:
                                                    *mut spiPinConfig_t) {
    let mut i: size_t = 0 as libc::c_int as size_t;
    while i <
              (::core::mem::size_of::<[spiDefaultConfig_t; 3]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<spiDefaultConfig_t>()
                                                   as libc::c_ulong) {
        let mut defconf: *const spiDefaultConfig_t =
            &*spiDefaultConfig.as_ptr().offset(i as isize) as
                *const spiDefaultConfig_t;
        (*spiPinConfig.offset((*defconf).device as isize)).ioTagSck =
            (*defconf).sck;
        (*spiPinConfig.offset((*defconf).device as isize)).ioTagMiso =
            (*defconf).miso;
        (*spiPinConfig.offset((*defconf).device as isize)).ioTagMosi =
            (*defconf).mosi;
        i = i.wrapping_add(1)
    };
}
#[no_mangle]
pub static mut spiPreinitIPUConfig_SystemArray: [spiCs_t; 11] =
    [spiCs_t{csnTag: 0,}; 11];
#[no_mangle]
pub static mut spiPreinitIPUConfig_CopyArray: [spiCs_t; 11] =
    [spiCs_t{csnTag: 0,}; 11];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut spiPreinitIPUConfig_Registry: pgRegistry_t =
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
pub static mut spiPreinitOPUConfig_SystemArray: [spiCs_t; 2] =
    [spiCs_t{csnTag: 0,}; 2];
#[no_mangle]
pub static mut spiPreinitOPUConfig_CopyArray: [spiCs_t; 2] =
    [spiCs_t{csnTag: 0,}; 2];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut spiPreinitOPUConfig_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_0{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
// Initialization values for input pull-up are listed here.
// Explicit output with pull-up should handled in target dependent config.c.
// Generic target will be specifying both values by resource commands.
#[no_mangle]
pub static mut preinitIPUList: [ioTag_t; 11] =
    [((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
          12 as libc::c_int) as ioTag_t,
     ((0 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
          8 as libc::c_int) as ioTag_t,
     ((0 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
          10 as libc::c_int) as ioTag_t,
     ((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
          14 as libc::c_int) as ioTag_t,
     ((0 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
          15 as libc::c_int) as ioTag_t, 0 as libc::c_int as ioTag_t, 0, 0, 0,
     0, 0];
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_spiPreinitIPUConfig(mut config:
                                                           *mut spiCs_t) {
    let mut puPins: libc::c_int = 0 as libc::c_int;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 11 as libc::c_int {
        let mut current_block_1: u64;
        let mut j: libc::c_int = 0 as libc::c_int;
        loop  {
            if !(j < i) { current_block_1 = 2473556513754201174; break ; }
            if (*config.offset(j as isize)).csnTag as libc::c_int ==
                   preinitIPUList[i as usize] as libc::c_int {
                current_block_1 = 13513818773234778473;
                break ;
            }
            j += 1
        }
        match current_block_1 {
            2473556513754201174 => {
                let fresh0 = puPins;
                puPins = puPins + 1;
                (*config.offset(fresh0 as isize)).csnTag =
                    preinitIPUList[i as usize]
            }
            _ => { }
        }
        i += 1
    };
}
unsafe extern "C" fn run_static_initializers() {
    spiPinConfig_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (520 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<spiPinConfig_t>() as
                                       libc::c_ulong).wrapping_mul(4 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut spiPinConfig_SystemArray as
                                     *mut [spiPinConfig_t; 4] as *mut uint8_t,
                             copy:
                                 &mut spiPinConfig_CopyArray as
                                     *mut [spiPinConfig_t; 4] as *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut spiPinConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_spiPinConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut spiPinConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        };
    spiPreinitIPUConfig_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (535 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<spiCs_t>() as
                                       libc::c_ulong).wrapping_mul(11 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut spiPreinitIPUConfig_SystemArray as
                                     *mut [spiCs_t; 11] as *mut uint8_t,
                             copy:
                                 &mut spiPreinitIPUConfig_CopyArray as
                                     *mut [spiCs_t; 11] as *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut spiCs_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_spiPreinitIPUConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut spiCs_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        };
    spiPreinitOPUConfig_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (536 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<spiCs_t>() as
                                       libc::c_ulong).wrapping_mul(2 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut spiPreinitOPUConfig_SystemArray as
                                     *mut [spiCs_t; 2] as *mut uint8_t,
                             copy:
                                 &mut spiPreinitOPUConfig_CopyArray as
                                     *mut [spiCs_t; 2] as *mut uint8_t,
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
