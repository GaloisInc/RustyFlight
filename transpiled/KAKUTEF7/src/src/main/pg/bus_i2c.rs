use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
}
pub type size_t = libc::c_ulong;
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cConfig_s {
    pub ioTagScl: ioTag_t,
    pub ioTagSda: ioTag_t,
    pub overClock: bool,
    pub pullUp: bool,
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
pub type i2cConfig_t = i2cConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cDefaultConfig_s {
    pub device: I2CDevice,
    pub ioTagScl: ioTag_t,
    pub ioTagSda: ioTag_t,
    pub overClock: bool,
    pub pullUp: bool,
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
/*
 * Created by jflyper
 */
pub type i2cDefaultConfig_t = i2cDefaultConfig_s;
static mut i2cDefaultConfig: [i2cDefaultConfig_t; 1] =
    [{
         let mut init =
             i2cDefaultConfig_s{device: I2CDEV_1,
                                ioTagScl:
                                    ((1 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 6 as libc::c_int)
                                        as ioTag_t,
                                ioTagSda:
                                    ((1 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 7 as libc::c_int)
                                        as ioTag_t,
                                overClock: 1 as libc::c_int != 0,
                                pullUp: 0 as libc::c_int != 0,};
         init
     }];
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_i2cConfig(mut i2cConfig:
                                                 *mut i2cConfig_t) {
    memset(i2cConfig as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<i2cConfig_t>() as libc::c_ulong);
    let mut index: size_t = 0 as libc::c_int as size_t;
    while index <
              (::core::mem::size_of::<[i2cDefaultConfig_t; 1]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<i2cDefaultConfig_t>()
                                                   as libc::c_ulong) {
        let mut defconf: *const i2cDefaultConfig_t =
            &*i2cDefaultConfig.as_ptr().offset(index as isize) as
                *const i2cDefaultConfig_t;
        let mut device: libc::c_int = (*defconf).device as libc::c_int;
        (*i2cConfig.offset(device as isize)).ioTagScl = (*defconf).ioTagScl;
        (*i2cConfig.offset(device as isize)).ioTagSda = (*defconf).ioTagSda;
        (*i2cConfig.offset(device as isize)).overClock = (*defconf).overClock;
        (*i2cConfig.offset(device as isize)).pullUp = (*defconf).pullUp;
        index = index.wrapping_add(1)
    };
}
#[no_mangle]
pub static mut i2cConfig_SystemArray: [i2cConfig_t; 4] =
    [i2cConfig_t{ioTagScl: 0, ioTagSda: 0, overClock: false, pullUp: false,};
        4];
#[no_mangle]
pub static mut i2cConfig_CopyArray: [i2cConfig_t; 4] =
    [i2cConfig_t{ioTagScl: 0, ioTagSda: 0, overClock: false, pullUp: false,};
        4];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut i2cConfig_Registry: pgRegistry_t =
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
    i2cConfig_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (518 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<i2cConfig_t>() as
                                       libc::c_ulong).wrapping_mul(4 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut i2cConfig_SystemArray as
                                     *mut [i2cConfig_t; 4] as *mut uint8_t,
                             copy:
                                 &mut i2cConfig_CopyArray as
                                     *mut [i2cConfig_t; 4] as *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut i2cConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_i2cConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut i2cConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
// defined(USE_I2C) && !defined(USE_SOFT_I2C)
