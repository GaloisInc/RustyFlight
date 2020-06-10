use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type size_t = libc::c_ulong;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPinConfig_s {
    pub ioTagTx: [ioTag_t; 12],
    pub ioTagRx: [ioTag_t; 12],
    pub ioTagInverter: [ioTag_t; 12],
}
pub type serialPinConfig_t = serialPinConfig_s;
pub type serialPortIdentifier_e = libc::c_int;
pub const SERIAL_PORT_SOFTSERIAL2: serialPortIdentifier_e = 31;
pub const SERIAL_PORT_SOFTSERIAL1: serialPortIdentifier_e = 30;
pub const SERIAL_PORT_USB_VCP: serialPortIdentifier_e = 20;
pub const SERIAL_PORT_USART8: serialPortIdentifier_e = 7;
pub const SERIAL_PORT_USART7: serialPortIdentifier_e = 6;
pub const SERIAL_PORT_USART6: serialPortIdentifier_e = 5;
pub const SERIAL_PORT_UART5: serialPortIdentifier_e = 4;
pub const SERIAL_PORT_UART4: serialPortIdentifier_e = 3;
pub const SERIAL_PORT_USART3: serialPortIdentifier_e = 2;
pub const SERIAL_PORT_USART2: serialPortIdentifier_e = 1;
pub const SERIAL_PORT_USART1: serialPortIdentifier_e = 0;
pub const SERIAL_PORT_NONE: serialPortIdentifier_e = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialDefaultPin_s {
    pub ident: serialPortIdentifier_e,
    pub rxIO: ioTag_t,
    pub txIO: ioTag_t,
    pub inverterIO: ioTag_t,
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
// Backward compatibility for exisiting targets
// F1 targets don't explicitly define pins.
// STM32F1
// XXX Is there an F3 target that does not define UART pins?
// STM32F3
// Default pin (NONE).
pub type serialDefaultPin_t = serialDefaultPin_s;
static mut serialDefaultPin: [serialDefaultPin_t; 8] =
    [{
         let mut init =
             serialDefaultPin_s{ident: SERIAL_PORT_USART1,
                                rxIO:
                                    ((0 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 10 as libc::c_int)
                                        as ioTag_t,
                                txIO:
                                    ((0 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 9 as libc::c_int)
                                        as ioTag_t,
                                inverterIO: 0 as libc::c_int as ioTag_t,};
         init
     },
     {
         let mut init =
             serialDefaultPin_s{ident: SERIAL_PORT_USART2,
                                rxIO: 0 as libc::c_int as ioTag_t,
                                txIO: 0 as libc::c_int as ioTag_t,
                                inverterIO: 0 as libc::c_int as ioTag_t,};
         init
     },
     {
         let mut init =
             serialDefaultPin_s{ident: SERIAL_PORT_USART3,
                                rxIO:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 11 as libc::c_int)
                                        as ioTag_t,
                                txIO:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 10 as libc::c_int)
                                        as ioTag_t,
                                inverterIO: 0 as libc::c_int as ioTag_t,};
         init
     },
     {
         let mut init =
             serialDefaultPin_s{ident: SERIAL_PORT_UART4,
                                rxIO:
                                    ((0 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 1 as libc::c_int)
                                        as ioTag_t,
                                txIO: 0 as libc::c_int as ioTag_t,
                                inverterIO: 0 as libc::c_int as ioTag_t,};
         init
     },
     {
         let mut init =
             serialDefaultPin_s{ident: SERIAL_PORT_UART5,
                                rxIO:
                                    ((3 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 2 as libc::c_int)
                                        as ioTag_t,
                                txIO:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 12 as libc::c_int)
                                        as ioTag_t,
                                inverterIO: 0 as libc::c_int as ioTag_t,};
         init
     },
     {
         let mut init =
             serialDefaultPin_s{ident: SERIAL_PORT_USART6,
                                rxIO:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 7 as libc::c_int)
                                        as ioTag_t,
                                txIO:
                                    ((2 as libc::c_int + 1 as libc::c_int) <<
                                         4 as libc::c_int | 6 as libc::c_int)
                                        as ioTag_t,
                                inverterIO: 0 as libc::c_int as ioTag_t,};
         init
     },
     {
         let mut init =
             serialDefaultPin_s{ident: SERIAL_PORT_SOFTSERIAL1,
                                rxIO: 0 as libc::c_int as ioTag_t,
                                txIO: 0 as libc::c_int as ioTag_t,
                                inverterIO: 0 as libc::c_int as ioTag_t,};
         init
     },
     {
         let mut init =
             serialDefaultPin_s{ident: SERIAL_PORT_SOFTSERIAL2,
                                rxIO: 0 as libc::c_int as ioTag_t,
                                txIO: 0 as libc::c_int as ioTag_t,
                                inverterIO: 0 as libc::c_int as ioTag_t,};
         init
     }];
#[no_mangle]
pub static mut serialPinConfig_Copy: serialPinConfig_t =
    serialPinConfig_t{ioTagTx: [0; 12],
                      ioTagRx: [0; 12],
                      ioTagInverter: [0; 12],};
#[no_mangle]
pub static mut serialPinConfig_System: serialPinConfig_t =
    serialPinConfig_t{ioTagTx: [0; 12],
                      ioTagRx: [0; 12],
                      ioTagInverter: [0; 12],};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut serialPinConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (509 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<serialPinConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &serialPinConfig_System as
                                     *const serialPinConfig_t as
                                     *mut serialPinConfig_t as *mut uint8_t,
                             copy:
                                 &serialPinConfig_Copy as
                                     *const serialPinConfig_t as
                                     *mut serialPinConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut serialPinConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_serialPinConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut serialPinConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_serialPinConfig(mut serialPinConfig:
                                                       *mut serialPinConfig_t) {
    let mut index: size_t = 0 as libc::c_int as size_t;
    while index <
              (::core::mem::size_of::<[serialDefaultPin_t; 8]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<serialDefaultPin_t>()
                                                   as libc::c_ulong) {
        let mut defpin: *const serialDefaultPin_t =
            &*serialDefaultPin.as_ptr().offset(index as isize) as
                *const serialDefaultPin_t;
        (*serialPinConfig).ioTagRx[if (*defpin).ident as libc::c_int <=
                                          SERIAL_PORT_USART8 as libc::c_int {
                                       (*defpin).ident as libc::c_int
                                   } else {
                                       (10 as libc::c_int) +
                                           ((*defpin).ident as libc::c_int -
                                                SERIAL_PORT_SOFTSERIAL1 as
                                                    libc::c_int)
                                   } as usize] = (*defpin).rxIO;
        (*serialPinConfig).ioTagTx[if (*defpin).ident as libc::c_int <=
                                          SERIAL_PORT_USART8 as libc::c_int {
                                       (*defpin).ident as libc::c_int
                                   } else {
                                       (10 as libc::c_int) +
                                           ((*defpin).ident as libc::c_int -
                                                SERIAL_PORT_SOFTSERIAL1 as
                                                    libc::c_int)
                                   } as usize] = (*defpin).txIO;
        (*serialPinConfig).ioTagInverter[if (*defpin).ident as libc::c_int <=
                                                SERIAL_PORT_USART8 as
                                                    libc::c_int {
                                             (*defpin).ident as libc::c_int
                                         } else {
                                             (10 as libc::c_int) +
                                                 ((*defpin).ident as
                                                      libc::c_int -
                                                      SERIAL_PORT_SOFTSERIAL1
                                                          as libc::c_int)
                                         } as usize] = (*defpin).inverterIO;
        index = index.wrapping_add(1)
    };
}
