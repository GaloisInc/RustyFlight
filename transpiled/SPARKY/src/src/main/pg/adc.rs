use ::libc;
extern "C" {
    #[no_mangle]
    fn adcDeviceByInstance(instance: *mut ADC_TypeDef) -> ADCDevice;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @}
  */
/* * @addtogroup Peripheral_registers_structures
  * @{
  */
/* * 
  * @brief Analog to Digital Converter  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_TypeDef {
    pub ISR: uint32_t,
    pub IER: uint32_t,
    pub CR: uint32_t,
    pub CFGR: uint32_t,
    pub RESERVED0: uint32_t,
    pub SMPR1: uint32_t,
    pub SMPR2: uint32_t,
    pub RESERVED1: uint32_t,
    pub TR1: uint32_t,
    pub TR2: uint32_t,
    pub TR3: uint32_t,
    pub RESERVED2: uint32_t,
    pub SQR1: uint32_t,
    pub SQR2: uint32_t,
    pub SQR3: uint32_t,
    pub SQR4: uint32_t,
    pub DR: uint32_t,
    pub RESERVED3: uint32_t,
    pub RESERVED4: uint32_t,
    pub JSQR: uint32_t,
    pub RESERVED5: [uint32_t; 4],
    pub OFR1: uint32_t,
    pub OFR2: uint32_t,
    pub OFR3: uint32_t,
    pub OFR4: uint32_t,
    pub RESERVED6: [uint32_t; 4],
    pub JDR1: uint32_t,
    pub JDR2: uint32_t,
    pub JDR3: uint32_t,
    pub JDR4: uint32_t,
    pub RESERVED7: [uint32_t; 4],
    pub AWD2CR: uint32_t,
    pub AWD3CR: uint32_t,
    pub RESERVED8: uint32_t,
    pub RESERVED9: uint32_t,
    pub DIFSEL: uint32_t,
    pub CALFACT: uint32_t,
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
pub type ADCDevice = libc::c_int;
pub const ADCDEV_COUNT: ADCDevice = 4;
pub const ADCDEV_4: ADCDevice = 3;
pub const ADCDEV_3: ADCDevice = 2;
pub const ADCDEV_2: ADCDevice = 1;
pub const ADCDEV_1: ADCDevice = 0;
pub const ADCINVALID: ADCDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcConfig_s {
    pub vbat: adcChannelConfig_t,
    pub rssi: adcChannelConfig_t,
    pub current: adcChannelConfig_t,
    pub external1: adcChannelConfig_t,
    pub device: int8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcChannelConfig_t {
    pub enabled: bool,
    pub ioTag: ioTag_t,
}
pub type adcConfig_t = adcConfig_s;
// ADCDevice
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
pub static mut adcConfig_System: adcConfig_t =
    adcConfig_t{vbat: adcChannelConfig_t{enabled: false, ioTag: 0,},
                rssi: adcChannelConfig_t{enabled: false, ioTag: 0,},
                current: adcChannelConfig_t{enabled: false, ioTag: 0,},
                external1: adcChannelConfig_t{enabled: false, ioTag: 0,},
                device: 0,};
#[no_mangle]
pub static mut adcConfig_Copy: adcConfig_t =
    adcConfig_t{vbat: adcChannelConfig_t{enabled: false, ioTag: 0,},
                rssi: adcChannelConfig_t{enabled: false, ioTag: 0,},
                current: adcChannelConfig_t{enabled: false, ioTag: 0,},
                external1: adcChannelConfig_t{enabled: false, ioTag: 0,},
                device: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut adcConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (510 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<adcConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &adcConfig_System as *const adcConfig_t as
                                     *mut adcConfig_t as *mut uint8_t,
                             copy:
                                 &adcConfig_Copy as *const adcConfig_t as
                                     *mut adcConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut adcConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_adcConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut adcConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_adcConfig(mut adcConfig:
                                                 *mut adcConfig_t) {
    (*adcConfig).device =
        (adcDeviceByInstance((0x40000000 as libc::c_int as
                                  uint32_t).wrapping_add(0x10000000 as
                                                             libc::c_int as
                                                             libc::c_uint).wrapping_add(0x100
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint)
                                 as *mut ADC_TypeDef) as libc::c_int +
             1 as libc::c_int) as int8_t;
    (*adcConfig).vbat.enabled = 1 as libc::c_int != 0;
    (*adcConfig).vbat.ioTag =
        ((0 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
             4 as libc::c_int) as ioTag_t;
    (*adcConfig).current.enabled = 1 as libc::c_int != 0;
    (*adcConfig).current.ioTag =
        ((0 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
             7 as libc::c_int) as ioTag_t;
}
// USE_ADC
