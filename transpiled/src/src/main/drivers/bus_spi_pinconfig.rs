use core;
use libc;
extern "C" {
    #[no_mangle]
    static mut spiDevice: [spiDevice_t; 3];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type size_t = libc::c_ulong;
/* *
  * @brief Serial Peripheral Interface
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
    /* !< Reserved, 0x22                                                            */
}
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
pub type rccPeriphTag_t = uint8_t;
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct spiPinConfig_s {
    pub ioTagSck: ioTag_t,
    pub ioTagMiso: ioTag_t,
    pub ioTagMosi: ioTag_t,
}
pub type spiPinConfig_t = spiPinConfig_s;
pub type spiDevice_t = SPIDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SPIDevice_s {
    pub dev: *mut SPI_TypeDef,
    pub sck: ioTag_t,
    pub miso: ioTag_t,
    pub mosi: ioTag_t,
    pub af: uint8_t,
    pub rcc: rccPeriphTag_t,
    pub errorCount: uint16_t,
    pub leadingEdge: bool,
}
pub type spiHardware_t = spiHardware_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct spiHardware_s {
    pub device: SPIDevice,
    pub reg: *mut SPI_TypeDef,
    pub sckPins: [spiPinDef_t; 2],
    pub misoPins: [spiPinDef_t; 2],
    pub mosiPins: [spiPinDef_t; 2],
    pub af: uint8_t,
    pub rcc: rccPeriphTag_t,
}
pub type spiPinDef_t = spiPinDef_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct spiPinDef_s {
    pub pin: ioTag_t,
}
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
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
pub type rcc_reg = libc::c_uint;
pub const RCC_AHB1: rcc_reg = 4;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
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
// Initialized in run_static_initializers
#[no_mangle]
pub static mut spiHardware: [spiHardware_t; 3] =
    [spiHardware_t{device: SPIDEV_1,
                   reg: 0 as *mut SPI_TypeDef,
                   sckPins: [spiPinDef_t{pin: 0,}; 2],
                   misoPins: [spiPinDef_t{pin: 0,}; 2],
                   mosiPins: [spiPinDef_t{pin: 0,}; 2],
                   af: 0,
                   rcc: 0,}; 3];
#[no_mangle]
pub unsafe extern "C" fn spiPinConfigure(mut pConfig: *const spiPinConfig_t) {
    let mut hwindex: size_t = 0i32 as size_t;
    while hwindex <
              (::core::mem::size_of::<[spiHardware_t; 3]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<spiHardware_t>()
                                                   as libc::c_ulong) {
        let mut hw: *const spiHardware_t =
            &*spiHardware.as_ptr().offset(hwindex as isize) as
                *const spiHardware_t;
        if !(*hw).reg.is_null() {
            let mut device: SPIDevice = (*hw).device;
            let mut pDev: *mut spiDevice_t =
                &mut *spiDevice.as_mut_ptr().offset(device as isize) as
                    *mut spiDevice_t;
            let mut pindex: libc::c_int = 0i32;
            while pindex < 2i32 {
                if (*pConfig.offset(device as isize)).ioTagSck as libc::c_int
                       == (*hw).sckPins[pindex as usize].pin as libc::c_int {
                    (*pDev).sck = (*hw).sckPins[pindex as usize].pin
                }
                if (*pConfig.offset(device as isize)).ioTagMiso as libc::c_int
                       == (*hw).misoPins[pindex as usize].pin as libc::c_int {
                    (*pDev).miso = (*hw).misoPins[pindex as usize].pin
                }
                if (*pConfig.offset(device as isize)).ioTagMosi as libc::c_int
                       == (*hw).mosiPins[pindex as usize].pin as libc::c_int {
                    (*pDev).mosi = (*hw).mosiPins[pindex as usize].pin
                }
                pindex += 1
            }
            if (*pDev).sck as libc::c_int != 0 &&
                   (*pDev).miso as libc::c_int != 0 &&
                   (*pDev).mosi as libc::c_int != 0 {
                (*pDev).dev = (*hw).reg;
                (*pDev).af = (*hw).af;
                (*pDev).rcc = (*hw).rcc;
                (*pDev).leadingEdge = 0i32 != 0
                // XXX Should be part of transfer context
            }
        }
        hwindex = hwindex.wrapping_add(1)
    };
}
unsafe extern "C" fn run_static_initializers() {
    spiHardware =
        [{
             let mut init =
                 spiHardware_s{device: SPIDEV_1,
                               reg:
                                   (0x40000000i32 as
                                        uint32_t).wrapping_add(0x10000i32 as
                                                                   libc::c_uint).wrapping_add(0x3000i32
                                                                                                  as
                                                                                                  libc::c_uint)
                                       as *mut SPI_TypeDef,
                               sckPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (0i32 + 1i32 <<
                                                                 4i32 | 5i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 3i32)
                                                                as ioTag_t,};
                                        init
                                    }],
                               misoPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (0i32 + 1i32 <<
                                                                 4i32 | 6i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 4i32)
                                                                as ioTag_t,};
                                        init
                                    }],
                               mosiPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (0i32 + 1i32 <<
                                                                 4i32 | 7i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 5i32)
                                                                as ioTag_t,};
                                        init
                                    }],
                               af: 0x5i32 as uint8_t,
                               rcc:
                                   (((RCC_APB2 as libc::c_int) << 5i32) as
                                        libc::c_long |
                                        (16i32 *
                                             (0x1000i32 as uint32_t as
                                                  libc::c_long > 65535i64) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8i32 *
                                                  (0x1000i32 as uint32_t as
                                                       libc::c_long * 1i64 >>
                                                       16i32 *
                                                           (0x1000i32 as
                                                                uint32_t as
                                                                libc::c_long >
                                                                65535i64) as
                                                               libc::c_int >
                                                       255i32 as libc::c_long)
                                                      as libc::c_int) as
                                                 libc::c_long +
                                                 (8i32 as libc::c_long -
                                                      90i32 as libc::c_long /
                                                          ((0x1000i32 as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1i64 >>
                                                                16i32 *
                                                                    (0x1000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535i64)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8i32 *
                                                                    (0x1000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1i64
                                                                         >>
                                                                         16i32
                                                                             *
                                                                             (0x1000i32
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535i64)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255i32
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4i32 as
                                                                   libc::c_long
                                                               +
                                                               14i32 as
                                                                   libc::c_long
                                                               |
                                                               1i32 as
                                                                   libc::c_long)
                                                      -
                                                      2i32 as libc::c_long /
                                                          ((0x1000i32 as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1i64 >>
                                                                16i32 *
                                                                    (0x1000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535i64)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8i32 *
                                                                    (0x1000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1i64
                                                                         >>
                                                                         16i32
                                                                             *
                                                                             (0x1000i32
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535i64)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255i32
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2i32 as
                                                                   libc::c_long
                                                               +
                                                               1i32 as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,};
             init
         },
         {
             let mut init =
                 spiHardware_s{device: SPIDEV_2,
                               reg:
                                   (0x40000000i32 as
                                        uint32_t).wrapping_add(0x3800i32 as
                                                                   libc::c_uint)
                                       as *mut SPI_TypeDef,
                               sckPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 13i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 3i32)
                                                                as ioTag_t,};
                                        init
                                    }],
                               misoPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 14i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 4i32)
                                                                as ioTag_t,};
                                        init
                                    }],
                               mosiPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 15i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 5i32)
                                                                as ioTag_t,};
                                        init
                                    }],
                               af: 0x5i32 as uint8_t,
                               rcc:
                                   (((RCC_APB1 as libc::c_int) << 5i32) as
                                        libc::c_long |
                                        (16i32 *
                                             (0x4000i32 as uint32_t as
                                                  libc::c_long > 65535i64) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8i32 *
                                                  (0x4000i32 as uint32_t as
                                                       libc::c_long * 1i64 >>
                                                       16i32 *
                                                           (0x4000i32 as
                                                                uint32_t as
                                                                libc::c_long >
                                                                65535i64) as
                                                               libc::c_int >
                                                       255i32 as libc::c_long)
                                                      as libc::c_int) as
                                                 libc::c_long +
                                                 (8i32 as libc::c_long -
                                                      90i32 as libc::c_long /
                                                          ((0x4000i32 as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1i64 >>
                                                                16i32 *
                                                                    (0x4000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535i64)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8i32 *
                                                                    (0x4000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1i64
                                                                         >>
                                                                         16i32
                                                                             *
                                                                             (0x4000i32
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535i64)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255i32
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4i32 as
                                                                   libc::c_long
                                                               +
                                                               14i32 as
                                                                   libc::c_long
                                                               |
                                                               1i32 as
                                                                   libc::c_long)
                                                      -
                                                      2i32 as libc::c_long /
                                                          ((0x4000i32 as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1i64 >>
                                                                16i32 *
                                                                    (0x4000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535i64)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8i32 *
                                                                    (0x4000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1i64
                                                                         >>
                                                                         16i32
                                                                             *
                                                                             (0x4000i32
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535i64)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255i32
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2i32 as
                                                                   libc::c_long
                                                               +
                                                               1i32 as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,};
             init
         },
         {
             let mut init =
                 spiHardware_s{device: SPIDEV_3,
                               reg:
                                   (0x40000000i32 as
                                        uint32_t).wrapping_add(0x3c00i32 as
                                                                   libc::c_uint)
                                       as *mut SPI_TypeDef,
                               sckPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 3i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0i32 as ioTag_t,};
                                        init
                                    }],
                               misoPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 4i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0i32 as ioTag_t,};
                                        init
                                    }],
                               mosiPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            (1i32 + 1i32 <<
                                                                 4i32 | 5i32)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0i32 as ioTag_t,};
                                        init
                                    }],
                               af: 0x6i32 as uint8_t,
                               rcc:
                                   (((RCC_APB1 as libc::c_int) << 5i32) as
                                        libc::c_long |
                                        (16i32 *
                                             (0x8000i32 as uint32_t as
                                                  libc::c_long > 65535i64) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8i32 *
                                                  (0x8000i32 as uint32_t as
                                                       libc::c_long * 1i64 >>
                                                       16i32 *
                                                           (0x8000i32 as
                                                                uint32_t as
                                                                libc::c_long >
                                                                65535i64) as
                                                               libc::c_int >
                                                       255i32 as libc::c_long)
                                                      as libc::c_int) as
                                                 libc::c_long +
                                                 (8i32 as libc::c_long -
                                                      90i32 as libc::c_long /
                                                          ((0x8000i32 as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1i64 >>
                                                                16i32 *
                                                                    (0x8000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535i64)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8i32 *
                                                                    (0x8000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1i64
                                                                         >>
                                                                         16i32
                                                                             *
                                                                             (0x8000i32
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535i64)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255i32
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4i32 as
                                                                   libc::c_long
                                                               +
                                                               14i32 as
                                                                   libc::c_long
                                                               |
                                                               1i32 as
                                                                   libc::c_long)
                                                      -
                                                      2i32 as libc::c_long /
                                                          ((0x8000i32 as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1i64 >>
                                                                16i32 *
                                                                    (0x8000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535i64)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8i32 *
                                                                    (0x8000i32
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1i64
                                                                         >>
                                                                         16i32
                                                                             *
                                                                             (0x8000i32
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535i64)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255i32
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2i32 as
                                                                   libc::c_long
                                                               +
                                                               1i32 as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,};
             init
         }]
}
#[used]
#[cfg_attr ( target_os = "linux", link_section = ".init_array" )]
#[cfg_attr ( target_os = "windows", link_section = ".CRT$XIB" )]
#[cfg_attr ( target_os = "macos", link_section = "__DATA,__mod_init_func" )]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
