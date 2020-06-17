use core;
use libc;
extern "C" {
    #[no_mangle]
    static adcTagMap: [adcTagMap_t; 39];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
#[derive ( Copy, Clone )]
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
    /* !< ADC  Calibration Factors,                          Address offset: 0xB4 */
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct adcOperatingConfig_s {
    pub tag: ioTag_t,
    pub adcChannel: uint8_t,
    pub dmaIndex: uint8_t,
    pub enabled: bool,
    pub sampleTime: uint8_t,
}
pub type adcOperatingConfig_t = adcOperatingConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct adcTagMap_s {
    pub tag: ioTag_t,
    pub devices: uint8_t,
    pub channel: uint8_t,
}
pub type adcTagMap_t = adcTagMap_s;
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
//#define DEBUG_ADC_CHANNELS
#[no_mangle]
pub static mut adcOperatingConfig: [adcOperatingConfig_t; 4] =
    [adcOperatingConfig_t{tag: 0,
                          adcChannel: 0,
                          dmaIndex: 0,
                          enabled: false,
                          sampleTime: 0,}; 4];
#[no_mangle]
pub static mut adcValues: [uint16_t; 4] = [0; 4];
#[no_mangle]
pub unsafe extern "C" fn adcChannelByTag(mut ioTag: ioTag_t) -> uint8_t {
    let mut i: uint8_t = 0i32 as uint8_t;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[adcTagMap_t; 39]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<adcTagMap_t>()
                                                   as libc::c_ulong) {
        if ioTag as libc::c_int == adcTagMap[i as usize].tag as libc::c_int {
            return adcTagMap[i as usize].channel
        }
        i = i.wrapping_add(1)
    }
    return 0i32 as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn adcDeviceByInstance(mut instance: *mut ADC_TypeDef)
 -> ADCDevice {
    if instance ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef {
        return ADCDEV_1
    }
    if instance ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0x100i32
                                                                          as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef {
        return ADCDEV_2
    }
    if instance ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0x400i32
                                                                          as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef {
        return ADCDEV_3
    }
    if instance ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0x500i32
                                                                          as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef {
        return ADCDEV_4
    }
    return ADCINVALID;
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
// ADC1_INxx channel number
// index into DMA buffer in case of sparse channels
#[no_mangle]
pub unsafe extern "C" fn adcGetChannel(mut channel: uint8_t) -> uint16_t {
    return adcValues[adcOperatingConfig[channel as usize].dmaIndex as usize];
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
// F1 pins have uniform connection to ADC instances
// Encoding for adcTagMap_t.devices
// Verify a pin designated by tag has connection to an ADC instance designated by device
#[no_mangle]
pub unsafe extern "C" fn adcVerifyPin(mut tag: ioTag_t, mut device: ADCDevice)
 -> bool {
    if tag == 0 { return 0i32 != 0 }
    let mut map: libc::c_int = 0i32;
    while map < 39i32 {
        if adcTagMap[map as usize].tag as libc::c_int == tag as libc::c_int &&
               adcTagMap[map as usize].devices as libc::c_int &
                   1i32 << device as libc::c_int != 0 {
            return 1i32 != 0
        }
        map += 1
    }
    return 0i32 != 0;
}
