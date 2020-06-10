use ::libc;
extern "C" {
    #[no_mangle]
    static adcTagMap: [adcTagMap_t; 10];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* !< STM32F10x Standard Peripheral Library old definitions (maintained for legacy purpose) */
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
    pub SR: uint32_t,
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMPR1: uint32_t,
    pub SMPR2: uint32_t,
    pub JOFR1: uint32_t,
    pub JOFR2: uint32_t,
    pub JOFR3: uint32_t,
    pub JOFR4: uint32_t,
    pub HTR: uint32_t,
    pub LTR: uint32_t,
    pub SQR1: uint32_t,
    pub SQR2: uint32_t,
    pub SQR3: uint32_t,
    pub JSQR: uint32_t,
    pub JDR1: uint32_t,
    pub JDR2: uint32_t,
    pub JDR3: uint32_t,
    pub JDR4: uint32_t,
    pub DR: uint32_t,
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
pub type ADCDevice = libc::c_int;
pub const ADCDEV_COUNT: ADCDevice = 1;
pub const ADCDEV_1: ADCDevice = 0;
pub const ADCINVALID: ADCDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcOperatingConfig_s {
    pub tag: ioTag_t,
    pub adcChannel: uint8_t,
    pub dmaIndex: uint8_t,
    pub enabled: bool,
    pub sampleTime: uint8_t,
}
pub type adcOperatingConfig_t = adcOperatingConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcTagMap_s {
    pub tag: ioTag_t,
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
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[adcTagMap_t; 10]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<adcTagMap_t>()
                                                   as libc::c_ulong) {
        if ioTag as libc::c_int == adcTagMap[i as usize].tag as libc::c_int {
            return adcTagMap[i as usize].channel
        }
        i = i.wrapping_add(1)
    }
    return 0 as libc::c_int as uint8_t;
}
// F1 pins have uniform connection to ADC instances
// Encoding for adcTagMap_t.devices
#[no_mangle]
pub unsafe extern "C" fn adcDeviceByInstance(mut instance: *mut ADC_TypeDef)
 -> ADCDevice {
    if instance ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2400
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef {
        return ADCDEV_1
    }
    return ADCINVALID;
}
#[no_mangle]
pub unsafe extern "C" fn adcGetChannel(mut channel: uint8_t) -> uint16_t {
    return adcValues[adcOperatingConfig[channel as usize].dmaIndex as usize];
}
// Verify a pin designated by tag has connection to an ADC instance designated by device
#[no_mangle]
pub unsafe extern "C" fn adcVerifyPin(mut tag: ioTag_t, mut device: ADCDevice)
 -> bool {
    if tag == 0 { return 0 as libc::c_int != 0 }
    let mut map: libc::c_int = 0 as libc::c_int;
    while map < 10 as libc::c_int {
        if adcTagMap[map as usize].tag as libc::c_int == tag as libc::c_int {
            return 1 as libc::c_int != 0
        }
        map += 1
    }
    return 0 as libc::c_int != 0;
}
