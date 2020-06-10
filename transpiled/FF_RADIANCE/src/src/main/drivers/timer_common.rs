use ::libc;
extern "C" {
    #[no_mangle]
    static timerHardware: [timerHardware_t; 0];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint16_t,
    pub RESERVED9: uint16_t,
    pub ARR: uint32_t,
    pub RCR: uint16_t,
    pub RESERVED10: uint16_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED13: uint16_t,
    pub OR: uint16_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
}
pub type ioTag_t = uint8_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Channel_TypeDef,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Channel_TypeDef,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
// TIMUP
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
pub unsafe extern "C" fn timerIndexByTag(mut ioTag: ioTag_t) -> uint8_t {
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerGetByTag(mut ioTag: ioTag_t)
 -> *const timerHardware_t {
    if ioTag == 0 { return 0 as *const timerHardware_t }
    let mut timerIndex: uint8_t = timerIndexByTag(ioTag);
    let mut index: uint8_t = 1 as libc::c_int as uint8_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 9 as libc::c_int {
        if (*timerHardware.as_ptr().offset(i as isize)).tag as libc::c_int ==
               ioTag as libc::c_int {
            if index as libc::c_int == timerIndex as libc::c_int ||
                   timerIndex as libc::c_int == 0 as libc::c_int {
                return &*timerHardware.as_ptr().offset(i as isize) as
                           *const timerHardware_t
            }
            index = index.wrapping_add(1)
        }
        i += 1
    }
    return 0 as *const timerHardware_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerioTagGetByUsage(mut usageFlag: timerUsageFlag_e,
                                              mut index: uint8_t) -> ioTag_t {
    let mut currentIndex: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 9 as libc::c_int {
        if (*timerHardware.as_ptr().offset(i as isize)).usageFlags as
               libc::c_uint & usageFlag as libc::c_uint ==
               usageFlag as libc::c_uint {
            if currentIndex as libc::c_int == index as libc::c_int {
                return (*timerHardware.as_ptr().offset(i as isize)).tag
            }
            currentIndex = currentIndex.wrapping_add(1)
        }
        i += 1
    }
    return 0 as libc::c_int as ioTag_t;
}
