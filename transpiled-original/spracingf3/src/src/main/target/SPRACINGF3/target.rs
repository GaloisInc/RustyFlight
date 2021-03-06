use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* * 
  * @brief DMA Controller
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
    /* !< DMA channel x memory address register                                          */
}
/* *
  * @brief TIM
  */
#[derive ( Copy, Clone )]
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
    /* !< TIM capture/compare register 4,      Address offset: 0x5C */
}
pub type ioTag_t = uint8_t;
pub type C2RustUnnamed = libc::c_uint;
pub const DMA_LAST_HANDLER: C2RustUnnamed = 12;
pub const DMA2_CH5_HANDLER: C2RustUnnamed = 12;
pub const DMA2_CH4_HANDLER: C2RustUnnamed = 11;
pub const DMA2_CH3_HANDLER: C2RustUnnamed = 10;
pub const DMA2_CH2_HANDLER: C2RustUnnamed = 9;
pub const DMA2_CH1_HANDLER: C2RustUnnamed = 8;
pub const DMA1_CH7_HANDLER: C2RustUnnamed = 7;
pub const DMA1_CH6_HANDLER: C2RustUnnamed = 6;
pub const DMA1_CH5_HANDLER: C2RustUnnamed = 5;
pub const DMA1_CH4_HANDLER: C2RustUnnamed = 4;
pub const DMA1_CH3_HANDLER: C2RustUnnamed = 3;
pub const DMA1_CH2_HANDLER: C2RustUnnamed = 2;
pub const DMA1_CH1_HANDLER: C2RustUnnamed = 1;
pub const DMA_NONE: C2RustUnnamed = 0;
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
#[derive ( Copy, Clone )]
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed_0 = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed_0 = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed_0 = 0;
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
// Production boards swapped RC_CH3/4 swapped to make it easierTIM_USE_MOTOR,  to using supplied cables - compared to first prototype.
// Initialized in run_static_initializers
#[no_mangle]
pub static mut timerHardware: [timerHardware_t; 17] =
    [timerHardware_t{tim: 0 as *mut TIM_TypeDef,
                     tag: 0,
                     channel: 0,
                     usageFlags: TIM_USE_ANY,
                     output: 0,
                     alternateFunction: 0,
                     dmaRef: 0 as *mut DMA_Channel_TypeDef,
                     dmaIrqHandler: 0,
                     dmaTimUPRef: 0 as *mut DMA_Channel_TypeDef,
                     dmaTimUPIrqHandler: 0,}; 17];
unsafe extern "C" fn run_static_initializers() {
    timerHardware =
        [{
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (0i32 + 1i32 << 4i32 | 0i32) as ioTag_t,
                                 channel: 0i32 as uint16_t as uint8_t,
                                 usageFlags:
                                     (TIM_USE_PWM as libc::c_int |
                                          TIM_USE_PPM as libc::c_int) as
                                         timerUsageFlag_e,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x1i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x58i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x1ci32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (0i32 + 1i32 << 4i32 | 1i32) as ioTag_t,
                                 channel: 0x4i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x1i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x80i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x1ci32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     (1i32 + 1i32 << 4i32 | 11i32) as ioTag_t,
                                 channel: 0xci32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x1i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x80i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x1ci32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     (1i32 + 1i32 << 4i32 | 10i32) as ioTag_t,
                                 channel: 0x8i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x1i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x8i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x1ci32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x400i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (1i32 + 1i32 << 4i32 | 4i32) as ioTag_t,
                                 channel: 0i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x2i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x6ci32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH6_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x30i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x400i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (1i32 + 1i32 << 4i32 | 5i32) as ioTag_t,
                                 channel: 0x4i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x2i32 as uint8_t,
                                 dmaRef: 0 as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler: 0i32 as uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x30i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x400i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (1i32 + 1i32 << 4i32 | 0i32) as ioTag_t,
                                 channel: 0x8i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x2i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x1ci32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x30i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x400i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (1i32 + 1i32 << 4i32 | 1i32) as ioTag_t,
                                 channel: 0xci32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x2i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x30i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x30i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x10000i32 as
                                                                     libc::c_uint).wrapping_add(0x4400i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (0i32 + 1i32 << 4i32 | 6i32) as ioTag_t,
                                 channel: 0i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x1i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x30i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x30i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x10000i32 as
                                                                     libc::c_uint).wrapping_add(0x4800i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (0i32 + 1i32 << 4i32 | 7i32) as ioTag_t,
                                 channel: 0i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x1i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x80i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x80i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x800i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     (0i32 + 1i32 << 4i32 | 11i32) as ioTag_t,
                                 channel: 0i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0xai32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x8i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x80i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x800i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     (0i32 + 1i32 << 4i32 | 12i32) as ioTag_t,
                                 channel: 0x4i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0xai32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x44i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH4_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x80i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x800i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (1i32 + 1i32 << 4i32 | 8i32) as ioTag_t,
                                 channel: 0x8i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x2i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x58i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x80i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x800i32 as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (1i32 + 1i32 << 4i32 | 9i32) as ioTag_t,
                                 channel: 0xci32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x2i32 as uint8_t,
                                 dmaRef: 0 as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler: 0i32 as uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x80i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x10000i32 as
                                                                     libc::c_uint).wrapping_add(0x4000i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (0i32 + 1i32 << 4i32 | 2i32) as ioTag_t,
                                 channel: 0i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x9i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x58i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x58i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x10000i32 as
                                                                     libc::c_uint).wrapping_add(0x4000i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (0i32 + 1i32 << 4i32 | 3i32) as ioTag_t,
                                 channel: 0x4i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x9i32 as uint8_t,
                                 dmaRef: 0 as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler: 0i32 as uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x58i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x10000i32 as
                                                                     libc::c_uint).wrapping_add(0x2c00i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag: (0i32 + 1i32 << 4i32 | 8i32) as ioTag_t,
                                 channel: 0i32 as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_LED,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int | 0i32)
                                         as uint8_t,
                                 alternateFunction: 0x6i32 as uint8_t,
                                 dmaRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x1ci32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000i32 as
                                          uint32_t).wrapping_add(0x20000i32 as
                                                                     libc::c_uint).wrapping_add(0x58i32
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr ( target_os = "linux", link_section = ".init_array" )]
#[cfg_attr ( target_os = "windows", link_section = ".CRT$XIB" )]
#[cfg_attr ( target_os = "macos", link_section = "__DATA,__mod_init_func" )]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
