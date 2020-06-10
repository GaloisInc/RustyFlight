use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
/* *
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint32_t,
    pub ARR: uint32_t,
    pub RCR: uint32_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint32_t,
    pub DMAR: uint32_t,
    pub OR: uint32_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
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
pub type C2RustUnnamed = libc::c_uint;
pub const DMA_LAST_HANDLER: C2RustUnnamed = 16;
pub const DMA2_ST7_HANDLER: C2RustUnnamed = 16;
pub const DMA2_ST6_HANDLER: C2RustUnnamed = 15;
pub const DMA2_ST5_HANDLER: C2RustUnnamed = 14;
pub const DMA2_ST4_HANDLER: C2RustUnnamed = 13;
pub const DMA2_ST3_HANDLER: C2RustUnnamed = 12;
pub const DMA2_ST2_HANDLER: C2RustUnnamed = 11;
pub const DMA2_ST1_HANDLER: C2RustUnnamed = 10;
pub const DMA2_ST0_HANDLER: C2RustUnnamed = 9;
pub const DMA1_ST7_HANDLER: C2RustUnnamed = 8;
pub const DMA1_ST6_HANDLER: C2RustUnnamed = 7;
pub const DMA1_ST5_HANDLER: C2RustUnnamed = 6;
pub const DMA1_ST4_HANDLER: C2RustUnnamed = 5;
pub const DMA1_ST3_HANDLER: C2RustUnnamed = 4;
pub const DMA1_ST2_HANDLER: C2RustUnnamed = 3;
pub const DMA1_ST1_HANDLER: C2RustUnnamed = 2;
pub const DMA1_ST0_HANDLER: C2RustUnnamed = 1;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Stream_TypeDef,
    pub dmaChannel: uint32_t,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Stream_TypeDef,
    pub dmaTimUPChannel: uint32_t,
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
// Initialized in run_static_initializers
#[no_mangle]
pub static mut timerHardware: [timerHardware_t; 8] =
    [timerHardware_t{tim: 0 as *mut TIM_TypeDef,
                     tag: 0,
                     channel: 0,
                     usageFlags: TIM_USE_ANY,
                     output: 0,
                     alternateFunction: 0,
                     dmaRef: 0 as *mut DMA_Stream_TypeDef,
                     dmaChannel: 0,
                     dmaIrqHandler: 0,
                     dmaTimUPRef: 0 as *mut DMA_Stream_TypeDef,
                     dmaTimUPChannel: 0,
                     dmaTimUPIrqHandler: 0,}; 8];
unsafe extern "C" fn run_static_initializers() {
    timerHardware =
        [{
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x10000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0
                                                                                                        as
                                                                                                        libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((4 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          13 as libc::c_int) as ioTag_t,
                                 channel: 0x8 as libc::c_uint as uint8_t,
                                 usageFlags: TIM_USE_PPM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_uint as uint8_t,
                                 dmaRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6400
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0xa0
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaChannel: 0xc000000 as libc::c_uint,
                                 dmaIrqHandler:
                                     DMA2_ST6_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6400
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x88
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaTimUPChannel: 0xc000000 as libc::c_uint,
                                 dmaTimUPIrqHandler:
                                     DMA2_ST5_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x400 as
                                                                         libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 0 as libc::c_int)
                                         as ioTag_t,
                                 channel: 0x8 as libc::c_uint as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_uint as uint8_t,
                                 dmaRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0xb8
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaChannel: 0xa000000 as libc::c_uint,
                                 dmaIrqHandler:
                                     DMA1_ST7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x40
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaTimUPChannel: 0xa000000 as libc::c_uint,
                                 dmaTimUPIrqHandler:
                                     DMA1_ST2_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x400 as
                                                                         libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 1 as libc::c_int)
                                         as ioTag_t,
                                 channel: 0xc as libc::c_uint as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_uint as uint8_t,
                                 dmaRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x40
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaChannel: 0xa000000 as libc::c_uint,
                                 dmaIrqHandler:
                                     DMA1_ST2_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x40
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaTimUPChannel: 0xa000000 as libc::c_uint,
                                 dmaTimUPIrqHandler:
                                     DMA1_ST2_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x10000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0
                                                                                                        as
                                                                                                        libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((4 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 9 as libc::c_int)
                                         as ioTag_t,
                                 channel: 0 as libc::c_uint as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_uint as uint8_t,
                                 dmaRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6400
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x58
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaChannel: 0xc000000 as libc::c_uint,
                                 dmaIrqHandler:
                                     DMA2_ST3_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6400
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x88
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaTimUPChannel: 0xc000000 as libc::c_uint,
                                 dmaTimUPIrqHandler:
                                     DMA2_ST5_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x10000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0
                                                                                                        as
                                                                                                        libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((4 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          11 as libc::c_int) as ioTag_t,
                                 channel: 0x4 as libc::c_uint as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_uint as uint8_t,
                                 dmaRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6400
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x40
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaChannel: 0xc000000 as libc::c_uint,
                                 dmaIrqHandler:
                                     DMA2_ST2_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6400
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x88
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaTimUPChannel: 0xc000000 as libc::c_uint,
                                 dmaTimUPIrqHandler:
                                     DMA2_ST5_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x10000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x400
                                                                                                        as
                                                                                                        libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 9 as libc::c_int)
                                         as ioTag_t,
                                 channel: 0xc as libc::c_uint as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x3 as libc::c_uint as uint8_t,
                                 dmaRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6400
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0xb8
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaChannel: 0xe000000 as libc::c_uint,
                                 dmaIrqHandler:
                                     DMA2_ST7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6400
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x28
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaTimUPChannel: 0xe000000 as libc::c_uint,
                                 dmaTimUPIrqHandler:
                                     DMA2_ST1_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0xc00 as
                                                                         libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 3 as libc::c_int)
                                         as ioTag_t,
                                 channel: 0xc as libc::c_uint as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_uint as uint8_t,
                                 dmaRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x28
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaChannel: 0xc000000 as libc::c_uint,
                                 dmaIrqHandler:
                                     DMA1_ST1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x10
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaTimUPChannel: 0xc000000 as libc::c_uint,
                                 dmaTimUPIrqHandler:
                                     DMA1_ST0_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x800 as
                                                                         libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((3 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          12 as libc::c_int) as ioTag_t,
                                 channel: 0 as libc::c_uint as uint8_t,
                                 usageFlags: TIM_USE_LED,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_uint as uint8_t,
                                 dmaRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0x10
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaChannel: 0x4000000 as libc::c_uint,
                                 dmaIrqHandler:
                                     DMA1_ST0_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as
                                          libc::c_uint).wrapping_add(0x20000
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x6000
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_add(0xa0
                                                                                                                                       as
                                                                                                                                       libc::c_uint)
                                         as *mut DMA_Stream_TypeDef,
                                 dmaTimUPChannel: 0x4000000 as libc::c_uint,
                                 dmaTimUPIrqHandler:
                                     DMA1_ST6_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
