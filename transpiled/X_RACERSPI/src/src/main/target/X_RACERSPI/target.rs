use ::libc;
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
/* *
  * @brief Independent WATCHDOG
  */
/* !< IWDG Key register,       Address offset: 0x00 */
/* !< IWDG Prescaler register, Address offset: 0x04 */
/* !< IWDG Reload register,    Address offset: 0x08 */
/* !< IWDG Status register,    Address offset: 0x0C */
/* !< IWDG Window register,    Address offset: 0x10 */
/* *
  * @brief Power Control
  */
/* !< PWR power control register,        Address offset: 0x00 */
/* !< PWR power control/status register, Address offset: 0x04 */
/* *
  * @brief Reset and Clock Control
  */
/* !< RCC clock control register,                                  Address offset: 0x00 */
/* !< RCC clock configuration register,                            Address offset: 0x04 */
/* !< RCC clock interrupt register,                                Address offset: 0x08 */
/* !< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
/* !< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
/* !< RCC AHB peripheral clock register,                           Address offset: 0x14 */
/* !< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
/* !< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
/* !< RCC Backup domain control register,                          Address offset: 0x20 */
/* !< RCC clock control & status register,                         Address offset: 0x24 */
/* !< RCC AHB peripheral reset register,                           Address offset: 0x28 */
/* !< RCC clock configuration register 2,                          Address offset: 0x2C */
/* !< RCC clock configuration register 3,                          Address offset: 0x30 */
/* *
  * @brief Real-Time Clock
  */
/* !< RTC time register,                                        Address offset: 0x00 */
/* !< RTC date register,                                        Address offset: 0x04 */
/* !< RTC control register,                                     Address offset: 0x08 */
/* !< RTC initialization and status register,                   Address offset: 0x0C */
/* !< RTC prescaler register,                                   Address offset: 0x10 */
/* !< RTC wakeup timer register,                                Address offset: 0x14 */
/* !< Reserved, 0x18                                                                 */
/* !< RTC alarm A register,                                     Address offset: 0x1C */
/* !< RTC alarm B register,                                     Address offset: 0x20 */
/* !< RTC write protection register,                            Address offset: 0x24 */
/* !< RTC sub second register,                                  Address offset: 0x28 */
/* !< RTC shift control register,                               Address offset: 0x2C */
/* !< RTC time stamp time register,                             Address offset: 0x30 */
/* !< RTC time stamp date register,                             Address offset: 0x34 */
/* !< RTC time-stamp sub second register,                       Address offset: 0x38 */
/* !< RTC calibration register,                                 Address offset: 0x3C */
/* !< RTC tamper and alternate function configuration register, Address offset: 0x40 */
/* !< RTC alarm A sub second register,                          Address offset: 0x44 */
/* !< RTC alarm B sub second register,                          Address offset: 0x48 */
/* !< Reserved, 0x4C                                                                 */
/* !< RTC backup register 0,                                    Address offset: 0x50 */
/* !< RTC backup register 1,                                    Address offset: 0x54 */
/* !< RTC backup register 2,                                    Address offset: 0x58 */
/* !< RTC backup register 3,                                    Address offset: 0x5C */
/* !< RTC backup register 4,                                    Address offset: 0x60 */
/* !< RTC backup register 5,                                    Address offset: 0x64 */
/* !< RTC backup register 6,                                    Address offset: 0x68 */
/* !< RTC backup register 7,                                    Address offset: 0x6C */
/* !< RTC backup register 8,                                    Address offset: 0x70 */
/* !< RTC backup register 9,                                    Address offset: 0x74 */
/* !< RTC backup register 10,                                   Address offset: 0x78 */
/* !< RTC backup register 11,                                   Address offset: 0x7C */
/* !< RTC backup register 12,                                   Address offset: 0x80 */
/* !< RTC backup register 13,                                   Address offset: 0x84 */
/* !< RTC backup register 14,                                   Address offset: 0x88 */
/* !< RTC backup register 15,                                   Address offset: 0x8C */
/* *
  * @brief Serial Peripheral Interface
  */
/* !< SPI Control register 1 (not used in I2S mode),       Address offset: 0x00 */
/* !< Reserved, 0x02                                                            */
/* !< SPI Control register 2,                              Address offset: 0x04 */
/* !< Reserved, 0x06                                                            */
/* !< SPI Status register,                                 Address offset: 0x08 */
/* !< Reserved, 0x0A                                                            */
/* !< SPI data register,                                   Address offset: 0x0C */
/* !< Reserved, 0x0E                                                            */
/* !< SPI CRC polynomial register (not used in I2S mode),  Address offset: 0x10 */
/* !< Reserved, 0x12                                                            */
/* !< SPI Rx CRC register (not used in I2S mode),          Address offset: 0x14 */
/* !< Reserved, 0x16                                                            */
/* !< SPI Tx CRC register (not used in I2S mode),          Address offset: 0x18 */
/* !< Reserved, 0x1A                                                            */
/* !< SPI_I2S configuration register,                      Address offset: 0x1C */
/* !< Reserved, 0x1E                                                            */
/* !< SPI_I2S prescaler register,                          Address offset: 0x20 */
/* !< Reserved, 0x22                                                            */
/* *
  * @brief TIM
  */
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
pub type C2RustUnnamed = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed = 0;
pub const DMA1_CH5_HANDLER: C2RustUnnamed_0 = 5;
pub const DMA1_CH2_HANDLER: C2RustUnnamed_0 = 2;
pub const DMA2_CH1_HANDLER: C2RustUnnamed_0 = 8;
pub const DMA1_CH7_HANDLER: C2RustUnnamed_0 = 7;
pub const DMA1_CH4_HANDLER: C2RustUnnamed_0 = 4;
pub const DMA1_CH1_HANDLER: C2RustUnnamed_0 = 1;
pub const DMA1_CH3_HANDLER: C2RustUnnamed_0 = 3;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const DMA_LAST_HANDLER: C2RustUnnamed_0 = 12;
pub const DMA2_CH5_HANDLER: C2RustUnnamed_0 = 12;
pub const DMA2_CH4_HANDLER: C2RustUnnamed_0 = 11;
pub const DMA2_CH3_HANDLER: C2RustUnnamed_0 = 10;
pub const DMA2_CH2_HANDLER: C2RustUnnamed_0 = 9;
pub const DMA1_CH6_HANDLER: C2RustUnnamed_0 = 6;
pub const DMA_NONE: C2RustUnnamed_0 = 0;
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 0 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags:
                                     (TIM_USE_PWM as libc::c_int |
                                          TIM_USE_PPM as libc::c_int) as
                                         timerUsageFlag_e,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 1 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x4 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          11 as libc::c_int) as ioTag_t,
                                 channel:
                                     0xc as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          10 as libc::c_int) as ioTag_t,
                                 channel:
                                     0x8 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x8
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x400 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 0 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x8 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x400 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 1 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0xc as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_PWM,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x4400
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 6 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x4800
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 7 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x800 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          11 as libc::c_int) as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0xa as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x8
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x800 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          12 as libc::c_int) as ioTag_t,
                                 channel:
                                     0x4 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0xa as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x44
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH4_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x800 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 8 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x8 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x3400
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 9 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x8 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0xa as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x408
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA2_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x408
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA2_CH1_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x4000
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 2 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x9 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x4000
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 3 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x4 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x9 as libc::c_int as uint8_t,
                                 dmaRef: 0 as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler: 0 as libc::c_int as uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                    as
                                                                                                    libc::c_int
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
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x2c00
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 8 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags:
                                     (TIM_USE_MOTOR as libc::c_int |
                                          TIM_USE_LED as libc::c_int) as
                                         timerUsageFlag_e,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x6 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         timerHardware_t{tim: 0 as *mut TIM_TypeDef,
                         tag: 0,
                         channel: 0,
                         usageFlags: TIM_USE_ANY,
                         output: 0,
                         alternateFunction: 0,
                         dmaRef: 0 as *mut DMA_Channel_TypeDef,
                         dmaIrqHandler: 0,
                         dmaTimUPRef: 0 as *mut DMA_Channel_TypeDef,
                         dmaTimUPIrqHandler: 0,},
         timerHardware_t{tim: 0 as *mut TIM_TypeDef,
                         tag: 0,
                         channel: 0,
                         usageFlags: TIM_USE_ANY,
                         output: 0,
                         alternateFunction: 0,
                         dmaRef: 0 as *mut DMA_Channel_TypeDef,
                         dmaIrqHandler: 0,
                         dmaTimUPRef: 0 as *mut DMA_Channel_TypeDef,
                         dmaTimUPIrqHandler: 0,}]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
