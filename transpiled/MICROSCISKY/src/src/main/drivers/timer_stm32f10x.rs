use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    system_stm32f10x.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Header File.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
    /* * @addtogroup CMSIS
  * @{
  */
    /* * @addtogroup stm32f10x_system
  * @{
  */
    /* *
  * @brief Define to prevent recursive inclusion
  */
    /* * @addtogroup STM32F10x_System_Includes
  * @{
  */
    /* *
  * @}
  */
    /* * @addtogroup STM32F10x_System_Exported_types
  * @{
  */
    #[no_mangle]
    static mut SystemCoreClock: uint32_t;
}
pub type IRQn = libc::c_int;
pub const USBWakeUp_IRQn: IRQn = 42;
pub const RTCAlarm_IRQn: IRQn = 41;
pub const EXTI15_10_IRQn: IRQn = 40;
pub const USART3_IRQn: IRQn = 39;
pub const USART2_IRQn: IRQn = 38;
pub const USART1_IRQn: IRQn = 37;
pub const SPI2_IRQn: IRQn = 36;
pub const SPI1_IRQn: IRQn = 35;
pub const I2C2_ER_IRQn: IRQn = 34;
pub const I2C2_EV_IRQn: IRQn = 33;
pub const I2C1_ER_IRQn: IRQn = 32;
pub const I2C1_EV_IRQn: IRQn = 31;
pub const TIM4_IRQn: IRQn = 30;
pub const TIM3_IRQn: IRQn = 29;
pub const TIM2_IRQn: IRQn = 28;
pub const TIM1_CC_IRQn: IRQn = 27;
pub const TIM1_TRG_COM_IRQn: IRQn = 26;
pub const TIM1_UP_IRQn: IRQn = 25;
pub const TIM1_BRK_IRQn: IRQn = 24;
pub const EXTI9_5_IRQn: IRQn = 23;
pub const CAN1_SCE_IRQn: IRQn = 22;
pub const CAN1_RX1_IRQn: IRQn = 21;
pub const USB_LP_CAN1_RX0_IRQn: IRQn = 20;
pub const USB_HP_CAN1_TX_IRQn: IRQn = 19;
pub const ADC1_2_IRQn: IRQn = 18;
pub const DMA1_Channel7_IRQn: IRQn = 17;
pub const DMA1_Channel6_IRQn: IRQn = 16;
pub const DMA1_Channel5_IRQn: IRQn = 15;
pub const DMA1_Channel4_IRQn: IRQn = 14;
pub const DMA1_Channel3_IRQn: IRQn = 13;
pub const DMA1_Channel2_IRQn: IRQn = 12;
pub const DMA1_Channel1_IRQn: IRQn = 11;
pub const EXTI4_IRQn: IRQn = 10;
pub const EXTI3_IRQn: IRQn = 9;
pub const EXTI2_IRQn: IRQn = 8;
pub const EXTI1_IRQn: IRQn = 7;
pub const EXTI0_IRQn: IRQn = 6;
pub const RCC_IRQn: IRQn = 5;
pub const FLASH_IRQn: IRQn = 4;
pub const RTC_IRQn: IRQn = 3;
pub const TAMPER_IRQn: IRQn = 2;
pub const PVD_IRQn: IRQn = 1;
pub const WWDG_IRQn: IRQn = 0;
pub const SysTick_IRQn: IRQn = -1;
pub const PendSV_IRQn: IRQn = -2;
pub const DebugMonitor_IRQn: IRQn = -4;
pub const SVCall_IRQn: IRQn = -5;
pub const UsageFault_IRQn: IRQn = -10;
pub const BusFault_IRQn: IRQn = -11;
pub const MemoryManagement_IRQn: IRQn = -12;
pub const NonMaskableInt_IRQn: IRQn = -14;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* * 
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SMCR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DIER: uint16_t,
    pub RESERVED3: uint16_t,
    pub SR: uint16_t,
    pub RESERVED4: uint16_t,
    pub EGR: uint16_t,
    pub RESERVED5: uint16_t,
    pub CCMR1: uint16_t,
    pub RESERVED6: uint16_t,
    pub CCMR2: uint16_t,
    pub RESERVED7: uint16_t,
    pub CCER: uint16_t,
    pub RESERVED8: uint16_t,
    pub CNT: uint16_t,
    pub RESERVED9: uint16_t,
    pub PSC: uint16_t,
    pub RESERVED10: uint16_t,
    pub ARR: uint16_t,
    pub RESERVED11: uint16_t,
    pub RCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub CCR1: uint16_t,
    pub RESERVED13: uint16_t,
    pub CCR2: uint16_t,
    pub RESERVED14: uint16_t,
    pub CCR3: uint16_t,
    pub RESERVED15: uint16_t,
    pub CCR4: uint16_t,
    pub RESERVED16: uint16_t,
    pub BDTR: uint16_t,
    pub RESERVED17: uint16_t,
    pub DCR: uint16_t,
    pub RESERVED18: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED19: uint16_t,
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
pub type rccPeriphTag_t = uint8_t;
pub type rcc_reg = libc::c_uint;
pub const RCC_AHB1: rcc_reg = 4;
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerDef_s {
    pub TIMx: *mut TIM_TypeDef,
    pub rcc: rccPeriphTag_t,
    pub inputIrq: uint8_t,
}
pub type timerDef_t = timerDef_s;
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
pub static mut timerDefinitions: [timerDef_t; 4] =
    [timerDef_t{TIMx: 0 as *mut TIM_TypeDef, rcc: 0, inputIrq: 0,}; 4];
#[no_mangle]
pub unsafe extern "C" fn timerClock(mut tim: *mut TIM_TypeDef) -> uint32_t {
    return SystemCoreClock;
}
unsafe extern "C" fn run_static_initializers() {
    timerDefinitions =
        [{
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x10000 as
                                                                libc::c_int as
                                                                libc::c_uint).wrapping_add(0x2c00
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB2 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (0x800 as libc::c_int as uint32_t as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (0x800 as libc::c_int as
                                                    uint32_t as libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (0x800 as libc::c_int
                                                             as uint32_t as
                                                             libc::c_long >
                                                             65535 as
                                                                 libc::c_long)
                                                            as libc::c_int >
                                                    255 as libc::c_int as
                                                        libc::c_long) as
                                                   libc::c_int) as
                                              libc::c_long +
                                              (8 as libc::c_int as
                                                   libc::c_long -
                                                   90 as libc::c_int as
                                                       libc::c_long /
                                                       ((0x800 as libc::c_int
                                                             as uint32_t as
                                                             libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (0x800 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (0x800 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (0x800
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint32_t
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            4 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            14 as libc::c_int
                                                                as
                                                                libc::c_long |
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long)
                                                   -
                                                   2 as libc::c_int as
                                                       libc::c_long /
                                                       ((0x800 as libc::c_int
                                                             as uint32_t as
                                                             libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (0x800 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (0x800 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (0x800
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint32_t
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            2 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long))))
                                    as rccPeriphTag_t,
                            inputIrq:
                                TIM1_CC_IRQn as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0 as libc::c_int
                                                                as
                                                                libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (0x1 as libc::c_int as uint32_t as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (0x1 as libc::c_int as uint32_t
                                                    as libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (0x1 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long >
                                                             65535 as
                                                                 libc::c_long)
                                                            as libc::c_int >
                                                    255 as libc::c_int as
                                                        libc::c_long) as
                                                   libc::c_int) as
                                              libc::c_long +
                                              (8 as libc::c_int as
                                                   libc::c_long -
                                                   90 as libc::c_int as
                                                       libc::c_long /
                                                       ((0x1 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (0x1 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (0x1 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (0x1
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint32_t
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            4 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            14 as libc::c_int
                                                                as
                                                                libc::c_long |
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long)
                                                   -
                                                   2 as libc::c_int as
                                                       libc::c_long /
                                                       ((0x1 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (0x1 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (0x1 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (0x1
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint32_t
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            2 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long))))
                                    as rccPeriphTag_t,
                            inputIrq: TIM2_IRQn as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x400 as
                                                                libc::c_int as
                                                                libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (0x2 as libc::c_int as uint32_t as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (0x2 as libc::c_int as uint32_t
                                                    as libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (0x2 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long >
                                                             65535 as
                                                                 libc::c_long)
                                                            as libc::c_int >
                                                    255 as libc::c_int as
                                                        libc::c_long) as
                                                   libc::c_int) as
                                              libc::c_long +
                                              (8 as libc::c_int as
                                                   libc::c_long -
                                                   90 as libc::c_int as
                                                       libc::c_long /
                                                       ((0x2 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (0x2 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (0x2 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (0x2
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint32_t
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            4 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            14 as libc::c_int
                                                                as
                                                                libc::c_long |
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long)
                                                   -
                                                   2 as libc::c_int as
                                                       libc::c_long /
                                                       ((0x2 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (0x2 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (0x2 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (0x2
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint32_t
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            2 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long))))
                                    as rccPeriphTag_t,
                            inputIrq: TIM3_IRQn as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x800 as
                                                                libc::c_int as
                                                                libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (0x4 as libc::c_int as uint32_t as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (0x4 as libc::c_int as uint32_t
                                                    as libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (0x4 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long >
                                                             65535 as
                                                                 libc::c_long)
                                                            as libc::c_int >
                                                    255 as libc::c_int as
                                                        libc::c_long) as
                                                   libc::c_int) as
                                              libc::c_long +
                                              (8 as libc::c_int as
                                                   libc::c_long -
                                                   90 as libc::c_int as
                                                       libc::c_long /
                                                       ((0x4 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (0x4 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (0x4 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (0x4
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint32_t
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            4 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            14 as libc::c_int
                                                                as
                                                                libc::c_long |
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long)
                                                   -
                                                   2 as libc::c_int as
                                                       libc::c_long /
                                                       ((0x4 as libc::c_int as
                                                             uint32_t as
                                                             libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (0x4 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (0x4 as
                                                                      libc::c_int
                                                                      as
                                                                      uint32_t
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (0x4
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint32_t
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            2 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long))))
                                    as rccPeriphTag_t,
                            inputIrq: TIM4_IRQn as libc::c_int as uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
