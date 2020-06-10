use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    system_stm32f7xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   CMSIS Cortex-M7 Device System Source File for STM32F7xx devices.       
  ******************************************************************************  
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */
    /* * @addtogroup CMSIS
  * @{
  */
    /* * @addtogroup stm32f7xx_system
  * @{
  */
    /* *
  * @brief Define to prevent recursive inclusion
  */
    /* * @addtogroup STM32F7xx_System_Includes
  * @{
  */
    /* *
  * @}
  */
    /* * @addtogroup STM32F7xx_System_Exported_Variables
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
    */
    #[no_mangle]
    static mut SystemCoreClock: uint32_t;
}
pub type C2RustUnnamed = libc::c_int;
pub const SPDIF_RX_IRQn: C2RustUnnamed = 97;
pub const I2C4_ER_IRQn: C2RustUnnamed = 96;
pub const I2C4_EV_IRQn: C2RustUnnamed = 95;
pub const CEC_IRQn: C2RustUnnamed = 94;
pub const LPTIM1_IRQn: C2RustUnnamed = 93;
pub const QUADSPI_IRQn: C2RustUnnamed = 92;
pub const SAI2_IRQn: C2RustUnnamed = 91;
pub const DMA2D_IRQn: C2RustUnnamed = 90;
pub const LTDC_ER_IRQn: C2RustUnnamed = 89;
pub const LTDC_IRQn: C2RustUnnamed = 88;
pub const SAI1_IRQn: C2RustUnnamed = 87;
pub const SPI6_IRQn: C2RustUnnamed = 86;
pub const SPI5_IRQn: C2RustUnnamed = 85;
pub const SPI4_IRQn: C2RustUnnamed = 84;
pub const UART8_IRQn: C2RustUnnamed = 83;
pub const UART7_IRQn: C2RustUnnamed = 82;
pub const FPU_IRQn: C2RustUnnamed = 81;
pub const RNG_IRQn: C2RustUnnamed = 80;
pub const DCMI_IRQn: C2RustUnnamed = 78;
pub const OTG_HS_IRQn: C2RustUnnamed = 77;
pub const OTG_HS_WKUP_IRQn: C2RustUnnamed = 76;
pub const OTG_HS_EP1_IN_IRQn: C2RustUnnamed = 75;
pub const OTG_HS_EP1_OUT_IRQn: C2RustUnnamed = 74;
pub const I2C3_ER_IRQn: C2RustUnnamed = 73;
pub const I2C3_EV_IRQn: C2RustUnnamed = 72;
pub const USART6_IRQn: C2RustUnnamed = 71;
pub const DMA2_Stream7_IRQn: C2RustUnnamed = 70;
pub const DMA2_Stream6_IRQn: C2RustUnnamed = 69;
pub const DMA2_Stream5_IRQn: C2RustUnnamed = 68;
pub const OTG_FS_IRQn: C2RustUnnamed = 67;
pub const CAN2_SCE_IRQn: C2RustUnnamed = 66;
pub const CAN2_RX1_IRQn: C2RustUnnamed = 65;
pub const CAN2_RX0_IRQn: C2RustUnnamed = 64;
pub const CAN2_TX_IRQn: C2RustUnnamed = 63;
pub const ETH_WKUP_IRQn: C2RustUnnamed = 62;
pub const ETH_IRQn: C2RustUnnamed = 61;
pub const DMA2_Stream4_IRQn: C2RustUnnamed = 60;
pub const DMA2_Stream3_IRQn: C2RustUnnamed = 59;
pub const DMA2_Stream2_IRQn: C2RustUnnamed = 58;
pub const DMA2_Stream1_IRQn: C2RustUnnamed = 57;
pub const DMA2_Stream0_IRQn: C2RustUnnamed = 56;
pub const TIM7_IRQn: C2RustUnnamed = 55;
pub const TIM6_DAC_IRQn: C2RustUnnamed = 54;
pub const UART5_IRQn: C2RustUnnamed = 53;
pub const UART4_IRQn: C2RustUnnamed = 52;
pub const SPI3_IRQn: C2RustUnnamed = 51;
pub const TIM5_IRQn: C2RustUnnamed = 50;
pub const SDMMC1_IRQn: C2RustUnnamed = 49;
pub const FMC_IRQn: C2RustUnnamed = 48;
pub const DMA1_Stream7_IRQn: C2RustUnnamed = 47;
pub const TIM8_CC_IRQn: C2RustUnnamed = 46;
pub const TIM8_TRG_COM_TIM14_IRQn: C2RustUnnamed = 45;
pub const TIM8_UP_TIM13_IRQn: C2RustUnnamed = 44;
pub const TIM8_BRK_TIM12_IRQn: C2RustUnnamed = 43;
pub const OTG_FS_WKUP_IRQn: C2RustUnnamed = 42;
pub const RTC_Alarm_IRQn: C2RustUnnamed = 41;
pub const EXTI15_10_IRQn: C2RustUnnamed = 40;
pub const USART3_IRQn: C2RustUnnamed = 39;
pub const USART2_IRQn: C2RustUnnamed = 38;
pub const USART1_IRQn: C2RustUnnamed = 37;
pub const SPI2_IRQn: C2RustUnnamed = 36;
pub const SPI1_IRQn: C2RustUnnamed = 35;
pub const I2C2_ER_IRQn: C2RustUnnamed = 34;
pub const I2C2_EV_IRQn: C2RustUnnamed = 33;
pub const I2C1_ER_IRQn: C2RustUnnamed = 32;
pub const I2C1_EV_IRQn: C2RustUnnamed = 31;
pub const TIM4_IRQn: C2RustUnnamed = 30;
pub const TIM3_IRQn: C2RustUnnamed = 29;
pub const TIM2_IRQn: C2RustUnnamed = 28;
pub const TIM1_CC_IRQn: C2RustUnnamed = 27;
pub const TIM1_TRG_COM_TIM11_IRQn: C2RustUnnamed = 26;
pub const TIM1_UP_TIM10_IRQn: C2RustUnnamed = 25;
pub const TIM1_BRK_TIM9_IRQn: C2RustUnnamed = 24;
pub const EXTI9_5_IRQn: C2RustUnnamed = 23;
pub const CAN1_SCE_IRQn: C2RustUnnamed = 22;
pub const CAN1_RX1_IRQn: C2RustUnnamed = 21;
pub const CAN1_RX0_IRQn: C2RustUnnamed = 20;
pub const CAN1_TX_IRQn: C2RustUnnamed = 19;
pub const ADC_IRQn: C2RustUnnamed = 18;
pub const DMA1_Stream6_IRQn: C2RustUnnamed = 17;
pub const DMA1_Stream5_IRQn: C2RustUnnamed = 16;
pub const DMA1_Stream4_IRQn: C2RustUnnamed = 15;
pub const DMA1_Stream3_IRQn: C2RustUnnamed = 14;
pub const DMA1_Stream2_IRQn: C2RustUnnamed = 13;
pub const DMA1_Stream1_IRQn: C2RustUnnamed = 12;
pub const DMA1_Stream0_IRQn: C2RustUnnamed = 11;
pub const EXTI4_IRQn: C2RustUnnamed = 10;
pub const EXTI3_IRQn: C2RustUnnamed = 9;
pub const EXTI2_IRQn: C2RustUnnamed = 8;
pub const EXTI1_IRQn: C2RustUnnamed = 7;
pub const EXTI0_IRQn: C2RustUnnamed = 6;
pub const RCC_IRQn: C2RustUnnamed = 5;
pub const FLASH_IRQn: C2RustUnnamed = 4;
pub const RTC_WKUP_IRQn: C2RustUnnamed = 3;
pub const TAMP_STAMP_IRQn: C2RustUnnamed = 2;
pub const PVD_IRQn: C2RustUnnamed = 1;
pub const WWDG_IRQn: C2RustUnnamed = 0;
pub const SysTick_IRQn: C2RustUnnamed = -1;
pub const PendSV_IRQn: C2RustUnnamed = -2;
pub const DebugMonitor_IRQn: C2RustUnnamed = -4;
pub const SVCall_IRQn: C2RustUnnamed = -5;
pub const UsageFault_IRQn: C2RustUnnamed = -10;
pub const BusFault_IRQn: C2RustUnnamed = -11;
pub const MemoryManagement_IRQn: C2RustUnnamed = -12;
pub const NonMaskableInt_IRQn: C2RustUnnamed = -14;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
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
pub type rccPeriphTag_t = uint8_t;
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
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
// make sure that default value (0) does not enable anything
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
pub static mut timerDefinitions: [timerDef_t; 14] =
    [timerDef_t{TIMx: 0 as *mut TIM_TypeDef, rcc: 0, inputIrq: 0,}; 14];
/*
    need a mapping from dma and timers to pins, and the values should all be set here to the dmaMotors array.
    this mapping could be used for both these motors and for led strip.

    only certain pins have OC output (already used in normal PWM et al) but then
    there are only certain DMA streams/channels available for certain timers and channels.
     *** (this may highlight some hardware limitations on some targets) ***

    DMA1

    Channel Stream0     Stream1     Stream2     Stream3     Stream4     Stream5     Stream6     Stream7
    0
    1
    2       TIM4_CH1                            TIM4_CH2                                        TIM4_CH3
    3                   TIM2_CH3                                        TIM2_CH1    TIM2_CH1    TIM2_CH4
                                                                                    TIM2_CH4
    4
    5                               TIM3_CH4                TIM3_CH1    TIM3_CH2                TIM3_CH3
    6       TIM5_CH3    TIM5_CH4    TIM5_CH1    TIM5_CH4    TIM5_CH2
    7

    DMA2

    Channel Stream0     Stream1     Stream2     Stream3     Stream4     Stream5     Stream6     Stream7
    0                               TIM8_CH1                                        TIM1_CH1
                                    TIM8_CH2                                        TIM1_CH2
                                    TIM8_CH3                                        TIM1_CH3
    1
    2
    3
    4
    5
    6                   TIM1_CH1    TIM1_CH2    TIM1_CH1    TIM1_CH4                TIM1_CH3
    7                               TIM8_CH1    TIM8_CH2    TIM8_CH3                            TIM8_CH4
*/
#[no_mangle]
pub unsafe extern "C" fn timerClock(mut tim: *mut TIM_TypeDef) -> uint32_t {
    return SystemCoreClock;
}
unsafe extern "C" fn run_static_initializers() {
    timerDefinitions =
        [{
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x10000 as
                                                                    libc::c_uint).wrapping_add(0
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB2 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                0 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     0 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              0 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              0 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       0 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       0 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                0
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              0 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       0 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       0 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                0
                                                                                    as
                                                                                    libc::c_uint)
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
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                0 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     0 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              0 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              0 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       0 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       0 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                0
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              0 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       0 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       0 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                0
                                                                                    as
                                                                                    libc::c_uint)
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
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x400 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                1 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     1 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              1 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              1 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       1 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       1 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                1
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              1 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       1 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       1 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                1
                                                                                    as
                                                                                    libc::c_uint)
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
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x800 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                2 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     2 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              2 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              2 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       2 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       2 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                2
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              2 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       2 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       2 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                2
                                                                                    as
                                                                                    libc::c_uint)
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
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0xc00 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                3 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     3 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              3 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              3 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       3 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       3 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                3
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              3 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       3 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       3 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                3
                                                                                    as
                                                                                    libc::c_uint)
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
                            inputIrq: TIM5_IRQn as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x1000 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                4 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     4 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              4 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              4 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       4 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       4 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                4
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              4 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       4 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       4 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                4
                                                                                    as
                                                                                    libc::c_uint)
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
                            inputIrq: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x1400 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                5 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     5 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              5 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              5 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       5 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       5 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                5
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              5 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       5 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       5 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                5
                                                                                    as
                                                                                    libc::c_uint)
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
                            inputIrq: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x10000 as
                                                                    libc::c_uint).wrapping_add(0x400
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB2 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                1 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     1 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              1 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              1 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       1 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       1 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                1
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              1 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       1 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       1 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                1
                                                                                    as
                                                                                    libc::c_uint)
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
                                TIM8_CC_IRQn as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x10000 as
                                                                    libc::c_uint).wrapping_add(0x4000
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB2 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                16 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     16 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              16 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              16 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       16 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       16 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                16
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              16 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       16 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       16 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                16
                                                                                    as
                                                                                    libc::c_uint)
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
                                TIM1_BRK_TIM9_IRQn as libc::c_int as
                                    uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x10000 as
                                                                    libc::c_uint).wrapping_add(0x4400
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB2 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                17 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     17 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              17 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              17 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       17 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       17 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                17
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              17 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       17 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       17 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                17
                                                                                    as
                                                                                    libc::c_uint)
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
                                TIM1_UP_TIM10_IRQn as libc::c_int as
                                    uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x10000 as
                                                                    libc::c_uint).wrapping_add(0x4800
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB2 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                18 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     18 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              18 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              18 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       18 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       18 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                18
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              18 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       18 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       18 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                18
                                                                                    as
                                                                                    libc::c_uint)
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
                                TIM1_TRG_COM_TIM11_IRQn as libc::c_int as
                                    uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x1800 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                6 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     6 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              6 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              6 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       6 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       6 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                6
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              6 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       6 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       6 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                6
                                                                                    as
                                                                                    libc::c_uint)
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
                                TIM8_BRK_TIM12_IRQn as libc::c_int as
                                    uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x1c00 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                7 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     7 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              7 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              7 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       7 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       7 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                7
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              7 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       7 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       7 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                7
                                                                                    as
                                                                                    libc::c_uint)
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
                                TIM8_UP_TIM13_IRQn as libc::c_int as
                                    uint8_t,};
             init
         },
         {
             let mut init =
                 timerDef_s{TIMx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x2000 as
                                                                    libc::c_uint)
                                    as *mut TIM_TypeDef,
                            rcc:
                                (((RCC_APB1 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                8 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     8 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              8 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              8 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       8 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       8 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                8
                                                                                    as
                                                                                    libc::c_uint)
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
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              8 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       8 as
                                                                           libc::c_uint)
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
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       8 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                8
                                                                                    as
                                                                                    libc::c_uint)
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
                                TIM8_TRG_COM_TIM14_IRQn as libc::c_int as
                                    uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
