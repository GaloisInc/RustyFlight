use core;
use libc;
extern "C" {
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                 GPIO_InitStruct: *mut GPIO_InitTypeDef);
    /* *
  ******************************************************************************
  * @file    system_stm32f30x.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    28-March-2014
  * @brief   CMSIS Cortex-M4 Device System Source File for STM32F30x devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
    /* * @addtogroup CMSIS
  * @{
  */
    /* * @addtogroup stm32f30x_system
  * @{
  */
    /* *
  * @brief Define to prevent recursive inclusion
  */
    /* Exported types ------------------------------------------------------------*/
    #[no_mangle]
    static mut SystemCoreClock: uint32_t;
    #[no_mangle]
    fn RCC_AHBPeriphClockCmd(RCC_AHBPeriph: uint32_t,
                             NewState: FunctionalState);
    #[no_mangle]
    fn RCC_ClearFlag();
    /* !< 2 bits for pre-emption priority
                                                            2 bits for subpriority */
    /* !< 3 bits for pre-emption priority
                                                            1 bits for subpriority */
    /* !< 4 bits for pre-emption priority
                                                            0 bits for subpriority */
    /* *
  * @}
  */
    /* * @defgroup MISC_SysTick_clock_source 
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
    #[no_mangle]
    fn NVIC_PriorityGroupConfig(NVIC_PriorityGroup: uint32_t);
    #[no_mangle]
    fn cycleCounterInit();
    #[no_mangle]
    static mut cachedRccCsrValue: uint32_t;
    #[no_mangle]
    fn SetSysClock();
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type IRQn = libc::c_int;
pub const FPU_IRQn: IRQn = 81;
pub const USBWakeUp_RMP_IRQn: IRQn = 76;
pub const USB_LP_IRQn: IRQn = 75;
pub const USB_HP_IRQn: IRQn = 74;
pub const COMP7_IRQn: IRQn = 66;
pub const COMP4_5_6_IRQn: IRQn = 65;
pub const COMP1_2_3_IRQn: IRQn = 64;
pub const ADC4_IRQn: IRQn = 61;
pub const DMA2_Channel5_IRQn: IRQn = 60;
pub const DMA2_Channel4_IRQn: IRQn = 59;
pub const DMA2_Channel3_IRQn: IRQn = 58;
pub const DMA2_Channel2_IRQn: IRQn = 57;
pub const DMA2_Channel1_IRQn: IRQn = 56;
pub const TIM7_IRQn: IRQn = 55;
pub const TIM6_DAC_IRQn: IRQn = 54;
pub const UART5_IRQn: IRQn = 53;
pub const UART4_IRQn: IRQn = 52;
pub const SPI3_IRQn: IRQn = 51;
pub const ADC3_IRQn: IRQn = 47;
pub const TIM8_CC_IRQn: IRQn = 46;
pub const TIM8_TRG_COM_IRQn: IRQn = 45;
pub const TIM8_UP_IRQn: IRQn = 44;
pub const TIM8_BRK_IRQn: IRQn = 43;
pub const USBWakeUp_IRQn: IRQn = 42;
pub const RTC_Alarm_IRQn: IRQn = 41;
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
pub const TIM1_TRG_COM_TIM17_IRQn: IRQn = 26;
pub const TIM1_UP_TIM16_IRQn: IRQn = 25;
pub const TIM1_BRK_TIM15_IRQn: IRQn = 24;
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
pub const EXTI2_TS_IRQn: IRQn = 8;
pub const EXTI1_IRQn: IRQn = 7;
pub const EXTI0_IRQn: IRQn = 6;
pub const RCC_IRQn: IRQn = 5;
pub const FLASH_IRQn: IRQn = 4;
pub const RTC_WKUP_IRQn: IRQn = 3;
pub const TAMPER_STAMP_IRQn: IRQn = 2;
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
pub type IRQn_Type = IRQn;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct NVIC_Type {
    pub ISER: [uint32_t; 8],
    pub RESERVED0: [uint32_t; 24],
    pub ICER: [uint32_t; 8],
    pub RSERVED1: [uint32_t; 24],
    pub ISPR: [uint32_t; 8],
    pub RESERVED2: [uint32_t; 24],
    pub ICPR: [uint32_t; 8],
    pub RESERVED3: [uint32_t; 24],
    pub IABR: [uint32_t; 8],
    pub RESERVED4: [uint32_t; 56],
    pub IP: [uint8_t; 240],
    pub RESERVED5: [uint32_t; 644],
    pub STIR: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SCB_Type {
    pub CPUID: uint32_t,
    pub ICSR: uint32_t,
    pub VTOR: uint32_t,
    pub AIRCR: uint32_t,
    pub SCR: uint32_t,
    pub CCR: uint32_t,
    pub SHP: [uint8_t; 12],
    pub SHCSR: uint32_t,
    pub CFSR: uint32_t,
    pub HFSR: uint32_t,
    pub DFSR: uint32_t,
    pub MMFAR: uint32_t,
    pub BFAR: uint32_t,
    pub AFSR: uint32_t,
    pub PFR: [uint32_t; 2],
    pub DFR: uint32_t,
    pub ADR: uint32_t,
    pub MMFR: [uint32_t; 4],
    pub ISAR: [uint32_t; 5],
    pub RESERVED0: [uint32_t; 5],
    pub CPACR: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SysTick_Type {
    pub CTRL: uint32_t,
    pub LOAD: uint32_t,
    pub VAL: uint32_t,
    pub CALIB: uint32_t,
}
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief General Purpose I/O
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint16_t,
    pub RESERVED0: uint16_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint16_t,
    pub RESERVED1: uint16_t,
    pub ODR: uint16_t,
    pub RESERVED2: uint16_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
    pub BRR: uint16_t,
    pub RESERVED3: uint16_t,
    /* !< Reserved,                                                                 0x2A */
}
/* *
  * @brief Reset and Clock Control
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub APB1RSTR: uint32_t,
    pub AHBENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub APB1ENR: uint32_t,
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
    pub AHBRSTR: uint32_t,
    pub CFGR2: uint32_t,
    pub CFGR3: uint32_t,
    /* !< RCC clock configuration register 3,                          Address offset: 0x30 */
}
pub type GPIOMode_TypeDef = libc::c_uint;
pub const GPIO_Mode_AN: GPIOMode_TypeDef = 3;
pub const GPIO_Mode_AF: GPIOMode_TypeDef = 2;
pub const GPIO_Mode_OUT: GPIOMode_TypeDef = 1;
pub const GPIO_Mode_IN: GPIOMode_TypeDef = 0;
pub type GPIOOType_TypeDef = libc::c_uint;
pub const GPIO_OType_OD: GPIOOType_TypeDef = 1;
pub const GPIO_OType_PP: GPIOOType_TypeDef = 0;
pub type GPIOSpeed_TypeDef = libc::c_uint;
pub const GPIO_Speed_Level_3: GPIOSpeed_TypeDef = 3;
pub const GPIO_Speed_Level_2: GPIOSpeed_TypeDef = 2;
pub const GPIO_Speed_Level_1: GPIOSpeed_TypeDef = 1;
pub type GPIOPuPd_TypeDef = libc::c_uint;
pub const GPIO_PuPd_DOWN: GPIOPuPd_TypeDef = 2;
pub const GPIO_PuPd_UP: GPIOPuPd_TypeDef = 1;
pub const GPIO_PuPd_NOPULL: GPIOPuPd_TypeDef = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub GPIO_Pin: uint32_t,
    pub GPIO_Mode: GPIOMode_TypeDef,
    pub GPIO_Speed: GPIOSpeed_TypeDef,
    pub GPIO_OType: GPIOOType_TypeDef,
    pub GPIO_PuPd: GPIOPuPd_TypeDef,
}
#[inline]
unsafe extern "C" fn __NVIC_SetPriority(mut IRQn: IRQn_Type,
                                        mut priority: uint32_t) {
    if IRQn as int32_t >= 0i32 {
        ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0x100u64)
                                                as
                                                *mut NVIC_Type)).IP[IRQn as
                                                                        uint32_t
                                                                        as
                                                                        usize]
                                        as *mut uint8_t,
                                    (priority <<
                                         8u32.wrapping_sub(4i32 as
                                                               libc::c_uint) &
                                         0xffu64 as uint32_t) as uint8_t)
    } else {
        ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0xd00u64)
                                                as
                                                *mut SCB_Type)).SHP[(IRQn as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_ulong
                                                                         &
                                                                         0xfu64).wrapping_sub(4u64)
                                                                        as
                                                                        usize]
                                        as *mut uint8_t,
                                    (priority <<
                                         8u32.wrapping_sub(4i32 as
                                                               libc::c_uint) &
                                         0xffu64 as uint32_t) as uint8_t)
    };
}
#[inline]
unsafe extern "C" fn SysTick_Config(mut ticks: uint32_t) -> uint32_t {
    if (ticks as libc::c_ulong).wrapping_sub(1u64) > 0xffffffu64 {
        return 1u64 as uint32_t
    }
    ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0x10u64) as
                                            *mut SysTick_Type)).LOAD as
                                    *mut uint32_t,
                                (ticks as libc::c_ulong).wrapping_sub(1u64) as
                                    uint32_t);
    __NVIC_SetPriority(SysTick_IRQn,
                       (1u64 << 4i32).wrapping_sub(1u64) as uint32_t);
    ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0x10u64) as
                                            *mut SysTick_Type)).VAL as
                                    *mut uint32_t, 0u64 as uint32_t);
    ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0x10u64) as
                                            *mut SysTick_Type)).CTRL as
                                    *mut uint32_t,
                                (1u64 << 2u32 | 1u64 << 1u32 | 1u64) as
                                    uint32_t);
    return 0u64 as uint32_t;
}
#[inline(always)]
unsafe extern "C" fn __set_MSP(mut topOfMainStack: uint32_t) {
    asm!("MSR msp, $0" : : "r" (topOfMainStack) : : "volatile");
}
#[inline(always)]
unsafe extern "C" fn __enable_irq() {
    asm!("cpsie i" : : : "memory" : "volatile");
}
// bootloader/IAP
#[no_mangle]
pub unsafe extern "C" fn systemReset() {
    // Generate system reset
    ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0xd00u64)
                                            as *mut SCB_Type)).AIRCR as
                                    *mut uint32_t,
                                0x5fa0000i32 as uint32_t |
                                    0x4i32 as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn systemResetToBootloader() {
    // 1FFFF000 -> 20000200 -> SP
    // 1FFFF004 -> 1FFFF021 -> PC
    *(0x20009ffci32 as *mut uint32_t) = 0xdeadbeefu32; // 40KB SRAM STM32F30X
    systemReset(); // Leave JTAG pins alone
}
#[no_mangle]
pub unsafe extern "C" fn enableGPIOPowerUsageAndNoiseReductions() {
    RCC_AHBPeriphClockCmd(0x20000i32 as uint32_t | 0x40000i32 as uint32_t |
                              0x80000i32 as uint32_t | 0x100000i32 as uint32_t
                              | 0x200000i32 as uint32_t |
                              0x400000i32 as uint32_t, ENABLE);
    let mut GPIO_InitStructure: GPIO_InitTypeDef =
        {
            let mut init =
                GPIO_InitTypeDef{GPIO_Pin: 0,
                                 GPIO_Mode: GPIO_Mode_AN,
                                 GPIO_Speed: 0 as GPIOSpeed_TypeDef,
                                 GPIO_OType: GPIO_OType_OD,
                                 GPIO_PuPd: GPIO_PuPd_NOPULL,};
            init
        };
    GPIO_InitStructure.GPIO_Pin =
        (0xffffi32 as uint16_t as libc::c_int &
             !(0x2000i32 as uint16_t as libc::c_int |
                   0x4000i32 as uint16_t as libc::c_int |
                   0x8000i32 as uint16_t as libc::c_int)) as uint32_t;
    GPIO_Init((0x40000000i32 as
                   uint32_t).wrapping_add(0x8000000i32 as
                                              libc::c_uint).wrapping_add(0i32
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = 0xffffi32 as uint16_t as uint32_t;
    GPIO_Init((0x40000000i32 as
                   uint32_t).wrapping_add(0x8000000i32 as
                                              libc::c_uint).wrapping_add(0x400i32
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_Init((0x40000000i32 as
                   uint32_t).wrapping_add(0x8000000i32 as
                                              libc::c_uint).wrapping_add(0x800i32
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_Init((0x40000000i32 as
                   uint32_t).wrapping_add(0x8000000i32 as
                                              libc::c_uint).wrapping_add(0xc00i32
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_Init((0x40000000i32 as
                   uint32_t).wrapping_add(0x8000000i32 as
                                              libc::c_uint).wrapping_add(0x1000i32
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_Init((0x40000000i32 as
                   uint32_t).wrapping_add(0x8000000i32 as
                                              libc::c_uint).wrapping_add(0x1400i32
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
}
#[no_mangle]
pub unsafe extern "C" fn isMPUSoftReset() -> bool {
    if cachedRccCsrValue & 0x10000000i32 as uint32_t != 0 {
        return 1i32 != 0
    } else { return 0i32 != 0 };
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
#[no_mangle]
pub unsafe extern "C" fn systemInit() {
    checkForBootLoaderRequest();
    // Enable FPU
    ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0xd00u64)
                                            as *mut SCB_Type)).CPACR as
                                    *mut uint32_t,
                                (0x3i32 << 10i32 * 2i32 |
                                     0x3i32 << 11i32 * 2i32) as uint32_t);
    SetSysClock();
    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(0x500i32 as uint32_t);
    // cache RCC->CSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CSR;
    RCC_ClearFlag();
    enableGPIOPowerUsageAndNoiseReductions();
    // Init cycle counter
    cycleCounterInit();
    // SysTick
    SysTick_Config(SystemCoreClock.wrapping_div(1000i32 as libc::c_uint));
}
#[no_mangle]
pub unsafe extern "C" fn checkForBootLoaderRequest() {
    let mut bootJump: Option<unsafe extern "C" fn() -> ()> = None;
    if *(0x20009ffci32 as *mut uint32_t) == 0xdeadbeefu32 {
        *(0x20009ffci32 as *mut uint32_t) = 0i32 as uint32_t;
        __enable_irq();
        __set_MSP(*(0x1fffd800i32 as *mut uint32_t));
        bootJump =
            ::core::mem::transmute::<libc::intptr_t,
                                     Option<unsafe extern "C" fn()
                                                ->
                                                    ()>>(*(0x1fffd804i32 as
                                                               *mut uint32_t)
                                                             as
                                                             libc::intptr_t);
        bootJump.expect("non-null function pointer")();
        loop  { }
    };
}
