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
    #[no_mangle]
    fn GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                 GPIO_InitStruct: *mut GPIO_InitTypeDef);
    #[no_mangle]
    fn RCC_APB2PeriphClockCmd(RCC_APB2Periph: uint32_t,
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
    /* * @defgroup SysTick_clock_source 
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* * @defgroup MISC_Exported_Macros
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup MISC_Exported_Functions
  * @{
  */
    #[no_mangle]
    fn NVIC_PriorityGroupConfig(NVIC_PriorityGroup: uint32_t);
    #[no_mangle]
    fn NVIC_SetVectorTable(NVIC_VectTab: uint32_t, Offset: uint32_t);
    #[no_mangle]
    fn cycleCounterInit();
    #[no_mangle]
    static mut cachedRccCsrValue: uint32_t;
    // from system_stm32f10x.c
    #[no_mangle]
    fn SetSysClock(overclock: bool);
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
pub type IRQn_Type = IRQn;
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub CRL: uint32_t,
    pub CRH: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub BRR: uint32_t,
    pub LCKR: uint32_t,
}
/* * 
  * @brief Alternate Function I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AFIO_TypeDef {
    pub EVCR: uint32_t,
    pub MAPR: uint32_t,
    pub EXTICR: [uint32_t; 4],
    pub RESERVED0: uint32_t,
    pub MAPR2: uint32_t,
}
/* * 
  * @brief Reset and Clock Control
  */
#[derive(Copy, Clone)]
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
}
/* *
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library.
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
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup GPIO
  * @{
  */
/* * @defgroup GPIO_Exported_Types
  * @{
  */
/* * 
  * @brief  Output Maximum frequency selection  
  */
pub type GPIOSpeed_TypeDef = libc::c_uint;
pub const GPIO_Speed_50MHz: GPIOSpeed_TypeDef = 3;
pub const GPIO_Speed_2MHz: GPIOSpeed_TypeDef = 2;
pub const GPIO_Speed_10MHz: GPIOSpeed_TypeDef = 1;
/* * 
  * @brief  Configuration Mode enumeration  
  */
pub type GPIOMode_TypeDef = libc::c_uint;
pub const GPIO_Mode_AF_PP: GPIOMode_TypeDef = 24;
pub const GPIO_Mode_AF_OD: GPIOMode_TypeDef = 28;
pub const GPIO_Mode_Out_PP: GPIOMode_TypeDef = 16;
pub const GPIO_Mode_Out_OD: GPIOMode_TypeDef = 20;
pub const GPIO_Mode_IPU: GPIOMode_TypeDef = 72;
pub const GPIO_Mode_IPD: GPIOMode_TypeDef = 40;
pub const GPIO_Mode_IN_FLOATING: GPIOMode_TypeDef = 4;
pub const GPIO_Mode_AIN: GPIOMode_TypeDef = 0;
/* * 
  * @brief  GPIO Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub GPIO_Pin: uint16_t,
    pub GPIO_Speed: GPIOSpeed_TypeDef,
    pub GPIO_Mode: GPIOMode_TypeDef,
}
#[inline]
unsafe extern "C" fn __NVIC_SetPriority(mut IRQn: IRQn_Type,
                                        mut priority: uint32_t) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).IP[IRQn as
                                                                        uint32_t
                                                                        as
                                                                        usize]
                                        as *mut uint8_t,
                                    (priority <<
                                         (8 as
                                              libc::c_uint).wrapping_sub(4 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                                         & 0xff as libc::c_ulong as uint32_t)
                                        as uint8_t)
    } else {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0xd00
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut SCB_Type)).SHP[(IRQn as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_ulong
                                                                         &
                                                                         0xf
                                                                             as
                                                                             libc::c_ulong).wrapping_sub(4
                                                                                                             as
                                                                                                             libc::c_ulong)
                                                                        as
                                                                        usize]
                                        as *mut uint8_t,
                                    (priority <<
                                         (8 as
                                              libc::c_uint).wrapping_sub(4 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                                         & 0xff as libc::c_ulong as uint32_t)
                                        as uint8_t)
    };
}
#[inline]
unsafe extern "C" fn SysTick_Config(mut ticks: uint32_t) -> uint32_t {
    if (ticks as libc::c_ulong).wrapping_sub(1 as libc::c_ulong) >
           0xffffff as libc::c_ulong {
        return 1 as libc::c_ulong as uint32_t
    }
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0x10
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SysTick_Type)).LOAD as
                                    *mut uint32_t,
                                (ticks as
                                     libc::c_ulong).wrapping_sub(1 as
                                                                     libc::c_ulong)
                                    as uint32_t);
    __NVIC_SetPriority(SysTick_IRQn,
                       ((1 as libc::c_ulong) <<
                            4 as libc::c_int).wrapping_sub(1 as libc::c_ulong)
                           as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0x10
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SysTick_Type)).VAL as
                                    *mut uint32_t,
                                0 as libc::c_ulong as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0x10
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SysTick_Type)).CTRL as
                                    *mut uint32_t,
                                ((1 as libc::c_ulong) << 2 as libc::c_uint |
                                     (1 as libc::c_ulong) << 1 as libc::c_uint
                                     | 1 as libc::c_ulong) as uint32_t);
    return 0 as libc::c_ulong as uint32_t;
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
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).AIRCR as
                                    *mut uint32_t,
                                0x5fa0000 as libc::c_int as uint32_t |
                                    0x4 as libc::c_int as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn systemResetToBootloader() {
    // 1FFFF000 -> 20000200 -> SP
    // 1FFFF004 -> 1FFFF021 -> PC
    *(0x20004ff0 as libc::c_int as *mut uint32_t) =
        0xdeadbeef as libc::c_uint; // 20KB STM32F103
    systemReset();
}
#[no_mangle]
pub unsafe extern "C" fn enableGPIOPowerUsageAndNoiseReductions() {
    RCC_APB2PeriphClockCmd(0x4 as libc::c_int as uint32_t |
                               0x8 as libc::c_int as uint32_t |
                               0x10 as libc::c_int as uint32_t, ENABLE);
    let mut GPIO_InitStructure: GPIO_InitTypeDef =
        {
            let mut init =
                GPIO_InitTypeDef{GPIO_Pin: 0xffff as libc::c_int as uint16_t,
                                 GPIO_Speed: 0 as GPIOSpeed_TypeDef,
                                 GPIO_Mode: GPIO_Mode_AIN,};
            init
        };
    GPIO_Init((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x800
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_Init((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0xc00
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_Init((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x1000
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
}
#[no_mangle]
pub unsafe extern "C" fn isMPUSoftReset() -> bool {
    if cachedRccCsrValue & 0x10000000 as libc::c_int as uint32_t != 0 {
        return 1 as libc::c_int != 0
    } else { return 0 as libc::c_int != 0 };
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
    SetSysClock(0 as libc::c_int != 0);
    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    extern "C" {
        #[no_mangle]
        static mut isr_vector_table_base: *mut libc::c_void;
    }
    NVIC_SetVectorTable(&mut isr_vector_table_base as *mut *mut libc::c_void
                            as uint32_t, 0 as libc::c_int as uint32_t);
    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(0x500 as libc::c_int as uint32_t);
    // Turn on clocks for stuff we use
    RCC_APB2PeriphClockCmd(0x1 as libc::c_int as uint32_t, ENABLE);
    // cache RCC->CSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CSR;
    RCC_ClearFlag();
    enableGPIOPowerUsageAndNoiseReductions();
    // Set USART1 TX (PA9) to output and high state to prevent a rs232 break condition on reset.
    // See issue https://github.com/cleanflight/cleanflight/issues/1433
    let mut GPIO_InitStructure: GPIO_InitTypeDef =
        {
            let mut init =
                GPIO_InitTypeDef{GPIO_Pin: 0x200 as libc::c_int as uint16_t,
                                 GPIO_Speed: GPIO_Speed_2MHz,
                                 GPIO_Mode: GPIO_Mode_Out_PP,};
            init
        };
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x800
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut GPIO_TypeDef)).BSRR as
                                    *mut uint32_t,
                                GPIO_InitStructure.GPIO_Pin as uint32_t);
    GPIO_Init((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x800
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    // Turn off JTAG port 'cause we're using the GPIO for leds
    let ref mut fresh0 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut AFIO_TypeDef)).MAPR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x2 as libc::c_int) <<
                                          24 as libc::c_int) as libc::c_uint)
                                    as uint32_t as uint32_t);
    // Init cycle counter
    cycleCounterInit();
    // SysTick
    SysTick_Config(SystemCoreClock.wrapping_div(1000 as libc::c_int as
                                                    libc::c_uint));
}
#[no_mangle]
pub unsafe extern "C" fn checkForBootLoaderRequest() {
    let mut bootJump: Option<unsafe extern "C" fn() -> ()> = None;
    if *(0x20004ff0 as libc::c_int as *mut uint32_t) ==
           0xdeadbeef as libc::c_uint {
        *(0x20004ff0 as libc::c_int as *mut uint32_t) =
            0 as libc::c_int as uint32_t;
        __enable_irq();
        __set_MSP(*(0x1ffff000 as libc::c_int as *mut uint32_t));
        bootJump =
            ::core::mem::transmute::<libc::intptr_t,
                                     Option<unsafe extern "C" fn()
                                                ->
                                                    ()>>(*(0x1ffff004 as
                                                               libc::c_int as
                                                               *mut uint32_t)
                                                             as
                                                             libc::intptr_t);
        bootJump.expect("non-null function pointer")();
        loop  { }
    };
}
