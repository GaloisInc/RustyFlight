use ::libc;
pub type IRQn_Type = libc::c_int;
pub const SPDIF_RX_IRQn: IRQn_Type = 97;
pub const I2C4_ER_IRQn: IRQn_Type = 96;
pub const I2C4_EV_IRQn: IRQn_Type = 95;
pub const CEC_IRQn: IRQn_Type = 94;
pub const LPTIM1_IRQn: IRQn_Type = 93;
pub const QUADSPI_IRQn: IRQn_Type = 92;
pub const SAI2_IRQn: IRQn_Type = 91;
pub const DMA2D_IRQn: IRQn_Type = 90;
pub const SAI1_IRQn: IRQn_Type = 87;
pub const SPI6_IRQn: IRQn_Type = 86;
pub const SPI5_IRQn: IRQn_Type = 85;
pub const SPI4_IRQn: IRQn_Type = 84;
pub const UART8_IRQn: IRQn_Type = 83;
pub const UART7_IRQn: IRQn_Type = 82;
pub const FPU_IRQn: IRQn_Type = 81;
pub const RNG_IRQn: IRQn_Type = 80;
pub const DCMI_IRQn: IRQn_Type = 78;
pub const OTG_HS_IRQn: IRQn_Type = 77;
pub const OTG_HS_WKUP_IRQn: IRQn_Type = 76;
pub const OTG_HS_EP1_IN_IRQn: IRQn_Type = 75;
pub const OTG_HS_EP1_OUT_IRQn: IRQn_Type = 74;
pub const I2C3_ER_IRQn: IRQn_Type = 73;
pub const I2C3_EV_IRQn: IRQn_Type = 72;
pub const USART6_IRQn: IRQn_Type = 71;
pub const DMA2_Stream7_IRQn: IRQn_Type = 70;
pub const DMA2_Stream6_IRQn: IRQn_Type = 69;
pub const DMA2_Stream5_IRQn: IRQn_Type = 68;
pub const OTG_FS_IRQn: IRQn_Type = 67;
pub const CAN2_SCE_IRQn: IRQn_Type = 66;
pub const CAN2_RX1_IRQn: IRQn_Type = 65;
pub const CAN2_RX0_IRQn: IRQn_Type = 64;
pub const CAN2_TX_IRQn: IRQn_Type = 63;
pub const ETH_WKUP_IRQn: IRQn_Type = 62;
pub const ETH_IRQn: IRQn_Type = 61;
pub const DMA2_Stream4_IRQn: IRQn_Type = 60;
pub const DMA2_Stream3_IRQn: IRQn_Type = 59;
pub const DMA2_Stream2_IRQn: IRQn_Type = 58;
pub const DMA2_Stream1_IRQn: IRQn_Type = 57;
pub const DMA2_Stream0_IRQn: IRQn_Type = 56;
pub const TIM7_IRQn: IRQn_Type = 55;
pub const TIM6_DAC_IRQn: IRQn_Type = 54;
pub const UART5_IRQn: IRQn_Type = 53;
pub const UART4_IRQn: IRQn_Type = 52;
pub const SPI3_IRQn: IRQn_Type = 51;
pub const TIM5_IRQn: IRQn_Type = 50;
pub const SDMMC1_IRQn: IRQn_Type = 49;
pub const FMC_IRQn: IRQn_Type = 48;
pub const DMA1_Stream7_IRQn: IRQn_Type = 47;
pub const TIM8_CC_IRQn: IRQn_Type = 46;
pub const TIM8_TRG_COM_TIM14_IRQn: IRQn_Type = 45;
pub const TIM8_UP_TIM13_IRQn: IRQn_Type = 44;
pub const TIM8_BRK_TIM12_IRQn: IRQn_Type = 43;
pub const OTG_FS_WKUP_IRQn: IRQn_Type = 42;
pub const RTC_Alarm_IRQn: IRQn_Type = 41;
pub const EXTI15_10_IRQn: IRQn_Type = 40;
pub const USART3_IRQn: IRQn_Type = 39;
pub const USART2_IRQn: IRQn_Type = 38;
pub const USART1_IRQn: IRQn_Type = 37;
pub const SPI2_IRQn: IRQn_Type = 36;
pub const SPI1_IRQn: IRQn_Type = 35;
pub const I2C2_ER_IRQn: IRQn_Type = 34;
pub const I2C2_EV_IRQn: IRQn_Type = 33;
pub const I2C1_ER_IRQn: IRQn_Type = 32;
pub const I2C1_EV_IRQn: IRQn_Type = 31;
pub const TIM4_IRQn: IRQn_Type = 30;
pub const TIM3_IRQn: IRQn_Type = 29;
pub const TIM2_IRQn: IRQn_Type = 28;
pub const TIM1_CC_IRQn: IRQn_Type = 27;
pub const TIM1_TRG_COM_TIM11_IRQn: IRQn_Type = 26;
pub const TIM1_UP_TIM10_IRQn: IRQn_Type = 25;
pub const TIM1_BRK_TIM9_IRQn: IRQn_Type = 24;
pub const EXTI9_5_IRQn: IRQn_Type = 23;
pub const CAN1_SCE_IRQn: IRQn_Type = 22;
pub const CAN1_RX1_IRQn: IRQn_Type = 21;
pub const CAN1_RX0_IRQn: IRQn_Type = 20;
pub const CAN1_TX_IRQn: IRQn_Type = 19;
pub const ADC_IRQn: IRQn_Type = 18;
pub const DMA1_Stream6_IRQn: IRQn_Type = 17;
pub const DMA1_Stream5_IRQn: IRQn_Type = 16;
pub const DMA1_Stream4_IRQn: IRQn_Type = 15;
pub const DMA1_Stream3_IRQn: IRQn_Type = 14;
pub const DMA1_Stream2_IRQn: IRQn_Type = 13;
pub const DMA1_Stream1_IRQn: IRQn_Type = 12;
pub const DMA1_Stream0_IRQn: IRQn_Type = 11;
pub const EXTI4_IRQn: IRQn_Type = 10;
pub const EXTI3_IRQn: IRQn_Type = 9;
pub const EXTI2_IRQn: IRQn_Type = 8;
pub const EXTI1_IRQn: IRQn_Type = 7;
pub const EXTI0_IRQn: IRQn_Type = 6;
pub const RCC_IRQn: IRQn_Type = 5;
pub const FLASH_IRQn: IRQn_Type = 4;
pub const RTC_WKUP_IRQn: IRQn_Type = 3;
pub const TAMP_STAMP_IRQn: IRQn_Type = 2;
pub const PVD_IRQn: IRQn_Type = 1;
pub const WWDG_IRQn: IRQn_Type = 0;
pub const SysTick_IRQn: IRQn_Type = -1;
pub const PendSV_IRQn: IRQn_Type = -2;
pub const DebugMonitor_IRQn: IRQn_Type = -4;
pub const SVCall_IRQn: IRQn_Type = -5;
pub const UsageFault_IRQn: IRQn_Type = -10;
pub const BusFault_IRQn: IRQn_Type = -11;
pub const MemoryManagement_IRQn: IRQn_Type = -12;
pub const NonMaskableInt_IRQn: IRQn_Type = -14;
pub type __uint8_t = libc::c_uchar;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
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
    pub SHPR: [uint8_t; 12],
    pub SHCSR: uint32_t,
    pub CFSR: uint32_t,
    pub HFSR: uint32_t,
    pub DFSR: uint32_t,
    pub MMFAR: uint32_t,
    pub BFAR: uint32_t,
    pub AFSR: uint32_t,
    pub ID_PFR: [uint32_t; 2],
    pub ID_DFR: uint32_t,
    pub ID_AFR: uint32_t,
    pub ID_MFR: [uint32_t; 4],
    pub ID_ISAR: [uint32_t; 5],
    pub RESERVED0: [uint32_t; 1],
    pub CLIDR: uint32_t,
    pub CTR: uint32_t,
    pub CCSIDR: uint32_t,
    pub CSSELR: uint32_t,
    pub CPACR: uint32_t,
    pub RESERVED3: [uint32_t; 93],
    pub STIR: uint32_t,
    pub RESERVED4: [uint32_t; 15],
    pub MVFR0: uint32_t,
    pub MVFR1: uint32_t,
    pub MVFR2: uint32_t,
    pub RESERVED5: [uint32_t; 1],
    pub ICIALLU: uint32_t,
    pub RESERVED6: [uint32_t; 1],
    pub ICIMVAU: uint32_t,
    pub DCIMVAC: uint32_t,
    pub DCISW: uint32_t,
    pub DCCMVAU: uint32_t,
    pub DCCMVAC: uint32_t,
    pub DCCSW: uint32_t,
    pub DCCIMVAC: uint32_t,
    pub DCCISW: uint32_t,
    pub RESERVED7: [uint32_t; 6],
    pub ITCMCR: uint32_t,
    pub DTCMCR: uint32_t,
    pub AHBPCR: uint32_t,
    pub CACR: uint32_t,
    pub AHBSCR: uint32_t,
    pub RESERVED8: [uint32_t; 1],
    pub ABFSR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SysTick_Type {
    pub CTRL: uint32_t,
    pub LOAD: uint32_t,
    pub VAL: uint32_t,
    pub CALIB: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct MPU_Type {
    pub TYPE: uint32_t,
    pub CTRL: uint32_t,
    pub RNR: uint32_t,
    pub RBAR: uint32_t,
    pub RASR: uint32_t,
    pub RBAR_A1: uint32_t,
    pub RASR_A1: uint32_t,
    pub RBAR_A2: uint32_t,
    pub RASR_A2: uint32_t,
    pub RBAR_A3: uint32_t,
    pub RASR_A3: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f7xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   CMSIS STM32F7xx Device Peripheral Access Layer Header File.           
  *            
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32F7xx device used in the target application
  *              - To use or not the peripheral�s drivers in application code(i.e. 
  *                code will be based on direct access to peripheral�s registers 
  *                rather than drivers API), this option is controlled by 
  *                "#define USE_HAL_DRIVER"
  *  
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
/* * @addtogroup stm32f7xx
  * @{
  */
/* __cplusplus */
/* * @addtogroup Library_configuration_section
  * @{
  */
/* *
  * @brief STM32 Family
  */
/* STM32F7 */
/* Uncomment the line below according to the target STM32 device used in your
   application 
  */
/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
/* USE_HAL_DRIVER */
/* *
  * @brief CMSIS Device version number V1.2.0
  */
/* !< [31:24] main version */
/* !< [23:16] sub1 version */
/* !< [15:8]  sub2 version */
/* !< [7:0]  release candidate */
/* *
  * @}
  */
/* * @addtogroup Device_Included
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup Exported_types
  * @{
  */
pub type C2RustUnnamed = libc::c_uint;
pub const SET: C2RustUnnamed = 1;
pub const RESET: C2RustUnnamed = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_cortex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of CORTEX HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup CORTEX
  * @{
  */ 
/* Exported types ------------------------------------------------------------*/
/* * @defgroup CORTEX_Exported_Types Cortex Exported Types
  * @{
  */
/* * @defgroup CORTEX_MPU_Region_Initialization_Structure_definition MPU Region Initialization Structure Definition
  * @brief  MPU Region initialization structure 
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct MPU_Region_InitTypeDef {
    pub Enable: uint8_t,
    pub Number: uint8_t,
    pub BaseAddress: uint32_t,
    pub Size: uint8_t,
    pub SubRegionDisable: uint8_t,
    pub TypeExtField: uint8_t,
    pub AccessPermission: uint8_t,
    pub DisableExec: uint8_t,
    pub IsShareable: uint8_t,
    pub IsCacheable: uint8_t,
    pub IsBufferable: uint8_t,
}
#[inline(always)]
unsafe extern "C" fn __ISB() { asm!("isb 0xF" : : : "memory" : "volatile"); }
#[inline(always)]
unsafe extern "C" fn __DSB() { asm!("dsb 0xF" : : : "memory" : "volatile"); }
#[inline(always)]
unsafe extern "C" fn __DMB() { asm!("dmb 0xF" : : : "memory" : "volatile"); }
#[inline]
unsafe extern "C" fn __NVIC_SetPriorityGrouping(mut PriorityGroup: uint32_t) {
    let mut reg_value: uint32_t = 0;
    let mut PriorityGroupTmp: uint32_t =
        PriorityGroup & 0x7 as libc::c_ulong as uint32_t;
    reg_value =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).AIRCR;
    reg_value &=
        !(((0xffff as libc::c_ulong) << 16 as libc::c_uint |
               (7 as libc::c_ulong) << 8 as libc::c_uint) as uint32_t);
    reg_value =
        reg_value | (0x5fa as libc::c_ulong as uint32_t) << 16 as libc::c_uint
            | PriorityGroupTmp << 8 as libc::c_uint;
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).AIRCR as
                                    *mut uint32_t, reg_value);
}
#[inline]
unsafe extern "C" fn __NVIC_GetPriorityGrouping() -> uint32_t {
    return (((*((0xe000e000 as
                     libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong) as
                    *mut SCB_Type)).AIRCR as libc::c_ulong &
                 (7 as libc::c_ulong) << 8 as libc::c_uint) >>
                8 as libc::c_uint) as uint32_t;
}
#[inline]
unsafe extern "C" fn __NVIC_EnableIRQ(mut IRQn: IRQn_Type) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).ISER[(IRQn as
                                                                           uint32_t
                                                                           >>
                                                                           5
                                                                               as
                                                                               libc::c_ulong)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    ((1 as libc::c_ulong) <<
                                         (IRQn as uint32_t as libc::c_ulong &
                                              0x1f as libc::c_ulong)) as
                                        uint32_t)
    };
}
#[inline]
unsafe extern "C" fn __NVIC_DisableIRQ(mut IRQn: IRQn_Type) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).ICER[(IRQn as
                                                                           uint32_t
                                                                           >>
                                                                           5
                                                                               as
                                                                               libc::c_ulong)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    ((1 as libc::c_ulong) <<
                                         (IRQn as uint32_t as libc::c_ulong &
                                              0x1f as libc::c_ulong)) as
                                        uint32_t);
        __DSB();
        __ISB();
    };
}
#[inline]
unsafe extern "C" fn __NVIC_GetPendingIRQ(mut IRQn: IRQn_Type) -> uint32_t {
    if IRQn as int32_t >= 0 as libc::c_int {
        return if (*((0xe000e000 as
                          libc::c_ulong).wrapping_add(0x100 as libc::c_ulong)
                         as
                         *mut NVIC_Type)).ISPR[(IRQn as uint32_t >>
                                                    5 as libc::c_ulong) as
                                                   usize] as libc::c_ulong &
                      (1 as libc::c_ulong) <<
                          (IRQn as uint32_t as libc::c_ulong &
                               0x1f as libc::c_ulong) != 0 as libc::c_ulong {
                   1 as libc::c_ulong
               } else { 0 as libc::c_ulong } as uint32_t
    } else { return 0 as libc::c_uint };
}
#[inline]
unsafe extern "C" fn __NVIC_SetPendingIRQ(mut IRQn: IRQn_Type) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).ISPR[(IRQn as
                                                                           uint32_t
                                                                           >>
                                                                           5
                                                                               as
                                                                               libc::c_ulong)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    ((1 as libc::c_ulong) <<
                                         (IRQn as uint32_t as libc::c_ulong &
                                              0x1f as libc::c_ulong)) as
                                        uint32_t)
    };
}
#[inline]
unsafe extern "C" fn __NVIC_ClearPendingIRQ(mut IRQn: IRQn_Type) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).ICPR[(IRQn as
                                                                           uint32_t
                                                                           >>
                                                                           5
                                                                               as
                                                                               libc::c_ulong)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    ((1 as libc::c_ulong) <<
                                         (IRQn as uint32_t as libc::c_ulong &
                                              0x1f as libc::c_ulong)) as
                                        uint32_t)
    };
}
#[inline]
unsafe extern "C" fn __NVIC_GetActive(mut IRQn: IRQn_Type) -> uint32_t {
    if IRQn as int32_t >= 0 as libc::c_int {
        return if (*((0xe000e000 as
                          libc::c_ulong).wrapping_add(0x100 as libc::c_ulong)
                         as
                         *mut NVIC_Type)).IABR[(IRQn as uint32_t >>
                                                    5 as libc::c_ulong) as
                                                   usize] as libc::c_ulong &
                      (1 as libc::c_ulong) <<
                          (IRQn as uint32_t as libc::c_ulong &
                               0x1f as libc::c_ulong) != 0 as libc::c_ulong {
                   1 as libc::c_ulong
               } else { 0 as libc::c_ulong } as uint32_t
    } else { return 0 as libc::c_uint };
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
                                                *mut SCB_Type)).SHPR[(IRQn as
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
unsafe extern "C" fn __NVIC_GetPriority(mut IRQn: IRQn_Type) -> uint32_t {
    if IRQn as int32_t >= 0 as libc::c_int {
        return (*((0xe000e000 as
                       libc::c_ulong).wrapping_add(0x100 as libc::c_ulong) as
                      *mut NVIC_Type)).IP[IRQn as uint32_t as usize] as
                   uint32_t >>
                   (8 as
                        libc::c_uint).wrapping_sub(4 as libc::c_int as
                                                       libc::c_uint)
    } else {
        return (*((0xe000e000 as
                       libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong) as
                      *mut SCB_Type)).SHPR[(IRQn as uint32_t as libc::c_ulong
                                                &
                                                0xf as
                                                    libc::c_ulong).wrapping_sub(4
                                                                                    as
                                                                                    libc::c_ulong)
                                               as usize] as uint32_t >>
                   (8 as
                        libc::c_uint).wrapping_sub(4 as libc::c_int as
                                                       libc::c_uint)
    };
}
#[inline]
unsafe extern "C" fn NVIC_EncodePriority(mut PriorityGroup: uint32_t,
                                         mut PreemptPriority: uint32_t,
                                         mut SubPriority: uint32_t)
 -> uint32_t {
    let mut PriorityGroupTmp: uint32_t =
        PriorityGroup & 0x7 as libc::c_ulong as uint32_t;
    let mut PreemptPriorityBits: uint32_t = 0;
    let mut SubPriorityBits: uint32_t = 0;
    PreemptPriorityBits =
        if (7 as
                libc::c_ulong).wrapping_sub(PriorityGroupTmp as libc::c_ulong)
               > 4 as libc::c_int as uint32_t as libc::c_ulong {
            4 as libc::c_int as uint32_t
        } else {
            (7 as
                 libc::c_ulong).wrapping_sub(PriorityGroupTmp as
                                                 libc::c_ulong) as uint32_t
        };
    SubPriorityBits =
        if PriorityGroupTmp.wrapping_add(4 as libc::c_int as uint32_t) <
               7 as libc::c_ulong as uint32_t {
            0 as libc::c_ulong as uint32_t
        } else {
            (PriorityGroupTmp as
                 libc::c_ulong).wrapping_sub(7 as
                                                 libc::c_ulong).wrapping_add(4
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 uint32_t
                                                                                 as
                                                                                 libc::c_ulong)
                as uint32_t
        };
    return (PreemptPriority &
                ((1 as libc::c_ulong) <<
                     PreemptPriorityBits).wrapping_sub(1 as libc::c_ulong) as
                    uint32_t) << SubPriorityBits |
               SubPriority &
                   ((1 as libc::c_ulong) <<
                        SubPriorityBits).wrapping_sub(1 as libc::c_ulong) as
                       uint32_t;
}
#[inline]
unsafe extern "C" fn NVIC_DecodePriority(mut Priority: uint32_t,
                                         mut PriorityGroup: uint32_t,
                                         pPreemptPriority: *mut uint32_t,
                                         pSubPriority: *mut uint32_t) {
    let mut PriorityGroupTmp: uint32_t =
        PriorityGroup & 0x7 as libc::c_ulong as uint32_t;
    let mut PreemptPriorityBits: uint32_t = 0;
    let mut SubPriorityBits: uint32_t = 0;
    PreemptPriorityBits =
        if (7 as
                libc::c_ulong).wrapping_sub(PriorityGroupTmp as libc::c_ulong)
               > 4 as libc::c_int as uint32_t as libc::c_ulong {
            4 as libc::c_int as uint32_t
        } else {
            (7 as
                 libc::c_ulong).wrapping_sub(PriorityGroupTmp as
                                                 libc::c_ulong) as uint32_t
        };
    SubPriorityBits =
        if PriorityGroupTmp.wrapping_add(4 as libc::c_int as uint32_t) <
               7 as libc::c_ulong as uint32_t {
            0 as libc::c_ulong as uint32_t
        } else {
            (PriorityGroupTmp as
                 libc::c_ulong).wrapping_sub(7 as
                                                 libc::c_ulong).wrapping_add(4
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 uint32_t
                                                                                 as
                                                                                 libc::c_ulong)
                as uint32_t
        };
    *pPreemptPriority =
        Priority >> SubPriorityBits &
            ((1 as libc::c_ulong) <<
                 PreemptPriorityBits).wrapping_sub(1 as libc::c_ulong) as
                uint32_t;
    *pSubPriority =
        Priority &
            ((1 as libc::c_ulong) <<
                 SubPriorityBits).wrapping_sub(1 as libc::c_ulong) as
                uint32_t;
}
#[inline]
unsafe extern "C" fn __NVIC_SystemReset() {
    __DSB();
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).AIRCR as
                                    *mut uint32_t,
                                ((0x5fa as libc::c_ulong) <<
                                     16 as libc::c_uint |
                                     (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).AIRCR as
                                         libc::c_ulong &
                                         (7 as libc::c_ulong) <<
                                             8 as libc::c_uint |
                                     (1 as libc::c_ulong) <<
                                         2 as libc::c_uint) as uint32_t);
    __DSB();
    loop  { asm!("nop" : : : : "volatile") };
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
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_cortex.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   CORTEX HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the CORTEX:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions 
  *
  @verbatim  
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================

    [..]  
    *** How to configure Interrupts using CORTEX HAL driver ***
    ===========================================================
    [..]     
    This section provides functions allowing to configure the NVIC interrupts (IRQ).
    The Cortex-M4 exceptions are managed by CMSIS functions.
   
    (#) Configure the NVIC Priority Grouping using HAL_NVIC_SetPriorityGrouping()
        function according to the following table.
    (#) Configure the priority of the selected IRQ Channels using HAL_NVIC_SetPriority(). 
    (#) Enable the selected IRQ Channels using HAL_NVIC_EnableIRQ().
    (#) please refer to programming manual for details in how to configure priority. 
      
     -@- When the NVIC_PRIORITYGROUP_0 is selected, IRQ preemption is no more possible. 
         The pending IRQ priority will be managed only by the sub priority.
   
     -@- IRQ priority order (sorted by highest to lowest priority):
        (+@) Lowest preemption priority
        (+@) Lowest sub priority
        (+@) Lowest hardware priority (IRQ number)
 
    [..]  
    *** How to configure Systick using CORTEX HAL driver ***
    ========================================================
    [..]
    Setup SysTick Timer for time base.
           
   (+) The HAL_SYSTICK_Config() function calls the SysTick_Config() function which
       is a CMSIS function that:
        (++) Configures the SysTick Reload register with value passed as function parameter.
        (++) Configures the SysTick IRQ priority to the lowest value (0x0F).
        (++) Resets the SysTick Counter register.
        (++) Configures the SysTick Counter clock source to be Core Clock Source (HCLK).
        (++) Enables the SysTick Interrupt.
        (++) Starts the SysTick Counter.
    
   (+) You can change the SysTick Clock source to be HCLK_Div8 by calling the macro
       __HAL_CORTEX_SYSTICKCLK_CONFIG(SYSTICK_CLKSOURCE_HCLK_DIV8) just after the
       HAL_SYSTICK_Config() function call. The __HAL_CORTEX_SYSTICKCLK_CONFIG() macro is defined
       inside the stm32f7xx_hal_cortex.h file.

   (+) You can change the SysTick IRQ priority by calling the
       HAL_NVIC_SetPriority(SysTick_IRQn,...) function just after the HAL_SYSTICK_Config() function 
       call. The HAL_NVIC_SetPriority() call the NVIC_SetPriority() function which is a CMSIS function.

   (+) To adjust the SysTick time base, use the following formula:
                            
       Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
       (++) Reload Value is the parameter to be passed for HAL_SYSTICK_Config() function
       (++) Reload Value should not exceed 0xFFFFFF
   
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @defgroup CORTEX CORTEX
  * @brief CORTEX HAL module driver
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @defgroup CORTEX_Exported_Functions CORTEX Exported Functions
  * @{
  */
/* * @defgroup CORTEX_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]
      This section provides the CORTEX HAL driver functions allowing to configure Interrupts
      Systick functionalities 

@endverbatim
  * @{
  */
/* *
  * @brief  Sets the priority grouping field (preemption priority and subpriority)
  *         using the required unlock sequence.
  * @param  PriorityGroup: The priority grouping bits length. 
  *         This parameter can be one of the following values:
  *         @arg NVIC_PRIORITYGROUP_0: 0 bits for preemption priority
  *                                    4 bits for subpriority
  *         @arg NVIC_PRIORITYGROUP_1: 1 bits for preemption priority
  *                                    3 bits for subpriority
  *         @arg NVIC_PRIORITYGROUP_2: 2 bits for preemption priority
  *                                    2 bits for subpriority
  *         @arg NVIC_PRIORITYGROUP_3: 3 bits for preemption priority
  *                                    1 bits for subpriority
  *         @arg NVIC_PRIORITYGROUP_4: 4 bits for preemption priority
  *                                    0 bits for subpriority
  * @note   When the NVIC_PriorityGroup_0 is selected, IRQ preemption is no more possible. 
  *         The pending IRQ priority will be managed only by the subpriority. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_SetPriorityGrouping(mut PriorityGroup:
                                                          uint32_t) {
    /* Check the parameters */
    /* Set the PRIGROUP[10:8] bits according to the PriorityGroup parameter value */
    __NVIC_SetPriorityGrouping(PriorityGroup);
}
/* *
  * @brief  Sets the priority of an interrupt.
  * @param  IRQn: External interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f7xxxx.h))
  * @param  PreemptPriority: The preemption priority for the IRQn channel.
  *         This parameter can be a value between 0 and 15
  *         A lower priority value indicates a higher priority 
  * @param  SubPriority: the subpriority level for the IRQ channel.
  *         This parameter can be a value between 0 and 15
  *         A lower priority value indicates a higher priority.          
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_SetPriority(mut IRQn: IRQn_Type,
                                              mut PreemptPriority: uint32_t,
                                              mut SubPriority: uint32_t) {
    let mut prioritygroup: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    prioritygroup = __NVIC_GetPriorityGrouping();
    __NVIC_SetPriority(IRQn,
                       NVIC_EncodePriority(prioritygroup, PreemptPriority,
                                           SubPriority));
}
/* *
  * @brief  Enables a device specific interrupt in the NVIC interrupt controller.
  * @note   To configure interrupts priority correctly, the NVIC_PriorityGroupConfig()
  *         function should be called before. 
  * @param  IRQn External interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f7xxxx.h))
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_EnableIRQ(mut IRQn: IRQn_Type) {
    /* Check the parameters */
    /* Enable interrupt */
    __NVIC_EnableIRQ(IRQn);
}
/* *
  * @brief  Disables a device specific interrupt in the NVIC interrupt controller.
  * @param  IRQn External interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f7xxxx.h))
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_DisableIRQ(mut IRQn: IRQn_Type) {
    /* Check the parameters */
    /* Disable interrupt */
    __NVIC_DisableIRQ(IRQn);
}
/* *
  * @brief  Initiates a system reset request to reset the MCU.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_SystemReset() {
    /* System Reset */
    __NVIC_SystemReset();
}
/* *
  * @brief  Initializes the System Timer and its interrupt, and starts the System Tick Timer.
  *         Counter is in free running mode to generate periodic interrupts.
  * @param  TicksNumb: Specifies the ticks Number of ticks between two interrupts.
  * @retval status:  - 0  Function succeeded.
  *                  - 1  Function failed.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SYSTICK_Config(mut TicksNumb: uint32_t)
 -> uint32_t {
    return SysTick_Config(TicksNumb);
}
/* *
  * @}
  */
/* * @defgroup CORTEX_Exported_Functions_Group2 Peripheral Control functions
 *  @brief   Cortex control functions 
 *
@verbatim   
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================  
    [..]
      This subsection provides a set of functions allowing to control the CORTEX
      (NVIC, SYSTICK, MPU) functionalities. 
 
      
@endverbatim
  * @{
  */
/* *
  * @brief  Disables the MPU
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MPU_Disable() {
    /* Make sure outstanding transfers are done */
    __DMB();
    /* Disable fault exceptions */
    let ref mut fresh0 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SHCSR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_ulong &
                                     !((1 as libc::c_ulong) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable the MPU and clear the control register*/
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd90
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut MPU_Type)).CTRL as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
}
/* *
  * @brief  Enables the MPU
  * @param  MPU_Control: Specifies the control mode of the MPU during hard fault, 
  *          NMI, FAULTMASK and privileged access to the default memory 
  *          This parameter can be one of the following values:
  *            @arg MPU_HFNMI_PRIVDEF_NONE
  *            @arg MPU_HARDFAULT_NMI
  *            @arg MPU_PRIVILEGED_DEFAULT
  *            @arg MPU_HFNMI_PRIVDEF
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MPU_Enable(mut MPU_Control: uint32_t) {
    /* Enable the MPU */
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd90
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut MPU_Type)).CTRL as
                                    *mut uint32_t,
                                (MPU_Control as libc::c_ulong |
                                     1 as libc::c_ulong) as uint32_t);
    /* Enable fault exceptions */
    let ref mut fresh1 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SHCSR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_ulong |
                                     (1 as libc::c_ulong) <<
                                         16 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Ensure MPU setting take effects */
    __DSB();
    __ISB();
}
/* *
  * @brief  Initializes and configures the Region and the memory to be protected.
  * @param  MPU_Init: Pointer to a MPU_Region_InitTypeDef structure that contains
  *                the initialization and configuration information.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MPU_ConfigRegion(mut MPU_Init:
                                                  *mut MPU_Region_InitTypeDef) {
    /* Check the parameters */
    /* Set the Region number */
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd90
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut MPU_Type)).RNR as
                                    *mut uint32_t,
                                (*MPU_Init).Number as uint32_t);
    if (*MPU_Init).Enable as libc::c_int != RESET as libc::c_int {
        /* Check the parameters */
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0xd90
                                                                                 as
                                                                                 libc::c_ulong)
                                                as *mut MPU_Type)).RBAR as
                                        *mut uint32_t,
                                    (*MPU_Init).BaseAddress);
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0xd90
                                                                                 as
                                                                                 libc::c_ulong)
                                                as *mut MPU_Type)).RASR as
                                        *mut uint32_t,
                                    ((*MPU_Init).DisableExec as uint32_t) <<
                                        28 as libc::c_uint |
                                        ((*MPU_Init).AccessPermission as
                                             uint32_t) << 24 as libc::c_uint |
                                        ((*MPU_Init).TypeExtField as uint32_t)
                                            << 19 as libc::c_uint |
                                        ((*MPU_Init).IsShareable as uint32_t)
                                            << 18 as libc::c_uint |
                                        ((*MPU_Init).IsCacheable as uint32_t)
                                            << 17 as libc::c_uint |
                                        ((*MPU_Init).IsBufferable as uint32_t)
                                            << 16 as libc::c_uint |
                                        ((*MPU_Init).SubRegionDisable as
                                             uint32_t) << 8 as libc::c_uint |
                                        ((*MPU_Init).Size as uint32_t) <<
                                            1 as libc::c_uint |
                                        ((*MPU_Init).Enable as uint32_t) <<
                                            0 as libc::c_uint)
    } else {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0xd90
                                                                                 as
                                                                                 libc::c_ulong)
                                                as *mut MPU_Type)).RBAR as
                                        *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0xd90
                                                                                 as
                                                                                 libc::c_ulong)
                                                as *mut MPU_Type)).RASR as
                                        *mut uint32_t,
                                    0 as libc::c_int as uint32_t)
    };
}
/* __MPU_PRESENT */
/* *
  * @brief  Gets the priority grouping field from the NVIC Interrupt Controller.
  * @retval Priority grouping field (SCB->AIRCR [10:8] PRIGROUP field)
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_GetPriorityGrouping() -> uint32_t {
    /* Get the PRIGROUP[10:8] field value */
    return __NVIC_GetPriorityGrouping();
}
/* *
  * @brief  Gets the priority of an interrupt.
  * @param  IRQn: External interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f7xxxx.h))
  * @param   PriorityGroup: the priority grouping bits length.
  *         This parameter can be one of the following values:
  *           @arg NVIC_PRIORITYGROUP_0: 0 bits for preemption priority
  *                                      4 bits for subpriority
  *           @arg NVIC_PRIORITYGROUP_1: 1 bits for preemption priority
  *                                      3 bits for subpriority
  *           @arg NVIC_PRIORITYGROUP_2: 2 bits for preemption priority
  *                                      2 bits for subpriority
  *           @arg NVIC_PRIORITYGROUP_3: 3 bits for preemption priority
  *                                      1 bits for subpriority
  *           @arg NVIC_PRIORITYGROUP_4: 4 bits for preemption priority
  *                                      0 bits for subpriority
  * @param  pPreemptPriority: Pointer on the Preemptive priority value (starting from 0).
  * @param  pSubPriority: Pointer on the Subpriority value (starting from 0).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_GetPriority(mut IRQn: IRQn_Type,
                                              mut PriorityGroup: uint32_t,
                                              mut pPreemptPriority:
                                                  *mut uint32_t,
                                              mut pSubPriority:
                                                  *mut uint32_t) {
    /* Check the parameters */
    /* Get priority for Cortex-M system or device specific interrupts */
    NVIC_DecodePriority(__NVIC_GetPriority(IRQn), PriorityGroup,
                        pPreemptPriority, pSubPriority);
}
/* *
  * @brief  Sets Pending bit of an external interrupt.
  * @param  IRQn External interrupt number
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f7xxxx.h))
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_SetPendingIRQ(mut IRQn: IRQn_Type) {
    /* Check the parameters */
    /* Set interrupt pending */
    __NVIC_SetPendingIRQ(IRQn);
}
/* *
  * @brief  Gets Pending Interrupt (reads the pending register in the NVIC 
  *         and returns the pending bit for the specified interrupt).
  * @param  IRQn External interrupt number.
  *          This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f7xxxx.h))
  * @retval status: - 0  Interrupt status is not pending.
  *                 - 1  Interrupt status is pending.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_GetPendingIRQ(mut IRQn: IRQn_Type)
 -> uint32_t {
    /* Check the parameters */
    /* Return 1 if pending else 0 */
    return __NVIC_GetPendingIRQ(IRQn);
}
/* *
  * @brief  Clears the pending bit of an external interrupt.
  * @param  IRQn External interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f7xxxx.h))
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_ClearPendingIRQ(mut IRQn: IRQn_Type) {
    /* Check the parameters */
    /* Clear pending interrupt */
    __NVIC_ClearPendingIRQ(IRQn);
}
/* *
  * @brief Gets active interrupt ( reads the active register in NVIC and returns the active bit).
  * @param IRQn External interrupt number
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f7xxxx.h))
  * @retval status: - 0  Interrupt status is not pending.
  *                 - 1  Interrupt status is pending.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_NVIC_GetActive(mut IRQn: IRQn_Type) -> uint32_t {
    /* Check the parameters */
    /* Return 1 if active else 0 */
    return __NVIC_GetActive(IRQn);
}
/* *
  * @brief  Configures the SysTick clock source.
  * @param  CLKSource: specifies the SysTick clock source.
  *          This parameter can be one of the following values:
  *             @arg SYSTICK_CLKSOURCE_HCLK_DIV8: AHB clock divided by 8 selected as SysTick clock source.
  *             @arg SYSTICK_CLKSOURCE_HCLK: AHB clock selected as SysTick clock source.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SYSTICK_CLKSourceConfig(mut CLKSource:
                                                         uint32_t) {
    /* Check the parameters */
    if CLKSource == 0x4 as libc::c_uint {
        let ref mut fresh2 =
            (*((0xe000e000 as
                    libc::c_ulong).wrapping_add(0x10 as libc::c_ulong) as
                   *mut SysTick_Type)).CTRL;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        let ref mut fresh3 =
            (*((0xe000e000 as
                    libc::c_ulong).wrapping_add(0x10 as libc::c_ulong) as
                   *mut SysTick_Type)).CTRL;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    };
}
/* *
  * @brief  This function handles SYSTICK interrupt request.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SYSTICK_IRQHandler() { HAL_SYSTICK_Callback(); }
/* *
  * @}
  */
/* __MPU_PRESENT */
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup CORTEX_Exported_Constants CORTEX Exported Constants
  * @{
  */
/* * @defgroup CORTEX_Preemption_Priority_Group CORTEX Preemption Priority Group
  * @{
  */
/* !< 0 bits for pre-emption priority
                                                                 4 bits for subpriority */
/* !< 1 bits for pre-emption priority
                                                                 3 bits for subpriority */
/* !< 2 bits for pre-emption priority
                                                                 2 bits for subpriority */
/* !< 3 bits for pre-emption priority
                                                                 1 bits for subpriority */
/* !< 4 bits for pre-emption priority
                                                                 0 bits for subpriority */
/* *
  * @}
  */
/* * @defgroup CORTEX_SysTick_clock_source CORTEX _SysTick clock source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_HFNMI_PRIVDEF_Control MPU HFNMI and PRIVILEGED Access control
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_Region_Enable CORTEX MPU Region Enable
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_Instruction_Access CORTEX MPU Instruction Access
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_Access_Shareable CORTEX MPU Instruction Access Shareable
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_Access_Cacheable CORTEX MPU Instruction Access Cacheable
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_Access_Bufferable CORTEX MPU Instruction Access Bufferable
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_TEX_Levels MPU TEX Levels
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_Region_Size CORTEX MPU Region Size
  * @{
  */
/* *                                
  * @}
  */
/* * @defgroup CORTEX_MPU_Region_Permission_Attributes CORTEX MPU Region Permission Attributes 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup CORTEX_MPU_Region_Number CORTEX MPU Region Number
  * @{
  */
/* *
  * @}
  */
/* __MPU_PRESENT */
/* *
  * @}
  */
/* Exported Macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup CORTEX_Exported_Functions
  * @{
  */
/* * @addtogroup CORTEX_Exported_Functions_Group1
 * @{
 */
/* Initialization and de-initialization functions *****************************/
/* *
  * @}
  */
/* * @addtogroup CORTEX_Exported_Functions_Group2
 * @{
 */
/* Peripheral Control functions ***********************************************/
/* __MPU_PRESENT */
/* *
  * @brief  SYSTICK callback.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SYSTICK_Callback() {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SYSTICK_Callback could be implemented in the user file
   */
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_CORTEX_MODULE_ENABLED */
/* *
  * @}
  */
/* *
  * @}
  */
