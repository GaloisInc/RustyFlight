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
    /* !< System Clock Frequency (Core Clock) */
    #[no_mangle]
    static AHBPrescTable: [uint8_t; 16];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SysTick_Type {
    pub CTRL: uint32_t,
    pub LOAD: uint32_t,
    pub VAL: uint32_t,
    pub CALIB: uint32_t,
}
/* *
  * @brief FLASH Registers
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_TypeDef {
    pub ACR: uint32_t,
    pub KEYR: uint32_t,
    pub OPTKEYR: uint32_t,
    pub SR: uint32_t,
    pub CR: uint32_t,
    pub OPTCR: uint32_t,
    pub OPTCR1: uint32_t,
}
/* *
  * @brief Power Control
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PWR_TypeDef {
    pub CR1: uint32_t,
    pub CSR1: uint32_t,
    pub CR2: uint32_t,
    pub CSR2: uint32_t,
}
/* *
  * @brief Reset and Clock Control
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub PLLCFGR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub AHB1RSTR: uint32_t,
    pub AHB2RSTR: uint32_t,
    pub AHB3RSTR: uint32_t,
    pub RESERVED0: uint32_t,
    pub APB1RSTR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub RESERVED1: [uint32_t; 2],
    pub AHB1ENR: uint32_t,
    pub AHB2ENR: uint32_t,
    pub AHB3ENR: uint32_t,
    pub RESERVED2: uint32_t,
    pub APB1ENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub RESERVED3: [uint32_t; 2],
    pub AHB1LPENR: uint32_t,
    pub AHB2LPENR: uint32_t,
    pub AHB3LPENR: uint32_t,
    pub RESERVED4: uint32_t,
    pub APB1LPENR: uint32_t,
    pub APB2LPENR: uint32_t,
    pub RESERVED5: [uint32_t; 2],
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
    pub RESERVED6: [uint32_t; 2],
    pub SSCGR: uint32_t,
    pub PLLI2SCFGR: uint32_t,
    pub PLLSAICFGR: uint32_t,
    pub DCKCFGR1: uint32_t,
    pub DCKCFGR2: uint32_t,
}
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_UTILS_PLLInitTypeDef {
    pub PLLM: uint32_t,
    pub PLLN: uint32_t,
    pub PLLP: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_UTILS_ClkInitTypeDef {
    pub AHBCLKDivider: uint32_t,
    pub APB1CLKDivider: uint32_t,
    pub APB2CLKDivider: uint32_t,
}
#[inline]
unsafe extern "C" fn LL_InitTick(mut HCLKFrequency: uint32_t,
                                 mut Ticks: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0x10
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SysTick_Type)).LOAD as
                                    *mut uint32_t,
                                (HCLKFrequency.wrapping_div(Ticks) as
                                     libc::c_ulong).wrapping_sub(1 as
                                                                     libc::c_ulong)
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
                                     1 as libc::c_ulong) as uint32_t);
}
/* *
  * @brief  Enable HSE external oscillator (HSE Bypass)
  * @rmtoll CR           HSEBYP        LL_RCC_HSE_EnableBypass
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_HSE_EnableBypass() {
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         18 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Disable HSE external oscillator (HSE Bypass)
  * @rmtoll CR           HSEBYP        LL_RCC_HSE_DisableBypass
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_HSE_DisableBypass() {
    let ref mut fresh1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           18 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Enable HSE crystal oscillator (HSE ON)
  * @rmtoll CR           HSEON         LL_RCC_HSE_Enable
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_HSE_Enable() {
    let ref mut fresh2 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         16 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Check if HSE oscillator Ready
  * @rmtoll CR           HSERDY        LL_RCC_HSE_IsReady
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_RCC_HSE_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 17 as libc::c_uint ==
                (0x1 as libc::c_uint) << 17 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_HSI HSI
  * @{
  */
/* *
  * @brief  Enable HSI oscillator
  * @rmtoll CR           HSION         LL_RCC_HSI_Enable
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_HSI_Enable() {
    let ref mut fresh3 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Check if HSI clock is ready
  * @rmtoll CR           HSIRDY        LL_RCC_HSI_IsReady
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_RCC_HSI_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                (0x1 as libc::c_uint) << 1 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_System System
  * @{
  */
/* *
  * @brief  Configure the system clock source
  * @rmtoll CFGR         SW            LL_RCC_SetSysClkSource
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_HSE
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_PLL
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_SetSysClkSource(mut Source: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).CFGR &
                                    !((0x3 as libc::c_uint) <<
                                          0 as libc::c_uint) | Source);
}
/* *
  * @brief  Get the system clock source
  * @rmtoll CFGR         SWS           LL_RCC_GetSysClkSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_HSI
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_HSE
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_PLL
  */
#[inline]
unsafe extern "C" fn LL_RCC_GetSysClkSource() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR &
               (0x3 as libc::c_uint) << 2 as libc::c_uint;
}
/* *
  * @brief  Set AHB prescaler
  * @rmtoll CFGR         HPRE          LL_RCC_SetAHBPrescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_SetAHBPrescaler(mut Prescaler: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).CFGR &
                                    !((0xf as libc::c_uint) <<
                                          4 as libc::c_uint) | Prescaler);
}
/* *
  * @brief  Set APB1 prescaler
  * @rmtoll CFGR         PPRE1         LL_RCC_SetAPB1Prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_SetAPB1Prescaler(mut Prescaler: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).CFGR &
                                    !((0x7 as libc::c_uint) <<
                                          10 as libc::c_uint) | Prescaler);
}
/* *
  * @brief  Set APB2 prescaler
  * @rmtoll CFGR         PPRE2         LL_RCC_SetAPB2Prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB2_DIV_1
  *         @arg @ref LL_RCC_APB2_DIV_2
  *         @arg @ref LL_RCC_APB2_DIV_4
  *         @arg @ref LL_RCC_APB2_DIV_8
  *         @arg @ref LL_RCC_APB2_DIV_16
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_SetAPB2Prescaler(mut Prescaler: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).CFGR &
                                    !((0x7 as libc::c_uint) <<
                                          13 as libc::c_uint) | Prescaler);
}
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_PLL PLL
  * @{
  */
/* *
  * @brief  Enable PLL
  * @rmtoll CR           PLLON         LL_RCC_PLL_Enable
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_PLL_Enable() {
    let ref mut fresh4 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         24 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Check if PLL Ready
  * @rmtoll CR           PLLRDY        LL_RCC_PLL_IsReady
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_RCC_PLL_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 25 as libc::c_uint ==
                (0x1 as libc::c_uint) << 25 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @brief  Configure PLL used for SYSCLK Domain
  * @note PLL Source and PLLM Divider can be written only when PLL,
  *       PLLI2S and PLLSAI are disabled
  * @note PLLN/PLLP can be written only when PLL is disabled
  * @rmtoll PLLCFGR      PLLSRC        LL_RCC_PLL_ConfigDomain_SYS\n
  *         PLLCFGR      PLLM          LL_RCC_PLL_ConfigDomain_SYS\n
  *         PLLCFGR      PLLN          LL_RCC_PLL_ConfigDomain_SYS\n
  *         PLLCFGR      PLLP          LL_RCC_PLL_ConfigDomain_SYS
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  * @param  PLLM This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  PLLN Between 50 and 432
  * @param  PLLP This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLP_DIV_2
  *         @arg @ref LL_RCC_PLLP_DIV_4
  *         @arg @ref LL_RCC_PLLP_DIV_6
  *         @arg @ref LL_RCC_PLLP_DIV_8
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_RCC_PLL_ConfigDomain_SYS(mut Source: uint32_t,
                                                 mut PLLM: uint32_t,
                                                 mut PLLN: uint32_t,
                                                 mut PLLP: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).PLLCFGR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).PLLCFGR &
                                    !((0x1 as libc::c_uint) <<
                                          22 as libc::c_uint |
                                          (0x3f as libc::c_uint) <<
                                              0 as libc::c_uint |
                                          (0x1ff as libc::c_uint) <<
                                              6 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              16 as libc::c_uint) |
                                    (Source | PLLM | PLLN << 6 as libc::c_uint
                                         | PLLP));
}
/* *
  * @brief  Check if PLLI2S Ready
  * @rmtoll CR           PLLI2SRDY    LL_RCC_PLLI2S_IsReady
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_RCC_PLLI2S_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 27 as libc::c_uint ==
                (0x1 as libc::c_uint) << 27 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @brief  Check if PLLSAI Ready
  * @rmtoll CR           PLLSAIRDY    LL_RCC_PLLSAI_IsReady
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_RCC_PLLSAI_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 29 as libc::c_uint ==
                (0x1 as libc::c_uint) << 29 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @}
  */
/* * @defgroup SYSTEM_LL_EF_FLASH FLASH
  * @{
  */
/* *
  * @brief  Set FLASH Latency
  * @rmtoll FLASH_ACR    LATENCY       LL_FLASH_SetLatency
  * @param  Latency This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_LATENCY_0
  *         @arg @ref LL_FLASH_LATENCY_1
  *         @arg @ref LL_FLASH_LATENCY_2
  *         @arg @ref LL_FLASH_LATENCY_3
  *         @arg @ref LL_FLASH_LATENCY_4
  *         @arg @ref LL_FLASH_LATENCY_5
  *         @arg @ref LL_FLASH_LATENCY_6
  *         @arg @ref LL_FLASH_LATENCY_7
  *         @arg @ref LL_FLASH_LATENCY_8
  *         @arg @ref LL_FLASH_LATENCY_9
  *         @arg @ref LL_FLASH_LATENCY_10
  *         @arg @ref LL_FLASH_LATENCY_11
  *         @arg @ref LL_FLASH_LATENCY_12
  *         @arg @ref LL_FLASH_LATENCY_13
  *         @arg @ref LL_FLASH_LATENCY_14
  *         @arg @ref LL_FLASH_LATENCY_15
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_FLASH_SetLatency(mut Latency: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut FLASH_TypeDef)).ACR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3c00
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut FLASH_TypeDef)).ACR &
                                    !((0xf as libc::c_uint) <<
                                          0 as libc::c_uint) | Latency);
}
/* *
  * @brief  Get FLASH Latency
  * @rmtoll FLASH_ACR    LATENCY       LL_FLASH_GetLatency
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_LATENCY_0
  *         @arg @ref LL_FLASH_LATENCY_1
  *         @arg @ref LL_FLASH_LATENCY_2
  *         @arg @ref LL_FLASH_LATENCY_3
  *         @arg @ref LL_FLASH_LATENCY_4
  *         @arg @ref LL_FLASH_LATENCY_5
  *         @arg @ref LL_FLASH_LATENCY_6
  *         @arg @ref LL_FLASH_LATENCY_7
  *         @arg @ref LL_FLASH_LATENCY_8
  *         @arg @ref LL_FLASH_LATENCY_9
  *         @arg @ref LL_FLASH_LATENCY_10
  *         @arg @ref LL_FLASH_LATENCY_11
  *         @arg @ref LL_FLASH_LATENCY_12
  *         @arg @ref LL_FLASH_LATENCY_13
  *         @arg @ref LL_FLASH_LATENCY_14
  *         @arg @ref LL_FLASH_LATENCY_15
  */
#[inline]
unsafe extern "C" fn LL_FLASH_GetLatency() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).ACR &
               (0xf as libc::c_uint) << 0 as libc::c_uint;
}
/* *
  * @brief  Check if Over drive switching is enabled
  * @rmtoll CR1    ODEN       LL_PWR_IsEnabledOverDriveMode
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_PWR_IsEnabledOverDriveMode() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                   *mut PWR_TypeDef)).CR1 &
                (0x1 as libc::c_uint) << 16 as libc::c_uint ==
                (0x1 as libc::c_uint) << 16 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @brief  Get the main internal Regulator output voltage
  * @rmtoll CR1    VOS       LL_PWR_GetRegulVoltageScaling
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_REGU_VOLTAGE_SCALE1
  *         @arg @ref LL_PWR_REGU_VOLTAGE_SCALE2
  *         @arg @ref LL_PWR_REGU_VOLTAGE_SCALE3
  */
#[inline]
unsafe extern "C" fn LL_PWR_GetRegulVoltageScaling() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                  *mut PWR_TypeDef)).CR1 &
               (0x3 as libc::c_uint) << 14 as libc::c_uint;
}
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup UTILS_LL_Exported_Functions
  * @{
  */
/* * @addtogroup UTILS_LL_EF_DELAY
  * @{
  */
/* *
  * @brief  This function configures the Cortex-M SysTick source to have 1ms time base.
  * @note   When a RTOS is used, it is recommended to avoid changing the Systick
  *         configuration by calling this function, for a delay use rather osDelay RTOS service.
  * @param  HCLKFrequency HCLK frequency in Hz
  * @note   HCLK frequency can be calculated thanks to RCC helper macro or function @ref LL_RCC_GetSystemClocksFreq
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_Init1msTick(mut HCLKFrequency: uint32_t) {
    /* Use frequency provided in argument */
    LL_InitTick(HCLKFrequency, 1000 as libc::c_uint);
}
/* *
  * @brief  This function provides accurate delay (in milliseconds) based
  *         on SysTick counter flag
  * @note   When a RTOS is used, it is recommended to avoid using blocking delay
  *         and use rather osDelay service.
  * @note   To respect 1ms timebase, user should call @ref LL_Init1msTick function which
  *         will configure Systick to 1ms
  * @param  Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_mDelay(mut Delay: uint32_t) {
    let mut tmp: uint32_t =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0x10 as libc::c_ulong)
               as *mut SysTick_Type)).CTRL; /* Clear the COUNTFLAG first */
    /* Add this code to indicate that local variable is not used */
    /* Add a period to guaranty minimum wait */
    if Delay < 0xffffffff as libc::c_uint { Delay = Delay.wrapping_add(1) }
    while Delay != 0 {
        if (*((0xe000e000 as
                   libc::c_ulong).wrapping_add(0x10 as libc::c_ulong) as
                  *mut SysTick_Type)).CTRL as libc::c_ulong &
               (1 as libc::c_ulong) << 16 as libc::c_uint !=
               0 as libc::c_uint as libc::c_ulong {
            Delay = Delay.wrapping_sub(1)
        }
    };
}
/* *
  * @}
  */
/* * @addtogroup UTILS_EF_SYSTEM
  *  @brief    System Configuration functions
  *
  @verbatim
 ===============================================================================
           ##### System Configuration functions #####
 ===============================================================================
    [..]
         System, AHB and APB buses clocks configuration

         (+) The maximum frequency of the SYSCLK, HCLK, PCLK1 and PCLK2 is 216000000 Hz.
  @endverbatim
  @internal
             Depending on the device voltage range, the maximum frequency should be
             adapted accordingly:
             (++) +------------------------------------------------------------------------------------------------+
             (++) |  Wait states   |                           HCLK clock frequency (MHz)                          |
             (++) |                |-------------------------------------------------------------------------------|
             (++) |  (Latency)     |   voltage range   |   voltage range   |   voltage range   |   voltage range   |
             (++) |                |    2.7V - 3.6V    |    2.4V - 2.7V    |    2.1V - 2.7V    |    1.8V - 2.1V    |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |0WS(1CPU cycle) |   0 < HCLK <= 30  |   0 < HCLK <= 24  |   0 < HCLK <= 22  |   0 < HCLK <= 20  |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |1WS(2CPU cycle) |  30 < HCLK <= 60  |  24 < HCLK <= 48  |  22 < HCLK <= 44  |  20 < HCLK <= 44  |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |2WS(3CPU cycle) |  60 < HCLK <= 90  |  48 < HCLK <= 72  |  44 < HCLK <= 66  |  40 < HCLK <= 60  |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |3WS(4CPU cycle) |  90 < HCLK <= 120 |  72 < HCLK <= 96  |  66 < HCLK <= 88  |  60 < HCLK <= 80  |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |4WS(5CPU cycle) | 120 < HCLK <= 150 |  96 < HCLK <= 120 |  88 < HCLK <= 110 |  80 < HCLK <= 100 |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |5WS(6CPU cycle) | 150 < HCLK <= 180 | 120 < HCLK <= 144 | 110 < HCLK <= 132 | 100 < HCLK <= 120 |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |6WS(7CPU cycle) | 180 < HCLK <= 210 | 144 < HCLK <= 168 | 132 < HCLK <= 154 | 120 < HCLK <= 140 |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |7WS(8CPU cycle) | 210 < HCLK <= 216 | 168 < HCLK <= 192 | 154 < HCLK <= 176 | 140 < HCLK <= 160 |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |8WS(9CPU cycle) |        --         | 192 < HCLK <= 216 | 176 < HCLK <= 198 | 160 < HCLK <= 180 |
             (++) |----------------|-------------------|-------------------|-------------------|-------------------|
             (++) |9WS(10CPU cycle)|        --         |         --        | 198 < HCLK <= 216 |         --        |
             (++) +------------------------------------------------------------------------------------------------+

  @endinternal
  * @{
  */
/* *
  * @brief  This function sets directly SystemCoreClock CMSIS variable.
  * @note   Variable can be calculated also through SystemCoreClockUpdate function.
  * @param  HCLKFrequency HCLK frequency in Hz (can be calculated thanks to RCC helper macro)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_SetSystemCoreClock(mut HCLKFrequency: uint32_t) {
    /* HCLK clock frequency */
    SystemCoreClock = HCLKFrequency;
}
/* *
  * @brief  This function configures system clock at maximum frequency with HSI as clock source of the PLL
  * @note   The application need to ensure that PLL is disabled.
  * @note   Function is based on the following formula:
  *         - PLL output frequency = (((HSI frequency / PLLM) * PLLN) / PLLP)
  *         - PLLM: ensure that the VCO input frequency ranges from 0.95 to 2.1 MHz (PLLVCO_input = HSI frequency / PLLM)
  *         - PLLN: ensure that the VCO output frequency is between 100 and 432 MHz (PLLVCO_output = PLLVCO_input * PLLN)
  *         - PLLP: ensure that max frequency at 216000000 Hz is reach (PLLVCO_output / PLLP)
  * @param  UTILS_PLLInitStruct pointer to a @ref LL_UTILS_PLLInitTypeDef structure that contains
  *                             the configuration information for the PLL.
  * @param  UTILS_ClkInitStruct pointer to a @ref LL_UTILS_ClkInitTypeDef structure that contains
  *                             the configuration information for the BUS prescalers.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Max frequency configuration done
  *          - ERROR: Max frequency configuration not done
  */
#[no_mangle]
pub unsafe extern "C" fn LL_PLL_ConfigSystemClock_HSI(mut UTILS_PLLInitStruct:
                                                          *mut LL_UTILS_PLLInitTypeDef,
                                                      mut UTILS_ClkInitStruct:
                                                          *mut LL_UTILS_ClkInitTypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = SUCCESS;
    let mut pllfreq: uint32_t = 0 as libc::c_uint;
    /* Check if one of the PLL is enabled */
    if UTILS_PLL_IsBusy() as libc::c_uint ==
           SUCCESS as libc::c_int as libc::c_uint {
        /* Calculate the new PLL output frequency */
        pllfreq =
            UTILS_GetPLLOutputFrequency(16000000 as libc::c_uint,
                                        UTILS_PLLInitStruct);
        /* Enable HSI if not enabled */
        if LL_RCC_HSI_IsReady() != 1 as libc::c_uint {
            LL_RCC_HSI_Enable();
            /* Wait for HSI ready */
            while LL_RCC_HSI_IsReady() != 1 as libc::c_uint { }
        }
        /* Configure PLL */
        LL_RCC_PLL_ConfigDomain_SYS(0 as libc::c_uint,
                                    (*UTILS_PLLInitStruct).PLLM,
                                    (*UTILS_PLLInitStruct).PLLN,
                                    (*UTILS_PLLInitStruct).PLLP);
        /* Enable PLL and switch system clock to PLL */
        status = UTILS_EnablePLLAndSwitchSystem(pllfreq, UTILS_ClkInitStruct)
    } else {
        /* Current PLL configuration cannot be modified */
        status = ERROR
    }
    return status;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_utils.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of UTILS LL module.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The LL UTILS driver contains a set of generic APIs that can be
    used by user:
      (+) Device electronic signature
      (+) Timing functions
      (+) PLL configuration functions

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
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_LL_Driver
  * @{
  */
/* * @defgroup UTILS_LL UTILS
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* * @defgroup UTILS_LL_Private_Constants UTILS Private Constants
  * @{
  */
/* Max delay can be used in LL_mDelay */
/* *
 * @brief Unique device ID register base address
 */
/* *
 * @brief Flash size data register base address
 */
/* *
 * @brief Package data register base address
 */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @defgroup UTILS_LL_Private_Macros UTILS Private Macros
  * @{
  */
/* *
  * @}
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup UTILS_LL_ES_INIT UTILS Exported structures
  * @{
  */
/* *
  * @brief  UTILS PLL structure definition
  */
/* !< Division factor for PLL VCO input clock.
                        This parameter can be a value of @ref RCC_LL_EC_PLLM_DIV

                        This feature can be modified afterwards using unitary function
                        @ref LL_RCC_PLL_ConfigDomain_SYS(). */
/* !< Multiplication factor for PLL VCO output clock.
                        This parameter must be a number between Min_Data = 50 and Max_Data = 432

                        This feature can be modified afterwards using unitary function
                        @ref LL_RCC_PLL_ConfigDomain_SYS(). */
/* !< Division for the main system clock.
                        This parameter can be a value of @ref RCC_LL_EC_PLLP_DIV

                        This feature can be modified afterwards using unitary function
                        @ref LL_RCC_PLL_ConfigDomain_SYS(). */
/* *
  * @brief  UTILS System, AHB and APB buses clock configuration structure definition
  */
/* !< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_LL_EC_SYSCLK_DIV

                                       This feature can be modified afterwards using unitary function
                                       @ref LL_RCC_SetAHBPrescaler(). */
/* !< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_LL_EC_APB1_DIV

                                       This feature can be modified afterwards using unitary function
                                       @ref LL_RCC_SetAPB1Prescaler(). */
/* !< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_LL_EC_APB2_DIV

                                       This feature can be modified afterwards using unitary function
                                       @ref LL_RCC_SetAPB2Prescaler(). */
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup UTILS_LL_Exported_Constants UTILS Exported Constants
  * @{
  */
/* * @defgroup UTILS_EC_HSE_BYPASS HSE Bypass activation
  * @{
  */
/* !< HSE Bypass is not enabled                */
/* !< HSE Bypass is enabled                    */
/* *
  * @}
  */
/* * @defgroup UTILS_EC_PACKAGETYPE PACKAGE TYPE
  * @{
  */
/* !< LQFP100 package type                         */
/* !< LQFP144 or WLCSP143 package type             */
/* !< WLCSP180, LQFP176 or UFBGA176 package type   */
/* !< LQFP176, LQFP208 or TFBGA216 package type    */
/* !< LQFP176, LQFP208 or TFBGA216 package type    */
/* !< LQFP176, LQFP208 or TFBGA216 package type    */
/* !< LQFP176, LQFP208 or TFBGA216 package type    */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @defgroup UTILS_LL_Exported_Functions UTILS Exported Functions
  * @{
  */
/* * @defgroup UTILS_EF_DEVICE_ELECTRONIC_SIGNATURE DEVICE ELECTRONIC SIGNATURE
  * @{
  */
/* *
  * @brief  Get Word0 of the unique device identifier (UID based on 96 bits)
  * @retval UID[31:0]
  */
/* *
  * @brief  Get Word1 of the unique device identifier (UID based on 96 bits)
  * @retval UID[63:32]
  */
/* *
  * @brief  Get Word2 of the unique device identifier (UID based on 96 bits)
  * @retval UID[95:64]
  */
/* *
  * @brief  Get Flash memory size
  * @note   This bitfield indicates the size of the device Flash memory expressed in
  *         Kbytes. As an example, 0x040 corresponds to 64 Kbytes.
  * @retval FLASH_SIZE[15:0]: Flash memory size
  */
/* *
  * @brief  Get Package type
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_UTILS_PACKAGETYPE_LQFP100
  *         @arg @ref LL_UTILS_PACKAGETYPE_LQFP144_WLCSP143 (*)
  *         @arg @ref LL_UTILS_PACKAGETYPE_WLCSP180_LQFP176_UFBGA176 (*)
  *         @arg @ref LL_UTILS_PACKAGETYPE_LQFP176_LQFP208_TFBGA216 (*)
  *
  *         (*) value not defined in all devices.
  */
/* *
  * @}
  */
/* * @defgroup UTILS_LL_EF_DELAY DELAY
  * @{
  */
/* *
  * @brief  This function configures the Cortex-M SysTick source of the time base.
  * @param  HCLKFrequency HCLK frequency in Hz (can be calculated thanks to RCC helper macro)
  * @note   When a RTOS is used, it is recommended to avoid changing the SysTick 
  *         configuration by calling this function, for a delay use rather osDelay RTOS service.
  * @param  Ticks Number of ticks
  * @retval None
  */
/* Configure the SysTick to have interrupt in 1ms time base */
/* set reload register */
/* Load the SysTick Counter Value */
/* Enable the Systick Timer */
/* *
  * @}
  */
/* * @defgroup UTILS_EF_SYSTEM SYSTEM
  * @{
  */
/* *
  * @brief  This function configures system clock with HSE as clock source of the PLL
  * @note   The application need to ensure that PLL is disabled.
  * @note   Function is based on the following formula:
  *         - PLL output frequency = (((HSE frequency / PLLM) * PLLN) / PLLP)
  *         - PLLM: ensure that the VCO input frequency ranges from 0.95 to 2.10 MHz (PLLVCO_input = HSE frequency / PLLM)
  *         - PLLN: ensure that the VCO output frequency is between 100 and 432 MHz (PLLVCO_output = PLLVCO_input * PLLN)
  *         - PLLP: ensure that max frequency at 216000000 Hz is reached (PLLVCO_output / PLLP)
  * @param  HSEFrequency Value between Min_Data = 4000000 and Max_Data = 26000000
  * @param  HSEBypass This parameter can be one of the following values:
  *         @arg @ref LL_UTILS_HSEBYPASS_ON
  *         @arg @ref LL_UTILS_HSEBYPASS_OFF
  * @param  UTILS_PLLInitStruct pointer to a @ref LL_UTILS_PLLInitTypeDef structure that contains
  *                             the configuration information for the PLL.
  * @param  UTILS_ClkInitStruct pointer to a @ref LL_UTILS_ClkInitTypeDef structure that contains
  *                             the configuration information for the BUS prescalers.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Max frequency configuration done
  *          - ERROR: Max frequency configuration not done
  */
#[no_mangle]
pub unsafe extern "C" fn LL_PLL_ConfigSystemClock_HSE(mut HSEFrequency:
                                                          uint32_t,
                                                      mut HSEBypass: uint32_t,
                                                      mut UTILS_PLLInitStruct:
                                                          *mut LL_UTILS_PLLInitTypeDef,
                                                      mut UTILS_ClkInitStruct:
                                                          *mut LL_UTILS_ClkInitTypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = SUCCESS;
    let mut pllfreq: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Check if one of the PLL is enabled */
    if UTILS_PLL_IsBusy() as libc::c_uint ==
           SUCCESS as libc::c_int as libc::c_uint {
        /* Calculate the new PLL output frequency */
        pllfreq =
            UTILS_GetPLLOutputFrequency(HSEFrequency, UTILS_PLLInitStruct);
        /* Enable HSE if not enabled */
        if LL_RCC_HSE_IsReady() != 1 as libc::c_uint {
            /* Check if need to enable HSE bypass feature or not */
            if HSEBypass == 0x1 as libc::c_uint {
                LL_RCC_HSE_EnableBypass();
            } else { LL_RCC_HSE_DisableBypass(); }
            /* Enable HSE */
            LL_RCC_HSE_Enable();
            /* Wait for HSE ready */
            while LL_RCC_HSE_IsReady() != 1 as libc::c_uint { }
        }
        /* Configure PLL */
        LL_RCC_PLL_ConfigDomain_SYS((0x1 as libc::c_uint) <<
                                        22 as libc::c_uint,
                                    (*UTILS_PLLInitStruct).PLLM,
                                    (*UTILS_PLLInitStruct).PLLN,
                                    (*UTILS_PLLInitStruct).PLLP);
        /* Enable PLL and switch system clock to PLL */
        status = UTILS_EnablePLLAndSwitchSystem(pllfreq, UTILS_ClkInitStruct)
    } else {
        /* Current PLL configuration cannot be modified */
        status = ERROR
    }
    return status;
}
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup UTILS_LL_Private_Functions
  * @{
  */
/* *
  * @brief  Update number of Flash wait states in line with new frequency and current
            voltage range.
  * @note   This Function support ONLY devices with supply voltage (voltage range) between 2.7V and 3.6V
  * @param  HCLK_Frequency  HCLK frequency
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Latency has been modified
  *          - ERROR: Latency cannot be modified
  */
unsafe extern "C" fn UTILS_SetFlashLatency(mut HCLK_Frequency: uint32_t)
 -> ErrorStatus {
    let mut status: ErrorStatus = SUCCESS; /* default value 0WS */
    let mut latency: uint32_t = 0 as libc::c_uint;
    /* Frequency cannot be equal to 0 */
    if HCLK_Frequency == 0 as libc::c_uint {
        status = ERROR
    } else {
        if LL_PWR_GetRegulVoltageScaling() ==
               (0x1 as libc::c_uint) << 14 as libc::c_uint |
                   (0x2 as libc::c_uint) << 14 as libc::c_uint {
            if LL_PWR_IsEnabledOverDriveMode() != 0 as libc::c_uint {
                if HCLK_Frequency > 210000000 as libc::c_uint {
                    /* 210 < HCLK <= 216 => 7WS (8 CPU cycles) */
                    latency = 0x7 as libc::c_uint
                } else {
                    /* (HCLK_Frequency > UTILS_SCALE1_LATENCY6_FREQ) */
                    /* 180 < HCLK <= 210 => 6WS (7 CPU cycles) */
                    latency = 0x6 as libc::c_uint
                }
            }
            if HCLK_Frequency > 150000000 as libc::c_uint &&
                   latency == 0 as libc::c_uint {
                /* 150 < HCLK <= 180 => 5WS (6 CPU cycles) */
                latency = 0x5 as libc::c_uint
            } else if HCLK_Frequency > 120000000 as libc::c_uint &&
                          latency == 0 as libc::c_uint {
                /* 120 < HCLK <= 150 => 4WS (5 CPU cycles) */
                latency = 0x4 as libc::c_uint
            } else if HCLK_Frequency > 90000000 as libc::c_uint &&
                          latency == 0 as libc::c_uint {
                /* 90 < HCLK <= 120 => 3WS (4 CPU cycles) */
                latency = 0x3 as libc::c_uint
            } else if HCLK_Frequency > 60000000 as libc::c_uint &&
                          latency == 0 as libc::c_uint {
                /* 60 < HCLK <= 90 => 2WS (3 CPU cycles) */
                latency = 0x2 as libc::c_uint
            } else if HCLK_Frequency > 30000000 as libc::c_uint &&
                          latency == 0 as libc::c_uint {
                /* 30 < HCLK <= 60 => 1WS (2 CPU cycles) */
                latency = 0x1 as libc::c_uint
            }
        } else if LL_PWR_GetRegulVoltageScaling() ==
                      (0x2 as libc::c_uint) << 14 as libc::c_uint {
            if HCLK_Frequency > 150000000 as libc::c_uint {
                /* else HCLK_Frequency < 30MHz default LL_FLASH_LATENCY_0 0WS */
                /* 150 < HCLK <= 168 OR 150 < HCLK <= 180 (when OverDrive mode is enable) => 5WS (6 CPU cycles) */
                latency = 0x5 as libc::c_uint
            } else if HCLK_Frequency > 120000000 as libc::c_uint {
                /* 120 < HCLK <= 150 => 4WS (5 CPU cycles) */
                latency = 0x4 as libc::c_uint
            } else if HCLK_Frequency > 90000000 as libc::c_uint {
                /* 90 < HCLK <= 120 => 3WS (4 CPU cycles) */
                latency = 0x3 as libc::c_uint
            } else if HCLK_Frequency > 60000000 as libc::c_uint {
                /* 60 < HCLK <= 90 => 2WS (3 CPU cycles) */
                latency = 0x2 as libc::c_uint
            } else if HCLK_Frequency > 30000000 as libc::c_uint {
                /* 30 < HCLK <= 60 => 1WS (2 CPU cycles) */
                latency = 0x1 as libc::c_uint
            }
        } else if HCLK_Frequency > 120000000 as libc::c_uint {
            /* else HCLK_Frequency < 24MHz default LL_FLASH_LATENCY_0 0WS */
            /* Scale 3 */
            /* 120 < HCLK <= 144 => 4WS (5 CPU cycles) */
            latency = 0x4 as libc::c_uint
        } else if HCLK_Frequency > 90000000 as libc::c_uint {
            /* 90 < HCLK <= 120 => 3WS (4 CPU cycles) */
            latency = 0x3 as libc::c_uint
        } else if HCLK_Frequency > 60000000 as libc::c_uint {
            /* 60 < HCLK <= 90 => 2WS (3 CPU cycles) */
            latency = 0x2 as libc::c_uint
        } else if HCLK_Frequency > 30000000 as libc::c_uint {
            /* 30 < HCLK <= 60 => 1WS (2 CPU cycles) */
            latency = 0x1 as libc::c_uint
        }
        LL_FLASH_SetLatency(latency);
        /* else HCLK_Frequency < 22MHz default LL_FLASH_LATENCY_0 0WS */
        /* Check that the new number of wait states is taken into account to access the Flash
       memory by reading the FLASH_ACR register */
        if LL_FLASH_GetLatency() != latency { status = ERROR }
    }
    return status;
}
/* !< HCLK frequency to set FLASH latency 4 in power scale 3  */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @addtogroup UTILS_LL_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* * @defgroup UTILS_LL_Private_Functions UTILS Private functions
  * @{
  */
/* *
  * @brief  Function to check that PLL can be modified
  * @param  PLL_InputFrequency  PLL input frequency (in Hz)
  * @param  UTILS_PLLInitStruct pointer to a @ref LL_UTILS_PLLInitTypeDef structure that contains
  *                             the configuration information for the PLL.
  * @retval PLL output frequency (in Hz)
  */
unsafe extern "C" fn UTILS_GetPLLOutputFrequency(mut PLL_InputFrequency:
                                                     uint32_t,
                                                 mut UTILS_PLLInitStruct:
                                                     *mut LL_UTILS_PLLInitTypeDef)
 -> uint32_t {
    let mut pllfreq: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Check different PLL parameters according to RM                          */
  /*  - PLLM: ensure that the VCO input frequency ranges from 0.95 to 2.1 MHz.   */
    pllfreq =
        PLL_InputFrequency.wrapping_div((*UTILS_PLLInitStruct).PLLM &
                                            (0x3f as libc::c_uint) <<
                                                0 as libc::c_uint >>
                                                0 as libc::c_uint);
    /*  - PLLN: ensure that the VCO output frequency is between 100 and 432 MHz.*/
    pllfreq =
        pllfreq.wrapping_mul((*UTILS_PLLInitStruct).PLLN &
                                 (0x1ff as libc::c_uint) << 6 as libc::c_uint
                                     >> 6 as libc::c_uint);
    /*  - PLLP: ensure that max frequency at 216000000 Hz is reached     */
    pllfreq =
        pllfreq.wrapping_div(((*UTILS_PLLInitStruct).PLLP >>
                                  16 as
                                      libc::c_uint).wrapping_add(1 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_mul(2
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint));
    return pllfreq;
}
/* *
  * @brief  Function to check that PLL can be modified
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: PLL modification can be done
  *          - ERROR: PLL is busy
  */
unsafe extern "C" fn UTILS_PLL_IsBusy() -> ErrorStatus {
    let mut status: ErrorStatus = SUCCESS;
    /* Check if PLL is busy*/
    if LL_RCC_PLL_IsReady() != 0 as libc::c_uint {
        /* PLL configuration cannot be modified */
        status = ERROR
    }
    /* Check if PLLSAI is busy*/
    if LL_RCC_PLLSAI_IsReady() != 0 as libc::c_uint {
        /* PLLSAI1 configuration cannot be modified */
        status = ERROR
    }
    /* Check if PLLI2S is busy*/
    if LL_RCC_PLLI2S_IsReady() != 0 as libc::c_uint {
        /* PLLI2S configuration cannot be modified */
        status = ERROR
    }
    return status;
}
/* *
  * @brief  Function to enable PLL and switch system clock to PLL
  * @param  SYSCLK_Frequency SYSCLK frequency
  * @param  UTILS_ClkInitStruct pointer to a @ref LL_UTILS_ClkInitTypeDef structure that contains
  *                             the configuration information for the BUS prescalers.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: No problem to switch system to PLL
  *          - ERROR: Problem to switch system to PLL
  */
unsafe extern "C" fn UTILS_EnablePLLAndSwitchSystem(mut SYSCLK_Frequency:
                                                        uint32_t,
                                                    mut UTILS_ClkInitStruct:
                                                        *mut LL_UTILS_ClkInitTypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = SUCCESS;
    let mut hclk_frequency: uint32_t = 0 as libc::c_uint;
    /* Calculate HCLK frequency */
    hclk_frequency =
        SYSCLK_Frequency >>
            AHBPrescTable[(((*UTILS_ClkInitStruct).AHBCLKDivider &
                                (0xf as libc::c_uint) << 4 as libc::c_uint) >>
                               4 as libc::c_uint) as usize] as libc::c_int;
    /* Increasing the number of wait states because of higher CPU frequency */
    if SystemCoreClock < hclk_frequency {
        /* Set FLASH latency to highest latency */
        status = UTILS_SetFlashLatency(hclk_frequency)
    }
    /* Update system clock configuration */
    if status as libc::c_uint == SUCCESS as libc::c_int as libc::c_uint {
        /* Enable PLL */
        LL_RCC_PLL_Enable();
        /* Wait for PLL ready */
        while LL_RCC_PLL_IsReady() != 1 as libc::c_uint { }
        /* Sysclk activation on the main PLL */
        LL_RCC_SetAHBPrescaler((*UTILS_ClkInitStruct).AHBCLKDivider);
        LL_RCC_SetSysClkSource(0x2 as libc::c_uint);
        /* Wait for system clock switch to PLL */
        while LL_RCC_GetSysClkSource() != 0x8 as libc::c_uint { }
        /* Set APB1 & APB2 prescaler*/
        LL_RCC_SetAPB1Prescaler((*UTILS_ClkInitStruct).APB1CLKDivider);
        LL_RCC_SetAPB2Prescaler((*UTILS_ClkInitStruct).APB2CLKDivider);
    }
    /* Decreasing the number of wait states because of lower CPU frequency */
    if SystemCoreClock > hclk_frequency {
        /* Set FLASH latency to lowest latency */
        status = UTILS_SetFlashLatency(hclk_frequency)
    }
    /* Update SystemCoreClock variable */
    if status as libc::c_uint == SUCCESS as libc::c_int as libc::c_uint {
        LL_SetSystemCoreClock(hclk_frequency);
    }
    return status;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
