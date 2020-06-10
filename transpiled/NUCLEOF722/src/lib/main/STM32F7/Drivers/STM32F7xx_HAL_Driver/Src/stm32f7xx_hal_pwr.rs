use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
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
/* *
  * @brief External Interrupt/Event Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EXTI_TypeDef {
    pub IMR: uint32_t,
    pub EMR: uint32_t,
    pub RTSR: uint32_t,
    pub FTSR: uint32_t,
    pub SWIER: uint32_t,
    pub PR: uint32_t,
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
pub type C2RustUnnamed = libc::c_uint;
pub const SET: C2RustUnnamed = 1;
pub const RESET: C2RustUnnamed = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_pwr.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of PWR HAL module.
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
/* * @addtogroup PWR
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup PWR_Exported_Types PWR Exported Types
  * @{
  */
/* *
  * @brief  PWR PVD configuration structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PWR_PVDTypeDef {
    pub PVDLevel: uint32_t,
    pub Mode: uint32_t,
}
/* *
  * @}
  */
/* * @defgroup PWR_ENABLE_WUP_Mask PWR Enable WUP Mask
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup PWR_Exported_Functions PWR Exported Functions
  * @{
  */
/* * @defgroup PWR_Exported_Functions_Group1 Initialization and de-initialization functions 
  *  @brief    Initialization and de-initialization functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
      After reset, the backup domain (RTC registers, RTC backup data 
      registers and backup SRAM) is protected against possible unwanted 
      write accesses. 
      To enable access to the RTC Domain and RTC registers, proceed as follows:
        (+) Enable the Power Controller (PWR) APB1 interface clock using the
            __HAL_RCC_PWR_CLK_ENABLE() macro.
        (+) Enable access to RTC domain using the HAL_PWR_EnableBkUpAccess() function.
 
@endverbatim
  * @{
  */
/* *
  * @brief Deinitializes the HAL PWR peripheral registers to their default reset values.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_DeInit() {
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1RSTR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         28 as libc::c_uint) as uint32_t as
                                    uint32_t);
    let ref mut fresh1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1RSTR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           28 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Enables access to the backup domain (RTC registers, RTC 
  *         backup data registers and backup SRAM).
  * @note If the HSE divided by 2, 3, ..31 is used as the RTC clock, the 
  *         Backup Domain Access should be kept enabled.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_EnableBkUpAccess() {
    /* Enable access to RTC and backup registers */
    let ref mut fresh2 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Disables access to the backup domain (RTC registers, RTC 
  *         backup data registers and backup SRAM).
  * @note If the HSE divided by 2, 3, ..31 is used as the RTC clock, the 
  *         Backup Domain Access should be kept enabled.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_DisableBkUpAccess() {
    /* Disable access to RTC and backup registers */
    let ref mut fresh3 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @}
  */
/* * @defgroup PWR_Exported_Functions_Group2 Peripheral Control functions 
  *  @brief Low Power modes configuration functions 
  *
@verbatim

 ===============================================================================
                 ##### Peripheral Control functions #####
 ===============================================================================
     
    *** PVD configuration ***
    =========================
    [..]
      (+) The PVD is used to monitor the VDD power supply by comparing it to a 
          threshold selected by the PVD Level (PLS[2:0] bits in the PWR_CR).
      (+) A PVDO flag is available to indicate if VDD/VDDA is higher or lower 
          than the PVD threshold. This event is internally connected to the EXTI 
          line16 and can generate an interrupt if enabled. This is done through
          __HAL_PWR_PVD_EXTI_ENABLE_IT() macro.
      (+) The PVD is stopped in Standby mode.

    *** Wake-up pin configuration ***
    ================================
    [..]
      (+) Wake-up pin is used to wake up the system from Standby mode. This pin is 
          forced in input pull-down configuration and is active on rising edges.
      (+) There are up to 6 Wake-up pin in the STM32F7 devices family

    *** Low Power modes configuration ***
    =====================================
    [..]
      The devices feature 3 low-power modes:
      (+) Sleep mode: Cortex-M7 core stopped, peripherals kept running.
      (+) Stop mode: all clocks are stopped, regulator running, regulator 
          in low power mode
      (+) Standby mode: 1.2V domain powered off.
   
   *** Sleep mode ***
   ==================
    [..]
      (+) Entry:
        The Sleep mode is entered by using the HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI)
              functions with
          (++) PWR_SLEEPENTRY_WFI: enter SLEEP mode with WFI instruction
          (++) PWR_SLEEPENTRY_WFE: enter SLEEP mode with WFE instruction
      
      -@@- The Regulator parameter is not used for the STM32F7 family 
              and is kept as parameter just to maintain compatibility with the 
              lower power families (STM32L).
      (+) Exit:
        Any peripheral interrupt acknowledged by the nested vectored interrupt 
              controller (NVIC) can wake up the device from Sleep mode.

   *** Stop mode ***
   =================
    [..]
      In Stop mode, all clocks in the 1.2V domain are stopped, the PLL, the HSI,
      and the HSE RC oscillators are disabled. Internal SRAM and register contents 
      are preserved.
      The voltage regulator can be configured either in normal or low-power mode.
      To minimize the consumption In Stop mode, FLASH can be powered off before 
      entering the Stop mode using the HAL_PWREx_EnableFlashPowerDown() function.
      It can be switched on again by software after exiting the Stop mode using
      the HAL_PWREx_DisableFlashPowerDown() function. 

      (+) Entry:
         The Stop mode is entered using the HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON) 
             function with:
          (++) Main regulator ON.
          (++) Low Power regulator ON.
      (+) Exit:
        Any EXTI Line (Internal or External) configured in Interrupt/Event mode.

   *** Standby mode ***
   ====================
    [..]
    (+)
      The Standby mode allows to achieve the lowest power consumption. It is based 
      on the Cortex-M7 deep sleep mode, with the voltage regulator disabled. 
      The 1.2V domain is consequently powered off. The PLL, the HSI oscillator and 
      the HSE oscillator are also switched off. SRAM and register contents are lost 
      except for the RTC registers, RTC backup registers, backup SRAM and Standby 
      circuitry.
   
      The voltage regulator is OFF.
      
      (++) Entry:
        (+++) The Standby mode is entered using the HAL_PWR_EnterSTANDBYMode() function.
      (++) Exit:
        (+++) WKUP pin rising or falling edge, RTC alarm (Alarm A and Alarm B), RTC
             wakeup, tamper event, time stamp event, external reset in NRST pin, IWDG reset.

   *** Auto-wakeup (AWU) from low-power mode ***
   =============================================
    [..]
    
     (+) The MCU can be woken up from low-power mode by an RTC Alarm event, an RTC 
      Wakeup event, a tamper event or a time-stamp event, without depending on 
      an external interrupt (Auto-wakeup mode).

      (+) RTC auto-wakeup (AWU) from the Stop and Standby modes
       
        (++) To wake up from the Stop mode with an RTC alarm event, it is necessary to 
              configure the RTC to generate the RTC alarm using the HAL_RTC_SetAlarm_IT() function.

        (++) To wake up from the Stop mode with an RTC Tamper or time stamp event, it 
             is necessary to configure the RTC to detect the tamper or time stamp event using the
                HAL_RTCEx_SetTimeStamp_IT() or HAL_RTCEx_SetTamper_IT() functions.
                  
        (++) To wake up from the Stop mode with an RTC WakeUp event, it is necessary to
              configure the RTC to generate the RTC WakeUp event using the HAL_RTCEx_SetWakeUpTimer_IT() function.

@endverbatim
  * @{
  */
/* *
  * @brief Configures the voltage threshold detected by the Power Voltage Detector(PVD).
  * @param sConfigPVD: pointer to an PWR_PVDTypeDef structure that contains the configuration
  *        information for the PVD.
  * @note Refer to the electrical characteristics of your device datasheet for
  *         more details about the voltage threshold corresponding to each 
  *         detection level.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_ConfigPVD(mut sConfigPVD:
                                               *mut PWR_PVDTypeDef) {
    /* Check the parameters */
    /* Set PLS[7:5] bits according to PVDLevel value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x7000
                                                                            as
                                                                            libc::c_uint)
                                            as *mut PWR_TypeDef)).CR1 as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x7000 as
                                                                       libc::c_uint)
                                       as *mut PWR_TypeDef)).CR1 &
                                    !((0x7 as libc::c_uint) <<
                                          5 as libc::c_uint) |
                                    (*sConfigPVD).PVDLevel);
    /* Clear any previous config. Keep it clear if no event or IT mode is selected */
    let ref mut fresh4 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut EXTI_TypeDef)).EMR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh5 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut EXTI_TypeDef)).IMR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh6 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut EXTI_TypeDef)).RTSR;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh7 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut EXTI_TypeDef)).FTSR;
    ::core::ptr::write_volatile(fresh7,
                                (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Configure interrupt mode */
    if (*sConfigPVD).Mode & 0x10000 as libc::c_uint == 0x10000 as libc::c_uint
       {
        let ref mut fresh8 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x3c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut EXTI_TypeDef)).IMR;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             16 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Configure event mode */
    if (*sConfigPVD).Mode & 0x20000 as libc::c_uint == 0x20000 as libc::c_uint
       {
        let ref mut fresh9 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x3c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut EXTI_TypeDef)).EMR;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             16 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Configure the edge */
    if (*sConfigPVD).Mode & 0x1 as libc::c_uint == 0x1 as libc::c_uint {
        let ref mut fresh10 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x3c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut EXTI_TypeDef)).RTSR;
        ::core::ptr::write_volatile(fresh10,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             16 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    if (*sConfigPVD).Mode & 0x2 as libc::c_uint == 0x2 as libc::c_uint {
        let ref mut fresh11 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x3c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut EXTI_TypeDef)).FTSR;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             16 as libc::c_uint) as uint32_t
                                        as uint32_t)
    };
}
/* *
  * @brief Enables the Power Voltage Detector(PVD).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_EnablePVD() {
    /* Enable the power voltage detector */
    let ref mut fresh12 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         4 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Disables the Power Voltage Detector(PVD).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_DisablePVD() {
    /* Disable the power voltage detector */
    let ref mut fresh13 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh13,
                                (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Enable the WakeUp PINx functionality.
  * @param WakeUpPinPolarity: Specifies which Wake-Up pin to enable.
  *         This parameter can be one of the following legacy values, which sets the default polarity: 
  *         detection on high level (rising edge):
  *           @arg PWR_WAKEUP_PIN1, PWR_WAKEUP_PIN2, PWR_WAKEUP_PIN3, PWR_WAKEUP_PIN4, PWR_WAKEUP_PIN5, PWR_WAKEUP_PIN6 
  *         or one of the following value where the user can explicitly states the enabled pin and
  *         the chosen polarity  
  *           @arg PWR_WAKEUP_PIN1_HIGH or PWR_WAKEUP_PIN1_LOW 
  *           @arg PWR_WAKEUP_PIN2_HIGH or PWR_WAKEUP_PIN2_LOW 
  *           @arg PWR_WAKEUP_PIN3_HIGH or PWR_WAKEUP_PIN3_LOW 
  *           @arg PWR_WAKEUP_PIN4_HIGH or PWR_WAKEUP_PIN4_LOW
  *           @arg PWR_WAKEUP_PIN5_HIGH or PWR_WAKEUP_PIN5_LOW 
  *           @arg PWR_WAKEUP_PIN6_HIGH or PWR_WAKEUP_PIN6_LOW 
  * @note  PWR_WAKEUP_PINx and PWR_WAKEUP_PINx_HIGH are equivalent.               
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_EnableWakeUpPin(mut WakeUpPinPolarity:
                                                     uint32_t) {
    /* Enable wake-up pin */
    let ref mut fresh14 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CSR2;
    ::core::ptr::write_volatile(fresh14,
                                (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x3f00 as libc::c_int as uint32_t &
                                         WakeUpPinPolarity) as uint32_t as
                                    uint32_t);
    /* Specifies the Wake-Up pin polarity for the event detection
    (rising or falling edge) */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x7000
                                                                            as
                                                                            libc::c_uint)
                                            as *mut PWR_TypeDef)).CR2 as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x7000 as
                                                                       libc::c_uint)
                                       as *mut PWR_TypeDef)).CR2 &
                                    !(0x3f00 as libc::c_int as uint32_t &
                                          WakeUpPinPolarity) |
                                    WakeUpPinPolarity >> 0x6 as libc::c_int);
}
/* *
  * @brief Disables the WakeUp PINx functionality.
  * @param WakeUpPinx: Specifies the Power Wake-Up pin to disable.
  *         This parameter can be one of the following values:
  *           @arg PWR_WAKEUP_PIN1
  *           @arg PWR_WAKEUP_PIN2
  *           @arg PWR_WAKEUP_PIN3
  *           @arg PWR_WAKEUP_PIN4
  *           @arg PWR_WAKEUP_PIN5
  *           @arg PWR_WAKEUP_PIN6 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_DisableWakeUpPin(mut WakeUpPinx: uint32_t) {
    let ref mut fresh15 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CSR2;
    ::core::ptr::write_volatile(fresh15,
                                (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !WakeUpPinx) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief Enters Sleep mode.
  *   
  * @note In Sleep mode, all I/O pins keep the same state as in Run mode.
  * 
  * @note In Sleep mode, the systick is stopped to avoid exit from this mode with
  *       systick interrupt when used as time base for Timeout 
  *                
  * @param Regulator: Specifies the regulator state in SLEEP mode.
  *            This parameter can be one of the following values:
  *            @arg PWR_MAINREGULATOR_ON: SLEEP mode with regulator ON
  *            @arg PWR_LOWPOWERREGULATOR_ON: SLEEP mode with low power regulator ON
  * @note This parameter is not used for the STM32F7 family and is kept as parameter
  *       just to maintain compatibility with the lower power families.
  * @param SLEEPEntry: Specifies if SLEEP mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg PWR_SLEEPENTRY_WFI: enter SLEEP mode with WFI instruction
  *            @arg PWR_SLEEPENTRY_WFE: enter SLEEP mode with WFE instruction
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_EnterSLEEPMode(mut Regulator: uint32_t,
                                                mut SLEEPEntry: uint8_t) {
    /* Check the parameters */
    /* Clear SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh16 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh16,
                                (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((1 as libc::c_ulong) <<
                                            2 as libc::c_uint) as uint32_t))
                                    as uint32_t as uint32_t);
    /* Select SLEEP mode entry -------------------------------------------------*/
    if SLEEPEntry as libc::c_int ==
           0x1 as libc::c_uint as uint8_t as libc::c_int {
        /* Request Wait For Interrupt */
        asm!("wfi" : : : : "volatile")
    } else {
        /* Request Wait For Event */
        asm!("sev" : : : : "volatile");
        asm!("wfe" : : : : "volatile");
        asm!("wfe" : : : : "volatile")
    };
}
/* *
  * @brief Enters Stop mode. 
  * @note In Stop mode, all I/O pins keep the same state as in Run mode.
  * @note When exiting Stop mode by issuing an interrupt or a wakeup event, 
  *         the HSI RC oscillator is selected as system clock.
  * @note When the voltage regulator operates in low power mode, an additional 
  *         startup delay is incurred when waking up from Stop mode. 
  *         By keeping the internal regulator ON during Stop mode, the consumption 
  *         is higher although the startup time is reduced.    
  * @param Regulator: Specifies the regulator state in Stop mode.
  *          This parameter can be one of the following values:
  *            @arg PWR_MAINREGULATOR_ON: Stop mode with regulator ON
  *            @arg PWR_LOWPOWERREGULATOR_ON: Stop mode with low power regulator ON
  * @param STOPEntry: Specifies if Stop mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg PWR_STOPENTRY_WFI: Enter Stop mode with WFI instruction
  *            @arg PWR_STOPENTRY_WFE: Enter Stop mode with WFE instruction
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_EnterSTOPMode(mut Regulator: uint32_t,
                                               mut STOPEntry: uint8_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Select the regulator state in Stop mode ---------------------------------*/
    tmpreg =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    /* Clear PDDS and LPDS bits */
    tmpreg &=
        !((0x1 as libc::c_uint) << 1 as libc::c_uint |
              (0x1 as libc::c_uint) << 0 as libc::c_uint);
    /* Set LPDS, MRLVDS and LPLVDS bits according to Regulator value */
    tmpreg |= Regulator;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x7000
                                                                            as
                                                                            libc::c_uint)
                                            as *mut PWR_TypeDef)).CR1 as
                                    *mut uint32_t, tmpreg);
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh17 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh17,
                                (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_ulong |
                                     (1 as libc::c_ulong) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select Stop mode entry --------------------------------------------------*/
    if STOPEntry as libc::c_int ==
           0x1 as libc::c_uint as uint8_t as libc::c_int {
        /* Request Wait For Interrupt */
        asm!("wfi" : : : : "volatile")
    } else {
        /* Request Wait For Event */
        asm!("sev" : : : : "volatile");
        asm!("wfe" : : : : "volatile");
        asm!("wfe" : : : : "volatile")
    }
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh18 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh18,
                                (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((1 as libc::c_ulong) <<
                                            2 as libc::c_uint) as uint32_t))
                                    as uint32_t as uint32_t);
}
/* *
  * @brief Enters Standby mode.
  * @note In Standby mode, all I/O pins are high impedance except for:
  *          - Reset pad (still available) 
  *          - RTC_AF1 pin (PC13) if configured for tamper, time-stamp, RTC 
  *            Alarm out, or RTC clock calibration out.
  *          - RTC_AF2 pin (PI8) if configured for tamper or time-stamp.  
  *          - WKUP pins if enabled.       
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_EnterSTANDBYMode() {
    /* Select Standby mode */
    let ref mut fresh19 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh19,
                                (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh20 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh20,
                                (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_ulong |
                                     (1 as libc::c_ulong) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* This option is used to ensure that store operations are completed */
    /* Request Wait For Interrupt */
    asm!("wfi" : : : : "volatile");
}
/* *
  * @brief This function handles the PWR PVD interrupt request.
  * @note This API should be called under the PVD_IRQHandler().
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_PVD_IRQHandler() {
    /* Check PWR Exti flag */
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut EXTI_TypeDef)).PR &
           (0x1 as libc::c_uint) << 16 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* PWR PVD interrupt user callback */
        HAL_PWR_PVDCallback();
        /* Clear PWR Exti pending bit */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x10000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut EXTI_TypeDef)).PR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        16 as libc::c_uint)
    };
}
/* *
  * @brief  PWR PVD interrupt callback
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_PVDCallback() {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PWR_PVDCallback could be implemented in the user file
   */
}
/* *
  * @brief Indicates Sleep-On-Exit when returning from Handler mode to Thread mode. 
  * @note Set SLEEPONEXIT bit of SCR register. When this bit is set, the processor 
  *       re-enters SLEEP mode when an interruption handling is over.
  *       Setting this bit is useful when the processor is expected to run only on
  *       interruptions handling.         
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_EnableSleepOnExit() {
    /* Set SLEEPONEXIT bit of Cortex System Control Register */
    let ref mut fresh21 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh21,
                                (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((1 as libc::c_ulong) <<
                                          1 as libc::c_uint) as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief Disables Sleep-On-Exit feature when returning from Handler mode to Thread mode. 
  * @note Clears SLEEPONEXIT bit of SCR register. When this bit is set, the processor 
  *       re-enters SLEEP mode when an interruption handling is over.          
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_DisableSleepOnExit() {
    /* Clear SLEEPONEXIT bit of Cortex System Control Register */
    let ref mut fresh22 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh22,
                                (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((1 as libc::c_ulong) <<
                                            1 as libc::c_uint) as uint32_t))
                                    as uint32_t as uint32_t);
}
/* *
  * @brief Enables CORTEX M4 SEVONPEND bit. 
  * @note Sets SEVONPEND bit of SCR register. When this bit is set, this causes 
  *       WFE to wake up when an interrupt moves from inactive to pended.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_EnableSEVOnPend() {
    /* Set SEVONPEND bit of Cortex System Control Register */
    let ref mut fresh23 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh23,
                                (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((1 as libc::c_ulong) <<
                                          4 as libc::c_uint) as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup PWR_Exported_Constants PWR Exported Constants
  * @{
  */
/* * @defgroup PWR_PVD_detection_level PWR PVD detection level
  * @{
  */
/* External input analog voltage 
                                                          (Compare internally to VREFINT) */
/* *
  * @}
  */
/* * @defgroup PWR_PVD_Mode PWR PVD Mode
  * @{
  */
/* !< basic mode is used */
/* !< External Interrupt Mode with Rising edge trigger detection */
/* !< External Interrupt Mode with Falling edge trigger detection */
/* !< External Interrupt Mode with Rising/Falling edge trigger detection */
/* !< Event Mode with Rising edge trigger detection */
/* !< Event Mode with Falling edge trigger detection */
/* !< Event Mode with Rising/Falling edge trigger detection */
/* *
  * @}
  */
/* * @defgroup PWR_Regulator_state_in_STOP_mode PWR Regulator state in SLEEP/STOP mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_SLEEP_mode_entry PWR SLEEP mode entry
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_STOP_mode_entry PWR STOP mode entry
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_Regulator_Voltage_Scale PWR Regulator Voltage Scale
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_Flag PWR Flag
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup PWR_Exported_Macro PWR Exported Macro
  * @{
  */
/* * @brief  macros configure the main internal regulator output voltage.
  * @param  __REGULATOR__: specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption when the device does
  *         not operate at the maximum frequency (refer to the datasheets for more details).
  *          This parameter can be one of the following values:
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output Scale 1 mode
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output Scale 2 mode
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE3: Regulator voltage output Scale 3 mode
  * @retval None
  */
/* Delay after an RCC peripheral clock enabling */
/* * @brief  Check PWR flag is set or not.
  * @param  __FLAG__: specifies the flag to check.
  *           This parameter can be one of the following values:
  *            @arg PWR_FLAG_WU: Wake Up flag. This flag indicates that a wakeup event 
  *                  was received on the internal wakeup line in standby mode (RTC alarm (Alarm A or Alarm B),
  *                  RTC Tamper event, RTC TimeStamp event or RTC Wakeup)).
  *            @arg PWR_FLAG_SB: StandBy flag. This flag indicates that the system was
  *                  resumed from StandBy mode.    
  *            @arg PWR_FLAG_PVDO: PVD Output. This flag is valid only if PVD is enabled 
  *                  by the HAL_PWR_EnablePVD() function. The PVD is stopped by Standby mode 
  *                  For this reason, this bit is equal to 0 after Standby or reset
  *                  until the PVDE bit is set.
  *            @arg PWR_FLAG_BRR: Backup regulator ready flag. This bit is not reset 
  *                  when the device wakes up from Standby mode or by a system reset 
  *                  or power reset.  
  *            @arg PWR_FLAG_VOSRDY: This flag indicates that the Regulator voltage 
  *                 scaling output selection is ready.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
/* * @brief  Clear the PWR's pending flags.
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be one of the following values:
  *            @arg PWR_FLAG_SB: StandBy flag
  */
/* *
  * @brief Enable the PVD Exti Line 16.
  * @retval None.
  */
/* *
  * @brief Disable the PVD EXTI Line 16.
  * @retval None.
  */
/* *
  * @brief Enable event on PVD Exti Line 16.
  * @retval None.
  */
/* *
  * @brief Disable event on PVD Exti Line 16.
  * @retval None.
  */
/* *
  * @brief Enable the PVD Extended Interrupt Rising Trigger.
  * @retval None.
  */
/* *
  * @brief Disable the PVD Extended Interrupt Rising Trigger.
  * @retval None.
  */
/* *
  * @brief Enable the PVD Extended Interrupt Falling Trigger.
  * @retval None.
  */
/* *
  * @brief Disable the PVD Extended Interrupt Falling Trigger.
  * @retval None.
  */
/* *
  * @brief  PVD EXTI line configuration: set rising & falling edge trigger.
  * @retval None.
  */
/* *
  * @brief Disable the PVD Extended Interrupt Rising & Falling Trigger.
  * @retval None.
  */
/* *
  * @brief checks whether the specified PVD Exti interrupt flag is set or not.
  * @retval EXTI PVD Line Status.
  */
/* *
  * @brief Clear the PVD Exti flag.
  * @retval None.
  */
/* *
  * @brief  Generates a Software interrupt on PVD EXTI line.
  * @retval None
  */
/* *
  * @}
  */
/* Include PWR HAL Extension module */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup PWR_Exported_Functions PWR Exported Functions
  * @{
  */
/* * @addtogroup PWR_Exported_Functions_Group1 Initialization and de-initialization functions 
  * @{
  */
/* Initialization and de-initialization functions *****************************/
/* *
  * @}
  */
/* * @addtogroup PWR_Exported_Functions_Group2 Peripheral Control functions 
  * @{
  */
/* Peripheral Control functions  **********************************************/
/* PVD configuration */
/* WakeUp pins configuration */
/* Low Power modes entry */
/* Power PVD IRQ Handler */
/* Cortex System Control functions  *******************************************/
/* *
  * @brief Disables CORTEX M4 SEVONPEND bit. 
  * @note Clears SEVONPEND bit of SCR register. When this bit is set, this causes 
  *       WFE to wake up when an interrupt moves from inactive to pended.         
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWR_DisableSEVOnPend() {
    /* Clear SEVONPEND bit of Cortex System Control Register */
    let ref mut fresh24 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh24,
                                (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((1 as libc::c_ulong) <<
                                            4 as libc::c_uint) as uint32_t))
                                    as uint32_t as uint32_t);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_PWR_MODULE_ENABLED */
/* *
  * @}
  */
/* *
  * @}
  */
