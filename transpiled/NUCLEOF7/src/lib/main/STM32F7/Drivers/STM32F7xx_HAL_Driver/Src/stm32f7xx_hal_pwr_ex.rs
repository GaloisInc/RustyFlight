use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_GetTick() -> uint32_t;
}
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PWR_TypeDef {
    pub CR1: uint32_t,
    pub CSR1: uint32_t,
    pub CR2: uint32_t,
    pub CSR2: uint32_t,
}
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
  * @file    stm32f7xx_hal_def.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file contains HAL common defines, enumeration, macros and 
  *          structures definitions. 
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
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  HAL Status structures definition  
  */
pub type HAL_StatusTypeDef = libc::c_uint;
pub const HAL_TIMEOUT: HAL_StatusTypeDef = 3;
pub const HAL_BUSY: HAL_StatusTypeDef = 2;
pub const HAL_ERROR: HAL_StatusTypeDef = 1;
pub const HAL_OK: HAL_StatusTypeDef = 0;
/* *
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup PWREx_Exported_Functions PWREx Exported Functions
  *  @{
  */
/* * @defgroup PWREx_Exported_Functions_Group1 Peripheral Extended features functions 
  *  @brief Peripheral Extended features functions 
  *
@verbatim   

 ===============================================================================
                 ##### Peripheral extended features functions #####
 ===============================================================================

    *** Main and Backup Regulators configuration ***
    ================================================
    [..] 
      (+) The backup domain includes 4 Kbytes of backup SRAM accessible only from 
          the CPU, and address in 32-bit, 16-bit or 8-bit mode. Its content is 
          retained even in Standby or VBAT mode when the low power backup regulator
          is enabled. It can be considered as an internal EEPROM when VBAT is 
          always present. You can use the HAL_PWREx_EnableBkUpReg() function to 
          enable the low power backup regulator. 

      (+) When the backup domain is supplied by VDD (analog switch connected to VDD) 
          the backup SRAM is powered from VDD which replaces the VBAT power supply to 
          save battery life.

      (+) The backup SRAM is not mass erased by a tamper event. It is read 
          protected to prevent confidential data, such as cryptographic private 
          key, from being accessed. The backup SRAM can be erased only through 
          the Flash interface when a protection level change from level 1 to 
          level 0 is requested. 
      -@- Refer to the description of Read protection (RDP) in the Flash 
          programming manual.

      (+) The main internal regulator can be configured to have a tradeoff between 
          performance and power consumption when the device does not operate at 
          the maximum frequency. This is done through __HAL_PWR_MAINREGULATORMODE_CONFIG() 
          macro which configure VOS bit in PWR_CR register
          
        Refer to the product datasheets for more details.

    *** FLASH Power Down configuration ****
    =======================================
    [..] 
      (+) By setting the FPDS bit in the PWR_CR register by using the 
          HAL_PWREx_EnableFlashPowerDown() function, the Flash memory also enters power 
          down mode when the device enters Stop mode. When the Flash memory 
          is in power down mode, an additional startup delay is incurred when 
          waking up from Stop mode.

    *** Over-Drive and Under-Drive configuration ****
    =================================================
    [..]         
       (+) In Run mode: the main regulator has 2 operating modes available:
        (++) Normal mode: The CPU and core logic operate at maximum frequency at a given 
             voltage scaling (scale 1, scale 2 or scale 3)
        (++) Over-drive mode: This mode allows the CPU and the core logic to operate at a 
            higher frequency than the normal mode for a given voltage scaling (scale 1,  
            scale 2 or scale 3). This mode is enabled through HAL_PWREx_EnableOverDrive() function and
            disabled by HAL_PWREx_DisableOverDrive() function, to enter or exit from Over-drive mode please follow 
            the sequence described in Reference manual.
             
       (+) In Stop mode: the main regulator or low power regulator supplies a low power 
           voltage to the 1.2V domain, thus preserving the content of registers 
           and internal SRAM. 2 operating modes are available:
         (++) Normal mode: the 1.2V domain is preserved in nominal leakage mode. This mode is only 
              available when the main regulator or the low power regulator is used in Scale 3 or 
              low voltage mode.
         (++) Under-drive mode: the 1.2V domain is preserved in reduced leakage mode. This mode is only
              available when the main regulator or the low power regulator is in low voltage mode.

@endverbatim
  * @{
  */
/* *
  * @brief Enables the Backup Regulator.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_EnableBkUpReg() -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Enable Backup regulator */
    let ref mut fresh0 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CSR1;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         9 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Workaround for the following hardware bug: */
  /* Id 19: PWR : No STANDBY wake-up when Back-up RAM enabled (ref. Errata Sheet p23) */
    let ref mut fresh1 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CSR1;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till Backup regulator ready flag is set */
    while ((*((0x40000000 as
                   libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                  *mut PWR_TypeDef)).CSR1 &
               (0x1 as libc::c_uint) << 3 as libc::c_uint ==
               (0x1 as libc::c_uint) << 3 as libc::c_uint) as libc::c_int ==
              RESET as libc::c_int {
        if HAL_GetTick().wrapping_sub(tickstart) >
               1000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
    }
    return HAL_OK;
}
/* *
  * @brief Disables the Backup Regulator.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_DisableBkUpReg() -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable Backup regulator */
    let ref mut fresh2 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CSR1;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           9 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Workaround for the following hardware bug: */
  /* Id 19: PWR : No STANDBY wake-up when Back-up RAM enabled (ref. Errata Sheet p23) */
    let ref mut fresh3 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CSR1;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till Backup regulator ready flag is set */
    while ((*((0x40000000 as
                   libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                  *mut PWR_TypeDef)).CSR1 &
               (0x1 as libc::c_uint) << 3 as libc::c_uint ==
               (0x1 as libc::c_uint) << 3 as libc::c_uint) as libc::c_int !=
              RESET as libc::c_int {
        if HAL_GetTick().wrapping_sub(tickstart) >
               1000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
    }
    return HAL_OK;
}
/* *
  * @brief Enables the Flash Power Down in Stop mode.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_EnableFlashPowerDown() {
    /* Enable the Flash Power Down */
    let ref mut fresh4 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         9 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Disables the Flash Power Down in Stop mode.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_DisableFlashPowerDown() {
    /* Disable the Flash Power Down */
    let ref mut fresh5 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           9 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Enables Main Regulator low voltage mode.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_EnableMainRegulatorLowVoltage() {
    /* Enable Main regulator low voltage */
    let ref mut fresh6 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         11 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Disables Main Regulator low voltage mode.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_DisableMainRegulatorLowVoltage() {
    /* Disable Main regulator low voltage */
    let ref mut fresh7 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh7,
                                (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           11 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Enables Low Power Regulator low voltage mode.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_EnableLowRegulatorLowVoltage() {
    /* Enable low power regulator */
    let ref mut fresh8 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh8,
                                (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         10 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Disables Low Power Regulator low voltage mode.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_DisableLowRegulatorLowVoltage() {
    /* Disable low power regulator */
    let ref mut fresh9 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           10 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Activates the Over-Drive mode.
  * @note   This mode allows the CPU and the core logic to operate at a higher frequency
  *         than the normal mode for a given voltage scaling (scale 1, scale 2 or scale 3).   
  * @note   It is recommended to enter or exit Over-drive mode when the application is not running 
  *         critical tasks and when the system clock source is either HSI or HSE. 
  *         During the Over-drive switch activation, no peripheral clocks should be enabled.   
  *         The peripheral clocks must be enabled once the Over-drive mode is activated.   
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_EnableOverDrive() -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh10 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         28 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        28 as libc::c_uint);
    /* Enable the Over-drive to extend the clock frequency to 216 MHz */
    let ref mut fresh11 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         16 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Get tick */
    tickstart = HAL_GetTick();
    while !((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                   *mut PWR_TypeDef)).CSR1 &
                (0x1 as libc::c_uint) << 16 as libc::c_uint ==
                (0x1 as libc::c_uint) << 16 as libc::c_uint) {
        if HAL_GetTick().wrapping_sub(tickstart) >
               1000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
    }
    /* Enable the Over-drive switch */
    let ref mut fresh12 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         17 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Get tick */
    tickstart = HAL_GetTick();
    while !((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                   *mut PWR_TypeDef)).CSR1 &
                (0x1 as libc::c_uint) << 17 as libc::c_uint ==
                (0x1 as libc::c_uint) << 17 as libc::c_uint) {
        if HAL_GetTick().wrapping_sub(tickstart) >
               1000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
    }
    return HAL_OK;
}
/* *
  * @brief  Deactivates the Over-Drive mode.
  * @note   This mode allows the CPU and the core logic to operate at a higher frequency
  *         than the normal mode for a given voltage scaling (scale 1, scale 2 or scale 3).    
  * @note   It is recommended to enter or exit Over-drive mode when the application is not running 
  *         critical tasks and when the system clock source is either HSI or HSE. 
  *         During the Over-drive switch activation, no peripheral clocks should be enabled.   
  *         The peripheral clocks must be enabled once the Over-drive mode is activated.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_DisableOverDrive() -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh13 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh13,
                                (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         28 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        28 as libc::c_uint);
    /* Disable the Over-drive switch */
    let ref mut fresh14 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh14,
                                (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           17 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get tick */
    tickstart = HAL_GetTick();
    while (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
                 as *mut PWR_TypeDef)).CSR1 &
              (0x1 as libc::c_uint) << 17 as libc::c_uint ==
              (0x1 as libc::c_uint) << 17 as libc::c_uint {
        if HAL_GetTick().wrapping_sub(tickstart) >
               1000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
    }
    /* Disable the Over-drive */
    let ref mut fresh15 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh15,
                                (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get tick */
    tickstart = HAL_GetTick();
    while (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
                 as *mut PWR_TypeDef)).CSR1 &
              (0x1 as libc::c_uint) << 16 as libc::c_uint ==
              (0x1 as libc::c_uint) << 16 as libc::c_uint {
        if HAL_GetTick().wrapping_sub(tickstart) >
               1000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
    }
    return HAL_OK;
}
/* *
  * @brief  Enters in Under-Drive STOP mode.
  * 
  * @note    This mode can be selected only when the Under-Drive is already active 
  *   
  * @note    This mode is enabled only with STOP low power mode.
  *          In this mode, the 1.2V domain is preserved in reduced leakage mode. This 
  *          mode is only available when the main regulator or the low power regulator 
  *          is in low voltage mode
  *        
  * @note   If the Under-drive mode was enabled, it is automatically disabled after 
  *         exiting Stop mode. 
  *         When the voltage regulator operates in Under-drive mode, an additional  
  *         startup delay is induced when waking up from Stop mode.
  *                    
  * @note   In Stop mode, all I/O pins keep the same state as in Run mode.
  *   
  * @note   When exiting Stop mode by issuing an interrupt or a wakeup event, 
  *         the HSI RC oscillator is selected as system clock.
  *           
  * @note   When the voltage regulator operates in low power mode, an additional 
  *         startup delay is incurred when waking up from Stop mode. 
  *         By keeping the internal regulator ON during Stop mode, the consumption 
  *         is higher although the startup time is reduced.
  *     
  * @param  Regulator: specifies the regulator state in STOP mode.
  *          This parameter can be one of the following values:
  *            @arg PWR_MAINREGULATOR_UNDERDRIVE_ON:  Main Regulator in under-drive mode 
  *                 and Flash memory in power-down when the device is in Stop under-drive mode
  *            @arg PWR_LOWPOWERREGULATOR_UNDERDRIVE_ON:  Low Power Regulator in under-drive mode 
  *                and Flash memory in power-down when the device is in Stop under-drive mode
  * @param  STOPEntry: specifies if STOP mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg PWR_SLEEPENTRY_WFI: enter STOP mode with WFI instruction
  *            @arg PWR_SLEEPENTRY_WFE: enter STOP mode with WFE instruction
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_EnterUnderDriveSTOPMode(mut Regulator:
                                                               uint32_t,
                                                           mut STOPEntry:
                                                               uint8_t)
 -> HAL_StatusTypeDef {
    let mut tempreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Enable Power ctrl clock */
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh16 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh16,
                                (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         28 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        28 as libc::c_uint);
    /* Enable the Under-drive Mode ---------------------------------------------*/
  /* Clear Under-drive flag */
    let ref mut fresh17 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CSR1;
    ::core::ptr::write_volatile(fresh17,
                                (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x3 as libc::c_uint) <<
                                         18 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable the Under-drive */
    let ref mut fresh18 =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    ::core::ptr::write_volatile(fresh18,
                                (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x3 as libc::c_uint) <<
                                         18 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait for UnderDrive mode is ready */
    while (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
                 as *mut PWR_TypeDef)).CSR1 &
              (0x3 as libc::c_uint) << 18 as libc::c_uint ==
              (0x3 as libc::c_uint) << 18 as libc::c_uint {
        if HAL_GetTick().wrapping_sub(tickstart) >
               1000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
    }
    /* Select the regulator state in STOP mode ---------------------------------*/
    tempreg =
        (*((0x40000000 as libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
               as *mut PWR_TypeDef)).CR1;
    /* Clear PDDS, LPDS, MRLUDS and LPLUDS bits */
    tempreg &=
        !((0x1 as libc::c_uint) << 1 as libc::c_uint |
              (0x1 as libc::c_uint) << 0 as libc::c_uint |
              (0x1 as libc::c_uint) << 10 as libc::c_uint |
              (0x1 as libc::c_uint) << 11 as libc::c_uint);
    /* Set LPDS, MRLUDS and LPLUDS bits according to PWR_Regulator value */
    tempreg |= Regulator;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x7000
                                                                            as
                                                                            libc::c_uint)
                                            as *mut PWR_TypeDef)).CR1 as
                                    *mut uint32_t, tempreg);
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh19 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh19,
                                (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_ulong |
                                     (1 as libc::c_ulong) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select STOP mode entry --------------------------------------------------*/
    if STOPEntry as libc::c_int ==
           0x1 as libc::c_uint as uint8_t as libc::c_int {
        /* Request Wait For Interrupt */
        asm!("wfi" : : : : "volatile")
    } else {
        /* Request Wait For Event */
        asm!("wfe" : : : : "volatile")
    }
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh20 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh20,
                                (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((1 as libc::c_ulong) <<
                                            2 as libc::c_uint) as uint32_t))
                                    as uint32_t as uint32_t);
    return HAL_OK;
}
/* *
  * @brief Returns Voltage Scaling Range.
  * @retval VOS bit field (PWR_REGULATOR_VOLTAGE_SCALE1, PWR_REGULATOR_VOLTAGE_SCALE2 or 
  *            PWR_REGULATOR_VOLTAGE_SCALE3)PWR_REGULATOR_VOLTAGE_SCALE1
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_GetVoltageRange() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                  *mut PWR_TypeDef)).CR1 &
               (0x3 as libc::c_uint) << 14 as libc::c_uint;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_pwr_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of PWR HAL Extension module.
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
/* * @addtogroup PWREx
  * @{
  */
/* Exported types ------------------------------------------------------------*/ 
/* Exported constants --------------------------------------------------------*/
/* * @defgroup PWREx_Exported_Constants PWREx Exported Constants
  * @{
  */
/* * @defgroup PWREx_WakeUp_Pins PWREx Wake Up Pins
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWREx_Regulator_state_in_UnderDrive_mode PWREx Regulator state in UnderDrive mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWREx_Over_Under_Drive_Flag PWREx Over Under Drive Flag
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWREx_Wakeup_Pins_Flag PWREx Wake Up Pin Flags
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup PWREx_Exported_Macro PWREx Exported Macro
  *  @{
  */
/* * @brief Macros to enable or disable the Over drive mode.
  */
/* * @brief Macros to enable or disable the Over drive switching.
  */
/* * @brief Macros to enable or disable the Under drive mode.
  * @note  This mode is enabled only with STOP low power mode.
  *        In this mode, the 1.2V domain is preserved in reduced leakage mode. This 
  *        mode is only available when the main regulator or the low power regulator 
  *        is in low voltage mode.      
  * @note  If the Under-drive mode was enabled, it is automatically disabled after 
  *        exiting Stop mode. 
  *        When the voltage regulator operates in Under-drive mode, an additional  
  *        startup delay is induced when waking up from Stop mode.
  */
/* * @brief  Check PWR flag is set or not.
  * @param  __FLAG__: specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg PWR_FLAG_ODRDY: This flag indicates that the Over-drive mode
  *                                 is ready 
  *            @arg PWR_FLAG_ODSWRDY: This flag indicates that the Over-drive mode
  *                                   switching is ready  
  *            @arg PWR_FLAG_UDRDY: This flag indicates that the Under-drive mode
  *                                 is enabled in Stop mode
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
/* * @brief Clear the Under-Drive Ready flag.
  */
/* * @brief  Check Wake Up flag is set or not.
  * @param  __WUFLAG__: specifies the Wake Up flag to check.
  *          This parameter can be one of the following values:
  *            @arg PWR_WAKEUP_PIN_FLAG1: Wakeup Pin Flag for PA0
  *            @arg PWR_WAKEUP_PIN_FLAG2: Wakeup Pin Flag for PA2
  *            @arg PWR_WAKEUP_PIN_FLAG3: Wakeup Pin Flag for PC1
  *            @arg PWR_WAKEUP_PIN_FLAG4: Wakeup Pin Flag for PC13
  *            @arg PWR_WAKEUP_PIN_FLAG5: Wakeup Pin Flag for PI8
  *            @arg PWR_WAKEUP_PIN_FLAG6: Wakeup Pin Flag for PI11          
  */
/* * @brief  Clear the WakeUp pins flags.
  * @param  __WUFLAG__: specifies the Wake Up pin flag to clear.
  *          This parameter can be one of the following values:
  *            @arg PWR_WAKEUP_PIN_FLAG1: Wakeup Pin Flag for PA0
  *            @arg PWR_WAKEUP_PIN_FLAG2: Wakeup Pin Flag for PA2
  *            @arg PWR_WAKEUP_PIN_FLAG3: Wakeup Pin Flag for PC1
  *            @arg PWR_WAKEUP_PIN_FLAG4: Wakeup Pin Flag for PC13
  *            @arg PWR_WAKEUP_PIN_FLAG5: Wakeup Pin Flag for PI8
  *            @arg PWR_WAKEUP_PIN_FLAG6: Wakeup Pin Flag for PI11          
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup PWREx_Exported_Functions PWREx Exported Functions
  *  @{
  */
/* * @addtogroup PWREx_Exported_Functions_Group1
  * @{
  */
/* *
  * @brief Configures the main internal regulator output voltage.
  * @param  VoltageScaling: specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption.
  *          This parameter can be one of the following values:
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output range 1 mode,
  *                                                typical output voltage at 1.4 V,  
  *                                                system frequency up to 216 MHz.
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output range 2 mode,
  *                                                typical output voltage at 1.2 V,                
  *                                                system frequency up to 180 MHz.
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE3: Regulator voltage output range 2 mode,
  *                                                typical output voltage at 1.00 V,                
  *                                                system frequency up to 151 MHz.
  * @note To update the system clock frequency(SYSCLK):
  *        - Set the HSI or HSE as system clock frequency using the HAL_RCC_ClockConfig().
  *        - Call the HAL_RCC_OscConfig() to configure the PLL.
  *        - Call HAL_PWREx_ConfigVoltageScaling() API to adjust the voltage scale.
  *        - Set the new system clock frequency using the HAL_RCC_ClockConfig().
  * @note The scale can be modified only when the HSI or HSE clock source is selected 
  *        as system clock source, otherwise the API returns HAL_ERROR.  
  * @note When the PLL is OFF, the voltage scale 3 is automatically selected and the VOS bits
  *       value in the PWR_CR1 register are not taken in account.
  * @note This API forces the PLL state ON to allow the possibility to configure the voltage scale 1 or 2.
  * @note The new voltage scale is active only when the PLL is ON.  
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PWREx_ControlVoltageScaling(mut VoltageScaling:
                                                             uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Enable Power ctrl clock */
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh21 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh21,
                                (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         28 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        28 as libc::c_uint);
    /* Check if the PLL is used as system clock or not */
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3800
                                                                             as
                                                                             libc::c_uint)
              as *mut RCC_TypeDef)).CFGR &
           (0x3 as libc::c_uint) << 2 as libc::c_uint != 0x8 as libc::c_uint {
        /* Disable the main PLL */
        let ref mut fresh22 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh22,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               24 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Get Start Tick */
        tickstart = HAL_GetTick();
        /* Wait till PLL is disabled */
        while (if (if 0x39 as libc::c_uint as uint8_t as libc::c_int >>
                          5 as libc::c_int == 1 as libc::c_int {
                       (*((0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x3800
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut RCC_TypeDef)).CR
                   } else {
                       (if 0x39 as libc::c_uint as uint8_t as libc::c_int >>
                               5 as libc::c_int == 2 as libc::c_int {
                            (*((0x40000000 as
                                    libc::c_uint).wrapping_add(0x20000 as
                                                                   libc::c_uint).wrapping_add(0x3800
                                                                                                  as
                                                                                                  libc::c_uint)
                                   as *mut RCC_TypeDef)).BDCR
                        } else {
                            (if 0x39 as libc::c_uint as uint8_t as libc::c_int
                                    >> 5 as libc::c_int == 3 as libc::c_int {
                                 (*((0x40000000 as
                                         libc::c_uint).wrapping_add(0x20000 as
                                                                        libc::c_uint).wrapping_add(0x3800
                                                                                                       as
                                                                                                       libc::c_uint)
                                        as *mut RCC_TypeDef)).CSR
                             } else {
                                 (*((0x40000000 as
                                         libc::c_uint).wrapping_add(0x20000 as
                                                                        libc::c_uint).wrapping_add(0x3800
                                                                                                       as
                                                                                                       libc::c_uint)
                                        as *mut RCC_TypeDef)).CIR
                             })
                        })
                   }) &
                      (1 as libc::c_int as uint32_t) <<
                          (0x39 as libc::c_uint as uint8_t as libc::c_int &
                               0x1f as libc::c_int as uint8_t as libc::c_int)
                      != 0 as libc::c_int as libc::c_uint {
                   1 as libc::c_int
               } else { 0 as libc::c_int }) != RESET as libc::c_int {
            if HAL_GetTick().wrapping_sub(tickstart) >
                   2 as libc::c_int as uint32_t {
                return HAL_TIMEOUT
            }
        }
        /* Set Range */
        let mut tmpreg_0: uint32_t = 0;
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x7000
                                                                                as
                                                                                libc::c_uint)
                                                as *mut PWR_TypeDef)).CR1 as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x7000
                                                                           as
                                                                           libc::c_uint)
                                           as *mut PWR_TypeDef)).CR1 &
                                        !((0x3 as libc::c_uint) <<
                                              14 as libc::c_uint) |
                                        VoltageScaling);
        ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x7000
                                                                           as
                                                                           libc::c_uint)
                                           as *mut PWR_TypeDef)).CR1 &
                                        (0x3 as libc::c_uint) <<
                                            14 as libc::c_uint);
        /* Enable the main PLL */
        let ref mut fresh23 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh23,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             24 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Get Start Tick */
        tickstart = HAL_GetTick();
        /* Wait till PLL is ready */
        while (if (if 0x39 as libc::c_uint as uint8_t as libc::c_int >>
                          5 as libc::c_int == 1 as libc::c_int {
                       (*((0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x3800
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut RCC_TypeDef)).CR
                   } else {
                       (if 0x39 as libc::c_uint as uint8_t as libc::c_int >>
                               5 as libc::c_int == 2 as libc::c_int {
                            (*((0x40000000 as
                                    libc::c_uint).wrapping_add(0x20000 as
                                                                   libc::c_uint).wrapping_add(0x3800
                                                                                                  as
                                                                                                  libc::c_uint)
                                   as *mut RCC_TypeDef)).BDCR
                        } else {
                            (if 0x39 as libc::c_uint as uint8_t as libc::c_int
                                    >> 5 as libc::c_int == 3 as libc::c_int {
                                 (*((0x40000000 as
                                         libc::c_uint).wrapping_add(0x20000 as
                                                                        libc::c_uint).wrapping_add(0x3800
                                                                                                       as
                                                                                                       libc::c_uint)
                                        as *mut RCC_TypeDef)).CSR
                             } else {
                                 (*((0x40000000 as
                                         libc::c_uint).wrapping_add(0x20000 as
                                                                        libc::c_uint).wrapping_add(0x3800
                                                                                                       as
                                                                                                       libc::c_uint)
                                        as *mut RCC_TypeDef)).CIR
                             })
                        })
                   }) &
                      (1 as libc::c_int as uint32_t) <<
                          (0x39 as libc::c_uint as uint8_t as libc::c_int &
                               0x1f as libc::c_int as uint8_t as libc::c_int)
                      != 0 as libc::c_int as libc::c_uint {
                   1 as libc::c_int
               } else { 0 as libc::c_int }) == RESET as libc::c_int {
            if HAL_GetTick().wrapping_sub(tickstart) >
                   2 as libc::c_int as uint32_t {
                return HAL_TIMEOUT
            }
        }
        /* Get Start Tick */
        tickstart = HAL_GetTick();
        while ((*((0x40000000 as
                       libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                      *mut PWR_TypeDef)).CSR1 &
                   (0x1 as libc::c_uint) << 14 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 14 as libc::c_uint) as libc::c_int
                  == RESET as libc::c_int {
            if HAL_GetTick().wrapping_sub(tickstart) >
                   1000 as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
        }
    } else { return HAL_ERROR }
    return HAL_OK;
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
