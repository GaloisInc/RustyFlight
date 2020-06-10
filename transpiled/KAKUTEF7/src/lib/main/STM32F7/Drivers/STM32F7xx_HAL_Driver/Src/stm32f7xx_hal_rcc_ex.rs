use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    stm32f7xx_hal.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file contains all the functions prototypes for the HAL 
  *          module driver.
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
    /* * @addtogroup HAL
  * @{
  */
    /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup HAL_Exported_Constants HAL Exported Constants
  * @{
  */
    /* * @defgroup SYSCFG_BootMode Boot Mode
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup HAL_Exported_Macros HAL Exported Macros
  * @{
  */
    /* * @brief  Freeze/Unfreeze Peripherals in Debug mode 
  */
    /* * @brief  FMC (NOR/RAM) mapped at 0x60000000 and SDRAM mapped at 0xC0000000
  */
    /* * @brief  FMC/SDRAM  mapped at 0x60000000 (NOR/RAM) mapped at 0xC0000000
  */
    /* *
  * @brief  Return the memory boot mapping as configured by user.
  * @retval The boot mode as configured by user. The returned value can be one
  *         of the following values:
  *           @arg @ref SYSCFG_MEM_BOOT_ADD0
  *           @arg @ref SYSCFG_MEM_BOOT_ADD1
  */
    /* STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
    /* *
  * @}
  */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup HAL_Exported_Functions
  * @{
  */
/* * @addtogroup HAL_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions  ******************************/
    /* *
  * @}
  */
    /* * @addtogroup HAL_Exported_Functions_Group2
  * @{
  */ 
/* Peripheral Control functions  ************************************************/
    #[no_mangle]
    fn HAL_GetTick() -> uint32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
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
  * @brief  PLLI2S Clock structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_PLLI2SInitTypeDef {
    pub PLLI2SN: uint32_t,
    pub PLLI2SR: uint32_t,
    pub PLLI2SQ: uint32_t,
    pub PLLI2SP: uint32_t,
}
/* * 
  * @brief  PLLSAI Clock structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_PLLSAIInitTypeDef {
    pub PLLSAIN: uint32_t,
    pub PLLSAIQ: uint32_t,
    pub PLLSAIR: uint32_t,
    pub PLLSAIP: uint32_t,
}
/* * 
  * @brief  RCC extended clocks structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_PeriphCLKInitTypeDef {
    pub PeriphClockSelection: uint32_t,
    pub PLLI2S: RCC_PLLI2SInitTypeDef,
    pub PLLSAI: RCC_PLLSAIInitTypeDef,
    pub PLLI2SDivQ: uint32_t,
    pub PLLSAIDivQ: uint32_t,
    pub PLLSAIDivR: uint32_t,
    pub RTCClockSelection: uint32_t,
    pub I2sClockSelection: uint32_t,
    pub TIMPresSelection: uint32_t,
    pub Sai1ClockSelection: uint32_t,
    pub Sai2ClockSelection: uint32_t,
    pub Usart1ClockSelection: uint32_t,
    pub Usart2ClockSelection: uint32_t,
    pub Usart3ClockSelection: uint32_t,
    pub Uart4ClockSelection: uint32_t,
    pub Uart5ClockSelection: uint32_t,
    pub Usart6ClockSelection: uint32_t,
    pub Uart7ClockSelection: uint32_t,
    pub Uart8ClockSelection: uint32_t,
    pub I2c1ClockSelection: uint32_t,
    pub I2c2ClockSelection: uint32_t,
    pub I2c3ClockSelection: uint32_t,
    pub I2c4ClockSelection: uint32_t,
    pub Lptim1ClockSelection: uint32_t,
    pub CecClockSelection: uint32_t,
    pub Clk48ClockSelection: uint32_t,
    pub Sdmmc1ClockSelection: uint32_t,
}
#[inline(always)]
unsafe extern "C" fn __RBIT(mut value: uint32_t) -> uint32_t {
    let mut result: uint32_t = 0;
    let mut s: uint32_t =
        (4 as
             libc::c_uint).wrapping_mul(8 as
                                            libc::c_uint).wrapping_sub(1 as
                                                                           libc::c_uint);
    result = value;
    value >>= 1 as libc::c_uint;
    while value != 0 as libc::c_uint {
        result <<= 1 as libc::c_uint;
        result |= value & 1 as libc::c_uint;
        s = s.wrapping_sub(1);
        value >>= 1 as libc::c_uint
    }
    result <<= s;
    return result;
}
/* Timeout value fixed to 100 ms  */
/* *
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* * @defgroup RCCEx_Private_Macros RCCEx Private Macros
 * @{
 */
/* *
  * @}
  */
/* * @defgroup RCCEx_Private_Macros RCCEx Private Macros
 * @{
 */
/* *
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup RCCEx_Exported_Functions RCCEx Exported Functions
  * @{
  */
/* * @defgroup RCCEx_Exported_Functions_Group1 Extended Peripheral Control functions 
 *  @brief  Extended Peripheral Control functions  
 *
@verbatim   
 ===============================================================================
                ##### Extended Peripheral Control functions  #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the RCC Clocks 
    frequencies.
    [..] 
    (@) Important note: Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to
        select the RTC clock source; in this case the Backup domain will be reset in  
        order to modify the RTC Clock source, as consequence RTC registers (including 
        the backup registers) and RCC_BDCR register will be set to their reset values.
      
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the RCC extended peripherals clocks according to the specified
  *         parameters in the RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit: pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals
  *         clocks(I2S, SAI, LTDC, RTC, TIM, UARTs, USARTs, LTPIM, SDMMC...).
  *         
  * @note   Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select 
  *         the RTC clock source; in this case the Backup domain will be reset in  
  *         order to modify the RTC Clock source, as consequence RTC registers (including 
  *         the backup registers) are set to their reset values.
  *
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCCEx_PeriphCLKConfig(mut PeriphClkInit:
                                                       *mut RCC_PeriphCLKInitTypeDef)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg0: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut plli2sused: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllsaiused: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*----------------------------------- I2S configuration ----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x1 as libc::c_uint ==
           0x1 as libc::c_uint {
        /* Check the parameters */
        /* Configure I2S Clock source */
        let ref mut fresh0 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CFGR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               23 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        let ref mut fresh1 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CFGR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*PeriphClkInit).I2sClockSelection)
                                        as uint32_t as uint32_t);
        if (*PeriphClkInit).I2sClockSelection == 0 as libc::c_uint {
            plli2sused = 1 as libc::c_int as uint32_t
        }
    }
    /* Enable the PLLI2S when it's used as clock source for I2S */
    /*------------------------------------ SAI1 configuration --------------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x80000 as libc::c_uint ==
           0x80000 as libc::c_uint {
        /* Check the parameters */
        /* Configure SAI1 Clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR1
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR1 &
                                        !((0x3 as libc::c_uint) <<
                                              20 as libc::c_uint) |
                                        (*PeriphClkInit).Sai1ClockSelection);
        if (*PeriphClkInit).Sai1ClockSelection ==
               (0x1 as libc::c_uint) << 20 as libc::c_uint {
            plli2sused = 1 as libc::c_int as uint32_t
        }
        if (*PeriphClkInit).Sai1ClockSelection == 0 as libc::c_uint {
            pllsaiused = 1 as libc::c_int as uint32_t
        }
    }
    /* Enable the PLLI2S when it's used as clock source for SAI */
    /* Enable the PLLSAI when it's used as clock source for SAI */
    /*------------------------------------ SAI2 configuration --------------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x100000 as libc::c_uint ==
           0x100000 as libc::c_uint {
        /* Check the parameters */
        /* Configure SAI2 Clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR1
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR1 &
                                        !((0x3 as libc::c_uint) <<
                                              22 as libc::c_uint) |
                                        (*PeriphClkInit).Sai2ClockSelection);
        if (*PeriphClkInit).Sai2ClockSelection ==
               (0x1 as libc::c_uint) << 22 as libc::c_uint {
            plli2sused = 1 as libc::c_int as uint32_t
        }
        if (*PeriphClkInit).Sai2ClockSelection == 0 as libc::c_uint {
            pllsaiused = 1 as libc::c_int as uint32_t
        }
    }
    /* Enable the PLLI2S when it's used as clock source for SAI */
    /* Enable the PLLSAI when it's used as clock source for SAI */
    /*-------------------------------------- SPDIF-RX Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x1000000 as libc::c_uint ==
           0x1000000 as libc::c_uint {
        plli2sused = 1 as libc::c_int as uint32_t
    }
    /*------------------------------------ RTC configuration --------------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x20 as libc::c_uint ==
           0x20 as libc::c_uint {
        /* Check for RTC Parameters used to output RTCCLK */
        /* Enable Power Clock*/
        let mut tmpreg: uint32_t = 0;
        let ref mut fresh2 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).APB1ENR;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             28 as libc::c_uint) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).APB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            28 as libc::c_uint);
        let ref mut fresh3 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                   *mut PWR_TypeDef)).CR1;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        tickstart = HAL_GetTick();
        while (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                     *mut PWR_TypeDef)).CR1 &
                  (0x1 as libc::c_uint) << 8 as libc::c_uint ==
                  RESET as libc::c_int as libc::c_uint
              /* Enable write access to Backup domain */
              /* Get Start Tick*/
              /* Wait for Backup domain Write protection disable */
              {
            if HAL_GetTick().wrapping_sub(tickstart) >
                   100 as libc::c_int as uint32_t {
                return HAL_TIMEOUT
            }
        }
        tmpreg0 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).BDCR &
                (0x3 as libc::c_uint) << 8 as libc::c_uint;
        if tmpreg0 != 0 as libc::c_uint &&
               tmpreg0 !=
                   (*PeriphClkInit).RTCClockSelection &
                       (0x3 as libc::c_uint) << 8 as libc::c_uint {
            /* Reset the Backup domain only if the RTC Clock source selection is modified */
            /* Store the content of BDCR register before the reset of Backup Domain */
            tmpreg0 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR &
                    !((0x3 as libc::c_uint) << 8 as libc::c_uint);
            /* RTC Clock selection can be changed only if the Backup Domain is reset */
            let ref mut fresh4 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh4,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 16 as libc::c_uint) as
                                            uint32_t as uint32_t);
            let ref mut fresh5 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh5,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   16 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Restore the Content of BDCR register */
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as *mut RCC_TypeDef)).BDCR
                                            as *mut uint32_t, tmpreg0);
            /* Wait for LSE reactivation if LSE was enable prior to Backup Domain reset */
            if (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x3800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut RCC_TypeDef)).BDCR &
                   (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
                /* Get Start Tick*/
                tickstart = HAL_GetTick();
                /* Wait till LSE is ready */
                while (if (if 0x41 as libc::c_uint as uint8_t as libc::c_int
                                  >> 5 as libc::c_int == 1 as libc::c_int {
                               (*((0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x3800
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut RCC_TypeDef)).CR
                           } else {
                               (if 0x41 as libc::c_uint as uint8_t as
                                       libc::c_int >> 5 as libc::c_int ==
                                       2 as libc::c_int {
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).BDCR
                                } else {
                                    (if 0x41 as libc::c_uint as uint8_t as
                                            libc::c_int >> 5 as libc::c_int ==
                                            3 as libc::c_int {
                                         (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).CSR
                                     } else {
                                         (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).CIR
                                     })
                                })
                           }) &
                              (1 as libc::c_int as uint32_t) <<
                                  (0x41 as libc::c_uint as uint8_t as
                                       libc::c_int &
                                       0x1f as libc::c_int as uint8_t as
                                           libc::c_int) !=
                              0 as libc::c_int as libc::c_uint {
                           1 as libc::c_int
                       } else { 0 as libc::c_int }) == RESET as libc::c_int {
                    if HAL_GetTick().wrapping_sub(tickstart) >
                           5000 as libc::c_uint {
                        return HAL_TIMEOUT
                    }
                }
            }
        }
        if (*PeriphClkInit).RTCClockSelection &
               (0x3 as libc::c_uint) << 8 as libc::c_uint ==
               (0x3 as libc::c_uint) << 8 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as *mut RCC_TypeDef)).CFGR
                                            as *mut uint32_t,
                                        (*((0x40000000 as
                                                libc::c_uint).wrapping_add(0x20000
                                                                               as
                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                              as
                                                                                                              libc::c_uint)
                                               as *mut RCC_TypeDef)).CFGR &
                                            !((0x1f as libc::c_uint) <<
                                                  16 as libc::c_uint) |
                                            (*PeriphClkInit).RTCClockSelection
                                                &
                                                0xffffcff as libc::c_int as
                                                    libc::c_uint)
        } else {
            let ref mut fresh6 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR;
            ::core::ptr::write_volatile(fresh6,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1f as libc::c_uint) <<
                                                   16 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        };
        let ref mut fresh7 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).BDCR;
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*PeriphClkInit).RTCClockSelection &
                                             0xfff as libc::c_int as
                                                 libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /*------------------------------------ TIM configuration --------------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x10 as libc::c_uint ==
           0x10 as libc::c_uint {
        /* Check the parameters */
        /* Configure Timer Prescaler */
        let ref mut fresh8 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).DCKCFGR1;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               24 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        let ref mut fresh9 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).DCKCFGR1;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*PeriphClkInit).TIMPresSelection) as
                                        uint32_t as uint32_t)
    }
    /*-------------------------------------- I2C1 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x4000 as libc::c_uint ==
           0x4000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the I2C1 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              16 as libc::c_uint) |
                                        (*PeriphClkInit).I2c1ClockSelection)
    }
    /*-------------------------------------- I2C2 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x8000 as libc::c_uint ==
           0x8000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the I2C2 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              18 as libc::c_uint) |
                                        (*PeriphClkInit).I2c2ClockSelection)
    }
    /*-------------------------------------- I2C3 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x10000 as libc::c_uint ==
           0x10000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the I2C3 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              20 as libc::c_uint) |
                                        (*PeriphClkInit).I2c3ClockSelection)
    }
    /*-------------------------------------- I2C4 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x20000 as libc::c_uint ==
           0x20000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the I2C4 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              22 as libc::c_uint) |
                                        (*PeriphClkInit).I2c4ClockSelection)
    }
    /*-------------------------------------- USART1 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x40 as libc::c_uint ==
           0x40 as libc::c_uint {
        /* Check the parameters */
        /* Configure the USART1 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              0 as libc::c_uint) |
                                        (*PeriphClkInit).Usart1ClockSelection)
    }
    /*-------------------------------------- USART2 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x80 as libc::c_uint ==
           0x80 as libc::c_uint {
        /* Check the parameters */
        /* Configure the USART2 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              2 as libc::c_uint) |
                                        (*PeriphClkInit).Usart2ClockSelection)
    }
    /*-------------------------------------- USART3 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x100 as libc::c_uint ==
           0x100 as libc::c_uint {
        /* Check the parameters */
        /* Configure the USART3 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              4 as libc::c_uint) |
                                        (*PeriphClkInit).Usart3ClockSelection)
    }
    /*-------------------------------------- UART4 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x200 as libc::c_uint ==
           0x200 as libc::c_uint {
        /* Check the parameters */
        /* Configure the UART4 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              6 as libc::c_uint) |
                                        (*PeriphClkInit).Uart4ClockSelection)
    }
    /*-------------------------------------- UART5 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x400 as libc::c_uint ==
           0x400 as libc::c_uint {
        /* Check the parameters */
        /* Configure the UART5 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              8 as libc::c_uint) |
                                        (*PeriphClkInit).Uart5ClockSelection)
    }
    /*-------------------------------------- USART6 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x800 as libc::c_uint ==
           0x800 as libc::c_uint {
        /* Check the parameters */
        /* Configure the USART6 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              10 as libc::c_uint) |
                                        (*PeriphClkInit).Usart6ClockSelection)
    }
    /*-------------------------------------- UART7 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x1000 as libc::c_uint ==
           0x1000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the UART7 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              12 as libc::c_uint) |
                                        (*PeriphClkInit).Uart7ClockSelection)
    }
    /*-------------------------------------- UART8 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x2000 as libc::c_uint ==
           0x2000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the UART8 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              14 as libc::c_uint) |
                                        (*PeriphClkInit).Uart8ClockSelection)
    }
    /*--------------------------------------- CEC Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x400000 as libc::c_uint ==
           0x400000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the CEC clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x1 as libc::c_uint) <<
                                              26 as libc::c_uint) |
                                        (*PeriphClkInit).CecClockSelection)
    }
    /*-------------------------------------- CK48 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x200000 as libc::c_uint ==
           0x200000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the CLK48 source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x1 as libc::c_uint) <<
                                              27 as libc::c_uint) |
                                        (*PeriphClkInit).Clk48ClockSelection);
        if (*PeriphClkInit).Clk48ClockSelection ==
               (0x1 as libc::c_uint) << 27 as libc::c_uint {
            pllsaiused = 1 as libc::c_int as uint32_t
        }
    }
    /* Enable the PLLSAI when it's used as clock source for CK48 */
    /*-------------------------------------- LTDC Configuration -----------------------------------*/
    /* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
    /*-------------------------------------- LPTIM1 Configuration -----------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x40000 as libc::c_uint ==
           0x40000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the LTPIM1 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x3 as libc::c_uint) <<
                                              24 as libc::c_uint) |
                                        (*PeriphClkInit).Lptim1ClockSelection)
    }
    /*------------------------------------- SDMMC1 Configuration ------------------------------------*/
    if (*PeriphClkInit).PeriphClockSelection & 0x800000 as libc::c_uint ==
           0x800000 as libc::c_uint {
        /* Check the parameters */
        /* Configure the SDMMC1 clock source */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).DCKCFGR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).DCKCFGR2 &
                                        !((0x1 as libc::c_uint) <<
                                              28 as libc::c_uint) |
                                        (*PeriphClkInit).Sdmmc1ClockSelection)
    }
    /* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
    /*-------------------------------------- PLLI2S Configuration ---------------------------------*/
  /* PLLI2S is configured when a peripheral will use it as source clock : SAI1, SAI2, I2S or SPDIF-RX */
    if plli2sused == 1 as libc::c_int as libc::c_uint ||
           (*PeriphClkInit).PeriphClockSelection == 0x2000000 as libc::c_uint
       {
        /* Disable the PLLI2S */
        let ref mut fresh10 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh10,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               26 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Get Start Tick*/
        tickstart = HAL_GetTick();
        /* Wait till PLLI2S is disabled */
        while (if (if 0x3b as libc::c_uint as uint8_t as libc::c_int >>
                          5 as libc::c_int == 1 as libc::c_int {
                       (*((0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x3800
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut RCC_TypeDef)).CR
                   } else {
                       (if 0x3b as libc::c_uint as uint8_t as libc::c_int >>
                               5 as libc::c_int == 2 as libc::c_int {
                            (*((0x40000000 as
                                    libc::c_uint).wrapping_add(0x20000 as
                                                                   libc::c_uint).wrapping_add(0x3800
                                                                                                  as
                                                                                                  libc::c_uint)
                                   as *mut RCC_TypeDef)).BDCR
                        } else {
                            (if 0x3b as libc::c_uint as uint8_t as libc::c_int
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
                          (0x3b as libc::c_uint as uint8_t as libc::c_int &
                               0x1f as libc::c_int as uint8_t as libc::c_int)
                      != 0 as libc::c_int as libc::c_uint {
                   1 as libc::c_int
               } else { 0 as libc::c_int }) != RESET as libc::c_int {
            if HAL_GetTick().wrapping_sub(tickstart) >
                   100 as libc::c_int as libc::c_uint {
                /* return in case of Timeout detected */
                return HAL_TIMEOUT
            }
        }
        /* check for common PLLI2S Parameters */
        /*----------------- In Case of PLLI2S is selected as source clock for I2S -------------------*/
        if (*PeriphClkInit).PeriphClockSelection & 0x1 as libc::c_uint ==
               0x1 as libc::c_uint &&
               (*PeriphClkInit).I2sClockSelection == 0 as libc::c_uint {
            /* check for Parameters */
            /* Read PLLI2SP and PLLI2SQ value from PLLI2SCFGR register (this value is not needed for I2S configuration) */
            tmpreg0 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLI2SCFGR &
                     (0x3 as libc::c_uint) << 16 as libc::c_uint) >>
                    __RBIT((0x3 as libc::c_uint) <<
                               16 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            tmpreg1 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLI2SCFGR &
                     (0xf as libc::c_uint) << 24 as libc::c_uint) >>
                    __RBIT((0xf as libc::c_uint) <<
                               24 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut RCC_TypeDef)).PLLI2SCFGR
                                            as *mut uint32_t,
                                        (*PeriphClkInit).PLLI2S.PLLI2SN <<
                                            __RBIT((0x1ff as libc::c_uint) <<
                                                       6 as
                                                           libc::c_uint).leading_zeros()
                                                as i32 as uint8_t as
                                                libc::c_int |
                                            tmpreg0 <<
                                                __RBIT((0x3 as libc::c_uint)
                                                           <<
                                                           16 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            tmpreg1 <<
                                                __RBIT((0xf as libc::c_uint)
                                                           <<
                                                           24 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            (*PeriphClkInit).PLLI2S.PLLI2SR <<
                                                __RBIT((0x7 as libc::c_uint)
                                                           <<
                                                           28 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int)
        }
        if (*PeriphClkInit).PeriphClockSelection & 0x80000 as libc::c_uint ==
               0x80000 as libc::c_uint &&
               (*PeriphClkInit).Sai1ClockSelection ==
                   (0x1 as libc::c_uint) << 20 as libc::c_uint ||
               (*PeriphClkInit).PeriphClockSelection &
                   0x100000 as libc::c_uint == 0x100000 as libc::c_uint &&
                   (*PeriphClkInit).Sai2ClockSelection ==
                       (0x1 as libc::c_uint) << 22 as libc::c_uint {
            /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) x (PLLI2SN/PLLM) */
      /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
            /* Check for PLLI2S Parameters */
            /* Read PLLI2SP and PLLI2SR values from PLLI2SCFGR register (this value is not needed for SAI configuration) */
            tmpreg0 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLI2SCFGR &
                     (0x3 as libc::c_uint) << 16 as libc::c_uint) >>
                    __RBIT((0x3 as libc::c_uint) <<
                               16 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            tmpreg1 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLI2SCFGR &
                     (0x7 as libc::c_uint) << 28 as libc::c_uint) >>
                    __RBIT((0x7 as libc::c_uint) <<
                               28 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut RCC_TypeDef)).PLLI2SCFGR
                                            as *mut uint32_t,
                                        (*PeriphClkInit).PLLI2S.PLLI2SN <<
                                            __RBIT((0x1ff as libc::c_uint) <<
                                                       6 as
                                                           libc::c_uint).leading_zeros()
                                                as i32 as uint8_t as
                                                libc::c_int |
                                            tmpreg0 <<
                                                __RBIT((0x3 as libc::c_uint)
                                                           <<
                                                           16 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            (*PeriphClkInit).PLLI2S.PLLI2SQ <<
                                                __RBIT((0xf as libc::c_uint)
                                                           <<
                                                           24 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            tmpreg1 <<
                                                __RBIT((0x7 as libc::c_uint)
                                                           <<
                                                           28 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut RCC_TypeDef)).DCKCFGR1
                                            as *mut uint32_t,
                                        (*((0x40000000 as
                                                libc::c_uint).wrapping_add(0x20000
                                                                               as
                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                              as
                                                                                                              libc::c_uint)
                                               as *mut RCC_TypeDef)).DCKCFGR1
                                            &
                                            !((0x1f as libc::c_uint) <<
                                                  0 as libc::c_uint) |
                                            (*PeriphClkInit).PLLI2SDivQ.wrapping_sub(1
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         libc::c_uint))
        }
        if (*PeriphClkInit).PeriphClockSelection & 0x1000000 as libc::c_uint
               == 0x1000000 as libc::c_uint {
            /* Configure the PLLI2S division factors */      
      /* PLLI2S_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
      /* SAI_CLK(first level) = PLLI2S_VCO Output/PLLI2SQ */
            /* SAI_CLK_x = SAI_CLK(first level)/PLLI2SDIVQ */
            /*----------------- In Case of PLLI2S is selected as source clock for SAI -------------------*/
            /* check for Parameters */
            /* Read PLLI2SR value from PLLI2SCFGR register (this value is not needed for SPDIF-RX configuration) */
            tmpreg0 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLI2SCFGR &
                     (0xf as libc::c_uint) << 24 as libc::c_uint) >>
                    __RBIT((0xf as libc::c_uint) <<
                               24 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            tmpreg1 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLI2SCFGR &
                     (0x7 as libc::c_uint) << 28 as libc::c_uint) >>
                    __RBIT((0x7 as libc::c_uint) <<
                               28 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut RCC_TypeDef)).PLLI2SCFGR
                                            as *mut uint32_t,
                                        (*PeriphClkInit).PLLI2S.PLLI2SN <<
                                            __RBIT((0x1ff as libc::c_uint) <<
                                                       6 as
                                                           libc::c_uint).leading_zeros()
                                                as i32 as uint8_t as
                                                libc::c_int |
                                            (*PeriphClkInit).PLLI2S.PLLI2SP <<
                                                __RBIT((0x3 as libc::c_uint)
                                                           <<
                                                           16 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            tmpreg0 <<
                                                __RBIT((0xf as libc::c_uint)
                                                           <<
                                                           24 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            tmpreg1 <<
                                                __RBIT((0x7 as libc::c_uint)
                                                           <<
                                                           28 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int)
        }
        if (*PeriphClkInit).PeriphClockSelection & 0x2000000 as libc::c_uint
               == 0x2000000 as libc::c_uint {
            /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) x (PLLI2SN/PLLM) */
      /* SPDIFCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SP */
            /*----------------- In Case of PLLI2S is selected as source clock for SPDIF-RX -------------------*/
            /* Check for Parameters */
            /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) x (PLLI2SN/PLLI2SM) */
      /* SPDIFRXCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SP */
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut RCC_TypeDef)).PLLI2SCFGR
                                            as *mut uint32_t,
                                        (*PeriphClkInit).PLLI2S.PLLI2SN <<
                                            __RBIT((0x1ff as libc::c_uint) <<
                                                       6 as
                                                           libc::c_uint).leading_zeros()
                                                as i32 as uint8_t as
                                                libc::c_int |
                                            (*PeriphClkInit).PLLI2S.PLLI2SP <<
                                                __RBIT((0x3 as libc::c_uint)
                                                           <<
                                                           16 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            (*PeriphClkInit).PLLI2S.PLLI2SQ <<
                                                __RBIT((0xf as libc::c_uint)
                                                           <<
                                                           24 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            (*PeriphClkInit).PLLI2S.PLLI2SR <<
                                                __RBIT((0x7 as libc::c_uint)
                                                           <<
                                                           28 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int)
        }
        let ref mut fresh11 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             26 as libc::c_uint) as uint32_t
                                        as uint32_t);
        tickstart = HAL_GetTick();
        while (if (if 0x3b as libc::c_uint as uint8_t as libc::c_int >>
                          5 as libc::c_int == 1 as libc::c_int {
                       (*((0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0x3800
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut RCC_TypeDef)).CR
                   } else {
                       (if 0x3b as libc::c_uint as uint8_t as libc::c_int >>
                               5 as libc::c_int == 2 as libc::c_int {
                            (*((0x40000000 as
                                    libc::c_uint).wrapping_add(0x20000 as
                                                                   libc::c_uint).wrapping_add(0x3800
                                                                                                  as
                                                                                                  libc::c_uint)
                                   as *mut RCC_TypeDef)).BDCR
                        } else {
                            (if 0x3b as libc::c_uint as uint8_t as libc::c_int
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
                          (0x3b as libc::c_uint as uint8_t as libc::c_int &
                               0x1f as libc::c_int as uint8_t as libc::c_int)
                      != 0 as libc::c_int as libc::c_uint {
                   1 as libc::c_int
               } else { 0 as libc::c_int }) == RESET as libc::c_int
              /*----------------- In Case of PLLI2S is just selected  -----------------*/
              /* Enable the PLLI2S */
              /* Get Start Tick*/
              /* Wait till PLLI2S is ready */
              {
            if HAL_GetTick().wrapping_sub(tickstart) >
                   100 as libc::c_int as libc::c_uint {
                /* return in case of Timeout detected */
                return HAL_TIMEOUT
            }
        }
    }
    /*-------------------------------------- PLLSAI Configuration ---------------------------------*/
  /* PLLSAI is configured when a peripheral will use it as source clock : SAI1, SAI2, LTDC or CK48 */
    if pllsaiused == 1 as libc::c_int as libc::c_uint {
        /* Disable PLLSAI Clock */
        let ref mut fresh12 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               28 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Get Start Tick*/
        tickstart = HAL_GetTick();
        /* Wait till PLLSAI is disabled */
        while ((*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x3800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut RCC_TypeDef)).CR &
                   (0x1 as libc::c_uint) << 29 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 29 as libc::c_uint) as libc::c_int
                  != RESET as libc::c_int {
            if HAL_GetTick().wrapping_sub(tickstart) >
                   100 as libc::c_int as libc::c_uint {
                /* return in case of Timeout detected */
                return HAL_TIMEOUT
            }
        }
        /* Check the PLLSAI division factors */
        /*----------------- In Case of PLLSAI is selected as source clock for SAI -------------------*/
        if (*PeriphClkInit).PeriphClockSelection & 0x80000 as libc::c_uint ==
               0x80000 as libc::c_uint &&
               (*PeriphClkInit).Sai1ClockSelection == 0 as libc::c_uint ||
               (*PeriphClkInit).PeriphClockSelection &
                   0x100000 as libc::c_uint == 0x100000 as libc::c_uint &&
                   (*PeriphClkInit).Sai2ClockSelection == 0 as libc::c_uint {
            /* check for PLLSAIQ Parameter */
            /* Read PLLSAIP value from PLLSAICFGR register (this value is not needed for SAI configuration) */
            tmpreg0 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLSAICFGR &
                     (0x3 as libc::c_uint) << 16 as libc::c_uint) >>
                    __RBIT((0x3 as libc::c_uint) <<
                               16 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            tmpreg1 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLSAICFGR &
                     (0x7 as libc::c_uint) << 28 as libc::c_uint) >>
                    __RBIT((0x7 as libc::c_uint) <<
                               28 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut RCC_TypeDef)).PLLSAICFGR
                                            as *mut uint32_t,
                                        (*PeriphClkInit).PLLSAI.PLLSAIN <<
                                            __RBIT((0x1ff as libc::c_uint) <<
                                                       6 as
                                                           libc::c_uint).leading_zeros()
                                                as i32 as uint8_t as
                                                libc::c_int |
                                            tmpreg0 <<
                                                __RBIT((0x3 as libc::c_uint)
                                                           <<
                                                           16 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            (*PeriphClkInit).PLLSAI.PLLSAIQ <<
                                                __RBIT((0xf as libc::c_uint)
                                                           <<
                                                           24 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            tmpreg1 <<
                                                __RBIT((0x7 as libc::c_uint)
                                                           <<
                                                           28 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut RCC_TypeDef)).DCKCFGR1
                                            as *mut uint32_t,
                                        (*((0x40000000 as
                                                libc::c_uint).wrapping_add(0x20000
                                                                               as
                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                              as
                                                                                                              libc::c_uint)
                                               as *mut RCC_TypeDef)).DCKCFGR1
                                            &
                                            !((0x1f as libc::c_uint) <<
                                                  8 as libc::c_uint) |
                                            (*PeriphClkInit).PLLSAIDivQ.wrapping_sub(1
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         libc::c_uint)
                                                << 8 as libc::c_int)
        }
        if (*PeriphClkInit).PeriphClockSelection & 0x200000 as libc::c_uint ==
               0x200000 as libc::c_uint &&
               (*PeriphClkInit).Clk48ClockSelection ==
                   (0x1 as libc::c_uint) << 27 as libc::c_uint {
            /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
      /* SAI_CLK(first level) = PLLSAI_VCO Output/PLLSAIQ */
            /* SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ */
            /* check for Parameters */
            /* Read PLLSAIQ and PLLSAIR value from PLLSAICFGR register (this value is not needed for CK48 configuration) */
            tmpreg0 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLSAICFGR &
                     (0xf as libc::c_uint) << 24 as libc::c_uint) >>
                    __RBIT((0xf as libc::c_uint) <<
                               24 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            tmpreg1 =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLSAICFGR &
                     (0x7 as libc::c_uint) << 28 as libc::c_uint) >>
                    __RBIT((0x7 as libc::c_uint) <<
                               28 as libc::c_uint).leading_zeros() as i32 as
                        uint8_t as libc::c_int;
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut RCC_TypeDef)).PLLSAICFGR
                                            as *mut uint32_t,
                                        (*PeriphClkInit).PLLSAI.PLLSAIN <<
                                            __RBIT((0x1ff as libc::c_uint) <<
                                                       6 as
                                                           libc::c_uint).leading_zeros()
                                                as i32 as uint8_t as
                                                libc::c_int |
                                            (*PeriphClkInit).PLLSAI.PLLSAIP <<
                                                __RBIT((0x3 as libc::c_uint)
                                                           <<
                                                           16 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            tmpreg0 <<
                                                __RBIT((0xf as libc::c_uint)
                                                           <<
                                                           24 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int |
                                            tmpreg1 <<
                                                __RBIT((0x7 as libc::c_uint)
                                                           <<
                                                           28 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int)
        }
        let ref mut fresh13 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh13,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             28 as libc::c_uint) as uint32_t
                                        as uint32_t);
        tickstart = HAL_GetTick();
        while ((*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x3800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut RCC_TypeDef)).CR &
                   (0x1 as libc::c_uint) << 29 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 29 as libc::c_uint) as libc::c_int
                  == RESET as libc::c_int
              /* Configure the PLLSAI division factors */
      /* PLLSAI_VCO = f(VCO clock) = f(PLLSAI clock input) x (PLLI2SN/PLLM) */
      /* 48CLK = f(PLLSAI clock output) = f(VCO clock) / PLLSAIP */
              /*----------------- In Case of PLLSAI is selected as source clock for CLK48 -------------------*/   
    /* In Case of PLLI2S is selected as source clock for CK48 */
              /* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
              /* Enable PLLSAI Clock */
              /* Get Start Tick*/
              /* Wait till PLLSAI is ready */
              {
            if HAL_GetTick().wrapping_sub(tickstart) >
                   100 as libc::c_int as libc::c_uint {
                /* return in case of Timeout detected */
                return HAL_TIMEOUT
            }
        }
    }
    return HAL_OK;
}
/* *
  * @brief  Get the RCC_PeriphCLKInitTypeDef according to the internal
  *         RCC configuration registers.
  * @param  PeriphClkInit: pointer to the configured RCC_PeriphCLKInitTypeDef structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCCEx_GetPeriphCLKConfig(mut PeriphClkInit:
                                                          *mut RCC_PeriphCLKInitTypeDef) {
    let mut tempreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Set all possible values for the extended clock type parameter------------*/
    (*PeriphClkInit).PeriphClockSelection =
        0x1 as libc::c_uint | 0x40000 as libc::c_uint |
            0x80000 as libc::c_uint | 0x100000 as libc::c_uint |
            0x10 as libc::c_uint | 0x20 as libc::c_uint |
            0x400000 as libc::c_uint | 0x20000 as libc::c_uint |
            0x4000 as libc::c_uint | 0x8000 as libc::c_uint |
            0x10000 as libc::c_uint | 0x40 as libc::c_uint |
            0x80 as libc::c_uint | 0x100 as libc::c_uint |
            0x200 as libc::c_uint | 0x400 as libc::c_uint |
            0x800 as libc::c_uint | 0x1000 as libc::c_uint |
            0x2000 as libc::c_uint | 0x800000 as libc::c_uint |
            0x200000 as libc::c_uint;
    /* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
    /* Get the PLLI2S Clock configuration -----------------------------------------------*/
    (*PeriphClkInit).PLLI2S.PLLI2SN =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLI2SCFGR &
             (0x1ff as libc::c_uint) << 6 as libc::c_uint) >>
            __RBIT((0x1ff as libc::c_uint) <<
                       6 as libc::c_uint).leading_zeros() as i32 as uint8_t as
                libc::c_int;
    (*PeriphClkInit).PLLI2S.PLLI2SP =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLI2SCFGR &
             (0x3 as libc::c_uint) << 16 as libc::c_uint) >>
            __RBIT((0x3 as libc::c_uint) <<
                       16 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
    (*PeriphClkInit).PLLI2S.PLLI2SQ =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLI2SCFGR &
             (0xf as libc::c_uint) << 24 as libc::c_uint) >>
            __RBIT((0xf as libc::c_uint) <<
                       24 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
    (*PeriphClkInit).PLLI2S.PLLI2SR =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLI2SCFGR &
             (0x7 as libc::c_uint) << 28 as libc::c_uint) >>
            __RBIT((0x7 as libc::c_uint) <<
                       28 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
    /* Get the PLLSAI Clock configuration -----------------------------------------------*/
    (*PeriphClkInit).PLLSAI.PLLSAIN =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLSAICFGR &
             (0x1ff as libc::c_uint) << 6 as libc::c_uint) >>
            __RBIT((0x1ff as libc::c_uint) <<
                       6 as libc::c_uint).leading_zeros() as i32 as uint8_t as
                libc::c_int;
    (*PeriphClkInit).PLLSAI.PLLSAIP =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLSAICFGR &
             (0x3 as libc::c_uint) << 16 as libc::c_uint) >>
            __RBIT((0x3 as libc::c_uint) <<
                       16 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
    (*PeriphClkInit).PLLSAI.PLLSAIQ =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLSAICFGR &
             (0xf as libc::c_uint) << 24 as libc::c_uint) >>
            __RBIT((0xf as libc::c_uint) <<
                       24 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
    (*PeriphClkInit).PLLSAI.PLLSAIR =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLSAICFGR &
             (0x7 as libc::c_uint) << 28 as libc::c_uint) >>
            __RBIT((0x7 as libc::c_uint) <<
                       28 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
    /* Get the PLLSAI/PLLI2S division factors -------------------------------------------*/
    (*PeriphClkInit).PLLI2SDivQ =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).DCKCFGR1 &
             (0x1f as libc::c_uint) << 0 as libc::c_uint) >>
            __RBIT((0x1f as libc::c_uint) <<
                       0 as libc::c_uint).leading_zeros() as i32 as uint8_t as
                libc::c_int;
    (*PeriphClkInit).PLLSAIDivQ =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).DCKCFGR1 &
             (0x1f as libc::c_uint) << 8 as libc::c_uint) >>
            __RBIT((0x1f as libc::c_uint) <<
                       8 as libc::c_uint).leading_zeros() as i32 as uint8_t as
                libc::c_int;
    (*PeriphClkInit).PLLSAIDivR =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).DCKCFGR1 &
             (0x3 as libc::c_uint) << 16 as libc::c_uint) >>
            __RBIT((0x3 as libc::c_uint) <<
                       16 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
    /* Get the SAI1 clock configuration ----------------------------------------------*/
    (*PeriphClkInit).Sai1ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR1 &
            (0x3 as libc::c_uint) << 20 as libc::c_uint;
    /* Get the SAI2 clock configuration ----------------------------------------------*/
    (*PeriphClkInit).Sai2ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR1 &
            (0x3 as libc::c_uint) << 22 as libc::c_uint;
    /* Get the I2S clock configuration ------------------------------------------*/
    (*PeriphClkInit).I2sClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CFGR &
            (0x1 as libc::c_uint) << 23 as libc::c_uint;
    /* Get the I2C1 clock configuration ------------------------------------------*/
    (*PeriphClkInit).I2c1ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 16 as libc::c_uint;
    /* Get the I2C2 clock configuration ------------------------------------------*/
    (*PeriphClkInit).I2c2ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 18 as libc::c_uint;
    /* Get the I2C3 clock configuration ------------------------------------------*/
    (*PeriphClkInit).I2c3ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 20 as libc::c_uint;
    /* Get the I2C4 clock configuration ------------------------------------------*/
    (*PeriphClkInit).I2c4ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 22 as libc::c_uint;
    /* Get the USART1 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Usart1ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 0 as libc::c_uint;
    /* Get the USART2 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Usart2ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 2 as libc::c_uint;
    /* Get the USART3 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Usart3ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 4 as libc::c_uint;
    /* Get the UART4 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Uart4ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 6 as libc::c_uint;
    /* Get the UART5 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Uart5ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 8 as libc::c_uint;
    /* Get the USART6 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Usart6ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 10 as libc::c_uint;
    /* Get the UART7 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Uart7ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 12 as libc::c_uint;
    /* Get the UART8 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Uart8ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 14 as libc::c_uint;
    /* Get the LPTIM1 clock configuration ------------------------------------------*/
    (*PeriphClkInit).Lptim1ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x3 as libc::c_uint) << 24 as libc::c_uint;
    /* Get the CEC clock configuration -----------------------------------------------*/
    (*PeriphClkInit).CecClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x1 as libc::c_uint) << 26 as libc::c_uint;
    /* Get the CK48 clock configuration -----------------------------------------------*/
    (*PeriphClkInit).Clk48ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x1 as libc::c_uint) << 27 as libc::c_uint;
    /* Get the SDMMC1 clock configuration -----------------------------------------------*/
    (*PeriphClkInit).Sdmmc1ClockSelection =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR2 &
            (0x1 as libc::c_uint) << 28 as libc::c_uint;
    /* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
    /* Get the RTC Clock configuration -----------------------------------------------*/
    tempreg =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CFGR &
            (0x1f as libc::c_uint) << 16 as libc::c_uint;
    (*PeriphClkInit).RTCClockSelection =
        tempreg |
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).BDCR &
                (0x3 as libc::c_uint) << 8 as libc::c_uint;
    /* Get the TIM Prescaler configuration --------------------------------------------*/
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3800
                                                                             as
                                                                             libc::c_uint)
              as *mut RCC_TypeDef)).DCKCFGR1 &
           (0x1 as libc::c_uint) << 24 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint {
        (*PeriphClkInit).TIMPresSelection = 0 as libc::c_uint
    } else {
        (*PeriphClkInit).TIMPresSelection =
            (0x1 as libc::c_uint) << 24 as libc::c_uint
    };
}
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup RCCEx_Exported_Constants RCCEx Exported Constants
  * @{
  */
/* * @defgroup RCCEx_Periph_Clock_Selection RCC Periph Clock Selection
  * @{
  */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/* * @defgroup RCCEx_PLLI2SP_Clock_Divider RCCEx PLLI2SP Clock Divider
  * @{
  */
/* *
  * @}
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @defgroup RCCEx_PLLSAIP_Clock_Divider RCCEx PLLSAIP Clock Divider
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_PLLSAI_DIVR RCCEx PLLSAI DIVR
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_I2S_Clock_Source RCCEx I2S Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_SAI1_Clock_Source RCCEx SAI1 Clock Source
  * @{
  */
/* STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/* * @defgroup RCCEx_SAI2_Clock_Source RCCEx SAI2 Clock Source
  * @{
  */
/* STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/* * @defgroup RCCEx_CEC_Clock_Source RCCEx CEC Clock Source
  * @{
  */
/* CEC clock is HSI/488*/
/* *
  * @}
  */
/* * @defgroup RCCEx_USART1_Clock_Source RCCEx USART1 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_USART2_Clock_Source RCCEx USART2 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_USART3_Clock_Source RCCEx USART3 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_UART4_Clock_Source RCCEx UART4 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_UART5_Clock_Source RCCEx UART5 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_USART6_Clock_Source RCCEx USART6 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_UART7_Clock_Source RCCEx UART7 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_UART8_Clock_Source RCCEx UART8 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_I2C1_Clock_Source RCCEx I2C1 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_I2C2_Clock_Source RCCEx I2C2 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_I2C3_Clock_Source RCCEx I2C3 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_I2C4_Clock_Source RCCEx I2C4 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_LPTIM1_Clock_Source RCCEx LPTIM1 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_CLK48_Clock_Source RCCEx CLK48 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_TIM_Prescaler_Selection RCCEx TIM Prescaler Selection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_SDMMC1_Clock_Source RCCEx SDMMC1 Clock Source
  * @{
  */
/* *
  * @}
  */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup RCCEx_Exported_Macros RCCEx Exported Macros
  * @{
  */
/* * @defgroup RCCEx_Peripheral_Clock_Enable_Disable RCCEx_Peripheral_Clock_Enable_Disable
  * @brief  Enables or disables the AHB/APB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.   
  * @{
  */
/* * @brief  Enables or disables the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @brief  Enable ETHERNET clock.
  */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* *
  * @brief  Disable ETHERNET clock.
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Enable or disable the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  */
/* Delay after an RCC peripheral clock enabling */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* STM32F756x || STM32F777xx || STM32F779xx */
/* STM32F732xx || STM32F733xx */
/* * @brief  Enables or disables the AHB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx ||
          STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx ||
          STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F723xx || STM32F733xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F723xx || STM32F733xx */
/* *
  * @}
  */
/* * @defgroup RCCEx_Peripheral_Clock_Enable_Disable_Status Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB/APB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/* * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it. 
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @brief  Enable ETHERNET clock.
  */
/* *
  * @brief  Disable ETHERNET clock.
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Get the enable or disable status of the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it. 
  */
/* STM32F756xx || STM32F777xx || STM32F779xx */
/* STM32F732xx || STM32F733xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Get the enable or disable status of the AHB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  */
/* * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx ||
          STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Get the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F723xx || STM32F733xx */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F723xx || STM32F733xx */
/* *
  * @}
  */
/* * @defgroup RCCEx_Force_Release_Peripheral_Reset RCCEx Force Release Peripheral Reset
  * @brief  Forces or releases AHB/APB peripheral reset.
  * @{
  */
/* * @brief  Force or release AHB1 peripheral reset.
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Force or release AHB2 peripheral reset.
  */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F756xx || STM32F777xx || STM32F779xx */
/* STM32F732xx || STM32F733xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Force or release AHB3 peripheral reset
  */
/* * @brief  Force or release APB1 peripheral reset.
  */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Force or release APB2 peripheral reset.
  */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F723xx || STM32F733xx */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F723xx || STM32F733xx */
/* STM32F769xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/* * @defgroup RCCEx_Peripheral_Clock_Sleep_Enable_Disable RCCEx Peripheral Clock Sleep Enable Disable
  * @brief  Enables or disables the AHB/APB peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/* * @brief  Enable or disable the AHB1 peripheral clock during Low Power (Sleep) mode.
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Enable or disable the AHB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F756xx || STM32F777xx || STM32F779xx */
/* STM32F732xx || STM32F733xx */
/* * @brief  Enable or disable the AHB3 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* * @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx ||
          STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/* * @defgroup RCC_Clock_Sleep_Enable_Disable_Status AHB/APB Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the AHB/APB peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/* * @brief  Get the enable or disable status of the AHB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.  
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Get the enable or disable status of the AHB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F756xx || STM32F777xx || STM32F779xx */
/* STM32F732xx || STM32F733xx */
/* * @brief  Get the enable or disable status of the AHB3 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* * @brief  Get the enable or disable status of the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx ||
          STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx ||
          STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Get the enable or disable status of the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F746xx || STM32F756xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/*------------------------------- PLL Configuration --------------------------*/
/* * @brief  Macro to configure the main PLL clock source, multiplication and division factors.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __RCC_PLLSource__: specifies the PLL entry clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_PLLSOURCE_HSI: HSI oscillator clock selected as PLL clock entry
  *            @arg RCC_PLLSOURCE_HSE: HSE oscillator clock selected as PLL clock entry
  * @note   This clock source (RCC_PLLSource) is common for the main PLL and PLLI2S.  
  * @param  __PLLM__: specifies the division factor for PLL VCO input clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 63.
  * @note   You have to set the PLLM parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
  *         of 2 MHz to limit PLL jitter.
  * @param  __PLLN__: specifies the multiplication factor for PLL VCO output clock
  *         This parameter must be a number between Min_Data = 50 and Max_Data = 432.
  * @note   You have to set the PLLN parameter correctly to ensure that the VCO
  *         output frequency is between 100 and 432 MHz.
  * @param  __PLLP__: specifies the division factor for main system clock (SYSCLK)
  *         This parameter must be a number in the range {2, 4, 6, or 8}.
  * @note   You have to set the PLLP parameter correctly to not exceed 216 MHz on
  *         the System clock frequency.
  * @param  __PLLQ__: specifies the division factor for OTG FS, SDMMC and RNG clocks
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 15.
  * @note   If the USB OTG FS is used in your application, you have to set the
  *         PLLQ parameter correctly to have 48 MHz clock for the USB. However,
  *         the SDMMC and RNG need a frequency lower than or equal to 48 MHz to work
  *         correctly.
  */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/*---------------------------------------------------------------------------------------------*/
/* * @brief  Macro to configure the Timers clocks prescalers 
  * @param  __PRESC__ : specifies the Timers clocks prescalers selection
  *         This parameter can be one of the following values:
  *            @arg RCC_TIMPRES_DESACTIVATED: The Timers kernels clocks prescaler is 
  *                 equal to HPRE if PPREx is corresponding to division by 1 or 2, 
  *                 else it is equal to [(HPRE * PPREx) / 2] if PPREx is corresponding to 
  *                 division by 4 or more.       
  *            @arg RCC_TIMPRES_ACTIVATED: The Timers kernels clocks prescaler is 
  *                 equal to HPRE if PPREx is corresponding to division by 1, 2 or 4, 
  *                 else it is equal to [(HPRE * PPREx) / 4] if PPREx is corresponding 
  *                 to division by 8 or more.
  */
/* * @brief Macros to Enable or Disable the PLLISAI. 
  * @note  The PLLSAI is disabled by hardware when entering STOP and STANDBY modes. 
  */
/* * @brief  Macro to configure the PLLSAI clock multiplication and division factors.
  * @note   This function must be used only when the PLLSAI is disabled.
  * @note   PLLSAI clock source is common with the main PLL (configured in 
  *         RCC_PLLConfig function )
  * @param  __PLLSAIN__: specifies the multiplication factor for PLLSAI VCO output clock.
  *         This parameter must be a number between Min_Data = 50 and Max_Data = 432.
  * @note   You have to set the PLLSAIN parameter correctly to ensure that the VCO 
  *         output frequency is between Min_Data = 100 and Max_Data = 432 MHz.
  * @param  __PLLSAIP__: specifies the division factor for USB, RNG, SDMMC clocks
  *         This parameter can be a value of @ref RCCEx_PLLSAIP_Clock_Divider.                                                  
  * @param  __PLLSAIQ__: specifies the division factor for SAI clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 15.
  * @param  __PLLSAIR__: specifies the division factor for LTDC clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 7.
  */
/* * @brief  Macro to configure the PLLI2S clock multiplication and division factors.
  * @note   This macro must be used only when the PLLI2S is disabled.
  * @note   PLLI2S clock source is common with the main PLL (configured in 
  *         HAL_RCC_ClockConfig() API)             
  * @param  __PLLI2SN__: specifies the multiplication factor for PLLI2S VCO output clock.
  *         This parameter must be a number between Min_Data = 50 and Max_Data = 432.
  * @note   You have to set the PLLI2SN parameter correctly to ensure that the VCO 
  *         output frequency is between Min_Data = 100 and Max_Data = 432 MHz.
  * @param  __PLLI2SP__: specifies the division factor for SPDDIF-RX clock.
  *         This parameter can be a value of @ref RCCEx_PLLI2SP_Clock_Divider.                                 
  * @param  __PLLI2SQ__: specifies the division factor for SAI clock.
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 15. 
  * @param  __PLLI2SR__: specifies the division factor for I2S clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 7.
  * @note   You have to set the PLLI2SR parameter correctly to not exceed 192 MHz
  *         on the I2S clock frequency. 
  */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx */
/* * @brief  Macro to configure the SAI clock Divider coming from PLLI2S.
  * @note   This function must be called before enabling the PLLI2S.          
  * @param  __PLLI2SDivQ__: specifies the PLLI2S division factor for SAI1 clock .
  *          This parameter must be a number between 1 and 32.
  *          SAI1 clock frequency = f(PLLI2SQ) / __PLLI2SDivQ__ 
  */
/* * @brief  Macro to configure the SAI clock Divider coming from PLLSAI.
  * @note   This function must be called before enabling the PLLSAI.
  * @param  __PLLSAIDivQ__: specifies the PLLSAI division factor for SAI1 clock .
  *         This parameter must be a number between Min_Data = 1 and Max_Data = 32.
  *         SAI1 clock frequency = f(PLLSAIQ) / __PLLSAIDivQ__  
  */
/* * @brief  Macro to configure the LTDC clock Divider coming from PLLSAI.
  * @note   This function must be called before enabling the PLLSAI. 
  * @param  __PLLSAIDivR__: specifies the PLLSAI division factor for LTDC clock .
  *          This parameter can be a value of @ref RCCEx_PLLSAI_DIVR.
  *          LTDC clock frequency = f(PLLSAIR) / __PLLSAIDivR__ 
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* * @brief  Macro to configure SAI1 clock source selection.
  * @note   This function must be called before enabling PLLSAI, PLLI2S and  
  *         the SAI clock.
  * @param  __SOURCE__: specifies the SAI1 clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_SAI1CLKSOURCE_PLLI2S: PLLI2S_Q clock divided by PLLI2SDIVQ used 
  *                                           as SAI1 clock. 
  *            @arg RCC_SAI1CLKSOURCE_PLLSAI: PLLISAI_Q clock divided by PLLSAIDIVQ used 
  *                                           as SAI1 clock.
  *            @arg RCC_SAI1CLKSOURCE_PIN: External clock mapped on the I2S_CKIN pin
  *                                        used as SAI1 clock.
  *            @arg RCC_SAI1CLKSOURCE_PLLSRC: HSI or HSE depending from PLL Source clock 
  *                                           used as SAI1 clock.
  * @note      The RCC_SAI1CLKSOURCE_PLLSRC value is only available with STM32F767/769/777/779xx Devices                               
  */
/* * @brief  Macro to get the SAI1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_SAI1CLKSOURCE_PLLI2S: PLLI2S_Q clock divided by PLLI2SDIVQ used 
  *                                           as SAI1 clock. 
  *            @arg RCC_SAI1CLKSOURCE_PLLSAI: PLLISAI_Q clock divided by PLLSAIDIVQ used 
  *                                           as SAI1 clock.
  *            @arg RCC_SAI1CLKSOURCE_PIN: External clock mapped on the I2S_CKIN pin
  *                                        used as SAI1 clock.
  *            @arg RCC_SAI1CLKSOURCE_PLLSRC: HSI or HSE depending from PLL Source clock 
  *                                           used as SAI1 clock.
  * @note      The RCC_SAI1CLKSOURCE_PLLSRC value is only available with STM32F767/769/777/779xx Devices                               
  */
/* * @brief  Macro to configure SAI2 clock source selection.
  * @note   This function must be called before enabling PLLSAI, PLLI2S and  
  *         the SAI clock.
  * @param  __SOURCE__: specifies the SAI2 clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_SAI2CLKSOURCE_PLLI2S: PLLI2S_Q clock divided by PLLI2SDIVQ used 
  *                                           as SAI2 clock. 
  *            @arg RCC_SAI2CLKSOURCE_PLLSAI: PLLISAI_Q clock divided by PLLSAIDIVQ used 
  *                                           as SAI2 clock. 
  *            @arg RCC_SAI2CLKSOURCE_PIN: External clock mapped on the I2S_CKIN pin
  *                                        used as SAI2 clock.
  *            @arg RCC_SAI2CLKSOURCE_PLLSRC: HSI or HSE depending from PLL Source clock 
  *                                           used as SAI2 clock.
  * @note      The RCC_SAI2CLKSOURCE_PLLSRC value is only available with STM32F767/769/777/779xx Devices                                
  */
/* * @brief  Macro to get the SAI2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_SAI2CLKSOURCE_PLLI2S: PLLI2S_Q clock divided by PLLI2SDIVQ used 
  *                                           as SAI2 clock. 
  *            @arg RCC_SAI2CLKSOURCE_PLLSAI: PLLISAI_Q clock divided by PLLSAIDIVQ used 
  *                                           as SAI2 clock.
  *            @arg RCC_SAI2CLKSOURCE_PIN: External clock mapped on the I2S_CKIN pin
  *                                        used as SAI2 clock.
  *            @arg RCC_SAI2CLKSOURCE_PLLSRC: HSI or HSE depending from PLL Source clock 
  *                                           used as SAI2 clock.
  * @note      The RCC_SAI2CLKSOURCE_PLLSRC value is only available with STM32F767/769/777/779xx Devices                              
  */
/* * @brief Enable PLLSAI_RDY interrupt.
  */
/* * @brief Disable PLLSAI_RDY interrupt.
  */
/* * @brief Clear the PLLSAI RDY interrupt pending bits.
  */
/* * @brief Check the PLLSAI RDY interrupt has occurred or not.
  * @retval The new state (TRUE or FALSE).
  */
/* * @brief  Check PLLSAI RDY flag is set or not.
  * @retval The new state (TRUE or FALSE).
  */
/* * @brief  Macro to Get I2S clock source selection.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2SCLKSOURCE_PLLI2S: PLLI2S VCO output clock divided by PLLI2SR used as I2S clock. 
  *            @arg RCC_I2SCLKSOURCE_EXT: External clock mapped on the I2S_CKIN pin used as I2S clock source
  */
/* * @brief  Macro to configure the I2C1 clock (I2C1CLK).
  *
  * @param  __I2C1_CLKSOURCE__: specifies the I2C1 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2C1CLKSOURCE_PCLK1: PCLK1 selected as I2C1 clock
  *            @arg RCC_I2C1CLKSOURCE_HSI: HSI selected as I2C1 clock
  *            @arg RCC_I2C1CLKSOURCE_SYSCLK: System Clock selected as I2C1 clock
  */
/* * @brief  Macro to get the I2C1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2C1CLKSOURCE_PCLK1: PCLK1 selected as I2C1 clock
  *            @arg RCC_I2C1CLKSOURCE_HSI: HSI selected as I2C1 clock
  *            @arg RCC_I2C1CLKSOURCE_SYSCLK: System Clock selected as I2C1 clock
  */
/* * @brief  Macro to configure the I2C2 clock (I2C2CLK).
  *
  * @param  __I2C2_CLKSOURCE__: specifies the I2C2 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2C2CLKSOURCE_PCLK1: PCLK1 selected as I2C2 clock
  *            @arg RCC_I2C2CLKSOURCE_HSI: HSI selected as I2C2 clock
  *            @arg RCC_I2C2CLKSOURCE_SYSCLK: System Clock selected as I2C2 clock
  */
/* * @brief  Macro to get the I2C2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2C2CLKSOURCE_PCLK1: PCLK1 selected as I2C2 clock
  *            @arg RCC_I2C2CLKSOURCE_HSI: HSI selected as I2C2 clock
  *            @arg RCC_I2C2CLKSOURCE_SYSCLK: System Clock selected as I2C2 clock
  */
/* * @brief  Macro to configure the I2C3 clock (I2C3CLK).
  *
  * @param  __I2C3_CLKSOURCE__: specifies the I2C3 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2C3CLKSOURCE_PCLK1: PCLK1 selected as I2C3 clock
  *            @arg RCC_I2C3CLKSOURCE_HSI: HSI selected as I2C3 clock
  *            @arg RCC_I2C3CLKSOURCE_SYSCLK: System Clock selected as I2C3 clock
  */
/* * @brief  macro to get the I2C3 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2C3CLKSOURCE_PCLK1: PCLK1 selected as I2C3 clock
  *            @arg RCC_I2C3CLKSOURCE_HSI: HSI selected as I2C3 clock
  *            @arg RCC_I2C3CLKSOURCE_SYSCLK: System Clock selected as I2C3 clock
  */
/* * @brief  Macro to configure the I2C4 clock (I2C4CLK).
  *
  * @param  __I2C4_CLKSOURCE__: specifies the I2C4 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2C4CLKSOURCE_PCLK1: PCLK1 selected as I2C4 clock
  *            @arg RCC_I2C4CLKSOURCE_HSI: HSI selected as I2C4 clock
  *            @arg RCC_I2C4CLKSOURCE_SYSCLK: System Clock selected as I2C4 clock
  */
/* * @brief  macro to get the I2C4 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_I2C4CLKSOURCE_PCLK1: PCLK1 selected as I2C4 clock
  *            @arg RCC_I2C4CLKSOURCE_HSI: HSI selected as I2C4 clock
  *            @arg RCC_I2C4CLKSOURCE_SYSCLK: System Clock selected as I2C4 clock
  */
/* * @brief  Macro to configure the USART1 clock (USART1CLK).
  *
  * @param  __USART1_CLKSOURCE__: specifies the USART1 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_USART1CLKSOURCE_PCLK2: PCLK2 selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_HSI: HSI selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_SYSCLK: System Clock selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_LSE: LSE selected as USART1 clock
  */
/* * @brief  macro to get the USART1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_USART1CLKSOURCE_PCLK2: PCLK2 selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_HSI: HSI selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_SYSCLK: System Clock selected as USART1 clock
  *            @arg RCC_USART1CLKSOURCE_LSE: LSE selected as USART1 clock
  */
/* * @brief  Macro to configure the USART2 clock (USART2CLK).
  *
  * @param  __USART2_CLKSOURCE__: specifies the USART2 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_USART2CLKSOURCE_PCLK1: PCLK1 selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_HSI: HSI selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_SYSCLK: System Clock selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_LSE: LSE selected as USART2 clock
  */
/* * @brief  macro to get the USART2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_USART2CLKSOURCE_PCLK1: PCLK1 selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_HSI: HSI selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_SYSCLK: System Clock selected as USART2 clock
  *            @arg RCC_USART2CLKSOURCE_LSE: LSE selected as USART2 clock
  */
/* * @brief  Macro to configure the USART3 clock (USART3CLK).
  *
  * @param  __USART3_CLKSOURCE__: specifies the USART3 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_USART3CLKSOURCE_PCLK1: PCLK1 selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_HSI: HSI selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_SYSCLK: System Clock selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_LSE: LSE selected as USART3 clock
  */
/* * @brief  macro to get the USART3 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_USART3CLKSOURCE_PCLK1: PCLK1 selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_HSI: HSI selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_SYSCLK: System Clock selected as USART3 clock
  *            @arg RCC_USART3CLKSOURCE_LSE: LSE selected as USART3 clock
  */
/* * @brief  Macro to configure the UART4 clock (UART4CLK).
  *
  * @param  __UART4_CLKSOURCE__: specifies the UART4 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_UART4CLKSOURCE_PCLK1: PCLK1 selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_HSI: HSI selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_SYSCLK: System Clock selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_LSE: LSE selected as UART4 clock
  */
/* * @brief  macro to get the UART4 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_UART4CLKSOURCE_PCLK1: PCLK1 selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_HSI: HSI selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_SYSCLK: System Clock selected as UART4 clock
  *            @arg RCC_UART4CLKSOURCE_LSE: LSE selected as UART4 clock
  */
/* * @brief  Macro to configure the UART5 clock (UART5CLK).
  *
  * @param  __UART5_CLKSOURCE__: specifies the UART5 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_UART5CLKSOURCE_PCLK1: PCLK1 selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_HSI: HSI selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_SYSCLK: System Clock selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_LSE: LSE selected as UART5 clock
  */
/* * @brief  macro to get the UART5 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_UART5CLKSOURCE_PCLK1: PCLK1 selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_HSI: HSI selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_SYSCLK: System Clock selected as UART5 clock
  *            @arg RCC_UART5CLKSOURCE_LSE: LSE selected as UART5 clock
  */
/* * @brief  Macro to configure the USART6 clock (USART6CLK).
  *
  * @param  __USART6_CLKSOURCE__: specifies the USART6 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_USART6CLKSOURCE_PCLK1: PCLK1 selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_HSI: HSI selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_SYSCLK: System Clock selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_LSE: LSE selected as USART6 clock
  */
/* * @brief  macro to get the USART6 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_USART6CLKSOURCE_PCLK1: PCLK1 selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_HSI: HSI selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_SYSCLK: System Clock selected as USART6 clock
  *            @arg RCC_USART6CLKSOURCE_LSE: LSE selected as USART6 clock
  */
/* * @brief  Macro to configure the UART7 clock (UART7CLK).
  *
  * @param  __UART7_CLKSOURCE__: specifies the UART7 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_UART7CLKSOURCE_PCLK1: PCLK1 selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_HSI: HSI selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_SYSCLK: System Clock selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_LSE: LSE selected as UART7 clock
  */
/* * @brief  macro to get the UART7 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_UART7CLKSOURCE_PCLK1: PCLK1 selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_HSI: HSI selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_SYSCLK: System Clock selected as UART7 clock
  *            @arg RCC_UART7CLKSOURCE_LSE: LSE selected as UART7 clock
  */
/* * @brief  Macro to configure the UART8 clock (UART8CLK).
  *
  * @param  __UART8_CLKSOURCE__: specifies the UART8 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_UART8CLKSOURCE_PCLK1: PCLK1 selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_HSI: HSI selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_SYSCLK: System Clock selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_LSE: LSE selected as UART8 clock
  */
/* * @brief  macro to get the UART8 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_UART8CLKSOURCE_PCLK1: PCLK1 selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_HSI: HSI selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_SYSCLK: System Clock selected as UART8 clock
  *            @arg RCC_UART8CLKSOURCE_LSE: LSE selected as UART8 clock
  */
/* * @brief  Macro to configure the LPTIM1 clock (LPTIM1CLK).
  *
  * @param  __LPTIM1_CLKSOURCE__: specifies the LPTIM1 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_LPTIM1CLKSOURCE_PCLK1: PCLK selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_HSI: HSI selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_LSI: LSI selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_LSE: LSE selected as LPTIM1 clock
  */
/* * @brief  macro to get the LPTIM1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_LPTIM1CLKSOURCE_PCLK1: PCLK selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_HSI: HSI selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_LSI: LSI selected as LPTIM1 clock
  *            @arg RCC_LPTIM1CLKSOURCE_LSE: LSE selected as LPTIM1 clock
  */
/* * @brief  Macro to configure the CEC clock (CECCLK).
  *
  * @param  __CEC_CLKSOURCE__: specifies the CEC clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_CECCLKSOURCE_LSE: LSE selected as CEC clock
  *            @arg RCC_CECCLKSOURCE_HSI: HSI divided by 488 selected as CEC clock
  */
/* * @brief  macro to get the CEC clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_CECCLKSOURCE_LSE: LSE selected as CEC clock
  *            @arg RCC_CECCLKSOURCE_HSI: HSI selected as CEC clock
  */
/* * @brief  Macro to configure the CLK48 source (CLK48CLK).
  *
  * @param  __CLK48_SOURCE__: specifies the CLK48 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_CLK48SOURCE_PLL: PLL selected as CLK48 source
  *            @arg RCC_CLK48SOURCE_PLLSAIP: PLLSAIP selected as CLK48 source
  */
/* * @brief  macro to get the CLK48 source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_CLK48SOURCE_PLL: PLL used as CLK48 source
  *            @arg RCC_CLK48SOURCE_PLLSAIP: PLLSAIP used as CLK48 source
  */
/* * @brief  Macro to configure the SDMMC1 clock (SDMMC1CLK).
  *
  * @param  __SDMMC1_CLKSOURCE__: specifies the SDMMC1 clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_SDMMC1CLKSOURCE_CLK48: CLK48 selected as SDMMC clock
  *            @arg RCC_SDMMC1CLKSOURCE_SYSCLK: SYSCLK selected as SDMMC clock
  */
/* * @brief  macro to get the SDMMC1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_SDMMC1CLKSOURCE_CLK48: CLK48 selected as SDMMC1 clock
  *            @arg RCC_SDMMC1CLKSOURCE_SYSCLK: SYSCLK selected as SDMMC1 clock
  */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F769xx || STM32F779xx */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup RCCEx_Exported_Functions_Group1
  * @{
  */
/* STM32F745xx || STM32F746xx || STM32F756xx || STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx */
/* *
  * @brief  Return the peripheral clock frequency for a given peripheral(SAI..) 
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk: Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg RCC_PERIPHCLK_SAI1: SAI1 peripheral clock
  *            @arg RCC_PERIPHCLK_SAI2: SAI2 peripheral clock
  * @retval Frequency in KHz
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCCEx_GetPeriphCLKFreq(mut PeriphClk: uint32_t)
 -> uint32_t {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* This variable is used to store the SAI clock frequency (value in Hz) */
    let mut frequency: uint32_t = 0 as libc::c_int as uint32_t;
    /* This variable is used to store the VCO Input (value in Hz) */
    let mut vcoinput: uint32_t = 0 as libc::c_int as uint32_t;
    /* This variable is used to store the SAI clock source */
    let mut saiclocksource: uint32_t = 0 as libc::c_int as uint32_t;
    if PeriphClk == 0x80000 as libc::c_uint {
        saiclocksource =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).DCKCFGR1;
        saiclocksource &= (0x3 as libc::c_uint) << 20 as libc::c_uint;
        match saiclocksource {
            0 => {
                /* PLLSAI is the clock source for SAI1 */
                /* Configure the PLLSAI division factor */
        /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x3800
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut RCC_TypeDef)).PLLCFGR &
                       (0x1 as libc::c_uint) << 22 as libc::c_uint ==
                       0 as libc::c_uint {
                    /* In Case the PLL Source is HSI (Internal Clock) */
                    vcoinput =
                        (16000000 as
                             libc::c_uint).wrapping_div((*((0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut RCC_TypeDef)).PLLCFGR
                                                            &
                                                            (0x3f as
                                                                 libc::c_uint)
                                                                <<
                                                                0 as
                                                                    libc::c_uint)
                } else {
                    /* In Case the PLL Source is HSE (External Clock) */
                    vcoinput =
                        (8000000 as libc::c_int as
                             libc::c_uint).wrapping_div((*((0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut RCC_TypeDef)).PLLCFGR
                                                            &
                                                            (0x3f as
                                                                 libc::c_uint)
                                                                <<
                                                                0 as
                                                                    libc::c_uint)
                }
                /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
        /* SAI_CLK(first level) = PLLSAI_VCO Output/PLLSAIQ */
                tmpreg =
                    ((*((0x40000000 as
                             libc::c_uint).wrapping_add(0x20000 as
                                                            libc::c_uint).wrapping_add(0x3800
                                                                                           as
                                                                                           libc::c_uint)
                            as *mut RCC_TypeDef)).PLLSAICFGR &
                         (0xf as libc::c_uint) << 24 as libc::c_uint) >>
                        24 as libc::c_int;
                frequency =
                    vcoinput.wrapping_mul(((*((0x40000000 as
                                                   libc::c_uint).wrapping_add(0x20000
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                                 as
                                                                                                                 libc::c_uint)
                                                  as
                                                  *mut RCC_TypeDef)).PLLSAICFGR
                                               &
                                               (0x1ff as libc::c_uint) <<
                                                   6 as libc::c_uint) >>
                                              6 as
                                                  libc::c_int).wrapping_div(tmpreg);
                /* SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ */
                tmpreg =
                    (((*((0x40000000 as
                              libc::c_uint).wrapping_add(0x20000 as
                                                             libc::c_uint).wrapping_add(0x3800
                                                                                            as
                                                                                            libc::c_uint)
                             as *mut RCC_TypeDef)).DCKCFGR1 &
                          (0x1f as libc::c_uint) << 8 as libc::c_uint) >>
                         8 as
                             libc::c_int).wrapping_add(1 as libc::c_int as
                                                           libc::c_uint);
                frequency = frequency.wrapping_div(tmpreg)
            }
            1048576 => {
                /* PLLI2S is the clock source for SAI1 */
                /* Configure the PLLI2S division factor */
        /* PLLI2S_VCO Input  = PLL_SOURCE/PLLM */
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x3800
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut RCC_TypeDef)).PLLCFGR &
                       (0x1 as libc::c_uint) << 22 as libc::c_uint ==
                       0 as libc::c_uint {
                    /* In Case the PLL Source is HSI (Internal Clock) */
                    vcoinput =
                        (16000000 as
                             libc::c_uint).wrapping_div((*((0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut RCC_TypeDef)).PLLCFGR
                                                            &
                                                            (0x3f as
                                                                 libc::c_uint)
                                                                <<
                                                                0 as
                                                                    libc::c_uint)
                } else {
                    /* In Case the PLL Source is HSE (External Clock) */
                    vcoinput =
                        (8000000 as libc::c_int as
                             libc::c_uint).wrapping_div((*((0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut RCC_TypeDef)).PLLCFGR
                                                            &
                                                            (0x3f as
                                                                 libc::c_uint)
                                                                <<
                                                                0 as
                                                                    libc::c_uint)
                }
                /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
        /* SAI_CLK(first level) = PLLI2S_VCO Output/PLLI2SQ */
                tmpreg =
                    ((*((0x40000000 as
                             libc::c_uint).wrapping_add(0x20000 as
                                                            libc::c_uint).wrapping_add(0x3800
                                                                                           as
                                                                                           libc::c_uint)
                            as *mut RCC_TypeDef)).PLLI2SCFGR &
                         (0xf as libc::c_uint) << 24 as libc::c_uint) >>
                        24 as libc::c_int;
                frequency =
                    vcoinput.wrapping_mul(((*((0x40000000 as
                                                   libc::c_uint).wrapping_add(0x20000
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                                 as
                                                                                                                 libc::c_uint)
                                                  as
                                                  *mut RCC_TypeDef)).PLLI2SCFGR
                                               &
                                               (0x1ff as libc::c_uint) <<
                                                   6 as libc::c_uint) >>
                                              6 as
                                                  libc::c_int).wrapping_div(tmpreg);
                /* SAI_CLK_x = SAI_CLK(first level)/PLLI2SDIVQ */
                tmpreg =
                    ((*((0x40000000 as
                             libc::c_uint).wrapping_add(0x20000 as
                                                            libc::c_uint).wrapping_add(0x3800
                                                                                           as
                                                                                           libc::c_uint)
                            as *mut RCC_TypeDef)).DCKCFGR1 &
                         (0x1f as libc::c_uint) <<
                             0 as
                                 libc::c_uint).wrapping_add(1 as libc::c_int
                                                                as
                                                                libc::c_uint);
                frequency = frequency.wrapping_div(tmpreg)
            }
            2097152 => {
                /* External clock is the clock source for SAI1 */
                frequency = 12288000 as libc::c_uint
            }
            _ => { }
        }
    }
    if PeriphClk == 0x100000 as libc::c_uint {
        saiclocksource =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).DCKCFGR1;
        saiclocksource &= (0x3 as libc::c_uint) << 22 as libc::c_uint;
        match saiclocksource {
            0 => {
                /* PLLSAI is the clock source for SAI*/
                /* Configure the PLLSAI division factor */
        /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x3800
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut RCC_TypeDef)).PLLCFGR &
                       (0x1 as libc::c_uint) << 22 as libc::c_uint ==
                       0 as libc::c_uint {
                    /* In Case the PLL Source is HSI (Internal Clock) */
                    vcoinput =
                        (16000000 as
                             libc::c_uint).wrapping_div((*((0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut RCC_TypeDef)).PLLCFGR
                                                            &
                                                            (0x3f as
                                                                 libc::c_uint)
                                                                <<
                                                                0 as
                                                                    libc::c_uint)
                } else {
                    /* In Case the PLL Source is HSE (External Clock) */
                    vcoinput =
                        (8000000 as libc::c_int as
                             libc::c_uint).wrapping_div((*((0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut RCC_TypeDef)).PLLCFGR
                                                            &
                                                            (0x3f as
                                                                 libc::c_uint)
                                                                <<
                                                                0 as
                                                                    libc::c_uint)
                }
                /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
        /* SAI_CLK(first level) = PLLSAI_VCO Output/PLLSAIQ */
                tmpreg =
                    ((*((0x40000000 as
                             libc::c_uint).wrapping_add(0x20000 as
                                                            libc::c_uint).wrapping_add(0x3800
                                                                                           as
                                                                                           libc::c_uint)
                            as *mut RCC_TypeDef)).PLLSAICFGR &
                         (0xf as libc::c_uint) << 24 as libc::c_uint) >>
                        24 as libc::c_int;
                frequency =
                    vcoinput.wrapping_mul(((*((0x40000000 as
                                                   libc::c_uint).wrapping_add(0x20000
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                                 as
                                                                                                                 libc::c_uint)
                                                  as
                                                  *mut RCC_TypeDef)).PLLSAICFGR
                                               &
                                               (0x1ff as libc::c_uint) <<
                                                   6 as libc::c_uint) >>
                                              6 as
                                                  libc::c_int).wrapping_div(tmpreg);
                /* SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ */
                tmpreg =
                    (((*((0x40000000 as
                              libc::c_uint).wrapping_add(0x20000 as
                                                             libc::c_uint).wrapping_add(0x3800
                                                                                            as
                                                                                            libc::c_uint)
                             as *mut RCC_TypeDef)).DCKCFGR1 &
                          (0x1f as libc::c_uint) << 8 as libc::c_uint) >>
                         8 as
                             libc::c_int).wrapping_add(1 as libc::c_int as
                                                           libc::c_uint);
                frequency = frequency.wrapping_div(tmpreg)
            }
            4194304 => {
                /* PLLI2S is the clock source for SAI2 */
                /* Configure the PLLI2S division factor */
        /* PLLI2S_VCO Input  = PLL_SOURCE/PLLM */
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x3800
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut RCC_TypeDef)).PLLCFGR &
                       (0x1 as libc::c_uint) << 22 as libc::c_uint ==
                       0 as libc::c_uint {
                    /* In Case the PLL Source is HSI (Internal Clock) */
                    vcoinput =
                        (16000000 as
                             libc::c_uint).wrapping_div((*((0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut RCC_TypeDef)).PLLCFGR
                                                            &
                                                            (0x3f as
                                                                 libc::c_uint)
                                                                <<
                                                                0 as
                                                                    libc::c_uint)
                } else {
                    /* In Case the PLL Source is HSE (External Clock) */
                    vcoinput =
                        (8000000 as libc::c_int as
                             libc::c_uint).wrapping_div((*((0x40000000 as
                                                                libc::c_uint).wrapping_add(0x20000
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                              as
                                                                                                                              libc::c_uint)
                                                               as
                                                               *mut RCC_TypeDef)).PLLCFGR
                                                            &
                                                            (0x3f as
                                                                 libc::c_uint)
                                                                <<
                                                                0 as
                                                                    libc::c_uint)
                }
                /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
        /* SAI_CLK(first level) = PLLI2S_VCO Output/PLLI2SQ */
                tmpreg =
                    ((*((0x40000000 as
                             libc::c_uint).wrapping_add(0x20000 as
                                                            libc::c_uint).wrapping_add(0x3800
                                                                                           as
                                                                                           libc::c_uint)
                            as *mut RCC_TypeDef)).PLLI2SCFGR &
                         (0xf as libc::c_uint) << 24 as libc::c_uint) >>
                        24 as libc::c_int;
                frequency =
                    vcoinput.wrapping_mul(((*((0x40000000 as
                                                   libc::c_uint).wrapping_add(0x20000
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                                 as
                                                                                                                 libc::c_uint)
                                                  as
                                                  *mut RCC_TypeDef)).PLLI2SCFGR
                                               &
                                               (0x1ff as libc::c_uint) <<
                                                   6 as libc::c_uint) >>
                                              6 as
                                                  libc::c_int).wrapping_div(tmpreg);
                /* SAI_CLK_x = SAI_CLK(first level)/PLLI2SDIVQ */
                tmpreg =
                    ((*((0x40000000 as
                             libc::c_uint).wrapping_add(0x20000 as
                                                            libc::c_uint).wrapping_add(0x3800
                                                                                           as
                                                                                           libc::c_uint)
                            as *mut RCC_TypeDef)).DCKCFGR1 &
                         (0x1f as libc::c_uint) <<
                             0 as
                                 libc::c_uint).wrapping_add(1 as libc::c_int
                                                                as
                                                                libc::c_uint);
                frequency = frequency.wrapping_div(tmpreg)
            }
            8388608 => {
                /* External clock is the clock source for SAI2 */
                frequency = 12288000 as libc::c_uint
            }
            _ => { }
        }
    }
    return frequency;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_RCC_MODULE_ENABLED */
/* *
  * @}
  */
/* *
  * @}
  */
