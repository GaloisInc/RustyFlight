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
    /* !< AHB prescalers table values */
    #[no_mangle]
    static APBPrescTable: [uint8_t; 8];
    #[no_mangle]
    fn HAL_GetTick() -> uint32_t;
    #[no_mangle]
    fn HAL_InitTick(TickPriority: uint32_t) -> HAL_StatusTypeDef;
    /* * 
  * @brief  GPIO Bit SET and Bit RESET enumeration 
  */
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup GPIO_Exported_Constants GPIO Exported Constants
  * @{
  */
    /* * @defgroup GPIO_pins_define GPIO pins define
  * @{
  */
    /* Pin 0 selected    */
    /* Pin 1 selected    */
    /* Pin 2 selected    */
    /* Pin 3 selected    */
    /* Pin 4 selected    */
    /* Pin 5 selected    */
    /* Pin 6 selected    */
    /* Pin 7 selected    */
    /* Pin 8 selected    */
    /* Pin 9 selected    */
    /* Pin 10 selected   */
    /* Pin 11 selected   */
    /* Pin 12 selected   */
    /* Pin 13 selected   */
    /* Pin 14 selected   */
    /* Pin 15 selected   */
    /* All pins selected */
    /* PIN mask for assert test */
    /* *
  * @}
  */
    /* * @defgroup GPIO_mode_define GPIO mode define
  * @brief GPIO Configuration Mode 
  *        Elements values convention: 0xX0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection 
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @{
  */
    /* !< Input Floating Mode                   */
    /* !< Output Push Pull Mode                 */
    /* !< Output Open Drain Mode                */
    /* !< Alternate Function Push Pull Mode     */
    /* !< Alternate Function Open Drain Mode    */
    /* !< Analog Mode  */
    /* !< External Interrupt Mode with Rising edge trigger detection          */
    /* !< External Interrupt Mode with Falling edge trigger detection         */
    /* !< External Interrupt Mode with Rising/Falling edge trigger detection  */
    /* !< External Event Mode with Rising edge trigger detection               */
    /* !< External Event Mode with Falling edge trigger detection              */
    /* !< External Event Mode with Rising/Falling edge trigger detection       */
    /* *
  * @}
  */
    /* * @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */
    /* !< Low speed     */
    /* !< Medium speed  */
    /* !< Fast speed    */
    /* !< High speed    */
    /* *
  * @}
  */
    /* * @defgroup GPIO_pull_define GPIO pull define
   * @brief GPIO Pull-Up or Pull-Down Activation
   * @{
   */
    /* !< No Pull-up or Pull-down activation  */
    /* !< Pull-up activation                  */
    /* !< Pull-down activation                */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Macros GPIO Exported Macros
  * @{
  */
    /* *
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  __EXTI_LINE__: specifies the EXTI line flag to check.
  *         This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
    /* *
  * @brief  Clears the EXTI's line pending flags.
  * @param  __EXTI_LINE__: specifies the EXTI lines flags to clear.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
    /* *
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
    /* *
  * @brief  Clears the EXTI's line pending bits.
  * @param  __EXTI_LINE__: specifies the EXTI lines to clear.
  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
    /* *
  * @brief  Generates a Software interrupt on selected EXTI line.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval None
  */
    /* *
  * @}
  */
    /* Include GPIO HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup GPIO_Exported_Functions
  * @{
  */
    /* * @addtogroup GPIO_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
    #[no_mangle]
    fn HAL_GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                     GPIO_Init: *mut GPIO_InitTypeDef);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint32_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
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
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
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
  ******************************************************************************
  * @file    stm32f7xx_hal_rcc_ex.h
  * @author  MCD Application Team                                                                                                     
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of RCC HAL Extension module.
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
/* * @addtogroup RCCEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup RCCEx_Exported_Types RCCEx Exported Types
  * @{
  */
/* * 
  * @brief  RCC PLL configuration structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_PLLInitTypeDef {
    pub PLLState: uint32_t,
    pub PLLSource: uint32_t,
    pub PLLM: uint32_t,
    pub PLLN: uint32_t,
    pub PLLP: uint32_t,
    pub PLLQ: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_OscInitTypeDef {
    pub OscillatorType: uint32_t,
    pub HSEState: uint32_t,
    pub LSEState: uint32_t,
    pub HSIState: uint32_t,
    pub HSICalibrationValue: uint32_t,
    pub LSIState: uint32_t,
    pub PLL: RCC_PLLInitTypeDef,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_ClkInitTypeDef {
    pub ClockType: uint32_t,
    pub SYSCLKSource: uint32_t,
    pub AHBCLKDivider: uint32_t,
    pub APB1CLKDivider: uint32_t,
    pub APB2CLKDivider: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub Pin: uint32_t,
    pub Mode: uint32_t,
    pub Pull: uint32_t,
    pub Speed: uint32_t,
    pub Alternate: uint32_t,
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
/* *
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* * @defgroup RCC_Private_Variables RCC Private Variables
  * @{
  */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/* * @defgroup RCC_Exported_Functions RCC Exported Functions
  * @{
  */
/* * @defgroup RCC_Exported_Functions_Group1 Initialization and de-initialization functions 
  *  @brief    Initialization and Configuration functions 
  *
  @verbatim    
  ===============================================================================
##### Initialization and de-initialization functions #####
  ===============================================================================
    [..]
      This section provides functions allowing to configure the internal/external oscillators
      (HSE, HSI, LSE, LSI, PLL, CSS and MCO) and the System buses clocks (SYSCLK, AHB, APB1 
      and APB2).

    [..] Internal/external clock and PLL configuration
      (#) HSI (high-speed internal), 16 MHz factory-trimmed RC used directly or through
          the PLL as System clock source.

      (#) LSI (low-speed internal), 32 KHz low consumption RC used as IWDG and/or RTC
          clock source.

      (#) HSE (high-speed external), 4 to 26 MHz crystal oscillator used directly or
          through the PLL as System clock source. Can be used also as RTC clock source.

      (#) LSE (low-speed external), 32 KHz oscillator used as RTC clock source.   

      (#) PLL (clocked by HSI or HSE), featuring two different output clocks:
        (++) The first output is used to generate the high speed system clock (up to 216 MHz)
        (++) The second output is used to generate the clock for the USB OTG FS (48 MHz),
             the random analog generator (<=48 MHz) and the SDIO (<= 48 MHz).

      (#) CSS (Clock security system), once enable using the function HAL_RCC_EnableCSS()
          and if a HSE clock failure occurs(HSE used directly or through PLL as System 
          clock source), the System clock is automatically switched to HSI and an interrupt
          is generated if enabled. The interrupt is linked to the Cortex-M7 NMI 
          (Non-Maskable Interrupt) exception vector.   

      (#) MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or PLL
          clock (through a configurable prescaler) on PA8 pin.

      (#) MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or PLLI2S
          clock (through a configurable prescaler) on PC9 pin.

    [..] System, AHB and APB busses clocks configuration  
      (#) Several clock sources can be used to drive the System clock (SYSCLK): HSI,
          HSE and PLL.
          The AHB clock (HCLK) is derived from System clock through configurable 
          prescaler and used to clock the CPU, memory and peripherals mapped 
          on AHB bus (DMA, GPIO...). APB1 (PCLK1) and APB2 (PCLK2) clocks are derived 
          from AHB clock through configurable prescalers and used to clock 
          the peripherals mapped on these busses. You can use 
          "HAL_RCC_GetSysClockFreq()" function to retrieve the frequencies of these clocks.  

      -@- All the peripheral clocks are derived from the System clock (SYSCLK) except:
          (+@) I2S: the I2S clock can be derived either from a specific PLL (PLLI2S) or
              from an external clock mapped on the I2S_CKIN pin. 
              You have to use __HAL_RCC_PLLI2S_CONFIG() macro to configure this clock.
          (+@)  SAI: the SAI clock can be derived either from a specific PLL (PLLI2S) or (PLLSAI) or
              from an external clock mapped on the I2S_CKIN pin. 
               You have to use __HAL_RCC_PLLI2S_CONFIG() macro to configure this clock. 
          (+@) RTC: the RTC clock can be derived either from the LSI, LSE or HSE clock
              divided by 2 to 31. You have to use __HAL_RCC_RTC_CONFIG() and __HAL_RCC_RTC_ENABLE()
              macros to configure this clock. 
          (+@) USB OTG FS, SDIO and RTC: USB OTG FS require a frequency equal to 48 MHz
              to work correctly, while the SDIO require a frequency equal or lower than
              to 48. This clock is derived of the main PLL through PLLQ divider.
          (+@) IWDG clock which is always the LSI clock.
@endverbatim
  * @{
  */
/* *
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE, PLL and PLLI2S OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 and MCO2 OFF
  *            - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks  
  *            - LSI, LSE and RTC clocks 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_DeInit() {
    /* Set HSION bit */
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
                                     ((0x1 as libc::c_uint) <<
                                          0 as libc::c_uint |
                                          (0x10 as libc::c_uint) <<
                                              3 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Reset CFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Reset HSEON, CSSON, PLLON, PLLI2S */
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
                                           16 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               19 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               24 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               26 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    /* Reset PLLCFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).PLLCFGR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    let ref mut fresh2 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).PLLCFGR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x10 as libc::c_uint) <<
                                          0 as libc::c_uint |
                                          (0x40 as libc::c_uint) <<
                                              6 as libc::c_uint |
                                          (0x80 as libc::c_uint) <<
                                              6 as libc::c_uint |
                                          (0x4 as libc::c_uint) <<
                                              24 as libc::c_uint |
                                          0x20000000 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    /* Reset PLLI2SCFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).PLLI2SCFGR
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    let ref mut fresh3 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).PLLI2SCFGR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x40 as libc::c_uint) <<
                                          6 as libc::c_uint |
                                          (0x80 as libc::c_uint) <<
                                              6 as libc::c_uint |
                                          (0x2 as libc::c_uint) <<
                                              28 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Reset HSEBYP bit */
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
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           18 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable all interrupts */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CIR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Update the SystemCoreClock global variable */
    SystemCoreClock = 16000000 as libc::c_uint;
}
/* *
  * @brief  Initializes the RCC Oscillators according to the specified parameters in the
  *         RCC_OscInitTypeDef.
  * @param  RCC_OscInitStruct: pointer to an RCC_OscInitTypeDef structure that
  *         contains the configuration information for the RCC Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not
  *         supported by this function. User should request a transition to LSE Off
  *         first and then LSE On or LSE Bypass.
  * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
  *         supported by this function. User should request a transition to HSE Off
  *         first and then HSE On or HSE Bypass.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_OscConfig(mut RCC_OscInitStruct:
                                               *mut RCC_OscInitTypeDef)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pwrclkchanged: FlagStatus = RESET;
    /* Check the parameters */
    /*------------------------------- HSE Configuration ------------------------*/
    if (*RCC_OscInitStruct).OscillatorType & 0x1 as libc::c_uint ==
           0x1 as libc::c_uint {
        /* Check the parameters */
        /* When the HSE is used as system clock or clock source for PLL, It can not be disabled */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR &
               (0x3 as libc::c_uint) << 2 as libc::c_uint ==
               0x4 as libc::c_uint ||
               (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x3800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut RCC_TypeDef)).CFGR &
                   (0x3 as libc::c_uint) << 2 as libc::c_uint ==
                   0x8 as libc::c_uint &&
                   (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x3800
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut RCC_TypeDef)).PLLCFGR &
                       (0x1 as libc::c_uint) << 22 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 22 as libc::c_uint {
            if (if (if 0x31 as libc::c_uint as uint8_t as libc::c_int >>
                           5 as libc::c_int == 1 as libc::c_int {
                        (*((0x40000000 as
                                libc::c_uint).wrapping_add(0x20000 as
                                                               libc::c_uint).wrapping_add(0x3800
                                                                                              as
                                                                                              libc::c_uint)
                               as *mut RCC_TypeDef)).CR
                    } else {
                        (if 0x31 as libc::c_uint as uint8_t as libc::c_int >>
                                5 as libc::c_int == 2 as libc::c_int {
                             (*((0x40000000 as
                                     libc::c_uint).wrapping_add(0x20000 as
                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut RCC_TypeDef)).BDCR
                         } else {
                             (if 0x31 as libc::c_uint as uint8_t as
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
                           (0x31 as libc::c_uint as uint8_t as libc::c_int &
                                0x1f as libc::c_int as uint8_t as libc::c_int)
                       != 0 as libc::c_int as libc::c_uint {
                    1 as libc::c_int
                } else { 0 as libc::c_int }) != RESET as libc::c_int &&
                   (*RCC_OscInitStruct).HSEState == 0 as libc::c_uint {
                return HAL_ERROR
            }
        } else {
            /* Set the new HSE configuration ---------------------------------------*/
            if (*RCC_OscInitStruct).HSEState ==
                   (0x1 as libc::c_uint) << 16 as libc::c_uint {
                let ref mut fresh5 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh5,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     16 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else if (*RCC_OscInitStruct).HSEState == 0 as libc::c_uint {
                let ref mut fresh6 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh6,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       16 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                let ref mut fresh7 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh7,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       18 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            } else if (*RCC_OscInitStruct).HSEState ==
                          (0x1 as libc::c_uint) << 18 as libc::c_uint |
                              (0x1 as libc::c_uint) << 16 as libc::c_uint {
                let ref mut fresh8 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh8,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     18 as libc::c_uint) as
                                                uint32_t as uint32_t);
                let ref mut fresh9 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh9,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     16 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
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
                                                       16 as libc::c_uint)) as
                                                uint32_t as uint32_t);
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
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       18 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
            /* Check the HSE State */
            if (*RCC_OscInitStruct).HSEState != 0 as libc::c_uint {
                /* Get Start Tick*/
                tickstart = HAL_GetTick();
                /* Wait till HSE is ready */
                while (if (if 0x31 as libc::c_uint as uint8_t as libc::c_int
                                  >> 5 as libc::c_int == 1 as libc::c_int {
                               (*((0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x3800
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut RCC_TypeDef)).CR
                           } else {
                               (if 0x31 as libc::c_uint as uint8_t as
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
                                    (if 0x31 as libc::c_uint as uint8_t as
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
                                  (0x31 as libc::c_uint as uint8_t as
                                       libc::c_int &
                                       0x1f as libc::c_int as uint8_t as
                                           libc::c_int) !=
                              0 as libc::c_int as libc::c_uint {
                           1 as libc::c_int
                       } else { 0 as libc::c_int }) == RESET as libc::c_int {
                    if HAL_GetTick().wrapping_sub(tickstart) >
                           100 as libc::c_uint {
                        return HAL_TIMEOUT
                    }
                }
            } else {
                /* Get Start Tick*/
                tickstart = HAL_GetTick();
                /* Wait till HSE is bypassed or disabled */
                while (if (if 0x31 as libc::c_uint as uint8_t as libc::c_int
                                  >> 5 as libc::c_int == 1 as libc::c_int {
                               (*((0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x3800
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut RCC_TypeDef)).CR
                           } else {
                               (if 0x31 as libc::c_uint as uint8_t as
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
                                    (if 0x31 as libc::c_uint as uint8_t as
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
                                  (0x31 as libc::c_uint as uint8_t as
                                       libc::c_int &
                                       0x1f as libc::c_int as uint8_t as
                                           libc::c_int) !=
                              0 as libc::c_int as libc::c_uint {
                           1 as libc::c_int
                       } else { 0 as libc::c_int }) != RESET as libc::c_int {
                    if HAL_GetTick().wrapping_sub(tickstart) >
                           100 as libc::c_uint {
                        return HAL_TIMEOUT
                    }
                }
            }
        }
    }
    /*----------------------------- HSI Configuration --------------------------*/
    if (*RCC_OscInitStruct).OscillatorType & 0x2 as libc::c_uint ==
           0x2 as libc::c_uint {
        /* Check the parameters */
        /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR &
               (0x3 as libc::c_uint) << 2 as libc::c_uint == 0 as libc::c_uint
               ||
               (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x3800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut RCC_TypeDef)).CFGR &
                   (0x3 as libc::c_uint) << 2 as libc::c_uint ==
                   0x8 as libc::c_uint &&
                   (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x3800
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut RCC_TypeDef)).PLLCFGR &
                       (0x1 as libc::c_uint) << 22 as libc::c_uint ==
                       0 as libc::c_uint {
            /* When HSI is used as system clock it will not disabled */
            if (if (if 0x21 as libc::c_uint as uint8_t as libc::c_int >>
                           5 as libc::c_int == 1 as libc::c_int {
                        (*((0x40000000 as
                                libc::c_uint).wrapping_add(0x20000 as
                                                               libc::c_uint).wrapping_add(0x3800
                                                                                              as
                                                                                              libc::c_uint)
                               as *mut RCC_TypeDef)).CR
                    } else {
                        (if 0x21 as libc::c_uint as uint8_t as libc::c_int >>
                                5 as libc::c_int == 2 as libc::c_int {
                             (*((0x40000000 as
                                     libc::c_uint).wrapping_add(0x20000 as
                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut RCC_TypeDef)).BDCR
                         } else {
                             (if 0x21 as libc::c_uint as uint8_t as
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
                           (0x21 as libc::c_uint as uint8_t as libc::c_int &
                                0x1f as libc::c_int as uint8_t as libc::c_int)
                       != 0 as libc::c_int as libc::c_uint {
                    1 as libc::c_int
                } else { 0 as libc::c_int }) != RESET as libc::c_int &&
                   (*RCC_OscInitStruct).HSIState !=
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                return HAL_ERROR
            } else {
                /* Otherwise, just the calibration is allowed */
                /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x3800
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut RCC_TypeDef)).CR
                                                as *mut uint32_t,
                                            (*((0x40000000 as
                                                    libc::c_uint).wrapping_add(0x20000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x3800
                                                                                                                  as
                                                                                                                  libc::c_uint)
                                                   as *mut RCC_TypeDef)).CR &
                                                !((0x1f as libc::c_uint) <<
                                                      3 as libc::c_uint) |
                                                (*RCC_OscInitStruct).HSICalibrationValue
                                                    <<
                                                    __RBIT((0x1f as
                                                                libc::c_uint)
                                                               <<
                                                               3 as
                                                                   libc::c_uint).leading_zeros()
                                                        as i32 as uint8_t as
                                                        libc::c_int)
            }
        } else if (*RCC_OscInitStruct).HSIState != 0 as libc::c_uint {
            /* Check the HSI State */
            /* Enable the Internal High Speed oscillator (HSI). */
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
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Get Start Tick*/
            tickstart = HAL_GetTick();
            /* Wait till HSI is ready */
            while (if (if 0x21 as libc::c_uint as uint8_t as libc::c_int >>
                              5 as libc::c_int == 1 as libc::c_int {
                           (*((0x40000000 as
                                   libc::c_uint).wrapping_add(0x20000 as
                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                 as
                                                                                                 libc::c_uint)
                                  as *mut RCC_TypeDef)).CR
                       } else {
                           (if 0x21 as libc::c_uint as uint8_t as libc::c_int
                                   >> 5 as libc::c_int == 2 as libc::c_int {
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).BDCR
                            } else {
                                (if 0x21 as libc::c_uint as uint8_t as
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
                              (0x21 as libc::c_uint as uint8_t as libc::c_int
                                   &
                                   0x1f as libc::c_int as uint8_t as
                                       libc::c_int) !=
                          0 as libc::c_int as libc::c_uint {
                       1 as libc::c_int
                   } else { 0 as libc::c_int }) == RESET as libc::c_int {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       2 as libc::c_int as uint32_t {
                    return HAL_TIMEOUT
                }
            }
            /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as *mut RCC_TypeDef)).CR
                                            as *mut uint32_t,
                                        (*((0x40000000 as
                                                libc::c_uint).wrapping_add(0x20000
                                                                               as
                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                              as
                                                                                                              libc::c_uint)
                                               as *mut RCC_TypeDef)).CR &
                                            !((0x1f as libc::c_uint) <<
                                                  3 as libc::c_uint) |
                                            (*RCC_OscInitStruct).HSICalibrationValue
                                                <<
                                                __RBIT((0x1f as libc::c_uint)
                                                           <<
                                                           3 as
                                                               libc::c_uint).leading_zeros()
                                                    as i32 as uint8_t as
                                                    libc::c_int)
        } else {
            /* Disable the Internal High Speed oscillator (HSI). */
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
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Get Start Tick*/
            tickstart = HAL_GetTick();
            /* Wait till HSI is ready */
            while (if (if 0x21 as libc::c_uint as uint8_t as libc::c_int >>
                              5 as libc::c_int == 1 as libc::c_int {
                           (*((0x40000000 as
                                   libc::c_uint).wrapping_add(0x20000 as
                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                 as
                                                                                                 libc::c_uint)
                                  as *mut RCC_TypeDef)).CR
                       } else {
                           (if 0x21 as libc::c_uint as uint8_t as libc::c_int
                                   >> 5 as libc::c_int == 2 as libc::c_int {
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).BDCR
                            } else {
                                (if 0x21 as libc::c_uint as uint8_t as
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
                              (0x21 as libc::c_uint as uint8_t as libc::c_int
                                   &
                                   0x1f as libc::c_int as uint8_t as
                                       libc::c_int) !=
                          0 as libc::c_int as libc::c_uint {
                       1 as libc::c_int
                   } else { 0 as libc::c_int }) != RESET as libc::c_int {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       2 as libc::c_int as uint32_t {
                    return HAL_TIMEOUT
                }
            }
        }
    }
    /*------------------------------ LSI Configuration -------------------------*/
    if (*RCC_OscInitStruct).OscillatorType & 0x8 as libc::c_uint ==
           0x8 as libc::c_uint {
        /* Check the parameters */
        /* Check the LSI State */
        if (*RCC_OscInitStruct).LSIState != 0 as libc::c_uint {
            /* Enable the Internal Low Speed oscillator (LSI). */
            let ref mut fresh14 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).CSR;
            ::core::ptr::write_volatile(fresh14,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Get Start Tick*/
            tickstart = HAL_GetTick();
            /* Wait till LSI is ready */
            while (if (if 0x61 as libc::c_uint as uint8_t as libc::c_int >>
                              5 as libc::c_int == 1 as libc::c_int {
                           (*((0x40000000 as
                                   libc::c_uint).wrapping_add(0x20000 as
                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                 as
                                                                                                 libc::c_uint)
                                  as *mut RCC_TypeDef)).CR
                       } else {
                           (if 0x61 as libc::c_uint as uint8_t as libc::c_int
                                   >> 5 as libc::c_int == 2 as libc::c_int {
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).BDCR
                            } else {
                                (if 0x61 as libc::c_uint as uint8_t as
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
                              (0x61 as libc::c_uint as uint8_t as libc::c_int
                                   &
                                   0x1f as libc::c_int as uint8_t as
                                       libc::c_int) !=
                          0 as libc::c_int as libc::c_uint {
                       1 as libc::c_int
                   } else { 0 as libc::c_int }) == RESET as libc::c_int {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       2 as libc::c_int as uint32_t {
                    return HAL_TIMEOUT
                }
            }
        } else {
            /* Disable the Internal Low Speed oscillator (LSI). */
            let ref mut fresh15 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).CSR;
            ::core::ptr::write_volatile(fresh15,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Get Start Tick*/
            tickstart = HAL_GetTick();
            /* Wait till LSI is ready */
            while (if (if 0x61 as libc::c_uint as uint8_t as libc::c_int >>
                              5 as libc::c_int == 1 as libc::c_int {
                           (*((0x40000000 as
                                   libc::c_uint).wrapping_add(0x20000 as
                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                 as
                                                                                                 libc::c_uint)
                                  as *mut RCC_TypeDef)).CR
                       } else {
                           (if 0x61 as libc::c_uint as uint8_t as libc::c_int
                                   >> 5 as libc::c_int == 2 as libc::c_int {
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).BDCR
                            } else {
                                (if 0x61 as libc::c_uint as uint8_t as
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
                              (0x61 as libc::c_uint as uint8_t as libc::c_int
                                   &
                                   0x1f as libc::c_int as uint8_t as
                                       libc::c_int) !=
                          0 as libc::c_int as libc::c_uint {
                       1 as libc::c_int
                   } else { 0 as libc::c_int }) != RESET as libc::c_int {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       2 as libc::c_int as uint32_t {
                    return HAL_TIMEOUT
                }
            }
        }
    }
    /*------------------------------ LSE Configuration -------------------------*/
    if (*RCC_OscInitStruct).OscillatorType & 0x4 as libc::c_uint ==
           0x4 as libc::c_uint {
        /* Check the parameters */
        /* Update LSE configuration in Backup Domain control register    */
    /* Requires to enable write access to Backup Domain of necessary */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).APB1ENR &
               (0x1 as libc::c_uint) << 28 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Enable Power Clock*/
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
                                                 28 as libc::c_uint) as
                                            uint32_t as uint32_t);
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
            pwrclkchanged = SET
        }
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                  *mut PWR_TypeDef)).CR1 &
               (0x1 as libc::c_uint) << 8 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Enable write access to Backup domain */
            let ref mut fresh17 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x7000 as libc::c_uint) as
                       *mut PWR_TypeDef)).CR1;
            ::core::ptr::write_volatile(fresh17,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 8 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Wait for Backup domain Write protection disable */
            tickstart = HAL_GetTick();
            while (*((0x40000000 as
                          libc::c_uint).wrapping_add(0x7000 as libc::c_uint)
                         as *mut PWR_TypeDef)).CR1 &
                      (0x1 as libc::c_uint) << 8 as libc::c_uint ==
                      RESET as libc::c_int as libc::c_uint {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       100 as libc::c_int as uint32_t {
                    return HAL_TIMEOUT
                }
            }
        }
        if (*RCC_OscInitStruct).LSEState ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            let ref mut fresh18 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh18,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else if (*RCC_OscInitStruct).LSEState == 0 as libc::c_uint {
            let ref mut fresh19 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh19,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh20 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh20,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else if (*RCC_OscInitStruct).LSEState ==
                      (0x1 as libc::c_uint) << 2 as libc::c_uint |
                          (0x1 as libc::c_uint) << 0 as libc::c_uint {
            let ref mut fresh21 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh21,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 2 as libc::c_uint) as
                                            uint32_t as uint32_t);
            let ref mut fresh22 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh22,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            let ref mut fresh23 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh23,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh24 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).BDCR;
            ::core::ptr::write_volatile(fresh24,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        if (*RCC_OscInitStruct).LSEState != 0 as libc::c_uint {
            /* Set the new LSE configuration -----------------------------------------*/
            /* Get Start Tick*/
            tickstart = HAL_GetTick();
            /* Wait till LSE is ready */
            while (if (if 0x41 as libc::c_uint as uint8_t as libc::c_int >>
                              5 as libc::c_int == 1 as libc::c_int {
                           (*((0x40000000 as
                                   libc::c_uint).wrapping_add(0x20000 as
                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                 as
                                                                                                 libc::c_uint)
                                  as *mut RCC_TypeDef)).CR
                       } else {
                           (if 0x41 as libc::c_uint as uint8_t as libc::c_int
                                   >> 5 as libc::c_int == 2 as libc::c_int {
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
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
                              (0x41 as libc::c_uint as uint8_t as libc::c_int
                                   &
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
        } else {
            /* Get Start Tick*/
            tickstart = HAL_GetTick();
            /* Wait till LSE is ready */
            while (if (if 0x41 as libc::c_uint as uint8_t as libc::c_int >>
                              5 as libc::c_int == 1 as libc::c_int {
                           (*((0x40000000 as
                                   libc::c_uint).wrapping_add(0x20000 as
                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                 as
                                                                                                 libc::c_uint)
                                  as *mut RCC_TypeDef)).CR
                       } else {
                           (if 0x41 as libc::c_uint as uint8_t as libc::c_int
                                   >> 5 as libc::c_int == 2 as libc::c_int {
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
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
                              (0x41 as libc::c_uint as uint8_t as libc::c_int
                                   &
                                   0x1f as libc::c_int as uint8_t as
                                       libc::c_int) !=
                          0 as libc::c_int as libc::c_uint {
                       1 as libc::c_int
                   } else { 0 as libc::c_int }) != RESET as libc::c_int {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       5000 as libc::c_uint {
                    return HAL_TIMEOUT
                }
            }
        }
        if pwrclkchanged as libc::c_uint == SET as libc::c_int as libc::c_uint
           {
            let ref mut fresh25 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).APB1ENR;
            ::core::ptr::write_volatile(fresh25,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh25
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   28 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Check the LSE State */
    /* Restore clock configuration if changed */
    /*-------------------------------- PLL Configuration -----------------------*/
  /* Check the parameters */
    if (*RCC_OscInitStruct).PLL.PLLState != 0 as libc::c_uint {
        /* Check if the PLL is used as system clock or not */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR &
               (0x3 as libc::c_uint) << 2 as libc::c_uint !=
               0x8 as libc::c_uint {
            if (*RCC_OscInitStruct).PLL.PLLState == 0x2 as libc::c_uint {
                /* Check the parameters */
                /* Disable the main PLL. */
                let ref mut fresh26 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh26,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh26
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       24 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                tickstart = HAL_GetTick();
                while (if (if 0x39 as libc::c_uint as uint8_t as libc::c_int
                                  >> 5 as libc::c_int == 1 as libc::c_int {
                               (*((0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x3800
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut RCC_TypeDef)).CR
                           } else {
                               (if 0x39 as libc::c_uint as uint8_t as
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
                                    (if 0x39 as libc::c_uint as uint8_t as
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
                                  (0x39 as libc::c_uint as uint8_t as
                                       libc::c_int &
                                       0x1f as libc::c_int as uint8_t as
                                           libc::c_int) !=
                              0 as libc::c_int as libc::c_uint {
                           1 as libc::c_int
                       } else { 0 as libc::c_int }) != RESET as libc::c_int
                      /* Get Start Tick*/
                      /* Wait till PLL is ready */
                      {
                    if HAL_GetTick().wrapping_sub(tickstart) >
                           2 as libc::c_int as uint32_t {
                        return HAL_TIMEOUT
                    }
                }
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x3800
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut RCC_TypeDef)).PLLCFGR
                                                as *mut uint32_t,
                                            0x20000000 as libc::c_int as
                                                libc::c_uint |
                                                (*RCC_OscInitStruct).PLL.PLLSource
                                                |
                                                (*RCC_OscInitStruct).PLL.PLLM
                                                |
                                                (*RCC_OscInitStruct).PLL.PLLN
                                                    <<
                                                    __RBIT((0x1ff as
                                                                libc::c_uint)
                                                               <<
                                                               6 as
                                                                   libc::c_uint).leading_zeros()
                                                        as i32 as uint8_t as
                                                        libc::c_int |
                                                ((*RCC_OscInitStruct).PLL.PLLP
                                                     >>
                                                     1 as
                                                         libc::c_int).wrapping_sub(1
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       libc::c_uint)
                                                    <<
                                                    __RBIT((0x3 as
                                                                libc::c_uint)
                                                               <<
                                                               16 as
                                                                   libc::c_uint).leading_zeros()
                                                        as i32 as uint8_t as
                                                        libc::c_int |
                                                (*RCC_OscInitStruct).PLL.PLLQ
                                                    <<
                                                    __RBIT((0xf as
                                                                libc::c_uint)
                                                               <<
                                                               24 as
                                                                   libc::c_uint).leading_zeros()
                                                        as i32 as uint8_t as
                                                        libc::c_int);
                let ref mut fresh27 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh27,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh27
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     24 as libc::c_uint) as
                                                uint32_t as uint32_t);
                tickstart = HAL_GetTick();
                while (if (if 0x39 as libc::c_uint as uint8_t as libc::c_int
                                  >> 5 as libc::c_int == 1 as libc::c_int {
                               (*((0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x3800
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut RCC_TypeDef)).CR
                           } else {
                               (if 0x39 as libc::c_uint as uint8_t as
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
                                    (if 0x39 as libc::c_uint as uint8_t as
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
                                  (0x39 as libc::c_uint as uint8_t as
                                       libc::c_int &
                                       0x1f as libc::c_int as uint8_t as
                                           libc::c_int) !=
                              0 as libc::c_int as libc::c_uint {
                           1 as libc::c_int
                       } else { 0 as libc::c_int }) == RESET as libc::c_int
                      /* Configure the main PLL clock source, multiplication and division factors. */
                      /* Enable the main PLL. */
                      /* Get Start Tick*/
                      /* Wait till PLL is ready */
                      {
                    if HAL_GetTick().wrapping_sub(tickstart) >
                           2 as libc::c_int as uint32_t {
                        return HAL_TIMEOUT
                    }
                }
            } else {
                /* Disable the main PLL. */
                let ref mut fresh28 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3800
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut RCC_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh28,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh28
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       24 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                /* Get Start Tick*/
                tickstart = HAL_GetTick();
                /* Wait till PLL is ready */
                while (if (if 0x39 as libc::c_uint as uint8_t as libc::c_int
                                  >> 5 as libc::c_int == 1 as libc::c_int {
                               (*((0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x3800
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut RCC_TypeDef)).CR
                           } else {
                               (if 0x39 as libc::c_uint as uint8_t as
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
                                    (if 0x39 as libc::c_uint as uint8_t as
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
                                  (0x39 as libc::c_uint as uint8_t as
                                       libc::c_int &
                                       0x1f as libc::c_int as uint8_t as
                                           libc::c_int) !=
                              0 as libc::c_int as libc::c_uint {
                           1 as libc::c_int
                       } else { 0 as libc::c_int }) != RESET as libc::c_int {
                    if HAL_GetTick().wrapping_sub(tickstart) >
                           2 as libc::c_int as uint32_t {
                        return HAL_TIMEOUT
                    }
                }
            }
        } else { return HAL_ERROR }
    }
    return HAL_OK;
}
/* *
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified 
  *         parameters in the RCC_ClkInitStruct.
  * @param  RCC_ClkInitStruct: pointer to an RCC_OscInitTypeDef structure that
  *         contains the configuration information for the RCC peripheral.
  * @param  FLatency: FLASH Latency, this parameter depend on device selected
  * 
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency 
  *         and updated by HAL_RCC_GetHCLKFreq() function called within this function
  *
  * @note   The HSI is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  *           
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked). 
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready. 
  *         You can use HAL_RCC_GetClockConfig() function to know which clock is
  *         currently used as system clock source.
  * @note   Depending on the device voltage range, the software has to set correctly
  *         HPRE[3:0] bits to ensure that HCLK not exceed the maximum allowed frequency
  *         (for more details refer to section above "Initialization/de-initialization functions")
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_ClockConfig(mut RCC_ClkInitStruct:
                                                 *mut RCC_ClkInitTypeDef,
                                             mut FLatency: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* To correctly read data from FLASH memory, the number of wait states (LATENCY) 
  must be correctly programmed according to the frequency of the CPU clock 
  (HCLK) and the supply voltage of the device. */
    /* Increasing the CPU frequency */
    if FLatency >
           (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).ACR &
               (0xf as libc::c_uint) << 0 as libc::c_uint {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).ACR as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut FLASH_TypeDef)).ACR &
                                        !((0xf as libc::c_uint) <<
                                              0 as libc::c_uint) | FLatency);
        /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).ACR &
               (0xf as libc::c_uint) << 0 as libc::c_uint != FLatency {
            return HAL_ERROR
        }
    }
    /*-------------------------- HCLK Configuration --------------------------*/
    if (*RCC_ClkInitStruct).ClockType & 0x2 as libc::c_uint ==
           0x2 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).CFGR as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).CFGR &
                                        !((0xf as libc::c_uint) <<
                                              4 as libc::c_uint) |
                                        (*RCC_ClkInitStruct).AHBCLKDivider)
    }
    /*------------------------- SYSCLK Configuration ---------------------------*/
    if (*RCC_ClkInitStruct).ClockType & 0x1 as libc::c_uint ==
           0x1 as libc::c_uint {
        /* HSE is selected as System Clock Source */
        if (*RCC_ClkInitStruct).SYSCLKSource == 0x1 as libc::c_uint {
            /* Check the HSE ready flag */
            if (if (if 0x31 as libc::c_uint as uint8_t as libc::c_int >>
                           5 as libc::c_int == 1 as libc::c_int {
                        (*((0x40000000 as
                                libc::c_uint).wrapping_add(0x20000 as
                                                               libc::c_uint).wrapping_add(0x3800
                                                                                              as
                                                                                              libc::c_uint)
                               as *mut RCC_TypeDef)).CR
                    } else {
                        (if 0x31 as libc::c_uint as uint8_t as libc::c_int >>
                                5 as libc::c_int == 2 as libc::c_int {
                             (*((0x40000000 as
                                     libc::c_uint).wrapping_add(0x20000 as
                                                                    libc::c_uint).wrapping_add(0x3800
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut RCC_TypeDef)).BDCR
                         } else {
                             (if 0x31 as libc::c_uint as uint8_t as
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
                           (0x31 as libc::c_uint as uint8_t as libc::c_int &
                                0x1f as libc::c_int as uint8_t as libc::c_int)
                       != 0 as libc::c_int as libc::c_uint {
                    1 as libc::c_int
                } else { 0 as libc::c_int }) == RESET as libc::c_int {
                return HAL_ERROR
            }
        } else if (*RCC_ClkInitStruct).SYSCLKSource == 0x2 as libc::c_uint {
            /* PLL is selected as System Clock Source */
            /* Check the PLL ready flag */
            if (if (if 0x39 as libc::c_uint as uint8_t as libc::c_int >>
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
                             (if 0x39 as libc::c_uint as uint8_t as
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
                           (0x39 as libc::c_uint as uint8_t as libc::c_int &
                                0x1f as libc::c_int as uint8_t as libc::c_int)
                       != 0 as libc::c_int as libc::c_uint {
                    1 as libc::c_int
                } else { 0 as libc::c_int }) == RESET as libc::c_int {
                return HAL_ERROR
            }
        } else if (if (if 0x21 as libc::c_uint as uint8_t as libc::c_int >>
                              5 as libc::c_int == 1 as libc::c_int {
                           (*((0x40000000 as
                                   libc::c_uint).wrapping_add(0x20000 as
                                                                  libc::c_uint).wrapping_add(0x3800
                                                                                                 as
                                                                                                 libc::c_uint)
                                  as *mut RCC_TypeDef)).CR
                       } else {
                           (if 0x21 as libc::c_uint as uint8_t as libc::c_int
                                   >> 5 as libc::c_int == 2 as libc::c_int {
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).BDCR
                            } else {
                                (if 0x21 as libc::c_uint as uint8_t as
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
                              (0x21 as libc::c_uint as uint8_t as libc::c_int
                                   &
                                   0x1f as libc::c_int as uint8_t as
                                       libc::c_int) !=
                          0 as libc::c_int as libc::c_uint {
                       1 as libc::c_int
                   } else { 0 as libc::c_int }) == RESET as libc::c_int {
            return HAL_ERROR
        }
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).CFGR as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).CFGR &
                                        !((0x3 as libc::c_uint) <<
                                              0 as libc::c_uint) |
                                        (*RCC_ClkInitStruct).SYSCLKSource);
        /* HSI is selected as System Clock Source */
        /* Check the HSI ready flag */
        /* Get Start Tick*/
        tickstart = HAL_GetTick();
        if (*RCC_ClkInitStruct).SYSCLKSource == 0x1 as libc::c_uint {
            while (*((0x40000000 as
                          libc::c_uint).wrapping_add(0x20000 as
                                                         libc::c_uint).wrapping_add(0x3800
                                                                                        as
                                                                                        libc::c_uint)
                         as *mut RCC_TypeDef)).CFGR &
                      (0x3 as libc::c_uint) << 2 as libc::c_uint !=
                      0x4 as libc::c_uint {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       5000 as libc::c_int as uint32_t {
                    return HAL_TIMEOUT
                }
            }
        } else if (*RCC_ClkInitStruct).SYSCLKSource == 0x2 as libc::c_uint {
            while (*((0x40000000 as
                          libc::c_uint).wrapping_add(0x20000 as
                                                         libc::c_uint).wrapping_add(0x3800
                                                                                        as
                                                                                        libc::c_uint)
                         as *mut RCC_TypeDef)).CFGR &
                      (0x3 as libc::c_uint) << 2 as libc::c_uint !=
                      0x8 as libc::c_uint {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       5000 as libc::c_int as uint32_t {
                    return HAL_TIMEOUT
                }
            }
        } else {
            while (*((0x40000000 as
                          libc::c_uint).wrapping_add(0x20000 as
                                                         libc::c_uint).wrapping_add(0x3800
                                                                                        as
                                                                                        libc::c_uint)
                         as *mut RCC_TypeDef)).CFGR &
                      (0x3 as libc::c_uint) << 2 as libc::c_uint !=
                      0 as libc::c_uint {
                if HAL_GetTick().wrapping_sub(tickstart) >
                       5000 as libc::c_int as uint32_t {
                    return HAL_TIMEOUT
                }
            }
        }
    }
    /* Decreasing the number of wait states because of lower CPU frequency */
    if FLatency <
           (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).ACR &
               (0xf as libc::c_uint) << 0 as libc::c_uint {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).ACR as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut FLASH_TypeDef)).ACR &
                                        !((0xf as libc::c_uint) <<
                                              0 as libc::c_uint) | FLatency);
        /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).ACR &
               (0xf as libc::c_uint) << 0 as libc::c_uint != FLatency {
            return HAL_ERROR
        }
    }
    /*-------------------------- PCLK1 Configuration ---------------------------*/
    if (*RCC_ClkInitStruct).ClockType & 0x4 as libc::c_uint ==
           0x4 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).CFGR as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).CFGR &
                                        !((0x7 as libc::c_uint) <<
                                              10 as libc::c_uint) |
                                        (*RCC_ClkInitStruct).APB1CLKDivider)
    }
    /*-------------------------- PCLK2 Configuration ---------------------------*/
    if (*RCC_ClkInitStruct).ClockType & 0x8 as libc::c_uint ==
           0x8 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).CFGR as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).CFGR &
                                        !((0x7 as libc::c_uint) <<
                                              13 as libc::c_uint) |
                                        (*RCC_ClkInitStruct).APB2CLKDivider <<
                                            3 as libc::c_int)
    }
    /* Update the SystemCoreClock global variable */
    SystemCoreClock =
        HAL_RCC_GetSysClockFreq() >>
            AHBPrescTable[(((*((0x40000000 as
                                    libc::c_uint).wrapping_add(0x20000 as
                                                                   libc::c_uint).wrapping_add(0x3800
                                                                                                  as
                                                                                                  libc::c_uint)
                                   as *mut RCC_TypeDef)).CFGR &
                                (0xf as libc::c_uint) << 4 as libc::c_uint) >>
                               __RBIT((0xf as libc::c_uint) <<
                                          4 as libc::c_uint).leading_zeros()
                                   as i32 as uint8_t as libc::c_int) as usize]
                as libc::c_int;
    /* Configure the source of time base considering new system clocks settings*/
    HAL_InitTick(0 as libc::c_uint);
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup RCC_Exported_Functions_Group2 Peripheral Control functions 
  *  @brief   RCC clocks control functions 
  *
  @verbatim   
  ===============================================================================
                  ##### Peripheral Control functions #####
  ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the RCC Clocks 
    frequencies.
      
@endverbatim
  * @{
  */
/* *
  * @brief  Selects the clock source to output on MCO1 pin(PA8) or on MCO2 pin(PC9).
  * @note   PA8/PC9 should be configured in alternate function mode.
  * @param  RCC_MCOx: specifies the output direction for the clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO1: Clock source to output on MCO1 pin(PA8).
  *            @arg RCC_MCO2: Clock source to output on MCO2 pin(PC9).
  * @param  RCC_MCOSource: specifies the clock source to output.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO1SOURCE_HSI: HSI clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_LSE: LSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_HSE: HSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_PLLCLK: main PLL clock selected as MCO1 source
  *            @arg RCC_MCO2SOURCE_SYSCLK: System clock (SYSCLK) selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLI2SCLK: PLLI2S clock selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_HSE: HSE clock selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLCLK: main PLL clock selected as MCO2 source
  * @param  RCC_MCODiv: specifies the MCOx prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_MCOConfig(mut RCC_MCOx: uint32_t,
                                           mut RCC_MCOSource: uint32_t,
                                           mut RCC_MCODiv: uint32_t) {
    let mut GPIO_InitStruct: GPIO_InitTypeDef =
        GPIO_InitTypeDef{Pin: 0, Mode: 0, Pull: 0, Speed: 0, Alternate: 0,};
    /* Check the parameters */
    /* RCC_MCO1 */
    if RCC_MCOx == 0 as libc::c_uint {
        /* MCO1 Clock Enable */
        let mut tmpreg: uint32_t = 0;
        let ref mut fresh29 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh29,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh29
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint);
        /* Configure the MCO1 pin in alternate function mode */
        GPIO_InitStruct.Pin = 0x100 as libc::c_uint as uint16_t as uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Speed = 0x3 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Alternate = 0 as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        /* Mask MCO1 and MCO1PRE[2:0] bits then Select MCO1 clock source and prescaler */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).CFGR as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).CFGR &
                                        !((0x3 as libc::c_uint) <<
                                              21 as libc::c_uint |
                                              (0x7 as libc::c_uint) <<
                                                  24 as libc::c_uint) |
                                        (RCC_MCOSource | RCC_MCODiv))
    } else {
        /* MCO2 Clock Enable */
        let mut tmpreg_0: uint32_t = 0;
        let ref mut fresh30 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh30,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh30
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             2 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            2 as libc::c_uint);
        /* Configure the MCO2 pin in alternate function mode */
        GPIO_InitStruct.Pin = 0x200 as libc::c_uint as uint16_t as uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Speed = 0x3 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Alternate = 0 as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x800
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        /* Mask MCO2 and MCO2PRE[2:0] bits then Select MCO2 clock source and prescaler */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3800
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut RCC_TypeDef)).CFGR as
                                        *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).CFGR &
                                        !((0x3 as libc::c_uint) <<
                                              30 as libc::c_uint |
                                              (0x7 as libc::c_uint) <<
                                                  27 as libc::c_uint) |
                                        (RCC_MCOSource |
                                             RCC_MCODiv << 3 as libc::c_int))
    };
}
/* *
  * @brief  Enables the Clock Security System.
  * @note   If a failure is detected on the HSE oscillator clock, this oscillator
  *         is automatically disabled and an interrupt is generated to inform the
  *         software about the failure (Clock Security System Interrupt, CSSI),
  *         allowing the MCU to perform rescue operations. The CSSI is linked to 
  *         the Cortex-M7 NMI (Non-Maskable Interrupt) exception vector.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_EnableCSS() {
    let ref mut fresh31 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh31,
                                (::core::ptr::read_volatile::<uint32_t>(fresh31
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         19 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Disables the Clock Security System.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_DisableCSS() {
    let ref mut fresh32 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh32,
                                (::core::ptr::read_volatile::<uint32_t>(fresh32
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           19 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Returns the SYSCLK frequency
  *        
  * @note   The system frequency computed by this function is not the real 
  *         frequency in the chip. It is calculated based on the predefined 
  *         constant and the selected clock source:
  * @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
  * @note     If SYSCLK source is HSE, function returns values based on HSE_VALUE(**)
  * @note     If SYSCLK source is PLL, function returns values based on HSE_VALUE(**) 
  *           or HSI_VALUE(*) multiplied/divided by the PLL factors.         
  * @note     (*) HSI_VALUE is a constant defined in stm32f7xx_hal_conf.h file (default value
  *               16 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) HSE_VALUE is a constant defined in stm32f7xx_hal_conf.h file (default value
  *                25 MHz), user has to ensure that HSE_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  *                  
  * @note   The result of this function could be not correct when using fractional
  *         value for HSE crystal.
  *           
  * @note   This function can be used by the user application to compute the 
  *         baudrate for the communication peripherals or configure other parameters.
  *           
  * @note   Each time SYSCLK changes, this function must be called to update the
  *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
  *         
  *               
  * @retval SYSCLK frequency
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_GetSysClockFreq() -> uint32_t {
    let mut pllm: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllvco: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut sysclockfreq: uint32_t = 0 as libc::c_int as uint32_t;
    /* Get SYSCLK source -------------------------------------------------------*/
    match (*((0x40000000 as
                  libc::c_uint).wrapping_add(0x20000 as
                                                 libc::c_uint).wrapping_add(0x3800
                                                                                as
                                                                                libc::c_uint)
                 as *mut RCC_TypeDef)).CFGR &
              (0x3 as libc::c_uint) << 2 as libc::c_uint {
        0 => {
            /* HSI used as system clock source */
            sysclockfreq = 16000000 as libc::c_uint
        }
        4 => {
            /* HSE used as system clock  source */
            sysclockfreq = 8000000 as libc::c_int as uint32_t
        }
        8 => {
            /* PLL used as system clock  source */
            /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
      SYSCLK = PLL_VCO / PLLP */
            pllm =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).PLLCFGR &
                    (0x3f as libc::c_uint) << 0 as libc::c_uint;
            if (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x3800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut RCC_TypeDef)).PLLCFGR &
                   (0x1 as libc::c_uint) << 22 as libc::c_uint !=
                   0 as libc::c_uint {
                /* HSE used as PLL clock source */
                pllvco =
                    (8000000 as libc::c_int as
                         libc::c_uint).wrapping_div(pllm).wrapping_mul(((*((0x40000000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x20000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                                               as
                                                                               *mut RCC_TypeDef)).PLLCFGR
                                                                            &
                                                                            (0x1ff
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                6
                                                                                    as
                                                                                    libc::c_uint)
                                                                           >>
                                                                           __RBIT((0x1ff
                                                                                       as
                                                                                       libc::c_uint)
                                                                                      <<
                                                                                      6
                                                                                          as
                                                                                          libc::c_uint).leading_zeros()
                                                                               as
                                                                               i32
                                                                               as
                                                                               uint8_t
                                                                               as
                                                                               libc::c_int)
            } else {
                /* HSI used as PLL clock source */
                pllvco =
                    (16000000 as
                         libc::c_uint).wrapping_div(pllm).wrapping_mul(((*((0x40000000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x20000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                                               as
                                                                               *mut RCC_TypeDef)).PLLCFGR
                                                                            &
                                                                            (0x1ff
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                6
                                                                                    as
                                                                                    libc::c_uint)
                                                                           >>
                                                                           __RBIT((0x1ff
                                                                                       as
                                                                                       libc::c_uint)
                                                                                      <<
                                                                                      6
                                                                                          as
                                                                                          libc::c_uint).leading_zeros()
                                                                               as
                                                                               i32
                                                                               as
                                                                               uint8_t
                                                                               as
                                                                               libc::c_int)
            }
            pllp =
                (((*((0x40000000 as
                          libc::c_uint).wrapping_add(0x20000 as
                                                         libc::c_uint).wrapping_add(0x3800
                                                                                        as
                                                                                        libc::c_uint)
                         as *mut RCC_TypeDef)).PLLCFGR &
                      (0x3 as libc::c_uint) << 16 as libc::c_uint) >>
                     __RBIT((0x3 as libc::c_uint) <<
                                16 as libc::c_uint).leading_zeros() as i32 as
                         uint8_t as
                         libc::c_int).wrapping_add(1 as libc::c_int as
                                                       libc::c_uint).wrapping_mul(2
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint);
            sysclockfreq = pllvco.wrapping_div(pllp)
        }
        _ => { sysclockfreq = 16000000 as libc::c_uint }
    }
    return sysclockfreq;
}
/* *
  * @brief  Returns the HCLK frequency     
  * @note   Each time HCLK changes, this function must be called to update the
  *         right HCLK value. Otherwise, any configuration based on this function will be incorrect. 
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency.
  * @retval HCLK frequency
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_GetHCLKFreq() -> uint32_t {
    return SystemCoreClock;
}
/* *
  * @brief  Returns the PCLK1 frequency     
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_GetPCLK1Freq() -> uint32_t {
    /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
    return HAL_RCC_GetHCLKFreq() >>
               APBPrescTable[(((*((0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x3800
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut RCC_TypeDef)).CFGR &
                                   (0x7 as libc::c_uint) <<
                                       10 as libc::c_uint) >>
                                  __RBIT((0x7 as libc::c_uint) <<
                                             10 as
                                                 libc::c_uint).leading_zeros()
                                      as i32 as uint8_t as libc::c_int) as
                                 usize] as libc::c_int;
}
/* *
  * @brief  Returns the PCLK2 frequency     
  * @note   Each time PCLK2 changes, this function must be called to update the
  *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK2 frequency
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_GetPCLK2Freq() -> uint32_t {
    /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
    return HAL_RCC_GetHCLKFreq() >>
               APBPrescTable[(((*((0x40000000 as
                                       libc::c_uint).wrapping_add(0x20000 as
                                                                      libc::c_uint).wrapping_add(0x3800
                                                                                                     as
                                                                                                     libc::c_uint)
                                      as *mut RCC_TypeDef)).CFGR &
                                   (0x7 as libc::c_uint) <<
                                       13 as libc::c_uint) >>
                                  __RBIT((0x7 as libc::c_uint) <<
                                             13 as
                                                 libc::c_uint).leading_zeros()
                                      as i32 as uint8_t as libc::c_int) as
                                 usize] as libc::c_int;
}
/* *
  * @brief  Configures the RCC_OscInitStruct according to the internal 
  * RCC configuration registers.
  * @param  RCC_OscInitStruct: pointer to an RCC_OscInitTypeDef structure that 
  * will be configured.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_GetOscConfig(mut RCC_OscInitStruct:
                                                  *mut RCC_OscInitTypeDef) {
    /* Set all possible values for the Oscillator type parameter ---------------*/
    (*RCC_OscInitStruct).OscillatorType =
        0x1 as libc::c_uint | 0x2 as libc::c_uint | 0x4 as libc::c_uint |
            0x8 as libc::c_uint;
    /* Get the HSE configuration -----------------------------------------------*/
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3800
                                                                             as
                                                                             libc::c_uint)
              as *mut RCC_TypeDef)).CR &
           (0x1 as libc::c_uint) << 18 as libc::c_uint ==
           (0x1 as libc::c_uint) << 18 as libc::c_uint {
        (*RCC_OscInitStruct).HSEState =
            (0x1 as libc::c_uint) << 18 as libc::c_uint |
                (0x1 as libc::c_uint) << 16 as libc::c_uint
    } else if (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).CR &
                  (0x1 as libc::c_uint) << 16 as libc::c_uint ==
                  (0x1 as libc::c_uint) << 16 as libc::c_uint {
        (*RCC_OscInitStruct).HSEState =
            (0x1 as libc::c_uint) << 16 as libc::c_uint
    } else { (*RCC_OscInitStruct).HSEState = 0 as libc::c_uint }
    /* Get the HSI configuration -----------------------------------------------*/
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3800
                                                                             as
                                                                             libc::c_uint)
              as *mut RCC_TypeDef)).CR &
           (0x1 as libc::c_uint) << 0 as libc::c_uint ==
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        (*RCC_OscInitStruct).HSIState =
            (0x1 as libc::c_uint) << 0 as libc::c_uint
    } else { (*RCC_OscInitStruct).HSIState = 0 as libc::c_uint }
    (*RCC_OscInitStruct).HSICalibrationValue =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).CR &
             (0x1f as libc::c_uint) << 3 as libc::c_uint) >>
            __RBIT((0x1f as libc::c_uint) <<
                       3 as libc::c_uint).leading_zeros() as i32 as uint8_t as
                libc::c_int;
    /* Get the LSE configuration -----------------------------------------------*/
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3800
                                                                             as
                                                                             libc::c_uint)
              as *mut RCC_TypeDef)).BDCR &
           (0x1 as libc::c_uint) << 2 as libc::c_uint ==
           (0x1 as libc::c_uint) << 2 as libc::c_uint {
        (*RCC_OscInitStruct).LSEState =
            (0x1 as libc::c_uint) << 2 as libc::c_uint |
                (0x1 as libc::c_uint) << 0 as libc::c_uint
    } else if (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).BDCR &
                  (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                  (0x1 as libc::c_uint) << 0 as libc::c_uint {
        (*RCC_OscInitStruct).LSEState =
            (0x1 as libc::c_uint) << 0 as libc::c_uint
    } else { (*RCC_OscInitStruct).LSEState = 0 as libc::c_uint }
    /* Get the LSI configuration -----------------------------------------------*/
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3800
                                                                             as
                                                                             libc::c_uint)
              as *mut RCC_TypeDef)).CSR &
           (0x1 as libc::c_uint) << 0 as libc::c_uint ==
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        (*RCC_OscInitStruct).LSIState =
            (0x1 as libc::c_uint) << 0 as libc::c_uint
    } else { (*RCC_OscInitStruct).LSIState = 0 as libc::c_uint }
    /* Get the PLL configuration -----------------------------------------------*/
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3800
                                                                             as
                                                                             libc::c_uint)
              as *mut RCC_TypeDef)).CR &
           (0x1 as libc::c_uint) << 24 as libc::c_uint ==
           (0x1 as libc::c_uint) << 24 as libc::c_uint {
        (*RCC_OscInitStruct).PLL.PLLState = 0x2 as libc::c_uint
    } else { (*RCC_OscInitStruct).PLL.PLLState = 0x1 as libc::c_uint }
    (*RCC_OscInitStruct).PLL.PLLSource =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).PLLCFGR &
            (0x1 as libc::c_uint) << 22 as libc::c_uint;
    (*RCC_OscInitStruct).PLL.PLLM =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).PLLCFGR &
            (0x3f as libc::c_uint) << 0 as libc::c_uint;
    (*RCC_OscInitStruct).PLL.PLLN =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLCFGR &
             (0x1ff as libc::c_uint) << 6 as libc::c_uint) >>
            __RBIT((0x1ff as libc::c_uint) <<
                       6 as libc::c_uint).leading_zeros() as i32 as uint8_t as
                libc::c_int;
    (*RCC_OscInitStruct).PLL.PLLP =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLCFGR &
             (0x3 as libc::c_uint) <<
                 16 as
                     libc::c_uint).wrapping_add((0x1 as libc::c_uint) <<
                                                    16 as libc::c_uint) <<
            1 as libc::c_int >>
            __RBIT((0x3 as libc::c_uint) <<
                       16 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
    (*RCC_OscInitStruct).PLL.PLLQ =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).PLLCFGR &
             (0xf as libc::c_uint) << 24 as libc::c_uint) >>
            __RBIT((0xf as libc::c_uint) <<
                       24 as libc::c_uint).leading_zeros() as i32 as uint8_t
                as libc::c_int;
}
/* *
  * @brief  Configures the RCC_ClkInitStruct according to the internal 
  * RCC configuration registers.
  * @param  RCC_ClkInitStruct: pointer to an RCC_ClkInitTypeDef structure that 
  * will be configured.
  * @param  pFLatency: Pointer on the Flash Latency.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_GetClockConfig(mut RCC_ClkInitStruct:
                                                    *mut RCC_ClkInitTypeDef,
                                                mut pFLatency:
                                                    *mut uint32_t) {
    /* Set all possible values for the Clock type parameter --------------------*/
    (*RCC_ClkInitStruct).ClockType =
        0x1 as libc::c_uint | 0x2 as libc::c_uint | 0x4 as libc::c_uint |
            0x8 as libc::c_uint;
    /* Get the SYSCLK configuration --------------------------------------------*/
    (*RCC_ClkInitStruct).SYSCLKSource =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CFGR &
            (0x3 as libc::c_uint) << 0 as libc::c_uint;
    /* Get the HCLK configuration ----------------------------------------------*/
    (*RCC_ClkInitStruct).AHBCLKDivider =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CFGR &
            (0xf as libc::c_uint) << 4 as libc::c_uint;
    /* Get the APB1 configuration ----------------------------------------------*/
    (*RCC_ClkInitStruct).APB1CLKDivider =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CFGR &
            (0x7 as libc::c_uint) << 10 as libc::c_uint;
    /* Get the APB2 configuration ----------------------------------------------*/
    (*RCC_ClkInitStruct).APB2CLKDivider =
        ((*((0x40000000 as
                 libc::c_uint).wrapping_add(0x20000 as
                                                libc::c_uint).wrapping_add(0x3800
                                                                               as
                                                                               libc::c_uint)
                as *mut RCC_TypeDef)).CFGR &
             (0x7 as libc::c_uint) << 13 as libc::c_uint) >> 3 as libc::c_int;
    /* Get the Flash Wait State (Latency) configuration ------------------------*/
    *pFLatency =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).ACR &
            (0xf as libc::c_uint) << 0 as libc::c_uint;
}
/* *
  * @brief This function handles the RCC CSS interrupt request.
  * @note This API should be called under the NMI_Handler().
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_NMI_IRQHandler() {
    /* Check RCC CSSF flag  */
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3800
                                                                             as
                                                                             libc::c_uint)
              as *mut RCC_TypeDef)).CIR &
           0x80 as libc::c_uint as uint8_t as libc::c_uint ==
           0x80 as libc::c_uint as uint8_t as libc::c_uint {
        /* RCC Clock Security System interrupt user callback */
        HAL_RCC_CSSCallback();
        /* Clear RCC CSS pending bit */
        ::core::ptr::write_volatile((0x40000000 as
                                         libc::c_uint).wrapping_add(0x20000 as
                                                                        libc::c_uint).wrapping_add(0x3800
                                                                                                       as
                                                                                                       libc::c_uint).wrapping_add(0xc
                                                                                                                                      as
                                                                                                                                      libc::c_int
                                                                                                                                      as
                                                                                                                                      libc::c_uint).wrapping_add(0x2
                                                                                                                                                                     as
                                                                                                                                                                     libc::c_int
                                                                                                                                                                     as
                                                                                                                                                                     libc::c_uint)
                                        as *mut uint8_t,
                                    0x80 as libc::c_uint as uint8_t)
    };
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_rcc.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of RCC HAL module.
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
/* Include RCC HAL Extended module */
/* (include on top of file since RCC structures are defined in extended file) */
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup RCC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup RCC_Exported_Types RCC Exported Types
  * @{
  */
/* *
  * @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition  
  */
/* !< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */
/* !< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */
/* !< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */
/* !< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */
/* !< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */
/* !< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */
/* !< PLL structure parameters                                                    */
/* *
  * @brief  RCC System, AHB and APB busses clock configuration structure definition  
  */
/* !< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type */
/* !< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source    */
/* !< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source       */
/* !< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
/* !< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup RCC_Exported_Constants RCC Exported Constants
  * @{
  */
/* * @defgroup RCC_Oscillator_Type Oscillator Type
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_HSE_Config RCC HSE Config
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_LSE_Config RCC LSE Config
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_HSI_Config RCC HSI Config
  * @{
  */
/* Default HSI calibration trimming value */
/* *
  * @}
  */
/* * @defgroup RCC_LSI_Config RCC LSI Config
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_PLL_Config RCC PLL Config
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_PLLP_Clock_Divider PLLP Clock Divider
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_PLL_Clock_Source PLL Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_System_Clock_Type RCC System Clock Type
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_System_Clock_Source RCC System Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_System_Clock_Source_Status System Clock Source Status
  * @{
  */
/* !< HSI used as system clock */
/* !< HSE used as system clock */
/* !< PLL used as system clock */
/* *
  * @}
  */
/* * @defgroup RCC_AHB_Clock_Source RCC AHB Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB1_APB2_Clock_Source RCC APB1/APB2 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_RTC_Clock_Source RCC RTC Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_MCO_Index RCC MCO Index
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_MCO1_Clock_Source RCC MCO1 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_MCO2_Clock_Source RCC MCO2 Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_MCOx_Clock_Prescaler RCC MCO1 Clock Prescaler
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Interrupt RCC Interrupt 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Flag RCC Flags
  *        Elements values convention: 0XXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - 0XX  : Register index
  *                 - 01: CR register
  *                 - 10: BDCR register
  *                 - 11: CSR register
  * @{
  */
/* Flags in the CR register */
/* Flags in the BDCR register */
/* Flags in the CSR register */
/* *
  * @}
  */
/* * @defgroup RCC_LSEDrive_Configuration RCC LSE Drive configurations
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup RCC_Exported_Macros RCC Exported Macros
  * @{
  */
/* * @defgroup RCC_AHB1_Clock_Enable_Disable AHB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.   
  * @{
  */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* *
  * @}
  */
/* * @defgroup RCC_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/* Delay after an RCC peripheral clock enabling */
/* Delay after an RCC peripheral clock enabling */
/* *
  * @}
  */
/* * @defgroup RCC_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable                                      
  * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{
  */
/* Delay after an RCC peripheral clock enabling */
/* *
  * @}
  */
/* * @defgroup RCC_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB1_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable  Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB2_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  EGet the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Peripheral_Clock_Force_Release RCC Peripheral Clock Force Release
  * @brief  Force or release AHB peripheral reset.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB1_Force_Release_Reset APB1 Force Release Reset 
  * @brief  Force or release APB1 peripheral reset.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB2_Force_Release_Reset APB2 Force Release Reset 
  * @brief  Force or release APB2 peripheral reset.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Peripheral_Clock_Sleep_Enable_Disable RCC Peripheral Clock Sleep Enable Disable
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/* * @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* * @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
/* *
  * @}
  */
/* * @defgroup RCC_AHB1_Clock_Sleep_Enable_Disable_Status AHB1 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB1_Clock_Sleep_Enable_Disable_Status APB1 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB2_Clock_Sleep_Enable_Disable_Status APB2 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_HSI_Configuration HSI Configuration
  * @{   
  */
/* * @brief  Macros to enable or disable the Internal High Speed oscillator (HSI).
  * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
  *         It is used (enabled by hardware) as system clock source after startup
  *         from Reset, wakeup from STOP and STANDBY mode, or in case of failure
  *         of the HSE used directly or indirectly as system clock (if the Clock
  *         Security System CSS is enabled).             
  * @note   HSI can not be stopped if it is used as system clock source. In this case,
  *         you have to select another source of the system clock then stop the HSI.  
  * @note   After enabling the HSI, the application software should wait on HSIRDY
  *         flag to be set indicating that HSI clock is stable and can be used as
  *         system clock source.  
  * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
  *         clock cycles.  
  */
/* * @brief  Macro to adjust the Internal High Speed oscillator (HSI) calibration value.
  * @note   The calibration is used to compensate for the variations in voltage
  *         and temperature that influence the frequency of the internal HSI RC.
  * @param  __HSICALIBRATIONVALUE__: specifies the calibration trimming value.
  *         (default is RCC_HSICALIBRATION_DEFAULT).
  */
/* *
  * @}
  */
/* * @defgroup RCC_LSI_Configuration LSI Configuration
  * @{   
  */
/* * @brief  Macros to enable or disable the Internal Low Speed oscillator (LSI).
  * @note   After enabling the LSI, the application software should wait on 
  *         LSIRDY flag to be set indicating that LSI clock is stable and can
  *         be used to clock the IWDG and/or the RTC.
  * @note   LSI can not be disabled if the IWDG is running.
  * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
  *         clock cycles. 
  */
/* *
  * @}
  */
/* * @defgroup RCC_HSE_Configuration HSE Configuration
  * @{   
  */ 
/* *
  * @brief  Macro to configure the External High Speed oscillator (HSE).
  * @note   Transitions HSE Bypass to HSE On and HSE On to HSE Bypass are not
  *         supported by this macro. User should request a transition to HSE Off
  *         first and then HSE On or HSE Bypass.
  * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
  *         software should wait on HSERDY flag to be set indicating that HSE clock
  *         is stable and can be used to clock the PLL and/or system clock.
  * @note   HSE state can not be changed if it is used directly or through the
  *         PLL as system clock. In this case, you have to select another source
  *         of the system clock then change the HSE state (ex. disable it).
  * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
  * @note   This function reset the CSSON bit, so if the clock security system(CSS)
  *         was previously enabled you have to enable it again after calling this
  *         function.
  * @param  __STATE__: specifies the new state of the HSE.
  *         This parameter can be one of the following values:
  *            @arg RCC_HSE_OFF: turn OFF the HSE oscillator, HSERDY flag goes low after
  *                              6 HSE oscillator clock cycles.
  *            @arg RCC_HSE_ON: turn ON the HSE oscillator.
  *            @arg RCC_HSE_BYPASS: HSE oscillator bypassed with external clock.
  */
/* *
  * @}
  */
/* * @defgroup RCC_LSE_Configuration LSE Configuration
  * @{   
  */
/* *
  * @brief  Macro to configure the External Low Speed oscillator (LSE).
  * @note   Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not supported by this macro. 
  *         User should request a transition to LSE Off first and then LSE On or LSE Bypass.  
  * @note   As the LSE is in the Backup domain and write access is denied to
  *         this domain after reset, you have to enable write access using 
  *         HAL_PWR_EnableBkUpAccess() function before to configure the LSE
  *         (to be done once after reset).  
  * @note   After enabling the LSE (RCC_LSE_ON or RCC_LSE_BYPASS), the application
  *         software should wait on LSERDY flag to be set indicating that LSE clock
  *         is stable and can be used to clock the RTC.
  * @param  __STATE__: specifies the new state of the LSE.
  *         This parameter can be one of the following values:
  *            @arg RCC_LSE_OFF: turn OFF the LSE oscillator, LSERDY flag goes low after
  *                              6 LSE oscillator clock cycles.
  *            @arg RCC_LSE_ON: turn ON the LSE oscillator.
  *            @arg RCC_LSE_BYPASS: LSE oscillator bypassed with external clock.
  */
/* *
  * @}
  */
/* * @defgroup RCC_Internal_RTC_Clock_Configuration RTC Clock Configuration
  * @{   
  */
/* * @brief  Macros to enable or disable the RTC clock.
  * @note   These macros must be used only after the RTC clock source was selected.
  */
/* * @brief  Macros to configure the RTC clock (RTCCLK).
  * @note   As the RTC clock configuration bits are in the Backup domain and write
  *         access is denied to this domain after reset, you have to enable write
  *         access using the Power Backup Access macro before to configure
  *         the RTC clock source (to be done once after reset).    
  * @note   Once the RTC clock is configured it can't be changed unless the  
  *         Backup domain is reset using __HAL_RCC_BackupReset_RELEASE() macro, or by
  *         a Power On Reset (POR).
  * @param  __RTCCLKSource__: specifies the RTC clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_RTCCLKSOURCE_LSE: LSE selected as RTC clock.
  *            @arg RCC_RTCCLKSOURCE_LSI: LSI selected as RTC clock.
  *            @arg RCC_RTCCLKSOURCE_HSE_DIVx: HSE clock divided by x selected
  *                                            as RTC clock, where x:[2,31]
  * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
  *         work in STOP and STANDBY modes, and can be used as wakeup source.
  *         However, when the HSE clock is used as RTC clock source, the RTC
  *         cannot be used in STOP and STANDBY modes.    
  * @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
  *         RTC clock source).
  */
/* * @brief  Macros to force or release the Backup domain reset.
  * @note   This function resets the RTC peripheral (including the backup registers)
  *         and the RTC clock source selection in RCC_CSR register.
  * @note   The BKPSRAM is not affected by this reset.   
  */
/* *
  * @}
  */
/* * @defgroup RCC_PLL_Configuration PLL Configuration
  * @{   
  */
/* * @brief  Macros to enable or disable the main PLL.
  * @note   After enabling the main PLL, the application software should wait on 
  *         PLLRDY flag to be set indicating that PLL clock is stable and can
  *         be used as system clock source.
  * @note   The main PLL can not be disabled if it is used as system clock source
  * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
  */
/* * @brief  Macro to configure the PLL clock source.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __PLLSOURCE__: specifies the PLL entry clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_PLLSOURCE_HSI: HSI oscillator clock selected as PLL clock entry
  *            @arg RCC_PLLSOURCE_HSE: HSE oscillator clock selected as PLL clock entry
  *      
  */
/* * @brief  Macro to configure the PLL multiplication factor.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __PLLM__: specifies the division factor for PLL VCO input clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 63.
  * @note   You have to set the PLLM parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
  *         of 2 MHz to limit PLL jitter.
  *      
  */
/* *
  * @}
  */
/* * @defgroup RCC_PLL_I2S_Configuration PLL I2S Configuration
  * @{   
  */
/* * @brief  Macro to configure the I2S clock source (I2SCLK).
  * @note   This function must be called before enabling the I2S APB clock.
  * @param  __SOURCE__: specifies the I2S clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_I2SCLKSOURCE_PLLI2S: PLLI2S clock used as I2S clock source.
  *            @arg RCC_I2SCLKSOURCE_EXT: External clock mapped on the I2S_CKIN pin
  *                                       used as I2S clock source.
  */
/* * @brief Macros to enable or disable the PLLI2S. 
  * @note  The PLLI2S is disabled by hardware when entering STOP and STANDBY modes.
  */
/* *
  * @}
  */
/* * @defgroup RCC_Get_Clock_source Get Clock source
  * @{   
  */
/* *
  * @brief Macro to configure the system clock source.
  * @param __RCC_SYSCLKSOURCE__: specifies the system clock source.
  * This parameter can be one of the following values:
  *              - RCC_SYSCLKSOURCE_HSI: HSI oscillator is used as system clock source.
  *              - RCC_SYSCLKSOURCE_HSE: HSE oscillator is used as system clock source.
  *              - RCC_SYSCLKSOURCE_PLLCLK: PLL output is used as system clock source.
  */
/* * @brief  Macro to get the clock source used as system clock.
  * @retval The clock source used as system clock. The returned value can be one
  *         of the following:
  *              - RCC_SYSCLKSOURCE_STATUS_HSI: HSI used as system clock.
  *              - RCC_SYSCLKSOURCE_STATUS_HSE: HSE used as system clock.
  *              - RCC_SYSCLKSOURCE_STATUS_PLLCLK: PLL used as system clock.
  */
/* *
  * @brief  Macro to configures the External Low Speed oscillator (LSE) drive capability.
  * @note   As the LSE is in the Backup domain and write access is denied to
  *         this domain after reset, you have to enable write access using
  *         HAL_PWR_EnableBkUpAccess() function before to configure the LSE
  *         (to be done once after reset).
  * @param  __RCC_LSEDRIVE__: specifies the new state of the LSE drive capability.
  *          This parameter can be one of the following values:
  *            @arg RCC_LSEDRIVE_LOW: LSE oscillator low drive capability.
  *            @arg RCC_LSEDRIVE_MEDIUMLOW: LSE oscillator medium low drive capability.
  *            @arg RCC_LSEDRIVE_MEDIUMHIGH: LSE oscillator medium high drive capability.
  *            @arg RCC_LSEDRIVE_HIGH: LSE oscillator high drive capability.
  * @retval None
  */
/* * @brief  Macro to get the oscillator used as PLL clock source.
  * @retval The oscillator used as PLL clock source. The returned value can be one
  *         of the following:
  *              - RCC_PLLSOURCE_HSI: HSI oscillator is used as PLL clock source.
  *              - RCC_PLLSOURCE_HSE: HSE oscillator is used as PLL clock source.
  */
/* *
  * @}
  */
/* * @defgroup RCCEx_MCOx_Clock_Config RCC Extended MCOx Clock Config
  * @{   
  */
/* * @brief  Macro to configure the MCO1 clock.
  * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO1SOURCE_HSI: HSI clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_LSE: LSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_HSE: HSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_PLLCLK: main PLL clock selected as MCO1 source
  * @param  __MCODIV__ specifies the MCO clock prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
  */
/* * @brief  Macro to configure the MCO2 clock.
  * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO2SOURCE_SYSCLK: System clock (SYSCLK) selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLI2SCLK: PLLI2S clock selected as MCO2 source 
  *            @arg RCC_MCO2SOURCE_HSE: HSE clock selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLCLK: main PLL clock selected as MCO2 source
  * @param  __MCODIV__ specifies the MCO clock prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
  */
/* *
  * @}
  */
/* * @defgroup RCC_Flags_Interrupts_Management Flags Interrupts Management
  * @brief macros to manage the specified RCC Flags and interrupts.
  * @{
  */
/* * @brief  Enable RCC interrupt (Perform Byte access to RCC_CIR[14:8] bits to enable
  *         the selected interrupts).
  * @param  __INTERRUPT__: specifies the RCC interrupt sources to be enabled.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  */
/* * @brief Disable RCC interrupt (Perform Byte access to RCC_CIR[14:8] bits to disable 
  *        the selected interrupts).
  * @param  __INTERRUPT__: specifies the RCC interrupt sources to be disabled.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  */
/* * @brief  Clear the RCC's interrupt pending bits (Perform Byte access to RCC_CIR[23:16]
  *         bits to clear the selected interrupt pending bits.
  * @param  __INTERRUPT__: specifies the interrupt pending bit to clear.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.  
  *            @arg RCC_IT_CSS: Clock Security System interrupt
  */
/* * @brief  Check the RCC's interrupt has occurred or not.
  * @param  __INTERRUPT__: specifies the RCC interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  *            @arg RCC_IT_CSS: Clock Security System interrupt
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
/* * @brief Set RMVF bit to clear the reset flags: RCC_FLAG_PINRST, RCC_FLAG_PORRST, 
  *        RCC_FLAG_SFTRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST and RCC_FLAG_LPWRRST.
  */
/* * @brief  Check RCC flag is set or not.
  * @param  __FLAG__: specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready.
  *            @arg RCC_FLAG_HSERDY: HSE oscillator clock ready.
  *            @arg RCC_FLAG_PLLRDY: Main PLL clock ready.
  *            @arg RCC_FLAG_PLLI2SRDY: PLLI2S clock ready.
  *            @arg RCC_FLAG_LSERDY: LSE oscillator clock ready.
  *            @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready.
  *            @arg RCC_FLAG_BORRST: POR/PDR or BOR reset.
  *            @arg RCC_FLAG_PINRST: Pin reset.
  *            @arg RCC_FLAG_PORRST: POR/PDR reset.
  *            @arg RCC_FLAG_SFTRST: Software reset.
  *            @arg RCC_FLAG_IWDGRST: Independent Watchdog reset.
  *            @arg RCC_FLAG_WWDGRST: Window Watchdog reset.
  *            @arg RCC_FLAG_LPWRRST: Low Power reset.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Include RCC HAL Extension module */
/* Exported functions --------------------------------------------------------*/
 /* * @addtogroup RCC_Exported_Functions
  * @{
  */
/* * @addtogroup RCC_Exported_Functions_Group1
  * @{
  */                             
/* Initialization and de-initialization functions  ******************************/
/* *
  * @}
  */
/* * @addtogroup RCC_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  ************************************************/
/* CSS NMI IRQ handler */
/* User Callbacks in non blocking mode (IT mode) */
/* *
  * @brief  RCC Clock Security System interrupt callback
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RCC_CSSCallback() {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_RCC_CSSCallback could be implemented in the user file
   */
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
