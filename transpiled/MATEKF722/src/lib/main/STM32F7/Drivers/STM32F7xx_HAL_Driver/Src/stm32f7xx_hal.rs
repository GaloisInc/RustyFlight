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
    #[no_mangle]
    fn HAL_NVIC_SetPriorityGrouping(PriorityGroup: uint32_t);
    #[no_mangle]
    fn HAL_NVIC_SetPriority(IRQn: IRQn_Type, PreemptPriority: uint32_t,
                            SubPriority: uint32_t);
    #[no_mangle]
    fn HAL_SYSTICK_Config(TicksNumb: uint32_t) -> uint32_t;
}
pub type IRQn_Type = libc::c_int;
pub const SDMMC2_IRQn: IRQn_Type = 103;
pub const LPTIM1_IRQn: IRQn_Type = 93;
pub const QUADSPI_IRQn: IRQn_Type = 92;
pub const SAI2_IRQn: IRQn_Type = 91;
pub const SAI1_IRQn: IRQn_Type = 87;
pub const SPI5_IRQn: IRQn_Type = 85;
pub const SPI4_IRQn: IRQn_Type = 84;
pub const UART8_IRQn: IRQn_Type = 83;
pub const UART7_IRQn: IRQn_Type = 82;
pub const FPU_IRQn: IRQn_Type = 81;
pub const RNG_IRQn: IRQn_Type = 80;
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
pub type __uint32_t = libc::c_uint;
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
  * @brief Debug MCU
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DBGMCU_TypeDef {
    pub IDCODE: uint32_t,
    pub CR: uint32_t,
    pub APB1FZ: uint32_t,
    pub APB2FZ: uint32_t,
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
    pub OPTCR2: uint32_t,
}
/* *
  * @brief System configuration controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SYSCFG_TypeDef {
    pub MEMRMP: uint32_t,
    pub PMC: uint32_t,
    pub EXTICR: [uint32_t; 4],
    pub RESERVED: [uint32_t; 2],
    pub CMPCR: uint32_t,
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
/* * @addtogroup HAL_Private_Variables
  * @{
  */
#[no_mangle]
pub static mut uwTick: uint32_t = 0;
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup HAL_Exported_Functions HAL Exported Functions
  * @{
  */
/* * @defgroup HAL_Exported_Functions_Group1 Initialization and de-initialization Functions 
 *  @brief    Initialization and de-initialization functions
 *
@verbatim    
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initializes the Flash interface the NVIC allocation and initial clock 
          configuration. It initializes the systick also when timeout is needed 
          and the backup domain when enabled.
      (+) de-Initializes common part of the HAL
      (+) Configure The time base source to have 1ms time base with a dedicated 
          Tick interrupt priority. 
        (++) Systick timer is used by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
        (++) Time base configuration function (HAL_InitTick ()) is called automatically 
             at the beginning of the program after reset by HAL_Init() or at any time 
             when clock is configured, by HAL_RCC_ClockConfig(). 
        (++) Source of time base is configured  to generate interrupts at regular 
             time intervals. Care must be taken if HAL_Delay() is called from a 
             peripheral ISR process, the Tick interrupt line must have higher priority 
            (numerically lower) than the peripheral interrupt. Otherwise the caller 
            ISR process will be blocked. 
       (++) functions affecting time base configurations are declared as __weak  
             to make  override possible  in case of other  implementations in user file.
@endverbatim
  * @{
  */
/* *
  * @brief  This function is used to initialize the HAL Library; it must be the first 
  *         instruction to be executed in the main program (before to call any other
  *         HAL function), it performs the following:
  *           Configure the Flash prefetch, and instruction cache through ART accelerator.
  *           Configures the SysTick to generate an interrupt each 1 millisecond,
  *           which is clocked by the HSI (at this stage, the clock is not yet
  *           configured and thus the system is running from the internal HSI at 16 MHz).
  *           Set NVIC Group Priority to 4.
  *           Calls the HAL_MspInit() callback function defined in user file 
  *           "stm32f7xx_hal_msp.c" to do the global low level hardware initialization 
  *            
  * @note   SysTick is used as time base for the HAL_Delay() function, the application
  *         need to ensure that the SysTick time base is always set to 1 millisecond
  *         to have correct HAL operation.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_Init() -> HAL_StatusTypeDef {
    /* Configure Flash prefetch and Instruction cache through ART accelerator */
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).ACR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         9 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* ART_ACCLERATOR_ENABLE */
    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriorityGrouping(0x3 as libc::c_uint);
    /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
    HAL_InitTick(0 as libc::c_uint);
    /* Init the low level hardware */
    HAL_MspInit();
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  This function de-Initializes common part of the HAL and stops the systick.
  *         This function is optional.   
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DeInit() -> HAL_StatusTypeDef {
    /* Reset of all peripherals */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).APB1RSTR as
                                    *mut uint32_t,
                                0xffffffff as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).APB1RSTR as
                                    *mut uint32_t, 0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).APB2RSTR as
                                    *mut uint32_t,
                                0xffffffff as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).APB2RSTR as
                                    *mut uint32_t, 0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).AHB1RSTR as
                                    *mut uint32_t,
                                0xffffffff as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).AHB1RSTR as
                                    *mut uint32_t, 0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).AHB2RSTR as
                                    *mut uint32_t,
                                0xffffffff as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).AHB2RSTR as
                                    *mut uint32_t, 0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).AHB3RSTR as
                                    *mut uint32_t,
                                0xffffffff as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).AHB3RSTR as
                                    *mut uint32_t, 0 as libc::c_uint);
    /* De-Init the low level hardware */
    HAL_MspDeInit();
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Initializes the MSP.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MspInit() {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes the MSP.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MspDeInit() {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_MspDeInit could be implemented in the user file
   */
}
/* *
  * @brief This function configures the source of the time base.
  *        The time source is configured  to have 1ms time base with a dedicated 
  *        Tick interrupt priority.
  * @note This function is called  automatically at the beginning of program after
  *       reset by HAL_Init() or at any time when clock is reconfigured  by HAL_RCC_ClockConfig().
  * @note In the default implementation, SysTick timer is the source of time base. 
  *       It is used to generate interrupts at regular time intervals. 
  *       Care must be taken if HAL_Delay() is called from a peripheral ISR process, 
  *       The the SysTick interrupt must have higher priority (numerically lower) 
  *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *       The function is declared as __weak  to be overwritten  in case of other
  *       implementation  in user file.
  * @param TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_InitTick(mut TickPriority: uint32_t)
 -> HAL_StatusTypeDef {
    /*Configure the SysTick to have interrupt in 1ms time basis*/
    HAL_SYSTICK_Config(SystemCoreClock.wrapping_div(1000 as libc::c_int as
                                                        libc::c_uint));
    /*Configure the SysTick IRQ priority */
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority,
                         0 as libc::c_int as uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup HAL_Exported_Functions_Group2 HAL Control functions 
 *  @brief    HAL Control functions
 *
@verbatim
 ===============================================================================
                      ##### HAL Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Provide a tick value in millisecond
      (+) Provide a blocking delay in millisecond
      (+) Suspend the time base source interrupt
      (+) Resume the time base source interrupt
      (+) Get the HAL API driver version
      (+) Get the device identifier
      (+) Get the device revision identifier
      (+) Enable/Disable Debug module during SLEEP mode
      (+) Enable/Disable Debug module during STOP mode
      (+) Enable/Disable Debug module during STANDBY mode

@endverbatim
  * @{
  */
/* *
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
 * @note This function is declared as __weak to be overwritten in case of other 
  *      implementations in user file.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_IncTick() {
    ::core::ptr::write_volatile(&mut uwTick as *mut uint32_t,
                                ::core::ptr::read_volatile::<uint32_t>(&uwTick
                                                                           as
                                                                           *const uint32_t).wrapping_add(1));
}
/* *
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other 
  *       implementations in user file.
  * @retval tick value
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GetTick() -> uint32_t { return uwTick; }
/* *
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_Delay(mut Delay: uint32_t) {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    tickstart = HAL_GetTick();
    while HAL_GetTick().wrapping_sub(tickstart) < Delay { };
}
/* *
  * @brief Suspend Tick increment.
  * @note In the default implementation , SysTick timer is the source of time base. It is
  *       used to generate interrupts at regular time intervals. Once HAL_SuspendTick()
  *       is called, the SysTick interrupt will be disabled and so Tick increment 
  *       is suspended.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SuspendTick() {
    /* Disable SysTick Interrupt */
    let ref mut fresh1 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0x10 as libc::c_ulong)
               as *mut SysTick_Type)).CTRL;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_ulong &
                                     !((1 as libc::c_ulong) <<
                                           1 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Resume Tick increment.
  * @note In the default implementation , SysTick timer is the source of time base. It is
  *       used to generate interrupts at regular time intervals. Once HAL_ResumeTick()
  *       is called, the SysTick interrupt will be enabled and so Tick increment 
  *       is resumed.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ResumeTick() {
    /* Enable SysTick Interrupt */
    let ref mut fresh2 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0x10 as libc::c_ulong)
               as *mut SysTick_Type)).CTRL;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_ulong |
                                     (1 as libc::c_ulong) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Returns the HAL revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GetHalVersion() -> uint32_t {
    return ((0x1 as libc::c_int) << 24 as libc::c_int |
                (0x2 as libc::c_int) << 16 as libc::c_int |
                (0x2 as libc::c_int) << 8 as libc::c_int | 0 as libc::c_int)
               as uint32_t;
}
/* *
  * @brief  Returns the device revision identifier.
  * @retval Device revision identifier
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GetREVID() -> uint32_t {
    return (*(0xe0042000 as libc::c_uint as *mut DBGMCU_TypeDef)).IDCODE >>
               16 as libc::c_uint;
}
/* *
  * @brief  Returns the device identifier.
  * @retval Device identifier
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_GetDEVID() -> uint32_t {
    return (*(0xe0042000 as libc::c_uint as *mut DBGMCU_TypeDef)).IDCODE &
               0xfff as libc::c_int as uint32_t;
}
/* *
  * @brief  Enable the Debug Module during SLEEP mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DBGMCU_EnableDBGSleepMode() {
    let ref mut fresh3 =
        (*(0xe0042000 as libc::c_uint as *mut DBGMCU_TypeDef)).CR;
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
  * @brief  Disable the Debug Module during SLEEP mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DBGMCU_DisableDBGSleepMode() {
    let ref mut fresh4 =
        (*(0xe0042000 as libc::c_uint as *mut DBGMCU_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Enable the Debug Module during STOP mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DBGMCU_EnableDBGStopMode() {
    let ref mut fresh5 =
        (*(0xe0042000 as libc::c_uint as *mut DBGMCU_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Disable the Debug Module during STOP mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DBGMCU_DisableDBGStopMode() {
    let ref mut fresh6 =
        (*(0xe0042000 as libc::c_uint as *mut DBGMCU_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Enable the Debug Module during STANDBY mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DBGMCU_EnableDBGStandbyMode() {
    let ref mut fresh7 =
        (*(0xe0042000 as libc::c_uint as *mut DBGMCU_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh7,
                                (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Disable the Debug Module during STANDBY mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DBGMCU_DisableDBGStandbyMode() {
    let ref mut fresh8 =
        (*(0xe0042000 as libc::c_uint as *mut DBGMCU_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh8,
                                (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           2 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Enables the I/O Compensation Cell.
  * @note   The I/O compensation cell can be used only when the device supply
  *         voltage ranges from 2.4 to 3.6 V.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_EnableCompensationCell() {
    let ref mut fresh9 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut SYSCFG_TypeDef)).CMPCR;
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Power-down the I/O Compensation Cell.
  * @note   The I/O compensation cell can be used only when the device supply
  *         voltage ranges from 2.4 to 3.6 V.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DisableCompensationCell() {
    let ref mut fresh10 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut SYSCFG_TypeDef)).CMPCR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Enables the FMC Memory Mapping Swapping.
  *   
  * @note   SDRAM is accessible at 0x60000000 
  *         and NOR/RAM is accessible at 0xC0000000   
  *
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_EnableFMCMemorySwapping() {
    let ref mut fresh11 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut SYSCFG_TypeDef)).MEMRMP;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         10 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
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
/* *
  * @brief  Disables the FMC Memory Mapping Swapping
  *   
  * @note   SDRAM is accessible at 0xC0000000 (default mapping)  
  *         and NOR/RAM is accessible at 0x60000000 (default mapping)    
  *           
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DisableFMCMemorySwapping() {
    let ref mut fresh12 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut SYSCFG_TypeDef)).MEMRMP;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3 as libc::c_uint) <<
                                           10 as libc::c_uint)) as uint32_t as
                                    uint32_t);
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
/* *
  * @}
  */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
