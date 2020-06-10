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
    #[no_mangle]
    fn HAL_NVIC_SetPriority(IRQn: IRQn_Type, PreemptPriority: uint32_t,
                            SubPriority: uint32_t);
    #[no_mangle]
    fn HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
    /* *
  * @}
  */
    /* * @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
    /* *
  * @}
  */
    /* Force 32bits alignment */
    /* * @defgroup USBD_CORE_Exported_Macros
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USBD_CORE_Exported_Variables
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_CORE_Exported_Functions
  * @{
  */
    #[no_mangle]
    fn USBD_CDC_ReceivePacket(pdev: *mut USBD_HandleTypeDef) -> uint8_t;
    #[no_mangle]
    fn USBD_CDC_SetRxBuffer(pdev: *mut USBD_HandleTypeDef,
                            pbuff: *mut uint8_t) -> uint8_t;
    #[no_mangle]
    fn HAL_TIM_Base_Init(htim: *mut TIM_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_Base_Start_IT(htim: *mut TIM_HandleTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_IRQHandler(htim: *mut TIM_HandleTypeDef);
    #[no_mangle]
    static mut USBD_Device: USBD_HandleTypeDef;
    #[no_mangle]
    fn USBD_CDC_TransmitPacket(pdev: *mut USBD_HandleTypeDef) -> uint8_t;
    #[no_mangle]
    fn USBD_CDC_SetTxBuffer(pdev: *mut USBD_HandleTypeDef,
                            pbuff: *mut uint8_t, length: uint16_t) -> uint8_t;
    #[no_mangle]
    fn delay(ms: timeMs_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
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
  * @brief  HAL Lock structures definition  
  */
pub type HAL_LockTypeDef = libc::c_uint;
pub const HAL_LOCKED: HAL_LockTypeDef = 1;
pub const HAL_UNLOCKED: HAL_LockTypeDef = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_dma.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DMA HAL module.
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
/* * @addtogroup DMA
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup DMA_Exported_Types DMA Exported Types
  * @brief    DMA Exported Types 
  * @{
  */
/* * 
  * @brief  DMA Configuration Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub Channel: uint32_t,
    pub Direction: uint32_t,
    pub PeriphInc: uint32_t,
    pub MemInc: uint32_t,
    pub PeriphDataAlignment: uint32_t,
    pub MemDataAlignment: uint32_t,
    pub Mode: uint32_t,
    pub Priority: uint32_t,
    pub FIFOMode: uint32_t,
    pub FIFOThreshold: uint32_t,
    pub MemBurst: uint32_t,
    pub PeriphBurst: uint32_t,
}
/* * 
  * @brief  HAL DMA State structures definition
  */
pub type HAL_DMA_StateTypeDef = libc::c_uint;
/* !< DMA Abort state                     */
/* !< DMA error state                     */
pub const HAL_DMA_STATE_ABORT: HAL_DMA_StateTypeDef = 5;
/* !< DMA timeout state                   */
pub const HAL_DMA_STATE_ERROR: HAL_DMA_StateTypeDef = 4;
/* !< DMA process is ongoing              */
pub const HAL_DMA_STATE_TIMEOUT: HAL_DMA_StateTypeDef = 3;
/* !< DMA initialized and ready for use   */
pub const HAL_DMA_STATE_BUSY: HAL_DMA_StateTypeDef = 2;
/* !< DMA not yet initialized or disabled */
pub const HAL_DMA_STATE_READY: HAL_DMA_StateTypeDef = 1;
pub const HAL_DMA_STATE_RESET: HAL_DMA_StateTypeDef = 0;
/* * 
  * @brief  DMA handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __DMA_HandleTypeDef {
    pub Instance: *mut DMA_Stream_TypeDef,
    pub Init: DMA_InitTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_DMA_StateTypeDef,
    pub Parent: *mut libc::c_void,
    pub XferCpltCallback: Option<unsafe extern "C" fn(_:
                                                          *mut __DMA_HandleTypeDef)
                                     -> ()>,
    pub XferHalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                              *mut __DMA_HandleTypeDef)
                                         -> ()>,
    pub XferM1CpltCallback: Option<unsafe extern "C" fn(_:
                                                            *mut __DMA_HandleTypeDef)
                                       -> ()>,
    pub XferM1HalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                                *mut __DMA_HandleTypeDef)
                                           -> ()>,
    pub XferErrorCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub XferAbortCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub ErrorCode: uint32_t,
    pub StreamBaseAddress: uint32_t,
    pub StreamIndex: uint32_t,
}
pub type DMA_HandleTypeDef = __DMA_HandleTypeDef;
/* !< DMA Stream Index                       */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_tim.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of TIM HAL module.
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
/* * @addtogroup TIM
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup TIM_Exported_Types TIM Exported Types
  * @{
  */
/* * 
  * @brief  TIM Time base Configuration Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_Base_InitTypeDef {
    pub Prescaler: uint32_t,
    pub CounterMode: uint32_t,
    pub Period: uint32_t,
    pub ClockDivision: uint32_t,
    pub RepetitionCounter: uint32_t,
    pub AutoReloadPreload: uint32_t,
}
/* * 
  * @brief  HAL State structures definition  
  */
pub type HAL_TIM_StateTypeDef = libc::c_uint;
/* !< Reception process is ongoing                */
/* !< Timeout state                               */
pub const HAL_TIM_STATE_ERROR: HAL_TIM_StateTypeDef = 4;
/* !< An internal process is ongoing              */
pub const HAL_TIM_STATE_TIMEOUT: HAL_TIM_StateTypeDef = 3;
/* !< Peripheral Initialized and ready for use    */
pub const HAL_TIM_STATE_BUSY: HAL_TIM_StateTypeDef = 2;
/* !< Peripheral not yet initialized or disabled  */
pub const HAL_TIM_STATE_READY: HAL_TIM_StateTypeDef = 1;
pub const HAL_TIM_STATE_RESET: HAL_TIM_StateTypeDef = 0;
/* * 
  * @brief  HAL Active channel structures definition  
  */
pub type HAL_TIM_ActiveChannel = libc::c_uint;
/* !< All active channels cleared */
/* !< The active channel is 4     */
pub const HAL_TIM_ACTIVE_CHANNEL_CLEARED: HAL_TIM_ActiveChannel = 0;
/* !< The active channel is 3     */
pub const HAL_TIM_ACTIVE_CHANNEL_4: HAL_TIM_ActiveChannel = 8;
/* !< The active channel is 2     */
pub const HAL_TIM_ACTIVE_CHANNEL_3: HAL_TIM_ActiveChannel = 4;
/* !< The active channel is 1     */
pub const HAL_TIM_ACTIVE_CHANNEL_2: HAL_TIM_ActiveChannel = 2;
pub const HAL_TIM_ACTIVE_CHANNEL_1: HAL_TIM_ActiveChannel = 1;
/* * 
  * @brief  TIM Time Base Handle Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_HandleTypeDef {
    pub Instance: *mut TIM_TypeDef,
    pub Init: TIM_Base_InitTypeDef,
    pub Channel: HAL_TIM_ActiveChannel,
    pub hdma: [*mut DMA_HandleTypeDef; 7],
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_TIM_StateTypeDef,
}
pub const USBD_OK: C2RustUnnamed = 0;
/*  EP0 State */
/* *
  * @}
  */
/* * @defgroup USBD_DEF_Exported_TypesDefinitions
  * @{
  */
/* Control Endpoints*/
/* Class Specific Endpoints*/
/* Following USB Device Speed */
/* Following USB Device status */
/* USB Device descriptors structure */
/* USB Device handle structure */
/* USB Device handle structure */
pub type USBD_HandleTypeDef = _USBD_HandleTypeDef;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USBD_HandleTypeDef {
    pub id: uint8_t,
    pub dev_config: uint32_t,
    pub dev_default_config: uint32_t,
    pub dev_config_status: uint32_t,
    pub dev_speed: USBD_SpeedTypeDef,
    pub ep_in: [USBD_EndpointTypeDef; 15],
    pub ep_out: [USBD_EndpointTypeDef; 15],
    pub ep0_state: uint32_t,
    pub ep0_data_len: uint32_t,
    pub dev_state: uint8_t,
    pub dev_old_state: uint8_t,
    pub dev_address: uint8_t,
    pub dev_connection_status: uint8_t,
    pub dev_test_mode: uint8_t,
    pub dev_remote_wakeup: uint32_t,
    pub request: USBD_SetupReqTypedef,
    pub pDesc: *mut USBD_DescriptorsTypeDef,
    pub pClass: *mut USBD_ClassTypeDef,
    pub pCDC_ClassData: *mut libc::c_void,
    pub pCDC_UserData: *mut libc::c_void,
    pub pHID_ClassData: *mut libc::c_void,
    pub pHID_UserData: *mut libc::c_void,
    pub pMSC_ClassData: *mut libc::c_void,
    pub pMSC_UserData: *mut libc::c_void,
    pub pData: *mut libc::c_void,
}
pub type USBD_ClassTypeDef = _Device_cb;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _Device_cb {
    pub Init: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                          _: uint8_t) -> uint8_t>,
    pub DeInit: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                            _: uint8_t) -> uint8_t>,
    pub Setup: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                           _: *mut USBD_SetupReqTypedef)
                          -> uint8_t>,
    pub EP0_TxSent: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                               -> uint8_t>,
    pub EP0_RxReady: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                                -> uint8_t>,
    pub DataIn: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                            _: uint8_t) -> uint8_t>,
    pub DataOut: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                             _: uint8_t) -> uint8_t>,
    pub SOF: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                        -> uint8_t>,
    pub IsoINIncomplete: Option<unsafe extern "C" fn(_:
                                                         *mut _USBD_HandleTypeDef,
                                                     _: uint8_t) -> uint8_t>,
    pub IsoOUTIncomplete: Option<unsafe extern "C" fn(_:
                                                          *mut _USBD_HandleTypeDef,
                                                      _: uint8_t) -> uint8_t>,
    pub GetHSConfigDescriptor: Option<unsafe extern "C" fn(_: *mut uint16_t)
                                          -> *mut uint8_t>,
    pub GetFSConfigDescriptor: Option<unsafe extern "C" fn(_: *mut uint16_t)
                                          -> *mut uint8_t>,
    pub GetOtherSpeedConfigDescriptor: Option<unsafe extern "C" fn(_:
                                                                       *mut uint16_t)
                                                  -> *mut uint8_t>,
    pub GetDeviceQualifierDescriptor: Option<unsafe extern "C" fn(_:
                                                                      *mut uint16_t)
                                                 -> *mut uint8_t>,
}
pub type USBD_SetupReqTypedef = usb_setup_req;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usb_setup_req {
    pub bmRequest: uint8_t,
    pub bRequest: uint8_t,
    pub wValue: uint16_t,
    pub wIndex: uint16_t,
    pub wLength: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_DescriptorsTypeDef {
    pub GetDeviceDescriptor: Option<unsafe extern "C" fn(_: USBD_SpeedTypeDef,
                                                         _: *mut uint16_t)
                                        -> *mut uint8_t>,
    pub GetLangIDStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                USBD_SpeedTypeDef,
                                                            _: *mut uint16_t)
                                           -> *mut uint8_t>,
    pub GetManufacturerStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                      USBD_SpeedTypeDef,
                                                                  _:
                                                                      *mut uint16_t)
                                                 -> *mut uint8_t>,
    pub GetProductStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                 USBD_SpeedTypeDef,
                                                             _: *mut uint16_t)
                                            -> *mut uint8_t>,
    pub GetSerialStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                USBD_SpeedTypeDef,
                                                            _: *mut uint16_t)
                                           -> *mut uint8_t>,
    pub GetConfigurationStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                       USBD_SpeedTypeDef,
                                                                   _:
                                                                       *mut uint16_t)
                                                  -> *mut uint8_t>,
    pub GetInterfaceStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                   USBD_SpeedTypeDef,
                                                               _:
                                                                   *mut uint16_t)
                                              -> *mut uint8_t>,
}
pub type USBD_SpeedTypeDef = libc::c_uint;
pub const USBD_SPEED_LOW: USBD_SpeedTypeDef = 2;
pub const USBD_SPEED_FULL: USBD_SpeedTypeDef = 1;
pub const USBD_SPEED_HIGH: USBD_SpeedTypeDef = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_EndpointTypeDef {
    pub status: uint32_t,
    pub total_length: uint32_t,
    pub rem_length: uint32_t,
    pub maxpacket: uint32_t,
}
pub type C2RustUnnamed = libc::c_uint;
pub const USBD_FAIL: C2RustUnnamed = 2;
pub const USBD_BUSY: C2RustUnnamed = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_CDC_LineCodingTypeDef {
    pub bitrate: uint32_t,
    pub format: uint8_t,
    pub paritytype: uint8_t,
    pub datatype: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USBD_CDC_Itf {
    pub Init: Option<unsafe extern "C" fn() -> int8_t>,
    pub DeInit: Option<unsafe extern "C" fn() -> int8_t>,
    pub Control: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                             _: uint16_t) -> int8_t>,
    pub Receive: Option<unsafe extern "C" fn(_: *mut uint8_t,
                                             _: *mut uint32_t) -> int8_t>,
}
pub type USBD_CDC_ItfTypeDef = _USBD_CDC_Itf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_CDC_HandleTypeDef {
    pub data: [uint32_t; 128],
    pub CmdOpCode: uint8_t,
    pub CmdLength: uint8_t,
    pub RxBuffer: *mut uint8_t,
    pub TxBuffer: *mut uint8_t,
    pub RxLength: uint32_t,
    pub TxLength: uint32_t,
    pub TxState: uint32_t,
    pub RxState: uint32_t,
}
/* This is stupid, any nice solution to handle multiple interfaces
   * would be much apriciated. Or at least a flow how this should be rewritten instead.
   */
// millisecond time
pub type timeMs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct LINE_CODING {
    pub bitrate: uint32_t,
    pub format: uint8_t,
    pub paritytype: uint8_t,
    pub datatype: uint8_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#[no_mangle]
pub static mut LineCoding: USBD_CDC_LineCodingTypeDef =
    {
        let mut init =
            USBD_CDC_LineCodingTypeDef{bitrate:
                                           115200 as libc::c_int as uint32_t,
                                       format: 0 as libc::c_int as uint8_t,
                                       paritytype:
                                           0 as libc::c_int as uint8_t,
                                       datatype:
                                           0x8 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub static mut UserRxBuffer: [uint8_t; 2048] = [0; 2048];
/* Received Data over USB are stored in this buffer */
#[no_mangle]
pub static mut UserTxBuffer: [uint8_t; 2048] = [0; 2048];
/* Received Data over UART (CDC interface) are stored in this buffer */
#[no_mangle]
pub static mut BuffLength: uint32_t = 0;
#[no_mangle]
pub static mut UserTxBufPtrIn: uint32_t = 0 as libc::c_int as uint32_t;
/* Increment this pointer or roll it back to
                               start address when data are received over USART */
#[no_mangle]
pub static mut UserTxBufPtrOut: uint32_t = 0 as libc::c_int as uint32_t;
/* Increment this pointer or roll it back to
                                 start address when data are sent over USB */
#[no_mangle]
pub static mut rxAvailable: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut rxBuffPtr: *mut uint8_t = 0 as *const uint8_t as *mut uint8_t;
/* TIM handler declaration */
#[no_mangle]
pub static mut TimHandle: TIM_HandleTypeDef =
    TIM_HandleTypeDef{Instance: 0 as *const TIM_TypeDef as *mut TIM_TypeDef,
                      Init:
                          TIM_Base_InitTypeDef{Prescaler: 0,
                                               CounterMode: 0,
                                               Period: 0,
                                               ClockDivision: 0,
                                               RepetitionCounter: 0,
                                               AutoReloadPreload: 0,},
                      Channel: HAL_TIM_ACTIVE_CHANNEL_CLEARED,
                      hdma:
                          [0 as *const DMA_HandleTypeDef as
                               *mut DMA_HandleTypeDef; 7],
                      Lock: HAL_UNLOCKED,
                      State: HAL_TIM_STATE_RESET,};
static mut ctrlLineStateCb:
       Option<unsafe extern "C" fn(_: *mut libc::c_void, _: uint16_t) -> ()> =
    None;
static mut ctrlLineStateCbContext: *mut libc::c_void =
    0 as *const libc::c_void as *mut libc::c_void;
static mut baudRateCb:
       Option<unsafe extern "C" fn(_: *mut libc::c_void, _: uint32_t) -> ()> =
    None;
static mut baudRateCbContext: *mut libc::c_void =
    0 as *const libc::c_void as *mut libc::c_void;
#[no_mangle]
pub static mut USBD_CDC_fops: USBD_CDC_ItfTypeDef =
    unsafe {
        {
            let mut init =
                _USBD_CDC_Itf{Init:
                                  Some(CDC_Itf_Init as
                                           unsafe extern "C" fn() -> int8_t),
                              DeInit:
                                  Some(CDC_Itf_DeInit as
                                           unsafe extern "C" fn() -> int8_t),
                              Control:
                                  Some(CDC_Itf_Control as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _: uint16_t)
                                               -> int8_t),
                              Receive:
                                  Some(CDC_Itf_Receive as
                                           unsafe extern "C" fn(_:
                                                                    *mut uint8_t,
                                                                _:
                                                                    *mut uint32_t)
                                               -> int8_t),};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn TIM7_IRQHandler() {
    HAL_TIM_IRQHandler(&mut TimHandle);
}
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* *
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
unsafe extern "C" fn CDC_Itf_Init() -> int8_t {
    /*##-3- Configure the TIM Base generation  #################################*/
    TIM_Config();
    /*##-4- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
    if HAL_TIM_Base_Start_IT(&mut TimHandle) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        /* Starting Error */
        Error_Handler();
    }
    /*##-5- Set Application Buffers ############################################*/
    USBD_CDC_SetTxBuffer(&mut USBD_Device, UserTxBuffer.as_mut_ptr(),
                         0 as libc::c_int as uint16_t);
    USBD_CDC_SetRxBuffer(&mut USBD_Device, UserRxBuffer.as_mut_ptr());
    ctrlLineStateCb = None;
    baudRateCb = None;
    return USBD_OK as libc::c_int as int8_t;
}
/* *
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
unsafe extern "C" fn CDC_Itf_DeInit() -> int8_t {
    return USBD_OK as libc::c_int as int8_t;
}
/* *
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
unsafe extern "C" fn CDC_Itf_Control(mut cmd: uint8_t, mut pbuf: *mut uint8_t,
                                     mut length: uint16_t) -> int8_t {
    let mut plc: *mut LINE_CODING = pbuf as *mut LINE_CODING;
    match cmd as libc::c_int {
        1 => { }
        2 => { }
        3 => { }
        4 => { }
        32 => {
            if !pbuf.is_null() &&
                   length as libc::c_ulong ==
                       ::core::mem::size_of::<LINE_CODING>() as libc::c_ulong
               {
                LineCoding.bitrate = (*plc).bitrate;
                LineCoding.format = (*plc).format;
                LineCoding.paritytype = (*plc).paritytype;
                LineCoding.datatype = (*plc).datatype;
                // If a callback is provided, tell the upper driver of changes in baud rate
                if baudRateCb.is_some() {
                    baudRateCb.expect("non-null function pointer")(baudRateCbContext,
                                                                   LineCoding.bitrate);
                }
            }
        }
        33 => {
            if !pbuf.is_null() &&
                   length as libc::c_ulong ==
                       ::core::mem::size_of::<LINE_CODING>() as libc::c_ulong
               {
                (*plc).bitrate = LineCoding.bitrate;
                (*plc).format = LineCoding.format;
                (*plc).paritytype = LineCoding.paritytype;
                (*plc).datatype = LineCoding.datatype
            }
        }
        34 => {
            // If a callback is provided, tell the upper driver of changes in DTR/RTS state
            if !pbuf.is_null() &&
                   length as libc::c_ulong ==
                       ::core::mem::size_of::<uint16_t>() as libc::c_ulong {
                if ctrlLineStateCb.is_some() {
                    ctrlLineStateCb.expect("non-null function pointer")(ctrlLineStateCbContext,
                                                                        *(pbuf
                                                                              as
                                                                              *mut uint16_t));
                }
            }
        }
        35 => { }
        0 | _ => { }
    }
    return USBD_OK as libc::c_int as int8_t;
}
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup TIM_Exported_Constants  TIM Exported Constants
  * @{
  */
/* * @defgroup TIM_Input_Channel_Polarity TIM Input Channel Polarity
  * @{
  */
/* !< Polarity for TIx source */
/* !< Polarity for TIx source */
/* !< Polarity for TIx source */
/* *
  * @}
  */
/* * @defgroup TIM_ETR_Polarity  TIM ETR Polarity
  * @{
  */
/* !< Polarity for ETR source */
/* !< Polarity for ETR source */
/* *
  * @}
  */
/* * @defgroup TIM_ETR_Prescaler  TIM ETR Prescaler
  * @{
  */
/* !< No prescaler is used */
/* !< ETR input source is divided by 2 */
/* !< ETR input source is divided by 4 */
/* !< ETR input source is divided by 8 */
/* *
  * @}
  */
/* * @defgroup TIM_Counter_Mode  TIM Counter Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_ClockDivision TIM Clock Division
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_State TIM Output Compare State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_AutoReloadPreload TIM Auto-Reload Preload
  * @{
  */
/* !< TIMx_ARR register is not buffered */
/* !< TIMx_ARR register is buffered */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Fast_State  TIM Output Fast State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_N_State TIM Complementary Output Compare State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_Polarity TIM Output Compare Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_N_Polarity TIM Complementary Output Compare Polarity
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_Idle_State  TIM Output Compare Idle State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_N_Idle_State  TIM Output Compare N Idle State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Input_Capture_Polarity  TIM Input Capture Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Input_Capture_Selection  TIM Input Capture Selection
  * @{
  */
/* !< TIM Input 1, 2, 3 or 4 is selected to be 
                                                                     connected to IC1, IC2, IC3 or IC4, respectively */
/* !< TIM Input 1, 2, 3 or 4 is selected to be
                                                                     connected to IC2, IC1, IC4 or IC3, respectively */
/* !< TIM Input 1, 2, 3 or 4 is selected to be connected to TRC */
/* *
  * @}
  */
/* * @defgroup TIM_Input_Capture_Prescaler  TIM Input Capture Prescaler
  * @{
  */
/* !< Capture performed each time an edge is detected on the capture input */
/* !< Capture performed once every 2 events */
/* !< Capture performed once every 4 events */
/* !< Capture performed once every 8 events */
/* *
  * @}
  */
/* * @defgroup TIM_One_Pulse_Mode TIM One Pulse Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Encoder_Mode TIM Encoder Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Interrupt_definition  TIM Interrupt definition
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Commutation_Source  TIM Commutation Source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_DMA_sources  TIM DMA sources
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Event_Source  TIM Event Source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Flag_definition  TIM Flag definition
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Clock_Source  TIM Clock Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Clock_Polarity  TIM Clock Polarity
  * @{
  */
/* !< Polarity for ETRx clock sources */
/* !< Polarity for ETRx clock sources */
/* !< Polarity for TIx clock sources */
/* !< Polarity for TIx clock sources */
/* !< Polarity for TIx clock sources */
/* *
  * @}
  */
/* * @defgroup TIM_Clock_Prescaler  TIM Clock Prescaler
  * @{
  */
/* !< No prescaler is used */
/* !< Prescaler for External ETR Clock: Capture performed once every 2 events. */
/* !< Prescaler for External ETR Clock: Capture performed once every 4 events. */
/* !< Prescaler for External ETR Clock: Capture performed once every 8 events. */
/* *
  * @}
  */
/* * @defgroup TIM_ClearInput_Polarity  TIM Clear Input Polarity
  * @{
  */
/* !< Polarity for ETRx pin */
/* !< Polarity for ETRx pin */
/* *
  * @}
  */
/* * @defgroup TIM_ClearInput_Prescaler TIM Clear Input Prescaler
  * @{
  */
/* !< No prescaler is used */
/* !< Prescaler for External ETR pin: Capture performed once every 2 events. */
/* !< Prescaler for External ETR pin: Capture performed once every 4 events. */
/* !< Prescaler for External ETR pin: Capture performed once every 8 events. */
/* *
  * @}
  */
/* * @defgroup TIM_OSSR_Off_State_Selection_for_Run_mode_state TIM OSSR OffState Selection for Run mode state
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_OSSI_Off_State_Selection_for_Idle_mode_state TIM OSSI OffState Selection for Idle mode state
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Lock_level  TIM Lock level
  * @{
  */
/* *
  * @}
  */  
/* * @defgroup TIM_Break_Input_enable_disable  TIM Break Input State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break_Polarity  TIM Break Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_AOE_Bit_Set_Reset  TIM AOE Bit State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Master_Mode_Selection TIM Master Mode Selection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Master_Slave_Mode  TIM Master Slave Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Trigger_Selection  TIM Trigger Selection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Trigger_Polarity TIM Trigger Polarity
  * @{
  */
/* !< Polarity for ETRx trigger sources */
/* !< Polarity for ETRx trigger sources */
/* !< Polarity for TIxFPx or TI1_ED trigger sources */
/* !< Polarity for TIxFPx or TI1_ED trigger sources */
/* !< Polarity for TIxFPx or TI1_ED trigger sources */
/* *
  * @}
  */
/* * @defgroup TIM_Trigger_Prescaler TIM Trigger Prescaler
  * @{
  */
/* !< No prescaler is used */
/* !< Prescaler for External ETR Trigger: Capture performed once every 2 events. */
/* !< Prescaler for External ETR Trigger: Capture performed once every 4 events. */
/* !< Prescaler for External ETR Trigger: Capture performed once every 8 events. */
/* *
  * @}
  */
/* * @defgroup TIM_TI1_Selection TIM TI1 Selection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_DMA_Base_address  TIM DMA Base address
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_DMA_Burst_Length  TIM DMA Burst Length 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Handle_index  DMA Handle index
  * @{
  */
/* !< Index of the DMA handle used for Update DMA requests */
/* !< Index of the DMA handle used for Capture/Compare 1 DMA requests */
/* !< Index of the DMA handle used for Capture/Compare 2 DMA requests */
/* !< Index of the DMA handle used for Capture/Compare 3 DMA requests */
/* !< Index of the DMA handle used for Capture/Compare 4 DMA requests */
/* !< Index of the DMA handle used for Commutation DMA requests */
/* !< Index of the DMA handle used for Trigger DMA requests */
/* *
  * @}
  */
/* * @defgroup Channel_CC_State  Channel CC State
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup TIM_Exported_Macros TIM Exported Macros
  * @{
  */
/* * @brief Reset TIM handle state
  * @param  __HANDLE__: TIM handle
  * @retval None
  */
/* *
  * @brief  Enable the TIM peripheral.
  * @param  __HANDLE__: TIM handle
  * @retval None
 */
/* *
  * @brief  Enable the TIM update source request.
  * @param  __HANDLE__: TIM handle
  * @retval None
 */
/* *
  * @brief  Enable the TIM main Output.
  * @param  __HANDLE__: TIM handle
  * @retval None
  */
/* The counter of a timer instance is disabled only if all the CCx and CCxN
   channels have been disabled */
/* *
  * @brief  Disable the TIM peripheral.
  * @param  __HANDLE__: TIM handle
  * @retval None
  */
/* *
  * @brief  Disable the TIM update source request.
  * @param  __HANDLE__: TIM handle
  * @retval None
 */
/* The Main Output of a timer instance is disabled only if all the CCx and CCxN
   channels have been disabled */
/* *
  * @brief  Disable the TIM main Output.
  * @param  __HANDLE__: TIM handle
  * @retval None
  */
/* *
  * @brief  Sets the TIM Counter Register value on runtime.
  * @param  __HANDLE__: TIM handle.
  * @param  __COUNTER__: specifies the Counter register new value.
  * @retval None
  */
/* *
  * @brief  Gets the TIM Counter Register value on runtime.
  * @param  __HANDLE__: TIM handle.
  * @retval None
  */
/* *
  * @brief  Sets the TIM Autoreload Register value on runtime without calling 
  *         another time any Init function.
  * @param  __HANDLE__: TIM handle.
  * @param  __AUTORELOAD__: specifies the Counter register new value.
  * @retval None
  */
/* *
  * @brief  Gets the TIM Autoreload Register value on runtime
  * @param  __HANDLE__: TIM handle.
  * @retval None
  */
/* *
  * @brief  Sets the TIM Clock Division value on runtime without calling 
  *         another time any Init function. 
  * @param  __HANDLE__: TIM handle.
  * @param  __CKD__: specifies the clock division value.
  *          This parameter can be one of the following value:
  *            @arg TIM_CLOCKDIVISION_DIV1
  *            @arg TIM_CLOCKDIVISION_DIV2
  *            @arg TIM_CLOCKDIVISION_DIV4
  * @retval None
  */
/* *
  * @brief  Gets the TIM Clock Division value on runtime
  * @param  __HANDLE__: TIM handle.
  * @retval None
  */
/* *
  * @brief  Sets the TIM Input Capture prescaler on runtime without calling 
  *         another time HAL_TIM_IC_ConfigChannel() function.
  * @param  __HANDLE__: TIM handle.
  * @param  __CHANNEL__ : TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  __ICPSC__: specifies the Input Capture4 prescaler new value.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPSC_DIV1: no prescaler
  *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
  *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
  *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
/* *
  * @brief  Gets the TIM Input Capture prescaler on runtime
  * @param  __HANDLE__: TIM handle.
  * @param  __CHANNEL__ : TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: get input capture 1 prescaler value
  *            @arg TIM_CHANNEL_2: get input capture 2 prescaler value
  *            @arg TIM_CHANNEL_3: get input capture 3 prescaler value
  *            @arg TIM_CHANNEL_4: get input capture 4 prescaler value
  * @retval None
  */
/* *
  * @brief  Sets the TIM Capture x input polarity on runtime.
  * @param  __HANDLE__: TIM handle.
  * @param  __CHANNEL__: TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  __POLARITY__: Polarity for TIx source   
  *            @arg TIM_INPUTCHANNELPOLARITY_RISING: Rising Edge
  *            @arg TIM_INPUTCHANNELPOLARITY_FALLING: Falling Edge
  *            @arg TIM_INPUTCHANNELPOLARITY_BOTHEDGE: Rising and Falling Edge
  * @note  The polarity TIM_INPUTCHANNELPOLARITY_BOTHEDGE is not authorized  for TIM Channel 4.     
  * @retval None
  */
/* *
  * @}
  */
/* Include TIM HAL Extension module */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup TIM_Exported_Functions
  * @{
  */
/* * @addtogroup TIM_Exported_Functions_Group1
  * @{
  */
/* Time Base functions ********************************************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group2
  * @{
  */
/* Timer Output Compare functions **********************************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group3
  * @{
  */
/* Timer PWM functions *********************************************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group4
  * @{
  */
/* Timer Input Capture functions ***********************************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group5
  * @{
  */
/* Timer One Pulse functions ***************************************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* *
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group6
  * @{
  */
/* Timer Encoder functions *****************************************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group7
  * @{
  */
/* Interrupt Handler functions  **********************************************/
/* *
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group8
  * @{
  */
/* Control functions  *********************************************************/
/* *
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group9
  * @{
  */
/* Callback in non blocking modes (Interrupt and DMA) *************************/
/* *
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PeriodElapsedCallback(mut htim:
                                                           *mut TIM_HandleTypeDef) {
    if (*htim).Instance !=
           (0x40000000 as libc::c_uint).wrapping_add(0x1400 as libc::c_uint)
               as *mut TIM_TypeDef {
        return
    }
    let mut buffptr: uint32_t = 0;
    let mut buffsize: uint32_t = 0;
    if UserTxBufPtrOut != UserTxBufPtrIn {
        if UserTxBufPtrOut > UserTxBufPtrIn {
            /* Roll-back */
            buffsize =
                (2048 as libc::c_int as
                     libc::c_uint).wrapping_sub(UserTxBufPtrOut)
        } else { buffsize = UserTxBufPtrIn.wrapping_sub(UserTxBufPtrOut) }
        buffptr = UserTxBufPtrOut;
        USBD_CDC_SetTxBuffer(&mut USBD_Device,
                             &mut *UserTxBuffer.as_mut_ptr().offset(buffptr as
                                                                        isize)
                                 as *mut uint8_t, buffsize as uint16_t);
        if USBD_CDC_TransmitPacket(&mut USBD_Device) as libc::c_int ==
               USBD_OK as libc::c_int {
            UserTxBufPtrOut =
                (UserTxBufPtrOut as libc::c_uint).wrapping_add(buffsize) as
                    uint32_t as uint32_t;
            if UserTxBufPtrOut == 2048 as libc::c_int as libc::c_uint {
                UserTxBufPtrOut = 0 as libc::c_int as uint32_t
            }
        }
    };
}
/* *
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
unsafe extern "C" fn CDC_Itf_Receive(mut Buf: *mut uint8_t,
                                     mut Len: *mut uint32_t) -> int8_t {
    rxAvailable = *Len;
    rxBuffPtr = Buf;
    return USBD_OK as libc::c_int as int8_t;
}
/* *
  * @brief  TIM_Config: Configure TIMusb timer
  * @param  None.
  * @retval None
  */
unsafe extern "C" fn TIM_Config() {
    /* Set TIMusb instance */
    TimHandle.Instance =
        (0x40000000 as libc::c_uint).wrapping_add(0x1400 as libc::c_uint) as
            *mut TIM_TypeDef;
    /* Initialize TIMx peripheral as follow:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
    TimHandle.Init.Period =
        (10 as libc::c_int * 1000 as libc::c_int - 1 as libc::c_int) as
            uint32_t;
    TimHandle.Init.Prescaler =
        SystemCoreClock.wrapping_div(2 as libc::c_int as
                                         libc::c_uint).wrapping_div(1000000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_sub(1
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint);
    TimHandle.Init.ClockDivision = 0 as libc::c_int as uint32_t;
    TimHandle.Init.CounterMode = 0 as libc::c_uint;
    if HAL_TIM_Base_Init(&mut TimHandle) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        /* Initialization Error */
        Error_Handler();
    }
    /*##-6- Enable TIM peripherals Clock #######################################*/
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         5 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        5 as libc::c_uint);
    /*##-7- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM7_IRQn, 6 as libc::c_int as uint32_t,
                         0 as libc::c_int as uint32_t);
    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}
/* *
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
unsafe extern "C" fn Error_Handler() {
    /* Add your own code here */
}
#[no_mangle]
pub unsafe extern "C" fn CDC_Receive_DATA(mut recvBuf: *mut uint8_t,
                                          mut len: uint32_t) -> uint32_t {
    let mut count: uint32_t = 0 as libc::c_int as uint32_t;
    if !rxBuffPtr.is_null() {
        while rxAvailable > 0 as libc::c_int as libc::c_uint && count < len {
            *recvBuf.offset(count as isize) =
                *rxBuffPtr.offset(0 as libc::c_int as isize);
            rxBuffPtr = rxBuffPtr.offset(1);
            rxAvailable = rxAvailable.wrapping_sub(1);
            count = count.wrapping_add(1);
            if rxAvailable < 1 as libc::c_int as libc::c_uint {
                USBD_CDC_ReceivePacket(&mut USBD_Device);
            }
        }
    }
    return count;
}
#[no_mangle]
pub unsafe extern "C" fn CDC_Receive_BytesAvailable() -> uint32_t {
    return rxAvailable;
}
#[no_mangle]
pub unsafe extern "C" fn CDC_Send_FreeBytes() -> uint32_t {
    /*
        return the bytes free in the circular buffer

        functionally equivalent to:
        (APP_Rx_ptr_out > APP_Rx_ptr_in ? APP_Rx_ptr_out - APP_Rx_ptr_in : APP_RX_DATA_SIZE - APP_Rx_ptr_in + APP_Rx_ptr_in)
        but without the impact of the condition check.
    */
    return UserTxBufPtrOut.wrapping_sub(UserTxBufPtrIn).wrapping_add((-((UserTxBufPtrOut
                                                                             <=
                                                                             UserTxBufPtrIn)
                                                                            as
                                                                            libc::c_int)
                                                                          &
                                                                          2048
                                                                              as
                                                                              libc::c_int)
                                                                         as
                                                                         libc::c_uint).wrapping_sub(1
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        libc::c_uint);
}
/* *
 * @brief  CDC_Send_DATA
 *         CDC received data to be send over USB IN endpoint are managed in
 *         this function.
 * @param  ptrBuffer: Buffer of data to be sent
 * @param  sendLength: Number of data to be sent (in bytes)
 * @retval Bytes sent
 */
#[no_mangle]
pub unsafe extern "C" fn CDC_Send_DATA(mut ptrBuffer: *const uint8_t,
                                       mut sendLength: uint32_t) -> uint32_t {
    let mut hcdc: *mut USBD_CDC_HandleTypeDef =
        USBD_Device.pCDC_ClassData as *mut USBD_CDC_HandleTypeDef;
    while (*hcdc).TxState != 0 as libc::c_int as libc::c_uint { }
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    while i < sendLength {
        UserTxBuffer[UserTxBufPtrIn as usize] = *ptrBuffer.offset(i as isize);
        UserTxBufPtrIn =
            UserTxBufPtrIn.wrapping_add(1 as libc::c_int as
                                            libc::c_uint).wrapping_rem(2048 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint);
        while CDC_Send_FreeBytes() == 0 as libc::c_int as libc::c_uint {
            delay(1 as libc::c_int as timeMs_t);
        }
        i = i.wrapping_add(1)
    }
    return sendLength;
}
/* ******************************************************************************
 * Function Name  : usbIsConfigured.
 * Description    : Determines if USB VCP is configured or not
 * Input          : None.
 * Output         : None.
 * Return         : True if configured.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn usbIsConfigured() -> uint8_t {
    return (USBD_Device.dev_state as libc::c_int == 3 as libc::c_int) as
               libc::c_int as uint8_t;
}
/* ******************************************************************************
 * Function Name  : usbIsConnected.
 * Description    : Determines if USB VCP is connected ot not
 * Input          : None.
 * Output         : None.
 * Return         : True if connected.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn usbIsConnected() -> uint8_t {
    return (USBD_Device.dev_state as libc::c_int != 1 as libc::c_int) as
               libc::c_int as uint8_t;
}
/* ******************************************************************************
 * Function Name  : CDC_BaudRate.
 * Description    : Get the current baud rate
 * Input          : None.
 * Output         : None.
 * Return         : Baud rate in bps
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CDC_BaudRate() -> uint32_t {
    return LineCoding.bitrate;
}
/* ******************************************************************************
 * Function Name  : CDC_SetBaudRateCb
 * Description    : Set a callback to call when baud rate changes
 * Input          : callback function and context.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CDC_SetBaudRateCb(mut cb:
                                               Option<unsafe extern "C" fn(_:
                                                                               *mut libc::c_void,
                                                                           _:
                                                                               uint32_t)
                                                          -> ()>,
                                           mut context: *mut libc::c_void) {
    baudRateCbContext = context;
    baudRateCb = cb;
}
/* ******************************************************************************
 * Function Name  : CDC_SetCtrlLineStateCb
 * Description    : Set a callback to call when control line state changes
 * Input          : callback function and context.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CDC_SetCtrlLineStateCb(mut cb:
                                                    Option<unsafe extern "C" fn(_:
                                                                                    *mut libc::c_void,
                                                                                _:
                                                                                    uint16_t)
                                                               -> ()>,
                                                mut context:
                                                    *mut libc::c_void) {
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
