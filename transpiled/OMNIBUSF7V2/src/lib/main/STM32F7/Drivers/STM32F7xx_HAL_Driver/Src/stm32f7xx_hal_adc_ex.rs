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
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DMA_Abort(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
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
    #[no_mangle]
    fn HAL_ADC_ErrorCallback(hadc: *mut ADC_HandleTypeDef);
    #[no_mangle]
    fn HAL_ADC_ConvHalfCpltCallback(hadc: *mut ADC_HandleTypeDef);
    #[no_mangle]
    fn HAL_ADC_ConvCpltCallback(hadc: *mut ADC_HandleTypeDef);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* !< CM7 uses 4 Bits for the Priority Levels       */
/* !< Set to 1 if different SysTick Config is used  */
/* !< FPU present                                   */
/* !< CM7 instruction cache present                 */
/* !< CM7 data cache present                        */
/* * @addtogroup Peripheral_registers_structures
  * @{
  */
/* *
  * @brief Analog to Digital Converter
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_TypeDef {
    pub SR: uint32_t,
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMPR1: uint32_t,
    pub SMPR2: uint32_t,
    pub JOFR1: uint32_t,
    pub JOFR2: uint32_t,
    pub JOFR3: uint32_t,
    pub JOFR4: uint32_t,
    pub HTR: uint32_t,
    pub LTR: uint32_t,
    pub SQR1: uint32_t,
    pub SQR2: uint32_t,
    pub SQR3: uint32_t,
    pub JSQR: uint32_t,
    pub JDR1: uint32_t,
    pub JDR2: uint32_t,
    pub JDR3: uint32_t,
    pub JDR4: uint32_t,
    pub DR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_Common_TypeDef {
    pub CSR: uint32_t,
    pub CCR: uint32_t,
    pub CDR: uint32_t,
}
/* *
  * @brief DMA Controller
  */
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const ENABLE: C2RustUnnamed_0 = 1;
pub const DISABLE: C2RustUnnamed_0 = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_InitTypeDef {
    pub ClockPrescaler: uint32_t,
    pub Resolution: uint32_t,
    pub DataAlign: uint32_t,
    pub ScanConvMode: uint32_t,
    pub EOCSelection: uint32_t,
    pub ContinuousConvMode: uint32_t,
    pub NbrOfConversion: uint32_t,
    pub DiscontinuousConvMode: uint32_t,
    pub NbrOfDiscConversion: uint32_t,
    pub ExternalTrigConv: uint32_t,
    pub ExternalTrigConvEdge: uint32_t,
    pub DMAContinuousRequests: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_HandleTypeDef {
    pub Instance: *mut ADC_TypeDef,
    pub Init: ADC_InitTypeDef,
    pub NbrOfCurrentConversionRank: uint32_t,
    pub DMA_Handle: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: uint32_t,
    pub ErrorCode: uint32_t,
}
/* !< DMA Stream Index                       */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_adc.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of ADC HAL module.
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
/* * @addtogroup ADCEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup ADCEx_Exported_Types ADC Exported Types
  * @{
  */
/* * 
  * @brief  ADC Configuration injected Channel structure definition
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope channel: InjectedChannel, InjectedRank, InjectedSamplingTime, InjectedOffset
  *          - Scope injected group (affects all channels of injected group): InjectedNbrOfConversion, InjectedDiscontinuousConvMode,
  *            AutoInjectedConv, ExternalTrigInjecConvEdge, ExternalTrigInjecConv.
  * @note   The setting of these parameters with function HAL_ADCEx_InjectedConfigChannel() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled
  *          - For all except parameters 'InjectedDiscontinuousConvMode' and 'AutoInjectedConv': ADC enabled without conversion on going on injected group.
  *          - For parameters 'ExternalTrigInjecConv' and 'ExternalTrigInjecConvEdge': ADC enabled, even with conversion on going on injected group.
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_InjectionConfTypeDef {
    pub InjectedChannel: uint32_t,
    pub InjectedRank: uint32_t,
    pub InjectedSamplingTime: uint32_t,
    pub InjectedOffset: uint32_t,
    pub InjectedNbrOfConversion: uint32_t,
    pub InjectedDiscontinuousConvMode: uint32_t,
    pub AutoInjectedConv: uint32_t,
    pub ExternalTrigInjecConv: uint32_t,
    pub ExternalTrigInjecConvEdge: uint32_t,
}
/* * 
  * @brief ADC Configuration multi-mode structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_MultiModeTypeDef {
    pub Mode: uint32_t,
    pub DMAAccessMode: uint32_t,
    pub TwoSamplingDelay: uint32_t,
}
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup ADCEx_Exported_Functions ADC Exported Functions
  * @{
  */
/* * @defgroup ADCEx_Exported_Functions_Group1  Extended features functions 
  *  @brief    Extended features functions  
  *
@verbatim   
 ===============================================================================
                 ##### Extended features functions #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Start conversion of injected channel.
      (+) Stop conversion of injected channel.
      (+) Start multimode and enable DMA transfer.
      (+) Stop multimode and disable DMA transfer.
      (+) Get result of injected channel conversion.
      (+) Get result of multimode conversion.
      (+) Configure injected channels.
      (+) Configure multimode.
               
@endverbatim
  * @{
  */
/* *
  * @brief  Enables the selected ADC software start conversion of the injected channels.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_InjectedStart(mut hadc:
                                                     *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut counter: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Enable the ADC peripheral */
    /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Enable the Peripheral */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
        ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                    (3 as
                                         libc::c_uint).wrapping_mul(SystemCoreClock.wrapping_div(1000000
                                                                                                     as
                                                                                                     libc::c_int
                                                                                                     as
                                                                                                     libc::c_uint)));
        while counter != 0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&counter
                                                                                   as
                                                                                   *const uint32_t).wrapping_sub(1))
        }
    }
    /* Start conversion if ADC is effectively enabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Set ADC state                                                          */
    /* - Clear state bitfield related to injected group conversion results    */
    /* - Set state bitfield related to injected operation                     */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x1 as libc::c_uint |
                                              0x2000 as libc::c_uint) |
                                        0x1000 as libc::c_uint);
        /* Check if a regular conversion is ongoing */
    /* Note: On this device, there is no ADC error code fields related to     */
    /*       conversions on group injected only. In case of conversion on     */
    /*       going on group regular, no error code is reset.                  */
        if (*hadc).State & 0x100 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Reset ADC all error code fields */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t, 0 as libc::c_uint)
        }
        /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
        (*hadc).Lock = HAL_UNLOCKED;
        /* Clear injected group conversion flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          2 as libc::c_uint));
        /* Check if Multimode enabled */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x10000 as
                                                  libc::c_uint).wrapping_add(0x2300
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut ADC_Common_TypeDef)).CCR &
               (0x1f as libc::c_uint) << 0 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            tmp1 =
                ((*(*hadc).Instance).CR2 &
                     (0x3 as libc::c_uint) << 20 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) as libc::c_int as
                    uint32_t;
            tmp2 =
                ((*(*hadc).Instance).CR1 &
                     (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) as libc::c_int as
                    uint32_t;
            if tmp1 != 0 && tmp2 != 0 {
                /* Enable the selected ADC software conversion for injected group */
                ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     22 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        } else {
            tmp1 =
                ((*(*hadc).Instance).CR2 &
                     (0x3 as libc::c_uint) << 20 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) as libc::c_int as
                    uint32_t;
            tmp2 =
                ((*(*hadc).Instance).CR1 &
                     (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) as libc::c_int as
                    uint32_t;
            if (*hadc).Instance ==
                   (0x40000000 as
                        libc::c_uint).wrapping_add(0x10000 as
                                                       libc::c_uint).wrapping_add(0x2000
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut ADC_TypeDef && tmp1 != 0 && tmp2 != 0 {
                /* Enable the selected ADC software conversion for injected group */
                ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     22 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Enables the interrupt and starts ADC conversion of injected channels.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  *
  * @retval HAL status.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_InjectedStart_IT(mut hadc:
                                                        *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut counter: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Enable the ADC peripheral */
    /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Enable the Peripheral */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
        ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                    (3 as
                                         libc::c_uint).wrapping_mul(SystemCoreClock.wrapping_div(1000000
                                                                                                     as
                                                                                                     libc::c_int
                                                                                                     as
                                                                                                     libc::c_uint)));
        while counter != 0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&counter
                                                                                   as
                                                                                   *const uint32_t).wrapping_sub(1))
        }
    }
    /* Start conversion if ADC is effectively enabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Set ADC state                                                          */
    /* - Clear state bitfield related to injected group conversion results    */
    /* - Set state bitfield related to injected operation                     */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x1 as libc::c_uint |
                                              0x2000 as libc::c_uint) |
                                        0x1000 as libc::c_uint);
        /* Check if a regular conversion is ongoing */
    /* Note: On this device, there is no ADC error code fields related to     */
    /*       conversions on group injected only. In case of conversion on     */
    /*       going on group regular, no error code is reset.                  */
        if (*hadc).State & 0x100 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Reset ADC all error code fields */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t, 0 as libc::c_uint)
        }
        /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
        (*hadc).Lock = HAL_UNLOCKED;
        /* Clear injected group conversion flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          2 as libc::c_uint));
        /* Enable end of conversion interrupt for injected channels */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Check if Multimode enabled */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x10000 as
                                                  libc::c_uint).wrapping_add(0x2300
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut ADC_Common_TypeDef)).CCR &
               (0x1f as libc::c_uint) << 0 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            tmp1 =
                ((*(*hadc).Instance).CR2 &
                     (0x3 as libc::c_uint) << 20 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) as libc::c_int as
                    uint32_t;
            tmp2 =
                ((*(*hadc).Instance).CR1 &
                     (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) as libc::c_int as
                    uint32_t;
            if tmp1 != 0 && tmp2 != 0 {
                /* Enable the selected ADC software conversion for injected group */
                ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     22 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        } else {
            tmp1 =
                ((*(*hadc).Instance).CR2 &
                     (0x3 as libc::c_uint) << 20 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) as libc::c_int as
                    uint32_t;
            tmp2 =
                ((*(*hadc).Instance).CR1 &
                     (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) as libc::c_int as
                    uint32_t;
            if (*hadc).Instance ==
                   (0x40000000 as
                        libc::c_uint).wrapping_add(0x10000 as
                                                       libc::c_uint).wrapping_add(0x2000
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut ADC_TypeDef && tmp1 != 0 && tmp2 != 0 {
                /* Enable the selected ADC software conversion for injected group */
                ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     22 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stop conversion of injected channels. Disable ADC peripheral if
  *         no regular conversion is on going.
  * @note   If ADC must be disabled and if conversion is on going on 
  *         regular group, function HAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @note   If injected group mode auto-injection is enabled,
  *         function HAL_ADC_Stop must be used.
  * @note   In case of auto-injection mode, HAL_ADC_Stop must be used.
  * @param  hadc: ADC handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_InjectedStop(mut hadc:
                                                    *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp_hal_status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Stop potential conversion and disable ADC peripheral                     */
  /* Conditioned to:                                                          */
  /* - No conversion on the other group (regular group) is intended to        */
  /*   continue (injected and regular groups stop conversion and ADC disable  */
  /*   are common)                                                            */
  /* - In case of auto-injection mode, HAL_ADC_Stop must be used.             */
    if (*hadc).State & 0x100 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint &&
           (*(*hadc).Instance).CR1 &
               (0x1 as libc::c_uint) << 10 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
        /* Stop potential conversion on going, on regular and injected groups */
    /* Disable ADC peripheral */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Check if ADC is effectively disabled */
        if (*(*hadc).Instance).CR2 &
               (0x1 as libc::c_uint) << 0 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (*hadc).State &
                                            !(0x100 as libc::c_uint |
                                                  0x1000 as libc::c_uint) |
                                            0x1 as libc::c_uint)
        }
    } else {
        /* Update ADC state machine to error */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t);
        tmp_hal_status = HAL_ERROR
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return tmp_hal_status;
}
/* *
  * @brief  Poll for injected conversion complete
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  Timeout: Timeout value in millisecond.  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_InjectedPollForConversion(mut hadc:
                                                                 *mut ADC_HandleTypeDef,
                                                             mut Timeout:
                                                                 uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Check End of conversion flag */
    while !((*(*hadc).Instance).SR &
                (0x1 as libc::c_uint) << 2 as libc::c_uint ==
                (0x1 as libc::c_uint) << 2 as libc::c_uint) {
        /* Check for the Timeout */
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_int as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(tickstart) > Timeout {
                ::core::ptr::write_volatile(&mut (*hadc).State as
                                                *mut uint32_t,
                                            0x4 as libc::c_uint);
                /* Process unlocked */
                (*hadc).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    /* Clear injected group conversion flag */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as *mut uint32_t,
                                !((0x1 as libc::c_uint) << 3 as libc::c_uint |
                                      (0x1 as libc::c_uint) <<
                                          2 as libc::c_uint));
    /* Update ADC state machine */
    ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x2000 as libc::c_uint)
                                    as uint32_t as uint32_t);
    /* Determine whether any further conversion upcoming on group injected      */
  /* by external trigger, continuous mode or scan sequence on going.          */
  /* Note: On STM32F7, there is no independent flag of end of sequence.       */
  /*       The test of scan sequence on going is done either with scan        */
  /*       sequence disabled or with end of conversion flag set to            */
  /*       of end of sequence.                                                */
    if (*(*hadc).Instance).CR2 & (0x3 as libc::c_uint) << 20 as libc::c_uint
           == RESET as libc::c_int as libc::c_uint &&
           ((*(*hadc).Instance).JSQR &
                (0x3 as libc::c_uint) << 20 as libc::c_uint ==
                RESET as libc::c_int as libc::c_uint ||
                (*(*hadc).Instance).CR2 &
                    (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                    RESET as libc::c_int as libc::c_uint) &&
           ((*(*hadc).Instance).CR1 &
                (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                RESET as libc::c_int as libc::c_uint &&
                ((*(*hadc).Instance).CR2 &
                     (0x3 as libc::c_uint) << 28 as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint &&
                     (*hadc).Init.ContinuousConvMode ==
                         DISABLE as libc::c_int as libc::c_uint)) {
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1000 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if (*hadc).State & 0x100 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    }
    /* Return ADC state */
    return HAL_OK;
}
/* *
  * @brief  Stop conversion of injected channels, disable interruption of 
  *         end-of-conversion. Disable ADC peripheral if no regular conversion
  *         is on going.
  * @note   If ADC must be disabled and if conversion is on going on 
  *         regular group, function HAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @note   If injected group mode auto-injection is enabled,
  *         function HAL_ADC_Stop must be used.
  * @param  hadc: ADC handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_InjectedStop_IT(mut hadc:
                                                       *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp_hal_status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Stop potential conversion and disable ADC peripheral                     */
  /* Conditioned to:                                                          */
  /* - No conversion on the other group (regular group) is intended to        */
  /*   continue (injected and regular groups stop conversion and ADC disable  */
  /*   are common)                                                            */
  /* - In case of auto-injection mode, HAL_ADC_Stop must be used.             */
    if (*hadc).State & 0x100 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint &&
           (*(*hadc).Instance).CR1 &
               (0x1 as libc::c_uint) << 10 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
        /* Stop potential conversion on going, on regular and injected groups */
    /* Disable ADC peripheral */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Check if ADC is effectively disabled */
        if (*(*hadc).Instance).CR2 &
               (0x1 as libc::c_uint) << 0 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Disable ADC end of conversion interrupt for injected channels */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   7 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (*hadc).State &
                                            !(0x100 as libc::c_uint |
                                                  0x1000 as libc::c_uint) |
                                            0x1 as libc::c_uint)
        }
    } else {
        /* Update ADC state machine to error */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t);
        tmp_hal_status = HAL_ERROR
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return tmp_hal_status;
}
/* *
  * @brief  Gets the converted value from data register of injected channel.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  InjectedRank: the ADC injected rank.
  *          This parameter can be one of the following values:
  *            @arg ADC_INJECTED_RANK_1: Injected Channel1 selected
  *            @arg ADC_INJECTED_RANK_2: Injected Channel2 selected
  *            @arg ADC_INJECTED_RANK_3: Injected Channel3 selected
  *            @arg ADC_INJECTED_RANK_4: Injected Channel4 selected
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_InjectedGetValue(mut hadc:
                                                        *mut ADC_HandleTypeDef,
                                                    mut InjectedRank:
                                                        uint32_t)
 -> uint32_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Clear injected group conversion flag to have similar behaviour as        */
  /* regular group: reading data register also clears end of conversion flag. */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as *mut uint32_t,
                                !((0x1 as libc::c_uint) <<
                                      2 as libc::c_uint));
    /* Return the selected ADC converted value */
    match InjectedRank {
        4 => {
            ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                        (*(*hadc).Instance).JDR4)
        }
        3 => {
            ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                        (*(*hadc).Instance).JDR3)
        }
        2 => {
            ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                        (*(*hadc).Instance).JDR2)
        }
        1 => {
            ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                        (*(*hadc).Instance).JDR1)
        }
        _ => { }
    }
    return tmp;
}
/* *
  * @brief  Enables ADC DMA request after last transfer (Multi-ADC mode) and enables ADC peripheral
  * 
  * @note   Caution: This function must be used only with the ADC master.  
  *
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  pData:   Pointer to buffer in which transferred from ADC peripheral to memory will be stored. 
  * @param  Length:  The length of data to be transferred from ADC peripheral to memory.  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_MultiModeStart_DMA(mut hadc:
                                                          *mut ADC_HandleTypeDef,
                                                      mut pData:
                                                          *mut uint32_t,
                                                      mut Length: uint32_t)
 -> HAL_StatusTypeDef {
    let mut counter: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Enable the Peripheral */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Delay for temperature sensor stabilization time */
    /* Compute number of CPU cycles to wait for */
        ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                    (3 as
                                         libc::c_uint).wrapping_mul(SystemCoreClock.wrapping_div(1000000
                                                                                                     as
                                                                                                     libc::c_int
                                                                                                     as
                                                                                                     libc::c_uint)));
        while counter != 0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&counter
                                                                                   as
                                                                                   *const uint32_t).wrapping_sub(1))
        }
    }
    /* Start conversion if ADC is effectively enabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Set ADC state                                                          */
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x1 as libc::c_uint |
                                              0x200 as libc::c_uint |
                                              0x400 as libc::c_uint) |
                                        0x100 as libc::c_uint);
        /* If conversions on group regular are also triggering group injected,    */
    /* update ADC state.                                                      */
        if (*(*hadc).Instance).CR1 &
               (0x1 as libc::c_uint) << 10 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (*hadc).State &
                                            !(0x2000 as libc::c_uint) |
                                            0x1000 as libc::c_uint)
        }
        /* State machine update: Check if an injected conversion is ongoing */
        if (*hadc).State & 0x1000 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            /* Reset ADC error code fields related to conversions on group regular */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x2 as libc::c_uint |
                                                   0x4 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* Reset ADC all error code fields */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t, 0 as libc::c_uint)
        }
        /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
        (*hadc).Lock = HAL_UNLOCKED;
        /* Set the DMA transfer complete callback */
        (*(*hadc).DMA_Handle).XferCpltCallback =
            Some(ADC_MultiModeDMAConvCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA half transfer complete callback */
        (*(*hadc).DMA_Handle).XferHalfCpltCallback =
            Some(ADC_MultiModeDMAHalfConvCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*hadc).DMA_Handle).XferErrorCallback =
            Some(ADC_MultiModeDMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
    /* start (in case of SW start):                                           */
        /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          1 as libc::c_uint));
        /* Enable ADC overrun interrupt */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             26 as libc::c_uint) as uint32_t
                                        as uint32_t);
        if (*hadc).Init.DMAContinuousRequests !=
               DISABLE as libc::c_int as libc::c_uint {
            /* Enable the selected ADC DMA request after last transfer */
            let ref mut fresh0 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x10000 as
                                                       libc::c_uint).wrapping_add(0x2300
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut ADC_Common_TypeDef)).CCR;
            ::core::ptr::write_volatile(fresh0,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 13 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            /* Disable the selected ADC EOC rising on each regular channel conversion */
            let ref mut fresh1 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x10000 as
                                                       libc::c_uint).wrapping_add(0x2300
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut ADC_Common_TypeDef)).CCR;
            ::core::ptr::write_volatile(fresh1,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   13 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        /* Enable the DMA Stream */
        HAL_DMA_Start_IT((*hadc).DMA_Handle,
                         &mut (*((0x40000000 as
                                      libc::c_uint).wrapping_add(0x10000 as
                                                                     libc::c_uint).wrapping_add(0x2300
                                                                                                    as
                                                                                                    libc::c_uint)
                                     as *mut ADC_Common_TypeDef)).CDR as
                             *mut uint32_t as uint32_t, pData as uint32_t,
                         Length);
        /* if no external trigger present enable software conversion of regular channels */
        if (*(*hadc).Instance).CR2 &
               (0x3 as libc::c_uint) << 28 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Enable the selected ADC software conversion for regular group */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 30 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Disables ADC DMA (multi-ADC mode) and disables ADC peripheral    
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_MultiModeStop_DMA(mut hadc:
                                                         *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp_hal_status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Check if ADC is effectively disabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint {
        /* Disable the selected ADC DMA mode for multimode */
        let ref mut fresh2 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2300
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               13 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Disable the DMA channel (in case of DMA in circular mode or stop while */
    /* DMA transfer is on going)                                              */
        tmp_hal_status = HAL_DMA_Abort((*hadc).DMA_Handle);
        /* Disable ADC overrun interrupt */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               26 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x100 as libc::c_uint |
                                              0x1000 as libc::c_uint) |
                                        0x1 as libc::c_uint)
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return tmp_hal_status;
}
/* *
  * @brief  Returns the last ADC1, ADC2 and ADC3 regular conversions results 
  *         data in the selected multi mode.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval The converted data value.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_MultiModeGetValue(mut hadc:
                                                         *mut ADC_HandleTypeDef)
 -> uint32_t {
    /* Return the multi mode conversion value */
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x10000 as
                                                  libc::c_uint).wrapping_add(0x2300
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut ADC_Common_TypeDef)).CDR;
}
/* *
  * @brief  Injected conversion complete callback in non blocking mode 
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_InjectedConvCpltCallback(mut hadc:
                                                                *mut ADC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_InjectedConvCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  Configures for the selected ADC injected channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  sConfigInjected: ADC configuration structure for injected channel. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_InjectedConfigChannel(mut hadc:
                                                             *mut ADC_HandleTypeDef,
                                                         mut sConfigInjected:
                                                             *mut ADC_InjectionConfTypeDef)
 -> HAL_StatusTypeDef {
    /* USE_FULL_ASSERT  */
    /* Check the parameters */
    /* USE_FULL_ASSERT  */
    ((*sConfigInjected).ExternalTrigInjecConv) !=
        ((0xf as libc::c_uint) <<
             16 as
                 libc::c_uint).wrapping_add(1 as libc::c_int as libc::c_uint);
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
    if (*sConfigInjected).InjectedChannel >
           (0x8 as libc::c_uint) << 0 as libc::c_uint |
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Clear the old sample time */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x7 as libc::c_uint) <<
                                                0 as libc::c_uint) <<
                                               (3 as libc::c_int as
                                                    libc::c_uint).wrapping_mul(((*sConfigInjected).InjectedChannel
                                                                                    as
                                                                                    uint16_t
                                                                                    as
                                                                                    uint32_t).wrapping_sub(10
                                                                                                               as
                                                                                                               libc::c_int
                                                                                                               as
                                                                                                               libc::c_uint))))
                                        as uint32_t as uint32_t);
        /* Set the new sample time */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfigInjected).InjectedSamplingTime
                                             <<
                                             (3 as libc::c_int as
                                                  libc::c_uint).wrapping_mul(((*sConfigInjected).InjectedChannel
                                                                                  as
                                                                                  uint16_t
                                                                                  as
                                                                                  uint32_t).wrapping_sub(10
                                                                                                             as
                                                                                                             libc::c_int
                                                                                                             as
                                                                                                             libc::c_uint)))
                                        as uint32_t as uint32_t)
    } else {
        /* ADC_Channel include in ADC_Channel_[0..9] */
        /* Clear the old sample time */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x7 as libc::c_uint) <<
                                                0 as libc::c_uint) <<
                                               (3 as libc::c_int as
                                                    libc::c_uint).wrapping_mul((*sConfigInjected).InjectedChannel
                                                                                   as
                                                                                   uint16_t
                                                                                   as
                                                                                   uint32_t)))
                                        as uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfigInjected).InjectedSamplingTime
                                             <<
                                             (3 as libc::c_int as
                                                  libc::c_uint).wrapping_mul((*sConfigInjected).InjectedChannel
                                                                                 as
                                                                                 uint16_t
                                                                                 as
                                                                                 uint32_t))
                                        as uint32_t as uint32_t)
    }
    /* Set the new sample time */
    /*---------------------------- ADCx JSQR Configuration -----------------*/
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JSQR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JSQR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3 as libc::c_uint) <<
                                           20 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JSQR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JSQR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*sConfigInjected).InjectedNbrOfConversion.wrapping_sub(1
                                                                                                 as
                                                                                                 libc::c_int
                                                                                                 as
                                                                                                 uint8_t
                                                                                                 as
                                                                                                 libc::c_uint)
                                         << 20 as libc::c_int) as uint32_t as
                                    uint32_t);
    /* Rank configuration */
    /* Clear the old SQx bits for the selected rank */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JSQR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JSQR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((((0x1f as libc::c_uint) <<
                                             0 as libc::c_uint) as uint16_t as
                                            uint32_t) <<
                                           5 as libc::c_int *
                                               (*sConfigInjected).InjectedRank.wrapping_add(3
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint).wrapping_sub((*sConfigInjected).InjectedNbrOfConversion)
                                                   as uint8_t as libc::c_int))
                                    as uint32_t as uint32_t);
    /* Set the SQx bits for the selected rank */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JSQR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JSQR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((*sConfigInjected).InjectedChannel as
                                          uint16_t as uint32_t) <<
                                         5 as libc::c_int *
                                             (*sConfigInjected).InjectedRank.wrapping_add(3
                                                                                              as
                                                                                              libc::c_int
                                                                                              as
                                                                                              libc::c_uint).wrapping_sub((*sConfigInjected).InjectedNbrOfConversion)
                                                 as uint8_t as libc::c_int) as
                                    uint32_t as uint32_t);
    /* Enable external trigger if trigger selection is different of software  */
  /* start.                                                                 */
  /* Note: This configuration keeps the hardware feature of parameter       */
  /*       ExternalTrigConvEdge "trigger edge none" equivalent to           */
  /*       software start.                                                  */
    if (*sConfigInjected).ExternalTrigInjecConv !=
           ((0xf as libc::c_uint) <<
                16 as
                    libc::c_uint).wrapping_add(1 as libc::c_int as
                                                   libc::c_uint) {
        /* Select external trigger to start conversion */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xf as libc::c_uint) <<
                                               16 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfigInjected).ExternalTrigInjecConv)
                                        as uint32_t as uint32_t);
        /* Select external trigger polarity */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3 as libc::c_uint) <<
                                               20 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfigInjected).ExternalTrigInjecConvEdge)
                                        as uint32_t as uint32_t)
    } else {
        /* Reset the external trigger */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xf as libc::c_uint) <<
                                               16 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3 as libc::c_uint) <<
                                               20 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    if (*sConfigInjected).AutoInjectedConv !=
           DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC automatic injected group conversion */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             10 as libc::c_uint) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected ADC automatic injected group conversion */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               10 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    if (*sConfigInjected).InjectedDiscontinuousConvMode !=
           DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC injected discontinuous mode */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             12 as libc::c_uint) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected ADC injected discontinuous mode */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               12 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    match (*sConfigInjected).InjectedRank {
        1 => {
            /* Set injected channel 1 offset */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JOFR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JOFR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0xfff as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JOFR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JOFR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfigInjected).InjectedOffset)
                                            as uint32_t as uint32_t)
        }
        2 => {
            /* Set injected channel 2 offset */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JOFR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JOFR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0xfff as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JOFR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JOFR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfigInjected).InjectedOffset)
                                            as uint32_t as uint32_t)
        }
        3 => {
            /* Set injected channel 3 offset */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JOFR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JOFR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0xfff as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JOFR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JOFR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfigInjected).InjectedOffset)
                                            as uint32_t as uint32_t)
        }
        _ => {
            /* Set injected channel 4 offset */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JOFR4 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JOFR4
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0xfff as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).JOFR4 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).JOFR4
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfigInjected).InjectedOffset)
                                            as uint32_t as uint32_t)
        }
    }
    /* if ADC1 Channel_18 is selected enable VBAT Channel */
    if (*hadc).Instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_TypeDef &&
           (*sConfigInjected).InjectedChannel ==
               (0x10 as libc::c_uint) << 0 as libc::c_uint |
                   (0x2 as libc::c_uint) << 0 as libc::c_uint {
        /* Enable the VBAT channel*/
        let ref mut fresh3 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2300
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             22 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* if ADC1 Channel_16 or Channel_17 is selected enable TSVREFE Channel(Temperature sensor and VREFINT) */
    if (*hadc).Instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_TypeDef &&
           ((*sConfigInjected).InjectedChannel ==
                (0x10 as libc::c_uint) << 0 as libc::c_uint |
                    (0x2 as libc::c_uint) << 0 as libc::c_uint |
                    0x10000000 as libc::c_uint ||
                (*sConfigInjected).InjectedChannel ==
                    (0x10 as libc::c_uint) << 0 as libc::c_uint |
                        (0x1 as libc::c_uint) << 0 as libc::c_uint) {
        /* Enable the TSVREFE channel*/
        let ref mut fresh4 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2300
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             23 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup ADCEx_Exported_Constants ADC Exported Constants
  * @{
  */
/* * @defgroup ADCEx_Common_mode ADC Common Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADCEx_Direct_memory_access_mode_for_multi_mode ADC Direct Memory Access Mode For Multi Mode
  * @{
  */
/* !< DMA mode disabled */
/* !< DMA mode 1 enabled (2 / 3 half-words one by one - 1 then 2 then 3)*/
/* !< DMA mode 2 enabled (2 / 3 half-words by pairs - 2&1 then 1&3 then 3&2)*/
/* !< DMA mode 3 enabled (2 / 3 bytes by pairs - 2&1 then 1&3 then 3&2) */
/* *
  * @}
  */
/* * @defgroup ADCEx_External_trigger_edge_Injected ADC External Trigger Edge Injected
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADCEx_External_trigger_Source_Injected ADC External Trigger Source Injected
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADCEx_injected_rank ADC Injected Channel Rank
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADCEx_channels  ADC Specific Channels
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup ADC_Exported_Macros ADC Exported Macros
  * @{
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup ADCEx_Exported_Functions
  * @{
  */
/* * @addtogroup ADCEx_Exported_Functions_Group1
  * @{
  */
/* I/O operation functions ******************************************************/
/* Peripheral Control functions *************************************************/
/* *
  * @brief  Configures the ADC multi-mode 
  * @param  hadc      : pointer to a ADC_HandleTypeDef structure that contains
  *                     the configuration information for the specified ADC.  
  * @param  multimode : pointer to an ADC_MultiModeTypeDef structure that contains 
  *                     the configuration information for  multimode.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADCEx_MultiModeConfigChannel(mut hadc:
                                                              *mut ADC_HandleTypeDef,
                                                          mut multimode:
                                                              *mut ADC_MultiModeTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Set ADC mode */
    let ref mut fresh5 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2300
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_Common_TypeDef)).CCR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1f as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh6 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2300
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_Common_TypeDef)).CCR;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | (*multimode).Mode) as
                                    uint32_t as uint32_t);
    /* Set the ADC DMA access mode */
    let ref mut fresh7 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2300
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_Common_TypeDef)).CCR;
    ::core::ptr::write_volatile(fresh7,
                                (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3 as libc::c_uint) <<
                                           14 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh8 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2300
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_Common_TypeDef)).CCR;
    ::core::ptr::write_volatile(fresh8,
                                (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*multimode).DMAAccessMode) as uint32_t
                                    as uint32_t);
    /* Set delay between two sampling phases */
    let ref mut fresh9 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2300
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_Common_TypeDef)).CCR;
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0xf as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh10 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2300
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_Common_TypeDef)).CCR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*multimode).TwoSamplingDelay) as
                                    uint32_t as uint32_t);
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_adc_ex.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the ADC extension peripheral:
  *           + Extended features functions
  *         
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
    [..]
    (#)Initialize the ADC low level resources by implementing the HAL_ADC_MspInit():
       (##) Enable the ADC interface clock using __HAL_RCC_ADC_CLK_ENABLE()
       (##) ADC pins configuration
             (+++) Enable the clock for the ADC GPIOs using the following function:
                   __HAL_RCC_GPIOx_CLK_ENABLE()  
             (+++) Configure these ADC pins in analog mode using HAL_GPIO_Init() 
       (##) In case of using interrupts (e.g. HAL_ADC_Start_IT())
             (+++) Configure the ADC interrupt priority using HAL_NVIC_SetPriority()
             (+++) Enable the ADC IRQ handler using HAL_NVIC_EnableIRQ()
             (+++) In ADC IRQ handler, call HAL_ADC_IRQHandler()
      (##) In case of using DMA to control data transfer (e.g. HAL_ADC_Start_DMA())
             (+++) Enable the DMAx interface clock using __HAL_RCC_DMAx_CLK_ENABLE()
             (+++) Configure and enable two DMA streams stream for managing data
                 transfer from peripheral to memory (output stream)
             (+++) Associate the initialized DMA handle to the ADC DMA handle
                 using  __HAL_LINKDMA()
             (+++) Configure the priority and enable the NVIC for the transfer complete
                 interrupt on the two DMA Streams. The output stream should have higher
                 priority than the input stream.                  
     (#) Configure the ADC Prescaler, conversion resolution and data alignment 
         using the HAL_ADC_Init() function. 
  
     (#) Configure the ADC Injected channels group features, use HAL_ADC_Init()
         and HAL_ADC_ConfigChannel() functions.
         
     (#) Three operation modes are available within this driver :     
  
     *** Polling mode IO operation ***
     =================================
     [..]    
       (+) Start the ADC peripheral using HAL_ADCEx_InjectedStart() 
       (+) Wait for end of conversion using HAL_ADCEx_InjectedPollForConversion(), at this stage
           user can specify the value of timeout according to his end application      
       (+) To read the ADC converted values, use the HAL_ADCEx_InjectedGetValue() function.
       (+) Stop the ADC peripheral using HAL_ADCEx_InjectedStop()
  
     *** Interrupt mode IO operation ***    
     ===================================
     [..]    
       (+) Start the ADC peripheral using HAL_ADCEx_InjectedStart_IT() 
       (+) Use HAL_ADC_IRQHandler() called under ADC_IRQHandler() Interrupt subroutine
       (+) At ADC end of conversion HAL_ADCEx_InjectedConvCpltCallback() function is executed and user can 
            add his own code by customization of function pointer HAL_ADCEx_InjectedConvCpltCallback 
       (+) In case of ADC Error, HAL_ADCEx_InjectedErrorCallback() function is executed and user can 
            add his own code by customization of function pointer HAL_ADCEx_InjectedErrorCallback
       (+) Stop the ADC peripheral using HAL_ADCEx_InjectedStop_IT()
       
            
     *** DMA mode IO operation ***    
     ==============================
     [..]    
       (+) Start the ADC peripheral using HAL_ADCEx_InjectedStart_DMA(), at this stage the user specify the length 
           of data to be transferred at each end of conversion 
       (+) At The end of data transfer ba HAL_ADCEx_InjectedConvCpltCallback() function is executed and user can 
            add his own code by customization of function pointer HAL_ADCEx_InjectedConvCpltCallback 
       (+) In case of transfer Error, HAL_ADCEx_InjectedErrorCallback() function is executed and user can 
            add his own code by customization of function pointer HAL_ADCEx_InjectedErrorCallback
        (+) Stop the ADC peripheral using HAL_ADCEx_InjectedStop_DMA()
        
     *** Multi mode ADCs Regular channels configuration ***
     ======================================================
     [..]        
       (+) Select the Multi mode ADC regular channels features (dual or triple mode)  
          and configure the DMA mode using HAL_ADCEx_MultiModeConfigChannel() functions. 
       (+) Start the ADC peripheral using HAL_ADCEx_MultiModeStart_DMA(), at this stage the user specify the length 
           of data to be transferred at each end of conversion           
       (+) Read the ADCs converted values using the HAL_ADCEx_MultiModeGetValue() function.
  
  
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
/* * @defgroup ADCEx ADCEx
  * @brief ADC Extended driver modules
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @addtogroup ADCEx_Private_Functions
  * @{
  */
/* Private function prototypes -----------------------------------------------*/
/* *
  * @}
  */
/* *
  * @brief  DMA transfer complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn ADC_MultiModeDMAConvCplt(mut hdma:
                                                  *mut DMA_HandleTypeDef) {
    /* Retrieve ADC handle corresponding to current DMA handle */
    let mut hadc: *mut ADC_HandleTypeDef =
        (*hdma).Parent as *mut ADC_HandleTypeDef;
    /* Update state machine on conversion status if not in error state */
    if (*hadc).State & (0x10 as libc::c_uint | 0x40 as libc::c_uint) ==
           RESET as libc::c_int as libc::c_uint {
        /* Update ADC state machine */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x200 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Determine whether any further conversion upcoming on group regular   */
    /* by external trigger, continuous mode or scan sequence on going.      */
    /* Note: On STM32F7, there is no independent flag of end of sequence.   */
    /*       The test of scan sequence on going is done either with scan    */
    /*       sequence disabled or with end of conversion flag set to        */
    /*       of end of sequence.                                            */
        if (*(*hadc).Instance).CR2 &
               (0x3 as libc::c_uint) << 28 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint &&
               (*hadc).Init.ContinuousConvMode ==
                   DISABLE as libc::c_int as libc::c_uint &&
               ((*(*hadc).Instance).SQR1 &
                    (0xf as libc::c_uint) << 20 as libc::c_uint ==
                    RESET as libc::c_int as libc::c_uint ||
                    (*(*hadc).Instance).CR2 &
                        (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                        RESET as libc::c_int as libc::c_uint) {
            /* Disable ADC end of single conversion interrupt on group regular */
      /* Note: Overrun interrupt was enabled with EOC interrupt in          */
      /* HAL_ADC_Start_IT(), but is not disabled here because can be used   */
      /* by overrun IRQ process below.                                      */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   5 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x100 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if (*hadc).State & 0x1000 as libc::c_uint ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hadc).State as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x1 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
        /* Conversion complete callback */
        HAL_ADC_ConvCpltCallback(hadc);
    } else {
        /* Call DMA error callback */
        (*(*hadc).DMA_Handle).XferErrorCallback.expect("non-null function pointer")(hdma);
    };
}
/* *
  * @brief  DMA half transfer complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn ADC_MultiModeDMAHalfConvCplt(mut hdma:
                                                      *mut DMA_HandleTypeDef) {
    let mut hadc: *mut ADC_HandleTypeDef =
        (*hdma).Parent as *mut ADC_HandleTypeDef;
    /* Conversion complete callback */
    HAL_ADC_ConvHalfCpltCallback(hadc);
}
/* *
  * @brief  DMA error callback 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn ADC_MultiModeDMAError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hadc: *mut ADC_HandleTypeDef =
        (*hdma).Parent as *mut ADC_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                0x40 as libc::c_uint);
    /* Set ADC error code to DMA error */
    ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hadc).ErrorCode
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x4 as libc::c_uint) as
                                    uint32_t as uint32_t);
    HAL_ADC_ErrorCallback(hadc);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_ADC_MODULE_ENABLED */
/* *
  * @}
  */
