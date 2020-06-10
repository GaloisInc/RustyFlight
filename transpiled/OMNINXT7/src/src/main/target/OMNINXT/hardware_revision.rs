use ::libc;
extern "C" {
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
    #[no_mangle]
    fn HAL_ADCEx_InjectedStart(hadc: *mut ADC_HandleTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_ADCEx_InjectedPollForConversion(hadc: *mut ADC_HandleTypeDef,
                                           Timeout: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_ADCEx_InjectedGetValue(hadc: *mut ADC_HandleTypeDef,
                                  InjectedRank: uint32_t) -> uint32_t;
    /* Peripheral Control functions *************************************************/
    #[no_mangle]
    fn HAL_ADCEx_InjectedConfigChannel(hadc: *mut ADC_HandleTypeDef,
                                       sConfigInjected:
                                           *mut ADC_InjectionConfTypeDef)
     -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
/* * @defgroup ADC_Exported_Constants ADC Exported Constants
  * @{
  */
    /* * @defgroup ADC_Error_Code ADC Error Code
  * @{
  */
    /* !< No error                                              */
    /* !< ADC IP internal error: if problem of clocking, 
                                                          enable/disable, erroneous state                       */
    /* !< Overrun error                                         */
    /* !< DMA transfer error                                    */
    /* *
  * @}
  */
    /* * @defgroup ADC_ClockPrescaler  ADC Clock Prescaler
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_delay_between_2_sampling_phases ADC Delay Between 2 Sampling Phases
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_Resolution ADC Resolution
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_External_trigger_edge_Regular ADC External Trigger Edge Regular
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_External_trigger_Source_Regular ADC External Trigger Source Regular
  * @{
  */
/* Note: Parameter ADC_SOFTWARE_START is a software parameter used for        */
/*       compatibility with other STM32 devices.                              */
    /* *
  * @}
  */
    /* * @defgroup ADC_Data_Align ADC Data Align
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_Scan_mode ADC sequencer scan mode
  * @{
  */
    /* !< Scan mode disabled */
    /* !< Scan mode enabled  */
    /* *
  * @}
  */
    /* * @defgroup ADC_regular_rank ADC group regular sequencer rank
  * @{
  */
    /* !< ADC regular conversion rank 1  */
    /* !< ADC regular conversion rank 2  */
    /* !< ADC regular conversion rank 3  */
    /* !< ADC regular conversion rank 4  */
    /* !< ADC regular conversion rank 5  */
    /* !< ADC regular conversion rank 6  */
    /* !< ADC regular conversion rank 7  */
    /* !< ADC regular conversion rank 8  */
    /* !< ADC regular conversion rank 9  */
    /* !< ADC regular conversion rank 10 */
    /* !< ADC regular conversion rank 11 */
    /* !< ADC regular conversion rank 12 */
    /* !< ADC regular conversion rank 13 */
    /* !< ADC regular conversion rank 14 */
    /* !< ADC regular conversion rank 15 */
    /* !< ADC regular conversion rank 16 */
    /* *
  * @}
  */
    /* * @defgroup ADC_channels ADC Common Channels
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_sampling_times ADC Sampling Times
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_EOCSelection ADC EOC Selection
  * @{
  */
    /* !< reserved for future use */
    /* *
  * @}
  */
    /* * @defgroup ADC_Event_type ADC Event Type
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_analog_watchdog_selection ADC Analog Watchdog Selection
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_interrupts_definition ADC Interrupts Definition
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_flags_definition ADC Flags Definition
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup ADC_channels_type ADC Channels Type
  * @{
  */
    /* !< reserved for future use */
    /* !< reserved for future use */
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
    /* * @brief Reset ADC handle state
  * @param  __HANDLE__: ADC handle
  * @retval None
  */
    /* *
  * @brief  Enable the ADC peripheral.
  * @param  __HANDLE__: ADC handle
  * @retval None
  */
    /* *
  * @brief  Disable the ADC peripheral.
  * @param  __HANDLE__: ADC handle
  * @retval None
  */
    /* *
  * @brief  Enable the ADC end of conversion interrupt.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __INTERRUPT__: ADC Interrupt.
  * @retval None
  */
    /* *
  * @brief  Disable the ADC end of conversion interrupt.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __INTERRUPT__: ADC interrupt.
  * @retval None
  */
    /* * @brief  Check if the specified ADC interrupt source is enabled or disabled.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __INTERRUPT__: specifies the ADC interrupt source to check.
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
    /* *
  * @brief  Clear the ADC's pending flags.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __FLAG__: ADC flag.
  * @retval None
  */
    /* *
  * @brief  Get the selected ADC's flag status.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __FLAG__: ADC flag.
  * @retval None
  */
    /* *
  * @}
  */
    /* Include ADC HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup ADC_Exported_Functions
  * @{
  */
    /* * @addtogroup ADC_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions ***********************************/
    #[no_mangle]
    fn HAL_ADC_Init(hadc: *mut ADC_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_ADC_DeInit(hadc: *mut ADC_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn adcChannelByTag(ioTag: ioTag_t) -> uint8_t;
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type size_t = libc::c_ulong;
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
pub type C2RustUnnamed = libc::c_uint;
pub const ENABLE: C2RustUnnamed = 1;
pub const DISABLE: C2RustUnnamed = 0;
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
  * @file    stm32f7xx_hal_adc.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of ADC HAL extension module.
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
/* * @addtogroup ADC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup ADC_Exported_Types ADC Exported Types
  * @{
  */
/* * 
  * @brief  Structure definition of ADC and regular group initialization 
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope entire ADC (affects regular and injected groups): ClockPrescaler, Resolution, ScanConvMode, DataAlign, ScanConvMode, EOCSelection, LowPowerAutoWait, LowPowerAutoPowerOff, ChannelsBank.
  *          - Scope regular group: ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode, NbrOfDiscConversion, ExternalTrigConvEdge, ExternalTrigConv.
  * @note   The setting of these parameters with function HAL_ADC_Init() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled
  *          - For all parameters except 'Resolution', 'ScanConvMode', 'DiscontinuousConvMode', 'NbrOfDiscConversion' : ADC enabled without conversion on going on regular group.
  *          - For parameters 'ExternalTrigConv' and 'ExternalTrigConvEdge': ADC enabled, even with conversion on going.
  *         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
  *         without error reporting (as it can be the expected behaviour in case of intended action to update another parameter (which fullfills the ADC state condition) on the fly).
  */
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
/* * 
  * @brief  HAL ADC state machine: ADC states definition (bitfields)
  */ 
/* States of ADC global scope */
/* !< ADC not yet initialized or disabled */
/* !< ADC peripheral ready for use */
/* !< ADC is busy to internal process (initialization, calibration) */
/* !< TimeOut occurrence */
/* States of ADC errors */
/* !< Internal error occurrence */
/* !< Configuration error occurrence */
/* !< DMA error occurrence */
/* States of ADC group regular */
/* !< A conversion on group regular is ongoing or can occur (either by continuous mode,
                                                                       external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available)) */
/* !< Conversion data available on group regular */
/* !< Overrun occurrence */
/* States of ADC group injected */
/* !< A conversion on group injected is ongoing or can occur (either by auto-injection mode,
                                                                       external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available)) */
/* !< Conversion data available on group injected */
/* States of ADC analog watchdogs */
/* !< Out-of-window occurrence of analog watchdog 1 */
/* !< Not available on STM32F7 device: Out-of-window occurrence of analog watchdog 2 */
/* !< Not available on STM32F7 device: Out-of-window occurrence of analog watchdog 3 */
/* States of ADC multi-mode */
/* !< Not available on STM32F7 device: ADC in multimode slave state, controlled by another ADC master ( */
/* * 
  * @brief  ADC handle Structure definition
  */
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
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
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
pub type rccPeriphTag_t = uint8_t;
// Encoding for adcTagMap_t.devices
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcDevice_s {
    pub ADCx: *mut ADC_TypeDef,
    pub rccADC: rccPeriphTag_t,
    pub DMAy_Streamx: *mut DMA_Stream_TypeDef,
    pub channel: uint32_t,
    pub ADCHandle: ADC_HandleTypeDef,
    pub DmaHandle: DMA_HandleTypeDef,
}
pub type adcDevice_t = adcDevice_s;
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
pub type rcc_reg = libc::c_uint;
pub const RCC_AHB1: rcc_reg = 4;
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
// Do ADC on IDDetectPin and determine revision
// If VREFINT is used, we can (probably) get a pretty good precision
// that we can distinguish tens of different voltages.
pub type idDetect_t = idDetect_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct idDetect_s {
    pub ratio: uint32_t,
    pub revision: uint8_t,
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
pub static mut hardwareRevision: uint8_t = 0 as libc::c_int as uint8_t;
static mut idDetectTable: [idDetect_t; 1] =
    [{
         let mut init =
             idDetect_s{ratio:
                            (10000 as libc::c_int * 1000 as libc::c_int /
                                 (10000 as libc::c_int +
                                      10000 as libc::c_int)) as uint32_t,
                        revision: 1 as libc::c_int as uint8_t,};
         init
     }];
#[no_mangle]
pub static mut idDetectTag: ioTag_t = 0;
// Initialized in run_static_initializers
static mut adcIDDetHardware: adcDevice_t =
    adcDevice_t{ADCx: 0 as *const ADC_TypeDef as *mut ADC_TypeDef,
                rccADC: 0,
                DMAy_Streamx:
                    0 as *const DMA_Stream_TypeDef as *mut DMA_Stream_TypeDef,
                channel: 0,
                ADCHandle:
                    ADC_HandleTypeDef{Instance: 0 as *mut ADC_TypeDef,
                                      Init:
                                          ADC_InitTypeDef{ClockPrescaler: 0,
                                                          Resolution: 0,
                                                          DataAlign: 0,
                                                          ScanConvMode: 0,
                                                          EOCSelection: 0,
                                                          ContinuousConvMode:
                                                              0,
                                                          NbrOfConversion: 0,
                                                          DiscontinuousConvMode:
                                                              0,
                                                          NbrOfDiscConversion:
                                                              0,
                                                          ExternalTrigConv: 0,
                                                          ExternalTrigConvEdge:
                                                              0,
                                                          DMAContinuousRequests:
                                                              0,},
                                      NbrOfCurrentConversionRank: 0,
                                      DMA_Handle: 0 as *mut DMA_HandleTypeDef,
                                      Lock: HAL_UNLOCKED,
                                      State: 0,
                                      ErrorCode: 0,},
                DmaHandle:
                    DMA_HandleTypeDef{Instance: 0 as *mut DMA_Stream_TypeDef,
                                      Init:
                                          DMA_InitTypeDef{Channel: 0,
                                                          Direction: 0,
                                                          PeriphInc: 0,
                                                          MemInc: 0,
                                                          PeriphDataAlignment:
                                                              0,
                                                          MemDataAlignment: 0,
                                                          Mode: 0,
                                                          Priority: 0,
                                                          FIFOMode: 0,
                                                          FIFOThreshold: 0,
                                                          MemBurst: 0,
                                                          PeriphBurst: 0,},
                                      Lock: HAL_UNLOCKED,
                                      State: HAL_DMA_STATE_RESET,
                                      Parent: 0 as *mut libc::c_void,
                                      XferCpltCallback: None,
                                      XferHalfCpltCallback: None,
                                      XferM1CpltCallback: None,
                                      XferM1HalfCpltCallback: None,
                                      XferErrorCallback: None,
                                      XferAbortCallback: None,
                                      ErrorCode: 0,
                                      StreamBaseAddress: 0,
                                      StreamIndex: 0,},};
// XXX adcIDDetectInitDevice is an exact copy of adcInitDevice() from adc_stm32f7xx.c. Export and use?
unsafe extern "C" fn adcIDDetectInitDevice(mut adcdev: *mut adcDevice_t,
                                           mut channelCount: libc::c_int) {
    (*adcdev).ADCHandle.Init.ClockPrescaler =
        (0x3 as libc::c_uint) << 16 as libc::c_uint;
    (*adcdev).ADCHandle.Init.ContinuousConvMode =
        ENABLE as libc::c_int as uint32_t;
    (*adcdev).ADCHandle.Init.Resolution = 0 as libc::c_uint;
    (*adcdev).ADCHandle.Init.ExternalTrigConv = 0 as libc::c_uint;
    (*adcdev).ADCHandle.Init.ExternalTrigConvEdge = 0 as libc::c_uint;
    (*adcdev).ADCHandle.Init.DataAlign = 0 as libc::c_uint;
    (*adcdev).ADCHandle.Init.NbrOfConversion = channelCount as uint32_t;
    // Multiple injected channel seems to require scan conversion mode to be
    // enabled even if main (non-injected) channel count is 1.
    (*adcdev).ADCHandle.Init.ScanConvMode =
        ENABLE as libc::c_int as uint32_t; // Don't care
    (*adcdev).ADCHandle.Init.DiscontinuousConvMode =
        DISABLE as libc::c_int as uint32_t; // Don't care
    (*adcdev).ADCHandle.Init.NbrOfDiscConversion =
        0 as libc::c_int as uint32_t;
    (*adcdev).ADCHandle.Init.DMAContinuousRequests =
        ENABLE as libc::c_int as uint32_t;
    (*adcdev).ADCHandle.Init.EOCSelection =
        DISABLE as libc::c_int as uint32_t;
    (*adcdev).ADCHandle.Instance = (*adcdev).ADCx;
    (HAL_ADC_Init(&mut (*adcdev).ADCHandle) as libc::c_uint) !=
        HAL_OK as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn adcIDDetectInit() {
    idDetectTag =
        ((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
             2 as libc::c_int) as ioTag_t;
    IOConfigGPIO(IOGetByTag(idDetectTag),
                 (0x3 as libc::c_uint |
                      ((0 as libc::c_int) << 2 as libc::c_int) as libc::c_uint
                      | (0 as libc::c_uint) << 5 as libc::c_int) as
                     ioConfig_t);
    adcIDDetectInitDevice(&mut adcIDDetHardware, 2 as libc::c_int);
    let mut iConfig: ADC_InjectionConfTypeDef =
        ADC_InjectionConfTypeDef{InjectedChannel: 0,
                                 InjectedRank: 0,
                                 InjectedSamplingTime: 0,
                                 InjectedOffset: 0,
                                 InjectedNbrOfConversion: 0,
                                 InjectedDiscontinuousConvMode: 0,
                                 AutoInjectedConv: 0,
                                 ExternalTrigInjecConv: 0,
                                 ExternalTrigInjecConvEdge: 0,};
    iConfig.InjectedSamplingTime = (0x7 as libc::c_uint) << 0 as libc::c_uint;
    iConfig.InjectedOffset = 0 as libc::c_int as uint32_t;
    iConfig.InjectedNbrOfConversion = 2 as libc::c_int as uint32_t;
    iConfig.InjectedDiscontinuousConvMode =
        DISABLE as libc::c_int as uint32_t;
    iConfig.AutoInjectedConv = DISABLE as libc::c_int as uint32_t;
    iConfig.ExternalTrigInjecConv = 0 as libc::c_int as uint32_t;
    iConfig.ExternalTrigInjecConvEdge = 0 as libc::c_int as uint32_t;
    iConfig.InjectedChannel =
        (0x10 as libc::c_uint) << 0 as libc::c_uint |
            (0x1 as libc::c_uint) << 0 as libc::c_uint;
    iConfig.InjectedRank = 1 as libc::c_int as uint32_t;
    (HAL_ADCEx_InjectedConfigChannel(&mut adcIDDetHardware.ADCHandle,
                                     &mut iConfig) as libc::c_uint) !=
        HAL_OK as libc::c_int as libc::c_uint;
    iConfig.InjectedChannel = adcChannelByTag(idDetectTag) as uint32_t;
    iConfig.InjectedRank = 2 as libc::c_int as uint32_t;
    (HAL_ADCEx_InjectedConfigChannel(&mut adcIDDetHardware.ADCHandle,
                                     &mut iConfig) as libc::c_uint) !=
        HAL_OK as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn adcIDDetectDeinit() {
    HAL_ADC_DeInit(&mut adcIDDetHardware.ADCHandle);
    IOConfigGPIO(IOGetByTag(idDetectTag),
                 (0 as libc::c_uint | (0 as libc::c_uint) << 2 as libc::c_int
                      | (0x1 as libc::c_uint) << 5 as libc::c_int) as
                     ioConfig_t);
}
unsafe extern "C" fn adcIDDetectStart() {
    HAL_ADCEx_InjectedStart(&mut adcIDDetHardware.ADCHandle);
}
unsafe extern "C" fn adcIDDetectWait() {
    // Empty
    while HAL_ADCEx_InjectedPollForConversion(&mut adcIDDetHardware.ADCHandle,
                                              0 as libc::c_int as uint32_t) as
              libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
    };
}
unsafe extern "C" fn adcIDDetectReadVrefint() -> uint16_t {
    return HAL_ADCEx_InjectedGetValue(&mut adcIDDetHardware.ADCHandle,
                                      0x1 as libc::c_uint) as uint16_t;
}
unsafe extern "C" fn adcIDDetectReadIDDet() -> uint16_t {
    return HAL_ADCEx_InjectedGetValue(&mut adcIDDetHardware.ADCHandle,
                                      0x2 as libc::c_uint) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn detectHardwareRevision() {
    adcIDDetectInit();
    let mut vrefintValue: uint32_t = 0 as libc::c_int as uint32_t;
    let mut iddetValue: uint32_t = 0 as libc::c_int as uint32_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 16 as libc::c_int {
        adcIDDetectStart();
        adcIDDetectWait();
        iddetValue =
            (iddetValue as
                 libc::c_uint).wrapping_add(adcIDDetectReadIDDet() as
                                                libc::c_uint) as uint32_t as
                uint32_t;
        vrefintValue =
            (vrefintValue as
                 libc::c_uint).wrapping_add(adcIDDetectReadVrefint() as
                                                libc::c_uint) as uint32_t as
                uint32_t;
        i += 1
    }
    vrefintValue =
        (vrefintValue as
             libc::c_uint).wrapping_div(16 as libc::c_int as libc::c_uint) as
            uint32_t as uint32_t;
    iddetValue =
        (iddetValue as
             libc::c_uint).wrapping_div(16 as libc::c_int as libc::c_uint) as
            uint32_t as uint32_t;
    let mut iddetRatio: uint32_t =
        iddetValue.wrapping_mul(vrefintValue).wrapping_div(*(0x1ff07a2a as
                                                                 libc::c_int
                                                                 as
                                                                 *mut uint16_t)
                                                               as
                                                               libc::c_uint);
    iddetRatio =
        iddetRatio.wrapping_mul(1000 as libc::c_int as
                                    libc::c_uint).wrapping_div(4096 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint);
    let mut entry: size_t = 0 as libc::c_int as size_t;
    while entry <
              (::core::mem::size_of::<[idDetect_t; 1]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<idDetect_t>()
                                                   as libc::c_ulong) {
        if idDetectTable[entry as
                             usize].ratio.wrapping_sub(12 as libc::c_int as
                                                           libc::c_uint) <
               iddetRatio &&
               iddetRatio <
                   idDetectTable[entry as
                                     usize].ratio.wrapping_add(12 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint)
           {
            hardwareRevision = idDetectTable[entry as usize].revision;
            break ;
        } else { entry = entry.wrapping_add(1) }
    }
    adcIDDetectDeinit();
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
pub unsafe extern "C" fn updateHardwareRevision() {
    // Empty
}
#[no_mangle]
pub unsafe extern "C" fn selectMPUIntExtiConfigByHardwareRevision()
 -> ioTag_t {
    return 0 as libc::c_int as ioTag_t;
}
unsafe extern "C" fn run_static_initializers() {
    adcIDDetHardware =
        {
            let mut init =
                adcDevice_s{ADCx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x10000 as
                                                                    libc::c_uint).wrapping_add(0x2000
                                                                                                   as
                                                                                                   libc::c_uint)
                                    as *mut ADC_TypeDef,
                            rccADC:
                                (((RCC_APB2 as libc::c_int) <<
                                      5 as libc::c_int) as libc::c_long |
                                     (16 as libc::c_int *
                                          (((0x1 as libc::c_uint) <<
                                                8 as libc::c_uint) as
                                               libc::c_long >
                                               65535 as libc::c_long) as
                                              libc::c_int) as libc::c_long +
                                         ((8 as libc::c_int *
                                               (((0x1 as libc::c_uint) <<
                                                     8 as libc::c_uint) as
                                                    libc::c_long *
                                                    1 as libc::c_long >>
                                                    16 as libc::c_int *
                                                        (((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              8 as
                                                                  libc::c_uint)
                                                             as libc::c_long >
                                                             65535 as
                                                                 libc::c_long)
                                                            as libc::c_int >
                                                    255 as libc::c_int as
                                                        libc::c_long) as
                                                   libc::c_int) as
                                              libc::c_long +
                                              (8 as libc::c_int as
                                                   libc::c_long -
                                                   90 as libc::c_int as
                                                       libc::c_long /
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              8 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       8 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       8 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                8
                                                                                    as
                                                                                    libc::c_uint)
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            4 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            14 as libc::c_int
                                                                as
                                                                libc::c_long |
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long)
                                                   -
                                                   2 as libc::c_int as
                                                       libc::c_long /
                                                       ((((0x1 as
                                                               libc::c_uint)
                                                              <<
                                                              8 as
                                                                  libc::c_uint)
                                                             as libc::c_long *
                                                             1 as libc::c_long
                                                             >>
                                                             16 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       8 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      >
                                                                      65535 as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int
                                                             >>
                                                             8 as libc::c_int
                                                                 *
                                                                 (((0x1 as
                                                                        libc::c_uint)
                                                                       <<
                                                                       8 as
                                                                           libc::c_uint)
                                                                      as
                                                                      libc::c_long
                                                                      *
                                                                      1 as
                                                                          libc::c_long
                                                                      >>
                                                                      16 as
                                                                          libc::c_int
                                                                          *
                                                                          (((0x1
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                8
                                                                                    as
                                                                                    libc::c_uint)
                                                                               as
                                                                               libc::c_long
                                                                               >
                                                                               65535
                                                                                   as
                                                                                   libc::c_long)
                                                                              as
                                                                              libc::c_int
                                                                      >
                                                                      255 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_long)
                                                                     as
                                                                     libc::c_int)
                                                            /
                                                            2 as libc::c_int
                                                                as
                                                                libc::c_long +
                                                            1 as libc::c_int
                                                                as
                                                                libc::c_long))))
                                    as rccPeriphTag_t,
                            DMAy_Streamx:
                                (0x40000000 as
                                     libc::c_uint).wrapping_add(0x20000 as
                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                   as
                                                                                                   libc::c_uint).wrapping_add(0x70
                                                                                                                                  as
                                                                                                                                  libc::c_uint)
                                    as *mut DMA_Stream_TypeDef,
                            channel: 0 as libc::c_uint,
                            ADCHandle:
                                ADC_HandleTypeDef{Instance:
                                                      0 as *mut ADC_TypeDef,
                                                  Init:
                                                      ADC_InitTypeDef{ClockPrescaler:
                                                                          0,
                                                                      Resolution:
                                                                          0,
                                                                      DataAlign:
                                                                          0,
                                                                      ScanConvMode:
                                                                          0,
                                                                      EOCSelection:
                                                                          0,
                                                                      ContinuousConvMode:
                                                                          0,
                                                                      NbrOfConversion:
                                                                          0,
                                                                      DiscontinuousConvMode:
                                                                          0,
                                                                      NbrOfDiscConversion:
                                                                          0,
                                                                      ExternalTrigConv:
                                                                          0,
                                                                      ExternalTrigConvEdge:
                                                                          0,
                                                                      DMAContinuousRequests:
                                                                          0,},
                                                  NbrOfCurrentConversionRank:
                                                      0,
                                                  DMA_Handle:
                                                      0 as
                                                          *mut DMA_HandleTypeDef,
                                                  Lock: HAL_UNLOCKED,
                                                  State: 0,
                                                  ErrorCode: 0,},
                            DmaHandle:
                                DMA_HandleTypeDef{Instance:
                                                      0 as
                                                          *mut DMA_Stream_TypeDef,
                                                  Init:
                                                      DMA_InitTypeDef{Channel:
                                                                          0,
                                                                      Direction:
                                                                          0,
                                                                      PeriphInc:
                                                                          0,
                                                                      MemInc:
                                                                          0,
                                                                      PeriphDataAlignment:
                                                                          0,
                                                                      MemDataAlignment:
                                                                          0,
                                                                      Mode: 0,
                                                                      Priority:
                                                                          0,
                                                                      FIFOMode:
                                                                          0,
                                                                      FIFOThreshold:
                                                                          0,
                                                                      MemBurst:
                                                                          0,
                                                                      PeriphBurst:
                                                                          0,},
                                                  Lock: HAL_UNLOCKED,
                                                  State: HAL_DMA_STATE_RESET,
                                                  Parent:
                                                      0 as *mut libc::c_void,
                                                  XferCpltCallback: None,
                                                  XferHalfCpltCallback: None,
                                                  XferM1CpltCallback: None,
                                                  XferM1HalfCpltCallback:
                                                      None,
                                                  XferErrorCallback: None,
                                                  XferAbortCallback: None,
                                                  ErrorCode: 0,
                                                  StreamBaseAddress: 0,
                                                  StreamIndex: 0,},};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
