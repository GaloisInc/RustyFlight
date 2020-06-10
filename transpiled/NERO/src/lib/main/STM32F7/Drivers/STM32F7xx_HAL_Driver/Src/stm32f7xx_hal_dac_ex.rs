use ::libc;
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
/* *
  * @brief Digital to Analog Converter
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DAC_TypeDef {
    pub CR: uint32_t,
    pub SWTRIGR: uint32_t,
    pub DHR12R1: uint32_t,
    pub DHR12L1: uint32_t,
    pub DHR8R1: uint32_t,
    pub DHR12R2: uint32_t,
    pub DHR12L2: uint32_t,
    pub DHR8R2: uint32_t,
    pub DHR12RD: uint32_t,
    pub DHR12LD: uint32_t,
    pub DHR8RD: uint32_t,
    pub DOR1: uint32_t,
    pub DOR2: uint32_t,
    pub SR: uint32_t,
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
  * @file    stm32f7xx_hal_dac.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DAC HAL module.
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
/* * @addtogroup DAC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup DAC_Exported_Types DAC Exported Types
  * @{
  */
/* * 
  * @brief HAL State structures definition
  */
pub type HAL_DAC_StateTypeDef = libc::c_uint;
/* !< DAC error state                      */
/* !< DAC timeout state                    */
pub const HAL_DAC_STATE_ERROR: HAL_DAC_StateTypeDef = 4;
/* !< DAC internal processing is ongoing   */
pub const HAL_DAC_STATE_TIMEOUT: HAL_DAC_StateTypeDef = 3;
/* !< DAC initialized and ready for use    */
pub const HAL_DAC_STATE_BUSY: HAL_DAC_StateTypeDef = 2;
/* !< DAC not yet initialized or disabled  */
pub const HAL_DAC_STATE_READY: HAL_DAC_StateTypeDef = 1;
pub const HAL_DAC_STATE_RESET: HAL_DAC_StateTypeDef = 0;
/* * 
  * @brief DAC handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DAC_HandleTypeDef {
    pub Instance: *mut DAC_TypeDef,
    pub State: HAL_DAC_StateTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub DMA_Handle1: *mut DMA_HandleTypeDef,
    pub DMA_Handle2: *mut DMA_HandleTypeDef,
    pub ErrorCode: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_dac_ex.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Extended DAC HAL module driver.
  *         This file provides firmware functions to manage the following 
  *         functionalities of DAC extension peripheral:
  *           + Extended features functions
  *     
  *
  @verbatim      
  ==============================================================================
                      ##### How to use this driver #####
  ==============================================================================
    [..]          
      (+) When Dual mode is enabled (i.e DAC Channel1 and Channel2 are used simultaneously) :
          Use HAL_DACEx_DualGetValue() to get digital data to be converted and use
          HAL_DACEx_DualSetValue() to set digital value to converted simultaneously in Channel 1 and Channel 2.  
      (+) Use HAL_DACEx_TriangleWaveGenerate() to generate Triangle signal.
      (+) Use HAL_DACEx_NoiseWaveGenerate() to generate Noise signal.
   
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
/* * @defgroup DACEx DACEx
  * @brief DAC driver modules
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @defgroup DACEx_Exported_Functions DAC Exported Functions
  * @{
  */
/* * @defgroup DACEx_Exported_Functions_Group1 Extended features functions
 *  @brief    Extended features functions 
 *
@verbatim   
  ==============================================================================
                 ##### Extended features functions #####
  ==============================================================================  
    [..]  This section provides functions allowing to:
      (+) Start conversion.
      (+) Stop conversion.
      (+) Start conversion and enable DMA transfer.
      (+) Stop conversion and disable DMA transfer.
      (+) Get result of conversion.
      (+) Get result of dual mode conversion.
                     
@endverbatim
  * @{
  */
/* *
  * @brief  Returns the last data output value of the selected DAC channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval The selected DAC channel data output value.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DACEx_DualGetValue(mut hdac:
                                                    *mut DAC_HandleTypeDef)
 -> uint32_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    tmp |= (*(*hdac).Instance).DOR1;
    tmp |= (*(*hdac).Instance).DOR2 << 16 as libc::c_int;
    /* Returns the DAC channel data output register value */
    return tmp;
}
/* *
  * @brief  Enables or disables the selected DAC channel wave generation.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected 
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @param  Amplitude: Select max triangle amplitude. 
  *          This parameter can be one of the following values:
  *            @arg DAC_TRIANGLEAMPLITUDE_1: Select max triangle amplitude of 1
  *            @arg DAC_TRIANGLEAMPLITUDE_3: Select max triangle amplitude of 3
  *            @arg DAC_TRIANGLEAMPLITUDE_7: Select max triangle amplitude of 7
  *            @arg DAC_TRIANGLEAMPLITUDE_15: Select max triangle amplitude of 15
  *            @arg DAC_TRIANGLEAMPLITUDE_31: Select max triangle amplitude of 31
  *            @arg DAC_TRIANGLEAMPLITUDE_63: Select max triangle amplitude of 63
  *            @arg DAC_TRIANGLEAMPLITUDE_127: Select max triangle amplitude of 127
  *            @arg DAC_TRIANGLEAMPLITUDE_255: Select max triangle amplitude of 255
  *            @arg DAC_TRIANGLEAMPLITUDE_511: Select max triangle amplitude of 511
  *            @arg DAC_TRIANGLEAMPLITUDE_1023: Select max triangle amplitude of 1023
  *            @arg DAC_TRIANGLEAMPLITUDE_2047: Select max triangle amplitude of 2047
  *            @arg DAC_TRIANGLEAMPLITUDE_4095: Select max triangle amplitude of 4095                               
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DACEx_TriangleWaveGenerate(mut hdac:
                                                            *mut DAC_HandleTypeDef,
                                                        mut Channel: uint32_t,
                                                        mut Amplitude:
                                                            uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Process locked */
    if (*hdac).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hdac).Lock = HAL_LOCKED }
    /* Change DAC state */
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_BUSY);
    /* Enable the selected wave generation for the selected DAC channel */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                (*(*hdac).Instance).CR &
                                    !(((0x3 as libc::c_uint) <<
                                           6 as libc::c_uint |
                                           (0xf as libc::c_uint) <<
                                               8 as libc::c_uint) << Channel)
                                    |
                                    ((0x2 as libc::c_uint) <<
                                         6 as libc::c_uint | Amplitude) <<
                                        Channel);
    /* Change DAC state */
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_READY);
    /* Process unlocked */
    (*hdac).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Enables or disables the selected DAC channel wave generation.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC. 
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected 
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @param  Amplitude: Unmask DAC channel LFSR for noise wave generation. 
  *          This parameter can be one of the following values: 
  *            @arg DAC_LFSRUNMASK_BIT0: Unmask DAC channel LFSR bit0 for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS1_0: Unmask DAC channel LFSR bit[1:0] for noise wave generation  
  *            @arg DAC_LFSRUNMASK_BITS2_0: Unmask DAC channel LFSR bit[2:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS3_0: Unmask DAC channel LFSR bit[3:0] for noise wave generation 
  *            @arg DAC_LFSRUNMASK_BITS4_0: Unmask DAC channel LFSR bit[4:0] for noise wave generation 
  *            @arg DAC_LFSRUNMASK_BITS5_0: Unmask DAC channel LFSR bit[5:0] for noise wave generation 
  *            @arg DAC_LFSRUNMASK_BITS6_0: Unmask DAC channel LFSR bit[6:0] for noise wave generation 
  *            @arg DAC_LFSRUNMASK_BITS7_0: Unmask DAC channel LFSR bit[7:0] for noise wave generation 
  *            @arg DAC_LFSRUNMASK_BITS8_0: Unmask DAC channel LFSR bit[8:0] for noise wave generation 
  *            @arg DAC_LFSRUNMASK_BITS9_0: Unmask DAC channel LFSR bit[9:0] for noise wave generation 
  *            @arg DAC_LFSRUNMASK_BITS10_0: Unmask DAC channel LFSR bit[10:0] for noise wave generation 
  *            @arg DAC_LFSRUNMASK_BITS11_0: Unmask DAC channel LFSR bit[11:0] for noise wave generation 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DACEx_NoiseWaveGenerate(mut hdac:
                                                         *mut DAC_HandleTypeDef,
                                                     mut Channel: uint32_t,
                                                     mut Amplitude: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Process locked */
    if (*hdac).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hdac).Lock = HAL_LOCKED }
    /* Change DAC state */
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_BUSY);
    /* Enable the selected wave generation for the selected DAC channel */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                (*(*hdac).Instance).CR &
                                    !(((0x3 as libc::c_uint) <<
                                           6 as libc::c_uint |
                                           (0xf as libc::c_uint) <<
                                               8 as libc::c_uint) << Channel)
                                    |
                                    ((0x1 as libc::c_uint) <<
                                         6 as libc::c_uint | Amplitude) <<
                                        Channel);
    /* Change DAC state */
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_READY);
    /* Process unlocked */
    (*hdac).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Set the specified data holding register value for dual DAC channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *               the configuration information for the specified DAC.
  * @param  Alignment: Specifies the data alignment for dual channel DAC.
  *          This parameter can be one of the following values:
  *            DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @param  Data1: Data for DAC Channel2 to be loaded in the selected data holding register.
  * @param  Data2: Data for DAC Channel1 to be loaded in the selected data  holding register.
  * @note   In dual mode, a unique register access is required to write in both
  *          DAC channels at the same time.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DACEx_DualSetValue(mut hdac:
                                                    *mut DAC_HandleTypeDef,
                                                mut Alignment: uint32_t,
                                                mut Data1: uint32_t,
                                                mut Data2: uint32_t)
 -> HAL_StatusTypeDef {
    let mut data: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Calculate and set dual DAC data holding register value */
    if Alignment == 0x8 as libc::c_uint {
        data = Data2 << 8 as libc::c_int | Data1
    } else { data = Data2 << 16 as libc::c_int | Data1 }
    tmp = (*hdac).Instance as uint32_t;
    tmp =
        (tmp as
             libc::c_uint).wrapping_add((0x20 as
                                             libc::c_uint).wrapping_add(Alignment))
            as uint32_t as uint32_t;
    /* Set the dual DAC selected data holding register */
    ::core::ptr::write_volatile(tmp as *mut uint32_t, data);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* *
  * @brief  Conversion complete callback in non blocking mode for Channel2 
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DACEx_ConvCpltCallbackCh2(mut hdac:
                                                           *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_ConvCpltCallbackCh2 could be implemented in the user file
   */
}
/* *
  * @brief  Conversion half DMA transfer callback in non blocking mode for Channel2 
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DACEx_ConvHalfCpltCallbackCh2(mut hdac:
                                                               *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DACEx_ConvHalfCpltCallbackCh2 could be implemented in the user file
   */
}
/* *
  * @brief  Error DAC callback for Channel2.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DACEx_ErrorCallbackCh2(mut hdac:
                                                        *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DACEx_ErrorCallbackCh2 could be implemented in the user file
   */
}
/* *
  * @brief  DMA underrun DAC callback for channel2.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DACEx_DMAUnderrunCallbackCh2(mut hdac:
                                                              *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DACEx_DMAUnderrunCallbackCh2 could be implemented in the user file
   */
}
/* *
  * @brief  DMA conversion complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DMAConvCpltCh2(mut hdma:
                                                *mut DMA_HandleTypeDef) {
    let mut hdac: *mut DAC_HandleTypeDef =
        (*hdma).Parent as *mut DAC_HandleTypeDef;
    HAL_DACEx_ConvCpltCallbackCh2(hdac);
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_READY);
}
/* *
  * @brief  DMA half transfer complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DMAHalfConvCpltCh2(mut hdma:
                                                    *mut DMA_HandleTypeDef) {
    let mut hdac: *mut DAC_HandleTypeDef =
        (*hdma).Parent as *mut DAC_HandleTypeDef;
    /* Conversion complete callback */
    HAL_DACEx_ConvHalfCpltCallbackCh2(hdac);
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_dac.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DAC HAL Extension module.
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
/* * @addtogroup DACEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup DACEx_Exported_Constants DAC Exported Constants
  * @{
  */
/* * @defgroup DACEx_lfsrunmask_triangleamplitude DAC LFS Run Mask Triangle Amplitude
  * @{
  */
/* !< Unmask DAC channel LFSR bit0 for noise wave generation */
/* !< Unmask DAC channel LFSR bit[1:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[2:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[3:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[4:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[5:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[6:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[7:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[8:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[9:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[10:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[11:0] for noise wave generation */
/* !< Select max triangle amplitude of 1 */
/* !< Select max triangle amplitude of 3 */
/* !< Select max triangle amplitude of 7 */
/* !< Select max triangle amplitude of 15 */
/* !< Select max triangle amplitude of 31 */
/* !< Select max triangle amplitude of 63 */
/* !< Select max triangle amplitude of 127 */
/* !< Select max triangle amplitude of 255 */
/* !< Select max triangle amplitude of 511 */
/* !< Select max triangle amplitude of 1023 */
/* !< Select max triangle amplitude of 2047 */
/* !< Select max triangle amplitude of 4095 */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup DACEx_Exported_Functions
  * @{
  */
/* * @addtogroup DACEx_Exported_Functions_Group1
  * @{
  */
/* Extension features functions ***********************************************/
/* *
  * @}
  */
/* *
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* * @defgroup DACEx_Private_Constants DAC Private Constants
  * @{
  */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @defgroup DACEx_Private_Macros DAC Private Macros
  * @{
  */
/* *
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/* * @defgroup DACEx_Private_Functions DAC Private Functions
  * @{
  */
/* *
  * @brief  DMA error callback 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DMAErrorCh2(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hdac: *mut DAC_HandleTypeDef =
        (*hdma).Parent as *mut DAC_HandleTypeDef;
    /* Set DAC error code to DMA error */
    ::core::ptr::write_volatile(&mut (*hdac).ErrorCode as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hdac).ErrorCode
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x4 as libc::c_uint) as
                                    uint32_t as uint32_t);
    HAL_DACEx_ErrorCallbackCh2(hdac);
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_READY);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_DAC_MODULE_ENABLED */
/* *
  * @}
  */
