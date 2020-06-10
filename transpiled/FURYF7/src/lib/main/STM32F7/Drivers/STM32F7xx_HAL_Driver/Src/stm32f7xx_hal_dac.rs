use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DMA_Abort(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DACEx_DMAUnderrunCallbackCh2(hdac: *mut DAC_HandleTypeDef);
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
    #[no_mangle]
    fn DAC_DMAConvCpltCh2(hdma: *mut DMA_HandleTypeDef);
    #[no_mangle]
    fn DAC_DMAErrorCh2(hdma: *mut DMA_HandleTypeDef);
    #[no_mangle]
    fn DAC_DMAHalfConvCpltCh2(hdma: *mut DMA_HandleTypeDef);
}
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
  * @brief DAC Configuration regular Channel structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DAC_ChannelConfTypeDef {
    pub DAC_Trigger: uint32_t,
    pub DAC_OutputBuffer: uint32_t,
}
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup DAC_Exported_Functions DAC Exported Functions
  * @{
  */
/* * @defgroup DAC_Exported_Functions_Group1 Initialization and de-initialization functions 
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DAC. 
      (+) De-initialize the DAC. 
         
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the DAC peripheral according to the specified parameters
  *         in the DAC_InitStruct.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_Init(mut hdac: *mut DAC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check DAC handle */
    if hdac.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*hdac).State as libc::c_uint ==
           HAL_DAC_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*hdac).Lock = HAL_UNLOCKED;
        /* Init the low level hardware */
        HAL_DAC_MspInit(hdac);
    }
    /* Initialize the DAC state*/
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_BUSY);
    /* Set DAC error code to none */
    ::core::ptr::write_volatile(&mut (*hdac).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    /* Initialize the DAC state*/
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Deinitializes the DAC peripheral registers to their default reset values.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_DeInit(mut hdac: *mut DAC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check DAC handle */
    if hdac.is_null() { return HAL_ERROR }
    /* Check the parameters */
    /* Change DAC state */
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_BUSY);
    /* DeInit the low level hardware */
    HAL_DAC_MspDeInit(hdac);
    /* Set DAC error code to none */
    ::core::ptr::write_volatile(&mut (*hdac).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    /* Change DAC state */
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_RESET);
    /* Release Lock */
    (*hdac).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Initializes the DAC MSP.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_MspInit(mut hdac: *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes the DAC MSP.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_MspDeInit(mut hdac: *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_MspDeInit could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup DAC_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions 
 *
@verbatim   
  ==============================================================================
             ##### IO operation functions #####
  ==============================================================================  
    [..]  This section provides functions allowing to:
      (+) Start conversion.
      (+) Stop conversion.
      (+) Start conversion and enable DMA transfer.
      (+) Stop conversion and disable DMA transfer.
      (+) Get result of conversion.
                     
@endverbatim
  * @{
  */
/* *
  * @brief  Enables DAC and starts conversion of channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_Start(mut hdac: *mut DAC_HandleTypeDef,
                                       mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmp1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp2: uint32_t = 0 as libc::c_int as uint32_t;
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
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          0 as libc::c_uint) << Channel) as
                                    uint32_t as uint32_t);
    if Channel == 0 as libc::c_uint {
        tmp1 =
            (*(*hdac).Instance).CR &
                (0x1 as libc::c_uint) << 2 as libc::c_uint;
        tmp2 =
            (*(*hdac).Instance).CR &
                (0x7 as libc::c_uint) << 3 as libc::c_uint;
        /* Check if software trigger enabled */
        if tmp1 == (0x1 as libc::c_uint) << 2 as libc::c_uint &&
               tmp2 == (0x7 as libc::c_uint) << 3 as libc::c_uint {
            /* Enable the selected DAC software conversion */
            ::core::ptr::write_volatile(&mut (*(*hdac).Instance).SWTRIGR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).SWTRIGR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    } else {
        tmp1 =
            (*(*hdac).Instance).CR &
                (0x1 as libc::c_uint) << 18 as libc::c_uint;
        tmp2 =
            (*(*hdac).Instance).CR &
                (0x7 as libc::c_uint) << 19 as libc::c_uint;
        /* Check if software trigger enabled */
        if tmp1 == (0x1 as libc::c_uint) << 18 as libc::c_uint &&
               tmp2 == (0x7 as libc::c_uint) << 19 as libc::c_uint {
            /* Enable the selected DAC software conversion*/
            ::core::ptr::write_volatile(&mut (*(*hdac).Instance).SWTRIGR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).SWTRIGR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 1 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
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
  * @brief  Disables DAC and stop conversion of channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_Stop(mut hdac: *mut DAC_HandleTypeDef,
                                      mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((0x1 as libc::c_uint) <<
                                            0 as libc::c_uint) << Channel)) as
                                    uint32_t as uint32_t);
    /* Change DAC state */
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Enables DAC and starts conversion of channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @param  pData: The Source memory Buffer address.
  * @param  Length: The length of data to be transferred from memory to DAC peripheral
  * @param  Alignment: Specifies the data alignment for DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_Start_DMA(mut hdac: *mut DAC_HandleTypeDef,
                                           mut Channel: uint32_t,
                                           mut pData: *mut uint32_t,
                                           mut Length: uint32_t,
                                           mut Alignment: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
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
    if Channel == 0 as libc::c_uint {
        /* Set the DMA transfer complete callback for channel1 */
        (*(*hdac).DMA_Handle1).XferCpltCallback =
            Some(DAC_DMAConvCpltCh1 as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA half transfer complete callback for channel1 */
        (*(*hdac).DMA_Handle1).XferHalfCpltCallback =
            Some(DAC_DMAHalfConvCpltCh1 as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback for channel1 */
        (*(*hdac).DMA_Handle1).XferErrorCallback =
            Some(DAC_DMAErrorCh1 as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Enable the selected DAC channel1 DMA request */
        ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             12 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Case of use of channel 1 */
        match Alignment {
            0 => {
                /* Get DHR12R1 address */
                tmpreg =
                    &mut (*(*hdac).Instance).DHR12R1 as *mut uint32_t as
                        uint32_t
            }
            4 => {
                /* Get DHR12L1 address */
                tmpreg =
                    &mut (*(*hdac).Instance).DHR12L1 as *mut uint32_t as
                        uint32_t
            }
            8 => {
                /* Get DHR8R1 address */
                tmpreg =
                    &mut (*(*hdac).Instance).DHR8R1 as *mut uint32_t as
                        uint32_t
            }
            _ => { }
        }
    } else {
        /* Set the DMA transfer complete callback for channel2 */
        (*(*hdac).DMA_Handle2).XferCpltCallback =
            Some(DAC_DMAConvCpltCh2 as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA half transfer complete callback for channel2 */
        (*(*hdac).DMA_Handle2).XferHalfCpltCallback =
            Some(DAC_DMAHalfConvCpltCh2 as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback for channel2 */
        (*(*hdac).DMA_Handle2).XferErrorCallback =
            Some(DAC_DMAErrorCh2 as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Enable the selected DAC channel2 DMA request */
        ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             28 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Case of use of channel 2 */
        match Alignment {
            0 => {
                /* Get DHR12R2 address */
                tmpreg =
                    &mut (*(*hdac).Instance).DHR12R2 as *mut uint32_t as
                        uint32_t
            }
            4 => {
                /* Get DHR12L2 address */
                tmpreg =
                    &mut (*(*hdac).Instance).DHR12L2 as *mut uint32_t as
                        uint32_t
            }
            8 => {
                /* Get DHR8R2 address */
                tmpreg =
                    &mut (*(*hdac).Instance).DHR8R2 as *mut uint32_t as
                        uint32_t
            }
            _ => { }
        }
    }
    /* Enable the DMA Stream */
    if Channel == 0 as libc::c_uint {
        /* Enable the DAC DMA underrun interrupt */
        ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             13 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Enable the DMA Stream */
        HAL_DMA_Start_IT((*hdac).DMA_Handle1, pData as uint32_t, tmpreg,
                         Length);
    } else {
        /* Enable the DAC DMA underrun interrupt */
        ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             29 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Enable the DMA Stream */
        HAL_DMA_Start_IT((*hdac).DMA_Handle2, pData as uint32_t, tmpreg,
                         Length);
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          0 as libc::c_uint) << Channel) as
                                    uint32_t as uint32_t);
    /* Process Unlocked */
    (*hdac).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Disables DAC and stop conversion of channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected   
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_Stop_DMA(mut hdac: *mut DAC_HandleTypeDef,
                                          mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Disable the selected DAC channel DMA request */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((0x1 as libc::c_uint) <<
                                            12 as libc::c_uint) << Channel))
                                    as uint32_t as uint32_t);
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((0x1 as libc::c_uint) <<
                                            0 as libc::c_uint) << Channel)) as
                                    uint32_t as uint32_t);
    /* Disable the DMA Channel */
  /* Channel1 is used */
    if Channel == 0 as libc::c_uint {
        status = HAL_DMA_Abort((*hdac).DMA_Handle1)
    } else {
        /* Channel2 is used for */
        status = HAL_DMA_Abort((*hdac).DMA_Handle2)
    }
    /* Check if DMA Channel effectively disabled */
    if status as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        /* Update DAC state machine to error */
        ::core::ptr::write_volatile(&mut (*hdac).State as
                                        *mut HAL_DAC_StateTypeDef,
                                    HAL_DAC_STATE_ERROR)
    } else {
        /* Change DAC state */
        ::core::ptr::write_volatile(&mut (*hdac).State as
                                        *mut HAL_DAC_StateTypeDef,
                                    HAL_DAC_STATE_READY)
    }
    /* Return function status */
    return status;
}
/* *
  * @brief  Returns the last data output value of the selected DAC channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @retval The selected DAC channel data output value.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_GetValue(mut hdac: *mut DAC_HandleTypeDef,
                                          mut Channel: uint32_t) -> uint32_t {
    /* Check the parameters */
    /* Returns the DAC channel data output register value */
    if Channel == 0 as libc::c_uint {
        return (*(*hdac).Instance).DOR1
    } else { return (*(*hdac).Instance).DOR2 };
}
/* *
  * @brief  Handles DAC interrupt request  
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_IRQHandler(mut hdac:
                                                *mut DAC_HandleTypeDef) {
    /* Check underrun channel 1 flag */
    if (*(*hdac).Instance).SR & (0x1 as libc::c_uint) << 13 as libc::c_uint ==
           (0x1 as libc::c_uint) << 13 as libc::c_uint {
        /* Change DAC state to error state */
        ::core::ptr::write_volatile(&mut (*hdac).State as
                                        *mut HAL_DAC_StateTypeDef,
                                    HAL_DAC_STATE_ERROR);
        /* Set DAC error code to channel1 DMA underrun error */
        ::core::ptr::write_volatile(&mut (*hdac).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hdac).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Clear the underrun flag */
        ::core::ptr::write_volatile(&mut (*(*hdac).Instance).SR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        13 as libc::c_uint);
        /* Disable the selected DAC channel1 DMA request */
        ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               12 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Error callback */
        HAL_DAC_DMAUnderrunCallbackCh1(hdac);
    }
    /* Check underrun channel 2 flag */
    if (*(*hdac).Instance).SR & (0x1 as libc::c_uint) << 29 as libc::c_uint ==
           (0x1 as libc::c_uint) << 29 as libc::c_uint {
        /* Change DAC state to error state */
        ::core::ptr::write_volatile(&mut (*hdac).State as
                                        *mut HAL_DAC_StateTypeDef,
                                    HAL_DAC_STATE_ERROR);
        /* Set DAC error code to channel2 DMA underrun error */
        ::core::ptr::write_volatile(&mut (*hdac).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hdac).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Clear the underrun flag */
        ::core::ptr::write_volatile(&mut (*(*hdac).Instance).SR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        29 as libc::c_uint);
        /* Disable the selected DAC channel1 DMA request */
        ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               28 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Error callback */
        HAL_DACEx_DMAUnderrunCallbackCh2(hdac);
    };
}
/* *
  * @brief  Conversion complete callback in non blocking mode for Channel1 
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_ConvCpltCallbackCh1(mut hdac:
                                                         *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_ConvCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  Conversion half DMA transfer callback in non blocking mode for Channel1 
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_ConvHalfCpltCallbackCh1(mut hdac:
                                                             *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_ConvHalfCpltCallbackCh1 could be implemented in the user file
   */
}
/* *
  * @brief  Error DAC callback for Channel1.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_ErrorCallbackCh1(mut hdac:
                                                      *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_ErrorCallbackCh1 could be implemented in the user file
   */
}
/* *
  * @brief  DMA underrun DAC callback for channel1.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_DMAUnderrunCallbackCh1(mut hdac:
                                                            *mut DAC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_DMAUnderrunCallbackCh1 could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup DAC_Exported_Functions_Group3 Peripheral Control functions
 *  @brief   	Peripheral Control functions 
 *
@verbatim   
  ==============================================================================
             ##### Peripheral Control functions #####
  ==============================================================================  
    [..]  This section provides functions allowing to:
      (+) Configure channels. 
      (+) Set the specified data holding register value for DAC channel.
      
@endverbatim
  * @{
  */
/* *
  * @brief  Configures the selected DAC channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  sConfig: DAC configuration structure.
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_ConfigChannel(mut hdac:
                                                   *mut DAC_HandleTypeDef,
                                               mut sConfig:
                                                   *mut DAC_ChannelConfTypeDef,
                                               mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the DAC parameters */
    /* Process locked */
    if (*hdac).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hdac).Lock = HAL_LOCKED }
    /* Change DAC state */
    ::core::ptr::write_volatile(&mut (*hdac).State as
                                    *mut HAL_DAC_StateTypeDef,
                                HAL_DAC_STATE_BUSY);
    /* Get the DAC CR value */
    tmpreg1 = (*(*hdac).Instance).CR;
    /* Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits */
    tmpreg1 &=
        !(((0xf as libc::c_uint) << 8 as libc::c_uint |
               (0x3 as libc::c_uint) << 6 as libc::c_uint |
               (0x7 as libc::c_uint) << 3 as libc::c_uint |
               (0x1 as libc::c_uint) << 2 as libc::c_uint |
               (0x1 as libc::c_uint) << 1 as libc::c_uint) << Channel);
    /* Configure for the selected DAC channel: buffer output, trigger */
  /* Set TSELx and TENx bits according to DAC_Trigger value */
  /* Set BOFFx bit according to DAC_OutputBuffer value */
    tmpreg2 = (*sConfig).DAC_Trigger | (*sConfig).DAC_OutputBuffer;
    /* Calculate CR register value depending on DAC_Channel */
    tmpreg1 |= tmpreg2 << Channel;
    /* Write to DAC CR */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                tmpreg1);
    /* Disable wave generation */
    ::core::ptr::write_volatile(&mut (*(*hdac).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdac).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(((0x3 as libc::c_uint) <<
                                            6 as libc::c_uint) << Channel)) as
                                    uint32_t as uint32_t);
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
  * @brief  Set the specified data holding register value for DAC channel.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected  
  * @param  Alignment: Specifies the data alignment.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @param  Data: Data to be loaded in the selected data holding register.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_SetValue(mut hdac: *mut DAC_HandleTypeDef,
                                          mut Channel: uint32_t,
                                          mut Alignment: uint32_t,
                                          mut Data: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (*hdac).Instance as uint32_t);
    if Channel == 0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                                as
                                                                                *const uint32_t)
                                         as
                                         libc::c_uint).wrapping_add((0x8 as
                                                                         libc::c_uint).wrapping_add(Alignment))
                                        as uint32_t as uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                                as
                                                                                *const uint32_t)
                                         as
                                         libc::c_uint).wrapping_add((0x14 as
                                                                         libc::c_uint).wrapping_add(Alignment))
                                        as uint32_t as uint32_t)
    }
    /* Set the DAC channel1 selected data holding register */
    ::core::ptr::write_volatile(tmp as *mut uint32_t, Data);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup DAC_Exported_Functions_Group4 Peripheral State and Errors functions
 *  @brief   Peripheral State and Errors functions 
 *
@verbatim   
  ==============================================================================
            ##### Peripheral State and Errors functions #####
  ==============================================================================  
    [..]
    This subsection provides functions allowing to
      (+) Check the DAC state.
      (+) Check the DAC Errors.
        
@endverbatim
  * @{
  */
/* *
  * @brief  return the DAC state
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_GetState(mut hdac: *mut DAC_HandleTypeDef)
 -> HAL_DAC_StateTypeDef {
    /* Return DAC state */
    return (*hdac).State;
}
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup DAC_Exported_Constants DAC Exported Constants
  * @{
  */
/* * @defgroup DAC_Error_Code DAC Error Code
  * @{
  */
/* !< No error                          */
/* !< DAC channel1 DAM underrun error   */
/* !< DAC channel2 DAM underrun error   */
/* !< DMA error                         */
/* *
  * @}
  */
/* * @defgroup DAC_trigger_selection DAC Trigger Selection
  * @{
  */
/* !< Conversion is automatic once the DAC1_DHRxxxx register 
                                                                       has been loaded, and not by external trigger */
/* !< TIM2 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM4 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM5 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM6 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM7 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM8 TRGO selected as external conversion trigger for DAC channel */
/* !< EXTI Line9 event selected as external conversion trigger for DAC channel */
/* !< Conversion started by software trigger for DAC channel */
/* *
  * @}
  */
/* * @defgroup DAC_output_buffer  DAC Output Buffer
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_Channel_selection DAC Channel Selection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_data_alignment DAC Data Alignment
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_flags_definition DAC Flags Definition
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_IT_definition DAC IT Definition
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup DAC_Exported_Macros DAC Exported Macros
  * @{
  */
/* * @brief Reset DAC handle state
  * @param  __HANDLE__: specifies the DAC handle.
  * @retval None
  */
/* * @brief Enable the DAC channel
  * @param  __HANDLE__: specifies the DAC handle.
  * @param  __DAC_CHANNEL__: specifies the DAC channel
  * @retval None
  */
/* * @brief Disable the DAC channel
  * @param  __HANDLE__: specifies the DAC handle
  * @param  __DAC_CHANNEL__: specifies the DAC channel.
  * @retval None
  */
/* * @brief Enable the DAC interrupt
  * @param  __HANDLE__: specifies the DAC handle
  * @param  __INTERRUPT__: specifies the DAC interrupt.
  * @retval None
  */
/* * @brief Disable the DAC interrupt
  * @param  __HANDLE__: specifies the DAC handle
  * @param  __INTERRUPT__: specifies the DAC interrupt.
  * @retval None
  */
/* * @brief  Checks if the specified DAC interrupt source is enabled or disabled.
  * @param __HANDLE__: DAC handle
  * @param __INTERRUPT__: DAC interrupt source to check
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1: DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2: DAC channel 2 DMA underrun interrupt
  * @retval State of interruption (SET or RESET)
  */
/* * @brief  Get the selected DAC's flag status.
  * @param  __HANDLE__: specifies the DAC handle.
  * @param  __FLAG__: specifies the flag to clear.
  *         This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1: DMA underrun 1 flag
  *            @arg DAC_FLAG_DMAUDR2: DMA underrun 2 flag
  * @retval None
  */
/* * @brief  Clear the DAC's flag.
  * @param  __HANDLE__: specifies the DAC handle.
  * @param  __FLAG__: specifies the flag to clear.
  *         This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1: DMA underrun 1 flag
  *            @arg DAC_FLAG_DMAUDR2: DMA underrun 2 flag
  * @retval None
  */
/* *
  * @}
  */
/* Include DAC HAL Extension module */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup DAC_Exported_Functions
  * @{
  */
/* * @addtogroup DAC_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions *********************************/
/* *
  * @}
  */
/* * @addtogroup DAC_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ****************************************************/
/* *
  * @}
  */
/* * @addtogroup DAC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions ***********************************************/
/* *
  * @}
  */
/* * @addtogroup DAC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State functions *************************************************/
/* *
  * @brief  Return the DAC error code
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval DAC Error Code
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DAC_GetError(mut hdac: *mut DAC_HandleTypeDef)
 -> uint32_t {
    return (*hdac).ErrorCode;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_dac.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   DAC HAL module driver.
  *         This file provides firmware functions to manage the following 
  *         functionalities of the Digital to Analog Converter (DAC) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Errors functions      
  *     
  *
  @verbatim      
  ==============================================================================
                      ##### DAC Peripheral features #####
  ==============================================================================
    [..]        
      *** DAC Channels ***
      ====================  
    [..]  
    The device integrates two 12-bit Digital Analog Converters that can 
    be used independently or simultaneously (dual mode):
      (#) DAC channel1 with DAC_OUT1 (PA4) as output
      (#) DAC channel2 with DAC_OUT2 (PA5) as output
      
      *** DAC Triggers ***
      ====================
    [..]
    Digital to Analog conversion can be non-triggered using DAC_TRIGGER_NONE
    and DAC_OUT1/DAC_OUT2 is available once writing to DHRx register. 
    [..] 
    Digital to Analog conversion can be triggered by:
      (#) External event: EXTI Line 9 (any GPIOx_Pin9) using DAC_TRIGGER_EXT_IT9.
          The used pin (GPIOx_Pin9) must be configured in input mode.
  
      (#) Timers TRGO: TIM2, TIM4, TIM5, TIM6, TIM7 and TIM8 
          (DAC_TRIGGER_T2_TRGO, DAC_TRIGGER_T4_TRGO...)
  
      (#) Software using DAC_TRIGGER_SOFTWARE
  
      *** DAC Buffer mode feature ***
      =============================== 
      [..] 
      Each DAC channel integrates an output buffer that can be used to 
      reduce the output impedance, and to drive external loads directly
      without having to add an external operational amplifier.
      To enable, the output buffer use  
      sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
      [..]           
      (@) Refer to the device datasheet for more details about output 
          impedance value with and without output buffer.
            
       *** DAC wave generation feature ***
       =================================== 
       [..]     
       Both DAC channels can be used to generate
         (#) Noise wave using HAL_DACEx_NoiseWaveGenerate() 
         (#) Triangle wave using HAL_DACEx_TriangleWaveGenerate()
            
       *** DAC data format ***
       =======================
       [..]   
       The DAC data format can be:
         (#) 8-bit right alignment using DAC_ALIGN_8B_R
         (#) 12-bit left alignment using DAC_ALIGN_12B_L
         (#) 12-bit right alignment using DAC_ALIGN_12B_R
  
       *** DAC data value to voltage correspondence ***  
       ================================================ 
       [..] 
       The analog output voltage on each DAC channel pin is determined
       by the following equation: 
       DAC_OUTx = VREF+ * DOR / 4095
       with  DOR is the Data Output Register
          VEF+ is the input voltage reference (refer to the device datasheet)
        e.g. To set DAC_OUT1 to 0.7V, use
          Assuming that VREF+ = 3.3V, DAC_OUT1 = (3.3 * 868) / 4095 = 0.7V
  
       *** DMA requests  ***
       =====================
       [..]    
       A DMA1 request can be generated when an external trigger (but not
       a software trigger) occurs if DMA1 requests are enabled using
       HAL_DAC_Start_DMA()
       [..]
       DMA1 requests are mapped as following:
         (#) DAC channel1 : mapped on DMA1 Stream5 channel7 which must be 
             already configured
         (#) DAC channel2 : mapped on DMA1 Stream6 channel7 which must be 
             already configured
       
    -@- For Dual mode and specific signal (Triangle and noise) generation please 
        refer to Extension Features Driver description        
  
      
                      ##### How to use this driver #####
  ==============================================================================
    [..]          
      (+) DAC APB clock must be enabled to get write access to DAC
          registers using HAL_DAC_Init()
      (+) Configure DAC_OUTx (DAC_OUT1: PA4, DAC_OUT2: PA5) in analog mode.
      (+) Configure the DAC channel using HAL_DAC_ConfigChannel() function.
      (+) Enable the DAC channel using HAL_DAC_Start() or HAL_DAC_Start_DMA functions

     *** Polling mode IO operation ***
     =================================
     [..]    
       (+) Start the DAC peripheral using HAL_DAC_Start() 
       (+) To read the DAC last data output value, use the HAL_DAC_GetValue() function.
       (+) Stop the DAC peripheral using HAL_DAC_Stop()

	   
     *** DMA mode IO operation ***    
     ==============================
     [..]    
       (+) Start the DAC peripheral using HAL_DAC_Start_DMA(), at this stage the user specify the length 
           of data to be transferred at each end of conversion 
       (+) At The end of data transfer HAL_DAC_ConvCpltCallbackCh1()or HAL_DAC_ConvCpltCallbackCh2()  
           function is executed and user can add his own code by customization of function pointer 
           HAL_DAC_ConvCpltCallbackCh1 or HAL_DAC_ConvCpltCallbackCh2
       (+) In case of transfer Error, HAL_DAC_ErrorCallbackCh1() function is executed and user can 
            add his own code by customization of function pointer HAL_DAC_ErrorCallbackCh1
       (+) Stop the DAC peripheral using HAL_DAC_Stop_DMA()

                    
     *** DAC HAL driver macros list ***
     ============================================= 
     [..]
       Below the list of most used macros in DAC HAL driver.
       
      (+) __HAL_DAC_ENABLE : Enable the DAC peripheral
      (+) __HAL_DAC_DISABLE : Disable the DAC peripheral
      (+) __HAL_DAC_CLEAR_FLAG: Clear the DAC's pending flags
      (+) __HAL_DAC_GET_FLAG: Get the selected DAC's flag status
      
     [..]
      (@) You can refer to the DAC HAL driver header file for more useful macros  
   
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
/* * @defgroup DAC DAC
  * @brief DAC driver modules
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @addtogroup DAC_Private_Functions
  * @{
  */
/* Private function prototypes -----------------------------------------------*/
/* *
  * @}
  */
/* *
  * @brief  DMA conversion complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn DAC_DMAConvCpltCh1(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hdac: *mut DAC_HandleTypeDef =
        (*hdma).Parent as *mut DAC_HandleTypeDef;
    HAL_DAC_ConvCpltCallbackCh1(hdac);
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
unsafe extern "C" fn DAC_DMAHalfConvCpltCh1(mut hdma:
                                                *mut DMA_HandleTypeDef) {
    let mut hdac: *mut DAC_HandleTypeDef =
        (*hdma).Parent as *mut DAC_HandleTypeDef;
    /* Conversion complete callback */
    HAL_DAC_ConvHalfCpltCallbackCh1(hdac);
}
/* *
  * @brief  DMA error callback 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn DAC_DMAErrorCh1(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hdac: *mut DAC_HandleTypeDef =
        (*hdma).Parent as *mut DAC_HandleTypeDef;
    /* Set DAC error code to DMA error */
    ::core::ptr::write_volatile(&mut (*hdac).ErrorCode as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hdac).ErrorCode
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x4 as libc::c_uint) as
                                    uint32_t as uint32_t);
    HAL_DAC_ErrorCallbackCh1(hdac);
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
