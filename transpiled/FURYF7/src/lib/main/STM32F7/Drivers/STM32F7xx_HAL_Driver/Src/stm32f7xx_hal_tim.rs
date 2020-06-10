use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DMA_Abort(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup TIMEx_Exported_Functions_Group6
  * @{
  */ 
/* Extension Callback *********************************************************/
    #[no_mangle]
    fn HAL_TIMEx_CommutationCallback(htim: *mut TIM_HandleTypeDef);
    #[no_mangle]
    fn HAL_TIMEx_BreakCallback(htim: *mut TIM_HandleTypeDef);
    #[no_mangle]
    fn TIMEx_DMACommutationCplt(hdma: *mut DMA_HandleTypeDef);
}
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub struct TIM_Base_InitTypeDef {
    pub Prescaler: uint32_t,
    pub CounterMode: uint32_t,
    pub Period: uint32_t,
    pub ClockDivision: uint32_t,
    pub RepetitionCounter: uint32_t,
    pub AutoReloadPreload: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_OC_InitTypeDef {
    pub OCMode: uint32_t,
    pub Pulse: uint32_t,
    pub OCPolarity: uint32_t,
    pub OCNPolarity: uint32_t,
    pub OCFastMode: uint32_t,
    pub OCIdleState: uint32_t,
    pub OCNIdleState: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_OnePulse_InitTypeDef {
    pub OCMode: uint32_t,
    pub Pulse: uint32_t,
    pub OCPolarity: uint32_t,
    pub OCNPolarity: uint32_t,
    pub OCIdleState: uint32_t,
    pub OCNIdleState: uint32_t,
    pub ICPolarity: uint32_t,
    pub ICSelection: uint32_t,
    pub ICFilter: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_IC_InitTypeDef {
    pub ICPolarity: uint32_t,
    pub ICSelection: uint32_t,
    pub ICPrescaler: uint32_t,
    pub ICFilter: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_Encoder_InitTypeDef {
    pub EncoderMode: uint32_t,
    pub IC1Polarity: uint32_t,
    pub IC1Selection: uint32_t,
    pub IC1Prescaler: uint32_t,
    pub IC1Filter: uint32_t,
    pub IC2Polarity: uint32_t,
    pub IC2Selection: uint32_t,
    pub IC2Prescaler: uint32_t,
    pub IC2Filter: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_ClockConfigTypeDef {
    pub ClockSource: uint32_t,
    pub ClockPolarity: uint32_t,
    pub ClockPrescaler: uint32_t,
    pub ClockFilter: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_ClearInputConfigTypeDef {
    pub ClearInputState: uint32_t,
    pub ClearInputSource: uint32_t,
    pub ClearInputPolarity: uint32_t,
    pub ClearInputPrescaler: uint32_t,
    pub ClearInputFilter: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_SlaveConfigTypeDef {
    pub SlaveMode: uint32_t,
    pub InputTrigger: uint32_t,
    pub TriggerPolarity: uint32_t,
    pub TriggerPrescaler: uint32_t,
    pub TriggerFilter: uint32_t,
}
pub type HAL_TIM_StateTypeDef = libc::c_uint;
pub const HAL_TIM_STATE_ERROR: HAL_TIM_StateTypeDef = 4;
pub const HAL_TIM_STATE_TIMEOUT: HAL_TIM_StateTypeDef = 3;
pub const HAL_TIM_STATE_BUSY: HAL_TIM_StateTypeDef = 2;
pub const HAL_TIM_STATE_READY: HAL_TIM_StateTypeDef = 1;
pub const HAL_TIM_STATE_RESET: HAL_TIM_StateTypeDef = 0;
pub type HAL_TIM_ActiveChannel = libc::c_uint;
pub const HAL_TIM_ACTIVE_CHANNEL_CLEARED: HAL_TIM_ActiveChannel = 0;
pub const HAL_TIM_ACTIVE_CHANNEL_4: HAL_TIM_ActiveChannel = 8;
pub const HAL_TIM_ACTIVE_CHANNEL_3: HAL_TIM_ActiveChannel = 4;
pub const HAL_TIM_ACTIVE_CHANNEL_2: HAL_TIM_ActiveChannel = 2;
pub const HAL_TIM_ACTIVE_CHANNEL_1: HAL_TIM_ActiveChannel = 1;
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
/* !< DMA Stream Index                       */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup TIM_Exported_Functions TIM Exported Functions
  * @{
  */
/* * @defgroup TIM_Exported_Functions_Group1 Time Base functions 
 *  @brief    Time Base functions 
 *
@verbatim    
  ==============================================================================
              ##### Time Base functions #####
  ==============================================================================
  [..]  
    This section provides functions allowing to:
    (+) Initialize and configure the TIM base. 
    (+) De-initialize the TIM base.
    (+) Start the Time Base.
    (+) Stop the Time Base.
    (+) Start the Time Base and enable interrupt.
    (+) Stop the Time Base and disable interrupt.
    (+) Start the Time Base and enable DMA transfer.
    (+) Stop the Time Base and disable DMA transfer.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM Time base Unit according to the specified
  *         parameters in the TIM_HandleTypeDef and create the associated handle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_Init(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the TIM handle allocation */
    if htim.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*htim).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, NVIC */
        HAL_TIM_Base_MspInit(htim);
    }
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Set the Time Base configuration */
    TIM_Base_SetConfig((*htim).Instance, &mut (*htim).Init);
    /* Initialize the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the TIM Base peripheral 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_DeInit(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Disable the TIM Peripheral Clock */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
    HAL_TIM_Base_MspDeInit(htim);
    /* Change TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_RESET);
    /* Release Lock */
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM Base MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_MspInit(mut htim:
                                                  *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_Base_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes TIM Base MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_MspDeInit(mut htim:
                                                    *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_Base_MspDeInit could be implemented in the user file
   */
}
/* *
  * @brief  Starts the TIM Base generation.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_Start(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Change the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Base generation.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_Stop(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Change the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM Base generation in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_Start_IT(mut htim:
                                                   *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the TIM Update interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Base generation in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_Stop_IT(mut htim:
                                                  *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the TIM Update interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM Base generation in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  pData: The source Buffer address.
  * @param  Length: The length of data to be transferred from memory to peripheral.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_Start_DMA(mut htim:
                                                    *mut TIM_HandleTypeDef,
                                                mut pData: *mut uint32_t,
                                                mut Length: uint16_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_BUSY as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else {
        if (*htim).State as libc::c_uint ==
               HAL_TIM_STATE_READY as libc::c_int as libc::c_uint {
            if pData.is_null() && Length as libc::c_int > 0 as libc::c_int {
                return HAL_ERROR
            } else {
                ::core::ptr::write_volatile(&mut (*htim).State as
                                                *mut HAL_TIM_StateTypeDef,
                                            HAL_TIM_STATE_BUSY)
            }
        }
    }
    /* Set the DMA Period elapsed callback */
    (*(*htim).hdma[0 as libc::c_uint as uint16_t as usize]).XferCpltCallback =
        Some(TIM_DMAPeriodElapsedCplt as
                 unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
    /* Set the DMA error callback */
    (*(*htim).hdma[0 as libc::c_uint as uint16_t as usize]).XferErrorCallback
        =
        Some(TIM_DMAError as
                 unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
    /* Enable the DMA Stream */
    HAL_DMA_Start_IT((*htim).hdma[0 as libc::c_uint as uint16_t as usize],
                     pData as uint32_t,
                     &mut (*(*htim).Instance).ARR as *mut uint32_t as
                         uint32_t, Length as uint32_t);
    /* Enable the TIM Update DMA request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Base generation in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_Stop_DMA(mut htim:
                                                   *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the TIM Update DMA request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Change the htim state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group2 Time Output Compare functions 
 *  @brief    Time Output Compare functions 
 *
@verbatim    
  ==============================================================================
                  ##### Time Output Compare functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TIM Output Compare. 
    (+) De-initialize the TIM Output Compare.
    (+) Start the Time Output Compare.
    (+) Stop the Time Output Compare.
    (+) Start the Time Output Compare and enable interrupt.
    (+) Stop the Time Output Compare and disable interrupt.
    (+) Start the Time Output Compare and enable DMA transfer.
    (+) Stop the Time Output Compare and disable DMA transfer.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM Output Compare according to the specified
  *         parameters in the TIM_HandleTypeDef and create the associated handle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_Init(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the TIM handle allocation */
    if htim.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*htim).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
        HAL_TIM_OC_MspInit(htim);
    }
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Init the base time for the Output Compare */
    TIM_Base_SetConfig((*htim).Instance, &mut (*htim).Init);
    /* Initialize the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the TIM peripheral 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_DeInit(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Disable the TIM Peripheral Clock */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
    HAL_TIM_OC_MspDeInit(htim);
    /* Change TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_RESET);
    /* Release Lock */
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM Output Compare MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_MspInit(mut htim:
                                                *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_OC_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes TIM Output Compare MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_MspDeInit(mut htim:
                                                  *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_OC_MspDeInit could be implemented in the user file
   */
}
/* *
  * @brief  Starts the TIM Output Compare signal generation.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.  
  * @param  Channel: TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected   
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_Start(mut htim: *mut TIM_HandleTypeDef,
                                          mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the Output compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Enable the main output */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Output Compare signal generation.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channel to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_Stop(mut htim: *mut TIM_HandleTypeDef,
                                         mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Output compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Disable the Main Output */
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 8 as libc::c_uint |
                    (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            if (*(*htim).Instance).CCER &
                   ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                        (0x1 as libc::c_uint) << 6 as libc::c_uint |
                        (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM Output Compare signal generation in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_Start_IT(mut htim: *mut TIM_HandleTypeDef,
                                             mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Enable the TIM Capture/Compare 1 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 1 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Enable the TIM Capture/Compare 2 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 2 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Enable the TIM Capture/Compare 3 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Enable the TIM Capture/Compare 4 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 4 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Enable the Output compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Enable the main output */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Output Compare signal generation in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channel to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_Stop_IT(mut htim: *mut TIM_HandleTypeDef,
                                            mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Disable the TIM Capture/Compare 1 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   1 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Disable the TIM Capture/Compare 2 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Disable the TIM Capture/Compare 3 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   3 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Disable the TIM Capture/Compare 4 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   4 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Disable the Output compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Disable the Main Output */
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 8 as libc::c_uint |
                    (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            if (*(*htim).Instance).CCER &
                   ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                        (0x1 as libc::c_uint) << 6 as libc::c_uint |
                        (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM Output Compare signal generation in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  pData: The source Buffer address.
  * @param  Length: The length of data to be transferred from memory to TIM peripheral
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_Start_DMA(mut htim:
                                                  *mut TIM_HandleTypeDef,
                                              mut Channel: uint32_t,
                                              mut pData: *mut uint32_t,
                                              mut Length: uint16_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_BUSY as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else {
        if (*htim).State as libc::c_uint ==
               HAL_TIM_STATE_READY as libc::c_int as libc::c_uint {
            if pData as uint32_t == 0 as libc::c_int as libc::c_uint &&
                   Length as libc::c_int > 0 as libc::c_int {
                return HAL_ERROR
            } else {
                ::core::ptr::write_volatile(&mut (*htim).State as
                                                *mut HAL_TIM_StateTypeDef,
                                            HAL_TIM_STATE_BUSY)
            }
        }
    }
    match Channel {
        0 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                              usize], pData as uint32_t,
                             &mut (*(*htim).Instance).CCR1 as *mut uint32_t as
                                 uint32_t, Length as uint32_t);
            /* Enable the TIM Capture/Compare 1 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 9 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                              usize], pData as uint32_t,
                             &mut (*(*htim).Instance).CCR2 as *mut uint32_t as
                                 uint32_t, Length as uint32_t);
            /* Enable the TIM Capture/Compare 2 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 10 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x3 as libc::c_uint as uint16_t as
                                              usize], pData as uint32_t,
                             &mut (*(*htim).Instance).CCR3 as *mut uint32_t as
                                 uint32_t, Length as uint32_t);
            /* Enable the TIM Capture/Compare 3 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x4 as libc::c_uint as uint16_t as
                                              usize], pData as uint32_t,
                             &mut (*(*htim).Instance).CCR4 as *mut uint32_t as
                                 uint32_t, Length as uint32_t);
            /* Enable the TIM Capture/Compare 4 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Enable the Output compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Enable the main output */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Output Compare signal generation in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channel to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_Stop_DMA(mut htim: *mut TIM_HandleTypeDef,
                                             mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Disable the TIM Capture/Compare 1 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   9 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Disable the TIM Capture/Compare 2 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Disable the TIM Capture/Compare 3 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   11 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Disable the TIM Capture/Compare 4 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Disable the Output compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Disable the Main Output */
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 8 as libc::c_uint |
                    (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            if (*(*htim).Instance).CCER &
                   ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                        (0x1 as libc::c_uint) << 6 as libc::c_uint |
                        (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Change the htim state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group3 Time PWM functions 
 *  @brief    Time PWM functions 
 *
@verbatim    
  ==============================================================================
                          ##### Time PWM functions #####
  ==============================================================================
  [..]  
    This section provides functions allowing to:
    (+) Initialize and configure the TIM OPWM. 
    (+) De-initialize the TIM PWM.
    (+) Start the Time PWM.
    (+) Stop the Time PWM.
    (+) Start the Time PWM and enable interrupt.
    (+) Stop the Time PWM and disable interrupt.
    (+) Start the Time PWM and enable DMA transfer.
    (+) Stop the Time PWM and disable DMA transfer.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM PWM Time Base according to the specified
  *         parameters in the TIM_HandleTypeDef and create the associated handle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_Init(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the TIM handle allocation */
    if htim.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*htim).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
        HAL_TIM_PWM_MspInit(htim);
    }
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Init the base time for the PWM */
    TIM_Base_SetConfig((*htim).Instance, &mut (*htim).Init);
    /* Initialize the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the TIM peripheral 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_DeInit(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Disable the TIM Peripheral Clock */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
    HAL_TIM_PWM_MspDeInit(htim);
    /* Change TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_RESET);
    /* Release Lock */
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM PWM MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_MspInit(mut htim:
                                                 *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_PWM_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes TIM PWM MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_MspDeInit(mut htim:
                                                   *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_PWM_MspDeInit could be implemented in the user file
   */
}
/* *
  * @brief  Starts the PWM signal generation.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_Start(mut htim: *mut TIM_HandleTypeDef,
                                           mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Enable the main output */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the PWM signal generation.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_Stop(mut htim: *mut TIM_HandleTypeDef,
                                          mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Capture compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Disable the Main Output */
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 8 as libc::c_uint |
                    (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            if (*(*htim).Instance).CCER &
                   ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                        (0x1 as libc::c_uint) << 6 as libc::c_uint |
                        (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Change the htim state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the PWM signal generation in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_Start_IT(mut htim:
                                                  *mut TIM_HandleTypeDef,
                                              mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Enable the TIM Capture/Compare 1 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 1 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Enable the TIM Capture/Compare 2 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 2 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Enable the TIM Capture/Compare 3 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Enable the TIM Capture/Compare 4 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 4 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Enable the main output */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the PWM signal generation in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_Stop_IT(mut htim: *mut TIM_HandleTypeDef,
                                             mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Disable the TIM Capture/Compare 1 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   1 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Disable the TIM Capture/Compare 2 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Disable the TIM Capture/Compare 3 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   3 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Disable the TIM Capture/Compare 4 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   4 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Disable the Capture compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Disable the Main Output */
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 8 as libc::c_uint |
                    (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            if (*(*htim).Instance).CCER &
                   ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                        (0x1 as libc::c_uint) << 6 as libc::c_uint |
                        (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM PWM signal generation in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  pData: The source Buffer address.
  * @param  Length: The length of data to be transferred from memory to TIM peripheral
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_Start_DMA(mut htim:
                                                   *mut TIM_HandleTypeDef,
                                               mut Channel: uint32_t,
                                               mut pData: *mut uint32_t,
                                               mut Length: uint16_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_BUSY as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else {
        if (*htim).State as libc::c_uint ==
               HAL_TIM_STATE_READY as libc::c_int as libc::c_uint {
            if pData as uint32_t == 0 as libc::c_int as libc::c_uint &&
                   Length as libc::c_int > 0 as libc::c_int {
                return HAL_ERROR
            } else {
                ::core::ptr::write_volatile(&mut (*htim).State as
                                                *mut HAL_TIM_StateTypeDef,
                                            HAL_TIM_STATE_BUSY)
            }
        }
    }
    match Channel {
        0 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                              usize], pData as uint32_t,
                             &mut (*(*htim).Instance).CCR1 as *mut uint32_t as
                                 uint32_t, Length as uint32_t);
            /* Enable the TIM Capture/Compare 1 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 9 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                              usize], pData as uint32_t,
                             &mut (*(*htim).Instance).CCR2 as *mut uint32_t as
                                 uint32_t, Length as uint32_t);
            /* Enable the TIM Capture/Compare 2 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 10 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x3 as libc::c_uint as uint16_t as
                                              usize], pData as uint32_t,
                             &mut (*(*htim).Instance).CCR3 as *mut uint32_t as
                                 uint32_t, Length as uint32_t);
            /* Enable the TIM Output Capture/Compare 3 request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x4 as libc::c_uint as uint16_t as
                                              usize], pData as uint32_t,
                             &mut (*(*htim).Instance).CCR4 as *mut uint32_t as
                                 uint32_t, Length as uint32_t);
            /* Enable the TIM Capture/Compare 4 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Enable the main output */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM PWM signal generation in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_Stop_DMA(mut htim:
                                                  *mut TIM_HandleTypeDef,
                                              mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Disable the TIM Capture/Compare 1 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   9 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Disable the TIM Capture/Compare 2 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Disable the TIM Capture/Compare 3 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   11 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Disable the TIM Capture/Compare 4 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Disable the Capture compare channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Disable the Main Output */
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 8 as libc::c_uint |
                    (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            if (*(*htim).Instance).CCER &
                   ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                        (0x1 as libc::c_uint) << 6 as libc::c_uint |
                        (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Change the htim state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group4 Time Input Capture functions 
 *  @brief    Time Input Capture functions 
 *
@verbatim    
  ==============================================================================
              ##### Time Input Capture functions #####
  ==============================================================================
 [..]  
   This section provides functions allowing to:
   (+) Initialize and configure the TIM Input Capture. 
   (+) De-initialize the TIM Input Capture.
   (+) Start the Time Input Capture.
   (+) Stop the Time Input Capture.
   (+) Start the Time Input Capture and enable interrupt.
   (+) Stop the Time Input Capture and disable interrupt.
   (+) Start the Time Input Capture and enable DMA transfer.
   (+) Stop the Time Input Capture and disable DMA transfer.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM Input Capture Time base according to the specified
  *         parameters in the TIM_HandleTypeDef and create the associated handle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_Init(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the TIM handle allocation */
    if htim.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*htim).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
        HAL_TIM_IC_MspInit(htim);
    }
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Init the base time for the input capture */
    TIM_Base_SetConfig((*htim).Instance, &mut (*htim).Init);
    /* Initialize the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the TIM peripheral 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_DeInit(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Disable the TIM Peripheral Clock */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
    HAL_TIM_IC_MspDeInit(htim);
    /* Change TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_RESET);
    /* Release Lock */
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM INput Capture MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_MspInit(mut htim:
                                                *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_IC_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes TIM Input Capture MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_MspDeInit(mut htim:
                                                  *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_IC_MspDeInit could be implemented in the user file
   */
}
/* *
  * @brief  Starts the TIM Input Capture measurement.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_Start(mut htim: *mut TIM_HandleTypeDef,
                                          mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the Input Capture channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Input Capture measurement.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_Stop(mut htim: *mut TIM_HandleTypeDef,
                                         mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Input Capture channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM Input Capture measurement in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_Start_IT(mut htim: *mut TIM_HandleTypeDef,
                                             mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Enable the TIM Capture/Compare 1 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 1 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Enable the TIM Capture/Compare 2 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 2 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Enable the TIM Capture/Compare 3 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Enable the TIM Capture/Compare 4 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 4 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Enable the Input Capture channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Input Capture measurement in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_Stop_IT(mut htim: *mut TIM_HandleTypeDef,
                                            mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Disable the TIM Capture/Compare 1 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   1 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Disable the TIM Capture/Compare 2 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Disable the TIM Capture/Compare 3 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   3 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Disable the TIM Capture/Compare 4 interrupt */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   4 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Disable the Input Capture channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM Input Capture measurement on in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  pData: The destination Buffer address.
  * @param  Length: The length of data to be transferred from TIM peripheral to memory.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_Start_DMA(mut htim:
                                                  *mut TIM_HandleTypeDef,
                                              mut Channel: uint32_t,
                                              mut pData: *mut uint32_t,
                                              mut Length: uint16_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_BUSY as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else {
        if (*htim).State as libc::c_uint ==
               HAL_TIM_STATE_READY as libc::c_int as libc::c_uint {
            if pData.is_null() && Length as libc::c_int > 0 as libc::c_int {
                return HAL_ERROR
            } else {
                ::core::ptr::write_volatile(&mut (*htim).State as
                                                *mut HAL_TIM_StateTypeDef,
                                            HAL_TIM_STATE_BUSY)
            }
        }
    }
    match Channel {
        0 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).CCR1 as *mut uint32_t as
                                 uint32_t, pData as uint32_t,
                             Length as uint32_t);
            /* Enable the TIM Capture/Compare 1 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 9 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).CCR2 as *mut uint32_t as
                                 uint32_t, pData as uint32_t,
                             Length as uint32_t);
            /* Enable the TIM Capture/Compare 2  DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 10 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x3 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).CCR3 as *mut uint32_t as
                                 uint32_t, pData as uint32_t,
                             Length as uint32_t);
            /* Enable the TIM Capture/Compare 3  DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x4 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).CCR4 as *mut uint32_t as
                                 uint32_t, pData as uint32_t,
                             Length as uint32_t);
            /* Enable the TIM Capture/Compare 4  DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Enable the Input Capture channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0x1 as libc::c_uint);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Input Capture measurement on in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_Stop_DMA(mut htim: *mut TIM_HandleTypeDef,
                                             mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Disable the TIM Capture/Compare 1 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   9 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Disable the TIM Capture/Compare 2 DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        8 => {
            /* Disable the TIM Capture/Compare 3  DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   11 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Disable the TIM Capture/Compare 4  DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Disable the Input Capture channel */
    TIM_CCxChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Change the htim state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group5 Time One Pulse functions 
 *  @brief    Time One Pulse functions 
 *
@verbatim    
  ==============================================================================
                        ##### Time One Pulse functions #####
  ==============================================================================
  [..]  
    This section provides functions allowing to:
    (+) Initialize and configure the TIM One Pulse. 
    (+) De-initialize the TIM One Pulse.
    (+) Start the Time One Pulse.
    (+) Stop the Time One Pulse.
    (+) Start the Time One Pulse and enable interrupt.
    (+) Stop the Time One Pulse and disable interrupt.
    (+) Start the Time One Pulse and enable DMA transfer.
    (+) Stop the Time One Pulse and disable DMA transfer.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM One Pulse Time Base according to the specified
  *         parameters in the TIM_HandleTypeDef and create the associated handle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OnePulseMode: Select the One pulse mode.
  *         This parameter can be one of the following values:
  *            @arg TIM_OPMODE_SINGLE: Only one pulse will be generated.
  *            @arg TIM_OPMODE_REPETITIVE: Repetitive pulses will be generated.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_Init(mut htim:
                                                   *mut TIM_HandleTypeDef,
                                               mut OnePulseMode: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the TIM handle allocation */
    if htim.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*htim).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
        HAL_TIM_OnePulse_MspInit(htim);
    }
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Configure the Time base in the One Pulse Mode */
    TIM_Base_SetConfig((*htim).Instance, &mut (*htim).Init);
    /* Reset the OPM Bit */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           3 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Configure the OPM Mode */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | OnePulseMode) as
                                    uint32_t as uint32_t);
    /* Initialize the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the TIM One Pulse  
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_DeInit(mut htim:
                                                     *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Disable the TIM Peripheral Clock */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
    HAL_TIM_OnePulse_MspDeInit(htim);
    /* Change TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_RESET);
    /* Release Lock */
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM One Pulse MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_MspInit(mut htim:
                                                      *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_OnePulse_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes TIM One Pulse MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_MspDeInit(mut htim:
                                                        *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_OnePulse_MspDeInit could be implemented in the user file
   */
}
/* *
  * @brief  Starts the TIM One Pulse signal generation.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel : TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_Start(mut htim:
                                                    *mut TIM_HandleTypeDef,
                                                mut OutputChannel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Enable the Capture compare and the Input Capture channels 
    (in the OPM Mode the two possible channels that can be used are TIM_CHANNEL_1 and TIM_CHANNEL_2)
    if TIM_CHANNEL_1 is used as output, the TIM_CHANNEL_2 will be used as input and
    if TIM_CHANNEL_1 is used as input, the TIM_CHANNEL_2 will be used as output 
    in all combinations, the TIM_CHANNEL_1 and TIM_CHANNEL_2 should be enabled together 
    
    No need to enable the counter, it's enabled automatically by hardware 
    (the counter starts in response to a stimulus and generate a pulse */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                      0x1 as libc::c_uint);
    TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                      0x1 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Enable the main output */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM One Pulse signal generation.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel : TIM Channels to be disable.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_Stop(mut htim:
                                                   *mut TIM_HandleTypeDef,
                                               mut OutputChannel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Disable the Capture compare and the Input Capture channels 
  (in the OPM Mode the two possible channels that can be used are TIM_CHANNEL_1 and TIM_CHANNEL_2)
  if TIM_CHANNEL_1 is used as output, the TIM_CHANNEL_2 will be used as input and
  if TIM_CHANNEL_1 is used as input, the TIM_CHANNEL_2 will be used as output 
  in all combinations, the TIM_CHANNEL_1 and TIM_CHANNEL_2 should be disabled together */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint, 0 as libc::c_uint);
    TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                      0 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Disable the Main Output */
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 8 as libc::c_uint |
                    (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            if (*(*htim).Instance).CCER &
                   ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                        (0x1 as libc::c_uint) << 6 as libc::c_uint |
                        (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM One Pulse signal generation in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel : TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_Start_IT(mut htim:
                                                       *mut TIM_HandleTypeDef,
                                                   mut OutputChannel:
                                                       uint32_t)
 -> HAL_StatusTypeDef {
    /* Enable the Capture compare and the Input Capture channels 
    (in the OPM Mode the two possible channels that can be used are TIM_CHANNEL_1 and TIM_CHANNEL_2)
    if TIM_CHANNEL_1 is used as output, the TIM_CHANNEL_2 will be used as input and
    if TIM_CHANNEL_1 is used as input, the TIM_CHANNEL_2 will be used as output 
    in all combinations, the TIM_CHANNEL_1 and TIM_CHANNEL_2 should be enabled together 
    
    No need to enable the counter, it's enabled automatically by hardware 
    (the counter starts in response to a stimulus and generate a pulse */
    /* Enable the TIM Capture/Compare 1 interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable the TIM Capture/Compare 2 interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                      0x1 as libc::c_uint);
    TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                      0x1 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Enable the main output */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM One Pulse signal generation in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel : TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_Stop_IT(mut htim:
                                                      *mut TIM_HandleTypeDef,
                                                  mut OutputChannel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Disable the TIM Capture/Compare 1 interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable the TIM Capture/Compare 2 interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           2 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable the Capture compare and the Input Capture channels 
  (in the OPM Mode the two possible channels that can be used are TIM_CHANNEL_1 and TIM_CHANNEL_2)
  if TIM_CHANNEL_1 is used as output, the TIM_CHANNEL_2 will be used as input and
  if TIM_CHANNEL_1 is used as input, the TIM_CHANNEL_2 will be used as output 
  in all combinations, the TIM_CHANNEL_1 and TIM_CHANNEL_2 should be disabled together */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint, 0 as libc::c_uint);
    TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                      0 as libc::c_uint);
    if ((*htim).Instance ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            (*htim).Instance ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Disable the Main Output */
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 8 as libc::c_uint |
                    (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            if (*(*htim).Instance).CCER &
                   ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                        (0x1 as libc::c_uint) << 6 as libc::c_uint |
                        (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group6 Time Encoder functions 
 *  @brief    Time Encoder functions 
 *
@verbatim    
  ==============================================================================
                          ##### Time Encoder functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TIM Encoder. 
    (+) De-initialize the TIM Encoder.
    (+) Start the Time Encoder.
    (+) Stop the Time Encoder.
    (+) Start the Time Encoder and enable interrupt.
    (+) Stop the Time Encoder and disable interrupt.
    (+) Start the Time Encoder and enable DMA transfer.
    (+) Stop the Time Encoder and disable DMA transfer.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM Encoder Interface and create the associated handle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig: TIM Encoder Interface configuration structure
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_Init(mut htim:
                                                  *mut TIM_HandleTypeDef,
                                              mut sConfig:
                                                  *mut TIM_Encoder_InitTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmpsmcr: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccmr1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the TIM handle allocation */
    if htim.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*htim).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
        HAL_TIM_Encoder_MspInit(htim);
    }
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Reset the SMS bits */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x10007 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Configure the Time base in the Encoder Mode */
    TIM_Base_SetConfig((*htim).Instance, &mut (*htim).Init);
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*(*htim).Instance).SMCR;
    /* Get the TIMx CCMR1 register value */
    tmpccmr1 = (*(*htim).Instance).CCMR1;
    /* Get the TIMx CCER register value */
    tmpccer = (*(*htim).Instance).CCER;
    /* Set the encoder Mode */
    tmpsmcr |= (*sConfig).EncoderMode;
    /* Select the Capture Compare 1 and the Capture Compare 2 as input */
    tmpccmr1 &=
        !((0x3 as libc::c_uint) << 0 as libc::c_uint |
              (0x3 as libc::c_uint) << 8 as libc::c_uint);
    tmpccmr1 |=
        (*sConfig).IC1Selection | (*sConfig).IC2Selection << 8 as libc::c_int;
    /* Set the Capture Compare 1 and the Capture Compare 2 prescalers and filters */
    tmpccmr1 &=
        !((0x3 as libc::c_uint) << 2 as libc::c_uint |
              (0x3 as libc::c_uint) << 10 as libc::c_uint);
    tmpccmr1 &=
        !((0xf as libc::c_uint) << 4 as libc::c_uint |
              (0xf as libc::c_uint) << 12 as libc::c_uint);
    tmpccmr1 |=
        (*sConfig).IC1Prescaler | (*sConfig).IC2Prescaler << 8 as libc::c_int;
    tmpccmr1 |=
        (*sConfig).IC1Filter << 4 as libc::c_int |
            (*sConfig).IC2Filter << 12 as libc::c_int;
    /* Set the TI1 and the TI2 Polarities */
    tmpccer &=
        !((0x1 as libc::c_uint) << 1 as libc::c_uint |
              (0x1 as libc::c_uint) << 5 as libc::c_uint);
    tmpccer &=
        !((0x1 as libc::c_uint) << 3 as libc::c_uint |
              (0x1 as libc::c_uint) << 7 as libc::c_uint);
    tmpccer |=
        (*sConfig).IC1Polarity | (*sConfig).IC2Polarity << 4 as libc::c_int;
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t, tmpsmcr);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                    *mut uint32_t, tmpccmr1);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCER as
                                    *mut uint32_t, tmpccer);
    /* Initialize the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the TIM Encoder interface  
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_DeInit(mut htim:
                                                    *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Disable the TIM Peripheral Clock */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
    HAL_TIM_Encoder_MspDeInit(htim);
    /* Change TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_RESET);
    /* Release Lock */
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM Encoder Interface MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_MspInit(mut htim:
                                                     *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_Encoder_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes TIM Encoder Interface MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_MspDeInit(mut htim:
                                                       *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_Encoder_MspDeInit could be implemented in the user file
   */
}
/* *
  * @brief  Starts the TIM Encoder Interface.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_Start(mut htim:
                                                   *mut TIM_HandleTypeDef,
                                               mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the encoder interface channels */
    match Channel {
        0 => {
            TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                              0x1 as libc::c_uint);
        }
        4 => {
            TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                              0x1 as libc::c_uint);
        }
        _ => {
            TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                              0x1 as libc::c_uint);
            TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                              0x1 as libc::c_uint);
        }
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Encoder Interface.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_Stop(mut htim:
                                                  *mut TIM_HandleTypeDef,
                                              mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Input Capture channels 1 and 2
    (in the EncoderInterface the two possible channels that can be used are TIM_CHANNEL_1 and TIM_CHANNEL_2) */
    match Channel {
        0 => {
            TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                              0 as libc::c_uint);
        }
        4 => {
            TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                              0 as libc::c_uint);
        }
        _ => {
            TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                              0 as libc::c_uint);
            TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                              0 as libc::c_uint);
        }
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM Encoder Interface in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_Start_IT(mut htim:
                                                      *mut TIM_HandleTypeDef,
                                                  mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the encoder interface channels */
  /* Enable the capture compare Interrupts 1 and/or 2 */
    match Channel {
        0 => {
            TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                              0x1 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 1 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        4 => {
            TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                              0x1 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 2 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        _ => {
            TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                              0x1 as libc::c_uint);
            TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                              0x1 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 1 as libc::c_uint) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 2 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Encoder Interface in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_Stop_IT(mut htim:
                                                     *mut TIM_HandleTypeDef,
                                                 mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Input Capture channels 1 and 2
    (in the EncoderInterface the two possible channels that can be used are TIM_CHANNEL_1 and TIM_CHANNEL_2) */
    if Channel == 0 as libc::c_uint {
        TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                          0 as libc::c_uint);
        /* Disable the capture compare Interrupts 1 */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               1 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    } else if Channel == 0x4 as libc::c_uint {
        TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                          0 as libc::c_uint);
        /* Disable the capture compare Interrupts 2 */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               2 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    } else {
        TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                          0 as libc::c_uint);
        TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                          0 as libc::c_uint);
        /* Disable the capture compare Interrupts 1 and 2 */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               1 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               2 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Change the htim state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Starts the TIM Encoder Interface in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @param  pData1: The destination Buffer address for IC1.
  * @param  pData2: The destination Buffer address for IC2.
  * @param  Length: The length of data to be transferred from TIM peripheral to memory.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_Start_DMA(mut htim:
                                                       *mut TIM_HandleTypeDef,
                                                   mut Channel: uint32_t,
                                                   mut pData1: *mut uint32_t,
                                                   mut pData2: *mut uint32_t,
                                                   mut Length: uint16_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_BUSY as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else {
        if (*htim).State as libc::c_uint ==
               HAL_TIM_STATE_READY as libc::c_int as libc::c_uint {
            if (pData1.is_null() || pData2.is_null()) &&
                   Length as libc::c_int > 0 as libc::c_int {
                return HAL_ERROR
            } else {
                ::core::ptr::write_volatile(&mut (*htim).State as
                                                *mut HAL_TIM_StateTypeDef,
                                            HAL_TIM_STATE_BUSY)
            }
        }
    }
    match Channel {
        0 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).CCR1 as *mut uint32_t as
                                 uint32_t, pData1 as uint32_t,
                             Length as uint32_t);
            /* Enable the TIM Input Capture DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 9 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable the Peripheral */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable the Capture compare channel */
            TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                              0x1 as libc::c_uint);
        }
        4 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).CCR2 as *mut uint32_t as
                                 uint32_t, pData2 as uint32_t,
                             Length as uint32_t);
            /* Enable the TIM Input Capture  DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 10 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable the Peripheral */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable the Capture compare channel */
            TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                              0x1 as libc::c_uint);
        }
        60 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).CCR1 as *mut uint32_t as
                                 uint32_t, pData1 as uint32_t,
                             Length as uint32_t);
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).CCR2 as *mut uint32_t as
                                 uint32_t, pData2 as uint32_t,
                             Length as uint32_t);
            /* Enable the Peripheral */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable the Capture compare channel */
            TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                              0x1 as libc::c_uint);
            TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                              0x1 as libc::c_uint);
            /* Enable the TIM Input Capture  DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 9 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable the TIM Input Capture  DMA request */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 10 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM Encoder Interface in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_Stop_DMA(mut htim:
                                                      *mut TIM_HandleTypeDef,
                                                  mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Input Capture channels 1 and 2
    (in the EncoderInterface the two possible channels that can be used are TIM_CHANNEL_1 and TIM_CHANNEL_2) */
    if Channel == 0 as libc::c_uint {
        TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                          0 as libc::c_uint);
        /* Disable the capture compare DMA Request 1 */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               9 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    } else if Channel == 0x4 as libc::c_uint {
        TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                          0 as libc::c_uint);
        /* Disable the capture compare DMA Request 2 */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               10 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    } else {
        TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                          0 as libc::c_uint);
        TIM_CCxChannelCmd((*htim).Instance, 0x4 as libc::c_uint,
                          0 as libc::c_uint);
        /* Disable the capture compare DMA Request 1 and 2 */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               9 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               10 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    /* Disable the Peripheral */
    if (*(*htim).Instance).CCER &
           ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                (0x1 as libc::c_uint) << 12 as libc::c_uint) ==
           0 as libc::c_int as libc::c_uint {
        if (*(*htim).Instance).CCER &
               ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
               0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Change the htim state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group7 TIM IRQ handler management 
 *  @brief    IRQ handler management 
 *
@verbatim   
  ==============================================================================
                        ##### IRQ handler management #####
  ==============================================================================  
  [..]  
    This section provides Timer IRQ handler function.
               
@endverbatim
  * @{
  */
/* *
  * @brief  This function handles TIM interrupts requests.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IRQHandler(mut htim:
                                                *mut TIM_HandleTypeDef) {
    /* Capture compare 1 event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 1 as libc::c_uint ==
            (0x1 as libc::c_uint) << 1 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 1 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              1 as libc::c_uint));
            (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_1;
            /* Input capture event */
            if (*(*htim).Instance).CCMR1 &
                   (0x3 as libc::c_uint) << 0 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                HAL_TIM_IC_CaptureCallback(htim);
            } else {
                /* Output compare event */
                HAL_TIM_OC_DelayElapsedCallback(htim);
                HAL_TIM_PWM_PulseFinishedCallback(htim);
            }
            (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED
        }
    }
    /* Capture compare 2 event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 2 as libc::c_uint ==
            (0x1 as libc::c_uint) << 2 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 2 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 2 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              2 as libc::c_uint));
            (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_2;
            /* Input capture event */
            if (*(*htim).Instance).CCMR1 &
                   (0x3 as libc::c_uint) << 8 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                HAL_TIM_IC_CaptureCallback(htim);
            } else {
                /* Output compare event */
                HAL_TIM_OC_DelayElapsedCallback(htim);
                HAL_TIM_PWM_PulseFinishedCallback(htim);
            }
            (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED
        }
    }
    /* Capture compare 3 event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 3 as libc::c_uint ==
            (0x1 as libc::c_uint) << 3 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 3 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 3 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              3 as libc::c_uint));
            (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_3;
            /* Input capture event */
            if (*(*htim).Instance).CCMR2 &
                   (0x3 as libc::c_uint) << 0 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                HAL_TIM_IC_CaptureCallback(htim);
            } else {
                /* Output compare event */
                HAL_TIM_OC_DelayElapsedCallback(htim);
                HAL_TIM_PWM_PulseFinishedCallback(htim);
            }
            (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED
        }
    }
    /* Capture compare 4 event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 4 as libc::c_uint ==
            (0x1 as libc::c_uint) << 4 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 4 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              4 as libc::c_uint));
            (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_4;
            /* Input capture event */
            if (*(*htim).Instance).CCMR2 &
                   (0x3 as libc::c_uint) << 8 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                HAL_TIM_IC_CaptureCallback(htim);
            } else {
                /* Output compare event */
                HAL_TIM_OC_DelayElapsedCallback(htim);
                HAL_TIM_PWM_PulseFinishedCallback(htim);
            }
            (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED
        }
    }
    /* TIM Update event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
            (0x1 as libc::c_uint) << 0 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              0 as libc::c_uint));
            HAL_TIM_PeriodElapsedCallback(htim);
        }
    }
    /* TIM Break input event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 7 as libc::c_uint ==
            (0x1 as libc::c_uint) << 7 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 7 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 7 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              7 as libc::c_uint));
            HAL_TIMEx_BreakCallback(htim);
        }
    }
    /* TIM Break input event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 8 as libc::c_uint ==
            (0x1 as libc::c_uint) << 8 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 7 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 7 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              7 as libc::c_uint));
            HAL_TIMEx_BreakCallback(htim);
        }
    }
    /* TIM Trigger detection event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 6 as libc::c_uint ==
            (0x1 as libc::c_uint) << 6 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 6 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 6 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              6 as libc::c_uint));
            HAL_TIM_TriggerCallback(htim);
        }
    }
    /* TIM commutation event */
    if ((*(*htim).Instance).SR & (0x1 as libc::c_uint) << 5 as libc::c_uint ==
            (0x1 as libc::c_uint) << 5 as libc::c_uint) as libc::c_int !=
           RESET as libc::c_int {
        if (if (*(*htim).Instance).DIER &
                   (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 5 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) != RESET as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              5 as libc::c_uint));
            HAL_TIMEx_CommutationCallback(htim);
        }
    };
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group8 Peripheral Control functions
 *  @brief   	Peripheral Control functions 
 *
@verbatim   
  ==============================================================================
                   ##### Peripheral Control functions #####
  ==============================================================================  
 [..] 
   This section provides functions allowing to:
   (+) Configure The Input Output channels for OC, PWM, IC or One Pulse mode. 
   (+) Configure External Clock source.
   (+) Configure Complementary channels, break features and dead time.
   (+) Configure Master and the Slave synchronization.
   (+) Configure the DMA Burst Mode.
      
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM Output Compare Channels according to the specified
  *         parameters in the TIM_OC_InitTypeDef.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig: TIM Output Compare configuration structure
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_ConfigChannel(mut htim:
                                                      *mut TIM_HandleTypeDef,
                                                  mut sConfig:
                                                      *mut TIM_OC_InitTypeDef,
                                                  mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Check input state */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    match Channel {
        0 => {
            /* Configure the TIM Channel 1 in Output Compare */
            TIM_OC1_SetConfig((*htim).Instance, sConfig);
        }
        4 => {
            /* Configure the TIM Channel 2 in Output Compare */
            TIM_OC2_SetConfig((*htim).Instance, sConfig);
        }
        8 => {
            /* Configure the TIM Channel 3 in Output Compare */
            TIM_OC3_SetConfig((*htim).Instance, sConfig);
        }
        12 => {
            /* Configure the TIM Channel 4 in Output Compare */
            TIM_OC4_SetConfig((*htim).Instance, sConfig);
        }
        _ => { }
    }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM Input Capture Channels according to the specified
  *         parameters in the TIM_IC_InitTypeDef.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig: TIM Input Capture configuration structure
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_ConfigChannel(mut htim:
                                                      *mut TIM_HandleTypeDef,
                                                  mut sConfig:
                                                      *mut TIM_IC_InitTypeDef,
                                                  mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    if Channel == 0 as libc::c_uint {
        /* TI1 Configuration */
        TIM_TI1_SetConfig((*htim).Instance, (*sConfig).ICPolarity,
                          (*sConfig).ICSelection, (*sConfig).ICFilter);
        /* Reset the IC1PSC Bits */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3 as libc::c_uint) <<
                                               2 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Set the IC1PSC value */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfig).ICPrescaler) as uint32_t
                                        as uint32_t)
    } else if Channel == 0x4 as libc::c_uint {
        /* TI2 Configuration */
        TIM_TI2_SetConfig((*htim).Instance, (*sConfig).ICPolarity,
                          (*sConfig).ICSelection, (*sConfig).ICFilter);
        /* Reset the IC2PSC Bits */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3 as libc::c_uint) <<
                                               10 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Set the IC2PSC value */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfig).ICPrescaler <<
                                             8 as libc::c_int) as uint32_t as
                                        uint32_t)
    } else if Channel == 0x8 as libc::c_uint {
        /* TI3 Configuration */
        TIM_TI3_SetConfig((*htim).Instance, (*sConfig).ICPolarity,
                          (*sConfig).ICSelection, (*sConfig).ICFilter);
        /* Reset the IC3PSC Bits */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3 as libc::c_uint) <<
                                               2 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Set the IC3PSC value */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfig).ICPrescaler) as uint32_t
                                        as uint32_t)
    } else {
        /* TI4 Configuration */
        TIM_TI4_SetConfig((*htim).Instance, (*sConfig).ICPolarity,
                          (*sConfig).ICSelection, (*sConfig).ICFilter);
        /* Reset the IC4PSC Bits */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3 as libc::c_uint) <<
                                               10 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Set the IC4PSC value */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfig).ICPrescaler <<
                                             8 as libc::c_int) as uint32_t as
                                        uint32_t)
    }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM PWM  channels according to the specified
  *         parameters in the TIM_OC_InitTypeDef.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig: TIM PWM configuration structure
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_ConfigChannel(mut htim:
                                                       *mut TIM_HandleTypeDef,
                                                   mut sConfig:
                                                       *mut TIM_OC_InitTypeDef,
                                                   mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    match Channel {
        0 => {
            /* Configure the Channel 1 in PWM mode */
            TIM_OC1_SetConfig((*htim).Instance, sConfig);
            /* Set the Preload enable bit for channel1 */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Configure the Output Fast mode */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfig).OCFastMode) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* Configure the Channel 2 in PWM mode */
            TIM_OC2_SetConfig((*htim).Instance, sConfig);
            /* Set the Preload enable bit for channel2 */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Configure the Output Fast mode */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfig).OCFastMode <<
                                                 8 as libc::c_int) as uint32_t
                                            as uint32_t)
        }
        8 => {
            /* Configure the Channel 3 in PWM mode */
            TIM_OC3_SetConfig((*htim).Instance, sConfig);
            /* Set the Preload enable bit for channel3 */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Configure the Output Fast mode */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfig).OCFastMode) as
                                            uint32_t as uint32_t)
        }
        12 => {
            /* Configure the Channel 4 in PWM mode */
            TIM_OC4_SetConfig((*htim).Instance, sConfig);
            /* Set the Preload enable bit for channel4 */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Configure the Output Fast mode */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfig).OCFastMode <<
                                                 8 as libc::c_int) as uint32_t
                                            as uint32_t)
        }
        _ => { }
    }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM One Pulse Channels according to the specified
  *         parameters in the TIM_OnePulse_InitTypeDef.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig: TIM One Pulse configuration structure
  * @param  OutputChannel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @param  InputChannel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_ConfigChannel(mut htim:
                                                            *mut TIM_HandleTypeDef,
                                                        mut sConfig:
                                                            *mut TIM_OnePulse_InitTypeDef,
                                                        mut OutputChannel:
                                                            uint32_t,
                                                        mut InputChannel:
                                                            uint32_t)
 -> HAL_StatusTypeDef {
    let mut temp1: TIM_OC_InitTypeDef =
        TIM_OC_InitTypeDef{OCMode: 0,
                           Pulse: 0,
                           OCPolarity: 0,
                           OCNPolarity: 0,
                           OCFastMode: 0,
                           OCIdleState: 0,
                           OCNIdleState: 0,};
    /* Check the parameters */
    if OutputChannel != InputChannel {
        if (*htim).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*htim).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*htim).State as
                                        *mut HAL_TIM_StateTypeDef,
                                    HAL_TIM_STATE_BUSY);
        /* Extract the Output compare configuration from sConfig structure */
        temp1.OCMode = (*sConfig).OCMode;
        temp1.Pulse = (*sConfig).Pulse;
        temp1.OCPolarity = (*sConfig).OCPolarity;
        temp1.OCNPolarity = (*sConfig).OCNPolarity;
        temp1.OCIdleState = (*sConfig).OCIdleState;
        temp1.OCNIdleState = (*sConfig).OCNIdleState;
        match OutputChannel {
            0 => { TIM_OC1_SetConfig((*htim).Instance, &mut temp1); }
            4 => { TIM_OC2_SetConfig((*htim).Instance, &mut temp1); }
            _ => { }
        }
        match InputChannel {
            0 => {
                TIM_TI1_SetConfig((*htim).Instance, (*sConfig).ICPolarity,
                                  (*sConfig).ICSelection,
                                  (*sConfig).ICFilter);
                /* Reset the IC1PSC Bits */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x3 as libc::c_uint) <<
                                                       2 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                /* Select the Trigger source */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x7 as libc::c_uint) <<
                                                       4 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x50 as libc::c_uint) as
                                                uint32_t as uint32_t);
                /* Select the Slave Mode */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x10007 as libc::c_uint)
                                                       << 0 as libc::c_uint))
                                                as uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((0x4 as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                      (0x2 as libc::c_uint) <<
                                                          0 as libc::c_uint))
                                                as uint32_t as uint32_t)
            }
            4 => {
                TIM_TI2_SetConfig((*htim).Instance, (*sConfig).ICPolarity,
                                  (*sConfig).ICSelection,
                                  (*sConfig).ICFilter);
                /* Reset the IC2PSC Bits */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x3 as libc::c_uint) <<
                                                       10 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                /* Select the Trigger source */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x7 as libc::c_uint) <<
                                                       4 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x60 as libc::c_uint) as
                                                uint32_t as uint32_t);
                /* Select the Slave Mode */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x10007 as libc::c_uint)
                                                       << 0 as libc::c_uint))
                                                as uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((0x4 as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                      (0x2 as libc::c_uint) <<
                                                          0 as libc::c_uint))
                                                as uint32_t as uint32_t)
            }
            _ => { }
        }
        ::core::ptr::write_volatile(&mut (*htim).State as
                                        *mut HAL_TIM_StateTypeDef,
                                    HAL_TIM_STATE_READY);
        (*htim).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_ERROR };
}
/* *
  * @brief  Configure the DMA Burst to transfer Data from the memory to the TIM peripheral  
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstBaseAddress: TIM Base address from when the DMA will starts the Data write.
  *         This parameters can be on of the following values:
  *            @arg TIM_DMABASE_CR1  
  *            @arg TIM_DMABASE_CR2
  *            @arg TIM_DMABASE_SMCR
  *            @arg TIM_DMABASE_DIER
  *            @arg TIM_DMABASE_SR
  *            @arg TIM_DMABASE_EGR
  *            @arg TIM_DMABASE_CCMR1
  *            @arg TIM_DMABASE_CCMR2
  *            @arg TIM_DMABASE_CCER
  *            @arg TIM_DMABASE_CNT   
  *            @arg TIM_DMABASE_PSC   
  *            @arg TIM_DMABASE_ARR
  *            @arg TIM_DMABASE_RCR
  *            @arg TIM_DMABASE_CCR1
  *            @arg TIM_DMABASE_CCR2
  *            @arg TIM_DMABASE_CCR3  
  *            @arg TIM_DMABASE_CCR4
  *            @arg TIM_DMABASE_BDTR
  *            @arg TIM_DMABASE_DCR
  * @param  BurstRequestSrc: TIM DMA Request sources.
  *         This parameters can be on of the following values:
  *            @arg TIM_DMA_UPDATE: TIM update Interrupt source
  *            @arg TIM_DMA_CC1: TIM Capture Compare 1 DMA source
  *            @arg TIM_DMA_CC2: TIM Capture Compare 2 DMA source
  *            @arg TIM_DMA_CC3: TIM Capture Compare 3 DMA source
  *            @arg TIM_DMA_CC4: TIM Capture Compare 4 DMA source
  *            @arg TIM_DMA_COM: TIM Commutation DMA source
  *            @arg TIM_DMA_TRIGGER: TIM Trigger DMA source
  * @param  BurstBuffer: The Buffer address.
  * @param  BurstLength: DMA Burst length. This parameter can be one value
  *         between TIM_DMABURSTLENGTH_1TRANSFER and TIM_DMABURSTLENGTH_18TRANSFERS.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_DMABurst_WriteStart(mut htim:
                                                         *mut TIM_HandleTypeDef,
                                                     mut BurstBaseAddress:
                                                         uint32_t,
                                                     mut BurstRequestSrc:
                                                         uint32_t,
                                                     mut BurstBuffer:
                                                         *mut uint32_t,
                                                     mut BurstLength:
                                                         uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_BUSY as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else {
        if (*htim).State as libc::c_uint ==
               HAL_TIM_STATE_READY as libc::c_int as libc::c_uint {
            if BurstBuffer.is_null() &&
                   BurstLength > 0 as libc::c_int as libc::c_uint {
                return HAL_ERROR
            } else {
                ::core::ptr::write_volatile(&mut (*htim).State as
                                                *mut HAL_TIM_StateTypeDef,
                                            HAL_TIM_STATE_BUSY)
            }
        }
    }
    match BurstRequestSrc {
        256 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMAPeriodElapsedCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0 as libc::c_uint as uint16_t as
                                              usize], BurstBuffer as uint32_t,
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        512 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                              usize], BurstBuffer as uint32_t,
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        1024 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                              usize], BurstBuffer as uint32_t,
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        2048 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x3 as libc::c_uint as uint16_t as
                                              usize], BurstBuffer as uint32_t,
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        4096 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMADelayPulseCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x4 as libc::c_uint as uint16_t as
                                              usize], BurstBuffer as uint32_t,
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        8192 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x5 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIMEx_DMACommutationCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x5 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x5 as libc::c_uint as uint16_t as
                                              usize], BurstBuffer as uint32_t,
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        16384 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x6 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMATriggerCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x6 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x6 as libc::c_uint as uint16_t as
                                              usize], BurstBuffer as uint32_t,
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        _ => { }
    }
    /* configure the DMA Burst Mode */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DCR as *mut uint32_t,
                                BurstBaseAddress | BurstLength);
    /* Enable the TIM DMA Request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | BurstRequestSrc) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM DMA Burst mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstRequestSrc: TIM DMA Request sources to disable
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_DMABurst_WriteStop(mut htim:
                                                        *mut TIM_HandleTypeDef,
                                                    mut BurstRequestSrc:
                                                        uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Abort the DMA transfer (at least disable the DMA channel) */
    match BurstRequestSrc {
        256 => {
            HAL_DMA_Abort((*htim).hdma[0 as libc::c_uint as uint16_t as
                                           usize]);
        }
        512 => {
            HAL_DMA_Abort((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                           usize]);
        }
        1024 => {
            HAL_DMA_Abort((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                           usize]);
        }
        2048 => {
            HAL_DMA_Abort((*htim).hdma[0x3 as libc::c_uint as uint16_t as
                                           usize]);
        }
        4096 => {
            HAL_DMA_Abort((*htim).hdma[0x4 as libc::c_uint as uint16_t as
                                           usize]);
        }
        8192 => {
            HAL_DMA_Abort((*htim).hdma[0x5 as libc::c_uint as uint16_t as
                                           usize]);
        }
        16384 => {
            HAL_DMA_Abort((*htim).hdma[0x6 as libc::c_uint as uint16_t as
                                           usize]);
        }
        _ => { }
    }
    /* Disable the TIM Update DMA request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !BurstRequestSrc) as
                                    uint32_t as uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Configure the DMA Burst to transfer Data from the TIM peripheral to the memory 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstBaseAddress: TIM Base address from when the DMA will starts the Data read.
  *         This parameters can be on of the following values:
  *            @arg TIM_DMABASE_CR1  
  *            @arg TIM_DMABASE_CR2
  *            @arg TIM_DMABASE_SMCR
  *            @arg TIM_DMABASE_DIER
  *            @arg TIM_DMABASE_SR
  *            @arg TIM_DMABASE_EGR
  *            @arg TIM_DMABASE_CCMR1
  *            @arg TIM_DMABASE_CCMR2
  *            @arg TIM_DMABASE_CCER
  *            @arg TIM_DMABASE_CNT   
  *            @arg TIM_DMABASE_PSC   
  *            @arg TIM_DMABASE_ARR
  *            @arg TIM_DMABASE_RCR
  *            @arg TIM_DMABASE_CCR1
  *            @arg TIM_DMABASE_CCR2
  *            @arg TIM_DMABASE_CCR3  
  *            @arg TIM_DMABASE_CCR4
  *            @arg TIM_DMABASE_BDTR
  *            @arg TIM_DMABASE_DCR
  * @param  BurstRequestSrc: TIM DMA Request sources.
  *         This parameters can be on of the following values:
  *            @arg TIM_DMA_UPDATE: TIM update Interrupt source
  *            @arg TIM_DMA_CC1: TIM Capture Compare 1 DMA source
  *            @arg TIM_DMA_CC2: TIM Capture Compare 2 DMA source
  *            @arg TIM_DMA_CC3: TIM Capture Compare 3 DMA source
  *            @arg TIM_DMA_CC4: TIM Capture Compare 4 DMA source
  *            @arg TIM_DMA_COM: TIM Commutation DMA source
  *            @arg TIM_DMA_TRIGGER: TIM Trigger DMA source
  * @param  BurstBuffer: The Buffer address.
  * @param  BurstLength: DMA Burst length. This parameter can be one value
  *         between TIM_DMABURSTLENGTH_1TRANSFER and TIM_DMABURSTLENGTH_18TRANSFERS.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_DMABurst_ReadStart(mut htim:
                                                        *mut TIM_HandleTypeDef,
                                                    mut BurstBaseAddress:
                                                        uint32_t,
                                                    mut BurstRequestSrc:
                                                        uint32_t,
                                                    mut BurstBuffer:
                                                        *mut uint32_t,
                                                    mut BurstLength: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).State as libc::c_uint ==
           HAL_TIM_STATE_BUSY as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else {
        if (*htim).State as libc::c_uint ==
               HAL_TIM_STATE_READY as libc::c_int as libc::c_uint {
            if BurstBuffer.is_null() &&
                   BurstLength > 0 as libc::c_int as libc::c_uint {
                return HAL_ERROR
            } else {
                ::core::ptr::write_volatile(&mut (*htim).State as
                                                *mut HAL_TIM_StateTypeDef,
                                            HAL_TIM_STATE_BUSY)
            }
        }
    }
    match BurstRequestSrc {
        256 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMAPeriodElapsedCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t, BurstBuffer as uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        512 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t, BurstBuffer as uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        1024 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x2 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t, BurstBuffer as uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        2048 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x3 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x3 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t, BurstBuffer as uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        4096 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMACaptureCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x4 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x4 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t, BurstBuffer as uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        8192 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x5 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIMEx_DMACommutationCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x5 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x5 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t, BurstBuffer as uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        16384 => {
            /* Set the DMA Period elapsed callback */
            (*(*htim).hdma[0x6 as libc::c_uint as uint16_t as
                               usize]).XferCpltCallback =
                Some(TIM_DMATriggerCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*htim).hdma[0x6 as libc::c_uint as uint16_t as
                               usize]).XferErrorCallback =
                Some(TIM_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Enable the DMA Stream */
            HAL_DMA_Start_IT((*htim).hdma[0x6 as libc::c_uint as uint16_t as
                                              usize],
                             &mut (*(*htim).Instance).DMAR as *mut uint32_t as
                                 uint32_t, BurstBuffer as uint32_t,
                             (BurstLength >>
                                  8 as
                                      libc::c_int).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint));
        }
        _ => { }
    }
    /* configure the DMA Burst Mode */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DCR as *mut uint32_t,
                                BurstBaseAddress | BurstLength);
    /* Enable the TIM DMA Request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | BurstRequestSrc) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stop the DMA burst reading 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstRequestSrc: TIM DMA Request sources to disable.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_DMABurst_ReadStop(mut htim:
                                                       *mut TIM_HandleTypeDef,
                                                   mut BurstRequestSrc:
                                                       uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Abort the DMA transfer (at least disable the DMA channel) */
    match BurstRequestSrc {
        256 => {
            HAL_DMA_Abort((*htim).hdma[0 as libc::c_uint as uint16_t as
                                           usize]);
        }
        512 => {
            HAL_DMA_Abort((*htim).hdma[0x1 as libc::c_uint as uint16_t as
                                           usize]);
        }
        1024 => {
            HAL_DMA_Abort((*htim).hdma[0x2 as libc::c_uint as uint16_t as
                                           usize]);
        }
        2048 => {
            HAL_DMA_Abort((*htim).hdma[0x3 as libc::c_uint as uint16_t as
                                           usize]);
        }
        4096 => {
            HAL_DMA_Abort((*htim).hdma[0x4 as libc::c_uint as uint16_t as
                                           usize]);
        }
        8192 => {
            HAL_DMA_Abort((*htim).hdma[0x5 as libc::c_uint as uint16_t as
                                           usize]);
        }
        16384 => {
            HAL_DMA_Abort((*htim).hdma[0x6 as libc::c_uint as uint16_t as
                                           usize]);
        }
        _ => { }
    }
    /* Disable the TIM Update DMA request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !BurstRequestSrc) as
                                    uint32_t as uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Generate a software event
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  EventSource: specifies the event source.
  *          This parameter can be one of the following values:
  *            @arg TIM_EVENTSOURCE_UPDATE: Timer update Event source
  *            @arg TIM_EVENTSOURCE_CC1: Timer Capture Compare 1 Event source
  *            @arg TIM_EVENTSOURCE_CC2: Timer Capture Compare 2 Event source
  *            @arg TIM_EVENTSOURCE_CC3: Timer Capture Compare 3 Event source
  *            @arg TIM_EVENTSOURCE_CC4: Timer Capture Compare 4 Event source
  *            @arg TIM_EVENTSOURCE_COM: Timer COM event source  
  *            @arg TIM_EVENTSOURCE_TRIGGER: Timer Trigger Event source
  *            @arg TIM_EVENTSOURCE_BREAK: Timer Break event source
  *            @arg TIM_EVENTSOURCE_BREAK2: Timer Break2 event source  
  * @note   TIM6 and TIM7 can only generate an update event. 
  * @note   TIM_EVENTSOURCE_COM, TIM_EVENTSOURCE_BREAK and TIM_EVENTSOURCE_BREAK2 are used only with TIM1 and TIM8.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_GenerateEvent(mut htim:
                                                   *mut TIM_HandleTypeDef,
                                               mut EventSource: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Process Locked */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    /* Change the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Set the event sources */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).EGR as *mut uint32_t,
                                EventSource);
    /* Change the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Configures the OCRef clear feature
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sClearInputConfig: pointer to a TIM_ClearInputConfigTypeDef structure that
  *         contains the OCREF clear feature and parameters for the TIM peripheral. 
  * @param  Channel: specifies the TIM Channel.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_ConfigOCrefClear(mut htim:
                                                      *mut TIM_HandleTypeDef,
                                                  mut sClearInputConfig:
                                                      *mut TIM_ClearInputConfigTypeDef,
                                                  mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Process Locked */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    if (*sClearInputConfig).ClearInputSource == 0x1 as libc::c_uint {
        TIM_ETR_SetConfig((*htim).Instance,
                          (*sClearInputConfig).ClearInputPrescaler,
                          (*sClearInputConfig).ClearInputPolarity,
                          (*sClearInputConfig).ClearInputFilter);
    }
    match Channel {
        0 => {
            if (*sClearInputConfig).ClearInputState !=
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the Ocref clear feature for Channel 1 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     7 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                /* Disable the Ocref clear feature for Channel 1 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       7 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
        4 => {
            if (*sClearInputConfig).ClearInputState !=
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the Ocref clear feature for Channel 2 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     15 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                /* Disable the Ocref clear feature for Channel 2 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
        8 => {
            if (*sClearInputConfig).ClearInputState !=
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the Ocref clear feature for Channel 3 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     7 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                /* Disable the Ocref clear feature for Channel 3 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       7 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
        12 => {
            if (*sClearInputConfig).ClearInputState !=
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the Ocref clear feature for Channel 4 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     15 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                /* Disable the Ocref clear feature for Channel 4 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       15 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
        _ => { }
    }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief   Configures the clock source to be used
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sClockSourceConfig: pointer to a TIM_ClockConfigTypeDef structure that
  *         contains the clock source information for the TIM peripheral. 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_ConfigClockSource(mut htim:
                                                       *mut TIM_HandleTypeDef,
                                                   mut sClockSourceConfig:
                                                       *mut TIM_ClockConfigTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmpsmcr: uint32_t = 0 as libc::c_int as uint32_t;
    /* Process Locked */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Check the parameters */
    /* Reset the SMS, TS, ECE, ETPS and ETRF bits */
    tmpsmcr = (*(*htim).Instance).SMCR;
    tmpsmcr &=
        !((0x10007 as libc::c_uint) << 0 as libc::c_uint |
              (0x7 as libc::c_uint) << 4 as libc::c_uint);
    tmpsmcr &=
        !((0xf as libc::c_uint) << 8 as libc::c_uint |
              (0x3 as libc::c_uint) << 12 as libc::c_uint |
              (0x1 as libc::c_uint) << 14 as libc::c_uint |
              (0x1 as libc::c_uint) << 15 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t, tmpsmcr);
    match (*sClockSourceConfig).ClockSource {
        4096 => {
            /* Disable slave mode to clock the prescaler directly with the internal clock */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x10007 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        112 => {
            /* Configure the ETR Clock source */
            TIM_ETR_SetConfig((*htim).Instance,
                              (*sClockSourceConfig).ClockPrescaler,
                              (*sClockSourceConfig).ClockPolarity,
                              (*sClockSourceConfig).ClockFilter);
            /* Get the TIMx SMCR register value */
            tmpsmcr = (*(*htim).Instance).SMCR;
            /* Reset the SMS and TS Bits */
            tmpsmcr &=
                !((0x10007 as libc::c_uint) << 0 as libc::c_uint |
                      (0x7 as libc::c_uint) << 4 as libc::c_uint);
            /* Select the External clock mode1 and the ETRF trigger */
            tmpsmcr |=
                (0x4 as libc::c_uint) << 0 as libc::c_uint |
                    (0x2 as libc::c_uint) << 0 as libc::c_uint |
                    (0x1 as libc::c_uint) << 0 as libc::c_uint |
                    (0x7 as libc::c_uint) << 4 as libc::c_uint;
            /* Write to TIMx SMCR */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                            *mut uint32_t, tmpsmcr)
        }
        8192 => {
            /* Configure the ETR Clock source */
            TIM_ETR_SetConfig((*htim).Instance,
                              (*sClockSourceConfig).ClockPrescaler,
                              (*sClockSourceConfig).ClockPolarity,
                              (*sClockSourceConfig).ClockFilter);
            /* Enable the External clock mode2 */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 14 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        80 => {
            /* Check TI1 input conditioning related parameters */
            TIM_TI1_ConfigInputStage((*htim).Instance,
                                     (*sClockSourceConfig).ClockPolarity,
                                     (*sClockSourceConfig).ClockFilter);
            TIM_ITRx_SetConfig((*htim).Instance,
                               ((0x1 as libc::c_uint) << 4 as libc::c_uint |
                                    (0x4 as libc::c_uint) <<
                                        4 as libc::c_uint) as uint16_t);
        }
        96 => {
            /* Check TI1 input conditioning related parameters */
            TIM_TI2_ConfigInputStage((*htim).Instance,
                                     (*sClockSourceConfig).ClockPolarity,
                                     (*sClockSourceConfig).ClockFilter);
            TIM_ITRx_SetConfig((*htim).Instance,
                               ((0x2 as libc::c_uint) << 4 as libc::c_uint |
                                    (0x4 as libc::c_uint) <<
                                        4 as libc::c_uint) as uint16_t);
        }
        64 => {
            /* Check TI1 input conditioning related parameters */
            TIM_TI1_ConfigInputStage((*htim).Instance,
                                     (*sClockSourceConfig).ClockPolarity,
                                     (*sClockSourceConfig).ClockFilter);
            TIM_ITRx_SetConfig((*htim).Instance,
                               ((0x4 as libc::c_uint) << 4 as libc::c_uint) as
                                   uint16_t);
        }
        0 => {
            TIM_ITRx_SetConfig((*htim).Instance,
                               0 as libc::c_uint as uint16_t);
        }
        16 => {
            TIM_ITRx_SetConfig((*htim).Instance,
                               ((0x1 as libc::c_uint) << 4 as libc::c_uint) as
                                   uint16_t);
        }
        32 => {
            TIM_ITRx_SetConfig((*htim).Instance,
                               ((0x2 as libc::c_uint) << 4 as libc::c_uint) as
                                   uint16_t);
        }
        48 => {
            TIM_ITRx_SetConfig((*htim).Instance,
                               ((0x1 as libc::c_uint) << 4 as libc::c_uint |
                                    (0x2 as libc::c_uint) <<
                                        4 as libc::c_uint) as uint16_t);
        }
        _ => { }
    }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Selects the signal connected to the TI1 input: direct from CH1_input
  *         or a XOR combination between CH1_input, CH2_input & CH3_input
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  TI1_Selection: Indicate whether or not channel 1 is connected to the
  *         output of a XOR gate.
  *         This parameter can be one of the following values:
  *            @arg TIM_TI1SELECTION_CH1: The TIMx_CH1 pin is connected to TI1 input
  *            @arg TIM_TI1SELECTION_XORCOMBINATION: The TIMx_CH1, CH2 and CH3
  *            pins are connected to the TI1 input (XOR combination)
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_ConfigTI1Input(mut htim:
                                                    *mut TIM_HandleTypeDef,
                                                mut TI1_Selection: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmpcr2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*(*htim).Instance).CR2;
    /* Reset the TI1 selection */
    tmpcr2 &= !((0x1 as libc::c_uint) << 7 as libc::c_uint);
    /* Set the TI1 selection */
    tmpcr2 |= TI1_Selection;
    /* Write to TIMxCR2 */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                tmpcr2);
    return HAL_OK;
}
/* *
  * @brief  Configures the TIM in Slave mode
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sSlaveConfig: pointer to a TIM_SlaveConfigTypeDef structure that
  *         contains the selected trigger (internal trigger input, filtered
  *         timer input or external trigger input) and the ) and the Slave 
  *         mode (Disable, Reset, Gated, Trigger, External clock mode 1). 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_SlaveConfigSynchronization(mut htim:
                                                                *mut TIM_HandleTypeDef,
                                                            mut sSlaveConfig:
                                                                *mut TIM_SlaveConfigTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmpsmcr: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccmr1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*(*htim).Instance).SMCR;
    /* Reset the Trigger Selection Bits */
    tmpsmcr &= !((0x7 as libc::c_uint) << 4 as libc::c_uint);
    /* Set the Input Trigger source */
    tmpsmcr |= (*sSlaveConfig).InputTrigger;
    /* Reset the slave mode Bits */
    tmpsmcr &= !((0x10007 as libc::c_uint) << 0 as libc::c_uint);
    /* Set the slave mode */
    tmpsmcr |= (*sSlaveConfig).SlaveMode;
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t, tmpsmcr);
    /* Configure the trigger prescaler, filter, and polarity */
    match (*sSlaveConfig).InputTrigger {
        112 => {
            /* Check the parameters */
            /* Configure the ETR Trigger source */
            TIM_ETR_SetConfig((*htim).Instance,
                              (*sSlaveConfig).TriggerPrescaler,
                              (*sSlaveConfig).TriggerPolarity,
                              (*sSlaveConfig).TriggerFilter);
        }
        64 => {
            /* Check the parameters */
            /* Disable the Channel 1: Reset the CC1E Bit */
            tmpccer = (*(*htim).Instance).CCER;
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            tmpccmr1 = (*(*htim).Instance).CCMR1;
            tmpccmr1 &= !((0xf as libc::c_uint) << 4 as libc::c_uint);
            tmpccmr1 |= (*sSlaveConfig).TriggerFilter << 4 as libc::c_int;
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t, tmpccmr1);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCER as
                                            *mut uint32_t, tmpccer)
        }
        80 => {
            /* Set the filter */
            /* Write to TIMx CCMR1 and CCER registers */
            /* Check the parameters */
            /* Configure TI1 Filter and Polarity */
            TIM_TI1_ConfigInputStage((*htim).Instance,
                                     (*sSlaveConfig).TriggerPolarity,
                                     (*sSlaveConfig).TriggerFilter);
        }
        96 => {
            /* Check the parameters */
            /* Configure TI2 Filter and Polarity */
            TIM_TI2_ConfigInputStage((*htim).Instance,
                                     (*sSlaveConfig).TriggerPolarity,
                                     (*sSlaveConfig).TriggerFilter);
        }
        0 => { }
        16 => { }
        32 => { }
        48 | _ => { }
    }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Configures the TIM in Slave mode in interrupt mode
  * @param  htim: TIM handle.
  * @param  sSlaveConfig: pointer to a TIM_SlaveConfigTypeDef structure that
  *         contains the selected trigger (internal trigger input, filtered
  *         timer input or external trigger input) and the ) and the Slave 
  *         mode (Disable, Reset, Gated, Trigger, External clock mode 1). 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_SlaveConfigSynchronization_IT(mut htim:
                                                                   *mut TIM_HandleTypeDef,
                                                               mut sSlaveConfig:
                                                                   *mut TIM_SlaveConfigTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    TIM_SlaveTimer_SetConfig(htim, sSlaveConfig);
    /* Enable Trigger Interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         6 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Disable Trigger DMA request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           14 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Read the captured value from Capture Compare unit
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval Captured value
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_ReadCapturedValue(mut htim:
                                                       *mut TIM_HandleTypeDef,
                                                   mut Channel: uint32_t)
 -> uint32_t {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY as libc::c_int as uint32_t
    } else { (*htim).Lock = HAL_LOCKED }
    match Channel {
        0 => {
            /* Check the parameters */
            /* Return the capture 1 value */
            tmpreg = (*(*htim).Instance).CCR1
        }
        4 => {
            /* Check the parameters */
            /* Return the capture 2 value */
            tmpreg = (*(*htim).Instance).CCR2
        }
        8 => {
            /* Check the parameters */
            /* Return the capture 3 value */
            tmpreg = (*(*htim).Instance).CCR3
        }
        12 => {
            /* Check the parameters */
            /* Return the capture 4 value */
            tmpreg = (*(*htim).Instance).CCR4
        }
        _ => { }
    }
    (*htim).Lock = HAL_UNLOCKED;
    return tmpreg;
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group9 TIM Callbacks functions
 *  @brief    TIM Callbacks functions 
 *
@verbatim   
  ==============================================================================
                        ##### TIM Callbacks functions #####
  ==============================================================================  
 [..]  
   This section provides TIM callback functions:
   (+) Timer Period elapsed callback
   (+) Timer Output Compare callback
   (+) Timer Input capture callback
   (+) Timer Trigger callback
   (+) Timer Error callback

@endverbatim
  * @{
  */
/* *
  * @brief  Period elapsed callback in non blocking mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PeriodElapsedCallback(mut htim:
                                                           *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}
/* *
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_DelayElapsedCallback(mut htim:
                                                             *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_OC_DelayElapsedCallback could be implemented in the user file
   */
}
/* *
  * @brief  Input Capture callback in non blocking mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_CaptureCallback(mut htim:
                                                        *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
}
/* *
  * @brief  PWM Pulse finished callback in non blocking mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_PulseFinishedCallback(mut htim:
                                                               *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PWM_PulseFinishedCallback could be implemented in the user file
   */
}
/* *
  * @brief  Hall Trigger detection callback in non blocking mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_TriggerCallback(mut htim:
                                                     *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_TriggerCallback could be implemented in the user file
   */
}
/* *
  * @brief  Timer error callback in non blocking mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_ErrorCallback(mut htim:
                                                   *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_ErrorCallback could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup TIM_Exported_Functions_Group10 Peripheral State functions 
 *  @brief   Peripheral State functions 
 *
@verbatim   
  ==============================================================================
                        ##### Peripheral State functions #####
  ==============================================================================  
  [..]
    This subsection permits to get in run-time the status of the peripheral 
    and the data flow.

@endverbatim
  * @{
  */
/* *
  * @brief  Return the TIM Base state
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Base_GetState(mut htim:
                                                   *mut TIM_HandleTypeDef)
 -> HAL_TIM_StateTypeDef {
    return (*htim).State;
}
/* *
  * @brief  Return the TIM OC state
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OC_GetState(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_TIM_StateTypeDef {
    return (*htim).State;
}
/* *
  * @brief  Return the TIM PWM state
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_GetState(mut htim:
                                                  *mut TIM_HandleTypeDef)
 -> HAL_TIM_StateTypeDef {
    return (*htim).State;
}
/* *
  * @brief  Return the TIM Input Capture state
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_IC_GetState(mut htim: *mut TIM_HandleTypeDef)
 -> HAL_TIM_StateTypeDef {
    return (*htim).State;
}
/* *
  * @brief  Return the TIM One Pulse Mode state
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_OnePulse_GetState(mut htim:
                                                       *mut TIM_HandleTypeDef)
 -> HAL_TIM_StateTypeDef {
    return (*htim).State;
}
/* *
  * @brief  Return the TIM Encoder Mode state
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_Encoder_GetState(mut htim:
                                                      *mut TIM_HandleTypeDef)
 -> HAL_TIM_StateTypeDef {
    return (*htim).State;
}
/* *
  * @}
  */
/* *
  * @brief  TIM DMA error callback 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_DMAError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut htim: *mut TIM_HandleTypeDef =
        (*hdma).Parent as *mut TIM_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    HAL_TIM_ErrorCallback(htim);
}
/* *
  * @brief  TIM DMA Delay Pulse complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_DMADelayPulseCplt(mut hdma:
                                                   *mut DMA_HandleTypeDef) {
    let mut htim: *mut TIM_HandleTypeDef =
        (*hdma).Parent as *mut TIM_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    if hdma == (*htim).hdma[0x1 as libc::c_uint as uint16_t as usize] {
        (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_1
    } else if hdma == (*htim).hdma[0x2 as libc::c_uint as uint16_t as usize] {
        (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_2
    } else if hdma == (*htim).hdma[0x3 as libc::c_uint as uint16_t as usize] {
        (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_3
    } else if hdma == (*htim).hdma[0x4 as libc::c_uint as uint16_t as usize] {
        (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_4
    }
    HAL_TIM_PWM_PulseFinishedCallback(htim);
    (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}
/* *
  * @brief  TIM DMA Capture complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_DMACaptureCplt(mut hdma:
                                                *mut DMA_HandleTypeDef) {
    let mut htim: *mut TIM_HandleTypeDef =
        (*hdma).Parent as *mut TIM_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    if hdma == (*htim).hdma[0x1 as libc::c_uint as uint16_t as usize] {
        (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_1
    } else if hdma == (*htim).hdma[0x2 as libc::c_uint as uint16_t as usize] {
        (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_2
    } else if hdma == (*htim).hdma[0x3 as libc::c_uint as uint16_t as usize] {
        (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_3
    } else if hdma == (*htim).hdma[0x4 as libc::c_uint as uint16_t as usize] {
        (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_4
    }
    HAL_TIM_IC_CaptureCallback(htim);
    (*htim).Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}
/* *
  * @brief  TIM DMA Period Elapse complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn TIM_DMAPeriodElapsedCplt(mut hdma:
                                                  *mut DMA_HandleTypeDef) {
    let mut htim: *mut TIM_HandleTypeDef =
        (*hdma).Parent as *mut TIM_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    HAL_TIM_PeriodElapsedCallback(htim);
}
/* *
  * @brief  TIM DMA Trigger callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn TIM_DMATriggerCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut htim: *mut TIM_HandleTypeDef =
        (*hdma).Parent as *mut TIM_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    HAL_TIM_TriggerCallback(htim);
}
/* *
  * @brief  Time Base configuration
  * @param  TIMx: TIM peripheral
  * @param  Structure: pointer on TIM Time Base required parameters  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_Base_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                            mut Structure:
                                                *mut TIM_Base_InitTypeDef) {
    let mut tmpcr1: uint32_t = 0 as libc::c_int as uint32_t;
    tmpcr1 = (*TIMx).CR1;
    /* Set TIM Time Base Unit parameters ---------------------------------------*/
    if (TIMx ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0xc00 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Select the Counter Mode */
        tmpcr1 &=
            !((0x1 as libc::c_uint) << 4 as libc::c_uint |
                  (0x3 as libc::c_uint) << 5 as libc::c_uint);
        tmpcr1 |= (*Structure).CounterMode
    }
    if (TIMx ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0xc00 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x4000
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x4400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x4800
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x1800 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x1c00 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x2000 as libc::c_uint) as
                    *mut TIM_TypeDef) as libc::c_int != RESET as libc::c_int {
        /* Set the clock division */
        tmpcr1 &= !((0x3 as libc::c_uint) << 8 as libc::c_uint);
        tmpcr1 |= (*Structure).ClockDivision
    }
    /* Set the auto-reload preload */
    tmpcr1 =
        tmpcr1 & !((0x1 as libc::c_uint) << 7 as libc::c_uint) |
            (*Structure).AutoReloadPreload;
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint32_t, tmpcr1);
    /* Set the Auto-reload value */
    ::core::ptr::write_volatile(&mut (*TIMx).ARR as *mut uint32_t,
                                (*Structure).Period);
    /* Set the Prescaler value */
    ::core::ptr::write_volatile(&mut (*TIMx).PSC as *mut uint32_t,
                                (*Structure).Prescaler);
    if (TIMx ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Set the Repetition Counter value */
        ::core::ptr::write_volatile(&mut (*TIMx).RCR as *mut uint32_t,
                                    (*Structure).RepetitionCounter)
    }
    /* Generate an update event to reload the Prescaler 
     and the repetition counter(only for TIM1 and TIM8) value immediately */
    ::core::ptr::write_volatile(&mut (*TIMx).EGR as *mut uint32_t,
                                (0x1 as libc::c_uint) << 0 as libc::c_uint);
}
/* *
  * @brief  Time Output Compare 1 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config: The output configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC1_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut OC_Config:
                                               *mut TIM_OC_InitTypeDef) {
    let mut tmpccmrx: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpcr2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 1: Reset the CC1E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmrx = (*TIMx).CCMR1;
    /* Reset the Output Compare Mode Bits */
    tmpccmrx &= !((0x1007 as libc::c_uint) << 4 as libc::c_uint);
    tmpccmrx &= !((0x3 as libc::c_uint) << 0 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*OC_Config).OCMode;
    /* Reset the Output Polarity level */
    tmpccer &= !((0x1 as libc::c_uint) << 1 as libc::c_uint);
    /* Set the Output Compare Polarity */
    tmpccer |= (*OC_Config).OCPolarity;
    if (TIMx ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Reset the Output N Polarity level */
        tmpccer &= !((0x1 as libc::c_uint) << 3 as libc::c_uint);
        /* Set the Output N Polarity */
        tmpccer |= (*OC_Config).OCNPolarity;
        /* Reset the Output N State */
        tmpccer &= !((0x1 as libc::c_uint) << 2 as libc::c_uint);
        /* Reset the Output Compare and Output Compare N IDLE State */
        tmpcr2 &= !((0x1 as libc::c_uint) << 8 as libc::c_uint);
        tmpcr2 &= !((0x1 as libc::c_uint) << 9 as libc::c_uint);
        /* Set the Output Idle state */
        tmpcr2 |= (*OC_Config).OCIdleState;
        /* Set the Output N Idle state */
        tmpcr2 |= (*OC_Config).OCNIdleState
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR1 as *mut uint32_t,
                                (*OC_Config).Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Time Output Compare 2 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config: The output configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC2_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut OC_Config:
                                               *mut TIM_OC_InitTypeDef) {
    let mut tmpccmrx: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpcr2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 2: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmrx = (*TIMx).CCMR1;
    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= !((0x1007 as libc::c_uint) << 12 as libc::c_uint);
    tmpccmrx &= !((0x3 as libc::c_uint) << 8 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*OC_Config).OCMode << 8 as libc::c_int;
    /* Reset the Output Polarity level */
    tmpccer &= !((0x1 as libc::c_uint) << 5 as libc::c_uint);
    /* Set the Output Compare Polarity */
    tmpccer |= (*OC_Config).OCPolarity << 4 as libc::c_int;
    if (TIMx ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Reset the Output N Polarity level */
        tmpccer &= !((0x1 as libc::c_uint) << 7 as libc::c_uint);
        /* Set the Output N Polarity */
        tmpccer |= (*OC_Config).OCNPolarity << 4 as libc::c_int;
        /* Reset the Output N State */
        tmpccer &= !((0x1 as libc::c_uint) << 6 as libc::c_uint);
        /* Reset the Output Compare and Output Compare N IDLE State */
        tmpcr2 &= !((0x1 as libc::c_uint) << 10 as libc::c_uint);
        tmpcr2 &= !((0x1 as libc::c_uint) << 11 as libc::c_uint);
        /* Set the Output Idle state */
        tmpcr2 |= (*OC_Config).OCIdleState << 2 as libc::c_int;
        /* Set the Output N Idle state */
        tmpcr2 |= (*OC_Config).OCNIdleState << 2 as libc::c_int
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR2 as *mut uint32_t,
                                (*OC_Config).Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Time Output Compare 3 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config: The output configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC3_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut OC_Config:
                                               *mut TIM_OC_InitTypeDef) {
    let mut tmpccmrx: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpcr2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 3: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR2 register value */
    tmpccmrx = (*TIMx).CCMR2;
    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= !((0x1007 as libc::c_uint) << 4 as libc::c_uint);
    tmpccmrx &= !((0x3 as libc::c_uint) << 0 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*OC_Config).OCMode;
    /* Reset the Output Polarity level */
    tmpccer &= !((0x1 as libc::c_uint) << 9 as libc::c_uint);
    /* Set the Output Compare Polarity */
    tmpccer |= (*OC_Config).OCPolarity << 8 as libc::c_int;
    if (TIMx ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Reset the Output N Polarity level */
        tmpccer &= !((0x1 as libc::c_uint) << 11 as libc::c_uint);
        /* Set the Output N Polarity */
        tmpccer |= (*OC_Config).OCNPolarity << 8 as libc::c_int;
        /* Reset the Output N State */
        tmpccer &= !((0x1 as libc::c_uint) << 10 as libc::c_uint);
        /* Reset the Output Compare and Output Compare N IDLE State */
        tmpcr2 &= !((0x1 as libc::c_uint) << 12 as libc::c_uint);
        tmpcr2 &= !((0x1 as libc::c_uint) << 13 as libc::c_uint);
        /* Set the Output Idle state */
        tmpcr2 |= (*OC_Config).OCIdleState << 4 as libc::c_int;
        /* Set the Output N Idle state */
        tmpcr2 |= (*OC_Config).OCNIdleState << 4 as libc::c_int
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR3 as *mut uint32_t,
                                (*OC_Config).Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Time Output Compare 4 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config: The output configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC4_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut OC_Config:
                                               *mut TIM_OC_InitTypeDef) {
    let mut tmpccmrx: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpcr2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 4: Reset the CC4E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           12 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR2 register value */
    tmpccmrx = (*TIMx).CCMR2;
    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= !((0x1007 as libc::c_uint) << 12 as libc::c_uint);
    tmpccmrx &= !((0x3 as libc::c_uint) << 8 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*OC_Config).OCMode << 8 as libc::c_int;
    /* Reset the Output Polarity level */
    tmpccer &= !((0x1 as libc::c_uint) << 13 as libc::c_uint);
    /* Set the Output Compare Polarity */
    tmpccer |= (*OC_Config).OCPolarity << 12 as libc::c_int;
    /*if((TIMx == TIM1) || (TIMx == TIM8))*/
    if (TIMx ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef) as libc::c_int !=
           RESET as libc::c_int {
        /* Reset the Output Compare IDLE State */
        tmpcr2 &= !((0x1 as libc::c_uint) << 14 as libc::c_uint);
        /* Set the Output Idle state */
        tmpcr2 |= (*OC_Config).OCIdleState << 6 as libc::c_int
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR4 as *mut uint32_t,
                                (*OC_Config).Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Time Output Compare 4 configuration
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sSlaveConfig: The slave configuration structure
  * @retval None
  */
unsafe extern "C" fn TIM_SlaveTimer_SetConfig(mut htim:
                                                  *mut TIM_HandleTypeDef,
                                              mut sSlaveConfig:
                                                  *mut TIM_SlaveConfigTypeDef) {
    let mut tmpsmcr: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccmr1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*(*htim).Instance).SMCR;
    /* Reset the Trigger Selection Bits */
    tmpsmcr &= !((0x7 as libc::c_uint) << 4 as libc::c_uint);
    /* Set the Input Trigger source */
    tmpsmcr |= (*sSlaveConfig).InputTrigger;
    /* Reset the slave mode Bits */
    tmpsmcr &= !((0x10007 as libc::c_uint) << 0 as libc::c_uint);
    /* Set the slave mode */
    tmpsmcr |= (*sSlaveConfig).SlaveMode;
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t, tmpsmcr);
    /* Configure the trigger prescaler, filter, and polarity */
    match (*sSlaveConfig).InputTrigger {
        112 => {
            /* Check the parameters */
            /* Configure the ETR Trigger source */
            TIM_ETR_SetConfig((*htim).Instance,
                              (*sSlaveConfig).TriggerPrescaler,
                              (*sSlaveConfig).TriggerPolarity,
                              (*sSlaveConfig).TriggerFilter);
        }
        64 => {
            /* Check the parameters */
            /* Disable the Channel 1: Reset the CC1E Bit */
            tmpccer = (*(*htim).Instance).CCER;
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCER as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            tmpccmr1 = (*(*htim).Instance).CCMR1;
            tmpccmr1 &= !((0xf as libc::c_uint) << 4 as libc::c_uint);
            tmpccmr1 |= (*sSlaveConfig).TriggerFilter << 4 as libc::c_int;
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t, tmpccmr1);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCER as
                                            *mut uint32_t, tmpccer)
        }
        80 => {
            /* Set the filter */
            /* Write to TIMx CCMR1 and CCER registers */
            /* Check the parameters */
            /* Configure TI1 Filter and Polarity */
            TIM_TI1_ConfigInputStage((*htim).Instance,
                                     (*sSlaveConfig).TriggerPolarity,
                                     (*sSlaveConfig).TriggerFilter);
        }
        96 => {
            /* Check the parameters */
            /* Configure TI2 Filter and Polarity */
            TIM_TI2_ConfigInputStage((*htim).Instance,
                                     (*sSlaveConfig).TriggerPolarity,
                                     (*sSlaveConfig).TriggerFilter);
        }
        0 => { }
        16 => { }
        32 => { }
        48 | _ => { }
    };
}
/* *
  * @brief  Configure the TI1 as Input.
  * @param  TIMx to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge  
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 1 is selected to be connected to IC1.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 1 is selected to be connected to IC2.
  *            @arg TIM_ICSelection_TRC: TIM Input 1 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None  
  * @note TIM_ICFilter and TIM_ICPolarity are not used in INDIRECT mode as TI2FP1 
  *       (on channel2 path) is used as the input signal. Therefore CCMR1 must be 
  *        protected against un-initialized filter and polarity values.  
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_TI1_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut TIM_ICPolarity: uint32_t,
                                           mut TIM_ICSelection: uint32_t,
                                           mut TIM_ICFilter: uint32_t) {
    let mut tmpccmr1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 1: Reset the CC1E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    tmpccmr1 = (*TIMx).CCMR1;
    tmpccer = (*TIMx).CCER;
    /* Select the Input */
    if (TIMx ==
            (0x40000000 as
                 libc::c_uint).wrapping_add(0x10000 as
                                                libc::c_uint).wrapping_add(0
                                                                               as
                                                                               libc::c_uint)
                as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0xc00 as libc::c_uint) as
                    *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x400
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x10000 as
                                                    libc::c_uint).wrapping_add(0x4000
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut TIM_TypeDef ||
            TIMx ==
                (0x40000000 as
                     libc::c_uint).wrapping_add(0x1800 as libc::c_uint) as
                    *mut TIM_TypeDef) as libc::c_int != RESET as libc::c_int {
        tmpccmr1 &= !((0x3 as libc::c_uint) << 0 as libc::c_uint);
        tmpccmr1 |= TIM_ICSelection
    } else { tmpccmr1 |= (0x1 as libc::c_uint) << 0 as libc::c_uint }
    /* Set the filter */
    tmpccmr1 &= !((0xf as libc::c_uint) << 4 as libc::c_uint);
    tmpccmr1 |=
        TIM_ICFilter << 4 as libc::c_int &
            (0xf as libc::c_uint) << 4 as libc::c_uint;
    /* Select the Polarity and set the CC1E Bit */
    tmpccer &=
        !((0x1 as libc::c_uint) << 1 as libc::c_uint |
              (0x1 as libc::c_uint) << 3 as libc::c_uint);
    tmpccer |=
        TIM_ICPolarity &
            ((0x1 as libc::c_uint) << 1 as libc::c_uint |
                 (0x1 as libc::c_uint) << 3 as libc::c_uint);
    /* Write to TIMx CCMR1 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_tim.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   TIM HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Timer (TIM) peripheral:
  *           + Time Base Initialization
  *           + Time Base Start
  *           + Time Base Start Interruption
  *           + Time Base Start DMA
  *           + Time Output Compare/PWM Initialization
  *           + Time Output Compare/PWM Channel Configuration
  *           + Time Output Compare/PWM  Start
  *           + Time Output Compare/PWM  Start Interruption
  *           + Time Output Compare/PWM Start DMA
  *           + Time Input Capture Initialization
  *           + Time Input Capture Channel Configuration
  *           + Time Input Capture Start
  *           + Time Input Capture Start Interruption 
  *           + Time Input Capture Start DMA
  *           + Time One Pulse Initialization
  *           + Time One Pulse Channel Configuration
  *           + Time One Pulse Start 
  *           + Time Encoder Interface Initialization
  *           + Time Encoder Interface Start
  *           + Time Encoder Interface Start Interruption
  *           + Time Encoder Interface Start DMA
  *           + Commutation Event configuration with Interruption and DMA
  *           + Time OCRef clear configuration
  *           + Time External Clock configuration
  @verbatim 
  ==============================================================================
                      ##### TIMER Generic features #####
  ==============================================================================
  [..] The Timer features include: 
       (#) 16-bit up, down, up/down auto-reload counter.
       (#) 16-bit programmable prescaler allowing dividing (also on the fly) the 
           counter clock frequency either by any factor between 1 and 65536.
       (#) Up to 4 independent channels for:
           (++) Input Capture
           (++) Output Compare
           (++) PWM generation (Edge and Center-aligned Mode)
           (++) One-pulse mode output               
   
                        ##### How to use this driver #####
  ==============================================================================
    [..]
     (#) Initialize the TIM low level resources by implementing the following functions 
         depending from feature used :
           (++) Time Base : HAL_TIM_Base_MspInit() 
           (++) Input Capture : HAL_TIM_IC_MspInit()
           (++) Output Compare : HAL_TIM_OC_MspInit()
           (++) PWM generation : HAL_TIM_PWM_MspInit()
           (++) One-pulse mode output : HAL_TIM_OnePulse_MspInit()
           (++) Encoder mode output : HAL_TIM_Encoder_MspInit()
           
     (#) Initialize the TIM low level resources :
        (##) Enable the TIM interface clock using __HAL_RCC_TIMx_CLK_ENABLE(); 
        (##) TIM pins configuration
            (+++) Enable the clock for the TIM GPIOs using the following function:
                 __HAL_RCC_GPIOx_CLK_ENABLE();   
            (+++) Configure these TIM pins in Alternate function mode using HAL_GPIO_Init();  

     (#) The external Clock can be configured, if needed (the default clock is the 
         internal clock from the APBx), using the following function:
         HAL_TIM_ConfigClockSource, the clock configuration should be done before 
         any start function.
  
     (#) Configure the TIM in the desired functioning mode using one of the 
         initialization function of this driver:
         (++) HAL_TIM_Base_Init: to use the Timer to generate a simple time base
         (++) HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal.
         (++) HAL_TIM_PWM_Init and HAL_TIM_PWM_ConfigChannel: to use the Timer to generate a 
              PWM signal.
         (++) HAL_TIM_IC_Init and HAL_TIM_IC_ConfigChannel: to use the Timer to measure an 
              external signal.
         (++) HAL_TIM_OnePulse_Init and HAL_TIM_OnePulse_ConfigChannel: to use the Timer 
              in One Pulse Mode.
         (++) HAL_TIM_Encoder_Init: to use the Timer Encoder Interface.
         
     (#) Activate the TIM peripheral using one of the start functions depending from the feature used: 
           (++) Time Base : HAL_TIM_Base_Start(), HAL_TIM_Base_Start_DMA(), HAL_TIM_Base_Start_IT()
           (++) Input Capture :  HAL_TIM_IC_Start(), HAL_TIM_IC_Start_DMA(), HAL_TIM_IC_Start_IT()
           (++) Output Compare : HAL_TIM_OC_Start(), HAL_TIM_OC_Start_DMA(), HAL_TIM_OC_Start_IT()
           (++) PWM generation : HAL_TIM_PWM_Start(), HAL_TIM_PWM_Start_DMA(), HAL_TIM_PWM_Start_IT()
           (++) One-pulse mode output : HAL_TIM_OnePulse_Start(), HAL_TIM_OnePulse_Start_IT()
           (++) Encoder mode output : HAL_TIM_Encoder_Start(), HAL_TIM_Encoder_Start_DMA(), HAL_TIM_Encoder_Start_IT().

     (#) The DMA Burst is managed with the two following functions:
         HAL_TIM_DMABurst_WriteStart()
         HAL_TIM_DMABurst_ReadStart()
  
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
/* * @defgroup TIM TIM
  * @brief TIM HAL module driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @addtogroup TIM_Private_Functions
  * @{
  */
/* Private function prototypes -----------------------------------------------*/
/* *
  * @brief  Configure the Polarity and Filter for TI1.
  * @param  TIMx to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
unsafe extern "C" fn TIM_TI1_ConfigInputStage(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_ICPolarity: uint32_t,
                                              mut TIM_ICFilter: uint32_t) {
    let mut tmpccmr1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 1: Reset the CC1E Bit */
    tmpccer = (*TIMx).CCER;
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    tmpccmr1 = (*TIMx).CCMR1;
    /* Set the filter */
    tmpccmr1 &= !((0xf as libc::c_uint) << 4 as libc::c_uint);
    tmpccmr1 |= TIM_ICFilter << 4 as libc::c_int;
    /* Select the Polarity and set the CC1E Bit */
    tmpccer &=
        !((0x1 as libc::c_uint) << 1 as libc::c_uint |
              (0x1 as libc::c_uint) << 3 as libc::c_uint);
    tmpccer |= TIM_ICPolarity;
    /* Write to TIMx CCMR1 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configure the TI2 as Input.
  * @param  TIMx to select the TIM peripheral
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge   
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 2 is selected to be connected to IC2.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 2 is selected to be connected to IC1.
  *            @arg TIM_ICSelection_TRC: TIM Input 2 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  * @note TIM_ICFilter and TIM_ICPolarity are not used in INDIRECT mode as TI1FP2 
  *       (on channel1 path) is used as the input signal. Therefore CCMR1 must be 
  *        protected against un-initialized filter and polarity values.  
  */
unsafe extern "C" fn TIM_TI2_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                       mut TIM_ICPolarity: uint32_t,
                                       mut TIM_ICSelection: uint32_t,
                                       mut TIM_ICFilter: uint32_t) {
    let mut tmpccmr1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 2: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    tmpccmr1 = (*TIMx).CCMR1;
    tmpccer = (*TIMx).CCER;
    /* Select the Input */
    tmpccmr1 &= !((0x3 as libc::c_uint) << 8 as libc::c_uint);
    tmpccmr1 |= TIM_ICSelection << 8 as libc::c_int;
    /* Set the filter */
    tmpccmr1 &= !((0xf as libc::c_uint) << 12 as libc::c_uint);
    tmpccmr1 |=
        TIM_ICFilter << 12 as libc::c_int &
            (0xf as libc::c_uint) << 12 as libc::c_uint;
    /* Select the Polarity and set the CC2E Bit */
    tmpccer &=
        !((0x1 as libc::c_uint) << 5 as libc::c_uint |
              (0x1 as libc::c_uint) << 7 as libc::c_uint);
    tmpccer |=
        TIM_ICPolarity << 4 as libc::c_int &
            ((0x1 as libc::c_uint) << 5 as libc::c_uint |
                 (0x1 as libc::c_uint) << 7 as libc::c_uint);
    /* Write to TIMx CCMR1 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configure the Polarity and Filter for TI2.
  * @param  TIMx to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
unsafe extern "C" fn TIM_TI2_ConfigInputStage(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_ICPolarity: uint32_t,
                                              mut TIM_ICFilter: uint32_t) {
    let mut tmpccmr1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 2: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    tmpccmr1 = (*TIMx).CCMR1;
    tmpccer = (*TIMx).CCER;
    /* Set the filter */
    tmpccmr1 &= !((0xf as libc::c_uint) << 12 as libc::c_uint);
    tmpccmr1 |= TIM_ICFilter << 12 as libc::c_int;
    /* Select the Polarity and set the CC2E Bit */
    tmpccer &=
        !((0x1 as libc::c_uint) << 5 as libc::c_uint |
              (0x1 as libc::c_uint) << 7 as libc::c_uint);
    tmpccer |= TIM_ICPolarity << 4 as libc::c_int;
    /* Write to TIMx CCMR1 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configure the TI3 as Input.
  * @param  TIMx to select the TIM peripheral
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge         
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 3 is selected to be connected to IC3.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 3 is selected to be connected to IC4.
  *            @arg TIM_ICSelection_TRC: TIM Input 3 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  * @note TIM_ICFilter and TIM_ICPolarity are not used in INDIRECT mode as TI3FP4 
  *       (on channel1 path) is used as the input signal. Therefore CCMR2 must be 
  *        protected against un-initialized filter and polarity values.  
  */
unsafe extern "C" fn TIM_TI3_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                       mut TIM_ICPolarity: uint32_t,
                                       mut TIM_ICSelection: uint32_t,
                                       mut TIM_ICFilter: uint32_t) {
    let mut tmpccmr2: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 3: Reset the CC3E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    tmpccmr2 = (*TIMx).CCMR2;
    tmpccer = (*TIMx).CCER;
    /* Select the Input */
    tmpccmr2 &= !((0x3 as libc::c_uint) << 0 as libc::c_uint);
    tmpccmr2 |= TIM_ICSelection;
    /* Set the filter */
    tmpccmr2 &= !((0xf as libc::c_uint) << 4 as libc::c_uint);
    tmpccmr2 |=
        TIM_ICFilter << 4 as libc::c_int &
            (0xf as libc::c_uint) << 4 as libc::c_uint;
    /* Select the Polarity and set the CC3E Bit */
    tmpccer &=
        !((0x1 as libc::c_uint) << 9 as libc::c_uint |
              (0x1 as libc::c_uint) << 11 as libc::c_uint);
    tmpccer |=
        TIM_ICPolarity << 8 as libc::c_int &
            ((0x1 as libc::c_uint) << 9 as libc::c_uint |
                 (0x1 as libc::c_uint) << 11 as libc::c_uint);
    /* Write to TIMx CCMR2 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configure the TI4 as Input.
  * @param  TIMx to select the TIM peripheral
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge     
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 4 is selected to be connected to IC4.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 4 is selected to be connected to IC3.
  *            @arg TIM_ICSelection_TRC: TIM Input 4 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  * @note TIM_ICFilter and TIM_ICPolarity are not used in INDIRECT mode as TI4FP3 
  *       (on channel1 path) is used as the input signal. Therefore CCMR2 must be 
  *        protected against un-initialized filter and polarity values.  
  */
unsafe extern "C" fn TIM_TI4_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                       mut TIM_ICPolarity: uint32_t,
                                       mut TIM_ICSelection: uint32_t,
                                       mut TIM_ICFilter: uint32_t) {
    let mut tmpccmr2: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the Channel 4: Reset the CC4E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           12 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    tmpccmr2 = (*TIMx).CCMR2;
    tmpccer = (*TIMx).CCER;
    /* Select the Input */
    tmpccmr2 &= !((0x3 as libc::c_uint) << 8 as libc::c_uint);
    tmpccmr2 |= TIM_ICSelection << 8 as libc::c_int;
    /* Set the filter */
    tmpccmr2 &= !((0xf as libc::c_uint) << 12 as libc::c_uint);
    tmpccmr2 |=
        TIM_ICFilter << 12 as libc::c_int &
            (0xf as libc::c_uint) << 12 as libc::c_uint;
    /* Select the Polarity and set the CC4E Bit */
    tmpccer &=
        !((0x1 as libc::c_uint) << 13 as libc::c_uint |
              (0x1 as libc::c_uint) << 15 as libc::c_uint);
    tmpccer |=
        TIM_ICPolarity << 12 as libc::c_int &
            ((0x1 as libc::c_uint) << 13 as libc::c_uint |
                 (0x1 as libc::c_uint) << 15 as libc::c_uint);
    /* Write to TIMx CCMR2 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Selects the Input Trigger source
  * @param  TIMx to select the TIM peripheral
  * @param  TIM_ITRx: The Input Trigger source.
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal Trigger 0
  *            @arg TIM_TS_ITR1: Internal Trigger 1
  *            @arg TIM_TS_ITR2: Internal Trigger 2
  *            @arg TIM_TS_ITR3: Internal Trigger 3
  *            @arg TIM_TS_TI1F_ED: TI1 Edge Detector
  *            @arg TIM_TS_TI1FP1: Filtered Timer Input 1
  *            @arg TIM_TS_TI2FP2: Filtered Timer Input 2
  *            @arg TIM_TS_ETRF: External Trigger input
  * @retval None
  */
unsafe extern "C" fn TIM_ITRx_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                        mut TIM_ITRx: uint16_t) {
    let mut tmpsmcr: uint32_t = 0 as libc::c_int as uint32_t;
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*TIMx).SMCR;
    /* Reset the TS Bits */
    tmpsmcr &= !((0x7 as libc::c_uint) << 4 as libc::c_uint);
    /* Set the Input Trigger source and the slave mode*/
    tmpsmcr |=
        TIM_ITRx as libc::c_uint |
            ((0x4 as libc::c_uint) << 0 as libc::c_uint |
                 (0x2 as libc::c_uint) << 0 as libc::c_uint |
                 (0x1 as libc::c_uint) << 0 as libc::c_uint);
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t, tmpsmcr);
}
/* *
  * @brief  Configures the TIMx External Trigger (ETR).
  * @param  TIMx to select the TIM peripheral
  * @param  TIM_ExtTRGPrescaler: The external Trigger Prescaler.
  *          This parameter can be one of the following values:
  *            @arg TIM_ExtTRGPSC_DIV1: ETRP Prescaler OFF.
  *            @arg TIM_ExtTRGPSC_DIV2: ETRP frequency divided by 2.
  *            @arg TIM_ExtTRGPSC_DIV4: ETRP frequency divided by 4.
  *            @arg TIM_ExtTRGPSC_DIV8: ETRP frequency divided by 8.
  * @param  TIM_ExtTRGPolarity: The external Trigger Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ExtTRGPolarity_Inverted: active low or falling edge active.
  *            @arg TIM_ExtTRGPolarity_NonInverted: active high or rising edge active.
  * @param  ExtTRGFilter: External Trigger Filter.
  *          This parameter must be a value between 0x00 and 0x0F
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ETR_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut TIM_ExtTRGPrescaler: uint32_t,
                                           mut TIM_ExtTRGPolarity: uint32_t,
                                           mut ExtTRGFilter: uint32_t) {
    let mut tmpsmcr: uint32_t = 0 as libc::c_int as uint32_t;
    tmpsmcr = (*TIMx).SMCR;
    /* Reset the ETR Bits */
    tmpsmcr &=
        !((0xf as libc::c_uint) << 8 as libc::c_uint |
              (0x3 as libc::c_uint) << 12 as libc::c_uint |
              (0x1 as libc::c_uint) << 14 as libc::c_uint |
              (0x1 as libc::c_uint) << 15 as libc::c_uint);
    /* Set the Prescaler, the Filter value and the Polarity */
    tmpsmcr |=
        TIM_ExtTRGPrescaler |
            (TIM_ExtTRGPolarity | ExtTRGFilter << 8 as libc::c_int);
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t, tmpsmcr);
}
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
/* !< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */
/* !< Specifies the counter mode.
                                   This parameter can be a value of @ref TIM_Counter_Mode */
/* !< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.  */
/* !< Specifies the clock division.
                                   This parameter can be a value of @ref TIM_ClockDivision */
/* !< Specifies the repetition counter value. Each time the RCR down-counter
                                    reaches zero, an update event is generated and counting restarts
                                    from the RCR value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                     This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. 
                                     @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the auto-reload preload.
                                   This parameter can be a value of @ref TIM_AutoReloadPreload */
/* * 
  * @brief  TIM Output Compare Configuration Structure definition  
  */
/* !< Specifies the TIM mode.
                               This parameter can be a value of @ref TIMEx_Output_Compare_and_PWM_modes */
/* !< Specifies the pulse value to be loaded into the Capture Compare Register. 
                               This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */
/* !< Specifies the output polarity.
                               This parameter can be a value of @ref TIM_Output_Compare_Polarity */
/* !< Specifies the complementary output polarity.
                               This parameter can be a value of @ref TIM_Output_Compare_N_Polarity
                               @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the Fast mode state.
                               This parameter can be a value of @ref TIM_Output_Fast_State
                               @note This parameter is valid only in PWM1 and PWM2 mode. */
/* !< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TIM_Output_Compare_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */
/* * 
  * @brief  TIM One Pulse Mode Configuration Structure definition  
  */
/* !< Specifies the TIM mode.
                               This parameter can be a value of @ref TIMEx_Output_Compare_and_PWM_modes */
/* !< Specifies the pulse value to be loaded into the Capture Compare Register. 
                               This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */
/* !< Specifies the output polarity.
                               This parameter can be a value of @ref TIM_Output_Compare_Polarity */
/* !< Specifies the complementary output polarity.
                               This parameter can be a value of @ref TIM_Output_Compare_N_Polarity
                               @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TIM_Output_Compare_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TIM_Input_Capture_Polarity */
/* !< Specifies the input.
                              This parameter can be a value of @ref TIM_Input_Capture_Selection */
/* !< Specifies the input capture filter.
                              This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
/* * 
  * @brief  TIM Input Capture Configuration Structure definition  
  */
/* !< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TIM_Input_Capture_Polarity */
/* !< Specifies the input.
                              This parameter can be a value of @ref TIM_Input_Capture_Selection */
/* !< Specifies the Input Capture Prescaler.
                              This parameter can be a value of @ref TIM_Input_Capture_Prescaler */
/* !< Specifies the input capture filter.
                              This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
/* * 
  * @brief  TIM Encoder Configuration Structure definition  
  */
/* !< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TIM_Encoder_Mode */
/* !< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TIM_Input_Capture_Polarity */
/* !< Specifies the input.
                               This parameter can be a value of @ref TIM_Input_Capture_Selection */
/* !< Specifies the Input Capture Prescaler.
                               This parameter can be a value of @ref TIM_Input_Capture_Prescaler */
/* !< Specifies the input capture filter.
                               This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
/* !< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TIM_Input_Capture_Polarity */
/* !< Specifies the input.
                              This parameter can be a value of @ref TIM_Input_Capture_Selection */
/* !< Specifies the Input Capture Prescaler.
                               This parameter can be a value of @ref TIM_Input_Capture_Prescaler */
/* !< Specifies the input capture filter.
                               This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
/* * 
  * @brief  Clock Configuration Handle Structure definition  
  */
/* !< TIM clock sources. 
                                 This parameter can be a value of @ref TIM_Clock_Source */
/* !< TIM clock polarity. 
                                 This parameter can be a value of @ref TIM_Clock_Polarity */
/* !< TIM clock prescaler. 
                                 This parameter can be a value of @ref TIM_Clock_Prescaler */
/* !< TIM clock filter. 
                                This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
/* * 
  * @brief  Clear Input Configuration Handle Structure definition  
  */
/* !< TIM clear Input state. 
                                      This parameter can be ENABLE or DISABLE */
/* !< TIM clear Input sources. 
                                      This parameter can be a value of @ref TIMEx_ClearInput_Source */
/* !< TIM Clear Input polarity. 
                                      This parameter can be a value of @ref TIM_ClearInput_Polarity */
/* !< TIM Clear Input prescaler. 
                                      This parameter can be a value of @ref TIM_ClearInput_Prescaler */
/* !< TIM Clear Input filter. 
                                     This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
/* * 
  * @brief  TIM Slave configuration Structure definition  
  */
/* !< Slave mode selection 
                                  This parameter can be a value of @ref TIMEx_Slave_Mode */
/* !< Input Trigger source 
                                  This parameter can be a value of @ref TIM_Trigger_Selection */
/* !< Input Trigger polarity 
                                  This parameter can be a value of @ref TIM_Trigger_Polarity */
/* !< Input trigger prescaler 
                                  This parameter can be a value of @ref TIM_Trigger_Prescaler */
/* !< Input trigger filter 
                                  This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
/* * 
  * @brief  HAL State structures definition  
  */
/* !< Peripheral not yet initialized or disabled  */
/* !< Peripheral Initialized and ready for use    */
/* !< An internal process is ongoing              */
/* !< Timeout state                               */
/* !< Reception process is ongoing                */
/* * 
  * @brief  HAL Active channel structures definition  
  */
/* !< The active channel is 1     */
/* !< The active channel is 2     */
/* !< The active channel is 3     */
/* !< The active channel is 4     */
/* !< All active channels cleared */
/* * 
  * @brief  TIM Time Base Handle Structure definition  
  */
/* !< Register base address             */
/* !< TIM Time Base required parameters */
/* !< Active channel                    */
/* !< DMA Handlers array
                                             This array is accessed by a @ref DMA_Handle_index */
/* !< Locking object                    */
/* !< TIM operation state               */
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
  * @}
  */
/* * @addtogroup TIM_Exported_Functions_Group10
  * @{
  */
/* Peripheral State functions  **************************************************/
/* *
  * @}
  */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @defgroup TIM_Private_Macros TIM Private Macros
  * @{
  */
/* * @defgroup TIM_IS_TIM_Definitions TIM Private macros to check input parameters
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/* * @defgroup TIM_Private_Functions TIM Private Functions
  * @{
  */
/* *
  * @brief  Enables or disables the TIM Capture Compare Channel x.
  * @param  TIMx to select the TIM peripheral
  * @param  Channel: specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_Channel_1: TIM Channel 1
  *            @arg TIM_Channel_2: TIM Channel 2
  *            @arg TIM_Channel_3: TIM Channel 3
  *            @arg TIM_Channel_4: TIM Channel 4
  * @param  ChannelState: specifies the TIM Channel CCxE bit new state.
  *          This parameter can be: TIM_CCx_ENABLE or TIM_CCx_Disable. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_CCxChannelCmd(mut TIMx: *mut TIM_TypeDef,
                                           mut Channel: uint32_t,
                                           mut ChannelState: uint32_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmp = ((0x1 as libc::c_uint) << 0 as libc::c_uint) << Channel;
    /* Reset the CCxE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !tmp) as uint32_t as
                                    uint32_t);
    /* Set or reset the CCxE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ChannelState << Channel) as uint32_t as
                                    uint32_t);
}
/* *
  * @}
  */ 
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* HAL_TIM_MODULE_ENABLED */
/* *
  * @}
  */
