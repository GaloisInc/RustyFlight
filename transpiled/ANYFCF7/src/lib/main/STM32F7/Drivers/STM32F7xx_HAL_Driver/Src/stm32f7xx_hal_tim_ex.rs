use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
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
    #[no_mangle]
    fn TIM_OC2_SetConfig(TIMx: *mut TIM_TypeDef,
                         OC_Config: *mut TIM_OC_InitTypeDef);
    #[no_mangle]
    fn TIM_TI1_SetConfig(TIMx: *mut TIM_TypeDef, TIM_ICPolarity: uint32_t,
                         TIM_ICSelection: uint32_t, TIM_ICFilter: uint32_t);
    #[no_mangle]
    fn TIM_Base_SetConfig(TIMx: *mut TIM_TypeDef,
                          Structure: *mut TIM_Base_InitTypeDef);
    #[no_mangle]
    fn TIM_CCxChannelCmd(TIMx: *mut TIM_TypeDef, Channel: uint32_t,
                         ChannelState: uint32_t);
    #[no_mangle]
    fn TIM_DMAError(hdma: *mut DMA_HandleTypeDef);
    #[no_mangle]
    fn TIM_DMACaptureCplt(hdma: *mut DMA_HandleTypeDef);
    #[no_mangle]
    fn TIM_DMADelayPulseCplt(hdma: *mut DMA_HandleTypeDef);
    #[no_mangle]
    fn TIM_OC4_SetConfig(TIMx: *mut TIM_TypeDef,
                         OC_Config: *mut TIM_OC_InitTypeDef);
    #[no_mangle]
    fn TIM_OC3_SetConfig(TIMx: *mut TIM_TypeDef,
                         OC_Config: *mut TIM_OC_InitTypeDef);
    #[no_mangle]
    fn TIM_OC1_SetConfig(TIMx: *mut TIM_TypeDef,
                         OC_Config: *mut TIM_OC_InitTypeDef);
    #[no_mangle]
    fn TIM_ETR_SetConfig(TIMx: *mut TIM_TypeDef,
                         TIM_ExtTRGPrescaler: uint32_t,
                         TIM_ExtTRGPolarity: uint32_t,
                         ExtTRGFilter: uint32_t);
}
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
  * @brief TIM
  */
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
pub struct TIM_ClearInputConfigTypeDef {
    pub ClearInputState: uint32_t,
    pub ClearInputSource: uint32_t,
    pub ClearInputPolarity: uint32_t,
    pub ClearInputPrescaler: uint32_t,
    pub ClearInputFilter: uint32_t,
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
  ******************************************************************************
  * @file    stm32f7xx_hal_tim_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of TIM HAL Extension module.
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
/* * @addtogroup TIMEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/ 
/* * @defgroup TIMEx_Exported_Types TIM Exported Types
  * @{
  */
/* * 
  * @brief  TIM Hall sensor Configuration Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_HallSensor_InitTypeDef {
    pub IC1Polarity: uint32_t,
    pub IC1Prescaler: uint32_t,
    pub IC1Filter: uint32_t,
    pub Commutation_Delay: uint32_t,
}
/* * 
  * @brief  TIM Master configuration Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_MasterConfigTypeDef {
    pub MasterOutputTrigger: uint32_t,
    pub MasterOutputTrigger2: uint32_t,
    pub MasterSlaveMode: uint32_t,
}
/* * 
  * @brief  TIM Break input(s) and Dead time configuration Structure definition  
  * @note   2 break inputs can be configured (BKIN and BKIN2) with configurable 
  *        filter and polarity.
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_BreakDeadTimeConfigTypeDef {
    pub OffStateRunMode: uint32_t,
    pub OffStateIDLEMode: uint32_t,
    pub LockLevel: uint32_t,
    pub DeadTime: uint32_t,
    pub BreakState: uint32_t,
    pub BreakPolarity: uint32_t,
    pub BreakFilter: uint32_t,
    pub Break2State: uint32_t,
    pub Break2Polarity: uint32_t,
    pub Break2Filter: uint32_t,
    pub AutomaticOutput: uint32_t,
}
/* *
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/* * @defgroup TIMEx_Exported_Functions TIMEx Exported Functions
  * @{
  */
/* * @defgroup TIMEx_Exported_Functions_Group1 Extended Timer Hall Sensor functions
 *  @brief    Timer Hall Sensor functions 
 *
@verbatim    
  ==============================================================================
                      ##### Timer Hall Sensor functions #####
  ==============================================================================
  [..]  
    This section provides functions allowing to:
    (+) Initialize and configure TIM HAL Sensor. 
    (+) De-initialize TIM HAL Sensor.
    (+) Start the Hall Sensor Interface.
    (+) Stop the Hall Sensor Interface.
    (+) Start the Hall Sensor Interface and enable interrupts.
    (+) Stop the Hall Sensor Interface and disable interrupts.
    (+) Start the Hall Sensor Interface and enable DMA transfers.
    (+) Stop the Hall Sensor Interface and disable DMA transfers.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM Hall Sensor Interface and create the associated handle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig: TIM Hall Sensor configuration structure
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_Init(mut htim:
                                                       *mut TIM_HandleTypeDef,
                                                   mut sConfig:
                                                       *mut TIM_HallSensor_InitTypeDef)
 -> HAL_StatusTypeDef {
    let mut OC_Config: TIM_OC_InitTypeDef =
        TIM_OC_InitTypeDef{OCMode: 0,
                           Pulse: 0,
                           OCPolarity: 0,
                           OCNPolarity: 0,
                           OCFastMode: 0,
                           OCIdleState: 0,
                           OCNIdleState: 0,};
    /* Check the TIM handle allocation */
    if htim.is_null() { return HAL_ERROR }
    /* Set the TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    HAL_TIMEx_HallSensor_MspInit(htim);
    /* Configure the Time base in the Encoder Mode */
    TIM_Base_SetConfig((*htim).Instance, &mut (*htim).Init);
    /* Configure the Channel 1 as Input Channel to interface with the three Outputs of the  Hall sensor */
    TIM_TI1_SetConfig((*htim).Instance, (*sConfig).IC1Polarity,
                      (0x3 as libc::c_uint) << 0 as libc::c_uint,
                      (*sConfig).IC1Filter);
    /* Reset the IC1PSC Bits */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3 as libc::c_uint) <<
                                           2 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the IC1PSC value */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*sConfig).IC1Prescaler) as uint32_t as
                                    uint32_t);
    /* Enable the Hall sensor interface (XOR function of the three inputs) */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select the TIM_TS_TI1F_ED signal as Input trigger for the TIM */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x7 as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x40 as libc::c_uint)
                                    as uint32_t as uint32_t);
    /* Use the TIM_TS_TI1F_ED signal to reset the TIM counter each edge detection */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x10007 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x4 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Program channel 2 in PWM 2 mode with the desired Commutation_Delay*/
    OC_Config.OCFastMode = 0 as libc::c_uint;
    OC_Config.OCIdleState = 0 as libc::c_uint;
    OC_Config.OCMode =
        (0x4 as libc::c_uint) << 4 as libc::c_uint |
            (0x2 as libc::c_uint) << 4 as libc::c_uint |
            (0x1 as libc::c_uint) << 4 as libc::c_uint;
    OC_Config.OCNIdleState = 0 as libc::c_uint;
    OC_Config.OCNPolarity = 0 as libc::c_uint;
    OC_Config.OCPolarity = 0 as libc::c_uint;
    OC_Config.Pulse = (*sConfig).Commutation_Delay;
    TIM_OC2_SetConfig((*htim).Instance, &mut OC_Config);
    /* Select OC2REF as trigger output on TRGO: write the MMS bits in the TIMx_CR2
    register to 101 */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x7 as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x4 as libc::c_uint) <<
                                          4 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              4 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Initialize the TIM state*/
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the TIM Hall Sensor interface  
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_DeInit(mut htim:
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
    HAL_TIMEx_HallSensor_MspDeInit(htim);
    /* Change TIM state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_RESET);
    /* Release Lock */
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM Hall Sensor MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_MspInit(mut htim:
                                                          *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIMEx_HallSensor_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes TIM Hall Sensor MSP.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_MspDeInit(mut htim:
                                                            *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIMEx_HallSensor_MspDeInit could be implemented in the user file
   */
}
/* *
  * @brief  Starts the TIM Hall Sensor Interface.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_Start(mut htim:
                                                        *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the Input Capture channels 1
    (in the Hall Sensor Interface the Three possible channels that can be used are TIM_CHANNEL_1, TIM_CHANNEL_2 and TIM_CHANNEL_3) */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                      0x1 as libc::c_uint);
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
  * @brief  Stops the TIM Hall sensor Interface.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_Stop(mut htim:
                                                       *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Input Capture channels 1, 2 and 3
    (in the Hall Sensor Interface the Three possible channels that can be used are TIM_CHANNEL_1, TIM_CHANNEL_2 and TIM_CHANNEL_3) */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint, 0 as libc::c_uint);
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
  * @brief  Starts the TIM Hall Sensor Interface in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_Start_IT(mut htim:
                                                           *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the capture compare Interrupts 1 event */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable the Input Capture channels 1
    (in the Hall Sensor Interface the Three possible channels that can be used are TIM_CHANNEL_1, TIM_CHANNEL_2 and TIM_CHANNEL_3) */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                      0x1 as libc::c_uint);
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
  * @brief  Stops the TIM Hall Sensor Interface in interrupt mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_Stop_IT(mut htim:
                                                          *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Input Capture channels 1
    (in the Hall Sensor Interface the Three possible channels that can be used are TIM_CHANNEL_1, TIM_CHANNEL_2 and TIM_CHANNEL_3) */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint, 0 as libc::c_uint);
    /* Disable the capture compare Interrupts event */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint)) as uint32_t as
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
  * @brief  Starts the TIM Hall Sensor Interface in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  pData: The destination Buffer address.
  * @param  Length: The length of data to be transferred from TIM peripheral to memory.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_Start_DMA(mut htim:
                                                            *mut TIM_HandleTypeDef,
                                                        mut pData:
                                                            *mut uint32_t,
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
    /* Enable the Input Capture channels 1
    (in the Hall Sensor Interface the Three possible channels that can be used are TIM_CHANNEL_1, TIM_CHANNEL_2 and TIM_CHANNEL_3) */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint,
                      0x1 as libc::c_uint);
    /* Set the DMA Input Capture 1 Callback */
    (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as usize]).XferCpltCallback
        =
        Some(TIM_DMACaptureCplt as
                 unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
    /* Set the DMA error callback */
    (*(*htim).hdma[0x1 as libc::c_uint as uint16_t as
                       usize]).XferErrorCallback =
        Some(TIM_DMAError as
                 unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
    /* Enable the DMA Stream for Capture 1*/
    HAL_DMA_Start_IT((*htim).hdma[0x1 as libc::c_uint as uint16_t as usize],
                     &mut (*(*htim).Instance).CCR1 as *mut uint32_t as
                         uint32_t, pData as uint32_t, Length as uint32_t);
    /* Enable the capture compare 1 Interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         9 as libc::c_uint) as uint32_t as
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
  * @brief  Stops the TIM Hall Sensor Interface in DMA mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_Stop_DMA(mut htim:
                                                           *mut TIM_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Input Capture channels 1
    (in the Hall Sensor Interface the Three possible channels that can be used are TIM_CHANNEL_1, TIM_CHANNEL_2 and TIM_CHANNEL_3) */
    TIM_CCxChannelCmd((*htim).Instance, 0 as libc::c_uint, 0 as libc::c_uint);
    /* Disable the capture compare Interrupts 1 event */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           9 as libc::c_uint)) as uint32_t as
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
  * @}
  */
/* * @defgroup TIMEx_Exported_Functions_Group2 Extended Timer Complementary Output Compare functions
 *  @brief    Timer Complementary Output Compare functions 
 *
@verbatim   
  ==============================================================================
              ##### Timer Complementary Output Compare functions #####
  ==============================================================================  
  [..]  
    This section provides functions allowing to:
    (+) Start the Complementary Output Compare/PWM.
    (+) Stop the Complementary Output Compare/PWM.
    (+) Start the Complementary Output Compare/PWM and enable interrupts.
    (+) Stop the Complementary Output Compare/PWM and disable interrupts.
    (+) Start the Complementary Output Compare/PWM and enable DMA transfers.
    (+) Stop the Complementary Output Compare/PWM and disable DMA transfers.
               
@endverbatim
  * @{
  */
/* *
  * @brief  Starts the TIM Output Compare signal generation on the complementary
  *         output.
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
pub unsafe extern "C" fn HAL_TIMEx_OCN_Start(mut htim: *mut TIM_HandleTypeDef,
                                             mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the Capture compare channel N */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0x4 as libc::c_uint);
    /* Enable the Main Output */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
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
  * @brief  Stops the TIM Output Compare signal generation on the complementary
  *         output.
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
pub unsafe extern "C" fn HAL_TIMEx_OCN_Stop(mut htim: *mut TIM_HandleTypeDef,
                                            mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the Capture compare channel N */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
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
  * @brief  Starts the TIM Output Compare signal generation in interrupt mode 
  *         on the complementary output.
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
pub unsafe extern "C" fn HAL_TIMEx_OCN_Start_IT(mut htim:
                                                    *mut TIM_HandleTypeDef,
                                                mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Enable the TIM Output Compare interrupt */
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
            /* Enable the TIM Output Compare interrupt */
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
            /* Enable the TIM Output Compare interrupt */
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
            /* Enable the TIM Output Compare interrupt */
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
    /* Enable the TIM Break interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable the Capture compare channel N */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0x4 as libc::c_uint);
    /* Enable the Main Output */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
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
  * @brief  Stops the TIM Output Compare signal generation in interrupt mode 
  *         on the complementary output.
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
pub unsafe extern "C" fn HAL_TIMEx_OCN_Stop_IT(mut htim:
                                                   *mut TIM_HandleTypeDef,
                                               mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    match Channel {
        0 => {
            /* Disable the TIM Output Compare interrupt */
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
            /* Disable the TIM Output Compare interrupt */
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
            /* Disable the TIM Output Compare interrupt */
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
            /* Disable the TIM Output Compare interrupt */
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
    /* Disable the Capture compare channel N */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    /* Disable the TIM Break interrupt (only if no more channel is active) */
    tmpccer = (*(*htim).Instance).CCER;
    if tmpccer &
           ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
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
  * @brief  Starts the TIM Output Compare signal generation in DMA mode 
  *         on the complementary output.
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
pub unsafe extern "C" fn HAL_TIMEx_OCN_Start_DMA(mut htim:
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
            /* Enable the TIM Output Compare DMA request */
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
            /* Enable the TIM Output Compare DMA request */
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
            /* Enable the TIM Output Compare DMA request */
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
            /* Enable the TIM Output Compare DMA request */
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
    /* Enable the Capture compare channel N */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0x4 as libc::c_uint);
    /* Enable the Main Output */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
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
  * @brief  Stops the TIM Output Compare signal generation in DMA mode 
  *         on the complementary output.
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
pub unsafe extern "C" fn HAL_TIMEx_OCN_Stop_DMA(mut htim:
                                                    *mut TIM_HandleTypeDef,
                                                mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    match Channel {
        0 => {
            /* Disable the TIM Output Compare DMA request */
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
            /* Disable the TIM Output Compare DMA request */
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
            /* Disable the TIM Output Compare DMA request */
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
            /* Disable the TIM Output Compare interrupt */
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
    /* Disable the Capture compare channel N */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
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
/* * @defgroup TIMEx_Exported_Functions_Group3 Extended Timer Complementary PWM functions
 *  @brief    Timer Complementary PWM functions 
 *
@verbatim   
  ==============================================================================
                 ##### Timer Complementary PWM functions #####
  ==============================================================================  
  [..]  
    This section provides functions allowing to:
    (+) Start the Complementary PWM.
    (+) Stop the Complementary PWM.
    (+) Start the Complementary PWM and enable interrupts.
    (+) Stop the Complementary PWM and disable interrupts.
    (+) Start the Complementary PWM and enable DMA transfers.
    (+) Stop the Complementary PWM and disable DMA transfers.
    (+) Start the Complementary Input Capture measurement.
    (+) Stop the Complementary Input Capture.
    (+) Start the Complementary Input Capture and enable interrupts.
    (+) Stop the Complementary Input Capture and disable interrupts.
    (+) Start the Complementary Input Capture and enable DMA transfers.
    (+) Stop the Complementary Input Capture and disable DMA transfers.
    (+) Start the Complementary One Pulse generation.
    (+) Stop the Complementary One Pulse.
    (+) Start the Complementary One Pulse and enable interrupts.
    (+) Stop the Complementary One Pulse and disable interrupts.
               
@endverbatim
  * @{
  */
/* *
  * @brief  Starts the PWM signal generation on the complementary output.
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
pub unsafe extern "C" fn HAL_TIMEx_PWMN_Start(mut htim:
                                                  *mut TIM_HandleTypeDef,
                                              mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the complementary PWM output  */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0x4 as libc::c_uint);
    /* Enable the Main Output */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
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
  * @brief  Stops the PWM signal generation on the complementary output.
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
pub unsafe extern "C" fn HAL_TIMEx_PWMN_Stop(mut htim: *mut TIM_HandleTypeDef,
                                             mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the complementary PWM output  */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
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
  * @brief  Starts the PWM signal generation in interrupt mode on the 
  *         complementary output.
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
pub unsafe extern "C" fn HAL_TIMEx_PWMN_Start_IT(mut htim:
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
    /* Enable the TIM Break interrupt */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable the complementary PWM output  */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0x4 as libc::c_uint);
    /* Enable the Main Output */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
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
  * @brief  Stops the PWM signal generation in interrupt mode on the 
  *         complementary output.
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
pub unsafe extern "C" fn HAL_TIMEx_PWMN_Stop_IT(mut htim:
                                                    *mut TIM_HandleTypeDef,
                                                mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
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
            /* Disable the TIM Capture/Compare 3 interrupt */
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
    /* Disable the complementary PWM output  */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
    /* Disable the TIM Break interrupt (only if no more channel is active) */
    tmpccer = (*(*htim).Instance).CCER;
    if tmpccer &
           ((0x1 as libc::c_uint) << 2 as libc::c_uint |
                (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
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
  * @brief  Starts the TIM PWM signal generation in DMA mode on the 
  *         complementary output
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
pub unsafe extern "C" fn HAL_TIMEx_PWMN_Start_DMA(mut htim:
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
    /* Enable the complementary PWM output  */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0x4 as libc::c_uint);
    /* Enable the Main Output */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
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
  * @brief  Stops the TIM PWM signal generation in DMA mode on the complementary
  *         output
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
pub unsafe extern "C" fn HAL_TIMEx_PWMN_Stop_DMA(mut htim:
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
            /* Disable the TIM Capture/Compare 4 DMA request */
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
    /* Disable the complementary PWM output */
    TIM_CCxNChannelCmd((*htim).Instance, Channel, 0 as libc::c_uint);
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
/* * @defgroup TIMEx_Exported_Functions_Group4 Extended Timer Complementary One Pulse functions
 *  @brief    Timer Complementary One Pulse functions 
 *
@verbatim   
  ==============================================================================
                ##### Timer Complementary One Pulse functions #####
  ==============================================================================  
  [..]  
    This section provides functions allowing to:
    (+) Start the Complementary One Pulse generation.
    (+) Stop the Complementary One Pulse.
    (+) Start the Complementary One Pulse and enable interrupts.
    (+) Stop the Complementary One Pulse and disable interrupts.
               
@endverbatim
  * @{
  */
/* *
  * @brief  Starts the TIM One Pulse signal generation on the complemetary 
  *         output.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel: TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_OnePulseN_Start(mut htim:
                                                       *mut TIM_HandleTypeDef,
                                                   mut OutputChannel:
                                                       uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Enable the complementary One Pulse output */
    TIM_CCxNChannelCmd((*htim).Instance, OutputChannel, 0x4 as libc::c_uint);
    /* Enable the Main Output */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM One Pulse signal generation on the complementary 
  *         output.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel: TIM Channel to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_OnePulseN_Stop(mut htim:
                                                      *mut TIM_HandleTypeDef,
                                                  mut OutputChannel: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Disable the complementary One Pulse output */
    TIM_CCxNChannelCmd((*htim).Instance, OutputChannel, 0 as libc::c_uint);
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
  * @brief  Starts the TIM One Pulse signal generation in interrupt mode on the
  *         complementary channel.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel: TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_OnePulseN_Start_IT(mut htim:
                                                          *mut TIM_HandleTypeDef,
                                                      mut OutputChannel:
                                                          uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
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
    /* Enable the complementary One Pulse output */
    TIM_CCxNChannelCmd((*htim).Instance, OutputChannel, 0x4 as libc::c_uint);
    /* Enable the Main Output */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Stops the TIM One Pulse signal generation in interrupt mode on the
  *         complementary channel.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel: TIM Channel to be disabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_OnePulseN_Stop_IT(mut htim:
                                                         *mut TIM_HandleTypeDef,
                                                     mut OutputChannel:
                                                         uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
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
    /* Disable the complementary One Pulse output */
    TIM_CCxNChannelCmd((*htim).Instance, OutputChannel, 0 as libc::c_uint);
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
/* * @defgroup TIMEx_Exported_Functions_Group5 Extended Peripheral Control functions
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
    (+) Configure the commutation event in case of use of the Hall sensor interface.
    (+) Configure the DMA Burst Mode.
      
@endverbatim
  * @{
  */
/* *
  * @brief  Configure the TIM commutation event sequence.
  * @note  This function is mandatory to use the commutation event in order to 
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer) 
  *        configured in Hall sensor interface, this interface Timer will generate the 
  *        commutation at its TRGO output (connected to Timer used in this function) each time 
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  InputTrigger: the Internal trigger corresponding to the Timer Interfacing with the Hall sensor.
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal trigger 0 selected
  *            @arg TIM_TS_ITR1: Internal trigger 1 selected
  *            @arg TIM_TS_ITR2: Internal trigger 2 selected
  *            @arg TIM_TS_ITR3: Internal trigger 3 selected
  *            @arg TIM_TS_NONE: No trigger is needed 
  * @param  CommutationSource: the Commutation Event source.
  *          This parameter can be one of the following values:
  *            @arg TIM_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TIM_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_ConfigCommutationEvent(mut htim:
                                                              *mut TIM_HandleTypeDef,
                                                          mut InputTrigger:
                                                              uint32_t,
                                                          mut CommutationSource:
                                                              uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    if InputTrigger == 0 as libc::c_uint ||
           InputTrigger == 0x10 as libc::c_uint ||
           InputTrigger == 0x20 as libc::c_uint ||
           InputTrigger == 0x30 as libc::c_uint {
        /* Select the Input trigger */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x7 as libc::c_uint) <<
                                               4 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | InputTrigger) as
                                        uint32_t as uint32_t)
    }
    /* Select the Capture Compare preload feature */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select the Commutation event source */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           2 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | CommutationSource) as
                                    uint32_t as uint32_t);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Configure the TIM commutation event sequence with interrupt.
  * @note  This function is mandatory to use the commutation event in order to 
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer) 
  *        configured in Hall sensor interface, this interface Timer will generate the 
  *        commutation at its TRGO output (connected to Timer used in this function) each time 
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  InputTrigger: the Internal trigger corresponding to the Timer Interfacing with the Hall sensor.
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal trigger 0 selected
  *            @arg TIM_TS_ITR1: Internal trigger 1 selected
  *            @arg TIM_TS_ITR2: Internal trigger 2 selected
  *            @arg TIM_TS_ITR3: Internal trigger 3 selected
  *            @arg TIM_TS_NONE: No trigger is needed
  * @param  CommutationSource: the Commutation Event source.
  *          This parameter can be one of the following values:
  *            @arg TIM_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TIM_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_ConfigCommutationEvent_IT(mut htim:
                                                                 *mut TIM_HandleTypeDef,
                                                             mut InputTrigger:
                                                                 uint32_t,
                                                             mut CommutationSource:
                                                                 uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    if InputTrigger == 0 as libc::c_uint ||
           InputTrigger == 0x10 as libc::c_uint ||
           InputTrigger == 0x20 as libc::c_uint ||
           InputTrigger == 0x30 as libc::c_uint {
        /* Select the Input trigger */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x7 as libc::c_uint) <<
                                               4 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | InputTrigger) as
                                        uint32_t as uint32_t)
    }
    /* Select the Capture Compare preload feature */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select the Commutation event source */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           2 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | CommutationSource) as
                                    uint32_t as uint32_t);
    /* Enable the Commutation Interrupt Request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         5 as libc::c_uint) as uint32_t as
                                    uint32_t);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Configure the TIM commutation event sequence with DMA.
  * @note  This function is mandatory to use the commutation event in order to 
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer) 
  *        configured in Hall sensor interface, this interface Timer will generate the 
  *        commutation at its TRGO output (connected to Timer used in this function) each time 
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @note: The user should configure the DMA in his own software, in This function only the COMDE bit is set
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  InputTrigger: the Internal trigger corresponding to the Timer Interfacing with the Hall sensor.
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal trigger 0 selected
  *            @arg TIM_TS_ITR1: Internal trigger 1 selected
  *            @arg TIM_TS_ITR2: Internal trigger 2 selected
  *            @arg TIM_TS_ITR3: Internal trigger 3 selected
  *            @arg TIM_TS_NONE: No trigger is needed
  * @param  CommutationSource: the Commutation Event source.
  *          This parameter can be one of the following values:
  *            @arg TIM_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TIM_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_ConfigCommutationEvent_DMA(mut htim:
                                                                  *mut TIM_HandleTypeDef,
                                                              mut InputTrigger:
                                                                  uint32_t,
                                                              mut CommutationSource:
                                                                  uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    if InputTrigger == 0 as libc::c_uint ||
           InputTrigger == 0x10 as libc::c_uint ||
           InputTrigger == 0x20 as libc::c_uint ||
           InputTrigger == 0x30 as libc::c_uint {
        /* Select the Input trigger */
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x7 as libc::c_uint) <<
                                               4 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).SMCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | InputTrigger) as
                                        uint32_t as uint32_t)
    }
    /* Select the Capture Compare preload feature */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select the Commutation event source */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           2 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | CommutationSource) as
                                    uint32_t as uint32_t);
    /* Enable the Commutation DMA Request */
  /* Set the DMA Commutation Callback */
    (*(*htim).hdma[0x5 as libc::c_uint as uint16_t as usize]).XferCpltCallback
        =
        Some(TIMEx_DMACommutationCplt as
                 unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
    /* Set the DMA error callback */
    (*(*htim).hdma[0x5 as libc::c_uint as uint16_t as
                       usize]).XferErrorCallback =
        Some(TIM_DMAError as
                 unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
    /* Enable the Commutation DMA Request */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).DIER as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         13 as libc::c_uint) as uint32_t as
                                    uint32_t);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Configures the TIM in master mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.   
  * @param  sMasterConfig: pointer to a TIM_MasterConfigTypeDef structure that
  *         contains the selected trigger output (TRGO) and the Master/Slave 
  *         mode. 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_MasterConfigSynchronization(mut htim:
                                                                   *mut TIM_HandleTypeDef,
                                                               mut sMasterConfig:
                                                                   *mut TIM_MasterConfigTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmpcr2: uint32_t = 0;
    let mut tmpsmcr: uint32_t = 0;
    /* Check the parameters */
    /* Check input state */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*(*htim).Instance).CR2;
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*(*htim).Instance).SMCR;
    /* If the timer supports ADC synchronization through TRGO2, set the master mode selection 2 */
    if (*htim).Instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           (*htim).Instance ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Check the parameters */
        /* Clear the MMS2 bits */
        tmpcr2 &= !((0xf as libc::c_uint) << 20 as libc::c_uint);
        tmpcr2 |= (*sMasterConfig).MasterOutputTrigger2
    }
    /* Select the TRGO2 source*/
    /* Reset the MMS Bits */
    tmpcr2 &= !((0x7 as libc::c_uint) << 4 as libc::c_uint);
    /* Select the TRGO source */
    tmpcr2 |= (*sMasterConfig).MasterOutputTrigger;
    /* Reset the MSM Bit */
    tmpsmcr &= !((0x1 as libc::c_uint) << 7 as libc::c_uint);
    /* Set master mode */
    tmpsmcr |= (*sMasterConfig).MasterSlaveMode;
    /* Update TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CR2 as *mut uint32_t,
                                tmpcr2);
    /* Update TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                    *mut uint32_t, tmpsmcr);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief   Configures the Break feature, dead time, Lock level, OSSI/OSSR State
  *         and the AOE(automatic output enable).
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sBreakDeadTimeConfig: pointer to a TIM_ConfigBreakDeadConfig_TypeDef structure that
  *         contains the BDTR Register configuration  information for the TIM peripheral. 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_ConfigBreakDeadTime(mut htim:
                                                           *mut TIM_HandleTypeDef,
                                                       mut sBreakDeadTimeConfig:
                                                           *mut TIM_BreakDeadTimeConfigTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmpbdtr: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Check input state */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Set the Lock level, the Break enable Bit and the Polarity, the OSSR State,
     the OSSI State, the dead time value and the Automatic Output Enable Bit */
  /* Set the BDTR bits */
    tmpbdtr =
        tmpbdtr & !((0xff as libc::c_uint) << 0 as libc::c_uint) |
            (*sBreakDeadTimeConfig).DeadTime;
    tmpbdtr =
        tmpbdtr & !((0x3 as libc::c_uint) << 8 as libc::c_uint) |
            (*sBreakDeadTimeConfig).LockLevel;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 10 as libc::c_uint) |
            (*sBreakDeadTimeConfig).OffStateIDLEMode;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 11 as libc::c_uint) |
            (*sBreakDeadTimeConfig).OffStateRunMode;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 12 as libc::c_uint) |
            (*sBreakDeadTimeConfig).BreakState;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 13 as libc::c_uint) |
            (*sBreakDeadTimeConfig).BreakPolarity;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 14 as libc::c_uint) |
            (*sBreakDeadTimeConfig).AutomaticOutput;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 15 as libc::c_uint) |
            (*sBreakDeadTimeConfig).AutomaticOutput;
    tmpbdtr =
        tmpbdtr & !((0xf as libc::c_uint) << 16 as libc::c_uint) |
            (*sBreakDeadTimeConfig).BreakFilter << 16 as libc::c_int;
    if (*htim).Instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           (*htim).Instance ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the BREAK2 input related BDTR bits */
        tmpbdtr =
            tmpbdtr & !((0xf as libc::c_uint) << 20 as libc::c_uint) |
                (*sBreakDeadTimeConfig).Break2Filter << 20 as libc::c_int;
        tmpbdtr =
            tmpbdtr & !((0x1 as libc::c_uint) << 24 as libc::c_uint) |
                (*sBreakDeadTimeConfig).Break2State;
        tmpbdtr =
            tmpbdtr & !((0x1 as libc::c_uint) << 25 as libc::c_uint) |
                (*sBreakDeadTimeConfig).Break2Polarity
    }
    /* Set TIMx_BDTR */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).BDTR as
                                    *mut uint32_t, tmpbdtr);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @brief  Configures the TIM2, TIM5 and TIM11 Remapping input capabilities.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Remap: specifies the TIM input remapping source.
  *          This parameter can be one of the following values:
  *            @arg TIM_TIM2_TIM8_TRGO: TIM2 ITR1 input is connected to TIM8 Trigger output(default)
  *            @arg TIM_TIM2_ETH_PTP:   TIM2 ITR1 input is connected to ETH PTP trigger output.
  *            @arg TIM_TIM2_USBFS_SOF: TIM2 ITR1 input is connected to USB FS SOF. 
  *            @arg TIM_TIM2_USBHS_SOF: TIM2 ITR1 input is connected to USB HS SOF. 
  *            @arg TIM_TIM5_GPIO:      TIM5 CH4 input is connected to dedicated Timer pin(default)
  *            @arg TIM_TIM5_LSI:       TIM5 CH4 input is connected to LSI clock.
  *            @arg TIM_TIM5_LSE:       TIM5 CH4 input is connected to LSE clock.
  *            @arg TIM_TIM5_RTC:       TIM5 CH4 input is connected to RTC Output event.
  *            @arg TIM_TIM11_GPIO:     TIM11 CH4 input is connected to dedicated Timer pin(default) 
  *            @arg TIM_TIM11_SPDIF:    SPDIF Frame synchronous   
  *            @arg TIM_TIM11_HSE:      TIM11 CH4 input is connected to HSE_RTC clock
  *                                     (HSE divided by a programmable prescaler) 
  *            @arg TIM_TIM11_MCO1:     TIM11 CH1 input is connected to MCO1    
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_RemapConfig(mut htim:
                                                   *mut TIM_HandleTypeDef,
                                               mut Remap: uint32_t)
 -> HAL_StatusTypeDef {
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    /* Check parameters */
    /* Set the Timer remapping configuration */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).OR as *mut uint32_t,
                                Remap);
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Group channel 5 and channel 1, 2 or 3
  * @param  htim: TIM handle.
  * @param  OCRef: specifies the reference signal(s) the OC5REF is combined with.
  *         This parameter can be any combination of the following values:
  *         TIM_GROUPCH5_NONE: No effect of OC5REF on OC1REFC, OC2REFC and OC3REFC
  *         TIM_GROUPCH5_OC1REFC: OC1REFC is the logical AND of OC1REFC and OC5REF
  *         TIM_GROUPCH5_OC2REFC: OC2REFC is the logical AND of OC2REFC and OC5REF
  *         TIM_GROUPCH5_OC3REFC: OC3REFC is the logical AND of OC3REFC and OC5REF
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_GroupChannel5(mut htim:
                                                     *mut TIM_HandleTypeDef,
                                                 mut OCRef: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check parameters */
    /* Process Locked */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_BUSY);
    /* Clear GC5Cx bit fields */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCR5 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCR5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           31 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               30 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               29 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    /* Set GC5Cx bit fields */
    ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCR5 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCR5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | OCRef) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup TIMEx_Exported_Functions_Group6 Extended Callbacks functions 
  * @brief    Extended Callbacks functions
 *
@verbatim   
  ==============================================================================
                    ##### Extension Callbacks functions #####
  ==============================================================================  
  [..]  
    This section provides Extension TIM callback functions:
    (+) Timer Commutation callback
    (+) Timer Break callback

@endverbatim
  * @{
  */
/* *
  * @brief  Hall commutation changed callback in non blocking mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_CommutationCallback(mut htim:
                                                           *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIMEx_CommutationCallback could be implemented in the user file
   */
}
/* *
  * @brief  Hall Break detection callback in non blocking mode 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_BreakCallback(mut htim:
                                                     *mut TIM_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIMEx_BreakCallback could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @addtogroup TIMEx_Exported_Functions_Group7
  * @{
  */
/* Extension Peripheral State functions  **************************************/
/* *
  * @}
  */
/* * @defgroup TIMEx_Exported_Functions_Group7 Extended Peripheral State functions 
 *  @brief    Extended Peripheral State functions
 *
@verbatim   
  ==============================================================================
                ##### Extension Peripheral State functions #####
  ==============================================================================  
  [..]
    This subsection permits to get in run-time the status of the peripheral 
    and the data flow.

@endverbatim
  * @{
  */
/* *
  * @brief  Return the TIM Hall Sensor interface state
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIMEx_HallSensor_GetState(mut htim:
                                                           *mut TIM_HandleTypeDef)
 -> HAL_TIM_StateTypeDef {
    return (*htim).State;
}
/* *
  * @}
  */
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup TIMEx_Exported_Macros TIMEx Exported Macros
  * @{
  */
/* *
  * @brief  Sets the TIM Capture Compare Register value on runtime without
  *         calling another time ConfigChannel function.
  * @param  __HANDLE__: TIM handle.
  * @param  __CHANNEL__ : TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected
  * @param  __COMPARE__: specifies the Capture Compare register new value.
  * @retval None
  */
/* *
  * @brief  Gets the TIM Capture Compare Register value on runtime
  * @param  __HANDLE__: TIM handle.
  * @param  __CHANNEL__ : TIM Channel associated with the capture compare register
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: get capture/compare 1 register value
  *            @arg TIM_CHANNEL_2: get capture/compare 2 register value
  *            @arg TIM_CHANNEL_3: get capture/compare 3 register value
  *            @arg TIM_CHANNEL_4: get capture/compare 4 register value
  *            @arg TIM_CHANNEL_5: get capture/compare 5 register value
  *            @arg TIM_CHANNEL_6: get capture/compare 6 register value
  * @retval None
  */
/* *
  * @brief  Sets the TIM Output compare preload.
  * @param  __HANDLE__: TIM handle.
  * @param  __CHANNEL__: TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected
  * @retval None
  */
/* *
  * @brief  Resets the TIM Output compare preload.
  * @param  __HANDLE__: TIM handle.
  * @param  __CHANNEL__: TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected
  * @retval None
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup TIMEx_Exported_Functions
  * @{
  */
/* * @addtogroup TIMEx_Exported_Functions_Group1
  * @{
  */
/*  Timer Hall Sensor functions  **********************************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup TIMEx_Exported_Functions_Group2
  * @{
  */
/*  Timer Complementary Output Compare functions  *****************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup TIMEx_Exported_Functions_Group3
  * @{
  */
/*  Timer Complementary PWM functions  ****************************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup TIMEx_Exported_Functions_Group4
  * @{
  */
/*  Timer Complementary One Pulse functions  **********************************/
/* Blocking mode: Polling */
/* Non-Blocking mode: Interrupt */
/* *
  * @}
  */
/* * @addtogroup TIMEx_Exported_Functions_Group5
  * @{
  */
/* Extension Control functions  ************************************************/
/* STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/* * @addtogroup TIMEx_Exported_Functions_Group6
  * @{
  */ 
/* Extension Callback *********************************************************/
/* *
  * @}
  */
/* *
  * @brief  TIM DMA Commutation callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIMEx_DMACommutationCplt(mut hdma:
                                                      *mut DMA_HandleTypeDef) {
    let mut htim: *mut TIM_HandleTypeDef =
        (*hdma).Parent as *mut TIM_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    HAL_TIMEx_CommutationCallback(htim);
}
/* *
  * @}
  */
/* *
  * @brief  Configures the OCRef clear feature
  * @param  htim: TIM handle
  * @param  sClearInputConfig: pointer to a TIM_ClearInputConfigTypeDef structure that
  *         contains the OCREF clear feature and parameters for the TIM peripheral. 
  * @param  Channel: specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_Channel_1: TIM Channel 1
  *            @arg TIM_Channel_2: TIM Channel 2
  *            @arg TIM_Channel_3: TIM Channel 3
  *            @arg TIM_Channel_4: TIM Channel 4
  *            @arg TIM_Channel_5: TIM Channel 5
  *            @arg TIM_Channel_6: TIM Channel 6
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_ConfigOCrefClear(mut htim:
                                                      *mut TIM_HandleTypeDef,
                                                  mut sClearInputConfig:
                                                      *mut TIM_ClearInputConfigTypeDef,
                                                  mut Channel: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmpsmcr: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Check input state */
    if (*htim).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*htim).Lock = HAL_LOCKED }
    match (*sClearInputConfig).ClearInputSource {
        0 => {
            /* Get the TIMx SMCR register value */
            tmpsmcr = (*(*htim).Instance).SMCR;
            /* Clear the ETR Bits */
            tmpsmcr &=
                !((0xf as libc::c_uint) << 8 as libc::c_uint |
                      (0x3 as libc::c_uint) << 12 as libc::c_uint |
                      (0x1 as libc::c_uint) << 14 as libc::c_uint |
                      (0x1 as libc::c_uint) << 15 as libc::c_uint);
            /* Set TIMx_SMCR */
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).SMCR as
                                            *mut uint32_t, tmpsmcr)
        }
        1 => {
            /* Check the parameters */
            TIM_ETR_SetConfig((*htim).Instance,
                              (*sClearInputConfig).ClearInputPrescaler,
                              (*sClearInputConfig).ClearInputPolarity,
                              (*sClearInputConfig).ClearInputFilter);
        }
        _ => { }
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
        16 => {
            if (*sClearInputConfig).ClearInputState !=
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the Ocref clear feature for Channel 1 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     7 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                /* Disable the Ocref clear feature for Channel 1 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       7 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            }
        }
        20 => {
            if (*sClearInputConfig).ClearInputState !=
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the Ocref clear feature for Channel 1 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     15 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                /* Disable the Ocref clear feature for Channel 1 */
                ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
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
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initializes the TIM Output Compare Channels according to the specified
  *         parameters in the TIM_OC_InitTypeDef.
  * @param  htim: TIM Output Compare handle
  * @param  sConfig: TIM Output Compare configuration structure
  * @param  Channel : TIM Channels to configure
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected 
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected 
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected 
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
            /* Check the parameters */
            /* Configure the TIM Channel 1 in Output Compare */
            TIM_OC1_SetConfig((*htim).Instance, sConfig);
        }
        4 => {
            /* Check the parameters */
            /* Configure the TIM Channel 2 in Output Compare */
            TIM_OC2_SetConfig((*htim).Instance, sConfig);
        }
        8 => {
            /* Check the parameters */
            /* Configure the TIM Channel 3 in Output Compare */
            TIM_OC3_SetConfig((*htim).Instance, sConfig);
        }
        12 => {
            /* Check the parameters */
            /* Configure the TIM Channel 4 in Output Compare */
            TIM_OC4_SetConfig((*htim).Instance, sConfig);
        }
        16 => {
            /* Check the parameters */
            /* Configure the TIM Channel 5 in Output Compare */
            TIM_OC5_SetConfig((*htim).Instance, sConfig);
        }
        20 => {
            /* Check the parameters */
            /* Configure the TIM Channel 6 in Output Compare */
            TIM_OC6_SetConfig((*htim).Instance, sConfig);
        }
        _ => { }
    }
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    (*htim).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
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
  * @brief  Initializes the TIM PWM  channels according to the specified
  *         parameters in the TIM_OC_InitTypeDef.
  * @param  htim: TIM PWM handle
  * @param  sConfig: TIM PWM configuration structure
  * @param  Channel : TIM Channels to be configured
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected 
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_TIM_PWM_ConfigChannel(mut htim:
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
            /* Check the parameters */
            /* Configure the Channel 1 in PWM mode */
            TIM_OC1_SetConfig((*htim).Instance, sConfig);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t);
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
            /* Set the Preload enable bit for channel1 */
            /* Configure the Output Fast mode */
            /* Check the parameters */
            /* Configure the Channel 2 in PWM mode */
            TIM_OC2_SetConfig((*htim).Instance, sConfig);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint) as
                                            uint32_t as uint32_t);
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
            /* Set the Preload enable bit for channel2 */
            /* Configure the Output Fast mode */
            /* Check the parameters */
            /* Configure the Channel 3 in PWM mode */
            TIM_OC3_SetConfig((*htim).Instance, sConfig);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t);
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
            /* Set the Preload enable bit for channel3 */
            /* Configure the Output Fast mode */
            /* Check the parameters */
            /* Configure the Channel 4 in PWM mode */
            TIM_OC4_SetConfig((*htim).Instance, sConfig);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint) as
                                            uint32_t as uint32_t);
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
        16 => {
            /* Set the Preload enable bit for channel4 */
            /* Configure the Output Fast mode */
            /* Check the parameters */
            /* Configure the Channel 5 in PWM mode */
            TIM_OC5_SetConfig((*htim).Instance, sConfig);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfig).OCFastMode) as
                                            uint32_t as uint32_t)
        }
        20 => {
            /* Set the Preload enable bit for channel5*/
            /* Configure the Output Fast mode */
            /* Check the parameters */
            /* Configure the Channel 5 in PWM mode */
            TIM_OC6_SetConfig((*htim).Instance, sConfig);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(*htim).Instance).CCMR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*htim).Instance).CCMR3
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
/* Set the Preload enable bit for channel6 */
/* Configure the Output Fast mode */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_tim_ex.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   TIM HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Timer extension peripheral:
  *           + Time Hall Sensor Interface Initialization
  *           + Time Hall Sensor Interface Start
  *           + Time Complementary signal bread and dead time configuration  
  *           + Time Master and Slave synchronization configuration
  *           + Time Output Compare/PWM Channel Configuration (for channels 5 and 6)
  *           + Time OCRef clear configuration
  *           + Timer remapping capabilities configuration  
  @verbatim 
  ==============================================================================
                      ##### TIMER Extended features #####
  ==============================================================================
  [..] 
    The Timer Extension features include: 
    (#) Complementary outputs with programmable dead-time for :
        (++) Input Capture
        (++) Output Compare
        (++) PWM generation (Edge and Center-aligned Mode)
        (++) One-pulse mode output
    (#) Synchronization circuit to control the timer with external signals and to 
        interconnect several timers together.
    (#) Break input to put the timer output signals in reset state or in a known state.
    (#) Supports incremental (quadrature) encoder and hall-sensor circuitry for 
        positioning purposes                
   
                        ##### How to use this driver #####
  ==============================================================================
  [..]
     (#) Initialize the TIM low level resources by implementing the following functions 
         depending from feature used :
           (++) Complementary Output Compare : HAL_TIM_OC_MspInit()
           (++) Complementary PWM generation : HAL_TIM_PWM_MspInit()
           (++) Complementary One-pulse mode output : HAL_TIM_OnePulse_MspInit()
           (++) Hall Sensor output : HAL_TIM_HallSensor_MspInit()
           
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
        (++) HAL_TIMEx_HallSensor_Init and HAL_TIMEx_ConfigCommutationEvent: to use the 
             Timer Hall Sensor Interface and the commutation event with the corresponding 
             Interrupt and DMA request if needed (Note that One Timer is used to interface 
             with the Hall sensor Interface and another Timer should be used to use 
             the commutation event).

    (#) Activate the TIM peripheral using one of the start functions: 
           (++) Complementary Output Compare : HAL_TIMEx_OCN_Start(), HAL_TIMEx_OCN_Start_DMA(), HAL_TIMEx_OC_Start_IT()
           (++) Complementary PWM generation : HAL_TIMEx_PWMN_Start(), HAL_TIMEx_PWMN_Start_DMA(), HAL_TIMEx_PWMN_Start_IT()
           (++) Complementary One-pulse mode output : HAL_TIMEx_OnePulseN_Start(), HAL_TIMEx_OnePulseN_Start_IT()
           (++) Hall Sensor output : HAL_TIMEx_HallSensor_Start(), HAL_TIMEx_HallSensor_Start_DMA(), HAL_TIMEx_HallSensor_Start_IT().

  
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
/* * @defgroup TIMEx TIMEx
  * @brief TIM Extended HAL module driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* * @addtogroup TIMEx_Private_Functions
  * @{
  */
/* *
  * @brief  Enables or disables the TIM Capture Compare Channel xN.
  * @param  TIMx to select the TIM peripheral
  * @param  Channel: specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_Channel_1: TIM Channel 1
  *            @arg TIM_Channel_2: TIM Channel 2
  *            @arg TIM_Channel_3: TIM Channel 3
  * @param  ChannelNState: specifies the TIM Channel CCxNE bit new state.
  *          This parameter can be: TIM_CCxN_ENABLE or TIM_CCxN_Disable. 
  * @retval None
  */
unsafe extern "C" fn TIM_CCxNChannelCmd(mut TIMx: *mut TIM_TypeDef,
                                        mut Channel: uint32_t,
                                        mut ChannelNState: uint32_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmp = ((0x1 as libc::c_uint) << 2 as libc::c_uint) << Channel;
    /* Reset the CCxNE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !tmp) as uint32_t as
                                    uint32_t);
    /* Set or reset the CCxNE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ChannelNState << Channel) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Timer Output Compare 5 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config: The output configuration structure
  * @retval None
  */
unsafe extern "C" fn TIM_OC5_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                       mut OC_Config:
                                           *mut TIM_OC_InitTypeDef) {
    let mut tmpccmrx: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpcr2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the output: Reset the CCxE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmrx = (*TIMx).CCMR3;
    /* Reset the Output Compare Mode Bits */
    tmpccmrx &= !((0x1007 as libc::c_uint) << 4 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*OC_Config).OCMode;
    /* Reset the Output Polarity level */
    tmpccer &= !((0x1 as libc::c_uint) << 17 as libc::c_uint);
    /* Set the Output Compare Polarity */
    tmpccer |= (*OC_Config).OCPolarity << 16 as libc::c_int;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Reset the Output Compare IDLE State */
        tmpcr2 &= !((0x1 as libc::c_uint) << 16 as libc::c_uint);
        /* Set the Output Idle state */
        tmpcr2 |= (*OC_Config).OCIdleState << 8 as libc::c_int
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR3 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                (*OC_Config).Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Timer Output Compare 6 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config: The output configuration structure
  * @retval None
  */
unsafe extern "C" fn TIM_OC6_SetConfig(mut TIMx: *mut TIM_TypeDef,
                                       mut OC_Config:
                                           *mut TIM_OC_InitTypeDef) {
    let mut tmpccmrx: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpccer: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpcr2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Disable the output: Reset the CCxE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           20 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmrx = (*TIMx).CCMR3;
    /* Reset the Output Compare Mode Bits */
    tmpccmrx &= !((0x1007 as libc::c_uint) << 12 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*OC_Config).OCMode << 8 as libc::c_int;
    /* Reset the Output Polarity level */
    tmpccer &= !((0x1 as libc::c_uint) << 21 as libc::c_uint);
    /* Set the Output Compare Polarity */
    tmpccer |= (*OC_Config).OCPolarity << 20 as libc::c_int;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Reset the Output Compare IDLE State */
        tmpcr2 &= !((0x1 as libc::c_uint) << 18 as libc::c_uint);
        /* Set the Output Idle state */
        tmpcr2 |= (*OC_Config).OCIdleState << 10 as libc::c_int
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR3 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR6 as *mut uint32_t,
                                (*OC_Config).Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @}
  */ 
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* HAL_TIM_MODULE_ENABLED */
