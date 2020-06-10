use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup DMA_Exported_Constants DMA Exported Constants
  * @brief    DMA Exported constants 
  * @{
  */
    /* * @defgroup DMA_Error_Code DMA Error Code
  * @brief    DMA Error Code 
  * @{
  */
    /* !< No error                               */
    /* !< Transfer error                         */
    /* !< FIFO error                             */
    /* !< Direct Mode error                      */
    /* !< Timeout error                          */
    /* !< Parameter error                        */
    /* !< Abort requested with no Xfer ongoing   */
    /* !< Not supported mode                     */
    /* *
  * @}
  */
    /* * @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @brief    DMA data transfer direction 
  * @{
  */
    /* !< Peripheral to memory direction */
    /* !< Memory to peripheral direction */
    /* !< Memory to memory direction     */
    /* *
  * @}
  */
    /* * @defgroup DMA_Peripheral_incremented_mode DMA Peripheral incremented mode
  * @brief    DMA peripheral incremented mode 
  * @{
  */
    /* !< Peripheral increment mode enable  */
    /* !< Peripheral increment mode disable */
    /* *
  * @}
  */
    /* * @defgroup DMA_Memory_incremented_mode DMA Memory incremented mode
  * @brief    DMA memory incremented mode 
  * @{
  */
    /* !< Memory increment mode enable  */
    /* !< Memory increment mode disable */
    /* *
  * @}
  */
    /* * @defgroup DMA_Peripheral_data_size DMA Peripheral data size
  * @brief    DMA peripheral data size 
  * @{
  */
    /* !< Peripheral data alignment: Byte     */
    /* !< Peripheral data alignment: HalfWord */
    /* !< Peripheral data alignment: Word     */
    /* *
  * @}
  */
    /* * @defgroup DMA_Memory_data_size DMA Memory data size
  * @brief    DMA memory data size 
  * @{ 
  */
    /* !< Memory data alignment: Byte     */
    /* !< Memory data alignment: HalfWord */
    /* !< Memory data alignment: Word     */
    /* *
  * @}
  */
    /* * @defgroup DMA_mode DMA mode
  * @brief    DMA mode 
  * @{
  */
    /* !< Normal mode                  */
    /* !< Circular mode                */
    /* !< Peripheral flow control mode */
    /* *
  * @}
  */
    /* * @defgroup DMA_Priority_level DMA Priority level
  * @brief    DMA priority levels 
  * @{
  */
    /* !< Priority level: Low       */
    /* !< Priority level: Medium    */
    /* !< Priority level: High      */
    /* !< Priority level: Very High */
    /* *
  * @}
  */
    /* * @defgroup DMA_FIFO_direct_mode DMA FIFO direct mode
  * @brief    DMA FIFO direct mode
  * @{
  */
    /* !< FIFO mode disable */
    /* !< FIFO mode enable  */
    /* *
  * @}
  */
    /* * @defgroup DMA_FIFO_threshold_level DMA FIFO threshold level
  * @brief    DMA FIFO level 
  * @{
  */
    /* !< FIFO threshold 1 quart full configuration  */
    /* !< FIFO threshold half full configuration     */
    /* !< FIFO threshold 3 quarts full configuration */
    /* !< FIFO threshold full configuration          */
    /* *
  * @}
  */
    /* * @defgroup DMA_Memory_burst DMA Memory burst
  * @brief    DMA memory burst 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_Peripheral_burst DMA Peripheral burst
  * @brief    DMA peripheral burst 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @brief    DMA interrupts definition 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_flag_definitions DMA flag definitions
  * @brief    DMA flag definitions 
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
    /* * @brief Reset DMA handle state
  * @param  __HANDLE__: specifies the DMA handle.
  * @retval None
  */
    /* *
  * @brief  Return the current DMA Stream FIFO filled level.
  * @param  __HANDLE__: DMA handle
  * @retval The FIFO filling state.
  *           - DMA_FIFOStatus_Less1QuarterFull: when FIFO is less than 1 quarter-full 
  *                                              and not empty.
  *           - DMA_FIFOStatus_1QuarterFull: if more than 1 quarter-full.
  *           - DMA_FIFOStatus_HalfFull: if more than 1 half-full.
  *           - DMA_FIFOStatus_3QuartersFull: if more than 3 quarters-full.
  *           - DMA_FIFOStatus_Empty: when FIFO is empty
  *           - DMA_FIFOStatus_Full: when FIFO is full
  */
    /* *
  * @brief  Enable the specified DMA Stream.
  * @param  __HANDLE__: DMA handle
  * @retval None
  */
    /* *
  * @brief  Disable the specified DMA Stream.
  * @param  __HANDLE__: DMA handle
  * @retval None
  */
    /* Interrupt & Flag management */
    /* *
  * @brief  Return the current DMA Stream transfer complete flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified transfer complete flag index.
  */
    /* *
  * @brief  Return the current DMA Stream half transfer complete flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified half transfer complete flag index.
  */
    /* *
  * @brief  Return the current DMA Stream transfer error flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified transfer error flag index.
  */
    /* *
  * @brief  Return the current DMA Stream FIFO error flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified FIFO error flag index.
  */
    /* *
  * @brief  Return the current DMA Stream direct mode error flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified direct mode error flag index.
  */
    /* *
  * @brief  Get the DMA Stream pending flags.
  * @param  __HANDLE__: DMA handle
  * @param  __FLAG__: Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCIFx: Transfer complete flag.
  *            @arg DMA_FLAG_HTIFx: Half transfer complete flag.
  *            @arg DMA_FLAG_TEIFx: Transfer error flag.
  *            @arg DMA_FLAG_DMEIFx: Direct mode error flag.
  *            @arg DMA_FLAG_FEIFx: FIFO error flag.
  *         Where x can be 0_4, 1_5, 2_6 or 3_7 to select the DMA Stream flag.   
  * @retval The state of FLAG (SET or RESET).
  */
    /* *
  * @brief  Clear the DMA Stream pending flags.
  * @param  __HANDLE__: DMA handle
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCIFx: Transfer complete flag.
  *            @arg DMA_FLAG_HTIFx: Half transfer complete flag.
  *            @arg DMA_FLAG_TEIFx: Transfer error flag.
  *            @arg DMA_FLAG_DMEIFx: Direct mode error flag.
  *            @arg DMA_FLAG_FEIFx: FIFO error flag.
  *         Where x can be 0_4, 1_5, 2_6 or 3_7 to select the DMA Stream flag.   
  * @retval None
  */
    /* *
  * @brief  Enable the specified DMA Stream interrupts.
  * @param  __HANDLE__: DMA handle
  * @param  __INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled. 
  *        This parameter can be one of the following values:
  *           @arg DMA_IT_TC: Transfer complete interrupt mask.
  *           @arg DMA_IT_HT: Half transfer complete interrupt mask.
  *           @arg DMA_IT_TE: Transfer error interrupt mask.
  *           @arg DMA_IT_FE: FIFO error interrupt mask.
  *           @arg DMA_IT_DME: Direct mode error interrupt.
  * @retval None
  */
    /* *
  * @brief  Disable the specified DMA Stream interrupts.
  * @param  __HANDLE__: DMA handle
  * @param  __INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled. 
  *         This parameter can be one of the following values:
  *            @arg DMA_IT_TC: Transfer complete interrupt mask.
  *            @arg DMA_IT_HT: Half transfer complete interrupt mask.
  *            @arg DMA_IT_TE: Transfer error interrupt mask.
  *            @arg DMA_IT_FE: FIFO error interrupt mask.
  *            @arg DMA_IT_DME: Direct mode error interrupt.
  * @retval None
  */
    /* *
  * @brief  Check whether the specified DMA Stream interrupt is enabled or not.
  * @param  __HANDLE__: DMA handle
  * @param  __INTERRUPT__: specifies the DMA interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg DMA_IT_TC: Transfer complete interrupt mask.
  *            @arg DMA_IT_HT: Half transfer complete interrupt mask.
  *            @arg DMA_IT_TE: Transfer error interrupt mask.
  *            @arg DMA_IT_FE: FIFO error interrupt mask.
  *            @arg DMA_IT_DME: Direct mode error interrupt.
  * @retval The state of DMA_IT.
  */
    /* *
  * @brief  Writes the number of data units to be transferred on the DMA Stream.
  * @param  __HANDLE__: DMA handle
  * @param  __COUNTER__: Number of data units to be transferred (from 0 to 65535) 
  *          Number of data items depends only on the Peripheral data format.
  *            
  * @note   If Peripheral data format is Bytes: number of data units is equal 
  *         to total number of bytes to be transferred.
  *           
  * @note   If Peripheral data format is Half-Word: number of data units is  
  *         equal to total number of bytes to be transferred / 2.
  *           
  * @note   If Peripheral data format is Word: number of data units is equal 
  *         to total  number of bytes to be transferred / 4.
  *      
  * @retval The number of remaining data units in the current DMAy Streamx transfer.
  */
    /* *
  * @brief  Returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param  __HANDLE__: DMA handle
  *   
  * @retval The number of remaining data units in the current DMA Stream transfer.
  */
    /* Include DMA HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
    /* * @defgroup DMA_Exported_Functions DMA Exported Functions
  * @brief    DMA Exported functions 
  * @{
  */
    /* * @defgroup DMA_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief   Initialization and de-initialization functions 
  * @{
  */
    #[no_mangle]
    fn HAL_DMA_Init(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
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
    fn HAL_ADC_Start_DMA(hadc: *mut ADC_HandleTypeDef, pData: *mut uint32_t,
                         Length: uint32_t) -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup ADC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions *************************************************/
    #[no_mangle]
    fn HAL_ADC_ConfigChannel(hadc: *mut ADC_HandleTypeDef,
                             sConfig: *mut ADC_ChannelConfTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    fn dmaGetIdentifier(stream: *const DMA_Stream_TypeDef) -> dmaIdentifier_e;
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
    #[no_mangle]
    static mut adcVREFINTCAL: uint16_t;
    #[no_mangle]
    static mut adcTSCAL1: uint16_t;
    #[no_mangle]
    static mut adcTSCAL2: uint16_t;
    #[no_mangle]
    static mut adcTSSlopeK: uint16_t;
    #[no_mangle]
    fn adcDeviceByInstance(instance: *mut ADC_TypeDef) -> ADCDevice;
    #[no_mangle]
    static mut adcOperatingConfig: [adcOperatingConfig_t; 4];
    #[no_mangle]
    static mut adcValues: [uint16_t; 4];
    #[no_mangle]
    fn adcChannelByTag(ioTag: ioTag_t) -> uint8_t;
    #[no_mangle]
    fn adcVerifyPin(tag: ioTag_t, device: ADCDevice) -> bool;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
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
  * @brief  Structure definition of ADC channel for regular group   
  * @note   The setting of these parameters with function HAL_ADC_ConfigChannel() is conditioned to ADC state.
  *         ADC can be either disabled or enabled without conversion on going on regular group.
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_ChannelConfTypeDef {
    pub Channel: uint32_t,
    pub Rank: uint32_t,
    pub SamplingTime: uint32_t,
    pub Offset: uint32_t,
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
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
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
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
pub type dmaIdentifier_e = libc::c_uint;
pub const DMA_LAST_HANDLER: dmaIdentifier_e = 16;
pub const DMA2_ST7_HANDLER: dmaIdentifier_e = 16;
pub const DMA2_ST6_HANDLER: dmaIdentifier_e = 15;
pub const DMA2_ST5_HANDLER: dmaIdentifier_e = 14;
pub const DMA2_ST4_HANDLER: dmaIdentifier_e = 13;
pub const DMA2_ST3_HANDLER: dmaIdentifier_e = 12;
pub const DMA2_ST2_HANDLER: dmaIdentifier_e = 11;
pub const DMA2_ST1_HANDLER: dmaIdentifier_e = 10;
pub const DMA2_ST0_HANDLER: dmaIdentifier_e = 9;
pub const DMA1_ST7_HANDLER: dmaIdentifier_e = 8;
pub const DMA1_ST6_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_ST5_HANDLER: dmaIdentifier_e = 6;
pub const DMA1_ST4_HANDLER: dmaIdentifier_e = 5;
pub const DMA1_ST3_HANDLER: dmaIdentifier_e = 4;
pub const DMA1_ST2_HANDLER: dmaIdentifier_e = 3;
pub const DMA1_ST1_HANDLER: dmaIdentifier_e = 2;
pub const DMA1_ST0_HANDLER: dmaIdentifier_e = 1;
pub const DMA_NONE: dmaIdentifier_e = 0;
pub type ADCDevice = libc::c_int;
pub const ADCDEV_COUNT: ADCDevice = 3;
pub const ADCDEV_3: ADCDevice = 2;
pub const ADCDEV_2: ADCDevice = 1;
pub const ADCDEV_1: ADCDevice = 0;
pub const ADCINVALID: ADCDevice = -1;
pub type C2RustUnnamed = libc::c_uint;
pub const ADC_CHANNEL_COUNT: C2RustUnnamed = 4;
pub const ADC_RSSI: C2RustUnnamed = 3;
pub const ADC_EXTERNAL1: C2RustUnnamed = 2;
pub const ADC_CURRENT: C2RustUnnamed = 1;
pub const ADC_BATTERY: C2RustUnnamed = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcOperatingConfig_s {
    pub tag: ioTag_t,
    pub adcChannel: uint8_t,
    pub dmaIndex: uint8_t,
    pub enabled: bool,
    pub sampleTime: uint8_t,
}
pub type adcOperatingConfig_t = adcOperatingConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcConfig_s {
    pub vbat: adcChannelConfig_t,
    pub rssi: adcChannelConfig_t,
    pub current: adcChannelConfig_t,
    pub external1: adcChannelConfig_t,
    pub device: int8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcChannelConfig_t {
    pub enabled: bool,
    pub ioTag: ioTag_t,
}
pub type adcConfig_t = adcConfig_s;
pub type adcDevice_t = adcDevice_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcTagMap_s {
    pub tag: ioTag_t,
    pub devices: uint8_t,
    pub channel: uint8_t,
}
// ADC1_INxx channel number
// index into DMA buffer in case of sparse channels
// ADCDevice
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
pub type adcTagMap_t = adcTagMap_s;
// F1 pins have uniform connection to ADC instances
// Initialized in run_static_initializers
#[no_mangle]
pub static mut adcHardware: [adcDevice_t; 3] =
    [adcDevice_t{ADCx: 0 as *mut ADC_TypeDef,
                 rccADC: 0,
                 DMAy_Streamx: 0 as *mut DMA_Stream_TypeDef,
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
                                                           ExternalTrigConv:
                                                               0,
                                                           ExternalTrigConvEdge:
                                                               0,
                                                           DMAContinuousRequests:
                                                               0,},
                                       NbrOfCurrentConversionRank: 0,
                                       DMA_Handle:
                                           0 as *mut DMA_HandleTypeDef,
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
                                                           MemDataAlignment:
                                                               0,
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
                                       StreamIndex: 0,},}; 3];
/* note these could be packed up for saving space */
#[no_mangle]
pub static mut adcTagMap: [adcTagMap_t; 16] =
    [{
         let mut init =
             adcTagMap_s{tag:
                             ((2 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 0 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_3 as libc::c_int) as uint8_t,
                         channel:
                             ((0x8 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x2 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((2 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 1 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_3 as libc::c_int) as uint8_t,
                         channel:
                             ((0x8 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x2 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x1 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((2 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 2 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_3 as libc::c_int) as uint8_t,
                         channel:
                             ((0x8 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x4 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((2 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 3 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_3 as libc::c_int) as uint8_t,
                         channel:
                             ((0x8 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x4 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x1 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((2 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 4 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel:
                             ((0x8 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x4 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x2 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((2 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 5 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel:
                             ((0x8 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x4 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x2 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x1 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 0 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel:
                             ((0x8 as libc::c_uint) << 0 as libc::c_uint) as
                                 uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 1 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel:
                             ((0x8 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x1 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 0 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_3 as libc::c_int) as uint8_t,
                         channel: 0 as libc::c_uint as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 1 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_3 as libc::c_int) as uint8_t,
                         channel:
                             ((0x1 as libc::c_uint) << 0 as libc::c_uint) as
                                 uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 2 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_3 as libc::c_int) as uint8_t,
                         channel:
                             ((0x2 as libc::c_uint) << 0 as libc::c_uint) as
                                 uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 3 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_3 as libc::c_int) as uint8_t,
                         channel:
                             ((0x2 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x1 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 4 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel:
                             ((0x4 as libc::c_uint) << 0 as libc::c_uint) as
                                 uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 5 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel:
                             ((0x4 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x1 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 6 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel:
                             ((0x4 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x2 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 7 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel:
                             ((0x4 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x2 as libc::c_uint) << 0 as libc::c_uint |
                                  (0x1 as libc::c_uint) << 0 as libc::c_uint)
                                 as uint8_t,};
         init
     }];
#[no_mangle]
pub unsafe extern "C" fn adcInitDevice(mut adcdev: *mut adcDevice_t,
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
static mut adc: adcDevice_t =
    adcDevice_t{ADCx: 0 as *mut ADC_TypeDef,
                rccADC: 0,
                DMAy_Streamx: 0 as *mut DMA_Stream_TypeDef,
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
static mut adcInternal: adcDevice_t =
    adcDevice_t{ADCx: 0 as *mut ADC_TypeDef,
                rccADC: 0,
                DMAy_Streamx: 0 as *mut DMA_Stream_TypeDef,
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
static mut adcInternalHandle: *mut ADC_HandleTypeDef =
    0 as *const ADC_HandleTypeDef as *mut ADC_HandleTypeDef;
#[no_mangle]
pub unsafe extern "C" fn adcInitInternalInjected(mut adcdev:
                                                     *mut adcDevice_t) {
    adcInternalHandle = &mut (*adcdev).ADCHandle;
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
    iConfig.InjectedChannel =
        (0x10 as libc::c_uint) << 0 as libc::c_uint |
            (0x1 as libc::c_uint) << 0 as libc::c_uint;
    iConfig.InjectedRank = 1 as libc::c_int as uint32_t;
    iConfig.InjectedSamplingTime = (0x7 as libc::c_uint) << 0 as libc::c_uint;
    iConfig.InjectedOffset = 0 as libc::c_int as uint32_t;
    iConfig.InjectedNbrOfConversion = 2 as libc::c_int as uint32_t;
    iConfig.InjectedDiscontinuousConvMode =
        DISABLE as libc::c_int as uint32_t;
    iConfig.AutoInjectedConv = DISABLE as libc::c_int as uint32_t;
    iConfig.ExternalTrigInjecConv = 0 as libc::c_int as uint32_t;
    iConfig.ExternalTrigInjecConvEdge = 0 as libc::c_int as uint32_t;
    (HAL_ADCEx_InjectedConfigChannel(adcInternalHandle, &mut iConfig) as
         libc::c_uint) != HAL_OK as libc::c_int as libc::c_uint;
    iConfig.InjectedChannel =
        (0x10 as libc::c_uint) << 0 as libc::c_uint |
            (0x2 as libc::c_uint) << 0 as libc::c_uint |
            0x10000000 as libc::c_uint;
    iConfig.InjectedRank = 2 as libc::c_int as uint32_t;
    (HAL_ADCEx_InjectedConfigChannel(adcInternalHandle, &mut iConfig) as
         libc::c_uint) != HAL_OK as libc::c_int as libc::c_uint;
    adcVREFINTCAL = *(0x1ff0f44a as libc::c_int as *mut uint16_t);
    adcTSCAL1 = *(0x1ff0f44c as libc::c_int as *mut uint16_t);
    adcTSCAL2 = *(0x1ff0f44e as libc::c_int as *mut uint16_t);
    adcTSSlopeK =
        ((110 as libc::c_int - 30 as libc::c_int) * 1000 as libc::c_int /
             (adcTSCAL2 as libc::c_int - adcTSCAL1 as libc::c_int)) as
            uint16_t;
}
// Note on sampling time for temperature sensor and vrefint:
// Both sources have minimum sample time of 10us.
// With prescaler = 8:
// 168MHz : fAPB2 = 84MHz, fADC = 10.5MHz, tcycle = 0.090us, 10us = 105cycle < 144cycle
// 240MHz : fAPB2 = 120MHz, fADC = 15.0MHz, tcycle = 0.067usk 10us = 150cycle < 480cycle
//
// 480cycles@15.0MHz = 32us
static mut adcInternalConversionInProgress: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub unsafe extern "C" fn adcInternalIsBusy() -> bool {
    if adcInternalConversionInProgress {
        if HAL_ADCEx_InjectedPollForConversion(adcInternalHandle,
                                               0 as libc::c_int as uint32_t)
               as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
            adcInternalConversionInProgress = 0 as libc::c_int != 0
        }
    }
    return adcInternalConversionInProgress;
}
#[no_mangle]
pub unsafe extern "C" fn adcInternalStartConversion() {
    HAL_ADCEx_InjectedStart(adcInternalHandle);
    adcInternalConversionInProgress = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn adcInternalReadVrefint() -> uint16_t {
    return HAL_ADCEx_InjectedGetValue(adcInternalHandle, 0x1 as libc::c_uint)
               as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn adcInternalReadTempsensor() -> uint16_t {
    return HAL_ADCEx_InjectedGetValue(adcInternalHandle, 0x2 as libc::c_uint)
               as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn adcInit(mut config: *const adcConfig_t) {
    let mut i: uint8_t = 0;
    let mut configuredAdcChannels: uint8_t = 0 as libc::c_int as uint8_t;
    memset(&mut adcOperatingConfig as *mut [adcOperatingConfig_t; 4] as
               *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[adcOperatingConfig_t; 4]>() as
               libc::c_ulong);
    if (*config).vbat.enabled {
        adcOperatingConfig[ADC_BATTERY as libc::c_int as usize].tag =
            (*config).vbat.ioTag
    }
    if (*config).rssi.enabled {
        adcOperatingConfig[ADC_RSSI as libc::c_int as usize].tag =
            (*config).rssi.ioTag
        //RSSI_ADC_CHANNEL;
    }
    if (*config).external1.enabled {
        adcOperatingConfig[ADC_EXTERNAL1 as libc::c_int as usize].tag =
            (*config).external1.ioTag
        //EXTERNAL1_ADC_CHANNEL;
    }
    if (*config).current.enabled {
        adcOperatingConfig[ADC_CURRENT as libc::c_int as usize].tag =
            (*config).current.ioTag
        //CURRENT_METER_ADC_CHANNEL;
    }
    let mut device: ADCDevice =
        adcDeviceByInstance((0x40000000 as
                                 libc::c_uint).wrapping_add(0x10000 as
                                                                libc::c_uint).wrapping_add(0x2000
                                                                                               as
                                                                                               libc::c_uint)
                                as *mut ADC_TypeDef);
    if device as libc::c_int == ADCINVALID as libc::c_int { return }
    adc = adcHardware[device as usize];
    let mut adcActive: bool = 0 as libc::c_int != 0;
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < ADC_CHANNEL_COUNT as libc::c_int {
        if adcVerifyPin(adcOperatingConfig[i_0 as usize].tag, device) {
            adcActive = 1 as libc::c_int != 0;
            IOInit(IOGetByTag(adcOperatingConfig[i_0 as usize].tag),
                   (OWNER_ADC_BATT as libc::c_int + i_0) as resourceOwner_e,
                   0 as libc::c_int as uint8_t);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i_0 as usize].tag),
                         (0x3 as libc::c_uint |
                              ((0 as libc::c_int) << 2 as libc::c_int) as
                                  libc::c_uint |
                              (0 as libc::c_uint) << 5 as libc::c_int) as
                             ioConfig_t);
            adcOperatingConfig[i_0 as usize].adcChannel =
                adcChannelByTag(adcOperatingConfig[i_0 as usize].tag);
            let fresh0 = configuredAdcChannels;
            configuredAdcChannels = configuredAdcChannels.wrapping_add(1);
            adcOperatingConfig[i_0 as usize].dmaIndex = fresh0;
            adcOperatingConfig[i_0 as usize].sampleTime =
                ((0x7 as libc::c_uint) << 0 as libc::c_uint) as uint8_t;
            adcOperatingConfig[i_0 as usize].enabled = 1 as libc::c_int != 0
        }
        i_0 += 1
    }
    RCC_ClockCmd(adc.rccADC, ENABLE);
    adcInitDevice(&mut adc, configuredAdcChannels as libc::c_int);
    // If device is not ADC1 or there's no active channel, then initialize ADC1  here.
    if device as libc::c_int != ADCDEV_1 as libc::c_int || !adcActive {
        adcInternal = adcHardware[ADCDEV_1 as libc::c_int as usize];
        RCC_ClockCmd(adcInternal.rccADC, ENABLE);
        adcInitDevice(&mut adcInternal, 0 as libc::c_int);
        adcInitInternalInjected(&mut adcInternal);
    } else { adcInitInternalInjected(&mut adc); }
    let mut rank: uint8_t = 1 as libc::c_int as uint8_t;
    i = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < ADC_CHANNEL_COUNT as libc::c_int {
        if adcOperatingConfig[i as usize].enabled {
            let mut sConfig: ADC_ChannelConfTypeDef =
                ADC_ChannelConfTypeDef{Channel: 0,
                                       Rank: 0,
                                       SamplingTime: 0,
                                       Offset: 0,};
            sConfig.Channel =
                adcOperatingConfig[i as usize].adcChannel as uint32_t;
            let fresh1 = rank;
            rank = rank.wrapping_add(1);
            sConfig.Rank = fresh1 as uint32_t;
            sConfig.SamplingTime =
                adcOperatingConfig[i as usize].sampleTime as uint32_t;
            sConfig.Offset = 0 as libc::c_int as uint32_t;
            (HAL_ADC_ConfigChannel(&mut adc.ADCHandle, &mut sConfig) as
                 libc::c_uint) != HAL_OK as libc::c_int as libc::c_uint;
        }
        i = i.wrapping_add(1)
    }
    dmaInit(dmaGetIdentifier(adc.DMAy_Streamx), OWNER_ADC,
            0 as libc::c_int as uint8_t);
    adc.DmaHandle.Init.Channel = adc.channel;
    adc.DmaHandle.Init.Direction = 0 as libc::c_uint;
    adc.DmaHandle.Init.PeriphInc = 0 as libc::c_uint;
    adc.DmaHandle.Init.MemInc =
        if configuredAdcChannels as libc::c_int > 1 as libc::c_int {
            ((0x1 as libc::c_uint)) << 10 as libc::c_uint
        } else { 0 as libc::c_uint };
    adc.DmaHandle.Init.PeriphDataAlignment =
        (0x1 as libc::c_uint) << 11 as libc::c_uint;
    adc.DmaHandle.Init.MemDataAlignment =
        (0x1 as libc::c_uint) << 13 as libc::c_uint;
    adc.DmaHandle.Init.Mode = (0x1 as libc::c_uint) << 8 as libc::c_uint;
    adc.DmaHandle.Init.Priority = (0x2 as libc::c_uint) << 16 as libc::c_uint;
    adc.DmaHandle.Init.FIFOMode = 0 as libc::c_uint;
    adc.DmaHandle.Init.FIFOThreshold =
        (0x3 as libc::c_uint) << 0 as libc::c_uint;
    adc.DmaHandle.Init.MemBurst = 0 as libc::c_uint;
    adc.DmaHandle.Init.PeriphBurst = 0 as libc::c_uint;
    adc.DmaHandle.Instance = adc.DMAy_Streamx;
    (HAL_DMA_Init(&mut adc.DmaHandle) as libc::c_uint) !=
        HAL_OK as libc::c_int as libc::c_uint;
    adc.ADCHandle.DMA_Handle = &mut adc.DmaHandle;
    adc.DmaHandle.Parent =
        &mut adc.ADCHandle as *mut ADC_HandleTypeDef as *mut libc::c_void;
    //HAL_CLEANINVALIDATECACHE((uint32_t*)&adcValues, configuredAdcChannels);
    (HAL_ADC_Start_DMA(&mut adc.ADCHandle,
                       &mut adcValues as *mut [uint16_t; 4] as *mut uint32_t,
                       configuredAdcChannels as uint32_t) as libc::c_uint) !=
        HAL_OK as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn run_static_initializers() {
    adcHardware =
        [{
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
                                                              as libc::c_long
                                                              >
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
                                                              as libc::c_long
                                                              *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        8 as
                                                                            libc::c_uint)
                                                                       as
                                                                       libc::c_long
                                                                       >
                                                                       65535
                                                                           as
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
                                                                 libc::c_long
                                                             +
                                                             14 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             |
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
                                                              as libc::c_long
                                                              *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        8 as
                                                                            libc::c_uint)
                                                                       as
                                                                       libc::c_long
                                                                       >
                                                                       65535
                                                                           as
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
                                                                 libc::c_long
                                                             +
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
                                                                       Mode:
                                                                           0,
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
         },
         {
             let mut init =
                 adcDevice_s{ADCx:
                                 (0x40000000 as
                                      libc::c_uint).wrapping_add(0x10000 as
                                                                     libc::c_uint).wrapping_add(0x2100
                                                                                                    as
                                                                                                    libc::c_uint)
                                     as *mut ADC_TypeDef,
                             rccADC:
                                 (((RCC_APB2 as libc::c_int) <<
                                       5 as libc::c_int) as libc::c_long |
                                      (16 as libc::c_int *
                                           (((0x1 as libc::c_uint) <<
                                                 9 as libc::c_uint) as
                                                libc::c_long >
                                                65535 as libc::c_long) as
                                               libc::c_int) as libc::c_long +
                                          ((8 as libc::c_int *
                                                (((0x1 as libc::c_uint) <<
                                                      9 as libc::c_uint) as
                                                     libc::c_long *
                                                     1 as libc::c_long >>
                                                     16 as libc::c_int *
                                                         (((0x1 as
                                                                libc::c_uint)
                                                               <<
                                                               9 as
                                                                   libc::c_uint)
                                                              as libc::c_long
                                                              >
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
                                                               9 as
                                                                   libc::c_uint)
                                                              as libc::c_long
                                                              *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        9 as
                                                                            libc::c_uint)
                                                                       as
                                                                       libc::c_long
                                                                       >
                                                                       65535
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int
                                                              >>
                                                              8 as libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        9 as
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
                                                                                 9
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
                                                                 libc::c_long
                                                             +
                                                             14 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             |
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long)
                                                    -
                                                    2 as libc::c_int as
                                                        libc::c_long /
                                                        ((((0x1 as
                                                                libc::c_uint)
                                                               <<
                                                               9 as
                                                                   libc::c_uint)
                                                              as libc::c_long
                                                              *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        9 as
                                                                            libc::c_uint)
                                                                       as
                                                                       libc::c_long
                                                                       >
                                                                       65535
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int
                                                              >>
                                                              8 as libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        9 as
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
                                                                                 9
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
                                                                 libc::c_long
                                                             +
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long))))
                                     as rccPeriphTag_t,
                             DMAy_Streamx:
                                 (0x40000000 as
                                      libc::c_uint).wrapping_add(0x20000 as
                                                                     libc::c_uint).wrapping_add(0x6400
                                                                                                    as
                                                                                                    libc::c_uint).wrapping_add(0x58
                                                                                                                                   as
                                                                                                                                   libc::c_uint)
                                     as *mut DMA_Stream_TypeDef,
                             channel: 0x2000000 as libc::c_uint,
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
                                                                       Mode:
                                                                           0,
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
         },
         {
             let mut init =
                 adcDevice_s{ADCx:
                                 (0x40000000 as
                                      libc::c_uint).wrapping_add(0x10000 as
                                                                     libc::c_uint).wrapping_add(0x2200
                                                                                                    as
                                                                                                    libc::c_uint)
                                     as *mut ADC_TypeDef,
                             rccADC:
                                 (((RCC_APB2 as libc::c_int) <<
                                       5 as libc::c_int) as libc::c_long |
                                      (16 as libc::c_int *
                                           (((0x1 as libc::c_uint) <<
                                                 10 as libc::c_uint) as
                                                libc::c_long >
                                                65535 as libc::c_long) as
                                               libc::c_int) as libc::c_long +
                                          ((8 as libc::c_int *
                                                (((0x1 as libc::c_uint) <<
                                                      10 as libc::c_uint) as
                                                     libc::c_long *
                                                     1 as libc::c_long >>
                                                     16 as libc::c_int *
                                                         (((0x1 as
                                                                libc::c_uint)
                                                               <<
                                                               10 as
                                                                   libc::c_uint)
                                                              as libc::c_long
                                                              >
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
                                                               10 as
                                                                   libc::c_uint)
                                                              as libc::c_long
                                                              *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        10 as
                                                                            libc::c_uint)
                                                                       as
                                                                       libc::c_long
                                                                       >
                                                                       65535
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int
                                                              >>
                                                              8 as libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        10 as
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
                                                                                 10
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
                                                                 libc::c_long
                                                             +
                                                             14 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             |
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long)
                                                    -
                                                    2 as libc::c_int as
                                                        libc::c_long /
                                                        ((((0x1 as
                                                                libc::c_uint)
                                                               <<
                                                               10 as
                                                                   libc::c_uint)
                                                              as libc::c_long
                                                              *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        10 as
                                                                            libc::c_uint)
                                                                       as
                                                                       libc::c_long
                                                                       >
                                                                       65535
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int
                                                              >>
                                                              8 as libc::c_int
                                                                  *
                                                                  (((0x1 as
                                                                         libc::c_uint)
                                                                        <<
                                                                        10 as
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
                                                                                 10
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
                                                                 libc::c_long
                                                             +
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long))))
                                     as rccPeriphTag_t,
                             DMAy_Streamx:
                                 (0x40000000 as
                                      libc::c_uint).wrapping_add(0x20000 as
                                                                     libc::c_uint).wrapping_add(0x6400
                                                                                                    as
                                                                                                    libc::c_uint).wrapping_add(0x10
                                                                                                                                   as
                                                                                                                                   libc::c_uint)
                                     as *mut DMA_Stream_TypeDef,
                             channel: 0x4000000 as libc::c_uint,
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
                                                                       Mode:
                                                                           0,
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
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
