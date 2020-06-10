use ::libc;
extern "C" {
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
    #[no_mangle]
    fn HAL_DMA_IRQHandler(hdma: *mut DMA_HandleTypeDef);
    /* *
  * @}
  */
    /* * @addtogroup TIMEx_Exported_Functions_Group3
  * @{
  */
/*  Timer Complementary PWM functions  ****************************************/
/* Blocking mode: Polling */
    #[no_mangle]
    fn HAL_TIMEx_PWMN_Start(htim: *mut TIM_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_PWM_Init(htim: *mut TIM_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_PWM_Start(htim: *mut TIM_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_PWM_ConfigChannel(htim: *mut TIM_HandleTypeDef,
                                 sConfig: *mut TIM_OC_InitTypeDef,
                                 Channel: uint32_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    static mut BIT_COMPARE_0: uint16_t;
    #[no_mangle]
    static mut ws2811LedDataTransferInProgress: uint8_t;
    #[no_mangle]
    static mut ledStripDMABuffer: [uint32_t; 810];
    #[no_mangle]
    static mut BIT_COMPARE_1: uint16_t;
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
    #[no_mangle]
    fn dmaSetHandler(identifier: dmaIdentifier_e,
                     callback: dmaCallbackHandlerFuncPtr, priority: uint32_t,
                     userParam: uint32_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    fn TIM_DMACmd(htim: *mut TIM_HandleTypeDef, Channel: uint32_t,
                  NewState: FunctionalState) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn DMA_SetCurrDataCounter(htim: *mut TIM_HandleTypeDef, Channel: uint32_t,
                              pData: *mut uint32_t, Length: uint16_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn timerDmaIndex(channel: uint8_t) -> uint16_t;
    #[no_mangle]
    fn timerGetPrescalerByDesiredMhz(tim: *mut TIM_TypeDef, mhz: uint16_t)
     -> uint16_t;
    #[no_mangle]
    fn timerGetPeriodByPrescaler(tim: *mut TIM_TypeDef, prescaler: uint16_t,
                                 hz: uint32_t) -> uint16_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
pub const LTDC_ER_IRQn: IRQn_Type = 89;
pub const LTDC_IRQn: IRQn_Type = 88;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_TypeDef {
    pub LISR: uint32_t,
    pub HISR: uint32_t,
    pub LIFCR: uint32_t,
    pub HIFCR: uint32_t,
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
/* !< DMA Stream Index                       */
// 0 - 359
// 0 - 255
// 0 - 255
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed = 2;
pub type timerHardware_t = timerHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Stream_TypeDef,
    pub dmaChannel: uint32_t,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Stream_TypeDef,
    pub dmaTimUPChannel: uint32_t,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed = 1;
pub type dmaChannelDescriptor_t = dmaChannelDescriptor_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dmaChannelDescriptor_s {
    pub dma: *mut DMA_TypeDef,
    pub ref_0: *mut DMA_Stream_TypeDef,
    pub stream: uint8_t,
    pub irqHandlerCallback: dmaCallbackHandlerFuncPtr,
    pub flagsShift: uint8_t,
    pub irqN: IRQn_Type,
    pub userParam: uint32_t,
    pub owner: resourceOwner_e,
    pub resourceIndex: uint8_t,
    pub completeFlag: uint32_t,
}
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
// TIMUP
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
pub type dmaCallbackHandlerFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut dmaChannelDescriptor_s) -> ()>;
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
pub type C2RustUnnamed = libc::c_uint;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed = 0;
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
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
static mut ws2811IO: IO_t = 0 as *const libc::c_void as IO_t;
#[no_mangle]
pub static mut ws2811Initialised: bool = 0 as libc::c_int != 0;
static mut TimHandle: TIM_HandleTypeDef =
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
static mut timerChannel: uint16_t = 0 as libc::c_int as uint16_t;
#[no_mangle]
pub unsafe extern "C" fn WS2811_DMA_IRQHandler(mut descriptor:
                                                   *mut dmaChannelDescriptor_t) {
    HAL_DMA_IRQHandler(TimHandle.hdma[(*descriptor).userParam as usize]);
    TIM_DMACmd(&mut TimHandle, timerChannel as uint32_t, DISABLE);
    ::core::ptr::write_volatile(&mut ws2811LedDataTransferInProgress as
                                    *mut uint8_t,
                                0 as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn ws2811LedStripHardwareInit(mut ioTag: ioTag_t) {
    if ioTag == 0 { return }
    let mut timerHardware: *const timerHardware_t = timerGetByTag(ioTag);
    let mut timer: *mut TIM_TypeDef = (*timerHardware).tim;
    timerChannel = (*timerHardware).channel as uint16_t;
    if (*timerHardware).dmaRef.is_null() { return }
    TimHandle.Instance = timer;
    /* Compute the prescaler value */
    let mut prescaler: uint16_t =
        timerGetPrescalerByDesiredMhz(timer,
                                      48 as libc::c_int as
                                          uint16_t); // 800kHz
    let mut period: uint16_t =
        timerGetPeriodByPrescaler(timer, prescaler,
                                  800000 as libc::c_int as uint32_t);
    BIT_COMPARE_1 =
        (period as libc::c_int / 3 as libc::c_int * 2 as libc::c_int) as
            uint16_t;
    BIT_COMPARE_0 = (period as libc::c_int / 3 as libc::c_int) as uint16_t;
    TimHandle.Init.Prescaler = prescaler as uint32_t;
    TimHandle.Init.Period = period as uint32_t;
    TimHandle.Init.ClockDivision = 0 as libc::c_uint;
    TimHandle.Init.CounterMode = 0 as libc::c_uint;
    if HAL_TIM_PWM_Init(&mut TimHandle) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        /* Initialization Error */
        return
    }
    static mut hdma_tim: DMA_HandleTypeDef =
        DMA_HandleTypeDef{Instance:
                              0 as *const DMA_Stream_TypeDef as
                                  *mut DMA_Stream_TypeDef,
                          Init:
                              DMA_InitTypeDef{Channel: 0,
                                              Direction: 0,
                                              PeriphInc: 0,
                                              MemInc: 0,
                                              PeriphDataAlignment: 0,
                                              MemDataAlignment: 0,
                                              Mode: 0,
                                              Priority: 0,
                                              FIFOMode: 0,
                                              FIFOThreshold: 0,
                                              MemBurst: 0,
                                              PeriphBurst: 0,},
                          Lock: HAL_UNLOCKED,
                          State: HAL_DMA_STATE_RESET,
                          Parent:
                              0 as *const libc::c_void as *mut libc::c_void,
                          XferCpltCallback: None,
                          XferHalfCpltCallback: None,
                          XferM1CpltCallback: None,
                          XferM1HalfCpltCallback: None,
                          XferErrorCallback: None,
                          XferAbortCallback: None,
                          ErrorCode: 0,
                          StreamBaseAddress: 0,
                          StreamIndex: 0,};
    ws2811IO = IOGetByTag(ioTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0 as libc::c_int as uint8_t);
    IOConfigGPIOAF(ws2811IO,
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0x2 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, (*timerHardware).alternateFunction);
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         21 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        21 as libc::c_uint);
    /* Set the parameters to be configured */
    hdma_tim.Init.Channel = (*timerHardware).dmaChannel;
    hdma_tim.Init.Direction = (0x1 as libc::c_uint) << 6 as libc::c_uint;
    hdma_tim.Init.PeriphInc = 0 as libc::c_uint;
    hdma_tim.Init.MemInc = (0x1 as libc::c_uint) << 10 as libc::c_uint;
    hdma_tim.Init.PeriphDataAlignment =
        (0x2 as libc::c_uint) << 11 as libc::c_uint;
    hdma_tim.Init.MemDataAlignment =
        (0x2 as libc::c_uint) << 13 as libc::c_uint;
    hdma_tim.Init.Mode = 0 as libc::c_uint;
    hdma_tim.Init.Priority = (0x2 as libc::c_uint) << 16 as libc::c_uint;
    hdma_tim.Init.FIFOMode = 0 as libc::c_uint;
    hdma_tim.Init.FIFOThreshold = (0x3 as libc::c_uint) << 0 as libc::c_uint;
    hdma_tim.Init.MemBurst = 0 as libc::c_uint;
    hdma_tim.Init.PeriphBurst = 0 as libc::c_uint;
    /* Set hdma_tim instance */
    hdma_tim.Instance = (*timerHardware).dmaRef;
    let mut dmaIndex: uint16_t = timerDmaIndex(timerChannel as uint8_t);
    /* Link hdma_tim to hdma[x] (channelx) */
    TimHandle.hdma[dmaIndex as usize] = &mut hdma_tim;
    hdma_tim.Parent =
        &mut TimHandle as *mut TIM_HandleTypeDef as *mut libc::c_void;
    dmaInit((*timerHardware).dmaIrqHandler as dmaIdentifier_e,
            OWNER_LED_STRIP, 0 as libc::c_int as uint8_t);
    dmaSetHandler((*timerHardware).dmaIrqHandler as dmaIdentifier_e,
                  Some(WS2811_DMA_IRQHandler as
                           unsafe extern "C" fn(_:
                                                    *mut dmaChannelDescriptor_t)
                               -> ()),
                  (((1 as libc::c_int) <<
                        (4 as libc::c_int as
                             libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                             libc::c_uint).wrapping_sub(0x5
                                                                                            as
                                                                                            libc::c_uint))
                        |
                        2 as libc::c_int &
                            0xf as libc::c_int >>
                                (7 as libc::c_int as
                                     libc::c_uint).wrapping_sub(0x5 as
                                                                    libc::c_uint))
                       << 4 as libc::c_int & 0xf0 as libc::c_int) as uint32_t,
                  dmaIndex as uint32_t);
    /* Initialize TIMx DMA handle */
    if HAL_DMA_Init(TimHandle.hdma[dmaIndex as usize]) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        /* Initialization Error */
        return
    }
    let mut TIM_OCInitStructure: TIM_OC_InitTypeDef =
        TIM_OC_InitTypeDef{OCMode: 0,
                           Pulse: 0,
                           OCPolarity: 0,
                           OCNPolarity: 0,
                           OCFastMode: 0,
                           OCIdleState: 0,
                           OCNIdleState: 0,};
    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode =
        (0x4 as libc::c_uint) << 4 as libc::c_uint |
            (0x2 as libc::c_uint) << 4 as libc::c_uint;
    TIM_OCInitStructure.OCIdleState = 0 as libc::c_uint;
    TIM_OCInitStructure.OCPolarity =
        if (*timerHardware).output as libc::c_int &
               TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
            ((0x1 as libc::c_uint)) << 1 as libc::c_uint
        } else { 0 as libc::c_uint };
    TIM_OCInitStructure.OCNIdleState = 0 as libc::c_uint;
    TIM_OCInitStructure.OCNPolarity =
        if (*timerHardware).output as libc::c_int &
               TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
            ((0x1 as libc::c_uint)) << 3 as libc::c_uint
        } else { 0 as libc::c_uint };
    TIM_OCInitStructure.Pulse = 0 as libc::c_int as uint32_t;
    TIM_OCInitStructure.OCFastMode = 0 as libc::c_uint;
    if HAL_TIM_PWM_ConfigChannel(&mut TimHandle, &mut TIM_OCInitStructure,
                                 timerChannel as uint32_t) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        /* Configuration Error */
        return
    }
    if (*timerHardware).output as libc::c_int &
           TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
        if HAL_TIMEx_PWMN_Start(&mut TimHandle, timerChannel as uint32_t) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            /* Starting PWM generation Error */
            return
        }
    } else if HAL_TIM_PWM_Start(&mut TimHandle, timerChannel as uint32_t) as
                  libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        /* Starting PWM generation Error */
        return
    }
    ws2811Initialised = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn ws2811LedStripDMAEnable() {
    if !ws2811Initialised {
        ::core::ptr::write_volatile(&mut ws2811LedDataTransferInProgress as
                                        *mut uint8_t,
                                    0 as libc::c_int as uint8_t);
        return
    }
    if DMA_SetCurrDataCounter(&mut TimHandle, timerChannel as uint32_t,
                              ledStripDMABuffer.as_mut_ptr(),
                              (24 as libc::c_int * 32 as libc::c_int +
                                   42 as libc::c_int) as uint16_t) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        /* DMA set error */
        ::core::ptr::write_volatile(&mut ws2811LedDataTransferInProgress as
                                        *mut uint8_t,
                                    0 as libc::c_int as uint8_t);
        return
    }
    /* Reset timer counter */
    ::core::ptr::write_volatile(&mut (*TimHandle.Instance).CNT as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Enable channel DMA requests */
    TIM_DMACmd(&mut TimHandle, timerChannel as uint32_t, ENABLE);
}
