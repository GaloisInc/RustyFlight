use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn HAL_NVIC_SetPriority(IRQn: IRQn_Type, PreemptPriority: uint32_t,
                            SubPriority: uint32_t);
    #[no_mangle]
    fn HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
    #[no_mangle]
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIMEx_MasterConfigSynchronization(htim: *mut TIM_HandleTypeDef,
                                             sMasterConfig:
                                                 *mut TIM_MasterConfigTypeDef)
     -> HAL_StatusTypeDef;
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
    #[no_mangle]
    fn HAL_TIM_Base_Init(htim: *mut TIM_HandleTypeDef) -> HAL_StatusTypeDef;
    /* Blocking mode: Polling */
    #[no_mangle]
    fn HAL_TIM_Base_Start(htim: *mut TIM_HandleTypeDef) -> HAL_StatusTypeDef;
    /* Blocking mode: Polling */
    #[no_mangle]
    fn HAL_TIM_OC_Start(htim: *mut TIM_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    /* Non-Blocking mode: Interrupt */
    #[no_mangle]
    fn HAL_TIM_OC_Start_IT(htim: *mut TIM_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup TIM_Exported_Functions_Group8
  * @{
  */
/* Control functions  *********************************************************/
    #[no_mangle]
    fn HAL_TIM_OC_ConfigChannel(htim: *mut TIM_HandleTypeDef,
                                sConfig: *mut TIM_OC_InitTypeDef,
                                Channel: uint32_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_IC_ConfigChannel(htim: *mut TIM_HandleTypeDef,
                                sConfig: *mut TIM_IC_InitTypeDef,
                                Channel: uint32_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_ConfigClockSource(htim: *mut TIM_HandleTypeDef,
                                 sClockSourceConfig:
                                     *mut TIM_ClockConfigTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn TIM_DMADelayPulseCplt(hdma: *mut DMA_HandleTypeDef);
    #[no_mangle]
    fn TIM_DMAError(hdma: *mut DMA_HandleTypeDef);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    static timerHardware: [timerHardware_t; 0];
    #[no_mangle]
    static timerDefinitions: [timerDef_t; 0];
    #[no_mangle]
    fn timerClock(tim: *mut TIM_TypeDef) -> uint32_t;
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
  * @brief  TIM Output Compare Configuration Structure definition  
  */
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
/* * 
  * @brief  TIM Input Capture Configuration Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_IC_InitTypeDef {
    pub ICPolarity: uint32_t,
    pub ICSelection: uint32_t,
    pub ICPrescaler: uint32_t,
    pub ICFilter: uint32_t,
}
/* * 
  * @brief  Clock Configuration Handle Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_ClockConfigTypeDef {
    pub ClockSource: uint32_t,
    pub ClockPolarity: uint32_t,
    pub ClockPrescaler: uint32_t,
    pub ClockFilter: uint32_t,
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
pub type ioTag_t = uint8_t;
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
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
pub type timCCER_t = uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerCCHandlerRec_s {
    pub fn_0: Option<timerCCHandlerCallback>,
}
// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
pub type timerCCHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerCCHandlerRec_s, _: uint16_t) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerOvrHandlerRec_s {
    pub fn_0: Option<timerOvrHandlerCallback>,
    pub next: *mut timerOvrHandlerRec_s,
}
pub type timerOvrHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerOvrHandlerRec_s, _: uint16_t) -> ();
pub type timerCCHandlerRec_t = timerCCHandlerRec_s;
pub type timerOvrHandlerRec_t = timerOvrHandlerRec_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerDef_s {
    pub TIMx: *mut TIM_TypeDef,
    pub rcc: rccPeriphTag_t,
    pub inputIrq: uint8_t,
}
pub type timerDef_t = timerDef_s;
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
pub type timerHardware_t = timerHardware_s;
pub type channelType_t = libc::c_uint;
pub const TYPE_TIMER: channelType_t = 14;
pub const TYPE_SERIAL_RXTX: channelType_t = 13;
pub const TYPE_SERIAL_TX: channelType_t = 12;
pub const TYPE_SERIAL_RX: channelType_t = 11;
pub const TYPE_ADC: channelType_t = 10;
pub const TYPE_SOFTSERIAL_AUXTIMER: channelType_t = 9;
pub const TYPE_SOFTSERIAL_RXTX: channelType_t = 8;
pub const TYPE_SOFTSERIAL_TX: channelType_t = 7;
pub const TYPE_SOFTSERIAL_RX: channelType_t = 6;
pub const TYPE_PWMOUTPUT_SERVO: channelType_t = 5;
pub const TYPE_PWMOUTPUT_FAST: channelType_t = 4;
pub const TYPE_PWMOUTPUT_MOTOR: channelType_t = 3;
pub const TYPE_PPMINPUT: channelType_t = 2;
pub const TYPE_PWMINPUT: channelType_t = 1;
pub const TYPE_FREE: channelType_t = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHandle_t {
    pub Handle: TIM_HandleTypeDef,
}
pub type timerConfig_t = timerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerConfig_s {
    pub edgeCallback: [*mut timerCCHandlerRec_t; 4],
    pub overflowCallback: [*mut timerOvrHandlerRec_t; 4],
    pub overflowCallbackActive: *mut timerOvrHandlerRec_t,
    pub forcedOverflowTimerValue: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerInfo_t {
    pub priority: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerChannelInfo_t {
    pub type_0: channelType_t,
}
// TIMUP
// null-terminated linkded list of active overflow callbacks
// set BASEPRI_MAX, with global memory barrier, returns true
#[inline]
unsafe extern "C" fn __basepriSetMemRetVal(mut prio: uint8_t) -> uint8_t {
    __set_BASEPRI_MAX(prio as libc::c_int);
    return 1 as libc::c_int as uint8_t;
}
#[no_mangle]
pub static mut timerConfig: [timerConfig_t; 7] =
    [timerConfig_t{edgeCallback:
                       [0 as *const timerCCHandlerRec_t as
                            *mut timerCCHandlerRec_t; 4],
                   overflowCallback:
                       [0 as *const timerOvrHandlerRec_t as
                            *mut timerOvrHandlerRec_t; 4],
                   overflowCallbackActive:
                       0 as *const timerOvrHandlerRec_t as
                           *mut timerOvrHandlerRec_t,
                   forcedOverflowTimerValue: 0,}; 7];
#[no_mangle]
pub static mut timerChannelInfo: [timerChannelInfo_t; 13] =
    [timerChannelInfo_t{type_0: TYPE_FREE,}; 13];
#[no_mangle]
pub static mut timerInfo: [timerInfo_t; 7] = [timerInfo_t{priority: 0,}; 7];
#[no_mangle]
pub static mut timerHandle: [timerHandle_t; 7] =
    [timerHandle_t{Handle:
                       TIM_HandleTypeDef{Instance:
                                             0 as *const TIM_TypeDef as
                                                 *mut TIM_TypeDef,
                                         Init:
                                             TIM_Base_InitTypeDef{Prescaler:
                                                                      0,
                                                                  CounterMode:
                                                                      0,
                                                                  Period: 0,
                                                                  ClockDivision:
                                                                      0,
                                                                  RepetitionCounter:
                                                                      0,
                                                                  AutoReloadPreload:
                                                                      0,},
                                         Channel:
                                             HAL_TIM_ACTIVE_CHANNEL_CLEARED,
                                         hdma:
                                             [0 as *const DMA_HandleTypeDef as
                                                  *mut DMA_HandleTypeDef; 7],
                                         Lock: HAL_UNLOCKED,
                                         State: HAL_TIM_STATE_RESET,},}; 7];
// return index of timer in timer table. Lowest timer has index 0
unsafe extern "C" fn lookupTimerIndex(mut tim: *const TIM_TypeDef)
 -> uint8_t {
    // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
    // let gcc do the work, switch should be quite optimized
    match tim as libc::c_uint >> 10 as libc::c_int {
        1048640 => {
            return (((((1 as libc::c_int) << 1 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int)) -
                         ((((1 as libc::c_int) << 1 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 1 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 1 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 1 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int)) -
                              ((((1 as libc::c_int) << 1 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 1 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 1 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
            // make sure final index is out of range
        }
        1048576 => {
            return (((((1 as libc::c_int) << 2 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int)) -
                         ((((1 as libc::c_int) << 2 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 2 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 2 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 2 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int)) -
                              ((((1 as libc::c_int) << 2 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 2 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 2 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        1048577 => {
            return (((((1 as libc::c_int) << 3 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int)) -
                         ((((1 as libc::c_int) << 3 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 3 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 3 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 3 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int)) -
                              ((((1 as libc::c_int) << 3 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 3 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 3 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        1048578 => {
            return (((((1 as libc::c_int) << 4 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int)) -
                         ((((1 as libc::c_int) << 4 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 4 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 4 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 4 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int)) -
                              ((((1 as libc::c_int) << 4 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 4 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 4 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        1048641 => {
            return (((((1 as libc::c_int) << 8 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int)) -
                         ((((1 as libc::c_int) << 8 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 8 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 8 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 8 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int)) -
                              ((((1 as libc::c_int) << 8 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 8 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 8 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        1048656 => {
            return (((((1 as libc::c_int) << 9 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int)) -
                         ((((1 as libc::c_int) << 9 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 9 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 9 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 9 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 4 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 9 as libc::c_int)) -
                              ((((1 as libc::c_int) << 9 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 9 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 9 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             4 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             9 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        _ => { return !(1 as libc::c_int) as uint8_t }
    };
}
// Initialized in run_static_initializers
#[no_mangle]
pub static mut usedTimers: [*mut TIM_TypeDef; 6] =
    [0 as *const TIM_TypeDef as *mut TIM_TypeDef; 6];
// Map timer index to timer number (Straight copy of usedTimers array)
#[no_mangle]
pub static mut timerNumbers: [int8_t; 6] =
    [1 as libc::c_int as int8_t, 2 as libc::c_int as int8_t,
     3 as libc::c_int as int8_t, 4 as libc::c_int as int8_t,
     8 as libc::c_int as int8_t, 9 as libc::c_int as int8_t];
#[no_mangle]
pub unsafe extern "C" fn timerGetTIMNumber(mut tim: *const TIM_TypeDef)
 -> int8_t {
    let mut index: uint8_t = lookupTimerIndex(tim);
    if (index as libc::c_int) <
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return timerNumbers[index as usize]
    } else { return 0 as libc::c_int as int8_t };
}
#[inline]
unsafe extern "C" fn lookupChannelIndex(channel: uint16_t) -> uint8_t {
    return (channel as libc::c_int >> 2 as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerLookupChannelIndex(channel: uint16_t)
 -> uint8_t {
    return lookupChannelIndex(channel);
}
// TODO - just for migration
#[no_mangle]
pub unsafe extern "C" fn timerRCC(mut tim: *mut TIM_TypeDef)
 -> rccPeriphTag_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 14 as libc::c_int {
        if (*timerDefinitions.as_ptr().offset(i as isize)).TIMx == tim {
            return (*timerDefinitions.as_ptr().offset(i as isize)).rcc
        }
        i += 1
    }
    return 0 as libc::c_int as rccPeriphTag_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerInputIrq(mut tim: *mut TIM_TypeDef) -> uint8_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 14 as libc::c_int {
        if (*timerDefinitions.as_ptr().offset(i as isize)).TIMx == tim {
            return (*timerDefinitions.as_ptr().offset(i as isize)).inputIrq
        }
        i += 1
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerNVICConfigure(mut irq: uint8_t) {
    HAL_NVIC_SetPriority(irq as IRQn_Type,
                         ((((1 as libc::c_int) <<
                                (4 as libc::c_int as
                                     libc::c_uint).wrapping_sub((7 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_sub(0x5
                                                                                                    as
                                                                                                    libc::c_uint))
                                |
                                1 as libc::c_int &
                                    0xf as libc::c_int >>
                                        (7 as libc::c_int as
                                             libc::c_uint).wrapping_sub(0x5 as
                                                                            libc::c_uint))
                               << 4 as libc::c_int & 0xf0 as libc::c_int) >>
                              (4 as libc::c_int as
                                   libc::c_uint).wrapping_sub((7 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint).wrapping_sub(0x5
                                                                                                  as
                                                                                                  libc::c_uint))
                              >> 4 as libc::c_int) as uint32_t,
                         ((((1 as libc::c_int) <<
                                (4 as libc::c_int as
                                     libc::c_uint).wrapping_sub((7 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_sub(0x5
                                                                                                    as
                                                                                                    libc::c_uint))
                                |
                                1 as libc::c_int &
                                    0xf as libc::c_int >>
                                        (7 as libc::c_int as
                                             libc::c_uint).wrapping_sub(0x5 as
                                                                            libc::c_uint))
                               << 4 as libc::c_int & 0xf0 as libc::c_int &
                               0xf as libc::c_int >>
                                   (7 as libc::c_int as
                                        libc::c_uint).wrapping_sub(0x5 as
                                                                       libc::c_uint))
                              >> 4 as libc::c_int) as uint32_t);
    HAL_NVIC_EnableIRQ(irq as IRQn_Type);
}
#[no_mangle]
pub unsafe extern "C" fn timerFindTimerHandle(mut tim: *mut TIM_TypeDef)
 -> *mut TIM_HandleTypeDef {
    let mut timerIndex: uint8_t = lookupTimerIndex(tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return 0 as *mut TIM_HandleTypeDef
    }
    return &mut (*timerHandle.as_mut_ptr().offset(timerIndex as
                                                      isize)).Handle;
}
#[no_mangle]
pub unsafe extern "C" fn configTimeBase(mut tim: *mut TIM_TypeDef,
                                        mut period: uint16_t,
                                        mut hz: uint32_t) {
    let mut timerIndex: uint8_t = lookupTimerIndex(tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    if timerHandle[timerIndex as usize].Handle.Instance == tim {
        // already configured
        return
    } // AKA TIMx_ARR
    timerHandle[timerIndex as usize].Handle.Instance = tim;
    timerHandle[timerIndex as usize].Handle.Init.Period =
        (period as libc::c_int - 1 as libc::c_int & 0xffff as libc::c_int) as
            uint32_t;
    timerHandle[timerIndex as usize].Handle.Init.Prescaler =
        timerClock(tim).wrapping_div(hz).wrapping_sub(1 as libc::c_int as
                                                          libc::c_uint);
    timerHandle[timerIndex as usize].Handle.Init.ClockDivision =
        0 as libc::c_uint;
    timerHandle[timerIndex as usize].Handle.Init.CounterMode =
        0 as libc::c_uint;
    timerHandle[timerIndex as usize].Handle.Init.RepetitionCounter =
        0 as libc::c_int as uint32_t;
    HAL_TIM_Base_Init(&mut (*timerHandle.as_mut_ptr().offset(timerIndex as
                                                                 isize)).Handle);
    if tim ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0xc00 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x4000
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        let mut sClockSourceConfig: TIM_ClockConfigTypeDef =
            TIM_ClockConfigTypeDef{ClockSource: 0,
                                   ClockPolarity: 0,
                                   ClockPrescaler: 0,
                                   ClockFilter: 0,};
        memset(&mut sClockSourceConfig as *mut TIM_ClockConfigTypeDef as
                   *mut libc::c_void, 0 as libc::c_int,
               ::core::mem::size_of::<TIM_ClockConfigTypeDef>() as
                   libc::c_ulong);
        sClockSourceConfig.ClockSource =
            (0x1 as libc::c_uint) << 12 as libc::c_uint;
        if HAL_TIM_ConfigClockSource(&mut (*timerHandle.as_mut_ptr().offset(timerIndex
                                                                                as
                                                                                isize)).Handle,
                                     &mut sClockSourceConfig) as libc::c_uint
               != HAL_OK as libc::c_int as libc::c_uint {
            return
        }
    }
    if tim ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0xc00 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           tim ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        let mut sMasterConfig: TIM_MasterConfigTypeDef =
            TIM_MasterConfigTypeDef{MasterOutputTrigger: 0,
                                    MasterOutputTrigger2: 0,
                                    MasterSlaveMode: 0,};
        memset(&mut sMasterConfig as *mut TIM_MasterConfigTypeDef as
                   *mut libc::c_void, 0 as libc::c_int,
               ::core::mem::size_of::<TIM_MasterConfigTypeDef>() as
                   libc::c_ulong);
        sMasterConfig.MasterSlaveMode = 0 as libc::c_uint;
        if HAL_TIMEx_MasterConfigSynchronization(&mut (*timerHandle.as_mut_ptr().offset(timerIndex
                                                                                            as
                                                                                            isize)).Handle,
                                                 &mut sMasterConfig) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            return
        }
    };
}
// old interface for PWM inputs. It should be replaced
#[no_mangle]
pub unsafe extern "C" fn timerConfigure(mut timerHardwarePtr:
                                            *const timerHardware_t,
                                        mut period: uint16_t,
                                        mut hz: uint32_t) {
    let mut timerIndex: uint8_t = lookupTimerIndex((*timerHardwarePtr).tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    configTimeBase((*timerHardwarePtr).tim, period, hz);
    HAL_TIM_Base_Start(&mut (*timerHandle.as_mut_ptr().offset(timerIndex as
                                                                  isize)).Handle);
    let mut irq: uint8_t = timerInputIrq((*timerHardwarePtr).tim);
    timerNVICConfigure(irq);
    // HACK - enable second IRQ on timers that need it
    match irq as libc::c_int {
        27 => {
            timerNVICConfigure(TIM1_UP_TIM10_IRQn as libc::c_int as uint8_t);
        }
        46 => {
            timerNVICConfigure(TIM8_UP_TIM13_IRQn as libc::c_int as uint8_t);
        }
        _ => { }
    };
}
// allocate and configure timer channel. Timer priority is set to highest priority of its channels
#[no_mangle]
pub unsafe extern "C" fn timerChInit(mut timHw: *const timerHardware_t,
                                     mut type_0: channelType_t,
                                     mut irqPriority: libc::c_int,
                                     mut irq: uint8_t) {
    let mut timerIndex: uint8_t = lookupTimerIndex((*timHw).tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    let mut channel: libc::c_uint =
        timHw.wrapping_offset_from(timerHardware.as_ptr()) as libc::c_long as
            libc::c_uint;
    if channel >= 13 as libc::c_int as libc::c_uint { return }
    timerChannelInfo[channel as usize].type_0 = type_0;
    let mut timer: libc::c_uint =
        lookupTimerIndex((*timHw).tim) as libc::c_uint;
    if timer >=
           ((((1 as libc::c_int) << 1 as libc::c_int |
                  (1 as libc::c_int) << 2 as libc::c_int |
                  (1 as libc::c_int) << 3 as libc::c_int |
                  (1 as libc::c_int) << 4 as libc::c_int |
                  (1 as libc::c_int) << 8 as libc::c_int |
                  (1 as libc::c_int) << 9 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      1 as libc::c_int & 0x77777777 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      2 as libc::c_int & 0x33333333 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      3 as libc::c_int & 0x11111111 as libc::c_int) +
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           1 as libc::c_int & 0x77777777 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           2 as libc::c_int & 0x33333333 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           3 as libc::c_int & 0x11111111 as libc::c_int) >>
                      4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                255 as libc::c_int) as libc::c_uint {
        return
    }
    if irqPriority < timerInfo[timer as usize].priority as libc::c_int {
        // it would be better to set priority in the end, but current startup sequence is not ready
        configTimeBase(usedTimers[timer as usize],
                       0 as libc::c_int as uint16_t,
                       1 as libc::c_int as uint32_t);
        HAL_TIM_Base_Start(&mut (*timerHandle.as_mut_ptr().offset(timerIndex
                                                                      as
                                                                      isize)).Handle);
        HAL_NVIC_SetPriority(irq as IRQn_Type,
                             (irqPriority >>
                                  (4 as libc::c_int as
                                       libc::c_uint).wrapping_sub((7 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint).wrapping_sub(0x5
                                                                                                      as
                                                                                                      libc::c_uint))
                                  >> 4 as libc::c_int) as uint32_t,
                             ((irqPriority &
                                   0xf as libc::c_int >>
                                       (7 as libc::c_int as
                                            libc::c_uint).wrapping_sub(0x5 as
                                                                           libc::c_uint))
                                  >> 4 as libc::c_int) as uint32_t);
        HAL_NVIC_EnableIRQ(irq as IRQn_Type);
        timerInfo[timer as usize].priority = irqPriority as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn timerChCCHandlerInit(mut self_0:
                                                  *mut timerCCHandlerRec_t,
                                              mut fn_0:
                                                  Option<timerCCHandlerCallback>) {
    (*self_0).fn_0 = fn_0;
}
#[no_mangle]
pub unsafe extern "C" fn timerChOvrHandlerInit(mut self_0:
                                                   *mut timerOvrHandlerRec_t,
                                               mut fn_0:
                                                   Option<timerOvrHandlerCallback>) {
    (*self_0).fn_0 = fn_0;
    (*self_0).next = 0 as *mut timerOvrHandlerRec_s;
}
// update overflow callback list
// some synchronization mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
unsafe extern "C" fn timerChConfig_UpdateOverflow(mut cfg: *mut timerConfig_t,
                                                  mut tim: *mut TIM_TypeDef) {
    let mut timerIndex: uint8_t = lookupTimerIndex(tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    let mut chain: *mut *mut timerOvrHandlerRec_t =
        &mut (*cfg).overflowCallbackActive;
    let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
    let mut __ToDo: uint8_t =
        __basepriSetMemRetVal((((1 as libc::c_int) <<
                                    (4 as libc::c_int as
                                         libc::c_uint).wrapping_sub((7 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_sub(0x5
                                                                                                        as
                                                                                                        libc::c_uint))
                                    |
                                    1 as libc::c_int &
                                        0xf as libc::c_int >>
                                            (7 as libc::c_int as
                                                 libc::c_uint).wrapping_sub(0x5
                                                                                as
                                                                                libc::c_uint))
                                   << 4 as libc::c_int & 0xf0 as libc::c_int)
                                  as uint8_t);
    while __ToDo != 0 {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < 4 as libc::c_int {
            if !(*cfg).overflowCallback[i as usize].is_null() {
                *chain = (*cfg).overflowCallback[i as usize];
                chain =
                    &mut (**(*cfg).overflowCallback.as_mut_ptr().offset(i as
                                                                            isize)).next
            }
            i += 1
        }
        *chain = 0 as *mut timerOvrHandlerRec_t;
        __ToDo = 0 as libc::c_int as uint8_t
    }
    // enable or disable IRQ
    if !(*cfg).overflowCallbackActive.is_null() {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    };
}
// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
#[no_mangle]
pub unsafe extern "C" fn timerChConfigCallbacks(mut timHw:
                                                    *const timerHardware_t,
                                                mut edgeCallback:
                                                    *mut timerCCHandlerRec_t,
                                                mut overflowCallback:
                                                    *mut timerOvrHandlerRec_t) {
    let mut timerIndex: uint8_t = lookupTimerIndex((*timHw).tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    let mut channelIndex: uint8_t =
        lookupChannelIndex((*timHw).channel as uint16_t);
    if edgeCallback.is_null() {
        // disable irq before changing callback to NULL
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x1 as libc::c_uint) <<
                                                1 as libc::c_uint) <<
                                               (*timHw).channel as libc::c_int
                                                   / 4 as libc::c_int)) as
                                        uint32_t as uint32_t)
    }
    // setup callback info
    timerConfig[timerIndex as usize].edgeCallback[channelIndex as usize] =
        edgeCallback;
    timerConfig[timerIndex as usize].overflowCallback[channelIndex as usize] =
        overflowCallback;
    // enable channel IRQ
    if !edgeCallback.is_null() {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              1 as libc::c_uint) <<
                                             (*timHw).channel as libc::c_int /
                                                 4 as libc::c_int) as uint32_t
                                        as uint32_t)
    }
    timerChConfig_UpdateOverflow(&mut *timerConfig.as_mut_ptr().offset(timerIndex
                                                                           as
                                                                           isize),
                                 (*timHw).tim);
}
// configure callbacks for pair of channels (1+2 or 3+4).
// Hi(2,4) and Lo(1,3) callbacks are specified, it is not important which timHw channel is used.
// This is intended for dual capture mode (each channel handles one transition)
#[no_mangle]
pub unsafe extern "C" fn timerChConfigCallbacksDual(mut timHw:
                                                        *const timerHardware_t,
                                                    mut edgeCallbackLo:
                                                        *mut timerCCHandlerRec_t,
                                                    mut edgeCallbackHi:
                                                        *mut timerCCHandlerRec_t,
                                                    mut overflowCallback:
                                                        *mut timerOvrHandlerRec_t) {
    let mut timerIndex: uint8_t =
        lookupTimerIndex((*timHw).tim); // lower channel
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    } // upper channel
    let mut chLo: uint16_t =
        ((*timHw).channel as libc::c_uint & !(0x4 as libc::c_uint)) as
            uint16_t; // get index of lower channel
    let mut chHi: uint16_t =
        ((*timHw).channel as libc::c_uint | 0x4 as libc::c_uint) as uint16_t;
    let mut channelIndex: uint8_t = lookupChannelIndex(chLo);
    if edgeCallbackLo.is_null() {
        // disable irq before changing setting callback to NULL
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x1 as libc::c_uint) <<
                                                1 as libc::c_uint) <<
                                               chLo as libc::c_int /
                                                   4 as libc::c_int)) as
                                        uint32_t as uint32_t)
    }
    if edgeCallbackHi.is_null() {
        // disable irq before changing setting callback to NULL
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x1 as libc::c_uint) <<
                                                1 as libc::c_uint) <<
                                               chHi as libc::c_int /
                                                   4 as libc::c_int)) as
                                        uint32_t as uint32_t)
    }
    // setup callback info
    timerConfig[timerIndex as usize].edgeCallback[channelIndex as usize] =
        edgeCallbackLo;
    timerConfig[timerIndex as
                    usize].edgeCallback[(channelIndex as libc::c_int +
                                             1 as libc::c_int) as usize] =
        edgeCallbackHi;
    timerConfig[timerIndex as usize].overflowCallback[channelIndex as usize] =
        overflowCallback;
    timerConfig[timerIndex as
                    usize].overflowCallback[(channelIndex as libc::c_int +
                                                 1 as libc::c_int) as usize] =
        0 as *mut timerOvrHandlerRec_t;
    // enable channel IRQs
    if !edgeCallbackLo.is_null() {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).SR
                                        as *mut uint32_t,
                                    !(((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint) <<
                                          chLo as libc::c_int /
                                              4 as libc::c_int));
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              1 as libc::c_uint) <<
                                             chLo as libc::c_int /
                                                 4 as libc::c_int) as uint32_t
                                        as uint32_t)
    }
    if !edgeCallbackHi.is_null() {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).SR
                                        as *mut uint32_t,
                                    !(((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint) <<
                                          chHi as libc::c_int /
                                              4 as libc::c_int));
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              1 as libc::c_uint) <<
                                             chHi as libc::c_int /
                                                 4 as libc::c_int) as uint32_t
                                        as uint32_t)
    }
    timerChConfig_UpdateOverflow(&mut *timerConfig.as_mut_ptr().offset(timerIndex
                                                                           as
                                                                           isize),
                                 (*timHw).tim);
}
// enable/disable IRQ for low channel in dual configuration
//void timerChITConfigDualLo(const timerHardware_t *timHw, FunctionalState newState) {
//    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel&~TIM_Channel_2), newState);
//}
// enable/disable IRQ for low channel in dual configuration
#[no_mangle]
pub unsafe extern "C" fn timerChITConfigDualLo(mut timHw:
                                                   *const timerHardware_t,
                                               mut newState:
                                                   FunctionalState) {
    let mut timerIndex: uint8_t = lookupTimerIndex((*timHw).tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    if newState as u64 != 0 {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              1 as libc::c_uint) <<
                                             ((*timHw).channel as libc::c_uint
                                                  &
                                                  !(0x4 as
                                                        libc::c_uint)).wrapping_div(4
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        libc::c_uint))
                                        as uint32_t as uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x1 as libc::c_uint) <<
                                                1 as libc::c_uint) <<
                                               ((*timHw).channel as
                                                    libc::c_uint &
                                                    !(0x4 as
                                                          libc::c_uint)).wrapping_div(4
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)))
                                        as uint32_t as uint32_t)
    };
}
// // enable or disable IRQ
//void timerChITConfig(const timerHardware_t *timHw, FunctionalState newState)
//{
//    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), newState);
//}
// enable or disable IRQ
#[no_mangle]
pub unsafe extern "C" fn timerChITConfig(mut timHw: *const timerHardware_t,
                                         mut newState: FunctionalState) {
    let mut timerIndex: uint8_t = lookupTimerIndex((*timHw).tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    if newState as u64 != 0 {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              1 as libc::c_uint) <<
                                             (*timHw).channel as libc::c_int /
                                                 4 as libc::c_int) as uint32_t
                                        as uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                           usize].Handle.Instance).DIER
                                        as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*timerHandle[timerIndex
                                                                                               as
                                                                                               usize].Handle.Instance).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x1 as libc::c_uint) <<
                                                1 as libc::c_uint) <<
                                               (*timHw).channel as libc::c_int
                                                   / 4 as libc::c_int)) as
                                        uint32_t as uint32_t)
    };
}
// clear Compare/Capture flag for channel
//void timerChClearCCFlag(const timerHardware_t *timHw)
//{
//    TIM_ClearFlag(timHw->tim, TIM_IT_CCx(timHw->channel));
//}
// clear Compare/Capture flag for channel
#[no_mangle]
pub unsafe extern "C" fn timerChClearCCFlag(mut timHw:
                                                *const timerHardware_t) {
    let mut timerIndex: uint8_t = lookupTimerIndex((*timHw).tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 4 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 9 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 4 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 9 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 4 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 9 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    ::core::ptr::write_volatile(&mut (*timerHandle[timerIndex as
                                                       usize].Handle.Instance).SR
                                    as *mut uint32_t,
                                !(((0x1 as libc::c_uint) << 1 as libc::c_uint)
                                      <<
                                      (*timHw).channel as libc::c_int /
                                          4 as libc::c_int));
}
// configure timer channel GPIO mode
#[no_mangle]
pub unsafe extern "C" fn timerChConfigGPIO(mut timHw: *const timerHardware_t,
                                           mut mode: ioConfig_t) {
    IOInit(IOGetByTag((*timHw).tag), OWNER_TIMER,
           0 as libc::c_int as uint8_t);
    IOConfigGPIO(IOGetByTag((*timHw).tag), mode);
}
// calculate input filter constant
// TODO - we should probably setup DTS to higher value to allow reasonable input filtering
//   - notice that prescaler[0] does use DTS for sampling - the sequence won't be monotonous anymore
unsafe extern "C" fn getFilter(mut ticks: libc::c_uint) -> libc::c_uint {
    static mut ftab: [libc::c_uint; 16] =
        [(1 as libc::c_int * 1 as libc::c_int) as libc::c_uint,
         (1 as libc::c_int * 2 as libc::c_int) as libc::c_uint,
         (1 as libc::c_int * 4 as libc::c_int) as libc::c_uint,
         (1 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (2 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (2 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (4 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (4 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (8 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (8 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (16 as libc::c_int * 5 as libc::c_int) as libc::c_uint,
         (16 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (16 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (32 as libc::c_int * 5 as libc::c_int) as libc::c_uint,
         (32 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (32 as libc::c_int * 8 as libc::c_int) as libc::c_uint];
    let mut i: libc::c_uint = 1 as libc::c_int as libc::c_uint;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[libc::c_uint; 16]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<libc::c_uint>()
                                                   as libc::c_ulong) {
        if ftab[i as usize] > ticks {
            return i.wrapping_sub(1 as libc::c_int as libc::c_uint)
        }
        i = i.wrapping_add(1)
    }
    return 0xf as libc::c_int as libc::c_uint;
}
// This interface should be replaced.
// Configure input captupre
#[no_mangle]
pub unsafe extern "C" fn timerChConfigIC(mut timHw: *const timerHardware_t,
                                         mut polarityRising: bool,
                                         mut inputFilterTicks: libc::c_uint) {
    let mut timer: libc::c_uint =
        lookupTimerIndex((*timHw).tim) as libc::c_uint;
    if timer >=
           ((((1 as libc::c_int) << 1 as libc::c_int |
                  (1 as libc::c_int) << 2 as libc::c_int |
                  (1 as libc::c_int) << 3 as libc::c_int |
                  (1 as libc::c_int) << 4 as libc::c_int |
                  (1 as libc::c_int) << 8 as libc::c_int |
                  (1 as libc::c_int) << 9 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      1 as libc::c_int & 0x77777777 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      2 as libc::c_int & 0x33333333 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      3 as libc::c_int & 0x11111111 as libc::c_int) +
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           1 as libc::c_int & 0x77777777 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           2 as libc::c_int & 0x33333333 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           3 as libc::c_int & 0x11111111 as libc::c_int) >>
                      4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                255 as libc::c_int) as libc::c_uint {
        return
    }
    let mut TIM_ICInitStructure: TIM_IC_InitTypeDef =
        TIM_IC_InitTypeDef{ICPolarity: 0,
                           ICSelection: 0,
                           ICPrescaler: 0,
                           ICFilter: 0,};
    TIM_ICInitStructure.ICPolarity =
        if polarityRising as libc::c_int != 0 {
            0 as libc::c_uint
        } else { ((0x1 as libc::c_uint)) << 1 as libc::c_uint };
    TIM_ICInitStructure.ICSelection =
        (0x1 as libc::c_uint) << 0 as libc::c_uint;
    TIM_ICInitStructure.ICPrescaler = 0 as libc::c_uint;
    TIM_ICInitStructure.ICFilter = getFilter(inputFilterTicks);
    HAL_TIM_IC_ConfigChannel(&mut (*timerHandle.as_mut_ptr().offset(timer as
                                                                        isize)).Handle,
                             &mut TIM_ICInitStructure,
                             (*timHw).channel as uint32_t);
}
// configure dual channel input channel for capture
// polarity is for Low channel (capture order is always Lo - Hi)
#[no_mangle]
pub unsafe extern "C" fn timerChConfigICDual(mut timHw:
                                                 *const timerHardware_t,
                                             mut polarityRising: bool,
                                             mut inputFilterTicks:
                                                 libc::c_uint) {
    let mut timer: libc::c_uint =
        lookupTimerIndex((*timHw).tim) as libc::c_uint;
    if timer >=
           ((((1 as libc::c_int) << 1 as libc::c_int |
                  (1 as libc::c_int) << 2 as libc::c_int |
                  (1 as libc::c_int) << 3 as libc::c_int |
                  (1 as libc::c_int) << 4 as libc::c_int |
                  (1 as libc::c_int) << 8 as libc::c_int |
                  (1 as libc::c_int) << 9 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      1 as libc::c_int & 0x77777777 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      2 as libc::c_int & 0x33333333 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      3 as libc::c_int & 0x11111111 as libc::c_int) +
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           1 as libc::c_int & 0x77777777 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           2 as libc::c_int & 0x33333333 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           3 as libc::c_int & 0x11111111 as libc::c_int) >>
                      4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                255 as libc::c_int) as libc::c_uint {
        return
    }
    let mut TIM_ICInitStructure: TIM_IC_InitTypeDef =
        TIM_IC_InitTypeDef{ICPolarity: 0,
                           ICSelection: 0,
                           ICPrescaler: 0,
                           ICFilter: 0,};
    let mut directRising: bool =
        if (*timHw).channel as libc::c_uint & 0x4 as libc::c_uint != 0 {
            !polarityRising as libc::c_int
        } else { polarityRising as libc::c_int } != 0;
    // configure direct channel
    TIM_ICInitStructure.ICPolarity =
        if directRising as libc::c_int != 0 {
            0 as libc::c_uint
        } else { ((0x1 as libc::c_uint)) << 1 as libc::c_uint };
    TIM_ICInitStructure.ICSelection =
        (0x1 as libc::c_uint) << 0 as libc::c_uint;
    TIM_ICInitStructure.ICPrescaler = 0 as libc::c_uint;
    TIM_ICInitStructure.ICFilter = getFilter(inputFilterTicks);
    HAL_TIM_IC_ConfigChannel(&mut (*timerHandle.as_mut_ptr().offset(timer as
                                                                        isize)).Handle,
                             &mut TIM_ICInitStructure,
                             (*timHw).channel as uint32_t);
    // configure indirect channel
    TIM_ICInitStructure.ICPolarity =
        if directRising as libc::c_int != 0 {
            ((0x1 as libc::c_uint)) << 1 as libc::c_uint
        } else { 0 as libc::c_uint };
    TIM_ICInitStructure.ICSelection =
        (0x2 as libc::c_uint) << 0 as libc::c_uint;
    HAL_TIM_IC_ConfigChannel(&mut (*timerHandle.as_mut_ptr().offset(timer as
                                                                        isize)).Handle,
                             &mut TIM_ICInitStructure,
                             (*timHw).channel as libc::c_uint ^
                                 0x4 as libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn timerChICPolarity(mut timHw: *const timerHardware_t,
                                           mut polarityRising: bool) {
    let mut tmpccer: timCCER_t = (*(*timHw).tim).CCER;
    tmpccer &=
        !(((0x1 as libc::c_uint) << 1 as libc::c_uint) <<
              (*timHw).channel as libc::c_int);
    tmpccer |=
        if polarityRising as libc::c_int != 0 {
            ((0 as libc::c_uint)) << (*timHw).channel as libc::c_int
        } else {
            (((0x1 as libc::c_uint) << 1 as libc::c_uint)) <<
                (*timHw).channel as libc::c_int
        };
    ::core::ptr::write_volatile(&mut (*(*timHw).tim).CCER as *mut uint32_t,
                                tmpccer);
}
#[no_mangle]
pub unsafe extern "C" fn timerChCCRHi(mut timHw: *const timerHardware_t)
 -> *mut timCCR_t {
    return (&mut (*(*timHw).tim).CCR1 as *mut uint32_t as
                *mut libc::c_char).offset(((*timHw).channel as libc::c_uint |
                                               0x4 as libc::c_uint) as isize)
               as *mut timCCR_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerChCCRLo(mut timHw: *const timerHardware_t)
 -> *mut timCCR_t {
    return (&mut (*(*timHw).tim).CCR1 as *mut uint32_t as
                *mut libc::c_char).offset(((*timHw).channel as libc::c_uint &
                                               !(0x4 as libc::c_uint)) as
                                              isize) as *mut timCCR_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerChCCR(mut timHw: *const timerHardware_t)
 -> *mut timCCR_t {
    return (&mut (*(*timHw).tim).CCR1 as *mut uint32_t as
                *mut libc::c_char).offset((*timHw).channel as libc::c_int as
                                              isize) as *mut timCCR_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerChConfigOC(mut timHw: *const timerHardware_t,
                                         mut outEnable: bool,
                                         mut stateHigh: bool) {
    let mut timer: libc::c_uint =
        lookupTimerIndex((*timHw).tim) as libc::c_uint;
    if timer >=
           ((((1 as libc::c_int) << 1 as libc::c_int |
                  (1 as libc::c_int) << 2 as libc::c_int |
                  (1 as libc::c_int) << 3 as libc::c_int |
                  (1 as libc::c_int) << 4 as libc::c_int |
                  (1 as libc::c_int) << 8 as libc::c_int |
                  (1 as libc::c_int) << 9 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      1 as libc::c_int & 0x77777777 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      2 as libc::c_int & 0x33333333 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) >>
                      3 as libc::c_int & 0x11111111 as libc::c_int) +
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 4 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 9 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           1 as libc::c_int & 0x77777777 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           2 as libc::c_int & 0x33333333 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 4 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 9 as libc::c_int) >>
                           3 as libc::c_int & 0x11111111 as libc::c_int) >>
                      4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                255 as libc::c_int) as libc::c_uint {
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
    TIM_OCInitStructure.OCMode = (0x2 as libc::c_uint) << 4 as libc::c_uint;
    TIM_OCInitStructure.Pulse = 0 as libc::c_int as uint32_t;
    TIM_OCInitStructure.OCPolarity =
        if stateHigh as libc::c_int != 0 {
            0 as libc::c_uint
        } else { ((0x1 as libc::c_uint)) << 1 as libc::c_uint };
    TIM_OCInitStructure.OCNPolarity = 0 as libc::c_uint;
    TIM_OCInitStructure.OCIdleState = 0 as libc::c_uint;
    TIM_OCInitStructure.OCNIdleState = 0 as libc::c_uint;
    HAL_TIM_OC_ConfigChannel(&mut (*timerHandle.as_mut_ptr().offset(timer as
                                                                        isize)).Handle,
                             &mut TIM_OCInitStructure,
                             (*timHw).channel as uint32_t);
    if outEnable {
        TIM_OCInitStructure.OCMode =
            (0x2 as libc::c_uint) << 4 as libc::c_uint;
        HAL_TIM_OC_ConfigChannel(&mut (*timerHandle.as_mut_ptr().offset(timer
                                                                            as
                                                                            isize)).Handle,
                                 &mut TIM_OCInitStructure,
                                 (*timHw).channel as uint32_t);
        HAL_TIM_OC_Start(&mut (*timerHandle.as_mut_ptr().offset(timer as
                                                                    isize)).Handle,
                         (*timHw).channel as uint32_t);
    } else {
        TIM_OCInitStructure.OCMode = 0 as libc::c_uint;
        HAL_TIM_OC_ConfigChannel(&mut (*timerHandle.as_mut_ptr().offset(timer
                                                                            as
                                                                            isize)).Handle,
                                 &mut TIM_OCInitStructure,
                                 (*timHw).channel as uint32_t);
        HAL_TIM_OC_Start_IT(&mut (*timerHandle.as_mut_ptr().offset(timer as
                                                                       isize)).Handle,
                            (*timHw).channel as uint32_t);
    };
}
unsafe extern "C" fn timCCxHandler(mut tim: *mut TIM_TypeDef,
                                   mut timerConfig_0: *mut timerConfig_t) {
    let mut capture: uint16_t = 0;
    let mut tim_status: libc::c_uint = 0;
    tim_status = (*tim).SR & (*tim).DIER;
    while tim_status != 0 {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        let mut bit: libc::c_uint =
            tim_status.leading_zeros() as i32 as libc::c_uint;
        let mut mask: libc::c_uint = !(0x80000000 as libc::c_uint >> bit);
        ::core::ptr::write_volatile(&mut (*tim).SR as *mut uint32_t, mask);
        tim_status &= mask;
        match bit {
            31 => {
                if (*timerConfig_0).forcedOverflowTimerValue !=
                       0 as libc::c_int as libc::c_uint {
                    capture =
                        (*timerConfig_0).forcedOverflowTimerValue.wrapping_sub(1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint)
                            as uint16_t;
                    (*timerConfig_0).forcedOverflowTimerValue =
                        0 as libc::c_int as uint32_t
                } else { capture = (*tim).ARR as uint16_t }
                let mut cb: *mut timerOvrHandlerRec_t =
                    (*timerConfig_0).overflowCallbackActive;
                while !cb.is_null() {
                    (*cb).fn_0.expect("non-null function pointer")(cb,
                                                                   capture);
                    cb = (*cb).next
                }
            }
            30 => {
                (*(*timerConfig_0).edgeCallback[0 as libc::c_int as
                                                    usize]).fn_0.expect("non-null function pointer")((*timerConfig_0).edgeCallback[0
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       usize],
                                                                                                     (*tim).CCR1
                                                                                                         as
                                                                                                         uint16_t);
            }
            29 => {
                (*(*timerConfig_0).edgeCallback[1 as libc::c_int as
                                                    usize]).fn_0.expect("non-null function pointer")((*timerConfig_0).edgeCallback[1
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       usize],
                                                                                                     (*tim).CCR2
                                                                                                         as
                                                                                                         uint16_t);
            }
            28 => {
                (*(*timerConfig_0).edgeCallback[2 as libc::c_int as
                                                    usize]).fn_0.expect("non-null function pointer")((*timerConfig_0).edgeCallback[2
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       usize],
                                                                                                     (*tim).CCR3
                                                                                                         as
                                                                                                         uint16_t);
            }
            27 => {
                (*(*timerConfig_0).edgeCallback[3 as libc::c_int as
                                                    usize]).fn_0.expect("non-null function pointer")((*timerConfig_0).edgeCallback[3
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       usize],
                                                                                                     (*tim).CCR4
                                                                                                         as
                                                                                                         uint16_t);
            }
            _ => { }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn TIM1_CC_IRQHandler() {
    timCCxHandler((0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                1 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        4 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        9 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM1_UP_TIM10_IRQHandler() {
    timCCxHandler((0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                1 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        4 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        9 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
// timer10 is not used
#[no_mangle]
pub unsafe extern "C" fn TIM2_IRQHandler() {
    timCCxHandler((0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                2 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        4 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        9 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     2 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     2 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     2 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     2 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          2 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          2 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          2 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM3_IRQHandler() {
    timCCxHandler((0x40000000 as
                       libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                      *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                3 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        4 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        9 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     3 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     3 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     3 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     3 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          3 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          3 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          3 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM4_IRQHandler() {
    timCCxHandler((0x40000000 as
                       libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                      *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                4 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        4 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        9 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     4 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     4 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     4 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     4 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          4 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          4 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          4 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM8_CC_IRQHandler() {
    timCCxHandler((0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x400
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                8 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        4 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        9 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM8_UP_TIM13_IRQHandler() {
    timCCxHandler((0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x400
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                8 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        4 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        9 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
// timer13 is not used
#[no_mangle]
pub unsafe extern "C" fn TIM1_BRK_TIM9_IRQHandler() {
    timCCxHandler((0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x4000
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                9 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        4 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        9 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     9 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     9 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     9 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     9 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             9
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          9 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          9 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          9 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  4
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  9
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn timerInit() {
    memset(timerConfig.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[timerConfig_t; 7]>() as libc::c_ulong);
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        0 as libc::c_uint);
    let mut tmpreg_0: uint32_t = 0;
    let ref mut fresh1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        0 as libc::c_uint);
    let mut tmpreg_1: uint32_t = 0;
    let ref mut fresh2 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_1 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        1 as libc::c_uint);
    let mut tmpreg_2: uint32_t = 0;
    let ref mut fresh3 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_2 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        2 as libc::c_uint);
    let mut tmpreg_3: uint32_t = 0;
    let ref mut fresh4 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_3 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        1 as libc::c_uint);
    let mut tmpreg_4: uint32_t = 0;
    let ref mut fresh5 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         16 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_4 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        16 as libc::c_uint);
    /* enable the timer peripherals */
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 13 as libc::c_int {
        RCC_ClockCmd(timerRCC((*timerHardware.as_ptr().offset(i as
                                                                  isize)).tim),
                     ENABLE);
        i += 1
    }
    let mut timerIndex: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while timerIndex < 13 as libc::c_int as libc::c_uint {
        let mut timerHardwarePtr: *const timerHardware_t =
            &*timerHardware.as_ptr().offset(timerIndex as isize) as
                *const timerHardware_t;
        if !((*timerHardwarePtr).usageFlags as libc::c_uint ==
                 TIM_USE_NONE as libc::c_int as libc::c_uint) {
            // XXX IOConfigGPIOAF in timerInit should eventually go away.
            IOConfigGPIOAF(IOGetByTag((*timerHardwarePtr).tag),
                           (0x2 as libc::c_uint |
                                (0 as libc::c_uint) << 2 as libc::c_int |
                                (0 as libc::c_uint) << 5 as libc::c_int) as
                               ioConfig_t,
                           (*timerHardwarePtr).alternateFunction);
        }
        timerIndex = timerIndex.wrapping_add(1)
    }
    /* enable the timer peripherals */
    let mut i_0: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i_0 < 13 as libc::c_int as libc::c_uint {
        RCC_ClockCmd(timerRCC((*timerHardware.as_ptr().offset(i_0 as
                                                                  isize)).tim),
                     ENABLE);
        i_0 = i_0.wrapping_add(1)
    }
    // initialize timer channel structures
    let mut i_1: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i_1 < 13 as libc::c_int as libc::c_uint {
        timerChannelInfo[i_1 as usize].type_0 = TYPE_FREE;
        i_1 = i_1.wrapping_add(1)
    }
    let mut i_2: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i_2 <
              ((((1 as libc::c_int) << 1 as libc::c_int |
                     (1 as libc::c_int) << 2 as libc::c_int |
                     (1 as libc::c_int) << 3 as libc::c_int |
                     (1 as libc::c_int) << 4 as libc::c_int |
                     (1 as libc::c_int) << 8 as libc::c_int |
                     (1 as libc::c_int) << 9 as libc::c_int) -
                    (((1 as libc::c_int) << 1 as libc::c_int |
                          (1 as libc::c_int) << 2 as libc::c_int |
                          (1 as libc::c_int) << 3 as libc::c_int |
                          (1 as libc::c_int) << 4 as libc::c_int |
                          (1 as libc::c_int) << 8 as libc::c_int |
                          (1 as libc::c_int) << 9 as libc::c_int) >>
                         1 as libc::c_int & 0x77777777 as libc::c_int) -
                    (((1 as libc::c_int) << 1 as libc::c_int |
                          (1 as libc::c_int) << 2 as libc::c_int |
                          (1 as libc::c_int) << 3 as libc::c_int |
                          (1 as libc::c_int) << 4 as libc::c_int |
                          (1 as libc::c_int) << 8 as libc::c_int |
                          (1 as libc::c_int) << 9 as libc::c_int) >>
                         2 as libc::c_int & 0x33333333 as libc::c_int) -
                    (((1 as libc::c_int) << 1 as libc::c_int |
                          (1 as libc::c_int) << 2 as libc::c_int |
                          (1 as libc::c_int) << 3 as libc::c_int |
                          (1 as libc::c_int) << 4 as libc::c_int |
                          (1 as libc::c_int) << 8 as libc::c_int |
                          (1 as libc::c_int) << 9 as libc::c_int) >>
                         3 as libc::c_int & 0x11111111 as libc::c_int) +
                    (((1 as libc::c_int) << 1 as libc::c_int |
                          (1 as libc::c_int) << 2 as libc::c_int |
                          (1 as libc::c_int) << 3 as libc::c_int |
                          (1 as libc::c_int) << 4 as libc::c_int |
                          (1 as libc::c_int) << 8 as libc::c_int |
                          (1 as libc::c_int) << 9 as libc::c_int) -
                         (((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int) >>
                              1 as libc::c_int & 0x77777777 as libc::c_int) -
                         (((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int) >>
                              2 as libc::c_int & 0x33333333 as libc::c_int) -
                         (((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 4 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 9 as libc::c_int) >>
                              3 as libc::c_int & 0x11111111 as libc::c_int) >>
                         4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                   255 as libc::c_int) as libc::c_uint {
        timerInfo[i_2 as usize].priority = !(0 as libc::c_int) as uint8_t;
        i_2 = i_2.wrapping_add(1)
    };
}
// finish configuring timers after allocation phase
// start timers
// TODO - Work in progress - initialization routine must be modified/verified to start correctly without timers
#[no_mangle]
pub unsafe extern "C" fn timerStart() { }
/* *
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param TIM_Typedef *tim The timer to overflow
 * @return void
 **/
#[no_mangle]
pub unsafe extern "C" fn timerForceOverflow(mut tim: *mut TIM_TypeDef) {
    let mut timerIndex: uint8_t = lookupTimerIndex(tim as *const TIM_TypeDef);
    let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
    let mut __ToDo: uint8_t =
        __basepriSetMemRetVal((((1 as libc::c_int) <<
                                    (4 as libc::c_int as
                                         libc::c_uint).wrapping_sub((7 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_sub(0x5
                                                                                                        as
                                                                                                        libc::c_uint))
                                    |
                                    1 as libc::c_int &
                                        0xf as libc::c_int >>
                                            (7 as libc::c_int as
                                                 libc::c_uint).wrapping_sub(0x5
                                                                                as
                                                                                libc::c_uint))
                                   << 4 as libc::c_int & 0xf0 as libc::c_int)
                                  as uint8_t);
    while __ToDo != 0 {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        timerConfig[timerIndex as usize].forcedOverflowTimerValue =
            (*tim).CNT.wrapping_add(1 as libc::c_int as libc::c_uint);
        // Force an overflow by setting the UG bit
        ::core::ptr::write_volatile(&mut (*tim).EGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*tim).EGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        __ToDo = 0 as libc::c_int as uint8_t
    };
}
// DMA_Handle_index
#[no_mangle]
pub unsafe extern "C" fn timerDmaIndex(mut channel: uint8_t) -> uint16_t {
    match channel as libc::c_int {
        0 => { return 0x1 as libc::c_uint as uint16_t }
        4 => { return 0x2 as libc::c_uint as uint16_t }
        8 => { return 0x3 as libc::c_uint as uint16_t }
        12 => { return 0x4 as libc::c_uint as uint16_t }
        _ => { }
    }
    return 0 as libc::c_int as uint16_t;
}
// TIM_DMA_sources
#[no_mangle]
pub unsafe extern "C" fn timerDmaSource(mut channel: uint8_t) -> uint16_t {
    match channel as libc::c_int {
        0 => {
            return ((0x1 as libc::c_uint) << 9 as libc::c_uint) as uint16_t
        }
        4 => {
            return ((0x1 as libc::c_uint) << 10 as libc::c_uint) as uint16_t
        }
        8 => {
            return ((0x1 as libc::c_uint) << 11 as libc::c_uint) as uint16_t
        }
        12 => {
            return ((0x1 as libc::c_uint) << 12 as libc::c_uint) as uint16_t
        }
        _ => { }
    }
    return 0 as libc::c_int as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerGetPrescalerByDesiredMhz(mut tim:
                                                           *mut TIM_TypeDef,
                                                       mut mhz: uint16_t)
 -> uint16_t {
    return timerGetPrescalerByDesiredHertz(tim,
                                           (mhz as libc::c_int *
                                                1000000 as libc::c_int) as
                                               uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn timerGetPeriodByPrescaler(mut tim: *mut TIM_TypeDef,
                                                   mut prescaler: uint16_t,
                                                   mut hz: uint32_t)
 -> uint16_t {
    return timerClock(tim).wrapping_div((prescaler as libc::c_int +
                                             1 as libc::c_int) as
                                            libc::c_uint).wrapping_div(hz) as
               uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerGetPrescalerByDesiredHertz(mut tim:
                                                             *mut TIM_TypeDef,
                                                         mut hz: uint32_t)
 -> uint16_t {
    // protection here for desired hertz > SystemCoreClock???
    if hz > timerClock(tim) { return 0 as libc::c_int as uint16_t }
    return (timerClock(tim).wrapping_add(hz.wrapping_div(2 as libc::c_int as
                                                             libc::c_uint)).wrapping_div(hz)
                as uint16_t as libc::c_int - 1 as libc::c_int) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn TIM_DMACmd(mut htim: *mut TIM_HandleTypeDef,
                                    mut Channel: uint32_t,
                                    mut NewState: FunctionalState)
 -> HAL_StatusTypeDef {
    match Channel {
        0 => {
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
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
            } else {
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
        }
        4 => {
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
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
            } else {
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
        }
        8 => {
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
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
            } else {
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
        }
        12 => {
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
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
            } else {
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
        }
        _ => { }
    }
    /* Change the htim state */
    ::core::ptr::write_volatile(&mut (*htim).State as
                                    *mut HAL_TIM_StateTypeDef,
                                HAL_TIM_STATE_READY);
    /* Return function status */
    return HAL_OK;
}
#[no_mangle]
pub unsafe extern "C" fn DMA_SetCurrDataCounter(mut htim:
                                                    *mut TIM_HandleTypeDef,
                                                mut Channel: uint32_t,
                                                mut pData: *mut uint32_t,
                                                mut Length: uint16_t)
 -> HAL_StatusTypeDef {
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
        }
        _ => { }
    }
    /* Return function status */
    return HAL_OK;
}
unsafe extern "C" fn run_static_initializers() {
    usedTimers =
        [(0x40000000 as
              libc::c_uint).wrapping_add(0x10000 as
                                             libc::c_uint).wrapping_add(0 as
                                                                            libc::c_uint)
             as *mut TIM_TypeDef,
         (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint) as
             *mut TIM_TypeDef,
         (0x40000000 as libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
             *mut TIM_TypeDef,
         (0x40000000 as libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
             *mut TIM_TypeDef,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x10000 as
                                             libc::c_uint).wrapping_add(0x400
                                                                            as
                                                                            libc::c_uint)
             as *mut TIM_TypeDef,
         (0x40000000 as
              libc::c_uint).wrapping_add(0x10000 as
                                             libc::c_uint).wrapping_add(0x4000
                                                                            as
                                                                            libc::c_uint)
             as *mut TIM_TypeDef]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
