use ::libc;
extern "C" {
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup GPIO_Exported_Constants GPIO Exported Constants
  * @{
  */
    /* * @defgroup GPIO_pins_define GPIO pins define
  * @{
  */
    /* Pin 0 selected    */
    /* Pin 1 selected    */
    /* Pin 2 selected    */
    /* Pin 3 selected    */
    /* Pin 4 selected    */
    /* Pin 5 selected    */
    /* Pin 6 selected    */
    /* Pin 7 selected    */
    /* Pin 8 selected    */
    /* Pin 9 selected    */
    /* Pin 10 selected   */
    /* Pin 11 selected   */
    /* Pin 12 selected   */
    /* Pin 13 selected   */
    /* Pin 14 selected   */
    /* Pin 15 selected   */
    /* All pins selected */
    /* PIN mask for assert test */
    /* *
  * @}
  */
    /* * @defgroup GPIO_mode_define GPIO mode define
  * @brief GPIO Configuration Mode 
  *        Elements values convention: 0xX0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection 
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @{
  */
    /* !< Input Floating Mode                   */
    /* !< Output Push Pull Mode                 */
    /* !< Output Open Drain Mode                */
    /* !< Alternate Function Push Pull Mode     */
    /* !< Alternate Function Open Drain Mode    */
    /* !< Analog Mode  */
    /* !< External Interrupt Mode with Rising edge trigger detection          */
    /* !< External Interrupt Mode with Falling edge trigger detection         */
    /* !< External Interrupt Mode with Rising/Falling edge trigger detection  */
    /* !< External Event Mode with Rising edge trigger detection               */
    /* !< External Event Mode with Falling edge trigger detection              */
    /* !< External Event Mode with Rising/Falling edge trigger detection       */
    /* *
  * @}
  */
    /* * @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */
    /* !< Low speed     */
    /* !< Medium speed  */
    /* !< Fast speed    */
    /* !< High speed    */
    /* *
  * @}
  */
    /* * @defgroup GPIO_pull_define GPIO pull define
   * @brief GPIO Pull-Up or Pull-Down Activation
   * @{
   */
    /* !< No Pull-up or Pull-down activation  */
    /* !< Pull-up activation                  */
    /* !< Pull-down activation                */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Macros GPIO Exported Macros
  * @{
  */
    /* *
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  __EXTI_LINE__: specifies the EXTI line flag to check.
  *         This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
    /* *
  * @brief  Clears the EXTI's line pending flags.
  * @param  __EXTI_LINE__: specifies the EXTI lines flags to clear.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
    /* *
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
    /* *
  * @brief  Clears the EXTI's line pending bits.
  * @param  __EXTI_LINE__: specifies the EXTI lines to clear.
  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
    /* *
  * @brief  Generates a Software interrupt on selected EXTI line.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval None
  */
    /* *
  * @}
  */
    /* Include GPIO HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup GPIO_Exported_Functions
  * @{
  */
    /* * @addtogroup GPIO_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
    #[no_mangle]
    fn HAL_GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                     GPIO_Init: *mut GPIO_InitTypeDef);
    #[no_mangle]
    fn HAL_DACEx_NoiseWaveGenerate(hdac_0: *mut DAC_HandleTypeDef,
                                   Channel: uint32_t, Amplitude: uint32_t)
     -> HAL_StatusTypeDef;
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
    #[no_mangle]
    fn HAL_DAC_Init(hdac_0: *mut DAC_HandleTypeDef) -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup DAC_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ****************************************************/
    #[no_mangle]
    fn HAL_DAC_Start(hdac_0: *mut DAC_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DAC_Stop(hdac_0: *mut DAC_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup DAC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions ***********************************************/
    #[no_mangle]
    fn HAL_DAC_ConfigChannel(hdac_0: *mut DAC_HandleTypeDef,
                             sConfig_0: *mut DAC_ChannelConfTypeDef,
                             Channel: uint32_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DAC_SetValue(hdac_0: *mut DAC_HandleTypeDef, Channel: uint32_t,
                        Alignment: uint32_t, Data: uint32_t)
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
    #[no_mangle]
    fn HAL_TIM_Base_Stop(htim: *mut TIM_HandleTypeDef) -> HAL_StatusTypeDef;
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
    fn TIM_Base_SetConfig(TIMx: *mut TIM_TypeDef,
                          Structure: *mut TIM_Base_InitTypeDef);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint32_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
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
  * @file    stm32f7xx_hal_gpio.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of GPIO HAL module.
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
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Types GPIO Exported Types
  * @{
  */
/* * 
  * @brief GPIO Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub Pin: uint32_t,
    pub Mode: uint32_t,
    pub Pull: uint32_t,
    pub Speed: uint32_t,
    pub Alternate: uint32_t,
}
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
pub type HAL_DMA_StateTypeDef = libc::c_uint;
pub const HAL_DMA_STATE_ABORT: HAL_DMA_StateTypeDef = 5;
pub const HAL_DMA_STATE_ERROR: HAL_DMA_StateTypeDef = 4;
pub const HAL_DMA_STATE_TIMEOUT: HAL_DMA_StateTypeDef = 3;
pub const HAL_DMA_STATE_BUSY: HAL_DMA_StateTypeDef = 2;
pub const HAL_DMA_STATE_READY: HAL_DMA_StateTypeDef = 1;
pub const HAL_DMA_STATE_RESET: HAL_DMA_StateTypeDef = 0;
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
/* * 
  * @brief  DMA handle Structure definition
  */
pub type DMA_HandleTypeDef = __DMA_HandleTypeDef;
/* !< Register base address                  */
/* !< DMA communication parameters           */
/* !< DMA locking object                     */
/* !< DMA transfer state                     */
/* !< Parent object state                    */
/* !< DMA transfer complete callback         */
/* !< DMA Half transfer complete callback    */
/* !< DMA transfer complete Memory1 callback */
/* !< DMA transfer Half complete Memory1 callback */
/* !< DMA transfer error callback            */
/* !< DMA transfer Abort callback            */
/* !< DMA Error code                          */
/* !< DMA Stream Base Address                */
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
static mut hdac: DAC_HandleTypeDef =
    DAC_HandleTypeDef{Instance: 0 as *const DAC_TypeDef as *mut DAC_TypeDef,
                      State: HAL_DAC_STATE_RESET,
                      Lock: HAL_UNLOCKED,
                      DMA_Handle1:
                          0 as *const DMA_HandleTypeDef as
                              *mut DMA_HandleTypeDef,
                      DMA_Handle2:
                          0 as *const DMA_HandleTypeDef as
                              *mut DMA_HandleTypeDef,
                      ErrorCode: 0,};
static mut handle: TIM_HandleTypeDef =
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
static mut sConfig: DAC_ChannelConfTypeDef =
    DAC_ChannelConfTypeDef{DAC_Trigger: 0, DAC_OutputBuffer: 0,};
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
pub unsafe extern "C" fn audioSetupIO() {
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
                                         29 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        29 as libc::c_uint);
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
                                         4 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        4 as libc::c_uint);
    hdac.Instance =
        (0x40000000 as libc::c_uint).wrapping_add(0x7400 as libc::c_uint) as
            *mut DAC_TypeDef;
    HAL_DAC_Init(&mut hdac);
    let mut GPIO_InitStruct: GPIO_InitTypeDef =
        GPIO_InitTypeDef{Pin: 0, Mode: 0, Pull: 0, Speed: 0, Alternate: 0,};
    GPIO_InitStruct.Pin = 0x10 as libc::c_uint as uint16_t as uint32_t;
    GPIO_InitStruct.Mode = 0x3 as libc::c_uint;
    GPIO_InitStruct.Pull = 0 as libc::c_uint;
    HAL_GPIO_Init((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
}
#[no_mangle]
pub unsafe extern "C" fn audioGenerateWhiteNoise() {
    // TIM6 runs on APB2 at 42Mhz
    handle.Instance =
        (0x40000000 as libc::c_uint).wrapping_add(0x1000 as libc::c_uint) as
            *mut TIM_TypeDef;
    handle.Init.Period = 0xff as libc::c_int as uint32_t;
    handle.Init.Prescaler = 0 as libc::c_int as uint32_t;
    handle.Init.ClockDivision = 0 as libc::c_uint;
    handle.Init.CounterMode = 0 as libc::c_uint;
    handle.Init.RepetitionCounter = 0 as libc::c_int as uint32_t;
    HAL_TIM_Base_Init(&mut handle);
    let mut sMasterConfig: TIM_MasterConfigTypeDef =
        TIM_MasterConfigTypeDef{MasterOutputTrigger: 0,
                                MasterOutputTrigger2: 0,
                                MasterSlaveMode: 0,};
    sMasterConfig.MasterSlaveMode = 0x80 as libc::c_int as uint32_t;
    sMasterConfig.MasterOutputTrigger =
        (0x2 as libc::c_uint) << 4 as libc::c_uint;
    HAL_TIMEx_MasterConfigSynchronization(&mut handle, &mut sMasterConfig);
    HAL_TIM_Base_Start(&mut handle);
    sConfig.DAC_Trigger = (0x1 as libc::c_uint) << 2 as libc::c_uint;
    sConfig.DAC_OutputBuffer = 0 as libc::c_uint;
    HAL_DAC_ConfigChannel(&mut hdac, &mut sConfig, 0 as libc::c_uint);
    HAL_DACEx_NoiseWaveGenerate(&mut hdac, 0 as libc::c_uint,
                                (0x8 as libc::c_uint) << 8 as libc::c_uint |
                                    (0x2 as libc::c_uint) <<
                                        8 as libc::c_uint);
    HAL_DAC_SetValue(&mut hdac, 0 as libc::c_uint, 0x4 as libc::c_uint,
                     0xcd00 as libc::c_int as uint32_t);
    HAL_DAC_Start(&mut hdac, 0 as libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn audioPlayTone(mut tone: uint8_t) {
    handle.Init.Period =
        (64 as libc::c_int +
             ({
                  let mut _a: libc::c_int = 0 as libc::c_int;
                  let mut _b: libc::c_int =
                      ({
                           let mut _a_0: uint8_t = tone;
                           let mut _b_0: libc::c_int =
                               64 as libc::c_int - 1 as libc::c_int;
                           (if (_a_0 as libc::c_int) < _b_0 {
                                _a_0 as libc::c_int
                            } else { _b_0 })
                       });
                  (if _a > _b { _a } else { _b })
              }) * 8 as libc::c_int) as uint32_t;
    TIM_Base_SetConfig(handle.Instance, &mut handle.Init);
}
// TONE_MIN to TONE_MAX
#[no_mangle]
pub unsafe extern "C" fn audioSilence() {
    HAL_DAC_Stop(&mut hdac, 0 as libc::c_uint);
    HAL_TIM_Base_Stop(&mut handle);
}
