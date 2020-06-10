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
    fn HAL_DMA_DeInit(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
/* * @defgroup UART_Exported_Constants UART Exported Constants
  * @{
  */
/* * @defgroup UART_Error_Definition   UART Error Definition
  * @{
  */
    /* !< No error            */
    /* !< Parity error        */
    /* !< Noise error         */
    /* !< frame error         */
    /* !< Overrun error       */
    /* !< DMA transfer error  */
    /* *
  * @}
  */
/* * @defgroup UART_Stop_Bits   UART Number of Stop Bits
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Parity  UART Parity
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Hardware_Flow_Control UART Hardware Flow Control
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Mode UART Transfer Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_State  UART State
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Over_Sampling UART Over Sampling
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_OneBit_Sampling UART One Bit Sampling Method
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_AutoBaud_Rate_Mode    UART Advanced Feature AutoBaud Rate Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Receiver_TimeOut UART Receiver TimeOut
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_LIN    UART Local Interconnection Network mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_LIN_Break_Detection  UART LIN Break Detection
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_DMA_Tx    UART DMA Tx
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_DMA_Rx   UART DMA Rx
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Half_Duplex_Selection  UART Half Duplex Selection
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_WakeUp_Methods   UART WakeUp Methods
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Request_Parameters UART Request Parameters
  * @{
  */
    /* !< Auto-Baud Rate Request */
    /* !< Send Break Request */
    /* !< Mute Mode Request */
    /* !< Receive Data flush Request */
    /* !< Transmit data flush Request */
    /* *
  * @}
  */
    /* * @defgroup UART_Advanced_Features_Initialization_Type  UART Advanced Feature Initialization Type
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Tx_Inv UART Advanced Feature TX Pin Active Level Inversion
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Rx_Inv UART Advanced Feature RX Pin Active Level Inversion
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Data_Inv  UART Advanced Feature Binary Data Inversion
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Rx_Tx_Swap UART Advanced Feature RX TX Pins Swap
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Overrun_Disable  UART Advanced Feature Overrun Disable
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_AutoBaudRate_Enable  UART Advanced Feature Auto BaudRate Enable
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_DMA_Disable_on_Rx_Error   UART Advanced Feature DMA Disable On Rx Error
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_MSB_First   UART Advanced Feature MSB First
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Mute_Mode   UART Advanced Feature Mute Mode Enable
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_CR2_ADDRESS_LSB_POS    UART Address-matching LSB Position In CR2 Register
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_DriverEnable_Polarity      UART DriverEnable Polarity
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_CR1_DEAT_ADDRESS_LSB_POS    UART Driver Enable Assertion Time LSB Position In CR1 Register
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_CR1_DEDT_ADDRESS_LSB_POS    UART Driver Enable DeAssertion Time LSB Position In CR1 Register
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Interruption_Mask    UART Interruptions Flag Mask
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_TimeOut_Value    UART polling-based communications time-out value
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Flags     UART Status Flags
  *        Elements values convention: 0xXXXX
  *           - 0xXXXX  : Flag mask in the ISR register
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup UART_Interrupt_definition   UART Interrupts Definition
  *        Elements values convention: 0000ZZZZ0XXYYYYYb
  *           - YYYYY  : Interrupt source position in the XX register (5bits)
  *           - XX  : Interrupt source register (2bits)
  *                 - 01: CR1 register
  *                 - 10: CR2 register
  *                 - 11: CR3 register
  *           - ZZZZ  : Flag position in the ISR register(4bits)
  * @{
  */
    /* *       Elements values convention: 000000000XXYYYYYb
  *           - YYYYY  : Interrupt source position in the XX register (5bits)
  *           - XX  : Interrupt source register (2bits)
  *                 - 01: CR1 register
  *                 - 10: CR2 register
  *                 - 11: CR3 register
  */
    /* *       Elements values convention: 0000ZZZZ00000000b
  *           - ZZZZ  : Flag position in the ISR register(4bits)
  */
    /* *
  * @}
  */
    /* * @defgroup UART_IT_CLEAR_Flags  UART Interruption Clear Flags
  * @{
  */
    /* !< Parity Error Clear Flag */
    /* !< Framing Error Clear Flag */
    /* !< Noise detected Clear Flag */
    /* !< OverRun Error Clear Flag */
    /* !< IDLE line detected Clear Flag */
    /* !< Transmission Complete Clear Flag */
    /* !< LIN Break Detection Clear Flag */
    /* !< CTS Interrupt Clear Flag */
    /* !< Receiver Time Out Clear Flag */
    /* !< End Of Block Clear Flag */
    /* !< Character Match Clear Flag */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macros -----------------------------------------------------------*/
/* * @defgroup UART_Exported_Macros UART Exported Macros
  * @{
  */
    /* * @brief Reset UART handle state
  * @param  __HANDLE__: UART handle.
  * @retval None
  */
    /* * @brief  Flush the UART Data registers
  * @param  __HANDLE__: specifies the UART Handle.
  */
    /* * @brief  Clears the specified UART ISR flag, in setting the proper ICR register flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __FLAG__: specifies the interrupt clear register flag that needs to be set
  *                       to clear the corresponding interrupt
  *          This parameter can be one of the following values:
  *            @arg UART_CLEAR_PEF: Parity Error Clear Flag
  *            @arg UART_CLEAR_FEF: Framing Error Clear Flag
  *            @arg UART_CLEAR_NEF: Noise detected Clear Flag
  *            @arg UART_CLEAR_OREF: OverRun Error Clear Flag
  *            @arg UART_CLEAR_IDLEF: IDLE line detected Clear Flag
  *            @arg UART_CLEAR_TCF: Transmission Complete Clear Flag
  *            @arg UART_CLEAR_LBDF: LIN Break Detection Clear Flag
  *            @arg UART_CLEAR_CTSF: CTS Interrupt Clear Flag
  *            @arg UART_CLEAR_RTOF: Receiver Time Out Clear Flag
  *            @arg UART_CLEAR_EOBF: End Of Block Clear Flag
  *            @arg UART_CLEAR_CMF: Character Match Clear Flag
  * @retval None
  */
    /* * @brief  Clear the UART PE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
    /* * @brief  Clear the UART FE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
    /* * @brief  Clear the UART NE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
    /* * @brief  Clear the UART ORE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
    /* * @brief  Clear the UART IDLE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
    /* * @brief  Checks whether the specified UART flag is set or not.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __FLAG__: specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg UART_FLAG_REACK: Receive enable acknowledge flag
  *            @arg UART_FLAG_TEACK: Transmit enable acknowledge flag
  *            @arg UART_FLAG_WUF:   Wake up from stop mode flag
  *            @arg UART_FLAG_RWU:   Receiver wake up flag (is the UART in mute mode)
  *            @arg UART_FLAG_SBKF:  Send Break flag
  *            @arg UART_FLAG_CMF:   Character match flag
  *            @arg UART_FLAG_BUSY:  Busy flag
  *            @arg UART_FLAG_ABRF:  Auto Baud rate detection flag
  *            @arg UART_FLAG_ABRE:  Auto Baud rate detection error flag
  *            @arg UART_FLAG_EOBF:  End of block flag
  *            @arg UART_FLAG_RTOF:  Receiver timeout flag
  *            @arg UART_FLAG_CTS:   CTS Change flag (not available for UART4 and UART5)
  *            @arg UART_FLAG_LBD:   LIN Break detection flag
  *            @arg UART_FLAG_TXE:   Transmit data register empty flag
  *            @arg UART_FLAG_TC:    Transmission Complete flag
  *            @arg UART_FLAG_RXNE:  Receive data register not empty flag
  *            @arg UART_FLAG_IDLE:  Idle Line detection flag
  *            @arg UART_FLAG_ORE:   OverRun Error flag
  *            @arg UART_FLAG_NE:    Noise Error flag
  *            @arg UART_FLAG_FE:    Framing Error flag
  *            @arg UART_FLAG_PE:    Parity Error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
    /* * @brief  Enables the specified UART interrupt.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __INTERRUPT__: specifies the UART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_WUF:  Wakeup from stop mode interrupt
  *            @arg UART_IT_CM:   Character match interrupt
  *            @arg UART_IT_CTS:  CTS change interrupt
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
    /* * @brief  Disables the specified UART interrupt.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __INTERRUPT__: specifies the UART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CM:   Character match interrupt
  *            @arg UART_IT_CTS:  CTS change interrupt
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
    /* * @brief  Checks whether the specified UART interrupt has occurred or not.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __IT__: specifies the UART interrupt to check.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CM:   Character match interrupt
  *            @arg UART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_ORE:  OverRun Error interrupt
  *            @arg UART_IT_NE:   Noise Error interrupt
  *            @arg UART_IT_FE:   Framing Error interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
    /* * @brief  Checks whether the specified UART interrupt source is enabled.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __IT__: specifies the UART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CTS: CTS change interrupt (not available for UART4 and UART5)
  *            @arg UART_IT_LBD: LIN Break detection interrupt
  *            @arg UART_IT_TXE: Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:  Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_ORE: OverRun Error interrupt
  *            @arg UART_IT_NE: Noise Error interrupt
  *            @arg UART_IT_FE: Framing Error interrupt
  *            @arg UART_IT_PE: Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
    /* * @brief  Set a specific UART request flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __REQ__: specifies the request flag to set
  *          This parameter can be one of the following values:
  *            @arg UART_AUTOBAUD_REQUEST: Auto-Baud Rate Request
  *            @arg UART_SENDBREAK_REQUEST: Send Break Request
  *            @arg UART_MUTE_MODE_REQUEST: Mute Mode Request
  *            @arg UART_RXDATA_FLUSH_REQUEST: Receive Data flush Request
  *            @arg UART_TXDATA_FLUSH_REQUEST: Transmit data flush Request
  * @retval None
  */
    /* * @brief  Enables the UART one bit sample method
  * @param  __HANDLE__: specifies the UART Handle.  
  * @retval None
  */
    /* * @brief  Disables the UART one bit sample method
  * @param  __HANDLE__: specifies the UART Handle.  
  * @retval None
  */
    /* * @brief  Enable UART
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
    /* * @brief  Disable UART
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
    /* * @brief  Enable CTS flow control 
  *         This macro allows to enable CTS hardware flow control for a given UART instance, 
  *         without need to call HAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying CTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of HAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __HAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __HAL_UART_ENABLE(__HANDLE__)).                                                                                                                  
  * @param  __HANDLE__: specifies the UART Handle.
  *         The Handle Instance can be USART1, USART2 or LPUART.
  * @retval None
  */
    /* * @brief  Disable CTS flow control 
  *         This macro allows to disable CTS hardware flow control for a given UART instance, 
  *         without need to call HAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying CTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of HAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __HAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __HAL_UART_ENABLE(__HANDLE__)). 
  * @param  __HANDLE__: specifies the UART Handle.
  *         The Handle Instance can be USART1, USART2 or LPUART.
  * @retval None
  */
    /* * @brief  Enable RTS flow control 
  *         This macro allows to enable RTS hardware flow control for a given UART instance, 
  *         without need to call HAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying RTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of HAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __HAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __HAL_UART_ENABLE(__HANDLE__)). 
  * @param  __HANDLE__: specifies the UART Handle.
  *         The Handle Instance can be USART1, USART2 or LPUART.
  * @retval None
  */
    /* * @brief  Disable RTS flow control 
  *         This macro allows to disable RTS hardware flow control for a given UART instance, 
  *         without need to call HAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying RTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of HAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __HAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __HAL_UART_ENABLE(__HANDLE__)). 
  * @param  __HANDLE__: specifies the UART Handle.
  *         The Handle Instance can be USART1, USART2 or LPUART.
  * @retval None
  */
    /* *
  * @}
  */
    /* Private macros --------------------------------------------------------*/
/* * @defgroup UART_Private_Macros   UART Private Macros
  * @{
  */
/* * @brief  BRR division operation to set BRR register with LPUART
  * @param  _PCLK_: LPUART clock
  * @param  _BAUD_: Baud rate set by the user
  * @retval Division result
  */
    /* * @brief  BRR division operation to set BRR register in 8-bit oversampling mode
  * @param  _PCLK_: UART clock
  * @param  _BAUD_: Baud rate set by the user
  * @retval Division result
  */
    /* * @brief  BRR division operation to set BRR register in 16-bit oversampling mode
  * @param  _PCLK_: UART clock
  * @param  _BAUD_: Baud rate set by the user
  * @retval Division result
  */
    /* * @brief  Check UART Baud rate
  * @param  BAUDRATE: Baudrate specified by the user
  *         The maximum Baud Rate is derived from the maximum clock on F7 (i.e. 216 MHz)
  *         divided by the smallest oversampling used on the USART (i.e. 8)
  * @retval Test result (TRUE or FALSE).
  */
    /* * @brief  Check UART assertion time
  * @param  TIME: 5-bit value assertion time
  * @retval Test result (TRUE or FALSE).
  */
    /* * @brief  Check UART deassertion time
  * @param  TIME: 5-bit value deassertion time
  * @retval Test result (TRUE or FALSE).
  */
    /* *
  * @}
  */
/* Include UART HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup UART_Exported_Functions UART Exported Functions
  * @{
  */
    /* * @addtogroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
    /* Initialization and de-initialization functions  ****************************/
    #[no_mangle]
    fn HAL_UART_Init(huart: *mut UART_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_HalfDuplex_Init(huart: *mut UART_HandleTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_UART_DeInit(huart: *mut UART_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_UART_Transmit_DMA(huart: *mut UART_HandleTypeDef,
                             pData: *mut uint8_t, Size: uint16_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_UART_Receive_DMA(huart: *mut UART_HandleTypeDef,
                            pData: *mut uint8_t, Size: uint16_t)
     -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup UART_Exported_Functions_Group4 Peripheral State and Error functions
  * @{
  */
    /* Peripheral State and Errors functions  **************************************************/
    #[no_mangle]
    fn HAL_UART_GetState(huart: *mut UART_HandleTypeDef)
     -> HAL_UART_StateTypeDef;
    #[no_mangle]
    static mut uartDevmap: [*mut uartDevice_t; 0];
    #[no_mangle]
    fn serialUART(device: UARTDevice_e, baudRate: uint32_t, mode: portMode_e,
                  options: portOptions_e) -> *mut uartPort_t;
    #[no_mangle]
    fn uartIrqHandler(s: *mut uartPort_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
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
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub CR3: uint32_t,
    pub BRR: uint32_t,
    pub GTPR: uint32_t,
    pub RTOR: uint32_t,
    pub RQR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub RDR: uint32_t,
    pub TDR: uint32_t,
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
  * @file    stm32f7xx_hal_uart.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of UART HAL module.
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
/* * @addtogroup UART
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup UART_Exported_Types UART Exported Types
  * @{
  */
/* *
  * @brief UART Init Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_InitTypeDef {
    pub BaudRate: uint32_t,
    pub WordLength: uint32_t,
    pub StopBits: uint32_t,
    pub Parity: uint32_t,
    pub Mode: uint32_t,
    pub HwFlowCtl: uint32_t,
    pub OverSampling: uint32_t,
    pub OneBitSampling: uint32_t,
}
/* *
  * @brief  UART Advanced Features initialization structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_AdvFeatureInitTypeDef {
    pub AdvFeatureInit: uint32_t,
    pub TxPinLevelInvert: uint32_t,
    pub RxPinLevelInvert: uint32_t,
    pub DataInvert: uint32_t,
    pub Swap: uint32_t,
    pub OverrunDisable: uint32_t,
    pub DMADisableonRxError: uint32_t,
    pub AutoBaudRateEnable: uint32_t,
    pub AutoBaudRateMode: uint32_t,
    pub MSBFirst: uint32_t,
}
/* *
  * @brief HAL UART State structures definition
  * @note  HAL UART State value is a combination of 2 different substates: gState and RxState.
  *        - gState contains UART state information related to global Handle management 
  *          and also information related to Tx operations.
  *          gState value coding follow below described bitmap :
  *          b7-b6  Error information 
  *             00 : No Error
  *             01 : (Not Used)
  *             10 : Timeout
  *             11 : Error
  *          b5     IP initilisation status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP not initialized. HAL UART Init function already called)
  *          b4-b3  (not used)
  *             xx : Should be set to 00
  *          b2     Intrinsic process state
  *             0  : Ready
  *             1  : Busy (IP busy with some configuration or internal operations)
  *          b1     (not used)
  *             x  : Should be set to 0
  *          b0     Tx state
  *             0  : Ready (no Tx operation ongoing)
  *             1  : Busy (Tx operation ongoing)
  *        - RxState contains information related to Rx operations.
  *          RxState value coding follow below described bitmap :
  *          b7-b6  (not used)
  *             xx : Should be set to 00
  *          b5     IP initilisation status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP not initialized)
  *          b4-b2  (not used)
  *            xxx : Should be set to 000
  *          b1     Rx state
  *             0  : Ready (no Rx operation ongoing)
  *             1  : Busy (Rx operation ongoing)
  *          b0     (not used)
  *             x  : Should be set to 0.
  */
pub type HAL_UART_StateTypeDef = libc::c_uint;
/* !< Error
                                                   Value is allowed for gState only */
/* !< Timeout state
                                                   Value is allowed for gState only */
pub const HAL_UART_STATE_ERROR: HAL_UART_StateTypeDef = 224;
/* !< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
pub const HAL_UART_STATE_TIMEOUT: HAL_UART_StateTypeDef = 160;
/* !< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
pub const HAL_UART_STATE_BUSY_TX_RX: HAL_UART_StateTypeDef = 35;
/* !< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
pub const HAL_UART_STATE_BUSY_RX: HAL_UART_StateTypeDef = 34;
/* !< an internal process is ongoing 
                                                   Value is allowed for gState only */
pub const HAL_UART_STATE_BUSY_TX: HAL_UART_StateTypeDef = 33;
/* !< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
pub const HAL_UART_STATE_BUSY: HAL_UART_StateTypeDef = 36;
/* !< Peripheral is not initialized
                                                   Value is allowed for gState and RxState */
pub const HAL_UART_STATE_READY: HAL_UART_StateTypeDef = 32;
pub const HAL_UART_STATE_RESET: HAL_UART_StateTypeDef = 0;
/* *
  * @brief  UART handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_HandleTypeDef {
    pub Instance: *mut USART_TypeDef,
    pub Init: UART_InitTypeDef,
    pub AdvancedInit: UART_AdvFeatureInitTypeDef,
    pub pTxBuffPtr: *mut uint8_t,
    pub TxXferSize: uint16_t,
    pub TxXferCount: uint16_t,
    pub pRxBuffPtr: *mut uint8_t,
    pub RxXferSize: uint16_t,
    pub RxXferCount: uint16_t,
    pub Mask: uint16_t,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub gState: HAL_UART_StateTypeDef,
    pub RxState: HAL_UART_StateTypeDef,
    pub ErrorCode: uint32_t,
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
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
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
pub type UARTDevice_e = libc::c_uint;
pub const UARTDEV_8: UARTDevice_e = 7;
pub const UARTDEV_7: UARTDevice_e = 6;
pub const UARTDEV_6: UARTDevice_e = 5;
pub const UARTDEV_5: UARTDevice_e = 4;
pub const UARTDEV_4: UARTDevice_e = 3;
pub const UARTDEV_3: UARTDevice_e = 2;
pub const UARTDEV_2: UARTDevice_e = 1;
pub const UARTDEV_1: UARTDevice_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartPort_s {
    pub port: serialPort_t,
    pub rxDMAHandle: DMA_HandleTypeDef,
    pub txDMAHandle: DMA_HandleTypeDef,
    pub rxDMAStream: *mut DMA_Stream_TypeDef,
    pub txDMAStream: *mut DMA_Stream_TypeDef,
    pub rxDMAChannel: uint32_t,
    pub txDMAChannel: uint32_t,
    pub rxDMAIrq: uint32_t,
    pub txDMAIrq: uint32_t,
    pub rxDMAPos: uint32_t,
    pub txDMAPeripheralBaseAddr: uint32_t,
    pub rxDMAPeripheralBaseAddr: uint32_t,
    pub Handle: UART_HandleTypeDef,
    pub USARTx: *mut USART_TypeDef,
    pub txDMAEmpty: bool,
}
pub type uartPort_t = uartPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartHardware_s {
    pub device: UARTDevice_e,
    pub reg: *mut USART_TypeDef,
    pub DMAChannel: uint32_t,
    pub txDMAStream: *mut DMA_Stream_TypeDef,
    pub rxDMAStream: *mut DMA_Stream_TypeDef,
    pub rxPins: [ioTag_t; 3],
    pub txPins: [ioTag_t; 3],
    pub rcc_ahb1: uint32_t,
    pub rcc_apb2: rccPeriphTag_t,
    pub rcc_apb1: rccPeriphTag_t,
    pub af: uint8_t,
    pub txIrq: uint8_t,
    pub rxIrq: uint8_t,
    pub txPriority: uint8_t,
    pub rxPriority: uint8_t,
}
// All USARTs can also be used as UART, and we use them only as UART.
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
// Configuration constants
// Count number of configured UARTs
pub type uartHardware_t = uartHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartDevice_s {
    pub port: uartPort_t,
    pub hardware: *const uartHardware_t,
    pub rx: ioTag_t,
    pub tx: ioTag_t,
    pub rxBuffer: [uint8_t; 128],
    pub txBuffer: [uint8_t; 256],
}
// XXX Not required for full allocation
// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.
pub type uartDevice_t = uartDevice_s;
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
/*
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
unsafe extern "C" fn usartConfigurePinInversion(mut uartPort:
                                                    *mut uartPort_t) {
    let mut inverted: bool =
        (*uartPort).port.options as libc::c_uint &
            SERIAL_INVERTED as libc::c_int as libc::c_uint != 0;
    if inverted {
        if (*uartPort).port.mode as libc::c_uint &
               MODE_RX as libc::c_int as libc::c_uint != 0 {
            (*uartPort).Handle.AdvancedInit.AdvFeatureInit |=
                0x2 as libc::c_uint;
            (*uartPort).Handle.AdvancedInit.RxPinLevelInvert =
                (0x1 as libc::c_uint) << 16 as libc::c_uint
        }
        if (*uartPort).port.mode as libc::c_uint &
               MODE_TX as libc::c_int as libc::c_uint != 0 {
            (*uartPort).Handle.AdvancedInit.AdvFeatureInit |=
                0x1 as libc::c_uint;
            (*uartPort).Handle.AdvancedInit.TxPinLevelInvert =
                (0x1 as libc::c_uint) << 17 as libc::c_uint
        }
    };
}
// XXX uartReconfigure does not handle resource management properly.
#[no_mangle]
pub unsafe extern "C" fn uartReconfigure(mut uartPort: *mut uartPort_t) {
    /*RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3|
            RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART8;
    RCC_PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Usart6ClockSelection = RCC_USART6CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Uart7ClockSelection = RCC_UART7CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Uart8ClockSelection = RCC_UART8CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);*/
    HAL_UART_DeInit(&mut (*uartPort).Handle);
    (*uartPort).Handle.Init.BaudRate = (*uartPort).port.baudRate;
    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    (*uartPort).Handle.Init.WordLength =
        if (*uartPort).port.options as libc::c_uint &
               SERIAL_PARITY_EVEN as libc::c_int as libc::c_uint != 0 {
            ((0x1 as libc::c_uint)) << 12 as libc::c_uint
        } else { 0 as libc::c_uint };
    (*uartPort).Handle.Init.StopBits =
        if (*uartPort).port.options as libc::c_uint &
               SERIAL_STOPBITS_2 as libc::c_int as libc::c_uint != 0 {
            ((0x2 as libc::c_uint)) << 12 as libc::c_uint
        } else { 0 as libc::c_uint };
    (*uartPort).Handle.Init.Parity =
        if (*uartPort).port.options as libc::c_uint &
               SERIAL_PARITY_EVEN as libc::c_int as libc::c_uint != 0 {
            ((0x1 as libc::c_uint)) << 10 as libc::c_uint
        } else { 0 as libc::c_uint };
    (*uartPort).Handle.Init.HwFlowCtl = 0 as libc::c_uint;
    (*uartPort).Handle.Init.OneBitSampling = 0 as libc::c_uint;
    (*uartPort).Handle.Init.Mode = 0 as libc::c_int as uint32_t;
    if (*uartPort).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint != 0 {
        (*uartPort).Handle.Init.Mode |=
            (0x1 as libc::c_uint) << 2 as libc::c_uint
    }
    if (*uartPort).port.mode as libc::c_uint &
           MODE_TX as libc::c_int as libc::c_uint != 0 {
        (*uartPort).Handle.Init.Mode |=
            (0x1 as libc::c_uint) << 3 as libc::c_uint
    }
    usartConfigurePinInversion(uartPort);
    if (*uartPort).port.options as libc::c_uint &
           SERIAL_BIDIR as libc::c_int as libc::c_uint != 0 {
        HAL_HalfDuplex_Init(&mut (*uartPort).Handle);
    } else { HAL_UART_Init(&mut (*uartPort).Handle); }
    // Receive DMA or IRQ
    if (*uartPort).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint != 0 {
        if !(*uartPort).rxDMAStream.is_null() {
            (*uartPort).rxDMAHandle.Instance = (*uartPort).rxDMAStream;
            (*uartPort).rxDMAHandle.Init.Channel = (*uartPort).rxDMAChannel;
            (*uartPort).rxDMAHandle.Init.Direction = 0 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.PeriphInc = 0 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.MemInc =
                (0x1 as libc::c_uint) << 10 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.PeriphDataAlignment =
                0 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.MemDataAlignment = 0 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.Mode =
                (0x1 as libc::c_uint) << 8 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.FIFOMode = 0 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.FIFOThreshold = 0 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.PeriphBurst = 0 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.MemBurst = 0 as libc::c_uint;
            (*uartPort).rxDMAHandle.Init.Priority =
                (0x1 as libc::c_uint) << 16 as libc::c_uint;
            HAL_DMA_DeInit(&mut (*uartPort).rxDMAHandle);
            HAL_DMA_Init(&mut (*uartPort).rxDMAHandle);
            /* Associate the initialized DMA handle to the UART handle */
            (*uartPort).Handle.hdmarx = &mut (*uartPort).rxDMAHandle;
            (*uartPort).rxDMAHandle.Parent =
                &mut (*uartPort).Handle as *mut UART_HandleTypeDef as
                    *mut libc::c_void;
            HAL_UART_Receive_DMA(&mut (*uartPort).Handle,
                                 (*uartPort).port.rxBuffer as *mut uint8_t,
                                 (*uartPort).port.rxBufferSize as uint16_t);
            (*uartPort).rxDMAPos = (*(*uartPort).rxDMAHandle.Instance).NDTR
        } else {
            /* Enable the UART Parity Error Interrupt */
            ::core::ptr::write_volatile(&mut (*(*uartPort).USARTx).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*uartPort).USARTx).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 8 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            ::core::ptr::write_volatile(&mut (*(*uartPort).USARTx).CR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*uartPort).USARTx).CR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable the UART Data Register not empty Interrupt */
            ::core::ptr::write_volatile(&mut (*(*uartPort).USARTx).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*uartPort).USARTx).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    // Transmit DMA or IRQ
    if (*uartPort).port.mode as libc::c_uint &
           MODE_TX as libc::c_int as libc::c_uint != 0 {
        if !(*uartPort).txDMAStream.is_null() {
            (*uartPort).txDMAHandle.Instance = (*uartPort).txDMAStream;
            (*uartPort).txDMAHandle.Init.Channel = (*uartPort).txDMAChannel;
            (*uartPort).txDMAHandle.Init.Direction =
                (0x1 as libc::c_uint) << 6 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.PeriphInc = 0 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.MemInc =
                (0x1 as libc::c_uint) << 10 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.PeriphDataAlignment =
                0 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.MemDataAlignment = 0 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.Mode = 0 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.FIFOMode = 0 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.FIFOThreshold = 0 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.PeriphBurst = 0 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.MemBurst = 0 as libc::c_uint;
            (*uartPort).txDMAHandle.Init.Priority =
                (0x1 as libc::c_uint) << 16 as libc::c_uint;
            HAL_DMA_DeInit(&mut (*uartPort).txDMAHandle);
            let mut status: HAL_StatusTypeDef =
                HAL_DMA_Init(&mut (*uartPort).txDMAHandle);
            if status as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint
               {
                loop  { }
            }
            /* Associate the initialized DMA handle to the UART handle */
            (*uartPort).Handle.hdmatx = &mut (*uartPort).txDMAHandle;
            (*uartPort).txDMAHandle.Parent =
                &mut (*uartPort).Handle as *mut UART_HandleTypeDef as
                    *mut libc::c_void;
            ::core::ptr::write_volatile(&mut (*(*uartPort).txDMAHandle.Instance).NDTR
                                            as *mut uint32_t,
                                        0 as libc::c_int as uint16_t as
                                            uint32_t)
        } else {
            /* Enable the UART Transmit Data Register Empty Interrupt */
            ::core::ptr::write_volatile(&mut (*(*uartPort).USARTx).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*uartPort).USARTx).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 7 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn uartOpen(mut device: UARTDevice_e,
                                  mut callback: serialReceiveCallbackPtr,
                                  mut callbackData: *mut libc::c_void,
                                  mut baudRate: uint32_t,
                                  mut mode: portMode_e,
                                  mut options: portOptions_e)
 -> *mut serialPort_t {
    let mut s: *mut uartPort_t = serialUART(device, baudRate, mode, options);
    if s.is_null() { return s as *mut serialPort_t }
    (*s).txDMAEmpty = 1 as libc::c_int != 0;
    // common serial initialisation code should move to serialPort::init()
    (*s).port.rxBufferTail = 0 as libc::c_int as uint32_t;
    (*s).port.rxBufferHead = (*s).port.rxBufferTail;
    (*s).port.txBufferTail = 0 as libc::c_int as uint32_t;
    (*s).port.txBufferHead = (*s).port.txBufferTail;
    // callback works for IRQ-based RX ONLY
    (*s).port.rxCallback = callback;
    (*s).port.rxCallbackData = callbackData;
    (*s).port.mode = mode;
    (*s).port.baudRate = baudRate;
    (*s).port.options = options;
    uartReconfigure(s);
    return s as *mut serialPort_t;
}
#[no_mangle]
pub unsafe extern "C" fn uartSetBaudRate(mut instance: *mut serialPort_t,
                                         mut baudRate: uint32_t) {
    let mut uartPort: *mut uartPort_t = instance as *mut uartPort_t;
    (*uartPort).port.baudRate = baudRate;
    uartReconfigure(uartPort);
}
#[no_mangle]
pub unsafe extern "C" fn uartSetMode(mut instance: *mut serialPort_t,
                                     mut mode: portMode_e) {
    let mut uartPort: *mut uartPort_t = instance as *mut uartPort_t;
    (*uartPort).port.mode = mode;
    uartReconfigure(uartPort);
}
#[no_mangle]
pub unsafe extern "C" fn uartStartTxDMA(mut s: *mut uartPort_t) {
    let mut size: uint16_t = 0 as libc::c_int as uint16_t;
    let mut fromwhere: uint32_t = 0 as libc::c_int as uint32_t;
    let mut state: HAL_UART_StateTypeDef =
        HAL_UART_GetState(&mut (*s).Handle);
    if state as libc::c_uint &
           HAL_UART_STATE_BUSY_TX as libc::c_int as libc::c_uint ==
           HAL_UART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        return
    }
    if (*s).port.txBufferHead > (*s).port.txBufferTail {
        size =
            (*s).port.txBufferHead.wrapping_sub((*s).port.txBufferTail) as
                uint16_t;
        fromwhere = (*s).port.txBufferTail;
        (*s).port.txBufferTail = (*s).port.txBufferHead
    } else {
        size =
            (*s).port.txBufferSize.wrapping_sub((*s).port.txBufferTail) as
                uint16_t;
        fromwhere = (*s).port.txBufferTail;
        (*s).port.txBufferTail = 0 as libc::c_int as uint32_t
    }
    (*s).txDMAEmpty = 0 as libc::c_int != 0;
    //HAL_CLEANCACHE((uint8_t *)&s->port.txBuffer[fromwhere],size);
    HAL_UART_Transmit_DMA(&mut (*s).Handle,
                          &mut *(*s).port.txBuffer.offset(fromwhere as isize)
                              as *mut uint8_t as *mut uint8_t, size);
}
#[no_mangle]
pub unsafe extern "C" fn uartTotalRxBytesWaiting(mut instance:
                                                     *const serialPort_t)
 -> uint32_t {
    let mut s: *mut uartPort_t = instance as *mut uartPort_t;
    if !(*s).rxDMAStream.is_null() {
        let mut rxDMAHead: uint32_t = (*(*(*s).Handle.hdmarx).Instance).NDTR;
        if rxDMAHead >= (*s).rxDMAPos {
            return rxDMAHead.wrapping_sub((*s).rxDMAPos)
        } else {
            return (*s).port.rxBufferSize.wrapping_add(rxDMAHead).wrapping_sub((*s).rxDMAPos)
        }
    }
    if (*s).port.rxBufferHead >= (*s).port.rxBufferTail {
        return (*s).port.rxBufferHead.wrapping_sub((*s).port.rxBufferTail)
    } else {
        return (*s).port.rxBufferSize.wrapping_add((*s).port.rxBufferHead).wrapping_sub((*s).port.rxBufferTail)
    };
}
#[no_mangle]
pub unsafe extern "C" fn uartTotalTxBytesFree(mut instance:
                                                  *const serialPort_t)
 -> uint32_t {
    let mut s: *mut uartPort_t = instance as *mut uartPort_t;
    let mut bytesUsed: uint32_t = 0;
    if (*s).port.txBufferHead >= (*s).port.txBufferTail {
        bytesUsed =
            (*s).port.txBufferHead.wrapping_sub((*s).port.txBufferTail)
    } else {
        bytesUsed =
            (*s).port.txBufferSize.wrapping_add((*s).port.txBufferHead).wrapping_sub((*s).port.txBufferTail)
    }
    if !(*s).txDMAStream.is_null() {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must add
         * the remaining size of that in-progress transfer here instead:
         */
        bytesUsed =
            (bytesUsed as
                 libc::c_uint).wrapping_add((*(*(*s).Handle.hdmatx).Instance).NDTR)
                as uint32_t as uint32_t;
        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if bytesUsed >=
               (*s).port.txBufferSize.wrapping_sub(1 as libc::c_int as
                                                       libc::c_uint) {
            return 0 as libc::c_int as uint32_t
        }
    }
    return (*s).port.txBufferSize.wrapping_sub(1 as libc::c_int as
                                                   libc::c_uint).wrapping_sub(bytesUsed);
}
#[no_mangle]
pub unsafe extern "C" fn isUartTransmitBufferEmpty(mut instance:
                                                       *const serialPort_t)
 -> bool {
    let mut s: *const uartPort_t = instance as *mut uartPort_t;
    if !(*s).txDMAStream.is_null() {
        return (*s).txDMAEmpty
    } else { return (*s).port.txBufferTail == (*s).port.txBufferHead };
}
#[no_mangle]
pub unsafe extern "C" fn uartRead(mut instance: *mut serialPort_t)
 -> uint8_t {
    let mut ch: uint8_t = 0;
    let mut s: *mut uartPort_t = instance as *mut uartPort_t;
    if !(*s).rxDMAStream.is_null() {
        ch =
            *(*s).port.rxBuffer.offset((*s).port.rxBufferSize.wrapping_sub((*s).rxDMAPos)
                                           as isize);
        (*s).rxDMAPos = (*s).rxDMAPos.wrapping_sub(1);
        if (*s).rxDMAPos == 0 as libc::c_int as libc::c_uint {
            (*s).rxDMAPos = (*s).port.rxBufferSize
        }
    } else {
        ch = *(*s).port.rxBuffer.offset((*s).port.rxBufferTail as isize);
        if (*s).port.rxBufferTail.wrapping_add(1 as libc::c_int as
                                                   libc::c_uint) >=
               (*s).port.rxBufferSize {
            (*s).port.rxBufferTail = 0 as libc::c_int as uint32_t
        } else {
            (*s).port.rxBufferTail = (*s).port.rxBufferTail.wrapping_add(1)
        }
    }
    return ch;
}
#[no_mangle]
pub unsafe extern "C" fn uartWrite(mut instance: *mut serialPort_t,
                                   mut ch: uint8_t) {
    let mut s: *mut uartPort_t = instance as *mut uartPort_t;
    ::core::ptr::write_volatile((*s).port.txBuffer.offset((*s).port.txBufferHead
                                                              as isize), ch);
    if (*s).port.txBufferHead.wrapping_add(1 as libc::c_int as libc::c_uint)
           >= (*s).port.txBufferSize {
        (*s).port.txBufferHead = 0 as libc::c_int as uint32_t
    } else { (*s).port.txBufferHead = (*s).port.txBufferHead.wrapping_add(1) }
    if !(*s).txDMAStream.is_null() {
        if (*(*s).txDMAStream).CR & 1 as libc::c_int as libc::c_uint == 0 {
            uartStartTxDMA(s);
        }
    } else {
        if 0x727 as libc::c_uint as uint8_t as libc::c_int >>
               5 as libc::c_uint == 1 as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*s).Handle.Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*s).Handle.Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (1 as libc::c_uint) <<
                                                 (0x727 as libc::c_uint &
                                                      0x1f as libc::c_uint))
                                            as uint32_t as uint32_t)
        } else {
            if 0x727 as libc::c_uint as uint8_t as libc::c_int >>
                   5 as libc::c_uint == 2 as libc::c_int {
                ::core::ptr::write_volatile(&mut (*(*s).Handle.Instance).CR2
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*s).Handle.Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (1 as libc::c_uint) <<
                                                     (0x727 as libc::c_uint &
                                                          0x1f as
                                                              libc::c_uint))
                                                as uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*s).Handle.Instance).CR3
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*s).Handle.Instance).CR3
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (1 as libc::c_uint) <<
                                                     (0x727 as libc::c_uint &
                                                          0x1f as
                                                              libc::c_uint))
                                                as uint32_t as uint32_t)
            };
        };
    };
}
#[no_mangle]
pub static mut uartVTable: [serialPortVTable; 1] =
    unsafe {
        [{
             let mut init =
                 serialPortVTable{serialWrite:
                                      Some(uartWrite as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint8_t)
                                                   -> ()),
                                  serialTotalRxWaiting:
                                      Some(uartTotalRxBytesWaiting as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialTotalTxFree:
                                      Some(uartTotalTxBytesFree as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialRead:
                                      Some(uartRead as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> uint8_t),
                                  serialSetBaudRate:
                                      Some(uartSetBaudRate as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint32_t)
                                                   -> ()),
                                  isSerialTransmitBufferEmpty:
                                      Some(isUartTransmitBufferEmpty as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> bool),
                                  setMode:
                                      Some(uartSetMode as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        portMode_e)
                                                   -> ()),
                                  setCtrlLineStateCb: None,
                                  setBaudRateCb: None,
                                  writeBuf: None,
                                  beginWrite: None,
                                  endWrite: None,};
             init
         }]
    };
// USART1 Rx/Tx IRQ Handler
#[no_mangle]
pub unsafe extern "C" fn USART1_IRQHandler() {
    let mut s: *mut uartPort_t =
        &mut (**uartDevmap.as_mut_ptr().offset(UARTDEV_1 as libc::c_int as
                                                   isize)).port;
    uartIrqHandler(s);
}
// USART2 Rx/Tx IRQ Handler
#[no_mangle]
pub unsafe extern "C" fn USART2_IRQHandler() {
    let mut s: *mut uartPort_t =
        &mut (**uartDevmap.as_mut_ptr().offset(UARTDEV_2 as libc::c_int as
                                                   isize)).port;
    uartIrqHandler(s);
}
// USART3 Rx/Tx IRQ Handler
#[no_mangle]
pub unsafe extern "C" fn USART3_IRQHandler() {
    let mut s: *mut uartPort_t =
        &mut (**uartDevmap.as_mut_ptr().offset(UARTDEV_3 as libc::c_int as
                                                   isize)).port;
    uartIrqHandler(s);
}
// UART4 Rx/Tx IRQ Handler
#[no_mangle]
pub unsafe extern "C" fn UART4_IRQHandler() {
    let mut s: *mut uartPort_t =
        &mut (**uartDevmap.as_mut_ptr().offset(UARTDEV_4 as libc::c_int as
                                                   isize)).port;
    uartIrqHandler(s);
}
