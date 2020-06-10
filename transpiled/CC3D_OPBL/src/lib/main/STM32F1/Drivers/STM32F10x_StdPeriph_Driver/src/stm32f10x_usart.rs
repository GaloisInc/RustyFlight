use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_GetClocksFreq(RCC_Clocks: *mut RCC_ClocksTypeDef);
    /* STM32F10X_CL */
    #[no_mangle]
    fn RCC_APB2PeriphResetCmd(RCC_APB2Periph: uint32_t,
                              NewState: FunctionalState);
    #[no_mangle]
    fn RCC_APB1PeriphResetCmd(RCC_APB1Periph: uint32_t,
                              NewState: FunctionalState);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type u16_0 = uint16_t;
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type ITStatus = FlagStatus;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_TypeDef {
    pub SR: uint16_t,
    pub RESERVED0: uint16_t,
    pub DR: uint16_t,
    pub RESERVED1: uint16_t,
    pub BRR: uint16_t,
    pub RESERVED2: uint16_t,
    pub CR1: uint16_t,
    pub RESERVED3: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED4: uint16_t,
    pub CR3: uint16_t,
    pub RESERVED5: uint16_t,
    pub GTPR: uint16_t,
    pub RESERVED6: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f10x_rcc.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the RCC firmware 
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup RCC
  * @{
  */
/* * @defgroup RCC_Exported_Types
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_ClocksTypeDef {
    pub SYSCLK_Frequency: uint32_t,
    pub HCLK_Frequency: uint32_t,
    pub PCLK1_Frequency: uint32_t,
    pub PCLK2_Frequency: uint32_t,
    pub ADCCLK_Frequency: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_InitTypeDef {
    pub USART_BaudRate: uint32_t,
    pub USART_WordLength: uint16_t,
    pub USART_StopBits: uint16_t,
    pub USART_Parity: uint16_t,
    pub USART_Mode: uint16_t,
    pub USART_HardwareFlowControl: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f10x_usart.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the USART 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup USART
  * @{
  */
/* * @defgroup USART_Exported_Types
  * @{
  */
/* * 
  * @brief  USART Init Structure definition  
  */
/* !< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */
/* !< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_Word_Length */
/* !< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_Stop_Bits */
/* !< Specifies the parity mode.
                                           This parameter can be a value of @ref USART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */
/* !< Specifies wether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_Mode */
/* !< Specifies wether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control */
/* * 
  * @brief  USART Clock Init Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_ClockInitTypeDef {
    pub USART_Clock: uint16_t,
    pub USART_CPOL: uint16_t,
    pub USART_CPHA: uint16_t,
    pub USART_LastBit: uint16_t,
}
/* USART ONEBITE mode Disable Mask */
/* *
  * @}
  */
/* * @defgroup USART_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the USARTx peripheral registers to their default reset values.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values: 
  *      USART1, USART2, USART3, UART4 or UART5.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_DeInit(mut USARTx: *mut USART_TypeDef) {
    /* Check the parameters */
    if USARTx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x3800
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut USART_TypeDef {
        RCC_APB2PeriphResetCmd(0x4000 as libc::c_int as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x4000 as libc::c_int as uint32_t, DISABLE);
    } else if USARTx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x4400 as libc::c_int as
                                                  libc::c_uint) as
                      *mut USART_TypeDef {
        RCC_APB1PeriphResetCmd(0x20000 as libc::c_int as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x20000 as libc::c_int as uint32_t, DISABLE);
    } else if USARTx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x4800 as libc::c_int as
                                                  libc::c_uint) as
                      *mut USART_TypeDef {
        RCC_APB1PeriphResetCmd(0x40000 as libc::c_int as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x40000 as libc::c_int as uint32_t, DISABLE);
    } else if USARTx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x4c00 as libc::c_int as
                                                  libc::c_uint) as
                      *mut USART_TypeDef {
        RCC_APB1PeriphResetCmd(0x80000 as libc::c_int as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x80000 as libc::c_int as uint32_t, DISABLE);
    } else if USARTx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x5000 as libc::c_int as
                                                  libc::c_uint) as
                      *mut USART_TypeDef {
        RCC_APB1PeriphResetCmd(0x100000 as libc::c_int as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x100000 as libc::c_int as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Initializes the USARTx peripheral according to the specified
  *         parameters in the USART_InitStruct .
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *         that contains the configuration information for the specified USART 
  *         peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_Init(mut USARTx: *mut USART_TypeDef,
                                    mut USART_InitStruct:
                                        *mut USART_InitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut apbclock: uint32_t = 0 as libc::c_int as uint32_t;
    let mut integerdivider: uint32_t = 0 as libc::c_int as uint32_t;
    let mut fractionaldivider: uint32_t = 0 as libc::c_int as uint32_t;
    let mut usartxbase: uint32_t = 0 as libc::c_int as uint32_t;
    let mut RCC_ClocksStatus: RCC_ClocksTypeDef =
        RCC_ClocksTypeDef{SYSCLK_Frequency: 0,
                          HCLK_Frequency: 0,
                          PCLK1_Frequency: 0,
                          PCLK2_Frequency: 0,
                          ADCCLK_Frequency: 0,};
    /* Check the parameters */
    /* The hardware flow control is available only for USART1, USART2 and USART3 */
    ((*USART_InitStruct).USART_HardwareFlowControl as libc::c_int) !=
        0 as libc::c_int as uint16_t as libc::c_int;
    usartxbase = USARTx as uint32_t;
    /*---------------------------- USART CR2 Configuration -----------------------*/
    tmpreg = (*USARTx).CR2 as uint32_t;
    /* Clear STOP[13:12] bits */
    tmpreg &= 0xcfff as libc::c_int as uint16_t as libc::c_uint;
    /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to USART_StopBits value */
    tmpreg |= (*USART_InitStruct).USART_StopBits as uint32_t;
    /* Write to USART CR2 */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint16_t,
                                tmpreg as uint16_t);
    /*---------------------------- USART CR1 Configuration -----------------------*/
    tmpreg = (*USARTx).CR1 as uint32_t;
    /* Clear M, PCE, PS, TE and RE bits */
    tmpreg &= 0xe9f3 as libc::c_int as uint16_t as libc::c_uint;
    /* Configure the USART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to USART_WordLength value */
  /* Set PCE and PS bits according to USART_Parity value */
  /* Set TE and RE bits according to USART_Mode value */
    tmpreg |=
        (*USART_InitStruct).USART_WordLength as uint32_t |
            (*USART_InitStruct).USART_Parity as libc::c_uint |
            (*USART_InitStruct).USART_Mode as libc::c_uint;
    /* Write to USART CR1 */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                tmpreg as uint16_t);
    /*---------------------------- USART CR3 Configuration -----------------------*/
    tmpreg = (*USARTx).CR3 as uint32_t;
    /* Clear CTSE and RTSE bits */
    tmpreg &= 0xfcff as libc::c_int as uint16_t as libc::c_uint;
    /* Configure the USART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
    tmpreg |= (*USART_InitStruct).USART_HardwareFlowControl as libc::c_uint;
    /* Write to USART CR3 */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                tmpreg as uint16_t);
    /*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate -------------------------------------------*/
    RCC_GetClocksFreq(&mut RCC_ClocksStatus);
    if usartxbase ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x3800
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
       {
        apbclock = RCC_ClocksStatus.PCLK2_Frequency
    } else { apbclock = RCC_ClocksStatus.PCLK1_Frequency }
    /* Determine the integer part */
    if (*USARTx).CR1 as libc::c_int &
           0x8000 as libc::c_int as u16_0 as libc::c_int != 0 as libc::c_int {
        /* Integer part computing in case Oversampling mode is 8 Samples */
        integerdivider =
            (25 as libc::c_int as
                 libc::c_uint).wrapping_mul(apbclock).wrapping_div((2 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_mul((*USART_InitStruct).USART_BaudRate))
    } else {
        /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
        /* Integer part computing in case Oversampling mode is 16 Samples */
        integerdivider =
            (25 as libc::c_int as
                 libc::c_uint).wrapping_mul(apbclock).wrapping_div((4 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_mul((*USART_InitStruct).USART_BaudRate))
    }
    tmpreg =
        integerdivider.wrapping_div(100 as libc::c_int as libc::c_uint) <<
            4 as libc::c_int;
    /* Determine the fractional part */
    fractionaldivider =
        integerdivider.wrapping_sub((100 as libc::c_int as
                                         libc::c_uint).wrapping_mul(tmpreg >>
                                                                        4 as
                                                                            libc::c_int));
    /* Implement the fractional part in the register */
    if (*USARTx).CR1 as libc::c_int &
           0x8000 as libc::c_int as u16_0 as libc::c_int != 0 as libc::c_int {
        tmpreg |=
            fractionaldivider.wrapping_mul(8 as libc::c_int as
                                               libc::c_uint).wrapping_add(50
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint).wrapping_div(100
                                                                                                             as
                                                                                                             libc::c_int
                                                                                                             as
                                                                                                             libc::c_uint)
                & 0x7 as libc::c_int as uint8_t as libc::c_uint
    } else {
        /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
        tmpreg |=
            fractionaldivider.wrapping_mul(16 as libc::c_int as
                                               libc::c_uint).wrapping_add(50
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint).wrapping_div(100
                                                                                                             as
                                                                                                             libc::c_int
                                                                                                             as
                                                                                                             libc::c_uint)
                & 0xf as libc::c_int as uint8_t as libc::c_uint
    }
    /* Write to USART BRR */
    ::core::ptr::write_volatile(&mut (*USARTx).BRR as *mut uint16_t,
                                tmpreg as uint16_t);
}
/* *
  * @brief  Fills each USART_InitStruct member with its default value.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_StructInit(mut USART_InitStruct:
                                              *mut USART_InitTypeDef) {
    /* USART_InitStruct members default value */
    (*USART_InitStruct).USART_BaudRate = 9600 as libc::c_int as uint32_t;
    (*USART_InitStruct).USART_WordLength = 0 as libc::c_int as uint16_t;
    (*USART_InitStruct).USART_StopBits = 0 as libc::c_int as uint16_t;
    (*USART_InitStruct).USART_Parity = 0 as libc::c_int as uint16_t;
    (*USART_InitStruct).USART_Mode =
        (0x4 as libc::c_int as uint16_t as libc::c_int |
             0x8 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    (*USART_InitStruct).USART_HardwareFlowControl =
        0 as libc::c_int as uint16_t;
}
/* *
  * @brief  Initializes the USARTx peripheral Clock according to the 
  *          specified parameters in the USART_ClockInitStruct .
  * @param  USARTx: where x can be 1, 2, 3 to select the USART peripheral.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *         structure that contains the configuration information for the specified 
  *         USART peripheral.  
  * @note The Smart Card and Synchronous modes are not available for UART4 and UART5.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ClockInit(mut USARTx: *mut USART_TypeDef,
                                         mut USART_ClockInitStruct:
                                             *mut USART_ClockInitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*---------------------------- USART CR2 Configuration -----------------------*/
    tmpreg = (*USARTx).CR2 as uint32_t;
    /* Clear CLKEN, CPOL, CPHA and LBCL bits */
    tmpreg &= 0xf0ff as libc::c_int as uint16_t as libc::c_uint;
    /* Configure the USART Clock, CPOL, CPHA and LastBit ------------*/
  /* Set CLKEN bit according to USART_Clock value */
  /* Set CPOL bit according to USART_CPOL value */
  /* Set CPHA bit according to USART_CPHA value */
  /* Set LBCL bit according to USART_LastBit value */
    tmpreg |=
        (*USART_ClockInitStruct).USART_Clock as uint32_t |
            (*USART_ClockInitStruct).USART_CPOL as libc::c_uint |
            (*USART_ClockInitStruct).USART_CPHA as libc::c_uint |
            (*USART_ClockInitStruct).USART_LastBit as libc::c_uint;
    /* Write to USART CR2 */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint16_t,
                                tmpreg as uint16_t);
}
/* *
  * @brief  Fills each USART_ClockInitStruct member with its default value.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ClockStructInit(mut USART_ClockInitStruct:
                                                   *mut USART_ClockInitTypeDef) {
    /* USART_ClockInitStruct members default value */
    (*USART_ClockInitStruct).USART_Clock = 0 as libc::c_int as uint16_t;
    (*USART_ClockInitStruct).USART_CPOL = 0 as libc::c_int as uint16_t;
    (*USART_ClockInitStruct).USART_CPHA = 0 as libc::c_int as uint16_t;
    (*USART_ClockInitStruct).USART_LastBit = 0 as libc::c_int as uint16_t;
}
/* *
  * @brief  Enables or disables the specified USART peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *         This parameter can be one of the following values:
  *           USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USARTx peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_Cmd(mut USARTx: *mut USART_TypeDef,
                                   mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected USART by setting the UE bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x2000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected USART by clearing the UE bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xdfff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified USART interrupts.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Transmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  *     @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @param  NewState: new state of the specified USARTx interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ITConfig(mut USARTx: *mut USART_TypeDef,
                                        mut USART_IT: uint16_t,
                                        mut NewState: FunctionalState) {
    let mut usartreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itpos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itmask: uint32_t = 0 as libc::c_int as uint32_t;
    let mut usartxbase: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* The CTS interrupt is not available for UART4 and UART5 */
    (USART_IT as libc::c_int) ==
        0x96a as libc::c_int as uint16_t as libc::c_int;
    usartxbase = USARTx as uint32_t;
    /* Get the USART register index */
    usartreg =
        (USART_IT as uint8_t as libc::c_int >> 0x5 as libc::c_int) as
            uint32_t;
    /* Get the interrupt position */
    itpos =
        (USART_IT as libc::c_int &
             0x1f as libc::c_int as uint16_t as libc::c_int) as uint32_t;
    itmask = (0x1 as libc::c_int as uint32_t) << itpos;
    if usartreg == 0x1 as libc::c_int as libc::c_uint {
        /* The IT is in CR1 register */
        usartxbase =
            (usartxbase as
                 libc::c_uint).wrapping_add(0xc as libc::c_int as
                                                libc::c_uint) as uint32_t as
                uint32_t
    } else if usartreg == 0x2 as libc::c_int as libc::c_uint {
        /* The IT is in CR2 register */
        usartxbase =
            (usartxbase as
                 libc::c_uint).wrapping_add(0x10 as libc::c_int as
                                                libc::c_uint) as uint32_t as
                uint32_t
    } else {
        /* The IT is in CR3 register */
        usartxbase =
            (usartxbase as
                 libc::c_uint).wrapping_add(0x14 as libc::c_int as
                                                libc::c_uint) as uint32_t as
                uint32_t
    }
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh0 = *(usartxbase as *mut uint32_t);
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | itmask) as uint32_t
                                        as uint32_t)
    } else {
        let ref mut fresh1 = *(usartxbase as *mut uint32_t);
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !itmask) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the USART�s DMA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_DMAReq: specifies the DMA request.
  *   This parameter can be any combination of the following values:
  *     @arg USART_DMAReq_Tx: USART DMA transmit request
  *     @arg USART_DMAReq_Rx: USART DMA receive request
  * @param  NewState: new state of the DMA Request sources.
  *   This parameter can be: ENABLE or DISABLE.
  * @note The DMA mode is not available for UART5 except in the STM32
  *       High density value line devices(STM32F10X_HD_VL).  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_DMACmd(mut USARTx: *mut USART_TypeDef,
                                      mut USART_DMAReq: uint16_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the DMA transfer for selected requests by setting the DMAT and/or
       DMAR bits in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         USART_DMAReq as libc::c_int) as
                                        uint16_t as uint16_t)
    } else {
        /* Disable the DMA transfer for selected requests by clearing the DMAT and/or
       DMAR bits in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(USART_DMAReq as libc::c_int) as
                                             uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    };
}
/* *
  * @brief  Sets the address of the USART node.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_Address: Indicates the address of the USART node.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetAddress(mut USARTx: *mut USART_TypeDef,
                                          mut USART_Address: uint8_t) {
    /* Check the parameters */
    /* Clear the USART address */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xfff0 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Set the USART address node */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     USART_Address as libc::c_int) as uint16_t
                                    as uint16_t);
}
/* *
  * @brief  Selects the USART WakeUp method.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_WakeUp: specifies the USART wakeup method.
  *   This parameter can be one of the following values:
  *     @arg USART_WakeUp_IdleLine: WakeUp by an idle line detection
  *     @arg USART_WakeUp_AddressMark: WakeUp by an address mark
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_WakeUpConfig(mut USARTx: *mut USART_TypeDef,
                                            mut USART_WakeUp: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xf7ff as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     USART_WakeUp as libc::c_int) as uint16_t
                                    as uint16_t);
}
/* *
  * @brief  Determines if the USART is in mute mode or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART mute mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ReceiverWakeUpCmd(mut USARTx:
                                                     *mut USART_TypeDef,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the USART mute mode  by setting the RWU bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x2 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the USART mute mode by clearing the RWU bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfffd as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Sets the USART LIN Break detection length.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_LINBreakDetectLength: specifies the LIN break detection length.
  *   This parameter can be one of the following values:
  *     @arg USART_LINBreakDetectLength_10b: 10-bit break detection
  *     @arg USART_LINBreakDetectLength_11b: 11-bit break detection
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_LINBreakDetectLengthConfig(mut USARTx:
                                                              *mut USART_TypeDef,
                                                          mut USART_LINBreakDetectLength:
                                                              uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xffdf as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     USART_LINBreakDetectLength as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
}
/* *
  * @brief  Enables or disables the USART�s LIN mode.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART LIN mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_LINCmd(mut USARTx: *mut USART_TypeDef,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the LIN mode by setting the LINEN bit in the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x4000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the LIN mode by clearing the LINEN bit in the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xbfff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  Data: the data to transmit.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SendData(mut USARTx: *mut USART_TypeDef,
                                        mut Data: uint16_t) {
    /* Check the parameters */
    /* Transmit Data */
    ::core::ptr::write_volatile(&mut (*USARTx).DR as *mut uint16_t,
                                (Data as libc::c_int &
                                     0x1ff as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t);
}
/* *
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval The received data.
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ReceiveData(mut USARTx: *mut USART_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    /* Receive Data */
    return ((*USARTx).DR as libc::c_int &
                0x1ff as libc::c_int as uint16_t as libc::c_int) as uint16_t;
}
/* *
  * @brief  Transmits break characters.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SendBreak(mut USARTx: *mut USART_TypeDef) {
    /* Check the parameters */
    /* Send break characters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     0x1 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
}
/* *
  * @brief  Sets the specified USART guard time.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  USART_GuardTime: specifies the guard time.
  * @note The guard time bits are not available for UART4 and UART5.   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetGuardTime(mut USARTx: *mut USART_TypeDef,
                                            mut USART_GuardTime: uint8_t) {
    /* Check the parameters */
    /* Clear the USART Guard time */
    ::core::ptr::write_volatile(&mut (*USARTx).GTPR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).GTPR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xff as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Set the USART guard time */
    ::core::ptr::write_volatile(&mut (*USARTx).GTPR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).GTPR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     ((USART_GuardTime as uint16_t as
                                           libc::c_int) << 0x8 as libc::c_int)
                                         as uint16_t as libc::c_int) as
                                    uint16_t as uint16_t);
}
/* *
  * @brief  Sets the system clock prescaler.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_Prescaler: specifies the prescaler clock.  
  * @note   The function is used for IrDA mode with UART4 and UART5.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetPrescaler(mut USARTx: *mut USART_TypeDef,
                                            mut USART_Prescaler: uint8_t) {
    /* Check the parameters */
    /* Clear the USART prescaler */
    ::core::ptr::write_volatile(&mut (*USARTx).GTPR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).GTPR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xff00 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Set the USART prescaler */
    ::core::ptr::write_volatile(&mut (*USARTx).GTPR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).GTPR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     USART_Prescaler as libc::c_int) as
                                    uint16_t as uint16_t);
}
/* *
  * @}
  */
/* * @defgroup USART_Exported_Constants
  * @{
  */
/* * @defgroup USART_Word_Length 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Stop_Bits 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Parity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Hardware_Flow_Control 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Clock 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Clock_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Clock_Phase
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Last_Bit
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Interrupt_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_DMA_Requests 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_WakeUp_methods
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_LIN_Break_Detection_Length 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_IrDA_Low_Power 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Flags 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup USART_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Exported_Functions
  * @{
  */
/* *
  * @brief  Enables or disables the USART�s Smart Card mode.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  NewState: new state of the Smart Card mode.
  *   This parameter can be: ENABLE or DISABLE.     
  * @note The Smart Card mode is not available for UART4 and UART5. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SmartCardCmd(mut USARTx: *mut USART_TypeDef,
                                            mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the SC mode by setting the SCEN bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x20 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the SC mode by clearing the SCEN bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xffdf as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables NACK transmission.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral. 
  * @param  NewState: new state of the NACK transmission.
  *   This parameter can be: ENABLE or DISABLE.  
  * @note The Smart Card mode is not available for UART4 and UART5.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SmartCardNACKCmd(mut USARTx:
                                                    *mut USART_TypeDef,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the NACK transmission by setting the NACK bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x10 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the NACK transmission by clearing the NACK bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xffef as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the USART�s Half Duplex communication.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART Communication.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_HalfDuplexCmd(mut USARTx: *mut USART_TypeDef,
                                             mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the Half-Duplex mode by setting the HDSEL bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x8 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the Half-Duplex mode by clearing the HDSEL bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfff7 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the USART's 8x oversampling mode.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART one bit sampling method.
  *   This parameter can be: ENABLE or DISABLE.
  * @note
  *     This function has to be called before calling USART_Init()
  *     function in order to have correct baudrate Divider value.   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_OverSampling8Cmd(mut USARTx:
                                                    *mut USART_TypeDef,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the 8x Oversampling mode by setting the OVER8 bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x8000 as libc::c_int as u16_0 as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the 8x Oversampling mode by clearing the OVER8 bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0x7fff as libc::c_int as u16_0 as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the USART's one bit sampling method.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART one bit sampling method.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_OneBitMethodCmd(mut USARTx: *mut USART_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the one bit method by setting the ONEBITE bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x800 as libc::c_int as u16_0 as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable tthe one bit method by clearing the ONEBITE bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xf7ff as libc::c_int as u16_0 as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Configures the USART's IrDA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IrDAMode: specifies the IrDA mode.
  *   This parameter can be one of the following values:
  *     @arg USART_IrDAMode_LowPower
  *     @arg USART_IrDAMode_Normal
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_IrDAConfig(mut USARTx: *mut USART_TypeDef,
                                          mut USART_IrDAMode: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xfffb as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     USART_IrDAMode as libc::c_int) as
                                    uint16_t as uint16_t);
}
/* *
  * @brief  Enables or disables the USART's IrDA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the IrDA mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_IrDACmd(mut USARTx: *mut USART_TypeDef,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the IrDA mode by setting the IREN bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x2 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the IrDA mode by clearing the IREN bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfffd as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Checks whether the specified USART flag is set or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5)
  *     @arg USART_FLAG_LBD:  LIN Break detection flag
  *     @arg USART_FLAG_TXE:  Transmit data register empty flag
  *     @arg USART_FLAG_TC:   Transmission Complete flag
  *     @arg USART_FLAG_RXNE: Receive data register not empty flag
  *     @arg USART_FLAG_IDLE: Idle Line detection flag
  *     @arg USART_FLAG_ORE:  OverRun Error flag
  *     @arg USART_FLAG_NE:   Noise Error flag
  *     @arg USART_FLAG_FE:   Framing Error flag
  *     @arg USART_FLAG_PE:   Parity Error flag
  * @retval The new state of USART_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn USART_GetFlagStatus(mut USARTx: *mut USART_TypeDef,
                                             mut USART_FLAG: uint16_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* The CTS flag is not available for UART4 and UART5 */
    (USART_FLAG as libc::c_int) ==
        0x200 as libc::c_int as uint16_t as libc::c_int;
    if (*USARTx).SR as libc::c_int & USART_FLAG as libc::c_int !=
           RESET as libc::c_int as uint16_t as libc::c_int {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the USARTx's pending flags.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5).
  *     @arg USART_FLAG_LBD:  LIN Break detection flag.
  *     @arg USART_FLAG_TC:   Transmission Complete flag.
  *     @arg USART_FLAG_RXNE: Receive data register not empty flag.
  *   
  * @note
  *   - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun 
  *     error) and IDLE (Idle line detected) flags are cleared by software 
  *     sequence: a read operation to USART_SR register (USART_GetFlagStatus()) 
  *     followed by a read operation to USART_DR register (USART_ReceiveData()).
  *   - RXNE flag can be also cleared by a read to the USART_DR register 
  *     (USART_ReceiveData()).
  *   - TC flag can be also cleared by software sequence: a read operation to 
  *     USART_SR register (USART_GetFlagStatus()) followed by a write operation
  *     to USART_DR register (USART_SendData()).
  *   - TXE flag is cleared only by a write to the USART_DR register 
  *     (USART_SendData()).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ClearFlag(mut USARTx: *mut USART_TypeDef,
                                         mut USART_FLAG: uint16_t) {
    /* Check the parameters */
    /* The CTS flag is not available for UART4 and UART5 */
    (USART_FLAG as libc::c_int &
         0x200 as libc::c_int as uint16_t as libc::c_int) ==
        0x200 as libc::c_int as uint16_t as libc::c_int;
    ::core::ptr::write_volatile(&mut (*USARTx).SR as *mut uint16_t,
                                !(USART_FLAG as libc::c_int) as uint16_t);
}
/* *
  * @brief  Checks whether the specified USART interrupt has occurred or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Tansmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_ORE:  OverRun Error interrupt
  *     @arg USART_IT_NE:   Noise Error interrupt
  *     @arg USART_IT_FE:   Framing Error interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  * @retval The new state of USART_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn USART_GetITStatus(mut USARTx: *mut USART_TypeDef,
                                           mut USART_IT: uint16_t)
 -> ITStatus {
    let mut bitpos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itmask: uint32_t = 0 as libc::c_int as uint32_t;
    let mut usartreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut bitstatus: ITStatus = RESET;
    /* Check the parameters */
    /* The CTS interrupt is not available for UART4 and UART5 */
    (USART_IT as libc::c_int) ==
        0x96a as libc::c_int as uint16_t as libc::c_int;
    /* Get the USART register index */
    usartreg =
        (USART_IT as uint8_t as libc::c_int >> 0x5 as libc::c_int) as
            uint32_t;
    /* Get the interrupt position */
    itmask =
        (USART_IT as libc::c_int &
             0x1f as libc::c_int as uint16_t as libc::c_int) as uint32_t;
    itmask = (0x1 as libc::c_int as uint32_t) << itmask;
    if usartreg == 0x1 as libc::c_int as libc::c_uint {
        /* The IT  is in CR1 register */
        itmask &= (*USARTx).CR1 as libc::c_uint
    } else if usartreg == 0x2 as libc::c_int as libc::c_uint {
        /* The IT  is in CR2 register */
        itmask &= (*USARTx).CR2 as libc::c_uint
    } else {
        /* The IT  is in CR3 register */
        itmask &= (*USARTx).CR3 as libc::c_uint
    }
    bitpos = (USART_IT as libc::c_int >> 0x8 as libc::c_int) as uint32_t;
    bitpos = (0x1 as libc::c_int as uint32_t) << bitpos;
    bitpos &= (*USARTx).SR as libc::c_uint;
    if itmask != RESET as libc::c_int as uint16_t as libc::c_uint &&
           bitpos != RESET as libc::c_int as uint16_t as libc::c_uint {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the USARTx's interrupt pending bits.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt. 
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt.
  *   
  * @note
  *   - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun 
  *     error) and IDLE (Idle line detected) pending bits are cleared by 
  *     software sequence: a read operation to USART_SR register 
  *     (USART_GetITStatus()) followed by a read operation to USART_DR register 
  *     (USART_ReceiveData()).
  *   - RXNE pending bit can be also cleared by a read to the USART_DR register 
  *     (USART_ReceiveData()).
  *   - TC pending bit can be also cleared by software sequence: a read 
  *     operation to USART_SR register (USART_GetITStatus()) followed by a write 
  *     operation to USART_DR register (USART_SendData()).
  *   - TXE pending bit is cleared only by a write to the USART_DR register 
  *     (USART_SendData()).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ClearITPendingBit(mut USARTx:
                                                     *mut USART_TypeDef,
                                                 mut USART_IT: uint16_t) {
    let mut bitpos: uint16_t = 0 as libc::c_int as uint16_t;
    let mut itmask: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* The CTS interrupt is not available for UART4 and UART5 */
    (USART_IT as libc::c_int) ==
        0x96a as libc::c_int as uint16_t as libc::c_int;
    bitpos = (USART_IT as libc::c_int >> 0x8 as libc::c_int) as uint16_t;
    itmask =
        ((0x1 as libc::c_int as uint16_t as libc::c_int) <<
             bitpos as libc::c_int) as uint16_t;
    ::core::ptr::write_volatile(&mut (*USARTx).SR as *mut uint16_t,
                                !(itmask as libc::c_int) as uint16_t);
}
/* ****************** (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
