use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_GetClocksFreq(RCC_Clocks: *mut RCC_ClocksTypeDef);
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
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub CR3: uint32_t,
    pub BRR: uint16_t,
    pub RESERVED1: uint16_t,
    pub GTPR: uint16_t,
    pub RESERVED2: uint16_t,
    pub RTOR: uint32_t,
    pub RQR: uint16_t,
    pub RESERVED3: uint16_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub RDR: uint16_t,
    pub RESERVED4: uint16_t,
    pub TDR: uint16_t,
    pub RESERVED5: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_rcc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the RCC 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup RCC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_ClocksTypeDef {
    pub SYSCLK_Frequency: uint32_t,
    pub HCLK_Frequency: uint32_t,
    pub PCLK1_Frequency: uint32_t,
    pub PCLK2_Frequency: uint32_t,
    pub ADC12CLK_Frequency: uint32_t,
    pub ADC34CLK_Frequency: uint32_t,
    pub I2C1CLK_Frequency: uint32_t,
    pub I2C2CLK_Frequency: uint32_t,
    pub I2C3CLK_Frequency: uint32_t,
    pub TIM1CLK_Frequency: uint32_t,
    pub HRTIM1CLK_Frequency: uint32_t,
    pub TIM8CLK_Frequency: uint32_t,
    pub USART1CLK_Frequency: uint32_t,
    pub USART2CLK_Frequency: uint32_t,
    pub USART3CLK_Frequency: uint32_t,
    pub UART4CLK_Frequency: uint32_t,
    pub UART5CLK_Frequency: uint32_t,
    pub TIM15CLK_Frequency: uint32_t,
    pub TIM16CLK_Frequency: uint32_t,
    pub TIM17CLK_Frequency: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_InitTypeDef {
    pub USART_BaudRate: uint32_t,
    pub USART_WordLength: uint32_t,
    pub USART_StopBits: uint32_t,
    pub USART_Parity: uint32_t,
    pub USART_Mode: uint32_t,
    pub USART_HardwareFlowControl: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_ClockInitTypeDef {
    pub USART_Clock: uint32_t,
    pub USART_CPOL: uint32_t,
    pub USART_CPHA: uint32_t,
    pub USART_LastBit: uint32_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup USART_Private_Functions
  * @{
  */
/* * @defgroup USART_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions 
 *
@verbatim 
 ===============================================================================
           ##### Initialization and Configuration functions #####
 ===============================================================================  
  [..]
      This subsection provides a set of functions allowing to initialize the USART 
      in asynchronous and in synchronous modes.
       (+) For the asynchronous mode only these parameters can be configured: 
            (++) Baud Rate.
            (++) Word Length. 
            (++) Stop Bit.
            (++) Parity: If the parity is enabled, then the MSB bit of the data written
                 in the data register is transmitted but is changed by the parity bit.
                 Depending on the frame length defined by the M bit (8-bits or 9-bits),
                 the possible USART frame formats are as listed in the following table:
    [..]
   +-------------------------------------------------------------+     
   |   M bit |  PCE bit  |            USART frame                |
   |---------------------|---------------------------------------|             
   |    0    |    0      |    | SB | 8 bit data | STB |          |
   |---------|-----------|---------------------------------------|  
   |    0    |    1      |    | SB | 7 bit data | PB | STB |     |
   |---------|-----------|---------------------------------------|  
   |    1    |    0      |    | SB | 9 bit data | STB |          |
   |---------|-----------|---------------------------------------|  
   |    1    |    1      |    | SB | 8 bit data | PB | STB |     |
   +-------------------------------------------------------------+            
    [..]
           (++) Hardware flow control.
           (++) Receiver/transmitter modes.
    [..] The USART_Init() function follows the USART  asynchronous configuration 
         procedure(details for the procedure are available in reference manual.
        (+) For the synchronous mode in addition to the asynchronous mode parameters
            these parameters should be also configured:
            (++) USART Clock Enabled.
            (++) USART polarity.
            (++) USART phase.
            (++) USART LastBit.
    [..] These parameters can be configured using the USART_ClockInit() function.

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the USARTx peripheral registers to their default reset values.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
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
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *         that contains the configuration information for the specified USART peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_Init(mut USARTx: *mut USART_TypeDef,
                                    mut USART_InitStruct:
                                        *mut USART_InitTypeDef) {
    let mut divider: uint32_t = 0 as libc::c_int as uint32_t;
    let mut apbclock: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut RCC_ClocksStatus: RCC_ClocksTypeDef =
        RCC_ClocksTypeDef{SYSCLK_Frequency: 0,
                          HCLK_Frequency: 0,
                          PCLK1_Frequency: 0,
                          PCLK2_Frequency: 0,
                          ADC12CLK_Frequency: 0,
                          ADC34CLK_Frequency: 0,
                          I2C1CLK_Frequency: 0,
                          I2C2CLK_Frequency: 0,
                          I2C3CLK_Frequency: 0,
                          TIM1CLK_Frequency: 0,
                          HRTIM1CLK_Frequency: 0,
                          TIM8CLK_Frequency: 0,
                          USART1CLK_Frequency: 0,
                          USART2CLK_Frequency: 0,
                          USART3CLK_Frequency: 0,
                          UART4CLK_Frequency: 0,
                          UART5CLK_Frequency: 0,
                          TIM15CLK_Frequency: 0,
                          TIM16CLK_Frequency: 0,
                          TIM17CLK_Frequency: 0,};
    /* Check the parameters */
    /* Disable USART */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    /*---------------------------- USART CR2 Configuration -----------------------*/
    tmpreg = (*USARTx).CR2;
    /* Clear STOP[13:12] bits */
    tmpreg &= !(0x3000 as libc::c_int as uint32_t);
    /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to USART_StopBits value */
    tmpreg |= (*USART_InitStruct).USART_StopBits;
    /* Write to USART CR2 */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t, tmpreg);
    /*---------------------------- USART CR1 Configuration -----------------------*/
    tmpreg = (*USARTx).CR1;
    /* Clear M, PCE, PS, TE and RE bits */
    tmpreg &=
        !(0x1000 as libc::c_int as uint32_t | 0x400 as libc::c_int as uint32_t
              | 0x200 as libc::c_int as uint32_t |
              0x8 as libc::c_int as uint32_t |
              0x4 as libc::c_int as uint32_t);
    /* Configure the USART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to USART_WordLength value */
  /* Set PCE and PS bits according to USART_Parity value */
  /* Set TE and RE bits according to USART_Mode value */
    tmpreg |=
        (*USART_InitStruct).USART_WordLength |
            (*USART_InitStruct).USART_Parity | (*USART_InitStruct).USART_Mode;
    /* Write to USART CR1 */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t, tmpreg);
    /*---------------------------- USART CR3 Configuration -----------------------*/
    tmpreg = (*USARTx).CR3;
    /* Clear CTSE and RTSE bits */
    tmpreg &=
        !(0x100 as libc::c_int as uint32_t |
              0x200 as libc::c_int as uint32_t);
    /* Configure the USART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
    tmpreg |= (*USART_InitStruct).USART_HardwareFlowControl;
    /* Write to USART CR3 */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t, tmpreg);
    /*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate -------------------------------------------*/
    RCC_GetClocksFreq(&mut RCC_ClocksStatus);
    if USARTx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x3800
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut USART_TypeDef {
        apbclock = RCC_ClocksStatus.USART1CLK_Frequency
    } else if USARTx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x4400 as libc::c_int as
                                                  libc::c_uint) as
                      *mut USART_TypeDef {
        apbclock = RCC_ClocksStatus.USART2CLK_Frequency
    } else if USARTx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x4800 as libc::c_int as
                                                  libc::c_uint) as
                      *mut USART_TypeDef {
        apbclock = RCC_ClocksStatus.USART3CLK_Frequency
    } else if USARTx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x4c00 as libc::c_int as
                                                  libc::c_uint) as
                      *mut USART_TypeDef {
        apbclock = RCC_ClocksStatus.UART4CLK_Frequency
    } else { apbclock = RCC_ClocksStatus.UART5CLK_Frequency }
    /* Determine the integer part */
    if (*USARTx).CR1 & 0x8000 as libc::c_int as uint32_t !=
           0 as libc::c_int as libc::c_uint {
        /* (divider * 10) computing in case Oversampling mode is 8 Samples */
        divider =
            (2 as libc::c_int as
                 libc::c_uint).wrapping_mul(apbclock).wrapping_div((*USART_InitStruct).USART_BaudRate);
        tmpreg =
            (2 as libc::c_int as
                 libc::c_uint).wrapping_mul(apbclock).wrapping_rem((*USART_InitStruct).USART_BaudRate)
    } else {
        /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
        /* (divider * 10) computing in case Oversampling mode is 16 Samples */
        divider = apbclock.wrapping_div((*USART_InitStruct).USART_BaudRate);
        tmpreg = apbclock.wrapping_rem((*USART_InitStruct).USART_BaudRate)
    }
    /* round the divider : if fractional part i greater than 0.5 increment divider */
    if tmpreg >=
           (*USART_InitStruct).USART_BaudRate.wrapping_div(2 as libc::c_int as
                                                               libc::c_uint) {
        divider = divider.wrapping_add(1)
    }
    /* Implement the divider in case Oversampling mode is 8 Samples */
    if (*USARTx).CR1 & 0x8000 as libc::c_int as uint32_t !=
           0 as libc::c_int as libc::c_uint {
        /* get the LSB of divider and shift it to the right by 1 bit */
        tmpreg =
            (divider & 0xf as libc::c_int as uint16_t as libc::c_uint) >>
                1 as libc::c_int;
        /* update the divider value */
        divider =
            divider & 0xfff0 as libc::c_int as uint16_t as libc::c_uint |
                tmpreg
    }
    /* Write to USART BRR */
    ::core::ptr::write_volatile(&mut (*USARTx).BRR as *mut uint16_t,
                                divider as uint16_t);
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
    (*USART_InitStruct).USART_WordLength = 0 as libc::c_int as uint32_t;
    (*USART_InitStruct).USART_StopBits = 0 as libc::c_int as uint32_t;
    (*USART_InitStruct).USART_Parity = 0 as libc::c_int as uint32_t;
    (*USART_InitStruct).USART_Mode =
        0x4 as libc::c_int as uint32_t | 0x8 as libc::c_int as uint32_t;
    (*USART_InitStruct).USART_HardwareFlowControl =
        0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Initializes the USARTx peripheral Clock according to the
  *         specified parameters in the USART_ClockInitStruct.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the
  *         following values: USART1 or USART2 or USART3.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *         structure that contains the configuration information for the specified
  *         USART peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ClockInit(mut USARTx: *mut USART_TypeDef,
                                         mut USART_ClockInitStruct:
                                             *mut USART_ClockInitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*---------------------------- USART CR2 Configuration -----------------------*/
    tmpreg = (*USARTx).CR2;
    /* Clear CLKEN, CPOL, CPHA, LBCL and SSM bits */
    tmpreg &=
        !(0x800 as libc::c_int as uint32_t | 0x400 as libc::c_int as uint32_t
              | 0x200 as libc::c_int as uint32_t |
              0x100 as libc::c_int as uint32_t);
    /* Configure the USART Clock, CPOL, CPHA, LastBit and SSM ------------*/
  /* Set CLKEN bit according to USART_Clock value */
  /* Set CPOL bit according to USART_CPOL value */
  /* Set CPHA bit according to USART_CPHA value */
  /* Set LBCL bit according to USART_LastBit value */
    tmpreg |=
        (*USART_ClockInitStruct).USART_Clock |
            (*USART_ClockInitStruct).USART_CPOL |
            (*USART_ClockInitStruct).USART_CPHA |
            (*USART_ClockInitStruct).USART_LastBit;
    /* Write to USART CR2 */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t, tmpreg);
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
    (*USART_ClockInitStruct).USART_Clock = 0 as libc::c_int as uint32_t;
    (*USART_ClockInitStruct).USART_CPOL = 0 as libc::c_int as uint32_t;
    (*USART_ClockInitStruct).USART_CPHA = 0 as libc::c_int as uint32_t;
    (*USART_ClockInitStruct).USART_LastBit = 0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Enables or disables the specified USART peripheral.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
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
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected USART by clearing the UE bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the USART's transmitter or receiver.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_Direction: specifies the USART direction.
  *         This parameter can be any combination of the following values:
  *         @arg USART_Mode_Tx: USART Transmitter
  *         @arg USART_Mode_Rx: USART Receiver
  * @param  NewState: new state of the USART transfer direction.
  *         This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_DirectionModeCmd(mut USARTx:
                                                    *mut USART_TypeDef,
                                                mut USART_DirectionMode:
                                                    uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the USART's transfer interface by setting the TE and/or RE bits 
       in the USART CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         USART_DirectionMode) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the USART's transfer interface by clearing the TE and/or RE bits
       in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !USART_DirectionMode) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the USART's 8x oversampling mode.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USART 8x oversampling mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @note
  *   This function has to be called before calling USART_Init()
  *   function in order to have correct baudrate Divider value.
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
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the 8x Oversampling mode by clearing the OVER8 bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the USART's one bit sampling method.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USART one bit sampling method.
  *         This parameter can be: ENABLE or DISABLE.
  * @note
  *   This function has to be called before calling USART_Cmd() function.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_OneBitMethodCmd(mut USARTx: *mut USART_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the one bit method by setting the ONEBIT bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the one bit method by clearing the ONEBIT bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x800 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the USART's most significant bit first 
  *         transmitted/received following the start bit.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USART most significant bit first
  *         transmitted/received following the start bit.
  *         This parameter can be: ENABLE or DISABLE.
  * @note
  *   This function has to be called before calling USART_Cmd() function. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_MSBFirstCmd(mut USARTx: *mut USART_TypeDef,
                                           mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the most significant bit first transmitted/received following the
       start bit by setting the MSBFIRST bit in the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x80000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the most significant bit first transmitted/received following the
       start bit by clearing the MSBFIRST bit in the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x80000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the binary data inversion.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new defined levels for the USART data.
  *         This parameter can be: ENABLE or DISABLE.
  *   @arg  ENABLE: Logical data from the data register are send/received in negative
  *         logic. (1=L, 0=H). The parity bit is also inverted.
  *   @arg  DISABLE: Logical data from the data register are send/received in positive
  *         logic. (1=H, 0=L) 
  * @note
  *   This function has to be called before calling USART_Cmd() function. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_DataInvCmd(mut USARTx: *mut USART_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the binary data inversion feature by setting the DATAINV bit in
       the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x40000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the binary data inversion feature by clearing the DATAINV bit in
       the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x40000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Pin(s) active level inversion.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_InvPin: specifies the USART pin(s) to invert.
  *         This parameter can be any combination of the following values:
  *         @arg USART_InvPin_Tx: USART Tx pin active level inversion.
  *         @arg USART_InvPin_Rx: USART Rx pin active level inversion.
  * @param  NewState: new active level status for the USART pin(s).
  *         This parameter can be: ENABLE or DISABLE.
  *          - ENABLE: pin(s) signal values are inverted (Vdd =0, Gnd =1).
  *          - DISABLE: pin(s) signal works using the standard logic levels (Vdd =1, Gnd =0).
  * @note
  *   This function has to be called before calling USART_Cmd() function.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_InvPinCmd(mut USARTx: *mut USART_TypeDef,
                                         mut USART_InvPin: uint32_t,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the active level inversion for selected pins by setting the TXINV 
       and/or RXINV bits in the USART CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | USART_InvPin) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the active level inversion for selected requests by clearing the 
       TXINV and/or RXINV bits in the USART CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !USART_InvPin) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the swap Tx/Rx pins.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USARTx TX/RX pins pinout.
  *         This parameter can be: ENABLE or DISABLE.
  *         @arg ENABLE: The TX and RX pins functions are swapped.
  *         @arg DISABLE: TX/RX pins are used as defined in standard pinout
  * @note
  *   This function has to be called before calling USART_Cmd() function.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SWAPPinCmd(mut USARTx: *mut USART_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the SWAP feature by setting the SWAP bit in the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the SWAP feature by clearing the SWAP bit in the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the receiver Time Out feature.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USARTx receiver Time Out.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ReceiverTimeOutCmd(mut USARTx:
                                                      *mut USART_TypeDef,
                                                  mut NewState:
                                                      FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the receiver time out feature by setting the RTOEN bit in the CR2 
       register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the receiver time out feature by clearing the RTOEN bit in the CR2 
       register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x800000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Sets the receiver Time Out value.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_ReceiverTimeOut: specifies the Receiver Time Out value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetReceiverTimeOut(mut USARTx:
                                                      *mut USART_TypeDef,
                                                  mut USART_ReceiverTimeOut:
                                                      uint32_t) {
    /* Check the parameters */
    /* Clear the receiver Time Out value by clearing the RTO[23:0] bits in the RTOR
     register  */
    ::core::ptr::write_volatile(&mut (*USARTx).RTOR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).RTOR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xffffff as libc::c_int as uint32_t))
                                    as uint32_t as uint32_t);
    /* Set the receiver Time Out value by setting the RTO[23:0] bits in the RTOR
     register  */
    ::core::ptr::write_volatile(&mut (*USARTx).RTOR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).RTOR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_ReceiverTimeOut)
                                    as uint32_t as uint32_t);
}
/* *
  * @brief  Sets the system clock prescaler.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_Prescaler: specifies the prescaler clock.  
  * @note
  *   This function has to be called before calling USART_Cmd() function.  
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
/* * @defgroup USART_Group2 STOP Mode functions
 *  @brief   STOP Mode functions
 *
@verbatim
 ===============================================================================
                        ##### STOP Mode functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage 
         WakeUp from STOP mode.

    [..] The USART is able to WakeUp from Stop Mode if USART clock is set to HSI
         or LSI.
         
    [..] The WakeUp source is configured by calling USART_StopModeWakeUpSourceConfig()
         function.
         
    [..] After configuring the source of WakeUp and before entering in Stop Mode 
         USART_STOPModeCmd() function should be called to allow USART WakeUp.
                           
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified USART peripheral in STOP Mode.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USARTx peripheral state in stop mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @note
  *   This function has to be called when USART clock is set to HSI or LSE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_STOPModeCmd(mut USARTx: *mut USART_TypeDef,
                                           mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected USART in STOP mode by setting the UESM bit in the CR1
       register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected USART in STOP mode by clearing the UE bit in the CR1
       register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Selects the USART WakeUp method form stop mode.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_WakeUp: specifies the selected USART wakeup method.
  *         This parameter can be one of the following values:
  *         @arg USART_WakeUpSource_AddressMatch: WUF active on address match.
  *         @arg USART_WakeUpSource_StartBit: WUF active on Start bit detection.
  *         @arg USART_WakeUpSource_RXNE: WUF active on RXNE.
  * @note
  *   This function has to be called before calling USART_Cmd() function.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_StopModeWakeUpSourceConfig(mut USARTx:
                                                              *mut USART_TypeDef,
                                                          mut USART_WakeUpSource:
                                                              uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x300000 as libc::c_int as uint32_t))
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_WakeUpSource) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup USART_Group3 AutoBaudRate functions
 *  @brief   AutoBaudRate functions 
 *
@verbatim
 ===============================================================================
                       ##### AutoBaudRate functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage 
         the AutoBaudRate detections.
         
    [..] Before Enabling AutoBaudRate detection using USART_AutoBaudRateCmd ()
         The character patterns used to calculate baudrate must be chosen by calling 
         USART_AutoBaudRateConfig() function. These function take as parameter :
        (#)USART_AutoBaudRate_StartBit : any character starting with a bit 1.
        (#)USART_AutoBaudRate_FallingEdge : any character starting with a 10xx bit pattern. 
                          
    [..] At any later time, another request for AutoBaudRate detection can be performed
         using USART_RequestCmd() function.
         
    [..] The AutoBaudRate detection is monitored by the status of ABRF flag which indicate
         that the AutoBaudRate detection is completed. In addition to ABRF flag, the ABRE flag
         indicate that this procedure is completed without success. USART_GetFlagStatus () 
         function should be used to monitor the status of these flags.  
             
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the Auto Baud Rate.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USARTx auto baud rate.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_AutoBaudRateCmd(mut USARTx: *mut USART_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the auto baud rate feature by setting the ABREN bit in the CR2 
       register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x100000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the auto baud rate feature by clearing the ABREN bit in the CR2 
       register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x100000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Selects the USART auto baud rate method.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_AutoBaudRate: specifies the selected USART auto baud rate method.
  *         This parameter can be one of the following values:
  *         @arg USART_AutoBaudRate_StartBit: Start Bit duration measurement.
  *         @arg USART_AutoBaudRate_FallingEdge: Falling edge to falling edge measurement.
  *         @arg USART_AutoBaudRate_0x7FFrame: 0x7F frame.
  *         @arg USART_AutoBaudRate_0x55Frame: 0x55 frame.
  * @note
  *   This function has to be called before calling USART_Cmd() function. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_AutoBaudRateConfig(mut USARTx:
                                                      *mut USART_TypeDef,
                                                  mut USART_AutoBaudRate:
                                                      uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x600000 as libc::c_int as uint32_t))
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_AutoBaudRate) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup USART_Group4 Data transfers functions
 *  @brief   Data transfers functions 
 *
@verbatim
 ===============================================================================
                    ##### Data transfers functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage 
         the USART data transfers.
    [..] During an USART reception, data shifts in least significant bit first 
         through the RX pin. When a transmission is taking place, a write instruction to 
         the USART_TDR register stores the data in the shift register.
    [..] The read access of the USART_RDR register can be done using 
         the USART_ReceiveData() function and returns the RDR value.
         Whereas a write access to the USART_TDR can be done using USART_SendData()
         function and stores the written data into TDR.

@endverbatim
  * @{
  */
/* *
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  Data: the data to transmit.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SendData(mut USARTx: *mut USART_TypeDef,
                                        mut Data: uint16_t) {
    /* Check the parameters */
    /* Transmit Data */
    ::core::ptr::write_volatile(&mut (*USARTx).TDR as *mut uint16_t,
                                (Data as libc::c_int &
                                     0x1ff as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t);
}
/* *
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @retval The received data.
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ReceiveData(mut USARTx: *mut USART_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    /* Receive Data */
    return ((*USARTx).RDR as libc::c_int &
                0x1ff as libc::c_int as uint16_t as libc::c_int) as uint16_t;
}
/* *
  * @}
  */
/* * @defgroup USART_Group5 MultiProcessor Communication functions
 *  @brief   Multi-Processor Communication functions 
 *
@verbatim   
 ===============================================================================
             ##### Multi-Processor Communication functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the USART
         multiprocessor communication.
    [..] For instance one of the USARTs can be the master, its TX output is
         connected to the RX input of the other USART. The others are slaves,
         their respective TX outputs are logically ANDed together and connected 
         to the RX input of the master. USART multiprocessor communication is 
         possible through the following procedure:
         (#) Program the Baud rate, Word length = 9 bits, Stop bits, Parity, 
             Mode transmitter or Mode receiver and hardware flow control values 
             using the USART_Init() function.
         (#) Configures the USART address using the USART_SetAddress() function.
         (#) Configures the wake up methode (USART_WakeUp_IdleLine or 
             USART_WakeUp_AddressMark) using USART_WakeUpConfig() function only 
             for the slaves.
         (#) Enable the USART using the USART_Cmd() function.
         (#) Enter the USART slaves in mute mode using USART_ReceiverWakeUpCmd() 
             function.
    [..] The USART Slave exit from mute mode when receive the wake up condition.

@endverbatim
  * @{
  */
/* *
  * @brief  Sets the address of the USART node.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_Address: Indicates the address of the USART node.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetAddress(mut USARTx: *mut USART_TypeDef,
                                          mut USART_Address: uint8_t) {
    /* Check the parameters */
    /* Clear the USART address */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xff000000 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    /* Set the USART address node */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (USART_Address as uint32_t) <<
                                         0x18 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Enables or disables the USART's mute mode.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USART mute mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_MuteModeCmd(mut USARTx: *mut USART_TypeDef,
                                           mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the USART mute mode by setting the MME bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the USART mute mode by clearing the MME bit in the CR1 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Selects the USART WakeUp method from mute mode.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_WakeUp: specifies the USART wakeup method.
  *         This parameter can be one of the following values:
  *         @arg USART_WakeUp_IdleLine: WakeUp by an idle line detection
  *         @arg USART_WakeUp_AddressMark: WakeUp by an address mark
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_MuteModeWakeUpConfig(mut USARTx:
                                                        *mut USART_TypeDef,
                                                    mut USART_WakeUp:
                                                        uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x800 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_WakeUp) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configure the the USART Address detection length.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_AddressLength: specifies the USART address length detection.
  *         This parameter can be one of the following values:
  *         @arg USART_AddressLength_4b: 4-bit address length detection 
  *         @arg USART_AddressLength_7b: 7-bit address length detection 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_AddressDetectionConfig(mut USARTx:
                                                          *mut USART_TypeDef,
                                                      mut USART_AddressLength:
                                                          uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x10 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_AddressLength) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup USART_Group6 LIN mode functions
 *  @brief   LIN mode functions 
 *
@verbatim   
 ===============================================================================
                       ##### LIN mode functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the USART 
         LIN Mode communication.
    [..] In LIN mode, 8-bit data format with 1 stop bit is required in accordance 
         with the LIN standard.
    [..] Only this LIN Feature is supported by the USART IP:
         (+) LIN Master Synchronous Break send capability and LIN slave break 
             detection capability :  13-bit break generation and 10/11 bit break 
             detection.
    [..] USART LIN Master transmitter communication is possible through the 
         following procedure:
         (#) Program the Baud rate, Word length = 8bits, Stop bits = 1bit, Parity, 
             Mode transmitter or Mode receiver and hardware flow control values 
             using the USART_Init() function.
         (#) Enable the LIN mode using the USART_LINCmd() function.
         (#) Enable the USART using the USART_Cmd() function.
         (#) Send the break character using USART_SendBreak() function.
    [..] USART LIN Master receiver communication is possible through the 
         following procedure:
         (#) Program the Baud rate, Word length = 8bits, Stop bits = 1bit, Parity, 
             Mode transmitter or Mode receiver and hardware flow control values 
             using the USART_Init() function.
         (#) Configures the break detection length 
             using the USART_LINBreakDetectLengthConfig() function.
         (#) Enable the LIN mode using the USART_LINCmd() function.
         (#) Enable the USART using the USART_Cmd() function.
         [..]
         (@) In LIN mode, the following bits must be kept cleared:
             (+@) CLKEN in the USART_CR2 register.
             (+@) STOP[1:0], SCEN, HDSEL and IREN in the USART_CR3 register.

@endverbatim
  * @{
  */
/* *
  * @brief  Sets the USART LIN Break detection length.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_LINBreakDetectLength: specifies the LIN break detection length.
  *         This parameter can be one of the following values:
  *         @arg USART_LINBreakDetectLength_10b: 10-bit break detection
  *         @arg USART_LINBreakDetectLength_11b: 11-bit break detection
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_LINBreakDetectLengthConfig(mut USARTx:
                                                              *mut USART_TypeDef,
                                                          mut USART_LINBreakDetectLength:
                                                              uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x20 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     USART_LINBreakDetectLength) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Enables or disables the USART's LIN mode.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USART LIN mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_LINCmd(mut USARTx: *mut USART_TypeDef,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the LIN mode by setting the LINEN bit in the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the LIN mode by clearing the LINEN bit in the CR2 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup USART_Group7 Halfduplex mode function
 *  @brief   Half-duplex mode function 
 *
@verbatim   
 ===============================================================================
                   ##### Half-duplex mode function #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the USART
         Half-duplex communication.
    [..] The USART can be configured to follow a single-wire half-duplex protocol 
         where the TX and RX lines are internally connected.
    [..] USART Half duplex communication is possible through the following procedure:
         (#) Program the Baud rate, Word length, Stop bits, Parity, Mode transmitter 
             or Mode receiver and hardware flow control values using the USART_Init()
            function.
         (#) Configures the USART address using the USART_SetAddress() function.
         (#) Enable the half duplex mode using USART_HalfDuplexCmd() function.
         (#) Enable the USART using the USART_Cmd() function.
         [..]
         (@) The RX pin is no longer used.
         (@) In Half-duplex mode the following bits must be kept cleared:
             (+@) LINEN and CLKEN bits in the USART_CR2 register.
             (+@) SCEN and IREN bits in the USART_CR3 register.

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the USART's Half Duplex communication.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the USART Communication.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_HalfDuplexCmd(mut USARTx: *mut USART_TypeDef,
                                             mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the Half-Duplex mode by setting the HDSEL bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the Half-Duplex mode by clearing the HDSEL bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup USART_Group8 Smartcard mode functions
 *  @brief   Smartcard mode functions 
 *
@verbatim   
 ===============================================================================
                     ##### Smartcard mode functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the USART
         Smartcard communication.
    [..] The Smartcard interface is designed to support asynchronous protocol 
         Smartcards as defined in the ISO 7816-3 standard. The USART can provide 
         a clock to the smartcard through the SCLK output. In smartcard mode, 
         SCLK is not associated to the communication but is simply derived from 
         the internal peripheral input clock through a 5-bit prescaler.
    [..] Smartcard communication is possible through the following procedure:
         (#) Configures the Smartcard Prsecaler using the USART_SetPrescaler() 
             function.
         (#) Configures the Smartcard Guard Time using the USART_SetGuardTime() 
             function.
         (#) Program the USART clock using the USART_ClockInit() function as following:
             (++) USART Clock enabled.
             (++) USART CPOL Low.
             (++) USART CPHA on first edge.
             (++) USART Last Bit Clock Enabled.
         (#) Program the Smartcard interface using the USART_Init() function as 
             following:
             (++) Word Length = 9 Bits.
             (++) 1.5 Stop Bit.
             (++) Even parity.
             (++) BaudRate = 12096 baud.
             (++) Hardware flow control disabled (RTS and CTS signals).
             (++) Tx and Rx enabled
         (#) Optionally you can enable the parity error interrupt using 
             the USART_ITConfig() function.
         (#) Enable the Smartcard NACK using the USART_SmartCardNACKCmd() function.
         (#) Enable the Smartcard interface using the USART_SmartCardCmd() function.
         (#) Enable the USART using the USART_Cmd() function.
    [..] 
  Please refer to the ISO 7816-3 specification for more details.
    [..] 
         (@) It is also possible to choose 0.5 stop bit for receiving but it is 
             recommended to use 1.5 stop bits for both transmitting and receiving 
             to avoid switching between the two configurations.
         (@) In smartcard mode, the following bits must be kept cleared:
             (+@) LINEN bit in the USART_CR2 register.
             (+@) HDSEL and IREN bits in the USART_CR3 register.

@endverbatim
  * @{
  */
/* *
  * @brief  Sets the specified USART guard time.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3.
  * @param  USART_GuardTime: specifies the guard time.
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
  * @brief  Enables or disables the USART's Smart Card mode.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3.
  * @param  NewState: new state of the Smart Card mode.
  *         This parameter can be: ENABLE or DISABLE.      
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SmartCardCmd(mut USARTx: *mut USART_TypeDef,
                                            mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the SC mode by setting the SCEN bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the SC mode by clearing the SCEN bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x20 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables NACK transmission.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3.
  * @param  NewState: new state of the NACK transmission.
  *         This parameter can be: ENABLE or DISABLE.  
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
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the NACK transmission by clearing the NACK bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x10 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Sets the Smart Card number of retries in transmit and receive.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3.
  * @param  USART_AutoCount: specifies the Smart Card auto retry count.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetAutoRetryCount(mut USARTx:
                                                     *mut USART_TypeDef,
                                                 mut USART_AutoCount:
                                                     uint8_t) {
    /* Check the parameters */
    /* Clear the USART auto retry count */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xe0000 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    /* Set the USART auto retry count*/
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (USART_AutoCount as uint32_t) <<
                                         0x11 as libc::c_int) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Sets the Smart Card Block length.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3.
  * @param  USART_BlockLength: specifies the Smart Card block length.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetBlockLength(mut USARTx: *mut USART_TypeDef,
                                              mut USART_BlockLength:
                                                  uint8_t) {
    /* Check the parameters */
    /* Clear the Smart card block length */
    ::core::ptr::write_volatile(&mut (*USARTx).RTOR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).RTOR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xff000000 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    /* Set the Smart Card block length */
    ::core::ptr::write_volatile(&mut (*USARTx).RTOR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).RTOR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (USART_BlockLength as uint32_t) <<
                                         0x18 as libc::c_int) as uint32_t as
                                    uint32_t);
}
/* *
  * @}
  */
/* * @defgroup USART_Group9 IrDA mode functions
 *  @brief   IrDA mode functions 
 *
@verbatim   
 ===============================================================================
                        ##### IrDA mode functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the USART
         IrDA communication.
    [..] IrDA is a half duplex communication protocol. If the Transmitter is busy, 
         any data on the IrDA receive line will be ignored by the IrDA decoder 
         and if the Receiver is busy, data on the TX from the USART to IrDA will 
         not be encoded by IrDA. While receiving data, transmission should be 
         avoided as the data to be transmitted could be corrupted.
    [..] IrDA communication is possible through the following procedure:
         (#) Program the Baud rate, Word length = 8 bits, Stop bits, Parity, 
             Transmitter/Receiver modes and hardware flow control values using 
             the USART_Init() function.
         (#) Configures the IrDA pulse width by configuring the prescaler using  
             the USART_SetPrescaler() function.
         (#) Configures the IrDA  USART_IrDAMode_LowPower or USART_IrDAMode_Normal 
             mode using the USART_IrDAConfig() function.
         (#) Enable the IrDA using the USART_IrDACmd() function.
         (#) Enable the USART using the USART_Cmd() function.         
    [..]
    (@) A pulse of width less than two and greater than one PSC period(s) may or 
        may not be rejected.
    (@) The receiver set up time should be managed by software. The IrDA physical 
        layer specification specifies a minimum of 10 ms delay between 
        transmission and reception (IrDA is a half duplex protocol).
    (@) In IrDA mode, the following bits must be kept cleared:
        (+@) LINEN, STOP and CLKEN bits in the USART_CR2 register.
        (+@) SCEN and HDSEL bits in the USART_CR3 register.

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the USART's IrDA interface.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_IrDAMode: specifies the IrDA mode.
  *         This parameter can be one of the following values:
  *         @arg USART_IrDAMode_LowPower
  *         @arg USART_IrDAMode_Normal
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_IrDAConfig(mut USARTx: *mut USART_TypeDef,
                                          mut USART_IrDAMode: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x4 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_IrDAMode) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Enables or disables the USART's IrDA interface. 
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the IrDA mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_IrDACmd(mut USARTx: *mut USART_TypeDef,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the IrDA mode by setting the IREN bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the IrDA mode by clearing the IREN bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup USART_Group10 RS485 mode function
 *  @brief   RS485 mode function 
 *
@verbatim  
 ===============================================================================
                        ##### RS485 mode functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to manage the USART
         RS485 flow control.
    [..] RS485 flow control (Driver enable feature) handling is possible through
         the following procedure:
         (#) Program the Baud rate, Word length = 8 bits, Stop bits, Parity, 
             Transmitter/Receiver modes and hardware flow control values using 
             the USART_Init() function.
         (#) Enable the Driver Enable using the USART_DECmd() function.
         (#) Configures the Driver Enable polarity using the USART_DEPolarityConfig()
             function.
         (#) Configures the Driver Enable assertion time using USART_SetDEAssertionTime() 
             function and deassertion time using the USART_SetDEDeassertionTime()
             function.    
         (#) Enable the USART using the USART_Cmd() function.
      [..]  
       (@) The assertion and dessertion times are expressed in sample time units (1/8 or 
            1/16 bit time, depending on the oversampling rate).
       
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the USART's DE functionality.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  NewState: new state of the driver enable mode.
  *         This parameter can be: ENABLE or DISABLE.      
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_DECmd(mut USARTx: *mut USART_TypeDef,
                                     mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the DE functionality by setting the DEM bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the DE functionality by clearing the DEM bit in the CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configures the USART's DE polarity
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_DEPolarity: specifies the DE polarity.
  *         This parameter can be one of the following values:
  *         @arg USART_DEPolarity_Low
  *         @arg USART_DEPolarity_High
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_DEPolarityConfig(mut USARTx:
                                                    *mut USART_TypeDef,
                                                mut USART_DEPolarity:
                                                    uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x8000 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_DEPolarity) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Sets the specified RS485 DE assertion time
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_AssertionTime: specifies the time between the activation of the DE
  *          signal and the beginning of the start bit
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetDEAssertionTime(mut USARTx:
                                                      *mut USART_TypeDef,
                                                  mut USART_DEAssertionTime:
                                                      uint32_t) {
    /* Check the parameters */
    /* Clear the DE assertion time */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x3e00000 as libc::c_int as uint32_t))
                                    as uint32_t as uint32_t);
    /* Set the new value for the DE assertion time */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     USART_DEAssertionTime <<
                                         0x15 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Sets the specified RS485 DE deassertion time
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_DeassertionTime: specifies the time between the middle of the last 
  *         stop bit in a transmitted message and the de-activation of the DE signal
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_SetDEDeassertionTime(mut USARTx:
                                                        *mut USART_TypeDef,
                                                    mut USART_DEDeassertionTime:
                                                        uint32_t) {
    /* Check the parameters */
    /* Clear the DE deassertion time */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1f0000 as libc::c_int as uint32_t))
                                    as uint32_t as uint32_t);
    /* Set the new value for the DE deassertion time */
    ::core::ptr::write_volatile(&mut (*USARTx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     USART_DEDeassertionTime <<
                                         0x10 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup USART_Group11 DMA transfers management functions
 *  @brief   DMA transfers management functions
 *
@verbatim   
 ===============================================================================
               ##### DMA transfers management functions #####
 ===============================================================================
    [..] This section provides two functions that can be used only in DMA mode.
    [..] In DMA Mode, the USART communication can be managed by 2 DMA Channel 
         requests:
         (#) USART_DMAReq_Tx: specifies the Tx buffer DMA transfer request.
         (#) USART_DMAReq_Rx: specifies the Rx buffer DMA transfer request.
    [..] In this Mode it is advised to use the following function:
         (+) void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, 
             FunctionalState NewState).
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the USART's DMA interface.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4.
  * @param  USART_DMAReq: specifies the DMA request.
  *         This parameter can be any combination of the following values:
  *         @arg USART_DMAReq_Tx: USART DMA transmit request
  *         @arg USART_DMAReq_Rx: USART DMA receive request
  * @param  NewState: new state of the DMA Request sources.
  *         This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_DMACmd(mut USARTx: *mut USART_TypeDef,
                                      mut USART_DMAReq: uint32_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the DMA transfer for selected requests by setting the DMAT and/or
       DMAR bits in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | USART_DMAReq) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the DMA transfer for selected requests by clearing the DMAT and/or
       DMAR bits in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !USART_DMAReq) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the USART's DMA interface when reception error occurs.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4.
  * @param  USART_DMAOnError: specifies the DMA status in case of reception error.
  *         This parameter can be any combination of the following values:
  *         @arg USART_DMAOnError_Enable: DMA receive request enabled when the USART DMA  
  *          reception error is asserted.
  *         @arg USART_DMAOnError_Disable: DMA receive request disabled when the USART DMA 
  *          reception error is asserted.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_DMAReceptionErrorConfig(mut USARTx:
                                                           *mut USART_TypeDef,
                                                       mut USART_DMAOnError:
                                                           uint32_t) {
    /* Check the parameters */
    /* Clear the DMA Reception error detection bit */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x2000 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    /* Set the new value for the DMA Reception error detection bit */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_DMAOnError) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup USART_Group12 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions 
 *
@verbatim   
 ===============================================================================
            ##### Interrupts and flags management functions #####
 ===============================================================================
    [..] This subsection provides a set of functions allowing to configure the 
         USART Interrupts sources, Requests and check or clear the flags or pending bits status. 
         The user should identify which mode will be used in his application to 
         manage the communication: Polling mode, Interrupt mode.
         
 *** Polling Mode ***
 ====================
    [..] In Polling Mode, the SPI communication can be managed by these flags:
         (#) USART_FLAG_REACK: to indicate the status of the Receive Enable 
             acknowledge flag
         (#) USART_FLAG_TEACK: to indicate the status of the Transmit Enable 
             acknowledge flag.
         (#) USART_FLAG_WUF: to indicate the status of the Wake up flag.
         (#) USART_FLAG_RWU: to indicate the status of the Receive Wake up flag.
         (#) USART_FLAG_SBK: to indicate the status of the Send Break flag.
         (#) USART_FLAG_CMF: to indicate the status of the Character match flag.
         (#) USART_FLAG_BUSY: to indicate the status of the Busy flag.
         (#) USART_FLAG_ABRF: to indicate the status of the Auto baud rate flag.
         (#) USART_FLAG_ABRE: to indicate the status of the Auto baud rate error flag.
         (#) USART_FLAG_EOBF: to indicate the status of the End of block flag.
         (#) USART_FLAG_RTOF: to indicate the status of the Receive time out flag.
         (#) USART_FLAG_nCTSS: to indicate the status of the Inverted nCTS input 
             bit status.
         (#) USART_FLAG_TXE: to indicate the status of the transmit buffer register.
         (#) USART_FLAG_RXNE: to indicate the status of the receive buffer register.
         (#) USART_FLAG_TC: to indicate the status of the transmit operation.
         (#) USART_FLAG_IDLE: to indicate the status of the Idle Line.
         (#) USART_FLAG_CTS: to indicate the status of the nCTS input.
         (#) USART_FLAG_LBD: to indicate the status of the LIN break detection.
         (#) USART_FLAG_NE: to indicate if a noise error occur.
         (#) USART_FLAG_FE: to indicate if a frame error occur.
         (#) USART_FLAG_PE: to indicate if a parity error occur.
         (#) USART_FLAG_ORE: to indicate if an Overrun error occur.
    [..] In this Mode it is advised to use the following functions:
         (+) FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG).
         (+) void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG).
               
 *** Interrupt Mode ***
 ======================
    [..] In Interrupt Mode, the USART communication can be managed by 8 interrupt 
         sources and 10 pending bits:
         (+) Pending Bits:
             (##) USART_IT_WU: to indicate the status of the Wake up interrupt.
             (##) USART_IT_CM: to indicate the status of Character match interrupt.
             (##) USART_IT_EOB: to indicate the status of End of block interrupt.
             (##) USART_IT_RTO: to indicate the status of Receive time out interrupt.
             (##) USART_IT_CTS: to indicate the status of CTS change interrupt.
             (##) USART_IT_LBD: to indicate the status of LIN Break detection interrupt.
             (##) USART_IT_TC: to indicate the status of Transmission complete interrupt.
             (##) USART_IT_IDLE: to indicate the status of IDLE line detected interrupt.
             (##) USART_IT_ORE: to indicate the status of OverRun Error interrupt.
             (##) USART_IT_NE: to indicate the status of Noise Error interrupt.
             (##) USART_IT_FE: to indicate the status of Framing Error interrupt.
             (##) USART_IT_PE: to indicate the status of Parity Error interrupt.  
              
         (+) Interrupt Source:
             (##) USART_IT_WU: specifies the interrupt source for Wake up interrupt.
             (##) USART_IT_CM: specifies the interrupt source for Character match 
                  interrupt.
             (##) USART_IT_EOB: specifies the interrupt source for End of block
                  interrupt.
             (##) USART_IT_RTO: specifies the interrupt source for Receive time-out
                  interrupt.
             (##) USART_IT_CTS: specifies the interrupt source for CTS change interrupt.
             (##) USART_IT_LBD: specifies the interrupt source for LIN Break 
                  detection interrupt.
             (##) USART_IT_TXE: specifies the interrupt source for Tansmit Data 
                  Register empty interrupt.
             (##) USART_IT_TC: specifies the interrupt source for Transmission 
                  complete interrupt.
             (##) USART_IT_RXNE: specifies the interrupt source for Receive Data 
                  register not empty interrupt.
             (##) USART_IT_IDLE: specifies the interrupt source for Idle line 
                  detection interrupt.
             (##) USART_IT_PE: specifies the interrupt source for Parity Error interrupt.
             (##) USART_IT_ERR: specifies the interrupt source for Error interrupt
                  (Frame error, noise error, overrun error)
             -@@- Some parameters are coded in order to use them as interrupt 
                 source or as pending bits.
    [..] In this Mode it is advised to use the following functions:
         (+) void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState).
         (+) ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT).
         (+) void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT).

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified USART interrupts.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt sources to be enabled or disabled.
  *         This parameter can be one of the following values:
  *         @arg USART_IT_WU:  Wake up interrupt.
  *         @arg USART_IT_CM:  Character match interrupt.
  *         @arg USART_IT_EOB:  End of block interrupt.
  *         @arg USART_IT_RTO:  Receive time out interrupt.
  *         @arg USART_IT_CTS:  CTS change interrupt.
  *         @arg USART_IT_LBD:  LIN Break detection interrupt.
  *         @arg USART_IT_TXE:  Tansmit Data Register empty interrupt.
  *         @arg USART_IT_TC:  Transmission complete interrupt.
  *         @arg USART_IT_RXNE:  Receive Data register not empty interrupt.
  *         @arg USART_IT_IDLE:  Idle line detection interrupt.
  *         @arg USART_IT_PE:  Parity Error interrupt.
  *         @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @param  NewState: new state of the specified USARTx interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ITConfig(mut USARTx: *mut USART_TypeDef,
                                        mut USART_IT: uint32_t,
                                        mut NewState: FunctionalState) {
    let mut usartreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itpos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itmask: uint32_t = 0 as libc::c_int as uint32_t;
    let mut usartxbase: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    usartxbase = USARTx as uint32_t;
    /* Get the USART register index */
    usartreg =
        (USART_IT as uint16_t as libc::c_int >> 0x8 as libc::c_int) as
            uint32_t;
    /* Get the interrupt position */
    itpos = USART_IT & 0xff as libc::c_int as uint32_t;
    itmask = (0x1 as libc::c_int as uint32_t) << itpos;
    if usartreg == 0x2 as libc::c_int as libc::c_uint {
        /* The IT is in CR2 register */
        usartxbase =
            (usartxbase as
                 libc::c_uint).wrapping_add(0x4 as libc::c_int as
                                                libc::c_uint) as uint32_t as
                uint32_t
    } else if usartreg == 0x3 as libc::c_int as libc::c_uint {
        /* The IT is in CR3 register */
        usartxbase =
            (usartxbase as
                 libc::c_uint).wrapping_add(0x8 as libc::c_int as
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
  * @brief  Enables the specified USART's Request.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_Request: specifies the USART request.
  *         This parameter can be any combination of the following values:
  *         @arg USART_Request_TXFRQ: Transmit data flush ReQuest
  *         @arg USART_Request_RXFRQ: Receive data flush ReQuest
  *         @arg USART_Request_MMRQ: Mute Mode ReQuest
  *         @arg USART_Request_SBKRQ: Send Break ReQuest
  *         @arg USART_Request_ABRRQ: Auto Baud Rate ReQuest
  * @param  NewState: new state of the DMA interface when reception error occurs.
  *         This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_RequestCmd(mut USARTx: *mut USART_TypeDef,
                                          mut USART_Request: uint32_t,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the USART ReQuest by setting the dedicated request bit in the RQR
       register.*/
        ::core::ptr::write_volatile(&mut (*USARTx).RQR as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).RQR
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_uint | USART_Request) as
                                        uint16_t as uint16_t)
    } else {
        /* Disable the USART ReQuest by clearing the dedicated request bit in the RQR
       register.*/
        ::core::ptr::write_volatile(&mut (*USARTx).RQR as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*USARTx).RQR
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_uint & !USART_Request) as
                                        uint16_t as uint16_t)
    };
}
/* *
  * @brief  Enables or disables the USART's Overrun detection.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_OVRDetection: specifies the OVR detection status in case of OVR error.
  *         This parameter can be any combination of the following values:
  *         @arg USART_OVRDetection_Enable: OVR error detection enabled when the USART OVR error 
  *          is asserted.
  *         @arg USART_OVRDetection_Disable: OVR error detection disabled when the USART OVR error 
  *          is asserted.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_OverrunDetectionConfig(mut USARTx:
                                                          *mut USART_TypeDef,
                                                      mut USART_OVRDetection:
                                                          uint32_t) {
    /* Check the parameters */
    /* Clear the OVR detection bit */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1000 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    /* Set the new value for the OVR detection bit */
    ::core::ptr::write_volatile(&mut (*USARTx).CR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USARTx).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | USART_OVRDetection) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Checks whether the specified USART flag is set or not.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to check.
  *         This parameter can be one of the following values:
  *         @arg USART_FLAG_REACK:  Receive Enable acknowledge flag.
  *         @arg USART_FLAG_TEACK:  Transmit Enable acknowledge flag.
  *         @arg USART_FLAG_WUF:  Wake up flag.
  *         @arg USART_FLAG_RWU:  Receive Wake up flag.
  *         @arg USART_FLAG_SBK:  Send Break flag.
  *         @arg USART_FLAG_CMF:  Character match flag.
  *         @arg USART_FLAG_BUSY:  Busy flag.
  *         @arg USART_FLAG_ABRF:  Auto baud rate flag.
  *         @arg USART_FLAG_ABRE:  Auto baud rate error flag.
  *         @arg USART_FLAG_EOBF:  End of block flag.
  *         @arg USART_FLAG_RTOF:  Receive time out flag.
  *         @arg USART_FLAG_nCTSS:  Inverted nCTS input bit status.
  *         @arg USART_FLAG_CTS:  CTS Change flag.
  *         @arg USART_FLAG_LBD:  LIN Break detection flag.
  *         @arg USART_FLAG_TXE:  Transmit data register empty flag.
  *         @arg USART_FLAG_TC:  Transmission Complete flag.
  *         @arg USART_FLAG_RXNE:  Receive data register not empty flag.
  *         @arg USART_FLAG_IDLE:  Idle Line detection flag.
  *         @arg USART_FLAG_ORE:  OverRun Error flag.
  *         @arg USART_FLAG_NE:  Noise Error flag.
  *         @arg USART_FLAG_FE:  Framing Error flag.
  *         @arg USART_FLAG_PE:  Parity Error flag.
  * @retval The new state of USART_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn USART_GetFlagStatus(mut USARTx: *mut USART_TypeDef,
                                             mut USART_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if (*USARTx).ISR & USART_FLAG !=
           RESET as libc::c_int as uint16_t as libc::c_uint {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the USARTx's pending flags.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to clear.
  *         This parameter can be any combination of the following values:
  *         @arg USART_FLAG_WUF:  Wake up flag.
  *         @arg USART_FLAG_CMF:  Character match flag.
  *         @arg USART_FLAG_EOBF:  End of block flag.
  *         @arg USART_FLAG_RTOF:  Receive time out flag.
  *         @arg USART_FLAG_CTS:  CTS Change flag.
  *         @arg USART_FLAG_LBD:  LIN Break detection flag.
  *         @arg USART_FLAG_TC:  Transmission Complete flag.
  *         @arg USART_FLAG_IDLE:  IDLE line detected flag.
  *         @arg USART_FLAG_ORE:  OverRun Error flag.
  *         @arg USART_FLAG_NE: Noise Error flag.
  *         @arg USART_FLAG_FE: Framing Error flag.
  *         @arg USART_FLAG_PE:   Parity Errorflag.
  *
  * @note
  *   - RXNE pending bit is cleared by a read to the USART_RDR register 
  *     (USART_ReceiveData()) or by writing 1 to the RXFRQ in the register USART_RQR
  *     (USART_RequestCmd()).
  *   - TC flag can be also cleared by software sequence: a read operation to 
  *     USART_SR register (USART_GetFlagStatus()) followed by a write operation
  *     to USART_TDR register (USART_SendData()).
  *   - TXE flag is cleared by a write to the USART_TDR register 
  *     (USART_SendData()) or by writing 1 to the TXFRQ in the register USART_RQR
  *     (USART_RequestCmd()).
  *   - SBKF flag is cleared by 1 to the SBKRQ in the register USART_RQR
  *     (USART_RequestCmd()).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ClearFlag(mut USARTx: *mut USART_TypeDef,
                                         mut USART_FLAG: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*USARTx).ICR as *mut uint32_t,
                                USART_FLAG);
}
/* *
  * @brief  Checks whether the specified USART interrupt has occurred or not.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt source to check.
  *         This parameter can be one of the following values:
  *         @arg USART_IT_WU:  Wake up interrupt.
  *         @arg USART_IT_CM:  Character match interrupt.
  *         @arg USART_IT_EOB:  End of block interrupt.
  *         @arg USART_IT_RTO:  Receive time out interrupt.
  *         @arg USART_IT_CTS:  CTS change interrupt.
  *         @arg USART_IT_LBD:  LIN Break detection interrupt.
  *         @arg USART_IT_TXE:  Tansmit Data Register empty interrupt.
  *         @arg USART_IT_TC:  Transmission complete interrupt.
  *         @arg USART_IT_RXNE:  Receive Data register not empty interrupt.
  *         @arg USART_IT_IDLE:  Idle line detection interrupt.
  *         @arg USART_IT_ORE:  OverRun Error interrupt.
  *         @arg USART_IT_NE:  Noise Error interrupt.
  *         @arg USART_IT_FE:  Framing Error interrupt.
  *         @arg USART_IT_PE:  Parity Error interrupt.
  * @retval The new state of USART_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn USART_GetITStatus(mut USARTx: *mut USART_TypeDef,
                                           mut USART_IT: uint32_t)
 -> ITStatus {
    let mut bitpos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itmask: uint32_t = 0 as libc::c_int as uint32_t;
    let mut usartreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut bitstatus: ITStatus = RESET;
    /* Check the parameters */
    /* Get the USART register index */
    usartreg =
        (USART_IT as uint16_t as libc::c_int >> 0x8 as libc::c_int) as
            uint32_t;
    /* Get the interrupt position */
    itmask = USART_IT & 0xff as libc::c_int as uint32_t;
    itmask = (0x1 as libc::c_int as uint32_t) << itmask;
    if usartreg == 0x1 as libc::c_int as libc::c_uint {
        /* The IT  is in CR1 register */
        itmask &= (*USARTx).CR1
    } else if usartreg == 0x2 as libc::c_int as libc::c_uint {
        /* The IT  is in CR2 register */
        itmask &= (*USARTx).CR2
    } else {
        /* The IT  is in CR3 register */
        itmask &= (*USARTx).CR3
    }
    bitpos = USART_IT >> 0x10 as libc::c_int;
    bitpos = (0x1 as libc::c_int as uint32_t) << bitpos;
    bitpos &= (*USARTx).ISR;
    if itmask != RESET as libc::c_int as uint16_t as libc::c_uint &&
           bitpos != RESET as libc::c_int as uint16_t as libc::c_uint {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_usart.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the USART 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup USART
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  USART Init Structure definition  
  */
/* !< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 16) + 0.5 */
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
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control*/
/* * 
  * @brief  USART Clock Init Structure definition
  */
/* !< Specifies whether the USART clock is enabled or disabled.
                                         This parameter can be a value of @ref USART_Clock */
/* !< Specifies the steady state of the serial clock.
                                         This parameter can be a value of @ref USART_Clock_Polarity */
/* !< Specifies the clock transition on which the bit capture is made.
                                         This parameter can be a value of @ref USART_Clock_Phase */
/* !< Specifies whether the clock pulse corresponding to the last transmitted
                                         data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                                         This parameter can be a value of @ref USART_Last_Bit */
/* Exported constants --------------------------------------------------------*/
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
/* * @defgroup USART_DMA_Requests 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_DMA_Recception_Error
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_MuteMode_WakeUp_methods
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Address_Detection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_StopMode_WakeUp_methods 
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
/* * @defgroup USART_DE_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Inversion_Pins 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_AutoBaudRate_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_OVR_DETECTION
  * @{
  */
/* *
  * @}
  */ 
/* * @defgroup USART_Request 
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
/* * @defgroup USART_Interrupt_definition 
  * @brief USART Interrupt definition
  * USART_IT possible values
  * Elements values convention: 0xZZZZYYXX
  *   XX: Position of the corresponding Interrupt
  *   YY: Register index
  *   ZZZZ: Flag position
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Global_definition 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Initialization and Configuration functions *********************************/
/* STOP Mode functions ********************************************************/
/* AutoBaudRate functions *****************************************************/
/* Data transfers functions ***************************************************/
/* Multi-Processor Communication functions ************************************/
/* LIN mode functions *********************************************************/
/* Half-duplex mode function **************************************************/
/* Smartcard mode functions ***************************************************/
/* IrDA mode functions ********************************************************/
/* RS485 mode functions *******************************************************/
/* DMA transfers management functions *****************************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Clears the USARTx's interrupt pending bits.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_IT: specifies the interrupt pending bit to clear.
  *         This parameter can be one of the following values:
  *         @arg USART_IT_WU:  Wake up interrupt.
  *         @arg USART_IT_CM:  Character match interrupt.
  *         @arg USART_IT_EOB:  End of block interrupt.
  *         @arg USART_IT_RTO:  Receive time out interrupt.
  *         @arg USART_IT_CTS:  CTS change interrupt.
  *         @arg USART_IT_LBD:  LIN Break detection interrupt.
  *         @arg USART_IT_TC:  Transmission complete interrupt.
  *         @arg USART_IT_IDLE:  IDLE line detected interrupt.
  *         @arg USART_IT_ORE:  OverRun Error interrupt.
  *         @arg USART_IT_NE:  Noise Error interrupt.
  *         @arg USART_IT_FE:  Framing Error interrupt.
  *         @arg USART_IT_PE:  Parity Error interrupt.
  * @note
  *   - RXNE pending bit is cleared by a read to the USART_RDR register 
  *     (USART_ReceiveData()) or by writing 1 to the RXFRQ in the register USART_RQR
  *     (USART_RequestCmd()).
  *   - TC pending bit can be also cleared by software sequence: a read 
  *     operation to USART_SR register (USART_GetITStatus()) followed by a write 
  *     operation to USART_TDR register (USART_SendData()).
  *   - TXE pending bit is cleared by a write to the USART_TDR register 
  *     (USART_SendData()) or by writing 1 to the TXFRQ in the register USART_RQR
  *     (USART_RequestCmd()).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USART_ClearITPendingBit(mut USARTx:
                                                     *mut USART_TypeDef,
                                                 mut USART_IT: uint32_t) {
    let mut bitpos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itmask: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    bitpos = USART_IT >> 0x10 as libc::c_int;
    itmask = (0x1 as libc::c_int as uint32_t) << bitpos;
    ::core::ptr::write_volatile(&mut (*USARTx).ICR as *mut uint32_t, itmask);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
